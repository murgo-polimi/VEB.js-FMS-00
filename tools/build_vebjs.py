"""Build a VEB.js project (scene, environment, animation) of the FMS simulated by fmssim.

    python tools/build_vebjs.py                                   # reference.yaml, 0-2 h
    python tools/build_vebjs.py --config configs/reference_stochastic.yaml --name FMS_stochastic
    python tools/build_vebjs.py --from-h 8 --to-h 12 --seed 3 --time-scale 10

It reads the plant layout from the fmssim YAML configuration, runs the SimPy model
(package `fmssim` in the SymPy-FMS folder) and writes, in scenes/:

    <name>.json          scene: assets placed with the GLB models in models/
    <name>_env.json      cameras, lights, ground (loaded automatically by VEB.js)
    <name>_anim.json     animation: shuttle and pallet motions + resource states
    <name>_layout.json   the same scene without animation (static layout)
    <name>_layout_env.json

It also copies the GLB models from SymPy-FMS/3Dmodels into models/ and creates
one coloured pallet per fixture type (pallet_<type>.glb).

Geometry conventions are those of SymPy-FMS/3Dmodels/README.md (metres, +Y up,
X = rail direction, stations at z = -1.40 facing +Z, pallet bottom at 0.90 m).
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import struct
import sys
from bisect import bisect_right
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent

# ---------------------------------------------------------------- geometry ---
RAIL_TO_STATION = 1.40   # rail axis -> station origin (m)
TRANSFER_H = 0.90        # pallet bottom height at every transfer point (m)
APC_DX = 0.55            # APC receivers at x -/+ 0.55 from the machine origin
WORK_DZ = 1.75           # machine table behind the APC (towards -Z)
TICK_MS = 100            # VEB.js plays the animation in 100 ms ticks and fires only the
                         # events whose timestamp is a multiple of it
TRACK_MARGIN = 2.0       # track extends 2 m beyond the first/last position

FACTORY = "https://w3id.org/ontoeng/factory#"
IFC = "https://standards.buildingsmart.org/IFC/DEV/IFC4/ADD1/OWL#"

# marker colour (linear RGB) of each pallet fixture type
TYPE_COLOURS = [
    (0.02, 0.20, 0.80),   # blue
    (0.85, 0.08, 0.02),   # red
    (0.05, 0.55, 0.10),   # green
    (0.55, 0.05, 0.60),   # purple
    (0.95, 0.57, 0.004),  # yellow
]

MODEL_FILES = ["hmc_2apc.glb", "lu_station_manual.glb", "transporter_shuttle.glb",
               "pallet_base.glb"]


def r3(v):
    return [round(float(c), 4) for c in v]


# ------------------------------------------------------------ simulation ----
def run_model(fms_dir: Path, config: Path, seed, t_end: float, overrides: dict):
    sys.path.insert(0, str(fms_dir))
    from fmssim.config import load_config
    from fmssim.controller import Deadlock
    from fmssim.model import FMSModel

    cfg = load_config(config, overrides)
    seed = cfg.run.seed if seed is None else seed
    model = FMSModel(cfg, seed)
    deadlock = None
    try:
        model.run(until=t_end)
    except Deadlock as exc:  # keep what was simulated up to the deadlock
        deadlock = str(exc)
    return cfg, model, seed, deadlock


# ------------------------------------------------------------- kinematics ---
class ShuttleKinematics:
    """Shuttle x(t) from the logged moves (t0, t1, x0, x1): trapezoidal speed
    profile (speed, accel) followed by the final positioning time t_pos."""

    def __init__(self, moves, x_start, speed, accel):
        self.moves = sorted(moves)
        self.starts = [m[0] for m in self.moves]
        self.x_start, self.v, self.a = x_start, speed, accel

    def _travel(self, d, tau):
        v, a = self.v, self.a
        if d <= 0:
            return 0.0
        if d >= v * v / a:                      # trapezoid
            ta, T = v / a, d / v + v / a
            if tau <= ta:
                return 0.5 * a * tau * tau
            if tau <= T - ta:
                return 0.5 * v * ta + v * (tau - ta)
            tau = min(tau, T)
            return d - 0.5 * a * (T - tau) ** 2
        ta = math.sqrt(d / a)                   # triangle
        T = 2 * ta
        if tau <= ta:
            return 0.5 * a * tau * tau
        tau = min(tau, T)
        return d - 0.5 * a * (T - tau) ** 2

    def motion_time(self, d):
        v, a = self.v, self.a
        if d <= 0:
            return 0.0
        return d / v + v / a if d >= v * v / a else 2 * math.sqrt(d / a)

    def x(self, t):
        i = bisect_right(self.starts, t) - 1
        if i < 0:
            return self.x_start
        a, b, x0, x1 = self.moves[i]
        if t >= b:
            return x1
        d = abs(x1 - x0)
        s = self._travel(d, t - a)
        return x0 + math.copysign(s, x1 - x0)

    def moving_intervals(self):
        """[(a, a + motion time)] of every move (the positioning time is a hold)."""
        out = []
        for a, b, x0, x1 in self.moves:
            T = min(self.motion_time(abs(x1 - x0)), b - a)
            if T > 0:
                out.append((a, a + T))
        return out


# --------------------------------------------------------------- timelines --
class Timeline:
    """Piecewise description of an asset's position: segments (ta, tb, f, moving)."""

    def __init__(self):
        self.segs = []

    def add(self, ta, tb, f, moving):
        if tb > ta or not self.segs:
            self.segs.append((ta, tb, f, moving))

    def pos(self, t):
        for ta, tb, f, _ in reversed(self.segs):
            if ta <= t:
                return f(min(t, tb))
        return self.segs[0][2](self.segs[0][0])

    def samples(self, T0, T1, dt):
        """Times at which a 'show' keyframe must be emitted in [T0, T1]."""
        times = {T0}
        for ta, tb, f, moving in self.segs:
            if tb < T0 or ta > T1:
                continue
            a, b = max(ta, T0), min(tb, T1)
            times.add(a)
            if moving:
                n = max(1, math.ceil((b - a) / dt))
                times.update(a + (b - a) * k / n for k in range(n + 1))
        return sorted(times)


def lerp_path(points, ta, tb):
    """Position function moving along a polyline between ta and tb (constant speed)."""
    lens = [math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
    total = sum(lens) or 1.0

    def f(t):
        s = 0.0 if tb <= ta else max(0.0, min(1.0, (t - ta) / (tb - ta))) * total
        for i, L in enumerate(lens):
            if s <= L or i == len(lens) - 1:
                u = 0.0 if L == 0 else min(1.0, s / L)
                p, q = points[i], points[i + 1]
                return tuple(p[k] + (q[k] - p[k]) * u for k in range(3))
            s -= L
        return points[-1]
    return f


def const(p):
    return lambda t: p


# ------------------------------------------------------------- the builder --
class Builder:
    def __init__(self, cfg, model, shuttle_x0):
        self.cfg, self.model, self.log = cfg, model, model.log
        self.lu_x = {s.name: s.x for s in cfg.lu_stations}
        self.mt_x = {m.name: m.x for m in cfg.machines}
        self.rack_x = {f"R{i + 1}": x for i, x in enumerate(cfg.rack_x)}
        sh = cfg.shuttle
        self.kin = ShuttleKinematics(self.log.moves, shuttle_x0, sh.speed, sh.accel)

    # positions -----------------------------------------------------------
    def station_pos(self, loc, receiver=1):
        if loc in self.lu_x:
            return (self.lu_x[loc], TRANSFER_H, -RAIL_TO_STATION)
        if loc in self.rack_x:
            return (self.rack_x[loc], TRANSFER_H, RAIL_TO_STATION)
        mt, kind = loc.split(".")
        x = self.mt_x[mt]
        if kind == "WORK":
            return (x, TRANSFER_H, -RAIL_TO_STATION - WORK_DZ)
        dx = -APC_DX if receiver == 1 else APC_DX
        return (x + dx, TRANSFER_H, -RAIL_TO_STATION)

    def shuttle_pallet_pos(self, t):
        return (self.kin.x(t), TRANSFER_H, 0.0)

    # helper: start of the activity that ends at t ------------------------
    def _state_start(self, obj, state, t, pallet=None, default=None):
        best = None
        for ts, st, p in self.log.states.get(obj, []):
            if ts > t + 1e-9:
                break
            if st == state and (pallet is None or p == pallet):
                best = ts
        if best is None or t - best > 3600:
            return default if default is not None else t
        return best

    # pallet timeline --------------------------------------------------------
    def pallet_timeline(self, name, t_end):
        locs = self.log.locations[name]
        tl = Timeline()
        receiver = {}             # machine -> receiver (1 incoming, 2 outgoing)
        pos = self.station_pos(locs[0][1])
        prev, t_prev = locs[0][1], locs[0][0]
        tl.add(t_prev, t_prev, const(pos), False)
        moving_sh = self.kin.moving_intervals()

        for i, (t, loc) in enumerate(locs[1:] + [(t_end, None)]):
            if loc is None:      # end of the run: hold the last location
                start = t
            elif loc == "SHUTTLE":
                start = self._state_start("SHUTTLE", "PICK", t, name, default=t - self.cfg.shuttle.t_transfer)
            elif prev == "SHUTTLE":
                start = self._state_start("SHUTTLE", "DROP", t, name, default=t - self.cfg.shuttle.t_transfer)
            else:                # APC <-> WORK pallet change
                mt = loc.split(".")[0]
                start = self._state_start(mt, "APC_CHANGE", t, default=t - 20)
            start = max(start, t_prev)

            # stay at prev between t_prev and start
            if prev == "SHUTTLE":
                cuts = sorted({t_prev, start} | {c for a, b in moving_sh for c in (a, b)
                                                  if t_prev < c < start})
                for a, b in zip(cuts[:-1], cuts[1:]):
                    mov = any(ma < b and mb > a for ma, mb in moving_sh)
                    tl.add(a, b, self.shuttle_pallet_pos, mov)
                pos = self.shuttle_pallet_pos(start)
            else:
                tl.add(t_prev, start, const(pos), False)
            if loc is None:
                break

            # transition prev -> loc during [start, t]
            if loc == "SHUTTLE":
                dst = self.shuttle_pallet_pos(t)
                path = [pos, (dst[0], pos[1], pos[2] * 0.5), dst] if abs(dst[0] - pos[0]) > 1e-6 else [pos, dst]
                new = dst
            elif prev == "SHUTTLE":
                if loc.endswith(".APC"):
                    receiver[loc] = 1
                new = self.station_pos(loc, receiver.get(loc, 1))
                path = [pos, (new[0], pos[1], pos[2]), new] if abs(new[0] - pos[0]) > 1e-6 else [pos, new]
            else:
                mt = loc.split(".")[0]
                door = (self.mt_x[mt], TRANSFER_H, -RAIL_TO_STATION)
                if loc.endswith(".WORK"):
                    new = self.station_pos(loc)
                else:                       # finished pallet back on the 2nd receiver
                    receiver[loc] = 2
                    new = self.station_pos(loc, 2)
                path = [pos, door, new]
                # pallet change: the finished pallet leaves the table in the first half,
                # the new one enters in the second half (no overlap in the door)
                mid = 0.5 * (start + t)
                if loc.endswith(".WORK"):
                    tl.add(start, mid, const(pos), False)
                    start = mid
                else:
                    tl.add(start, mid, lerp_path(path, start, mid), True)
                    tl.add(mid, t, const(new), False)
                    pos, prev, t_prev = new, loc, t
                    continue
            tl.add(start, t, lerp_path(path, start, t), True)
            pos, prev, t_prev = new, loc, t
        return tl

    def shuttle_timeline(self, t_end):
        tl = Timeline()
        cuts = sorted({0.0, t_end} | {c for a, b in self.kin.moving_intervals() for c in (a, b)})
        mov = self.kin.moving_intervals()
        f = lambda t: (self.kin.x(t), 0.0, 0.0)
        for a, b in zip(cuts[:-1], cuts[1:]):
            tl.add(a, b, f, any(ma < b and mb > a for ma, mb in mov))
        return tl


# --------------------------------------------------------------- models -----
def read_glb(path: Path):
    b = path.read_bytes()
    jlen = struct.unpack("<I", b[12:16])[0]
    gltf = json.loads(b[20:20 + jlen])
    rest = b[20 + jlen:]
    return gltf, rest


def write_glb(path: Path, gltf, rest: bytes):
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    total = 12 + 8 + len(js) + len(rest)
    path.write_bytes(struct.pack("<III", 0x46546C67, 2, total) + struct.pack("<II", len(js), 0x4E4F534A) + js + rest)


def recentre_x(src: Path, dst: Path, cx: float):
    """Copy a GLB moving its origin to x = cx (glTF axes), so that a model symmetric
    about cx looks the same whether the viewer mirrors the model X axis or not
    (Babylon.js left-handed scenes do)."""
    gltf, rest = read_glb(src)
    for i in gltf["scenes"][gltf.get("scene", 0)]["nodes"]:
        n = gltf["nodes"][i]
        if "matrix" in n:
            n["matrix"][12] -= cx
        else:
            t = n.get("translation", [0, 0, 0])
            n["translation"] = [t[0] - cx, t[1], t[2]]
    write_glb(dst, gltf, rest)


def make_pallet_variant(src: Path, dst: Path, rgb, label):
    gltf, rest = read_glb(src)
    for m in gltf.get("materials", []):
        if m.get("name") == "safety_yellow":
            m["pbrMetallicRoughness"]["baseColorFactor"] = [*rgb, 1.0]
            m["name"] = f"fixture_{label}"
    for n in gltf["nodes"]:
        if n.get("name") == "pallet_base":
            n["name"] = f"pallet_{label}"
    write_glb(dst, gltf, rest)


def copy_models(fms_dir: Path, models_dir: Path, track_len: float, types):
    """Copy the GLB models; the track is the shortest generated one >= track_len."""
    models_dir.mkdir(parents=True, exist_ok=True)
    src = fms_dir / "3Dmodels"
    for f in MODEL_FILES:
        shutil.copyfile(src / f, models_dir / f)
    tracks = []
    for f in src.glob("transporter_track_*m.glb"):
        try:
            tracks.append((float(f.stem.split("_")[-1][:-1]), f.name))
        except ValueError:
            pass
    ok = sorted(t for t in tracks if t[0] >= track_len - 1e-6)
    if not ok:
        raise SystemExit(f"no track >= {track_len:g} m: run 'python make_models.py --track {track_len:g}' in {src}")
    length, name = ok[0]
    track_file = name.replace(".glb", "_centred.glb")
    recentre_x(src / name, models_dir / track_file, length / 2)
    for i, t in enumerate(types):
        make_pallet_variant(src / "pallet_base.glb", models_dir / f"pallet_{t}.glb",
                            TYPE_COLOURS[i % len(TYPE_COLOURS)], t)
    return track_file, length


def rack_model(fms_dir: Path, models_dir: Path, count: int, spacing: float):
    name = f"storage_1L_{count}S_{spacing:g}m.glb"
    src = fms_dir / "3Dmodels" / name
    if not src.exists():
        raise SystemExit(f"{name} not found: run 'python make_models.py --storage 1 {count} {spacing:g}' "
                         f"in {src.parent}")
    out = name.replace(".glb", "_centred.glb")
    recentre_x(src, models_dir / out, (count - 1) * spacing / 2)   # origin between slot 1 and slot n
    return out


# ---------------------------------------------------------------- scene -----
def asset(id_, type_, descr, file=None, position=(0, 0, 0), rotation=(0, 0, 0), **extra):
    a = {"id": id_, "descr": descr, "type": type_}
    if file:
        a["representations"] = [{"file": file, "unit": 1}]
    a["position"] = r3(position)
    a["rotation"] = r3(rotation)
    a.update(extra)
    return a


def build_scene(cfg, builder, track_file, track_len, rack_file, pallet_pos0, shuttle_x0):
    xs = [s.x for s in cfg.lu_stations] + [m.x for m in cfg.machines] + list(cfg.rack_x)
    x_mid = (min(xs) + max(xs)) / 2
    assets = [
        asset("FMS", FACTORY + "ManufacturingSystem", f"FMS '{cfg.name}' (fmssim)"),
        asset("building", IFC + "IfcBuilding", "Shop floor"),
        asset("track", IFC + "IfcElement", f"Rail track of the shuttle ({track_len:g} m)", track_file,
              (x_mid, 0, 0), placementRelTo="building", assignmentTo=["FMS"]),
        asset("SHUTTLE", FACTORY + "ProductionResource", "Rail-guided shuttle with telescopic fork",
              "transporter_shuttle.glb", (shuttle_x0, 0, 0), assignmentTo=["FMS"]),
    ]
    for s in cfg.lu_stations:
        assets.append(asset(s.name, FACTORY + "ProductionResource", f"Manual load/unload station {s.name}",
                            "lu_station_manual.glb", (s.x, 0, -RAIL_TO_STATION),
                            placementRelTo="building", assignmentTo=["FMS"]))
    for m in cfg.machines:
        assets.append(asset(m.name, FACTORY + "MachineTool", f"Horizontal machining centre {m.name} with 2-position APC",
                            "hmc_2apc.glb", (m.x, 0, -RAIL_TO_STATION),
                            placementRelTo="building", assignmentTo=["FMS"]))
    # rack on the other side of the rail, rotated 180 deg so that its front faces the rail;
    # the model origin is centred between the first and last slot
    x_rack = (min(cfg.rack_x) + max(cfg.rack_x)) / 2
    assets.append(asset("RACK", FACTORY + "StorageElement",
                        f"Pallet rack, {len(cfg.rack_x)} slots (R1 at x = {min(cfg.rack_x):g} m)",
                        rack_file, (x_rack, 0, RAIL_TO_STATION), (0, math.pi, 0),
                        placementRelTo="building", assignmentTo=["FMS"]))
    for p in builder.model.pallets:
        assets.append(asset(p.name, FACTORY + "Pallet", f"Pallet {p.name}, fixture for part type {p.fixture}",
                            f"pallet_{p.fixture}.glb", pallet_pos0[p.name], assignmentTo=["FMS"]))
    return {
        "context": {"UnitOfMeasureScale": 1, "Zup": False, "RepoPath": ""},
        "scene": [a["id"] for a in assets if a["id"] != "FMS"],
        "assets": assets,
    }


def build_env(cfg):
    xs = [s.x for s in cfg.lu_stations] + [m.x for m in cfg.machines] + list(cfg.rack_x)
    xc = (min(xs) + max(xs)) / 2
    L = max(xs) - min(xs)
    cams = [
        # overview from the rack side, looking at the machines (-Z)
        {"id": "camera.overview", "type": "FlyCamera", "position": r3((xc, 0.55 * L, 0.6 * L)),
         "rotation": r3((math.atan2(0.55 * L, 0.6 * L + 1.5), math.pi, 0)), "speed": 0.3},
        # along the rail, from the L/U end
        {"id": "camera.rail", "type": "FlyCamera", "position": r3((min(xs) - 6, 4.5, 4.0)),
         "rotation": r3((0.28, math.pi / 2 + 0.35, 0)), "speed": 0.2},
        # operator view at the L/U stations
        {"id": "camera.lu", "type": "UniversalCamera",
         "position": r3((cfg.lu_stations[0].x + 1.5, 1.7, -RAIL_TO_STATION - 4.5)),
         "rotation": r3((0.18, -0.3, 0)), "speed": 0.1},
        # machines from behind (table / work area side)
        {"id": "camera.machines", "type": "FlyCamera",
         "position": r3((cfg.machines[0].x - 6, 5.0, -RAIL_TO_STATION - 9)),
         "rotation": r3((0.35, 0.75, 0)), "speed": 0.2},
    ]
    return {
        "cameras": cams,
        "lights": [
            {"id": "light.hemi", "type": "HemisphericLight", "direction": [0.3, 1, 0.2], "intensity": 0.9},
            {"id": "light.p1", "type": "PointLight", "position": r3((xc - L / 3, 8, 0)), "intensity": 0.6},
            {"id": "light.p2", "type": "PointLight", "position": r3((xc + L / 3, 8, 0)), "intensity": 0.6},
        ],
        "shadows": {"darkness": 0.3, "bias": 1e-05, "environmentIntensity": 0.4},
        "env": {"skyboxTexture": "textures/studio.env", "backgroundColor": [0.2, 0.22, 0.28]},
    }


# ------------------------------------------------------------ animation -----
def build_anim(cfg, builder, model, T0, T1, dt, scale, t_end):
    ms = lambda t: str(int(round((t - T0) * 1000.0 / scale / TICK_MS)) * TICK_MS)
    show = lambda t, p: {"trigger": {"type": "timestamp", "data": ms(t)},
                         "event": {"type": "show", "position": r3(p), "rotation": [0, 0, 0]}}
    state = lambda t, d: {"trigger": {"type": "timestamp", "data": ms(t)},
                          "event": {"type": "state", "descr": d}}
    nodes = []

    # shuttle --------------------------------------------------------------
    tl = builder.shuttle_timeline(t_end)
    acts = [show(t, tl.pos(t)) for t in tl.samples(T0, T1, dt)]
    acts += _states(model.log.states.get("SHUTTLE", []), T0, T1, state, lambda st, p: f"{st} {p or ''}".strip())
    nodes.append({"id": "SHUTTLE", "actions": _sorted(acts)})

    # pallets ---------------------------------------------------------------
    for p in model.pallets:
        tl = builder.pallet_timeline(p.name, t_end)
        acts = [show(t, tl.pos(t)) for t in tl.samples(T0, T1, dt)]
        ps = [(t, s, pid) for t, s, pid in model.log.pallet_states.get(p.name, [])]
        acts += _states(ps, T0, T1, state, lambda st, pid: st + (f" (part {pid})" if pid else ""))
        nodes.append({"id": p.name, "actions": _sorted(acts)})

    # machines and L/U stations: states, with the operation in progress ----
    ops = {}
    for o in model.log.ops:
        ops[(o["machine"], round(o["start"], 6))] = o
    for res in [s.name for s in cfg.lu_stations] + [m.name for m in cfg.machines]:
        def fmt(st, pal, res=res):
            return f"{st} {pal or ''}".strip()
        log = model.log.states.get(res, [])
        acts = []
        for i, (t, st, pal) in enumerate(log):
            t_next = log[i + 1][0] if i + 1 < len(log) else math.inf
            if t_next < T0 or t > T1:
                continue
            d = fmt(st, pal)
            o = ops.get((res, round(t, 6)))
            if st == "MACHINING" and o:
                d = f"MACHINING {o['pallet']} {o['op']} part {o['part']} ({o['end'] - o['start']:.0f} s)"
            acts.append(state(max(t, T0), d))
        nodes.append({"id": res, "actions": _sorted(acts)})

    step = 3600 if T1 - T0 > 3 * 3600 else 900
    bookmarks = [{"name": f"t = {t / 3600:g} h", "time": ms(t)}
                 for t in _frange(T0, T1, step)]
    return {
        "context": {"assetTrail": False, "UnitOfMeasureScale": 1, "Zup": False, "RepoAnim": ""},
        "nodes": nodes,
        "sequences": [],
        "bookmarks": bookmarks,
    }


def _states(log, T0, T1, state, fmt):
    out = []
    for i, (t, st, x) in enumerate(log):
        t_next = log[i + 1][0] if i + 1 < len(log) else math.inf
        if t_next < T0 or t > T1:
            continue
        out.append(state(max(t, T0), fmt(st, x)))
    return out


def _sorted(acts):
    """Sort by time; keep only the last 'show' of each tick (they would overwrite each other)."""
    acts = sorted(acts, key=lambda a: (int(a["trigger"]["data"]), a["event"]["type"] != "show"))
    out, last_show = [], {}
    for a in acts:
        t = a["trigger"]["data"]
        if a["event"]["type"] == "show":
            if t in last_show:
                out[last_show[t]] = a
                continue
            last_show[t] = len(out)
        out.append(a)
    return out


def _frange(a, b, s):
    t = math.ceil(a / s) * s
    out = [a]
    while t <= b:
        if t > a:
            out.append(t)
        t += s
    return out


# ------------------------------------------------------------------ main ----
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fms", default=str(PROJECT.parent / "SymPy-FMS"),
                    help="SymPy-FMS folder (package fmssim, 3Dmodels, configs)")
    ap.add_argument("--config", default=None, help="YAML configuration (default: <fms>/configs/reference.yaml)")
    ap.add_argument("--name", default=None, help="scene name (default: FMS_<config name>)")
    ap.add_argument("--from-h", type=float, default=0.0, help="animation start (simulated hours)")
    ap.add_argument("--to-h", type=float, default=2.0, help="animation end (simulated hours)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--dt", type=float, default=0.2, help="keyframe step during motions (simulated s)")
    ap.add_argument("--time-scale", type=float, default=1.0,
                    help="simulated seconds per animation second (1 = real time)")
    ap.add_argument("--set", action="append", default=[], help="override a config value, key=value")
    args = ap.parse_args(argv)

    import yaml
    fms = Path(args.fms).resolve()
    config = Path(args.config) if args.config else fms / "configs" / "reference.yaml"
    overrides = {k: yaml.safe_load(v) for k, v in (s.split("=", 1) for s in args.set)}
    T0, T1 = args.from_h * 3600, args.to_h * 3600
    t_end = T1 + 3600  # simulate a bit beyond the window so that running activities have an end

    cfg, model, seed, deadlock = run_model(fms, config, args.seed, t_end, overrides)
    t_end = min(t_end, model.env.now)
    T1 = min(T1, t_end)
    name = args.name or f"FMS_{cfg.name}"

    # shuttle start position
    x0 = {**{s.name: s.x for s in cfg.lu_stations}, **{m.name: m.x for m in cfg.machines}}[cfg.shuttle.start]
    builder = Builder(cfg, model, x0)

    models_dir = PROJECT / "models"
    xs = [s.x for s in cfg.lu_stations] + [m.x for m in cfg.machines] + list(cfg.rack_x)
    track_len = math.ceil(max(xs) - min(xs) + 2 * TRACK_MARGIN)
    types = list(cfg.part_types)
    track_file, track_len = copy_models(fms, models_dir, track_len, types)
    rack_count = len(cfg.rack_x)
    spacing = (cfg.rack_x[1] - cfg.rack_x[0]) if rack_count > 1 else 3.0
    rack_file = rack_model(fms, models_dir, rack_count, spacing)

    # pallet and shuttle poses at T0 (scene) and at time 0 (static layout)
    tls = {p.name: builder.pallet_timeline(p.name, t_end) for p in model.pallets}
    sh_tl = builder.shuttle_timeline(t_end)
    pos_T0 = {k: tl.pos(T0) for k, tl in tls.items()}
    pos_0 = {k: tl.pos(0.0) for k, tl in tls.items()}

    scenes = PROJECT / "scenes"
    scenes.mkdir(exist_ok=True)
    scene = build_scene(cfg, builder, track_file, track_len, rack_file, pos_T0, sh_tl.pos(T0)[0])
    layout = build_scene(cfg, builder, track_file, track_len, rack_file, pos_0, x0)
    env = build_env(cfg)
    anim = build_anim(cfg, builder, model, T0, T1, args.dt, args.time_scale, t_end)

    def dump(obj, fn, compact=False):
        path = scenes / fn
        path.write_text(json.dumps(obj, indent=None if compact else 2, separators=(",", ":") if compact else None),
                        encoding="utf-8")
        return path

    dump(scene, f"{name}.json")
    dump(env, f"{name}_env.json")
    p = dump(anim, f"{name}_anim.json", compact=True)
    dump(layout, f"{name}_layout.json")
    dump(env, f"{name}_layout_env.json")

    n_ev = sum(len(n["actions"]) for n in anim["nodes"])
    print(f"{name}: seed {seed}, window {T0 / 3600:g}-{T1 / 3600:g} h, time scale {args.time_scale:g}, "
          f"{n_ev} animation events ({p.stat().st_size / 1e6:.1f} MB)")
    if deadlock:
        print(f"WARNING: the run stopped with a deadlock: {deadlock}")
    print(f"written to {scenes}")


if __name__ == "__main__":
    main()
