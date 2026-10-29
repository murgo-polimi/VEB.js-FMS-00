# VEB.js-FMS-00 — the fmssim FMS in VEB.js

3D scene of the pallet-based FMS used by the SimPy simulator (`../SymPy-FMS`),
built with the generated GLB models, plus an animated replay of a simulation run,
for [VEB.js](https://virtualfactory.gitbook.io/vlft/tools/vebjs).

## Open it

```bash
python serve.py          # serves this folder at http://localhost:8000 with CORS
```

The launcher page (`http://localhost:8000/index.html`) opens VEB.js
(`https://digital-factory.io/vebjs/`) with:

| Link | Scene |
| --- | --- |
| Animated replay | `scenes/FMS_reference.json` (+ `_env.json`, `_anim.json` loaded automatically) |
| Static layout | `scenes/FMS_reference_layout.json` (+ `_env.json`) |

i.e. `https://digital-factory.io/vebjs/?inputscene=http://localhost:8000/scenes/FMS_reference.json&repoMod3d=http://localhost:8000/models/`

Use Chrome. If Chrome asks to allow access to devices on the local network, allow it
(the VEB.js page reads files from `localhost`). To share the scene, put this folder on any
web server with CORS (e.g. GitHub Pages) and use its URL instead of `http://localhost:8000/`.

## Content

```
models/   GLB models (copied from SymPy-FMS/3Dmodels, metres, +Y up)
          pallet_A.glb, pallet_B.glb      pallet with the fixture marker coloured by part type (A blue, B red)
          *_centred.glb                   track and rack with the origin moved to their centre
scenes/   FMS_reference.json              scene (assets and placement)
          FMS_reference_env.json          cameras, lights, shadows
          FMS_reference_anim.json         animation from the simulation run
          FMS_reference_layout*.json      same scene, no animation
tools/build_vebjs.py                      generator of models/ and scenes/
serve.py, index.html                      local server and launcher
```

### Scene

Built from `configs/reference.yaml` with the conventions of `SymPy-FMS/3Dmodels/README.md`
(units m, `UnitOfMeasureScale: 1`, X = rail):

| Asset id | Model | Position |
| --- | --- | --- |
| `track` | `transporter_track_31m_centred.glb` | rail axis, x −0.5 … 30.5 m |
| `SHUTTLE` | `transporter_shuttle.glb` | on the rail, `shuttle.start` |
| `LU1`, `LU2` | `lu_station_manual.glb` | (x, 0, −1.40), front towards the rail |
| `MT1` … `MT3` | `hmc_2apc.glb` | (x, 0, −1.40), APC towards the rail |
| `RACK` | `storage_1L_10S_3m_centred.glb` | other side, z = +1.40, rotated 180°; slots R1…R10 at x = 1.5 … 28.5 |
| `P01` … `P08` | `pallet_<type>.glb` | pallet bottom at 0.90 m |

Track and rack are re-centred because Babylon.js (left-handed) mirrors the X axis of glTF
models: with the origin at the centre of a symmetric model the scene is the same in both
conventions.

### Animation

`tools/build_vebjs.py` runs the SimPy model and converts its log into VEB.js events:

* **shuttle**: x(t) with the trapezoidal speed profile (speed, accel) of each move;
* **pallets**: rack → shuttle → L/U → shuttle → machine APC → table, following the shuttle
  while carried; pick/drop transfers last `t_transfer`, the APC change `t_apc` (the
  finished pallet leaves the table on the second receiver, then the new one enters);
* **states** (`state` events, shown in the animation Log): machines (`MACHINING P01 Op10 part 1`,
  `APC_CHANGE`, `BLOCKED`, `DOWN:<mode>` …), L/U stations, shuttle, pallets;
* bookmarks every 15 min (every hour for windows longer than 3 h).

Positions are `show` keyframes every 0.2 simulated s during motions (VEB.js does not
interpolate between `show` events). VEB.js plays the animation in 100 ms ticks and fires
only events whose timestamp is a multiple of 100 ms, so all timestamps are rounded to that
grid. Animation time = simulated time (1 s = 1 s); use the speed slider, or `--time-scale`.

## Regenerate / other runs

Needs Python 3.11+, `simpy pyyaml numpy` and the SymPy-FMS folder next to this one
(or `--fms <path>`).

```bash
python tools/build_vebjs.py                                   # reference.yaml, 0–2 h
python tools/build_vebjs.py --from-h 8 --to-h 12 --seed 3     # another window/seed
python tools/build_vebjs.py --config ../SymPy-FMS/configs/reference_stochastic.yaml \
       --name FMS_stochastic --to-h 4                         # -> scenes/FMS_stochastic*.json
python tools/build_vebjs.py --set pallets.count=6 --name FMS_6pallets
python tools/build_vebjs.py --time-scale 10                   # 10 simulated s per animation s
```

A layout with a different rack or a longer rail needs the matching models
(`python make_models.py --storage 1 <slots> <pitch> --track <length>` in `SymPy-FMS/3Dmodels`);
the script says which one is missing. The file size grows with the window (≈1 MB per
simulated hour with 8 pallets).

## Limits / next steps

* Assets are moved as a whole: the moving nodes in the GLB models (shuttle fork, APC
  carriage, pallet door, L/U turntable, spindle) are not animated yet; they can be addressed
  as VEB.js components (`"file": "hmc_2apc.glb#pallet_door"`, `parentObject`).
* Parts are not drawn on the pallets (the pallet state in the Log says when it carries a part).
* Only the 0.90 m rack level is used, as in the simulator.
