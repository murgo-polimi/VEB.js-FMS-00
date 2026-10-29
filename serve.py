"""Serve this project over HTTP with CORS, so that VEB.js can load scenes and models.

    python serve.py            # http://localhost:8000
    python serve.py 8080

VEB.js runs at https://digital-factory.io/vebjs/ and reads the scene, environment,
animation and GLB models from this server (URLs printed below; open them in Chrome).
"""

import sys
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import quote

ROOT = Path(__file__).resolve().parent
VEBJS = "https://digital-factory.io/vebjs/"


class CORSHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        # Chrome: a public https page reading from localhost (Private/Local Network Access)
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.end_headers()

    def log_message(self, fmt, *args):
        if "404" in (args[1] if len(args) > 1 else ""):
            sys.stderr.write(f"  not found: {args[0]}\n")


def vebjs_url(base: str, scene: str) -> str:
    return (f"{VEBJS}?inputscene={quote(base + 'scenes/' + scene, safe=':/')}"
            f"&repoMod3d={quote(base + 'models/', safe=':/')}")


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    base = f"http://localhost:{port}/"
    scenes = sorted(p.name for p in (ROOT / "scenes").glob("*.json")
                    if not p.stem.endswith(("_env", "_anim", "_config")))
    print(f"Serving {ROOT} at {base}  (Ctrl+C to stop)\n")
    for s in scenes:
        print(f"{s}:\n  {vebjs_url(base, s)}\n")
    print(f"Launcher page: {base}index.html")
    server = ThreadingHTTPServer(("localhost", port), partial(CORSHandler, directory=str(ROOT)))
    if "--no-browser" not in sys.argv:
        webbrowser.open(base + "index.html")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
