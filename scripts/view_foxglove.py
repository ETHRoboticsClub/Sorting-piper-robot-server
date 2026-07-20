#!/usr/bin/env python
"""Serve recorded episodes and open them in Foxglove.

    python scripts/view_foxglove.py              # newest episode of the newest session
    python scripts/view_foxglove.py --list       # list what has been recorded
    python scripts/view_foxglove.py path/to/episode_0003.mcap
    python scripts/view_foxglove.py --no-browser # just serve, print the URL

Foxglove's web app loads a log over HTTP, which needs two things a plain
``http.server`` does not provide: permissive **CORS** (the page is served from
app.foxglove.dev, the file from localhost) and **Range** requests (Foxglove seeks
within the file instead of downloading it whole). Both are implemented below.

The server runs until you Ctrl+C it -- keep it alive while you are scrubbing.
"""

from __future__ import annotations

import argparse
import os
import posixpath
import re
import socketserver
import sys
import threading
import urllib.parse
import webbrowser
from http.server import SimpleHTTPRequestHandler

DEFAULT_ROOT = os.path.join("outputs", "foxglove")
FOXGLOVE_APP = "https://app.foxglove.dev/~/view"


class RangeCORSHandler(SimpleHTTPRequestHandler):
    """Static file handler with CORS and single-range byte serving."""

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.send_header("Accept-Ranges", "bytes")
        super().end_headers()

    def do_OPTIONS(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        self.send_response(204)
        self.end_headers()

    def do_GET(self):  # noqa: N802
        rng = self.headers.get("Range")
        if not rng:
            return super().do_GET()

        path = self.translate_path(self.path)
        if not os.path.isfile(path):
            return super().do_GET()

        size = os.path.getsize(path)
        match = re.match(r"bytes=(\d*)-(\d*)", rng.strip())
        if not match:
            return super().do_GET()
        start_s, end_s = match.groups()
        if start_s == "":  # suffix range: last N bytes
            length = int(end_s or 0)
            start, end = max(0, size - length), size - 1
        else:
            start = int(start_s)
            end = int(end_s) if end_s else size - 1
        if start >= size:
            self.send_response(416)
            self.send_header("Content-Range", f"bytes */{size}")
            self.end_headers()
            return None
        end = min(end, size - 1)

        self.send_response(206)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.send_header("Content-Length", str(end - start + 1))
        self.end_headers()
        with open(path, "rb") as fh:
            fh.seek(start)
            remaining = end - start + 1
            while remaining > 0:
                chunk = fh.read(min(64 * 1024, remaining))
                if not chunk:
                    break
                self.wfile.write(chunk)
                remaining -= len(chunk)
        return None

    def log_message(self, *args):  # keep the terminal readable
        pass


def find_episodes(root: str) -> list[str]:
    hits: list[str] = []
    for dirpath, _dirnames, filenames in os.walk(root):
        hits += [os.path.join(dirpath, f) for f in filenames if f.endswith(".mcap")]
    return sorted(hits, key=os.path.getmtime)


def main() -> int:
    ap = argparse.ArgumentParser(description="Open recorded HIL-SERL episodes in Foxglove")
    ap.add_argument("episode", nargs="?", help="path to a .mcap (default: most recent)")
    ap.add_argument("--root", default=DEFAULT_ROOT, help=f"recording dir (default {DEFAULT_ROOT})")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--list", action="store_true", help="list recorded episodes and exit")
    ap.add_argument("--no-browser", action="store_true", help="serve only; do not open a browser")
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        print(f"No recordings at {root}", file=sys.stderr)
        print("Run a session first -- episodes are written as they finish.", file=sys.stderr)
        return 1

    episodes = find_episodes(root)
    if not episodes:
        print(f"No .mcap files under {root} yet.", file=sys.stderr)
        return 1

    if args.list:
        print(f"{len(episodes)} episode(s) under {root}:\n")
        for path in episodes:
            size = os.path.getsize(path) / 1e6
            print(f"  {os.path.relpath(path, root):40} {size:7.1f} MB")
        return 0

    target = os.path.abspath(args.episode) if args.episode else episodes[-1]
    if not os.path.isfile(target):
        print(f"Not a file: {target}", file=sys.stderr)
        return 1

    # Serve the recording root so other episodes are reachable without a restart.
    os.chdir(root)
    rel = posixpath.join(*os.path.relpath(target, root).split(os.sep))

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(("127.0.0.1", args.port), RangeCORSHandler) as httpd:
        file_url = f"http://127.0.0.1:{args.port}/{urllib.parse.quote(rel)}"
        view_url = f"{FOXGLOVE_APP}?ds=remote-file&ds.url={urllib.parse.quote(file_url, safe='')}"

        print(f"serving  {root}")
        print(f"episode  {rel}")
        print(f"url      {view_url}\n")
        if not args.no_browser:
            threading.Timer(0.5, lambda: webbrowser.open(view_url)).start()
            print("Opening Foxglove in your browser...")
        print("Ctrl+C to stop serving.\n")
        print("Suggested layout: Image panels on /camera/* and /grasp/overlay, Plot on")
        print("/reward.reward + /control.value, Raw Messages on /state and /action.\n")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nstopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
