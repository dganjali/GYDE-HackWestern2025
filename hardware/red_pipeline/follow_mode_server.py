#!/usr/bin/env python3
"""
follow_mode_server.py

Tiny web server to control the robot's follow/stay mode for testing.
- GET /            -> simple HTML page with current mode and toggle links
- GET /mode        -> returns plain text: "follow" or "stay"
- GET /set/follow  -> sets mode to "follow" and redirects to /
- GET /set/stay    -> sets mode to "stay" and redirects to /

Defaults to host 0.0.0.0 and port 8080 so the Pi can reach it.

Usage:
  python3 hardware/red_pipeline/follow_mode_server.py --port 8080 --host 0.0.0.0

Then point the Pi controller's FOLLOW_MODE_URL to:
  http://<laptop_ip>:8080/mode
"""

import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import socket

MODE = "follow"  # default


def html_page(mode: str) -> str:
    color = "#2e7d32" if mode == "follow" else "#c62828"
    status = "FOLLOW" if mode == "follow" else "STAY"
    return f"""
<!DOCTYPE html>
<html>
<head>
  <meta charset=\"utf-8\">
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
  <title>Follow Mode Control</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 40px; }}
    .status {{ font-size: 48px; color: {color}; font-weight: 700; }}
    .buttons a {{ display: inline-block; margin-right: 12px; padding: 10px 16px; text-decoration: none; border-radius: 6px; color: white; }}
    .follow {{ background: #2e7d32; }}
    .stay {{ background: #c62828; }}
    .modebox {{ margin-top: 20px; }}
    code {{ background: #f2f2f2; padding: 2px 6px; border-radius: 4px; }}
  </style>
</head>
<body>
  <h1>Follow Mode Control</h1>
  <div class="status">Current Mode: {status}</div>
  <div class="modebox">
    <div class="buttons">
      <a class="follow" href="/set/follow">Set FOLLOW</a>
      <a class="stay" href="/set/stay">Set STAY</a>
    </div>
    <p>API endpoint for the Pi controller: <code>/mode</code></p>
    <p>Returns plain text <code>follow</code> or <code>stay</code>.</p>
  </div>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code=200, body=b"", content_type="text/plain; charset=utf-8"):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.end_headers()
        if body:
            self.wfile.write(body)

    def do_GET(self):
        global MODE
        if self.path == "/":
            page = html_page(MODE).encode("utf-8")
            self._send(200, page, content_type="text/html; charset=utf-8")
            return
        if self.path == "/mode":
            self._send(200, MODE.encode("utf-8"))
            return
        if self.path.startswith("/set/"):
            new_mode = self.path.split("/set/")[-1].strip().lower()
            if new_mode in ("follow", "stay"):
                MODE = new_mode
                # Redirect back to home
                self.send_response(302)
                self.send_header("Location", "/")
                self.end_headers()
                return
            self._send(400, b"invalid mode; use follow or stay")
            return
        self._send(404, b"not found")

    def log_message(self, fmt, *args):
        # Make logs concise
        host, _ = self.client_address
        print(f"[{host}] {self.command} {self.path}")


def main():
    parser = argparse.ArgumentParser(description="Follow Mode Test Server")
    parser.add_argument("--host", default="0.0.0.0", help="Host/IP to bind (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Port to bind (default: 8080)")
    args = parser.parse_args()

    httpd = HTTPServer((args.host, args.port), Handler)
    ip = socket.gethostbyname(socket.gethostname())
    print(f"Follow Mode Server running on http://{args.host}:{args.port}")
    print(f"Local hostname/IP guess: {socket.gethostname()} / {ip}")
    print("Endpoints:")
    print("  /        -> control UI")
    print("  /mode    -> returns 'follow' or 'stay'")
    print("  /set/follow  /set/stay -> toggle mode")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")


if __name__ == "__main__":
    main()
