#!/usr/bin/env python3
"""측면 웹캠(Logitech C270, /dev/video8) MJPEG 스트림 — 9/4. 삽입 중 벽 밑동 측면 관찰용.
  python3 side_cam.py            # http://<PC>:8771/  (뷰어)  ·  /raw 단일 프레임  ·  /stream MJPEG
"""
import sys, time, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2

DEV = int(sys.argv[sys.argv.index("--dev") + 1]) if "--dev" in sys.argv else 8
PORT = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 8771
latest = {"jpg": None, "t": 0}


def worker():
    cap = cv2.VideoCapture(DEV)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while True:
        ok, f = cap.read()
        if not ok:
            time.sleep(0.1); continue
        ok2, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok2:
            latest["jpg"] = jpg.tobytes(); latest["t"] = time.time()
        time.sleep(0.04)


PAGE = b"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>\xec\xb8\xa1\xeb\xa9\xb4 \xec\xb9\xb4\xeb\xa9\x94\xeb\x9d\xbc</title><style>body{margin:0;background:#111}img{width:100%;max-width:960px;display:block;margin:0 auto}</style></head>
<body><img src="/stream"></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    if latest["jpg"]:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(latest["jpg"]); self.wfile.write(b"\r\n")
                    time.sleep(0.05)
            except Exception:
                return
        if self.path.startswith("/health"):
            b = ('{"ok": true, "source": "sidecam", "age": %.1f}' % (time.time() - latest["t"] if latest["t"] else 999)).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b); return
        if self.path.startswith("/raw"):
            j = latest["jpg"] or b""
            self.send_response(200); self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(j))); self.end_headers(); self.wfile.write(j)
            return
        self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(PAGE))); self.end_headers(); self.wfile.write(PAGE)


if __name__ == "__main__":
    threading.Thread(target=worker, daemon=True).start()
    time.sleep(1.0)
    print(f"side_cam[/dev/video{DEV}] → http://0.0.0.0:{PORT}/", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
