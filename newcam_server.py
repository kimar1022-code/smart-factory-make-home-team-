#!/usr/bin/env python3
"""새 USB 카메라(뎁스 반대편) → MJPEG 중계 (9/1). 콘솔 <img> 로 바로 띄우기 위함.
    python3 newcam_server.py                 # http://<PC>:8768/stream
    DEV=/dev/video0 ROT=90 python3 newcam_server.py

  /stream  MJPEG 멀티파트(축소본, 콘솔용)      /snap  현재 1장(축소본)
  /full    원본 1280x720 1장 (계측용)          /health {frames,age,rot,dev,boot}
  /rot?d=90|180|270|0   회전 즉시 변경(조준 중에 쓰라고)  · /rot 만 부르면 90° 씩 증가
장치는 Realtek 0bda:5844 USB Camera — 1280x720 MJPEG 30fps, 고정초점(AF 컨트롤 없음).
"""
import os, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2

def find_dev():
    """9/1: USB 재삽입·D435 연결로 /dev/videoN 번호가 밀린다(새캠 video0 → video1).
    고정 경로 대신 udev 의 시리얼/모델로 '이 카메라'를 찾는다. DEV 를 주면 그걸 우선."""
    want = os.environ.get("DEV")
    if want and os.path.exists(want):
        return want
    import glob
    import subprocess
    for d in sorted(glob.glob("/dev/video*")):
        try:
            p = subprocess.run(["udevadm", "info", "-q", "property", "-n", d],
                               capture_output=True, text=True, timeout=3).stdout
        except Exception:
            continue
        if "ID_SERIAL_SHORT=200901010001" in p and "ID_V4L_CAPABILITIES=:capture:" in p:
            return d
    return want or "/dev/video0"


DEV  = find_dev()
PORT = int(os.environ.get("PORT", "8768"))
W_FULL, H_FULL = 1280, 720
W_STREAM = 640              # 콘솔용 축소
FPS = 15
BOOT = int(time.time())

state = {"rot": int(os.environ.get("ROT", "0")) % 360, "n": 0, "t": 0.0, "err": "", "dev": DEV}
latest = {"full": None, "small": None}
lock = threading.Lock()


def rotate(img, deg):
    if deg == 90:  return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if deg == 180: return cv2.rotate(img, cv2.ROTATE_180)
    if deg == 270: return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img


def pump():
    while True:
        dev = find_dev()
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            state["err"] = f"open fail {dev}"; time.sleep(3); continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W_FULL)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H_FULL)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        state["err"] = ""; state["dev"] = dev
        fail = 0
        while True:
            ok, f = cap.read()
            if not ok:
                fail += 1
                if fail > 30: break
                time.sleep(0.05); continue
            fail = 0
            f = rotate(f, state["rot"])
            h, w = f.shape[:2]
            small = cv2.resize(f, (W_STREAM, max(2, int(round(h * W_STREAM / w / 2)) * 2)))
            ok1, j1 = cv2.imencode(".jpg", f,     [cv2.IMWRITE_JPEG_QUALITY, 92])
            ok2, j2 = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok1 and ok2:
                with lock:
                    latest["full"] = j1.tobytes(); latest["small"] = j2.tobytes()
                    state["n"] += 1; state["t"] = time.time()
            time.sleep(max(0.0, 1.0 / FPS - 0.02))
        cap.release(); state["err"] = "read fail"; time.sleep(2)


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _j(self, code, body, ctype="application/json"):
        self.send_response(code); self.send_header("Content-Type", ctype)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store"); self.end_headers()
        try: self.wfile.write(body)
        except Exception: pass

    def do_GET(self):
        p = self.path
        if p.startswith("/health"):
            age = round(time.time() - state["t"], 1) if state["t"] else None
            self._j(200, ('{"frames":%d,"age":%s,"rot":%d,"dev":"%s","boot":%d,"err":"%s"}'
                          % (state["n"], age if age is not None else "null", state["rot"], state["dev"], BOOT, state["err"])).encode())
            return
        if p.startswith("/rot"):
            d = None
            if "d=" in p:
                try: d = int(p.split("d=")[1].split("&")[0])
                except Exception: d = None
            state["rot"] = (state["rot"] + 90) % 360 if d is None else d % 360
            self._j(200, ('{"rot":%d}' % state["rot"]).encode()); return
        if p.startswith("/full") or p.startswith("/snap"):
            with lock:
                j = latest["full"] if p.startswith("/full") else latest["small"]
            if not j: self._j(503, b'{"err":"no frame"}'); return
            self._j(200, j, "image/jpeg"); return
        if p.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store"); self.end_headers()
            last = 0
            try:
                while True:
                    with lock:
                        j, t = latest["small"], state["t"]
                    if j and t != last:
                        last = t
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(j))
                        self.wfile.write(j); self.wfile.write(b"\r\n")
                    else:
                        time.sleep(0.02)
            except Exception:
                return
        self._j(404, b'{"err":"see /stream /snap /full /health /rot"}')


if __name__ == "__main__":
    threading.Thread(target=pump, daemon=True).start()
    print(f"[newcam] {DEV} -> http://0.0.0.0:{PORT}/stream  rot={state['rot']}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
