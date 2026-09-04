#!/usr/bin/env python3
"""자세별 회전 뎁스 뷰어 — 9/4. 로봇 자세에 따라 D435 뎁스 화면을 회전(화면만, 검출 무관).
  로봇이 rot_poses.json 의 어느 트리거 자세(관절 ±TOL) 안에 오면 그 rot 각도로 회전, 벗어나면 0.
  python3 depth_view_rot.py            # http://<PC>:8772/
트리거 정의: tools/rot_poses.json = {"check_front":{"joints":[...],"rot":90}, "check_right":{...,"rot":180}}
"""
import json, math, sys, time, threading, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2, numpy as np

CAM = "http://127.0.0.1:8766"      # D435 (rs)
BRIDGE = "http://127.0.0.1:8765"
POSES_F = "/home/ar/bf2_console/tools/rot_poses.json"
TOL = 8.0                           # 관절 매칭 허용(도) — 이 안이면 그 자세로 판정
PORT = 8772
state = {"jpg": None, "rot": 0, "pose": "정상", "t": 0}


def load_poses():
    try:
        return json.load(open(POSES_F))
    except Exception:
        return {}


def cur_joints():
    try:
        f = json.loads(urllib.request.urlopen(BRIDGE + "/status", timeout=3).read())["robots"]["fr5"]
        return f.get("joints")
    except Exception:
        return None


def match_pose(j, poses):
    if not j:
        return 0, "정상(자세 불명)"
    for name, p in poses.items():
        pj = p.get("joints")
        if pj and len(pj) == len(j) and all(abs(a - b) <= TOL for a, b in zip(j, pj)):
            return int(p.get("rot", 0)), name
    return 0, "정상"


def rotate(img, rot):
    if rot == 90:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if rot == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    if rot == 270:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img


def worker():
    last_pose_t = 0; rot, pose = 0, "정상"
    while True:
        try:
            # 자세는 0.4s 마다만 갱신(브리지 부담↓), 프레임은 빠르게
            if time.time() - last_pose_t > 0.4:
                rot, pose = match_pose(cur_joints(), load_poses()); last_pose_t = time.time()
            buf = urllib.request.urlopen(CAM + "/raw", timeout=3).read()
            img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                time.sleep(0.1); continue
            out = rotate(img, rot)
            cv2.putText(out, f"{pose} (rot {rot})", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            ok, jpg = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if ok:
                state.update(jpg=jpg.tobytes(), rot=rot, pose=pose, t=time.time())
        except Exception:
            time.sleep(0.2)
        time.sleep(0.06)


PAGE = b"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>\xed\x9a\x8c\xec\xa0\x84 \xeb\x8e\x81\xec\x8a\xa4 \xeb\xb7\xb0</title><style>body{margin:0;background:#111;color:#ddd;font-family:system-ui}img{width:100%;max-width:900px;display:block;margin:0 auto}</style></head>
<body><img src="/stream"></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/stream"):
            self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
            try:
                while True:
                    if state["jpg"]:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + state["jpg"] + b"\r\n")
                    time.sleep(0.06)
            except Exception:
                return
        if self.path.startswith("/status"):
            b = json.dumps({"rot": state["rot"], "pose": state["pose"]}).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b); return
        if self.path.startswith("/raw"):
            j = state["jpg"] or b""
            self.send_response(200); self.send_header("Content-Type", "image/jpeg"); self.send_header("Content-Length", str(len(j))); self.end_headers(); self.wfile.write(j); return
        self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.send_header("Content-Length", str(len(PAGE))); self.end_headers(); self.wfile.write(PAGE)


if __name__ == "__main__":
    threading.Thread(target=worker, daemon=True).start()
    time.sleep(1.0)
    print(f"depth_view_rot → http://0.0.0.0:{PORT}/  (트리거 {list(load_poses())})", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
