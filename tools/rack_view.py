#!/usr/bin/env python3
"""랙 관측 뷰 — 벽 양 끝점·중앙점 실시간 표시 (:8774, 9/5 사용자 요청).

랙 관측 중앙자세에서 각 색 벽의 **가장 먼 두 점(양 끝)** 과 그 **중앙**을 그린다.
사용자가 양끝 인식·중앙 파악이 맞는지 직접 보고, 잘못됐을 때 원인을 함께 확인하기 위함.
  · 색 점(작은 원) = 검출된 벽 색점 전부
  · 굵은 원 두 개 = 양 끝점 · X 표 = 중앙 · 잇는 선 = 벽 축
  · 상단: 색별 [끝점 수 / 중앙(x,y) / 길이px / 각°]

  python3 rack_view.py     # http://<PC>:8774/
"""
import sys, json, time, math, threading, urllib.request as UR
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2, numpy as np

sys.path.insert(0, "/home/ar/bf2_console/tools")
import place_calc as PC

CAM = "http://127.0.0.1:8766"
PORT = 8774
COL = {"blue": (255, 70, 0), "yellow": (0, 210, 255), "red": (0, 0, 255)}
COLORS = ["blue", "yellow", "red"]
state = {"jpg": None, "info": {}, "t": 0}


def wall_dots_all(hsv, color):
    lo, hi = PC.WALL_DOT_HSV[color]
    m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    pts = []
    for i in range(1, n):
        a = int(st[i, 4]); x, y = cen[i]
        if 40 < a < 3000 and 20 < x < 1260 and 20 < y < 700:
            pts.append((float(x), float(y), a))
    return pts


def worker():
    while True:
        try:
            buf = UR.urlopen(CAM + "/raw", timeout=3).read()
            img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                time.sleep(0.15); continue
            out = img.copy(); hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            info = {}
            for c in COLORS:
                pts = wall_dots_all(hsv, c)
                for p in pts:                              # 검출된 점 전부 작은 원
                    cv2.circle(out, (int(p[0]), int(p[1])), 5, COL[c], 1)
                walls = PC._cluster_walls([(p[0], p[1]) for p in pts])   # ★같은 색 여러 벽 분리(red_s/red)
                info[c] = {"n": len(pts), "walls": []}
                for g in walls:
                    e = max(((math.dist(g[i], g[j]), i, j) for i in range(len(g)) for j in range(i + 1, len(g))))
                    a, b = g[e[1]], g[e[2]]; mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                    L = e[0]; ang = math.degrees(math.atan2(b[0] - a[0], b[1] - a[1]))
                    cv2.line(out, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), COL[c], 1)
                    for p in (a, b):
                        cv2.circle(out, (int(p[0]), int(p[1])), 13, COL[c], 3)
                    cv2.drawMarker(out, (int(mid[0]), int(mid[1])), (255, 255, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
                    cv2.drawMarker(out, (int(mid[0]), int(mid[1])), COL[c], cv2.MARKER_TILTED_CROSS, 18, 1)
                    info[c]["walls"].append({"mid": (round(mid[0], 1), round(mid[1], 1)), "len": round(L), "ndots": len(g)})
            # 상단 바
            cv2.rectangle(out, (0, 0), (out.shape[1], 30), (0, 0, 0), -1)
            xoff = 8
            for c in COLORS:
                d = info.get(c, {}); ws = d.get("walls", [])
                if ws:
                    t = f"{c[0].upper()}:" + " ".join(f"({w['mid'][0]:.0f},{w['mid'][1]:.0f}){w['len']}px" for w in ws)
                    color = COL[c]
                else:
                    t = f"{c[0].upper()}:{d.get('n', 0)}pt"
                    color = (120, 120, 120)
                cv2.putText(out, t, (xoff, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1)
                xoff += 10 * len(t) + 8
            ok, jpg = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                state.update(jpg=jpg.tobytes(), info=info, t=time.time())
        except Exception:
            time.sleep(0.3)
        time.sleep(0.12)


PAGE = ("""<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>랙 관측 뷰</title>
<style>body{margin:0;background:#111;color:#ddd;font-family:system-ui}
img{width:100%;max-width:1100px;display:block;margin:0 auto}
p{max-width:1100px;margin:8px auto;font-size:13px;color:#9aa}</style></head>
<body><img src="/stream">
<p>굵은 원 = 벽 양 끝점 · 흰 X = 중앙점 · 선 = 벽 축 · 작은 원 = 검출된 색점 전부.
랙 관측 중앙자세에서 봐야 벽 전체(양 끝)가 들어옵니다. 중앙이 벽 정가운데면 그리퍼가 정중앙을 뭅니다.</p>
</body></html>""").encode()


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
                    time.sleep(0.12)
            except Exception:
                return
        if self.path.startswith("/status"):
            b = json.dumps({"info": state["info"], "t": state["t"]}).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b); return
        if self.path.startswith("/raw"):
            j = state["jpg"] or b""
            self.send_response(200); self.send_header("Content-Type", "image/jpeg"); self.send_header("Content-Length", str(len(j))); self.end_headers(); self.wfile.write(j); return
        self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.send_header("Content-Length", str(len(PAGE))); self.end_headers(); self.wfile.write(PAGE)


if __name__ == "__main__":
    threading.Thread(target=worker, daemon=True).start()
    time.sleep(1.2)
    print(f"rack_view → http://0.0.0.0:{PORT}/", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
