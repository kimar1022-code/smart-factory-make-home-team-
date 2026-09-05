#!/usr/bin/env python3
"""호버 정렬 뷰(:8775, 9/6) — 손목캠·새카메라·측면캠 3장에 hover_align 검출을 그려 보여준다(사용자 공동 진단용).
  흰 원 = 든 벽 점(기준 있으면 기준 자리 근처에서 찾은 것) · 초록 원 = 베이스 특징(기둥·안착벽 점) · 빨간 십자 = 기준이 말하는 '벽이 있어야 할 자리'
  http://<PC>:8775/            (색 바꾸기: /?color=yellow)      /wrist.jpg /newcam.jpg /side.jpg
  기준·매핑·로봇 상태가 있으면 각 카메라 Δ(mm/°)도 상단에 적는다. 없으면 검출만."""
import sys, time, json, threading, math
import urllib.request as UR
import urllib.parse as UP
import cv2, numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, "/home/ar/bf2_console/tools")
import hover_align as HA

PORT = 8775
state = {"color": "blue", "jpg": {}, "txt": {}}
COLS = {"blue": (255, 80, 0), "yellow": (0, 200, 255), "red": (0, 0, 255), "red_s": (0, 0, 255)}


def annotate(src, color):
    img = HA.grab(src)
    ref = HA.load_ref(color, src)
    w = HA.wall_dots(img, color, ref, None, src)
    f = HA.base_feats(img, w, src)
    out = img.copy(); r = 14 if src != "side" else 7
    for c, x, y, a in f:
        cv2.circle(out, (int(x), int(y)), r, (0, 255, 0), 2)
    for x, y, a in w:
        cv2.circle(out, (int(x), int(y)), r + 4, (255, 255, 255), 2)
    txt = f"[{src}] wall {len(w)} feats {len(f)}"
    if ref:
        try:
            t = HA.st()["tcp"]
            D, why = HA.delta(ref, {"wall": [(q[0], q[1], q[2]) for q in sorted(w, key=lambda q: (q[1], q[0]))], "pillars": f}, t[2], t[5], src) if w else (None, "벽 점 없음")
            if D:
                s_, d_, _ = HA.match_feats(ref["pillars"], f, HA.DET[src]["search"])
                for p in HA.apply_sim(HA.similarity(s_, d_), ref["wall"]):
                    cv2.drawMarker(out, (int(p[0]), int(p[1])), (0, 0, 255), cv2.MARKER_CROSS, 2 * r, 2)
                txt += f"  dXY ({D['dmm'][0]:+.2f},{D['dmm'][1]:+.2f})mm rz {D['drz']:+.2f}  feats{len(D['matched'])} rms{D['sim']['rms']:.1f}"
            else:
                txt += "  " + str(why)
        except Exception as e:
            txt += f"  (robot/ref: {e})"
    else:
        txt += "  ref none"
    cv2.rectangle(out, (0, 0), (out.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(out, txt, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
    return out, txt


def worker():
    while True:
        for src in ("wrist", "newcam", "side"):
            try:
                out, txt = annotate(src, state["color"])
                ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 80])
                state["jpg"][src] = buf.tobytes(); state["txt"][src] = txt
            except Exception as e:
                state["txt"][src] = f"[{src}] err {e}"
        time.sleep(0.4)


PAGE = """<!doctype html><html><head><meta charset=utf-8><title>hover view</title>
<style>body{background:#111;color:#eee;font:13px monospace;margin:6px}img{max-width:100%%;display:block}.g{display:grid;grid-template-columns:1fr 1fr;gap:6px}</style></head>
<body><div>hover_align view · color=<b>%s</b> · <a style="color:#8cf" href="/?color=blue">blue</a> <a style="color:#8cf" href="/?color=yellow">yellow</a> <a style="color:#8cf" href="/?color=red">red</a> <a style="color:#8cf" href="/?color=red_s">red_s</a></div>
<div class=g><div><img id=a src="/wrist.jpg"></div><div><img id=b src="/newcam.jpg"></div><div><img id=c src="/side.jpg"></div></div>
<script>setInterval(()=>{for(const i of ['a','b','c']){const e=document.getElementById(i);e.src=e.src.split('?')[0]+'?t='+Date.now();}},700);</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        u = UP.urlparse(self.path); q = UP.parse_qs(u.query)
        if u.path == "/":
            if "color" in q and q["color"][0] in COLS:
                state["color"] = q["color"][0]
            b = (PAGE % state["color"]).encode()
            self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.end_headers(); self.wfile.write(b); return
        if u.path == "/health":
            self.send_response(200); self.end_headers(); self.wfile.write(json.dumps({"ok": True, "color": state["color"], "txt": state["txt"]}).encode()); return
        src = u.path.strip("/").replace(".jpg", "")
        if src in state["jpg"]:
            self.send_response(200); self.send_header("Content-Type", "image/jpeg"); self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(state["jpg"][src]); return
        self.send_response(404); self.end_headers()


if __name__ == "__main__":
    threading.Thread(target=worker, daemon=True).start()
    print(f"hover_view → http://0.0.0.0:{PORT}/", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
