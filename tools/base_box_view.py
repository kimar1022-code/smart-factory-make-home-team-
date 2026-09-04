#!/usr/bin/env python3
"""9/3 밑판 네모 라이브 뷰어 — http://<PC>:8769/  (손목캠 컬러+뎁스 → base_box 측정 → 골든(노랑)·현재(초록) 오버레이, 1.5초 갱신)"""
import json, time, threading, math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2, numpy as np
import sys; sys.path.insert(0, "/home/ar/bf2_console/tools")
import base_depth_corner as B, base_box as X

latest = {"jpg": None, "txt": "대기", "t": 0}
import os
PTS_FILE = "/home/ar/bf2_console/base_user_pts.json"
user_pts = []   # 사용자가 클릭한 점(최대 4, 5번째 클릭은 처음부터) — 파일에 보존(뷰어 재기동에도 유지)
try:
    user_pts = [tuple(p) for p in json.load(open(PTS_FILE))]
except Exception:
    pass
def _save_pts():
    try: json.dump(user_pts, open(PTS_FILE, "w"))
    except Exception: pass
show = {"overlay": False, "user": True}   # 선·글 표시 토글 / 사용자 클릭점 표시


def worker():
    while True:
        try:
            img, grid = B.grab_pair()
            try:
                m, why = X.measure(img, grid)
            except Exception as ex:          # 랙 장면 등 밑판이 없으면 측정만 실패, 프레임은 계속
                m, why = None, f"no base ({type(ex).__name__})"
            # 표시용 화이트밸런스: 흰 책상 영역(밑판 왼쪽)의 채널 평균을 같게(그레이월드) — 검출에는 사용 안 함
            roi = img[150:600, 100:380].reshape(-1, 3).mean(0)
            gain = (roi.mean() / np.maximum(roi, 1)).reshape(1, 1, 3)
            out = np.clip(img.astype(np.float32) * gain, 0, 255).astype(np.uint8)
            cal = json.load(open(X.CAL)); g = cal.get("base_box_golden")
            if not show["overlay"]:
                # 선 끔 상태에서도 사용자 클릭점(자홍)은 항상 표시
                up0 = list(user_pts)
                for i, p_ in enumerate(up0):
                    cv2.circle(out, (int(p_[0]), int(p_[1])), 7, (255, 0, 255), 2)
                    cv2.putText(out, f"P{i+1}", (int(p_[0]) + 9, int(p_[1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                    if i >= 1:
                        cv2.line(out, (int(up0[i-1][0]), int(up0[i-1][1])), (int(p_[0]), int(p_[1])), (255, 0, 255), 2)
                if len(up0) == 4:
                    cv2.line(out, (int(up0[3][0]), int(up0[3][1])), (int(up0[0][0]), int(up0[0][1])), (255, 0, 255), 2)
                ok, jpg = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 92])
                latest["jpg"] = jpg.tobytes(); latest["txt"] = "overlay off" + (f" · pts {len(up0)}" if up0 else ""); latest["t"] = time.time()
                time.sleep(0.3); continue
            if g and m:      # 골든 네모는 밑판이 검출된 장면에서만
                cv2.polylines(out, [np.array(g["rect"], np.int32).reshape(-1, 1, 2)], True, (0, 220, 255), 2)
                cv2.putText(out, "golden", (int(g["rect"][3][0]), int(g["rect"][3][1]) - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)
            if m:
                # ★STL 슬롯 모델 투영: 현재 네모(4점) ↔ 판 mm 호모그래피, 기둥 꼭대기는 원근 배율(밑판 뎁스/꼭대기 뎁스)
                try:
                    bm = cal.get("base_model")
                    if bm:
                        Pm = np.array([[-199, 90], [-199, 230], [11, 230], [11, 90]], np.float32)
                        Hm = cv2.getPerspectiveTransform(Pm, np.array(m["rect"], np.float32))
                        dd = np.array([q["d"] for q in grid["pts"] if q["d"] and 100 < q["d"] < 1000]); Zp = np.percentile(dd, 8); kk = Zp / (Zp - 80.0)
                        cc = np.array([640.0, 360.0])
                        def tp(x, y):
                            v = Hm @ np.array([x, y, 1.0]); q = np.array([v[0] / v[2], v[1] / v[2]]); q = cc + (q - cc) * kk; return (int(q[0]), int(q[1]))
                        pass
                        pass   # STL 주점 투영 사각형/홈은 20px 오차라 표시 안 함(검출 기둥 꼭대기로 대체)
                except Exception as ex:
                    cv2.putText(out, f"slot overlay err {ex}", (300, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                # ★흰 점(기둥 꼭대기)·빨간 점(밑판 귀퉁이) 검출
                try:
                    bd = B.base_dots(img, m["rect"])
                    pass   # 흰 점 원 표시 끔(흰 점은 검정 꼭짓점 찾기용 보조 — 사용자 지시)
                    for x, y, a in bd["red"]:
                        cv2.circle(out, (int(x), int(y)), 10, (0, 0, 255), 2)
                    wp = sorted(bd["white"], key=lambda t: (t[1] > 360, t[0]))     # 위 2개(x순), 아래 2개(x순)
                    if len(wp) == 4:
                        order = [wp[2], wp[3], wp[1], wp[0]]   # 좌하, 우하, 우상, 좌상
                        oc = B.pillar_outer_corners(img, order, m["rect"])          # ★흰 점 너머 검정 꼭대기 바깥 꼭짓점
                        for i in range(4):
                            a_, b_ = oc[i], oc[(i + 1) % 4]
                            if a_ and b_:
                                cv2.line(out, (int(a_[0]), int(a_[1])), (int(b_[0]), int(b_[1])), (200, 0, 200), 1)
                            if a_:
                                cv2.circle(out, (int(a_[0]), int(a_[1])), 6, (255, 0, 255), 2)
                        show["outer"] = [None if not c_ else (round(c_[0], 1), round(c_[1], 1)) for c_ in oc]
                        sl = B.slot_lines_from_outer(oc)          # ★STL 홈 오프셋 → 슬롯 선(빨강)·홈 중심(빨간 점)
                        if sl:
                            for k_, (a_, b_) in sl["slots"].items():
                                cv2.line(out, (int(a_[0]), int(a_[1])), (int(b_[0]), int(b_[1])), (0, 0, 255), 2)
                            for i_, g_ in sl["grooves"].items():
                                for kk in ("long", "short"):
                                    cv2.circle(out, (int(g_[kk][0]), int(g_[kk][1])), 4, (0, 0, 255), -1)
                    show["dots"] = {"white": [(round(x, 1), round(y, 1)) for x, y, a in bd["white"]], "red": [(round(x, 1), round(y, 1)) for x, y, a in bd["red"]]}
                except Exception as ex:
                    cv2.putText(out, f"dots err {ex}", (300, 660), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                # ★검출 기둥 꼭대기(색 없이) + 그 사이 슬롯 선(회색)
                try:
                    tops = B.pillar_tops_color(img, m["rect"])
                    pass   # 청록 막대끝 사각형 표시 끔(흰 점 기준으로 대체)
                    for a, b in ():     # (회색 막대끝 슬롯선은 끔 — 흰 점 사각형(자홍)으로 대체)
                        if tops[a] and tops[b]:
                            pa, pb = tops[a]["center"], tops[b]["center"]
                            cv2.line(out, (int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1])), (160, 160, 160), 1)
                    show["tops"] = [None if not t else (round(t["center"][0], 1), round(t["center"][1], 1)) for t in tops]
                except Exception as ex:
                    cv2.putText(out, f"tops err {ex}", (300, 680), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.polylines(out, [np.array(m["rect"], np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
                for i, p in enumerate(m["rect"]):
                    cv2.circle(out, (int(p[0]), int(p[1])), 8, (0, 0, 255), 2)
                c = m["center"]; cv2.drawMarker(out, (int(c[0]), int(c[1])), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
                txt = f"yaw {m['yaw']:+.2f}deg  center ({c[0]:.1f},{c[1]:.1f})  H {m['H']:.0f} W {m['W']:.0f}px"
                if g:
                    d = (c[0] - g["center"][0], c[1] - g["center"][1]); dy = m["yaw"] - g["yaw"]
                    txt += f"  |  vs golden: d({d[0]:+.1f},{d[1]:+.1f})px ~({d[0]*0.4:+.1f},{d[1]*0.4:+.1f})mm  dyaw {dy:+.2f}deg"
            else:
                txt = f"detect fail: {why}"
                # ★슬롯 장면(밑판 전체가 안 보일 때): 기둥 꼭대기 2개 → 바깥 꼭짓점(자홍)·홈 중심선(빨강)·판 가장자리(초록)
                try:
                    sl2, why2 = B.slot_from_two_pillars(img, long_mm=140.0)     # ★보이는 두 기둥 = 짧은 변(140)
                    bl_, _w = B.blue_slot_line(img, list(user_pts))
                    if bl_:
                        cv2.line(out, (int(bl_["g0"][0]), int(bl_["g0"][1])), (int(bl_["g1"][0]), int(bl_["g1"][1])), (0, 0, 255), 2)
                        cv2.circle(out, (int(bl_["g0"][0]), int(bl_["g0"][1])), 6, (0, 0, 255), -1)
                        cv2.putText(out, "blue slot groove", (int(bl_["g0"][0]) + 8, int(bl_["g0"][1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                    if sl2:
                        for p_ in sl2["outer"]:
                            cv2.circle(out, (int(p_[0]), int(p_[1])), 7, (255, 0, 255), 2)
                        cv2.line(out, (int(sl2["outer"][0][0]), int(sl2["outer"][0][1])), (int(sl2["outer"][1][0]), int(sl2["outer"][1][1])), (255, 0, 255), 1)
                        g0_, g1_ = sl2["grooves"]
                        cv2.line(out, (int(g0_[0]), int(g0_[1])), (int(g1_[0]), int(g1_[1])), (0, 0, 255), 2)
                        for p_ in sl2["grooves"]:
                            cv2.circle(out, (int(p_[0]), int(p_[1])), 5, (0, 0, 255), -1)
                        e_ = B.plate_edge_near_slot(img, sl2)
                        if e_ and abs(e_["sigma"]) < 3.0:
                            cv2.line(out, (int(e_["p"][0]), int(e_["p"][1])), (int(e_["q"][0]), int(e_["q"][1])), (0, 255, 0), 2)
                        # 사용자 클릭 선 정제(초록 가는 선)
                        upl = list(user_pts)
                        if len(upl) >= 2:
                            for l_ in B.refine_user_lines(img, upl):
                                if l_ and l_["sigma"] < 2.0:
                                    cv2.line(out, (int(l_["p"][0]), int(l_["p"][1])), (int(l_["q"][0]), int(l_["q"][1])), (0, 200, 0), 1)
                        txt = f"short slot {sl2['slot_len_px']/sl2['scale']:.1f}mm ang {sl2['ang']:+.2f}deg scale {sl2['scale']:.2f}px/mm" + (f" | BLUE groove ang {bl_['ang']:+.2f}" if bl_ else "") + (f" | plate edge {e_['offset_px']/sl2['scale']:+.1f}mm sig {e_['sigma']:.2f}" if e_ else "")
                        show["slot"] = {"grooves": [(round(a, 1), round(b, 1)) for a, b in sl2["grooves"]], "outer": [(round(a, 1), round(b, 1)) for a, b in sl2["outer"]], "ang": round(sl2["ang"], 2)}
                    else:
                        txt += f" / slot: {why2}"
                except Exception as ex:
                    txt += f" / slot err {ex}"
            # 사용자 클릭점(자홍) + 선
            up = list(user_pts) if show.get("user") else []
            for i, p in enumerate(up):
                cv2.circle(out, (int(p[0]), int(p[1])), 7, (255, 0, 255), 2)
                cv2.putText(out, f"P{i+1}", (int(p[0]) + 9, int(p[1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                if i >= 1:
                    cv2.line(out, (int(up[i-1][0]), int(up[i-1][1])), (int(p[0]), int(p[1])), (255, 0, 255), 2)
            if len(up) == 4:
                cv2.line(out, (int(up[3][0]), int(up[3][1])), (int(up[0][0]), int(up[0][1])), (255, 0, 255), 2)
            # 왼쪽 세로 글상자
            lines = []
            if m:
                lines += [f"yaw {m['yaw']:+.2f} deg", f"center ({c[0]:.1f},{c[1]:.1f})", f"H {m['H']:.0f}  W {m['W']:.0f} px"]
                if g:
                    lines += [f"vs golden", f" d ({d[0]:+.1f},{d[1]:+.1f}) px", f" ~({d[0]*0.4:+.1f},{d[1]*0.4:+.1f}) mm", f" dyaw {dy:+.2f} deg"]
            else:
                if show.get("slot"):
                    sl_ = show["slot"]; lines += ["SLOT view", f" groove ang {sl_['ang']:+.2f}", " grooves:"] + [f"  {g_}" for g_ in sl_["grooves"]] + [" pillar outer:"] + [f"  {o_}" for o_ in sl_["outer"]]
                else:
                    lines += ["detect fail", str(why)[:22]]
            if show.get("dots"):
                lines += ["pillar outer:"] + [f" {t}" for t in show.get("outer", [])] + ["red(plate):"] + [f" {t}" for t in show["dots"]["red"][:5]]
            if up:
                lines += ["click pts:"] + [f" P{i+1} ({p[0]:.0f},{p[1]:.0f})" for i, p in enumerate(up)]
            hbox = 22 * len(lines) + 12
            cv2.rectangle(out, (0, 0), (250, hbox), (0, 0, 0), -1)
            for i, l in enumerate(lines):
                cv2.putText(out, l, (6, 20 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
            ok, jpg = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 80])
            latest["jpg"] = jpg.tobytes(); latest["txt"] = txt; latest["t"] = time.time()
        except Exception as e:
            import traceback; traceback.print_exc()
            latest["txt"] = f"err {e}"
        time.sleep(0.3)


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path.startswith("/frame.jpg"):
            b = latest["jpg"]
            if b is None:
                self.send_response(503); self.end_headers(); return
            self.send_response(200); self.send_header("Content-Type", "image/jpeg"); self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(b); return
        if self.path.startswith("/click"):
            from urllib.parse import urlparse, parse_qs
            q = parse_qs(urlparse(self.path).query)
            try:
                x, y = float(q["x"][0]), float(q["y"][0])
                if len(user_pts) >= 4:
                    user_pts.clear()
                user_pts.append((x, y)); _save_pts()
            except Exception:
                pass
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
            self.wfile.write(json.dumps({"pts": user_pts}).encode()); return
        if self.path.startswith("/overlay"):
            from urllib.parse import urlparse, parse_qs
            q = parse_qs(urlparse(self.path).query); show["overlay"] = q.get("on", ["1"])[0] == "1"
            self.send_response(200); self.end_headers(); self.wfile.write(str(show["overlay"]).encode()); return
        if self.path.startswith("/user"):
            from urllib.parse import urlparse, parse_qs
            q = parse_qs(urlparse(self.path).query); show["user"] = q.get("on", ["1"])[0] == "1"
            self.send_response(200); self.end_headers(); self.wfile.write(str(show["user"]).encode()); return
        if self.path.startswith("/clear"):
            user_pts.clear(); _save_pts(); self.send_response(200); self.end_headers(); self.wfile.write(b"ok"); return
        if self.path.startswith("/teach_user"):
            if len(user_pts) == 4:
                cal = json.load(open(X.CAL)); c = np.mean(np.array(user_pts), axis=0)
                cal["base_box_user"] = {"rect": [list(p) for p in user_pts], "center": [float(c[0]), float(c[1])], "made": time.strftime("%Y-%m-%d %H:%M"), "tcp": cal.get("base_view_tcp")}
                json.dump(cal, open(X.CAL, "w"), ensure_ascii=False, indent=1)
                self.send_response(200); self.end_headers(); self.wfile.write("base_box_user saved".encode()); return
            self.send_response(400); self.end_headers(); self.wfile.write("need 4 pts".encode()); return
        if self.path.startswith("/status"):
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
            self.wfile.write(json.dumps({"txt": latest["txt"], "age": round(time.time() - latest["t"], 1)}, ensure_ascii=False).encode()); return
        html = """<html><head><meta charset='utf-8'><title>밑판 네모</title><style>body{background:#111;color:#eee;font-family:sans-serif;margin:8px}img{max-width:100%;cursor:crosshair}button{margin:4px;padding:6px 12px}</style></head>
<body><div>밑판 네모 뷰어 — 노랑=골든, 초록=현재, 빨강=꼭짓점, <b style='color:#f0f'>자홍=내가 찍은 점(이미지 클릭, 4점까지)</b>
 <button onclick="fetch('/overlay?on=0')">선 끄기</button> <button onclick="fetch('/overlay?on=1')">선 켜기</button> <button onclick="fetch('/user?on=1')">내 점 보기</button> <button onclick="fetch('/user?on=0')">내 점 숨기기</button> <button onclick="fetch('/clear')">점 지우기</button> <button onclick="fetch('/teach_user').then(r=>r.text()).then(t=>alert(t))">내 점 4개를 골든으로 저장</button></div>
<img id=i src='/frame.jpg'>
<script>
const im=document.getElementById('i');
im.addEventListener('click',e=>{const r=im.getBoundingClientRect();const x=(e.clientX-r.left)*1280/r.width;const y=(e.clientY-r.top)*720/r.height;fetch('/click?x='+x.toFixed(1)+'&y='+y.toFixed(1));});
setInterval(()=>{im.src='/frame.jpg?'+Date.now();},1000);
</script></body></html>"""
        self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.end_headers(); self.wfile.write(html.encode())


if __name__ == "__main__":
    threading.Thread(target=worker, daemon=True).start()
    ThreadingHTTPServer(("0.0.0.0", 8769), H).serve_forever()
