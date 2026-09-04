#!/usr/bin/env python3
"""9/3 밑판 네모(외곽 사각형) 검출 — 뎁스로 밑판 마스크 → 신뢰 변(왼쪽·아래) 서브픽셀 → 오른쪽 변은 행별 전이 중앙값 → 4꼭짓점·yaw·중심.
  python3 base_box.py                 # 손목캠 라이브(밑판 관측 자세 base_view_tcp 에서)
  python3 base_box.py --teach         # 지금 값을 dot_calib.base_box_golden 으로 저장(밑판이 골든 자리에 있을 때만!)
  python3 base_box.py --pair c.jpg g.json [--save out.jpg]"""
import sys, json, math
import cv2, numpy as np
sys.path.insert(0, "/home/ar/bf2_console/tools")
import base_depth_corner as B
CAL = "/home/ar/bf2_console/dot_calib.json"


def measure(img, grid):
    f, why = B.base_frame(img, grid)
    if not f:
        return None, why
    r = f["rect"]; cs = r["corners"]; ed = r["edges"]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(float)
    c2 = np.array(cs[2]); c3 = np.array(cs[3]); v = (c3 - c2) / np.linalg.norm(c3 - c2); H = float(np.linalg.norm(c3 - c2))
    a = math.radians(ed[2]["ang"]); u = np.array([math.cos(a), math.sin(a)]); u = u if u[0] > 0 else -u
    offs = []
    for dy in list(range(30, 200, 4)) + list(range(300, 485, 4)):        # 9/3 19:00: 행 촘촘히(빨간 점 걸린 행은 중앙값 필터가 걸러냄)
        prof = []; ts = np.arange(250, 460, 0.5)
        for t in ts:
            x, y = c2 + u * t + v * dy; x0, y0 = int(x), int(y); fx, fy = x - x0, y - y0
            if not (0 <= x0 < gray.shape[1] - 1 and 0 <= y0 < gray.shape[0] - 1):
                prof = []; break
            prof.append(gray[y0, x0] * (1 - fx) * (1 - fy) + gray[y0, x0 + 1] * fx * (1 - fy) + gray[y0 + 1, x0] * (1 - fx) * fy + gray[y0 + 1, x0 + 1] * fx * fy)
        if not prof:
            continue
        prof = np.array(prof); lo, hi = prof.min(), prof.max()
        if hi - lo < 60:
            continue
        thr = (lo + hi) / 2; idx = np.where((prof[:-1] < thr) & (prof[1:] >= thr))[0]
        if len(idx) == 0:
            continue
        i = idx[0]; offs.append(ts[i] + 0.5 * (thr - prof[i]) / (prof[i + 1] - prof[i]))
    offs = np.array(offs)
    e1 = ed[1]
    if e1 and not e1.get("par") and e1["sigma"] <= 0.3 and e1["n"] >= 60 and cs[1]:     # 오른쪽 변 서브픽셀 직선이 깨끗하면 그것을 우선
        W = float((np.array(cs[1]) - c2) @ u); wsig = e1["sigma"]
    elif len(offs) >= 8:
        med = np.median(offs); keep = offs[np.abs(offs - med) < 3]
        if len(keep) < 6:
            keep = offs[np.abs(offs - med) < 8]
        W = float(np.median(keep)); wsig = float(keep.std())
    else:                                                               # 대체: detect_rect 의 오른쪽 변(꼭짓점 1) 을 u 방향으로 투영
        c1 = np.array(cs[1]) if cs[1] else None
        if c1 is None:
            return None, "오른쪽 변 샘플 부족(대체 불가)"
        W = float((c1 - c2) @ u); wsig = 9.9
    rect = [c2, c2 + u * W, c2 + u * W + v * H, c2 + v * H]
    ctr = np.mean(rect, axis=0)
    # ★yaw 정의 고정: 왼쪽 변(가장 길고 σ 최저) 각도 → 수직 기준 편차(+ = 시계). 아래 변 각도는 참고(원근으로 ~1° 다를 수 있음)
    yaw_left = float(((math.degrees(math.atan2(v[1], v[0])) + 90 + 45) % 90) - 45)
    yaw_bottom = float(((ed[2]["ang"] + 45) % 90) - 45)
    return {"rect": [(round(float(p[0]), 2), round(float(p[1]), 2)) for p in rect], "yaw": round(yaw_left, 3), "yaw_bottom": round(yaw_bottom, 3),
            "center": (round(float(ctr[0]), 2), round(float(ctr[1]), 2)), "H": round(H, 2), "W": round(W, 2),
            "sigma": {"left": ed[3]["sigma"], "bottom": ed[2]["sigma"], "right": round(wsig, 2)}, "plate_h": round(f["plate_h"], 1)}, None


def draw(img, m, path):
    out = img.copy(); rect = m["rect"]
    cv2.polylines(out, [np.array(rect, np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
    for i, p in enumerate(rect):
        cv2.circle(out, (int(p[0]), int(p[1])), 9, (0, 0, 255), 2); cv2.putText(out, str(i), (int(p[0]) + 10, int(p[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.imwrite(path, out)


if __name__ == "__main__":
    if "--pair" in sys.argv:
        i = sys.argv.index("--pair"); img = cv2.imread(sys.argv[i + 1]); grid = json.load(open(sys.argv[i + 2]))
    else:
        img, grid = B.grab_pair()
    m, why = measure(img, grid)
    if not m:
        print("실패:", why); sys.exit(1)
    cal = json.load(open(CAL)); g = cal.get("base_box_golden")
    line = f"네모 {m['rect']}  yaw(왼쪽변) {m['yaw']:+.3f}° (아래변 {m['yaw_bottom']:+.3f}°)  중심 {m['center']}  H {m['H']} W {m['W']}px  σ {m['sigma']}  밑판높이 {m['plate_h']}mm"
    if g:
        d = (m["center"][0] - g["center"][0], m["center"][1] - g["center"][1]); dy = m["yaw"] - g["yaw"]
        line += f"\n골든 대비: 중심 Δ({d[0]:+.1f},{d[1]:+.1f})px  yaw Δ{dy:+.3f}°"
    print(line)
    if "--teach" in sys.argv:
        import time
        cal["base_box_golden"] = {**m, "made": time.strftime("%Y-%m-%d %H:%M"), "tcp": cal.get("base_view_tcp")}
        json.dump(cal, open(CAL, "w"), ensure_ascii=False, indent=1); print("base_box_golden 저장")
    if "--save" in sys.argv:
        draw(img, m, sys.argv[sys.argv.index("--save") + 1])
