#!/usr/bin/env python3
"""9/3 저녁 — 'edge' 앵커 (마커·색점 없이 삽입). 사용자 원칙: 호버에서 밑판 슬롯 가장자리 선 yaw = 벽 선 yaw.
손목캠(cam1) 한 프레임에서 4개를 서브픽셀로 잰다.
  ① 밑판 파랑쪽 세로 가장자리(골든 씨앗 근처, 밝→어둡)      → base_ang, base_x
  ② 벽 밑변(밑판 바깥 회색 책상 → 밝은 벽, 행 스캔)          → wall_ang, wall_x
  ③ 밑판 윗변(짧은 변, 밝은 책상 → 검정 밑판, 열 스캔)        → plate_y
  ④ 벽 끝(벽 밑변 선을 따라 위로 가다 밝기 떨어지는 곳)        → wall_end_y
파생: dyaw = wall − base (수직 기준 편차 차, °), gap = wall_x − base_x (px, y_ref 에서), end = wall_end_y − plate_y (px)
벽은 카메라와 같이 움직이므로 gap/end 변화 = 밑판 선의 화면 이동 = 로봇 XY 이동 → 자코비안(px/mm) 으로 환산.
  python3 edge_place.py blue [이미지] [--save out.jpg]
"""
import sys, json, math, urllib.request
import cv2, numpy as np
sys.path.insert(0, "/home/ar/bf2_console/tools")
from edge_anchor import _subpix_line
CAL = "/home/ar/bf2_console/dot_calib.json"


def grab(port=8766):
    buf = urllib.request.urlopen(f"http://127.0.0.1:{port}/raw", timeout=5).read()
    return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)


def _vdev(ang):
    """직선 각(°) → 수직 기준 편차(−90~90). 89.6 → −0.4, −90 → 0, 90 → 0"""
    return ((ang - 90.0) + 90.0) % 180.0 - 90.0


def _fit(P):
    P = np.array(P, np.float32)
    vx, vy, x0, y0 = cv2.fitLine(P, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    d = np.abs((P[:, 0] - x0) * vy - (P[:, 1] - y0) * vx)
    keep = d < max(1.0, 2.5 * np.median(d) + 1.0)
    if keep.sum() >= 12:
        vx, vy, x0, y0 = cv2.fitLine(P[keep], cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        d = np.abs((P[keep][:, 0] - x0) * vy - (P[keep][:, 1] - y0) * vx)
    return float(vx), float(vy), float(x0), float(y0), float(d.std()), int(keep.sum())


def _x_at(line, y):
    vx, vy, x0, y0 = line["v"][0], line["v"][1], line["x0"], line["y0"]
    return x0 + vx * (y - y0) / vy if abs(vy) > 1e-6 else x0


def _y_at(line, x):
    vx, vy, x0, y0 = line["v"][0], line["v"][1], line["x0"], line["y0"]
    return y0 + vy * (x - x0) / vx if abs(vx) > 1e-6 else y0


def base_side_line(gray, seed, offsets=range(-72, 73, 8), band=8):
    """골든 씨앗(파랑 쪽 밑판 세로선) 근처에서 서브픽셀 직선 후보 → 씨앗에 가장 가까운 '밝→어둡(+쪽 어둡)' 선."""
    x1, y1, x2, y2 = seed; ux, uy = x2 - x1, y2 - y1; L = math.hypot(ux, uy); ux, uy = ux / L, uy / L; nx, ny = -uy, ux
    best = None
    for off in offsets:
        sd = (x1 + nx * off, y1 + ny * off, x2 + nx * off, y2 + ny * off)
        r = _subpix_line(gray, sd, band=band, step=2)
        if r is None or r[1] > 0.6 or r[2] < 60:
            continue
        ang, sig, n, (vx, vy, x0, y0) = r
        px, py = int(x0 + nx * 6), int(y0 + ny * 6); mx, my = int(x0 - nx * 6), int(y0 - ny * 6)
        if not (0 <= py < gray.shape[0] and 0 <= px < gray.shape[1] and 0 <= my < gray.shape[0] and 0 <= mx < gray.shape[1]):
            continue
        bp, bm = float(gray[py, px]), float(gray[my, mx])
        if bp > 60 or bm < bp + 30:          # + 쪽(밑판)이 검정이어야
            continue
        d = abs((x0 - x1) * nx + (y0 - y1) * ny)
        if best is None or d < best["off"]:
            best = {"ang": float(ang), "sigma": float(sig), "n": int(n), "x0": float(x0), "y0": float(y0), "v": (float(vx), float(vy)), "off": float(d)}
    return best


def wall_edge_line(gray, x_ref, y0, y1, half=30, step=3, rows_avg=5, min_contrast=8.0):
    """벽 밑변(세로): 행 밴드(rows_avg 행 평균)마다 [x_ref−half, x_ref+half] 프로파일(5탭 평활),
    왼쪽(책상 회색)·오른쪽(벽 밝음) 중앙값 50% 교차(서브픽셀). 저녁 저대비(10레벨)도 잡도록 행 평균·문턱 8."""
    pts = []; gf = gray.astype(float); h = rows_avg // 2; k = np.ones(5) / 5.0
    xa, xb = int(x_ref - half), int(x_ref + half)
    if xa < 0 or xb >= gray.shape[1]:
        return None
    for y in range(int(y0), int(y1), step):
        if y - h < 0 or y + h >= gray.shape[0]:
            continue
        prof = gf[y - h:y + h + 1, xa:xb].mean(0)
        prof = np.convolve(prof, k, mode="same")
        lo = float(np.median(prof[3:16])); hi = float(np.median(prof[-16:-3]))
        if hi - lo < min_contrast or lo < 30:      # 왼쪽이 밑판(검정)이면 벽 밑변이 아니라 밑판 변 — 제외
            continue
        thr = (lo + hi) / 2
        idx = np.where((prof[:-1] < thr) & (prof[1:] >= thr))[0]
        if len(idx) == 0:
            continue
        # 여러 교차면 기울기가 가장 큰(진짜 에지) 것
        i = int(max(idx, key=lambda j: prof[j + 1] - prof[j]))
        f = (thr - prof[i]) / max(1e-6, prof[i + 1] - prof[i])
        pts.append((xa + i + f, float(y)))
    if len(pts) < 12:
        return None
    vx, vy, x0, y0_, sig, n = _fit(pts)
    if vy < 0:
        vx, vy = -vx, -vy
    return {"ang": math.degrees(math.atan2(vy, vx)), "sigma": sig, "n": n, "x0": x0, "y0": y0_, "v": (vx, vy)}


def plate_top_edge(gray, cols, y0, y1):
    """밑판 윗변(짧은 변, 가로): 열마다 위→아래 프로파일에서 검정(<40) 20px 연속 구간 시작 직전 50% 교차."""
    pts = []
    for x in cols:
        x = int(x)
        if x < 0 or x >= gray.shape[1]:
            continue
        prof = gray[int(y0):int(y1), x].astype(float)
        dark = prof < 40; run = 0; i_start = None
        for i, dk in enumerate(dark):
            run = run + 1 if dk else 0
            if run >= 20:
                i_start = i - run + 1; break
        if i_start is None or i_start < 12:
            continue
        hi = float(np.median(prof[max(0, i_start - 40):max(1, i_start - 22)]))   # 검정 직전 회색 띠(가이드·측면) 앞의 밝은 책상
        lo = float(np.median(prof[i_start:i_start + 10])); thr = (hi + lo) / 2
        j = i_start
        while j > 0 and prof[j - 1] < thr:
            j -= 1
        if j == 0:
            continue
        den = prof[j] - prof[j - 1]; f = (thr - prof[j - 1]) / den if abs(den) > 1e-3 else 0.5
        pts.append((float(x), int(y0) + (j - 1) + min(1.0, max(0.0, float(f)))))
    if len(pts) < 8:
        return None
    vx, vy, x0, y0_, sig, n = _fit(pts)
    if vx < 0:
        vx, vy = -vx, -vy
    return {"ang": math.degrees(math.atan2(vy, vx)), "sigma": sig, "n": n, "x0": x0, "y0": y0_, "v": (vx, vy)}


def wall_end_y(gray, wall, y_guess, span=70, dx=(5, 9, 13, 17, 21)):
    """벽 끝: 벽 밑변 선 안쪽(벽 쪽) 몇 px 열에서 위→아래 회색(책상)→밝음(벽) 50% 교차. 끝면이 비스듬해 열마다 y 가 다르므로
    (dx, y) 직선 맞춤 후 dx=0(밑변 교점) 으로 외삽. 반환 (y_end, 잔차σ)"""
    pts = []
    for d in dx:
        x = int(round(_x_at(wall, y_guess) + d))
        ya, yb = int(y_guess - span), int(y_guess + span)
        if x < 0 or x >= gray.shape[1] or ya < 0 or yb >= gray.shape[0]:
            continue
        prof = gray[ya:yb, x].astype(float)
        lo = float(np.median(prof[:20])); hi = float(np.median(prof[-20:]))
        if hi - lo < 8:
            continue
        thr = (lo + hi) / 2
        idx = np.where((prof[:-1] < thr) & (prof[1:] >= thr))[0]
        if len(idx) == 0:
            continue
        i = int(idx[0]); f = (thr - prof[i]) / max(1e-6, prof[i + 1] - prof[i])
        pts.append((float(d), ya + i + f))
    if len(pts) < 3:
        return None
    P = np.array(pts); a, b = np.polyfit(P[:, 0], P[:, 1], 1)
    res = P[:, 1] - (a * P[:, 0] + b)
    return float(b), float(res.std())


def measure(img, cfg):
    """cfg: {"seed":[x1,y1,x2,y2], "wall_x":px, "wall_rows":[y0,y1], "plate_cols":[x0,x1,step], "plate_rows":[y0,y1], "end_y":px, "y_ref":px}
    반환 dict(측정) 또는 (None, why)"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    b = base_side_line(gray, cfg["seed"])
    if not b:
        return None, "밑판 파랑쪽 선 없음"
    w = wall_edge_line(gray, cfg["wall_x"], *cfg["wall_rows"])
    if not w:
        return None, "벽 밑변 없음"
    y_ref = cfg["y_ref"]
    out = {"base_ang": _vdev(b["ang"]), "base_x": _x_at(b, y_ref), "base_sig": b["sigma"], "base_n": b["n"],
           "wall_ang": _vdev(w["ang"]), "wall_x": _x_at(w, y_ref), "wall_sig": w["sigma"], "wall_n": w["n"]}
    out["dyaw"] = out["wall_ang"] - out["base_ang"]
    out["gap"] = out["wall_x"] - out["base_x"]
    pc = cfg.get("plate_cols"); pr = cfg.get("plate_rows")
    p = plate_top_edge(gray, range(pc[0], pc[1], pc[2]), *pr) if pc and pr else None
    if p:
        out["plate_y"] = _y_at(p, out["wall_x"]); out["plate_ang"] = p["ang"]; out["plate_sig"] = p["sigma"]
    e = wall_end_y(gray, w, cfg.get("end_y", 300)) if cfg.get("end_y") else None
    if e:
        out["wall_end_y"] = e[0]; out["end_sig"] = e[1]
    if p and e:
        out["end"] = out["wall_end_y"] - out["plate_y"]
    out["_lines"] = {"base": b, "wall": w, "plate": p}
    return out, None


def draw(img, m, cfg):
    out = img.copy(); L = m["_lines"]
    for k, col in (("base", (0, 255, 0)), ("wall", (0, 0, 255))):
        l = L[k]; vx, vy = l["v"]; S = 420
        cv2.line(out, (int(l["x0"] - vx * S / 2), int(l["y0"] - vy * S / 2)), (int(l["x0"] + vx * S / 2), int(l["y0"] + vy * S / 2)), col, 2)
    if L.get("plate"):
        l = L["plate"]; vx, vy = l["v"]; S = 600
        cv2.line(out, (int(l["x0"] - vx * S / 2), int(l["y0"] - vy * S / 2)), (int(l["x0"] + vx * S / 2), int(l["y0"] + vy * S / 2)), (255, 200, 0), 2)
    if "wall_end_y" in m:
        cv2.circle(out, (int(m["wall_x"]), int(m["wall_end_y"])), 7, (255, 0, 255), 2)
    cv2.putText(out, f"dyaw {m['dyaw']:+.2f}  gap {m['gap']:+.1f}px  end {m.get('end', float('nan')):+.1f}px", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return out


def default_cfg(cal, ck):
    sg = cal["slot_golden"][ck]
    eg = sg.get("edge_cfg")
    if eg:
        return eg
    bl = sg["base_lines_z478"]["refined"][1]; hr = sg["hover_ref_z478"]
    return {"seed": [bl["p"][0], bl["p"][1], bl["q"][0], bl["q"][1]], "wall_x": hr["wall_line"]["x0"], "wall_rows": [335, 545],
            "plate_cols": [450, 720, 10], "plate_rows": [230, 360], "end_y": 300, "y_ref": 440}


# ═══ cam2(새카메라, +Y 끝) — 벽을 옆에서 봐 벽 면이 40px 띠로 보이고 밑판 변은 벽 밑에 가려짐 → 기준 = 기둥 꼭대기 흰 점
def white_dot_near(img, guess, r=90):
    """골든 근처(반경 r px) 흰 점(기둥 꼭대기) 가중중심. 탑햇으로 밝은 책상 제거, 주변에 검정(기둥) 있어야."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY); hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    x0, y0 = int(guess[0]), int(guess[1]); xa, ya = max(0, x0 - r), max(0, y0 - r)
    sub = gray[ya:y0 + r, xa:x0 + r]; hs = hsv[ya:y0 + r, xa:x0 + r]
    if sub.size == 0:
        return None
    th = cv2.morphologyEx(sub, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (35, 35)))
    wm = ((th > 70) & (hs[..., 1] < 130)).astype(np.uint8) * 255; wm = cv2.morphologyEx(wm, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(wm)
    best = None
    for i in range(1, n):
        a = st[i, 4]; w, h = st[i, 2], st[i, 3]
        if not (80 < a < 3000 and 0.5 < w / max(1, h) < 2.0):
            continue
        cx, cy = int(cen[i][0]), int(cen[i][1]); ring = sub[max(0, cy - 25):cy + 26, max(0, cx - 25):cx + 26]
        if ring.size == 0 or np.percentile(ring, 20) > 90:
            continue
        ys_, xs_ = np.nonzero(lab == i); wts = sub[ys_, xs_].astype(float)
        px, py = float((xs_ * wts).sum() / wts.sum()) + xa, float((ys_ * wts).sum() / wts.sum()) + ya
        d = math.hypot(px - guess[0], py - guess[1])
        if best is None or d < best[2]:
            best = (px, py, d, int(a))
    return best


def wall_strip_left_edge(gray, x_ref, y0, y1, half=30, step=3):
    """cam2 벽 띠 왼쪽 변(검정 밑판 → 밝은 벽): 행 스캔 50% 교차. 왼쪽이 검정(<60)·오른쪽 밝음(>150) 인 행만."""
    pts = []
    for y in range(int(y0), int(y1), step):
        xa, xb = int(x_ref - half), int(x_ref + half)
        if xa < 0 or xb >= gray.shape[1] or y < 0 or y >= gray.shape[0]:
            continue
        prof = gray[y, xa:xb].astype(float)
        lo = float(np.median(prof[:12])); hi = float(np.median(prof[-12:]))
        if lo > 60 or hi < 150:
            continue
        thr = (lo + hi) / 2
        idx = np.where((prof[:-1] < thr) & (prof[1:] >= thr))[0]
        if len(idx) == 0:
            continue
        i = int(idx[0]); f = (thr - prof[i]) / max(1e-6, prof[i + 1] - prof[i])
        pts.append((xa + i + f, float(y)))
    if len(pts) < 12:
        return None
    vx, vy, x0, y0_, sig, n = _fit(pts)
    if vy < 0:
        vx, vy = -vx, -vy
    return {"ang": math.degrees(math.atan2(vy, vx)), "sigma": sig, "n": n, "x0": x0, "y0": y0_, "v": (vx, vy)}


def strip_end_y(gray, wall, y_guess, span=60, dx=(6, 10, 14, 18)):
    """cam2 벽 끝: 띠 안쪽 열에서 위→아래 밝음(벽)→어둠(기둥/밑판) 50% 교차, (dx,y) 직선 → dx=0 외삽."""
    pts = []
    for d in dx:
        x = int(round(_x_at(wall, y_guess) + d)); ya, yb = int(y_guess - span), int(y_guess + span)
        if x < 0 or x >= gray.shape[1] or ya < 0 or yb >= gray.shape[0]:
            continue
        prof = gray[ya:yb, x].astype(float)
        hi = float(np.median(prof[:16])); lo = float(np.median(prof[-16:]))
        if hi - lo < 40:
            continue
        thr = (hi + lo) / 2
        idx = np.where((prof[:-1] >= thr) & (prof[1:] < thr))[0]
        if len(idx) == 0:
            continue
        i = int(idx[0]); f = (prof[i] - thr) / max(1e-6, prof[i] - prof[i + 1])
        pts.append((float(d), ya + i + f))
    if len(pts) < 3:
        return None
    P = np.array(pts); a, b = np.polyfit(P[:, 0], P[:, 1], 1)
    return float(b), float((P[:, 1] - (a * P[:, 0] + b)).std())


def measure_cam2(img, cfg):
    """cfg: {"dot":[x,y], "wall_x":px, "wall_rows":[y0,y1], "end_y":px, "y_ref":px}. 반환 dict 또는 (None, why)
    gap2 = wall_x(y_ref) − dot_x, end2 = wall_end_y − dot_y, wall_ang2 = 수직 편차"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    w = wall_strip_left_edge(gray, cfg["wall_x"], *cfg["wall_rows"])
    if not w:
        return None, "cam2 벽 띠 변 없음"
    d = white_dot_near(img, cfg["dot"])
    if not d:
        return None, "cam2 기둥 흰 점 없음"
    y_ref = cfg["y_ref"]
    out = {"wall_ang": _vdev(w["ang"]), "wall_x": _x_at(w, y_ref), "wall_sig": w["sigma"], "wall_n": w["n"],
           "dot_x": d[0], "dot_y": d[1], "dot_area": d[3]}
    out["gap"] = out["wall_x"] - d[0]
    e = strip_end_y(gray, w, cfg.get("end_y", d[1] - 30)) if cfg.get("end_y") else None
    if e:
        out["wall_end_y"] = e[0]; out["end_sig"] = e[1]; out["end"] = e[0] - d[1]
    out["_lines"] = {"wall": w, "dot": d}
    return out, None


def draw_cam2(img, m):
    out = img.copy(); l = m["_lines"]["wall"]; vx, vy = l["v"]; S = 500
    cv2.line(out, (int(l["x0"] - vx * S / 2), int(l["y0"] - vy * S / 2)), (int(l["x0"] + vx * S / 2), int(l["y0"] + vy * S / 2)), (0, 0, 255), 2)
    cv2.circle(out, (int(m["dot_x"]), int(m["dot_y"])), 9, (0, 255, 0), 2)
    if "wall_end_y" in m:
        cv2.circle(out, (int(m["wall_x"]), int(m["wall_end_y"])), 7, (255, 0, 255), 2)
    cv2.putText(out, f"cam2 ang {m['wall_ang']:+.2f} gap {m['gap']:+.1f}px end {m.get('end', float('nan')):+.1f}px", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return out


def default_cfg2(cal, ck):
    sg = cal["slot_golden"][ck]
    if sg.get("edge_cfg2"):
        return sg["edge_cfg2"]
    return {"dot": [608, 452], "wall_x": 592, "wall_rows": [40, 380], "end_y": 420, "y_ref": 250}


def main():
    ck = sys.argv[1] if len(sys.argv) > 1 else "blue"
    path = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else None
    cam2 = "--cam2" in sys.argv
    img = cv2.imread(path) if path else grab(8768 if cam2 else 8766)
    cal = json.load(open(CAL)); cfg = default_cfg2(cal, ck) if cam2 else default_cfg(cal, ck)
    m, why = measure_cam2(img, cfg) if cam2 else measure(img, cfg)
    if not m:
        print("실패:", why); return
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items() if not k.startswith("_")})
    if "--save" in sys.argv:
        cv2.imwrite(sys.argv[sys.argv.index("--save") + 1], draw_cam2(img, m) if cam2 else draw(img, m, cfg))


if __name__ == "__main__":
    main()
