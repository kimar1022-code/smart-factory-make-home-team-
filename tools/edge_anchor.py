#!/usr/bin/env python3
"""9/3 밑판 외곽선 앵커 — 검정 밑판 vs 흰 바닥. 긴 직선 변 2개(가로/세로)를 서브픽셀로 맞춰 교점(꼭짓점)·각도를 준다.
  python3 edge_anchor.py <이미지 또는 cam1|cam2> [--save out.jpg]
반환 dict: {"corner": (x,y), "ang_h": 가로변 각도°, "ang_v": 세로변 각도°, "len_h","len_v","sigma_h","sigma_v"}"""
import sys, math, json, urllib.request
import cv2, numpy as np

PORT = {"cam1": 8766, "cam2": 8768}


def _grab(src):
    if src in PORT:
        buf = urllib.request.urlopen(f"http://127.0.0.1:{PORT[src]}/raw", timeout=5).read()
        return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
    return cv2.imread(src)


def _subpix_line(gray, seg, band=8, step=2, _pass=0):
    """세그먼트 주변에서 법선 방향 50% 교차점을 서브픽셀로 뽑아 fitLine. (각도°, 잔차σ, n, (vx,vy,x0,y0))"""
    x1, y1, x2, y2 = seg
    L = math.hypot(x2 - x1, y2 - y1)
    if L < 40:
        return None
    ux, uy = (x2 - x1) / L, (y2 - y1) / L
    nx, ny = -uy, ux
    pts = []
    for t in np.arange(0, L, step):
        cx, cy = x1 + ux * t, y1 + uy * t
        prof, coords = [], []
        for k in range(-band, band + 1):
            px, py = cx + nx * k, cy + ny * k
            if not (0 <= px < gray.shape[1] - 1 and 0 <= py < gray.shape[0] - 1):
                prof = []; break
            # 쌍선형 보간
            x0, y0 = int(px), int(py); fx, fy = px - x0, py - y0
            v = (gray[y0, x0] * (1 - fx) * (1 - fy) + gray[y0, x0 + 1] * fx * (1 - fy)
                 + gray[y0 + 1, x0] * (1 - fx) * fy + gray[y0 + 1, x0 + 1] * fx * fy)
            prof.append(v); coords.append((px, py))
        if len(prof) < 2 * band:
            continue
        prof = np.array(prof, float)
        lo, hi = prof.min(), prof.max()
        if hi - lo < 50:
            continue
        thr = (lo + hi) / 2
        idx = np.where(np.diff(np.sign(prof - thr)) != 0)[0]
        if len(idx) != 1:          # 교차가 하나뿐인(깨끗한 단일 에지) 자리만
            continue
        i = idx[0]; f = (thr - prof[i]) / (prof[i + 1] - prof[i])
        pts.append((coords[i][0] + (coords[i + 1][0] - coords[i][0]) * f, coords[i][1] + (coords[i + 1][1] - coords[i][1]) * f))
    if len(pts) < 25:
        return None
    P = np.array(pts, np.float32)
    vx, vy, x0, y0 = cv2.fitLine(P, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    d = np.abs((P[:, 0] - x0) * vy - (P[:, 1] - y0) * vx)
    keep = d < max(1.0, 3 * d.std())                              # 이상치 컷 후 재맞춤
    if keep.sum() >= 25:
        vx, vy, x0, y0 = cv2.fitLine(P[keep], cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        d = np.abs((P[keep][:, 0] - x0) * vy - (P[keep][:, 1] - y0) * vx)
    if _pass < 1:                                                  # ★2패스: 맞춘 직선 위에서 다시 샘플(초기 세그먼트 오차 제거)
        seg2 = (x0 - vx * L / 2, y0 - vy * L / 2, x0 + vx * L / 2, y0 + vy * L / 2)
        r2 = _subpix_line(gray, seg2, band=max(4, band // 2), step=step, _pass=_pass + 1)
        if r2:
            return r2
    ang = math.degrees(math.atan2(vy, vx))
    return ang, float(d.std()), int(keep.sum()), (float(vx), float(vy), float(x0), float(y0))


def detect(img, min_len=200):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(gray, (5, 5), 0)
    e = cv2.Canny(g, 40, 120)
    ls = cv2.HoughLinesP(e, 1, np.pi / 720, threshold=100, minLineLength=min_len, maxLineGap=10)
    if ls is None:
        return None, "직선 없음"
    segs = []
    for x1, y1, x2, y2 in ls[:, 0]:
        L = math.hypot(x2 - x1, y2 - y1); a = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180
        segs.append((L, a, (int(x1), int(y1), int(x2), int(y2))))
    segs.sort(reverse=True)
    best = {"h": None, "v": None}
    for L, a, seg in segs:
        kind = "h" if (a < 30 or a > 150) else ("v" if 60 < a < 120 else None)
        if kind is None or best[kind] is not None:
            continue
        r = _subpix_line(gray, seg)
        if r and r[1] < 1.0:
            best[kind] = (L, r)
        if best["h"] and best["v"]:
            break
    if not (best["h"] and best["v"]):
        return None, f"변 부족 (h={'○' if best['h'] else '✗'} v={'○' if best['v'] else '✗'})"
    (Lh, (ah, sh, nh, (vx1, vy1, x1, y1))), (Lv, (av, sv, nv, (vx2, vy2, x2, y2))) = best["h"], best["v"]
    # 교점
    A = np.array([[vx1, -vx2], [vy1, -vy2]], float); b = np.array([x2 - x1, y2 - y1], float)
    t = np.linalg.solve(A, b)
    cx, cy = x1 + vx1 * t[0], y1 + vy1 * t[0]
    return {"corner": (round(float(cx), 2), round(float(cy), 2)), "ang_h": round(ah, 3), "ang_v": round(av, 3),
            "len_h": int(Lh), "len_v": int(Lv), "sigma_h": round(sh, 2), "sigma_v": round(sv, 2), "n_h": nh, "n_v": nv}, None


if __name__ == "__main__":
    src = sys.argv[1]
    img = _grab(src)
    r, why = detect(img)
    print(json.dumps(r, ensure_ascii=False) if r else f"실패: {why}")
    if "--save" in sys.argv and r:
        out = img.copy(); c = tuple(int(v) for v in r["corner"])
        cv2.circle(out, c, 8, (0, 0, 255), 2); cv2.imwrite(sys.argv[sys.argv.index("--save") + 1], out)
