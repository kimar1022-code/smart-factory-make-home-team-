#!/usr/bin/env python3
"""9/3 검정 밑판(흰 바닥 위 돌출 블록) 꼭짓점 검출 — 굵은 검정 덩어리의 외곽 → 변별 서브픽셀 직선 → 교점.
  python3 base_corner.py <이미지|cam1|cam2> [--save out.jpg]"""
import sys, math, json, urllib.request
import cv2, numpy as np
from edge_anchor import _subpix_line

PORT = {"cam1": 8766, "cam2": 8768}


def grab(src):
    if src in PORT:
        buf = urllib.request.urlopen(f"http://127.0.0.1:{PORT[src]}/raw", timeout=5).read()
        return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
    return cv2.imread(src)


def base_mask(gray):
    _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)      # 검정=255
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((25, 25), np.uint8))           # 테이프 선(가는 것) 제거, 블록만
    n, lab, st, cen = cv2.connectedComponentsWithStats(th)
    if n < 2:
        return None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    return (lab == i).astype(np.uint8) * 255, st[i]


def detect(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    m = base_mask(gray)
    if m is None:
        return None, "검정 블록 없음"
    mask, st = m
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cs, key=cv2.contourArea)
    ap = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True), True).reshape(-1, 2)
    # 변: 연속 꼭짓점 쌍 중 길이 ≥120px 인 것만 서브픽셀 직선 맞춤
    lines = []
    for i in range(len(ap)):
        p, q = ap[i], ap[(i + 1) % len(ap)]
        L = math.hypot(q[0] - p[0], q[1] - p[1])
        if L < 120:
            continue
        r = _subpix_line(gray, (int(p[0]), int(p[1]), int(q[0]), int(q[1])), band=8, step=2)
        if r and r[1] < 1.0:
            lines.append({"len": int(L), "ang": round(r[0], 3), "sigma": round(r[1], 2), "n": r[2], "v": r[3], "p": (int(p[0]), int(p[1])), "q": (int(q[0]), int(q[1]))})
    if len(lines) < 2:
        return None, f"변 부족({len(lines)})"
    # 꼭짓점: 인접(각도차 60~120°) 변 쌍의 교점
    corners = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            a, b = lines[i], lines[j]
            d = abs(((a["ang"] - b["ang"]) + 180) % 180)
            if not (60 < d < 120):
                continue
            vx1, vy1, x1, y1 = a["v"]; vx2, vy2, x2, y2 = b["v"]
            A = np.array([[vx1, -vx2], [vy1, -vy2]], float); rhs = np.array([x2 - x1, y2 - y1], float)
            if abs(np.linalg.det(A)) < 1e-6:
                continue
            t = np.linalg.solve(A, rhs); cx, cy = x1 + vx1 * t[0], y1 + vy1 * t[0]
            # 교점이 두 변의 끝점 근처(60px)여야 실제 꼭짓점
            near = min(math.hypot(cx - e[0], cy - e[1]) for e in (a["p"], a["q"], b["p"], b["q"]))
            if near < 60 and 0 <= cx < gray.shape[1] and 0 <= cy < gray.shape[0]:
                corners.append({"xy": (round(float(cx), 2), round(float(cy), 2)), "edges": (i, j), "sig": max(a["sigma"], b["sigma"])})
    return {"bbox": [int(v) for v in st[:4]], "area": int(st[4]),
            "edges": [{k: v for k, v in l.items() if k in ("len", "ang", "sigma", "n")} for l in lines],
            "corners": corners}, None


if __name__ == "__main__":
    img = grab(sys.argv[1]); r, why = detect(img)
    print(json.dumps(r, ensure_ascii=False) if r else f"실패: {why}")
    if "--save" in sys.argv and r:
        out = img.copy()
        for c in r["corners"]:
            cv2.circle(out, (int(c["xy"][0]), int(c["xy"][1])), 9, (0, 0, 255), 2)
        cv2.imwrite(sys.argv[sys.argv.index("--save") + 1], out)
