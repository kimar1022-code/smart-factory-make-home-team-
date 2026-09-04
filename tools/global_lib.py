"""8/30 글로벌캠(탑뷰) 공용 계측 — 랙과 밑판만 본다.
손목캠은 벽을 물고 나면 벽 점이 하나뿐이라 yaw 를 못 잰다. 글로벌캠은 '물기 전' 랙에서
벽 yaw 를 0.1° 로 재므로, 파지 순간 손목 rz 와 빼면 '그리퍼 안 벽 각도'를 계산으로 알 수 있다.
"""
import json, math, urllib.request
import cv2, numpy as np

CAL = "/home/ar/bf2_console/dot_calib.json"
FULL = "http://127.0.0.1:8767/full"
RANGES = {'red':   [((0,120,80),(8,255,255)), ((170,120,80),(180,255,255))],
          'yellow':[((20,110,110),(35,255,255))],
          'blue':  [((95,110,70),(120,255,255))],
          'green': [((40,45,40),(90,255,255))]}     # 8/30: 초록 미검출 → 하한 완화

def grab(timeout=25):
    d = urllib.request.urlopen(FULL, timeout=timeout).read()
    return cv2.imdecode(np.frombuffer(d, np.uint8), cv2.IMREAD_COLOR)

def find(im, kind, roi, amin=3, amax=250):
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    x0, y0, x1, y1 = roi
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in RANGES[kind]:
        m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    m[:y0, :] = 0; m[y1:, :] = 0; m[:, :x0] = 0; m[:, x1:] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    out = []
    for i in range(1, n):
        a = st[i, cv2.CC_STAT_AREA]
        if not (amin <= a <= amax): continue
        ys, xs = np.where(lab == i)
        w = g[ys, xs].astype(float) + 1.0
        out.append((float((xs*w).sum()/w.sum()), float((ys*w).sum()/w.sum()), int(a)))
    return sorted(out, key=lambda q: -q[2])

def wrap90(a):
    while a > 90: a -= 180
    while a <= -90: a += 180
    return a

def line_fit(pts):
    P = np.array([(p[0], p[1]) for p in pts], float)
    c = P.mean(0); _, _, Vt = np.linalg.svd(P - c); v = Vt[0]
    resid = float(np.abs((P - c) @ np.array([-v[1], v[0]])).max())
    return wrap90(math.degrees(math.atan2(v[1], v[0]))), resid, c

def rack_wall(kind, im=None, n=3):
    """랙에 꽂힌 이 색 벽의 (yaw°, 직선잔차px, 중심, 점수). 없으면 yaw=None."""
    cal = json.load(open(CAL))
    roi = cal.get("global_rack_roi", [638, 225, 887, 468])
    im = im if im is not None else grab()
    b = find(im, kind, roi)[:n]
    if len(b) < 2: return None, None, None, len(b)
    a, res, c = line_fit(b)
    return a, res, (float(c[0]), float(c[1])), len(b)

def plate_angle(im=None):
    """밑판 외곽선 각도(°) — 슬롯 방향의 기준. (각, 중심, 면적)"""
    cal = json.load(open(CAL))
    x0, y0, x1, y1 = cal.get("global_base_roi", [952, 60, 1268, 448])
    im = im if im is not None else grab()
    g = cv2.GaussianBlur(cv2.cvtColor(im[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY), (5, 5), 0)
    _, th = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cs, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cs: return None, None, 0
    c = max(cs, key=cv2.contourArea)
    if cv2.contourArea(c) < 2000: return None, None, 0
    (cx, cy), (w, h), ang = cv2.minAreaRect(c)
    return wrap90(ang), (cx + x0, cy + y0), float(cv2.contourArea(c))

def wall_in_rack(kind, im=None):
    """이 색 벽이 랙에 돌아왔는가 — 점 3개 + 간격이 기준(point_span_mm)과 맞으면 True."""
    cal = json.load(open(CAL))
    im = im if im is not None else grab()
    a, res, c, n = rack_wall(kind, im)
    if n < 3 or a is None: return False, n, None
    roi = cal.get("global_rack_roi", [638, 225, 887, 468])
    b = find(im, kind, roi)[:3]
    d = max(((p[0]-q[0])**2 + (p[1]-q[1])**2) ** 0.5 for p in b for q in b)
    span = cal['refs'][kind].get('point_span_mm')
    mmpx = (span/d) if (span and d > 10) else None
    ok = bool(mmpx and 0.9 < mmpx < 1.6 and res < 1.5)
    return ok, n, mmpx
