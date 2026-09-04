"""8/30 파지 기울기 계측 — 물고 있는 벽면을 뎁스 격자로 평면 피팅.
카메라는 그리퍼에 붙어 있으므로(벽과 한 몸) 로봇 z 와 무관하게 '파지 상태' 만 본다.
벽이 수직으로 매달리면 벽면 법선은 광축(≈수직)과 직교 → n_z≈0. 기울기 θ = asin(n_z).
  python3 wall_tilt.py [라벨]
"""
import json, math, sys, urllib.request
import numpy as np

CAM = "http://127.0.0.1:8766"
FX = FY = 924.0; CX, CY = 640.0, 360.0      # D435 컬러 1280x720 (HFOV 69.4°) 근사
LABEL = sys.argv[1] if len(sys.argv) > 1 else ""

def grid(x0, y0, x1, y1, nx, ny, r=3):
    u = f"{CAM}/depthgrid?x0={x0}&y0={y0}&x1={x1}&y1={y1}&nx={nx}&ny={ny}&r={r}"
    return json.load(urllib.request.urlopen(u, timeout=8))["pts"]

def raw():
    import cv2
    d = urllib.request.urlopen(f"{CAM}/raw", timeout=8).read()
    return cv2.imdecode(np.frombuffer(d, np.uint8), cv2.IMREAD_COLOR)

def fit(pts):
    """(x,y,d) → 카메라 3D → 평면 최소자승(이상치 2회 트림). 반환 (n, rms, n_used)"""
    P = np.array([[(p[0]-CX)*p[2]/FX, (p[1]-CY)*p[2]/FY, p[2]] for p in pts])
    for _ in range(3):
        c = P.mean(0)
        _, _, V = np.linalg.svd(P - c)
        n = V[2] / np.linalg.norm(V[2])
        res = (P - c) @ n
        rms = float(np.sqrt((res**2).mean()))
        keep = np.abs(res) < max(2.0, 2.5*rms)
        if keep.all() or keep.sum() < 12: break
        P = P[keep]
    if n[2] < 0: n = -n
    return n, rms, len(P)

im = raw()
# 벽 마스크: 프레임 3회 평균 뎁스가 유효하고 180~400mm(= 물고 있는 벽) 인 격자점만 사용
raws = []
for _ in range(3):
    raws.append(grid(700, 200, 1120, 700, 15, 15))
acc = {}
for g in raws:
    for p in g:
        if p["d"]: acc.setdefault((p["x"], p["y"]), []).append(p["d"])
pts = [(x, y, sorted(v)[len(v)//2]) for (x, y), v in acc.items() if len(v) >= 2]
pts = [p for p in pts if 180 <= p[2] <= 400]
if len(pts) < 20:
    raise SystemExit(f"유효 뎁스 점 {len(pts)}개 — 격자/거리 확인")
n, rms, used = fit(pts)
theta = math.degrees(math.asin(max(-1, min(1, n[2]))))
yaw = math.degrees(math.atan2(n[1], n[0]))
d = [p[2] for p in pts]
print(f"[{LABEL or 'wall_tilt'}] 점 {used}/{len(pts)}  거리 {min(d):.0f}~{max(d):.0f}mm  평면 RMS {rms:.2f}mm")
print(f"  법선 n = ({n[0]:+.4f}, {n[1]:+.4f}, {n[2]:+.4f})")
print(f"  ★기울기 θ(광축 대비) = {theta:+.2f}°   면내 방위 = {yaw:+.1f}°")
print(f"  → 벽 아래끝(80mm) 횡변위 ≈ {80*math.sin(math.radians(theta)):+.2f}mm")
