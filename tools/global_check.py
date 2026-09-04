"""8/30 글로벌캠(탑뷰) 계측 — 랙과 밑판만 본다.
  · 랙: 벽마다 점 3개로 yaw(선각) + 위치, 축척(mm/px) 자체 검증
  · 밑판: 기둥 점 위치 + 기둥선 각도
  · ★핵심 출력: 벽 yaw − 밑판 기둥선 yaw = 삽입에 필요한 손목 회전량(독립 측정)
손목캠은 파지 후 벽 점이 하나뿐이라 yaw 를 못 잰다 → 글로벌캠이 그 구멍을 메운다.
  python3 global_check.py [--save]
"""
import json, math, sys, urllib.request
import cv2, numpy as np

CAL = "/home/ar/bf2_console/dot_calib.json"
G = "http://127.0.0.1:8767/full"
RANGES = {'red':   [((0,120,80),(8,255,255)), ((170,120,80),(180,255,255))],
          'yellow':[((20,110,110),(35,255,255))],
          'blue':  [((95,110,70),(120,255,255))],
          'green': [((45,60,50),(85,255,255))]}

def grab():
    d = urllib.request.urlopen(G, timeout=25).read()
    return cv2.imdecode(np.frombuffer(d, np.uint8), cv2.IMREAD_COLOR)

def find(im, kind, roi, amin=3, amax=250):
    """서브픽셀 중심(밝기 가중) 블롭 목록."""
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    x0, y0, x1, y1 = roi
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in RANGES[kind]:
        m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    m[:y0, :] = 0; m[y1:, :] = 0; m[:, :x0] = 0; m[:, x1:] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for i in range(1, n):
        a = st[i, cv2.CC_STAT_AREA]
        if not (amin <= a <= amax): continue
        ys, xs = np.where(lab == i)
        w = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)[ys, xs].astype(float) + 1.0
        out.append((float((xs*w).sum()/w.sum()), float((ys*w).sum()/w.sum()), int(a)))
    return sorted(out, key=lambda q: -q[2])

def line_angle(pts):
    """최소자승 직선의 각도(°, -90~90) — 점 3개 이상이면 전체 사용."""
    P = np.array([(p[0], p[1]) for p in pts], float)
    c = P.mean(0); U, S, Vt = np.linalg.svd(P - c)
    v = Vt[0]
    a = math.degrees(math.atan2(v[1], v[0]))
    while a > 90: a -= 180
    while a <= -90: a += 180
    resid = float(np.abs((P - c) @ np.array([-v[1], v[0]])).max())
    return a, resid, c

cal = json.load(open(CAL))
RACK = cal.get("global_rack_roi", [640, 230, 860, 440])
BASE = cal.get("global_base_roi", [960, 80, 1280, 350])
im = grab()
print(f"글로벌캠 원본 {im.shape[1]}x{im.shape[0]}   랙 ROI {RACK}   밑판 ROI {BASE}\n")

print("── 랙 (벽마다 점 3개) ─────────────────────────")
scales, wall = [], {}
for ck in ('blue', 'red', 'yellow', 'green'):
    b = find(im, ck, RACK)[:3]
    if len(b) < 2:
        print(f"  {ck:7s} 점 {len(b)}개 — 계측 불가"); continue
    a, res, c = line_angle(b)
    span = cal['refs'][ck].get('point_span_mm')
    d = max(((p[0]-q[0])**2 + (p[1]-q[1])**2) ** 0.5 for p in b for q in b)
    s = span/d if (span and d > 10) else None
    if s: scales.append(s)
    wall[ck] = a
    print(f"  {ck:7s} 점{len(b)}  중심({c[0]:6.1f},{c[1]:6.1f})  선각 {a:+7.2f}°  직선잔차 {res:.2f}px"
          + (f"  축척 {s:.3f}mm/px" if s else ""))
if scales:
    print(f"  → 축척 평균 {np.mean(scales):.3f} mm/px (편차 {np.std(scales):.3f}) "
          f"— 각 1° 는 벽 끝단에서 {math.tan(math.radians(1))*126:.1f}mm")

print("\n── 밑판 (기둥 점) ─────────────────────────────")
base = {}
for ck in ('blue', 'red', 'yellow', 'green'):
    b = find(im, ck, BASE)
    if not b: continue
    base[ck] = b
    print(f"  {ck:7s} {len(b)}개: " + "  ".join(f"({x:6.1f},{y:6.1f})a{a}" for x, y, a in b[:4]))
# 밑판 방향은 점 두 개(대각선일 수 있음)가 아니라 '판 외곽선' 으로 잡는다 — 길고 확실한 특징
def plate_angle(im, roi):
    x0, y0, x1, y1 = roi
    g = cv2.cvtColor(im[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(g, (5, 5), 0)
    _, th = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cs, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cs: return None
    c = max(cs, key=cv2.contourArea)
    if cv2.contourArea(c) < 2000: return None
    (cx, cy), (w, h), ang = cv2.minAreaRect(c)
    return (cx + x0, cy + y0), (w, h), ang, cv2.contourArea(c)
pa = plate_angle(im, BASE)
if pa:
    (cx, cy), (w, h), ang, area = pa
    a1 = ang; a2 = ang + 90
    for a in (a1, a2):
        aa = a
        while aa > 90: aa -= 180
        while aa <= -90: aa += 180
        print(f"  밑판 외곽 변 각 {aa:+7.2f}°", end="  ")
    print(f"\n  밑판 중심({cx:.1f},{cy:.1f})  크기 {w:.0f}x{h:.0f}px = {w*1.201:.0f}x{h*1.201:.0f}mm  면적 {area:.0f}px")
    PLATE = ang
else:
    PLATE = None
    print("  밑판 외곽 검출 실패 — ROI 확인")
pairs = [(k, v) for k, v in base.items() if len(v) >= 2]
for k, v in pairs:
    a, res, c = line_angle(v[:2])
    print(f"  (참고) {k} 점 두 개를 이은 선 {a:+7.2f}° — 대각선일 수 있으니 기준으로 쓰지 말 것")

print("\n── ★ 벽 yaw − 밑판 기둥선 yaw (삽입에 필요한 회전량) ──")
if PLATE is not None and wall:
    for ck, wa in wall.items():
        d = ((wa - PLATE + 90) % 180) - 90
        print(f"  {ck:7s} 벽 {wa:+7.2f}° − 밑판 {PLATE:+7.2f}° = {d:+7.2f}°"
              f"   (벽 끝단 {math.tan(math.radians(abs(d)))*126:5.2f}mm)")
    ws = sorted(wall.items(), key=lambda kv: kv[1])
    print(f"\n  ★ 벽끼리 편차: {ws[-1][0]} {ws[-1][1]:+.2f}° vs {ws[0][0]} {ws[0][1]:+.2f}° "
          f"= {ws[-1][1]-ws[0][1]:.2f}°  (끝단 {math.tan(math.radians(ws[-1][1]-ws[0][1]))*126:.2f}mm)")
else:
    print("  밑판 각도 없음 — ROI 조정 필요")
if "--save" in sys.argv:
    cal["global_rack_roi"] = RACK; cal["global_base_roi"] = BASE
    json.dump(cal, open(CAL, "w"), ensure_ascii=False, indent=1); print("\nROI 저장")
