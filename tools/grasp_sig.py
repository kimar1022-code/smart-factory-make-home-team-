"""8/30 파지 지문(grasp signature) — 물고 있는 벽이 그리퍼 안에서 어떻게 물렸는지 계측.

카메라가 그리퍼에 붙어 있어 벽과 한 몸 → 로봇 자세와 무관하게 이 값은 '파지 상태'만 반영한다.
(로봇 z 를 바꿔도 벽 실루엣은 그대로. 바뀌면 벽이 손가락 안에서 움직인 것.)

계측: 벽의 오른쪽 실루엣 경계(벽↔배경)를 행마다 서브픽셀로 찾아 직선 피팅.
  edge_x(y0)  : 기준 행에서의 경계 x  → 진자 기울기(벽이 손가락 접촉선 축으로 기울면 횡이동)
  slope_deg   : 경계선 기울기          → yaw/롤 성분
  dot         : 물고 있는 벽의 색 점 픽셀(있으면)
  기준과 비교해 Δ 가 크면 재파지.

  python3 grasp_sig.py <색> [--save 메모]     저장 없이 계측만: python3 grasp_sig.py <색>
"""
import json, math, sys, time, urllib.request
import cv2, numpy as np

CAM = "http://127.0.0.1:8766"
CAL = "/home/ar/bf2_console/dot_calib.json"
Y0, Y1, YSTEP = 320, 680, 10       # 실루엣을 재는 행 범위
YREF = 500.0                        # 기준 행
XSCAN0, XSCAN1 = 1140, 780          # 배경(오른쪽) → 벽(왼쪽) 으로 스캔

def raw():
    d = urllib.request.urlopen(f"{CAM}/raw", timeout=8).read()
    return cv2.imdecode(np.frombuffer(d, np.uint8), cv2.IMREAD_COLOR)

def edge_line(imgs):
    """여러 프레임의 그레이 평균에서 행별 경계 x → (x@YREF, 기울기°, 잔차px, n)"""
    g = np.mean([cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).astype(np.float32) for im in imgs], axis=0)
    ys, xs = [], []
    for y in range(Y0, Y1 + 1, YSTEP):
        row = g[y]
        wall = float(np.median(row[820:930]))          # 벽 안쪽
        bg = float(np.median(row[1000:1120]))          # 배경
        if abs(wall - bg) < 25: continue               # 대비 부족한 행은 버림
        mid = (wall + bg) / 2.0
        x = None
        for xi in range(XSCAN0, XSCAN1, -1):           # 배경에서 벽 쪽으로
            a, b = row[xi], row[xi - 1]
            if (a - mid) * (b - mid) <= 0 and a != b:  # mid 를 가로지르는 첫 지점
                x = (xi - 1) + (mid - b) / (a - b)     # 서브픽셀
                break
        if x is not None: ys.append(y); xs.append(x)
    if len(ys) < 8: raise SystemExit(f"경계 검출 행 {len(ys)}개 — 대비/화각 확인")
    ys = np.array(ys, float); xs = np.array(xs, float)
    for _ in range(3):                                  # 이상치 트림
        A = np.vstack([ys, np.ones_like(ys)]).T
        (m, c), *_ = np.linalg.lstsq(A, xs, rcond=None)
        r = xs - (m * ys + c); s = float(np.sqrt((r ** 2).mean()))
        keep = np.abs(r) < max(1.5, 2.5 * s)
        if keep.all() or keep.sum() < 8: break
        ys, xs = ys[keep], xs[keep]
    return float(m * YREF + c), math.degrees(math.atan(m)), s, len(ys)

def held_dot(ck, n=6):
    acc = []
    for _ in range(n):
        for p in json.load(urllib.request.urlopen(f"{CAM}/dots", timeout=4))["dots"]:
            if p["kind"] == ck and p["px"] > 850: acc.append((p["px"], p["py"]))
        time.sleep(0.1)
    if len(acc) < 3: return None
    return [round(float(np.median([a[0] for a in acc])), 1), round(float(np.median([a[1] for a in acc])), 1)]

ck = sys.argv[1]
save = "--save" in sys.argv
imgs = []
for _ in range(5):
    imgs.append(raw()); time.sleep(0.12)
x0, slope, res, n = edge_line(imgs)
dot = held_dot(ck)
print(f"[{ck}] 파지 지문:  경계 x@y{YREF:.0f} = {x0:.2f}px   기울기 {slope:+.3f}°   잔차 {res:.2f}px (행 {n})")
print(f"        물고 있는 벽 점 = {dot}")
cal = json.load(open(CAL))
ref = (cal.get("grasp_sig") or {}).get(ck)
if ref:
    dx = x0 - ref["edge_x"]; dsl = slope - ref["slope_deg"]
    print(f"  기준({ref.get('made','')}) 대비  Δx {dx:+.2f}px   Δ기울기 {dsl:+.3f}°", end="")
    if dot and ref.get("dot"):
        print(f"   Δ점 ({dot[0]-ref['dot'][0]:+.1f},{dot[1]-ref['dot'][1]:+.1f})px")
    else: print()
else:
    print("  (이 색 기준 없음 — --save 로 저장)")
if save:
    i = sys.argv.index("--save")
    note = sys.argv[i+1] if len(sys.argv) > i+1 else ""
    cal.setdefault("grasp_sig", {})[ck] = {"edge_x": round(x0, 2), "slope_deg": round(slope, 3),
                                           "resid_px": round(res, 2), "rows": n, "dot": dot,
                                           "yref": YREF, "made": time.strftime("%Y-%m-%d %H:%M"), "note": note}
    json.dump(cal, open(CAL, "w"), ensure_ascii=False, indent=1)
    print(f"  ★기준 저장: {note}")
