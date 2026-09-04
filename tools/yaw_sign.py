"""8/30 글로벌캠 각도 ↔ 로봇 rz 부호·배율 실증.
벽을 문 채 높이 들고 손목 rz 를 알려진 각도로 돌리며 글로벌캠에서 벽 선각을 잰다.
   Δ(글로벌 선각) = k × Δrz   →  k 가 +1 이면 같은 방향, -1 이면 반대. |k|≠1 이면 배율 문제.
  python3 yaw_sign.py <색> [들고올릴z] [회전각들…]
"""
import json, math, sys, time, urllib.request
import numpy as np
import global_lib as G

CK = sys.argv[1] if len(sys.argv) > 1 else 'red'
ZUP = float(sys.argv[2]) if len(sys.argv) > 2 else 700.0
ANGS = [float(v) for v in sys.argv[3:] if not v.startswith('--')] or [0.0, +5.0, -5.0, -10.0, 0.0]
CAL = "/home/ar/bf2_console/dot_calib.json"
def post(a, b):
    r = urllib.request.Request(f'http://localhost:8765/fr5/{a}', data=json.dumps(b).encode(),
                               headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(r, timeout=30).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status', timeout=3).read())['robots']['fr5']
def wait(t=200):
    time.sleep(1.2)
    for _ in range(int(t/0.4)):
        if not stt()['busy']: break
        time.sleep(0.4)
    last = None; same = 0
    for _ in range(50):
        time.sleep(0.25); v = round(stt()['tcp'][5], 3)
        same = same + 1 if (last is not None and abs(v-last) < 0.02) else 0; last = v
        if same >= 2: return

cal = json.load(open(CAL))
RACK = cal.get("global_rack_roi", [638, 225, 887, 468])
BASE = cal.get("global_base_roi", [952, 60, 1268, 448])
def allpts(im=None):
    im = im if im is not None else G.grab()
    return G.find(im, CK, (0, 0, im.shape[1], im.shape[0])), im

ROI_HELD = {"box": None}      # 움직임으로 찾아낸 '물고 있는 벽' 영역

def find_moving(post_fn, wait_fn, stt_fn, test_deg=8.0):
    """rz 를 test_deg 돌려 '움직인 점들' 만 골라 물고 있는 벽을 특정한다.
    화면에는 정지한 같은 색 점이 많다(다른 트레이 등) → 움직임이 유일한 구분자."""
    a, im0 = allpts()
    rz0 = stt_fn()['tcp'][5]
    t = list(stt_fn()['tcp']); t[5] = rz0 + test_deg
    post_fn('move_tcp', {'tcp': t, 'dry_run': False}); wait_fn()
    b, im1 = allpts()
    moved = []
    for p in a:
        q = min(b, key=lambda r: (r[0]-p[0])**2 + (r[1]-p[1])**2) if b else None
        if q is None: continue
        d = ((q[0]-p[0])**2 + (q[1]-p[1])**2) ** 0.5
        if d > 3.0: moved.append(p)
    t = list(stt_fn()['tcp']); t[5] = rz0
    post_fn('move_tcp', {'tcp': t, 'dry_run': False}); wait_fn()
    if len(moved) < 2: return None
    xs = [p[0] for p in moved]; ys = [p[1] for p in moved]
    ROI_HELD["box"] = [min(xs)-40, min(ys)-40, max(xs)+40, max(ys)+40]
    return moved

def held_wall(n=3):
    """ROI_HELD 안에서 이 색 점들의 선각. (각, 잔차, 점수, 점들)"""
    im = G.grab()
    box = ROI_HELD["box"] or (0, 0, im.shape[1], im.shape[0])
    b = G.find(im, CK, [int(v) for v in box])[:4]
    if len(b) < 2: return None, None, len(b), b
    a, res, c = G.line_fit(b)
    span = max(((p[0]-q[0])**2 + (p[1]-q[1])**2) ** 0.5 for p in b for q in b)
    if span < 30: return None, None, len(b), b
    return a, res, len(b), b

print(f"[{CK}] 글로벌캠 각도 부호·배율 실증  (z{ZUP:.0f} 에서 rz {ANGS})")
f = stt()
if f['gripper'] > 20:
    raise SystemExit("그리퍼가 열려 있음 — 먼저 벽을 집어야 함 (pick_place.py 또는 verify_pick.py 사용)")
post('speed', {'value': 3, 'dry_run': False}); time.sleep(0.3)
t = list(stt()['tcp']); t[2] = ZUP
post('move_tcp', {'tcp': t, 'dry_run': False}); wait()
print("  들어올린 tcp", [round(v, 2) for v in stt()['tcp']])
print("  물고 있는 벽 특정 중 (rz 8° 돌려 '움직인 점' 만 추출)…")
mv = find_moving(post, wait, stt)
if mv:
    print(f"  ✔ 움직인 점 {len(mv)}개 → 탐색창 {[round(v) for v in ROI_HELD['box']]}")
else:
    print("  ⚠ 움직인 점을 못 찾음 — 그리퍼에 가려졌을 가능성")
a0, res0, n0, pts0 = held_wall()
if a0 is None:
    print(f"  ✘ 물고 있는 벽 점을 못 찾음(랙·밑판 밖 {CK} 점 {n0}개) — 그리퍼에 가려진 듯")
    print("     → 다른 높이/자세에서 재시도하거나, 이 방법은 포기하고 다른 기준 필요")
    raise SystemExit(1)
print(f"  ✔ 물고 있는 벽 검출: 점 {n0}개, 선각 {a0:+.2f}°, 직선잔차 {res0:.2f}px")
rows = []
base_rz = stt()['tcp'][5]
for da in ANGS:
    t = list(stt()['tcp']); t[5] = base_rz + da
    post('move_tcp', {'tcp': t, 'dry_run': False}); wait()
    rz = stt()['tcp'][5]
    time.sleep(0.5)
    a, res, n, _ = held_wall()
    if a is None:
        print(f"   rz {rz:+8.3f} (명령 {base_rz+da:+.2f}) → 벽 미검출"); continue
    rows.append((rz, a, res))
    print(f"   rz {rz:+8.3f} → 글로벌 선각 {a:+7.2f}°  (잔차 {res:.2f}px, 점 {n})")
t = list(stt()['tcp']); t[5] = base_rz
post('move_tcp', {'tcp': t, 'dry_run': False}); wait()
if len(rows) < 3:
    raise SystemExit("\n측정점 3개 미만 — 판정 불가")
rz = np.array([r[0] for r in rows]); ga = np.unwrap(np.radians(np.array([r[1] for r in rows])*2))/2
ga = np.degrees(ga)
k, b = np.polyfit(rz, ga, 1)
pred = k*rz + b
print(f"\n★ 글로벌 선각 = {k:+.3f} × rz {b:+.2f}°   (최대 잔차 {np.abs(ga-pred).max():.2f}°)")
print(f"   부호 {'같음(+)' if k > 0 else '반대(-)'}, 배율 |k| = {abs(k):.3f}")
if abs(abs(k)-1.0) > 0.15:
    print("   ⚠ 배율이 1 에서 크게 벗어남 — 글로벌캠 원근/기울기 영향. 이 축에서는 각도 환산에 배율 필요")
else:
    print("   ✔ 배율 1 에 가까움 — 글로벌 선각 차이를 그대로 rz 보정에 쓸 수 있음")
if "--save" in sys.argv:
    cal['global_yaw_k'] = round(float(k), 4); cal['global_yaw_b'] = round(float(b), 3)
    cal['global_yaw_made'] = time.strftime('%Y-%m-%d %H:%M') + ' 벽 물고 rz 회전 실증'
    json.dump(cal, open(CAL, 'w'), ensure_ascii=False, indent=1); print("   저장 완료")
