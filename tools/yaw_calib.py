"""8/30 글로벌캠 ↔ 로봇 base 각도 부호·오프셋 실증.
같은 벽들을 손목캠(→base 좌표로 사상)과 글로벌캠(영상 좌표)으로 동시에 재서
   base_angle = sign * global_angle + offset
를 최소자승으로 구한다. 부호를 모르면 yaw 보정이 오차를 두 배로 만든다.
  python3 yaw_calib.py [--save]
"""
import json, math, sys, time, urllib.request
import numpy as np
import global_lib as G
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
    time.sleep(1.0)
cal = json.load(open(CAL))
Minv = cal["Minv_mm_per_obs"]
def theta_base(p1, p2):
    dx, dy = p2[0]-p1[0], p2[1]-p1[1]
    bx = Minv[0][0]*dx + Minv[0][1]*dy
    by = Minv[1][0]*dx + Minv[1][1]*dy
    return G.wrap90(math.degrees(math.atan2(by, bx)))
if abs(stt()['tcp'][2] - cal['observe_tcp'][2]) > 5 or abs(stt()['tcp'][0] - cal['observe_tcp'][0]) > 10:
    print('관측 자세로 이동…')
    post('speed', {'value': 4, 'dry_run': False}); time.sleep(0.3)
    t = list(stt()['tcp']); t[2] = max(t[2], 620.0); post('move_tcp', {'tcp': t, 'dry_run': False}); wait()
    post('move', {'joints': cal['observe_joints'], 'dry_run': False}); wait()
print('관측 tcp', [round(v, 2) for v in stt()['tcp']])
# 손목캠: 색별 점 3개 → base 각
acc = {}
for _ in range(12):
    for p in json.load(urllib.request.urlopen('http://localhost:8766/dots', timeout=4))['dots']:
        xb = (cal.get('pick_xband') or {}).get(p['kind'])
        if xb and not (xb[0] <= p['px'] <= xb[1]): continue
        acc.setdefault((p['kind'], round(p['py']/60)), []).append((p['px'], p['py']))
    time.sleep(0.08)
wrist = {}
for (k, _), v in acc.items():
    if len(v) >= 6: wrist.setdefault(k, []).append((float(np.median([q[0] for q in v])), float(np.median([q[1] for q in v]))))
im = G.grab()
rows = []
print(f"\n{'색':8s}{'손목캠(base°)':>16s}{'글로벌캠(영상°)':>18s}")
for k in ('blue', 'red', 'yellow', 'green'):
    pts = sorted(wrist.get(k, []), key=lambda q: q[1])
    if len(pts) < 2: print(f"  {k:7s} 손목캠 점 {len(pts)}개 — 제외"); continue
    wb = theta_base(pts[0], pts[-1])
    ga, res, c, n = G.rack_wall(k, im)
    if ga is None: print(f"  {k:7s} 글로벌캠 점 {n}개 — 제외"); continue
    rows.append((k, wb, ga))
    print(f"  {k:7s} {wb:14.2f} {ga:16.2f}   (글로벌 잔차 {res:.2f}px)")
if len(rows) < 2:
    raise SystemExit("\n비교 가능한 벽이 2개 미만 — 부호 판정 불가")
best = None
for sign in (+1, -1):
    offs = [G.wrap90(wb - sign*ga) for _, wb, ga in rows]
    m = math.degrees(math.atan2(np.mean([math.sin(math.radians(2*o)) for o in offs]),
                                np.mean([math.cos(math.radians(2*o)) for o in offs]))/2)
    err = max(abs(G.wrap90(o - m)) for o in offs)
    print(f"\n  부호 {sign:+d} 가정 → 오프셋 {m:+.2f}°, 벽 간 불일치 최대 {err:.2f}°")
    if best is None or err < best[2]: best = (sign, m, err)
sign, off, err = best
print(f"\n★ 결론: base_angle = {sign:+d} × global_angle {off:+.2f}°  (불일치 {err:.2f}°)")
print(f"   → 글로벌캠 1° 차이 = base 에서 {sign:+d}° 차이")
if err > 0.6: print("   ⚠ 불일치가 커서 신뢰도 낮음 — 벽이 실제로 다르게 꽂혀 있거나 검출 오차")
if "--save" in sys.argv:
    cal['global_yaw_sign'] = sign; cal['global_yaw_offset'] = round(off, 3)
    cal['global_yaw_made'] = time.strftime('%Y-%m-%d %H:%M') + f' 불일치 {err:.2f}°'
    json.dump(cal, open(CAL, 'w'), ensure_ascii=False, indent=1); print("   저장 완료")
