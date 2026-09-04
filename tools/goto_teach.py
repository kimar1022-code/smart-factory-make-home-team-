"""파지 → place 티칭 높이(기본 z580)로 안전 이동 → 드래그 모드 ON.
경로: 파지 후 z650 까지 수직 상승 → 그 높이에서 삽입 XY·자세로 수평 이동 → 수직 하강.
(기둥 높이(≈571 진입)보다 충분히 위에서만 옆으로 움직인다)"""
import json, re, sys, time, urllib.request
ck = sys.argv[1] if len(sys.argv) > 1 else 'red'
ZT = float(sys.argv[2]) if len(sys.argv) > 2 else 580.0
ZSAFE = 650.0
def post(a, b):
    r = urllib.request.Request(f'http://localhost:8765/fr5/{a}', data=json.dumps(b).encode(),
                               headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(r, timeout=25).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status', timeout=3).read())['robots']['fr5']
def wait(t=300):
    time.sleep(1.5)
    for _ in range(int(t/0.4)):
        if not stt()['busy']: break
        time.sleep(0.4)
    last = None; same = 0
    for _ in range(40):
        time.sleep(0.25); z = round(stt()['tcp'][2], 2)
        same = same + 1 if (last is not None and abs(z-last) < 0.03) else 0; last = z
        if same >= 2: return
def tail(): return open('/tmp/bf2_bridge.log').read().split('dot_pick started')[-1]
T0 = time.time()
if stt()['manual']:
    print('드래그 해제:', post('manual', {'on': False, 'dry_run': False})['result'][:40])
post('speed', {'value': 5, 'dry_run': False}); time.sleep(0.3)
print(post('dot_pick', {'kind': ck, 'dry_run': False})); time.sleep(5)
for i in range(240):
    t = tail()
    if 'z380 정지' in t or 'ERR' in t or '중단' in t or ('-> done' in t and not stt()['busy']): break
    time.sleep(1)
time.sleep(1)
if 'z380 정지' in tail():
    post('dot_pick_continue', {'dry_run': False})
    for i in range(150):
        t = tail()
        if ('-> done' in t and not stt()['busy']) or 'ERR' in t: break
        time.sleep(1)
for line in tail().splitlines():
    m = re.search(r'-> (.*)$', line)
    if m and any(k in line for k in ('보정 완료', 'z380 검증', '그리퍼 명령', 'ERR', '헛집', '중단')): print("  ", m.group(1)[:110])
if 'ERR' in tail(): raise SystemExit("pick 실패 — 중단")
print(f"[pick {time.time()-T0:.0f}s] 실측", post('grip_read', {'dry_run': True})['result'])
ins = json.load(open('/home/ar/bf2_console/dot_calib.json'))['refs_place'][ck]['insert_tcp']
post('speed', {'value': 4, 'dry_run': False}); time.sleep(0.3)
t = list(stt()['tcp']); t[2] = ZSAFE
post('move_tcp', {'tcp': t, 'dry_run': False}); wait(); print("  안전고도 z", round(stt()['tcp'][2], 1))
t = list(ins); t[2] = ZSAFE
post('move_tcp', {'tcp': t, 'dry_run': False}); wait(); print("  삽입 XY·자세 위 도착", [round(v, 1) for v in stt()['tcp']])
post('speed', {'value': 2, 'dry_run': False}); time.sleep(0.3)
t = list(stt()['tcp']); t[2] = ZT
post('move_tcp', {'tcp': t, 'dry_run': False}); wait()
print(f"[{ck}] z{stt()['tcp'][2]:.1f} 도착 tcp {[round(v,2) for v in stt()['tcp']]}  총 {time.time()-T0:.0f}s")
print("드래그 모드 ON:", post('manual', {'on': True, 'dry_run': False})['result'][-30:])
