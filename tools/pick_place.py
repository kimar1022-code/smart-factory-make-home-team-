"""pick → place 호버(z571)까지. 삽입은 insert_search.py 가 이어받는다.
(8/28 '슬롯 안 재파지' 단계는 효과 없다고 판정돼 제거 — 변수만 늘렸음)"""
import json, re, sys, time, urllib.request
ck = sys.argv[1]
def post(a, b):
    r = urllib.request.Request(f'http://localhost:8765/fr5/{a}', data=json.dumps(b).encode(),
                               headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(r, timeout=20).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status', timeout=3).read())['robots']['fr5']
def tail(): return open('/tmp/bf2_bridge.log').read().split('dot_pick started')[-1]
T0 = time.time()
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
post('speed', {'value': 5, 'dry_run': False}); time.sleep(0.3)
post('cart_jog', {'axis': 2, 'delta': 150.0, 'dry_run': False}); time.sleep(2)
for i in range(300):
    if not stt()['busy']: break
    time.sleep(0.4)
n0 = len(open('/tmp/bf2_bridge.log').read()); t0 = time.time()
print(post('dot_place', {'kind': ck, 'dry_run': False})); time.sleep(5)
for i in range(400):
    tl = open('/tmp/bf2_bridge.log').read()[n0:]
    if '호버 정지' in tl or 'ERR' in tl or '중단' in tl: break
    time.sleep(1)
time.sleep(2)
for line in open('/tmp/bf2_bridge.log').read()[n0:].splitlines():
    m = re.search(r'dot_place \{\} -> (.*)$', line)
    if m and any(k in line for k in ('검증 OK', '잔차', '벽 물림', '벽 기준', '⚠', 'ERR')): print("  ", m.group(1)[:125])
if 'ERR' in open('/tmp/bf2_bridge.log').read()[n0:]: raise SystemExit("place 실패 — 중단")
ins = json.load(open('/home/ar/bf2_console/dot_calib.json'))['refs_place'][ck]['insert_tcp']
t = stt()['tcp']
print(f"[place {time.time()-t0:.0f}s] z571 tcp {[round(v,2) for v in t]}  성공자세 대비 ΔX {t[0]-ins[0]:+.2f} Y {t[1]-ins[1]:+.2f} rz {((t[5]-ins[5]+180)%360)-180:+.2f}°")
post('stop', {'dry_run': False})
