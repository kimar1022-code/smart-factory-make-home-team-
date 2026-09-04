"""그리퍼를 연 채 파지 티칭 자세로 복귀 → 드래그 모드 ON (손으로 파지 재티칭용).
경로: 그리퍼 열기 → z650 수직 상승 → 파지 XY·자세 위로 수평 → z380 → 티칭 z 로 저속 하강."""
import json, sys, time, urllib.request
ck = sys.argv[1] if len(sys.argv) > 1 else 'red'
GRIP = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else 50   # 9/1: 개도 인자화(빨강 30 등)
NODRAG = '--nodrag' in sys.argv                                                  # 9/1: 드래그 모드 자동 진입 끄기
def post(a, b):
    r = urllib.request.Request(f'http://localhost:8765/fr5/{a}', data=json.dumps(b).encode(),
                               headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(r, timeout=25).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status', timeout=3).read())['robots']['fr5']
def wait(t=200):
    time.sleep(1.2)
    for _ in range(int(t/0.4)):
        if not stt()['busy']: break
        time.sleep(0.4)
    last = None; same = 0
    for _ in range(40):
        time.sleep(0.25); z = round(stt()['tcp'][2], 2)
        same = same + 1 if (last is not None and abs(z-last) < 0.03) else 0; last = z
        if same >= 2: return
if stt()['manual']: post('manual', {'on': False, 'dry_run': False}); time.sleep(1)
print(f'그리퍼 개도 {GRIP}:', post('gripper', {'pos': GRIP, 'dry_run': False})); time.sleep(12)
print('  실측', post('grip_read', {'dry_run': True})['result'])
tt = json.load(open('/home/ar/bf2_console/dot_calib.json'))['refs'][ck]['pick_tcp_taught']
post('speed', {'value': 4, 'dry_run': False}); time.sleep(0.3)
t = list(stt()['tcp']); t[2] = 650.0
post('move_tcp', {'tcp': t, 'dry_run': False}); wait(); print('  안전고도 650')
t = list(tt); t[2] = 650.0
post('move_tcp', {'tcp': t, 'dry_run': False}); wait(); print('  파지 XY·자세 위 도착', [round(v, 1) for v in stt()['tcp']])
post('speed', {'value': 2, 'dry_run': False}); time.sleep(0.3)
t = list(stt()['tcp']); t[2] = 400.0
post('move_tcp', {'tcp': t, 'dry_run': False}); wait(); print('  z400')
post('speed', {'value': 1, 'dry_run': False}); time.sleep(0.3)
t = list(stt()['tcp']); t[2] = float(tt[2])
post('move_tcp', {'tcp': t, 'dry_run': False}); wait()
print(f'[{ck}] 파지 티칭 자세 도착 tcp {[round(v,2) for v in stt()["tcp"]]}  (그리퍼 열림)')
if NODRAG:
    print('드래그 모드 진입 안 함(--nodrag) — 그리퍼는 열린 채 대기')
else:
    print('드래그 모드 ON:', post('manual', {'on': True, 'dry_run': False})['result'][-24:])
