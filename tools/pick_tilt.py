# 8/30 ① 탑뷰 벽 기울기 측정 — pick 후 z400 에서 손목캠 원본 캡처
#   python3 pick_tilt.py <색> [nopick]
import json,time,urllib.request,re,sys,os
ck=sys.argv[1] if len(sys.argv)>1 else 'blue'
SKIP = len(sys.argv)>2 and sys.argv[2]=='nopick'
OUT=os.environ.get('OUT','/tmp/claude-1000/-home-ar/dbd1216c-e2b6-4000-90fc-6499fdfa3287/scratchpad')
def post(a,b):
    r=urllib.request.Request(f'http://localhost:8765/fr5/{a}',data=json.dumps(b).encode(),headers={'Content-Type':'application/json'})
    return json.loads(urllib.request.urlopen(r,timeout=25).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status',timeout=3).read())['robots']['fr5']
def wait(tmax=240):
    time.sleep(1.5)
    for i in range(int(tmax/0.4)):
        if not stt()['busy']: break
        time.sleep(0.4)
    time.sleep(1.0)
def tail(): return open('/tmp/bf2_bridge.log').read().split('dot_pick started')[-1]
def snap(name):
    urllib.request.urlretrieve('http://127.0.0.1:8766/raw', f'{OUT}/{name}.jpg')
    print("   캡처", f'{OUT}/{name}.jpg')
T0=time.time()
if not SKIP:
    post('speed',{'value':5,'dry_run':False}); time.sleep(0.3)
    print(post('dot_pick',{'kind':ck,'dry_run':False})); time.sleep(5)
    for i in range(240):
        t=tail()
        if 'z380 정지' in t or 'ERR' in t or '중단' in t or ('-> done' in t and not stt()['busy']): break
        time.sleep(1)
    time.sleep(1)
    if 'z380 정지' in tail():
        post('dot_pick_continue',{'dry_run':False})
        for i in range(150):
            t=tail()
            if ('-> done' in t and not stt()['busy']) or 'ERR' in t: break
            time.sleep(1)
    for line in tail().splitlines():
        m=re.search(r'-> (.*)$',line)
        if m and any(k in line for k in ('보정 완료','z380 검증','그리퍼 명령','ERR','헛집','중단','손목각')): print("  ",m.group(1)[:110])
    if 'ERR' in tail(): raise SystemExit("pick 실패 — 중단")
    print(f"[pick {time.time()-T0:.0f}s] 실측", post('grip_read',{'dry_run':True})['result'])
# z400 으로 상승 (랙 위, 옆 장애물 없음)
post('speed',{'value':3,'dry_run':False}); time.sleep(0.3)
z=stt()['tcp'][2]; d=400.0-z
print(f"  z {z:.1f} → 400 ({d:+.1f}mm)")
if abs(d)>0.5: post('cart_jog',{'axis':2,'delta':d,'dry_run':False}); wait()
time.sleep(1.0)
f=stt(); print("  z400 tcp",[round(v,2) for v in f['tcp']])
snap(f'tilt_{ck}_z400')
