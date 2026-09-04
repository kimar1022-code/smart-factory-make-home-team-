import json,time,urllib.request,sys,re
ck=sys.argv[1]
def post(a,b):
    r=urllib.request.Request(f'http://localhost:8765/fr5/{a}',data=json.dumps(b).encode(),headers={'Content-Type':'application/json'}); return json.loads(urllib.request.urlopen(r,timeout=15).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status',timeout=3).read())['robots']['fr5']
def tail(): return open('/tmp/bf2_bridge.log').read().split('dot_pick started')[-1]
t0=time.time()
print(post('dot_pick',{'kind':ck,'dry_run':False,'grip_pause':True}))
time.sleep(5)
for i in range(240):
    t=tail()
    if 'z380 정지' in t or 'ERR' in t or '중단' in t or ('-> done' in t and not stt()['busy']): break
    time.sleep(1)
time.sleep(1)
if 'z380 정지' in tail():
    post('dot_pick_continue',{'dry_run':False})
    for i in range(150):
        t=tail()
        if '파지 직전 정지' in t or 'ERR' in t or '중단' in t: break
        time.sleep(1)
time.sleep(1); f=stt()
print(f"[{ck}] {time.time()-t0:.0f}s  tcp",[round(v,1) for v in f['tcp']],"busy",f['busy'])
for line in tail().splitlines():
    m=re.search(r'-> (.*)$',line)
    if m and any(k in line for k in ('보정 완료','z380 검증','z380 보정','재검출','파지 직전','ERR','중단','손목각','yaw 회전')): print("  ",m.group(1)[:130])
