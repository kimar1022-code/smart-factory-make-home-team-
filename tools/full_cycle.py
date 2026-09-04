import json,time,urllib.request,re,sys
ck=sys.argv[1] if len(sys.argv)>1 else 'blue'
def post(a,b):
    r=urllib.request.Request(f'http://localhost:8765/fr5/{a}',data=json.dumps(b).encode(),headers={'Content-Type':'application/json'}); return json.loads(urllib.request.urlopen(r,timeout=20).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status',timeout=3).read())['robots']['fr5']
def wait(tmax=240):
    time.sleep(1.5)
    for i in range(int(tmax/0.4)):
        if not stt()['busy']: break
        time.sleep(0.4)
    time.sleep(1.0)
def tail(): return open('/tmp/bf2_bridge.log').read().split('dot_pick started')[-1]
def dots():
    acc={}
    for i in range(6):
        for p in json.loads(urllib.request.urlopen('http://localhost:8766/dots',timeout=3).read())['dots']:
            acc.setdefault((p['kind'],round(p['px']/30),round(p['py']/30)),[]).append((p['px'],p['py']))
        time.sleep(0.08)
    return [(k[0],sum(x for x,_ in v)/len(v),sum(y for _,y in v)/len(v)) for k,v in acc.items() if len(v)>=4]
T0=time.time()
SKIP_PICK = len(sys.argv)>2 and sys.argv[2] in ('nopick','insertonly')
INSERT_ONLY = len(sys.argv)>2 and sys.argv[2]=='insertonly'
# ── 1. pick ──
if not SKIP_PICK:
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
      if m and any(k in line for k in ('보정 완료','z380 검증','그리퍼 명령','ERR','헛집','중단')): print("  ",m.group(1)[:110])
  if 'ERR' in tail(): raise SystemExit("pick 실패 — 중단")
  print(f"[pick {time.time()-T0:.0f}s] 실측",post('grip_read',{'dry_run':True})['result'])
  # ── 슬롯 안 재파지 (8/28 밤): 3mm 올려 살짝 열면 벽이 슬롯 안내로 수직 복귀 → 다시 닫아 패드에 평행 물림(진자 기울기 대책)
  post('speed',{'value':2,'dry_run':False}); time.sleep(0.3)
  post('cart_jog',{'axis':2,'delta':3.0,'dry_run':False}); wait()
  post('gripper',{'pos':25,'dry_run':False}); time.sleep(6); wait(30)
  post('cart_jog',{'axis':2,'delta':-3.0,'dry_run':False}); wait()
  post('gripper',{'pos':6,'dry_run':False}); time.sleep(10); wait(30)
  print("   재파지 실측",post('grip_read',{'dry_run':True})['result'])
  post('speed',{'value':5,'dry_run':False}); time.sleep(0.3)
  post('cart_jog',{'axis':2,'delta':150.0,'dry_run':False}); wait()
# ── 2. place → 571 ──
ref=json.load(open('/home/ar/bf2_console/dot_calib.json'))['refs_place'][ck]; ins=ref['insert_tcp']
if not INSERT_ONLY:
 ref=json.load(open('/home/ar/bf2_console/dot_calib.json'))['refs_place'][ck]; ins=ref['insert_tcp']
 n0=len(open('/tmp/bf2_bridge.log').read()); t0=time.time()
 print(post('dot_place',{'kind':ck,'dry_run':False})); time.sleep(5)
 for i in range(400):
     tl=open('/tmp/bf2_bridge.log').read()[n0:]
     if '호버 정지' in tl or 'ERR' in tl or '중단' in tl: break
     time.sleep(1)
 time.sleep(2); f=stt(); t=f['tcp']
 for line in open('/tmp/bf2_bridge.log').read()[n0:].splitlines():
     m=re.search(r'dot_place \{\} -> (.*)$',line)
     if m and any(k in line for k in ('검증 OK','잔차','벽 물림','⚠','ERR')): print("  ",m.group(1)[:120])
 if 'ERR' in open('/tmp/bf2_bridge.log').read()[n0:]: raise SystemExit("place 실패 — 중단")
 print(f"[place {time.time()-t0:.0f}s] z571 tcp",[round(v,2) for v in t]," 성공자세 대비 Δ X %+.2f Y %+.2f rz %+.2f°"%(t[0]-ins[0],t[1]-ins[1],((t[5]-ins[5]+180)%360)-180))
# ── 3. 스텝 삽입 (카메라 접촉 가드) ──
post('stop',{'dry_run':False}); time.sleep(3)
post('speed',{'value':1,'dry_run':False}); time.sleep(0.5)
cur=stt()['tcp']; z=cur[2]; prev=dots(); rate={}
# 초기 이동률: 첫 스텝을 4mm 로 짧게 가서 실측
def key(k,x,y): return (k,round(x/60),round(y/60))
def find_rate(k,x,y):
    c=[kv for kv in rate.items() if kv[0][0]==k]
    if not c: return None
    return min(c,key=lambda kv:(kv[0][1]-x/60)**2+(kv[0][2]-y/60)**2)[1]
ZT=float(sys.argv[3]) if len(sys.argv)>3 else 491.0
ok=True; first=True; t1=time.time()
while z>ZT+0.2:
    znext=max(ZT,z-(4.0 if first else 8.0))
    tgt=list(cur); tgt[2]=znext; post('move_tcp',{'tcp':tgt,'dry_run':False}); wait()
    f=stt(); zr=f['tcp'][2]; dz=z-zr
    if f['frozen']: print("🧊 동결!"); ok=False; break
    now=dots(); worst=0; wall=0; rep=[]; newrate={}
    for k,x,y in prev:
        r=find_rate(k,x,y)
        if k==ck and x>600: ex,ey=x,y
        elif r is None: ex,ey=None,None
        else: ex,ey=x+r[0]*dz,y+r[1]*dz
        cands=[q for q in now if q[0]==k]
        if not cands: rep.append(f"{k}({x:.0f},{y:.0f})→미검출"); continue
        if ex is None:   # 첫 스텝: 가장 가까운 점으로 이동률만 학습
            c=min(cands,key=lambda q:(q[1]-x)**2+(q[2]-y)**2); newrate[key(k,c[1],c[2])]=((c[1]-x)/dz,(c[2]-y)/dz); rep.append(f"{k} 이동률학습"); continue
        c=min(cands,key=lambda q:(q[1]-ex)**2+(q[2]-ey)**2)
        if (c[1]-ex)**2+(c[2]-ey)**2>40**2: rep.append(f"{k}({x:.0f},{y:.0f})→화면밖"); continue
        d=((c[1]-ex)**2+(c[2]-ey)**2)**0.5
        if k==ck and x>600: wall = 0.0 if first else max(wall,d)   # 첫 4mm 스텝은 벽점 검사 제외(571→567 에서 7.6px 체계적 튐, 2회 동일)
        else: worst=max(worst,d); newrate[key(k,c[1],c[2])]=((c[1]-x)/dz,(c[2]-y)/dz)
        rep.append(f"{k} {d:.1f}")
    print(f"  z{zr:.1f}: 강체잔차 {worst:.1f}px 벽점 {wall:.1f}px | "+" ".join(rep))
    if worst>5.0 or wall>6.0:
        if zr <= ZT+4.0:      # 목표 근방 접촉 = 바닥 안착(이번 벽은 여기서 다 들어감) → 1mm 만 올려 힘 빼고 성공 처리
            print(f"  ✔ z{zr:.1f} 바닥 접촉(안착) → 1mm 상승 후 놓기"); up=list(cur); up[2]=zr+1.0; post('move_tcp',{'tcp':up,'dry_run':False}); wait(); break
        print("⚠ 접촉 의심 → 정지, 10mm 상승"); up=list(cur); up[2]=zr+10; post('move_tcp',{'tcp':up,'dry_run':False}); wait(); ok=False; break
    rate.update(newrate); prev=now; z=zr; first=False
if ok:
    post('gripper',{'pos':50,'dry_run':False}); time.sleep(12); wait(30)
    g=post('grip_read',{'dry_run':True})['result']
    post('speed',{'value':3,'dry_run':False}); time.sleep(0.5)
    up=list(stt()['tcp']); up[2]+=60; post('move_tcp',{'tcp':up,'dry_run':False}); wait()
    print(f"[insert {time.time()-t1:.0f}s] ★ 삽입 완료 z491 → 그리퍼 {g} → 후퇴 z{stt()['tcp'][2]:.0f}")
post('speed',{'value':5,'dry_run':False})
print(f"총 {time.time()-T0:.0f}s, 결과: {'성공' if ok else '중단'}")
