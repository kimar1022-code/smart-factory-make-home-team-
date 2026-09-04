import json,time,urllib.request,statistics as st,shutil,sys
ck=sys.argv[1]; note=sys.argv[2] if len(sys.argv)>2 else ''
CAL='/home/ar/bf2_console/dot_calib.json'
def post(a,b):
    r=urllib.request.Request(f'http://localhost:8765/fr5/{a}',data=json.dumps(b).encode(),headers={'Content-Type':'application/json'}); return json.loads(urllib.request.urlopen(r,timeout=15).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status',timeout=3).read())['robots']['fr5']
def wait(tmax=90):
    time.sleep(2.0)
    for i in range(tmax*2):
        if not stt()['busy']: break
        time.sleep(0.5)
    time.sleep(0.8)
def dots(): return json.loads(urllib.request.urlopen('http://localhost:8766/dots',timeout=3).read())['dots']
def dd(a,b): return abs((a-b+180)%360-180)
post('gripper',{'pos':50,'dry_run':False}); time.sleep(12); wait(30); print(f"[{ck}] 놓기 → 실측",post('grip_read',{'dry_run':True})['result'])
z=stt()['tcp'][2]; post('cart_jog',{'axis':2,'delta':round(380.0-z,2),'dry_run':False}); wait()
print("z380 tcp",[round(v,1) for v in stt()['tcp']])
allp=[]
for i in range(12):
    allp+=[(p['px'],p['py']) for p in dots() if p['kind']==ck and p['px']>700]; time.sleep(0.1)
px=[round(st.median([q[0] for q in allp]),1),round(st.median([q[1] for q in allp]),1)]
snap=f'/home/ar/bf2_console/refs/{ck}_z380_ref.jpg'
try: shutil.copy(snap, snap.replace('.jpg','_prev.jpg'))
except Exception: pass
open(snap,'wb').write(urllib.request.urlopen('http://localhost:8766/raw',timeout=5).read())
c=json.load(open(CAL)); r=c['refs'][ck]
r['low_ref']={'px':px,'pts':[px],'theta_img':None,'img':snap,'z':380.0,'n':len(allp),'made':time.strftime('%Y-%m-%d %H:%M')+' 손확인 파지자세에서 수직 z380 캡처 '+note}
r['check_px']=px; r['offset_note']='8/28 손 티칭(그리퍼 50) '+note
json.dump(c,open(CAL,'w'),indent=1,ensure_ascii=False); print("low_ref",px,"n",len(allp))
post('cart_jog',{'axis':2,'delta':115.0,'dry_run':False}); wait()
post('dot_align',{'kind':ck,'ref':1,'station':'pick','dry_run':False}); wait(120)
c=json.load(open(CAL)); r=c['refs'][ck]; th=r['target_theta_deg']; rzt=r['pick_rz']
auto=min((th+90,th-90),key=lambda x:dd(x,rzt)); r['yaw_trim_deg']=round(((rzt-auto+180)%360)-180,3)
json.dump(c,open(CAL,'w'),indent=1,ensure_ascii=False)
print(f"[{ck}] 재저장 target_obs",r['target_obs'],"θ",th,"offset",r['pick_offset'],"yaw_trim",r['yaw_trim_deg'],"| 관측 tcp",[round(v,1) for v in stt()['tcp']])
