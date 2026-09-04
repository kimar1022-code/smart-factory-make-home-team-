#!/usr/bin/env python3
"""단방향 삽입 — 랙에서 집어 밑판에 꽂고 놓기까지. 9/1.
★그리퍼 명령을 2회(파지·놓기)로 최소화 — 오늘 동결 3회가 전부 그리퍼 명령 직후였다.
   조회도 꼭 필요한 지점에서만(파지 확인·놓기 확인).
    python3 oneway.py [색]
"""
import json, subprocess, sys, time, urllib.request
B="http://localhost:8765"; CAL="/home/ar/bf2_console/dot_calib.json"; HERE="/home/ar/bf2_console/tools"
def post(a,b):
    r=urllib.request.Request(f"{B}/fr5/{a}", json.dumps(b).encode(), {"Content-Type":"application/json"})
    return json.loads(urllib.request.urlopen(r,timeout=45).read())
def st(): return json.loads(urllib.request.urlopen(B+"/status",timeout=5).read())["robots"]["fr5"]
def wait(t=150):
    time.sleep(0.9); t0=time.time()
    while time.time()-t0<t and st()["busy"]: time.sleep(0.35)
def stable(n=4):
    p=None;s=0
    for _ in range(50):
        t=st()["tcp"]
        if p and all(abs(a-b)<0.05 for a,b in zip(t,p)): s+=1
        else: s=0
        p=t
        if s>=n: break
        time.sleep(0.3)
    return p
def spd(v): post("speed",{"value":v,"dry_run":False}); time.sleep(0.25)
def goz(z,v=2,tol=2.0):
    spd(v); t=list(st()["tcp"]); t[2]=float(z)
    post("move_tcp",{"tcp":t,"dry_run":False}); wait()
    g=stable()[2]
    if abs(g-z)>tol: raise RuntimeError(f"z 미도달 목표{z:.0f} 도달{g:.1f}")
def goxy(tc,z,v=2):
    spd(v); post("move_tcp",{"tcp":[tc[0],tc[1],z,tc[3],tc[4],tc[5]],"dry_run":False}); wait()
def run(sc,*a):
    r=subprocess.run([sys.executable,f"{HERE}/{sc}"]+list(a),capture_output=True,text=True,timeout=600,cwd=HERE)
    return ("완료:" in r.stdout or "✔ 수렴" in r.stdout), r.stdout.strip()
ck=sys.argv[1] if len(sys.argv)>1 else "blue"
cal=json.load(open(CAL)); tt=cal["refs"][ck]["pick_tcp_taught"]; it=cal["refs"][ck]["insert_tcp"]
gc=int(cal["refs"][ck].get("grip_close",13))
print(f"[{ck}] 단방향 삽입 시작")
goz(max(st()["tcp"][2],650.0))
post("move",{"joints":cal["observe_joints"],"dry_run":False}); wait(90)
goxy(tt,520.0)
ok,out=run("descend_ref.py","play",ck,"--nocam2")
if not ok: sys.exit("픽 하강 실패\n"+out[-300:])
print("  픽 도달",[round(v,2) for v in stable()[:3]])
post("gripper",{"pos":gc,"dry_run":False}); time.sleep(9.5)
g=int(str(post("grip_read",{"dry_run":True})["result"] or 0)); print(f"  파지 실측 {g}")
if g<10: sys.exit("헛집음 — 중단")
goz(425.0,1); goz(650.0)
goxy(it,650.0); goxy(it,478.0)
ok,out=run("yaw_measure.py","fix","--use","A1_rel","--tol","0.05")
print("  yaw:", [l.strip() for l in out.splitlines() if "편차" in l or "수렴" in l][-1] if out else "?")
ok,out=run("descend_ref.py","play",ck,"--place")
if not ok: sys.exit("place 하강 실패\n"+out[-400:])
for l in out.splitlines():
    if l.strip().startswith("z"): print("  "+l.strip())
p=stable(); print("  삽입 도달",[round(v,2) for v in p[:3]],"Δ",[round(p[k]-it[k],2) for k in range(3)])
post("gripper",{"pos":30,"dry_run":False}); time.sleep(9.5)
g=int(str(post("grip_read",{"dry_run":True})["result"] or 0)); print(f"  놓기 실측 {g}")
goz(478.0,1)
print("✅ 삽입 완료 — 벽이 밑판에 꽂혔는지 확인하세요")
