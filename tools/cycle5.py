"""8/30 5연속 무개입 프로토콜 — 사람은 벽을 랙에 되돌리기만 한다.
회차: (랙에 벽 돌아옴 감지) → pick → place → 순응 삽입 → 놓기·후퇴 → 다음 회차 대기
  python3 cycle5.py <색> <목표z> [횟수]
"""
import json, subprocess, sys, time, urllib.request
import numpy as np
CK = sys.argv[1]; ZT = sys.argv[2]; N = int(sys.argv[3]) if len(sys.argv) > 3 else 5
HERE = "/home/ar/bf2_console/tools"
LOG = f"/home/ar/bf2_console/tools/{CK}_5x_0830.log"
def post(a, b):
    r = urllib.request.Request(f'http://localhost:8765/fr5/{a}', data=json.dumps(b).encode(),
                               headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(r, timeout=30).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status', timeout=3).read())['robots']['fr5']
def wait_busy(t=250):
    time.sleep(1.2)
    for _ in range(int(t/0.4)):
        if not stt()['busy']: break
        time.sleep(0.4)
    time.sleep(0.8)
def say(m):
    print(m, flush=True)
    open(LOG, 'a').write(time.strftime('%H:%M:%S ') + m + "\n")
cal = json.load(open('/home/ar/bf2_console/dot_calib.json'))
XB = (cal.get('pick_xband') or {}).get(CK)
GAP = float(cal['refs'][CK]['end_gap_px'])
def wall_in_rack(n=6):
    """관측 자세에서 이 색 벽의 3점(밴드 안·간격 기준±25%)이 보이면 랙에 돌아온 것."""
    acc = {}
    for _ in range(n):
        for p in json.load(urllib.request.urlopen('http://localhost:8766/dots', timeout=4))['dots']:
            if p['kind'] != CK: continue
            if XB and not (XB[0] <= p['px'] <= XB[1]): continue
            acc.setdefault(round(p['py']/60), []).append((p['px'], p['py']))
        time.sleep(0.1)
    pts = [(float(np.median([q[0] for q in v])), float(np.median([q[1] for q in v])))
           for v in acc.values() if len(v) >= n*0.5]
    if len(pts) < 3: return False, len(pts), None
    pts.sort(key=lambda q: q[1])
    g = ((pts[-1][0]-pts[0][0])**2 + (pts[-1][1]-pts[0][1])**2) ** 0.5
    return abs(g-GAP) < 0.25*GAP, len(pts), g
res = []
for i in range(1, N+1):
    say(f"\n━━━━ {i}/{N} 회차 ━━━━")
    post('speed', {'value': 4, 'dry_run': False}); time.sleep(0.3)
    t = list(stt()['tcp']); t[2] = max(t[2], 620.0)
    post('move_tcp', {'tcp': t, 'dry_run': False}); wait_busy()
    # ★8/30: 글로벌캠(탑뷰)으로 감시 — 팔을 관측 자세로 보낼 필요 없이 랙을 직접 본다.
    #   글로벌캠이 죽어 있으면 종전대로 관측 자세에서 손목캠으로 감시.
    use_global = False
    try:
        import global_lib as GL
        GL.grab(timeout=20); use_global = True
    except Exception as e:
        say(f"  글로벌캠 사용 불가({str(e)[:50]}) → 손목캠 감시로 대체")
    if not use_global:
        post('move', {'joints': cal['observe_joints'], 'dry_run': False}); wait_busy()
    say(f"  랙에 벽 돌아오기를 기다리는 중 ({'글로벌캠 탑뷰' if use_global else '관측 자세 손목캠'}, 최대 10분)")
    t0 = time.time(); seen = False
    while time.time() - t0 < 600:
        if use_global:
            try:
                ok, n, mmpx = GL.wall_in_rack(CK)
                if ok:
                    say(f"  ✔ 글로벌캠 벽 감지 (점 {n}개, 축척 {mmpx:.3f}mm/px) — 5초 후 시작"); seen = True; break
            except Exception as e:
                say(f"  글로벌캠 오류({str(e)[:40]}) → 손목캠으로 전환")
                post('move', {'joints': cal['observe_joints'], 'dry_run': False}); wait_busy()
                use_global = False
        else:
            ok, n, g = wall_in_rack()
            if ok:
                say(f"  ✔ 벽 감지 (3점, 간격 {g:.0f}px / 기준 {GAP:.0f}px) — 5초 후 시작"); seen = True; break
        time.sleep(3)
    if not seen:
        say("  ✘ 10분 대기 초과 — 중단"); res.append((i, '대기초과', None)); break
    time.sleep(5)
    t1 = time.time()
    p = subprocess.run([sys.executable, f"{HERE}/pick_place.py", CK], capture_output=True, text=True, timeout=600)
    for ln in p.stdout.strip().splitlines()[-8:]: say("  " + ln[:150])
    if p.returncode != 0:
        say(f"  ✘ pick/place 실패: {p.stdout.strip().splitlines()[-1][:120] if p.stdout.strip() else p.stderr[:120]}")
        res.append((i, 'pick/place 실패', round(time.time()-t1))); continue
    q = subprocess.run([sys.executable, f"{HERE}/insert_search.py", CK, ZT], capture_output=True, text=True, timeout=900)
    for ln in q.stdout.strip().splitlines()[-6:]: say("  " + ln[:150])
    ok = ('성공' in q.stdout and q.returncode == 0)
    say(f"  {'★ 성공' if ok else '✘ 실패'}  ({time.time()-t1:.0f}s)")
    res.append((i, '성공' if ok else '실패', round(time.time()-t1)))
say("\n═══ 결과 ═══")
for i, r, s in res: say(f"  {i}회차: {r}" + (f"  {s}s" if s else ""))
say(f"  성공 {sum(1 for _,r,_ in res if r=='성공')}/{len(res)}")
