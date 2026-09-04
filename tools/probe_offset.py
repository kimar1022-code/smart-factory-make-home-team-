"""8/30 삽입 오차 탐침 — 호버에서 XY 를 조금씩 바꿔가며 얼마나 내려가는지 실측.
어느 방향으로 몇 mm 어긋났는지 직접 답을 낸다(카메라 접촉 가드로 보호).

  python3 probe_offset.py <색> [축 x|y] [오프셋들…]     예) probe_offset.py red x -1.5 -0.75 0 0.75 1.5
매 시도: z571 복귀 → 벽 기준 정렬(P_ref+ΔW) → 오프셋 적용 → 4mm 스텝 하강(최대 16mm) → 복귀
"""
import json, sys, time, urllib.request
import numpy as np

CK = sys.argv[1]
AX = (sys.argv[2] if len(sys.argv) > 2 else "x").lower()
OFFS = [float(v) for v in sys.argv[3:]] or [-1.5, -0.75, 0.0, 0.75, 1.5]
CAL = "/home/ar/bf2_console/dot_calib.json"
B = "http://localhost:8765"
STEP, MAXDOWN = 4.0, 16.0
GUARD_RIGID, GUARD_WALL = 5.0, 8.0

def post(a, b):
    r = urllib.request.Request(f"{B}/fr5/{a}", data=json.dumps(b).encode(),
                               headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=25).read())
def stt(): return json.loads(urllib.request.urlopen(f"{B}/status", timeout=3).read())["robots"]["fr5"]
def wait(t=120):
    """동작 완료 + TCP 안정까지 대기.
    8/28 함정: 정지 직후 status tcp 는 최대 2mm 스테일 → 같은 값이 3번 연속 나올 때만 확정.
    (안 기다리면 dz 를 1.0mm 로 잘못 읽어 이동률 학습이 4배 틀어지고 가짜 접촉이 뜬다 — 8/30 실측)"""
    time.sleep(1.2)
    for _ in range(int(t/0.4)):
        if not stt()["busy"]: break
        time.sleep(0.4)
    last = None; same = 0
    for _ in range(40):
        time.sleep(0.25)
        z = round(stt()["tcp"][2], 2)
        same = same + 1 if (last is not None and abs(z - last) < 0.03) else 0
        last = z
        if same >= 2: return
    return
def dots(n=8):
    acc = {}
    for _ in range(n):
        for p in json.load(urllib.request.urlopen("http://localhost:8766/dots", timeout=4))["dots"]:
            acc.setdefault((p["kind"], round(p["px"]/60), round(p["py"]/60)), []).append((p["px"], p["py"]))
        time.sleep(0.08)
    return [(k[0], float(np.median([q[0] for q in v])), float(np.median([q[1] for q in v])))
            for k, v in acc.items() if len(v) >= n*0.5]
def near(cur, kind, x, y, r=60):
    c = [d for d in cur if d[0] == kind and (d[1]-x)**2 + (d[2]-y)**2 < r*r]
    return min(c, key=lambda d: (d[1]-x)**2 + (d[2]-y)**2) if c else None

cal = json.load(open(CAL)); ref = cal["refs_place"][CK]
hr = min(ref["hover_refs"], key=lambda e: abs(e["z"] - 571.0))
M = np.array(hr["M"], float); Minv = np.linalg.inv(M)
WREF = [d for d in hr["dots"] if d.get("role") == "wall"]
RREF = [d for d in hr["dots"] if d.get("role", "rigid") == "rigid"]
Z0 = float(hr["z"])

def align(iters=3):
    """벽 기준 정렬: 강체점을 P_ref+ΔW 로. 반환 잔차 px"""
    for _ in range(iters):
        cur = dots()
        ws = [(m[1]-d["px"], m[2]-d["py"]) for d in WREF
              if (m := near(cur, d["kind"], d["px"], d["py"])) and m[1] > 600]
        W = [float(np.mean([w[0] for w in ws])), float(np.mean([w[1] for w in ws]))] if ws else [0.0, 0.0]
        dl = [(m[1]-d["px"]-W[0], m[2]-d["py"]-W[1]) for d in RREF
              if (m := near(cur, d["kind"], d["px"]+W[0], d["py"]+W[1], 100))]
        if not dl: return None, W
        dpx = np.array([np.mean([d[0] for d in dl]), np.mean([d[1] for d in dl])])
        if np.linalg.norm(dpx) < 1.0: return float(np.linalg.norm(dpx)), W
        mm = -(Minv @ dpx)
        if max(abs(mm)) > 4: return None, W
        t = list(stt()["tcp"]); t[0] += float(mm[0]); t[1] += float(mm[1])
        post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    return float(np.linalg.norm(dpx)), W

def descend(base):
    """4mm 스텝 하강, 강체/벽 점 잔차로 접촉 감지. 반환 (도달 깊이mm, 사유)"""
    prev = dots(); rate = {}
    z = base[2]; gone = 0.0; first = True
    while gone < MAXDOWN - 0.1:
        d = min(STEP, MAXDOWN - gone)
        t = list(base); t[2] = z - d
        post("move_tcp", {"tcp": t, "dry_run": False}); wait()
        f = stt()
        if f["frozen"]: return gone, "동결"
        zr = f["tcp"][2]; dz = z - zr; now = dots()
        worst = wall = 0.0
        newrate = {}
        for k, x, y in prev:
            isw = (k == CK and x > 600)
            r = rate.get((k, round(x/60), round(y/60)))
            ex, ey = (x + r[0]*dz, y + r[1]*dz) if r else (None, None)
            c = near(now, k, ex if ex else x, ey if ey else y, 45)
            if not c: continue
            if ex is None:
                newrate[(k, round(c[1]/60), round(c[2]/60))] = ((c[1]-x)/dz, (c[2]-y)/dz); continue
            e = ((c[1]-ex)**2 + (c[2]-ey)**2) ** 0.5
            if isw: wall = max(wall, 0.0 if first else e)
            else:
                worst = max(worst, e)
                newrate[(k, round(c[1]/60), round(c[2]/60))] = ((c[1]-x)/dz, (c[2]-y)/dz)
        gone += dz
        print(f"      z{zr:.1f} (-{gone:.1f}mm) 강체 {worst:.1f}px 벽 {wall:.1f}px")
        if worst > GUARD_RIGID or wall > GUARD_WALL:
            return gone, f"접촉(강체{worst:.1f}/벽{wall:.1f})"
        rate.update(newrate); prev = now; z = zr; first = False
    return gone, "끝까지"

post("speed", {"value": 2, "dry_run": False}); time.sleep(0.3)
res = []
for off in OFFS:
    t = list(stt()["tcp"]); t[2] = Z0
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    r, W = align()
    print(f"\n── {AX}{off:+.2f}mm  (정렬 잔차 {r}px, 파지편차 {[round(v,1) for v in W]}px)")
    t = list(stt()["tcp"]); t[0 if AX == "x" else 1] += off
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    base = list(stt()["tcp"])
    depth, why = descend(base)
    print(f"   → {AX}{off:+.2f}: {depth:.1f}mm 하강 후 {why}")
    res.append((off, depth, why))
    up = list(base); up[2] = Z0
    post("move_tcp", {"tcp": up, "dry_run": False}); wait()
print("\n=== 결과 ===")
for off, d, why in res: print(f"  {AX}{off:+6.2f}mm → {d:5.1f}mm  {why}")
