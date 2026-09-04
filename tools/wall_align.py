"""8/30 ★벽 기준 정렬(wall-referenced align) — 카메라가 아니라 '벽'을 밑판에 맞춘다.

지금까지: 호버에서 밑판 강체점을 기준 픽셀로 되돌림 = 카메라를 밑판에 맞춤.
   → 벽이 그리퍼 안에서 티칭 때와 다르게 물리면(진자/미끄러짐) 그 차이가 그대로 삽입 오차.
   (카메라와 벽은 한 몸이라 팔을 움직여도 화면 속 벽 점은 안 움직인다 → 종전 정렬로는 못 잡는다.)
이 도구: 벽 점의 기준 대비 편차 ΔW 를 강체점 목표에 그대로 더한다.
   목표 P* = P_ref + ΔW   ⇒  (벽 − 밑판) 상대 배치가 성공 때와 같아진다.

  python3 wall_align.py <색> [--apply] [--descend <목표z>]
"""
import json, sys, time, urllib.request
import numpy as np

CK = sys.argv[1]
APPLY = "--apply" in sys.argv
DESC = float(sys.argv[sys.argv.index("--descend")+1]) if "--descend" in sys.argv else None
CAL = "/home/ar/bf2_console/dot_calib.json"
B = "http://localhost:8765"

def post(a, b):
    r = urllib.request.Request(f"{B}/fr5/{a}", data=json.dumps(b).encode(),
                               headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=25).read())
def stt(): return json.loads(urllib.request.urlopen(f"{B}/status", timeout=3).read())["robots"]["fr5"]
def wait(t=120):
    time.sleep(1.2)
    for _ in range(int(t/0.4)):
        if not stt()["busy"]: break
        time.sleep(0.4)
    time.sleep(0.8)
def dots(n=8):
    acc = {}
    for _ in range(n):
        for p in json.load(urllib.request.urlopen("http://localhost:8766/dots", timeout=4))["dots"]:
            acc.setdefault((p["kind"], round(p["px"]/60), round(p["py"]/60)), []).append((p["px"], p["py"]))
        time.sleep(0.08)
    return [(k[0], float(np.median([q[0] for q in v])), float(np.median([q[1] for q in v])))
            for k, v in acc.items() if len(v) >= n*0.5]

cal = json.load(open(CAL))
ref = cal["refs_place"][CK]
z_now = stt()["tcp"][2]
hr = min(ref["hover_refs"], key=lambda e: abs(e["z"] - z_now))
M = np.array(hr["M"], float); Minv = np.linalg.inv(M)
print(f"[{CK}] 현재 z {z_now:.1f} → 기준 z{hr['z']:.0f} 사용 (강체/벽 점 {len(hr['dots'])}개)")

cur = dots()
def match(rx, ry, kind):
    c = [d for d in cur if d[0] == kind]
    if not c: return None
    m = min(c, key=lambda d: (d[1]-rx)**2 + (d[2]-ry)**2)
    return m if (m[1]-rx)**2 + (m[2]-ry)**2 < 45**2 else None

wall_d, rigid = None, []
for d in hr["dots"]:
    m = match(d["px"], d["py"], d["kind"])
    tag = "벽 " if (d["kind"] == CK and d["px"] > 600) else "밑판"
    if m is None:
        print(f"  {tag} {d['kind']:6s} ref({d['px']:.0f},{d['py']:.0f}) → 미검출"); continue
    dx, dy = m[1]-d["px"], m[2]-d["py"]
    print(f"  {tag} {d['kind']:6s} ref({d['px']:6.1f},{d['py']:6.1f}) 현재({m[1]:6.1f},{m[2]:6.1f})  Δ({dx:+5.1f},{dy:+5.1f})px")
    if tag == "벽 ": wall_d = (dx, dy)
    else: rigid.append((dx, dy))

if not rigid: raise SystemExit("밑판 강체점 미검출 — 중단")
P = np.array([np.mean([r[0] for r in rigid]), np.mean([r[1] for r in rigid])])
W = np.array(wall_d) if wall_d else np.zeros(2)
if wall_d is None: print("  ⚠ 벽 점 미검출 → 종전 방식(강체만)으로 계산")
dW_mm = Minv @ W
print(f"\n  밑판 평균 편차 {P[0]:+.2f},{P[1]:+.2f}px   벽 편차 {W[0]:+.2f},{W[1]:+.2f}px "
      f"(= 파지가 기준 대비 X{dW_mm[0]:+.2f} Y{dW_mm[1]:+.2f}mm 어긋남)")
delta_px = W - P                      # 강체점을 P_ref+ΔW 로 보내기 위해 더 움직여야 하는 화면 이동
move = Minv @ delta_px                # 팔 +Δmm → 화면 속 물체 +Δpx (8/28 부호 실증)
print(f"  → 필요한 팔 이동  X {move[0]:+.3f}  Y {move[1]:+.3f} mm")
if max(abs(move)) > 4.0: raise SystemExit(f"보정량 {max(abs(move)):.1f}mm > 4mm — 오검출 의심, 중단")

if APPLY:
    t = list(stt()["tcp"]); t[0] += float(move[0]); t[1] += float(move[1])
    post("speed", {"value": 2, "dry_run": False}); time.sleep(0.3)
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    cur = dots()
    rr = []
    for d in hr["dots"]:
        if d["kind"] == CK and d["px"] > 600: continue
        m = match(d["px"], d["py"], d["kind"])
        if m: rr.append((m[1]-d["px"]-W[0], m[2]-d["py"]-W[1]))
    if rr:
        e = np.array([np.mean([r[0] for r in rr]), np.mean([r[1] for r in rr])])
        print(f"  적용 후 목표(P_ref+ΔW) 대비 잔차 {e[0]:+.2f},{e[1]:+.2f}px "
              f"({np.linalg.norm(Minv@e):.2f}mm)")
