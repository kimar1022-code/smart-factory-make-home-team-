"""하강하며 모든 점의 '원시 픽셀'을 그대로 기록 — 벽 점 튐이 진짜 미끄러짐인지 검출 아티팩트인지 판정.
  python3 watch_down.py <색> <최저z> [스텝mm]
안전: 밑판 강체점이 예상 대비 6px 넘게 밀리면 즉시 정지·상승."""
import json, sys, time, urllib.request
import numpy as np
CK = sys.argv[1]; ZMIN = float(sys.argv[2]); STEP = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
def post(a, b):
    r = urllib.request.Request(f'http://localhost:8765/fr5/{a}', data=json.dumps(b).encode(),
                               headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(r, timeout=25).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status', timeout=3).read())['robots']['fr5']
def wait(t=120):
    time.sleep(1.2)
    for _ in range(int(t/0.4)):
        if not stt()['busy']: break
        time.sleep(0.4)
    last = None; same = 0
    for _ in range(40):
        time.sleep(0.25); z = round(stt()['tcp'][2], 2)
        same = same + 1 if (last is not None and abs(z-last) < 0.03) else 0; last = z
        if same >= 2: return
def dots(n=8):
    acc = {}
    for _ in range(n):
        for p in json.load(urllib.request.urlopen('http://localhost:8766/dots', timeout=4))['dots']:
            acc.setdefault((p['kind'], round(p['px']/60), round(p['py']/60)), []).append((p['px'], p['py'], p['area']))
        time.sleep(0.07)
    return [(k[0], float(np.median([q[0] for q in v])), float(np.median([q[1] for q in v])),
             float(np.median([q[2] for q in v])), len(v)) for k, v in acc.items() if len(v) >= n*0.4]
post('speed', {'value': 1, 'dry_run': False}); time.sleep(0.4)
base = list(stt()['tcp']); z = base[2]
prev = None
while z > ZMIN + 0.2:
    t = list(base); t[2] = max(ZMIN, z - STEP)
    post('move_tcp', {'tcp': t, 'dry_run': False}); wait()
    z = stt()['tcp'][2]
    cur = dots()
    ws = [d for d in cur if d[0] == CK and 600 < d[1] < 1150]
    rs = [d for d in cur if not (d[0] == CK and 600 < d[1] < 1150)]
    wtxt = " ".join(f"({d[1]:.0f},{d[2]:.0f})a{d[3]:.0f}n{d[4]}" for d in sorted(ws, key=lambda q: -q[3]))
    rtxt = " ".join(f"{d[0][:3]}({d[1]:.0f},{d[2]:.0f})" for d in sorted(rs, key=lambda q: q[1]))
    print(f"z{z:6.1f} | 벽 {wtxt or '없음':38s} | 밑판 {rtxt}")
    if prev:
        bad = 0
        for k, x, y, a, n in prev[1]:
            c = [d for d in rs if d[0] == k and (d[1]-x)**2 + (d[2]-y)**2 < 60**2]
            if c:
                d = min(c, key=lambda q: (q[1]-x)**2 + (q[2]-y)**2)
                bad = max(bad, ((d[1]-x)**2 + (d[2]-y)**2) ** 0.5)
        if bad > 12:
            print(f"  ⚠ 밑판 {bad:.1f}px 밀림 → 정지"); break
    prev = (ws, rs)
up = list(base); up[2] = min(base[2], stt()['tcp'][2] + 10)
post('move_tcp', {'tcp': up, 'dry_run': False}); wait()
print("상승 완료 z", round(stt()['tcp'][2], 1))
