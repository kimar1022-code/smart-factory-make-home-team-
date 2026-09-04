#!/usr/bin/env python3
"""9/3: 슬롯 상공 z478 에서 두 캠 마커(6프레임 평균)를 골든 z478 과 대조 — 이웃 벽이 밀어낸 마커 찾기.
  python3 marker_check.py <색> [--no-move]   (벽 든 채 z443 정지 상태에서 호출: z478 로 올려 측정)"""
import sys, json, time, math, urllib.request
sys.path.insert(0, "/home/ar/bf2_console/tools")
import cycle_golden as C
ck = sys.argv[1]
if "--no-move" not in sys.argv:
    C.speed(1); C.go_z(478.0, spd=1); time.sleep(0.8)
print("tcp", [round(v, 2) for v in C.st()["tcp"]])
cal = json.load(open(C.CAL)); r = cal["refs"][ck]
gold = {}
for row in r["golden_raw"]["rows"]:
    if abs(row["z"] - 478) < 1:
        gold = {c: row[c]["aruco"] for c in ("cam1", "cam2")}
def read(port, n=6):
    acc = {}
    for i in range(n):
        j = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/aruco", timeout=4).read())
        for m in j["markers"]:
            c = m["corners"]; cx = sum(p[0] for p in c) / 4; cy = sum(p[1] for p in c) / 4
            ang = math.degrees(math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0]))
            acc.setdefault(str(m["id"]), []).append((cx, cy, ang))
        time.sleep(0.15)
    return {k: (sum(a[0] for a in v) / len(v), sum(a[1] for a in v) / len(v), sum(a[2] for a in v) / len(v), len(v)) for k, v in acc.items()}
wrap = lambda a: (a + 180) % 360 - 180
for cam, port in (("cam1", 8766), ("cam2", 8768)):
    now = read(port)
    for mid, g in gold.get(cam, {}).items():
        n = now.get(mid)
        if n:
            print(f"{cam} id{mid}: 골든 ({g['cx']:.1f},{g['cy']:.1f}) ang {g['ang']:+.2f} | 지금 ({n[0]:.1f},{n[1]:.1f}) ang {n[2]:+.2f} | Δpx ({n[0]-g['cx']:+.1f},{n[1]-g['cy']:+.1f}) Δang {wrap(n[2]-g['ang']):+.2f}° (n={n[3]})")
        else:
            print(f"{cam} id{mid}: 골든에 있는데 지금 안 보임")
    print(f"{cam} 지금 보이는 id: {sorted(now, key=int)}")
