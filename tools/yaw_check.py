#!/usr/bin/env python3
"""9/3 z478 밑판 yaw 측정 검증(하강 없음, 빈 그리퍼 가능).
  python3 yaw_check.py <색> [--go] [--probe] [--n 5]
  --go   : 그 색 슬롯 상공 z478 홈포즈로 이동   --probe: J6 +0.5° 돌려 각차 감도 확인 후 원복"""
import sys, json, time, math
sys.path.insert(0, "/home/ar/bf2_console/tools")
import golden_place as GP
from golden import move_j6, wrap, st
import cycle_golden as C
ck = sys.argv[1]; n = int(sys.argv[sys.argv.index("--n")+1]) if "--n" in sys.argv else 5
cal = json.load(open(C.CAL)); ref = cal["refs"][ck]; it = ref["insert_tcp"]
rz = round(it[5]/90.0)*90.0; rz = 180.0 if rz == -180.0 else rz
if "--go" in sys.argv:
    C.go_z(max(C.st()["tcp"][2], C.SAFE_Z)); C.speed(C.FAST)
    C.post("move_tcp", {"tcp": [it[0], it[1], C.SAFE_Z, 180.0, 0.0, rz], "dry_run": False}); C.wait(90)
    C.go_z(478.0, spd=3); C.speed(1)
    C.post("move_tcp", {"tcp": [it[0], it[1], 478.0, 180.0, 0.0, rz], "dry_run": False}); C.wait(60)
print("tcp", [round(v, 2) for v in C.stable_tcp()])
A = {}
for r_ in ref["golden_raw"]["rows"]:
    if abs(r_["z"]-478) < 1:
        A = {c: dict(r_[c]["aruco"]) for c in ("cam1", "cam2") if r_[c]["aruco"]}
def measure():
    out = {}
    for cam in ("cam1", "cam2"):
        ams = {k: v for k, v in A.get(cam, {}).items() if int(k) < 50}
        cms = GP.aruco_now(cam, ams.keys()) if ams else {}
        d = {k: wrap(cms[k]["ang"] - ams[k]["ang"]) for k in cms}
        line = None
        ks = sorted(cms)
        if len(ks) >= 2:
            a, b = ks[0], ks[1]
            g = math.degrees(math.atan2(ams[b]["cy"]-ams[a]["cy"], ams[b]["cx"]-ams[a]["cx"]))
            c = math.degrees(math.atan2(cms[b]["cy"]-cms[a]["cy"], cms[b]["cx"]-cms[a]["cx"]))
            line = wrap(c - g)
        out[cam] = (d, line)
    return out
def show(tag, m):
    for cam in ("cam1", "cam2"):
        d, line = m[cam]
        print(f"  {tag} {cam}: 마커각차 " + " ".join(f"id{k} {v:+.2f}°" for k, v in d.items()) + (f" | 2마커 선각차 {line:+.2f}°" if line is not None else ""))
vals = {"cam1": [], "cam2": []}
for i in range(n):
    m = measure(); show(f"[{i+1}]", m)
    for cam in vals:
        d, line = m[cam]
        if d: vals[cam].append(sum(d.values())/len(d))
for cam in vals:
    v = vals[cam]
    if v: print(f"  {cam} 마커각차 평균 {sum(v)/len(v):+.2f}°  범위 {min(v):+.2f}~{max(v):+.2f}° (n={len(v)})")
def rot_rz(deg):
    t = list(C.stable_tcp()); t[5] = wrap(t[5] + deg)
    C.speed(1); C.post("move_tcp", {"tcp": t, "dry_run": False}); C.wait(60); time.sleep(0.5)
    return C.stable_tcp()
if "--probe" in sys.argv:
    t0 = C.stable_tcp(); l0 = measure()["cam1"][1]
    print("  직교 rz +0.5° 찔러보기 (관절 J6 는 왕복마다 TCP ~1mm 드리프트 → 사용 금지)")
    t1 = rot_rz(0.5); m = measure(); show("probe+0.5", m); l1 = m["cam1"][1]
    print(f"    TCP Δxyz {[round(t1[i]-t0[i],2) for i in range(3)]}  rz {t1[5]:+.2f}")
    if l0 is not None and l1 is not None:
        print(f"    cam1 선각차 {l0:+.2f} → {l1:+.2f}  gain {(l1-l0)/0.5:+.2f}°/°")
    C.speed(1); C.post("move_tcp", {"tcp": list(t0), "dry_run": False}); C.wait(60); time.sleep(0.5)
    t2 = C.stable_tcp(); m = measure(); show("복귀", m)
    print(f"    복귀 TCP Δ {[round(t2[i]-t0[i],2) for i in range(6)]}")
