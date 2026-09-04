#!/usr/bin/env python3
"""J6(손목 yaw) ↔ 두 카메라 '물고 있는 벽 점' 픽셀 감도 캘리브 — 9/1.

    python3 yaw6_calib.py [--kind blue] [--step 2.0]

왜: 벽이 그리퍼 안에서 yaw 로 비틀려 물리면 삽입이 실패한다(8/30, 5연속 0/2).
    카메라 1대일 땐 "점 하나로 yaw 를 못 잰다"가 원리적 한계였는데, 손목에 카메라가
    2대가 되면서 서로 다른 각도에서 같은 벽을 보므로 역산이 가능해졌다.

원리: J6 를 ±step° 돌리며 **물고 있는 벽 점**(= 팔과 한 몸이라 높이를 바꿔도 화면에서
    안 움직이는 점)의 픽셀 변화를 잰다. yaw 1° 당 (dx1,dy1,dx2,dy2) 4차원 감도 벡터를 얻으면,
    파지할 때마다 관측된 편차를 최소자승으로 θ 에 투영해 J6 보정량을 계산할 수 있다.

★안전: J6 만 돌린다(직교 위치 이동 없음). 벽이 뭔가에 닿지 않는 높은 자세에서 실행할 것.
결과: dot_calib.json 의 yaw6_* 키.
"""
import argparse
import json
import sys
import time
import urllib.request

import numpy as np

BRIDGE = "http://localhost:8765"
CAL = "/home/ar/bf2_console/dot_calib.json"
CAM1, CAM2 = "http://localhost:8766", "http://localhost:8768"


def post(a, b):
    r = urllib.request.Request(f"{BRIDGE}/fr5/{a}", json.dumps(b).encode(),
                               {"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=40).read())


def st():
    return json.loads(urllib.request.urlopen(BRIDGE + "/status", timeout=5).read())["robots"]["fr5"]


def wait(t=90):
    time.sleep(0.9)
    t0 = time.time()
    while time.time() - t0 < t and st()["busy"]:
        time.sleep(0.3)


def read_dot(cam, kind, near, n=7, r=90.0):
    """near(px) 주변에서 kind 색 점 1개 — n회 중앙값. 물고 있는 벽 점을 추적."""
    acc = []
    for _ in range(n * 6):
        try:
            ds = json.loads(urllib.request.urlopen(cam + "/dots?raw=1", timeout=5).read())["dots"]
        except Exception:
            time.sleep(0.2)
            continue
        hit = [t for t in ds if t["kind"] == kind
               and (t["px"] - near[0]) ** 2 + (t["py"] - near[1]) ** 2 < r * r]
        if hit:
            hit.sort(key=lambda t: (t["px"] - near[0]) ** 2 + (t["py"] - near[1]) ** 2)
            acc.append([hit[0]["px"], hit[0]["py"]])
            if len(acc) >= n:
                break
        time.sleep(0.14)
    if len(acc) < 3:
        raise RuntimeError(f"{cam} 에서 {kind} 점 부족 {len(acc)}/{n} (기준 {near})")
    return np.median(np.array(acc, float), axis=0)


def find_held(rows, cam, kind):
    """기록된 하강 기준에서 '높이를 바꿔도 안 움직이는' 점 = 물고 있는 벽 점."""
    hi, lo = rows[0][cam], rows[-1][cam]
    best = None
    for p in hi:
        if p[0] != kind:
            continue
        cand = [q for q in lo if q[0] == kind]
        if not cand:
            continue
        q = min(cand, key=lambda q: (q[1] - p[1]) ** 2 + (q[2] - p[2]) ** 2)
        mv = ((q[1] - p[1]) ** 2 + (q[2] - p[2]) ** 2) ** 0.5
        if mv < 8 and (best is None or mv < best[0]):
            best = (mv, [p[1], p[2]])
    return best[1] if best else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kind", default="blue")
    ap.add_argument("--step", type=float, default=2.0, help="J6 회전량(도). ±step 왕복")
    a = ap.parse_args()

    cal = json.load(open(CAL))
    ref = cal["refs"][a.kind]
    rows = ref["descent_refs_place"]["rows"]
    near1 = find_held(rows, "cam1", a.kind)
    near2 = find_held(rows, "cam2", a.kind)
    if not near1 or not near2:
        sys.exit(f"물고 있는 벽 점을 못 찾음 (cam1={near1}, cam2={near2}) — "
                 f"descent_refs_place 를 벽 문 채로 기록했는지 확인")
    print(f"물고 있는 벽 점: 손목캠 {near1}  새카메라 {near2}")

    j0 = list(st()["joints"])
    print(f"J6 시작값 {j0[5]:.3f}°  (±{a.step}° 왕복, 위치 이동 없음)")
    post("speed", {"value": 2, "dry_run": False}); time.sleep(0.3)

    def measure():
        time.sleep(0.7)
        return read_dot(CAM1, a.kind, near1), read_dot(CAM2, a.kind, near2)

    p1_0, p2_0 = measure()
    print(f"  기준  손목캠({p1_0[0]:.1f},{p1_0[1]:.1f})  새캠({p2_0[0]:.1f},{p2_0[1]:.1f})")

    per = []
    for d in (+a.step, -a.step):
        j = list(j0); j[5] = j0[5] + d
        post("move", {"joints": [round(v, 3) for v in j], "dry_run": False}); wait()
        p1, p2 = measure()
        # 추적 기준을 원위치로 되돌려 다음 측정이 엉뚱한 점을 잡지 않게
        v = np.array([p1[0]-p1_0[0], p1[1]-p1_0[1], p2[0]-p2_0[0], p2[1]-p2_0[1]]) / d
        per.append(v)
        print(f"  J6{d:+.1f}° → deg당 손목캠({v[0]:+.2f},{v[1]:+.2f}) 새캠({v[2]:+.2f},{v[3]:+.2f}) px/°")
        post("move", {"joints": [round(v2, 3) for v2 in j0], "dry_run": False}); wait()

    S = np.mean(per, axis=0)
    consist = float(np.linalg.norm(per[0] - per[1]) / max(np.linalg.norm(S), 1e-6))
    p1_e, p2_e = measure()
    drift = float(np.linalg.norm([p1_e[0]-p1_0[0], p1_e[1]-p1_0[1],
                                  p2_e[0]-p2_0[0], p2_e[1]-p2_0[1]]))
    print(f"감도 S = ({S[0]:+.2f},{S[1]:+.2f} | {S[2]:+.2f},{S[3]:+.2f}) px/°   "
          f"크기 {np.linalg.norm(S):.2f}px/°")
    print(f"±방향 불일치 {100*consist:.0f}%  ·  복귀 잔차 {drift:.1f}px")

    cal["yaw6_made"] = time.strftime("%Y-%m-%d %H:%M")
    cal["yaw6_kind"] = a.kind
    cal["yaw6_step_deg"] = a.step
    cal["yaw6_held_px"] = {"cam1": [round(v, 1) for v in near1], "cam2": [round(v, 1) for v in near2]}
    cal["yaw6_sens_px_per_deg"] = [round(float(v), 4) for v in S]   # [dx1,dy1,dx2,dy2]
    cal["yaw6_consistency"] = round(consist, 3)
    cal["yaw6_return_drift_px"] = round(drift, 1)
    cal["yaw6_note"] = ("물고 있는 벽 점의 J6 1° 당 픽셀 변화(두 카메라 4차원). "
                        "파지 편차 v=[dx1,dy1,dx2,dy2] 관측 시 θ ≈ (S·v)/(S·S) 도. "
                        "★이 값은 파지 자세(카메라~벽 거리)에 의존 — 자세가 크게 바뀌면 재측정.")
    json.dump(cal, open(CAL, "w"), indent=1, ensure_ascii=False)
    print("저장: dot_calib.json yaw6_*")
    if np.linalg.norm(S) < 1.0:
        print("⚠ 감도가 너무 작다(<1px/°) — step 을 키우거나 벽 점이 화면 중앙에서 먼지 확인")
    if consist > 0.35:
        print("⚠ ± 방향 불일치 큼 — 백래시/추적 오류 의심, 재실행 권장")


if __name__ == "__main__":
    main()
