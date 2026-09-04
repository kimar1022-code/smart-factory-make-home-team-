#!/usr/bin/env python3
"""골든 픽 반복 검증 — 파지 없이 정렬만 N회 (9/1 밤, 1mm 스텝판).

    python3 pick_repeat.py [n]        # 기본 10회

1회 = 안전고도 → 관측자세 → 픽 상공 z520 → golden.py run pick → 결과 기록 → 다시 상공.
★그리퍼를 건드리지 않는다(golden.py 도 안 건드림). 벽은 랙에 그대로 → 사람이 손댈 일 없음.

재현성 판정 = 매회 '최종 TCP'가 서로 얼마나 일치하는가.
벽이 안 움직였다면 매번 같은 자리에 서야 한다. 그 산포가 곧 픽의 반복 정밀도다.
"""
import json
import os
import statistics as S
import subprocess
import sys
import time
import urllib.request

BRIDGE = "http://localhost:8765"
CAL = "/home/ar/bf2_console/dot_calib.json"
HERE = "/home/ar/bf2_console/tools"
LOGDIR = "/home/ar/bf2_console/logs"
SAFE_Z, APPROACH_Z = 650.0, 520.0
FAST = 15                 # 안전고도 이동만 — 파지물 없음, 정확도 무관


def post(a, b):
    r = urllib.request.Request(f"{BRIDGE}/fr5/{a}", json.dumps(b).encode(),
                               {"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=45).read())


def st():
    return json.loads(urllib.request.urlopen(BRIDGE + "/status", timeout=5).read())["robots"]["fr5"]


def wait(t=150):
    time.sleep(0.45)
    t0 = time.time()
    while time.time() - t0 < t and st()["busy"]:
        time.sleep(0.2)


def stable_tcp(n=3):
    prev, same = None, 0
    for _ in range(60):
        t = st()["tcp"]
        if prev and all(abs(a - b) < 0.05 for a, b in zip(t, prev)):
            same += 1
        else:
            same = 0
        prev = t
        if same >= n:
            break
        time.sleep(0.15)
    return prev


def speed(v):
    post("speed", {"value": v, "dry_run": False}); time.sleep(0.15)


def go_z(z, spd=FAST):
    speed(spd)
    t = list(st()["tcp"]); t[2] = float(z)
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()


def go_xy(tcp, z, spd=FAST):
    speed(spd)
    post("move_tcp", {"tcp": [tcp[0], tcp[1], z, tcp[3], tcp[4], tcp[5]], "dry_run": False})
    wait()


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    cal = json.load(open(CAL))
    tt = cal["refs"]["blue"]["pick_tcp_taught"]
    os.makedirs(LOGDIR, exist_ok=True)

    g = int(str(post("grip_read", {"dry_run": True})["result"] or 0))
    if abs(g - 30) > 2:
        sys.exit(f"그리퍼가 30 이 아님(실측 {g}) — 빈 그리퍼로만 반복 검증합니다. 중단")

    oks, tcps, secs = 0, [], []
    for i in range(1, n + 1):
        t0 = time.time()
        print(f"\n━━━━━ 픽 {i}/{n} ━━━━━", flush=True)
        go_z(max(st()["tcp"][2], SAFE_Z))
        post("move", {"joints": cal["observe_joints"], "dry_run": False}); wait(90)
        go_xy(tt, APPROACH_Z)
        r = subprocess.run([sys.executable, "-u", f"{HERE}/golden.py", "run", "pick", "blue"],
                           capture_output=True, text=True, timeout=900, cwd=HERE)
        out = (r.stdout + r.stderr).strip()
        with open(f"{LOGDIR}/pick_rep_{i}.log", "w") as f:
            f.write(out + "\n")
        ok = "완료(골든 일치)" in out and r.returncode == 0
        p = stable_tcp()
        dt = time.time() - t0
        secs.append(dt)
        stages = [l.strip() for l in out.splitlines() if "✔ 골든 일치" in l]
        last = [l.strip() for l in out.splitlines() if l.strip().startswith("골든z")]
        if ok:
            oks += 1
            tcps.append(p[:3])
            print(f"  ✅ 통과 · 단계 {len(stages)}/4 · 최종 TCP "
                  f"[{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}] · {dt:.0f}초", flush=True)
            if last:
                print(f"     {last[-1]}", flush=True)
        else:
            tail = [l.strip() for l in out.splitlines() if l.strip()][-2:]
            print(f"  ❌ 실패 · 단계 {len(stages)}/4 · {dt:.0f}초", flush=True)
            for l in tail:
                print(f"     {l}", flush=True)
            print("  로봇 그 자리 정지 — 그리퍼 무조작(벽은 랙에 그대로)", flush=True)
            break
        go_z(APPROACH_Z, spd=6)          # 다음 회차를 위해 상공 복귀(벽에서 멀어지는 방향)

    print(f"\n━━━ 결과: {oks}/{n} 통과 ━━━")
    if secs:
        print(f"    회당 {S.mean(secs):.0f}초 (최소 {min(secs):.0f} · 최대 {max(secs):.0f})")
    if len(tcps) >= 2:
        for k, nm in ((0, "X"), (1, "Y")):
            v = [t[k] for t in tcps]
            print(f"    최종 {nm}: 평균 {S.mean(v):.3f} · 산포 {max(v)-min(v):.3f}mm "
                  f"· 표준편차 {S.pstdev(v):.3f}mm")
        print("    ※ 산포가 곧 픽 반복 정밀도(벽이 안 움직였다는 전제)")


if __name__ == "__main__":
    main()
