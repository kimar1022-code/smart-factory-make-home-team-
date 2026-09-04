#!/usr/bin/env python3
"""블루 벽 왕복 풀사이클 — 9/1.

  1회 = 랙에서 집기 → 운반 → yaw 보정 → 밑판 삽입 → 놓기 → 다시 빼기 → 랙 반납

    python3 cycle_blue.py [n]        # n회 반복(기본 5), 매 회 수치 로그

시작 조건: 벽이 랙 슬롯에 꽂혀 있고 그리퍼는 비어 있음(어디에 있든 무방, 안전고도로 올린 뒤 시작).
★반납은 티칭 좌표로 직접 내려간다 — 픽 기준 사진은 '빈 그리퍼' 기준이라 벽을 물고 내려가면
  매칭이 안 된다(9/1 실증: 그 상태에서 그리퍼를 열어 벽을 125mm 높이에서 떨어뜨렸다).
★하강이 성공했는지 확인하기 전에는 절대 그리퍼를 열지 않는다.
"""
import json
import subprocess
import sys
import time
import urllib.request

BRIDGE = "http://localhost:8765"
CAL = "/home/ar/bf2_console/dot_calib.json"
HERE = "/home/ar/bf2_console/tools"
SAFE_Z, HOVER_Z, SLOW_Z = 650.0, 478.0, 425.0


def post(a, b):
    r = urllib.request.Request(f"{BRIDGE}/fr5/{a}", json.dumps(b).encode(),
                               {"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=45).read())


def st():
    return json.loads(urllib.request.urlopen(BRIDGE + "/status", timeout=5).read())["robots"]["fr5"]


def wait(t=150):
    time.sleep(0.9)
    t0 = time.time()
    while time.time() - t0 < t and st()["busy"]:
        time.sleep(0.35)


def stable_tcp(n=4):
    prev, same = None, 0
    for _ in range(50):
        t = st()["tcp"]
        if prev and all(abs(a - b) < 0.05 for a, b in zip(t, prev)):
            same += 1
        else:
            same = 0
        prev = t
        if same >= n:
            break
        time.sleep(0.3)
    return prev


def speed(v):
    post("speed", {"value": v, "dry_run": False}); time.sleep(0.25)


def go_z(z, spd=2, verify=True, tol=2.0):
    """★9/1: 하강이 중간에 멈췄는데도 다음 단계(그리퍼 열기)로 넘어가 벽을 떨어뜨릴 뻔했다.
    도달 z 를 반드시 확인한다."""
    speed(spd)
    t = list(st()["tcp"]); t[2] = float(z)
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    if verify:
        got = stable_tcp()[2]
        if abs(got - z) > tol:
            raise RuntimeError(f"z 이동 미완: 목표 {z:.1f} → 도달 {got:.1f} (차 {got-z:+.1f}mm)")


def go_xy(tcp, z, spd=2):
    speed(spd)
    post("move_tcp", {"tcp": [tcp[0], tcp[1], z, tcp[3], tcp[4], tcp[5]], "dry_run": False}); wait()


def grip(pos, expect=None, tag=""):
    post("gripper", {"pos": pos, "dry_run": False})
    time.sleep(9.5)
    g = int(str(post("grip_read", {"dry_run": True})["result"]))
    if expect is not None and abs(g - expect) > 2:
        raise RuntimeError(f"{tag} 그리퍼 실측 {g} (기대 {expect}) — 중단")
    return g


def ensure_open_before_descend(tag=""):
    """★사용자 규칙(9/1): '내려올 땐 무조건 그리퍼 30'.
    빈 그리퍼가 닫힌 채로 내려가면 손가락이 벽/기둥 위에 얹혀 부러뜨린다."""
    g = int(str(post("grip_read", {"dry_run": True})["result"] or 0))
    if g != 30:
        post("gripper", {"pos": 30, "dry_run": False})
        time.sleep(9.5)
        g = int(str(post("grip_read", {"dry_run": True})["result"] or 0))
        if abs(g - 30) > 2:
            raise RuntimeError(f"{tag} 하강 전 그리퍼 30 확보 실패(실측 {g}) — 중단")
    return g


def run(script, *args):
    r = subprocess.run([sys.executable, f"{HERE}/{script}"] + list(args),
                       capture_output=True, text=True, timeout=600, cwd=HERE)
    out = r.stdout.strip()
    ok = ("완료:" in out) or ("✔ 수렴" in out)
    return ok, out


def cycle(i, cal):
    tt = cal["refs"]["blue"]["pick_tcp_taught"]
    it = cal["refs"]["blue"]["insert_tcp"]
    log = {}

    # 1) 랙에서 집기
    go_z(max(st()["tcp"][2], SAFE_Z))
    post("move", {"joints": cal["observe_joints"], "dry_run": False}); wait(90)
    go_xy(tt, 520.0)
    ensure_open_before_descend("픽")          # 내려가기 전 반드시 30
    ok, out = run("descend_ref.py", "play", "blue", "--nocam2")
    if not ok:
        raise RuntimeError("픽 하강 실패:\n" + out[-300:])
    log["pick_tcp"] = [round(v, 2) for v in stable_tcp()[:3]]
    grip(13, 13, "파지")

    # 2) 운반
    go_z(SLOW_Z, spd=1)
    go_z(SAFE_Z)
    go_xy(it, SAFE_Z)
    go_xy(it, HOVER_Z)

    # 3) yaw 보정
    ok, out = run("yaw_measure.py", "fix", "--use", "A1_rel", "--tol", "0.05")
    dev = [l for l in out.splitlines() if "편차" in l]
    log["yaw"] = dev[0].strip() if dev else "?"
    log["yaw_ok"] = ok

    # 4) 삽입 하강
    ok, out = run("descend_ref.py", "play", "blue", "--place")
    if not ok:
        raise RuntimeError("place 하강 실패:\n" + out[-400:])
    log["place_lines"] = [l.strip() for l in out.splitlines() if l.strip().startswith("z")]
    p = stable_tcp()
    log["insert_tcp"] = [round(v, 2) for v in p[:3]]
    log["insert_delta"] = [round(p[k] - it[k], 2) for k in range(3)]

    # 5) 놓기 — 여기서만 연다(하강 성공 확인 후)
    grip(30, 30, "놓기")
    go_z(HOVER_Z, spd=1)
    log["released"] = True

    # 6) 다시 빼기 (삽입 자세로 직접 하강 → 파지)
    go_xy(it, HOVER_Z)
    ensure_open_before_descend("재파지")       # 내려가기 전 반드시 30
    speed(1)
    post("move_tcp", {"tcp": list(it), "dry_run": False}); wait()
    p2 = stable_tcp()
    if max(abs(p2[k] - it[k]) for k in range(3)) > 2.0:
        raise RuntimeError(f"삽입 자세 미도달 {[round(v,2) for v in p2[:3]]} — 그리퍼 조작 안 함")
    g = grip(13, None, "재파지")
    log["repick_grip"] = g
    if g < 10:
        raise RuntimeError(f"재파지 실패(실측 {g}) — 벽이 슬롯에 없거나 위치 이탈")
    go_z(SLOW_Z, spd=1)
    go_z(SAFE_Z)

    # 7) 랙 반납 — 티칭 좌표로 직접(픽 기준 사진은 빈 그리퍼용이라 못 씀)
    go_xy(tt, SAFE_Z)
    go_xy(tt, 430.0)
    speed(1)
    post("move_tcp", {"tcp": list(tt), "dry_run": False}); wait()
    p3 = stable_tcp()
    if max(abs(p3[k] - tt[k]) for k in range(3)) > 2.0:
        raise RuntimeError(f"랙 반납 자세 미도달 {[round(v,2) for v in p3[:3]]} — 그리퍼 안 엶(벽 낙하 방지)")
    grip(30, 30, "반납")
    go_z(470.0, spd=1)
    return log


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    cal = json.load(open(CAL))
    okn = 0
    for i in range(1, n + 1):
        print(f"\n━━━━━ 사이클 {i}/{n} ━━━━━", flush=True)
        t0 = time.time()
        try:
            log = cycle(i, cal)
            okn += 1
            print(f"  파지 {log['pick_tcp']}")
            print(f"  yaw  {log['yaw']}")
            for l in log["place_lines"]:
                print("  " + l)
            print(f"  삽입 {log['insert_tcp']}  Δ{log['insert_delta']}mm")
            print(f"  재파지 실측 {log['repick_grip']}  ·  {time.time()-t0:.0f}초")
            print(f"  ✅ 사이클 {i} 성공")
        except Exception as e:
            print(f"  ❌ 사이클 {i} 실패: {e}")
            print("  로봇은 그 자리에 정지 — 수동 확인 필요")
            break
    print(f"\n━━━ 결과: {okn}/{n} 성공 ━━━")


if __name__ == "__main__":
    main()
