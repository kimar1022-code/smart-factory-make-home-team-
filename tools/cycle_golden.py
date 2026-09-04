#!/usr/bin/env python3
"""블루 벽 왕복 풀사이클 — 골든 러너판 (9/1 밤).

  1회 = 골든 픽(랙) → 파지 → 골든 place(운반·4단·감시 진입) → 놓기 → 다시 빼기 → 랙 반납

    python3 cycle_golden.py [n]      # n회 반복(기본 5)

cycle_blue.py 와 같은 뼈대에서 러너만 교체:
  · 픽   : descend_ref.py play → golden.py run pick   (성공 마커 "완료(골든 일치)")
  · place: yaw_measure+descend_ref --place → golden_place.py run (성공 마커 "삽입 완료")
    (golden_place 가 운반·파지검증·yaw 게이트·4단 정렬·잔스텝 접촉감시 진입까지 전부 수행)

철칙 그대로: 하강 성공 확인 전 그리퍼 금지 · 반납은 티칭 좌표 직행 · 실패 시 그 자리 정지.
"""
import json
import os
import re
import subprocess
import sys
import time
import urllib.request

BRIDGE = "http://localhost:8765"
CAL = "/home/ar/bf2_console/dot_calib.json"
HERE = "/home/ar/bf2_console/tools"
SAFE_Z, HOVER_Z, SLOW_Z = 650.0, 478.0, 425.0
# ★안전고도 이동만 빠르게(9/1). 정렬·하강·진입은 종전 저속 그대로 — 정확도 우선 원칙.
FAST = 8
PICK_OPEN = 30   # 픽 개도 30 (사용자 확정)


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
        time.sleep(0.2)
    return prev


def speed(v):
    post("speed", {"value": v, "dry_run": False}); time.sleep(0.25)


def go_z(z, spd=FAST, tol=2.0):
    speed(spd)
    t = list(st()["tcp"]); t[2] = float(z)
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    got = stable_tcp()[2]
    if abs(got - z) > tol:
        raise RuntimeError(f"z 이동 미완: 목표 {z:.1f} → 도달 {got:.1f}")


def go_xy(tcp, z, spd=FAST):
    speed(spd)
    post("move_tcp", {"tcp": [tcp[0], tcp[1], z, tcp[3], tcp[4], tcp[5]], "dry_run": False})
    wait()


def grip_settle(target, timeout=12.0):
    """★고정 9.5초 대기 대신 실측이 목표에 닿거나 멎을 때까지만 기다린다(사이클당 ~25초 절감).
    그리퍼 호출은 최소화한다(동결 3회가 전부 그리퍼 명령 직후였다) — 0.6초 간격 폴링."""
    t0, prev, same = time.time(), None, 0
    g = None
    while time.time() - t0 < timeout:
        time.sleep(0.6)
        try:
            g = int(str(post("grip_read", {"dry_run": True})["result"] or 0))
        except Exception:
            continue
        if abs(g - target) <= 1:
            return g
        if prev is not None and g == prev:
            same += 1
            if same >= 3:                      # 3회(1.8초) 변화 없으면 멎은 것
                return g
        else:
            same = 0
        prev = g
    return g if g is not None else -1


def grip(pos, expect=None, tag=""):
    post("gripper", {"pos": pos, "dry_run": False})
    g = grip_settle(pos)
    if expect is not None and abs(g - expect) > 2:
        raise RuntimeError(f"{tag} 그리퍼 실측 {g} (기대 {expect}) — 중단")
    return g


def ensure_open_before_descend(tag=""):
    g = int(str(post("grip_read", {"dry_run": True})["result"] or 0))
    if abs(g - PICK_OPEN) > 2:
        post("gripper", {"pos": PICK_OPEN, "dry_run": False})
        g = grip_settle(PICK_OPEN)
        if abs(g - PICK_OPEN) > 2:
            raise RuntimeError(f"{tag} 하강 전 그리퍼 {PICK_OPEN} 확보 실패(실측 {g}) — 중단")
    return g


LOGDIR = "/home/ar/bf2_console/logs"


class PlaceFail(RuntimeError):
    """place 실패 — 벽을 든 상태. 접촉 px 와 정렬 로그를 함께 나른다."""

    def __init__(self, msg, px, align):
        super().__init__(msg)
        self.px = px
        self.align = align


def run(script, marker, *args, tag=""):
    r = subprocess.run([sys.executable, "-u", f"{HERE}/{script}"] + list(args),
                       capture_output=True, text=True, timeout=900, cwd=HERE)
    out = (r.stdout + r.stderr).strip()
    if tag:
        try:
            os.makedirs(LOGDIR, exist_ok=True)
            with open(f"{LOGDIR}/{tag}.log", "w") as f:
                f.write(out + "\n")
        except OSError:
            pass
    return (marker in out) and r.returncode == 0, out


def contact_px(out):
    """진입 접촉 로그에서 밀림 px 추출 — 없으면 None."""
    m = re.search(r"벽점 ([\d.]+)px 밀림", out)
    return float(m.group(1)) if m else None


def park_and_report():
    """★사용자 규칙(9/1): 부품을 랙에 되돌리는 일은 사용자가 직접 한다.
    실패 시 로봇은 벽을 든 채 안전 고도로만 올라가 멈추고, 무엇을 되돌려야 하는지 알린다.
    ★9/2 기둥 파손 교훈: 자식(place)이 죽어도 그 이동 명령이 컨트롤러에 살아 있을 수 있다
    — 위로 올리기 전에 반드시 정지부터."""
    try:
        post("stop", {"dry_run": False}); time.sleep(0.8)
    except Exception:
        pass
    try:
        go_z(max(st()["tcp"][2], SLOW_Z), spd=1)
        go_z(HOVER_Z, spd=1)
    except Exception as e:
        print(f"  (안전 상승 실패: {e})")
    p = stable_tcp()
    print(f"  ▶ 현재 상태: 벽을 문 채 z{p[2]:.0f} 대기 (그리퍼 13). "
          f"랙 반납은 사용자가 직접 — 그리퍼 열기가 필요하면 말씀해 주세요.", flush=True)


def cycle(i, cal):
    ref = cal["refs"]["blue"]
    tt = ref["pick_tcp_taught"]
    it = ref["insert_tcp"]
    log = {}

    # 1) 골든 픽
    go_z(max(st()["tcp"][2], SAFE_Z))
    post("move", {"joints": cal["observe_joints"], "dry_run": False}); wait(90)
    go_xy(tt, 520.0)
    ensure_open_before_descend("픽")
    ok, out = run("golden.py", "완료(골든 일치)", "run", "pick", "blue", tag=f"c{i}_pick")
    if not ok:
        raise RuntimeError("골든 픽 실패:\n" + out[-400:])
    log["pick_tcp"] = [round(v, 2) for v in stable_tcp()[:3]]
    grip(13, 13, "파지")

    # 2) 골든 place (운반~감시 진입 전부 포함)
    ok, out = run("golden_place.py", "삽입 완료", "run", "blue", tag=f"c{i}_place")
    if not ok:
        cp = contact_px(out)
        al = [l.strip() for l in out.splitlines() if "골든 일치" in l or "파지검증" in l]
        raise PlaceFail("골든 place 실패:\n" + out[-500:], cp, al)
    log["place_lines"] = [l.strip() for l in out.splitlines()
                          if "골든 일치" in l or "파지검증" in l or "삽입 완료" in l]
    p = stable_tcp()
    log["insert_tcp"] = [round(v, 2) for v in p[:3]]
    log["insert_delta"] = [round(p[k] - it[k], 2) for k in range(3)]

    # 3) 놓기 — 삽입 성공 확인 후에만
    grip(30, 30, "놓기")
    go_z(HOVER_Z, spd=1)
    log["released"] = True

    # 4) ★9/2 사용자 지시: 랙 반납은 사용자가 직접 — 로봇은 관측 자세에서 대기하고,
    #    벽이 랙에 다시 꽂히는 것을 카메라로 감지하면 다음 사이클로 진행한다.
    go_z(SAFE_Z)
    post("move", {"joints": cal["observe_joints"], "dry_run": False}); wait(90)
    print("  ▶ 벽을 랙에 꽂아주세요 — 랙에서 감지되면 자동으로 다음 사이클 진행", flush=True)
    ok_n = 0
    t0 = time.time()
    while time.time() - t0 < 1800:   # 9/2: 자리 비움 대비 30분
        try:
            ds = json.loads(urllib.request.urlopen(
                "http://127.0.0.1:8766/dots?raw=1", timeout=4).read())["dots"]
            n = len([d for d in ds if d["kind"] == "blue" and 480 <= d["px"] <= 660])
        except Exception:
            n = 0
        ok_n = ok_n + 1 if n >= 3 else 0          # 랙 열(파랑 5점 중 3점 이상) 3연속 확인
        if ok_n >= 3:
            print("  ▶ 랙 재장착 감지 — 계속", flush=True)
            log["re_racked"] = True
            return log
        time.sleep(2)
    raise RuntimeError("30분 내 랙 재장착 미감지 — 정지")


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    cal = json.load(open(CAL))
    okn, contacts = 0, []
    for i in range(1, n + 1):
        print(f"\n━━━━━ 사이클 {i}/{n} ━━━━━", flush=True)
        t0 = time.time()
        try:
            log = cycle(i, cal)
            okn += 1
            print(f"  파지 {log['pick_tcp']}")
            for l in log["place_lines"]:
                print("  " + l)
            print(f"  삽입 {log['insert_tcp']}  Δ{log['insert_delta']}mm  ·  {time.time()-t0:.0f}초")
            print(f"  ✅ 사이클 {i} 성공", flush=True)
        except PlaceFail as e:
            # ★삽입만 실패 — 벽은 그리퍼에 있고 파손 없음. 반납은 사용자 몫이므로 여기서 멈춘다.
            for l in e.align:
                print("  " + l)
            print(f"  ⛔ 사이클 {i} 삽입 실패 (접촉 {e.px}px)", flush=True)
            contacts.append(e.px)
            park_and_report()
            break
        except Exception as e:
            print(f"  ❌ 사이클 {i} 실패: {e}")
            print("  로봇은 그 자리에 정지 — 수동 확인 필요", flush=True)
            break
    print(f"\n━━━ 결과: {okn}/{n} 성공 ━━━")
    if contacts:
        print(f"    삽입 접촉 px: {contacts}  (문턱 cam1 {6.0}px)")


if __name__ == "__main__":
    main()
