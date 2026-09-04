#!/usr/bin/env python3
"""앞끝우선 러너 반복 검증 사이클 (9/2 오후 사용자 지시: 트림 0 확정 + 앞끝우선 본선 검증).

  python3 cycle_front_first.py [n] [--seated]   # --seated: 지금 벽이 꽂혀 있으면 놓기부터 시작

1회 = 골든 픽(랙) → 운반 z478 → place_front_first.py --no-approach → 놓기 → 관측자세 대기
      → 사용자가 랙에 반납(카메라 감지 즉시 다음 사이클). 대기 폴링 1초·3연속 확인.
철칙: 반납은 사용자 수동 · 실패 시 그 자리 정지(벽 든 채) · 삽입 완료 선언 후에만 놓기.
"""
import json
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, "/home/ar/bf2_console/tools")
import cycle_golden as C  # noqa: E402
from pick_carry_blue import open_to_30, grip_val, verify_grasp  # noqa: E402

RACK_PX = {"blue": (400, 900, 3), "yellow": (400, 900, 2), "red": (775, 865, 2), "red_s": (700, 790, 2)}   # 왼쪽서 3번째 슬롯(x≈−215)   # 관측자세 손목캠 랙 영역 px 창·최소 점수(열은 아래서 실측)
# ★9/2 저녁: 벽이 어느 슬롯에 꽂혀도 픽되게 — 골든 픽을 찍은 열(px)과 지금 열의 차 × 슬롯 축척(mm/px) 만큼 픽 상공 X 이동.
#   실측: 파랑 열 566px ↔ x −153.5 · 노랑 골든 열 740px ↔ x −215.0 → −0.353 mm/px (슬롯 간격 ≈90px ≈ 32mm)
RACK_REF_PX = {"blue": 566.0, "yellow": 656.0, "red": 820.0, "red_s": 745.0}   # red = 긴 창문벽(x −243 골든 슬롯, 열 ±45px 만 — 짧은 빨강과 구분)
RACK_MM_PER_PX = -0.353
RACK_SHIFT_MAX = 45.0
RACK_COL = {"px": None}

HERE = "/home/ar/bf2_console/tools"
LOGDIR = "/home/ar/bf2_console/logs"
SAFE_Z, HOVER_Z, SLOW_Z = C.SAFE_Z, C.HOVER_Z, C.SLOW_Z


CK = "blue"


def to_observe(cal):
    """관측자세로만 이동(다음 벽 대기 없이). 마지막 벽 뒤 연속 실행 종료용."""
    C.go_z(max(C.st()["tcp"][2], SAFE_Z))
    C.post("move", {"joints": cal["observe_joints"], "dry_run": False}); C.wait(90)


def wait_rack(cal, limit=1200):
    lo, hi, need = RACK_PX.get(CK, (0, 1280, 1))
    color = cal["refs"][CK].get("dot_color", CK)
    C.go_z(max(C.st()["tcp"][2], SAFE_Z))
    C.post("move", {"joints": cal["observe_joints"], "dry_run": False}); C.wait(90)
    print("  ▶ 벽을 랙에 꽂아주세요 — 감지 즉시 진행", flush=True)
    ok_n, t0 = 0, time.time()
    while time.time() - t0 < limit:
        try:
            ds = json.loads(urllib.request.urlopen("http://127.0.0.1:8766/dots?raw=1", timeout=4).read())["dots"]
            n = len([d for d in ds if d["kind"] == color and lo <= d["px"] <= hi])
        except Exception:
            n = 0
        ok_n = ok_n + 1 if n >= need else 0
        if ok_n >= 3:
            xs = sorted(d["px"] for d in ds if d["kind"] == color and lo <= d["px"] <= hi)
            RACK_COL["px"] = xs[len(xs)//2]
            print(f"  ▶ 랙 재장착 감지 — {CK} 열 px {RACK_COL['px']:.0f} (골든 열 {RACK_REF_PX.get(CK, RACK_COL['px']):.0f})", flush=True)
            return
        time.sleep(1.0)
    raise RuntimeError("반납 미감지(시간 초과) — 정지")


def release(cal):
    C.grip(30, 30, "놓기")
    C.go_z(HOVER_Z, spd=1)


def cycle(i, cal, last=False):
    ref = cal["refs"][CK]
    tt = ref["pick_tcp_taught"]
    it = ref["insert_tcp"]
    gc = int(ref.get("grip_close", 13))
    t0 = time.time()
    # 픽 — 슬롯 이동 보정(관측 열 px → 로봇 X)
    tt = list(tt)
    if RACK_COL["px"] is not None and CK in RACK_REF_PX:
        dx = (RACK_COL["px"] - RACK_REF_PX[CK]) * RACK_MM_PER_PX
        if abs(dx) > RACK_SHIFT_MAX:
            raise RuntimeError(f"슬롯 이동 {dx:+.1f}mm 가 상한 {RACK_SHIFT_MAX} 초과 — 정지")
        if abs(dx) > 3.0:
            print(f"  슬롯 보정: 픽 상공 X {dx:+.1f}mm (열 {RACK_COL['px']:.0f}px)", flush=True)
        tt[0] += dx
    C.go_xy(tt, 520.0)
    C.speed(2); C.post("move_tcp", {"tcp": [tt[0], tt[1], 520.0, 180.0, 0.0, 180.0], "dry_run": False}); C.wait(60)   # ★홈포즈(픽): 벽을 직각으로 잡기
    # ★조명 대비(아침 햇빛/밤): 픽 상공에서 손목캠 밝기가 80~130 밖이면 100 으로 재정규화(골든 점 면적 대역 유지)
    try:
        e = json.loads(urllib.request.urlopen("http://127.0.0.1:8766/expo", timeout=5).read())
        if e.get("bright") is not None and not (80.0 <= e["bright"] <= 130.0):
            e2 = json.loads(urllib.request.urlopen("http://127.0.0.1:8766/expo?bright=100", timeout=25).read())
            print(f"  픽 상공 밝기 {e['bright']:.0f} → 재정규화 {e2.get('bright', 0):.0f} (노출 {e2.get('exposure', 0):.0f})", flush=True)
    except Exception as ex:
        print(f"  (밝기 재정규화 건너뜀: {ex})", flush=True)
    open_to_30()
    ok, out = C.run("golden.py", "완료(골든 일치)", "run", "pick", CK, tag=f"ff{i}_{CK}_pick")
    if not ok:
        raise RuntimeError("골든 픽 실패:\n" + out[-400:])
    pick = [round(v, 2) for v in C.stable_tcp()[:3]]
    C.grip(gc, gc, "파지")
    verify_grasp(CK, ref)                       # ★빈손 픽 차단
    # 운반 (벽 든 채 1→2%)
    C.speed(1)
    if C.st()["tcp"][2] < SLOW_Z:
        C.go_z(SLOW_Z, spd=1)
    C.go_z(SAFE_Z, spd=2)
    tgt = [it[0], it[1], SAFE_Z, 180.0, 0.0, (180.0 if round(it[5]/90.0)*90.0 in (-180.0, 180.0) else round(it[5]/90.0)*90.0)]
    C.speed(2)
    C.post("move_tcp", {"tcp": tgt, "dry_run": False}); C.wait(90)
    got = C.stable_tcp()
    if any(abs(got[k] - tgt[k]) > 2.0 for k in range(3)):
        raise RuntimeError(f"운반 미도달 {[round(v,1) for v in got[:3]]}")
    C.go_z(HOVER_Z, spd=2)
    rz = round(it[5] / 90.0) * 90.0; rz = 180.0 if rz == -180.0 else rz
    C.speed(1); C.post("move_tcp", {"tcp": [it[0], it[1], HOVER_Z, 180.0, 0.0, rz], "dry_run": False}); C.wait(60)   # ★홈포즈(플레이스): yaw 리셋
    hp = C.stable_tcp(); print(f"  홈포즈 스냅 rx {hp[3]:+.2f} ry {hp[4]:+.2f} rz {hp[5]:+.2f}", flush=True)
    # 앞끝우선 place
    ok, out = C.run("place_front_first.py", "앞끝우선 삽입 완료", "run", CK, "--no-approach", "--trim", "0",
                    tag=f"ff{i}_{CK}_place")
    lines = [l.strip() for l in out.splitlines() if any(k in l for k in ("앞끝[", "뒤끝[", "✔", "확인", "안착 판정", "J6"))]
    if not ok:
        raise RuntimeError("앞끝우선 place 실패:\n" + "\n".join(lines[-6:]) + "\n" + out[-300:])
    p = C.stable_tcp()
    print(f"  파지 {pick}")
    for l in lines:
        print("  " + l)
    print(f"  ✅ 사이클 {i} 삽입 Δ{[round(p[k]-it[k],2) for k in range(3)]}mm · {time.time()-t0:.0f}초", flush=True)
    release(cal)
    if last:
        to_observe(cal)     # 9/3: 마지막 벽이면 다음 벽 대기 없이 관측자세로 종료(연속 실행이 다음 색으로 넘어가게)
    else:
        wait_rack(cal)


def main():
    global CK
    n = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 3
    for a in sys.argv[1:]:
        if a in ("blue", "yellow", "red", "green", "red_s"):
            CK = a
    print(f"━━ 색 {CK} · {n}회", flush=True)
    cal = json.load(open(C.CAL))
    try:
        if "--seated" in sys.argv:
            print("━━ 지금 꽂힌 벽 놓기 → 관측자세 대기", flush=True)
            release(cal)
        wait_rack(cal)          # ★어디서 시작하든 안전고도→관측자세→랙 감지 후 픽(z478 슬롯 상공에서 직행 금지)
        okn = 0
        for i in range(1, n + 1):
            print(f"\n━━━━━ 앞끝우선 사이클 {i}/{n} ━━━━━", flush=True)
            cycle(i, cal, last=(i == n))
            okn += 1
        print(f"\n━━━ 결과 {okn}/{n} 성공 ━━━")
    except Exception as e:
        try:
            C.post("stop", {"dry_run": False})
        except Exception:
            pass
        print(f"  ❌ 실패: {e}\n  로봇 그 자리 정지 — 상태 {[round(v,1) for v in C.st()['tcp'][:3]]} 그리퍼 {grip_val()}", flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
