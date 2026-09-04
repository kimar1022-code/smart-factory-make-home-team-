#!/usr/bin/env python3
"""블루 골든 픽 → 운반 → z478(HOVER) 인계까지만. 그 뒤는 사용자 수동 (9/2 오후).

  python3 pick_carry_blue.py

cycle_golden.cycle() 의 1) 골든 픽 + golden_place.py 의 approach 구간을 그대로 옮긴 것.
철칙: 하강 성공 확인 전 그리퍼 금지 · 실패 시 그 자리 정지(예외 경로는 stop 선행).
"""
import json
import math
import sys
import time

sys.path.insert(0, "/home/ar/bf2_console/tools")
import cycle_golden as C  # noqa: E402
RACK_REF_PX = {"blue": 566.0, "yellow": 656.0, "red": 841.0, "red_s": 745.0}   # 단독 실행용(cycle_front_first 와 동일)
RACK_MM_PER_PX = -0.353
RACK_SHIFT_MAX = 45.0
RACK_COL = {"px": None}

SAFE_Z, HOVER_Z, SLOW_Z, FAST = C.SAFE_Z, C.HOVER_Z, C.SLOW_Z, C.FAST


def grip_val():
    """브리지가 위치 0 을 'ok' 로 돌려준다(ret or "ok") — 숫자 아니면 0 취급."""
    r = C.post("grip_read", {"dry_run": True})["result"]
    try:
        return int(str(r))
    except ValueError:
        return 0


PICK_OPEN = 30   # 픽 개도 30 (사용자 확정 — 9/3 20 으로 바꿨다 철회)


def open_to_30():
    """(이름 유지) 픽 개도 PICK_OPEN 으로 열기."""
    g = grip_val()
    if abs(g - PICK_OPEN) > 2:
        C.post("gripper", {"pos": PICK_OPEN, "dry_run": False})
        for _ in range(20):
            time.sleep(0.6)
            g = grip_val()
            if abs(g - 30) <= 1:
                break
    if abs(g - 30) > 2:
        raise RuntimeError(f"하강 전 그리퍼 30 확보 실패(실측 {g})")
    return g


def verify_grasp(ck, ref):
    ck = ref.get("dot_color", ck)
    """★9/2 저녁(노랑 빈손 픽 교훈): 파지 직후 벽점이 골든 파지사진 위치(±60px)에 보여야 한다.
    한 캠이라도 보이면 통과, 둘 다 없으면 그리퍼 열고 정지(그리퍼 위치값은 벽 유무를 못 본다)."""
    from golden import dots
    g = ref["grasp_ref"]
    seen = {}
    for cam in ("cam1", "cam2"):
        cur = [d for d in dots(cam) if d[0] == ck]
        hit = [d for d in cur if any(math.hypot(d[1]-r[1], d[2]-r[2]) < 60 for r in g[f"{cam}_wall"])]
        seen[cam] = hit
    n = sum(len(v) for v in seen.values())
    print(f"  파지 검증: cam1 {len(seen['cam1'])} · cam2 {len(seen['cam2'])} 벽점 (골든 위치 ±60px)", flush=True)
    if n == 0:
        C.post("gripper", {"pos": 30, "dry_run": False})
        raise RuntimeError("파지 검증 실패 — 벽점이 골든 파지 위치에 없음(빈손/미끄러짐). 그리퍼 열고 정지")
    return seen


GRASP_DEV_MAX = {"blue": 0.8, "red": 0.8, "yellow": 0.6, "red_s": 0.6, "green": 0.6}   # 9/3: 파지 직후 벽 밀림 상한(mm) = 놀음 기준(긴 벽 1.0·짧은 벽 0.5). 측정 노이즈 ±0.3(파랑 cam1 0.36 vs cam2 0.64 실측)


def grasp_dev(ck, ref):
    """파지 직후 벽점을 골든 파지사진(grasp_ref)과 비교 — 캠별 (dx,dy,mm). 벽·두 캠이 전부 그리퍼에 있어 위치 무관."""
    import golden_place as GP
    from golden import snap_all
    refs = {cam: ref["grasp_ref"][f"{cam}_wall"] for cam in ("cam1", "cam2")}
    pairs = GP.match_wall(refs, snap_all())
    dev = GP.wall_dev(pairs)
    worst = max((d[2] for d in dev.values() if d), default=None)
    ds = {c: (f"({dev[c][0]:+.1f},{dev[c][1]:+.1f})px" if dev[c] else "―") for c in ("cam1", "cam2")}
    print(f"  파지 밀림 {'―' if worst is None else f'{worst:.2f}mm'} (cam1 {ds['cam1']} · cam2 {ds['cam2']})", flush=True)
    return worst


def main():
    ck = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "blue"
    cal = json.load(open(C.CAL))
    ref = cal["refs"][ck]
    gc = int(ref.get("grip_close", 13))
    tt = ref["pick_tcp_taught"]
    it = ref["insert_tcp"]
    try:
        # 0) 현재 위치(조립대 위 z340 부근·빈 그리퍼)에서 안전 고도로 — 기둥 근처는 저속
        cur = C.st()["tcp"]
        print(f"시작 tcp {[round(v,1) for v in cur[:3]]}", flush=True)
        if cur[2] < SLOW_Z:
            C.go_z(SLOW_Z, spd=1)
        C.go_z(max(C.st()["tcp"][2], SAFE_Z))

        # 1) 골든 픽 (cycle_golden 과 동일) + 슬롯 열 감지(어느 슬롯이든 픽)
        C.post("move", {"joints": cal["observe_joints"], "dry_run": False}); C.wait(90)
        import urllib.request as _u
        RACK_REF_PX = {"blue": 566.0, "yellow": 656.0, "red": 820.0, "red_s": 745.0}   # red = 긴 창문벽(x −243 골든 슬롯, 열 ±45px 만 — 짧은 빨강과 구분); RACK_MM_PER_PX = -0.353
        color = ref.get("dot_color", ck)
        tt = list(tt)
        if ck in RACK_REF_PX:
            xs = []
            for _ in range(5):
                try:
                    ds = json.loads(_u.urlopen("http://127.0.0.1:8766/dots?raw=1", timeout=4).read())["dots"]
                    xs += [d["px"] for d in ds if d["kind"] == color and 400 <= d["px"] <= 900]
                except Exception:
                    pass
                time.sleep(0.3)
            if not xs:
                raise RuntimeError(f"관측자세에서 {ck} 랙 점이 안 보임 — 정지")
            col = sorted(xs)[len(xs)//2]
            dx = (col - RACK_REF_PX[ck]) * RACK_MM_PER_PX
            if abs(dx) > 45:
                raise RuntimeError(f"슬롯 이동 {dx:+.1f}mm 상한 초과 — 정지")
            print(f"  랙 열 px {col:.0f} (골든 {RACK_REF_PX[ck]:.0f}) → 픽 상공 X {dx:+.1f}mm", flush=True)
            tt[0] += dx
        C.go_xy(tt, 520.0)
        print(f"  그리퍼 열기 → {open_to_30()}", flush=True)
        C.ensure_open_before_descend("픽")
        ok, out = C.run("golden.py", "완료(골든 일치)", "run", "pick", ck, tag=f"pc_pick_{ck}")
        if not ok:
            raise RuntimeError("골든 픽 실패:\n" + out[-400:])
        p = C.stable_tcp()
        print(f"  파지 위치 {[round(v,2) for v in p[:3]]}", flush=True)
        C.grip(gc, gc, "파지")
        verify_grasp(ck, ref)

        # 2) 운반 — golden_place approach 그대로 (벽 든 채: 1%→2%)
        C.speed(1)
        if C.st()["tcp"][2] < SLOW_Z:
            C.go_z(SLOW_Z, spd=1)
        C.go_z(SAFE_Z, spd=2)
        tgt = [it[0], it[1], SAFE_Z] + list(it[3:])
        C.speed(2)
        C.post("move_tcp", {"tcp": tgt, "dry_run": False}); C.wait(90)
        got = C.stable_tcp()
        if any(abs(got[k] - tgt[k]) > 2.0 for k in range(3)):
            raise RuntimeError(f"운반 미도달: 목표 {[round(v,1) for v in tgt[:3]]} "
                               f"현재 {[round(v,1) for v in got[:3]]}")
        C.go_z(HOVER_Z, spd=2)
        # ★9/2 저녁 사용자 설계 '홈포즈': 손목 자세를 rx180/ry0/rz(90° 단위)로 스냅 → 벽이 바닥과 평행·슬롯과 직각.
        #   이후 골든 티칭은 X·Y·Z 조그만으로. (기존 골든 자세 rx179.36/ry−0.81 = 밑동 1.3~1.7mm 이탈의 원인)
        cur = C.stable_tcp()
        rz = round(cur[5] / 90.0) * 90.0
        rz = 180.0 if rz == -180.0 else rz
        C.speed(1)
        C.post("move_tcp", {"tcp": [cur[0], cur[1], HOVER_Z, 180.0, 0.0, rz], "dry_run": False}); C.wait(60)
        p = C.stable_tcp()
        print(f"  홈포즈 스냅: rx {p[3]:+.2f} ry {p[4]:+.2f} rz {p[5]:+.2f}", flush=True)
        g = grip_val()
        print(f"▶ 인계 완료: tcp {[round(v,2) for v in p]}  그리퍼 {g}  "
              f"(insert XY 대비 Δ{[round(p[k]-it[k],2) for k in range(2)]}mm)  — 이제 수동", flush=True)
    except BaseException as e:
        try:
            C.post("stop", {"dry_run": False})
        except Exception:
            pass
        print(f"❌ 중단(정지함): {e}", flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
