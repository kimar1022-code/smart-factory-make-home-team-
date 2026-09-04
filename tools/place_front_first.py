#!/usr/bin/env python3
"""앞끝(뎁스캠·노랑 기둥) 먼저 맞추는 place 러너 — 9/2 오후 사용자 설계.

  python3 place_front_first.py run blue [--no-approach] [--trim 0]

사용자 설계:
  · 파지하면 벽이 뎁스캠(cam1) 쪽으로 기울어 그 끝이 먼저 기둥에 닿는다 → 그쪽부터 맞춘다.
  · 보정은 x → y → yaw 순서로 한 축씩(1mm/1° 이하 스텝), z478 에서 한 번, z443 에서 다시.
  · 뒤끝(새카메라 cam2·빨강 기둥)은 마커 각도로 yaw 를 맞춘다(앞끝을 기준으로 회전).
  · 끼우기 전 아래 높이(z413·z388)에서 뎁스캠으로 한 번 더 확인하고 내려간다.
앵커 = ArUco 마커(색점 보정용, 9/2) + 벽점(파지 편차 보상). 골든 = golden_capture 캡처본.
철칙: 검증 없이 이동 없음 · 실패는 그 자리 정지(예외 경로 stop 선행) · 그리퍼 무조작.
"""
import json
import math
import sys
import time

sys.path.insert(0, "/home/ar/bf2_console/tools")
import golden_place as GP  # noqa: E402
from golden import (CAL, DANG_DJ6, move_j6, move_xy_eff, move_z, post, st, stable, wrap,  # noqa: E402
                    _verify_reach)

OVERBACK = 1.0           # ★9/2 사용자 지시: 한 번 움직일 땐 1mm 단위(작은 값은 1mm 더 갔다 1mm 되돌기)
STEP_MAX, STEP_MIN = 1.0, 0.6          # 축별 스텝(mm) — 사용자 "1씩" 상한, 로봇 최소 실행치 하한
XY_TOL = 0.35                          # 축별 수렴(mm) — 삽입 성공 실측 0.25~0.30 (긴 벽 198: 홈 끝단 놀음 1.0mm)
XY_TOL_BY_COLOR = {"yellow": 0.25, "green": 0.25, "red_s": 0.25}   # ★STL 실측(9/2): 짧은 벽 125 는 기둥 사이 120 + 홈 깊이 3 → 길이방향 놀음 0.5mm 뿐
PILLAR_TOP_DZ = 86.0                    # 기둥 가이드 높이(STL) — seat + 86 아래는 벽이 홈 안: 찔러보기/보정 금지, 검사만
YAW_TOL, YAW_STEP_MAX, YAW_STEP_MIN, YAW_CAP = 0.35, 1.0, 0.5, 1.5
CUM_CAP = {478.0: 15.0, 443.0: 6.0, 413.0: 1.5}
MAX_IT = 30
LOG = []
RZ_CAL = 180.0     # M1/M2 축척행렬을 실측한 손목 자세(파랑 슬롯, rz ≈ ±180)


def rot_to_robot(sol):
    """9/2 저녁: 고정 매핑(회전/반사) 포기 — 원시 해를 돌려주고, stage_front 가 자코비안을 실측해 쓴다."""
    return sol


def say(s):
    print(s, flush=True); LOG.append(s)


def cam_solve(cam, z, refs):
    """한 캠만으로 (dX,dY) 해(mm, RESP·TRIM 반영) + 잔차px + 마커각차(°). 벽 편차는 파지 보상."""
    cur = GP.snap_all()
    wp = GP.match_wall(refs, cur)
    dev = GP.wall_dev(wp)
    d = dev.get(cam)
    if d is None:
        return None, f"{cam} 벽점 없음"
    if d[2] > GP.GRASP_DEV_CAP_MM:
        return None, f"벽 밀림 {d[2]:.2f}mm(파지 이상)"
    ams = GP.A_REF.get(round(z), {}).get(cam) or {}
    cms = GP.aruco_now(cam, ams.keys()) if ams else {}
    if not cms:
        return None, f"{cam} 마커 없음"
    r = GP.scene_ratio(cam, z)
    pairs, dangs = [], []
    for mid, cm in cms.items():
        am = ams[mid]
        da = wrap(cm["ang"] - am["ang"])
        if abs(da) > 2.0:
            return None, f"{cam} id{mid} 각도 이상({da:+.1f}°)"
        dangs.append(da)
        pairs.append((["a", am["cx"] + d[0] * r, am["cy"] + d[1] * r, None], ["a", cm["cx"], cm["cy"], None]))
    sol, err = GP.solve_scaled(pairs if cam == "cam1" else [], pairs if cam == "cam2" else [], z)
    if sol is None:
        return None, "해 없음"
    sol = (sol[0] / GP.RESP[0] + GP.TRIM_XY[0], sol[1] / GP.RESP[1] + GP.TRIM_XY[1])
    sol = rot_to_robot(sol)
    return {"sol": sol, "err": err, "dang": sum(dangs) / len(dangs), "ids": list(cms), "wall": d}, None


def axis_step(v):
    # ★9/3 coarse-to-fine(사용자 지시): 멀면 큰 스텝, 수렴할수록 1mm. 자코비안이 정확해 비례 착지, 발산은 worse 게이트가 잡음.
    a = abs(v)
    step = 3.0 if a > 4.0 else (2.0 if a > 2.0 else min(STEP_MAX, max(STEP_MIN, a)))
    return math.copysign(step, v)


PROBE_MM = 0.6           # 자코비안 실측용 찔러보기(로봇 최소 실행치 이상)
JAC = {}                 # {z: 2x2} 실측 자코비안 캐시(같은 z 에서 재사용)


def _solve_raw(cam, z, refs):
    s, why = cam_solve(cam, z, refs)
    if why:
        raise RuntimeError(f"z{z:.0f} {cam}: {why} — 정지")
    return s


def probe_jacobian(z, refs):
    """★9/2 저녁(노랑 rz90 매핑 실패 3회 교훈): 이 자세에서 로봇 +X/+Y 1mm 가 해를 얼마나 바꾸는지 실측.
    A = [[dsx/dX, dsx/dY],[dsy/dX, dsy/dY]]. 왕복이라 위치는 원복된다."""
    import numpy as np
    s0 = _solve_raw("cam1", z, refs)["sol"]
    move_xy_eff(PROBE_MM, 0.0); time.sleep(0.4)
    sx = _solve_raw("cam1", z, refs)["sol"]
    move_xy_eff(-PROBE_MM, 0.0); time.sleep(0.4)
    move_xy_eff(0.0, PROBE_MM); time.sleep(0.4)
    sy = _solve_raw("cam1", z, refs)["sol"]
    move_xy_eff(0.0, -PROBE_MM); time.sleep(0.4)
    A = np.array([[(sx[0]-s0[0])/PROBE_MM, (sy[0]-s0[0])/PROBE_MM],
                  [(sx[1]-s0[1])/PROBE_MM, (sy[1]-s0[1])/PROBE_MM]])
    det = float(np.linalg.det(A))
    say(f"  z{z:.0f} 자코비안 실측: +X→해Δ({A[0,0]:+.2f},{A[1,0]:+.2f}) +Y→해Δ({A[0,1]:+.2f},{A[1,1]:+.2f}) det {det:+.2f}")
    if abs(det) < 0.05:
        raise RuntimeError(f"z{z:.0f}: 자코비안 퇴화(det {det:.3f}) — 정지")
    return A


def stage_front(z, refs, cap, moved):
    """앞끝(cam1) XY — 실측 자코비안으로 필요한 로봇 이동을 구해 한 축씩(0.6~1.0mm) 움직인다."""
    import numpy as np
    if z not in JAC:
        JAC[z] = probe_jacobian(z, refs)
    Ainv = np.linalg.inv(JAC[z])
    prev_err, worse = None, 0
    for it in range(MAX_IT):
        s = _solve_raw("cam1", z, refs)
        n = np.array(s["sol"])
        m = -Ainv @ n                                   # 해(need)를 0 으로 만드는 로봇 이동(mm)
        dX, dY = float(m[0]), float(m[1])
        if prev_err is not None and s["err"] > prev_err + 0.8:
            worse += 1
            if worse >= 2:
                raise RuntimeError(f"z{z:.0f} 앞끝: 보정 후 잔차 연속 증가({prev_err:.1f}→{s['err']:.1f}px) — 발산, 정지")
        else:
            worse = 0
        prev_err = s["err"]
        say(f"  z{z:.0f} 앞끝[{it+1}] cam1 {s['ids']} 잔차 {s['err']:.2f}px  해 ({s['sol'][0]:+.2f},{s['sol'][1]:+.2f})  로봇이동 필요 ({dX:+.2f},{dY:+.2f})mm  누적 {moved:.1f}mm")
        if abs(dX) < XY_TOL and abs(dY) < XY_TOL:
            return moved
        ax_i = 0 if abs(dX) >= abs(dY) else 1
        need = dX if ax_i == 0 else dY
        if abs(need) < STEP_MIN:
            # ★9/2 저녁: 로봇은 0.6mm 미만 명령을 버린다 → '넘어갔다 되돌기'로 순이동을 만든다(둘 다 실행되는 크기).
            over = math.copysign(abs(need) + OVERBACK, need)
            m1 = (over, 0.0) if ax_i == 0 else (0.0, over)
            m2 = (-math.copysign(OVERBACK, need), 0.0) if ax_i == 0 else (0.0, -math.copysign(OVERBACK, need))
            if moved + abs(need) > cap:
                raise RuntimeError(f"z{z:.0f} 앞끝: 누적 상한 {cap}mm — 정지")
            a1 = move_xy_eff(*m1); time.sleep(0.3); a2 = move_xy_eff(*m2)
            net = (a1[0]+a2[0], a1[1]+a2[1])
            say(f"    미세이동 {need:+.2f}mm = {over:+.2f} 후 {-math.copysign(OVERBACK, need):+.2f} (순 {net[ax_i]:+.2f})")
            moved += math.hypot(*net)
        else:
            mv = (axis_step(need), 0.0) if ax_i == 0 else (0.0, axis_step(need))
            if moved + math.hypot(*mv) > cap:
                raise RuntimeError(f"z{z:.0f} 앞끝: 누적 상한 {cap}mm — 정지")
            ax, ay = move_xy_eff(*mv)
            moved += math.hypot(ax, ay)
        time.sleep(0.45)
    raise RuntimeError(f"z{z:.0f} 앞끝: {MAX_IT}회 미수렴 — 정지")


def stage_rear_yaw(z, refs, yaw_used):
    """뒤끝(cam2) — 마커 각도로 yaw. z478 에서만 J6 0.5~1° 씩, 두 판독 일치할 때만.
    반환 (yaw_used, s, rotated)."""
    rotated = False
    for it in range(6):
        s, why = cam_solve("cam2", z, refs)
        if why:
            raise RuntimeError(f"z{z:.0f} 뒤끝: {why} — 정지")
        say(f"  z{z:.0f} 뒤끝[{it+1}] cam2 {s['ids']} 잔차 {s['err']:.2f}px  해 ({s['sol'][0]:+.2f},{s['sol'][1]:+.2f})mm  각차 {s['dang']:+.2f}°")
        if abs(s["dang"]) < YAW_TOL or z != YAW_Z:
            if abs(s["dang"]) >= YAW_TOL:
                say(f"    (z{z:.0f} 에서는 yaw 측정만 — 회전은 z{YAW_Z:.0f} 전용)")
            return yaw_used, s, rotated
        s2, why2 = cam_solve("cam2", z, refs)                    # ★재판독 일치 확인
        if why2 or abs(s2["dang"] - s["dang"]) > YAW_AGREE or (s2["dang"] * s["dang"]) <= 0:
            say(f"    yaw 재판독 불일치({s['dang']:+.2f}/{(s2 or {}).get('dang', float('nan')):+.2f}°) — 회전 안 함")
            return yaw_used, s, rotated
        dang = (s["dang"] + s2["dang"]) / 2
        dj = -dang / DANG_DJ6
        dj = math.copysign(max(YAW_STEP_MIN, min(YAW_STEP_MAX, abs(dj))), dj)
        if abs(yaw_used + dj) > YAW_CAP:
            raise RuntimeError(f"z{z:.0f} 뒤끝: yaw 누적 상한 {YAW_CAP}° — 정지(재파지 대상)")
        say(f"    J6 {dj:+.2f}° (판독 {s['dang']:+.2f}/{s2['dang']:+.2f}°)")
        move_j6(dj); yaw_used += dj; rotated = True
        time.sleep(0.5)
    return yaw_used, None, rotated


REAR_RESID_MAX = 0.6     # 앞끝 맞춘 뒤 뒤끝 XY 잔류 상한(mm) — 1.0 은 노랑 실패 때 0.82 를 통과시킴


YAW_Z = 478.0            # yaw 회전은 기둥에서 먼 z478 전용
YAW_PROBE_DEG = 0.3       # yaw 자코비안 실측용 J6 찔러보기
YAW_MIN_STEP, YAW_MAX_STEP = 0.15, 1.0


def rear_resid(z, refs):
    s = _solve_raw("cam2", z, refs)
    return s, math.hypot(*s["sol"])


def stage_rear_yaw_probe(z, refs, yaw_used):
    """★9/2 저녁(노랑 뒤끝 0.78mm 교훈): 마커 각도 대신 '뒤끝 잔차 벡터가 J6 에 어떻게 반응하는지'를 실측해
    회전량을 구한다(XY 자코비안과 같은 철학). z478 전용. 반환 (yaw_used, rotated)."""
    s0, r0 = rear_resid(z, refs)
    say(f"  z{z:.0f} 뒤끝 cam2 {s0['ids']} 잔차 {s0['err']:.2f}px  해 ({s0['sol'][0]:+.2f},{s0['sol'][1]:+.2f})mm  각차 {s0['dang']:+.2f}°")
    if r0 <= REAR_RESID_MAX:
        return yaw_used, False
    if abs(s0["dang"]) < 0.2:
        # ★9/2 실측: 각차 0.00° 인데 뒤끝만 0.8mm → 회전 아님 = 두 캠의 위치 의견 차. 중간으로 반씩 양보(회전 금지).
        import numpy as np
        A = JAC.get(z)
        if A is None:
            raise RuntimeError(f"z{z:.0f}: 자코비안 없음 — 정지")
        m = -np.linalg.inv(A) @ np.array(s0["sol"])          # 뒤끝 의견대로 가려면 필요한 로봇 이동(mm)
        half = (float(m[0]) * 0.5, float(m[1]) * 0.5)
        say(f"    각차 {s0['dang']:+.2f}° → 회전 아님. 두 캠 의견차 {math.hypot(*m):.2f}mm → 절반 {half[0]:+.2f},{half[1]:+.2f}mm 이동(양끝 균등)")
        if math.hypot(*half) > 1.0:
            raise RuntimeError(f"z{z:.0f}: 두 캠 의견차 {math.hypot(*m):.2f}mm 과대 — 정지")
        if math.hypot(*half) >= 0.3:
            move_xy_eff(*half); time.sleep(0.45)
        return yaw_used, None                                   # None = 앞끝 재정렬 없이 진행
    if z != YAW_Z:
        say(f"    (z{z:.0f} 에서는 회전 금지 — 정지 사유)")
        raise RuntimeError(f"z{z:.0f}: 뒤끝 {r0:.2f}mm 어긋남(회전은 z{YAW_Z:.0f} 전용) — 정지")
    move_j6(YAW_PROBE_DEG); time.sleep(0.5)
    s1, _ = rear_resid(z, refs)
    move_j6(-YAW_PROBE_DEG); time.sleep(0.5)
    g = ((s1["sol"][0]-s0["sol"][0]) / YAW_PROBE_DEG, (s1["sol"][1]-s0["sol"][1]) / YAW_PROBE_DEG)   # 해Δ / J6 1°
    gg = g[0]**2 + g[1]**2
    say(f"    J6 +{YAW_PROBE_DEG}° 찔러보기: 뒤끝 해Δ/° ({g[0]:+.2f},{g[1]:+.2f})")
    if gg < 0.02:
        raise RuntimeError(f"z{z:.0f}: J6 에 뒤끝이 반응 없음(|g|² {gg:.3f}) — 정지")
    dj = -(s0["sol"][0]*g[0] + s0["sol"][1]*g[1]) / gg          # 잔차를 g 방향으로 소거하는 J6 량
    dj = math.copysign(max(YAW_MIN_STEP, min(YAW_MAX_STEP, abs(dj))), dj)
    if abs(yaw_used + dj) > YAW_CAP:
        raise RuntimeError(f"z{z:.0f}: yaw 누적 상한 {YAW_CAP}° — 정지(재파지 대상)")
    say(f"    J6 {dj:+.2f}°")
    move_j6(dj); time.sleep(0.5)
    return yaw_used + dj, True


def stage(z, refs, yaw_used):
    moved = 0.0
    cap = CUM_CAP.get(z, 1.5)
    for rnd in range(4):
        moved = stage_front(z, refs, cap, moved)
        yaw_used, rotated = stage_rear_yaw_probe(z, refs, yaw_used)
        if rotated is None:                                      # 중간 타협 이동 완료 → 양끝 잔차 보고 후 진행
            s1 = _solve_raw("cam1", z, refs); s2, r2 = rear_resid(z, refs)
            say(f"  z{z:.0f}: ✔ 타협 정렬 · 앞끝 해 ({s1['sol'][0]:+.2f},{s1['sol'][1]:+.2f}) · 뒤끝 {r2:.2f}mm · 각차 {s2['dang']:+.2f}°")
            return yaw_used
        if rotated:
            JAC.pop(z, None)                                     # 회전 뒤 XY 자코비안 재실측
            say(f"  z{z:.0f}: J6 회전 후 앞끝 재정렬(회차 {rnd+1})")
            continue
        s2, r2 = rear_resid(z, refs)
        say(f"  z{z:.0f}: ✔ 앞끝 {XY_TOL}mm 이내 · 뒤끝 {r2:.2f}mm · 각차 {s2['dang']:+.2f}°")
        return yaw_used
    raise RuntimeError(f"z{z:.0f}: 앞끝↔뒤끝 4회 왕복 미수렴 — 정지")


CK_NAME = {"ck": "blue"}


def jacobian_cached(z, refs):
    """★9/2 저녁 사용자 지시(보정 최소화): 자코비안은 색·높이별로 한 번만 실측해 dot_calib 에 저장, 이후 재사용.
    → 매 사이클 '측정 1회 → 이동 1회'. (실측 재현성: z443 두 번 (+0.07,−0.54)/(+0.71,+0.04) vs (−0.10,−0.40)/(+0.38,−0.03))"""
    import numpy as np
    cal = json.load(open(CAL)); ref = cal["refs"][CK_NAME["ck"]]
    key = str(int(z))
    jc = ref.get("jacobian", {})
    if key in jc:
        A = np.array(jc[key]["A"])
        say(f"  z{z:.0f} 자코비안(저장값 {jc[key]['made']}) 사용: +X→({A[0,0]:+.2f},{A[1,0]:+.2f}) +Y→({A[0,1]:+.2f},{A[1,1]:+.2f})")
        return A
    A = probe_jacobian(z, refs)
    jc[key] = {"A": A.tolist(), "made": time.strftime("%Y-%m-%d %H:%M")}
    ref["jacobian"] = jc; cal["refs"][CK_NAME["ck"]] = ref
    json.dump(cal, open(CAL, "w"), indent=1, ensure_ascii=False)
    say(f"  z{z:.0f} 자코비안 저장(다음부터 찔러보기 생략)")
    return A


def stage_once(z, refs):
    """z443 단일 보정: 자코비안(저장/실측) → 두 축 동시 뉴턴 스텝(최대 2회, 각 축 미세이동 지원) → 뒤끝 타협 1회."""
    import numpy as np
    A = jacobian_cached(z, refs); Ainv = np.linalg.inv(A)
    for it in range(2):
        s = _solve_raw("cam1", z, refs)
        m = -Ainv @ np.array(s["sol"]); dX, dY = float(m[0]), float(m[1])
        say(f"  z{z:.0f} 앞끝[{it+1}] cam1 {s['ids']} 잔차 {s['err']:.2f}px  로봇이동 필요 ({dX:+.2f},{dY:+.2f})mm")
        if abs(dX) < XY_TOL and abs(dY) < XY_TOL:
            break
        for ax_i, need in ((0, dX), (1, dY)):
            if abs(need) < XY_TOL:
                continue
            if abs(need) < STEP_MIN:
                over = math.copysign(abs(need) + OVERBACK, need); back = -math.copysign(OVERBACK, need)
                m1 = (over, 0.0) if ax_i == 0 else (0.0, over); m2 = (back, 0.0) if ax_i == 0 else (0.0, back)
                move_xy_eff(*m1); time.sleep(0.3); move_xy_eff(*m2)
                say(f"    {'XY'[ax_i]} 미세이동 {need:+.2f} (={over:+.2f} 후 {back:+.2f})")
            else:
                step = math.copysign(1.0, need)                                   # 1mm 단위 한 스텝
                mv = (step, 0.0) if ax_i == 0 else (0.0, step)
                move_xy_eff(*mv); say(f"    {'XY'[ax_i]} 이동 {step:+.1f} (필요 {need:+.2f})")
            time.sleep(0.4)
    s1 = _solve_raw("cam1", z, refs); s2, r2 = rear_resid(z, refs)
    m2v = -Ainv @ np.array(s2["sol"]); rr = math.hypot(float(m2v[0]), float(m2v[1]))
    say(f"  z{z:.0f} 확인: 앞끝 해 ({s1['sol'][0]:+.2f},{s1['sol'][1]:+.2f}) 잔차 {s1['err']:.2f}px · 뒤끝 필요이동 {rr:.2f}mm 각차 {s2['dang']:+.2f}°")
    if rr > REAR_RESID_MAX and abs(s2["dang"]) < 0.2:
        half = (float(m2v[0]) * 0.5, float(m2v[1]) * 0.5)
        if math.hypot(*half) > 1.0:
            raise RuntimeError(f"z{z:.0f}: 두 캠 의견차 {rr:.2f}mm 과대 — 정지")
        say(f"    두 캠 의견차 {rr:.2f}mm → 절반 ({half[0]:+.2f},{half[1]:+.2f}) 1회 이동(양끝 균등)")
        for ax_i, need in ((0, half[0]), (1, half[1])):
            if abs(need) < 0.15:
                continue
            if abs(need) < STEP_MIN:
                over = math.copysign(abs(need) + OVERBACK, need); back = -math.copysign(OVERBACK, need)
                move_xy_eff(*((over, 0.0) if ax_i == 0 else (0.0, over))); time.sleep(0.3)
                move_xy_eff(*((back, 0.0) if ax_i == 0 else (0.0, back)))
            else:
                move_xy_eff(*((need, 0.0) if ax_i == 0 else (0.0, need)))
            time.sleep(0.4)
    elif rr > REAR_RESID_MAX:
        raise RuntimeError(f"z{z:.0f}: 뒤끝 {rr:.2f}mm · 각차 {s2['dang']:+.2f}° — 벽 회전 의심(재파지) 정지")
    return 0.0


def check_front(z, refs, correct_cap, final_z=None):
    """끼우기 전 아래 높이 확인(뎁스캠). correct_cap>0 이면 그만큼만 보정 허용, 0 이면 검사만.
    ★9/2 STL 실측: 홈 폭 5.0 vs 탭 4.2(놀음 0.8) · 기둥 상단에 리드인 챔퍼 없음 → 기둥 꼭대기(seat+86) 아래에서
    옆으로 찔러보거나 보정하면 홈 벽을 밀어 벽이 그리퍼에서 돈다. 그 아래는 검사만."""
    move_z(z); time.sleep(0.6)
    below_top = final_z is not None and z < final_z + PILLAR_TOP_DZ + 1.0
    if correct_cap > 0 and not below_top:
        stage_front(z, refs, correct_cap, 0.0)
        return
    if below_top:
        say(f"  z{z:.0f}: 기둥 꼭대기(z{final_z + PILLAR_TOP_DZ:.0f}) 아래 — 보정 없이 검사만")
    s, why = cam_solve("cam1", z, refs)
    if why:
        raise RuntimeError(f"z{z:.0f} 확인: {why} — 정지")
    mag = math.hypot(*s["sol"]) / 0.55          # 원시 해는 실이동의 ~0.55배(실측) → 실 mm 근사
    say(f"  z{z:.0f} 확인 cam1 {s['ids']} 잔차 {s['err']:.2f}px 해 ({s['sol'][0]:+.2f},{s['sol'][1]:+.2f})mm 각차 {s['dang']:+.2f}°")
    if mag > 1.0:
        raise RuntimeError(f"z{z:.0f} 확인: {mag:.2f}mm 어긋남 — 진입 금지, 정지")


def cam1_bright(tag):
    """손목캠 밝기·노출 판독만(변경 없음)."""
    import urllib.request as _u
    try:
        e = json.loads(_u.urlopen("http://127.0.0.1:8766/expo", timeout=5).read())
        say(f"  {tag} 손목캠 밝기 {e.get('bright', 0):.1f} 노출 {e.get('exposure', 0):.0f}")
        return e
    except Exception as ex:
        say(f"  ({tag} 손목캠 노출 조회 실패: {ex})"); return None


def cam1_bright_norm(cal, ck):
    """★9/2 저녁 대비(사용자 지시): 안착 직후(=골든을 캡처한 조립대 장면)에서 손목캠 평균밝기를
    골든 조건값으로 되돌리고, 그 노출이 다음 사이클로 이어진다.
    기준 = refs.<색>.seat_bright (start_cam 정규화 실측 109.9 @ 노출 60, 9/2 15:22 조립대 벽 든 자세
    = 골든 캡처 직전 조건). 같은 조명이면 4% 이내라 노출이 안 움직이고(무영향), 어두워졌을 때만
    노출을 올려 같은 밝기를 되찾는다. 새카메라(v4l2)는 노출 조절 불가 소스라 손대지 않는다."""
    import urllib.request as _u
    ref = cal["refs"][ck]
    sb = ref.get("seat_bright")
    if not sb:
        say("  (seat_bright 기준 없음 — 재정규화 건너뜀)"); return
    e = cam1_bright("안착")
    if not e or e.get("bright") is None:
        return
    try:
        e2 = json.loads(_u.urlopen(f"http://127.0.0.1:8766/expo?bright={sb['bright']}", timeout=25).read())
    except Exception as ex:
        say(f"  (재정규화 호출 실패 — 현재 노출 유지: {ex})"); return
    say(f"  밝기 재정규화: {e['bright']:.1f}→{e2.get('bright', 0):.1f} (목표 {sb['bright']})  "
        f"노출 {e['exposure']:.0f}→{e2.get('exposure', 0):.0f} (골든 {sb['exposure']:.0f})")
    if abs(e2.get("exposure", sb["exposure"]) - sb["exposure"]) > 0.5 * sb["exposure"]:
        say("  ⚠ 노출이 골든 대비 50% 이상 다름 — 조명이 크게 바뀐 상태(색점 면적 대역 주의)")


def main():
    if len(sys.argv) < 3 or sys.argv[1] != "run":
        sys.exit("사용법: place_front_first.py run blue [--no-approach] [--trim 0]")
    ck = sys.argv[2]
    CK_NAME["ck"] = ck
    global XY_TOL
    XY_TOL = XY_TOL_BY_COLOR.get(ck, XY_TOL)
    if "--trim" in sys.argv:
        t = float(sys.argv[sys.argv.index("--trim") + 1])
        GP.TRIM_XY = (0.0, t)
    cal = json.load(open(CAL))
    ref = cal["refs"][ck]
    D = ref["dist_refs"]
    grasp = ref["grasp_ref"]
    it_tcp = ref["insert_tcp"]
    rows = sorted(D["rows"], key=lambda r: -r["z"])
    refs_by_z = {r["z"]: GP.wall_refs(r, grasp) for r in rows}
    # 마커 기준: golden_raw 의 4단 전부(aruco_place 는 478/443 만) — id별 dict
    # ★이웃 파랑 벽이 꽂히면 cam2 id34 판독이 밀림(노랑·빨강 뒤끝 오차) → 그 벽 cam2 는 id34 제외(9/3 아침 성공이 쓰던 제외)
    EXCLUDE = {"yellow": {"cam2": ["34"]}, "red": {"cam2": ["34"]}}
    GP.A_REF.clear()
    for r_ in ref["golden_raw"]["rows"]:
        GP.A_REF[round(r_["z"])] = {c: dict(r_[c]["aruco"]) for c in ("cam1", "cam2") if r_[c]["aruco"]}
        for cam_, ids_ in EXCLUDE.get(ck, {}).items():
            for id_ in ids_:
                GP.A_REF[round(r_["z"])].get(cam_, {}).pop(id_, None)
    say(f"트림 {GP.TRIM_XY}  앞끝 허용 {XY_TOL}mm  마커기준 높이 {sorted(GP.A_REF)}  insert {it_tcp[:3]}  기둥꼭대기 z{D['final_z']+PILLAR_TOP_DZ:.0f}")

    gr = str(post("grip_read", {"dry_run": True})["result"])
    g = int(gr) if gr.isdigit() else 0                    # 브리지가 0 을 'ok' 로 돌려줌
    if abs(g - grasp["grip"]) > 3:
        sys.exit(f"그리퍼 실측 {g} (골든 파지 {grasp['grip']}) — 벽을 물고 있지 않음. 중단")

    try:
        if "--no-approach" not in sys.argv:
            cur = stable()
            GP.speed(1)
            if cur[2] < GP.SLOW_Z:
                GP.move_z(GP.SLOW_Z)
            GP.speed(2)
            GP.move_z(GP.SAFE_Z)
            tgt = [it_tcp[0], it_tcp[1], GP.SAFE_Z] + list(it_tcp[3:])
            post("move_tcp", {"tcp": tgt, "dry_run": False}); time.sleep(0.9)
            t0 = time.time()
            while time.time() - t0 < 90 and st()["busy"]:
                time.sleep(0.35)
            if not _verify_reach(tgt, 2.0, timeout=15):
                raise RuntimeError("운반 미도달 — 정지")
        GP.speed(1)
        move_z(rows[0]["z"]); time.sleep(0.6)
        e = cam1_bright("z478")                                    # ★어두우면(저녁) 그 자리에서 재정규화 — 노랑 점 임계 이탈 방지(9/2 실증: 밝기 52 → H18/V96 깜빡임)
        if e and e.get("bright") is not None and e["bright"] < 80:
            import urllib.request as _u
            try:
                e2 = json.loads(_u.urlopen("http://127.0.0.1:8766/expo?bright=100", timeout=25).read())
                say(f"  z478 밝기 {e['bright']:.0f}<80 → 재정규화 {e2.get('bright', 0):.0f} (노출 {e2.get('exposure', 0):.0f})")
            except Exception as ex:
                say(f"  (재정규화 실패: {ex})")
        GP.grasp_gate(refs_by_z[rows[0]["z"]], "파지검증")
        yaw_used = 0.0
        # ★9/2 저녁 사용자 지시: 보정은 z443 한 곳에서만(여러 번 움직이며 쌓이는 오차 제거). z478 은 측정·보고만.
        s478 = _solve_raw("cam1", 478.0, refs_by_z[478.0])
        say(f"  z478 측정만: cam1 해 ({s478['sol'][0]:+.2f},{s478['sol'][1]:+.2f}) 잔차 {s478['err']:.2f}px")
        move_z(443.0); time.sleep(0.6)
        yaw_used = stage_once(443.0, refs_by_z[443.0])
        check_front(413.0, refs_by_z[413.0], CUM_CAP[413.0], D["final_z"])   # 기둥 위면 소보정, 아래면 검사만
        check_front(388.0, refs_by_z[388.0], 0.0, D["final_z"])              # 진입 직전: 검사만
        GP.commit_descend(D["final_z"], refs_by_z[rows[-1]["z"]])
        cam1_bright_norm(cal, ck)                                  # ★안착 장면에서 밝기 재정규화 → 다음 사이클로 이월
        p = stable()
        say(f"▶ 앞끝우선 삽입 완료: TCP {[round(v,2) for v in p[:3]]} insert 대비 Δ{[round(p[k]-it_tcp[k],2) for k in range(3)]}mm  yaw 사용 {yaw_used:+.2f}°")
    except BaseException as e:
        try:
            post("stop", {"dry_run": False})
        except Exception:
            pass
        say(f"❌ 중단(정지함): {e}")
        raise


if __name__ == "__main__":
    main()
