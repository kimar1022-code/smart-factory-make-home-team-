#!/usr/bin/env python3
"""골든 place 러너 — 픽과 같은 '골든 사진 전체 매칭' 방식 (9/1 사용자 합의 설계).

  python3 golden_place.py run blue              # 접근(SAFE→insert XY→z478) 포함 전체 실행
  python3 golden_place.py run blue --no-approach  # 이미 insert XY 상공 z478 근처면

★dist_align.py(거리식)를 대체한다 — 거리 4개는 z443 에서 방향이 거의 평행(퇴화)이라
  잔차 1.8px '통과'인데 직각 방향 수 mm 어긋난 채 하강해 기둥 접촉 사고(9/1).
  여기서는 기둥점의 '픽셀 위치 자체'를 골든에 맞추므로 퇴화가 없다.

★기준 데이터 = dist_refs (9/1 19:46, 성공 자세에서 골든 파지로 캡처 — 재티칭 불필요):
  4단(z478/443/413/388) 각 캠의 벽점 w·기둥점 s 픽셀 + final_z 353(=insert_tcp z).
  기둥점 면적 대역은 descent_refs_place 의 같은 높이 실측 면적을 쓴다.

★사용자 기준 (9/1): 픽과 같은 4단 검사, 단 place 는 기둥에 들어가기 전이므로
  픽보다 타이트하게 — 수렴 허용을 높이가 낮을수록 조이고(마지막 단 1.5px < 픽 3px),
  수렴 후 새 스냅샷으로 재확인해야 통과.

원리 (두 캠 모두 손목에 있음):
  · 기둥점(월드 고정) 픽셀은 TCP 위치의 함수 → 골든 픽셀로 맞추면 골든 TCP 재현.
  · 벽점(그리퍼와 한 몸) 픽셀은 파지 상태만의 함수 → 골든 파지 대비 밀린 픽셀
    × (기둥 축척/벽 축척) 만큼 기둥 목표를 이동하면 파지편차가 보상된다.
    (벽이 화면 +u 밀림 ⇒ 필요한 TCP 추가이동 후 기둥 픽셀은 ref+u/k 에 놓인다)
  · yaw: 벽선은 J6 와 무관(같이 돈다) → 골든 파지사진 선과의 차 dθ 가 곧 파지 yaw 편차,
    dφ = dθ/1.034(화면각≈1.03×상대각), J6 를 -dφ 돌리면 벽의 월드 yaw 가 골든으로 복귀.

검증 없이는 아무것도 하지 않는다: 점 부족·해 불안정·누적 상한·미도달·미수렴 → 그 자리 정지.
그리퍼는 어떤 경우에도 건드리지 않는다(놓기는 사용자 명령).
"""
import json
import math
import sys
import time

sys.path.insert(0, "/home/ar/bf2_console/tools")
from golden import (BRIDGE, CAL, M1, M2, dots, match, move_j6, move_xy, move_xy_eff,
                    move_z, post, snap_all, solve_xy, st, stable, wrap, _verify_reach)

SAFE_Z, SLOW_Z = 650.0, 425.0
MOVE_SPEED = 30            # ★이동(운반·들어올리기) 속도% — 9/5 사용자 지시로 2%→30%(브리지 상한 30).
                           #   정렬·하강 감시 속도는 건드리지 않는다(1~3%, 실증값).
# ★place 는 삽입 성공(9/1)이 검증된 설정을 유지한다 — 사용자 지시로 1mm 통일에서 제외.
#   조립대 자세에서는 0.45mm 스텝이 실제로 실행되며, 이 값으로 0.25~0.29mm 정렬 후 삽입 성공.
GAIN, CAP_MM = 0.6, 0.6
MIN_STEP = 0.45
RESP = (1.49, 1.89)                        # 해/실이동 배율 (9/1 z388 실측 3회 평균)
TRIM_XY = (0.0, 0.0)                       # ★9/2 오후 사용자 확정: 새 자리 골든(수동 안착 재캡처)에선 트림 0 — +1.0 은 순수 편향 1.5mm 로 실증
MAX_IT = 40
# ★픽(3.0px 일괄)보다 타이트 — 기둥 진입 전 마지막 단이 가장 엄격.
#   px 잔차 + 해 크기(mm) 둘 다 만족해야 수렴(px 만 보면 보상 목표에 못 미친 채 통과함 — 시뮬 실증)
# px 잔차엔 두 캠 간 목표 불일치 바닥(9/1 실측 ~2.3px)이 섞인다 — 이동으로 못 없애는 성분이라
# 하한을 그 위(2.5)에 둔다. 실질 정밀도는 MAG(해 크기 mm) 게이트가 잡는다(로봇 분해능 ~0.25mm).
TOL_BY_Z = {478.0: 2.5, 443.0: 2.5, 413.0: 2.5, 388.0: 2.5}
# 해 크기 게이트 — RESP 보정 후의 '실제 mm'. 삽입 성공 시 실측 0.25/0.26/0.29/0.28mm.
MAG_TOL_BY_Z = {478.0: 0.60, 443.0: 0.50, 413.0: 0.40, 388.0: 0.35}
CUM_CAP_BY_Z = {478.0: 15.0, 443.0: 6.0, 413.0: 4.0, 388.0: 3.0}
GRASP_DEV_CAP_MM = 1.2   # 9/3: 3.5→1.2 (랙에서 0.6 게이트 통과 후 운반 중 밀림 그물)                     # 파지 밀림 상한 — 넘으면 재파지 대상
GRASP_YAW_CAP = 1.5                        # 파지 yaw 편차 상한(°) — 넘으면 재파지(회전 보정 안 함)
YAW_TOL, J6_MIN, J6_MAX = 0.35, 0.5, 1.0
IMG_PER_REL = 1.034                        # 화면각/상대각 (9/1 ArUco 실측)
WALL_SCALE = {"cam1": 9.90, "cam2": 8.65}  # px/mm — 그리퍼에 문 벽 (9/1 실측)
# ★진입 감시 2구간 (9/1 사용자 승인):
#   접근부(384~358) = 벽 밑동이 아직 기둥 사이로 안 들어간 구간. 여기서 밀리면 '걸림' → 즉시 중단.
#   안착부(355.5·353) = 밑동이 기둥 사이에 앉는 구간. 가벼운 접촉은 정상 안착이므로 문턱을 넓히고,
#                       넘으면 후퇴 후 '밀린 방향으로' 살짝 비켜 재시도한다(사람이 하는 동작).
CONTACT_PX = {"cam1": 6.0, "cam2": 5.0}          # 접근부 (0.6mm)
SEAT_CONTACT_PX = {"cam1": 25.0, "cam2": 25.0}   # 안착 판정 한도 (9/2 조정)
# ★안착 판정의 물리(9/2 실증): 정상 안착 마찰 = 벽점 8~12px 이동 / 기둥 위 '얹힘' = 로봇이
#   z353 까지 내려가는 동안 벽이 그리퍼에서 ~35mm(수백 px) 밀림. 두 신호는 수십 배 차이라
#   한도 25px 면 안착은 통과, 얹힘은 확실히 잡는다. (10~12px 한도는 정상 안착 11.7px 를
#   0.2mm 차로 퇴짜 놓았다 — 분해능보다 촘촘한 기준의 재판.)
SEAT_FROM_Z = 356.0                              # 이 아래가 안착부
SEAT_RETRY = 2                                   # 안착 접촉 시 재시도 횟수
SEAT_BACKOFF = 4.0                               # 후퇴 높이(mm)
SEAT_NUDGE = 0.6                                 # 비켜주는 양(mm)
ENTRY_STEPS = [384, 380, 376, 372, 368, 365.5, 363, 360.5, 358, 355.5, 353]

# 기둥점 면적 기준(descent_refs_place 같은 높이 실측) — 유령/오검출 컷용
SCENE_AREA = {478.0: {"cam1": 407, "cam2": 341}, 443.0: {"cam1": 603, "cam2": 526},
              413.0: {"cam1": 904, "cam2": 791}, 388.0: {"cam1": 1353, "cam2": 1170}}
SCENE_KIND = {"cam1": "yellow", "cam2": "red"}
# ★9/2 사용자 지시: 월드 앵커를 색 점 → ArUco 마커로 교체.
#   근거 실측: 카메라 설정 변경으로 색 블롭 무게중심이 4~10.5px 계통 이동(=착지 편향, 기둥 파손의
#   한 축)한 반면 마커 중심은 편차 0.0~0.3px. 마커는 노출·임계 무관 + 각도(회전)까지 준다.
ARUCO_PORT = {"cam1": 8766, "cam2": 8768}
A_REF = {}          # main 에서 로드: {z: {cam: {id(str): {cx,cy,ang,span}}}}  ★9/2 오후 다중 마커
# ★9/2 오후: 기둥 이설(y −33mm)로 id31/32 가 어느 높이에서도 안 보임 → 기준 행에 저장된
#   '보이는 마커 전부'(cam1 id30/35 · cam2 id33/34)를 앵커로 쓴다. 마커마다 자기 기준과 짝지어
#   최소자승에 넣으므로 하나가 가려져도 나머지로 해가 선다(사용자: 마커는 색점 보정용 앵커).


def aruco_now(cam, ids, n=6):
    """요청한 id 들의 현재 중심·각 (중앙값). {id(str): {cx,cy,ang}} — 3샘플 미만 id 는 제외."""
    import urllib.request as _u, statistics as _s
    port = ARUCO_PORT[cam]
    want = {int(i) for i in ids}
    acc = {}
    for _ in range(n):
        try:
            ms = json.loads(_u.urlopen(f"http://127.0.0.1:{port}/aruco", timeout=4).read())["markers"]
        except Exception:
            continue
        for m in ms:
            if m["id"] not in want:
                continue
            c = m["corners"]
            acc.setdefault(str(m["id"]), []).append((
                sum(q[0] for q in c) / 4, sum(q[1] for q in c) / 4,
                math.degrees(math.atan2(c[1][1]-c[0][1], c[1][0]-c[0][0]))))
        time.sleep(0.25)
    return {k: {"cx": _s.median(p[0] for p in v), "cy": _s.median(p[1] for p in v),
                "ang": _s.median(p[2] for p in v)}
            for k, v in acc.items() if len(v) >= max(3, n * 0.5)}


def speed(v):
    post("speed", {"value": v, "dry_run": False}); time.sleep(0.3)


def cam2_scene_scale(z):
    """새캠 밑판 축척은 높이별 2.66~3.43 px/mm (9/1 실측, 191mm 기선)."""
    t = (478.0 - z) / (478.0 - 388.0)
    return 2.66 + max(0.0, min(1.0, t)) * (3.43 - 2.66)


def scene_ratio(cam, z):
    """기둥 축척 / 벽 축척 — 파지편차 픽셀을 기둥 픽셀 이동량으로 바꾸는 비."""
    if cam == "cam1":
        return 3.105 / WALL_SCALE[cam]
    return cam2_scene_scale(z) / WALL_SCALE[cam]


def wall_refs(row, grasp):
    """벽점 기준 = ★dist_refs 그 높이 행의 w (성공 상태의 벽↔기둥 짝은 자기완결 —
    grasp_ref 파지사진과는 ~7px 차이가 있어 섞으면 0.7mm 계통오차). 면적 대역만
    grasp_ref 에서 y 순서로 빌린다(w 에는 면적이 없음)."""
    out = {}
    for cam in ("cam1", "cam2"):
        ws = sorted([e["w"] for e in row[cam]], key=lambda p: p[1])   # y 순
        ga = sorted(grasp[f"{cam}_wall"], key=lambda p: p[2])
        kind = ga[0][0] if ga else "blue"                  # 9/2 저녁: 벽 색 = 골든 파지사진 색(노랑 확장)
        out[cam] = [[kind, w[0], w[1], ga[i][3] if i < len(ga) else None]
                    for i, w in enumerate(ws)]
    return out


def match_wall(refs, cur):
    """캠별 벽점 짝: 골든 파지사진 점과 색·거리·면적 3중 조건. y 순서 고정."""
    out = {}
    for cam in ("cam1", "cam2"):
        pr = match(refs[cam], cur[cam])
        pr.sort(key=lambda t: t[0][2])
        out[cam] = pr
    return out


def wall_dev(pairs):
    """캠별 벽점 평균 편차(px)와 mm 환산 크기."""
    dev = {}
    for cam, pr in pairs.items():
        if not pr:
            dev[cam] = None
            continue
        dx = sum(c[1] - r[1] for r, c in pr) / len(pr)
        dy = sum(c[2] - r[2] for r, c in pr) / len(pr)
        dev[cam] = (dx, dy, math.hypot(dx, dy) / WALL_SCALE[cam])
    return dev


def wall_angle(pr):
    """짝지어진 벽점 2개의 (현재선 − 골든선) 화면각 차."""
    if len(pr) < 2:
        return None
    a, b = pr[0], pr[1]
    return wrap(math.degrees(math.atan2(b[1][2]-a[1][2], b[1][1]-a[1][1]))
                - math.degrees(math.atan2(b[0][2]-a[0][2], b[0][1]-a[0][1])))


def grasp_gate(refs, tag):
    """파지 검증: 골든 파지사진 대비 벽 밀림·회전. 통과 못 하면 예외(정지)."""
    cur = snap_all()
    pairs = match_wall(refs, cur)
    n = sum(len(v) for v in pairs.values())
    n1 = len(pairs.get("cam1", []))
    if n1 < 1:                              # ★앞끝(cam1) 벽점은 필수 — match_wall 은 골든 위치 ±60px 만 짝지으므로 빈손이면 0
        raise RuntimeError(f"{tag}: 앞끝(cam1) 벽점 없음 — 파지 실패, 정지")
    if n < 2:                               # ★9/3 red_s: cam2 벽점이 프레임 맨 위(y22)라 화면 밖 → cam1+그리퍼로 파지 확인, 진행
        print(f"{tag}: ⚠ 벽점 {n}개(cam2 미검출) — 앞끝 벽점 + 그리퍼 일치로 파지 확인, 진행")
    dev = wall_dev(pairs)
    worst = max(d[2] for d in dev.values() if d)
    angs = [a for a in (wall_angle(pairs["cam1"]), wall_angle(pairs["cam2"])) if a is not None]
    ang = sum(angs) / len(angs) if angs else 0.0
    ds = {c: (f"({dev[c][0]:+.1f},{dev[c][1]:+.1f})px" if dev[c] else "―")
          for c in ("cam1", "cam2")}
    print(f"{tag}: 벽 밀림 최대 {worst:.2f}mm · 벽선 편차 {ang:+.2f}° "
          f"(cam1 {ds['cam1']} · cam2 {ds['cam2']})")
    if worst > GRASP_DEV_CAP_MM:
        raise RuntimeError(f"{tag}: 파지 밀림 {worst:.2f}mm > {GRASP_DEV_CAP_MM} — 재파지 필요. 정지")
    if abs(ang) > GRASP_YAW_CAP:
        raise RuntimeError(f"{tag}: 파지 yaw {ang:+.2f}° > {GRASP_YAW_CAP}° — 재파지 필요. 정지")
    return pairs, dev, ang


# ★yaw 는 측정-게이트만 한다(grasp_gate 의 벽선 편차 상한). 보정 루프는 9/1 실기에서 봉인:
#   벽선-only 측정은 J6 보정을 제대로 관측하지 못한다 — J6 -2.7° 동안 측정이 +0.72→+1.73 으로
#   역행(감도가 0 도 -1.03 도 아닌 불명확한 값 + z 높이에 따라 측정 바이어스). 벽의 월드 yaw 는
#   골든 픽이 랙 벽점(월드 기준)으로 이미 닫아 주므로 place 에서 다시 돌리지 않는 게 맞다.


def scene_solve(row, refs, z):
    """한 프레임: 기둥점을 (파지편차 보상된) 골든 픽셀에 맞추는 (dX,dY)·잔차.
    두 캠 기둥점이 모두 보여야 한다(교차검증) — 아니면 정지 사유 반환.
    벽 편차는 이 높이 행의 w 기준(성공 상태와의 짝 유지)."""
    cur = snap_all()
    wp = match_wall(refs, cur)
    if not wp["cam1"] and not wp["cam2"]:
        return None, None, "벽점 전멸"
    dev = wall_dev(wp)
    worst = max(d[2] for d in dev.values() if d)
    if worst > GRASP_DEV_CAP_MM:
        return None, None, f"벽 밀림 {worst:.2f}mm(파지 이상)"
    pairs1, pairs2 = [], []
    for cam, plist in (("cam1", pairs1), ("cam2", pairs2)):
        # ★9/2: 벽을 물면 cam1 의 id31 이 벽에 가려진다(실증) — 보이는 마커만 쓴다.
        #   마커 하나면 XY 2자유도가 완전 결정이므로 cam2 단독으로 충분하다.
        ams = A_REF.get(round(z), {}).get(cam) or {}
        cms = aruco_now(cam, ams.keys()) if ams else {}
        d = dev[cam]
        if not cms or d is None:                       # 마커 없음/이 캠 벽점 없음 → 보상 불가
            continue
        r = scene_ratio(cam, z)
        for mid, cm in cms.items():
            am = ams[mid]
            dang = wrap(cm["ang"] - am["ang"])
            if abs(dang) > 2.0:
                return None, None, f"{cam} id{mid} 마커 각도 이상({dang:+.1f}° — 밑판 회전/오검출)"
            adj = ["aruco", am["cx"] + d[0] * r, am["cy"] + d[1] * r, None]
            plist.append((adj, ["aruco", cm["cx"], cm["cy"], None]))
    if not pairs1 and not pairs2:
        return None, None, "가용 마커 없음(양 캠 모두 가림/기준 없음)"
    sol, err = solve_scaled(pairs1, pairs2, z)
    # ★해 배율 보정(9/1 z388 실측): 알려진 이동 3회에 대해 해가 X 1.49배·Y 1.89배로
    #   과대 반응했다(0.600mm 이동 → 해 0.843/1.09~1.17 변화). 보정 없이는 mm 게이트가
    #   실제보다 큰 값을 보고 도달 불가능한 기준을 요구한다.
    sol = (sol[0] / RESP[0] + TRIM_XY[0], sol[1] / RESP[1] + TRIM_XY[1])
    # ★9/2 저녁: 손목 rz 가 캘리브 자세(±180)와 다르면 해를 로봇축으로 회전(노랑 슬롯 rz 90)
    th = math.radians(wrap(st()["tcp"][5] - 180.0))   # 실측 부호(9/2): robot = R(rz−180)·sol
    sol = (sol[0] * math.cos(th) - sol[1] * math.sin(th), sol[0] * math.sin(th) + sol[1] * math.cos(th))
    return sol, err, None


def solve_scaled(pairs1, pairs2, z):
    """픽셀 최소자승 — ★새캠 M 은 place 높이 실축척으로 스케일(해가 실제 mm 오차와 일치).
    골든의 solve_xy 는 M2 일반값(4.825) 그대로라 해 크기가 0.68배로 나온다(시뮬 실증)."""
    import numpy as np
    k2 = cam2_scene_scale(z) / 4.825
    M2e = [[M2[0][0]*k2, M2[0][1]*k2], [M2[1][0]*k2, M2[1][1]*k2]]
    A, b = [], []
    for M, pairs in ((M1, pairs1), (M2e, pairs2)):
        for r, c in pairs:
            A.append([M[0][0], M[0][1]]); b.append(r[1] - c[1])
            A.append([M[1][0], M[1][1]]); b.append(r[2] - c[2])
    if len(A) < 2:
        return None, None
    A = np.array(A, float); b = np.array(b, float)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return (float(sol[0]), float(sol[1])), float(np.sqrt(np.mean(b ** 2)))


def move_xy_r(dx, dy):
    """★9/1 실증한 두 함정을 모두 막는 이동.
      · 로봇은 축당 0.3mm 안팎의 명령을 통째로 버린다(0.6mm 는 정확히 0.600 실행).
      · golden.move_xy 의 도달검증은 축당 0.3mm 허용이라, 그렇게 버려진 이동도
        '도달'로 통과시킨다 → 7회 연속 제자리인데 누적 1.9mm 로 기록된 사고.
    여기서는 명령 크기를 최소 실행치 이상으로 올리고, 도달을 0.15mm 로 엄격히 본다."""
    b = stable()
    t = list(b); t[0] += dx; t[1] += dy
    for k in range(2):
        post("move_tcp", {"tcp": t, "dry_run": False})
        time.sleep(0.4)
        t0 = time.time()
        while time.time() - t0 < 30 and st()["busy"]:
            time.sleep(0.3)
        g = stable()
        if abs(g[0] - t[0]) <= 0.15 and abs(g[1] - t[1]) <= 0.15:
            return
        if k:
            raise RuntimeError(f"XY 이동 미도달: 명령 ({t[0]:.2f},{t[1]:.2f}) 도달 ({g[0]:.2f},{g[1]:.2f})")
        print(f"  (이동 누락 — 재시도) 명령 ({t[0]:.2f},{t[1]:.2f}) 도달 ({g[0]:.2f},{g[1]:.2f})")
        time.sleep(0.6)


def align_at(row, refs, z, tol, mtol, cap):
    """한 높이에서 골든 일치까지 정렬. px 잔차·해 크기(실 mm) 둘 다 만족 + 새 스냅샷 재확인.
    스텝은 항상 MIN_STEP 이상이라 '명령했는데 안 움직였다'가 생기지 않는다(9/1 사고 원인)."""
    moved = 0.0
    for it in range(MAX_IT):
        sol, err, why = scene_solve(row, refs, z)
        if why:
            raise RuntimeError(f"z{z:.0f}: {why} — 정지")
        mag = math.hypot(*sol)
        print(f"  z{z:.0f} [{it+1}] 잔차 {err:5.2f}px  해 ({sol[0]:+.2f},{sol[1]:+.2f})mm  누적 {moved:.1f}mm")
        # ★9/2: 수렴의 실질 기준은 '실 mm 오차'(RESP 보정 후, 최종단 0.35mm). 픽셀 잔차는
        #   두 캠 불일치 바닥이 조명·점크기에 따라 2.3~3.4px 로 떠다녀서 1차 기준으로 쓰면
        #   '다 맞았는데 못 넘는' 진동이 난다(z443 실증: 0.41mm 인데 2.96px 로 재시도→±1.15 진동).
        #   → mm 게이트 + 픽셀은 총체 이상 상한(4.5px)만.
        px_ceil = 4.5 + 3.0 * math.hypot(*TRIM_XY)   # 트림만큼 마커가 기준에서 비껴 앉는 몫
        if mag < mtol and err < max(tol, px_ceil):
            sol2, err2, why2 = scene_solve(row, refs, z)          # ★재확인(새 스냅샷)
            if why2:
                raise RuntimeError(f"z{z:.0f}: 재확인 {why2} — 정지")
            if math.hypot(*sol2) < mtol and err2 < max(tol, px_ceil):
                print(f"  z{z:.0f}: ✔ 골든 일치(재확인 {err2:.2f}px·{math.hypot(*sol2):.2f}mm)")
                return
            print(f"  z{z:.0f}: 재확인 불일치({err2:.2f}px·{math.hypot(*sol2):.2f}mm) — 계속")
            continue
        if err < 6.0 and mag > 3.0:
            raise RuntimeError(f"z{z:.0f}: 해 불안정(잔차 {err:.1f}px vs 해 {mag:.1f}mm) — 정지")

        dX = max(-CAP_MM, min(CAP_MM, GAIN * sol[0]))
        dY = max(-CAP_MM, min(CAP_MM, GAIN * sol[1]))
        step = math.hypot(dX, dY)
        if 0 < step < MIN_STEP:
            # 로봇이 버리지 않는 최소 크기까지 키운다(방향 유지). 과보정분은 다음 판에서 잡힌다.
            dX, dY = dX / step * MIN_STEP, dY / step * MIN_STEP
        if moved + math.hypot(dX, dY) > cap:
            raise RuntimeError(f"z{z:.0f}: 누적 이동 상한 {cap}mm — 정지")
        # ★9/2: 소이동 부분실행이 place 자세에서도 발생(0.57mm 명령에 0.13mm 실행 실측).
        #   실행될 때까지 키워 보내고(0.45→1.15→1.3) '실이동'만 누적한다 — 게이트는 그대로.
        ax, ay = move_xy_eff(dX, dY)
        moved += math.hypot(ax, ay)
        time.sleep(0.45)
    raise RuntimeError(f"z{z:.0f}: {MAX_IT}회 내 미수렴 — ★기둥 진입 금지. 정지")


def wall_now(refs, n=3):
    """빠른 벽점 판독(진입 감시용) — 캠별 평균 편차 px."""
    cur = {"cam1": dots("cam1", n), "cam2": dots("cam2", n)}
    return wall_dev(match_wall(refs, cur))


def px_to_mm_wall(cam, dpx):
    """벽점 픽셀 밀림 → mm 벡터(로봇 XY). 벽은 손목에 가까워 축척이 다르다(WALL_SCALE)."""
    import numpy as np
    M = M1 if cam == "cam1" else M2
    base = 3.105 if cam == "cam1" else 4.825
    k = WALL_SCALE[cam] / base
    Mw = np.array([[M[0][0]*k, M[0][1]*k], [M[1][0]*k, M[1][1]*k]], float)
    return tuple(float(v) for v in np.linalg.solve(Mw, np.array(dpx, float)))


def move_z_slow(zt, tol=1.2):
    t = list(stable()); t[2] = float(zt)
    post("move_tcp", {"tcp": t, "dry_run": False})
    time.sleep(0.4)
    t0 = time.time()
    while time.time() - t0 < 25 and st()["busy"]:
        time.sleep(0.3)
    return _verify_reach(t, tol), t


def guarded_descend(z_from, z_to, refs, step=3.0):
    """단간 감시 하강 (9/2): 기둥 상단 통과 구간(z443 이하)을 잔스텝으로 내려가며 벽점을
    감시한다. 0.6mm 밀림 = 접촉 → 4mm 후퇴 + 밀린 방향으로 비켜 1회 재시도, 재발 시 정지."""
    base = wall_now(refs)
    if not any(base.values()):
        raise RuntimeError("감시 하강 전 벽점 판독 실패 — 정지")
    speed(1)
    retried = False
    z = z_from
    while z > z_to + 0.5:
        z = max(z_to, z - step)
        ok, t = move_z_slow(z)
        if not ok:
            raise RuntimeError(f"감시 하강 z{z:.0f} 미도달 — 걸림 의심, 정지(그리퍼 무조작)")
        now = wall_now(refs)
        hit = None
        for cam in ("cam1", "cam2"):
            if base[cam] and now[cam]:
                dpx = (now[cam][0]-base[cam][0], now[cam][1]-base[cam][1])
                if math.hypot(*dpx) > CONTACT_PX[cam]:
                    now2 = wall_now(refs)
                    if now2[cam]:
                        dpx = (now2[cam][0]-base[cam][0], now2[cam][1]-base[cam][1])
                        if math.hypot(*dpx) > CONTACT_PX[cam]:
                            hit = (cam, dpx)
                            break
        if hit is None:
            continue
        cam, dpx = hit
        d = math.hypot(*dpx)
        if retried:
            u = list(stable()); u[2] += 25.0
            post("move_tcp", {"tcp": u, "dry_run": False}); time.sleep(1.0)
            raise RuntimeError(f"기둥 상단 접촉 재발(z{z:.0f}, {cam} {d:.1f}px) — 정지")
        retried = True
        mx, my = px_to_mm_wall(cam, dpx)
        L = math.hypot(mx, my)
        nx, ny = (mx/L*SEAT_NUDGE, my/L*SEAT_NUDGE) if L > 1e-6 else (0.0, 0.0)
        print(f"  ↺ z{z:.0f}: 기둥 상단 접촉 {d:.1f}px({cam}) → 4mm 후퇴 + ({nx:+.2f},{ny:+.2f})mm 비켜 재시도", flush=True)
        u = list(stable()); u[2] += 4.0
        post("move_tcp", {"tcp": u, "dry_run": False}); time.sleep(0.4)
        t0 = time.time()
        while time.time() - t0 < 20 and st()["busy"]:
            time.sleep(0.3)
        move_xy_eff(nx, ny)
        base = wall_now(refs)
        if not any(base.values()):
            raise RuntimeError("재시도 기준 판독 실패 — 정지")
        z += 4.0



def commit_descend(z_to, refs):
    """★9/2 사용자 설계: 검사는 '꽂기 전'(z478·z443 정렬)에 끝낸다. 그 뒤는 멈추지 말고
    z353 까지 한 번에 내려간다. 하강 중에는 이동을 세우는 검사 없이, 빠른 단발 판독으로
    벽점만 곁눈질하다가(비상 브레이크) 실제로 밀릴 때만 — 2연속 10px(≈1mm) — 정지·상승한다."""
    base = wall_now(refs)
    if not any(base.values()):
        raise RuntimeError("하강 전 벽점 판독 실패 — 정지")
    speed(3)                              # 9/3 속도(사용자 지시 "기둥 진입 후 빠르게"): 채널 안 하강 2→3%(벽 밀림 감시 그대로)
    t = list(stable()); t[2] = float(z_to)
    post("move_tcp", {"tcp": t, "dry_run": False})
    time.sleep(0.4)
    bad = 0
    # ★9/2: 고정 90초 한도가 버그였다 — 90mm 를 1% 속도로 내려가면 2분 넘게 걸려,
    #   멀쩡히 내려가는 중에 '미도달'로 오판해 중단시켰다(실증 z378 에서 중단, 접촉 0회).
    #   → '진행 중이면 기다린다': z 가 10초간 안 변할 때만 이상으로 본다.
    last_z, last_t = st()["tcp"][2], time.time()
    t0 = time.time()
    while time.time() - t0 < 420 and st()["busy"]:
        zc = st()["tcp"][2]
        if abs(zc - last_z) > 0.5:
            last_z, last_t = zc, time.time()
        elif time.time() - last_t > 10.0:
            raise RuntimeError(f"하강 정체(z{zc:.1f} 에서 10초 무진행) — 걸림 의심, 정지")
        slip = 0.0
        try:
            for cam, port in (("cam1", 8766),):
                ds = json.loads(urllib.request.urlopen(
                    f"http://127.0.0.1:{port}/dots?raw=1", timeout=2).read())["dots"]
                cur = {cam: [[d["kind"], d["px"], d["py"], d["area"]] for d in ds],
                       "cam2": []}
                pr = match_wall(refs, cur)[cam]
                if pr and base[cam]:
                    dx = sum(c[1]-r[1] for r, c in pr)/len(pr) - base[cam][0]
                    dy = sum(c[2]-r[2] for r, c in pr)/len(pr) - base[cam][1]
                    slip = math.hypot(dx, dy)
        except Exception:
            pass
        if slip > 10.0:
            bad += 1
            if bad >= 2:
                u = list(st()["tcp"]); u[2] += 25.0
                post("move_tcp", {"tcp": u, "dry_run": False}); time.sleep(1.0)
                raise RuntimeError(f"하강 중 벽 밀림 {slip:.0f}px — 비상정지·상승")
        else:
            bad = 0
        time.sleep(0.5)
    if not _verify_reach(t, 1.5):
        raise RuntimeError(f"z{z_to} 미도달 — 걸림 의심, 정지(그리퍼 무조작)")
    # ★안착 판정 게이트 (9/2 사용자 지시: '꽂혔는지' 판정이 정확해야 놓기가 안전하다):
    #   벽이 홈에 안 들어가고 기둥에 얹히면 로봇은 z353 에 도달해도 벽이 그리퍼 안에서 위로
    #   밀린다 → 벽점이 기준 대비 크게 이동. 정상 안착 접촉은 12px(1.2mm) 이내(9/1~2 실측).
    #   한도를 넘으면 '삽입 완료'를 선언하지 않는다 → 놓기(그리퍼 열기)가 원천 차단된다.
    now = wall_now(refs)
    seated = True
    for cam in ("cam1", "cam2"):
        if base[cam] and now[cam]:
            d = math.hypot(now[cam][0]-base[cam][0], now[cam][1]-base[cam][1])
            print(f"  안착 판정 {cam}: 벽점 변화 {d:.1f}px (한도 {SEAT_CONTACT_PX[cam]})")
            if d > SEAT_CONTACT_PX[cam]:
                seated = False
    if not seated:
        u = list(stable()); u[2] += 25.0
        post("move_tcp", {"tcp": u, "dry_run": False}); time.sleep(1.0)
        raise RuntimeError("안착 판정 실패 — 벽이 홈에 안 들어감(얹힘 의심). 상승·정지, 놓기 금지")
    print(f"삽입 완료 z{z_to}: TCP {[round(v,2) for v in stable()[:3]]} — 그리퍼 무조작, 놓기는 사용자 명령")


def entry_descend(refs, final_z):
    """기둥 진입: 잔스텝 하강 + 매 스텝 벽점 감시.
    접근부에서 밀리면 걸림 → 즉시 중단. 안착부는 문턱을 넓히고, 넘으면 후퇴·비켜서 재시도."""
    base = wall_now(refs)
    if not any(base.values()):
        raise RuntimeError("진입 전 벽점 판독 실패 — 정지")
    print(f"  진입 시작(기준 cam1 {base['cam1'] and (round(base['cam1'][0],1), round(base['cam1'][1],1))}px)")
    speed(1)
    retries = 0
    i = 0
    while i < len(ENTRY_STEPS):
        zt = ENTRY_STEPS[i]
        seating = zt < SEAT_FROM_Z
        lim = SEAT_CONTACT_PX if seating else CONTACT_PX
        ok, t = move_z_slow(zt)
        if not ok:
            raise RuntimeError(f"진입 z{zt} 미도달 — ★걸림 의심. 그 자리 정지(그리퍼 무조작)")
        now = wall_now(refs)
        hit = None
        for cam in ("cam1", "cam2"):
            if base[cam] and now[cam]:
                d = math.hypot(now[cam][0]-base[cam][0], now[cam][1]-base[cam][1])
                if d > lim[cam]:
                    now2 = wall_now(refs)                          # 노이즈 배제 재판독
                    if not now2[cam]:
                        continue
                    dpx = (now2[cam][0]-base[cam][0], now2[cam][1]-base[cam][1])
                    d2 = math.hypot(*dpx)
                    if d2 > lim[cam]:
                        hit = (cam, d2, dpx)
                        break
        if hit is None:
            print(f"  z{zt} ✓", flush=True)
            i += 1
            continue

        cam, d2, dpx = hit
        if not seating:
            print(f"  ⛔ z{zt}: {cam} 벽점 {d2:.1f}px 밀림 = 걸림! 25mm 상승")
            u = list(stable()); u[2] += 25.0
            post("move_tcp", {"tcp": u, "dry_run": False})
            time.sleep(1.0)
            raise RuntimeError(f"기둥 접촉으로 중단(z{zt}, {cam} {d2:.1f}px) — 재정렬 필요")

        # 안착부 접촉 → 후퇴 + 밀린 방향으로 비켜서 재시도
        if retries >= SEAT_RETRY:
            print(f"  ⛔ z{zt}: 안착 재시도 {SEAT_RETRY}회 모두 접촉({d2:.1f}px) — 25mm 상승 후 중단")
            u = list(stable()); u[2] += 25.0
            post("move_tcp", {"tcp": u, "dry_run": False})
            time.sleep(1.0)
            raise RuntimeError(f"안착 실패(z{zt}, {cam} {d2:.1f}px) — 재정렬 필요")
        retries += 1
        # 벽이 밀린 쪽 = 벽이 있어야 했던 쪽. 그쪽으로 TCP 를 따라 옮겨 주면 기둥을 비껴간다.
        mx, my = px_to_mm_wall(cam, dpx)
        L = math.hypot(mx, my)
        nx, ny = (mx / L * SEAT_NUDGE, my / L * SEAT_NUDGE) if L > 1e-6 else (0.0, 0.0)
        print(f"  ↺ z{zt}: 안착 접촉 {d2:.1f}px({cam}) → {SEAT_BACKOFF}mm 후퇴 + "
              f"({nx:+.2f},{ny:+.2f})mm 비켜 재시도 {retries}/{SEAT_RETRY}", flush=True)
        u = list(stable()); u[2] += SEAT_BACKOFF
        post("move_tcp", {"tcp": u, "dry_run": False})
        time.sleep(0.4)
        t0 = time.time()
        while time.time() - t0 < 20 and st()["busy"]:
            time.sleep(0.3)
        move_xy_r(nx, ny)
        base = wall_now(refs)                    # 비킨 뒤를 새 기준으로(이동분을 접촉으로 오인 방지)
        if not any(base.values()):
            raise RuntimeError("재시도 기준 판독 실패 — 정지")
        time.sleep(0.4)
    print(f"삽입 완료 z{final_z}: TCP {[round(v,2) for v in stable()[:3]]} — 그리퍼 무조작, 놓기는 사용자 명령")


def main():
    if len(sys.argv) < 3 or sys.argv[1] != "run":
        sys.exit("사용법: golden_place.py run blue [--no-approach]")
    ck = sys.argv[2]
    cal = json.load(open(CAL))
    ref = cal["refs"][ck]
    D = ref["dist_refs"]
    grasp = ref["grasp_ref"]
    it_tcp = ref["insert_tcp"]
    rows = sorted(D["rows"], key=lambda r: -r["z"])
    refs_by_z = {r["z"]: wall_refs(r, grasp) for r in rows}
    AP = ref.get("aruco_place")
    if not AP:
        sys.exit("aruco_place 기준 없음 — 캡처 먼저 (9/2 마커 앵커 방식)")
    for r_ in AP["rows"]:
        A_REF[round(r_["z"])] = {}
        for c in ("cam1", "cam2"):
            v = r_.get(c)
            if not v:
                continue
            A_REF[round(r_["z"])][c] = {str(v["id"]): v} if "cx" in v else dict(v)   # 구형(단일)/신형(id별)

    g = int(str(post("grip_read", {"dry_run": True})["result"] or 0))
    if abs(g - grasp["grip"]) > 3:
        sys.exit(f"그리퍼 실측 {g} (골든 파지 {grasp['grip']}) — 벽을 물고 있지 않음. 중단")

    if "--no-approach" not in sys.argv:
        cur = stable()
        # ★9/5 사용자 지시("산업용이면 이렇게 느리면 안 된다 — 이동은 빠르게, 정렬은 어쩔 수 없고"):
        #   이 블록은 전부 '이동'이다 — 랙에서 벽을 들어 올려 조립대 위 SAFE_Z 로 운반하는 구간.
        #   측정도 정렬도 하지 않고, 기둥 꼭대기(133mm)보다 한참 위 자유공간이라 속도를 올려도
        #   품질에 닿지 않는다. 정렬(1%)은 아래 speed(1) 부터 그대로다.
        speed(MOVE_SPEED)
        if cur[2] < SLOW_Z:
            move_z(SLOW_Z)               # 벽 들어 올리기 — 사용자 승인으로 이 속도 사용
        move_z(SAFE_Z)
        tgt = [it_tcp[0], it_tcp[1], SAFE_Z] + list(it_tcp[3:])
        post("move_tcp", {"tcp": tgt, "dry_run": False})
        time.sleep(0.4)
        t0 = time.time()
        while time.time() - t0 < 90 and st()["busy"]:
            time.sleep(0.15)
        if not _verify_reach(tgt, 2.0, timeout=15):
            sys.exit(f"운반 미도달: 목표 {[round(v,1) for v in tgt[:3]]} "
                     f"현재 {[round(v,1) for v in stable()[:3]]} — 정지")
        move_z(rows[0]["z"])             # 첫 정렬단 높이까지 하강도 이동 구간(측정 전)
    speed(1)                             # ← 여기부터 정렬: 1% 고정(9/1 밤 5% 진동 실패로 철회된 값)

    # 1) 파지 검증 (★최상단 높이로 올라가서 — 벽선 각 측정은 z 에 따라 바이어스가 있어
    #    기준 높이에서 재야 한다. 9/1: 같은 파지가 z478 에서 +0.32°, z388 에서 +0.74°)
    move_z(rows[0]["z"])
    time.sleep(0.6)
    top_refs = refs_by_z[rows[0]["z"]]
    grasp_gate(top_refs, "파지검증")

    # 2) 4단 정렬 — 아래로 갈수록 타이트 (픽 3px 보다 엄격)
    # ★9/2 규명: 기둥 꼭대기 ≈ z439 (안착 353 + 가이드 86). z443→413 하강이 밑동의 기둥
    #   상단 통과 구간인데 감시가 z388 부터라 '사각지대'였다 — 어제 성공은 이 구간을 무사히
    #   지난 것, 오늘 실패는 여기서 스쳐 벽이 그리퍼 안에서 6.3mm 회전(강체 회전 패턴 실증).
    #   → z443 아래 단간 하강도 진입과 같은 보호(잔스텝+벽점 감시+후퇴·비켜 재시도)로 내려간다.
    # ★9/2 사용자 설계: 검사는 꽂기 전(478·443 정렬)까지만. 그 아래는 무정지 연속 하강.
    for row in rows:
        z = row["z"]
        if z < 440.0:
            continue                          # 413·388 정렬 생략 — 꽂기 전 검사로 충분
        move_z(z)
        time.sleep(0.6)
        align_at(row, refs_by_z[z], z, TOL_BY_Z.get(z, 1.5),
                 MAG_TOL_BY_Z.get(z, 0.35), CUM_CAP_BY_Z.get(z, 2.0))

    # 3) 연속 하강 → 안착
    try:
        commit_descend(D["final_z"], refs_by_z[rows[-1]["z"]])
    except BaseException:
        # ★9/2 기둥 파손 교훈: 예외로 빠져나갈 때 진행 중이던 하강 명령이 살아 있으면
        #   로봇이 계속 내려가며 힘을 가한다 — 어떤 경로로 죽든 정지가 먼저다.
        try:
            post("stop", {"dry_run": False})
        except Exception:
            pass
        raise


if __name__ == "__main__":
    main()
