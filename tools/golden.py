#!/usr/bin/env python3
"""골든 정렬 하강 — 픽·플레이스 공용 (사용자 설계 최종형, 9/1 야간).

  python3 golden.py teach pick  blue    # 골든 자세(사용자가 보정해준 성공 자세)에서 4단 기준 기록
  python3 golden.py teach place blue
  python3 golden.py run   pick  blue    # 4단 정렬 하강 → 골든 일치 → 수직 하강 (그리퍼는 안 건드림)
  python3 golden.py run   place blue

★사용자 설계 (9/1 확정) — 이대로만 동작한다
  1. 기준은 사용자가 보정해준 '골든' 자세에서 찍은 4단계 화면이다.
  2. 내려올 때마다 각 높이에서 화면이 골든과 같아질 때까지 X·Y 를 0.5mm 씩 정렬한다.
  3. 골든이 되면 수직으로 내려간다. 파지/삽입은 사용자(또는 호출자)가 명령한다.
  4. 검증 없이는 아무것도 하지 않는다:
       · 필요한 점이 다 안 보이면 정지     · 잔차와 보정량이 모순이면 정지(해 불안정)
       · 한 높이 누적 이동 상한 초과면 정지 · 이동 후 도달 확인(스테일 배제) 실패면 정지
       · 수렴 실패면 절대 다음으로 안 넘어간다(기둥 보호)

★오늘 실증된 함정과 대책 (전부 이 파일에 반영)
  · 축척은 대상 거리마다 다르다 → 캘리브 행렬(M)의 '방향'만 믿고, 크기는 게인 0.5 + 0.5mm 상한
    반복으로 흡수한다(방향이 맞으면 반복은 수렴한다. 크기 추측이 오늘 진동의 원인).
  · 트래커 낡은 트랙이 틀린 좌표를 내보낸다 → 여기서는 raw 검출만 쓰고, 기준 점과
    (색·거리·면적대역) 3중 조건으로 직접 짝짓는다.
  · 면적 60 유령 → 기준 점 면적의 40% 미만은 짝짓기에서 제외.
  · 픽은 대상(랙의 벽)을 골든 픽셀 위치로, 플레이스는 (벽↔기둥) 골든 거리로 맞춘다.
    플레이스에서 절대 픽셀을 맞추면 파지가 달라질 때마다 무너진다(오늘 실패의 근본).
"""
import json
import math
import statistics as S
import sys
import time
import urllib.request

BRIDGE = "http://localhost:8765"
CAL = "/home/ar/bf2_console/dot_calib.json"
PORT = {"cam1": 8766, "cam2": 8768}
# 캘리브된 픽셀/mm 행렬 (9/1 실측·검증: 복귀잔차 0.2px/1.0px). '방향'의 근거.
M1 = [[3.09, 0.00], [0.02, -3.12]]        # cam1: dpx = M1 @ (dX,dY)
M2 = [[4.825, 0.15], [0.175, -4.825]]     # cam2
OFFS = [35, 60, 90, 125]                  # 골든 z + 이 값들 = 4단 높이
# ★9/1 사용자 지시: 스텝을 1mm 로 통일. 0.5mm 는 자세에 따라 로봇이 통째로 버려서
#   (랙 위 z379 실측: 0.50mm→0.00mm) 정렬이 제자리에서 시간만 먹었다.
GAIN, CAP_MM = 0.7, 1.0                   # 한 번에 최대 1mm, 계산값의 0.7배
                                          # (해가 실이동보다 ~1.5배 크게 나와 0.7 이 실질 1배)
# yaw: J6 는 1° 미만 명령에 신뢰성이 없다(9/1 실측 +0.5°→+0.039°). 0.5~1.0° 사이로만 보낸다.
# ★허용치는 최소 실행 스텝보다 커야 한다(9/1 실증): 0.30° 허용 + 0.50° 최소스텝이면
#   0.31° 편차에 0.50° 를 때려 부호가 뒤집히고, 남은 회전 성분(0.5°≈5px)은 XY 평행이동으로
#   지울 수 없어 잔차가 5.5px 에 고정된다 → 픽이 영영 수렴하지 못하고 상한에서 정지했다.
YAW_TOL, J6_STEP_MIN, J6_STEP_MAX = 0.45, 0.5, 1.0
DANG_DJ6 = -1.03    # 월드 선의 화면각 변화 / J6 1° (9/1 ArUco 실측: J6+2° → 화면각 -2.06°)
TOL_PX = 3.0                              # 수렴 허용(≈1mm)
# 대안 수렴: 해 크기가 이 이하면 잔차 6px 까지 인정.
# ★스텝 1mm 의 정렬 바닥은 ±0.5mm 이고 해는 실이동의 ~1.5배로 나오므로 기준도 그에 맞춰야 한다
#   (더 촘촘하면 원리적으로 도달 불가 → 오늘 픽이 상한까지 헛돈 이유). 픽의 잔여 오차는
#   place 가 파지편차를 실측해 보상하므로 이 정도로 충분하다.
MAG_OK, PX_CEIL = 1.0, 8.0
MIN_STEP = 1.0                            # 기본 스텝 = 1mm (사용자 지시)
ESC_MAX = 1.3                             # 1mm 도 무시되는 자세를 대비한 상한
PICK_SPEED = 1                            # 정렬 속도(%) — 5% 로 올렸다가 진동으로 철회(9/1)
MAX_IT = 40    # 0.5mm×40 = 첫 단에서 최대 20mm 추종(벽 이동 허용량). 스텝 크기는 사용자 지시대로 0.5mm 유지
CUM_CAP = {0: 20.0, 1: 6.0, 2: 3.0, 3: 2.5}   # 높이 인덱스별 누적 이동 상한(mm) — 위는 넉넉, 아래는 엄격
AREA_LO, AREA_HI = 0.4, 3.0               # 기준 대비 면적 허용 대역(유령/오검출 컷)
PAIR_R = 220.0                            # 짝짓기 최대 거리


def post(a, b):
    r = urllib.request.Request(f"{BRIDGE}/fr5/{a}", json.dumps(b).encode(),
                               {"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=45).read())


def st():
    return json.loads(urllib.request.urlopen(BRIDGE + "/status", timeout=5).read())["robots"]["fr5"]


def wait(t=120):
    time.sleep(0.35)                      # 9/3 속도: 0.9→0.35 (busy 플래그로 판별, 측정 아님)
    t0 = time.time()
    while time.time() - t0 < t and st()["busy"]:
        time.sleep(0.12)


def stable(n=4):
    p, c = None, 0
    for _ in range(50):
        t = st()["tcp"]
        if p and all(abs(a - b) < 0.05 for a, b in zip(t, p)):
            c += 1
        else:
            c = 0
        p = t
        if c >= n:
            break
        time.sleep(0.12)                  # 9/3 속도: 0.2→0.12 (샘플 수 n 유지)
    return p


def _verify_reach(tgt, tol, timeout=8.0):
    """★도달 검증은 '한 번 읽고 판정'이 아니라 '들어올 때까지 관찰'.
    이동 직후 판독은 스테일이라(오늘 3회 실증) 즉시 읽으면 이전 위치가 나온다."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = st()["tcp"]
        if all(abs(p[i] - tgt[i]) <= tol for i in range(3)):
            return True
        time.sleep(0.25)
    return False


def move_z(z):
    t = list(stable()); t[2] = float(z)
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    if not _verify_reach(t, 1.5):
        raise RuntimeError(f"z 이동 미도달: 목표 {z:.1f} 도달 {stable()[2]:.1f}")


FAR_MM, FAR_CAP = 4.0, 3.0   # 9/3 속도: 첫 단에서 해가 4mm 넘게 멀면 3mm 스텝(측정 근처 1mm 스텝·미세 마무리는 그대로)


def clamp_step(sol, far_ok=False):
    """보정량 → 첫 시도 스텝(방향 유지, 최소 실행 크기 이상)."""
    cap = FAR_CAP if (far_ok and math.hypot(*sol) > FAR_MM) else CAP_MM
    dX = max(-cap, min(cap, GAIN * sol[0]))
    dY = max(-cap, min(cap, GAIN * sol[1]))
    step = math.hypot(dX, dY)
    if 0 < step < MIN_STEP:
        dX, dY = dX / step * MIN_STEP, dY / step * MIN_STEP
    return dX, dY


def move_xy_eff(dX, dY):
    """★실행될 때까지 스텝을 키우며 명령하고, '실제로 움직인 양'을 돌려준다.

    9/1 실측: 로봇의 최소 실행 이동량은 자세마다 다르다.
      랙 위 z379 → 0.30/0.45/0.50mm 는 0.00mm, 0.60mm 는 0.04mm, 0.80mm 라야 0.80mm 실행.
      조립대 z388 → 0.60mm 가 정확히 0.600mm 실행.
    고정 최소치로는 어느 자세에서든 맞출 수 없으므로, 안 움직이면 키워서 다시 보낸다
    (사용자 철칙의 0.5~1mm 대역 안에서만). 반환값을 누적에 써야 '안 간 이동'이
    누적 상한을 잡아먹는 사고를 막는다(9/1 실증: 제자리인데 상한 도달로 정지)."""
    b = stable()
    want = math.hypot(dX, dY)
    ux, uy = (dX / want, dY / want) if want > 1e-9 else (0.0, 0.0)
    for size in (want, max(want, 1.15), ESC_MAX):
        if size < want - 1e-6:
            continue
        t = list(stable()); t[0] += ux * size; t[1] += uy * size
        post("move_tcp", {"tcp": t, "dry_run": False}); wait()
        time.sleep(0.35)
        g = stable()
        moved = math.hypot(g[0] - b[0], g[1] - b[1])
        if moved > 0.3 * size:                      # 실제로 움직였다
            return (g[0] - b[0], g[1] - b[1])
        if size >= ESC_MAX - 1e-6:
            break
        print(f"  (명령 {size:.2f}mm 무시됨 — 이 자세의 분해능 한계, 키워서 재시도)")
    g = stable()
    return (g[0] - b[0], g[1] - b[1])


def move_xy(dX, dY):
    """★9/1: 로봇이 서브mm 명령을 부분 실행/누락하는 일이 있다(실측 0.38mm 미달로 픽 중단).
    정렬 루프는 매 반복 다시 측정하므로 '덜 간 이동'은 오류가 아니라 작은 스텝일 뿐이다.
    → 1회 재시도하고, 남은 미달이 0.6mm 미만이면 알리고 계속(다음 반복이 보정한다).
      그보다 크게 어긋나면 진짜 이상이므로 종전대로 정지."""
    b = stable()
    t = list(b); t[0] += dX; t[1] += dY
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    if _verify_reach(t, 0.3):
        return
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()      # 같은 절대 목표로 재시도
    if _verify_reach(t, 0.3):
        return
    g = stable()
    short = math.hypot(g[0] - t[0], g[1] - t[1])
    if short < 0.6:
        print(f"  (이동 {short:.2f}mm 미달 — 다음 반복에서 보정)")
        return
    raise RuntimeError(f"XY 이동 미도달: 명령 ({t[0]:.2f},{t[1]:.2f}) 도달 ({g[0]:.2f},{g[1]:.2f})")


SAMPLE_N, SAMPLE_DT = 6, 0.1      # 기본값 = place 가 쓰는 검증된 값. 픽은 run 에서 낮춘다.


def dots(cam, n=None):
    """raw 검출 → 군집 중앙값. 트래커를 거치지 않는다(낡은 트랙 사고 차단).
    ★샘플 수는 SAMPLE_N/SAMPLE_DT 로 조절. 카메라는 8Hz 이므로 0.07초 간격 16회면
      약 9프레임을 담아 중앙값이 흔들리지 않는다(검출 자체는 0.6ms 로 즉답)."""
    n = SAMPLE_N if n is None else n
    acc = {}
    for _ in range(n * 4):
        try:
            ds = json.loads(urllib.request.urlopen(
                f"http://127.0.0.1:{PORT[cam]}/dots?raw=1", timeout=5).read())["dots"]
        except Exception:
            continue
        for t in ds:
            k = None
            for kk in acc:
                if kk[0] == t["kind"] and (kk[1]-t["px"])**2 + (kk[2]-t["py"])**2 < 30**2:
                    k = kk
                    break
            if k is None:
                k = (t["kind"], t["px"], t["py"])
                acc[k] = []
            acc[k].append((t["px"], t["py"], t["area"]))
        time.sleep(SAMPLE_DT)
    return [[k[0], round(S.median([p[0] for p in v]), 2),
             round(S.median([p[1] for p in v]), 2),
             round(S.median([p[2] for p in v]))]
            for k, v in acc.items() if len(v) >= 3]


def match(ref, cur):
    """기준 점 ↔ 현재 점: 색 일치 + 거리 + 면적대역 3중 조건. [(ref, cur), ...]"""
    out = []
    for r in ref:
        best, bd = None, PAIR_R ** 2
        for c in cur:
            if c[0] != r[0]:
                continue
            if r[3] and not (AREA_LO * r[3] <= c[3] <= AREA_HI * r[3]):
                continue
            d = (c[1]-r[1])**2 + (c[2]-r[2])**2
            if d < bd:
                best, bd = c, d
        if best is not None:
            out.append((r, best))
    return out


def solve_xy(pairs1, pairs2, want_dist=None):
    """짝지어진 점들로 (dX,dY) 최소자승.
    want_dist=None  → 픽: 현재 점을 기준 픽셀로 (M @ d = ref-cur)
    want_dist=refs  → 플레이스: (벽↔기둥) 거리를 기준 거리로"""
    A, b = [], []
    if want_dist is None:
        for M, pairs in ((M1, pairs1), (M2, pairs2)):
            for r, c in pairs:
                A.append([M[0][0], M[0][1]]); b.append(r[1] - c[1])
                A.append([M[1][0], M[1][1]]); b.append(r[2] - c[2])
    else:
        for M, dd in ((M1, want_dist.get("cam1") or []), (M2, want_dist.get("cam2") or [])):
            for item in dd:
                (wx, wy), (sx, sy), dref, dcur = item
                L = max(dcur, 1.0)
                ux, uy = (sx - wx) / L, (sy - wy) / L        # 거리 증가 방향(밑판 쪽)
                A.append([ux*M[0][0] + uy*M[1][0], ux*M[0][1] + uy*M[1][1]])
                b.append(dref - dcur)
    if len(A) < 2:
        return None, None
    import numpy as np
    A = np.array(A, float); b = np.array(b, float)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    resid = float(np.sqrt(np.mean((A @ sol - b) ** 2)))
    return (float(sol[0]), float(sol[1])), float(np.sqrt(np.mean(b ** 2)))


def wrap(a):
    return (a + 90.0) % 180.0 - 90.0


def line_of(pts):
    """가장 멀리 떨어진 두 점의 선 각(도). 점 2개 미만이면 None."""
    if len(pts) < 2:
        return None
    best = max(((math.hypot(a[1]-b[1], a[2]-b[2]), a, b)
                for i, a in enumerate(pts) for b in pts[i+1:]), key=lambda t: t[0])
    _, a, b = best
    return math.degrees(math.atan2(b[2]-a[2], b[1]-a[1])) % 180.0


def yaw_meas(station, row, cur):
    """(현재각 − 골든각) 를 J6 로 지울 수 있는 형태로 잰다.
    픽  : 대상 벽(월드)의 파랑 2점 선 — J6 를 돌리면 화면에서 돈다(감도 DANG_DJ6).
    플레이스: (물린 벽 선) − (밑판 선) 상대각 — 벽 선은 카메라와 한 몸이라 J6 에 안 돌고
              밑판 선만 돌므로 상대각 감도는 -DANG_DJ6. 벽 점만 보면 감도 0 (9/1 실측)."""
    def wall_pts(dset):
        b = sorted([p for p in dset if p[0] == "blue"], key=lambda p: -p[3])[:2]
        return b if len(b) >= 2 else None
    if station == "pick":
        # ★골든의 벽 점 2개와 '짝지어진' 현재 점만 쓴다 — "가장 큰 파랑 2개"를 그때그때 고르면
        #   브래킷 파랑(면적 비슷)이 끼어들어 각이 +6° 로 튄다(9/1 실증).
        for cam in ("cam1", "cam2"):
            rw = wall_pts(row[cam])
            if not rw:
                continue
            pr = match(rw, cur[cam])
            if len(pr) < 2:
                continue
            pr.sort(key=lambda t: t[0][2])          # 기준 점 y 순으로 정렬해 순서 고정
            d = wrap(math.degrees(math.atan2(pr[1][1][2]-pr[0][1][2], pr[1][1][1]-pr[0][1][1]))
                     - math.degrees(math.atan2(pr[1][0][2]-pr[0][0][2], pr[1][0][1]-pr[0][0][1])))
            return d, DANG_DJ6
        return None, None
    # place: 벽(면적 큰 파랑 2) 과 밑판 선(노랑 기둥 + 나머지 점)
    def rel(dset):
        w = wall_pts(dset)
        others = [p for p in dset if p not in (w or [])]
        base = line_of(others)
        if not w or base is None:
            return None
        return wrap(line_of(w) - base)
    r, c = rel(row["cam1"]), rel(cur["cam1"])
    if r is None or c is None:
        return None, None
    return wrap(c - r), -DANG_DJ6


def move_j6(deg):
    j = list(st()["joints"]); j[5] += deg
    post("move", {"joints": [round(v, 4) for v in j], "dry_run": False}); wait()


def wall_pillar(cam, ds, ref_row, golden):
    """플레이스용: (벽점, 기둥점, 기준거리, 현재거리) 목록. 벽=면적 큰 파랑(카메라와 한 몸),
    기둥=cam1 노랑 / cam2 빨강 (사용자 지정)."""
    pk = {"cam1": "yellow", "cam2": "red"}[cam]
    gw = [g for g in golden[cam] if g[0] == "blue"]
    gp = [g for g in golden[cam] if g[0] == pk]
    if len(gw) < 1 or len(gp) < 1:
        return None
    mw = match(gw, ds)
    mp = match(gp, ds)
    if len(mw) < 1 or len(mp) < 1:
        return None
    out = []
    rp, cp = mp[0]
    for rw, cw in mw:
        dref = next((d for g, d in ref_row if g == (rw[1], rw[2])), None)
        if dref is None:
            dref = math.hypot(rp[1]-rw[1], rp[2]-rw[2])
        dcur = math.hypot(cp[1]-cw[1], cp[2]-cw[2])
        out.append(((cw[1], cw[2]), (cp[1], cp[2]), dref, dcur))
    return out


def snap_all():
    return {"cam1": dots("cam1"), "cam2": dots("cam2")}



TOP_ONLY = True      # 9/2 저녁 사용자 지시: 맨 위 단에서 한 번 맞추면 바로 수직 하강(개도 30 vs 벽 4.2 → 여유 충분)
WALL_KIND = "blue"   # 9/2 저녁: 대상 벽의 점 색(main 에서 색 인자로 설정 — 노랑 벽 확장)


FINE_TOL, FINE_OVER = 0.3, 1.3   # 9/3: 픽 마지막 미세 마무리 — 잔차 0.3 목표. 양쪽 다리 ≥1.3(=ESC_MAX): 빨강 랙 자세는 1.0mm 명령을 무시해 되돌기가 빠지며 ±1mm 왕복(13:49 실증)


def fine_finish(row):
    """9/3 아침 실증: 벽이 골든 랙 자리에서 7mm 옮겨져 있으면 1mm 스텝 사다리가 0.83mm 를 남긴 채
    '골든 일치'(허용 1mm) 를 선언 → 파지 편차 1.2~1.35mm(어젯밤 0.05) → 슬롯 뒤끝 0.97mm → 하강 중 벽 밀림.
    place 와 같은 방식(축별, |need|<1mm 면 1mm 더 갔다 1mm 되돌기)으로 잔차를 0.3 이하로 마무리한다."""
    for it in range(3):
        cur = snap_all()
        p1 = match([p for p in row["cam1"] if p[0] == WALL_KIND], cur["cam1"])
        p2 = match([p for p in row["cam2"] if p[0] == WALL_KIND], cur["cam2"])
        if len(p1) + len(p2) < 2:
            print("  미세 마무리: 점 부족 — 건너뜀"); return
        sol, err = solve_xy(p1, p2)
        if sol is None:
            print("  미세 마무리: 해 없음 — 건너뜀"); return
        print(f"  미세 마무리[{it+1}] 잔차 {err:5.1f}px  해 ({sol[0]:+.2f},{sol[1]:+.2f})mm")
        if abs(sol[0]) < FINE_TOL and abs(sol[1]) < FINE_TOL:
            print("  미세 마무리: ✔ 잔차 0.3 이하"); return
        if math.hypot(*sol) > 2.0:
            print("  미세 마무리: 해 2mm 초과(불안정) — 건너뜀"); return
        for ax_i, need in ((0, sol[0]), (1, sol[1])):
            if abs(need) < FINE_TOL:
                continue
            if abs(need) < MIN_STEP:
                over = math.copysign(abs(need) + FINE_OVER, need); back = -math.copysign(FINE_OVER, need)
                move_xy_eff(*((over, 0.0) if ax_i == 0 else (0.0, over))); time.sleep(0.3)
                move_xy_eff(*((back, 0.0) if ax_i == 0 else (0.0, back)))
                print(f"    {'XY'[ax_i]} 미세이동 {need:+.2f} (={over:+.2f} 후 {back:+.2f})")
            else:
                stp = math.copysign(MIN_STEP, need)
                move_xy_eff(*((stp, 0.0) if ax_i == 0 else (0.0, stp)))
                print(f"    {'XY'[ax_i]} 이동 {stp:+.1f} (필요 {need:+.2f})")
            time.sleep(0.3)
    print("  미세 마무리: 3회 후에도 잔차 남음 — 현 상태로 진행")


def run_pick_simple(G):
    """★9/2 사용자 설계 — 픽 단순화.
    · yaw 를 재지 않는다: 파지 yaw 편차는 플레이스 파지검증(1.5° 게이트)이 다시 본다.
      (yaw 서보는 9/1~9/2 이틀간 과보정·노이즈 추종으로 픽 정지의 최다 원인이었다)
    · 4단 중 '한 단이라도' 골든에 맞으면 충분 — 마지막까지 내려가 본 뒤 수직 하강해 파지
      위치로 간다. 수직 하강은 실증상 정확하고(0.1~0.4mm), 그리퍼 개방 30 대비 벽 4~5mm 의
      여유가 커서 mm 급 오차는 파지에 지장이 없다. 실제 파지 편차는 플레이스가 실측해 보상한다.
    · 한 단이 실패해도 정지하지 않는다: 그 단에서 움직인 만큼 되돌려(이전 정렬 보존) 다음 단으로.
      전 단 실패일 때만 정지(파지 금지)."""
    rows = [r for r in G["rows"] if r["z"] > G["golden_z"] + 1]
    post("speed", {"value": PICK_SPEED, "dry_run": False}); time.sleep(0.25)
    any_ok = False
    for hi, row in enumerate(rows):
        z = row["z"]
        move_z(z)
        time.sleep(0.5)
        moved = 0.0
        net = [0.0, 0.0]
        ok = False
        budget = 25 if hi == 0 else 15        # 첫 단은 벽 이동 추종용으로 넉넉히
        for it in range(budget):
            cur = snap_all()
            p1 = match([p for p in row["cam1"] if p[0] == WALL_KIND], cur["cam1"])
            p2 = match([p for p in row["cam2"] if p[0] == WALL_KIND], cur["cam2"])
            if len(p1) + len(p2) < 2:
                print(f"  z{z} [{it+1}] 대상 점 부족(손목캠 {len(p1)}·새캠 {len(p2)}) — 이 단 포기")
                break
            sol, err = solve_xy(p1, p2)
            if sol is None:
                print(f"  z{z}: 해 없음 — 이 단 포기")
                break
            mag = math.hypot(*sol)
            print(f"  z{z} [{it+1}] 잔차 {err:5.1f}px  해 ({sol[0]:+.2f},{sol[1]:+.2f})mm  누적 {moved:.1f}mm")
            if err < TOL_PX or (mag < MAG_OK and err < PX_CEIL):
                ok = True
                break
            if err < 6.0 and mag > 3.0:
                print(f"  z{z}: 해 불안정 — 이 단 포기")
                break
            dX, dY = clamp_step(sol, far_ok=(hi == 0))
            if moved + math.hypot(dX, dY) > CUM_CAP.get(hi, 2.0) + (18.0 if hi == 0 else 0.0):
                print(f"  z{z}: 누적 이동 상한 — 이 단 포기")
                break
            ax, ay = move_xy_eff(dX, dY)
            net[0] += ax; net[1] += ay
            moved += math.hypot(ax, ay)
            time.sleep(0.25)
        if ok:
            print(f"  z{z}: ✔ 골든 일치")
            any_ok = True
            if TOP_ONLY:
                fine_finish(row)     # ★9/3: 1mm 사다리가 남긴 잔차(≤1mm)를 '1mm 더 갔다 되돌기'로 0.3 이하까지
                print("  (위에서 한 번 보정 → 바로 수직 하강)")
                break
        else:
            print(f"  z{z}: 미일치 — 다음 단에서 재시도")
            if math.hypot(*net) > 0.2:
                # 이 단의 미완 이동을 되돌려, 앞서 맞춘 정렬을 보존한다
                move_xy_eff(-net[0], -net[1])
                print(f"  z{z}: 이 단 이동 {math.hypot(*net):.1f}mm 원복")
    if not any_ok:
        print("4단 전부 미일치 — ★파지 금지. 정지")
        return
    move_z(G["golden_z"])
    print("완료(골든 일치):", [round(v, 2) for v in stable()[:3]],
          "— 그리퍼 무변경. 파지/삽입은 사용자 명령으로")


def main():
    if len(sys.argv) < 3:
        sys.exit("사용법: golden.py teach|run pick|place [색]")
    mode, station = sys.argv[1], sys.argv[2]
    ck = sys.argv[3] if len(sys.argv) > 3 else "blue"
    key = f"golden_{station}"
    cal = json.load(open(CAL))
    ref = cal["refs"][ck]
    global WALL_KIND
    WALL_KIND = ref.get("dot_color", ck)             # 벽 점 색(red_s 는 red)

    if mode == "teach":
        # ★골든 확정 잠금(사용자 지시 9/1): 확정된 골든은 --force 없이 재기록 금지
        if station == "pick" and ref.get("golden_locked") and ref.get(key) and "--force" not in sys.argv:
            sys.exit("골든이 사용자 확정으로 잠겨 있음 — 재기록하려면 사용자 지시 + --force")
        gz = stable()[2]
        rows = []
        post("speed", {"value": 1, "dry_run": False}); time.sleep(0.3)
        g0 = snap_all()
        rows.append({"z": round(gz, 1), "cam1": g0["cam1"], "cam2": g0["cam2"]})
        print(f"골든 z{gz:.1f}: 손목캠 {len(g0['cam1'])}점 · 새캠 {len(g0['cam2'])}점")
        for off in OFFS:
            move_z(gz + off)
            time.sleep(0.7)
            s_ = snap_all()
            rows.append({"z": round(gz + off, 1), "cam1": s_["cam1"], "cam2": s_["cam2"]})
            print(f"  z{gz+off:.1f}: 손목캠 {len(s_['cam1'])}점 {[(p[0][:1],round(p[1]),round(p[2]),p[3]) for p in s_['cam1']]}")
            print(f"           새캠 {len(s_['cam2'])}점 {[(p[0][:1],round(p[1]),round(p[2]),p[3]) for p in s_['cam2']]}")
        rows.sort(key=lambda r: -r["z"])
        tcp = stable()
        ref[key] = {"made": time.strftime("%Y-%m-%d %H:%M"), "golden_z": round(gz, 1),
                    "golden_tcp": [round(v, 2) for v in stable()], "rows": rows}
        cal["refs"][ck] = ref
        json.dump(cal, open(CAL, "w"), indent=1, ensure_ascii=False)
        print(f"저장: refs.{ck}.{key} (골든 z{gz:.1f} + 4단). 팔은 최고점 대기")
        return

    if mode == "run":
        G = ref.get(key)
        if not G:
            sys.exit(f"{key} 없음 — teach 먼저")
        if station == "pick":
            run_pick_simple(G)          # 9/2 사용자 설계: yaw 없음·한 단 일치면 수직 하강
            return
        rows = [r for r in G["rows"] if r["z"] > G["golden_z"] + 1]     # 4단(골든 제외)
        golden_row = next(r for r in G["rows"] if abs(r["z"] - G["golden_z"]) < 1)
        post("speed", {"value": PICK_SPEED, "dry_run": False}); time.sleep(0.25)
        # ★샘플 축소·정착대기 축소·정렬속도 상향은 9/1 밤 실패로 철회했다:
        #   잔차가 16.8px 로 튀고 해가 +1.77↔-1.68 로 진동(측정이 정착 전에 읽힘).
        #   측정 품질에 닿는 것은 속도 대상이 아니다.
        for hi, row in enumerate(rows):
            z = row["z"]
            move_z(z)
            time.sleep(0.6)
            moved = 0.0
            turned = 0.0
            ok = False
            prev_yaw = None
            for it in range(MAX_IT):
                cur = snap_all()
                dyaw, sens = yaw_meas(station, row, cur)
                if dyaw is not None and abs(dyaw) >= YAW_TOL:
                    # ★노이즈 프레임 방어 (9/2): 이상 프레임(~7%)의 yaw 오독 한 방을 쫓아 돌면
                    #   회전 오차가 누적돼 XY 로 못 지우는 잔차가 남는다(z379 실증: -0.48 보정 후
                    #   -1.43 으로 악화 → 순누적 -0.63° → 잔차 9px 고착). 대책 두 겹:
                    #   ⑴ 새 스냅샷으로 재측정해 0.3° 이내로 일치할 때만 보정(불일치 = 노이즈, skip)
                    #   ⑵ 직전 보정 후 편차가 되레 0.5° 이상 커졌으면 응답 이상 — 정지
                    if prev_yaw is not None and abs(dyaw) > abs(prev_yaw) + 0.5:
                        print(f"  z{z}: yaw 응답 이상(보정 후 {prev_yaw:+.2f}→{dyaw:+.2f}°) — 정지"); return
                    d2, _ = yaw_meas(station, row, snap_all())
                    if d2 is None or abs(d2 - dyaw) > 0.3:
                        print(f"  z{z} [{it+1}] yaw 재확인 불일치({dyaw:+.2f} vs {d2 if d2 is None else format(d2, '+.2f')}°) — 노이즈로 보고 건너뜀")
                        prev_yaw = None
                        time.sleep(0.4)
                        continue
                    dyaw = (dyaw + d2) / 2.0
                    # Δ각 = sens × ΔJ6 이고 원하는 Δ각 = −dyaw ⇒ ΔJ6 = −dyaw/sens
                    # ★부호 실수(9/1): dyaw/sens 로 보내 편차가 +0.44→+3.12° 로 발산, 4° 게이트가 정지시킴
                    corr = -dyaw / sens
                    step = math.copysign(min(J6_STEP_MAX, max(J6_STEP_MIN, abs(corr))), corr)
                    if turned + abs(step) > 4.0:
                        print(f"  z{z}: yaw 누적 회전 상한 4° 도달 — 정지"); return
                    print(f"  z{z} [{it+1}] yaw 편차 {dyaw:+.2f}° → J6 {step:+.2f}°")
                    move_j6(step)
                    turned += abs(step)
                    prev_yaw = dyaw
                    time.sleep(0.25)
                    continue
                if station == "pick":
                    # 대상(벽 파랑)만 골든 픽셀로
                    r1 = [p for p in row["cam1"] if p[0] == "blue"]
                    r2 = [p for p in row["cam2"] if p[0] == "blue"]
                    p1, p2 = match(r1, cur["cam1"]), match(r2, cur["cam2"])
                    if len(p1) + len(p2) < 2:
                        print(f"  z{z}: 대상 점 부족(손목캠 {len(p1)}·새캠 {len(p2)}) — 정지"); return
                    sol, err = solve_xy(p1, p2)
                else:
                    dd = {}
                    for cam in ("cam1", "cam2"):
                        dd[cam] = wall_pillar(cam, cur[cam], [], {"cam1": row["cam1"], "cam2": row["cam2"]})
                    n_d = sum(len(v) for v in dd.values() if v)
                    if n_d < 2:
                        print(f"  z{z}: 벽↔기둥 거리 부족({n_d}) — 정지"); return
                    sol, err = solve_xy([], [], want_dist=dd)
                if sol is None:
                    print(f"  z{z}: 해 없음 — 정지"); return
                mag = math.hypot(*sol)
                px_err = err
                ys = f"{dyaw:+.2f}°" if dyaw is not None else "―"
                print(f"  z{z} [{it+1}] 잔차 {px_err:5.1f}px  yaw {ys}  해 ({sol[0]:+.2f},{sol[1]:+.2f})mm  누적 {moved:.1f}mm")
                # ★수렴 판정 (9/1 개정): 픽셀 잔차만 보면 J6 회전이 남긴 회전 성분 때문에
                #   영영 못 넘는다 — 그 성분은 XY 평행이동으로 지울 수 없기 때문(실증: 실제
                #   이동 오차 0.17mm 인데 잔차 3.1px 로 고정, 상한까지 헛돌다 정지).
                #   → 픽셀이 깨끗하거나, '실제 이동 오차'가 충분히 작으면 수렴으로 본다.
                if (px_err < TOL_PX or (mag < MAG_OK and px_err < PX_CEIL)) \
                        and (dyaw is None or abs(dyaw) < YAW_TOL):
                    ok = True
                    break
                # ★검증 게이트: 잔차는 작은데 해가 크면 불안정 — 움직이지 않는다
                if px_err < 6.0 and mag > 3.0:
                    print(f"  z{z}: 해 불안정(잔차 {px_err:.1f}px vs 해 {mag:.1f}mm) — 정지"); return
                dX, dY = clamp_step(sol)
                if moved + math.hypot(dX, dY) > CUM_CAP.get(hi, 2.0):
                    print(f"  z{z}: 누적 이동 상한 {CUM_CAP.get(hi,2.0)}mm 도달 — 정지"); return
                ax, ay = move_xy_eff(dX, dY)      # ★실제로 움직인 양만 누적
                moved += math.hypot(ax, ay)
                time.sleep(0.45)
            if not ok:
                print(f"  z{z}: {MAX_IT}회 내 미수렴 — ★다음 단계 진행 금지. 정지"); return
            print(f"  z{z}: ✔ 골든 일치")
        print(f"  전 단계 골든 일치 → 수직 하강 z{G['golden_z']}")
        move_z(G["golden_z"])
        # ★마무리 정렬(사용자 선택 9/1): 마지막 단→골든 z 수직 하강 중 벽 기울기만큼 벌어질 수 있다
        #   (실측 10.5px≈3mm). 골든 z 의 골든 사진과 직접 대조해 3px 이내까지 다듬고 끝낸다.
        moved = 0.0
        fin_ok = False
        resp = None          # ★해/실이동 배율을 '그 자리에서' 측정한다(아래 설명)
        prev = None          # (직전 sol, 직전 실이동 벡터)
        for it in range(MAX_IT):
            cur = snap_all()
            if station == "pick":
                p1 = match([p for p in golden_row["cam1"] if p[0] == "blue" and p[3] >= 150], cur["cam1"])
                p2 = match([p for p in golden_row["cam2"] if p[0] == "blue" and p[3] >= 150], cur["cam2"])
                if len(p1) + len(p2) < 2:
                    print("  골든z: 대상 점 부족 — 정지(그리퍼 안 건드림)"); return
                sol, err = solve_xy(p1, p2)
            else:
                dd = {c: wall_pillar(c, cur[c], [], {"cam1": golden_row["cam1"],
                                                     "cam2": golden_row["cam2"]}) for c in ("cam1", "cam2")}
                if sum(len(v) for v in dd.values() if v) < 2:
                    print("  골든z: 거리 부족 — 정지"); return
                sol, err = solve_xy([], [], want_dist=dd)
            # ★배율 자가측정 (9/2): 골든 z 는 카메라가 벽에 바짝 붙어 축척이 커서, 먼 거리 기준
            #   행렬(M)로 푼 해가 실이동의 2.8배로 부풀려 나온다(실측: 1.0mm 이동 → 해 2.83 변화).
            #   그대로 믿고 1mm 를 움직이면 실제 0.34mm 오차를 지나쳐 반대편으로 넘어가 진동한다
            #   (9/2 실패: -1.02 ↔ +1.81 왕복 후 누적 상한 정지). 높이마다 값이 다르므로 상수로
            #   박지 않고, 직전 이동이 해를 얼마나 바꿨는지로 그 자리에서 잰다.
            if prev is not None:
                psol, pmv = prev
                mv = math.hypot(*pmv)
                dchg = math.hypot(sol[0] - psol[0], sol[1] - psol[1])
                if mv > 0.15 and dchg > 0.1:
                    r = dchg / mv
                    if 0.5 <= r <= 6.0:
                        resp = r if resp is None else 0.5 * resp + 0.5 * r
                        print(f"     (배율 자가측정: 해 {dchg:.2f} / 실이동 {mv:.2f} = {r:.2f}배"
                              f"{'' if resp == r else f' → 평균 {resp:.2f}'})")
            k = resp or 1.0
            true_mag = math.hypot(*sol) / k
            print(f"  골든z [{it+1}] 잔차 {err:5.1f}px  해 ({sol[0]:+.2f},{sol[1]:+.2f})"
                  f"{f' → 실 {true_mag:.2f}mm' if resp else ''}  누적 {moved:.1f}mm")
            if err < TOL_PX or (true_mag < MAG_OK and err < PX_CEIL):
                fin_ok = True
                break
            if err < 6.0 and true_mag > 3.0:
                print("  골든z: 해 불안정 — 정지"); return
            dX, dY = clamp_step((sol[0] / k, sol[1] / k))
            if moved + math.hypot(dX, dY) > 3.0:
                print("  골든z: 누적 3mm 상한 — 정지"); return
            ax, ay = move_xy_eff(dX, dY)
            prev = (sol, (ax, ay))
            moved += math.hypot(ax, ay)
            time.sleep(0.45)
        if not fin_ok:
            print("  골든z: 미수렴 — 정지(그리퍼 안 건드림)"); return
        print("완료(골든 일치):", [round(v, 2) for v in stable()[:3]],
              "— 그리퍼 무변경. 파지/삽입은 사용자 명령으로")
        return

    sys.exit("mode 는 teach 또는 run")


if __name__ == "__main__":
    main()
