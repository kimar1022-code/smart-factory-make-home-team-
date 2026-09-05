#!/usr/bin/env python3
"""베이스 pose 측정 + 반복정밀도 점검 — 9/4 (내일 첫 게이트).

손목캠에서 기둥 꼭대기 4점(흰 점)을 잡아 STL 모델(200×130 사각형)에 2D 강체 맞춤 →
베이스 (x, y, yaw). N회 반복해 σ 를 낸다.

★mm/px 자가 보정: 기둥 중심 간격이 200/130 mm 로 알려져 있으므로, 검출된 픽셀 간격에서
  축척을 직접 구한다 → 카메라 마운트가 바뀌어도 별도 캘리브레이션이 필요 없다.

게이트(긴벽 허용 0.29° 의 1/3 기준):
    yaw σ ≤ 0.10°  ·  중심 σ ≤ 0.2 mm  →  통과(형상 검출만으로 삽입 가능)
    넘으면 → 밑판 바닥 ArUco 2장 보강 판단(사용자 승인 필요)

  python3 base_pose_check.py [N] [--save out.jpg] [--yaw-hint 0] [--json]

로봇을 움직이지 않는다(현재 자세에서 카메라만 반복 촬영). base_view 자세에서 돌릴 것.
"""
import sys, json, math, statistics as st
import numpy as np

sys.path.insert(0, "/home/ar/bf2_console/tools")
import base_depth_corner as B
import house_geometry as HG
import pillar_dots as PD

MODEL = HG.COLUMN_CENTERS          # ((5,5),(205,5),(205,135),(5,135)) mm

# ★9/5 실물 배치: 기둥 꼭대기가 색점(파랑2·노랑1·빨강1)이다 → 색으로 모서리가 확정되므로
#   조합 탐색도 180° 뒤집힘 처리도 필요 없다. 흰 점 경로는 폴백으로만 남긴다.
#   실측 변 길이: 노랑–빨강 493.3px, 파랑–파랑 493.9px(둘 다 긴 변 200mm)
#                 노랑–파랑 313.5px, 빨강–파랑 319.6px(짧은 변 130mm)
#   → 노랑과 빨강이 한 긴 변, 파랑 둘이 반대 긴 변.
#   ⚠아래 대응은 '자기일관적'이지만 어느 물리 모서리가 원점인지는 골든 재티칭에서 확정할 것.
COLOR_MAP = {"yellow": (5.0, 5.0), "red": (205.0, 5.0)}      # 같은 긴 변
BLUE_FOR = {"yellow": (5.0, 135.0), "red": (205.0, 135.0)}   # 각 색의 짧은 변 건너편
GATE_YAW_SIGMA = 0.10              # °
GATE_POS_SIGMA = 0.20              # mm
CAND_MAX = 7                       # ★흰 점 후보 상위 이만큼에서 조합 탐색(벽=가짜 점 배제)
EDGE_MARGIN_PX = 18                # ★프레임 가장자리 이 안쪽의 점은 '잘린 점'이라 무게중심이 안으로 끌린다.
                                   #   9/5 실측: y=5.4px 점 때문에 변이 130→118.3mm 로 나와 rms 5.3mm.
                                   #   반복정밀도는 멀쩡해 보이므로(매번 같게 잘림) 반드시 따로 걸러야 한다.
RMS_OK_MM = 2.0                    # 이 이하면 '모양 맞음' — 더 볼 것 없이 채택
RMS_REJECT_MM = 6.0                # 이 이상이면 기둥이 아니라고 보고 실패 처리


def _order_ccw(pts):
    """4점을 무게중심 기준 반시계로 정렬."""
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def _model_edges():
    """모델의 변 길이(순서대로) — 대응 후보 검증용."""
    n = len(MODEL)
    return [math.dist(MODEL[i], MODEL[(i + 1) % n]) for i in range(n)]


def _fit3(px_pts, yaw_hint=0.0):
    """★기둥 3개만 보일 때(벽이 하나를 가림) — 모델 4점 중 어느 하나를 뺐는지 모르므로
    4가지 '뺀 경우' × 순환대응을 모두 시도해 rms 최소를 고른다. 4점보다 약하지만 측정은 유지된다."""
    ring = _order_ccw(px_pts)
    all_c = []
    for drop in range(4):                       # 모델에서 뺀 꼭짓점
        sub = [MODEL[i] for i in range(4) if i != drop]
        me = [math.dist(sub[i], sub[(i + 1) % 3]) for i in range(3)]
        for shift in range(3):
            cand = ring[shift:] + ring[:shift]
            pe = [math.dist(cand[i], cand[(i + 1) % 3]) for i in range(3)]
            num = sum(m * p for m, p in zip(me, pe)); den = sum(p * p for p in pe)
            if den <= 0:
                continue
            s = num / den
            pose, rms = HG.fit_base_pose([(p[0] * s, p[1] * s) for p in cand], tuple(sub))
            all_c.append((rms, pose, s, cand))
    if not all_c:
        return None
    # ★직사각형에서 한 꼭짓점을 빼면 남은 삼각형은 어느 것을 뺐든 **합동**이다
    #   → 맞춤 오차(rms)가 사실상 동률인 후보가 여럿 나오고, 그 중 하나는 180° 뒤집힌 자세다.
    #   rms 로는 원리적으로 못 가르므로 4점 때와 같이 동률 묶음에서 yaw_hint 에 가까운 것을 택한다.
    best_rms = min(c[0] for c in all_c)
    tie = [c for c in all_c if c[0] <= best_rms * 1.05 + 0.01]
    rms, pose, s, cand = min(tie, key=lambda c: abs(HG.wrap_deg(c[1].yaw_deg - yaw_hint)))
    return pose, rms, s, cand


def _fit_with_correspondence(px_pts, yaw_hint=0.0):
    """검출 4점(px)을 모델에 대응시켜 축척·pose 추정.
    4가지 순환 대응을 모두 시도해 변 길이비가 맞는 것(rms 최소)을 고르고,
    직사각형의 180° 모호성은 yaw_hint 에 가까운 쪽으로 푼다.
    반환 (pose, rms_mm, mm_per_px, ordered_px)"""
    ring = _order_ccw(px_pts)
    me = _model_edges()                       # [200,130,200,130] 순
    cands = []
    for shift in range(4):
        cand = ring[shift:] + ring[:shift]
        # 이 대응에서의 변 길이(px)
        pe = [math.dist(cand[i], cand[(i + 1) % 4]) for i in range(4)]
        # 변 길이비가 모델과 맞는지 → 축척 추정(길이비 가중 최소제곱)
        num = sum(m * p for m, p in zip(me, pe))
        den = sum(p * p for p in pe)
        if den <= 0:
            continue
        s = num / den                          # mm per px
        # px → mm (로컬, 원점은 임의) 후 강체 맞춤
        meas_mm = [(p[0] * s, p[1] * s) for p in cand]
        pose, rms = HG.fit_base_pose(meas_mm, MODEL)
        cands.append((rms, pose, s, cand))
    if not cands:
        return None
    # ★직사각형은 180° 돌려도 같은 모양 → 정대응과 180° 뒤집힘이 '같은 rms'를 준다(부동소수 미세차뿐).
    #   따라서 rms 최소만으로 고르면 안 되고, rms 가 사실상 동률인 후보들 중 yaw_hint 에 가까운 것을 택한다.
    #   (전제: 베이스는 대략 알려진 방향으로 놓인다. 첫 회차 hint 는 0 또는 --yaw-hint, 이후는 직전 측정값.)
    best_rms = min(c[0] for c in cands)
    tie = [c for c in cands if c[0] <= best_rms * 1.05 + 0.01]
    rms, pose, s, cand = min(tie, key=lambda c: abs(HG.wrap_deg(c[1].yaw_deg - yaw_hint)))
    return pose, rms, s, cand


def _best_subsets(pool, yaw_hint):
    """후보 흰 점들에서 '모양이 맞는' 부분집합을 고른다.
    ★4점은 '모양이 맞을 때만' 3점보다 우선한다 — 진짜 기둥이 가려지면 최선의 4점 조합조차
      반드시 가짜를 포함해 rms 가 폭증하는데, k 를 무조건 우선하면 그 엉터리를 고른다(실측 확인:
      rms 60mm·yaw 36° 오차). 그래서 4점 rms 가 나쁘면 3점과 rms 로 정면 비교한다.
    반환 (fit결과, 뽑힌 인덱스, k) 또는 None"""
    from itertools import combinations
    picks = {}
    for k in (4, 3):
        if len(pool) < k:
            continue
        bk = None
        for comb in combinations(range(len(pool)), k):
            pts_c = [(pool[i][0], pool[i][1]) for i in comb]
            rr = _fit_with_correspondence(pts_c, yaw_hint) if k == 4 else _fit3(pts_c, yaw_hint)
            if rr is None:
                continue
            if bk is None or rr[1] < bk[0][1]:
                bk = (rr, comb)
        if bk:
            picks[k] = bk
        if k == 4 and bk and bk[0][1] <= RMS_OK_MM:
            break                                   # 모양 맞는 4점 → 3점 탐색 불필요
    b4, b3 = picks.get(4), picks.get(3)
    if b4 and b4[0][1] <= RMS_OK_MM:
        return b4[0], b4[1], 4
    if b3 and (b4 is None or b3[0][1] < b4[0][1]):
        return b3[0], b3[1], 3
    if b4:
        return b4[0], b4[1], 4
    return None


def measure_color(yaw_hint=0.0):
    """★기둥 색점으로 측정(9/5 본선). 색이 모서리를 확정하므로 대응 탐색이 필요 없다.
    반환 dict 또는 (None, 이유)."""
    img, grid = B.grab_pair()
    if img is None:
        return None, "프레임 없음"
    try:
        m, why = B.detect_rect(img, grid)
    except Exception as e:
        m, why = None, f"detect_rect 오류 {e}"
    rect = None
    if m:
        rect = PD.plate_rect(m)
    if rect is None:
        return None, f"밑판 사각형 검출 실패({why})"
    pts, why2 = PD.four_corners(img, rect)
    if not pts or len(pts) < 3:
        return None, why2 or "기둥 색점 검출 실패"
    # ★3점 폴백 지원(9/5): five-tuple (x,y,area,color,model_idx) → 있는 꼭짓점만으로 맞춘다
    idx = [int(p[4]) for p in pts]
    meas = [(p[0], p[1]) for p in pts]
    model = [PD.CORNER_MODEL[i] for i in idx]
    # 축척: 대응이 확정됐으므로 변 길이비 최소제곱으로 바로 구한다
    me = [math.dist(model[i], model[(i + 1) % 4]) for i in range(4)]
    pe = [math.dist(meas[i], meas[(i + 1) % 4]) for i in range(4)]
    den = sum(v * v for v in pe)
    if den <= 0:
        return None, "변 길이 0"
    s = sum(a * b for a, b in zip(me, pe)) / den
    pose, rms = HG.fit_base_pose([(p[0] * s, p[1] * s) for p in meas], tuple(model))
    return {"pose": pose, "rms_mm": rms, "mm_per_px": s, "pts": meas,
            "n_pillars": len(pts), "n_white_seen": len(pts), "n_clipped": 0,
            "colors": [p[3] for p in pts], "areas": [p[2] for p in pts],
            "warn": why2, "plate_yaw": (m["yaw"] if m else None), "img": img}, None


def measure_once(yaw_hint=0.0):
    """한 번 측정. 반환 dict 또는 (None, 이유)."""
    img, grid = B.grab_pair()
    if img is None:
        return None, "프레임 없음"
    try:
        m, why = B.detect_rect(img, grid)
    except Exception as e:
        m, why = None, f"detect_rect 오류 {e}"
    # ★detect_rect 반환 키는 box / corners 이며 'rect' 는 없다(9/5 라이브 실행에서 KeyError 로 확인).
    #   서브픽셀 정제 corners 우선, 없으면 minAreaRect box.
    rect = None
    if m:
        rect = PD.plate_rect(m)
    try:
        d = B.base_dots(img, rect)
    except Exception as e:
        return None, f"base_dots 오류 {e}"
    ws_all = sorted(d.get("white") or [], key=lambda p: -p[2])
    Himg, Wimg = img.shape[:2]
    ws = [p for p in ws_all
          if EDGE_MARGIN_PX <= p[0] <= Wimg - EDGE_MARGIN_PX
          and EDGE_MARGIN_PX <= p[1] <= Himg - EDGE_MARGIN_PX]
    n_clip = len(ws_all) - len(ws)
    n_white = len(ws)
    if n_white < 3:
        return None, (f"기둥 꼭대기 흰 점 {n_white}개(3 이상 필요)"
                      + (f" · 프레임 가장자리에 걸려 버린 점 {n_clip}개 → 밑판이 화면에 다 안 들어옴(자세 조정 필요)" if n_clip else "")
                      + (' · ' + why if why else ''))
    # ★★벽이 꽂히면 ①벽 윗변이 밝게 잡혀 '가짜 기둥'이 되고 ②진짜 기둥 하나가 가려질 수 있다.
    #   면적 큰 순서로 4개를 집으면 가짜가 섞인다 → **모양이 맞는 조합**(모델 맞춤 rms 최소)을 고른다.
    #   후보는 면적 상위 CAND_MAX 개, 4점 조합 우선, 없으면 3점으로 내려간다.
    pool = ws[:CAND_MAX]
    best_of = _best_subsets(pool, yaw_hint)
    if not best_of:
        return None, f"대응 실패(후보 {min(len(ws), CAND_MAX)}점)"
    r, comb, k = best_of
    use = [pool[i] for i in comb]
    if r[1] > RMS_REJECT_MM:
        return None, (f"모양 불일치 rms {r[1]:.2f}mm > {RMS_REJECT_MM}mm "
                      f"(후보 {len(pool)}점 중 {k}점 조합) — 벽/반사가 기둥으로 오인된 듯")
    pose, rms, s, ordered = r
    return {"pose": pose, "rms_mm": rms, "mm_per_px": s, "pts": ordered,
            "n_pillars": k, "n_white_seen": n_white, "n_clipped": n_clip,
            "areas": [round(p[2]) for p in use],
            "plate_yaw": (m["yaw"] if m else None), "img": img}, None


def main():
    n = 10
    for a in sys.argv[1:]:
        if a.isdigit():
            n = int(a)
    hint = 0.0
    if "--yaw-hint" in sys.argv:
        hint = float(sys.argv[sys.argv.index("--yaw-hint") + 1])

    rows, fails = [], []
    last_img = None
    for i in range(n):
        r, why = measure_color(hint)
        if r is None and "--white" in sys.argv:
            r, why = measure_once(hint)          # 폴백: 옛 흰 점 배치
        if r is None:
            fails.append(why); print(f"  [{i+1}/{n}] 실패: {why}", flush=True); continue
        rows.append(r); last_img = r["img"]; hint = r["pose"].yaw_deg   # 다음 회차 힌트
        print(f"  [{i+1}/{n}] x {r['pose'].x:8.2f}  y {r['pose'].y:8.2f}  yaw {r['pose'].yaw_deg:+7.3f}°"
              f"  rms {r['rms_mm']:.3f}mm  축척 {r['mm_per_px']:.4f}mm/px", flush=True)

    if len(rows) < 2:
        print(f"\n❌ 측정 {len(rows)}회 — 판정 불가. 실패 사유: {set(fails)}")
        return

    xs = [r["pose"].x for r in rows]; ys = [r["pose"].y for r in rows]
    yw = [r["pose"].yaw_deg for r in rows]; sc = [r["mm_per_px"] for r in rows]
    rm = [r["rms_mm"] for r in rows]
    sx, sy, sw = st.pstdev(xs), st.pstdev(ys), st.pstdev(yw)
    print("\n━━ 반복정밀도 (n=%d, 실패 %d) ━━" % (len(rows), len(fails)))
    print(f"  x   평균 {st.mean(xs):8.2f} mm   σ {sx:.3f}")
    print(f"  y   평균 {st.mean(ys):8.2f} mm   σ {sy:.3f}")
    print(f"  yaw 평균 {st.mean(yw):+7.3f}°    σ {sw:.4f}")
    print(f"  축척 평균 {st.mean(sc):.4f} mm/px  (σ {st.pstdev(sc):.4f})")
    print(f"  모델 맞춤 rms 평균 {st.mean(rm):.3f} mm  (최대 {max(rm):.3f})")

    pos_sigma = max(sx, sy)
    ok_yaw = sw <= GATE_YAW_SIGMA
    ok_pos = pos_sigma <= GATE_POS_SIGMA
    print("\n━━ 게이트 판정 ━━")
    print(f"  yaw σ {sw:.4f}° ≤ {GATE_YAW_SIGMA}°  → {'✅ 통과' if ok_yaw else '❌ 미달'}")
    print(f"  위치 σ {pos_sigma:.3f}mm ≤ {GATE_POS_SIGMA}mm → {'✅ 통과' if ok_pos else '❌ 미달'}")
    if ok_yaw and ok_pos:
        print("  → 형상 검출만으로 삽입 진행 가능(긴벽 허용 0.29°의 1/3 이내).")
    else:
        print("  → 밑판 바닥 ArUco 2장 보강 검토(사용자 승인 필요). 또는 조명·ROI 재점검.")

    if "--save" in sys.argv and last_img is not None:
        import cv2
        out = last_img.copy()
        for i, p in enumerate(rows[-1]["pts"]):
            cv2.circle(out, (int(p[0]), int(p[1])), 8, (0, 255, 255), 2)
            cv2.putText(out, str(i), (int(p[0]) + 10, int(p[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        po = rows[-1]["pose"]
        cv2.putText(out, f"x{po.x:.1f} y{po.y:.1f} yaw{po.yaw_deg:+.2f} sig{sw:.3f}",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imwrite(sys.argv[sys.argv.index("--save") + 1], out)
        print("저장:", sys.argv[sys.argv.index("--save") + 1])

    # ★두 조건(빈 베이스 / 벽 꽂힌 베이스) 비교용 기록
    if "--label" in sys.argv:
        lab = sys.argv[sys.argv.index("--label") + 1]
        import os, time as _t
        LOGF = "/home/ar/bf2_console/logs/base_pose_runs.json"
        try:
            db = json.load(open(LOGF)) if os.path.exists(LOGF) else {}
        except Exception:
            db = {}
        db[lab] = {"made": _t.strftime("%Y-%m-%d %H:%M"), "n": len(rows),
                   "mean_x": st.mean(xs), "mean_y": st.mean(ys), "mean_yaw": st.mean(yw),
                   "sigma_x": sx, "sigma_y": sy, "sigma_yaw": sw,
                   "mean_mm_per_px": st.mean(sc), "mean_rms": st.mean(rm),
                   "pillars": [r.get("n_pillars") for r in rows],
                   "white_seen": [r.get("n_white_seen") for r in rows]}
        json.dump(db, open(LOGF, "w"), ensure_ascii=False, indent=1)
        print(f"\n기록 저장: {LOGF}  [{lab}]")
        if len(db) >= 2:
            print("━━ 저장된 조건 비교 ━━")
            for k, v in db.items():
                pil = v.get("pillars") or []
                print(f"  {k:14} yawσ {v['sigma_yaw']:.4f}°  위치σ {max(v['sigma_x'],v['sigma_y']):.3f}mm  "
                      f"yaw평균 {v['mean_yaw']:+.3f}°  기둥 {min(pil) if pil else '?'}~{max(pil) if pil else '?'}점  n={v['n']}")
            ks = list(db)
            if len(ks) >= 2:
                a, b = db[ks[-2]], db[ks[-1]]
                dyaw = HG.wrap_deg(b["mean_yaw"] - a["mean_yaw"])
                dxy = math.dist((a["mean_x"], a["mean_y"]), (b["mean_x"], b["mean_y"]))
                print(f"  → 두 조건 차이: yaw {dyaw:+.3f}°, 중심 {dxy:.2f}mm")
                print("     (베이스를 안 움직였다면 이 차이는 '벽 가림이 만든 측정 편향'이다. "
                      "yaw 0.1°·중심 0.3mm 넘으면 벽 꽂힌 상태 보정이 필요)")

    if "--json" in sys.argv:
        print(json.dumps({"n": len(rows), "sigma_yaw_deg": sw, "sigma_pos_mm": pos_sigma,
                          "mean_mm_per_px": st.mean(sc), "pass": bool(ok_yaw and ok_pos)}))


if __name__ == "__main__":
    main()
