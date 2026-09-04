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

MODEL = HG.COLUMN_CENTERS          # ((5,5),(205,5),(205,135),(5,135)) mm
GATE_YAW_SIGMA = 0.10              # °
GATE_POS_SIGMA = 0.20              # mm


def _order_ccw(pts):
    """4점을 무게중심 기준 반시계로 정렬."""
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def _model_edges():
    """모델의 변 길이(순서대로) — 대응 후보 검증용."""
    n = len(MODEL)
    return [math.dist(MODEL[i], MODEL[(i + 1) % n]) for i in range(n)]


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


def measure_once(yaw_hint=0.0):
    """한 번 측정. 반환 dict 또는 (None, 이유)."""
    img, grid = B.grab_pair()
    if img is None:
        return None, "프레임 없음"
    try:
        m, why = B.detect_rect(img, grid)
    except Exception as e:
        m, why = None, f"detect_rect 오류 {e}"
    rect = m["rect"] if m else None
    try:
        d = B.base_dots(img, rect)
    except Exception as e:
        return None, f"base_dots 오류 {e}"
    white = [(p[0], p[1]) for p in (d.get("white") or [])]
    if len(white) < 4:
        return None, f"기둥 꼭대기 흰 점 {len(white)}개(4 필요){' · ' + why if why else ''}"
    white = sorted(d["white"], key=lambda p: -p[2])[:4]      # 큰 것 4개
    white = [(p[0], p[1]) for p in white]
    r = _fit_with_correspondence(white, yaw_hint)
    if r is None:
        return None, "대응 실패"
    pose, rms, s, ordered = r
    return {"pose": pose, "rms_mm": rms, "mm_per_px": s, "pts": ordered,
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
        r, why = measure_once(hint)
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

    if "--json" in sys.argv:
        print(json.dumps({"n": len(rows), "sigma_yaw_deg": sw, "sigma_pos_mm": pos_sigma,
                          "mean_mm_per_px": st.mean(sc), "pass": bool(ok_yaw and ok_pos)}))


if __name__ == "__main__":
    main()
