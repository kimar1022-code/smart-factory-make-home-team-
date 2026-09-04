#!/usr/bin/env python3
"""안착 판정 (80 mm 신호) — 9/4 재설계.

원리(STL 실측): 벽 높이 80 = 기둥 높이 80.
  · 제대로 꽂히면  → 벽 윗변 = 기둥 꼭대기 **같은 평면**
  · 기둥에 얹히면  → 벽 윗변이 기둥 꼭대기보다 **80 mm 위**
그래서 "기둥 꼭대기 평면보다 위로 솟은 것이 있는가"만 보면 얹힘이 100% 드러난다.
기존 벽점 25 px(≈2.5 mm) 간접 신호 대비 **32배 큰 신호** — 오판이 사라진다.

사용:
  python3 seat_verify.py            # 현재 손목캠/뎁스로 판정
  python3 seat_verify.py --save out.jpg
반환(라이브러리): verify() -> dict(state, above_mm, n_above, d_pillar_mm, ...)
  state: "seated"(솟은 것 없음) · "perched"(얹힘 의심) · "unknown"(기준면 못 잡음)

주의: 손목캠이 밑판을 내려다보는 자세에서 쓴다. 그리퍼가 화면에 크게 들어오면
      GRIP_MARGIN 으로 하단을 제외한다(기본 아래 12%).
"""
import sys, math, statistics as st
import numpy as np

sys.path.insert(0, "/home/ar/bf2_console/tools")
import base_depth_corner as B

PERCH_MM = 40.0        # 기둥 꼭대기보다 이만큼 위 = 얹힘(안착 0 vs 얹힘 80 의 중간)
SEAT_TOL_MM = 15.0     # 안착 허용(평면 요철·뎁스 잡음)
MIN_PTS = 8            # 얹힘 판정 최소 격자점 수(잡음 컷)
PILLAR_R = 22          # 흰 점 주변 이 반경(px) 안의 뎁스로 기둥 꼭대기 평면 추정
GRIP_MARGIN = 0.12     # 화면 아래 이 비율은 그리퍼일 수 있어 제외


def _depth_near(grid, x, y, r=PILLAR_R):
    """(x,y) 주변 r px 안 격자점의 유효 뎁스 목록."""
    out = []
    for q in grid["pts"]:
        d = q.get("d")
        if not d or d <= 0:
            continue
        if abs(q["x"] - x) <= r and abs(q["y"] - y) <= r:
            out.append(d)
    return out


def verify(img=None, grid=None, perch_mm=PERCH_MM):
    if img is None or grid is None:
        img, grid = B.grab_pair()
    if img is None:
        return {"state": "unknown", "why": "프레임 없음"}
    H, W = img.shape[:2]

    # 1) 기준면 = 기둥 꼭대기 4점의 뎁스
    try:
        m, _why = B.detect_rect(img, grid)
    except Exception:
        m = None
    # detect_rect 반환: {"box": minAreaRect 4점, "corners": 서브픽셀 정제 4점(None 가능), ...}
    rect = None
    if m:
        cs = [c for c in (m.get("corners") or []) if c]
        rect = cs if len(cs) == 4 else (m.get("box") or None)
    try:
        d = B.base_dots(img, rect)
        whites = sorted(d.get("white") or [], key=lambda p: -p[2])[:4]
    except Exception as e:
        return {"state": "unknown", "why": f"기둥점 검출 실패 {e}"}
    if len(whites) < 2:
        return {"state": "unknown", "why": f"기둥 꼭대기 흰 점 {len(whites)}개(2 이상 필요)"}

    pill_d = []
    for p in whites:
        pill_d += _depth_near(grid, p[0], p[1])
    if len(pill_d) < 4:
        return {"state": "unknown", "why": f"기둥 꼭대기 뎁스 표본 {len(pill_d)}개"}
    d_pillar = st.median(pill_d)

    # 2) 판정 영역 = 밑판 사각형(있으면) 안쪽, 그리퍼 마진 제외
    poly = np.array(rect, np.float32) if rect is not None else None
    ymax = H * (1.0 - GRIP_MARGIN)
    above, inside = [], 0
    for q in grid["pts"]:
        dv = q.get("d")
        if not dv or dv <= 0 or q["y"] > ymax:
            continue
        if poly is not None:
            import cv2
            if cv2.pointPolygonTest(poly, (float(q["x"]), float(q["y"])), False) < 0:
                continue
        inside += 1
        h = d_pillar - dv          # +면 기둥 꼭대기보다 위(카메라에 가까움)
        if h > perch_mm:
            above.append((q["x"], q["y"], h))

    n = len(above)
    res = {"d_pillar_mm": round(d_pillar, 1), "n_inside": inside, "n_above": n,
           "pillars": len(whites), "rect": rect is not None}
    if n >= MIN_PTS:
        hs = [a[2] for a in above]
        res.update(state="perched", above_mm=round(st.median(hs), 1),
                   above_max_mm=round(max(hs), 1),
                   spot=(round(st.median([a[0] for a in above])), round(st.median([a[1] for a in above]))),
                   why=f"기둥 꼭대기보다 {st.median(hs):.0f}mm 위로 솟은 격자점 {n}개 — 얹힘 의심")
    else:
        res.update(state="seated", above_mm=0.0,
                   why=f"기둥 꼭대기 평면 위로 솟은 것 없음(격자점 {n} < {MIN_PTS})")
    res["_above_pts"] = above
    res["_img"] = img
    return res


def main():
    r = verify()
    st_ = r["state"]
    mark = {"seated": "✅ 안착", "perched": "❌ 얹힘", "unknown": "⚠ 판정불가"}[st_]
    print(f"{mark} — {r.get('why')}")
    for k in ("d_pillar_mm", "n_inside", "n_above", "above_mm", "above_max_mm", "spot", "pillars", "rect"):
        if k in r:
            print(f"  {k}: {r[k]}")
    if "--save" in sys.argv and r.get("_img") is not None:
        import cv2
        out = r["_img"].copy()
        for (x, y, h) in r.get("_above_pts", []):
            cv2.circle(out, (int(x), int(y)), 4, (0, 0, 255), -1)
        cv2.putText(out, f"{st_} n_above={r.get('n_above')} d_pillar={r.get('d_pillar_mm')}",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imwrite(sys.argv[sys.argv.index("--save") + 1], out)
        print("저장:", sys.argv[sys.argv.index("--save") + 1])


if __name__ == "__main__":
    main()
