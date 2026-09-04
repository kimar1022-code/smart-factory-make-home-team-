#!/usr/bin/env python3
"""파지 재측정 — 잡은 뒤 "벽이 그리퍼에 어떻게 물렸나"를 매 사이클 측정 (9/4 재설계).

왜: 랙에서 벽이 3 mm 씩 흔들려도(red_s 실증) **잡은 뒤 정확히 재면** 삽입 계산이 맞는다.
    골든은 폐기하지 않는다 — 골든 = 공칭, 이 측정 = 보정치.

원리:
  · 고정 '제시 자세'에서 손목캠으로 벽 **아래변 선**과 양 끝점을 서브픽셀로 잡는다.
  · 벽 길이가 알려져 있으므로(긴벽 198 / 짧은벽 125) 검출된 끝점 간 픽셀거리에서
    **mm/px 를 자가 보정** — 마운트가 바뀌어도 별도 캘리브 불필요.
  · 공칭(teach) 대비 (Δ중점, Δ각) 을 mm/° 로 내어 house_geometry.GripMeasure 에 얹는다.

  python3 grip_measure.py teach <색>      # 지금 파지를 공칭으로 저장
  python3 grip_measure.py run   <색>      # 지금 파지 측정 → 공칭 대비 편차 + GripMeasure
  python3 grip_measure.py check           # 검출만(저장 없음)

게이트(재설계 v2.1): 길이방향 편차 > 1.0 mm 또는 |Δ각| > 0.2° → 정지·보고(자동 재파지 금지, 철칙).
"""
import sys, json, math
import numpy as np

sys.path.insert(0, "/home/ar/bf2_console/tools")
import edge_place as EP
import house_geometry as HG

CAL = "/home/ar/bf2_console/dot_calib.json"
WALL_LEN_MM = {"blue": 198.0, "red": 198.0, "yellow": 125.5, "red_s": 125.0, "green": 126.0}
GATE_LEN_MM = 1.0        # 길이방향 편차 상한
GATE_ANG_DEG = 0.2       # 그리퍼 내 기울기 상한


def measure_edge(img, cfg):
    """벽 아래변 선 + 양 끝점(px). cfg = edge_place 의 cfg(대비 규칙 ROI 포함).
    반환 {"mid":(x,y), "ang":deg, "len_px":L, "sigma":σ, "n":n} 또는 (None, why)"""
    m, why = EP.measure(img, cfg)
    if not m:
        return None, why
    w = m["_lines"]["wall"]
    vx, vy = w["v"]
    x0, y0 = w["x0"], w["y0"]
    # ROI 세로 범위를 선 위로 투영해 양 끝점
    y_lo, y_hi = cfg["wall_rows"]
    def at(y):
        return (x0 + vx * (y - y0) / vy, y) if abs(vy) > 1e-6 else (x0, y)
    p1, p2 = at(y_lo), at(y_hi)
    L = math.dist(p1, p2)
    mid = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)
    return {"mid": mid, "ang": math.degrees(math.atan2(vy, vx)), "len_px": L,
            "sigma": w["sigma"], "n": w["n"], "p1": p1, "p2": p2}, None


def to_mm(meas, wall_len_mm):
    """벽 길이 기지 → mm/px 자가 보정. 검출 선 길이는 ROI 로 잘린 일부이므로
    ROI 전체를 쓰지 않고 **끝점이 실제 벽 끝**일 때만 유효하다.
    ROI 가 벽보다 짧으면 축척은 외부(base_pose_check 의 mm/px)를 받아 쓴다."""
    if meas["len_px"] <= 1:
        return None
    return wall_len_mm / meas["len_px"]


def deviation(cur, nom, mm_per_px):
    """공칭 대비 편차 → (길이방향 mm, 두께방향 mm, 각 °).
    선 방향을 길이축으로 잡아 분해한다."""
    a = math.radians(nom["ang"])
    ux, uy = math.cos(a), math.sin(a)          # 길이축
    nx, ny = -uy, ux                            # 두께축(법선)
    dx = (cur["mid"][0] - nom["mid"][0]) * mm_per_px
    dy = (cur["mid"][1] - nom["mid"][1]) * mm_per_px
    return {"along_mm": dx * ux + dy * uy,
            "across_mm": dx * nx + dy * ny,
            "dang_deg": HG.wrap_deg(cur["ang"] - nom["ang"])}


def grip_measure_from(nom_grip, dev):
    """공칭 GripMeasure + 편차 → 이번 사이클 GripMeasure."""
    a = math.radians(nom_grip.angle_deg)
    ux, uy = math.cos(a), math.sin(a)
    nx, ny = -uy, ux
    cx = nom_grip.center[0] + dev["along_mm"] * ux + dev["across_mm"] * nx
    cy = nom_grip.center[1] + dev["along_mm"] * uy + dev["across_mm"] * ny
    return HG.GripMeasure(center=(cx, cy),
                          angle_deg=HG.wrap_deg(nom_grip.angle_deg + dev["dang_deg"]),
                          bottom_dz=nom_grip.bottom_dz)


def _load_cfg(ck):
    cal = json.load(open(CAL))
    return cal, EP.default_cfg(cal, ck)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    ck = sys.argv[2] if len(sys.argv) > 2 else "blue"
    cal, cfg = _load_cfg(ck)
    img = EP.grab()
    m, why = measure_edge(img, cfg)
    if not m:
        print(f"❌ 벽 아래변 검출 실패: {why}"); return
    print(f"검출: 중점({m['mid'][0]:.1f},{m['mid'][1]:.1f})px 각 {m['ang']:+.3f}° "
          f"길이 {m['len_px']:.1f}px σ{m['sigma']:.2f} n{m['n']}")

    if mode == "check":
        return

    key = f"grip_ref_{ck}"
    if mode == "teach":
        cal.setdefault("grip_ref", {})[ck] = {
            "mid": list(m["mid"]), "ang": m["ang"], "len_px": m["len_px"],
            "wall_len_mm": WALL_LEN_MM.get(ck), "made": __import__("time").strftime("%Y-%m-%d %H:%M")}
        json.dump(cal, open(CAL, "w"), ensure_ascii=False, indent=1)
        print(f"✅ 공칭 저장: grip_ref.{ck}")
        return

    nom = (cal.get("grip_ref") or {}).get(ck)
    if not nom:
        print(f"❌ 공칭 없음 — 먼저 `grip_measure.py teach {ck}`"); return
    # 축척: 공칭의 길이(px)와 알려진 벽 길이로
    mmpx = (nom.get("wall_len_mm") or WALL_LEN_MM.get(ck, 198.0)) / max(nom["len_px"], 1.0)
    dev = deviation(m, nom, mmpx)
    print(f"공칭 대비: 길이방향 {dev['along_mm']:+.2f}mm · 두께방향 {dev['across_mm']:+.2f}mm · 각 {dev['dang_deg']:+.3f}°  (축척 {mmpx:.4f}mm/px)")
    bad = abs(dev["along_mm"]) > GATE_LEN_MM or abs(dev["dang_deg"]) > GATE_ANG_DEG
    print(("❌ 게이트 초과 — 정지·보고(자동 재파지 금지)" if bad else "✅ 게이트 통과")
          + f"  [길이 ≤{GATE_LEN_MM}mm, 각 ≤{GATE_ANG_DEG}°]")


if __name__ == "__main__":
    main()
