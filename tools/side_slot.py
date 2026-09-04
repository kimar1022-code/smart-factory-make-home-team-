#!/usr/bin/env python3
"""측면 카메라(C270 :8771) 삽입 감시 검출기 — 9/4 재설계.
목적: 손목캠이 눈머는 기둥 진입 후(z437↓), 옆에서 벽 밑동이 슬롯에 들어가는지 vs 기둥에 얹히는지 판정.
     베이스가 움직여도 슬롯 위치를 측면에서 형상으로 재검출하므로 절대좌표 의존 없음.

구조(오늘 스캐폴드):
  base_top_line(img, roi)  : 밑판 윗면(수평 기준선) — 안착 목표 높이. 밝음(책상/벽 윗부분)→어둠(밑판) 전이.
  wall_bottom_y(img, roi)  : 벽 실루엣 아래 끝 y — 하강 중 벽 밑동 위치.
  seating(img, cfg)        : wall_bottom 이 base_top 까지 내려왔는지 → 'seated'/'descending'/'jam'(위로 밀림) 판정.
검출 파라미터(ROI·문턱)는 로봇이 실제로 벽을 내리는 프레임으로 튜닝해야 확정된다(cfg 저장은 나중).

  python3 side_slot.py [이미지|live] [--save out.jpg]
"""
import sys, json, math, urllib.request
import cv2, numpy as np

PORT = 8771
CFG_DEFAULT = {
    # 아래 값들은 임시(현재 프레임 기준 추정) — 삽입 프레임으로 재실측 필요
    "wall_roi": [230, 60, 180, 340],     # [x,y,w,h] 벽이 내려오는 세로 통로(슬롯 위)
    "base_top_roi": [200, 300, 240, 120], # 밑판 윗면 찾을 대역
    "dark_th": 70,                        # 벽/밑판 어둠 문턱
    "min_run": 12,                        # 연속 어둠 최소(px)
}


def grab(port=PORT):
    buf = urllib.request.urlopen(f"http://127.0.0.1:{port}/raw", timeout=5).read()
    return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)


def _cols_dark_bottom(gray, x0, y0, w, h, th, min_run):
    """ROI 각 열에서 위→아래로 '연속 어둠(벽)' 의 아래 끝 y. 열별 중앙값 반환."""
    ys = []
    for x in range(x0, x0 + w, 3):
        col = gray[y0:y0 + h, x].astype(int)
        dark = col < th
        # 가장 긴 어둠 런의 끝
        best_end, run, start = None, 0, None
        for i, d in enumerate(dark):
            if d:
                run += 1
                if run == 1:
                    start = i
            else:
                if run >= min_run:
                    best_end = i
                run = 0
        if run >= min_run:
            best_end = len(dark)
        if best_end is not None:
            ys.append(y0 + best_end)
    if len(ys) < 5:
        return None
    return float(np.median(ys)), float(np.std(ys)), len(ys)


def wall_bottom_y(img, cfg):
    """벽 실루엣 아래 끝 y (하강 중 벽 밑동)."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    x0, y0, w, h = cfg["wall_roi"]
    r = _cols_dark_bottom(g, x0, y0, w, h, cfg["dark_th"], cfg["min_run"])
    return None if r is None else {"y": r[0], "sigma": r[1], "n": r[2]}


def base_top_line(img, cfg):
    """밑판 윗면 수평 기준선 y — 밝음→어둠(밑판) 전이의 위쪽 경계. 안착 목표."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    x0, y0, w, h = cfg["base_top_roi"]
    ys = []
    for x in range(x0, x0 + w, 4):
        col = g[y0:y0 + h, x].astype(int)
        dark = col < cfg["dark_th"]
        # 위에서 첫 '연속 어둠' 시작 = 밑판 윗면
        run, start = 0, None
        for i, d in enumerate(dark):
            run = run + 1 if d else 0
            if run >= cfg["min_run"]:
                start = i - run + 1; break
        if start is not None:
            ys.append(y0 + start)
    if len(ys) < 5:
        return None
    return {"y": float(np.median(ys)), "sigma": float(np.std(ys)), "n": len(ys)}


def seating(img, cfg, golden_base_top=None):
    """판정: 벽 밑동 y vs 밑판 윗면 y.
    - descending: 벽 밑동이 밑판 윗면보다 위(아직 내려가는 중)
    - seated    : 벽 밑동이 밑판 윗면 ±TOL
    - jam       : 벽 밑동이 밑판 윗면보다 아래로 안 내려가고 되레 위로(그리퍼서 밀림) — 골든 대비 상승
    (측면캠 y 증가 = 화면 아래 = 물리 하강, 카메라 각도에 따라 부호는 튜닝 필요)"""
    wb = wall_bottom_y(img, cfg); bt = base_top_line(img, cfg)
    out = {"wall_bottom": wb, "base_top": bt}
    if wb and bt:
        gap = bt["y"] - wb["y"]     # +면 벽 밑동이 밑판 윗면 위(아직 위) — 부호는 실측 튜닝
        out["gap_px"] = gap
        out["state"] = "seated" if abs(gap) < 10 else ("descending" if gap > 0 else "past")
    return out


def draw(img, s, cfg):
    out = img.copy()
    for key, col in (("wall_roi", (0, 200, 0)), ("base_top_roi", (255, 160, 0))):
        x, y, w, h = cfg[key]; cv2.rectangle(out, (x, y), (x + w, y + h), col, 1)
    if s.get("wall_bottom"):
        y = int(s["wall_bottom"]["y"]); cv2.line(out, (cfg["wall_roi"][0], y), (cfg["wall_roi"][0] + cfg["wall_roi"][2], y), (0, 255, 0), 2)
        cv2.putText(out, "wall bottom", (cfg["wall_roi"][0], y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    if s.get("base_top"):
        y = int(s["base_top"]["y"]); cv2.line(out, (cfg["base_top_roi"][0], y), (cfg["base_top_roi"][0] + cfg["base_top_roi"][2], y), (255, 160, 0), 2)
        cv2.putText(out, "base top", (cfg["base_top_roi"][0], y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 160, 0), 1)
    cv2.putText(out, f"{s.get('state','?')} gap {s.get('gap_px','-')}", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return out


def main():
    src = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else "live"
    img = grab() if src == "live" else cv2.imread(src)
    cfg = CFG_DEFAULT
    s = seating(img, cfg)
    print({k: (v if not isinstance(v, dict) else {kk: round(vv, 1) for kk, vv in v.items()}) for k, v in s.items()})
    if "--save" in sys.argv:
        cv2.imwrite(sys.argv[sys.argv.index("--save") + 1], draw(img, s, cfg))
        print("저장:", sys.argv[sys.argv.index("--save") + 1])


if __name__ == "__main__":
    main()
