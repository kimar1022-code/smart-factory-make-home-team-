#!/usr/bin/env python3
"""호버에서 '밑판 슬롯 선 yaw = 벽 선 yaw' 판정. 골든 밑판 선(slot_golden.<색>.base_lines_z478) 근처에서 밑판 선·벽 선을 서브픽셀로 잡아 각차·간격 출력.
  python3 wall_vs_base.py blue [이미지경로] [--save out.jpg]"""
import sys, json, math, urllib.request
import cv2, numpy as np
sys.path.insert(0, "/home/ar/bf2_console/tools")
from edge_anchor import _subpix_line
CAL = "/home/ar/bf2_console/dot_calib.json"


def lines_near(gray, seed, band_offsets=(-70, -50, -30, -15, 0, 15, 30, 50, 70), band=8):
    """씨앗 직선을 법선 방향으로 평행이동한 여러 씨앗에서 서브픽셀 직선 후보 수집(중복 제거)."""
    x1, y1, x2, y2 = seed; ux, uy = x2 - x1, y2 - y1; L = math.hypot(ux, uy); ux, uy = ux / L, uy / L; nx, ny = -uy, ux
    found = []
    for off in band_offsets:
        sd = (x1 + nx * off, y1 + ny * off, x2 + nx * off, y2 + ny * off)
        r = _subpix_line(gray, sd, band=band, step=2)
        if r is None or r[1] > 0.6 or r[2] < 60:
            continue
        ang, sig, n, (vx, vy, x0, y0) = r
        d = (x0 - x1) * nx + (y0 - y1) * ny                   # 씨앗 선 기준 법선 오프셋(px)
        if all(abs(d - f["off"]) > 3 for f in found):
            # 전이 방향(밝→어둡 or 어둡→밝): 법선 +쪽 밝기 - 법선 −쪽 밝기
            px, py = int(x0 + nx * 6), int(y0 + ny * 6); mx, my = int(x0 - nx * 6), int(y0 - ny * 6)
            bp = float(gray[py, px]) if 0 <= py < gray.shape[0] and 0 <= px < gray.shape[1] else -1
            bm = float(gray[my, mx]) if 0 <= my < gray.shape[0] and 0 <= mx < gray.shape[1] else -1
            found.append({"off": float(d), "ang": float(ang), "sigma": float(sig), "n": int(n), "x0": float(x0), "y0": float(y0), "bright_plus": bp, "bright_minus": bm, "v": (float(vx), float(vy))})
    found.sort(key=lambda f: f["off"])
    return found


def main():
    ck = sys.argv[1] if len(sys.argv) > 1 else "blue"
    path = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else None
    if path:
        img = cv2.imread(path)
    else:
        buf = urllib.request.urlopen("http://127.0.0.1:8766/raw", timeout=5).read(); img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cal = json.load(open(CAL)); bl = cal["slot_golden"][ck]["base_lines_z478"]["refined"][1]     # 파랑 쪽(P3-P4)
    seed = (bl["p"][0], bl["p"][1], bl["q"][0], bl["q"][1])
    print(f"골든 밑판 선(파랑 쪽): ({seed[0]:.1f},{seed[1]:.1f})→({seed[2]:.1f},{seed[3]:.1f}) 각 {bl['ang']:+.3f}°")
    fs = lines_near(gray, seed)
    for f in fs:
        kind = "밝→어둡" if f["bright_minus"] > f["bright_plus"] else "어둡→밝"
        print(f"  선 오프셋 {f['off']:+6.1f}px  각 {f['ang']:+.3f}°  σ {f['sigma']:.2f} n {f['n']}  (+쪽 {f['bright_plus']:.0f} / −쪽 {f['bright_minus']:.0f}, {kind})")
    if "--save" in sys.argv:
        out = img.copy(); cv2.line(out, (int(seed[0]), int(seed[1])), (int(seed[2]), int(seed[3])), (0, 255, 0), 1)
        for f in fs:
            vx, vy = f["v"]; L = 380
            cv2.line(out, (int(f["x0"] - vx * L / 2), int(f["y0"] - vy * L / 2)), (int(f["x0"] + vx * L / 2), int(f["y0"] + vy * L / 2)), (0, 0, 255), 2)
            cv2.putText(out, f"{f['off']:+.0f}px {f['ang']:+.2f}", (int(f["x0"]) + 6, int(f["y0"])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imwrite(sys.argv[sys.argv.index("--save") + 1], out)


if __name__ == "__main__":
    main()
