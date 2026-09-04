"""8/30 되돌리기 — 빨강 pick 기준을 '직전 티칭'(15:0x, X-157.14) 으로 복원.
  python3 restore_red_pick_prev.py
지금 저장된 값(15:1x, X-156.89/Y-515.91)이 마음에 안 들 때만 실행."""
import json
P = "/home/ar/bf2_console/dot_calib.json"
c = json.load(open(P))
r = c["refs"]["red"]
prev = {"pick_tcp_taught": [-157.14, -523.53, 342.51, 178.61, 0.95, 179.08],
        "pick_offset": [43.03, -73.34, -207.49], "pick_rz": 179.08,
        "target_obs": [625.0, 370.0, 310.2], "target_theta_deg": -89.7,
        "yaw_trim_deg": -1.22, "end_gap_px": 359.0}
print("현재 →", {k: r.get(k) for k in prev})
r.update(prev)
json.dump(c, open(P, "w"), ensure_ascii=False, indent=1)
print("복원 완료 →", prev)
