#!/usr/bin/env python3
"""골든 파지사진(grasp_ref) + 파지 티칭 자세 저장 — 9/2 저녁 (노랑 확장, 색 범용).

  python3 grasp_capture.py yellow            # 지금 벽을 문 자세를 refs.<색>.grasp_ref / pick_tcp_taught 로 저장
  python3 grasp_capture.py yellow --lock     # 사용자 확정 잠금 표시

저장: grasp_ref = {tcp, grip(실측), cam1_wall/cam2_wall(벽 색 점 [kind,x,y,area]), all(전 점)},
      pick_tcp_taught/pick_joints_taught(구값은 *_bak_<시각> 보존), pick_bright(손목캠 밝기·노출).
"""
import json
import shutil
import sys
import time
import urllib.request

sys.path.insert(0, "/home/ar/bf2_console/tools")
from golden import CAL, dots, post, st, stable  # noqa: E402

ck = sys.argv[1] if len(sys.argv) > 1 else "yellow"
cal = json.load(open(CAL))
ref = cal["refs"][ck]
color = ref.get("dot_color", ck)
bak = CAL + time.strftime(".BEFORE_%m%d_%H%M_grasp")
shutil.copy(CAL, bak)
gr = str(post("grip_read", {"dry_run": True})["result"])
g = int(gr) if gr.isdigit() else 0
p = stable(); j = st()["joints"]
snap = {c: dots(c) for c in ("cam1", "cam2")}
made = time.strftime("%Y-%m-%d %H:%M")
try:
    e = json.loads(urllib.request.urlopen("http://127.0.0.1:8766/expo", timeout=5).read())
except Exception:
    e = {}
ref["grasp_ref"] = {"made": made, "locked": "--lock" in sys.argv, "grip": g,
                    "tcp": [round(v, 2) for v in p],
                    "cam1_wall": [d for d in snap["cam1"] if d[0] == color],
                    "cam2_wall": [d for d in snap["cam2"] if d[0] == color],
                    "all": snap,
                    "note": f"{made} 사용자 수동 파지({g}) 골든 파지사진 — grasp_capture.py"}
for k in ("pick_tcp_taught", "pick_joints_taught"):
    if k in ref:
        ref[f"{k}_bak_{time.strftime('%m%d')}"] = ref[k]
ref["pick_tcp_taught"] = [round(v, 2) for v in p]
ref["pick_joints_taught"] = [round(v, 3) for v in j]
ref["grip_close"] = g
ref["pick_bright"] = {"bright": e.get("bright"), "exposure": e.get("exposure"), "made": made,
                      "note": "골든 파지사진 촬영 시 손목캠 밝기·노출(109.9 정규화 후)"}
cal["refs"][ck] = ref
json.dump(cal, open(CAL, "w"), indent=1, ensure_ascii=False)
print(f"[{ck}] grasp_ref 저장: grip {g} tcp {ref['pick_tcp_taught'][:3]}  "
      f"cam1 벽점 {len(ref['grasp_ref']['cam1_wall'])} · cam2 벽점 {len(ref['grasp_ref']['cam2_wall'])}  "
      f"밝기 {e.get('bright', 0):.0f}/노출 {e.get('exposure', 0):.0f}  백업 {bak}")
print("  cam1:", snap["cam1"]); print("  cam2:", snap["cam2"])
