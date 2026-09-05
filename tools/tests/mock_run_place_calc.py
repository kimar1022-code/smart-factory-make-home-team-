"""로봇·카메라 없이 run() 순서·게이트 검증(모의 브리지). 설계 4단계 순서와 각 게이트가 정지시키는지 확인."""
import sys, json, types, math
sys.path.insert(0, "/home/ar/bf2_console/tools")
import place_calc as PC, hover_align as HA, house_geometry as HG, slot_target as STG

LOG = []
state = {"tcp": [200.0, -330.0, 650.0, 180.0, 0.0, 180.0], "grip": 30}
def st(): return {"tcp": list(state["tcp"]), "gripper": state["grip"], "frozen": False}
def move(tcp, tol=0.6, timeout=60, tag=""): state["tcp"] = list(tcp); LOG.append(("move", tag, [round(v,1) for v in tcp[:3]], round(tcp[5],1)))
def speed(v): pass
def post(a, b): LOG.append(("post", a))
def gripper(pos): state["grip"] = SCEN["grip_real"](pos); LOG.append(("grip", pos, state["grip"])); return str(state["grip"])
def grip_read(): return str(state["grip"])
PC.st = st; PC.move = move; PC.speed = speed; PC.post = post; PC.gripper = gripper; PC.grip_read = grip_read
PC.health_gate = lambda max_try=4: SCEN["health"]()
STG.pillars_px = lambda n=6, mask_held=False: SCEN["pillars"](n)
PC.rack_grip_xy = lambda color, dxy=(0.0,0.0): SCEN["rack_xy"](dxy)
PC.rack_ends = lambda color, n=4, x_hint=None: {"mid": (569,391), "len_px": 567, "ang": 1.39, "p1": (569,100), "p2": (569,680), "n_dots": 3}
PC._rack_color_center = lambda color, x_hint, radius=220.0: SCEN["rack_center"]()
PC._all_color_blobs_center = lambda img: ((900.0, 360.0), 6)
import hover_align as HA2; HA2.grab = lambda src="wrist": None
PC.rack_len_check = lambda color, e, strict=False: LOG.append(("rack_len_check", strict))
PC.held_wall_dots = lambda color: SCEN["held"]
PC.held_wall_depth = lambda color, nx=68, ny=72: ({"d_w": 109.0, "mm_px": 0.1195, "ang_img": 90.0, "span_px": 400, "center_px": (1025,430), "n": 300}, None)
PC.grasp_measure = lambda color, rack_dang=None: SCEN["grasp"](rack_dang)
PC.descend_monitored = lambda color, x, y, rot, zs, g_close: LOG.append(("descend", round(x,2), round(y,2), round(rot[2],2)))
HA.load_ref = lambda color, src=None: SCEN["ref"]
HA.align = lambda color, dry=False, tol_mm=0.3, tol_deg=0.15: SCEN["align"]()
HA.promote_ref = lambda color, note="": LOG.append(("promote", color))

def base_scenario():
    return {"grip_real": lambda pos: 13 if pos == 7 else pos, "held": [(1027,220,1349),(1028,646,1606)],
            "health": lambda: True,
            "pillars": lambda n: ([(723.8,124.4,0),(724.9,616.5,1),(404.8,617.7,2),(409.7,122.0,3)], None),
            "rack_xy": lambda dxy: (-153.5 + dxy[0], -522.9 + dxy[1], 0.3),
            "rack_center": lambda: ((640.0, 360.0), 2),
            "grasp": lambda rd: (HG.GripMeasure((0.2,-0.4), 90.0+0.1, 0.0), {"how":"2점","across_mm":0.2,"along_mm":-0.4,"dang":0.1,"scale":0.1195,"dx_px":1,"dy_px":3}),
            "ref": {"wrist": {}}, "align": lambda: (state.__setitem__("tcp", [state["tcp"][0]+0.6, state["tcp"][1]-1.2, state["tcp"][2], 180.0, 0.0, state["tcp"][5]-0.3]) or True)}

def run_case(name, mod):
    global SCEN, LOG
    SCEN = base_scenario(); SCEN.update(mod); LOG = []
    state["tcp"] = [200.0, -330.0, 650.0, 180.0, 0.0, 180.0]; state["grip"] = 30
    import io, contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf): PC.run("blue", seat=True)
    out = buf.getvalue()
    stops = [l for l in out.splitlines() if l.startswith("❌")]
    print(f"\n=== {name}")
    print("  순서:", " → ".join(t[1] if t[0]=="move" else (t[0] if t[0]!="grip" else f"grip{t[1]}") for t in LOG if t[0] in ("move","grip","descend","promote","rack_len_check")))
    for l in out.splitlines():
        if l.startswith(("①","②","③","④")) or "베이스(로봇)" in l or "목표 [" in l or "정렬 후" in l or "파지 편차" in l or "파지 판정" in l: print("  ", l)
    print("  정지:", stops or "없음", "| descend:", [t for t in LOG if t[0]=="descend"])
    return out

run_case("정상 풀사이클", {})
run_case("빈 파지(그리퍼 7→7, 벽점 0)", {"grip_real": lambda pos: pos, "held": []})
run_case("파지 편차 게이트 초과(길이 +2.3mm)", {"grasp": lambda rd: (HG.GripMeasure((0.1,2.3),90.0,0.0), {"how":"2점","across_mm":0.1,"along_mm":2.3,"dang":0.0,"scale":0.12,"dx_px":1,"dy_px":19})})
run_case("호버 기준 없음", {"ref": None})
run_case("호버 정렬 발산", {"align": (lambda: (_ for _ in ()).throw(RuntimeError("호버 정렬 발산(1.2→2.5mm) — 정지")))})
run_case("랙 양끝 끝내 못 잡음(3라운드)", {"rack_xy": lambda dxy: None, "rack_center": lambda: ((900.0, 300.0), 1)})

# --- 탐색 시나리오: 베이스 4점이 첫 라운드엔 3점(오른쪽으로 치우침) → 카메라 이동 후 4점
cnt = {"n": 0}
def health_seq():
    cnt["n"] += 1; return cnt["n"] >= 2
def pillars_seq(n):
    if cnt["n"] < 2: return ([(1100.0,124.4,0),(1101.0,616.5,1),(785.0,122.0,3)], None)     # 3점, 우측
    return ([(723.8,124.4,0),(724.9,616.5,1),(404.8,617.7,2),(409.7,122.0,3)], None)
cnt["n"] = 0
out = run_case("베이스 3점 → 카메라 이동 → 4점 (Δ 반영)", {"health": health_seq, "pillars": pillars_seq})
print("  탐색 이동:", [t for t in LOG if t[0]=="move" and "탐색" in t[1]][:3])
# --- 랙 탐색: 첫 관측 실패(점은 보임, 우측) → XY 이동 → 성공
rc = {"n": 0}
def rack_seq(dxy):
    rc["n"] += 1
    return None if rc["n"] == 1 else (-153.5 + dxy[0], -522.9 + dxy[1], 0.3)
run_case("랙 양끝 미검출 → 카메라 이동 → 성공(Δ 반영)", {"rack_xy": rack_seq, "rack_center": lambda: ((900.0, 380.0), 2)})
print("  랙 탐색 이동:", [t for t in LOG if t[0]=="move" and "랙 탐색" in t[1]], " 파지 XY:", [t for t in LOG if t[0]=="move" and t[1]=="픽 자세"])
