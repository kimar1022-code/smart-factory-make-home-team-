"""house_cycle 모의 검증 — 브리지·카메라 없이 5 시나리오."""
import os, sys, json, threading, time, shutil
SD = os.path.dirname(os.path.abspath(__file__)); ST = os.path.join(SD, "hs_mock")
shutil.rmtree(ST, ignore_errors=True)
os.environ["HOUSE_STATE"] = ST
sys.path.insert(0, "/home/ar/bf2_console/tools")
import house_cycle as HC
PC, HA, HG = HC.PC, HC.HA, HC.HG
HC.seed_state()
state = {"tcp": [200.0, -330.0, 650.0, 180.0, 0.0, 180.0], "grip": 30}
LOG = []
def st(): return {"tcp": list(state["tcp"]), "gripper": state["grip"], "busy": False, "frozen": False}
def post(a, b):
    LOG.append(("post", a))
    if a == "move_tcp":
        if b.get("dry_run"): return {"result": "dry_run"}
        state["tcp"] = list(b["tcp"]); return {"result": "started"}
    if a == "grip_read": return {"result": str(state["grip"])}
    return {"result": "ok"}
def gripper(pos): state["grip"] = SC["grip_real"](pos); LOG.append(("grip", pos, state["grip"])); return str(state["grip"])
PC.st = st; PC.post = post; PC.speed = lambda v: LOG.append(("speed", v)); PC.gripper = gripper; PC.grip_read = lambda: str(state["grip"])
PC.wait_idle = lambda t=30: None
HA.st = st; HA.post = lambda a, b: "ok"
def move_rel(dx, dy, drz, tol=0.3, timeout=40):
    c = state["tcp"]; state["tcp"] = [c[0]+dx, c[1]+dy, c[2], c[3], c[4], HG.wrap_deg(c[5]+drz)]; LOG.append(("rel", dx, dy, drz)); return state["tcp"]
HC._ha_move_rel = move_rel
# 검출기 모의
BASE_PX = [(300.0, 200.0, 0), (800.0, 200.0, 1), (800.0, 520.0, 2), (300.0, 520.0, 3)]
PC.find_base_4pts = lambda holding=False: ([(x+SC["base_shift_px"][0], y+SC["base_shift_px"][1], i) for x, y, i in BASE_PX], (0.0, 0.0))
HC.BT.markers = lambda: (None, "모의: 마커 없음")
PC.rack_ends = lambda color, n=4, x_hint=None: SC["rack"]
PC._grip_ok = lambda gr, g_close, color: (SC["grip_ok"], "모의")
PC.grasp_measure = lambda color, rack_dang=None: (object(), {"across_mm": SC["dev"][0], "along_mm": SC["dev"][1], "dang": SC["dev"][2], "how": "모의"})
PC.load_grasp_ref = lambda color: {"x": 1} if SC["sig"] else None
def align(color, dry=False, tol_mm=0.3, tol_deg=0.15):
    if SC["align"] == "diverge": raise RuntimeError("호버 정렬 발산(모의)")
    move_rel(0.6, -1.2, -0.3); LOG.append(("align", color))
HA.align = align; HA.restore_expo = lambda: None; HA.load_ref = lambda color, src=None: ({"z": 440} if SC["hover"] else None); HA.promote_ref = lambda color, note="": LOG.append(("promote", color))
PC.descend_monitored = lambda color, x, y, rot, zs, g_close: LOG.append(("descend", round(x, 2), round(y, 2), round(rot[2], 2), zs))
import types; sys.modules["seat_verify"] = types.SimpleNamespace(verify=lambda: {"state": "seated(모의)"})

def slot_ref(shift=(0, 0), dyaw=0.0):
    json.dump({"blue": {"seat_tcp": [203.0, -409.0, 354.0, 180.0, 0.0, 180.0], "base": {"x": 0.0, "y": 0.0, "yaw": -90.0}, "made": "모의"}}, open(HC.F["slot"], "w"))
    # 기준 베이스 = 지금 모의 픽셀에서 나오는 값으로 맞춘다
    B = HC.stage_base(); d = json.load(open(HC.F["slot"])); d["blue"]["base"] = {"x": B["x"], "y": B["y"], "yaw": B["yaw"]}; json.dump(d, open(HC.F["slot"], "w"))

def run(name, teach=False, buttons=()):
    global SC
    print(f"\n=== {name}"); LOG.clear(); HC.ABORT.clear(); HC.S.update(err=None, wait=None)
    def presser():
        for delay, op in buttons:
            if op == "abort": time.sleep(delay); HC.handle_cmd({"op": ["abort"]}); continue
            t0 = time.time()
            while HC.S.get("wait") is None and time.time() - t0 < 20: time.sleep(0.05)   # 실제 대기 상태가 된 뒤 버튼
            time.sleep(0.1)
            if op == "rack_offset": state["tcp"][0] += 2.0; state["tcp"][1] -= 1.0; HC.teach_rack_offset("blue"); HC.RESUME.set()
            elif op == "hover_ref": SC["hover"] = True; HC.RESUME.set()
            elif op == "sig": SC["sig"] = True; HC.RESUME.set()
    threading.Thread(target=presser, daemon=True).start()
    try:
        HC.run_cycle("blue", teach); stop = None
    except HC.Abort: stop = "중단"
    except HC.Gate as g: stop = f"게이트: {g}"
    except Exception as e: stop = f"예외: {e}"
    moves = [l for l in LOG if l[0] == "post" and l[1] == "move_tcp"]
    print("  단계:", HC.S["stage"], "| 정지:", stop)
    print("  이동수", len(moves), "| grip:", [l for l in LOG if l[0] == "grip"], "| align/promote:", [l for l in LOG if l[0] in ("align", "promote", "descend")])
    return stop

SC = {"base_shift_px": (0, 0), "rack": {"mid": (569.3, 390.7), "len_px": 567.8, "ang": 1.39, "n_dots": 5, "p1": (569.3, 106.8), "p2": (569.3, 674.6)},
      "grip_ok": True, "dev": (0.1, -0.4, 0.05), "sig": True, "hover": True, "align": "ok", "grip_real": lambda p: 13 if p < 20 else 30}
state["tcp"] = [200.0, -330.0, 650.0, 180.0, 0.0, 180.0]

# 0. 슬롯 기준 없음 → 픽 전에 정지
s = run("슬롯 기준 없음 → 픽 전 정지"); assert s and "슬롯 기준 없음" in s and not any(l[0] == "grip" for l in LOG)
slot_ref()
# 1. 정상: 베이스가 화면에서 +24px(≈10mm) 이동 + 정렬 → WAIT DESCEND → 하강 → DONE
SC["base_shift_px"] = (24.0, 0.0)
s = run("정상 풀사이클(베이스 +24px 이동)"); assert s is None and HC.S["stage"] == "WAIT DESCEND"
T = HC.S["target"]; print("  목표:", {k: round(v, 2) if isinstance(v, float) else v for k, v in T.items() if k in ("x", "y", "rz", "dyaw")})
assert abs((T["x"] - 203.0) - (-24 * 0.4135)) < 0.3 or abs((T["x"] - 203.0) + (-24 * 0.4135)) < 0.3, "베이스 이동이 목표에 반영돼야"
HC.stage_descend("blue"); d = [l for l in LOG if l[0] == "descend"][0]; a = HC.S["align"]
assert abs(d[1] - a["x"]) < 0.01 and abs(d[2] - a["y"]) < 0.01, "정렬된 TCP 로 하강해야"; print("  하강 TCP = 정렬 TCP ✓", d, "| 단계:", HC.S["stage"])
# 2. 랙 길이 불일치 → 파지 금지
SC["base_shift_px"] = (0, 0); SC["rack"] = dict(SC["rack"], len_px=479.0)
s = run("랙 길이 −16% → 파지 금지"); assert s and "길이 불일치" in s and not any(l[0] == "grip" and l[1] < 20 for l in LOG)
SC["rack"] = dict(SC["rack"], len_px=567.8)
# 3. 파지 편차 게이트
SC["dev"] = (0.1, 2.3, 0.0); s = run("파지 편차 +2.3mm → 정지"); assert s and "게이트 초과" in s
SC["dev"] = (0.1, -0.4, 0.05)
# 4. 랙 파지 티칭: 대기 → 조그(+2,−1) → 랙 보정 저장 → 계속 → 정상 완료, offset 이 벽 축으로 저장됨
s = run("랙 파지 티칭 흐름", teach=True, buttons=[(0.5, "rack_offset")]); assert s is None
off = json.load(open(HC.F["rack"]))["blue"]["offset"]; print("  저장 offset:", off); assert abs(abs(off["along"]) - 1.0) < 0.05 and abs(abs(off["across"]) - 2.0) < 0.05
# 5. 서명 없음 → 대기 → 서명 저장 → 계속
SC["sig"] = False; s = run("좋은 파지 서명 없음 → 버튼 대기 → 계속", buttons=[(0.5, "sig")]); assert s is None
# 6. z440 기준 없음 → 대기 → 저장 → 계속 → 정렬
SC["hover"] = False; s = run("z440 기준 없음 → 버튼 대기 → 계속", buttons=[(0.5, "hover_ref")]); assert s is None and any(l[0] == "align" for l in LOG)
# 7. 정렬 발산 → 정지(하강 없음)
SC["align"] = "diverge"; s = run("호버 정렬 발산 → 정지"); assert s and "발산" in s; SC["align"] = "ok"
# 8. 사용자 중단(운반 중)
s = run("운반 중 ⛔ 중단", buttons=[(0.05, "abort")]); assert s == "중단" and ("post", "stop") in LOG
# 9. 빈 파지
SC["grip_ok"] = False; s = run("빈 파지 → 정지"); assert s and "빈 파지" in s
print("\n모의 전부 통과 (10 시나리오)")
