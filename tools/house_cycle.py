#!/usr/bin/env python3
"""house_cycle.py — 조립식 주택 벽 삽입 사이클 (9/6 재설계, 사용자 설계 4단계 + 버튼)  :8776

설계(사용자, 9/6 11:2x):
  1. base above(관측자세 z650, 빈 손): ArUco(고정 자) 대비 기둥 4점이 얼마나 움직였나 → 곧 꽂을 벽의 자리(x, y, yaw).
  2. rack above: 그 벽의 양끝점 → 중앙에 그리퍼(벌림 30) → 벽별 보정(사용자 버튼으로 기억) → z 하강 → 파지
     → 든 벽의 색점 기록(좋은 파지 서명과 비교해 편차를 3단계에 반영).
  2'. 1 의 자리로 운반 → z_seat+85(=z440) → 기둥점↔벽점 관계를 '잘 들어갔을 때' 기준과 비교, 다르면 맞춘다.
  3. 사용자 허락 → 버튼으로 수직 하강(막힘 감시). x·y·yaw 는 그 전에 다 맞아 있어야 한다.

원칙: 상태 파일은 state/house/ 에 새로(옛 golden/anchor/ref 안 읽음). 검출기·하강감시·호버정렬은 어제 실기 검증된 부품을 그대로 쓴다.
      어떤 예외든 stop 이 먼저. 기둥 꼭대기 아래(z_seat+85 미만)에서 XY·rz 이동 금지. 자동 재시도 없음(정지·보고, 사용자 판단).
      랙 위 벽은 항상 같은 방향으로 놓는다(사용자 결정) → 벽별 보정 부호가 유지된다.

버튼(벽별): 슬롯 기준 저장(1/2 안착 TCP, 2/2 관측 페어링) · 랙 보정 저장 · 좋은 파지 서명 저장 · z440 기준 저장 · 고정캠 매핑(newcam/side)
동작:  ▶ 사이클(1→2→2' 자동, 필요한 기준이 없으면 그 자리에서 멈추고 버튼을 기다림) · ⬇ 하강 · ⛔ 중단 · 관측자세로

9/6 보강 5건:
  ① 하강 후 안착 판정이 seated 가 아니면 **그리퍼를 열지 않고 그 자리에서 정지**(SEAT FAIL) → [⛔중단](그대로 정지, 벽은 사용자가)
     또는 [▶계속](사용자가 육안으로 안착 확인 → 개방·상승, 기준 승격은 없음). seated 일 때만 promote_ref·슬롯 기준 승격·개방·상승.
  ② 하강 게이트: 정렬 완료 TCP ↔ 하강 직전 TCP 가 XY 0.5mm / rz 0.3° 안 + z ±3 + 정렬 후 15분 이내(넘으면 재정렬 요구) + 색 일치.
  ③ 안착 성공 시 슬롯 기준 자동 승격(안착 TCP x·y·rz + 이번 사이클 베이스; z 는 티칭값 유지) — 이전 값은 history(최대 5) 보존.
  ④ run4(op=run4 / --auto): blue→yellow→red→red_s 연속, 벽마다 빈손 베이스 재측정, 하강은 매번 [⬇ 하강] 버튼에서 대기. 하나라도 정지·실패면 전체 중단.
  ⑤ slot_both: 슬롯 기준 1/2(벽 물고 TCP) → "벽 놓고 그리퍼 빼기" 대기(▶계속) → 2/2 를 한 버튼으로. 빈손(그리퍼 ≤ 닫힘값 또는 벽 점 0)이면 1/2 거부.

  python3 house_cycle.py            # 서버 :8776  (http://<ip>:8776/)
  python3 house_cycle.py --auto     # 서버 기동 + run4 를 바로 대기열에 (각 벽 하강은 버튼)
"""
import sys, os, json, math, time, threading, queue, traceback
import urllib.request as UR
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import numpy as np

sys.path.insert(0, "/home/ar/bf2_console/tools")
import place_calc as PC          # 부품: find_base_4pts / rack_ends / _grip_ok / grasp_measure / save_grasp_sig_now / descend_monitored / held_wall_dots
import hover_align as HA         # 부품: align / save_ref / promote_ref / probe_fixed / wall_dots / grab
import house_geometry as HG
import slot_target as STG        # 부품: pillars_px / base_pose_robot / load_map
import base_twist as BT          # 부품: markers / board_frame_transform / apply_sim  (ArUco 고정 자)

STATE = os.environ.get("HOUSE_STATE", "/home/ar/bf2_console/state/house")
os.makedirs(os.path.join(STATE, "logs"), exist_ok=True)
F = {k: os.path.join(STATE, v) for k, v in {
    "slot": "slot_ref.json",        # 색별 {seat_tcp, base(x,y,yaw 로컬), made}
    "rack": "rack_ref.json",        # 색별 {Pc0, Tg0, ang0, len0_px, z_pick, grip_open, grip_close, offset{along,across}}
    "rack_map": "rack_map.json",    # 랙 관측자세 화면→로봇 Jinv(mm/px)
    "rack_pose": "rack_pose.json",  # 랙 관측자세 TCP
    "sig": "grasp_sig.json",        # 좋은 파지 서명(place_calc by_color 스키마)
    "hover": "hover_ref.json",      # z440 기준 관계(hover_align 스키마)
    "aruco": "aruco_ref.json",      # 관측자세 ArUco 기준 코너
    "base_last": "base_last.json",
}.items()}
PC.GRASP_REF = F["sig"]            # ★부품이 새 상태 파일을 보게 전환(옛 grasp_ref_0905/hover_ref_0905 안 읽음)
HA.REF = F["hover"]

PORT = 8776
OBS = PC.OBS                                    # [200,-330,650,180,0,180]
SAFE_Z = PC.SAFE_Z                              # 650
HOVER_Z = PC.HOVER_Z                            # 478
COLORS = ("blue", "yellow", "red", "red_s")
GRIP_OPEN = 30                                  # 사용자 설계: 벌림 30 으로 내려온다
RACK_RZ_FOLLOW = False                          # 랙 위 벽 각을 rz 로 따라갈지(부호 미검증 → 기본 끔, 각은 보고만)
RACK_ANG_MAX = 3.0                              # 랙 위 벽 각 변화 상한(넘으면 벽이 삐뚤게 놓인 것 → 정지)
ARUCO_WARN_MM, ARUCO_WARN_DEG = 1.5, 0.3        # 고정 자 대비 카메라 복귀 오차 경고
GRASP_GATE_MM, GRASP_GATE_DEG = PC.GRASP_GATE_MM, PC.GRASP_GATE_DEG   # 1.0 / 0.3
PRECORR_MAX_MM = 1.0                            # 파지 편차 선보정 상한(부호 미검증 — 호버 정렬이 나머지를 흡수)
PRECORR_RZ = False                              # 13:36·14:01 실기 2회: rz 선보정 −0.43° 를 사용자가 매번 정확히 되돌림(rz 180) → 끔
SEAT_NEWCAM_TOL_PX = 12                         # 새카메라 안착 판정: 기둥 점이 안착 기준 자리에서 이 px 안이면 seated(≈2.5mm)
SPD_MOVE, SPD_DESC, SPD_SEAT = PC.SPD_MOVE, PC.SPD_DESC, PC.SPD_SEAT   # 30 / 10 / 3
ALIGN_GATE_MM, ALIGN_GATE_DEG = 0.5, 0.3        # ②하강 직전 TCP 가 정렬 완료 TCP 와 이만큼 안이어야(XY 거리 / rz)
ALIGN_MAX_AGE_S = 15 * 60                       # ②정렬 완료 후 이 시간이 지나면 하강 거부(베이스가 움직였을 수 있음 → 재정렬)
SLOT_HISTORY_MAX = 5                            # ③슬롯 기준 승격 시 보존하는 이전 값 개수
RUN4_ORDER = ("blue", "yellow", "red", "red_s") # ④4벽 연속 순서
# 13:53 실기 2회: 손목캠↔새카메라 불일치 2.1mm 반복. 사용자 육안 자리와 비교하면 새카메라가 두 번 다 가까웠음(rz 특히).
#   손목캠은 든 벽 윗점이 프레임 가장자리(x≈1025)라 원근·죠 안 기울기에 민감 → 파랑은 새카메라 단독 정렬(손목캠은 참고 출력).
ALIGN_SRCS = {"blue": ("newcam",)}                # 색별 정렬 카메라(없으면 가용 전부)


# ------------------------------------------------------------------ 상태·로그
class Abort(Exception): pass
class Gate(Exception): pass

LOCK = threading.Lock()
ABORT = threading.Event()
RESUME = threading.Event()
S = {"stage": "IDLE", "color": None, "busy": False, "wait": None, "log": [], "err": None,
     "base": None, "rack": None, "grasp": None, "target": None, "align": None, "seat": None,
     "gates": {}, "tcp": None, "grip": None, "run4": None, "made": time.strftime("%H:%M:%S")}
_logf = None


def log(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    print(line, flush=True)
    with LOCK:
        S["log"].append(line); S["log"] = S["log"][-300:]
    if _logf:
        _logf.write(line + "\n"); _logf.flush()


def set_stage(stage, wait=None, **kw):
    with LOCK:
        S["stage"] = stage; S["wait"] = wait; S.update(kw)
    log(f"── {stage}" + (f" (대기: {wait})" if wait else ""))


def jload(path, default=None):
    return json.load(open(path)) if os.path.exists(path) else default


def jsave(path, d):
    json.dump(d, open(path, "w"), ensure_ascii=False, indent=1)


# ------------------------------------------------------------------ 로봇(중단 가능 이동) — 부품들이 이걸 쓰도록 주입
def move(tcp, tol=0.6, timeout=60, tag=""):
    if ABORT.is_set():
        raise Abort()
    r = PC.post("move_tcp", {"tcp": tcp, "dry_run": True})
    if r.get("result") != "dry_run":
        raise RuntimeError(f"{tag} dry_run 거부 {r}")
    r = PC.post("move_tcp", {"tcp": tcp, "dry_run": False})
    if r.get("result") not in ("started", "ok"):
        raise RuntimeError(f"{tag} 이동 거부 {r}")
    t0 = time.time()
    while time.time() - t0 < timeout:
        if ABORT.is_set():
            PC.post("stop", {"dry_run": False}); raise Abort()
        s = PC.st()
        if s.get("frozen"):
            raise RuntimeError(f"{tag} 동결")
        c = s["tcp"]
        if max(abs(c[i] - tcp[i]) for i in range(3)) <= tol and abs(HG.wrap_deg(c[5] - tcp[5])) <= 0.3 and not s["busy"]:
            time.sleep(0.4)
            log(f"  ✓ {tag} ({c[0]:.1f},{c[1]:.1f},{c[2]:.1f}) rz{c[5]:+.2f}")
            return c
        time.sleep(0.2)
    raise RuntimeError(f"{tag} 미도달 목표{[round(v, 1) for v in tcp[:3]]} 현재{[round(v, 1) for v in PC.st()['tcp'][:3]]}")


PC.move = move                                   # find_base_4pts / descend_monitored 등이 중단 가능 이동을 쓴다
_ha_move_rel = HA.move_rel
def _move_rel_guard(dx, dy, drz, tol=0.3, timeout=40):
    """hover_align 의 상대이동을 이 파일의 중단 가능 move(move_tcp+dry_run+TCP 폴링) 로. 1% 속도."""
    if ABORT.is_set():
        raise Abort()
    c = PC.st()["tcp"]
    tgt = [c[0] + dx, c[1] + dy, c[2], c[3], c[4], HG.wrap_deg(c[5] + drz)]
    PC.speed(1)
    return move(tgt, tol=tol, timeout=timeout, tag=f"상대({dx:+.1f},{dy:+.1f},{drz:+.1f}°)")
HA.move_rel = _move_rel_guard


def wait_user(what):
    """버튼 대기. RESUME 또는 ABORT."""
    RESUME.clear()
    with LOCK:
        S["wait"] = what
    log(f"⏸ 사용자 대기: {what}")
    while not RESUME.wait(0.2):
        if ABORT.is_set():
            raise Abort()
    if ABORT.is_set():                 # ★abort 는 RESUME 도 함께 set 하므로(대기 깨우기) 깨어난 직후 반드시 ABORT 우선 — 아니면 '계속' 으로 오인
        raise Abort()
    with LOCK:
        S["wait"] = None


def up_to_safe():
    cur = PC.st()["tcp"]
    if cur[2] < SAFE_Z - 1:
        PC.speed(SPD_MOVE); move([cur[0], cur[1], SAFE_Z] + list(cur[3:]), tag="상승 SAFE")


def goto_obs():
    up_to_safe(); PC.speed(SPD_MOVE); move(OBS, tag="관측자세"); PC.speed(1)


# ------------------------------------------------------------------ 씨앗(첫 기동 시 1회): 랙 자세·매핑·벽별 파지 z 는 로봇 자세 수준의 사실이라 가져온다
def seed_state():
    if not os.path.exists(F["rack_pose"]):
        src = jload("/home/ar/bf2_console/rack_observe_pose.json")
        if src: jsave(F["rack_pose"], dict(src, note="seed 9/5 rack_observe_pose")); log("seed: rack_pose")
    if not os.path.exists(F["rack_map"]):
        rc = jload("/home/ar/bf2_console/rack_calib.json") or {}
        if rc.get("blue", {}).get("Jinv"):
            jsave(F["rack_map"], {"Jinv_mm_per_px": rc["blue"]["Jinv"], "note": "seed 9/5 ±10mm probe @rack observe (0.322mm/px)"}); log("seed: rack_map")
    if not os.path.exists(F["rack"]):
        rc = jload("/home/ar/bf2_console/rack_calib.json") or {}
        cal = (jload("/home/ar/bf2_console/dot_calib.json") or {}).get("refs", {})
        pr = jload("/home/ar/bf2_console/pick_ref_0905.json") or {}
        out = {}
        for c in COLORS:
            if c not in rc: continue
            z = (pr.get(c) or {}).get("tcp", cal.get(c, {}).get("pick_tcp_taught", [0, 0, 342]))[2]
            out[c] = {"Pc0": rc[c]["Pc0"], "Tg0": rc[c]["Tg0"], "ang0": rc[c]["ang0"], "len0_px": rc[c]["len0_px"],
                      "z_pick": z, "grip_open": GRIP_OPEN, "grip_close": (pr.get(c) or {}).get("grip_close", cal.get(c, {}).get("grip_close", 13)),
                      "offset": {"along": 0.0, "across": 0.0}, "note": "seed 9/5 사용자 시연 중앙(파랑·빨강·red_s)/taught(노랑) — 랙 보정 버튼으로 덮어씀"}
        if out: jsave(F["rack"], out); log(f"seed: rack_ref {list(out)}")


# ------------------------------------------------------------------ 1단계: 베이스(빈 손, ArUco 자 보정)
def aruco_correct(px4):
    """고정 ArUco 자로 카메라 복귀 오차를 상쇄한 기둥 px. 기준 없으면 저장만 제안, 마커 부족이면 미보정."""
    m, why = BT.markers()
    if m is None:
        return px4, {"applied": False, "why": why}
    ref = jload(F["aruco"])
    if not ref:
        return px4, {"applied": False, "why": "ArUco 기준 없음(슬롯 기준 2/2 저장 시 자동 생성)", "n": len(m)}
    T, whyT = BT.board_frame_transform(m, {int(k): v for k, v in ref["markers"].items()})
    if T is None:
        return px4, {"applied": False, "why": whyT, "n": len(m)}
    mmpx = 0.4135
    shift_mm = math.hypot(T["tx"], T["ty"]) * mmpx; dth = math.degrees(T["th"]) if abs(T["th"]) < 6.3 else T["th"]
    fixed = BT.apply_sim([(p[0], p[1]) for p in px4], T["s"], T["th"], T["tx"], T["ty"])
    out = [(x, y) + tuple(p[2:]) for (x, y), p in zip(fixed, px4)]
    info = {"applied": True, "ids": T["ids"], "rms_px": round(T["rms_px"], 2), "shift_mm": round(shift_mm, 2), "dth_deg": round(dth, 3), "s": round(T["s"], 4)}
    if shift_mm > ARUCO_WARN_MM or abs(dth) > ARUCO_WARN_DEG:
        log(f"  ⚠ ArUco 자: 카메라가 기준 대비 {shift_mm:.2f}mm/{dth:+.2f}° 벗어남 — 마운트/복귀 오차 의심(보정은 적용)")
    return out, info


def stage_base():
    """관측자세 z650 빈 손 → 기둥 4점(탐색 포함) → ArUco 보정 → 베이스 자세(로컬 로봇축 mm, 원점=관측 화면중심)."""
    set_stage("1 BASE")
    cur = PC.st()["tcp"]
    if max(abs(cur[i] - OBS[i]) for i in range(3)) > 2.0:
        goto_obs()
    Jinv, _mp = STG.load_map()
    px4, dxy = PC.find_base_4pts(False)                       # 부품(건강게이트·노출사다리·탐색 이동 포함)
    info = {"applied": False, "why": "탐색 이동으로 카메라가 관측자세를 벗어남 → 자 미적용"}
    if abs(dxy[0]) + abs(dxy[1]) < 0.5:
        px4, info = aruco_correct(px4)
    pose, rms, _ = STG.base_pose_robot(px4, Jinv, (0.0, 0.0), (640.0, 360.0))
    if abs(dxy[0]) + abs(dxy[1]) > 0.01:
        pose = HG.Pose2D(pose.x + dxy[0], pose.y + dxy[1], pose.yaw_deg)
    prev = jload(F["base_last"])
    d = None
    if prev:
        d = (pose.x - prev["x"], pose.y - prev["y"], HG.wrap_deg(pose.yaw_deg - prev["yaw"]))
        log(f"  베이스 이동(직전 {prev['made']} 대비): Δx {d[0]:+.2f} Δy {d[1]:+.2f} Δyaw {d[2]:+.3f}°")
    B = {"x": pose.x, "y": pose.y, "yaw": pose.yaw_deg, "rms": rms, "aruco": info, "px4": [list(p) for p in px4],
         "made": time.strftime("%Y-%m-%d %H:%M:%S"), "delta_prev": d}
    jsave(F["base_last"], B)
    log(f"  베이스 x {pose.x:.2f} y {pose.y:.2f} yaw {pose.yaw_deg:+.3f}° rms {rms:.2f}mm  ArUco {info}")
    with LOCK: S["base"] = B
    return B


def slot_target(color, B, grasp=None):
    """벽별 슬롯 기준(손 안착 TCP + 그때 베이스) 을 지금 베이스로 강체 이동 → 목표 (x, y, rz, z_seat).
    파지 편차(있으면) 는 상한 PRECORR_MAX_MM 안에서 선보정(부호 미검증 — 호버 정렬이 최종)."""
    ref = (jload(F["slot"]) or {}).get(color)
    if not ref:
        raise Gate(f"{color} 슬롯 기준 없음 — 손으로 안착 → 로봇으로 물고 '슬롯 기준 1/2' → 놓고 관측자세 → '슬롯 기준 2/2'")
    br = ref["base"]; seat = ref["seat_tcp"]
    dyaw = HG.wrap_deg(B["yaw"] - br["yaw"])
    sx, sy = seat[0] - br["x"], seat[1] - br["y"]
    c, s = math.cos(math.radians(dyaw)), math.sin(math.radians(dyaw))
    tx, ty = B["x"] + c * sx - s * sy, B["y"] + s * sx + c * sy
    rz = HG.wrap_deg(seat[5] + dyaw)
    pre = (0.0, 0.0, 0.0)
    if grasp and grasp.get("ok"):
        ac, al, da = grasp["across_mm"], grasp["along_mm"], grasp["dang"]
        n = math.hypot(ac, al)
        if n > PRECORR_MAX_MM:
            ac, al = ac * PRECORR_MAX_MM / n, al * PRECORR_MAX_MM / n
        cr, sr = math.cos(math.radians(rz)), math.sin(math.radians(rz))
        tx -= cr * ac - sr * al; ty -= sr * ac + cr * al
        if PRECORR_RZ:
            rz = HG.wrap_deg(rz - da)
        else:
            da = 0.0
        pre = (ac, al, da)
    T = {"x": tx, "y": ty, "rz": rz, "z_seat": seat[2], "dyaw": dyaw, "precorr": pre, "ref_made": ref["made"]}
    log(f"  목표 [{color}] x {tx:.2f} y {ty:.2f} rz {rz:+.2f} (베이스 Δyaw {dyaw:+.3f}°, 선보정 {pre[0]:+.2f}/{pre[1]:+.2f}mm {pre[2]:+.2f}°)")
    with LOCK: S["target"] = T
    return T


# ------------------------------------------------------------------ 2단계: 랙 양끝 → 중앙 → 벽별 보정 → 파지 → 서명 편차
def rack_axes(e, Jinv):
    """벽 축 단위벡터(로봇 프레임): along = p1→p2, across = 그 수직."""
    u = np.array([e["p2"][0] - e["p1"][0], e["p2"][1] - e["p1"][1]], float)
    u = Jinv @ u; u = u / (np.linalg.norm(u) + 1e-9)
    return u, np.array([-u[1], u[0]])


def stage_rack(color, teach_rack=False):
    set_stage("2 RACK", color=color)
    rr = (jload(F["rack"]) or {}).get(color)
    if not rr:
        raise Gate(f"{color} 랙 기준 없음")
    Jinv = np.array(jload(F["rack_map"])["Jinv_mm_per_px"], float)
    rp = jload(F["rack_pose"])["tcp"]
    up_to_safe(); PC.speed(SPD_MOVE)
    move([rp[0], rp[1], SAFE_Z, 180.0, 0.0, 180.0], tag="랙 위 SAFE")
    log(f"  그리퍼 열기 → {PC.gripper(rr.get('grip_open', GRIP_OPEN))}")
    move(rp, tag="랙 관측자세")
    e = PC.rack_ends(color, x_hint=rr["Pc0"][0])                # 부품(4프레임, 같은 색 여러 벽이면 x 로 선택)
    if not e:
        raise Gate("랙에서 벽 양끝을 못 잡음 — 벽이 뒤집혔거나 점 가림. 랙에 다시 놓기")
    L0 = rr["len0_px"]
    if abs(e["len_px"] - L0) > 0.10 * L0:
        raise Gate(f"벽 길이 불일치 {e['len_px']:.0f}px vs 기준 {L0:.0f}px ({(e['len_px']-L0)/L0*100:+.0f}%) — 끝점 미검출, 파지 금지")
    dang = HG.wrap_deg(e["ang"] - rr["ang0"])
    if abs(dang) > RACK_ANG_MAX:
        raise Gate(f"랙 위 벽 각 변화 {dang:+.2f}° > {RACK_ANG_MAX}° — 벽이 삐뚤게 놓임")
    dmm = Jinv @ np.array([e["mid"][0] - rr["Pc0"][0], e["mid"][1] - rr["Pc0"][1]])
    gx, gy = rr["Tg0"][0] - dmm[0], rr["Tg0"][1] - dmm[1]         # 검증된 부호(rack_grip_xy 와 동일)
    ua, ux = rack_axes(e, Jinv)
    off = rr.get("offset") or {"along": 0.0, "across": 0.0}
    gx += off["along"] * ua[0] + off["across"] * ux[0]
    gy += off["along"] * ua[1] + off["across"] * ux[1]
    rz = 180.0
    if RACK_RZ_FOLLOW:
        rz = HG.wrap_deg(180.0 + dang)
    rot = [180.0, 0.0, rz]
    R = {"mid": e["mid"], "len_px": e["len_px"], "ang": e["ang"], "dang": dang, "n_dots": e["n_dots"], "grip_xy": (gx, gy), "offset": off, "made": time.strftime("%H:%M:%S")}
    with LOCK: S["rack"] = R
    log(f"  랙: 중앙 ({e['mid'][0]:.0f},{e['mid'][1]:.0f}) 길이 {e['len_px']:.0f}px 각 {e['ang']:+.2f}°(Δ{dang:+.2f}) → 파지 XY ({gx:.2f},{gy:.2f}) 보정 along {off['along']:+.2f} across {off['across']:+.2f}")
    zp = rr["z_pick"]
    PC.speed(SPD_MOVE); move([gx, gy, rp[2]] + rot, tag="파지 XY 위")
    PC.speed(SPD_DESC); move([gx, gy, zp + 40] + rot, tag="픽 −40")
    PC.speed(SPD_SEAT); move([gx, gy, zp] + rot, tol=0.8, tag="픽 자세")
    if teach_rack:
        # 사용자 조그로 진짜 중앙 → '랙 보정 저장' 버튼(현재 TCP − 계산 TCP 를 벽 축으로 분해해 저장) → 계속
        with LOCK: S["rack"]["calc_xy"] = (gx, gy); S["rack"]["axes"] = (ua.tolist(), ux.tolist())
        wait_user("랙 파지 티칭: 그리퍼를 벽 진짜 중앙으로 조그한 뒤 [랙 보정 저장] → 계속")
        cur = PC.st()["tcp"]; gx, gy = cur[0], cur[1]
    g_close = rr["grip_close"]
    gr = PC.gripper(g_close); log(f"  그리퍼 닫기 {g_close} → 실측 {gr}")
    ok, why = PC._grip_ok(gr, g_close, color)                     # 부품(실측>닫힘 또는 손목캠 벽 점)
    if not ok:
        raise Gate("빈 파지 의심(" + why + ")")
    log("  파지 판정 OK: " + why)
    PC.speed(SPD_DESC); move([gx, gy, zp + 125] + rot, tag="들어올림")
    log(f"  (참고) 들어올린 뒤 그리퍼 {PC.grip_read()}")
    return grasp_check(color, dang, g_close, gr)


def grasp_check(color, rack_dang, g_close, gr):
    """든 벽 점 ↔ 좋은 파지 서명. 서명 없으면 사용자 확인 후 저장(버튼). 게이트 1.0mm/0.3°."""
    if PC.load_grasp_ref(color) is None:
        wait_user(f"{color} 좋은 파지 서명 없음: 파지가 정상이면 [파지 서명 저장] → 계속 (아니면 중단)")
    g, info = PC.grasp_measure(color, rack_dang=rack_dang)      # 부품(2점/1점, 축척 핀홀 상수)
    if g is None:
        raise Gate("파지 편차 측정 불가: " + str(info))
    G = {"ok": True, "across_mm": info["across_mm"], "along_mm": info["along_mm"], "dang": info["dang"], "how": info["how"], "grip": gr}
    with LOCK: S["grasp"] = G
    log(f"  파지 편차({info['how']}): 가로 {info['across_mm']:+.2f} 길이 {info['along_mm']:+.2f}mm 각 {info['dang']:+.2f}°")
    if abs(info["along_mm"]) > GRASP_GATE_MM or abs(info["across_mm"]) > GRASP_GATE_MM or abs(info["dang"]) > GRASP_GATE_DEG:
        # 13:27 실기: 위치 −0.1mm 인데 각 +0.44° 로 정지(랙 위 벽이 −0.55° 돌아 있고 파지 rz 180 고정). z440 정렬이 회전을 보정하므로
        # 즉시 정지 대신 사용자 선택: [▶계속]=이 파지로 진행(z440 에서 보정) / [⛔중단]=정지(벽은 사용자가 랙으로).
        wait_user(f"파지 편차 게이트 초과({GRASP_GATE_MM}mm/{GRASP_GATE_DEG}°): 가로 {info['across_mm']:+.2f} 길이 {info['along_mm']:+.2f}mm 각 {info['dang']:+.2f}° — "
                  f"[▶계속]=이 파지로 진행(z440 정렬이 보정) / [⛔중단]=정지 후 재파지")
        G["gate_override"] = True
    return G


# ------------------------------------------------------------------ 2'단계: 운반 → z_seat+85 → 호버 정렬(기둥점↔벽점 기준 관계)
def stage_carry_hover(color, T):
    set_stage("2' CARRY", color=color)
    rot = [180.0, 0.0, T["rz"]]
    up_to_safe(); PC.speed(SPD_MOVE)
    move([T["x"], T["y"], SAFE_Z] + rot, tag="목표 위 SAFE(rz 정렬)")
    log(f"  (참고) 운반 후 그리퍼 {PC.grip_read()}")
    PC.speed(SPD_DESC); move([T["x"], T["y"], HOVER_Z] + rot, tag="호버 z478")
    zh = T["z_seat"] + 85.0
    PC.speed(SPD_SEAT); move([T["x"], T["y"], zh] + rot, tag=f"기둥 위 z{zh:.0f}")
    set_stage("2' HOVER ALIGN", color=color)
    if HA.load_ref(color) is None:
        wait_user(f"{color} z{zh:.0f} 기준 없음: 콘솔 조그로 육안 정렬 후 [z440 기준 저장] → 계속")
    A = {"done": False}
    try:
        srcs = ALIGN_SRCS.get(color)
        if srcs:
            try:
                _, per, _ = HA.check(color)                         # 참고: 전 카메라 측정치 로그
                for D in per.values(): log(f"  (참고) {HA.fmt(D)}")
            except Exception as ex: log(f"  (참고 측정 실패: {ex})")
            log(f"  정렬 카메라: {srcs}")
        HA.align(color, srcs=srcs)                             # 부품(지정 카메라, 수렴/발산/불일치 게이트)
        A["done"] = True
    finally:
        HA.restore_expo()
    cur = PC.st()["tcp"]
    A.update(x=cur[0], y=cur[1], rz=cur[5], z=cur[2], made_t=time.time(), made=time.strftime("%H:%M:%S"))
    with LOCK: S["align"] = A
    log(f"  정렬 후 TCP x {cur[0]:.2f} y {cur[1]:.2f} rz {cur[5]:+.2f} z {cur[2]:.1f}")
    return A


# ------------------------------------------------------------------ 3단계: 사용자 버튼 하강(막힘 감시) → 안착 판정 → (seated 만) 승격·놓기
def descend_gate(color):
    """②하강 직전 게이트. 정렬 완료 상태 + 같은 색 + 정렬 후 15분 이내 + z 일치 + 정렬 TCP 와 XY/rz 일치."""
    with LOCK:
        T = S.get("target"); A = S.get("align"); col = S.get("color")
    cur0 = PC.st()["tcp"]
    if not T:                                                   # 서버 재시작 등으로 목표가 없으면 슬롯 기준의 z_seat 만 가져온다(사용자 수동 정렬 전제)
        ref = (jload(F["slot"]) or {}).get(color)
        if not ref:
            raise Gate(f"{color} 슬롯 기준 없음 — 하강 불가")
        T = {"x": None, "y": None, "rz": None, "z_seat": ref["seat_tcp"][2], "user_fallback": True}
        with LOCK: S["target"] = T; S["color"] = color
        col = color
        log(f"  목표 없음 → 슬롯 기준 z_seat {T['z_seat']:.1f} 만 사용(사용자 수동 정렬)")
    if not (A and A.get("done")):
        # 13:3x 실기: 카메라 불일치로 정렬이 안 돈 상태에서 사용자가 육안으로 맞춤 → 설계 3단계(사용자 허락 후 버튼 하강) 그대로,
        # 지금 TCP 를 '사용자 정렬 완료' 로 간주. 단 z 는 z_seat+85 ±3 이어야.
        if not (T["z_seat"] - 1.0 <= cur0[2] <= T["z_seat"] + 88.0):
            raise Gate(f"정렬 상태 없음 + 지금 z{cur0[2]:.0f} 가 z{T['z_seat']+3:.0f}~{T['z_seat']+88:.0f} 밖 — 수동 정렬이면 z440(또는 채널 안)에서 누르세요")
        A = {"done": True, "by": "user", "x": cur0[0], "y": cur0[1], "z": cur0[2], "rz": cur0[5], "made_t": time.time(), "made": time.strftime("%H:%M:%S")}
        with LOCK: S["align"] = A
        log(f"  정렬 상태 없음 → 사용자 수동 정렬 TCP ({cur0[0]:.2f},{cur0[1]:.2f}) rz{cur0[5]:+.2f} 를 정렬 완료로 간주")
    if T.get("x") is not None:                                  # 계산 목표 대비 실제 하강 자리 차이를 항상 기록(반복되면 보정값 후보)
        nd = (cur0[0] - T["x"], cur0[1] - T["y"], HG.wrap_deg(cur0[5] - T["rz"]))
        rec = {"made": time.strftime("%Y-%m-%d %H:%M:%S"), "color": color, "by": A.get("by", "align"), "target": [T["x"], T["y"], T["rz"]],
               "descend_tcp": [round(v, 3) for v in cur0], "descend_minus_target": [round(v, 3) for v in nd], "precorr": T.get("precorr"), "dyaw": T.get("dyaw")}
        try:
            open(os.path.join(STATE, "nudge_log.jsonl"), "a").write(json.dumps(rec, ensure_ascii=False) + "\n")
        except Exception: pass
        log(f"  하강 자리 − 계산 목표: dx {nd[0]:+.2f} dy {nd[1]:+.2f} mm drz {nd[2]:+.2f}° ({A.get('by', 'align')}) → nudge_log")
    if col != color:
        raise Gate(f"하강 색 불일치: 정렬된 벽은 {col}, 요청은 {color}")
    age = time.time() - float(A.get("made_t") or 0.0)
    if age > ALIGN_MAX_AGE_S:
        raise Gate(f"정렬 후 {age/60:.0f}분 경과(> {ALIGN_MAX_AGE_S//60}분) — 베이스가 움직였을 수 있음, 사이클 다시(재정렬)")
    cur = PC.st()["tcp"]
    if not (T["z_seat"] - 1.0 <= cur[2] <= T["z_seat"] + 88.0):        # 막힘 후퇴 뒤 재하강·안착 높이(사용자 조그) 허용: z_seat−1 ~ +88
        raise Gate(f"지금 z{cur[2]:.0f} 가 z{T['z_seat']+3:.0f}~{T['z_seat']+88:.0f} 밖 — 정렬 후 움직였음, 사이클 다시")
    dxy = math.hypot(cur[0] - A["x"], cur[1] - A["y"]); drz = abs(HG.wrap_deg(cur[5] - A["rz"]))
    if dxy > ALIGN_GATE_MM or drz > ALIGN_GATE_DEG:
        raise Gate(f"정렬 TCP 와 불일치 ΔXY {dxy:.2f}mm Δrz {drz:.2f}° (허용 {ALIGN_GATE_MM}mm/{ALIGN_GATE_DEG}°) — 정렬 후 움직였음, 사이클 다시")
    log(f"  하강 게이트 OK: ΔXY {dxy:.2f}mm Δrz {drz:.2f}° 정렬 {age/60:.1f}분 전")
    return T, A, cur


def seat_check(color=None):
    """안착 판정. ①seat_verify(손목캠 밑판 사각형 — z355 에선 원리적으로 못 봄, 14:03 실기 2회 unknown)
    ②새카메라: 안착 기준 사진(pose_refs/<색>.json seat.cams.newcam.dots, 12:35 사용자 손 안착)의 기둥 점 자리와 지금 기둥 점 비교.
    카메라가 손목에 붙어 있으므로 '안착 TCP ↔ 베이스' 관계가 맞으면 기둥이 같은 px 에 온다. 실패해도 예외 대신 unknown."""
    try:
        import seat_verify as SV
        seat = SV.verify()
    except Exception as ex:
        seat = {"state": "unknown", "why": str(ex)}
    seat = {k: v for k, v in (seat or {}).items() if not str(k).startswith("_")}
    if str(seat.get("state", "")).startswith("seated") or not color:
        return seat
    try:
        pr = jload(os.path.join(STATE, "pose_refs", f"{color}.json")) or {}
        dots = (((pr.get("seat") or {}).get("cams") or {}).get("newcam") or {}).get("dots") or {}
        wall_c = "red" if color == "red_s" else color
        refs = [(c, p[0], p[1]) for c, lst in dots.items() if c != wall_c for p in (lst or []) if len(p) >= 2]
        if not refs:
            seat["why"] = str(seat.get("why")) + " | 새카메라 안착 기준 없음"; return seat
        img = HA.grab("newcam")
        res = []
        for c, rx, ry in refs:
            now = HA._blobs(img, c, 20, "newcam")
            cand = [q for q in now if math.hypot(q[0] - rx, q[1] - ry) <= 60]
            if cand:
                q = min(cand, key=lambda q: math.hypot(q[0] - rx, q[1] - ry)); res.append((c, round(rx), round(ry), round(q[0]), round(q[1]), math.hypot(q[0] - rx, q[1] - ry)))
        if not res:
            seat["why"] = str(seat.get("why")) + f" | 새카메라: 기준 기둥 {[(c, round(x), round(y)) for c, x, y in refs]} 근처(60px)에 점 없음"; return seat
        worst = max(r[5] for r in res)
        desc = " ".join(f"{c}({rx},{ry})→({x},{y}) {d:.1f}px" for c, rx, ry, x, y, d in res)
        if worst <= SEAT_NEWCAM_TOL_PX:
            return {"state": "seated_newcam", "why": f"새카메라 기둥 점이 안착 기준 자리와 일치: {desc} (허용 {SEAT_NEWCAM_TOL_PX}px)", "px": worst}
        return {"state": "unknown", "why": f"새카메라 기둥 점이 안착 기준에서 {worst:.1f}px (> {SEAT_NEWCAM_TOL_PX}): {desc}", "px": worst}
    except Exception as ex:
        seat["why"] = str(seat.get("why")) + f" | 새카메라 판정 실패: {ex}"
        return seat


def promote_slot_ref(color, seat_tcp, B, note="자동 승격: 안착 성공 TCP + 이번 사이클 베이스"):
    """③안착 성공한 TCP + 이번 사이클 베이스 측정을 slot_ref 로 승격. 이전 값은 history(최대 SLOT_HISTORY_MAX) 로 보존."""
    if not B:
        log("  ⚠ 슬롯 기준 승격 생략: 이번 사이클 베이스 측정이 없음"); return False
    d = jload(F["slot"]) or {}
    prev = d.get(color)
    hist = []
    if prev:
        hist = [{k: v for k, v in prev.items() if k != "history"}] + list(prev.get("history") or [])
    d[color] = {"seat_tcp": [round(float(v), 3) for v in seat_tcp], "base": {"x": B["x"], "y": B["y"], "yaw": B["yaw"]},
                "base_rms": B.get("rms"), "made": time.strftime("%Y-%m-%d %H:%M"), "note": note, "history": hist[:SLOT_HISTORY_MAX]}
    jsave(F["slot"], d)
    log(f"✅ 슬롯 기준 승격 [{color}] seat {[round(v, 1) for v in seat_tcp[:3]]} rz {seat_tcp[5]:+.2f} ↔ 베이스 ({B['x']:.2f},{B['y']:.2f},{B['yaw']:+.3f}°)  history {len(hist[:SLOT_HISTORY_MAX])}건")
    return True


def release_and_rise(color, rr):
    set_stage("3 RELEASE", color=color)
    log(f"  그리퍼 열기 → {PC.gripper(rr.get('grip_open', GRIP_OPEN))}")
    cur = PC.st()["tcp"]
    PC.speed(SPD_SEAT); move([cur[0], cur[1], cur[2] + 30] + list(cur[3:]), tag="수직 +30")
    PC.speed(SPD_DESC); move([cur[0], cur[1], HOVER_Z] + list(cur[3:]), tag="호버 z478")
    PC.speed(SPD_MOVE); move([cur[0], cur[1], SAFE_Z] + list(cur[3:]), tag="SAFE"); PC.speed(1)


def stage_descend(color):
    """③하강 → 안착 판정. seated: promote_ref + 슬롯 기준 승격 + 개방 + 상승.
    아니면 ①그리퍼 유지·그 자리 정지(SEAT FAIL) → [⛔중단] 또는 [▶계속](사용자 육안 확인 → 개방·상승, 승격 없음). 반환: seated."""
    T, A, cur = descend_gate(color)
    set_stage("3 DESCEND", color=color)
    rr = (jload(F["rack"]) or {}).get(color) or {}
    PC.descend_monitored(color, cur[0], cur[1], [180.0, 0.0, cur[5]], T["z_seat"], rr.get("grip_close", 13))   # 부품(3mm 단계·벽점 밀림·놓침·정체 → stop+25mm)
    set_stage("3 SEAT CHECK", color=color)
    seat = seat_check(color)
    seated = str(seat.get("state", "")).startswith("seated")
    with LOCK: S["seat"] = seat
    log(f"  안착 판정: {seat}")
    if not seated:
        set_stage("SEAT FAIL", color=color)
        log(f"🛑 안착 미확인({seat.get('state')}: {seat.get('why')}) — 그리퍼 유지, 그 자리 정지. "
            "[⛔중단]=이대로 정지(벽은 사용자가 처리) / [▶계속]=사용자가 안착을 육안 확인 → 그리퍼 열고 상승(기준 승격 없음)")
        wait_user("SEAT FAIL — 그리퍼 유지, 사용자 판단 대기: [⛔중단] 또는 [▶계속](안착 육안 확인 시)")
        log("  사용자 [▶계속]: 안착 육안 확인으로 간주 → 개방·상승 (기준 승격 없음)")
        with LOCK: S["seat"] = dict(seat, user_override=True)
    else:
        at = PC.st()["tcp"]
        if not HA.promote_ref(color):                               # 성공 사이클 정렬 상태 → 다음 z440 기준
            log("  (z440 기준 승격 없음: 이번 정렬의 마지막 측정이 없음)")
        with LOCK: B = S.get("base")
        promote_slot_ref(color, [at[0], at[1], T["z_seat"], at[3], at[4], at[5]], B)   # z 는 티칭값 유지(접촉 조기정지 z 승격 시 위로 표류 방지)
    release_and_rise(color, rr)
    set_stage("DONE" if seated else "DONE (안착 미확인·사용자 개방)", color=color)
    return seated


def stage_descend_reteach(color):
    """③' 하강 → 안착 판정 → seated 면 **든 채로** z_seat+85 로 올려 z440 기준(손목·새카메라)을 이 자리에서 재촬영 → 다시 내려 놓고 상승.
    13:4x 사용자 지시: 정렬 루프가 안 돈 채 사용자가 육안으로 맞춘 자리 → 성공하면 그 관계를 기준 사진으로 박아 다음부터 카메라가 재현."""
    T, A, cur = descend_gate(color)
    rr = (jload(F["rack"]) or {}).get(color) or {}
    if cur[2] <= T["z_seat"] + 2.0:
        log(f"  이미 안착 높이 z{cur[2]:.1f}(사용자 조그) → 하강 생략, 안착 판정부터")
    else:
        set_stage("3 DESCEND", color=color)
        PC.descend_monitored(color, cur[0], cur[1], [180.0, 0.0, cur[5]], T["z_seat"], rr.get("grip_close", 13))
    set_stage("3 SEAT CHECK", color=color)
    seat = seat_check(color)
    seated = str(seat.get("state", "")).startswith("seated")
    with LOCK: S["seat"] = seat
    log(f"  안착 판정: {seat}")
    if not seated:
        set_stage("SEAT FAIL", color=color)
        log(f"🛑 안착 미확인({seat.get('state')}: {seat.get('why')}) — 그리퍼 유지, 그 자리 정지. "
            "[⛔중단]=정지 / [▶계속]=사용자가 안착 육안 확인 → 이 자리를 성공으로 간주하고 기준 재촬영 진행")
        wait_user("SEAT FAIL — 그리퍼 유지: [⛔중단] 또는 [▶계속](안착 육안 확인 시 → 기준 재촬영)")
        log("  사용자 [▶계속]: 안착 육안 확인 → 기준 재촬영 진행")
        with LOCK: S["seat"] = dict(seat, user_override=True)
    at = PC.st()["tcp"]
    with LOCK: B = S.get("base")
    if not B:
        B = jload(F["base_last"]); log(f"  베이스: base_last({(B or {}).get('made')}) 사용")
    seat_tcp = [at[0], at[1], T["z_seat"], at[3], at[4], at[5]]
    promote_slot_ref(color, seat_tcp, B, note="사용자 육안 정렬 자리에서 안착 성공 → 슬롯 기준 갱신")
    _pending_seat[color] = list(seat_tcp); jsave(_PENDING_F, _pending_seat)
    set_stage("3' RETEACH z440", color=color)
    zh = T["z_seat"] + 85.0
    PC.speed(SPD_SEAT); move([at[0], at[1], zh] + list(at[3:]), tag=f"든 채 z{zh:.0f} (기준 재촬영)")
    time.sleep(0.8)
    for src in ("wrist", "newcam"):
        try:
            teach_hover(color, src)
        except Exception as ex:
            log(f"  ⚠ z440 기준 재촬영 실패 [{src}]: {ex}")
    set_stage("3' RE-DESCEND", color=color)
    PC.descend_monitored(color, at[0], at[1], [180.0, 0.0, at[5]], T["z_seat"], rr.get("grip_close", 13))
    seat2 = seat_check(color); log(f"  재안착 판정: {seat2}")
    release_and_rise(color, rr)
    set_stage("DONE (기준 재촬영)", color=color)
    return True


# ------------------------------------------------------------------ 사이클
def run_cycle(color, teach_rack=False):
    global _logf
    if _logf:
        try: _logf.close()
        except Exception: pass
    _logf = open(os.path.join(STATE, "logs", f"cycle_{color}_{time.strftime('%m%d_%H%M%S')}.log"), "a")
    with LOCK:
        S.update(color=color, err=None, base=None, rack=None, grasp=None, target=None, align=None, seat=None)
    B = stage_base()                                               # 빈손 베이스 재측정(벽마다)
    slot_target(color, B)                                          # 기준 없으면 여기서 정지(픽 전에 안다)
    G = stage_rack(color, teach_rack)
    T = slot_target(color, B, G)
    stage_carry_hover(color, T)
    set_stage("WAIT DESCEND", wait="[⬇ 하강] 버튼 (x·y·yaw 확인 후)", color=color)


def run_held(color):
    """든 채로 3단계부터(13:27 실기: 파지 게이트 정지 후 사용자 '잘 잡았다' → 재파지 없이 운반·z440 정렬·하강 대기).
    필요: 이번 사이클 베이스(base_last, 20분 이내) + 파지 편차(메모리 S 또는 재시작 전 저장한 last_state) + 그리퍼가 벽을 물고 있음."""
    with LOCK:
        G = S.get("grasp"); S.update(color=color, err=None, target=None, align=None, seat=None)
    if not G:
        ls = jload(os.path.join(STATE, "last_state_1327.json")) or {}
        G = ls.get("grasp"); log(f"  파지 편차: 재시작 전 저장분 사용 {G}")
    if not G:
        raise Gate("파지 편차 기록 없음 — [▶사이클] 로 처음부터")
    B = jload(F["base_last"])
    if not B:
        raise Gate("베이스 측정 없음 — [▶사이클] 로 처음부터")
    age = time.time() - time.mktime(time.strptime(B["made"], "%Y-%m-%d %H:%M:%S"))
    if age > 20 * 60:
        raise Gate(f"베이스 측정이 {age/60:.0f}분 전 — [▶사이클] 로 처음부터(빈손 재측정)")
    rr = (jload(F["rack"]) or {}).get(color) or {}
    g = PC.grip_read()
    if g.isdigit() and int(g) <= rr.get("grip_close", 13):
        raise Gate(f"그리퍼 {g} ≤ 닫힘값 — 벽을 물고 있지 않음")
    with LOCK: S["base"] = B; S["grasp"] = G
    log(f"══ 든 채로 3단계부터 [{color}]: 베이스 {B['made']} 파지 편차 가로 {G['across_mm']:+.2f} 길이 {G['along_mm']:+.2f} 각 {G['dang']:+.2f}°")
    T = slot_target(color, B, G)
    stage_carry_hover(color, T)
    set_stage("WAIT DESCEND", wait="[⬇ 하강] 버튼 (x·y·yaw 확인 후)", color=color)


def align_here(color):
    """든 채 z440 근처(±12)에서 정렬만 다시(SAFE 왕복 없음): z_seat+85 로 수직 이동 → ALIGN_SRCS 카메라로 정렬 → WAIT DESCEND."""
    ref = (jload(F["slot"]) or {}).get(color)
    if not ref:
        raise Gate(f"{color} 슬롯 기준 없음")
    zs = ref["seat_tcp"][2]; zh = zs + 85.0
    cur = PC.st()["tcp"]
    if abs(cur[2] - zh) > 12.0:
        raise Gate(f"지금 z{cur[2]:.0f} — z{zh:.0f}±12 에서만(든 채)")
    g = PC.grip_read(); rr = (jload(F["rack"]) or {}).get(color) or {}
    if g.isdigit() and int(g) <= rr.get("grip_close", 13):
        raise Gate(f"그리퍼 {g} ≤ 닫힘값 — 벽을 물고 있지 않음")
    with LOCK:
        S.update(color=color, err=None, align=None, seat=None)
        if not S.get("target"):
            S["target"] = {"x": None, "y": None, "rz": None, "z_seat": zs, "user_fallback": True}
    set_stage("2' HOVER ALIGN(재)", color=color)
    PC.speed(SPD_SEAT); move([cur[0], cur[1], zh] + list(cur[3:]), tag=f"z{zh:.0f}")
    A = {"done": False}
    with LOCK: S["align"] = A
    srcs = ALIGN_SRCS.get(color)
    try:
        _, per, _ = HA.check(color)
        for D in per.values(): log(f"  (참고) {HA.fmt(D)}")
    except Exception as ex: log(f"  (참고 측정 실패: {ex})")
    log(f"  정렬 카메라: {srcs or '가용 전부'}")
    HA.align(color, srcs=srcs)
    c = PC.st()["tcp"]
    A.update(done=True, x=c[0], y=c[1], rz=c[5], z=c[2], made_t=time.time(), made=time.strftime("%H:%M:%S"), by="align")
    with LOCK: S["align"] = A
    log(f"  ✅ 정렬 완료 TCP ({c[0]:.2f},{c[1]:.2f}) rz{c[5]:+.2f}")
    set_stage("WAIT DESCEND", wait="[⬇ 하강] 버튼 (x·y·yaw 확인 후)", color=color)


def run_multi(order=RUN4_ORDER):
    """④4벽 연속. 벽마다 run_cycle(빈손 베이스 재측정 포함) → [⬇ 하강] 버튼 대기 → stage_descend.
    어느 벽이든 Gate/Abort/예외/안착 미확인이면 그 자리에서 전체 중단(다음 벽 진행 없음)."""
    order = [c for c in order if c in COLORS]
    with LOCK:
        S["run4"] = {"order": order, "idx": 0, "done": [], "active": True}
    log(f"══ run4 시작: {' → '.join(order)} (벽마다 빈손 베이스 재측정, 하강은 매번 버튼)")
    try:
        for i, color in enumerate(order):
            with LOCK: S["run4"]["idx"] = i
            if i > 0:
                g = PC.grip_read(); gc = ((jload(F["rack"]) or {}).get(order[i - 1]) or {}).get("grip_close", 13)
                if g.isdigit() and int(g) <= gc:
                    raise Gate(f"run4: 그리퍼 {g} ≤ 닫힘값 {gc} — 빈손이 아님, [{color}] 진행 금지")
            log(f"══ run4 {i+1}/{len(order)} [{color}] — 빈손 베이스 재측정부터")
            run_cycle(color)
            wait_user(f"[⬇ 하강] 버튼 (x·y·yaw 확인 후) — run4 {i+1}/{len(order)} {color}")
            if not stage_descend(color):
                raise Gate(f"run4 중단: [{color}] 안착 미확인(사용자 개방) — 남은 {order[i+1:]} 진행 안 함")
            with LOCK: S["run4"]["done"].append(color)
        set_stage("RUN4 DONE")
        log(f"✅ run4 완료: {order}")
    finally:
        with LOCK:
            if S.get("run4"): S["run4"]["active"] = False


# ------------------------------------------------------------------ 티칭 버튼
_PENDING_F = os.path.join(STATE, "pending_seat.json")      # 1/2 는 서버 재시작에도 살아남게 파일로(12:48 재시작 때 유실 경험)
_pending_seat = jload(_PENDING_F) or {}


def teach_slot_tcp(color):
    """1/2: 로봇이 손 안착 벽을 물고 있는 지금 TCP 저장."""
    cur = PC.st()["tcp"]; _pending_seat[color] = list(cur); jsave(_PENDING_F, _pending_seat)
    log(f"  슬롯 기준 1/2 [{color}] 안착 TCP {[round(v, 2) for v in cur]} (다음: 놓고 관측자세 → 2/2)")


def teach_slot_base(color):
    """2/2: 관측자세에서 베이스 측정 → 1/2 의 TCP 와 페어링. ArUco 기준이 없으면 지금 마커로 생성."""
    seat = _pending_seat.get(color)
    if not seat:
        raise Gate("먼저 [슬롯 기준 1/2] 로 안착 TCP 를 저장")
    cur = PC.st()["tcp"]
    if max(abs(cur[i] - OBS[i]) for i in range(3)) > 2.0:      # 13:16 사고: 관측자세 밖에서 ArUco 기준을 찍어 자가 41mm/축척 0.885 로 어긋남 → 베이스 rms 12.6mm
        goto_obs()
    if not os.path.exists(F["aruco"]):
        m, why = BT.markers()
        if m and len(m) >= 3:
            jsave(F["aruco"], {"markers": {str(k): v for k, v in m.items()}, "tcp": PC.st()["tcp"], "made": time.strftime("%Y-%m-%d %H:%M")})
            log(f"  ArUco 기준 저장(마커 {sorted(m)})")
        else:
            log(f"  ⚠ ArUco 기준 미생성: {why or f'마커 {len(m)}개(<3)'} — 자 보정 없이 진행")
    B = stage_base()
    d = jload(F["slot"]) or {}
    d[color] = {"seat_tcp": seat, "base": {"x": B["x"], "y": B["y"], "yaw": B["yaw"]}, "base_rms": B["rms"], "made": time.strftime("%Y-%m-%d %H:%M")}
    jsave(F["slot"], d); _pending_seat.pop(color, None); jsave(_PENDING_F, _pending_seat)
    log(f"✅ 슬롯 기준 저장 [{color}] seat {[round(v, 1) for v in seat[:3]]} rz {seat[5]:+.1f} ↔ 베이스 ({B['x']:.1f},{B['y']:.1f},{B['yaw']:+.2f}°)")


def teach_slot_both(color):
    """⑤슬롯 기준 1/2 + 2/2 한 버튼. 1/2 는 벽을 물고 있어야(그리퍼 > 닫힘값 그리고 손목캠 벽 점 ≥1) — 빈손이면 거부.
    1/2 저장 → 사용자가 그리퍼 열고 벽에서 빼낸 뒤 [▶계속] → (그리퍼 열림 확인) → 관측자세 베이스 측정 → 2/2 저장."""
    set_stage("TEACH SLOT 1/2", color=color)
    gc = ((jload(F["rack"]) or {}).get(color) or {}).get("grip_close", 13)
    g = PC.grip_read()
    if g.isdigit() and int(g) <= gc:
        raise Gate(f"슬롯 기준 1/2 거부: 그리퍼 {g} ≤ 닫힘값 {gc} — 벽을 물고 있지 않음(빈손)")
    if not PC.held_wall_dots(color):
        raise Gate(f"슬롯 기준 1/2 거부: 손목캠에 든 {color} 벽 점 0 — 벽을 물고 있어야 함(빈손)")
    teach_slot_tcp(color)
    wait_user(f"[{color}] 1/2 저장됨 — 그리퍼를 열어 벽을 놓고 벽에서 빼낸 뒤 [▶계속] (로봇이 관측자세로 올라가 2/2 측정)")
    g = PC.grip_read()
    if g.isdigit() and int(g) <= gc:
        raise Gate(f"슬롯 기준 2/2 거부: 그리퍼 {g} 아직 닫힘(≤ {gc}) — 벽을 놓고 빼낸 뒤 [슬롯 기준 2/2] 로 마무리 (1/2 는 저장돼 있음)")
    set_stage("TEACH SLOT 2/2", color=color)
    teach_slot_base(color)


def teach_rack_offset(color):
    """WAIT(랙 티칭) 중: 현재 TCP − 계산 TCP 를 벽 축(along/across) 으로 분해해 저장."""
    with LOCK:
        R = S.get("rack") or {}
    if "calc_xy" not in R:
        raise Gate("랙 티칭 대기 상태가 아님")
    cur = PC.st()["tcp"]; d = np.array([cur[0] - R["calc_xy"][0], cur[1] - R["calc_xy"][1]])
    ua, ux = np.array(R["axes"][0]), np.array(R["axes"][1])
    prev = R.get("offset") or {"along": 0.0, "across": 0.0}
    off = {"along": float(prev["along"] + d @ ua), "across": float(prev["across"] + d @ ux)}
    rr = jload(F["rack"]); rr[color]["offset"] = off; rr[color]["offset_made"] = time.strftime("%Y-%m-%d %H:%M")
    rr[color]["z_pick"] = cur[2]
    jsave(F["rack"], rr)
    log(f"✅ 랙 보정 저장 [{color}] along {off['along']:+.2f} across {off['across']:+.2f}mm (조그 {d[0]:+.2f},{d[1]:+.2f}) z_pick {cur[2]:.1f}")


def teach_grasp_sig(color):
    rr = (jload(F["rack"]) or {}).get(color) or {}
    pts = PC.save_grasp_sig_now(color, rr.get("grip_close", 13), PC.grip_read())      # 부품(by_color 스키마 → F["sig"])
    if not pts:
        raise Gate("손목캠에 든 벽 점이 없음")


def fixed_cam_seeds(src, color):
    """고정캠(newcam/side)의 든 벽 점 씨앗 자동 선택. hover_align.wall_dots 는 고정캠에서 씨앗 없이는 항상 [] 이므로
    (CLI 는 --wall x,y 를 받음) 여기서 고른다: 베이스 특징 면적 상한(feat_area hi)보다 큰 그 색 점 = 카메라에 가까운 든 벽 점.
    없으면 최대 점이 2위의 2배 이상일 때만 채택. 애매하면 후보 목록과 함께 거부(사용자가 hover_view 로 확인)."""
    img = HA.grab(src)
    d = HA.DET[src]
    pts = sorted([q for q in HA._blobs(img, color, d["amin_wall"], src) if q[2] >= d["amin_wall"]], key=lambda q: -q[2])
    if not pts:
        raise Gate(f"{src} 에 {color} 점이 하나도 없음(벽 든 채 z440 이어야 함)")
    big = [q for q in pts if q[2] > d["feat_area"][1]]
    if big:
        pick = big[:2]
    elif len(pts) == 1 or pts[0][2] >= 2.0 * pts[1][2]:
        pick = pts[:1]
    else:
        raise Gate(f"{src} 에 {color} 든 벽 점을 못 고름 — 후보 {[(round(q[0]), round(q[1]), int(q[2])) for q in pts[:5]]} (면적 상한 {d['feat_area'][1]}, hover_view :8775 로 확인)")
    seeds = [(float(q[0]), float(q[1])) for q in pick]
    log(f"  {src} 든 {color} 벽 점 씨앗 {[(round(x), round(y)) for x, y in seeds]} 면적 {[int(q[2]) for q in pick]} (후보 {len(pts)})")
    return seeds


def teach_hover(color, src="wrist"):
    seeds = None if src == "wrist" else fixed_cam_seeds(src, color)
    HA.save_ref(color, src, seeds)                                  # 부품
    log(f"✅ z440 기준 저장 [{color}/{src}]")


def probe_cam(src, color):
    if HA.SRC[src]["moving"] == "base":                             # 손목 부착 카메라(새카메라): 베이스 특징 추적
        log(f"  {src} 매핑 시작(손목 부착): 베이스 특징 ±10mm 조그 + rz ±2°")
        PC.speed(1); HA.probe_arm(src, color); return
    seeds = fixed_cam_seeds(src, color)
    log(f"  {src} 매핑 시작: 벽 점 {[(round(x), round(y)) for x, y in seeds]} ±10mm 조그")
    HA.probe_fixed(src, color, seeds)                               # 부품(J·rot_sign 실측 → cam2robot_<src>.json)


# ------------------------------------------------------------------ 명령 처리(워커 1개: 로봇을 움직이는 명령은 순차)
Q = queue.Queue()


def worker():
    while True:
        op, arg = Q.get()
        with LOCK:
            S["busy"] = True; S["err"] = None
        try:
            ABORT.clear()
            if op == "start": run_cycle(arg["color"], arg.get("teach_rack", False))
            elif op == "resume_held": run_held(arg["color"])
            elif op == "align_here": align_here(arg["color"])
            elif op == "descend": stage_descend(arg["color"])
            elif op == "descend_reteach": stage_descend_reteach(arg["color"])
            elif op == "goto_obs": set_stage("GOTO OBS"); goto_obs(); set_stage("IDLE")
            elif op == "slot2": set_stage("TEACH SLOT 2/2"); teach_slot_base(arg["color"]); set_stage("IDLE")
            elif op == "slot_both": teach_slot_both(arg["color"]); set_stage("IDLE")
            elif op == "run4": run_multi(arg.get("order") or RUN4_ORDER)
            elif op == "probe": set_stage(f"PROBE {arg['src']}"); probe_cam(arg["src"], arg["color"]); set_stage("IDLE")
        except Abort:
            try: PC.post("stop", {"dry_run": False})
            except Exception: pass
            set_stage("STOPPED", err="사용자 중단"); log("⛔ 중단 — 로봇 정지. 벽은 사용자가 처리")
        except Gate as g:
            try: PC.post("stop", {"dry_run": False})
            except Exception: pass
            set_stage("STOPPED", err=str(g)); log(f"🛑 게이트 정지: {g}")
        except Exception as ex:
            try: PC.post("stop", {"dry_run": False})
            except Exception: pass
            set_stage("STOPPED", err=str(ex)); log("❌ 예외 정지: " + str(ex)); traceback.print_exc()
        finally:
            try: PC.speed(1)
            except Exception: pass
            with LOCK: S["busy"] = False


def handle_cmd(q):
    op = q.get("op", [""])[0]; color = q.get("color", [None])[0]; src = q.get("src", ["wrist"])[0]
    if color is not None and color not in COLORS:
        return {"ok": False, "err": f"색 {color}?"}
    if op == "abort":
        ABORT.set(); RESUME.set()
        try: PC.post("stop", {"dry_run": False})
        except Exception as e: return {"ok": False, "err": f"stop 실패 {e}"}
        return {"ok": True}
    if op == "resume":
        RESUME.set(); return {"ok": True}
    with LOCK:
        r4 = S.get("run4"); in_run4_wait = bool(r4 and r4.get("active") and S.get("stage") == "WAIT DESCEND" and S.get("wait"))
    if op == "descend" and in_run4_wait:                             # ④run4 는 워커가 점유 중 → 하강 버튼 = 계속
        RESUME.set(); return {"ok": True, "note": "run4: 하강 진행"}
    if op in ("start", "descend", "goto_obs", "slot2", "probe", "run4", "slot_both", "resume_held", "descend_reteach", "align_here"):
        if S["busy"]:
            return {"ok": False, "err": "실행 중 — 먼저 중단"}
        order = [c for c in (q.get("order", [""])[0] or "").split(",") if c] or list(RUN4_ORDER)
        if op == "run4" and any(c not in COLORS for c in order):
            return {"ok": False, "err": f"run4 순서 {order}?"}
        Q.put((op, {"color": color, "src": src, "order": order, "teach_rack": q.get("teach", ["0"])[0] == "1"})); return {"ok": True}
    try:                                                            # 로봇 이동 없는 즉시 명령
        if op == "slot1": teach_slot_tcp(color)
        elif op == "rack_offset": teach_rack_offset(color)
        elif op == "sig": teach_grasp_sig(color)
        elif op == "hover_ref": teach_hover(color, src)
        else: return {"ok": False, "err": f"op {op}?"}
        return {"ok": True}
    except Exception as ex:
        log(f"⚠ {op}: {ex}"); return {"ok": False, "err": str(ex)}


def snapshot():
    with LOCK:
        d = dict(S)
    try:
        s = PC.st(); d["tcp"] = [round(v, 2) for v in s["tcp"]]; d["grip"] = s.get("gripper"); d["frozen"] = s.get("frozen")
    except Exception as e:
        d["tcp"] = None; d["bridge_err"] = str(e)
    refs = {}
    slot = jload(F["slot"]) or {}; rack = jload(F["rack"]) or {}; sig = (jload(F["sig"]) or {}).get("by_color", {}); hov = jload(F["hover"]) or {}
    for c in COLORS:
        refs[c] = {"slot": c in slot, "rack": c in rack, "rack_offset": bool(rack.get(c, {}).get("offset_made")), "sig": c in sig, "hover": sorted((hov.get(c) or {}).keys())}
    d["refs"] = refs
    d["maps"] = {"newcam": os.path.exists(HA.SRC["newcam"]["map"]), "side": os.path.exists(HA.SRC["side"]["map"]), "aruco_ref": os.path.exists(F["aruco"])}
    d["gates"] = {"정렬 완료": bool((d.get("align") or {}).get("done")), "파지 게이트": bool((d.get("grasp") or {}).get("ok")),
                  "베이스 이번 사이클": bool(d.get("base")), "동결 아님": not d.get("frozen"),
                  "대기 없음(하강 대기 제외)": d.get("wait") is None or d.get("stage") == "WAIT DESCEND"}
    return d


PAGE = r"""<!doctype html><meta charset=utf-8><title>HOUSE CYCLE</title>
<style>body{font:14px system-ui;margin:12px;background:#111;color:#eee}button{margin:2px;padding:6px 10px;font-size:14px}
.big{font-size:18px;padding:10px 16px}.st{font-size:22px;margin:6px 0}.ok{color:#5f5}.no{color:#f66}.wait{color:#fc3}
pre{background:#000;padding:8px;height:260px;overflow:auto;font-size:12px}table{border-collapse:collapse}td,th{border:1px solid #444;padding:2px 8px}
.card{display:inline-block;vertical-align:top;background:#1c1c1c;padding:8px;margin:4px;border-radius:6px;min-width:260px}</style>
<h2>HOUSE CYCLE <small id=tcp></small></h2>
<div class=st>단계: <b id=stage>-</b> <span id=wait class=wait></span></div>
<div>색: <select id=color><option>blue<option>yellow<option>red<option>red_s</select>
 <button class=big onclick="cmd('start')">▶ 사이클(1→2→2')</button>
 <button onclick="cmd('start',{teach:1})">▶ 사이클 + 랙 파지 티칭</button>
 <button onclick="cmd('resume_held')">▶ 든 채로 3단계부터(운반→z440 정렬→하강 대기)</button>
 <button onclick="cmd('align_here')">▶ 여기서 정렬만 다시(z440, 든 채)</button>
 <button class=big onclick="if(confirm('4벽 연속 blue→yellow→red→red_s? 벽마다 빈손 베이스 재측정, 하강은 매번 [⬇ 하강] 버튼'))cmd('run4')">▶ 4벽 연속(run4)</button>
 <button class=big id=desc onclick="if(confirm('수직 하강? x·y·yaw 확인했나'))cmd('descend')">⬇ 하강(3)</button>
 <button onclick="if(confirm('하강 → 안착 성공 시 든 채로 z440 올려 기준 재촬영 → 재하강·놓기?'))cmd('descend_reteach')">⬇ 하강+성공 시 z440 기준 재촬영</button>
 <button class=big style="background:#a00;color:#fff" onclick="cmd('abort')">⛔ 중단</button>
 <button onclick="cmd('resume')">▶ 계속</button>
 <button onclick="cmd('goto_obs')">관측자세로</button></div>
<div class=card><b>티칭(선택한 색)</b><br>
 <button onclick="cmd('slot_both')">슬롯 기준 1/2+2/2 한 버튼 (물고 저장 → 놓고 ▶계속 → 측정)</button><br>
 <button onclick="cmd('slot1')">슬롯 기준 1/2 (안착 TCP)</button> <button onclick="cmd('slot2')">슬롯 기준 2/2 (관측 페어링)</button><br>
 <button onclick="cmd('rack_offset')">랙 보정 저장</button> <button onclick="cmd('sig')">좋은 파지 서명 저장</button><br>
 <button onclick="cmd('hover_ref',{src:'wrist'})">z440 기준 저장(손목)</button>
 <button onclick="cmd('hover_ref',{src:'newcam'})">(새카메라)</button> <button onclick="cmd('hover_ref',{src:'side'})">(측면)</button><br>
 <button onclick="cmd('probe',{src:'newcam'})">고정캠 매핑: 새카메라</button> <button onclick="cmd('probe',{src:'side'})">측면캠</button></div>
<div class=card><b>게이트</b><div id=gates></div></div>
<div class=card><b>기준 보유</b><div id=refs></div></div>
<div class=card><b>수치</b><div id=nums></div></div>
<pre id=log></pre>
<script>
const $=id=>document.getElementById(id);
async function cmd(op,extra={}){const p=new URLSearchParams({op,color:$('color').value,...extra});const r=await fetch('/cmd?'+p).then(r=>r.json());if(!r.ok)alert(r.err);}
function f(o){return o?JSON.stringify(o,(k,v)=>typeof v==='number'?+v.toFixed(2):v).replace(/[{}"]/g,'').replace(/,/g,'  '):'-'}
async function poll(){try{const s=await fetch('/state').then(r=>r.json());
 $('stage').textContent=s.stage+(s.err?'  ✖ '+s.err:'')+(s.run4?`  [run4 ${s.run4.idx+1}/${s.run4.order.length} ${s.run4.order[s.run4.idx]} 완료:${s.run4.done.join(',')||'-'}${s.run4.active?'':' 종료'}]`:'');
 $('stage').style.color=s.stage.startsWith('SEAT FAIL')?'#f66':'';$('wait').textContent=s.wait?'⏸ '+s.wait:'';
 $('tcp').textContent=s.tcp?`tcp ${s.tcp.slice(0,3).join(',')} rz${s.tcp[5]} grip ${s.grip}${s.frozen?' ❄FROZEN':''}`:'브리지 없음';
 $('gates').innerHTML=Object.entries(s.gates).map(([k,v])=>`<div class=${v?'ok':'no'}>${v?'✔':'✘'} ${k}</div>`).join('');
 const allok=(Object.values(s.gates).every(Boolean)&&s.stage==='WAIT DESCEND')||((s.stage==='IDLE'||s.stage==='STOPPED')&&!s.busy&&s.tcp&&s.tcp[2]>=354&&s.tcp[2]<=443);$('desc').disabled=!allok;$('desc').style.opacity=allok?1:.4;  // 13:38: 사용자 수동 정렬(z440±3, IDLE/STOPPED)도 하강 허용 — 서버 게이트가 최종
 $('refs').innerHTML='<table><tr><th>색<th>슬롯<th>랙보정<th>서명<th>z440</tr>'+Object.entries(s.refs).map(([c,r])=>`<tr><td>${c}<td>${r.slot?'✔':'✘'}<td>${r.rack_offset?'✔':(r.rack?'seed':'✘')}<td>${r.sig?'✔':'✘'}<td>${r.hover.join('/')||'✘'}`).join('')+'</table>'
  +`<div>매핑 newcam ${s.maps.newcam?'✔':'✘'} · side ${s.maps.side?'✔':'✘'} · ArUco기준 ${s.maps.aruco_ref?'✔':'✘'}</div>`;
 $('nums').innerHTML=`<div>베이스: ${f(s.base&&{x:s.base.x,y:s.base.y,yaw:s.base.yaw,rms:s.base.rms})}</div><div>랙: ${f(s.rack&&{len_px:s.rack.len_px,dang:s.rack.dang,xy:s.rack.grip_xy})}</div>
  <div>파지: ${f(s.grasp)}</div><div>목표: ${f(s.target&&{x:s.target.x,y:s.target.y,rz:s.target.rz,dyaw:s.target.dyaw})}</div><div>정렬: ${f(s.align&&{x:s.align.x,y:s.align.y,rz:s.align.rz,z:s.align.z,made:s.align.made})}</div><div>안착: ${f(s.seat&&{state:s.seat.state,why:s.seat.why,user_override:s.seat.user_override})}</div>`;
 $('log').textContent=s.log.slice(-60).join('\n');$('log').scrollTop=1e9;}catch(e){}}
setInterval(poll,1000);poll();
</script>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _send(self, code, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code); self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query)
        if u.path == "/": return self._send(200, PAGE, "text/html")
        if u.path == "/state": return self._send(200, json.dumps(snapshot(), ensure_ascii=False))
        if u.path == "/cmd": return self._send(200, json.dumps(handle_cmd(q), ensure_ascii=False))
        if u.path == "/health": return self._send(200, json.dumps({"ok": True, "stage": S["stage"]}))
        self._send(404, "{}")


if __name__ == "__main__":
    seed_state()
    threading.Thread(target=worker, daemon=True).start()
    if "--auto" in sys.argv:                                        # ④서버 기동과 함께 run4 를 대기열에(하강은 버튼)
        Q.put(("run4", {"order": list(RUN4_ORDER)})); log("--auto: run4 대기열 등록 (벽마다 하강은 [⬇ 하강] 버튼)")
    log(f"house_cycle :{PORT}  state={STATE}")
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
