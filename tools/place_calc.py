#!/usr/bin/env python3
"""계산 목표 삽입 러너 — "베이스가 움직여도 벽을 꽂는다" 3단계 (9/5 저녁).

골든 '자세'로 가지 않는다. 매 실행마다:
  ① 관측자세에서 기둥 색점 → 베이스 자세(로봇 좌표)         (slot_target)
  ② 그 벽의 슬롯 목표 TCP (x, y, rz) 계산                     (house_geometry.target_tcp)
  ③ 픽(골든 파지 좌표, 파지 실측 게이트) → 운반(30%) → 목표 위 호버 z478 → [옵션] 안착 하강
  하강은 --seat 를 줄 때만. 기본은 호버에서 정지해 사람이/카메라가 확인한다.

  python3 place_calc.py plan <색>              # 로봇 안 움직이고 목표만 출력
  python3 place_calc.py run  <색> [--seat] [--no-pick]
      --no-pick : 이미 물고 있으면 픽 생략(현재 자세에서 운반 시작)
      --seat    : 호버 후 안착 z 까지 천천히 하강(3%→1%), 기둥 꼭대기 아래로는 XY 보정 없음(C2)

안전: 이동은 전부 dry_run 선검사 · TCP 도달 폴링 · 동결 시 즉시 정지 · 실패 시 부품 자동 복귀 없음(철칙).
"""
import sys, json, time, math, os
import urllib.request as UR
import numpy as np

sys.path.insert(0, "/home/ar/bf2_console/tools")
import house_geometry as HG

BR = "http://127.0.0.1:8765"
CAL = "/home/ar/bf2_console/dot_calib.json"
OBS = [200.0, -330.0, 650.0, 180.0, 0.0, 180.0]      # 관측자세(매핑 기준)
SAFE_Z, HOVER_Z = 650.0, 478.0
SEAT_Z = {"blue": 355.0, "yellow": 354.0, "red": 353.0, "red_s": 351.0}   # 9/2~3 골든 안착 z
SPD_MOVE, SPD_DESC, SPD_SEAT = 30, 10, 3


def post(a, b):
    r = UR.Request(f"{BR}/fr5/{a}", json.dumps(b).encode(), {"Content-Type": "application/json"})
    return json.loads(UR.urlopen(r, timeout=45).read())


def st():
    return json.loads(UR.urlopen(BR + "/status", timeout=6).read())["robots"]["fr5"]


def speed(v):
    post("speed", {"value": v, "dry_run": False}); time.sleep(0.2)


def wait_idle(t=30):
    t0 = time.time(); time.sleep(0.3)
    while time.time() - t0 < t and st()["busy"]:
        time.sleep(0.2)


def move(tcp, tol=0.6, timeout=60, tag=""):
    r = post("move_tcp", {"tcp": tcp, "dry_run": True})
    if r.get("result") != "dry_run":
        raise RuntimeError(f"{tag} dry_run 거부 {r}")
    r = post("move_tcp", {"tcp": tcp, "dry_run": False})
    if r.get("result") not in ("started", "ok"):
        raise RuntimeError(f"{tag} 이동 거부 {r}")
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = st()
        if s.get("frozen"):
            raise RuntimeError(f"{tag} 동결")
        c = s["tcp"]
        if max(abs(c[i] - tcp[i]) for i in range(3)) <= tol and abs(HG.wrap_deg(c[5] - tcp[5])) <= 0.3 and not s["busy"]:
            time.sleep(0.4)
            print(f"  ✓ {tag} ({c[0]:.1f},{c[1]:.1f},{c[2]:.1f}) rz{c[5]:+.2f}", flush=True)
            return c
        time.sleep(0.2)
    raise RuntimeError(f"{tag} 미도달 목표{[round(v,1) for v in tcp[:3]]} 현재{[round(v,1) for v in st()['tcp'][:3]]}")


def grip_read():
    return str(post("grip_read", {"dry_run": True})["result"])


def gripper(pos):
    post("gripper", {"pos": int(pos), "dry_run": False}); wait_idle(); time.sleep(0.6)
    return grip_read()


GRASP_REF = "/home/ar/bf2_console/grasp_ref_0905.json"
# ★파지 편차 부호(가정, 첫 실주행에서 검증할 것):
#   손목캠 화면 +x  = 로봇 −X = (rz 180 파지 자세에서) 그리퍼 +x = 벽을 '가로지르는' 방향(across)
#   손목캠 화면 +y  = 로봇 −Y = 그리퍼 −y = 벽 '길이' 방향(along) 의 반대
ACROSS_SIGN, ALONG_SIGN = +1.0, -1.0


def load_grasp_ref(color):
    """색별 공칭 파지 서명. 없으면 None (→ 공칭 파지로 진행, 경고)."""
    if not os.path.exists(GRASP_REF):
        return None
    ref = json.load(open(GRASP_REF))
    if ref.get("color") == color and "cam1_wall_mid" in ref:
        return ref
    return (ref.get("by_color") or {}).get(color)


def wall_dots_cam1(ref):
    """물고 있는 벽의 색점 2개(손목캠). ref["cam1_wall_detect"] 규칙(색별)."""
    import cv2, numpy as np
    cfg = ref["cam1_wall_detect"]
    b = UR.urlopen("http://127.0.0.1:8766/raw", timeout=5).read()
    img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(cfg["hsv_lo"], np.uint8), np.array(cfg["hsv_hi"], np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stt, cen = cv2.connectedComponentsWithStats(m); pts = []
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    for i in range(1, n):
        a = int(stt[i, 4]); x, y = cen[i]
        if not (cfg["area"][0] <= a <= cfg["area"][1]) or x < cfg["x_min"] or x > 1260 or y < 20 or y > 700:
            continue
        ys, xs = np.nonzero(lab == i); w = g[ys, xs].astype(float) + 1
        pts.append((float((xs * w).sum() / w.sum()), float((ys * w).sum() / w.sum()), a))
    pts.sort(key=lambda q: q[1])
    return pts


def grasp_measure(color="blue", rack_dang=None):
    """설계 2단계: 잡은 벽이 그리퍼에 어떻게 물렸나 — 공칭 파지 서명 대비 편차 → GripMeasure.
    점 2개면 위치+각, 점 1개(노랑·red_s)면 위치만(각은 랙 관측 각차 rack_dang 로). 축척은 뎁스 실측(d_w/fx).
    반환 (grip, info) / 실패 (None, why)."""
    ref = load_grasp_ref(color)
    if ref is None:
        return None, f"{color} 공칭 파지 서명 없음(`place_calc.py grasp {color} <개도>` 로 1회 기록)"
    pts = wall_dots_cam1(ref)
    n_ref = len(ref.get("cam1_wall_pd") or [])
    if len(pts) < 1:
        return None, "든 벽 점 0개"
    scale = ref.get("wall_scale_mm_per_px") or wall_scale(color)
    if len(pts) >= 2 and n_ref >= 2:
        (x1, y1, _), (x2, y2, _) = pts[0], pts[-1]
        ang = math.degrees(math.atan2(x2 - x1, y2 - y1)); mid = ((x1 + x2) / 2, (y1 + y2) / 2)
        dang = HG.wrap_deg(ang - ref["cam1_wall_ang_deg"]); how = "2점"
    else:
        # 1점: 기준 점 자리에 가장 가까운 점 하나로 위치만. 각은 랙 관측 각차(없으면 0).
        rx, ry = ref["cam1_wall_mid"]
        q = min(pts, key=lambda t: math.hypot(t[0] - rx, t[1] - ry)); mid = (q[0], q[1])
        dang = rack_dang if rack_dang is not None else 0.0; how = "1점" + ("+랙각" if rack_dang is not None else "")
    dx = mid[0] - ref["cam1_wall_mid"][0]; dy = mid[1] - ref["cam1_wall_mid"][1]
    across = ACROSS_SIGN * dx * scale; along = ALONG_SIGN * dy * scale
    grip = HG.GripMeasure(center=(across, along), angle_deg=90.0 + dang, bottom_dz=0.0)
    return grip, {"dx_px": dx, "dy_px": dy, "dang": dang, "across_mm": across, "along_mm": along, "scale": scale, "how": how}


GRASP_GATE_MM, GRASP_GATE_DEG = 1.0, 0.3     # 설계 2단계 게이트(9/4 v2.1). 넘으면 정지·보고, 자동 재파지 금지
HELD_TOP_DEPTH_OFFSET = 273.0   # 든 벽 윗변 뎁스(mm) = SEAT_Z − 273  (아래 유도)


def wall_scale(color):
    """든 벽 윗변 평면의 손목캠 축척(mm/px) — **뎁스도, 점 간격 가정도 안 쓴다**(9/6 새벽 검증).
    유도: 관측자세 z650 에서 기둥꼭대기 뎁스 377mm(실측) → TCP z 에서 기둥꼭대기 뎁스 = 377−(650−z).
          안착 z_seat 에서 벽 윗변 = 기둥 꼭대기이므로 TCP z 에서 벽 윗변은 기둥 꼭대기보다 (z−z_seat) 위
          → 벽 윗변 뎁스 = 377−650+z−(z−z_seat) = z_seat−273 (z 무관 상수). 파랑 82 · 노랑 81 · 빨강 80 · red_s 78mm.
          축척 = 뎁스/fx, fx = 377/0.4135 = 912px.
    교차검증(9/6): 파랑 든 점 간격 425.9px × 0.0899 = 38.3mm ↔ 랙 관측 실측 인접 간격 38.8mm (차 1.3%).
          (랙 실측 5점 간격 38.8/53.5/63.1/29.7mm → '균등 간격' 가정은 틀렸음, 폐기)
    ★D435 최소 거리(1280×720 에서 ~280mm)보다 훨씬 가까워 든 벽 뎁스는 원리적으로 안 나온다 — 사용자 관찰과 일치."""
    return (SEAT_Z[color] - HELD_TOP_DEPTH_OFFSET) / FX_PX


def save_grasp_sig_now(color, grip_cmd, gr):
    """★든 상태에서 지금 손목캠 벽 점을 공칭 파지 서명으로 저장(그리퍼 조작 없음). gap_mm 주면 축척도 캘리브."""
    import cv2, numpy as np
    lo, hi = WALL_DOT_HSV[color]
    b = UR.urlopen("http://127.0.0.1:8766/raw", timeout=5).read()
    img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR); hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8)); m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stt, cen = cv2.connectedComponentsWithStats(m); g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY); pts = []
    for i in range(1, n):
        a = int(stt[i, 4]); x, y = cen[i]
        if not (500 <= a <= 6000) or x < 600 or x > 1260 or y < 20 or y > 700:
            continue
        ys, xs = np.nonzero(lab == i); w = g[ys, xs].astype(float) + 1
        pts.append((float((xs * w).sum() / w.sum()), float((ys * w).sum() / w.sum()), a))
    pts.sort(key=lambda q: q[1])
    if not pts:
        print("  ⚠ 서명 저장 실패: 벽 점 0개"); return None
    if len(pts) >= 2:
        (x1, y1, _), (x2, y2, _) = pts[0], pts[-1]
        ang = math.degrees(math.atan2(x2 - x1, y2 - y1)); L = math.hypot(x2 - x1, y2 - y1); mid = [(x1 + x2) / 2, (y1 + y2) / 2]
    else:
        ang, L, mid = 0.0, 0.0, [pts[0][0], pts[0][1]]
    ref = json.load(open(GRASP_REF)) if os.path.exists(GRASP_REF) else {}
    ref.setdefault("by_color", {})[color] = {"made": time.strftime("%Y-%m-%d %H:%M"), "grip_cmd": grip_cmd, "grip_real": gr, "tcp": st()["tcp"],
        "cam1_wall_pd": [[q[0], q[1], q[2]] for q in pts], "cam1_wall_ang_deg": ang, "cam1_wall_gap_px": L, "cam1_wall_mid": mid,
        "cam1_wall_detect": {"hsv_lo": list(lo), "hsv_hi": list(hi), "area": [500, 6000], "x_min": 600},
        "note": "랙 중앙 파지(rack_calib) 상태에서 --grasp-teach 로 저장"}
    json.dump(ref, open(GRASP_REF, "w"), ensure_ascii=False, indent=1)
    print(f"  ✅ [{color}] 파지 서명 저장({len(pts)}점): 각 {ang:+.2f}° 간격 {L:.0f}px 중점 ({mid[0]:.0f},{mid[1]:.0f})")
    return pts


# ★뎁스 기반 벽 계측(9/5 밤, 사용자 설계 "잡으면 뎁스로 벽 길이를 재서 검산"). ⚠ 실기 미검증(로봇 꺼진 뒤 작성).
#   fx 는 관측 매핑(z650 기둥꼭대기 뎁스 377mm ↔ 0.4135mm/px)에서 역산: fx = 377/0.4135 ≈ 912px.
#   ① 든 벽(호버·랙 위): 손목캠 프레임에 벽 **전체가 안 들어온다**(9/5 파랑 z440 프레임: 흰 판이 위아래 프레임 밖,
#      198mm ≈ 1650px > 720px). 그래서 길이는 못 재고, **뎁스 d_w → 실측 축척(d_w/fx)** 과 **벽 축 각(PCA)** 만 잰다.
#      축척은 grasp_measure 의 가상값 0.12 를 대체하고, 축 각은 점 1개 벽(노랑·red_s)의 파지 회전 관측에 쓴다.
#   ② 랙 관측자세(z556, 벽 전체 5점 보임): 색점 축 주변 뎁스 돌출 띠의 양 끝 = 물리적 양 끝 → 길이 검산.
#      색점 두 끝 중간 ≠ 물리 중앙(~9mm) 문제를 색점과 독립인 방법으로 교차검증한다.
FX_PX = 377.0 / 0.4134670689041742
WALL_LEN_MM = {"blue": 198.0, "red": 198.0, "yellow": 125.0, "red_s": 125.0}
LEN_GATE_MM = 4.0
HELD_DEPTH_MAX = 250.0          # 이보다 먼 점은 든 벽이 아님(밑판·책상·랙)
HELD_BOX_DEPTH = (600, 0, 1279, 719)


def _depthgrid(x0, y0, x1, y1, nx, ny):
    import numpy as np
    g = json.loads(UR.urlopen(f"http://127.0.0.1:8766/depthgrid?x0={x0}&y0={y0}&x1={x1}&y1={y1}&nx={nx}&ny={ny}&r=2", timeout=8).read())
    return np.array([(p["x"], p["y"], p["d"]) for p in g["pts"] if p["d"]], float)


def held_wall_depth(color, nx=68, ny=72):
    """든 벽 윗면 뎁스 → {"d_w", "mm_px", "ang_img", "span_px", "n"} 또는 (None, why). 길이는 프레임 클리핑으로 못 잰다."""
    import numpy as np
    x0, y0, x1, y1 = HELD_BOX_DEPTH
    try:
        pts = _depthgrid(x0, y0, x1, y1, nx, ny)
    except Exception as e:
        return None, f"depthgrid 실패: {e}"
    pts = pts[(pts[:, 2] > 40) & (pts[:, 2] < HELD_DEPTH_MAX)]
    if len(pts) < 30:
        return None, f"든 벽 뎁스 점 {len(pts)}개(<30) — 벽을 안 들었거나 뎁스 무효"
    d_w = float(np.median(pts[:, 2]))
    if not (80.0 <= d_w <= 300.0):
        return None, f"든 벽 뎁스 {d_w:.0f}mm 비현실(검은 벽 뎁스 불량 의심)"
    near = pts[np.abs(pts[:, 2] - d_w) < 12.0]
    c = near[:, :2].mean(0); X = near[:, :2] - c
    w, v = np.linalg.eigh(X.T @ X); ax = v[:, int(np.argmax(w))]
    proj = X @ ax
    return {"d_w": d_w, "mm_px": d_w / FX_PX, "ang_img": math.degrees(math.atan2(ax[1], ax[0])),
            "span_px": float(proj.max() - proj.min()), "center_px": (float(c[0]), float(c[1])), "n": int(len(near))}, None


def rack_len_depth(color, ends, band_px=60, step_px=6.0):
    """랙 관측자세에서 색점 양 끝(ends["p1"],["p2"]) 축 주변 뎁스 띠 → 물리적 길이·중심. RACK_MAP 축척 사용.
    반환 {"len_mm","len_px","center_px","dot_len_px","d_top"} 또는 (None, why)."""
    import numpy as np
    (x1, y1), (x2, y2) = ends["p1"], ends["p2"]
    ux, uy = x2 - x1, y2 - y1; L = math.hypot(ux, uy) or 1.0; ux, uy = ux / L, uy / L
    ext = 0.35 * L                                        # 양 끝 바깥으로 35% 더 본다(점이 끝이 아닐 수 있음)
    xs = [x1 - ux * ext, x2 + ux * ext]; ys = [y1 - uy * ext, y2 + uy * ext]
    bx0 = int(max(0, min(xs) - band_px)); bx1 = int(min(1279, max(xs) + band_px))
    by0 = int(max(0, min(ys) - band_px)); by1 = int(min(719, max(ys) + band_px))
    # 격자 해상도: 축 방향 끝 판정 오차 = 한 칸이므로 x·y 모두 step_px(≈6px≈2mm) 로 깐다(점 수 ≤ ~4000)
    nx = int(min(220, max(8, (bx1 - bx0) / step_px))); ny = int(min(220, max(8, (by1 - by0) / step_px)))
    try:
        pts = _depthgrid(bx0, by0, bx1, by1, nx, ny)
    except Exception as e:
        return None, f"depthgrid 실패: {e}"
    pts = pts[(pts[:, 2] > 100) & (pts[:, 2] < 900)]
    if len(pts) < 40:
        return None, "뎁스 점 부족"
    # 축 수직거리 band 안 점만, 뎁스 최빈 대역(벽 윗면 = 가장 가까운 큰 덩어리)
    rel = pts[:, :2] - np.array([x1, y1]); along = rel @ np.array([ux, uy]); perp = rel @ np.array([-uy, ux])
    m = np.abs(perp) < 25.0
    if m.sum() < 20:
        return None, "축 띠 안 뎁스 점 부족"
    d = pts[m, 2]; hist, edges = np.histogram(d, bins=int(max(5, (d.max() - d.min()) / 3.0)))
    d_top = 0.5 * (edges[np.argmax(hist)] + edges[np.argmax(hist) + 1])
    top = m & (np.abs(pts[:, 2] - d_top) < 8.0)
    a = along[top]
    if len(a) < 10:
        return None, "벽 윗면 대역 점 부족"
    step = abs(ux) * (bx1 - bx0) / nx + abs(uy) * (by1 - by0) / ny     # 축 방향 격자 한 칸
    a0, a1 = float(a.min()) - step / 2, float(a.max()) + step / 2
    scale = float(np.hypot(*np.array(json.load(open(RACK_MAP))["Jinv_mm_per_px"])[:, 0]))
    cx, cy = x1 + ux * (a0 + a1) / 2, y1 + uy * (a0 + a1) / 2
    return {"len_px": a1 - a0, "len_mm": (a1 - a0) * scale, "center_px": (float(cx), float(cy)),
            "dot_len_px": L, "d_top": float(d_top), "n": int(top.sum())}, None


def rack_len_check(color, ends, strict=False):
    """랙 길이 검산: 뎁스 물리 길이 vs 기지 길이, 뎁스 중심 vs 색점 중간. strict 면 게이트(예외)."""
    m, why = rack_len_depth(color, ends)
    if m is None:
        print("  ⚠ 랙 뎁스 길이 측정 불가:", why)
        if strict: raise RuntimeError("랙 뎁스 길이 측정 불가: " + why)
        return None
    L0 = WALL_LEN_MM[color]; dl = m["len_mm"] - L0
    off = math.dist(m["center_px"], ends["mid"])
    print(f"  랙 뎁스 검산[{color}]: 물리 길이 {m['len_mm']:.1f}mm (기지 {L0:.0f}, Δ{dl:+.1f}) · 뎁스중심↔색점중간 {off:.1f}px · 윗면 {m['d_top']:.0f}mm · 점 {m['n']}")
    if abs(dl) > LEN_GATE_MM:
        msg = f"랙 벽 길이 불일치 Δ{dl:+.1f}mm > {LEN_GATE_MM} — 끝 가림/겹침/다른 벽 의심"
        if strict: raise RuntimeError(msg)
        print("  ⚠", msg, "(보고만, --len-gate 로 게이트화)")
    return m


PICK_REF = "/home/ar/bf2_console/pick_ref_0905.json"
WALL_DOT_HSV = {                       # 물고 있는 벽의 점 색(손목캠, 카메라 가까움 → 밝고 큼)
    "blue":   ((95, 150, 140), (115, 255, 255)),
    "yellow": ((15, 80, 110), (38, 255, 255)),
    "red":    ((135, 90, 55), (175, 255, 255)),
    "red_s":  ((135, 90, 55), (175, 255, 255)),
}


def pick_pose(color):
    """랙 파지 자세: 사용자가 9/5 수정한 값(pick_ref_0905)이 있으면 우선, 없으면 dot_calib 골든."""
    refs = json.load(open(CAL))["refs"][color]
    pick, hover, g_open, g_close = refs["pick_tcp_taught"], refs["golden_pick"]["golden_tcp"], refs["grip_open"], refs["grip_close"]
    if os.path.exists(PICK_REF):
        pr = json.load(open(PICK_REF)).get(color)
        if pr:
            pick = pr["tcp"]; hover = [pick[0], pick[1], pick[2] + 125.0] + list(pick[3:])
            g_close = pr.get("grip_close", g_close); g_open = pr.get("grip_open", g_open)
    return pick, hover, g_open, g_close


def capture_grasp_sig(color, grip_cmd):
    """★사용자가 랙에서 자세를 맞춘 뒤 '잡아': 닫기 → 실측 → 벽 점 서명 저장 → 파지 TCP 기록."""
    import cv2, numpy as np
    tcp0 = st()["tcp"]
    gr = gripper(grip_cmd); print(f"  그리퍼 닫기 {grip_cmd} → 실측 {gr}")
    time.sleep(0.4)
    lo, hi = WALL_DOT_HSV[color]
    b = UR.urlopen("http://127.0.0.1:8766/raw", timeout=5).read()
    img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR); hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8)); m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stt, cen = cv2.connectedComponentsWithStats(m); g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY); pts = []
    for i in range(1, n):
        a = int(stt[i, 4]); x, y = cen[i]
        if not (500 <= a <= 6000) or x < 600 or x > 1260 or y < 20 or y > 700:
            continue
        ys, xs = np.nonzero(lab == i); w = g[ys, xs].astype(float) + 1
        pts.append((float((xs * w).sum() / w.sum()), float((ys * w).sum() / w.sum()), a))
    pts.sort(key=lambda q: q[1])
    print(f"  벽 점({color}): " + " ".join(f"({q[0]:.0f},{q[1]:.0f})a{q[2]}" for q in pts))
    cv2.imwrite(f"/tmp/claude-1000/-home-ar/4c72d906-75dc-49ee-b7b6-6139e1b44a52/scratchpad/grasp_{color}.jpg", img)
    ref = json.load(open(GRASP_REF)) if os.path.exists(GRASP_REF) else {}
    ref.setdefault("by_color", {})
    if len(pts) >= 1:
        if len(pts) >= 2:
            (x1, y1, _), (x2, y2, _) = pts[0], pts[-1]
            ang = math.degrees(math.atan2(x2 - x1, y2 - y1)); L = math.hypot(x2 - x1, y2 - y1); mid = [(x1 + x2) / 2, (y1 + y2) / 2]
        else:
            ang, L, mid = 0.0, 0.0, [pts[0][0], pts[0][1]]          # 1점(노랑·red_s): 위치 서명만
        ref["by_color"][color] = {"made": time.strftime("%Y-%m-%d %H:%M"), "grip_cmd": grip_cmd, "grip_real": gr, "tcp": tcp0,
            "cam1_wall_pd": [[q[0], q[1], q[2]] for q in pts], "cam1_wall_ang_deg": ang, "cam1_wall_gap_px": L,
            "cam1_wall_mid": mid,
            "cam1_wall_detect": {"hsv_lo": list(lo), "hsv_hi": list(hi), "area": [500, 6000], "x_min": 600}}
        print(f"  서명 저장({len(pts)}점): 각 {ang:+.2f}° 간격 {L:.0f}px 중점 ({mid[0]:.0f},{mid[1]:.0f})")
    else:
        print("  ⚠ 벽 점 0개 — 서명 저장 안 함")
    json.dump(ref, open(GRASP_REF, "w"), ensure_ascii=False, indent=1)
    pr = json.load(open(PICK_REF)) if os.path.exists(PICK_REF) else {}
    pr[color] = {"tcp": tcp0, "grip_close": int(grip_cmd), "grip_real": gr, "made": time.strftime("%Y-%m-%d %H:%M"),
                 "note": "사용자 수정 랙 파지 자세(9/5) — dot_calib 골든은 그대로"}
    json.dump(pr, open(PICK_REF, "w"), ensure_ascii=False, indent=1)
    print(f"  랙 파지 TCP 기록 {[round(v,1) for v in tcp0[:3]]} → pick_ref_0905.json")
    return gr


# ★벽별 파지·안착 보정(9/5 교차검증 실측): 파랑 골든만 기준으로 다른 벽 골든 안착 TCP 를 예측했을 때의 잔차.
#   공칭(중앙 파지) 예측 − 실제 골든 = 그 벽의 "치우친 파지 + 안착 특성". 골든 시점 베이스 yaw −90.03° 기준의
#   로봇 프레임 Δ 를 베이스 프레임으로 바꿔 저장하고, 실행 시 현재 베이스 yaw 로 돌려 적용한다.
#   노랑 +X 치우침 사고(9/5 18:0x)의 직접 원인 = 이 보정을 빼고 공칭으로 계산한 것.
GOLDEN_YAW = -90.032
RZ_BIAS = 0.40      # ★9/5 실측: 노랑 +0.42°, 빨강 +0.40° 로 일관된 yaw 편향 → 일괄 보정
WALL_DELTA_ROBOT = {"blue": (0.0, 0.0), "yellow": (-1.27, 1.81),
                    "red": (-4.51, 3.50), "red_s": (229.0 - 234.92, -511.5 + 510.56)}


def wall_delta_now(color, yaw_now):
    dx, dy = WALL_DELTA_ROBOT.get(color, (0.0, 0.0))
    a = math.radians(yaw_now - GOLDEN_YAW)          # 골든 때 대비 베이스가 돈 만큼 같이 돌린다
    return (dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a))


USE_LEGACY_DELTA = False   # ★벽별 상수 보정(WALL_DELTA/RZ_BIAS)은 파지 치우침을 상수화한 것 → 호버 정렬이 있으면 끈다(설계 3단계)
BASE_LAST = "/home/ar/bf2_console/base_pose_last.json"


def measure_base(holding=False):
    """설계 3단계 앞부분: 관측자세(z650)에서 베이스 자세(로봇 좌표). 로봇이 관측자세가 아니면 간다(자유공간).
    ★매 사이클 빈 손으로 호출한다(직전 벽 삽입이 베이스를 밀 수 있으니). 직전 측정과의 차이를 같이 보고."""
    import slot_target as STG
    cur = st()["tcp"]
    if max(abs(cur[i] - OBS[i]) for i in range(3)) > 2.0:
        speed(SPD_MOVE)
        move([cur[0], cur[1], SAFE_Z] + list(cur[3:]), tag="상승 SAFE")
        move(OBS, tag="관측자세")
        speed(1)
    Jinv, mp = STG.load_map()
    if not health_gate():
        raise RuntimeError("검출 건강 게이트 실패(기둥 4점 미달) — 이동 금지")
    px4, why = STG.pillars_px(mask_held=holding)
    if px4 is None:
        raise RuntimeError("기둥 검출 실패: " + str(why))
    a = json.load(open(STG.ANCH))
    pose, rms, _ = STG.base_pose_robot(px4, Jinv, tuple(a["C"]), tuple(a["p0"]))
    prev = json.load(open(BASE_LAST)) if os.path.exists(BASE_LAST) else None
    if prev:
        print(f"  베이스 이동(직전 측정 대비): Δx {pose.x - prev['x']:+.2f} Δy {pose.y - prev['y']:+.2f} Δyaw {HG.wrap_deg(pose.yaw_deg - prev['yaw']):+.3f}°  (직전 {prev['made']})")
    json.dump({"x": pose.x, "y": pose.y, "yaw": pose.yaw_deg, "rms": rms, "made": time.strftime("%Y-%m-%d %H:%M:%S")}, open(BASE_LAST, "w"))
    print(f"  베이스(로봇) x {pose.x:.2f} y {pose.y:.2f} yaw {pose.yaw_deg:+.3f}° rms {rms:.2f}mm  (기준점 {a['color']} {a['made']})")
    return {"base": pose, "rms": rms, "anchor": a["color"], "anchor_made": a["made"]}


def target_for(color, B, grip=None):
    """베이스 자세 B + 파지 측정 grip → 그 벽의 목표 TCP (x, y, rz)."""
    import slot_target as STG
    pose = B["base"]
    slot = HG.SLOTS[STG.WALL_SLOT[color]]
    rz_ref = json.load(open(CAL))["refs"][color]["insert_tcp"][5]
    tgt, _dj = HG.target_tcp(pose, slot, grip or STG.grip_nominal(color), rz_ref)
    rz = HG.wrap_deg(STG.rz_line_sym(tgt.yaw_deg, rz_ref) + (RZ_BIAS if USE_LEGACY_DELTA else 0.0))
    ddx, ddy = wall_delta_now(color, pose.yaw_deg) if USE_LEGACY_DELTA else (0.0, 0.0)
    return dict(B, x=tgt.x + ddx, y=tgt.y + ddy, rz=rz, delta=(ddx, ddy))


def plan(color, grip=None, holding=False):
    return target_for(color, measure_base(holding), grip)


def _grip_ok(gr, g_close, color):
    """파지 판정: 실측 > 닫힘값이면 물었음. 같으면(red_s 는 물어도 8→8) 손목캠 벽 점으로 2차 판정."""
    try:
        gv = int(gr)
    except Exception:
        gv = -1
    if gv > g_close:
        return True, f"그리퍼 {gr} > 닫힘 {g_close}"
    held = held_wall_dots(color)
    if held:
        return True, f"그리퍼 {gr} = 닫힘값이지만 손목캠 벽 점 {len(held)}개"
    return False, f"그리퍼 {gr}, 닫힘 {g_close}, 벽 점 0"


def run(color, seat=False, do_pick=True, target=None, align=True, len_gate=False, grasp_gate=True, grasp_teach=False):
    """★설계 4단계 고정 순서(9/5 밤, 사용자 설계):
      ① 빈 손 베이스 재확인(관측자세)  — 매 사이클(직전 삽입이 베이스를 밀 수 있음)
      ② 랙 재관측 → 벽 중앙 → 하강 파지 → 파지 판정  — 매 픽(픽마다 랙이 밀림)
      ③ 20mm 상승 미끄러짐 확인 → 파지 편차 측정(게이트) → ①의 베이스로 목표 TCP → 운반 → 호버 z478 → 파지 재확인 → z+85
      ④ z+85 에서 두 카메라 호버 정렬(기둥 기준 상대) → 정렬된 TCP 로 막힘감시 하강 → 안착 시 기준 자동 승격
    target=(x,y,rz) 를 주면 ①③ 계산을 건너뛰고 그 목표로(디버그용). --no-pick 은 이미 든 상태(①은 든 채 마스크 측정)."""
    pick, hover, g_open, g_close = pick_pose(color)
    grip = None; rack_dang = None
    tgt_rot = None
    try:
        # ---------- ① 베이스 재확인
        if target is not None:
            P = {"x": target[0], "y": target[1], "rz": target[2], "delta": (0.0, 0.0), "anchor": "직접지정", "anchor_made": "-",
                 "base": HG.Pose2D(0.0, 0.0, 0.0), "rms": 0.0}; B = P
            print("  ⚠ 목표 직접 지정 — 베이스 측정 생략")
        elif do_pick:
            print("① 베이스 재확인(빈 손)"); B = measure_base(holding=False)
        else:
            print("① 베이스 재확인(든 채, 마스크)"); B = measure_base(holding=True)
            g, info = grasp_measure(color)
            if g: grip = g; print(f"  파지 편차({info['how']}): 가로 {info['across_mm']:+.2f} 길이 {info['along_mm']:+.2f}mm 각 {info['dang']:+.2f}°")
            else: print("  ⚠ 파지 편차 측정 불가:", info)
        # ---------- ② 랙 재관측 → 파지
        if do_pick:
            print("② 랙 재관측 → 중앙 파지")
            speed(SPD_MOVE); cur = st()["tcp"]
            move([cur[0], cur[1], SAFE_Z] + list(cur[3:]), tag="상승 SAFE")
            obs = json.load(open(RACK_OBS))["tcp"]
            move([obs[0], obs[1], SAFE_Z, 180.0, 0.0, 180.0], tag="랙 위 SAFE")
            print("  그리퍼 열기 →", gripper(g_open))
            move(obs, tag="랙 중앙 관측자세")
            tg = rack_grip_xy(color)                      # ★매번 새 프레임(4장)으로 벽 중앙·길이 게이트
            if tg is None:
                raise RuntimeError("랙 관측에서 벽 중앙을 못 잡음(캘리브 없음/끝점 누락/길이 불일치) — 파지 중단")
            gx, gy, rack_dang = tg
            e = rack_ends(color, x_hint=json.load(open(RACK_CALIB))[color]["Pc0"][0])
            print(f"  랙: 벽 중앙 ({e['mid'][0]:.0f},{e['mid'][1]:.0f}) 길이 {e['len_px']:.0f}px 각 {e['ang']:+.2f}° → 파지 XY ({gx:.1f},{gy:.1f}) 각차 {rack_dang:+.2f}°")
            rack_len_check(color, e, strict=len_gate)     # 뎁스 물리 길이 검산(색점과 독립, 보고만 — 검은 랙에서 뎁스 불안정)
            rot = [180.0, 0.0, 180.0]                     # ★랙 하강은 항상 rz 180(9/5: −177 잔류 → 3° 물림·동결)
            speed(SPD_MOVE); move([gx, gy, obs[2]] + rot, tag="파지 XY 위(관측 높이)")
            speed(SPD_DESC); move([gx, gy, pick[2] + 40] + rot, tag="픽 −40")
            speed(SPD_SEAT); move([gx, gy, pick[2]] + rot, tol=0.8, tag="픽 자세")
            gr = gripper(g_close); print("  그리퍼 닫기 →", gr)
            ok, why = _grip_ok(gr, g_close, color)
            if not ok:
                raise RuntimeError("빈 파지 의심(" + why + ") — 정지")
            print("  파지 판정 OK:", why)
            # ---------- ③ 미끄러짐 확인 → 파지 편차 → 목표
            print("③ 파지 검증 → 목표 계산")
            speed(SPD_DESC); move([gx, gy, pick[2] + 20] + rot, tag="+20 미끄러짐 확인")
            time.sleep(0.4); ok, why = _grip_ok(grip_read(), g_close, color)
            if not ok:
                raise RuntimeError("20mm 상승 후 파지 이탈(" + why + ") — 정지")
            move([gx, gy, hover[2]] + rot, tag="들어올림")
            hd, _w = held_wall_depth(color)                 # 참고 출력만(뎁스 불신)
            if hd: print(f"  (참고) 든 벽 뎁스 {hd['d_w']:.0f}mm · 축 {hd['ang_img']:+.2f}° · 폭 {hd['span_px']:.0f}px")
            if grasp_teach:
                # ★서명 티칭: 랙 중앙 파지가 곧 공칭 파지. 지금 든 점을 서명으로 저장(+파랑이면 축척 캘리브), 게이트 생략
                save_grasp_sig_now(color, g_close, grip_read())
                grasp_gate = False
            g, info = grasp_measure(color, rack_dang=rack_dang)
            if g is None:
                if grasp_gate:
                    raise RuntimeError("파지 편차 측정 불가: " + str(info) + " — 정지(--no-grasp-gate 로만 우회)")
                print("  ⚠ 파지 편차 측정 불가:", info, "→ 공칭 파지")
            else:
                grip = g
                print(f"  파지 편차({info['how']}): 가로 {info['across_mm']:+.2f} 길이 {info['along_mm']:+.2f}mm 각 {info['dang']:+.2f}° (축척 {info['scale']:.4f})")
                if grasp_gate and (abs(info['along_mm']) > GRASP_GATE_MM or abs(info['across_mm']) > GRASP_GATE_MM or abs(info['dang']) > GRASP_GATE_DEG):
                    raise RuntimeError(f"파지 편차 게이트 초과(길이 {info['along_mm']:+.2f} 가로 {info['across_mm']:+.2f}mm 각 {info['dang']:+.2f}°) — 정지, 재파지는 사용자 판단")
        if target is None:
            P = target_for(color, B, grip)                # ★①의 베이스(빈 손 측정) + 파지 측정 → 목표. 든 채 재측정 안 함
        print(f"  목표 [{color}] x {P['x']:.2f} y {P['y']:.2f} rz {P['rz']:+.2f}  호버 z{HOVER_Z:.0f} 안착 z{SEAT_Z[color]:.0f}"
              + ("" if not USE_LEGACY_DELTA else f"  (구 상수보정 {P['delta'][0]:+.1f},{P['delta'][1]:+.1f})"))
        tgt_rot = [180.0, 0.0, P["rz"]]
        speed(SPD_MOVE); cur = st()["tcp"]
        move([cur[0], cur[1], SAFE_Z] + list(cur[3:]), tag="상승 SAFE")
        move([P["x"], P["y"], SAFE_Z] + tgt_rot, tag="목표 위 SAFE(rz 정렬)")
        ok, why = _grip_ok(grip_read(), g_close, color)     # ★운반 후 파지 재확인(9/5 이탈 사고)
        if not ok:
            raise RuntimeError("운반 후 파지 이탈 의심(" + why + ") — 정지")
        print("  운반 후 파지 재확인 OK:", why)
        speed(SPD_DESC); move([P["x"], P["y"], HOVER_Z] + tgt_rot, tag="목표 호버 z478")
        if not seat:
            print("호버 정지. --seat 를 주면 z+85 에서 호버 정렬 후 하강."); return
        # ---------- ④ 호버 정렬 → 하강
        zs = SEAT_Z[color]
        speed(SPD_SEAT); move([P["x"], P["y"], zs + 85] + tgt_rot, tag="기둥 꼭대기 위 z+85")
        print("④ 호버 정렬(두 카메라, 기둥 기준 상대)")
        import hover_align as HA
        if align:
            if HA.load_ref(color) is None:
                raise RuntimeError(f"{color} 호버 기준 없음 — z+85 에서 사용자 정렬 확인 후 `hover_align.py ref {color}` (--no-align 으로만 우회)")
            HA.align(color)                                # 미수렴·발산·측정 실패·카메라 불일치 = 예외 → 정지
            cur = st()["tcp"]                              # ★정렬로 움직인 TCP 로 하강(옛 P 로 내리면 정렬이 되돌아감)
            P["x"], P["y"] = cur[0], cur[1]; tgt_rot = [180.0, 0.0, cur[5]]
            print(f"  정렬 후 하강 기준 x {cur[0]:.2f} y {cur[1]:.2f} rz {cur[5]:+.2f}")
        else:
            print("  ⚠ 호버 정렬 생략(--no-align)")
        descend_monitored(color, P["x"], P["y"], tgt_rot, zs, g_close)
        if align:
            HA.promote_ref(color)                          # ★안착 성공 사이클의 정렬 상태를 다음 기준으로
    except Exception as e:
        post("stop", {"dry_run": False})
        print("❌ 정지:", e)
    finally:
        speed(1)
        s_ = st(); print("현재 tcp", [round(v, 1) for v in s_["tcp"]], "grip", s_.get("gripper"), "frozen", s_.get("frozen"))


def held_wall_dots(color):
    """물고 있는 벽의 색점(손목캠) — 막힘 감시용. 서명이 없어도 색 규칙만으로 잡는다."""
    import cv2, numpy as np
    lo, hi = WALL_DOT_HSV[color]
    b = UR.urlopen("http://127.0.0.1:8766/raw", timeout=5).read()
    img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR); hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8)); m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stt, cen = cv2.connectedComponentsWithStats(m); pts = []
    for i in range(1, n):
        a = int(stt[i, 4]); x, y = cen[i]
        if 500 <= a <= 6000 and 600 <= x <= 1260 and 20 <= y <= 700:
            pts.append((float(x), float(y), a))
    pts.sort(key=lambda q: -q[2])
    return pts[:2]


JAM_PX = 6.0        # 벽 점이 죠 안에서 이만큼(≈0.7mm) 움직이면 막힘
JAM_STEP = 3.0      # 채널 안 하강 단위(mm)


def descend_monitored(color, x, y, rot, zs, g_close):
    """★막힘 감시 하강(9/5 사고 후 신설). 채널 진입부터 안착까지 JAM_STEP 씩.
    매 단계: ①TCP 도달(정체=막힘) ②그리퍼(놓침) ③손목캠 벽 점 이동(죠 안에서 밀림=막힘).
    하나라도 걸리면 즉시 정지 → 25mm 상승 → 예외. 절대 계속 밀지 않는다."""
    ref = held_wall_dots(color)
    if not ref:
        raise RuntimeError("막힘 감시용 벽 점이 손목캠에 없음 — 하강 금지")
    ref_c = (sum(p[0] for p in ref) / len(ref), sum(p[1] for p in ref) / len(ref))
    print(f"  감시 기준 벽 점 {[(round(p[0]),round(p[1])) for p in ref]}")
    z = zs + 60.0
    speed(1)
    try:
        while True:
            move([x, y, z] + rot, tol=0.8, timeout=25, tag=f"z+{z - zs:.0f}")
            g = grip_read()
            cur = held_wall_dots(color)
            # ★빈 손 = 닫힘값 그대로(빨강은 물어도 +1 뿐). red_s 는 물어도 8→8 이라 그리퍼 값으로는 못 가림
            #   → 그리퍼 값이 닫힘값이면 '손목캠 벽 점 소실' 일 때만 놓침으로 판정.
            if g.isdigit() and int(g) <= g_close and not cur:
                raise RuntimeError(f"벽 놓침(그리퍼 {g}, 벽 점 없음)")
            if cur:
                c = (sum(p[0] for p in cur) / len(cur), sum(p[1] for p in cur) / len(cur))
                d = math.hypot(c[0] - ref_c[0], c[1] - ref_c[1])
                print(f"    그리퍼 {g} · 벽 점 이동 {d:.1f}px", flush=True)
                if d > JAM_PX:
                    # ★9/5 실증: 채널 끝까지 내려간 뒤 마지막 1mm 에서 7.5px 밀림 = 밑동이 밑판에 닿은 '안착 접촉'.
                    #   바닥 근처(z_seat+8 이내)의 밀림은 막힘이 아니라 성공 신호 → 멈추고 성공 처리(더 누르지 않음).
                    if z <= zs + 8.0:
                        print(f"  ★안착 접촉: 바닥 근처에서 벽 밀림 {d:.1f}px → 정지(성공)", flush=True)
                        return
                    raise RuntimeError(f"막힘: 벽이 죠 안에서 {d:.1f}px(≈{d*0.12:.1f}mm) 밀림 (z+{z - zs:.0f})")
            else:
                print(f"    그리퍼 {g} · 벽 점 소실 → 정지", flush=True)
                raise RuntimeError("막힘 감시 불가(벽 점 소실)")
            if z <= zs + 0.01:
                break
            z = max(zs, z - JAM_STEP)
        print(f"  ★안착 z 도달, 그리퍼 {grip_read()}")
    except Exception as e:
        post("stop", {"dry_run": False}); time.sleep(0.5)
        cur = st()["tcp"]
        print(f"  ❌ {e} → 25mm 상승")
        move([cur[0], cur[1], cur[2] + 25.0] + list(cur[3:]), tol=1.0, timeout=30, tag="후퇴 +25")
        raise


def jamtest(color, secs=40):
    """★막힘 감시 벤치 테스트(로봇 정지, 채널 밖): 벽을 든 채 사용자가 죠 안에서 벽을 살짝 밀면
    벽 점 이동이 JAM_PX 를 넘는지 확인. 넘으면 'STOP' 판정 출력(실제로는 정지·상승 호출)."""
    ref = held_wall_dots(color)
    if not ref:
        print("벽 점 없음 — 벽을 물고 있어야 한다"); return
    rc = (sum(p[0] for p in ref) / len(ref), sum(p[1] for p in ref) / len(ref))
    print(f"기준 벽 점 {[(round(p[0]),round(p[1])) for p in ref]} — {secs}초 동안 벽을 죠 안에서 살짝 밀어 보세요")
    t0 = time.time(); worst = 0.0; fired = False
    while time.time() - t0 < secs:
        cur = held_wall_dots(color)
        if not cur:
            print(f"  {time.time()-t0:4.1f}s 벽 점 소실 → STOP 판정"); fired = True
        else:
            c = (sum(p[0] for p in cur) / len(cur), sum(p[1] for p in cur) / len(cur))
            d = math.hypot(c[0] - rc[0], c[1] - rc[1]); worst = max(worst, d)
            flag = "  ← STOP 판정" if d > JAM_PX else ""
            if d > JAM_PX: fired = True
            print(f"  {time.time()-t0:4.1f}s 이동 {d:5.1f}px (≈{d*0.12:4.2f}mm) 그리퍼 {grip_read()}{flag}", flush=True)
        time.sleep(0.5)
    print(f"최대 이동 {worst:.1f}px · 감지 {'작동' if fired else '미작동(밀지 않았거나 문턱 미달)'}")


RACK_REF = "/home/ar/bf2_console/rack_ref_0905.json"
RACK_MAP = "/home/ar/bf2_console/cam2robot_rack.json"
FLICKER_LADDER = (167, 250, 333, 417)


def rack_dots(color, n=4):
    """랙 픽 호버(z467)에서 벽의 색점(카메라 정면 아래). n 프레임 평균, 면적 큰 2개."""
    import cv2, numpy as np
    lo, hi = WALL_DOT_HSV[color]
    acc = []
    for _ in range(n):
        b = UR.urlopen("http://127.0.0.1:8766/raw", timeout=5).read()
        img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR); hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8)); m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        nn, lab, stt, cen = cv2.connectedComponentsWithStats(m); pts = []
        for i in range(1, nn):
            a = int(stt[i, 4]); x, y = cen[i]
            if 60 <= a <= 3000 and 20 <= x <= 1260 and 20 <= y <= 700:
                pts.append((float(x), float(y), a))
        pts.sort(key=lambda q: -q[2]); acc.append(pts[:2]); time.sleep(0.1)
    good = [p for p in acc if len(p) == 2]
    if len(good) < max(2, n // 2):
        return None
    good = [sorted(p, key=lambda q: q[1]) for p in good]
    import statistics as st_
    return [(st_.mean(g[i][0] for g in good), st_.mean(g[i][1] for g in good)) for i in range(2)]


RACK_OBS = "/home/ar/bf2_console/rack_observe_pose.json"


def _cluster_walls(pts, min_dots=2, x_gap=60.0):
    """색점들을 x-간격으로 벽 클러스터로 나눈다(같은 색 벽이 여러 슬롯이면 x 로 갈림).
    각 클러스터는 세로로 늘어선 점 min_dots 개 이상이어야 벽으로 인정(고립 유령 제거)."""
    if not pts:
        return []
    ps = sorted(pts, key=lambda p: p[0]); groups = [[ps[0]]]
    for p in ps[1:]:
        if p[0] - groups[-1][-1][0] <= x_gap:
            groups[-1].append(p)
        else:
            groups.append([p])
    return [g for g in groups if len(g) >= min_dots]


def rack_ends(color, n=4, x_hint=None):
    """★랙 관측자세에서 벽의 색점 → 양 끝점 중앙. 같은 색 벽이 여럿(red_s/red)이면 x_hint 로 고른다.
    x_hint 없으면 점이 가장 많은(가장 뚜렷한) 벽 클러스터. 고립 유령점은 클러스터에서 배제."""
    import cv2, numpy as np, math as _m
    lo, hi = WALL_DOT_HSV[color]
    acc = []
    for _ in range(n):
        b = UR.urlopen("http://127.0.0.1:8766/raw", timeout=5).read()
        img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR); hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8)); m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        nn, lab, stt, cen = cv2.connectedComponentsWithStats(m); pts = []
        for i in range(1, nn):
            a = int(stt[i, 4]); x, y = cen[i]
            if 40 < a < 3000 and 20 < x < 1260 and 20 < y < 700:
                pts.append((float(x), float(y)))
        walls = _cluster_walls(pts)
        if not walls:
            continue
        if x_hint is not None:
            walls.sort(key=lambda g: abs(sum(p[0] for p in g) / len(g) - x_hint))
        else:
            walls.sort(key=lambda g: -len(g))          # 점 가장 많은 벽
        g = walls[0]
        e = max(((_m.dist(g[i], g[j]), i, j) for i in range(len(g)) for j in range(i + 1, len(g))))
        acc.append((g[e[1]], g[e[2]], len(g)))
        time.sleep(0.1)
    if len(acc) < max(1, (n + 1) // 2):
        return None
    import statistics as st_
    mids = [((a[0][0] + a[1][0]) / 2, (a[0][1] + a[1][1]) / 2) for a in acc]
    L = [math.hypot(a[1][0] - a[0][0], a[1][1] - a[0][1]) for a in acc]
    ang = [math.degrees(math.atan2(a[1][0] - a[0][0], a[1][1] - a[0][1])) for a in acc]
    p1 = (st_.mean(a[0][0] for a in acc), st_.mean(a[0][1] for a in acc))
    p2 = (st_.mean(a[1][0] for a in acc), st_.mean(a[1][1] for a in acc))
    return {"mid": (st_.mean(m[0] for m in mids), st_.mean(m[1] for m in mids)),
            "len_px": st_.mean(L), "ang": st_.mean(ang), "n_dots": acc[-1][2], "p1": p1, "p2": p2}


def save_rack_obs():
    json.dump({"tcp": st()["tcp"], "made": time.strftime("%Y-%m-%d %H:%M")}, open(RACK_OBS, "w"), indent=1)
    print("랙 관측자세 저장:", [round(v, 1) for v in st()["tcp"][:3]])


def rack_probe():
    """★랙 호버에서 화면→로봇 매핑 1회 실측(관측자세와 같은 ±10mm 방법). 벽 점을 특징으로 쓴다."""
    import numpy as np, json as _j
    cur = st()["tcp"]; color = "blue"
    base = rack_ends(color)
    if not base:
        raise RuntimeError("랙 벽 끝점 검출 실패 — 파란 벽이 랙 관측자세에서 보여야 함")
    res = {}
    speed(10)
    for name, d in (("+X", (10, 0)), ("-X", (-10, 0)), ("+Y", (0, 10)), ("-Y", (0, -10))):
        move([cur[0] + d[0], cur[1] + d[1], cur[2]] + list(cur[3:]), tag=f"probe {name}")
        res[name] = rack_ends(color)["mid"]
        if not res[name]:
            raise RuntimeError(f"probe {name} 끝점 실패")
    move(cur, tag="복귀")
    dXpx = [(res["+X"][k] - res["-X"][k]) / 20.0 for k in range(2)]
    dYpx = [(res["+Y"][k] - res["-Y"][k]) / 20.0 for k in range(2)]
    J = np.array([[dXpx[0], dYpx[0]], [dXpx[1], dYpx[1]]]); Jinv = np.linalg.inv(J)
    _j.dump({"made": time.strftime("%Y-%m-%d %H:%M"), "tcp": cur, "J_px_per_mm": J.tolist(), "Jinv_mm_per_px": Jinv.tolist(),
             "scale_mm_per_px": float(1 / np.sqrt(abs(np.linalg.det(J))))}, open(RACK_MAP, "w"), indent=1)
    print(f"  랙 매핑 J={np.round(J, 3).tolist()} 축척 {1/np.sqrt(abs(np.linalg.det(J))):.4f}mm/px 저장")


def rack_ref(color):
    """사용자 승인 파지 자세 위 호버(z+125)에서 벽 점 2개를 '정렬 기준'으로 저장."""
    pts = rack_dots(color)
    if not pts:
        print("❌ 벽 점 2개 검출 실패"); return
    ref = json.load(open(RACK_REF)) if os.path.exists(RACK_REF) else {}
    mid = ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2)
    ang = math.degrees(math.atan2(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]))
    ref[color] = {"made": time.strftime("%Y-%m-%d %H:%M"), "tcp": st()["tcp"], "pts": pts, "mid": mid, "ang": ang}
    json.dump(ref, open(RACK_REF, "w"), ensure_ascii=False, indent=1)
    print(f"✅ 랙 정렬 기준 저장 [{color}] 중점 ({mid[0]:.1f},{mid[1]:.1f}) 각 {ang:+.2f}°")


GRIP_AXIS_PX = "/home/ar/bf2_console/grip_axis_px.json"    # 색별 그리퍼 축 픽셀(랙 뷰), 중앙 파지 기준


RACK_CALIB = "/home/ar/bf2_console/rack_calib.json"     # 색별 (관측중앙픽셀 Pc0, 검증파지XY Tg0) 한 쌍


def rack_grip_calib(color):
    """★핵심 캘리브: 지금 관측 중앙자세에서 벽 중앙픽셀 Pc0 을 재고, 검증 파지 XY(Tg0)와 짝지어 저장.
    이후 파지: 관측 → 중앙픽셀 Pc → Tg = Tg0 − Jinv·(Pc − Pc0). 벽이 랙에서 비틀려도 추종."""
    import numpy as np
    e = rack_ends(color)
    if not e:
        print("  벽 양끝 검출 실패"); return
    pick, _h, _o, _c = pick_pose(color)
    mp = json.load(open(RACK_MAP)); obs = json.load(open(RACK_OBS))["tcp"]
    d = json.load(open(RACK_CALIB)) if os.path.exists(RACK_CALIB) else {}
    d[color] = {"Pc0": list(e["mid"]), "Tg0": [pick[0], pick[1]], "ang0": e["ang"], "len0_px": e["len_px"],
                "obs_tcp": obs, "Jinv": mp["Jinv_mm_per_px"], "made": time.strftime("%Y-%m-%d %H:%M")}
    json.dump(d, open(RACK_CALIB, "w"), ensure_ascii=False, indent=1)
    print(f"✅ [{color}] 랙 캘리브: 관측중앙 ({e['mid'][0]:.1f},{e['mid'][1]:.1f}) ↔ 파지XY ({pick[0]:.1f},{pick[1]:.1f}) 각 {e['ang']:+.2f}°")


def rack_grip_xy(color):
    """관측 중앙자세에서 현재 벽 중앙 → 파지 XY 계산(캘리브 기반). 반환 (Tgx, Tgy, dang) 또는 None."""
    import numpy as np
    if not os.path.exists(RACK_CALIB):
        return None
    cal = json.load(open(RACK_CALIB)).get(color)
    if not cal:
        return None
    e = rack_ends(color, x_hint=cal["Pc0"][0])           # ★같은 색 여러 벽이면 캘리브 x 로 그 벽 선택
    if not e:
        return None
    # ★길이 게이트(9/5 실증): 벽 끝점을 하나 놓치면 길이가 짧게 나오고 '중앙'이 치우쳐 엉뚱한 곳을 문다
    #   (476px vs 캘리브 567px → 중앙 이탈 파지). 캘리브 길이와 ±10% 넘게 다르면 파지 금지.
    L0 = cal.get("len0_px")
    if L0 and abs(e["len_px"] - L0) > 0.10 * L0:
        print(f"  ❌ 벽 길이 불일치: 지금 {e['len_px']:.0f}px vs 캘리브 {L0:.0f}px "
              f"({(e['len_px']-L0)/L0*100:+.0f}%) — 끝점 미검출 의심, 파지 중단")
        return None
    Jinv = np.array(cal["Jinv"]); dpx = np.array([e["mid"][0] - cal["Pc0"][0], e["mid"][1] - cal["Pc0"][1]])
    dmm = Jinv @ dpx                                  # 벽이 화면에서 dpx 옮겨짐 = 세계에서 −dmm → 파지 XY 도 그만큼
    Tg = (cal["Tg0"][0] - dmm[0], cal["Tg0"][1] - dmm[1])
    return Tg[0], Tg[1], HG.wrap_deg(e["ang"] - cal["ang0"])


def rack_center_align(color, tol_px=1.5, max_iter=4):
    """★사용자 설계: 랙 위에서 벽 양 끝점(두 색점) → 중앙 픽셀을 그리퍼 축 픽셀에 맞춘다 → 벽 기하 중앙을 문다.
    그리퍼 축 픽셀 GRIP_AXIS_PX[color] 는 '중앙을 물었을 때 벽-중앙 픽셀이 있던 자리'로 1회 캘리브(성공 삽입에서).
    없으면 rack_ref 로 폴백."""
    import numpy as np
    if not os.path.exists(RACK_MAP):
        print("  랙 매핑 없음 → 골든 자세"); return None
    Jinv = np.array(json.load(open(RACK_MAP))["Jinv_mm_per_px"])
    ga = json.load(open(GRIP_AXIS_PX)).get(color) if os.path.exists(GRIP_AXIS_PX) else None
    if ga is None:
        print(f"  {color} 그리퍼 축 픽셀 미캘리브 → rack_align(중점 기준) 폴백"); return rack_align(color, tol_px, max_iter)
    speed(3)
    for it in range(max_iter):
        e = rack_ends(color)
        if not e:
            print("  벽 양끝 검출 실패 → 중단"); return None
        mid = e["mid"]
        dpx = (ga[0] - mid[0], ga[1] - mid[1]); err = math.hypot(*dpx)
        print(f"  랙 중앙정렬 {it}: 벽중앙 ({mid[0]:.0f},{mid[1]:.0f}) → 그리퍼축 ({ga[0]:.0f},{ga[1]:.0f})  오차 {err:.1f}px")
        if err <= tol_px:
            return mid
        dmm = np.clip(Jinv @ np.array(dpx), -6, 6); c = st()["tcp"]
        move([c[0] + dmm[0], c[1] + dmm[1], c[2]] + list(c[3:]), tol=0.5, tag=f"    중앙보정 ({dmm[0]:+.2f},{dmm[1]:+.2f})")
    return mid


def calib_grip_axis(color):
    """★현재 파지(사용자가 중앙을 문 상태) 위 랙 호버에서 벽 양끝 중앙 픽셀을 그리퍼 축으로 저장."""
    pts = rack_dots(color)
    if not pts:
        print("  벽 끝점 검출 실패"); return
    mid = ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2)
    d = json.load(open(GRIP_AXIS_PX)) if os.path.exists(GRIP_AXIS_PX) else {}
    d[color] = {"px": list(mid), "made": time.strftime("%Y-%m-%d %H:%M"), "tcp": st()["tcp"]}
    json.dump(d, open(GRIP_AXIS_PX, "w"), ensure_ascii=False, indent=1)
    print(f"✅ [{color}] 그리퍼 축 픽셀 = 벽중앙 ({mid[0]:.1f},{mid[1]:.1f}) 저장")


def rack_align(color, tol_px=1.5, max_iter=4):
    """★픽 호버에서 벽 점 중점을 기준 픽셀에 맞춘다 → 랙이 비틀려도 벽의 같은 지점을 문다."""
    import numpy as np
    if not (os.path.exists(RACK_REF) and os.path.exists(RACK_MAP)):
        print("  랙 정렬 기준/매핑 없음 → 골든 자세 그대로 파지"); return None
    ref = json.load(open(RACK_REF)).get(color)
    if not ref:
        print(f"  {color} 랙 정렬 기준 없음 → 골든 자세 그대로"); return None
    Jinv = np.array(json.load(open(RACK_MAP))["Jinv_mm_per_px"])
    speed(3)
    for it in range(max_iter):
        pts = rack_dots(color)
        if not pts:
            print("  벽 점 검출 실패 → 정렬 중단"); return None
        mid = ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2)
        dpx = (ref["mid"][0] - mid[0], ref["mid"][1] - mid[1])
        err = math.hypot(*dpx)
        ang = math.degrees(math.atan2(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]))
        print(f"  랙 정렬 {it}: 중점 오차 {err:.1f}px  각 차 {HG.wrap_deg(ang - ref['ang']):+.2f}°")
        if err <= tol_px:
            return mid
        # 점이 기준 쪽으로 dpx 만큼 가야 함 = 로봇을 Jinv·dpx 만큼(부호는 관측 매핑과 동일: 로봇 ΔX → 점 +J·ΔX)
        dmm = Jinv @ np.array(dpx)
        dmm = np.clip(dmm, -6, 6)
        c = st()["tcp"]
        move([c[0] + dmm[0], c[1] + dmm[1], c[2]] + list(c[3:]), tol=0.5, tag=f"    보정 ({dmm[0]:+.2f},{dmm[1]:+.2f})")
    return mid


def health_gate(max_try=4):
    """★검출 건강 게이트 + 적응 노출: 관측자세에서 4프레임 중 3프레임 ≥3점·rms<2mm 이어야 통과.
    실패하면 플리커 안전값 사이에서 기둥이 가장 많이 잡히는 노출로 바꾼다(바닥 미만 저장 금지)."""
    import slot_target as STG, color_lock as CL
    def score():
        """★4점 전부 검출된 프레임만 유효(3점 폴백은 go/no-go 에서 불인정 — 사용자 지시)."""
        ok = 0; rms_l = []
        for _ in range(4):
            px, why = STG.pillars_px(n=1)
            if px and len(px) == 4:
                Jinv, _m = STG.load_map(); a = json.load(open(STG.ANCH))
                pose, rms, _ = STG.base_pose_robot(px, Jinv, tuple(a["C"]), tuple(a["p0"]))
                if rms < 2.0:
                    ok += 1; rms_l.append(rms)
        return ok, (sum(rms_l) / len(rms_l) if rms_l else 99)
    ok, rms = score()
    if ok >= 3:
        print(f"  건강 게이트 통과 ({ok}/4, rms {rms:.2f})"); return True
    cur = CL.current_settings().get("exposure")
    print(f"  건강 게이트 미달 ({ok}/4) → 노출 탐색 (현재 {cur})")
    best = (ok, cur)
    for ex in FLICKER_LADDER:
        if ex == cur:
            continue
        CL.expo(set=ex); time.sleep(1.5); o, r = score(); print(f"    노출 {ex}: {o}/4 rms {r:.2f}")
        if o > best[0]:
            best = (o, ex)
    CL.expo(set=best[1]); time.sleep(1.2)
    if best[0] >= 3:
        stf = json.load(open(CL.STORE)) if os.path.exists(CL.STORE) else {}
        stf.update(apply={"set": best[1]}, made=time.strftime("%Y-%m-%d %H:%M"), note="health_gate 적응 노출")
        json.dump(stf, open(CL.STORE, "w"), ensure_ascii=False, indent=1)
        print(f"  노출 {best[1]} 채택·저장 ({best[0]}/4)"); return True
    print("  ❌ 어떤 노출에서도 기둥 검출 부족 — 정지"); return False


def release_and_retreat(color):
    """안착된 벽을 놓고(그리퍼 열기) 관측자세로 복귀. 부품은 그 자리에 둔다."""
    refs = json.load(open(CAL))["refs"][color]
    print("  그리퍼 열기 →", gripper(refs["grip_open"]))
    speed(SPD_SEAT); cur = st()["tcp"]
    move([cur[0], cur[1], cur[2] + 30] + list(cur[3:]), tag="벽 위로 +30")
    speed(SPD_DESC); move([cur[0], cur[1], HOVER_Z] + list(cur[3:]), tag="호버 z478")
    speed(SPD_MOVE); move([cur[0], cur[1], SAFE_Z] + list(cur[3:]), tag="SAFE")
    move(OBS, tag="관측자세"); speed(1)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "plan"
    color = sys.argv[2] if len(sys.argv) > 2 else "yellow"
    if mode == "plan":
        P = plan(color)
        print(f"베이스(로봇) x {P['base'].x:.2f} y {P['base'].y:.2f} yaw {P['base'].yaw_deg:+.3f}° rms {P['rms']:.2f}mm")
        print(f"목표 [{color}] x {P['x']:.2f} y {P['y']:.2f} rz {P['rz']:+.2f}  (기준점 {P['anchor']} {P['anchor_made']})")
    elif mode == "run":
        tgt = None
        if "--target" in sys.argv:
            i = sys.argv.index("--target"); tgt = (float(sys.argv[i + 1]), float(sys.argv[i + 2]), float(sys.argv[i + 3]))
        run(color, seat="--seat" in sys.argv, do_pick="--no-pick" not in sys.argv, target=tgt,
            align="--no-align" not in sys.argv, len_gate="--len-gate" in sys.argv, grasp_gate="--no-grasp-gate" not in sys.argv,
            grasp_teach="--grasp-teach" in sys.argv)
    elif mode == "held_depth":          # 벽 든 채(어느 높이든): 뎁스·축척·축 각
        m, why = held_wall_depth(color)
        print("❌ " + why if m is None else f"[{color}] 든 벽 뎁스 {m['d_w']:.0f}mm · {m['mm_px']:.4f}mm/px · 축 {m['ang_img']:+.2f}° · 폭 {m['span_px']:.0f}px · 중심 ({m['center_px'][0]:.0f},{m['center_px'][1]:.0f}) · 점 {m['n']}")
    elif mode == "rack_len":            # 랙 관측자세에서: 색점 양끝 + 뎁스 물리 길이 검산
        cal = json.load(open(RACK_CALIB)).get(color) if os.path.exists(RACK_CALIB) else None
        e = rack_ends(color, x_hint=cal["Pc0"][0] if cal else None)
        if not e: print("❌ 색점 양끝 검출 실패")
        else:
            print(f"[{color}] 색점: 중앙 ({e['mid'][0]:.0f},{e['mid'][1]:.0f}) 길이 {e['len_px']:.0f}px 각 {e['ang']:+.2f}°")
            rack_len_check(color, e, strict=False)
    elif mode == "release":
        release_and_retreat(color)
    elif mode == "nudge":
        # 현재 높이에서 로봇 X/Y(mm)·rz(°) 소폭 이동 — 기둥 위 높이에서만 쓸 것(채널 안 XY 이동 금지)
        dx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
        dy = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        drz = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        c = st()["tcp"]
        if c[2] < 430.0:
            print(f"❌ z {c[2]:.1f} < 430 (기둥 꼭대기 아래) — XY 이동 금지"); sys.exit(1)
        if abs(dx) > 6 or abs(dy) > 6 or abs(drz) > 3:
            print("❌ 한 번에 6mm/3° 이하로"); sys.exit(1)
        speed(1); move([c[0] + dx, c[1] + dy, c[2], c[3], c[4], HG.wrap_deg(c[5] + drz)], tol=0.5, tag=f"nudge {dx:+.1f},{dy:+.1f},{drz:+.1f}")
        print("그리퍼", grip_read())
    elif mode == "rack_probe":
        rack_probe()
    elif mode == "rack_ref":
        rack_ref(color)
    elif mode == "calib_grip_axis":
        calib_grip_axis(color)
    elif mode == "rack_calib":
        rack_grip_calib(color)
    elif mode == "rack_gripxy":
        r=rack_grip_xy(color); print("파지XY", tuple(round(v,2) for v in r[:2]),"각차",round(r[2],2)) if r else print("캘리브/검출 없음")
    elif mode == "jamtest":
        jamtest(color, int(sys.argv[3]) if len(sys.argv) > 3 else 40)
    elif mode == "grasp":
        capture_grasp_sig(color, int(sys.argv[3]) if len(sys.argv) > 3 else 15)
    else:
        print(__doc__)
