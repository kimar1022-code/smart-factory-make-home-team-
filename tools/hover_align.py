#!/usr/bin/env python3
"""호버 정렬(설계 4단계) — z_seat+85 에서 **든 벽 점 ↔ 베이스 특징(기둥 점·안착된 벽 점)** 을 같은 프레임에서 기준 관계에
맞춘다. 카메라 2대: 손목캠(:8766, 앞끝) + 새카메라(:8768, 뒤끝). 9/5 밤 신설(사용자 설계).

왜: 목표 TCP 계산(z650 관측 1회)은 파지 치우침·베이스 미세 이동을 못 흡수했고 그 빈자리를 사용자 육안 nudge 가 메웠다.
    이 모듈이 그 nudge 를 대신한다. 그리퍼가 벽 어디를 물었든 이 정렬 뒤엔 무관해진다.

원리(카메라별 동일):
  · 기준(ref): 사용자가 "맞다" 한 정렬 상태에서 [베이스 특징 P_ref, 든 벽 점 W_ref(1~2개)] 저장.
  · 지금: P_ref→P_now 2D 유사변환 S(베이스 기준계) → W_exp=S(W_ref) → Δ=W_now−W_exp (px).
  · 로봇 mm 환산은 카메라 종류로 다르다:
      손목캠(moving="base"): 로봇이 δ 움직이면 **베이스 px 가** J·δ 움직이고 벽 px 는 그대로 → δ = +Jinv·Δ.
        매핑 = 관측자세 매핑을 높이(d_h/377)와 rz(R(rz−180)) 로 환산. ★rz 회전 없으면 노랑(rz 90)에서 90° 틀림(오프라인 실증).
      새카메라(moving="wall"): 고정 카메라라 **벽 px 가** J·δ 움직이고 베이스는 그대로 → δ = −Jinv·Δ.
        매핑 = `probe newcam` 으로 z440 에서 ±10mm 조그해 실측(cam2robot_newcam.json).
  · 회전: 로봇 +rz → 화면 각 변화 = ROT_SIGN[src]·(+rz). 손목캠 −1(거울 매핑), 새카메라는 probe 가 재서 저장. 미검증 → 발산 가드.
  · 두 카메라가 다 있으면 XY 는 평균, 서로 COMBINE_TOL 넘게 다르면 정지. rz 는 벽 점 2개인 카메라 우선.
  · 벽 점 1개(노랑·red_s)인 카메라는 위치만, 회전은 −θ(베이스가 돈 만큼) 가정.

게이트(하나라도 걸리면 하강 금지·정지·보고, 자동 재시도 없음):
  특징 매칭 ≥2 · 벽 점 ≥1 · 축척 |s−1|≤3% · rms≤6px · 카메라 간 불일치 ≤1.5mm/0.6° · MAX_ITER 수렴(0.3mm/0.15°) · 발산 즉시 정지

  hover_align.py ref   <색> [--src wrist|newcam] [--wall x,y[,x,y]]   # 지금 프레임을 기준으로(새카메라는 벽 점 좌표 지정)
  hover_align.py check <색>                                           # 모든 가용 카메라 Δ(이동 없음)
  hover_align.py align <색> [--dry]                                   # 보정 루프(1% 속도, ≤3mm/0.5° 스텝)
  hover_align.py probe <newcam|side> <색> --wall x,y[,x,y]           # 고정카메라 매핑(벽 든 채 z440, ±10mm 조그)
  hover_align.py detect <색> [--src side|newcam|wrist]                # 지금 프레임 검출만 출력(기준 없이)
  hover_align.py test  <색> ref.jpg now.jpg [z] [--rz r] [--save out]   # 손목캠 저장 프레임 오프라인 검증
"""
import sys, os, json, math, time
import urllib.request as UR
import numpy as np
import cv2

sys.path.insert(0, "/home/ar/bf2_console/tools")
import pillar_dots as PD
import house_geometry as HG

BR = "http://127.0.0.1:8765"
MAP = "/home/ar/bf2_console/cam2robot_observe.json"
NEWCAM_MAP = "/home/ar/bf2_console/cam2robot_newcam.json"
REF = "/home/ar/bf2_console/hover_ref_0905.json"
D_OBS, Z_OBS, RZ_MAP = 377.0, 650.0, 180.0
# 카메라 3대. moving: 로봇이 움직일 때 화면에서 움직이는 쪽(손목캠=베이스, 고정캠=든 벽). 매핑 파일은 고정캠은 probe 로 생성.
SRC = {"wrist":  {"url": "http://127.0.0.1:8766/raw", "moving": "base", "map": None},
       "newcam": {"url": "http://127.0.0.1:8768/raw", "moving": "base", "map": "/home/ar/bf2_console/cam2robot_newcam.json"},   # 9/6 13:05 실증: 손목 뎁스캠 180° 반대편에 부착 → 로봇과 함께 움직임(베이스 px 가 이동, 든 벽은 고정). 매핑은 probe_arm
       "side":   {"url": "http://127.0.0.1:8771/raw", "moving": "wall", "map": "/home/ar/bf2_console/cam2robot_side.json"}}
ROT_SIGN = {"wrist": +1.0}                      # 고정캠은 probe 가 rot_sign 을 파일에 저장
# 소스별 검출 파라미터(9/6 새벽 라이브 프레임 실측): 새카메라 파랑 V≈100(손목캠 범위 V140 미달), 측면캠 640×480 점 면적 29~118
# 9/6 19:1x: 빨강 랩어라운드 수정 후 근거리(z440~470)에서 기둥 점이 2300px² 까지 커져 옛 상한 1400 에 걸려 사라짐 → 3200 으로.
#   (newcam 상한은 fixed_cam_seeds 가 '든 벽 점 = 상한 초과' 로 쓰므로 그대로 둔다)
DET = {"wrist":  {"amin_wall": 150, "feat_area": (60, 3200), "near": 80.0, "search": 140.0, "ranges": None},
       "newcam": {"amin_wall": 40,  "feat_area": (20, 1400), "near": 60.0, "search": 120.0,
                  "ranges": {"blue": ((95, 150, 70), (118, 255, 255)), "yellow": ((15, 80, 110), (40, 255, 255)),
                             "red": [((0, 100, 80), (10, 255, 255)), ((160, 100, 80), (180, 255, 255))]},
                  "exclude": [(240, 370, 420, 470), (360, 650, 560, 720)]},   # 카메라에 붙어 같이 움직이는 고정물: 왼쪽 빨간 케이블(13:12 프로브 8px, 14:56 (272,427)), 아래 파란 점(4px)
       # 9/7: 측면캠을 1280×720 으로 올림(같은 거리에서 점 면적 16~31 → 51~115, σ 0.03~0.2px) → 면적·반경 기준 2배로
       "side":   {"amin_wall": 30,  "feat_area": (25, 1000), "near": 60.0, "search": 120.0,
                  # 측면 빨간 기둥점 H 0~5(라이브 실측, 랩어라운드) → 두 구간 합집합
                  "ranges": {"blue": ((95, 120, 80), (125, 255, 255)), "yellow": ((15, 60, 100), (40, 255, 255)),
                             "red": [((0, 60, 60), (12, 255, 255)), ((160, 60, 60), (180, 255, 255))]},
                  "exclude": [(740, 410, 820, 500), (1150, 60, 1280, 320), (820, 560, 980, 720)]}}   # 9/7 1280×720 좌표(옛 640×480 값 2배) + 창 반사·바닥 반사(면적 1600·2192) 제외          # 검은 상자 위 파란 점(고정물, 베이스 아님) 제외
HELD_BOX = (820, 0, 1280, 720)                  # 손목캠에서 든 벽이 보이는 영역(ref 생성 시)
SCALE_TOL, RMS_TOL_PX = 0.03, 6.0
MAX_STEP_MM, MAX_STEP_DEG = 3.0, 0.5
TOL_MM, TOL_DEG = 0.3, 0.15
COMBINE_TOL_MM, COMBINE_TOL_DEG = 1.5, 0.6
MAX_ITER = 10                                   # 스텝 ≤3mm 라 20mm 급 초기 오차(회전 중심 버그 전) 도 수렴하게
# ★z440 노출: 관측자세(z650)용 417 이면 가까워진 기둥 점이 하얗게 날아간다(9/6 00:5x 라이브: 노랑 S4·V255, 파랑 H90·V251 → 검출 0).
#   이것이 "벽 물고 가까워지면 기둥점·벽점 인식 안 됨"의 원인. 라이브 재현(00:5x, 관측 노출 417): 333/250/167 은 파랑만, 83 에서 파랑+노랑.
#   호버 정렬은 플리커 안전값 사다리(250·167·83)를 전부 시도해 기준 특징이 가장 많이 매칭되는 노출을 고르고, 끝나면 관측 노출로 복원.
HOVER_EXPO_LADDER = (250, 167, 83)
NEWCAM_MAP = SRC["newcam"]["map"]
WALL_DOT_HSV = {
    "blue":   ((95, 150, 140), (115, 255, 255)),
    "yellow": ((15, 80, 110), (38, 255, 255)),
    # 9/6 20:2x 실측(빨강 긴 벽 든 손목캠): 빨강 점 H 가 175 를 넘어가 조각남(306+240px) → 상한 179 로 하나(2424px).
    #   0~8 구간까지 더해도 차이 없어 단일 범위 유지(place_calc 6곳이 lo,hi 튜플을 그대로 씀).
    "red":    ((135, 90, 55), (179, 255, 255)),
    "red_s":  ((135, 90, 55), (179, 255, 255)),
}


# ------------------------------------------------------------------ 검출
def grab(src="wrist"):
    b = UR.urlopen(SRC[src]["url"], timeout=5).read()
    return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)


def _blobs(img, color, amin, src="wrist"):
    rng = DET[src]["ranges"]
    r = (rng[color if color != "red_s" else "red"] if rng else WALL_DOT_HSV[color])
    ranges = r if isinstance(r, list) else [r]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = None
    for lo, hi in ranges:
        mm = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        m = mm if m is None else cv2.bitwise_or(m, mm)
    if src == "wrist":
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    out = [(float(cen[i][0]), float(cen[i][1]), int(st[i, 4])) for i in range(1, n) if st[i, 4] >= amin]
    ex = DET[src].get("exclude") or []
    return [q for q in out if not any(x0 <= q[0] <= x1 and y0 <= q[1] <= y1 for x0, y0, x1, y1 in ex)]


def _merge_fragments(pts, r):
    """★9/6 라이브: 프레임 아래쪽 벽 점이 그늘로 94/61/61px 조각 3개로 갈라져 검출 실패 → r px 안 조각을 하나로 합친다(면적 가중 중심)."""
    out = []
    for x, y, a in sorted(pts, key=lambda q: -q[2]):
        for m in out:
            if math.hypot(m[0] - x, m[1] - y) <= r:
                A = m[2] + a; m[0] = (m[0] * m[2] + x * a) / A; m[1] = (m[1] * m[2] + y * a) / A; m[2] = A; break
        else:
            out.append([x, y, a])
    return [(m[0], m[1], int(m[2])) for m in out]


def wall_dots(img, color, ref=None, seeds=None, src="wrist"):
    """든 벽 점(1~2). ref 있으면 기준 자리 ±near 의 같은 색 점(물린 벽은 화면에서 거의 안 움직임).
    seeds(ref 생성용 좌표) 있으면 그 근처. 둘 다 없으면(손목캠) HELD_BOX 안 큰 점."""
    pts = _merge_fragments(_blobs(img, color, 30 if src == "wrist" else 8, src), 30.0 if src == "wrist" else 10.0)
    pts = [q for q in pts if q[2] >= DET[src]["amin_wall"]]
    near = DET[src]["near"]
    anchors = [(p[0], p[1]) for p in ref["wall"]] if ref else (seeds or None)
    if anchors:
        out = []
        for rx, ry in anchors:
            c = [q for q in pts if math.hypot(q[0] - rx, q[1] - ry) <= near]
            if c:
                out.append(min(c, key=lambda q: math.hypot(q[0] - rx, q[1] - ry)))
        return out
    if src != "wrist":
        return []
    x0, y0, x1, y1 = HELD_BOX
    pts = [q for q in pts if x0 <= q[0] <= x1 and y0 <= q[1] <= y1 and q[2] >= 300]
    pts.sort(key=lambda q: -q[2])
    return pts[:2]


def base_feats(img, exclude, src="wrist"):
    """베이스 고정 특징 = 색점 중 든 벽 점이 아닌 것 전부(기둥 꼭대기 + 안착 벽 점). [(color,x,y,area)]"""
    lo_a, hi_a = DET[src]["feat_area"]
    if src == "wrist":
        d = PD.detect(img, None)
    else:
        d = {c: _blobs(img, c, lo_a, src) for c in ("blue", "yellow", "red")}
    out = []
    for c, lst in d.items():
        for p in lst:
            if not (lo_a <= p[2] <= hi_a):
                continue
            if any(math.dist(p[:2], e[:2]) < (25 if src == "wrist" else 12) for e in exclude):
                continue
            out.append((c, p[0], p[1], p[2]))
    return out


def measure(img, color, ref=None, seeds=None, src="wrist"):
    w = wall_dots(img, color, ref, seeds, src)
    if len(w) < 1:
        return None, f"[{src}] 든 벽 점 0개(벽을 안 들었거나 기준 자리에 없음)"
    w = sorted(w, key=lambda q: (q[1], q[0]))
    return {"wall": [(q[0], q[1], q[2]) for q in w], "pillars": base_feats(img, w, src)}, None


def set_expo(val):
    """손목캠 노출(플리커 안전값만). color_lock 저장은 건드리지 않는다(관측자세 값은 그대로, 정렬 뒤 restore)."""
    try:
        import color_lock as CL
        CL.expo(set=int(val)); time.sleep(0.9); return True
    except Exception as e:
        print("  ⚠ 노출 설정 실패:", e); return False


def restore_expo():
    try:
        import color_lock as CL
        st_ = json.load(open(CL.STORE)) if os.path.exists(CL.STORE) else {}
        ap = st_.get("apply") or {}
        if ap.get("set"): CL.expo(set=int(ap["set"]), **({"gain": int(ap["gain"])} if ap.get("gain") else {})); time.sleep(0.6)
    except Exception as e:
        print("  ⚠ 노출 복원 실패:", e)


def pick_hover_expo(color, ref):
    """★z440 노출 자동 선택(9/6 00:5x 라이브 실측: 417→파랑·노랑 둘 다 날아감 / 333·250·167→파랑만 / **83→파랑+노랑 둘 다**).
    사다리 전부 시도해 **기준 특징과 매칭되는 수가 최대**인 노출을 고른다(첫 성공에서 멈추면 333 에서 파랑만 잡고 끝남)."""
    best = None
    for e in HOVER_EXPO_LADDER:
        set_expo(e)
        meas, why = measure(grab("wrist"), color, ref, None, "wrist")
        nw = len(meas["wall"]) if meas else 0
        nm = len(match_feats(ref["pillars"], meas["pillars"], DET["wrist"]["search"])[0]) if meas else 0
        print(f"  호버 노출 {e}: 벽 점 {nw} 매칭 특징 {nm}")
        key = (nm, nw)
        if nw >= 1 and nm >= 2 and (best is None or key > best[0]):
            best = (key, e)
    if best is None:
        return None
    set_expo(best[1]); print(f"  → 호버 노출 {best[1]} 채택(매칭 {best[0][0]})"); return best[1]


# ------------------------------------------------------------------ 기하
def _rot(th):
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    return np.array([[c, -s], [s, c]])


def similarity(src, dst):
    A = np.array(src, float); B = np.array(dst, float)
    ca, cb = A.mean(0), B.mean(0); A0, B0 = A - ca, B - cb
    za = A0[:, 0] + 1j * A0[:, 1]; zb = B0[:, 0] + 1j * B0[:, 1]
    den = float((np.abs(za) ** 2).sum())
    if den < 1e-9 or len(src) < 2:
        s, th = 1.0, 0.0
    else:
        z = complex((np.conj(za) * zb).sum() / den); s, th = abs(z), math.degrees(math.atan2(z.imag, z.real))
    t = cb - s * (_rot(th) @ ca)
    fit = (s * (_rot(th) @ A.T)).T + t
    return s, th, float(t[0]), float(t[1]), float(np.sqrt(((fit - B) ** 2).sum(1).mean()))


def apply_sim(sim, pts):
    s, th, tx, ty, _ = sim
    P = np.array([p[:2] for p in pts], float)
    return (s * (_rot(th) @ P.T)).T + np.array([tx, ty])


def wall_mid_ang(w):
    (x1, y1), (x2, y2) = w[0][:2], w[1][:2]
    return ((x1 + x2) / 2, (y1 + y2) / 2), math.degrees(math.atan2(y2 - y1, x2 - x1))


def match_feats(ref_p, now_p, search=None):
    search = search or DET["wrist"]["search"]
    src, dst, lab, used = [], [], [], set()
    for c, x, y, *_ in ref_p:
        best = None
        for j, (c2, x2, y2, a2) in enumerate(now_p):
            if c2 != c or j in used:
                continue
            d = math.hypot(x2 - x, y2 - y)
            if d <= search and (best is None or d < best[0]):
                best = (d, j)
        if best is not None:
            used.add(best[1]); j = best[1]
            src.append((x, y)); dst.append((now_p[j][1], now_p[j][2]))
            lab.append(f"{c}({x:.0f},{y:.0f})→({now_p[j][1]:.0f},{now_p[j][2]:.0f}) {best[0]:.0f}px")
    return src, dst, lab


def jinv_for(src, z_tcp, rz_tcp):
    """카메라별 (Jinv[mm/px, 로봇 프레임], rot_sign). 손목캠: 높이·rz 환산. 고정캠(newcam/side): probe 파일 그대로."""
    if src == "wrist":
        m = json.load(open(MAP)); d_h = D_OBS - (Z_OBS - z_tcp)
        if d_h < 60:
            raise RuntimeError(f"z{z_tcp:.0f}: 기둥 뎁스 {d_h:.0f}mm — 매핑 환산 불가")
        return _rot(HG.wrap_deg(rz_tcp - RZ_MAP)) @ (np.array(m["Jinv_mm_per_px"], float) * (d_h / D_OBS)), ROT_SIGN["wrist"]
    f = SRC[src]["map"]
    if not os.path.exists(f):
        raise RuntimeError(f"{src} 매핑 없음 — `hover_align.py probe {src} <색> --wall x,y`")
    m = json.load(open(f))
    if abs(z_tcp - m["z"]) > 5:
        raise RuntimeError(f"{src} 매핑은 z{m['z']:.0f} 용, 지금 z{z_tcp:.0f}")
    J = np.array(m["Jinv_mm_per_px"], float)
    if SRC[src]["moving"] == "base":
        # ★손목에 붙은 카메라(새카메라): rz 가 돌면 화면도 같이 돈다 → 매핑 촬영 시 rz 대비 차이만큼 회전.
        #   9/6 19:0x: 매핑은 rz −180 에서 측정. 노랑 슬롯은 rz 90 → 이 보정 없으면 90° 틀린 방향으로 감(9/2 '축매핑 발산 3회' 와 같은 함정).
        rz_map = float((m.get("tcp") or [0, 0, 0, 0, 0, RZ_MAP])[5])
        J = _rot(HG.wrap_deg(rz_tcp - rz_map)) @ J
    return J, m.get("rot_sign", -1.0)


def delta(ref, meas, z_tcp, rz_tcp, src="wrist"):
    s_, d_, lab = match_feats(ref["pillars"], meas["pillars"], DET[src]["search"])
    if len(s_) < 1:
        return None, f"[{src}] 베이스 특징 매칭 {len(s_)}개(1 이상 필요) — 후보 {[(c, round(x), round(y)) for c, x, y, a in meas['pillars']]}"
    if len(s_) == 1:
        # 15:43 실기: 베이스 −2° 회전으로 새카메라에 기둥 1개만 → 평행이동만(s=1, θ=0). 회전은 rz 담당 카메라(손목 2점)가 맡는다.
        #   베이스 회전분의 벽 기대위치 오차 ≈ 기둥↔벽 거리(≈120px)×sin(θ) — 2° 에 4px(1mm) 수준.
        sim = (1.0, 0.0, float(d_[0][0] - s_[0][0]), float(d_[0][1] - s_[0][1]), 0.0)
        lab = lab + ["(1특징: 평행이동만)"]
    else:
        sim = similarity(s_, d_)
    s, th, tx, ty, rms = sim
    if abs(s - 1.0) > SCALE_TOL:
        return None, f"[{src}] 특징 축척 {s:.3f} — 기준과 높이/거리가 다름"
    if rms > RMS_TOL_PX:
        return None, f"[{src}] 특징 맞춤 rms {rms:.1f}px > {RMS_TOL_PX} — 오매칭 의심(안착 벽 이동/점 뒤바뀜)"
    w_exp = apply_sim(sim, ref["wall"]); w_now = np.array([p[:2] for p in meas["wall"]], float)
    one_dot = len(meas["wall"]) < 2 or len(ref["wall"]) < 2
    if one_dot:
        w_exp, w_now = w_exp[:1], w_now[:1]
        mid_exp, mid_now = tuple(w_exp[0]), tuple(w_now[0]); dang = -th
    else:
        if np.linalg.norm(w_now - w_exp) > np.linalg.norm(w_now[::-1] - w_exp):
            w_now = w_now[::-1]
        mid_exp, ang_exp = wall_mid_ang(w_exp); mid_now, ang_now = wall_mid_ang(w_now)
        dang = HG.wrap_deg(ang_now - ang_exp)
        if dang > 90: dang -= 180
        if dang < -90: dang += 180
    dpx = (mid_now[0] - mid_exp[0], mid_now[1] - mid_exp[1])
    Jinv, rsign = jinv_for(src, z_tcp, rz_tcp)
    dmm = Jinv @ np.array(dpx)
    if SRC[src]["moving"] == "wall":
        dmm = -dmm                     # 벽이 로봇과 함께 움직이는 카메라: Δ 를 없애려면 반대로
    return {"src": src, "dpx": dpx, "dang_img": dang, "dmm": (float(dmm[0]), float(dmm[1])), "drz": rsign * dang,
            "sim": {"s": s, "theta": th, "rms": rms}, "matched": lab, "one_dot": one_dot,
            "scale_mm_px": float(np.hypot(*Jinv[:, 0])), "rz": rz_tcp}, None


# ------------------------------------------------------------------ 로봇
def st():
    return json.loads(UR.urlopen(BR + "/status", timeout=6).read())["robots"]["fr5"]


def post(a, b):
    r = UR.Request(BR + "/" + a, json.dumps(b).encode(), {"Content-Type": "application/json"})
    return UR.urlopen(r, timeout=20).read().decode()


def move_rel(dx, dy, drz, tol=0.3, timeout=40):
    """1% 속도 상대 이동. 도달은 TCP 폴링(9/5: busy 조기 False 오판 사고)."""
    c = st()["tcp"]
    tgt = [c[0] + dx, c[1] + dy, c[2], c[3], c[4], HG.wrap_deg(c[5] + drz)]
    # 9/6 12:56 실기: 브리지 fr5/move 는 관절 명령(joints 6개 요구) → 422. 카테시안은 move_tcp + dry_run 선검사(place_calc.move 와 동일 계약).
    post("fr5/speed", {"value": 1, "dry_run": False})
    r = json.loads(post("fr5/move_tcp", {"tcp": tgt, "dry_run": True}))
    if r.get("result") != "dry_run":
        raise RuntimeError(f"상대이동 dry_run 거부 {r}")
    r = json.loads(post("fr5/move_tcp", {"tcp": tgt, "dry_run": False}))
    if r.get("result") not in ("started", "ok"):
        raise RuntimeError(f"상대이동 거부 {r}")
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.3); n = st()["tcp"]
        if max(abs(n[i] - tgt[i]) for i in range(3)) <= tol and abs(HG.wrap_deg(n[5] - tgt[5])) <= 0.1:
            return n
    raise RuntimeError(f"이동 미도달 (목표 {[round(v, 2) for v in tgt]} 현재 {[round(v, 2) for v in st()['tcp']]})")


# ------------------------------------------------------------------ 기준·측정·정렬
def load_ref(color, src=None):
    """색별 기준 {"wrist": {...}, "newcam": {...}}. src 주면 그 카메라 것만."""
    if not os.path.exists(REF):
        return None
    d = json.load(open(REF)).get(color)
    if not d:
        return None
    return d.get(src) if src else d


def save_ref(color, src="wrist", seeds=None, img=None):
    img = img if img is not None else grab(src)
    meas, why = measure(img, color, None, seeds, src)
    if not meas:
        raise RuntimeError(why)
    if len(meas["pillars"]) < 2:
        raise RuntimeError(f"[{src}] 베이스 특징 {len(meas['pillars'])}개 — 이 자세에선 기준을 못 만든다(둘 다 보이는 자세 필요)")
    if len(meas["wall"]) < 2:
        print(f"  ⚠ [{src}] 든 벽 점 {len(meas['wall'])}개 — 위치만 정렬, 회전은 다른 카메라/랙 각으로")
    tcp = st()["tcp"]
    d = json.load(open(REF)) if os.path.exists(REF) else {}
    expo = None
    if src == "wrist":
        try:
            expo = json.loads(UR.urlopen("http://127.0.0.1:8766/expo", timeout=3).read()).get("exposure")
        except Exception:
            pass
    d.setdefault(color, {})[src] = {"wall": meas["wall"], "pillars": meas["pillars"], "tcp": tcp, "z": tcp[2], "expo": expo,
                                    "made": time.strftime("%Y-%m-%d %H:%M"), "note": "사용자 정렬 확인 상태(하강 직전)"}
    json.dump(d, open(REF, "w"), ensure_ascii=False, indent=1)
    print(f"✅ [{color}/{src}] 호버 기준 저장: 벽 점 {[(round(x), round(y)) for x, y, a in meas['wall']]} "
          f"베이스 특징 {len(meas['pillars'])}개 z{tcp[2]:.0f} rz{tcp[5]:.1f}")


def promote_ref(color, note="삽입 성공 사이클에서 자동 승격"):
    """★삽입 성공 직후 호출: 성공한 사이클의 정렬 상태(마지막 check 프레임)를 기준으로 승격. 손으로 맞춘 기준보다 정확."""
    last = promote_ref.last.get(color) if hasattr(promote_ref, "last") else None
    if not last:
        return False
    d = json.load(open(REF)) if os.path.exists(REF) else {}
    for src, m in last.items():
        d.setdefault(color, {})[src] = dict(m, made=time.strftime("%Y-%m-%d %H:%M"), note=note)
    json.dump(d, open(REF, "w"), ensure_ascii=False, indent=1)
    print(f"  ✅ [{color}] 호버 기준 승격({', '.join(last)})"); return True
promote_ref.last = {}


def available_sources(color):
    refs = load_ref(color) or {}
    out = []
    for src in ("wrist", "newcam", "side"):
        if src not in refs:
            continue
        if SRC[src]["map"] and not os.path.exists(SRC[src]["map"]):
            print(f"  ⚠ {src} 기준은 있으나 매핑 없음(probe 필요) → 제외"); continue
        try:
            UR.urlopen(SRC[src]["url"].replace("/raw", "/health"), timeout=2).read()
        except Exception:
            print(f"  ⚠ {src} 카메라 응답 없음 → 제외"); continue
        out.append(src)
    return out


def check(color, srcs=None, roles=None):
    """가용 카메라 전부 Δ. 반환 (combined, per_src, why). combined=None 이면 하강 금지.
    roles: {src: "xy"|"rz"|"both"|"measure"} — 결합에 쓰는 역할(measure=측정·승격만, 결합 제외). 없으면 전부 both."""
    if not hasattr(check, "expo_done"):
        check.expo_done = False
    roles = roles or {}
    srcs = srcs or (list(roles) if roles else None) or available_sources(color)
    if not srcs:
        return None, {}, f"{color} 호버 기준 없음 (hover_align.py ref {color} [--src newcam --wall x,y])"
    t = st()["tcp"]; z, rz = t[2], t[5]
    per, last = {}, {}
    for src in srcs:
        ref = load_ref(color, src)
        if abs(z - ref["z"]) > 3.0:
            return None, per, f"[{src}] 높이 z{z:.0f} ≠ 기준 z{ref['z']:.0f}"
        img = grab(src)
        meas, why = measure(img, color, ref, None, src)
        if src == "wrist" and (not meas or len(meas["pillars"]) < 2) and not check.expo_done:
            check.expo_done = True
            if pick_hover_expo(color, ref) is not None:
                meas, why = measure(grab(src), color, ref, None, src)
        if not meas:
            return None, per, why
        D, why = delta(ref, meas, z, rz, src)
        if D is None:
            return None, per, why
        per[src] = D
        last[src] = {"wall": meas["wall"], "pillars": meas["pillars"], "tcp": t, "z": z}
    promote_ref.last[color] = last
    # 결합: XY 평균(불일치 게이트), rz 는 점 2개 카메라 우선 — roles 가 있으면 역할별로
    xy_keys = [k for k in per if roles.get(k, "both") in ("xy", "both")]
    rz_keys = [k for k in per if roles.get(k, "both") in ("rz", "both")]
    if not xy_keys:
        return None, per, "XY 담당 카메라 없음(roles)"
    for i in range(len(xy_keys)):
        for j in range(i + 1, len(xy_keys)):
            A, B_ = per[xy_keys[i]], per[xy_keys[j]]
            dd = math.dist(A["dmm"], B_["dmm"])
            if dd > COMBINE_TOL_MM:
                return None, per, f"카메라 불일치({xy_keys[i]}↔{xy_keys[j]}) XY {dd:.2f}mm — 매핑/기준 의심, 정지"
    xs = [per[k]["dmm"] for k in xy_keys]
    mx = sum(v[0] for v in xs) / len(xs); my = sum(v[1] for v in xs) / len(xs)
    two = [per[k] for k in rz_keys if not per[k]["one_dot"]]
    if len(two) >= 2:
        da = max(abs(HG.wrap_deg(a["drz"] - b["drz"])) for a in two for b in two)
        if da > COMBINE_TOL_DEG:
            return None, per, f"카메라 rz 불일치 {da:.2f}° — 매핑/기준 의심, 정지"
    if two:
        drz = sum(D["drz"] for D in two) / len(two); rz_from = "2점(" + ",".join(D["src"] for D in two) + ")"
    elif rz_keys:
        drz = per[rz_keys[0]]["drz"]; rz_from = f"1점 {rz_keys[0]}(−θ 가정)"
    else:
        # rz 담당 카메라 없음(전부 xy/measure) = ★rz 고정 모드: 상대 회전을 아예 안 준다.
        #   9/6 17:4x 실측 — 절대 rz 명령은 0.01° 안에 정확히 도달, 상대 회전 누적은 177.85~179.90 로 흩어짐.
        #   rz 는 운반 단계의 절대 명령(슬롯 기준 rz + 베이스 Δyaw)으로 확정하고 정렬은 XY 만 맞춘다(사용자 설계, 9/2 홈포즈 방식과 동일).
        drz = 0.0; rz_from = "고정(절대 rz 유지)"
    return {"dmm": (mx, my), "drz": drz, "n_src": len(xy_keys), "rz_from": rz_from}, per, None


def fmt(D):
    return (f"[{D['src']}{' 점1개' if D.get('one_dot') else ''}] Δpx ({D['dpx'][0]:+.1f},{D['dpx'][1]:+.1f}) 각 {D['dang_img']:+.2f}° → "
            f"XY ({D['dmm'][0]:+.2f},{D['dmm'][1]:+.2f})mm rz {D['drz']:+.2f}° "
            f"(특징 {len(D['matched'])} s={D['sim']['s']:.3f} θ={D['sim']['theta']:+.2f}° rms {D['sim']['rms']:.1f}px {D['scale_mm_px']:.3f}mm/px)")


def align(color, dry=False, tol_mm=TOL_MM, tol_deg=TOL_DEG, srcs=None, roles=None):
    """보정 루프. 수렴 True / dry False / 실패 예외(호출자가 정지·보고). srcs 로 카메라 지정, roles 로 역할(xy/rz/both/measure)."""
    prev = None
    check.expo_done = False
    for it in range(MAX_ITER):
        C, per, why = check(color, srcs, roles)
        for D in per.values():
            print(f"  호버정렬 {it}: {fmt(D)}", flush=True)
        if C is None:
            raise RuntimeError("호버 정렬 측정 실패: " + why)
        e_mm = math.hypot(*C["dmm"]); e_deg = abs(C["drz"])
        print(f"  호버정렬 {it}: 결합 XY ({C['dmm'][0]:+.2f},{C['dmm'][1]:+.2f}) rz {C['drz']:+.2f}° [{C['n_src']}캠, rz {C['rz_from']}]", flush=True)
        if e_mm <= tol_mm and e_deg <= tol_deg:
            print(f"  ✅ 호버 정렬 수렴 ({e_mm:.2f}mm, {e_deg:.2f}°)"); return True
        # 15:44 실기: 18.6→14.2mm 로 줄고 있는데 rz 0.05→0.17°(손목캠 잡음) 로 '발산' 오판 → 각은 0.3° 이하 변동은 무시
        if prev is not None and (e_mm > prev[0] * 1.2 + 0.2 or (e_deg > 0.3 and e_deg > prev[1] * 1.2 + 0.1)):
            raise RuntimeError(f"호버 정렬 발산({prev[0]:.2f}→{e_mm:.2f}mm, {prev[1]:.2f}→{e_deg:.2f}°) — 부호/매핑 의심, 정지")
        prev = (e_mm, e_deg)
        dx, dy = (max(-MAX_STEP_MM, min(MAX_STEP_MM, v)) for v in C["dmm"])
        drz = max(-MAX_STEP_DEG, min(MAX_STEP_DEG, C["drz"])) if e_deg > tol_deg else 0.0
        if dry:
            print(f"    (dry) 이동 ({dx:+.2f},{dy:+.2f}) rz {drz:+.2f}"); return False
        move_rel(dx, dy, drz); time.sleep(0.6)
    raise RuntimeError(f"호버 정렬 {MAX_ITER}회 내 미수렴")


def probe_fixed(src, color, seeds, probe_mm=10.0):
    """고정카메라(newcam/side) 매핑: 벽 든 채 z440 에서 X·Y ±probe 조그 → 벽 점 이동(px)/mm. rot_sign 은 +2° 조그로 실측."""
    def wall_mid():
        pts = []
        for _ in range(3):
            w = wall_dots(grab(src), color, None, seeds, src)
            if len(w) != len(seeds):
                raise RuntimeError(f"{src} 벽 점 {len(w)}개(기대 {len(seeds)})")
            pts.append(np.array([p[:2] for p in sorted(w, key=lambda q: (q[1], q[0]))], float)); time.sleep(0.2)
        return np.mean(pts, axis=0)
    c0 = st()["tcp"]; P0 = wall_mid(); J = np.zeros((2, 2))
    for k, ax in enumerate(("X", "Y")):
        d = [0.0, 0.0]; d[k] = probe_mm
        move_rel(d[0], d[1], 0); time.sleep(0.5); Pp = wall_mid().mean(0)
        move_rel(-2 * d[0], -2 * d[1], 0); time.sleep(0.5); Pm = wall_mid().mean(0)
        move_rel(d[0], d[1], 0); time.sleep(0.4)
        J[:, k] = (Pp - Pm) / (2 * probe_mm)
        print(f"  {ax} ±{probe_mm}: 벽 점 이동 {(Pp - Pm)}px → J 열 {J[:, k]}")
    rot_sign = -1.0
    if len(seeds) >= 2:
        _, a0 = wall_mid_ang(P0); move_rel(0, 0, 2.0); time.sleep(0.5); _, a1 = wall_mid_ang(wall_mid()); move_rel(0, 0, -2.0)
        rot_sign = 1.0 if HG.wrap_deg(a1 - a0) > 0 else -1.0
        print(f"  rz +2° → 화면 각 {HG.wrap_deg(a1 - a0):+.2f}° → rot_sign {rot_sign:+.0f}")
    Jinv = np.linalg.inv(J)
    json.dump({"made": time.strftime("%Y-%m-%d %H:%M"), "tcp": c0, "z": c0[2], "J_px_per_mm": J.tolist(),
               "Jinv_mm_per_px": Jinv.tolist(), "rot_sign": rot_sign, "color": color, "seeds": seeds,
               "scale_mm_per_px": float(1.0 / math.sqrt(abs(np.linalg.det(J)))), "src": src}, open(SRC[src]["map"], "w"), indent=1)
    print(f"✅ {src} 매핑 저장 {SRC[src]['map']}: 축척 {1.0 / math.sqrt(abs(np.linalg.det(J))):.4f}mm/px, 직교 {math.degrees(math.atan2(J[1,0],J[0,0]))-math.degrees(math.atan2(J[1,1],J[0,1]))+90:+.1f}°")


def probe_arm(src, color, probe_mm=10.0):
    """손목 부착(moving="base") 카메라 매핑: X·Y ±probe 조그 → **베이스 특징(기둥 점)** 이동 px/mm, rz +2° → rot_sign.
    (probe_fixed 는 벽 점을 추적하므로 손목 부착 카메라에선 0px → 9/6 13:01 새카메라 184mm/px 오류의 원인)"""
    search = DET[src]["search"]
    def feats():
        img = grab(src); w = wall_dots(img, color, None, None, src)
        f = base_feats(img, w, src)
        if len(f) < 2:
            raise RuntimeError(f"{src} 베이스 특징 {len(f)}개(<2) — 후보 {[(c, round(x), round(y), int(a)) for c, x, y, a in f]}")
        return f
    F0 = feats(); c0 = st()["tcp"]; J = np.zeros((2, 2))
    def pairs(Fn):
        """F0↔Fn 매칭 후, 카메라에 붙어 같이 움직이는 고정물(케이블 등 — 이동이 거의 0)은 제외: 이동량 < 최대의 30%."""
        s_, d_, lab = match_feats(F0, Fn, search)
        if len(s_) < 1:
            raise RuntimeError(f"{src} 특징 매칭 0개")
        v = np.array(d_, float) - np.array(s_, float); mag = np.hypot(v[:, 0], v[:, 1]); keep = mag >= 0.3 * mag.max()
        if keep.sum() < len(s_):
            print(f"    고정물 제외 {[l for l, k in zip(lab, keep) if not k]}")
        return [p for p, k in zip(s_, keep) if k], [p for p, k in zip(d_, keep) if k]
    def shift(Fn):
        a, b = pairs(Fn)
        return np.mean(np.array(b, float) - np.array(a, float), axis=0), len(a)
    for k, ax in enumerate(("X", "Y")):
        d = [0.0, 0.0]; d[k] = probe_mm
        move_rel(d[0], d[1], 0); time.sleep(0.5); Pp, n1 = shift(feats())
        move_rel(-2 * d[0], -2 * d[1], 0); time.sleep(0.5); Pm, n2 = shift(feats())
        move_rel(d[0], d[1], 0); time.sleep(0.4)
        J[:, k] = (Pp - Pm) / (2 * probe_mm)
        print(f"  {ax} ±{probe_mm}: 베이스 특징 이동 {(Pp - Pm)}px (매칭 {n1}/{n2}) → J 열 {J[:, k]}")
    move_rel(0, 0, 2.0); time.sleep(0.5)
    a, b = pairs(feats())
    if len(a) < 2:
        raise RuntimeError(f"{src} rz 프로브: 유효 특징 {len(a)}개(<2)")
    th = similarity(a, b)[1]
    move_rel(0, 0, -2.0); time.sleep(0.3)
    rot_sign = 1.0 if th > 0 else -1.0          # delta(): drz = rot_sign·(−θ) 가 +2° 를 되돌려야 → rot_sign = sign(θ)
    print(f"  rz +2° → 특징 회전 θ {th:+.2f}° → rot_sign {rot_sign:+.0f} (손목캠 ROT_SIGN {ROT_SIGN['wrist']:+.0f} 와 같아야 정상)")
    if abs(abs(th) - 2.0) > 0.8:
        print(f"  ⚠ 특징 회전 {th:+.2f}° 가 2° 와 다름 — 매칭/검출 점검")
    Jinv = np.linalg.inv(J)
    json.dump({"made": time.strftime("%Y-%m-%d %H:%M"), "tcp": c0, "z": c0[2], "J_px_per_mm": J.tolist(),
               "Jinv_mm_per_px": Jinv.tolist(), "rot_sign": rot_sign, "color": color, "moving": "base", "theta_probe": th,
               "scale_mm_per_px": float(1.0 / math.sqrt(abs(np.linalg.det(J)))), "src": src}, open(SRC[src]["map"], "w"), indent=1)
    print(f"✅ {src} 매핑 저장(손목 부착) {SRC[src]['map']}: 축척 {1.0 / math.sqrt(abs(np.linalg.det(J))):.4f}mm/px, 직교 {math.degrees(math.atan2(J[1,0],J[0,0]))-math.degrees(math.atan2(J[1,1],J[0,1]))+90:+.1f}°")


def _seeds(argv):
    if "--wall" not in argv:
        return None
    v = [float(t) for t in argv[argv.index("--wall") + 1].split(",")]
    return [(v[i], v[i + 1]) for i in range(0, len(v), 2)]


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    color = sys.argv[2] if len(sys.argv) > 2 else "blue"
    src = sys.argv[sys.argv.index("--src") + 1] if "--src" in sys.argv else "wrist"
    if mode == "ref":
        save_ref(color, src, _seeds(sys.argv))
    elif mode == "check":
        check.expo_done = False
        C, per, why = check(color)
        for D in per.values(): print(fmt(D))
        print(f"결합 XY ({C['dmm'][0]:+.2f},{C['dmm'][1]:+.2f}) rz {C['drz']:+.2f}° [{C['n_src']}캠]" if C else "❌ " + why)
    elif mode == "align":
        align(color, dry="--dry" in sys.argv)
    elif mode == "detect":
        img = grab(src); w = wall_dots(img, color, None, _seeds(sys.argv), src); f = base_feats(img, w, src)
        print(f"[{src}] 벽 점 {[(round(x), round(y), a) for x, y, a in w]}  베이스 특징 {[(c, round(x), round(y), a) for c, x, y, a in f]}")
    elif mode == "probe":       # hover_align.py probe <newcam|side> <색> --wall x,y[,x,y]
        seeds = _seeds(sys.argv); psrc = color; pcolor = sys.argv[3] if len(sys.argv) > 3 and not sys.argv[3].startswith("--") else "blue"
        if psrc not in SRC or psrc == "wrist": print("probe 대상은 newcam 또는 side"); return
        if SRC[psrc]["moving"] == "base":
            probe_arm(psrc, pcolor); return
        if not seeds: print("--wall x,y[,x,y] 로 그 카메라 화면의 든 벽 점 좌표를 주세요"); return
        probe_fixed(psrc, pcolor, seeds)
    elif mode == "test":
        ref_img = cv2.imread(sys.argv[3]); now_img = cv2.imread(sys.argv[4])
        z = float(sys.argv[5]) if len(sys.argv) > 5 and not sys.argv[5].startswith("--") else 440.0
        rz = float(sys.argv[sys.argv.index("--rz") + 1]) if "--rz" in sys.argv else RZ_MAP
        m0, why = measure(ref_img, color, None, _seeds(sys.argv), src)
        if not m0: print("❌ 기준 프레임:", why); return
        print(f"기준: 벽 {[(round(x), round(y), a) for x, y, a in m0['wall']]}  특징 {[(c, round(x), round(y), a) for c, x, y, a in m0['pillars']]}")
        ref = {"wall": m0["wall"], "pillars": m0["pillars"], "z": z}
        m1, why = measure(now_img, color, ref, None, src)
        if not m1: print("❌ 지금 프레임:", why); return
        print(f"지금: 벽 {[(round(x), round(y), a) for x, y, a in m1['wall']]}  특징 {[(c, round(x), round(y), a) for c, x, y, a in m1['pillars']]}")
        D, why = delta(ref, m1, z, rz, src)
        print(fmt(D) if D else "❌ " + why)
        if D:
            for l in D["matched"]: print("   ", l)
        if "--save" in sys.argv and D:
            out = now_img.copy()
            for c, x, y, a in m1["pillars"]: cv2.circle(out, (int(x), int(y)), 14, (0, 255, 0), 2)
            for x, y, a in m1["wall"]: cv2.circle(out, (int(x), int(y)), 18, (255, 255, 255), 2)
            s_, d_, _ = match_feats(ref["pillars"], m1["pillars"])
            for p in apply_sim(similarity(s_, d_), ref["wall"]): cv2.drawMarker(out, (int(p[0]), int(p[1])), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
            cv2.imwrite(sys.argv[sys.argv.index("--save") + 1], out)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
