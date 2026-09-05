#!/usr/bin/env python3
"""호버 정렬 — 든 벽 점과 베이스 특징(기둥 점·안착된 벽 점)을 **같은 손목캠 프레임**에서 맞춘다 (9/5 밤 신설, 사용자 설계).

왜: 랙에서 그리퍼가 벽 어디를 물었든(길이방향 9mm 치우침 실측) 이 정렬 뒤엔 무관해진다.
    목표 TCP 계산(place_calc.plan)은 z650 관측 1회 계산값이라 파지 편차·베이스 미세 이동을 못 흡수했고,
    그 빈자리를 사용자 육안 nudge 가 메워 왔다. 이 모듈이 그 nudge 를 대신한다.

원리:
  · 기준(ref): 사용자가 "맞다" 한 정렬 상태(z_seat+85, 벽 든 채)에서 [베이스 특징 P_ref, 든 벽 점 W_ref(1~2개)] 저장.
  · 매 사이클: 같은 높이에서 P_now, W_now 검출 → P_ref→P_now 2D 유사변환 S(베이스 기준계) →
    W_exp = S(W_ref) 가 "벽이 있어야 할 자리". Δ = W_now − W_exp (px) → 로봇 XY = Jinv_h·Δ, rz = ROT_SIGN·Δang.
  · 축척: 관측자세 매핑(z650, 기둥꼭대기 뎁스 377mm)을 높이 비율로 환산. Jinv_h = Jinv_obs·(d_h/d_obs).
  · 로봇을 δ 옮기면 베이스 px 는 J_h·δ 움직이고 벽 px 는 그대로(들고 있으니) → Δ 가 J_h·δ 만큼 줄어든다.
    그래서 δ = Jinv_h·Δ (부호는 관측 매핑에서 그대로 옴, 추측 아님).
  · 회전 부호: 매핑 det(J)<0(화면은 로봇 XY 의 거울) → 로봇 +rz 는 화면에서 −각. ROT_SIGN=−1.
    ★1° 조그로 1회 검증할 것 — 검증 전엔 스텝 후 오차가 커지면 즉시 중단(발산 가드).
  · 벽 점 1개(노랑·red_s): 위치만 관측. 회전은 "기준과 같은 각으로 물렸다" 가정으로 −θ(베이스가 돈 만큼).
    파지 회전은 랙 관측 각(rack_ends.ang)·뎁스 길이 게이트로 따로 막는다.

게이트(하나라도 걸리면 하강 금지·정지·보고 — 자동 재시도 없음):
  · 베이스 특징 매칭 ≥ 2 (색 같고 기준 자리에서 SEARCH_PX 안)  · 든 벽 점 ≥ 1 (기준 자리 ±WALL_NEAR_PX)
  · 유사변환 축척 |s−1| ≤ 3% (높이가 다르면 기준 무효)  · 맞춤 rms ≤ 6px  · MAX_ITER 안에 수렴  · 발산 시 즉시 정지

  python3 hover_align.py ref   <색>                 # 지금 프레임(사용자 정렬 상태)을 기준으로 저장
  python3 hover_align.py check <색>                 # Δ 만 계산(이동 없음)
  python3 hover_align.py align <색> [--dry]         # 보정 루프(1% 속도, 스텝 ≤3mm/0.5°)
  python3 hover_align.py test  <색> ref.jpg now.jpg [z] [--save out.jpg]   # 저장 프레임 오프라인 검증
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
REF = "/home/ar/bf2_console/hover_ref_0905.json"
D_OBS = 377.0            # 관측자세(z650)에서 기둥 꼭대기 뎁스(mm) — 9/5 실측
Z_OBS = 650.0
SEARCH_PX = 140.0        # 기준 특징 자리에서 이 안의 같은 색 점을 그 특징으로 본다(≈25mm @z440)
WALL_NEAR_PX = 80.0      # 기준 벽 점 자리에서 이 안의 같은 색 점 = 든 벽 점(파지 편차 수 mm = 10~30px)
HELD_BOX = (820, 0, 1280, 720)   # 기준 없을 때(ref 생성) 든 벽이 보이는 영역 — 그리퍼 고정
SCALE_TOL = 0.03
RMS_TOL_PX = 6.0
ROT_SIGN = -1.0          # ★미검증(1° 조그로 확인). 발산 가드가 보호.
MAX_STEP_MM, MAX_STEP_DEG = 3.0, 0.5
TOL_MM, TOL_DEG = 0.3, 0.15
MAX_ITER = 5
PILLAR_AREA = (60, 1400)  # z440 에서 베이스 특징 점 면적(관측자세 40~400 의 약 3배)

# 든 벽 점 색 규칙 — place_calc.WALL_DOT_HSV 와 동일
WALL_DOT_HSV = {
    "blue":   ((95, 150, 140), (115, 255, 255)),
    "yellow": ((15, 80, 110), (38, 255, 255)),
    "red":    ((135, 90, 55), (175, 255, 255)),
    "red_s":  ((135, 90, 55), (175, 255, 255)),
}


# ------------------------------------------------------------------ 검출
def grab():
    b = UR.urlopen("http://127.0.0.1:8766/raw", timeout=5).read()
    return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)


def _blobs(img, color, amin):
    lo, hi = WALL_DOT_HSV[color]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    return [(float(cen[i][0]), float(cen[i][1]), int(st[i, 4])) for i in range(1, n) if st[i, 4] >= amin]


def wall_dots(img, color, ref=None):
    """든 벽의 색점(1~2개). ref 가 있으면 **기준 자리 ±WALL_NEAR_PX 의 같은 색 점**(그리퍼에 물린 벽은 화면에서
    거의 안 움직인다) — 기둥·다른 벽 점과 면적이 겹쳐도 자리로 갈린다. ref 없으면 HELD_BOX 안 큰 점(≤2)."""
    pts = _blobs(img, color, 150)
    if ref is not None:
        out = []
        for rx, ry, _a in ref["wall"]:
            c = [q for q in pts if math.hypot(q[0] - rx, q[1] - ry) <= WALL_NEAR_PX]
            if c:
                out.append(min(c, key=lambda q: math.hypot(q[0] - rx, q[1] - ry)))
        return out
    x0, y0, x1, y1 = HELD_BOX
    pts = [q for q in pts if x0 <= q[0] <= x1 and y0 <= q[1] <= y1 and q[2] >= 300]
    pts.sort(key=lambda q: -q[2])
    return pts[:2]


def base_feats(img, exclude):
    """베이스 고정 특징 = 화면의 색점 중 든 벽 점이 아닌 것 전부(기둥 꼭대기 점 + 이미 안착된 벽의 점).
    안착 벽 윗변 = 기둥 꼭대기 높이라 같은 평면 → 유사변환의 점으로 쓸 수 있다. 반환 [(color, x, y, area)]."""
    d = PD.detect(img, None)
    out = []
    for c, lst in d.items():
        for p in lst:
            if not (PILLAR_AREA[0] <= p[2] <= PILLAR_AREA[1]):
                continue
            if any(math.dist(p[:2], e[:2]) < 25 for e in exclude):
                continue
            out.append((c, p[0], p[1], p[2]))
    return out


def measure(img, color, ref=None):
    w = wall_dots(img, color, ref)
    if len(w) < 1:
        return None, "든 벽 점 0개(벽을 안 들었거나 기준 자리에 없음)"
    w = sorted(w, key=lambda q: (q[1], q[0]))
    p = base_feats(img, w)
    return {"wall": [(q[0], q[1], q[2]) for q in w], "pillars": [(c, x, y, a) for c, x, y, a in p]}, None


# ------------------------------------------------------------------ 기하
def similarity(src, dst):
    """2D 유사변환(회전·축척·이동) src→dst 최소자승. 반환 (s, theta_deg, tx, ty, rms)."""
    A = np.array(src, float); B = np.array(dst, float)
    ca, cb = A.mean(0), B.mean(0)
    A0, B0 = A - ca, B - cb
    za = A0[:, 0] + 1j * A0[:, 1]; zb = B0[:, 0] + 1j * B0[:, 1]
    den = float((np.abs(za) ** 2).sum())
    if den < 1e-9 or len(src) < 2:
        s, th = 1.0, 0.0
    else:
        z = complex((np.conj(za) * zb).sum() / den)
        s, th = abs(z), math.degrees(math.atan2(z.imag, z.real))
    R = _rot(th)
    t = cb - s * (R @ ca)
    fit = (s * (R @ A.T)).T + t
    rms = float(np.sqrt(((fit - B) ** 2).sum(1).mean()))
    return s, th, float(t[0]), float(t[1]), rms


def _rot(th):
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    return np.array([[c, -s], [s, c]])


def apply_sim(sim, pts):
    s, th, tx, ty, _ = sim
    P = np.array([p[:2] for p in pts], float)
    return (s * (_rot(th) @ P.T)).T + np.array([tx, ty])


def wall_mid_ang(w):
    (x1, y1), (x2, y2) = w[0][:2], w[1][:2]
    return ((x1 + x2) / 2, (y1 + y2) / 2), math.degrees(math.atan2(y2 - y1, x2 - x1))


def match_feats(ref_p, now_p):
    """기준 특징(color,x,y) ↔ 지금 후보: 같은 색, SEARCH_PX 안 최근접(중복 배정 없음). 반환 (src, dst, labels)."""
    src, dst, lab = [], [], []
    used = set()
    for c, x, y, *_ in ref_p:
        best = None
        for j, (c2, x2, y2, a2) in enumerate(now_p):
            if c2 != c or j in used:
                continue
            d = math.hypot(x2 - x, y2 - y)
            if d <= SEARCH_PX and (best is None or d < best[0]):
                best = (d, j)
        if best is not None:
            used.add(best[1]); j = best[1]
            src.append((x, y)); dst.append((now_p[j][1], now_p[j][2]))
            lab.append(f"{c}({x:.0f},{y:.0f})→({now_p[j][1]:.0f},{now_p[j][2]:.0f}) {best[0]:.0f}px")
    return src, dst, lab


RZ_MAP = 180.0           # 관측 매핑을 잰 자세의 rz. 카메라는 손목에 붙어 있어 rz 가 다르면 매핑을 그만큼 돌려야 한다.


def jinv_at(z_tcp, rz_tcp=RZ_MAP):
    """높이·rz 환산 매핑. δ_robot = R(rz−RZ_MAP)·Jinv_h·Δpx.
    ★9/5 오프라인 실증(노랑 z441 rz≈90, 사용자 nudge 프레임): rz 회전 없이는 (+0.6,+2.1) 로 90° 틀린 답,
      회전 적용하면 (+2.1,−0.6) = 실제 사용자 이동 (+2,−1) 과 일치."""
    m = json.load(open(MAP))
    d_h = D_OBS - (Z_OBS - z_tcp)
    if d_h < 60:
        raise RuntimeError(f"높이 z{z_tcp:.0f} 에서 기둥 뎁스 {d_h:.0f}mm — 매핑 환산 불가")
    Jinv = np.array(m["Jinv_mm_per_px"], float) * (d_h / D_OBS)
    return _rot(HG.wrap_deg(rz_tcp - RZ_MAP)) @ Jinv, d_h


def delta(ref, meas, z_tcp, rz_tcp=RZ_MAP):
    """기준 대비 지금의 벽↔베이스 관계 차이. 반환 (dict, None) 또는 (None, why)."""
    src, dst, lab = match_feats(ref["pillars"], meas["pillars"])
    if len(src) < 2:
        return None, (f"베이스 특징 매칭 {len(src)}개(2 필요) — 후보 "
                      f"{[(c, round(x), round(y)) for c, x, y, a in meas['pillars']]}")
    sim = similarity(src, dst)
    s, th, tx, ty, rms = sim
    if abs(s - 1.0) > SCALE_TOL:
        return None, f"특징 간격 축척 {s:.3f} — 기준과 높이가 다름(|s−1|>{SCALE_TOL})"
    if rms > RMS_TOL_PX:
        return None, f"특징 맞춤 rms {rms:.1f}px > {RMS_TOL_PX} — 오매칭 의심(안착 벽이 움직였거나 점 뒤바뀜)"
    w_exp = apply_sim(sim, ref["wall"])
    w_now = np.array([p[:2] for p in meas["wall"]], float)
    one_dot = len(meas["wall"]) < 2 or len(ref["wall"]) < 2
    if one_dot:
        w_exp, w_now = w_exp[:1], w_now[:1]
        mid_exp, mid_now = tuple(w_exp[0]), tuple(w_now[0])
        dang = -th                     # 베이스가 θ 돌았으면 벽도 θ 돌아야 함(파지 회전 = 기준과 같다고 가정)
    else:
        if np.linalg.norm(w_now - w_exp) > np.linalg.norm(w_now[::-1] - w_exp):
            w_now = w_now[::-1]
        mid_exp, ang_exp = wall_mid_ang(w_exp); mid_now, ang_now = wall_mid_ang(w_now)
        dang = HG.wrap_deg(ang_now - ang_exp)
        if dang > 90: dang -= 180
        if dang < -90: dang += 180
    dpx = (mid_now[0] - mid_exp[0], mid_now[1] - mid_exp[1])
    Jinv, d_h = jinv_at(z_tcp, rz_tcp)
    dmm = Jinv @ np.array(dpx)
    return {"dpx": dpx, "dang_img": dang, "dmm": (float(dmm[0]), float(dmm[1])), "drz": ROT_SIGN * dang,
            "sim": {"s": s, "theta": th, "rms": rms}, "matched": lab, "d_h": d_h, "one_dot": one_dot,
            "scale_mm_px": float(np.hypot(*Jinv[:, 0])), "rz": rz_tcp}, None


# ------------------------------------------------------------------ 로봇
def st():
    return json.loads(UR.urlopen(BR + "/status", timeout=6).read())["robots"]["fr5"]


def post(a, b):
    r = UR.Request(BR + "/" + a, json.dumps(b).encode(), {"Content-Type": "application/json"})
    return UR.urlopen(r, timeout=20).read().decode()


def move_rel(dx, dy, drz, tol=0.3, timeout=40):
    """1% 속도 상대 이동. 도달은 busy 가 아니라 TCP 폴링으로(9/5: busy 조기 False 오판 사고)."""
    c = st()["tcp"]
    tgt = [c[0] + dx, c[1] + dy, c[2], c[3], c[4], HG.wrap_deg(c[5] + drz)]
    post("fr5/move", {"tcp": tgt, "speed": 1})
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.3)
        n = st()["tcp"]
        if max(abs(n[i] - tgt[i]) for i in range(3)) <= tol and abs(HG.wrap_deg(n[5] - tgt[5])) <= 0.1:
            return n
    raise RuntimeError(f"이동 미도달 (목표 {[round(v, 2) for v in tgt]} 현재 {[round(v, 2) for v in st()['tcp']]})")


# ------------------------------------------------------------------ 모드
def load_ref(color):
    if not os.path.exists(REF):
        return None
    return json.load(open(REF)).get(color)


def save_ref(color, img=None):
    img = img if img is not None else grab()
    meas, why = measure(img, color)
    if not meas:
        raise RuntimeError(why)
    if len(meas["pillars"]) < 2:
        raise RuntimeError(f"베이스 특징 {len(meas['pillars'])}개 — 이 높이/자세에선 기준을 못 만든다(둘 다 보이는 자세 필요)")
    if len(meas["wall"]) < 2:
        print(f"  ⚠ 든 벽 점 {len(meas['wall'])}개 — 위치만 정렬, 파지 회전은 관측 못 함(랙 관측 각·뎁스 길이로 보완)")
    tcp = st()["tcp"]
    d = json.load(open(REF)) if os.path.exists(REF) else {}
    d[color] = {"wall": meas["wall"], "pillars": meas["pillars"], "tcp": tcp, "z": tcp[2],
                "made": time.strftime("%Y-%m-%d %H:%M"), "note": "사용자 정렬 확인 상태(하강 직전)"}
    json.dump(d, open(REF, "w"), ensure_ascii=False, indent=1)
    print(f"✅ [{color}] 호버 기준 저장: 벽 점 {[(round(x), round(y)) for x, y, a in meas['wall']]} "
          f"베이스 특징 {[(c, round(x), round(y)) for c, x, y, a in meas['pillars']]} z{tcp[2]:.0f}")


def check(color, img=None, z=None, ref=None, rz=None):
    ref = ref or load_ref(color)
    if not ref:
        return None, f"{color} 호버 기준 없음 (hover_align.py ref {color})"
    img = img if img is not None else grab()
    if z is None or rz is None:
        t = st()["tcp"]; z = t[2] if z is None else z; rz = t[5] if rz is None else rz
    if abs(z - ref["z"]) > 3.0:
        return None, f"높이 z{z:.0f} ≠ 기준 z{ref['z']:.0f} — 같은 높이에서만 유효"
    meas, why = measure(img, color, ref)
    if not meas:
        return None, why
    return delta(ref, meas, z, rz)


def fmt(D):
    return (f"{'[점1개] ' if D.get('one_dot') else ''}Δ벽 px ({D['dpx'][0]:+.1f},{D['dpx'][1]:+.1f}) 각 {D['dang_img']:+.2f}°  →  "
            f"로봇 XY ({D['dmm'][0]:+.2f},{D['dmm'][1]:+.2f})mm rz {D['drz']:+.2f}°  "
            f"[특징 {len(D['matched'])}개 s={D['sim']['s']:.3f} θ={D['sim']['theta']:+.2f}° rms {D['sim']['rms']:.1f}px, {D['scale_mm_px']:.3f}mm/px, rz {D['rz']:.1f}]")


def align(color, dry=False, tol_mm=TOL_MM, tol_deg=TOL_DEG):
    """보정 루프. 수렴 True / dry 면 False / 실패 예외(호출자가 정지·보고)."""
    prev = None
    for it in range(MAX_ITER):
        D, why = check(color)
        if D is None:
            raise RuntimeError("호버 정렬 측정 실패: " + why)
        e_mm = math.hypot(*D["dmm"]); e_deg = abs(D["drz"])
        print(f"  호버정렬 {it}: {fmt(D)}", flush=True)
        if e_mm <= tol_mm and e_deg <= tol_deg:
            print(f"  ✅ 호버 정렬 수렴 ({e_mm:.2f}mm, {e_deg:.2f}°)"); return True
        if prev is not None and (e_mm > prev[0] * 1.2 + 0.2 or e_deg > prev[1] * 1.2 + 0.1):
            raise RuntimeError(f"호버 정렬 발산(오차 {prev[0]:.2f}→{e_mm:.2f}mm, {prev[1]:.2f}→{e_deg:.2f}°) — 부호/매핑 의심, 정지")
        prev = (e_mm, e_deg)
        dx, dy = (max(-MAX_STEP_MM, min(MAX_STEP_MM, v)) for v in D["dmm"])
        drz = max(-MAX_STEP_DEG, min(MAX_STEP_DEG, D["drz"])) if e_deg > tol_deg else 0.0
        if dry:
            print(f"    (dry) 이동 ({dx:+.2f},{dy:+.2f}) rz {drz:+.2f}"); return False
        move_rel(dx, dy, drz)
        time.sleep(0.6)
    raise RuntimeError(f"호버 정렬 {MAX_ITER}회 내 미수렴")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    color = sys.argv[2] if len(sys.argv) > 2 else "blue"
    if mode == "ref":
        save_ref(color)
    elif mode == "check":
        D, why = check(color)
        print(fmt(D) if D else "❌ " + why)
    elif mode == "align":
        align(color, dry="--dry" in sys.argv)
    elif mode == "test":
        ref_img = cv2.imread(sys.argv[3]); now_img = cv2.imread(sys.argv[4])
        z = float(sys.argv[5]) if len(sys.argv) > 5 and not sys.argv[5].startswith("--") else 440.0
        rz = float(sys.argv[sys.argv.index("--rz") + 1]) if "--rz" in sys.argv else RZ_MAP
        m0, why = measure(ref_img, color)
        if not m0:
            print("❌ 기준 프레임:", why); return
        print(f"기준: 벽 {[(round(x), round(y), a) for x, y, a in m0['wall']]}  특징 {[(c, round(x), round(y), a) for c, x, y, a in m0['pillars']]}")
        ref = {"wall": m0["wall"], "pillars": m0["pillars"], "z": z}
        m1, why = measure(now_img, color, ref)
        if not m1:
            print("❌ 지금 프레임:", why); return
        print(f"지금: 벽 {[(round(x), round(y), a) for x, y, a in m1['wall']]}  특징 {[(c, round(x), round(y), a) for c, x, y, a in m1['pillars']]}")
        D, why = delta(ref, m1, z, rz)
        print(fmt(D) if D else "❌ " + why)
        if D:
            for l in D["matched"]: print("   ", l)
        if "--save" in sys.argv:
            out = now_img.copy()
            for c, x, y, a in m1["pillars"]: cv2.circle(out, (int(x), int(y)), 14, (0, 255, 0), 2)
            for x, y, a in m1["wall"]: cv2.circle(out, (int(x), int(y)), 18, (255, 255, 255), 2)
            if D:
                src, dst, _ = match_feats(ref["pillars"], m1["pillars"])
                for p in apply_sim(similarity(src, dst), ref["wall"]):
                    cv2.drawMarker(out, (int(p[0]), int(p[1])), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
            cv2.imwrite(sys.argv[sys.argv.index("--save") + 1], out)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
