#!/usr/bin/env python3
"""높이별 점-절대값 기준 하강 (9/1, 사용자 설계: "보이는 장면 높이별로 사진 찍어서
점 위치 절대값으로 똑같이 맞춰서 내리는 걸로 — 새카메라·뎁스 두 개 다").

  python3 descend_ref.py record_up blue           # 티칭 자세에서 올라가며 4개 측정점 기록(권장)
  python3 descend_ref.py record_up blue --place    # place(삽입) 쪽 — 기준 자세=insert_tcp, 저장키=descent_refs_place
  python3 descend_ref.py play blue [--place]       # 4점 보정 하강 → 최종 수직
  python3 descend_ref.py record blue          # 티칭 XY 그대로 계단 하강하며 두 캠의 점 픽셀 기록
  python3 descend_ref.py play   blue          # 기록된 높이마다 점을 기준에 맞춰 보정하며 하강
  python3 descend_ref.py play   blue --to 380 # 이 z 까지만

record 는 방금 손으로 티칭한 경로가 정답이라는 전제로, 그 XY 수직선을 내려가며
각 높이에서 손목캠(:8766)·새카메라(:8768)의 해당 색 점들을 저장한다(refs.<색>.descent_refs).
play 는 각 높이에서:
  · 손목캠: 기준 점들과 현재 점들을 최근접 매칭 → 평균 Δpx → 뎁스 보정 축척으로 Δmm → XY 보정(최대 3회, 2px 수렴)
  · 새카메라: 기준 대비 어긋남(px)만 검사 — CAM2_TOL 넘으면 그 자리 정지(검증자 역할, 뎁스 없어 축척 불확실)
그리퍼는 절대 건드리지 않는다(파지는 사용자 "파지해" 후 별도).
"""
import json
import math
import sys
import time
import urllib.request

import numpy as np

BRIDGE = "http://localhost:8765"
CAL = "/home/ar/bf2_console/dot_calib.json"
CAM1, CAM2 = "http://localhost:8766", "http://localhost:8768"
W_C, H_C = 1280, 720          # 손목캠 프레임(회전 중심 = 화면 중앙 ≈ 광축)
W_C2, H_C2 = 1280, 720        # 새카메라 프레임(rot 0)
HEIGHTS = [520, 490, 460, 430, 410, 395, 383, 372, 362, 353, 345]
UP_OFFS = [35, 60, 90, 125]     # 9/1 사용자: "올라가며 4번 측정점, 파지 전 4번 보정" — 티칭 z + 이 값들
#   ★+15(z360)는 뺐다: 그 높이면 손목캠이 벽에 너무 붙어 파란 점이 면적 868(정상 130) 블롭으로
#     뭉개지고 뎁스도 무효(<175mm) → 무게중심이 20px 튄다(9/1 실증). 최저 측정점은 +35 부터.
S1_0, D1_0 = 3.105, 288.0      # 손목캠 캘리브 축척(px/mm @288mm) — 9/1 실측
CAM2_TOL = 18.0                # px — 새카메라 기준 이탈 허용(≈5~6mm)


def post(a, b):
    r = urllib.request.Request(f"{BRIDGE}/fr5/{a}", json.dumps(b).encode(),
                               {"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=30).read())


def st():
    return json.loads(urllib.request.urlopen(BRIDGE + "/status", timeout=5).read())["robots"]["fr5"]


def wait(t=90):
    time.sleep(0.8)
    t0 = time.time()
    while time.time() - t0 < t:
        if not st()["busy"]:
            return
        time.sleep(0.3)
    raise RuntimeError("busy timeout")


HELD_AREA = 700.0        # 이보다 크면 '물고 있는 벽'(카메라에 가까워 크게 보인다)


def base_ang(cam_url, max_age=1.5):
    """ArUco 마커 2개 중심선 각(도) — J6 회전량을 재는 신뢰 기준(엔코더는 1° 미만에서 못 믿는다)."""
    try:
        r = json.loads(urllib.request.urlopen(cam_url + "/aruco", timeout=6).read())
    except Exception:
        return None
    if r.get("age") is None or r["age"] > max_age or len(r["markers"]) < 2:
        return None
    ms = sorted(r["markers"], key=lambda m: m["id"])[:2]
    c = [np.mean(np.array(m["corners"], float), axis=0) for m in ms]
    return math.degrees(math.atan2(c[1][1]-c[0][1], c[1][0]-c[0][0])) % 180.0



def held_positions(rows, cam, tol=12.0):
    """기록 자체에서 '물고 있는 벽 점'을 판별 — 높이를 바꿔도 화면에서 안 움직이는 점.
    ★면적 정보 없이 기록된 옛 기준에도 쓸 수 있다(9/1: 이게 없어 벽 점이 회전 추정에 섞여
      회전보상이 +0.75~+1.59° 로 튀었다)."""
    if len(rows) < 2:
        return []
    hi, lo = norm(rows[0][cam]), norm(rows[-1][cam])
    out = []
    for p in hi:
        cand = [q for q in lo if p[0] is None or q[0] is None or q[0] == p[0]]
        if not cand:
            continue
        q = min(cand, key=lambda q: (q[1]-p[1])**2 + (q[2]-p[2])**2)
        if (q[1]-p[1])**2 + (q[2]-p[2])**2 < tol*tol:
            out.append((p[0], p[1], p[2]))
    return out


def is_held(p, hps, r=35.0):
    """★반경을 크게(70px) 잡고 색을 안 보면 옆의 밑판 점까지 '벽'으로 오분류된다
    (9/1 실측: 벽 blue(570,323) 에서 65px 떨어진 밑판 red(576,388) 가 벽으로 잡혔다)."""
    return any((p[0] is None or h[0] is None or p[0] == h[0])
               and (p[1]-h[1])**2 + (p[2]-h[2])**2 < r*r for h in hps)


def fit_rot(ref, cur):
    """짝지어진 점들로 최적 회전각(도) 추정 — 2D Kabsch. J6 가 돌면 월드 점이 화면에서 함께 돈다."""
    if len(ref) < 2:
        return 0.0
    R = np.array([[p[1], p[2]] for p in ref], float)
    C = np.array([[p[1], p[2]] for p in cur], float)
    R -= R.mean(axis=0); C -= C.mean(axis=0)
    num = float(np.sum(R[:, 0]*C[:, 1] - R[:, 1]*C[:, 0]))
    den = float(np.sum(R[:, 0]*C[:, 0] + R[:, 1]*C[:, 1]))
    return math.degrees(math.atan2(num, den))


def pair_up(ref, cur, r=140.0):
    """같은 색 최근접 짝짓기 → (ref_matched, cur_matched)"""
    R, C = norm(ref), norm(cur)
    a, b = [], []
    for p in R:
        cc = [q for q in C if p[0] is None or q[0] is None or q[0] == p[0]]
        if not cc:
            continue
        q = min(cc, key=lambda q: (q[1]-p[1])**2 + (q[2]-p[2])**2)
        if (q[1]-p[1])**2 + (q[2]-p[2])**2 < r*r:
            a.append(p); b.append(q)
    return a, b


def norm(rows):
    """행을 (kind|None, x, y, depth, area|None) 로 통일 — ★멱등해야 한다.
    옛 포맷 [x,y,d] 도, 새 포맷 [kind,x,y,d,area] 도, 이미 정규화된 튜플도 같은 결과."""
    out = []
    for r in rows:
        if r and (isinstance(r[0], str) or r[0] is None):      # 이미 kind 자리를 가진 행
            out.append((r[0], r[1], r[2],
                        r[3] if len(r) > 3 else None,
                        r[4] if len(r) > 4 else None))
        else:                                                   # 옛 포맷 [x, y, depth]
            out.append((None, r[0], r[1], r[2] if len(r) > 2 else None, None))
    return out


def rot_about(pts, deg, cx, cy):
    """점들을 (cx,cy) 중심으로 deg 회전 — J6 가 바뀌면 월드 고정물이 화면에서 그만큼 돈다."""
    t = math.radians(deg); co, si = math.cos(t), math.sin(t)
    out = []
    for p in norm(pts):
        x, y = p[1]-cx, p[2]-cy
        out.append((p[0], cx + x*co - y*si, cy + x*si + y*co, p[3], p[4]))
    return out


def read_dots(cam, kind, n=6, allkind=False, kinds=None):
    """색=kind 점들의 (px,py,depth) 목록 — n회 읽어 점별 중앙값(최근접 군집).
    kinds: {"blue","yellow"} 처럼 쓸 색 집합(9/1 사용자 지정 — 뎁스=노랑+파랑, 새캠=빨강+파랑).
    allkind=True: 모든 색. 둘 다 없으면 kind 하나만.
    반환은 항상 [kind, x, y, depth]."""
    if kinds:
        allkind = True
    acc = {}
    got = 0
    url = cam + ("/dots?raw=1" if allkind else "/dots")
    for _ in range(n * 5):
        try:
            ds = json.loads(urllib.request.urlopen(url, timeout=5).read())["dots"]
        except Exception:
            time.sleep(0.2)
            continue
        hit = ([t for t in ds if t["kind"] in kinds] if kinds
               else (ds if allkind else [t for t in ds if t["kind"] == kind]))
        if hit:
            got += 1
            for t in hit:
                cand = [K for K in acc if K[2] == t["kind"]]
                k = min(cand, key=lambda K: (K[0]-t["px"])**2 + (K[1]-t["py"])**2) if cand else None
                if k is not None and (k[0]-t["px"])**2 + (k[1]-t["py"])**2 < 40**2:
                    acc[k].append((t["px"], t["py"], t.get("depth_mm"), t.get("area", 0)))
                else:
                    acc[(t["px"], t["py"], t["kind"])] = [(t["px"], t["py"], t.get("depth_mm"), t.get("area", 0))]
        if got >= n:
            break
        time.sleep(0.15)
    out = []
    for K, v in acc.items():
        if len(v) < max(2, got // 2):
            continue                       # 절반 미만 프레임에만 보인 건 잡티
        xs = sorted(p[0] for p in v); ys = sorted(p[1] for p in v)
        dp = [p[2] for p in v if p[2]]
        # (p[3] = area)
        ar = sorted((p[3] if len(p) > 3 else 0) for p in v)
        out.append([K[2], xs[len(xs)//2], ys[len(ys)//2],
                    (sorted(dp)[len(dp)//2] if dp else None), ar[len(ar)//2]])
    out.sort(key=lambda p: (p[0], p[2], p[1]))
    return out


def match(ref, cur):
    """기준 ↔ 현재 최근접 매칭 → (Δpx 중앙값, 매칭 수). 양쪽 kind 가 있으면 같은 색끼리만."""
    R, C = norm(ref), norm(cur)
    if not R or not C:
        return None, 0
    dx, dy, n = [], [], 0
    for k, rx, ry, _d, _a in R:
        cc = [c for c in C if k is None or c[0] is None or c[0] == k]
        if not cc:
            continue
        c = min(cc, key=lambda p: (p[1]-rx)**2 + (p[2]-ry)**2)
        if (c[1]-rx)**2 + (c[2]-ry)**2 < 120**2:
            dx.append(c[1]-rx); dy.append(c[2]-ry); n += 1
    if not n:
        return None, 0
    return (float(np.median(dx)), float(np.median(dy))), n


def go_z(z):
    t = list(st()["tcp"]); t[2] = float(z)
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()


def set_focus(k1, k2):
    """9/1 사용자: '블루 잡을 땐 블루만 띄워' — 카메라별로 쓰는 색만 검출/표시."""
    for cam, k in ((CAM1, k1), (CAM2, k2)):
        try:
            urllib.request.urlopen(f"{cam}/focus?kinds={k}", timeout=3).read()
        except Exception:
            pass


def main():
    mode, ck = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "blue")
    # 9/1: pick(랙에서 집기) / place(밑판에 꽂기) 공용. 기준 자세와 저장 키만 다르다.
    station = "place" if "--place" in sys.argv else "pick"
    TKEY = {"pick": "pick_tcp_taught", "place": "insert_tcp"}[station]
    RKEY = {"pick": "descent_refs", "place": "descent_refs_place"}[station]
    # 9/1 사용자 지정: place 는 뎁스=노랑+파랑, 새카메라=빨강+파랑 을 절대값으로 저장/대조.
    #   pick 은 종전대로 손목캠=작업색, 새카메라=전 색.
    def _ks(flag, dflt):
        v = sys.argv[sys.argv.index(flag)+1] if flag in sys.argv else None
        return set(v.split(",")) if v else dflt
    if station == "place":
        C1K = _ks("--c1", {"yellow", ck})
        C2K = _ks("--c2", {"red", ck})
    else:
        C1K = _ks("--c1", {ck})
        C2K = _ks("--c2", None)
    set_focus(",".join(sorted(C1K)), ",".join(sorted(C2K)) if C2K else ck)
    to_z = float(sys.argv[sys.argv.index("--to")+1]) if "--to" in sys.argv else HEIGHTS[-1]
    cal = json.load(open(CAL))
    ref = cal["refs"][ck]
    tt = ref[TKEY]
    hs = [h for h in HEIGHTS if h >= to_z - 0.1]

    if mode == "record_up":
        # 방금 손으로 맞춘 파지 자세에서 '올라가며' 기록 — 내려온 경로보다 진짜 정답 선에 가깝다.
        cur = st()["tcp"]
        tz = tt[2]
        if abs(cur[0]-tt[0]) > 3 or abs(cur[1]-tt[1]) > 3 or abs(cur[2]-tz) > 3:
            print(f"⚠ 현재 자세가 티칭({tt[:3]})에서 벗어남 {[round(cur[i]-tt[i],1) for i in range(3)]}mm — 티칭 자세에서 시작할 것")
            return
        post("speed", {"value": 1, "dry_run": False}); time.sleep(0.3)
        rows = []
        for off in UP_OFFS:                    # 낮은 곳부터 위로
            go_z(tz + off)
            time.sleep(0.8)
            c1 = read_dots(CAM1, ck, kinds=C1K)
            c2 = read_dots(CAM2, ck, allkind=True, kinds=C2K)
            _dp = sorted(p[3] for p in c1 if p[3])
            rows.append({"z": round(tz + off, 1), "cam1": c1, "cam2": c2,
                         "d1": (_dp[len(_dp)//2] if _dp else None)})   # 9/1 사용자: 높이별 축척 저장
            print(f"  z{tz+off:.0f}: 손목캠 {len(c1)}점 {[(p[0][:1], round(p[1]), round(p[2])) for p in c1]}"
                  f" | 새캠 {len(c2)}점 {[(p[0][:1], round(p[1]), round(p[2])) for p in c2]}")
        rows.sort(key=lambda r: -r["z"])       # play 는 높은 곳부터
        ref[RKEY] = {"made": time.strftime("%Y-%m-%d %H:%M"), "mode": "up4", "station": station,
                     "cam1_kinds": sorted(C1K), "cam2_kinds": sorted(C2K) if C2K else None,
                     "tcp_line": [tt[0], tt[1]], "final_z": tz, "rows": rows}
        cal["refs"][ck] = ref
        json.dump(cal, open(CAL, "w"), indent=1, ensure_ascii=False)
        print(f"저장 완료: {len(rows)}개 측정점(z{rows[0]['z']}→{rows[-1]['z']}) + 최종 z{tz}. 팔은 최고점 대기")
        return

    if mode == "record":
        print(f"[{ck}] 기록: 티칭 XY ({tt[0]:.1f},{tt[1]:.1f}) 수직선, z{hs[0]}→{hs[-1]}")
        post("speed", {"value": 2, "dry_run": False}); time.sleep(0.3)
        cur = st()["tcp"]
        t = [tt[0], tt[1], max(cur[2], hs[0]), tt[3], tt[4], tt[5]]
        post("move_tcp", {"tcp": t, "dry_run": False}); wait()
        rows = []
        for z in hs:
            go_z(z)
            time.sleep(0.8)
            c1 = read_dots(CAM1, ck, kinds=C1K)
            c2 = read_dots(CAM2, ck, allkind=True, kinds=C2K)
            _dp = sorted(p[3] for p in c1 if p[3])
            rows.append({"z": z, "cam1": c1, "cam2": c2,
                         "d1": (_dp[len(_dp)//2] if _dp else None)})   # 9/1 사용자: 높이별 축척 저장
            print(f"  z{z}: 손목캠 {len(c1)}점 {[(p[0][:1], round(p[1]), round(p[2])) for p in c1]}"
                  f" | 새캠 {len(c2)}점 {[(p[0][:1], round(p[1]), round(p[2])) for p in c2]}")
        ref[RKEY] = {"made": time.strftime("%Y-%m-%d %H:%M"), "station": station,
                     "tcp_line": [tt[0], tt[1]], "rows": rows}
        cal["refs"][ck] = ref
        json.dump(cal, open(CAL, "w"), indent=1, ensure_ascii=False)
        print(f"저장 완료: refs.{ck}.descent_refs ({len(rows)}개 높이) — 팔은 z{hs[-1]} 에 정지(그리퍼 무변경)")
        return

    if mode == "play":
        rows = ref[RKEY]["rows"]
        HP1 = held_positions(rows, "cam1")
        HP2 = held_positions(rows, "cam2")

        def _gain(HP, span_mm, base_scale):
            """게인 = 밑판 축척 / 벽 축척. 벽 축척 = 기준 벽 점 픽셀간격 / 실제 간격(mm)."""
            if len(HP) < 2 or not span_mm:
                return 1.0
            px = math.dist((HP[0][1], HP[0][2]), (HP[1][1], HP[1][2]))
            return base_scale / (px / float(span_mm)) if px > 1 else 1.0

        G1 = _gain(HP1, cal.get("wall_span_mm_cam1"), S1_0)
        # ★새카메라 밑판 축척은 place 높이마다 다르다(z478 2.66 → z413 3.43 px/mm, 실측).
        #   픽 위치에서 잰 cam2_scale(4.83) 을 쓰면 게인이 크게 틀어진다 → 각 높이의 기준에서 직접 구한다.
        def _cam2_base_scale(row):
            sc = [p for p in norm(row["cam2"]) if p[0] == "blue" and not is_held(p, HP2)]
            span = float(cal.get("base_span_mm_cam2") or 191.0)
            if len(sc) >= 2 and span:
                d = max(math.dist((a[1], a[2]), (b[1], b[2]))
                        for i, a in enumerate(sc) for b in sc[i+1:])
                return d / span
            return float(cal.get("cam2_scale_px_per_mm") or 4.83)
        if station == "place":
            g2s = [round(_gain(HP2, cal.get("wall_span_mm_cam2"), _cam2_base_scale(r)), 3) for r in rows]
            print(f"  파지보정 게인: 손목캠 {G1:.3f} · 새캠(높이별) {g2s}")
        print(f"  물고 있는 벽 점: 손목캠 {[(round(h[1]),round(h[2])) for h in HP1]}"
              f" · 새캠 {[(round(h[1]),round(h[2])) for h in HP2]}")
        print(f"[{ck}] 재생: {len(rows)}개 높이, z{rows[0]['z']}→{to_z}")
        post("speed", {"value": 2, "dry_run": False}); time.sleep(0.3)
        # 9/1: 보정 상한을 높이별로 — 위쪽은 크게(부품이 옮겨졌을 때 따라가야 함),
        #   아래로 갈수록 작게(충돌 위험). 상한 3mm 고정이면 18mm 이동한 벽을 못 쫓아간다(실증).
        CAPS = [25.0, 10.0, 5.0, 3.0]
        # 9/1: 요구 정밀도가 10배 다르다 — pick 은 ±2~3mm 면 집히고, place 는 ±0.2mm 라야 꽂힌다.
        TOL = 5.0 if station == "pick" else 2.5      # px (손목캠 축척 ≈3.1px/mm → 1.6mm / 0.8mm)
        # 새카메라 교차검증: place 는 엄격(불일치 시 정지), pick 은 참고용.
        #   pick 기준은 스티커 추가·J6 변경 전 기록이라 짝짓기가 깨진다 → 낡으면 재기록할 것.
        TOL2 = CAM2_TOL if station == "place" else 45.0
        if "--nocam2" in sys.argv:      # 새카메라 기준이 낡았을 때 참고용으로 낮춤(로그는 그대로)
            TOL2 = 999.0
        for ri, row in enumerate(rows):
            z = row["z"]
            if z < to_z - 0.1:
                break
            cap = CAPS[min(ri, len(CAPS) - 1)]
            go_z(z)
            time.sleep(0.7)
            rot_used = 0.0
            for it in range(5):
                c1 = read_dots(CAM1, ck, n=5, kinds=C1K)
                # ★9/1: yaw 보정으로 J6 가 돌면 카메라도 돌아 '월드(밑판) 점'이 화면에서 통째로 회전한다.
                #   평행이동 보정만으로는 원리적으로 못 지운다(실측 잔차 8.2px 에서 정체).
                #   → 기준의 월드 점을 그만큼 회전시켜 놓고 비교한다. 물고 있는 벽 점(면적 큰 것)은
                #     카메라와 한 몸이라 회전 대상이 아니다.
                refs_all = norm(row["cam1"])
                held_ref = [p for p in refs_all if is_held(p, HP1)]
                scen_ref = [p for p in refs_all if not is_held(p, HP1)]
                scen_cur = [p for p in norm(c1) if not is_held(p, HP1)]
                if len(scen_ref) >= 2 and len(scen_cur) >= 2:
                    a, b = pair_up(scen_ref, scen_cur)
                    if len(a) >= 2:
                        rot_used = fit_rot(a, b)
                        scen_ref = rot_about(scen_ref, rot_used, W_C / 2, H_C / 2)
                # ★벽 기준 정렬(8/30 원리, 9/1 구현): 파지가 달라지면 벽이 그리퍼 안에서
                #   평행이동도 한다. 밑판만 기준 픽셀에 맞추면 벽은 그만큼 어긋난 곳에 꽂힌다.
                #   벽 점은 카메라와 한 몸이라 팔을 움직여도 화면에서 안 움직이므로,
                #   맞춰야 하는 건 '벽→밑판 상대 벡터'다.
                #   err = Δ밑판 − Δ벽  (팔 1mm 이동 → 밑판만 s px 움직임 ⇒ 게인 불필요)
                held_cur = [p for p in norm(c1) if is_held(p, HP1)]
                dh = (0.0, 0.0)
                if station == "place" and held_ref and held_cur:
                    ah, bh = pair_up(held_ref, held_cur)
                    if ah:
                        dh = (float(np.median([b[1]-a_[1] for a_, b in zip(ah, bh)])),
                              float(np.median([b[2]-a_[2] for a_, b in zip(ah, bh)])))
                        # ★축척 게인(9/1 실패 원인): 벽 점은 카메라에서 ~90mm, 밑판은 ~288mm.
                        #   같은 1mm 가 벽에서 3배 크게 보이므로 벽의 px 이동을 그대로 밑판 px 로
                        #   옮기면 3배 과보정한다(실측 1.93mm 이동해야 할 것을 6px→1.93mm 로 밀어
                        #   1.33mm 과보정 → 밑동이 기둥에 걸려 벽이 그리퍼에서 빠졌다).
                        dh = (dh[0] * G1, dh[1] * G1)
                ref_use = scen_ref if scen_ref else refs_all
                d1, n1 = match(ref_use, scen_cur or c1)
                if d1 is None:
                    print(f"  z{z}: 손목캠 {ck} 미검출 — 정지"); return
                d1 = (d1[0] - dh[0], d1[1] - dh[1])
                dep = [p[3] for p in c1 if p[3]]
                d = (sorted(dep)[len(dep)//2] if dep
                     else (row.get("d1") or D1_0))     # 9/1: 라이브 뎁스 무효(흰 종이 IR)면 기록값 사용
                s = S1_0 * D1_0 / d
                ex, ey = d1
                err = (ex*ex + ey*ey) ** 0.5
                if err < TOL:
                    break
                dX = max(-cap, min(cap, ex / s))
                dY = max(-cap, min(cap, ey / -s))
                cur = list(st()["tcp"]); cur[0] -= dX; cur[1] -= dY   # 점이 +로 밀렸으면 팔을 −로
                post("move_tcp", {"tcp": cur, "dry_run": False}); wait()
                time.sleep(0.45)      # ★정착 대기 — 없으면 움직이는 중에 읽어 잔차가 안 줄어든다(9/1)
            if err >= TOL:
                # ★9/1: 마지막 반복에서 보정을 적용한 뒤 재측정 없이 끝나 '미수렴'으로 오판했다.
                #   실제로는 정렬돼 있었다(실측 0.14px) → 종료 직전 한 번 더 잰다.
                time.sleep(0.5)
                c1 = read_dots(CAM1, ck, n=5, kinds=C1K)
                d1, _ = match(row["cam1"], c1)
                if d1 is not None:
                    err = (d1[0]**2 + d1[1]**2) ** 0.5
                if err >= TOL:
                    print(f"  z{z}: 손목캠 잔차 {err:.1f}px 로 미수렴(허용 {TOL}px, 상한 {cap:.0f}mm×5회) — 정지")
                    return
            c2 = read_dots(CAM2, ck, n=4, allkind=True, kinds=C2K)
            # 9/1: 검증도 '작업 색'만 — 옆 벽(랙) 점과 섞으면, 손으로 다시 꽂아 벽이 랙 대비
            #   몇 mm 이동했을 때 팔은 벽을 잘 따라가는데도 '이탈'로 오탐한다(z411 실증).
            #   작업 색이 그 높이 기준에 없을 때만 전 색으로 대체.
            # 손목캠과 같은 이유로 새카메라도 회전 보상 — J6 가 돌면 이 카메라의 월드 점도 함께 돈다.
            r2 = norm(row["cam2"]); c2n = norm(c2)
            r2h = [p for p in r2 if is_held(p, HP2)]
            r2s = [p for p in r2 if not is_held(p, HP2)]
            c2s = [p for p in c2n if not is_held(p, HP2)]
            rot2 = 0.0
            if len(r2s) >= 2 and len(c2s) >= 2:
                a2, b2 = pair_up(r2s, c2s)
                if len(a2) >= 2:
                    rot2 = fit_rot(a2, b2)
                    r2s = rot_about(r2s, rot2, W_C2 / 2, H_C2 / 2)
            # 새카메라도 같은 '벽 기준' 보정을 받아야 한다 — 파지가 달라져 목표를 의도적으로
            #   옮겼으므로, 밑판만 원래 기준과 대조하면 그 이동량이 통째로 '이탈'로 잡힌다.
            c2h = [p for p in c2n if is_held(p, HP2)]
            dh2 = (0.0, 0.0)
            if station == "place" and r2h and c2h:
                a3, b3 = pair_up(r2h, c2h)
                if a3:
                    g2 = _gain(HP2, cal.get("wall_span_mm_cam2"), _cam2_base_scale(row))
                    dh2 = (float(np.median([b[1]-a_[1] for a_, b in zip(a3, b3)])) * g2,
                           float(np.median([b[2]-a_[2] for a_, b in zip(a3, b3)])) * g2)
            d2, n2 = match(r2s, c2s or c2)
            if d2 is not None:
                d2 = (d2[0] - dh2[0], d2[1] - dh2[1])
            tag = ""
            if d2 is not None:
                e2 = (d2[0]**2 + d2[1]**2) ** 0.5
                tag = f" | 새캠 이탈 {e2:.0f}px(회전 {rot2:+.2f}°)"
                if e2 > TOL2:
                    print(f"  z{z}: 손목캠 잔차 {err:.1f}px 인데 새카메라 이탈 {e2:.0f}px > {TOL2}"
                          f" — 두 눈 불일치, 정지(그리퍼 무변경)")
                    return
            print(f"  z{z}: 잔차 {err:.1f}px (보정 {it}회, 뎁스 {d:.0f}, 회전보상 {rot_used:+.2f}°, "
                  f"파지이동 {dh[0]:+.1f},{dh[1]:+.1f}px){tag}")
        fz = ref[RKEY].get("final_z")
        if fz is not None and to_z <= rows[-1]["z"] and fz < rows[-1]["z"]:
            post("speed", {"value": 1, "dry_run": False}); time.sleep(0.3)
            go_z(fz)                       # 마지막: 보정 없이 수직 하강 (벽이 손가락 사이라 XY 밀지 않음)
            print(f"  최종 수직 하강 z{fz}")
        s_ = st()
        print(f"완료: tcp {[round(v, 1) for v in s_['tcp']]} — 그리퍼 무변경, '파지해' 대기")
        return

    print("사용법: descend_ref.py record|play <색> [--to z]")


if __name__ == "__main__":
    main()
