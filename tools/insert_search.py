"""8/30 순응 삽입(compliant insert) — 걸리면 X/Y 를 교대로 미세이동하고 J6(rz)를 미세회전하며 재시도.
사용자 실제 수동 절차를 그대로 자동화. 힘센서가 없으므로 카메라(밑판 강체점·벽 점)를 접촉 센서로 쓴다.

  python3 insert_search.py <색> [목표z] [최대시도]
전제: 이미 호버(z571 부근)에서 정렬이 끝나 있을 것(dot_place 또는 wall_align 직후).
"""
import json, sys, time, urllib.request
import numpy as np

CK = sys.argv[1]
ZT = float(sys.argv[2]) if len(sys.argv) > 2 else 491.0
MAXTRY = int(sys.argv[3]) if len(sys.argv) > 3 else 14
B = "http://localhost:8765"
STEP, FIRST = 2.0, 1.5     # 8/30: 걸린 뒤 더 밀면 벽이 패드 밖으로 빠진다(3회) → 스텝 최소화
G_SLIP_FIX, MAX_FIX = 20.0, 3   # 이 이하의 미끄러짐이면 재정렬 후 재개(그 이상이면 중단)
G_RIGID, G_WALL = 5.0, 3.0      # 8/30: 벽 임계 6→3px, 미끄러짐을 훨씬 일찍 잡는다
BACK = 3.0                       # 접촉 시 후퇴량
NUDGE = 0.4                      # 미세이동 1칸
RZN = 0.35                       # J6 미세회전 1칸(°)
LIM_XY, LIM_RZ = 2.0, 1.4        # 누적 한계

def post(a, b):
    r = urllib.request.Request(f"{B}/fr5/{a}", data=json.dumps(b).encode(),
                               headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=25).read())
def stt(): return json.loads(urllib.request.urlopen(f"{B}/status", timeout=3).read())["robots"]["fr5"]
def wait(t=120):
    time.sleep(1.2)
    for _ in range(int(t/0.4)):
        if not stt()["busy"]: break
        time.sleep(0.4)
    last = None; same = 0                      # TCP 스테일(최대 2mm) 방지
    for _ in range(40):
        time.sleep(0.25); z = round(stt()["tcp"][2], 2)
        same = same + 1 if (last is not None and abs(z - last) < 0.03) else 0; last = z
        if same >= 2: return
def dots(n=7):
    acc = {}
    for _ in range(n):
        for p in json.load(urllib.request.urlopen("http://localhost:8766/dots", timeout=4))["dots"]:
            acc.setdefault((p["kind"], round(p["px"]/60), round(p["py"]/60)), []).append((p["px"], p["py"]))
        time.sleep(0.07)
    return [(k[0], float(np.median([q[0] for q in v])), float(np.median([q[1] for q in v])))
            for k, v in acc.items() if len(v) >= n*0.5]
def near(cur, kind, x, y, r=45):
    c = [d for d in cur if d[0] == kind and (d[1]-x)**2 + (d[2]-y)**2 < r*r]
    return min(c, key=lambda d: (d[1]-x)**2 + (d[2]-y)**2) if c else None

cal = json.load(open("/home/ar/bf2_console/dot_calib.json"))
_hr = min(cal["refs_place"][CK]["hover_refs"], key=lambda e: abs(e["z"]-571))
M = np.array(_hr["M"], float)
Minv = np.linalg.inv(M)
_wr = [d for d in _hr["dots"] if d.get("role") == "wall"]
WREF = (_wr[0]["px"], _wr[0]["py"]) if _wr else None
# 벽 축척 게인 (브리지와 동일 계산): 벽은 카메라에 d_w 로 가깝고 밑판은 멀다
try:
    _pr = cal["refs"][CK]
    _dep = float(_pr["target_obs"][2]) - float(_pr.get("ref_lift_mm") or 0.0)
    _dw = _dep + float(_pr["pick_offset"][2])
    _sw = (float(_pr["end_gap_px"]) / float(_pr["point_span_mm"])) * _dep / _dw
    GAIN = max(0.15, min(1.2, max(abs(M[0][0]), abs(M[0][1]), abs(M[1][0]), abs(M[1][1])) / _sw))
except Exception:
    GAIN = 0.45
def wall_now():
    c = [d for d in dots() if d[0] == CK and 600 < d[1] < 1150]
    return (c[0][1], c[0][2]) if c else None
def realign_wall():
    """벽이 그리퍼 안에서 밀렸을 때: 지금 벽 위치를 기준으로 팔을 다시 맞춘다.
    (평행이동 성분만 — 회전은 점 하나로는 못 잰다)  반환: 적용한 mm 또는 None"""
    if WREF is None: return None
    w = wall_now()
    if w is None: return None
    dpx = np.array([w[0]-WREF[0], w[1]-WREF[1]]) * GAIN
    mm = Minv @ dpx                      # 팔 이동 = +Minv@Δpx (8/30 부호 실증)
    if max(abs(mm)) > 3.0: return None
    t = list(stt()["tcp"]); t[0] += float(mm[0]); t[1] += float(mm[1])
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    return mm

def descend_to(base, z_from, z_to):
    """접촉 감지하며 하강. 반환 (도달 z, 접촉여부, 접촉 방향(밑판이 밀린 화면벡터))"""
    prev = dots(); rate = {}; z = z_from; first = True
    while z > z_to + 0.2:
        t = list(base); t[2] = max(z_to, z - (FIRST if first else STEP))
        post("move_tcp", {"tcp": t, "dry_run": False}); wait()
        f = stt()
        if f["frozen"]: print("   🧊 동결"); return f["tcp"][2], True, None
        zr = f["tcp"][2]; dz = z - zr; now = dots()
        if dz < 0.15:                      # 로봇이 실제로 안 내려감(명령 미달·스테일) → 이동률 학습 불가
            print(f"      z{zr:.1f}  실제 이동 {dz:.2f}mm — 이번 스텝 판정 생략")
            if abs(zr - z) < 0.05 and zr <= z: return zr, True, None
            z = zr; continue
        worst = wall = 0.0; vec = None; newrate = {}
        for k, x, y in prev:
            isw = (k == CK and x > 600)
            r = rate.get((k, round(x/60), round(y/60)))
            ex, ey = (x + r[0]*dz, y + r[1]*dz) if r else (x, y)
            c = near(now, k, ex, ey)
            if not c: continue
            if r is None:
                newrate[(k, round(c[1]/60), round(c[2]/60))] = ((c[1]-x)/dz, (c[2]-y)/dz); continue
            e = ((c[1]-ex)**2 + (c[2]-ey)**2) ** 0.5
            if isw: wall = max(wall, 0.0 if first else e)
            else:
                if e > worst: worst, vec = e, (c[1]-ex, c[2]-ey)
                newrate[(k, round(c[1]/60), round(c[2]/60))] = ((c[1]-x)/dz, (c[2]-y)/dz)
        print(f"      z{zr:.1f}  강체 {worst:.1f}px  벽 {wall:.1f}px")
        if wall > G_WALL:
            # ★8/30: 벽이 그리퍼 안에서 미끄러지면 기준(파지 지문)이 무효 → 밀어붙이지 말고 중단.
            #   계속 미세보정하며 눌렀더니 벽이 기울어 어긋난 채 들어갔다(사용자 판정 '실패').
            print(f"      ⚠ 벽이 그리퍼 안에서 {wall:.1f}px 미끄러짐 → 탐색 중단(재파지 필요)")
            return zr, True, "SLIP"
        if worst > G_RIGID:
            return zr, True, vec
        rate.update(newrate); prev = now; z = zr; first = False
    return stt()["tcp"][2], False, None

post("speed", {"value": 1, "dry_run": False}); time.sleep(0.4)
base = list(stt()["tcp"]); z0 = base[2]
HOV = {"dots": [[k, round(x, 1), round(y, 1)] for k, x, y in dots()], "tcp": [round(v, 2) for v in base]}
acc_xy = np.zeros(2); acc_rz = 0.0; fixes = 0
best = z0; tries = 0
# 탐색 순서: ①접촉 방향의 반대(밑판이 밀린 쪽 = 벽이 파고든 쪽) ②X/Y 교대 ③J6 미세회전
plan = []
res_ok = False
print(f"[{CK}] 순응 삽입 시작 z{z0:.1f} → {ZT:.1f}")
while tries < MAXTRY:
    tries += 1
    zr, hit, vec = descend_to(base, stt()["tcp"][2], ZT)
    best = min(best, zr)
    if vec == "SLIP":
        # 8/30: 벽이 그리퍼 안에서 밀렸다 = 파지 기하가 바뀐 것. 무조건 중단하지 말고
        #   5mm 물러나 '지금 물린 상태' 기준으로 다시 맞춘 뒤 재개한다(최대 MAX_FIX 회).
        w = wall_now(); dev = (((w[0]-WREF[0])**2 + (w[1]-WREF[1])**2) ** 0.5) if (w and WREF) else 999
        up = list(base); up[2] = min(z0, zr + 5.0)
        post("move_tcp", {"tcp": up, "dry_run": False}); wait()
        if fixes < MAX_FIX and dev <= G_SLIP_FIX:
            mm = realign_wall()
            fixes += 1
            if mm is not None:
                base = list(stt()["tcp"]); base[2] = z0
                print(f"   ↻ 벽 재정렬 {fixes}/{MAX_FIX}: 기준 대비 {dev:.1f}px → 팔 X{mm[0]:+.2f} Y{mm[1]:+.2f}mm 보정 후 재개")
                continue
            print(f"   ↻ 재정렬 실패(벽 미검출/보정 과대) — 중단")
        else:
            print(f"   기준 대비 {dev:.1f}px (한계 {G_SLIP_FIX:.0f}px) 또는 재정렬 {fixes}회 소진")
        up2 = list(stt()["tcp"]); up2[2] = min(z0, up2[2] + 7.0)
        post("move_tcp", {"tcp": up2, "dry_run": False}); wait()
        print(f"\n=== 중단(벽 미끄러짐) · 최저 z{best:.1f} — 재파지 후 재시도 ===")
        post("speed", {"value": 3, "dry_run": False}); raise SystemExit(2)
    if not hit:
        print(f"   ✔ z{zr:.1f} 도달 — 삽입 완료"); res_ok = True; break
    if zr <= ZT + 4.0:
        print(f"   ✔ z{zr:.1f} 목표 ±4mm 접촉 = 바닥 안착"); res_ok = True
        up = list(base); up[2] = zr + 1.0; post("move_tcp", {"tcp": up, "dry_run": False}); wait(); break
    # 후퇴
    up = list(base); up[2] = min(z0, zr + BACK); post("move_tcp", {"tcp": up, "dry_run": False}); wait()
    if not plan:
        d0 = None
        if vec is not None:
            v = -(Minv @ np.array(vec)); n = np.linalg.norm(v)
            if n > 1e-6:
                d0 = (v/n) * NUDGE
                print(f"   접촉 방향 반대로 첫 시도: X{d0[0]:+.2f} Y{d0[1]:+.2f}mm")
        plan = ([(d0[0], d0[1], 0.0)] if d0 is not None else []) + [
            (NUDGE, 0, 0), (-2*NUDGE, 0, 0), (NUDGE, NUDGE, 0), (0, -2*NUDGE, 0),
            (0, 0, RZN), (0, 0, -2*RZN), (NUDGE, NUDGE, RZN), (-2*NUDGE, 0, 0),
            (0, 2*NUDGE, 0), (0, 0, 2*RZN), (NUDGE, -NUDGE, -RZN), (NUDGE, NUDGE, 0)]
    dx, dy, drz = plan.pop(0)
    if abs(acc_xy[0]+dx) > LIM_XY or abs(acc_xy[1]+dy) > LIM_XY or abs(acc_rz+drz) > LIM_RZ:
        print(f"   한계 초과 건너뜀 ({dx:+.2f},{dy:+.2f},{drz:+.2f})"); continue
    acc_xy += (dx, dy); acc_rz += drz
    t = list(stt()["tcp"]); t[0] += dx; t[1] += dy; t[5] += drz
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    base = list(stt()["tcp"]); base[2] = z0
    print(f"   시도 {tries}: 걸린 z{zr:.1f} → 미세보정 X{dx:+.2f} Y{dy:+.2f} rz{drz:+.2f}° "
          f"(누적 X{acc_xy[0]:+.2f} Y{acc_xy[1]:+.2f} rz{acc_rz:+.2f}°)")
print(f"\n=== {'성공' if res_ok else '실패'} · 시도 {tries}회 · 최저 z{best:.1f} · 누적 보정 X{acc_xy[0]:+.2f} Y{acc_xy[1]:+.2f} rz{acc_rz:+.2f}° ===")
if res_ok:
    # ★사용자 요청(8/30): 성공했을 때의 픽셀을 기억한다 — 호버 기하 + 최종 미세보정량
    import datetime
    c2 = json.load(open("/home/ar/bf2_console/dot_calib.json"))
    c2["refs_place"][CK].setdefault("success_log", []).append({
        "made": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
        "hover": HOV, "nudge_xy": [round(float(acc_xy[0]), 2), round(float(acc_xy[1]), 2)],
        "nudge_rz": round(acc_rz, 2), "final_z": round(best, 1), "tries": tries})
    json.dump(c2, open("/home/ar/bf2_console/dot_calib.json", "w"), ensure_ascii=False, indent=1)
    print(f"★성공 기하 저장: 호버 점 {len(HOV['dots'])}개 + 미세보정 X{acc_xy[0]:+.2f} Y{acc_xy[1]:+.2f} rz{acc_rz:+.2f}°")
    # 사용자 요청(8/30): z 높이별 벽↔밑판 거리·yaw 를 성공 기준으로 전부 저장
    try:
        print("  ", post("place_success", {"kind": CK, "dry_run": False,
                                           "nudge_xy": [round(float(acc_xy[0]), 2), round(float(acc_xy[1]), 2)],
                                           "nudge_rz": round(acc_rz, 2), "final_z": round(best, 1)})["result"])
    except Exception as e:
        print("   높이별 기하 저장 실패:", e)
    post("gripper", {"pos": 50, "dry_run": False}); time.sleep(12)
    post("speed", {"value": 3, "dry_run": False}); time.sleep(0.4)
    # 8/30 사용자: +60(z533) 로는 그리퍼가 걸려 벽을 손으로 못 뺀다 → 최소 z600 까지 후퇴
    up = list(stt()["tcp"]); up[2] = max(up[2] + 60, 600.0); post("move_tcp", {"tcp": up, "dry_run": False}); wait()
    print(f"놓기·후퇴 완료 z{stt()['tcp'][2]:.0f}")
post("speed", {"value": 3, "dry_run": False})
