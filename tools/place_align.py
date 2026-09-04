#!/usr/bin/env python3
"""place 삽입 — X·Y·yaw 를 0.5mm/0.5° 단위로 맞추며 4단계 하강 (사용자 설계, 9/1 야간).

    python3 place_align.py [색] [--dry]

★사용자 규칙 (반드시 지킬 것)
  1. 보정은 **0.5mm(0.5°) 단위**로 살살. 팍팍 움직이면 지나쳐서 진동한다(9/1 실증:
     상한 4mm 로 하니 +22→-22→+19px 로 계속 튕겼고, 0.5mm 로 낮추니 5스텝 단조 수렴).
  2. **X·Y·yaw 전부** 맞춘다. 하나라도 남으면 밑동이 기둥에 걸린다.
  3. **수렴 못 하면 절대 안 넣는다.** 기둥이 부러진다. 중단하고 사람을 부른다.

★축척은 대상마다 다르다 — 반드시 그 대상의 실측 축척을 쓸 것(9/1 실패의 반복 원인)
    밑판 점(멀다)      3.105 px/mm   ← 팔을 움직이면 이게 움직인다
    랙에 꽂힌 벽        2.09 px/mm
    그리퍼에 문 벽(가깝다) 9.90 px/mm  ← 팔을 움직여도 화면에서 안 움직인다
  기준 대조는 '밑판 점'으로 하므로 보정 축척은 밑판 축척을 쓴다.
"""
import json
import math
import statistics as S
import sys
import time
import urllib.request

BRIDGE = "http://localhost:8765"
CAL = "/home/ar/bf2_console/dot_calib.json"
CAM1, CAM2 = 8766, 8768
BASE_PXMM = 3.105          # 밑판 점 축척(손목캠) — 팔 1mm 당 px
GAIN = 0.5                 # 계산값의 절반만 이동
CAP_MM = 0.5               # 1회 XY 이동 상한
CAP_DEG = 1.0              # 1회 J6 회전 상한(사용자 지시 9/1)
# ★9/1 실측: J6 는 1° 미만 명령에 거의 반응하지 않는다(명령 +0.5°→엔코더 +0.039°, +1.0°→+0.99°).
#   0.5° 로 잘라 보내면 12회를 돌려도 yaw 가 -2.0°에서 -1.95° 로만 움직인다(실증).
#   그래서 J6 만 1.5° 까지 허용한다. XY 는 사용자 지시대로 0.5mm 유지.
J6_MIN = 0.8               # 이보다 작은 J6 명령은 안 먹으므로 최소 이 값으로 보낸다(부호 유지)
TOL_PX = 3.0               # 위치 수렴 허용(≈1.0mm)
TOL_DEG = 0.15             # yaw 수렴 허용(126mm 끝단 0.33mm)
MAX_IT = 12


def post(a, b):
    r = urllib.request.Request(f"{BRIDGE}/fr5/{a}", json.dumps(b).encode(),
                               {"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=45).read())


def st():
    return json.loads(urllib.request.urlopen(BRIDGE + "/status", timeout=5).read())["robots"]["fr5"]


def wait(t=120):
    time.sleep(0.9)
    t0 = time.time()
    while time.time() - t0 < t and st()["busy"]:
        time.sleep(0.35)


def stable():
    p, n = None, 0
    for _ in range(50):
        t = st()["tcp"]
        if p and all(abs(a - b) < 0.05 for a, b in zip(t, p)):
            n += 1
        else:
            n = 0
        p = t
        if n >= 4:
            break
        time.sleep(0.3)
    return p


def dots(port, n=6):
    acc = {}
    for _ in range(n * 4):
        try:
            ds = json.loads(urllib.request.urlopen(
                f"http://127.0.0.1:{port}/dots?raw=1", timeout=5).read())["dots"]
        except Exception:
            continue
        for t in ds:
            k = None
            for kk in acc:
                if kk[0] == t["kind"] and (kk[1]-t["px"])**2 + (kk[2]-t["py"])**2 < 35**2:
                    k = kk
                    break
            if k is None:
                k = (t["kind"], t["px"], t["py"])
                acc[k] = []
            acc[k].append((t["px"], t["py"], t["area"]))
        time.sleep(0.1)
    return sorted([[k[0], round(S.median([p[0] for p in v]), 2),
                    round(S.median([p[1] for p in v]), 2),
                    round(S.median([p[2] for p in v]))]
                   for k, v in acc.items() if len(v) >= 3], key=lambda p: (p[0], p[2]))


HELD_MIN_AREA = 800.0   # ★물고 있는 벽 점은 카메라에 가까워 면적 1400~2000. 면적 60짜리 유령이
                        #   45px 안에 있어 벽 점으로 오분류되면 yaw 가 -8° 로 튄다(9/1 실증).


def split(ds, held_px, r=45.0):
    """물고 있는 벽 점(화면 고정) / 밑판 점(팔 따라 움직임) 분리. 면적으로 유령 배제."""
    held, scene = [], []
    for p in ds:
        near = any((p[1]-h[0])**2 + (p[2]-h[1])**2 < r*r for h in held_px)
        big = (len(p) < 4) or (p[3] is None) or (p[3] >= HELD_MIN_AREA)
        if near and big:
            held.append(p)
        elif not near:
            scene.append(p)
        # near 이지만 작은 점 = 유령 → 양쪽 다 제외
    return held, scene


SCENE_MIN_AREA = 150.0   # ★밑판 점 유령(면적 60)이 끼면 기준선이 통째로 바뀌어 상대각이
                         #   -1.4° ↔ +18.4° 로 널뛴다(9/1 실증). 면적으로 먼저 거른다.


def line_ang(pts, kinds=None):
    """두 점을 잇는 선의 각. kinds 를 주면 그 색 조합으로만 긋는다(점 쌍이 바뀌면 각이 튄다)."""
    ps = [p for p in pts if (len(p) < 4 or p[3] is None or p[3] >= SCENE_MIN_AREA)]
    if kinds:
        a = [p for p in ps if p[0] == kinds[0]]
        b = [p for p in ps if p[0] == kinds[1]]
        if not a or not b:
            return None
        pa = max(a, key=lambda p: p[3] or 0)
        pb = max(b, key=lambda p: p[3] or 0)
        return math.degrees(math.atan2(pb[2]-pa[2], pb[1]-pa[1])) % 180.0
    if len(ps) < 2:
        return None
    ps = sorted(ps, key=lambda p: p[2])
    return math.degrees(math.atan2(ps[-1][2]-ps[0][2], ps[-1][1]-ps[0][1])) % 180.0


def rel_yaw(held, scene, kinds=None):
    """★J6 로 제어할 수 있는 유일한 각 = '벽선 − 밑판선' 상대각.
    벽 점 2개만의 각은 J6 를 돌려도 안 변한다(카메라도 손목에 있어 함께 돈다, 9/1 실측 0.00px/°).
    그걸 0 으로 만들려고 J6 를 돌리면 밑판만 화면에서 회전해 XY 가 계속 틀어진다(실증: 21→36mm)."""
    w, b = line_ang(held), line_ang(scene, kinds)
    if w is None or b is None:
        return None
    return (w - b + 90.0) % 180.0 - 90.0


def pair(ref, cur, r=130.0):
    out = []
    for p in ref:
        cc = [q for q in cur if q[0] == p[0]]
        if not cc:
            continue
        q = min(cc, key=lambda q: (q[1]-p[1])**2 + (q[2]-p[2])**2)
        if (q[1]-p[1])**2 + (q[2]-p[2])**2 < r*r:
            out.append((p, q))
    return out


def main():
    ck = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "blue"
    dry = "--dry" in sys.argv
    cal = json.load(open(CAL))
    ref = cal["refs"][ck]
    rows = ref["descent_refs_place"]["rows"]
    it_tcp = ref["insert_tcp"]

    # 기준에서 '물고 있는 벽 점' 위치(높이를 바꿔도 안 움직이는 점)
    def held_px(cam):
        hi, lo = rows[0][cam], rows[-1][cam]
        out = []
        for p in hi:
            k = p[0] if isinstance(p[0], str) else None
            x, y = (p[1], p[2]) if k else (p[0], p[1])
            cand = [q for q in lo if (q[0] == k) if k] or lo
            for q in cand:
                qx, qy = (q[1], q[2]) if isinstance(q[0], str) else (q[0], q[1])
                if (qx-x)**2 + (qy-y)**2 < 12**2:
                    out.append((x, y))
                    break
        return out

    H1, H2 = held_px("cam1"), held_px("cam2")
    YAW_REF = [None]; SK = [None]
    print(f"[{ck}] 벽 점(기준): 손목캠 {[(round(a),round(b)) for a,b in H1]} · 새캠 {[(round(a),round(b)) for a,b in H2]}")
    print(f"  규칙: {CAP_MM}mm/{CAP_DEG}° 단위 · 수렴 실패 시 삽입 금지")

    post("speed", {"value": 1, "dry_run": False}); time.sleep(0.3)
    for ri, row in enumerate(rows):
        z = row["z"]
        t = list(stable()); t[2] = float(z)
        if not dry:
            post("move_tcp", {"tcp": t, "dry_run": False}); wait()
        time.sleep(0.6)

        ref1 = [[p[0], p[1], p[2], p[4] if len(p) > 4 else 0] if isinstance(p[0], str)
                else ["blue", p[0], p[1], 0] for p in row["cam1"]]
        rh, rs = split(ref1, H1)
        # ★벽 점은 카메라와 한 몸이라 높이가 달라도 화면에서 안 움직인다 → yaw 기준은 하나로 고정.
        #   높이마다 다시 재면 검출 잡음이 그대로 기준에 섞여 -8° 같은 값이 나온다(9/1 버그).
        # 기준선을 이루는 색 조합을 기준에서 뽑아 고정 — 현재도 같은 조합으로만 긋는다
        SK[0] = tuple(sorted({p[0] for p in rs if (p[3] or 0) >= SCENE_MIN_AREA}))[:2]
        if len(SK[0]) < 2:
            SK[0] = None
        ryaw = rel_yaw(rh, rs, SK[0])      # 기준의 상대각 — 높이마다 밑판선이 달라지므로 매 행에서 구한다

        ok = False
        for k in range(MAX_IT):
            d1 = dots(CAM1)
            ch, cs = split(d1, H1)
            pr = pair(rs, cs)
            if not pr:
                print(f"  z{z}: 밑판 점 매칭 실패 — 중단(삽입 안 함)"); return
            dx = S.median([q[1]-p[1] for p, q in pr])
            dy = S.median([q[2]-p[2] for p, q in pr])
            # 파지 편차: 벽 점이 기준에서 밀린 만큼 목표도 옮긴다(축척차 보정: 벽 9.9 → 밑판 3.105)
            hp = pair(rh, ch)
            if hp:
                hx = S.median([q[1]-p[1] for p, q in hp]) * (BASE_PXMM / 9.904)
                hy = S.median([q[2]-p[2] for p, q in hp]) * (BASE_PXMM / 9.904)
                dx -= hx; dy -= hy
            err = math.hypot(dx, dy)   # 파지편차 보정 후 남은 '밑판 대비' 오차
            cyaw = rel_yaw(ch, cs, SK[0])
            dyaw = (((cyaw - ryaw) + 90.0) % 180.0 - 90.0) if (cyaw is not None and ryaw is not None) else None
            dy_s = f"{dyaw:+.3f}°" if dyaw is not None else "측정불가"
            print(f"  z{z} [{k+1}] Δ({dx:+.1f},{dy:+.1f})px={err/BASE_PXMM:.2f}mm  yaw {dy_s}")
            if dyaw is None:
                print(f"  z{z}: yaw 측정 불가(벽 점 2개 필요) — 중단(삽입 안 함)"); return
            if err < TOL_PX and abs(dyaw) < TOL_DEG:
                ok = True
                break
            if dry:
                break                      # 측정만 (아래 ok 판정은 실제 err/yaw 로)
            if abs(dyaw) >= TOL_DEG:                      # yaw 먼저
                corr = max(-CAP_DEG, min(CAP_DEG, -dyaw / 1.034))
                if abs(corr) < J6_MIN:                    # 소각도는 안 먹는다 → 최소치로
                    corr = math.copysign(J6_MIN, corr)
                j = list(st()["joints"]); j[5] += corr
                post("move", {"joints": [round(v, 4) for v in j], "dry_run": False}); wait()
            if err >= TOL_PX:                              # XY 는 매 회 함께(yaw 대기 중에도 다듬는다)
                dX = max(-CAP_MM, min(CAP_MM, GAIN * dx / BASE_PXMM))
                dY = max(-CAP_MM, min(CAP_MM, GAIN * -dy / BASE_PXMM))
                t = list(stable()); t[0] -= dX; t[1] -= dY
                post("move_tcp", {"tcp": t, "dry_run": False}); wait()
            time.sleep(0.5)
        if not ok:
            msg = "측정값" if dry else "★삽입 금지(기둥 보호). 그 자리 정지"
            print(f"  z{z}: 미수렴 — {msg}")
            if not dry:
                return
        else:
            print(f"  z{z}: ✔ 통과")

    if dry:
        print("(--dry: 최종 삽입 생략)"); return
    print(f"  최종 수직 하강 z{it_tcp[2]}")
    t = list(stable()); t[2] = float(it_tcp[2])
    post("move_tcp", {"tcp": t, "dry_run": False}); wait()
    print("완료:", [round(v, 2) for v in stable()[:3]], "— 그리퍼 무변경")


if __name__ == "__main__":
    main()
