#!/usr/bin/env python3
"""거리 기준 정렬 — 사용자 설계 (9/1 야간 확정).

    python3 dist_align.py teach [색]     # 지금 자세(성공 자세)에서 4단계 높이 기준 거리 기록
    python3 dist_align.py run   [색]     # 그 거리가 되도록 X·Y·yaw 맞추며 4단계 하강
    python3 dist_align.py show  [색]     # 현재 거리 vs 기준 거리만 출력(로봇 안 움직임)

★기준은 '점의 절대 픽셀'이 아니라 **점 사이의 거리**다.
  파지가 매번 조금씩 달라도, 벽과 밑판의 실제 기하 관계(=거리)가 성공했을 때와 같으면 들어간다.
  절대 위치를 맞추려 하면 파지가 다를 때 영원히 안 맞고 팔만 엉뚱하게 간다(9/1 실패의 진짜 원인).

★쓰는 점 (사용자가 캡쳐로 지정, 9/1)
    손목캠(:8766)  벽 파랑 2개  ↔  밑판 **노랑** 1개   → 거리 2개
    새카메라(:8768) 벽 파랑 2개  ↔  밑판 **빨강** 1개   → 거리 2개
  합계 거리 4개 → X·Y·yaw 3자유도를 최소자승으로 푼다.

★수학
  벽 점은 카메라와 한 몸이라 팔을 움직여도 화면에서 안 움직인다. 밑판 점만 움직인다.
  거리 d = |밑판 − 벽| 일 때, 팔이 (dX,dY) 움직이면 밑판이 (s·dX, −s·dY) px 이동하므로
      ∂d/∂dX = +s·(bx−wx)/d ,  ∂d/∂dY = −s·(by−wy)/d
  J6 는 카메라를 돌리므로 밑판 점이 화면 중앙 기준으로 회전한다(벽 점은 그대로).

★안전 규칙 (사용자 지시)
  · 보정은 0.5mm / 1.0° 단위. 팍팍 움직이면 지나쳐 진동한다.
  · 수렴 못 하면 **절대 삽입하지 않는다**. 기둥이 부러진다.
  · 유령 점(면적 작은 것)은 먼저 거른다 — 유령 하나가 기준선을 통째로 바꾼다.
"""
import json
import math
import statistics as S
import sys
import time
import urllib.request

BRIDGE = "http://localhost:8765"
CAL = "/home/ar/bf2_console/dot_calib.json"
CAM = {"cam1": 8766, "cam2": 8768}
BASE_S = {"cam1": 3.105, "cam2": 2.663}   # 밑판 점 축척 px/mm (손목캠 실측 / 새캠 z478 실측)
WALL_MIN = 800.0        # 물고 있는 벽 점 최소 면적(가까워서 크다)
SCENE_MIN = 150.0       # 밑판 점 최소 면적 — 유령(60)을 거른다
SCENE_KIND = {"cam1": "yellow", "cam2": "red"}   # ★사용자 지정
GAIN, CAP_MM, CAP_DEG = 0.5, 0.5, 1.0
TOL_PX, MAX_IT = 4.0, 14
HEIGHTS_OFF = [125, 90, 60, 35]           # 삽입 z 기준 +offset (높은 곳부터)


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


def dots(cam, n=6):
    port = CAM[cam]
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
    return [{"kind": k[0], "x": round(S.median([p[0] for p in v]), 2),
             "y": round(S.median([p[1] for p in v]), 2),
             "a": round(S.median([p[2] for p in v]))}
            for k, v in acc.items() if len(v) >= 3]


def pick_points(cam, ds):
    """벽 파랑 2개(면적 큰 순) + 밑판 지정색 1개(면적 큰 것)."""
    wall = sorted([d for d in ds if d["kind"] == "blue" and d["a"] >= WALL_MIN],
                  key=lambda d: d["y"])
    sk = SCENE_KIND[cam]
    sc = [d for d in ds if d["kind"] == sk and d["a"] >= SCENE_MIN]
    scene = max(sc, key=lambda d: d["a"]) if sc else None
    return wall, scene


def dists(cam, ds):
    """[(벽점, 밑판점, 거리), ...] — 벽 점 개수만큼."""
    wall, scene = pick_points(cam, ds)
    if len(wall) < 1 or scene is None:
        return None, wall, scene
    return [{"w": [w["x"], w["y"]], "s": [scene["x"], scene["y"]],
             "d": round(math.hypot(scene["x"]-w["x"], scene["y"]-w["y"]), 2)}
            for w in wall], wall, scene


def measure():
    out = {}
    for cam in ("cam1", "cam2"):
        ds = dots(cam)
        dd, wall, scene = dists(cam, ds)
        out[cam] = {"dists": dd, "wall": wall, "scene": scene, "all": ds}
    return out


def solve(ref, cur):
    """거리 오차를 없애는 (dX, dY) 최소자승. 반환 mm."""
    A, b = [], []
    for cam in ("cam1", "cam2"):
        r, c = ref.get(cam), cur[cam]["dists"]
        if not r or not c:
            continue
        s = BASE_S[cam]
        for i in range(min(len(r), len(c))):
            wx, wy = c[i]["w"]
            sx, sy = c[i]["s"]
            d = max(c[i]["d"], 1.0)
            A.append([s*(sx-wx)/d, -s*(sy-wy)/d])
            b.append(r[i]["d"] - c[i]["d"])          # 목표거리 − 현재거리
    if len(A) < 2:
        return None, 0
    import numpy as np
    A = np.array(A, float); b = np.array(b, float)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    resid = float(np.sqrt(np.mean(b**2)))
    return (float(sol[0]), float(sol[1])), resid


def show_cmp(ref, cur):
    for cam in ("cam1", "cam2"):
        r, c = ref.get(cam), cur[cam]["dists"]
        nm = "손목캠" if cam == "cam1" else "새카메라"
        if not c:
            w, s = cur[cam]["wall"], cur[cam]["scene"]
            print(f"    {nm}: 점 부족 (벽 {len(w)}개, 밑판{SCENE_KIND[cam]} {'있음' if s else '없음'})")
            continue
        txt = " · ".join(f"{c[i]['d']:.1f}" + (f"(기준 {r[i]['d']:.1f}, {c[i]['d']-r[i]['d']:+.1f})" if r and i < len(r) else "")
                         for i in range(len(c)))
        print(f"    {nm}: {txt}")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "show"
    ck = sys.argv[2] if len(sys.argv) > 2 else "blue"
    cal = json.load(open(CAL))
    ref = cal["refs"][ck]
    it = ref["insert_tcp"]

    if mode == "teach":
        rows = []
        post("speed", {"value": 1, "dry_run": False}); time.sleep(0.3)
        for off in sorted(HEIGHTS_OFF):                     # 낮은 곳부터 올라가며
            z = it[2] + off
            t = list(stable()); t[2] = z
            post("move_tcp", {"tcp": t, "dry_run": False}); wait(); time.sleep(0.7)
            m = measure()
            row = {"z": round(z, 1)}
            for cam in ("cam1", "cam2"):
                row[cam] = m[cam]["dists"]
            rows.append(row)
            print(f"  z{z:.0f}:", end=" ")
            show_cmp({}, m)
        rows.sort(key=lambda r: -r["z"])
        ref["dist_refs"] = {"made": time.strftime("%Y-%m-%d %H:%M"),
                            "scene_kind": SCENE_KIND, "final_z": it[2], "rows": rows}
        cal["refs"][ck] = ref
        json.dump(cal, open(CAL, "w"), indent=1, ensure_ascii=False)
        print(f"저장: refs.{ck}.dist_refs ({len(rows)}개 높이)")
        return

    if mode == "show":
        m = measure()
        dr = ref.get("dist_refs")
        z = stable()[2]
        r0 = {}
        if dr:
            near = min(dr["rows"], key=lambda r: abs(r["z"] - z))
            r0 = {c: near[c] for c in ("cam1", "cam2")}
            print(f"  (가장 가까운 기준 높이 z{near['z']}, 현재 z{z:.1f})")
        show_cmp(r0, m)
        if r0:
            sol, resid = solve(r0, m)
            print(f"  → 보정 필요량 {sol} mm, 거리잔차 {resid:.1f}px" if sol else "  → 계산 불가")
        return

    if mode == "run":
        dr = ref.get("dist_refs")
        if not dr:
            sys.exit("기준 없음 — 'teach' 먼저")
        post("speed", {"value": 1, "dry_run": False}); time.sleep(0.3)
        for row in dr["rows"]:
            z = row["z"]
            t = list(stable()); t[2] = float(z)
            post("move_tcp", {"tcp": t, "dry_run": False}); wait(); time.sleep(0.6)
            r0 = {c: row[c] for c in ("cam1", "cam2")}
            ok = False
            for k in range(MAX_IT):
                m = measure()
                sol, resid = solve(r0, m)
                if sol is None:
                    print(f"  z{z}: 점 부족 — 중단(삽입 안 함)"); show_cmp(r0, m); return
                print(f"  z{z} [{k+1}] 거리잔차 {resid:5.1f}px  보정 ({sol[0]:+.2f},{sol[1]:+.2f})mm")
                if resid < TOL_PX:
                    ok = True
                    break
                dX = max(-CAP_MM, min(CAP_MM, GAIN * sol[0]))
                dY = max(-CAP_MM, min(CAP_MM, GAIN * sol[1]))
                t = list(stable()); t[0] += dX; t[1] += dY
                post("move_tcp", {"tcp": t, "dry_run": False}); wait(); time.sleep(0.45)
            if not ok:
                print(f"  z{z}: {MAX_IT}회 내 미수렴 — ★삽입 금지(기둥 보호)"); return
            print(f"  z{z}: ✔ 통과")
        print(f"  최종 수직 하강 z{dr['final_z']}")
        t = list(stable()); t[2] = float(dr["final_z"])
        post("move_tcp", {"tcp": t, "dry_run": False}); wait()
        print("완료:", [round(v, 2) for v in stable()[:3]], "— 그리퍼 무변경")
        return

    print("사용법: dist_align.py teach|run|show [색]")


if __name__ == "__main__":
    main()
