#!/usr/bin/env python3
"""꽂았다 뺐다 반복 + 마커↔색점 거리 분석 (9/2 오후 사용자 지시).

  python3 cycle_place_analyze.py [n]     # 전제: insert XY 상공 z478 에서 벽을 문 채(그리퍼 13)

1회 = golden_place.py run blue --no-approach (마커 앵커 정렬 → 하강 → 안착 판정)
     → 안착 자세 스냅샷 → 1% 로 z388/413/443/478 인출하며 스냅샷
     → 골든(golden_raw) 대비: 마커 중심 Δpx·각 Δ, 색점 Δpx, '마커 기준 색점 상대벡터' Δ
그리퍼는 절대 건드리지 않는다. 실패 시 그 자리 정지(러너가 정지 처리).
결과 JSON: logs/place_analyze_<시각>.json
"""
import json
import math
import subprocess
import sys
import time

sys.path.insert(0, "/home/ar/bf2_console/tools")
from golden import CAL, move_z, post, st, stable  # noqa: E402
from golden_capture import HEIGHTS, aruco_all, snap, fmt  # noqa: E402

HERE = "/home/ar/bf2_console/tools"
LOGDIR = "/home/ar/bf2_console/logs"


def nearest(kind, px, py, ds, r=60.0):
    best, bd = None, r * r
    for d in ds:
        if d[0] != kind:
            continue
        q = (d[1]-px)**2 + (d[2]-py)**2
        if q < bd:
            best, bd = d, q
    return best


def compare(g, c):
    """골든 행 g vs 현재 행 c → 캠별 {marker: {id: (dcx,dcy,dang)}, dots: [(kind, dpx, dpy)],
    rel: [(id, kind, d_rel_x, d_rel_y, d_dist)]}  (rel = 색점 − 마커중심 벡터의 골든 대비 변화)"""
    out = {}
    for cam in ("cam1", "cam2"):
        gm, cm = g[cam]["aruco"], c[cam]["aruco"]
        mk = {mid: (round(cm[mid]["cx"]-gm[mid]["cx"], 1), round(cm[mid]["cy"]-gm[mid]["cy"], 1),
                    round(cm[mid]["ang"]-gm[mid]["ang"], 2)) for mid in gm if mid in cm}
        dots, rel = [], []
        for gd in g[cam]["dots"]:
            cd = nearest(gd[0], gd[1], gd[2], c[cam]["dots"])
            if not cd:
                dots.append((gd[0], None, None)); continue
            dots.append((gd[0], round(cd[1]-gd[1], 1), round(cd[2]-gd[2], 1)))
            for mid in mk:
                grx, gry = gd[1]-gm[mid]["cx"], gd[2]-gm[mid]["cy"]
                crx, cry = cd[1]-cm[mid]["cx"], cd[2]-cm[mid]["cy"]
                rel.append((mid, gd[0], round(crx-grx, 1), round(cry-gry, 1),
                            round(math.hypot(crx, cry)-math.hypot(grx, gry), 1)))
        out[cam] = {"marker": mk, "dots": dots, "rel": rel}
    return out


def show(tag, cmp):
    for cam in ("cam1", "cam2"):
        v = cmp[cam]
        m = " ".join(f"id{k}Δ({a:+.0f},{b:+.0f},{c:+.1f}°)" for k, (a, b, c) in v["marker"].items()) or "마커 없음"
        d = " ".join(f"{k[:2]}Δ({a:+.0f},{b:+.0f})" if a is not None else f"{k[:2]}✗" for k, a, b in v["dots"])
        r = " ".join(f"id{i}-{k[:2]}({x:+.0f},{y:+.0f}|{dd:+.0f})" for i, k, x, y, dd in v["rel"])
        print(f"    {tag} {cam}: {m} | 점 {d}\n      마커기준 상대: {r}", flush=True)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    cal = json.load(open(CAL))
    ref = cal["refs"]["blue"]
    raw = ref["golden_raw"]
    it = ref["insert_tcp"]
    g_rows = {r["z"]: r for r in raw["rows"]}
    ts = time.strftime("%m%d_%H%M")
    result = {"made": ts, "cycles": []}
    for i in range(1, n + 1):
        print(f"\n━━━━━ place 분석 사이클 {i}/{n} ━━━━━", flush=True)
        t0 = time.time()
        rec = {"i": i}
        r = subprocess.run([sys.executable, "-u", f"{HERE}/golden_place.py", "run", "blue", "--no-approach"],
                           capture_output=True, text=True, timeout=900, cwd=HERE)
        out = (r.stdout + r.stderr).strip()
        open(f"{LOGDIR}/pa{ts}_c{i}_place.log", "w").write(out + "\n")
        for l in out.splitlines():
            if any(k in l for k in ("골든 일치", "파지검증", "삽입 완료", "안착 판정", "잔차")):
                print("  " + l.strip(), flush=True)
        ok = ("삽입 완료" in out) and r.returncode == 0
        rec["place_ok"] = ok
        if not ok:
            print("  ⛔ place 실패 — 러너가 정지시킴. 로봇 상태:", [round(v, 1) for v in st()["tcp"][:3]])
            print("  " + out[-600:].replace("\n", "\n  "))
            result["cycles"].append(rec)
            break
        p = stable()
        rec["seat_tcp"] = [round(v, 2) for v in p]
        rec["seat_delta"] = [round(p[k]-it[k], 2) for k in range(3)]
        print(f"  ✅ 안착 TCP {rec['seat_tcp'][:3]}  insert 대비 Δ{rec['seat_delta']}mm  ({time.time()-t0:.0f}초)", flush=True)
        s = snap(round(p[2], 1))
        rec["seat_snap"] = s
        cmp = compare(raw["seat"], s)
        rec["seat_cmp"] = cmp
        show("안착", cmp)

        # 인출 — 1% 수직, 높이별 스냅샷
        post("speed", {"value": 1, "dry_run": False}); time.sleep(0.3)
        rec["rows"] = []
        for z in HEIGHTS:
            move_z(z); time.sleep(0.8)
            sz = snap(z)
            cz = compare(g_rows[z], sz)
            rec["rows"].append({"z": z, "snap": sz, "cmp": cz})
            show(f"z{z:.0f}", cz)
        result["cycles"].append(rec)
        json.dump(result, open(f"{LOGDIR}/place_analyze_{ts}.json", "w"), ensure_ascii=False, indent=1)
    json.dump(result, open(f"{LOGDIR}/place_analyze_{ts}.json", "w"), ensure_ascii=False, indent=1)
    okn = sum(1 for c in result["cycles"] if c.get("place_ok"))
    print(f"\n━━━ 결과 {okn}/{len(result['cycles'])} 삽입 성공 · JSON logs/place_analyze_{ts}.json ━━━")
    print(f"▶ 현재 z{stable()[2]:.1f} 벽 든 채 대기 (그리퍼 {st()['gripper']})")


if __name__ == "__main__":
    main()
