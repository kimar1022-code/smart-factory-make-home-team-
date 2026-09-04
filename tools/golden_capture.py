#!/usr/bin/env python3
"""골든 place 캡처 (9/2 오후, 사용자 수동 안착 자세 기준).

  python3 golden_capture.py capture blue

사용자가 벽을 손으로 자리 잡은 '지금 자세'를 insert 기준으로 저장하고, 1% 로 수직 인출하며
z388/413/443/478 네 높이에서 (a) 색점 3종(파랑=벽·노랑/빨강=기둥) (b) 보이는 ArUco 마커 전부를
기록한다. 러너(golden_place.py)가 읽는 dist_refs·aruco_place 를 그대로 갱신하고, 원시값은
golden_raw 에 따로 남긴다(마커↔색점 거리 분석용). 그리퍼는 건드리지 않는다.
"""
import json
import math
import shutil
import statistics as S
import sys
import time
import urllib.request

sys.path.insert(0, "/home/ar/bf2_console/tools")
from golden import CAL, PORT, dots, move_z, post, st, stable  # noqa: E402

HEIGHTS = [388.0, 413.0, 443.0, 478.0]          # 러너 TOL 표와 같은 절대 높이
PRIMARY = {"cam1": 31, "cam2": 32}               # 러너 기본 앵커 id
PILLAR = {"cam1": "yellow", "cam2": "red"}


def aruco_all(cam, n=6):
    acc = {}
    for _ in range(n):
        try:
            ms = json.loads(urllib.request.urlopen(
                f"http://127.0.0.1:{PORT[cam]}/aruco", timeout=4).read())["markers"]
        except Exception:
            continue
        for m in ms:
            c = m["corners"]
            cx = sum(q[0] for q in c) / 4
            cy = sum(q[1] for q in c) / 4
            ang = math.degrees(math.atan2(c[1][1]-c[0][1], c[1][0]-c[0][0]))
            span = sum(math.hypot(c[(i+1) % 4][0]-c[i][0], c[(i+1) % 4][1]-c[i][1]) for i in range(4)) / 4
            acc.setdefault(m["id"], []).append((cx, cy, ang, span))
        time.sleep(0.25)
    out = {}
    for mid, v in acc.items():
        if len(v) >= 3:
            out[str(mid)] = {"id": mid, "cx": round(S.median(p[0] for p in v), 2),
                             "cy": round(S.median(p[1] for p in v), 2),
                             "ang": round(S.median(p[2] for p in v), 3),
                             "span": round(S.median(p[3] for p in v), 1), "n": len(v)}
    return out


def snap(z):
    row = {"z": z, "tcp": [round(v, 2) for v in stable()]}
    for cam in ("cam1", "cam2"):
        row[cam] = {"dots": dots(cam), "aruco": aruco_all(cam)}
    return row


WALL_KIND = "blue"
GRASP = None          # main 에서 refs.<색>.grasp_ref — 벽점 선별 기준(벽은 카메라와 한 몸이라 화면 위치가 고정)


def dist_row(raw):
    r = {"z": raw["z"]}
    for cam in ("cam1", "cam2"):
        ds = raw[cam]["dots"]
        if GRASP:
            gw = GRASP[f"{cam}_wall"]
            ws = [d for d in ds if d[0] == WALL_KIND
                  and any(math.hypot(d[1]-g[1], d[2]-g[2]) < 60 for g in gw)]
        else:
            ws = [d for d in ds if d[0] == WALL_KIND]
        ps = sorted([d for d in ds if d[0] == PILLAR[cam] and d not in ws], key=lambda d: -d[3])
        # ★9/2 저녁: 기둥점이 없어도 벽점(w)은 남긴다 — 러너(마커 앵커)는 w 만 쓴다. 노랑 슬롯 cam1 은 기둥점 색이 다름.
        s = ps[0] if ps else None
        r[cam] = [{"w": [w[1], w[2]], "s": ([s[1], s[2]] if s else None),
                   "d": (round(math.hypot(s[1]-w[1], s[2]-w[2]), 2) if s else None)} for w in ws]
    return r


def fmt(raw):
    out = []
    for cam in ("cam1", "cam2"):
        d = " ".join(f"{x[0][:2]}({x[1]:.0f},{x[2]:.0f})a{x[3]}" for x in raw[cam]["dots"])
        a = " ".join(f"id{m['id']}({m['cx']:.0f},{m['cy']:.0f},{m['ang']:+.1f}°)" for m in raw[cam]["aruco"].values())
        out.append(f"    {cam}: {d} | {a or '마커 없음'}")
    return "\n".join(out)


def main():
    if len(sys.argv) < 3 or sys.argv[1] != "capture":
        sys.exit("사용법: golden_capture.py capture blue")
    ck = sys.argv[2]
    cal = json.load(open(CAL))
    ref = cal["refs"][ck]
    global WALL_KIND, GRASP
    WALL_KIND = ref.get("dot_color", ck)
    GRASP = ref.get("grasp_ref")
    bak = CAL + time.strftime(".BEFORE_%m%d_%H%M")
    shutil.copy(CAL, bak)
    print(f"백업 {bak}")

    g = st()["gripper"]
    p0 = stable()
    j0 = st()["joints"]
    print(f"안착 자세 tcp {p0}  joints {j0}  그리퍼 {g}", flush=True)
    seat = snap(round(p0[2], 1))
    print("  [안착]\n" + fmt(seat), flush=True)

    post("speed", {"value": 1, "dry_run": False}); time.sleep(0.3)
    raws = []
    try:
        for z in HEIGHTS:
            move_z(z); time.sleep(0.8)
            r = snap(z)
            raws.append(r)
            print(f"  [z{z:.0f}]\n" + fmt(r), flush=True)
    except BaseException as e:
        try:
            post("stop", {"dry_run": False})
        except Exception:
            pass
        print(f"❌ 중단(정지함): {e}")
        raise

    made = time.strftime("%Y-%m-%d %H:%M")
    ref["insert_tcp"] = [round(v, 2) for v in p0]
    ref["insert_joints"] = [round(v, 3) for v in j0]
    ref["insert_note"] = f"{made} 사용자 수동 안착 자세(기둥 교체 후) — golden_capture.py"
    ref["dist_refs"] = {"made": made, "scene_kind": PILLAR, "final_z": round(p0[2], 1),
                        "rows": [dist_row(r) for r in sorted(raws, key=lambda r: -r["z"])]}
    ap_rows = []
    for r in sorted(raws, key=lambda r: -r["z"]):
        if r["z"] < 440:
            continue
        row = {"z": r["z"]}
        for cam in ("cam1", "cam2"):
            if r[cam]["aruco"]:
                row[cam] = r[cam]["aruco"]          # 보이는 마커 전부 (id별) — 러너 다중 앵커
        ap_rows.append(row)
    ref["aruco_place"] = {"made": made, "note": "사용자 수동 안착 기준 재캡처 — golden_capture.py",
                          "ids": {c: [int(k) for k in ap_rows[0].get(c, {})] for c in ("cam1", "cam2")} if ap_rows else PRIMARY, "tcp_line": [round(p0[0], 2), round(p0[1], 2)], "rows": ap_rows}
    ref["golden_raw"] = {"made": made, "seat": seat, "rows": raws,
                         "note": "높이별 색점 전부 + 보이는 마커 전부(마커↔색점 거리 분석용)"}
    cal["refs"][ck] = ref
    json.dump(cal, open(CAL, "w"), indent=1, ensure_ascii=False)
    print(f"저장 완료: insert_tcp / dist_refs({len(raws)}단) / aruco_place({len(ap_rows)}단) / golden_raw")
    print(f"▶ 현재 z{stable()[2]:.1f} 벽 든 채 대기 (그리퍼 {st()['gripper']})")


if __name__ == "__main__":
    main()
