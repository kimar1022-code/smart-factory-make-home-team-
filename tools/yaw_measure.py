#!/usr/bin/env python3
"""물고 있는 벽의 yaw(비틀림) 측정 + J6 보정 — 9/1 v2.

    python3 yaw_measure.py show               # 세 방법으로 측정·비교 (로봇 안 움직임)
    python3 yaw_measure.py prec [--n 30]      # 반복 정밀도(1σ) 실측
    python3 yaw_measure.py calib [--step 2]   # J6 ±step° 감도 실측 → dot_calib.json
    python3 yaw_measure.py ref                # 지금(정삽입 자세) 값을 기준으로 저장
    python3 yaw_measure.py fix [--max 3]      # 기준 대비 편차만큼 J6 보정

★원리 (9/1 실증)
  두 카메라가 모두 손목에 있다 → J6 를 돌리면 카메라와 벽이 함께 돈다. 그래서 '벽 위의 점/에지'
  만 보면 J6 감도가 0.00px/° 다(실측). 재야 하는 건 **벽선 ↔ 월드 기준선의 상대각**이고,
  J6 는 카메라를 돌리므로 월드 기준선이 화면에서 −1°/° 로 돈다 → 상대각 감도 ≈ +1.0°/°.

★기준선 = 밑판 ArUco 마커 2개의 중심을 잇는 선
  종전에는 '가장 먼 도트 두 개'를 매번 새로 골랐는데, 측정마다 다른 쌍이 뽑혀 각도가 7° 씩
  튀었다(9/1 캘리브 실패의 진범). ArUco 는 ID 로 신원이 고정되고 코너가 서브픽셀이라
  실측 σ 0.016° 로 안정적이다.

측정 3종 (사용자 요청 A·B·C)
  A dots : 물고 있는 벽의 스티커 2개를 잇는 선   (σ 0.018°, 가장 견고)
  B edge : 손목캠에서 벽 실루엣 에지(Canny+Hough)
  C side : 새카메라 측면 뷰의 벽 긴 모서리 에지
"""
import argparse
import json
import math
import sys
import time
import urllib.request

import cv2
import numpy as np

BRIDGE = "http://localhost:8765"
CAL = "/home/ar/bf2_console/dot_calib.json"
CAMS = {"cam1": "http://localhost:8766", "cam2": "http://localhost:8768"}
HELD_AREA = 700.0        # 이보다 큰 점 = 물고 있는 벽(카메라에 가까워 크게 보인다). 밑판 점은 150~300


def post(a, b):
    r = urllib.request.Request(f"{BRIDGE}/fr5/{a}", json.dumps(b).encode(),
                               {"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(r, timeout=40).read())


def st():
    return json.loads(urllib.request.urlopen(BRIDGE + "/status", timeout=5).read())["robots"]["fr5"]


def wait(t=90):
    time.sleep(0.9)
    t0 = time.time()
    while time.time() - t0 < t and st()["busy"]:
        time.sleep(0.3)


def get(url):
    return json.loads(urllib.request.urlopen(url, timeout=6).read())


def ang(p, q):
    """두 점을 잇는 선의 각(도, 0~180)."""
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) % 180.0


def wrap(a):
    """-90~+90 로 접기."""
    return (a + 90.0) % 180.0 - 90.0


def base_angle(cam, ids=None, max_age=1.5):
    """밑판 ArUco 마커 2개 중심선 각 — 신원 고정 기준선.
    ★9/1: 마커가 프레임 밖으로 나가면 서버가 직전 검출을 그대로 들고 있다(age 로 판별).
      낡은 값으로 각을 재면 조용히 틀린다 → age 초과면 '미검출' 로 처리."""
    r = get(CAMS[cam] + "/aruco")
    if r.get("age") is None or r["age"] > max_age:
        return None, None
    ms = {m["id"]: np.array(m["corners"], float) for m in r["markers"]}
    if ids is None:
        ids = sorted(ms)[:2]
    if len(ids) < 2 or not all(i in ms for i in ids):
        return None, None
    c = [ms[i].mean(axis=0) for i in ids]
    return ang(c[0], c[1]), list(ids)


def held_dots(cam, kind="blue"):
    """물고 있는 벽의 점들 — 면적으로 구분(밑판 점보다 훨씬 크게 보인다)."""
    ds = get(CAMS[cam] + "/dots?raw=1")["dots"]
    h = [t for t in ds if t["kind"] == kind and t["area"] >= HELD_AREA]
    h.sort(key=lambda t: -t["area"])
    return [(t["px"], t["py"], t["area"]) for t in h]


def edge_angle(cam, near, half=130):
    """B/C: near 주변 창에서 Canny+Hough 지배 선분 각."""
    d = urllib.request.urlopen(CAMS[cam] + "/raw", timeout=8).read()
    img = cv2.imdecode(np.frombuffer(d, np.uint8), cv2.IMREAD_COLOR)
    h, w = img.shape[:2]
    x0, y0 = max(0, int(near[0]-half)), max(0, int(near[1]-half))
    x1, y1 = min(w, int(near[0]+half)), min(h, int(near[1]+half))
    roi = img[y0:y1, x0:x1]
    if roi.size == 0:
        return None, 0
    g = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    ls = cv2.HoughLinesP(cv2.Canny(g, 40, 120), 1, np.pi/720, threshold=45,
                         minLineLength=max(35, half//2), maxLineGap=6)
    if ls is None:
        return None, 0
    segs = sorted(((math.hypot(l[2]-l[0], l[3]-l[1]),
                    math.degrees(math.atan2(l[3]-l[1], l[2]-l[0])) % 180.0)
                   for l in ls[:, 0]), reverse=True)
    a0 = segs[0][1]
    sel = [a for L, a in segs[:max(3, len(segs)//4)]
           if min(abs(a-a0), 180-abs(a-a0)) < 12]
    return (float(np.mean(sel)) if sel else a0), len(sel)


def measure(cfg=None, want_edges=True):
    """벽선 − 기준선 상대각 (A/B/C). 로봇은 움직이지 않는다."""
    cfg = cfg or {}
    out = {}
    for cam, key in (("cam2", "A"), ):
        b, ids = base_angle(cam, cfg.get(cam + "_ids"))
        if b is None:
            out["error"] = f"{cam} ArUco 마커 2개 미검출"
            return out
        out["base_" + cam] = round(b, 3)
        out["base_ids_" + cam] = ids
    h2 = held_dots("cam2")
    if len(h2) >= 2:
        w = ang(h2[0], h2[1])
        out["A_wall"] = round(w, 3)
        out["A_rel"] = round(wrap(w - out["base_cam2"]), 3)
        out["A_base_px"] = round(math.dist(h2[0][:2], h2[1][:2]), 1)
    if want_edges and h2:
        c = (float(np.mean([p[0] for p in h2])), float(np.mean([p[1] for p in h2])))
        a, n = edge_angle("cam2", c)
        if a is not None:
            out["C_rel"] = round(wrap(a - out["base_cam2"]), 3); out["C_n"] = n
    b1, ids1 = base_angle("cam1", cfg.get("cam1_ids"))
    h1 = held_dots("cam1")
    if b1 is not None:
        out["base_cam1"] = round(b1, 3); out["base_ids_cam1"] = ids1
        if len(h1) >= 2:
            w1 = ang(h1[0], h1[1])
            out["A1_rel"] = round(wrap(w1 - b1), 3)
        if want_edges and h1:
            a, n = edge_angle("cam1", (h1[0][0], h1[0][1]))
            if a is not None:
                out["B_rel"] = round(wrap(a - b1), 3); out["B_n"] = n
    return out


NAMES = {"A_rel": "A 벽점2개(새캠)", "A1_rel": "A' 벽점2개(손목캠)",
         "B_rel": "B 손목캠 에지", "C_rel": "C 새캠 측면에지"}


def show(m, tag=""):
    print(f"--- {tag}")
    if "error" in m:
        print("  ⚠", m["error"]); return
    for k, nm in NAMES.items():
        if k in m:
            print(f"  {nm:18s} 상대각 {m[k]:+8.3f}°")
    print(f"  (기준선 새캠 {m.get('base_cam2')}° id{m.get('base_ids_cam2')}"
          f" · 손목캠 {m.get('base_cam1')}° id{m.get('base_ids_cam1')}"
          f" · 벽 기선 {m.get('A_base_px')}px)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["show", "prec", "calib", "ref", "fix"])
    ap.add_argument("--step", type=float, default=2.0)
    ap.add_argument("--max", type=float, default=3.0)
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--tol", type=float, default=0.05, help="수렴 허용 편차(도). 0.05°=126mm 끝단 0.11mm")
    ap.add_argument("--use", default="A1_rel", help="보정에 쓸 측정 — 기본 A1_rel(손목캠 탑뷰: "
                    "화면각≈실제 yaw, J6 감도 1.034로 이론값에 가장 근접)")
    a = ap.parse_args()
    cal = json.load(open(CAL))
    cfg = cal.get("yaw_cfg", {})

    if a.mode == "show":
        show(measure(cfg), "측정 (현재)")
        return

    if a.mode == "prec":
        acc = {}
        for _ in range(a.n):
            m = measure(cfg, want_edges=False)
            for k in ("A_rel", "A1_rel"):
                if k in m:
                    acc.setdefault(k, []).append(m[k])
            time.sleep(0.12)
        print(f"--- 반복 정밀도 ({a.n}회, 로봇 정지)")
        for k, v in acc.items():
            v = np.array(v)
            print(f"  {NAMES[k]:18s} 평균 {v.mean():+8.3f}°  1σ {v.std():.4f}°  "
                  f"범위 {v.max()-v.min():.4f}°  → 126mm 벽 끝단 {126*math.tan(math.radians(v.std())):.3f}mm")
        return

    if a.mode == "ref":
        m = measure(cfg)
        cal["yaw_ref"] = {k: v for k, v in m.items()}
        cal["yaw_ref"]["made"] = time.strftime("%Y-%m-%d %H:%M")
        cal["yaw_ref"]["j6"] = round(st()["joints"][5], 3)
        cal["yaw_cfg"] = {"cam1_ids": m.get("base_ids_cam1"), "cam2_ids": m.get("base_ids_cam2")}
        json.dump(cal, open(CAL, "w"), indent=1, ensure_ascii=False)
        show(m, "기준 저장")
        return

    if a.mode == "calib":
        j0 = list(st()["joints"])
        base = measure(cfg); show(base, f"J6={j0[5]:.2f}° 기준")
        sens = {}
        for d in (+a.step, -a.step):
            j = list(j0); j[5] = j0[5] + d
            post("move", {"joints": [round(v, 3) for v in j], "dry_run": False}); wait()
            m = measure(cfg); show(m, f"J6{d:+.1f}°")
            for k in NAMES:
                if k in m and k in base:
                    sens.setdefault(k, []).append(wrap(m[k] - base[k]) / d)
            post("move", {"joints": [round(v, 3) for v in j0], "dry_run": False}); wait()
        print("--- J6 감도 (상대각 ° / J6 1°)   ※이론값 +1.0")
        out = {}
        for k, v in sens.items():
            out[k] = round(float(np.mean(v)), 4)
            print(f"  {NAMES[k]:18s} {out[k]:+.3f}   (±방향 {[round(x,3) for x in v]})")
        cal["yaw_j6_sens"] = out
        cal["yaw_j6_made"] = time.strftime("%Y-%m-%d %H:%M")
        cal["yaw_cfg"] = {"cam1_ids": base.get("base_ids_cam1"), "cam2_ids": base.get("base_ids_cam2")}
        json.dump(cal, open(CAL, "w"), indent=1, ensure_ascii=False)
        print("저장: dot_calib.json yaw_j6_sens / yaw_cfg")
        return

    if a.mode == "fix":
        ref, sens = cal.get("yaw_ref"), cal.get("yaw_j6_sens") or {}
        if not ref:
            sys.exit("기준 없음 — 정삽입 자세에서 'ref' 먼저")
        k = a.use if a.use in ref else "A1_rel"
        # ★9/1 실증: J6 엔코더 readback 은 1° 미만에서 못 믿는다(명령 +0.5° → 엔코더 +0.039°,
        #   그러나 실제 회전은 +0.35°). 그래서 '명령 대비 전달률'(실측 ≈0.9)로 열고,
        #   엔코더가 아니라 **측정각이 기준에 닿을 때까지 반복**해서 닫는다.
        GAIN = float(cal.get("yaw_j6_cmd_gain", 0.9))
        for it in range(7):
            m = measure(cfg, want_edges=False)
            if k not in m:
                sys.exit(f"{k} 측정 실패 — {m.get('error','')}")
            dev = wrap(m[k] - ref[k])
            print(f"  [{it+1}] {NAMES[k]} {m[k]:+.3f}° (기준 {ref[k]:+.3f}) 편차 {dev:+.3f}°")
            if abs(dev) <= a.tol:
                print(f"  ✔ 수렴 (허용 {a.tol}°) — 126mm 벽 끝단 "
                      f"{126*math.tan(math.radians(abs(dev))):.3f}mm")
                return
            corr = max(-a.max, min(a.max, -dev / GAIN))
            print(f"      → J6 {corr:+.3f}° 명령")
            j = list(st()["joints"]); j[5] += corr
            post("move", {"joints": [round(v, 4) for v in j], "dry_run": False}); wait()
            time.sleep(0.6)
        m = measure(cfg, want_edges=False)
        dev = wrap(m[k] - ref[k]) if k in m else None
        print(f"  ⚠ 4회 내 미수렴 — 남은 편차 {dev:+.3f}°" if dev is not None else "  ⚠ 미수렴")


if __name__ == "__main__":
    main()
