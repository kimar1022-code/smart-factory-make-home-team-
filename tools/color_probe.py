#!/usr/bin/env python3
"""점 색 실측 → 임계 제안 (9/2).

    python3 color_probe.py                 # 현재 검출된 점들의 색을 실측
    python3 color_probe.py 8766 40         # 카메라 포트 / 표본 수
    python3 color_probe.py --at 930,590 blue   # 좌표 지정(검출이 안 되는 점도 측정)

왜 필요한가 (9/2 실증):
  평균 밝기는 102~109 로 안정적인데도 빨강이 20% 프레임에서 통째로 사라졌다.
  → 광량 문제가 아니라 '점 색이 HSV 임계 경계에 걸쳐 있어서' 들락날락하는 것.
  추측으로 임계를 넓히면 유령이 늘고 좁히면 놓친다. 실제 점 색을 재서 정해야 한다.

출력: 색별 H/S/V 중앙값과 5~95% 범위, 그리고 그 범위를 감싸는 임계 제안.
"""
import json
import statistics as S
import sys
import time
import urllib.request
from collections import defaultdict

PORT = 8766
N = 30


def get(u):
    return json.loads(urllib.request.urlopen(u, timeout=8).read())


def hsv_at(port, x, y, r=4):
    try:
        return get(f"http://127.0.0.1:{port}/hsv?x={int(x)}&y={int(y)}&r={r}")
    except Exception:
        return None


def band(vals, lo_p=5, hi_p=95):
    v = sorted(vals)
    if not v:
        return None
    n = len(v)
    return v[max(0, n * lo_p // 100)], S.median(v), v[min(n - 1, n * hi_p // 100)]


def main():
    args = [a for a in sys.argv[1:]]
    port, n = PORT, N
    fixed = []
    if "--at" in args:
        i = args.index("--at")
        xy = args[i + 1].split(",")
        kind = args[i + 2] if len(args) > i + 2 else "?"
        fixed.append((kind, float(xy[0]), float(xy[1])))
        args = args[:i]
    if args:
        port = int(args[0])
    if len(args) > 1:
        n = int(args[1])

    # 1) 기준 위치 정하기 — 지정 좌표가 없으면 현재 검출점(면적 큰 순)
    if not fixed:
        ds = get(f"http://127.0.0.1:{port}/dots?raw=1")["dots"]
        by = defaultdict(list)
        for d in ds:
            by[d["kind"]].append(d)
        for k, v in by.items():
            for d in sorted(v, key=lambda t: -t["area"])[:4]:
                fixed.append((k, d["px"], d["py"]))
    if not fixed:
        sys.exit("검출된 점이 없습니다. --at x,y 색 으로 좌표를 지정하세요.")

    print(f"카메라 :{port} · 표본 {n}회 · 측정점 {len(fixed)}개")
    e = get(f"http://127.0.0.1:{port}/expo") if port == 8766 else {}
    if e:
        print(f"현재 설정: 노출 {e.get('exposure')} · WB {e.get('wb')}(자동 {e.get('awb')}) "
              f"· 밝기 {e.get('bright', 0):.1f}")

    samples = defaultdict(lambda: {"h": [], "s": [], "v": [], "seen": 0})
    for _ in range(n):
        try:
            ds = get(f"http://127.0.0.1:{port}/dots?raw=1")["dots"]
        except Exception:
            ds = []
        live = defaultdict(int)
        for d in ds:
            live[d["kind"]] += 1
        for kind, x, y in fixed:
            r = hsv_at(port, x, y)
            if not r:
                continue
            h = r.get("hsv") or r.get("HSV") or [r.get("h"), r.get("s"), r.get("v")]
            if not h or h[0] is None:
                continue
            key = (kind, round(x), round(y))
            samples[key]["h"].append(float(h[0]))
            samples[key]["s"].append(float(h[1]))
            samples[key]["v"].append(float(h[2]))
            samples[key]["seen"] += 1
        time.sleep(0.3)

    print(f"\n{'점':<22} {'H (5%~중앙~95%)':<24} {'S':<24} {'V':<24}")
    per_color = defaultdict(lambda: {"h": [], "s": [], "v": []})
    for (kind, x, y), d in sorted(samples.items()):
        bh, bs, bv = band(d["h"]), band(d["s"]), band(d["v"])
        if not bh:
            continue
        print(f"{kind:>6}({x:>4},{y:>4})   "
              f"{bh[0]:>3.0f}~{bh[1]:>3.0f}~{bh[2]:>3.0f}{'':<12}"
              f"{bs[0]:>3.0f}~{bs[1]:>3.0f}~{bs[2]:>3.0f}{'':<12}"
              f"{bv[0]:>3.0f}~{bv[1]:>3.0f}~{bv[2]:>3.0f}")
        for c in "hsv":
            per_color[kind][c] += d[c]

    print("\n=== 색별 통합 + 임계 제안 (측정 범위에 여유 H±5 / S,V −20 적용) ===")
    for kind, d in sorted(per_color.items()):
        bh, bs, bv = band(d["h"]), band(d["s"]), band(d["v"])
        if not bh:
            continue
        lo = (max(0, bh[0] - 5), max(0, bs[0] - 20), max(0, bv[0] - 20))
        hi = (min(179, bh[2] + 5), 255, 255)
        print(f"  {kind:>6}: 실측 H {bh[0]:.0f}~{bh[2]:.0f} · S {bs[0]:.0f}~{bs[2]:.0f} "
              f"· V {bv[0]:.0f}~{bv[2]:.0f}")
        print(f"          → inRange(({lo[0]:.0f},{lo[1]:.0f},{lo[2]:.0f}), "
              f"({hi[0]:.0f},{hi[1]:.0f},{hi[2]:.0f}))")
    print("\n※ 색 간 H 구간이 겹치면 겹치는 쪽 여유를 줄여야 한다(유령의 주원인).")


if __name__ == "__main__":
    main()
