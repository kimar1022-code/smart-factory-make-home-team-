#!/usr/bin/env python3
"""HOUSE STL 실측 스크립트 (2026-09-04).

베이스(B_printbase.stl)의 기둥 노치(벽 끼우는 홈) 형상과 벽 판 두께를 단면으로 잰다.
결과는 docs/2026-09-04_재설계_답변_베이스yaw보정_벽파지.md 의 '실측 수치' 절의 근거.

필요: pip install numpy trimesh shapely scipy rtree networkx
사용: python3 tools/measure_house_stl.py
"""
import glob, os
import numpy as np
import trimesh
from shapely.geometry import Polygon, LineString

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "..", "cad")


def slice_xy(m, z):
    """z 높이 수평 단면을 XY 폴리곤 목록으로 (원래 좌표계 유지)."""
    s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if s is None:
        return []
    out = []
    for line in s.discrete:
        pts = np.asarray(line)[:, :2]
        if len(pts) >= 3:
            p = Polygon(pts)
            if p.is_valid and p.area > 0.01:
                out.append(p)
    return out


def base():
    m = trimesh.load(os.path.join(CAD, "base", "B_printbase.stl"), force="mesh")
    m.apply_translation(-m.bounds[0])  # 최소 모서리를 원점으로
    print("== B_printbase 외형(mm):", np.round(m.extents, 2))
    plate = slice_xy(m, 3)
    print("   밑판 z=3 단면 면적:", round(sum(p.area for p in plate)))
    cols = [p for p in slice_xy(m, 20) if p.area < 200]
    print(f"   z=20 기둥 {len(cols)}개")
    for p in cols:
        b = p.bounds
        print(f"   기둥 bbox x[{b[0]:.1f},{b[2]:.1f}] y[{b[1]:.1f},{b[3]:.1f}] 면적 {p.area:.1f}")
        print("     꼭짓점:", np.round(np.array(p.exterior.coords)[:-1], 2).tolist())
    # 기둥 단면이 높이에 따라 변하는지(=상단 리드인 챔퍼 유무)
    a85 = sum(p.area for p in slice_xy(m, 85.5) if p.area < 200)
    a20 = sum(p.area for p in cols)
    print(f"   기둥 단면 면적 z=20: {a20:.1f} / z=85.5: {a85:.1f}  -> 같으면 상단 리드인 챔퍼 없음")


def walls():
    for f in sorted(glob.glob(os.path.join(CAD, "집B_벽", "B_*.stl"))):
        m = trimesh.load(f, force="mesh")
        m.apply_translation(-m.bounds[0])
        ext = m.extents
        L = int(np.argmax(ext[:2]))
        polys = slice_xy(m, 40)
        print(f"== {os.path.basename(f)} 외형 {np.round(ext, 2)}  긴축 {'xy'[L]}")
        for p in polys:
            b = p.bounds
            for frac in (0.0, 0.02, 0.5):
                pos = b[L] + frac * (b[2 + L] - b[L])
                pos = min(max(pos, b[L] + 0.05), b[2 + L] - 0.05)
                ln = LineString([(pos, b[1] - 1), (pos, b[3] + 1)]) if L == 0 else LineString([(b[0] - 1, pos), (b[2] + 1, pos)])
                print(f"     {'xy'[L]}={pos:.2f} 두께 {p.intersection(ln).length:.2f}")


if __name__ == "__main__":
    base()
    walls()
