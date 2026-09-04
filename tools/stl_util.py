#!/usr/bin/env python3
"""의존성 없는 STL 로더(바이너리/ASCII) + 높이 단면 윤곽 도우미."""
import struct, numpy as np

def load_stl(path):
    b = open(path, "rb").read()
    if b[:5] == b"solid" and b"facet" in b[:1000]:
        v = []
        for line in b.decode(errors="ignore").splitlines():
            t = line.strip().split()
            if t and t[0] == "vertex":
                v.append([float(t[1]), float(t[2]), float(t[3])])
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)

def slice_z(tris, z):
    """평면 z 와 삼각형의 교선 세그먼트 목록 [(x1,y1,x2,y2)...]"""
    segs = []
    for t in tris:
        zs = t[:, 2]
        if zs.min() > z or zs.max() < z:
            continue
        pts = []
        for i in range(3):
            p, q = t[i], t[(i + 1) % 3]
            if (p[2] - z) * (q[2] - z) < 0:
                f = (z - p[2]) / (q[2] - p[2]); pts.append(p[:2] + (q[:2] - p[:2]) * f)
        if len(pts) == 2:
            segs.append((pts[0][0], pts[0][1], pts[1][0], pts[1][1]))
    return segs
