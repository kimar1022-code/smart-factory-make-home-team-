"""8/30 사용자 기준 검사 — 각 점의 '사진 오른쪽 끝/아래 끝까지 거리'와 '벽↔밑판 거리(e)'를 기준과 비교.
   a,b,c,d = 각 점에서 화면 오른쪽 끝(1280-px)·아래 끝(720-py) 까지의 절대 거리
   e       = 벽 점 ↔ 밑판 점 거리(+각도). 팔 위치와 무관 → 이게 다르면 '파지가 다른 것'
  python3 abcde.py <색> [z기준]      기본: 현재 z 에 가장 가까운 hover_refs 기준
"""
import json, math, sys, time, urllib.request
import numpy as np
CK = sys.argv[1]
CAL = "/home/ar/bf2_console/dot_calib.json"
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status', timeout=3).read())['robots']['fr5']
def dots(n=10):
    acc = {}
    for _ in range(n):
        for p in json.load(urllib.request.urlopen('http://localhost:8766/dots', timeout=4))['dots']:
            acc.setdefault((p['kind'], round(p['px']/60), round(p['py']/60)), []).append((p['px'], p['py']))
        time.sleep(0.07)
    return [(k[0], float(np.median([q[0] for q in v])), float(np.median([q[1] for q in v])))
            for k, v in acc.items() if len(v) >= n*0.5]
cal = json.load(open(CAL)); ref = cal['refs_place'][CK]
z = float(sys.argv[2]) if len(sys.argv) > 2 else stt()['tcp'][2]
hr = min(ref['hover_refs'], key=lambda e: abs(e['z']-z))
cur = dots()
def near(kind, x, y, r=120):
    c = [d for d in cur if d[0] == kind and (d[1]-x)**2 + (d[2]-y)**2 < r*r]
    return min(c, key=lambda d: (d[1]-x)**2 + (d[2]-y)**2) if c else None
print(f"[{CK}] 현재 z {z:.1f} · 기준 z{hr['z']:.0f} ({ref.get('refs_made', '')})\n")
print(f"{'점':14s} {'기준px':>14s} {'현재px':>14s} {'→오른쪽끝':>20s} {'→아래끝':>18s}")
wall_r = wall_n = None; rig = []
for d in hr['dots']:
    m = near(d['kind'], d['px'], d['py'])
    tag = ('벽 ' if d.get('role') == 'wall' else '밑판') + ' ' + d['kind']
    if m is None:
        print(f"{tag:14s} {f'({d[chr(39)+chr(39)] if 0 else d[chr(112)+chr(120)]:.0f},{d[chr(112)+chr(121)]:.0f})':>14s}   미검출"); continue
    ra, rb = 1280-d['px'], 720-d['py']
    na, nb = 1280-m[1], 720-m[2]
    print(f"{tag:14s} ({d['px']:6.1f},{d['py']:6.1f}) ({m[1]:6.1f},{m[2]:6.1f})"
          f"   {ra:6.1f} → {na:6.1f} ({na-ra:+5.1f})   {rb:6.1f} → {nb:6.1f} ({nb-rb:+5.1f})")
    if d.get('role') == 'wall': wall_r, wall_n = (d['px'], d['py']), (m[1], m[2])
    else: rig.append(((d['px'], d['py']), (m[1], m[2]), d['kind']))
if wall_r and rig:
    print(f"\n★ e = 벽↔밑판 (팔 위치와 무관 — 다르면 '파지가 기준과 다르다'는 뜻)")
    for (rp, np_, k) in rig:
        dr = math.hypot(wall_r[0]-rp[0], wall_r[1]-rp[1]); an_r = math.degrees(math.atan2(wall_r[1]-rp[1], wall_r[0]-rp[0]))
        dn = math.hypot(wall_n[0]-np_[0], wall_n[1]-np_[1]); an_n = math.degrees(math.atan2(wall_n[1]-np_[1], wall_n[0]-np_[0]))
        print(f"   벽↔{k:6s} 거리 {dr:7.1f} → {dn:7.1f} ({dn-dr:+5.1f}px)   각 {an_r:7.2f}° → {an_n:7.2f}° ({an_n-an_r:+5.2f}°)")
    dw = (wall_n[0]-wall_r[0], wall_n[1]-wall_r[1])
    print(f"   벽 점 자체 편차 ({dw[0]:+.1f},{dw[1]:+.1f})px = 파지가 기준 대비 그만큼 다름")
elif not wall_r:
    print("\n⚠ 기준에 벽(role=wall) 점이 없음")
