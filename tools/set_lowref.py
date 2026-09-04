"""8/30 사용자 기준: 파지 높이에서 '벽 점이 사진의 같은 자리'에 오도록 하는 기준 픽셀을 잡는다.
   a = 1280 - px (오른쪽 끝까지), b = 720 - py (아래 끝까지) — 이 절대값이 매번 같아야 한다.
지금 승인된 파지 자세에서 z380 으로 올라가 그 높이의 기준 픽셀(low_ref)을 저장.
  python3 set_lowref.py <색> [z]
"""
import json, sys, time, urllib.request
import numpy as np
CK = sys.argv[1]; ZR = float(sys.argv[2]) if len(sys.argv) > 2 else 380.0
CAL = "/home/ar/bf2_console/dot_calib.json"
def post(a, b):
    r = urllib.request.Request(f'http://localhost:8765/fr5/{a}', data=json.dumps(b).encode(),
                               headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(r, timeout=30).read())
def stt(): return json.loads(urllib.request.urlopen('http://localhost:8765/status', timeout=3).read())['robots']['fr5']
def settle(t=120):
    time.sleep(1.2)
    for _ in range(int(t/0.4)):
        if not stt()['busy']: break
        time.sleep(0.4)
    last = None; same = 0
    for _ in range(60):
        time.sleep(0.25); v = [round(q, 2) for q in stt()['tcp']]
        same = same + 1 if (last is not None and v == last) else 0; last = v
        if same >= 3: return v
    return last
def dot(n=14):
    acc = []
    for _ in range(n):
        for p in json.load(urllib.request.urlopen('http://localhost:8766/dots', timeout=4))['dots']:
            if p['kind'] == CK and p['px'] > 600: acc.append((p['px'], p['py'], p['area']))
        time.sleep(0.08)
    if len(acc) < n*0.4: return None, len(acc)
    return (float(np.median([a[0] for a in acc])), float(np.median([a[1] for a in acc])),
            float(np.median([a[2] for a in acc]))), len(acc)
grasp = settle(); print('파지 자세', grasp)
post('speed', {'value': 2, 'dry_run': False}); time.sleep(0.3)
t = list(grasp); t[2] = ZR
post('move_tcp', {'tcp': t, 'dry_run': False}); here = settle()
print(f'z{ZR:.0f} 도착', here)
d, n = dot()
if d is None: raise SystemExit(f"{CK} 점 검출 실패({n}) — 화각/조명 확인")
px, py, area = d
print(f"\n★기준 픽셀 ({px:.1f}, {py:.1f})  면적 {area:.0f}  (샘플 {n})")
print(f"   a = 1280 - px = {1280-px:.1f}px      b = 720 - py = {720-py:.1f}px")
c = json.load(open(CAL)); r = c['refs'][CK]
r['low_ref'] = {"px": [round(px, 1), round(py, 1)], "pts": [[round(px, 1), round(py, 1)]],
                "theta_img": None, "img": f"/home/ar/bf2_console/refs/{CK}_z380_ref.jpg",
                "z": ZR, "n": n, "a_px": round(1280-px, 1), "b_px": round(720-py, 1),
                "made": time.strftime("%Y-%m-%d %H:%M") + f" 사용자 승인 파지자세에서 z{ZR:.0f} 캡처(a/b 불변 기준)"}
r['grasp_tcp_approved'] = grasp
json.dump(c, open(CAL, 'w'), ensure_ascii=False, indent=1)
urllib.request.urlretrieve('http://127.0.0.1:8766/raw', f"/home/ar/bf2_console/refs/{CK}_z380_ref.jpg")
print("저장 완료 (low_ref + grasp_tcp_approved + 사진)")
t = list(here); t[2] = grasp[2]
post('move_tcp', {'tcp': t, 'dry_run': False}); settle()
print('파지 자세 복귀', [round(v, 2) for v in stt()['tcp']])
