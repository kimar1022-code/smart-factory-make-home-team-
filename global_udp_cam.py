#!/usr/bin/env python3
"""글로벌 카메라(비전 PC → HMV1/UDP) 수신 → MJPEG/JPEG 로 다시 내보내는 중계 (9/11).

    python3 global_udp_cam.py            # http://<로봇PC>:8779/  (/stream · /stream?w=640 · /snap · /health)

비전팀 사양(9/11): 송신 192.168.20.30 → 수신 192.168.20.10:21031, stream_id 3, 1280x960 JPEG q85 10fps,
헤더 32B + 페이로드 ≤1200B, ACK/재전송 없음.

★HMV1 헤더 32바이트 — 9/11 실측으로 풀어낸 배치(모두 big-endian):
    0..3   b"HMV1"          4  ver(1)        5  stream_id(3)     6..7  header_len(32)
    8..11  frame_id(u32)   12..15 (상수 416 — 용도 미상, 안 씀)   16..19 timestamp(u32, ms 로 보임)
    20..21 chunk_idx(u16)  22..23 chunk_count(u16)   24..27 frame_bytes(u32)   28..29 chunk_payload_len(u16)   30..31 0
  검증: chunk 0 페이로드가 ffd8ff(JPEG SOI), 150 청크×1200B, 합계 = frame_bytes, cv2 디코드 1280x960 OK.
  청크가 하나라도 빠지면 그 프레임은 버린다(재전송 없음) — /health 의 dropped 로 손실을 본다.
"""
import json
import os
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

UDP_PORT = int(os.environ.get("GCAM_UDP_PORT", "21031"))
HTTP_PORT = int(os.environ.get("GCAM_HTTP_PORT", "8779"))
STREAM_ID = 3
HDR = struct.Struct(">4sBBHIIIHHIHH")      # 32 bytes
BOOT = time.time()

latest = {"jpg": None, "fid": -1, "ts": 0, "t": 0.0}
stats = {"packets": 0, "complete": 0, "dropped": 0, "bad": 0, "src": None, "fps": 0.0}
lock = threading.Lock()
new_frame = threading.Condition(lock)


def rx_loop():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
    s.bind(("0.0.0.0", UDP_PORT))
    partial = {}                     # fid -> {idx: payload}
    meta = {}                        # fid -> (count, total)
    fps_t, fps_n = time.time(), 0
    while True:
        d, a = s.recvfrom(4096)
        stats["packets"] += 1
        if len(d) < 32 or d[:4] != b"HMV1":
            stats["bad"] += 1; continue
        magic, ver, sid, hlen, fid, _f12, ts, cidx, ccnt, total, plen, _r = HDR.unpack_from(d, 0)
        if sid != STREAM_ID or hlen < 32:
            stats["bad"] += 1; continue
        stats["src"] = a[0]
        payload = d[hlen:hlen + plen] if plen else d[hlen:]
        ch = partial.setdefault(fid, {})
        ch[cidx] = payload
        meta[fid] = (ccnt, total, ts)
        if len(ch) == ccnt:
            buf = b"".join(ch[i] for i in range(ccnt) if i in ch)
            if len(buf) == total and buf[:2] == b"\xff\xd8":
                with new_frame:
                    latest.update(jpg=buf, fid=fid, ts=ts, t=time.time())
                    new_frame.notify_all()
                stats["complete"] += 1
                fps_n += 1
            else:
                stats["bad"] += 1
            partial.pop(fid, None); meta.pop(fid, None)
            # 이 프레임보다 오래된 미완성 프레임은 버린다(청크 손실 = 재전송 없음)
            for old in [f for f in partial if f < fid]:
                partial.pop(old, None); meta.pop(old, None); stats["dropped"] += 1
        if len(partial) > 8:          # 안전장치: 고아 프레임 누적 방지
            for old in sorted(partial)[:-4]:
                partial.pop(old, None); meta.pop(old, None); stats["dropped"] += 1
        now = time.time()
        if now - fps_t >= 2.0:
            stats["fps"] = round(fps_n / (now - fps_t), 1); fps_t, fps_n = now, 0


def encode_scaled(jpg, w):
    """콘솔용 축소본(무선 부하 — 원본은 프레임당 ~180KB). w 가 없거나 원본 이상이면 그대로."""
    if not w:
        return jpg
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    if img is None or w >= img.shape[1]:
        return jpg
    h = int(round(img.shape[0] * w / img.shape[1]))
    ok, out = cv2.imencode(".jpg", cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 80])
    return out.tobytes() if ok else jpg


PAGE = """<!doctype html><meta charset=utf-8><title>글로벌캠</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px/1.4 system-ui}#h{padding:6px 10px;background:#222}
img{display:block;max-width:100vw;max-height:calc(100vh - 32px);margin:auto}</style>
<div id=h>글로벌캠 (비전 PC 192.168.20.30 → UDP 21031 → :%d) <span id=s></span></div>
<img id=v src="/stream">
<script>
const v=document.getElementById('v'),s=document.getElementById('s');
setInterval(async()=>{try{const j=await(await fetch('/health',{cache:'no-store'})).json();
 s.textContent=` — ${j.fps} fps · 프레임 ${j.fid} · 손실 ${j.dropped} · 나이 ${j.age==null?'-':j.age.toFixed(1)+'s'}`;
 if(j.age!=null&&j.age>5)v.src='/stream?'+Date.now();}catch(e){s.textContent=' — 서버 응답 없음'}},2000);
v.onerror=()=>setTimeout(()=>{v.src='/stream?'+Date.now()},1000);
</script>""" % HTTP_PORT


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype); self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Content-Length", str(len(body)))
        self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        path, _, q = self.path.partition("?")
        qs = dict(p.split("=", 1) for p in q.split("&") if "=" in p)
        if path == "/health":
            with lock:
                age = None if latest["t"] == 0 else round(time.time() - latest["t"], 2)
                body = json.dumps(dict(ok=latest["jpg"] is not None and age is not None and age < 3, boot=BOOT, fid=latest["fid"],
                                       age=age, **stats)).encode()
            return self._send(200, body, "application/json")
        if path == "/":
            return self._send(200, PAGE.encode(), "text/html; charset=utf-8")
        w = int(qs.get("w", "0") or 0)
        if path == "/snap":
            with lock:
                jpg = latest["jpg"]
            if jpg is None:
                return self._send(503, b"no frame", "text/plain")
            return self._send(200, encode_scaled(jpg, w), "image/jpeg")
        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store"); self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            last = -1
            try:
                while True:
                    with new_frame:
                        if latest["fid"] == last:
                            new_frame.wait(timeout=2.0)
                        if latest["fid"] == last or latest["jpg"] is None:
                            continue
                        jpg, last = latest["jpg"], latest["fid"]
                    out = encode_scaled(jpg, w)
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(out))
                    self.wfile.write(out); self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return
        return self._send(404, b"not found", "text/plain")


if __name__ == "__main__":
    threading.Thread(target=rx_loop, daemon=True).start()
    print(f"global_udp_cam: UDP :{UDP_PORT} (HMV1 stream {STREAM_ID}) → http://0.0.0.0:{HTTP_PORT}/  /stream /stream?w=640 /snap /health", flush=True)
    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), H).serve_forever()
