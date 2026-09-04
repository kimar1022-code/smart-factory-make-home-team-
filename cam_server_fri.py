#!/usr/bin/env python3
"""D435 손목캠 라이브 스트림 + ArUco 검출 오버레이 (8/25).

    python3 cam_server.py            # http://<PC IP>:8766 브라우저로 접속 (USB D435 직접)
⚠ RealSense 파이프는 프로세스 하나만 열 수 있다 — aruco_check.py 등과 동시 실행 금지.

8/27 영상 소스 선택 (D435 케이블을 비전 담당이 가져갔을 때 — 네트워크로 받기):
    --source rs                      # 기본. pyrealsense2 로 USB 직접 (컬러+정렬 뎁스)
    --source ros [--color-topic auto|T] [--depth-topic off|auto|T] [--domain 73]
                                     # ROS2 이미지 구독. 기본 auto = 토픽 목록에서 color/rgb 이미지 자동 선택
                                     #   (compressed 우선 → WiFi 대역 1/10), 뎁스는 기본 off(캘리브 땐 --depth-topic auto).
                                     #   15s 프레임 없으면 자동 재탐색. 비전 PC 가 realsense-ros 든 cam_pub.py 든 무관.
                                     #   ★ROS env(cyclonedds_team.xml) source 한 셸에서 — 또는 ./start_cam.sh 사용.
    --source udp [--udp-port 5005]   # ★순수 UDP JPEG 조각 수신 — 카메라 PC 에서 cam_udp_send.py --to <이 PC IP>.
                                     #   ROS/DDS/도메인 설정 전혀 불필요. 뎁스는 송신 --depth 일 때만(png16).
    --source mjpeg --url http://...  # MJPEG/RTSP 등 cv2.VideoCapture 가 여는 URL (컬러만, 뎁스 없음 → depth_mm=None)
    --port 8766
검출·/dots·/stream·/focus·/click 은 소스와 무관하게 동일. 프레임은 항상 1280x720 으로 맞춘다(픽셀 기준 좌표 보존).
"""
import argparse
import json
import os
import threading
import time

import cv2
import numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

DICT_NAMES = ["DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_5X5_50",
              "DICT_5X5_100", "DICT_5X5_250", "DICT_6X6_50", "DICT_6X6_250",
              "DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_36h11"]

latest = {"jpg": None, "info": "starting"}
# 8/30: 물고 있는 벽의 기울기를 뎁스 평면 피팅으로 직접 재기 위한 격자 샘플 요청함.
#   rs 프레임 객체를 핸들러로 넘기면 프레임 풀 수명 문제가 생기므로, 요청만 걸어두고
#   다음 프레임에서 루프가 채운다(요청 1개, 응답 1개).
depth_req = {"box": None, "res": None}
# 8/30: 뎁스 화면 보기(/depthview) — 카메라 위치를 새로 잡을 때 뎁스가 유효한지 눈으로 확인용.
depth_img = {"want": False, "jpg": None, "t": 0.0, "info": ""}
lock = threading.Lock()
# 사용자가 라이브 화면에서 클릭한 타깃(8/25 요청: "내가 점 찍을 순 없어?").
# 설정되면 검출은 이 주변(반경 SEED_R)에서 '가장 가까운' 블롭만 채택 + 추적 갱신.
seed = {"xy": None}
SEED_R = 200


def make_detector(name):
    # cv2 4.6 = 구 API(Dictionary_get/DetectorParameters_create), 4.7+ = ArucoDetector.
    # ⚠혼용 금지: 4.6에서 DetectorParameters() 직접 생성 후 사용 → segfault(8/25 실증)
    if hasattr(cv2.aruco, "ArucoDetector"):
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)
    d = cv2.aruco.Dictionary_get(getattr(cv2.aruco, name))
    p = cv2.aruco.DetectorParameters_create()
    return lambda img: cv2.aruco.detectMarkers(img, d, parameters=p)


MAX_PER_KIND = 6   # 색깔당 최대 표시 개수
# 8/26 작업별 색 포커스: 서버 JobStep 이 정한 부품(색)만 검출 — /focus?kinds=blue,red  /focus?clear=1
focus = {"kinds": None}
BOOT = int(time.time())   # 서버 기동 시각 — 콘솔 워치독이 재시작을 감지해 스트림 재접속


def detect_dots(img):
    """파지점 도트 검출 — 검은 부품 위 '흰 점' + 흰 부품 위 '파란 점'.

    반환: [(kind, (cx,cy), area), ...] 면적 상위 각 1개.
    원리: ①흰 점 = 어두운 주변(검은 부품) 속 밝은 소형 블롭
          ②파란 점 = HSV 파랑 범위 소형 블롭
    """
    out = []
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # ── 파란 점 (흰 부품 위) ──
    # 8/26: 색깔당 1개(면적 최대)만 반환하던 결함 → 전부 반환(면적순, 색당 최대 MAX_PER_KIND)
    #       "빨간 점 하나가 인식 안 됨" = 두 번째 빨간 점이 면적 경쟁에서 밀려 버려진 것.
    def _all(mask, kind, lo, hi):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        got = []
        for c in cnts:
            a = cv2.contourArea(c)
            if not (lo < a < hi):
                continue
            # 8/26 모양 필터(파랑 오탐 대응): 스티커는 원형 — 실측 원형도 0.55~0.78·채움 0.5~0.6.
            #   그림자/모서리 반사는 길쭉(원형도 0.2~0.4). 문턱 원형도 0.45·채움 0.4.
            x, y, w, h = cv2.boundingRect(c)
            circ = 4 * np.pi * a / (cv2.arcLength(c, True) ** 2 + 1e-6)
            if circ < 0.45 or a / float(w * h) < 0.4 or max(w, h) > 2.2 * min(w, h):
                continue
            m = cv2.moments(c)
            got.append((kind, (int(m["m10"]/m["m00"]), int(m["m01"]/m["m00"])), a))
        got.sort(key=lambda t: -t[2])
        return got[:MAX_PER_KIND]

    # 파랑: 8/26 채도 하한 80→110 (오탐 = 어두운 남색 그림자 S88·V78 / 실제 스티커 S168~224)
    out += _all(cv2.inRange(hsv, (95, 110, 80), (130, 255, 255)), "blue", 30, 5000)

    # ── 빨간 점 (8/25 사용자가 빨강으로 변경 — 채도 높아 흑백 부품 모두에서 유리) ──
    mask_r = cv2.inRange(hsv, (0, 100, 70), (10, 255, 255)) | \
             cv2.inRange(hsv, (168, 100, 70), (180, 255, 255))
    out += _all(mask_r, "red", 15, 5000)

    # ── 노란 점 (8/26 추가): HSV 18~35, 실측 노란 점 면적 ≈500px @1280x720 ──
    out += _all(cv2.inRange(hsv, (18, 90, 90), (35, 255, 255)), "yellow", 15, 5000)

    # ── 초록 점 (8/26 추가): 실측 펜 끝 HSV(83,193,74) — 청록 쪽·어두워서 V 하한 45, 파랑(95~)과 미겹침 ──
    # 8/26 저녁: 어두운 청록 그림자(색상 80~92·명도<70) 오탐 → 상한 85·채도 130·명도 75 로 조임
    # 8/27 오후: 조명이 바뀌어 가운데·아래 점이 흐려짐(실측 아래 H82 S108 V63, 면적 12) → V 62→52 · 면적 20→12.
    #   12프레임 검증: 현재값은 대부분 1점만, V52+area12 는 대부분 3점 + 벽 영역 밖 오탐 0.
    #   ★색상 상한 85 는 유지(어두운 청록 오탐 차단 핵심). V35 까지 낮추면 오탐 다발(8/27 실증) — 내리지 말 것.
    #   ★면적 하한 12 는 오검출(면적 14 블롭) 유발 — 20 유지. V52 만으로 아래 점 면적 159 확보(8/27 실측).
    out += _all(cv2.inRange(hsv, (48, 80, 52), (85, 255, 255)), "green", 20, 5000)

    # 흰 점 검출은 8/26 제거(사용자 요청: 랙에서 흰 부품 경계 오탐). 백업 backup/cam_server.py.0826_before_shape
    return out


# ─────────────────────────────────────────────────────────────────────
# 8/27 영상 소스 추상화: read() → (color_bgr 1280x720, depth) / depth 는 get_distance(x,y)[m] 을 가진 객체 또는 None
# ─────────────────────────────────────────────────────────────────────
W, HGT = 1280, 720
SRC = {"name": "rs", "last_ts": 0.0, "frames": 0}   # /health 용


def _fit(img):
    """어떤 소스든 1280x720 으로 — 도트 픽셀 좌표(캘리브 Minv·ROI)가 이 해상도 기준."""
    if img is None:
        return None
    if img.shape[1] != W or img.shape[0] != HGT:
        img = cv2.resize(img, (W, HGT), interpolation=cv2.INTER_AREA)
    return img


class NpDepth:
    """numpy 뎁스(정렬된 1280x720, mm 또는 m) 를 rs.depth_frame.get_distance 처럼 노출."""
    def __init__(self, arr, scale_to_m):
        self.a = arr
        self.k = scale_to_m
    def get_distance(self, x, y):
        v = float(self.a[int(y), int(x)])
        return v * self.k if v > 0 else 0.0


class RsSource:
    """USB D435 직접 (원래 방식)."""
    def __init__(self):
        import pyrealsense2 as rs      # 지연 import — ros/mjpeg 모드는 pyrealsense2 없어도 됨
        self.rs = rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        # 8/25: 뎁스 동시 스트림 + 컬러 정렬 — 도트까지 거리(mm)로 3축 보정("뎁스니까 z도 되잖아")
        cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)
        self.pipe = pipe
        pipe.start(cfg)

    def read(self):
        frames = self.align.process(self.pipe.wait_for_frames())
        return np.asanyarray(frames.get_color_frame().get_data()), frames.get_depth_frame()


class RosSource:
    """ROS2 토픽 구독 (비전 PC 가 카메라를 들고 있을 때). rclpy 스핀은 백그라운드 스레드."""
    STALE_REDISCOVER_S = 15.0     # 이만큼 프레임이 없으면 토픽 재탐색(비전 PC 가 재기동/이름 변경)

    def __init__(self, color_topic, depth_topic):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CompressedImage
        rclpy.init()
        self.node = Node("bf2_cam_server_sub")
        self.qos = qos_profile_sensor_data
        self.Image, self.CompressedImage = Image, CompressedImage
        self.cv = threading.Condition()
        self.color = None
        self.depth = None
        self.seq = 0
        self.subs = []
        self.want_color, self.want_depth = color_topic, depth_topic   # "auto" 가능
        self.color_topic = self.depth_topic = None
        self.compressed = False
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()
        self._subscribe()

    def _discover(self):
        """토픽 목록에서 컬러/뎁스 이미지 토픽을 고른다.
        컬러: 이름에 color|rgb 포함 + Image/CompressedImage. ★compressed 우선(WiFi 대역 1/10) → raw.
        뎁스: aligned_depth 포함 Image 만(정렬 안 된 raw depth 는 픽셀이 안 맞아 오히려 해로움)."""
        # ★퍼블리셔가 있는 토픽만 — 우리 옛 구독이 그래프에 남긴 이름을 다시 고르는 함정(8/27 재탐색 실패 진범)
        tt = {t: ty for t, ty in self.node.get_topic_names_and_types()
              if self.node.count_publishers(t) > 0}
        imgs = [t for t, ty in tt.items() if "sensor_msgs/msg/Image" in ty]
        comps = [t for t, ty in tt.items() if "sensor_msgs/msg/CompressedImage" in ty]
        def _is_color(t):
            l = t.lower()
            return ("color" in l or "rgb" in l) and "depth" not in l and "infra" not in l
        c_comp = sorted(t for t in comps if _is_color(t) and "compressedDepth" not in t)
        c_raw = sorted(t for t in imgs if _is_color(t))
        color = (c_comp or c_raw or [None])[0]
        depth = sorted(t for t in imgs if "aligned_depth" in t.lower())
        return color, (depth[0] if depth else None), (imgs + comps)

    def _subscribe(self):
        for sub in self.subs:
            self.node.destroy_subscription(sub)
        self.subs = []
        color, depth = self.want_color, self.want_depth
        if color == "auto" or depth == "auto":
            dc, dd, all_t = self._discover()
            if color == "auto":
                color = dc
            if depth == "auto":
                depth = dd
            if color is None:
                print(f"[ros] 컬러 이미지 토픽 없음 — 보이는 이미지 토픽: {all_t or '(없음)'}  (재탐색 대기)")
                return False
        self.color_topic, self.depth_topic = color, depth
        self.compressed = color.endswith("/compressed")
        ctype = self.CompressedImage if self.compressed else self.Image
        self.subs.append(self.node.create_subscription(ctype, color, self._on_color, self.qos))
        if depth:
            self.subs.append(self.node.create_subscription(self.Image, depth, self._on_depth, self.qos))
        print(f"[ros] color={color}{' (compressed)' if self.compressed else ''}  depth={depth or '-'}"
              f"  domain={os.environ.get('ROS_DOMAIN_ID','0')}")
        SRC["topic"] = color
        return True

    def _on_color(self, msg):
        try:
            if self.compressed:
                img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            else:
                img = self._img(msg)
        except Exception as e:      # 인코딩 미지원 등 — 죽지 말고 건너뜀
            self.node.get_logger().warn(f"color decode fail: {e}", throttle_duration_sec=5)
            return
        with self.cv:
            self.color = img
            self.seq += 1
            self.cv.notify_all()

    def _on_depth(self, msg):
        try:
            a = np.frombuffer(msg.data, np.uint16 if msg.encoding == "16UC1" else np.float32)
            a = a.reshape(msg.height, msg.width)
            k = 0.001 if msg.encoding == "16UC1" else 1.0
            if (msg.width, msg.height) != (W, HGT):
                a = cv2.resize(a, (W, HGT), interpolation=cv2.INTER_NEAREST)
            with self.cv:
                self.depth = NpDepth(a, k)
        except Exception as e:
            self.node.get_logger().warn(f"depth decode fail: {e}", throttle_duration_sec=5)

    @staticmethod
    def _img(msg):
        enc = msg.encoding
        if enc in ("bgr8", "rgb8"):
            a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if enc == "rgb8" else a.copy()
        if enc in ("bgra8", "rgba8"):
            a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 4)
            return cv2.cvtColor(a, cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR)
        if enc == "mono8":
            return cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width),
                                cv2.COLOR_GRAY2BGR)
        raise ValueError(f"encoding {enc}")

    def read(self):
        """새 컬러 프레임이 올 때까지 대기(최대 2s). 안 오면 None → 호출부가 '신호 없음' 표시.
        STALE_REDISCOVER_S 동안 프레임이 없으면(비전 PC 재기동·토픽명 변경) 토픽 재탐색."""
        with self.cv:
            s0 = self.seq
            self.cv.wait_for(lambda: self.seq != s0, timeout=2.0)
            if self.seq != s0:
                self._stale_since = None
                return self.color, self.depth
        now = time.time()
        if getattr(self, "_stale_since", None) is None:
            self._stale_since = now
        elif now - self._stale_since > self.STALE_REDISCOVER_S:
            self._stale_since = now
            print("[ros] 프레임 없음 → 토픽 재탐색")
            self._subscribe()
        return None, None


class UdpSource:
    """순수 UDP JPEG 조각 수신 (송신 = cam_udp_send.py, ROS/DDS 불필요). 헤더 '!4sIBHHI' 17B, 자세한 건 송신기 docstring."""
    HDR = None

    def __init__(self, port):
        import socket
        import struct
        self.HDR = struct.Struct("!4sIBHHI")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        self.sock.bind(("0.0.0.0", port))
        self.port = port
        self.cv = threading.Condition()
        self.color = None
        self.depth = None
        self.seq = 0
        self.peer = None
        self.stat = {"frames": 0, "dropped": 0, "pkts": 0, "bad_magic": 0}
        SRC["udp"] = self.stat
        threading.Thread(target=self._rx, daemon=True).start()
        print(f"[udp] listen 0.0.0.0:{port}")
        SRC["topic"] = f"udp:{port}"

    def _rx(self):
        bufs = {}          # (fid, kind) → {"n":cnt, "total":len, "parts":{idx:bytes}}
        last_fid = {0: None, 1: None}     # None = 아직 수신 전 → 어떤 id 든 수락
        last_rx = 0.0
        H = self.HDR
        while True:
            try:
                pkt, addr = self.sock.recvfrom(65535)
            except OSError:
                continue
            if len(pkt) < H.size:
                continue
            magic, fid, kind, idx, cnt, total = H.unpack_from(pkt)
            self.stat["pkts"] = self.stat.get("pkts", 0) + 1
            if magic != b"BF2C" or kind not in (0, 1):
                self.stat["bad_magic"] = self.stat.get("bad_magic", 0) + 1
                if self.stat["bad_magic"] <= 3:
                    print(f"[udp] 매직 불일치 from {addr[0]}: {pkt[:8]!r} len={len(pkt)}")
                continue
            now = time.time()
            if now - last_rx > 1.0:
                # 1s 이상 무신호 뒤 첫 패킷 = 송신기 재기동으로 간주 → frame_id 기준 리셋
                #   (8/27: 재기동한 송신기가 id 1 부터 다시 시작해 "과거 프레임" 필터에 전부 걸린 사고)
                last_fid = {0: None, 1: None}
                bufs.clear()
            last_rx = now
            if self.peer != addr[0]:
                self.peer = addr[0]
                SRC["topic"] = f"udp:{self.port} ← {addr[0]}"
                print(f"[udp] 송신자 {addr[0]}")
            # 과거 프레임 조각은 버림(uint32 랩어라운드 고려해 반차이로 비교)
            lf = last_fid[kind]
            if lf is not None and fid != lf and ((lf - fid) & 0xFFFFFFFF) < 0x7FFFFFFF:
                continue
            key = (fid, kind)
            b = bufs.get(key)
            if b is None:
                b = bufs[key] = {"n": cnt, "total": total, "parts": {}}
            b["parts"][idx] = pkt[H.size:]
            if len(b["parts"]) < b["n"]:
                continue
            data = b"".join(b["parts"][i] for i in range(b["n"]))
            del bufs[key]
            # 같은 kind 의 더 오래된 미완성 프레임 정리(손실분)
            for k in [k for k in bufs if k[1] == kind and k[0] != fid]:
                del bufs[k]; self.stat["dropped"] += 1
            last_fid[kind] = fid
            if len(data) != total or not data:
                continue
            arr = np.frombuffer(data, np.uint8)
            if arr.size == 0:      # 9/1: 빈 페이로드가 imdecode 어서션을 터뜨려 _rx 스레드가 통째로 죽었다
                continue           #      (그 뒤로는 소켓만 열린 채 아무도 안 읽는 '조용한 귀머거리' 상태)
            if kind == 0:
                try:
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                except Exception:
                    continue
                if img is None:
                    continue
                with self.cv:
                    self.color = img
                    self.seq += 1
                    self.stat["frames"] += 1
                    self.cv.notify_all()
            else:
                try:
                    dep = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
                except Exception:
                    continue
                if dep is None or dep.dtype != np.uint16:
                    continue
                if (dep.shape[1], dep.shape[0]) != (W, HGT):
                    dep = cv2.resize(dep, (W, HGT), interpolation=cv2.INTER_NEAREST)
                with self.cv:
                    self.depth = NpDepth(dep, 0.001)

    def read(self):
        with self.cv:
            s0 = self.seq
            self.cv.wait_for(lambda: self.seq != s0, timeout=2.0)
            if self.seq == s0:
                return None, None
            return self.color, self.depth


class MjpegSource:
    """cv2.VideoCapture 가 여는 URL(MJPEG http / RTSP). 컬러만."""
    def __init__(self, url):
        self.url = url
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise RuntimeError(f"open fail: {url}")

    def read(self):
        ok, img = self.cap.read()
        if not ok:
            self.cap.release(); time.sleep(1.0)
            self.cap = cv2.VideoCapture(self.url)      # 끊기면 재접속
            return None, None
        return img, None


def cam_loop(src):
    dets = {n: make_detector(n) for n in DICT_NAMES}
    locked = None            # 한 번 잡힌 딕셔너리로 고정(프레임률 확보)
    n_frame = 0
    blank = np.zeros((HGT, W, 3), np.uint8)
    while True:
        img, depth_f = src.read()
        if img is None:
            # 소스 끊김 — 검은 화면에 표시만 갱신(콘솔 워치독이 스트림은 살아있다고 보게)
            b = blank.copy()
            cv2.putText(b, f"NO SIGNAL ({SRC['name']}) {time.time()-SRC['last_ts']:.0f}s",
                        (300, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            ok, jpg = cv2.imencode(".jpg", b, [cv2.IMWRITE_JPEG_QUALITY, 60])
            with lock:
                latest["jpg"] = jpg.tobytes(); latest["info"] = "no signal"; latest["dots"] = []
            continue
        img = _fit(img)
        SRC["last_ts"] = time.time(); SRC["frames"] += 1
        raw = img.copy()      # ⚠검출은 원본에서 — 오버레이(파란 id 글씨)를 도트로 오검출한 사고(8/25)
        n_frame += 1
        info = "no marker"
        try_names = [locked] if locked else (DICT_NAMES if n_frame % 3 == 0 else [])
        for name in try_names:
            corners, ids, _ = dets[name](img)
            if ids is not None and len(ids):
                locked = name
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
                pxs = [f"id{int(i)}:{np.linalg.norm(c[0][0]-c[0][1]):.0f}px"
                       for i, c in zip(ids.flatten(), corners)]
                info = f"{name}  " + " ".join(pxs)
                break
        else:
            if locked and try_names:
                locked = None    # 놓치면 다시 전수 스캔
        # ── 파지점 도트 검출 (8/25 사용자 방식: 검은 부품=흰 점, 흰 부품=파란 점) ──
        dots = [d for d in detect_dots(raw)
                if 20 < d[1][0] < 1260 and 20 < d[1][1] < 700]   # 가장자리 오검출 제거
        fk = focus["kinds"]
        if fk:
            dots = [d for d in dots if d[0] in fk]
        sxy = seed["xy"]
        if sxy:
            near = [d for d in dots
                    if (d[1][0]-sxy[0])**2 + (d[1][1]-sxy[1])**2 < SEED_R**2]
            near.sort(key=lambda d: (d[1][0]-sxy[0])**2 + (d[1][1]-sxy[1])**2)
            dots = near[:1]
            if dots:
                seed["xy"] = list(dots[0][1])   # 추적: 찾은 위치로 시드 갱신
            cv2.circle(img, tuple(map(int, sxy)), SEED_R, (255, 0, 255), 1)
            cv2.drawMarker(img, tuple(map(int, sxy)), (255, 0, 255),
                           cv2.MARKER_TILTED_CROSS, 24, 2)
        for kind, (dx, dy), area in dots:
            color = {"white": (255, 255, 255), "blue": (255, 128, 0),
                     "red": (0, 0, 255), "yellow": (0, 200, 255),
                     "green": (0, 255, 0)}.get(kind, (0, 255, 0))
            cv2.drawMarker(img, (dx, dy), color, cv2.MARKER_CROSS, 30, 2)
            cv2.circle(img, (dx, dy), 12, color, 2)
            cv2.putText(img, kind, (dx + 14, dy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 2)
        if dots:
            info += "  |  " + " ".join(f"{k}({x},{y})" for k, (x, y), _ in dots)
        cv2.drawMarker(img, (640, 360), (0, 0, 255), cv2.MARKER_CROSS, 20, 1)  # 화면 중앙
        if fk:
            cv2.putText(img, "FOCUS " + "+".join(sorted(fk)), (1000, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        cv2.putText(img, info, (12, 705), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 255), 2)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with lock:
                latest["jpg"] = jpg.tobytes(); latest["raw"] = raw.copy()   # 8/28 /raw·/hsv 용
                latest["info"] = info
                def _dmm(x, y, r=2):
                    """(x,y) 주변 (2r+1) 창 뎁스 중앙값(mm) — 0(무효)은 제외."""
                    if depth_f is None:
                        return None
                    try:
                        vs = [depth_f.get_distance(min(1279, max(0, x+dx)),
                                                   min(719, max(0, y+dy))) * 1000.0
                              for dx in (-r, 0, r) for dy in (-r, 0, r)]
                        vs = sorted(v for v in vs if v > 1)
                        return round(vs[len(vs)//2], 1) if vs else None
                    except Exception:
                        return None
                if depth_img.get("want"):                # 8/30 /depthview
                    try:
                        if depth_f is None:
                            depth_img["info"] = "뎁스 없음(소스에 뎁스 스트림이 없음)"
                            depth_img["jpg"] = None
                        else:
                            raw16 = np.asanyarray(depth_f.get_data())
                            v = raw16.astype(np.float32)
                            valid = v > 0
                            if valid.sum() < 100:
                                depth_img["info"] = "유효 뎁스 픽셀 거의 없음 — 너무 가깝거나(<175mm) 반사"
                            else:
                                lo = float(np.percentile(v[valid], 5)); hi = float(np.percentile(v[valid], 95))
                                depth_img["info"] = (f"유효 {100*valid.mean():.0f}% · 5~95% 구간 "
                                                     f"{lo*0.001*1000:.0f}~{hi*0.001*1000:.0f}(raw)")
                            hi2 = max(hi if valid.sum() >= 100 else 1.0, 1.0)
                            n = np.clip((v / hi2) * 255.0, 0, 255).astype(np.uint8)
                            col = cv2.applyColorMap(n, cv2.COLORMAP_JET)
                            col[~valid] = (0, 0, 0)          # 무효는 검정
                            if col.shape[:2] != img.shape[:2]:
                                col = cv2.resize(col, (img.shape[1], img.shape[0]))
                            ok2, j2 = cv2.imencode(".jpg", col, [cv2.IMWRITE_JPEG_QUALITY, 75])
                            depth_img["jpg"] = j2.tobytes() if ok2 else None
                        depth_img["t"] = time.time()
                    except Exception as e:
                        depth_img["info"] = f"뎁스 변환 실패: {e}"; depth_img["jpg"] = None
                    depth_img["want"] = False
                if depth_req.get("box") is not None:      # 8/30 /depthgrid
                    _x0, _y0, _x1, _y1, _nx, _ny, _r = depth_req["box"]
                    _g = []
                    for _j in range(_ny):
                        for _i in range(_nx):
                            _x = int(round(_x0 + (_x1-_x0) * (_i/(_nx-1) if _nx > 1 else 0)))
                            _y = int(round(_y0 + (_y1-_y0) * (_j/(_ny-1) if _ny > 1 else 0)))
                            _g.append([_x, _y, _dmm(_x, _y, _r)])
                    depth_req["res"] = _g
                    depth_req["box"] = None
                latest["dots"] = [{"kind": k, "px": x, "py": y, "area": a,
                                   "depth_mm": _dmm(x, y)}
                                  for k, (x, y), a in dots]


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):  # 요청 로그 소음 제거
        pass

    def do_GET(self):
        if self.path.startswith("/raw"):
            # 8/28: 오버레이 없는 원본 프레임(JPEG q95) — 색 임계 실측용
            with lock:
                rw = latest.get("raw")
            if rw is None:
                self.send_response(503); self.end_headers(); return
            ok, j = cv2.imencode(".jpg", rw, [cv2.IMWRITE_JPEG_QUALITY, 95])
            self.send_response(200); self.send_header("Content-Type", "image/jpeg"); self.end_headers()
            self.wfile.write(j.tobytes()); return
        if self.path.startswith("/hsv"):
            # 8/28: /hsv?x=&y=&r=5 → 원본 프레임 (x,y) 주변 (2r+1)^2 창의 HSV 중앙값 + BGR
            q = parse_qs(urlparse(self.path).query)
            with lock:
                rw = latest.get("raw")
            try:
                x = int(float(q["x"][0])); y = int(float(q["y"][0])); r = int(q.get("r", ["5"])[0])
            except (KeyError, ValueError):
                self.send_response(400); self.end_headers(); return
            if rw is None:
                self.send_response(503); self.end_headers(); return
            win = rw[max(0,y-r):y+r+1, max(0,x-r):x+r+1]
            hv = cv2.cvtColor(win, cv2.COLOR_BGR2HSV).reshape(-1, 3)
            bg = win.reshape(-1, 3)
            out = {"x": x, "y": y, "hsv": [int(v) for v in np.median(hv, axis=0)],
                   "bgr": [int(v) for v in np.median(bg, axis=0)]}
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
            self.wfile.write(json.dumps(out).encode()); return
        if self.path.startswith("/depthview"):
            html = (b"<body style='margin:0;background:#111;color:#ddd;font-family:sans-serif'>"
                    b"<div style='padding:6px'>\xec\x86\x90\xeb\xaa\xa9\xec\xba\xa0 \xeb\x8e\x81\xec\x8a\xa4 "
                    b"(\xea\xb2\x80\xec\xa0\x95=\xeb\xac\xb4\xed\x9a\xa8 \xc2\xb7 \xed\x8c\x8c\xeb\x9e\x91=\xea\xb0\x80\xea\xb9\x8c\xec\x9b\x80 \xc2\xb7 "
                    b"\xeb\xb9\xa8\xea\xb0\x95=\xeb\xa9\x80\xec\x9d\x8c)</div>"
                    b"<img id=d style='width:49vw'><img id=c style='width:49vw'>"
                    b"<div id=t style='padding:6px;font-size:13px'></div><script>"
                    b"function f(){document.getElementById('d').src='/depthimg?'+Date.now();"
                    b"document.getElementById('c').src='/raw?'+Date.now();"
                    b"fetch('/depthinfo').then(r=>r.text()).then(t=>document.getElementById('t').textContent=t);"
                    b"setTimeout(f,400);}f();</script></body>")
            self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html))); self.end_headers()
            self.wfile.write(html); return
        if self.path.startswith("/depthinfo"):
            with lock: depth_img["want"] = True
            t0 = time.time()
            while time.time() - t0 < 3.0 and depth_img["t"] < t0: time.sleep(0.02)
            b = depth_img.get("info", "").encode()
            self.send_response(200); self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(b))); self.end_headers()
            self.wfile.write(b); return
        if self.path.startswith("/depthimg"):
            with lock: depth_img["want"] = True
            t0 = time.time()
            while time.time() - t0 < 3.0 and depth_img["t"] < t0: time.sleep(0.02)
            j = depth_img.get("jpg")
            if not j:
                self.send_response(503); self.end_headers(); return
            self.send_response(200); self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store"); self.end_headers()
            self.wfile.write(j); return
        if self.path.startswith("/depthgrid"):
            # 8/30: /depthgrid?x0=&y0=&x1=&y1=&nx=&ny=&r= → 격자 점의 뎁스(mm) 목록.
            #   물고 있는 벽면에 격자를 깔고 평면 피팅 → 파지 기울기를 직접 계측.
            q = parse_qs(urlparse(self.path).query)
            try:
                box = (int(float(q["x0"][0])), int(float(q["y0"][0])),
                       int(float(q["x1"][0])), int(float(q["y1"][0])),
                       int(q.get("nx", ["8"])[0]), int(q.get("ny", ["8"])[0]),
                       int(q.get("r", ["2"])[0]))
            except (KeyError, ValueError):
                self.send_response(400); self.end_headers(); return
            with lock:
                depth_req["res"] = None; depth_req["box"] = box
            t0 = time.time(); res = None
            while time.time() - t0 < 4.0:
                with lock:
                    res = depth_req.get("res")
                if res is not None: break
                time.sleep(0.02)
            if res is None:
                self.send_response(503); self.end_headers(); return
            out = {"pts": [{"x": a, "y": b, "d": c} for a, b, c in res]}
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
            self.wfile.write(json.dumps(out).encode()); return
        if self.path.startswith("/click"):
            # 라이브 화면 클릭 → 타깃 시드 설정 (x,y = 1280x720 좌표) / clear=1 해제
            q = parse_qs(urlparse(self.path).query)
            if q.get("clear"):
                seed["xy"] = None
            else:
                try:
                    seed["xy"] = [float(q["x"][0]), float(q["y"][0])]
                except (KeyError, ValueError):
                    pass
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(str(seed["xy"]).encode())
            return
        if self.path.startswith("/focus"):
            q = parse_qs(urlparse(self.path).query)
            if "clear" in q or not q.get("kinds", [""])[0]:
                focus["kinds"] = None
            else:
                focus["kinds"] = set(k.strip() for k in q["kinds"][0].split(",") if k.strip())
            self.send_response(200); self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Type", "text/plain"); self.end_headers()
            self.wfile.write(str(sorted(focus["kinds"]) if focus["kinds"] else None).encode())
            return
        if self.path.startswith("/dots"):
            # 도트 검출 결과 JSON (캘리브·dot_align 용, 8/25)
            import json as _j
            with lock:
                body = _j.dumps({"boot": BOOT, "source": SRC["name"],
                                 "focus": sorted(focus["kinds"]) if focus["kinds"] else None,
                                 "dots": latest.get("dots", []),
                                 "info": latest.get("info", "")}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/health"):
            import json as _j
            age = time.time() - SRC["last_ts"] if SRC["last_ts"] else None
            body = _j.dumps({"boot": BOOT, "source": SRC["name"], "frames": SRC["frames"],
                             "topic": SRC.get("topic"), "udp": SRC.get("udp"),
                             "age": round(age, 2) if age is not None else None,
                             "ok": bool(age is not None and age < 3.0)}).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*"); self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with lock:
                        jpg = latest["jpg"]
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.05)   # ~20fps
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write("""<title>FR5 손목캠</title>
<body style="margin:0;background:#111;color:#eee;font-family:sans-serif">
<div style="padding:8px 12px">FR5 손목 D435 — <b>점을 클릭하면 그 점만 추적</b>(보라 원 = 추적 범위) · 더블클릭 = 해제</div>
<img id="v" src="/stream" style="width:100%;max-width:1280px;display:block;cursor:crosshair">
<script>
const v=document.getElementById('v');
v.onclick=e=>{const r=v.getBoundingClientRect();
  const x=(e.clientX-r.left)*1280/r.width, y=(e.clientY-r.top)*720/r.height;
  fetch(`/click?x=${x.toFixed(0)}&y=${y.toFixed(0)}`)};
v.ondblclick=()=>fetch('/click?clear=1');
</script>
</body>""".encode())


if __name__ == "__main__":
    import sys
    sys.stdout.reconfigure(line_buffering=True)     # 로그 파일로 보낼 때 [ros] 줄이 바로 보이게
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["rs", "ros", "udp", "mjpeg"], default=os.environ.get("CAM_SOURCE", "rs"))
    ap.add_argument("--udp-port", type=int, default=int(os.environ.get("CAM_UDP_PORT", "5005")),
                    help="udp: cam_udp_send.py 가 보내는 포트")
    ap.add_argument("--color-topic", default=os.environ.get("CAM_COLOR_TOPIC", "auto"),
                    help="ros: 토픽명 또는 auto(토픽 목록에서 color/rgb 이미지 자동 선택, compressed 우선)")
    ap.add_argument("--depth-topic", default=os.environ.get("CAM_DEPTH_TOPIC", "off"),
                    help="ros: 토픽명 | auto(aligned_depth 자동) | off(기본 — WiFi 대역 보호, 캘리브 땐 auto)")
    ap.add_argument("--domain", default=os.environ.get("ROS_DOMAIN_ID"),
                    help="ros: ROS_DOMAIN_ID (비전 PC 와 같아야 함, 계약=73)")
    ap.add_argument("--url", default=os.environ.get("CAM_URL", ""))
    ap.add_argument("--port", type=int, default=int(os.environ.get("CAM_PORT", "8766")))
    a = ap.parse_args()
    SRC["name"] = a.source
    if a.source == "rs":
        src = RsSource()
    elif a.source == "ros":
        if a.domain:
            os.environ["ROS_DOMAIN_ID"] = str(a.domain)
        dt = None if a.depth_topic in ("off", "", "none") else a.depth_topic
        src = RosSource(a.color_topic, dt)
    elif a.source == "udp":
        src = UdpSource(a.udp_port)
    else:
        if not a.url:
            ap.error("--source mjpeg 는 --url 필요")
        src = MjpegSource(a.url)
    # ⚠RealSense 파이프는 메인 스레드에서 돌린다 — 서브스레드에서 segfault(8/25 실증)
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"cam_server[{a.source}] → http://0.0.0.0:{a.port}  (스트림: /stream, 상태: /health)")
    cam_loop(src)
