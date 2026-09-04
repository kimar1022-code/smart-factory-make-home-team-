#!/usr/bin/env bash
# 새 USB 손목카메라(:8768) 기동 — 9/1
#   장치는 시리얼(200901010001)로 자동 탐색(USB 재삽입으로 번호 밀려도 무관)
#   9/1 오후: 카메라를 손목→측면 거치로 이설 — rot 0(사용자 지정), 그리퍼 제외구역 해제.
#   CAM_EXCLUDE = 그리퍼 몸체(기판 초록·부품 빨강 오검출) 고정 제외 구역
#   CAM_HSV_GREEN = 이 카메라 실측(스티커 H81 V120 / 기판 V53) 맞춤 임계
set -u
cd "$(dirname "$0")"
P=$(ps -eo pid,args | awk '/[c]am_server.py --source v4l2/ {print $1}')
[ -n "$P" ] && kill -TERM $P && sleep 2
CAM_DET_HZ=${CAM_DET_HZ:-8} \
CAM_EXCLUDE=${CAM_EXCLUDE:-""} \
CAM_HSV_GREEN=${CAM_HSV_GREEN:-"48,80,70,88,255,255"} \
setsid -f python3 cam_server.py --source v4l2 --serial 200901010001 --rot 0 --fps 15 --capres 1280x720 --port 8768 > /tmp/newcam2.log 2>&1
sleep 6
echo "━━ 새카메라 http://$(hostname -I | awk '{print $1}'):8768  로그 /tmp/newcam2.log"
curl -s -m 3 http://127.0.0.1:8766/health >/dev/null 2>&1   # (무관, 존재 확인용 아님)
curl -s -m 3 http://127.0.0.1:8768/health; echo
