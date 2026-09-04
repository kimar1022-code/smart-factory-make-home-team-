#!/bin/bash
# 카메라 자동 기동·복구 감시 데몬 — 9/4.
#  ① D435 뎁스캠(:8766, rs): USB(8086:0b07) 꽂혀 있고 8766 꺼지면 start_cam.sh rs + 밝기 100.
#  ② 새카메라(:8768, v4l2 Realtek/Generic): 서버 죽거나 '얼어붙음'(프레임 무변화) 감지 시 by-id 경로로 재기동.
# USB 재열거로 video 인덱스가 바뀌어도 by-id 안정 경로로 다시 붙는다. 3초 폴링.
#   nohup bash cam_autostart.sh > logs/cam_autostart.log 2>&1 &
cd /home/ar/bf2_console
LOG(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
NEWCAM_BYID="/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
LOG "cam_autostart 시작 — D435(8766)+새카메라(8768) 감시(3s)"
D435_UP=0
frozen_count=0
prev_hash=""

start_newcam(){
  local dev=$(readlink -f "$NEWCAM_BYID" 2>/dev/null)
  [ -z "$dev" ] && { LOG "새카메라 by-id 없음(USB 빠짐?)"; return 1; }
  pkill -f "cam_server.py --source v4l2 .*--port 8768" 2>/dev/null; sleep 1
  nohup python3 cam_server.py --source v4l2 --dev "$dev" --rot 0 --fps 15 --capres 1280x720 --port 8768 \
    > /tmp/cam_server2.log 2>&1 &
  for i in $(seq 1 12); do ss -tlnp 2>/dev/null | grep -q ':8768 ' && break; sleep 1; done
  ss -tlnp 2>/dev/null | grep -q ':8768 ' && LOG "새카메라 8768 재기동 ($dev)" || LOG "⚠ 8768 재기동 실패"
}

while true; do
  # ── ① D435 ──
  USB=$(lsusb 2>/dev/null | grep -c "8086:0b07")
  UP=$(ss -tlnp 2>/dev/null | grep -c ':8766 ')
  if [ "$USB" -ge 1 ] && [ "$UP" -eq 0 ]; then
    LOG "D435 감지 + 8766 꺼짐 → start_cam.sh rs"
    ./start_cam.sh rs >> logs/cam_autostart_start.log 2>&1
    for i in $(seq 1 20); do ss -tlnp 2>/dev/null | grep -q ':8766 ' && break; sleep 1; done
    if ss -tlnp 2>/dev/null | grep -q ':8766 '; then
      sleep 2; curl -s -m25 "http://127.0.0.1:8766/expo?bright=100" >/dev/null 2>&1
      LOG "8766 기동 완료·밝기 정규화"; D435_UP=1
    fi
  elif [ "$USB" -eq 0 ] && [ "$D435_UP" -eq 1 ]; then
    LOG "D435 USB 빠짐 — 대기"; D435_UP=0
  fi

  # ── ② 새카메라 8768: 죽음 or 얼어붙음 ──
  NUP=$(ss -tlnp 2>/dev/null | grep -c ':8768 ')
  NUSB=$(lsusb 2>/dev/null | grep -c "0bda:5844")
  if [ "$NUSB" -ge 1 ]; then
    if [ "$NUP" -eq 0 ]; then
      LOG "새카메라 USB 있는데 8768 죽음 → 재기동"; start_newcam; frozen_count=0; prev_hash=""
    else
      # 얼어붙음 감지: 프레임 해시가 3회 연속(약 9s) 같으면 재기동
      h=$(curl -s -m3 http://127.0.0.1:8768/raw 2>/dev/null | md5sum | awk '{print $1}')
      if [ -n "$h" ] && [ "$h" = "$prev_hash" ]; then
        frozen_count=$((frozen_count+1))
        if [ "$frozen_count" -ge 3 ]; then
          LOG "새카메라 8768 얼어붙음(3회 동일) → 재기동"; start_newcam; frozen_count=0; prev_hash=""
        fi
      else
        frozen_count=0
      fi
      prev_hash="$h"
    fi
  fi
  sleep 3
done
