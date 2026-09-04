#!/usr/bin/env bash
# HARMONY 로봇 콘솔 기동 — 정적서버(:8000) + 브리지(:8765)
#   ./start_console.sh        # REAL (FR5 스택은 별도로 ~/fr5_up.sh)
#   MOCK=1 ./start_console.sh # mock 브리지
set -u
cd "$(dirname "$0")"
fuser -k 8000/tcp 8765/tcp 2>/dev/null; sleep 1
nohup python3 -m http.server 8000 >/dev/null 2>&1 &
if [ "${MOCK:-0}" = "1" ]; then
  nohup python3 bridge_server.py > /tmp/bf2_bridge.log 2>&1 &
else
  nohup bash -c 'export ROS_DOMAIN_ID=73 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/cyclonedds_team.xml BF2_REAL=1
    source /opt/ros/jazzy/setup.bash; source $HOME/fr5_jazzy_test_ws/install/setup.bash
    exec python3 bridge_server.py' > /tmp/bf2_bridge.log 2>&1 &
fi
# ★9/4: D435 뎁스캠 자동 기동 감시 데몬 — USB 꽂히면 8766 자동 기동(중복 방지)
if ! pgrep -f "cam_autostart.s[h]" >/dev/null 2>&1; then
  nohup bash "$HOME/bf2_console/tools/cam_autostart.sh" > "$HOME/bf2_console/logs/cam_autostart.log" 2>&1 &
  echo "  cam_autostart 데몬 기동(D435 꽂히면 8766 자동)"
fi
sleep 3
echo "━━ HARMONY 콘솔 기동 ━━"
for ip in $(ip -4 addr show | awk '/inet / && !/127.0.0.1/ {sub(/\/.*/,"",$2); print $2}'); do
  echo "  http://$ip:8000/bf2_robot_console.html"
done
echo "  (같은 네트워크 아무 PC 브라우저에서 접속 — 브리지는 자동 연결)"
echo "  브리지 로그: /tmp/bf2_bridge.log"
