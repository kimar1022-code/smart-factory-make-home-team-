#!/bin/bash
# 4벽 연속 자동: 색마다 앵커 코드 자동 전환 → 성공하면 다음 색으로. 실패하면 그 색에서 멈춤(로봇 안전 정지).
# 각 색은 wait_rack 에서 사용자가 그 벽을 랙에 꽂을 때까지 기다린다.
# 사용: run_sequence.sh [color ...]   기본 = blue yellow red red_s
cd /home/ar/bf2_console/tools
DOTS=place_front_first.py.0903_2130_dotsbak
MARK=place_front_first.py.0903_2110_0902markerbak
COLORS="${*:-blue yellow red red_s}"
for ck in $COLORS; do
  if [ "$ck" = "blue" ]; then cp $DOTS place_front_first.py; A=dots; else cp $MARK place_front_first.py; A=marker; fi
  echo "======== $ck ($A 앵커) ========"
  L=/home/ar/bf2_console/logs/seq_${ck}.log
  python3 -u cycle_front_first.py 1 "$ck" > "$L" 2>&1
  if grep -qE "결과 1/1 성공" "$L"; then
    T=$(grep -oE "· [0-9]+초" "$L" | tail -1)
    echo "  OK $ck 성공 $T"
  else
    echo "  FAIL $ck 실패 — 시퀀스 중단"
    grep -vE "^[[:space:]]+File|^[[:space:]]+\^|^Traceback" "$L" | tail -8
    exit 1
  fi
done
echo "======== 전체 완료 ========"
