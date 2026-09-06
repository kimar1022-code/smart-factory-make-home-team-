# 9/6(일) 검증 런북 — 로봇 켠 뒤 순서대로 (코드 수정 없이 검증만)

전제: 데몬 전부 살아 있음(8766/8768/8771/8772/8773/8774/**8775 호버뷰**), 노출 417, 상태 파일 백업 `~/bf2_console/backup/state_0906_am/`.
뷰: 콘솔 손목캠 패널 :8773 · 랙 :8774 · **호버 정렬 3캠 :8775** (색 바꾸기 `/?color=yellow`).
모든 명령: `cd ~/bf2_console/tools`

## 0. 기동 (로봇 전원 ON 후)
1. `~/fr5_up.sh` (알람이면 `~/fr5_rescue.sh`). 브리지 :8765/status 로 tcp 확인.
2. 아침 햇빛 — 밝기 238 로 어제보다 밝음. 관측자세에서 `python3 color_lock.py check` → 4점 안 나오면 run 의 건강게이트가 노출을 알아서 내림(250/167 예상).
3. 벽 4개 랙에 정위치(점 3개 이상 위에서 보이게). 베이스는 그대로 둠.

## 1. 파랑 — 부호 검증 + 첫 사이클  (정오 판단선의 기준)
1. `python3 place_calc.py run blue --grasp-teach`
   - ① 베이스 4점(안 보이면 탐색 이동) → ② 랙 재관측·중앙 파지 → ③ 들어올려 **파지 서명 저장**(게이트 없음) → 목표 → z478 호버 정지.
   - 확인: 출력의 "베이스(로봇)" / "랙: 벽 중앙 … 길이 ~568px" / "파지 서명 저장(2점) 간격 ≈426px". 간격이 크게 다르면 다른 점 쌍이 보이는 것 → 말해줄 것.
2. z+85(z440)로: `python3 place_calc.py nudge 0 0 0` 는 XY 이동이라 쓰지 말고, 콘솔 조그로 z 를 440 까지 내림(XY 그대로).
3. :8775 파랑 화면에서 손목캠에 **벽 점 2(흰)·기둥 점 2(초록)** 보이는지 확인. 안 보이면 `python3 hover_align.py detect blue` 로 수치 확인. (노출은 check 가 자동 사다리로 고름)
4. 사용자 육안으로 슬롯 위 정렬(콘솔 조그, XY·rz 소량). 맞았으면 `python3 hover_align.py ref blue`.
5. **부호 검증**: 콘솔 조그로 X +1mm → `python3 hover_align.py check blue` → 결합 XY 가 (−1, 0) 근처여야 함. 되돌리고 rz +1° → check → rz 가 −1 근처(ROT_SIGN −1 가정). **부호가 반대로 나오면 hover_align.py 의 ROT_SIGN["wrist"] 를 +1 로** (그것만 바꿈).
6. 되돌린 뒤 `python3 hover_align.py align blue` → 0.3mm/0.15° 수렴 확인 → 그대로 `python3 place_calc.py run blue --seat --no-pick` 은 ①을 든 채로 재측정하니 **쓰지 말고**, 대신 정렬 상태에서 콘솔로 하강하지 말고 → 벽을 사용자가 빼고 랙에 두고 **`run blue --seat`** 풀사이클.
7. 성공 시 자동으로 호버 기준 승격. `seat_verify` 로 안착 확인.

## 2. 노랑 / 빨강 긴벽
- 노랑: `run yellow` (서명 있음, 점 1개) → z440 → :8775 로 손목캠 특징 ≥2 + 노란 벽 점 1 확인 → 육안 정렬 → `ref yellow` → `run yellow --seat`.
  측면캠에 노란 벽 점이 보이면(뷰에서 흰 원) 추가: `hover_align.py probe side yellow --wall x,y`(벽 든 채 z440, ±10mm 자동 조그) → `ref yellow --src side --wall x,y`.
- 빨강: `run red --grasp-teach`(서명 없음) → 나머지 동일.

## 3. red_s (손목캠에 베이스 없음 → 고정캠)
1. `run red_s --grasp-teach` → z478 → z440 으로 내림.
2. :8775 에서 새카메라·측면캠에 **빨간 벽 점**(흰 원) 보이는지 확인. 좌표 읽기.
3. 새카메라: `hover_align.py probe newcam red_s --wall x,y` → 육안 정렬 → `ref red_s --src newcam --wall x,y`. 측면캠도 같은 식(`--src side`).
4. `run red_s --seat`. 정오까지 안 되면: 육안 정렬 + `place_calc.py run red_s --seat --no-align` 대신 **어제 방식(수동 정렬 후 막힘감시 하강)** 로 넘기고 나머지 셋 완성.

## 정지가 났을 때 읽는 법
- "베이스 기둥 4점을 4라운드 탐색에도 못 찾음" → 조명/베이스 위치. `color_lock.py check`.
- "랙 관측에서 벽 양끝을 끝내 못 잡음" → 벽이 뒤집혔거나 점 가림. 랙에 다시 놓기.
- "빈 파지 의심" → 그리퍼값=닫힘값이고 벽 점 0. 랙 위치 확인.
- "파지 편차 게이트 초과" → 1.0mm/0.3°. 너무 자주 나면 GRASP_GATE_MM 1.5 로(그것만).
- "호버 기준 없음" → 그 색 `ref` 안 한 것.
- "호버 정렬 발산" → 부호. 1단계 5 로 돌아가 확인.
- "카메라 불일치" → 고정캠 매핑(probe) 재실행.
- 막힘 → 25mm 후퇴 정지. 벽은 사용자가 뺀다.

## 정오 판단
파랑 정렬 루프가 수렴·삽입 성공 → 계속. 아니면 베이스 고정 + 새 마운트 골든 재티칭으로 전환(반나절). 저녁 백업 영상.
