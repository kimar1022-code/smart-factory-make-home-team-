---
name: session-0905-night-hover-align
description: "9/5 밤(로봇 OFF) — 사용자 지적 '설계한 보정 3개가 하나도 안 들어간 채 꽂고 있었다' 확인·원인 5가지 자백 → 호버 정렬(hover_align.py) 신설·오프라인 실증(사용자 nudge 3건과 부호·크기 일치) + place_calc run 배선(기준 없으면 하강 금지) + 뎁스 축척·랙 뎁스 길이 검산(미검증). 내일 첫 행동 = 파랑 z440 정렬 확인 후 hover_align ref blue"
metadata:
  node_type: memory
  type: project
  originSessionId: 154f9f7e-7e83-4958-992f-5bb8ff6624f6
---

# 9/5 밤 — "왜 설계한 보정이 안 들어가고 그냥 꽂으려 했나" + 호버 정렬 배선

## 0. 사용자 지적(사진 3장): 오늘 말한 보정 3개가 run 경로에 없었다
코드로 확인한 실제 상태(`place_calc.py run`, 9/5 22시 기준):
1. **기둥점↔벽점 같은 화면 비교 후 삽입** — 코드 어디에도 없음(grep 0건). z650 관측 1회 계산값으로 z478→z440→3mm 하강. 사용자 육안 nudge 가 정렬을 대신해 왔음.
2. **잡은 뒤 뎁스로 벽 길이 재서 x,y 검산** — `grip_measure.py`(9/4)가 있지만 run 이 안 부름. run 의 `grasp_measure` 는 손목캠 색점 2개 + **가상 축척 0.12**. 서명이 파랑만 점 2개 → 노랑·빨강·red_s 는 전부 "공칭 파지" 통과.
3. **픽 후 베이스 전체 보는 위치에서 정렬 마치고 z440** — 관측자세 재계산(plan holding=True)만 있고 정렬 루프 없음. `HELD_WALL_BOX` 마스크 미검증 상태로 배선.

## 1. 왜 그랬나 (자백 5가지 — 다음에 같은 짓 금지)
① 증상 패치(WALL_DELTA·RZ_BIAS 상수)를 설계 대신 넣었다 ② 두 갈래 구현(grip_measure vs grasp_measure) 만들고 약한 쪽을 배선 ③ "미검증" 이라 적은 걸 그대로 돌리고, 사용자 설계 정정(19:5x 양캠 기둥 기준)은 기록만 ④ "벽 들어가는 최단 경로" 를 목표로 삼아 육안 nudge 에 기댐 ⑤ 로봇 바쁠 때 코드 골격을 안 짰다.
→ **규칙: 사용자가 설계로 확정한 보정은 그 세션 안에 코드 골격까지 만들고, run 경로에 들어갔는지 grep 으로 증명한 뒤 "들어갔다" 고 말한다.**

## 2. 신설 `tools/hover_align.py` (오프라인 실증 완료, 실기 미투입)
- 원리: 기준 상태(사용자 "맞다" 한 z_seat+85, 벽 든 채)에서 [베이스 특징 P_ref(기둥 점 + 이미 안착된 벽 점 — 같은 높이 평면), 든 벽 점 W_ref(1~2개)] 저장. 매 사이클 P_ref→P_now 2D 유사변환 S → W_exp=S(W_ref) → Δ=W_now−W_exp → **δ_robot = R(rz−180)·Jinv_obs·(d_h/377)·Δ**, rz = ROT_SIGN·Δang.
- 든 벽 점 검출 = **기준 자리 ±80px 의 같은 색 점**(그리퍼에 물린 벽은 화면에서 거의 안 움직임) → 기둥·안착 벽 점과 면적이 겹쳐도 자리로 갈림.
- 점 1개 벽(노랑·red_s): 위치만, 회전은 −θ(베이스가 돈 만큼) 가정. 파지 회전은 랙 관측 각/뎁스 축 각으로 보완 예정.
- ★**오프라인 실증(사용자 nudge 프레임, 노랑 z441 rz 89.7)**: 예측 vs 실제 사용자 이동 — 1→3 (+2.12,−0.60) vs (+2,−1) · 2→3 (−0.10,+0.89) vs (0,+1) · 1→2 (+2.21,−1.49) vs (+2,−2). **rz 회전 매핑 없이는 90° 틀린 답**((+0.6,+2.1))이 나왔음 → `RZ_MAP=180` 기준 회전 필수(카메라가 손목에 붙어 있어서).
- 파랑 z440 프레임 3장: 기둥 2(좌상 파랑·우상 노랑)+벽점 2 전부 검출, 21:42 프레임 Δ(+0.57,−3.59)mm rz −1.77°(그 시각 "벽이 그리퍼에 3° 돌아 물림" 기록과 부합). 빈 손 프레임 → "든 벽 점 0개" 로 정확히 거부.
- **red_s 는 z440(rz 83)에서 베이스가 화면에 전혀 없음**(랙만 보임) → 정렬 자세 별도 티칭 필요.
- 게이트: 특징 매칭 ≥2 · 벽점 ≥1 · 축척 |s−1|≤3% · rms≤6px · 5회 내 수렴(0.3mm/0.15°) · **발산 시 즉시 정지**(ROT_SIGN=−1 미검증 보호). 스텝 ≤3mm/0.5°, 1% 속도, TCP 폴링 도달.
- CLI: `ref <색>` / `check <색>` / `align <색> [--dry]` / `test <색> ref.jpg now.jpg [z] [--rz r] [--save]`. 기준 파일 `hover_ref_0905.json`.

## 3. `place_calc.py` 배선 (백업 `place_calc.py.0905_2300_prealign_bak`)
- run(...,align=True,len_gate=False): z+85 도달 후 **hover_align.align → 정렬된 현재 TCP(x,y,rz) 로 descend_monitored**(옛 P 로 내리면 정렬을 되돌리므로 반드시 현재 TCP). **기준 없으면 예외=하강 금지**, `--no-align` 만 우회.
- 파지 축척: `held_wall_depth()` 뎁스 d_w → mm/px = d_w/fx(fx=377/0.4135≈912) 로 0.12 가상값 대체(+벽 축 각 PCA). ★**든 벽 전체는 손목캠 프레임에 안 들어옴**(파랑 z440: 198mm≈1650px>720px) → 든 채 길이 측정은 원리적으로 불가, 뎁스·축 각만.
- 랙 관측자세(벽 전체 보임)에서 `rack_len_check`: 색점 양끝 축 주변 뎁스 띠 → 물리 길이·중심(색점과 독립). **기본 보고만, `--len-gate` 면 정지.** 합성 검증: 198→197.9mm, 비대칭 색점 −4.9mm 어긋남을 −4.5mm 로 검출. `rack_ends` 반환에 p1/p2 추가.
- CLI 추가: `held_depth <색>`, `rack_len <색>`. ⚠ 뎁스 계열은 **실기 미검증** — 내일 첫 픽에서 수치 확인.

## 4. 내일 첫 행동(로봇 켠 뒤, 순서)
1. 데몬 재기동(코드 수정분 반영) → `fr5_up.sh` → 파랑 `run blue`(호버 z478 정지) → z+85 로 내려 사용자 육안 정렬 → **`hover_align.py ref blue`** → `hover_align.py check blue` 로 Δ≈0 확인 → 로봇 1mm/1° 조그 후 check 로 **부호 검증(ROT_SIGN)** → `run blue --seat`.
2. 노랑·빨강 같은 절차로 ref. red_s 는 둘 다 보이는 정렬 자세 먼저 찾기(z478~520 또는 XY 이동).
3. `held_depth`·`rack_len` 수치 확인 → 맞으면 `--len-gate` 상시화.
[[session-0905-base-pose-color-pillars]] · [[rule-read-yesterday-memory-first]] · [[rule-base-moves-aruco-board-fixed]]
