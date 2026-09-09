---
name: session-0909-reds-refs-fork-sign-lift
description: "9/9 — 오전 추가분 전량 되돌림(9/8 저녁 판) 뒤 red_s 기준 3종 재취득·2점 배열 크래시 수정·ArUco 정렬 특징 → 다섯 벽 전부 삽입, 포크 리프트 완주 + 파란 점 보정 부호 버그 실증·홈 자세 정정·리프트 버튼 신설. 다음 세션 = A타입 티칭(인수인계 문서 있음)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0ce0dc1a-a84c-4421-b0bb-7188aa5f80b4
  modified: 2026-09-09T09:09:13.639Z
---

# 9/9 (화) — red_s 종결·리프트 부호·인수인계

**다음 세션 1순위: `~/smart-factory-repo/docs/인수인계_0909_다음세션_A타입착수.md` 전문 읽기** (철칙 12·시스템 지도·버튼 절차·오늘 변경·미해결 11·A타입 계획). 사용자: "너하고 똑같이 할 수 있도록", "다음 세션 가면 A 티칭부터".

## 낮: 되돌리기
- 오전에 넣은 것(PROMOTE_OFF·house_type 분리·cam_idle_watch·resume_pause·OVERSHOOT_NUDGE·rz-follow·MIN_EXEC 0.5) 이 연쇄 실패를 불러 사용자 "9월8일 저녁7시 이후껄로 다되돌려" → `af15259` 로 바이트 일치 복원. 그 판은 `git show ccd610f:tools/house_cycle.py` 에 남아 있음(cam_idle_watch 재도입 시 참고).
- 로봇 동결(15:28) 진범 = D435 USB 탈락 → 기둥 0/4 → 탐색 이동 반복. 복구 = 전원 + `ros2_control_node` 좀비 종료 + `nmcli con up fr5-wired` + `fr5_up.sh`.

## 저녁: red_s
- 거부(축척 1.094) 원인 = 기준이 z437(정렬은 436)·expo 83(카메라 250)·특징 14개가 전부 **다른 벽 점**. 사용자 "거부된거 그대로 두면 안 되지, 1·2 둘 다 넣어".
- ② `hover_align.ALIGN_ARUCO={"red_s"}` — ArUco 중심(ar35·ar37)을 정렬 특징에 추가. ① 슬롯 1/2 → z436 상승 → 기준(wrist·side) → 안착 복귀 → 개방 +30 → 슬롯 2/2 를 **4분 안에 같은 조명**에서. 벽 점이 처음으로 **2개** 잡힘.
- ★사이클에서 **크래시** `operands could not be broadcast (3,2) (2,2)`: 기준 벽 점 2·측정 3(조각) → `w_now - w_exp`. red_s 가 1점이던 동안 `one_dot` 분기로 숨어 있던 결함. 해법 `WALL_PAIR_MAX_PX=90` 1:1 최근접 짝짓기(벽 점 간격 192px 의 절반 미만이라 교차 짝 불가), 넘으면 거부. 실기 "3개 중 2개 짝, 5px" → 삽입 ✅ 17:39.
- 서버 재기동 후 `align_here` 로 그 자리에서 이어감(벽 한 번도 안 놓음).
- red_in: 첫 파지 +2.38mm(랙 끝 치우침·재관측) → 정렬 5.5mm 이동 → **4mm 게이트 정지(정상)**. 사용자 재파지 → +0.92 → 1.3mm → 삽입 ✅ 17:51. **정렬은 파지 오차를 2배 증폭한다(시차)** — 게이트 완화 말고 재파지.
- red_s 2점 rz 가 1분 내 −0.28/+3.10/+2.47° → 아직 못 믿음(면적 896 점이 조각 의심). rz 고정 유지.

## 리프트
- 완주 1회(출하지 −700,−100, 그리퍼 50 유지). 그러나 **파란 점 보정이 반대로 감**(잔차 2배) → (99.5,−424.2) 치우친 파지로 완주. 부호 규칙은 [[rule-fork-dot-correction-sign]]. 새 부호 실기: 한 번에 0.1/0.3mm, 사용자 "이거 맞아".
- ★9/8 "1회 통과 ≤0.05mm" 는 Δ≈0 이라 **부호가 실행된 적 없던 것** — 성공 로그의 함정.
- 사용자 정정: 홈 ≠ 관측자세. `fork.json home_tcp`(102.75,−496.36,578.36)·관절(90,−90,90,−90,−90,0). 9/8 에 내가 멋대로 OBS 로 대체했었음.
- `house_cycle` 에 `stage_lift`/op `lift`/버튼 `🏠 리프트(출하)` 신설(게이트: 그리퍼<20 거부·z<400 거부). **실기 미검증.**
- 파란 점이 프레임 아래 가장자리(709/720) → 관측 자세 y 10mm 이동 후 ref_px 재촬영 권장.

## 내 실수 (다음 세션이 반복하지 말 것)
- 사이클 중 `measure_base()` 직접 호출 → 409. GOTO OBS 이동 중 서버 재기동 → 직후 409. 사이클 busy 중 손목캠 노출 변경(사다리와 충돌). **상태 확인 결과를 보고 나서** 로봇/카메라를 건드린다.
- `pgrep -f` 자기매칭으로 셸 4회 사망 → 킬은 파일 스크립트 + 대괄호 패턴.
- 성급한 "성공" 보고 2회 → 로그 `DONE`/`❌` 확인 후.

관련: [[session-0908-night-whitebalance-fork-lift]] [[session-0908-desk-moved-reteach-all]] [[rule-user-handles-part-returns]] [[rule-fork-dot-correction-sign]]
