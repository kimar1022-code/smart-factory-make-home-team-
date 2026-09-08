# /cell/control v0.3 — 회신 및 확정안 (2026-09-09)

보내주신 5건 전부 동의합니다. 특히 **`held_at` 의미 지적은 저희 설계 결함이 맞습니다** — 그대로 반영했습니다.

---

## 1. `held_at` 의미 정리 — 지적하신 대로 고쳤습니다

### 무엇이 잘못됐었나

저희 `CellControl.srv` 주석에는 원래부터 이렇게 적혀 있었습니다.

```
# 응답은 즉시 ACK — 완료 아님. 완료는 /cell/status 의 cell_state 로 확인.
```

그런데 v0.3 초안에서 저희가 얹은 `held_at`("실제로 멈춘 지점")이 **그 원칙을 스스로 깨고** 있었습니다.
`DEFERRED_UNSAFE` 구간에서는 ACK 시점에 어디서 멈출지 알 수 없으니, 응답에 실을 수 없는 값입니다.
말씀해주신 기준이 맞습니다.

### 확정 원칙

| | 무엇을 말하는가 | 어디서 읽는가 |
|---|---|---|
| **Service ACK** | 정지 요청 **수락 여부**와 **정지 방식** | `/cell/control` 응답 |
| **정지 완료 (authority)** | 실제로 멈췄는가 | **`/cell/status` 의 `cell_state == "HELD"`** |
| **실제 멈춘 지점** | 어디서 멈췄는가 | `/cell/status` 의 `hold.held_at` (HELD 일 때만) |

`held_at` 을 **서비스 응답에서 빼고 `/cell/status` 로 옮겼습니다.**

### 개정된 `CellControl.srv`

```
string ver        # "0.3"
string cmd        # PAUSE | RESUME | ABORT | RESET
string req_id
bool   immediate  # true = 현재 모션까지 즉시 감속 정지(안전 구간이면)
                  # false/미지정 = 현재 phase 완료 후 HELD  ← 기본값(v0.2 동작, 하위 호환)
---
bool   accepted
string cell_state # ACK 시점 상태 — 아직 HELD 가 아닐 수 있다
string detail
string stop_mode  # ★"어떻게 세울 것인가"에 대한 약속이지 결과가 아니다
                  #   IMMEDIATE | AT_PHASE_BOUNDARY | DEFERRED_UNSAFE | NOT_APPLICABLE
string eta_ms     # HELD 도달까지 최악 예상 지연(ms) — UX 문구 선택용
```

- `held_at` **삭제** → `stop_mode` 로 대체. 이름부터 "결과"가 아니라 "방식"임이 드러나게 바꿨습니다.
- `eta_ms` 를 추가했습니다. 음성 안내에서 **"지금 멈췄습니다" / "동작 마무리 후 멈춥니다(약 N초)"** 를
  ACK 하나로 바로 고를 수 있게 하려는 목적입니다.
  값은 `IMMEDIATE ≈ 0` / 삽입 스텝 경계 `≈ 1000` / **그리퍼 개폐 구간 `≈ 11000`** 입니다.

### `/cell/status` 확장 (JSON, 1Hz)

`cell_state == "HELD"` 일 때만 `hold` 블록이 실립니다. 그 외에는 `null` 입니다.

```json
{
  "ver": "0.3",
  "cell_state": "HELD",
  "hold": {
    "req_id": "…",            // 어느 PAUSE 요청으로 멈췄는지
    "stop_mode": "DEFERRED_UNSAFE",   // ACK 때 약속한 방식
    "held_at": "STEP_BOUNDARY",       // ★실제로 멈춘 지점
    "phase": "INSERT",                // 멈춘 시점의 phase
    "since": "2026-09-09T…Z",
    "resumable": true                 // false = 오류로 인한 HELD (RESET 필요)
  },
  "active_task": { "req_id": "…", "job_id": "…", "phase": "INSERT", … }
}
```

`held_at` 값 (실제 지점만, 방식은 안 들어갑니다):

```
PHASE_BOUNDARY | STEP_BOUNDARY | HOVER | OBSERVE | MID_MOTION
```

`active_task` 가 `null` 이 아닌 채로 `HELD` 라는 것 자체가 **ExecuteTask 가 살아 있다**는 표시입니다(아래 3번).

---

## 2. 음성 PAUSE → `immediate=true`

동의합니다. 다만 UX 문구를 위해 두 가지만 확인 부탁드립니다.

- `immediate=true` 라도 **❌ 구간에서는 `DEFERRED_UNSAFE`** 로 ACK 되고, 최악 **11초**(그리퍼 풀스트로크) 지연됩니다.
  이건 안전 문제가 아니라 **하드웨어 제약**입니다 — 그리퍼 동작을 중간에 끊으면 ServoJ 오류 14 → errno73 연쇄로
  **전원 재투입 말고는 복구가 안 되는 알람**이 납니다(8/14 실기).
- 그래서 `eta_ms` 를 보고 안내 문구를 고르시는 것을 권합니다.
  `eta_ms <= 1000` 이면 "멈췄습니다", 그보다 크면 "동작을 마무리하고 멈춥니다".

---

## 3. v0.3 검증 항목 추가 — 요청하신 것 그대로 넣었습니다

> FR5 immediate pause 시 내부 MoveGroup Goal 의 cancel/stop 이 `/cell/execute_task` 자체의
> CANCELED/FAILED 로 전파되지 않고, ExecuteTask 는 살아 있는 상태로 HELD 를 유지할 것

**타당한 지적이고, 실제로 저희가 틀리기 쉬운 지점입니다.** 저희 코드에서 MoveGroup 골 취소는
`stop()` 과 `ABORT` 가 공유하는 경로라, 구분해서 처리하지 않으면 PAUSE 가 Task 를 죽일 수 있습니다.

구현 방침을 이렇게 정했습니다.

- 취소 사유를 **`PAUSE` / `ABORT` / `ERROR` 로 태그**해서 내려보내고, `PAUSE` 태그로 인한 골 취소는
  Task 종료 판정에서 **제외**합니다. Task 는 `HELD` 루프에 들어갈 뿐 `_finish()` 를 타지 않습니다.
- `ABORT` 는 종전대로 `status="CANCELED"`, 장비·비전 오류는 `"FAILED"` — **v0.2 와 동일**합니다.

### v0.3 검증 항목 (도메인 99 격리 모의 → 실기)

| # | 항목 | 통과 기준 |
|---|---|---|
| V1 | `immediate=true` PAUSE 를 MOVE 구간에서 | ACK `stop_mode=IMMEDIATE`, 1초 내 `cell_state=HELD`, `held_at=MID_MOTION` |
| V2 | **ExecuteTask 생존** (★요청 항목) | PAUSE 전후로 골 핸들 동일, `status` 가 CANCELED/FAILED 로 **바뀌지 않음**, `active_task` 유지 |
| V3 | `immediate=true` 를 그리퍼 개폐 중에 | ACK `stop_mode=DEFERRED_UNSAFE`, `eta_ms≈11000`, 완료 후 `held_at=PHASE_BOUNDARY` |
| V4 | `immediate=true` 를 삽입 하강 중에 | `held_at=STEP_BOUNDARY`, 지연 ≤1초, **벽이 그리퍼에서 밀리지 않을 것** |
| V5 | RESUME | 같은 골 유지, 중단된 phase **재시작**, 완료 시 `status=SUCCEEDED` |
| V5b | **벽을 든 채 하강 중간에서 RESUME** | 정렬 높이로 수직 복귀 → **재측정** → 하강. 그 자리에서 이어 내려가지 않을 것 |
| V6 | ACK ↔ status 정합 | 모든 케이스에서 ACK `stop_mode` 와 이후 `hold.held_at` 이 §5 표대로 대응 |
| V7 | 흡착 타임아웃 | 설정값 초과 시 `FAULT`, RESUME 불가(RESET 필요) |
| V8 | RESUME 전 Vision 재검증 | 부품 이상 시 `FAULT` + 이벤트 발행 |
| V9 | 하위 호환 | `ver="0.2"` 요청(=`immediate` 없음) 이 v0.2 와 동일하게 동작 |

---

## 4. RESUME — 기존 ExecuteTask 유지 + 중단된 phase 재시작

동의하며, **이미 그렇게 구현되어 있습니다.** 재발주 불필요합니다.

phase 를 "이어붙이기"가 아니라 **"다시 시작"** 하는 이유는 저희 phase 가 전부 **절대 목표**이기 때문입니다 —
매 phase 가 비전으로 새로 측정하고 목표 TCP 를 다시 계산합니다. 중간 상태를 복원하는 것보다
다시 측정해서 다시 가는 쪽이 안전하고 결과도 같습니다. 이미 물고 있으면 파지를 유지한 채 그 phase 만 재실행합니다.

### 4-1. 벽을 든 채로 멈췄다 재개하는 경우 — **되올라가 다시 측정합니다**

Pause 가 **삽입 하강 중간**에 걸린 경우(벽이 그리퍼에 들려 채널 안에 반쯤 들어간 상태)는
멈춘 그 자리에서 이어 내려가지 않습니다.

```
RESUME → ①벽을 물고 있는지 확인 → ②정렬 높이(z_seat + 85, 내벽 102)까지 수직 복귀
       → ③비전 재측정·정렬 → ④하강
```

**이유**: 멈춰 있는 동안 밑판이 밀렸거나 벽이 그리퍼 안에서 조금 움직였을 수 있는데,
그 자리에서 이어 내려가면 그 변화를 **못 본 채로** 밀어 넣게 됩니다.
저희 phase 는 전부 절대 목표(측정 → 목표 계산 → 이동)라 되올라가 다시 재는 쪽이 안전하고 결과도 같습니다.
§4 의 "phase 재시작" 원칙을 삽입 구간에 구체화한 것입니다.

수직 복귀라 벽이 들어온 길을 그대로 되짚어 나옵니다(막힘 감시가 정지 시 쓰는 +25mm 후퇴와 같은 동작).
재측정에서 이상이 나오면 하강하지 않고 사용자 확인 대기로 갑니다.

> 구현 상태: `house_cycle` 에 `resume_pause` 경로로 **구현 완료**(콘솔 버튼 ⏯).
> 오케스트레이터의 RESUME 이 이 경로를 호출하도록 배선하는 것은 v0.3 구현 항목입니다. 검증은 V5 에 포함합니다.

---

## 5. 흡착 Pause 타임아웃 — configurable, 60초는 임시값

동의합니다. ROS2 파라미터로 뺐습니다.

```
suction_hold_timeout_sec   기본 60.0   (0 = 무제한, 권장하지 않음)
```

- 런타임에 `ros2 param set` 으로 조정 가능하고, 실기 장시간 흡착 검증 후 최종값을 확정하겠습니다.
- 초과 시 `FAULT` → RESUME 불가, RESET 후 Task 재발주입니다.
- 근거를 한 번 더 적어둡니다: **진공에는 피드백이 없습니다.** 계약 §6 의 E201 을 저희가 검출할 수 없어서,
  Pause 가 길어지면 부품이 조용히 떨어져도 아무도 모릅니다. 타임아웃은 그 공백을 시간으로 막는 임시 방편입니다.
- 근본 해결은 **진공 압력 스위치 추가**입니다. 하드웨어 추가가 가능한 시점에 다시 제안드리겠습니다.

RESUME 전 Vision 재검증 → 이상 시 `FAULT` — 동의하며 V8 로 검증합니다.

---

## 6. 정리 — 합의된 v0.3

| 항목 | 확정 |
|---|---|
| 음성 PAUSE | `immediate=true` |
| 정지 완료 authority | **`/cell/status` 의 `cell_state="HELD"`** |
| Service ACK 의미 | 수락 여부 + **정지 방식**(`stop_mode`) + 최악 지연(`eta_ms`) |
| 실제 멈춘 지점 | `/cell/status` 의 `hold.held_at` (HELD 일 때만) |
| RESUME | 기존 ExecuteTask 유지 + 중단된 phase 재시작 |
| RESUME (벽 든 채) | **정렬 높이 복귀 → 비전 재측정 → 하강** (그 자리에서 이어가지 않음) |
| 흡착 타임아웃 | `suction_hold_timeout_sec` 파라미터, 기본 60초(임시) |
| 하위 호환 | `immediate` 미지정 = v0.2 동작 |

### 진행 상태

- `CellControl.srv` **v0.3 개정 완료** (저장소 반영).
- `/cell/status` 의 `hold` 블록, `stop_mode`/`eta_ms` 산출, PAUSE 태그 골 취소는 **구현 예정**입니다.
- 순서: 구현 → **도메인 99 격리 모의**(V1~V9) → 실기. 실기 스택과 같은 도메인에서는 시험하지 않습니다(8/11 사고 이후 철칙).

### 저희가 확인 부탁드리는 것 하나

`eta_ms` 를 `string` 으로 뒀습니다(다른 필드와 타입을 맞추느라). **`int32` 로 바꾸는 편이 쓰시기 편하면 말씀해주세요** —
빌드 전이라 지금이 바꾸기 가장 쉬운 시점입니다.
