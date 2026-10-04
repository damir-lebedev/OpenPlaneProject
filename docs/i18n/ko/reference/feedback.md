# AUTOPILOT / feedback — 피드백 루프 (기초 작업)

> 🌐 이 문서는 [러시아어 원문](../../../reference/feedback.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다.

[← 레퍼런스](README.md)

> ⚠️ **기초 작업 단계이며 펌웨어에 연결되어 있지 않습니다.** `FlightController`도,
> `Autopilot`도, `main.cpp`도 이 헤더들을 포함하지 않습니다. 검증은 폐루프 시뮬레이션
> (`test/test_feedback`, PC와 보드에서 모두 실행)과 네이티브 단위 테스트로 합니다.
> 연결 계획은 [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#연결-계획)에 있습니다.

기본 아이디어는 다음과 같습니다. 한 가지 속도에 맞춰 이득을 조정한 각도 PID 대신,
**기체의 반응**으로 폐루프를 구성한 레귤레이터를 사용합니다. 축 모델은 비행 중에
학습하고, 실속 보호와 센서로 구동되는 이륙·착륙 단계를 갖춥니다. 입력은
`FlightSnapshot` 하나뿐이고, 출력은 `FeedbackOutput` 하나뿐입니다.

모든 모듈은 헤더로만 이루어져 있으며, `FeedbackModules.h`가 한 줄로 모두 포함합니다.

---

## namespace `FeedbackConfig`

**파일:** `autopilot/feedback/FeedbackConfig.h`

루프의 모든 상수입니다 (연결할 때 `Config.h`로 옮겨집니다). “прикидка” (“대략적인 추정”)로 표시한
값은 무게 약 1 kg, 날개폭 1.2 m인 기체를 기준으로 합니다. `[AXIS_COUNT]` 배열은 축을 인덱스로 사용합니다.

| 그룹 | 상수 |
|---|---|
| 일반 | `GRAVITY = 9.80665`, 축 `AXIS_ROLL = 0`, `AXIS_PITCH = 1`, `AXIS_YAW = 2`, `AXIS_COUNT = 3` |
| 속도 | `STALL_SPEED_MS = 8`, `REFERENCE_SPEED_MS = 14`, `ACCEL_FILTER_TAU_S = 0.3` |
| 공중/지상 | `AIRBORNE_HEIGHT_M = 3`, `AIRBORNE_CONFIRM_MS = 500`, `GROUND_STILL_MS = 2000`, `GROUND_ACCEL_TOLERANCE_G = 0.1` |
| 레귤레이터 | `ANGLE_GAIN = {4, 4, 2}` 1/s, `MAX_RATE_DPS = {120, 60, 30}`, `RATE_TAU_S = {0.15, 0.20, 0.30}`, `RATE_INTEGRAL_GAIN = {2, 2, 1}`, `MAX_DEFLECTION_US = {400, 400, 400}`, `DAMPING_COMPENSATION = 0.5` |
| 조종면 효율 | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs, `EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`, `EFFECTIVENESS_MAX = {30, 15, 6}`, `RESPONSE_DELAY_MS = 40`, `RLS_FORGETTING = 0.995`, `ESTIMATOR_PERIOD_MS = 20`, `ESTIMATOR_PREFILTER_HZ = 2`, `MIN_EXCITATION_US = 30` |
| 실속 | `DECEL_WARN_MS2 = 2`, `DECEL_CONFIRM_MS = 300`, `LOW_ENERGY_PITCH_DEG = 5`, `NOSE_DROP_RATE_DPS = 60`, `WING_DROP_RATE_DPS = 120`, `STALL_NOSE_UP_COMMAND_US = 50`, `LOW_EFFECTIVENESS_RATIO = 0.35`, `LOW_SPEED_MARGIN = 1.25`, `LOW_SPEED_EXIT_MARGIN = 1.5`, `LOW_ENERGY_THROTTLE_PERCENT = 80`, `LOW_ENERGY_MAX_PITCH_DEG = 5`, `STALL_THROTTLE_PERCENT = 100`, `STALL_MAX_PITCH_DEG = −5`, `STALL_MAX_BANK_DEG = 10`, `STALL_AILERON_LIMIT_US = 150`, `RECOVERY_HOLD_MS = 1000` |
| 이륙 | `TAKEOFF_HAND_LAUNCH = false`, `TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`, `TAKEOFF_THROTTLE_PERCENT = 100`, `LAUNCH_ACCEL_G = 1`, `LAUNCH_DETECT_MS = 50`, `ROTATE_SPEED_MS = 10`, `ROTATE_FALLBACK_MS = 1500`, `CLIMB_PITCH_DEG = 12`, `TAKEOFF_TARGET_ALTITUDE_M = 30`, `TAKEOFF_CLIMB_FALLBACK_MS = 10000`, `LAUNCH_TIMEOUT_MS = 8000`, `HEADING_HOLD_GAIN = 2` |
| 착륙 | `APPROACH_SINK_RATE_MS = 1`, `APPROACH_THROTTLE_PERCENT = 25`, `APPROACH_BASE_PITCH_DEG = −3`, `APPROACH_MIN_PITCH_DEG = −10`, `APPROACH_MAX_BANK_DEG = 20`, `GO_AROUND_THROTTLE_PERCENT = 80`, `SINK_TO_PITCH_GAIN = 4`, `FLARE_HEIGHT_M = 2`, `FLARE_SINK_RATE_MS = 0.3`, `FLARE_MAX_PITCH_DEG = 8`, `TOUCHDOWN_ACCEL_G = 0.5`, `TOUCHDOWN_HEIGHT_M = 0.3`, `TOUCHDOWN_STILL_MS = 500`, `TOUCHDOWN_STILL_RATE_DPS = 5`, `ROLLOUT_MS = 5000` |

---

## namespace `FeedbackMath`

**파일:** `autopilot/feedback/FeedbackMath.h` · **의존:** `<math.h>`

| 함수 | 설명 |
|---|---|
| `float wrap180(float deg)` | 각도를 `(−180, 180]` 범위로 만듭니다. 방위 350°와 10°의 차이는 −20°입니다 |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | `[−limit, limit]`로 제한합니다 |

---

## `FlightSnapshot`

**파일:** `autopilot/feedback/FlightSnapshot.h` · **종류:** struct

한 주기 동안 루프가 기체에 대해 아는 모든 것입니다. 부호는 항공 관례를 따릅니다.

| 그룹 | 필드 |
|---|---|
| 시간/상태 | `timeUs`, `armed`, `linkLost` |
| 자세 | `imuValid`, `rollDeg`, `pitchDeg`, `yawDeg`, `rollRateDps`, `pitchRateDps`, `yawRateDps`, `accelXg/Yg/Zg` |
| 고도 | `baroValid`, `altitudeM` (전원을 켠 지점 기준), `climbRateMs`, `heightAglValid`, `heightAglM` (향후 거리계) |
| 속도 | `airspeedValid`, `airspeedMs` (향후 피토관), `gpsValid`, `groundSpeedMs` |
| 모드 목표 | `stabilizationActive` (false = MANUAL: 학습만), `targetRollDeg`, `targetPitchDeg` |
| 명령, µs | `stick*Us` — 조종사의 입력, `command*Us` — 실제로 조종면에 나가는 값 |
| 스로틀, % | `pilotThrottlePercent`, `throttlePercent` (실제로 ESC에 나가는 값) |
| 플랩 | `flapsUs`, `flapsMoving` |

---

## `FeedbackOutput`

**파일:** `autopilot/feedback/FeedbackOutput.h` · **종류:** struct

| 필드 | 설명 |
|---|---|
| `float deflectionUs[3]` | 축별 조종면 변위, µs (`ControlCommand`의 부호) |
| `bool axisEnabled[3]` | `false` — 해당 축은 제어하지 않으며 조종면은 조종사에게 남습니다 |
| `float throttleOverridePercent` | 비행 단계가 지정하는 절대 스로틀. `< 0` — 지정하지 않음 |
| `float throttleFloorPercent` | 스로틀의 하한 (실속 보호). `< 0` — 없음 |
| `targetRollDeg`, `targetPitchDeg` | 제한을 적용한 뒤의 최종 목표 (디버깅용) |
| `const char* reason` | 로그/OLED용 짧은 설명 |

---

## `PhaseTargets`

**파일:** `autopilot/feedback/PhaseTargets.h` · **종류:** struct

`TakeoffSequencer`와 `LandingSequencer`의 공통 출력입니다. “무엇을”이지 “어떻게”가 아닙니다.

| 필드 | 기본값 | 설명 |
|---|---|---|
| `active` | `false` | 해당 단계가 지금 기체를 조종하고 있음 |
| `targetRollDeg`, `targetPitchDeg` | 0 | 목표 |
| `controlRoll`, `controlPitch` | `true` | `false` — 해당 축을 건드리지 않음 (바퀴 위에서는 피치를 착륙장치가 정합니다) |
| `holdHeading`, `headingDeg` | `false`, 0 | 방향타와 조향 바퀴로 방위를 유지 |
| `throttlePercent` | −1 | −1 — 조종사의 스로틀 |
| `reason` | `""` | 설명 |

---

## `SpeedEstimator`

**파일:** `autopilot/feedback/SpeedEstimator.h`

속도(대기속도 > GPS 대지속도 > 알 수 없음)와 IMU로 구한 전후 방향 가속도를 다룹니다.
`dV/dt = g · (ax − sin θ)`를 `ACCEL_FILTER_TAU_S` 저역 통과 필터에 통과시키므로,
속도 센서가 없어도 “속도가 떨어지고 있음”을 알 수 있습니다.

| 메서드 | 설명 |
|---|---|
| `void update(const FlightSnapshot&)` | 한 단계를 진행합니다. 직전 호출 이후 `dt ≤ 0` 또는 `> 0.5 s`이면 (첫 호출에서는 `timeUs = 0` 이후 기준) 건너뜁니다 |
| `bool hasSpeed() const`, `float getSpeed() const`, `Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`, `float getAcceleration() const` | m/s², “+”는 가속. IMU가 없으면 `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`, `0.05..4`로 제한. 속도가 없으면 1 |

---

## `AirborneDetector`

**파일:** `autopilot/feedback/AirborneDetector.h`

기체가 공중에 있는지 판단합니다. 학습, 적분 누적, 실속 감시는 비행 중에만 의미가 있습니다.

| 메서드 | 설명 |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | 시동이 걸려 있지 않으면 “지상”으로 되돌립니다. 상태 변경 후보는 `AIRBORNE_CONFIRM_MS`(이륙) 또는 `GROUND_STILL_MS`(착륙) 동안 유지되어야 합니다 |
| `void force(bool)` | 명시적으로 설정합니다 (이륙과 착륙은 상태를 알고 있습니다) |
| `void reset()` | 지상 |
| `bool isAirborne() const` | |

“비행처럼 보임”은 거리계 또는 기압계 기준 높이가 `AIRBORNE_HEIGHT_M`보다 크거나,
속도가 `ROTATE_SPEED_MS`보다 큰 경우입니다. “지상처럼 보임”은 낮은 높이에서 모든 축의 각속도가
`TOUCHDOWN_STILL_RATE_DPS` 미만이고 |a| ≈ 1g (± `GROUND_ACCEL_TOLERANCE_G`)인 경우입니다.

---

## `ControlEffectivenessEstimator`

**파일:** `autopilot/feedback/ControlEffectivenessEstimator.h`

한 축의 추정기입니다. 모델은 **각가속도 = b·조종면(t − 지연) + a·ω + c**입니다.
`b`는 조종면 효율(µs당 °/s². 부호는 반응의 방향), `a`는 감쇠(1/s, 보통 < 0),
`c`는 일정한 모멘트(자동 트림)입니다. `b`는 기준 속도에서 학습하며
`b = b_ref · (V/V_ref)²`, `a = a_ref · V/V_ref`로 환산합니다. 추정에는 망각 계수가 있는
재귀 최소제곱법을 사용합니다 (`λ = 0.995`, 기억은 약 4초).

| 메서드 | 설명 |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | `reset()`을 호출합니다 |
| `void reset()` | θ = (prior, 0, 0). 공분산은 b ± prior, a ± 5, c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | 매 주기 호출합니다. `ESTIMATOR_PERIOD_MS` 구간 동안 평균을 누적하고, 구간이 끝나면 자이로 차분으로 가속도를 구하고, 명령의 지연을 반영하고, 양쪽에 공통 저역 통과 필터를 적용하고, RLS를 한 단계 진행합니다 (학습이 허용되고 여기(excitation)가 있을 때) |
| `getEffectiveness()` | 현재 속도에서의 `b` |
| `getReferenceEffectiveness()` | 기준 속도에서의 `b` |
| `getDamping()` | 현재 속도에서의 `a` |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b` (`\|b\|`가 작으면 0) |
| `getEffectivenessSigma()` | 현재 속도에서 `b` 추정의 σ |
| `bool isConfident() const` | RLS 50단계 이상, `\|b\|`가 최솟값 이상, σ < 0.3·`\|b\|` |
| `getAngularAccel()` | 마지막 구간의 가속도 (디버깅용) |

특이 사항:

- 구간이 `MAX_GAP_MS = 200`보다 길면 (루프가 멈췄다는 뜻) 데이터를 처음부터 다시
  모읍니다 (지연 이력과 필터를 초기화합니다).
- 학습은 **여기**가 있을 때만 합니다. 16개 구간(약 0.3초)에 걸쳐 평균한 명령의
  진폭이 `MIN_EXCITATION_US` 이상이어야 하며, 그렇지 않으면 추정이 고정됩니다.
- 한 단계가 끝난 뒤의 float 정리로, `P`의 대칭화, 분산의 상한(초기값의 10배),
  `b`(`±EFFECTIVENESS_MAX`)와 `a`(`−40..5`)의 제한을 적용합니다.

---

## `AxisModel`

**파일:** `autopilot/feedback/AdaptiveRateController.h` · **종류:** struct

레귤레이터가 쓰는, 한 축의 반응에 대해 알려진 정보입니다. `effectiveness`
(b, 기본값 1), `damping` (a, 0 — 보상하지 않음), `bias` (c, 0).

---

## `AdaptiveRateController`

**파일:** `autopilot/feedback/AdaptiveRateController.h`

단일 축 레귤레이터이며 3단으로 구성됩니다.

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| 메서드 | 설명 |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | 적분, 포화, 출력을 0으로 만듭니다 |
| `float angleToRate(targetDeg, angleDeg) const` | 1단 (방위에서는 최단 경로) |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | 2–3단. 변위를 µs로 반환합니다 |
| `getDesiredRate()`, `getIntegral()`, `getOutput()`, `isSaturated()` | 상태 |

불변 조건: `|b|`는 `EFFECTIVENESS_MIN` 아래로 내려가지 않습니다 (b의 부호는 유지).
적분은 °/s 단위로 보관하므로 `b`가 바뀌어도 올바르게 유지되며, **스토퍼 방향으로
쌓이지 않습니다** (직전 단계의 포화 방향에 따른 안티 와인드업). `dt ≤ 0`이면
적분은 변하지 않습니다.

---

## `StallGuard`

**파일:** `autopilot/feedback/StallGuard.h`

속도 저하와 실속으로부터의 보호입니다. 단계는 `Level::{Normal, LowEnergy, Stall}`입니다.

| 메서드 | 설명 |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | 지상이거나 IMU가 없으면 Normal로 되돌립니다 |
| `Level getLevel() const`, `const char* getLevelName() const` | `"OK"`, `"LOW_ENERGY"`, `"STALL"` |
| `const char* getReason() const` | 마지막으로 작동한 징후 |
| `float maxPitchDeg() const` | Stall: −5°, LowEnergy: 5°, 그 외 90° |
| `float maxBankDeg() const` | Stall: 10°, 그 외 180° |
| `float maxAileronUs() const` | Stall: 150 µs, 그 외 `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall 100 %, LowEnergy 80 %, 그 외/링크 없음 −1 |
| `void reset()` | Normal |

`StallGuard::ControlState`는 `pitchEffectivenessKnown`, `pitchEffectiveness`
(피치 b 추정값의 절댓값)입니다.

**LowEnergy**의 징후는 다음과 같습니다. 피치가 5°보다 큰 상태에서 `DECEL_WARN_MS2`를
넘는 감속이 (`DECEL_CONFIRM_MS` 동안) 확인될 때, 속도가 `1.25·Vs` 미만일 때,
신뢰할 수 있는 승강타 효율이 사전 효율의 35 % 미만일 때입니다. **Stall**의 징후는
다음과 같습니다. 속도가 Vs 미만일 때, 승강타가 50 µs를 넘게 “올림”인 상태에서 기수가
60 °/s보다 빠르게 내려갈 때, 에너지가 낮은 상태에서 날개가 에일러론과 반대로
120 °/s보다 빠르게 떨어질 때입니다. 조치는 `RECOVERY_HOLD_MS`가 지난 뒤, 그리고
에너지가 회복되었을 때만 해제합니다 (속도가 `1.5·Vs` 이상이거나, 속도 센서가 없으면 가속도가 0 이상).

---

## `TakeoffSequencer`

**파일:** `autopilot/feedback/TakeoffSequencer.h`

단계별 이륙입니다. `State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`.
전이도는 [ARCHITECTURE.md §7](../ARCHITECTURE.md#이륙과-착륙피드백-루프-연결되지-않음)에 있습니다.

| 메서드 | 설명 |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | 활성 단계 → `Aborted`. 목표는 초기화됩니다 |
| `void update(snapshot, speed, nowMs)` | 한 주기에 전이는 최대 한 번이며, 그다음 새 단계의 목표를 설정합니다 |
| `void reset()` | → `Idle` |
| `getTargets()`, `getState()`, `getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

단계별 목표는 다음과 같습니다. 대기에서는 스로틀 0이고 조종면은 조종사에게 맡깁니다.
지상 활주에서는 스로틀 100 %, 날개 수평, 피치는 건드리지 않고, 출발하는 순간
고정한 방위를 유지합니다. 상승에서는 스로틀 100 %, 날개 수평, 피치는
`CLIMB_PITCH_DEG`입니다. 손으로 던져 이륙하는 경우는 전후 방향 가속도
`ax − sin θ ≥ LAUNCH_ACCEL_G`가 `LAUNCH_DETECT_MS`보다 오래 이어질 때 감지합니다.

---

## `LandingSequencer`

**파일:** `autopilot/feedback/LandingSequencer.h`

단계별 착륙입니다. `State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`.

| 메서드 | 설명 |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`, `void reset()`, `update(snapshot, nowMs)` | 이륙과 같습니다 |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout — 여기서는 실속 보호를 끕니다 |
| `bool isOnGround() const` | Rollout / Complete |

강하와 플레어에서의 피치는 수직 속도 오차로 구합니다.
`θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`를 `[min, FLARE_MAX_PITCH_DEG]`로
제한합니다. 기압계가 없으면 기준 각도를 씁니다. 높이는 거리계에서 가져오고, 없으면
기압계에서 가져옵니다. 접지는 |a − 1g| ≥ 0.5g의 피크이거나, `TOUCHDOWN_STILL_MS` 동안
“낮고 회전하지 않음”인 경우로 판단합니다. 착륙 후 활주 중에는 접지한 순간의 방위를 고정합니다.

---

## `FeedbackSupervisor`

**파일:** `autopilot/feedback/FeedbackSupervisor.h`

루프 전체를 총괄합니다. `SpeedEstimator`, `AirborneDetector`, 세 개의
`ControlEffectivenessEstimator`, 세 개의 `AdaptiveRateController`, `StallGuard`,
`TakeoffSequencer`, `LandingSequencer`를 소유합니다.

| 메서드 | 설명 |
|---|---|
| `bool requestTakeoff()` | 시동이 걸려 있고 링크가 있으며 지상에 있을 때만 가능합니다. 착륙을 취소합니다 |
| `bool requestLanding()` | 시동이 걸려 있고 링크가 있으며 공중에 있을 때만 가능합니다. 이륙을 취소합니다 |
| `void cancelPhase()` | 단계를 취소합니다 |
| `const FeedbackOutput& update(const FlightSnapshot&)` | 한 주기 (순서는 [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-피드백-루프연결되지-않음)에 있습니다) |
| `getOutput()`, `isAirborne()`, `getSpeedEstimator()`, `getEstimator(axis)`, `getController(axis)`, `getStallGuard()`, `getTakeoff()`, `getLanding()` | 로그와 테스트용 상태 |
| `void printStatus(Print& out) const` | 상태 한 줄 + 축마다 한 줄 (`b ± σ`, `*` — 신뢰 가능, `a`, `c`, `I`, 출력) |

핵심 규칙:

- **시동이 걸려 있지 않으면** 모든 축을 비활성화하고 `reason = "미시동"`으로 둡니다.
  ARM/DISARM(새 비행)을 하면 학습한 내용이 모두 지워집니다.
- **링크가 끊기면** 단계를 취소합니다. 스로틀은 건드리지 않습니다
  (펌웨어의 페일세이프가 동작합니다).
- **지상에서 이륙하면** 추정값과 레귤레이터를 초기화합니다 (바퀴 위에서
  “본” 값은 맞지 않습니다).
- `b`의 신뢰할 수 있는 추정값이 음수이면 레귤레이터에 **절대** 전달하지 않습니다.
  그 축은 사전 모델로 동작하고, `reason`에는 “… 조종면에 반대로 반응하나요? 지상에서 확인하세요”라는 경고가 담깁니다.
- 적분은 지상에서 동결됩니다. 다만 이륙 활주와 착륙 활주 중의 방위는 예외입니다.
- 협조 선회 (공중에서 속도를 알 때): 목표 피치 각속도에
  `+ g/V · sin φ · tg φ`를, 요 각속도에 `g/V · sin φ`를 더합니다
  (뱅크 각은 ±60°로 제한합니다).
- `reason`의 우선순위는 실속 > 에너지 부족 > 단계 > 부호 경고 >
  “안정화”/“수동(학습)”입니다.
