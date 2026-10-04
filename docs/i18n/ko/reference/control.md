# CONTROL과 COORDINATION — 믹서, 스로틀, ARM, 출력, 조율자

> 🌐 이 문서는 [러시아어 원문](../../../reference/control.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다. 이 번역은 AI가 작성했으며 원어민의 검수를 거치지 않았습니다. 오류를 발견하면 [Damir Lebedev](https://github.com/damir-lebedev)에게 알려 주시거나 [이슈](https://github.com/damir-lebedev/OpenPlaneProject/issues)로 남겨 주세요.

[← 참조](README.md)

CONTROL 계층은 UART, PWM, Wi-Fi 없이 데이터 위에서 동작하는 로직입니다.
`FlightController`(COORDINATION)는 모든 하위 계층을 한 주기로 묶는 유일한
클래스입니다.

---

## `ControlCommand`

**파일:** `control/ControlCommand.h` · **종류:** struct

**물리적 부호**로 나타낸 조종면 명령이며, 변위는 µs입니다(±500 = 최대 행정).
스틱, 오토파일럿, 믹서가 함께 쓰는 공통 언어입니다.

| 필드 | “+”의 의미 |
|---|---|
| `int16_t roll` | 오른쪽 롤(오른쪽 에일러론 위, 왼쪽 아래) |
| `int16_t pitch` | 기수 위(승강타 위) |
| `int16_t yaw` | 기수 오른쪽(방향타와 바퀴가 오른쪽) |
| `int16_t flaps` | 플랩 아래(두 에일러론 모두 아래). “−”는 에어브레이크(둘 다 위) |

모든 필드의 기본값은 0입니다.

---

## `FlightOutputState`

**파일:** `control/FlightOutputState.h` · **종류:** struct

원하는 출력 펄스이며 단위는 PWM µs입니다. 기본값은 조종면 중립, 스로틀 `PWM_MIN`입니다.

| 필드 | 기본값 |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `aux1` | `PAYLOAD_CLOSED_US` — 페이로드 투하기는 닫힘 |
| `aux2` | `PWM_CENTER` — 카메라 |

---

## `FlapsController`

**파일:** `control/FlapsController.h` · **의존:** `Config`

플랩을 부드럽게 펼치고 접습니다. 위치는 목표(스위치의 플랩, 노브의 플랩, 위쪽
에어브레이크 등 어떤 값이든)를 향해, `FLAPS_TRANSITION_MS` 동안 전체 행정
`FLAPS_DEPLOYED_US`를 넘지 않는 속도로 움직입니다. 시간은 매개변수로 전달합니다.

| 메서드 | 설명 |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | 목표를 향해 한 걸음 이동하고 현재 위치를 µs로 반환(+ 아래, − 위) |
| `int16_t getPosition() const` | 현재 위치 |

불변 조건:

- **첫 번째 호출**에서는 위치를 곧바로 목표에 맞춥니다. 전원을 켤 때 책상 위에서 플랩이
  “빠져나오지” 않습니다.
- 시간 간격은 `MAX_STEP_MS = 20`으로 제한됩니다. 긴 중단(failsafe, 보정) 뒤에도 플랩이
  한 주기 만에 목표로 튀지 않습니다.

---

## `ControlMixer`

**파일:** `control/ControlMixer.h` · **의존:** `RcInput`, `RcChannelState`, `FlapsController`, `ControlCommand`, `FlightOutputState`, `Config`, `Channels`

공기역학 로직을 두 단계로 처리합니다. `FlapsController`를 소유합니다.

| 메서드 | 설명 |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll`(2000 = 오른쪽), CH2 → `pitch`(**부호가 반대**: 2000 = 몸에서 멀어지는 방향 = 기수 아래), CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | 플랩의 목표는 `FlightController`가 고르고(에어브레이크 → 플랩 스위치 → 노브), 여기서는 부드러운 움직임을 담당 |
| `FlightOutputState mix(const ControlCommand& c) const` | 명령 → PWM. 롤/피치/요는 행정(`*_MAX_US`)으로 제한. 에일러론은 왼쪽 = `flaps + roll`, 오른쪽 = `flaps − roll`(아래 = “+”). PWM = `1500 ± 변위`이며 부호는 `Config::*_REVERSED`를 따르고 1000..2000으로 제한. `throttle`은 채우지 않음 |
| `int16_t getFlaps() const` | 현재 플랩 위치(µs) |

플래퍼론: 펼치면 두 에일러론이 `FLAPS_DEPLOYED_US`만큼 내려가고(새로운 “중립”)
롤은 그 위에 얹혀 동작합니다. 롤을 끝까지 주면 내려가는 에일러론이 올라가는
에일러론보다 먼저 행정 끝에 닿습니다. 이것이 에일러론 차동으로 작용합니다.

---

## `ThrottleManager`

**파일:** `control/ThrottleManager.h` · **의존:** `RcInput`, `RcChannelState`, `Config`, `Channels`

| 메서드 | 설명 |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | CH3의 스로틀을 1000..2000으로 제한해 반환. 링크를 잃으면 `FAILSAFE_THROTTLE` |

ARM과 오토파일럿에 대해서는 모릅니다. 그 보정은 `FlightController`가 적용합니다.

---

## `ArmingManager`

**파일:** `control/ArmingManager.h` · **의존:** `Autopilot`(null 가능), `RcChannelState`, `Config`, `Channels`

ARM은 별도의 스위치 SwA(CH5)로 합니다. 상태 기계는
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager)에 있습니다.

| 메서드 | 설명 |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | 오토파일럿이 없으면 스로틀만 점검 |
| `void update(const RcChannelState& rc, bool receiverFailsafe)` | failsafe 중에는 아무것도 하지 않음(failsafe 프레임의 스위치는 조종자의 조작을 반영하지 않음). 스위치 OFF → DISARM, `switchSeenOff = true`. OFF→ON 전환 → 점검 → ARM 또는 거부 |
| `bool isArmed() const` | ARM 상태인지 |
| `const char* getLastRefusalReason() const` | 마지막 거부 사유 또는 `nullptr`. 스위치를 끄면 지워짐 |

`checkFailureReason(rc)` — ARM 점검 항목:

| 조건 | 거부 사유 |
|---|---|
| 스로틀 ≥ `THROTTLE_LOW_US` | “스로틀이 최저가 아님” |
| MANUAL 이외의 모드에서 IMU는 있으나 응답하지 않음 | “IMU가 응답하지 않음…” |
| MANUAL 이외의 모드에서 IMU의 비행 전 점검에 문제가 있음 | `ImuSensor::getPreflightProblem()`의 문구 |
| 고도를 쓰는 모드(`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING)에서 기압계는 있으나 응답하지 않음 | “기압계가 응답하지 않음…” |

빌드에 없는 센서(`nullptr`)는 ARM을 막지 않으며, MANUAL에서는 센서가 하나도 없어도
기체가 ARM됩니다. GPS 측위는 의도적으로 점검 항목에 넣지 않았습니다. GPS가 없어도
내비게이션 모드는 안전하게 동작하고(제자리 선회), 홈은 GPS가 위성을 잡는 시점에 기록되기
때문입니다.

불변 조건: 스위치가 ON인 채로 보드의 전원을 켜도 ARM되지 않음. OFF→ON 전환 한 번에
시도는 한 번. 링크 상실이 ARM을 해제하지는 않음.

---

## `FlightOutputs`

**파일:** `control/FlightOutputs.h` · **의존:** `IBoard`, `FlightOutputState`, `Config`

PWM 출력의 구성과 순서를 아는 유일한 클래스입니다. 모든 출력이 하나의 표에
기술되어 있고, `begin()`, `write()`, 상태 출력, 자가 점검이 그 표를 루프로
순회합니다.

### `FlightOutputs::OutputInfo`

| 필드 | 설명 |
|---|---|
| `const char* key` | JSON/로그에서의 이름(`aileronLeft`, …, `esc`, `rudder`, `aux1`, `aux2`) |
| `const char* label` | 사람을 위한 이름 |
| `int16_t pin` | 핀 번호. `-1`은 배선되지 않음. STM32에서는 아날로그 핀의 번호가 `0xC0 + N`이라 `int16_t` |
| `bool required` | 이것이 없으면 기체가 비행할 수 없음(방향타는 선택 사항) |
| `uint16_t FlightOutputState::* field` | 상태 필드에 대한 포인터 |

| 메서드 | 설명 |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | 표의 한 행. 순서는 `ServoChannel`과 같음 |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | 각 출력의 `attach(PWM_MIN, PWM_MAX)`를 하고 상태를 출력. **필수** 출력이 모두 채널을 얻으면 `true` |
| `bool isAttached(uint8_t ch) const` | 출력이 연결되어 있음(범위를 벗어난 인덱스는 `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | 표에 따라 상태에서 출력 값을 꺼냄 |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | 배선된 각 핀에서 측정한 펄스를 기대값과 비교. 차이가 15 µs 이하이면 “OK” |
| `void write(const FlightOutputState&)` | 모든 출력을 기록하고 상태를 기억 |
| `void setFailsafe()` | 조종면 중립(`FAILSAFE_*`), 스로틀 `FAILSAFE_THROTTLE`. AUX는 그대로(링크 상실로 페이로드가 투하되지 않음) |
| `void setBuzzer(bool on)` | 보드의 부저(`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | 마지막으로 기록한 상태 |

출력을 추가하려면 표에 행을 추가하고, `FlightOutputState`에 필드를 추가하고,
`ServoChannel`에 인덱스를 추가합니다(그리고 `Esp32Board`에 핀과 LEDC 채널도).

---

## `FlightController`

**파일:** `control/FlightController.h` · **계층:** COORDINATION ·
**의존:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`

제어 루프의 유일한 조율자입니다. 직접 UART를 해석하지 않고, PWM을 건드리지 않고,
믹서를 계산하지도 않으며, 다른 클래스를 올바른 순서로 호출할 뿐입니다. 자세한 도표는
[ARCHITECTURE.md §6](../ARCHITECTURE.md#6-제어-주기-flightcontrollerupdate)에 있습니다.

| 메서드 | 설명 |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | 오토파일럿이 없으면 순수 수동 조종, 스위치가 없으면 스틱만 |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | 한 주기(아래 참고) |
| `bool isReceiverFailsafe() const` | 링크를 잃음 |
| `const IBusReceiver& getReceiver() const` | 로그용(프레임 카운터, 상실 원인) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | 출력에 마지막으로 기록한 상태 |
| `const RcChannelState& getRcState() const` | 채널 |
| `const FlightOutputs& getOutputs() const` | 출력 표와 `attached` |
| `int16_t getFlapsUs() const` | 플랩 위치 |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | 이번 주기의 스위치와 노브 |
| `bool isLostModelBeeping() const` | “여기 있어요” 부저가 울리는 중 |

`update()`의 순서:

1. `receiver.update()`. `failsafe = receiver.isSignalLost()`.
2. 링크가 살아 있으면 `switches->update(rc)`(모드, 기능, 노브).
3. `pilotThrottle = throttle.update(rc, failsafe)`.
4. 스틱 `mixer.fromSticks(rc)`(링크가 살아 있을 때) × `Knob::RATES`. 플랩은
   `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → 부드럽게, 링크를 잃으면 0.
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)`를 **항상** 실행.
6. 부저: `Beeper::update(BEEPER, armed, failsafe, now)`.
7. 링크 상실 → `applyLinkLoss()`를 실행하고 주기를 빠져나감.
8. `arming.update(rc, false)`.
9. `command = autopilot->getCommand()`(오토파일럿이 없으면 스틱). 플랩은 자체 값.
10. `output = mixer.mix(command)`. `output.throttle = autopilot->applyThrottle(pilotThrottle)`.
11. ARM하지 않았거나 `MOTOR_KILL` → `throttle = PWM_MIN`(맨 마지막에).
12. AUX1은 페이로드(`PAYLOAD_DROP`), AUX2는 카메라(`Knob::CAMERA_TILT`, `CAMERA_STAB`은 피치를 뺌).
13. `outputs.write(output)`.

`applyLinkLoss()`: 오토파일럿이 failsafe 상태(ARM 상태에서는 RTH 또는 활공)이면 조종면과
스로틀은 오토파일럿의 명령을 따릅니다(플랩은 부드럽게 접히고, `MOTOR_KILL`은 여전히
모터를 멈추며, AUX는 그대로). 그렇지 않으면 `outputs.setFailsafe()`를 실행합니다.

---

## `Beeper`

**파일:** `control/Beeper.h` · **의존:** `Config`

| 메서드 | 설명 |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | 부저 상태: `Feature::BEEPER`이거나 “기체 분실”(ARM하지 않았고 `LOST_MODEL_BEEP_DELAY_MS`보다 오래 링크가 없음)이면 2 Hz로 울림 |
| `bool isLostModel() const` | “풀숲에서 나를 찾아 줘” 모드 |
