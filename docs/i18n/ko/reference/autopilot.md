# AUTOPILOT — 모드, 내비게이션, 스위치

> 🌐 이 문서는 [러시아어 원문](../../../reference/autopilot.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다. 이 번역은 AI가 작성했으며 원어민의 검수를 거치지 않았습니다. 오류를 발견하면 [Damir Lebedev](https://github.com/damir-lebedev)에게 알려 주시거나 [이슈](https://github.com/damir-lebedev/OpenPlaneProject/issues)로 남겨 주세요.

[← 참조](README.md)

오토파일럿은 조종자의 스틱, 스위치/노브(`PilotInputs`), 센서를 받아 **최종 조종면
명령**(`getCommand()`)과 모드의 스로틀(`applyThrottle()`)을 내놓습니다. 필요한 센서가
없으면 모드는 비정상 종료하지 않고 안전하게 동작합니다(조종면은 조종자에게 넘어가거나
중립이 됨). 각 모드가 조종자에게 무엇을 해 주는지는 [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md)에 있습니다. 지금까지 비행에서
쓴 것은 수동 모드뿐이며, 오토파일럿은 테스트 벤치, 테스트, 폐루프 시뮬레이션
(`test/native/test_sim`)으로 확인했습니다.

---

## `AutopilotMode`, `Feature`, `Knob`

**파일:** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t`(스코프 없음 — 숫자 코드가 `/api/setmode`, `/api/status`의
JSON과 `MavlinkModes`에 쓰임):

| 값 | 코드 | 약칭(OLED) | 요지 |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | 조종면 = 스틱 |
| `MODE_STABILIZE` | 1 | STAB | 스틱이 롤/피치 각도 |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | 조종자의 스로틀로 시작하는 이륙 프로그램 |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + 승강타로 고도 유지 |
| `MODE_ACRO` | 4 | ACRO | 스틱이 각속도 |
| `MODE_CRUISE` | 5 | CRZ | 침로 + 고도 + 자동 스로틀 |
| `MODE_LOITER` | 6 | LOIT | 켠 지점 위에서 선회 |
| `MODE_RTH` | 7 | RTH | 홈으로 복귀, 홈 위에서 선회 |
| `MODE_LAUNCH` | 8 | LNCH | 손 발사 |
| `MODE_AUTO_LAND` | 9 | LAND | 활공 + 플레어 |
| `MODE_SOARING` | 10 | SOAR | 모터 없이 열상승 이용 |
| `MODE_RESCUE` | 11 | RESQ | 날개 수평, 기수 위, 스로틀 |
| `MODE_COUNT` | 12 | | 경계(`setMode()`는 ≥를 무시) |

`enum class Feature : uint8_t` — 스위치의 기능: `FLAPS`, `AIRBRAKE`,
`AUTO_TRIM`, `TURN_COORDINATION`, `MOTOR_KILL`, `BEEPER`, `PAYLOAD_DROP`,
`GEOFENCE`, `HOME_RESET`, `CAMERA_STAB`, `MOTOR_REVERSE`, `COUNT`.

`enum class Knob : uint8_t` — 노브: `STAB_GAIN`, `MAX_BANK`,
`CRUISE_SPEED`, `FLAPS`, `CAMERA_TILT`, `RATES`, `LOITER_RADIUS`, `THROTTLE_LIMIT`, `COUNT`.

`namespace AutopilotNames` — `mode()`, `modeShort()`(5자 이하),
`feature()`, `knob()`: 로그, OLED, 대시보드, MAVLink용 이름.

### `PilotInputs`

한 주기 동안의 스위치와 노브 상태입니다.

| 멤버 | 설명 |
|---|---|
| `bool has(Feature) const` | 해당 기능이 켜져 있음 |
| `float knob(Knob) const` | 노브 위치 −1…+1 |
| `bool isBound(Knob) const` | 노브가 할당 표에 있음 |
| `float knobValue(Knob, min, default, max) const` | 단위 값: 중앙이 `default`, 양 끝이 `min`/`max`. 할당되지 않았으면 `default` |

---

## `Binding`, `Bind`, `BindingCheck`

**파일:** `autopilot/ControlBinding.h` · 표: `config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`,
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`. 표의 각 행은 `namespace Bind`의
팩토리입니다(모두 `constexpr`):

| 팩토리 | 의미 |
|---|---|
| `modes(ch, up, middle, down)`, `modes(ch, up, down)` | 모드 선택 스위치(구간은 `PilotSwitches::zoneOf`로 결정) |
| `mode(ch, m)` | 채널이 `SWITCH_ON_US` 이상인 동안 위에 덮어씌우는 모드 |
| `feature(ch, f)` | 채널이 `SWITCH_ON_US` 이상인 동안 켜지는 기능 |
| `knob(ch, k)` | 노브. `(us − 1500) / 500`을 ±1로 제한 |

`namespace BindingCheck` — 재귀적인 `constexpr` 함수들(ESP32 코어는 C++11로 빌드됨):
`channelIsFree`, `channelsFree`, `channelsUnique`, `modeSwitchCount`,
`atMostOneModeSwitch`. `Controls.h`의 `static_assert`에서 사용됩니다.

---

## `PilotSwitches`

**파일:** `autopilot/PilotSwitches.h` · **의존:** `Autopilot*`, `RcChannelState`, 할당 표

| 메서드 | 설명 |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | 직접 만든 표(테스트, 시뮬레이션용) |
| `explicit PilotSwitches(Autopilot* = nullptr)` | `Controls::BINDINGS` 표 |
| `void update(const RcChannelState&)` | `PilotInputs`를 모아 `autopilot->setInputs()`에 전달. `setMode()`는 **스위치의 결과가 바뀐 경우에만** 호출(대시보드/지상국에서 정한 모드가 매 주기 덮어쓰이지 않음). `FlightController`는 링크가 살아 있을 때만 호출 |
| `void printBindings() const` | 전원을 켤 때 Serial로 배치를 출력: `SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (up / middle / down)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`, `"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2구간: < 1500 / ≥ 1500. 3구간: < 1250 / < 1750 / ≥ 1750 |
| `getInputs()`, `binding(i)` | 텔레메트리와 테스트용 |

`Bind::mode`는 `Bind::modes`보다 우선합니다. 켜진 `Bind::mode`가 여러 개이면 위쪽
행이 이깁니다.

---

## `Autopilot`

**파일:** `autopilot/Autopilot.h` · **의존:** `PidController`, `Navigation`, `AltitudeSpeedController`, `LaunchController`, `TakeoffController`, `SoaringController`, `AutoTrim`, 각 센서(모두 null 가능)

### 생명 주기

| 메서드 | 설명 |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | 롤/피치 PID: Kp 5, Ki 0.5, Kd 0.5, 출력 ±500 µs |
| `bool begin()` | 트림을 불러옴. IMU나 기압계가 없으면 `false`와 메시지 |
| `void setInputs(const PilotInputs&)` | 이번 주기의 스위치와 노브(`update` 전에 호출) |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | 주기마다 한 번: 센서(항상) → 내비게이션과 홈 → 지상에서 트림 저장 → failsafe → 지오펜스 → 모드 → 선회 협조 → 자동 트림 |
| `ControlCommand getCommand() const` | 최종 조종면 명령(roll/pitch/yaw, µs) |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | 모드의 스로틀: `PILOT`은 조종자의 것, `AUTO`는 자체 값(자동 이륙도 마찬가지: 스로틀 스틱은 시작과 중단만). ARM과 `MOTOR_KILL`은 `FlightController`가 처리 |

### 모드

| 메서드 | 설명 |
|---|---|
| `void setMode(AutopilotMode)` | 같은 모드이거나 ≥ `MODE_COUNT`이면 아무것도 하지 않음. 아니면 PID와 상태 기계를 초기화하고, 목표 = 현재 침로와 고도, 선회 원의 중심 = 현재 지점(GPS가 있을 때), RTH는 복귀 고도 |
| `getMode()`, `getModeName()` | 이름: 링크 상실 시 `FAILSAFE_GLIDE` / `FAILSAFE_RTH`, 그 외에는 모드 이름 |
| `isFailsafeActive()`, `isFailsafeGliding()`, `isFailsafeReturning()` | 모드 위에 덮어씌우는 failsafe |
| `isAutoThrottle()`, `getThrottleCorrection()` | 모드의 스로틀(%, 로그와 대시보드용) |
| `getLaunchState()`, `getSoaringState()` | LAUNCH와 SOARING의 상태 기계 |
| `getTakeoffStage()`, `getTakeoffAbortReason()` | 자동 이륙의 단계와 중단된 이유(`nullptr`은 중단 없음) |
| `bool takeTakeoffRefusal()` | 자동 이륙이 시작 전 점검에서 거부함: `FlightController`가 알림을 냄(한 번만 꺼냄) |
| `getLandingGear()`, `setLandingGear(LandingGear)` | 랜딩기어(기본값 `Config::LANDING_GEAR`. 테스트에서 바꿈) |
| `bool isGyroOnlyAttitude() const` | 지금은 수평 기준을 자이로만으로 구함(자동 이륙의 가속 구간) |

### 출력과 진단

| 메서드 | 설명 |
|---|---|
| `getRollCorrection()`, `getPitchCorrection()`, `getYawCorrection()` | 명령 − 스틱(µs) |
| `getDesiredRoll()`, `getDesiredPitch()`, `getTargetAltitude()` | 목표값 |
| `const NavStatus& getNavStatus()` | GPS, 홈, 위치, 홈까지의 거리와 방위, 침로와 목표 침로, 내비게이션용 속도, 지오펜스, 실속 |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | 기압계 고도(전원을 켠 지점 기준 m) |
| `getInputs()`, `getAutoTrim()` | 텔레메트리용 |
| `getImuSensor()` … `getAirspeedSensor()` | 각 센서(`nullptr`일 수 있음) |
| `getRollPid()`, `getPitchPid()`, `setPIDGains(...)` | PID(대시보드, MAVLink 파라미터) |

### 내부 동작

- `stabilize()` — 각도 PID이며 D 항은 자이로에서 가져오고 `Knob::STAB_GAIN`을 곱한다.
  적분기는 ARM 상태이고 오차가 `STAB_INTEGRATOR_ZONE_DEG` 미만일 때만 누적된다.
  `stabilizeOrManual()`은 IMU가 없으면 조종면을 조종자에게 맡기고,
  `stabilizeOrNeutral()`은 IMU가 없으면 중립으로 둔다(자동 모드용).
- `imuReady()` = IMU가 있고, 사용 가능하며, 비행 전 점검에 문제가 없음.
- 내비게이션용 속도: 피토관 → GPS → `NAV_ASSUMED_SPEED_MS`.
- `looksLanded()` — 기압계로 보아 지면 근처이고, 수직 속도가 거의 없으며, 피토관/GPS의
  임계값보다 느린 경우. 이때만 트림을 플래시에 기록한다.
- Failsafe: GPS와 홈이 있으면 모터를 쓰는 RTH, 없으면 활공. 이미 시작한 RTH는 GPS를
  잠깐 놓쳤다고 중단하지 않는다. 활주로 위의 AUTO_TAKEOFF(`holdsGround()`인 단계)에서는 통신 두절이 failsafe가 아니라 이륙 중단입니다. 상승 중에는 failsafe이고, 통신이 돌아오면 상승을 계속합니다.
- “자이로만” 구간(`manageAccelCorrection()`): 자동 이륙이 모터를 시동하면 수평 기준의 지지대로서 가속도계를 뺍니다(`ImuSensor::setAccelCorrection`). 다시 넣는 것은 상승 시작 뒤의 시간(`TAKEOFF_GYRO_ONLY_CLIMB_MS`), 상한, 이륙 중단, DISARM이며, 모드 전환으로는 돌아오지 않습니다.

---

## `Geo`, `Guidance`, `GeoPoint`

**파일:** `autopilot/Navigation.h`

미터 단위의 국소 “북/동” 평면입니다(정거 원통 도법 — 수 km 범위에서 오차는 1 %의
몇분의 일 수준).

| 함수 | 설명 |
|---|---|
| `Geo::wrap180`, `Geo::wrap360` | 각도 정규화 |
| `Geo::offsetNE(a, b, north, east)`, `distance(a, b)`, `bearing(a, b)` | 오프셋, 거리, 방위 0..360 |
| `Geo::moved(a, north, east)` | 오프셋을 더한 지점 |
| `Geo::fromGps(GpsData)` | GPS에서 `GeoPoint`를 만듦 |
| `Guidance::rollForCourse(target, course, bankLimit)` | 침로 오차에 대한 뱅크(`NAV_COURSE_GAIN`), 제한됨 |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | 원 궤도로 이끄는 벡터장의 침로(`LOITER_CONVERGENCE`) |
| `Guidance::orbitBankDeg(speed, radius)` | 원 궤도의 선행 뱅크: atan(V²/(g·R)) |

## `AltitudeSpeedController`

**파일:** `autopilot/AltitudeSpeedController.h` — TECS-lite.

| 메서드 | 설명 |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | 원하는 수직 속도 = `NAV_ALT_GAIN`·오차(≤ `NAV_MAX_CLIMB/SINK`). 피치 = 선행항 asin(Vz/V) + Vz 오차에 대한 PI이며 `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 범위 안 |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | 피토관이 있으면 `cruisePct`를 중심으로 대기속도에 대한 PI, 없으면 `cruisePct`. 필요한 상승분으로 `THROTTLE_PER_CLIMB_PCT`를 더함 |
| `reset()`, `getWantedClimb()` | |

## `LaunchController`

**파일:** `autopilot/LaunchController.h`

`State`: `IDLE → READY`(스로틀을 올림) `→ THROWN`(과부하 > `LAUNCH_ACCEL_G`가
`LAUNCH_ACCEL_TIME_MS`보다 오래 지속) `→ CLIMB`(`LAUNCH_MOTOR_DELAY_MS` 뒤: 모터 시동,
피치 `LAUNCH_CLIMB_PITCH_DEG`) `→ DONE`(`LAUNCH_CLIMB_MS` 또는
`LAUNCH_ALTITUDE_M`). 던지기 전에 스틱을 움직이면 취소. 메서드: `update(...)`,
`reset()`, `getState()`, `motorOn()`, `pitchTargetDeg()`, `stateName()`.

## `TakeoffController`

**파일:** `autopilot/TakeoffController.h` · **의존:** `Config`

활주로에서의 자동 이륙. 센서에 따른 단계로 진행하며 자체 센서와 조종면은 없음: 입력은 `Inputs`(ARM, 통신, 조종자의 스로틀, 각도와 각속도, 하중, 주기 자세, 고도, 속도, 노브 `STAB_GAIN`, 시간), 출력은 이번 주기에 할 일.

`Stage`: `IDLE`(모터 정지. 스로틀을 한 번 내렸다가 `TAKEOFF_TRIGGER_US` 위로) `→ CHECK`(`TAKEOFF_CHECK_MS` 동안 정지, 각도 = 주기 자세. 아니면 `ABORTED`와 알림) `→ TAIL_UP`(꼬리 바퀴: 승강타를 내려 꼬리를 듦) `→ ROLL`(활주. `TD_ROLL_MS` / `TRI_ROLL_MS` 또는 기수 들기 속도) `→ LIFTOFF`(피치 목표를 올림. 지면 가까이에서는 주기 자세에서 `TD_TAIL_CLEARANCE_DEG`를 뺀 값까지) `→ CLIMB`(기압계나 시간으로 부양 판단, `TAKEOFF_SAFE_ALTITUDE_M` / `TAKEOFF_CLIMB_MS`까지 상승) `→ DONE`. `ABORTED`는 이륙 중단이며, 스로틀을 1초 내리면 다시 `IDLE`.

| 메서드 | 설명 |
|---|---|
| `void update(const Inputs&)` | 주기마다 한 번: 단계 전환 뒤 그 단계의 출력. ARM이 없으면 `reset()` |
| `void abort(const char* reason, uint32_t nowMs)` | 외부에서의 중단(통신 두절). `IDLE`/`DONE`에서는 아무것도 하지 않음 |
| `void reset()`, `setGear()`, `getGear()` | |
| `getStage()`, `stageName()`, `stageShort()`, `getAbortReason()` | 단계(OLED용 짧은 이름) |
| `bool holdsGround() const` | 지상(`IDLE`…`LIFTOFF`, `ABORTED`): 통신 두절은 이륙 중단, PID 적분기는 0으로 유지 |
| `isAirborneStage()`, `isDone()` | 공중(상승/완료), 이륙 종료 |
| `getThrottlePct()` | 프로그램 스로틀, %(`TAKEOFF_THROTTLE_RAMP_MS`에 걸쳐 올림) |
| `getRollTargetDeg()`, `getPitchTargetDeg()` | 롤 목표(바퀴 위에서는 주기 상태의 롤)와 피치 목표(상승 중, PID용) |
| `hasElevatorCommand()`, `getElevatorUs()` | 바퀴 위에서는 승강타를 직접: 목표 쪽으로 `TD_PITCH_KP` / `TD_ROLL_PITCH_KP`(1°당 µs), 감쇠 `TD_PITCH_KD`, 아래 타각은 최대 `TD_TAIL_UP_ELEVATOR_US` |
| `hasRudderCommand()`, `getRudderUs()` | 방향타와 조향 바퀴로 침로 유지: 1°당 `TAKEOFF_HEADING_KP` µs, 1 °/s당 `TAKEOFF_YAW_RATE_KD` |
| `bool takeRefusal()`, `bool takeGyroTrim(float[3])` | 한 번뿐인 이벤트: 점검에서의 거부, 정지 상태에서 잰 자이로 잔여 영점 |

## `SoaringController`

**파일:** `autopilot/SoaringController.h`

`State`: `GLIDE ⇄ THERMAL`(승강계 > `SOAR_THERMAL_CLIMB_MS`가
`SOAR_THERMAL_CONFIRM_MS`보다 오래 지속 / `SOAR_EXIT_WINDOW_MS` 동안의 평균이
`SOAR_EXIT_CLIMB_MS` 미만), `→ MOTOR_CLIMB`(`SOAR_MIN_ALTITUDE_M` 아래에서
`SOAR_MAX_ALTITUDE_M`까지), `→ RETURN`(`SOAR_MAX_DISTANCE_M`를 넘으면 그 70 %까지).
메서드: `update(climb, alt, distHome, dt, now)`, `reset(now)`, `getState()`,
`motorOn()`, `getAverageClimb()`, `stateName()`.

## `AutoTrim`

**파일:** `autopilot/AutoTrim.h` · 저장: `Preferences`(NVS / STM32 플래시), 네임스페이스 `"autotrim"`

| 메서드 | 설명 |
|---|---|
| `void load()` | NVS에서 트림을 불러옴(없으면 0) |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += 명령 · `AUTOTRIM_RATE` · dt, 상한은 ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | 변경이 있으면 기록(지상에서 DISARM한 뒤 `Autopilot`이 호출) |
| `reset()`, `getRoll()`, `getPitch()` | |

---

## `PidController`

**파일:** `autopilot/PidController.h` · **의존:** `Config`(공칭 `dt`)

| 메서드 | 설명 |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`, `getKp/Ki/Kd()` | 계수 |
| `setLimits(minOut, maxOut)` | 출력 제한(기본값 ±500) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | 출력을 `[min, max]`로 제한해 반환 |
| `void reset()` | 적분기를 0으로 만들고 `dt`는 “지금”부터 계산 |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (센서에서 얻은 속도!)
out = constrain(P + I + D, min, max)
```

D 항은 오차의 미분이 아니라 측정량의 변화율(자이로 °/s)에서 구합니다. 미분 잡음이 없고
목표값을 바꿀 때에도 튀지 않습니다. `dt`는 `micros()`에서 구하며, `reset()` 뒤의 첫 호출이나
0.1초를 넘는 중단 뒤에는 공칭 `LOOP_PERIOD_MS`를 씁니다.
