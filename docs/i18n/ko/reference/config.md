# CONFIG — `Config`, `Channels`, `Controls`

> 🌐 이 문서는 [러시아어 원문](../../../reference/config.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다. 이 번역은 AI가 작성했으며 원어민의 검수를 거치지 않았습니다. 오류를 발견하면 [Damir Lebedev](https://github.com/damir-lebedev)에게 알려 주시거나 [이슈](https://github.com/damir-lebedev/OpenPlaneProject/issues)로 남겨 주세요.

[← 참조](README.md)

설정 계층은 `constexpr` 상수만 있고 코드는 없습니다. 클래스의 로직에 “매직 넘버”처럼
핀, 타임아웃, 임계값을 두어서는 안 됩니다. 특정 기체나 보드에 맞춰 바꿔야 할 수 있는
것은 모두 여기에 둡니다.

---

## namespace `Config`

**파일:** `include/config/Config.h` · **의존:** `<stdint.h>` ·
**사용처:** 거의 모든 계층.

### 핀(보드에 따라 다름)

핀 블록은 `platformio.ini`의 `[env:*]`가 정의하는 매크로로 선택됩니다
(`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`). 매크로가 없으면 `#error`입니다. STM32 블록은
[아래](#stm32h743vit6board_stm32h743)에서 설명합니다.

| 상수 | 타입 | 용도 | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | 에일러론 | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | 승강타 | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | 모터 조속기 | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | 방향타 + 바퀴. `-1`은 출력 비활성 | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | iBUS 수신기의 RX(UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | 센서 버스(`Wire`) | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | OLED 버스(`Wire1`). `-1`은 없음 | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | 공용 SPI 버스 | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | SPI로 연결한 IMU의 CS | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | SPI로 연결한 기압계의 CS | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | GPS의 UART. TX가 `-1`이면 수신 전용 | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | GPS용 하드웨어 UART 번호 | 2 | 0 | 2 |
| `PIN_AUX1`, `PIN_AUX2` | `int8_t` | 서보 출력: 페이로드 투하, 플랩. `-1`은 없음 | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | 트랜지스터를 거치는 부저. `-1`은 없음 | 38 | −1 | 2 |
| `PIN_AUX3`, `PIN_LIGHT`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_TX/RX` | `int8_t` | **S3 전용:** 비행 컨트롤러 보드용 예약([FC_BOARD.md](../FC_BOARD.md)) | 47, 21, 8, 3, 9/10 | — | — |

센서의 SPI 버스는 `PIN_SPI_*`가 아니라 `PIN_SENSOR_SPI_*`라는 이름입니다. STM32duino
코어(와 다른 Arduino 코어)에서는 `PIN_SPI_SCK/MISO/MOSI`가 변종의 매크로여서 `Config`의
상수를 대체해 버리기 때문입니다.

<a id="stm32h743"></a>

#### STM32H743VIT6(`BOARD_STM32H743`)

아직 보드가 없습니다. 핀 배치는 **실제 하드웨어에서 확인하지 않았습니다**(펌웨어는 PC에서 실행, env `native-stm32`). 핀은 WeAct MiniSTM32H743VITx
(PlatformIO env `stm32h743`의 보드)에서 비어 있는 핀 중에서 골랐고 STM32duino 변종의
`PeripheralPins` 표와 대조했습니다. 값은 변종의 매크로(`PA0`…)이므로, `Config.h`
맨 앞의 `#if defined(BOARD_STM32H743)` 아래에서 `<Arduino.h>`를 포함합니다. 모든 핀의
타입은 `int16_t`입니다(아날로그 핀의 번호는 `0xC0 + N`). UART 번호는 없으며, 주변장치는
코어가 핀을 보고 선택합니다.

| 상수 | 핀 | 주변장치 |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7(TX는 iBUS-SENS용 예약) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2 — 센서 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1 — 화면(WeAct에서는 카메라 커넥터) |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 — 페이로드 / 카메라 |
| `PIN_BUZZER` | PE15 | GPIO — 부저 |
| `PIN_VBAT_ADC`, `PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11 — 예약 |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4 — MAVLink 무선 모뎀(같은 핀이 FDCAN1이기도 함) |

콘솔 `Serial`은 LPUART1(PA9 TX / PA10 RX)이며 변종의 기본값입니다.

### iBUS와 링크 상실

| 상수 | 값 | 의미 |
|---|---|---|
| `IBUS_CHANNELS` | 10 | 프레임에서 사용하는 채널 수 |
| `IBUS_FRAME_LENGTH` | 32 | 프레임 길이(바이트) |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | 프레임 헤더 |
| `IBUS_BAUDRATE` | 115200 | UART 속도 |
| `RX_TIMEOUT_US` | 500 000 | 이보다 오래 올바른 프레임이 없으면 링크 상실 |
| `RX_FAILSAFE_THROTTLE_US` | 950 | 스로틀이 이보다 낮으면 수신기가 송신기의 failsafe를 알리는 것 |

### GPS

| 상수 | 값 | 의미 |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | NAV-PVT가 이보다 오래되면 `UbloxM10_Gps::isAvailable() == false` |

### PWM 범위와 조종면 행정

| 상수 | 값 | 의미 |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | 표준 RC 펄스(µs) |
| `AILERON_MAX_US`, `ELEVATOR_MAX_US`, `RUDDER_MAX_US` | 500 / 500 / 300 | 스틱을 끝까지 움직였을 때 중심에서의 변위(µs). 방향타가 작은 것은 같은 서보에 착륙장치 바퀴가 달려 있기 때문 |
| `THROTTLE_LIMIT_PCT` | 100 | ESC로 가는 스로틀의 상한(%)이며 스틱과 오토파일럿에 똑같이 적용됨(`FlightController::capThrottle`). 출력이 약한 3S1P 배터리로 테스트 벤치 시험을 할 때는 50으로 두었으며, 테스트는 이 값으로 기대 출력을 계산함 |
| `THROTTLE_KNOB_MIN_PCT` | 10 | 노브 `Knob::THROTTLE_LIMIT`: `THROTTLE_LIMIT_PCT` 위에 스로틀 상한 10 … 55 … 100%, 허용 범위를 스틱 전체 행정으로 늘림. 지정 안 함 — 상한 없음 |

### ESC와 모터 역회전

| 상수 | 값 | 의미 |
|---|---|---|
| `ESC_BIDIRECTIONAL` | `true` | 양방향 ESC(Bidirectional/3D 모드의 BLHeli 80A): 스로틀을 중립을 중심으로 한 펄스로 변환(`EscSignal`), “모터 정지”는 1500, 역회전 사용 가능. `false` — 일반 ESC: 1000 = 정지, 역회전 없음 |
| `ESC_NEUTRAL_US` | 1500 | 양방향 ESC의 중립: 모터 정지, 그보다 위는 전진, 아래는 후진 |
| `REVERSE_THROTTLE_LIMIT_PCT` | 50 | 후진 스로틀 상한, %(그리고 `THROTTLE_LIMIT_PCT` 이하): 프로펠러는 거꾸로 돌면 추력이 약하지만 전류는 그대로 다 먹음 |
| `REVERSE_SETTLE_MS` | 3000 | 방향을 바꾼 뒤 이 시간 동안 ARM 거부 — 스로틀을 올린 채 DISARM한 모터가 멈출 시간을 줌 |

### 플랩(플래퍼론)

| 상수 | 값 | 의미 |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6이 이보다 위이면 플랩 펼침(1500이 아닌 이유는 첫 프레임이 오기 전에는 채널이 1500이기 때문) |
| `FLAPS_DEPLOYED_US` | 220 | 각 에일러론을 아래로 내리는 변위(µs)(MG90S 혼으로 약 20°) |
| `FLAPS_TRANSITION_MS` | 1000 | 완전히 펼치거나 접는 데 걸리는 시간 |

### 리버스

모두 `Config.h`의 한 블록에 있습니다. 한 줄은 한 가지만 뒤집고, 나머지는 건드리지 않습니다.

| 플래그 | 기본값 | 무엇을 뒤집는가 |
|---|---|---|
| `AILERON_LEFT_REVERSED` / `AILERON_RIGHT_REVERSED` | `false` / `true` | 에일러론을 하나씩 따로(날개 안 서보는 거울 배치) |
| `ELEVATOR_REVERSED` | `true` | 엘리베이터 |
| `RUDDER_REVERSED` | `false` | 러더와 조향 바퀴(서보 하나) |
| `AUX1_REVERSED` | `false` | 화물 투하 서보: 열림 ↔ 닫힘 |
| `AUX2_REVERSED` | `false` | 카메라 서보 |
| `MOTOR_REVERSED` | `false` | 모터 회전 방향 — 모터 선 두 가닥을 다시 납땜하는 대신; `ESC_BIDIRECTIONAL`일 때만(아니면 빌드 실패) |
| `FLAPS_REVERSED` | `false` | 에일러론의 플랩과 에어브레이크, 롤은 건드리지 않음 |
| `CHANNEL_REVERSED[10]` | `false` ×10 | 조종기 채널 CH1…CH10, 조종기 메뉴의 Reverse처럼 스틱, 스위치, 노브가 반대로 동작. CH3(스로틀)는 불가: 통신 두절을 원시 스로틀 < 950 µs로 판정하기 때문(`static_assert`) |

`ControlMixer`는 물리적 부호로 계산하고 부호는 여기서만 뒤집으므로 스틱과 오토파일럿이 어긋날 수 없습니다. 조종면이 스틱에서도 오토파일럿에서도 반대로 움직이면 출력 플래그, 스틱에서만 반대면 채널 플래그입니다. `Config::directedUs(us, reversed)`는 펄스를 1500을 중심으로 뒤집습니다.

### 센서 장착

| 상수 | 값 | 의미 |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | IMU 칩의 축을 수직축 둘레로 돌리는 각도(0/90/180/270), 즉 칩의 X축이 향하는 방향. NVS에 장착 보정 `o`가 없을 때만 사용 |
| `MAG_ROTATION_CW_DEG` | 0 | 나침반에 대한 같은 설정(나침반에는 장착 보정이 없음) |

### 랜딩기어, 지상 캘리브레이션, 알림

| 상수 | 값 | 의미 |
|---|---|---|
| `LANDING_GEAR` | `LandingGear::TAIL_WHEEL` | 어느 바퀴로 조향하는지: `TAIL_WHEEL`은 꼬리 바퀴(지상에서 기수가 들림, 활주 때 꼬리가 먼저 들림), `NOSE_WHEEL`은 앞바퀴 |
| `CAL_SWITCH` | `Channels::SWB` | 지상 캘리브레이션 동작에 쓰는 스위치(SwD는 불가: 모터 역회전에 쓰임) |
| `CAL_SWITCH_FLIPS` / `CAL_SWITCH_WINDOW_MS` | 3 / 4000 | ARM 없이, 스로틀을 내린 채: 이 시간 안에 이 횟수만큼 내렸다 올리면 캘리브레이션 시작(한 번 더 하면 취소) |
| `CAL_STILL_MS`, `CAL_STILL_DPS`, `CAL_STILL_G` | 2000, 3.0, 0.15 | 이만큼의 ms 동안 각속도가 이 값보다 작고 하중이 1g ± 이 값이면 단계를 기록 |
| `CAL_PARK_WAIT_MS` | 5000 | 2단계는 비행기를 옮길 때까지, 또는 이 시간만큼 기다림(앞바퀴: 같은 자세) |
| `CAL_STEP_TIMEOUT_MS` | 60 000 | 이 시간 안에 정지하지 않으면 캘리브레이션 취소(오류) |
| `CAL_TAIL_PARK_MIN/MAX_DEG` | 3 / 25 | 꼬리 바퀴: 주기 피치가 이 범위 안이어야 하며, 아니면 오류(“꼬리를 내리지 않음”) |
| `CAL_NOSE_PARK_MAX_DEG`, `CAL_PARK_MAX_ROLL_DEG` | 6, 10 | 앞바퀴: 주기 자세가 거의 수평. 주기 롤은 최대 이 값 |
| `SIGNAL_MOTOR` / `SIGNAL_SURFACES` | `true` / `true` | 모터 알림(ESC 펄스를 멈춤: ESC가 준비 완료 멜로디를 냄)과 조종면 끄덕임 알림 |
| `SIGNAL_NOD_US`, `SIGNAL_NOD_MS` | 200, 150 | 승강타와 방향타의 끄덕임: 이만큼의 µs 위로, 다음에 아래로, 각각 이만큼의 ms |
| `SIGNAL_ESC_PAUSE_MS`, `SIGNAL_PERIOD_MS` | 1000, 3000 | ESC로 펄스가 가지 않는 시간. 알림은 이 주기로 이어짐(멜로디가 끝까지 나옴) |

### ARM

| 상수 | 값 | 의미 |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5가 이보다 위이면 ARM 스위치가 켜짐 |
| `THROTTLE_LOW_US` | 1050 | 스로틀이 이보다 낮으면 “스로틀 최저”로 보아 ARM 가능 |

### Failsafe

| 상수 | 값 | 의미 |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | 조종면 중립 |
| `FAILSAFE_THROTTLE` | 1000 | 모터 정지 |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | 공중에서 링크를 잃었을 때 활공의 뱅크 |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | 활공의 피치(수평선보다 약간 아래) |
| `FAILSAFE_RTH` | `true` | GPS와 홈이 있으면 공중에서의 링크 상실 시 활공 대신 모터를 쓰며 홈으로 복귀 |

### 스위치, 피토관, 오토파일럿

모든 모드와 기능의 수치는 `Config.h`에 자세한 주석과 함께 있습니다.
조종자에게 어떤 의미인지는 [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md)를 참고하세요.

| 그룹 | 상수 |
|---|---|
| 스위치 | `SWITCH_ON_US` = 1750(채널이 이보다 위이면 스위치 켜짐. 1500이 아닌 이유는 첫 프레임 전에 아무것도 켜지지 않게 하기 위해서) |
| 피토관 | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| 안정화 | `MAX_BANK_DEG` 45(노브 15…60), `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| 내비게이션 | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| 고도 | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| 스로틀과 속도 | `CRUISE_THROTTLE_PCT` 55(30…85), `CRUISE_AIRSPEED_MS` 14(10…22), `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| 실속 | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| 선회 원과 홈 | `LOITER_RADIUS_M` 50(25…150), `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| 지오펜스 | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| 손 발사 | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| 자동 이륙 | `TAKEOFF_TRIGGER_US` 1500. 점검: `TAKEOFF_CHECK_MS` 1000, `_CHECK_TIMEOUT_MS` 5000, `TAKEOFF_PARK_PITCH/ROLL_TOL_DEG` 4/5, `TAKEOFF_UNCALIBRATED_TILT_DEG` 10. 스로틀 `TAKEOFF_THROTTLE_PCT` 100을 `TAKEOFF_THROTTLE_RAMP_MS` 2000에 걸쳐. 침로 `TAKEOFF_HEADING_KP` 10, `TAKEOFF_YAW_RATE_KD` 2. `TAKEOFF_ROTATE_SPEED_MS` 11. 부양 `TAKEOFF_LIFTOFF_MS` 3000, `_LIFTOFF_ALT_M` 1, `_LIFTOFF_CONFIRM_MS` 500. 상승 `TAKEOFF_CLIMB_PITCH_DEG` 8로 `TAKEOFF_SAFE_ALTITUDE_M` 30까지(기압계 없으면 `TAKEOFF_CLIMB_MS` 15 000). 중단: `TAKEOFF_ABORT_ROLL_DEG` 15, `_ABORT_HEADING_DEG` 30, `TAKEOFF_NOSE_DOWN_MS` 300, `TAKEOFF_GROUND_TIMEOUT_MS` 15 000. “자이로만”: `TAKEOFF_GYRO_ONLY_CLIMB/MAX/ABORT_MS` 5000/30 000/3000 |
| 꼬리 바퀴 | `TD_TAIL_UP_ELEVATOR_US` 250(아래 타각의 상한), `TD_ROLL_PITCH_DEG` 1, `TD_PITCH_KP` 40(꼬리 들기), `TD_ROLL_PITCH_KP` 15(활주와 기수 들기), `TD_PITCH_KD` 3, `TD_TAIL_UP_DONE_DEG` 2, `TD_TAIL_UP_TIMEOUT_MS` 6000, `TD_ROLL_MS` 2500, `TD_ROTATE_RATE_DPS` 2, `TD_TAIL_CLEARANCE_DEG` 3, `TD_MIN_PITCH_DEG` −3 |
| 앞바퀴 | `TRI_ROLL_ELEVATOR_US` 0, `TRI_ROLL_MS` 3000, `TRI_ROTATE_RATE_DPS` 5, `TRI_MAX_GROUND_PITCH_DEG` 10 |
| 착륙 | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| 소어링 | `SOAR_*`: 활공 −3°, 열상승 > 0.5 m/s가 1.5초, 선회 25°, 이탈 < −0.2 m/s가 8초, 30 m 아래에서는 100 m까지 모터 사용, 400 m를 넘으면 홈으로 |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| 자동 트림 | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, 지상에서 저장: `AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| 협조 | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| 기능 | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### 루프, Wi-Fi, 디버깅

| 상수 | 값 | 의미 |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | 비행 루프의 주기(500 Hz). `PidController`의 공칭 `dt`이기도 함 |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | 대시보드의 액세스 포인트(비밀번호가 약함 — 테스트 벤치용 도구) |
| `WEB_SERVER_PORT` | 80 | HTTP 포트 |
| `TELEM_BAUDRATE` | 57600 | MAVLink 무선 모뎀의 속도(SiK 기본값) |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | MAVLink에서의 기체 주소 |
| `DEBUG_INTERVAL_MS` | 100 | `DebugLogger`가 로그 채널을 확인하는 빈도 |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | “변경 시” 모드에서 RC/PWM 채터링의 허용치 |

### 블랙박스

자세한 내용은 [BLACKBOX.md](../BLACKBOX.md)를 참고하세요.

| 상수 | 값 | 의미 |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 MB / 32 KB | PSRAM에 있는 기록 큐(PSRAM이 없으면 내부 메모리에) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32H743: RAM의 큐 — 10초 분량의 사전 기록과 카드 지연에 대비한 여유 |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 MB | STM32H743: SD 카드의 파일과 사용하는 부분의 상한(전원을 켤 때의 대조 시간은 영역과 함께 길어짐) |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | 시작(ARM + 스로틀) 전과 DISARM 후의 기록 시간 |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | ARM 상태에서 모터가 멈추고 기체가 이 시간 동안 정지해 있으면 중지 |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | “정지”로 간주하는 조건 |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | 비정상 재부팅 뒤에는 최소 이 시간 동안 기록 |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | 항상 준비해 두는 지워진 공간. 오래된 비행은 지상에서 통째로 지움 |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | 지우기 사이의 휴지 |
| `BLACKBOX_IMU_DIVIDER` | 1 | IMU를 N 주기에 한 번 기록(1이면 500 Hz) |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | 비행 컨트롤러 보드에서 배터리(56k/10k)와 전류 센서(10k/15k)의 분압비 |

---

## namespace `Channels`

**파일:** `include/config/Channels.h` · **의존:** `<stdint.h>`

물리적인 채널 번호와 용도를 연결하는 유일한 장소입니다. 값은 `RcChannelState`에서의
**인덱스**(0부터 시작)입니다.

| 상수 | 인덱스 | 채널 | FS-i6의 조작부 | 용도 |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | 오른쪽 스틱 ←→ | 롤 |
| `ELEVATOR` | 1 | CH2 | 오른쪽 스틱 ↑↓ | 피치(2000 = 몸에서 멀어지는 방향 = 기수 아래) |
| `THROTTLE` | 2 | CH3 | 왼쪽 스틱 ↑↓ | 스로틀 |
| `RUDDER` | 3 | CH4 | 왼쪽 스틱 ←→ | 방향타 + 바퀴 |
| `ARM` | 4 | CH5 | SwA | ARM 스위치(재할당 불가) |
| `SWB` | 5 | CH6 | SwB | `Controls.h` 표에 따름(기본값은 플랩) |
| `SWC` | 6 | CH7 | SwC(3단) | 기본값은 모드 MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | 기본값은 RTH |
| `VRA` | 8 | CH9 | VrA | 기본값은 `STAB_GAIN` |
| `VRB` | 9 | CH10 | VrB | 기본값은 `CRUISE_SPEED` |
| `COUNT` | 10 | | | 채널 수 |

---

## namespace `Controls`

**파일:** `include/config/Controls.h` · **의존:** `ControlBinding.h`, `Channels`

`constexpr Binding BINDINGS[]`는 각 스위치와 노브가 하는 일을 **채널마다 한 줄**로 적은
것입니다(`Bind::modes/mode/feature/knob`, [autopilot.md](autopilot.md#binding-bind-bindingcheck)
참고). 옆에는 주석 처리된, 바로 쓸 수 있는 아이디어가 있습니다. 세 개의 `static_assert`가
빌드 시점에 표의 오류를 잡아냅니다: 표 안의 스틱이나 ARM, 범위를 벗어난 채널, 중복된
채널, 모드 선택 스위치가 둘 이상인 경우.
