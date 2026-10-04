# DEVELOPER_GUIDE.md — OpenPlaneProject 개발자 가이드

> 🌐 이 문서는 [러시아어 원문](../../DEVELOPER_GUIDE.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다. 이 번역은 AI가 작성했으며 원어민의 검수를 거치지 않았습니다. 오류를 발견하면 [Damir Lebedev](https://github.com/damir-lebedev)에게 알려 주시거나 [이슈](https://github.com/damir-lebedev/OpenPlaneProject/issues)로 남겨 주세요.

펌웨어의 기술 지도입니다. 어느 파일이 무엇을 맡는지, 수신기와 센서에서 서보까지 데이터가 어떻게 흐르는지, 사슬 전체를 묶는 부호 규약이 무엇인지, 웹 API가 어떻게 구성되어 있는지, 프로젝트를 어떻게 확장하는지를 다룹니다. C++를 쓰며 이 저장소(`main` 브랜치)에서 빠르게 방향을 잡고 싶은 개발자를 위한 문서이며, 언어나 PlatformIO의 기초를 배우는 용도가 아닙니다.

프로젝트 개요와 시제품 상태는 [`../README.md`](README.md), 무엇을 어디에 연결하고 어떻게 날리는지는 [`PILOT_GUIDE.md`](PILOT_GUIDE.md), 계획은 [`ROADMAP.md`](ROADMAP.md)에 있습니다. 여기에는 코드만 있습니다. 아키텍처 전체(계층, 태스크, 상태 머신)는 [`ARCHITECTURE.md`](ARCHITECTURE.md), 각 클래스의 참조는 [`reference/`](reference/README.md), 테스트는 [`TESTING.md`](TESTING.md)에 있습니다.

> 프로젝트는 활발히 개발 중입니다. ESP32-S3 벤치는 조립되어 모든 센서로 확인했지만, **오토파일럿은 아직 비행에서 시험하지 않았습니다.** 특정 모듈과 관련된 곳에는 그 점을 표시해 두었습니다. 코드가 무엇을 하는지 의심스럽다면 문서가 아니라 소스를 다시 읽으세요.

---

## 목차

1. [계층 아키텍처](#계층-아키텍처)
2. [FreeRTOS 태스크와 제어 루프](#freertos-태스크와-제어-루프)
3. [파일 참조](#파일-참조)
4. [부호 규약: IMU에서 서보까지](#부호-규약-imu에서-서보까지)
5. [RC 채널 배치, ARM, failsafe](#rc-채널-배치-arm-failsafe)
6. [센서 데이터](#센서-데이터)
7. [FlightController::update() 상세](#flightcontrollerupdate-상세)
8. [웹 대시보드의 HTTP API](#웹-대시보드의-http-api)
9. [콘솔과 진단](#콘솔과-진단)
10. [보드 선택과 핀 배치](#보드-선택과-핀-배치)
11. [새 센서를 추가하는 방법](#새-센서를-추가하는-방법)
12. [새 오토파일럿 모드를 추가하는 방법](#새-오토파일럿-모드를-추가하는-방법)
13. [피드백(기초 작업, 연결되지 않음)](#피드백기초-작업-연결되지-않음)
14. [새 보드를 추가하는 방법](#새-보드를-추가하는-방법)
15. [빌드, 업로드, 모니터 명령](#빌드-업로드-모니터-명령)
16. [알려진 제한 사항](#알려진-제한-사항)
17. [변경 사항을 반영하는 방법](#변경-사항을-반영하는-방법)

---

## 계층 아키텍처

거의 모든 클래스는 `include/<계층>/` 폴더로 나눈 헤더 안에 있습니다. 각 헤더는 자신이 쓰는 것을 스스로 포함합니다(`#include "config/Config.h"`, `"hal/II2CBus.h"` 등. 경로는 `include/` 기준). `src/main.cpp`는 유일한 조립 지점(composition root)으로, 모든 객체를 만들어 서로 연결하고 `setup()`/`loop()`를 돌립니다. 의존성은 한 방향입니다. 하위 계층은 상위 계층에 대해 아무것도 모릅니다.

```
include/
├── config/      Config.h (핀, 모든 설정), Channels.h (채널 이름),
│                Controls.h (각 스위치가 하는 일(채널마다 한 줄))
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (I2C/SPI 위의 레지스터 장치), Rtos
│   ├── esp32/   Esp32Board + Wire/SPI/HardwareSerial/LEDC 래퍼
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (NVS 대신 플래시에 저장하는 설정)
├── storage/     KeyValueStore, KvPreferences — NVS 없는 설정 저장소
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  피드백의 기초 작업: 연결되지 않음(아래 절 참고)
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (기압계 두 개로 만든 피토관)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — ESP32 펌웨어(S3, C3, 38핀)
src/stm32/main.cpp  — STM32H743 펌웨어(FreeRTOS 태스크, MAVLink)
```

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION  src/main.cpp / src/stm32/main.cpp — 객체 조립            │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION  control/FlightController — 주기당 연산 순서              │
│ TELEMETRY     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ CONTROL             │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12개 모드  │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ 항법, PID          │  │ RcChannelState        │
│ ThrottleManager     │  │ PilotSwitches         │  │ RcInput               │
│ ArmingManager       │  └──────────┬────────────┘  └───────────────────────┘
│ FlightOutputs       │             │ ImuSensor* / BarometerSensor* / ...
└─────────┬───────────┘             ▼
          │           ┌─────────────────────────────────────────────────┐
          │           │ SENSORS                                          │
          │           │ ImuSensorBase ── MPU6050, ICM42688, LSM6DSV,     │
          │           │   └ AttitudeEstimator     ICM45686               │
          │           │ BarometerBase ── BMP388, BME280, SPL06, BMP581   │
          │           │ MagnetometerBase ── QMC5883P / L, QMC6309        │
          │           │ UbloxM10_Gps, PitotDualBaroAirspeed              │
          │           └──────────────────────┬──────────────────────────┘
          ▼                                  ▼ IRegisterDevice / IUartPort
┌───────────────────────────────────────────────────────────────────────┐
│ HAL  IBoard / II2CBus / ISpiBus / IUartPort / IServoOutput             │
│      RegisterDevice: I2cRegisterDevice, SpiRegisterDevice              │
│      esp32/Esp32Board — Wire, Wire1, SPI, HardwareSerial, LEDC         │
│      stm32/Stm32Board — Wire, I2C1, SPI, Uart, HardwareTimer, 플래시   │
└───────────────────────────────────────────────────────────────────────┘
```

아키텍처를 깔끔하게 유지하는 규칙:

- **HAL**은 특정 MCU를 알아도 되는 유일한 계층입니다(`Wire`, `SPI`, `HardwareSerial`, `ledc*`). 그 위의 모든 것은 인터페이스만 다룹니다. 다른 MCU로 옮기는 일은 새 `hal/<mcu>/<Mcu>Board.h`를 만드는 것이며 나머지 코드는 바뀌지 않습니다(예: STM32H743용 `hal/stm32/`).
- **센서 드라이버는 버스를 모릅니다.** `IRegisterDevice&`를 받으며, 주소가 있는 I2C 장치나 CS가 있는 SPI 장치는 `SensorSelection.h`에서 만듭니다. 같은 `BMP388_Sensor`가 I2C와 SPI 모두에서 동작합니다.
- **공통 부분은 기반 클래스에 있습니다.** 보정, 축 회전, 부호, 자세 필터, 고도와 수직 속도, 나침반 보정의 저장, 버스 오류 계수는 `ImuSensorBase`/`BarometerBase`/`MagnetometerBase`에 있습니다. 칩 드라이버에는 데이터시트의 레지스터와 공식만 있습니다.
- **RC와 Outputs**는 비행기에 대해 아무것도 모릅니다. iBUS 바이트 → 채널, PWM 값 → 출력입니다.
- **Control과 Autopilot**은 데이터에 대한 로직이며 UART도 PWM도 Wi-Fi도 없습니다. 시간이 필요한 곳(플랩)에서는 매개변수로 넘깁니다.
- **Coordination**(`FlightController`)은 여러 하위 계층을 한꺼번에 보고 연산 순서를 정하는 유일한 클래스입니다.
- **Application**(`main.cpp`)은 `Esp32Board`, 장치, 센서를 만들고 모든 것을 DI 프레임워크 없이 손으로 연결하는 유일한 곳입니다.

---

## FreeRTOS 태스크와 제어 루프

| 위치 | 내용 | 주기 |
|---|---|---|
| 코어 1, `loop()`(Arduino의 loopTask) | `applyPendingCommands()` → `FlightController::update()` → `DebugLogger::update()` → `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms(500 Hz), `vTaskDelayUntil` |
| 코어 0, `web` 태스크 | `WebServer::handleClient()` | 2 ms마다 |
| 코어 0, `oled` 태스크 | 두 번째 I2C 버스로 SSD1306 그리기 | 200 ms |
| 코어 0 | ESP-IDF의 Wi-Fi 스택 | — |

- 루프 주기는 작업 뒤의 `delay(2)`가 아니라 `vTaskDelayUntil`로 유지합니다. 그래서 주파수는 한 주기가 얼마나 걸렸는지에 좌우되지 않습니다. 긴 블로킹(콘솔에서 하는 보정) 뒤에는 계산을 처음부터 다시 시작하며, 놓친 주기를 한꺼번에 따라잡지 않습니다.
- 벤치(ESP32-S3, 모든 센서)에서는 500 Hz로 돌며, 한 주기당 작업은 평균 약 0.7 ms, 최악의 주기는 약 1.4 ms입니다. 이 값은 10초마다 `SYS:` 줄로 출력됩니다.
- I2C 트랜잭션 타임아웃은 5 ms입니다(Wire의 기본값은 50 ms). 노이즈로 멈춘 트랜잭션이 루프를 오래 막지 않습니다.
- **태스크 사이의 데이터 분리.** 웹과 OLED는 상태(`FlightController`/`Autopilot`/`LoopStats`)를 *읽기만* 합니다. 이들은 개별 16/32비트 필드이므로 최악의 경우에도 이웃한 주기의 값이 보일 뿐입니다. 대시보드의 *명령*(`setmode`/`setpid`)은 웹 태스크에서 직접 적용하지 않습니다. `portMUX` 아래에서 “우편함”에 넣어 두면 비행 루프가 `WebDebugServer::applyPendingCommands()`에서 가져갑니다.
- `Serial`(UART0 → CH343 브리지 → “COM” 커넥터)은 4 KB 송신 버퍼를 가집니다. 디버그 프레임(약 600자)이 전송되는 동안에도 루프를 막지 않습니다.

---

## 파일 참조

### `config/`

| 파일 | 담당 |
|---|---|
| `Config.h` | 모든 핀(보드별로 한 블록: `BOARD_ESP32_S3/C3/CLASSIC`, `BOARD_STM32H743`)과 설정: iBUS와 신호 상실, 조종면 작동 범위, 플랩, 서보 반전, IMU와 나침반 장착, ARM, failsafe(RTH 또는 활공), 피토관(`PITOT_*`), 오토파일럿 모드와 기능의 모든 수치, 제어 루프, Wi-Fi, MAVLink, 디버깅 |
| `Channels.h` | 채널 이름: `AILERON`, `ELEVATOR`, `THROTTLE`, `RUDDER`, `ARM`, `SWB`, `SWC`, `SWD`, `VRA`, `VRB` |
| `Controls.h` | `BINDINGS` 표: 각 스위치와 노브가 하는 일을 채널당 한 줄로 적고 `static_assert`로 검사 |

### `hal/`

| 파일 | 담당 |
|---|---|
| `IBoard.h` | 하드웨어로 들어가는 진입점: `i2c()`, `displayI2c()`(화면용 두 번째 버스, `nullptr`일 수 있음), `spi()`, `rcUart()`, `gpsUart()`, `telemetryUart()`(MAVLink, `nullptr`일 수 있음), `servo(ServoChannel::*)`(AUX1/AUX2를 포함한 7개 출력), `setBuzzer()` |
| `Rtos.h` | FreeRTOS 태스크. ESP32(코어 0)와 STM32(우선순위)에서 같은 방식으로 쓰며, 남은 힙 확인 |
| `II2CBus.h` | I2C 버스: `Wire` 형태의 기본 동작 + 보조 함수 `writeRegister()`, `readRegisters()`(정확히 `count` 바이트가 도착했는지 확인), `readRegister()`, `probe()` |
| `ISpiBus.h`, `IUartPort.h`, `IServoOutput.h` | SPI, UART, PWM 출력 하나(`measurePulseUs()` — 실제 펄스 진단) |
| `RegisterDevice.h` | `IRegisterDevice` — “8비트 레지스터 묶음”; `I2cRegisterDevice`(주소), `SpiRegisterDevice`(CS, 주파수, 데이터 앞의 더미 바이트) |
| `esp32/Esp32Board.h` | `IBoard`의 구현: `Wire`(센서), `Wire1`(화면, 칩에 I2C 컨트롤러가 두 개 있을 때), `SPI`, `HardwareSerial` 두 개, LEDC 채널 5개 |
| `esp32/Esp32I2CBus.h` | 임의의 `TwoWire` 위에 얹는 `II2CBus`, 타임아웃 5 ms |
| `esp32/Esp32ServoOutput.h` | LEDC를 통한 PWM: 50 Hz, 14비트. 핀이 −1이면 해당 출력은 배선되지 않은 것. ESP32Servo 라이브러리는 쓰지 않음 — [제약 사항](#알려진-제한-사항) 참고 |
| `esp32/Esp32SpiBus.h`, `esp32/Esp32UartPort.h` | `SPI`와 `HardwareSerial` 위의 얇은 래퍼 |
| `stm32/*` | STM32H743: `Stm32Board`(+ 무선 모뎀용 UART4), 각 버스, PWM 타이머, `Stm32FlashStorage`(설정을 플래시 섹터 하나에 저장하며 백그라운드 태스크가 기록), `compat/Preferences.h` |

### `storage/`

| 파일 | 담당 |
|---|---|
| `KeyValueStore.h` | 임의의 저장 매체(`IFlashStorage`) 위에서 RAM에 올려 두는 CRC32 포함 “네임스페이스/키 → 바이트” 이미지. 같은 값은 다시 쓰지 않음 |
| `KvPreferences.h` | `KeyValueStore` 위에서 동작하는 ESP32의 `Preferences` API |

### `rc/`

| 파일 | 담당 |
|---|---|
| `RcChannelState.h` | 10개 채널의 스냅샷 |
| `RcInput.h` | `clamp()`, `centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS → 채널: 32바이트 프레임, CRC, 채널 값은 하위 12비트(`& 0x0FFF`). `isSignalLost()` = 프레임이 없음(또는 아직 하나도 오지 않음) ∥ 스로틀의 failsafe 값. 프레임 카운터 |

### `control/`

| 파일 | 담당 |
|---|---|
| `ControlCommand.h` | 물리적 부호로 나타낸 조종면 명령 — 스틱, 오토파일럿, 믹서가 함께 쓰는 공통 언어 |
| `ControlMixer.h` | `fromSticks(rc)` → `ControlCommand`; `updateFlaps(목표, now)`; `mix(command)` → 서보 반전이 적용된 PWM; 플래퍼론: 에일러론은 `flaps ± roll`(마이너스는 에어브레이크) |
| `FlapsController.h` | 플랩을 부드럽게 내리고 올림. 시간은 매개변수로 전달 |
| `ThrottleManager.h` | 스틱에서 오는 스로틀. 신호를 잃으면 `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | SwA 스위치로 ARM(스로틀을 맨 아래에 둔 상태에서 OFF→ON 전환 + 해당 모드에 필요한 센서 점검), DISARM은 즉시 |
| `FlightOutputState.h` | 원하는 PWM: `aileronLeft`, `aileronRight`, `elevator`, `rudder`, `throttle`, `aux1`(페이로드), `aux2`(카메라) |
| `Beeper.h` | 부저: `BEEPER` 기능에 의해, 또는 지상에서 “기체 분실” 시 울림 |
| `FlightOutputs.h` | 출력 표(`outputInfo()`: 키, 이름, 핀, 필수 여부, 상태 필드)와 그 위에서 루프로 처리하는 모든 것: `begin()`, `write()`, `setFailsafe()`, 상태, `printPulseSelfTest()` |
| `FlightController.h` | 주기마다 작업을 수행하는 순서, 신호 상실(`applyLinkLoss()`), 텔레메트리용 게터 |

### `autopilot/`

| 파일 | 담당 |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode`(12개 모드), `Feature`, `Knob`, `PilotInputs`, 이름 |
| `ControlBinding.h` | `Binding`, 팩토리 `Bind::modes/mode/feature/knob`, `BindingCheck` 검사 |
| `PilotSwitches.h` | 할당 표 → 주기마다의 모드, 기능, 노브. 전원을 켤 때의 배치 |
| `Autopilot.h` | 12개 모드, failsafe RTH/활공, 지오펜스, 홈 포인트, 선회 협조, 자동 트림. `update(armed, linkLost, 스로틀, 스틱)` → `getCommand()`, `applyThrottle()` |
| `Navigation.h` | `Geo`(거리, 방위, 오프셋), `Guidance`(침로를 향한 롤, 원 궤도의 벡터장) |
| `AltitudeSpeedController.h` | 고도는 피치로, 대기속도는 스로틀로 제어(TECS-lite) |
| `LaunchController.h`, `SoaringController.h` | 손 발사와 소어링의 상태 기계 |
| `AutoTrim.h` | 자동 트림. NVS/플래시에 저장 |
| `PidController.h` | PID: D 항은 센서의 변화율(자이로스코프, 승강계)에서 가져오고, anti-windup 적용. ARM하지 않은 동안 적분기는 고정 |
| `feedback/*` | **준비 단계, 미연결**: 적응형 피드백, 이륙과 착륙 — [피드백](#피드백기초-작업-연결되지-않음) 참고 |

### `sensors/`

| 파일 | 담당 |
|---|---|
| `SensorInterface.h` | `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` 인터페이스와 데이터 구조체 |
| `SensorSelection.h` | 어떤 칩을 컴파일할지(`#define SENSOR_*`, 빌드 플래그로 덮어쓸 수 있음)와 어느 버스에 연결되어 있는지(`SELECTED_*_DEVICE(board)`) |
| `SensorMounting.h` | 칩의 축을 기체의 축으로 회전(시계 방향 0/90/180/270°) — 나침반용, 그리고 장착 보정이 없는 IMU용 |
| `imu/ImuOrientation.h` | “칩 축 → 기체 축” 행렬로 나타낸 IMU 장착: `IMU_ROTATION_CW_DEG`에서, 또는 세 가지 자세(수평, 기수 위, 오른쪽 날개 아래)에서 검증과 함께 얻음. NVS에 저장 |
| `imu/ImuSensorBase.h` | IMU의 공통 부분: 자이로 보정 + 비행 전 점검(정지 상태, 1g, “위쪽”이 장착과 일치), 장착 보정(`calibrateOrientation()`), 스케일, 회전, 항공 부호, 버스 오류 |
| `imu/AttitudeEstimator.h` | 롤/피치 상보 필터, 요 적분 |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500(칩은 WHO_AM_I로 구분): ±2000°/s, ±16g, DLPF 약 41 Hz, 1 kHz. **테스트 벤치에서 확인** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P: ±2000°/s, ±16g, 1 kHz, UI 필터 50 Hz. 실제 하드웨어에서는 확인하지 않음 |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X: ±2000°/s, ±16g, 960 Hz, LPF1/LPF2. I2C 0x6A/0x6B 또는 SPI. 실제 하드웨어에서는 확인하지 않음 |
| `imu/ICM45686_Sensor.h` | ICM-45686: ±2000°/s, ±16g, 1.6 kHz, 간접 레지스터 IPREG를 통한 저역 통과 필터. I2C 0x68/0x69 또는 SPI. 실제 하드웨어에서는 확인하지 않음 |
| `baro/BarometerBase.h` | 기압계의 공통 부분: 새 샘플만 폴링, 고도, 저역 통과 필터를 거친 수직 속도, 기준값 보정, 오류 |
| `baro/BMP388_Sensor.h` | I2C 또는 SPI(SPI 더미 바이트 포함)로 연결하는 BMP388, Bosch 보정, 데이터 준비 플래그로 읽기. **테스트 벤치에서 확인(I2C)** |
| `baro/BME280_Sensor.h` | BME280/BMP280, Bosch 보정 §8.1. 실제 하드웨어에서는 확인하지 않음 |
| `baro/SPL06_Sensor.h` | SPL06-001: 데이터시트의 계수와 수식, 32 Hz ×16. I2C 0x76/0x77 또는 SPI. 실제 하드웨어에서는 확인하지 않음 |
| `baro/BMP581_Sensor.h` | BMP581: BMP5_SensorAPI의 절차, 16×/2×, IIR. I2C 0x46/0x47 또는 SPI. 주 기압계로도 피토관으로도 쓸 수 있음. 실제 하드웨어에서는 확인하지 않음 |
| `mag/MagnetometerBase.h` | 나침반의 공통 부분: 50 Hz 폴링, NVS의 hard-iron 보정, 축 회전, 방위, 오류 |
| `mag/QMC5883P_Sensor.h` | QMC5883P, 0x2C. **테스트 벤치에서 확인** |
| `mag/QMC5883L_Sensor.h` | QMC5883L, 0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309, 0x7C: ±8 G, 200 Hz. 실제 하드웨어에서는 확인하지 않음 |
| `gps/UbloxM10_Gps.h` | u-blox M10: CFG-VALSET으로 설정(115200 보, 10 Hz, NAV-PVT, NMEA 없음), NAV-PVT 해석. 테스트 벤치에는 연결하지 않음 |
| `airspeed/AirspeedSensor.h` | 대기속도 센서의 인터페이스: 차압, IAS, TAS, 밀도 |
| `airspeed/PitotDualBaroAirspeed.h` | 직접 만든 피토관: 관 속의 BMP581 + 동체의 기압계. 지상에서 영점 설정, 저역 통과 필터, 정압으로 밀도 계산, 고장 감지 |

### `telemetry/`와 애플리케이션

| 파일 | 담당 |
|---|---|
| `DebugLogger.h` | 채널별 로그(`LogSettings.h`): 채널마다 자기 줄, 자기 채터링 허용치와 모드를 가짐. 메뉴가 열려 있는 동안에는 출력하지 않음 |
| `DebugConsole.h` | 포트 모니터의 텍스트 메뉴(`h`)와 단축키(`l`/스페이스/`s`/`i`/`o`/`m`/`p`/`b`). 로그 설정은 메뉴를 나갈 때 NVS에 기록하며, ARM하지 않았을 때만 기록 |
| `LogSettings.h` | 로그 채널(STAT, RC, OUT, ATT, AP, ALT, MAG, GPS, IMU, NAV, SYS)과 모드: 끔 / 변경 시 / 상시. NVS에 저장 |
| `WebDebugServer.h` | 액세스 포인트, 라우트, JSON `/api/status`, 명령 우편함. 코어 0에서 도는 전용 태스크 |
| `WebDashboardPage.h` | 대시보드의 HTML/JS를 하나의 리터럴로 담은 것. 채널/출력/센서 행은 브라우저가 JSON으로 만든다 |
| `OledDisplay.h` | `II2CBus` 위에서 U8g2를 통해 동작하는 SSD1306. 전용 태스크(`Rtos`) |
| `MavlinkCodec.h`, `MavlinkTelemetry.h` | QGroundControl / Mission Planner용 MAVLink 2: 프레임, 스트림, PID 파라미터, 지상에서의 모드 전환 |
| `LoopStats.h` | 초당 주파수, 주기의 평균 시간과 최악 시간(OLED), 그리고 마지막으로 읽은 뒤의 최악 값(`takePeakUs()`, SYS 줄) |
| `src/main.cpp` | ESP32: 객체 생성, `setup()`, `vTaskDelayUntil`을 쓰는 `loop()` |
| `src/stm32/main.cpp` | STM32H743: 같은 객체들, MAVLink, SD 카드 블랙박스, `flight`/`storage`/`oled`/`bbox` 태스크 |
| `src/stm32/sd_msp.cpp`, `src/stm32/bootloader.cpp` | STM32H743: `HAL_SD_Init`을 위한 SDMMC1의 핀과 클럭. 콘솔의 `D` 키는 USB DFU 부트로더로 재부팅 |

---

## 부호 규약: IMU에서 서보까지

경로 전체에서 부호 체계를 하나로 통일합니다. 그래서 스틱과 오토파일럿은 항상
같은 방향으로 조종면을 움직이고, 각 서보의 방향은 정확히 한 곳에서만 정해집니다.

**1. 센서 축 → 기체 축.** `ImuSensorBase`는 `ImuOrientation` 행렬
(body = R · chip)로 칩의 축을 기체 축으로 회전시킵니다. X는 기수 방향, Y는
왼쪽, Z는 위쪽입니다. 행렬은 다음 중 하나에서 얻습니다.

- **장착 보정**(명령 `o`, NVS에 저장): 보드는 어떤 방향으로 놓여 있어도
  됩니다. 자세는 세 가지로, “수평”은 Z축을 주고(수평선도 함께 정해짐 — 가속도계의
  영점 오프셋도 여기에 포함됨), “기수 위”는 X축을 주며(“위쪽” 중 Z에 수직인
  부분), “오른쪽 날개 아래”는 Y축을 줍니다. 2단계의 기수와 3단계의
  기수(Y × Z)는 약 25° 이내로 일치해야 하며, 그렇지 않으면 조종자가 엉뚱한
  쪽으로 기울인 것이므로 보정이 거부됩니다. 최종 결과는 두 추정값의 평균입니다.
  무작위 장착 300가지로 검증했습니다(`test/test_imu_orientation`, 오차 < 0.1°).
- 그렇지 않으면 `Config::IMU_ROTATION_CW_DEG`에서 얻습니다(칩이 위를 향하도록 둔
  보드. 값은 기수가 “12시” 방향일 때 *칩*의 X축이 어디를 향하는지를 나타냄).
  수평선은 전원을 켠 순간의 자세입니다.

자이로를 보정할 때마다(전원 투입, `i`) **비행 전 점검**을 합니다. 자이로
잡음 < 0.5 °/s(정지 상태. 정지 시 약 0.08), |a| ≈ 1g, “위쪽”이 저장된
값에서 45° 이내(보드를 옮기지 않았음). 통과하지 못하면
`ImuSensor::getPreflightProblem()` ≠ nullptr이 되어 `ArmingManager`는
안정화 모드를 ARM하지 않고, `Autopilot::imuReady()` = false가 됩니다(링크
상실 시 활공을 포함해 모든 모드에서 보정량이 0).

> 현재의 GY-521(MPU6500 복제품)에서는 칩이 인쇄된 화살표에 대해 90° 돌려진
> 채로 납땜되어 있습니다. 실크스크린의 X 화살표 = 칩의 Y축입니다. 그래서 장착
> 보정이 없을 때는 `IMU_ROTATION_CW_DEG = 90`입니다. 배치를 바꾼 뒤의 확인:
> 기수 위 → P가 양의 방향으로 증가, 오른쪽 날개 아래 → R이 양의 방향으로 증가.

**2. 각도와 각속도(`ImuData`) — 항공 부호:**

| 양 | “+”의 의미 |
|---|---|
| `roll`, `gyroX` | 오른쪽 날개 아래 |
| `pitch`, `gyroY` | 기수 위 |
| `yaw`, `gyroZ` | 기수 오른쪽(위에서 보아 시계 방향) |

**3. 명령(`ControlCommand`, 조종면 변위를 µs로 표시, ±500 = 최대 행정):**

| 필드 | “+”의 의미 | 스틱에서 |
|---|---|---|
| `roll` | 오른쪽 롤(오른쪽 에일러론 위, 왼쪽 아래) | CH1: 2000 = 오른쪽 |
| `pitch` | 기수 위(승강타 위) | CH2는 부호가 반대: 2000 = 몸에서 멀어지는 방향 = 기수 아래 |
| `yaw` | 기수 오른쪽(방향타와 앞바퀴가 오른쪽) | CH4: 2000 = 오른쪽 |
| `flaps` | 플랩 아래(두 에일러론 모두 아래) | SwB(CH6): 0 또는 `FLAPS_DEPLOYED_US`, `FLAPS_TRANSITION_MS` 동안 부드럽게 |

PID는 `오차 = 목표 − 실제`를 계산합니다. 오른쪽 롤(roll > 0) → 롤 명령이
음수 → 기체가 수평으로 돌아옵니다. 오토파일럿의 보정값은 믹서 **앞에서**
스틱 명령에 같은 부호로 더해집니다.

**4. 명령 → PWM.** `ControlMixer::mix()`는 각 조종면의 뒷전 변위를 계산하고
(에일러론: 아래 = “+”, 왼쪽 = `flaps + roll`, 오른쪽 = `flaps − roll`,
승강타: 위 = “+”, 방향타: 오른쪽 = “+”) PWM `1500 ± 변위`로 변환하며,
`Config::*_REVERSED = true`인 서보는 부호를 뒤집습니다. 기본값은 스틱에 대한
펌웨어의 기존 동작을 그대로 재현합니다. 조립이 끝난 기체에서의 확인은
[`PILOT_GUIDE.md`](PILOT_GUIDE.md)의 비행 전 점검 목록에 있습니다. 리버스는
`Config.h`에서 바꿔야 하며, **송신기에서 바꾸면 안 됩니다**. 그렇지 않으면
스틱과 오토파일럿의 방향이 어긋납니다.

---

## RC 채널 배치, ARM, failsafe

출처는 `include/config/Channels.h`입니다. 송신기 FS-i6(10채널, 모드 2) +
수신기 FS-iA6B, iBUS 115200.

| 채널 | 송신기의 조작부 | 이름 | 용도 |
|---|---|---|---|
| CH1 | 오른쪽 스틱 ←→ | `AILERON` | 롤 |
| CH2 | 오른쪽 스틱 ↑↓ | `ELEVATOR` | 피치 |
| CH3 | 왼쪽 스틱 ↑↓ | `THROTTLE` | 스로틀, 전체 행정. < 950 = 수신기의 failsafe |
| CH4 | 왼쪽 스틱 ←→ | `RUDDER` | 방향타 + 조향 바퀴(서보 하나) |
| CH5 | SwA | `ARM` | ≥ 1750 = ARM (FS-i6에서는 스위치를 아래, 자기 쪽으로 내린 상태) |
| CH6 | SwB | `SWB` | 기본값은 플랩(≥ 1750 — 펼침) |
| CH7 | SwC(3단) | `SWC` | 기본값은 모드: < 1250 MANUAL, 1250–1749 STABILIZE, ≥ 1750 AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | 기본값은 RTH |
| CH9 | VrA | `VRA` | 기본값은 안정화 강도 |
| CH10 | VrB | `VRB` | 기본값은 순항 속도 |

CH6–CH10은 `include/config/Controls.h`에 한 줄만 쓰면 할당됩니다
([AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#한-줄로-기능-지정하기)).

**ARM**(`ArmingManager`): 스위치가 OFF→ON으로 바뀌고, 스로틀이
`THROTTLE_LOW_US` 미만이며, 현재 모드에 필요한 센서 점검을 통과해야 합니다.
그렇지 않으면 이유를 Serial에 출력하며 거부하고, 새로 OFF→ON 동작이 필요합니다.
스위치가 ON인 채로 보드의 전원을 켜면 ARM되지 않습니다. OFF는 즉시 DISARM입니다.
ARM되지 않은 동안 ESC로 가는 스로틀은 강제로 `PWM_MIN`이 됩니다.

**링크 상실**(`IBusReceiver::isSignalLost()`):

1. `RX_TIMEOUT_US`(500 ms)보다 오래 프레임이 없음 — 배선이 끊겼거나 수신기
   전원이 나간 경우입니다. 전원을 켠 뒤 첫 프레임이 오기 전에도 링크가
   끊긴 것으로 간주합니다. 채널의 기본값(모두 1500)을 송신기의 명령으로
   착각하지 않게 하기 위해서입니다.
2. 스로틀 < `RX_FAILSAFE_THROTTLE_US`(950) — 송신기에 설정한 failsafe입니다.
   **FS-iA6B는 송신기를 잃어도 프레임 송출을 멈추지 않고** 마지막 값을
   반복합니다(테스트 벤치에서 확인). 따라서 송신기에서 failsafe를 설정하지
   않으면 링크 상실을 알아채지 못합니다. 설정 방법은 `PILOT_GUIDE.md`에
   있습니다.

링크를 잃었을 때의 동작(`FlightController::applyLinkLoss()`):

- **기체가 ARM 상태이고 GPS와 홈 포인트가 있음**(`FAILSAFE_RTH`) — 모터를
  쓰며 **홈으로 복귀**하고, 홈 상공에서 선회합니다. OLED에는 `FSRTH`, 로그에는
  `FAILSAFE_RTH`가 표시됩니다.
- **기체가 ARM 상태이고 GPS가 없음** — **활공**하며 모터는
  `FAILSAFE_THROTTLE`입니다. `Autopilot`은 어떤 모드에서든(MANUAL에서도)
  롤 `FAILSAFE_GLIDE_ROLL_DEG`(0은 직진, 10–20°는 조종자 상공에서 원 선회)와
  피치 `FAILSAFE_GLIDE_PITCH_DEG`(−3°, 모터 없이 속도를 잃지 않기 위함)를
  유지하고 플랩은 접습니다. OLED에는 `GLIDE`, 로그에는 모드
  `FAILSAFE_GLIDE`가 표시됩니다.
- **ARM하지 않음**(지상) 또는 IMU가 응답하지 않음 — 조종면은 중립이 됩니다.
- 모드와 기능은 스위치로 바뀌지 않으며 센서는 계속 읽습니다. ARM은 해제되지
  않으므로, 링크가 복구되면 기체는 다시 스틱과 선택된 모드를 따릅니다
  (자동 이륙과 손 발사는 처음부터 다시 시작해야 합니다).

---

## 센서 데이터

구조체는 `include/sensors/SensorInterface.h`에 있습니다.

### `ImuData`

| 필드 | 단위 | 의미 |
|---|---|---|
| `gyroX`, `gyroY`, `gyroZ` | °/s | 기체 축에서의 각속도, 항공 부호(위 참고) |
| `accelX`, `accelY`, `accelZ` | g | 기체 축에서의 가속도: X는 기수 방향, Y는 왼쪽, Z는 위쪽 |
| `roll`, `pitch` | ° | 상보 필터(α = 0.98, τ ≈ 0.1 초). 가속도계로 구한 각도에서 곧바로 시작 |
| `yaw` | ° | 자이로 적분값으로 천천히 드리프트함. 초깃값은 나침반 방위 |
| `temperature` | °C | 다이 온도(MPU6050 또는 MPU6500용 수식) |
| `timestamp` | µs | 읽은 시점의 `micros()` |

IMU 보정(시작할 때마다, 그리고 `i` 명령): 2초간 정지, 자이로 →
영점 오프셋, 가속도계 → **현재 자세가 수평선이 됨**.

### `BarometerData`

| 필드 | 단위 | 의미 |
|---|---|---|
| `pressure` | Pa | 기압 |
| `temperature` | °C | 센서 온도 |
| `altitude` | m | **보정 지점(시작 시)을 기준으로 한** 고도. 수식은 `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | 실제 샘플(50 Hz)에 대한 고도의 미분값을 τ = 0.5초 저역 통과 필터에 통과시킨 것 |
| `timestamp` | µs | 마지막 새 샘플의 시각 |

### `MagData`

| 필드 | 단위 | 의미 |
|---|---|---|
| `magX`, `magY`, `magZ` | µT | hard-iron 보정 후의 자기장, 기체 축 기준(`MAG_ROTATION_CW_DEG`) |
| `headingDegrees` | °(0..360) | `atan2(magY, magX)`. 기울기 보정 없음. 각도를 세는 방향은 조립된 기체에서 아직 확인하지 않음 |
| `timestamp` | µs | 읽은 시점(50 Hz) |

### `GpsData`

| 필드 | 단위 | 의미 |
|---|---|---|
| `latitude`, `longitude` | ° | UBX-NAV-PVT에서 가져옴 |
| `altitude` | m | 해발 고도(hMSL) |
| `groundSpeed`, `heading` | m/s, ° | 대지 속도와 대지 침로 |
| `numSatellites`, `fixType` | — | 0 = 측위 없음, 2 = 2D, 3 = 3D |
| `horizontalAccuracy`, `verticalAccuracy` | m | 모듈이 내놓는 정확도 추정치 |

**`isAvailable()`의 의미.** I2C 센서의 경우: `begin()`에서 센서가 응답했고
**또한** 최근 읽기가 연속으로 실패하지 않아야 합니다(MPU는 약 0.1초, 기압계와
나침반은 약 0.5초 동안 응답 없음). 읽기에 실패해도 데이터는 쓰레기 값으로
덮어쓰이지 않습니다. 이전 값이 남고 오류 카운터가 증가합니다(`s` 명령으로
확인). GPS의 경우: 유효한 NAV-PVT가 적어도 하나 있고, 가장 최근 것이
`GPS_TIMEOUT_US`보다 오래되지 않아야 합니다.

**센서가 없으면**(`nullptr` 또는 `isAvailable() == false`) `Autopilot`은
보정을 전혀 하지 않으며 기체는 MANUAL처럼 조종됩니다. `main.cpp`는 응답한
센서만 보정합니다.

---

## FlightController::update() 상세

`loop()`에서 2 ms마다 호출됩니다. 순서가 곧 우선순위입니다.

1. **`receiver.update()`** — 쌓인 iBUS 바이트를 해석합니다.
2. **스위치** — `switches->update(rc)`. 링크가 살아 있을 때만 실행합니다
   (failsafe 프레임에서는 채널이 스위치 상태를 반영하지 않습니다). 대상은
   모드(바뀔 때만), 기능, 노브입니다.
3. **조종자의 스로틀** — `throttle.update(rc, receiverFailsafe)`.
4. **스틱** — `mixer.fromSticks(rc)` × `Knob::RATES`. 플랩은
   `mixer.updateFlaps(target)`(브레이크, 스위치, 노브. 링크가 없으면 0).
5. **센서와 오토파일럿** — `autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   를 **항상** 실행합니다. 링크가 없어도 마찬가지입니다. 각도 필터가 멈추면
   안 되기 때문입니다. ARM하지 않은 동안에도 PID는 동작하며(조종면이 기울기에
   반응하므로 책상 위에서 확인하기 편함) 적분기만 0으로 유지됩니다. 링크가
   없고 ARM 상태이면 failsafe RTH 또는 활공입니다.
6. **부저** — `Beeper`.
7. **링크 상실** — `applyLinkLoss()`. ARM 상태에서는 조종면과 스로틀이 오토파일럿의
   failsafe 명령을 따르고, 그렇지 않으면 중립과 모터 정지이며 `return`합니다.
   아래의 모든 것보다 절대적으로 우선합니다.
8. **ARM** — `arming.update(rc, false)`.
9. **명령** — `autopilot->getCommand()`. 안정화 모드에서는 스틱이 원하는
   각도이고, 최종 조종면 명령은 오토파일럿이 냅니다.
10. **믹서** — `mixer.mix(command)` → 에일러론(플랩 + 롤), 승강타, 방향타의
    PWM. 리버스를 반영합니다.
11. **스로틀** — `autopilot->applyThrottle(pilotThrottle)`. 조종자의 스로틀,
    오토파일럿의 스로틀, 또는 둘 중 큰 값(자동 이륙). 그런 다음 ARM하지
    않았거나 `MOTOR_KILL`이면 강제로 `PWM_MIN`입니다. 어떤 모드도 ARM을
    우회해 스로틀을 내보내지 못하도록 이 검사는 맨 마지막에 둡니다.
12. **AUX** — 페이로드(`PAYLOAD_DROP`)와 카메라(`CAMERA_TILT`, `CAMERA_STAB`).
13. **`outputs.write(output)`** — 7개 출력으로 PWM을 내보냅니다.

---

## 웹 대시보드의 HTTP API

구현은 `include/telemetry/WebDebugServer.h`입니다. 액세스 포인트: SSID
`OpenPlane-Debug`, 비밀번호 `12345678`, 주소 `http://192.168.4.1`.

### `GET /api/status`

```json
{
  "rc": [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1500],
  "armed": false,
  "failsafe": false,
  "outputs": {
    "aileronLeft":  { "us": 1500, "attached": true },
    "aileronRight": { "us": 1500, "attached": true },
    "elevator":     { "us": 1500, "attached": true },
    "rudder":       { "us": 1500, "attached": true },
    "esc":          { "us": 1000, "attached": true },
    "aux1":         { "us": 1000, "attached": true },
    "aux2":         { "us": 1500, "attached": true }
  },
  "flapsUs": 0,
  "imu":  { "attached": true, "available": true, "roll": 0.12, "pitch": -0.40, "yaw": 38.50 },
  "baro": { "attached": true, "available": true, "altitude": 0.05, "climb": 0.01 },
  "mag":  { "attached": true, "available": true, "heading": 41.9 },
  "gps":  { "attached": true, "available": true, "fix": 3, "numSV": 12, "lat": 55.750000, "lon": 37.610000, "alt": 150.0 },
  "airspeed": { "attached": true, "available": true, "ias": 14.2, "tas": 14.3, "dp": 123.4 },
  "autopilot": {
    "attached": true, "mode": 1, "modeName": "STABILIZE",
    "desiredRoll": 0.0, "desiredPitch": 0.0, "targetAlt": 0.0,
    "rollCorr": 0.0, "pitchCorr": 0.0, "throttleCorr": 0.0,
    "kpRoll": 5.000, "kiRoll": 0.500, "kdRoll": 0.500,
    "kpPitch": 5.000, "kiPitch": 0.500, "kdPitch": 0.500,
    "nav": { "gps": true, "home": true, "homeDist": 120, "homeBearing": 185,
             "course": 90, "targetCourse": 90, "speed": 14.3, "fence": false, "stall": false },
    "features": ["FLAPS"]
  }
}
```

- `attached` — 객체가 빌드에 포함되어 있음. `available` — 센서가 실제로
  응답함. 데이터 필드는 `available: true`일 때**만** 추가됩니다.
- `outputs.*.attached` — MCU가 LEDC 채널과 핀을 할당함. 물리적 서보가 연결되어
  있는지는 소프트웨어로 알 수 없습니다(펄스를 확인하려면 콘솔의 `p` 명령을
  사용).
- `rollCorr`/`pitchCorr` — 오토파일럿의 최종 명령에서 스틱 값을 뺀 것(µs).
  `throttleCorr` — 오토파일럿의 스로틀(%). 스로틀이 조종자에게 있는 동안은 0.
- `nav` — 내비게이션 정보: 홈, 홈까지의 거리와 방위, 침로와 목표 침로,
  내비게이션에 쓰는 속도(피토관 / GPS), 지오펜스, 실속. `features` — 켜져 있는
  스위치 기능.

### `POST /api/setmode`

`{ "mode": 1 }` — `AutopilotMode`의 번호: `0` MANUAL, `1` STABILIZE, `2`
AUTO_TAKEOFF, `3` ALT_HOLD, `4` ACRO, `5` CRUISE, `6` LOITER, `7` RTH, `8`
LAUNCH, `9` AUTO_LAND, `10` SOARING, `11` RESCUE. 이 모드는 조종자가 모드
스위치를 딸깍 바꿀 때까지 유지됩니다.

### `POST /api/setpid`

`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }` — `kpRoll`, `kiRoll`, `kdRoll`,
`kpPitch`, `kiPitch`, `kdPitch` 중 아무 필드나 지정할 수 있으며, 생략한 필드는
이전 값을 유지합니다.

두 명령 모두 비행 루프가 다음 주기에 적용합니다([FreeRTOS 태스크](#freertos-태스크와-제어-루프)
참고).

### `GET /`

HTML 대시보드: 10개 채널의 막대, ARM/링크, 출력, 센서, 모드 버튼, PID 입력
양식. 200 ms마다 `/api/status`를 조회합니다.

---

## 콘솔과 진단

포트 모니터는 115200이며 커넥터는 “COM”입니다. 구현은 `DebugConsole`과
`DebugLogger`입니다([참조](reference/telemetry.md)). 키는 누르는 즉시 동작하며
Enter는 필요 없습니다. 보정과 `p`는 루프를 멈추게 하므로 ARM하지 않았을
때만 쓸 수 있습니다.

| 키 | 동작 |
|---|---|
| `h` / `?` | 메인 메뉴 |
| `l` | “로그에 무엇을 출력할지” 메뉴(채널, 모드, 주기) |
| 스페이스 | 로그 일시 정지 / 계속 |
| `s` | 모든 센서의 `printStatus()`: 데이터, 버스 오류 카운터, 보정, 비행 전 점검 |
| `i` | 자이로 보정 + 비행 전 점검(2초간 정지) |
| `o` | 세 가지 자세로 IMU 장착 보정, NVS에 저장 |
| `m` | 나침반 보정(15초간 회전), NVS에 저장 |
| `p` | 출력 자가 점검: 각 핀의 실제 펄스를 기대값과 비교 |

로그는 채널(`STAT`, `RC`, `OUT`, `ATT`, `AP`, `ALT`, `MAG`, `GPS`, `IMU`,
`SYS`)로 나뉘며, 각 채널에는 “끔 / 변경 시 / 상시” 모드가 있습니다. 설정은
NVS에 저장되고 메뉴를 닫을 때 기록되며, ARM하지 않았을 때만 기록됩니다.
기본값으로 `STAT`(변경 시)과 `SYS`(10초에 한 번)가 켜져 있습니다.

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (10초 중 최악) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

모든 채널의 형식은 [참조](reference/telemetry.md#debuglogger)에 있습니다.

---

## 보드 선택과 핀 배치

| 명령 | `board` | 매크로 | 상태 |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8(`qio_opi`, 16 MB) | `BOARD_ESP32_S3` | **주력, 기본값.** 모든 센서를 연결해 테스트 벤치에서 확인 |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | 예전 프로토타입. 수동 조종으로 비행했음 |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | 테스트 벤치용. 핀 배치는 실제 하드웨어에서 확인하지 않음 |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6: 전체 펌웨어 + MAVLink + SD 블랙박스. 맨 보드에서 확인([아래](#stm32h743)) |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | DevEBox H743에서도 동일: 콘솔은 USB CDC, 펌웨어는 DFU로 기록 |

| 용도 | ESP32-S3(테스트 벤치) | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| 에일러론 왼쪽 / 오른쪽 | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| 승강타 / ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| 방향타 | GPIO18 | —(핀 없음) | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| 센서 I2C SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| OLED I2C SDA / SCL | GPIO1 / GPIO2 | — | — |
| SPI SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI CS ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS RX / TX | GPIO39 / GPIO40(UART2) | GPIO9 ⚠️ / 없음(UART0) | GPIO4 / GPIO17(UART2) |
| Serial | UART0 → “COM” 커넥터 | USB-CDC | UART0 |

- **ESP32-S3 N16R8:** GPIO33–37은 옥탈 PSRAM이, 26–32는 플래시가, 19/20은
  USB가, 43/44는 Serial이 사용하며 48은 RGB LED입니다. 0/3/45/46은 strapping
  핀입니다.
- **ESP32-C3:** GPIO4/5의 에일러론은 S3와 서로 바뀌어 있습니다. 전체 구성을
  위한 핀이 부족합니다. BMP388의 CS와 GPS의 RX는 strapping 핀에 있고, GPS에는
  TX가 없습니다(수신 전용, UBX-CFG 없음). 자세한 내용은 `Config.h`에 있습니다.

### STM32H743

STM32H743VIT6(Cortex-M7 480 MHz, 플래시 2 MB, RAM 1 MB)은 **전체 펌웨어**를
실행합니다. ESP32-S3와 같은 센서, 오토파일럿, 스위치, 콘솔, 화면에 더해 MAVLink
텔레메트리와 SD 카드 블랙박스가 있습니다. 빌드가 되고, cppcheck와 공통 코드의
모든 네이티브 테스트를 통과합니다. 실제 하드웨어에서 확인한 것은 **센서 없는
DevEBox H743 보드**입니다: 부팅, USB 콘솔, SD 카드, 블랙박스
([TESTING.md](TESTING.md#stm32-보드에서의-테스트)), 그리고 iBUS, ARM, 서보와 모터로
가는 PWM(송신기로 수동 모드에서 조종, 영상 있음). STM32의 센서는 아직 테스트
벤치를 기다리고 있습니다. 주력 비행 보드는 ESP32-S3입니다.

- **HAL** — `include/hal/stm32/`: `Stm32Board`(`Esp32Board`와 같은 API에
  `telemetryUart()`가 추가됨), `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`,
  `Stm32ServoOutput`(`HardwareTimer`의 하드웨어 PWM, 타이머 하나가 여러 출력을
  담당). 자세한 내용은 [reference/hal.md](reference/hal.md#stm32h743용-구현).
- **설정과 보정** — NVS가 아니라 플래시의 마지막 섹터에 있는 `KeyValueStore`
  (`include/storage/`, `hal/stm32/Stm32FlashStorage.h`)입니다. 프로젝트 코드는
  여전히 `#include <Preferences.h>`라고 씁니다. env `stm32h743`에서는
  `include/hal/stm32/compat/`가 `-I`에 들어 있고, 그곳에 같은 API의
  `Preferences`가 있습니다. 이미지에는 CRC32가 붙으며, 손상된 이미지(지우는
  도중에 전원이 나간 경우)는 빈 것으로 읽힙니다. 플래시 쓰기는 백그라운드
  태스크에서 합니다. 128 KB 섹터를 지우는 데 몇 초가 걸리지만 그 섹터는 뱅크
  2에 있고 코드는 뱅크 1에서 실행되므로, 비행 태스크는 멈추지 않고 백그라운드
  태스크를 선점합니다.
- **태스크** — STM32duino FreeRTOS 라이브러리의 FreeRTOS, 단일 코어, 우선순위
  선점(`hal/Rtos.h`): `flight`(5)는 비행 루프, MAVLink, 로그, 콘솔. `oled`(1)와
  `storage`(1)는 백그라운드. `bbox`(2)는 SD 카드에 블랙박스를 기록합니다.
- **SD 카드의 블랙박스** — SDMMC1, 4비트, 24 MHz(`hal/stm32/Stm32SdCard.h`,
  핀은 `src/stm32/sd_msp.cpp`). 카드는 일반 FAT32로 남습니다. 미리 만들어 둔
  `BLACKBOX.BIN` 파일이 카드에 있고, 펌웨어는 그 안에 원시 블록을 쓰며
  파일 시스템 자체는 건드리지 않습니다(`storage/Fat32File.h`는 읽기 전용).
  카드 준비와 데이터 내려받기는 [BLACKBOX.md](BLACKBOX.md#sd-카드stm32h743)를
  참고하세요.
- **텔레메트리** — Wi-Fi 대시보드 대신 UART4의 MAVLink 2
  (`telemetry/MavlinkTelemetry.h`)를 씁니다. QGroundControl / Mission Planner로
  지상에서 모드와 PID를 바꿀 수 있습니다. 자세한 내용은
  [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#지상국-wi-fi-대시보드와-mavlink).
- **핀 배치** — `Config.h`의 `BOARD_STM32H743` 블록. 핀은 WeAct
  MiniSTM32H743VITx에서 비어 있는 핀 중에서 골랐고 STM32duino의 표와 대조했습니다.

| 용도 | STM32H743 | 주변장치 |
|---|---|---|
| 에일러론 왼쪽 / 오른쪽 | PA0 / PA1 | TIM2_CH1 / CH2 |
| 승강타 / ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| 방향타 | PD14 | TIM4_CH3 |
| AUX1(페이로드) / AUX2(카메라) | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX(TX는 예비) | PE7(PE8) | UART7 |
| 센서 I2C SDA / SCL | PB11 / PB10 | I2C2 |
| OLED I2C SDA / SCL | PB9 / PB8 | I2C1 |
| SPI SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI CS IMU / 기압계 | PB12 / PD10 | GPIO |
| GPS RX / TX | PD9 / PD8 | USART3 |
| 무선 모뎀 MAVLink RX / TX | PD0 / PD1 | UART4 |
| 부저 | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **DevEBox H743(MCUDEV)** — env `stm32h743-devebox`: 코드는 같고 코어는 전용
  변종을 쓰며, 콘솔은 USB-C를 통한 가상 COM 포트(CDC)라 USB-UART가 필요
  없습니다. 첫 펌웨어 기록은 내장 부트로더(DFU)를 통해 USB로 합니다.
  1. Windows: “STM32 BOOTLOADER”용 WinUSB 드라이버를 한 번 설치합니다
     ([Zadig](https://zadig.akeo.ie): DFU in FS Mode → WinUSB → Install Driver).
  2. **BT0**(BOOT0) 핀을 **3V3**에 전선으로 연결하고 **RST**를 눌렀다
     뗍니다. 보드가 DFU 모드가 됩니다(DevEBox에는 BOOT0 버튼이 없습니다).
  3. `pio run -e stm32h743-devebox -t upload` (`upload_protocol = dfu`).
  4. BT0 전선은 빼도 됩니다. 펌웨어가 저절로 시작됩니다.

  이후에는 전선이 필요 없습니다. 콘솔의 **`D`** 키(어느 메뉴에서든, ARM 중에는
  불가)를 누르면 보드가 부트로더로 재부팅합니다. RAM에 표식을 남김 → 리셋 →
  클럭을 설정하기 전에 시스템 메모리로 점프(`src/stm32/bootloader.cpp`). H7에서는
  동작 중인 펌웨어에서 곧바로 점프하면 멈춥니다. 보드에서 확인한 사실이며, 그래서
  두 단계로 나눕니다. 콘솔(USB CDC)을 열어 두어야 하고, 보드가 응답하지 않으면 BT0
  전선을 연결한 채로 RST를 누르세요.
- **진입점** — `src/stm32/main.cpp`(ESP32 빌드에서는 `build_src_filter`로
  제외). 객체는 `src/main.cpp`와 같고, `loop()` 대신 태스크가 있으며
  `vTaskStartScheduler()`는 `setup()`의 끝에 있습니다.
- **보드를 처음 켤 때:** `pio run -e stm32h743 -t upload`(ST-Link), 모니터는
  USB-UART를 통해 LPUART1에 연결합니다. `b`로 버스에 센서가 보이는지, `s`로 센서
  상태를, `p`로 출력의 펄스를 확인하고(프로펠러는 떼어 둘 것), 그다음 송신기와
  무선 모뎀을 통한 QGroundControl을 시험합니다.

---

## 새 센서를 추가하는 방법

### A) 기존 범주의 다른 칩(IMU, 기압계, 나침반)

공통 부분은 이미 기반 클래스에 작성되어 있으므로 칩 드라이버는 작게 나옵니다.

1. `include/sensors/<category>/<Name>_Sensor.h`를 만들고 `ImuSensorBase` /
   `BarometerBase` / `MagnetometerBase`를 상속합니다. 생성자는
   `IRegisterDevice&`를 받으며, 드라이버는 I2C인지 SPI인지 알지 못합니다.
2. 다음을 구현합니다.
   - `begin()` — `device.begin()`, 칩 ID 확인, 레지스터 쓰기,
     `setAvailable(true/false)` 호출.
   - IMU: `readSample()`(칩 축 기준의 원시 accel/gyro/temp),
     `accelLsbPerG()`, `gyroLsbPerDps()`, `temperatureC()`.
   - 기압계: `isNewSampleReady()`(준비 플래그 또는 그냥 `true`)와
     `readSample()`(기압은 Pa, 온도는 °C). 폴링 주기는 기반 클래스의
     생성자에서 지정합니다.
   - 나침반: `readRaw()`(칩 축 기준의 X/Y/Z)와 `lsbPerMicroTesla()`. 보정용
     NVS 네임스페이스 이름은 기반 클래스의 생성자에서 지정합니다.
3. SPI에서 데이터 앞에 더미 바이트가 필요하거나 특별한 주파수가 필요한 칩이라면
   `BMP388_Sensor`처럼 정적 팩토리 `spiDevice(bus, cs)`를 추가합니다.
4. `SensorSelection.h`에 분기를 추가합니다: `#define SENSOR_<CATEGORY>_<NAME>`,
   `using Selected... = ...;`, `#define SELECTED_..._DEVICE(board) ...`
   (`I2cRegisterDevice(board.i2c(), address)` 또는 SPI 팩토리). 센서를 바꿔도
   `main.cpp`는 건드리지 않습니다.
5. 파일을 고치지 않고 플래그로 새 센서의 빌드를 확인합니다:
   `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAME>" pio run`. 이어서
   세 환경 모두, 그다음 실제 하드웨어에서 확인합니다.

### B) 새 범주

1. 데이터 구조와 인터페이스는 `SensorInterface.h`에 `GpsSensor`/`GpsData`를
   본떠서 작성합니다.
2. 범주에 공통 로직(필터, 보정)이 있다면 `BarometerBase`를 본뜬 기반
   클래스를 만듭니다.
3. `Autopilot` 생성자에는 null을 허용하는 포인터를 받고(센서가 없으면 아무
   영향이 없고 비정상 종료도 없음), `GET /api/status`에는
   `attached`/`available` 쌍을 가진 필드를 추가합니다.

### 새 버스나 주변장치

`include/hal/`에 새 인터페이스를 만들고, `include/hal/esp32/`와
`include/hal/stm32/`에 구현을 두며, `IBoard`를 통해 접근합니다.

---

## 새 오토파일럿 모드를 추가하는 방법

1. `enum AutopilotMode`(`autopilot/AutopilotTypes.h`, `MODE_COUNT` 앞)에 값을
   추가하고, `AutopilotNames::mode()` / `modeShort()`에 이름과 짧은 이름(OLED용,
   최대 5자)을 추가합니다.
2. 핸들러 `run<Mode>()`를 만들고 `Autopilot::runMode()`에 분기를 추가합니다.
   초기 목표(침로, 고도, 선회 원의 중심)는 `initializeMode()`에 둡니다. 모드는
   `desiredRoll`/`desiredPitch`를 정하고 `stabilizeOrManual()`(IMU가 없으면 조종면은
   조종자가 직접 조작) 또는 `stabilizeOrNeutral()`(IMU가 없으면 중립)을
   호출합니다. 필요한 센서가 없으면 비정상 종료가 아니라 안전한 동작이어야
   합니다. 적분기는 `armed`일 때만 누적됩니다.
3. 스로틀: `throttleMode`(`PILOT` / `AUTO` / `AT_LEAST`)와 `autoThrottlePct`, 또는
   `autoThrottle()` — 노브 / 피토관에서 얻는 순항 스로틀. 이 경우
   `FlightController`는 바뀌지 않습니다.
4. 송신기 쪽은 `config/Controls.h`에 한 줄만 추가하면 됩니다
   (`Bind::mode(Channels::SWD, MODE_NEW)`). 대시보드와 MAVLink는 모드를 번호로
   가져갑니다. MAVLink의 경우 `MavlinkModes::toCustomMode()` /
   `fromCustomMode()`에서 가장 가까운 ArduPlane 모드로 대응시킵니다.
5. 모드의 ARM에 센서가 필요하다면 `ArmingManager`를 수정합니다.
6. 테스트: 각 센서에 대한 반응은 `test/native/test_autopilot_modes`, 폐루프
   비행은 `test/native/test_sim`의 시나리오(기체 모델 `helpers/PlaneSim.h`, 테스트
   환경 `helpers/SimHarness.h`)로 확인합니다. 그다음 프로펠러 없이 책상 위에서
   확인합니다. 조종면은 기울였을 때 수평으로 되돌리는 방향으로 반응해야 합니다.
7. [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md)에 절을 추가합니다.

---

## 피드백(기초 작업, 연결되지 않음)

`include/autopilot/feedback/`는 오토파일럿의 다음 단계입니다. **`FlightController`도
`Autopilot`도 `main.cpp`도 이 파일들을 포함하지 않습니다.** 비행 시험용 프로토타입은
아직 없고, 펌웨어는 이것들 없이 동작합니다. 검증은 폐루프 시뮬레이션
(`test/test_feedback/`)을 보드에서 그대로 실행해 수행합니다.

### 왜 필요한가

현재의 `Autopilot`은 각도에 대한 PID입니다: 오차 × 계수 = 조종면. 그 결과가 기체에서
어떻게 나타났는지는 알지 못하고, 계수도 한 가지 속도에서만 맞습니다. 저속에서는
조종면의 효과가 약해 PID가 덜 보정하고, 고속에서는 과하게 보정합니다. 피드백은
**기체의 반응**으로 루프를 닫습니다.

- 조종면을 움직였는데 기체가 필요한 것보다 느리게 돈다 → 도달할 때까지 더한다.
- 필요한 조종면의 양은 비행 중에 측정하고 속도에 따라 다시 계산한다.
- 기체가 엉뚱한 방향으로 돈다 → 부호가 뒤바뀐 것이므로 뒤집어서 확인한다.
- 각도는 맞췄는데 속도가 떨어진다 → 기체가 실속하지 않도록 스로틀을 올리고
  기수를 내린다.
- 이륙과 착륙 → 센서가 보여 주는 상태에 따라 단계별로 진행한다.

### 모듈

| 파일 | 하는 일 |
|---|---|
| `FlightSnapshot.h` | 한 주기 동안 피드백이 기체에 대해 아는 모든 것. 유일한 입력이며, 모듈은 센서와 RC를 직접 읽지 않으므로 시뮬레이션과 로그에서도 돌릴 수 있음 |
| `FeedbackOutput.h` | 한 주기의 출력: 축별 조종면 변위, 해당 축이 켜져 있는지, 축의 부호, 스로틀(지정 / 이하로 내려가지 않음), 이유 |
| `FeedbackConfig.h` | 모든 상수(연결할 때 `Config.h`로 옮길 예정) |
| `SpeedEstimator.h` | 속도(피토관 > GPS)와 IMU로 구한 전후 방향 가속도: `dV/dt = g·(ax − sin θ)` — 속도 센서가 없어도 “속도가 떨어진다”는 것을 알 수 있음 |
| `AirborneDetector.h` | 공중 / 지상 판정: 학습, 적분 누적, 실속 탐지는 비행 중에만 의미가 있음 |
| `ControlEffectivenessEstimator.h` | 축마다 재귀 최소제곱법으로 모델 `ε = b·u(t−delay) + a·ω + c`를 학습 |
| `AdaptiveRateController.h` | 각도 → 각속도 → 각가속도 → 학습한 모델을 거친 조종면으로 이어지는 캐스케이드 |
| `StallGuard.h` | 속도 저하와 실속에 대한 보호 |
| `TakeoffSequencer.h`, `LandingSequencer.h`, `PhaseTargets.h` | 이륙(활주로 또는 손 발사)과 착륙을 센서에 따라 단계별로 수행 |
| `FeedbackSupervisor.h` | 전체를 묶음: 한 주기 안의 순서, 우선순위, `requestTakeoff()`/`requestLanding()`/`cancelPhase()`, `printStatus()`, 연결 계획 |
| `FeedbackModules.h` | 전부를 한 번에 가져오는 include 하나 |

### 동작 원리

**조종면의 효과.** 축의 모델은 각가속도 `ε = b·u + a·ω + c`입니다. `b`는 조종면
1 µs가 만들어 내는 °/s²(부호는 반응의 방향), `a`는 감쇠(공기가 회전을 늦춤. 이 항이
없으면 정상 회전 중에 `b`의 추정이 0으로 가 버림), `c`는 일정한 모멘트(무게중심,
트림, 프로펠러)입니다. 조종면의 힘은 ∝ ρV²이므로 `b`는 기준 속도에서 학습하고
`(V/Vref)²`를 곱합니다. 기체가 빨라지면 재학습 없이 조종면이 곧바로 “강해지는”
셈입니다. 피토관의 지시 대기속도에는 이미 공기 밀도가 들어 있으므로 고도는 저절로
반영되고, 속도 센서가 없으면 스케일은 1이며 `b`를 직접 학습합니다.

데이터는 20 ms 구간마다 취합니다. 구간의 평균 가속도는 양 끝에서의 자이로 차이 ÷
구간 길이이고, 여기에 대응하는 것은 같은 구간의 평균 조종면 값과 평균 각속도입니다
(조종면은 `RESPONSE_DELAY_MS`만큼의 지연을 반영). 그다음 방정식 양변을 같은 2 Hz
저역 통과 필터에 통과시킵니다. 비율은 변하지 않고, 서보의 관성 때문에 “순수 지연”
모델이 어긋나는 고주파 성분은 제거됩니다. 학습은 공중에서, 그리고 조종면이 “흔들릴”
때(약 0.3초 동안 진폭 ≥ `MIN_EXCITATION_US`)만 가능합니다. 조종자의 스틱 조작도
흔들림에 해당하므로 MANUAL에서도 추정이 학습됩니다.

**제어기.** 3단 구성이며 축마다 처리합니다.

```
ω* = ANGLE_GAIN · (target − angle)              "기수가 10° 아래 — 40°/s로 들어 올림"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "필요보다 느리게 돈다 — 보정"
surface = (ε* − a·ω − c) / b                    학습한 모델을 거침
```

적분항 `I`는 조종면의 µs가 아니라 °/s로 저장합니다. 그래서 `b`의 추정이 바뀌어도 값이
정확하게 유지됩니다. 지상에서는 적분을 고정하고(이륙 활주·착륙 활주 중의 침로는 제외),
조종면이 한계에 닿으면 한계 방향으로는 누적하지 않습니다. 협조 선회도 반영합니다(속도를
아는 경우). 뱅크 중에는 피치 `g·sin φ·tg φ / V`와 요 `g·sin φ / V`가 필요합니다.

**축의 부호는 지상에서만 정한다.** 비행 중에는 축을 끄거나 뒤집지 않습니다. IMU 장착은
보정 `o`와 전원을 켤 때의 점검으로 정해지고, 조종면의 방향은 조종자의 비행 전 점검으로
정해집니다. 공중에서의 간접적인 징후(실속 이탈, 스핀, 곡예 비행, 돌풍)는 오판을 부를 수
있고, 그런 순간에 축을 끄거나 뒤집으면 기체를 잃을 수 있습니다. 어떤 축의 `b` 추정값이
확실히 음수라도 `reason`에 경고(“조종면에 반대로 반응? 지상에서 확인”)를 남길 뿐이며,
음수 추정값은 제어기로 들어가지 않고 그 축은 사전 모델로 동작합니다.

**실속 보호.** 두 단계가 있습니다. *LowEnergy*: 기수를 든 채 속도가 빠르게 떨어지거나,
실속에 가깝거나(< 1.25·Vs), 승강타의 효과가 사라진 경우: 스로틀 ≥ 80 %, 피치 ≤ 5°.
*Stall*: 속도가 실속 속도보다 낮고, 승강타에 반해 기수가 내려가거나, 낮은 에너지에서
에일러론에 반해 날개가 떨어지는 경우: 풀 스로틀, 기수 아래, 뱅크 ≤ 10°, 에일러론 제한
(큰 에일러론은 날개 끝을 실속시킴). 조치는 히스테리시스를 두고 해제합니다(속도
≥ 1.5·Vs). 링크를 잃었을 때는 스로틀을 건드리지 않으며, 지면에 아주 가까울 때(플레어,
착륙 활주)는 보호를 꺼 둡니다. 착륙 자체가 제어된 실속이기 때문입니다.

**이륙.** `WaitThrottle`(모터 정지) → 조종자가 스로틀 ≥ 50 %를 줌 → `GroundRoll`(풀
스로틀, 날개 수평, 침로는 방향타와 바퀴로 유지, 승강타는 자유) → 이륙 속도 도달(속도
센서가 없으면 타임아웃) → `Climb`(12°, 풀 스로틀) → 고도 30 m → `Complete`. 손 발사
(`TAKEOFF_HAND_LAUNCH`)에서는 활주 대신 `WaitLaunch`가 되어 던진 뒤에야 모터가
시동됩니다(전후 방향 가속도 ≥ 1g). 이륙 전에 스로틀을 줄이면 취소입니다.

**착륙.** `Approach`(스로틀 25 %, 강하율 1 m/s — 피치는 수직 속도 오차에서, 뱅크는
조종자에게서 ≤ 20°) → 고도 2 m → `Flare`(스로틀 0, 같은 규칙으로 강하율을 0.3 m/s까지
줄임) → 가속도계로 접지 충격 감지, 또는 “낮고 회전하지 않음” → `Rollout`(바퀴로 침로
유지) → `Complete`. 조종자의 스로틀 ≥ 80 %는 복행입니다. 플레어에는 거리계가
필요합니다. 기압계는 1 m쯤 틀립니다.

**우선순위**(`FeedbackSupervisor`): ARM하지 않음 > 실속 보호 > 이륙/착륙 > 모드의
목표. 링크를 잃으면 각 단계가 취소되고, 안정화 제어는 failsafe 활공의 목표를 수행합니다.

### 시뮬레이션

`test/test_feedback/test_main.cpp`(PC에서는 `pio test -e native -f test_feedback`)에는 기체 모델(독립된 각 축,
서보의 지연과 관성, 조종면 효과 ∝ V², 감쇠 ∝ V, 일정한
모멘트, 속도에 따른 받음각을 거친 양력, 실속, 조향
바퀴가 있는 착륙장치)과 10개의 시나리오가 있습니다.

| 시나리오 | 확인하는 내용 |
|---|---|
| 일정한 모멘트가 있는 상태에서 뱅크 30° / 피치 −15°로부터의 회복 | 수평 복귀와 “더 보정”: 적분항이 트림을 스스로 찾음 |
| 속도 센서 없이 14와 20 m/s에서 ±15° 흔들기 | `b`의 추정이 참값으로 수렴하고 속도에 따라 다시 계산됨 |
| 에일러론이 뒤바뀜, 조종자가 MANUAL에서 날개를 흔듦 | `b`의 추정이 음수 → 경고만 하고 그 축은 꺼지지 않음 |
| 30초간의 난기류 | 돌풍을 상쇄하고 뱅크가 10°를 넘지 않음 |
| 스로틀 20 %에서 기수 15°(속도 센서 있을 때와 없을 때) | 속도가 실속까지 떨어지지 않음 |
| 프로펠러의 반작용 토크가 있는 활주로 이륙 | 각 단계, 고도, 활주 중의 침로 |
| 15 m에서의 착륙 | 각 단계, 지면 근처에서 스로틀 없음, 부드러운 접지 |
| 이륙 활주 중 링크 상실, ARM하지 않음, MANUAL | 취소, 스로틀은 건드리지 않음, 조종면은 조종자 몫 |

모델은 조잡합니다. 확인하는 것은 로직과 부호이지, 특정 기체에 맞춘
튜닝이 아닙니다.

```bash
pio test -e native -f test_feedback      # PC에서 몇 초
pio test -e esp32-s3 -f test_feedback    # 테스트 펌웨어를 기록하고 실행
pio run -t upload                        # 일반 펌웨어로 되돌림
```

### 연결 계획

1. `FlightController::update()`가 센서를 읽고 명령을 계산한 뒤
   `FlightSnapshot`을 채우고 `FeedbackSupervisor::update()`를 호출합니다. 처음에는
   **섀도 모드**입니다. 출력은 로그(`printStatus()`)와 대시보드에만 내보내고
   조종면에는 보내지 않습니다. 수동 조종으로 비행하는 동안 각 축의 `b` 추정은
   양수여야 하고 속도와 함께 커져야 합니다.
2. 지상에서 기체를 손에 들고 STABILIZE: 기울이면 조종면이 이를 상쇄합니다.
3. 한 번에 한 축씩: `Autopilot::getRollCorrection()` 대신 `deflectionUs`를
   씁니다(처음에는 롤만), 그다음 피치입니다.
4. 스로틀: `throttleOverridePercent`/`throttleFloorPercent` —
   `Autopilot::applyThrottle()` 뒤, failsafe 앞에 둡니다(failsafe가 모든 것보다
   우선합니다).
5. 이륙/착륙은 비어 있는 스위치에 할당하고, `Autopilot`에서 `AUTO_TAKEOFF`
   모드를 제거합니다.
6. `FeedbackConfig`의 상수는 `Config.h`로 옮깁니다. 대기속도 센서는
   `AirspeedSensor` 구현과 `SensorSelection.h`의 범주로 마련합니다.

---

## 새 보드를 추가하는 방법

1. `platformio.ini`에 `[env:<name>]`을 추가하고 고유한
   `-D BOARD_ESP32_<NAME>`을 붙입니다.
2. `Config.h`에 `#elif defined(BOARD_ESP32_<NAME>)` 블록을 추가해 모든
   핀을 적습니다. `PIN_I2C2_SDA/SCL`도 포함합니다(OLED가 없으면 −1). GPIO 예산을
   미리 계산하세요: flash/PSRAM/USB/strapping.
3. 서보 출력에는 LEDC 채널 5개가 필요하며, 모든 ESP32에 있습니다. 방향타용 핀이
   없으면 `PIN_RUDDER = -1`로 두고, 그 출력은 그냥 꺼집니다.
4. 보드를 실제 하드웨어에서 확인하기 전에는 `default_envs`를 바꾸지 마세요. 핀
   배치를 확인하지 못했다면 커밋에 분명히 적으세요.

---

## 빌드, 업로드, 모니터 명령

```bash
pio run                        # 기본 보드(esp32-s3) 빌드
pio run -t upload              # 업로드
pio device monitor             # 모니터, 115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # 모든 보드가 빌드되는지 확인
```

- **ESP32-S3:** 업로드와 Serial은 “COM” 커넥터(CH343)를 통해 합니다. 브리지가
  멈추면(Windows가 “장치가 작동하지 않습니다”라고 응답함 — ESC의 잡음 때문에
  생길 수 있음) 케이블을 다시 연결하면 해결됩니다. “USB” 커넥터(내장 USB-JTAG)로도
  업로드할 수 있습니다: `pio run -t upload --upload-port <USB COM port>`.
- 포트 모니터가 열려 있는 동안에는 같은 포트로 업로드할 수 없습니다.
- `lib_deps`: `olikraus/U8g2`(OLED)가 유일한 외부 라이브러리입니다.
- `test/` — 자세한 내용은 [`TESTING.md`](TESTING.md)에 있습니다.
  - `pio test -e native -e native-stm32` — PC에서 387개 테스트(하드웨어 대역은
    `test/native/support/`), 커버리지는 `gcovr`.
  - `pio test -e esp32-s3` — 보드에서 `test_feedback/`(피드백 폐루프 시뮬레이션)과
    `test_imu_orientation/`을 실행합니다. 각각 테스트 펌웨어를 기록하므로, 그 뒤에
    `pio run -t upload`로 일반 펌웨어를 다시 올리세요.
- 정적 분석: `pio check -e esp32-s3`(cppcheck), `pio check -e stm32h743`(`hal/stm32/`와 `src/stm32/`에 대한
  cppcheck), `tools/clang-tidy.sh`(`.clang-tidy` 프로파일).

---

## 알려진 제한 사항

- **오토파일럿은 비행에서 시험하지 않았습니다.** 책상 위에서 부호를 실제로
  확인했습니다(기울임 → 수평으로 되돌리는 방향의 보정). PID 계수는 초깃값입니다.
- **STABILIZE는 스틱 위에 얹는 수평 복귀**이며, 스틱으로 롤/피치 각도를 지정하는
  “각도 모드”(FBWA)가 아닙니다. 조종자와 오토파일럿의 조작은 합산됩니다.
- **링크 상실 시의 활공은 비행에서 시험하지 않았습니다.** `FAILSAFE_GLIDE_*` 각도는
  초깃값이며, 피치 −3°는 개별 기체에 맞춰 정합니다(기수가 실속할 만큼 들려서도,
  급강하해서도 안 됩니다).
- **수평선.** 장착 보정(`o`)이 있으면 그 결과(NVS)에서 얻습니다. 가속도계의 영점
  오프셋은 온도에 따라 드리프트하며(20 °C당 약 1~2°), 수평선이 “틀어지면” `o`를
  다시 하세요. 보정이 없으면 전원을 켠 순간의 자세입니다(수평으로 놓고 켜세요).
- **나침반 장착**은 여전히 `MAG_ROTATION_CW_DEG`로 지정합니다(자세 기반 보정은
  이에 영향을 주지 않습니다).
- **나침반:** 방위는 기울기 보정이 없고, 각도를 세는 방향은 조립된 기체에서 확인하지
  않았으며, 보정은 기체에 장착한 상태로 해야 합니다. 방위를 쓰는 모드는 아직
  없습니다.
- **GPS**는 내비게이션에 쓰지 않습니다. ESP32-C3에서는 수신 전용입니다.
- **피드백(`autopilot/feedback/`)은 연결되어 있지 않으며** 조잡한 기체 모델의
  시뮬레이션에서만 검증했습니다. `FeedbackConfig.h`의 숫자 중 “прикидка”(“대략적인 추정”)라고 표시된 것은
  실제 기체에서 다듬어야 합니다. 대기속도 센서는 아직 없습니다(없으면 조종면 효과를
  더 느리게 학습하고, 실속은 감속으로만 알 수 있습니다).
- **실제 하드웨어에서 확인하지 않음:** `ICM42688_Sensor`(`ImuSensorBase`를 통해 공통
  규약에 맞춤), `BME280_Sensor`(Bosch 보정을 새로 구현), SPI 연결의 BMP388,
  `QMC5883L_Sensor`, CFG-VALSET을 통한 GPS 설정. 연결할 때는 부팅 로그, 콘솔의 `s`,
  기울여서 부호를 확인하세요.
- **브레드보드의 I2C는 잡음을 탑니다.** ESC/모터 때문이며, 간헐적인 오류는 `s`로
  보입니다. 드라이버는 이를 견디지만, 기체에서는 I2C 배선을 짧게 하고 전력선에서
  멀리 두세요.
- **ESP32Servo는 쓰지 않습니다.** 버전 3.2.1은 ESP32-S3에서 서보를 MCPWM에
  배분하고, `attachPin()`에서 MCPWM 유닛 번호와 타이머 번호를 혼동합니다. GPIO6/7이
  GPIO4/5의 신호를 내보냈습니다(ESC가 오른쪽 스틱으로 조종됨). 출력은 LEDC로 다시
  작성했으며, 라이브러리를 되돌리려면 `p`로 확인한 뒤에 하세요.
- **ESC는 50 Hz PWM**이며, 펌웨어에는 스로틀 범위 보정 모드가 아직 없습니다.
- **웹 대시보드:** 액세스 포인트의 비밀번호가 약하고, 비행 중에도 명령을 받습니다.
  테스트 벤치와 야외용 도구이며 비행 중에 쓰는 것이 아닙니다.
- **프로토타입의 기구:** 첫 프로토타입은 비행했으며, 모터 고정이 약하고 날개의
  강성이 부족하다는 점이 드러났습니다.
- **라이선스는 OpenPlane License**입니다([LICENSE](LICENSE.md)). 저작자 표시를
  의무로 하는 MIT이며, 군사적 이용을 금지하고, 사람과 재산에 대한 당사자의 서면
  동의 없는 의도적 위해를 금지합니다. 파일에 다른 라이선스 헤더를 추가하거나
  저작자 이름을 지우지 마세요.

---

## 변경 사항을 반영하는 방법

- **작은 커밋:** 논리적 단계 하나가 커밋 하나입니다.
- **커밋 전에 테스트와 분석:** `pio test -e native -e native-stm32`,
  `pio check -e esp32-s3`, `pio check -e stm32h743`, `tools/clang-tidy.sh`를
  모두 통과시킵니다([`TESTING.md`](TESTING.md)).
- **공통 코드를 고친 뒤에는 모든 보드를 빌드하세요.** S3가 주력이지만 C3, 38핀
  보드, `stm32h743`이 깨지면 안 됩니다. 릴리스 전에는
  `tools/build_matrix.sh`(모든 보드 × 모든 센서)를 실행합니다.
- **하드웨어에서 확인할 수 있는 것은 하드웨어에서 확인하세요:** 부호는 기울여서,
  출력은 `p` 명령으로, 링크는 송신기를 꺼서 확인합니다.
- **API를 지어내지 마세요.** `~/.platformio/packages/framework-arduinoespressif32/`의
  프레임워크 소스(Arduino core 2.0.x)와 대조하세요. 인터넷에는 API가 다른 3.x 버전을
  설명하는 글이 많습니다(예: LEDC).
- **상태를 미화하지 마세요.** 하드웨어에서 확인하지 않았다면 그대로 쓰세요.
- **계층은 자기 몫 이상을 알아서는 안 됩니다.** 하위 클래스가 갑자기 상위 클래스를
  필요로 한다면, 로직을 `FlightController`로 끌어올려야 합니다.
- **데이터 계약을 바꿀 때**(`FlightOutputState`, `ControlCommand`, `ImuData`,
  `/api/status`의 JSON)는 사용하는 쪽을 모두 같은 커밋에서 갱신하세요.
