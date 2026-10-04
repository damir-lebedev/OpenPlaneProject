# APPLICATION — `src/main.cpp`와 `src/stm32/main.cpp`

> 🌐 이 문서는 [러시아어 원문](../../../reference/application.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다.

[← 참조](README.md)

이 두 진입점은 각 보드의 **composition root**입니다. 펌웨어에서 유일한 번역 단위이자,
객체가 생성되어 참조로 서로 연결되는 유일한 장소입니다. 여기에는 비행 로직이 없고
객체 구성도 같습니다. 다른 것은 보드, 텔레메트리(Wi-Fi 또는 MAVLink), 그리고
비행 루프를 돌리는 방식입니다.

## 전역 객체(공통)

선언 순서 = 생성 순서입니다.

| 객체 | 타입 | 연결 |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice`, `imuSensor` | `SELECTED_IMU_DEVICE(board)`, `SelectedImu` | `SensorSelection.h`의 버스 |
| `baroDevice`, `baroSensor` | `SELECTED_BARO_DEVICE(board)`, `SelectedBaro` | 피토관이 있으면 이것이 정압 |
| `magDevice`, `magSensor`, `magnetometer` | … `SelectedMag`, `MagnetometerSensor* const` | `SENSOR_MAG != NONE`일 때만, 아니면 `nullptr` |
| `gpsSensor`, `gpsReceiver` | `SelectedGps`, `GpsSensor* const` | `SENSOR_GPS != NONE`일 때만 |
| `pitotDevice`, `pitotBaro`, `pitotSensor`, `airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`, `SelectedPitotBaro`(`"PITOT-BMP581"`), `PitotDualBaroAirspeed(pitotBaro, baroSensor)` | `SENSOR_AIRSPEED != NONE`일 때만 |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`, `throttleManager` | `ControlMixer`, `ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | 모든 센서(null 가능) |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`, `Controls::BINDINGS` 표 |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | 위의 모든 것 |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | 컨트롤러, 오토파일럿, 통계 |
| `debugConsole` | `DebugConsole` | 컨트롤러, 출력, 오토파일럿, 로그, `&board`(버스 조회 `b`) |
| `oledDisplay` | `OledDisplay` | 컨트롤러, 오토파일럿, 통계 |
| ESP32: `webDebugServer` | `WebDebugServer` | 컨트롤러, 오토파일럿 |
| STM32: `mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`, 컨트롤러, 오토파일럿, 통계 |

## `src/main.cpp` — ESP32(S3, C3, 38핀)

| 함수 | 설명 |
|---|---|
| `static void printBanner()` | `Serial`에 시작 화면 출력 |
| `static void setupSensors()` | 각 센서의 `begin()`. 응답한 센서의 보정: IMU `calibrate()`(2초간 정지 + 비행 전 점검), 기압계 `calibrateAltitude()`, 나침반은 25 ms 뒤의 첫 샘플로 IMU의 침로를 정함(`setYaw`). GPS `begin()`. 피토관 `begin()`(영점은 루프의 첫 1초 동안). `autopilot.begin()` |
| `void setup()` | `begin(115200)` **전에** `Serial.setTxBufferSize(4096)`, 시작 화면, `board.begin()`, `flightOutputs.begin()` + `setFailsafe()`, `setupSensors()`, `flightController.begin()`, OLED, 웹 서버, 스위치 배치, `debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`. 주기는 `vTaskDelayUntil(LOOP_PERIOD_MS)`. 100 ms를 넘게 늦으면 계수를 처음부터 다시 시작(“따라잡기”는 하지 않음) |

## `src/stm32/main.cpp` — STM32H743

env `stm32h743`의 진입점입니다(ESP32 빌드에서는 `src/stm32/` 디렉터리를
`build_src_filter`로 제외합니다). 실제 하드웨어에서는 센서 없는 DevEBox H743 보드를
확인했습니다(부팅, USB 콘솔, SD 카드, 블랙박스, iBUS, 서보와 모터의 수동 조종).
전체는 PC에서 `test/native_stm32` 테스트(env `native-stm32`)로 실행됩니다. 옆에는
SDMMC1의 핀과 클럭을 다루는 `sd_msp.cpp`와, 콘솔의 `D` 키(DFU로 재부팅)를 다루는
`bootloader.cpp`가 있습니다.

| 함수 | 설명 |
|---|---|
| `setup()` | `Serial.begin(115200)`, 시작 화면, `board.begin()`, 출력을 안전한 위치로, `Stm32FlashStorage::store().mount()`(설정 이미지: 비어 있음 / N바이트 / 손상 시 기본값), `setupSensors()`(ESP32와 같음), `flightController.begin()`, `mavlink.begin()`, OLED, 스위치 배치, `debugLogger.begin()`, 태스크 `flight`와 `storage`, `vTaskStartScheduler()`(반환하지 않음) |
| `static void flightTask(void*)` | 우선순위 `Rtos::PRIORITY_FLIGHT`, 스택 16 KB: `flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`. `vTaskDelayUntil(LOOP_PERIOD_MS)`, 100 ms를 넘게 늦으면 계수를 처음부터 다시 시작 |
| `static void storageTask(void*)` | 백그라운드: 100 ms에 한 번 `Stm32FlashStorage::instance().service()` 호출 — 설정 섹터를 지우고 기록하며, 비행 태스크에 선점됨 |
| `loop()` | 비어 있음: `vTaskStartScheduler()` 이후에는 태스크만 동작 |

콘솔(`Serial`, LPUART1 PA9/PA10, 115200)은 ESP32와 같은 `DebugConsole`입니다.
`h`는 메뉴, `s`는 센서, `b`는 버스 조회, `p`는 출력, 그리고 보정입니다.

## 불변 조건

- 출력은 센서를 초기화하기 **전에** 안전한 위치로 보냅니다
  (IMU 보정이 루프를 약 2초 붙잡아 두기 때문).
- ESP32: `Serial`의 TX 버퍼는 `begin()` 전에 설정합니다. STM32: UART 버퍼는
  `platformio.ini`의 `SERIAL_RX/TX_BUFFER_SIZE`입니다.
- 어떤 객체도 다른 객체를 소유하지 않습니다. 모든 참조는 소유권이 없고,
  수명은 프로그램 전체입니다.
- 스위치가 하는 일을 바꾸려면 `config/Controls.h`를 고칩니다. `main.cpp`가 아닙니다.
