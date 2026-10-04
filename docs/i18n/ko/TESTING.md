# TESTING.md: 테스트, 커버리지, 정적 분석

> 🌐 이 문서는 [러시아어 원문](../../TESTING.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다. 이 번역은 AI가 작성했으며 원어민의 검수를 거치지 않았습니다. 오류를 발견하면 [Damir Lebedev](https://github.com/damir-lebedev)에게 알려 주시거나 [이슈](https://github.com/damir-lebedev/OpenPlaneProject/issues)로 남겨 주세요.

펌웨어는 두 단계로 검증합니다.

| 위치 | 명령 | 내용 |
|---|---|---|
| **PC(native)** | `pio test -e native` | 펌웨어 헤더를 PC에서 그대로 빌드하고, 하드웨어는 제어할 수 있는 페이크로 대체합니다. 모듈, 드라이버, 폐루프 비행 시뮬레이션, 각 센서 세트를 갖춘 ESP32 펌웨어 전체(S3와 38핀)입니다. 커버리지를 계산합니다 |
| **PC(native-stm32)** | `pio test -e native-stm32` | STM32duino 페이크 계층 위에서 STM32H743 펌웨어 전체(`src/stm32/main.cpp`)를 돌립니다. FreeRTOS 태스크, 플래시, MAVLink, I2C와 SPI의 센서입니다 |
| **빌드 매트릭스** | `tools/build_matrix.sh` | 보드 4종 × 센서 세트 6종을 `-Wall -Wextra (-Wshadow)`로 빌드합니다. 프로젝트 코드에 경고가 하나라도 있으면 오류입니다 |
| **보드** | `pio test -e esp32-s3` | 실제 ESP32-S3에서 `test_feedback`과 `test_imu_orientation`을 실행합니다(테스트용 펌웨어를 올립니다. 끝나면 평소 펌웨어로 되돌리세요: `pio run -t upload`) |
| **STM32 보드** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | DevEBox H743의 **실제 SD 카드**에서 블랙박스를 시험하고, Cortex-M7에서 `test_feedback`과 `test_imu_orientation`도 실행합니다. [아래](#stm32-보드에서의-테스트) 참고 |

아키텍처 배경은 [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-테스트-용이성)에 있습니다.

---

## 빠른 시작

```bash
pip install platformio gcovr        # 한 번만
# Windows: PATH에 g++가 필요합니다. 예를 들어 WinLibs(winlibs.com, zip UCRT):
# 압축을 풀고 mingw64\bin을 PATH에 추가합니다. 설치는 필요 없습니다
pio test -e native -e native-stm32  # 모든 네이티브 테스트(약 1.5분)
gcovr                               # 파일별 커버리지(설정은 gcovr.cfg)
tools/build_matrix.sh               # 모든 보드 × 모든 센서(약 25분)
gcovr --html-details -o coverage/index.html   # HTML 보고서(coverage/는 .gitignore에 있음)

pio test -e native -f native/test_rc          # 세트 하나
pio test -e native -f test_feedback           # PC에서 피드백 시뮬레이션

# 폐루프 시뮬레이션의 궤적을 CSV로(그래프용):
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# MAVLink 스트림을 기준 디코더로 검사(pip install pymavlink):
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

테스트를 수정한 뒤 커버리지를 계산하기 전에는 깨끗한 빌드에서 시작하는 것이 좋습니다: `rm -rf .pio/build/native`. 그렇지 않으면 이전 실행의 카운터가 보고서에 섞여 들어갑니다.

---

## 네이티브 빌드의 구조

`platformio.ini`의 `[env:native]`: `platform = native`, Unity, `-std=gnu++17`, `-D BOARD_ESP32_S3`(주력 보드의 핀 배치), `-I test/native/support`, `-Wall -Wextra -Wshadow`, `--coverage`로 하는 커버리지 계산, 그리고 `-fkeep-inline-functions -fkeep-static-functions`입니다. 이 두 옵션이 없으면 한 번도 호출되지 않은 헤더 함수를 gcov가 보지 못해 커버리지가 부풀려집니다.

### 하드웨어 페이크: `test/native/support/`

ESP32 2.0.x용 Arduino 코어, ESP-IDF, 라이브러리와 이름과 시그니처가 같은 헤더이지만, 내부는 `namespace fake`에 있는 시뮬레이션된 세계 위에서 동작합니다.

| 파일 | 대체하는 것 | 시뮬레이션이 할 수 있는 일 |
|---|---|---|
| `Arduino.h`, `Print.h`, `WString.h`, `Stream.h` | Arduino 코어 | 매크로(`constrain`, `sq`, `DEG_TO_RAD` 등), `map()`, `String`, 원본과 같은 `print()` 서식. `ARDUINO`는 일부러 정의**하지 않습니다** |
| `esp32-hal-fake.h` | 시간, GPIO, ADC, LEDC, FreeRTOS, PSRAM, `ESP` | 시계는 `fake::advance*()`/`delay()`로만 흐릅니다. `millis()/micros()`는 ESP32처럼 `uint32_t`이며 오버플로도 실제 보드와 똑같이 동작합니다. LEDC 채널, 실제 듀티비에 따른 `pulseIn`(핀의 입력 버퍼가 켜져 있을 때만 보임), `analogReadMilliVolts`(전압은 `fake::gpio().analogMv`에서 가져옴). 태스크는 등록되고(핸들은 널이 아님), `fake::runTask(task, n)`은 그 무한 루프를 n번 돌리며, `ulTaskNotifyTake`는 한 번 돈 것으로 치고 `xTaskNotifyGive`는 카운터로 셉니다. FreeRTOS 뮤텍스는 "사용 중" 플래그입니다. `psramFound()`/`ps_malloc()`. 크리티컬 섹션은 횟수를 셉니다 |
| `HardwareSerial.h` | UART | 포트는 번호로 등록됩니다(`fake::uart(1)`). `pushRx()`, `txBytes()`, 동작 중 속도 변경(`updateBaudRate`, 이력은 `baudChanges()`). `Serial` = UART0 |
| `esp_partition.h` | ESP-IDF 플래시 파티션 | 파티션은 NOR처럼 동작하는 바이트 벡터입니다. 삭제는 4 KB 섹터 단위로만, 삭제된 값은 0xFF, 쓰기는 비트를 내리기만 하며(비트를 올리려는 시도는 `bitRaises`로 셉니다), `beforeWrite`는 "전원이 나갔다"는 상황이고, 읽기·쓰기·삭제 횟수 카운터가 있습니다 |
| `esp_system.h` | 재부팅 원인 | `fake::chip().resetReason`에서 가져오는 `esp_reset_reason()` |
| `Wire.h` | I2C | 주소별 장치. `fake::RegisterMapDevice`는 자동 증가 레지스터, 쓰기 로그, 고장(`present`, `failWrites`, `failReads`, `failReadIf`, `shortRead`), 훅 `beforeRead`/`onRegisterWrite`를 갖습니다 |
| `SPI.h` | SPI | CS 핀별 장치. `fake::SpiRegisterMapDevice`는 Bosch/InvenSense 프로토콜이며, 데이터 앞에 `dummyBytes`를 둡니다 |
| `Preferences.h` | NVS | 메모리 안의 저장소. `begin(readOnly)`/`get*`/`getBytes`의 동작은 원본과 같습니다. `failBegin` |
| `WiFi.h`, `WebServer.h` | Wi-Fi AP, HTTP | `softAP()`의 결과는 테스트가 정합니다. `WebServer::request(메서드, uri, 본문)`가 등록된 핸들러를 호출합니다. `fake::webServers()`는 모든 인스턴스입니다 |
| `U8g2lib.h` | U8g2 | 픽셀 대신 그려진 문자열과 사각형의 목록을 가집니다. `begin()/sendBuffer()`는 사용자 바이트 콜백으로 바이트를 넘깁니다. `fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`, `GPIO_PIN_MUX_REG`, `PIN_INPUT_ENABLE` |

### STM32duino 계층: `test/native/support_stm32/`(환경 `native-stm32`)

`-I`에서 `support/`보다 앞에 놓이며, 같은 페이크에 STM32duino에만 있는 것을 보태 줍니다. 이 환경의 `<Preferences.h>`는 `KeyValueStore` 위에 만든 **진짜** `include/hal/stm32/compat/Preferences.h`입니다.

| 파일 | 대체하는 것 | 할 수 있는 일 |
|---|---|---|
| `Arduino.h` | STM32duino 코어 | 핀 `PA0..PE15`(포트·16 + 번호), `pin_size_t`, 서보 출력 핀용 `PinMap_TIM`, `HardwareTimer`(펄스는 `fake::timerPulseUs(pin)`와 `pulseIn()`으로 보임), `Uart`, `noInterrupts()` |
| `STM32FreeRTOS.h` | STM32duino의 FreeRTOS | 공통 태스크 등록부로 들어가는 `xTaskCreate`(스택은 워드 단위), 반환되는 `vTaskStartScheduler()`(태스크는 테스트가 직접 돌립니다: `fake::runTask`), `xPortGetFreeHeapSize` |
| `EEPROM.h` | EEPROM 에뮬레이션 | 8 KB "플래시"(삭제된 값은 0xFF)와 버퍼, `fake::eeprom()`(카운터와 이미지 손상) |
| `SPI.h` | | `SPIMode` |

STM32용으로 공통 페이크에 추가한 것: `TwoWire(sda, scl)`, `setSDA/SCL`과 `fake::wireWithSda(pin)`(보드의 두 번째 버스를 찾기 위함), `HardwareSerial(rx, tx)`와 `fake::uartByRx(pin)`, `SPIClass::setSCLK/MISO/MOSI`.

### 칩 에뮬레이터와 비행기 모델: `test/native/helpers/`

| 파일 | 내용 |
|---|---|
| `ChipEmulators.h` | LSM6DSV, ICM-45686(IPREG 간접 레지스터 포함), QMC6309, SPL06-001, BMP581, u-blox NAV-PVT 프레임. I2C 또는 SPI 위의 레지스터 맵이며, 데이터는 `World` "세계"(각도와 속도, 고도, 대기속도, 침로, 좌표)에서 가져와 `IMU_ROTATION_CW_DEG`를 반영한 칩의 축으로 돌려줍니다 |
| `PlaneSim.h` | 약 1.2 kg 비행기 모델: 질점 + 롤/피치 회전, 실속을 포함한 CL(α), 항력, 추력, 바람, 서멀, 지면 |
| `SimHarness.h` | 폐루프: 조종기 → iBUS 프레임 → `IBusReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → PWM → 조종면 꺾임 → `PlaneSim` → 센서(노이즈가 있는 기압계 두 개로 만든 피토관 포함). `OPENPLANE_SIM_DIR`를 지정하면 CSV 궤적을 만듭니다 |

`test/native/helpers/TestSupport.h`는 여러 세트가 함께 쓰는 부분입니다. `resetWorld()`(`setUp()`에서 호출), `FakeUart`/`FakeServo`/`FakeBoard`와 센서(`FakeImu`, `FakeBaro`, `FakeMag`, `FakeGps`)의 대역, 프레임 조립기 `ibusFrame()`, 진짜 `Esp32I2CBus`/`Esp32SpiBus`와 시뮬레이션된 칩이 붙은 `*RegisterDevice` 위에 드라이버를 얹는 시험대 `I2cRig`/`SpiRig`입니다.

보드용 테스트(`test_feedback`, `test_imu_orientation`)는 이식이 가능합니다. `ARDUINO`가 있으면 `setup()/loop()`, 없으면 `main()`입니다. `test/native/*` 세트는 보드용으로 빌드하지 않습니다(`[esp32_common]`과 `[env:stm32h743]`의 `test_ignore`: 패턴은 한 줄에 하나씩 적습니다. 공백으로 구분하면 PlatformIO가 하나로 읽습니다). STM32에서는 `pio test -e stm32h743`입니다.

---

## 테스트 세트

| 세트 | 테스트 수 | 검증 내용 |
|---|---|---|
| `native/test_hal` | 17 | `II2CBus` 도우미(NACK, 짧은 읽기: 버퍼는 건드리지 않음), `I2cRegisterDevice`, `SpiRegisterDevice`(읽기 비트, BMP388의 더미 바이트), `Esp32I2CBus`(5 ms 타임아웃), `Esp32SpiBus`(모드 0–3), `Esp32UartPort`(8N1, 핀), `Esp32ServoOutput`(50 Hz/14비트, 펄스 제한, LEDC 고장, 입력 버퍼를 통한 측정), `Esp32Board`(버스, UART, 채널 순서, AUX, 부저) |
| `native/test_rc` | 16 | `RcChannelState`, `RcInput`, iBUS 파싱: 조각으로 도착하는 프레임, CRC, 12비트 값, 조종기의 페일세이프, 500 ms 타임아웃(`micros()` 오버플로를 넘어가는 경우 포함), 쓰레기 데이터, 재동기화 |
| `native/test_control` | 21 | 플랩(속도, 첫 호출, 일시 정지), 믹서(부호, 리버스, 플래퍼론), 스로틀, ARM 상태 머신과 모드 센서 확인, 출력 표와 펄스 자가 점검 |
| `native/test_autopilot` | 22 | PID(센서 각속도 기반 D항, 적분, 안티 와인드업, `dt`), 각도 모드로서의 STABILIZE, 시간 기반 자동 이륙, 승강타를 쓰는 ALT_HOLD, 신호 상실 시 활공 |
| `native/test_autopilot_modes` | 31 | 12개 모드 전체와 센서가 없을 때 각 모드의 반응, 바인딩 표와 `static_assert`, 기능과 노브, 항법(침로, 원, 홈 지점, 지오펜스), failsafe RTH/활공, 손으로 던져 이륙, 소어링, 오토 트림(기록은 지상에서만) |
| `native/test_flight_controller` | 12 | 실제 클래스로 돌리는 `FlightController`의 한 주기: 우선순위는 신호 상실 > ARM > 스틱/오토파일럿 > 스로틀, AUX, `MOTOR_KILL`, 부저 |
| `native/test_imu` | 21 | MPU6050/6500/9250과 ICM-42688: 식별, 레지스터, 스케일, 축 회전과 항공 부호, 버스 오류, 자이로 보정과 비행 전 점검, 세 가지 자세로 하는 장착 보정, NVS, 자세 필터 |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`, I2C와 SPI의 BMP388, Bosch 기준 구현과 비교한 BME280/BMP280, 나침반(침로, NVS에 저장하는 하드 아이언 보정), u-blox M10(CFG-VALSET, NAV-PVT, 손상된 프레임, 타임아웃), `SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV(16X/32X, 예비 주소, SPI), ICM-45686(간접 레지스터), QMC6309, SPL06-001(데이터시트 공식), BMP581(DRDY와 대체 경로), 피토관(영점, 필터, 밀도, 바뀐 호스, 오래된 데이터, 기압계 두 개의 노이즈를 반영한 “비행”) |
| `native/test_storage` | 16 | `KeyValueStore`(재로드, 마모: 같은 값은 쓰지 않음, 데이터 손실 없는 오버플로, CRC, 삭제 중 전원 차단, 쓰레기 데이터, 포맷 버전), `KvPreferences`(ESP32 NVS와 같은 동작) |
| `native/test_mavlink` | 20 | pymavlink 기준 프레임과 대조하는 코덱(v1, v2, 서명된 것), CRC, 재동기화, 텔레메트리: 스트림 주파수, HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS, PID 파라미터(목록, 읽기, 쓰기, 잘못된 값 거부), 지상에서의 모드 변경, 지상에서의 ARM은 거부, 미션은 0개, 가득 찬 UART 버퍼가 루프를 막지 않음 |
| `native/test_blackbox` | 19 | 블랙박스: 포맷과 CRC, NOR 페이크 위의 섹터 링(지우지 않은 새 파티션, 쓰레기는 항상 삭제, 오래된 비행은 통째로 그리고 공간 확보를 위해서만 삭제, 마지막 비행은 건드리지 않음, 링 끝을 넘어가는 경우, 재부팅 뒤의 머리 위치, 전원 차단, 쓰다 만 레코드를 CRC로 감지), 실제 `FlightController`/`Autopilot`으로 하는 비행 기록: ARM과 스로틀로 시작(사전 기록 포함), DISARM과 “지상에 정지” 뒤 중지, 신호 상실로는 멈추지 않음, 비정상 재부팅 뒤의 기록, 수동 시작, 이벤트, 배터리, 공중에서 플래시가 다 참, 파티션보다 긴 비행, CRC 프레임과 속도 전환을 쓰는 내려받기, 콘솔 메뉴 `k`, 파티션이 없으면 비활성 |
| `native/test_blackbox_scan` | 3 | 전원을 켤 때 하는 링의 표본 검증과 전체 검증의 비교: 링의 무작위 이력 300개 × 탐색 5단계(머리 위치, 번호, 비행 목록이 일치하며, 맞지 않을 때는 전체 검증에 양보), 그리고 64 MB SD 영역에서의 비용(32,000번 대신 약 530번 읽기) |
| `native/test_telemetry` | 28 | `LoopStats`, `LogSettings`(NVS, 버전), `DebugLogger`(모든 채널, NAV), `DebugConsole`(메뉴, 단축키, 버스 탐색 `b`, ARM 중 금지, ARM이 아닐 때만 저장), `WebDebugServer`(경로, JSON, 우편함), `OledDisplay`(I2C 바이트, 프레임, 신호 상실 시 반전) |
| `native/test_sim` | 15 | 비행기 모델로 하는 펌웨어 전체의 폐루프 비행: 뱅크에서의 회복, 옆바람 속 CRUISE, LOITER, RTH, failsafe RTH/활공, 지오펜스, 활주로에서의 자동 이륙, 손으로 던져 이륙, 자동 착륙, 서멀, 나선에서의 RESCUE, 속도 유지와 실속 방지, 루프 안의 실제 피토관, “삐뚤어진” 비행기의 오토 트림, 비행 중 센서 고장(IMU, 기압계, 피토관, GPS) |
| `native/test_feedback_units` | 14 | 피드백 모듈을 하나씩: 속도 소스, 공중/지상 판정, RLS 추정, 제어기, 실속 징후, 이륙/착륙 중단 |
| `native/test_app` | 10 | ESP32-S3의 `src/main.cpp`, 벤치용 구성 MPU6500/BMP388/QMC5883P/OLED: `loop()` 주기, 조종기 → 서보, ARM, 모드, 신호 상실, 콘솔, 대시보드, 화면, 블랙박스(코어 0의 태스크, 스로틀 시 기록, DISARM 뒤의 비행, `bb list`) |
| `native/test_app_lsm6dsv_pitot` | 9 | ESP32-S3의 `src/main.cpp`, 비행용 구성: LSM6DSV + QMC6309 + SPL06 + 피토관 속 BMP581 + GPS: 모든 칩 식별, 피토관 영점과 속도, 고도, GPS 기준 홈 지점, 칩의 각도로 하는 STABILIZE, 홈 지점으로 가는 RTH, 버스 탐색, 대시보드 |
| `native/test_app_icm45686_esp32dev` | 5 | **ESP32 38핀**(`BOARD_ESP32_CLASSIC`)의 `src/main.cpp`, ICM-45686 + QMC6309 + SPL06 + BMP581 구성: 보드 핀 배치, IPREG 필터, 손으로 던져 이륙, 안정화와 속도, 버스 하나만 탐색 |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | **STM32H743**의 `src/stm32/main.cpp`, 비행용 구성: 태스크와 우선순위, 2 ms 주기, 피토관, PWM 타이머와 `pulseIn`, 비행 중 MAVLink, GCS에서의 모드 변경, 백그라운드 태스크가 설정을 “플래시”에 기록, I2C1의 화면, 콘솔 |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743과 **SPI로 연결한** ICM-45686, BMP581 + QMC6309: 전원을 켤 때 손상된 플래시, GCS의 ALT_HOLD가 고도를 유지, 신호 상실 → RTH, MAVLink에서 확인 가능, 손상된 이미지 다시 쓰기 |
| `native_stm32/test_blackbox_sd` | 29 | SD 카드의 블랙박스: FAT32(MBR 있음/없음, 클러스터 두 개에 걸친 디렉터리, 노이즈 항목, 남의/흩어진/빈 볼륨), `SdFileRegion`(불완전한 블록, 캐시, 삭제, 경계, 고장), 가짜 `HAL_SD` 위에서 도는 실제 `Stm32SdCard` 드라이버(4비트, 예비 속도, 재시도, 사용 중인 카드, 정렬되지 않은 버퍼), 카드 위의 링(재부팅, 전원 차단, 검증 비용), “링이 비어 있음” 표시, `FlightController`에서의 비행 기록, `RCC->RSR`로 판별하는 비정상 재부팅, 배터리 ADC, 비행 중 카드 오류, 느린 카드, 콘솔을 통한 내려받기, `D` 키 |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | 카드가 있는 `src/stm32/main.cpp`: 부팅 시 카드와 파일을 찾음, `bbox` 태스크가 비행을 기록, 루프 주기가 늘어나지 않음, `bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | 카드가 없는 `src/stm32/main.cpp`: 블랙박스는 비활성화되고 이유를 설명함, 비행기는 비행함, 메뉴 `k`는 망가지지 않음 |
| `test_feedback` | 10 | 피드백 루프가 있는 비행기의 폐루프 시뮬레이션(PC와 보드 모두) |
| `test_imu_orientation` | 5 | 무작위 장착 300가지에서 하는 IMU 장착 보정(PC와 보드 모두) |
| **합계** | **387** | `native`에 340개 + `native-stm32`에 47개(그 밖에 보드에서만 도는 9개: `test_blackbox_sd`) |

### STM32 보드에서의 테스트

`test/test_blackbox_sd`는 네이티브가 아닙니다. SDMMC 드라이버, 카드, 시간이 모두 실제입니다. 테스트는 FreeRTOS 태스크에서 돌고, 그 옆에서 가장 높은 우선순위(주기 2 ms)로 비행 루프를 흉내 내는 태스크가 돕니다. 이 태스크는 펌웨어에서와 똑같이 카드에 접근하는 도중에 테스트를 선점합니다. 이것이 없으면 보드에서 실제로 발견된 오류를 잡을 수 없습니다. 선점될 때 SDMMC의 FIFO가 넘쳤으며(`HAL_SD_ERROR_RX_OVERRUN`), 아무것도 없는 단순한 루프에서는 이런 일이 일어나지 않습니다.

| 테스트 | 검증 내용 |
|---|---|
| `reset_cause_is_a_normal_one` | 리셋 원인(`RCC->RSR`)이 워치독도 전압 강하도 아님 |
| `card_is_detected_on_four_bit_bus` | 카드가 4비트 버스, 24 MHz로 인식됨 |
| `file_is_found_and_contiguous` | FAT32에서 `BLACKBOX.BIN`을 찾았고 연속으로 놓여 있음 |
| `multi_block_writes_work_at_every_length` | 1, 2, 4, 8블록을 한 번의 접근으로 쓰기 |
| `pages_write_with_bounded_latency_and_read_back_intact` | 256 B 페이지: 최악의 쓰기가 250 ms 미만(SD의 한계), 안정적으로 40 KB/s 초과, 읽기와 삭제 |
| `header_scan_cost_on_the_whole_area` | 섹터 헤더 읽기와 전체 검증의 비용 |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | 전체 삭제, 레코드 20,000개씩 두 번의 비행, “재부팅”: 표본 검증이 2 s 미만, 레코드가 올바른 CRC로 순서대로 읽힘. 빈 링은 표시를 보고 100 ms 미만에 판별 |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | 실제 `BlackBox`를 500 Hz IMU로 실시간 구동: 잃어버린 레코드가 하나도 없고, “재부팅” 뒤에도 비행이 읽힘 |
| `the_flight_task_was_not_disturbed` | 카드에 쓰는 동안 모의 태스크의 주기가 흐트러지지 않음(편차 3 ms 미만) |

실행 방법(파일이 들어 있는 카드: `python tools/blackbox.py sd-prepare E:`. **이 테스트는 파일의 모든 비행을 지웁니다**):

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

보드는 DFU 모드여야 합니다(DevEBox에서는 BT0→3V3 배선과 RST, Zadig로 설치하는 WinUSB 드라이버. 자세한 내용은 [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743) 참고). STM32 콘솔은 USB CDC입니다. 펌웨어를 올린 직후에는 포트가 바로 나타나지 않으며, `pio test`가 제때 열지 못할 때가 있습니다(“could not open port”). 그럴 때는 `pio test ... --without-testing`을 실행하고, DTR을 켠 아무 터미널 프로그램으로 출력을 읽으세요(테스트는 포트가 열릴 때까지 최대 60 s 기다립니다). 테스트가 끝나면 보드는 **`D`** 키를 기다립니다. 이 키를 누르면 배선 없이 DFU로 재부팅됩니다.

DevEBox H743 + 16 GB 카드에서의 결과(2026-10-02): `test_blackbox_sd`는 9/9, `test_feedback`은 10/10, `test_imu_orientation`은 5/5입니다. 카드 속도 수치는 [BLACKBOX.md](BLACKBOX.md#보드에서-측정한-결과)에 있습니다.

### 벤치용 펌웨어: `test/bench/`

이것들은 테스트 세트가 아니라, 비행용 펌웨어 대신 보드에 올리는 독립적인 작은 PlatformIO 프로젝트입니다(`pio test`에는 보이지 않습니다. 폴더 이름이 `test_`로 시작하지 않기 때문입니다). 핀과 한계값은 공용 `Config.h`에서 가져옵니다.

| 프로젝트 | 하는 일 |
|---|---|
| `bench/elevator_sweep` | `ControlMixer`와 `FlightOutputs`를 거쳐 승강타 스틱(CH2)을 프로그램으로 움직입니다. 실제 스틱처럼 위로 이동 범위의 100%, 아래로 60%를 부드럽게, 중간에 멈춤을 두고 움직입니다. 20 s 동안 움직이면 20 s는 중립에 둡니다. 양 끝 위치에서는 출력의 펄스를 측정합니다. 스로틀은 최소로 둡니다 |

올리기: `pio run -d test/bench/elevator_sweep -t upload`. 비행용 펌웨어로 되돌리기: `pio run -e esp32-s3 -t upload`.

---

## 커버리지

`gcovr`가 `include/`와 `src/`(펌웨어에 들어가는 모든 것)를 대상으로, 두 네이티브 환경을 합쳐서 계산합니다: `gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`.

| 계층 | 줄 | 분기 |
|---|---|---|
| `autopilot` | 920/943 (97.6%) | 645/731 (88.2%) |
| `autopilot/feedback` | 683/702 (97.3%) | 501/570 (87.9%) |
| `control` | 252/256 (98.4%) | 171/189 (90.5%) |
| `hal` | 98/102 (96.1%) | 26/26 (100%) |
| `hal/esp32` | 101/102 (99.0%) | 21/22 (95.5%) |
| `hal/stm32` | 149/158 (94.3%) | 35/52 (67.3%) |
| `rc` | 92/92 (100%) | 41/42 (97.6%) |
| `sensors`(전체) | 1444/1446 (99.9%) | 716/835 (85.7%) |
| `storage` | 220/220 (100%) | 158/178 (88.8%) |
| `telemetry` | 1413/1440 (98.1%) | 1123/1269 (88.5%) |
| `src`(`main.cpp`, `stm32/main.cpp`) | 118/123 (95.9%) | 20/29 (69.0%) |
| **합계** | **5490/5584 (98.3%)** | **3457/3943 (87.7%)**, 함수 877/902 (97.2%) |

아직 커버되지 않은 부분과 그 이유:

- **손으로 던져 이륙**(`TakeoffSequencer`: `WaitLaunch`, `launchDetected()`): `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false`인 동안에는 도달할 수 없습니다. 이 상수를 설정할 수 있게 되면(`Config.h`로 옮기면) 테스트에 들어옵니다.
- **보드에 의존하는 부분**: 핀이 없는 출력(`PIN_RUDDER = -1`은 C3에서만 나옵니다), TX 핀이 없는 GPS(C3). 네이티브 테스트는 S3, 38핀, STM32의 핀 배치를 돌려 보지만 C3는 돌리지 않습니다(C3는 빌드 매트릭스로 확인합니다).
- **STM32**: 코어의 오류 분기(핀에 타이머가 없음, 타이머 풀이 바닥남), 메시지 `FreeRTOS не запустился`(“FreeRTOS가 시작되지 않았다”): PC에서는 `vTaskStartScheduler()`가 항상 반환됩니다.
- **방어용 분기**로, 공개 API를 통해서는 도달할 수 없는 것: 열거형에 대한 `switch`의 `default`/`Count`, `return "?"`.
- 실행 가능한 줄이 없는 파일(`Config.h`, `Channels.h`, `FeedbackConfig.h`, 구조체 `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets`, `SensorSelection.h`의 매크로, 대시보드 HTML)은 보고서에 나타나지 않습니다. 테스트에는 컴파일되지만 gcov가 셀 것이 없기 때문입니다.

---

## 정적 분석

| 도구 | 명령 | 프로필 |
|---|---|---|
| GCC | `tools/build_matrix.sh`(또는 `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`) | 모든 보드 × 모든 센서 구성. 네이티브 테스트 빌드는 항상 `-Wall -Wextra -Wshadow`로 합니다. `stm32h743`은 `-Wall -Wextra`입니다(`build_src_flags`. `-Wshadow`는 STM32duino 자체 헤더에서 노이즈를 내기 때문입니다) |
| cppcheck | `pio check -e esp32-s3`; `pio check -e stm32h743` | `[esp32_common]`의 `check_*`: `include/`와 `src/`(`stm32/` 제외), warning/style/performance/portability, 줄 안의 `// cppcheck-suppress`는 오탐에만 사용(U8g2 콜백, `setup/loop`). `stm32h743`은 `include/hal/stm32/`와 `src/stm32/`에 같은 플래그 |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`: bugprone, clang-analyzer, performance, `misc-include-cleaner` 등. 끈 검사의 이유는 파일 안에 설명되어 있습니다 |

clang-tidy는 `test/native/support`의 페이크와 함께 실행합니다. clang이 호스트 아키텍처용으로 ESP-IDF 헤더를 해석하지 못하기 때문입니다(`pio check`를 `clangtidy`로 시도하면 분석이 파싱 오류에서 중단되어 사실상 아무것도 검사하지 않습니다). `misc-include-cleaner`는 각 헤더가 자기가 쓰는 것을 직접 포함하는지 살핍니다. “우산” 헤더(`FeedbackModules.h`, `IBoard.h`/`RegisterDevice.h`의 API, `SensorSelection.h`의 매크로)에는 `// IWYU pragma: export`를 붙여 두었습니다. 스크립트는 STM32 코드(`include/hal/stm32/`, `src/stm32/`)를 건너뜁니다. 이 코드는 빌드, 환경 `stm32h743`의 cppcheck, 환경 `native-stm32`의 테스트로 확인합니다.

마지막으로 실행한 시점의 빌드 매트릭스: **24/24 경고 없음**

| 보드 | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck(`esp32-s3`, `stm32h743`): 프로젝트 코드에 대한 지적은 0건입니다.

---

## 새 테스트 작성 방법

1. 하드웨어 없이 로직만 있는 모듈: 직접 단위 테스트를 씁니다. 시간은 매개변수로 넘기거나 `fake::advanceMs()`로 진행시킵니다.
2. 칩 드라이버: `I2cRig`/`SpiRig`를 이용합니다. 시뮬레이션한 칩의 레지스터, 기록된 값(`chip.lastWrite(reg)`)과 데이터 해석을 검증합니다. 공식은 코드의 복사본이 아니라 데이터시트의 기준값이나 독립적인 계산으로 검증합니다.
3. FreeRTOS 무한 태스크가 있는 클래스: `fake::findTask("이름")` + `fake::runTask(task, n)`. STM32의 비행 태스크도 같은 방식으로 돌립니다.
4. 다른 센서 구성이나 다른 보드로 펌웨어 전체를 돌리는 경우: 별도 세트를 만들어 `#include "../../../src/main.cpp"` 앞에서 `SENSOR_KIT`을 정의합니다(또는 `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`). 칩은 `helpers/ChipEmulators.h`의 것을 씁니다. STM32는 `test/native_stm32/`를 사용합니다.
5. 새 오토파일럿 모드: `test_sim`에 폐루프 비행 시나리오를 추가합니다.
6. 새 세트: `main()`이 있는 폴더 `test/native/test_<이름>/test_main.cpp`를 만듭니다. 테스트 사이에 상태가 필요 없는 세트라면 `setUp()`에서 `resetWorld()`를 호출합니다.
7. 버그를 찾았다면: 먼저 그것을 잡아내는 테스트를 쓰고, 그다음에 고칩니다.
