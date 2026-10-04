# TELEMETRY — 로그, 콘솔, 웹 대시보드, OLED, 블랙박스

> 🌐 이 문서는 [러시아어 원문](../../../reference/telemetry.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다.

[← 참조](README.md)

텔레메트리는 비행 로직과 완전히 분리되어 있습니다. 읽는 것은
`FlightController`, `Autopilot`, 센서, `LoopStats`의 const 게터뿐입니다.
“거꾸로” 가는 유일한 경로는 대시보드 명령이며, 이는 `WebDebugServer`의
메일박스를 거쳐 비행 루프가 적용합니다.

---

## `LoopStats`

**파일:** `telemetry/LoopStats.h` · **종류:** struct

비행 루프의 주파수와 소요 시간입니다.

| 멤버 | 설명 |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | 1초에 한 번 공개되며, 다른 태스크에서 읽습니다 (32비트 값이라 “찢어진” 읽기가 없음) |
| `void record(uint32_t durationUs)` | 매 틱마다 `loop()`에서 호출합니다 |
| `uint32_t takePeakUs()` | 지난 호출 이후 가장 나빴던 틱 (10초마다 나오는 SYS 줄용). `record()`와 같은 태스크에서 호출해야 합니다 |

`maxUs`는 직전 1초 동안의 최악 값일 뿐이며, 드물게 생기는 끊김은
`takePeakUs()`로 확인할 수 있습니다.

---

## `LogSettings`

**파일:** `telemetry/LogSettings.h` · **의존:** `Preferences` (NVS, 네임스페이스 `debuglog`)

### `LogChannel` (enum class)

| 채널 | 접두어 | 출력하는 내용 | 기본값 |
|---|---|---|---|
| `Status` | `STAT` | 링크, ARM, 모드, 플랩, 센서 | 변경 시 |
| `Rc` | `RC` | 송신기 채널 | 꺼짐 |
| `Outputs` | `OUT` | 조종면과 ESC로 나가는 출력 | 꺼짐 |
| `Attitude` | `ATT` | 롤, 피치, 방위 | 꺼짐 |
| `Autopilot` | `AP` | 목표와 보정값 | 꺼짐 |
| `Altitude` | `ALT` | 고도, 수직 속도 | 꺼짐 |
| `Heading` | `MAG` | 나침반 방위 | 꺼짐 |
| `Gps` | `GPS` | 위성, 좌표 | 꺼짐 |
| `Imu` | `IMU` | 자이로와 가속도계 | 꺼짐 |
| `Nav` | `NAV` | 홈, 방위, 속도, 피토관, 켜 둔 기능 | 꺼짐 |
| `System` | `SYS` | 루프 주파수, 메모리 (10초마다), 꺼짐/켜짐만 | 켜짐 |
| `Count` | — | 채널의 수 | — |

`LogMode` (enum class): `Off`, `OnChange`, `Periodic`.

`LogChannelInfo`: `tag`, `title`, `periodicOnly`, `defaultMode`.

| 메서드 | 설명 |
|---|---|
| `static constexpr uint8_t COUNT`, `PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | 채널 표의 한 행 |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms (순환) |
| `LogSettings()`, `void setDefaults()` | 기본 모드, 주기는 1초 |
| `LogMode mode(uint8_t)`, `mode(LogChannel)` | 채널의 모드 |
| `void setMode(uint8_t, LogMode)` | `periodicOnly` 채널에서는 `OnChange`가 `Periodic`이 됩니다 |
| `void cycleMode(uint8_t)` | 꺼짐 → 변경 시 → 계속 → 꺼짐 (SYS: 꺼짐 ↔ 켜짐) |
| `void setAll(LogMode)` | 모든 채널에 적용합니다. “모두 변경 시” 명령은 SYS를 건드리지 않습니다 |
| `uint16_t periodMs() const`, `void cyclePeriod()` | “계속” 모드의 주기 |
| `static const char* modeName(LogMode, bool periodicOnly)` | “꺼짐” / “변경 시” / “계속” (또는 “켜짐”) |
| `void load()` | NVS에서 불러옵니다. `VERSION`이나 길이가 맞지 않으면 기본값이 남고, 알 수 없는 모드 코드는 해당 채널의 기본값이 됩니다 |
| `void save() const` | NVS에 저장합니다 (모드, 주기, 버전) |

`VERSION`은 채널 목록과 함께 바뀌며, 이전 설정은 초기화됩니다
(`VERSION = 2`: NAV 채널 추가). 메뉴에서의 채널 키는 `1`..`9`, NAV는
`n`, SYS는 `s`입니다.

---

## `DebugLogger`

**파일:** `telemetry/DebugLogger.h` · **의존:** `FlightController`, `Autopilot*`, `LoopStats*`, `LogSettings`, `Config`

상태를 채널별로 시리얼 모니터에 출력합니다. 채널마다 고유한 줄, 고유한
모드, 고유한 허용 오차가 있습니다.

| 메서드 | 설명 |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | `DEBUG_INTERVAL_MS`마다 한 번씩 채널을 순회합니다 (일시 정지 중이거나 메뉴가 열려 있는 동안은 조용히 있습니다) |
| `LogSettings& getSettings()`, `void saveSettings() const` | 콘솔 메뉴용 |
| `void suspend(bool)` | 메뉴가 열려 있으면 조용히 있습니다. 해제하면 `refresh()` |
| `void setPaused(bool)`, `bool isPaused() const` | 스페이스 바로 일시 정지합니다. 해제하면 `refresh()` |
| `void refresh()` | 다음 틱에 켜져 있는 모든 채널을 출력합니다 |

채널의 동작 (`updateChannel`):

- `Off` — 출력하지 않습니다.
- `Periodic` — `periodMs()`마다 한 번 (SYS는 10초마다 한 번), 값은 “있는 그대로”.
- `OnChange` — 줄은 **허용 오차**를 적용해 조립합니다 (중첩된 `Shown`은
  새 값이 허용 오차 이상 벗어날 때까지 이전 값을 유지합니다. RC/PWM은 3 µs, 각도는
  0.5°, 방위는 1°, 보정값은 2, 고도는 0.3 m, 가속도는 0.03 g, 좌표는
  1e−5°). 직전에 출력한 줄과 다를 때만 출력합니다.

중첩된 타입: `LineBuffer : Print` (출력 전 비교를 위한 최대 200바이트의 줄),
`Shown` (히스테리시스가 있는 값).

줄의 형식:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  목표 0.0 m
MAG  방위 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (10초 중 최악) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=`는 `LOST(프레임 없음)`과 `LOST(송신기 페일세이프)`를 구분합니다. `IMU=`는
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK` 중 하나입니다.

---

## `DebugConsole`

**파일:** `telemetry/DebugConsole.h` · **의존:** `FlightController`, `FlightOutputs`, `Autopilot`, `DebugLogger`, `LogSettings`, `IBoard*` (버스 스캔)

시리얼 모니터의 텍스트 메뉴입니다. 화면 상태 기계 `Screen::{None, Main, Log}`를 가집니다.

| 메서드 | 설명 |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | 보드가 있으면 — 명령 `b`와 메뉴의 7번 항목 |
| `static const char* guessI2cDevice(uint8_t address)` | 주소로 칩을 추정합니다: 0x6A LSM6DSV, 0x68 MPU/ICM, 0x76 BME280/BMP388/SPL06, 0x46/0x47 BMP581, 0x7C QMC6309, 0x2C QMC5883P, 0x0D QMC5883L, 0x3C OLED |
| `void printHint() const` | 한 줄짜리 도움말 |
| `void update()` | `Serial`의 모든 바이트를 처리합니다. 로그 설정이 바뀌었고 메뉴가 닫혀 있으며 기체가 **armed가 아니면** NVS에 저장합니다 |

단축키 (메뉴 밖): `h`/`?` — 주 메뉴, `l` — 로그 메뉴, 스페이스 —
로그 일시 정지, `s` — 센서 상태, `i` — 자이로 보정, `o` —
IMU 장착 방향 보정, `m` — 나침반 보정, `p` — 출력 자가 점검,
`b` — I2C 버스 스캔 (0x08..0x7F — QMC6309가 0x7C에 있으므로 0x7F까지)과
칩 이름 표시, 그 밖의 키 — 도움말. `\r`/`\n`은 무시합니다.

로그 메뉴: `1`..`9` — 채널 0..8의 모드를 순환, `n` — NAV, `s` — SYS, `p` — 주기, `a` —
모두 “변경 시”, `x` — 모두 끄기, `d` — 기본값, `0`/`q` — 뒤로, `l`/`h` —
닫기.

작업을 막는 동작 (`i`, `o`, `m`, `p`)은 **ARM 중에는 금지**됩니다. 메뉴가
열려 있는 동안 로그는 일시 중단됩니다 (`DebugLogger::suspend`). 메뉴 항목의 폭은
바이트가 아니라 UTF-8 문자 수로 셉니다 (키릴 문자는 2바이트).

---

## `WebDashboardPage`

**파일:** `telemetry/WebDashboardPage.h` · **종류:** namespace

`static const char HTML[] PROGMEM` — 페이지 전체(HTML + CSS + JS)를 하나의
리터럴로 담았습니다. 동적인 부분은 모두 브라우저가 `/api/status`의 JSON으로 만듭니다 (
200 ms마다 폴링). 채널, 출력, 센서의 행은 JSON의 키로 생성되므로 새
출력은 페이지를 고치지 않아도 나타납니다. 사용자가 편집하기 시작한 PID 필드는
폴링이 더 이상 덮어쓰지 않습니다.

---

## `WebDebugServer`

**파일:** `telemetry/WebDebugServer.h` · **의존:** `WebServer`, `WiFi`, `FlightController`, `Autopilot*`, `WebDashboardPage`, `Config`

| 메서드 | 설명 |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | Wi-Fi AP (`persistent(false)` — 플래시에 쓰지 않음), 라우트, 코어 0의 `web` 태스크. 액세스 포인트가 올라오지 않으면 `false` |
| `void applyPendingCommands()` | 비행 루프에서 호출합니다. 스핀락 아래에서 명령을 꺼내 오토파일럿에 적용합니다 |

라우트:

| 라우트 | 응답 |
|---|---|
| `GET /` | 대시보드 페이지 |
| `GET /api/status` | 상태 JSON (`buildStatusJson()`), 형식은 [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus)에 있습니다 |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}`, 본문 없음 → 400 `no data`, 오토파일럿 없음 → 503, 잘못된 모드 → 400 `invalid mode` |
| `POST /api/setpid` | `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch` 중 아무거나. 생략한 값은 현재 값 그대로입니다 |
| 그 밖 | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }` — `portMUX` 아래의
메일박스입니다. `extractJsonNumber(body, key, fallback)` — ArduinoJson 없이
평평한 JSON을 최소한으로 해석합니다: `"key"`, 공백, `:`, 공백, JSON의
모든 표기법의 숫자 (부호, 소수, 지수 `1e-7`). 키나 숫자가 없으면
`fallback`.

JSON의 `attached`/`available` 필드는 **항상** 있으며, 센서 데이터는
`available: true`일 때만 있습니다.

---

## `OledDisplay`

**파일:** `telemetry/OledDisplay.h` · **의존:** U8g2, `II2CBus`, `FlightController`, `Autopilot*`, `LoopStats`

두 번째 I2C 버스에 연결한 SSD1306 128×64 (I2C 0x3C)입니다. 전용 태스크 `oled`
(`Rtos::startTask`: ESP32에서는 코어 0, STM32에서는 낮은 우선순위)가 200 ms마다 동작합니다.

| 메서드 | 설명 |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr`이거나 화면이 0x3C에서 응답하지 않으면 `false`. 그렇지 않으면 U8g2를 설정하고 태스크를 시작합니다 |

U8g2는 `II2CBus` 위에 얹은 `byteCallback`으로 바이트를 보냅니다 (화면은
`Wire1`에 대해 알지 못합니다). C 콜백은 컨텍스트를 받지 못하므로 버스는 정적
변수 `busSlot()`에 보관합니다. 기체에 달린 화면은 하나뿐입니다.

화면:

```
RX ok ARM STAB FL       링크 (끊기면 반전 줄) / ARM / 모드 / 플랩
R  +1.2 P  -0.4         롤 / 피치, °           (IMU --)
Alt +0.3 Vz +0.1 A14    고도 / 수직 속도 / 대기속도 (피토관이 있을 때) (BARO --)
H123 T1000 Y1500        방위 / 스로틀 / 러더 (H---)
L1500 R1500 E1500       에일러론 / 엘리베이터
Loop 500Hz max1100us    주파수와 1초 중 최악의 틱
```

모드의 짧은 이름은 `AutopilotNames::modeShort()`입니다 (`MAN`, `STAB`,
`TKOFF`, `ALT`, `ACRO`, `CRZ`, `LOIT`, `RTH`, `LNCH`, `LAND`, `SOAR`, `RESQ`).
공중에서 링크가 끊기면 `GLIDE` 또는 `FSRTH`가 됩니다.

---

## `BlackBox`

**파일:** `telemetry/BlackBox.h` · **의존:** `FlightController`, `Autopilot`, `LoopStats`, `BlackBoxStorage`, `PilotSwitches*`

비행 기록을 플래시 (ESP32-S3) 또는 SD 카드 (STM32H743)에 남깁니다. 무엇을 언제 어떻게 내려받는지는 [BLACKBOX.md](../BLACKBOX.md)를 보십시오.

| 메서드 | 설명 |
|---|---|
| `bool begin(bool startTask = true)` | 매체를 읽고 (`BlackBoxStorage::begin()`), 큐를 할당하고 (ESP32는 PSRAM, STM32는 `malloc`), 지워진 영역을 대조하고 (최대 0.3초), `bbox` 태스크를 시작합니다 (`Rtos::startTask`). 기록할 공간(파티션, 카드, 파일)이 없으면 `false`이며 블랙박스는 꺼집니다 |
| `void update(uint32_t workUs)` | 매 틱 뒤에 `loop()`에서 호출합니다: 이벤트, 시작/정지, 큐에 스냅샷 넣기, 쓰기 태스크 깨우기 |
| `void writerStep()` | 쓰기 태스크의 한 단계: 플래시에 한두 페이지, 또는 지상에서 한 번의 지우기 |
| `requestManualStart()` / `requestManualStop()` | 수동 기록 (콘솔의 `k` → `r`) |
| `State getState()` / `bool isRecording()` | `Off`, `Idle`, `Recording`, `Stopping` (END를 기록하기 전에 큐를 끝까지 씁니다) |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | 콘솔용 |
| `void handleHostCommand(const char*)` | `bb list`, `bb get <n> [보율]` — `tools/blackbox.py`용 (USB CDC에서는 속도가 아무 영향도 주지 않습니다) |

플랫폼마다 다른 부분: 재부팅 원인은 `readResetCause()`, 배터리 전압과 전류는
ADC (S3는 `analogReadMilliVolts`, STM32는 12비트 `analogRead`), 매체 오류
(`BlackBoxStorage::writeErrors`/`eraseErrors`)는 초당 한 번 “매체: 쓰기 오류 …”라는
이벤트로 로그에 들어가며 비행을 방해하지 않습니다.

## `BlackBoxStorage`

**파일:** `telemetry/BlackBoxStorage.h` · **의존:** `IFlashRegion`

4 KB 섹터의 링입니다. 헤드와 비행 목록은 `begin()` 때 섹터 헤더에서 얻습니다 (첫 번째 패스는 모든 섹터의 헤더를 읽어 진짜인 것을 기억하고, 두 번째 패스는 그것들만 다룹니다. 빈 영역은 한 번만 읽습니다). `openFlight()`/`append()`/`flush()`/`closeFlight()` — 페이지 단위 쓰기 (각 레코드에 CRC-8). `eraseStep(target, protect, allowErase)` — 헤드 앞에서 하는 대조/지우기 한 단계: 쓰레기는 항상, 비행은 통째로, 그리고 빈 공간이 `target`보다 적을 때만. `protect`는 절대 건드리지 않습니다.

## `BlackBoxRing`, `BlackBoxFormat`

`BlackBoxRing`은 태스크/코어 사이의 레코드 바이트 큐이며 `Rtos::CriticalSection` 아래에서 동작합니다. 넘치면 가장 오래된 것을 버립니다. `BlackBoxFormat` — 섹터 헤더, 레코드의 종류와 구조체, 스키마 문자열 (크기는 `static_assert`로 대조), CRC-8과 CRC-32.

---

## `Mavlink` (코덱)

**파일:** `telemetry/MavlinkCodec.h` · **종류:** namespace · **의존:** 없음 (이식 가능)

생성된 라이브러리 없이 구현한 MAVLink 2입니다. MAVLink의 순서로 필드를 채우고
(pymavlink와 대조 완료), CRC-16/MCRF4XX + `CRC_EXTRA`를 쓰며, 끝의 0을 잘라 냅니다.

| 개체 | 설명 |
|---|---|
| `Msg::*` | 식별자: HEARTBEAT, SYS_STATUS, SET_MODE, PARAM_*, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW, MISSION_REQUEST_LIST/COUNT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS, REQUEST_DATA_STREAM, VFR_HUD, COMMAND_LONG/ACK, HOME_POSITION, STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | 메시지의 `CRC_EXTRA`. −1은 알 수 없음 |
| `crcAccumulate`, `crcCalculate` | X.25 (mavlink의 `crc_accumulate()`와 같음) |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` — 필드를 순서대로 |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` — 연속된 `seq`가 붙은 v2 프레임 |
| `Message` | 수신한 메시지: `msgid`, `sysid`, `compid`, 페이로드 (0으로 채움), 오프셋으로 필드 읽기 |
| `Parser` | `bool feed(byte)` → `message()`. v1과 v2를 처리하며 v2의 서명은 건너뜁니다. `goodCount()`, `badCrcCount()`. `CRC_EXTRA`를 알 수 없는 메시지는 조용히 건너뜁니다 |

## `MavlinkModes`

**파일:** `telemetry/MavlinkTelemetry.h` · **종류:** namespace

| 함수 | 설명 |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | ArduPlane의 모드 번호: MANUAL 0, STABILIZE→FBWA 5, ALT_HOLD→FBWB 6, ACRO 4, CRUISE 7, LOITER 12, RTH→RTL 11, AUTO_TAKEOFF/LAUNCH→TAKEOFF 13, AUTO_LAND→AUTO 10, SOARING→THERMAL 24, RESCUE→STABILIZE 2. 페일세이프는 RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | 역변환으로, 지상에서 온 명령용입니다. AUTO, CIRCLE, GUIDED는 `false` |
| `isAutonomous(mode)` | HEARTBEAT의 `AUTO_ENABLED` 플래그 |

## `MavlinkTelemetry`

**파일:** `telemetry/MavlinkTelemetry.h` · **의존:** `IUartPort`, `FlightController`, `Autopilot*`, `LoopStats*`

QGroundControl / Mission Planner를 위한 무선 모뎀 텔레메트리입니다 (기체는
`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_ARDUPILOTMEGA`). Wi-Fi가 없는 STM32
(UART4)에서 사용합니다.

| 메서드 | 설명 |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | 포트를 열고 “OpenPlane online” 메시지를 보냅니다 |
| `void update()` | 비행 루프에서 호출합니다: 들어온 데이터 해석 (틱당 ≤ 128바이트), 이벤트 메시지, 틱당 최대 2프레임 |
| `void statusText(severity, text)` | GCS 피드로 (4줄 큐, 최대 50자) |
| `isGcsConnected()` | 최근 3초 안에 GCS의 HEARTBEAT가 있음 |
| `getSentFrames()`, `getDeferredFrames()`, `getParser()` | 진단 |
| `static const char* paramName(uint8_t)` | `RLL_KP`, `RLL_KI`, `RLL_KD`, `PTCH_KP`, `PTCH_KI`, `PTCH_KD` |

스트림 (Hz): ATTITUDE 10, GLOBAL_POSITION_INT와 VFR_HUD 5, GPS_RAW_INT,
RC_CHANNELS, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT 2, HEARTBEAT와 SYS_STATUS 1,
HOME_POSITION 0.2. 프레임은 `availableForWrite()`에 들어갈 공간이 있을 때만 보냅니다.
그렇지 않으면 다음 틱까지 기다립니다 (루프는 절대 막히지 않습니다).

수신하는 것: GCS의 HEARTBEAT, PARAM_REQUEST_LIST / READ / SET (PID는 곧바로
오토파일럿에 적용되며 값은 0..100이고 저장되지 않습니다), SET_MODE와 COMMAND_LONG의
`DO_SET_MODE` (176) — 스위치를 다음에 딸깍 넘길 때까지 유지되는 모드, `COMPONENT_ARM_DISARM`
(400) — **DENIED**, `REQUEST_MESSAGE` (512) — 스트림의 순서를 건너뛴 전송,
MISSION_REQUEST_LIST — 같은 `mission_type`의 MISSION_COUNT 0.
외부 디코더로 스트림을 점검하려면 `tools/check_mavlink.py` (pymavlink)를 사용합니다.
