# RC — 송신기 명령 수신

> 🌐 이 문서는 [러시아어 원문](../../../reference/rc.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다. 이 번역은 AI가 작성했으며 원어민의 검수를 거치지 않았습니다. 오류를 발견하면 [Damir Lebedev](https://github.com/damir-lebedev)에게 알려 주시거나 [이슈](https://github.com/damir-lebedev/OpenPlaneProject/issues)로 남겨 주세요.

[← 참조](README.md)

RC 계층은 UART 바이트를 채널 값과 “링크 없음” 표시로 바꿉니다. 이 계층은 기체, ARM,
failsafe 동작, 서보에 대해 아무것도 모릅니다. 수신기 프로토콜(iBUS, SBUS, CRSF)은
`Config::RX_PROTOCOL`이 정하며, 새 프로토콜은 기존 세 파서 옆에 프레임 파서를 하나 더
두는 것뿐입니다.

---

## `RcChannelState`

**파일:** `rc/RcChannelState.h` · **의존:** `Config`, `Channels`

제어 로직 없이 수신기 10개 채널(µs)의 스냅샷만 담습니다.

| 메서드 | 설명 |
|---|---|
| `RcChannelState()` | `reset()`을 호출 |
| `void reset()` | 안전한 값: 모든 채널은 `PWM_CENTER`, 스로틀은 `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | 채널 값. 범위를 벗어난 인덱스는 `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | 채널을 기록. 범위를 벗어난 인덱스는 무시 |
| `const uint16_t* data() const` | 배열 전체(디버깅용) |

---

## `RcInput`

**파일:** `rc/RcInput.h` · **종류:** 정적 함수 모음 · **의존:** `Config`

RC 신호의 공통 변환입니다.

| 메서드 | 설명 |
|---|---|
| `static uint16_t clamp(uint16_t value)` | `PWM_MIN..PWM_MAX`로 제한 |
| `static uint16_t reversed(uint16_t value)` | 채널을 반대로: `3000 − clamp(value)`(1000 ↔ 2000) — `Config::CHANNEL_REVERSED`용 |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | 선형 변환: 1000 → `−max`, 1500 → 0, 2000 → `+max`(입력을 먼저 제한함). `reverse`는 부호를 뒤집습니다. 결과는 ±`max`로 제한 |

예: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `RcReceiver`

**파일:** `rc/RcReceiver.h` · **의존:** `IUartPort`, `IBusParser`, `SbusParser`, `CrsfParser`, `RcChannelState`, `RcInput`, `Config`, `Channels`

조종기 수신기: 프로토콜 형식으로 UART를 열고, 바이트를 파서에 넘기며, 모든 프로토콜에
공통인 일을 합니다 — 앞쪽 `RC_CHANNELS` = 10개 채널, 채널 반전, 프레임 타임아웃,
failsafe, 카운터.

| 프로토콜 (`Config::RX_PROTOCOL`) | 수신기 | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B, FS-iA10B | 115200, 8N1 |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper 및 대부분의 다른 수신기 | 100000, 8E2, 반전 신호 — UART가 직접 반전 |
| `CRSF` | ExpressLRS, TBS Crossfire, Tracer | 420000, 8N1 |

| 메서드 | 설명 |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | 생성자에서는 포트를 열지 않음; 테스트에서는 어떤 프로토콜이든 줄 수 있음 |
| `void begin()` | 프로토콜 형식으로 UART를 열고(`IUartPort::begin(baud, format)`) 타임아웃을 “지금”부터 셈 |
| `void update()` | UART에 쌓인 것을 모두 읽음; 매 주기마다 호출 |
| `const RcChannelState& getState() const` | 마지막으로 받은 채널 |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | 아직 프레임이 하나도 오지 않았거나 **또는** 마지막 프레임이 `RX_TIMEOUT_US`보다 오래됨 |
| `bool isFailsafeReported() const` | 수신기가 링크 끊김을 알림: SBUS 프레임의 failsafe 플래그, CRSF 링크 품질 0 %, 또는 스로틀 < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | 마지막 올바른 프레임의 `micros()` |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | 받아들인 프레임과 버린 프레임(CRC, 프레임 끝)의 카운터 |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | 프로토콜; 이름은 `"iBUS"`, `"SBUS"`, `"CRSF"` |
| `int16_t getLinkQuality() const` | 무선 링크 품질, % — CRSF만, 그 밖에는 −1 |
| `bool isPortReady() const` | UART가 필요한 형식으로 열림; `false` — 포트가 그렇게 못 함(반전 없는 SBUS), 링크가 없을 것 |

자유 함수 `rxProtocolName(Config::RxProtocol)`은 프로토콜 이름을 돌려줍니다. 펌웨어 여권
(`@id`, `rx` 필드)이 출력하는 것도 같은 이름입니다.

불변 조건:

- 첫 번째 올바른 프레임 전까지 `isSignalLost() == true` — 기본값(모두 1500)을 조종기
  명령으로 받아들이지 않습니다.
- failsafe 표시는 채널 프레임 **마다** 다시 계산됩니다 — 정상 스로틀인 첫 프레임으로
  링크가 돌아옵니다(CRSF는 링크 품질이 0보다 큰 통계 프레임으로도).
- `Config::CHANNEL_REVERSED`인 채널은 뒤집힌 값으로 저장됩니다(`RcInput::reversed`).
  failsafe는 그 전에 **원시** 스로틀로 판단하며, CH3 반전은 `static_assert`로 금지됩니다.

---

## `IBusParser`

**파일:** `rc/IBusParser.h` · **의존:** `Config`

FlySky iBUS를 바이트 단위로 해석: `bool feed(uint8_t)`는 CRC가 맞는 프레임이 다 모이면
`true`를 돌려주고, 채널은 `values()`에 있습니다. **프레임**(32바이트): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(처음 30바이트)`; 채널 값은 **하위 12비트**입니다(상위 비트로 FS-iA6B는
서비스 데이터를 보냄, 예: failsafe에서 `0x2384` → 900 µs). `0x20`을 기다리고, 다음
바이트는 `0x40`이어야 하며 아니면 탐색을 다시 합니다(두 번째 `0x20` 자체가 프레임 시작일
수 있음). iBUS에는 failsafe 플래그가 없고, FS-iA6B는 정상보다 낮은 스로틀로 링크 끊김을
알립니다.

---

## `SbusParser`

**파일:** `rc/SbusParser.h` · **의존:** `Packed11`

**프레임**(25바이트): `0x0F | 11비트 채널 16개(22바이트) | 플래그 | 끝`; 플래그: 비트 2 —
무선으로 프레임 하나 손실(아직 링크 끊김은 아님), 비트 3 — failsafe; 끝 — SBUS는 `0x00`,
SBUS2는 `0x04`/`0x14`/`0x24`/`0x34`. 바이트 `0x0F`는 데이터 안에도 나오므로 시작과 끝이
모두 제자리에 있을 때만 프레임을 받아들이고, 아니면 받은 데이터 안의 다음 `0x0F`부터
해석을 이어 갑니다 — 스트림은 한두 프레임 안에 동기화됩니다.

---

## `CrsfParser`

**파일:** `rc/CrsfParser.h` · **의존:** `Packed11`

**프레임:** `0xC8 | 길이 | 종류 | 데이터 | CRC8`; 길이 = 종류 + 데이터 + CRC, 프레임 전체는
64바이트 이하; CRC8은 다항식 `0xD5`(DVB-S2)로 종류와 데이터에 대해 계산합니다(`static uint8_t crc8(...)`).
필요한 종류는 두 가지: `0x16` — 채널(SBUS처럼 11비트 16개)과 `0x14` — 링크 통계(세 번째
바이트가 링크 품질, %). 링크가 끊기면 ExpressLRS는 기본적으로 채널 전송을 그냥 멈춥니다 —
이것이 프레임 타임아웃입니다. “마지막 위치” 모드에서는 채널이 계속 오지만 링크 품질이
0입니다 — 이것이 `failsafe()`입니다.

---

## `Packed11`

**파일:** `rc/Packed11.h` · **종류:** 네임스페이스

22바이트에 11비트 채널 16개, 하위 비트 먼저 — SBUS와 CRSF 모두 이렇게 묶습니다.
`toMicros(raw)`: 172…1811 → 988…2012 µs, 992 → 1500(`5/8 · 값 + 880`, Betaflight와 INAV처럼);
`unpack(data, out)`은 22바이트를 µs 단위 값 16개로 바꿉니다.
