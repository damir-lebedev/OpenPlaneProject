# RC — 송신기 명령 수신

> 🌐 이 문서는 [러시아어 원문](../../../reference/rc.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다.

[← 참조](README.md)

RC 계층은 UART 바이트를 채널 값과 “링크 없음” 표시로 바꿉니다. 이 계층은 기체, ARM,
failsafe 동작, 서보에 대해 아무것도 모릅니다. 프로토콜을 바꾸더라도(S-Bus, PPM)
영향을 받는 것은 이 계층뿐입니다.

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
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | 선형 변환: 1000 → `−max`, 1500 → 0, 2000 → `+max`(입력을 먼저 제한함). `reverse`는 부호를 뒤집습니다. 결과는 ±`max`로 제한 |

예: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `IBusReceiver`

**파일:** `rc/IBusReceiver.h` · **의존:** `IUartPort`, `RcChannelState`, `Config`, `Channels`

FlySky iBUS 프로토콜을 한 바이트씩 해석하는 파서입니다.

**프레임 형식**(32바이트): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(first 30 bytes)`. 앞의 `IBUS_CHANNELS` = 10개 채널을 사용하며,
채널 값은 **하위 12비트**입니다(상위 비트로 FS-iA6B는 서비스 데이터를 보냅니다.
예를 들어 failsafe에서는 `0x2384` → 900 µs).

| 메서드 | 설명 |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | 생성자에서는 포트를 열지 않음 |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`, 타임아웃은 “지금”부터 계산 |
| `void update()` | UART에 쌓인 것을 모두 읽어냄. 주기마다 호출 |
| `const RcChannelState& getState() const` | 마지막으로 받은 채널 |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | 아직 프레임이 하나도 오지 않았거나 **또는** 마지막 프레임이 `RX_TIMEOUT_US`보다 오래됨 |
| `bool isFailsafeReported() const` | 마지막 프레임에서 스로틀이 `RX_FAILSAFE_THROTTLE_US` 미만 |
| `uint32_t getLastFrameTime() const` | 마지막으로 올바른 프레임의 `micros()` |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | 올바른 프레임과 CRC 오류의 카운터 |

해석 상태 기계(`processByte`): `0x20`을 기다립니다. 다음 바이트는 `0x40`이어야 하며,
아니면 탐색을 처음부터 다시 합니다. 그다음 32바이트를 모아 `processFrame()`을
호출합니다. CRC가 틀린 프레임은 통째로 버립니다(채널은 변하지 않고
`badFrames++`).

불변 조건:

- 첫 번째 올바른 프레임이 올 때까지 `isSignalLost() == true`입니다. 기본값(모두 1500)을
  송신기의 명령으로 착각하지 않습니다.
- failsafe 표시는 **올바른 프레임마다** 다시 계산됩니다. 스로틀이 정상인 첫
  프레임만으로 링크가 복구됩니다.
