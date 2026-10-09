# HAL — 하드웨어 추상화

> 🌐 이 문서는 [러시아어 원문](../../../reference/hal.md)을 번역한 것입니다. 번역과 원문이 다를 경우 원문이 우선합니다. 펌웨어는 콘솔 메시지를 러시아어로 출력하므로 그대로 인용했습니다. 이 번역은 AI가 작성했으며 원어민의 검수를 거치지 않았습니다. 오류를 발견하면 [Damir Lebedev](https://github.com/damir-lebedev)에게 알려 주시거나 [이슈](https://github.com/damir-lebedev/OpenPlaneProject/issues)로 남겨 주세요.

[← 참조](README.md)

HAL은 특정 MCU를 알아도 되는 유일한 계층입니다. 인터페이스는
`include/hal/`에 있으며, 구현은 다음과 같습니다.

- `include/hal/esp32/` — ESP32 (Arduino core 2.0.x)입니다.
- `include/hal/stm32/` — STM32H743 (STM32duino 3.x), **주력 구현**입니다. 전체 펌웨어가 빌드되고 (`pio run -e stm32h743-devebox`) PC에서 실행됩니다 (`pio test -e native-stm32`). DevEBox 보드에서는 SD 카드, 블랙박스, iBUS, 서보를 확인했고, 센서는 아직 확인하지 않았습니다.
- `hal/Rtos.h` — FreeRTOS 태스크로, 두 플랫폼에서 똑같이 동작합니다.

이 위의 모든 코드는 인터페이스만 사용하므로, 다른 MCU로 옮기는 일은 센서를 다시 쓰는 것이 아니라
`IBoard`의 새 구현을 작성하는 것입니다.

---

## namespace `ServoChannel`

**파일:** `hal/IBoard.h`

`IBoard::servo(channel)`에 쓰는 출력 인덱스입니다. 이름 붙은 메서드가 아니라 평평한 목록이므로,
출력을 추가해도 `IBoard` 인터페이스는 바뀌지 않습니다. 순서는 `FlightOutputs::outputInfo()` 표의
행과 같습니다.

| 상수 | 값 |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 — 화물 투하 (`Feature::PAYLOAD_DROP`) |
| `AUX2` | 6 — 카메라 (`Knob::CAMERA_TILT`, `Feature::CAMERA_STAB`) |
| `COUNT` | 7 |

---

## `IBoard`

**파일:** `hal/IBoard.h` · **종류:** 인터페이스 · **구현:** `Esp32Board`, `Stm32Board`

하드웨어로 들어가는 유일한 진입점입니다. 이 위의 어떤 코드도 `<Wire.h>`,
`<SPI.h>`, `HardwareSerial`을 포함하지 않으며 LEDC를 직접 호출하지도 않습니다.

| 메서드 | 설명 |
|---|---|
| `virtual void begin()` | I2C/SPI 버스의 일회성 초기화. UART는 각 소유자(`IBusReceiver`, GPS)가 자기 속도로 열고, PWM은 `FlightOutputs::begin()`이 엽니다 |
| `virtual II2CBus& i2c()` | 센서 버스 |
| `virtual ISpiBus& spi()` | SPI 버스 |
| `virtual II2CBus* displayI2c()` | 화면 전용 두 번째 I2C 버스. 없으면 `nullptr` |
| `virtual IUartPort& rcUart()` | iBUS 수신기의 UART |
| `virtual IUartPort& gpsUart()` | GPS의 UART |
| `virtual IUartPort* telemetryUart()` | MAVLink 무선 모뎀의 UART. 기본값은 `nullptr` (ESP32에는 남는 UART가 없습니다) |
| `virtual IServoOutput& servo(uint8_t channel)` | `ServoChannel::*` 인덱스로 고르는 PWM 출력 |
| `virtual void setBuzzer(bool on)` | 부저 `PIN_BUZZER`. 기본적으로는 아무것도 하지 않습니다 |

---

## `II2CBus`

**파일:** `hal/II2CBus.h` · **종류:** 비가상 도우미 함수를 가진 인터페이스 ·
**구현:** `Esp32I2CBus`, `Stm32I2CBus`

`Wire`와 같은 모양의 I2C 버스 추상화입니다. 핀과 주파수는 구현이 생성자에서 고정하므로
`begin()`/`setClock()`은 핀을 받지 않습니다. 버스에 장치가 여러 개 있어도 버스는 정확히 한 번만
초기화됩니다.

| 메서드 | 설명 |
|---|---|
| `begin()`, `setClock(hz)` | 초기화, 주파수 |
| `beginTransmission(addr)`, `write(byte)`, `write(data, len)`, `endTransmission(sendStop = true)` | 쓰기. 성공하면 `endTransmission`이 0을 반환합니다 (`Wire`와 같음) |
| `requestFrom(addr, n)`, `available()`, `read()` | 읽기 |
| `bool writeRegister(addr, reg, value)` | 도우미: 레지스터 하나를 씁니다. `false`는 NACK |
| `bool readRegisters(addr, reg, buf, count)` | 도우미: 반복 시작 + `count` 바이트 읽기. NACK이거나 **`count` 바이트보다 적게 도착하면** `false`이며, 이때 버퍼는 건드리지 않습니다 |
| `int readRegister(addr, reg)` | 레지스터 값 또는 `-1` |
| `bool probe(addr)` | 장치가 해당 주소에 ACK로 응답함 |

불변 조건: 실패했을 때 도우미는 버퍼에 쓰지 않습니다. 드라이버는 쓰레기 값
(빈 버퍼에서 `read()`가 반환하는 `0xFF`) 대신 이전 데이터를 유지합니다.

---

## `ISpiBus`

**파일:** `hal/ISpiBus.h` · **종류:** 인터페이스 · **구현:** `Esp32SpiBus`, `Stm32SpiBus`

**CS를 관리하지 않는** SPI 버스입니다. 하나의 버스에 장치가 여러 개 있고,
CS는 `SpiRegisterDevice`가 전환합니다.

| 메서드 | 설명 |
|---|---|
| `begin()` | SCK/MISO/MOSI를 설정합니다 (핀은 구현의 생성자에 있습니다) |
| `beginTransaction(clockHz, spiMode)` | `spiMode`는 0~3 (CPOL/CPHA) |
| `uint8_t transfer(data)` | 한 바이트의 전이중 교환 |
| `endTransaction()` | 트랜잭션 종료 |

---

## `IUartPort`

**파일:** `hal/IUartPort.h` · **종류:** 인터페이스 · **구현:** `Esp32UartPort`, `Stm32UartPort`

`HardwareSerial`과 같은 모양의 UART이지만 `begin()`은 속도만 받습니다.
핀과 형식(8N1)은 구현이 고정합니다.

| 메서드 | 설명 |
|---|---|
| `begin(baud)` | 포트를 엽니다 |
| `int available()`, `int read()` | 수신 |
| `size_t write(byte)`, `size_t write(buffer, size)` | 송신 |
| `virtual int availableForWrite()` | 송신 버퍼의 남은 공간. `-1`은 알 수 없음 (기본값). 텔레메트리는 이 값을 보고 기다리는 대신 프레임을 뒤로 미룹니다 |

---

## `IServoOutput`

**파일:** `hal/IServoOutput.h` · **종류:** 인터페이스 · **구현:** `Esp32ServoOutput`, `Stm32ServoOutput`

PWM 출력 하나입니다. 핀은 구현이 고정합니다.

| 메서드 | 설명 |
|---|---|
| `bool attach(minUs, maxUs)` | 채널/타이머를 할당하고 핀을 설정합니다. 펄스를 제한하는 범위입니다. `true`는 MCU가 자원을 할당했다는 뜻일 뿐, 서보가 연결되어 있다는 뜻은 **아닙니다** |
| `writeMicroseconds(us)` | 펄스 폭, µs (`attach`의 범위로 제한됩니다) |
| `bool isAttached() const` | `attach()`의 결과 |
| `virtual int32_t measurePulseUs()` | 진단: 핀에서 실제로 측정한 펄스 폭 또는 `-1`. 기본 구현은 `-1`을 반환합니다 |

---

## `IFlashRegion`

**파일:** `hal/IFlashRegion.h` · **종류:** 인터페이스 · **구현:** `Esp32FlashPartition`, `SdFileRegion`

로그(블랙박스)용 NOR 플래시 영역입니다. 지우기는 4 KB 섹터 단위로만 할 수 있고
(지워진 곳은 `0xFF`로 읽힙니다), 쓰기는 비트를 내리기만 합니다. 지워진 바이트에는 쓸 수 있으며,
한 페이지에 나누어 쓸 수도 있습니다. ESP32에서는 쓰기와 지우기 모두 두 코어를 멈추므로,
언제 허용되는지는 호출하는 쪽이 판단합니다.

| 메서드 | 설명 |
|---|---|
| `uint32_t size() const` | 영역의 크기, 바이트. 0은 영역이 없음을 뜻합니다 |
| `bool read(offset, data, length)` | 읽기 |
| `bool write(offset, data, length)` | 쓰기 (지워진 바이트에) |
| `bool erase(offset, length)` | 지우기. 주소와 길이는 4096의 배수입니다 |

`Esp32FlashPartition(const char* name)`은 파티션 테이블의 이름으로 지정하는 데이터 파티션입니다
(`esp_partition_*`). `begin()`이 파티션을 찾으며 (코어가 시작된 뒤),
파티션이 없으면 `false`를 반환하고 `size() == 0`이 됩니다.

---

## `IBlockDevice`

**파일:** `hal/IBlockDevice.h` · **종류:** 인터페이스 · **구현:** `Stm32SdCard` (테스트에서는 `fake::SdCardModel`)

512바이트 블록의 배열로 본 SD 카드입니다. 지우기는 없으며, 블록은 덮어쓸 수 있습니다.

| 메서드 | 설명 |
|---|---|
| `uint32_t blockCount() const` | 블록 단위 크기. 0은 카드가 없음을 뜻합니다 |
| `bool read(block, data, count)` / `write(...)` | 연속된 `count`개의 블록. `data`는 임의의 주소 |

## `SdFileRegion`

**파일:** `hal/SdFileRegion.h` · **상속:** `IFlashRegion` · **의존:** `IBlockDevice`, `Fat32::locate`

SD 카드 위의 블랙박스 영역입니다. FAT32 루트에 있는 파일 (기본값은
`BLACKBOX.BIN`)로, PC에서 미리 하나의 연속된 덩어리로 만들어 (`tools/blackbox.py
sd-prepare`) `0xFF`로 채워 둡니다. 파일은 **찾기만** 하고 (FAT 테이블과
디렉터리는 건드리지 않습니다), 그 안에 원시 블록을 씁니다.
`BlackBoxStorage` 입장에서는 ESP32의 플래시 파티션과 같은 `IFlashRegion`입니다.

| 메서드 | 설명 |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes`는 영역의 상한입니다. 전원을 켤 때 섹터를 대조하는 시간은 이 값에 비례해 늘어납니다 |
| `Fat32::Result begin()` | 파일을 찾습니다. `Ok`이면 `size() > 0`이고, 아니면 이유 (`Fat32::describe()`): 카드가 없음, FAT32가 아님, 파일이 없음, 조각나 있음, 비어 있음 |
| `size()` | 파일 크기 (`maxBytes` 이하)를 4 KB 섹터 단위로 내림한 값. 0은 영역이 없음을 뜻합니다 |
| `read` / `write` | 임의의 오프셋과 길이. 불완전한 블록은 읽어서 채운 뒤 통째로 씁니다. 방금 쓴 블록은 기억해 둡니다 (라이트스루 캐시). 따라서 256바이트 페이지를 연속해서 써도 카드를 다시 읽지 않습니다. 전원이 끊겨도 이미 `write()`에서 돌아온 데이터는 사라지지 않습니다 |
| `erase(offset, length)` | 4096의 배수. `0xFF`를 씁니다 (카드 내부에 자체 지우기가 있어서 바깥에서는 필요하지 않습니다) |

## `IRegisterDevice`

**파일:** `hal/RegisterDevice.h` · **종류:** 인터페이스 ·
**구현:** `I2cRegisterDevice`, `SpiRegisterDevice`

“8비트 레지스터의 집합”입니다. 센서 드라이버는 한 번만 작성하고,
버스는 `SensorSelection.h`에서 객체를 만들 때 고릅니다.

| 메서드 | 설명 |
|---|---|
| `virtual void begin()` | 장치의 신호선을 준비합니다 (SPI에서는 CS). 기본적으로는 아무것도 하지 않습니다 |
| `virtual bool probe()` | 장치가 응답했음 (SPI에서는 항상 `true`. ACK가 없으므로 ID 레지스터를 확인합니다) |
| `virtual bool writeRegister(reg, value)` | 레지스터 쓰기 |
| `virtual bool writeRegisters(reg, data, count)` | 연속 쓰기 (주소 자동 증가) |
| `virtual bool readRegisters(reg, buffer, count)` | `count` 바이트 연속 읽기. `false`이면 버퍼는 건드리지 않습니다 |
| `int readRegister(reg)` | 값 또는 `-1` (비가상 도우미) |

---

## `I2cRegisterDevice`

**파일:** `hal/RegisterDevice.h` · **상속:** `IRegisterDevice`

`II2CBus` 위의 7비트 주소를 쓰는 장치입니다. 모든 동작은 `II2CBus`의 도우미에 위임합니다.

| 메서드 | 설명 |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress`는 칩의 두 번째 주소입니다 (SDO/SA0 핀): LSM6DSV 0x6A/0x6B, ICM-45686 0x68/0x69, SPL06 0x76/0x77, BMP581 0x46/0x47 |
| `begin()` | 기본 주소가 응답하지 않고 예비 주소가 응답하면, 이후에는 예비 주소로 동작합니다 |
| `probe()`, `writeRegister()`, `writeRegisters()`, `readRegisters()` | → `II2CBus(address, …)` 도우미 |
| `uint8_t getAddress() const` | 장치의 현재 주소 |

---

## `SpiRegisterDevice`

**파일:** `hal/RegisterDevice.h` · **상속:** `IRegisterDevice`

`ISpiBus` 위에서 전용 CS 핀을 쓰는 장치입니다. Bosch/InvenSense 프로토콜로,
읽기는 주소에 `0x80` 비트를 세우고, 쓰기는 비트 7을 내려서 합니다.

| 메서드 | 설명 |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData`는 칩이 주소 뒤, 데이터 앞에 내보내는 “쓰레기” 바이트의 수입니다 (BMP388은 1, ICM42688은 0). `mode`는 SPI 모드 0~3 |
| `begin()` | `pinMode(cs, OUTPUT)`, CS = HIGH |
| `probe()` | 항상 `true` |
| `writeRegister(reg, value)` | CS↓, `reg & 0x7F`, `value`, CS↑. 항상 `true` |
| `readRegisters(reg, buf, n)` | CS↓, `reg \| 0x80`, `dummyReadBytes` 건너뛰기, `n` 바이트, CS↑. 항상 `true` |

각 동작은 독립된 `beginTransaction(clockHz, spiMode)` …
`endTransaction()` 트랜잭션입니다.

---

## `Esp32Board`

**파일:** `hal/esp32/Esp32Board.h` · **상속:** `IBoard`

ESP32의 구체적인 주변장치 객체를 만들고 `Config.h`의 핀을 아는 유일한 곳입니다.

| 필드 | 타입 | 내용 |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `PIN_I2C_SDA/SCL`의 `Wire`, 400 kHz |
| `displayBus` | `Esp32I2CBus` | `PIN_I2C2_SDA/SCL`의 `Wire1` — `SOC_I2C_NUM > 1`일 때만 |
| `spiBus` | `Esp32SpiBus` | 전역 `SPI` |
| `rcSerial`, `rcPort` | `HardwareSerial(1)`, `Esp32UartPort` | `PIN_IBUS`의 iBUS, RX 전용 |
| `gpsSerial`, `gpsPort` | `HardwareSerial(UART_NUM_GPS)`, `Esp32UartPort` | `PIN_GPS_RX/TX`의 GPS |
| `servos[7]` | `Esp32ServoOutput` | `ServoChannel` 순서의 LEDC 채널 0~6 (AUX1/AUX2는 `PIN_AUX1/2`, 배선되어 있다면) |

| 메서드 | 설명 |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, 두 번째 버스가 있으면 이어서 `displayBus.begin()`. 부저 핀 |
| `setBuzzer(on)` | 핀이 배선되어 있으면 `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | `hasDisplayBus()`이면 `&displayBus`, 아니면 `nullptr` |
| `static constexpr bool hasDisplayBus()` | 두 번째 버스의 핀이 둘 다 0 이상입니다. `SOC_I2C_NUM > 1`일 때만 존재합니다 (`displayBus` 필드도 마찬가지). C3에는 I2C 컨트롤러가 하나뿐입니다 |
| 나머지 | 해당하는 필드를 반환합니다 |

---

## `Esp32I2CBus`

**파일:** `hal/esp32/Esp32I2CBus.h` · **상속:** `II2CBus`

`TwoWire` (`Wire` 또는 `Wire1`) 위의 얇은 래퍼입니다.

| 메서드 | 설명 |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | 매개변수를 저장합니다 |
| `begin()` | `wire.begin(sda, scl, hz)`와 `wire.setTimeOut(TIMEOUT_MS)` — `wire.begin()`을 부르는 유일한 곳입니다 |
| 나머지 | `TwoWire`에 그대로 위임합니다 |

`TIMEOUT_MS = 5`: 400 kHz에서 IMU의 14바이트를 읽는 데는 약 0.4 ms가 걸립니다. 간섭으로 멈춘 트랜잭션이
있으면 기본값 50 ms 동안 루프가 멈추게 됩니다.

---

## `Esp32SpiBus`

**파일:** `hal/esp32/Esp32SpiBus.h` · **상속:** `ISpiBus`

전역 `SPI` 위의 래퍼입니다. `begin()` → `SPI.begin(sck, miso, mosi, -1)` (CS는
장치가 잡고 있습니다). `beginTransaction()`은 `SPISettings(hz, MSBFIRST,
SPI_MODEn)`을 만듭니다. `spiModeOf()`는 0~3을 Arduino 상수로 바꾸며, 알 수 없는
값은 `SPI_MODE0`이 됩니다.

---

## `Esp32UartPort`

**파일:** `hal/esp32/Esp32UartPort.h` · **상속:** `IUartPort`

`HardwareSerial` 위의 래퍼입니다. `begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)`이며, `tx = -1`이면 수신 전용입니다. 나머지는 위임입니다.

---

## `Esp32ServoOutput`

**파일:** `hal/esp32/Esp32ServoOutput.h` · **상속:** `IServoOutput`

LEDC로 직접 PWM을 냅니다 (Arduino core 2.x의 `ledcSetup/ledcAttachPin/ledcWrite`).
ESP32Servo 라이브러리는 **사용하지 않습니다**. S3에서 3.2.1 버전이 MCPWM
블록을 혼동했습니다 (GPIO6/7이 GPIO4/5를 그대로 따라 했습니다).

| 상수 | 값 |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384 (한 단계당 ≈1.2 µs) |

| 메서드 | 설명 |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | 핀 `< 0`은 출력이 배선되지 않았음을 뜻합니다 |
| `attach(minUs, maxUs)` | 범위를 저장합니다. 핀 < 0이면 `false`, 아니면 `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | attached가 아니면 아무것도 하지 않고, 아니면 `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | 같은 GPIO의 입력 버퍼를 켜고 (`PIN_INPUT_ENABLE`, 출력은 건드리지 않습니다) `pulseIn(pin, HIGH, 30 ms)`를 측정합니다. 펄스가 없으면 `-1` |

채널 2n과 2n+1은 LEDC 타이머를 공유하지만, 모든 출력이 50 Hz이므로 충돌은 없습니다.

---

# STM32H743용 구현

다음 세대 보드는 STM32H743VIT6입니다 (Cortex-M7 480 MHz, 플래시 2 MB,
RAM 1 MB). 전체 펌웨어가 빌드되고 (env `stm32h743`은 PlatformIO 보드
`weact_mini_h743vitx`, `stm32h743-devebox`는 DevEBox H743이며 콘솔은
USB CDC), cppcheck와 PC 테스트를 통과합니다 (env `native-stm32`, STM32duino의
가짜 계층 사용). **센서 없이** DevEBox 보드에서 확인한 것은 부팅, SD 카드, 블랙박스
([보드 테스트](../TESTING.md#stm32-보드에서의-테스트))와 iBUS 수신, ARM, 서보와 모터로 나가는
PWM입니다. 비행기는 송신기로 수동 모드에서 조종되며, 시동 장면은 영상으로 남겼습니다.
센서는 아직 보드에 연결하지 않았습니다.
핀 배치는 [`Config.h`](config.md#stm32h743vit6board_stm32h743)의 `BOARD_STM32H743` 블록에 있습니다.

이 계층이 감추는 ESP32와의 공통된 차이는 다음과 같습니다.

- **주변장치는 코어가 고릅니다.** STM32duino는 변형(variant)의 `PeripheralPins` 테이블에 있는
  핀 번호로 컨트롤러 (I2C1/I2C2, SPI2, USART3, UART4, UART7, TIMx)를 스스로 찾아냅니다.
  그래서 `Config.h`에는 UART/채널 번호가 없습니다.
- **핀 번호**는 GPIO가 아니라 변형의 “Arduino 핀” (`PA0`, `PD14`...)입니다.
  아날로그 핀은 `0xC0 + N`이므로, STM32 블록의 핀은 `int16_t`입니다.
- **UART 핀**은 `begin()`이 아니라 `Uart(rx, tx)` 객체를 만들 때 지정합니다.

## `Stm32Board`

**파일:** `hal/stm32/Stm32Board.h` · **상속:** `IBoard`

`Esp32Board`와 같은 역할을 STM32duino 위에서 합니다.

| 필드 | 타입 | 내용 |
|---|---|---|
| `displayWire` | `TwoWire` | 두 번째 I2C 컨트롤러 (전역 `Wire`는 센서가 사용 중입니다). 이를 참조하는 `displayBus`보다 먼저 선언되어 있습니다 |
| `i2cBus` | `Stm32I2CBus` | `PIN_I2C_SDA/SCL` (I2C2: PB11/PB10)의 `Wire`, 400 kHz |
| `displayBus` | `Stm32I2CBus` | `PIN_I2C2_SDA/SCL` (I2C1: PB9/PB8)의 `displayWire` — 두 번째 버스는 항상 있습니다 |
| `spiBus` | `Stm32SpiBus` | `PIN_SENSOR_SPI_*` (SPI2)의 전역 `SPI` |
| `rcSerial`, `rcPort` | `Uart`, `Stm32UartPort` | iBUS: UART7, RX는 `PIN_IBUS` (PE7), TX는 `PIN_IBUS_TX` (PE8, iBUS-SENS용 예약) |
| `gpsSerial`, `gpsPort` | `Uart`, `Stm32UartPort` | GPS: USART3, `PIN_GPS_RX/TX` (PD9/PD8) |
| `telemetrySerial`, `telemetryPort` | `Uart`, `Stm32UartPort` | MAVLink 무선 모뎀: UART4, `PIN_TELEM_RX/TX` (PD0/PD1) |
| `servos[7]` | `Stm32ServoOutput` | `ServoChannel` 순서 (AUX1은 PD15/TIM4, AUX2는 PE9/TIM1) |

| 메서드 | 설명 |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, `displayBus.begin()`, 부저 핀 |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | 항상 `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | `Config.h`의 핀을 코어 API의 타입으로 바꿉니다 |
| 나머지 | 해당하는 필드를 반환합니다 |

## `Stm32I2CBus`

**파일:** `hal/stm32/Stm32I2CBus.h` · **상속:** `II2CBus`

| 메서드 | 설명 |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | 매개변수를 저장합니다 |
| `begin()` | `setSDA()`/`setSCL()` (`begin()` 이전에만 효과가 있습니다), `wire.begin()`, `wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)`. `size_t` 결과는 `uint8_t`로 변환됩니다 |
| 나머지 | `TwoWire`에 그대로 위임합니다 |

STM32duino에서 트랜잭션 타임아웃은 메서드가 아니라 매크로
`I2C_TIMEOUT_TICK` (ms, 기본값 100)입니다. env `stm32h743`에서는 플래그
`-D I2C_TIMEOUT_TICK=5`로 지정합니다. `Esp32I2CBus`의 `TIMEOUT_MS`와 같은 이유입니다.

## `Stm32SpiBus`

**파일:** `hal/stm32/Stm32SpiBus.h` · **상속:** `ISpiBus`

`SPIClass&` 위의 래퍼입니다. `begin()` → `setSCLK/setMISO/setMOSI` + `spi.begin()`입니다.
하드웨어 NSS는 쓰지 않으며, ESP32와 마찬가지로 CS는 `SpiRegisterDevice`가 전환합니다.
`beginTransaction()`은 `SPISettings(hz, MSBFIRST, SPIMode)`를 만듭니다. `spiModeOf()`는
0~3을 `SPI_MODEn`으로 바꾸며, 알 수 없는 값은 `SPI_MODE0`이 됩니다.

## `Stm32UartPort`

**파일:** `hal/stm32/Stm32UartPort.h` · **상속:** `IUartPort`

`HardwareSerial&` 위의 래퍼입니다 (STM32duino 3.x에서는 추상 기반 클래스
`arduino::HardwareSerial`이며, 구체적인 `Uart` 객체는 `Stm32Board`가 만듭니다).
`begin(baud)` → `serial.begin(baud, SERIAL_8N1)`입니다. `availableForWrite()`는
`HardwareSerial`의 것입니다. 버퍼 (env의 `SERIAL_RX/TX_BUFFER_SIZE`)는 수신이 256
바이트 (NAV-PVT 프레임이 100바이트라서 표준 64로는 모자랍니다), 송신이 1024입니다 (로그 줄과
MAVLink 프레임을 기다리지 않고 내보내기 위해서입니다).

## `Stm32ServoOutput`

**파일:** `hal/stm32/Stm32ServoOutput.h` · **상속:** `IServoOutput`

`HardwareTimer`를 통한 타이머의 하드웨어 PWM이며, 50 Hz입니다. 펄스는 타이머가
인터럽트와 CPU 없이 만들어 냅니다. 하나의 타이머 인터럽트에서 핀을 흔들어 지터가 생기는
STM32용 `Servo` 라이브러리와는 다릅니다.

| 상수 | 값 |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4 — 출력이 차지할 수 있는 서로 다른 타이머의 수 (현재는 TIM2와 TIM4가 사용 중) |

| 메서드 | 설명 |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | 핀 `< 0`은 출력이 배선되지 않았음을 뜻합니다 |
| `attach(minUs, maxUs)` | 타이머와 채널은 핀으로 `PinMap_TIM`에서 구합니다 (`pinmap_peripheral`, `STM_PIN_CHANNEL`). `analogWrite()`와 같습니다. 핀에 타이머가 없거나 풀이 소진되면 `false`입니다. 아니면 `setMode(PWM1)`, 비교값 0 (첫 쓰기 전까지 펄스 없음), `resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` → `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`. 비교 레지스터는 프리로드 방식이어서, 값은 다음 주기부터 적용됩니다 |
| `measurePulseUs()` | 핀을 다시 설정하지 않고 `pulseIn(pin, HIGH, 30 ms)`를 측정합니다. STM32에서는 IDR 레지스터가 대체 기능 모드에서도 레벨을 읽을 수 있습니다 |
| `static acquireTimer(TIM_TypeDef*)` | 공유 풀: **TIMx마다 `HardwareTimer` 하나**만 둡니다. 같은 타이머에 두 번째 객체를 만들면 코어의 핸들러 (`HardwareTimer_Handle[index]`)를 덮어쓰게 됩니다. 주기는 그 타이머의 첫 출력에서 정해지며, `setOverflow(MICROSEC_FORMAT)`가 분주비를 고릅니다. 타이머 클럭이 240 MHz일 때 한 단계는 ~0.3 µs입니다 |

## `Stm32FlashStorage`

**파일:** `hal/stm32/Stm32FlashStorage.h` · **상속:** `IFlashStorage` ([storage.md](storage.md))

STM32에서 `KeyValueStore`가 쓰는 저장 매체입니다. 플래시 (뱅크 2)의 마지막 섹터를
STM32duino의 EEPROM 에뮬레이션 (`eeprom_buffer_fill/flush`, 8 KB 버퍼 중
앞쪽 `KeyValueStore::CAPACITY` 바이트를 사용)을 통해 씁니다.

| 메서드 | 설명 |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` + 버퍼를 바이트 단위로 읽기 |
| `write(src, n)` | **빠름**: `noInterrupts()` 아래에서 이미지를 자체 버퍼로 복사하고 “쓰기 대기” 플래그를 세웁니다. 비행 태스크의 `KvPreferences::end()`에서 호출됩니다 |
| `bool service()` | **느림**: (`noInterrupts()` 아래에서) 에뮬레이션 버퍼로 스냅샷을 만들고 `eeprom_buffer_flush()`를 실행합니다. 128 KB 섹터를 지우고 (몇 초) 씁니다. 백그라운드 태스크 `storage`에서만 호출합니다 |
| `hasPending()`, `flushCount()` | 진단 |
| `static instance()`, `static store()` | 저장 매체와 펌웨어 공용 `KeyValueStore` |

비행이 멈추지 않는 이유: 설정 섹터는 뱅크 2에, 코드는 뱅크 1에 있고, H7의 플래시는
한 뱅크에 쓰는 동안 다른 뱅크를 읽을 수 있습니다. 또한 비행 태스크가 백그라운드 태스크를 선점합니다.

## `compat/Preferences.h`

**파일:** `hal/stm32/compat/Preferences.h` — env `stm32h743` (그리고
`native-stm32`)에서는 `compat/` 디렉터리가 라이브러리보다 앞서 `-I`에 들어가므로,
센서 드라이버, 오토 트리머, 로그 설정의 `#include <Preferences.h>`가
이 파일을 찾습니다. `class Preferences : public KvPreferences`를
`Stm32FlashStorage::store()` 위에 얹은 것으로, API는 ESP32의 NVS와 같습니다 ([storage.md](storage.md#kvpreferences)).

## `Stm32SdCard`

**파일:** `hal/stm32/Stm32SdCard.h` · **상속:** `IBlockDevice` · **핀:** `src/stm32/sd_msp.cpp`

SDMMC1에 연결된 SD 카드입니다. 4비트 버스이며, `HAL_SD`를 폴링 모드 (DMA와 인터럽트 없음)로
**하드웨어 흐름 제어와 함께** 씁니다. 비행 태스크가 블록 중간에 쓰기 태스크를 선점하므로,
흐름 제어가 없으면 FIFO가 넘쳤습니다
(`HAL_SD_ERROR_RX_OVERRUN`, 0x20). 보드에서는 콘솔과 기록이 몇 초 동안 멈추는 증상으로 나타났습니다.
핀 PC8~PC11 (D0~D3), PC12 (CK), PD2 (CMD)는 DevEBox와 WeAct의 µSD 슬롯입니다.
SDMMC 코어는 PLL1Q = 48 MHz로 클럭을 받고, `ClockDiv = 1` → **24 MHz**입니다.
24 MHz에서 첫 읽기가 실패하면 12와 6을 시도합니다.

| 멤버 | 설명 |
|---|---|
| `bool begin()` | 버스를 올리고, 카드를 식별하고, 시험 삼아 읽어 봅니다. `false`는 카드가 없음을 뜻하고, `initError()`가 코드를 줍니다 |
| `read` / `write` | 한 번에 4 KB 이하의 덩어리로 처리합니다 (짧은 휴지 포함). 4의 배수가 아닌 주소는 정렬된 버퍼를 거쳐 복사합니다 (HAL이 FIFO를 워드 단위로 읽기 때문입니다). 실패하면 한 번 재시도합니다 |
| 대기 | 쓰기 뒤에 다음 접근을 하기 전에 카드가 전송 상태로 돌아오기를 기다립니다 (`Rtos::sleepMs(1)`: 백그라운드 태스크가 굶지 않도록), 최대 1초. 읽기 뒤에는 불필요한 상태 요청을 보내지 않습니다. 전원을 켤 때의 대조가 수만 개의 섹터를 읽기 때문입니다 |
| `blockCount()`, `cardType()`, `clockDivider()`, `lastErrorCode()` | 상태 줄용 |
| `readOps`, `writeOps`, `errors`, `retries` | 카운터 |

## `ResetCause`

**파일:** `hal/ResetCause.h` · `readResetCause()`, `isCrashReset()`, `resetCauseName()`

재부팅의 원인을 두 보드에서 똑같이 알려 줍니다. ESP32는 `esp_reset_reason()`,
STM32는 `RCC->RSR` 플래그입니다 (한 번 읽고 지웁니다. H7에서는 `PINRSTF`가
어떤 리셋에서나 세워지므로, 더 구체적인 원인을 먼저 확인합니다:
워치독 → 전원 투입 → 전압 강하 → 소프트웨어 리셋).
패닉, 워치독, 전원 전압 강하는 “고장”으로 취급되며, 이 경우 블랙박스는
즉시 기록을 시작합니다.

## `Rtos`

**파일:** `hal/Rtos.h` · namespace

| 멤버 | 설명 |
|---|---|
| `PRIORITY_BACKGROUND` (1), `PRIORITY_TELEMETRY` (2), `PRIORITY_FLIGHT` (5) | 태스크 우선순위 |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | ESP32는 `xTaskCreatePinnedToCore(..., 코어 0)`이며 스택은 바이트 단위입니다. STM32는 `xTaskCreate`이며 스택을 워드로 환산합니다. `handle`은 `xTaskNotifyGive`용입니다 |
| `void sleepMs(ms)` | `vTaskDelay`. 스케줄러가 시작되기 전 (STM32의 `setup()`)에는 `delay()` |
| `class CriticalSection` | `enter()`/`exit()`: ESP32는 `portMUX` 스핀락, STM32는 `taskENTER_CRITICAL()`입니다. 내부에서는 바이트 복사 (블랙박스 큐)만 합니다 |
| `uint32_t freeHeapBytes()` | ESP32는 `ESP.getFreeHeap()`, STM32는 `xPortGetFreeHeapSize()` |

## 진입점 `src/stm32/main.cpp`

전체 펌웨어입니다. `src/main.cpp`와 같은 객체를 쓰되, Wi-Fi 대신 MAVLink 텔레메트리를,
`loop()` 대신 FreeRTOS 태스크를 사용합니다 ([application.md](application.md#srcstm32maincpp--stm32h743)).
