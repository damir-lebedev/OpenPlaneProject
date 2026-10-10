# HAL – hårdvaruabstraktion

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/hal.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

HAL är det enda lagret som får känna till en specifik mikrokontroller. Gränssnitten finns
i `include/hal/`, implementeringarna är:

- `include/hal/esp32/` – ESP32 (Arduino core 2.0.x);
- `include/hal/stm32/` – STM32H743 (STM32duino 3.x), **huvudimplementeringen**: den fullständiga firmwaren byggs (`pio run -e stm32h743-devebox`) och körs på datorn (`pio test -e native-stm32`); på DevEBox-kortet har SD-kortet, den svarta lådan, iBUS och servona verifierats, sensorerna ännu inte;
- `hal/Rtos.h` – FreeRTOS-uppgifter, identiska på båda plattformarna.

Allt ovanför arbetar bara med gränssnitten, så att flytta till en annan mikrokontroller
innebär en ny implementering av `IBoard`, inte att sensorerna skrivs om.

---

## namnrymd `ServoChannel`

**Fil:** `hal/IBoard.h`

Utgångsindex för `IBoard::servo(channel)`. En platt lista snarare än namngivna
metoder – att lägga till en utgång ändrar inte gränssnittet `IBoard`. Ordningen
stämmer med raderna i tabellen `FlightOutputs::outputInfo()`.

| Konstant | Värde |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 – lastsläpp (`Feature::PAYLOAD_DROP`) |
| `AUX2` | 6 – kamera (`Knob::CAMERA_TILT`, `Feature::CAMERA_STAB`) |
| `COUNT` | 7 |

---

## `IBoard`

**Fil:** `hal/IBoard.h` · **Typ:** gränssnitt · **Implementeringar:** `Esp32Board`, `Stm32Board`

Den enda ingångspunkten till hårdvaran. Ingenting ovanför inkluderar `<Wire.h>`,
`<SPI.h>` eller `HardwareSerial`, och ingenting anropar LEDC direkt.

| Metod | Beskrivning |
|---|---|
| `virtual void begin()` | Engångsinitiering av I2C-/SPI-bussarna. UART:er öppnas av sina ägare (`IBusReceiver`, GPS) med sin egen baudhastighet, PWM av `FlightOutputs::begin()` |
| `virtual II2CBus& i2c()` | Sensorbussen |
| `virtual ISpiBus& spi()` | SPI-bussen |
| `virtual II2CBus* displayI2c()` | En andra I2C-buss enbart för skärmen; `nullptr` om den saknas |
| `virtual IUartPort& rcUart()` | iBUS-mottagarens UART |
| `virtual IUartPort& gpsUart()` | GPS:ens UART |
| `virtual IUartPort* telemetryUart()` | MAVLink-radiomodemets UART; `nullptr` som standard (ESP32 har ingen ledig UART) |
| `virtual IServoOutput& servo(uint8_t channel)` | En PWM-utgång efter index `ServoChannel::*` |
| `virtual void setBuzzer(bool on)` | Summern `PIN_BUZZER`; gör ingenting som standard |

---

## `II2CBus`

**Fil:** `hal/II2CBus.h` · **Typ:** gränssnitt med icke-virtuella hjälpare ·
**Implementeringar:** `Esp32I2CBus`, `Stm32I2CBus`

En abstraktion av en I2C-buss i stil med `Wire`. Implementeringen fixerar stiften
och frekvensen i sin konstruktor, så `begin()`/`setClock()` tar inga stift –
bussen initieras exakt en gång, även om flera enheter delar den.

| Metod | Beskrivning |
|---|---|
| `begin()`, `setClock(hz)` | Initiering, frekvens |
| `beginTransmission(addr)`, `write(byte)`, `write(data, len)`, `endTransmission(sendStop = true)` | Skrivning; `endTransmission` returnerar 0 vid lyckat resultat (som `Wire`) |
| `requestFrom(addr, n)`, `available()`, `read()` | Läsning |
| `bool writeRegister(addr, reg, value)` | Hjälpare: skriv ett enda register; `false` – NACK |
| `bool readRegisters(addr, reg, buf, count)` | Hjälpare: upprepad start + läsning av `count` byte. `false` vid NACK **eller om färre än `count` byte kom**; bufferten lämnas orörd |
| `int readRegister(addr, reg)` | Registrets värde eller `-1` |
| `bool probe(addr)` | Enheten svarar på adressen med ACK |

Invariant: vid fel skriver hjälparna inte till bufferten – drivrutinen behåller
föregående data snarare än skräp (den `0xFF` som `read()` returnerar från en
tom buffert).

---

## `ISpiBus`

**Fil:** `hal/ISpiBus.h` · **Typ:** gränssnitt · **Implementeringar:** `Esp32SpiBus`, `Stm32SpiBus`

En SPI-buss **utan CS-hantering**: flera enheter delar en buss, och
`SpiRegisterDevice` växlar CS.

| Metod | Beskrivning |
|---|---|
| `begin()` | Ställ in SCK/MISO/MOSI (stiften finns i implementeringens konstruktor) |
| `beginTransaction(clockHz, spiMode)` | `spiMode` 0..3 (CPOL/CPHA) |
| `uint8_t transfer(data)` | Full duplex-utbyte av en byte |
| `endTransaction()` | Slut på transaktionen |

---

## `IUartPort`

**Fil:** `hal/IUartPort.h` · **Typ:** gränssnitt · **Implementeringar:** `Esp32UartPort`, `Stm32UartPort`

En UART i stil med `HardwareSerial`, men `begin()` tar bara baudhastigheten:
implementeringen fixerar stiften och formatet (8N1).

| Metod | Beskrivning |
|---|---|
| `begin(baud)` | Öppna porten |
| `int available()`, `int read()` | Mottagning |
| `size_t write(byte)`, `size_t write(buffer, size)` | Sändning |
| `virtual int availableForWrite()` | Ledigt utrymme i sändbufferten; `-1` – okänt (standard). Telemetrin använder det för att skjuta upp en ram i stället för att vänta |

---

## `IServoOutput`

**Fil:** `hal/IServoOutput.h` · **Typ:** gränssnitt · **Implementeringar:** `Esp32ServoOutput`, `Stm32ServoOutput`

En enda PWM-utgång. Implementeringen fixerar stiftet.

| Metod | Beskrivning |
|---|---|
| `bool attach(minUs, maxUs)` | Tilldela kanalen/timern och konfigurera stiftet; intervallet för pulsbegränsning. `true` betyder bara att mikrokontrollern tilldelade resurserna, **inte** att ett servo är anslutet |
| `writeMicroseconds(us)` | Pulsbredd, µs (begränsad till intervallet från `attach`) |
| `bool isAttached() const` | Resultatet av `attach()` |
| `virtual int32_t measurePulseUs()` | Diagnostik: den verkliga pulsbredden på stiftet eller `-1`. Standardimplementeringen returnerar `-1` |

---

## `IFlashRegion`

**Fil:** `hal/IFlashRegion.h` · **Typ:** gränssnitt · **Implementeringar:** `Esp32FlashPartition`, `SdFileRegion`

Ett NOR-flashområde för loggen (den svarta lådan): radering bara i sektorer om 4 KB
(raderade byte läses som `0xFF`), skrivning nollställer bara bitar – du kan skriva i
raderade byte, även bit för bit i en och samma sida. På ESP32 stoppar både skrivning
och radering båda kärnorna – anroparen avgör när det är acceptabelt.

| Metod | Beskrivning |
|---|---|
| `uint32_t size() const` | Områdets storlek, byte; 0 – det finns inget område |
| `bool read(offset, data, length)` | Läs |
| `bool write(offset, data, length)` | Skriv (i raderade byte) |
| `bool erase(offset, length)` | Radera; adressen och längden är multiplar av 4096 |

`Esp32FlashPartition(const char* name)` – en datapartition efter namn från
partitionstabellen (`esp_partition_*`); `begin()` hittar partitionen (efter att
kärnan har startat), och om den saknas – `false` och `size() == 0`.

---

## `IBlockDevice`

**Fil:** `hal/IBlockDevice.h` · **Typ:** gränssnitt · **Implementeringar:** `Stm32SdCard` (i tester – `fake::SdCardModel`)

Ett SD-kort som en array av block på 512 byte. Det finns ingen radering: ett block kan skrivas över.

| Metod | Beskrivning |
|---|---|
| `uint32_t blockCount() const` | Storlek i block; 0 – det finns inget kort |
| `bool read(block, data, count)` / `write(...)` | `count` block i följd, `data` – valfri adress |

## `SdFileRegion`

**Fil:** `hal/SdFileRegion.h` · **Ärver:** `IFlashRegion` · **Beror på:** `IBlockDevice`, `Fat32::locate`

Ett område för den svarta lådan på SD-kortet: en fil i roten av FAT32-volymen (som
standard `BLACKBOX.BIN`), skapad i förväg på datorn som ett enda sammanhängande
stycke (`tools/blackbox.py
sd-prepare`) och fylld med `0xFF`. Filen **lokaliseras** bara (FAT-
tabellerna och katalogen rörs inte); därefter skrivs råa block
inuti den. För `BlackBoxStorage` är det samma `IFlashRegion` som ESP32:s
flashpartition.

| Metod | Beskrivning |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes` är områdets tak: sektorkontrollen vid start tar längre tid ju större det är |
| `Fat32::Result begin()` | Hitta filen. `Ok` – `size() > 0`; annars orsaken (`Fat32::describe()`): inget kort, inte FAT32, ingen fil, fragmenterad, tom |
| `size()` | Filen (högst `maxBytes`), avrundad nedåt till en sektor om 4 KB; 0 – det finns inget område |
| `read` / `write` | Valfri förskjutning och längd. Ett delblock läses, kompletteras och skrivs i sin helhet; det block som just skrevs kommer man ihåg (write-through-cache): på varandra följande sidor om 256 byte läser inte kortet igen. Ett strömavbrott förlorar ingenting som redan har returnerat från `write()` |
| `erase(offset, length)` | En multipel av 4096; skriver `0xFF` (kortet har sin egen radering internt, utsidan behöver den inte) |

## `IRegisterDevice`

**Fil:** `hal/RegisterDevice.h` · **Typ:** gränssnitt ·
**Implementeringar:** `I2cRegisterDevice`, `SpiRegisterDevice`

”En uppsättning 8-bitarsregister”. En sensordrivrutin skrivs en gång, och bussen
väljs när objektet skapas i `SensorSelection.h`.

| Metod | Beskrivning |
|---|---|
| `virtual void begin()` | Förbered enhetens ledningar (för SPI – CS). Gör ingenting som standard |
| `virtual bool probe()` | Enheten svarade (för SPI alltid `true` – det finns inget ACK, ID-registret kontrolleras i stället) |
| `virtual bool writeRegister(reg, value)` | Skriv ett register |
| `virtual bool writeRegisters(reg, data, count)` | Skriv i följd (autoinkrement av adressen) |
| `virtual bool readRegisters(reg, buffer, count)` | Läs `count` byte i följd; vid `false` lämnas bufferten orörd |
| `int readRegister(reg)` | Värdet eller `-1` (en icke-virtuell hjälpare) |

---

## `I2cRegisterDevice`

**Fil:** `hal/RegisterDevice.h` · **Ärver:** `IRegisterDevice`

En enhet på en `II2CBus` med en 7-bitarsadress. Alla operationer delegeras till
hjälparna i `II2CBus`.

| Metod | Beskrivning |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress` är kretsens andra adress (stiftet SDO/SA0): LSM6DSV 0x6A/0x6B, ICM-45686 0x68/0x69, SPL06 0x76/0x77, BMP581 0x46/0x47 |
| `begin()` | den primära svarar inte men den alternativa gör det – arbeta vidare med den alternativa därefter |
| `probe()`, `writeRegister()`, `writeRegisters()`, `readRegisters()` | → hjälparna `II2CBus(address, …)` |
| `uint8_t getAddress() const` | enhetens aktuella adress |

---

## `SpiRegisterDevice`

**Fil:** `hal/RegisterDevice.h` · **Ärver:** `IRegisterDevice`

En enhet på en `ISpiBus` med eget CS-stift. Bosch-/InvenSense-protokollet:
en läsning är adressen med biten `0x80`, en skrivning med bit 7 nollställd.

| Metod | Beskrivning |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` är hur många ”skräp”-byte kretsen ger ut efter adressen före data (BMP388 – 1, ICM42688 – 0); `mode` är SPI-läget 0..3 |
| `begin()` | `pinMode(cs, OUTPUT)`, CS = HIGH |
| `probe()` | Alltid `true` |
| `writeRegister(reg, value)` | CS↓, `reg & 0x7F`, `value`, CS↑; alltid `true` |
| `readRegisters(reg, buf, n)` | CS↓, `reg \| 0x80`, hoppa över `dummyReadBytes`, `n` byte, CS↑; alltid `true` |

Varje operation är en separat transaktion `beginTransaction(clockHz, spiMode)` …
`endTransaction()`.

---

## `Esp32Board`

**Fil:** `hal/esp32/Esp32Board.h` · **Ärver:** `IBoard`

Det enda stället som skapar de konkreta ESP32-objekten för kringutrustning och känner till
stiften från `Config.h`.

| Fält | Typ | Vad det är |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `Wire` på `PIN_I2C_SDA/SCL`, 400 kHz |
| `displayBus` | `Esp32I2CBus` | `Wire1` på `PIN_I2C2_SDA/SCL` – bara om `SOC_I2C_NUM > 1` |
| `spiBus` | `Esp32SpiBus` | Den globala `SPI` |
| `rcSerial`, `rcPort` | `HardwareSerial(1)`, `Esp32UartPort` | iBUS på `PIN_IBUS`, bara RX |
| `gpsSerial`, `gpsPort` | `HardwareSerial(UART_NUM_GPS)`, `Esp32UartPort` | GPS på `PIN_GPS_RX/TX` |
| `servos[7]` | `Esp32ServoOutput` | LEDC-kanalerna 0..6 i ordningen för `ServoChannel` (AUX1/AUX2 – `PIN_AUX1/2`, om de är dragna) |

| Metod | Beskrivning |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, sedan `displayBus.begin()` om den andra bussen finns; summerns stift |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` om stiftet är draget |
| `displayI2c()` | `&displayBus` om `hasDisplayBus()`, annars `nullptr` |
| `static constexpr bool hasDisplayBus()` | Båda stiften för den andra bussen är ≥ 0. Den finns (liksom fältet `displayBus`) bara när `SOC_I2C_NUM > 1` – C3 har en enda I2C-styrenhet |
| resten | Returnerar motsvarande fält |

---

## `Esp32I2CBus`

**Fil:** `hal/esp32/Esp32I2CBus.h` · **Ärver:** `II2CBus`

Ett tunt omslag kring `TwoWire` (`Wire` eller `Wire1`).

| Metod | Beskrivning |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | Sparar parametrarna |
| `begin()` | `wire.begin(sda, scl, hz)` och `wire.setTimeOut(TIMEOUT_MS)` – det enda anropet av `wire.begin()` |
| resten | Direkt delegering till `TwoWire` |

`TIMEOUT_MS = 5`: att läsa 14 IMU-byte vid 400 kHz tar ~0,4 ms; en transaktion
som hängt sig på grund av störningar skulle annars stoppa slingan i standardtiden 50 ms.

---

## `Esp32SpiBus`

**Fil:** `hal/esp32/Esp32SpiBus.h` · **Ärver:** `ISpiBus`

Ett omslag kring den globala `SPI`. `begin()` → `SPI.begin(sck, miso, mosi, -1)` (CS
hålls av enheterna). `beginTransaction()` bygger `SPISettings(hz, MSBFIRST,
SPI_MODEn)`; `spiModeOf()` omvandlar 0..3 till Arduino-konstanter, och ett okänt
värde → `SPI_MODE0`.

---

## `Esp32UartPort`

**Fil:** `hal/esp32/Esp32UartPort.h` · **Ärver:** `IUartPort`

Ett omslag kring `HardwareSerial`: `begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)`; `tx = -1` – bara mottagning. Resten är delegering.

---

## `Esp32ServoOutput`

**Fil:** `hal/esp32/Esp32ServoOutput.h` · **Ärver:** `IServoOutput`

PWM direkt via LEDC (`ledcSetup/ledcAttachPin/ledcWrite` i Arduino core 2.x).
Biblioteket ESP32Servo **används inte**: version 3.2.1 på S3 blandade ihop MCPWM-
blocken (GPIO6/7 upprepade GPIO4/5).

| Konstant | Värde |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384 (≈1,2 µs per steg) |

| Metod | Beskrivning |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | Stift `< 0` – utgången är inte dragen |
| `attach(minUs, maxUs)` | Sparar intervallet; stift < 0 → `false`; annars `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | Inte ansluten – ingenting; annars `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | Slår på ingångsbufferten för samma GPIO (`PIN_INPUT_ENABLE`, utgången rörs inte) och mäter `pulseIn(pin, HIGH, 30 ms)`; ingen puls → `-1` |

Kanalerna 2n och 2n+1 delar en LEDC-timer – alla utgångar går på 50 Hz, så det finns ingen konflikt.

---

# Implementering för STM32H743

Nästa generations kort är STM32H743VIT6 (Cortex-M7 480 MHz, 2 MB flash,
1 MB RAM). Den fullständiga firmwaren byggs (env `stm32h743` – PlatformIO-kortet
`weact_mini_h743vitx`, och `stm32h743-devebox` – DevEBox H743, konsol via
USB CDC), klarar cppcheck och testerna på datorn (env `native-stm32` med
STM32duino-attrapplagret). På DevEBox-kortet **utan sensorer** har följande verifierats: uppstart, SD-kortet, den svarta lådan –
[tester på kortet](../TESTING.md#tester-på-stm32-kortet) – samt iBUS-mottagning, ARM, PWM till
servona och motorn: planet styrs från sändaren i manuellt läge (starten filmades).
Sensorerna har ännu inte anslutits till kortet.
Stiftbeläggningen är blocket `BOARD_STM32H743` i [`Config.h`](config.md#stm32h743vit6-board_stm32h743).

De allmänna skillnaderna mot ESP32 som det här lagret döljer:

- **Kärnan väljer kringutrustningen.** STM32duino hittar själv styrenheten (I2C1/I2C2,
  SPI2, USART3, UART4, UART7, TIMx) från stiftnumren med hjälp av
  variantens tabeller `PeripheralPins`, så det finns inga UART-/kanalnummer i `Config.h`.
- **Stiftnummer** är variantens ”Arduino-stift” (`PA0`, `PD14`...), inte GPIO:er; för
  analoga stift är de `0xC0 + N`, och därför är stiften i STM32-blocket `int16_t`.
- **UART-stift** anges när objektet `Uart(rx, tx)` skapas, inte i `begin()`.

## `Stm32Board`

**Fil:** `hal/stm32/Stm32Board.h` · **Ärver:** `IBoard`

Samma sak som `Esp32Board`, ovanpå STM32duino.

| Fält | Typ | Vad det är |
|---|---|---|
| `displayWire` | `TwoWire` | Den andra I2C-styrenheten (den globala `Wire` är upptagen av sensorerna). Deklareras före `displayBus`, som håller en referens till den |
| `i2cBus` | `Stm32I2CBus` | `Wire` på `PIN_I2C_SDA/SCL` (I2C2: PB11/PB10), 400 kHz |
| `displayBus` | `Stm32I2CBus` | `displayWire` på `PIN_I2C2_SDA/SCL` (I2C1: PB9/PB8) – den andra bussen finns alltid |
| `spiBus` | `Stm32SpiBus` | Den globala `SPI` på `PIN_SENSOR_SPI_*` (SPI2) |
| `rcSerial`, `rcPort` | `Uart`, `Stm32UartPort` | iBUS: UART7, RX `PIN_IBUS` (PE7), TX `PIN_IBUS_TX` (PE8, reserverad för iBUS-SENS) |
| `gpsSerial`, `gpsPort` | `Uart`, `Stm32UartPort` | GPS: USART3, `PIN_GPS_RX/TX` (PD9/PD8) |
| `telemetrySerial`, `telemetryPort` | `Uart`, `Stm32UartPort` | MAVLink-radiomodemet: UART4, `PIN_TELEM_RX/TX` (PD0/PD1) |
| `servos[7]` | `Stm32ServoOutput` | I ordningen för `ServoChannel` (AUX1 – PD15/TIM4, AUX2 – PE9/TIM1) |

| Metod | Beskrivning |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, `displayBus.begin()`, summerns stift |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | Alltid `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | Omvandlar ett stift från `Config.h` till kärn-API:ts typ |
| resten | Returnerar motsvarande fält |

## `Stm32I2CBus`

**Fil:** `hal/stm32/Stm32I2CBus.h` · **Ärver:** `II2CBus`

| Metod | Beskrivning |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | Sparar parametrarna |
| `begin()` | `setSDA()`/`setSCL()` (verkar bara före `begin()`), `wire.begin()`, `wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)`; resultatet av typen `size_t` omvandlas till `uint8_t` |
| resten | Direkt delegering till `TwoWire` |

I STM32duino är tidsgränsen för en transaktion inte en metod utan makrot
`I2C_TIMEOUT_TICK` (ms, 100 som standard). I env `stm32h743` sätts det med
flaggan `-D I2C_TIMEOUT_TICK=5` – av samma skäl som `TIMEOUT_MS` i `Esp32I2CBus`.

## `Stm32SpiBus`

**Fil:** `hal/stm32/Stm32SpiBus.h` · **Ärver:** `ISpiBus`

Ett omslag kring `SPIClass&`. `begin()` → `setSCLK/setMISO/setMOSI` + `spi.begin()`;
hårdvaru-NSS används inte – CS växlas av `SpiRegisterDevice`, som på ESP32.
`beginTransaction()` bygger `SPISettings(hz, MSBFIRST, SPIMode)`; `spiModeOf()`
omvandlar 0..3 till `SPI_MODEn`, och ett okänt värde → `SPI_MODE0`.

## `Stm32UartPort`

**Fil:** `hal/stm32/Stm32UartPort.h` · **Ärver:** `IUartPort`

Ett omslag kring `HardwareSerial&` (i STM32duino 3.x är det den abstrakta basen
`arduino::HardwareSerial`; `Stm32Board` skapar det konkreta objektet `Uart`).
`begin(baud)` → `serial.begin(baud, SERIAL_8N1)`; `availableForWrite()` kommer
från `HardwareSerial`. Buffertarna (`SERIAL_RX/TX_BUFFER_SIZE` i env): mottagning 256
byte (en NAV-PVT-ram är 100, standardvärdet 64 är för lite), sändning 1024 (loggrader och
MAVLink-ramar utan väntan).

## `Stm32ServoOutput`

**Fil:** `hal/stm32/Stm32ServoOutput.h` · **Ärver:** `IServoOutput`

Hårdvaru-PWM från timer via `HardwareTimer`, 50 Hz. Pulsen genereras av
timern utan avbrott och utan CPU:n – till skillnad från biblioteket `Servo` för STM32,
som växlar stiften från avbrottet i en enda timer och ger jitter.

| Konstant | Värde |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4 – hur många olika timrar utgångarna kan uppta (TIM2 och TIM4 används nu) |

| Metod | Beskrivning |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | Stift `< 0` – utgången är inte dragen |
| `attach(minUs, maxUs)` | Timern och kanalen tas från `PinMap_TIM` efter stift (`pinmap_peripheral`, `STM_PIN_CHANNEL`), som i `analogWrite()`. Ingen timer på stiftet eller poolen är uttömd → `false`. Annars `setMode(PWM1)`, compare 0 (ingen puls före den första skrivningen), `resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` → `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`. Compare-registret förladdas – värdet träder i kraft från nästa period |
| `measurePulseUs()` | `pulseIn(pin, HIGH, 30 ms)` utan att konfigurera om stiftet: på STM32 ser registret IDR nivån även i läget för alternativ funktion |
| `static acquireTimer(TIM_TypeDef*)` | En gemensam pool: **en `HardwareTimer` per TIMx**. Ett andra objekt för samma timer skulle skriva över kärnans hanterare (`HardwareTimer_Handle[index]`). Perioden ställs in vid den första utgången på timern; `setOverflow(MICROSEC_FORMAT)` väljer förskalaren – ett steg på ~0,3 µs vid en timerklocka på 240 MHz |

## `Stm32FlashStorage`

**Fil:** `hal/stm32/Stm32FlashStorage.h` · **Ärver:** `IFlashStorage` ([storage.md](storage.md))

Mediet för `KeyValueStore` på STM32: den sista flashsektorn (bank 2) via
STM32duinos EEPROM-emulering (`eeprom_buffer_fill/flush`, en buffert på 8 KB av vilken
de första `KeyValueStore::CAPACITY` byten används).

| Metod | Beskrivning |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` + en läsning av bufferten byte för byte |
| `write(src, n)` | **snabb**: kopierar avbilden till sin egen buffert under `noInterrupts()`, sätter flaggan ”väntande skrivning”. Anropas från `KvPreferences::end()` i flyguppgiften |
| `bool service()` | **långsam**: en ögonblicksbild in i emuleringsbufferten (under `noInterrupts()`) och `eeprom_buffer_flush()` – radering av en sektor på 128 KB (sekunder) och skrivning. Bara från bakgrundsuppgiften `storage` |
| `hasPending()`, `flushCount()` | diagnostik |
| `static instance()`, `static store()` | mediet och firmwarens gemensamma `KeyValueStore` |

Varför flygningen inte fryser: inställningssektorn ligger i bank 2 och koden i
bank 1, H7:s flash kan läsas från en bank medan den andra skrivs;
flyguppgiften avbryter bakgrundsuppgiften.

## `compat/Preferences.h`

**Fil:** `hal/stm32/compat/Preferences.h` – i env `stm32h743` (och
`native-stm32`) kommer katalogen `compat/` i `-I` före biblioteken, och
`#include <Preferences.h>` i sensordrivrutinerna, autotrimmern och logg-
inställningarna hittar den. `class Preferences : public KvPreferences` ovanpå
`Stm32FlashStorage::store()` – samma API som ESP32:s NVS ([storage.md](storage.md#kvpreferences)).

## `Stm32SdCard`

**Fil:** `hal/stm32/Stm32SdCard.h` · **Ärver:** `IBlockDevice` · **Stift:** `src/stm32/sd_msp.cpp`

Ett SD-kort på SDMMC1: en 4-bitarsbuss, `HAL_SD` i polling-läge (ingen DMA och inga
avbrott) **med hårdvaruflödeskontroll**: flyguppgiften avbryter skriv-
uppgiften mitt i ett block, och utan den flödade FIFO:n över
(`HAL_SD_ERROR_RX_OVERRUN`, 0x20) – på kortet visade det sig som att konsolen och
loggningen frös i sekunder. Stiften PC8..PC11 (D0..D3), PC12 (CK), PD2 (CMD) är µSD-platsen på
DevEBox och WeAct. SDMMC-kärnan klockas från PLL1Q = 48 MHz, `ClockDiv = 1` →
**24 MHz**; om den första läsningen misslyckas vid 24 MHz provas 12 och 6.

| Medlem | Beskrivning |
|---|---|
| `bool begin()` | Starta bussen, identifiera kortet, en provläsning. `false` – det finns inget kort; `initError()` – koden |
| `read` / `write` | I bitar om högst 4 KB (korta pauser); en adress som inte är en multipel av 4 kopieras via en justerad buffert (HAL läser FIFO:n ordvis). Vid fel – ett omförsök |
| väntan | Före en åtkomst efter en skrivning väntar den på att kortet återgår till överföringstillståndet (`Rtos::sleepMs(1)`: bakgrundsuppgifterna svälter inte), upp till 1 s. Efter en läsning skickas ingen extra statusförfrågan – kontrollen vid start läser tiotusentals sektorer |
| `blockCount()`, `cardType()`, `clockDivider()`, `lastErrorCode()` | För statusraden |
| `readOps`, `writeOps`, `errors`, `retries` | Räknare |

## `ResetCause`

**Fil:** `hal/ResetCause.h` · `readResetCause()`, `isCrashReset()`, `resetCauseName()`

Återställningsorsaken, identisk på båda korten. ESP32 – `esp_reset_reason()`;
STM32 – flaggorna i `RCC->RSR` (läses en gång och rensas; på H7 sätts `PINRSTF`
vid varje återställning, så de mer specifika orsakerna kontrolleras först:
watchdog → strömpåslag → spänningsfall → mjukvaruåterställning).
En panic, watchdogarna och en spänningsdipp räknas som en ”krasch”: den svarta lådan börjar
spela in direkt vid dem.

## `Rtos`

**Fil:** `hal/Rtos.h` · namnrymd

| Medlem | Beskrivning |
|---|---|
| `PRIORITY_BACKGROUND` (1), `PRIORITY_TELEMETRY` (2), `PRIORITY_FLIGHT` (5) | uppgifternas prioriteter |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | ESP32 – `xTaskCreatePinnedToCore(..., core 0)`, stacken i byte; STM32 – `xTaskCreate`, stacken omvandlas till ord; `handle` är för `xTaskNotifyGive` |
| `void sleepMs(ms)` | `vTaskDelay`; innan schemaläggaren har startat (STM32 `setup()`) – `delay()` |
| `class CriticalSection` | `enter()`/`exit()`: ESP32 – spinlocket `portMUX`, STM32 – `taskENTER_CRITICAL()`. Inuti bara kopiering av byte (den svarta lådans kö) |
| `uint32_t freeHeapBytes()` | ESP32 – `ESP.getFreeHeap()`; STM32 – `xPortGetFreeHeapSize()` |

## Ingångspunkten `src/stm32/main.cpp`

Den fullständiga firmwaren: samma objekt som `src/main.cpp`, MAVLink-telemetri i stället för
Wi-Fi, FreeRTOS-uppgifter i stället för `loop()` – [application.md](application.md#srcstm32maincpp--stm32h743).
