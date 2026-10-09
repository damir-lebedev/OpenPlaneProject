# HAL — Hardware-Abstraktion

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/hal.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

[← Referenz](README.md)

Das HAL ist die einzige Schicht, die einen konkreten MCU kennen darf. Die
Schnittstellen liegen in `include/hal/`; die Implementierungen sind:

- `include/hal/esp32/` — ESP32 (Arduino core 2.0.x);
- `include/hal/stm32/` — STM32H743 (STM32duino 3.x), **die wichtigste**: Die vollständige Firmware lässt sich bauen (`pio run -e stm32h743-devebox`) und läuft auf dem PC (`pio test -e native-stm32`); auf dem DevEBox-Board sind SD-Karte, Blackbox, iBUS und Servos geprüft, die Sensoren noch nicht;
- `hal/Rtos.h` — FreeRTOS-Tasks, auf beiden Plattformen gleich.

Alles darüber arbeitet nur mit den Schnittstellen. Der Wechsel auf einen anderen
MCU bedeutet daher eine neue `IBoard`-Implementierung und kein Umschreiben der Sensoren.

---

## namespace `ServoChannel`

**Datei:** `hal/IBoard.h`

Die Ausgangsindizes für `IBoard::servo(channel)`. Eine flache Liste statt benannter
Methoden — ein zusätzlicher Ausgang ändert die Schnittstelle `IBoard` nicht. Die
Reihenfolge entspricht den Zeilen der Tabelle `FlightOutputs::outputInfo()`.

| Konstante | Wert |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 — Abwurf der Last (`Feature::PAYLOAD_DROP`) |
| `AUX2` | 6 — Kamera (`Knob::CAMERA_TILT`, `Feature::CAMERA_STAB`) |
| `COUNT` | 7 |

---

## `IBoard`

**Datei:** `hal/IBoard.h` · **Art:** Schnittstelle · **Implementierungen:** `Esp32Board`, `Stm32Board`

Der einzige Zugang zur Hardware. Nichts darüber bindet `<Wire.h>`, `<SPI.h>`
oder `HardwareSerial` ein, und nichts ruft LEDC direkt auf.

| Methode | Beschreibung |
|---|---|
| `virtual void begin()` | Einmalige Initialisierung der I2C-/SPI-Busse. Die UARTs öffnen ihre Besitzer (`IBusReceiver`, GPS) mit ihrer eigenen Baudrate, das PWM `FlightOutputs::begin()` |
| `virtual II2CBus& i2c()` | Der Sensorbus |
| `virtual ISpiBus& spi()` | Der SPI-Bus |
| `virtual II2CBus* displayI2c()` | Ein zweiter I2C-Bus nur für das Display; `nullptr`, wenn es ihn nicht gibt |
| `virtual IUartPort& rcUart()` | Der UART des iBUS-Empfängers |
| `virtual IUartPort& gpsUart()` | Der UART des GPS |
| `virtual IUartPort* telemetryUart()` | Der UART des MAVLink-Funkmodems; standardmäßig `nullptr` (der ESP32 hat keinen freien UART) |
| `virtual IServoOutput& servo(uint8_t channel)` | Ein PWM-Ausgang über den Index `ServoChannel::*` |
| `virtual void setBuzzer(bool on)` | Der Summer `PIN_BUZZER`; macht standardmäßig nichts |

---

## `II2CBus`

**Datei:** `hal/II2CBus.h` · **Art:** Schnittstelle mit nicht virtuellen Hilfsfunktionen ·
**Implementierungen:** `Esp32I2CBus`, `Stm32I2CBus`

Eine Abstraktion des I2C-Busses in der Form von `Wire`. Die Pins und die Frequenz legt
die Implementierung im Konstruktor fest, daher haben `begin()`/`setClock()` keine Pins —
der Bus wird genau einmal initialisiert, auch wenn mehrere Geräte an ihm hängen.

| Methode | Beschreibung |
|---|---|
| `begin()`, `setClock(hz)` | Initialisierung, Frequenz |
| `beginTransmission(addr)`, `write(byte)`, `write(data, len)`, `endTransmission(sendStop = true)` | Schreiben; `endTransmission` liefert bei Erfolg 0 (wie `Wire`) |
| `requestFrom(addr, n)`, `available()`, `read()` | Lesen |
| `bool writeRegister(addr, reg, value)` | Hilfsfunktion: schreibt ein einzelnes Register; `false` — NACK |
| `bool readRegisters(addr, reg, buf, count)` | Hilfsfunktion: Repeated Start + Lesen von `count` Bytes. `false`, wenn NACK **oder weniger als `count` Bytes ankamen**; der Puffer bleibt dabei unberührt |
| `int readRegister(addr, reg)` | Der Wert des Registers oder `-1` |
| `bool probe(addr)` | Das Gerät antwortet auf die Adresse mit ACK |

Invariante: Bei einem Fehler schreiben die Hilfsfunktionen nicht in den Puffer — der
Treiber behält die vorigen Daten statt Müll (das `0xFF`, das `read()` bei einem leeren
Puffer liefert).

---

## `ISpiBus`

**Datei:** `hal/ISpiBus.h` · **Art:** Schnittstelle · **Implementierungen:** `Esp32SpiBus`, `Stm32SpiBus`

Ein SPI-Bus **ohne CS-Verwaltung**: Mehrere Geräte teilen sich einen Bus, und das CS
schaltet `SpiRegisterDevice`.

| Methode | Beschreibung |
|---|---|
| `begin()` | SCK/MISO/MOSI einrichten (die Pins stehen im Konstruktor der Implementierung) |
| `beginTransaction(clockHz, spiMode)` | `spiMode` 0..3 (CPOL/CPHA) |
| `uint8_t transfer(data)` | Vollduplex-Austausch eines Bytes |
| `endTransaction()` | Ende der Transaktion |

---

## `IUartPort`

**Datei:** `hal/IUartPort.h` · **Art:** Schnittstelle · **Implementierungen:** `Esp32UartPort`, `Stm32UartPort`

Ein UART in der Form von `HardwareSerial`, doch `begin()` nimmt nur die Baudrate:
die Pins und das Format (8N1) legt die Implementierung fest.

| Methode | Beschreibung |
|---|---|
| `begin(baud)` | Den Port öffnen |
| `int available()`, `int read()` | Empfangen |
| `size_t write(byte)`, `size_t write(buffer, size)` | Senden |
| `virtual int availableForWrite()` | Freier Platz im Sendepuffer; `-1` — unbekannt (Standard). Die Telemetrie stellt damit einen Frame zurück, statt zu warten |

---

## `IServoOutput`

**Datei:** `hal/IServoOutput.h` · **Art:** Schnittstelle · **Implementierungen:** `Esp32ServoOutput`, `Stm32ServoOutput`

Ein einzelner PWM-Ausgang. Den Pin legt die Implementierung fest.

| Methode | Beschreibung |
|---|---|
| `bool attach(minUs, maxUs)` | Reserviert den Kanal/Timer und konfiguriert den Pin; der Begrenzungsbereich des Impulses. `true` zeigt nur, dass der MCU die Ressourcen reserviert hat, **nicht**, dass ein Servo angeschlossen ist |
| `writeMicroseconds(us)` | Impulsbreite, µs (auf den Bereich von `attach` begrenzt) |
| `bool isAttached() const` | Das Ergebnis von `attach()` |
| `virtual int32_t measurePulseUs()` | Diagnose: die tatsächliche Impulsbreite am Pin oder `-1`. Die Standardimplementierung liefert `-1` |

---

## `IFlashRegion`

**Datei:** `hal/IFlashRegion.h` · **Art:** Schnittstelle · **Implementierungen:** `Esp32FlashPartition`, `SdFileRegion`

Ein NOR-Flash-Bereich für das Protokoll (die Blackbox): Gelöscht wird nur in
Sektoren von 4 KB (Gelöschtes liest sich als `0xFF`), und das Schreiben setzt nur Bits
auf null — man kann in gelöschte Bytes schreiben, auch stückweise in dieselbe Seite.
Auf dem ESP32 halten sowohl das Schreiben als auch das Löschen beide Kerne an — wann das
zulässig ist, entscheidet der Aufrufer.

| Methode | Beschreibung |
|---|---|
| `uint32_t size() const` | Größe des Bereichs in Bytes; 0 — es gibt keinen Bereich |
| `bool read(offset, data, length)` | Lesen |
| `bool write(offset, data, length)` | Schreiben (in gelöschte Bytes) |
| `bool erase(offset, length)` | Löschen; Adresse und Länge sind Vielfache von 4096 |

`Esp32FlashPartition(const char* name)` — eine Datenpartition, benannt in der
Partitionstabelle (`esp_partition_*`); `begin()` findet die Partition (nach dem Start des
Kerns), und wenn es keine gibt, liefert es `false` und `size() == 0`.

---

## `IBlockDevice`

**Datei:** `hal/IBlockDevice.h` · **Art:** Schnittstelle · **Implementierungen:** `Stm32SdCard` (in den Tests — `fake::SdCardModel`)

Eine SD-Karte als Array von Blöcken zu 512 Bytes. Ein Löschen gibt es nicht: Ein Block lässt sich überschreiben.

| Methode | Beschreibung |
|---|---|
| `uint32_t blockCount() const` | Größe in Blöcken; 0 — es gibt keine Karte |
| `bool read(block, data, count)` / `write(...)` | `count` Blöcke hintereinander, `data` — eine beliebige Adresse |

## `SdFileRegion`

**Datei:** `hal/SdFileRegion.h` · **Erbt von:** `IFlashRegion` · **Abhängig von:** `IBlockDevice`, `Fat32::locate`

Der Blackbox-Bereich auf der SD-Karte: eine Datei im Wurzelverzeichnis von FAT32 (standardmäßig
`BLACKBOX.BIN`), vorab auf dem PC in einem Stück angelegt (`tools/blackbox.py
sd-prepare`) und mit `0xFF` gefüllt. Die Datei wird nur **gefunden** (die FAT-Tabellen
und das Verzeichnis werden nicht angefasst); danach werden rohe Blöcke in sie geschrieben.
Für `BlackBoxStorage` ist das dasselbe `IFlashRegion` wie die Flash-Partition des ESP32.

| Methode | Beschreibung |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes` ist die Obergrenze des Bereichs: Die Zeit für den Abgleich der Sektoren beim Einschalten wächst mit ihr |
| `Fat32::Result begin()` | Die Datei finden. `Ok` — `size() > 0`; sonst der Grund (`Fat32::describe()`): keine Karte, kein FAT32, keine Datei, fragmentiert, leer |
| `size()` | Die Datei (höchstens `maxBytes`), abgerundet auf einen Sektor von 4 KB; 0 — es gibt keinen Bereich |
| `read` / `write` | Beliebiger Offset und beliebige Länge. Ein unvollständiger Block wird gelesen, ergänzt und ganz geschrieben; der gerade geschriebene Block wird gemerkt (Write-Through-Cache): Aufeinanderfolgende Seiten zu 256 Bytes lesen die Karte nicht erneut. Ein Stromausfall verliert nichts von dem, was schon aus `write()` zurückgekehrt ist |
| `erase(offset, length)` | Vielfaches von 4096; schreibt `0xFF` (die Karte hat intern ihr eigenes Löschen, von außen wird es nicht gebraucht) |

## `IRegisterDevice`

**Datei:** `hal/RegisterDevice.h` · **Art:** Schnittstelle ·
**Implementierungen:** `I2cRegisterDevice`, `SpiRegisterDevice`

„Ein Satz von 8-Bit-Registern“. Der Treiber eines Sensors wird nur einmal geschrieben,
und der Bus wird beim Anlegen des Objekts in `SensorSelection.h` gewählt.

| Methode | Beschreibung |
|---|---|
| `virtual void begin()` | Die Leitungen des Geräts vorbereiten (beim SPI — das CS). Standardmäßig nichts |
| `virtual bool probe()` | Das Gerät hat geantwortet (beim SPI immer `true` — es gibt kein ACK, geprüft wird das ID-Register) |
| `virtual bool writeRegister(reg, value)` | Ein Register schreiben |
| `virtual bool writeRegisters(reg, data, count)` | Fortlaufend schreiben (automatisches Hochzählen der Adresse) |
| `virtual bool readRegisters(reg, buffer, count)` | `count` Bytes fortlaufend lesen; bei `false` bleibt der Puffer unberührt |
| `int readRegister(reg)` | Der Wert oder `-1` (eine nicht virtuelle Hilfsfunktion) |

---

## `I2cRegisterDevice`

**Datei:** `hal/RegisterDevice.h` · **Erbt von:** `IRegisterDevice`

Ein Gerät an einem `II2CBus` mit 7-Bit-Adresse. Alle Operationen werden an die
Hilfsfunktionen von `II2CBus` delegiert.

| Methode | Beschreibung |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress` ist die zweite Adresse des Chips (der Pin SDO/SA0): LSM6DSV 0x6A/0x6B, ICM-45686 0x68/0x69, SPL06 0x76/0x77, BMP581 0x46/0x47 |
| `begin()` | die Hauptadresse antwortet nicht, die alternative antwortet — dann wird mit der alternativen weitergearbeitet |
| `probe()`, `writeRegister()`, `writeRegisters()`, `readRegisters()` | → die Hilfsfunktionen von `II2CBus(address, …)` |
| `uint8_t getAddress() const` | die aktuelle Adresse des Geräts |

---

## `SpiRegisterDevice`

**Datei:** `hal/RegisterDevice.h` · **Erbt von:** `IRegisterDevice`

Ein Gerät an einem `ISpiBus` mit eigenem CS-Pin. Protokoll von Bosch/InvenSense:
Lesen — die Adresse mit dem Bit `0x80`, Schreiben — mit gelöschtem Bit 7.

| Methode | Beschreibung |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` — wie viele „Müll“-Bytes der Chip nach der Adresse vor den Daten ausgibt (BMP388 — 1, ICM42688 — 0); `mode` — der SPI-Modus 0..3 |
| `begin()` | `pinMode(cs, OUTPUT)`, CS = HIGH |
| `probe()` | Immer `true` |
| `writeRegister(reg, value)` | CS↓, `reg & 0x7F`, `value`, CS↑; immer `true` |
| `readRegisters(reg, buf, n)` | CS↓, `reg \| 0x80`, `dummyReadBytes` überspringen, `n` Bytes, CS↑; immer `true` |

Jede Operation ist eine eigene Transaktion `beginTransaction(clockHz, spiMode)` …
`endTransaction()`.

---

## `Esp32Board`

**Datei:** `hal/esp32/Esp32Board.h` · **Erbt von:** `IBoard`

Die einzige Stelle, die die konkreten Peripherieobjekte des ESP32 anlegt und die
Pins aus `Config.h` kennt.

| Feld | Typ | Was es ist |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `Wire` an `PIN_I2C_SDA/SCL`, 400 kHz |
| `displayBus` | `Esp32I2CBus` | `Wire1` an `PIN_I2C2_SDA/SCL` — nur wenn `SOC_I2C_NUM > 1` |
| `spiBus` | `Esp32SpiBus` | Das globale `SPI` |
| `rcSerial`, `rcPort` | `HardwareSerial(1)`, `Esp32UartPort` | iBUS an `PIN_IBUS`, nur RX |
| `gpsSerial`, `gpsPort` | `HardwareSerial(UART_NUM_GPS)`, `Esp32UartPort` | GPS an `PIN_GPS_RX/TX` |
| `servos[7]` | `Esp32ServoOutput` | LEDC-Kanäle 0..6 in der Reihenfolge von `ServoChannel` (AUX1/AUX2 — `PIN_AUX1/2`, falls herausgeführt) |

| Methode | Beschreibung |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, dann `displayBus.begin()`, wenn es den zweiten Bus gibt; der Pin des Summers |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)`, wenn der Pin herausgeführt ist |
| `displayI2c()` | `&displayBus`, wenn `hasDisplayBus()`, sonst `nullptr` |
| `static constexpr bool hasDisplayBus()` | Beide Pins des zweiten Busses sind ≥ 0. Es existiert (wie das Feld `displayBus`) nur bei `SOC_I2C_NUM > 1` — der C3 hat nur einen I2C-Controller |
| die übrigen | Liefern die entsprechenden Felder |

---

## `Esp32I2CBus`

**Datei:** `hal/esp32/Esp32I2CBus.h` · **Erbt von:** `II2CBus`

Ein dünner Wrapper um `TwoWire` (`Wire` oder `Wire1`).

| Methode | Beschreibung |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | Merkt sich die Parameter |
| `begin()` | `wire.begin(sda, scl, hz)` und `wire.setTimeOut(TIMEOUT_MS)` — der einzige Aufruf von `wire.begin()` |
| die übrigen | Direkte Delegation an `TwoWire` |

`TIMEOUT_MS = 5`: Das Lesen von 14 Bytes der IMU bei 400 kHz dauert ~0,4 ms; eine durch
eine Störung hängende Transaktion würde die Schleife sonst für die üblichen 50 ms anhalten.

---

## `Esp32SpiBus`

**Datei:** `hal/esp32/Esp32SpiBus.h` · **Erbt von:** `ISpiBus`

Ein Wrapper um das globale `SPI`. `begin()` → `SPI.begin(sck, miso, mosi, -1)` (das CS
halten die Geräte). `beginTransaction()` baut `SPISettings(hz, MSBFIRST,
SPI_MODEn)`; `spiModeOf()` wandelt 0..3 in Arduino-Konstanten um, ein unbekannter
Wert → `SPI_MODE0`.

---

## `Esp32UartPort`

**Datei:** `hal/esp32/Esp32UartPort.h` · **Erbt von:** `IUartPort`

Ein Wrapper um `HardwareSerial`: `begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)`; `tx = -1` — nur Empfang. Der Rest ist Delegation.

---

## `Esp32ServoOutput`

**Datei:** `hal/esp32/Esp32ServoOutput.h` · **Erbt von:** `IServoOutput`

PWM direkt über LEDC (`ledcSetup/ledcAttachPin/ledcWrite` des Arduino core 2.x).
Die Bibliothek ESP32Servo wird **nicht verwendet**: Die Version 3.2.1 verwechselte auf dem S3 die
MCPWM-Blöcke (GPIO6/7 wiederholten GPIO4/5).

| Konstante | Wert |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384 (≈1,2 µs pro Schritt) |

| Methode | Beschreibung |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | Pin `< 0` — der Ausgang ist nicht herausgeführt |
| `attach(minUs, maxUs)` | Merkt sich den Bereich; Pin < 0 → `false`; sonst `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | Nicht attached — nichts; sonst `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | Schaltet den Eingangspuffer desselben GPIO ein (`PIN_INPUT_ENABLE`, der Ausgang bleibt unberührt) und misst `pulseIn(pin, HIGH, 30 ms)`; kein Impuls → `-1` |

Die Kanäle 2n und 2n+1 teilen sich einen LEDC-Timer — alle Ausgänge laufen mit 50 Hz, es gibt also keinen Konflikt.

---

# Implementierung für den STM32H743

Das Board der nächsten Generation ist der STM32H743VIT6 (Cortex-M7 mit 480 MHz, 2 MB Flash,
1 MB RAM). Die vollständige Firmware lässt sich bauen (Env `stm32h743` — das PlatformIO-Board
`weact_mini_h743vitx`, und `stm32h743-devebox` — das DevEBox H743, Konsole über
USB CDC), besteht cppcheck und die Tests auf dem PC (Env `native-stm32` mit der
Fake-Schicht von STM32duino). Auf dem DevEBox-Board **ohne Sensoren** sind geprüft: der Start, die SD-Karte, die Blackbox —
[Tests auf dem Board](../TESTING.md#tests-auf-dem-stm32-board) — sowie der iBUS-Empfang, ARM, das PWM zu
den Servos und zum Motor: Das Flugzeug wird im manuellen Modus vom Sender gesteuert (der Start wurde auf Video aufgenommen).
Die Sensoren wurden noch nicht an das Board angeschlossen.
Die Pinbelegung steht im Block `BOARD_STM32H743` in [`Config.h`](config.md#stm32h743vit6-board_stm32h743).

Die allgemeinen Unterschiede zum ESP32, die diese Schicht verbirgt:

- **Die Peripherie wählt der Kern.** STM32duino findet den Controller
  (I2C1/I2C2, SPI2, USART3, UART4, UART7, TIMx) selbst anhand der Pinnummern in den
  `PeripheralPins`-Tabellen der Variante, daher gibt es in `Config.h` keine UART-/Kanalnummern.
- **Die Pinnummern** sind die „Arduino-Pins“ der Variante (`PA0`, `PD14`...), keine GPIOs; bei
  den analogen Pins sind es `0xC0 + N`, deshalb sind die Pins im STM32-Block `int16_t`.
- **Die UART-Pins** werden beim Anlegen des Objekts `Uart(rx, tx)` festgelegt, nicht in `begin()`.

## `Stm32Board`

**Datei:** `hal/stm32/Stm32Board.h` · **Erbt von:** `IBoard`

Dasselbe wie `Esp32Board`, aufgesetzt auf STM32duino.

| Feld | Typ | Was es ist |
|---|---|---|
| `displayWire` | `TwoWire` | Der zweite I2C-Controller (das globale `Wire` ist von den Sensoren belegt). Vor `displayBus` deklariert, das eine Referenz darauf hält |
| `i2cBus` | `Stm32I2CBus` | `Wire` an `PIN_I2C_SDA/SCL` (I2C2: PB11/PB10), 400 kHz |
| `displayBus` | `Stm32I2CBus` | `displayWire` an `PIN_I2C2_SDA/SCL` (I2C1: PB9/PB8) — der zweite Bus ist immer vorhanden |
| `spiBus` | `Stm32SpiBus` | Das globale `SPI` an `PIN_SENSOR_SPI_*` (SPI2) |
| `rcSerial`, `rcPort` | `Uart`, `Stm32UartPort` | iBUS: UART7, RX `PIN_IBUS` (PE7), TX `PIN_IBUS_TX` (PE8, reserviert für iBUS-SENS) |
| `gpsSerial`, `gpsPort` | `Uart`, `Stm32UartPort` | GPS: USART3, `PIN_GPS_RX/TX` (PD9/PD8) |
| `telemetrySerial`, `telemetryPort` | `Uart`, `Stm32UartPort` | das MAVLink-Funkmodem: UART4, `PIN_TELEM_RX/TX` (PD0/PD1) |
| `servos[7]` | `Stm32ServoOutput` | In der Reihenfolge von `ServoChannel` (AUX1 — PD15/TIM4, AUX2 — PE9/TIM1) |

| Methode | Beschreibung |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, `displayBus.begin()`, der Pin des Summers |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | Immer `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | Wandelt einen Pin aus `Config.h` in den Typ der Kern-API um |
| die übrigen | Liefern die entsprechenden Felder |

## `Stm32I2CBus`

**Datei:** `hal/stm32/Stm32I2CBus.h` · **Erbt von:** `II2CBus`

| Methode | Beschreibung |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | Merkt sich die Parameter |
| `begin()` | `setSDA()`/`setSCL()` (wirken nur vor `begin()`), `wire.begin()`, `wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)`; das Ergebnis vom Typ `size_t` wird in `uint8_t` umgewandelt |
| die übrigen | Direkte Delegation an `TwoWire` |

Das Timeout der Transaktion ist bei STM32duino keine Methode, sondern das Makro
`I2C_TIMEOUT_TICK` (ms, standardmäßig 100). Im Env `stm32h743` wird es mit dem Flag
`-D I2C_TIMEOUT_TICK=5` gesetzt — aus demselben Grund wie `TIMEOUT_MS` bei `Esp32I2CBus`.

## `Stm32SpiBus`

**Datei:** `hal/stm32/Stm32SpiBus.h` · **Erbt von:** `ISpiBus`

Ein Wrapper um `SPIClass&`. `begin()` → `setSCLK/setMISO/setMOSI` + `spi.begin()`;
das Hardware-NSS wird nicht verwendet — das CS schaltet `SpiRegisterDevice`, wie beim ESP32.
`beginTransaction()` baut `SPISettings(hz, MSBFIRST, SPIMode)`; `spiModeOf()`
wandelt 0..3 in `SPI_MODEn` um, ein unbekannter Wert → `SPI_MODE0`.

## `Stm32UartPort`

**Datei:** `hal/stm32/Stm32UartPort.h` · **Erbt von:** `IUartPort`

Ein Wrapper um `HardwareSerial&` (in STM32duino 3.x die abstrakte Basis
`arduino::HardwareSerial`; das konkrete Objekt `Uart` legt `Stm32Board` an).
`begin(baud)` → `serial.begin(baud, SERIAL_8N1)`; `availableForWrite()` kommt
von `HardwareSerial`. Die Puffer (`SERIAL_RX/TX_BUFFER_SIZE` im Env): Empfang 256
Bytes (ein NAV-PVT-Frame hat 100, die üblichen 64 sind zu wenig), Senden 1024 (Protokollzeilen und
MAVLink-Frames ohne Warten).

## `Stm32ServoOutput`

**Datei:** `hal/stm32/Stm32ServoOutput.h` · **Erbt von:** `IServoOutput`

Hardware-PWM eines Timers über `HardwareTimer`, 50 Hz. Den Impuls erzeugt der Timer
ohne Interrupts und ohne die CPU — anders als die Bibliothek `Servo` für STM32,
die die Pins aus dem Interrupt eines einzigen Timers schaltet und Jitter erzeugt.

| Konstante | Wert |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4 — wie viele verschiedene Timer die Ausgänge belegen können (derzeit belegt: TIM2 und TIM4) |

| Methode | Beschreibung |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | Pin `< 0` — der Ausgang ist nicht herausgeführt |
| `attach(minUs, maxUs)` | Timer und Kanal kommen anhand des Pins aus `PinMap_TIM` (`pinmap_peripheral`, `STM_PIN_CHANNEL`), wie bei `analogWrite()`. Kein Timer am Pin oder der Vorrat ist erschöpft → `false`. Sonst `setMode(PWM1)`, Vergleich 0 (bis zum ersten Schreiben kein Impuls), `resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` → `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`. Das Vergleichsregister ist vorgeladen — der Wert gilt ab der nächsten Periode |
| `measurePulseUs()` | `pulseIn(pin, HIGH, 30 ms)` ohne den Pin umzukonfigurieren: Beim STM32 sieht das IDR-Register den Pegel auch im Modus der alternativen Funktion |
| `static acquireTimer(TIM_TypeDef*)` | Ein gemeinsamer Vorrat: **ein `HardwareTimer` je TIMx**. Ein zweites Objekt für denselben Timer würde den Handler des Kerns überschreiben (`HardwareTimer_Handle[index]`). Die Periode wird beim ersten Ausgang auf dem Timer festgelegt; `setOverflow(MICROSEC_FORMAT)` wählt den Teiler — ein Schritt von ~0,3 µs bei einem Timertakt von 240 MHz |

## `Stm32FlashStorage`

**Datei:** `hal/stm32/Stm32FlashStorage.h` · **Erbt von:** `IFlashStorage` ([storage.md](storage.md))

Der Träger des `KeyValueStore` beim STM32: der letzte Flash-Sektor (Bank 2) über die
EEPROM-Emulation von STM32duino (`eeprom_buffer_fill/flush`, ein Puffer von 8 KB, von dem
die ersten `KeyValueStore::CAPACITY` Bytes genutzt werden).

| Methode | Beschreibung |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` + byteweises Lesen des Puffers |
| `write(src, n)` | **schnell**: kopiert das Abbild unter `noInterrupts()` in den eigenen Puffer und setzt das Flag „Schreibvorgang ausstehend“. Wird aus `KvPreferences::end()` im Flug-Task aufgerufen |
| `bool service()` | **langsam**: ein Schnappschuss in den Emulationspuffer (unter `noInterrupts()`) und `eeprom_buffer_flush()` — Löschen eines Sektors von 128 KB (Sekunden) und Schreiben. Nur aus dem Hintergrund-Task `storage` |
| `hasPending()`, `flushCount()` | Diagnose |
| `static instance()`, `static store()` | der Träger und der gemeinsame `KeyValueStore` der Firmware |

Warum der Flug nicht einfriert: Der Einstellungssektor liegt in Bank 2, der Code in
Bank 1; der Flash des H7 kann eine Bank lesen, während die andere beschrieben wird;
der Flug-Task verdrängt den Hintergrund-Task.

## `compat/Preferences.h`

**Datei:** `hal/stm32/compat/Preferences.h` — im Env `stm32h743` (und
`native-stm32`) steht das Verzeichnis `compat/` in `-I` vor den Bibliotheken, und
das `#include <Preferences.h>` der Sensortreiber, des Autotrimmers und der Protokolleinstellungen
findet sie. `class Preferences : public KvPreferences` auf
`Stm32FlashStorage::store()` — dieselbe API wie das NVS des ESP32 ([storage.md](storage.md#kvpreferences)).

## `Stm32SdCard`

**Datei:** `hal/stm32/Stm32SdCard.h` · **Erbt von:** `IBlockDevice` · **Pins:** `src/stm32/sd_msp.cpp`

Eine SD-Karte an SDMMC1: 4-Bit-Bus, `HAL_SD` im Polling-Modus (ohne DMA und
Interrupts) **mit Hardware-Flusskontrolle**: Der Flug-Task verdrängt den Schreib-Task
mitten in einem Block, und ohne sie lief das FIFO über
(`HAL_SD_ERROR_RX_OVERRUN`, 0x20) — auf dem Board zeigte sich das als für Sekunden eingefrorene Konsole und
Aufzeichnung. Die Pins PC8..PC11 (D0..D3), PC12 (CK), PD2 (CMD) sind der µSD-Slot der
DevEBox und von WeAct. Der SDMMC-Kern wird von PLL1Q = 48 MHz getaktet, `ClockDiv = 1` →
**24 MHz**; wenn das erste Lesen bei 24 MHz nicht gelingt, werden 12 und 6 versucht.

| Member | Beschreibung |
|---|---|
| `bool begin()` | Den Bus hochfahren, die Karte erkennen, Probelesen. `false` — es gibt keine Karte; `initError()` — der Code |
| `read` / `write` | In Stücken von höchstens 4 KB (kurze Pausen); eine Adresse, die kein Vielfaches von 4 ist, wird über einen ausgerichteten Puffer kopiert (das HAL liest das FIFO wortweise). Bei einem Fehler — ein weiterer Versuch |
| Warten | Vor einem Zugriff nach einem Schreibvorgang wartet es, bis die Karte in den Transferzustand zurückkehrt (`Rtos::sleepMs(1)`: Die Hintergrund-Tasks verhungern nicht), bis zu 1 s. Nach einem Lesevorgang wird keine überflüssige Zustandsanfrage gesendet — der Abgleich beim Einschalten liest Zehntausende von Sektoren |
| `blockCount()`, `cardType()`, `clockDivider()`, `lastErrorCode()` | Für die Statuszeile |
| `readOps`, `writeOps`, `errors`, `retries` | Zähler |

## `ResetCause`

**Datei:** `hal/ResetCause.h` · `readResetCause()`, `isCrashReset()`, `resetCauseName()`

Der Grund für den Neustart, auf beiden Boards gleich. ESP32 — `esp_reset_reason()`;
STM32 — die Flags von `RCC->RSR` (werden einmal gelesen und gelöscht; beim H7 wird `PINRSTF`
bei jedem Reset gesetzt, deshalb werden zuerst die genaueren Ursachen
geprüft: Watchdog → Einschalten → Spannungseinbruch → Software-Reset).
Eine Panik, die Watchdogs und ein Einbruch der Versorgung zählen als „Absturz“: Bei ihnen
beginnt die Blackbox sofort mit der Aufzeichnung.

## `Rtos`

**Datei:** `hal/Rtos.h` · namespace

| Member | Beschreibung |
|---|---|
| `PRIORITY_BACKGROUND` (1), `PRIORITY_TELEMETRY` (2), `PRIORITY_FLIGHT` (5) | Prioritäten der Tasks |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | ESP32 — `xTaskCreatePinnedToCore(..., Kern 0)`, der Stack in Bytes; STM32 — `xTaskCreate`, der Stack wird in Wörter umgerechnet; `handle` ist für `xTaskNotifyGive` |
| `void sleepMs(ms)` | `vTaskDelay`; vor dem Start des Schedulers (das `setup()` des STM32) — `delay()` |
| `class CriticalSection` | `enter()`/`exit()`: ESP32 — der Spinlock `portMUX`, STM32 — `taskENTER_CRITICAL()`. Darin nur das Kopieren von Bytes (die Warteschlange der Blackbox) |
| `uint32_t freeHeapBytes()` | ESP32 — `ESP.getFreeHeap()`; STM32 — `xPortGetFreeHeapSize()` |

## Der Einstiegspunkt `src/stm32/main.cpp`

Die vollständige Firmware: dieselben Objekte wie in `src/main.cpp`, MAVLink-Telemetrie statt
WLAN, FreeRTOS-Tasks statt `loop()` — [application.md](application.md#srcstm32maincpp--stm32h743).
