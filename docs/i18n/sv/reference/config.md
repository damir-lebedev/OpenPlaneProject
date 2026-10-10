# KONFIGURATION – `Config`, `Channels`, `Controls`

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/config.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

Konfigurationslagret består bara av `constexpr`-konstanter, utan kod. Klassernas
logik får inte innehålla ”magiska” stift, tidsgränser och trösklar: allt som
kan behöva ändras för ett visst flygplan eller kort finns här.

---

## namnrymd `Config`

**Fil:** `include/config/Config.h` · **Beror på:** `<stdint.h>` ·
**Används av:** nästan alla lager.

### Stift (kortberoende)

Stiftblocket väljs av det makro som `[env:*]` sätter i `platformio.ini`
(`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`). Utan makro – `#error`. STM32-blocket beskrivs
[nedan](#stm32h743vit6-board_stm32h743).

| Konstant | Typ | Syfte | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | Skevroder | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | Höjdroder | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | Motorns fartreglage | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | Sidroder + hjul; `-1` – utgången är avstängd | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | iBUS-mottagarens RX (UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | Sensorbussen (`Wire`) | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | OLED-bussen (`Wire1`); `-1` – ingen | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | Den gemensamma SPI-bussen | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | IMU:ns CS över SPI | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | Barometerns CS över SPI | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | GPS-UART:en; TX `-1` – bara mottagning | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | Hårdvaru-UART:ens nummer för GPS:en | 2 | 0 | 2 |
| `PIN_AUX1`, `PIN_AUX2` | `int8_t` | servoutgångar: lastsläpp, klaffar; `-1` – ingen | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | summern via en transistor; `-1` – ingen | 38 | −1 | 2 |
| `PIN_AUX3`, `PIN_LIGHT`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_TX/RX` | `int8_t` | **Bara S3:** reserverade för flygkontrollerkortet ([FC_BOARD.md](../FC_BOARD.md)) | 47, 21, 8, 3, 9/10 | – | – |

Sensorernas SPI-buss heter `PIN_SENSOR_SPI_*`, inte `PIN_SPI_*`: i
STM32duino-kärnan (och andra Arduino-kärnor) är `PIN_SPI_SCK/MISO/MOSI` variantens
makron, och de skulle ersätta `Config`-konstanterna.

<a id="stm32h743"></a>

#### STM32H743VIT6 (`BOARD_STM32H743`)

Det finns ännu inget kort: stiftbeläggningen är **inte provad på hårdvara** (firmwaren körs på en dator, env `native-stm32`). Stiften är valda bland de lediga
på WeAct MiniSTM32H743VITx (kortet för PlatformIO-env `stm32h743`) och
kontrollerade mot tabellerna `PeripheralPins` i STM32duino-varianten.
Värdena är variantens makron (`PA0`…), så högst upp i `Config.h`, under
`#if defined(BOARD_STM32H743)`, inkluderas `<Arduino.h>`. Typen för alla
stift är `int16_t` (analoga stift numreras `0xC0 + N`). Det finns inga UART-
nummer – kärnan väljer kringutrustningen utifrån stiften.

| Konstant | Stift | Kringutrustning |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7 (TX – reserverad för iBUS-SENS) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2 – sensorer |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1 – skärmen (på WeAct – kamerakontakten) |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 – last / kamera |
| `PIN_BUZZER` | PE15 | GPIO – summern |
| `PIN_VBAT_ADC`, `PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11 – reserverade |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4 – MAVLink-radiomodemet (samma stift är FDCAN1) |

Konsolen `Serial` är LPUART1 (PA9 TX / PA10 RX), variantens standard.

### iBUS och förlorad förbindelse

| Konstant | Värde | Betydelse |
|---|---|---|
| `IBUS_CHANNELS` | 10 | Hur många av ramens kanaler som används |
| `IBUS_FRAME_LENGTH` | 32 | Ramens längd, byte |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | Ramens huvud |
| `IBUS_BAUDRATE` | 115200 | UART-hastighet |
| `RX_TIMEOUT_US` | 500 000 | Ingen korrekt ram på längre tid än så – förbindelsen är förlorad |
| `RX_FAILSAFE_THROTTLE_US` | 950 | Gas under detta – mottagaren rapporterar sändarens failsafe |

### GPS

| Konstant | Värde | Betydelse |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | En NAV-PVT äldre än så – `UbloxM10_Gps::isAvailable() == false` |

### PWM-område och roderutslag

| Konstant | Värde | Betydelse |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | Standard-RC-pulsen, µs |
| `AILERON_MAX_US`, `ELEVATOR_MAX_US`, `RUDDER_MAX_US` | 500 / 500 / 300 | Utslag från mitten vid fullt spakutslag, µs. Sidrodret är mindre: landningsställets hjul sitter på samma servo |
| `THROTTLE_LIMIT_PCT` | 100 | Gastaket till ESC:n, %, samma för spaken och autopiloten (`FlightController::capThrottle`). För bänktester med ett svagt 3S1P-batteri var det satt till 50; testerna räknar ut den förväntade utdatan från det här värdet |

### Klaffar (flaperoner)

| Konstant | Värde | Betydelse |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 över detta – klaffarna är utfällda (inte 1500: före den första ramen är kanalerna = 1500) |
| `FLAPS_DEPLOYED_US` | 220 | Nedåtgående utslag för varje skevroder, µs (~20° på MG90S-armen) |
| `FLAPS_TRANSITION_MS` | 1000 | Tiden för en fullständig utfällning/infällning |

### Servoriktning

`AILERON_LEFT_REVERSED`, `AILERON_RIGHT_REVERSED` (`true` – skevroderservona är spegelvända), `ELEVATOR_REVERSED` (`true`),
`RUDDER_REVERSED` – det enda stället där reversering anges. `ControlMixer`
räknar med fysiska tecken och vänder tecknet bara här, så spakarna och
autopiloten kan inte bli oense. Reversering på sändaren **får inte** användas.

### Sensormontering

| Konstant | Värde | Betydelse |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | Rotationen av IMU-kretsens axlar kring vertikalen (0/90/180/270), vart kretsens X-axel pekar. Används bara så länge det inte finns någon monteringskalibrering `o` i NVS |
| `MAG_ROTATION_CW_DEG` | 0 | Samma sak för kompassen (kompassen har ingen monteringskalibrering) |

### ARM

| Konstant | Värde | Betydelse |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 över detta – ARM-brytaren är på |
| `THROTTLE_LOW_US` | 1050 | Gas under detta – ”gasen i botten”, armering är tillåten |

### Failsafe

| Konstant | Värde | Betydelse |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | Roderytor i neutralläge |
| `FAILSAFE_THROTTLE` | 1000 | Motorn av |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | Krängningen vid glidflykt vid förlorad förbindelse i luften |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | Tippningen vid glidflykt (något under horisonten) |
| `FAILSAFE_RTH` | `true` | Med GPS och en hempunkt – förlorad förbindelse i luften ger hemflygning med motor i stället för glidflykt |

### Brytare, pitotrör, autopilot

Siffrorna för alla lägen och funktioner finns i `Config.h` bredvid utförliga
kommentarer; vad de betyder för piloten finns i [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md).

| Grupp | Konstanter |
|---|---|
| Brytare | `SWITCH_ON_US` = 1750 (kanal över detta – brytaren är på; inte 1500, så att ingenting slås på före den första ramen) |
| Pitotrör | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| Stabilisering | `MAX_BANK_DEG` 45 (ratt 15…60), `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| Navigering | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| Höjd | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| Gas och fart | `CRUISE_THROTTLE_PCT` 55 (30…85), `CRUISE_AIRSPEED_MS` 14 (10…22), `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| Överstegring | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| Cirklar och hem | `LOITER_RADIUS_M` 50 (25…150), `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| Geofence | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| Handstart | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| Landning | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| Termikflygning | `SOAR_*`: glidflykt −3°, termik > 0,5 m/s i 1,5 s, en cirkel med 25°, utgång < −0,2 m/s i 8 s, motor under 30 m upp till 100 m, hem längre bort än 400 m |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| Autotrimning | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, sparas på marken: `AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| Koordinering | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| Funktioner | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### Slinga, Wi-Fi, felsökning

| Konstant | Värde | Betydelse |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | Flygslingans period (500 Hz); också det nominella `dt` för `PidController` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | Panelens åtkomstpunkt (lösenordet är svagt – ett bänkverktyg) |
| `WEB_SERVER_PORT` | 80 | HTTP-port |
| `TELEM_BAUDRATE` | 57600 | MAVLink-radiomodemets hastighet (SiK-standard) |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | Farkostens adress i MAVLink |
| `DEBUG_INTERVAL_MS` | 100 | Hur ofta `DebugLogger` kontrollerar loggkanalerna |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | Tolerans för RC/PWM-darrning i läget ”vid ändring” |

### Svart låda

I detalj – [BLACKBOX.md](../BLACKBOX.md).

| Konstant | Värde | Betydelse |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 MB / 32 KB | Postkön i PSRAM (utan PSRAM – i internminnet) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32H743: kön i RAM – 10 s förinspelning och marginal för kortets fördröjningar |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 MB | STM32H743: filen på SD-kortet och taket för dess använda del (avstämningstiden vid start växer med området) |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | Inspelning före starten (ARM + gas) och efter DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Armerat, motorn stoppad, flygplanet orörligt så länge – stopp |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | Vad som räknas som ”orörligt” |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Efter en felomstart – inspelning i minst så länge |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | Raderat utrymme som hålls redo; gamla flygningar raderas i sin helhet på marken |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | Pausen mellan raderingar |
| `BLACKBOX_IMU_DIVIDER` | 1 | IMU:n var N:te cykel (1–500 Hz) |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | Spänningsdelarna för batteriet (56k/10k) och strömsensorn (10k/15k) på flygkontrollerkortet |

---

## namnrymd `Channels`

**Fil:** `include/config/Channels.h` · **Beror på:** `<stdint.h>`

Det enda stället där ett fysiskt kanalnummer knyts till ett syfte. Värdena
är **index** (från 0) i `RcChannelState`.

| Konstant | Index | Kanal | Reglage på FS-i6 | Syfte |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | höger spak ←→ | Roll |
| `ELEVATOR` | 1 | CH2 | höger spak ↑↓ | Tippning (2000 = bort från dig = nosen ned) |
| `THROTTLE` | 2 | CH3 | vänster spak ↑↓ | Gas |
| `RUDDER` | 3 | CH4 | vänster spak ←→ | Sidroder + hjul |
| `ARM` | 4 | CH5 | SwA | ARM-brytaren (kan inte tilldelas om) |
| `SWB` | 5 | CH6 | SwB | enligt tabellen i `Controls.h` (klaffar som standard) |
| `SWC` | 6 | CH7 | SwC (3 lägen) | läge som standard MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | RTH som standard |
| `VRA` | 8 | CH9 | VrA | `STAB_GAIN` som standard |
| `VRB` | 9 | CH10 | VrB | `CRUISE_SPEED` som standard |
| `COUNT` | 10 | | | antalet kanaler |

---

## namnrymd `Controls`

**Fil:** `include/config/Controls.h` · **Beror på:** `ControlBinding.h`, `Channels`

`constexpr Binding BINDINGS[]` – vad varje brytare och ratt gör, **en rad
per kanal** (`Bind::modes/mode/feature/knob`, se
[autopilot.md](autopilot.md#binding-bind-bindingcheck)). Bredvid finns
utkommenterade färdiga idéer. Tre `static_assert` fångar tabellfel vid
bygget: en spak eller ARM i tabellen, en kanal utanför intervallet, en upprepad
kanal, mer än en lägesväljare.
