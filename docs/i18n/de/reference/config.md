# CONFIG — `Config`, `Channels`, `Controls`

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/config.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert.

[← Referenz](README.md)

Die Konfigurationsschicht besteht nur aus `constexpr`-Konstanten, ohne Code.
Die Logik der Klassen darf keine „magischen“ Pins, Timeouts und Schwellwerte
enthalten: alles, was sich für ein bestimmtes Flugzeug oder Board ändern
lassen muss, steht hier.

---

## namespace `Config`

**Datei:** `include/config/Config.h` · **Abhängig von:** `<stdint.h>` ·
**Verwendet von:** fast allen Schichten.

### Pins (abhängig vom Board)

Der Pin-Block wird durch das Makro gewählt, das `[env:*]` in `platformio.ini`
setzt (`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`). Ohne Makro — `#error`. Der STM32-Block ist
[weiter unten](#stm32h743vit6-board_stm32h743) beschrieben.

| Konstante | Typ | Zweck | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | Querruder | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | Höhenruder | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | Motorsteller | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | Seitenruder + Rad; `-1` — der Ausgang ist abgeschaltet | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | RX des iBUS-Empfängers (UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | Der Sensorbus (`Wire`) | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | Der OLED-Bus (`Wire1`); `-1` — keiner | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | Der gemeinsame SPI-Bus | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | CS der IMU über SPI | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | CS des Barometers über SPI | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | Die UART des GPS; TX `-1` — nur Empfang | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | Die Nummer der Hardware-UART für das GPS | 2 | 0 | 2 |
| `PIN_AUX1`, `PIN_AUX2` | `int8_t` | Servo-Ausgänge: Nutzlast abwerfen, Klappen; `-1` — keiner | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | der Summer über einen Transistor; `-1` — keiner | 38 | −1 | 2 |
| `PIN_AUX3`, `PIN_LIGHT`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_TX/RX` | `int8_t` | **Nur S3:** reserviert für die Flugcontroller-Platine ([FC_BOARD.md](../FC_BOARD.md)) | 47, 21, 8, 3, 9/10 | — | — |

Der Sensor-SPI-Bus heißt `PIN_SENSOR_SPI_*` und nicht `PIN_SPI_*`: im
STM32duino-Core (und in anderen Arduino-Cores) sind `PIN_SPI_SCK/MISO/MOSI`
Makros der Variante, die die Konstanten von `Config` ersetzen würden.

<a id="stm32h743"></a>

#### STM32H743VIT6 (`BOARD_STM32H743`)

Eine Platine gibt es noch nicht: die Pinbelegung ist **nicht an der Hardware getestet** (die Firmware läuft am PC, Env `native-stm32`). Die Pins sind aus den freien
des WeAct MiniSTM32H743VITx (die Platine des PlatformIO-Env `stm32h743`)
gewählt und mit den `PeripheralPins`-Tabellen der STM32duino-Variante
abgeglichen. Die Werte sind Makros der Variante (`PA0`…), deshalb wird am
Anfang von `Config.h` unter `#if defined(BOARD_STM32H743)` `<Arduino.h>`
eingebunden. Der Typ aller Pins ist `int16_t` (analoge Pins haben die Nummer
`0xC0 + N`). UART-Nummern gibt es nicht — die Peripherie wählt der Core anhand
der Pins.

| Konstante | Pin | Peripherie |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7 (TX — reserviert für iBUS-SENS) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2 — Sensoren |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1 — das Display (auf dem WeAct — der Kameraanschluss) |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 — Nutzlast / Kamera |
| `PIN_BUZZER` | PE15 | GPIO — der Summer |
| `PIN_VBAT_ADC`, `PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11 — reserviert |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4 — das MAVLink-Funkmodem (dieselben Pins sind FDCAN1) |

Die Konsole `Serial` ist LPUART1 (PA9 TX / PA10 RX), der Standardwert der
Variante.

### iBUS und Verbindungsverlust

| Konstante | Wert | Bedeutung |
|---|---|---|
| `IBUS_CHANNELS` | 10 | Wie viele Kanäle des Frames verwendet werden |
| `IBUS_FRAME_LENGTH` | 32 | Länge des Frames, Bytes |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | Der Kopf des Frames |
| `IBUS_BAUDRATE` | 115200 | UART-Geschwindigkeit |
| `RX_TIMEOUT_US` | 500 000 | Länger als das kein korrekter Frame — die Verbindung ist verloren |
| `RX_FAILSAFE_THROTTLE_US` | 950 | Gas darunter — der Empfänger meldet das Failsafe des Senders |

### GPS

| Konstante | Wert | Bedeutung |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | Ein NAV-PVT, das älter ist — `UbloxM10_Gps::isAvailable() == false` |

### PWM-Bereich und Ruderwege

| Konstante | Wert | Bedeutung |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | Der Standard-RC-Impuls, µs |
| `AILERON_MAX_US`, `ELEVATOR_MAX_US`, `RUDDER_MAX_US` | 500 / 500 / 300 | Ausschlag aus der Mitte bei vollem Knüppelweg, µs. Das Seitenruder ist kleiner: am selben Servo hängt das Fahrwerksrad |
| `THROTTLE_LIMIT_PCT` | 100 | Die Gasobergrenze zum ESC, %, gleich für den Knüppel und den Autopiloten (`FlightController::capThrottle`). Für Prüfstandtests mit einem schwachen 3S1P-Akku wurde 50 gesetzt; die Tests berechnen die erwartete Ausgabe aus diesem Wert |

### Klappen (Flaperons)

| Konstante | Wert | Bedeutung |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 darüber — die Klappen sind ausgefahren (nicht 1500: bis zum ersten Frame sind die Kanäle = 1500) |
| `FLAPS_DEPLOYED_US` | 220 | Ausschlag jedes Querruders nach unten, µs (~20° des MG90S-Hebels) |
| `FLAPS_TRANSITION_MS` | 1000 | Die Zeit für das vollständige Aus-/Einfahren |

### Servo-Richtung

`AILERON_LEFT_REVERSED`, `AILERON_RIGHT_REVERSED` (`true` — die Querruderservos sind spiegelbildlich eingebaut), `ELEVATOR_REVERSED` (`true`),
`RUDDER_REVERSED` — die einzige Stelle, an der die Umkehr festgelegt wird.
`ControlMixer` rechnet in physikalischen Vorzeichen und kehrt das Vorzeichen
nur hier um, deshalb können Knüppel und Autopilot nicht auseinanderlaufen. Die
Umkehr am Sender **darf nicht** verwendet werden.

### Einbau der Sensoren

| Konstante | Wert | Bedeutung |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | Die Drehung der Achsen des IMU-Chips um die Hochachse (0/90/180/270), wohin die X-Achse des Chips zeigt. Wird nur verwendet, solange keine Einbaukalibrierung `o` im NVS liegt |
| `MAG_ROTATION_CW_DEG` | 0 | Dasselbe für den Kompass (der Kompass hat keine Einbaukalibrierung) |

### ARM

| Konstante | Wert | Bedeutung |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 darüber — der ARM-Schalter ist eingeschaltet |
| `THROTTLE_LOW_US` | 1050 | Gas darunter — „Gas unten“, Armen ist möglich |

### Failsafe

| Konstante | Wert | Bedeutung |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | Neutralstellung der Ruder |
| `FAILSAFE_THROTTLE` | 1000 | Motor aus |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | Die Schräglage des Gleitens bei Verbindungsverlust in der Luft |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | Das Nicken des Gleitens (etwas unter dem Horizont) |
| `FAILSAFE_RTH` | `true` | Mit GPS und Startpunkt bedeutet ein Verbindungsverlust in der Luft — Rückkehr zum Startpunkt mit Motor und nicht Gleiten |

### Schalter, Pitotrohr, Autopilot

Die Zahlen aller Modi und Funktionen stehen in `Config.h` neben ausführlichen
Kommentaren; was sie für den Piloten bedeuten, steht im
[AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md).

| Gruppe | Konstanten |
|---|---|
| Schalter | `SWITCH_ON_US` = 1750 (Kanal darüber — Schalter eingeschaltet; nicht 1500, damit vor dem ersten Frame nichts eingeschaltet wird) |
| Pitotrohr | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| Stabilisierung | `MAX_BANK_DEG` 45 (Drehregler 15…60), `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| Navigation | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| Höhe | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| Gas und Geschwindigkeit | `CRUISE_THROTTLE_PCT` 55 (30…85), `CRUISE_AIRSPEED_MS` 14 (10…22), `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| Strömungsabriss | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| Kreise und Startpunkt | `LOITER_RADIUS_M` 50 (25…150), `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| Geofence | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| Handstart | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| Landung | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| Segelflug | `SOAR_*`: Gleiten −3°, eine Thermik > 0.5 m/s über 1.5 s, ein Kreis von 25°, Ausstieg < −0.2 m/s über 8 s, Motor unter 30 m bis 100 m, Rückkehr zum Startpunkt jenseits von 400 m |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| Autotrimmung | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, Speichern am Boden: `AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| Koordination | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| Funktionen | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### Zyklus, WLAN, Debugging

| Konstante | Wert | Bedeutung |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | Die Periode der Flugschleife (500 Hz); auch das nominelle `dt` für `PidController` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | Der Access Point des Dashboards (das Passwort ist schwach — ein Prüfstandwerkzeug) |
| `WEB_SERVER_PORT` | 80 | HTTP-Port |
| `TELEM_BAUDRATE` | 57600 | Die Geschwindigkeit des MAVLink-Funkmodems (der Standard von SiK) |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | Die Adresse des Fluggeräts in MAVLink |
| `DEBUG_INTERVAL_MS` | 100 | Wie oft `DebugLogger` die Log-Kanäle prüft |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | Toleranz gegenüber RC/PWM-Zittern im Modus „bei Änderung“ |

### Blackbox

Ausführlich — [BLACKBOX.md](../BLACKBOX.md).

| Konstante | Wert | Bedeutung |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 MB / 32 KB | Die Warteschlange der Einträge im PSRAM (ohne PSRAM — im internen Speicher) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32H743: die Warteschlange im RAM — 10 s Vorlauf und Reserve für Verzögerungen der Karte |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 MB | STM32H743: die Datei auf der SD-Karte und die Obergrenze ihres genutzten Teils (die Abgleichszeit beim Einschalten wächst mit dem Bereich) |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | Aufzeichnung vor dem Start (ARM + Gas) und nach dem DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Gearmt, der Motor steht, das Flugzeug so lange regungslos — Stopp |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | Was als „regungslos“ gilt |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Nach einem fehlerhaften Neustart — mindestens so lange aufzeichnen |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | Gelöschter Platz, der bereitgehalten wird; alte Flüge werden am Boden komplett gelöscht |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | Die Pause zwischen den Löschvorgängen |
| `BLACKBOX_IMU_DIVIDER` | 1 | Die IMU in jedem N-ten Zyklus (1 — 500 Hz) |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | Die Teiler der Batterie (56k/10k) und des Stromsensors (10k/15k) auf der Flugcontroller-Platine |

---

## namespace `Channels`

**Datei:** `include/config/Channels.h` · **Abhängig von:** `<stdint.h>`

Die einzige Stelle, an der die physische Kanalnummer mit dem Zweck verknüpft
wird. Die Werte sind **Indizes** (ab 0) in `RcChannelState`.

| Konstante | Index | Kanal | Bedienelement der FS-i6 | Zweck |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | rechter Knüppel ←→ | Rollen |
| `ELEVATOR` | 1 | CH2 | rechter Knüppel ↑↓ | Nicken (2000 = von sich weg = Nase runter) |
| `THROTTLE` | 2 | CH3 | linker Knüppel ↑↓ | Gas |
| `RUDDER` | 3 | CH4 | linker Knüppel ←→ | Seitenruder + Rad |
| `ARM` | 4 | CH5 | SwA | Der ARM-Schalter (kann nicht umbelegt werden) |
| `SWB` | 5 | CH6 | SwB | laut Tabelle in `Controls.h` (standardmäßig die Klappen) |
| `SWC` | 6 | CH7 | SwC (3 Stellungen) | standardmäßig der Modus MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | standardmäßig RTH |
| `VRA` | 8 | CH9 | VrA | standardmäßig `STAB_GAIN` |
| `VRB` | 9 | CH10 | VrB | standardmäßig `CRUISE_SPEED` |
| `COUNT` | 10 | | | die Anzahl der Kanäle |

---

## namespace `Controls`

**Datei:** `include/config/Controls.h` · **Abhängig von:** `ControlBinding.h`, `Channels`

`constexpr Binding BINDINGS[]` — was jeder Schalter und jeder Drehregler tut,
**eine Zeile pro Kanal** (`Bind::modes/mode/feature/knob`, siehe
[autopilot.md](autopilot.md#binding-bind-bindingcheck)). Daneben stehen
auskommentierte fertige Ideen. Drei `static_assert` fangen Fehler der Tabelle
beim Bauen ab: ein Knüppel oder ARM in der Tabelle, ein Kanal außerhalb des
Bereichs, ein wiederholter Kanal, mehr als ein Schalter zur Modusauswahl.
