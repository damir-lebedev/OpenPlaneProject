# TESTING.md – Tests, Abdeckung und statische Analyse

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../TESTING.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

Die Firmware wird auf zwei Ebenen geprüft:

| Wo | Befehl | Was |
|---|---|---|
| **PC (native)** | `pio test -e native` | Die Header der Firmware werden unverändert auf dem PC gebaut, die Hardware ist durch steuerbare Fakes ersetzt: Module, Treiber, geschlossene Flugsimulationen, die gesamte ESP32-Firmware (S3 und 38-Pin) mit jedem Sensorsatz. Die Abdeckung wird gezählt |
| **PC (native-stm32)** | `pio test -e native-stm32` | Die gesamte STM32H743-Firmware (`src/stm32/main.cpp`) auf einer Schicht von STM32duino-Fakes: FreeRTOS-Tasks, Flash, MAVLink, Sensoren an I2C und SPI |
| **Build-Matrix** | `tools/build_matrix.sh` | 4 Boards × 6 Sensorsätze mit `-Wall -Wextra (-Wshadow)`; jede Warnung im Code des Projekts ist ein Fehler |
| **Board** | `pio test -e esp32-s3` | `test_feedback` und `test_imu_orientation` auf einem echten ESP32-S3 (es wird eine Test-Firmware geflasht; danach die normale wieder aufspielen: `pio run -t upload`) |
| **STM32-Board** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | Die Blackbox auf einer **echten SD-Karte** im DevEBox H743, dazu `test_feedback` und `test_imu_orientation` auf einem Cortex-M7 – [weiter unten](#tests-auf-dem-stm32-board) |

Der architektonische Zusammenhang steht in [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-testbarkeit).

---

## Schnellstart

```bash
pip install platformio gcovr        # einmalig
# Windows: g++ muss im PATH sein, zum Beispiel WinLibs (winlibs.com, zip UCRT):
# entpacken und mingw64\bin zum PATH hinzufügen – keine Installation nötig
pio test -e native -e native-stm32  # alle nativen Tests (~1,5 min)
gcovr                               # Abdeckung je Datei (Einstellungen – gcovr.cfg)
tools/build_matrix.sh               # alle Boards × alle Sensoren (~25 min)
gcovr --html-details -o coverage/index.html   # HTML-Bericht (coverage/ steht in .gitignore)

pio test -e native -f native/test_rc          # ein einzelner Satz
pio test -e native -f test_feedback           # Simulation der Rückführung auf dem PC

# Bahnen der geschlossenen Simulationen als CSV (für Diagramme):
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# Der MAVLink-Datenstrom – zur Prüfung mit einem Referenz-Decoder (pip install pymavlink):
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

Bevor Sie nach Änderungen an den Tests die Abdeckung zählen, ist es sinnvoll, mit einem sauberen Build zu beginnen: `rm -rf .pio/build/native`, sonst landen die Zähler früherer Läufe im Bericht.

---

## Wie der native Build aufgebaut ist

`[env:native]` in `platformio.ini`: `platform = native`, Unity, `-std=gnu++17`, `-D BOARD_ESP32_S3` (die Pinbelegung des ESP32-S3), `-I test/native/support`, `-Wall -Wextra -Wshadow`, Abdeckung mit `--coverage` und `-fkeep-inline-functions -fkeep-static-functions` – ohne sie sieht gcov Header-Funktionen, die nie aufgerufen wurden, gar nicht und überschätzt die Abdeckung.

### Hardware-Fakes – `test/native/support/`

Header mit denselben Namen und Signaturen wie der Arduino-Core für ESP32 2.0.x, ESP-IDF und die Bibliotheken, aber über einer simulierten Welt im `namespace fake`:

| Datei | Ersetzt | Was die Simulation kann |
|---|---|---|
| `Arduino.h`, `Print.h`, `WString.h`, `Stream.h` | den Arduino-Core | Makros (`constrain`, `sq`, `DEG_TO_RAD` …), `map()`, `String`, die Formatierung von `print()` wie im Original. `ARDUINO` ist absichtlich **nicht** definiert |
| `esp32-hal-fake.h` | Zeit, GPIO, ADC, LEDC, FreeRTOS, PSRAM, `ESP` | Die Uhr rückt nur über `fake::advance*()`/`delay()` vor; `millis()/micros()` sind `uint32_t` wie beim ESP32 (der Überlauf verhält sich wie auf dem Board). LEDC-Kanäle, `pulseIn` nach dem echten Tastverhältnis (sichtbar nur, wenn der Eingangspuffer des Pins eingeschaltet ist), `analogReadMilliVolts` – die Spannung kommt aus `fake::gpio().analogMv`. Tasks werden registriert (das Handle ist ungleich null); `fake::runTask(task, n)` führt n Durchläufe ihrer Endlosschleife aus, `ulTaskNotifyTake` zählt als ein Durchlauf, `xTaskNotifyGive` als Zähler. FreeRTOS-Mutexe sind ein „belegt“-Flag. `psramFound()`/`ps_malloc()`. Kritische Abschnitte werden gezählt |
| `HardwareSerial.h` | UART | Ports werden nach Nummer registriert (`fake::uart(1)`); `pushRx()`, `txBytes()`, Geschwindigkeitswechsel im laufenden Betrieb (`updateBaudRate`, der Verlauf ist `baudChanges()`). `Serial` = UART0 |
| `esp_partition.h` | Flash-Partitionen von ESP-IDF | Eine Partition ist ein Byte-Vektor mit NOR-Verhalten: Löschen nur in 4-KB-Sektoren, gelöscht = 0xFF, ein Schreibvorgang senkt nur Bits (der Versuch, ein Bit anzuheben, wird gezählt – `bitRaises`); `beforeWrite` – „die Spannung ist weg“; Zähler für Lese-, Schreib- und Löschvorgänge |
| `esp_system.h` | die Neustartursache | `esp_reset_reason()` aus `fake::chip().resetReason` |
| `Wire.h` | I2C | Geräte nach Adresse; `fake::RegisterMapDevice` – Register mit automatischer Erhöhung, ein Schreibprotokoll, Fehler (`present`, `failWrites`, `failReads`, `failReadIf`, `shortRead`), Hooks `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | Geräte nach CS-Pin; `fake::SpiRegisterMapDevice` – das Bosch/InvenSense-Protokoll, `dummyBytes` vor den Daten |
| `Preferences.h` | NVS | Speicher im RAM, das Verhalten von `begin(readOnly)`/`get*`/`getBytes` wie im Original; `failBegin` |
| `WiFi.h`, `WebServer.h` | WLAN-Zugangspunkt, HTTP | Das Ergebnis von `softAP()` legt der Test fest; `WebServer::request(Methode, uri, Rumpf)` ruft den registrierten Handler auf; `fake::webServers()` – alle Instanzen |
| `U8g2lib.h` | U8g2 | Statt Pixeln – eine Liste der gezeichneten Zeichenketten und Rechtecke; `begin()/sendBuffer()` schicken Bytes durch einen Byte-Callback des Benutzers; `fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`, `GPIO_PIN_MUX_REG`, `PIN_INPUT_ENABLE` |

### Die STM32duino-Schicht – `test/native/support_stm32/` (Umgebung `native-stm32`)

Sie liegt in `-I` vor `support/` und ergänzt dieselben Fakes um das, was nur STM32duino hat. `<Preferences.h>` ist in dieser Umgebung das **echte** `include/hal/stm32/compat/Preferences.h` über `KeyValueStore`.

| Datei | Ersetzt | Was sie kann |
|---|---|---|
| `Arduino.h` | den STM32duino-Core | Pins `PA0..PE15` (Port·16 + Nummer), `pin_size_t`, `PinMap_TIM` für die Pins der Servoausgänge, `HardwareTimer` (der Impuls ist über `fake::timerPulseUs(pin)` und `pulseIn()` sichtbar), `Uart`, `noInterrupts()` |
| `STM32FreeRTOS.h` | das FreeRTOS von STM32duino | `xTaskCreate` in die gemeinsame Task-Registrierung (Stack in Wörtern), `vTaskStartScheduler()` kehrt zurück – die Tasks treibt der Test selbst an (`fake::runTask`), `xPortGetFreeHeapSize` |
| `EEPROM.h` | EEPROM-Emulation | ein „Flash“ von 8 KB (gelöscht = 0xFF) und ein Puffer, `fake::eeprom()` – Zähler und Beschädigung des Abbilds |
| `SPI.h` | | `SPIMode` |

In den gemeinsamen Fakes wurde für STM32 ergänzt: `TwoWire(sda, scl)`, `setSDA/SCL` und `fake::wireWithSda(pin)` (um den zweiten Bus des Boards zu finden), `HardwareSerial(rx, tx)` und `fake::uartByRx(pin)`, `SPIClass::setSCLK/MISO/MOSI`.

### Chip-Emulatoren und Flugzeugmodell – `test/native/helpers/`

| Datei | Was es ist |
|---|---|
| `ChipEmulators.h` | LSM6DSV, ICM-45686 (mit indirekten IPREG-Registern), QMC6309, SPL06-001, BMP581, NAV-PVT-Frames von u-blox – Registerkarten an I2C oder SPI, mit Daten aus der „Welt“ `World` (Winkel und Geschwindigkeiten, Höhe, Fluggeschwindigkeit, Kurs, Koordinaten), in den Achsen des Chips unter Berücksichtigung von `IMU_ROTATION_CW_DEG` |
| `PlaneSim.h` | ein Flugzeugmodell von ~1,2 kg: ein Massepunkt + Drehung in Rollen/Nicken, CL(α) mit Strömungsabriss, Widerstand, Schub, Wind, Thermik, Boden |
| `SimHarness.h` | ein geschlossener Regelkreis: Sender → iBUS-Frame → `IBusReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → PWM → Ruderausschläge → `PlaneSim` → Sensoren (einschließlich eines Pitotrohrs an zwei verrauschten Barometern). Eine CSV-Bahn mit `OPENPLANE_SIM_DIR` |

`test/native/helpers/TestSupport.h` enthält, was den Sätzen gemeinsam ist: `resetWorld()` (wird aus `setUp()` aufgerufen), die Stellvertreter `FakeUart`/`FakeServo`/`FakeBoard` und die der Sensoren (`FakeImu`, `FakeBaro`, `FakeMag`, `FakeGps`), der Frame-Baukasten `ibusFrame()`, die Prüfstände `I2cRig`/`SpiRig` (ein Treiber über den echten `Esp32I2CBus`/`Esp32SpiBus` und einem `*RegisterDevice` mit simuliertem Chip).

Die Tests für das Board (`test_feedback`, `test_imu_orientation`) sind portabel: mit `ARDUINO` – `setup()/loop()`, sonst `main()`. Die Sätze `test/native/*` werden nicht für das Board gebaut (`test_ignore` in `[esp32_common]` und `[env:stm32h743]`: Die Muster stehen je eine pro Zeile – durch ein Leerzeichen getrennt liest PlatformIO sie als eines). Auf STM32: `pio test -e stm32h743`.

---

## Testsätze

| Satz | Tests | Was er prüft |
|---|---|---|
| `native/test_hal` | 17 | Die Hilfsklassen von `II2CBus` (NACK, kurzes Lesen – der Puffer bleibt unberührt), `I2cRegisterDevice`, `SpiRegisterDevice` (Lese-Bit, Dummy-Byte des BMP388), `Esp32I2CBus` (Timeout von 5 ms), `Esp32SpiBus` (Modi 0–3), `Esp32UartPort` (8N1, Pins), `Esp32ServoOutput` (50 Hz/14 Bit, Impulsbegrenzung, LEDC-Ausfall, Messung über den Eingangspuffer), `Esp32Board` (Busse, UART, Kanalreihenfolge, AUX, Summer) |
| `native/test_rc` | 16 | `RcChannelState`, `RcInput`, iBUS-Parsing: in Teilen eintreffende Frames, CRC, 12-Bit-Werte, Failsafe des Senders, Timeout von 500 ms (auch über einen Überlauf von `micros()` hinweg), Datenmüll, Neusynchronisierung |
| `native/test_control` | 21 | Klappen (Geschwindigkeit, erster Aufruf, Pausen), der Mischer (Vorzeichen, Umkehrung, Flaperons), Gas, der ARM-Zustandsautomat und die Sensorprüfungen der Modi, die Ausgangstabelle und der Selbsttest der Impulse |
| `native/test_autopilot` | 22 | PID (D-Anteil aus der Sensorrate, Integral, Anti-Windup, `dt`), STABILIZE als Winkelmodus, zeitgesteuerter Autostart, ALT_HOLD mit dem Höhenruder, Gleitflug bei Signalverlust |
| `native/test_autopilot_modes` | 31 | Alle 12 Modi und die Reaktion jedes einzelnen auf einen fehlenden Sensor, die Zuordnungstabelle und `static_assert`, Funktionen und Drehregler, Navigation (Kurs, Kreis, Startpunkt, Geofence), Failsafe RTH/Gleitflug, Handstart, Segelflug, Auto-Trimm (Schreiben nur am Boden) |
| `native/test_flight_controller` | 12 | Ein vollständiger Zyklus des `FlightController` auf den echten Klassen: Prioritäten Signalverlust > ARM > Knüppel/Autopilot > Gas; AUX, `MOTOR_KILL`, Summer |
| `native/test_imu` | 21 | MPU6050/6500/9250 und ICM-42688: Erkennung, Register, Skalen, Achsendrehung und luftfahrttypische Vorzeichen, Busfehler, Gyroskop-Kalibrierung und Vorflugprüfung, Einbaukalibrierung aus drei Lagen, NVS, Orientierungsfilter |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`, BMP388 an I2C und SPI, BME280/BMP280 nach der Bosch-Referenz, Kompasse (Kurs, Hard-Iron-Kalibrierung im NVS), u-blox M10 (CFG-VALSET, NAV-PVT, beschädigte Frames, Timeout), `SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV (16X/32X, alternative Adresse, SPI), ICM-45686 (indirekte Register), QMC6309, SPL06-001 (Formeln aus dem Datenblatt), BMP581 (DRDY und der Ersatzweg), das Pitotrohr (Nullpunkt, Filter, Dichte, vertauschte Schläuche, veraltete Daten, ein „Flug“ mit dem Rauschen zweier Barometer) |
| `native/test_storage` | 16 | `KeyValueStore` (Neuladen, Verschleiß – ein gleicher Wert wird nicht erneut geschrieben, Überlauf ohne Datenverlust, CRC, Spannungsausfall beim Löschen, Datenmüll, Formatversion), `KvPreferences` (Verhalten wie beim NVS des ESP32) |
| `native/test_mavlink` | 20 | Der Codec gegen Referenz-Frames von pymavlink (v1, v2, signiert), CRC, Neusynchronisierung; Telemetrie: Frequenzen der Datenströme, HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS, PID-Parameter (Liste, Lesen, Schreiben, Ablehnung fehlerhafter Werte), Moduswechsel vom Boden aus, ARM vom Boden aus – abgelehnt, Missionen – 0, ein übergelaufener UART-Puffer blockiert die Schleife nicht |
| `native/test_blackbox` | 19 | Die Blackbox: Format und CRC, der Sektorring auf einem NOR-Fake (eine frische Partition ohne Löschen, Datenmüll wird immer gelöscht, alte Flüge werden komplett und nur für den Platzbedarf gelöscht, der letzte wird nie angetastet, Übergang über das Ringende, der Kopf nach einem Neustart, Spannungsausfall, ein halb geschriebener Eintrag, erkannt am CRC), Flugaufzeichnung mit dem echten `FlightController`/`Autopilot`: Start bei ARM und Gas mit Vorabaufzeichnung, Stopp nach DISARM und „steht am Boden“, Signalverlust stoppt sie nicht, Aufzeichnung nach einem fehlerhaften Neustart, manueller Start, Ereignisse, Akku, der Flash wurde in der Luft voll, ein Flug länger als die Partition, Download in Frames mit CRC und Geschwindigkeitswechsel, das Konsolenmenü `k`, keine Partition – deaktiviert |
| `native/test_blackbox_scan` | 3 | Die stichprobenartige Prüfung des Rings beim Einschalten gegen die vollständige: 300 zufällige Ringverläufe × 5 Prüfschritte (Kopf, Nummern und Flugliste stimmen überein, und wenn das Bild nicht aufgeht, gibt sie der vollständigen Prüfung nach) und die Kosten auf einem SD-Bereich von 64 MB (≈530 Lesevorgänge statt 32 000) |
| `native/test_telemetry` | 28 | `LoopStats`, `LogSettings` (NVS, Version), `DebugLogger` (alle Kanäle, NAV), `DebugConsole` (Menü, Tastenkürzel, Busabfrage `b`, bei ARM gesperrt, Speichern nur ohne ARM), `WebDebugServer` (Routen, JSON, Postfach), `OledDisplay` (Bytes über I2C, Frame, Invertierung bei Signalverlust) |
| `native/test_sim` | 15 | Geschlossene Flüge der gesamten Firmware mit dem Flugzeugmodell: Ausleiten einer Schräglage, CRUISE bei Seitenwind, LOITER, RTH, Failsafe RTH/Gleitflug, Geofence, automatischer Start von einer Bahn, Handstart, automatische Landung, eine Thermik, RESCUE aus einer Spirale, Geschwindigkeitshaltung und Schutz vor Strömungsabriss, ein echtes Pitotrohr im Regelkreis, Auto-Trimm eines „schiefen“ Flugzeugs, Sensorausfälle im Flug (IMU, Barometer, Pitotrohr, GPS) |
| `native/test_feedback_units` | 14 | Die Rückführungsmodule einzeln: Geschwindigkeitsquellen, in der Luft/am Boden, die RLS-Schätzung, der Regler, Anzeichen eines Strömungsabrisses, Abbrüche von Start und Landung |
| `native/test_app` | 10 | `src/main.cpp` auf dem ESP32-S3 mit dem Prüfstandssatz MPU6500/BMP581/QMC5883P/OLED: Periode von `loop()`, Sender → Servos, ARM, Modi, Signalverlust, Konsole, Dashboard, Display, Blackbox (ein Task auf Kern 0, Aufzeichnung bei Gas, Flug nach DISARM, `bb list`) |
| `native/test_app_lsm6dsv_pitot` | 9 | `src/main.cpp` auf dem ESP32-S3 mit dem Flugsatz: LSM6DSV + QMC6309 + SPL06 + BMP581 im Rohr + GPS – Erkennung aller Chips, Nullpunkt des Rohrs und Geschwindigkeit, Höhe, Startpunkt per GPS, STABILIZE nach den Winkeln vom Chip, RTH zum Startpunkt, Busabfrage, Dashboard |
| `native/test_app_icm45686_esp32dev` | 5 | `src/main.cpp` auf dem **ESP32 mit 38 Pins** (`BOARD_ESP32_CLASSIC`) mit dem Satz ICM-45686 + QMC6309 + SPL06 + BMP581: Pinbelegung des Boards, IPREG-Filter, Handstart, Stabilisierung und Geschwindigkeit, Abfrage eines einzelnen Busses |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | `src/stm32/main.cpp` auf dem **STM32H743** mit dem Flugsatz: Tasks und Prioritäten, Periode von 2 ms, das Rohr, PWM-Timer und `pulseIn`, MAVLink im Flug, Moduswechsel von der GCS aus, Einstellungen, die ein Hintergrund-Task in den „Flash“ schreibt, das Display an I2C1, die Konsole |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743 mit ICM-45686 und BMP581 **über SPI** + QMC6309: beschädigter Flash beim Einschalten, ALT_HOLD von der GCS hält die Höhe, Signalverlust → RTH, in MAVLink sichtbar; Überschreiben eines beschädigten Abbilds |
| `native_stm32/test_blackbox_sd` | 29 | Die Blackbox auf einer SD-Karte: FAT32 (mit und ohne MBR, ein Verzeichnis über zwei Cluster, Rauscheinträge, ein fremdes/verstreutes/leeres Volume), `SdFileRegion` (unvollständige Blöcke, Cache, Löschen, Grenzen, Fehler), der echte Treiber `Stm32SdCard` über einem gefälschten `HAL_SD` (4 Bit, Ersatzgeschwindigkeiten, Wiederholung, belegte Karte, nicht ausgerichtete Puffer), der Ring auf der Karte (Neustart, Spannungsausfall, Prüfkosten), die Markierung „Ring leer“, Flugaufzeichnung auf einem `FlightController`, ein fehlerhafter Neustart, erkannt über `RCC->RSR`, der Akku-ADC, Kartenfehler im Flug, eine langsame Karte, Download über die Konsole, die Taste `D` |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | `src/stm32/main.cpp` mit Karte: der Start findet die Karte und die Datei, der Task `bbox` schreibt den Flug, die Periode der Schleife dehnt sich nicht, `bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | `src/stm32/main.cpp` ohne Karte: die Blackbox ist abgeschaltet und erklärt, warum, das Flugzeug fliegt, das Menü `k` geht nicht kaputt |
| `test_feedback` | 10 | Eine geschlossene Simulation des Flugzeugs mit dem Rückführungskreis (auf dem PC und auf dem Board) |
| `test_imu_orientation` | 5 | Einbaukalibrierung der IMU bei 300 zufälligen Einbaulagen (auf dem PC und auf dem Board) |
| **Gesamt** | **387** | 340 in `native` + 47 in `native-stm32` (dazu 9 nur auf dem Board – `test_blackbox_sd`) |

### Tests auf dem STM32-Board

`test/test_blackbox_sd` ist nicht nativ: Der SDMMC-Treiber, die Karte und die Zeit sind echt. Die Tests laufen in einem FreeRTOS-Task, und daneben läuft ein Task, der die Flugschleife nachahmt, mit der höchsten Priorität (Periode 2 ms): Er verdrängt die Tests mitten in den Zugriffen auf die Karte, genau wie in der Firmware. Ohne ihn lässt sich der Fehler nicht fangen, der auf dem Board gefunden wurde: Bei der Verdrängung lief der FIFO des SDMMC über (`HAL_SD_ERROR_RX_OVERRUN`), in einer nackten Schleife passiert das nicht.

| Test | Was er prüft |
|---|---|
| `reset_cause_is_a_normal_one` | die Reset-Ursache (`RCC->RSR`) ist weder der Watchdog noch ein Spannungseinbruch |
| `card_is_detected_on_four_bit_bus` | die Karte wird am 4-Bit-Bus mit 24 MHz erkannt |
| `file_is_found_and_contiguous` | `BLACKBOX.BIN` wird auf FAT32 gefunden und liegt zusammenhängend |
| `multi_block_writes_work_at_every_length` | Schreiben von 1, 2, 4 und 8 Blöcken in einem einzigen Zugriff |
| `pages_write_with_bounded_latency_and_read_back_intact` | Seiten zu 256 B: das schlechteste Schreiben < 250 ms (die Grenze der SD), stabil > 40 KB/s, Lesen und Löschen |
| `header_scan_cost_on_the_whole_area` | die Kosten für das Lesen eines Sektor-Headers und der vollständigen Prüfung |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | alles löschen, zwei Flüge mit je 20 000 Einträgen, „Neustart“: die stichprobenartige Prüfung dauert < 2 s, die Einträge werden der Reihe nach mit korrektem CRC gelesen; ein leerer Ring wird an der Markierung in < 100 ms erkannt |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | die echte `BlackBox` mit einer IMU bei 500 Hz in Echtzeit: kein einziger verlorener Eintrag, der Flug lässt sich nach einem „Neustart“ lesen |
| `the_flight_task_was_not_disturbed` | das Schreiben auf die Karte hat die Periode des nachahmenden Tasks nicht gestört (Abweichung < 3 ms) |

Start (eine Karte mit der Datei – `python tools/blackbox.py sd-prepare E:`; **der Test löscht alle Flüge in der Datei**):

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

Das Board muss im DFU-Modus sein (beim DevEBox – der Draht BT0→3V3 und RST, WinUSB-Treiber über Zadig; Einzelheiten in [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743)). Die Konsole der STM32 ist USB CDC: Nach dem Flashen erscheint der Port nicht sofort, und `pio test` schafft es manchmal nicht, ihn rechtzeitig zu öffnen („could not open port“) – starten Sie dann `pio test ... --without-testing` und lesen Sie die Ausgabe mit einem beliebigen Terminalprogramm mit aktiviertem DTR (die Tests warten bis zu 60 s darauf, dass der Port geöffnet wird). Nach den Tests wartet das Board auf die Taste **`D`** – sie startet es ohne Draht in den DFU-Modus neu.

Ergebnisse auf dem DevEBox H743 + einer 16-GB-Karte (2026-10-02): `test_blackbox_sd` – 9/9, `test_feedback` – 10/10, `test_imu_orientation` – 5/5; die Zahlen zur Kartengeschwindigkeit stehen in [BLACKBOX.md](BLACKBOX.md#was-auf-dem-board-gemessen-wurde).

### Prüfstand-Firmware – `test/bench/`

Das sind keine Testsätze, sondern eigenständige kleine PlatformIO-Projekte, die anstelle der Flug-Firmware auf das Board gespielt werden (`pio test` sieht sie nicht: Die Ordnernamen beginnen nicht mit `test_`). Pins und Grenzen stammen aus der gemeinsamen `Config.h`.

| Projekt | Was es tut |
|---|---|
| `bench/elevator_sweep` | Bewegt den Höhenruder-Knüppel (CH2) programmgesteuert über `ControlMixer` und `FlightOutputs`, wie einen echten Knüppel: nach oben 100 % des Wegs, nach unten 60 %, sanft und mit Pausen; 20 s Arbeit – 20 s in Neutral. In den Endlagen misst es den Impuls an den Ausgängen. Gas auf Minimum |

Zum Aufspielen: `pio run -d test/bench/elevator_sweep -t upload`. Um die Flug-Firmware zurückzuholen: `pio run -e esp32-s3 -t upload`.

---

## Abdeckung

Sie wird von `gcovr` über `include/` und `src/` berechnet (alles, was in die Firmware eingeht), über beide nativen Umgebungen zusammen: `gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`.

| Schicht | Zeilen | Verzweigungen |
|---|---|---|
| `autopilot` | 920/943 (97,6 %) | 645/731 (88,2 %) |
| `autopilot/feedback` | 683/702 (97,3 %) | 501/570 (87,9 %) |
| `control` | 252/256 (98,4 %) | 171/189 (90,5 %) |
| `hal` | 98/102 (96,1 %) | 26/26 (100 %) |
| `hal/esp32` | 101/102 (99,0 %) | 21/22 (95,5 %) |
| `hal/stm32` | 149/158 (94,3 %) | 35/52 (67,3 %) |
| `rc` | 92/92 (100 %) | 41/42 (97,6 %) |
| `sensors` (alle) | 1444/1446 (99,9 %) | 716/835 (85,7 %) |
| `storage` | 220/220 (100 %) | 158/178 (88,8 %) |
| `telemetry` | 1413/1440 (98,1 %) | 1123/1269 (88,5 %) |
| `src` (`main.cpp`, `stm32/main.cpp`) | 118/123 (95,9 %) | 20/29 (69,0 %) |
| **Gesamt** | **5490/5584 (98,3 %)** | **3457/3943 (87,7 %)**; Funktionen 877/902 (97,2 %) |

Was nicht abgedeckt bleibt und warum:

- **Handstart** (`TakeoffSequencer`: `WaitLaunch`, `launchDetected()`) – bei `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false` unerreichbar; er taucht in den Tests auf, sobald die Konstante konfigurierbar wird (Umzug nach `Config.h`).
- **Vom Board abhängiger Code:** ein Ausgang ohne Pin (`PIN_RUDDER = -1` gibt es nur beim C3), ein GPS ohne TX-Pin (C3) – die nativen Tests spielen die Pinbelegung von S3, 38-Pin und STM32 durch, aber nicht die des C3 (der C3 wird durch die Build-Matrix geprüft).
- **STM32:** die Fehlerzweige des Kerns (kein Timer am Pin, der Timer-Pool ist erschöpft), die Meldung `FreeRTOS не запустился` („FreeRTOS wurde nicht gestartet“) – auf dem PC kehrt `vTaskStartScheduler()` immer zurück.
- **Schutzzweige**, die sich über die öffentliche API nicht erreichen lassen: `default`/`Count` in einem `switch` über Aufzählungen, `return "?"`.
- Dateien ohne ausführbare Zeilen (`Config.h`, `Channels.h`, `FeedbackConfig.h`, die Strukturen `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets`, die Makros von `SensorSelection.h`, das HTML des Dashboards) tauchen im Bericht nicht auf – sie werden in die Tests einkompiliert, aber gcov hat dort nichts zu zählen.

---

## Statische Analyse

| Werkzeug | Befehl | Profil |
|---|---|---|
| GCC | `tools/build_matrix.sh` (oder `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`) | Alle Boards × alle Sensorsätze. Der native Testbuild nutzt immer `-Wall -Wextra -Wshadow`; `stm32h743` nutzt `-Wall -Wextra` (`build_src_flags`; `-Wshadow` macht auf den Headern von STM32duino selbst Lärm) |
| cppcheck | `pio check -e esp32-s3`; `pio check -e stm32h743` | `check_*` in `[esp32_common]`: `include/` und `src/` (außer `stm32/`), warning/style/performance/portability, eingebettete Unterdrückungen `// cppcheck-suppress` nur für Fehlalarme (der Callback von U8g2, `setup/loop`). Für `stm32h743` – dieselben Flags über `include/hal/stm32/` und `src/stm32/` |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`: bugprone, clang-analyzer, performance, `misc-include-cleaner` und weitere; deaktivierte Prüfungen sind in der Datei selbst erklärt |

clang-tidy wird mit den Fakes aus `test/native/support` gestartet: Die ESP-IDF-Header kann clang für die Host-Architektur nicht parsen (versucht man `pio check` mit `clangtidy`, bricht die Analyse bei Parserfehlern ab und prüft ehrlich gesagt nichts). `misc-include-cleaner` achtet darauf, dass jeder Header einbindet, was er benutzt: Die „Schirm“-Header (`FeedbackModules.h`, die API von `IBoard.h`/`RegisterDevice.h`, die Makros von `SensorSelection.h`) sind mit `// IWYU pragma: export` markiert. Der Code für STM32 (`include/hal/stm32/`, `src/stm32/`) wird vom Skript übersprungen – ihn prüfen der Build, cppcheck der Umgebung `stm32h743` und die Tests der Umgebung `native-stm32`.

Die Build-Matrix beim letzten Durchlauf – **24/24 ohne Warnungen**:

| Board | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck (`esp32-s3`, `stm32h743`) – 0 Beanstandungen am Code des Projekts.

---

## Wie man neue Tests schreibt

1. Ein Modul mit Logik ohne Hardware – ein direkter Unit-Test: Die Zeit wird als Parameter übergeben oder mit `fake::advanceMs()` vorgerückt.
2. Ein Chip-Treiber – über `I2cRig`/`SpiRig`: die Register eines simulierten Chips, Prüfung der geschriebenen Werte (`chip.lastWrite(reg)`) und der Datenauswertung. Für Formeln dient eine Referenz aus dem Datenblatt oder eine unabhängige Rechnung, nicht eine Kopie des Codes.
3. Klassen mit endlosen FreeRTOS-Tasks – `fake::findTask("Name")` + `fake::runTask(task, n)`; so wird auch der Flug-Task der STM32 angetrieben.
4. Die gesamte Firmware mit einem anderen Sensorsatz oder einem anderen Board – ein eigener Satz, der vor `#include "../../../src/main.cpp"` `SENSOR_KIT` festlegt (oder `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`); die Chips stammen aus `helpers/ChipEmulators.h`. Für STM32 – `test/native_stm32/`.
5. Ein neuer Autopilot-Modus – ein Szenario eines geschlossenen Flugs in `test_sim`.
6. Ein neuer Satz – ein Ordner `test/native/test_<Name>/test_main.cpp` mit `main()`; `setUp()` ruft `resetWorld()` auf, wenn der Satz keinen Zustand zwischen den Tests braucht.
7. Einen Fehler gefunden – zuerst ein Test, der ihn fängt, dann die Korrektur.
