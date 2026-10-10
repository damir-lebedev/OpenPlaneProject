# TESTING.md – tester, täckning och statisk analys

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../TESTING.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Firmwaren verifieras på två nivåer:

| Var | Kommando | Vad |
|---|---|---|
| **Dator (native)** | `pio test -e native` | Firmwarens headerfiler byggs på datorn oförändrade, med hårdvaran ersatt av styrbara attrapper: moduler, drivrutiner, flygsimuleringar i sluten slinga, hela ESP32-firmwaren (S3 och 38-pin) med varje sensorsats. Täckningen räknas |
| **Dator (native-stm32)** | `pio test -e native-stm32` | Hela STM32H743-firmwaren (`src/stm32/main.cpp`) ovanpå ett lager av STM32duino-attrapper: FreeRTOS-uppgifter, flash, MAVLink, sensorer på I2C och SPI |
| **Byggmatris** | `tools/build_matrix.sh` | 4 kort × 6 sensorsatser med `-Wall -Wextra (-Wshadow)`; varje varning i projektets kod är ett fel |
| **Kort** | `pio test -e esp32-s3` | `test_feedback` och `test_imu_orientation` på en riktig ESP32-S3 (den flashar en testfirmware; lägg tillbaka den vanliga efteråt: `pio run -t upload`) |
| **STM32-kort** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | Den svarta lådan på ett **riktigt SD-kort** i DevEBox H743, plus `test_feedback` och `test_imu_orientation` på en Cortex-M7 – [nedan](#tester-på-stm32-kortet) |

Den arkitektoniska kontexten finns i [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-testbarhet).

---

## Snabbstart

```bash
pip install platformio gcovr        # en gång
# Windows: du behöver g++ i PATH, till exempel WinLibs (winlibs.com, zip UCRT):
# packa upp och lägg till mingw64\bin i PATH – ingen installation behövs
pio test -e native -e native-stm32  # alla native-tester (~1,5 min)
gcovr                               # täckning per fil (inställningar – gcovr.cfg)
tools/build_matrix.sh               # alla kort × alla sensorer (~25 min)
gcovr --html-details -o coverage/index.html   # HTML-rapport (coverage/ finns i .gitignore)

pio test -e native -f native/test_rc          # en svit
pio test -e native -f test_feedback           # återkopplingssimulering på datorn

# Banorna från simuleringarna i sluten slinga som CSV (för diagram):
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# MAVLink-strömmen – för kontroll med en referensavkodare (pip install pymavlink):
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

Innan du räknar täckning efter ändringar i testerna hjälper det att börja från ett rent bygge: `rm -rf .pio/build/native`, annars hamnar räknarna från tidigare körningar i rapporten.

---

## Hur native-bygget fungerar

`[env:native]` i `platformio.ini`: `platform = native`, Unity, `-std=gnu++17`, `-D BOARD_ESP32_S3` (ESP32-S3:s stiftbeläggning), `-I test/native/support`, `-Wall -Wextra -Wshadow`, täckning med `--coverage` och `-fkeep-inline-functions -fkeep-static-functions` – utan dem ser gcov inte headerfunktioner som aldrig anropades och överskattar täckningen.

### Hårdvaruattrapper – `test/native/support/`

Headerfiler med samma namn och signaturer som Arduino-kärnan för ESP32 2.0.x, ESP-IDF och biblioteken, men ovanpå en simulerad värld i `namespace fake`:

| Fil | Ersätter | Vad simuleringen kan |
|---|---|---|
| `Arduino.h`, `Print.h`, `WString.h`, `Stream.h` | Arduino-kärnan | Makron (`constrain`, `sq`, `DEG_TO_RAD`…), `map()`, `String`, formatering med `print()` som i originalet. `ARDUINO` är avsiktligt **inte** definierad |
| `esp32-hal-fake.h` | tid, GPIO, ADC, LEDC, FreeRTOS, PSRAM, `ESP` | Klockan går bara framåt via `fake::advance*()`/`delay()`; `millis()/micros()` är `uint32_t`, som på ESP32 (överslag beter sig som på kortet). LEDC-kanaler, `pulseIn` efter den verkliga pulskvoten (syns bara om stiftets ingångsbuffert är påslagen), `analogReadMilliVolts` – spänningen från `fake::gpio().analogMv`. Uppgifter registreras (handtaget är inte null); `fake::runTask(task, n)` kör n varv av dess ändlösa slinga, `ulTaskNotifyTake` räknas som ett varv, `xTaskNotifyGive` som en räknare. FreeRTOS-mutexar är en ”upptagen”-flagga. `psramFound()`/`ps_malloc()`. Kritiska sektioner räknas |
| `HardwareSerial.h` | UART | Portar registreras efter nummer (`fake::uart(1)`); `pushRx()`, `txBytes()`, byte av hastighet i farten (`updateBaudRate`, historiken är `baudChanges()`). `Serial` = UART0 |
| `esp_partition.h` | ESP-IDF:s flashpartitioner | En partition är en vektor av byte med NOR-beteende: radering bara i sektorer om 4 KB, raderat = 0xFF, en skrivning sänker bara bitar (ett försök att höja en bit räknas – `bitRaises`); `beforeWrite` – ”strömmen gick”; räknare för läsningar, skrivningar och raderingar |
| `esp_system.h` | återställningsorsaken | `esp_reset_reason()` från `fake::chip().resetReason` |
| `Wire.h` | I2C | Enheter efter adress; `fake::RegisterMapDevice` – register med autoinkrement, en skrivlogg, fel (`present`, `failWrites`, `failReads`, `failReadIf`, `shortRead`), krokar `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | Enheter efter CS-stift; `fake::SpiRegisterMapDevice` – Bosch-/InvenSense-protokollet, `dummyBytes` före data |
| `Preferences.h` | NVS | Lagring i minnet, beteendet hos `begin(readOnly)`/`get*`/`getBytes` som i originalet; `failBegin` |
| `WiFi.h`, `WebServer.h` | Wi-Fi AP, HTTP | Resultatet av `softAP()` bestäms av testet; `WebServer::request(method, uri, body)` anropar den registrerade hanteraren; `fake::webServers()` – alla instanser |
| `U8g2lib.h` | U8g2 | I stället för pixlar – en lista över de strängar och rektanglar som ritats; `begin()/sendBuffer()` skickar byte genom en användarens byte-callback; `fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`, `GPIO_PIN_MUX_REG`, `PIN_INPUT_ENABLE` |

### STM32duino-lagret – `test/native/support_stm32/` (env `native-stm32`)

Det ligger i `-I` före `support/` och kompletterar samma attrapper med det som bara STM32duino har. `<Preferences.h>` i den här miljön är den **riktiga** `include/hal/stm32/compat/Preferences.h` ovanpå `KeyValueStore`.

| Fil | Ersätter | Vad den kan |
|---|---|---|
| `Arduino.h` | STM32duino-kärnan | stiften `PA0..PE15` (port·16 + nummer), `pin_size_t`, `PinMap_TIM` för servoutgångarnas stift, `HardwareTimer` (pulsen syns via `fake::timerPulseUs(pin)` och `pulseIn()`), `Uart`, `noInterrupts()` |
| `STM32FreeRTOS.h` | STM32duino FreeRTOS | `xTaskCreate` in i det gemensamma uppgiftsregistret (stacken i ord), `vTaskStartScheduler()` returnerar – testet kör själv uppgifterna (`fake::runTask`), `xPortGetFreeHeapSize` |
| `EEPROM.h` | EEPROM-emulering | en ”flash” på 8 KB (raderat = 0xFF) och en buffert, `fake::eeprom()` – räknare och korruption av avbilden |
| `SPI.h` | | `SPIMode` |

Tillagt i de gemensamma attrapperna för STM32: `TwoWire(sda, scl)`, `setSDA/SCL` och `fake::wireWithSda(pin)` (för att hitta kortets andra buss), `HardwareSerial(rx, tx)` och `fake::uartByRx(pin)`, `SPIClass::setSCLK/MISO/MOSI`.

### Kretsemulatorer och flygplansmodellen – `test/native/helpers/`

| Fil | Vad det är |
|---|---|
| `ChipEmulators.h` | LSM6DSV, ICM-45686 (med IPREG-indirekta register), QMC6309, SPL06-001, BMP581, u-blox NAV-PVT-ramar – registerkartor på I2C eller SPI, med data från ”världen” `World` (vinklar och hastigheter, höjd, lufthastighet, kurs, koordinater), i kretsens axlar med hänsyn till `IMU_ROTATION_CW_DEG` |
| `PlaneSim.h` | en flygplansmodell på ~1,2 kg: en punktmassa + rotation i roll/tippning, CL(α) med överstegring, luftmotstånd, dragkraft, vind, termik, mark |
| `SimHarness.h` | en sluten slinga: sändare → iBUS-ram → `IBusReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → PWM → roderutslag → `PlaneSim` → sensorer (inklusive ett pitotrör med två brusiga barometrar). En CSV-bana med `OPENPLANE_SIM_DIR` |

`test/native/helpers/TestSupport.h` är det som sviterna delar: `resetWorld()` (anropas från `setUp()`), ersättarna `FakeUart`/`FakeServo`/`FakeBoard` och för sensorerna (`FakeImu`, `FakeBaro`, `FakeMag`, `FakeGps`), rambyggaren `ibusFrame()`, riggarna `I2cRig`/`SpiRig` (en drivrutin ovanpå de riktiga `Esp32I2CBus`/`Esp32SpiBus` och `*RegisterDevice` med en simulerad krets).

Korttesterna (`test_feedback`, `test_imu_orientation`) är portabla: med `ARDUINO` – `setup()/loop()`, annars `main()`. Sviterna `test/native/*` byggs inte för kortet (`test_ignore` i `[esp32_common]` och `[env:stm32h743]`: mönstren skrivs ett per rad – åtskilda med mellanslag läser PlatformIO dem som ett). På STM32: `pio test -e stm32h743`.

---

## Testsviter

| Svit | Tester | Vad den kontrollerar |
|---|---|---|
| `native/test_hal` | 17 | `II2CBus`-hjälpare (NACK, kort läsning – bufferten lämnas orörd), `I2cRegisterDevice`, `SpiRegisterDevice` (läsbit, BMP388:s dummybyte), `Esp32I2CBus` (tidsgräns 5 ms), `Esp32SpiBus` (lägen 0–3), `Esp32UartPort` (8N1, stift), `Esp32ServoOutput` (50 Hz/14 bitar, pulsbegränsning, LEDC-fel, mätning via ingångsbufferten), `Esp32Board` (bussar, UART, kanalordning, AUX, summer) |
| `native/test_rc` | 16 | `RcChannelState`, `RcInput`, iBUS-tolkning: ramar som kommer i bitar, CRC, 12-bitarsvärden, sändarens failsafe, tidsgräns 500 ms (även över ett överslag i `micros()`), skräp, omsynkronisering |
| `native/test_control` | 21 | Klaffar (hastighet, första anropet, pauser), mixern (tecken, reversering, flaperoner), gas, ARM-tillståndsmaskinen och lägets sensorkontroller, utgångstabellen och pulsens självtest |
| `native/test_autopilot` | 22 | PID (D-term från sensorns hastighet, integral, anti-windup, `dt`), STABILIZE som vinkelläge, tidsbaserad autostart, ALT_HOLD med höjdrodret, glidflykt vid förlorad förbindelse |
| `native/test_autopilot_modes` | 31 | Alla 12 lägen och hur vart och ett reagerar på en saknad sensor, bindningstabellen och `static_assert`, funktioner och rattar, navigering (kurs, cirkel, hem, geofence), failsafe RTH/glidflykt, handstart, termikflygning, autotrimning (skrivs bara på marken) |
| `native/test_flight_controller` | 12 | En fullständig `FlightController`-cykel på de riktiga klasserna: prioriteterna förlorad förbindelse > ARM > spakar/autopilot > gas; AUX, `MOTOR_KILL`, summer |
| `native/test_imu` | 21 | MPU6050/6500/9250 och ICM-42688: identifiering, register, skalor, axelrotation och flygtekniska teckenkonventioner, bussfel, gyroskopkalibrering och kontrollen före flygning, monteringskalibrering från tre positioner, NVS, orienteringsfiltret |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`, BMP388 över I2C och SPI, BME280/BMP280 mot Bosch-referensen, kompasser (kurs, hard-iron-kalibrering i NVS), u-blox M10 (CFG-VALSET, NAV-PVT, korrupta ramar, tidsgräns), `SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV (16X/32X, alternativ adress, SPI), ICM-45686 (indirekta register), QMC6309, SPL06-001 (formlerna i databladet), BMP581 (DRDY och reservvägen), pitotröret (nollpunkt, filter, densitet, förväxlade slangar, inaktuella data, en ”flygning” med bruset från två barometrar) |
| `native/test_storage` | 16 | `KeyValueStore` (omladdning, slitage – ett identiskt värde skrivs inte om, överfyllnad utan dataförlust, CRC, strömavbrott under radering, skräp, formatversion), `KvPreferences` (beter sig som ESP32:s NVS) |
| `native/test_mavlink` | 20 | Kodeken mot referensramar från pymavlink (v1, v2, signerade), CRC, omsynkronisering; telemetri: strömfrekvenser, HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS, PID-parametrar (lista, läsning, skrivning, avvisning av dåliga värden), lägesbyte från marken, ARM från marken – nekas, uppdrag – 0, en överfull UART-buffert blockerar inte slingan |
| `native/test_blackbox` | 19 | Den svarta lådan: format och CRC, sektorringen på en NOR-attrapp (en ny partition utan radering, skräp raderas alltid, gamla flygningar raderas i sin helhet och bara för att frigöra utrymme, den senaste rörs aldrig, omslag vid ringens slut, huvudet efter en omstart, strömavbrott, en halvskriven post fångas av CRC), inspelning av en flygning på riktiga `FlightController`/`Autopilot`: start vid ARM och gas med förinspelning, stopp efter DISARM och ”står på marken”, förlorad förbindelse stoppar den inte, inspelning efter en felomstart, manuell start, händelser, batteri, flash tog slut i luften, en flygning längre än partitionen, hämtning i CRC-ramar med hastighetsbyte, konsolmenyn `k`, ingen partition – avstängd |
| `native/test_blackbox_scan` | 3 | En stickprovskontroll av ringen vid start mot den fullständiga: 300 slumpmässiga ringhistoriker × 5 provsteg (huvudet, numren och flyglistan stämmer, och när bilden inte går ihop – faller den tillbaka på den fullständiga avsökningen) och kostnaden på ett SD-område på 64 MB (≈530 läsningar i stället för 32 000) |
| `native/test_telemetry` | 28 | `LoopStats`, `LogSettings` (NVS, version), `DebugLogger` (alla kanaler, NAV), `DebugConsole` (meny, snabbtangenter, bussavsökning `b`, förbjudet under ARM, sparar bara utan ARM), `WebDebugServer` (rutter, JSON, brevlåda), `OledDisplay` (byte över I2C, ram, invertering vid förlorad förbindelse) |
| `native/test_sim` | 15 | Flygningar i sluten slinga av hela firmwaren med flygplansmodellen: återhämtning från krängning, CRUISE i sidvind, LOITER, RTH, failsafe RTH/glidflykt, geofence, autostart från bana, handstart, autolandning, termik, RESCUE ur en spiral, farthållning och överstegringsskydd, ett riktigt pitotrör i slingan, autotrimning av ett ”snett” flygplan, sensorfel under flygning (IMU, barometer, pitotrör, GPS) |
| `native/test_feedback_units` | 14 | Återkopplingsmodulerna en och en: hastighetskällor, i luften/på marken, RLS-skattningen, regulatorn, överstegringsindikatorer, avbrott av start/landning |
| `native/test_app` | 10 | `src/main.cpp` på ESP32-S3 med bänksatsen MPU6500/BMP581/QMC5883P/OLED: `loop()`-perioden, sändare → servon, ARM, lägen, förlorad förbindelse, konsol, panel, skärm, svart låda (en uppgift på kärna 0, inspelning vid gas, en flygning efter DISARM, `bb list`) |
| `native/test_app_lsm6dsv_pitot` | 9 | `src/main.cpp` på ESP32-S3 med flygsatsen: LSM6DSV + QMC6309 + SPL06 + BMP581 i pitotröret + GPS – identifiering av alla kretsar, pitotrörets nollpunkt och hastighet, höjd, hempunkt från GPS, STABILIZE efter vinklarna från kretsen, RTH till hempunkten, bussavsökning, panel |
| `native/test_app_icm45686_esp32dev` | 5 | `src/main.cpp` på **ESP32 38-pin** (`BOARD_ESP32_CLASSIC`) med satsen ICM-45686 + QMC6309 + SPL06 + BMP581: kortets stiftbeläggning, IPREG-filter, handstart, stabilisering och fart, avsökning av en enda buss |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | `src/stm32/main.cpp` på **STM32H743** med flygsatsen: uppgifter och prioriteter, en period på 2 ms, pitotröret, PWM-timrar och `pulseIn`, MAVLink under flygning, lägesbyte från GCS:en, inställningar skrivna av en bakgrundsuppgift till ”flashen”, skärmen på I2C1, konsolen |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743 med ICM-45686 och BMP581 **över SPI** + QMC6309: korrupt flash vid start, ALT_HOLD från GCS:en håller höjden, förlorad förbindelse → RTH, synligt i MAVLink; omskrivning av en skadad avbild |
| `native_stm32/test_blackbox_sd` | 29 | Den svarta lådan på ett SD-kort: FAT32 (med och utan MBR, en katalog som sträcker sig över två kluster, brusposter, en främmande/fragmenterad/tom volym), `SdFileRegion` (delblock, cache, radering, gränser, fel), den riktiga drivrutinen `Stm32SdCard` ovanpå en attrapp av `HAL_SD` (4 bitar, reservhastigheter, omförsök, upptaget kort, ojusterade buffertar), ringen på kortet (omstart, strömavbrott, avsökningskostnad), markören ”ringen är tom”, inspelning av en flygning på en `FlightController`, en felomstart upptäckt via `RCC->RSR`, batteriets ADC, kortfel under flygning, ett långsamt kort, hämtning via konsolen, tangenten `D` |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | `src/stm32/main.cpp` med ett SD-kort: uppstarten hittar kortet och filen, uppgiften `bbox` skriver flygningen, slingans period sträcks inte ut, `bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | `src/stm32/main.cpp` utan SD-kort: den svarta lådan är avstängd och förklarar varför, flygplanet flyger, menyn `k` går inte sönder |
| `test_feedback` | 10 | En flygplanssimulering i sluten slinga med en återkopplingsslinga (på datorn och på kortet) |
| `test_imu_orientation` | 5 | Kalibrering av IMU-monteringen på 300 slumpmässiga monteringar (på datorn och på kortet) |
| **Totalt** | **387** | 340 i `native` + 47 i `native-stm32` (plus 9 bara på kortet – `test_blackbox_sd`) |

### Tester på STM32-kortet

`test/test_blackbox_sd` är inte native: SDMMC-drivrutinen, kortet och tidtagningen är riktiga. Testerna körs i en FreeRTOS-uppgift, och bredvid den körs en uppgift som imiterar flygslingan med högsta prioritet (period 2 ms): den avbryter testerna mitt i kortåtkomster, precis som i firmwaren. Utan den kan man inte fånga felet som faktiskt hittades på kortet: vid avbrott flödade SDMMC:s FIFO över (`HAL_SD_ERROR_RX_OVERRUN`), vilket aldrig händer i en naken slinga.

| Test | Vad det kontrollerar |
|---|---|
| `reset_cause_is_a_normal_one` | återställningsorsaken (`RCC->RSR`) är varken watchdog eller spänningsfall |
| `card_is_detected_on_four_bit_bus` | kortet upptäcks på en 4-bitarsbuss vid 24 MHz |
| `file_is_found_and_contiguous` | `BLACKBOX.BIN` hittas på FAT32 och är sammanhängande |
| `multi_block_writes_work_at_every_length` | skrivning av 1, 2, 4 och 8 block i en enda åtkomst |
| `pages_write_with_bounded_latency_and_read_back_intact` | sidor på 256 B: den sämsta skrivningen < 250 ms (SD-gränsen), stadigt > 40 KB/s, läsning och radering |
| `header_scan_cost_on_the_whole_area` | kostnaden för att läsa ett sektorhuvud och för en fullständig avsökning |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | radera allt, två flygningar med 20 000 poster vardera, ”omstart”: stickprovsavsökningen tar < 2 s, posterna läses i ordning med korrekt CRC; en tom ring känns igen på sin markör på < 100 ms |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | den riktiga `BlackBox` med 500 Hz IMU i realtid: inte en enda förlorad post, flygningen kan läsas efter en ”omstart” |
| `the_flight_task_was_not_disturbed` | skrivningen till kortet rubbade inte imitatoruppgiftens period (avvikelse < 3 ms) |

Körning (ett kort med filen – `python tools/blackbox.py sd-prepare E:`; **testet raderar alla flygningar i filen**):

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

Sätt kortet i DFU-läge (på DevEBox – en tråd från BT0 till 3V3 plus RST, WinUSB-drivrutinen via Zadig; detaljer i [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743)). STM32-konsolen är USB CDC: efter flashning dyker porten inte upp direkt, och `pio test` hinner ibland inte öppna den (”could not open port”) – kör i så fall `pio test ... --without-testing` och läs utdata med valfritt terminalprogram med DTR påslaget (testerna väntar upp till 60 s på att porten öppnas). Efter testerna väntar kortet på tangenten **`D`** – det startar om i DFU utan tråden.

Resultat på DevEBox H743 + ett 16 GB-kort (2026-10-02): `test_blackbox_sd` – 9/9, `test_feedback` – 10/10, `test_imu_orientation` – 5/5; siffrorna för kortets hastighet finns i [BLACKBOX.md](BLACKBOX.md#vad-som-mättes-på-kortet).

### Bänkfirmware – `test/bench/`

Det här är inte testsviter utan separata små PlatformIO-projekt som flashas till kortet i stället för flygfirmwaren (`pio test` ser dem inte: mappnamnen börjar inte med `test_`). Stift och gränser tas från den gemensamma `Config.h`.

| Projekt | Vad det gör |
|---|---|
| `bench/elevator_sweep` | Svänger höjdroderspaken (CH2) programmatiskt genom `ControlMixer` och `FlightOutputs`, som en levande spak: upp 100 % av utslaget, ned 60 %, mjukt, med pauser; 20 s arbete – 20 s i neutralläge. I ändlägena mäter den pulsen på utgångarna. Gas på minimum |

För att flasha: `pio run -d test/bench/elevator_sweep -t upload`. För att återställa flygfirmwaren: `pio run -e esp32-s3 -t upload`.

---

## Täckning

Den beräknas av `gcovr` över `include/` och `src/` (allt som går in i firmwaren), över båda native-miljöerna tillsammans: `gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`.

| Lager | Rader | Grenar |
|---|---|---|
| `autopilot` | 920/943 (97,6 %) | 645/731 (88,2 %) |
| `autopilot/feedback` | 683/702 (97,3 %) | 501/570 (87,9 %) |
| `control` | 252/256 (98,4 %) | 171/189 (90,5 %) |
| `hal` | 98/102 (96,1 %) | 26/26 (100 %) |
| `hal/esp32` | 101/102 (99,0 %) | 21/22 (95,5 %) |
| `hal/stm32` | 149/158 (94,3 %) | 35/52 (67,3 %) |
| `rc` | 92/92 (100 %) | 41/42 (97,6 %) |
| `sensors` (alla) | 1444/1446 (99,9 %) | 716/835 (85,7 %) |
| `storage` | 220/220 (100 %) | 158/178 (88,8 %) |
| `telemetry` | 1413/1440 (98,1 %) | 1123/1269 (88,5 %) |
| `src` (`main.cpp`, `stm32/main.cpp`) | 118/123 (95,9 %) | 20/29 (69,0 %) |
| **Totalt** | **5490/5584 (98,3 %)** | **3457/3943 (87,7 %)**; funktioner 877/902 (97,2 %) |

Vad som förblir otäckt, och varför:

- **Handstart** (`TakeoffSequencer`: `WaitLaunch`, `launchDetected()`) – onåbar så länge `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false`; den kommer med i testerna när konstanten blir konfigurerbar (flyttas till `Config.h`).
- **Kortberoende kod:** en utgång utan stift (`PIN_RUDDER = -1` förekommer bara på C3), en GPS utan TX-stift (C3) – native-testerna kör stiftbeläggningarna för S3, 38-pin och STM32, men inte C3 (C3 kontrolleras av byggmatrisen).
- **STM32:** kärnans felgrenar (ingen timer på stiftet, timerpoolen uttömd), meddelandet `FreeRTOS не запустился` (”FreeRTOS startade inte”) – på datorn returnerar `vTaskStartScheduler()` alltid.
- **Defensiva grenar** som inte kan nås via det publika API:t: `default`/`Count` i en `switch` över uppräkningar, `return "?"`.
- Filer utan körbara rader (`Config.h`, `Channels.h`, `FeedbackConfig.h`, strukturerna `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets`, makrona i `SensorSelection.h`, panelens HTML) syns inte i rapporten – de kompileras in i testerna, men gcov har inget att räkna i dem.

---

## Statisk analys

| Verktyg | Kommando | Profil |
|---|---|---|
| GCC | `tools/build_matrix.sh` (eller `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`) | Alla kort × alla sensorsatser. Native-testbygget använder alltid `-Wall -Wextra -Wshadow`; `stm32h743` använder `-Wall -Wextra` (`build_src_flags`; `-Wshadow` brusar på STM32duinos egna headerfiler) |
| cppcheck | `pio check -e esp32-s3`; `pio check -e stm32h743` | `check_*` i `[esp32_common]`: `include/` och `src/` (utom `stm32/`), warning/style/performance/portability, inbäddade `// cppcheck-suppress`-kommentarer bara för falska larm (U8g2-callbacken, `setup/loop`). För `stm32h743` – samma flaggor över `include/hal/stm32/` och `src/stm32/` |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`: bugprone, clang-analyzer, performance, `misc-include-cleaner` och andra; avstängda kontroller förklaras i själva filen |

clang-tidy körs med attrapperna från `test/native/support`: clang kan inte tolka ESP-IDF:s headerfiler för värdarkitekturen (om du försöker med `pio check` med `clangtidy` avbryts analysen på tolkningsfel och kontrollerar ärligt talat ingenting). `misc-include-cleaner` ser till att varje headerfil inkluderar det den använder: ”paraply”-headerfilerna (`FeedbackModules.h`, API:t i `IBoard.h`/`RegisterDevice.h`, makrona i `SensorSelection.h`) är märkta med `// IWYU pragma: export`. Skriptet hoppar över STM32-koden (`include/hal/stm32/`, `src/stm32/`) – den täcks av bygget, av cppcheck för env `stm32h743` och av testerna i env `native-stm32`.

Byggmatrisen vid den senaste körningen – **24/24 utan varningar**:

| Kort | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck (`esp32-s3`, `stm32h743`) – 0 anmärkningar i projektets kod.

---

## Hur man skriver nya tester

1. En modul med logik och utan hårdvara – ett direkt enhetstest: tiden skickas som parameter eller flyttas fram med `fake::advanceMs()`.
2. En kretsdrivrutin – via `I2cRig`/`SpiRig`: registren i en simulerad krets, kontroll av de skrivna värdena (`chip.lastWrite(reg)`) och tolkningen av data. För formler, använd en referens från databladet eller en oberoende beräkning, inte en kopia av koden.
3. Klasser med oändliga FreeRTOS-uppgifter – `fake::findTask("name")` + `fake::runTask(task, n)`; STM32:s flyguppgift drivs på samma sätt.
4. Hela firmwaren med en annan sensorsats eller ett annat kort – en separat svit som, före `#include "../../../src/main.cpp"`, sätter `SENSOR_KIT` (eller `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`); kretsarna kommer från `helpers/ChipEmulators.h`. För STM32 – `test/native_stm32/`.
5. Ett nytt autopilotläge – ett flygscenario i sluten slinga i `test_sim`.
6. En ny svit – en mapp `test/native/test_<namn>/test_main.cpp` med `main()`; `setUp()` anropar `resetWorld()` om sviten inte behöver tillstånd mellan testerna.
7. Hittat ett fel – skriv först ett test som fångar det, åtgärda det sedan.
