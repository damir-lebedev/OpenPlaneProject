# DEVELOPER_GUIDE.md – utvecklarhandbok för OpenPlaneProject

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../DEVELOPER_GUIDE.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

En teknisk karta över firmwaren: vilken fil som ansvarar för vad, hur data flödar från mottagaren och sensorerna till servona, vilka teckenkonventioner som håller ihop hela kedjan, hur webb-API:t är organiserat och hur man bygger ut projektet. Den är avsedd för en utvecklare som skriver C++ och snabbt vill hitta rätt i det här repositoryt (grenen `main`), inte för att lära sig grunderna i språket eller i PlatformIO.

En översikt över projektet och prototypens status finns i [`../README.md`](README.md); vad som kopplas vart och hur man flyger finns i [`PILOT_GUIDE.md`](PILOT_GUIDE.md); planerna finns i [`ROADMAP.md`](ROADMAP.md). Här finns bara kod. Hela arkitekturen (lager, uppgifter, tillståndsmaskiner) finns i [`ARCHITECTURE.md`](ARCHITECTURE.md), en referens för varje klass i [`reference/`](reference/README.md) och testerna i [`TESTING.md`](TESTING.md).

> Projektet är under aktiv utveckling. ESP32-S3-bänken är monterad och verifierad med alla sensorer, men **autopiloten har ännu inte provats under flygning** – det noteras överallt där det gäller en specifik modul. Om du tvivlar på vad koden gör, läs källkoden igen, inte dokumentet.

---

## Innehåll

1. [Lagrens arkitektur](#lagrens-arkitektur)
2. [FreeRTOS-uppgifter och styrslingan](#freertos-uppgifter-och-styrslingan)
3. [Filreferens](#filreferens)
4. [Teckenkonvention: från IMU:n till servot](#teckenkonvention-från-imun-till-servot)
5. [RC-kanalkarta, ARM och failsafe](#rc-kanalkarta-arm-och-failsafe)
6. [Sensordata](#sensordata)
7. [Genomgång av FlightController::update()](#genomgång-av-flightcontrollerupdate)
8. [Webbpanelens HTTP-API](#webbpanelens-http-api)
9. [Konsol och diagnostik](#konsol-och-diagnostik)
10. [Val av kort och stiftbeläggning](#val-av-kort-och-stiftbeläggning)
11. [Hur man lägger till en ny sensor](#hur-man-lägger-till-en-ny-sensor)
12. [Hur man lägger till ett nytt autopilotläge](#hur-man-lägger-till-ett-nytt-autopilotläge)
13. [Återkoppling (förarbete, inte ansluten)](#återkoppling-förarbete-inte-ansluten)
14. [Hur man lägger till ett nytt kort](#hur-man-lägger-till-ett-nytt-kort)
15. [Kommandon för bygge, uppladdning och monitor](#kommandon-för-bygge-uppladdning-och-monitor)
16. [Kända begränsningar](#kända-begränsningar)
17. [Hur man gör ändringar](#hur-man-gör-ändringar)

---

## Lagrens arkitektur

Nästan alla klasser finns i headerfiler utlagda i mapparna `include/<lager>/`. Varje headerfil inkluderar själv det den använder (`#include "config/Config.h"`, `"hal/II2CBus.h"`, ... – sökvägar från `include/`). `src/main.cpp` är den enda monteringspunkten (composition root): den skapar alla objekt, länkar dem och kör `setup()`/`loop()`. Beroendena är enkelriktade – ett lägre lager vet ingenting om ett övre.

```
include/
├── config/      Config.h (stift, alla inställningar), Channels.h (kanalnamn),
│                Controls.h (vad varje brytare gör – en rad per kanal)
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (en registerenhet ovanpå I2C/SPI), Rtos
│   ├── esp32/   Esp32Board + omslag kring Wire/SPI/HardwareSerial/LEDC
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (inställningar i flash i stället för NVS)
├── storage/     KeyValueStore, KvPreferences – lagring av inställningar utan NVS
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  förarbete till återkopplingsslingan – INTE ansluten (se avsnittet nedan)
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (ett rör av två barometrar)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — ESP32-firmware (S3, C3, 38-pin)
src/stm32/main.cpp  — STM32H743-firmware (FreeRTOS-uppgifter, MAVLink)
```

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLIKATION  src/main.cpp / src/stm32/main.cpp – montering av objekt  │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ KOORDINERING  control/FlightController – operationsordning per cykel   │
│ TELEMETRI     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ STYRNING            │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12 lägen   │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ navigering, PID    │  │ RcChannelState        │
│ ThrottleManager     │  │ PilotSwitches         │  │ RcInput               │
│ ArmingManager       │  └──────────┬────────────┘  └───────────────────────┘
│ FlightOutputs       │             │ ImuSensor* / BarometerSensor* / ...
└─────────┬───────────┘             ▼
          │           ┌─────────────────────────────────────────────────┐
          │           │ SENSORER                                         │
          │           │ ImuSensorBase ── MPU6050, ICM42688, LSM6DSV,     │
          │           │   └ AttitudeEstimator     ICM45686               │
          │           │ BarometerBase ── BMP388, BME280, SPL06, BMP581   │
          │           │ MagnetometerBase ── QMC5883P / L, QMC6309        │
          │           │ UbloxM10_Gps, PitotDualBaroAirspeed              │
          │           └──────────────────────┬──────────────────────────┘
          ▼                                  ▼ IRegisterDevice / IUartPort
┌───────────────────────────────────────────────────────────────────────┐
│ HAL  IBoard / II2CBus / ISpiBus / IUartPort / IServoOutput             │
│      RegisterDevice: I2cRegisterDevice, SpiRegisterDevice              │
│      esp32/Esp32Board — Wire, Wire1, SPI, HardwareSerial, LEDC         │
│      stm32/Stm32Board — Wire, I2C1, SPI, Uart, HardwareTimer, flash    │
└───────────────────────────────────────────────────────────────────────┘
```

Reglerna som håller arkitekturen ren:

- **HAL** är det enda lagret som får känna till en specifik mikrokontroller (`Wire`, `SPI`, `HardwareSerial`, `ledc*`). Allt ovanför arbetar bara med gränssnitt. Att flytta till en annan mikrokontroller innebär en ny `hal/<mcu>/<Mcu>Board.h`; resten av koden ändras inte (ett exempel är `hal/stm32/` för STM32H743).
- **Sensordrivrutiner känner inte till bussen.** De får en `IRegisterDevice&` – en I2C-enhet med en adress eller en SPI-enhet med en CS skapas i `SensorSelection.h`. Samma `BMP388_Sensor` fungerar både över I2C och SPI.
- **Det som är gemensamt finns i basklasserna.** Kalibrering, axelrotation, tecken, orienteringsfiltret, höjd och vertikal hastighet, lagring av kompasskalibrering, räkning av bussfel – i `ImuSensorBase`/`BarometerBase`/`MagnetometerBase`. En kretsdrivrutin innehåller bara registren och formlerna från databladet.
- **RC och utgångar** vet ingenting om flygplanet: iBUS-byte → kanaler, PWM-värden → utgångar.
- **Styrning och Autopilot** är logik över data, utan UART, PWM eller Wi-Fi. Tid, där den behövs (klaffar), skickas som parameter.
- **Koordinering** (`FlightController`) är den enda klassen som ser flera lägre lager samtidigt och bestämmer operationsordningen.
- **Applikation** (`main.cpp`) är det enda stället där `Esp32Board`, enheterna och sensorerna skapas och där allt kopplas ihop för hand, utan ett DI-ramverk.

---

## FreeRTOS-uppgifter och styrslingan

| Var | Vad | Period |
|---|---|---|
| Kärna 1, `loop()` (Arduinos loopTask) | `applyPendingCommands()` → `FlightController::update()` → `DebugLogger::update()` → `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms (500 Hz), `vTaskDelayUntil` |
| Kärna 0, uppgiften `web` | `WebServer::handleClient()` | var 2:a ms |
| Kärna 0, uppgiften `oled` | ritning av SSD1306 över den andra I2C-bussen | 200 ms |
| Kärna 0 | ESP-IDF:s Wi-Fi-stack | – |

- Slingans period hålls av `vTaskDelayUntil`, inte av `delay(2)` efter arbetet –
  frekvensen beror inte på hur länge cykeln varade. Efter ett långt block
  (en kalibrering från konsolen) börjar tidtagningen om, och de missade cyklerna
  tas inte igen i en skur.
- På bänken (ESP32-S3, alla sensorer): 500 Hz, i genomsnitt cirka ~0,7 ms arbete per
  cykel, den sämsta cykeln ~1,4 ms. Det skrivs ut var 10:e s som en `SYS:`-rad.
- Tidsgränsen för en I2C-transaktion är 5 ms (standard i Wire är 50 ms): en transaktion
  som hängt sig på grund av störningar stoppar inte slingan länge.
- **Dataseparation mellan uppgifter.** Webben och OLED-skärmen *läser* bara
  tillståndet (`FlightController`/`Autopilot`/`LoopStats`) – det är separata
  16/32-bitarsfält, så i värsta fall syns värdena från intilliggande cykler.
  Panelens *kommandon* (`setmode`/`setpid`) tillämpas inte direkt från
  webbuppgiften: de läggs i en ”brevlåda” under `portMUX` och hämtas av
  flygslingan i `WebDebugServer::applyPendingCommands()`.
- `Serial` (UART0 → CH343-bryggan → ”COM”-kontakten) med en sändbuffert på 4 KB:
  en felsökningsram (~600 tecken) blockerar inte slingan medan den skickas.

---

## Filreferens

### `config/`

| Fil | Ansvarar för |
|---|---|
| `Config.h` | Alla stift (ett block per kort: `BOARD_ESP32_S3/C3/CLASSIC`, `BOARD_STM32H743`) och inställningarna: iBUS och förlorad förbindelse; roderytornas utslag; klaffar; servoreversering; montering av IMU och kompass; ARM; failsafe (RTH eller glidflykt); pitotröret (`PITOT_*`); alla siffror för autopilotens lägen och funktioner; slingan; Wi-Fi; MAVLink; felsökning |
| `Channels.h` | Kanalnamn: `AILERON`, `ELEVATOR`, `THROTTLE`, `RUDDER`, `ARM`, `SWB`, `SWC`, `SWD`, `VRA`, `VRB` |
| `Controls.h` | Tabellen `BINDINGS`: vad varje brytare och ratt gör, en rad per kanal, kontroller med `static_assert` |

### `hal/`

| Fil | Ansvarar för |
|---|---|
| `IBoard.h` | Ingångspunkten till hårdvaran: `i2c()`, `displayI2c()` (en andra buss för skärmen, kan vara `nullptr`), `spi()`, `rcUart()`, `gpsUart()`, `telemetryUart()` (MAVLink, kan vara `nullptr`), `servo(ServoChannel::*)` (7 utgångar med AUX1/AUX2), `setBuzzer()` |
| `Rtos.h` | FreeRTOS-uppgifter på samma sätt på ESP32 (kärna 0) och STM32 (prioriteter), ledig heap |
| `II2CBus.h` | I2C-bussen: primitiver i stil med `Wire` + hjälparna `writeRegister()`, `readRegisters()` (kontrollerar att exakt `count` byte kom), `readRegister()`, `probe()` |
| `ISpiBus.h`, `IUartPort.h`, `IServoOutput.h` | SPI, UART, en PWM-utgång (`measurePulseUs()` – diagnostik av den verkliga pulsen) |
| `RegisterDevice.h` | `IRegisterDevice` – ”en uppsättning 8-bitarsregister”; `I2cRegisterDevice` (adress), `SpiRegisterDevice` (CS, frekvens, dummybyte före data) |
| `esp32/Esp32Board.h` | Implementeringen av `IBoard`: `Wire` (sensorer), `Wire1` (skärmen, om kretsen har två I2C-styrenheter), `SPI`, två `HardwareSerial`, 5 LEDC-kanaler |
| `esp32/Esp32I2CBus.h` | `II2CBus` ovanpå valfri `TwoWire`, tidsgräns 5 ms |
| `esp32/Esp32ServoOutput.h` | PWM via LEDC: 50 Hz, 14 bitar; stift −1 – utgången är inte dragen. Biblioteket ESP32Servo används inte – se [begränsningar](#kända-begränsningar) |
| `esp32/Esp32SpiBus.h`, `esp32/Esp32UartPort.h` | Tunna omslag kring `SPI` och `HardwareSerial` |
| `stm32/*` | STM32H743: `Stm32Board` (+ radiomodemets UART4), bussar, PWM-timrar, `Stm32FlashStorage` (inställningar i en flashsektor, skrivna av en bakgrundsuppgift), `compat/Preferences.h` |

### `storage/`

| Fil | Ansvarar för |
|---|---|
| `KeyValueStore.h` | En avbild ”namnrymd/nyckel → byte” med CRC32 i RAM ovanpå valfritt medium (`IFlashStorage`); ett identiskt värde skrivs inte om |
| `KvPreferences.h` | ESP32:s `Preferences`-API ovanpå `KeyValueStore` |

### `rc/`

| Fil | Ansvarar för |
|---|---|
| `RcChannelState.h` | En ögonblicksbild av de 10 kanalerna |
| `RcInput.h` | `clamp()`, `centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS → kanaler: en ram på 32 byte, CRC, kanalvärdet är de låga 12 bitarna (`& 0x0FFF`); `isSignalLost()` = inga ramar (eller inga ännu) ∥ failsafe-värdet för gas; ramräknare |

### `control/`

| Fil | Ansvarar för |
|---|---|
| `ControlCommand.h` | Roderkommandot i fysiska tecken – det gemensamma språket för spakarna, autopiloten och mixern |
| `ControlMixer.h` | `fromSticks(rc)` → `ControlCommand`; `updateFlaps(target, now)`; `mix(command)` → PWM med servoreversering; flaperoner: skevrodren `flaps ± roll` (minus – luftbroms) |
| `FlapsController.h` | Mjuk utfällning/infällning av klaffarna, med tiden som parameter |
| `ThrottleManager.h` | Gas från spaken; vid förlorad förbindelse – `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | ARM med brytaren SwA (en övergång OFF→ON med gasen i botten + lägets sensorkontroller), omedelbar DISARM |
| `FlightOutputState.h` | Önskad PWM: `aileronLeft`, `aileronRight`, `elevator`, `rudder`, `throttle`, `aux1` (last), `aux2` (kamera) |
| `Beeper.h` | Summern: via funktionen `BEEPER` eller ”modellen borttappad” på marken |
| `FlightOutputs.h` | Utgångstabellen (`outputInfo()`: nyckel, namn, stift, om den är obligatorisk, tillståndsfältet) och allt ovanpå den i en slinga: `begin()`, `write()`, `setFailsafe()`, status, `printPulseSelfTest()` |
| `FlightController.h` | Operationsordningen per cykel, förlorad förbindelse (`applyLinkLoss()`), getters för telemetri |

### `autopilot/`

| Fil | Ansvarar för |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode` (12 lägen), `Feature`, `Knob`, `PilotInputs`, namn |
| `ControlBinding.h` | `Binding`, fabrikerna `Bind::modes/mode/feature/knob`, kontrollerna `BindingCheck` |
| `PilotSwitches.h` | Bindningstabellen → läget, funktionerna och rattarna i varje cykel; fördelningen vid start |
| `Autopilot.h` | 12 lägen, failsafe RTH/glidflykt, geofence, hempunkt, koordinerade svängar, autotrimning; `update(armed, linkLost, throttle, sticks)` → `getCommand()`, `applyThrottle()` |
| `Navigation.h` | `Geo` (avstånd, bäring, förskjutning), `Guidance` (roll för en kurs, cirkelns vektorfält) |
| `AltitudeSpeedController.h` | Tippning för höjd, gas för lufthastighet (TECS-lite) |
| `LaunchController.h`, `SoaringController.h` | Tillståndsmaskinerna för handstart och termikflygning |
| `AutoTrim.h` | Autotrimning, lagrad i NVS/flash |
| `PidController.h` | PID: D från sensorns hastighet (gyroskop, variometer), anti-windup, integratorn fryst utan ARM |
| `feedback/*` | **Förarbete, inte anslutet:** adaptiv återkoppling, start och landning – se [Återkoppling](#återkoppling-förarbete-inte-ansluten) |

### `sensors/`

| Fil | Ansvarar för |
|---|---|
| `SensorInterface.h` | Gränssnitten `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` och datastrukturerna |
| `SensorSelection.h` | Vilken krets som kompileras in (`#define SENSOR_*`, kan åsidosättas med en byggflagga) och på vilken buss (`SELECTED_*_DEVICE(board)`) |
| `SensorMounting.h` | Rotation av kretsens axlar till flygplanets axlar (0/90/180/270° medurs) – för kompassen och för en IMU utan monteringskalibrering |
| `imu/ImuOrientation.h` | IMU-montering som en matris ”kretsens axlar → flygplanets axlar”: från `IMU_ROTATION_CW_DEG` eller från tre positioner (plant, nosen upp, höger vinge ned) med rimlighetskontroll; lagras i NVS |
| `imu/ImuSensorBase.h` | Gemensam IMU-kod: gyrokalibrering + kontroll före flygning (stillhet, 1g, ”upp” stämmer med monteringen), monteringskalibrering (`calibrateOrientation()`), skala, rotation, flygtekniska tecken, bussfel |
| `imu/AttitudeEstimator.h` | Komplementärfilter för roll/tippning, girintegral |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500 (kretsen identifieras med WHO_AM_I): ±2000°/s, ±16g, DLPF ~41 Hz, 1 kHz. **På bänken** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P: ±2000°/s, ±16g, 1 kHz, UI-filter 50 Hz. Inte provad på hårdvara |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X: ±2000°/s, ±16g, 960 Hz, LPF1/LPF2; I2C 0x6A/0x6B eller SPI. Inte provad på hårdvara |
| `imu/ICM45686_Sensor.h` | ICM-45686: ±2000°/s, ±16g, 1,6 kHz, lågpassfilter via de indirekta IPREG-registren; I2C 0x68/0x69 eller SPI. Inte provad på hårdvara |
| `baro/BarometerBase.h` | Gemensam barometerkod: avläsning bara av nya mätningar, höjd, vertikal hastighet via ett lågpassfilter, baskalibrering, fel |
| `baro/BMP388_Sensor.h` | BMP388 över I2C eller SPI (med SPI:s dummybyte), Bosch-kompensering, avläsning på flaggan data-klar. **På bänken (I2C)** |
| `baro/BME280_Sensor.h` | BME280/BMP280, Bosch-kompensering §8.1. Inte provad på hårdvara |
| `baro/SPL06_Sensor.h` | SPL06-001: koefficienter och formler från databladet, 32 Hz ×16; I2C 0x76/0x77 eller SPI. Inte provad på hårdvara |
| `baro/BMP581_Sensor.h` | BMP581: sekvensen från BMP5_SensorAPI, 16×/2×, IIR; I2C 0x46/0x47 eller SPI; fungerar både som huvudbarometer (standardsatsen på bänken) och som pitotrör. Inte provad på hårdvara |
| `mag/MagnetometerBase.h` | Gemensam kompasskod: avläsning med 50 Hz, hard-iron-kalibrering i NVS, axelrotation, kurs, fel |
| `mag/QMC5883P_Sensor.h` | QMC5883P, 0x2C. **På bänken** |
| `mag/QMC5883L_Sensor.h` | QMC5883L, 0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309, 0x7C: ±8 G, 200 Hz. Inte provad på hårdvara |
| `gps/UbloxM10_Gps.h` | u-blox M10: inställning med CFG-VALSET (115200 baud, 10 Hz, NAV-PVT, ingen NMEA), tolkning av NAV-PVT. Inte ansluten på bänken |
| `airspeed/AirspeedSensor.h` | Gränssnittet för lufthastighetssensorn: differenstryck, IAS, TAS, densitet |
| `airspeed/PitotDualBaroAirspeed.h` | Det hemmabyggda pitotröret: en BMP581 i röret + en barometer i flygkroppen; nollpunkt på marken, lågpassfilter, densitet från statiskt tryck, felupptäckt |

### `telemetry/` och applikationen

| Fil | Ansvarar för |
|---|---|
| `DebugLogger.h` | Logg per kanal (`LogSettings.h`): varje kanal har sin egen rad, sin egen debounce-tolerans och sitt eget läge; tyst medan menyn är öppen |
| `DebugConsole.h` | En textmeny i portmonitorn (`h`) och snabbtangenter (`l`/mellanslag/`s`/`i`/`o`/`m`/`p`/`b`); sparar logginställningarna i NVS när du lämnar menyn och bara utan ARM |
| `LogSettings.h` | Loggkanalerna (STAT, RC, OUT, ATT, AP, ALT, MAG, GPS, IMU, NAV, SYS) och deras lägen: av / vid ändring / kontinuerligt; lagras i NVS |
| `WebDebugServer.h` | Åtkomstpunkten, rutterna, JSON för `/api/status`, kommandobrevlådan; körs i en egen uppgift på kärna 0 |
| `WebDashboardPage.h` | Panelens HTML/JS som en enda literal; webbläsaren bygger raderna för kanaler/utgångar/sensorer från JSON |
| `OledDisplay.h` | SSD1306 via U8g2 ovanpå `II2CBus`, en egen uppgift (`Rtos`) |
| `MavlinkCodec.h`, `MavlinkTelemetry.h` | MAVLink 2 för QGroundControl / Mission Planner: ramar, strömmar, PID-parametrar, lägesbyte från marken |
| `LoopStats.h` | Frekvens, genomsnittlig och sämsta cykeltid per sekund (OLED) och den sämsta sedan senaste avläsningen (`takePeakUs()`, SYS-raden) |
| `src/main.cpp` | ESP32: skapande av objekten, `setup()`, `loop()` med `vTaskDelayUntil` |
| `src/stm32/main.cpp` | STM32H743: samma objekt, MAVLink, den svarta lådan på SD-kort, uppgifterna `flight`/`storage`/`oled`/`bbox` |
| `src/stm32/sd_msp.cpp`, `src/stm32/bootloader.cpp` | STM32H743: SDMMC1-stift och klockor för `HAL_SD_Init`; konsoltangenten `D` – omstart i USB DFU-bootloadern |

---

## Teckenkonvention: från IMU:n till servot

Ett teckensystem för hela kedjan – så att spaken och autopiloten garanterat
rör roderytorna åt samma håll, och riktningen för varje servo anges på
exakt ett ställe.

**1. Sensorns axlar → flygplanets axlar.** `ImuSensorBase` roterar kretsens axlar
med matrisen `ImuOrientation` (body = R · chip) till flygplanets axlar: X
mot nosen, Y åt vänster, Z uppåt. Matrisen tas:

- från **monteringskalibreringen** (kommandot `o`, lagras i NVS) – kortet
  kan sitta i vilken orientering som helst. Tre positioner: ”plant” ger Z-axeln
  (och horisonten – accelerometerns nollförskjutning viks in i den), ”nosen
  upp” ger X-axeln (den del av ”upp” som är vinkelrät mot Z), ”höger vinge
  ned” ger Y-axeln. Nosen från steg 2 och nosen från steg 3
  (Y × Z) måste stämma överens inom ~25°, annars lutade piloten åt fel
  håll – kalibreringen avvisas; resultatet är medelvärdet av de två
  skattningarna. Verifierat på 300 slumpmässiga monteringar
  (`test/test_imu_orientation`, fel < 0,1°);
- annars – från `Config::IMU_ROTATION_CW_DEG` (kortet med kretsen
  uppåt; värdet är vart *kretsens* X-axel pekar om nosen är ”klockan
  12”), och horisonten är läget vid start.

Vid varje gyrokalibrering (start, `i`) görs en **kontroll före flygning**:
gyrobrus < 0,5 °/s (stillhet; i vila ~0,08), |a| ≈ 1g, ”upp” inom 45°
från det lagrade (kortet har inte flyttats). Om den misslyckas –
`ImuSensor::getPreflightProblem()` ≠ nullptr: `ArmingManager` armerar inte
de stabiliserade lägena, och `Autopilot::imuReady()` = false (noll korrigeringar
i alla lägen, inklusive glidflykt vid förlorad förbindelse).

> På den nuvarande GY-521 (en MPU6500-klon) är kretsen lödd vriden 90°
> i förhållande till de tryckta pilarna: X-pilen i tryckningen = kretsens
> Y-axel. Därför är `IMU_ROTATION_CW_DEG = 90` utan monteringskalibrering.
> Kontrollen efter varje omflyttning: nosen upp → P ökar åt det positiva
> hållet, höger vinge ned → R åt det positiva hållet.

**2. Vinklar och hastigheter (`ImuData`) – flygtekniska tecken:**

| Storhet | ”+” betyder |
|---|---|
| `roll`, `gyroX` | höger vinge ned |
| `pitch`, `gyroY` | nosen upp |
| `yaw`, `gyroZ` | nosen åt höger (medurs sett ovanifrån) |

**3. Kommandot (`ControlCommand`, µs utslag, ±500 = fullt utslag):**

| Fält | ”+” betyder | Från spaken |
|---|---|---|
| `roll` | roll åt höger (höger skevroder upp, vänster ned) | CH1: 2000 = höger |
| `pitch` | nosen upp (höjdrodret upp) | CH2 med motsatt tecken: 2000 = bort från dig = nosen ned |
| `yaw` | nosen åt höger (sidroder och noshjul åt höger) | CH4: 2000 = höger |
| `flaps` | klaffar ned (båda skevrodren ned) | SwB (CH6): 0 eller `FLAPS_DEPLOYED_US`, mjukt över `FLAPS_TRANSITION_MS` |

PID:en beräknar `error = target − actual`: roll åt höger (roll > 0) → ett
negativt rollkommando → flygplanet rätar upp sig. Autopilotens korrigeringar
läggs till spakkommandot **före** mixern, med samma tecken.

**4. Kommando → PWM.** `ControlMixer::mix()` beräknar bakkantens
utslag för varje yta (skevroder: ned = ”+”, vänster = `flaps + roll`,
höger = `flaps − roll`; höjdroder: upp = ”+”; sidroder: höger = ”+”) och
omvandlar det till PWM `1500 ± utslag`, med omvänt tecken för servon med
`Config::*_REVERSED = true`. Standardvärdena återger firmwarens
tidigare beteende för spakarna. Kontrollen på det monterade flygplanet finns i
checklistan före flygning i [`PILOT_GUIDE.md`](PILOT_GUIDE.md). Reverseringen
ska ändras i `Config.h`, **inte på sändaren** – annars blir
spaken och autopiloten oense.

---

## RC-kanalkarta, ARM och failsafe

Källan är `include/config/Channels.h`. En FS-i6-sändare (10 kanaler,
mode 2) + en FS-iA6B-mottagare, iBUS 115200.

| Kanal | Reglage på sändaren | Namn | Syfte |
|---|---|---|---|
| CH1 | höger spak ←→ | `AILERON` | Roll |
| CH2 | höger spak ↑↓ | `ELEVATOR` | Tippning |
| CH3 | vänster spak ↑↓ | `THROTTLE` | Gas, fullt utslag; < 950 = mottagarens failsafe |
| CH4 | vänster spak ←→ | `RUDDER` | Sidroder + styrhjul (ett servo) |
| CH5 | SwA | `ARM` | ≥ 1750 = ARM (på FS-i6 är det brytaren ned, mot dig) |
| CH6 | SwB | `SWB` | klaffar som standard (≥ 1750 – utfällda) |
| CH7 | SwC (3 lägen) | `SWC` | läge som standard: < 1250 MANUAL, 1250–1749 STABILIZE, ≥ 1750 AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | RTH som standard |
| CH9 | VrA | `VRA` | stabiliseringens styrka som standard |
| CH10 | VrB | `VRB` | marschfart som standard |

CH6–CH10 tilldelas med en enda rad i `include/config/Controls.h`
([AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#tilldela-en-funktion-med-en-enda-rad)).

**ARM** (`ArmingManager`): brytaren går OFF→ON, gas < `THROTTLE_LOW_US`,
och sensorkontrollerna för det aktuella läget har godkänts. Annars nekas den,
med orsaken utskriven i Serial, och en ny cykel OFF→ON behövs. Att starta
kortet med brytaren redan på ON armerar inte. OFF – DISARM direkt.
Så länge planet inte är armerat tvingas gasen till ESC:n till `PWM_MIN`.

**Förlorad förbindelse** (`IBusReceiver::isSignalLost()`):

1. Inga ramar på mer än `RX_TIMEOUT_US` (500 ms) – en bruten tråd eller
   förlorad matning till mottagaren. Före den första ramen efter start räknas förbindelsen
   också som förlorad: kanalernas standardvärden (alla 1500) tas inte
   för sändarkommandon.
2. Gas < `RX_FAILSAFE_THROTTLE_US` (950) – den failsafe som ställts in i
   sändaren. **När sändaren tappas slutar FS-iA6B inte
   skicka ramar**, den upprepar de senaste värdena (verifierat på bänken), så
   utan en failsafe inställd i sändaren upptäcks den förlorade förbindelsen
   inte. Inställningen beskrivs i `PILOT_GUIDE.md`.

Vad som händer vid förlorad förbindelse (`FlightController::applyLinkLoss()`):

- **planet är armerat, GPS och en hempunkt finns**
  (`FAILSAFE_RTH`) – **hemflygning** med motor, cirkling över hempunkten; på
  OLED-skärmen – `FSRTH`, i loggen – `FAILSAFE_RTH`;
- **planet är armerat, ingen GPS** – **glidflykt**, motorn på
  `FAILSAFE_THROTTLE`: i alla lägen, även MANUAL, håller `Autopilot` rollen
  `FAILSAFE_GLIDE_ROLL_DEG` (0 – rakt, 10–20° – en cirkel över piloten)
  och tippningen `FAILSAFE_GLIDE_PITCH_DEG` (−3°, för att inte tappa fart
  utan motor), klaffarna fälls in; på OLED-skärmen – `GLIDE`, i
  loggen – läget `FAILSAFE_GLIDE`;
- **inte armerat** (på marken) eller IMU:n svarar inte – roderytorna
  går till neutralläge;
- läget och funktionerna byts inte från brytarna, sensorerna
  fortsätter att läsas. ARM nollställs inte – när förbindelsen återställts lyder
  flygplanet åter spakarna och det valda läget (automatisk start
  och handstart börjar bara om).

---

## Sensordata

Strukturerna finns i `include/sensors/SensorInterface.h`.

### `ImuData`

| Fält | Enhet | Betydelse |
|---|---|---|
| `gyroX`, `gyroY`, `gyroZ` | °/s | Vinkelhastigheter i flygplanets axlar, flygtekniska tecken (se ovan) |
| `accelX`, `accelY`, `accelZ` | g | Acceleration i flygplanets axlar: X mot nosen, Y åt vänster, Z uppåt |
| `roll`, `pitch` | ° | Komplementärfilter (α = 0,98, τ ≈ 0,1 s); de startar direkt från accelerometerns vinkel |
| `yaw` | ° | Gyrointegral, driver långsamt; startvärdet är kompasskursen |
| `temperature` | °C | Kretsens temperatur (formel för MPU6050 eller MPU6500) |
| `timestamp` | µs | `micros()` vid avläsningsögonblicket |

IMU-kalibrering (vid varje start och med kommandot `i`): 2 s orörligt,
gyrot → nollförskjutning, accelerometern → **det aktuella läget blir
horisonten**.

### `BarometerData`

| Fält | Enhet | Betydelse |
|---|---|---|
| `pressure` | Pa | Tryck |
| `temperature` | °C | Sensorns temperatur |
| `altitude` | m | Höjd **relativt kalibreringspunkten** (vid start); formel `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | Höjdens derivata över de verkliga mätningarna (50 Hz) via ett lågpassfilter med τ = 0,5 s |
| `timestamp` | µs | Ögonblicket för den senaste nya mätningen |

### `MagData`

| Fält | Enhet | Betydelse |
|---|---|---|
| `magX`, `magY`, `magZ` | µT | Fältet efter hard-iron-kalibrering, i flygplanets axlar (`MAG_ROTATION_CW_DEG`) |
| `headingDegrees` | ° (0..360) | `atan2(magY, magX)`, utan lutningskompensation; avläsningens riktning har ännu inte verifierats på ett monterat flygplan |
| `timestamp` | µs | Avläsningsögonblicket (50 Hz) |

### `GpsData`

| Fält | Enhet | Betydelse |
|---|---|---|
| `latitude`, `longitude` | ° | Från UBX-NAV-PVT |
| `altitude` | m | Över havet (hMSL) |
| `groundSpeed`, `heading` | m/s, ° | Markhastighet och kurs över marken |
| `numSatellites`, `fixType` | – | 0 = ingen fix, 2 = 2D, 3 = 3D |
| `horizontalAccuracy`, `verticalAccuracy` | m | Modulens noggrannhetsskattningar |

**Vad `isAvailable()` betyder.** För I2C-sensorer – sensorn svarade vid
`begin()` **och** de senaste avläsningarna misslyckas inte i följd (MPU – ~0,1 s,
barometer och kompass – ~0,5 s utan svar). När en avläsning misslyckas
skrivs data inte över med skräp: de tidigare värdena står kvar och felräknaren
ökar (syns med kommandot `s`). För GPS – minst en giltig
NAV-PVT, och den senaste är inte äldre än `GPS_TIMEOUT_US`.

**Om en sensor saknas** (`nullptr` eller `isAvailable() == false`) ger `Autopilot`
inga korrigeringar, och flygplanet flygs som i MANUAL. `main.cpp`
kalibrerar bara de sensorer som svarade.

---

## Genomgång av FlightController::update()

Anropas från `loop()` var 2:a ms. Ordningen är prioriteten:

1. **`receiver.update()`** – tolkning av de ackumulerade iBUS-byten.
2. **Brytare** – `switches->update(rc)`, bara medan förbindelsen lever (i en
   failsafe-ram återspeglar kanalerna inte brytarna): läget (bara
   vid en ändring), funktioner, rattar.
3. **Pilotens gas** – `throttle.update(rc, receiverFailsafe)`.
4. **Spakar** – `mixer.fromSticks(rc)` × `Knob::RATES`; klaffar –
   `mixer.updateFlaps(target)` (broms, brytare, ratt; utan förbindelse – 0).
5. **Sensorer och autopilot** – `autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   **alltid**, även utan förbindelse: vinkelfiltren får inte frysa. Så länge
   planet inte är armerat körs PID:en (roderytorna reagerar på lutning – praktiskt
   på skrivbordet), men integratorn hålls på noll. Utan förbindelse och
   armerat – failsafe RTH eller glidflykt.
6. **Summer** – `Beeper`.
7. **Förlorad förbindelse** – `applyLinkLoss()`: när armerat – roderytorna och
   gasen följer autopilotens failsafe-kommando, annars neutralläge och
   motorn av; `return`. Absolut prioritet över allt nedanför.
8. **ARM** – `arming.update(rc, false)`.
9. **Kommando** – `autopilot->getCommand()`: i de stabiliserade lägena är spaken
   den önskade vinkeln, och autopiloten ger de slutliga
   roderkommandona.
10. **Mixer** – `mixer.mix(command)` → PWM för skevrodren (flaps + roll),
    höjdrodret och sidrodret, med hänsyn till reverseringen.
11. **Gas** – `autopilot->applyThrottle(pilotThrottle)`: pilotens
    gas, autopilotens gas eller det högsta av de två (automatisk
    start). Därefter, om inte armerat eller `MOTOR_KILL` – tvingad till `PWM_MIN`. Den här
    kontrollen kommer sist så att inget läge kan smyga gasen förbi ARM.
12. **AUX** – last (`PAYLOAD_DROP`) och kamera (`CAMERA_TILT`, `CAMERA_STAB`).
13. **`outputs.write(output)`** – PWM till de 7 utgångarna.

---

## Webbpanelens HTTP-API

Implementeringen är `include/telemetry/WebDebugServer.h`. Åtkomstpunkten:
SSID `OpenPlane-Debug`, lösenord `12345678`, adress `http://192.168.4.1`.

### `GET /api/status`

```json
{
  "rc": [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1500],
  "armed": false,
  "failsafe": false,
  "outputs": {
    "aileronLeft":  { "us": 1500, "attached": true },
    "aileronRight": { "us": 1500, "attached": true },
    "elevator":     { "us": 1500, "attached": true },
    "rudder":       { "us": 1500, "attached": true },
    "esc":          { "us": 1000, "attached": true },
    "aux1":         { "us": 1000, "attached": true },
    "aux2":         { "us": 1500, "attached": true }
  },
  "flapsUs": 0,
  "imu":  { "attached": true, "available": true, "roll": 0.12, "pitch": -0.40, "yaw": 38.50 },
  "baro": { "attached": true, "available": true, "altitude": 0.05, "climb": 0.01 },
  "mag":  { "attached": true, "available": true, "heading": 41.9 },
  "gps":  { "attached": true, "available": true, "fix": 3, "numSV": 12, "lat": 55.750000, "lon": 37.610000, "alt": 150.0 },
  "airspeed": { "attached": true, "available": true, "ias": 14.2, "tas": 14.3, "dp": 123.4 },
  "autopilot": {
    "attached": true, "mode": 1, "modeName": "STABILIZE",
    "desiredRoll": 0.0, "desiredPitch": 0.0, "targetAlt": 0.0,
    "rollCorr": 0.0, "pitchCorr": 0.0, "throttleCorr": 0.0,
    "kpRoll": 5.000, "kiRoll": 0.500, "kdRoll": 0.500,
    "kpPitch": 5.000, "kiPitch": 0.500, "kdPitch": 0.500,
    "nav": { "gps": true, "home": true, "homeDist": 120, "homeBearing": 185,
             "course": 90, "targetCourse": 90, "speed": 14.3, "fence": false, "stall": false },
    "features": ["FLAPS"]
  }
}
```

- `attached` – objektet finns i bygget; `available` – sensorn
  svarar verkligen. Datafälten läggs till **bara** när `available: true`.
- `outputs.*.attached` – mikrokontrollern har tilldelat en LEDC-kanal och ett stift;
  om ett fysiskt servo är anslutet kan inte ses från mjukvaran (för att
  kontrollera pulsen, använd konsolen, kommandot `p`).
- `rollCorr`/`pitchCorr` – autopilotens slutliga kommando minus spakarna,
  µs. `throttleCorr` – autopilotens gas, % (0 medan gasen ligger
  hos piloten).
- `nav` – navigering: hempunkt, avståndet och bäringen till den, kursen och
  målkursen, den hastighet som används för navigering (pitotrör / GPS),
  geofencen, överstegring; `features` – de brytarfunktioner som är påslagna.

### `POST /api/setmode`

`{ "mode": 1 }` – numret på `AutopilotMode`: `0` MANUAL, `1` STABILIZE, `2`
AUTO_TAKEOFF, `3` ALT_HOLD, `4` ACRO, `5` CRUISE, `6` LOITER, `7` RTH, `8`
LAUNCH, `9` AUTO_LAND, `10` SOARING, `11` RESCUE. Läget gäller tills
piloten slår om lägesbrytaren.

### `POST /api/setpid`

`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }` – valfria av fälten `kpRoll`,
`kiRoll`, `kdRoll`, `kpPitch`, `kiPitch`, `kdPitch`; utelämnade behåller sina
tidigare värden.

Båda kommandona tillämpas av flygslingan i nästa cykel (se
[FreeRTOS-uppgifter](#freertos-uppgifter-och-styrslingan)).

### `GET /`

HTML-panelen: staplar för de 10 kanalerna, ARM/förbindelse, utgångarna,
sensorerna, lägesknappar, PID-formuläret. Den frågar `/api/status` var 200:e ms.

---

## Konsol och diagnostik

Portmonitorn – 115200, ”COM”-kontakten. Implementeringen är
`DebugConsole` och `DebugLogger` ([referens](reference/telemetry.md)). Tangenterna
verkar direkt, Enter krävs inte; kalibreringarna och `p` blockerar
slingan och är därför bara tillgängliga utan ARM.

| Tangent | Vad den gör |
|---|---|
| `h` / `?` | Huvudmeny |
| `l` | Menyn ”vad som ska skrivas till loggen” (kanaler, lägen, period) |
| mellanslag | Pausa loggen / återuppta |
| `s` | `printStatus()` för alla sensorer: data, räknare för bussfel, kalibreringar, kontrollen före flygning |
| `i` | Gyrokalibrering + kontroll före flygning (2 s orörligt) |
| `o` | Kalibrering av IMU-monteringen med tre positioner, sparas i NVS |
| `m` | Kompasskalibrering (15 s rotation), sparas i NVS |
| `p` | Självtest av utgångar: den verkliga pulsen på varje stift mot den förväntade |

Loggen är uppdelad i kanaler (`STAT`, `RC`, `OUT`, `ATT`, `AP`, `ALT`, `MAG`,
`GPS`, `IMU`, `SYS`), var och en med ett läge ”av / vid ändring / kontinuerligt”;
inställningarna lagras i NVS och skrivs när menyn stängs, bara utan
ARM. Som standard är `STAT` (vid ändring) och `SYS` (var 10:e s) påslagna:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (worst over 10 s) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

Formaten för alla kanaler finns i [referensen](reference/telemetry.md#debuglogger).

---

## Val av kort och stiftbeläggning

| Kommando | `board` | Makro | Status |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8 (`qio_opi`, 16 MB) | `BOARD_ESP32_S3` | **Huvudkort, standard.** Provat på bänken med alla sensorer |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | Den gamla prototypen, flugen med manuell styrning |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | För bänken, stiftbeläggningen har inte provats på hårdvara |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6: den fullständiga firmwaren + MAVLink + en svart låda på SD-kort; provad på ett naket kort ([nedan](#stm32h743)) |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | Samma sak på DevEBox H743: konsolen är USB CDC, flashning via DFU |

| Syfte | ESP32-S3 (bänk) | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| Skevroder vänster / höger | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| Höjdroder / ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| Sidroder | GPIO18 | – (inget stift) | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| Sensorernas I2C SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| OLED I2C SDA / SCL | GPIO1 / GPIO2 | – | – |
| SPI SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI CS ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS RX / TX | GPIO39 / GPIO40 (UART2) | GPIO9 ⚠️ / inget (UART0) | GPIO4 / GPIO17 (UART2) |
| Serial | UART0 → ”COM”-kontakten | USB-CDC | UART0 |

- **ESP32-S3 N16R8:** GPIO33–37 är upptagna av oktal PSRAM, 26–32 av
  flash, 19/20 av USB, 43/44 av Serial, 48 är RGB-lysdioden; 0/3/45/46 är
  strapping-stift.
- **ESP32-C3:** skevrodren på GPIO4/5 är omkastade jämfört med S3. Det
  finns inte tillräckligt med stift för hela uppsättningen: BMP388:s CS och GPS:ens RX sitter på
  strapping-stift, GPS:en har ingen TX (bara mottagning, ingen UBX-CFG). Detaljer finns
  i `Config.h`.

### STM32H743

STM32H743VIT6 (Cortex-M7 480 MHz, 2 MB flash, 1 MB RAM) kör den
**fullständiga firmwaren**: samma sensorer, autopilot, brytare, konsol och skärm
som på ESP32-S3, plus MAVLink-telemetri och en svart låda på SD-kort. Den
byggs, klarar cppcheck och alla native-tester av den gemensamma koden. På
hårdvara har **DevEBox H743-kortet utan sensorer** provats: uppstart,
konsolen via USB, SD-kortet, den svarta lådan –
[TESTING.md](TESTING.md#tester-på-stm32-kortet) – samt iBUS, ARM och PWM
till servona och motorn: styrning från sändaren i manuellt läge (på
video). Sensorerna kopplas nu in till STM32. **Huvudkortet är STM32H743 (DevEBox)**; ESP32-S3 är det tidigare huvudkortet, och det klarade bänken med alla sensorer.

- **HAL** – `include/hal/stm32/`: `Stm32Board` (samma API som `Esp32Board`,
  plus `telemetryUart()`), `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`,
  `Stm32ServoOutput` (hårdvaru-PWM från `HardwareTimer`, en timer för
  flera utgångar). I detalj – [reference/hal.md](reference/hal.md#implementering-för-stm32h743).
- **Inställningar och kalibreringar** – inte NVS utan en `KeyValueStore` i den sista
  flashsektorn (`include/storage/`, `hal/stm32/Stm32FlashStorage.h`).
  Projektets kod skriver fortfarande `#include <Preferences.h>`: i env `stm32h743`
  ligger katalogen `include/hal/stm32/compat/` på `-I`, och där finns en
  `Preferences` med samma API. Avbilden bär en CRC32: en
  korrupt (strömmen gick under raderingen) läses som tom. Flashskrivningen
  sker i en bakgrundsuppgift: radering av en sektor på 128 KB tar sekunder,
  men sektorn ligger i bank 2 medan koden körs från bank 1, och
  flyguppgiften avbryter bakgrundsuppgiften utan att stanna.
- **Uppgifter** – FreeRTOS från biblioteket STM32duino FreeRTOS, en enda kärna,
  preemption efter prioritet (`hal/Rtos.h`): `flight` (5) – flygslingan,
  MAVLink, logg, konsol; `oled` (1) och `storage` (1) – i bakgrunden;
  `bbox` (2) – skrivning av den svarta lådan till SD-kortet.
- **Den svarta lådan på SD-kortet** – SDMMC1, 4 bitar, 24 MHz
  (`hal/stm32/Stm32SdCard.h`, stiften i `src/stm32/sd_msp.cpp`). Kortet
  förblir ett vanligt FAT32: en i förväg skapad fil `BLACKBOX.BIN` ligger på det,
  firmwaren skriver råa block inuti den och rör inte själva filsystemet
  (`storage/Fat32File.h` är skrivskyddad). Förberedelse av kortet och
  hämtning – [BLACKBOX.md](BLACKBOX.md#sd-kort-stm32h743).
- **Telemetri** – MAVLink 2 på UART4 (`telemetry/MavlinkTelemetry.h`) i stället
  för Wi-Fi-panelen: QGroundControl / Mission Planner, byte av läge
  och PID från marken. I detalj –
  [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#markstation-wi-fi-panel-och-mavlink).
- **Stiftbeläggning** – blocket `BOARD_STM32H743` i `Config.h`, stiften valda
  bland de lediga på WeAct MiniSTM32H743VITx och kontrollerade mot
  STM32duinos tabeller:

| Syfte | STM32H743 | Kringutrustning |
|---|---|---|
| Skevroder vänster / höger | PA0 / PA1 | TIM2_CH1 / CH2 |
| Höjdroder / ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| Sidroder | PD14 | TIM4_CH3 |
| AUX1 (last) / AUX2 (kamera) | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX (TX – reserv) | PE7 (PE8) | UART7 |
| Sensorernas I2C SDA / SCL | PB11 / PB10 | I2C2 |
| OLED I2C SDA / SCL | PB9 / PB8 | I2C1 |
| SPI SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI CS IMU / barometer | PB12 / PD10 | GPIO |
| GPS RX / TX | PD9 / PD8 | USART3 |
| Radiomodem MAVLink RX / TX | PD0 / PD1 | UART4 |
| Summer | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **DevEBox H743 (MCUDEV)** – env `stm32h743-devebox`: samma kod, en egen
  kärnvariant, konsolen via USB-C som en virtuell COM-port (CDC) – ingen
  USB-UART behövs. Den första flashningen sker via USB genom den inbyggda
  bootloadern (DFU):
  1. Windows: installera WinUSB-drivrutinen för ”STM32 BOOTLOADER” en gång
     ([Zadig](https://zadig.akeo.ie): DFU in FS Mode → WinUSB → Install Driver).
  2. Anslut stiftet **BT0** (BOOT0) till **3V3** med en tråd, tryck på och
     släpp **RST**: kortet är i DFU-läge (DevEBox har ingen BOOT0-
     knapp).
  3. `pio run -e stm32h743-devebox -t upload` (`upload_protocol = dfu`).
  4. BT0-tråden kan tas bort – firmwaren startar av sig själv.

  Därefter behövs inte tråden: tangenten **`D`** i konsolen (från valfri
  meny, inte under ARM) startar om kortet i bootloadern: en markör i
  RAM → återställning → ett hopp in i systemminnet innan klockorna konfigureras
  (`src/stm32/bootloader.cpp`). Att hoppa direkt från körande firmware hänger sig
  på H7 – verifierat på kortet, därav de två stegen. En öppen konsol
  (USB CDC) behövs; om kortet inte svarar – RST med BT0-tråden
  på plats.
- **Ingångspunkt** – `src/stm32/main.cpp` (utesluten från ESP32-byggena via
  `build_src_filter`). Objekten är desamma som i `src/main.cpp`; i stället
  för `loop()` finns uppgifter, och `vTaskStartScheduler()` står i slutet av
  `setup()`.
- **Kortets första start:** `pio run -e stm32h743 -t upload` (ST-Link),
  monitorn på LPUART1 via en USB-UART; `b` – om sensorerna syns
  på bussarna, `s` – sensorstatus, `p` – pulser på utgångarna
  (ta av propellern), sedan sändaren och QGroundControl via
  radiomodemet.

---

## Hur man lägger till en ny sensor

### A) En annan krets i en befintlig kategori (IMU, barometer, kompass)

Den gemensamma koden är redan skriven i basklasserna – en kretsdrivrutin blir
liten:

1. Skapa `include/sensors/<kategori>/<Namn>_Sensor.h` och ärv från
   `ImuSensorBase` / `BarometerBase` / `MagnetometerBase`. Konstruktorn
   tar en `IRegisterDevice&` – drivrutinen vet inte om det är I2C
   eller SPI.
2. Implementera:
   - `begin()` – `device.begin()`, kontrollera kretsens ID, skriv registren,
     anropa `setAvailable(true/false)`;
   - IMU: `readSample()` (rå accel/gyro/temp i kretsens axlar),
     `accelLsbPerG()`, `gyroLsbPerDps()`, `temperatureC()`;
   - barometer: `isNewSampleReady()` (en klar-flagga eller helt enkelt `true`) och
     `readSample()` (tryck i Pa, temperatur i °C), avläsningsperioden
     anges i baskonstruktorn;
   - kompass: `readRaw()` (X/Y/Z i kretsens axlar) och
     `lsbPerMicroTesla()`, NVS-namnrymden för kalibreringen anges i
     baskonstruktorn.
3. Om kretsen behöver en dummybyte före data över SPI eller en särskild
   frekvens – lägg till en statisk fabrik `spiDevice(bus, cs)`, som den i
   `BMP388_Sensor`.
4. En gren i `SensorSelection.h`: `#define SENSOR_<KATEGORI>_<NAMN>`,
   `using Selected... = ...;` och `#define SELECTED_..._DEVICE(board) ...`
   (`I2cRegisterDevice(board.i2c(), address)` eller SPI-fabriken). `main.cpp`
   rörs inte när sensorn byts.
5. Kontrollera bygget med den nya sensorn utan att redigera filen – med en
   flagga: `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAMN>" pio run`,
   sedan alla tre miljöerna, sedan på hårdvara.

### B) En ny kategori

1. Datastrukturen och gränssnittet läggs i `SensorInterface.h`, efter förebild av
   `GpsSensor`/`GpsData`.
2. Om kategorin har gemensam logik (filter, kalibrering) – en basklass
   efter förebild av `BarometerBase`.
3. En nullbar pekare i konstruktorn för `Autopilot` (ingen sensor – inga effekter,
   snarare än en krasch) och fält i `GET /api/status` med ett
   `attached`/`available`-par.

### En ny buss eller kringutrustning

Ett nytt gränssnitt i `include/hal/`, en implementering i `include/hal/esp32/`
och i `include/hal/stm32/`, åtkomst via `IBoard`.

---

## Hur man lägger till ett nytt autopilotläge

1. Ett värde i `enum AutopilotMode` (`autopilot/AutopilotTypes.h`, före
   `MODE_COUNT`), ett namn och ett kort namn (upp till 5 tecken, för OLED-skärmen)
   i `AutopilotNames::mode()` / `modeShort()`.
2. En hanterare `run<Mode>()` och en gren i `Autopilot::runMode()`; de initiala
   målen (kurs, höjd, cirkelns mittpunkt) läggs i `initializeMode()`. Läget
   sätter `desiredRoll`/`desiredPitch` och anropar `stabilizeOrManual()`
   (utan IMU har piloten roderytorna) eller
   `stabilizeOrNeutral()` (utan IMU – neutralläge). Utan den nödvändiga
   sensorn – säkert beteende, inte en krasch. Integratorn ackumulerar bara när
   `armed`.
3. Gas: `throttleMode` (`PILOT` / `AUTO` / `AT_LEAST`) och
   `autoThrottlePct`, eller `autoThrottle()` – marschgasen från
   ratten / från pitotröret. `FlightController` ändras inte.
4. På sändaren – en enda rad i `config/Controls.h`
   (`Bind::mode(Channels::SWD, MODE_NEW)`). Panelen och MAVLink plockar upp
   läget efter dess nummer; för MAVLink – det närmaste ArduPlane-läget i
   `MavlinkModes::toCustomMode()` / `fromCustomMode()`.
5. Om läget behöver sensorer för ARM – `ArmingManager`.
6. Tester: reaktionen på varje sensor – `test/native/test_autopilot_modes`,
   flygning i sluten slinga – ett scenario i `test/native/test_sim` (flygplansmodellen
   `helpers/PlaneSim.h`, riggen `helpers/SimHarness.h`). Sedan – skrivbordet
   utan propeller: roderytorna måste reagera på lutning
   i riktning mot plant läge.
7. Ett avsnitt i [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

---

## Återkoppling (förarbete, inte ansluten)

`include/autopilot/feedback/` är nästa steg för autopiloten. **Varken
`FlightController`, `Autopilot` eller `main.cpp` inkluderar de här filerna:**
det finns ännu ingen prototyp för flygprov, och firmwaren fungerar utan
dem. De verifieras med en simulering i sluten slinga (`test/test_feedback/`)
direkt på kortet.

### Varför

I dag är `Autopilot` en PID på vinkeln: fel × förstärkning = roderyta. Den
vet inte vad det blev av det på flygplanet, och förstärkningarna är rätt
bara för en hastighet: vid låg fart är ytan svagare och PID:en
korrigerar för lite, vid hög fart korrigerar den för mycket. Återkopplingen sluter slingan över
**flygplanets respons**:

- ytan gav utslag men flygplanet roterar långsammare än det behövs – lägg
  på mer, tills det kommer dit;
- hur mycket roderyta som behövs mäts under flygning och räknas om med
  hastigheten;
- flygplanet roterar åt fel håll – tecknet är förväxlat, vänd det och
  kontrollera;
- vinkeln har rätats upp men farten sjunker – gas upp och nosen
  ned, innan flygplanet steglar;
- start och landning – i faser, utifrån vad sensorerna visar.

### Moduler

| Fil | Vad den gör |
|---|---|
| `FlightSnapshot.h` | Allt som återkopplingen vet om flygplanet under en cykel. Den enda indatan: modulerna läser inte sensorerna och RC direkt, så de kan köras på en simulering och på loggar |
| `FeedbackOutput.h` | Utdatan för en cykel: roderutslag per axel, om axeln är påslagen, axelns tecken, gas (satt / inte under), orsaken |
| `FeedbackConfig.h` | Alla konstanter (de flyttas till `Config.h` vid anslutning) |
| `SpeedEstimator.h` | Hastighet (pitotrör > GPS) och longitudinell acceleration från IMU:n: `dV/dt = g·(ax − sin θ)` – ”farten sjunker” syns även utan lufthastighetssensor |
| `AirborneDetector.h` | I luften / på marken: inlärning, ackumulering av integralen och sökandet efter överstegring är bara meningsfulla under flygning |
| `ControlEffectivenessEstimator.h` | Lär sig för varje axel modellen `ε = b·u(t−delay) + a·ω + c` med rekursiva minsta kvadrater |
| `AdaptiveRateController.h` | En kaskad vinkel → vinkelhastighet → vinkelacceleration → roderyta genom den inlärda modellen |
| `StallGuard.h` | Skydd mot fartförlust och överstegring |
| `TakeoffSequencer.h`, `LandingSequencer.h`, `PhaseTargets.h` | Start (från bana eller för hand) och landning i faser, utifrån sensorerna |
| `FeedbackSupervisor.h` | Allt tillsammans: ordningen inom en cykel, prioriteterna, `requestTakeoff()`/`requestLanding()`/`cancelPhase()`, `printStatus()`, anslutningsplanen |
| `FeedbackModules.h` | En enda include för allt |

### Hur det fungerar

**Roderytornas verkan.** Axelmodellen: vinkelacceleration
`ε = b·u + a·ω + c`. `b` är hur många °/s² 1 µs roderyta ger (tecknet är
responsens riktning), `a` är dämpning (luften bromsar rotationen;
utan den termen skulle skattningen av `b` gå mot noll under en jämn
rotation), `c` är ett konstant moment (tyngdpunkt, trim, propeller). En
rodrytas kraft ∝ ρV², så `b` lärs in vid en referenshastighet och
multipliceras med `(V/Vref)²`: flygplanet ökar farten – ytan ”blir
starkare” direkt utan ny inlärning. Den indikerade lufthastigheten från pitotröret
innehåller redan luftens densitet, så höjden tas med av sig
själv; utan lufthastighetssensor är skalan 1, och `b` lärs in
direkt.

Data tas över intervall på 20 ms: den genomsnittliga accelerationen över ett
intervall är skillnaden i gyro vid ändarna / längden, och den
motsvarar det genomsnittliga roderutslaget och vinkelhastigheten över samma
intervall (roderytan – med fördröjningen `RESPONSE_DELAY_MS`). Sedan passerar båda sidor
av ekvationen genom samma lågpassfilter på 2 Hz: sambandet
ändras inte, medan de höga frekvenserna, där modellen med ”ren fördröjning” ljuger
på grund av servots tröghet, tas bort. Inlärning är möjlig bara i luften
och bara medan roderytan ”skakas” (ett utslag ≥ `MIN_EXCITATION_US`
över ~0,3 s); pilotens spakar skakar också, så skattningen lär sig även
i MANUAL.

**Regulatorn.** Tre steg, axel för axel:

```
ω* = ANGLE_GAIN · (target − angle)              "nosen är 10° ned – höj den med 40°/s"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "roterar långsammare än det behövs – korrigera"
surface = (ε* − a·ω − c) / b                    genom den inlärda modellen
```

Integralen `I` lagras i °/s, inte i µs roderyta – så den förblir
korrekt när skattningen av `b` ändras. På marken är integralen
fryst (utom för kursen under startrullningen / utrullningen), och vid
ytans gräns ackumulerar den inte mot gränsen. En koordinerad sväng
tas med i beräkningen (om farten är känd): i en krängning behöver tippningen
`g·sin φ·tg φ / V` och giren `g·sin φ / V`.

**Axeltecken – bara på marken.** Under flygning stängs axlarna varken av
eller vänds: IMU-monteringen bestäms av kalibreringen `o` och
kontrollen vid start, och roderytornas riktningar av pilotens kontroll
före flygning. Indirekta tecken i luften (en avgång, en spinn, manövrer, byar) kan
lura, och en avstängd eller vänd axel i ett sådant ögonblick kostar flygplanet.
Om skattningen av `b` för en axel säkert är negativ är det bara en
varning i `reason` (”reagerar bakvänt på roderytan? kontrollera på
marken”); en negativ skattning går inte in i regulatorn – axeln
arbetar med a priori-modellen.

**Överstegringsskydd.** Två nivåer. *LowEnergy* – farten sjunker snabbt
med höjd nos, eller är nära överstegringen (< 1,25·Vs), eller höjdrodret
har förlorat verkan: gas ≥ 80 %, tippning ≤ 5°. *Stall* – farten är
under överstegringsfarten, nosen faller mot höjdrodret, vingen faller mot
skevrodren vid låg energi: full gas, nosen ned, krängning ≤ 10°, skevrodren
begränsade (ett stort skevroderutslag får vingspetsen att stegla). Åtgärderna släpps med
hysteres (fart ≥ 1,5·Vs). Vid förlorad förbindelse rörs inte gasen, och
alldeles nära marken (flaren, utrullningen) är skyddet avstängt –
landning är i sig en kontrollerad överstegring.

**Start.** `WaitThrottle` (motorn står still) → piloten gav gas
≥ 50 % → `GroundRoll` (full gas, vingarna plana, kursen hålls av
sidrodret och hjulet, höjdrodret fritt) → lättningsfart eller en tidsgräns
utan fartsensor → `Climb` (12°, full gas) → höjd 30 m →
`Complete`. För hand (`TAKEOFF_HAND_LAUNCH`), i stället för rullningen – `WaitLaunch`:
motorn startar först efter kastet (longitudinell acceleration ≥ 1g). Gasen
dragen tillbaka före lättning – avbryt.

**Landning.** `Approach` (gas 25 %, sjunkning 1 m/s – tippningen från
felet i vertikal hastighet, krängningen från piloten ≤ 20°) → höjd 2 m →
`Flare` (gas 0, sjunkningen dämpas till 0,3 m/s enligt samma regel) →
stöt i accelerometern eller ”lågt och roterar inte” → `Rollout` (kurs
med hjulet) → `Complete`. Pilotens gas ≥ 80 % – pådrag och ny inflygning. Flaren
behöver en avståndsmätare: barometern kan slå fel med en meter.

**Prioriteter** (`FeedbackSupervisor`): inte armerat > överstegringsskydd >
start/landning > lägets mål. Vid förlorad förbindelse avbryts faserna, och
stabiliseringen utför målen för glidflykten i failsafe.

### Simulering

`test/test_feedback/test_main.cpp` (på en dator: `pio test -e native -f test_feedback`) – en flygplansmodell (oberoende axlar,
servofördröjning och tröghet, roderytornas verkan ∝ V², dämpning ∝ V, konstanta
moment, lyftkraft via anfallsvinkeln från farten, överstegring, landningsställ med
ett styrhjul) och 10 scenarier:

| Scenario | Vad som kontrolleras |
|---|---|
| Återhämtning från 30° krängning / −15° tippning med ett konstant moment | Upprätning och ”fortsätt korrigera”: integralen hittar trimmet själv |
| Skakning ±15° vid 14 och 20 m/s, utan fartsensor | Skattningen av `b` konvergerar mot det sanna värdet och skalas om med farten |
| Ett förväxlat skevroder, piloten gungar vingarna i MANUAL | Skattningen av `b` är negativ → bara en varning, axeln stängs inte av |
| 30 s turbulens | Byar motverkas, krängningen går inte över 10° |
| Nosen 15° vid 20 % gas (med fartsensor och utan) | Farten sjunker inte till överstegring |
| Start från bana med propellerns reaktionsmoment | Faser, höjd, kurs under rullningen |
| Landning från 15 m | Faser, ingen gas nära marken, en mjuk sättning |
| Förlorad förbindelse under startrullningen; inte armerat; MANUAL | Avbrott, gasen rörs inte, roderytorna stannar hos piloten |

Modellen är grov – den kontrollerar logiken och tecknen, inte justeringen för en
specifik flygplansstomme.

```bash
pio test -e native -f test_feedback      # på en dator, på sekunder
pio test -e esp32-s3 -f test_feedback    # flashar testfirmwaren och kör den
pio run -t upload                        # återställ den vanliga firmwaren
```

### Anslutningsplan

1. `FlightController::update()` fyller, efter avläsning av sensorerna och beräkning av
   kommandona, i en `FlightSnapshot` och anropar
   `FeedbackSupervisor::update()`. Först – **skuggläge**: utdatan går
   bara till loggen (`printStatus()`) och till panelen, inte till
   roderytorna. Under flygning med manuell styrning måste skattningen av `b` för varje axel
   vara positiv och växa med farten.
2. På marken, flygplanet i händerna, STABILIZE: luta det – roderytorna
   motverkar.
3. En axel i taget: `deflectionUs` i stället för
   `Autopilot::getRollCorrection()` (först bara roll), sedan tippning.
4. Gas: `throttleOverridePercent`/`throttleFloorPercent` – efter
   `Autopilot::applyThrottle()`, före failsafe (failsafe går före
   allt).
5. Start/landning – på en ledig brytare; ta bort läget `AUTO_TAKEOFF` från
   `Autopilot`.
6. Konstanterna i `FeedbackConfig` – till `Config.h`; lufthastighetssensorn – en
   implementering av `AirspeedSensor` och en kategori i `SensorSelection.h`.

---

## Hur man lägger till ett nytt kort

1. `[env:<namn>]` i `platformio.ini` med en unik `-D BOARD_ESP32_<NAMN>`.
2. Ett block `#elif defined(BOARD_ESP32_<NAMN>)` i `Config.h` med alla
   stift, inklusive `PIN_I2C2_SDA/SCL` (−1 om det inte finns någon OLED). Räkna ut
   GPIO-budgeten i förväg: flash/PSRAM/USB/strapping.
3. Servoutgångarna behöver 5 LEDC-kanaler – varje ESP32 har dem. Om det inte finns
   något stift för sidrodret – `PIN_RUDDER = -1`, och utgången stängs helt
   enkelt av.
4. Ändra inte `default_envs` förrän kortet har provats på hårdvara;
   ange uttryckligen i commiten om stiftbeläggningen inte har provats.

---

## Kommandon för bygge, uppladdning och monitor

```bash
pio run                        # bygg standardkortet (stm32h743-devebox)
pio run -t upload              # ladda upp
pio device monitor             # monitor, 115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # kontrollera att alla kort byggs
```

- **ESP32-S3:** uppladdning och Serial går via ”COM”-kontakten (CH343).
  Om bryggan hänger sig (Windows svarar ”enheten fungerar inte” –
  det kan hända på grund av störningar från ESC:n) hjälper det att ansluta
  kabeln på nytt; du kan också ladda upp via ”USB”-kontakten (den inbyggda
  USB-JTAG): `pio run -t upload --upload-port <USB COM-port>`.
- Medan portmonitorn är öppen fungerar inte uppladdning till samma port.
- `lib_deps`: `olikraus/U8g2` (OLED) är det enda externa biblioteket.
- `test/` – i detalj i [`TESTING.md`](TESTING.md):
  - `pio test -e native -e native-stm32` – 387 tester på en dator (hårdvaruattrapper
    i `test/native/support/`), täckning – `gcovr`;
  - `pio test -e esp32-s3` – `test_feedback/` (återkopplingssimuleringen
    i sluten slinga) och `test_imu_orientation/` på själva kortet; var och en
    flashar en testfirmware, ladda efteråt upp den vanliga med
    `pio run -t upload`.
- Statisk analys: `pio check -e esp32-s3` (cppcheck), `pio check -e stm32h743` (cppcheck över
  `hal/stm32/` och `src/stm32/`) och `tools/clang-tidy.sh`
  (profilen `.clang-tidy`).

---

## Kända begränsningar

- **Autopiloten har inte provats under flygning.** På skrivbordet har tecknen
  verifierats live (lutning → korrigering mot plant läge), PID-förstärkningarna är
  startvärden.
- **STABILIZE är upprätning ovanpå spakarna**, inte ett ”vinkelläge”
  (FBWA) där spaken anger roll-/tippvinkeln. Piloten och
  autopiloten adderas.
- **Glidflykt vid förlorad förbindelse har inte provats under flygning.**
  Vinklarna `FAILSAFE_GLIDE_*` är startvärden; tippningen −3° är vald för
  en specifik flygplansstomme (nosen får varken stiga till överstegring eller dyka).
- **Horisonten.** Med monteringskalibreringen (`o`) – från den (NVS);
  accelerometerns nollförskjutning driver med temperaturen (~1–2° per 20 °C), om
  horisonten har ”drivit” – upprepa `o`. Utan den – läget vid start
  (slå på strömmen när flygplanet står plant).
- **Kompassens montering** anges fortfarande av `MAG_ROTATION_CW_DEG` (den
  positionsbaserade kalibreringen påverkar den inte).
- **Kompass:** kursen är utan lutningskompensation, avläsningens
  riktning har inte verifierats på ett monterat flygplan, och kalibreringen
  måste göras i själva flygplanet. Inget läge använder kursen ännu.
- **GPS** används inte för navigering; på ESP32-C3 är den bara för mottagning.
- **Återkopplingen (`autopilot/feedback/`) är inte ansluten** och har verifierats
  bara i simulering med en grov flygplansmodell. Alla siffror i
  `FeedbackConfig.h` som är märkta ”прикидка” (”grov uppskattning”) behöver förfinas på en verklig
  flygplansstomme; det finns ännu ingen lufthastighetssensor (utan den lärs roderytornas
  verkan in långsammare, och en överstegring syns bara genom
  inbromsningen).
- **Inte provat på hårdvara:** `ICM42688_Sensor` (anpassad till den gemensamma
  konventionen via `ImuSensorBase`), `BME280_Sensor` (Bosch-
  kompenseringen implementerades på nytt), BMP388 över SPI, `QMC5883L_Sensor`, GPS-
  inställningen via CFG-VALSET. Vid anslutning – startloggen, `s` i
  konsolen, tecken genom lutning.
- **I2C på ett kopplingsdäck fångar upp störningar** från ESC/motor (enstaka
  fel syns via `s`). Drivrutinerna överlever dem, men i flygplanet
  bör I2C-trådarna vara korta och hållas borta från kraftledningarna.
- **ESP32Servo används inte.** Version 3.2.1 på ESP32-S3 fördelar
  servona över MCPWM och förväxlar i `attachPin()` MCPWM-enhetens nummer med
  timerns nummer: GPIO6/7 gav signalen från GPIO4/5 (ESC:n
  styrdes av den högra spaken). Utgångarna skrevs om på LEDC; ta tillbaka
  biblioteket bara efter kontroll med `p`.
- **ESC:n är 50 Hz PWM**, firmwaren har ännu inget läge för kalibrering av
  gasområdet.
- **Webbpanel:** åtkomstpunktens lösenord är svagt, och kommandon
  tas emot även under flygning. Det är ett verktyg för bänken och fältet, inte för
  flygning.
- **Prototypens mekanik:** den första prototypen flög, ett svagt motorfäste och
  otillräcklig styvhet i vingen upptäcktes.
- **Licensen är OpenPlane License** ([LICENSE](LICENSE.md)): MIT med
  obligatorisk angivelse av upphovspersonen, förbud mot militär användning och förbud mot avsiktlig skada på människor och
  egendom utan deras skriftliga medgivande. Lägg inte till andra licenshuvuden
  i filerna och ta inte bort upphovspersonens namn.

---

## Hur man gör ändringar

- **Små commits:** ett logiskt steg – en commit.
- **Tester och analys före en commit:** `pio test -e native -e native-stm32`,
  `pio check -e esp32-s3`, `pio check -e stm32h743`, `tools/clang-tidy.sh` –
  allt grönt ([`TESTING.md`](TESTING.md)).
- **Bygg alla kort** efter ändringar i gemensam kod – STM32H743 är huvudkortet, men ESP32-S3, C3 och 38-pin får inte heller gå sönder; före en release – `tools/build_matrix.sh` (alla kort × alla sensorer).
- **Kontrollera på hårdvara det som kan kontrolleras:** tecken – genom lutning, utgångar – med
  kommandot `p`, förbindelsen – genom att stänga av sändaren.
- **Hitta inte på API:er.** Konsultera ramverkets källkod i
  `~/.platformio/packages/framework-arduinoespressif32/` (Arduino core 2.0.x)
  – internet beskriver ofta version 3.x med ett annat API (till exempel
  LEDC).
- **Försköna inte statusen.** Inte provat på hårdvara – säg det.
- **Ett lager får inte veta mer än det ska.** Om en lägre klass plötsligt
  behöver en högre måste logiken flyttas upp i `FlightController`.
- **När ett datakontrakt ändras** (`FlightOutputState`, `ControlCommand`,
  `ImuData`, JSON för `/api/status`) – uppdatera alla konsumenter i samma
  commit.
