# DEVELOPER_GUIDE.md — guide du développeur d’OpenPlaneProject

> 🌐 Cette page est la traduction de l’[original en russe](../../DEVELOPER_GUIDE.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels.

Une carte technique du firmware : quel fichier est responsable de quoi, comment les données circulent du récepteur et des capteurs jusqu’aux servos, quelles conventions de signes tiennent toute la chaîne ensemble, comment l’API web est organisée et comment étendre le projet. Il s’adresse à un développeur qui écrit du C++ et veut s’orienter rapidement dans ce dépôt (la branche `main`), et non à l’apprentissage des bases du langage ou de PlatformIO.

Pour une vue d’ensemble du projet et l’état du prototype, voir [`../README.md`](README.md) ; pour savoir quoi brancher où et comment voler, [`PILOT_GUIDE.md`](PILOT_GUIDE.md) ; pour les projets, [`ROADMAP.md`](ROADMAP.md). Ici, il n’y a que du code. L’architecture complète (couches, tâches, machines à états) se trouve dans [`ARCHITECTURE.md`](ARCHITECTURE.md), la référence de chaque classe dans [`reference/`](reference/README.md) et les tests dans [`TESTING.md`](TESTING.md).

> Le projet est en développement actif. Le banc avec l’ESP32-S3 est monté et vérifié avec tous les capteurs, mais **le pilote automatique n’a pas encore été éprouvé en vol** — c’est signalé là où cela concerne un module précis. Si vous doutez de ce que fait le code, relisez le code source, pas le document.

---

## Sommaire

1. [Architecture des couches](#architecture-des-couches)
2. [Tâches FreeRTOS et boucle de contrôle](#tâches-freertos-et-boucle-de-contrôle)
3. [Référence des fichiers](#référence-des-fichiers)
4. [Convention de signes : de l’IMU au servo](#convention-de-signes-de-limu-au-servo)
5. [Carte des canaux RC, ARM et failsafe](#carte-des-canaux-rc-arm-et-failsafe)
6. [Données des capteurs](#données-des-capteurs)
7. [Détail de FlightController::update()](#détail-de-flightcontrollerupdate)
8. [API HTTP du tableau de bord web](#api-http-du-tableau-de-bord-web)
9. [Console et diagnostic](#console-et-diagnostic)
10. [Choix de la carte et brochage](#choix-de-la-carte-et-brochage)
11. [Comment ajouter un nouveau capteur](#comment-ajouter-un-nouveau-capteur)
12. [Comment ajouter un nouveau mode du pilote automatique](#comment-ajouter-un-nouveau-mode-du-pilote-automatique)
13. [Rétroaction (ébauche, non branchée)](#rétroaction-ébauche-non-branchée)
14. [Comment ajouter une nouvelle carte](#comment-ajouter-une-nouvelle-carte)
15. [Commandes de compilation, de téléversement et de moniteur](#commandes-de-compilation-de-téléversement-et-de-moniteur)
16. [Limitations connues](#limitations-connues)
17. [Comment apporter des modifications](#comment-apporter-des-modifications)

---

## Architecture des couches

Presque toutes les classes vivent dans des en-têtes répartis dans les dossiers `include/<couche>/`. Chaque en-tête inclut lui-même ce qu’il utilise (`#include "config/Config.h"`, `"hal/II2CBus.h"`, ... — chemins à partir de `include/`). `src/main.cpp` est l’unique point d’assemblage (composition root) : il crée tous les objets, les relie et exécute `setup()`/`loop()`. Les dépendances sont à sens unique — une couche inférieure ne sait rien d’une couche supérieure.

```
include/
├── config/      Config.h (broches, tous les réglages), Channels.h (noms des canaux),
│                Controls.h (ce que fait chaque interrupteur — une ligne par canal)
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (périphérique à registres au-dessus d’I2C/SPI), Rtos
│   ├── esp32/   Esp32Board + enveloppes autour de Wire/SPI/HardwareSerial/LEDC
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (réglages en flash au lieu de la NVS)
├── storage/     KeyValueStore, KvPreferences — stockage des réglages sans NVS
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  ébauche de la rétroaction — NON branchée (voir la section ci-dessous)
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (un tube fait de deux baromètres)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — firmware de l’ESP32 (S3, C3, 38 broches)
src/stm32/main.cpp  — firmware de la STM32H743 (tâches FreeRTOS, MAVLink)
```

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION  src/main.cpp / src/stm32/main.cpp — montage des objets   │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION  control/FlightController — ordre des opérations          │
│ TELEMETRY     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ CONTROL             │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12 modes   │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ navigation, PID    │  │ RcChannelState        │
│ ThrottleManager     │  │ PilotSwitches         │  │ RcInput               │
│ ArmingManager       │  └──────────┬────────────┘  └───────────────────────┘
│ FlightOutputs       │             │ ImuSensor* / BarometerSensor* / ...
└─────────┬───────────┘             ▼
          │           ┌─────────────────────────────────────────────────┐
          │           │ SENSORS                                          │
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

Les règles qui gardent l’architecture propre :

- La **HAL** est la seule couche autorisée à connaître un MCU précis (`Wire`, `SPI`, `HardwareSerial`, `ledc*`). Tout ce qui est au-dessus ne travaille qu’avec des interfaces. Passer à un autre MCU, c’est un nouveau `hal/<mcu>/<Mcu>Board.h` ; le reste du code ne change pas (un exemple : `hal/stm32/` pour la STM32H743).
- **Les pilotes de capteurs ne connaissent pas le bus.** Ils reçoivent un `IRegisterDevice&` — le périphérique I2C avec son adresse ou SPI avec son CS est créé dans `SensorSelection.h`. Un même `BMP388_Sensor` fonctionne aussi bien en I2C qu’en SPI.
- **Ce qui est commun vit dans les classes de base.** Étalonnage, rotation des axes, signes, filtre d’orientation, altitude et vitesse verticale, stockage de l’étalonnage de la boussole, comptage des erreurs de bus — dans `ImuSensorBase`/`BarometerBase`/`MagnetometerBase`. Le pilote d’une puce ne contient que les registres et les formules de la fiche technique.
- **RC et Outputs** ne savent rien de l’avion : octets iBUS → canaux, valeurs PWM → sorties.
- **Control et Autopilot** sont de la logique sur des données, sans UART, PWM ni Wi-Fi. Le temps, là où il en faut (volets), est passé en paramètre.
- **Coordination** (`FlightController`) est la seule classe qui voie plusieurs couches inférieures à la fois et décide de l’ordre des opérations.
- **Application** (`main.cpp`) est le seul endroit où `Esp32Board`, les périphériques et les capteurs sont créés et où tout est relié à la main, sans framework d’injection de dépendances.

---

## Tâches FreeRTOS et boucle de contrôle

| Où | Quoi | Période |
|---|---|---|
| Cœur 1, `loop()` (le loopTask d’Arduino) | `applyPendingCommands()` → `FlightController::update()` → `DebugLogger::update()` → `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms (500 Hz), `vTaskDelayUntil` |
| Cœur 0, la tâche `web` | `WebServer::handleClient()` | toutes les 2 ms |
| Cœur 0, la tâche `oled` | dessin du SSD1306 via le second bus I2C | 200 ms |
| Cœur 0 | la pile Wi-Fi d’ESP-IDF | — |

- La période de la boucle est tenue par `vTaskDelayUntil`, et non par un `delay(2)` après le travail — la fréquence ne dépend pas de la durée du cycle. Après un long blocage (un étalonnage lancé depuis la console), le décompte repart de zéro et les cycles manqués ne sont pas rattrapés en rafale.
- Sur le banc (ESP32-S3, tous les capteurs) : 500 Hz, en moyenne ~0,7 ms de travail par cycle, ~1,4 ms pour le pire cycle. Cela s’imprime toutes les 10 s dans une ligne `SYS:`.
- Le délai d’attente d’une transaction I2C est de 5 ms (celui de Wire par défaut est de 50 ms) : une transaction bloquée par une perturbation n’arrête pas la boucle longtemps.
- **Séparation des données entre tâches.** Le web et l’OLED ne font que *lire* l’état (`FlightController`/`Autopilot`/`LoopStats`) — ce sont des champs isolés de 16/32 bits, donc, dans le pire des cas, on voit les valeurs de cycles voisins. Les *commandes* du tableau de bord (`setmode`/`setpid`) ne sont pas appliquées directement depuis la tâche web : elles sont placées dans une « boîte aux lettres » sous `portMUX` et reprises par la boucle de vol dans `WebDebugServer::applyPendingCommands()`.
- `Serial` (UART0 → le pont CH343 → le connecteur « COM ») avec un tampon d’émission de 4 Ko : une trame de débogage (~600 caractères) ne bloque pas la boucle pendant son envoi.

---

## Référence des fichiers

### `config/`

| Fichier | Rôle |
|---|---|
| `Config.h` | Toutes les broches (un bloc par carte : `BOARD_ESP32_S3/C3/CLASSIC`, `BOARD_STM32H743`) et les réglages : iBUS et perte de signal ; débattements des gouvernes ; volets ; inversion des servos ; montage de l’IMU et de la boussole ; ARM ; failsafe (RTH ou plané) ; le tube de Pitot (`PITOT_*`) ; tous les nombres des modes et des fonctions du pilote automatique ; la boucle ; Wi-Fi ; MAVLink ; débogage |
| `Channels.h` | Noms des canaux : `AILERON`, `ELEVATOR`, `THROTTLE`, `RUDDER`, `ARM`, `SWB`, `SWC`, `SWD`, `VRA`, `VRB` |
| `Controls.h` | Le tableau `BINDINGS` : ce que fait chaque interrupteur et chaque potentiomètre, une ligne par canal, vérifications par `static_assert` |

### `hal/`

| Fichier | Rôle |
|---|---|
| `IBoard.h` | Le point d’entrée vers le matériel : `i2c()`, `displayI2c()` (un second bus pour l’écran, peut valoir `nullptr`), `spi()`, `rcUart()`, `gpsUart()`, `telemetryUart()` (MAVLink, peut valoir `nullptr`), `servo(ServoChannel::*)` (7 sorties avec AUX1/AUX2), `setBuzzer()` |
| `Rtos.h` | Les tâches FreeRTOS de la même façon sur l’ESP32 (cœur 0) et sur la STM32 (priorités), le tas libre |
| `II2CBus.h` | Le bus I2C : primitives à la forme de `Wire` + les utilitaires `writeRegister()`, `readRegisters()` (vérifie qu’exactement `count` octets sont arrivés), `readRegister()`, `probe()` |
| `ISpiBus.h`, `IUartPort.h`, `IServoOutput.h` | SPI, UART, une sortie PWM (`measurePulseUs()` — diagnostic de l’impulsion réelle) |
| `RegisterDevice.h` | `IRegisterDevice` — « un ensemble de registres de 8 bits » ; `I2cRegisterDevice` (adresse), `SpiRegisterDevice` (CS, fréquence, octets factices avant les données) |
| `esp32/Esp32Board.h` | L’implémentation d’`IBoard` : `Wire` (capteurs), `Wire1` (l’écran, si la puce a deux contrôleurs I2C), `SPI`, deux `HardwareSerial`, 5 canaux LEDC |
| `esp32/Esp32I2CBus.h` | `II2CBus` au-dessus de n’importe quel `TwoWire`, délai d’attente de 5 ms |
| `esp32/Esp32ServoOutput.h` | PWM via LEDC : 50 Hz, 14 bits ; broche −1 — la sortie n’est pas câblée. La bibliothèque ESP32Servo n’est pas utilisée — voir les [limitations](#limitations-connues) |
| `esp32/Esp32SpiBus.h`, `esp32/Esp32UartPort.h` | Fines enveloppes au-dessus de `SPI` et de `HardwareSerial` |
| `stm32/*` | STM32H743 : `Stm32Board` (+ l’UART4 du modem radio), les bus, les timers PWM, `Stm32FlashStorage` (réglages dans un secteur de la flash, écrits par une tâche de fond), `compat/Preferences.h` |

### `storage/`

| Fichier | Rôle |
|---|---|
| `KeyValueStore.h` | Une image « espace/clé → octets » avec CRC32 en RAM, au-dessus de n’importe quel support (`IFlashStorage`) ; une valeur identique n’est pas réécrite |
| `KvPreferences.h` | L’API `Preferences` de l’ESP32 au-dessus de `KeyValueStore` |

### `rc/`

| Fichier | Rôle |
|---|---|
| `RcChannelState.h` | Un instantané des 10 canaux |
| `RcInput.h` | `clamp()`, `centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS → canaux : trame de 32 octets, CRC, la valeur du canal correspond aux 12 bits de poids faible (`& 0x0FFF`) ; `isSignalLost()` = pas de trames (ou aucune encore) ∥ la valeur de failsafe des gaz ; compteurs de trames |

### `control/`

| Fichier | Rôle |
|---|---|
| `ControlCommand.h` | L’ordre donné aux gouvernes en signes physiques — la langue commune des manches, du pilote automatique et du mixeur |
| `ControlMixer.h` | `fromSticks(rc)` → `ControlCommand` ; `updateFlaps(cible, now)` ; `mix(command)` → PWM avec inversion des servos ; flaperons : les ailerons `flaps ± roll` (le moins, un aérofrein) |
| `FlapsController.h` | Sortie et rentrée progressives des volets, le temps est passé en paramètre |
| `ThrottleManager.h` | Les gaz issus du manche ; en cas de perte de signal — `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | ARM par l’interrupteur SwA (une transition OFF→ON avec les gaz en bas + les vérifications des capteurs du mode), DISARM instantané |
| `FlightOutputState.h` | Les PWM voulus : `aileronLeft`, `aileronRight`, `elevator`, `rudder`, `throttle`, `aux1` (charge utile), `aux2` (caméra) |
| `Beeper.h` | Le buzzer : par la fonction `BEEPER` ou « modèle perdu » au sol |
| `FlightOutputs.h` | Le tableau des sorties (`outputInfo()` : clé, nom, broche, caractère obligatoire, champ d’état) et tout ce qui se fait par-dessus en boucle : `begin()`, `write()`, `setFailsafe()`, état, `printPulseSelfTest()` |
| `FlightController.h` | L’ordre des opérations à chaque cycle, la perte de signal (`applyLinkLoss()`), les accesseurs pour la télémétrie |

### `autopilot/`

| Fichier | Rôle |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode` (12 modes), `Feature`, `Knob`, `PilotInputs`, noms |
| `ControlBinding.h` | `Binding`, les fabriques `Bind::modes/mode/feature/knob`, les vérifications `BindingCheck` |
| `PilotSwitches.h` | Le tableau des affectations → le mode, les fonctions et les potentiomètres de chaque cycle ; la disposition à la mise sous tension |
| `Autopilot.h` | 12 modes, failsafe RTH/plané, géobarrière, point de départ, coordination du virage, auto-trim ; `update(armed, linkLost, gaz, manches)` → `getCommand()`, `applyThrottle()` |
| `Navigation.h` | `Geo` (distance, relèvement, décalage), `Guidance` (roulis vers un cap, le champ de vecteurs du cercle) |
| `AltitudeSpeedController.h` | Tangage pour l’altitude, gaz pour la vitesse air (TECS-lite) |
| `LaunchController.h`, `SoaringController.h` | Les machines à états du lancer à la main et du vol à voile |
| `AutoTrim.h` | Auto-trim, sauvegardé dans la NVS/flash |
| `PidController.h` | PID : D sur la vitesse du capteur (gyroscope, variomètre), anti-windup, intégrateur gelé sans ARM |
| `feedback/*` | **Ébauche, non connectée :** rétroaction adaptative, décollage et atterrissage — voir [Rétroaction](#rétroaction-ébauche-non-branchée) |

### `sensors/`

| Fichier | Rôle |
|---|---|
| `SensorInterface.h` | Les interfaces `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` et les structures de données |
| `SensorSelection.h` | Quelle puce est compilée (`#define SENSOR_*`, modifiable par un drapeau de compilation) et sur quel bus elle se trouve (`SELECTED_*_DEVICE(board)`) |
| `SensorMounting.h` | Rotation des axes de la puce vers les axes de l’avion (0/90/180/270° dans le sens horaire) — pour la boussole et pour une IMU sans calibration de montage |
| `imu/ImuOrientation.h` | Montage de l’IMU sous forme de matrice « axes de la puce → axes de l’avion » : à partir de `IMU_ROTATION_CW_DEG` ou de trois poses (à plat, nez en haut, aile droite en bas) avec vérification ; sauvegardé dans la NVS |
| `imu/ImuSensorBase.h` | Ce qui est commun aux IMU : calibration du gyroscope + vérification avant vol (immobilité, 1g, le « haut » correspond au montage), calibration du montage (`calibrateOrientation()`), échelle, rotation, signes aéronautiques, erreurs du bus |
| `imu/AttitudeEstimator.h` | Filtre complémentaire de roulis/tangage, intégrale du lacet |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500 (puce reconnue par WHO_AM_I) : ±2000°/s, ±16g, DLPF ~41 Hz, 1 kHz. **Sur le banc** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P : ±2000°/s, ±16g, 1 kHz, filtre UI de 50 Hz. Non testé sur le matériel |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X : ±2000°/s, ±16g, 960 Hz, LPF1/LPF2 ; I2C 0x6A/0x6B ou SPI. Non testé sur le matériel |
| `imu/ICM45686_Sensor.h` | ICM-45686 : ±2000°/s, ±16g, 1,6 kHz, filtre passe-bas via les registres indirects IPREG ; I2C 0x68/0x69 ou SPI. Non testé sur le matériel |
| `baro/BarometerBase.h` | Ce qui est commun aux baromètres : interrogation des seuls nouveaux échantillons, altitude, vitesse verticale via un filtre passe-bas, calibration de la base, erreurs |
| `baro/BMP388_Sensor.h` | BMP388 en I2C ou en SPI (avec l’octet factice du SPI), compensation Bosch, lecture sur l’indicateur de donnée prête. **Sur le banc (I2C)** |
| `baro/BME280_Sensor.h` | BME280/BMP280, compensation Bosch §8.1. Non testé sur le matériel |
| `baro/SPL06_Sensor.h` | SPL06-001 : coefficients et formules de la fiche technique, 32 Hz ×16 ; I2C 0x76/0x77 ou SPI. Non testé sur le matériel |
| `baro/BMP581_Sensor.h` | BMP581 : la séquence du BMP5_SensorAPI, 16×/2×, IIR ; I2C 0x46/0x47 ou SPI ; sert à la fois de baromètre principal et de tube de Pitot. Non testé sur le matériel |
| `mag/MagnetometerBase.h` | Ce qui est commun aux boussoles : interrogation à 50 Hz, calibration hard-iron dans la NVS, rotation des axes, cap, erreurs |
| `mag/QMC5883P_Sensor.h` | QMC5883P, 0x2C. **Sur le banc** |
| `mag/QMC5883L_Sensor.h` | QMC5883L, 0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309, 0x7C : ±8 G, 200 Hz. Non testé sur le matériel |
| `gps/UbloxM10_Gps.h` | u-blox M10 : configuration par CFG-VALSET (115200 bauds, 10 Hz, NAV-PVT, sans NMEA), analyse de NAV-PVT. Non connecté sur le banc |
| `airspeed/AirspeedSensor.h` | L’interface du capteur de vitesse air : pression différentielle, IAS, TAS, masse volumique |
| `airspeed/PitotDualBaroAirspeed.h` | Le tube de Pitot artisanal : un BMP581 dans le tube + un baromètre dans le fuselage ; zéro au sol, filtre passe-bas, masse volumique d’après la pression statique, détection de panne |

### `telemetry/` et l’application

| Fichier | Rôle |
|---|---|
| `DebugLogger.h` | Journal par canaux (`LogSettings.h`) : chacun a sa propre ligne, sa propre tolérance aux rebonds et son mode ; reste muet tant que le menu est ouvert |
| `DebugConsole.h` | Un menu texte dans le moniteur du port (`h`) et des raccourcis clavier (`l`/espace/`s`/`i`/`o`/`m`/`p`/`b`) ; écrit les réglages du journal dans la NVS à la sortie du menu et seulement sans ARM |
| `LogSettings.h` | Les canaux du journal (STAT, RC, OUT, ATT, AP, ALT, MAG, GPS, IMU, NAV, SYS) et leurs modes : désactivé / en cas de changement / en continu ; sauvegardés dans la NVS |
| `WebDebugServer.h` | Le point d’accès, les routes, le JSON `/api/status`, la boîte aux lettres des commandes ; sa propre tâche sur le cœur 0 |
| `WebDashboardPage.h` | Le HTML/JS du tableau de bord dans un seul littéral ; les lignes des canaux, des sorties et des capteurs sont construites par le navigateur à partir du JSON |
| `OledDisplay.h` | SSD1306 via U8g2 au-dessus d’`II2CBus`, sa propre tâche (`Rtos`) |
| `MavlinkCodec.h`, `MavlinkTelemetry.h` | MAVLink 2 pour QGroundControl / Mission Planner : trames, flux, paramètres PID, changement de mode depuis le sol |
| `LoopStats.h` | Fréquence, temps de cycle moyen et pire temps par seconde (OLED) et le pire depuis la dernière lecture (`takePeakUs()`, ligne SYS) |
| `src/main.cpp` | ESP32 : création des objets, `setup()`, `loop()` avec `vTaskDelayUntil` |
| `src/stm32/main.cpp` | STM32H743 : les mêmes objets, MAVLink, la boîte noire sur carte SD, les tâches `flight`/`storage`/`oled`/`bbox` |
| `src/stm32/sd_msp.cpp`, `src/stm32/bootloader.cpp` | STM32H743 : broches et horloges de SDMMC1 pour `HAL_SD_Init` ; la touche `D` de la console — redémarrage dans le chargeur USB DFU |

---

## Convention de signes : de l’IMU au servo

Un seul système de signes pour toute la chaîne : ainsi le manche et le pilote
automatique déplacent forcément les gouvernes dans le même sens, et le sens
de chaque servo est défini à un seul endroit.

**1. Axes du capteur → axes de l’avion.** `ImuSensorBase` fait tourner les
axes de la puce avec la matrice `ImuOrientation` (body = R · chip) jusqu’aux
axes de l’avion : X vers le nez, Y vers la gauche, Z vers le haut. La matrice
provient :

- de la **calibration du montage** (la commande `o`, sauvegardée dans la NVS)
  — la carte peut être posée dans n’importe quelle orientation. Trois poses :
  « à plat » donne l’axe Z (et l’horizon : le décalage de zéro de
  l’accéléromètre y est inclus), « nez en haut » donne l’axe X (la part du
  « haut » perpendiculaire à Z), « aile droite en bas » donne l’axe Y. Le nez
  de l’étape 2 et le nez de l’étape 3 (Y × Z) doivent coïncider à ~25° près,
  sinon le pilote a incliné dans le mauvais sens : la calibration est
  rejetée ; le résultat est la moyenne des deux estimations. Vérifié sur 300
  montages aléatoires (`test/test_imu_orientation`, erreur < 0,1°) ;
- sinon, de `Config::IMU_ROTATION_CW_DEG` (carte avec la puce vers le haut ;
  la valeur indique où pointe l’axe X de la *puce* si le nez est à « 12
  heures »), et l’horizon est la pose au moment de la mise sous tension.

À chaque calibration du gyroscope (mise sous tension, `i`) a lieu une
**vérification avant vol** : bruit du gyroscope < 0,5 °/s (immobilité ; au
repos ~0,08), |a| ≈ 1g, le « haut » à moins de 45° de celui qui est
sauvegardé (la carte n’a pas été déplacée). En cas d’échec,
`ImuSensor::getPreflightProblem()` ≠ nullptr : `ArmingManager` n’arme pas les
modes stabilisés et `Autopilot::imuReady()` = false (corrections nulles dans
tous les modes, y compris le plané en cas de perte de liaison).

> Sur le GY-521 actuel (un clone avec MPU6500), la puce est soudée tournée de
> 90° par rapport aux flèches imprimées : la flèche X de la sérigraphie = l’axe
> Y de la puce. C’est pourquoi, sans calibration du montage,
> `IMU_ROTATION_CW_DEG = 90`. Vérification après tout déplacement : nez en
> haut → P augmente vers le positif, aile droite en bas → R vers le positif.

**2. Angles et vitesses (`ImuData`) — signes aéronautiques :**

| Grandeur | « + » signifie |
|---|---|
| `roll`, `gyroX` | aile droite en bas |
| `pitch`, `gyroY` | nez en haut |
| `yaw`, `gyroZ` | nez à droite (sens horaire vu de dessus) |

**3. L’ordre (`ControlCommand`, µs de débattement, ±500 = course complète) :**

| Champ | « + » signifie | Depuis le manche |
|---|---|---|
| `roll` | roulis à droite (aileron droit en haut, gauche en bas) | CH1 : 2000 = à droite |
| `pitch` | nez en haut (profondeur en haut) | CH2 avec le signe inverse : 2000 = vers l’avant = nez en bas |
| `yaw` | nez à droite (direction et roue de nez à droite) | CH4 : 2000 = à droite |
| `flaps` | volets en bas (les deux ailerons en bas) | SwB (CH6) : 0 ou `FLAPS_DEPLOYED_US`, progressivement en `FLAPS_TRANSITION_MS` |

Le PID calcule `erreur = cible − réel` : roulis à droite (roll > 0) → ordre
de roulis négatif → l’avion se remet à plat. Les corrections du pilote
automatique s’ajoutent à l’ordre des manches **avant** le mixeur, avec les
mêmes signes.

**4. Ordre → PWM.** `ControlMixer::mix()` calcule le débattement du bord de
fuite de chaque surface (ailerons : en bas = « + », gauche = `flaps + roll`,
droit = `flaps − roll` ; profondeur : en haut = « + » ; direction : à droite
= « + ») et le convertit en PWM `1500 ± débattement`, en inversant le signe
pour les servos avec `Config::*_REVERSED = true`. Les valeurs par défaut
reproduisent le comportement antérieur du micrologiciel pour les manches. La
vérification sur l’avion monté figure dans la liste de contrôle avant vol du
[`PILOT_GUIDE.md`](PILOT_GUIDE.md). L’inversion doit être modifiée dans
`Config.h`, **et non sur la radiocommande** — sinon le manche et le pilote
automatique divergeront.

---

## Carte des canaux RC, ARM et failsafe

La source est `include/config/Channels.h`. Radiocommande FS-i6 (10 canaux,
mode 2) + récepteur FS-iA6B, iBUS 115200.

| Canal | Élément de la radiocommande | Nom | Rôle |
|---|---|---|---|
| CH1 | manche droit ←→ | `AILERON` | Roulis |
| CH2 | manche droit ↑↓ | `ELEVATOR` | Tangage |
| CH3 | manche gauche ↑↓ | `THROTTLE` | Gaz, course complète ; < 950 = failsafe du récepteur |
| CH4 | manche gauche ←→ | `RUDDER` | Direction + roue directrice (un seul servo) |
| CH5 | SwA | `ARM` | ≥ 1750 = ARM (sur la FS-i6 c’est l’interrupteur vers le bas, vers soi) |
| CH6 | SwB | `SWB` | par défaut, les volets (≥ 1750 — sortis) |
| CH7 | SwC (3 positions) | `SWC` | par défaut, le mode : < 1250 MANUAL, 1250–1749 STABILIZE, ≥ 1750 AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | par défaut, RTH |
| CH9 | VrA | `VRA` | par défaut, la force de la stabilisation |
| CH10 | VrB | `VRB` | par défaut, la vitesse de croisière |

CH6–CH10 s’affectent en une seule ligne dans `include/config/Controls.h`
([AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#affecter-une-fonction-en-une-seule-ligne)).

**ARM** (`ArmingManager`) : l’interrupteur passe de OFF à ON, les gaz sont
sous `THROTTLE_LOW_US` et les vérifications des capteurs du mode courant sont
passées. Sinon, refus avec la raison indiquée sur Serial ; il faut un nouveau
cycle OFF→ON. Allumer la carte avec l’interrupteur déjà sur ON n’arme pas.
OFF — DISARM immédiat. Tant que l’appareil n’est pas armé, les gaz envoyés à
l’ESC sont forcés à `PWM_MIN`.

**Perte de liaison** (`IBusReceiver::isSignalLost()`) :

1. Pas de trames pendant plus de `RX_TIMEOUT_US` (500 ms) — fil coupé ou
   alimentation du récepteur perdue. Avant la première trame après la mise
   sous tension, la liaison est aussi considérée comme perdue : les valeurs
   par défaut des canaux (toutes à 1500) ne sont pas prises pour des ordres de
   la radiocommande.
2. Gaz < `RX_FAILSAFE_THROTTLE_US` (950) — le failsafe réglé dans la
   radiocommande. **Quand la radiocommande est perdue, le FS-iA6B ne cesse pas
   d’émettre des trames** : il répète les dernières valeurs (vérifié sur le
   banc), donc sans failsafe réglé dans la radiocommande, la perte de liaison
   n’est pas détectée. Le réglage est décrit dans `PILOT_GUIDE.md`.

Ce qui se passe en cas de perte de liaison (`FlightController::applyLinkLoss()`) :

- **l’appareil est armé, le GPS et le point de départ sont disponibles**
  (`FAILSAFE_RTH`) — **retour au point de départ** avec le moteur, puis
  cercles au-dessus de celui-ci ; sur l’OLED — `FSRTH`, dans le journal —
  `FAILSAFE_RTH` ;
- **l’appareil est armé, pas de GPS** — **plané**, moteur à
  `FAILSAFE_THROTTLE` : dans n’importe quel mode, même en MANUAL,
  `Autopilot` maintient le roulis `FAILSAFE_GLIDE_ROLL_DEG` (0 — en ligne
  droite, 10–20° — un cercle au-dessus du pilote) et le tangage
  `FAILSAFE_GLIDE_PITCH_DEG` (−3°, pour ne pas perdre de vitesse sans le
  moteur), volets rentrés ; sur l’OLED — `GLIDE`, dans le journal — le mode
  `FAILSAFE_GLIDE` ;
- **non armé** (au sol) ou l’IMU ne répond pas — gouvernes au neutre ;
- le mode et les fonctions ne sont pas changés par les interrupteurs, les
  capteurs continuent d’être lus. L’ARM n’est pas annulé — une fois la liaison
  rétablie, l’avion obéit de nouveau aux manches et au mode choisi (le
  décollage automatique et le lancer à la main ne reprennent que depuis le
  début).

---

## Données des capteurs

Les structures se trouvent dans `include/sensors/SensorInterface.h`.

### `ImuData`

| Champ | Unité | Signification |
|---|---|---|
| `gyroX`, `gyroY`, `gyroZ` | °/s | Vitesses angulaires dans les axes de l’avion, signes aéronautiques (voir plus haut) |
| `accelX`, `accelY`, `accelZ` | g | Accélération dans les axes de l’avion : X vers le nez, Y vers la gauche, Z vers le haut |
| `roll`, `pitch` | ° | Filtre complémentaire (α = 0,98, τ ≈ 0,1 s) ; ils démarrent directement à l’angle de l’accéléromètre |
| `yaw` | ° | Intégrale du gyroscope, dérive lentement ; la valeur initiale est le cap de la boussole |
| `temperature` | °C | Température de la puce (formule pour le MPU6050 ou le MPU6500) |
| `timestamp` | µs | `micros()` à l’instant de la lecture |

Calibration de l’IMU (à chaque démarrage et par la commande `i`) : 2 s
immobile, gyroscope → décalage de zéro, accéléromètre → **la position
actuelle devient l’horizon**.

### `BarometerData`

| Champ | Unité | Signification |
|---|---|---|
| `pressure` | Pa | Pression |
| `temperature` | °C | Température du capteur |
| `altitude` | m | Altitude **par rapport au point de calibration** (au démarrage) ; formule `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | Dérivée de l’altitude sur les échantillons réels (50 Hz) à travers un filtre passe-bas avec τ = 0,5 s |
| `timestamp` | µs | Instant du dernier nouvel échantillon |

### `MagData`

| Champ | Unité | Signification |
|---|---|---|
| `magX`, `magY`, `magZ` | µT | Le champ après la calibration hard-iron, dans les axes de l’avion (`MAG_ROTATION_CW_DEG`) |
| `headingDegrees` | ° (0..360) | `atan2(magY, magX)`, sans compensation d’inclinaison ; le sens de comptage n’a pas encore été vérifié sur l’avion monté |
| `timestamp` | µs | Instant de la lecture (50 Hz) |

### `GpsData`

| Champ | Unité | Signification |
|---|---|---|
| `latitude`, `longitude` | ° | Issues de UBX-NAV-PVT |
| `altitude` | m | Au-dessus du niveau de la mer (hMSL) |
| `groundSpeed`, `heading` | m/s, ° | Vitesse sol et route sol |
| `numSatellites`, `fixType` | — | 0 = pas de fix, 2 = 2D, 3 = 3D |
| `horizontalAccuracy`, `verticalAccuracy` | m | Estimations de précision du module |

**Ce que signifie `isAvailable()`.** Pour les capteurs I2C : le capteur a
répondu à `begin()` **et** les dernières lectures n’échouent pas à la suite
(MPU — ~0,1 s ; baromètre et boussole — ~0,5 s sans réponse). Lorsqu’une
lecture échoue, les données ne sont pas écrasées par du bruit : les valeurs
précédentes restent et le compteur d’erreurs augmente (visible avec la
commande `s`). Pour le GPS : au moins un NAV-PVT valide, et le dernier n’est
pas plus ancien que `GPS_TIMEOUT_US`.

**Si le capteur est absent** (`nullptr` ou `isAvailable() == false`),
`Autopilot` ne donne aucune correction et l’avion se pilote comme en MANUAL.
`main.cpp` ne calibre que les capteurs qui ont répondu.

---

## Détail de FlightController::update()

Appelée depuis `loop()` toutes les 2 ms. L’ordre est la priorité :

1. **`receiver.update()`** — analyse des octets iBUS accumulés.
2. **Interrupteurs** — `switches->update(rc)`, uniquement quand la liaison est
   vivante (dans une trame de failsafe, les canaux ne reflètent pas les
   interrupteurs) : le mode (seulement lors d’un changement), les fonctions,
   les potentiomètres.
3. **Gaz du pilote** — `throttle.update(rc, receiverFailsafe)`.
4. **Manches** — `mixer.fromSticks(rc)` × `Knob::RATES` ; volets —
   `mixer.updateFlaps(target)` (frein, interrupteur, potentiomètre ; sans
   liaison — 0).
5. **Capteurs et pilote automatique** — `autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   **toujours**, même sans liaison : les filtres d’angles ne doivent pas se
   figer. Tant que l’appareil n’est pas armé, le PID tourne (les gouvernes
   réagissent à l’inclinaison — pratique sur la table), mais l’intégrateur est
   maintenu à zéro. Sans liaison et armé — failsafe RTH ou plané.
6. **Buzzer** — `Beeper`.
7. **Perte de liaison** — `applyLinkLoss()` : si l’appareil est armé, les
   gouvernes et les gaz suivent l’ordre de failsafe du pilote automatique,
   sinon neutre et moteur coupé ; `return`. Priorité absolue sur tout ce qui
   suit.
8. **ARM** — `arming.update(rc, false)`.
9. **Ordre** — `autopilot->getCommand()` : dans les modes stabilisés, le
   manche est l’angle voulu, et le pilote automatique émet les ordres finaux
   aux gouvernes.
10. **Mixeur** — `mixer.mix(command)` → PWM des ailerons (volets + roulis), de
    la profondeur et de la direction, en tenant compte de l’inversion.
11. **Gaz** — `autopilot->applyThrottle(pilotThrottle)` : les gaz du pilote,
    ceux du pilote automatique ou le maximum des deux (décollage
    automatique). Ensuite, si l’appareil n’est pas armé ou en `MOTOR_KILL`, —
    forçage à `PWM_MIN`. Cette vérification vient en dernier pour qu’aucun
    mode ne puisse faire passer les gaz en contournant l’ARM.
12. **AUX** — charge utile (`PAYLOAD_DROP`) et caméra (`CAMERA_TILT`,
    `CAMERA_STAB`).
13. **`outputs.write(output)`** — PWM sur les 7 sorties.

---

## API HTTP du tableau de bord web

L’implémentation se trouve dans `include/telemetry/WebDebugServer.h`. Le point
d’accès : SSID `OpenPlane-Debug`, mot de passe `12345678`, adresse
`http://192.168.4.1`.

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

- `attached` — l’objet existe dans la compilation ; `available` — le capteur
  répond réellement. Les champs de données ne sont ajoutés **que** si
  `available: true`.
- `outputs.*.attached` — le MCU a alloué un canal LEDC et une broche ; la
  présence d’un servo physique branché ne se voit pas par le logiciel (pour
  vérifier l’impulsion, utiliser la console, commande `p`).
- `rollCorr`/`pitchCorr` — l’ordre final du pilote automatique moins les
  manches, en µs. `throttleCorr` — les gaz du pilote automatique, en % (0 tant
  que les gaz sont au pilote).
- `nav` — navigation : le point de départ, la distance et le relèvement vers
  lui, le cap et le cap visé, la vitesse utilisée pour la navigation (tube de
  Pitot / GPS), la géobarrière, le décrochage ; `features` — les fonctions
  des interrupteurs qui sont activées.

### `POST /api/setmode`

`{ "mode": 1 }` — le numéro d’`AutopilotMode` : `0` MANUAL, `1` STABILIZE, `2`
AUTO_TAKEOFF, `3` ALT_HOLD, `4` ACRO, `5` CRUISE, `6` LOITER, `7` RTH, `8`
LAUNCH, `9` AUTO_LAND, `10` SOARING, `11` RESCUE. Le mode est maintenu jusqu’à
ce que le pilote actionne l’interrupteur de mode.

### `POST /api/setpid`

`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }` — n’importe lequel des champs
`kpRoll`, `kiRoll`, `kdRoll`, `kpPitch`, `kiPitch`, `kdPitch` ; les champs
omis conservent leurs valeurs précédentes.

Les deux commandes sont appliquées par la boucle de vol au cycle suivant (voir
les [tâches FreeRTOS](#tâches-freertos-et-boucle-de-contrôle)).

### `GET /`

Le tableau de bord HTML : barres des 10 canaux, ARM/liaison, les sorties, les
capteurs, boutons de mode, le formulaire du PID. Il interroge `/api/status`
toutes les 200 ms.

---

## Console et diagnostic

Le moniteur du port — 115200, connecteur « COM ». L’implémentation est
`DebugConsole` et `DebugLogger` ([référence](reference/telemetry.md)). Les
touches agissent immédiatement, Entrée n’est pas nécessaire ; les
calibrations et `p` bloquent la boucle et ne sont donc disponibles que sans
ARM.

| Touche | Ce qu’elle fait |
|---|---|
| `h` / `?` | Menu principal |
| `l` | Le menu « que faut-il écrire dans le journal » (canaux, modes, période) |
| espace | Mettre le journal en pause / reprendre |
| `s` | `printStatus()` de tous les capteurs : données, compteurs d’erreurs du bus, calibrations, la vérification avant vol |
| `i` | Calibration du gyroscope + vérification avant vol (2 s immobile) |
| `o` | Calibration du montage de l’IMU selon trois poses, sauvegardée dans la NVS |
| `m` | Calibration de la boussole (15 s de rotation), sauvegardée dans la NVS |
| `p` | Autotest des sorties : l’impulsion réelle sur chaque broche face à l’impulsion attendue |

Le journal est divisé en canaux (`STAT`, `RC`, `OUT`, `ATT`, `AP`, `ALT`,
`MAG`, `GPS`, `IMU`, `SYS`), chacun avec le mode « désactivé / en cas de
changement / en continu » ; les réglages sont conservés dans la NVS et écrits
à la fermeture du menu, seulement sans ARM. Par défaut, `STAT` (en cas de
changement) et `SYS` (une fois toutes les 10 s) sont activés :

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (le pire sur 10 s) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

Les formats de tous les canaux figurent dans la [référence](reference/telemetry.md#debuglogger).

---

## Choix de la carte et brochage

| Commande | `board` | Macro | Statut |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8 (`qio_opi`, 16 Mo) | `BOARD_ESP32_S3` | **Principale, celle par défaut.** Testée sur le banc avec tous les capteurs |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | L’ancien prototype, a volé en pilotage manuel |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | Pour le banc, le brochage n’a pas été testé sur le matériel |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6 : le micrologiciel complet + MAVLink + boîte noire sur SD ; testée sur une carte nue ([ci-dessous](#stm32h743)) |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | Pareil sur la DevEBox H743 : la console est en USB CDC, le micrologiciel se charge par DFU |

| Rôle | ESP32-S3 (banc) | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| Aileron gauche / droit | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| Profondeur / ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| Direction | GPIO18 | — (pas de broche) | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| I2C des capteurs SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| I2C de l’OLED SDA / SCL | GPIO1 / GPIO2 | — | — |
| SPI SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI CS ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS RX / TX | GPIO39 / GPIO40 (UART2) | GPIO9 ⚠️ / aucune (UART0) | GPIO4 / GPIO17 (UART2) |
| Serial | UART0 → connecteur « COM » | USB-CDC | UART0 |

- **ESP32-S3 N16R8 :** les GPIO33–37 sont pris par la PSRAM octale, les 26–32
  par la flash, les 19/20 par l’USB, les 43/44 par le Serial, le 48 est la LED
  RVB ; les 0/3/45/46 sont des broches de strapping.
- **ESP32-C3 :** les ailerons sur GPIO4/5 sont permutés par rapport à la S3. Il
  n’y a pas assez de broches pour l’ensemble complet : le CS du BMP388 et le RX
  du GPS sont sur des broches de strapping, le GPS n’a pas de TX (réception
  seule, sans UBX-CFG). Détails dans `Config.h`.

### STM32H743

La STM32H743VIT6 (Cortex-M7 à 480 MHz, 2 Mo de flash, 1 Mo de RAM) exécute le
**micrologiciel complet** : les mêmes capteurs, pilote automatique,
interrupteurs, console et écran que sur l’ESP32-S3, plus la télémétrie MAVLink
et une boîte noire sur carte SD. Il se compile, passe cppcheck et tous les
tests natifs du code commun. Sur le matériel, c’est la **carte DevEBox H743
sans capteurs** qui a été testée : démarrage, console par USB, carte SD, boîte
noire — [TESTING.md](TESTING.md#tests-sur-la-carte-stm32) — ainsi que l’iBUS, l’ARM
et le PWM vers les servos et le moteur : pilotage depuis la radiocommande en
mode manuel (en vidéo). Les capteurs sur la STM32 attendent encore un banc. La
carte de vol principale est l’ESP32-S3.

- **HAL** — `include/hal/stm32/` : `Stm32Board` (la même API que `Esp32Board`,
  plus `telemetryUart()`), `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`,
  `Stm32ServoOutput` (PWM matériel de `HardwareTimer`, un timer pour plusieurs
  sorties). En détail — [reference/hal.md](reference/hal.md#implémentation-pour-le-stm32h743).
- **Réglages et calibrations** — pas dans la NVS, mais dans un `KeyValueStore`
  situé dans le dernier secteur de la flash (`include/storage/`,
  `hal/stm32/Stm32FlashStorage.h`). Le code du projet écrit toujours
  `#include <Preferences.h>` : dans l’env `stm32h743`, le répertoire
  `include/hal/stm32/compat/` est dans `-I`, et un `Preferences` à la même API
  s’y trouve. L’image porte un CRC32 : une image corrompue (coupure de courant
  pendant l’effacement) est lue comme vide. L’écriture en flash se fait dans une
  tâche de fond : effacer un secteur de 128 Ko prend des secondes, mais le
  secteur est dans la banque 2 alors que le code s’exécute depuis la banque 1,
  et la tâche de vol préempte celle de fond sans s’arrêter.
- **Tâches** — FreeRTOS de la bibliothèque STM32duino FreeRTOS, un seul cœur,
  préemption par priorité (`hal/Rtos.h`) : `flight` (5) — la boucle de vol,
  MAVLink, journal, console ; `oled` (1) et `storage` (1) — en arrière-plan ;
  `bbox` (2) — l’écriture de la boîte noire sur la carte SD.
- **La boîte noire sur la carte SD** — SDMMC1, 4 bits, 24 MHz
  (`hal/stm32/Stm32SdCard.h`, les broches dans `src/stm32/sd_msp.cpp`). La carte
  reste une FAT32 ordinaire : un fichier `BLACKBOX.BIN` créé à l’avance s’y
  trouve, le micrologiciel écrit des blocs bruts à l’intérieur et ne touche pas
  au système de fichiers lui-même (`storage/Fat32File.h` est en lecture
  seule). Préparation de la carte et téléchargement —
  [BLACKBOX.md](BLACKBOX.md#carte-sd-stm32h743).
- **Télémétrie** — MAVLink 2 sur l’UART4 (`telemetry/MavlinkTelemetry.h`) à la
  place du tableau de bord Wi-Fi : QGroundControl / Mission Planner,
  changement de mode et de PID depuis le sol. En détail —
  [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#station-sol-tableau-de-bord-wi-fi-et-mavlink).
- **Brochage** — le bloc `BOARD_STM32H743` de `Config.h`, les broches ayant été
  choisies parmi celles qui sont libres sur la WeAct MiniSTM32H743VITx et
  recoupées avec les tables de STM32duino :

| Rôle | STM32H743 | Périphérique |
|---|---|---|
| Aileron gauche / droit | PA0 / PA1 | TIM2_CH1 / CH2 |
| Profondeur / ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| Direction | PD14 | TIM4_CH3 |
| AUX1 (charge utile) / AUX2 (caméra) | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX (TX — réserve) | PE7 (PE8) | UART7 |
| I2C des capteurs SDA / SCL | PB11 / PB10 | I2C2 |
| I2C de l’OLED SDA / SCL | PB9 / PB8 | I2C1 |
| SPI SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI CS IMU / baromètre | PB12 / PD10 | GPIO |
| GPS RX / TX | PD9 / PD8 | USART3 |
| Modem radio MAVLink RX / TX | PD0 / PD1 | UART4 |
| Buzzer | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **DevEBox H743 (MCUDEV)** — l’env `stm32h743-devebox` : le même code, sa
  propre variante du cœur, la console par USB-C en port COM virtuel (CDC) — pas
  besoin d’USB-UART. Le premier chargement du micrologiciel se fait par USB via
  le chargeur d’amorçage intégré (DFU) :
  1. Windows : installer une fois le pilote WinUSB pour le « STM32
     BOOTLOADER » ([Zadig](https://zadig.akeo.ie) : DFU in FS Mode → WinUSB →
     Install Driver).
  2. Relier la broche **BT0** (BOOT0) à **3V3** avec un fil, appuyer sur
     **RST** puis relâcher : la carte est en mode DFU (la DevEBox n’a pas de
     bouton BOOT0).
  3. `pio run -e stm32h743-devebox -t upload` (`upload_protocol = dfu`).
  4. Le fil de BT0 peut être retiré — le micrologiciel démarre tout seul.

  Ensuite, le fil n’est plus nécessaire : la touche **`D`** de la console
  (depuis n’importe quel menu, pas en ARM) redémarre la carte dans le chargeur
  d’amorçage : un marqueur en RAM → réinitialisation → saut dans la mémoire
  système avant la configuration des horloges (`src/stm32/bootloader.cpp`). Un
  saut direct depuis le micrologiciel en cours d’exécution se fige sur le H7 —
  vérifié sur la carte, d’où les deux étapes. Il faut une console ouverte (USB
  CDC) ; si la carte ne répond pas — RST avec le fil de BT0 en place.
- **Point d’entrée** — `src/stm32/main.cpp` (exclu des compilations ESP32 par
  `build_src_filter`). Les objets sont les mêmes que dans `src/main.cpp` ; à la
  place de `loop()` il y a des tâches, et `vTaskStartScheduler()` se trouve à la
  fin de `setup()`.
- **Première mise sous tension de la carte :** `pio run -e stm32h743 -t upload`
  (ST-Link), le moniteur sur LPUART1 via un USB-UART ; `b` — les capteurs
  sont-ils visibles sur les bus, `s` — état des capteurs, `p` — impulsions sur
  les sorties (retirer l’hélice), puis la radiocommande et QGroundControl via le
  modem radio.

---

## Comment ajouter un nouveau capteur

### A) Une autre puce d’une catégorie existante (IMU, baromètre, boussole)

Le code commun est déjà écrit dans les classes de base — le pilote de la puce
reste donc petit :

1. Créez `include/sensors/<category>/<Name>_Sensor.h` et héritez de
   `ImuSensorBase` / `BarometerBase` / `MagnetometerBase`. Le constructeur
   reçoit un `IRegisterDevice&` — le pilote ne sait pas s’il s’agit d’I2C ou de
   SPI.
2. Implémentez :
   - `begin()` — `device.begin()`, vérifier l’identifiant de la puce, écrire
     les registres, appeler `setAvailable(true/false)` ;
   - IMU : `readSample()` (accel/gyro/temp bruts dans les axes de la puce),
     `accelLsbPerG()`, `gyroLsbPerDps()`, `temperatureC()` ;
   - baromètre : `isNewSampleReady()` (un indicateur de donnée prête ou
     simplement `true`) et `readSample()` (pression en Pa, température en
     °C) ; la période d’interrogation se règle dans le constructeur de la
     base ;
   - boussole : `readRaw()` (X/Y/Z dans les axes de la puce) et
     `lsbPerMicroTesla()` ; le nom de l’espace NVS pour la calibration se
     donne dans le constructeur de la base.
3. Si, en SPI, la puce a besoin d’un octet factice avant les données ou d’une
   fréquence particulière — ajoutez une fabrique statique `spiDevice(bus, cs)`,
   comme celle de `BMP388_Sensor`.
4. Une branche dans `SensorSelection.h` : `#define SENSOR_<CATEGORY>_<NAME>`,
   `using Selected... = ...;` et `#define SELECTED_..._DEVICE(board) ...`
   (`I2cRegisterDevice(board.i2c(), address)` ou la fabrique SPI). `main.cpp`
   n’est pas modifié quand on change de capteur.
5. Vérifiez la compilation avec le nouveau capteur sans modifier le fichier —
   avec un drapeau :
   `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAME>" pio run`, puis les
   trois environnements, puis sur le matériel.

### B) Une nouvelle catégorie

1. La structure de données et l’interface vont dans `SensorInterface.h`, sur le
   modèle de `GpsSensor`/`GpsData`.
2. Si la catégorie a une logique commune (filtres, calibration) — une classe de
   base sur le modèle de `BarometerBase`.
3. Un pointeur nullable dans le constructeur d’`Autopilot` (sans capteur —
   aucun effet, plutôt qu’un plantage) et des champs dans `GET /api/status`
   avec une paire `attached`/`available`.

### Un nouveau bus ou périphérique

Une nouvelle interface dans `include/hal/`, une implémentation dans
`include/hal/esp32/` et dans `include/hal/stm32/`, un accès via `IBoard`.

---

## Comment ajouter un nouveau mode du pilote automatique

1. Une valeur dans `enum AutopilotMode` (`autopilot/AutopilotTypes.h`, avant
   `MODE_COUNT`), le nom et un nom court (jusqu’à 5 caractères, pour l’OLED)
   dans `AutopilotNames::mode()` / `modeShort()`.
2. Un gestionnaire `run<Mode>()` et une branche dans `Autopilot::runMode()` ;
   les objectifs initiaux (cap, altitude, centre des cercles) vont dans
   `initializeMode()`. Le mode fixe `desiredRoll`/`desiredPitch` et appelle
   `stabilizeOrManual()` (sans IMU, les gouvernes restent au pilote) ou
   `stabilizeOrNeutral()` (sans IMU — neutre). Sans le capteur requis — un
   comportement sûr, et non un plantage. L’intégrateur ne s’accumule qu’avec
   `armed`.
3. Gaz : `throttleMode` (`PILOT` / `AUTO` / `AT_LEAST`) et `autoThrottlePct`,
   ou `autoThrottle()` — les gaz de croisière issus du potentiomètre / du tube
   de Pitot. `FlightController` ne change pas.
4. Sur la radiocommande — une seule ligne dans `config/Controls.h`
   (`Bind::mode(Channels::SWD, MODE_NEW)`). Le tableau de bord et MAVLink
   reprennent le mode par son numéro ; pour MAVLink — le mode ArduPlane le plus
   proche dans `MavlinkModes::toCustomMode()` / `fromCustomMode()`.
5. Si le mode a besoin de capteurs pour l’ARM — `ArmingManager`.
6. Tests : la réaction à chaque capteur — `test/native/test_autopilot_modes` ;
   le vol en boucle fermée — un scénario dans `test/native/test_sim` (le modèle
   d’avion `helpers/PlaneSim.h`, le banc `helpers/SimHarness.h`). Ensuite — la
   table sans hélice : les gouvernes doivent réagir à l’inclinaison dans le sens
   du redressement.
7. Une section dans [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

---

## Rétroaction (ébauche, non branchée)

`include/autopilot/feedback/` est la prochaine étape du pilote automatique.
**Ni `FlightController`, ni `Autopilot`, ni `main.cpp` n’incluent ces
fichiers :** il n’existe pas encore de prototype pour les essais en vol, et le
micrologiciel fonctionne sans eux. Ils sont vérifiés par une simulation en
boucle fermée (`test/test_feedback/`) directement sur la carte.

### Pourquoi

Aujourd’hui `Autopilot` est un PID sur l’angle : erreur × gain = gouverne. Il
ne sait pas ce qu’il en est résulté sur l’avion, et les gains ne sont justes
que pour une seule vitesse : à basse vitesse la gouverne est plus faible et le
PID corrige trop peu, à haute vitesse il corrige trop. La rétroaction referme
la boucle sur **la réponse de l’avion** :

- la gouverne a été braquée, mais l’avion tourne plus lentement que
  nécessaire — en ajouter, jusqu’à ce qu’il y arrive ;
- la quantité de gouverne nécessaire est mesurée en vol et recalculée selon la
  vitesse ;
- l’avion tourne dans le mauvais sens — le signe est inversé, le retourner et
  vérifier ;
- l’angle a été redressé mais la vitesse chute — gaz et nez vers le bas,
  jusqu’à ce que l’avion ne décroche plus ;
- décollage et atterrissage — par phases, d’après ce que montrent les
  capteurs.

### Modules

| Fichier | Ce qu’il fait |
|---|---|
| `FlightSnapshot.h` | Tout ce que la rétroaction sait de l’avion pour un cycle. C’est la seule entrée : les modules ne lisent pas directement les capteurs ni le RC, ils peuvent donc tourner sur une simulation et sur des journaux |
| `FeedbackOutput.h` | La sortie d’un cycle : braquages des gouvernes par axe, l’axe est-il activé, le signe de l’axe, les gaz (imposer / pas en dessous de), la raison |
| `FeedbackConfig.h` | Toutes les constantes (elles migreront dans `Config.h` lors du branchement) |
| `SpeedEstimator.h` | La vitesse (tube de Pitot > GPS) et l’accélération longitudinale d’après l’IMU : `dV/dt = g·(ax − sin θ)` — on voit que « la vitesse chute » même sans capteur de vitesse |
| `AirborneDetector.h` | En l’air / au sol : apprendre, accumuler l’intégrale et chercher le décrochage n’a de sens qu’en vol |
| `ControlEffectivenessEstimator.h` | Pour chaque axe, apprend le modèle `ε = b·u(t−delay) + a·ω + c` par moindres carrés récursifs |
| `AdaptiveRateController.h` | Une cascade angle → vitesse angulaire → accélération angulaire → gouverne par le modèle appris |
| `StallGuard.h` | Protection contre la perte de vitesse et le décrochage |
| `TakeoffSequencer.h`, `LandingSequencer.h`, `PhaseTargets.h` | Décollage (depuis une piste ou à la main) et atterrissage par phases, d’après les capteurs |
| `FeedbackSupervisor.h` | Le tout ensemble : l’ordre au sein d’un cycle, les priorités, `requestTakeoff()`/`requestLanding()`/`cancelPhase()`, `printStatus()`, le plan de branchement |
| `FeedbackModules.h` | Un seul include pour tout |

### Comment cela fonctionne

**Efficacité des gouvernes.** Le modèle de l’axe : accélération angulaire
`ε = b·u + a·ω + c`. `b` est le nombre de °/s² que donne 1 µs de gouverne (le
signe est le sens de la réponse), `a` est l’amortissement (l’air freine la
rotation ; sans ce terme, l’estimation de `b` tendrait vers zéro lors d’une
rotation établie), `c` est un moment constant (centrage, compensateur,
hélice). La force d’une gouverne ∝ ρV², donc `b` est appris à une vitesse de
référence et multiplié par `(V/Vref)²` : l’avion accélère — la gouverne
« devient plus efficace » instantanément, sans réapprentissage. La vitesse
indiquée du tube de Pitot contient déjà la masse volumique de l’air, donc
l’altitude est prise en compte d’elle-même ; sans capteur de vitesse,
l’échelle vaut 1 et `b` est appris directement.

Les données sont prises par intervalles de 20 ms : l’accélération moyenne sur
un intervalle est la différence du gyroscope aux extrémités / la durée, et
elle correspond à la gouverne moyenne et à la vitesse angulaire moyenne sur
le même intervalle (la gouverne — avec le retard `RESPONSE_DELAY_MS`). Puis
les deux membres de l’équation passent par le même filtre passe-bas à 2 Hz :
le rapport ne change pas, tandis que les hautes fréquences, où le modèle du
« retard pur » ment à cause de l’inertie du servo, sont éliminées. On ne peut
apprendre qu’en l’air et seulement quand la gouverne est « secouée » (une
amplitude ≥ `MIN_EXCITATION_US` en ~0,3 s) ; les manches du pilote secouent
aussi, donc l’estimation apprend aussi en MANUAL.

**Le régulateur.** Trois étages, axe par axe :

```
ω* = ANGLE_GAIN · (target − angle)              "nez 10° trop bas — relever à 40°/s"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "tourne plus lentement que nécessaire — corriger"
surface = (ε* − a·ω − c) / b                    par le modèle appris
```

L’intégrale `I` est conservée en °/s et non en µs de gouverne — elle reste donc
juste quand l’estimation de `b` change. Au sol, l’intégrale est gelée (sauf le
cap pendant la course au décollage / à l’atterrissage), et, la gouverne étant
en butée, elle ne s’accumule pas vers la butée. Le virage coordonné est pris
en compte (si la vitesse est connue) : en virage incliné, il faut un
tangage `g·sin φ·tg φ / V` et un lacet `g·sin φ / V`.

**Signes des axes — seulement au sol.** En vol, les axes ne sont ni
désactivés ni inversés : le montage de l’IMU est déterminé par la calibration
`o` et la vérification à la mise sous tension, et les sens des gouvernes par la
vérification avant vol du pilote. Les indices indirects en l’air (un
décrochage, une vrille, des figures, des rafales) peuvent tromper, et un axe
désactivé ou inversé à un tel moment coûte l’avion. Si l’estimation de `b`
d’un axe est nettement négative, ce n’est qu’un avertissement dans `reason`
(« répond à la gouverne à l’envers ? vérifier au sol ») ; une estimation
négative n’entre pas dans le régulateur — l’axe fonctionne avec le modèle a
priori.

**Protection contre le décrochage.** Deux niveaux. *LowEnergy* — la vitesse
chute vite nez levé, ou est proche du décrochage (< 1.25·Vs), ou la
profondeur a perdu en efficacité : gaz ≥ 80 %, tangage ≤ 5°. *Stall* — la
vitesse est inférieure à celle du décrochage, le nez tombe contre la
profondeur, l’aile tombe contre les ailerons à faible énergie : plein gaz,
nez vers le bas, inclinaison ≤ 10°, ailerons limités (un grand aileron fait
décrocher le bout d’aile). Les mesures sont levées avec hystérésis (vitesse
≥ 1.5·Vs). En cas de perte de liaison, les gaz ne sont pas touchés, et tout
près du sol (arrondi, course à l’atterrissage) la protection est désactivée —
l’atterrissage est lui-même un décrochage contrôlé.

**Décollage.** `WaitThrottle` (le moteur est arrêté) → le pilote a donné les
gaz ≥ 50 % → `GroundRoll` (plein gaz, ailes à plat, le cap tenu par la
direction et la roue, la profondeur libre) → vitesse de décollage, ou un délai
sans capteur de vitesse → `Climb` (12°, plein gaz) → altitude 30 m →
`Complete`. À la main (`TAKEOFF_HAND_LAUNCH`), à la place de la course —
`WaitLaunch` : le moteur ne démarre qu’après le lancer (accélération
longitudinale ≥ 1g). Gaz retirés avant le décollage — annulation.

**Atterrissage.** `Approach` (gaz 25 %, descente de 1 m/s — le tangage d’après
l’erreur de vitesse verticale, l’inclinaison donnée par le pilote ≤ 20°) →
altitude 2 m → `Flare` (gaz 0, la descente est amortie jusqu’à 0.3 m/s par la
même règle) → choc détecté par l’accéléromètre ou « bas et ne tourne pas » →
`Rollout` (cap par la roue) → `Complete`. Gaz du pilote ≥ 80 % — remise des
gaz. L’arrondi demande un télémètre : le baromètre se trompe d’un mètre.

**Priorités** (`FeedbackSupervisor`) : non armé > protection contre le
décrochage > décollage/atterrissage > objectifs du mode. En cas de perte de
liaison, les phases sont annulées et la stabilisation exécute les objectifs de
plané du failsafe.

### Simulation

`test/test_feedback/test_main.cpp` (sur PC : `pio test -e native -f test_feedback`) — un modèle d’avion (axes indépendants,
retard et inertie du servo, efficacité des gouvernes ∝ V², amortissement ∝ V, moments
constants, portance via l’angle d’incidence en fonction de la vitesse, décrochage, train d’atterrissage avec
roue directrice) et 10 scénarios :

| Scénario | Ce qui est vérifié |
|---|---|
| Sortie d’une inclinaison de 30° / d’un tangage de −15° avec un moment constant | Le redressement et le « corriger encore » : l’intégrale trouve seule le compensateur |
| Secousse de ±15° à 14 et 20 m/s, sans capteur de vitesse | L’estimation de `b` converge vers la vérité et est remise à l’échelle avec la vitesse |
| Aileron inversé, le pilote balance les ailes en MANUAL | L’estimation de `b` est négative → simple avertissement, l’axe n’est pas désactivé |
| 30 s de turbulence | Les rafales sont contrées, l’inclinaison ne dépasse pas 10° |
| Nez à 15° avec 20 % de gaz (avec capteur de vitesse et sans) | La vitesse ne chute pas jusqu’au décrochage |
| Décollage depuis une piste avec le couple de réaction de l’hélice | Phases, altitude, cap pendant la course |
| Atterrissage depuis 15 m | Phases, pas de gaz près du sol, toucher doux |
| Perte de liaison pendant la course au décollage ; non armé ; MANUAL | Annulation, les gaz ne sont pas touchés, les gouvernes restent au pilote |

Le modèle est grossier — il vérifie la logique et les signes, et non le réglage
pour une cellule précise.

```bash
pio test -e native -f test_feedback      # sur PC, en quelques secondes
pio test -e esp32-s3 -f test_feedback    # charge le micrologiciel de test et le lance
pio run -t upload                        # remettre le micrologiciel normal
```

### Plan de branchement

1. `FlightController::update()`, après la lecture des capteurs et le calcul
   des ordres, remplit un `FlightSnapshot` et appelle
   `FeedbackSupervisor::update()`. Au début — **mode fantôme** : la sortie va
   seulement dans le journal (`printStatus()`) et sur le tableau de bord, pas
   aux gouvernes. En vol en pilotage manuel, l’estimation de `b` de chaque
   axe doit être positive et croître avec la vitesse.
2. Au sol, l’avion dans les mains, STABILIZE : l’incliner — les gouvernes
   s’y opposent.
3. Un axe à la fois : `deflectionUs` à la place de
   `Autopilot::getRollCorrection()` (d’abord le roulis seul), puis le tangage.
4. Gaz : `throttleOverridePercent`/`throttleFloorPercent` — après
   `Autopilot::applyThrottle()`, avant le failsafe (le failsafe prime sur
   tout).
5. Décollage/atterrissage — sur un interrupteur libre ; retirer le mode
   `AUTO_TAKEOFF` d’`Autopilot`.
6. Les constantes de `FeedbackConfig` — dans `Config.h` ; le capteur de
   vitesse air — une implémentation d’`AirspeedSensor` et une catégorie dans
   `SensorSelection.h`.

---

## Comment ajouter une nouvelle carte

1. `[env:<name>]` dans `platformio.ini` avec un `-D BOARD_ESP32_<NAME>` unique.
2. Un bloc `#elif defined(BOARD_ESP32_<NAME>)` dans `Config.h` avec toutes les
   broches, y compris `PIN_I2C2_SDA/SCL` (−1 s’il n’y a pas d’OLED). Calculez le
   budget de GPIO à l’avance : flash/PSRAM/USB/strapping.
3. Les sorties des servos ont besoin de 5 canaux LEDC — toutes les ESP32 en
   ont. S’il n’y a pas de broche pour la direction — `PIN_RUDDER = -1`, et la
   sortie est simplement désactivée.
4. Ne modifiez pas `default_envs` tant que la carte n’a pas été testée sur le
   matériel ; indiquez explicitement dans le commit si le brochage n’a pas été
   testé.

---

## Commandes de compilation, de téléversement et de moniteur

```bash
pio run                        # compiler la carte par défaut (esp32-s3)
pio run -t upload              # téléverser
pio device monitor             # moniteur, 115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # vérifier que toutes les cartes se compilent
```

- **ESP32-S3 :** le téléversement et le Serial passent par le connecteur
  « COM » (CH343). Si le pont se bloque (Windows répond « le périphérique ne
  fonctionne pas » — cela peut venir des parasites de l’ESC), il suffit de
  rebrancher le câble ; on peut aussi téléverser par le connecteur « USB » (le
  USB-JTAG intégré) : `pio run -t upload --upload-port <USB COM port>`.
- Tant que le moniteur du port est ouvert, le téléversement vers le même port
  ne passera pas.
- `lib_deps` : `olikraus/U8g2` (OLED) est la seule bibliothèque externe.
- `test/` — en détail dans [`TESTING.md`](TESTING.md) :
  - `pio test -e native -e native-stm32` — 387 tests sur PC (simulacres du
    matériel dans `test/native/support/`), couverture — `gcovr` ;
  - `pio test -e esp32-s3` — `test_feedback/` (la simulation en boucle fermée
    de la rétroaction) et `test_imu_orientation/` sur la carte elle-même ;
    chacun charge un micrologiciel de test, puis il faut recharger le normal
    avec `pio run -t upload`.
- Analyse statique : `pio check -e esp32-s3` (cppcheck), `pio check -e stm32h743` (cppcheck sur
  `hal/stm32/` et `src/stm32/`) et `tools/clang-tidy.sh`
  (le profil `.clang-tidy`).

---

## Limitations connues

- **Le pilote automatique n’a pas été testé en vol.** Sur la table, les signes
  ont été vérifiés en direct (inclinaison → correction dans le sens du
  redressement) ; les coefficients du PID sont des valeurs de départ.
- **STABILIZE est un redressement superposé aux manches**, et non un « mode
  angulaire » (FBWA) où le manche fixe l’angle de roulis/tangage. Le pilote et
  le pilote automatique s’additionnent.
- **Le plané en cas de perte de liaison n’a pas été testé en vol.** Les angles
  `FAILSAFE_GLIDE_*` sont des valeurs de départ ; le tangage de −3° se règle
  pour une cellule précise (le nez ne doit ni se cabrer jusqu’au décrochage ni
  piquer).
- **L’horizon.** Avec la calibration du montage (`o`), il en provient (NVS) ;
  le décalage de zéro de l’accéléromètre dérive avec la température (~1–2° par
  20 °C) ; si l’horizon a « dérivé », refaire `o`. Sans elle — la pose à la mise
  sous tension (allumer à plat).
- **Le montage de la boussole** est toujours fixé par `MAG_ROTATION_CW_DEG`
  (la calibration par poses ne le concerne pas).
- **Boussole :** le cap est sans compensation d’inclinaison, le sens de
  comptage n’a pas été vérifié sur l’avion monté, et la calibration doit être
  faite dans l’avion lui-même. Aucun mode n’utilise encore le cap.
- **Le GPS** n’est pas utilisé pour la navigation ; sur l’ESP32-C3, c’est de la
  réception seule.
- **La rétroaction (`autopilot/feedback/`) n’est pas branchée** et n’a été
  vérifiée qu’en simulation avec un modèle d’avion grossier. Tous les nombres
  de `FeedbackConfig.h` marqués « прикидка » (« estimation grossière ») sont à affiner sur une cellule réelle ;
  il n’y a pas encore de capteur de vitesse air (sans lui, l’efficacité des
  gouvernes s’apprend plus lentement, et le décrochage n’est visible que par la
  décélération).
- **Non testés sur le matériel :** `ICM42688_Sensor` (ramené à la convention
  commune via `ImuSensorBase`), `BME280_Sensor` (la compensation Bosch a été
  réimplémentée), BMP388 en SPI, `QMC5883L_Sensor`, la configuration du GPS par
  CFG-VALSET. Au branchement : le journal de démarrage, `s` dans la console, les
  signes par l’inclinaison.
- **L’I2C sur une plaque d’essai capte des parasites** de l’ESC/du moteur (des
  erreurs isolées sont visibles avec `s`). Les pilotes les supportent, mais
  dans l’avion les fils de l’I2C doivent être courts et éloignés des fils de
  puissance.
- **ESP32Servo n’est pas utilisée.** La version 3.2.1 sur l’ESP32-S3 répartit
  les servos sur les MCPWM et, dans `attachPin()`, confond le numéro de
  l’unité MCPWM avec celui du timer : les GPIO6/7 sortaient le signal des
  GPIO4/5 (l’ESC était commandé par le manche droit). Les sorties ont été
  réécrites sur LEDC ; ne remettez la bibliothèque qu’après vérification avec
  `p`.
- **L’ESC est en PWM à 50 Hz** ; le micrologiciel n’a pas encore de mode de
  calibration de la plage des gaz.
- **Tableau de bord web :** le mot de passe du point d’accès est faible, et les
  commandes sont aussi acceptées en vol. C’est un outil pour le banc et le
  terrain, pas pour le vol.
- **Mécanique du prototype :** le premier prototype a volé, une fixation
  faible du moteur et une rigidité insuffisante de l’aile ont été constatées.
- **La licence est l’OpenPlane License** ([LICENSE](LICENSE.md)) : MIT avec
  mention obligatoire de l’auteur, interdiction de l’usage militaire et
  interdiction de nuire intentionnellement aux personnes et aux biens sans leur
  consentement écrit. N’ajoutez pas d’autres en-têtes de licence aux fichiers
  et ne retirez pas le nom de l’auteur.

---

## Comment apporter des modifications

- **Petits commits :** une étape logique — un commit.
- **Tests et analyse avant un commit :** `pio test -e native -e native-stm32`,
  `pio check -e esp32-s3`, `pio check -e stm32h743`, `tools/clang-tidy.sh` —
  tout au vert ([`TESTING.md`](TESTING.md)).
- **Compilez toutes les cartes** après des modifications du code commun — la S3
  est la principale, mais la C3, la 38 broches et `stm32h743` ne doivent pas
  casser ; avant une version — `tools/build_matrix.sh` (toutes les cartes ×
  tous les capteurs).
- **Vérifiez sur le matériel ce qui peut l’être :** les signes — par
  l’inclinaison, les sorties — avec la commande `p`, la liaison — en éteignant
  la radiocommande.
- **N’inventez pas d’API.** Reportez-vous aux sources du framework dans
  `~/.platformio/packages/framework-arduinoespressif32/` (Arduino core 2.0.x)
  — Internet décrit souvent la version 3.x, dont l’API est différente (par
  exemple, LEDC).
- **N’enjolivez pas l’état.** Non testé sur le matériel — écrivez-le tel quel.
- **Une couche ne doit pas en savoir plus qu’il ne lui revient.** Si une
  classe inférieure a soudain besoin d’une classe supérieure, la logique doit
  remonter dans `FlightController`.
- **Quand vous changez un contrat de données** (`FlightOutputState`,
  `ControlCommand`, `ImuData`, le JSON de `/api/status`) — mettez à jour tous
  les consommateurs dans le même commit.
