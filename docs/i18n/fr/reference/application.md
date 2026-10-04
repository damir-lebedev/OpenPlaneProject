# APPLICATION — `src/main.cpp` et `src/stm32/main.cpp`

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/application.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels.

[← Référence](README.md)

Les deux points d’entrée sont la **composition root** de leurs cartes : la
seule unité de traduction du micrologiciel et le seul endroit où les objets
sont créés et reliés par des références. Ils ne contiennent aucune logique de
vol et l’ensemble des objets est le même ; ce qui diffère, c’est la carte, la
télémétrie (Wi-Fi ou MAVLink) et la façon dont la boucle de vol est cadencée.

## Objets globaux (communs)

L’ordre de déclaration = l’ordre de construction.

| Objet | Type | Liens |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice`, `imuSensor` | `SELECTED_IMU_DEVICE(board)`, `SelectedImu` | le bus de `SensorSelection.h` |
| `baroDevice`, `baroSensor` | `SELECTED_BARO_DEVICE(board)`, `SelectedBaro` | avec le tube de Pitot — c’est la pression statique |
| `magDevice`, `magSensor`, `magnetometer` | … `SelectedMag`, `MagnetometerSensor* const` | seulement si `SENSOR_MAG != NONE`, sinon `nullptr` |
| `gpsSensor`, `gpsReceiver` | `SelectedGps`, `GpsSensor* const` | seulement si `SENSOR_GPS != NONE` |
| `pitotDevice`, `pitotBaro`, `pitotSensor`, `airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`, `SelectedPitotBaro` (`"PITOT-BMP581"`), `PitotDualBaroAirspeed(pitotBaro, baroSensor)` | seulement si `SENSOR_AIRSPEED != NONE` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`, `throttleManager` | `ControlMixer`, `ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | tous les capteurs (nullables) |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`, le tableau `Controls::BINDINGS` |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | tout ce qui précède |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | le contrôleur, le pilote automatique, les statistiques |
| `debugConsole` | `DebugConsole` | le contrôleur, les sorties, le pilote automatique, le journal, `&board` (interrogation des bus `b`) |
| `oledDisplay` | `OledDisplay` | le contrôleur, le pilote automatique, les statistiques |
| ESP32 : `webDebugServer` | `WebDebugServer` | le contrôleur, le pilote automatique |
| STM32 : `mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`, le contrôleur, le pilote automatique, les statistiques |

## `src/main.cpp` — ESP32 (S3, C3, 38 broches)

| Fonction | Description |
|---|---|
| `static void printBanner()` | l’écran d’accueil dans `Serial` |
| `static void setupSensors()` | `begin()` de chaque capteur ; calibration de ceux qui ont répondu : IMU `calibrate()` (2 s immobile + la vérification avant vol), baromètre `calibrateAltitude()`, boussole — le premier échantillon au bout de 25 ms fixe le cap de l’IMU (`setYaw`) ; GPS `begin()` ; Pitot `begin()` (le zéro — dans la première seconde de la boucle) ; `autopilot.begin()` |
| `void setup()` | `Serial.setTxBufferSize(4096)` **avant** `begin(115200)` ; l’écran d’accueil ; `board.begin()` ; `flightOutputs.begin()` + `setFailsafe()` ; `setupSensors()` ; `flightController.begin()` ; OLED ; le serveur web ; la disposition des interrupteurs ; `debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()` ; la période est `vTaskDelayUntil(LOOP_PERIOD_MS)` ; un retard > 100 ms — le décompte repart de zéro (sans « rattrapage ») |

## `src/stm32/main.cpp` — STM32H743

Le point d’entrée de l’env `stm32h743` (dans les compilations ESP32, le
répertoire `src/stm32/` est exclu via `build_src_filter`). Sur le matériel, la
carte DevEBox H743 sans capteurs a été testée (démarrage, console par USB,
carte SD, boîte noire, iBUS et pilotage manuel des servos et du moteur) ;
l’ensemble tourne sur PC grâce aux tests `test/native_stm32` (l’env
`native-stm32`). À côté : `sd_msp.cpp` — les broches et horloges de SDMMC1,
`bootloader.cpp` — la touche `D` de la console (redémarrage en DFU).

| Fonction | Description |
|---|---|
| `setup()` | `Serial.begin(115200)` ; l’écran d’accueil ; `board.begin()` ; les sorties en position de sécurité ; `Stm32FlashStorage::store().mount()` — l’image des réglages (vide / N octets / corrompue — valeurs par défaut) ; `setupSensors()` (comme sur l’ESP32) ; `flightController.begin()` ; `mavlink.begin()` ; OLED ; la disposition des interrupteurs ; `debugLogger.begin()` ; les tâches `flight` et `storage` ; `vTaskStartScheduler()` (ne retourne pas) |
| `static void flightTask(void*)` | priorité `Rtos::PRIORITY_FLIGHT`, pile de 16 Ko : `flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()` ; `vTaskDelayUntil(LOOP_PERIOD_MS)`, un retard > 100 ms — le décompte repart de zéro |
| `static void storageTask(void*)` | en arrière-plan : `Stm32FlashStorage::instance().service()` une fois toutes les 100 ms — effacement et écriture du secteur des réglages, préemptée par la tâche de vol |
| `loop()` | vide : après `vTaskStartScheduler()`, seules les tâches fonctionnent |

La console (`Serial`, LPUART1 PA9/PA10, 115200) est la même `DebugConsole` que
sur l’ESP32 : `h` menu, `s` capteurs, `b` interrogation des bus, `p` sorties,
calibrations.

## Invariants

- Les sorties passent en position de sécurité **avant** l’initialisation des
  capteurs (la calibration de l’IMU bloque la boucle ~2 s).
- ESP32 : le tampon TX de `Serial` est fixé avant `begin()`. STM32 : les
  tampons des UART sont `SERIAL_RX/TX_BUFFER_SIZE` dans `platformio.ini`.
- Aucun objet n’en possède un autre : toutes les références sont non
  propriétaires, la durée de vie est celle du programme entier.
- Pour changer ce que fait un interrupteur — `config/Controls.h`, pas
  `main.cpp`.
