# TESTING.md — tests, couverture et analyse statique

> 🌐 Cette page est la traduction de l’[original en russe](../../TESTING.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Le firmware est vérifié à deux niveaux :

| Où | Commande | Quoi |
|---|---|---|
| **PC (native)** | `pio test -e native` | Les en-têtes du firmware sont compilés sur le PC sans modification, le matériel étant remplacé par des fakes pilotables : modules, pilotes, simulations de vol en boucle fermée, tout le firmware de l’ESP32 (S3 et 38 broches) avec chaque kit de capteurs. La couverture est calculée |
| **PC (native-stm32)** | `pio test -e native-stm32` | Tout le firmware de la STM32H743 (`src/stm32/main.cpp`) au-dessus d’une couche de fakes de STM32duino : tâches FreeRTOS, flash, MAVLink, capteurs sur I2C et SPI |
| **Matrice de compilations** | `tools/build_matrix.sh` | 4 cartes × 6 kits de capteurs avec `-Wall -Wextra (-Wshadow)` ; tout avertissement dans le code du projet est une erreur |
| **Description pour l’application** | `python tools/test_opconfig.py` | `tools/opconfig.py` : description du firmware et écriture de la configuration pour OpenPlane Studio, sur une copie des sources (14 tests) ; `tools/opconfig_build_check.py` — la configuration écrite se compile sur toutes les cartes |
| **Carte** | `pio test -e esp32-s3` | `test_feedback` et `test_imu_orientation` sur un vrai ESP32-S3 (il flashe un firmware de test ; ensuite, remettez le firmware normal : `pio run -t upload`) |
| **Carte STM32** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | La boîte noire sur une **vraie carte SD** de la DevEBox H743, plus `test_feedback` et `test_imu_orientation` sur un Cortex-M7 — [plus bas](#tests-sur-la-carte-stm32) |

Le contexte d’architecture se trouve dans [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-testabilité).

---

## Démarrage rapide

```bash
pip install platformio gcovr        # une seule fois
# Windows : il faut g++ dans le PATH, par exemple WinLibs (winlibs.com, zip UCRT) :
# décompressez-le et ajoutez mingw64\bin au PATH — aucune installation nécessaire
pio test -e native -e native-stm32  # tous les tests natifs (~1,5 min)
gcovr                               # couverture par fichier (réglages — gcovr.cfg)
tools/build_matrix.sh               # toutes les cartes × tous les capteurs (~25 min)
python tools/test_opconfig.py       # description et réglage du firmware pour OpenPlane Studio (~3 s)
python tools/opconfig_build_check.py # ce qu’écrit l’application se compile sur toutes les cartes (~10 min)
gcovr --html-details -o coverage/index.html   # rapport HTML (coverage/ est dans .gitignore)

pio test -e native -f native/test_rc          # un seul jeu
pio test -e native -f test_feedback           # simulation de la rétroaction sur le PC

# Trajectoires des simulations en boucle fermée en CSV (pour des courbes) :
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# Le flux MAVLink — à vérifier avec un décodeur de référence (pip install pymavlink) :
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
```

Avant de calculer la couverture après des modifications des tests, il est utile de repartir d’une compilation propre : `rm -rf .pio/build/native` ; sinon, les compteurs des exécutions précédentes se retrouvent dans le rapport.

---

## Comment est faite la compilation native

`[env:native]` dans `platformio.ini` : `platform = native`, Unity, `-std=gnu++17`, `-D BOARD_ESP32_S3` (le brochage de l’ESP32-S3), `-I test/native/support`, `-Wall -Wextra -Wshadow`, couverture avec `--coverage` et `-fkeep-inline-functions -fkeep-static-functions` — sans eux, gcov ne voit pas les fonctions d’en-tête jamais appelées et surestime la couverture.

### Fakes du matériel — `test/native/support/`

Des en-têtes portant les mêmes noms et signatures que le cœur Arduino pour ESP32 2.0.x, ESP-IDF et les bibliothèques, mais au-dessus d’un monde simulé dans `namespace fake` :

| Fichier | Remplace | Ce que sait faire la simulation |
|---|---|---|
| `Arduino.h`, `Print.h`, `WString.h`, `Stream.h` | le cœur Arduino | Macros (`constrain`, `sq`, `DEG_TO_RAD`…), `map()`, `String`, formatage de `print()` comme l’original. `ARDUINO` n’est volontairement **pas** défini |
| `esp32-hal-fake.h` | temps, GPIO, ADC, LEDC, FreeRTOS, PSRAM, `ESP` | L’horloge n’avance que par `fake::advance*()`/`delay()` ; `millis()/micros()` sont des `uint32_t`, comme sur l’ESP32 (le débordement se comporte comme sur la carte). Canaux LEDC, `pulseIn` d’après le rapport cyclique réel (visible seulement si le tampon d’entrée de la broche est activé), `analogReadMilliVolts` — la tension vient de `fake::gpio().analogMv`. Les tâches sont enregistrées (le handle est non nul) ; `fake::runTask(task, n)` exécute n passages de sa boucle infinie, `ulTaskNotifyTake` compte pour un passage, `xTaskNotifyGive` pour un compteur. Les mutex de FreeRTOS sont un indicateur « occupé ». `psramFound()`/`ps_malloc()`. Les sections critiques sont comptées |
| `HardwareSerial.h` | UART | Les ports sont enregistrés par numéro (`fake::uart(1)`) ; `pushRx()`, `txBytes()`, changement de vitesse à la volée (`updateBaudRate`, l’historique est `baudChanges()`). `Serial` = UART0 |
| `esp_partition.h` | partitions de flash d’ESP-IDF | Une partition est un vecteur d’octets au comportement NOR : effacement uniquement par secteurs de 4 Ko, effacé = 0xFF, une écriture ne fait que baisser des bits (la tentative de remonter un bit est comptée — `bitRaises`) ; `beforeWrite` — « l’alimentation a disparu » ; compteurs de lectures, d’écritures, d’effacements |
| `esp_system.h` | la cause du redémarrage | `esp_reset_reason()` d’après `fake::chip().resetReason` |
| `Wire.h` | I2C | Des périphériques par adresse ; `fake::RegisterMapDevice` — registres à auto-incrémentation, journal des écritures, pannes (`present`, `failWrites`, `failReads`, `failReadIf`, `shortRead`), crochets `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | Des périphériques par broche CS ; `fake::SpiRegisterMapDevice` — le protocole Bosch/InvenSense, `dummyBytes` avant les données |
| `Preferences.h` | NVS | Stockage en mémoire, comportement de `begin(readOnly)`/`get*`/`getBytes` comme l’original ; `failBegin` |
| `WiFi.h`, `WebServer.h` | point d’accès Wi-Fi, HTTP | Le résultat de `softAP()` est fixé par le test ; `WebServer::request(méthode, uri, corps)` appelle le gestionnaire enregistré ; `fake::webServers()` — toutes les instances |
| `U8g2lib.h` | U8g2 | À la place des pixels — une liste des chaînes et des rectangles dessinés ; `begin()/sendBuffer()` font passer des octets par un callback d’octets de l’utilisateur ; `fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`, `GPIO_PIN_MUX_REG`, `PIN_INPUT_ENABLE` |

### La couche STM32duino — `test/native/support_stm32/` (env `native-stm32`)

Elle est placée dans `-I` avant `support/` et complète les mêmes fakes avec ce que seul STM32duino possède. `<Preferences.h>` dans cet environnement est le `include/hal/stm32/compat/Preferences.h` **réel**, au-dessus de `KeyValueStore`.

| Fichier | Remplace | Ce qu’il sait faire |
|---|---|---|
| `Arduino.h` | le cœur STM32duino | broches `PA0..PE15` (port·16 + numéro), `pin_size_t`, `PinMap_TIM` pour les broches de sortie des servos, `HardwareTimer` (l’impulsion se voit par `fake::timerPulseUs(pin)` et `pulseIn()`), `Uart`, `noInterrupts()` |
| `STM32FreeRTOS.h` | le FreeRTOS de STM32duino | `xTaskCreate` dans le registre commun des tâches (pile en mots), `vTaskStartScheduler()` retourne — les tâches sont actionnées par le test lui-même (`fake::runTask`), `xPortGetFreeHeapSize` |
| `EEPROM.h` | émulation d’EEPROM | une « flash » de 8 Ko (effacée = 0xFF) et un tampon, `fake::eeprom()` — compteurs et corruption de l’image |
| `SPI.h` | | `SPIMode` |

Dans les fakes communs, il a été ajouté pour STM32 : `TwoWire(sda, scl)`, `setSDA/SCL` et `fake::wireWithSda(pin)` (pour trouver le second bus de la carte), `HardwareSerial(rx, tx)` et `fake::uartByRx(pin)`, `SPIClass::setSCLK/MISO/MOSI`.

### Émulateurs de puces et modèle d’avion — `test/native/helpers/`

| Fichier | Ce que c’est |
|---|---|
| `ChipEmulators.h` | LSM6DSV, ICM-45686 (avec registres indirects IPREG), QMC6309, SPL06-001, BMP581, trames NAV-PVT d’u-blox — cartes de registres sur I2C ou SPI, avec des données issues du « monde » `World` (angles et vitesses, altitude, vitesse air, cap, coordonnées), dans les axes de la puce en tenant compte de `IMU_ROTATION_CW_DEG` |
| `PlaneSim.h` | un modèle d’avion de ~1,2 kg : une masse ponctuelle + rotation en roulis/tangage, CL(α) avec décrochage, traînée, poussée, vent, thermiques, sol; train d’atterrissage à roulette de queue ou de nez — l’avion bascule sur les roues principales, on compte la charge sur la roulette de queue, les chocs de la queue et les contacts de l’hélice, le lacet au sol avec le couple de l’hélice et la roue directrice |
| `SimHarness.h` | une boucle fermée : radio → trame iBUS → `RcReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → PWM → braquages des gouvernes → `PlaneSim` → capteurs (y compris un tube de Pitot sur deux baromètres bruités). Une trajectoire CSV avec `OPENPLANE_SIM_DIR`; avec `realisticImu`, le vrai `AttitudeEstimator` calcule les angles à partir de la force spécifique — il voit l’accélération sur la piste comme à bord |

`test/native/helpers/TestSupport.h` regroupe ce que les jeux de tests ont en commun : `resetWorld()` (appelé depuis `setUp()`), les doublures `FakeUart`/`FakeServo`/`FakeBoard` et celles des capteurs (`FakeImu`, `FakeBaro`, `FakeMag`, `FakeGps`), le constructeur de trames `ibusFrame()`, les bancs `I2cRig`/`SpiRig` (un pilote au-dessus des vrais `Esp32I2CBus`/`Esp32SpiBus` et d’un `*RegisterDevice` avec une puce simulée).

Les tests de la carte (`test_feedback`, `test_imu_orientation`) sont portables : avec `ARDUINO`, `setup()/loop()` ; sinon, `main()`. Les jeux `test/native/*` ne sont pas compilés pour la carte (`test_ignore` dans `[esp32_common]` et `[env:stm32h743]` : les motifs vont un par ligne — séparés par un espace, PlatformIO les lit comme un seul). Sur STM32 : `pio test -e stm32h743`.

---

## Jeux de tests

| Jeu | Tests | Ce qu’il vérifie |
|---|---|---|
| `native/test_hal` | 19 | Les utilitaires de `II2CBus` (NACK, lecture courte — le tampon n’est pas touché), `I2cRegisterDevice`, `SpiRegisterDevice` (bit de lecture, octet factice du BMP388), `Esp32I2CBus` (délai d’attente de 5 ms), `Esp32SpiBus` (modes 0–3), `Esp32UartPort` (8N1, broches), `Esp32ServoOutput` (50 Hz/14 bits, limitation de l’impulsion, panne du LEDC, mesure via le tampon d’entrée), `Esp32Board` (bus, UART, ordre des canaux, AUX, buzzer); « pas d’impulsions » (`stopPulses()`) pour le signal moteur ; SBUS — 8E2 et inversion du RX dans `Esp32UartPort` |
| `native/test_rc` | 33 | `RcChannelState`, `RcInput`, le récepteur : iBUS — trames en morceaux, CRC, valeurs sur 12 bits, failsafe de la radio, délai de 500 ms (y compris à travers le débordement de `micros()`), déchets, resynchronisation ; SBUS — 8E2 avec inversion, indicateurs failsafe et trame perdue, SBUS2, synchronisation au milieu du flux, port sans inversion ; CRSF — CRC8 contre la référence, canaux, qualité de liaison 0 % = perte de liaison, silence = délai dépassé, trames étrangères et déchets ; 16 canaux de 11 bits |
| `native/test_control` | 26 | Volets (vitesse, premier appel, pauses), le mixeur (signes, inversion, flaperons), gaz, la machine à états de l’ARM et les vérifications des capteurs des modes, la table des sorties et l’autotest des impulsions, impulsion de l’ESC bidirectionnel, inversion du moteur uniquement en DISARM et ARM bloqué après un changement de sens, aides d’inversion (miroir d’impulsion, voie inversée, moteur inversé); AUTO_TAKEOFF avec roulette de queue — seulement avec la calibration du train |
| `native/test_autopilot` | 22 | PID (terme D issu de la vitesse du capteur, intégrale, anti-windup, `dt`), STABILIZE comme mode d’angles, décollage automatique (moteur arrêté jusqu’au départ, gaz du programme, DISARM), ALT_HOLD par la gouverne de profondeur, plané en cas de perte de signal, perte de signal pendant la montée — la montée continue |
| `native/test_autopilot_modes` | 31 | Les 12 modes et la réaction de chacun à l’absence d’un capteur, la table des affectations et `static_assert`, fonctions et potentiomètres, navigation (cap, cercle, point de départ, géorepérage), failsafe RTH/plané, lancement à la main, vol à voile, auto-trim (écriture uniquement au sol) |
| `native/test_flight_controller` | 20 | Un cycle complet de `FlightController` sur les vraies classes : priorités perte de signal > ARM > sticks/pilote automatique > gaz ; AUX, `MOTOR_KILL`, buzzer ; inversion sur SwD : uniquement en DISARM, 3 s sans ARM, 50 % des gaz en arrière, pas de gaz du pilote automatique, le moteur s’arrête en cas de perte de liaison ; le potentiomètre de limite des gaz étire la plage autorisée sur toute la course du manche; calibration au sol par le geste de SwB et depuis la console, signaux sur les sorties (hochement des gouvernes, impulsions de l’ESC coupées seulement sans ARM), ARM bloqué pendant la calibration et le signal, un basculement d’ARM annule la calibration, un refus de décollage hoche sans toucher à l’ESC, le signal joue aussi sans liaison |
| `native/test_imu` | 21 | MPU6050/6500/9250 et ICM-42688 : identification, registres, échelles, rotation des axes et signes aéronautiques, erreurs de bus, étalonnage du gyroscope et vérification avant vol, étalonnage du montage à partir de trois positions, NVS, filtre d’orientation |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`, BMP388 sur I2C et SPI, BME280/BMP280 selon la référence de Bosch, boussoles (cap, étalonnage hard-iron dans la NVS), u-blox M10 (CFG-VALSET, NAV-PVT, trames corrompues, délai d’attente), `SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV (16X/32X, adresse alternative, SPI), ICM-45686 (registres indirects), QMC6309, SPL06-001 (formules de la fiche technique), BMP581 (DRDY et le chemin de secours), le tube de Pitot (zéro, filtre, densité, tuyaux inversés, données périmées, un « vol » avec le bruit de deux baromètres) |
| `native/test_storage` | 16 | `KeyValueStore` (rechargement, usure — une valeur identique n’est pas réécrite, débordement sans perte de données, CRC, coupure d’alimentation pendant l’effacement, données parasites, version du format), `KvPreferences` (se comporte comme la NVS de l’ESP32) |
| `native/test_mavlink` | 20 | Le codec face aux trames de référence de pymavlink (v1, v2, signé), CRC, resynchronisation ; télémétrie : fréquences des flux, HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS, paramètres du PID (liste, lecture, écriture, refus des valeurs erronées), changement de mode depuis le sol, ARM depuis le sol — refusé, missions — 0, un tampon UART saturé ne bloque pas la boucle |
| `native/test_blackbox` | 20 | La boîte noire : format et CRC, l’anneau de secteurs sur un fake de NOR (une partition neuve sans effacement, les données parasites sont toujours effacées, les anciens vols sont effacés en entier et seulement pour libérer de la place, le dernier n’est jamais touché, passage de la fin de l’anneau, la tête après un redémarrage, coupure d’alimentation, un enregistrement à moitié écrit détecté par le CRC), enregistrement du vol avec les vrais `FlightController`/`Autopilot` : démarrage sur ARM et gaz avec préenregistrement, arrêt après DISARM et « posé au sol », la perte de signal ne l’arrête pas, enregistrement après un redémarrage en défaut, démarrage manuel, événements, batterie, la flash s’est épuisée en vol, un vol plus long que la partition, téléchargement en trames avec CRC et changement de vitesse, le menu de la console `k`, pas de partition — désactivée; événements de la calibration au sol et des étapes du décollage automatique |
| `native/test_blackbox_scan` | 3 | La vérification par échantillonnage de l’anneau à la mise sous tension face à la vérification complète : 300 historiques aléatoires de l’anneau × 5 pas de sondage (la tête, les numéros et la liste des vols coïncident, et quand le tableau ne tient pas, elle s’efface devant la vérification complète) et le coût sur une zone de SD de 64 Mo (≈530 lectures au lieu de 32 000) |
| `native/test_telemetry` | 35 | `LoopStats`, `LogSettings` (NVS, version), `DebugLogger` (tous les canaux, NAV), `DebugConsole` (menu, raccourcis clavier, sondage des bus `b`, interdit pendant l’ARM, enregistrement seulement sans ARM), `WebDebugServer` (routes, JSON, boîte aux lettres), `OledDisplay` (octets sur I2C, trame, inversion en cas de perte de signal); calibration `c` et signal `e` dans la console, étape du décollage dans la ligne de journal `AP`, étape de calibration et étape du décollage sur l’OLED, identité du firmware sur la commande PC `id` (pour OpenPlane Studio), données en direct pour l’application (`AppLink` : trame `@live` avec les calibrations et la raison du refus d’ARM, trame sautée quand le tampon du port est plein), commandes `scan` et `act X` |
| `native/test_sim` | 18 | Vols en boucle fermée de tout le firmware avec le modèle d’avion : sortie d’une inclinaison, CRUISE par vent de travers, LOITER, RTH, failsafe RTH/plané, géorepérage, décollage automatique avec roulette de queue (charge sur la roulette jamais au-dessus de la charge à l’arrêt, ni l’hélice ni la queue ne touchent la piste, un filtre d’angles honnête ; avec et sans baromètre), avec roulette de nez, un décollage interrompu par les gaz, lancement à la main, atterrissage automatique, une thermique, RESCUE depuis une spirale, maintien de la vitesse et protection contre le décrochage, un vrai tube de Pitot dans la boucle, auto-trim d’un avion « bancal », pannes de capteurs en vol (IMU, baromètre, tube de Pitot, GPS) |
| `native/test_takeoff` | 23 | Décollage automatique et calibration au sol de l’avion : horizon depuis la pose « queue levée » et assiette au sol (`ImuOrientation`), filtre d’angles sans accéléromètre pendant la course, calibration du train en flash d’un vrai `ImuSensorBase` (horizon après une mise sous tension sur les roues, `o` efface l’assiette au sol, affinage du zéro du gyroscope), `GroundCalibration` (deux étapes, contrôle des angles, annulation, geste à l’interrupteur), `SignalPlayer`, étapes de `TakeoffController` pour roulette de queue et de nez, lois des gouvernes, décollage interrompu (gaz, liaison, panne de l’IMU, roulis, cap, nez, queue, pas de décollage), sans baromètre selon le temps, la fenêtre « gyroscope seul » |
| `native/test_feedback_units` | 14 | Les modules de rétroaction un par un : sources de vitesse, en l’air/au sol, l’estimation RLS, le régulateur, signes de décrochage, annulations du décollage et de l’atterrissage |
| `native/test_app` | 10 | `src/main.cpp` sur l’ESP32-S3 avec le kit de banc MPU6500/BMP581/QMC5883P/OLED : période de `loop()`, radio → servos, ARM, modes, perte de signal, console, tableau de bord, écran, boîte noire (une tâche sur le cœur 0, enregistrement sur les gaz, vol après DISARM, `bb list`) |
| `native/test_app_lsm6dsv_pitot` | 9 | `src/main.cpp` sur l’ESP32-S3 avec le kit de vol : LSM6DSV + QMC6309 + SPL06 + BMP581 dans le tube + GPS — identification de toutes les puces, zéro du tube et vitesse, altitude, point de départ d’après le GPS, STABILIZE d’après les angles de la puce, RTH vers le point de départ, sondage des bus, tableau de bord ; SwD (inversion) sans effet en vol jusqu’au DISARM |
| `native/test_app_icm45686_esp32dev` | 5 | `src/main.cpp` sur l’**ESP32 38 broches** (`BOARD_ESP32_CLASSIC`) avec le kit ICM-45686 + QMC6309 + SPL06 + BMP581 : brochage de la carte, filtres IPREG, lancement à la main, stabilisation et vitesse, sondage d’un seul bus |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | `src/stm32/main.cpp` sur la **STM32H743** avec le kit de vol : tâches et priorités, période de 2 ms, le tube, minuteries PWM et `pulseIn`, MAVLink en vol, changement de mode depuis la GCS, réglages écrits par une tâche d’arrière-plan dans la « flash », l’écran sur I2C1, la console |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743 avec l’ICM-45686 et le BMP581 **sur SPI** + QMC6309 : flash corrompue à la mise sous tension, ALT_HOLD depuis la GCS maintient l’altitude, perte de signal → RTH, visible dans MAVLink ; réécriture d’une image endommagée |
| `native_stm32/test_blackbox_sd` | 29 | La boîte noire sur la carte SD : FAT32 (avec et sans MBR, un répertoire sur deux clusters, entrées parasites, un volume étranger/dispersé/vide), `SdFileRegion` (blocs incomplets, cache, effacement, limites, pannes), le vrai pilote `Stm32SdCard` au-dessus d’un faux `HAL_SD` (4 bits, vitesses de repli, nouvelle tentative, carte occupée, tampons non alignés), l’anneau sur la carte (redémarrage, coupure d’alimentation, coût de la vérification), le repère « anneau vide », enregistrement du vol sur un `FlightController`, un redémarrage en défaut détecté via `RCC->RSR`, l’ADC de la batterie, erreurs de la carte en vol, une carte lente, téléchargement par la console, la touche `D` |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | `src/stm32/main.cpp` avec une carte : le démarrage trouve la carte et le fichier, la tâche `bbox` écrit le vol, la période de la boucle ne s’allonge pas, `bb list` |
| `native_stm32/test_app_stm32_no_sd` | 4 | `src/stm32/main.cpp` sans carte : la boîte noire est désactivée et explique pourquoi, l’avion vole, le menu `k` ne casse pas; autotest des sorties sur PD14 (contournement du bug de `pulseIn()` du cœur), signal moteur `e` ; SBUS sur `Stm32UartPort` — 8E2 et inversion du RX |
| `test_feedback` | 10 | Simulation en boucle fermée de l’avion avec la boucle de rétroaction (sur le PC et sur la carte) |
| `test_imu_orientation` | 5 | Étalonnage du montage de l’IMU sur 300 montages aléatoires (sur le PC et sur la carte) |
| **Total** | **455** | 406 dans `native` + 49 dans `native-stm32` (plus 9 seulement sur la carte — `test_blackbox_sd`) |

### Tests sur la carte STM32

`test/test_blackbox_sd` n’est pas natif : le pilote SDMMC, la carte et le temps sont réels. Les tests s’exécutent dans une tâche FreeRTOS, et à côté tourne une tâche imitant la boucle de vol, de priorité maximale (période de 2 ms) : elle préempte les tests au milieu des accès à la carte, comme dans le firmware. Sans elle, on ne peut pas attraper l’erreur qui a été trouvée sur la carte : lors de la préemption, la FIFO du SDMMC débordait (`HAL_SD_ERROR_RX_OVERRUN`), ce qui n’arrive pas dans une boucle nue.

| Test | Ce qu’il vérifie |
|---|---|
| `reset_cause_is_a_normal_one` | la cause du redémarrage (`RCC->RSR`) n’est ni le chien de garde ni une chute de tension |
| `card_is_detected_on_four_bit_bus` | la carte est reconnue sur un bus de 4 bits à 24 MHz |
| `file_is_found_and_contiguous` | `BLACKBOX.BIN` est trouvé sur FAT32 et occupe des blocs contigus |
| `multi_block_writes_work_at_every_length` | écriture de 1, 2, 4 et 8 blocs en un seul accès |
| `pages_write_with_bounded_latency_and_read_back_intact` | pages de 256 o : la pire écriture < 250 ms (la limite de la SD), de façon stable > 40 Ko/s, lecture et effacement |
| `header_scan_cost_on_the_whole_area` | le coût de la lecture de l’en-tête d’un secteur et de la vérification complète |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | effacement de tout, deux vols de 20 000 enregistrements, « redémarrage » : la vérification par échantillonnage prend < 2 s, les enregistrements sont lus dans l’ordre avec le bon CRC ; un anneau vide est reconnu grâce au repère en < 100 ms |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | le vrai `BlackBox` avec une IMU à 500 Hz en temps réel : pas un seul enregistrement perdu, le vol se relit après un « redémarrage » |
| `the_flight_task_was_not_disturbed` | l’écriture sur la carte n’a pas perturbé la période de la tâche d’imitation (écart < 3 ms) |

Lancement (une carte avec le fichier — `python tools/blackbox.py sd-prepare E:` ; **le test efface tous les vols du fichier**) :

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

La carte doit être en mode DFU (sur la DevEBox — le fil BT0→3V3 et RST, pilote WinUSB via Zadig ; détails dans [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743)). La console de la STM32 est un USB CDC : après le téléversement, le port n’apparaît pas tout de suite, et `pio test` n’arrive parfois pas à l’ouvrir à temps (« could not open port ») — dans ce cas, lancez `pio test ... --without-testing` et lisez la sortie avec n’importe quel programme de terminal dont le DTR est activé (les tests attendent jusqu’à 60 s que le port soit ouvert). Après les tests, la carte attend la touche **`D`** — elle la redémarre en DFU sans le fil.

Résultats sur la DevEBox H743 + une carte de 16 Go (2026-10-02) : `test_blackbox_sd` — 9/9, `test_feedback` — 10/10, `test_imu_orientation` — 5/5 ; les chiffres de vitesse de la carte sont dans [BLACKBOX.md](BLACKBOX.md#ce-qui-a-été-mesuré-sur-la-carte).

### Firmwares de banc — `test/bench/`

Ce ne sont pas des jeux de tests, mais de petits projets PlatformIO indépendants qu’on téléverse sur la carte à la place du firmware de vol (`pio test` ne les voit pas : les noms des dossiers ne commencent pas par `test_`). Les broches et les limites viennent du `Config.h` commun.

| Projet | Ce qu’il fait |
|---|---|
| `bench/elevator_sweep` | Fait balayer par programme le stick de la gouverne de profondeur (CH2) via `ControlMixer` et `FlightOutputs`, comme un vrai stick : vers le haut 100 % de la course, vers le bas 60 %, en douceur, avec des pauses ; 20 s de travail — 20 s au neutre. Aux positions extrêmes, il mesure l’impulsion sur les sorties. Gaz au minimum |

Pour téléverser : `pio run -d test/bench/elevator_sweep -t upload`. Pour remettre le firmware de vol : `pio run -e esp32-s3 -t upload`.

---

## Couverture

Elle est calculée par `gcovr` sur `include/` et `src/` (tout ce qui entre dans le firmware), sur les deux environnements natifs réunis : `gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`.

| Couche | Lignes | Branches |
|---|---|---|
| `autopilot` | 1328/1395 (95,2 %) | 921/1039 (88,6 %) |
| `autopilot/feedback` | 682/713 (95,7 %) | 502/571 (87,9 %) |
| `config` | 2/2 (100 %) | 2/2 (100 %) |
| `control` | 561/584 (96,1 %) | 438/469 (93,4 %) |
| `hal` | 238/301 (79,1 %) | 130/168 (77,4 %) |
| `hal/esp32` | 117/148 (79,1 %) | 32/38 (84,2 %) |
| `hal/stm32` | 241/288 (83,7 %) | 115/144 (79,9 %) |
| `rc` | 93/100 (93,0 %) | 44/46 (95,7 %) |
| `sensors` (tous) | 1533/1625 (94,3 %) | 835/994 (84,0 %) |
| `storage` | 350/364 (96,2 %) | 275/334 (82,3 %) |
| `telemetry` | 2775/2920 (95,0 %) | 2082/2464 (84,5 %) |
| `src` (`main.cpp`, `stm32/main.cpp`) | 145/150 (96,7 %) | 26/32 (81,2 %) |
| **Total** | **8065/8590 (93,9 %)** | **5402/6301 (85,7 %)** ; fonctions 1204/1245 (96,7 %) |

Ce qui reste non couvert, et pourquoi :

- **Lancement à la main** (`TakeoffSequencer` : `WaitLaunch`, `launchDetected()`) — inatteignable tant que `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false` ; il apparaîtra dans les tests lorsque la constante deviendra configurable (déménagement vers `Config.h`).
- **Ce qui dépend de la carte :** une sortie sans broche (`PIN_RUDDER = -1` n’existe que sur la C3), un GPS sans broche TX (C3) — les tests natifs exercent le brochage de la S3, de la 38 broches et de la STM32, mais pas celui de la C3 (la C3 est vérifiée par la matrice de compilations).
- **STM32 :** les branches d’erreur du cœur (pas de minuterie sur la broche, réserve de minuteries épuisée), le message `FreeRTOS не запустился` (« FreeRTOS n’a pas démarré ») — sur le PC, `vTaskStartScheduler()` retourne toujours.
- **Branches défensives** inaccessibles par l’API publique : `default`/`Count` dans un `switch` sur des énumérations, `return "?"`.
- **Destructeurs et implémentations par défaut des interfaces** (`IBoard`, `IUartPort`, `ImuSensor`, etc.) — toutes les implémentations les redéfinissent, et avec `-fkeep-inline-functions` gcov compte aussi ces lignes.
- Les fichiers sans lignes exécutables (`Channels.h`, `FeedbackConfig.h`, les structures `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/`FeedbackOutput`/`PhaseTargets`, les macros de `SensorSelection.h`, le HTML du tableau de bord) n’apparaissent pas dans le rapport — ils sont compilés dans les tests, mais gcov n’a rien à y compter.

---

## Analyse statique

| Outil | Commande | Profil |
|---|---|---|
| GCC | `tools/build_matrix.sh` (ou `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`) | Toutes les cartes × tous les kits de capteurs. La compilation native des tests utilise toujours `-Wall -Wextra -Wshadow` ; `stm32h743` utilise `-Wall -Wextra` (`build_src_flags` ; `-Wshadow` fait du bruit sur les en-têtes de STM32duino eux-mêmes) |
| cppcheck | `pio check -e esp32-s3` ; `pio check -e stm32h743` | `check_*` dans `[esp32_common]` : `include/` et `src/` (sauf `stm32/`), warning/style/performance/portability, suppressions en ligne `// cppcheck-suppress` uniquement pour les faux positifs (le callback d’U8g2, `setup/loop`). Pour `stm32h743` : les mêmes options sur `include/hal/stm32/` et `src/stm32/` |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy` : bugprone, clang-analyzer, performance, `misc-include-cleaner` et d’autres ; les vérifications désactivées sont expliquées dans le fichier lui-même |

clang-tidy est lancé avec les fakes de `test/native/support` : clang ne sait pas analyser les en-têtes d’ESP-IDF pour l’architecture de l’hôte (si l’on essaie `pio check` avec `clangtidy`, l’analyse s’interrompt sur des erreurs d’analyse syntaxique et, en toute honnêteté, ne vérifie rien). `misc-include-cleaner` veille à ce que chaque en-tête inclue ce qu’il utilise : les en-têtes « parapluie » (`FeedbackModules.h`, l’API de `IBoard.h`/`RegisterDevice.h`, les macros de `SensorSelection.h`) sont marqués `// IWYU pragma: export`. Le script saute le code STM32 (`include/hal/stm32/`, `src/stm32/`) — il est vérifié par la compilation, par cppcheck pour l’env `stm32h743` et par les tests de l’env `native-stm32`.

La matrice de compilations lors de la dernière passe — **24/24 sans avertissement** :

| Carte | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck (`esp32-s3`, `stm32h743`) — 0 remarque sur le code du projet.

---

## Comment écrire de nouveaux tests

1. Un module avec de la logique et sans matériel — un test unitaire direct : le temps est passé en paramètre ou avancé avec `fake::advanceMs()`.
2. Un pilote de puce — via `I2cRig`/`SpiRig` : les registres d’une puce simulée, vérification des valeurs écrites (`chip.lastWrite(reg)`) et de l’analyse des données. Pour les formules, une référence tirée de la fiche technique ou un calcul indépendant, et non une copie du code.
3. Les classes avec des tâches FreeRTOS infinies — `fake::findTask("nom")` + `fake::runTask(task, n)` ; c’est ainsi que l’on fait tourner aussi la tâche de vol de la STM32.
4. Le firmware entier avec un autre kit de capteurs ou une autre carte — un jeu à part qui, avant `#include "../../../src/main.cpp"`, définit `SENSOR_KIT` (ou `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`) ; les puces viennent de `helpers/ChipEmulators.h`. Pour la STM32 — `test/native_stm32/`.
5. Un nouveau mode du pilote automatique — un scénario de vol en boucle fermée dans `test_sim`.
6. Un nouveau jeu — un dossier `test/native/test_<nom>/test_main.cpp` avec `main()` ; `setUp()` appelle `resetWorld()` si le jeu n’a pas besoin d’état entre les tests.
7. Vous avez trouvé un bogue — d’abord un test qui l’attrape, puis la correction.
