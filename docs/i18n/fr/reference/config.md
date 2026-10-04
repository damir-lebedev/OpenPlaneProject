# CONFIG — `Config`, `Channels`, `Controls`

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/config.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Référence](README.md)

La couche de configuration n’est faite que de constantes `constexpr`, sans
code. La logique des classes ne doit contenir ni broches, ni délais, ni seuils
« magiques » : tout ce qu’il peut falloir changer pour un avion ou une carte
précis se trouve ici.

---

## namespace `Config`

**Fichier :** `include/config/Config.h` · **Dépend de :** `<stdint.h>` ·
**Utilisée par :** presque toutes les couches.

### Broches (dépendent de la carte)

Le bloc de broches est choisi par la macro que définit `[env:*]` dans
`platformio.ini` (`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`). Sans macro — `#error`. Le bloc STM32 est décrit
[plus bas](#stm32h743vit6-board_stm32h743).

| Constante | Type | Rôle | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | Ailerons | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | Profondeur | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | Régulateur du moteur | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | Direction + roue ; `-1` — la sortie est désactivée | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | RX du récepteur iBUS (UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | Le bus des capteurs (`Wire`) | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | Le bus de l’OLED (`Wire1`) ; `-1` — aucun | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | Le bus SPI commun | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | CS de l’IMU en SPI | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | CS du baromètre en SPI | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | L’UART du GPS ; TX `-1` — réception seule | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | Le numéro de l’UART matérielle du GPS | 2 | 0 | 2 |
| `PIN_AUX1`, `PIN_AUX2` | `int8_t` | sorties servo : largage de la charge, volets ; `-1` — aucune | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | le buzzer via un transistor ; `-1` — aucun | 38 | −1 | 2 |
| `PIN_AUX3`, `PIN_LIGHT`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_TX/RX` | `int8_t` | **S3 uniquement :** réservées pour la carte du contrôleur de vol ([FC_BOARD.md](../FC_BOARD.md)) | 47, 21, 8, 3, 9/10 | — | — |

Le bus SPI des capteurs s’appelle `PIN_SENSOR_SPI_*` et non `PIN_SPI_*` : dans
le cœur STM32duino (et d’autres cœurs Arduino), `PIN_SPI_SCK/MISO/MOSI` sont des
macros de la variante, et elles remplaceraient les constantes de `Config`.

<a id="stm32h743"></a>

#### STM32H743VIT6 (`BOARD_STM32H743`)

Il n’y a pas encore de carte : le brochage **n’est pas testé sur le matériel** (le micrologiciel tourne sur PC, env `native-stm32`). Les broches sont choisies parmi celles qui sont libres
sur la WeAct MiniSTM32H743VITx (la carte de l’env `stm32h743` de PlatformIO) et
recoupées avec les tables `PeripheralPins` de la variante STM32duino. Les
valeurs sont des macros de la variante (`PA0`…), c’est pourquoi, au début de
`Config.h`, sous `#if defined(BOARD_STM32H743)`, on inclut `<Arduino.h>`. Le
type de toutes les broches est `int16_t` (les broches analogiques portent le
numéro `0xC0 + N`). Il n’y a pas de numéros d’UART — le cœur choisit le
périphérique d’après les broches.

| Constante | Broche | Périphérique |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7 (TX — réservée pour iBUS-SENS) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2 — capteurs |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1 — l’écran (sur la WeAct — le connecteur de la caméra) |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 — charge utile / caméra |
| `PIN_BUZZER` | PE15 | GPIO — le buzzer |
| `PIN_VBAT_ADC`, `PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11 — réservées |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4 — le modem radio MAVLink (ces mêmes broches sont le FDCAN1) |

La console `Serial` est la LPUART1 (PA9 TX / PA10 RX), la valeur par défaut de
la variante.

### iBUS et perte de liaison

| Constante | Valeur | Signification |
|---|---|---|
| `IBUS_CHANNELS` | 10 | Combien de canaux de la trame sont utilisés |
| `IBUS_FRAME_LENGTH` | 32 | Longueur de la trame, octets |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | L’en-tête de la trame |
| `IBUS_BAUDRATE` | 115200 | Vitesse de l’UART |
| `RX_TIMEOUT_US` | 500 000 | Pas de trame correcte depuis plus longtemps que cela — la liaison est perdue |
| `RX_FAILSAFE_THROTTLE_US` | 950 | Gaz en dessous de cette valeur — le récepteur signale le failsafe de la radiocommande |

### GPS

| Constante | Valeur | Signification |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | Un NAV-PVT plus ancien que cela — `UbloxM10_Gps::isAvailable() == false` |

### Plage du PWM et débattements des gouvernes

| Constante | Valeur | Signification |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | L’impulsion RC standard, µs |
| `AILERON_MAX_US`, `ELEVATOR_MAX_US`, `RUDDER_MAX_US` | 500 / 500 / 300 | Débattement par rapport au centre à pleine course du manche, µs. La direction est plus faible : la roue du train d’atterrissage est sur le même servo |
| `THROTTLE_LIMIT_PCT` | 100 | Le plafond des gaz vers l’ESC, %, identique pour le manche et pour le pilote automatique (`FlightController::capThrottle`). Pour les essais sur banc avec une batterie 3S1P faible, on avait mis 50 ; les tests calculent la sortie attendue à partir de cette valeur |

### Volets (flaperons)

| Constante | Valeur | Signification |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 au-dessus de cette valeur — les volets sont sortis (pas 1500 : jusqu’à la première trame, les canaux = 1500) |
| `FLAPS_DEPLOYED_US` | 220 | Braquage vers le bas de chaque aileron, µs (~20° du palonnier du MG90S) |
| `FLAPS_TRANSITION_MS` | 1000 | Le temps de la sortie/rentrée complète |

### Sens des servos

`AILERON_LEFT_REVERSED`, `AILERON_RIGHT_REVERSED` (`true` — les servos des ailerons sont montés en miroir), `ELEVATOR_REVERSED` (`true`),
`RUDDER_REVERSED` — le seul endroit où l’inversion est définie. `ControlMixer`
calcule en signes physiques et ne change le signe qu’ici, donc les manches et
le pilote automatique ne peuvent pas diverger. L’inversion sur la radiocommande
**ne doit pas** être utilisée.

### Montage des capteurs

| Constante | Valeur | Signification |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | La rotation des axes de la puce de l’IMU autour de la verticale (0/90/180/270), vers où pointe l’axe X de la puce. N’est utilisée que tant qu’il n’y a pas de calibration du montage `o` dans la NVS |
| `MAG_ROTATION_CW_DEG` | 0 | Pareil pour la boussole (la boussole n’a pas de calibration du montage) |

### ARM

| Constante | Valeur | Signification |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 au-dessus de cette valeur — l’interrupteur ARM est activé |
| `THROTTLE_LOW_US` | 1050 | Gaz en dessous de cette valeur — « gaz en bas », on peut armer |

### Failsafe

| Constante | Valeur | Signification |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | Neutre des gouvernes |
| `FAILSAFE_THROTTLE` | 1000 | Moteur coupé |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | L’inclinaison du plané en cas de perte de liaison en l’air |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | Le tangage du plané (un peu sous l’horizon) |
| `FAILSAFE_RTH` | `true` | Avec le GPS et un point de départ, la perte de liaison en l’air — retour au point de départ avec le moteur, et non plané |

### Interrupteurs, tube de Pitot, pilote automatique

Les nombres de tous les modes et fonctions sont dans `Config.h`, à côté de
commentaires détaillés ; ce qu’ils signifient pour le pilote se trouve dans
[AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md).

| Groupe | Constantes |
|---|---|
| Interrupteurs | `SWITCH_ON_US` = 1750 (canal au-dessus de cette valeur — interrupteur activé ; pas 1500, pour que rien ne s’active avant la première trame) |
| Tube de Pitot | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| Stabilisation | `MAX_BANK_DEG` 45 (potentiomètre 15…60), `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| Navigation | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| Altitude | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| Gaz et vitesse | `CRUISE_THROTTLE_PCT` 55 (30…85), `CRUISE_AIRSPEED_MS` 14 (10…22), `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| Décrochage | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| Cercles et point de départ | `LOITER_RADIUS_M` 50 (25…150), `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| Géobarrière | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| Lancer à la main | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| Atterrissage | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| Vol à voile | `SOAR_*` : plané −3°, une ascendance > 0.5 m/s pendant 1.5 s, un cercle de 25°, sortie < −0.2 m/s pendant 8 s, moteur sous 30 m jusqu’à 100 m, retour au point de départ au-delà de 400 m |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| Auto-trim | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, enregistrement au sol : `AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| Coordination | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| Fonctions | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### Cycle, Wi-Fi, débogage

| Constante | Valeur | Signification |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | La période de la boucle de vol (500 Hz) ; aussi le `dt` nominal de `PidController` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | Le point d’accès du tableau de bord (le mot de passe est faible — un outil de banc) |
| `WEB_SERVER_PORT` | 80 | Port HTTP |
| `TELEM_BAUDRATE` | 57600 | La vitesse du modem radio MAVLink (la valeur par défaut de SiK) |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | L’adresse de l’appareil dans MAVLink |
| `DEBUG_INTERVAL_MS` | 100 | À quelle fréquence `DebugLogger` vérifie les canaux du journal |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | Tolérance aux rebonds du RC/PWM dans le mode « en cas de changement » |

### Boîte noire

En détail — [BLACKBOX.md](../BLACKBOX.md).

| Constante | Valeur | Signification |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 Mo / 32 Ko | La file des enregistrements en PSRAM (sans PSRAM — en mémoire interne) |
| `BLACKBOX_RING_STM32_BYTES` | 384 Ko | STM32H743 : la file en RAM — 10 s de pré-enregistrement et de la marge pour les retards de la carte |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 Mo | STM32H743 : le fichier sur la carte SD et le plafond de sa partie utilisée (le temps de vérification à la mise sous tension croît avec la zone) |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | Enregistrement avant le départ (ARM + gaz) et après le DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Armé, moteur arrêté, avion immobile pendant cette durée — arrêt |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | Ce qui compte comme « immobile » |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Après un redémarrage anormal — enregistrer au moins cette durée |
| `BLACKBOX_MIN_FREE_BYTES` | 10 Mo | Espace effacé tenu prêt ; les anciens vols sont effacés en entier au sol |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | La pause entre les effacements |
| `BLACKBOX_IMU_DIVIDER` | 1 | L’IMU tous les N cycles (1 — 500 Hz) |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | Les diviseurs de la batterie (56k/10k) et du capteur de courant (10k/15k) sur la carte du contrôleur de vol |

---

## namespace `Channels`

**Fichier :** `include/config/Channels.h` · **Dépend de :** `<stdint.h>`

Le seul endroit où le numéro physique d’un canal est relié à son rôle. Les
valeurs sont des **indices** (à partir de 0) dans `RcChannelState`.

| Constante | Indice | Canal | Élément de la FS-i6 | Rôle |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | manche droit ←→ | Roulis |
| `ELEVATOR` | 1 | CH2 | manche droit ↑↓ | Tangage (2000 = vers l’avant = nez en bas) |
| `THROTTLE` | 2 | CH3 | manche gauche ↑↓ | Gaz |
| `RUDDER` | 3 | CH4 | manche gauche ←→ | Direction + roue |
| `ARM` | 4 | CH5 | SwA | L’interrupteur ARM (ne peut pas être réaffecté) |
| `SWB` | 5 | CH6 | SwB | selon le tableau de `Controls.h` (par défaut, les volets) |
| `SWC` | 6 | CH7 | SwC (3 pos.) | par défaut, le mode MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | par défaut, RTH |
| `VRA` | 8 | CH9 | VrA | par défaut, `STAB_GAIN` |
| `VRB` | 9 | CH10 | VrB | par défaut, `CRUISE_SPEED` |
| `COUNT` | 10 | | | le nombre de canaux |

---

## namespace `Controls`

**Fichier :** `include/config/Controls.h` · **Dépend de :** `ControlBinding.h`, `Channels`

`constexpr Binding BINDINGS[]` — ce que fait chaque interrupteur et chaque
potentiomètre, **une ligne par canal** (`Bind::modes/mode/feature/knob`, voir
[autopilot.md](autopilot.md#binding-bind-bindingcheck)). À côté se trouvent des
idées toutes prêtes, en commentaire. Trois `static_assert` attrapent les erreurs
du tableau à la compilation : un manche ou l’ARM dans le tableau, un canal hors
plage, un canal répété, plus d’un interrupteur de sélection de mode.
