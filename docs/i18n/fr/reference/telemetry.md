# TELEMETRY — journal, console, tableau de bord web, OLED, boîte noire

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/telemetry.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Référence](README.md)

La télémétrie est entièrement séparée de la logique de vol : elle ne fait que lire les
accesseurs constants de `FlightController`, `Autopilot`, des capteurs et de `LoopStats`.
Le seul chemin « de retour » est celui des commandes du tableau de bord, qui passent par la boîte aux lettres de
`WebDebugServer` et sont appliquées par la boucle de vol.

---

## `LoopStats`

**Fichier :** `telemetry/LoopStats.h` · **Genre :** struct

La fréquence et la durée de la boucle de vol.

| Membre | Description |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | Publiés une fois par seconde ; lus depuis d’autres tâches (valeurs de 32 bits — pas de lectures « déchirées ») |
| `void record(uint32_t durationUs)` | À appeler à chaque cycle depuis `loop()` |
| `uint32_t takePeakUs()` | Le pire cycle depuis l’appel précédent (pour la ligne SYS toutes les 10 s) ; à appeler depuis la même tâche que `record()` |

`maxUs` n’est que le pire de la dernière seconde ; un accroc rare se voit grâce à
`takePeakUs()`.

---

## `LogSettings`

**Fichier :** `telemetry/LogSettings.h` · **Dépend de :** `Preferences` (NVS, l’espace de noms `debuglog`)

### `LogChannel` (enum class)

| Canal | Préfixe | Ce qu’il affiche | Par défaut |
|---|---|---|---|
| `Status` | `STAT` | liaison, ARM, mode, volets, capteurs | au changement |
| `Rc` | `RC` | canaux de la radiocommande | désactivé |
| `Outputs` | `OUT` | sorties vers les gouvernes et l’ESC | désactivé |
| `Attitude` | `ATT` | roulis, tangage, cap | désactivé |
| `Autopilot` | `AP` | consignes et corrections | désactivé |
| `Altitude` | `ALT` | altitude, vitesse verticale | désactivé |
| `Heading` | `MAG` | cap du compas | désactivé |
| `Gps` | `GPS` | satellites, coordonnées | désactivé |
| `Imu` | `IMU` | gyroscope et accéléromètre | désactivé |
| `Nav` | `NAV` | point de départ, cap, vitesse, tube de Pitot, fonctions activées | désactivé |
| `System` | `SYS` | fréquence de la boucle, mémoire (toutes les 10 s), seulement désactivé/activé | activé |
| `Count` | — | le nombre de canaux | — |

`LogMode` (enum class) : `Off`, `OnChange`, `Periodic`.

`LogChannelInfo` : `tag`, `title`, `periodicOnly`, `defaultMode`.

| Méthode | Description |
|---|---|
| `static constexpr uint8_t COUNT`, `PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | Une ligne du tableau des canaux |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms (en boucle) |
| `LogSettings()`, `void setDefaults()` | Les modes par défaut, une période de 1 s |
| `LogMode mode(uint8_t)`, `mode(LogChannel)` | Le mode d’un canal |
| `void setMode(uint8_t, LogMode)` | Pour les canaux `periodicOnly`, `OnChange` devient `Periodic` |
| `void cycleMode(uint8_t)` | désactivé → au changement → continu → désactivé (SYS : désactivé ↔ activé) |
| `void setAll(LogMode)` | Pour tous les canaux ; la commande « tout au changement » ne touche pas SYS |
| `uint16_t periodMs() const`, `void cyclePeriod()` | La période du mode « continu » |
| `static const char* modeName(LogMode, bool periodicOnly)` | « désactivé » / « au changement » / « continu » (ou « activé ») |
| `void load()` | Depuis la NVS ; si `VERSION` ou la longueur ne correspondent pas, les valeurs par défaut restent ; un code de mode inconnu → la valeur par défaut du canal |
| `void save() const` | Vers la NVS (les modes, la période, la version) |

`VERSION` change en même temps que la liste des canaux — les anciens réglages sont réinitialisés
(`VERSION = 2` : le canal NAV a été ajouté). Les touches des canaux dans le menu : `1`..`9`, NAV —
`n`, SYS — `s`.

---

## `DebugLogger`

**Fichier :** `telemetry/DebugLogger.h` · **Dépend de :** `FlightController`, `Autopilot*`, `LoopStats*`, `LogSettings`, `Config`

Affichage de l’état dans le moniteur série, canal par canal : chacun a sa propre ligne, son propre
mode et ses propres tolérances.

| Méthode | Description |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | Une fois par `DEBUG_INTERVAL_MS`, parcourt les canaux (reste silencieux en pause et tant que le menu est ouvert) |
| `LogSettings& getSettings()`, `void saveSettings() const` | Pour le menu de la console |
| `void suspend(bool)` | Le menu est ouvert — se taire ; au relâchement — `refresh()` |
| `void setPaused(bool)`, `bool isPaused() const` | Pause avec la barre d’espace ; au relâchement — `refresh()` |
| `void refresh()` | Le prochain cycle affichera tous les canaux activés |

La logique d’un canal (`updateChannel`) :

- `Off` — ne rien afficher ;
- `Periodic` — une fois par `periodMs()` (SYS — une fois toutes les 10 s), valeurs « telles quelles » ;
- `OnChange` — la ligne est assemblée avec des **tolérances** (le `Shown` imbriqué garde
  l’ancienne valeur tant que la nouvelle ne s’en écarte pas de plus que la tolérance : RC/PWM 3 µs, angles
  0,5°, cap 1°, corrections 2, altitude 0,3 m, accélération 0,03 g, coordonnées
  1e−5°) et n’est affichée que si elle diffère de la dernière affichée.

Les types imbriqués : `LineBuffer : Print` (une ligne d’au plus 200 octets pour comparer avant
l’affichage), `Shown` (une valeur avec hystérésis).

Les formats des lignes :

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  cible 0.0 m
MAG  cap 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (le pire sur 10 s) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=` distingue `LOST(pas de trames)` de `LOST(failsafe de la radiocommande)` ; `IMU=` vaut
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK`.

---

## `DebugConsole`

**Fichier :** `telemetry/DebugConsole.h` · **Dépend de :** `FlightController`, `FlightOutputs`, `Autopilot`, `DebugLogger`, `LogSettings`, `IBoard*` (balayage des bus)

Un menu textuel dans le moniteur série. Un automate d’écrans `Screen::{None, Main, Log}`.

| Méthode | Description |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | avec une carte — la commande `b` et l’entrée 7 du menu |
| `static const char* guessI2cDevice(uint8_t address)` | la puce d’après l’adresse : 0x6A LSM6DSV, 0x68 MPU/ICM, 0x76 BME280/BMP388/SPL06, 0x46/0x47 BMP581, 0x7C QMC6309, 0x2C QMC5883P, 0x0D QMC5883L, 0x3C OLED |
| `void printHint() const` | Une aide d’une ligne |
| `void update()` | Traiter tous les octets de `Serial` ; si les réglages du journal ont été modifiés, que le menu est fermé et que l’appareil **n’est pas armed** — enregistrer dans la NVS |

Raccourcis clavier (hors menu) : `h`/`?` — le menu principal ; `l` — le menu du journal ; espace —
pause du journal ; `s` — état des capteurs ; `i` — étalonnage du gyroscope ; `o` —
étalonnage du montage de l’IMU ; `m` — étalonnage du compas ; `p` — autotest des sorties ;
`b` — balayage des bus I2C (0x08..0x7F — jusqu’à 0x7F, parce que le QMC6309 se trouve en 0x7C) avec
les noms des puces ; toute autre touche — une aide. `\r`/`\n` sont ignorés.

Le menu du journal : `1`..`9` — fait tourner le mode des canaux 0..8, `n` — NAV, `s` — SYS, `p` — période, `a` —
tout « au changement », `x` — tout désactivé, `d` — valeurs par défaut, `0`/`q` — retour, `l`/`h` —
fermer.

Les actions bloquantes (`i`, `o`, `m`, `p`) sont **interdites en ARM**. Tant que le menu
est ouvert, le journal est suspendu (`DebugLogger::suspend`). La largeur des entrées du menu
est comptée en caractères UTF-8 et non en octets (le cyrillique occupe 2 octets).

---

## `WebDashboardPage`

**Fichier :** `telemetry/WebDashboardPage.h` · **Genre :** namespace

`static const char HTML[] PROGMEM` — la page entière (HTML + CSS + JS) en un seul
littéral. Tout ce qui est dynamique est construit par le navigateur d’après le JSON de `/api/status` (interrogé
toutes les 200 ms) : les lignes des canaux, des sorties et des capteurs sont créées d’après les clés du JSON, si bien qu’une nouvelle
sortie apparaît sans modifier la page. Un champ PID que l’utilisateur a commencé
à modifier n’est plus écrasé par l’interrogation.

---

## `WebDebugServer`

**Fichier :** `telemetry/WebDebugServer.h` · **Dépend de :** `WebServer`, `WiFi`, `FlightController`, `Autopilot*`, `WebDashboardPage`, `Config`

| Méthode | Description |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | Un point d’accès Wi-Fi (`persistent(false)` — rien n’est écrit en flash), les routes, la tâche `web` sur le cœur 0. `false` si le point d’accès n’a pas démarré |
| `void applyPendingCommands()` | À appeler depuis la boucle de vol : prend les commandes sous un spinlock et les applique au pilote automatique |

Les routes :

| Route | Réponse |
|---|---|
| `GET /` | La page du tableau de bord |
| `GET /api/status` | Le JSON d’état (`buildStatusJson()`), le format est dans le [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}` ; sans corps → 400 `no data` ; sans pilote automatique → 503 ; mode incorrect → 400 `invalid mode` |
| `POST /api/setpid` | N’importe lesquels de `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch` ; les omis restent inchangés |
| le reste | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }` — une boîte aux lettres sous un
`portMUX`. `extractJsonNumber(body, key, fallback)` — une analyse minimale de
JSON plat sans ArduinoJson : `"key"`, des espaces, `:`, des espaces, un nombre dans
n’importe quelle notation JSON (signe, fraction, exposant `1e-7`) ; si la clé ou le nombre manque —
`fallback`.

Dans le JSON, les champs `attached`/`available` sont présents **toujours** ; les données du capteur, seulement
avec `available: true`.

---

## `OledDisplay`

**Fichier :** `telemetry/OledDisplay.h` · **Dépend de :** U8g2, `II2CBus`, `FlightController`, `Autopilot*`, `LoopStats`

Un SSD1306 128×64 (I2C 0x3C) sur le second bus I2C ; sa propre tâche `oled`
(`Rtos::startTask` : cœur 0 sur l’ESP32, faible priorité sur le STM32), toutes les 200 ms.

| Méthode | Description |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr` ou l’écran ne répond pas en 0x3C → `false` ; sinon configure U8g2 et lance la tâche |

U8g2 envoie les octets par un `byteCallback` au-dessus de `II2CBus` (l’écran ne sait rien de
`Wire1`). Un rappel en C ne reçoit pas de contexte, c’est pourquoi le bus est conservé dans une variable
statique `busSlot()` — il n’y a qu’un seul écran à bord.

L’écran :

```
RX ok ARM STAB FL       liaison (perte — ligne inversée) / ARM / mode / volets
R  +1.2 P  -0.4         roulis / tangage, °           (IMU --)
Alt +0.3 Vz +0.1 A14    altitude / vitesse verticale / vitesse de l’air, s’il y a un tube de Pitot (BARO --)
H123 T1000 Y1500        cap / gaz / gouverne de direction (H---)
L1500 R1500 E1500       ailerons / gouverne de profondeur
Loop 500Hz max1100us    fréquence et le pire cycle sur la seconde
```

Les noms courts des modes sont `AutopilotNames::modeShort()` (`MAN`, `STAB`,
`TKOFF`, `ALT`, `ACRO`, `CRZ`, `LOIT`, `RTH`, `LNCH`, `LAND`, `SOAR`, `RESQ`) ;
en cas de perte de la liaison en l’air — `GLIDE` ou `FSRTH`.

---

## `BlackBox`

**Fichier :** `telemetry/BlackBox.h` · **Dépend de :** `FlightController`, `Autopilot`, `LoopStats`, `BlackBoxStorage`, `PilotSwitches*`

L’enregistrement du vol en flash (ESP32-S3) ou sur une carte SD (STM32H743). Quoi récupérer, quand et comment — voir [BLACKBOX.md](../BLACKBOX.md).

| Méthode | Description |
|---|---|
| `bool begin(bool startTask = true)` | Lit le support (`BlackBoxStorage::begin()`), alloue la file (PSRAM sur l’ESP32, `malloc` sur le STM32), vérifie l’espace effacé (jusqu’à 0,3 s), lance la tâche `bbox` (`Rtos::startTask`). S’il n’y a pas de place pour enregistrer (partition, carte, fichier) — `false`, la boîte noire est désactivée |
| `void update(uint32_t workUs)` | Depuis `loop()` après chaque cycle : événements, démarrage/arrêt, instantanés dans la file, réveille la tâche d’écriture |
| `void writerStep()` | Un pas de la tâche d’écriture : une ou deux pages en flash, ou un effacement au sol |
| `requestManualStart()` / `requestManualStop()` | Enregistrement manuel (console `k` → `r`) |
| `State getState()` / `bool isRecording()` | `Off`, `Idle`, `Recording`, `Stopping` (écrit le reste de la file avant d’enregistrer END) |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | Pour la console |
| `void handleHostCommand(const char*)` | `bb list`, `bb get <n> [bauds]` — pour `tools/blackbox.py` (sur USB CDC, la vitesse n’a aucune influence) |

Les points propres à chaque plateforme : la cause du redémarrage — `readResetCause()` ; la tension et le courant de la batterie —
le CAN (`analogReadMilliVolts` sur le S3, un `analogRead` de 12 bits sur le STM32) ; les erreurs du support
(`BlackBoxStorage::writeErrors`/`eraseErrors`) entrent dans le journal une fois par seconde
sous la forme de l’événement « support : erreurs d’écriture … » et ne gênent pas le vol.

## `BlackBoxStorage`

**Fichier :** `telemetry/BlackBoxStorage.h` · **Dépend de :** `IFlashRegion`

Un anneau de secteurs de 4 Ko : la tête et la liste des vols viennent des en-têtes de secteurs lors de `begin()` (la première passe lit l’en-tête de chaque secteur et retient les authentiques, la seconde seulement ceux-là : une zone vide n’est lue qu’une fois) ; `openFlight()`/`append()`/`flush()`/`closeFlight()` — écriture par pages (un CRC-8 sur chaque enregistrement) ; `eraseStep(target, protect, allowErase)` — un pas de vérification/effacement devant la tête : les déchets — toujours, les vols — en entier et seulement tant qu’il reste moins de `target` de libre ; `protect` n’est jamais touché.

## `BlackBoxRing`, `BlackBoxFormat`

`BlackBoxRing` est une file d’octets d’enregistrements entre tâches/cœurs sous `Rtos::CriticalSection` ; en cas de débordement, elle jette les plus anciens. `BlackBoxFormat` — l’en-tête de secteur, les types et structures des enregistrements, les chaînes des schémas (la taille est vérifiée par `static_assert`), CRC-8 et CRC-32.

---

## `Mavlink` (codec)

**Fichier :** `telemetry/MavlinkCodec.h` · **Genre :** namespace · **Dépend de :** rien (portable)

MAVLink 2 sans la bibliothèque générée : empaquetage des champs dans l’ordre de MAVLink
(vérifié avec pymavlink), CRC-16/MCRF4XX + `CRC_EXTRA`, suppression des zéros de fin.

| Entité | Description |
|---|---|
| `Msg::*` | identifiants : HEARTBEAT, SYS_STATUS, SET_MODE, PARAM_*, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW, MISSION_REQUEST_LIST/COUNT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS, REQUEST_DATA_STREAM, VFR_HUD, COMMAND_LONG/ACK, HOME_POSITION, STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | le `CRC_EXTRA` d’un message, −1 — inconnu |
| `crcAccumulate`, `crcCalculate` | X.25 (comme le `crc_accumulate()` de mavlink) |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` — les champs dans l’ordre |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` — une trame v2 avec un `seq` séquentiel |
| `Message` | un message reçu : `msgid`, `sysid`, `compid`, la charge utile (complétée par des zéros), lecture des champs par décalage |
| `Parser` | `bool feed(byte)` → `message()` ; v1 et v2, la signature de la v2 est sautée ; `goodCount()`, `badCrcCount()` ; les messages au `CRC_EXTRA` inconnu sont ignorés en silence |

## `MavlinkModes`

**Fichier :** `telemetry/MavlinkTelemetry.h` · **Genre :** namespace

| Fonction | Description |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | le numéro de mode d’ArduPlane : MANUAL 0, STABILIZE→FBWA 5, ALT_HOLD→FBWB 6, ACRO 4, CRUISE 7, LOITER 12, RTH→RTL 11, AUTO_TAKEOFF/LAUNCH→TAKEOFF 13, AUTO_LAND→AUTO 10, SOARING→THERMAL 24, RESCUE→STABILIZE 2 ; failsafe → RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | l’inverse, pour les commandes venant du sol ; AUTO, CIRCLE, GUIDED — `false` |
| `isAutonomous(mode)` | le drapeau `AUTO_ENABLED` dans HEARTBEAT |

## `MavlinkTelemetry`

**Fichier :** `telemetry/MavlinkTelemetry.h` · **Dépend de :** `IUartPort`, `FlightController`, `Autopilot*`, `LoopStats*`

Télémétrie par modem radio pour QGroundControl / Mission Planner (le véhicule est
`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_ARDUPILOTMEGA`). Utilisée sur le STM32
(UART4), qui n’a pas de Wi-Fi.

| Méthode | Description |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | ouvrir le port, le message « OpenPlane online » |
| `void update()` | depuis la boucle de vol : analyse ce qui arrive (≤ 128 octets par cycle), messages d’événements, pas plus de 2 trames par cycle |
| `void statusText(severity, text)` | vers le flux de la GCS (une file de 4 lignes, jusqu’à 50 caractères) |
| `isGcsConnected()` | un HEARTBEAT de la GCS dans les 3 dernières s |
| `getSentFrames()`, `getDeferredFrames()`, `getParser()` | diagnostic |
| `static const char* paramName(uint8_t)` | `RLL_KP`, `RLL_KI`, `RLL_KD`, `PTCH_KP`, `PTCH_KI`, `PTCH_KD` |

Les flux (Hz) : ATTITUDE 10 ; GLOBAL_POSITION_INT, VFR_HUD 5 ; GPS_RAW_INT,
RC_CHANNELS, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT 2 ; HEARTBEAT, SYS_STATUS 1 ;
HOME_POSITION 0,2. Une trame n’est envoyée que si `availableForWrite()` a la place pour elle
— sinon elle attend le cycle suivant (la boucle n’est jamais bloquée).

En entrée : le HEARTBEAT de la GCS ; PARAM_REQUEST_LIST / READ / SET (le PID — directement dans
le pilote automatique, valeurs 0..100, non enregistrées) ; SET_MODE et COMMAND_LONG
`DO_SET_MODE` (176) — le mode jusqu’au prochain basculement de l’interrupteur ; `COMPONENT_ARM_DISARM`
(400) — **DENIED** ; `REQUEST_MESSAGE` (512) — un envoi hors tour d’un flux ;
MISSION_REQUEST_LIST — MISSION_COUNT 0 avec le même `mission_type`.
La vérification du flux avec un décodeur externe — `tools/check_mavlink.py` (pymavlink).
