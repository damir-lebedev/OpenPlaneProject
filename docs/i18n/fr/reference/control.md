# CONTROL et COORDINATION — le mixeur, les gaz, l’ARM, les sorties, l’orchestrateur

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/control.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Référence](README.md)

La couche CONTROL est de la logique au-dessus des données, sans UART, PWM ni
Wi-Fi. `FlightController` (COORDINATION) est la seule classe qui réunit toutes
les couches inférieures en un seul cycle.

---

## `ControlCommand`

**Fichier :** `control/ControlCommand.h` · **Genre :** struct

Un ordre aux gouvernes en **signes physiques**, µs de braquage (±500 = course
complète). La langue commune des manches, du pilote automatique et du mixeur.

| Champ | « + » signifie |
|---|---|
| `int16_t roll` | roulis à droite (aileron droit en haut, gauche en bas) |
| `int16_t pitch` | nez en haut (profondeur en haut) |
| `int16_t yaw` | nez à droite (direction et roue à droite) |
| `int16_t flaps` | volets en bas (les deux ailerons en bas) ; « − » — aérofrein (les deux en haut) |

Tous les champs valent 0 par défaut.

---

## `FlightOutputState`

**Fichier :** `control/FlightOutputState.h` · **Genre :** struct

Les impulsions voulues des sorties, µs de PWM. Par défaut — gouvernes au neutre
et gaz à `PWM_MIN`.

`throttle` est l’amplitude des gaz (1000..2000 = 0..100 %) dans un sens comme dans l’autre ; `FlightOutputs` la convertit, avec `motorReverse`, en impulsion de l’ESC (via `EscSignal`).

| Champ | Valeur par défaut |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `bool motorReverse` | `false` — le moteur tourne en avant |
| `bool escPulsesOff` | `false` ; `true` — aucune impulsion vers l’ESC (signal moteur, seulement sans ARM) |
| `aux1` | `PAYLOAD_CLOSED_US` — le largueur de charge est fermé |
| `aux2` | `PWM_CENTER` — la caméra |

---

## `FlapsController`

**Fichier :** `control/FlapsController.h` · **Dépend de :** `Config`

Sortie et rentrée progressives des volets : la position tend vers la cible
(n’importe quelle valeur — volets de l’interrupteur, du potentiomètre, un
aérofrein vers le haut) sans aller plus vite que la course complète
`FLAPS_DEPLOYED_US` en `FLAPS_TRANSITION_MS`. Le temps est passé en paramètre.

| Méthode | Description |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | Un pas vers la cible ; renvoie la position actuelle, µs (+ vers le bas, − vers le haut) |
| `int16_t getPosition() const` | La position actuelle |

Invariants :

- **Le premier appel** place la position directement sur la cible — les volets
  ne « sortent » pas sur la table à la mise sous tension.
- Le pas de temps est borné à `MAX_STEP_MS = 20` : après une longue pause
  (failsafe, calibration), les volets ne sautent pas sur la cible en un seul
  cycle.

---

## `ControlMixer`

**Fichier :** `control/ControlMixer.h` · **Dépend de :** `RcInput`, `RcChannelState`, `FlapsController`, `ControlCommand`, `FlightOutputState`, `Config`, `Channels`

La logique aérodynamique en deux étapes. Possède le `FlapsController`.

| Méthode | Description |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll` (2000 = à droite) ; CH2 → `pitch` **avec le signe inverse** (2000 = vers l’avant = nez en bas) ; CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | la cible des volets est choisie par `FlightController` (aérofrein → interrupteur des volets → potentiomètre), ici — la course progressive |
| `FlightOutputState mix(const ControlCommand& c) const` | Ordre → PWM. Le roulis, le tangage et le lacet sont bornés par la course (`*_MAX_US`) ; ailerons : gauche = `flaps + roll`, droit = `flaps − roll` (vers le bas = « + ») ; PWM = `1500 ± braquage` avec le signe de `Config::*_REVERSED`, le signe des volets — `FLAPS_REVERSED`, borné à 1000..2000. `throttle` n’est pas rempli |
| `int16_t getFlaps() const` | La position actuelle des volets, µs |

Flaperons : à la sortie, les deux ailerons s’abaissent de `FLAPS_DEPLOYED_US`
(le nouveau « neutre »), et le roulis agit par-dessus. À plein roulis,
l’aileron qui descend arrive en butée avant celui qui monte — cela fonctionne
comme un différentiel d’ailerons.

---

## `ThrottleManager`

**Fichier :** `control/ThrottleManager.h` · **Dépend de :** `RcInput`, `RcChannelState`, `Config`, `Channels`

| Méthode | Description |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | Les gaz du CH3, bornés à 1000..2000 ; en cas de perte de liaison — `FAILSAFE_THROTTLE` |

Ne sait rien de l’ARM ni du pilote automatique — leurs corrections sont
appliquées par `FlightController`.

---

## `EscSignal`

**Fichier :** `control/EscSignal.h` · **Genre :** namespace · **Dépend de :** `Config`

Gaz (1000..2000 = 0..100 %) et sens → impulsion sur la broche de l’ESC.

| Fonction | Description |
|---|---|
| `uint16_t pulseUs(uint16_t throttleUs, bool reverse)` | `ESC_BIDIRECTIONAL` (BLHeli 80A en 3D) : 0 % → `ESC_NEUTRAL_US` (1500), en avant 1500..2000, en arrière 1500..1000. ESC unidirectionnel : 1000..2000 tel quel, `reverse` est ignoré. Les gaz hors de 1000..2000 sont écrêtés |
| `uint16_t stopUs()` | L’impulsion « moteur arrêté » (1500 pour un ESC bidirectionnel, 1000 pour un ESC ordinaire) |

Conversion seulement, sans protections : c’est `MotorReverse` qui décide quand changer de sens, et `FlightController` qui limite les gaz en arrière.

---

## `MotorReverse`

**Fichier :** `control/MotorReverse.h` · **Dépend de :** `Config`

Le sens du moteur suivant l’interrupteur `Feature::MOTOR_REVERSE` (SwD par défaut) — **uniquement hors ARM**.

| Méthode | Description |
|---|---|
| `void update(bool requested, bool armed, uint32_t nowMs)` | Hors ARM — le sens suit aussitôt l’interrupteur (Serial affiche `MotorReverse: мотор НАЗАД` / `вперёд` — « moteur EN ARRIÈRE » / « en avant »). En ARM — l’interrupteur est sans effet, « реверс НЕ переключён » (« inversion NON basculée ») est affiché une fois, et le nouveau sens attend le DISARM. Avec `ESC_BIDIRECTIONAL = false`, l’inversion ne s’enclenche jamais |
| `bool isReversed() const` | Le moteur tourne en arrière |
| `bool isPending() const` | L’interrupteur est déjà dans l’autre position ; le sens attend le DISARM |
| `const char* armBlockReason(uint32_t nowMs) const` | Raison de refuser l’ARM : moins de `REVERSE_SETTLE_MS` depuis le dernier changement de sens (le moteur tourne peut-être encore) ; sinon `nullptr` |

---

## `ArmingManager`

**Fichier :** `control/ArmingManager.h` · **Dépend de :** `Autopilot` (nullable), `RcChannelState`, `Config`, `Channels`

L’ARM par un interrupteur à part, SwA (CH5). L’automate est décrit dans
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Méthode | Description |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Sans pilote automatique, seuls les gaz sont vérifiés |
| `void update(const RcChannelState& rc, bool receiverFailsafe, const char* blockReason = nullptr)` | En failsafe — rien (l’interrupteur dans une trame de failsafe ne reflète pas le pilote). Interrupteur OFF → DISARM, `switchSeenOff = true`. Une transition OFF→ON → vérifications → ARM ou refus `blockReason` est une raison de refus externe (venant de `FlightController` : `MotorReverse::armBlockReason()`) |
| `bool isArmed() const` | Armé |
| `const char* getLastRefusalReason() const` | La raison du dernier refus ou `nullptr` ; effacée à l’extinction de l’interrupteur |

`checkFailureReason(rc)` — les vérifications de l’ARM :

| Condition | Raison du refus |
|---|---|
| Gaz ≥ `THROTTLE_LOW_US` | « gaz pas au minimum » |
| Tout mode sauf MANUAL, l’IMU existe mais ne répond pas | « l’IMU ne répond pas… » |
| Tout mode sauf MANUAL, l’IMU a un problème à la vérification avant vol | le texte de `ImuSensor::getPreflightProblem()` |
| Un mode avec altitude (`needsAltitude` : ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), le baromètre existe mais ne répond pas | « le baromètre ne répond pas… » |
| Tout le reste est passé, mais `blockReason` est fourni | `blockReason` (par exemple « le sens du moteur vient de changer… ») |

Un capteur absent de la compilation (`nullptr`) ne bloque pas l’ARM ; en MANUAL,
l’appareil s’arme même sans aucun capteur. Le fix GPS ne fait volontairement pas
partie des vérifications : sans GPS, les modes de navigation se comportent
de façon sûre (un cercle sur place), et le point de départ s’enregistrera quand
le GPS captera les satellites.

Invariants : allumer la carte avec l’interrupteur sur ON n’arme pas ; une seule
tentative par transition OFF→ON ; la perte de liaison n’annule pas l’ARM.

---

## `FlightOutputs`

**Fichier :** `control/FlightOutputs.h` · **Dépend de :** `IBoard`, `FlightOutputState`, `EscSignal`, `Config`

La seule classe qui connaît l’ensemble et l’ordre des sorties PWM. Toutes les
sorties sont décrites dans un tableau ; `begin()`, `write()`, l’état et
l’autotest le parcourent en boucle.

### `FlightOutputs::OutputInfo`

| Champ | Description |
|---|---|
| `const char* key` | Le nom dans le JSON/journal (`aileronLeft`, …, `esc`, `rudder`, `aux1`, `aux2`) |
| `const char* label` | Le nom pour les humains |
| `int16_t pin` | Le numéro de broche ; `-1` — non câblée. `int16_t`, car sur la STM32 les numéros des broches analogiques sont `0xC0 + N` |
| `bool required` | Sans elle l’appareil ne vole pas (la direction est facultative) |
| `uint16_t FlightOutputState::* field` | Un pointeur vers le champ de l’état |

| Méthode | Description |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | Une ligne du tableau ; l’ordre = `ServoChannel` |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` de chaque sortie, affichage de l’état et du type d’ESC ; `true` si toutes les sorties **obligatoires** ont obtenu un canal |
| `bool isAttached(uint8_t ch) const` | La sortie est connectée (un indice hors limites → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | L’impulsion sur la broche de la sortie : le champ selon la table, pour l’ESC — via `EscSignal::pulseUs(throttle, motorReverse)`, avec `escPulsesOff` — 0 |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | L’impulsion mesurée face à l’impulsion attendue sur chaque broche câblée ; « OK » si l’écart est ≤ 15 µs |
| `void write(const FlightOutputState&)` | Écrire toutes les sorties et mémoriser l’état ; l’ESC avec `escPulsesOff` — `IServoOutput::stopPulses()` |
| `void setFailsafe()` | Gouvernes au neutre (`FAILSAFE_*`), gaz `FAILSAFE_THROTTLE` (1500 sur la broche d’un ESC bidirectionnel) ; AUX comme avant (la charge n’est pas larguée sur perte de liaison) |
| `void setBuzzer(bool on)` | le buzzer de la carte (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | Le dernier état écrit |

Pour ajouter une sortie : une ligne dans le tableau + un champ dans
`FlightOutputState` + un indice dans `ServoChannel` (+ une broche et un canal
LEDC dans `Esp32Board`).

---

## `FlightController`

**Fichier :** `control/FlightController.h` · **Couche :** COORDINATION ·
**Dépend de :** `RcReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`, `MotorReverse`, `GroundCalibration`, `SignalPlayer`

Le seul coordinateur de la boucle de contrôle : il n’analyse pas lui-même
l’UART, ne touche pas au PWM et ne calcule pas le mixeur — il ne fait qu’appeler
les autres dans le bon ordre. Un schéma détaillé se trouve dans
[ARCHITECTURE.md §6](../ARCHITECTURE.md#6-le-cycle-de-contrôle-flightcontrollerupdate).

| Méthode | Description |
|---|---|
| `FlightController(RcReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | Sans pilote automatique — pilotage purement manuel ; sans interrupteurs — seulement les manches |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | Un cycle (voir ci-dessous) |
| `bool isReceiverFailsafe() const` | La liaison est perdue |
| `const RcReceiver& getReceiver() const` | Pour le journal (compteurs de trames, cause de la perte) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | Le dernier état écrit sur les sorties |
| `const RcChannelState& getRcState() const` | Les canaux |
| `const FlightOutputs& getOutputs() const` | Le tableau des sorties et `attached` |
| `int16_t getFlapsUs() const` | La position des volets |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | Les interrupteurs et potentiomètres de ce cycle |
| `bool isLostModelBeeping() const` | Le buzzer « je suis ici » fonctionne |
| `const MotorReverse& getMotorReverse() const` | Le sens du moteur et l’inversion en attente |
| `void toggleGroundCalibration()` | Console `c` : calibration au sol de l’avion — la lancer (seulement sans ARM), en cours — l’annuler |
| `void playSignal(uint8_t count)` | Console `e` : un signal (sans ARM — aussi par le moteur) |
| `const GroundCalibration& getCalibration() const`, `const SignalPlayer& getSignals() const` | Pour l’OLED, le journal et la boîte noire |

L’ordre de `update()` :

1. `receiver.update()` ; `failsafe = receiver.isSignalLost()` ;
2. avec une liaison vivante — `switches->update(rc)` (le mode, les fonctions, les potentiomètres) ;
3. `pilotThrottle = throttle.update(rc, failsafe)` ;
4. les manches `mixer.fromSticks(rc)` (avec une liaison vivante) × `Knob::RATES` ;
   les volets `mixer.updateFlaps(target)` : `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → progressivement, en cas de perte de liaison — 0 ;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **toujours** ;
   en inversion, `PWM_MIN` est passé à la place de `pilotThrottle` (pour le pilote automatique, le moteur est coupé) ;
6. la calibration : un basculement d’ARM pendant celle-ci l’annule ; sans ARM et avec la liaison — le geste `CAL_SWITCH` ; un pas de `GroundCalibration::update()` ; ses signaux et le signal de refus du décollage automatique vont à `SignalPlayer` ; le buzzer : `Beeper::update(BEEPER, armed, failsafe, now)` ou la lumière du signal ;
7. liaison perdue → `applyLinkLoss()` et sortie du cycle ;
8. `arming.update(rc, false, raison)` — l’ARM est refusé tant qu’une calibration est en cours ou qu’un signal joue (et pendant le cycle où un basculement d’ARM a annulé une calibration), sinon `reverse.armBlockReason(now)` ; puis `reverse.update(MOTOR_REVERSE, armed, now)` — au DISARM, le sens change dans le même cycle ;
9. `command = autopilot->getCommand()` (ou les manches sans pilote automatique), les volets — les leurs ; le hochement du signal va à la profondeur et à la direction ;
10. `output = mixer.mix(command)` ; `output.motorReverse = reverse.isReversed()` ;
    en avant — `output.throttle = autopilot->applyThrottle(pilotThrottle)` limité par
    `THROTTLE_LIMIT_PCT` × le potentiomètre `Knob::THROTTLE_LIMIT`, en arrière — les gaz du pilote limités par
    `min(REVERSE_THROTTLE_LIMIT_PCT, limite avant)` ; la plage autorisée est étirée sur toute la course du manche ;
11. non armé ou `MOTOR_KILL` → `throttle = PWM_MIN` (en dernier) ; sans ARM, le signal moteur pose `escPulsesOff` ;
12. AUX1 — la charge (`PAYLOAD_DROP`), AUX2 — la caméra (`Knob::CAMERA_TILT`, `CAMERA_STAB` soustrait le tangage) ; `AUX1_REVERSED`/`AUX2_REVERSED` reflètent l’impulsion.
13. `outputs.write(output)`.

`applyLinkLoss()` : si le pilote automatique est en failsafe (armé : RTH ou plané) — les gouvernes et les gaz suivent l’ordre du pilote automatique (les volets rentrent progressivement, `MOTOR_KILL` et l’inversion coupent le moteur, AUX comme avant) ; sinon `outputs.setFailsafe()`, et tant qu’un signal joue — neutre avec hochement et pause des impulsions de l’ESC (la radio a été posée : la calibration est en cours).

---

## `Beeper`

**Fichier :** `control/Beeper.h` · **Dépend de :** `Config`

| Méthode | Description |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | l’état du buzzer : 2 Hz si `Feature::BEEPER` ou « modèle perdu » (non armé, pas de liaison depuis plus de `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | le mode « cherchez-moi dans l’herbe » |

---

## `GroundCalibration`

**Fichier :** `control/GroundCalibration.h` · **Dépend de :** `Config`, `RcChannelState`, `ImuSensor`, `ImuOrientation`

La calibration au sol de l’avion : étape 1 — queue levée, fuselage à l’horizontale (l’horizon, `ImuOrientation::withLevel()`), étape 2 — sur les roues (l’assiette au sol). Une étape est validée d’elle-même après `CAL_STILL_MS` d’immobilité ; l’étape 2 attend que l’avion soit déplacé, ou `CAL_PARK_WAIT_MS`. L’enregistrement se fait seulement en entier (`ImuSensor::setGroundCalibration()`), après contrôle de l’assiette au sol selon `LANDING_GEAR`. Ne bloque pas la boucle.

`Stage` : `IDLE`, `LEVEL`, `PARK`, `DONE`, `FAILED`.

| Méthode | Description |
|---|---|
| `bool watchSwitch(const RcChannelState&, uint32_t nowMs)` | le geste : `CAL_SWITCH` `CAL_SWITCH_FLIPS` fois en bas et en haut en moins de `CAL_SWITCH_WINDOW_MS`, gaz en bas ; le premier regard et des gaz pas en bas remettent le compte à zéro |
| `void forgetSwitch()` | ARM ou pas de liaison — le compte repart de zéro |
| `bool start(const ImuSensor*, uint32_t nowMs, const char* source)` | lancer (signal 1) ; pas d’IMU ou non pris en charge — erreur (signal 3) |
| `void cancel(const char* why)` | annulation sans signal |
| `void update(ImuSensor*, bool armed, bool signalBusy, uint32_t nowMs)` | un pas : l’ARM annule ; tant qu’un signal joue, rien n’est validé ; délai `CAL_STEP_TIMEOUT_MS` dépassé — erreur |
| `isActive()`, `getStage()`, `getStageSinceMs()`, `getFailure()`, `getParkPitch()`, `getParkRoll()` | état et résultat |
| `uint8_t takeSignal()` | combien de signaux jouer : 1 — lancée / étape 1, 2 — terminée, 3 — erreur (lu une seule fois) |
| `static const char* stageShort(Stage)` | `CAL1`, `CAL2`, `CALOK`, `CALER` — pour l’OLED |

---

## `SignalPlayer`

**Fichier :** `control/SignalPlayer.h` · **Dépend de :** `Config`

Les signaux au pilote — une fonction du temps écoulé depuis le début de la série. Un signal : la profondeur et la direction hochent (`SIGNAL_NOD_US` en haut, puis en bas, `SIGNAL_NOD_MS` chacun), sans ARM les impulsions de l’ESC s’arrêtent pendant `SIGNAL_ESC_PAUSE_MS` (puis l’ESC joue sa mélodie de disponibilité), la lumière du buzzer s’allume pendant le hochement ; le signal suivant vient après `SIGNAL_PERIOD_MS`.

| Méthode | Description |
|---|---|
| `void play(uint8_t count, uint32_t nowMs, bool motor)` | une série ; `motor` — si l’on peut toucher à l’ESC (et `SIGNAL_MOTOR`) |
| `void stop()` | silence |
| `bool isBusy(uint32_t nowMs) const`, `uint8_t getCount(uint32_t nowMs) const` | une série est en cours (l’ARM est refusé pendant ce temps) |
| `bool escPulsesOff(uint32_t nowMs) const` | couper maintenant les impulsions de l’ESC |
| `int16_t nodUs(uint32_t nowMs) const` | hochement des gouvernes, µs (`SIGNAL_SURFACES`) |
| `bool light(uint32_t nowMs) const` | buzzer / LED sur `PIN_BUZZER` |
