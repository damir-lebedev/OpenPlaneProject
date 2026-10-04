# CONTROL et COORDINATION — le mixeur, les gaz, l’ARM, les sorties, l’orchestrateur

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/control.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels.

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

| Champ | Valeur par défaut |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
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
| `FlightOutputState mix(const ControlCommand& c) const` | Ordre → PWM. Le roulis, le tangage et le lacet sont bornés par la course (`*_MAX_US`) ; ailerons : gauche = `flaps + roll`, droit = `flaps − roll` (vers le bas = « + ») ; PWM = `1500 ± braquage` avec le signe de `Config::*_REVERSED`, borné à 1000..2000. `throttle` n’est pas rempli |
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

## `ArmingManager`

**Fichier :** `control/ArmingManager.h` · **Dépend de :** `Autopilot` (nullable), `RcChannelState`, `Config`, `Channels`

L’ARM par un interrupteur à part, SwA (CH5). L’automate est décrit dans
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Méthode | Description |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Sans pilote automatique, seuls les gaz sont vérifiés |
| `void update(const RcChannelState& rc, bool receiverFailsafe)` | En failsafe — rien (l’interrupteur dans une trame de failsafe ne reflète pas le pilote). Interrupteur OFF → DISARM, `switchSeenOff = true`. Une transition OFF→ON → vérifications → ARM ou refus |
| `bool isArmed() const` | Armé |
| `const char* getLastRefusalReason() const` | La raison du dernier refus ou `nullptr` ; effacée à l’extinction de l’interrupteur |

`checkFailureReason(rc)` — les vérifications de l’ARM :

| Condition | Raison du refus |
|---|---|
| Gaz ≥ `THROTTLE_LOW_US` | « gaz pas au minimum » |
| Tout mode sauf MANUAL, l’IMU existe mais ne répond pas | « l’IMU ne répond pas… » |
| Tout mode sauf MANUAL, l’IMU a un problème à la vérification avant vol | le texte de `ImuSensor::getPreflightProblem()` |
| Un mode avec altitude (`needsAltitude` : ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), le baromètre existe mais ne répond pas | « le baromètre ne répond pas… » |

Un capteur absent de la compilation (`nullptr`) ne bloque pas l’ARM ; en MANUAL,
l’appareil s’arme même sans aucun capteur. Le fix GPS ne fait volontairement pas
partie des vérifications : sans GPS, les modes de navigation se comportent
de façon sûre (un cercle sur place), et le point de départ s’enregistrera quand
le GPS captera les satellites.

Invariants : allumer la carte avec l’interrupteur sur ON n’arme pas ; une seule
tentative par transition OFF→ON ; la perte de liaison n’annule pas l’ARM.

---

## `FlightOutputs`

**Fichier :** `control/FlightOutputs.h` · **Dépend de :** `IBoard`, `FlightOutputState`, `Config`

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
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` de chaque sortie, affiche l’état ; `true` si toutes les sorties **obligatoires** ont obtenu un canal |
| `bool isAttached(uint8_t ch) const` | La sortie est connectée (un indice hors limites → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | La valeur de la sortie d’après l’état, selon le tableau |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | L’impulsion mesurée face à l’impulsion attendue sur chaque broche câblée ; « OK » si l’écart est ≤ 15 µs |
| `void write(const FlightOutputState&)` | Écrire toutes les sorties et mémoriser l’état |
| `void setFailsafe()` | Gouvernes au neutre (`FAILSAFE_*`), gaz `FAILSAFE_THROTTLE` ; AUX comme avant (la charge n’est pas larguée en cas de perte de liaison) |
| `void setBuzzer(bool on)` | le buzzer de la carte (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | Le dernier état écrit |

Pour ajouter une sortie : une ligne dans le tableau + un champ dans
`FlightOutputState` + un indice dans `ServoChannel` (+ une broche et un canal
LEDC dans `Esp32Board`).

---

## `FlightController`

**Fichier :** `control/FlightController.h` · **Couche :** COORDINATION ·
**Dépend de :** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`

Le seul coordinateur de la boucle de contrôle : il n’analyse pas lui-même
l’UART, ne touche pas au PWM et ne calcule pas le mixeur — il ne fait qu’appeler
les autres dans le bon ordre. Un schéma détaillé se trouve dans
[ARCHITECTURE.md §6](../ARCHITECTURE.md#6-le-cycle-de-contrôle-flightcontrollerupdate).

| Méthode | Description |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | Sans pilote automatique — pilotage purement manuel ; sans interrupteurs — seulement les manches |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | Un cycle (voir ci-dessous) |
| `bool isReceiverFailsafe() const` | La liaison est perdue |
| `const IBusReceiver& getReceiver() const` | Pour le journal (compteurs de trames, cause de la perte) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | Le dernier état écrit sur les sorties |
| `const RcChannelState& getRcState() const` | Les canaux |
| `const FlightOutputs& getOutputs() const` | Le tableau des sorties et `attached` |
| `int16_t getFlapsUs() const` | La position des volets |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | Les interrupteurs et potentiomètres de ce cycle |
| `bool isLostModelBeeping() const` | Le buzzer « je suis ici » fonctionne |

L’ordre de `update()` :

1. `receiver.update()` ; `failsafe = receiver.isSignalLost()` ;
2. avec une liaison vivante — `switches->update(rc)` (le mode, les fonctions, les potentiomètres) ;
3. `pilotThrottle = throttle.update(rc, failsafe)` ;
4. les manches `mixer.fromSticks(rc)` (avec une liaison vivante) × `Knob::RATES` ;
   les volets `mixer.updateFlaps(target)` : `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → progressivement, en cas de perte de liaison — 0 ;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **toujours** ;
6. le buzzer : `Beeper::update(BEEPER, armed, failsafe, now)` ;
7. liaison perdue → `applyLinkLoss()` et sortie du cycle ;
8. `arming.update(rc, false)` ;
9. `command = autopilot->getCommand()` (ou les manches sans pilote automatique), les volets — les leurs ;
10. `output = mixer.mix(command)` ; `output.throttle = autopilot->applyThrottle(pilotThrottle)` ;
11. non armé ou `MOTOR_KILL` → `throttle = PWM_MIN` (en dernier) ;
12. AUX1 — la charge (`PAYLOAD_DROP`), AUX2 — la caméra (`Knob::CAMERA_TILT`, `CAMERA_STAB` soustrait le tangage) ;
13. `outputs.write(output)`.

`applyLinkLoss()` : si le pilote automatique est en failsafe (armé : RTH ou
plané) — les gouvernes et les gaz suivent l’ordre du pilote automatique (les
volets rentrent progressivement, `MOTOR_KILL` coupe toujours le moteur, AUX
comme avant) ; sinon `outputs.setFailsafe()`.

---

## `Beeper`

**Fichier :** `control/Beeper.h` · **Dépend de :** `Config`

| Méthode | Description |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | l’état du buzzer : 2 Hz si `Feature::BEEPER` ou « modèle perdu » (non armé, pas de liaison depuis plus de `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | le mode « cherchez-moi dans l’herbe » |
