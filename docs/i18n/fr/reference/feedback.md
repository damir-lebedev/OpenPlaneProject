# AUTOPILOT / feedback — la boucle de rétroaction (ébauche)

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/feedback.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels.

[← Référence](README.md)

> ⚠️ **Ébauche, non branchée au micrologiciel.** Ni `FlightController`, ni
> `Autopilot`, ni `main.cpp` n’incluent ces en-têtes. Ils sont vérifiés par
> une simulation en boucle fermée (`test/test_feedback`, sur PC et sur la
> carte) et par des tests unitaires natifs. Le plan de branchement se trouve
> dans [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#plan-de-branchement).

L’idée : au lieu d’un PID sur l’angle avec des gains réglés pour une seule
vitesse, un régulateur refermé sur **la réponse de l’avion**, avec un modèle
d’axe appris en vol, une protection contre le décrochage et des phases de
décollage et d’atterrissage pilotées par les capteurs. La seule entrée est
`FlightSnapshot`, la seule sortie est `FeedbackOutput`.

Tous les modules sont uniquement en en-têtes ; `FeedbackModules.h` les inclut
en une seule ligne.

---

## namespace `FeedbackConfig`

**Fichier :** `autopilot/feedback/FeedbackConfig.h`

Toutes les constantes de la boucle (elles migreront dans `Config.h` lors du
branchement). Les valeurs marquées « прикидка » (« estimation grossière ») concernent un modèle d’environ 1 kg et de 1,2 m
d’envergure. Les tableaux `[AXIS_COUNT]` sont indexés par l’axe.

| Groupe | Constantes |
|---|---|
| Général | `GRAVITY = 9.80665` ; les axes `AXIS_ROLL = 0`, `AXIS_PITCH = 1`, `AXIS_YAW = 2`, `AXIS_COUNT = 3` |
| Vitesse | `STALL_SPEED_MS = 8`, `REFERENCE_SPEED_MS = 14`, `ACCEL_FILTER_TAU_S = 0.3` |
| En l’air/au sol | `AIRBORNE_HEIGHT_M = 3`, `AIRBORNE_CONFIRM_MS = 500`, `GROUND_STILL_MS = 2000`, `GROUND_ACCEL_TOLERANCE_G = 0.1` |
| Régulateur | `ANGLE_GAIN = {4, 4, 2}` 1/s, `MAX_RATE_DPS = {120, 60, 30}`, `RATE_TAU_S = {0.15, 0.20, 0.30}`, `RATE_INTEGRAL_GAIN = {2, 2, 1}`, `MAX_DEFLECTION_US = {400, 400, 400}`, `DAMPING_COMPENSATION = 0.5` |
| Efficacité des gouvernes | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs, `EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`, `EFFECTIVENESS_MAX = {30, 15, 6}`, `RESPONSE_DELAY_MS = 40`, `RLS_FORGETTING = 0.995`, `ESTIMATOR_PERIOD_MS = 20`, `ESTIMATOR_PREFILTER_HZ = 2`, `MIN_EXCITATION_US = 30` |
| Décrochage | `DECEL_WARN_MS2 = 2`, `DECEL_CONFIRM_MS = 300`, `LOW_ENERGY_PITCH_DEG = 5`, `NOSE_DROP_RATE_DPS = 60`, `WING_DROP_RATE_DPS = 120`, `STALL_NOSE_UP_COMMAND_US = 50`, `LOW_EFFECTIVENESS_RATIO = 0.35`, `LOW_SPEED_MARGIN = 1.25`, `LOW_SPEED_EXIT_MARGIN = 1.5`, `LOW_ENERGY_THROTTLE_PERCENT = 80`, `LOW_ENERGY_MAX_PITCH_DEG = 5`, `STALL_THROTTLE_PERCENT = 100`, `STALL_MAX_PITCH_DEG = −5`, `STALL_MAX_BANK_DEG = 10`, `STALL_AILERON_LIMIT_US = 150`, `RECOVERY_HOLD_MS = 1000` |
| Décollage | `TAKEOFF_HAND_LAUNCH = false`, `TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`, `TAKEOFF_THROTTLE_PERCENT = 100`, `LAUNCH_ACCEL_G = 1`, `LAUNCH_DETECT_MS = 50`, `ROTATE_SPEED_MS = 10`, `ROTATE_FALLBACK_MS = 1500`, `CLIMB_PITCH_DEG = 12`, `TAKEOFF_TARGET_ALTITUDE_M = 30`, `TAKEOFF_CLIMB_FALLBACK_MS = 10000`, `LAUNCH_TIMEOUT_MS = 8000`, `HEADING_HOLD_GAIN = 2` |
| Atterrissage | `APPROACH_SINK_RATE_MS = 1`, `APPROACH_THROTTLE_PERCENT = 25`, `APPROACH_BASE_PITCH_DEG = −3`, `APPROACH_MIN_PITCH_DEG = −10`, `APPROACH_MAX_BANK_DEG = 20`, `GO_AROUND_THROTTLE_PERCENT = 80`, `SINK_TO_PITCH_GAIN = 4`, `FLARE_HEIGHT_M = 2`, `FLARE_SINK_RATE_MS = 0.3`, `FLARE_MAX_PITCH_DEG = 8`, `TOUCHDOWN_ACCEL_G = 0.5`, `TOUCHDOWN_HEIGHT_M = 0.3`, `TOUCHDOWN_STILL_MS = 500`, `TOUCHDOWN_STILL_RATE_DPS = 5`, `ROLLOUT_MS = 5000` |

---

## namespace `FeedbackMath`

**Fichier :** `autopilot/feedback/FeedbackMath.h` · **Dépend de :** `<math.h>`

| Fonction | Description |
|---|---|
| `float wrap180(float deg)` | Un angle dans `(−180, 180]` : la différence entre les caps 350° et 10° est −20° |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | Borne à `[−limit, limit]` |

---

## `FlightSnapshot`

**Fichier :** `autopilot/feedback/FlightSnapshot.h` · **Genre :** struct

Tout ce que la boucle sait de l’avion pour un cycle. Les signes sont
aéronautiques.

| Groupe | Champs |
|---|---|
| Temps/état | `timeUs`, `armed`, `linkLost` |
| Attitude | `imuValid`, `rollDeg`, `pitchDeg`, `yawDeg`, `rollRateDps`, `pitchRateDps`, `yawRateDps`, `accelXg/Yg/Zg` |
| Altitude | `baroValid`, `altitudeM` (depuis le point de mise sous tension), `climbRateMs`, `heightAglValid`, `heightAglM` (un futur télémètre) |
| Vitesse | `airspeedValid`, `airspeedMs` (un futur tube de Pitot), `gpsValid`, `groundSpeedMs` |
| Objectifs du mode | `stabilizationActive` (false = MANUAL : apprentissage seul), `targetRollDeg`, `targetPitchDeg` |
| Ordres, µs | `stick*Us` — l’apport du pilote ; `command*Us` — le résultat réellement envoyé aux gouvernes |
| Gaz, % | `pilotThrottlePercent`, `throttlePercent` (réellement vers l’ESC) |
| Volets | `flapsUs`, `flapsMoving` |

---

## `FeedbackOutput`

**Fichier :** `autopilot/feedback/FeedbackOutput.h` · **Genre :** struct

| Champ | Description |
|---|---|
| `float deflectionUs[3]` | Braquages des gouvernes par axe, µs (les signes de `ControlCommand`) |
| `bool axisEnabled[3]` | `false` — l’axe n’est pas commandé, la gouverne reste au pilote |
| `float throttleOverridePercent` | Les gaz absolus d’une phase de vol ; `< 0` — non fixés |
| `float throttleFloorPercent` | La borne inférieure des gaz (protection contre le décrochage) ; `< 0` — aucune |
| `targetRollDeg`, `targetPitchDeg` | Les objectifs finaux après les limitations (débogage) |
| `const char* reason` | Une courte description pour le journal/l’OLED |

---

## `PhaseTargets`

**Fichier :** `autopilot/feedback/PhaseTargets.h` · **Genre :** struct

La sortie commune de `TakeoffSequencer` et de `LandingSequencer` — le « quoi »,
pas le « comment ».

| Champ | Par défaut | Description |
|---|---|---|
| `active` | `false` | La phase pilote l’avion en ce moment |
| `targetRollDeg`, `targetPitchDeg` | 0 | Les objectifs |
| `controlRoll`, `controlPitch` | `true` | `false` — ne pas toucher l’axe (sur roues, le tangage est imposé par le train d’atterrissage) |
| `holdHeading`, `headingDeg` | `false`, 0 | Tenir le cap avec la direction et la roue |
| `throttlePercent` | −1 | −1 — les gaz du pilote |
| `reason` | `""` | Une description |

---

## `SpeedEstimator`

**Fichier :** `autopilot/feedback/SpeedEstimator.h`

La vitesse (air > vitesse sol du GPS > inconnue) et l’accélération
longitudinale d’après l’IMU : `dV/dt = g · (ax − sin θ)` à travers un filtre
passe-bas `ACCEL_FILTER_TAU_S` — on voit que « la vitesse chute » même sans
capteur de vitesse.

| Méthode | Description |
|---|---|
| `void update(const FlightSnapshot&)` | Un pas ; `dt ≤ 0` ou `> 0.5 s` depuis l’appel précédent (pour le premier — depuis `timeUs = 0`) — ignoré |
| `bool hasSpeed() const`, `float getSpeed() const`, `Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`, `float getAcceleration() const` | m/s², « + » — accélération ; sans IMU — `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`, borné à `0.05..4` ; sans vitesse — 1 |

---

## `AirborneDetector`

**Fichier :** `autopilot/feedback/AirborneDetector.h`

Si l’avion est en l’air : apprendre, accumuler l’intégrale et chercher le
décrochage n’a de sens qu’en vol.

| Méthode | Description |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | Non armé — remise à « au sol ». Un candidat à un changement d’état doit se maintenir `AIRBORNE_CONFIRM_MS` (décollage) ou `GROUND_STILL_MS` (atterrissage) |
| `void force(bool)` | Le fixer explicitement (le décollage et l’atterrissage le savent) |
| `void reset()` | Au sol |
| `bool isAirborne() const` | |

« Ressemble à un vol » : la hauteur au télémètre ou au baromètre > `AIRBORNE_HEIGHT_M`,
ou la vitesse > `ROTATE_SPEED_MS`. « Ressemble au sol » : bas, les vitesses
angulaires de tous les axes < `TOUCHDOWN_STILL_RATE_DPS`, |a| ≈ 1g (± `GROUND_ACCEL_TOLERANCE_G`).

---

## `ControlEffectivenessEstimator`

**Fichier :** `autopilot/feedback/ControlEffectivenessEstimator.h`

Un axe. Le modèle : **accélération angulaire = b·gouverne(t − retard) + a·ω + c**.
`b` est l’efficacité de la gouverne (°/s² par µs, le signe est le sens de la
réponse), `a` est l’amortissement (1/s, en général < 0), `c` est un moment
constant (auto-trim). `b` est appris à une vitesse de référence :
`b = b_ref · (V/V_ref)²`, `a = a_ref · V/V_ref`. L’estimation est une méthode
des moindres carrés récursifs avec oubli (`λ = 0.995`, mémoire d’environ 4 s).

| Méthode | Description |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | Appelle `reset()` |
| `void reset()` | θ = (prior, 0, 0) ; covariance : b ± prior, a ± 5, c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | À appeler à chaque cycle : accumule des moyennes sur un intervalle de `ESTIMATOR_PERIOD_MS` ; à la fin de l’intervalle — l’accélération d’après la différence du gyroscope, le retard de l’ordre, un filtre passe-bas commun aux deux côtés et un pas de RLS (si l’apprentissage est permis et qu’il y a de l’excitation) |
| `getEffectiveness()` | `b` à la vitesse actuelle |
| `getReferenceEffectiveness()` | `b` à la vitesse de référence |
| `getDamping()` | `a` à la vitesse actuelle |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b` (0 si `\|b\|` est petit) |
| `getEffectivenessSigma()` | σ de l’estimation de `b` à la vitesse actuelle |
| `bool isConfident() const` | ≥ 50 pas de RLS, `\|b\|` ≥ le minimum et σ < 0.3·`\|b\|` |
| `getAngularAccel()` | L’accélération du dernier intervalle (débogage) |

Particularités :

- Un intervalle plus long que `MAX_GAP_MS = 200` (la boucle s’est arrêtée) — les
  données repartent de zéro (l’historique du retard et le filtre sont
  réinitialisés).
- On n’apprend que sous **excitation** : l’amplitude des ordres moyens sur 16
  intervalles (~0,3 s) ≥ `MIN_EXCITATION_US` ; sinon l’estimation se fige.
- Hygiène des float après un pas : la symétrie de `P`, un plafond sur les
  variances (×10 des valeurs initiales), une borne sur `b` (`±EFFECTIVENESS_MAX`)
  et sur `a` (`−40..5`).

---

## `AxisModel`

**Fichier :** `autopilot/feedback/AdaptiveRateController.h` · **Genre :** struct

Ce que l’on sait de la réponse d’un axe, pour le régulateur : `effectiveness`
(b, par défaut 1), `damping` (a, 0 — ne pas compenser), `bias` (c, 0).

---

## `AdaptiveRateController`

**Fichier :** `autopilot/feedback/AdaptiveRateController.h`

Un régulateur à un axe, trois étages :

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| Méthode | Description |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | L’intégrale, la saturation et la sortie — 0 |
| `float angleToRate(targetDeg, angleDeg) const` | Étage 1 (le plus court chemin pour le cap) |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | Étages 2–3, renvoie le braquage, µs |
| `getDesiredRate()`, `getIntegral()`, `getOutput()`, `isSaturated()` | État |

Invariants : `|b|` n’est pas inférieur à `EFFECTIVENESS_MIN` (en conservant le
signe de b) ; l’intégrale est conservée en °/s (elle reste juste quand `b`
change) et **ne s’accumule pas vers la butée** (anti-windup selon le sens de la
saturation du pas précédent) ; avec `dt ≤ 0`, l’intégrale ne change pas.

---

## `StallGuard`

**Fichier :** `autopilot/feedback/StallGuard.h`

Protection contre la perte de vitesse et le décrochage. Les niveaux sont `Level::{Normal, LowEnergy, Stall}`.

| Méthode | Description |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | Au sol ou sans IMU — remise à Normal |
| `Level getLevel() const`, `const char* getLevelName() const` | `"OK"`, `"LOW_ENERGY"`, `"STALL"` |
| `const char* getReason() const` | Le dernier indice qui s’est déclenché |
| `float maxPitchDeg() const` | Stall : −5°, LowEnergy : 5°, sinon 90° |
| `float maxBankDeg() const` | Stall : 10°, sinon 180° |
| `float maxAileronUs() const` | Stall : 150 µs, sinon `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall 100 %, LowEnergy 80 %, sinon/sans liaison −1 |
| `void reset()` | Normal |

`StallGuard::ControlState` — `pitchEffectivenessKnown`, `pitchEffectiveness`
(le module de l’estimation de b en tangage).

Indices de **LowEnergy** : une décélération confirmée (`DECEL_CONFIRM_MS`)
supérieure à `DECEL_WARN_MS2` avec un tangage > 5° ; vitesse < `1.25·Vs` ; une
efficacité fiable de la profondeur < 35 % de l’efficacité a priori. Indices de
**Stall** : vitesse < Vs ; le nez tombe plus vite que 60 °/s avec la profondeur
« en haut » > 50 µs ; à faible énergie, l’aile tombe plus vite que 120 °/s
contre les ailerons. Les mesures sont levées au bout de `RECOVERY_HOLD_MS` et
seulement quand l’énergie est rétablie (vitesse ≥ `1.5·Vs`, sans capteur de
vitesse — accélération ≥ 0).

---

## `TakeoffSequencer`

**Fichier :** `autopilot/feedback/TakeoffSequencer.h`

Décollage par phases. `State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`.
Le schéma se trouve dans [ARCHITECTURE.md §7](../ARCHITECTURE.md#décollage-et-atterrissage-boucle-de-rétroaction-non-branchée).

| Méthode | Description |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | Une phase active → `Aborted` ; les objectifs sont réinitialisés |
| `void update(snapshot, speed, nowMs)` | Au plus une transition par cycle, puis les objectifs de la nouvelle phase |
| `void reset()` | → `Idle` |
| `getTargets()`, `getState()`, `getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

Les objectifs des phases : attente — gaz 0, gouvernes au pilote ; course —
gaz 100 %, ailes à plat, ne pas toucher au tangage, tenir le cap fixé au moment
du départ ; montée — gaz 100 %, ailes à plat, tangage `CLIMB_PITCH_DEG`. Lancer
à la main : l’accélération longitudinale `ax − sin θ ≥ LAUNCH_ACCEL_G` pendant
plus de `LAUNCH_DETECT_MS`.

---

## `LandingSequencer`

**Fichier :** `autopilot/feedback/LandingSequencer.h`

Atterrissage par phases. `State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`.

| Méthode | Description |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`, `void reset()`, `update(snapshot, nowMs)` | Comme pour le décollage |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout — la protection contre le décrochage est désactivée ici |
| `bool isOnGround() const` | Rollout / Complete |

Le tangage à la descente et à l’arrondi vient de l’erreur de vitesse
verticale : `θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`, borné à
`[min, FLARE_MAX_PITCH_DEG]` ; sans baromètre — l’angle de base. La hauteur
vient du télémètre, sinon du baromètre. Le toucher : un pic de |a − 1g| ≥ 0.5g
ou « bas et ne tourne pas » pendant `TOUCHDOWN_STILL_MS`. Pendant la course à
l’atterrissage, le cap est fixé au moment du toucher.

---

## `FeedbackSupervisor`

**Fichier :** `autopilot/feedback/FeedbackSupervisor.h`

La boucle entière. Possède le `SpeedEstimator`, l’`AirborneDetector`, trois
`ControlEffectivenessEstimator`, trois `AdaptiveRateController`, le `StallGuard`,
le `TakeoffSequencer` et le `LandingSequencer`.

| Méthode | Description |
|---|---|
| `bool requestTakeoff()` | Seulement armé, avec liaison, au sol ; annule un atterrissage |
| `bool requestLanding()` | Seulement armé, avec liaison, en l’air ; annule un décollage |
| `void cancelPhase()` | Annuler la phase |
| `const FeedbackOutput& update(const FlightSnapshot&)` | Un cycle (l’ordre est dans [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-la-boucle-de-rétroaction-non-branchée)) |
| `getOutput()`, `isAirborne()`, `getSpeedEstimator()`, `getEstimator(axis)`, `getController(axis)`, `getStallGuard()`, `getTakeoff()`, `getLanding()` | État pour le journal et les tests |
| `void printStatus(Print& out) const` | Une ligne d’état + une ligne par axe (`b ± σ`, `*` — fiable, `a`, `c`, `I`, la sortie) |

Règles essentielles :

- **Non armé** — tous les axes sont désactivés, `reason = "non armé"` ;
  l’ARM/DISARM (un nouveau vol) efface tout ce qui a été appris.
- **La perte de liaison** annule les phases ; les gaz ne sont pas touchés (c’est
  le failsafe du micrologiciel qui agit).
- **Le décollage du sol** réinitialise les estimations et les régulateurs (ce
  qui a été « vu » sur les roues ne convient pas).
- Une estimation fiable et négative de `b` ne va **jamais** au régulateur — l’axe
  travaille avec le modèle a priori, et `reason` porte l’avertissement « …
  répond à la gouverne à l’envers ? vérifier au sol ».
- L’intégrale est gelée au sol, sauf le cap pendant la course au décollage et à
  l’atterrissage.
- Virage coordonné (avec une vitesse connue en l’air) : à la vitesse de
  tangage voulue s’ajoute `+ g/V · sin φ · tg φ`, et à celle de lacet
  `g/V · sin φ` (l’inclinaison est bornée à ±60°).
- La priorité de `reason` : décrochage > peu d’énergie > phase > avertissement
  de signe > « stabilisation »/« manuel (apprentissage) ».
