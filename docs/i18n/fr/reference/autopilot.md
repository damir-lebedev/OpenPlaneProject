# AUTOPILOT — modes, navigation, interrupteurs

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/autopilot.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Référence](README.md)

Le pilote automatique reçoit les manches du pilote, les interrupteurs et
potentiomètres (`PilotInputs`) et les capteurs, et fournit **l’ordre final aux
gouvernes** (`getCommand()`) et les gaz du mode (`applyThrottle()`). Sans un
capteur nécessaire, le mode se comporte de façon sûre (les gouvernes restent au
pilote ou au neutre) au lieu de planter. Ce que fait chaque mode pour le pilote
se trouve dans [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md). En vol, seul le mode manuel a été utilisé jusqu’ici ; le pilote
automatique a été éprouvé sur le banc, par des tests et par des simulations en
boucle fermée (`test/native/test_sim`).

---

## `AutopilotMode`, `Feature`, `Knob`

**Fichier :** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t` (sans portée — les codes numériques vont dans le
JSON de `/api/setmode`, `/api/status` et dans `MavlinkModes`) :

| Valeur | Code | Court (OLED) | En bref |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | gouvernes = manches |
| `MODE_STABILIZE` | 1 | STAB | le manche est l’angle de roulis/tangage |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | un programme de décollage piloté par les gaz du pilote |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + altitude à la profondeur |
| `MODE_ACRO` | 4 | ACRO | le manche est la vitesse angulaire |
| `MODE_CRUISE` | 5 | CRZ | cap + altitude + gaz automatiques |
| `MODE_LOITER` | 6 | LOIT | cercles au-dessus du point d’activation |
| `MODE_RTH` | 7 | RTH | retour au point de départ, cercles au-dessus |
| `MODE_LAUNCH` | 8 | LNCH | lancer à la main |
| `MODE_AUTO_LAND` | 9 | LAND | plané + arrondi |
| `MODE_SOARING` | 10 | SOAR | ascendances sans moteur |
| `MODE_RESCUE` | 11 | RESQ | ailes à plat, nez en haut, gaz |
| `MODE_COUNT` | 12 | | la borne (`setMode()` ignore ≥) |

`enum class Feature : uint8_t` — les fonctions des interrupteurs : `FLAPS`,
`AIRBRAKE`, `AUTO_TRIM`, `TURN_COORDINATION`, `MOTOR_KILL`, `BEEPER`,
`PAYLOAD_DROP`, `GEOFENCE`, `HOME_RESET`, `CAMERA_STAB`, `MOTOR_REVERSE`, `COUNT`.

`enum class Knob : uint8_t` — les potentiomètres : `STAB_GAIN`, `MAX_BANK`,
`CRUISE_SPEED`, `FLAPS`, `CAMERA_TILT`, `RATES`, `LOITER_RADIUS`, `THROTTLE_LIMIT`, `COUNT`.

`namespace AutopilotNames` — `mode()`, `modeShort()` (≤ 5 caractères),
`feature()`, `knob()` : les noms pour le journal, l’OLED, le tableau de bord et
MAVLink.

### `PilotInputs`

L’état des interrupteurs et potentiomètres pour un cycle.

| Membre | Description |
|---|---|
| `bool has(Feature) const` | la fonction est activée |
| `float knob(Knob) const` | la position du potentiomètre −1…+1 |
| `bool isBound(Knob) const` | le potentiomètre figure dans le tableau des affectations |
| `float knobValue(Knob, min, default, max) const` | en unités : le centre vaut `default`, les extrémités `min`/`max` ; non affecté — `default` |

---

## `Binding`, `Bind`, `BindingCheck`

**Fichier :** `autopilot/ControlBinding.h` · le tableau — `config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`,
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`. Les lignes du tableau sont les
fabriques de `namespace Bind` (toutes `constexpr`) :

| Fabrique | Signification |
|---|---|
| `modes(ch, up, middle, down)`, `modes(ch, up, down)` | un interrupteur de sélection de mode (la zone d’après `PilotSwitches::zoneOf`) |
| `mode(ch, m)` | un mode par-dessus tant que le canal ≥ `SWITCH_ON_US` |
| `feature(ch, f)` | une fonction tant que le canal ≥ `SWITCH_ON_US` |
| `knob(ch, k)` | un potentiomètre, `(us − 1500) / 500`, borné à ±1 |

`namespace BindingCheck` — des fonctions `constexpr` récursives (le cœur ESP32
est compilé en C++11) : `channelIsFree`, `channelsFree`, `channelsUnique`,
`modeSwitchCount`, `atMostOneModeSwitch`. Elles servent dans les `static_assert`
de `Controls.h`.

---

## `PilotSwitches`

**Fichier :** `autopilot/PilotSwitches.h` · **Dépend de :** `Autopilot*`, `RcChannelState`, le tableau des affectations

| Méthode | Description |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | un tableau personnalisé (tests, simulations) |
| `explicit PilotSwitches(Autopilot* = nullptr)` | le tableau `Controls::BINDINGS` |
| `void update(const RcChannelState&)` | rassembler `PilotInputs` et les passer à `autopilot->setInputs()` ; `setMode()` — **uniquement quand le résultat des interrupteurs a changé** (un mode imposé depuis le tableau de bord ou la station sol n’est pas écrasé à chaque cycle). `FlightController` ne l’appelle que lorsque la liaison est vivante |
| `void printBindings() const` | la disposition sur Serial à la mise sous tension : `SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (up / middle / down)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`, `"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2 zones : < 1500 / ≥ 1500 ; 3 zones : < 1250 / < 1750 / ≥ 1750 |
| `getInputs()`, `binding(i)` | pour la télémétrie et les tests |

`Bind::mode` l’emporte sur `Bind::modes` ; parmi plusieurs `Bind::mode`
activés, c’est la ligne du haut qui gagne.

---

## `Autopilot`

**Fichier :** `autopilot/Autopilot.h` · **Dépend de :** `PidController`, `Navigation`, `AltitudeSpeedController`, `LaunchController`, `TakeoffController`, `SoaringController`, `AutoTrim`, les capteurs (tous nullables)

### Cycle de vie

| Méthode | Description |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | Le PID de roulis/tangage Kp 5, Ki 0.5, Kd 0.5, sortie ±500 µs |
| `bool begin()` | charger le compensateur ; `false` et un message s’il n’y a pas d’IMU ou de baromètre |
| `void setInputs(const PilotInputs&)` | les interrupteurs et potentiomètres de ce cycle (avant `update`) |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | une fois par cycle : capteurs (toujours) → navigation et point de départ → enregistrement du compensateur au sol → failsafe → géobarrière → mode → coordination du virage → auto-trim |
| `ControlCommand getCommand() const` | les ordres finaux aux gouvernes (roll/pitch/yaw, µs) |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | les gaz du mode : `PILOT` — ceux du pilote ; `AUTO` — les siens (y compris au décollage automatique : le manche des gaz ne fait que lancer et interrompre). L’ARM et `MOTOR_KILL` sont gérés par `FlightController` |

### Modes

| Méthode | Description |
|---|---|
| `void setMode(AutopilotMode)` | le même ou ≥ `MODE_COUNT` — rien ; sinon remise à zéro du PID et des automates, objectifs = le cap et l’altitude actuels, centre des cercles = le point actuel (avec GPS), RTH — l’altitude de retour |
| `getMode()`, `getModeName()` | le nom : `FAILSAFE_GLIDE` / `FAILSAFE_RTH` en cas de perte de liaison, sinon le mode |
| `isFailsafeActive()`, `isFailsafeGliding()`, `isFailsafeReturning()` | le failsafe par-dessus le mode |
| `isAutoThrottle()`, `getThrottleCorrection()` | les gaz du mode (%, pour le journal et le tableau de bord) |
| `getLaunchState()`, `getSoaringState()` | les automates LAUNCH et SOARING |
| `getTakeoffStage()`, `getTakeoffAbortReason()` | l’étape du décollage automatique et pourquoi il a été interrompu (`nullptr` — pas interrompu) |
| `bool takeTakeoffRefusal()` | le décollage automatique a refusé au contrôle avant le départ — `FlightController` joue un signal (lu une seule fois) |
| `getLandingGear()`, `setLandingGear(LandingGear)` | le train d’atterrissage (par défaut `Config::LANDING_GEAR` ; les tests le changent) |
| `bool isGyroOnlyAttitude() const` | l’horizon est mené en ce moment par le seul gyroscope (fenêtre d’accélération du décollage) |

### Sorties et diagnostic

| Méthode | Description |
|---|---|
| `getRollCorrection()`, `getPitchCorrection()`, `getYawCorrection()` | ordre − manches, µs |
| `getDesiredRoll()`, `getDesiredPitch()`, `getTargetAltitude()` | les objectifs |
| `const NavStatus& getNavStatus()` | GPS, point de départ, position, distance/relèvement vers le départ, cap et cap visé, vitesse pour la navigation, géobarrière, décrochage |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | l’altitude barométrique (m depuis le point de mise sous tension) |
| `getInputs()`, `getAutoTrim()` | pour la télémétrie |
| `getImuSensor()` … `getAirspeedSensor()` | les capteurs (peuvent valoir `nullptr`) |
| `getRollPid()`, `getPitchPid()`, `setPIDGains(...)` | le PID (tableau de bord, paramètres MAVLink) |

### Mécanique interne

- `stabilize()` — un PID d’angle avec D issu du gyroscope, multiplié par
  `Knob::STAB_GAIN` ; l’intégrateur ne s’accumule qu’en ARM et avec une erreur
  < `STAB_INTEGRATOR_ZONE_DEG`. `stabilizeOrManual()` — sans IMU, les gouvernes
  restent au pilote ; `stabilizeOrNeutral()` — sans IMU, neutre (modes
  automatiques).
- `imuReady()` = l’IMU existe, est disponible et sans problème de vérification
  avant vol.
- La vitesse pour la navigation : tube de Pitot → GPS → `NAV_ASSUMED_SPEED_MS`.
- `looksLanded()` — près du sol d’après le baromètre, presque sans vitesse
  verticale, plus lent que le seuil du tube de Pitot/GPS : c’est alors seulement
  que le compensateur est écrit en flash.
- Failsafe : avec GPS et point de départ — RTH avec le moteur, sinon plané ; un
  RTH commencé n’est pas abandonné à cause d’une brève perte du GPS. En AUTO_TAKEOFF sur la piste (étapes avec `holdsGround()`), la perte de liaison est un décollage interrompu, pas un failsafe ; en montée, c’est le failsafe, et au retour de la liaison la montée continue.
- La fenêtre « gyroscope seul » (`manageAccelCorrection()`) : le démarrage du moteur au décollage automatique retire l’accéléromètre comme appui de l’horizon (`ImuSensor::setAccelCorrection`) ; le remettent le temps écoulé depuis le début de la montée (`TAKEOFF_GYRO_ONLY_CLIMB_MS`), la limite, un décollage interrompu et le DISARM — pas un changement de mode.

---

## `Geo`, `Guidance`, `GeoPoint`

**Fichier :** `autopilot/Navigation.h`

Un plan local « nord/est » en mètres (une projection équirectangulaire — pour
des kilomètres, l’erreur est une fraction de pour cent).

| Fonction | Description |
|---|---|
| `Geo::wrap180`, `Geo::wrap360` | normalisation des angles |
| `Geo::offsetNE(a, b, north, east)`, `distance(a, b)`, `bearing(a, b)` | décalage, distance, relèvement 0..360 |
| `Geo::moved(a, north, east)` | un point décalé |
| `Geo::fromGps(GpsData)` | un `GeoPoint` à partir du GPS |
| `Guidance::rollForCourse(target, course, bankLimit)` | l’inclinaison pour une erreur de cap (`NAV_COURSE_GAIN`), bornée |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | le cap du champ de vecteurs vers un cercle (`LOITER_CONVERGENCE`) |
| `Guidance::orbitBankDeg(speed, radius)` | l’inclinaison anticipée d’un cercle : atan(V²/(g·R)) |

## `AltitudeSpeedController`

**Fichier :** `autopilot/AltitudeSpeedController.h` — TECS-lite.

| Méthode | Description |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | la vitesse verticale voulue = `NAV_ALT_GAIN`·erreur (≤ `NAV_MAX_CLIMB/SINK`) ; tangage = l’anticipation asin(Vz/V) + un PI sur l’erreur de Vz, dans les limites de `NAV_MAX_CLIMB/DIVE_PITCH_DEG` |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | avec un tube de Pitot — un PI sur la vitesse air autour de `cruisePct` ; sans — `cruisePct` ; + `THROTTLE_PER_CLIMB_PCT` pour la montée requise |
| `reset()`, `getWantedClimb()` | |

## `LaunchController`

**Fichier :** `autopilot/LaunchController.h`

`State` : `IDLE → READY` (gaz relevés) `→ THROWN` (une surcharge > `LAUNCH_ACCEL_G`
pendant plus de `LAUNCH_ACCEL_TIME_MS`) `→ CLIMB` (après `LAUNCH_MOTOR_DELAY_MS` : le
moteur, tangage `LAUNCH_CLIMB_PITCH_DEG`) `→ DONE` (`LAUNCH_CLIMB_MS` ou
`LAUNCH_ALTITUDE_M`). Bouger les manches avant le lancer — annulation. Méthodes :
`update(...)`, `reset()`, `getState()`, `motorOn()`, `pitchTargetDeg()`,
`stateName()`.

## `TakeoffController`

**Fichier :** `autopilot/TakeoffController.h` · **Dépend de :** `Config`

Le décollage automatique depuis une piste — des étapes d’après les capteurs, sans capteurs ni gouvernes à lui : en entrée des `Inputs` (ARM, liaison, gaz du pilote, angles et vitesses angulaires, facteur de charge, assiette au sol, altitude, vitesse, potentiomètre `STAB_GAIN`, temps), en sortie ce qu’il faut faire pendant ce cycle.

`Stage` : `IDLE` (moteur arrêté ; gaz au-dessus de `TAKEOFF_TRIGGER_US` après avoir été en bas) `→ CHECK` (immobile pendant `TAKEOFF_CHECK_MS`, angles = assiette au sol ; sinon `ABORTED` et un signal) `→ TAIL_UP` (roulette de queue : à piquer, queue en l’air) `→ ROLL` (course ; `TD_ROLL_MS` / `TRI_ROLL_MS` ou la vitesse de rotation) `→ LIFTOFF` (la cible de tangage monte, près du sol au plus l’assiette au sol moins `TD_TAIL_CLEARANCE_DEG`) `→ CLIMB` (décollage d’après le baromètre ou le temps ; montée jusqu’à `TAKEOFF_SAFE_ALTITUDE_M` / `TAKEOFF_CLIMB_MS`) `→ DONE`. `ABORTED` — décollage interrompu ; gaz en bas 1 s — de nouveau `IDLE`.

| Méthode | Description |
|---|---|
| `void update(const Inputs&)` | une fois par cycle : changement d’étape, puis les sorties de cette étape ; sans ARM — `reset()` |
| `void abort(const char* reason, uint32_t nowMs)` | interruption de l’extérieur (perte de liaison) ; en `IDLE`/`DONE` — rien |
| `void reset()`, `setGear()`, `getGear()` | |
| `getStage()`, `stageName()`, `stageShort()`, `getAbortReason()` | l’étape (nom court pour l’OLED) |
| `bool holdsGround() const` | au sol (`IDLE`…`LIFTOFF`, `ABORTED`) : la perte de liaison est un décollage interrompu, les intégrateurs des PID restent à zéro |
| `isAirborneStage()`, `isDone()` | en l’air (montée/terminé) ; décollage terminé |
| `getThrottlePct()` | gaz du programme, % (rampe sur `TAKEOFF_THROTTLE_RAMP_MS`) |
| `getRollTargetDeg()`, `getPitchTargetDeg()` | cible de roulis (sur les roues — le roulis à l’arrêt) et de tangage (en montée — pour le PID) |
| `hasElevatorCommand()`, `getElevatorUs()` | sur les roues, la profondeur directement : vers la cible avec `TD_PITCH_KP` / `TD_ROLL_PITCH_KP` µs par degré, amortissement `TD_PITCH_KD`, au plus `TD_TAIL_UP_ELEVATOR_US` à piquer |
| `hasRudderCommand()`, `getRudderUs()` | le cap à la direction et à la roue : `TAKEOFF_HEADING_KP` µs par degré, `TAKEOFF_YAW_RATE_KD` par °/s |
| `bool takeRefusal()`, `bool takeGyroTrim(float[3])` | événements uniques : refus au contrôle ; le zéro résiduel du gyroscope mesuré à l’arrêt |

## `SoaringController`

**Fichier :** `autopilot/SoaringController.h`

`State` : `GLIDE ⇄ THERMAL` (le variomètre > `SOAR_THERMAL_CLIMB_MS` pendant plus de
`SOAR_THERMAL_CONFIRM_MS` / la moyenne < `SOAR_EXIT_CLIMB_MS` sur
`SOAR_EXIT_WINDOW_MS`), `→ MOTOR_CLIMB` (sous `SOAR_MIN_ALTITUDE_M`, jusqu’à
`SOAR_MAX_ALTITUDE_M`), `→ RETURN` (au-delà de `SOAR_MAX_DISTANCE_M`, jusqu’à 70 %
de celle-ci). Méthodes : `update(climb, alt, distHome, dt, now)`, `reset(now)`,
`getState()`, `motorOn()`, `getAverageClimb()`, `stateName()`.

## `AutoTrim`

**Fichier :** `autopilot/AutoTrim.h` · stockage — `Preferences` (NVS / flash de la STM32), espace `"autotrim"`

| Méthode | Description |
|---|---|
| `void load()` | le compensateur depuis la NVS (absent — 0) |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += ordre · `AUTOTRIM_RATE` · dt, jusqu’à ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | écrire si cela a changé (appelé par `Autopilot` après le DISARM au sol) |
| `reset()`, `getRoll()`, `getPitch()` | |

---

## `PidController`

**Fichier :** `autopilot/PidController.h` · **Dépend de :** `Config` (le `dt` nominal)

| Méthode | Description |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`, `getKp/Ki/Kd()` | les gains |
| `setLimits(minOut, maxOut)` | la limite de la sortie (±500 par défaut) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | la sortie, bornée à `[min, max]` |
| `void reset()` | remettre l’intégrateur à zéro, le `dt` se compte à partir de « maintenant » |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (la vitesse issue du capteur !)
out = constrain(P + I + D, min, max)
```

Le D est pris sur la vitesse de la grandeur mesurée (gyroscope, °/s) et non sur
la dérivée de l’erreur : pas de bruit de dérivation et pas de saut quand la
consigne change. Le `dt` provient de `micros()` ; le premier appel après
`reset()` ou après une pause de plus de 0,1 s utilise le `LOOP_PERIOD_MS`
nominal.
