# AUTOPILOT / feedback — el lazo de realimentación (base preparada)

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/feedback.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual.

[← Referencia](README.md)

> ⚠️ **Base preparada, no conectada al firmware.** Ni `FlightController`, ni
> `Autopilot`, ni `main.cpp` incluyen estas cabeceras. Se comprueban con una
> simulación en lazo cerrado (`test/test_feedback`, en el PC y en la placa) y
> con pruebas unitarias nativas. El plan de conexión está en
> [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#plan-de-conexión).

La idea: en lugar de un PID sobre el ángulo con ganancias ajustadas a una sola
velocidad, un regulador cerrado sobre **la respuesta del avión**, con un modelo
del eje aprendido en vuelo, protección contra la pérdida de sustentación y
fases de despegue y aterrizaje guiadas por los sensores. La única entrada es
`FlightSnapshot`; la única salida, `FeedbackOutput`.

Todos los módulos son solo de cabecera; `FeedbackModules.h` los incluye en una
sola línea.

---

## namespace `FeedbackConfig`

**Archivo:** `autopilot/feedback/FeedbackConfig.h`

Todas las constantes del lazo (al conectarlo pasarán a `Config.h`). Los valores
marcados «прикидка» («estimación aproximada») son para un modelo de ~1 kg y 1.2 m de envergadura. Los
arrays `[AXIS_COUNT]` se indexan por eje.

| Grupo | Constantes |
|---|---|
| General | `GRAVITY = 9.80665`; los ejes `AXIS_ROLL = 0`, `AXIS_PITCH = 1`, `AXIS_YAW = 2`, `AXIS_COUNT = 3` |
| Velocidad | `STALL_SPEED_MS = 8`, `REFERENCE_SPEED_MS = 14`, `ACCEL_FILTER_TAU_S = 0.3` |
| En el aire/en tierra | `AIRBORNE_HEIGHT_M = 3`, `AIRBORNE_CONFIRM_MS = 500`, `GROUND_STILL_MS = 2000`, `GROUND_ACCEL_TOLERANCE_G = 0.1` |
| Regulador | `ANGLE_GAIN = {4, 4, 2}` 1/s, `MAX_RATE_DPS = {120, 60, 30}`, `RATE_TAU_S = {0.15, 0.20, 0.30}`, `RATE_INTEGRAL_GAIN = {2, 2, 1}`, `MAX_DEFLECTION_US = {400, 400, 400}`, `DAMPING_COMPENSATION = 0.5` |
| Efectividad de las superficies | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs, `EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`, `EFFECTIVENESS_MAX = {30, 15, 6}`, `RESPONSE_DELAY_MS = 40`, `RLS_FORGETTING = 0.995`, `ESTIMATOR_PERIOD_MS = 20`, `ESTIMATOR_PREFILTER_HZ = 2`, `MIN_EXCITATION_US = 30` |
| Pérdida de sustentación | `DECEL_WARN_MS2 = 2`, `DECEL_CONFIRM_MS = 300`, `LOW_ENERGY_PITCH_DEG = 5`, `NOSE_DROP_RATE_DPS = 60`, `WING_DROP_RATE_DPS = 120`, `STALL_NOSE_UP_COMMAND_US = 50`, `LOW_EFFECTIVENESS_RATIO = 0.35`, `LOW_SPEED_MARGIN = 1.25`, `LOW_SPEED_EXIT_MARGIN = 1.5`, `LOW_ENERGY_THROTTLE_PERCENT = 80`, `LOW_ENERGY_MAX_PITCH_DEG = 5`, `STALL_THROTTLE_PERCENT = 100`, `STALL_MAX_PITCH_DEG = −5`, `STALL_MAX_BANK_DEG = 10`, `STALL_AILERON_LIMIT_US = 150`, `RECOVERY_HOLD_MS = 1000` |
| Despegue | `TAKEOFF_HAND_LAUNCH = false`, `TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`, `TAKEOFF_THROTTLE_PERCENT = 100`, `LAUNCH_ACCEL_G = 1`, `LAUNCH_DETECT_MS = 50`, `ROTATE_SPEED_MS = 10`, `ROTATE_FALLBACK_MS = 1500`, `CLIMB_PITCH_DEG = 12`, `TAKEOFF_TARGET_ALTITUDE_M = 30`, `TAKEOFF_CLIMB_FALLBACK_MS = 10000`, `LAUNCH_TIMEOUT_MS = 8000`, `HEADING_HOLD_GAIN = 2` |
| Aterrizaje | `APPROACH_SINK_RATE_MS = 1`, `APPROACH_THROTTLE_PERCENT = 25`, `APPROACH_BASE_PITCH_DEG = −3`, `APPROACH_MIN_PITCH_DEG = −10`, `APPROACH_MAX_BANK_DEG = 20`, `GO_AROUND_THROTTLE_PERCENT = 80`, `SINK_TO_PITCH_GAIN = 4`, `FLARE_HEIGHT_M = 2`, `FLARE_SINK_RATE_MS = 0.3`, `FLARE_MAX_PITCH_DEG = 8`, `TOUCHDOWN_ACCEL_G = 0.5`, `TOUCHDOWN_HEIGHT_M = 0.3`, `TOUCHDOWN_STILL_MS = 500`, `TOUCHDOWN_STILL_RATE_DPS = 5`, `ROLLOUT_MS = 5000` |

---

## namespace `FeedbackMath`

**Archivo:** `autopilot/feedback/FeedbackMath.h` · **Depende de:** `<math.h>`

| Función | Descripción |
|---|---|
| `float wrap180(float deg)` | Un ángulo en `(−180, 180]`: la diferencia entre los rumbos 350° y 10° es −20° |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | Limita a `[−limit, limit]` |

---

## `FlightSnapshot`

**Archivo:** `autopilot/feedback/FlightSnapshot.h` · **Clase:** struct

Todo lo que el lazo sabe del avión en un ciclo. Los signos son aeronáuticos.

| Grupo | Campos |
|---|---|
| Tiempo/estado | `timeUs`, `armed`, `linkLost` |
| Actitud | `imuValid`, `rollDeg`, `pitchDeg`, `yawDeg`, `rollRateDps`, `pitchRateDps`, `yawRateDps`, `accelXg/Yg/Zg` |
| Altitud | `baroValid`, `altitudeM` (desde el punto de encendido), `climbRateMs`, `heightAglValid`, `heightAglM` (un futuro telémetro) |
| Velocidad | `airspeedValid`, `airspeedMs` (un futuro tubo de Pitot), `gpsValid`, `groundSpeedMs` |
| Objetivos del modo | `stabilizationActive` (false = MANUAL: solo aprendizaje), `targetRollDeg`, `targetPitchDeg` |
| Órdenes, µs | `stick*Us`: la aportación del piloto; `command*Us`: el resultado que realmente fue a las superficies |
| Acelerador, % | `pilotThrottlePercent`, `throttlePercent` (realmente al ESC) |
| Flaps | `flapsUs`, `flapsMoving` |

---

## `FeedbackOutput`

**Archivo:** `autopilot/feedback/FeedbackOutput.h` · **Clase:** struct

| Campo | Descripción |
|---|---|
| `float deflectionUs[3]` | Deflexiones de las superficies por eje, µs (los signos de `ControlCommand`) |
| `bool axisEnabled[3]` | `false`: el eje no se controla, la superficie queda en manos del piloto |
| `float throttleOverridePercent` | El acelerador absoluto de una fase de vuelo; `< 0`: no fijado |
| `float throttleFloorPercent` | El límite inferior del acelerador (protección contra la pérdida de sustentación); `< 0`: ninguno |
| `targetRollDeg`, `targetPitchDeg` | Los objetivos finales tras las limitaciones (depuración) |
| `const char* reason` | Una descripción breve para el registro/OLED |

---

## `PhaseTargets`

**Archivo:** `autopilot/feedback/PhaseTargets.h` · **Clase:** struct

La salida común de `TakeoffSequencer` y `LandingSequencer`: el «qué», no el
«cómo».

| Campo | Por defecto | Descripción |
|---|---|---|
| `active` | `false` | La fase está controlando ahora el avión |
| `targetRollDeg`, `targetPitchDeg` | 0 | Los objetivos |
| `controlRoll`, `controlPitch` | `true` | `false`: no tocar el eje (sobre ruedas el cabeceo lo fija el tren de aterrizaje) |
| `holdHeading`, `headingDeg` | `false`, 0 | Mantener el rumbo con el timón de dirección y la rueda |
| `throttlePercent` | −1 | −1: el acelerador del piloto |
| `reason` | `""` | Una descripción |

---

## `SpeedEstimator`

**Archivo:** `autopilot/feedback/SpeedEstimator.h`

La velocidad (aerodinámica > velocidad sobre el suelo del GPS > desconocida) y
la aceleración longitudinal a partir de la IMU: `dV/dt = g · (ax − sin θ)` a
través de un filtro paso bajo `ACCEL_FILTER_TAU_S`: se ve que «la velocidad
cae» incluso sin sensor de velocidad.

| Método | Descripción |
|---|---|
| `void update(const FlightSnapshot&)` | Un paso; `dt ≤ 0` o `> 0.5 s` desde la llamada anterior (para la primera, desde `timeUs = 0`): se omite |
| `bool hasSpeed() const`, `float getSpeed() const`, `Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`, `float getAcceleration() const` | m/s², «+»: acelerando; sin IMU, `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`, limitado a `0.05..4`; sin velocidad, 1 |

---

## `AirborneDetector`

**Archivo:** `autopilot/feedback/AirborneDetector.h`

Si el avión está en el aire: aprender, acumular la integral y buscar la pérdida
de sustentación solo tiene sentido en vuelo.

| Método | Descripción |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | No armado: se restablece a «en tierra». Un candidato a cambio de estado debe mantenerse `AIRBORNE_CONFIRM_MS` (despegue) o `GROUND_STILL_MS` (aterrizaje) |
| `void force(bool)` | Fijarlo explícitamente (lo saben el despegue y el aterrizaje) |
| `void reset()` | En tierra |
| `bool isAirborne() const` | |

«Parece vuelo»: la altura por telémetro o barómetro > `AIRBORNE_HEIGHT_M`, o la
velocidad > `ROTATE_SPEED_MS`. «Parece tierra»: bajo, las velocidades
angulares de todos los ejes < `TOUCHDOWN_STILL_RATE_DPS`, |a| ≈ 1g (± `GROUND_ACCEL_TOLERANCE_G`).

---

## `ControlEffectivenessEstimator`

**Archivo:** `autopilot/feedback/ControlEffectivenessEstimator.h`

Un eje. El modelo: **aceleración angular = b·superficie(t − retardo) + a·ω + c**.
`b` es la efectividad de la superficie (°/s² por µs, el signo es el sentido de
la respuesta), `a` es el amortiguamiento (1/s, normalmente < 0), `c` es un
momento constante (autotrim). `b` se aprende a una velocidad de referencia:
`b = b_ref · (V/V_ref)²`, `a = a_ref · V/V_ref`. La estimación son mínimos
cuadrados recursivos con olvido (`λ = 0.995`, memoria de ~4 s).

| Método | Descripción |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | Llama a `reset()` |
| `void reset()` | θ = (prior, 0, 0); covarianza: b ± prior, a ± 5, c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | Llamar en cada ciclo: acumula medias en un intervalo de `ESTIMATOR_PERIOD_MS`; al final del intervalo, la aceleración a partir de la diferencia del giroscopio, el retardo de la orden, un filtro paso bajo común a ambos lados y un paso de RLS (si se permite aprender y hay excitación) |
| `getEffectiveness()` | `b` a la velocidad actual |
| `getReferenceEffectiveness()` | `b` a la velocidad de referencia |
| `getDamping()` | `a` a la velocidad actual |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b` (0 si `\|b\|` es pequeño) |
| `getEffectivenessSigma()` | σ de la estimación de `b` a la velocidad actual |
| `bool isConfident() const` | ≥ 50 pasos de RLS, `\|b\|` ≥ el mínimo y σ < 0.3·`\|b\|` |
| `getAngularAccel()` | La aceleración del último intervalo (depuración) |

Particularidades:

- Un intervalo más largo que `MAX_GAP_MS = 200` (el ciclo se detuvo): los datos
  vuelven a empezar (se reinician el historial del retardo y el filtro).
- Solo aprende con **excitación**: la amplitud de las órdenes medias en 16
  intervalos (~0.3 s) ≥ `MIN_EXCITATION_US`; si no, la estimación se congela.
- Higiene de los float tras un paso: la simetría de `P`, un techo para las
  varianzas (×10 de las iniciales), un límite para `b` (`±EFFECTIVENESS_MAX`) y
  para `a` (`−40..5`).

---

## `AxisModel`

**Archivo:** `autopilot/feedback/AdaptiveRateController.h` · **Clase:** struct

Lo que se sabe de la respuesta de un eje, para el regulador: `effectiveness` (b,
por defecto 1), `damping` (a, 0: no compensar), `bias` (c, 0).

---

## `AdaptiveRateController`

**Archivo:** `autopilot/feedback/AdaptiveRateController.h`

Un regulador de un eje, tres etapas:

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| Método | Descripción |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | La integral, la saturación y la salida, a 0 |
| `float angleToRate(targetDeg, angleDeg) const` | Etapa 1 (el camino más corto para el rumbo) |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | Etapas 2–3, devuelve la deflexión, µs |
| `getDesiredRate()`, `getIntegral()`, `getOutput()`, `isSaturated()` | Estado |

Invariantes: `|b|` no es menor que `EFFECTIVENESS_MIN` (conservando el signo de
b); la integral se guarda en °/s (sigue siendo correcta cuando cambia `b`) y
**no se acumula hacia el tope** (anti-windup según el sentido de la saturación
del paso anterior); con `dt ≤ 0` la integral no cambia.

---

## `StallGuard`

**Archivo:** `autopilot/feedback/StallGuard.h`

Protección contra la pérdida de velocidad y la pérdida de sustentación. Los
niveles son `Level::{Normal, LowEnergy, Stall}`.

| Método | Descripción |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | En tierra o sin IMU, se restablece a Normal |
| `Level getLevel() const`, `const char* getLevelName() const` | `"OK"`, `"LOW_ENERGY"`, `"STALL"` |
| `const char* getReason() const` | El último indicio que saltó |
| `float maxPitchDeg() const` | Stall: −5°, LowEnergy: 5°, en otro caso 90° |
| `float maxBankDeg() const` | Stall: 10°, en otro caso 180° |
| `float maxAileronUs() const` | Stall: 150 µs, en otro caso `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall 100 %, LowEnergy 80 %, en otro caso/sin enlace −1 |
| `void reset()` | Normal |

`StallGuard::ControlState`: `pitchEffectivenessKnown`, `pitchEffectiveness`
(el módulo de la estimación de b en cabeceo).

Indicios de **LowEnergy**: una deceleración confirmada (`DECEL_CONFIRM_MS`)
superior a `DECEL_WARN_MS2` con un cabeceo > 5°; velocidad < `1.25·Vs`; una
efectividad fiable del timón de profundidad < 35 % de la a priori. Indicios de
**Stall**: velocidad < Vs; el morro cae más deprisa de 60 °/s con el timón de
profundidad «arriba» > 50 µs; con poca energía el ala cae más deprisa de
120 °/s contra los alerones. Las medidas se levantan pasados
`RECOVERY_HOLD_MS` y solo cuando la energía se ha recuperado (velocidad ≥
`1.5·Vs`; sin sensor de velocidad, aceleración ≥ 0).

---

## `TakeoffSequencer`

**Archivo:** `autopilot/feedback/TakeoffSequencer.h`

Despegue por fases. `State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`.
El diagrama está en [ARCHITECTURE.md §7](../ARCHITECTURE.md#despegue-y-aterrizaje-lazo-de-realimentación-no-conectado).

| Método | Descripción |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | Una fase activa → `Aborted`; los objetivos se restablecen |
| `void update(snapshot, speed, nowMs)` | Como máximo una transición por ciclo, y después los objetivos de la nueva fase |
| `void reset()` | → `Idle` |
| `getTargets()`, `getState()`, `getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

Los objetivos de las fases: la espera, gas 0 y las superficies en manos del
piloto; la carrera, gas 100 %, alas niveladas, sin tocar el cabeceo, mantener
el rumbo fijado en el momento del inicio; el ascenso, gas 100 %, alas niveladas,
cabeceo `CLIMB_PITCH_DEG`. Un lanzamiento a mano: la aceleración longitudinal
`ax − sin θ ≥ LAUNCH_ACCEL_G` durante más de `LAUNCH_DETECT_MS`.

---

## `LandingSequencer`

**Archivo:** `autopilot/feedback/LandingSequencer.h`

Aterrizaje por fases. `State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`.

| Método | Descripción |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`, `void reset()`, `update(snapshot, nowMs)` | Como en el despegue |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout: aquí se desactiva la protección contra la pérdida de sustentación |
| `bool isOnGround() const` | Rollout / Complete |

El cabeceo en el descenso y en el enderezamiento sale del error de la velocidad
vertical: `θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`, limitado a
`[min, FLARE_MAX_PITCH_DEG]`; sin barómetro, el ángulo base. La altura, del
telémetro y, si no, del barómetro. El toque: un pico de |a − 1g| ≥ 0.5g o «bajo
y sin girar» durante `TOUCHDOWN_STILL_MS`. En la carrera de aterrizaje el
rumbo se fija en el momento del toque.

---

## `FeedbackSupervisor`

**Archivo:** `autopilot/feedback/FeedbackSupervisor.h`

El lazo completo. Es propietario de `SpeedEstimator`, `AirborneDetector`, tres
`ControlEffectivenessEstimator`, tres `AdaptiveRateController`, `StallGuard`,
`TakeoffSequencer` y `LandingSequencer`.

| Método | Descripción |
|---|---|
| `bool requestTakeoff()` | Solo armado, con enlace y en tierra; cancela un aterrizaje |
| `bool requestLanding()` | Solo armado, con enlace y en el aire; cancela un despegue |
| `void cancelPhase()` | Cancelar la fase |
| `const FeedbackOutput& update(const FlightSnapshot&)` | Un ciclo (el orden está en [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-el-lazo-de-realimentación-no-conectado)) |
| `getOutput()`, `isAirborne()`, `getSpeedEstimator()`, `getEstimator(axis)`, `getController(axis)`, `getStallGuard()`, `getTakeoff()`, `getLanding()` | Estado para el registro y las pruebas |
| `void printStatus(Print& out) const` | Una línea de estado + una línea por eje (`b ± σ`, `*`: fiable, `a`, `c`, `I`, la salida) |

Reglas clave:

- **No armado**: todos los ejes desactivados, `reason = "no armado"`; el
  ARM/DISARM (un vuelo nuevo) borra todo lo aprendido.
- **La pérdida de enlace** cancela las fases; no se toca el acelerador (actúa
  el failsafe del firmware).
- **El despegue del suelo** reinicia las estimaciones y los reguladores (lo que
  se «vio» sobre las ruedas no sirve).
- Una estimación fiable y negativa de `b` **nunca** pasa al regulador: el eje
  trabaja con el modelo a priori, y en `reason` aparece la advertencia «… ¿responde
  a la superficie al revés? comprobar en tierra».
- La integral está congelada en tierra, salvo el rumbo en la carrera de
  despegue y de aterrizaje.
- Viraje coordinado (con la velocidad conocida en el aire): a la velocidad de
  cabeceo deseada se suma `+ g/V · sin φ · tg φ`, y a la de guiñada, `g/V · sin φ`
  (el alabeo se limita a ±60°).
- La prioridad de `reason`: pérdida de sustentación > poca energía > fase >
  advertencia de signo > «estabilización»/«manual (aprendizaje)».
