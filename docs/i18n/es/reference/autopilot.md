# AUTOPILOT — modos, navegación, interruptores

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/autopilot.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual.

[← Referencia](README.md)

El piloto automático recibe los sticks del piloto, los interruptores y
potenciómetros (`PilotInputs`) y los sensores, y entrega **la orden final a las
superficies** (`getCommand()`) y el acelerador del modo (`applyThrottle()`). Sin
un sensor necesario, el modo se comporta de forma segura (las superficies
quedan en manos del piloto o en neutro) y no falla. Lo que hace cada modo para
el piloto está en [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md). En vuelo, hasta
ahora solo se ha usado el modo manual; el piloto automático se ha probado en el
banco, con pruebas y con simulaciones en lazo cerrado (`test/native/test_sim`).

---

## `AutopilotMode`, `Feature`, `Knob`

**Archivo:** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t` (sin ámbito: los códigos numéricos van al JSON de
`/api/setmode`, `/api/status` y a `MavlinkModes`):

| Valor | Código | Corto (OLED) | Esencia |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | superficies = sticks |
| `MODE_STABILIZE` | 1 | STAB | el stick es el ángulo de alabeo/cabeceo |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | un programa de despegue guiado por el acelerador del piloto |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + altitud con el timón de profundidad |
| `MODE_ACRO` | 4 | ACRO | el stick es la velocidad angular |
| `MODE_CRUISE` | 5 | CRZ | rumbo + altitud + acelerador automático |
| `MODE_LOITER` | 6 | LOIT | círculos sobre el punto donde se activó |
| `MODE_RTH` | 7 | RTH | a casa, círculos sobre el punto de origen |
| `MODE_LAUNCH` | 8 | LNCH | lanzamiento a mano |
| `MODE_AUTO_LAND` | 9 | LAND | planeo + enderezamiento |
| `MODE_SOARING` | 10 | SOAR | térmicas sin motor |
| `MODE_RESCUE` | 11 | RESQ | alas niveladas, morro arriba, gas |
| `MODE_COUNT` | 12 | | el límite (`setMode()` ignora ≥) |

`enum class Feature : uint8_t`: las funciones de los interruptores: `FLAPS`,
`AIRBRAKE`, `AUTO_TRIM`, `TURN_COORDINATION`, `MOTOR_KILL`, `BEEPER`,
`PAYLOAD_DROP`, `GEOFENCE`, `HOME_RESET`, `CAMERA_STAB`, `COUNT`.

`enum class Knob : uint8_t`: los potenciómetros: `STAB_GAIN`, `MAX_BANK`,
`CRUISE_SPEED`, `FLAPS`, `CAMERA_TILT`, `RATES`, `LOITER_RADIUS`, `COUNT`.

`namespace AutopilotNames`: `mode()`, `modeShort()` (≤ 5 caracteres),
`feature()`, `knob()`: nombres para el registro, el OLED, el dashboard y
MAVLink.

### `PilotInputs`

El estado de los interruptores y potenciómetros en un ciclo.

| Miembro | Descripción |
|---|---|
| `bool has(Feature) const` | la función está activada |
| `float knob(Knob) const` | la posición del potenciómetro −1…+1 |
| `bool isBound(Knob) const` | el potenciómetro está en la tabla de asignaciones |
| `float knobValue(Knob, min, default, max) const` | en unidades: el centro es `default`, los extremos `min`/`max`; si no está asignado, `default` |

---

## `Binding`, `Bind`, `BindingCheck`

**Archivo:** `autopilot/ControlBinding.h` · la tabla: `config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`,
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`. Las filas de la tabla son las
fábricas de `namespace Bind` (todas `constexpr`):

| Fábrica | Significado |
|---|---|
| `modes(ch, up, middle, down)`, `modes(ch, up, down)` | un interruptor de selección de modo (la zona, según `PilotSwitches::zoneOf`) |
| `mode(ch, m)` | un modo por encima mientras el canal ≥ `SWITCH_ON_US` |
| `feature(ch, f)` | una función mientras el canal ≥ `SWITCH_ON_US` |
| `knob(ch, k)` | un potenciómetro, `(us − 1500) / 500`, limitado a ±1 |

`namespace BindingCheck`: funciones `constexpr` recursivas (el núcleo del ESP32
se compila como C++11): `channelIsFree`, `channelsFree`, `channelsUnique`,
`modeSwitchCount`, `atMostOneModeSwitch`. Se usan en los `static_assert` de
`Controls.h`.

---

## `PilotSwitches`

**Archivo:** `autopilot/PilotSwitches.h` · **Depende de:** `Autopilot*`, `RcChannelState`, la tabla de asignaciones

| Método | Descripción |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | una tabla propia (pruebas, simulaciones) |
| `explicit PilotSwitches(Autopilot* = nullptr)` | la tabla `Controls::BINDINGS` |
| `void update(const RcChannelState&)` | reunir `PilotInputs` y pasárselos a `autopilot->setInputs()`; `setMode()`: **solo cuando ha cambiado el resultado de los interruptores** (un modo fijado desde el dashboard o la estación de tierra no se pisa en cada ciclo). `FlightController` lo llama solo con el enlace vivo |
| `void printBindings() const` | la disposición por Serial al encender: `SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (up / middle / down)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`, `"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2 zonas: < 1500 / ≥ 1500; 3 zonas: < 1250 / < 1750 / ≥ 1750 |
| `getInputs()`, `binding(i)` | para la telemetría y las pruebas |

`Bind::mode` tiene prioridad sobre `Bind::modes`; de varios `Bind::mode`
activados gana la fila superior.

---

## `Autopilot`

**Archivo:** `autopilot/Autopilot.h` · **Depende de:** `PidController`, `Navigation`, `AltitudeSpeedController`, `LaunchController`, `SoaringController`, `AutoTrim`, los sensores (todos pueden ser nulos)

### Ciclo de vida

| Método | Descripción |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | El PID de alabeo/cabeceo con Kp 5, Ki 0.5, Kd 0.5, salida ±500 µs |
| `bool begin()` | cargar la compensación; `false` y un mensaje si no hay IMU o barómetro |
| `void setInputs(const PilotInputs&)` | los interruptores y potenciómetros de este ciclo (antes de `update`) |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | una vez por ciclo: sensores (siempre) → navegación y punto de origen → guardado de la compensación en tierra → failsafe → geovalla → modo → coordinación del viraje → autotrim |
| `ControlCommand getCommand() const` | las órdenes finales a las superficies (roll/pitch/yaw, µs) |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | el acelerador del modo: `PILOT`: el del piloto; `AUTO`: el propio; `AT_LEAST`: no menos que el propio (despegue automático). El ARM y `MOTOR_KILL` los trata `FlightController` |

### Modos

| Método | Descripción |
|---|---|
| `void setMode(AutopilotMode)` | el mismo o ≥ `MODE_COUNT`: nada; si no, se reinician el PID y las máquinas de estados, los objetivos = el rumbo y la altitud actuales, el centro de los círculos = el punto actual (con GPS), RTH: la altitud de regreso |
| `getMode()`, `getModeName()` | el nombre: `FAILSAFE_GLIDE` / `FAILSAFE_RTH` al perder el enlace, y si no, el modo |
| `isFailsafeActive()`, `isFailsafeGliding()`, `isFailsafeReturning()` | el failsafe por encima del modo |
| `isAutoThrottle()`, `getThrottleCorrection()` | el acelerador del modo (%, para el registro y el dashboard) |
| `getLaunchState()`, `getSoaringState()` | las máquinas de estados de LAUNCH y SOARING |

### Salidas y diagnóstico

| Método | Descripción |
|---|---|
| `getRollCorrection()`, `getPitchCorrection()`, `getYawCorrection()` | orden − sticks, µs |
| `getDesiredRoll()`, `getDesiredPitch()`, `getTargetAltitude()` | los objetivos |
| `const NavStatus& getNavStatus()` | GPS, punto de origen, posición, distancia/marcación hacia el origen, rumbo y rumbo objetivo, velocidad para la navegación, geovalla, pérdida de sustentación |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | la altitud barométrica (m desde el punto de encendido) |
| `getInputs()`, `getAutoTrim()` | para la telemetría |
| `getImuSensor()` … `getAirspeedSensor()` | los sensores (pueden ser `nullptr`) |
| `getRollPid()`, `getPitchPid()`, `setPIDGains(...)` | el PID (dashboard, parámetros de MAVLink) |

### Mecánica interna

- `stabilize()`: un PID de ángulo con D a partir del giroscopio, multiplicado
  por `Knob::STAB_GAIN`; el integrador se acumula solo con ARM y con un error
  < `STAB_INTEGRATOR_ZONE_DEG`. `stabilizeOrManual()`: sin IMU las superficies
  quedan en manos del piloto; `stabilizeOrNeutral()`: sin IMU, neutro (modos
  automáticos).
- `imuReady()` = la IMU existe, está disponible y no tiene problemas en la
  comprobación prevuelo.
- La velocidad para la navegación: tubo de Pitot → GPS → `NAV_ASSUMED_SPEED_MS`.
- `looksLanded()`: cerca del suelo según el barómetro, casi sin velocidad
  vertical, más lento que el umbral del tubo de Pitot o el GPS: solo entonces
  se escribe la compensación en la flash.
- Failsafe: con GPS y punto de origen, RTH con motor; si no, planeo; un RTH ya
  iniciado no se abandona por una pérdida breve del GPS.

---

## `Geo`, `Guidance`, `GeoPoint`

**Archivo:** `autopilot/Navigation.h`

Un plano local «norte/este» en metros (una proyección equirrectangular: para
kilómetros el error es una fracción de un porcentaje).

| Función | Descripción |
|---|---|
| `Geo::wrap180`, `Geo::wrap360` | normalización de ángulos |
| `Geo::offsetNE(a, b, north, east)`, `distance(a, b)`, `bearing(a, b)` | desplazamiento, distancia, marcación 0..360 |
| `Geo::moved(a, north, east)` | un punto con desplazamiento |
| `Geo::fromGps(GpsData)` | un `GeoPoint` a partir del GPS |
| `Guidance::rollForCourse(target, course, bankLimit)` | el alabeo para un error de rumbo (`NAV_COURSE_GAIN`), limitado |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | el rumbo del campo vectorial hacia una circunferencia (`LOITER_CONVERGENCE`) |
| `Guidance::orbitBankDeg(speed, radius)` | el alabeo anticipado de un círculo: atan(V²/(g·R)) |

## `AltitudeSpeedController`

**Archivo:** `autopilot/AltitudeSpeedController.h`: TECS-lite.

| Método | Descripción |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | la velocidad vertical deseada = `NAV_ALT_GAIN`·error (≤ `NAV_MAX_CLIMB/SINK`); cabeceo = la anticipación asin(Vz/V) + un PI sobre el error de Vz, dentro de `NAV_MAX_CLIMB/DIVE_PITCH_DEG` |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | con tubo de Pitot, un PI sobre la velocidad aerodinámica en torno a `cruisePct`; sin él, `cruisePct`; + `THROTTLE_PER_CLIMB_PCT` por el ascenso requerido |
| `reset()`, `getWantedClimb()` | |

## `LaunchController`

**Archivo:** `autopilot/LaunchController.h`

`State`: `IDLE → READY` (acelerador subido) `→ THROWN` (una sobrecarga > `LAUNCH_ACCEL_G`
durante más de `LAUNCH_ACCEL_TIME_MS`) `→ CLIMB` (tras `LAUNCH_MOTOR_DELAY_MS`: el
motor, cabeceo `LAUNCH_CLIMB_PITCH_DEG`) `→ DONE` (`LAUNCH_CLIMB_MS` o
`LAUNCH_ALTITUDE_M`). Mover los sticks antes del lanzamiento: cancelar. Métodos:
`update(...)`, `reset()`, `getState()`, `motorOn()`, `pitchTargetDeg()`,
`stateName()`.

## `SoaringController`

**Archivo:** `autopilot/SoaringController.h`

`State`: `GLIDE ⇄ THERMAL` (el variómetro > `SOAR_THERMAL_CLIMB_MS` durante más de
`SOAR_THERMAL_CONFIRM_MS` / la media < `SOAR_EXIT_CLIMB_MS` en
`SOAR_EXIT_WINDOW_MS`), `→ MOTOR_CLIMB` (por debajo de `SOAR_MIN_ALTITUDE_M`, hasta
`SOAR_MAX_ALTITUDE_M`), `→ RETURN` (más allá de `SOAR_MAX_DISTANCE_M`, hasta el 70 %
de esa distancia). Métodos: `update(climb, alt, distHome, dt, now)`, `reset(now)`,
`getState()`, `motorOn()`, `getAverageClimb()`, `stateName()`.

## `AutoTrim`

**Archivo:** `autopilot/AutoTrim.h` · almacenamiento: `Preferences` (NVS / flash de la STM32), espacio `"autotrim"`

| Método | Descripción |
|---|---|
| `void load()` | la compensación desde NVS (si no hay, 0) |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += orden · `AUTOTRIM_RATE` · dt, hasta ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | escribir si cambió (lo llama `Autopilot` tras un DISARM en tierra) |
| `reset()`, `getRoll()`, `getPitch()` | |

---

## `PidController`

**Archivo:** `autopilot/PidController.h` · **Depende de:** `Config` (el `dt` nominal)

| Método | Descripción |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`, `getKp/Ki/Kd()` | las ganancias |
| `setLimits(minOut, maxOut)` | el límite de la salida (±500 por defecto) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | la salida, limitada a `[min, max]` |
| `void reset()` | poner a cero el integrador, `dt` se cuenta desde «ahora» |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (¡la velocidad del sensor!)
out = constrain(P + I + D, min, max)
```

La D se toma sobre la velocidad de la magnitud medida (giroscopio, °/s) y no
sobre la derivada del error: sin ruido de derivación y sin saltos al cambiar la
consigna. `dt` sale de `micros()`; la primera llamada tras `reset()` o tras una
pausa de más de 0.1 s usa el `LOOP_PERIOD_MS` nominal.
