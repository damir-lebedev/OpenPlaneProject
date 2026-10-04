# AUTOPILOT — modos, navegação, chaves

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/autopilot.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão.

[← Referência](README.md)

O piloto automático recebe os sticks do piloto, as chaves e potenciômetros
(`PilotInputs`) e os sensores, e entrega **o comando final das superfícies**
(`getCommand()`) e o acelerador do modo (`applyThrottle()`). Sem um sensor
necessário, o modo se comporta com segurança (as superfícies ficam com o piloto
ou no neutro) em vez de falhar. O que cada modo faz para o piloto está no
[AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md). Em voo, até agora só foi usado o modo manual; o piloto
automático foi testado na bancada, com testes e com simulações em malha fechada
(`test/native/test_sim`).

---

## `AutopilotMode`, `Feature`, `Knob`

**Arquivo:** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t` (sem escopo — os códigos numéricos vão para o
JSON de `/api/setmode`, `/api/status` e para o `MavlinkModes`):

| Valor | Código | Curto (OLED) | Essência |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | superfícies = sticks |
| `MODE_STABILIZE` | 1 | STAB | o stick é o ângulo de rolagem/arfagem |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | um programa de decolagem conduzido pelo acelerador do piloto |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + altitude com o profundor |
| `MODE_ACRO` | 4 | ACRO | o stick é a velocidade angular |
| `MODE_CRUISE` | 5 | CRZ | rumo + altitude + acelerador automático |
| `MODE_LOITER` | 6 | LOIT | círculos sobre o ponto em que foi ligado |
| `MODE_RTH` | 7 | RTH | para casa, círculos sobre o ponto de origem |
| `MODE_LAUNCH` | 8 | LNCH | lançamento manual |
| `MODE_AUTO_LAND` | 9 | LAND | planeio + arredondamento |
| `MODE_SOARING` | 10 | SOAR | térmicas sem motor |
| `MODE_RESCUE` | 11 | RESQ | asas niveladas, nariz para cima, acelerador |
| `MODE_COUNT` | 12 | | o limite (`setMode()` ignora ≥) |

`enum class Feature : uint8_t` — as funções das chaves: `FLAPS`, `AIRBRAKE`,
`AUTO_TRIM`, `TURN_COORDINATION`, `MOTOR_KILL`, `BEEPER`, `PAYLOAD_DROP`,
`GEOFENCE`, `HOME_RESET`, `CAMERA_STAB`, `COUNT`.

`enum class Knob : uint8_t` — os potenciômetros: `STAB_GAIN`, `MAX_BANK`,
`CRUISE_SPEED`, `FLAPS`, `CAMERA_TILT`, `RATES`, `LOITER_RADIUS`, `COUNT`.

`namespace AutopilotNames` — `mode()`, `modeShort()` (≤ 5 caracteres),
`feature()`, `knob()`: nomes para o log, o OLED, o dashboard e o MAVLink.

### `PilotInputs`

O estado das chaves e dos potenciômetros em um ciclo.

| Membro | Descrição |
|---|---|
| `bool has(Feature) const` | a função está ligada |
| `float knob(Knob) const` | a posição do potenciômetro −1…+1 |
| `bool isBound(Knob) const` | o potenciômetro está na tabela de atribuições |
| `float knobValue(Knob, min, default, max) const` | em unidades: o centro é `default`, as pontas são `min`/`max`; não atribuído — `default` |

---

## `Binding`, `Bind`, `BindingCheck`

**Arquivo:** `autopilot/ControlBinding.h` · a tabela — `config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`,
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`. As linhas da tabela são as
fábricas de `namespace Bind` (todas `constexpr`):

| Fábrica | Significado |
|---|---|
| `modes(ch, up, middle, down)`, `modes(ch, up, down)` | uma chave de seleção de modo (a zona por `PilotSwitches::zoneOf`) |
| `mode(ch, m)` | um modo por cima enquanto o canal ≥ `SWITCH_ON_US` |
| `feature(ch, f)` | uma função enquanto o canal ≥ `SWITCH_ON_US` |
| `knob(ch, k)` | um potenciômetro, `(us − 1500) / 500`, limitado a ±1 |

`namespace BindingCheck` — funções `constexpr` recursivas (o núcleo da ESP32 é
compilado como C++11): `channelIsFree`, `channelsFree`, `channelsUnique`,
`modeSwitchCount`, `atMostOneModeSwitch`. São usadas nos `static_assert` do
`Controls.h`.

---

## `PilotSwitches`

**Arquivo:** `autopilot/PilotSwitches.h` · **Depende de:** `Autopilot*`, `RcChannelState`, a tabela de atribuições

| Método | Descrição |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | uma tabela própria (testes, simulações) |
| `explicit PilotSwitches(Autopilot* = nullptr)` | a tabela `Controls::BINDINGS` |
| `void update(const RcChannelState&)` | reunir `PilotInputs` e passá-los a `autopilot->setInputs()`; `setMode()` — **somente quando o resultado das chaves mudou** (um modo definido pelo dashboard/estação de solo não é sobrescrito a cada ciclo). O `FlightController` só o chama com o link vivo |
| `void printBindings() const` | a disposição no Serial ao ligar: `SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (up / middle / down)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`, `"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2 zonas: < 1500 / ≥ 1500; 3 zonas: < 1250 / < 1750 / ≥ 1750 |
| `getInputs()`, `binding(i)` | para a telemetria e os testes |

`Bind::mode` tem precedência sobre `Bind::modes`; de vários `Bind::mode`
ligados, vence a linha de cima.

---

## `Autopilot`

**Arquivo:** `autopilot/Autopilot.h` · **Depende de:** `PidController`, `Navigation`, `AltitudeSpeedController`, `LaunchController`, `SoaringController`, `AutoTrim`, os sensores (todos anuláveis)

### Ciclo de vida

| Método | Descrição |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | O PID de rolagem/arfagem com Kp 5, Ki 0.5, Kd 0.5, saída ±500 µs |
| `bool begin()` | carregar o compensador; `false` e uma mensagem se não houver IMU ou barômetro |
| `void setInputs(const PilotInputs&)` | as chaves e os potenciômetros deste ciclo (antes de `update`) |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | uma vez por ciclo: sensores (sempre) → navegação e ponto de origem → gravação do compensador no solo → failsafe → geofence → modo → coordenação da curva → auto-trim |
| `ControlCommand getCommand() const` | os comandos finais das superfícies (roll/pitch/yaw, µs) |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | o acelerador do modo: `PILOT` — o do piloto; `AUTO` — o próprio; `AT_LEAST` — não menor que o próprio (decolagem automática). O ARM e o `MOTOR_KILL` ficam por conta do `FlightController` |

### Modos

| Método | Descrição |
|---|---|
| `void setMode(AutopilotMode)` | o mesmo ou ≥ `MODE_COUNT` — nada; caso contrário reinicia o PID e as máquinas de estados, os alvos = o rumo e a altitude atuais, o centro dos círculos = o ponto atual (com GPS), RTH — a altitude de retorno |
| `getMode()`, `getModeName()` | o nome: `FAILSAFE_GLIDE` / `FAILSAFE_RTH` na perda de link, senão o modo |
| `isFailsafeActive()`, `isFailsafeGliding()`, `isFailsafeReturning()` | o failsafe por cima do modo |
| `isAutoThrottle()`, `getThrottleCorrection()` | o acelerador do modo (%, para o log e o dashboard) |
| `getLaunchState()`, `getSoaringState()` | as máquinas de estados de LAUNCH e SOARING |

### Saídas e diagnóstico

| Método | Descrição |
|---|---|
| `getRollCorrection()`, `getPitchCorrection()`, `getYawCorrection()` | comando − sticks, µs |
| `getDesiredRoll()`, `getDesiredPitch()`, `getTargetAltitude()` | os alvos |
| `const NavStatus& getNavStatus()` | GPS, ponto de origem, posição, distância/rumo até a origem, curso e curso-alvo, velocidade para a navegação, geofence, estol |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | a altitude barométrica (m a partir do ponto de partida) |
| `getInputs()`, `getAutoTrim()` | para a telemetria |
| `getImuSensor()` … `getAirspeedSensor()` | os sensores (podem ser `nullptr`) |
| `getRollPid()`, `getPitchPid()`, `setPIDGains(...)` | o PID (dashboard, parâmetros do MAVLink) |

### Mecânica interna

- `stabilize()` — um PID de ângulo com D vindo do giroscópio, multiplicado por
  `Knob::STAB_GAIN`; o integrador só acumula com ARM e com erro
  < `STAB_INTEGRATOR_ZONE_DEG`. `stabilizeOrManual()` — sem IMU as superfícies
  ficam com o piloto; `stabilizeOrNeutral()` — sem IMU, neutro (modos
  automáticos).
- `imuReady()` = a IMU existe, está disponível e sem problema na verificação
  pré-voo.
- A velocidade para a navegação: tubo de Pitot → GPS → `NAV_ASSUMED_SPEED_MS`.
- `looksLanded()` — perto do solo pelo barômetro, quase sem velocidade vertical,
  mais lento que o limiar do tubo de Pitot/GPS: só então o compensador é
  gravado na flash.
- Failsafe: com GPS e ponto de origem — RTH com o motor, senão planeio; um RTH
  já iniciado não é abandonado por uma perda curta do GPS.

---

## `Geo`, `Guidance`, `GeoPoint`

**Arquivo:** `autopilot/Navigation.h`

Um plano local “norte/leste” em metros (uma projeção equirretangular — para
quilômetros o erro é uma fração de um por cento).

| Função | Descrição |
|---|---|
| `Geo::wrap180`, `Geo::wrap360` | normalização de ângulos |
| `Geo::offsetNE(a, b, north, east)`, `distance(a, b)`, `bearing(a, b)` | deslocamento, distância, rumo 0..360 |
| `Geo::moved(a, north, east)` | um ponto com deslocamento |
| `Geo::fromGps(GpsData)` | um `GeoPoint` a partir do GPS |
| `Guidance::rollForCourse(target, course, bankLimit)` | a inclinação para um erro de curso (`NAV_COURSE_GAIN`), limitada |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | o curso do campo vetorial para uma circunferência (`LOITER_CONVERGENCE`) |
| `Guidance::orbitBankDeg(speed, radius)` | a inclinação antecipada de um círculo: atan(V²/(g·R)) |

## `AltitudeSpeedController`

**Arquivo:** `autopilot/AltitudeSpeedController.h` — TECS-lite.

| Método | Descrição |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | a velocidade vertical desejada = `NAV_ALT_GAIN`·erro (≤ `NAV_MAX_CLIMB/SINK`); arfagem = a antecipação asin(Vz/V) + um PI sobre o erro de Vz, dentro de `NAV_MAX_CLIMB/DIVE_PITCH_DEG` |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | com tubo de Pitot — um PI sobre a velocidade do ar em torno de `cruisePct`; sem ele — `cruisePct`; + `THROTTLE_PER_CLIMB_PCT` pela subida requerida |
| `reset()`, `getWantedClimb()` | |

## `LaunchController`

**Arquivo:** `autopilot/LaunchController.h`

`State`: `IDLE → READY` (acelerador levantado) `→ THROWN` (uma sobrecarga > `LAUNCH_ACCEL_G`
por mais de `LAUNCH_ACCEL_TIME_MS`) `→ CLIMB` (após `LAUNCH_MOTOR_DELAY_MS`: o
motor, arfagem `LAUNCH_CLIMB_PITCH_DEG`) `→ DONE` (`LAUNCH_CLIMB_MS` ou
`LAUNCH_ALTITUDE_M`). Mexer nos sticks antes do arremesso — cancelar. Métodos:
`update(...)`, `reset()`, `getState()`, `motorOn()`, `pitchTargetDeg()`,
`stateName()`.

## `SoaringController`

**Arquivo:** `autopilot/SoaringController.h`

`State`: `GLIDE ⇄ THERMAL` (o variômetro > `SOAR_THERMAL_CLIMB_MS` por mais de
`SOAR_THERMAL_CONFIRM_MS` / a média < `SOAR_EXIT_CLIMB_MS` em
`SOAR_EXIT_WINDOW_MS`), `→ MOTOR_CLIMB` (abaixo de `SOAR_MIN_ALTITUDE_M`, até
`SOAR_MAX_ALTITUDE_M`), `→ RETURN` (além de `SOAR_MAX_DISTANCE_M`, até 70 % dela).
Métodos: `update(climb, alt, distHome, dt, now)`, `reset(now)`, `getState()`,
`motorOn()`, `getAverageClimb()`, `stateName()`.

## `AutoTrim`

**Arquivo:** `autopilot/AutoTrim.h` · armazenamento — `Preferences` (NVS / flash da STM32), espaço `"autotrim"`

| Método | Descrição |
|---|---|
| `void load()` | o compensador do NVS (se não há — 0) |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += comando · `AUTOTRIM_RATE` · dt, até ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | gravar se mudou (chamado pelo `Autopilot` após o DISARM no solo) |
| `reset()`, `getRoll()`, `getPitch()` | |

---

## `PidController`

**Arquivo:** `autopilot/PidController.h` · **Depende de:** `Config` (o `dt` nominal)

| Método | Descrição |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`, `getKp/Ki/Kd()` | os ganhos |
| `setLimits(minOut, maxOut)` | o limite da saída (±500 por padrão) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | a saída, limitada a `[min, max]` |
| `void reset()` | zerar o integrador, o `dt` conta a partir de “agora” |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (a taxa vinda do sensor!)
out = constrain(P + I + D, min, max)
```

O D é tomado sobre a taxa da grandeza medida (giroscópio, °/s) e não sobre a
derivada do erro: sem ruído de derivação e sem salto quando o setpoint muda. O
`dt` vem de `micros()`; a primeira chamada após `reset()` ou após uma pausa de
mais de 0,1 s usa o `LOOP_PERIOD_MS` nominal.
