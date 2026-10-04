# AUTOPILOT / feedback — a malha de realimentação (base preparada)

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/feedback.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão.

[← Referência](README.md)

> ⚠️ **Base preparada, não conectada ao firmware.** Nem o `FlightController`,
> nem o `Autopilot`, nem o `main.cpp` incluem estes cabeçalhos. Eles são
> verificados por uma simulação em malha fechada (`test/test_feedback`, no PC e
> na placa) e por testes unitários nativos. O plano de conexão está no
> [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#plano-de-conexão).

A ideia: em vez de um PID sobre o ângulo com ganhos ajustados para uma única
velocidade, um regulador fechado sobre **a resposta da aeronave**, com um modelo
do eixo aprendido em voo, proteção contra estol e fases de decolagem e pouso
conduzidas pelos sensores. A única entrada é o `FlightSnapshot`; a única saída,
o `FeedbackOutput`.

Todos os módulos são somente de cabeçalho; o `FeedbackModules.h` inclui todos
eles em uma linha.

---

## namespace `FeedbackConfig`

**Arquivo:** `autopilot/feedback/FeedbackConfig.h`

Todas as constantes da malha (ao conectar, irão para o `Config.h`). Os valores
marcados como “прикидка” (“estimativa grosseira”) são para um modelo de ~1 kg e envergadura de 1,2 m. Os
arrays `[AXIS_COUNT]` são indexados pelo eixo.

| Grupo | Constantes |
|---|---|
| Geral | `GRAVITY = 9.80665`; os eixos `AXIS_ROLL = 0`, `AXIS_PITCH = 1`, `AXIS_YAW = 2`, `AXIS_COUNT = 3` |
| Velocidade | `STALL_SPEED_MS = 8`, `REFERENCE_SPEED_MS = 14`, `ACCEL_FILTER_TAU_S = 0.3` |
| No ar/no solo | `AIRBORNE_HEIGHT_M = 3`, `AIRBORNE_CONFIRM_MS = 500`, `GROUND_STILL_MS = 2000`, `GROUND_ACCEL_TOLERANCE_G = 0.1` |
| Regulador | `ANGLE_GAIN = {4, 4, 2}` 1/s, `MAX_RATE_DPS = {120, 60, 30}`, `RATE_TAU_S = {0.15, 0.20, 0.30}`, `RATE_INTEGRAL_GAIN = {2, 2, 1}`, `MAX_DEFLECTION_US = {400, 400, 400}`, `DAMPING_COMPENSATION = 0.5` |
| Efetividade das superfícies | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs, `EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`, `EFFECTIVENESS_MAX = {30, 15, 6}`, `RESPONSE_DELAY_MS = 40`, `RLS_FORGETTING = 0.995`, `ESTIMATOR_PERIOD_MS = 20`, `ESTIMATOR_PREFILTER_HZ = 2`, `MIN_EXCITATION_US = 30` |
| Estol | `DECEL_WARN_MS2 = 2`, `DECEL_CONFIRM_MS = 300`, `LOW_ENERGY_PITCH_DEG = 5`, `NOSE_DROP_RATE_DPS = 60`, `WING_DROP_RATE_DPS = 120`, `STALL_NOSE_UP_COMMAND_US = 50`, `LOW_EFFECTIVENESS_RATIO = 0.35`, `LOW_SPEED_MARGIN = 1.25`, `LOW_SPEED_EXIT_MARGIN = 1.5`, `LOW_ENERGY_THROTTLE_PERCENT = 80`, `LOW_ENERGY_MAX_PITCH_DEG = 5`, `STALL_THROTTLE_PERCENT = 100`, `STALL_MAX_PITCH_DEG = −5`, `STALL_MAX_BANK_DEG = 10`, `STALL_AILERON_LIMIT_US = 150`, `RECOVERY_HOLD_MS = 1000` |
| Decolagem | `TAKEOFF_HAND_LAUNCH = false`, `TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`, `TAKEOFF_THROTTLE_PERCENT = 100`, `LAUNCH_ACCEL_G = 1`, `LAUNCH_DETECT_MS = 50`, `ROTATE_SPEED_MS = 10`, `ROTATE_FALLBACK_MS = 1500`, `CLIMB_PITCH_DEG = 12`, `TAKEOFF_TARGET_ALTITUDE_M = 30`, `TAKEOFF_CLIMB_FALLBACK_MS = 10000`, `LAUNCH_TIMEOUT_MS = 8000`, `HEADING_HOLD_GAIN = 2` |
| Pouso | `APPROACH_SINK_RATE_MS = 1`, `APPROACH_THROTTLE_PERCENT = 25`, `APPROACH_BASE_PITCH_DEG = −3`, `APPROACH_MIN_PITCH_DEG = −10`, `APPROACH_MAX_BANK_DEG = 20`, `GO_AROUND_THROTTLE_PERCENT = 80`, `SINK_TO_PITCH_GAIN = 4`, `FLARE_HEIGHT_M = 2`, `FLARE_SINK_RATE_MS = 0.3`, `FLARE_MAX_PITCH_DEG = 8`, `TOUCHDOWN_ACCEL_G = 0.5`, `TOUCHDOWN_HEIGHT_M = 0.3`, `TOUCHDOWN_STILL_MS = 500`, `TOUCHDOWN_STILL_RATE_DPS = 5`, `ROLLOUT_MS = 5000` |

---

## namespace `FeedbackMath`

**Arquivo:** `autopilot/feedback/FeedbackMath.h` · **Depende de:** `<math.h>`

| Função | Descrição |
|---|---|
| `float wrap180(float deg)` | Um ângulo em `(−180, 180]`: a diferença entre os rumos 350° e 10° é −20° |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | Limita a `[−limit, limit]` |

---

## `FlightSnapshot`

**Arquivo:** `autopilot/feedback/FlightSnapshot.h` · **Tipo:** struct

Tudo o que a malha sabe sobre a aeronave em um ciclo. Os sinais são
aeronáuticos.

| Grupo | Campos |
|---|---|
| Tempo/status | `timeUs`, `armed`, `linkLost` |
| Atitude | `imuValid`, `rollDeg`, `pitchDeg`, `yawDeg`, `rollRateDps`, `pitchRateDps`, `yawRateDps`, `accelXg/Yg/Zg` |
| Altitude | `baroValid`, `altitudeM` (a partir do ponto de partida), `climbRateMs`, `heightAglValid`, `heightAglM` (um futuro telêmetro) |
| Velocidade | `airspeedValid`, `airspeedMs` (um futuro tubo de Pitot), `gpsValid`, `groundSpeedMs` |
| Alvos do modo | `stabilizationActive` (false = MANUAL: só aprendizado), `targetRollDeg`, `targetPitchDeg` |
| Comandos, µs | `stick*Us` — a contribuição do piloto; `command*Us` — o resultado que realmente foi para as superfícies |
| Acelerador, % | `pilotThrottlePercent`, `throttlePercent` (realmente para o ESC) |
| Flaps | `flapsUs`, `flapsMoving` |

---

## `FeedbackOutput`

**Arquivo:** `autopilot/feedback/FeedbackOutput.h` · **Tipo:** struct

| Campo | Descrição |
|---|---|
| `float deflectionUs[3]` | Deflexões das superfícies por eixo, µs (os sinais do `ControlCommand`) |
| `bool axisEnabled[3]` | `false` — o eixo não é controlado, a superfície fica com o piloto |
| `float throttleOverridePercent` | O acelerador absoluto de uma fase de voo; `< 0` — não definido |
| `float throttleFloorPercent` | O limite inferior do acelerador (proteção contra estol); `< 0` — nenhum |
| `targetRollDeg`, `targetPitchDeg` | Os alvos finais depois das limitações (depuração) |
| `const char* reason` | Uma descrição curta para o log/OLED |

---

## `PhaseTargets`

**Arquivo:** `autopilot/feedback/PhaseTargets.h` · **Tipo:** struct

A saída comum do `TakeoffSequencer` e do `LandingSequencer` — o “quê”, e não o
“como”.

| Campo | Padrão | Descrição |
|---|---|---|
| `active` | `false` | A fase está controlando a aeronave agora |
| `targetRollDeg`, `targetPitchDeg` | 0 | Os alvos |
| `controlRoll`, `controlPitch` | `true` | `false` — não mexer no eixo (sobre rodas a arfagem é definida pelo trem de pouso) |
| `holdHeading`, `headingDeg` | `false`, 0 | Manter o rumo com o leme e a roda |
| `throttlePercent` | −1 | −1 — o acelerador do piloto |
| `reason` | `""` | Uma descrição |

---

## `SpeedEstimator`

**Arquivo:** `autopilot/feedback/SpeedEstimator.h`

A velocidade (do ar > velocidade sobre o solo do GPS > desconhecida) e a
aceleração longitudinal pela IMU: `dV/dt = g · (ax − sin θ)` por um filtro
passa-baixa `ACCEL_FILTER_TAU_S` — dá para ver que “a velocidade está caindo”
mesmo sem sensor de velocidade.

| Método | Descrição |
|---|---|
| `void update(const FlightSnapshot&)` | Um passo; `dt ≤ 0` ou `> 0.5 s` desde a chamada anterior (para a primeira — desde `timeUs = 0`) — é ignorado |
| `bool hasSpeed() const`, `float getSpeed() const`, `Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`, `float getAcceleration() const` | m/s², “+” — acelerando; sem IMU — `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`, limitado a `0.05..4`; sem velocidade — 1 |

---

## `AirborneDetector`

**Arquivo:** `autopilot/feedback/AirborneDetector.h`

Se a aeronave está no ar: aprender, acumular a integral e procurar o estol só
faz sentido em voo.

| Método | Descrição |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | Não armado — reinicia para “no solo”. Um candidato a mudança de estado deve se manter por `AIRBORNE_CONFIRM_MS` (decolagem) ou `GROUND_STILL_MS` (pouso) |
| `void force(bool)` | Definir explicitamente (a decolagem e o pouso sabem) |
| `void reset()` | No solo |
| `bool isAirborne() const` | |

“Parece voo”: a altura pelo telêmetro ou barômetro > `AIRBORNE_HEIGHT_M`, ou a
velocidade > `ROTATE_SPEED_MS`. “Parece solo”: baixo, as velocidades angulares
de todos os eixos < `TOUCHDOWN_STILL_RATE_DPS`, |a| ≈ 1g (± `GROUND_ACCEL_TOLERANCE_G`).

---

## `ControlEffectivenessEstimator`

**Arquivo:** `autopilot/feedback/ControlEffectivenessEstimator.h`

Um eixo. O modelo: **aceleração angular = b·superfície(t − atraso) + a·ω + c**.
`b` é a efetividade da superfície (°/s² por µs, o sinal é o sentido da
resposta), `a` é o amortecimento (1/s, normalmente < 0), `c` é um momento
constante (auto-trim). O `b` é aprendido em uma velocidade de referência:
`b = b_ref · (V/V_ref)²`, `a = a_ref · V/V_ref`. A estimativa é de mínimos
quadrados recursivos com esquecimento (`λ = 0.995`, memória de ~4 s).

| Método | Descrição |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | Chama `reset()` |
| `void reset()` | θ = (prior, 0, 0); covariância: b ± prior, a ± 5, c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | Chamar a cada ciclo: acumula médias em um intervalo de `ESTIMATOR_PERIOD_MS`; no fim do intervalo — a aceleração pela diferença do giroscópio, o atraso do comando, um filtro passa-baixa comum aos dois lados e um passo do RLS (se o aprendizado é permitido e há excitação) |
| `getEffectiveness()` | `b` na velocidade atual |
| `getReferenceEffectiveness()` | `b` na velocidade de referência |
| `getDamping()` | `a` na velocidade atual |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b` (0 se `\|b\|` é pequeno) |
| `getEffectivenessSigma()` | σ da estimativa de `b` na velocidade atual |
| `bool isConfident() const` | ≥ 50 passos do RLS, `\|b\|` ≥ o mínimo e σ < 0.3·`\|b\|` |
| `getAngularAccel()` | A aceleração do último intervalo (depuração) |

Particularidades:

- Um intervalo maior que `MAX_GAP_MS = 200` (o laço parou) — os dados começam
  de novo (o histórico do atraso e o filtro são reiniciados).
- Só aprende sob **excitação**: a amplitude dos comandos médios em 16
  intervalos (~0,3 s) ≥ `MIN_EXCITATION_US`; caso contrário a estimativa
  congela.
- Higiene dos float após um passo: a simetria de `P`, um teto para as variâncias
  (×10 das iniciais), um limite para `b` (`±EFFECTIVENESS_MAX`) e para `a`
  (`−40..5`).

---

## `AxisModel`

**Arquivo:** `autopilot/feedback/AdaptiveRateController.h` · **Tipo:** struct

O que se sabe da resposta de um eixo, para o regulador: `effectiveness` (b,
padrão 1), `damping` (a, 0 — não compensar), `bias` (c, 0).

---

## `AdaptiveRateController`

**Arquivo:** `autopilot/feedback/AdaptiveRateController.h`

Um regulador de um eixo, três estágios:

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| Método | Descrição |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | A integral, a saturação e a saída — 0 |
| `float angleToRate(targetDeg, angleDeg) const` | Estágio 1 (o caminho mais curto para o rumo) |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | Estágios 2–3, devolve a deflexão, µs |
| `getDesiredRate()`, `getIntegral()`, `getOutput()`, `isSaturated()` | Estado |

Invariantes: `|b|` não é menor que `EFFECTIVENESS_MIN` (mantendo o sinal de b);
a integral é guardada em °/s (continua correta quando `b` muda) e **não se
acumula na direção do limite** (anti-windup pela direção da saturação do passo
anterior); com `dt ≤ 0` a integral não muda.

---

## `StallGuard`

**Arquivo:** `autopilot/feedback/StallGuard.h`

Proteção contra perda de velocidade e estol. Os níveis são `Level::{Normal, LowEnergy, Stall}`.

| Método | Descrição |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | No solo ou sem IMU — reinicia para Normal |
| `Level getLevel() const`, `const char* getLevelName() const` | `"OK"`, `"LOW_ENERGY"`, `"STALL"` |
| `const char* getReason() const` | O último indício que disparou |
| `float maxPitchDeg() const` | Stall: −5°, LowEnergy: 5°, senão 90° |
| `float maxBankDeg() const` | Stall: 10°, senão 180° |
| `float maxAileronUs() const` | Stall: 150 µs, senão `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall 100 %, LowEnergy 80 %, senão/sem link −1 |
| `void reset()` | Normal |

`StallGuard::ControlState` — `pitchEffectivenessKnown`, `pitchEffectiveness`
(o módulo da estimativa de b em arfagem).

Indícios de **LowEnergy**: uma desaceleração confirmada (`DECEL_CONFIRM_MS`)
acima de `DECEL_WARN_MS2` com arfagem > 5°; velocidade < `1.25·Vs`; uma
efetividade confiável do profundor < 35 % da a priori. Indícios de **Stall**:
velocidade < Vs; o nariz cai mais rápido que 60 °/s com o profundor “para cima” acima de
50 µs; com pouca energia a asa cai mais rápido que 120 °/s contra os
ailerons. As medidas são retiradas depois de `RECOVERY_HOLD_MS` e somente
quando a energia se recuperou (velocidade ≥ `1.5·Vs`, sem sensor de
velocidade — aceleração ≥ 0).

---

## `TakeoffSequencer`

**Arquivo:** `autopilot/feedback/TakeoffSequencer.h`

Decolagem em fases. `State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`.
O diagrama está em [ARCHITECTURE.md §7](../ARCHITECTURE.md#decolagem-e-pouso-malha-de-realimentação-não-conectada).

| Método | Descrição |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | Uma fase ativa → `Aborted`; os alvos são zerados |
| `void update(snapshot, speed, nowMs)` | No máximo uma transição por ciclo, depois os alvos da nova fase |
| `void reset()` | → `Idle` |
| `getTargets()`, `getState()`, `getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

Os alvos das fases: espera — acelerador 0, as superfícies com o piloto; corrida —
acelerador 100 %, asas niveladas, sem mexer na arfagem, manter o rumo fixado no
momento da partida; subida — acelerador 100 %, asas niveladas, arfagem
`CLIMB_PITCH_DEG`. Arremesso à mão: a aceleração longitudinal
`ax − sin θ ≥ LAUNCH_ACCEL_G` por mais de `LAUNCH_DETECT_MS`.

---

## `LandingSequencer`

**Arquivo:** `autopilot/feedback/LandingSequencer.h`

Pouso em fases. `State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`.

| Método | Descrição |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`, `void reset()`, `update(snapshot, nowMs)` | Como na decolagem |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout — a proteção contra estol é desligada aqui |
| `bool isOnGround() const` | Rollout / Complete |

A arfagem na descida e no arredondamento vem do erro da velocidade vertical:
`θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`, limitada a
`[min, FLARE_MAX_PITCH_DEG]`; sem barômetro — o ângulo base. A altura vem do
telêmetro, senão do barômetro. O toque: um pico de |a − 1g| ≥ 0.5g ou “baixo e
sem girar” por `TOUCHDOWN_STILL_MS`. Na corrida de pouso o rumo é fixado no
momento do toque.

---

## `FeedbackSupervisor`

**Arquivo:** `autopilot/feedback/FeedbackSupervisor.h`

A malha inteira. É dono do `SpeedEstimator`, do `AirborneDetector`, de três
`ControlEffectivenessEstimator`, de três `AdaptiveRateController`, do `StallGuard`,
do `TakeoffSequencer` e do `LandingSequencer`.

| Método | Descrição |
|---|---|
| `bool requestTakeoff()` | Só armado, com link, no solo; cancela um pouso |
| `bool requestLanding()` | Só armado, com link, no ar; cancela uma decolagem |
| `void cancelPhase()` | Cancelar a fase |
| `const FeedbackOutput& update(const FlightSnapshot&)` | Um ciclo (a ordem está no [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-a-malha-de-realimentação-não-conectada)) |
| `getOutput()`, `isAirborne()`, `getSpeedEstimator()`, `getEstimator(axis)`, `getController(axis)`, `getStallGuard()`, `getTakeoff()`, `getLanding()` | Estado para o log e os testes |
| `void printStatus(Print& out) const` | Uma linha de status + uma linha por eixo (`b ± σ`, `*` — confiável, `a`, `c`, `I`, a saída) |

Regras principais:

- **Não armado** — todos os eixos desligados, `reason = "não armado"`; o
  ARM/DISARM (um voo novo) apaga tudo o que foi aprendido.
- **A perda de link** cancela as fases; o acelerador não é tocado (o failsafe do
  firmware é que atua).
- **A decolagem do solo** reinicia as estimativas e os reguladores (o que foi
  “visto” sobre as rodas não serve).
- Uma estimativa confiável e negativa de `b` **nunca** vai para o regulador — o
  eixo trabalha com o modelo a priori, e no `reason` aparece o aviso “… responde
  à superfície ao contrário? verificar no solo”.
- A integral fica congelada no solo, exceto o rumo na corrida de decolagem e de
  pouso.
- Curva coordenada (com velocidade conhecida no ar): à taxa de arfagem desejada
  soma-se `+ g/V · sin φ · tg φ`, e à de guinada, `g/V · sin φ` (a inclinação é
  limitada a ±60°).
- A prioridade de `reason`: estol > pouca energia > fase > aviso de sinal >
  “estabilização”/“manual (aprendizado)”.
