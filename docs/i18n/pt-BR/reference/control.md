# CONTROL e COORDINATION — o mixer, o acelerador, o ARM, as saídas, o orquestrador

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/control.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão.

[← Referência](README.md)

A camada CONTROL é lógica sobre dados, sem UART, PWM ou Wi-Fi.
`FlightController` (COORDINATION) é a única classe que reúne todas as camadas
inferiores em um único ciclo.

---

## `ControlCommand`

**Arquivo:** `control/ControlCommand.h` · **Tipo:** struct

Um comando para as superfícies em **sinais físicos**, µs de deflexão (±500 =
curso total). A linguagem comum dos sticks, do piloto automático e do mixer.

| Campo | “+” significa |
|---|---|
| `int16_t roll` | rolagem para a direita (aileron direito para cima, esquerdo para baixo) |
| `int16_t pitch` | nariz para cima (profundor para cima) |
| `int16_t yaw` | nariz para a direita (leme e roda para a direita) |
| `int16_t flaps` | flaps para baixo (os dois ailerons para baixo); “−” — freio aerodinâmico (os dois para cima) |

Todos os campos são 0 por padrão.

---

## `FlightOutputState`

**Arquivo:** `control/FlightOutputState.h` · **Tipo:** struct

Os pulsos desejados das saídas, µs de PWM. Por padrão — superfícies no neutro e
acelerador em `PWM_MIN`.

| Campo | Padrão |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `aux1` | `PAYLOAD_CLOSED_US` — o lançador de carga está fechado |
| `aux2` | `PWM_CENTER` — a câmera |

---

## `FlapsController`

**Arquivo:** `control/FlapsController.h` · **Depende de:** `Config`

Extensão e recolhimento suaves dos flaps: a posição vai em direção ao alvo
(qualquer valor — flaps da chave, do potenciômetro, um freio aerodinâmico para
cima) sem ser mais rápida que o curso total `FLAPS_DEPLOYED_US` em
`FLAPS_TRANSITION_MS`. O tempo é passado como parâmetro.

| Método | Descrição |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | Um passo em direção ao alvo; devolve a posição atual, µs (+ para baixo, − para cima) |
| `int16_t getPosition() const` | A posição atual |

Invariantes:

- **A primeira chamada** coloca a posição direto no alvo — os flaps não
  “saem” sobre a mesa ao ligar.
- O passo de tempo é limitado a `MAX_STEP_MS = 20`: depois de uma pausa longa
  (failsafe, calibração) os flaps não pulam para o alvo em um único ciclo.

---

## `ControlMixer`

**Arquivo:** `control/ControlMixer.h` · **Depende de:** `RcInput`, `RcChannelState`, `FlapsController`, `ControlCommand`, `FlightOutputState`, `Config`, `Channels`

A lógica aerodinâmica em dois passos. É dono do `FlapsController`.

| Método | Descrição |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll` (2000 = para a direita); CH2 → `pitch` **com o sinal oposto** (2000 = para longe de você = nariz para baixo); CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | o alvo dos flaps é escolhido pelo `FlightController` (freio aerodinâmico → chave dos flaps → potenciômetro), aqui está o curso suave |
| `FlightOutputState mix(const ControlCommand& c) const` | Comando → PWM. Rolagem/arfagem/guinada são limitadas pelo curso (`*_MAX_US`); ailerons: esquerdo = `flaps + roll`, direito = `flaps − roll` (para baixo = “+”); PWM = `1500 ± deflexão` com o sinal de `Config::*_REVERSED`, limitado a 1000..2000. O `throttle` não é preenchido |
| `int16_t getFlaps() const` | A posição atual dos flaps, µs |

Flaperons: ao estender, os dois ailerons descem `FLAPS_DEPLOYED_US` (o novo
“neutro”), e a rolagem atua por cima. Com rolagem total, o aileron que desce
chega ao fim do curso antes do que sobe — isso funciona como um diferencial de
ailerons.

---

## `ThrottleManager`

**Arquivo:** `control/ThrottleManager.h` · **Depende de:** `RcInput`, `RcChannelState`, `Config`, `Channels`

| Método | Descrição |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | O acelerador do CH3, limitado a 1000..2000; na perda de link — `FAILSAFE_THROTTLE` |

Não sabe nada sobre o ARM e o piloto automático — as correções deles são
aplicadas pelo `FlightController`.

---

## `ArmingManager`

**Arquivo:** `control/ArmingManager.h` · **Depende de:** `Autopilot` (anulável), `RcChannelState`, `Config`, `Channels`

O ARM por uma chave separada, SwA (CH5). A máquina de estados está em
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Método | Descrição |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Sem piloto automático só o acelerador é verificado |
| `void update(const RcChannelState& rc, bool receiverFailsafe)` | Em failsafe — nada (a chave em um quadro de failsafe não reflete o piloto). Chave OFF → DISARM, `switchSeenOff = true`. Uma transição OFF→ON → verificações → ARM ou recusa |
| `bool isArmed() const` | Armado |
| `const char* getLastRefusalReason() const` | O motivo da última recusa ou `nullptr`; é limpo quando a chave é desligada |

`checkFailureReason(rc)` — as verificações do ARM:

| Condição | Motivo da recusa |
|---|---|
| Acelerador ≥ `THROTTLE_LOW_US` | “acelerador fora do mínimo” |
| Qualquer modo exceto MANUAL, a IMU existe mas não responde | “a IMU não responde…” |
| Qualquer modo exceto MANUAL, a IMU tem um problema na verificação pré-voo | o texto de `ImuSensor::getPreflightProblem()` |
| Um modo com altitude (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), o barômetro existe mas não responde | “o barômetro não responde…” |

Um sensor ausente da compilação (`nullptr`) não bloqueia o ARM; em MANUAL a
aeronave arma até sem nenhum sensor. O fix do GPS não entra nas verificações de
propósito: sem GPS os modos de navegação se comportam com segurança (um círculo
no lugar), e o ponto de origem é registrado quando o GPS pegar os satélites.

Invariantes: ligar a placa com a chave em ON não arma; uma tentativa por
transição OFF→ON; a perda de link não desfaz o ARM.

---

## `FlightOutputs`

**Arquivo:** `control/FlightOutputs.h` · **Depende de:** `IBoard`, `FlightOutputState`, `Config`

A única classe que conhece o conjunto e a ordem das saídas PWM. Todas as saídas
são descritas em uma tabela; `begin()`, `write()`, o status e o autoteste a
percorrem em um laço.

### `FlightOutputs::OutputInfo`

| Campo | Descrição |
|---|---|
| `const char* key` | O nome no JSON/log (`aileronLeft`, …, `esc`, `rudder`, `aux1`, `aux2`) |
| `const char* label` | O nome para humanos |
| `int16_t pin` | O número do pino; `-1` — não ligado. `int16_t`, porque na STM32 os números dos pinos analógicos são `0xC0 + N` |
| `bool required` | Sem ela a aeronave não voa (o leme é opcional) |
| `uint16_t FlightOutputState::* field` | Um ponteiro para o campo do estado |

| Método | Descrição |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | Uma linha da tabela; a ordem = `ServoChannel` |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` de cada saída, imprime o status; `true` se todas as **obrigatórias** obtiveram um canal |
| `bool isAttached(uint8_t ch) const` | A saída está conectada (um índice fora da faixa → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | O valor da saída a partir do estado, pela tabela |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | O pulso medido contra o esperado em cada pino ligado; “OK” quando a diferença é ≤ 15 µs |
| `void write(const FlightOutputState&)` | Gravar todas as saídas e lembrar o estado |
| `void setFailsafe()` | Superfícies no neutro (`FAILSAFE_*`), acelerador `FAILSAFE_THROTTLE`; AUX como estavam (a carga não é solta na perda de link) |
| `void setBuzzer(bool on)` | o buzzer da placa (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | O último estado gravado |

Para adicionar uma saída: uma linha na tabela + um campo em
`FlightOutputState` + um índice em `ServoChannel` (+ um pino e um canal LEDC no
`Esp32Board`).

---

## `FlightController`

**Arquivo:** `control/FlightController.h` · **Camada:** COORDINATION ·
**Depende de:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`

O único coordenador do laço de controle: ele mesmo não analisa a UART, não mexe
no PWM nem calcula o mixer — só chama os outros na ordem certa. O diagrama
detalhado está em [ARCHITECTURE.md §6](../ARCHITECTURE.md#6-o-ciclo-de-controle-flightcontrollerupdate).

| Método | Descrição |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | Sem piloto automático — controle manual puro; sem chaves — só os sticks |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | Um ciclo (veja abaixo) |
| `bool isReceiverFailsafe() const` | O link foi perdido |
| `const IBusReceiver& getReceiver() const` | Para o log (contadores de quadros, a causa da perda) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | O último gravado nas saídas |
| `const RcChannelState& getRcState() const` | Os canais |
| `const FlightOutputs& getOutputs() const` | A tabela de saídas e `attached` |
| `int16_t getFlapsUs() const` | A posição dos flaps |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | As chaves e os potenciômetros deste ciclo |
| `bool isLostModelBeeping() const` | O buzzer do “estou aqui” está tocando |

A ordem do `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. com o link vivo — `switches->update(rc)` (o modo, as funções, os potenciômetros);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. os sticks `mixer.fromSticks(rc)` (com o link vivo) × `Knob::RATES`; os
   flaps `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → suavemente, na perda de link — 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **sempre**;
6. o buzzer: `Beeper::update(BEEPER, armed, failsafe, now)`;
7. link perdido → `applyLinkLoss()` e saída do ciclo;
8. `arming.update(rc, false)`;
9. `command = autopilot->getCommand()` (ou os sticks sem piloto automático), os flaps — os próprios;
10. `output = mixer.mix(command)`; `output.throttle = autopilot->applyThrottle(pilotThrottle)`;
11. não armado ou `MOTOR_KILL` → `throttle = PWM_MIN` (por último);
12. AUX1 — a carga (`PAYLOAD_DROP`), AUX2 — a câmera (`Knob::CAMERA_TILT`, `CAMERA_STAB` subtrai a arfagem);
13. `outputs.write(output)`.

`applyLinkLoss()`: se o piloto automático está em failsafe (armado: RTH ou
planeio) — as superfícies e o acelerador seguem o comando do piloto automático
(os flaps recolhem suavemente, o `MOTOR_KILL` continua silenciando o motor, AUX
como estavam); caso contrário `outputs.setFailsafe()`.

---

## `Beeper`

**Arquivo:** `control/Beeper.h` · **Depende de:** `Config`

| Método | Descrição |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | o estado do buzzer: 2 Hz se `Feature::BEEPER` ou “modelo perdido” (não armado, sem link por mais de `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | o modo “procure por mim na grama” |
