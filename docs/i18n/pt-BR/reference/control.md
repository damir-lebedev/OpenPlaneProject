# CONTROL e COORDINATION — o mixer, o acelerador, o ARM, as saídas, o orquestrador

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/control.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

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

`throttle` é a magnitude do acelerador (1000..2000 = 0..100%) em qualquer sentido; o `FlightOutputs` a converte, junto com `motorReverse`, no pulso do ESC (via `EscSignal`).

| Campo | Padrão |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `bool motorReverse` | `false` — o motor gira para a frente |
| `bool escPulsesOff` | `false`; `true` — nenhum pulso para o ESC (sinal do motor, só sem ARM) |
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
| `FlightOutputState mix(const ControlCommand& c) const` | Comando → PWM. Rolagem/arfagem/guinada são limitadas pelo curso (`*_MAX_US`); ailerons: esquerdo = `flaps + roll`, direito = `flaps − roll` (para baixo = “+”); PWM = `1500 ± deflexão` com o sinal de `Config::*_REVERSED`, o sinal dos flaps — `FLAPS_REVERSED`, limitado a 1000..2000. O `throttle` não é preenchido |
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

## `EscSignal`

**Arquivo:** `control/EscSignal.h` · **Tipo:** namespace · **Depende de:** `Config`

Acelerador (1000..2000 = 0..100%) e sentido → o pulso no pino do ESC.

| Função | Descrição |
|---|---|
| `uint16_t pulseUs(uint16_t throttleUs, bool reverse)` | `ESC_BIDIRECTIONAL` (BLHeli 80A em 3D): 0% → `ESC_NEUTRAL_US` (1500), para a frente 1500..2000, para trás 1500..1000. ESC unidirecional: 1000..2000 como está, `reverse` é ignorado. Acelerador fora de 1000..2000 é limitado |
| `uint16_t stopUs()` | O pulso de “motor parado” (1500 num ESC bidirecional, 1000 num comum) |

Só conversão, sem proteções: quando trocar o sentido quem decide é o `MotorReverse`; o limite de acelerador para trás é do `FlightController`.

---

## `MotorReverse`

**Arquivo:** `control/MotorReverse.h` · **Depende de:** `Config`

O sentido do motor pela chave `Feature::MOTOR_REVERSE` (por padrão a SwD) — **só sem ARM**.

| Método | Descrição |
|---|---|
| `void update(bool requested, bool armed, uint32_t nowMs)` | Sem ARM — o sentido segue a chave na hora (no Serial, `MotorReverse: мотор НАЗАД` / `вперёд` — “motor PARA TRÁS” / “para a frente”). Com ARM — a chave não atua, “реверс НЕ переключён” (“reversão NÃO trocada”) é impresso uma vez, e o novo sentido espera o DISARM. Com `ESC_BIDIRECTIONAL = false` a reversão nunca é ativada |
| `bool isReversed() const` | O motor gira para trás |
| `bool isPending() const` | A chave já está na outra posição; o sentido espera o DISARM |
| `const char* armBlockReason(uint32_t nowMs) const` | Motivo para recusar o ARM: passou menos de `REVERSE_SETTLE_MS` desde a última troca de sentido (o motor pode ainda estar girando); senão `nullptr` |

---

## `ArmingManager`

**Arquivo:** `control/ArmingManager.h` · **Depende de:** `Autopilot` (anulável), `RcChannelState`, `Config`, `Channels`

O ARM por uma chave separada, SwA (CH5). A máquina de estados está em
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Método | Descrição |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Sem piloto automático só o acelerador é verificado |
| `void update(const RcChannelState& rc, bool receiverFailsafe, const char* blockReason = nullptr)` | Em failsafe — nada (a chave em um quadro de failsafe não reflete o piloto). Chave OFF → DISARM, `switchSeenOff = true`. Uma transição OFF→ON → verificações → ARM ou recusa `blockReason` é um motivo de recusa externo (do `FlightController`: `MotorReverse::armBlockReason()`) |
| `bool isArmed() const` | Armado |
| `const char* getLastRefusalReason() const` | O motivo da última recusa ou `nullptr`; é limpo quando a chave é desligada |

`checkFailureReason(rc)` — as verificações do ARM:

| Condição | Motivo da recusa |
|---|---|
| Acelerador ≥ `THROTTLE_LOW_US` | “acelerador fora do mínimo” |
| Qualquer modo exceto MANUAL, a IMU existe mas não responde | “a IMU não responde…” |
| Qualquer modo exceto MANUAL, a IMU tem um problema na verificação pré-voo | o texto de `ImuSensor::getPreflightProblem()` |
| Um modo com altitude (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), o barômetro existe mas não responde | “o barômetro não responde…” |
| Todo o resto passou, mas foi passado `blockReason` | `blockReason` (por exemplo, “o sentido do motor acabou de mudar…”) |

Um sensor ausente da compilação (`nullptr`) não bloqueia o ARM; em MANUAL a
aeronave arma até sem nenhum sensor. O fix do GPS não entra nas verificações de
propósito: sem GPS os modos de navegação se comportam com segurança (um círculo
no lugar), e o ponto de origem é registrado quando o GPS pegar os satélites.

Invariantes: ligar a placa com a chave em ON não arma; uma tentativa por
transição OFF→ON; a perda de link não desfaz o ARM.

---

## `FlightOutputs`

**Arquivo:** `control/FlightOutputs.h` · **Depende de:** `IBoard`, `FlightOutputState`, `EscSignal`, `Config`

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
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` de cada saída, imprime o status e o tipo de ESC; `true` se todas as **obrigatórias** obtiveram canal |
| `bool isAttached(uint8_t ch) const` | A saída está conectada (um índice fora da faixa → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | O pulso no pino da saída: o campo pela tabela; no ESC — via `EscSignal::pulseUs(throttle, motorReverse)`, com `escPulsesOff` — 0 |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | O pulso medido contra o esperado em cada pino ligado; “OK” quando a diferença é ≤ 15 µs |
| `void write(const FlightOutputState&)` | Gravar todas as saídas e lembrar o estado; o ESC com `escPulsesOff` — `IServoOutput::stopPulses()` |
| `void setFailsafe()` | Superfícies de controle em neutro (`FAILSAFE_*`), acelerador `FAILSAFE_THROTTLE` (1500 no pino de um ESC bidirecional); AUX como estavam (a carga não é lançada na perda de link) |
| `void setBuzzer(bool on)` | o buzzer da placa (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | O último estado gravado |

Para adicionar uma saída: uma linha na tabela + um campo em
`FlightOutputState` + um índice em `ServoChannel` (+ um pino e um canal LEDC no
`Esp32Board`).

---

## `FlightController`

**Arquivo:** `control/FlightController.h` · **Camada:** COORDINATION ·
**Depende de:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`, `MotorReverse`, `GroundCalibration`, `SignalPlayer`

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
| `const MotorReverse& getMotorReverse() const` | O sentido do motor e a reversão pendente |
| `void toggleGroundCalibration()` | Console `c`: calibração em solo do avião — iniciar (só sem ARM), em andamento — cancelar |
| `void playSignal(uint8_t count)` | Console `e`: um sinal (sem ARM — também pelo motor) |
| `const GroundCalibration& getCalibration() const`, `const SignalPlayer& getSignals() const` | Para o OLED, o log e a caixa-preta |

A ordem do `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. com o link vivo — `switches->update(rc)` (o modo, as funções, os potenciômetros);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. os sticks `mixer.fromSticks(rc)` (com o link vivo) × `Knob::RATES`; os
   flaps `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → suavemente, na perda de link — 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **sempre**;
   em reversão, no lugar de `pilotThrottle` vai `PWM_MIN` (para o piloto automático o motor está desligado);
6. a calibração: um movimento do ARM durante ela a cancela; sem ARM e com link — o gesto `CAL_SWITCH`; um passo de `GroundCalibration::update()`; os sinais dela e o sinal de recusa da decolagem automática vão para o `SignalPlayer`; o buzzer: `Beeper::update(BEEPER, armed, failsafe, now)` ou a luz do sinal;
7. link perdido → `applyLinkLoss()` e saída do ciclo;
8. `arming.update(rc, false, motivo)` — o ARM é recusado enquanto houver uma calibração em andamento ou um sinal tocando (e no ciclo em que um movimento do ARM cancelou uma calibração), senão `reverse.armBlockReason(now)`; depois `reverse.update(MOTOR_REVERSE, armed, now)` — no DISARM o sentido muda no mesmo ciclo;
9. `command = autopilot->getCommand()` (ou os sticks sem piloto automático), os flaps — os próprios; o aceno do sinal vai para o profundor e o leme;
10. `output = mixer.mix(command)`; `output.motorReverse = reverse.isReversed()`;
    para a frente — `output.throttle = autopilot->applyThrottle(pilotThrottle)` com o
    limite `THROTTLE_LIMIT_PCT` × o potenciômetro `Knob::THROTTLE_LIMIT`; para trás — o acelerador do piloto com o limite
    `min(REVERSE_THROTTLE_LIMIT_PCT, limite para a frente)`; o permitido é esticado por todo o curso do stick;
11. não armado ou `MOTOR_KILL` → `throttle = PWM_MIN` (por último); sem ARM, o sinal do motor liga `escPulsesOff`;
12. AUX1 — a carga (`PAYLOAD_DROP`), AUX2 — a câmera (`Knob::CAMERA_TILT`, `CAMERA_STAB` subtrai a arfagem); `AUX1_REVERSED`/`AUX2_REVERSED` espelham o pulso.
13. `outputs.write(output)`.

`applyLinkLoss()`: se o piloto automático está em failsafe (armado: RTH ou planeio) — as superfícies e o acelerador seguem o comando do piloto automático (os flaps recolhem suavemente, o `MOTOR_KILL` e a reversão silenciam o motor, AUX como estavam); caso contrário `outputs.setFailsafe()`, e enquanto um sinal toca — neutro com aceno e pausa dos pulsos do ESC (o rádio foi deixado de lado: a calibração está em andamento).

---

## `Beeper`

**Arquivo:** `control/Beeper.h` · **Depende de:** `Config`

| Método | Descrição |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | o estado do buzzer: 2 Hz se `Feature::BEEPER` ou “modelo perdido” (não armado, sem link por mais de `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | o modo “procure por mim na grama” |

---

## `GroundCalibration`

**Arquivo:** `control/GroundCalibration.h` · **Depende de:** `Config`, `RcChannelState`, `ImuSensor`, `ImuOrientation`

Calibração em solo do avião: passo 1 — cauda levantada, fuselagem na horizontal (o horizonte, `ImuOrientation::withLevel()`), passo 2 — sobre as rodas (a atitude em solo). Um passo é aceito sozinho depois de `CAL_STILL_MS` parado; o passo 2 espera o avião ser mudado de posição, ou `CAL_PARK_WAIT_MS`. Só grava completa (`ImuSensor::setGroundCalibration()`), depois de verificar a atitude em solo conforme `LANDING_GEAR`. Não bloqueia o ciclo.

`Stage`: `IDLE`, `LEVEL`, `PARK`, `DONE`, `FAILED`.

| Método | Descrição |
|---|---|
| `bool watchSwitch(const RcChannelState&, uint32_t nowMs)` | o gesto: `CAL_SWITCH` `CAL_SWITCH_FLIPS` vezes para baixo e para cima em até `CAL_SWITCH_WINDOW_MS`, acelerador embaixo; a primeira leitura e o acelerador fora de baixo zeram a contagem |
| `void forgetSwitch()` | ARM ou sem link — a contagem recomeça |
| `bool start(const ImuSensor*, uint32_t nowMs, const char* source)` | iniciar (sinal 1); sem IMU ou sem suporte — erro (sinal 3) |
| `void cancel(const char* why)` | cancelamento sem sinal |
| `void update(ImuSensor*, bool armed, bool signalBusy, uint32_t nowMs)` | um passo: o ARM cancela; enquanto um sinal toca, nada é aceito; estourou `CAL_STEP_TIMEOUT_MS` — erro |
| `isActive()`, `getStage()`, `getStageSinceMs()`, `getFailure()`, `getParkPitch()`, `getParkRoll()` | estado e resultado |
| `uint8_t takeSignal()` | quantos sinais tocar: 1 — iniciada / passo 1, 2 — pronta, 3 — erro (lido uma vez) |
| `static const char* stageShort(Stage)` | `CAL1`, `CAL2`, `CALOK`, `CALER` — para o OLED |

---

## `SignalPlayer`

**Arquivo:** `control/SignalPlayer.h` · **Depende de:** `Config`

Sinais para o piloto — uma função do tempo desde o início da série. Um sinal: o profundor e o leme acenam (`SIGNAL_NOD_US` para cima, depois para baixo, `SIGNAL_NOD_MS` cada), sem ARM os pulsos do ESC param por `SIGNAL_ESC_PAUSE_MS` (depois o ESC toca a melodia de pronto), a luz do buzzer acende durante o aceno; o sinal seguinte vem depois de `SIGNAL_PERIOD_MS`.

| Método | Descrição |
|---|---|
| `void play(uint8_t count, uint32_t nowMs, bool motor)` | uma série; `motor` — se pode mexer no ESC (e `SIGNAL_MOTOR`) |
| `void stop()` | silêncio |
| `bool isBusy(uint32_t nowMs) const`, `uint8_t getCount(uint32_t nowMs) const` | há uma série em andamento (enquanto isso o ARM é recusado) |
| `bool escPulsesOff(uint32_t nowMs) const` | tirar agora os pulsos do ESC |
| `int16_t nodUs(uint32_t nowMs) const` | aceno das superfícies, µs (`SIGNAL_SURFACES`) |
| `bool light(uint32_t nowMs) const` | buzzer / LED no `PIN_BUZZER` |
