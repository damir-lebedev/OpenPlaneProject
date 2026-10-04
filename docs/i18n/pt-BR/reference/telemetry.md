# TELEMETRY — registro, console, painel web, OLED, caixa-preta

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/telemetry.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referência](README.md)

A telemetria está totalmente separada da lógica de voo: ela apenas lê os
getters constantes de `FlightController`, `Autopilot`, dos sensores e de `LoopStats`.
O único caminho “de volta” são os comandos do painel, que passam pela caixa de correio do
`WebDebugServer` e são aplicados pelo laço de voo.

---

## `LoopStats`

**Arquivo:** `telemetry/LoopStats.h` · **Tipo:** struct

A frequência e a duração do laço de voo.

| Membro | Descrição |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | Publicados uma vez por segundo; lidos de outras tarefas (valores de 32 bits — sem leituras “rasgadas”) |
| `void record(uint32_t durationUs)` | Chamar a cada ciclo, a partir de `loop()` |
| `uint32_t takePeakUs()` | O pior ciclo desde a chamada anterior (para a linha SYS a cada 10 s); chamar da mesma tarefa que `record()` |

`maxUs` é o pior apenas do último segundo; um tropeço raro é visível por meio de
`takePeakUs()`.

---

## `LogSettings`

**Arquivo:** `telemetry/LogSettings.h` · **Depende de:** `Preferences` (NVS, o namespace `debuglog`)

### `LogChannel` (enum class)

| Canal | Prefixo | O que imprime | Padrão |
|---|---|---|---|
| `Status` | `STAT` | enlace, ARM, modo, flaps, sensores | ao mudar |
| `Rc` | `RC` | canais do rádio | desligado |
| `Outputs` | `OUT` | saídas para as superfícies de controle e o ESC | desligado |
| `Attitude` | `ATT` | rolagem, arfagem, rumo | desligado |
| `Autopilot` | `AP` | alvos e correções | desligado |
| `Altitude` | `ALT` | altitude, velocidade vertical | desligado |
| `Heading` | `MAG` | rumo da bússola | desligado |
| `Gps` | `GPS` | satélites, coordenadas | desligado |
| `Imu` | `IMU` | giroscópio e acelerômetro | desligado |
| `Nav` | `NAV` | casa, rumo, velocidade, tubo de Pitot, funções ativadas | desligado |
| `System` | `SYS` | frequência do laço, memória (a cada 10 s), somente desligado/ligado | ligado |
| `Count` | — | o número de canais | — |

`LogMode` (enum class): `Off`, `OnChange`, `Periodic`.

`LogChannelInfo`: `tag`, `title`, `periodicOnly`, `defaultMode`.

| Método | Descrição |
|---|---|
| `static constexpr uint8_t COUNT`, `PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | Uma linha da tabela de canais |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms (de forma cíclica) |
| `LogSettings()`, `void setDefaults()` | Os modos padrão, um período de 1 s |
| `LogMode mode(uint8_t)`, `mode(LogChannel)` | O modo de um canal |
| `void setMode(uint8_t, LogMode)` | Nos canais `periodicOnly`, `OnChange` vira `Periodic` |
| `void cycleMode(uint8_t)` | desligado → ao mudar → contínuo → desligado (SYS: desligado ↔ ligado) |
| `void setAll(LogMode)` | Para todos os canais; o comando “tudo ao mudar” não mexe no SYS |
| `uint16_t periodMs() const`, `void cyclePeriod()` | O período do modo “contínuo” |
| `static const char* modeName(LogMode, bool periodicOnly)` | “desligado” / “ao mudar” / “contínuo” (ou “ligado”) |
| `void load()` | Da NVS; se `VERSION` ou o comprimento não coincidirem, ficam os valores padrão; um código de modo desconhecido → o padrão do canal |
| `void save() const` | Para a NVS (os modos, o período, a versão) |

`VERSION` muda junto com a lista de canais — as configurações antigas são zeradas
(`VERSION = 2`: foi adicionado o canal NAV). As teclas dos canais no menu: `1`..`9`, NAV —
`n`, SYS — `s`.

---

## `DebugLogger`

**Arquivo:** `telemetry/DebugLogger.h` · **Depende de:** `FlightController`, `Autopilot*`, `LoopStats*`, `LogSettings`, `Config`

Impressão do estado no monitor serial por canais: cada um tem a sua linha, o seu
modo e as suas tolerâncias.

| Método | Descrição |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | Uma vez por `DEBUG_INTERVAL_MS` percorre os canais (fica em silêncio na pausa e enquanto o menu está aberto) |
| `LogSettings& getSettings()`, `void saveSettings() const` | Para o menu do console |
| `void suspend(bool)` | O menu está aberto — ficar em silêncio; ao soltar — `refresh()` |
| `void setPaused(bool)`, `bool isPaused() const` | Pausa com a barra de espaço; ao soltar — `refresh()` |
| `void refresh()` | O próximo ciclo imprimirá todos os canais ativados |

A lógica do canal (`updateChannel`):

- `Off` — não imprimir;
- `Periodic` — uma vez por `periodMs()` (SYS — uma vez a cada 10 s), valores “como estão”;
- `OnChange` — a linha é montada com **tolerâncias** (o `Shown` aninhado mantém
  o valor antigo até que o novo se afaste mais que a tolerância: RC/PWM 3 µs, ângulos
  0,5°, rumo 1°, correções 2, altitude 0,3 m, aceleração 0,03 g, coordenadas
  1e−5°) e é impressa somente se diferir da última impressa.

Os tipos aninhados: `LineBuffer : Print` (uma linha de até 200 bytes para comparar antes de
imprimir), `Shown` (um valor com histerese).

Os formatos das linhas:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  alvo 0.0 m
MAG  rumo 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (o pior em 10 s) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=` distingue `LOST(sem quadros)` de `LOST(failsafe do rádio)`; `IMU=` é
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK`.

---

## `DebugConsole`

**Arquivo:** `telemetry/DebugConsole.h` · **Depende de:** `FlightController`, `FlightOutputs`, `Autopilot`, `DebugLogger`, `LogSettings`, `IBoard*` (varredura dos barramentos)

Um menu de texto no monitor serial. Uma máquina de estados das telas `Screen::{None, Main, Log}`.

| Método | Descrição |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | com a placa — o comando `b` e o item 7 do menu |
| `static const char* guessI2cDevice(uint8_t address)` | o chip pelo endereço: 0x6A LSM6DSV, 0x68 MPU/ICM, 0x76 BME280/BMP388/SPL06, 0x46/0x47 BMP581, 0x7C QMC6309, 0x2C QMC5883P, 0x0D QMC5883L, 0x3C OLED |
| `void printHint() const` | Uma dica de uma linha |
| `void update()` | Processa todos os bytes de `Serial`; se as configurações do registro foram alteradas, o menu está fechado e a aeronave **não está armed** — salva na NVS |

Teclas de atalho (fora do menu): `h`/`?` — o menu principal; `l` — o menu do registro; espaço —
pausa do registro; `s` — status dos sensores; `i` — calibração do giroscópio; `o` —
calibração da montagem da IMU; `m` — calibração da bússola; `p` — autoteste das saídas;
`b` — varredura dos barramentos I2C (0x08..0x7F — até 0x7F, porque o QMC6309 fica em 0x7C) com
os nomes dos chips; qualquer outra — uma dica. `\r`/`\n` são ignorados.

O menu do registro: `1`..`9` — alternam o modo dos canais 0..8, `n` — NAV, `s` — SYS, `p` — período, `a` —
tudo “ao mudar”, `x` — tudo desligado, `d` — padrão, `0`/`q` — voltar, `l`/`h` —
fechar.

As ações bloqueantes (`i`, `o`, `m`, `p`) são **proibidas com ARM**. Enquanto o menu
está aberto, o registro fica suspenso (`DebugLogger::suspend`). A largura dos itens do menu
é contada em caracteres UTF-8, e não em bytes (o cirílico ocupa 2 bytes).

---

## `WebDashboardPage`

**Arquivo:** `telemetry/WebDashboardPage.h` · **Tipo:** namespace

`static const char HTML[] PROGMEM` — a página inteira (HTML + CSS + JS) em um único
literal. Tudo o que é dinâmico é construído pelo navegador a partir do JSON de `/api/status` (consultado
a cada 200 ms): as linhas de canais, saídas e sensores são criadas pelas chaves do JSON, de modo que uma nova
saída aparece sem editar a página. Um campo de PID que o usuário começou
a editar não é mais sobrescrito pela consulta periódica.

---

## `WebDebugServer`

**Arquivo:** `telemetry/WebDebugServer.h` · **Depende de:** `WebServer`, `WiFi`, `FlightController`, `Autopilot*`, `WebDashboardPage`, `Config`

| Método | Descrição |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | Um AP de Wi-Fi (`persistent(false)` — sem escrever na flash), as rotas, a tarefa `web` no núcleo 0. `false` se o ponto de acesso não subiu |
| `void applyPendingCommands()` | Chamar do laço de voo: pega os comandos sob um spinlock e os aplica ao piloto automático |

As rotas:

| Rota | Resposta |
|---|---|
| `GET /` | A página do painel |
| `GET /api/status` | O JSON de estado (`buildStatusJson()`), o formato está no [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}`; sem corpo → 400 `no data`; sem piloto automático → 503; modo errado → 400 `invalid mode` |
| `POST /api/setpid` | Qualquer um de `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch`; os omitidos ficam como estão |
| o resto | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }` — uma caixa de correio sob um
`portMUX`. `extractJsonNumber(body, key, fallback)` — uma análise mínima de
JSON plano sem ArduinoJson: `"key"`, espaços, `:`, espaços, um número em
qualquer notação JSON (sinal, fração, expoente `1e-7`); se faltar a chave ou o número —
`fallback`.

No JSON os campos `attached`/`available` existem **sempre**; os dados do sensor, somente
com `available: true`.

---

## `OledDisplay`

**Arquivo:** `telemetry/OledDisplay.h` · **Depende de:** U8g2, `II2CBus`, `FlightController`, `Autopilot*`, `LoopStats`

Um SSD1306 128×64 (I2C 0x3C) no segundo barramento I2C; uma tarefa própria `oled`
(`Rtos::startTask`: núcleo 0 no ESP32, prioridade baixa no STM32), a cada 200 ms.

| Método | Descrição |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr` ou a tela não responde em 0x3C → `false`; caso contrário configura o U8g2 e inicia a tarefa |

O U8g2 envia os bytes por um `byteCallback` sobre o `II2CBus` (a tela não sabe nada sobre
`Wire1`). Um callback de C não recebe contexto, por isso o barramento é guardado em uma variável
estática `busSlot()` — há apenas uma tela a bordo.

A tela:

```
RX ok ARM STAB FL       enlace (perda — linha invertida) / ARM / modo / flaps
R  +1.2 P  -0.4         rolagem / arfagem, °           (IMU --)
Alt +0.3 Vz +0.1 A14    altitude / velocidade vertical / velocidade do ar, se houver tubo de Pitot (BARO --)
H123 T1000 Y1500        rumo / aceleração / leme de direção (H---)
L1500 R1500 E1500       ailerons / profundor
Loop 500Hz max1100us    frequência e o pior ciclo no segundo
```

Os nomes curtos dos modos são `AutopilotNames::modeShort()` (`MAN`, `STAB`,
`TKOFF`, `ALT`, `ACRO`, `CRZ`, `LOIT`, `RTH`, `LNCH`, `LAND`, `SOAR`, `RESQ`);
na perda do enlace no ar — `GLIDE` ou `FSRTH`.

---

## `BlackBox`

**Arquivo:** `telemetry/BlackBox.h` · **Depende de:** `FlightController`, `Autopilot`, `LoopStats`, `BlackBoxStorage`, `PilotSwitches*`

A gravação do voo na flash (ESP32-S3) ou em um cartão SD (STM32H743). O que baixar, quando e como — em [BLACKBOX.md](../BLACKBOX.md).

| Método | Descrição |
|---|---|
| `bool begin(bool startTask = true)` | Lê o meio (`BlackBoxStorage::begin()`), aloca a fila (PSRAM no ESP32, `malloc` no STM32), confere o espaço apagado (até 0,3 s), inicia a tarefa `bbox` (`Rtos::startTask`). Se não houver onde gravar (partição, cartão, arquivo) — `false`, a caixa-preta fica desligada |
| `void update(uint32_t workUs)` | De `loop()` após cada ciclo: eventos, início/parada, instantâneos para a fila, acorda a tarefa de gravação |
| `void writerStep()` | Um passo da tarefa de gravação: uma ou duas páginas para a flash, ou um apagamento em solo |
| `requestManualStart()` / `requestManualStop()` | Gravação manual (console `k` → `r`) |
| `State getState()` / `bool isRecording()` | `Off`, `Idle`, `Recording`, `Stopping` (termina de gravar a fila antes de registrar o END) |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | Para o console |
| `void handleHostCommand(const char*)` | `bb list`, `bb get <n> [baud]` — para o `tools/blackbox.py` (no USB CDC a velocidade não influencia em nada) |

As partes específicas de cada plataforma: a causa da reinicialização — `readResetCause()`; a tensão e a corrente da bateria —
o ADC (`analogReadMilliVolts` no S3, um `analogRead` de 12 bits no STM32); os erros do meio
(`BlackBoxStorage::writeErrors`/`eraseErrors`) vão para o registro uma vez por segundo
como o evento “meio: erros de gravação …” e não atrapalham o voo.

## `BlackBoxStorage`

**Arquivo:** `telemetry/BlackBoxStorage.h` · **Depende de:** `IFlashRegion`

Um anel de setores de 4 KB: a cabeça e a lista de voos vêm dos cabeçalhos dos setores em `begin()` (a primeira passada lê o cabeçalho de cada setor e guarda os genuínos, a segunda só esses: uma área vazia é lida uma única vez); `openFlight()`/`append()`/`flush()`/`closeFlight()` — gravação por páginas (um CRC-8 em cada registro); `eraseStep(target, protect, allowErase)` — um passo de conferência/apagamento à frente da cabeça: o lixo — sempre, os voos — inteiros e só enquanto houver menos de `target` livre; `protect` nunca é tocado.

## `BlackBoxRing`, `BlackBoxFormat`

`BlackBoxRing` é uma fila de bytes de registros entre tarefas/núcleos sob `Rtos::CriticalSection`; quando transborda, descarta os mais antigos. `BlackBoxFormat` — o cabeçalho do setor, os tipos e as estruturas dos registros, as strings dos esquemas (o tamanho é conferido por `static_assert`), CRC-8 e CRC-32.

---

## `Mavlink` (codec)

**Arquivo:** `telemetry/MavlinkCodec.h` · **Tipo:** namespace · **Depende de:** nada (portável)

MAVLink 2 sem a biblioteca gerada: empacotamento dos campos na ordem do MAVLink
(conferido com o pymavlink), CRC-16/MCRF4XX + `CRC_EXTRA`, corte dos zeros finais.

| Entidade | Descrição |
|---|---|
| `Msg::*` | identificadores: HEARTBEAT, SYS_STATUS, SET_MODE, PARAM_*, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW, MISSION_REQUEST_LIST/COUNT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS, REQUEST_DATA_STREAM, VFR_HUD, COMMAND_LONG/ACK, HOME_POSITION, STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | o `CRC_EXTRA` de uma mensagem, −1 — desconhecido |
| `crcAccumulate`, `crcCalculate` | X.25 (como o `crc_accumulate()` do mavlink) |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` — os campos em ordem |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` — um quadro v2 com `seq` sequencial |
| `Message` | uma mensagem recebida: `msgid`, `sysid`, `compid`, o payload (completado com zeros), leitura dos campos por deslocamento |
| `Parser` | `bool feed(byte)` → `message()`; v1 e v2, a assinatura da v2 é ignorada; `goodCount()`, `badCrcCount()`; mensagens com `CRC_EXTRA` desconhecido são descartadas em silêncio |

## `MavlinkModes`

**Arquivo:** `telemetry/MavlinkTelemetry.h` · **Tipo:** namespace

| Função | Descrição |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | o número do modo do ArduPlane: MANUAL 0, STABILIZE→FBWA 5, ALT_HOLD→FBWB 6, ACRO 4, CRUISE 7, LOITER 12, RTH→RTL 11, AUTO_TAKEOFF/LAUNCH→TAKEOFF 13, AUTO_LAND→AUTO 10, SOARING→THERMAL 24, RESCUE→STABILIZE 2; failsafe → RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | o inverso, para comandos vindos do solo; AUTO, CIRCLE, GUIDED — `false` |
| `isAutonomous(mode)` | o sinalizador `AUTO_ENABLED` no HEARTBEAT |

## `MavlinkTelemetry`

**Arquivo:** `telemetry/MavlinkTelemetry.h` · **Depende de:** `IUartPort`, `FlightController`, `Autopilot*`, `LoopStats*`

Telemetria por modem de rádio para o QGroundControl / Mission Planner (o veículo é
`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_ARDUPILOTMEGA`). Usada no STM32
(UART4), que não tem Wi-Fi.

| Método | Descrição |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | abrir a porta, a mensagem “OpenPlane online” |
| `void update()` | do laço de voo: analisa o que chega (≤ 128 bytes por ciclo), mensagens de eventos, no máximo 2 quadros por ciclo |
| `void statusText(severity, text)` | para o fluxo da GCS (uma fila de 4 linhas, de até 50 caracteres) |
| `isGcsConnected()` | um HEARTBEAT da GCS nos últimos 3 s |
| `getSentFrames()`, `getDeferredFrames()`, `getParser()` | diagnóstico |
| `static const char* paramName(uint8_t)` | `RLL_KP`, `RLL_KI`, `RLL_KD`, `PTCH_KP`, `PTCH_KI`, `PTCH_KD` |

Os fluxos (Hz): ATTITUDE 10; GLOBAL_POSITION_INT, VFR_HUD 5; GPS_RAW_INT,
RC_CHANNELS, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT 2; HEARTBEAT, SYS_STATUS 1;
HOME_POSITION 0,2. Um quadro só é enviado se `availableForWrite()` tiver espaço para ele
— caso contrário, espera o próximo ciclo (o laço nunca é bloqueado).

Entrada: o HEARTBEAT da GCS; PARAM_REQUEST_LIST / READ / SET (o PID — direto para o
piloto automático, valores 0..100, não são salvos); SET_MODE e COMMAND_LONG
`DO_SET_MODE` (176) — o modo até o próximo clique da chave; `COMPONENT_ARM_DISARM`
(400) — **DENIED**; `REQUEST_MESSAGE` (512) — um envio fora de vez de um fluxo;
MISSION_REQUEST_LIST — MISSION_COUNT 0 com o mesmo `mission_type`.
A verificação do fluxo com um decodificador externo — `tools/check_mavlink.py` (pymavlink).
