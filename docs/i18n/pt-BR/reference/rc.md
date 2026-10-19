# RC — recepção dos comandos do rádio

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/rc.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referência](README.md)

A camada RC transforma os bytes da UART em valores de canal e em um indicador
de “sem link”. Ela não sabe nada sobre a aeronave, o ARM, o comportamento de
failsafe nem os servos. O protocolo do receptor — iBUS, SBUS ou CRSF — é definido
por `Config::RX_PROTOCOL`; um protocolo novo é só mais um analisador de quadros
ao lado dos três que já existem.

---

## `RcChannelState`

**Arquivo:** `rc/RcChannelState.h` · **Depende de:** `Config`, `Channels`

Um instantâneo dos 10 canais do receptor (µs), sem lógica de controle.

| Método | Descrição |
|---|---|
| `RcChannelState()` | Chama `reset()` |
| `void reset()` | Valores seguros: todos os canais em `PWM_CENTER`, o acelerador em `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | O valor do canal; um índice fora da faixa → `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | Grava um canal; um índice fora da faixa é ignorado |
| `const uint16_t* data() const` | O array inteiro (para depuração) |

---

## `RcInput`

**Arquivo:** `rc/RcInput.h` · **Tipo:** um conjunto de funções estáticas · **Depende de:** `Config`

Conversões comuns dos sinais RC.

| Método | Descrição |
|---|---|
| `static uint16_t clamp(uint16_t value)` | Limita a `PWM_MIN..PWM_MAX` |
| `static uint16_t reversed(uint16_t value)` | Canal ao contrário: `3000 − clamp(value)` (1000 ↔ 2000) — para `Config::CHANNEL_REVERSED` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Linear: 1000 → `−max`, 1500 → 0, 2000 → `+max` (a entrada é limitada primeiro); `reverse` inverte o sinal. O resultado é limitado a ±`max` |

Exemplo: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `RcReceiver`

**Arquivo:** `rc/RcReceiver.h` · **Depende de:** `IUartPort`, `IBusParser`, `SbusParser`, `CrsfParser`, `RcChannelState`, `RcInput`, `Config`, `Channels`

O receptor do rádio: abre a UART no formato do protocolo, entrega os bytes ao
analisador e faz o que é comum a todos os protocolos — os primeiros
`RC_CHANNELS` = 10 canais, a inversão de canais, o timeout de quadros, o
failsafe e os contadores.

| Protocolo (`Config::RX_PROTOCOL`) | Receptores | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B, FS-iA10B | 115200, 8N1 |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper e a maioria dos outros | 100000, 8E2, invertido — a própria UART inverte |
| `CRSF` | ExpressLRS, TBS Crossfire e Tracer | 420000, 8N1 |

| Método | Descrição |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | A porta não é aberta no construtor; nos testes o protocolo pode ser qualquer um |
| `void begin()` | Abre a UART no formato do protocolo (`IUartPort::begin(baud, format)`) e começa a contar o timeout a partir de “agora” |
| `void update()` | Lê tudo o que se acumulou na UART; chamar a cada ciclo |
| `const RcChannelState& getState() const` | Os últimos canais recebidos |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Ainda não chegou nenhum quadro **ou** o último é mais antigo que `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | O receptor informou a perda de link: flag de failsafe no quadro SBUS, qualidade de link 0 % no CRSF ou acelerador < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` do último quadro correto |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Contadores de quadros aceitos e descartados (CRC, fim do quadro) |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | O protocolo; o nome é `"iBUS"`, `"SBUS"` ou `"CRSF"` |
| `int16_t getLinkQuality() const` | Qualidade do link de rádio, % — só no CRSF; senão, −1 |
| `bool isPortReady() const` | A UART abriu no formato necessário; `false` — a porta não sabe fazer isso (SBUS sem inversão), não haverá link |

A função livre `rxProtocolName(Config::RxProtocol)` dá o nome do protocolo; é o
mesmo que o passaporte do firmware imprime (`@id`, campo `rx`).

Invariantes:

- Até o primeiro quadro correto `isSignalLost() == true` — os valores padrão
  (todos 1500) não são tomados como comandos do rádio.
- O indicador de failsafe é recalculado em **cada** quadro de canais — o link
  volta já no primeiro quadro com acelerador normal (no CRSF, também com uma
  estatística de qualidade de link acima de 0).
- Os canais com `Config::CHANNEL_REVERSED` são guardados invertidos
  (`RcInput::reversed`); o failsafe é detectado antes, pelo acelerador **bruto**, e
  a reversão do CH3 é proibida por `static_assert`.

---

## `IBusParser`

**Arquivo:** `rc/IBusParser.h` · **Depende de:** `Config`

Analisador byte a byte do FlySky iBUS: `bool feed(uint8_t)` retorna `true` quando um
quadro com CRC correto se completa; os canais ficam em `values()`. **Quadro** (32 bytes): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(primeiros 30 bytes)`; o valor do canal são os **12 bits baixos**
(nos altos o FS-iA6B envia dados de serviço, por exemplo em failsafe `0x2384` →
900 µs). Espera `0x20`; o byte seguinte deve ser `0x40`, senão a busca recomeça
(um segundo `0x20` pode ele mesmo iniciar um quadro). O iBUS não tem flag de
failsafe: o FS-iA6B informa a perda de link com o acelerador abaixo do normal.

---

## `SbusParser`

**Arquivo:** `rc/SbusParser.h` · **Depende de:** `Packed11`

**Quadro** (25 bytes): `0x0F | 16 canais de 11 bits (22 bytes) | flags | fim`;
flags: bit 2 — um quadro perdido pelo rádio (ainda não é perda de link), bit 3 —
failsafe; fim — `0x00` no SBUS, `0x04`/`0x14`/`0x24`/`0x34` no SBUS2. O byte
`0x0F` também aparece nos dados, por isso o quadro só é aceito quando início e
fim estão no lugar; senão a análise continua a partir do próximo `0x0F` dentro
do que foi recebido — o fluxo sincroniza em um ou dois quadros.

---

## `CrsfParser`

**Arquivo:** `rc/CrsfParser.h` · **Depende de:** `Packed11`

**Quadro:** `0xC8 | comprimento | tipo | dados | CRC8`; comprimento = tipo + dados +
CRC, o quadro tem no máximo 64 bytes; CRC8 com polinômio `0xD5` (DVB-S2) sobre
tipo e dados (`static uint8_t crc8(...)`). São necessários dois tipos: `0x16` —
canais (16 de 11 bits, como no SBUS) e `0x14` — estatística do link (o terceiro
byte é a qualidade do link, %). Ao perder o link, o ExpressLRS por padrão
simplesmente para de enviar canais — é o timeout de quadros; no modo “últimas
posições” os canais continuam chegando, mas a qualidade do link é 0 — isso é
`failsafe()`.

---

## `Packed11`

**Arquivo:** `rc/Packed11.h` · **Tipo:** namespace

16 canais de 11 bits em 22 bytes, bits menos significativos primeiro — é assim
que o SBUS e o CRSF os empacotam. `toMicros(raw)`: 172…1811 → 988…2012 µs,
992 → 1500 (`5/8 · valor + 880`, como no Betaflight e no INAV); `unpack(data, out)`
transforma 22 bytes em 16 valores em µs.
