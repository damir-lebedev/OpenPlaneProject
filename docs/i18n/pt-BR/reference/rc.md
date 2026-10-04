# RC — recepção dos comandos do rádio

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/rc.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referência](README.md)

A camada RC transforma os bytes da UART em valores de canal e em um indicador
de “sem link”. Ela não sabe nada sobre a aeronave, o ARM, o comportamento de
failsafe nem os servos — trocar o protocolo (S-Bus, PPM) afeta só esta camada.

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
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Linear: 1000 → `−max`, 1500 → 0, 2000 → `+max` (a entrada é limitada primeiro); `reverse` inverte o sinal. O resultado é limitado a ±`max` |

Exemplo: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `IBusReceiver`

**Arquivo:** `rc/IBusReceiver.h` · **Depende de:** `IUartPort`, `RcChannelState`, `Config`, `Channels`

Um analisador byte a byte do protocolo iBUS da FlySky.

**Formato do quadro** (32 bytes): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(first 30 bytes)`. São tomados os primeiros `IBUS_CHANNELS` =
10 canais; o valor do canal são os **12 bits menos significativos** (nos bits
altos o FS-iA6B transmite dados de serviço, por exemplo em failsafe
`0x2384` → 900 µs).

| Método | Descrição |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | A porta não é aberta no construtor |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`, o timeout conta a partir de “agora” |
| `void update()` | Ler tudo o que se acumulou na UART; chamar a cada ciclo |
| `const RcChannelState& getState() const` | Os últimos canais recebidos |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Ainda não houve nenhum quadro **ou** o último é mais antigo que `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | No último quadro o acelerador < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` do último quadro correto |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Contadores de quadros corretos e de erros de CRC |

A máquina de estados da análise (`processByte`): espera `0x20`; o byte seguinte
deve ser `0x40`, senão a busca recomeça; depois reúne 32 bytes e chama
`processFrame()`. Um quadro com CRC errado é descartado por inteiro (os canais
não mudam, `badFrames++`).

Invariantes:

- Até o primeiro quadro correto, `isSignalLost() == true` — os valores padrão
  (todos em 1500) não são tomados como comandos do rádio.
- O indicador de failsafe é recalculado a **cada** quadro correto — o link é
  restabelecido já pelo primeiro quadro com um acelerador normal.
