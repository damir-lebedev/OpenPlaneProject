# RC — recepción de las órdenes de la emisora

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/rc.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual.

[← Referencia](README.md)

La capa RC convierte los bytes de la UART en valores de canal y en un
indicador de «sin enlace». No sabe nada del avión, del ARM, del comportamiento
de failsafe ni de los servos: cambiar el protocolo (S-Bus, PPM) afecta solo a
esta capa.

---

## `RcChannelState`

**Archivo:** `rc/RcChannelState.h` · **Depende de:** `Config`, `Channels`

Una instantánea de los 10 canales del receptor (µs), sin lógica de control.

| Método | Descripción |
|---|---|
| `RcChannelState()` | Llama a `reset()` |
| `void reset()` | Valores seguros: todos los canales a `PWM_CENTER`, el acelerador a `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | El valor del canal; un índice fuera de rango → `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | Escribe un canal; un índice fuera de rango se ignora |
| `const uint16_t* data() const` | Todo el array (para depuración) |

---

## `RcInput`

**Archivo:** `rc/RcInput.h` · **Clase:** un conjunto de funciones estáticas · **Depende de:** `Config`

Conversiones comunes de las señales RC.

| Método | Descripción |
|---|---|
| `static uint16_t clamp(uint16_t value)` | Limita a `PWM_MIN..PWM_MAX` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Lineal: 1000 → `−max`, 1500 → 0, 2000 → `+max` (la entrada se limita primero); `reverse` invierte el signo. El resultado se limita a ±`max` |

Ejemplo: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `IBusReceiver`

**Archivo:** `rc/IBusReceiver.h` · **Depende de:** `IUartPort`, `RcChannelState`, `Config`, `Channels`

Un analizador byte a byte del protocolo iBUS de FlySky.

**Formato de la trama** (32 bytes): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(first 30 bytes)`. Se toman los primeros `IBUS_CHANNELS` = 10
canales; el valor del canal son los **12 bits bajos** (en los altos el FS-iA6B
transmite datos de servicio, por ejemplo en failsafe `0x2384` → 900 µs).

| Método | Descripción |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | El puerto no se abre en el constructor |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`, el tiempo de espera cuenta desde «ahora» |
| `void update()` | Leer todo lo acumulado en la UART; llamar en cada ciclo |
| `const RcChannelState& getState() const` | Los últimos canales recibidos |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Todavía no ha habido ninguna trama **o** la última es más antigua que `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | En la última trama el acelerador < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` de la última trama correcta |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Contadores de tramas correctas y de errores de CRC |

La máquina de estados del análisis (`processByte`): espera `0x20`; el byte
siguiente debe ser `0x40`, y si no, la búsqueda empieza de nuevo; después
reúne 32 bytes y llama a `processFrame()`. Una trama con un CRC erróneo se
descarta por completo (los canales no cambian, `badFrames++`).

Invariantes:

- Hasta la primera trama correcta, `isSignalLost() == true`: los valores por
  defecto (todos a 1500) no se toman por órdenes de la emisora.
- El indicador de failsafe se recalcula en **cada** trama correcta: el enlace
  se restablece con la primera trama que traiga un acelerador normal.
