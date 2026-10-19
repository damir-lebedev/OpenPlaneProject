# RC — recepción de las órdenes de la emisora

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/rc.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referencia](README.md)

La capa RC convierte los bytes de la UART en valores de canal y en un
indicador de «sin enlace». No sabe nada del avión, del ARM, del comportamiento
de failsafe ni de los servos. El protocolo del receptor —iBUS, SBUS o CRSF— lo
fija `Config::RX_PROTOCOL`; un protocolo nuevo es un analizador de tramas más
junto a los tres que ya hay.

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
| `static uint16_t reversed(uint16_t value)` | Canal al revés: `3000 − clamp(value)` (1000 ↔ 2000), para `Config::CHANNEL_REVERSED` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Lineal: 1000 → `−max`, 1500 → 0, 2000 → `+max` (la entrada se limita primero); `reverse` invierte el signo. El resultado se limita a ±`max` |

Ejemplo: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `RcReceiver`

**Archivo:** `rc/RcReceiver.h` · **Depende de:** `IUartPort`, `IBusParser`, `SbusParser`, `CrsfParser`, `RcChannelState`, `RcInput`, `Config`, `Channels`

El receptor de la emisora: abre la UART con el formato del protocolo, pasa los
bytes al analizador y hace lo común a todos los protocolos: los primeros
`RC_CHANNELS` = 10 canales, la inversión de canales, el tiempo límite de tramas,
el failsafe y los contadores.

| Protocolo (`Config::RX_PROTOCOL`) | Receptores | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B, FS-iA10B | 115200, 8N1 |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper y la mayoría de los demás | 100000, 8E2, invertida: invierte la propia UART |
| `CRSF` | ExpressLRS, TBS Crossfire y Tracer | 420000, 8N1 |

| Método | Descripción |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | El puerto no se abre en el constructor; en las pruebas el protocolo puede ser cualquiera |
| `void begin()` | Abre la UART con el formato del protocolo (`IUartPort::begin(baud, format)`) y empieza a contar el tiempo límite desde «ahora» |
| `void update()` | Lee todo lo acumulado en la UART; llamar en cada ciclo |
| `const RcChannelState& getState() const` | Los últimos canales recibidos |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Aún no ha llegado ninguna trama **o** la última es más antigua que `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | El receptor informó de la pérdida de enlace: indicador de failsafe en la trama SBUS, calidad de enlace 0 % en CRSF o gas < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` de la última trama correcta |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Contadores de tramas aceptadas y descartadas (CRC, final de trama) |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | El protocolo; su nombre es `"iBUS"`, `"SBUS"` o `"CRSF"` |
| `int16_t getLinkQuality() const` | Calidad del enlace de radio, %: solo CRSF; si no, −1 |
| `bool isPortReady() const` | La UART se abrió con el formato necesario; `false`: el puerto no sabe hacerlo (SBUS sin inversión) y no habrá enlace |

La función libre `rxProtocolName(Config::RxProtocol)` da el nombre del protocolo; es
el mismo que imprime el pasaporte del firmware (`@id`, campo `rx`).

Invariantes:

- Hasta la primera trama correcta `isSignalLost() == true`: los valores por
  defecto (todos 1500) no se toman como órdenes de la emisora.
- El indicador de failsafe se recalcula en **cada** trama de canales: el
  enlace vuelve con la primera trama con gas normal (en CRSF, además, con una
  estadística de calidad de enlace mayor que 0).
- Los canales con `Config::CHANNEL_REVERSED` se guardan invertidos
  (`RcInput::reversed`); el failsafe se detecta antes, con el gas **en bruto**, y
  la inversión de CH3 la prohíbe un `static_assert`.

---

## `IBusParser`

**Archivo:** `rc/IBusParser.h` · **Depende de:** `Config`

Analizador byte a byte de FlySky iBUS: `bool feed(uint8_t)` devuelve `true` cuando
se completa una trama con CRC correcto; los canales están en `values()`. **Trama** (32 bytes): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(primeros 30 bytes)`; el valor del canal son los **12 bits bajos**
(en los altos la FS-iA6B envía datos de servicio, p. ej. en failsafe `0x2384` →
900 µs). Espera `0x20`; el byte siguiente debe ser `0x40`, si no, la búsqueda
vuelve a empezar (un segundo `0x20` puede iniciar una trama). iBUS no tiene
indicador de failsafe: la FS-iA6B informa de la pérdida de enlace con un gas por
debajo de lo normal.

---

## `SbusParser`

**Archivo:** `rc/SbusParser.h` · **Depende de:** `Packed11`

**Trama** (25 bytes): `0x0F | 16 canales de 11 bits (22 bytes) | indicadores | final`;
indicadores: bit 2 — se perdió una trama por radio (aún no es pérdida de
enlace), bit 3 — failsafe; final — `0x00` en SBUS, `0x04`/`0x14`/`0x24`/`0x34`
en SBUS2. El byte `0x0F` también aparece en los datos, así que la trama solo se
acepta si el inicio y el final están en su sitio; si no, el análisis sigue desde
el siguiente `0x0F` dentro de lo recibido: el flujo se sincroniza en una o dos
tramas.

---

## `CrsfParser`

**Archivo:** `rc/CrsfParser.h` · **Depende de:** `Packed11`

**Trama:** `0xC8 | longitud | tipo | datos | CRC8`; longitud = tipo + datos + CRC,
la trama no supera los 64 bytes; CRC8 con polinomio `0xD5` (DVB-S2) sobre tipo
y datos (`static uint8_t crc8(...)`). Hacen falta dos tipos: `0x16` — canales
(16 de 11 bits, como en SBUS) y `0x14` — estadística del enlace (el tercer byte
es la calidad del enlace, %). Al perder el enlace, ExpressLRS por defecto
simplemente deja de enviar canales: es el tiempo límite de tramas; en el modo
«últimas posiciones» los canales siguen llegando, pero la calidad del enlace es
0: eso es `failsafe()`.

---

## `Packed11`

**Archivo:** `rc/Packed11.h` · **Clase:** espacio de nombres

16 canales de 11 bits en 22 bytes, primero los bits menos significativos: así
los empaquetan SBUS y CRSF. `toMicros(raw)`: 172…1811 → 988…2012 µs, 992 → 1500
(`5/8 · valor + 880`, como en Betaflight e INAV); `unpack(data, out)` convierte
22 bytes en 16 valores en µs.
