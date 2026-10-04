# TELEMETRY — registro, consola, panel web, OLED, caja negra

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/telemetry.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual.

[← Referencia](README.md)

La telemetría está completamente separada de la lógica de vuelo: solo lee los
getters constantes de `FlightController`, `Autopilot`, los sensores y `LoopStats`.
El único camino «de vuelta» son los comandos del panel, que pasan por el buzón de
`WebDebugServer` y los aplica el bucle de vuelo.

---

## `LoopStats`

**Archivo:** `telemetry/LoopStats.h` · **Clase:** struct

La frecuencia y la duración del bucle de vuelo.

| Miembro | Descripción |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | Se publican una vez por segundo; se leen desde otras tareas (valores de 32 bits — sin lecturas «rotas») |
| `void record(uint32_t durationUs)` | Llamar en cada ciclo desde `loop()` |
| `uint32_t takePeakUs()` | El peor ciclo desde la llamada anterior (para la línea SYS cada 10 s); llamar desde la misma tarea que `record()` |

`maxUs` es el peor solo del último segundo; un tropiezo poco frecuente se ve a través de
`takePeakUs()`.

---

## `LogSettings`

**Archivo:** `telemetry/LogSettings.h` · **Depende de:** `Preferences` (NVS, el espacio de nombres `debuglog`)

### `LogChannel` (enum class)

| Canal | Prefijo | Qué imprime | Por defecto |
|---|---|---|---|
| `Status` | `STAT` | enlace, ARM, modo, flaps, sensores | al cambiar |
| `Rc` | `RC` | canales del mando | apagado |
| `Outputs` | `OUT` | salidas a las superficies de control y al ESC | apagado |
| `Attitude` | `ATT` | alabeo, cabeceo, rumbo | apagado |
| `Autopilot` | `AP` | objetivos y correcciones | apagado |
| `Altitude` | `ALT` | altitud, velocidad vertical | apagado |
| `Heading` | `MAG` | rumbo de la brújula | apagado |
| `Gps` | `GPS` | satélites, coordenadas | apagado |
| `Imu` | `IMU` | giroscopio y acelerómetro | apagado |
| `Nav` | `NAV` | casa, rumbo, velocidad, tubo de Pitot, funciones activadas | apagado |
| `System` | `SYS` | frecuencia del bucle, memoria (cada 10 s), solo apagado/encendido | encendido |
| `Count` | — | el número de canales | — |

`LogMode` (enum class): `Off`, `OnChange`, `Periodic`.

`LogChannelInfo`: `tag`, `title`, `periodicOnly`, `defaultMode`.

| Método | Descripción |
|---|---|
| `static constexpr uint8_t COUNT`, `PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | Una fila de la tabla de canales |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms (de forma cíclica) |
| `LogSettings()`, `void setDefaults()` | Los modos por defecto, un periodo de 1 s |
| `LogMode mode(uint8_t)`, `mode(LogChannel)` | El modo de un canal |
| `void setMode(uint8_t, LogMode)` | En los canales `periodicOnly`, `OnChange` se convierte en `Periodic` |
| `void cycleMode(uint8_t)` | apagado → al cambiar → continuo → apagado (SYS: apagado ↔ encendido) |
| `void setAll(LogMode)` | Para todos los canales; el comando «todo al cambiar» no toca SYS |
| `uint16_t periodMs() const`, `void cyclePeriod()` | El periodo del modo «continuo» |
| `static const char* modeName(LogMode, bool periodicOnly)` | «apagado» / «al cambiar» / «continuo» (o «encendido») |
| `void load()` | Desde NVS; si `VERSION` o la longitud no coinciden, se quedan los valores por defecto; un código de modo desconocido → el valor por defecto del canal |
| `void save() const` | A NVS (los modos, el periodo, la versión) |

`VERSION` cambia junto con la lista de canales: los ajustes antiguos se restablecen
(`VERSION = 2`: se añadió el canal NAV). Las teclas de los canales en el menú: `1`..`9`, NAV —
`n`, SYS — `s`.

---

## `DebugLogger`

**Archivo:** `telemetry/DebugLogger.h` · **Depende de:** `FlightController`, `Autopilot*`, `LoopStats*`, `LogSettings`, `Config`

Imprime el estado en el monitor serie por canales: cada uno tiene su propia línea, su propio
modo y sus propias tolerancias.

| Método | Descripción |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | Una vez por `DEBUG_INTERVAL_MS` recorre los canales (calla en pausa y mientras el menú está abierto) |
| `LogSettings& getSettings()`, `void saveSettings() const` | Para el menú de la consola |
| `void suspend(bool)` | El menú está abierto — callar; al soltarlo — `refresh()` |
| `void setPaused(bool)`, `bool isPaused() const` | Pausa con la barra espaciadora; al soltarla — `refresh()` |
| `void refresh()` | El siguiente ciclo imprimirá todos los canales activados |

La lógica del canal (`updateChannel`):

- `Off` — no imprimir;
- `Periodic` — una vez por `periodMs()` (SYS — una vez cada 10 s), valores «tal cual»;
- `OnChange` — la línea se compone con **tolerancias** (el `Shown` anidado conserva
  el valor antiguo hasta que el nuevo se aleja más que la tolerancia: RC/PWM 3 µs, ángulos
  0.5°, rumbo 1°, correcciones 2, altitud 0.3 m, aceleración 0.03 g, coordenadas
  1e−5°) y se imprime solo si difiere de la última impresa.

Los tipos anidados: `LineBuffer : Print` (una línea de hasta 200 bytes para comparar antes de
imprimir), `Shown` (un valor con histéresis).

Los formatos de las líneas:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  objetivo 0.0 m
MAG  rumbo 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (el peor en 10 s) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=` distingue `LOST(sin tramas)` de `LOST(failsafe del mando)`; `IMU=` es
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK`.

---

## `DebugConsole`

**Archivo:** `telemetry/DebugConsole.h` · **Depende de:** `FlightController`, `FlightOutputs`, `Autopilot`, `DebugLogger`, `LogSettings`, `IBoard*` (sondeo de buses)

Un menú de texto en el monitor serie. Una máquina de estados de pantallas `Screen::{None, Main, Log}`.

| Método | Descripción |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | con placa — el comando `b` y el punto 7 del menú |
| `static const char* guessI2cDevice(uint8_t address)` | el chip según la dirección: 0x6A LSM6DSV, 0x68 MPU/ICM, 0x76 BME280/BMP388/SPL06, 0x46/0x47 BMP581, 0x7C QMC6309, 0x2C QMC5883P, 0x0D QMC5883L, 0x3C OLED |
| `void printHint() const` | Una pista de una línea |
| `void update()` | Procesa todos los bytes de `Serial`; si se cambiaron los ajustes del registro, el menú está cerrado y el avión **no está armed**, se guardan en NVS |

Teclas rápidas (fuera del menú): `h`/`?` — el menú principal; `l` — el menú del registro; espacio —
pausa del registro; `s` — estado de los sensores; `i` — calibración del giroscopio; `o` —
calibración del montaje de la IMU; `m` — calibración de la brújula; `p` — autocomprobación de las salidas;
`b` — sondeo de los buses I2C (0x08..0x7F — hasta 0x7F, porque el QMC6309 está en 0x7C) con
los nombres de los chips; cualquier otra — una pista. `\r`/`\n` se ignoran.

El menú del registro: `1`..`9` — cambian cíclicamente el modo de los canales 0..8, `n` — NAV, `s` — SYS, `p` — periodo, `a` —
todo «al cambiar», `x` — todo apagado, `d` — valores por defecto, `0`/`q` — volver, `l`/`h` —
cerrar.

Las acciones bloqueantes (`i`, `o`, `m`, `p`) están **prohibidas con ARM**. Mientras el menú
está abierto, el registro queda suspendido (`DebugLogger::suspend`). El ancho de los puntos del menú
se cuenta en caracteres UTF-8 y no en bytes (el cirílico ocupa 2 bytes).

---

## `WebDashboardPage`

**Archivo:** `telemetry/WebDashboardPage.h` · **Clase:** namespace

`static const char HTML[] PROGMEM` — la página entera (HTML + CSS + JS) en un único
literal. Todo lo dinámico lo construye el navegador a partir del JSON de `/api/status` (se consulta
cada 200 ms): las filas de canales, salidas y sensores se crean según las claves del JSON, así que una nueva
salida aparece sin tocar la página. Un campo PID que el usuario ha empezado
a editar ya no lo sobrescribe la consulta periódica.

---

## `WebDebugServer`

**Archivo:** `telemetry/WebDebugServer.h` · **Depende de:** `WebServer`, `WiFi`, `FlightController`, `Autopilot*`, `WebDashboardPage`, `Config`

| Método | Descripción |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | Un AP de Wi-Fi (`persistent(false)` — sin escribir en la flash), las rutas, la tarea `web` en el núcleo 0. `false` si el punto de acceso no se levantó |
| `void applyPendingCommands()` | Llamar desde el bucle de vuelo: toma los comandos bajo un spinlock y los aplica al autopiloto |

Las rutas:

| Ruta | Respuesta |
|---|---|
| `GET /` | La página del panel |
| `GET /api/status` | El JSON de estado (`buildStatusJson()`), el formato está en la [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}`; sin cuerpo → 400 `no data`; sin autopiloto → 503; modo incorrecto → 400 `invalid mode` |
| `POST /api/setpid` | Cualquiera de `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch`; los omitidos se quedan como están |
| lo demás | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }` — un buzón bajo un
`portMUX`. `extractJsonNumber(body, key, fallback)` — un análisis mínimo de
JSON plano sin ArduinoJson: `"key"`, espacios, `:`, espacios, un número en
cualquier notación JSON (signo, fracción, exponente `1e-7`); si falta la clave o el número —
`fallback`.

En el JSON los campos `attached`/`available` están **siempre**; los datos del sensor, solo
con `available: true`.

---

## `OledDisplay`

**Archivo:** `telemetry/OledDisplay.h` · **Depende de:** U8g2, `II2CBus`, `FlightController`, `Autopilot*`, `LoopStats`

Un SSD1306 128×64 (I2C 0x3C) en el segundo bus I2C; su propia tarea `oled`
(`Rtos::startTask`: núcleo 0 en el ESP32, prioridad baja en el STM32), cada 200 ms.

| Método | Descripción |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr` o la pantalla no responde en 0x3C → `false`; si no, configura U8g2 y lanza la tarea |

U8g2 envía los bytes mediante un `byteCallback` sobre `II2CBus` (la pantalla no sabe nada de
`Wire1`). Un callback de C no recibe contexto, así que el bus se guarda en una variable
estática `busSlot()` — a bordo hay una sola pantalla.

La pantalla:

```
RX ok ARM STAB FL       enlace (pérdida — línea invertida) / ARM / modo / flaps
R  +1.2 P  -0.4         alabeo / cabeceo, °           (IMU --)
Alt +0.3 Vz +0.1 A14    altitud / velocidad vertical / velocidad del aire, si hay tubo de Pitot (BARO --)
H123 T1000 Y1500        rumbo / gases / timón de dirección (H---)
L1500 R1500 E1500       alerones / timón de profundidad
Loop 500Hz max1100us    frecuencia y el peor ciclo en un segundo
```

Los nombres cortos de los modos son `AutopilotNames::modeShort()` (`MAN`, `STAB`,
`TKOFF`, `ALT`, `ACRO`, `CRZ`, `LOIT`, `RTH`, `LNCH`, `LAND`, `SOAR`, `RESQ`);
si se pierde el enlace en el aire — `GLIDE` o `FSRTH`.

---

## `BlackBox`

**Archivo:** `telemetry/BlackBox.h` · **Depende de:** `FlightController`, `Autopilot`, `LoopStats`, `BlackBoxStorage`, `PilotSwitches*`

La grabación del vuelo en la flash (ESP32-S3) o en una tarjeta SD (STM32H743). Qué descargar, cuándo y cómo — en [BLACKBOX.md](../BLACKBOX.md).

| Método | Descripción |
|---|---|
| `bool begin(bool startTask = true)` | Lee el soporte (`BlackBoxStorage::begin()`), reserva la cola (PSRAM en el ESP32, `malloc` en el STM32), verifica el espacio borrado (hasta 0.3 s), lanza la tarea `bbox` (`Rtos::startTask`). Si no hay sitio donde grabar (partición, tarjeta, archivo) — `false`, la caja negra queda apagada |
| `void update(uint32_t workUs)` | Desde `loop()` tras cada ciclo: eventos, inicio/parada, instantáneas a la cola, despierta la tarea de escritura |
| `void writerStep()` | Un paso de la tarea de escritura: una o dos páginas a la flash, o un borrado en tierra |
| `requestManualStart()` / `requestManualStop()` | Grabación manual (consola `k` → `r`) |
| `State getState()` / `bool isRecording()` | `Off`, `Idle`, `Recording`, `Stopping` (termina de escribir la cola antes de grabar END) |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | Para la consola |
| `void handleHostCommand(const char*)` | `bb list`, `bb get <n> [baudios]` — para `tools/blackbox.py` (por USB CDC la velocidad no influye en nada) |

Lo específico de cada plataforma: la causa del reinicio — `readResetCause()`; la tensión y la corriente de la batería —
el ADC (`analogReadMilliVolts` en el S3, un `analogRead` de 12 bits en el STM32); los errores del soporte
(`BlackBoxStorage::writeErrors`/`eraseErrors`) pasan al registro una vez por segundo
como un evento «soporte: errores de escritura …» y no estorban al vuelo.

## `BlackBoxStorage`

**Archivo:** `telemetry/BlackBoxStorage.h` · **Depende de:** `IFlashRegion`

Un anillo de sectores de 4 KB: la cabeza y la lista de vuelos salen de las cabeceras de los sectores en `begin()` (la primera pasada lee la cabecera de cada sector y recuerda las auténticas, la segunda solo esas: una zona vacía se lee una sola vez); `openFlight()`/`append()`/`flush()`/`closeFlight()` — escritura por páginas (un CRC-8 en cada registro); `eraseStep(target, protect, allowErase)` — un paso de verificación/borrado por delante de la cabeza: la basura — siempre, los vuelos — enteros y solo mientras haya menos de `target` libre; `protect` no se toca nunca.

## `BlackBoxRing`, `BlackBoxFormat`

`BlackBoxRing` es una cola de bytes de registros entre tareas/núcleos bajo `Rtos::CriticalSection`; si se desborda, descarta los más antiguos. `BlackBoxFormat` — la cabecera del sector, los tipos y las estructuras de los registros, las cadenas de los esquemas (el tamaño lo comprueba `static_assert`), CRC-8 y CRC-32.

---

## `Mavlink` (códec)

**Archivo:** `telemetry/MavlinkCodec.h` · **Clase:** namespace · **Depende de:** nada (portable)

MAVLink 2 sin la biblioteca generada: empaquetado de los campos en el orden de MAVLink
(contrastado con pymavlink), CRC-16/MCRF4XX + `CRC_EXTRA`, recorte de los ceros finales.

| Entidad | Descripción |
|---|---|
| `Msg::*` | identificadores: HEARTBEAT, SYS_STATUS, SET_MODE, PARAM_*, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW, MISSION_REQUEST_LIST/COUNT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS, REQUEST_DATA_STREAM, VFR_HUD, COMMAND_LONG/ACK, HOME_POSITION, STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | el `CRC_EXTRA` de un mensaje, −1 — desconocido |
| `crcAccumulate`, `crcCalculate` | X.25 (como `crc_accumulate()` de mavlink) |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` — los campos por orden |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` — una trama v2 con `seq` consecutivo |
| `Message` | un mensaje recibido: `msgid`, `sysid`, `compid`, la carga útil (rellenada con ceros), lectura de campos por desplazamiento |
| `Parser` | `bool feed(byte)` → `message()`; v1 y v2, la firma de v2 se omite; `goodCount()`, `badCrcCount()`; los mensajes con `CRC_EXTRA` desconocido se omiten en silencio |

## `MavlinkModes`

**Archivo:** `telemetry/MavlinkTelemetry.h` · **Clase:** namespace

| Función | Descripción |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | el número de modo de ArduPlane: MANUAL 0, STABILIZE→FBWA 5, ALT_HOLD→FBWB 6, ACRO 4, CRUISE 7, LOITER 12, RTH→RTL 11, AUTO_TAKEOFF/LAUNCH→TAKEOFF 13, AUTO_LAND→AUTO 10, SOARING→THERMAL 24, RESCUE→STABILIZE 2; failsafe → RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | a la inversa, para los comandos desde tierra; AUTO, CIRCLE, GUIDED — `false` |
| `isAutonomous(mode)` | la bandera `AUTO_ENABLED` en HEARTBEAT |

## `MavlinkTelemetry`

**Archivo:** `telemetry/MavlinkTelemetry.h` · **Depende de:** `IUartPort`, `FlightController`, `Autopilot*`, `LoopStats*`

Telemetría por módem de radio para QGroundControl / Mission Planner (el vehículo es
`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_ARDUPILOTMEGA`). Se usa en el STM32
(UART4), que no tiene Wi-Fi.

| Método | Descripción |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | abrir el puerto, el mensaje «OpenPlane online» |
| `void update()` | desde el bucle de vuelo: analiza lo que entra (≤ 128 bytes por ciclo), mensajes de eventos, no más de 2 tramas por ciclo |
| `void statusText(severity, text)` | al flujo de la GCS (una cola de 4 líneas, de hasta 50 caracteres) |
| `isGcsConnected()` | un HEARTBEAT de la GCS en los últimos 3 s |
| `getSentFrames()`, `getDeferredFrames()`, `getParser()` | diagnóstico |
| `static const char* paramName(uint8_t)` | `RLL_KP`, `RLL_KI`, `RLL_KD`, `PTCH_KP`, `PTCH_KI`, `PTCH_KD` |

Los flujos (Hz): ATTITUDE 10; GLOBAL_POSITION_INT, VFR_HUD 5; GPS_RAW_INT,
RC_CHANNELS, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT 2; HEARTBEAT, SYS_STATUS 1;
HOME_POSITION 0.2. Una trama se envía solo si `availableForWrite()` tiene sitio para ella
— si no, espera al siguiente ciclo (el bucle nunca se bloquea).

Entrante: el HEARTBEAT de la GCS; PARAM_REQUEST_LIST / READ / SET (el PID — directo al
autopiloto, valores 0..100, no se guardan); SET_MODE y COMMAND_LONG
`DO_SET_MODE` (176) — el modo hasta el siguiente cambio del interruptor; `COMPONENT_ARM_DISARM`
(400) — **DENIED**; `REQUEST_MESSAGE` (512) — un envío fuera de turno de un flujo;
MISSION_REQUEST_LIST — MISSION_COUNT 0 con el mismo `mission_type`.
Para comprobar el flujo con un decodificador externo — `tools/check_mavlink.py` (pymavlink).
