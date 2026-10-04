# CONTROL y COORDINATION — el mezclador, el acelerador, el ARM, las salidas, el orquestador

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/control.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referencia](README.md)

La capa CONTROL es lógica sobre datos, sin UART, PWM ni Wi-Fi.
`FlightController` (COORDINATION) es la única clase que reúne todas las capas
inferiores en un solo ciclo.

---

## `ControlCommand`

**Archivo:** `control/ControlCommand.h` · **Clase:** struct

La orden a las superficies en **signos físicos**, µs de deflexión (±500 =
recorrido completo). El lenguaje común de los sticks, el piloto automático y el
mezclador.

| Campo | «+» significa |
|---|---|
| `int16_t roll` | alabeo a la derecha (alerón derecho arriba, izquierdo abajo) |
| `int16_t pitch` | morro arriba (timón de profundidad arriba) |
| `int16_t yaw` | morro a la derecha (timón de dirección y rueda a la derecha) |
| `int16_t flaps` | flaps abajo (ambos alerones abajo); «−»: aerofreno (ambos arriba) |

Todos los campos valen 0 por defecto.

---

## `FlightOutputState`

**Archivo:** `control/FlightOutputState.h` · **Clase:** struct

Los pulsos deseados de las salidas, µs de PWM. Por defecto: superficies en
neutro y acelerador a `PWM_MIN`.

| Campo | Valor por defecto |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `aux1` | `PAYLOAD_CLOSED_US`: el lanzamiento de carga está cerrado |
| `aux2` | `PWM_CENTER`: la cámara |

---

## `FlapsController`

**Archivo:** `control/FlapsController.h` · **Depende de:** `Config`

Extensión y recogida suaves de los flaps: la posición avanza hacia el objetivo
(cualquier valor: flaps del interruptor, del potenciómetro, un aerofreno hacia
arriba) sin ir más deprisa que el recorrido completo `FLAPS_DEPLOYED_US` en
`FLAPS_TRANSITION_MS`. El tiempo se pasa como parámetro.

| Método | Descripción |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | Un paso hacia el objetivo; devuelve la posición actual, µs (+ abajo, − arriba) |
| `int16_t getPosition() const` | La posición actual |

Invariantes:

- **La primera llamada** pone la posición directamente en el objetivo: los
  flaps no «salen» sobre la mesa al encender.
- El paso de tiempo está limitado a `MAX_STEP_MS = 20`: tras una pausa larga
  (failsafe, calibración) los flaps no saltan al objetivo en un solo ciclo.

---

## `ControlMixer`

**Archivo:** `control/ControlMixer.h` · **Depende de:** `RcInput`, `RcChannelState`, `FlapsController`, `ControlCommand`, `FlightOutputState`, `Config`, `Channels`

La lógica aerodinámica en dos pasos. Es propietario de `FlapsController`.

| Método | Descripción |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll` (2000 = a la derecha); CH2 → `pitch` **con el signo contrario** (2000 = hacia delante = morro abajo); CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | el objetivo de los flaps lo elige `FlightController` (aerofreno → interruptor de flaps → potenciómetro); aquí está el recorrido suave |
| `FlightOutputState mix(const ControlCommand& c) const` | Orden → PWM. El alabeo, el cabeceo y la guiñada se limitan por el recorrido (`*_MAX_US`); alerones: izquierdo = `flaps + roll`, derecho = `flaps − roll` (abajo = «+»); PWM = `1500 ± deflexión` con el signo de `Config::*_REVERSED`, limitado a 1000..2000. `throttle` no se rellena |
| `int16_t getFlaps() const` | La posición actual de los flaps, µs |

Flaperones: al extender, ambos alerones bajan `FLAPS_DEPLOYED_US` (el nuevo
«neutro») y el alabeo actúa por encima. Con alabeo completo, el alerón que baja
llega al final de su recorrido antes que el que sube; esto funciona como un
diferencial de alerones.

---

## `ThrottleManager`

**Archivo:** `control/ThrottleManager.h` · **Depende de:** `RcInput`, `RcChannelState`, `Config`, `Channels`

| Método | Descripción |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | El acelerador de CH3, limitado a 1000..2000; al perder el enlace: `FAILSAFE_THROTTLE` |

No sabe nada del ARM ni del piloto automático: sus correcciones las aplica
`FlightController`.

---

## `ArmingManager`

**Archivo:** `control/ArmingManager.h` · **Depende de:** `Autopilot` (puede ser nulo), `RcChannelState`, `Config`, `Channels`

El ARM con un interruptor aparte, SwA (CH5). La máquina de estados está en
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Método | Descripción |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Sin piloto automático solo se comprueba el acelerador |
| `void update(const RcChannelState& rc, bool receiverFailsafe)` | En failsafe: nada (el interruptor en una trama de failsafe no refleja al piloto). Interruptor OFF → DISARM, `switchSeenOff = true`. Una transición OFF→ON → comprobaciones → ARM o rechazo |
| `bool isArmed() const` | Armado |
| `const char* getLastRefusalReason() const` | El motivo del último rechazo o `nullptr`; se borra al apagar el interruptor |

`checkFailureReason(rc)`: las comprobaciones del ARM:

| Condición | Motivo del rechazo |
|---|---|
| Acelerador ≥ `THROTTLE_LOW_US` | «acelerador no al mínimo» |
| Cualquier modo salvo MANUAL, la IMU existe pero no responde | «la IMU no responde…» |
| Cualquier modo salvo MANUAL, la IMU tiene un problema en la comprobación prevuelo | el texto de `ImuSensor::getPreflightProblem()` |
| Un modo con altitud (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), el barómetro existe pero no responde | «el barómetro no responde…» |

Un sensor que no está en la compilación (`nullptr`) no bloquea el ARM; en
MANUAL el aparato se arma incluso sin ningún sensor. El fix del GPS no figura
entre las comprobaciones a propósito: sin GPS los modos de navegación se
comportan de forma segura (un círculo en el sitio), y el punto de origen se
registra cuando el GPS capta satélites.

Invariantes: encender la placa con el interruptor en ON no arma; un intento por
cada transición OFF→ON; la pérdida de enlace no anula el ARM.

---

## `FlightOutputs`

**Archivo:** `control/FlightOutputs.h` · **Depende de:** `IBoard`, `FlightOutputState`, `Config`

La única clase que conoce el conjunto y el orden de las salidas PWM. Todas las
salidas están descritas en una tabla; `begin()`, `write()`, el estado y la
autocomprobación la recorren en un bucle.

### `FlightOutputs::OutputInfo`

| Campo | Descripción |
|---|---|
| `const char* key` | El nombre en el JSON/registro (`aileronLeft`, …, `esc`, `rudder`, `aux1`, `aux2`) |
| `const char* label` | El nombre para las personas |
| `int16_t pin` | El número de pin; `-1`: no cableado. `int16_t`, porque en la STM32 los números de los pines analógicos son `0xC0 + N` |
| `bool required` | Sin ella el aparato no vuela (el timón de dirección es opcional) |
| `uint16_t FlightOutputState::* field` | Un puntero al campo del estado |

| Método | Descripción |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | Una fila de la tabla; el orden = `ServoChannel` |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` de cada salida, imprime el estado; `true` si todas las **obligatorias** obtuvieron un canal |
| `bool isAttached(uint8_t ch) const` | La salida está conectada (un índice fuera de rango → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | El valor de la salida a partir del estado, según la tabla |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | El pulso medido frente al esperado en cada pin cableado; «OK» si la discrepancia es ≤ 15 µs |
| `void write(const FlightOutputState&)` | Escribir todas las salidas y recordar el estado |
| `void setFailsafe()` | Superficies en neutro (`FAILSAFE_*`), acelerador `FAILSAFE_THROTTLE`; AUX como estaban (la carga no se lanza por perder el enlace) |
| `void setBuzzer(bool on)` | el zumbador de la placa (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | El último estado escrito |

Para añadir una salida: una fila en la tabla + un campo en
`FlightOutputState` + un índice en `ServoChannel` (+ un pin y un canal LEDC en
`Esp32Board`).

---

## `FlightController`

**Archivo:** `control/FlightController.h` · **Capa:** COORDINATION ·
**Depende de:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`

El único coordinador del ciclo de control: no analiza la UART, no toca el PWM
ni calcula el mezclador; solo llama a los demás en el orden correcto. El
diagrama detallado está en [ARCHITECTURE.md §6](../ARCHITECTURE.md#6-el-ciclo-de-control-flightcontrollerupdate).

| Método | Descripción |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | Sin piloto automático: control manual puro; sin interruptores: solo los sticks |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | Un ciclo (véase más abajo) |
| `bool isReceiverFailsafe() const` | El enlace está perdido |
| `const IBusReceiver& getReceiver() const` | Para el registro (contadores de tramas, la causa de la pérdida) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | Lo último escrito en las salidas |
| `const RcChannelState& getRcState() const` | Los canales |
| `const FlightOutputs& getOutputs() const` | La tabla de salidas y `attached` |
| `int16_t getFlapsUs() const` | La posición de los flaps |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | Los interruptores y potenciómetros de este ciclo |
| `bool isLostModelBeeping() const` | El zumbador de «estoy aquí» está sonando |

El orden de `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. con el enlace vivo: `switches->update(rc)` (el modo, las funciones, los potenciómetros);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. los sticks `mixer.fromSticks(rc)` (con el enlace vivo) × `Knob::RATES`; los
   flaps `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → de forma suave, al perder el enlace: 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)`: **siempre**;
6. el zumbador: `Beeper::update(BEEPER, armed, failsafe, now)`;
7. enlace perdido → `applyLinkLoss()` y salida del ciclo;
8. `arming.update(rc, false)`;
9. `command = autopilot->getCommand()` (o los sticks sin piloto automático), los flaps, los suyos;
10. `output = mixer.mix(command)`; `output.throttle = autopilot->applyThrottle(pilotThrottle)`;
11. no armado o `MOTOR_KILL` → `throttle = PWM_MIN` (lo último);
12. AUX1: la carga (`PAYLOAD_DROP`), AUX2: la cámara (`Knob::CAMERA_TILT`, `CAMERA_STAB` resta el cabeceo);
13. `outputs.write(output)`.

`applyLinkLoss()`: si el piloto automático está en failsafe (armado: RTH o
planeo), las superficies y el acelerador siguen la orden del piloto automático
(los flaps se recogen suavemente, `MOTOR_KILL` sigue apagando el motor, AUX
como estaban); si no, `outputs.setFailsafe()`.

---

## `Beeper`

**Archivo:** `control/Beeper.h` · **Depende de:** `Config`

| Método | Descripción |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | el estado del zumbador: 2 Hz si hay `Feature::BEEPER` o «modelo perdido» (no armado, sin enlace durante más de `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | el modo «búsqueme en la hierba» |
