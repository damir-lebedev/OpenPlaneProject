# OpenPlaneProject — hoja de ruta y presentación para inversores y socios

> 🌐 Esta página es una traducción del [original en ruso](../../ROADMAP.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual.

> Repositorio: [github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject), rama `main`.
> Este documento es una versión más profunda del avance que ofrece el README, dirigida a quienes estudian invertir dinero,
> tiempo o una alianza en el proyecto. Describe dónde se encuentra el proyecto ahora, hacia dónde va y por qué,
> y qué es exactamente lo que, en el código ya escrito, hace que esta trayectoria sea realista y no meramente declarada.

## 1. Estado actual, con honestidad

OpenPlaneProject es hoy un prototipo de planeador que ya ha volado, pero todavía inmaduro, con su propio firmware para el ESP32; no es un producto terminado ni un dron autónomo. Hardware: 1200 mm de envergadura, 250 mm de cuerda, perfil NACA 4412, estructura de PETG (impresa en 3D), servos MG90S (uno aparte para cada alerón), alimentación con LiPo 3S. El primer prototipo (ESP32-C3, motor D2212 1000KV, ESC de 40A) ya voló con control manual y, tras el vuelo, dejó al descubierto problemas concretos: la sujeción del motor y del ala no es lo bastante resistente (hay que reforzarla con carbono) y hace falta ajustar los servos.

El montaje actual ha pasado al ESP32-S3 (N16R8) con un motor D3548 1100KV y un ESC de 60–80A, y en el banco tiene conectados todos los sensores del piloto automático: una IMU (MPU6500), un barómetro BMP388, una brújula QMC5883P y una pantalla OLED de estado. Comprobado en vivo: control manual por RC mediante iBUS con un mezclador de alerones, timón de profundidad, timón de dirección (con la rueda direccional) y flaps (flaperones); ARM con un interruptor aparte; failsafe con la emisora apagada; un ciclo de control de 500 Hz; un dashboard web en vivo por Wi-Fi. Sobre la mesa, la estabilización responde a las inclinaciones en el sentido correcto.

Desde entonces el firmware ha ganado 12 modos de piloto automático (estabilización, mantenimiento de altitud, crucero, círculos y regreso a casa por GPS, lanzamiento a mano, aterrizaje automático, vuelo a vela en térmicas, «rescate»), geovalla, regreso a casa al perder la señal, lanzamiento de carga, un tubo de Pitot formado por dos barómetros, compatibilidad con sensores nuevos (LSM6DSV, ICM-45686, QMC6309, SPL06, BMP581) y un firmware completo para el STM32H743 con telemetría MAVLink para QGroundControl. Todo esto está comprobado con 387 pruebas automáticas y simulaciones de vuelo en lazo cerrado (todo el firmware pilota un modelo de avión), pero **todavía no se ha probado en el aire**. Dicho de otro modo: hasta ahora solo ha volado el control manual; el piloto automático está escrito, comprobado con todo lo que se puede usar para comprobarlo sin volar, y espera las pruebas de vuelo.

## 2. Por qué importa

La aviación pequeña autónoma con una barrera de entrada baja cubre tareas en las que lo decisivo no es la carga útil ni el alcance máximos, sino la rapidez de reacción y el bajo coste de operación:

- **Entrega de medicamentos y sangre a zonas de difícil acceso y a zonas tras una catástrofe**: caminos arrasados, falta de infraestructura, destrucción tras desastres naturales o conflictos. Aquí lo decisivo no es la capacidad de carga (el paquete es pequeño), sino el tiempo de reacción: minutos y horas en lugar de un día en coche o a pie.
- **Operaciones de búsqueda y rescate**: lanzamiento puntual de equipo, botiquines, medios de comunicación y medios de flotación a los afectados antes de que llegue el grupo terrestre, en zonas donde el helicóptero es excesivamente caro o inaccesible por el clima o el terreno.
- **Agricultura de precisión**: monitoreo de cultivos y pulverización o aplicación localizada de productos donde las plataformas de drones cerradas para estas tareas cuestan miles de dólares, lo que no resulta rentable para las explotaciones pequeñas y medianas.

En las tres categorías la economía es la misma: la diferencia entre «hay una solución, pero es cara y cerrada» y «en la práctica no hay solución, porque es cara»; y es justo a ese nicho al que apunta una plataforma abierta y barata. Esto es una descripción del ámbito de aplicación y del problema de mercado, no una afirmación de que OpenPlaneProject ya sepa entregar cargas: en la etapa actual es una afirmación sobre el objetivo y sobre por qué merece la pena.

## 3. Tesis de inversión: por qué una arquitectura abierta sobre ESP32 es una asimetría

Las plataformas comerciales de drones autónomos para entrega o monitoreo suelen construirse sobre controladores de vuelo cerrados y software cerrado, cuestan de cientos a miles de dólares por aeronave y exigen regalías por licencia o contratos de servicio para operar la flota. OpenPlaneProject parte de otro supuesto:

- **Una base barata.** Un módulo ESP32 cuesta del orden de 5–15 USD, y el resto (servos MG90S, ESC, receptor iBUS) son componentes estándar de nivel aficionado. Es un orden de magnitud más barato que la entrada a las plataformas comerciales cerradas, algo crítico para despliegues piloto en condiciones de presupuesto limitado (ONG, agricultura a pequeña escala, servicios regionales de rescate).
- **El código abierto cambia la economía de la confianza.** Una organización que despliega una flota para entregas médicas puede auditar la seguridad (failsafe, lógica de ARM) y adaptar el firmware a sus propios sensores y reglamentos, en lugar de depender de un único proveedor y de su hoja de ruta.
- **La arquitectura ya hoy está diseñada para ampliarse, y no solo para el planeador actual.** No es una declaración, sino una consecuencia directa de cómo está hecho el código:
  - Un sensor nuevo se añade como una clase que implementa la interfaz existente `Sensor` → `ImuSensor`/`BarometerSensor` (`include/sensors/SensorInterface.h`), sin tocar el núcleo. Así se hicieron ya los drivers de IMU (MPU6050/MPU6500, ICM-42688), de barómetros (BMP388, BME280), de brújulas (QMC5883P/L) y de GPS (u-blox M10): todos escritos directamente sobre los registros del bus (interfaces `II2CBus`/`ISpiBus`/`IUartPort`), sin bibliotecas de terceros, es decir, sin dependencias ocultas de un SDK concreto de un fabricante.
  - Una placa nueva se añade con un solo bloque `#elif` en `include/config/Config.h` más un solo bloque `[env:...]` en `platformio.ini`: el cambio de placa ya funciona hoy para cuatro objetivos (véase la tabla de abajo); no es una posibilidad hipotética.
  - `Autopilot.h` ya recibe `ImuSensor*`/`BarometerSensor*` como parámetros (pueden ser `nullptr`); es decir, el contrato entre el piloto automático y el hardware está pensado para que la composición de los sensores cambie (el siguiente paso es el GPS como una clase más con el mismo patrón, véase la Fase 3).
  - `WebDebugServer.h` ya entrega un único JSON agregado (`GET /api/status`) y acepta comandos (`POST /api/setmode`, `/api/setpid`); es decir, el protocolo «la aeronave entrega telemetría y acepta comandos» ya existe, y la estación de tierra debe crecer a partir de él, no escribirse desde cero.

La asimetría está en que la barrera de entrada (dinero, tiempo de adaptación a una tarea nueva) de esta plataforma es un orden de magnitud más baja que la de los análogos cerrados y, además, el camino hacia la autonomía no exige reescribir el núcleo, solo añadir clases nuevas sobre las interfaces existentes. Es una plataforma de ingeniería abierta, no un producto comercial terminado; por eso la tesis para un inversor o socio no es «compre una solución lista», sino «entre en una etapa en la que los cimientos ya están comprobados y los siguientes pasos están técnicamente claros».

## 4. La arquitectura hoy: la base de las fases siguientes

### 4.1 Placas compatibles

La elección de la placa es una sola opción de compilación de PlatformIO; cambiar de placa no obliga a tocar la lógica (`include/config/Config.h` + `platformio.ini`):

| Entorno (`pio run -e ...`) | Placa | Estado | aileron L / R | elevator | esc | ibus_rx | i2c sda / scl |
|---|---|---|---|---|---|---|---|
| `esp32-s3` (default) | ESP32-S3 N16R8 (DevKitC-1) | **Principal, comprobada en el banco con todos los sensores** | GPIO4 / GPIO5 | GPIO6 | GPIO7 | GPIO17 | GPIO41 / GPIO42 |
| `esp32-c3` | ESP32-C3 SuperMini | Primer prototipo, voló con control manual | GPIO5 / GPIO4 | GPIO6 | GPIO7 | GPIO8 | GPIO1 / GPIO3 |
| `esp32-dev` | ESP32 clásica de 38 pines | Para el banco, **no comprobada en hardware** (el firmware completo está en las pruebas) | GPIO13 / GPIO14 | GPIO27 | GPIO26 | GPIO16 | GPIO21 / GPIO22 |
| `stm32h743` | STM32H743VIT6 (WeAct Mini) | Firmware completo + MAVLink, **todavía sin placa** (el firmware completo corre en las pruebas en el PC) | PA0 / PA1 | PA2 | PA3 | PE7 | PB11 / PB10 |

Cargar el firmware: `pio run -t upload`. Monitor: `pio device monitor` (115200).

### 4.2 Mapa de canales RC (FS-i6 + FS-iA6B, iBUS, 10 canales, 1000–2000 µs)

| Canal | Nombre | Función por defecto |
|---|---|---|
| CH1–CH4 | sticks | alabeo, cabeceo, acelerador, timón de dirección |
| CH5 | ARM | interruptor SwA: ARM con el acelerador abajo, DISARM al instante |
| CH6 | SWB | flaps |
| CH7 | SWC | modo: MANUAL / STABILIZE / AUTO_TAKEOFF |
| CH8 | SWD | RTH: regreso a casa |
| CH9 | VRA | intensidad de la estabilización |
| CH10 | VRB | velocidad de crucero |

CH6–CH10 se asignan con una sola línea en `include/config/Controls.h`: cualquiera de los 12 modos, 10 funciones (flaps, freno, lanzamiento de carga, geovalla…) y 7 potenciómetros — [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

### 4.3 Flujo de control (`FlightController::update()`)

Un único orquestador, con un orden fijo y un ciclo de 500 Hz de periodo fijo: recepción de iBUS → acelerador del piloto → modo desde CH7 → **lectura de los sensores y cálculo del piloto automático (siempre, incluso sin enlace)** → **comprobación de la pérdida de señal con prioridad absoluta** (en el aire: a casa por GPS con el motor, o planeo con las alas niveladas si no hay GPS; en tierra: superficies al neutro) → ARM → orden de los sticks + correcciones del piloto automático en signos aeronáuticos unificados → mezclador con inversión de servos → acelerador del modo → bloqueo del acelerador sin ARM → escritura en los servos/ESC. Es precisamente esta disciplina de orden (primero la seguridad, luego el control manual, luego el piloto automático como capa superior) la razón por la que se puede incorporar con seguridad un comportamiento cada vez más autónomo sin reescribir el ciclo básico. El Wi-Fi, el dashboard y la pantalla funcionan en el segundo núcleo y no retrasan el control.

### 4.4 El dashboard web como germen de la estación de tierra

`WebDebugServer.h` ya hoy levanta un punto de acceso (SSID `OpenPlane-Debug`, IP `192.168.4.1`) y entrega y acepta JSON:

| Método y ruta | Qué hace |
|---|---|
| `GET /api/status` | Un único JSON agregado: RC (10 canales), armed/failsafe, 7 salidas (`us`, `attached`), IMU, barómetro, brújula, GPS, tubo de Pitot (cada uno con `attached`/`available` + datos), piloto automático (modo, correcciones, PID, navegación, funciones activadas) |
| `POST /api/setmode` | `{mode: 0-11}`: cambiar el modo del piloto automático |
| `POST /api/setpid` | `{kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch}`: cualquier campo es opcional |
| `GET /` | El dashboard HTML: barras en vivo de los 10 canales, el estado de cada salida y sensor, botones de modos, un formulario de PID |

Los campos `attached`/`available` siempre están presentes en el JSON: el dashboard distingue con honestidad «no está en la configuración» de «está en la configuración, pero no responde», en lugar de callar sobre un sensor ausente. Es el mismo principio de honestidad que sustenta todo el documento: no hacer pasar lo deseado por lo existente.

## 5. Hoja de ruta técnica por fases

A continuación hay ocho fases, cada una descrita como el siguiente paso lógico sobre las clases que ya existen, sin fechas ni cifras inventadas.

### Fase 1 — Control manual y una base segura [hecho]

**Objetivo:** un planeador radiocontrolado fiable y predecible, con una depuración transparente. **Qué hay ya técnicamente:** el análisis de iBUS con detección de pérdida de señal en `IBusReceiver.h`, el mezclador `ControlMixer.h` (sticks → orden de alabeo/cabeceo/flaps → PWM con inversión de servos, sin saber nada de UART/PWM), `ThrottleManager.h`, `ArmingManager.h` (ARM con un interruptor aparte con el acelerador abajo, DISARM instantáneo), el failsafe con prioridad absoluta en `FlightController.h`, la salida por Serial (`DebugLogger.h`), el dashboard web (`WebDebugServer.h`) y la pantalla OLED (`OledDisplay.h`). **Por qué esta es la base de todo lo demás:** es la única capa que debe funcionar siempre, incluso si las demás fases aún no están implementadas o sus sensores están desconectados; precisamente por eso el failsafe y el ARM se escribieron primero y se comprobaron en el hardware (incluido el comportamiento real del receptor FS-iA6B con la emisora apagada).

### Fase 2 — IMU + barómetro → piloto automático [escrito y comprobado con pruebas y simulación, a la espera de las pruebas de vuelo]

**Objetivo:** el primer modo de vuelo autónomo: estabilización del horizonte, despegue automático, mantenimiento de altitud. **Qué hay ya técnicamente:** `imu/MPU6050_Sensor.h` (MPU6050 y MPU6500, registros directamente, giro de los ejes según el montaje de la placa, signos aeronáuticos, filtro complementario en `ImuSensorBase`), `baro/BMP388_Sensor.h` (I2C o SPI, compensación completa de Bosch, lectura según el indicador de dato listo, velocidad vertical filtrada), `Autopilot.h` con `PidController` (el término D a partir del giroscopio, el integrador solo acumula tras el ARM) y doce modos (de MANUAL a SOARING y RESCUE), que se conmutan con interruptores según la tabla de `Controls.h`, desde el dashboard web y desde QGroundControl, además del failsafe (a casa o planeo). Cada modo vuela en una simulación en lazo cerrado de todo el firmware con un modelo del avión (`test/native/test_sim`). En el banco con el ESP32-S3 todos los sensores responden y los signos se han comprobado en vivo: inclinación → corrección de las superficies hacia la nivelación. **Qué hace falta para cerrar la fase:** llevar la electrónica al planeador, comprobar los sentidos de las superficies en la aeronave montada y hacer las primeras pruebas de vuelo, empezando por STABILIZE a una altitud segura.

### Fase 2.5 — Realimentación a partir del avión real [base preparada, comprobada en simulación]

**Objetivo:** que el piloto automático no dependa de coeficientes ajustados para una sola velocidad, sino de cómo responde el avión real al mando en este mismo instante. El PID de la fase 2 deflecta la superficie «por fórmula» y no comprueba el resultado; a baja velocidad corrige de menos, a alta velocidad se pasa. **Qué hay ya técnicamente** (`include/autopilot/feedback/`, **sin conectar** al firmware): la estimación de la eficacia de las superficies en vuelo (mínimos cuadrados recursivos, reescalado con la velocidad ∝ V²), el regulador «ángulo → velocidad de giro → superficie» con corrección complementaria («la superficie no llegó hasta el final: girarla más»), la protección contra la pérdida de velocidad y la entrada en pérdida (acelerador, morro abajo, alas niveladas), el despegue desde pista o a mano y el aterrizaje por etapas según los sensores. Todo se ha comprobado con una simulación en lazo cerrado del avión en la propia placa (`pio test -e esp32-s3 -f test_feedback`, 10 escenarios), incluidos un alerón con el sentido invertido, turbulencia, el morro alto con poco acelerador, el despegue y el aterrizaje. **Qué hace falta para cerrar la fase:** tras los primeros vuelos de la fase 2, un «modo sombra» (la realimentación solo escribe en el registro lo que habría hecho), luego la conexión eje por eje, un sensor de velocidad aerodinámica (tubo de Pitot) y un telémetro para nivelar antes del toque. Detalles: la sección «Realimentación» de [`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md).

### Fase 3 — GPS y brújula [en el firmware, comprobado con simulación]

**Estado (actualizado): la navegación funciona**: rumbo por GPS/brújula/giroscopio con histéresis, punto de origen al hacer ARM, CRUISE, LOITER (un campo vectorial hacia una circunferencia), RTH, geovalla; comprobado con simulaciones en lazo cerrado. A continuación, el registro original de la fase. El GPS (u-blox M10, `sensors/gps/UbloxM10_Gps.h`, protocolo UBX-NAV-PVT) y el magnetómetro (dos variantes de la placa «GY-273»: QMC5883P, `sensors/mag/QMC5883P_Sensor.h`, que está en el banco actual y se ha comprobado en vivo; QMC5883L, `sensors/mag/QMC5883L_Sensor.h`) se añadieron como clases nuevas que implementan las interfaces `GpsSensor`/`MagnetometerSensor` (`include/sensors/SensorInterface.h`) con el mismo principio que la IMU y el barómetro: `Autopilot.h` y `FlightController.h` no se reescribieron, y recibieron dos fuentes de datos más de la misma manera (un puntero anulable en el constructor). De paso apareció la capa HAL (`include/hal/`), a través de la cual los sensores acceden a los buses: I2C/SPI/UART ya no dependen directamente de los `Wire`/`SPI`/`HardwareSerial` específicos del ESP32.

Por ahora los datos del GPS y de la brújula están disponibles mediante `Autopilot::getGpsSensor()`/`getMagnetometerSensor()` y en `GET /api/status`, además de un ajuste único del yaw inicial por la brújula al arrancar, **pero no intervienen en el control**. Cuestiones abiertas para terminar la fase: conectar el módulo GPS al ESP32-S3 (los pines de UART2 ya están reservados), calibrar la brújula en la aeronave montada y añadir al rumbo la compensación de inclinación. En el ESP32-C3 un GPS completo es imposible: faltan GPIO para TX (véase la distribución de pines en `DEVELOPER_GUIDE.md`).

### Fase 4 — Vuelo por puntos de ruta (waypoint navigation) [siguiente paso]

**Estado:** las primitivas están listas: `Guidance::rollForCourse`, un círculo alrededor de un punto, el regreso a un punto (RTH), un canal MAVLink para cargar la misión (ahora, ante una solicitud de misión, la aeronave responde con honestidad «0 puntos»). Queda por hacer: el almacenamiento de la ruta, el paso de un punto a otro y el protocolo MISSION_* de MAVLink.

**Objetivo:** la aeronave vuela por un conjunto dado de coordenadas sin intervención del operador en cada tramo de la ruta. **Cómo encaja esto en la arquitectura:** es un nuevo `AutopilotMode` en `Autopilot.h`, a la par de los ya existentes MANUAL/STABILIZE/AUTO_TAKEOFF/ALT_HOLD; es decir, el mecanismo de conmutación de modos (mediante las ranuras de RC y mediante `POST /api/setmode`) no cambia, y se añade un quinto modo que toma el rumbo y la distancia del GPS (Fase 3) en lugar de la entrada manual por RC. **Qué hace falta técnicamente:** un algoritmo para calcular el rumbo hacia un punto y la lógica del paso entre los puntos de la ruta, además de una forma de cargar la propia ruta en la aeronave (el candidato natural es ampliar la misma API HTTP con la que ya se controlan los modos y el PID).

### Fase 5 — Telemetría de largo alcance [hecho en el firmware de la STM32]

**Estado:** en la STM32H743, MAVLink 2 por radiomódem (SiK, ELRS en modo MAVLink): actitud, posición, velocidad, modo, parámetros del PID, cambio de modo desde tierra. Las tramas se han contrastado con la referencia pymavlink. En el ESP32 no hay ningún UART libre: allí se usa el dashboard por Wi-Fi. A continuación, el registro original de la fase.

**Objetivo:** un enlace aeronave↔tierra a las distancias propias de una entrega real, no del banco. **Una valoración honesta del estado actual:** el punto de acceso Wi-Fi de `WebDebugServer` ya hoy transmite el estado agregado completo y los comandos de control, pero el alcance de un punto de acceso Wi-Fi corriente es de decenas de metros, lo que basta para depurar sobre la mesa o en un aeródromo, pero no para una ruta autónoma más allá de la línea de visión. **Qué hace falta técnicamente:** un canal de radio aparte, de mayor alcance (por ejemplo, un módulo LoRa o un radiomódem de telemetría especializado), como transporte del mismo formato de datos que ya está definido en `GET /api/status`; es decir, sustituir o complementar la capa de transporte, no reescribir el formato de la telemetría.

### Fase 6 — Una GUI completa de control en tierra [en parte: QGroundControl / Mission Planner]

**Estado:** gracias a MAVLink, las estaciones de tierra estándar ya ven la aeronave (mapa, punto de origen, instrumentos, modos con los nombres de ArduPlane). Una estación propia para una flota sigue siendo un objetivo. A continuación, el registro original de la fase.

**Objetivo:** una estación de planificación de misiones con mapa, telemetría en vivo y gestión de flota, y no una página de depuración de una sola aeronave. **Cómo encaja esto en la arquitectura:** `WebDebugServer.h` ya hoy no es un esbozo, sino un servidor web que funciona, con un estado JSON agregado y una API de comandos (véase la tabla de la sección 4.4); es el punto de partida, no algo que habrá que tirar. Los siguientes pasos son un mapa con la posición actual (tras la Fase 3), mostrar y cargar una ruta (tras la Fase 4), trabajar sobre un canal de radio de largo alcance (tras la Fase 5) y escalar la interfaz de una aeronave a varias. Más detalles, en la sección 6.

### Fase 7 — Mecanismo de lanzamiento de carga y medidas de protección para la entrega [hecho en el firmware, aún sin volar]

**Estado:** lanzamiento de carga (servo AUX1, la función `PAYLOAD_DROP` en cualquier interruptor), geovalla (radio y techo → RTH), regreso a casa al perder la señal, el zumbador de «modelo perdido». A continuación, el registro original de la fase.

**Objetivo:** convertir la plataforma de «un avión que vuela de forma autónoma» en «un avión que entrega de forma autónoma». **Qué hace falta técnicamente:** un servo adicional para el mecanismo de lanzamiento o dispensado de la carga, controlado con el mismo principio que las demás salidas de `FlightOutputs.h`; y medidas de protección propias de la entrega y no del vuelo neutro: geovallas (límite del área de vuelo) y regreso automático al punto de salida al perder la señal (hoy, al perder la señal en el aire, `FlightController` apaga el motor y pasa a planear con las alas niveladas, lo cual es correcto para un planeador manejado a mano, pero para la entrega autónoma el siguiente paso lógico es volver a la base por GPS en lugar de limitarse a planear).

### Fase 8 — Escala a una flota

**Objetivo:** gestionar varias aeronaves a la vez: despacho de tareas, un panel de operaciones, historial de vuelos. Es el nivel en el que el proyecto deja de ser solo un prototipo de ingeniería a medio camino con la afición y se convierte en una herramienta operativa, interesante como problema de negocio: planificación de rutas para varias aeronaves, una cola de tareas, el estado de cada aeronave en tiempo real. Técnicamente es una superestructura sobre las Fases 3–6 (GPS, telemetría, GUI): en esencia, la misma API de `WebDebugServer`, pero ampliada a muchas fuentes de telemetría en lugar de una.

## 6. GUI: de una página de depuración a una estación de tierra

La tesis clave de esta sección: no hace falta construir la GUI desde cero, porque ya existe en parte y funciona. Hoy `WebDebugServer.h`:

- entrega una única instantánea JSON agregada del estado de la aeronave (`GET /api/status`): los canales RC, los indicadores armed/failsafe, el estado de cada salida, el estado de cada sensor (con honestidad, mediante indicadores separados `attached` y `available`) y el estado del piloto automático;
- acepta comandos de control en tiempo real (cambio de modo, edición del PID) sin volver a cargar el firmware;
- entrega un dashboard HTML ya hecho, con barras de canales en vivo y botones de control.

El camino hasta una estación de tierra completa es una ampliación sucesiva del protocolo que ya funciona, no un cambio de arquitectura:

1. Añadir al dashboard un mapa y la posición actual: requiere el GPS (Fase 3) como un campo más del mismo estado JSON.
2. Añadir la construcción y la carga de rutas: requiere un modo de puntos de ruta (Fase 4) y ampliar la API POST de manera análoga a `/api/setmode`/`/api/setpid`.
3. Trasladar el transporte del punto de acceso Wi-Fi a un enlace de largo alcance (Fase 5), conservando el mismo formato de mensajes para no tener que reescribir el frontend existente.
4. Escalar la interfaz de una aeronave a varias fuentes de telemetría (Fase 8).

Dicho de otro modo, el elemento de la futura estación de tierra que más riesgo tenía en cuanto a «si habrá que escribirlo desde cero» (la serialización del estado de la aeronave y la API de comandos) ya está implementado y comprobado en vivo en la placa.

## 7. Limitaciones, con honestidad: lo que todavía no funciona

Para que la hoja de ruta no parezca marketing, dejamos constancia aparte de lo que aún no está hecho o está hecho solo en parte:

- El piloto automático se ha comprobado en el banco, con 387 pruebas automáticas y con simulaciones en lazo cerrado, pero nunca se ha probado en vuelo: hasta ahora solo ha volado el control manual (en el primer prototipo). El modelo del avión en las simulaciones es simplificado y los coeficientes son valores iniciales.
- Los sensores nuevos (LSM6DSV, ICM-45686, QMC6309, SPL06, BMP581) se han comprobado con emuladores de registros hechos a partir de las hojas de datos; en hardware, todavía no.
- STM32H743: todo el firmware corre en un PC sobre fakes de STM32duino; todavía no hay una placa física.
- El horizonte de la estabilización es la posición de la aeronave al encender (la IMU se calibra en cada arranque), o bien la calibración del montaje a partir de tres posiciones.
- La conexión física de un servo no es visible para el software; solo se ve que el pulso sale de verdad por el pin (una autocomprobación desde la consola).
- La distribución de pines de la ESP32 corriente de 38 pines se eligió a partir de la documentación del chip y no se ha comprobado en hardware.
- La licencia es la [OpenPlane License](LICENSE.md): MIT con atribución obligatoria del autor y con prohibiciones del uso militar y del daño intencionado a personas y bienes sin su consentimiento. Por estas prohibiciones no se considera «de código abierto» en el sentido de la OSI.

## 8. Preguntas abiertas: una invitación a debatir

A continuación figuran los puntos sobre los que el proyecto aún no tiene respuesta, formulados a propósito como preguntas para un posible socio o inversor y no como hechos resueltos:

- El modelo de financiación y su volumen: se puede hablar; por ahora no hay cifras ni plazos concretos, y en este documento no se inventarán.
- La forma jurídica del proyecto (una empresa, una fundación, una comunidad puramente de código abierto): abierta al debate con quienes estén interesados en una alianza.
- La composición del equipo: por ahora el proyecto se lleva de forma pública y está abierto a la participación; no se postulan de antemano funciones ni compromisos concretos.

Si algo de esto es importante para ti como posible socio, el lugar adecuado para hablarlo son las Discussions de GitHub del proyecto (véase la sección 9), y no las suposiciones de este documento.

## 9. Cómo contactar y participar

El único canal oficial del proyecto hoy es el repositorio de GitHub:
[github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject)
(rama `main`). No existen otros contactos (correo, redes sociales, entidad jurídica) por el momento, y no se indican aquí a propósito, para no inducir a error.

- **Issues**: comunicar un fallo, proponer un cambio técnico concreto, informar de los resultados de una prueba de vuelo en tu propia copia del prototipo.
- **Discussions**: hablar de la hoja de ruta, de una alianza, de su uso en una tarea concreta (entrega, búsqueda y rescate, agricultura), de las cuestiones de licencia y de financiación de la sección 8.
- **Pull requests**: añadir un sensor nuevo mediante la interfaz `Sensor`, una placa nueva mediante un bloque en `Config.h`, un `AutopilotMode` nuevo, mejoras del dashboard web; la arquitectura está pensada para que esto pueda hacerse sin tocar el núcleo.

Si lees este documento como posible inversor o socio: el siguiente paso con sustancia no es firmar nada, sino abrir una Discussion en el repositorio con una pregunta o propuesta concreta. La hoja de ruta de arriba es una invitación a debatirla fase por fase, con pleno acceso al código en el que se basa.
