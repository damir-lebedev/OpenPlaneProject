# Guía del piloto de OpenPlaneProject

> 🌐 Esta página es una traducción del [original en ruso](../../PILOT_GUIDE.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Esta es una guía práctica de «qué conectar dónde y cómo volar» para quien tiene en la mano un soldador y una emisora, y no se pone a leer el código fuente. Si quieres entender la arquitectura del código, consulta los demás documentos del repositorio. Aquí solo hay hardware, canales, firmware y vuelos.

Repositorio: https://github.com/damir-lebedev/OpenPlaneProject, rama `main`.

Vamos a ser sinceros desde el principio: el proyecto está en pleno desarrollo y **no es un producto terminado**. El primer prototipo ya ha volado, pero con salvedades que se explican más abajo en una sección aparte. Léela antes de volar, no después.

---

## Contenido

1. [Qué hardware necesitas](#qué-hardware-necesitas)
2. [Elección de la placa y asignación de pines](#elección-de-la-placa-y-asignación-de-pines)
3. [Conexión del receptor](#conexión-del-receptor)
4. [Mapa de canales RC](#mapa-de-canales-rc)
5. [Canales del piloto automático](#canales-del-piloto-automático)
6. [ARM y failsafe](#arm-y-failsafe)
7. [Cargar el firmware en la placa](#cargar-el-firmware-en-la-placa)
8. [Panel web en el campo](#panel-web-en-el-campo)
9. [Caja negra](#caja-negra)
10. [Lista de comprobación previa al vuelo y seguridad](#lista-de-comprobación-previa-al-vuelo-y-seguridad)
11. [Resolución de problemas](#resolución-de-problemas)
12. [Estado actual de la estructura del prototipo](#estado-actual-de-la-estructura-del-prototipo)

---

## Qué hardware necesitas

El equipo de la versión actual (el firmware está comprobado con él):

- **Una placa ESP32-S3 N16R8** (un clon de la DevKitC-1 con dos USB-C: «USB» y «COM»).
- **Una emisora FS-i6 + un receptor FS-iA6B** (protocolo iBUS, 10 canales). Hace falta un solo cable de datos: el puerto iBUS SERVO. La emisora tiene que permitir configurar el failsafe; este ajuste es obligatorio, véase la sección sobre failsafe.
- **2 servos MG90S** para los alerones: uno por cada semiala (dos servos independientes, no uno para las dos alas).
- **1 servo MG90S** para el timón de profundidad.
- **1 servo MG90S** para el timón de dirección; en su mismo eje va montada la rueda direccional del tren de aterrizaje (para maniobrar en tierra).
- **Un variador (ESC)** de 60–80 A con BEC de 5 V (el BEC alimenta los servos y el receptor).
- **Un motor D3548 1100KV** + **una hélice 10x5**.
- **Una batería LiPo 3S**.

Sensores del piloto automático (todos por I2C; sin ellos el avión vuela en modo manual):

- **GY-521**: giroscopio + acelerómetro (la placa puede llevar un MPU6050 o, como en nuestro caso, un MPU6500; ambos son compatibles).
- **BMP388**: barómetro.
- **GY-273**: brújula (la nuestra lleva un QMC5883P; el QMC5883L también es compatible).
- Opcionalmente, una pantalla **OLED 128×64 SSD1306** (I2C) para mostrar el estado a bordo.

Estructura: envergadura de 1200 mm, cuerda de 250 mm, perfil NACA 4412, construcción en PETG (impresión 3D). El primer prototipo voló con un ESP32-C3, un motor D2212 1000KV y un ESC de 40 A.

---

## Elección de la placa y asignación de pines

El firmware admite tres placas; para cambiar de una a otra basta un parámetro de compilación (`pio run -e <nombre del entorno>`). Cada placa tiene su propia asignación de pines, fijada en el firmware para cada entorno concreto: no cambies los cables por tu cuenta y consulta la tabla de tu placa.

> **Importante:** la placa principal es ahora la **esp32-s3 (N16R8)**; su asignación de pines está comprobada en el banco con todos los sensores. La **esp32-c3** es el antiguo prototipo que ya ha volado. La asignación de la **esp32-dev** se eligió según la documentación del chip y **no se ha comprobado en hardware real**.

### esp32-s3 (N16R8): la placa principal, comprobada en el banco

`pio run -e esp32-s3`, placa `esp32-s3-devkitc-1` con los ajustes para el módulo N16R8 (16 MB de flash, 8 MB de PSRAM octal). Es la placa por defecto (`default_envs = esp32-s3`).

| Función | GPIO |
|---|---|
| Alerón, semiala izquierda | GPIO4 |
| Alerón, semiala derecha | GPIO5 |
| Timón de profundidad | GPIO6 |
| ESC (acelerador) | GPIO7 |
| Timón de dirección + rueda direccional | GPIO18 |
| iBUS del receptor (RX) | GPIO17 |
| I2C de los sensores SDA / SCL (MPU, BMP388, brújula) | GPIO41 / GPIO42 |
| I2C del OLED SDA / SCL (bus independiente) | GPIO1 / GPIO2 |
| Reserva: GPS RX / TX | GPIO39 / GPIO40 |
| Reserva: AUX1 / AUX2 (servos), AUX3, zumbador, LIGHT | GPIO15 / 16, 47, 38, 21 |
| Batería / sensor de corriente (ADC, lo registra la caja negra); reserva: telemetría TX / RX | GPIO8 / GPIO3, GPIO9 / GPIO10 |
| Solo banco: SPI (ICM42688) SCK / MISO / MOSI / CS | GPIO12 / 13 / 11 / 14 (+ CS del BMP388: GPIO21) |

> El bus de sensores estaba antes en los GPIO8/9 y se ha movido a los 41/42 para adaptarlo al trazado
> de la placa del controlador de vuelo. En el banco: SDA 8→41, SCL 9→42.

No uses: GPIO0/45/46 (de ellos depende el modo de arranque), 19/20 (USB), 26–32 (flash), 33–37 (PSRAM en la N16R8), 43/44 (conector «COM»), 48 (LED RGB). Los pines libres ya están repartidos como reserva; una placa portadora con conectores para el futuro: [`FC_BOARD.md`](FC_BOARD.md).

Conexión de los sensores en el banco (todos los módulos funcionan a **3.3 V**, no a 5 V):

| Módulo | Pines |
|---|---|
| MPU-6050 / GY-521 (la placa puede llevar un MPU6500, y es normal) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, AD0–GND, INT/XDA/XCL: sin conectar. Con el chip hacia arriba y la flecha X apuntando al morro; el giro de los ejes del chip se ajusta con `IMU_ROTATION_CW_DEG` en `Config.h` (90 en nuestro clon) |
| BMP388 | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, SDO–GND (dirección 0x76), **CSB–3.3V** (si no, el chip pasa a SPI), INT: sin conectar |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, DRDY: sin conectar. Lejos de los cables de los servos, del ESC y del motor |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–GPIO2, SDA–GPIO1 |

Los servos se alimentan **no desde la placa**, sino desde el BEC del variador (o desde una fuente independiente de 5 V y al menos 2 A); la masa de todas las fuentes es común. No conectes el cable rojo del ESC a los 5 V de la placa mientras esté conectado el USB.

### esp32-c3: el antiguo prototipo, ya ha volado

`pio run -e esp32-c3`, placa `esp32-c3-devkitm-1`.

| Función | GPIO |
|---|---|
| Alerón, semiala izquierda | GPIO5 |
| Alerón, semiala derecha | GPIO4 |
| Timón de profundidad | GPIO6 |
| ESC (acelerador) | GPIO7 |
| Timón de dirección | — (no quedan pines libres) |
| iBUS del receptor (RX) | GPIO8 |
| I2C SDA (sensores) | GPIO1 |
| I2C SCL (sensores) | GPIO3 |

### esp32-dev (la ESP32 clásica de siempre, de 38 pines): para el banco y la depuración, NO HA VOLADO

`pio run -e esp32-dev`, placa `esp32dev`.

| Función | GPIO |
|---|---|
| Alerón, semiala izquierda | GPIO13 |
| Alerón, semiala derecha | GPIO14 |
| Timón de profundidad | GPIO27 |
| ESC (acelerador) | GPIO26 |
| Timón de dirección | GPIO25 |
| iBUS del receptor (RX) | GPIO16 |
| I2C SDA (sensores) | GPIO21 |
| I2C SCL (sensores) | GPIO22 |

Su ventaja es que es la placa más común y barata de la gama, por lo que sirve para depurar el firmware en el banco, pero su asignación de pines no se ha comprobado en hardware.

---

## Conexión del receptor

Del receptor solo necesitas **un cable de datos iBUS**, que en la mayoría de los receptores compatibles con FlySky sale por un puerto independiente (suele estar rotulado «iBUS», o es la única salida que no es PPM). La conexión:

- **TX del receptor (salida iBUS)** → **pin RX de la placa** de la tabla anterior (GPIO8 en la esp32-c3, GPIO17 en la esp32-s3, GPIO16 en la esp32-dev).
- **Masa (GND) del receptor** → **GND de la placa**. Es obligatorio: sin masa común el protocolo no funciona.
- **Alimentación del receptor**: desde un BEC o regulador aparte o desde los 5 V de la placa, según cómo alimentes normalmente el receptor en tus montajes; no depende de ningún pin concreto del firmware.

El firmware no envía nada de vuelta al receptor, solo escucha, así que la línea TX de la placa no hace falta conectarla a ningún sitio.

La velocidad del puerto iBUS en el firmware es de 115200 baudios, que es la estándar del protocolo; no hay que cambiarla, porque el propio receptor mantiene esa velocidad.

---

## Mapa de canales RC

El mapa se ha comprobado en el banco con una emisora FS-i6 (10 canales, modo 2) y un receptor FS-iA6B.

| Canal | Mando de la emisora | Nombre | Qué hace |
|---|---|---|---|
| CH1 | stick derecho ←→ | AILERON | Alabeo: alerones (2000 = a la derecha) |
| CH2 | stick derecho ↑↓ | ELEVATOR | Cabeceo: timón de profundidad (2000 = stick hacia delante, morro abajo) |
| CH3 | stick izquierdo ↑↓ | THROTTLE | Acelerador. 1000 µs = apagado, 2000 µs = máximo, sin limitación |
| CH4 | stick izquierdo ←→ | RUDDER | Timón de dirección y rueda direccional del tren de aterrizaje (un solo servo) |
| CH5 | SwA | ARM | Interruptor ARM: véase la sección sobre ARM más abajo |
| CH6 | SwB | SWB | Por defecto, flaps: abajo, hacia ti, desplegados; arriba, recogidos (véase más abajo) |
| CH7 | SwC (3 posiciones) | SWC | Por defecto, modo: arriba MANUAL, centro STABILIZE, abajo AUTO_TAKEOFF |
| CH8 | SwD | SWD | Por defecto, RTH (regreso a casa) mientras esté activado |
| CH9 | VrA | VRA | Por defecto, intensidad de la estabilización |
| CH10 | VrB | VRB | Por defecto, velocidad de crucero |

CH6–CH10 pueden ser **lo que quieras, con una sola línea** de `include/config/Controls.h`: cualquiera de los 12 modos, de las 10 funciones (flaps, freno, lanzamiento de carga, geovalla, zumbador…) y de los 7 potenciómetros. Al encender la placa, la asignación real se imprime en el monitor serie. Todo sobre los modos y las asignaciones está en [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

Todos los valores de los canales usan el rango estándar de pulso del receptor, de 1000 a 2000 µs, con 1500 µs como centro/neutro.

### Flaps (flaperones)

No hay flaps independientes: su función la cumplen los alerones. **SwB abajo** (hacia ti): los dos alerones bajan suavemente, en aproximadamente un segundo, el mismo ángulo (`FLAPS_DEPLOYED_US` = 220 µs ≈ 20° de giro de la palanca del servo; se cambia en `include/config/Config.h`). Esto aumenta la sustentación para despegar y aterrizar a menor velocidad. El alabeo con el stick y con el piloto automático funciona como siempre: los alerones se mueven en sentidos opuestos, pero ahora alrededor de la posición bajada. **SwB arriba**: se recogen igual de suavemente. En el OLED, con los flaps desplegados se enciende `FL` en la primera línea.

Con alabeo máximo y los flaps desplegados, el alerón que baja llega al final de su recorrido antes que el que sube; es normal y funciona como un diferencial de alerones.

Desplegar los flaps suele levantar el morro, así que prepárate para empujar un poco el stick hacia delante; si el efecto es fuerte, se ajusta reduciendo `FLAPS_DEPLOYED_US`.

---

## Canales del piloto automático

En resumen, la asignación por defecto (`include/config/Controls.h`):

| Interruptor | Qué hace |
|---|---|
| **SwC** (CH7) | arriba **MANUAL** · centro **STABILIZE** · abajo **AUTO_TAKEOFF** |
| **SwD** (CH8) | **RTH**: a casa mientras esté activado |
| **SwB** (CH6) | flaps |
| **VrA / VrB** (CH9/10) | intensidad de la estabilización / velocidad de crucero |

- **STABILIZE: «el stick fija el ángulo».** Con el stick a fondo, 45° de alabeo y 25° de cabeceo; al soltarlo, el avión se nivela solo. El acelerador es tuyo.
- **AUTO_TAKEOFF.** Tras el ARM no ocurre nada hasta que subas el acelerador por encima de la mitad. Después: 0–1 s, el acelerador sube suavemente hasta el 100 % con las alas niveladas; 1–3 s, cabeceo +15°; luego +10° hasta que cambies SwC. Acelerador = el máximo entre el stick y el programa.
- **RTH.** Rumbo al punto del ARM, altitud de 40 m, círculos sobre el punto de origen. Necesita un GPS con fijación 3D **antes del ARM**.

Los 12 modos (ALT_HOLD, ACRO, CRUISE, LOITER, LAUNCH a mano, AUTO_LAND, SOARING, RESCUE…), lo que necesita cada uno en cuanto a sensores y cómo asignarlo a un interruptor se explican en [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md). Un modo elegido desde el panel o desde la estación de tierra se mantiene hasta que muevas el interruptor de modo.

La estabilización mueve las superficies de control incluso cuando la aeronave **no está armada**; así, en la mesa se ve hacia qué lado responde a una inclinación. El integrador, en este caso, no se acumula.

Signos de los ángulos (como en el OLED y en el registro): **alabeo R > 0: ala derecha abajo; cabeceo P > 0: morro arriba.**

---

## ARM y failsafe

### Procedimiento de ARM

- **ARM:** acelerador (CH3) abajo → interruptor **SwA (CH5) abajo, hacia ti** (en la FS-i6 es CH5 = 2000; arriba = 1000). En Serial aparece `ArmingManager: ARM`, y en el OLED, `ARMED`.
- **DISARM:** SwA arriba, alejado de ti: al instante, en cualquier momento; el motor se detiene de inmediato.
- Si se pasa SwA a ARM con el acelerador que no está abajo, o si no se han superado las comprobaciones previas al vuelo, el ARM **no** se produce (el motivo se imprime en Serial). Hay que volver a poner SwA arriba, bajar el acelerador y volver a ponerlo abajo.
- Si la placa se enciende con SwA ya en la posición de ARM (abajo), **no** se arma: el firmware tiene que ver primero SwA arriba (OFF).

**El ARM bloquea de verdad el acelerador:** mientras la aeronave no esté armada, el acelerador del ESC se mantiene forzosamente al mínimo, sea cual sea la posición del stick. Antes de armar se comprueba que los sensores que necesita el modo elegido responden; por ejemplo, STABILIZE sin un IMU en funcionamiento no se arma hasta que cambies a MANUAL. De todos modos, compórtate como si la hélice pudiera empezar a girar en cualquier momento después del ARM.

### Failsafe

La pérdida de enlace tiene **prioridad absoluta** sobre todo lo demás:

- **aeronave armada, con GPS y punto de origen**: **regreso a casa con motor** (como en ArduPilot/INAV): rumbo al punto de origen, altitud de 40 m, círculos sobre él hasta que vuelva el enlace. En el OLED aparece `FSRTH`. Se desactiva con `FAILSAFE_RTH = false` en `Config.h`;
- **aeronave armada, sin GPS**: **planeo**: motor apagado; el piloto automático, en cualquier modo, incluso en MANUAL, mantiene las alas niveladas y el morro algo por debajo del horizonte (−3°), y los flaps se recogen. En el OLED aparece `RX LOST ... GLIDE`. Si pones `FAILSAFE_GLIDE_ROLL_DEG` = 10–20, el avión describirá una espiral suave sobre ti;
- **sin armar** (en tierra) o con el IMU sin responder: motor apagado; alerones, timón de profundidad y timón de dirección en neutro (1500 µs); tras 10 s sin enlace suena el zumbador «estoy aquí» (si está soldado).

El firmware detecta la pérdida de enlace de dos maneras:

1. **No llegan tramas iBUS durante más de 500 ms**: un cable roto o un receptor sin alimentación.
2. **Acelerador por debajo de 950 µs**: así avisa el receptor de que ha perdido la emisora. **Esto exige configurar el failsafe en la emisora** (véase más abajo): al perder el enlace, el FS-iA6B NO deja de enviar tramas, sino que repite los últimos valores de los sticks; sin esa configuración el firmware no detectará la pérdida de enlace y el avión seguirá volando con el último acelerador.

El ARM **no** se restablece con el failsafe: cuando vuelve el enlace, el avión obedece de nuevo a los sticks y al modo elegido sin necesidad de rearmar (cambiar el interruptor en el aire con el acelerador a cero es más peligroso).

Comprobación en la mesa (sin hélice): ARM → apagar la emisora → en el OLED aparece `RX LOST ... GLIDE` y el motor se ha parado; inclina el avión: las superficies de control deben devolverlo a la horizontal. Enciende la emisora: `RX ok`, y el control vuelve a los sticks.

### Configuración del failsafe en la emisora FS-i6 (obligatoria, una sola vez)

La idea: al perder el enlace, el receptor debe dar un acelerador de unos 900 µs, por debajo del mínimo normal de 1000.

1. `Menu → Functions setup → End points` → canal 3: pon el punto inferior (el valor de la izquierda) en **120 %**. Guarda (pulsación larga de Cancel).
2. Acelerador **al mínimo**.
3. `Menu → Functions setup → Failsafe` → Channel 3 → **On**, con el acelerador todavía abajo → guarda con una pulsación larga de Cancel. El receptor recordará unos 900 µs.
4. Vuelve a `End points` → canal 3 → pon el punto inferior otra vez en **100 %**. Guarda.
5. Comprobación: no hace falta ARM. Apaga la emisora; al cabo de 1 s aproximadamente aparece en Serial `RX=LOST(failsafe пульта)` y en el OLED, una línea `RX LOST` en vídeo inverso. Enciende la emisora: `RX=OK`.

Mantén el trim del acelerador en el centro: con el trim muy bajado, el acelerador puede caer por debajo de 950 y el firmware lo tomará por una pérdida de enlace.

Antes había aquí un **boost en CH8** y un límite de acelerador del 40 %; se han eliminado: el límite protegía un montaje 3S1P débil, y las baterías nuevas no temen el acelerador a fondo. Además, el boost se activaba solo si SwD estaba arriba al encender la placa.

---

## Cargar el firmware en la placa

El firmware se compila con **PlatformIO** (framework Arduino, C++).

### Instalación de PlatformIO

Lo más sencillo es instalar la extensión **PlatformIO IDE** en VS Code (Extensions → buscar «PlatformIO IDE» → Install); así tendrás la CLI y también cómodos botones de compilación en la interfaz. También puedes instalarlo con `pip install platformio` y trabajar desde el terminal; ambas opciones usan los mismos comandos `pio`.

### Compilación y carga

Abre el proyecto (la carpeta del repositorio) en VS Code con PlatformIO instalado, conecta la placa por USB y ejecuta en el terminal el comando correspondiente a tu placa:

```bash
# esp32-s3 N16R8 (placa principal)
pio run -e esp32-s3 -t upload

# esp32-c3 (antiguo prototipo)
pio run -e esp32-c3 -t upload

# esp32-dev (ESP32 clásica de 38 pines, para el banco)
pio run -e esp32-dev -t upload
```

Si no indicas `-e`, se compila la placa por defecto: `esp32-s3`.

**esp32-s3:** la placa tiene dos conectores USB-C. La carga del firmware y Serial van por el conector **«COM»** (puente CH343; en Windows aparece como «USB-Enhanced-SERIAL CH343»). El conector «USB» (el USB nativo del chip) no hace falta para trabajar, pero puede estar conectado: no molesta en nada.

### Monitor serie

Para ver la salida de depuración (estado de los canales, ARM, sensores) directamente en la consola por USB:

```bash
pio device monitor -b 115200
```

La velocidad tiene que ser 115200; si no, verás un galimatías ilegible en lugar de texto. Cada 10 segundos se imprime una línea `SYS`: la frecuencia del ciclo (debe rondar los 500 Hz), el tiempo medio y el peor del ciclo en 10 s, los contadores de iBUS y la memoria libre. Todo lo demás sale por los canales que estén activados en el menú del registro (véase más abajo).

### Consola: menú y registro (monitor serie)

Al pulsar una tecla, esta actúa al instante; no hace falta Enter (en un monitor que envía por líneas, escribe la letra + Enter). Las calibraciones y la comprobación de las salidas solo funcionan cuando la aeronave no está armada.

| Tecla | Qué hace |
|---|---|
| `h` | **menú principal** (de texto, con las opciones numeradas) |
| `l` | menú «qué mostrar en el registro» |
| espacio | pausa del registro / reanudar |
| `s` | estado detallado de todos los sensores (incluidos los contadores de errores de I2C) |
| `i` | recalibrar el giroscopio y hacer la comprobación previa al vuelo del IMU: 2 s, sin mover el avión |
| `o` | **calibración de la instalación del IMU**: una sola vez, después de montar la placa en el avión (véase más abajo) |
| `m` | calibración de la brújula: durante 15 s, gira la placa o el avión sobre todos los ejes. El resultado se guarda en la flash y sobrevive a un reinicio |
| `p` | comprobación de las salidas: el pulso real en cada pin |

**Registro por canales.** Cada tipo de datos es una línea con su propio prefijo, y cada uno tiene su propio modo: **desactivado**, **al cambiar** (la línea aparece solo cuando los valores han cambiado de verdad; el temblor de los sticks y el ruido de los sensores no cuentan) o **continuo** (cada 0.2 / 0.5 / 1 / 2 s; el periodo se elige en el mismo menú).

| Canal | Qué muestra | Por defecto |
|---|---|---|
| `STAT` | enlace, ARM, modo, flaps, si el IMU y el barómetro están bien | al cambiar |
| `RC` | canales de la emisora, µs | desactivado |
| `OUT` | salidas a las superficies de control y al ESC, µs | desactivado |
| `ATT` | alabeo, cabeceo, rumbo | desactivado |
| `AP` | piloto automático: objetivos y correcciones | desactivado |
| `ALT` | altitud, velocidad vertical, objetivo de ALT_HOLD | desactivado |
| `MAG` | rumbo de la brújula | desactivado |
| `GPS` | fijación, satélites, coordenadas, velocidad | desactivado |
| `IMU` | giroscopio y acelerómetro | desactivado |
| `SYS` | frecuencia y duración del ciclo, memoria (cada 10 s) | activado |

Mientras el menú está abierto, el registro calla para que el menú no se desplace; al salir, se vuelven a imprimir todos los canales activados. La selección se guarda en la flash al salir del menú y sobrevive a un reinicio. Con la aeronave armada, surte efecto de inmediato, pero se escribe solo después del DISARM: una escritura en la flash detiene el ciclo de vuelo unos 0.4 s.

### Instalación del IMU: como quieras, con una sola calibración

La placa con el IMU se puede montar en el avión **como resulte cómodo**: en cualquier ángulo, de lado, boca abajo; el firmware averigua por sí mismo dónde están su morro y su parte superior. Se hace **una sola vez** después de la instalación (y de nuevo si se ha cambiado la placa de sitio):

1. El avión sobre la mesa, sin necesidad de emisora y con el motor sin armar. En el monitor serie, pulsa `o`.
2. **Paso 1:** el avión está nivelado, como en vuelo horizontal (si tiene rueda de cola, pon algo debajo de la cola). No lo toques durante unos 3 s.
3. **Paso 2:** levanta el **morro** de 30 a 60°, con las alas niveladas, y mantenlo quieto durante 1 s aproximadamente.
4. **Paso 3:** vuelve a bajar el morro, baja el **ala derecha** de 30 a 60° y mantenla así durante 1 s aproximadamente.

Cada paso se da por válido automáticamente (en el registro aparece «засчитано», es decir, «contado»). Al final se muestra el resultado («нос = +Y чипа, верх = −Z чипа», es decir, «morro = +Y del chip, arriba = −Z del chip») y «установка сохранена» («instalación guardada»). Si te has equivocado (has bajado el morro en vez de subirlo, o el ala izquierda en vez de la derecha), la calibración se rechaza con una explicación; repite `o`. El resultado se guarda en la flash y sobrevive a un reinicio. Comprobación: morro arriba → la P del OLED pasa a positivo; ala derecha abajo → la R pasa a positivo.

Mientras no exista una calibración de la instalación, rige el método antiguo: la placa debe ir con el chip hacia arriba, y el giro es `IMU_ROTATION_CW_DEG` en `Config.h`.

### Comprobación previa al vuelo al encender

Cada vez que se enciende, el IMU dedica unos 2 s a calibrar el giroscopio y, de paso, se comprueba a sí mismo:

- **el avión está inmóvil**: si en ese momento se sostenía en las manos o se movía, el desfase del giroscopio será incorrecto;
- el acelerómetro en reposo marca 1g;
- **«arriba» coincide con la calibración de la instalación**: si la placa se ha cambiado de sitio o se ha dado la vuelta, se nota enseguida (un avión sobre su rueda de cola o en una pendiente no es problema; la tolerancia es de 45°).

**Enciende el avión mientras está quieto.** No es imprescindible que esté nivelado si la instalación está calibrada (si no, la posición al encender se convierte en el horizonte). El resultado figura en el registro: `предполётная проверка пройдена` («comprobación previa al vuelo superada») o `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА — <причина>` («COMPROBACIÓN PREVIA AL VUELO NO SUPERADA», seguida del motivo). Si no se supera, **el ARM en STABILIZE y AUTO_TAKEOFF queda prohibido** (el motivo se imprime al intentarlo), y el piloto automático no da correcciones en ningún modo, incluido el planeo por pérdida de enlace: con ángulos erróneos gobernaría en sentido contrario. El ARM en MANUAL sigue siendo posible. Para arreglarlo: deja el avión inmóvil y vuelve a conectar la batería (o pulsa `i`); si la placa se ha cambiado de sitio, pulsa `o`.

### Pantalla OLED

Si hay un OLED conectado (GPIO1/GPIO2), muestra lo siguiente 5 veces por segundo:

```
RX ok disarm STAB        <- enlace / ARM / modo (si se pierde el enlace, la línea sale invertida)
R  +1.2 P  -0.4          <- alabeo / cabeceo, °
Alt +0.3 Vz +0.1         <- altitud desde el punto de encendido, m / velocidad vertical, m/s
Hdg 123  Thr 1000        <- rumbo de la brújula / acelerador al ESC, µs
L1500 R1500 E1500        <- PWM de los alerones y del timón de profundidad, µs
Loop 500Hz max 1100us    <- frecuencia y peor duración del ciclo
```

---

## Panel web en el campo

La placa levanta su propio punto de acceso Wi-Fi: no hace falta ningún router doméstico ni internet, y todo funciona directamente en el campo desde el móvil.

**Cómo conectarse:**

1. En el móvil o en el portátil, abre la lista de redes Wi-Fi.
2. Conéctate a la red **`OpenPlane-Debug`**, con la contraseña **`12345678`**.
3. Abre en el navegador la dirección **`http://192.168.4.1`**.

No hace falta instalar ninguna aplicación: es una página web normal.

**Qué se puede hacer desde el móvil en el campo, sin programar nada:**

- Ver las **barras en directo de los 10 canales RC**: es cómodo para comprobar que la emisora y el receptor envían de verdad lo que mueves en el stick, incluso antes de conectar los servos.
- Ver el estado de cada salida (alerón izquierdo/derecho, timón de profundidad, ESC): si el canal está conectado por software.
- Ver si responden los sensores (IMU, barómetro), si los tienes soldados: se muestran con sinceridad o bien los datos reales (alabeo/cabeceo/altitud), o bien una indicación explícita de que el sensor no existe físicamente o no responde.
- **Cambiar el modo del piloto automático** con botones (manual / estabilización / despegue automático / mantenimiento de altitud) directamente desde la página, sin emisora.
- **Ajustar los coeficientes del controlador PID** (para el alabeo y el cabeceo) mediante un formulario de la página: útil para afinar poco a poco la estabilización sin volver a cargar el firmware.

El alcance de este punto de acceso es, en la práctica, de decenas de metros: es el Wi-Fi normal de un ESP32, no una telemetría de largo alcance. Es una herramienta para ajustar sobre la mesa, en el banco y junto al campo, no para controlar la aeronave en vuelo a distancia.

---

## Caja negra

El firmware de la ESP32-S3 graba por sí solo cada vuelo en la flash: todo lo que vieron los sensores, lo que hicieron los sticks, adónde fueron los servos y lo que decidió el piloto automático. No hay que hacer nada:

- **graba** desde el momento en que la aeronave está armada y se sube el acelerador (más los 10 s anteriores);
- **deja de grabar** 10 s después del DISARM, o si una aeronave armada permanece inmóvil con el motor apagado durante 30 s (ha aterrizado o se ha estrellado y se ha olvidado el DISARM);
- la pérdida de enlace, el motor a cero y el planeo **no** detienen la grabación.

Al encender, el monitor serie muestra cuánto espacio hay para un vuelo (la consola imprime en ruso; la línea de abajo dice «espera ARM y acelerador | borrado por delante 12.9 MB (≈11 min) de 13.9 MB | vuelos 1»):

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

**Después del vuelo**, conecta el USB (el conector COM), cierra el monitor serie y descarga el vuelo:

```bash
python tools/blackbox.py download
```

El vuelo acaba en la carpeta `blackbox/` junto con su análisis: `summary.txt` (resumen y eventos), `events.txt` y tablas CSV por sensor. La caja negra borra sola los vuelos antiguos cuando necesita espacio, así que **descarga después de cada salida**. El menú de la consola es la tecla `k`. Todos los detalles están en [BLACKBOX.md](BLACKBOX.md).

**En la placa STM32H743** la caja negra graba en una tarjeta SD: la tarjeta se formatea en FAT32 y se prepara una sola vez en un PC (`python tools/blackbox.py sd-prepare E:`). Después del vuelo se puede descargar por USB con el mismo comando `download`, o bien sacar la tarjeta y descodificar el archivo directamente desde ella: `python tools/blackbox.py ring E:/BLACKBOX.BIN`.

---

## Lista de comprobación previa al vuelo y seguridad

Lee esta sección entera **antes** de la primera puesta en marcha, no después de un incidente.

### Obligatorio antes de cualquier prueba en el banco

- [ ] **La hélice está físicamente QUITADA** si estás comprobando canales, el ARM o el panel, ajustando el PID, o simplemente encendiendo la placa por primera vez con una asignación de pines nueva. El ESC puede hacer dar un tirón al motor en cualquier fase de la prueba: no es un riesgo hipotético, es el comportamiento normal la primera vez que se enciende.
- [ ] La batería LiPo está revisada (sin hinchazón ni daños), cargada con un cargador adecuado para LiPo, y se guarda y se carga sobre una superficie ignífuga.
- [ ] La emisora está encendida y sus canales se han comprobado en el panel (`http://192.168.4.1`) **antes** de conectar la batería al ESC.

### Antes del vuelo

- [ ] Una zona despejada, sin personas ni construcciones en un radio suficiente para un avión de 1200 mm de envergadura bajo control manual con un comportamiento anómalo de los servos o del ala (véase más abajo la sección sobre las limitaciones del prototipo: la fijación del motor y el ala aún no están reforzadas con carbono).
- [ ] Las tres superficies se mueven en el sentido correcto. Compruébalo en la mesa antes de cada salida, sin fiarte de la memoria de la vez anterior:
  - stick derecho a la derecha → **alerón derecho arriba, izquierdo abajo**;
  - stick derecho hacia ti → **timón de profundidad arriba**;
  - stick izquierdo a la derecha → **timón de dirección y rueda a la derecha**;
  - SwB (flaps) abajo → **los dos alerones bajan suavemente**, y el alabeo con el stick los sigue separando en sentidos opuestos;
  - en STABILIZE, inclina el avión con el ala derecha abajo → **alerón derecho abajo, izquierdo arriba** (las superficies lo devuelven a la horizontal); morro abajo → **timón de profundidad arriba**.
  Si algo no va bien, cambia el `*_REVERSED` correspondiente en `include/config/Config.h` (sección «Dirección de los servos»), no la inversión en la emisora: si no, el stick y el piloto automático discreparán.
- [ ] Al encender, el avión estaba inmóvil, y en el registro o en el panel no aparece `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА`; al inclinar el avión con las manos, la P y la R del OLED cambian en el sentido correcto.
- [ ] El failsafe está configurado en la emisora y comprobado: apaga la emisora → `RX LOST` en el OLED o en Serial (véase la sección sobre failsafe).
- [ ] Se ha comprobado el alcance de la emisora y la batería de la emisora está cargada.
- [ ] Al encender, el registro muestra `BlackBox: ... стёрто впереди N МБ (≈M мин)`: hay espacio suficiente para el vuelo. El vuelo anterior se ha descargado (`python tools/blackbox.py download`), si lo necesitas.
- [ ] Mantén las manos y la cara lejos de la hélice siempre que haya una LiPo conectada al ESC: tras el ARM, el acelerador responde al instante al stick, sin más avisos. DISARM: SwA arriba.
- [ ] Asegúrate de poder desconectar físicamente la alimentación con rapidez (acceso al conector de la LiPo) en lugar de confiar solo en el failsafe por pérdida de señal.

### Seguridad general con las LiPo

- No dejes nunca una LiPo cargándose sin vigilancia.
- No conectes ni desconectes la LiPo al ESC mientras estés junto al plano de giro de la hélice.
- Transporta y guarda las LiPo en una bolsa o contenedor de protección.

---

## Resolución de problemas

**«No hay señal del receptor» / el RX parece normal, pero los canales del panel no se mueven**
Comprueba que el cable de datos del receptor esté conectado justo al pin RX de iBUS de la tabla de tu placa (GPIO8/GPIO17/GPIO16), que no esté confundido con la masa o la alimentación y que la masa del receptor y la de la placa estén unidas. Si la asignación de pines coincide y los cables están bien, pero sigue sin haber señal, comprueba que el receptor esté emparejado (bind) con la emisora y que su salida esté configurada como iBUS, no como PPM/SBUS.

**Un sensor (IMU, barómetro, brújula) indica «no responde» / NO_RESPONSE**
El firmware informa con sinceridad de que el sensor no responde, en lugar de dar ceros. Comprueba: (1) la alimentación del módulo, 3.3V y la masa de la placa; (2) SDA/SCL, en los pines I2C de tu placa; (3) la dirección en la línea: MPU 0x68 (AD0 a GND), BMP388 0x76 (SDO a GND, **CSB a 3.3V**; si no, el chip está en modo SPI), QMC5883P 0x2C, QMC5883L 0x0D. Si el sensor a veces responde y a veces no (o contesta en una dirección ajena), es un mal contacto en la protoboard: aprieta VCC/GND/SDA/SCL y, mejor, alimenta cada módulo directamente desde el 3.3V/GND de la placa. El comando `s` de la consola muestra los contadores de errores de I2C de cada sensor.

**Los ángulos del OLED están cambiados (el morro arriba cambia la R, no la P) o tienen el signo contrario**
Haz la calibración de la instalación del IMU (`o`, véase «Instalación del IMU»): no depende de cómo esté soldado el chip en el módulo ni de cómo esté colocado el módulo en el avión. Sin ella: en los clones del GY-521 el chip a veces está soldado girado respecto a las flechas impresas; gira los ejes en `Config.h` → `IMU_ROTATION_CW_DEG` (0/90/180/270). Comprobación: morro arriba → la P pasa a positivo; ala derecha abajo → la R pasa a positivo.

**ARM rechazado: «IMU: ...»**
No se ha superado la comprobación previa al vuelo del IMU (véase «Comprobación previa al vuelo al encender»): el avión se movió al encender; ponlo inmóvil y vuelve a conectar la batería; «arriba» no coincide con la calibración, se cambió la placa de sitio; haz `o`; «la placa no está con el chip hacia arriba», la instalación no está calibrada; haz `o`.

**Un servo o el ESC responde a un stick que no es / no responde**
El comando `p` de la consola mide el pulso real en cada salida (GPIO4–7) y lo compara con el esperado. Si todo está «OK» y el servo no se mueve, el problema está más allá de la placa: (1) el servo no está alimentado (BEC/5V, masa común); (2) el cable de señal está en otro pin; (3) la mecánica se ha atascado. Si aparece «НЕ СОВПАДАЕТ» («NO COINCIDE»), el problema está en el firmware o en los periféricos: informa al desarrollador.

**El motor no gira en absoluto, aunque el acelerador del OLED o del panel responde al stick**
Comprueba que la LiPo esté conectada al ESC y que la aeronave esté realmente armada: hasta el ARM, el acelerador al ESC se mantiene forzosamente al mínimo, y eso no es una avería. Un ESC que al encender vio un acelerador que no era el mínimo puede pitar sin parar y no armarse: vuelve a conectar la batería con el acelerador abajo. Si tras mover SwA hacia abajo no aparece `ArmingManager: ARM`, mira Serial: el firmware imprime el motivo (acelerador que no está abajo, sensor que no responde, uno que necesita el modo elegido ahora en CH7); devuelve SwA arriba, elimina la causa y vuelve a bajarlo.

**El motor se cala o da tirones al acelerar**
Si el ESC se alimenta desde una fuente de laboratorio, se está chocando con el límite de corriente de la fuente: incluso sin hélice, el motor consume unos amperios durante un instante, la tensión cae y el ESC se reinicia. Sube el límite de corriente o usa una LiPo. Los picos de un reinicio así pueden colgar el puente USB de la placa (el puerto «COM» deja de abrirse): vuelve a conectar el cable.

**Tras cargar el firmware, la placa no responde / no aparece el punto de acceso `OpenPlane-Debug`**
Asegúrate de que la carga (`pio run -e <tu placa> -t upload`) terminó sin errores y de que cargaste justo el entorno de la placa que tienes en la mano (la esp32-c3 se diferencia de la esp32-s3 y de la esp32-dev no solo en los pines, sino también en el chip: el firmware para otro chip no se instalará en la placa o se instalará mal). Revisa la salida del monitor serie (`pio device monitor -b 115200`) justo después de reiniciar la placa: ahí se imprime en qué fase de setup() está.

---

## Estado actual de la estructura del prototipo

Para que las expectativas sean sinceras:

- El primer prototipo **ya ha volado**. Se detectaron problemas: **resistencia insuficiente de la fijación del motor** y **resistencia insuficiente del ala**; el ala necesita refuerzo de carbono. También hace falta ajustar mejor los servos. Tenlo en cuenta al planificar tus propios vuelos: no es una salvedad abstracta, sino un fallo real que ya ocurrió con este prototipo.
- **El banco de la esp32-s3 está montado con todos los sensores** (GY-521 con un MPU6500, BMP388 por I2C, GY-273 con un QMC5883P, OLED): todos responden y el ciclo va a 500 Hz. Los modos del piloto automático se han comprobado en la mesa, pero **todavía no se han probado en vuelo**.
- El barómetro BMP388 se calcula con la fórmula completa de compensación de Bosch; la altitud es relativa al punto de encendido. La altitud absoluta sobre el nivel del mar se calcula según la atmósfera estándar, sin corrección por el tiempo.
- La brújula QMC5883P necesita calibración (`m` en la consola) con el avión ya montado: junto al motor y los cables los desfases son distintos de los de la protoboard. El rumbo todavía no tiene compensación de inclinación y ningún modo lo utiliza.
- La licencia es la OpenPlane License: MIT con atribución obligatoria al autor (Damir Lebedev), prohibición del uso militar y prohibición de dañar intencionadamente a personas o bienes sin su consentimiento; véase [LICENSE](LICENSE.md).

Si construyes tu propia aeronave siguiendo esta guía, vuela primero con control manual (MANUAL) y solo después pasa al piloto automático.
