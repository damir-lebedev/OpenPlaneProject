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

- **Una placa STM32H743 DevEBox H743** (MCUDEV): la principal. La antigua placa principal, la ESP32-S3 de abajo, también es compatible.
- **Una placa ESP32-S3 N16R8** (un clon de la DevKitC-1 con dos USB-C: «USB» y «COM»).
- **Una emisora FS-i6 + un receptor FS-iA6B** (protocolo iBUS, 10 canales). Hace falta un solo cable de datos: el puerto iBUS SERVO. La emisora tiene que permitir configurar el failsafe; este ajuste es obligatorio, véase la sección sobre failsafe.
- **O cualquier otra emisora con receptor SBUS** (FrSky, Futaba, RadioMaster, Jumper…) **o CRSF** (ExpressLRS, TBS Crossfire): el protocolo se elige en `Config.h` (`RX_PROTOCOL`) y la conexión está más abajo.
- **2 servos MG90S** para los alerones: uno por cada semiala (dos servos independientes, no uno para las dos alas).
- **1 servo MG90S** para el timón de profundidad.
- **1 servo MG90S** para el timón de dirección; en su mismo eje va montada la rueda direccional del tren de aterrizaje (para maniobrar en tierra). El tren de aterrizaje: dos ruedas delante y una rueda de cola direccional (`LANDING_GEAR = TAIL_WHEEL` en `Config.h`; un tren con rueda de morro es `NOSE_WHEEL`).
- **Un variador (ESC) BLHeli 80A bidireccional** (modo Bidirectional/3D) con BEC de 5 V (el BEC alimenta los servos y el receptor). También sirve un variador unidireccional normal: `ESC_BIDIRECTIONAL = false` en `Config.h`, consulta [Inversión del motor](#inversión-del-motor-swd).
- **Un motor D3548 1100KV** + **una hélice 10x5**.
- **Una batería LiPo 3S**.

Sensores del piloto automático (todos por I2C; sin ellos el avión vuela en modo manual):

- **GY-521**: giroscopio + acelerómetro (la placa puede llevar un MPU6050 o, como en nuestro caso, un MPU6500; ambos son compatibles).
- **BMP581**: barómetro (el BMP388 anterior también es compatible).
- **GY-273**: brújula (la nuestra lleva un QMC5883P; el QMC5883L también es compatible).
- Opcionalmente, una pantalla **OLED 128×64 SSD1306** (I2C) para mostrar el estado a bordo.

Estructura: envergadura de 1200 mm, cuerda de 250 mm, perfil NACA 4412, construcción en PETG (impresión 3D). El primer prototipo voló con un ESP32-C3, un motor D2212 1000KV y un ESC de 40 A.

---

## Elección de la placa y asignación de pines

El firmware admite cinco placas; para cambiar de una a otra basta un parámetro de compilación (`pio run -e <nombre del entorno>`). Cada placa tiene su propia asignación de pines, fijada en el firmware para cada entorno concreto: no cambies los cables por tu cuenta y consulta la tabla de tu placa.

> **Importante:** la placa principal es ahora la **STM32H743 (DevEBox H743)**; en ella están comprobados el arranque, la consola por USB, la tarjeta SD, el iBUS, los servos y el motor, y los sensores se conectan por primera vez. La **esp32-s3 (N16R8)** es la antigua placa principal; su asignación de pines está comprobada en el banco con todos los sensores. La **esp32-c3** es el antiguo prototipo que ya ha volado. La asignación de la **esp32-dev** se eligió según la documentación del chip y **no se ha comprobado en hardware real**.

### STM32H743 (DevEBox H743): la placa principal

`pio run -e stm32h743-devebox`, placa MCUDEV DevEBox H743 (STM32H743VIT6). Es la placa por defecto (`default_envs = stm32h743-devebox`). En ella ya están comprobados el arranque, la consola por USB, la tarjeta SD y la caja negra, la recepción de iBUS, el ARM y los servos y el motor desde la emisora; los sensores se conectan por primera vez.

| Función | Pin |
|---|---|
| Alerón, semiala izquierda | PA0 |
| Alerón, semiala derecha | PA1 |
| Timón de profundidad | PA2 |
| ESC (acelerador) | PA3 |
| Timón de dirección + rueda direccional | PD14 |
| Receptor (RX): iBUS, SBUS, CRSF | PE7 |
| I2C de los sensores SDA / SCL (MPU, BMP581, brújula) | PB11 / PB10 |
| I2C del OLED SDA / SCL (bus independiente) | PB9 / PB8 |
| GPS: RX (← TX del GPS) / TX (→ RX del GPS) | PD9 / PD8 |
| Telemetría MAVLink (radiomódem): RX / TX | PD0 / PD1 |
| AUX1 / AUX2 (servos), zumbador | PD15 / PE9, PE15 |
| SPI de los sensores SCK / MISO / MOSI, CS del IMU / CS del barómetro | PB13 / PB14 / PB15, PB12 / PD10 |
| Batería / sensor de corriente (ADC, lo registra la caja negra) | PC0 / PC1 |

La consola, el registro y la descarga de la caja negra van por el USB-C de la placa (un puerto COM virtual). No ocupar: PA11/PA12 (USB), PA13/PA14 (SWD), PC8–PC12 y PD2 (ranura µSD), PE3 y PC5 (botones K1/K2).

Conexión de los sensores en el banco (todos los módulos funcionan a **3.3 V**, no a 5 V):

| Módulo | Pines |
|---|---|
| MPU-6050 / GY-521 (la placa puede llevar un MPU6500, y es normal) | VCC–3.3V, GND–GND, SCL–PB10, SDA–PB11, AD0–GND, INT/XDA/XCL: sin conectar. Módulo MPU-6500 suelto (10 pines): igual, más **NCS–3.3V** (si no, el chip pasa a SPI) y FSYNC–GND; EDA/ECL: sin conectar. Con el chip hacia arriba y la flecha X apuntando al morro; el giro de los ejes del chip se ajusta con `IMU_ROTATION_CW_DEG` en `Config.h` (90 en nuestro clon) |
| BMP581 | VCC–3.3V (**solo 3.3V**: muchos módulos no llevan regulador propio), GND–GND, SCL–PB10, SDA–PB11, **SDO–GND** (dirección 0x46; no dejarlo al aire), **CSB–3.3V** (si no, el chip pasa a SPI), INT: sin conectar |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–PB10, SDA–PB11, DRDY: sin conectar. Lejos de los cables de los servos, del ESC y del motor |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–PB8, SDA–PB9 |

Los servos se alimentan **no desde la placa**, sino desde el BEC del variador (o desde una fuente independiente de 5 V y al menos 2 A); la masa de todas las fuentes es común. No conectes el cable rojo del ESC a los 5 V de la placa mientras esté conectado el USB.

### Matek H743-WING V3: una controladora de vuelo lista para usar

`pio run -e matek-h743`: la placa Matek H743-WING V3 (la H743-WLITE y la H743-SLIM V3 tienen el mismo pinout). Una controladora de vuelo lista para usar con el mismo STM32H743: el regulador de alimentación, el sensor de corriente, la IMU ICM-42688-P, el barómetro DPS310 y la ranura microSD ya están en la placa; no hay que soldar nada: servos, variador, receptor y GPS se conectan a los pads rotulados. **El firmware aún no se ha probado en una placa real** (el pinout sale de la descripción de la placa en ArduPilot y todo el firmware está comprobado con pruebas en el PC): antes del primer vuelo, comprueba en tierra las superficies, el motor y los modos.

| Función | Pad | Pin |
|---|---|---|
| ESC (gas) | S1 | PB0 |
| Alerón, ala izquierda / derecha | S3 / S4 | PA0 / PA1 |
| Profundidad | S5 | PA2 |
| Timón de dirección + rueda | S6 | PA3 |
| AUX1 / AUX2 (lanzamiento de carga, cámara) | S7 / S8 | PD12 / PD13 |
| Receptor: iBUS, SBUS, CRSF (TX solo para CRSF) | R6 / T6 | PC7 / PC6 |
| GPS: RX (← TX del GPS) / TX (→ RX del GPS) | R2 / T2 | PD6 / PD5 |
| Telemetría MAVLink (radiomódem): RX / TX | R7 / T7 | PE7 / PE8 |
| Brújula, tubo de Pitot (I2C externo) | SDA / SCL | PB7 / PB6 |
| Zumbador | BZ+ / BZ− | PA15 |
| Batería / sensor de corriente | VBAT / Curr | PC0 / PC1 |

- **Sensores en la placa**: la IMU y el barómetro; su kit (`SENSOR_KIT_MATEK_H743`) lo fija el entorno `matek-h743`, no hay que tocar nada en `SensorSelection.h`. La placa no lleva brújula: un módulo GPS con brújula se conecta a los pads SDA/SCL; en OpenPlane Studio es la elección de brújula en el paso «Sensores», y desde la línea de comandos, el flag `-D SENSOR_MAG=SENSOR_MAG_QMC5883L` en `platformio.ini`.
- **La placa puede ir como resulte cómodo**, incluso con la flecha hacia atrás: la IMU está soldada en ella, así que antes del primer ARM con estabilización hace falta la calibración de montaje con tres posturas (`o` en la consola o «Puesta a punto» → «Montaje de la IMU» en la aplicación). Sin ella, el ARM en modos estabilizados se rechaza con una explicación; en MANUAL sí se puede.
- **Primer flasheo:** mantén pulsado el botón BOOT y conecta el USB: la placa entra en modo DFU (Windows necesita el controlador WinUSB, como para la DevEBox); luego `pio run -e matek-h743 -t upload` o el botón «Flashear» de OpenPlane Studio. Después, la tecla `D` en la consola reinicia la placa en el cargador de arranque por sí sola.
- **La caja negra** escribe en la tarjeta microSD de la ranura de la placa. La batería se mide con el divisor de la placa (×11) y el sensor de corriente, por la tensión de su salida (40 A por voltio).
- S2 y S9–S12 quedan libres. Los servos se alimentan del BEC de la placa (la línea Vx) o del BEC del variador, como indica el manual de Matek; las masas de todas las fuentes van unidas.

### esp32-s3 (N16R8): la antigua placa principal, comprobada en el banco

`pio run -e esp32-s3`, placa `esp32-s3-devkitc-1` con los ajustes para el módulo N16R8 (16 MB de flash, 8 MB de PSRAM octal).

| Función | GPIO |
|---|---|
| Alerón, semiala izquierda | GPIO4 |
| Alerón, semiala derecha | GPIO5 |
| Timón de profundidad | GPIO6 |
| ESC (acelerador) | GPIO7 |
| Timón de dirección + rueda direccional | GPIO18 |
| Receptor (RX): iBUS, SBUS, CRSF | GPIO17 |
| I2C de los sensores SDA / SCL (MPU, BMP581, brújula) | GPIO41 / GPIO42 |
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
| MPU-6050 / GY-521 (la placa puede llevar un MPU6500, y es normal) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, AD0–GND, INT/XDA/XCL: sin conectar. Módulo MPU-6500 suelto (10 pines): igual, más **NCS–3.3V** (si no, el chip pasa a SPI) y FSYNC–GND; EDA/ECL: sin conectar. Con el chip hacia arriba y la flecha X apuntando al morro; el giro de los ejes del chip se ajusta con `IMU_ROTATION_CW_DEG` en `Config.h` (90 en nuestro clon) |
| BMP581 | VCC–3.3V (**solo 3.3V**: muchos módulos no llevan regulador propio), GND–GND, SCL–GPIO42, SDA–GPIO41, **SDO–GND** (dirección 0x46; no dejarlo al aire), **CSB–3.3V** (si no, el chip pasa a SPI), INT: sin conectar |
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
| Receptor (RX): iBUS, SBUS, CRSF | GPIO8 |
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
| Receptor (RX): iBUS, SBUS, CRSF | GPIO16 |
| I2C SDA (sensores) | GPIO21 |
| I2C SCL (sensores) | GPIO22 |

Su ventaja es que es la placa más común y barata de la gama, por lo que sirve para depurar el firmware en el banco, pero su asignación de pines no se ha comprobado en hardware.

---

## Conexión del receptor

El receptor se conecta con un solo cable de datos al **pin RX del receptor** de la
tabla de tu placa (`PIN_RC_RX`: PE7 en la DevEBox, GPIO17 en la esp32-s3, GPIO8 en
la esp32-c3, GPIO16 en la esp32-dev). El protocolo debe coincidir con el de tu
receptor; lo fija `RX_PROTOCOL` en `Config.h` (en OpenPlane Studio, el paso
«Emisora»):

| `RX_PROTOCOL` | Receptor | Qué va dónde |
|---|---|---|
| `IBUS` (por defecto) | FlySky FS-iA6B, FS-iA10B | puerto iBUS SERVO (salida iBUS) → pin RX |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper y la mayoría de los demás | salida SBUS → pin RX; no hace falta inversor: la propia UART de la placa invierte la señal |
| `CRSF` | ExpressLRS, TBS Crossfire y Tracer | TX del receptor → pin RX; el RX del receptor puede ir al pin TX (`PIN_RC_TX`), aún sin uso |

- **Masa (GND) del receptor** → **GND de la placa**. Es obligatorio: sin masa común el protocolo no funciona.
- **Alimentación del receptor**: 5 V de un BEC/regulador aparte o de los 5V de la placa.

La velocidad del puerto la fija el firmware: iBUS, 115200 baudios; SBUS, 100000
(8E2); CRSF, 420000 (la de ExpressLRS por defecto). Configura una emisora con
EdgeTX igual que la FS-i6 (mira el mapa de canales más abajo): CH5, ARM; CH6–CH8,
interruptores; CH9–CH10, potenciómetros.

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
| CH8 | SwD | SWD | Por defecto, la inversión del motor; solo cambia en DISARM (ver abajo) |
| CH9 | VrA | VRA | Por defecto, intensidad de la estabilización |
| CH10 | VrB | VRB | Por defecto, velocidad de crucero |

CH6–CH10 pueden ser **lo que quieras, con una sola línea** de `include/config/Controls.h`: cualquiera de los 12 modos, de las 11 funciones (flaps, freno, lanzamiento de carga, geovalla, zumbador, inversión…) y de los 8 potenciómetros. Al encender la placa, la asignación real se imprime en el monitor serie. Todo sobre los modos y las asignaciones está en [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

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
| **SwD** (CH8) | **inversión del motor**: solo en DISARM (RTH vuelve al interruptor con una línea) |
| **SwB** (CH6) | flaps |
| **VrA / VrB** (CH9/10) | intensidad de la estabilización / velocidad de crucero |

- **STABILIZE: «el stick fija el ángulo».** Con el stick a fondo, 45° de alabeo y 25° de cabeceo; al soltarlo, el avión se nivela solo. El acelerador es tuyo.
- **AUTO_TAKEOFF.** Despegue desde pista por etapas. Tras el ARM, el motor sigue parado hasta que subas el acelerador por encima de la mitad; el avión comprueba durante ~1 s que está como en la calibración y luego lo hace todo solo: cola arriba con el timón a picar, carrera, despegue con el ala, ascenso hasta 30 m, y después mantiene el rumbo y la altitud. El acelerador es del programa; acelerador abajo antes del despegue = despegue abortado. Necesita la [calibración en tierra del avión](#calibración-en-tierra-del-avión-horizonte-y-actitud-en-tierra).
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

**El ARM bloquea de verdad el acelerador:** mientras la aeronave no esté armada, el acelerador del ESC se mantiene forzosamente en cero, sea cual sea la posición del stick (en un variador bidireccional es el pulso neutro de 1500 µs). Antes de armar se comprueba que los sensores que necesita el modo elegido responden; por ejemplo, STABILIZE sin un IMU en funcionamiento no se arma hasta que cambies a MANUAL. De todos modos, compórtate como si la hélice pudiera empezar a girar en cualquier momento después del ARM.

### Inversión del motor (SwD)

El variador principal es un BLHeli 80A bidireccional: el motor gira en ambos sentidos, **SwD abajo**, hacia atrás; arriba, hacia delante. La inversión sirve para rodar marcha atrás en tierra.

- **El sentido solo cambia en DISARM.** Si accionas SwD con ARM, no pasa nada (en Serial aparece `MotorReverse: реверс НЕ переключён`, «inversión NO cambiada») y el motor sigue girando como antes. El nuevo sentido se aplica cuando desarmas. Así no se puede activar la inversión a todo gas y quemar el variador y el motor.
- **Durante 3 segundos tras cambiar el sentido no se puede armar** (`REVERSE_SETTLE_MS`): si desarmaste con gas, el motor aún gira por inercia; deja que se pare. Mientras tanto, el ARM se rechaza con el motivo en Serial.
- **Hacia atrás, solo tu gas y no más del 50 %** (`REVERSE_THROTTLE_LIMIT_PCT`): stick a fondo = media potencia. En inversión el piloto automático no toca el gas (las superficies de mando funcionan como siempre); si se pierde el enlace, el motor simplemente se para.
- Se ve en la OLED (`REV` en la primera línea) y en el panel (la insignia «Мотор: РЕВЕРС», «Motor: INVERSIÓN»).

**Configuración del variador en BLHeliSuite** (una vez, sin hélice): Motor Direction, **Bidirectional** (en BLHeli_32, «Bidirectional»/3D); PPM Min Throttle, 1000; PPM Max Throttle, 2000; Center, 1500. El variador se arma al ver el neutro de 1500 µs, y el firmware lo emite desde el primer pulso. Si con el stick hacia delante el motor gira al revés, pon `MOTOR_REVERSED = true` en `Config.h` (una línea en lugar de resoldar cables).

Un variador unidireccional normal: `ESC_BIDIRECTIONAL = false` en `Config.h`; si no, en «parado» (1500) dará medio gas. En ese caso la inversión nunca se activa.

### Failsafe

La pérdida de enlace tiene **prioridad absoluta** sobre todo lo demás:

- **aeronave armada, con GPS y punto de origen**: **regreso a casa con motor** (como en ArduPilot/INAV): rumbo al punto de origen, altitud de 40 m, círculos sobre él hasta que vuelva el enlace. En el OLED aparece `FSRTH`. Se desactiva con `FAILSAFE_RTH = false` en `Config.h`;
- **aeronave armada, sin GPS**: **planeo**: motor apagado; el piloto automático, en cualquier modo, incluso en MANUAL, mantiene las alas niveladas y el morro algo por debajo del horizonte (−3°), y los flaps se recogen. En el OLED aparece `RX LOST ... GLIDE`. Si pones `FAILSAFE_GLIDE_ROLL_DEG` = 10–20, el avión describirá una espiral suave sobre ti;
- **motor en inversión**: el motor se apaga en cualquier caso; el piloto automático solo maneja las superficies de mando (a casa, planeando);
- **sin armar** (en tierra) o con el IMU sin responder: motor apagado; alerones, timón de profundidad y timón de dirección en neutro (1500 µs); tras 10 s sin enlace suena el zumbador «estoy aquí» (si está soldado).

El firmware reconoce la pérdida de enlace de tres maneras:

1. **Ninguna trama del receptor durante más de 500 ms**: un cable roto, un receptor sin alimentación o un receptor que enmudece al perder el enlace (ExpressLRS lo hace por defecto).
2. **El propio receptor informa de la pérdida de enlace**: el indicador de failsafe en la trama SBUS, calidad de enlace 0 % en CRSF. No hay nada que configurar.
3. **Acelerador por debajo de 950 µs**: así avisa el receptor de que ha perdido la emisora. **Esto exige configurar el failsafe en la emisora** (véase más abajo): al perder el enlace, el FS-iA6B NO deja de enviar tramas, sino que repite los últimos valores de los sticks; sin esa configuración el firmware no detectará la pérdida de enlace y el avión seguirá volando con el último acelerador.

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
# STM32H743 DevEBox (placa principal)
pio run -e stm32h743-devebox -t upload

# Matek H743-WING V3 (controladora lista para usar): botón BOOT + USB
pio run -e matek-h743 -t upload

# esp32-s3 N16R8 (antigua placa principal)
pio run -e esp32-s3 -t upload

# esp32-c3 (antiguo prototipo)
pio run -e esp32-c3 -t upload

# esp32-dev (ESP32 clásica de 38 pines, para el banco)
pio run -e esp32-dev -t upload
```

Si no indicas `-e`, se compila la placa por defecto: `stm32h743-devebox`.

**STM32 DevEBox:** el primer flasheo va por USB DFU: un puente BT0→3V3, pulsa RST y luego el comando de arriba (Windows necesita el controlador WinUSB para «STM32 BOOTLOADER», que se instala con Zadig). Después, la tecla `D` en la consola reinicia la placa en el cargador de arranque por sí sola y el puente ya no hace falta. La consola va por el mismo USB-C.

**Matek H743:** el primer flasheo también va por USB DFU, pero sin puente: mantén pulsado el botón BOOT de la placa, conecta el USB y ejecuta el comando de arriba. Después, la tecla `D` en la consola.

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
| `c` | **calibración en tierra del avión**: horizonte y actitud en tierra (véase más abajo); si está en marcha, la cancela |
| `e` | señal: cabezada de los timones y melodía del variador; comprueba que el variador la toca (¡quita la hélice!) |
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
2. **Paso 1:** el avión está nivelado, como en vuelo horizontal (un avión con rueda de cola puede quedarse sobre sus ruedas: el horizonte lo fijará después la calibración en tierra del avión). No lo toques durante ~3 s.
3. **Paso 2:** levanta el **morro** de 30 a 60°, con las alas niveladas, y mantenlo quieto durante 1 s aproximadamente.
4. **Paso 3:** vuelve a bajar el morro, baja el **ala derecha** de 30 a 60° y mantenla así durante 1 s aproximadamente.

Cada paso se da por válido automáticamente (en el registro aparece «засчитано», es decir, «contado»). Al final se muestra el resultado («нос = +Y чипа, верх = −Z чипа», es decir, «morro = +Y del chip, arriba = −Z del chip») y «установка сохранена» («instalación guardada»). Si te has equivocado (has bajado el morro en vez de subirlo, o el ala izquierda en vez de la derecha), la calibración se rechaza con una explicación; repite `o`. El resultado se guarda en la flash y sobrevive a un reinicio. Comprobación: morro arriba → la P del OLED pasa a positivo; ala derecha abajo → la R pasa a positivo.

Mientras no exista una calibración de la instalación, rige el método antiguo: la placa debe ir con el chip hacia arriba, y el giro es `IMU_ROTATION_CW_DEG` en `Config.h`. Tras `o`, repite la calibración en tierra del avión: la actitud en tierra se midió desde el horizonte anterior.

### Calibración en tierra del avión: horizonte y actitud en tierra

Un avión con rueda de cola descansa en el suelo con el morro levantado; cuánto, nadie lo sabe de antemano. La calibración lo mide ella misma y lo guarda en la flash: el **horizonte** (0° de cabeceo y alabeo para todos los modos, ya no «como estaba al encender») y la **actitud en tierra**, de la que el despegue automático deduce cuánto hay que levantar la cola y dónde tocaría la pista la rueda de cola. Una vez tras el montaje (y de nuevo si se ha movido la placa o tras `o`). No hace falta portátil:

1. El avión sobre suelo llano, la emisora encendida, **sin ARM**, gas abajo.
2. **SwB (flaps) tres veces seguidas abajo y arriba** (en menos de 4 s), o `c` en la consola. Una señal: empieza.
3. **Paso 1: horizonte.** Levanta la cola hasta que el fuselaje quede horizontal, como en vuelo (las ruedas principales en el suelo; ayuda una app de nivel de burbuja en el móvil apoyado sobre el fuselaje), y mantenlo quieto ~2 s. Fíjate de paso en el hueco entre la hélice y el suelo: más o menos así rodará el avión en el despegue. Una señal: aceptado.
4. **Paso 2: actitud en tierra.** Apoya el avión en sus tres ruedas y suéltalo. A los ~2 s, dos señales: terminado; el registro muestra «Стоянка: нос вверх на 9.4°, крен 0.3°» («en tierra: morro arriba 9.4°, alabeo 0.3°»).

Tres señales: un error, no se guarda nada (el motivo está en la consola): el avión no se quedó quieto en un minuto, en el paso 2 el morro apenas está levantado (no se bajó la cola) o más de 25°, o el alabeo pasa de 10°. Para cancelar: el mismo gesto con SwB o mover un momento el interruptor de ARM (ese movimiento no arma). Con rueda de morro (`NOSE_WHEEL`), el paso 1 se puede hacer directamente sobre las ruedas si el avión está nivelado: el paso 2 se acepta entonces solo a los 5 s. El interruptor del gesto es `CAL_SWITCH` en `Config.h` (SwD no: controla la inversión del motor).

**Señales.** Una señal: los timones de profundidad y de dirección dan una cabezada (en el paso 1 tienes la cola en la mano y lo notas) y, sin ARM, canta el motor: los pulsos al variador desaparecen durante un segundo, el variador cree que ha perdido la señal y, cuando vuelven los pulsos, toca su melodía de listo, como al encenderlo. El motor no gira. Por PWM un variador no conoce otra orden de «pitido», así que compruébalo en el banco (**sin hélice**): tecla `e` en la consola; la melodía tiene que sonar. Si no suena, pon `SIGNAL_MOTOR = false` en `Config.h` y quedarán las cabezadas. Un zumbador o un LED con resistencia en PE15 (S3: GPIO38) se enciende durante las cabezadas; el LED D2 de la propia DevEBox está en PA1, la salida del alerón derecho, así que no sirve para las señales. Mientras suena una señal, el ARM se rechaza: puede que el variador aún no se haya rearmado. En el OLED, la línea de ángulos muestra `CAL1`, `CAL2` y, como resultado, `CALOK` o `CALER`.

### Comprobación previa al vuelo al encender

Cada vez que se enciende, el IMU dedica unos 2 s a calibrar el giroscopio y, de paso, se comprueba a sí mismo:

- **el avión está inmóvil**: si en ese momento se sostenía en las manos o se movía, el desfase del giroscopio será incorrecto;
- el acelerómetro en reposo marca 1g;
- **«arriba» coincide con la calibración de la instalación**: si la placa se ha cambiado de sitio o se ha dado la vuelta, se nota enseguida (un avión sobre su rueda de cola o en una pendiente no es problema; la tolerancia es de 45°).

**Enciende el avión mientras está quieto.** No es imprescindible que esté nivelado si la instalación está calibrada (`o` o la calibración en tierra del avión; si no, la posición al encender se convierte en el horizonte). El resultado figura en el registro: `предполётная проверка пройдена` («comprobación previa al vuelo superada») o `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА — <причина>` («COMPROBACIÓN PREVIA AL VUELO NO SUPERADA», seguida del motivo). Si no se supera, **el ARM en STABILIZE y AUTO_TAKEOFF queda prohibido** (el motivo se imprime al intentarlo), y el piloto automático no da correcciones en ningún modo, incluido el planeo por pérdida de enlace: con ángulos erróneos gobernaría en sentido contrario. El ARM en MANUAL sigue siendo posible. Para arreglarlo: deja el avión inmóvil y vuelve a conectar la batería (o pulsa `i`); si la placa se ha cambiado de sitio, pulsa `o`.

### Pantalla OLED

Si hay un OLED conectado (GPIO1/GPIO2), muestra lo siguiente 5 veces por segundo:

```
RX ok disarm STAB        <- enlace / ARM / modo / REV: motor hacia atrás (si se pierde el enlace, la línea sale invertida)
R  +1.2 P  -0.4 ROLL     <- alabeo / cabeceo, °; paso de calibración o etapa del despegue
Alt +0.3 Vz +0.1         <- altitud desde el punto de encendido, m / velocidad vertical, m/s
Hdg 123  Thr 1000        <- rumbo de la brújula / acelerador 1000..2000 = 0..100% (también hacia atrás), µs
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
  - gas hacia delante (sin hélice, mira el sentido de giro) → **la hélice tira hacia delante**;
  - en STABILIZE, inclina el avión con el ala derecha abajo → **alerón derecho abajo, izquierdo arriba** (las superficies lo devuelven a la horizontal); morro abajo → **timón de profundidad arriba**.
  Si algo no va bien, cambia una línea del bloque «Inversión» de `include/config/Config.h`: cada alerón, el timón de profundidad, el de dirección, AUX, el motor (`MOTOR_REVERSED`) y los flaps (`FLAPS_REVERSED`) se invierten por separado y no se toca nada más. No inviertas servos en la emisora: el stick y el piloto automático discreparán; un canal al revés (stick, interruptor, potenciómetro) se cambia con `CHANNEL_REVERSED`, en el mismo sitio.
- [ ] Al encender, el avión estaba inmóvil, y en el registro o en el panel no aparece `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА`; al inclinar el avión con las manos, la P y la R del OLED cambian en el sentido correcto.
- [ ] Para AUTO_TAKEOFF: la calibración en tierra del avión está hecha (al encender, el registro muestra «стоянка на колёсах — тангаж …»), la rueda de cola está entera y el avión está en la pista con viento de cara. Las primeras carreras, con límite de gas (potenciómetro `THROTTLE_LIMIT`) y la mano en el gas: abajo = despegue abortado.
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
Comprueba que el cable de datos del receptor está conectado justo al pin RX del receptor de la tabla de tu placa (PE7/GPIO17/GPIO8/GPIO16), no confundido con masa o alimentación, y que las masas del receptor y de la placa están unidas. Si el pinout coincide y los cables están bien pero sigue sin haber señal, comprueba que el receptor está emparejado (bind) con la emisora (en ExpressLRS, la misma binding phrase y versión que el transmisor) y que su salida coincide con `RX_PROTOCOL`: iBUS, SBUS o CRSF, no PPM.

**Un sensor (IMU, barómetro, brújula) indica «no responde» / NO_RESPONSE**
El firmware informa con sinceridad de que el sensor no responde, en lugar de dar ceros. Comprueba: (1) la alimentación del módulo, 3.3V y la masa de la placa; (2) SDA/SCL, en los pines I2C de tu placa; (3) la dirección en la línea: MPU 0x68 (AD0 a GND), BMP581 0x46 (SDO a GND, CSB a 3.3V; 0x47 si SDO va a 3.3V), BMP388 0x76 (SDO a GND, **CSB a 3.3V**; si no, el chip está en modo SPI), QMC5883P 0x2C, QMC5883L 0x0D. Si el sensor a veces responde y a veces no (o contesta en una dirección ajena), es un mal contacto en la protoboard: aprieta VCC/GND/SDA/SCL y, mejor, alimenta cada módulo directamente desde el 3.3V/GND de la placa. El comando `s` de la consola muestra los contadores de errores de I2C de cada sensor.

**Los ángulos del OLED están cambiados (el morro arriba cambia la R, no la P) o tienen el signo contrario**
Haz la calibración de la instalación del IMU (`o`, véase «Instalación del IMU»): no depende de cómo esté soldado el chip en el módulo ni de cómo esté colocado el módulo en el avión. Sin ella: en los clones del GY-521 el chip a veces está soldado girado respecto a las flechas impresas; gira los ejes en `Config.h` → `IMU_ROTATION_CW_DEG` (0/90/180/270). Comprobación: morro arriba → la P pasa a positivo; ala derecha abajo → la R pasa a positivo.

**ARM rechazado: «IMU: ...»**
No se ha superado la comprobación previa al vuelo del IMU (véase «Comprobación previa al vuelo al encender»): el avión se movió al encender; ponlo inmóvil y vuelve a conectar la batería; «arriba» no coincide con la calibración, se cambió la placa de sitio; haz `o`; «la placa no está con el chip hacia arriba», la instalación no está calibrada; haz `o`.

**ARM rechazado: «нет калибровки шасси» (no hay calibración del tren)**
AUTO_TAKEOFF con rueda de cola no sabe dónde tocaría la cola la pista. Haz la calibración en tierra del avión: SwB tres veces abajo y arriba sin ARM (o `c`).

**«Автовзлёт: старт отменён — тангаж не стояночный» (arranque cancelado: el cabeceo no es el de la actitud en tierra; los timones dieron tres cabezadas)**
El avión no está como en la calibración: no está sobre sus tres ruedas, hay pendiente, la rueda de cola está rota o la placa se ha movido. Ponlo nivelado; si se movió la placa, repite `o` y la calibración en tierra.

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
- El barómetro por defecto es ahora el BMP581 (el BMP388 del banco se comprobó en real; el BMP581 aún no se ha probado en hardware). La altitud es relativa al punto de encendido. La altitud absoluta sobre el nivel del mar se calcula según la atmósfera estándar, sin corrección por el tiempo.
- La brújula QMC5883P necesita calibración (`m` en la consola) con el avión ya montado: junto al motor y los cables los desfases son distintos de los de la protoboard. El rumbo todavía no tiene compensación de inclinación y ningún modo lo utiliza.
- La licencia es la OpenPlane License: MIT con atribución obligatoria al autor (Damir Lebedev), prohibición del uso militar y prohibición de dañar intencionadamente a personas o bienes sin su consentimiento; véase [LICENSE](LICENSE.md).

Si construyes tu propia aeronave siguiendo esta guía, vuela primero con control manual (MANUAL) y solo después pasa al piloto automático.
