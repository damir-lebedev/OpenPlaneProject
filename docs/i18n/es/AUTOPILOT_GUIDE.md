# Referencia del piloto automático de OpenPlane

> 🌐 Esta página es una traducción del [original en ruso](../../AUTOPILOT_GUIDE.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual.

Qué sabe hacer el piloto automático, cómo activar cada función y cómo asignarla a cualquier interruptor o potenciómetro de la emisora **con una sola línea**.

> Primero, con honestidad. Todos los modos están verificados con pruebas unitarias y simulaciones de vuelo en lazo cerrado (`test/native/test_sim`: todo el firmware pilota un modelo de avión). El modelo es simplificado y los coeficientes de `Config.h` son valores de partida: **cada modo se prueba primero a una altura de más de 50 m, con el dedo en el interruptor MANUAL**. Hasta ahora solo ha volado el modo manual (el primer prototipo, con una ESP32-C3); la estabilización se ha comprobado en el banco: las superficies responden a las inclinaciones en el sentido correcto. La placa STM32H743 compila y pasa las mismas pruebas; en el hardware, la placa DevEBox se ha comprobado sin sensores: la tarjeta SD, la caja negra y el control manual de los servos y del motor desde la emisora (grabado en vídeo); todavía no se le han conectado sensores ni se han probado en ella los modos del piloto automático.

---

## Contenido

1. [Cómo funciona, en un minuto](#cómo-funciona-en-un-minuto)
2. [Distribución predeterminada de la emisora](#distribución-predeterminada-de-la-emisora)
3. [Asignar una función con una sola línea](#asignar-una-función-con-una-sola-línea)
4. [Modos](#modos)
5. [Funciones (interruptores)](#funciones-interruptores)
6. [Potenciómetros](#potenciómetros)
7. [Pérdida de señal, geovalla, punto de origen](#pérdida-de-señal-geovalla-punto-de-origen)
8. [Un tubo de Pitot casero](#un-tubo-de-pitot-casero)
9. [Estación de tierra: panel Wi-Fi y MAVLink](#estación-de-tierra-panel-wi-fi-y-mavlink)
10. [Orden de puesta a punto de un avión nuevo](#orden-de-puesta-a-punto-de-un-avión-nuevo)
11. [Comprobación previa al vuelo del piloto automático](#comprobación-previa-al-vuelo-del-piloto-automático)
12. [Qué necesita cada modo](#qué-necesita-cada-modo)

---

## Cómo funciona, en un minuto

```
sticks ──────┐
             ├─► PilotSwitches (config/Controls.h) ─► modo, funciones, potenciómetros
interruptores┘                                              │
                                                            ▼
sensores (IMU, barómetro, brújula, GPS, tubo de Pitot) ─► Autopilot ─► órdenes a las superficies y al acelerador
                                                            │
                               FlightController: flaps, carga, cámara, zumbador, failsafe
                                                            ▼
                                               alerones · timón de profundidad · timón de dirección · ESC · AUX1 · AUX2
```

- El **modo** decide quién pilota: el piloto (MANUAL), el piloto con un asistente (STABILIZE, ALT_HOLD, ACRO), el piloto automático con correcciones del piloto (CRUISE, LOITER, RTH…).
- Las **funciones** se activan por encima de cualquier modo: flaps, aerofreno, lanzamiento de carga, geovalla…
- Los **potenciómetros** cambian un número de forma gradual: la intensidad de la estabilización, la velocidad de crucero, el radio de los círculos…
- En los modos con estabilización, **el stick fija el ángulo**, no la deflexión de la superficie: sueltas el stick y el avión vuelve solo al horizonte.
- El fallo de un sensor nunca «sacude» el avión: sin IMU, las superficies quedan en manos del piloto; sin barómetro, el piloto mantiene la altitud; sin GPS no hay navegación, y los modos que lo necesitan se comportan de forma segura (véase [la tabla](#qué-necesita-cada-modo)).

---

## Distribución predeterminada de la emisora

FS-i6 + FS-iA6B, iBUS, 10 canales (`config/Channels.h`).

| Canal | Control de la emisora | Por defecto |
|---|---|---|
| CH1–CH4 | sticks | alabeo, cabeceo, acelerador, timón de dirección (no se pueden reasignar) |
| CH5 | **SwA** | **ARM** (abajo = armado, solo con el acelerador abajo; no se puede reasignar) |
| CH6 | SwB | flaps (`Feature::FLAPS`) |
| CH7 | **SwC** (3 posiciones) | arriba **MANUAL** · centro **STABILIZE** · abajo **AUTO_TAKEOFF** |
| CH8 | SwD | **RTH** — regreso a casa, mientras esté activado |
| CH9 | VrA | intensidad de la estabilización (`Knob::STAB_GAIN`) |
| CH10 | VrB | velocidad de crucero (`Knob::CRUISE_SPEED`) |

Al encender la placa, el monitor serie imprime la distribución real, es decir, lo que de verdad está cargado en ella (el firmware imprime en ruso; «вверх / середина / вниз» significa arriba / centro / abajo):

```
SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (вверх / середина / вниз)
SwB (CH6): FLAPS, пока включён
SwD (CH8): RTH, пока включён
VrA (CH9): крутилка STAB_GAIN
VrB (CH10): крутилка CRUISE_SPEED
```

> Los canales 7–10 no vienen habilitados de fábrica en la FS-i6. Menú de la emisora: **Functions setup → Aux. channels**; asigna SwC, SwD, VrA y VrB.

---

## Asignar una función con una sola línea

Todo está en un único archivo: `include/config/Controls.h`:

```cpp
constexpr Binding BINDINGS[] = {
    Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_AUTO_TAKEOFF),
    Bind::feature(Channels::SWB, Feature::FLAPS),
    Bind::mode   (Channels::SWD, MODE_RTH),
    Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
    Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
};
```

| Forma de la línea | Qué hace |
|---|---|
| `Bind::modes(canal, arriba, centro, abajo)` | un interruptor de tres posiciones elige el modo |
| `Bind::modes(canal, arriba, abajo)` | de dos posiciones: dos modos |
| `Bind::mode(canal, modo)` | el modo **por encima** de los demás mientras el interruptor esté activado; al desactivarlo, vuelve el modo del interruptor de modos |
| `Bind::feature(canal, función)` | la función actúa mientras el interruptor esté activado |
| `Bind::knob(canal, potenciómetro)` | potenciómetro: centro = valor de `Config.h`, extremos = mínimo y máximo |

«Activado» significa que el canal está por encima de 1750 µs (en la FS-i6, el interruptor hacia abajo, hacia ti). Hasta que llega el primer fotograma del receptor, todos los canales se consideran desactivados: al dar corriente no se desplegará ni se soltará nada.

### Recetas listas para usar

```cpp
// Planeador de térmicas: SwD — vuelo a vela, SwB — autotrim, VrB — radio de los círculos
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_SOARING),
Bind::feature(Channels::SWB, Feature::AUTO_TRIM),
Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Alumno: solo estabilización, RESCUE en el «botón de pánico», sticks suaves
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_ALT_HOLD, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_RESCUE),
Bind::feature(Channels::SWB, Feature::GEOFENCE),
Bind::knob   (Channels::VRA, Knob::RATES),
Bind::knob   (Channels::VRB, Knob::MAX_BANK),

// Fotografía y entrega: cámara con estabilización, lanzamiento de carga, círculos sobre un punto
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_CRUISE, MODE_LOITER),
Bind::feature(Channels::SWB, Feature::PAYLOAD_DROP),
Bind::feature(Channels::SWD, Feature::CAMERA_STAB),
Bind::knob   (Channels::VRA, Knob::CAMERA_TILT),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Lanzamiento a mano sin tren de aterrizaje: SwD — LAUNCH, flaps graduales con el potenciómetro
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_LAUNCH),
Bind::feature(Channels::SWB, Feature::AIRBRAKE),
Bind::knob   (Channels::VRA, Knob::FLAPS),
Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
```

### El compilador atrapa los errores

La tabla se comprueba al compilar (`static_assert`), antes de que el firmware llegue al avión:

- los sticks y SwA (ARM) no se pueden asignar, y el número de canal debe ser menor que `Channels::COUNT`;
- un canal tiene una sola asignación;
- el interruptor de selección de modos (`Bind::modes`) no puede ser más de uno.

Si varios interruptores `Bind::mode` están activados a la vez, gana la fila superior de la tabla.

---

## Modos

El modo cambia cuando se **acciona** un interruptor. Un modo activado desde el panel o desde la estación de tierra se mantiene hasta que el piloto vuelva a accionar un interruptor de modos. En la OLED se muestra un nombre corto (entre paréntesis).

### MANUAL (MAN)
Superficies = sticks, como si no hubiera controlador de vuelo. Solo funcionan las funciones (flaps, aerofreno, carga) y el autotrim. **El modo de seguridad principal**: tenlo siempre en un interruptor bajo el dedo.

### STABILIZE (STAB) — «el stick fija el ángulo»
El stick de alabeo fija el ángulo de alabeo hasta `MAX_BANK_DEG` (45°, potenciómetro `MAX_BANK` de 15…60°), y el stick de cabeceo fija el ángulo hasta `STAB_MAX_PITCH_DEG` (25°). Lo sueltas y el avión se nivela solo. El acelerador queda en manos del piloto. El integrador del PID solo acumula cerca del objetivo (±10°), por eso tras una maniobra brusca el avión no «se pasa» del horizonte.
**Necesita:** IMU. Sin IMU, funciona como MANUAL.

### ALT_HOLD (ALT) — «mantén la altitud»
Como STABILIZE en el alabeo, y el timón de profundidad mantiene la altitud con el barómetro. Si mueves el stick de cabeceo, pilotas tú; si lo sueltas, el avión mantiene la **nueva** altitud. El acelerador es del piloto (da más gas; si no, no habrá velocidad suficiente para subir).
**Necesita:** IMU + barómetro.

### ACRO — «el stick fija la velocidad de giro»
Stick a fondo: 180°/s. Lo sueltas y el avión **mantiene la actitud que tenía** (incluso boca abajo), y el giroscopio amortigua las ráfagas. Para acrobacia.
**Necesita:** IMU. Sin IMU, superficies = sticks.

### CRUISE (CRZ) — «mantén rumbo, altitud y velocidad»
El avión vuela en línea recta a la altitud actual, con el acelerador automático (potenciómetro `CRUISE_SPEED`: 30…55…85 % de gas y, con tubo de Pitot, **velocidad aerodinámica** de 10…14…22 m/s). El stick de alabeo gira (lo sueltas y mantiene el nuevo rumbo), el de cabeceo cambia la altitud. Con tubo de Pitot funciona la protección contra la pérdida de sustentación. El rumbo se toma del GPS (con velocidad sobre el suelo > 3 m/s; vuelve a la brújula por debajo de 2 m/s); si no, de la brújula; y si no, del giroscopio. En la simulación, con viento cruzado de 4 m/s, la derrota sobre el suelo se mantiene dentro de ±5° y la altitud, dentro de ±3 m.
**Necesita:** IMU + barómetro; GPS o brújula para que el rumbo no se desvíe.

### LOITER (LOIT) — «orbita aquí»
Círculos en sentido horario sobre el punto donde se activó el modo, de radio 50 m (potenciómetro `LOITER_RADIUS` de 25…150 m), a la altitud actual, con el acelerador automático. La guía es un campo vectorial: desde lejos el avión entra en la circunferencia por la tangente, y sobre ella mantiene un alabeo anticipado. Sin GPS, simplemente un círculo con alabeo constante en el sitio.
**Necesita:** IMU + barómetro + GPS.

### RTH — «a casa»
Rumbo al punto de origen (el punto del ARM), altitud `RTH_ALTITUDE_M` = 40 m (si está por debajo, sube por el camino; si está por encima, se queda). Sobre el punto de origen, círculos de radio LOITER hasta que el piloto retome el control. Sin GPS o sin punto de origen, círculos en el sitio. El mismo modo lo activan la pérdida de señal y la geovalla.
**Necesita:** IMU + barómetro + GPS con punto de origen.

### AUTO_TAKEOFF (TKOFF) — «despegue con el acelerador»
Tras el ARM no pasa nada hasta que el piloto sube el gas por encima de la mitad. Después, el programa: 1 s de aceleración hasta el 100 % de gas con las alas niveladas, 2 s de rotación con un cabeceo de 15° y luego ascenso con un cabeceo de 10°, hasta que el piloto cambie de modo. Los sticks se suman al programa: se puede corregir el rumbo durante la carrera de despegue.
**Necesita:** IMU.

### LAUNCH (LNCH) — «lanzamiento a mano»
1. Armado y con el gas por encima de la mitad: el lanzamiento queda **preparado**, el motor está parado.
2. El lanzamiento: sobrecarga hacia delante > 1.5 g durante más de 40 ms.
3. A los 0.3 s (la mano ya está lejos de la hélice): gas al 100 %, ascenso con un cabeceo de 15° y alas niveladas, durante 6 s o hasta los 30 m.
4. Después, como CRUISE a la altitud alcanzada.

Cualquier movimiento de los sticks (> 150 µs) antes del lanzamiento lo cancela: el avión queda en manos del piloto.
**Necesita:** IMU (acelerómetro). En la simulación: un lanzamiento a 9 m/s desde la altura de la mano; el avión no toca el suelo ni una sola vez y en 15 s gana más de 15 m.

### AUTO_LAND (LAND) — «aterrizaje»
Motor apagado, planeo en el rumbo con un cabeceo de −4°; por debajo de 3 m según el barómetro, se redondea (+4°). El stick de alabeo corrige el rumbo de aproximación. Actívalo en un tramo recto, con viento de cara, a 20–40 m y con pista de sobra. En la simulación, el contacto con el suelo se produce con una velocidad vertical inferior a 1.5 m/s, con las alas niveladas y no de morro.
**Necesita:** IMU + barómetro (puesto a cero en el suelo al encender).

### SOARING (SOAR) — «vuelo a vela en térmicas»
Motor apagado, planeo. Un variómetro (con tubo de Pitot, de energía total, sin «térmicas» falsas por tirar del stick hacia atrás) por encima de 0.5 m/s durante más de 1.5 s indica una térmica: círculos con un alabeo de 25°. Si el ascenso medio en 8 s cae por debajo de −0.2 m/s, se sale de la térmica. Por debajo de 30 m, motor hasta los 100 m; a más de 400 m del punto de origen, planea de vuelta. En la simulación encuentra una térmica (núcleo de 3 m/s) y gana más de 50 m sin motor.
**Necesita:** IMU + barómetro; GPS, para volver al punto de origen.

### RESCUE (RESQ) — «sálvame»
Alas niveladas, morro a +8°, gas al 70 %: desde cualquier espiral. ¿Has perdido la orientación? Acciona el interruptor y respira. En la simulación, desde una espiral con alabeo de 70° y morro a −40°, en 4 s: alas niveladas y ascenso.
**Necesita:** IMU.

---

## Funciones (interruptores)

| Función | Qué hace | Detalles y valores (`Config.h`) |
|---|---|---|
| `FLAPS` | los dos alerones hacia abajo: flaperones | `FLAPS_DEPLOYED_US` = 220 µs, de forma gradual en 1 s; el alabeo actúa por encima |
| `AIRBRAKE` | los dos alerones hacia arriba: aerofreno, senda de planeo más pronunciada | `AIRBRAKE_US` = 250; tiene prioridad sobre los flaps |
| `AUTO_TRIM` | aprende a mantener el avión recto sin tocar los sticks: la orden constante a las superficies en vuelo nivelado «pasa» al trim | 20 %/s, hasta ±120 µs; se guarda en la flash tras el DISARM **en el suelo** |
| `TURN_COORDINATION` | timón de dirección hacia el viraje, morro arriba en el alabeo | siempre activada en los modos de navegación |
| `MOTOR_KILL` | motor apagado en cualquier modo, incluso en los automáticos | más fuerte que cualquier modo y que el acelerador |
| `BEEPER` | zumbador «estoy aquí» | sin interruptor suena solo: en el suelo, con la señal perdida > 10 s |
| `PAYLOAD_DROP` | servo AUX1 abierto mientras el interruptor esté activado | 1000 µs cerrado, 2000 abierto |
| `GEOFENCE` | a más de 500 m del punto de origen o a más de 120 m de altura: RTH | `GEOFENCE_ALWAYS_ON`: sin interruptor |
| `HOME_RESET` | punto de origen = punto actual (en el momento de activar el interruptor) | solo con buen GPS |
| `CAMERA_STAB` | la cámara en AUX2 mantiene su ángulo respecto al horizonte | se resta el cabeceo del avión |

## Potenciómetros

El centro del potenciómetro = valor por defecto de `Config.h`; los extremos son el mínimo y el máximo. Si no está asignado, rige el valor por defecto.

| Potenciómetro | Mínimo … centro … máximo | Dónde actúa |
|---|---|---|
| `STAB_GAIN` | ×0.25 … ×1 … ×2 | todos los modos con estabilización y ACRO: «más suave/más duro» |
| `MAX_BANK` | 15° … 45° … 60° | alabeo máximo desde el stick y desde la navegación |
| `CRUISE_SPEED` | gas 30 … 55 … 85 % (con Pitot: 10 … 14 … 22 m/s) | CRUISE, LOITER, RTH, el motor en SOARING |
| `FLAPS` | 0 … 50 … 100 % de flaps | flaps graduales en lugar de interruptor |
| `CAMERA_TILT` | −90° … 0° … +30° | ángulo de la cámara (AUX2) |
| `RATES` | 30 … 65 … 100 % del recorrido de los sticks | todos los modos: sensibilidad de los sticks |
| `LOITER_RADIUS` | 25 … 50 … 150 m | LOITER y círculos sobre el punto de origen |

> Un truco útil: el potenciómetro `STAB_GAIN` en VrA es un ajuste «en vivo» de los coeficientes durante el vuelo. Si oscila, redúcelo; si está flojo, auméntalo; después pasa el multiplicador a `Config.h`.

---

## Pérdida de señal, geovalla, punto de origen

**El punto de origen** se registra en el ARM si el GPS es bueno (fijación 3D, ≥ 6 satélites, precisión ≤ 5 m). Si en el momento del ARM el GPS aún no ha fijado, el punto se registra en cuanto fije. Para cambiarlo en el campo, la función `HOME_RESET`.

**Pérdida de señal** (no hay fotogramas iBUS durante > 0.5 s o el receptor envió un gas inferior a 950 µs: es el failsafe configurado en la emisora; véase `docs/PILOT_GUIDE.md`):

| Situación | Qué hace la aeronave |
|---|---|
| en el suelo (sin armar) | motor 0, superficies en neutro; a los 10 s, el zumbador |
| en el aire, con GPS y punto de origen | **RTH** con motor; sobre el punto de origen, círculos a 40 m |
| en el aire, sin GPS | **planeo**: motor apagado, alas niveladas, morro −3° |
| la señal vuelve | enseguida, el modo del interruptor del piloto |

Un regreso ya iniciado no pasa a planeo por una pérdida breve del GPS. `FAILSAFE_RTH = false`: solo planeo.

**Geovalla** (`GEOFENCE` o `GEOFENCE_ALWAYS_ON`): salir más allá de `FENCE_RADIUS_M` (500 m) o por encima de `FENCE_ALTITUDE_M` (120 m): RTH. Para retomar el control, hay que accionar un interruptor de modos para ponerlo en cualquier otra posición (un modo se activa al cambiar de posición). Volverá a actuar cuando el avión regrese al interior con un margen del 10 %.

---

## Un tubo de Pitot casero

Velocidad aerodinámica sin un sensor de presión diferencial comprado: **dos barómetros**.

```
        flujo incidente ─►  ┌──────────── tubo (PVC/latón, Ø4–6 mm) ──┐
                            │  BMP581 (I2C 0x47) — presión total      │  estanco
                            └─────────────────────────────────────────┘
   fuselaje: barómetro principal (BMP581 0x46 / SPL06 / BMP388) — presión estática

   velocidad  V = √(2·(P_tubo − P_estática − cero) / ρ),   ρ — de la presión estática y la temperatura
```

**Montaje.** El BMP581 (módulo con dirección 0x47: patilla SDO a VCC) se pega dentro de un tubo abierto solo hacia delante: la placa queda dentro de una cavidad estanca, con los cables sacados a través de sellador. El tubo apunta hacia delante, fuera de la corriente de la hélice (en el ala o sobre el morro). El segundo barómetro va dentro del fuselaje, protegido del flujo directo (espuma).

**Activación en el firmware**: `sensors/SensorSelection.h`: `SENSOR_KIT_LSM6DSV_PITOT` o `SENSOR_KIT_ICM45686_PITOT` (kits ya preparados), o `SENSOR_AIRSPEED = SENSOR_AIRSPEED_PITOT_BMP581` en un kit propio.

**Cero.** Dos barómetros siempre difieren un poco: la precisión absoluta de cada uno es de decenas de pascales, y esa es toda la diferencia de presión a baja velocidad (10 m/s ≈ 60 Pa). Durante el primer segundo tras el encendido, el firmware promedia la diferencia y la toma como cero. **El avión está quieto al encender, y el tubo está tapado con un dedo o un capuchón, o girado de cara al viento.** En la OLED, en el panel y en la telemetría, la velocidad aparece después de poner a cero.

**Calibración de `PITOT_SCALE`.** La presión dentro del fuselaje no es estrictamente estática. Con aire en calma, vuela una recta de ida y vuelta en CRUISE y compara la velocidad media sobre el suelo del GPS con la velocidad del tubo: `PITOT_SCALE = V_GPS / V_tubo`.

**Protección.** Si la diferencia es muy negativa durante más de 2 s (mangueras cambiadas, agua) o las lecturas del tubo tienen más de 0.2 s, no se da la velocidad: el piloto automático pasa al acelerador del potenciómetro y al rumbo sin ella. Comprobado en una simulación en lazo cerrado con ruido en ambos barómetros: el error de velocidad en vuelo es < 0.5 m/s.

Qué aporta el tubo: CRUISE mantiene la **velocidad aerodinámica** y no el gas; protección contra la pérdida de sustentación; variómetro de energía total para SOARING; una velocidad fiable en la telemetría.

---

## Estación de tierra: panel Wi-Fi y MAVLink

**ESP32: panel Wi-Fi.** Punto de acceso `OpenPlane-Debug`, contraseña `12345678`, la dirección sale en el monitor serie. Canales, salidas, todos los sensores, el modo, la navegación y las funciones activadas; se puede cambiar el modo y el PID. Los detalles, en `docs/PILOT_GUIDE.md`.

**STM32H743: MAVLink por radiomódem** (UART4: PD0 RX, PD1 TX, 57600 baudios, el valor por defecto de SiK). Sirven SiK de 433/868/915 MHz, ELRS en modo MAVLink y un ESP-01 como puente Wi-Fi. **QGroundControl** y **Mission Planner** ven la aeronave como un avión ArduPilot:

- horizonte, mapa con el punto de origen, velocidad (del tubo de Pitot, si lo hay), altitud, variómetro, gas;
- los modos, con los nombres de ArduPlane: STABILIZE → FBWA, ALT_HOLD → FBWB, CRUISE → CRUISE, LOITER → LOITER, RTH → RTL, AUTO_TAKEOFF/LAUNCH → TAKEOFF, SOARING → THERMAL, RESCUE → STABILIZE, AUTO_LAND → AUTO; el failsafe se muestra como RTL o CIRCLE;
- cinta de mensajes: ARM/DISARM, cambio de modo (con nuestro nombre), pérdida de señal, geovalla;
- **cambio de modo desde tierra**, con el botón de modo de la GCS (salvo AUTO: OpenPlane no tiene misiones);
- **parámetros** `RLL_KP … PTCH_KD`: el PID de alabeo y de cabeceo, que se leen y se cambian desde la ventana de parámetros de la GCS en pleno vuelo (no se conservan tras un reinicio: pasa los valores que funcionen a `Config.h`).

El ARM/DISARM desde tierra **se rechaza**: solo con el interruptor de la emisora. Para comprobar el flujo sin hardware: `OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink && python3 tools/check_mavlink.py /tmp/tlm.bin` (hace falta `pip install pymavlink`).

---

## Orden de puesta a punto de un avión nuevo

1. **MANUAL en el suelo:** sentidos de las superficies (`*_REVERSED` en `Config.h`), flaps hacia abajo, AUX. Comprobación de las salidas con `p` en la consola (¡sin hélice!).
2. **Sensores en el suelo:** `b`: exploración de los buses; `s`: estado; `o`: calibración de la instalación del IMU (3 posturas, una sola vez); `m`: brújula.
3. **STABILIZE en el suelo:** inclina el avión hacia la derecha: el alerón derecho debe ir **hacia abajo** (para nivelar). Morro arriba: timón de profundidad hacia abajo. Si no es así, los reversos o la instalación del IMU están mal.
4. **Primer vuelo en MANUAL**, ascenso a más de 50 m → STABILIZE. Si oscila, `STAB_GAIN` menor; si está flojo, mayor.
5. **AUTO_TRIM** en vuelo nivelado durante 20–30 s, aterrizaje, DISARM: el trim se guardará.
6. **ALT_HOLD** y luego **CRUISE**: comprueba la altitud y el rumbo; con tubo de Pitot, calibra `PITOT_SCALE`.
7. **LOITER** y **RTH**: en altura, a la vista, con el dedo en MANUAL.
8. Solo después, la **prueba de failsafe** (apaga la emisora en altura; el avión debe volver a casa) y el despegue y el aterrizaje automáticos.

## Comprobación previa al vuelo del piloto automático

- [ ] La distribución impresa al encender es la que esperas.
- [ ] Consola/OLED: IMU ok, la comprobación previa al vuelo del IMU superada (el avión estaba quieto al encender).
- [ ] El barómetro está a cero en el suelo (altitud ~0 en la OLED).
- [ ] Con tubo de Pitot: velocidad ~0 en el suelo; si soplas en el tubo, sube.
- [ ] GPS: fijación 3D, ≥ 6 satélites **antes del ARM**; si no, no habrá punto de origen ni RTH.
- [ ] STABILIZE en el suelo: los alerones y el timón de profundidad nivelan el avión, no lo vuelcan.
- [ ] El failsafe está configurado en la emisora (gas por debajo de 950 al perder la señal) y comprobado apagando la emisora **en el suelo** y sin hélice.
- [ ] MANUAL, bajo el dedo.

---

## Qué necesita cada modo

| Modo | IMU | Barómetro | GPS | Brújula | Pitot | Gas | Sin el sensor necesario |
|---|:-:|:-:|:-:|:-:|:-:|---|---|
| MANUAL | | | | | | piloto | — |
| STABILIZE | ● | | | | | piloto | superficies = sticks |
| ALT_HOLD | ● | ● | | | | piloto | el piloto mantiene la altitud |
| ACRO | ● | | | | | piloto | superficies = sticks |
| CRUISE | ● | ● | ○ | ○ | ○ | auto | rumbo por giroscopio (se desvía), altitud a cargo del piloto |
| LOITER | ● | ● | ● | | ○ | auto | círculo con alabeo en el sitio |
| RTH | ● | ● | ● | | ○ | auto | círculos en el sitio |
| AUTO_TAKEOFF | ● | | | | | programa | superficies = sticks + gas del piloto |
| LAUNCH | ● | ○ | | | | programa | superficies en neutro |
| AUTO_LAND | ● | ● | | ○ | | 0 | sin redondeo |
| SOARING | ● | ● | ○ | | ○ | 0 / motor | sin térmicas: planeo |
| RESCUE | ● | | | | | 70 % | superficies en neutro |

● significa obligatorio y ○, que mejora. Las comprobaciones en el código: `Autopilot.h` (`imuReady`, `baroReady`, `nav.gpsGood`); las pruebas: `test/native/test_autopilot_modes` (reacción de los modos a cada sensor) y `test/native/test_sim` (vuelos en lazo cerrado).
