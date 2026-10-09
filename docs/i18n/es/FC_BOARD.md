# Placa del controlador de vuelo: bloques de conectores

> 🌐 Esta página es una traducción del [original en ruso](../../FC_BOARD.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Una placa portadora para el ESP32-S3 DevKitC-1 (N16R8): el DevKit se inserta en dos tiras de hembrillas, y alrededor van los bloques de conectores JST-XH. Este documento responde a tres preguntas: qué conectores montar en bloques, dónde poner los condensadores y qué se enchufa en cada sitio. Los pines coinciden con `include/config/Config.h` (el bloque `BOARD_ESP32_S3`).

La placa está pensada para ser **de una sola cara**: los GPIO están elegidos de modo que los pines de cada bloque vayan seguidos a lo largo de la tira del DevKit y las pistas de señal se abran en abanico sin cruzarse. No he comprobado el trazado en un CAD. Si en algún punto no cuadra, pon un puente de cable por el lado de los componentes: de uno a tres en una placa así es normal.

**Todos los conectores son JST-XH, de 1 a 5 contactos.** Los cables no se sueldan a la placa: en los cables de los servos, del ESC, del receptor y de los módulos se crimpan las carcasas hembra correspondientes. El XH tiene guía, así que no se puede enchufar al revés. Cada contacto soporta ~3 A.

---

## 1. Esquema de distribución

Vista desde arriba, por el lado de los componentes. Los conectores USB del DevKit están en el borde inferior.

```
                    arriba: antena del ESP, sin cobre debajo
+----------------------------------------------------------------------+
| anillo de GND por todo el borde                                      |
|  (470 µF)              3V3 --------------------+    +- C5 IMU        |
|  A1 AIL-L               ^  (tras el DevKit)    |    +- C4 BARO       |
|  A2 AIL-R               |                      |    +- C3 I2C-B      |
|  A3 ELE          +------+-------------+        |    +- C2 I2C-A      |
|  A4 ESC <-BEC    | J1-1,2: 3V3        |        +--->+- C1 OLED       |
|  A5 AUX1  <------| J1-4..11   J3-4..7 |-------------^  riel I2C      |
|  A6 AUX2         |                    |                              |
|  A7 RC           |      DevKit        | J3-8..10 --> D1 GPS          |
|  A8 RUD          |     ESP32-S3       |              D2 BUZ   [Q1]   |
|  (470 µF)        |                    |                              |
|  B1 BAT   <------| J1-12,13           | J3-17,18 --> D3 AUX3         |
|  B2 TELEM <------| J1-15,16           |              D4 LIGHT [Q2]   |
|                  | J1-21: 5V          |                  ^           |
|                  +----[USB]--[COM]----+                  | 5V lógica |
|    [diodo 1N5822] --> 5V lógica --- abajo, bajo el USB --+           |
+----------------------------------------------------------------------+
```

Cuatro zonas:

| Zona | Dónde | Qué hay | Alimentación |
|---|---|---|---|
| **A** Servos | el borde izquierdo, frente a J1-4…11 | 8 × XH-3: superficies, ESC, AUX, receptor | 5V de servos (sucio) |
| **B** Batería, telemetría | abajo a la izquierda, frente a J1-12…16 | sensores de batería y de corriente, el radiomódem | 5V lógica |
| **C** 3V3 | arriba a la derecha, frente a J3-4…7 | OLED, sensores en el riel I2C, conectores I2C | 3V3 |
| **D** 5V lógica | abajo a la derecha, frente a J3-8…18 | GPS, zumbador, AUX3, luces | 5V lógica |

A la izquierda está la parte de potencia (servos, ESC, batería), y a la derecha, los sensores y las comunicaciones. La corriente de los servos se queda a la izquierda y no pasa junto a los sensores.

---

## 2. Las tiras del DevKit: qué pata va adónde

El orden de las patas es el del ESP32-S3-DevKitC-1 (2 × 22). Compáralo con la serigrafía de tu placa y mide la distancia entre las filas antes de dibujar nada. La numeración va de la antena al USB.

| J1 (fila izquierda) | GPIO | A | | J3 (fila derecha) | GPIO | A |
|---|---|---|---|---|---|---|
| 1 | 3V3 | 3V3 por arriba → zona C | | 1 | GND | — |
| 2 | 3V3 | (igual) | | 2 | 43 | — (consola «COM») |
| 3 | RST | — | | 3 | 44 | — (consola «COM») |
| 4 | 4 | A1 AIL-L | | 4 | 1 | C1 OLED SDA |
| 5 | 5 | A2 AIL-R | | 5 | 2 | C1 OLED SCL |
| 6 | 6 | A3 ELE | | 6 | 42 | riel I2C: SCL |
| 7 | 7 | A4 ESC | | 7 | 41 | riel I2C: SDA |
| 8 | 15 | A5 AUX1 | | 8 | 40 | D1 GPS: TX |
| 9 | 16 | A6 AUX2 | | 9 | 39 | D1 GPS: RX |
| 10 | 17 | A7 RC (iBUS) | | 10 | 38 | D2 zumbador (mediante Q1) |
| 11 | 18 | A8 RUD | | 11 | 37 | — PSRAM |
| 12 | 8 | B1 VBAT (ADC) | | 12 | 36 | — PSRAM |
| 13 | 3 | B1 CURR (ADC) | | 13 | 35 | — PSRAM |
| 14 | 46 | — strapping | | 14 | 0 | — botón BOOT |
| 15 | 9 | B2 TELEM: TX | | 15 | 45 | — strapping |
| 16 | 10 | B2 TELEM: RX | | 16 | 48 | — LED RGB |
| 17 | 11 | libre | | 17 | 47 | D3 AUX3 |
| 18 | 12 | libre | | 18 | 21 | D4 LIGHT (mediante Q2) |
| 19 | 13 | libre | | 19 | 20 | — USB |
| 20 | 14 | libre | | 20 | 19 | — USB |
| 21 | 5V | entrada de 5V lógica (después del diodo) | | 21 | GND | tierra de la zona D |
| 22 | GND | tierra de la zona B | | 22 | GND | tierra de la zona D |

En el banco, el firmware usa los GPIO11–14 libres para el SPI (ICM-42688); en esta placa el SPI no está cableado.

---

## 3. Reglas para caber en una sola capa

1. **Componentes de agujero pasante arriba, SMD abajo.** Las hembrillas del DevKit, los XH, los electrolíticos y el diodo van por el lado de los componentes. Los 0805, 0603 y SOT-23 se sueldan directamente sobre el cobre. Con el método de transferencia de tóner (la plancha), el dibujo del cobre se imprime en espejo.
2. **En cada conector, la señal queda más cerca del DevKit, la alimentación más lejos y el GND hacia el borde.** Por eso en todos los conectores el **contacto 1 es el más cercano al DevKit**. Así las pistas de señal no cruzan la alimentación. La excepción es el riel I2C (punto 5).
3. **El GND es un plano de cobre por todo el perímetro** (un anillo). Los contactos extremos de los conectores salen directamente a él.
4. **De un lado del DevKit al otro pasan solo dos líneas.** El 3V3 sube desde J1-1/2, rodea el extremo superior del DevKit y va hacia la derecha: más allá del borde de la placa del DevKit, no bajo la antena. La 5V lógica va desde J1-21 por debajo del DevKit hacia abajo y por el borde inferior hacia la derecha, bajo los conectores USB (allí solo hay pistas; el enchufe queda más arriba).
5. **El riel I2C.** Cuatro pistas paralelas con paso de 2.54 mm, que salen del DevKit hacia fuera: **3V3 · GND · SCL · SDA**. Es el orden de los pines de los módulos GY (VCC GND SCL SDA). Las hembrillas y los conectores se colocan atravesados sobre el riel, como vagones: cada pista pasa por su propio contacto. El riel empieza en J3-6/7, se mete bajo el OLED y sube a lo largo de la fila derecha. Si no cabe en altura, dóblalo hacia la izquierda por encima del extremo superior del DevKit; el orden de las líneas se conserva en el giro.
6. **No pases pistas entre las patas del DevKit**: el paso de 2.54 es demasiado estrecho para la transferencia de tóner. Bajo el propio DevKit sí se puede: allí hay 11 mm hasta su placa.
7. **Los componentes de agujero pasante son puentes gratis.** Una pista pasa sin problema bajo el cuerpo del diodo (paso de patas de 12.5–15 mm) y entre las patas de un electrolítico (5 mm).
8. **0805 entre los contactos del conector.** Con el paso del XH, de 2.5 mm, un condensador 0805 se suelda directamente entre los contactos contiguos de +5V y GND, por el lado del cobre.
9. **Ancho de las pistas:** 5V de servos y GND de servos, desde 2 mm; 5V lógica, desde 1 mm; señales, 0.4–0.5 mm.
10. **Rótulos en la serigrafía:** el número del conector (A1, B2…), la inscripción y una flecha junto al contacto 1.

---

## 4. Bloques de conectores

### Bloque A — servos: 8 × XH-3, borde izquierdo, en columna frente a J1-4…11

Los contactos de cada conector:
**1 — señal** (más cerca del DevKit) · **2 — +5V de servos** · **3 — GND** (hacia el borde).
Es el orden del cable de un servo: naranja, rojo, marrón.

| Conector | Rótulo | Qué enchufar | GPIO (pata) |
|---|---|---|---|
| A1 | AIL-L | el alerón izquierdo | 4 (J1-4) |
| A2 | AIL-R | el alerón derecho | 5 (J1-5) |
| A3 | ELE | el timón de profundidad | 6 (J1-6) |
| A4 | ESC | el variador: señal de gas; **por el cable rojo, la entrada del BEC de 5V** | 7 (J1-7) |
| A5 | AUX1 | el servo de lanzamiento de carga | 15 (J1-8) |
| A6 | AUX2 | flaps (dos servos con un cable en Y) o cualquier servo | 16 (J1-9) |
| A7 | RC | el receptor FS-iA6B, puerto iBUS SERVO (el receptor se alimenta desde aquí) | 17 (J1-10) |
| A8 | RUD | el timón de dirección + la rueda | 18 (J1-11) |

Qué más se suelda en el bloque:

- **El bus de +5V de servos**: la columna central de contactos, con una pista de 2 mm como mínimo. **GND**: la columna exterior, que a la vez es parte del anillo de GND.
- **330 Ω (0603)** en serie con cada línea de señal, junto al conector. Si llegan 5 V a un contacto de señal (un servo averiado, un crimpado defectuoso), el pin del ESP sobrevivirá.
- **10 kΩ (0603)** entre la señal del A4 ESC y GND: mientras el ESP se reinicia, no llega basura al variador.
- **100 nF (0805)** entre los contactos 2 y 3 de cada conector.
- El **A4 ESC** lleva además 10 µF + 100 pF (0805): la entrada de alimentación de toda la placa (filtrado de baja, alta y muy alta frecuencia).
- El **A7 RC** lleva además 10 µF (0805): el receptor es sensible a las caídas de tensión.
- **2 × 470 µF 16 V** en el bus de servos, uno en cada extremo de la columna (sobre A1 y bajo A8): el positivo al bus y el negativo al anillo. Dentro de la columna el negativo no puede llegar al anillo, y 3 cm de pista ancha no influyen en un electrolítico.

Un zócalo para el ESC aguanta ~3 A. Para 4–6 servos MG90S es suficiente. Si va a haber más servos y más potentes, añade junto al A4 un zócalo de alimentación aparte desde el BEC.

### Bloque B — batería y telemetría, abajo a la izquierda, bajo los servos

**B1 BAT — XH-5** (el único XH-5 de la placa: un cable con 12–17 V no entra en ningún otro conector)

| Contacto | Qué | Adónde va en la placa |
|---|---|---|
| 1 | **VBAT** — el positivo de la batería con un cable fino (el «+» extremo del conector de balanceo, o desde el conector de la batería, no a través del ESC) | un divisor de 56 kΩ / 10 kΩ → GPIO8 (J1-12) |
| 2 | vacío — un hueco entre la tensión de la batería y todo lo demás | — |
| 3 | **CURR** — la salida del sensor de corriente | un divisor de 10 kΩ / 15 kΩ → GPIO3 (J1-13) |
| 4 | +5V lógica — alimentación del sensor de corriente | el bus de 5V lógica |
| 5 | GND | el anillo |

- **El divisor de VBAT:** 56 kΩ arriba, 10 kΩ abajo, 100 nF en paralelo con el de abajo. 3S (12,6 V) → 1,91 V, 4S (16,8 V) → 2,55 V, con margen hasta el límite del ADC (~3,1 V). No hace falta un cable de masa aparte: la masa es común a través del ESC, así que el contacto 5 se puede dejar sin crimpar.
- **El divisor de CURR:** 10 kΩ arriba, 15 kΩ abajo, 100 nF en paralelo con el de abajo. Un sensor Hall de 5 V (ACS758 y similares) da como máximo 5 V → 3,0 V en el pin. Si la salida del sensor es de 3,3 V, la resistencia de arriba es de 0 Ω y la de abajo no se suelda.
- Pon los divisores justo en el conector y toma la masa de su contacto 5: así al ESP llega una sola pista.
- ¿Todavía no hay sensor de corriente? Simplemente no crimpes los contactos 3–4.

**B2 TELEM — XH-4:** un radiomódem de telemetría o iBUS-SENS.

| Contacto | Qué | GPIO (pata) |
|---|---|---|
| 1 | TX → al RX del módem | 9 (J1-15) |
| 2 | RX ← del TX del módem | 10 (J1-16) |
| 3 | +5V lógica | — |
| 4 | GND | — |

- 10 µF + 100 nF entre los contactos 3 y 4. Para un módem de 1 W añade un electrolítico de 470 µF.
- El firmware aún no admite TELEM: las tres UART están ocupadas (consola, iBUS, GPS). Para activarlo hay que pasar la consola al USB integrado. Es un cambio de firmware; el conector ya se cablea desde ahora.

### Bloque C — 3V3: la pantalla y los sensores, arriba a la derecha

Todo lo de esta zona va en el **riel I2C** (sección 3, punto 5): cuatro pistas **3V3 · GND · SCL · SDA** que salen del DevKit hacia fuera. SCL sale del GPIO42 (J3-6) y SDA, del GPIO41 (J3-7). El 3V3 llega por arriba desde J1-1/2. El OLED es el más bajo del riel, y sobre él, por orden, C2–C5.

**C1 OLED — XH-4.** Toma la alimentación del riel y los datos, de su propio bus (GPIO1/2), que llegan por el lado interior.

| Contacto | Qué | De dónde |
|---|---|---|
| 1 | SDA | GPIO1 (J3-4) |
| 2 | SCL | GPIO2 (J3-5) |
| 3 | 3V3 | el riel |
| 4 | GND | el riel |

Es el orden de los pines del módulo OLED (GND VCC SCL SDA) al revés, así que el cable va sin cruces.

**C2 I2C-A y C3 I2C-B — XH-4**, idénticos:

| Contacto | Qué |
|---|---|
| 1 | 3V3 |
| 2 | GND |
| 3 | SCL |
| 4 | SDA |

- **C2** — la brújula: un GY-273 en un mástil (un cable directo, con el mismo orden que el módulo) o la brújula de un módulo GPS. En el GPS solo se crimpan GND, SCL y SDA: la brújula recibe la alimentación por el cable del GPS.
- **C3** — de reserva: un sensor de velocidad aerodinámica (MS4525DO), un telémetro, etc.

**C4 BARO — un zócalo 1×4 para un módulo BMP581:** 1 — VCC, 2 — GND, 3 — SCL, 4 — SDA.

- En el propio módulo suelda puentes de alambre **CSB→VCC** y **SDO→GND** (dirección 0x46; la 0x47 queda para el tubo de Pitot). Sin CSB→VCC el chip pasa a modo SPI; con SDO al aire, la dirección baila. Los demás pines del módulo quedan al aire.
- Solo 3V3 del riel: muchos módulos BMP581 no llevan regulador propio.
- El orden de los pines varía de un módulo a otro: comprueba el tuyo. Si no coincide, el módulo va con un cable a C3 y el zócalo no se monta.
- Encima, un trozo de espuma de poro abierto (contra el flujo de aire y la luz).

**C5 IMU — un zócalo 1×8 para un GY-521:**

| Contacto | Pin del módulo | A dónde |
|---|---|---|
| 1 | VCC | el riel de 3V3 |
| 2 | GND | el riel de GND |
| 3 | SCL | el riel de SCL |
| 4 | SDA | el riel de SDA |
| 5, 6 | XDA, XCL | a nada |
| 7 | AD0 | al plano de GND fuera del riel (dirección 0x68) |
| 8 | INT | a nada |

Cómo esté girado el IMU en la placa no importa: la instalación la fija la calibración `o` (PILOT_GUIDE, «Instalación del IMU»).

Un módulo MPU-6500 suelto (10 pines: VCC GND SCL SDA EDA ECL AD0 INT NCS FSYNC) no entra en este zócalo: el orden de los pines es otro. Va con un cable a C3 (VCC, GND, SCL, SDA) y en el módulo lleva los puentes **NCS→VCC** (si no, el chip pasa a modo SPI), **AD0→GND** (dirección 0x68) y **FSYNC→GND**.

Los condensadores del bloque: **10 µF + 100 nF** en el riel junto a C1 (donde empieza el 3V3), **100 nF** entre los contactos 1 y 2 en C2–C5. Las resistencias de pull-up del I2C ya están en los módulos; no las pongas en la placa.

### Bloque D — 5V lógica: GPS, zumbador, luces, abajo a la derecha

El bus de 5V lógica llega por abajo (desde bajo el DevKit, por el borde inferior) y sube por el borde derecho a través de los contactos «+5V» de todos los conectores del bloque.

**D1 GPS — XH-4.** Va justo bajo el riel, para que el conector del GPS y el de su brújula (C2) queden cerca.

| Contacto | Qué | GPIO (pata) |
|---|---|---|
| 1 | TX → al RX del GPS | 40 (J3-8) |
| 2 | RX ← del TX del GPS | 39 (J3-9) |
| 3 | +5V lógica | — |
| 4 | GND | — |

10 µF + 100 nF entre los contactos 3 y 4.

**D2 BUZ — XH-2:** un zumbador activo de 5 V, para encontrar el avión en la hierba y para avisar de la batería y del ARM.

| Contacto | Qué |
|---|---|
| 1 | el «−» del zumbador → interruptor Q1 |
| 2 | +5V lógica → el «+» del zumbador |

**D3 AUX3 — XH-3:** 1 — la señal del GPIO47 (J3-17) a través de 330 Ω, 2 — +5V lógica, 3 — GND, más 100 nF entre 2 y 3. Sirve para un pulsador, los datos de una tira de LED, el disparador de una cámara. **No cuelgues aquí un servo:** es 5V lógica, y su corriente pasaría por el diodo y haría oscilar la alimentación del ESP.

**D4 LIGHT — XH-2:** un interruptor para una carga de hasta ~0,5–1 A: luces de navegación, un faro, el electroimán de lanzamiento.

| Contacto | Qué |
|---|---|
| 1 | el «−» de la carga → interruptor Q2 |
| 2 | +5V lógica → el «+» de la carga |

**Los interruptores Q1 y Q2** — la misma huella SOT-23. En el BC817 y en el Si2302 las patas coinciden en su función:

| Pata del SOT-23 | BC817 | Si2302 | A dónde |
|---|---|---|---|
| 1 | base | puerta | ← 1 kΩ ← GPIO (38 para Q1, 21 para Q2); 10 kΩ de la pata 1 a GND |
| 2 | emisor | fuente | GND (la masa de la zona — J3-21/22) |
| 3 | colector | drenador | contacto 1 del conector (D2 / D4) |

- **Q1 (zumbador):** vale cualquiera de los dos.
- **Q2 (luces):** **Si2302**; el BC817 se calienta con cientos de miliamperios.
- Los 10 kΩ mantienen el interruptor cerrado mientras el ESP arranca: el zumbador no chilla y las luces no parpadean.
- Si la carga es una bobina (un electroimán, un zumbador magnético), pon un diodo SS14 o 1N4148 en paralelo con el conector, con el cátodo a +5V. Deja sitio para él entre los contactos 1 y 2.

---

## 5. Alimentación y todos los condensadores

```
 ESC (BEC 5V/5A) ──► A4 ──► bus 5V de servos ──┬──► A1…A8 (servos, receptor)
                                               │    2×470 µF en los extremos de la columna
                                               │
                                               └──► diodo 1N5822 ──► 5V lógica ──┬──► J1-21 (5V DevKit)
                                                                                 ├──► B1, B2 (sensor de corriente, módem)
                                                                                 └──► D1…D4 (GPS, zumbador, AUX3, luces)
 DevKit: su propio regulador de 3V3 ──► J1-1/2 ──► por arriba ──► riel C (OLED, sensores, conectores I2C)
```

- **La entrada es una sola: A4 ESC.** No hace falta un conector de alimentación aparte.
- **El diodo Schottky 1N5822** (3 A, de agujero pasante; el sustituto SMD es el SS34): hay que comprarlo. Hace tres cosas:
  - USB y BEC no se estorban: se puede dejar el USB enchufado con la batería conectada;
  - cuando los servos hunden el bus, el electrolítico de la lógica no se descarga hacia los servos y el ESP no se reinicia;
  - su cuerpo de agujero pasante hace de puente sobre la pista de GND en la esquina junto a J1-22.

  El ánodo va al extremo inferior del bus de servos y el cátodo, a J1-21. No pongas un 1N5819 (1 A): por el diodo pasan el ESP, el módem, el GPS y las luces.
- Los servos no se alimentan solo con el USB; el diodo los desconecta. Es lo previsto. El GPS, el módem y el zumbador funcionan desde el USB sobre la mesa solo si el pin de 5V del DevKit da alimentación desde el USB. Algunos clones llevan ahí un diodo propio, y entonces no funcionan: es normal.
- Los 3.3 V proceden únicamente del regulador del DevKit y solo para la zona C.

**Todos los condensadores en una tabla** (cerámicos: 0805; electrolíticos: 16 V):

| Dónde | Qué | Para qué |
|---|---|---|
| Bus de servos, sobre A1 y bajo A8 | 470 µF + 470 µF | las caídas cuando todos los servos se mueven a la vez |
| A4 ESC, entre + y GND | 10 µF + 100 nF + 100 pF | la entrada de alimentación: baja, alta y muy alta frecuencia |
| A1–A3, A5–A8 | 100 nF en cada uno | el ruido de los motores de los servos, en el origen |
| A7 RC | + 10 µF | el receptor |
| 5V lógica, en J1-21 | 470 µF + 10 µF + 100 nF | sostiene al ESP cuando se hunde el BEC |
| Riel de 3V3, en C1 | 10 µF + 100 nF | alimentación de los sensores |
| C2–C5 | 100 nF en cada uno | |
| B1: la entrada del ADC de VBAT y CURR | 100 nF en cada una, en paralelo con la resistencia de abajo | el filtro del ADC |
| B1: el +5V del sensor de corriente | 100 nF entre los contactos 4 y 5 | |
| B2 TELEM | 10 µF + 100 nF (un módem de 1 W: + 470 µF) | los picos de corriente del transmisor |
| D1 GPS | 10 µF + 100 nF | |
| D3 AUX3 | 100 nF | |

Un cerámico 0805 de 10 µF pierde hasta la mitad de su capacidad a 5 V: está tenido en cuenta, y de todos modos hay electrolíticos cerca.

Puntos de prueba: **5VS** (el bus de servos), **5VL** (5V lógica), **3V3**, **GND**. Son cómodos para medir con un polímetro.

---

## 6. Qué se enchufa en cada sitio

| Dispositivo | Conector | Notas |
|---|---|---|
| Los servos de los alerones, del timón de profundidad y del timón de dirección | A1, A2, A3, A8 | la señal va al contacto 1 |
| ESC | A4 | el cable rojo es la entrada del BEC |
| El receptor FS-iA6B | A7 | el puerto iBUS SERVO, un cable normal de 3 hilos |
| El servo de lanzamiento de carga, los flaps | A5, A6 | |
| GY-521 (MPU6500) | C5, zócalo | |
| BMP581 | C4, zócalo | puentes en el módulo CSB→VCC, SDO→GND (0x46) |
| GY-273 (brújula) o la brújula del GPS | C2 | lejos de los cables de potencia, mejor en un mástil |
| OLED 128×64 | C1 | |
| Sensor de velocidad aerodinámica y similares | C3 | |
| GPS u-blox M10 | D1 + C2 | los 6 cables se reparten en dos carcasas: D1 (alimentación, UART) y C2 (brújula) |
| Tensión de la batería, sensor de corriente | B1 | |
| Radiomódem / iBUS-SENS | B2 | |
| Zumbador | D2 | |
| Luces, faro | D4 | |

Colocación en el avión:

- Monta la placa sobre un soporte blando (espuma, almohadillas de gel), más cerca del centro de gravedad: la vibración del motor estropea los ángulos.
- Mantén los cables de potencia (batería → ESC → motor) lejos de la zona C y de la brújula.
- El barómetro va bajo espuma; el IMU, como quieras (calibración `o`).

---

## 7. Lista de componentes

| Componente | Cant. | Dónde |
|---|---|---|
| Zócalo PBS 1×22 (para el DevKit) | 2 | |
| XH-3 acodado o recto | 9 | A1–A8, D3 |
| XH-4 | 5 | B2, C1, C2, C3, D1 |
| XH-5 | 1 | B1 |
| XH-2 | 2 | D2, D4 |
| Zócalos PBS 1×8 y 1×4 | 1 de cada | C5, C4 |
| Diodo Schottky 1N5822 (o SS34) | 1 | **hay que comprarlo** |
| Electrolítico de 470 µF 16 V | 3 (+1 para un módem potente) | bus de servos ×2, 5V lógica |
| 0805 de 10 µF | 6 | A4, A7, 5V lógica, 3V3, B2, D1 |
| 0805 de 100 nF | 20 | véase la tabla de condensadores |
| 0805 de 100 pF | 1 | A4 |
| 0603 de 330 Ω | 9 | señales A1–A8, D3 |
| 0603 de 10 kΩ | 5 | ESC a GND, parte baja de VBAT, parte alta de CURR, pata 1 de Q1 y Q2 |
| 0603 de 56 kΩ | 1 | parte alta de VBAT |
| 0603 de 15 kΩ | 1 | parte baja de CURR |
| 0603 de 1 kΩ | 2 | a la pata 1 de Q1 y Q2 |
| BC817 o Si2302 | 1 | Q1 (zumbador) |
| Si2302 | 1 | Q2 (luces) |
| SS14 / 1N4148 | 0–2 | solo para bobinas en D2 y D4 |

La placa sale de unos 80×95 mm: la columna de servos y el riel de sensores sobresalen por encima del extremo superior del DevKit. Si no cabe en el fuselaje, la forma más sencilla de reducirla es llevar BARO y el IMU a cables hacia C3 y acortar el riel.

---

## 8. Qué GPIO no hay que tocar

0, 45, 46: de ellos depende el modo de arranque; 19/20: USB; 26–37: la flash y la PSRAM del módulo N16R8; 43/44: el conector «COM» (consola); 48: el LED RGB. Con este plan solo quedan libres los GPIO11–14 (en el banco, SPI).

---

## 9. Antes del primer encendido

1. Sin el DevKit y sin la batería, comprueba la continuidad: +5V de servos ↔ GND, 5V lógica ↔ GND, 3V3 ↔ GND; no debe haber un cortocircuito en ninguna parte.
2. Aplica el BEC (a través del A4), con el DevKit aún sin insertar. En 5VS debe haber 5,0–5,2 V y en 5VL, 0,3–0,5 V menos (la caída en el diodo).
3. Inserta el DevKit y conecta solo el USB. En 5VS hay 0 V: el diodo no deja pasar el USB a los servos.
4. Todo junto. El `s` de la consola mostrará si responden los sensores y cuántos errores hay en el I2C.

---

## 10. Qué ha cambiado respecto al plan anterior

- **Los pines se han reorganizado para una sola capa** (ya en `Config.h`):
  - I2C de los sensores 8/9 → **41/42**;
  - GPS 15/16 → **39/40**;
  - AUX1/AUX2 41/42 → **15/16**;
  - VBAT 3 → **8**, sensor de corriente 10 → **3**;
  - telemetría 39/40 → **9/10**;
  - una salida nueva, **LIGHT**, en el GPIO21.

  En el banco, cambia dos cables: SDA 8 → 41, SCL 9 → 42. Los demás pines son de reserva y en el banco no están conectados.
- **Un solo BEC a través del ESC**: el conector PWR aparte ha desaparecido.
- **Los sensores de la placa van por I2C** (como en el banco). Los zócalos SPI se han eliminado y los GPIO11–14 están libres. El ICM-42688 también habla I2C, pero el firmware necesitará para él una variante I2C en `SensorSelection.h`.
- **GPS-MAG ha desaparecido**: la brújula del GPS se enchufa en C2.
- **VBAT y el sensor de corriente se han unido** en un solo XH-5 (B1).
