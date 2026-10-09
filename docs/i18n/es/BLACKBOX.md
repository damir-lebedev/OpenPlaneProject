# Caja negra

> 🌐 Esta página es una traducción del [original en ruso](../../BLACKBOX.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

El firmware graba por sí solo cada vuelo en la flash integrada de la placa: sensores, sticks, salidas a los servos, decisiones del piloto automático, eventos. Después del vuelo, la grabación se descarga por USB y se descodifica en tablas CSV.

Funciona en dos placas:

| Placa | Dónde graba | Cuánto cabe (a ~20 KB/s) |
|---|---|---|
| **ESP32-S3 N16R8** | una partición de 13.9 MB de la flash integrada | unos **11 minutos** |
| **STM32H743** (DevEBox, la principal; WeAct) | un archivo en una tarjeta SD, [más abajo](#tarjeta-sd-stm32h743) | 64 MB: unos **55 minutos**, el tamaño lo fija el archivo |

En las demás placas (ESP32-C3, ESP32 normal) no hay soporte de almacenamiento: la caja negra está desactivada y no interfiere con el vuelo.

---

## Cuándo graba

| | Condición |
|---|---|
| **Inicio** | Armado **y** con el gas subido (el stick o el ESC por encima de `THROTTLE_LOW_US`). También se graban los **10 s anteriores**: el momento del ARM y la espera antes del despegue |
| | Un reinicio provocado por un fallo (pánico, watchdog, caída de la alimentación): se graba desde el primer ciclo y durante al menos 60 s; si ocurrió en el aire, se ve lo que pasó después |
| | A mano desde la consola (`k` → `r`): para el banco |
| **Parada** | **10 s después del DISARM** |
| | Armado, pero con el motor parado y el avión **inmóvil durante 30 s**: ha aterrizado o se ha estrellado y se ha olvidado el DISARM |
| | A mano (`k` → `r`) |
| **No es parada** | Pérdida de señal, failsafe, motor a cero en el aire, planeo, aterrizaje sin DISARM mientras el avión sigue rodando |

«Inmóvil» significa todo esto a la vez: giro inferior a 5 °/s en todos los ejes, el acelerómetro marca 1g ± 0.1, casi no hay velocidad vertical según el barómetro y, según el GPS y el tubo de Pitot (si lo hay), se va a menos de 2 m/s. En vuelo, una calma así durante 30 segundos seguidos no se da nunca.

## Qué se graba

| Registro | Frecuencia | Qué contiene |
|---|---|---|
| `IMU` | en cada ciclo, 500 Hz | giroscopio (°/s), acelerómetro (g), cuánto duró el trabajo del ciclo de control (µs) |
| `CTRL` | 100 Hz | alabeo/cabeceo/rumbo, objetivos del piloto automático, sticks del piloto, órdenes finales, **las 7 salidas** (µs), componentes del PID de alabeo y de cabeceo (P, I, D), gas del piloto y del piloto automático, flaps, modo, indicadores (ARM, enlace, failsafe, sensores vivos...), funciones activadas |
| `RC` | 50 Hz | los 10 canales de la emisora, contadores de tramas iBUS (completas y dañadas) |
| `BARO` | en cada muestra (~50 Hz) | presión, temperatura, altitud, velocidad vertical, altitud objetivo |
| `MAG` | hasta 50 Hz | el campo en tres ejes, rumbo |
| `GPS` | en cada solución | coordenadas, altitud, velocidad, rumbo, satélites, fijación, precisión |
| `AIR` | hasta 50 Hz | tubo de Pitot: diferencia de presión, velocidad indicada y verdadera, densidad |
| `NAV` | 10 Hz | punto de origen (distancia, marcación), rumbo y rumbo objetivo, velocidad de navegación, fuente del rumbo, fases del lanzamiento a mano y del vuelo a vela, autotrim |
| `POWER` | 10 Hz | tensión de la batería y salida del sensor de corriente (los divisores de la placa del controlador de vuelo, [FC_BOARD.md](FC_BOARD.md), bloque B) |
| `SYS` | 1 Hz | frecuencia y peor ciclo del lazo de control, memoria libre, contadores de iBUS, temperatura del IMU, cola de la caja negra, registros perdidos, la escritura en flash más larga, espacio libre |
| `EVENT` | por evento | ARM/DISARM, rechazo de ARM con el motivo, cambio de modo, enlace perdido/recuperado, sensor averiado/recuperado, fijación del GPS, punto de origen registrado, funciones de los interruptores, geovalla, protección contra la pérdida de sustentación, fases del lanzamiento a mano y del vuelo a vela |

Al comienzo de cada vuelo van los parámetros: el firmware (fecha de compilación), el motivo del arranque y del último reinicio, qué sensores hay y si superaron la comprobación previa al vuelo, los coeficientes del PID (con los cambios hechos desde el panel), los trims, valores importantes de `Config` y las asignaciones de los interruptores.

---

## Cómo usarla

### Antes del vuelo

No hay que hacer nada. Al encender, el monitor serie muestra el estado (la consola imprime en ruso; la línea de abajo dice «espera ARM y acelerador | borrado por delante 12.9 MB (≈11 min) de 13.9 MB | vuelos 1»):

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

«Borrado por delante» es cuánto cabrá en el siguiente vuelo. Tras el encendido, la caja negra tarda unos segundos (después de un vuelo largo, hasta un minuto) en preparar espacio: borra las grabaciones antiguas. Durante ese tiempo el ciclo de vuelo, en el suelo, a veces se queda congelado unos ~0.15 s: las superficies pueden dar un tirón con retraso, y es normal. **En el aire la flash no se borra nunca.**

### Después del vuelo: descarga

1. Conecta el USB al conector **COM**. Cierra el monitor serie (retiene el puerto).
2. Ejecuta en la carpeta del proyecto:

   ```bash
   python tools/blackbox.py download          # el último vuelo
   python tools/blackbox.py download --all    # todos
   python tools/blackbox.py list              # qué hay en la placa
   ```

   Hace falta `pyserial`: `pip install pyserial`. O el Python de PlatformIO, que ya lo trae: `%USERPROFILE%\.platformio\penv\Scripts\python tools\blackbox.py download`.

3. El vuelo se descarga en la carpeta `blackbox/` (~200 KB/s: 10 minutos de vuelo tardan alrededor de un minuto) y se descodifica enseguida al lado, en una carpeta con el mismo nombre.

Mientras dura la descarga, el ciclo de vuelo está parado, así que solo funciona sin ARM.

### Qué hay dentro de la carpeta del vuelo

| Archivo | Qué es |
|---|---|
| `summary.txt` | El resumen: duración, frecuencias, rangos de ángulos, altitudes, velocidades, tensiones, el peor ciclo de control, registros perdidos, todos los eventos |
| `events.txt` | Los parámetros del vuelo y todos los eventos por orden de tiempo |
| `IMU.csv`, `CTRL.csv`, `RC.csv`, ... | Una tabla por cada tipo de registro |

El tiempo en todas las tablas es `time_s`, segundos desde el inicio de la grabación (ARM y gas); la pregrabación va con signo menos. Los valores ya están en unidades: grados, g, metros, m/s, microsegundos de pulso. En `CTRL.csv` el modo se añade con su nombre (`mode_name`), las funciones como lista (`features_on`) y los indicadores se reparten en columnas 0/1 (`armed`, `rx_lost`, `fs_glide`, `imu_ok`...).

Los CSV se abren en Excel/LibreOffice, pero para las gráficas en función del tiempo es más cómodo [PlotJuggler](https://github.com/facontidavide/PlotJuggler): File → Load Data → CSV, la columna de tiempo `time_s`.

El archivo `.bbl` es una imagen en bruto de la flash y se puede descodificar de nuevo: `python tools/blackbox.py decode blackbox/flight_001_....bbl`.

### Consola: `k`

En el monitor serie, la tecla `k` abre el menú de la caja negra: estado, lista de vuelos, `r`: iniciar/detener la grabación a mano (para comprobar en el banco), `e`: borrar todos los vuelos (con confirmación `y`, ~40 s).

---

## Espacio en la flash

- Los vuelos se escriben en anillo. Cuando queda poco espacio, la caja negra, en el suelo, borra **enteros los vuelos más antiguos** hasta que haya 10 MB libres por delante (`BLACKBOX_MIN_FREE_BYTES`, ~9 minutos).
- **El último vuelo grabado no se borra nunca**: solo lo sobrescribe la siguiente grabación, si no le ha bastado el espacio.
- Si el espacio borrado se agota en el aire, la grabación continúa en una cola en la PSRAM (4 MB, ~3 minutos de los últimos datos); tras el aterrizaje y el DISARM, la caja negra libera espacio y la escribe. Un vuelo más largo que toda la partición (~11 min) no cabe entero: se conserva el principio y se pierde el final.
- Por eso **conviene descargar el vuelo después de cada salida**, sobre todo la primera.

## Fiabilidad

- Un corte de alimentación en cualquier momento (se estrelló, se soltó la batería): se conserva todo salvo los últimos ~15 ms. Los registros a medio escribir se descartan por CRC; en `summary.txt` es la línea «Недописанных записей (CRC)» (el resumen está en ruso; significa «Registros incompletos (CRC)»).
- El número de vuelo, la cabeza del anillo y la lista de vuelos se restauran a partir de los propios sectores: no hay ningún «mapa» aparte que se pueda corromper.
- La descarga comprueba el CRC-32 de cada sector y de todo el vuelo.

## Efecto sobre el vuelo

- El ciclo de vuelo solo deja una instantánea en una cola en la PSRAM: unos pocos microsegundos. Una tarea aparte en el núcleo 0 escribe en la flash, de una página (256 bytes) en una página, **justo después de un ciclo de control**: una escritura en flash detiene los dos núcleos del ESP32 durante 0.6–0.9 ms, y cae en la pausa entre ciclos.
- Medición en el banco (una DevKit sin sensores, dos pasadas de 30–40 s de grabación): el intervalo entre ciclos es de 2.00 ms, el 99.2–99.7 % de los intervalos están dentro de 1.9–2.1 ms, el más largo es de 2.5 ms y no se saltó ni un solo ciclo; el trabajo del ciclo es el mismo que sin grabar. El ciclo solo tiembla de forma apreciable en el suelo sin ARM, mientras la caja negra comprueba y borra espacio (leer un bloque de 64 KB: una pausa de ~3 ms; borrar: ~0.15 s).
- Con sensores, un ciclo ocupa ~0.7 ms y la escritura de una página sigue cabiendo en el 1.3 ms restante. Comprobación tras el primer vuelo: en `summary.txt`, las líneas «Такт IMU» y «Цикл: худший такт» (en ruso: «Ciclo del IMU» y «Lazo: peor ciclo»).

---

## Tarjeta SD (STM32H743)

En la STM32H743, la caja negra graba en una tarjeta SD (ranura µSD en SDMMC1, 4 bits, 24 MHz). La tarjeta sigue siendo una **FAT32** normal: en su raíz hay un archivo creado de antemano, `BLACKBOX.BIN`, dentro del cual el firmware escribe bloques en bruto, sin tocar nunca la tabla FAT ni el directorio. Por eso no hay nada que se pueda corromper si se pierde la alimentación en vuelo, y el archivo se puede copiar sin más a un PC.

### Preparación de la tarjeta (una sola vez)

1. Formatea la tarjeta en **FAT32** (no exFAT; Windows ofrece FAT32 para tarjetas de hasta 32 GB).
2. Con la tarjeta en un lector, en un PC:

   ```bash
   python tools/blackbox.py sd-prepare E:              # 64 MB, E: es la unidad de la tarjeta
   python tools/blackbox.py sd-prepare E: --size 256   # o más
   ```

   El archivo se crea de una sola pieza en una tarjeta vacía y se rellena con `0xFF`; el primer sector es una marca de servicio «el anillo está vacío». Si el archivo no es contiguo (la tarjeta no está vacía y está muy fragmentada) o no existe, al encender la consola indicará el motivo y la caja negra quedará desactivada.
3. Inserta la tarjeta en la placa. Al encender (la consola imprime en ruso: «Tarjeta SD: 15204 MB, SDMMC 24 MHz, 4 bits; archivo BLACKBOX.BIN: ok», luego la línea de estado y después «listo en 300 ms»):

   ```
   SD-карта: 15204 МБ, SDMMC 24 МГц, 4 бита; файл BLACKBOX.BIN: ок
   BlackBox: ждёт ARM и газ | стёрто впереди 0.7 МБ из 64.0 МБ | полётов 0
   BlackBox: готов за 300 мс
   ```

   «Borrado por delante» crece en segundo plano: la placa comprueba el espacio a ~2.5 MB/s.

### Recoger el vuelo

- **A través de la placa por USB**, como en la ESP32: `python tools/blackbox.py download` (la consola de la STM32 es USB CDC, la velocidad de descarga es de ~400 KB/s, 1 MB tarda menos de 3 s). `list`, `--all` y `--flight N` funcionan igual.
- **Sacando la tarjeta**: el archivo `BLACKBOX.BIN` de la tarjeta se descodifica directamente a CSV:

  ```bash
  python tools/blackbox.py ring E:/BLACKBOX.BIN              # todos los vuelos -> blackbox/
  python tools/blackbox.py ring E:/BLACKBOX.BIN --list       # solo enumerarlos
  ```

  El archivo es un anillo de sectores: la utilidad reúne por sí sola los vuelos según los números de sector, incluidos los que pasaron del final del archivo.

### Qué se ha medido en la placa

DevEBox H743 + tarjeta de 16 GB (la prueba `test_blackbox_sd`, [TESTING.md](TESTING.md#pruebas-en-la-placa-stm32)):

| | |
|---|---|
| Reconocimiento de la tarjeta | 12–18 ms, 4 bits, 24 MHz |
| Escritura de una página de 256 B | de media 2.3–3.7 ms, **la peor 60–190 ms**, ~75–110 KB/s sostenidos (hacen falta ~20 KB/s) |
| Lectura | un sector de 4 KB: 4.2 MB/s; un bloque aleatorio: 0.6 ms |
| Borrado | 64 KB: 13 ms; toda la zona de 64 MB: 20–28 s |
| Encendido | con el anillo vacío: 0 ms (por la marca); con vuelos: 0.3 s (comprobación por muestreo de ~530 lecturas); una comprobación completa de 64 MB tardaría ~20 s |
| 20 s de grabación en tiempo real (IMU a 500 Hz) | ni un solo registro perdido, 0 errores |
| La tarea de vuelo durante la grabación | desviación del periodo de 2 ms: **1 µs** (una tarea simuladora de máxima prioridad junto a la grabación) |

La peor escritura de una página es la «limpieza» interna de la tarjeta; la cola en RAM (384 KB ≈ 19 s de flujo) aguanta esas pausas. Las tarjetas baratas son las que más se diferencian en esto: antes de volar conviene comprobar la tarjeta con la prueba `test_blackbox_sd` (la peor escritura debe ser inferior a 250 ms, el límite de la especificación SD).

### En qué se diferencia de la flash de la ESP32

- **La tarea de escritura** (`bbox`, prioridad 2) es desplazada por la de vuelo (5) en mitad de un acceso a la tarjeta, y no «en la pausa del ciclo», como en la ESP32 con su parada de núcleos. La transferencia se hace con control de flujo por hardware del SDMMC: sin él, la FIFO se desbordaba al ser desplazada (en la placa eran `HAL_SD_ERROR_RX_OVERRUN` y la consola y la grabación congeladas durante segundos).
- **La cola está en la RAM**, de 384 KB (`BLACKBOX_RING_STM32_BYTES`), y no en 4 MB de PSRAM.
- **La comprobación al encender es por muestreo**: los sectores reales del anillo forman un arco continuo, se leen ~500 cabeceras y los límites del arco y de los vuelos se precisan por bisección. El resultado es el mismo que el de la comprobación completa; si el cuadro no cuadra, se hace la completa.
- **La marca «anillo vacío»** en el primer sector del archivo: para no comprobar durante segundos la zona vacía en cada encendido. Se pone al borrarlo todo y cuando una comprobación completa no ha encontrado nada; se quita antes de la primera escritura.
- **Los errores de la tarjeta** (la han sacado, fallo del bus) pasan al registro una vez por segundo como el evento «носитель: ошибок записи …» (en ruso: «soporte: errores de escritura …»); una página perdida deja un hueco `0xFF`, y la descodificación del sector se detiene en él (igual que en `tools/blackbox.py`); los demás sectores están intactos.

## Ajustes (`include/config/Config.h`, sección «Caja negra»)

| Constante | Por defecto | Significado |
|---|---|---|
| `BLACKBOX_RING_BYTES` | 4 MB | La cola en la PSRAM (sin PSRAM: `BLACKBOX_RING_NO_PSRAM_BYTES`, 32 KB) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32: la cola en la RAM |
| `BLACKBOX_SD_FILE`, `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN`, 256 MB | STM32: el archivo en la tarjeta y el tope de la parte utilizada |
| `BLACKBOX_PREROLL_MS` | 10 000 | Cuánto grabar antes del inicio |
| `BLACKBOX_POSTROLL_MS` | 10 000 | Cuánto grabar después del DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Inmóvil y con ARM: parada |
| `BLACKBOX_LANDED_GYRO_DPS`, `_ACCEL_G`, `_CLIMB_MS`, `_SPEED_MS` | 5, 0.1, 0.5, 2 | Qué se considera «inmóvil» |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Grabación tras un reinicio por fallo: no menos de esto |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | Cuánto mantener borrado para el siguiente vuelo |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | La pausa entre borrados en el suelo |
| `BLACKBOX_IMU_DIVIDER` | 1 | El IMU cada N ciclos: 2 equivale a 250 Hz y a una grabación ~25 % más larga |
| `BLACKBOX_VBAT_DIVIDER`, `_CURRENT_DIVIDER` | 6.6, 1.667 | Los divisores de la batería y del sensor de corriente en la placa |

La tabla de particiones es `partitions_blackbox.csv`: la aplicación, 2 MB (el firmware ocupa ahora ~0.9 MB), la caja negra, 13.9 MB, y el coredump, 64 KB. La partición NVS se quedó en su sitio: las calibraciones del IMU y de la brújula y los trims se conservan tras pasar a esta tabla. No hay un segundo slot para las actualizaciones por aire (OTA).

---

## Para el desarrollador

El código está en `include/telemetry/BlackBox*.h`:

| Archivo | Qué |
|---|---|
| `BlackBoxFormat.h` | El formato: cabecera del sector, tipos y estructuras de los registros, esquemas de los campos, CRC-8/CRC-32 |
| `BlackBoxStorage.h` | Un anillo de sectores sobre `IFlashRegion`: búsqueda de la cabeza al encender, lista de vuelos, escritura por páginas, borrado de los vuelos antiguos por pasos |
| `BlackBoxRing.h` | Una cola de registros entre los núcleos (spinlock) que descarta el más antiguo |
| `BlackBox.h` | Instantáneas en el ciclo, inicio/parada, eventos, la tarea de escritura, descarga por UART |
| `hal/esp32/Esp32FlashPartition.h` | `IFlashRegion` sobre `esp_partition` |
| `hal/SdFileRegion.h`, `storage/Fat32File.h` | `IFlashRegion` sobre un archivo de una tarjeta FAT32: búsqueda del archivo (la FAT es de solo lectura), bloques incompletos, borrado con `0xFF`, la marca «anillo vacío» |
| `hal/stm32/Stm32SdCard.h`, `src/stm32/sd_msp.cpp` | `IBlockDevice`: SDMMC1 sobre `HAL_SD` (sondeo, 4 bits, control de flujo por hardware) y los pines |
| `hal/ResetCause.h` | La causa del reinicio en ESP32 y STM32 (`RCC->RSR`) |

### Formato en la flash

Un sector de 4 KB = una cabecera de 16 bytes (`magic "OPBB"`, un `seq` correlativo, `millis()` en el momento de abrirlo, el número de vuelo, la versión del formato, un byte de control) + los registros uno tras otro. Un registro no cruza el límite del sector; la cola del sector es `0xFF`.

Un registro: `[tipo u8][longitud u8][datos][CRC-8]`; los datos empiezan con `t_us` (`micros()`). Los primeros registros de un vuelo son `SCHEMA`: el texto `"16 IMU t_us:I gx:h/10 ..."`: el nombre del campo, el carácter de `struct` de Python y el divisor. El descodificador toma los campos del registro, así que un campo nuevo es una modificación de la estructura y de la cadena del esquema en `BlackBoxFormat.h` (sus tamaños los contrasta `static_assert`); no hace falta tocar el descodificador.

### Protocolo de descarga

Los comandos son una línea después del byte STX (`0x02`), y la consola se la pasa a la caja negra:

```
PC:  \x02bb list\n
FC:  BB:STATE state=idle free_kb=... total_kb=... flights=... rate_bps=...
     BB:FLIGHT n=3 sectors=234 kb=936 seconds=41 start=1
     BB:END
PC:  \x02bb get 3 2000000\n
FC:  BB:SEND n=3 sectors=234 baud=2000000   (a 115200), luego pasa a 2 Mbaudios
PC:  pasa a 2 Mbaudios, envía 'G'
FC:  234 tramas: A5 5A, u16 número, 4096 bytes del sector, u32 CRC-32
     vuelve a 115200, BB:DONE n=3 crc=<CRC-32 de todos los sectores>
```

### Pruebas

`pio test -e native -f native/test_blackbox_scan`: una comprobación del anillo por muestreo frente a una completa, sobre historias aleatorias (300 anillos × 5 pasos de sondeo), y el coste sobre una zona de SD. `pio test -e native-stm32 -f native_stm32/test_blackbox_sd` y `test_app_stm32_*`: FAT32, la zona, el controlador de la tarjeta, la caja negra en la tarjeta, todo el firmware de la STM32. En la placa: `pio test -e stm32h743-devebox -f test_blackbox_sd` (una tarjeta real).

`pio test -e native -f native/test_blackbox`: el formato, el anillo sobre un NOR simulado de la partición (borrado por sectores, una escritura solo baja bits; subir un bit se considera un error), reinicios, cortes de alimentación, grabación de un vuelo con los `FlightController`/`Autopilot` reales, inicio/parada, eventos, desbordamiento de la flash en el aire, descarga. Una imagen de vuelo para comprobar el descodificador: `OPENPLANE_BLACKBOX_DUMP=/tmp/f.bbl pio test -e native -f native/test_blackbox`.
