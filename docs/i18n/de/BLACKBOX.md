# Blackbox

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../BLACKBOX.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

Die Firmware schreibt jeden Flug selbst in den eingebauten Flash des Boards: Sensoren, Knüppel, Ausgänge zu den Servos, Entscheidungen des Autopiloten, Ereignisse. Nach dem Flug wird die Aufzeichnung per USB heruntergeladen und in CSV-Tabellen ausgewertet.

Sie funktioniert auf zwei Boards:

| Board | Wohin sie schreibt | Wie viel hineinpasst (bei ~20 KB/s) |
|---|---|---|
| **ESP32-S3 N16R8** | eine Partition von 13,9 MB im eingebauten Flash | etwa **11 Minuten** |
| **STM32H743** (DevEBox – Hauptboard; WeAct) | eine Datei auf einer SD-Karte, [weiter unten](#sd-karte-stm32h743) | 64 MB – etwa **55 Minuten**, die Größe legt die Datei fest |

Auf den übrigen Boards (ESP32-C3, gewöhnlicher ESP32) gibt es keinen Speicher: Die Blackbox ist abgeschaltet und stört den Flug nicht.

---

## Wann sie aufzeichnet

| | Bedingung |
|---|---|
| **Start** | Scharfgeschaltet **und** Gas gegeben (Knüppel oder ESC über `THROTTLE_LOW_US`). Aufgezeichnet werden auch die **10 s davor** – der Moment des ARM und das Stehen vor dem Start |
| | Neustart wegen eines Fehlers (Panic, Watchdog, Spannungseinbruch) – Aufzeichnung ab dem ersten Zyklus und mindestens 60 s lang: Ist es in der Luft passiert, sieht man, was danach geschah |
| | Von Hand aus der Konsole (`k` → `r`) – für den Prüfstand |
| **Stopp** | **10 s nach dem DISARM** |
| | Scharfgeschaltet, aber der Motor steht und das Flugzeug ist **30 s lang bewegungslos** – gelandet oder abgestürzt, und das DISARM wurde vergessen |
| | Von Hand (`k` → `r`) |
| **Kein Stopp** | Verbindungsverlust, Failsafe, Motor auf null in der Luft, Gleitflug, Landung ohne DISARM, solange das Flugzeug noch rollt |

„Bewegungslos“ bedeutet all dies zugleich: Drehung unter 5 °/s um alle Achsen, der Beschleunigungssensor zeigt 1g ± 0,1, laut Barometer fast keine Vertikalgeschwindigkeit, laut GPS und Pitotrohr (falls vorhanden) langsamer als 2 m/s. Im Flug kommt es nie vor, dass es 30 Sekunden am Stück so ruhig ist.

## Was aufgezeichnet wird

| Datensatz | Rate | Was darin steht |
|---|---|---|
| `IMU` | in jedem Zyklus, 500 Hz | Gyroskop (°/s), Beschleunigungssensor (g), wie lange die Arbeit des Regelzyklus gedauert hat (µs) |
| `CTRL` | 100 Hz | Rollen/Nicken/Kurs, Sollwerte des Autopiloten, Knüppel des Piloten, endgültige Kommandos, **alle 7 Ausgänge** (µs), Anteile des PID für Rollen und Nicken (P, I, D), Gas des Piloten und des Autopiloten, Klappen, Modus, Flags (ARM, Verbindung, Failsafe, Sensoren aktiv ...), eingeschaltete Funktionen |
| `RC` | 50 Hz | alle 10 Senderkanäle, Zähler der iBUS-Frames (intakte und defekte) |
| `BARO` | bei jedem Messwert (~50 Hz) | Druck, Temperatur, Höhe, Vertikalgeschwindigkeit, Höhenvorgabe |
| `MAG` | bis 50 Hz | das Feld auf drei Achsen, Kurs |
| `GPS` | bei jeder Lösung | Koordinaten, Höhe, Geschwindigkeit, Kurs, Satelliten, Fix, Genauigkeit |
| `AIR` | bis 50 Hz | Pitotrohr: Druckdifferenz, angezeigte und wahre Fluggeschwindigkeit, Dichte |
| `NAV` | 10 Hz | Startpunkt (Entfernung, Peilung), Kurs und Kursvorgabe, Navigationsgeschwindigkeit, Kursquelle, Phasen von Handstart und Segelflug, Auto-Trimm |
| `POWER` | 10 Hz | Akkuspannung und Ausgang des Stromsensors (die Spannungsteiler der Flugsteuerungsplatine, [FC_BOARD.md](FC_BOARD.md), Block B) |
| `SYS` | 1 Hz | Frequenz und schlechtester Zyklus der Regelschleife, freier Speicher, iBUS-Zähler, IMU-Temperatur, Warteschlange der Blackbox, verlorene Datensätze, der längste Schreibvorgang im Flash, freier Platz |
| `EVENT` | bei einem Ereignis | ARM/DISARM, Ablehnung des ARM mit Grund, Moduswechsel, Verbindung verloren/wieder da, Sensor ausgefallen/wieder da, GPS-Fix, Startpunkt gespeichert, Funktionen der Schalter, Geofence, Schutz vor Strömungsabriss, Phasen von Handstart und Segelflug |

Zu Beginn jedes Flugs stehen die Parameter: die Firmware (Build-Datum), der Grund für den Start und für den letzten Neustart, welche Sensoren vorhanden sind und ob sie die Prüfung vor dem Flug bestanden haben, die PID-Koeffizienten (einschließlich der Änderungen aus dem Dashboard), die Trimmungen, wichtige `Config`-Werte und die Schalterbelegungen.

---

## So wird sie benutzt

### Vor dem Flug

Man muss nichts tun. Beim Einschalten zeigt der serielle Monitor den Zustand (die Konsole gibt Russisch aus; die Zeile unten bedeutet „wartet auf ARM und Gas | voraus gelöscht: 12,9 MB (≈11 min) von 13,9 MB | Flüge: 1“):

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

„Voraus gelöscht“ ist das, was in den nächsten Flug passt. Nach dem Einschalten bereitet die Blackbox einige Sekunden lang (nach einem langen Flug bis zu einer Minute) Platz vor: Sie löscht alte Aufzeichnungen. In dieser Zeit friert die Flugschleife am Boden manchmal für ~0,15 s ein – die Ruder können verspätet zucken, das ist normal. **In der Luft wird der Flash niemals gelöscht.**

### Nach dem Flug – Herunterladen

1. Schließen Sie USB an die Buchse **COM** an. Schließen Sie den seriellen Monitor (er hält den Port belegt).
2. Starten Sie im Projektordner:

   ```bash
   python tools/blackbox.py download          # der letzte Flug
   python tools/blackbox.py download --all    # alle
   python tools/blackbox.py list              # was auf dem Board liegt
   ```

   Dafür ist `pyserial` nötig: `pip install pyserial`. Oder das Python von PlatformIO, das es schon mitbringt: `%USERPROFILE%\.platformio\penv\Scripts\python tools\blackbox.py download`.

3. Der Flug wird in den Ordner `blackbox/` heruntergeladen (~200 KB/s: 10 Minuten Flug dauern etwa eine Minute) und gleich daneben in einen Ordner mit demselben Namen ausgewertet.

Solange der Download läuft, steht die Flugschleife still; deshalb funktioniert er nur ohne ARM.

### Was im Flugordner liegt

| Datei | Was es ist |
|---|---|
| `summary.txt` | Die Zusammenfassung: Dauer, Raten, Bereiche der Winkel, Höhen, Geschwindigkeiten, Spannungen, der schlechteste Regelzyklus, verlorene Datensätze, alle Ereignisse |
| `events.txt` | Die Parameter des Flugs und alle Ereignisse in zeitlicher Reihenfolge |
| `IMU.csv`, `CTRL.csv`, `RC.csv`, ... | Je eine Tabelle pro Datensatztyp |

Die Zeit in allen Tabellen ist `time_s`, Sekunden ab Beginn der Aufzeichnung (ARM und Gas); die Vorabaufzeichnung hat ein Minus. Die Werte liegen schon in Einheiten vor: Grad, g, Meter, m/s, Mikrosekunden Impulsbreite. In `CTRL.csv` ist der Modus mit seinem Namen ergänzt (`mode_name`), die Funktionen als Liste (`features_on`), und die Flags sind in 0/1-Spalten aufgeteilt (`armed`, `rx_lost`, `fs_glide`, `imu_ok` ...).

Die CSV-Dateien lassen sich in Excel/LibreOffice öffnen, aber für Diagramme über der Zeit ist [PlotJuggler](https://github.com/facontidavide/PlotJuggler) bequemer: File → Load Data → CSV, Zeitspalte `time_s`.

Eine `.bbl`-Datei ist ein Rohabbild des Flashs und lässt sich erneut auswerten: `python tools/blackbox.py decode blackbox/flight_001_....bbl`.

### Konsole: `k`

Im seriellen Monitor öffnet die Taste `k` das Menü der Blackbox: Status, Liste der Flüge, `r` – Aufzeichnung von Hand starten/stoppen (zum Prüfen auf dem Prüfstand), `e` – alle Flüge löschen (mit Bestätigung durch `y`, ~40 s).

---

## Platz im Flash

- Die Flüge werden im Ring geschrieben. Wird der Platz knapp, löscht die Blackbox am Boden **die ältesten Flüge komplett**, bis 10 MB frei vor ihr liegen (`BLACKBOX_MIN_FREE_BYTES`, ~9 Minuten).
- **Der zuletzt aufgezeichnete Flug wird nie gelöscht** – nur von der nächsten Aufzeichnung überschrieben, wenn deren Platz nicht gereicht hat.
- Ist der gelöschte Platz in der Luft aufgebraucht, läuft die Aufzeichnung in eine Warteschlange im PSRAM weiter (4 MB, ~3 Minuten der letzten Daten); nach der Landung und dem DISARM schafft die Blackbox Platz und schreibt sie nach. Ein Flug, der länger ist als die ganze Partition (~11 min), passt nicht vollständig hinein: Der Anfang bleibt erhalten, das Ende geht verloren.
- Deshalb **laden Sie den Flug nach jedem Ausflug herunter** – besonders den ersten.

## Zuverlässigkeit

- Ein Stromausfall zu jedem beliebigen Zeitpunkt (Absturz, der Akku hat sich gelöst): Alles bleibt erhalten, außer den letzten ~15 ms. Unvollständig geschriebene Datensätze werden per CRC verworfen – in `summary.txt` ist das die Zeile „Недописанных записей (CRC)“ (die Zusammenfassung ist auf Russisch; es bedeutet „Unvollständige Datensätze (CRC)“).
- Flugnummer, Kopf des Rings und Liste der Flüge werden aus den Sektoren selbst wiederhergestellt: Es gibt keine separate „Karte“, die beschädigt werden könnte.
- Der Download prüft den CRC-32 jedes Sektors und des gesamten Flugs.

## Einfluss auf den Flug

- Die Flugschleife legt nur einen Schnappschuss in eine Warteschlange im PSRAM – einige Mikrosekunden. In den Flash schreibt eine eigene Task auf Kern 0, jeweils eine Seite (256 Byte), **direkt nach einem Regelzyklus**: Ein Flash-Schreibvorgang hält beide ESP32-Kerne für 0,6–0,9 ms an und fällt in die Pause zwischen den Zyklen.
- Messung auf dem Prüfstand (ein DevKit ohne Sensoren, zwei Läufe mit je 30–40 s Aufzeichnung): Der Abstand zwischen den Zyklen beträgt 2,00 ms, 99,2–99,7 % der Abstände liegen innerhalb von 1,9–2,1 ms, der längste beträgt 2,5 ms, und kein einziger Zyklus wurde ausgelassen; die Arbeit des Zyklus ist dieselbe wie ohne Aufzeichnung. Merklich zuckt der Zyklus nur am Boden ohne ARM, solange die Blackbox Platz abgleicht und löscht (Lesen eines 64-KB-Blocks – eine Pause von ~3 ms, Löschen – ~0,15 s).
- Mit Sensoren dauert ein Zyklus ~0,7 ms, und das Schreiben einer Seite passt trotzdem in die verbleibenden 1,3 ms. Kontrolle nach dem ersten Flug: in `summary.txt` die Zeilen „Такт IMU“ und „Цикл: худший такт“ (auf Russisch: „IMU-Zyklus“ und „Schleife: schlechtester Zyklus“).

---

## SD-Karte (STM32H743)

Beim STM32H743 schreibt die Blackbox auf eine SD-Karte (µSD-Steckplatz an SDMMC1, 4 Bit, 24 MHz). Die Karte bleibt eine gewöhnliche **FAT32**-Karte: In ihrem Stammverzeichnis liegt eine vorab angelegte Datei `BLACKBOX.BIN`, in die die Firmware rohe Blöcke schreibt, ohne jemals die FAT-Tabelle oder das Verzeichnis anzufassen. Deshalb kann bei einem Stromausfall im Flug nichts beschädigt werden, und die Datei lässt sich einfach auf einen PC kopieren.

### Die Karte vorbereiten (einmalig)

1. Formatieren Sie die Karte als **FAT32** (nicht exFAT; Windows bietet FAT32 für Karten bis 32 GB an).
2. Mit der Karte in einem Kartenleser am PC:

   ```bash
   python tools/blackbox.py sd-prepare E:              # 64 MB, E: ist das Laufwerk der Karte
   python tools/blackbox.py sd-prepare E: --size 256   # oder mehr
   ```

   Die Datei wird in einem Stück auf einer leeren Karte angelegt und mit `0xFF` gefüllt; der erste Sektor ist eine Dienstmarke „der Ring ist leer“. Liegt die Datei nicht zusammenhängend (die Karte ist nicht leer und stark fragmentiert) oder fehlt sie, nennt die Konsole beim Einschalten den Grund, und die Blackbox ist abgeschaltet.
3. Stecken Sie die Karte ins Board. Beim Einschalten (die Konsole gibt Russisch aus: „SD-Karte: 15204 MB, SDMMC 24 MHz, 4 Bit; Datei BLACKBOX.BIN: ok“, dann die Statuszeile, dann „bereit in 300 ms“):

   ```
   SD-карта: 15204 МБ, SDMMC 24 МГц, 4 бита; файл BLACKBOX.BIN: ок
   BlackBox: ждёт ARM и газ | стёрто впереди 0.7 МБ из 64.0 МБ | полётов 0
   BlackBox: готов за 300 мс
   ```

   „Voraus gelöscht“ wächst im Hintergrund: Das Board gleicht den Platz mit ~2,5 MB/s ab.

### Den Flug abholen

- **Über das Board per USB** – wie beim ESP32: `python tools/blackbox.py download` (die Konsole des STM32 ist USB CDC, die Downloadgeschwindigkeit beträgt ~400 KB/s, 1 MB dauert weniger als 3 s). `list`, `--all`, `--flight N` funktionieren genauso.
- **Durch Herausnehmen der Karte**: Die Datei `BLACKBOX.BIN` von der Karte wird direkt in CSV ausgewertet –

  ```bash
  python tools/blackbox.py ring E:/BLACKBOX.BIN              # alle Flüge -> blackbox/
  python tools/blackbox.py ring E:/BLACKBOX.BIN --list       # nur auflisten
  ```

  Die Datei ist ein Ring aus Sektoren: Das Werkzeug setzt die Flüge anhand der Sektornummern selbst zusammen, auch solche, die über das Dateiende hinausgelaufen sind.

### Was auf dem Board gemessen wurde

DevEBox H743 + 16-GB-Karte (der Test `test_blackbox_sd`, [TESTING.md](TESTING.md#tests-auf-dem-stm32-board)):

| | |
|---|---|
| Erkennen der Karte | 12–18 ms, 4 Bit, 24 MHz |
| Schreiben einer Seite mit 256 B | im Mittel 2,3–3,7 ms, **schlechtester Fall 60–190 ms**, ~75–110 KB/s dauerhaft (nötig sind ~20 KB/s) |
| Lesen | ein 4-KB-Sektor – 4,2 MB/s; ein zufälliger Block – 0,6 ms |
| Löschen | 64 KB – 13 ms; der gesamte 64-MB-Bereich – 20–28 s |
| Einschalten | mit leerem Ring – 0 ms (per Marke); mit Flügen – 0,3 s (stichprobenartiger Abgleich mit ~530 Lesevorgängen); ein vollständiger Abgleich der 64 MB würde ~20 s dauern |
| 20 s Aufzeichnung in Echtzeit (IMU mit 500 Hz) | kein einziger verlorener Datensatz, 0 Fehler |
| Die Flug-Task bei der Aufzeichnung | Abweichung der Periode von 2 ms – **1 µs** (eine Simulations-Task höchster Priorität neben der Aufzeichnung) |

Der schlechteste Schreibvorgang einer Seite ist die interne „Aufräumarbeit“ der Karte; die Warteschlange im RAM (384 KB ≈ 19 s Datenstrom) übersteht solche Pausen. Billige Karten unterscheiden sich hierin am stärksten: Vor den Flügen sollte man die Karte mit dem Test `test_blackbox_sd` prüfen (der schlechteste Schreibvorgang muss unter 250 ms liegen – der Grenze der SD-Spezifikation).

### Worin sie sich vom Flash des ESP32 unterscheidet

- **Die Schreib-Task** (`bbox`, Priorität 2) wird von der Flug-Task (5) mitten in einem Kartenzugriff verdrängt – und nicht „in der Pause des Zyklus“, wie beim ESP32 mit seinem Anhalten der Kerne. Die Übertragung läuft mit der Hardware-Flusskontrolle des SDMMC: Ohne sie lief der FIFO bei der Verdrängung über (auf dem Board waren das `HAL_SD_ERROR_RX_OVERRUN` und eine für Sekunden eingefrorene Konsole und Aufzeichnung).
- **Die Warteschlange liegt im RAM**, 384 KB (`BLACKBOX_RING_STM32_BYTES`), nicht in 4 MB PSRAM.
- **Der Abgleich beim Einschalten erfolgt stichprobenartig**: Die echten Sektoren des Rings bilden einen zusammenhängenden Bogen, es werden ~500 Köpfe gelesen, und die Grenzen des Bogens und der Flüge werden durch Halbieren verfeinert. Das Ergebnis ist dasselbe wie bei einem vollständigen Abgleich; geht das Bild nicht auf, folgt der vollständige.
- **Die Marke „Ring leer“** im ersten Sektor der Datei: damit der leere Bereich nicht bei jedem Einschalten sekundenlang abgeglichen wird. Sie wird gesetzt, wenn alles gelöscht wird und wenn ein vollständiger Abgleich nichts gefunden hat; sie wird vor dem ersten Schreiben entfernt.
- **Kartenfehler** (herausgezogen, Busfehler) landen einmal pro Sekunde als Ereignis „носитель: ошибок записи …“ im Journal (auf Russisch: „Datenträger: Schreibfehler …“); eine verlorene Seite hinterlässt ein `0xFF`-Loch, und die Auswertung des Sektors endet dort (genau wie in `tools/blackbox.py`), die übrigen Sektoren sind unversehrt.

## Einstellungen (`include/config/Config.h`, Abschnitt „Blackbox“)

| Konstante | Standard | Bedeutung |
|---|---|---|
| `BLACKBOX_RING_BYTES` | 4 MB | Die Warteschlange im PSRAM (ohne PSRAM – `BLACKBOX_RING_NO_PSRAM_BYTES`, 32 KB) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32: die Warteschlange im RAM |
| `BLACKBOX_SD_FILE`, `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN`, 256 MB | STM32: die Datei auf der Karte und die Obergrenze des genutzten Teils |
| `BLACKBOX_PREROLL_MS` | 10 000 | Wie viel vor dem Start aufgezeichnet wird |
| `BLACKBOX_POSTROLL_MS` | 10 000 | Wie viel nach dem DISARM aufgezeichnet wird |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Bewegungslos bei ARM – Stopp |
| `BLACKBOX_LANDED_GYRO_DPS`, `_ACCEL_G`, `_CLIMB_MS`, `_SPEED_MS` | 5, 0,1, 0,5, 2 | Was als „bewegungslos“ gilt |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Aufzeichnung nach einem Neustart wegen eines Fehlers – nicht kürzer als dies |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | Wie viel für den nächsten Flug gelöscht gehalten wird |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | Die Pause zwischen den Löschvorgängen am Boden |
| `BLACKBOX_IMU_DIVIDER` | 1 | Die IMU in jedem N-ten Zyklus: 2 bedeutet 250 Hz und eine um ~25 % längere Aufzeichnung |
| `BLACKBOX_VBAT_DIVIDER`, `_CURRENT_DIVIDER` | 6,6, 1,667 | Die Teiler für Akku und Stromsensor auf dem Board |

Die Partitionstabelle ist `partitions_blackbox.csv`: die Anwendung 2 MB (die Firmware ist jetzt ~0,9 MB groß), die Blackbox 13,9 MB, der Coredump 64 KB. Die NVS-Partition ist an ihrem alten Platz geblieben – die Kalibrierungen von IMU und Kompass sowie die Trimmungen bleiben nach dem Wechsel auf diese Tabelle erhalten. Einen zweiten Slot für Updates über die Luft (OTA) gibt es nicht.

---

## Für Entwickler

Der Code liegt in `include/telemetry/BlackBox*.h`:

| Datei | Was |
|---|---|
| `BlackBoxFormat.h` | Das Format: Sektorkopf, Typen und Strukturen der Datensätze, Schemata der Felder, CRC-8/CRC-32 |
| `BlackBoxStorage.h` | Ein Ring aus Sektoren auf `IFlashRegion`: Suche des Kopfes beim Einschalten, Liste der Flüge, seitenweises Schreiben, schrittweises Löschen alter Flüge |
| `BlackBoxRing.h` | Eine Warteschlange von Datensätzen zwischen den Kernen (Spinlock), die den ältesten verwirft |
| `BlackBox.h` | Schnappschüsse in der Schleife, Start/Stopp, Ereignisse, die Schreib-Task, Download über UART |
| `hal/esp32/Esp32FlashPartition.h` | `IFlashRegion` auf Basis von `esp_partition` |
| `hal/SdFileRegion.h`, `storage/Fat32File.h` | `IFlashRegion` auf Basis einer Datei auf einer FAT32-Karte: Suche der Datei (FAT nur lesend), unvollständige Blöcke, Löschen mit `0xFF`, die Marke „Ring leer“ |
| `hal/stm32/Stm32SdCard.h`, `src/stm32/sd_msp.cpp` | `IBlockDevice`: SDMMC1 auf `HAL_SD` (Polling, 4 Bit, Hardware-Flusskontrolle) und die Pins |
| `hal/ResetCause.h` | Die Neustartursache beim ESP32 und STM32 (`RCC->RSR`) |

### Format im Flash

Ein 4-KB-Sektor = ein 16-Byte-Kopf (`magic "OPBB"`, ein durchlaufendes `seq`, `millis()` beim Öffnen, die Flugnummer, die Formatversion, ein Prüfbyte) + die Datensätze hintereinander. Ein Datensatz überschreitet die Sektorgrenze nicht; das Ende des Sektors ist `0xFF`.

Ein Datensatz: `[Typ u8][Länge u8][Daten][CRC-8]`, die Daten beginnen mit `t_us` (`micros()`). Die ersten Datensätze eines Flugs sind `SCHEMA`: der Text `"16 IMU t_us:I gx:h/10 ..."` – Feldname, das Zeichen des Python-`struct`, der Teiler. Der Decoder entnimmt die Felder dem Log; ein neues Feld in einem Datensatz bedeutet daher, die Struktur und die Schema-Zeichenkette in `BlackBoxFormat.h` zu ändern (ihre Größen gleicht `static_assert` ab), am Decoder muss nichts geändert werden.

### Download-Protokoll

Die Befehle sind eine Zeile nach dem STX-Byte (`0x02`), die die Konsole an die Blackbox weiterreicht:

```
PC:  \x02bb list\n
FC:  BB:STATE state=idle free_kb=... total_kb=... flights=... rate_bps=...
     BB:FLIGHT n=3 sectors=234 kb=936 seconds=41 start=1
     BB:END
PC:  \x02bb get 3 2000000\n
FC:  BB:SEND n=3 sectors=234 baud=2000000   (bei 115200), wechselt dann auf 2 MBaud
PC:  wechselt auf 2 MBaud, sendet 'G'
FC:  234 Frames: A5 5A, u16 Nummer, 4096 Byte des Sektors, u32 CRC-32
     kehrt auf 115200 zurück, BB:DONE n=3 crc=<CRC-32 aller Sektoren>
```

### Tests

`pio test -e native -f native/test_blackbox_scan` – ein stichprobenartiger Abgleich des Rings gegen einen vollständigen auf zufälligen Verläufen (300 Ringe × 5 Prüfschritte) sowie die Kosten auf einem SD-Bereich. `pio test -e native-stm32 -f native_stm32/test_blackbox_sd` und `test_app_stm32_*` – FAT32, der Bereich, der Kartentreiber, die Blackbox auf der Karte, die gesamte STM32-Firmware. Auf dem Board – `pio test -e stm32h743-devebox -f test_blackbox_sd` (eine echte Karte).

`pio test -e native -f native/test_blackbox` – das Format, der Ring auf einem NOR-Fake der Partition (Löschen sektorweise, ein Schreibvorgang senkt nur Bits – das Anheben eines Bits gilt als Fehler), Neustarts, Stromausfälle, Aufzeichnung eines Flugs auf dem echten `FlightController`/`Autopilot`, Start/Stopp, Ereignisse, Flash-Überlauf in der Luft, Download. Ein Flugabbild zum Prüfen des Decoders: `OPENPLANE_BLACKBOX_DUMP=/tmp/f.bbl pio test -e native -f native/test_blackbox`.
