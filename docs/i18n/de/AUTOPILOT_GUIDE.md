# Referenz zum OpenPlane-Autopiloten

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../AUTOPILOT_GUIDE.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

Was der Autopilot kann, wie man jede Funktion einschaltet und wie man sie **mit einer einzigen Zeile** auf einen beliebigen Schalter oder Drehregler des Senders legt.

> Zuerst ehrlich gesagt: Alle Modi sind durch Unit-Tests und geschlossene Flugsimulationen überprüft (`test/native/test_sim`: Die gesamte Firmware steuert ein Flugzeugmodell). Das Modell ist vereinfacht, und die Koeffizienten in `Config.h` sind Startwerte: **Jeder Modus wird zuerst in über 50 m Höhe erprobt, mit dem Finger am MANUAL-Schalter**. Bisher ist nur der manuelle Modus geflogen (der erste Prototyp mit einem ESP32-C3); die Stabilisierung wurde auf dem Prüfstand geprüft – die Ruder reagieren auf Neigungen in die richtige Richtung. Das Board STM32H743 lässt sich bauen und besteht dieselben Tests; auf echter Hardware wurde das Board DevEBox ohne Sensoren geprüft: die SD-Karte, die Blackbox und die manuelle Steuerung von Servos und Motor vom Sender aus (auf Video aufgenommen); Sensoren wurden daran noch nicht angeschlossen, und die Autopilot-Modi wurden darauf nicht erprobt.

---

## Inhalt

1. [Wie es in einer Minute funktioniert](#wie-es-in-einer-minute-funktioniert)
2. [Standardbelegung des Senders](#standardbelegung-des-senders)
3. [Eine Funktion mit einer Zeile zuweisen](#eine-funktion-mit-einer-zeile-zuweisen)
4. [Modi](#modi)
5. [Funktionen (Schalter)](#funktionen-schalter)
6. [Drehregler](#drehregler)
7. [Verbindungsverlust, Geofence, Startpunkt](#verbindungsverlust-geofence-startpunkt)
8. [Ein Pitotrohr zum Selberbauen](#ein-pitotrohr-zum-selberbauen)
9. [Bodenstation: WLAN-Dashboard und MAVLink](#bodenstation-wlan-dashboard-und-mavlink)
10. [Reihenfolge bei der Einrichtung eines neuen Flugzeugs](#reihenfolge-bei-der-einrichtung-eines-neuen-flugzeugs)
11. [Autopilot-Prüfung vor dem Flug](#autopilot-prüfung-vor-dem-flug)
12. [Was jeder Modus braucht](#was-jeder-modus-braucht)

---

## Wie es in einer Minute funktioniert

```
Knüppel ┐
        ├─► PilotSwitches (config/Controls.h) ─► Modus, Funktionen, Drehregler
Schalter┘                                               │
                                                        ▼
Sensoren (IMU, Barometer, Kompass, GPS, Pitotrohr) ─► Autopilot ─► Befehle für Ruder und Gas
                                                        │
                           FlightController: Klappen, Last, Kamera, Summer, Failsafe
                                                        ▼
                                           Querruder · Höhenruder · Seitenruder · ESC · AUX1 · AUX2
```

- Der **Modus** entscheidet, wer steuert: der Pilot (MANUAL), der Pilot mit einem Helfer (STABILIZE, ALT_HOLD, ACRO), der Autopilot mit Korrekturen des Piloten (CRUISE, LOITER, RTH …).
- **Funktionen** werden über jeden Modus gelegt: Klappen, Bremse, Lastabwurf, Geofence …
- **Drehregler** verändern einen Wert stufenlos: die Stärke der Stabilisierung, die Reisegeschwindigkeit, den Kreisradius …
- In den Modi mit Stabilisierung **gibt der Knüppel einen Winkel vor**, keinen Ruderausschlag: Knüppel losgelassen – das Flugzeug richtet sich von selbst waagerecht aus.
- Der Ausfall eines Sensors „ruckelt“ das Flugzeug nie: keine IMU – die Ruder bleiben beim Piloten; kein Barometer – die Höhe hält der Pilot; kein GPS – keine Navigation, und Modi, die es brauchen, verhalten sich sicher (siehe [die Tabelle](#was-jeder-modus-braucht)).

---

## Standardbelegung des Senders

FS-i6 + FS-iA6B, iBUS, 10 Kanäle (`config/Channels.h`).

| Kanal | Bedienelement am Sender | Standard |
|---|---|---|
| CH1–CH4 | Knüppel | Rollen, Nicken, Gas, Seitenruder (nicht umbelegbar) |
| CH5 | **SwA** | **ARM** (unten = scharfgeschaltet, nur bei Gas unten; nicht umbelegbar) |
| CH6 | SwB | Klappen (`Feature::FLAPS`) |
| CH7 | **SwC** (3 Stellungen) | oben **MANUAL** · Mitte **STABILIZE** · unten **AUTO_TAKEOFF** |
| CH8 | SwD | **RTH** – nach Hause, solange er eingeschaltet ist |
| CH9 | VrA | Stärke der Stabilisierung (`Knob::STAB_GAIN`) |
| CH10 | VrB | Reisegeschwindigkeit (`Knob::CRUISE_SPEED`) |

Beim Einschalten des Boards gibt der serielle Monitor die tatsächliche Belegung aus – das, was wirklich aufgespielt ist (die Firmware gibt Russisch aus; „вверх / середина / вниз“ bedeutet oben / Mitte / unten):

```
SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (вверх / середина / вниз)
SwB (CH6): FLAPS, пока включён
SwD (CH8): RTH, пока включён
VrA (CH9): крутилка STAB_GAIN
VrB (CH10): крутилка CRUISE_SPEED
```

> Die Kanäle 7–10 sind beim FS-i6 standardmäßig nicht ausgegeben. Menü des Senders: **Functions setup → Aux. channels**, dort SwC, SwD, VrA, VrB zuweisen.

---

## Eine Funktion mit einer Zeile zuweisen

Alles steht in einer einzigen Datei – `include/config/Controls.h`:

```cpp
constexpr Binding BINDINGS[] = {
    Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_AUTO_TAKEOFF),
    Bind::feature(Channels::SWB, Feature::FLAPS),
    Bind::mode   (Channels::SWD, MODE_RTH),
    Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
    Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
};
```

| Form der Zeile | Was sie tut |
|---|---|
| `Bind::modes(Kanal, oben, Mitte, unten)` | ein Dreistellungsschalter wählt den Modus |
| `Bind::modes(Kanal, oben, unten)` | ein Zweistellungsschalter – zwei Modi |
| `Bind::mode(Kanal, Modus)` | der Modus gilt **über** allen anderen, solange der Schalter eingeschaltet ist; ausgeschaltet – kehrt der Modus vom Modusschalter zurück |
| `Bind::feature(Kanal, Funktion)` | die Funktion wirkt, solange der Schalter eingeschaltet ist |
| `Bind::knob(Kanal, Drehregler)` | Drehregler: Mitte = Wert aus `Config.h`, Enden = Minimum und Maximum |

„Eingeschaltet“ heißt: Der Kanal liegt über 1750 µs (beim FS-i6 – Schalter nach unten, zu sich hin). Bis zum ersten Frame des Empfängers gelten alle Kanäle als ausgeschaltet: Beim Anlegen der Spannung wird nichts ausgefahren oder abgeworfen.

### Fertige Rezepte

```cpp
// Thermiksegler: SwD – Soaring, SwB – Auto-Trimm, VrB – Kreisradius
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_SOARING),
Bind::feature(Channels::SWB, Feature::AUTO_TRIM),
Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Flugschüler: nur Stabilisierung, RESCUE auf dem „Notfallknopf“, weiche Knüppel
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_ALT_HOLD, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_RESCUE),
Bind::feature(Channels::SWB, Feature::GEOFENCE),
Bind::knob   (Channels::VRA, Knob::RATES),
Bind::knob   (Channels::VRB, Knob::MAX_BANK),

// Aufnahme und Lieferung: Kamera mit Stabilisierung, Lastabwurf, Kreisen über einem Punkt
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_CRUISE, MODE_LOITER),
Bind::feature(Channels::SWB, Feature::PAYLOAD_DROP),
Bind::feature(Channels::SWD, Feature::CAMERA_STAB),
Bind::knob   (Channels::VRA, Knob::CAMERA_TILT),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Handstart ohne Fahrwerk: SwD – LAUNCH, Klappen stufenlos per Drehregler
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_LAUNCH),
Bind::feature(Channels::SWB, Feature::AIRBRAKE),
Bind::knob   (Channels::VRA, Knob::FLAPS),
Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
```

### Der Compiler fängt Fehler ab

Die Tabelle wird beim Bauen geprüft (`static_assert`), bevor die Firmware ins Flugzeug kommt:

- Knüppel und SwA (ARM) lassen sich nicht belegen, und die Kanalnummer muss kleiner als `Channels::COUNT` sein;
- ein Kanal hat nur eine Belegung;
- der Schalter zur Modusauswahl (`Bind::modes`) darf höchstens einmal vorkommen.

Sind mehrere `Bind::mode`-Schalter gleichzeitig eingeschaltet, gewinnt die oberste Zeile der Tabelle.

---

## Modi

Der Modus wechselt, wenn ein Schalter **umgelegt** wird. Ein vom Dashboard oder von der Bodenstation eingeschalteter Modus gilt, bis der Pilot wieder einen Modusschalter umlegt. Auf dem OLED erscheint ein Kurzname (in Klammern).

### MANUAL (MAN)
Ruder = Knüppel, wie ohne Flugsteuerung. Es arbeiten nur die Funktionen (Klappen, Bremse, Last) und der Auto-Trimm. **Der wichtigste Sicherheitsmodus**: Halten Sie ihn immer auf einem Schalter unter dem Finger.

### STABILIZE (STAB) — „der Knüppel gibt den Winkel vor“
Der Rollknüppel gibt den Rollwinkel bis `MAX_BANK_DEG` vor (45°, Drehregler `MAX_BANK` 15…60°), der Nickknüppel den Winkel bis `STAB_MAX_PITCH_DEG` (25°). Losgelassen – das Flugzeug richtet sich von selbst waagerecht aus. Das Gas liegt beim Piloten. Der Integrator des PID sammelt sich nur in der Nähe des Ziels (±10°), deshalb „überschwingt“ das Flugzeug nach einem scharfen Manöver den Horizont nicht.
**Benötigt:** IMU. Ohne IMU – wie MANUAL.

### ALT_HOLD (ALT) — „Höhe halten“
Wie STABILIZE beim Rollen, und das Höhenruder hält die Höhe nach dem Barometer. Bewegt man den Nickknüppel, steuert man selbst; lässt man los, hält das Flugzeug die **neue** Höhe. Das Gas liegt beim Piloten (geben Sie Gas, sonst reicht die Geschwindigkeit nicht zum Steigen).
**Benötigt:** IMU + Barometer.

### ACRO — „der Knüppel gibt die Drehrate vor“
Voller Knüppel – 180°/s. Losgelassen – das Flugzeug **hält die Lage, in der es sich befand** (auch auf dem Rücken), und das Gyroskop dämpft Böen. Für Kunstflug.
**Benötigt:** IMU. Ohne IMU – Ruder = Knüppel.

### CRUISE (CRZ) — „Kurs, Höhe und Geschwindigkeit halten“
Das Flugzeug fliegt geradeaus auf der aktuellen Höhe, das Gas ist automatisch (Drehregler `CRUISE_SPEED`: 30…55…85 % Gas, mit Pitotrohr – **Fluggeschwindigkeit** 10…14…22 m/s). Der Rollknüppel dient zum Kurven (losgelassen – hält es den neuen Kurs), der Nickknüppel zum Ändern der Höhe. Mit Pitotrohr arbeitet der Schutz vor dem Strömungsabriss. Der Kurs stammt vom GPS (bei einer Geschwindigkeit über Grund > 3 m/s; unter 2 m/s zurück zum Kompass), sonst vom Kompass, sonst vom Gyroskop. In der Simulation hält sich bei 4 m/s Seitenwind der Kurs über Grund innerhalb von ±5° und die Höhe innerhalb von ±3 m.
**Benötigt:** IMU + Barometer; GPS oder Kompass für einen Kurs ohne Abdrift.

### LOITER (LOIT) — „kreise hier“
Kreise im Uhrzeigersinn über dem Punkt, an dem der Modus eingeschaltet wurde, Radius 50 m (Drehregler `LOITER_RADIUS` 25…150 m), auf der aktuellen Höhe, das Gas automatisch. Die Führung erfolgt über ein Vektorfeld: Aus der Ferne läuft das Flugzeug tangential auf den Kreis zu, und auf dem Kreis hält es eine vorausschauende Schräglage. Ohne GPS – einfach ein Kreis mit konstanter Schräglage auf der Stelle.
**Benötigt:** IMU + Barometer + GPS.

### RTH — „nach Hause“
Kurs auf den Startpunkt (den Punkt des ARM), Höhe `RTH_ALTITUDE_M` = 40 m (darunter – steigt es unterwegs, darüber – bleibt es). Über dem Startpunkt – Kreise mit dem LOITER-Radius, bis der Pilot die Steuerung übernimmt. Ohne GPS oder Startpunkt – Kreise auf der Stelle. Derselbe Modus wird durch Verbindungsverlust und Geofence ausgelöst.
**Benötigt:** IMU + Barometer + GPS mit Startpunkt.

### AUTO_TAKEOFF (TKOFF) — „Start per Gas“
Nach dem ARM passiert nichts, bis der Pilot das Gas über die Mitte schiebt. Dann das Programm: 1 s Beschleunigung auf 100 % Gas bei waagerechten Flügeln, 2 s Abheben mit einem Nickwinkel von 15°, danach Steigflug mit einem Nickwinkel von 10°, bis der Pilot den Modus wechselt. Die Knüppel addieren sich zum Programm – den Kurs kann man auf dem Startlauf geraderichten.
**Benötigt:** IMU.

### LAUNCH (LNCH) — „Handstart“
1. Scharfgeschaltet, Gas über der Mitte – der Start ist **vorbereitet**, der Motor steht.
2. Der Wurf: Beschleunigung nach vorn > 1,5 g länger als 40 ms.
3. Nach 0,3 s (die Hand ist vom Propeller weg) – Gas 100 %, Steigflug mit einem Nickwinkel von 15°, Flügel waagerecht – 6 s oder bis 30 m.
4. Danach – wie CRUISE auf der erreichten Höhe.

Jede Knüppelbewegung (> 150 µs) vor dem Wurf bricht den Start ab: Das Flugzeug bleibt in der Hand des Piloten.
**Benötigt:** IMU (Beschleunigungssensor). In der Simulation: ein Wurf mit 9 m/s aus Handhöhe – das Flugzeug berührt nie den Boden und gewinnt in 15 s mehr als 15 m.

### AUTO_LAND (LAND) — „Landung“
Motor aus, Gleitflug auf Kurs mit einem Nickwinkel von −4°; unter 3 m laut Barometer – Abfangen (+4°). Der Rollknüppel korrigiert den Anflugkurs. Einschalten auf einer Geraden, gegen den Wind, in 20–40 m Höhe, mit reichlich Bahnlänge. In der Simulation erfolgt die Landung mit einer Vertikalgeschwindigkeit unter 1,5 m/s, die Flügel waagerecht, nicht auf die Nase.
**Benötigt:** IMU + Barometer (beim Einschalten am Boden genullt).

### SOARING (SOAR) — „Segelflug in Thermik“
Motor aus, Gleitflug. Ein Variometer (mit Pitotrohr – Gesamtenergie, ohne falsche „Thermik“ durch Ziehen am Knüppel) über 0,5 m/s länger als 1,5 s – das ist Thermik: Kreise mit 25° Schräglage. Fällt der mittlere Steigwert über 8 s unter −0,2 m/s, verlässt es die Thermik. Unter 30 m – Motor bis 100 m; weiter als 400 m vom Startpunkt entfernt – Gleitflug nach Hause. In der Simulation findet es eine Thermik (Kern 3 m/s) und gewinnt mehr als 50 m ohne Motor.
**Benötigt:** IMU + Barometer; GPS – für die Rückkehr zum Startpunkt.

### RESCUE (RESQ) — „Rettung“
Flügel waagerecht, Nase +8°, Gas 70 % – aus jeder Spirale. Orientierung verloren? Schalter umlegen und durchatmen. In der Simulation: aus einer Spirale mit 70° Schräglage und −40° Nase innerhalb von 4 s – Flügel waagerecht und Steigflug.
**Benötigt:** IMU.

---

## Funktionen (Schalter)

| Funktion | Was sie tut | Details und Werte (`Config.h`) |
|---|---|---|
| `FLAPS` | beide Querruder nach unten – Flaperons | `FLAPS_DEPLOYED_US` = 220 µs, sanft innerhalb von 1 s; das Rollen wirkt obendrauf |
| `AIRBRAKE` | beide Querruder nach oben – Bremsklappe, steilerer Gleitpfad | `AIRBRAKE_US` = 250; hat Vorrang vor den Klappen |
| `AUTO_TRIM` | lernt, das Flugzeug ohne Knüppel geradeaus zu halten: Das dauerhafte Ruderkommando im Geradeausflug „fließt“ in die Trimmung | 20 %/s, bis ±120 µs; wird nach dem DISARM **am Boden** im Flash gespeichert |
| `TURN_COORDINATION` | Seitenruder in die Kurve, Nase hoch in der Schräglage | in den Navigationsmodi immer eingeschaltet |
| `MOTOR_KILL` | Motor in jedem Modus aus, auch in den automatischen | stärker als jeder Modus und das Gas |
| `BEEPER` | Summer „Ich bin hier“ | ohne Schalter piept er von selbst: am Boden, Verbindung verloren > 10 s |
| `PAYLOAD_DROP` | Servo AUX1 offen, solange der Schalter eingeschaltet ist | 1000 µs geschlossen, 2000 offen |
| `GEOFENCE` | weiter als 500 m vom Startpunkt oder höher als 120 m – RTH | `GEOFENCE_ALWAYS_ON` – ohne Schalter |
| `HOME_RESET` | Startpunkt = aktueller Punkt (im Moment des Einschaltens des Schalters) | nur bei gutem GPS |
| `CAMERA_STAB` | die Kamera an AUX2 hält ihren Winkel zum Horizont | der Nickwinkel des Flugzeugs wird abgezogen |

## Drehregler

Die Mitte des Drehreglers = Standardwert aus `Config.h`; die Enden sind Minimum und Maximum. Ist er nicht zugewiesen, gilt der Standardwert.

| Drehregler | Minimum … Mitte … Maximum | Wo er wirkt |
|---|---|---|
| `STAB_GAIN` | ×0,25 … ×1 … ×2 | alle Modi mit Stabilisierung und ACRO – „weicher/härter“ |
| `MAX_BANK` | 15° … 45° … 60° | maximale Schräglage vom Knüppel und von der Navigation |
| `CRUISE_SPEED` | Gas 30 … 55 … 85 % (mit Pitot: 10 … 14 … 22 m/s) | CRUISE, LOITER, RTH, der Motor in SOARING |
| `FLAPS` | 0 … 50 … 100 % Klappen | stufenlose Klappen statt eines Schalters |
| `CAMERA_TILT` | −90° … 0° … +30° | Kamerawinkel (AUX2) |
| `RATES` | 30 … 65 … 100 % des Knüppelwegs | alle Modi: Empfindlichkeit der Knüppel |
| `LOITER_RADIUS` | 25 … 50 … 150 m | LOITER und Kreise über dem Startpunkt |

> Ein nützlicher Kniff: Der Drehregler `STAB_GAIN` auf VrA ist eine „Live“-Abstimmung der Koeffizienten im Flug. Schaukelt es sich auf – reduzieren; ist es träge – erhöhen; danach den Faktor in `Config.h` übernehmen.

---

## Verbindungsverlust, Geofence, Startpunkt

**Der Startpunkt** wird beim ARM gespeichert, wenn das GPS gut ist (3D-Fix, ≥ 6 Satelliten, Genauigkeit ≤ 5 m). Hat das GPS beim ARM noch keinen Fix, wird der Startpunkt gespeichert, sobald es einen hat. Zum Ändern im Feld – die Funktion `HOME_RESET`.

**Verbindungsverlust** (keine iBUS-Frames für > 0,5 s oder der Empfänger hat Gas unter 950 µs gesendet – das ist das im Sender eingestellte Failsafe; siehe `docs/PILOT_GUIDE.md`):

| Situation | Was das Fluggerät tut |
|---|---|
| am Boden (nicht scharfgeschaltet) | Motor 0, Ruder in Neutralstellung; nach 10 s – der Summer |
| in der Luft, GPS und Startpunkt vorhanden | **RTH** mit Motor, über dem Startpunkt – Kreise auf 40 m |
| in der Luft, kein GPS | **Gleitflug**: Motor aus, Flügel waagerecht, Nase −3° |
| die Verbindung ist wieder da | sofort der Modus vom Schalter des Piloten |

Ein bereits begonnener Rückflug fällt bei einem kurzen GPS-Ausfall nicht in den Gleitflug. `FAILSAFE_RTH = false` – nur Gleitflug.

**Geofence** (`GEOFENCE` oder `GEOFENCE_ALWAYS_ON`): Ein Verlassen weiter als `FENCE_RADIUS_M` (500 m) oder höher als `FENCE_ALTITUDE_M` (120 m) löst RTH aus. Um die Steuerung zurückzunehmen, legt man den Modusschalter auf eine andere Position (ein Modus wird durch das Wechseln der Position eingeschaltet). Er greift wieder, wenn das Flugzeug mit 10 % Reserve nach innen zurückgekehrt ist.

---

## Ein Pitotrohr zum Selberbauen

Fluggeschwindigkeit ohne gekauften Differenzdrucksensor: **zwei Barometer**.

```
       anströmende Luft ─►  ┌──────────── Rohr (PVC/Messing, Ø4–6 mm) ──┐
                            │  BMP581 (I2C 0x47) — Gesamtdruck          │  dicht
                            └───────────────────────────────────────────┘
   Rumpf: Hauptbarometer (BMP581 0x46 / SPL06 / BMP388) — statischer Druck

   Geschwindigkeit  V = √(2·(P_Rohr − P_statisch − Null) / ρ),   ρ — aus statischem Druck und Temperatur
```

**Aufbau.** Der BMP581 (Modul mit der Adresse 0x47: Pin SDO an VCC) wird in ein nur nach vorn offenes Rohr eingeklebt – die Platine sitzt in einem dichten Hohlraum, die Leitungen sind durch Dichtmasse nach außen geführt. Das Rohr zeigt nach vorn, außerhalb des Propellerstrahls (am Flügel oder über der Nase). Das zweite Barometer sitzt im Rumpf, vor dem direkten Luftstrom geschützt (Schaumstoff).

**Aktivierung in der Firmware** – `sensors/SensorSelection.h`: `SENSOR_KIT_LSM6DSV_PITOT` oder `SENSOR_KIT_ICM45686_PITOT` (fertige Sets) oder `SENSOR_AIRSPEED = SENSOR_AIRSPEED_PITOT_BMP581` im eigenen Set.

**Der Nullpunkt.** Zwei Barometer weichen immer etwas voneinander ab: Die absolute Genauigkeit jedes einzelnen liegt bei einigen zehn Pascal, und das ist bei niedriger Geschwindigkeit bereits der gesamte Druckunterschied (10 m/s ≈ 60 Pa). In der ersten Sekunde nach dem Einschalten mittelt die Firmware die Differenz und nimmt sie als Null. **Das Flugzeug steht beim Einschalten still, und das Rohr ist mit dem Finger oder einer Kappe abgedeckt oder in den Wind gedreht.** Auf dem OLED, im Dashboard und in der Telemetrie erscheint die Geschwindigkeit nach dem Nullen.

**Kalibrierung von `PITOT_SCALE`.** Der Druck im Rumpf ist nicht streng statisch. Fliegen Sie bei Windstille in CRUISE eine Gerade hin und zurück und vergleichen Sie die mittlere Geschwindigkeit über Grund vom GPS mit der Geschwindigkeit des Rohrs: `PITOT_SCALE = V_GPS / V_Rohr`.

**Schutz.** Ist die Druckdifferenz länger als 2 s stark negativ (Schläuche vertauscht, Wasser) oder sind die Messwerte des Rohrs älter als 0,2 s, wird keine Geschwindigkeit ausgegeben: Der Autopilot geht auf das Gas vom Drehregler und auf den Kurs ohne sie über. Geprüft in einer geschlossenen Simulation mit Rauschen auf beiden Barometern: Der Geschwindigkeitsfehler im Flug beträgt < 0,5 m/s.

Was das Rohr bringt: CRUISE hält die **Fluggeschwindigkeit** statt des Gases; Schutz vor dem Strömungsabriss; ein Gesamtenergie-Variometer für SOARING; eine ehrliche Geschwindigkeit in der Telemetrie.

---

## Bodenstation: WLAN-Dashboard und MAVLink

**ESP32 – WLAN-Dashboard.** Der Zugangspunkt `OpenPlane-Debug`, Passwort `12345678`, die Adresse steht im seriellen Monitor. Kanäle, Ausgänge, alle Sensoren, der Modus, die Navigation, die eingeschalteten Funktionen; man kann den Modus und den PID ändern. Details – `docs/PILOT_GUIDE.md`.

**STM32H743 – MAVLink über ein Funkmodem** (UART4: PD0 RX, PD1 TX, 57600 Baud – der Standard von SiK). Geeignet sind SiK 433/868/915 MHz, ELRS im MAVLink-Modus und ein ESP-01 als WLAN-Brücke. **QGroundControl** und **Mission Planner** sehen das Fluggerät als ArduPilot-Flugzeug:

- Horizont, Karte mit dem Startpunkt, Geschwindigkeit (vom Pitotrohr, falls vorhanden), Höhe, Variometer, Gas;
- die Modi unter den Namen von ArduPlane: STABILIZE → FBWA, ALT_HOLD → FBWB, CRUISE → CRUISE, LOITER → LOITER, RTH → RTL, AUTO_TAKEOFF/LAUNCH → TAKEOFF, SOARING → THERMAL, RESCUE → STABILIZE, AUTO_LAND → AUTO; Failsafe wird als RTL oder CIRCLE angezeigt;
- Meldungsleiste: ARM/DISARM, Moduswechsel (unter unserem Namen), Verbindungsverlust, Geofence;
- **Moduswechsel vom Boden aus** – mit der Modustaste in der GCS (außer AUTO: OpenPlane hat keine Missionen);
- **Parameter** `RLL_KP … PTCH_KD` – der PID für Rollen und Nicken, die im Flug direkt im Parameterfenster der GCS gelesen und geändert werden können (sie bleiben nach einem Neustart nicht erhalten – übernehmen Sie gute Werte in `Config.h`).

ARM/DISARM vom Boden aus wird **abgelehnt** – nur über den Schalter des Senders. Den Datenstrom ohne Hardware prüfen: `OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink && python3 tools/check_mavlink.py /tmp/tlm.bin` (dafür ist `pip install pymavlink` nötig).

---

## Reihenfolge bei der Einrichtung eines neuen Flugzeugs

1. **MANUAL am Boden:** Ruderrichtungen (`*_REVERSED` in `Config.h`), Klappen nach unten, AUX. Ausgänge prüfen mit `p` in der Konsole (Propeller abnehmen!).
2. **Sensoren am Boden:** `b` – Busabfrage, `s` – Status; `o` – Kalibrierung der IMU-Einbaulage (3 Lagen, einmalig), `m` – Kompass.
3. **STABILIZE am Boden:** Neigen Sie das Flugzeug nach rechts – das rechte Querruder muss **nach unten** gehen (zum Ausgleichen). Nase hoch – Höhenruder nach unten. Ist es anders, sind die Umkehrungen oder der IMU-Einbau vertauscht.
4. **Erster Flug in MANUAL**, Steigen auf über 50 m → STABILIZE. Schaukelt es – `STAB_GAIN` verringern; ist es träge – erhöhen.
5. **AUTO_TRIM** im Geradeausflug 20–30 s lang, Landung, DISARM – die Trimmung wird gespeichert.
6. **ALT_HOLD**, danach **CRUISE** – Höhe und Kurs prüfen; mit Pitotrohr `PITOT_SCALE` kalibrieren.
7. **LOITER** und **RTH** – in der Höhe, in Sichtweite, Finger auf MANUAL.
8. Erst danach – der **Failsafe-Test** (Sender in der Höhe ausschalten, das Flugzeug muss nach Hause fliegen) und automatischer Start/automatische Landung.

## Autopilot-Prüfung vor dem Flug

- [ ] Die beim Einschalten ausgegebene Belegung ist die erwartete.
- [ ] Konsole/OLED: IMU ok, die IMU-Prüfung vor dem Flug bestanden (das Flugzeug stand beim Einschalten still).
- [ ] Das Barometer ist am Boden genullt (Höhe ~0 auf dem OLED).
- [ ] Mit Pitotrohr: Geschwindigkeit ~0 am Boden; wenn man ins Rohr bläst, steigt sie.
- [ ] GPS: 3D-Fix, ≥ 6 Satelliten **vor dem ARM** – sonst gibt es keinen Startpunkt und kein RTH.
- [ ] STABILIZE am Boden: Querruder und Höhenruder richten das Flugzeug auf, statt es umzukippen.
- [ ] Failsafe ist im Sender eingerichtet (Gas unter 950 bei Verbindungsverlust) und durch Ausschalten des Senders **am Boden** ohne Propeller geprüft.
- [ ] MANUAL – unter dem Finger.

---

## Was jeder Modus braucht

| Modus | IMU | Barometer | GPS | Kompass | Pitot | Gas | Ohne den nötigen Sensor |
|---|:-:|:-:|:-:|:-:|:-:|---|---|
| MANUAL | | | | | | Pilot | — |
| STABILIZE | ● | | | | | Pilot | Ruder = Knüppel |
| ALT_HOLD | ● | ● | | | | Pilot | die Höhe hält der Pilot |
| ACRO | ● | | | | | Pilot | Ruder = Knüppel |
| CRUISE | ● | ● | ○ | ○ | ○ | automatisch | Kurs per Gyroskop (driftet), Höhe beim Piloten |
| LOITER | ● | ● | ● | | ○ | automatisch | Kreis mit Schräglage auf der Stelle |
| RTH | ● | ● | ● | | ○ | automatisch | Kreise auf der Stelle |
| AUTO_TAKEOFF | ● | | | | | Programm | Ruder = Knüppel + Gas des Piloten |
| LAUNCH | ● | ○ | | | | Programm | Ruder in Neutralstellung |
| AUTO_LAND | ● | ● | | ○ | | 0 | ohne Abfangen |
| SOARING | ● | ● | ○ | | ○ | 0 / Motor | ohne Thermik – Gleitflug |
| RESCUE | ● | | | | | 70 % | Ruder in Neutralstellung |

● – erforderlich, ○ – verbessert. Die Prüfungen im Code: `Autopilot.h` (`imuReady`, `baroReady`, `nav.gpsGood`); die Tests – `test/native/test_autopilot_modes` (Reaktion der Modi auf jeden Sensor) und `test/native/test_sim` (geschlossene Flüge).
