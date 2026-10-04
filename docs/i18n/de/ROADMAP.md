# OpenPlaneProject – Roadmap und Pitch für Investoren und Partner

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../ROADMAP.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

> Repository: [github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject), Branch `main`.
> Dieses Dokument ist die ausführlichere Fassung des README-Teasers und richtet sich an alle, die erwägen, Geld, Zeit
> oder eine Partnerschaft in das Projekt zu investieren. Es beschreibt, wo das Projekt jetzt steht, wohin und warum es sich bewegt,
> und was genau im bereits geschriebenen Code diesen Weg realistisch statt nur behauptet macht.

## 1. Aktueller Stand – ehrlich

OpenPlaneProject ist derzeit der Prototyp eines Motorseglers, der bereits geflogen ist, aber noch unausgereift ist, mit eigener Firmware auf dem ESP32 – kein fertiges Produkt und keine autonome Drohne. Hardware: 1200 mm Spannweite, 250 mm Flügeltiefe, Profil NACA 4412, eine Struktur aus PETG (3D-Druck), MG90S-Servos (ein eigenes für jedes Querruder), Stromversorgung über LiPo 3S. Der erste Prototyp (ESP32-C3, Motor D2212 1000KV, 40-A-ESC) ist bereits unter manueller Steuerung geflogen und hat nach dem Flug konkrete Probleme offenbart: unzureichende Festigkeit der Befestigung von Motor und Tragfläche (sie muss mit Carbon verstärkt werden) und die Notwendigkeit, die Servos einzustellen.

Der aktuelle Aufbau ist auf den ESP32-S3 (N16R8) umgezogen, mit einem Motor D3548 1100KV und einem 60–80-A-ESC; auf dem Prüfstand sind daran alle Sensoren des Autopiloten angeschlossen: eine IMU (MPU6500), ein Barometer BMP388, ein Kompass QMC5883P sowie ein OLED-Statusdisplay. Live überprüft: manuelle RC-Steuerung über iBUS mit einem Mischer für Querruder, Höhenruder, Seitenruder (mit lenkbarem Rad) und Klappen (Flaperons); ARM über einen eigenen Schalter; Failsafe bei ausgeschaltetem Sender; eine 500-Hz-Regelschleife; ein Live-Web-Dashboard über WLAN. Auf dem Tisch reagiert die Stabilisierung auf Neigungen in die richtige Richtung.

Seitdem hat die Firmware 12 Autopilot-Modi bekommen (Stabilisierung, Höhenhaltung, Reiseflug, Kreise und Rückkehr zum Startpunkt per GPS, Handstart, automatische Landung, Segelflug in Thermik, „Rettung“), einen Geofence, die Rückkehr zum Startpunkt bei Signalverlust, Lastabwurf, ein Pitotrohr aus zwei Barometern, Unterstützung neuer Sensoren (LSM6DSV, ICM-45686, QMC6309, SPL06, BMP581) und eine vollständige Firmware für den STM32H743 mit MAVLink-Telemetrie für QGroundControl. All das wurde durch 387 automatisierte Tests und geschlossene Flugsimulationen überprüft (die gesamte Firmware fliegt ein Flugzeugmodell), ist aber **in der Luft noch nicht erprobt**. Mit anderen Worten: Bisher ist nur die manuelle Steuerung geflogen; der Autopilot ist geschrieben, mit allem geprüft, was sich ohne Flug prüfen lässt, und wartet auf die Flugtests.

## 2. Warum das wichtig ist

Autonome Kleinluftfahrt mit niedriger Einstiegshürde deckt Aufgaben ab, bei denen nicht die maximale Nutzlast oder Reichweite entscheidet, sondern die Reaktionsgeschwindigkeit und niedrige Betriebskosten:

- **Lieferung von Medikamenten und Blut in schwer erreichbare Gebiete und in Katastrophengebiete** – weggespülte Straßen, fehlende Infrastruktur, Zerstörungen nach Naturkatastrophen oder Konflikten. Entscheidend ist hier nicht die Tragfähigkeit (das Paket ist klein), sondern die Reaktionszeit: Minuten und Stunden statt eines Tages mit dem Auto oder zu Fuß.
- **Such- und Rettungseinsätze** – gezielter Abwurf von Ausrüstung, Erste-Hilfe-Sets, Kommunikationsmitteln und Schwimmhilfen an Betroffene, bevor das Bodenteam eintrifft, in Zonen, in denen ein Hubschrauber übermäßig teuer oder wegen Wetter oder Gelände nicht verfügbar ist.
- **Präzisionslandwirtschaft** – Überwachung von Feldern und punktuelles Sprühen/Ausbringen von Mitteln dort, wo geschlossene Drohnenplattformen für diese Aufgaben ab Tausenden von Dollar kosten, was für kleine und mittlere Betriebe unwirtschaftlich ist.

In allen drei Kategorien ist die Ökonomie dieselbe: der Unterschied zwischen „es gibt eine Lösung, aber sie ist teuer und geschlossen“ und „es gibt faktisch keine Lösung, weil sie teuer ist“ – und genau auf diese Nische zielt eine günstige offene Plattform. Das ist eine Beschreibung des Einsatzbereichs und des Marktproblems, keine Behauptung, dass OpenPlaneProject schon Fracht liefern kann – im jetzigen Projektstadium ist es eine Aussage über das Ziel und darüber, warum sich das Ziel lohnt.

## 3. Investitionsthese: Warum eine offene ESP32-Architektur eine Asymmetrie ist

Kommerzielle autonome Drohnenplattformen für Lieferung/Überwachung basieren meist auf geschlossenen Flugsteuerungen und geschlossener Software, kosten von Hunderten bis zu Tausenden Dollar pro Fluggerät und erfordern Lizenzgebühren oder Servicevereinbarungen für den Betrieb der Flotte. OpenPlaneProject geht von einer anderen Annahme aus:

- **Eine günstige Basis.** Ein ESP32-Modul kostet in der Größenordnung von 5–15 $, und die übrige Beschaltung (MG90S-Servos, ESC, iBUS-Empfänger) besteht aus Standardkomponenten der Hobbyklasse. Das ist eine Größenordnung günstiger als das Eintrittsticket zu geschlossenen kommerziellen Plattformen, was für Pilotprojekte unter Budgetzwängen entscheidend ist (NGOs, Landwirtschaft in kleinem Maßstab, regionale Rettungsdienste).
- **Offener Code verändert die Ökonomie des Vertrauens.** Eine Organisation, die eine Flotte für medizinische Lieferungen einsetzt, kann die Sicherheit (Failsafe, ARM-Logik) prüfen und die Firmware an ihre Sensoren und Vorschriften anpassen, statt von einem einzigen Anbieter und dessen Roadmap abzuhängen.
- **Die Architektur ist schon heute auf Erweiterung ausgelegt, nicht nur auf den aktuellen Motorsegler.** Das ist keine Behauptung, sondern eine direkte Folge des Code-Aufbaus:
  - Ein neuer Sensor wird als Klasse hinzugefügt, die die bestehende Schnittstelle `Sensor` → `ImuSensor`/`BarometerSensor` (`include/sensors/SensorInterface.h`) implementiert, ohne den Kern zu ändern. So sind bereits die Treiber für IMUs (MPU6050/MPU6500, ICM-42688), Barometer (BMP388, BME280), Kompasse (QMC5883P/L) und GPS (u-blox M10) entstanden – alle direkt über die Register des Busses geschrieben (die Schnittstellen `II2CBus`/`ISpiBus`/`IUartPort`), ohne Bibliotheken von Drittanbietern, also ohne versteckte Abhängigkeiten von einem bestimmten Hersteller-SDK.
  - Ein neues Board kommt mit einem einzigen `#elif`-Block in `include/config/Config.h` plus einem einzigen `[env:...]`-Block in `platformio.ini` hinzu – der Boardwechsel funktioniert schon heute für vier Ziele (siehe die Tabelle unten); das ist keine hypothetische Möglichkeit.
  - `Autopilot.h` nimmt `ImuSensor*`/`BarometerSensor*` bereits als Parameter entgegen (sie dürfen `nullptr` sein) – der Vertrag zwischen Autopilot und Hardware geht also davon aus, dass sich die Sensorbestückung ändert (der nächste Schritt ist GPS als eine weitere Klasse nach demselben Muster, siehe Phase 3).
  - `WebDebugServer.h` liefert bereits ein einziges aggregiertes JSON (`GET /api/status`) und nimmt Befehle entgegen (`POST /api/setmode`, `/api/setpid`) – das Protokoll „das Fluggerät liefert Telemetrie und nimmt Befehle an“ existiert also schon, und die Bodenstation soll daraus erwachsen, statt von Grund auf neu geschrieben zu werden.

Die Asymmetrie besteht darin, dass die Einstiegshürde (Geld, Zeit für die Anpassung an eine neue Aufgabe) dieser Plattform um eine Größenordnung niedriger ist als bei geschlossenen Alternativen, während der Weg zur Autonomie kein Umschreiben des Kerns erfordert – nur das Hinzufügen neuer Klassen auf den bestehenden Schnittstellen. Das ist eine offene Engineering-Plattform, kein fertiges kommerzielles Produkt – die These für Investoren/Partner lautet entsprechend nicht „kaufen Sie eine fertige Lösung“, sondern „steigen Sie in einer Phase ein, in der das Fundament bereits geprüft ist und die nächsten Schritte technisch klar sind“.

## 4. Die Architektur heute – das Fundament für die folgenden Phasen

### 4.1 Unterstützte Boards

Die Wahl des Boards ist eine einzige Build-Option von PlatformIO; für den Boardwechsel muss die Logik nicht angefasst werden (`include/config/Config.h` + `platformio.ini`):

| Umgebung (`pio run -e ...`) | Board | Status | aileron L / R | elevator | esc | ibus_rx | i2c sda / scl |
|---|---|---|---|---|---|---|---|
| `esp32-s3` (default) | ESP32-S3 N16R8 (DevKitC-1) | **Hauptboard, auf dem Prüfstand mit allen Sensoren geprüft** | GPIO4 / GPIO5 | GPIO6 | GPIO7 | GPIO17 | GPIO41 / GPIO42 |
| `esp32-c3` | ESP32-C3 SuperMini | Erster Prototyp, mit manueller Steuerung geflogen | GPIO5 / GPIO4 | GPIO6 | GPIO7 | GPIO8 | GPIO1 / GPIO3 |
| `esp32-dev` | klassischer ESP32 mit 38 Pins | Für den Prüfstand, **nicht auf der Hardware geprüft** (die gesamte Firmware steckt in den Tests) | GPIO13 / GPIO14 | GPIO27 | GPIO26 | GPIO16 | GPIO21 / GPIO22 |
| `stm32h743` | STM32H743VIT6 (WeAct Mini) | Vollständige Firmware + MAVLink, **noch kein Board** (die gesamte Firmware läuft in den Tests auf dem PC) | PA0 / PA1 | PA2 | PA3 | PE7 | PB11 / PB10 |

Firmware aufspielen: `pio run -t upload`. Monitor: `pio device monitor` (115200).

### 4.2 Karte der RC-Kanäle (FS-i6 + FS-iA6B, iBUS, 10 Kanäle, 1000–2000 µs)

| Kanal | Name | Standardbelegung |
|---|---|---|
| CH1–CH4 | Knüppel | Roll, Nick, Gas, Seitenruder |
| CH5 | ARM | Schalter SwA: ARM bei Gas unten, DISARM sofort |
| CH6 | SWB | Klappen |
| CH7 | SWC | Modus: MANUAL / STABILIZE / AUTO_TAKEOFF |
| CH8 | SWD | RTH – Rückkehr zum Startpunkt |
| CH9 | VRA | Stärke der Stabilisierung |
| CH10 | VRB | Reisegeschwindigkeit |

CH6–CH10 werden mit einer einzigen Zeile in `include/config/Controls.h` zugewiesen: jeder der 12 Modi, 10 Funktionen (Klappen, Bremse, Lastabwurf, Geofence …) und 7 Drehregler – [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

### 4.3 Steuerungsablauf (`FlightController::update()`)

Ein einziger Orchestrator mit fester Reihenfolge, die Schleife läuft mit 500 Hz bei festem Takt: iBUS-Empfang → Gas des Piloten → Modus von CH7 → **Sensoren lesen und Autopilot berechnen (immer, auch ohne Verbindung)** → **Prüfung auf Signalverlust mit absoluter Priorität** (in der Luft – per GPS mit Motor zum Startpunkt oder ohne GPS Gleitflug mit waagerechten Flächen; am Boden – Ruder in Neutral) → ARM → Knüppelkommando + Korrekturen des Autopiloten in einheitlichen Luftfahrtvorzeichen → Mischer mit Servo-Umkehr → Gas des Modus → Gassperre ohne ARM → Ausgabe an Servos/ESC. Gerade diese Disziplin der Reihenfolge (zuerst Sicherheit, dann manuelle Steuerung, dann der Autopilot als Aufsatz) ist der Grund, warum sich immer autonomeres Verhalten sicher einbauen lässt, ohne den Basiszyklus umzuschreiben. WLAN, Dashboard und Display laufen auf dem zweiten Kern und verzögern die Steuerung nicht.

### 4.4 Das Web-Dashboard als Keimzelle einer Bodenstation

`WebDebugServer.h` startet schon heute einen Zugangspunkt (SSID `OpenPlane-Debug`, IP `192.168.4.1`) und liefert und akzeptiert JSON:

| Methode und Pfad | Was es tut |
|---|---|
| `GET /api/status` | Ein einziges aggregiertes JSON: RC (10 Kanäle), armed/failsafe, 7 Ausgänge (`us`, `attached`), IMU, Barometer, Kompass, GPS, Pitotrohr (bei jedem `attached`/`available` + Daten), Autopilot (Modus, Korrekturen, PID, Navigation, aktivierte Funktionen) |
| `POST /api/setmode` | `{mode: 0-11}` – den Autopilot-Modus umschalten |
| `POST /api/setpid` | `{kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch}` – jedes Feld ist optional |
| `GET /` | Das HTML-Dashboard: Live-Balken der 10 Kanäle, Status jedes Ausgangs/Sensors, Modus-Schaltflächen, ein PID-Formular |

Die Felder `attached`/`available` sind im JSON immer vorhanden – das Dashboard zeigt ehrlich „nicht in der Konfiguration“ getrennt von „in der Konfiguration, antwortet aber nicht“, statt über einen fehlenden Sensor zu schweigen. Das ist dasselbe Prinzip der Ehrlichkeit, auf dem das ganze Dokument beruht: Gewünschtes nicht als Tatsächliches auszugeben.

## 5. Technische Roadmap Phase für Phase

Nachfolgend acht Phasen, jede beschrieben als der nächste logische Schritt auf den bereits vorhandenen Klassen, ohne erfundene Termine und Summen.

### Phase 1 – Manuelle Steuerung und sichere Basis [fertig]

**Ziel:** ein zuverlässiger, vorhersagbarer ferngesteuerter Motorsegler mit transparenter Fehlersuche. **Was technisch schon vorhanden ist:** iBUS-Parsing mit Erkennung von Signalverlust in `IBusReceiver.h`, der Mischer `ControlMixer.h` (Knüppel → Kommando für Roll/Nick/Klappen → PWM mit Servo-Umkehr, ohne etwas von UART/PWM zu wissen), `ThrottleManager.h`, `ArmingManager.h` (ARM über einen eigenen Schalter bei Gas unten, sofortiges DISARM), Failsafe mit absoluter Priorität in `FlightController.h`, Ausgabe auf Serial (`DebugLogger.h`), das Web-Dashboard (`WebDebugServer.h`) und das OLED-Display (`OledDisplay.h`). **Warum das die Basis für alles andere ist:** Es ist die einzige Schicht, die immer funktionieren muss, selbst wenn alle anderen Phasen noch nicht umgesetzt sind oder ihre Sensoren abgeschaltet sind – genau deshalb wurden Failsafe und ARM zuerst geschrieben und auf der Hardware geprüft (einschließlich des echten Verhaltens des Empfängers FS-iA6B bei ausgeschaltetem Sender).

### Phase 2 – IMU + Barometer → Autopilot [geschrieben und durch Tests und Simulation geprüft, wartet auf Flugtests]

**Ziel:** der erste autonome Flugmodus – Horizontstabilisierung, automatischer Start, Höhenhaltung. **Was technisch schon vorhanden ist:** `imu/MPU6050_Sensor.h` (MPU6050 und MPU6500, Register direkt, Achsendrehung passend zum Einbau des Boards, luftfahrttypische Vorzeichen, ein Komplementärfilter in `ImuSensorBase`), `baro/BMP388_Sensor.h` (I2C oder SPI, vollständige Bosch-Kompensation, Lesen nach dem Bereit-Flag, gefilterte Vertikalgeschwindigkeit), `Autopilot.h` mit `PidController` (der D-Anteil aus dem Gyroskop, der Integrator sammelt erst nach ARM) und zwölf Modi (von MANUAL bis SOARING und RESCUE), die per Schalter nach der Tabelle in `Controls.h`, vom Web-Dashboard und aus QGroundControl umgeschaltet werden, dazu Failsafe (nach Hause oder Gleitflug). Jeder Modus fliegt in einer geschlossenen Simulation der gesamten Firmware mit einem Flugzeugmodell (`test/native/test_sim`). Auf dem Prüfstand mit dem ESP32-S3 antworten alle Sensoren und die Vorzeichen sind live geprüft: Neigung → Korrektur der Ruder in Richtung Ausgleich. **Was nötig ist, um die Phase abzuschließen:** die Elektronik in den Motorsegler einbauen, die Ruderrichtungen am fertig aufgebauten Fluggerät prüfen und die ersten Flugtests durchführen – beginnend mit STABILIZE in sicherer Höhe.

### Phase 2.5 – Rückführung vom realen Flugzeug [Vorarbeit, in der Simulation geprüft]

**Ziel:** dass der Autopilot nicht von Koeffizienten abhängt, die für eine einzige Geschwindigkeit abgestimmt wurden, sondern davon, wie das reale Flugzeug gerade jetzt auf das Ruder reagiert. Der PID aus Phase 2 lenkt das Ruder „nach Formel“ aus und prüft das Ergebnis nicht; bei niedriger Geschwindigkeit korrigiert er zu wenig, bei hoher übersteuert er. **Was technisch schon vorhanden ist** (`include/autopilot/feedback/`, **nicht an die Firmware angeschlossen**): die Schätzung der Ruderwirksamkeit im Flug (rekursive kleinste Quadrate, Umrechnung nach der Geschwindigkeit ∝ V²), der Regler „Winkel → Drehrate → Ruder“ mit Nachkorrektur („das Ruder ist nicht bis zum Anschlag gelaufen – weiter drehen“), der Schutz vor Geschwindigkeitsverlust und Strömungsabriss (Gas, Nase nach unten, Flächen waagerecht), Start von einer Bahn oder von Hand und Landung in Stufen nach den Sensoren. Alles wurde durch eine geschlossene Flugzeugsimulation auf dem Board selbst geprüft (`pio test -e esp32-s3 -f test_feedback`, 10 Szenarien) – einschließlich eines vertauschten Querruders, Turbulenz, einer hochgezogenen Nase bei wenig Gas, Start und Landung. **Was nötig ist, um die Phase abzuschließen:** nach den ersten Flügen der Phase 2 – ein „Schattenmodus“ (die Rückführung schreibt nur ins Log, was sie getan hätte), dann das Zuschalten Achse für Achse, ein Fluggeschwindigkeitssensor (Pitotrohr) und ein Entfernungsmesser für das Abfangen vor dem Aufsetzen. Einzelheiten – der Abschnitt „Rückführung“ im [`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md).

### Phase 3 – GPS und Kompass [in der Firmware, durch Simulation geprüft]

**Stand (aktualisiert): Die Navigation funktioniert** – Kurs nach GPS/Kompass/Gyroskop mit Hysterese, Startpunkt bei ARM, CRUISE, LOITER (ein Vektorfeld auf einen Kreis), RTH, Geofence; durch geschlossene Simulationen geprüft. Unten steht der ursprüngliche Eintrag der Phase. GPS (u-blox M10, `sensors/gps/UbloxM10_Gps.h`, Protokoll UBX-NAV-PVT) und Magnetometer (zwei Varianten des Boards „GY-273“: QMC5883P – `sensors/mag/QMC5883P_Sensor.h`, sitzt auf dem aktuellen Prüfstand und ist live geprüft; QMC5883L – `sensors/mag/QMC5883L_Sensor.h`) wurden als neue Klassen hinzugefügt, die die Schnittstellen `GpsSensor`/`MagnetometerSensor` (`include/sensors/SensorInterface.h`) nach demselben Prinzip wie IMU und Barometer implementieren – `Autopilot.h` und `FlightController.h` wurden nicht umgeschrieben, sie erhielten auf dieselbe Weise zwei weitere Datenquellen (ein Nullable-Zeiger im Konstruktor). Nebenbei entstand die HAL-Schicht (`include/hal/`), über die die Sensoren auf die Busse zugreifen – I2C/SPI/UART hängen nicht mehr direkt an den ESP32-spezifischen `Wire`/`SPI`/`HardwareSerial`.

Derzeit sind die GPS-/Kompassdaten über `Autopilot::getGpsSensor()`/`getMagnetometerSensor()` und in `GET /api/status` verfügbar, dazu die einmalige Festlegung des anfänglichen Yaw nach dem Kompass beim Start – **aber sie nehmen nicht an der Steuerung teil**. Offene Fragen zum Abschluss der Phase: das GPS-Modul an den ESP32-S3 anschließen (die Pins von UART2 sind schon reserviert), den Kompass am fertig aufgebauten Fluggerät kalibrieren und dem Kurs eine Neigungskompensation hinzufügen. Beim ESP32-C3 ist ein vollwertiges GPS unmöglich – es fehlen GPIOs für TX (siehe die Pinbelegung im `DEVELOPER_GUIDE.md`).

### Phase 4 – Flug entlang von Wegpunkten (Waypoint-Navigation) [nächster Schritt]

**Stand:** Die Grundbausteine sind fertig – `Guidance::rollForCourse`, ein Kreis um einen Punkt, die Rückkehr zu einem Punkt (RTH), ein MAVLink-Kanal zum Hochladen der Mission (derzeit antwortet das Fluggerät auf eine Missionsanfrage ehrlich mit „0 Punkte“). Es fehlen: die Speicherung der Route, der Übergang zwischen den Punkten, das MAVLink-Protokoll MISSION_*.

**Ziel:** das Fluggerät fliegt einen vorgegebenen Satz von Koordinaten ab, ohne dass der Bediener auf jedem Streckenabschnitt eingreift. **Wie sich das in die Architektur einfügt:** Es ist ein neuer `AutopilotMode` in `Autopilot.h`, gleichberechtigt neben den bestehenden MANUAL/STABILIZE/AUTO_TAKEOFF/ALT_HOLD – der Mechanismus zum Umschalten der Modi (über RC-Slots und über `POST /api/setmode`) ändert sich also nicht; es kommt ein fünfter Modus hinzu, der Kurs und Entfernung aus dem GPS (Phase 3) statt aus manueller Eingabe über die Fernsteuerung nimmt. **Was technisch nötig ist:** ein Algorithmus zur Berechnung des Kurses zu einem Punkt und die Logik des Übergangs zwischen den Routenpunkten, dazu ein Weg, die Route selbst auf das Fluggerät hochzuladen (der natürliche Kandidat ist die Erweiterung derselben HTTP-API, mit der schon die Modi und der PID gesteuert werden).

### Phase 5 – Telemetrie mit großer Reichweite [in der STM32-Firmware umgesetzt]

**Stand:** Auf dem STM32H743 – MAVLink 2 über ein Funkmodem (SiK, ELRS im MAVLink-Modus): Lage, Position, Geschwindigkeit, Modus, PID-Parameter, Moduswechsel vom Boden aus. Die Frames wurden mit der Referenz pymavlink abgeglichen. Der ESP32 hat keinen freien UART – dort gibt es das WLAN-Dashboard. Unten steht der ursprüngliche Eintrag der Phase.

**Ziel:** eine Verbindung zwischen Fluggerät und Boden auf Entfernungen, die für eine echte Lieferung relevant sind, nicht für den Prüfstand. **Eine ehrliche Einschätzung des aktuellen Stands:** Der WLAN-Zugangspunkt von `WebDebugServer` überträgt schon heute den vollständigen aggregierten Status und die Steuerbefehle, aber die Reichweite eines gewöhnlichen WLAN-Zugangspunkts beträgt einige Dutzend Meter, was für die Fehlersuche auf dem Tisch oder auf dem Flugplatz reicht, nicht aber für eine autonome Route außerhalb der Sichtweite. **Was technisch nötig ist:** ein separater Funkkanal mit größerer Reichweite (zum Beispiel ein LoRa-Modul oder ein spezielles Telemetrie-Funkmodem) als Transport für dasselbe Datenformat, das in `GET /api/status` bereits definiert ist – also ein Ersatz oder eine Ergänzung der Transportschicht, nicht das Umschreiben des Telemetrieformats.

### Phase 6 – Eine vollwertige GUI für die Bodensteuerung [teilweise: QGroundControl / Mission Planner]

**Stand:** Dank MAVLink sehen Standard-Bodenstationen das Fluggerät bereits (Karte, Startpunkt, Instrumente, Modi unter den Namen von ArduPlane). Eine eigene Station für eine Flotte bleibt ein Ziel. Unten steht der ursprüngliche Eintrag der Phase.

**Ziel:** eine Station zur Missionsplanung mit Karte, Live-Telemetrie und Flottenverwaltung, keine Fehlersuchseite für ein einzelnes Fluggerät. **Wie sich das in die Architektur einfügt:** `WebDebugServer.h` ist schon heute keine Attrappe, sondern ein funktionierender Webserver mit aggregiertem JSON-Status und Befehls-API (siehe die Tabelle in Abschnitt 4.4); er ist der Ausgangspunkt, nicht etwas, das man wegwerfen müsste. Die nächsten Schritte sind eine Karte mit der aktuellen Position (nach Phase 3), Anzeige und Hochladen einer Route (nach Phase 4), Betrieb über einen Funkkanal mit großer Reichweite (nach Phase 5) und die Skalierung der Oberfläche von einem Fluggerät auf mehrere. Mehr dazu in Abschnitt 6.

### Phase 7 – Lastabwurf-Mechanismus und Schutzmaßnahmen für die Lieferung [in der Firmware umgesetzt, noch nicht geflogen]

**Stand:** Lastabwurf (Servo AUX1, die Funktion `PAYLOAD_DROP` auf jedem Schalter), Geofence (Radius und Obergrenze → RTH), Rückkehr zum Startpunkt bei Signalverlust, der Summer „Modell verloren“. Unten steht der ursprüngliche Eintrag der Phase.

**Ziel:** die Plattform von „einem Flugzeug, das autonom fliegt“ zu „einem Flugzeug, das autonom liefert“ zu machen. **Was technisch nötig ist:** ein zusätzliches Servo für den Mechanismus zum Abwerfen/Ausgeben der Last, nach demselben Prinzip gesteuert wie die übrigen Ausgänge in `FlightOutputs.h`; und Schutzmaßnahmen, die speziell für die Lieferung und nicht für neutrales Fliegen gelten – Geofences (Begrenzung des Flugbereichs) und die automatische Rückkehr zum Startpunkt bei Signalverlust (heute schaltet `FlightController` bei Signalverlust in der Luft den Motor ab und geht in den Gleitflug mit waagerechten Flächen über, was für einen von Hand gesteuerten Motorsegler richtig ist, aber für die autonome Lieferung ist der logische nächste Schritt die Rückkehr zur Basis per GPS statt eines bloßen Gleitflugs).

### Phase 8 – Skalierung auf eine Flotte

**Ziel:** mehrere Fluggeräte gleichzeitig verwalten – Aufgabenverteilung, ein Betriebspanel, Flugverlauf. Das ist die Ebene, auf der das Projekt nicht mehr nur ein ingenieurmäßiger Hobby-Prototyp ist, sondern zu einem Betriebswerkzeug wird, das als geschäftliche Aufgabe interessant ist: Routenplanung für mehrere Fluggeräte, eine Aufgabenwarteschlange, der Status jedes Fluggeräts in Echtzeit. Technisch ist das ein Aufbau auf den Phasen 3–6 (GPS, Telemetrie, GUI) – im Grunde dieselbe API von `WebDebugServer`, aber auf viele Telemetriequellen statt auf eine erweitert.

## 6. GUI: von einer Fehlersuchseite zur Bodenstation

Die Kernthese dieses Abschnitts: Die GUI muss nicht von Grund auf gebaut werden – sie existiert teilweise bereits und funktioniert. `WebDebugServer.h` heute:

- liefert einen einzigen aggregierten JSON-Schnappschuss des Zustands des Fluggeräts (`GET /api/status`) – die RC-Kanäle, die Flags armed/failsafe, den Zustand jedes Ausgangs, den Zustand jedes Sensors (ehrlich, mit getrennten Flags `attached` und `available`) und den Zustand des Autopiloten;
- nimmt Steuerbefehle in Echtzeit entgegen (Moduswechsel, PID-Anpassung), ohne neu zu flashen;
- liefert ein fertiges HTML-Dashboard mit Live-Kanalbalken und Bedienschaltflächen.

Der Weg zu einer vollwertigen Bodenstation ist eine schrittweise Erweiterung des bereits funktionierenden Protokolls, kein Architekturwechsel:

1. Dem Dashboard eine Karte und die aktuelle Position hinzufügen – dafür wird GPS (Phase 3) als ein weiteres Feld im selben JSON-Status gebraucht.
2. Das Erstellen und Hochladen von Routen hinzufügen – dafür werden ein Wegpunktmodus (Phase 4) und eine Erweiterung der POST-API nach dem Muster von `/api/setmode`/`/api/setpid` gebraucht.
3. Den Transport vom WLAN-Zugangspunkt auf eine Funkverbindung mit großer Reichweite verlegen (Phase 5) und dabei dasselbe Nachrichtenformat beibehalten, damit das bestehende Frontend nicht neu geschrieben werden muss.
4. Die Oberfläche von einem Fluggerät auf mehrere Telemetriequellen skalieren (Phase 8).

Mit anderen Worten: Das Element der künftigen Bodenstation, das unter dem Gesichtspunkt „muss man es von Grund auf schreiben“ am riskantesten ist – die Serialisierung des Zustands des Fluggeräts und die Befehls-API –, ist bereits umgesetzt und live auf dem Board geprüft.

## 7. Ehrliche Einschränkungen – was noch nicht funktioniert

Damit die Roadmap nicht wie Marketing wirkt, halten wir gesondert fest, was noch nicht oder nur teilweise erledigt ist:

- Der Autopilot wurde auf dem Prüfstand, mit 387 automatisierten Tests und mit geschlossenen Simulationen geprüft, aber nie im Flug erprobt – bisher ist nur die manuelle Steuerung geflogen (am ersten Prototyp). Das Flugzeugmodell in den Simulationen ist vereinfacht, die Koeffizienten sind Startwerte.
- Die neuen Sensoren (LSM6DSV, ICM-45686, QMC6309, SPL06, BMP581) wurden mit Registeremulatoren nach den Datenblättern geprüft; auf der Hardware noch nicht.
- STM32H743: Die gesamte Firmware läuft auf einem PC über STM32duino-Fakes; ein physisches Board gibt es noch nicht.
- Der Horizont für die Stabilisierung ist die Lage des Fluggeräts beim Einschalten (die IMU kalibriert sich bei jedem Start) oder die Einbaukalibrierung aus drei Lagen.
- Der physische Anschluss eines Servos ist für die Software nicht sichtbar; sichtbar ist nur, dass der Impuls tatsächlich am Pin ausgegeben wird (ein Selbsttest über die Konsole).
- Die Pinbelegung des gewöhnlichen ESP32 mit 38 Pins wurde nach der Dokumentation des Chips gewählt und nicht auf der Hardware geprüft.
- Die Lizenz ist die [OpenPlane License](LICENSE.md): MIT mit verpflichtender Nennung des Autors und Verboten der militärischen Nutzung sowie der vorsätzlichen Schädigung von Menschen und Sachen ohne deren Einwilligung. Wegen dieser Verbote gilt sie im Sinne der OSI nicht als „Open Source“.

## 8. Offene Fragen – eine Einladung zur Diskussion

Nachfolgend die Punkte, auf die das Projekt noch keine Antwort hat; sie sind bewusst als Fragen an einen potenziellen Partner oder Investor formuliert und nicht als entschiedene Tatsachen:

- Das Finanzierungsmodell und sein Umfang – verhandelbar; konkrete Summen und Fristen gibt es derzeit nicht, und in diesem Dokument werden sie auch nicht erfunden.
- Die Rechtsform des Projekts (ein Unternehmen, eine Stiftung, eine reine Open-Source-Community) – offen für die Diskussion mit denen, die an einer Partnerschaft interessiert sind.
- Die Zusammensetzung des Teams – derzeit wird das Projekt öffentlich geführt und ist offen für Beteiligung; konkrete Rollen und Verpflichtungen werden nicht im Voraus festgelegt.

Wenn Ihnen etwas davon als potenziellem Partner wichtig ist, ist der richtige Ort für ein Gespräch darüber die GitHub-Diskussion des Projekts (siehe Abschnitt 9) und nicht Mutmaßungen in diesem Dokument.

## 9. Wie man Kontakt aufnimmt und mitmacht

Der einzige offizielle Kanal des Projekts ist heute das Repository auf GitHub:
[github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject)
(Branch `main`). Andere Kontakte (E-Mail, soziale Netzwerke, eine juristische Person) gibt es derzeit nicht, und sie sind hier bewusst nicht angegeben, um nicht in die Irre zu führen.

- **Issues** – einen Fehler melden, eine konkrete technische Änderung vorschlagen, über die Ergebnisse eines Flugtests an der eigenen Kopie des Prototyps berichten.
- **Discussions** – die Roadmap, eine Partnerschaft, den Einsatz bei einer konkreten Aufgabe (Lieferung, Suche und Rettung, Landwirtschaft) sowie die Lizenz- und Finanzierungsfragen aus Abschnitt 8 besprechen.
- **Pull Requests** – einen neuen Sensor über die Schnittstelle `Sensor`, ein neues Board über einen Block in `Config.h`, einen neuen `AutopilotMode`, Verbesserungen am Web-Dashboard hinzufügen – die Architektur ist so angelegt, dass sich das ohne Eingriff in den Kern machen lässt.

Wenn Sie dieses Dokument als potenzieller Investor oder Partner lesen: Der nächste sinnvolle Schritt ist nicht, etwas zu unterschreiben, sondern im Repository eine Discussion mit einer konkreten Frage oder einem Vorschlag zu eröffnen. Die Roadmap oben ist eine Einladung, sie Phase für Phase zu besprechen, mit vollem Zugriff auf den Code, auf dem sie beruht.
