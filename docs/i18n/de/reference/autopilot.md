# AUTOPILOT — Modi, Navigation, Schalter

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/autopilot.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

[← Referenz](README.md)

Der Autopilot erhält die Knüppel des Piloten, die Schalter/Drehregler
(`PilotInputs`) und die Sensoren und gibt **das endgültige Ruderkommando**
(`getCommand()`) und das Gas des Modus (`applyThrottle()`) aus. Fehlt ein
benötigter Sensor, verhält sich der Modus sicher (die Ruder bleiben beim
Piloten oder gehen in Neutral), statt abzustürzen. Was jeder Modus für den
Piloten tut, steht im [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md). Im Flug war bisher nur der manuelle Modus; der
Autopilot wurde am Prüfstand, mit Tests und mit Simulationen im
geschlossenen Regelkreis geprüft (`test/native/test_sim`).

---

## `AutopilotMode`, `Feature`, `Knob`

**Datei:** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t` (ohne Gültigkeitsbereich — die numerischen Codes
gehen in das JSON von `/api/setmode`, `/api/status` und in `MavlinkModes`):

| Wert | Code | Kurz (OLED) | Kern |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | Ruder = Knüppel |
| `MODE_STABILIZE` | 1 | STAB | der Knüppel ist der Roll-/Nickwinkel |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | ein Startprogramm, das vom Gas des Piloten angestoßen wird |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + Höhe mit dem Höhenruder |
| `MODE_ACRO` | 4 | ACRO | der Knüppel ist die Winkelgeschwindigkeit |
| `MODE_CRUISE` | 5 | CRZ | Kurs + Höhe + automatisches Gas |
| `MODE_LOITER` | 6 | LOIT | Kreise über dem Punkt des Einschaltens |
| `MODE_RTH` | 7 | RTH | zum Startpunkt, Kreise darüber |
| `MODE_LAUNCH` | 8 | LNCH | Handstart |
| `MODE_AUTO_LAND` | 9 | LAND | Gleiten + Abfangen |
| `MODE_SOARING` | 10 | SOAR | Thermik ohne Motor |
| `MODE_RESCUE` | 11 | RESQ | Flügel waagerecht, Nase hoch, Gas |
| `MODE_COUNT` | 12 | | die Grenze (`setMode()` ignoriert ≥) |

`enum class Feature : uint8_t` — die Funktionen der Schalter: `FLAPS`,
`AIRBRAKE`, `AUTO_TRIM`, `TURN_COORDINATION`, `MOTOR_KILL`, `BEEPER`,
`PAYLOAD_DROP`, `GEOFENCE`, `HOME_RESET`, `CAMERA_STAB`, `COUNT`.

`enum class Knob : uint8_t` — die Drehregler: `STAB_GAIN`, `MAX_BANK`,
`CRUISE_SPEED`, `FLAPS`, `CAMERA_TILT`, `RATES`, `LOITER_RADIUS`, `COUNT`.

`namespace AutopilotNames` — `mode()`, `modeShort()` (≤ 5 Zeichen),
`feature()`, `knob()`: Namen für Log, OLED, Dashboard und MAVLink.

### `PilotInputs`

Der Zustand der Schalter und Drehregler in einem Zyklus.

| Mitglied | Beschreibung |
|---|---|
| `bool has(Feature) const` | die Funktion ist eingeschaltet |
| `float knob(Knob) const` | die Stellung des Drehreglers −1…+1 |
| `bool isBound(Knob) const` | der Drehregler steht in der Zuordnungstabelle |
| `float knobValue(Knob, min, default, max) const` | in Einheiten: die Mitte ist `default`, die Enden sind `min`/`max`; nicht zugeordnet — `default` |

---

## `Binding`, `Bind`, `BindingCheck`

**Datei:** `autopilot/ControlBinding.h` · die Tabelle — `config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`,
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`. Die Tabellenzeilen sind die
Fabriken von `namespace Bind` (alle `constexpr`):

| Fabrik | Bedeutung |
|---|---|
| `modes(ch, up, middle, down)`, `modes(ch, up, down)` | ein Schalter zur Modusauswahl (die Zone nach `PilotSwitches::zoneOf`) |
| `mode(ch, m)` | ein Modus obenauf, solange der Kanal ≥ `SWITCH_ON_US` ist |
| `feature(ch, f)` | eine Funktion, solange der Kanal ≥ `SWITCH_ON_US` ist |
| `knob(ch, k)` | ein Drehregler, `(us − 1500) / 500`, auf ±1 begrenzt |

`namespace BindingCheck` — rekursive `constexpr`-Funktionen (der ESP32-Core wird
als C++11 gebaut): `channelIsFree`, `channelsFree`, `channelsUnique`,
`modeSwitchCount`, `atMostOneModeSwitch`. Sie werden in den `static_assert`s von
`Controls.h` verwendet.

---

## `PilotSwitches`

**Datei:** `autopilot/PilotSwitches.h` · **Abhängig von:** `Autopilot*`, `RcChannelState`, der Zuordnungstabelle

| Methode | Beschreibung |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | eine eigene Tabelle (Tests, Simulationen) |
| `explicit PilotSwitches(Autopilot* = nullptr)` | die Tabelle `Controls::BINDINGS` |
| `void update(const RcChannelState&)` | `PilotInputs` einsammeln und an `autopilot->setInputs()` übergeben; `setMode()` — **nur wenn sich das Ergebnis der Schalter geändert hat** (ein vom Dashboard/der Bodenstation gesetzter Modus wird nicht in jedem Zyklus überschrieben). `FlightController` ruft es nur bei lebender Verbindung auf |
| `void printBindings() const` | die Belegung auf Serial beim Einschalten: `SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (up / middle / down)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`, `"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2 Zonen: < 1500 / ≥ 1500; 3 Zonen: < 1250 / < 1750 / ≥ 1750 |
| `getInputs()`, `binding(i)` | für Telemetrie und Tests |

`Bind::mode` hat Vorrang vor `Bind::modes`; von mehreren eingeschalteten
`Bind::mode` gewinnt die oberste Zeile.

---

## `Autopilot`

**Datei:** `autopilot/Autopilot.h` · **Abhängig von:** `PidController`, `Navigation`, `AltitudeSpeedController`, `LaunchController`, `SoaringController`, `AutoTrim`, den Sensoren (alle nullbar)

### Lebenszyklus

| Methode | Beschreibung |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | Der PID für Rollen/Nicken mit Kp 5, Ki 0.5, Kd 0.5, Ausgang ±500 µs |
| `bool begin()` | den Trimmwert laden; `false` und eine Meldung, wenn IMU oder Barometer fehlen |
| `void setInputs(const PilotInputs&)` | die Schalter und Drehregler dieses Zyklus (vor `update`) |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | einmal pro Zyklus: Sensoren (immer) → Navigation und Startpunkt → Speichern des Trimmwerts am Boden → Failsafe → Geofence → Modus → Kurvenkoordination → Autotrimmung |
| `ControlCommand getCommand() const` | die endgültigen Ruderkommandos (roll/pitch/yaw, µs) |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | das Gas des Modus: `PILOT` — das des Piloten; `AUTO` — das eigene; `AT_LEAST` — nicht weniger als das eigene (Automatikstart). ARM und `MOTOR_KILL` behandelt `FlightController` |

### Modi

| Methode | Beschreibung |
|---|---|
| `void setMode(AutopilotMode)` | derselbe oder ≥ `MODE_COUNT` — nichts; sonst Zurücksetzen von PID und Automaten, Ziele = der aktuelle Kurs und die aktuelle Höhe, Kreismittelpunkt = der aktuelle Punkt (mit GPS), RTH — die Rückkehrhöhe |
| `getMode()`, `getModeName()` | der Name: `FAILSAFE_GLIDE` / `FAILSAFE_RTH` bei Verbindungsverlust, sonst der Modus |
| `isFailsafeActive()`, `isFailsafeGliding()`, `isFailsafeReturning()` | Failsafe über dem Modus |
| `isAutoThrottle()`, `getThrottleCorrection()` | das Gas des Modus (%, für Log und Dashboard) |
| `getLaunchState()`, `getSoaringState()` | die Automaten LAUNCH und SOARING |

### Ausgaben und Diagnose

| Methode | Beschreibung |
|---|---|
| `getRollCorrection()`, `getPitchCorrection()`, `getYawCorrection()` | Kommando − Knüppel, µs |
| `getDesiredRoll()`, `getDesiredPitch()`, `getTargetAltitude()` | die Ziele |
| `const NavStatus& getNavStatus()` | GPS, Startpunkt, Position, Entfernung/Peilung zum Startpunkt, Kurs und Zielkurs, Geschwindigkeit für die Navigation, Geofence, Strömungsabriss |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | die barometrische Höhe (m ab dem Einschaltpunkt) |
| `getInputs()`, `getAutoTrim()` | für die Telemetrie |
| `getImuSensor()` … `getAirspeedSensor()` | die Sensoren (können `nullptr` sein) |
| `getRollPid()`, `getPitchPid()`, `setPIDGains(...)` | der PID (Dashboard, MAVLink-Parameter) |

### Interne Mechanik

- `stabilize()` — ein Winkel-PID mit D aus dem Gyroskop, multipliziert mit
  `Knob::STAB_GAIN`; der Integrator sammelt nur bei ARM und einem Fehler
  < `STAB_INTEGRATOR_ZONE_DEG`. `stabilizeOrManual()` — ohne IMU bleiben die
  Ruder beim Piloten; `stabilizeOrNeutral()` — ohne IMU Neutral (automatische
  Modi).
- `imuReady()` = die IMU existiert, ist verfügbar und hat kein Problem bei der
  Prüfung vor dem Flug.
- Die Geschwindigkeit für die Navigation: Pitotrohr → GPS → `NAV_ASSUMED_SPEED_MS`.
- `looksLanded()` — laut Barometer nahe am Boden, fast ohne Vertikalgeschwindigkeit,
  langsamer als die Schwelle des Pitotrohrs/GPS: nur dann wird der Trimmwert in
  den Flash geschrieben.
- Failsafe: mit GPS und Startpunkt — RTH mit Motor, sonst Gleiten; ein begonnener
  RTH wird wegen eines kurzen GPS-Verlusts nicht abgebrochen.

---

## `Geo`, `Guidance`, `GeoPoint`

**Datei:** `autopilot/Navigation.h`

Eine lokale „Nord/Ost“-Ebene in Metern (eine äquirektanguläre Projektion — für
Kilometer ist der Fehler ein Bruchteil eines Prozents).

| Funktion | Beschreibung |
|---|---|
| `Geo::wrap180`, `Geo::wrap360` | Normalisierung von Winkeln |
| `Geo::offsetNE(a, b, north, east)`, `distance(a, b)`, `bearing(a, b)` | Versatz, Entfernung, Peilung 0..360 |
| `Geo::moved(a, north, east)` | ein Punkt mit Versatz |
| `Geo::fromGps(GpsData)` | ein `GeoPoint` aus dem GPS |
| `Guidance::rollForCourse(target, course, bankLimit)` | die Schräglage für einen Kursfehler (`NAV_COURSE_GAIN`), begrenzt |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | der Kurs des Vektorfelds auf einen Kreis (`LOITER_CONVERGENCE`) |
| `Guidance::orbitBankDeg(speed, radius)` | die vorgesteuerte Schräglage eines Kreises: atan(V²/(g·R)) |

## `AltitudeSpeedController`

**Datei:** `autopilot/AltitudeSpeedController.h` — TECS-lite.

| Methode | Beschreibung |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | die gewünschte Vertikalgeschwindigkeit = `NAV_ALT_GAIN`·Fehler (≤ `NAV_MAX_CLIMB/SINK`); Nicken = die Vorsteuerung asin(Vz/V) + ein PI auf den Vz-Fehler, innerhalb von `NAV_MAX_CLIMB/DIVE_PITCH_DEG` |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | mit Pitotrohr — ein PI auf die Fluggeschwindigkeit um `cruisePct`; ohne — `cruisePct`; + `THROTTLE_PER_CLIMB_PCT` für den geforderten Steigflug |
| `reset()`, `getWantedClimb()` | |

## `LaunchController`

**Datei:** `autopilot/LaunchController.h`

`State`: `IDLE → READY` (Gas angehoben) `→ THROWN` (eine Überlast > `LAUNCH_ACCEL_G`
länger als `LAUNCH_ACCEL_TIME_MS`) `→ CLIMB` (nach `LAUNCH_MOTOR_DELAY_MS`: der
Motor, Nicken `LAUNCH_CLIMB_PITCH_DEG`) `→ DONE` (`LAUNCH_CLIMB_MS` oder
`LAUNCH_ALTITUDE_M`). Bewegen der Knüppel vor dem Wurf — Abbruch. Methoden:
`update(...)`, `reset()`, `getState()`, `motorOn()`, `pitchTargetDeg()`,
`stateName()`.

## `SoaringController`

**Datei:** `autopilot/SoaringController.h`

`State`: `GLIDE ⇄ THERMAL` (das Variometer > `SOAR_THERMAL_CLIMB_MS` länger als
`SOAR_THERMAL_CONFIRM_MS` / der Mittelwert < `SOAR_EXIT_CLIMB_MS` über
`SOAR_EXIT_WINDOW_MS`), `→ MOTOR_CLIMB` (unter `SOAR_MIN_ALTITUDE_M`, bis
`SOAR_MAX_ALTITUDE_M`), `→ RETURN` (jenseits von `SOAR_MAX_DISTANCE_M`, bis 70 %
davon). Methoden: `update(climb, alt, distHome, dt, now)`, `reset(now)`,
`getState()`, `motorOn()`, `getAverageClimb()`, `stateName()`.

## `AutoTrim`

**Datei:** `autopilot/AutoTrim.h` · Speicherung — `Preferences` (NVS / Flash des STM32), Namensraum `"autotrim"`

| Methode | Beschreibung |
|---|---|
| `void load()` | der Trimmwert aus dem NVS (nicht vorhanden — 0) |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += Kommando · `AUTOTRIM_RATE` · dt, bis ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | schreiben, wenn er sich geändert hat (wird von `Autopilot` nach dem DISARM am Boden aufgerufen) |
| `reset()`, `getRoll()`, `getPitch()` | |

---

## `PidController`

**Datei:** `autopilot/PidController.h` · **Abhängig von:** `Config` (das nominelle `dt`)

| Methode | Beschreibung |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`, `getKp/Ki/Kd()` | die Faktoren |
| `setLimits(minOut, maxOut)` | die Begrenzung des Ausgangs (standardmäßig ±500) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | der Ausgang, auf `[min, max]` begrenzt |
| `void reset()` | den Integrator auf Null setzen, `dt` zählt ab „jetzt“ |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (die Rate vom Sensor!)
out = constrain(P + I + D, min, max)
```

D wird auf die Rate der gemessenen Größe (Gyroskop, °/s) gebildet und nicht auf
die Ableitung des Fehlers: kein Differenzierrauschen und kein Sprung bei einer
Änderung des Sollwerts. `dt` stammt aus `micros()`; der erste Aufruf nach
`reset()` oder nach einer Pause von mehr als 0,1 s verwendet das nominelle
`LOOP_PERIOD_MS`.
