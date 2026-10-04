# AUTOPILOT / feedback — die Rückkopplungsschleife (Vorarbeit)

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/feedback.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert.

[← Referenz](README.md)

> ⚠️ **Vorarbeit, nicht in die Firmware eingebunden.** Weder `FlightController`
> noch `Autopilot` noch `main.cpp` binden diese Header ein. Geprüft wird sie
> durch eine Closed-Loop-Simulation (`test/test_feedback`, auf dem PC und auf
> dem Board) und durch native Unit-Tests. Der Plan für die Einbindung steht in
> [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#anschlussplan).

Die Idee: Statt eines Winkel-PIDs mit Verstärkungen, die nur für eine
Geschwindigkeit eingestellt sind, ein Regler, der auf die **Reaktion des
Flugzeugs** zurückgekoppelt ist — mit einem im Flug gelernten Achsenmodell,
einem Schutz vor dem Strömungsabriss und sensorgesteuerten Start- und
Landephasen. Der einzige Eingang ist `FlightSnapshot`, der einzige Ausgang ist
`FeedbackOutput`.

Alle Module bestehen nur aus Headern; `FeedbackModules.h` bindet sie in einer
Zeile ein.

---

## namespace `FeedbackConfig`

**Datei:** `autopilot/feedback/FeedbackConfig.h`

Alle Konstanten der Schleife (bei der Einbindung wandern sie nach `Config.h`).
Die mit „прикидка“ („grobe Schätzung“) gekennzeichneten Werte gelten für ein Modell von etwa 1 kg und 1,2 m
Spannweite. Die Arrays `[AXIS_COUNT]` werden über die Achse indiziert.

| Gruppe | Konstanten |
|---|---|
| Allgemein | `GRAVITY = 9.80665`; die Achsen `AXIS_ROLL = 0`, `AXIS_PITCH = 1`, `AXIS_YAW = 2`, `AXIS_COUNT = 3` |
| Geschwindigkeit | `STALL_SPEED_MS = 8`, `REFERENCE_SPEED_MS = 14`, `ACCEL_FILTER_TAU_S = 0.3` |
| In der Luft/am Boden | `AIRBORNE_HEIGHT_M = 3`, `AIRBORNE_CONFIRM_MS = 500`, `GROUND_STILL_MS = 2000`, `GROUND_ACCEL_TOLERANCE_G = 0.1` |
| Regler | `ANGLE_GAIN = {4, 4, 2}` 1/s, `MAX_RATE_DPS = {120, 60, 30}`, `RATE_TAU_S = {0.15, 0.20, 0.30}`, `RATE_INTEGRAL_GAIN = {2, 2, 1}`, `MAX_DEFLECTION_US = {400, 400, 400}`, `DAMPING_COMPENSATION = 0.5` |
| Ruderwirksamkeit | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs, `EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`, `EFFECTIVENESS_MAX = {30, 15, 6}`, `RESPONSE_DELAY_MS = 40`, `RLS_FORGETTING = 0.995`, `ESTIMATOR_PERIOD_MS = 20`, `ESTIMATOR_PREFILTER_HZ = 2`, `MIN_EXCITATION_US = 30` |
| Strömungsabriss | `DECEL_WARN_MS2 = 2`, `DECEL_CONFIRM_MS = 300`, `LOW_ENERGY_PITCH_DEG = 5`, `NOSE_DROP_RATE_DPS = 60`, `WING_DROP_RATE_DPS = 120`, `STALL_NOSE_UP_COMMAND_US = 50`, `LOW_EFFECTIVENESS_RATIO = 0.35`, `LOW_SPEED_MARGIN = 1.25`, `LOW_SPEED_EXIT_MARGIN = 1.5`, `LOW_ENERGY_THROTTLE_PERCENT = 80`, `LOW_ENERGY_MAX_PITCH_DEG = 5`, `STALL_THROTTLE_PERCENT = 100`, `STALL_MAX_PITCH_DEG = −5`, `STALL_MAX_BANK_DEG = 10`, `STALL_AILERON_LIMIT_US = 150`, `RECOVERY_HOLD_MS = 1000` |
| Start | `TAKEOFF_HAND_LAUNCH = false`, `TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`, `TAKEOFF_THROTTLE_PERCENT = 100`, `LAUNCH_ACCEL_G = 1`, `LAUNCH_DETECT_MS = 50`, `ROTATE_SPEED_MS = 10`, `ROTATE_FALLBACK_MS = 1500`, `CLIMB_PITCH_DEG = 12`, `TAKEOFF_TARGET_ALTITUDE_M = 30`, `TAKEOFF_CLIMB_FALLBACK_MS = 10000`, `LAUNCH_TIMEOUT_MS = 8000`, `HEADING_HOLD_GAIN = 2` |
| Landung | `APPROACH_SINK_RATE_MS = 1`, `APPROACH_THROTTLE_PERCENT = 25`, `APPROACH_BASE_PITCH_DEG = −3`, `APPROACH_MIN_PITCH_DEG = −10`, `APPROACH_MAX_BANK_DEG = 20`, `GO_AROUND_THROTTLE_PERCENT = 80`, `SINK_TO_PITCH_GAIN = 4`, `FLARE_HEIGHT_M = 2`, `FLARE_SINK_RATE_MS = 0.3`, `FLARE_MAX_PITCH_DEG = 8`, `TOUCHDOWN_ACCEL_G = 0.5`, `TOUCHDOWN_HEIGHT_M = 0.3`, `TOUCHDOWN_STILL_MS = 500`, `TOUCHDOWN_STILL_RATE_DPS = 5`, `ROLLOUT_MS = 5000` |

---

## namespace `FeedbackMath`

**Datei:** `autopilot/feedback/FeedbackMath.h` · **Abhängig von:** `<math.h>`

| Funktion | Beschreibung |
|---|---|
| `float wrap180(float deg)` | Ein Winkel in `(−180, 180]`: Die Differenz der Steuerkurse 350° und 10° ist −20° |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | Begrenzt auf `[−limit, limit]` |

---

## `FlightSnapshot`

**Datei:** `autopilot/feedback/FlightSnapshot.h` · **Art:** struct

Alles, was die Schleife in einem Zyklus über das Flugzeug weiß. Die
Vorzeichen sind luftfahrtüblich.

| Gruppe | Felder |
|---|---|
| Zeit/Zustand | `timeUs`, `armed`, `linkLost` |
| Lage | `imuValid`, `rollDeg`, `pitchDeg`, `yawDeg`, `rollRateDps`, `pitchRateDps`, `yawRateDps`, `accelXg/Yg/Zg` |
| Höhe | `baroValid`, `altitudeM` (ab dem Einschaltpunkt), `climbRateMs`, `heightAglValid`, `heightAglM` (ein künftiger Entfernungsmesser) |
| Geschwindigkeit | `airspeedValid`, `airspeedMs` (ein künftiges Pitotrohr), `gpsValid`, `groundSpeedMs` |
| Ziele des Modus | `stabilizationActive` (false = MANUAL: nur Lernen), `targetRollDeg`, `targetPitchDeg` |
| Befehle, µs | `stick*Us` — die Vorgabe des Piloten; `command*Us` — das, was tatsächlich an die Ruder geht |
| Gas, % | `pilotThrottlePercent`, `throttlePercent` (tatsächlich zum ESC) |
| Klappen | `flapsUs`, `flapsMoving` |

---

## `FeedbackOutput`

**Datei:** `autopilot/feedback/FeedbackOutput.h` · **Art:** struct

| Feld | Beschreibung |
|---|---|
| `float deflectionUs[3]` | Ruderausschläge je Achse, µs (die Vorzeichen von `ControlCommand`) |
| `bool axisEnabled[3]` | `false` — die Achse wird nicht geregelt, das Ruder bleibt beim Piloten |
| `float throttleOverridePercent` | Das absolute Gas einer Flugphase; `< 0` — nicht gesetzt |
| `float throttleFloorPercent` | Die Untergrenze des Gases (Schutz vor dem Strömungsabriss); `< 0` — keine |
| `targetRollDeg`, `targetPitchDeg` | Die endgültigen Ziele nach den Begrenzungen (zum Debuggen) |
| `const char* reason` | Eine kurze Beschreibung für das Protokoll/das OLED |

---

## `PhaseTargets`

**Datei:** `autopilot/feedback/PhaseTargets.h` · **Art:** struct

Die gemeinsame Ausgabe von `TakeoffSequencer` und `LandingSequencer` — das
„Was“, nicht das „Wie“.

| Feld | Standard | Beschreibung |
|---|---|---|
| `active` | `false` | Die Phase steuert das Flugzeug gerade |
| `targetRollDeg`, `targetPitchDeg` | 0 | Die Ziele |
| `controlRoll`, `controlPitch` | `true` | `false` — die Achse nicht anfassen (auf Rädern gibt das Fahrwerk den Nickwinkel vor) |
| `holdHeading`, `headingDeg` | `false`, 0 | Den Kurs mit Seitenruder und Spornrad halten |
| `throttlePercent` | −1 | −1 — das Gas des Piloten |
| `reason` | `""` | Eine Beschreibung |

---

## `SpeedEstimator`

**Datei:** `autopilot/feedback/SpeedEstimator.h`

Die Geschwindigkeit (Luft > Grundgeschwindigkeit des GPS > unbekannt) und die
Längsbeschleunigung aus der IMU: `dV/dt = g · (ax − sin θ)` durch einen
Tiefpass `ACCEL_FILTER_TAU_S` — so sieht man, dass „die Geschwindigkeit sinkt“,
auch ohne Geschwindigkeitssensor.

| Methode | Beschreibung |
|---|---|
| `void update(const FlightSnapshot&)` | Ein Schritt; `dt ≤ 0` oder `> 0.5 s` seit dem vorigen Aufruf (beim ersten — seit `timeUs = 0`) — wird übersprungen |
| `bool hasSpeed() const`, `float getSpeed() const`, `Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`, `float getAcceleration() const` | m/s², „+“ — Beschleunigung; ohne IMU — `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`, begrenzt auf `0.05..4`; ohne Geschwindigkeit — 1 |

---

## `AirborneDetector`

**Datei:** `autopilot/feedback/AirborneDetector.h`

Ob das Flugzeug in der Luft ist: Lernen, das Integral aufbauen und nach dem
Strömungsabriss Ausschau halten ergibt nur im Flug Sinn.

| Methode | Beschreibung |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | Nicht scharf — Rücksetzen auf „am Boden“. Ein Kandidat für einen Zustandswechsel muss `AIRBORNE_CONFIRM_MS` (Abheben) oder `GROUND_STILL_MS` (Landung) lang bestehen bleiben |
| `void force(bool)` | Ausdrücklich setzen (Start und Landung wissen es) |
| `void reset()` | Am Boden |
| `bool isAirborne() const` | |

„Sieht nach Flug aus“: die Höhe über dem Entfernungsmesser oder dem Barometer > `AIRBORNE_HEIGHT_M`
oder die Geschwindigkeit > `ROTATE_SPEED_MS`. „Sieht nach Boden aus“: niedrig, die
Drehraten aller Achsen < `TOUCHDOWN_STILL_RATE_DPS`, |a| ≈ 1g (± `GROUND_ACCEL_TOLERANCE_G`).

---

## `ControlEffectivenessEstimator`

**Datei:** `autopilot/feedback/ControlEffectivenessEstimator.h`

Eine Achse. Das Modell: **Winkelbeschleunigung = b·Ruder(t − Verzögerung) + a·ω + c**.
`b` ist die Ruderwirksamkeit (°/s² pro µs, das Vorzeichen ist die Richtung der
Reaktion), `a` ist die Dämpfung (1/s, in der Regel < 0), `c` ist ein
konstantes Moment (Auto-Trimmung). `b` wird bei einer Referenzgeschwindigkeit
gelernt: `b = b_ref · (V/V_ref)²`, `a = a_ref · V/V_ref`. Die Schätzung erfolgt
nach der Methode der rekursiven kleinsten Quadrate mit Vergessen (`λ = 0.995`,
Gedächtnis von etwa 4 s).

| Methode | Beschreibung |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | Ruft `reset()` auf |
| `void reset()` | θ = (prior, 0, 0); Kovarianz: b ± prior, a ± 5, c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | In jedem Zyklus aufrufen: sammelt Mittelwerte über ein Intervall von `ESTIMATOR_PERIOD_MS`; am Ende des Intervalls — die Beschleunigung aus der Gyro-Differenz, die Verzögerung des Befehls, ein gemeinsamer Tiefpass für beide Seiten und ein RLS-Schritt (wenn das Lernen erlaubt ist und es Anregung gibt) |
| `getEffectiveness()` | `b` bei der aktuellen Geschwindigkeit |
| `getReferenceEffectiveness()` | `b` bei der Referenzgeschwindigkeit |
| `getDamping()` | `a` bei der aktuellen Geschwindigkeit |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b` (0, wenn `\|b\|` klein ist) |
| `getEffectivenessSigma()` | σ der Schätzung von `b` bei der aktuellen Geschwindigkeit |
| `bool isConfident() const` | ≥ 50 RLS-Schritte, `\|b\|` ≥ dem Minimum und σ < 0.3·`\|b\|` |
| `getAngularAccel()` | Die Beschleunigung des letzten Intervalls (zum Debuggen) |

Besonderheiten:

- Ein Intervall, das länger ist als `MAX_GAP_MS = 200` (die Schleife stand still),
  setzt die Daten auf null (die Verzögerungshistorie und der Filter werden
  zurückgesetzt).
- Gelernt wird nur unter **Anregung**: die Amplitude der über 16 Intervalle
  (~0,3 s) gemittelten Befehle ≥ `MIN_EXCITATION_US`; sonst friert die
  Schätzung ein.
- Float-Hygiene nach einem Schritt: die Symmetrie von `P`, eine Obergrenze für
  die Varianzen (das 10-Fache der Anfangswerte), eine Begrenzung von `b`
  (`±EFFECTIVENESS_MAX`) und von `a` (`−40..5`).

---

## `AxisModel`

**Datei:** `autopilot/feedback/AdaptiveRateController.h` · **Art:** struct

Was über die Reaktion einer Achse bekannt ist, für den Regler: `effectiveness`
(b, Standard 1), `damping` (a, 0 — nicht kompensieren), `bias` (c, 0).

---

## `AdaptiveRateController`

**Datei:** `autopilot/feedback/AdaptiveRateController.h`

Ein Einachsregler mit drei Stufen:

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| Methode | Beschreibung |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | Das Integral, die Sättigung und der Ausgang — 0 |
| `float angleToRate(targetDeg, angleDeg) const` | Stufe 1 (der kürzeste Weg beim Kurs) |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | Stufen 2–3, liefert den Ausschlag, µs |
| `getDesiredRate()`, `getIntegral()`, `getOutput()`, `isSaturated()` | Zustand |

Invarianten: `|b|` ist nicht kleiner als `EFFECTIVENESS_MIN` (das Vorzeichen von
b bleibt erhalten); das Integral wird in °/s gehalten (es bleibt richtig, wenn
sich `b` ändert) und **wächst nicht in den Anschlag hinein** (Anti-Windup nach
der Richtung der Sättigung des vorigen Schritts); bei `dt ≤ 0` ändert sich das
Integral nicht.

---

## `StallGuard`

**Datei:** `autopilot/feedback/StallGuard.h`

Schutz vor Geschwindigkeitsverlust und Strömungsabriss. Die Stufen sind `Level::{Normal, LowEnergy, Stall}`.

| Methode | Beschreibung |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | Am Boden oder ohne IMU — Rücksetzen auf Normal |
| `Level getLevel() const`, `const char* getLevelName() const` | `"OK"`, `"LOW_ENERGY"`, `"STALL"` |
| `const char* getReason() const` | Das letzte Anzeichen, das ausgelöst hat |
| `float maxPitchDeg() const` | Stall: −5°, LowEnergy: 5°, sonst 90° |
| `float maxBankDeg() const` | Stall: 10°, sonst 180° |
| `float maxAileronUs() const` | Stall: 150 µs, sonst `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall 100 %, LowEnergy 80 %, sonst/ohne Verbindung −1 |
| `void reset()` | Normal |

`StallGuard::ControlState` — `pitchEffectivenessKnown`, `pitchEffectiveness`
(der Betrag der Schätzung von b beim Nicken).

Anzeichen für **LowEnergy**: eine bestätigte Verzögerung (`DECEL_CONFIRM_MS`)
über `DECEL_WARN_MS2` bei einem Nickwinkel > 5°; Geschwindigkeit < `1.25·Vs`; eine
verlässliche Wirksamkeit des Höhenruders < 35 % der a-priori-Wirksamkeit.
Anzeichen für **Stall**: Geschwindigkeit < Vs; die Nase sinkt schneller als
60 °/s bei einem Höhenruder „nach oben“ > 50 µs; bei Energiemangel kippt die
Fläche schneller als 120 °/s gegen die Querruder. Die Maßnahmen werden nach
`RECOVERY_HOLD_MS` aufgehoben und nur dann, wenn die Energie wiederhergestellt
ist (Geschwindigkeit ≥ `1.5·Vs`, ohne Geschwindigkeitssensor — Beschleunigung ≥ 0).

---

## `TakeoffSequencer`

**Datei:** `autopilot/feedback/TakeoffSequencer.h`

Der Start in Phasen. `State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`.
Das Schema steht in [ARCHITECTURE.md §7](../ARCHITECTURE.md#start-und-landung-rückführungskreis-nicht-angeschlossen).

| Methode | Beschreibung |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | Eine aktive Phase → `Aborted`; die Ziele werden zurückgesetzt |
| `void update(snapshot, speed, nowMs)` | Höchstens ein Übergang pro Zyklus, danach die Ziele der neuen Phase |
| `void reset()` | → `Idle` |
| `getTargets()`, `getState()`, `getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

Die Ziele der Phasen: Warten — Gas 0, Ruder beim Piloten; Startlauf — Gas
100 %, Flügel waagerecht, den Nickwinkel nicht anfassen, den im Moment des
Startens festgehaltenen Kurs halten; Steigflug — Gas 100 %, Flügel waagerecht,
Nickwinkel `CLIMB_PITCH_DEG`. Handstart: die Längsbeschleunigung
`ax − sin θ ≥ LAUNCH_ACCEL_G` länger als `LAUNCH_DETECT_MS`.

---

## `LandingSequencer`

**Datei:** `autopilot/feedback/LandingSequencer.h`

Die Landung in Phasen. `State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`.

| Methode | Beschreibung |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`, `void reset()`, `update(snapshot, nowMs)` | Wie beim Start |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout — der Strömungsabriss-Schutz ist hier abgeschaltet |
| `bool isOnGround() const` | Rollout / Complete |

Der Nickwinkel im Sinkflug und beim Abfangen ergibt sich aus dem Fehler der
Vertikalgeschwindigkeit: `θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`,
begrenzt auf `[min, FLARE_MAX_PITCH_DEG]`; ohne Barometer — der Basiswinkel. Die
Höhe kommt vom Entfernungsmesser, sonst vom Barometer. Das Aufsetzen: ein
Ausschlag von |a − 1g| ≥ 0.5g oder „niedrig und dreht nicht“ über
`TOUCHDOWN_STILL_MS`. Beim Ausrollen wird der Kurs im Moment des Aufsetzens
festgehalten.

---

## `FeedbackSupervisor`

**Datei:** `autopilot/feedback/FeedbackSupervisor.h`

Die ganze Schleife. Besitzt den `SpeedEstimator`, den `AirborneDetector`, drei
`ControlEffectivenessEstimator`, drei `AdaptiveRateController`, den `StallGuard`,
den `TakeoffSequencer` und den `LandingSequencer`.

| Methode | Beschreibung |
|---|---|
| `bool requestTakeoff()` | Nur scharf, mit Verbindung, am Boden; bricht eine Landung ab |
| `bool requestLanding()` | Nur scharf, mit Verbindung, in der Luft; bricht einen Start ab |
| `void cancelPhase()` | Die Phase abbrechen |
| `const FeedbackOutput& update(const FlightSnapshot&)` | Ein Zyklus (die Reihenfolge steht in [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-der-rückführungskreis-nicht-angeschlossen)) |
| `getOutput()`, `isAirborne()`, `getSpeedEstimator()`, `getEstimator(axis)`, `getController(axis)`, `getStallGuard()`, `getTakeoff()`, `getLanding()` | Zustand für das Protokoll und die Tests |
| `void printStatus(Print& out) const` | Eine Statuszeile + eine Zeile je Achse (`b ± σ`, `*` — verlässlich, `a`, `c`, `I`, der Ausgang) |

Die wichtigsten Regeln:

- **Nicht scharf** — alle Achsen sind deaktiviert, `reason = "nicht scharf"`;
  ARM/DISARM (ein neuer Flug) löscht alles Gelernte.
- **Verbindungsverlust** bricht die Phasen ab; das Gas wird nicht angefasst
  (das übernimmt der Failsafe der Firmware).
- **Das Abheben vom Boden** setzt die Schätzungen und die Regler zurück (was
  auf den Rädern „gesehen“ wurde, passt nicht).
- Eine verlässliche, negative Schätzung von `b` gelangt **nie** in den Regler —
  die Achse arbeitet mit dem a-priori-Modell, und `reason` trägt die Warnung
  „… reagiert verkehrt herum aufs Ruder? am Boden prüfen“.
- Das Integral ist am Boden eingefroren, außer dem Kurs beim Startlauf und beim
  Ausrollen.
- Koordinierter Kurvenflug (bei bekannter Geschwindigkeit in der Luft): zur
  gewünschten Nickrate kommt `+ g/V · sin φ · tg φ` hinzu, zur Gierrate
  `g/V · sin φ` (die Querneigung ist auf ±60° begrenzt).
- Die Priorität von `reason`: Strömungsabriss > Energiemangel > Phase >
  Vorzeichenwarnung > „Stabilisierung“/„manuell (Lernen)“.
