# AUTOPILOT / återkoppling – återkopplingsslingan (förarbete)

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/feedback.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

> ⚠️ **Förarbete, inte anslutet till firmwaren.** Varken `FlightController`,
> `Autopilot` eller `main.cpp` inkluderar de här headerfilerna. De verifieras med
> en simulering i sluten slinga (`test/test_feedback`, på en dator och på kortet) och
> med native-enhetstester. Anslutningsplanen finns i
> [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#anslutningsplan).

Idén: i stället för en PID på vinkeln med förstärkningar justerade för en enda fart – en
regulator sluten över **flygplanets respons**, med en axelmodell som lärs in
under flygning, överstegringsskydd och start-/landningsfaser som drivs av
sensorerna. Den enda indatan är `FlightSnapshot`, den enda utdatan är `FeedbackOutput`.

Alla moduler är header-only; `FeedbackModules.h` inkluderar dem på en rad.

---

## namnrymd `FeedbackConfig`

**Fil:** `autopilot/feedback/FeedbackConfig.h`

Alla slingans konstanter (de flyttas till `Config.h` vid anslutning).
Värdena märkta ”прикидка” (”grov uppskattning”) gäller en modell på ~1 kg och
1,2 m spännvidd. Arrayerna `[AXIS_COUNT]` indexeras per axel.

| Grupp | Konstanter |
|---|---|
| Allmänt | `GRAVITY = 9.80665`; axlarna `AXIS_ROLL = 0`, `AXIS_PITCH = 1`, `AXIS_YAW = 2`, `AXIS_COUNT = 3` |
| Fart | `STALL_SPEED_MS = 8`, `REFERENCE_SPEED_MS = 14`, `ACCEL_FILTER_TAU_S = 0.3` |
| I luften/på marken | `AIRBORNE_HEIGHT_M = 3`, `AIRBORNE_CONFIRM_MS = 500`, `GROUND_STILL_MS = 2000`, `GROUND_ACCEL_TOLERANCE_G = 0.1` |
| Regulator | `ANGLE_GAIN = {4, 4, 2}` 1/s, `MAX_RATE_DPS = {120, 60, 30}`, `RATE_TAU_S = {0.15, 0.20, 0.30}`, `RATE_INTEGRAL_GAIN = {2, 2, 1}`, `MAX_DEFLECTION_US = {400, 400, 400}`, `DAMPING_COMPENSATION = 0.5` |
| Roderytornas verkan | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/s²/µs, `EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`, `EFFECTIVENESS_MAX = {30, 15, 6}`, `RESPONSE_DELAY_MS = 40`, `RLS_FORGETTING = 0.995`, `ESTIMATOR_PERIOD_MS = 20`, `ESTIMATOR_PREFILTER_HZ = 2`, `MIN_EXCITATION_US = 30` |
| Överstegring | `DECEL_WARN_MS2 = 2`, `DECEL_CONFIRM_MS = 300`, `LOW_ENERGY_PITCH_DEG = 5`, `NOSE_DROP_RATE_DPS = 60`, `WING_DROP_RATE_DPS = 120`, `STALL_NOSE_UP_COMMAND_US = 50`, `LOW_EFFECTIVENESS_RATIO = 0.35`, `LOW_SPEED_MARGIN = 1.25`, `LOW_SPEED_EXIT_MARGIN = 1.5`, `LOW_ENERGY_THROTTLE_PERCENT = 80`, `LOW_ENERGY_MAX_PITCH_DEG = 5`, `STALL_THROTTLE_PERCENT = 100`, `STALL_MAX_PITCH_DEG = −5`, `STALL_MAX_BANK_DEG = 10`, `STALL_AILERON_LIMIT_US = 150`, `RECOVERY_HOLD_MS = 1000` |
| Start | `TAKEOFF_HAND_LAUNCH = false`, `TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`, `TAKEOFF_THROTTLE_PERCENT = 100`, `LAUNCH_ACCEL_G = 1`, `LAUNCH_DETECT_MS = 50`, `ROTATE_SPEED_MS = 10`, `ROTATE_FALLBACK_MS = 1500`, `CLIMB_PITCH_DEG = 12`, `TAKEOFF_TARGET_ALTITUDE_M = 30`, `TAKEOFF_CLIMB_FALLBACK_MS = 10000`, `LAUNCH_TIMEOUT_MS = 8000`, `HEADING_HOLD_GAIN = 2` |
| Landning | `APPROACH_SINK_RATE_MS = 1`, `APPROACH_THROTTLE_PERCENT = 25`, `APPROACH_BASE_PITCH_DEG = −3`, `APPROACH_MIN_PITCH_DEG = −10`, `APPROACH_MAX_BANK_DEG = 20`, `GO_AROUND_THROTTLE_PERCENT = 80`, `SINK_TO_PITCH_GAIN = 4`, `FLARE_HEIGHT_M = 2`, `FLARE_SINK_RATE_MS = 0.3`, `FLARE_MAX_PITCH_DEG = 8`, `TOUCHDOWN_ACCEL_G = 0.5`, `TOUCHDOWN_HEIGHT_M = 0.3`, `TOUCHDOWN_STILL_MS = 500`, `TOUCHDOWN_STILL_RATE_DPS = 5`, `ROLLOUT_MS = 5000` |

---

## namnrymd `FeedbackMath`

**Fil:** `autopilot/feedback/FeedbackMath.h` · **Beror på:** `<math.h>`

| Funktion | Beskrivning |
|---|---|
| `float wrap180(float deg)` | En vinkel i `(−180, 180]`: skillnaden mellan kurserna 350° och 10° är −20° |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | Begränsar till `[−limit, limit]` |

---

## `FlightSnapshot`

**Fil:** `autopilot/feedback/FlightSnapshot.h` · **Typ:** struktur

Allt som slingan vet om flygplanet under en cykel. Tecknen är
flygtekniska.

| Grupp | Fält |
|---|---|
| Tid/status | `timeUs`, `armed`, `linkLost` |
| Attityd | `imuValid`, `rollDeg`, `pitchDeg`, `yawDeg`, `rollRateDps`, `pitchRateDps`, `yawRateDps`, `accelXg/Yg/Zg` |
| Höjd | `baroValid`, `altitudeM` (från startpunkten), `climbRateMs`, `heightAglValid`, `heightAglM` (en framtida avståndsmätare) |
| Fart | `airspeedValid`, `airspeedMs` (ett framtida pitotrör), `gpsValid`, `groundSpeedMs` |
| Lägets mål | `stabilizationActive` (false = MANUAL: bara inlärning), `targetRollDeg`, `targetPitchDeg` |
| Kommandon, µs | `stick*Us` – pilotens bidrag; `command*Us` – resultatet som faktiskt gick till roderytorna |
| Gas, % | `pilotThrottlePercent`, `throttlePercent` (faktiskt till ESC:n) |
| Klaffar | `flapsUs`, `flapsMoving` |

---

## `FeedbackOutput`

**Fil:** `autopilot/feedback/FeedbackOutput.h` · **Typ:** struktur

| Fält | Beskrivning |
|---|---|
| `float deflectionUs[3]` | Roderutslag per axel, µs (tecknen från `ControlCommand`) |
| `bool axisEnabled[3]` | `false` – axeln styrs inte, roderytan stannar hos piloten |
| `float throttleOverridePercent` | En flygfas absoluta gas; `< 0` – inte satt |
| `float throttleFloorPercent` | Gasens nedre gräns (överstegringsskydd); `< 0` – ingen |
| `targetRollDeg`, `targetPitchDeg` | De slutliga målen efter begränsningarna (felsökning) |
| `const char* reason` | En kort beskrivning för loggen/OLED |

---

## `PhaseTargets`

**Fil:** `autopilot/feedback/PhaseTargets.h` · **Typ:** struktur

Den gemensamma utdatan från `TakeoffSequencer` och `LandingSequencer` – ”vad”, inte
”hur”.

| Fält | Standard | Beskrivning |
|---|---|---|
| `active` | `false` | Fasen styr nu flygplanet |
| `targetRollDeg`, `targetPitchDeg` | 0 | Målen |
| `controlRoll`, `controlPitch` | `true` | `false` – låt axeln vara (på hjulen bestäms tippningen av landningsstället) |
| `holdHeading`, `headingDeg` | `false`, 0 | Håll kursen med sidrodret och hjulet |
| `throttlePercent` | −1 | −1 – pilotens gas |
| `reason` | `""` | En beskrivning |

---

## `SpeedEstimator`

**Fil:** `autopilot/feedback/SpeedEstimator.h`

Farten (lufthastighet > GPS-markhastighet > okänd) och den longitudinella
accelerationen från IMU:n: `dV/dt = g · (ax − sin θ)` via ett lågpassfilter
`ACCEL_FILTER_TAU_S` – ”farten sjunker” syns även utan lufthastighets-
sensor.

| Metod | Beskrivning |
|---|---|
| `void update(const FlightSnapshot&)` | Ett steg; `dt ≤ 0` eller `> 0.5 s` sedan föregående anrop (för det första – sedan `timeUs = 0`) – hoppas över |
| `bool hasSpeed() const`, `float getSpeed() const`, `Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`, `float getAcceleration() const` | m/s², ”+” – ökar farten; ingen IMU – `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`, begränsad till `0.05..4`; utan fart – 1 |

---

## `AirborneDetector`

**Fil:** `autopilot/feedback/AirborneDetector.h`

Om flygplanet är i luften: inlärning, ackumulering av integralen och
sökandet efter överstegring är bara meningsfulla under flygning.

| Metod | Beskrivning |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | Inte armerat – återställ till ”på marken”. En kandidat för tillståndsbyte måste hålla i `AIRBORNE_CONFIRM_MS` (start) eller `GROUND_STILL_MS` (landning) |
| `void force(bool)` | Sätt uttryckligen (känt från start/landning) |
| `void reset()` | På marken |
| `bool isAirborne() const` | |

”Ser ut som flygning”: avståndsmätarens eller barometerns höjd > `AIRBORNE_HEIGHT_M`,
eller farten > `ROTATE_SPEED_MS`. ”Ser ut som marken”: lågt, vinkel-
hastigheterna för alla axlar < `TOUCHDOWN_STILL_RATE_DPS`, |a| ≈ 1g (± `GROUND_ACCEL_TOLERANCE_G`).

---

## `ControlEffectivenessEstimator`

**Fil:** `autopilot/feedback/ControlEffectivenessEstimator.h`

En axel. Modellen: **vinkelacceleration = b·roderyta(t − fördröjning) + a·ω + c**.
`b` är roderytans verkan (°/s² per µs, tecknet är
responsens riktning), `a` är dämpning (1/s, oftast < 0), `c` är ett
konstant moment (autotrimning). `b` lärs in vid en referensfart:
`b = b_ref · (V/V_ref)²`, `a = a_ref · V/V_ref`. Skattningen är rekursiva minsta
kvadrater med glömska (`λ = 0.995`, ett minne på ~4 s).

| Metod | Beskrivning |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | Anropar `reset()` |
| `void reset()` | θ = (prior, 0, 0); kovarians: b ± prior, a ± 5, c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | Anropa den varje cykel: ackumulerar medelvärden över ett intervall på `ESTIMATOR_PERIOD_MS`, i slutet av intervallet – accelerationen från gyroskillnaden, kommandots fördröjning, ett gemensamt lågpassfilter på båda sidor, ett RLS-steg (om inlärning är tillåten och det finns excitation) |
| `getEffectiveness()` | `b` vid den aktuella farten |
| `getReferenceEffectiveness()` | `b` vid referensfarten |
| `getDamping()` | `a` vid den aktuella farten |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b` (0 om `\|b\|` är litet) |
| `getEffectivenessSigma()` | σ för skattningen av `b` vid den aktuella farten |
| `bool isConfident() const` | ≥ 50 RLS-steg, `\|b\|` ≥ minimum och σ < 0,3·`\|b\|` |
| `getAngularAccel()` | Accelerationen över det senaste intervallet (felsökning) |

Särdrag:

- Ett intervall längre än `MAX_GAP_MS = 200` (slingan stod still) – data
  börjar om (fördröjningshistoriken och filtret nollställs).
- Den lär sig bara under **excitation**: utslaget i medelkommandona över 16
  intervall (~0,3 s) ≥ `MIN_EXCITATION_US`; annars fryser skattningen.
- Flyttalshygien efter ett steg: symmetrin hos `P`, ett tak för varianserna
  (×10 av de initiala), en gräns för `b` (`±EFFECTIVENESS_MAX`) och `a`
  (`−40..5`).

---

## `AxisModel`

**Fil:** `autopilot/feedback/AdaptiveRateController.h` · **Typ:** struktur

Det som är känt om en axels respons, för regulatorn: `effectiveness` (b,
standard 1), `damping` (a, 0 – kompensera inte), `bias` (c, 0).

---

## `AdaptiveRateController`

**Fil:** `autopilot/feedback/AdaptiveRateController.h`

En enaxlig regulator, tre steg:

```
ω* = clamp(ANGLE_GAIN · wrap180(target − angle), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
surface = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| Metod | Beskrivning |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | Integralen, mättnaden och utdatan – 0 |
| `float angleToRate(targetDeg, angleDeg) const` | Steg 1 (den kortaste vägen för kursen) |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | Steg 2–3, returnerar utslaget, µs |
| `getDesiredRate()`, `getIntegral()`, `getOutput()`, `isSaturated()` | Tillstånd |

Invarianter: `|b|` är inte mindre än `EFFECTIVENESS_MIN` (tecknet på b behålls);
integralen lagras i °/s (den förblir korrekt när `b` ändras) och **ackumuleras inte
mot gränsen** (anti-windup efter riktningen på föregående
stegs mättnad); `dt ≤ 0` – integralen ändras inte.

---

## `StallGuard`

**Fil:** `autopilot/feedback/StallGuard.h`

Skydd mot fartförlust och överstegring. Nivåerna är `Level::{Normal, LowEnergy, Stall}`.

| Metod | Beskrivning |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | På marken eller utan IMU – återställ till Normal |
| `Level getLevel() const`, `const char* getLevelName() const` | `"OK"`, `"LOW_ENERGY"`, `"STALL"` |
| `const char* getReason() const` | Det senaste tecknet som löste ut |
| `float maxPitchDeg() const` | Stall: −5°, LowEnergy: 5°, annars 90° |
| `float maxBankDeg() const` | Stall: 10°, annars 180° |
| `float maxAileronUs() const` | Stall: 150 µs, annars `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall 100 %, LowEnergy 80 %, annars/utan förbindelse −1 |
| `void reset()` | Normal |

`StallGuard::ControlState` – `pitchEffectivenessKnown`, `pitchEffectiveness`
(storleken på skattningen av b för tippning).

**LowEnergy**-tecken: en bekräftad (`DECEL_CONFIRM_MS`) inbromsning
över `DECEL_WARN_MS2` vid en tippning > 5°; fart < `1,25·Vs`; en säker verkan hos höjdrodret
< 35 % av a priori-värdet. **Stall**-tecken: fart < Vs; nosen
faller snabbare än 60 °/s med höjdrodret ”upp” > 50 µs; vid låg energi faller
vingen snabbare än 120 °/s mot skevrodren. Åtgärderna släpps
efter `RECOVERY_HOLD_MS` och bara när energin har återhämtat sig (fart ≥
`1,5·Vs`, utan fartsensor – acceleration ≥ 0).

---

## `TakeoffSequencer`

**Fil:** `autopilot/feedback/TakeoffSequencer.h`

Start i faser. `State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`.
Diagrammet finns i [ARCHITECTURE.md §7](../ARCHITECTURE.md#start-och-landning-återkopplingsslingan-inte-ansluten).

| Metod | Beskrivning |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | En aktiv fas → `Aborted`; målen nollställs |
| `void update(snapshot, speed, nowMs)` | Högst en övergång per cykel, sedan den nya fasens mål |
| `void reset()` | → `Idle` |
| `getTargets()`, `getState()`, `getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

Fasernas mål: väntan – gas 0, roderytorna hos piloten;
rullningen – gas 100 %, vingarna plana, tippningen lämnas i fred, håll kursen
som fångades vid startögonblicket; stigningen – gas 100 %, vingarna plana,
tippning `CLIMB_PITCH_DEG`. Ett handkast: den longitudinella accelerationen
`ax − sin θ ≥ LAUNCH_ACCEL_G` i mer än `LAUNCH_DETECT_MS`.

---

## `LandingSequencer`

**Fil:** `autopilot/feedback/LandingSequencer.h`

Landning i faser. `State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`.

| Metod | Beskrivning |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`, `void reset()`, `update(snapshot, nowMs)` | Som för start |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout – överstegringsskyddet är avstängt här |
| `bool isOnGround() const` | Rollout / Complete |

Tippningen under sjunkningen och flaren kommer från felet i vertikal hastighet:
`θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`, begränsad till
`[min, FLARE_MAX_PITCH_DEG]`; utan barometer – grundvinkeln. Höjden
kommer från avståndsmätaren, annars barometern. Sättning: en spik i
|a − 1g| ≥ 0,5g eller ”lågt och roterar inte” i `TOUCHDOWN_STILL_MS`. Under
utrullningen fångas kursen vid sättningsögonblicket.

---

## `FeedbackSupervisor`

**Fil:** `autopilot/feedback/FeedbackSupervisor.h`

Slingan som helhet. Äger `SpeedEstimator`, `AirborneDetector`, tre
`ControlEffectivenessEstimator`, tre `AdaptiveRateController`, `StallGuard`,
`TakeoffSequencer` och `LandingSequencer`.

| Metod | Beskrivning |
|---|---|
| `bool requestTakeoff()` | Bara armerat, med förbindelse, på marken; avbryter en landning |
| `bool requestLanding()` | Bara armerat, med förbindelse, i luften; avbryter en start |
| `void cancelPhase()` | Avbryt fasen |
| `const FeedbackOutput& update(const FlightSnapshot&)` | En cykel (ordningen finns i [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-återkopplingsslingan-inte-ansluten)) |
| `getOutput()`, `isAirborne()`, `getSpeedEstimator()`, `getEstimator(axis)`, `getController(axis)`, `getStallGuard()`, `getTakeoff()`, `getLanding()` | Tillstånd för loggen och testerna |
| `void printStatus(Print& out) const` | En statusrad + en rad per axel (`b ± σ`, `*` – säker, `a`, `c`, `I`, utdatan) |

Viktiga regler:

- **Inte armerat** – alla axlar är av, `reason = "not armed"`; ARM/DISARM (en
  ny flygning) nollställer allt inlärt.
- **Förlorad förbindelse** avbryter faserna; gasen rörs inte (firmwarens
  failsafe verkar).
- **Lättning** nollställer skattningarna och regulatorerna (det som ”sågs” på
  hjulen duger inte).
- En negativ säker skattning av `b` går **aldrig** in i regulatorn –
  axeln arbetar med a priori-modellen, och `reason` bär varningen ”…
  reagerar bakvänt på roderytan? kontrollera på marken”.
- Integralen är fryst på marken, utom för kursen under
  startrullningen/utrullningen.
- Koordinerad sväng (med känd fart i luften): till den önskade tipphastigheten
  `+ g/V · sin φ · tg φ`, till girhastigheten `g/V · sin φ` (krängningen begränsas till ±60°).
- Prioriteten för `reason`: överstegring > låg energi > fas > teckenvarningen >
  ”stabilisering”/”manuellt (inlärning)”.
