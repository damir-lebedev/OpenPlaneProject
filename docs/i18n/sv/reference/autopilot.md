# AUTOPILOT – lägen, navigering, brytare

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/autopilot.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

Autopiloten tar emot pilotens spakar, brytarna/rattarna (`PilotInputs`)
och sensorerna, och ger ut **det slutliga roderkommandot**
(`getCommand()`) och lägets gas (`applyThrottle()`). Utan en
nödvändig sensor beter sig ett läge säkert (roderytorna stannar hos
piloten eller går till neutralläge) i stället för att krascha. Vad varje läge gör för piloten finns
i [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md). Hittills har bara manuellt läge
flugits; autopiloten har provats på bänken, med tester och med
simuleringar i sluten slinga (`test/native/test_sim`).

---

## `AutopilotMode`, `Feature`, `Knob`

**Fil:** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t` (utan scope – de numeriska koderna går in i JSON
för `/api/setmode`, `/api/status` och i `MavlinkModes`):

| Värde | Kod | Kort (OLED) | Kärnan |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | roderytor = spakar |
| `MODE_STABILIZE` | 1 | STAB | spaken är roll-/tippvinkeln |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | ett startprogram som drivs av pilotens gas |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + höjd med höjdrodret |
| `MODE_ACRO` | 4 | ACRO | spaken är vinkelhastigheten |
| `MODE_CRUISE` | 5 | CRZ | kurs + höjd + automatisk gas |
| `MODE_LOITER` | 6 | LOIT | cirklar över punkten där det slogs på |
| `MODE_RTH` | 7 | RTH | hem, cirklar över hempunkten |
| `MODE_LAUNCH` | 8 | LNCH | handstart |
| `MODE_AUTO_LAND` | 9 | LAND | glidflykt + flare |
| `MODE_SOARING` | 10 | SOAR | termik utan motor |
| `MODE_RESCUE` | 11 | RESQ | vingarna plana, nosen upp, gas |
| `MODE_COUNT` | 12 | | gränsen (`setMode()` ignorerar ≥) |

`enum class Feature : uint8_t` – brytarfunktionerna: `FLAPS`, `AIRBRAKE`,
`AUTO_TRIM`, `TURN_COORDINATION`, `MOTOR_KILL`, `BEEPER`, `PAYLOAD_DROP`,
`GEOFENCE`, `HOME_RESET`, `CAMERA_STAB`, `COUNT`.

`enum class Knob : uint8_t` – rattarna: `STAB_GAIN`, `MAX_BANK`,
`CRUISE_SPEED`, `FLAPS`, `CAMERA_TILT`, `RATES`, `LOITER_RADIUS`, `COUNT`.

`namespace AutopilotNames` – `mode()`, `modeShort()` (≤ 5 tecken),
`feature()`, `knob()`: namn för loggen, OLED, panelen, MAVLink.

### `PilotInputs`

Brytarnas och rattarnas tillstånd under en cykel.

| Medlem | Beskrivning |
|---|---|
| `bool has(Feature) const` | funktionen är på |
| `float knob(Knob) const` | rattens läge −1…+1 |
| `bool isBound(Knob) const` | ratten finns i bindningstabellen |
| `float knobValue(Knob, min, default, max) const` | i enheter: mitten är `default`, ändarna är `min`/`max`; inte bunden – `default` |

---

## `Binding`, `Bind`, `BindingCheck`

**Fil:** `autopilot/ControlBinding.h` · tabellen – `config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`,
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`. Tabellraderna är
fabrikerna i `namespace Bind` (alla `constexpr`):

| Fabrik | Betydelse |
|---|---|
| `modes(ch, up, middle, down)`, `modes(ch, up, down)` | en lägesväljare (zonen enligt `PilotSwitches::zoneOf`) |
| `mode(ch, m)` | ett läge ovanpå så länge kanalen ≥ `SWITCH_ON_US` |
| `feature(ch, f)` | en funktion så länge kanalen ≥ `SWITCH_ON_US` |
| `knob(ch, k)` | en ratt, `(us − 1500) / 500`, begränsad till ±1 |

`namespace BindingCheck` – rekursiva `constexpr`-funktioner (ESP32-kärnan
byggs som C++11): `channelIsFree`, `channelsFree`, `channelsUnique`,
`modeSwitchCount`, `atMostOneModeSwitch`. De används i `static_assert`
i `Controls.h`.

---

## `PilotSwitches`

**Fil:** `autopilot/PilotSwitches.h` · **Beror på:** `Autopilot*`, `RcChannelState`, bindningstabellen

| Metod | Beskrivning |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | din egen tabell (tester, simuleringar) |
| `explicit PilotSwitches(Autopilot* = nullptr)` | tabellen `Controls::BINDINGS` |
| `void update(const RcChannelState&)` | samla in `PilotInputs`, skicka dem till `autopilot->setInputs()`; `setMode()` – **bara när brytarnas resultat har ändrats** (ett läge som satts från panelen/GCS:en skrivs inte över varje cykel). `FlightController` anropar den bara medan förbindelsen lever |
| `void printBindings() const` | fördelningen till Serial vid start: `SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (upp / mitten / ned)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`, `"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2 zoner: < 1500 / ≥ 1500; 3 zoner: < 1250 / < 1750 / ≥ 1750 |
| `getInputs()`, `binding(i)` | för telemetri och tester |

`Bind::mode` går före `Bind::modes`; av flera påslagna `Bind::mode`-poster
vinner den översta raden.

---

## `Autopilot`

**Fil:** `autopilot/Autopilot.h` · **Beror på:** `PidController`, `Navigation`, `AltitudeSpeedController`, `LaunchController`, `SoaringController`, `AutoTrim`, sensorerna (alla nullbara)

### Livscykel

| Metod | Beskrivning |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | Roll-/tipp-PID Kp 5, Ki 0,5, Kd 0,5, utdata ±500 µs |
| `bool begin()` | ladda trimmet; `false` och ett meddelande om IMU eller barometer saknas |
| `void setInputs(const PilotInputs&)` | den här cykelns brytare och rattar (före `update`) |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | en gång per cykel: sensorer (alltid) → navigering och hempunkt → sparande av trimmet på marken → failsafe → geofence → läge → koordinerade svängar → autotrimning |
| `ControlCommand getCommand() const` | de slutliga roderkommandona (roll/pitch/yaw, µs) |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | lägets gas: `PILOT` – pilotens gas; `AUTO` – sin egen; `AT_LEAST` – inte mindre än sin egen (automatisk start). ARM och `MOTOR_KILL` hanteras av `FlightController` |

### Lägen

| Metod | Beskrivning |
|---|---|
| `void setMode(AutopilotMode)` | samma eller ≥ `MODE_COUNT` – ingenting; annars nollställ PID och tillståndsmaskinerna, målen = den aktuella kursen och höjden, cirkelns mittpunkt = den aktuella punkten (med GPS), RTH – återflygningshöjden |
| `getMode()`, `getModeName()` | namnet: `FAILSAFE_GLIDE` / `FAILSAFE_RTH` vid förlorad förbindelse, annars läget |
| `isFailsafeActive()`, `isFailsafeGliding()`, `isFailsafeReturning()` | failsafe ovanpå läget |
| `isAutoThrottle()`, `getThrottleCorrection()` | lägets gas (%, för loggen och panelen) |
| `getLaunchState()`, `getSoaringState()` | tillståndsmaskinerna för LAUNCH och SOARING |

### Utdata och diagnostik

| Metod | Beskrivning |
|---|---|
| `getRollCorrection()`, `getPitchCorrection()`, `getYawCorrection()` | kommando − spakar, µs |
| `getDesiredRoll()`, `getDesiredPitch()`, `getTargetAltitude()` | målen |
| `const NavStatus& getNavStatus()` | GPS, hempunkt, position, avstånd/bäring till hempunkten, kurs och målkurs, fart för navigering, geofence, överstegring |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | den barometriska höjden (m från startpunkten) |
| `getInputs()`, `getAutoTrim()` | för telemetri |
| `getImuSensor()` … `getAirspeedSensor()` | sensorerna (kan vara `nullptr`) |
| `getRollPid()`, `getPitchPid()`, `setPIDGains(...)` | PID (panelen, MAVLink-parametrar) |

### Intern mekanik

- `stabilize()` – en vinkel-PID med D från gyrot, multiplicerad med
  `Knob::STAB_GAIN`; integratorn ackumulerar bara under ARM och när felet
  är < `STAB_INTEGRATOR_ZONE_DEG`. `stabilizeOrManual()` – utan IMU stannar
  roderytorna hos piloten; `stabilizeOrNeutral()` – utan
  IMU, neutralläge (automatiska lägen).
- `imuReady()` = IMU:n finns, är tillgänglig och har inget problem i kontrollen före flygning.
- Farten för navigering: pitotrör → GPS → `NAV_ASSUMED_SPEED_MS`.
- `looksLanded()` – nära marken enligt barometern, med nästan ingen vertikal
  hastighet, långsammare än tröskeln för pitot/GPS: bara då skrivs trimmet till
  flash.
- Failsafe: med GPS och en hempunkt – RTH med motor, annars glidflykt;
  en påbörjad RTH överges inte på grund av en kort GPS-förlust.

---

## `Geo`, `Guidance`, `GeoPoint`

**Fil:** `autopilot/Navigation.h`

Ett lokalt plan ”norr/öster” i meter (en ekvirektangulär projektion – för
kilometer är felet en bråkdel av en procent).

| Funktion | Beskrivning |
|---|---|
| `Geo::wrap180`, `Geo::wrap360` | normalisering av vinklar |
| `Geo::offsetNE(a, b, north, east)`, `distance(a, b)`, `bearing(a, b)` | förskjutning, avstånd, bäring 0..360 |
| `Geo::moved(a, north, east)` | en punkt med förskjutning |
| `Geo::fromGps(GpsData)` | en `GeoPoint` från GPS:en |
| `Guidance::rollForCourse(target, course, bankLimit)` | krängningen för ett kursfel (`NAV_COURSE_GAIN`), begränsad |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | vektorfältets kurs mot en cirkel (`LOITER_CONVERGENCE`) |
| `Guidance::orbitBankDeg(speed, radius)` | den framkopplade krängningen för en cirkel: atan(V²/(g·R)) |

## `AltitudeSpeedController`

**Fil:** `autopilot/AltitudeSpeedController.h` – TECS-lite.

| Metod | Beskrivning |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | den önskade vertikala hastigheten = `NAV_ALT_GAIN`·fel (≤ `NAV_MAX_CLIMB/SINK`); tippning = den framkopplade asin(Vz/V) + en PI på felet i Vz, inom `NAV_MAX_CLIMB/DIVE_PITCH_DEG` |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | med pitotrör – en PI på lufthastigheten kring `cruisePct`; utan – `cruisePct`; + `THROTTLE_PER_CLIMB_PCT` för den nödvändiga stigningen |
| `reset()`, `getWantedClimb()` | |

## `LaunchController`

**Fil:** `autopilot/LaunchController.h`

`State`: `IDLE → READY` (gasen höjd) `→ THROWN` (en överbelastning > `LAUNCH_ACCEL_G`
i mer än `LAUNCH_ACCEL_TIME_MS`) `→ CLIMB` (efter `LAUNCH_MOTOR_DELAY_MS`:
motorn, tippning `LAUNCH_CLIMB_PITCH_DEG`) `→ DONE` (`LAUNCH_CLIMB_MS` eller
`LAUNCH_ALTITUDE_M`). Spakrörelse före kastet – avbryt. Metoder:
`update(...)`, `reset()`, `getState()`, `motorOn()`, `pitchTargetDeg()`,
`stateName()`.

## `SoaringController`

**Fil:** `autopilot/SoaringController.h`

`State`: `GLIDE ⇄ THERMAL` (variometern > `SOAR_THERMAL_CLIMB_MS` i mer än
`SOAR_THERMAL_CONFIRM_MS` / medelvärdet < `SOAR_EXIT_CLIMB_MS` över
`SOAR_EXIT_WINDOW_MS`), `→ MOTOR_CLIMB` (under `SOAR_MIN_ALTITUDE_M`, upp till
`SOAR_MAX_ALTITUDE_M`), `→ RETURN` (längre bort än `SOAR_MAX_DISTANCE_M`, ned till 70 % av
det). Metoder: `update(climb, alt, distHome, dt, now)`, `reset(now)`,
`getState()`, `motorOn()`, `getAverageClimb()`, `stateName()`.

## `AutoTrim`

**Fil:** `autopilot/AutoTrim.h` · lagring – `Preferences` (NVS / STM32-flash), namnrymd `"autotrim"`

| Metod | Beskrivning |
|---|---|
| `void load()` | trimmet från NVS (inget – 0) |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += kommando · `AUTOTRIM_RATE` · dt, upp till ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | skriv om det har ändrats (anropas av `Autopilot` efter DISARM på marken) |
| `reset()`, `getRoll()`, `getPitch()` | |

---

## `PidController`

**Fil:** `autopilot/PidController.h` · **Beror på:** `Config` (det nominella `dt`)

| Metod | Beskrivning |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`, `getKp/Ki/Kd()` | förstärkningarna |
| `setLimits(minOut, maxOut)` | utdatans gräns (±500 som standard) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | utdatan, begränsad till `[min, max]` |
| `void reset()` | nollställ integratorn, `dt` – räknas från ”nu” |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (hastigheten från sensorn!)
out = constrain(P + I + D, min, max)
```

D tas från hastigheten hos den uppmätta storheten (gyro °/s), inte från
felets derivata: inget deriveringsbrus och inget hopp när börvärdet
ändras. `dt` kommer från `micros()`; det första anropet efter `reset()` eller efter en
paus på mer än 0,1 s använder det nominella `LOOP_PERIOD_MS`.
