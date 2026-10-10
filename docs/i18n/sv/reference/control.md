# STYRNING och KOORDINERING – mixern, gasen, ARM, utgångarna, orkestreraren

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/control.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

STYRNINGS-lagret är logik över data, utan UART, PWM eller Wi-Fi.
`FlightController` (KOORDINERING) är den enda klassen som för samman alla lägre
lager i en enda cykel.

---

## `ControlCommand`

**Fil:** `control/ControlCommand.h` · **Typ:** struktur

Ett kommando till roderytorna med **fysiska tecken**, µs utslag (±500
= fullt utslag). Det gemensamma språket för spakarna, autopiloten och mixern.

| Fält | ”+” betyder |
|---|---|
| `int16_t roll` | roll åt höger (höger skevroder upp, vänster ned) |
| `int16_t pitch` | nosen upp (höjdrodret upp) |
| `int16_t yaw` | nosen åt höger (sidroder och hjul åt höger) |
| `int16_t flaps` | klaffar ned (båda skevrodren ned); ”−” – luftbroms (båda upp) |

Alla fält är 0 som standard.

---

## `FlightOutputState`

**Fil:** `control/FlightOutputState.h` · **Typ:** struktur

De önskade utgångspulserna, PWM µs. Som standard – roderytor i neutralläge och
gas `PWM_MIN`.

| Fält | Standard |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `aux1` | `PAYLOAD_CLOSED_US` – lastsläppet är stängt |
| `aux2` | `PWM_CENTER` – kameran |

---

## `FlapsController`

**Fil:** `control/FlapsController.h` · **Beror på:** `Config`

Mjuk utfällning/infällning av klaffarna: läget rör sig mot målet
(vilket värde som helst – klaffar från brytaren, från ratten, en luftbroms uppåt) inte
snabbare än det fulla utslaget `FLAPS_DEPLOYED_US` på `FLAPS_TRANSITION_MS`.
Tiden skickas som parameter.

| Metod | Beskrivning |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | Ett steg mot målet; returnerar det aktuella läget, µs (+ ned, − upp) |
| `int16_t getPosition() const` | Det aktuella läget |

Invarianter:

- **Det första anropet** sätter läget direkt till målet – klaffarna
  ”glider” inte ut på skrivbordet vid start.
- Tidssteget är begränsat till `MAX_STEP_MS = 20`: efter en lång paus (failsafe,
  kalibrering) hoppar klaffarna inte till målet på en enda cykel.

---

## `ControlMixer`

**Fil:** `control/ControlMixer.h` · **Beror på:** `RcInput`, `RcChannelState`, `FlapsController`, `ControlCommand`, `FlightOutputState`, `Config`, `Channels`

Den aerodynamiska logiken i två steg. Äger `FlapsController`.

| Metod | Beskrivning |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll` (2000 = höger); CH2 → `pitch` **med motsatt tecken** (2000 = bort från dig = nosen ned); CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | klaffmålet väljs av `FlightController` (luftbroms → klaffbrytare → ratt), den mjuka rörelsen sker här |
| `FlightOutputState mix(const ControlCommand& c) const` | Kommando → PWM. Roll/tippning/gir begränsas av utslaget (`*_MAX_US`); skevroder: vänster = `flaps + roll`, höger = `flaps − roll` (ned = ”+”); PWM = `1500 ± utslag` med tecknet från `Config::*_REVERSED`, begränsat till 1000..2000. `throttle` fylls inte i |
| `int16_t getFlaps() const` | Klaffarnas aktuella läge, µs |

Flaperoner: vid utfällning sänks båda skevrodren med `FLAPS_DEPLOYED_US` (det nya
”neutralläget”), och roll verkar ovanpå det. Vid full roll når det nedåtgående skevrodret
slutet av sitt utslag tidigare än det uppåtgående – det fungerar som skevroder-
differential.

---

## `ThrottleManager`

**Fil:** `control/ThrottleManager.h` · **Beror på:** `RcInput`, `RcChannelState`, `Config`, `Channels`

| Metod | Beskrivning |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | Gasen från CH3, begränsad till 1000..2000; vid förlorad förbindelse – `FAILSAFE_THROTTLE` |

Den vet ingenting om ARM och autopiloten – deras korrigeringar tillämpas av
`FlightController`.

---

## `ArmingManager`

**Fil:** `control/ArmingManager.h` · **Beror på:** `Autopilot` (nullbar), `RcChannelState`, `Config`, `Channels`

ARM med en separat brytare SwA (CH5). Tillståndsmaskinen finns i
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Metod | Beskrivning |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Utan autopilot kontrolleras bara gasen |
| `void update(const RcChannelState& rc, bool receiverFailsafe)` | Vid failsafe – ingenting (brytaren i en failsafe-ram återspeglar inte piloten). Brytaren OFF → DISARM, `switchSeenOff = true`. En övergång OFF→ON → kontroller → ARM eller en vägran |
| `bool isArmed() const` | Armerat |
| `const char* getLastRefusalReason() const` | Orsaken till den senaste vägran eller `nullptr`; nollställs när brytaren slås av |

`checkFailureReason(rc)` – ARM-kontrollerna:

| Villkor | Orsak till vägran |
|---|---|
| Gas ≥ `THROTTLE_LOW_US` | ”gasen inte på minimum” |
| Vilket läge som helst utom MANUAL, IMU:n finns men svarar inte | ”IMU:n svarar inte…” |
| Vilket läge som helst utom MANUAL, IMU:n har ett problem i kontrollen före flygning | texten från `ImuSensor::getPreflightProblem()` |
| Ett läge med höjd (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), barometern finns men svarar inte | ”barometern svarar inte…” |

En sensor som saknas i bygget (`nullptr`) blockerar inte ARM; i MANUAL
armeras planet helt utan sensorer. En GPS-fix finns avsiktligt inte bland
kontrollerna: utan GPS beter sig navigeringslägena säkert (en cirkel på
stället), och hempunkten registreras när GPS:en får satelliter.

Invarianter: att starta kortet med brytaren på ON armerar inte; ett försök
per övergång OFF→ON; förlorad förbindelse nollställer inte ARM.

---

## `FlightOutputs`

**Fil:** `control/FlightOutputs.h` · **Beror på:** `IBoard`, `FlightOutputState`, `Config`

Den enda klassen som känner till uppsättningen och ordningen för PWM-utgångarna. Alla utgångar
beskrivs i en tabell; `begin()`, `write()`, statusen och självtestet
går igenom den i en slinga.

### `FlightOutputs::OutputInfo`

| Fält | Beskrivning |
|---|---|
| `const char* key` | Namnet i JSON/loggen (`aileronLeft`, …, `esc`, `rudder`, `aux1`, `aux2`) |
| `const char* label` | Det läsbara namnet |
| `int16_t pin` | Stiftnumret; `-1` – inte draget. `int16_t`, eftersom analoga stiftnummer på STM32 är `0xC0 + N` |
| `bool required` | Utan den flyger inte planet (sidrodret är valfritt) |
| `uint16_t FlightOutputState::* field` | En pekare till tillståndsfältet |

| Metod | Beskrivning |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | En tabellrad; ordningen = `ServoChannel` |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` för varje utgång, skriver ut statusen; `true` om alla **obligatoriska** fick en kanal |
| `bool isAttached(uint8_t ch) const` | Utgången är ansluten (ett index utanför intervallet → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | Utgångens värde från tillståndet, enligt tabellen |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | Den uppmätta pulsen mot den förväntade på varje draget stift; ”OK” när avvikelsen är ≤ 15 µs |
| `void write(const FlightOutputState&)` | Skriv alla utgångar och kom ihåg tillståndet |
| `void setFailsafe()` | Roderytor i neutralläge (`FAILSAFE_*`), gas `FAILSAFE_THROTTLE`; AUX som de var (lasten släpps inte vid förlorad förbindelse) |
| `void setBuzzer(bool on)` | kortets summer (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | Det senast skrivna tillståndet |

För att lägga till en utgång: en tabellrad + ett fält i `FlightOutputState` + ett index i
`ServoChannel` (+ ett stift och en LEDC-kanal i `Esp32Board`).

---

## `FlightController`

**Fil:** `control/FlightController.h` · **Lager:** KOORDINERING ·
**Beror på:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`

Styrslingans enda koordinator: den tolkar inte UART:en, rör inte
PWM:en och beräknar inte mixern själv – den anropar bara de andra i rätt
ordning. Ett detaljerat diagram finns i [ARCHITECTURE.md §6](../ARCHITECTURE.md#6-styrcykeln-flightcontrollerupdate).

| Metod | Beskrivning |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | Utan autopilot – ren manuell styrning; utan brytare – bara spakarna |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | En cykel (se nedan) |
| `bool isReceiverFailsafe() const` | Förbindelsen är förlorad |
| `const IBusReceiver& getReceiver() const` | För loggen (ramräknare, orsaken till förlusten) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | Det senast skrivna till utgångarna |
| `const RcChannelState& getRcState() const` | Kanalerna |
| `const FlightOutputs& getOutputs() const` | Utgångstabellen och `attached` |
| `int16_t getFlapsUs() const` | Klaffarnas läge |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | Den här cykelns brytare och rattar |
| `bool isLostModelBeeping() const` | ”Jag är här”-summern är igång |

Ordningen i `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. med levande förbindelse – `switches->update(rc)` (läget, funktioner, rattar);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. spakarna `mixer.fromSticks(rc)` (med levande förbindelse) × `Knob::RATES`;
   klaffarna `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → steglöst, vid förlorad förbindelse – 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` – **alltid**;
6. summern: `Beeper::update(BEEPER, armed, failsafe, now)`;
7. förbindelsen förlorad → `applyLinkLoss()` och cykeln lämnas;
8. `arming.update(rc, false)`;
9. `command = autopilot->getCommand()` (eller spakarna utan autopilot), klaffarna – sina egna;
10. `output = mixer.mix(command)`; `output.throttle = autopilot->applyThrottle(pilotThrottle)`;
11. inte armerat eller `MOTOR_KILL` → `throttle = PWM_MIN` (sist);
12. AUX1 – lasten (`PAYLOAD_DROP`), AUX2 – kameran (`Knob::CAMERA_TILT`, `CAMERA_STAB` drar ifrån tippningen);
13. `outputs.write(output)`.

`applyLinkLoss()`: om autopiloten är i failsafe (armerat: RTH eller glidflykt) –
roderytorna och gasen följer autopilotens kommando (klaffarna
fälls in mjukt, `MOTOR_KILL` tystar fortfarande motorn, AUX som de var);
annars `outputs.setFailsafe()`.

---

## `Beeper`

**Fil:** `control/Beeper.h` · **Beror på:** `Config`

| Metod | Beskrivning |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | summerns tillstånd: 2 Hz om `Feature::BEEPER` eller ”modellen borttappad” (inte armerat, ingen förbindelse på längre tid än `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | läget ”leta efter mig i gräset” |
