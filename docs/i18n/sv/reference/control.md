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

`throttle` är gasens storlek (1000..2000 = 0..100 %) åt vilket håll som helst; `FlightOutputs` gör om den, tillsammans med `motorReverse`, till ESC-pulsen (via `EscSignal`).

| Fält | Standard |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `bool motorReverse` | `false` – motorn går framåt |
| `bool escPulsesOff` | `false`; `true` – inga pulser alls till ESC:n (motorsignal, bara utan ARM) |
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
| `FlightOutputState mix(const ControlCommand& c) const` | Kommando → PWM. Roll/tippning/gir begränsas av utslaget (`*_MAX_US`); skevroder: vänster = `flaps + roll`, höger = `flaps − roll` (ned = ”+”); PWM = `1500 ± utslag` med tecknet från `Config::*_REVERSED`, klaffarnas tecken – `FLAPS_REVERSED`, begränsat till 1000..2000. `throttle` fylls inte i |
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

## `EscSignal`

**Fil:** `control/EscSignal.h` · **Typ:** namespace · **Beror på:** `Config`

Gas (1000..2000 = 0..100 %) och riktning → pulsen på ESC-stiftet.

| Funktion | Beskrivning |
|---|---|
| `uint16_t pulseUs(uint16_t throttleUs, bool reverse)` | `ESC_BIDIRECTIONAL` (BLHeli 80A i 3D): 0 % → `ESC_NEUTRAL_US` (1500), framåt 1500..2000, bakåt 1500..1000. Enkelriktad ESC: 1000..2000 som det är, `reverse` ignoreras. Gas utanför 1000..2000 kapas |
| `uint16_t stopUs()` | Pulsen ”motorn står” (1500 för dubbelriktad ESC, 1000 för en vanlig) |

Bara omräkning, inga skydd: när riktningen får bytas avgör `MotorReverse`, och gasgränsen bakåt sätter `FlightController`.

---

## `MotorReverse`

**Fil:** `control/MotorReverse.h` · **Beror på:** `Config`

Motorns riktning enligt brytaren `Feature::MOTOR_REVERSE` (SwD som standard) – **bara utan ARM**.

| Metod | Beskrivning |
|---|---|
| `void update(bool requested, bool armed, uint32_t nowMs)` | Utan ARM – riktningen följer brytaren direkt (i Serial `MotorReverse: мотор НАЗАД` / `вперёд` – ”motor BAKÅT” / ”framåt”). I ARM – brytaren verkar inte, ”реверс НЕ переключён” (”reversering INTE omkopplad”) skrivs ut en gång, och den nya riktningen väntar på DISARM. Med `ESC_BIDIRECTIONAL = false` slås reversering aldrig på |
| `bool isReversed() const` | Motorn går bakåt |
| `bool isPending() const` | Brytaren står redan i det andra läget; riktningen väntar på DISARM |
| `const char* armBlockReason(uint32_t nowMs) const` | Skäl att neka ARM: sedan det senaste riktningsbytet har mindre än `REVERSE_SETTLE_MS` gått (motorn kan fortfarande snurra); annars `nullptr` |

---

## `ArmingManager`

**Fil:** `control/ArmingManager.h` · **Beror på:** `Autopilot` (nullbar), `RcChannelState`, `Config`, `Channels`

ARM med en separat brytare SwA (CH5). Tillståndsmaskinen finns i
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Metod | Beskrivning |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Utan autopilot kontrolleras bara gasen |
| `void update(const RcChannelState& rc, bool receiverFailsafe, const char* blockReason = nullptr)` | Vid failsafe – ingenting (brytaren i en failsafe-ram återspeglar inte piloten). Brytaren OFF → DISARM, `switchSeenOff = true`. En övergång OFF→ON → kontroller → ARM eller en vägran `blockReason` är ett externt skäl att neka (från `FlightController`: `MotorReverse::armBlockReason()`) |
| `bool isArmed() const` | Armerat |
| `const char* getLastRefusalReason() const` | Orsaken till den senaste vägran eller `nullptr`; nollställs när brytaren slås av |

`checkFailureReason(rc)` – ARM-kontrollerna:

| Villkor | Orsak till vägran |
|---|---|
| Gas ≥ `THROTTLE_LOW_US` | ”gasen inte på minimum” |
| Vilket läge som helst utom MANUAL, IMU:n finns men svarar inte | ”IMU:n svarar inte…” |
| Vilket läge som helst utom MANUAL, IMU:n har ett problem i kontrollen före flygning | texten från `ImuSensor::getPreflightProblem()` |
| Ett läge med höjd (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), barometern finns men svarar inte | ”barometern svarar inte…” |
| Allt annat godkänt, men `blockReason` skickades | `blockReason` (t.ex. ”motorns riktning har just bytts …”) |

En sensor som saknas i bygget (`nullptr`) blockerar inte ARM; i MANUAL
armeras planet helt utan sensorer. En GPS-fix finns avsiktligt inte bland
kontrollerna: utan GPS beter sig navigeringslägena säkert (en cirkel på
stället), och hempunkten registreras när GPS:en får satelliter.

Invarianter: att starta kortet med brytaren på ON armerar inte; ett försök
per övergång OFF→ON; förlorad förbindelse nollställer inte ARM.

---

## `FlightOutputs`

**Fil:** `control/FlightOutputs.h` · **Beror på:** `IBoard`, `FlightOutputState`, `EscSignal`, `Config`

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
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` för varje utgång, skriver ut status och ESC-typ; `true` om alla **obligatoriska** fick en kanal |
| `bool isAttached(uint8_t ch) const` | Utgången är ansluten (ett index utanför intervallet → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | Pulsen på utgångens stift: fältet enligt tabellen, för ESC – via `EscSignal::pulseUs(throttle, motorReverse)`, med `escPulsesOff` – 0 |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | Den uppmätta pulsen mot den förväntade på varje draget stift; ”OK” när avvikelsen är ≤ 15 µs |
| `void write(const FlightOutputState&)` | Skriv alla utgångar och kom ihåg tillståndet; ESC med `escPulsesOff` – `IServoOutput::stopPulses()` |
| `void setFailsafe()` | Roderytor i neutral (`FAILSAFE_*`), gas `FAILSAFE_THROTTLE` (1500 på stiftet till en dubbelriktad ESC); AUX som de var (lasten släpps inte vid förlorad förbindelse) |
| `void setBuzzer(bool on)` | kortets summer (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | Det senast skrivna tillståndet |

För att lägga till en utgång: en tabellrad + ett fält i `FlightOutputState` + ett index i
`ServoChannel` (+ ett stift och en LEDC-kanal i `Esp32Board`).

---

## `FlightController`

**Fil:** `control/FlightController.h` · **Lager:** KOORDINERING ·
**Beror på:** `RcReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`, `MotorReverse`, `GroundCalibration`, `SignalPlayer`

Styrslingans enda koordinator: den tolkar inte UART:en, rör inte
PWM:en och beräknar inte mixern själv – den anropar bara de andra i rätt
ordning. Ett detaljerat diagram finns i [ARCHITECTURE.md §6](../ARCHITECTURE.md#6-styrcykeln-flightcontrollerupdate).

| Metod | Beskrivning |
|---|---|
| `FlightController(RcReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | Utan autopilot – ren manuell styrning; utan brytare – bara spakarna |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | En cykel (se nedan) |
| `bool isReceiverFailsafe() const` | Förbindelsen är förlorad |
| `const RcReceiver& getReceiver() const` | För loggen (ramräknare, orsaken till förlusten) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | Det senast skrivna till utgångarna |
| `const RcChannelState& getRcState() const` | Kanalerna |
| `const FlightOutputs& getOutputs() const` | Utgångstabellen och `attached` |
| `int16_t getFlapsUs() const` | Klaffarnas läge |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | Den här cykelns brytare och rattar |
| `bool isLostModelBeeping() const` | ”Jag är här”-summern är igång |
| `const MotorReverse& getMotorReverse() const` | Motorns riktning och väntande reversering |
| `void toggleGroundCalibration()` | Konsolen `c`: markkalibrering av flygplanet – starta (bara utan ARM), pågår den – avbryt |
| `void playSignal(uint8_t count)` | Konsolen `e`: en signal (utan ARM – även med motorn) |
| `const GroundCalibration& getCalibration() const`, `const SignalPlayer& getSignals() const` | För OLED, logg och svart låda |

Ordningen i `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. med levande förbindelse – `switches->update(rc)` (läget, funktioner, rattar);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. spakarna `mixer.fromSticks(rc)` (med levande förbindelse) × `Knob::RATES`;
   klaffarna `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → steglöst, vid förlorad förbindelse – 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` – **alltid**;
   i reversering skickas `PWM_MIN` i stället för `pilotThrottle` (för autopiloten är motorn av);
6. kalibreringen: en ARM-omslagning under den avbryter den; utan ARM och med förbindelse – gesten `CAL_SWITCH`; ett steg i `GroundCalibration::update()`; dess signaler och autostartens vägransignal går till `SignalPlayer`; summern: `Beeper::update(BEEPER, armed, failsafe, now)` eller signalljuset;
7. förbindelsen förlorad → `applyLinkLoss()` och cykeln lämnas;
8. `arming.update(rc, false, orsak)` – ARM spärras medan en kalibrering pågår eller en signal spelas (och i den takt där en ARM-omslagning avbröt en kalibrering), annars `reverse.armBlockReason(now)`; därefter `reverse.update(MOTOR_REVERSE, armed, now)` – vid DISARM byts riktningen i samma takt;
9. `command = autopilot->getCommand()` (eller spakarna utan autopilot), klaffarna – sina egna; signalens nickning går till höjd- och sidroder;
10. `output = mixer.mix(command)`; `output.motorReverse = reverse.isReversed()`;
    framåt – `output.throttle = autopilot->applyThrottle(pilotThrottle)` med gränsen
    `THROTTLE_LIMIT_PCT` × ratten `Knob::THROTTLE_LIMIT`, bakåt – pilotens gas med gränsen
    `min(REVERSE_THROTTLE_LIMIT_PCT, framåtgränsen)`; det tillåtna sträcks ut över hela spakens slag;
11. inte armerat eller `MOTOR_KILL` → `throttle = PWM_MIN` (sist); utan ARM sätter motorsignalen `escPulsesOff`;
12. AUX1 – lasten (`PAYLOAD_DROP`), AUX2 – kameran (`Knob::CAMERA_TILT`, `CAMERA_STAB` drar ifrån tippningen); `AUX1_REVERSED`/`AUX2_REVERSED` speglar pulsen.
13. `outputs.write(output)`.

`applyLinkLoss()`: om autopiloten är i failsafe (armerat: RTH eller glidflykt) – roderytorna och gasen följer autopilotens kommando (klaffarna fälls in mjukt, `MOTOR_KILL` och reversering tystar motorn, AUX som de var); annars `outputs.setFailsafe()`, och medan en signal spelas – neutralläge med nickning och ESC-pulspaus (sändaren lades undan – kalibreringen pågår).

---

## `Beeper`

**Fil:** `control/Beeper.h` · **Beror på:** `Config`

| Metod | Beskrivning |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | summerns tillstånd: 2 Hz om `Feature::BEEPER` eller ”modellen borttappad” (inte armerat, ingen förbindelse på längre tid än `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | läget ”leta efter mig i gräset” |

---

## `GroundCalibration`

**Fil:** `control/GroundCalibration.h` · **Beror på:** `Config`, `RcChannelState`, `ImuSensor`, `ImuOrientation`

Markkalibrering av flygplanet: steg 1 – stjärten upp, flygkroppen vågrätt (horisonten, `ImuOrientation::withLevel()`), steg 2 – på hjulen (markläget). Ett steg godkänns av sig självt efter `CAL_STILL_MS` stillhet; steg 2 väntar på att flygplanet flyttas, eller `CAL_PARK_WAIT_MS`. Sparas bara i sin helhet (`ImuSensor::setGroundCalibration()`), efter kontroll av markläget mot `LANDING_GEAR`. Blockerar inte cykeln.

`Stage`: `IDLE`, `LEVEL`, `PARK`, `DONE`, `FAILED`.

| Metod | Beskrivning |
|---|---|
| `bool watchSwitch(const RcChannelState&, uint32_t nowMs)` | gesten: `CAL_SWITCH` `CAL_SWITCH_FLIPS` gånger ned och upp inom `CAL_SWITCH_WINDOW_MS`, gasen nere; första avläsningen och gas som inte är nere nollställer räkningen |
| `void forgetSwitch()` | ARM eller ingen förbindelse – räkningen börjar om |
| `bool start(const ImuSensor*, uint32_t nowMs, const char* source)` | starta (signal 1); ingen IMU eller inget stöd – fel (signal 3) |
| `void cancel(const char* why)` | avbrott utan signal |
| `void update(ImuSensor*, bool armed, bool signalBusy, uint32_t nowMs)` | ett steg: ARM avbryter; medan en signal spelas godkänns ingenting; tidsgränsen `CAL_STEP_TIMEOUT_MS` – fel |
| `isActive()`, `getStage()`, `getStageSinceMs()`, `getFailure()`, `getParkPitch()`, `getParkRoll()` | tillstånd och resultat |
| `uint8_t takeSignal()` | hur många signaler som ska spelas: 1 – startad / steg 1, 2 – klar, 3 – fel (hämtas en gång) |
| `static const char* stageShort(Stage)` | `CAL1`, `CAL2`, `CALOK`, `CALER` – för OLED |

---

## `SignalPlayer`

**Fil:** `control/SignalPlayer.h` · **Beror på:** `Config`

Signaler till piloten – en funktion av tiden sedan seriens början. En signal: höjd- och sidroder nickar (`SIGNAL_NOD_US` upp, sedan ned, `SIGNAL_NOD_MS` vardera), utan ARM pausas ESC-pulserna i `SIGNAL_ESC_PAUSE_MS` (sedan spelar ESC:n sin redo-melodi), summerljuset lyser under nickningen; nästa signal kommer efter `SIGNAL_PERIOD_MS`.

| Metod | Beskrivning |
|---|---|
| `void play(uint8_t count, uint32_t nowMs, bool motor)` | en serie; `motor` – om ESC:n får röras (och `SIGNAL_MOTOR`) |
| `void stop()` | tystnad |
| `bool isBusy(uint32_t nowMs) const`, `uint8_t getCount(uint32_t nowMs) const` | en serie pågår (ARM är spärrad under tiden) |
| `bool escPulsesOff(uint32_t nowMs) const` | ta bort ESC-pulserna nu |
| `int16_t nodUs(uint32_t nowMs) const` | rodrens nickning, µs (`SIGNAL_SURFACES`) |
| `bool light(uint32_t nowMs) const` | summer / LED på `PIN_BUZZER` |
