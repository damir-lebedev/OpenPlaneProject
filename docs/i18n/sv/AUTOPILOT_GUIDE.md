# Referens för OpenPlanes autopilot

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../AUTOPILOT_GUIDE.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Vad autopiloten kan, hur man slår på varje funktion och hur man lägger den på vilken brytare eller ratt som helst på sändaren **med en enda rad**.

> Ärligheten först. Alla lägen har verifierats med enhetstester och flygsimuleringar i sluten slinga (`test/native/test_sim`: hela firmwaren flyger en flygplansmodell). Modellen är förenklad, och koefficienterna i `Config.h` är startvärden: **prova varje läge först på minst 50 m höjd med fingret på MANUAL-brytaren**. Hittills har bara manuellt läge flugits (den första prototypen, med en ESP32-C3); stabiliseringen har verifierats på bänken – ytorna reagerar på lutning åt rätt håll. STM32H743-kortet byggs och klarar samma tester; på riktig hårdvara har DevEBox-kortet verifierats utan sensorer: SD-kortet, den svarta lådan och manuell styrning av servon och motor från sändaren (inspelat på video); inga sensorer har ännu anslutits till det, och autopilotlägena har inte provats på det.

---

## Innehåll

1. [Hur det fungerar på en minut](#hur-det-fungerar-på-en-minut)
2. [Standardfördelning på sändaren](#standardfördelning-på-sändaren)
3. [Tilldela en funktion med en enda rad](#tilldela-en-funktion-med-en-enda-rad)
4. [Lägen](#lägen)
5. [Funktioner (brytare)](#funktioner-brytare)
6. [Rattar](#rattar)
7. [Förlorad förbindelse, geofence, hem](#förlorad-förbindelse-geofence-hem)
8. [Ett hemmabyggt pitotrör](#ett-hemmabyggt-pitotrör)
9. [Markstation: Wi-Fi-panel och MAVLink](#markstation-wi-fi-panel-och-mavlink)
10. [Inställning av ett nytt flygplan, steg för steg](#inställning-av-ett-nytt-flygplan-steg-för-steg)
11. [Autopilotens kontroll före flygning](#autopilotens-kontroll-före-flygning)
12. [Vad varje läge behöver](#vad-varje-läge-behöver)

---

## Hur det fungerar på en minut

```
spakar  ─┐
         ├─► PilotSwitches (config/Controls.h) ─► läge, funktioner, rattar
brytare ─┘                                              │
                                                        ▼
sensorer (IMU, barometer, kompass, GPS, pitotrör) ─► Autopilot ─► roder- och gaskommandon
                                                        │
                           FlightController: klaffar, last, kamera, summer, failsafe
                                                        ▼
                                    skevroder · höjdroder · sidroder · ESC · AUX1 · AUX2
```

- **Läget** avgör vem som flyger: piloten (MANUAL), piloten med en hjälpreda (STABILIZE, ALT_HOLD, ACRO), autopiloten med pilotens korrigeringar (CRUISE, LOITER, RTH…).
- **Funktioner** slås på ovanpå vilket läge som helst: klaffar, broms, lastsläpp, geofence…
- **Rattar** ändrar ett tal steglöst: stabiliseringens styrka, marschfart, cirkelradie…
- I lägena med stabilisering **anger spaken vinkeln**, inte roderutslaget: släpp spaken så rätar flygplanet upp sig självt.
- Ett sensorfel ”rycker” aldrig i flygplanet: ingen IMU – ytorna stannar hos piloten; ingen barometer – piloten håller höjden; ingen GPS – ingen navigering, och de lägen som behöver den beter sig säkert (se [tabellen](#vad-varje-läge-behöver)).

---

## Standardfördelning på sändaren

FS-i6 + FS-iA6B, iBUS, 10 kanaler (`config/Channels.h`).

| Kanal | Reglage på sändaren | Standard |
|---|---|---|
| CH1–CH4 | spakar | roll, tippning, gas, sidroder (kan inte tilldelas om) |
| CH5 | **SwA** | **ARM** (ned = armerad, bara med gasen i botten; kan inte tilldelas om) |
| CH6 | SwB | klaffar (`Feature::FLAPS`) |
| CH7 | **SwC** (3 lägen) | upp **MANUAL** · mitten **STABILIZE** · ned **AUTO_TAKEOFF** |
| CH8 | SwD | **RTH** – hem, så länge den är påslagen |
| CH9 | VrA | stabiliseringens styrka (`Knob::STAB_GAIN`) |
| CH10 | VrB | marschfart (`Knob::CRUISE_SPEED`) |

När kortet startar skriver seriemonitorn ut den faktiska fördelningen – det som verkligen är flashat (firmwaren skriver på ryska; ”вверх / середина / вниз” betyder upp / mitten / ned):

```
SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (вверх / середина / вниз)
SwB (CH6): FLAPS, пока включён
SwD (CH8): RTH, пока включён
VrA (CH9): крутилка STAB_GAIN
VrB (CH10): крутилка CRUISE_SPEED
```

> Kanalerna 7–10 är inte påslagna på FS-i6 från början. I sändarens meny: **Functions setup → Aux. channels**, tilldela SwC, SwD, VrA, VrB.

---

## Tilldela en funktion med en enda rad

Allt finns i en fil – `include/config/Controls.h`:

```cpp
constexpr Binding BINDINGS[] = {
    Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_AUTO_TAKEOFF),
    Bind::feature(Channels::SWB, Feature::FLAPS),
    Bind::mode   (Channels::SWD, MODE_RTH),
    Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
    Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
};
```

| Radens form | Vad den gör |
|---|---|
| `Bind::modes(kanal, upp, mitten, ned)` | en trelägesbrytare väljer läget |
| `Bind::modes(kanal, upp, ned)` | en tvålägesbrytare – två lägen |
| `Bind::mode(kanal, läge)` | ett läge **ovanpå** de andra så länge brytaren är på; slå av den så återkommer läget från lägesbrytaren |
| `Bind::feature(kanal, funktion)` | funktionen verkar så länge brytaren är på |
| `Bind::knob(kanal, ratt)` | en ratt: mitten = värdet från `Config.h`, ändlägena = minimum och maximum |

”På” betyder att kanalen är över 1750 µs (på FS-i6, brytaren ned, mot dig). Tills den första ramen från mottagaren kommer räknas alla kanaler som avslagna: ingenting fälls ut eller släpps vid start.

### Färdiga recept

```cpp
// Termikglidare: SwD – termikflygning, SwB – autotrimning, VrB – cirkelradie
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_SOARING),
Bind::feature(Channels::SWB, Feature::AUTO_TRIM),
Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Elev: bara stabilisering, RESCUE på en "panikknapp", mjuka spakar
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_ALT_HOLD, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_RESCUE),
Bind::feature(Channels::SWB, Feature::GEOFENCE),
Bind::knob   (Channels::VRA, Knob::RATES),
Bind::knob   (Channels::VRB, Knob::MAX_BANK),

// Filmning och leverans: kamera med stabilisering, lastsläpp, cirkling över en punkt
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_CRUISE, MODE_LOITER),
Bind::feature(Channels::SWB, Feature::PAYLOAD_DROP),
Bind::feature(Channels::SWD, Feature::CAMERA_STAB),
Bind::knob   (Channels::VRA, Knob::CAMERA_TILT),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Handstart utan landningsställ: SwD – LAUNCH, klaffar steglöst på en ratt
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_LAUNCH),
Bind::feature(Channels::SWB, Feature::AIRBRAKE),
Bind::knob   (Channels::VRA, Knob::FLAPS),
Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
```

### Kompilatorn fångar misstag

Tabellen kontrolleras vid bygget (`static_assert`), innan firmwaren hamnar i flygplanet:

- spakarna och SwA (ARM) kan inte bindas, och kanalnumret måste vara mindre än `Channels::COUNT`;
- en kanal har bara en bindning;
- det får finnas högst en lägesväljare (`Bind::modes`).

Om flera `Bind::mode`-brytare är på samtidigt vinner den översta raden i tabellen.

---

## Lägen

Läget byts när en brytare **slås om**. Ett läge som slagits på från panelen eller markstationen gäller tills piloten slår om en lägesbrytare igen. OLED-skärmen visar ett kort namn (inom parentes).

### MANUAL (MAN)
Ytor = spakar, som utan flygkontroller. Bara funktionerna (klaffar, broms, last) och autotrimningen verkar. **Det viktigaste säkerhetsläget**: ha det alltid på en brytare under fingret.

### STABILIZE (STAB) – ”spaken anger vinkeln”
Rollspaken anger krängningsvinkeln upp till `MAX_BANK_DEG` (45°, ratten `MAX_BANK` 15…60°), tippspaken anger vinkeln upp till `STAB_MAX_PITCH_DEG` (25°). Släpp – flygplanet rätar upp sig självt. Gasen är pilotens. PID-integratorn ackumulerar bara nära målet (±10°), så efter en skarp manöver ”skjuter” flygplanet inte förbi horisonten.
**Behöver:** IMU. Utan IMU – samma som MANUAL.

### ALT_HOLD (ALT) – ”håll höjden”
Som STABILIZE i roll, medan höjdrodret håller höjden med barometern. Rör tippspaken – du flyger själv; släpp – flygplanet håller den **nya** höjden. Gasen är pilotens (ge gas, annars räcker inte farten för att stiga).
**Behöver:** IMU + barometer.

### ACRO – ”spaken anger rotationshastigheten”
Fullt spakutslag – 180°/s. Släpp – flygplanet **håller den attityd det hade** (även upp och ned), och gyroskopet dämpar byar. För avancerad flygning.
**Behöver:** IMU. Utan IMU – ytor = spakar.

### CRUISE (CRZ) – ”håll kurs, höjd och fart”
Flygplanet flyger rakt på den aktuella höjden, med automatisk gas (ratten `CRUISE_SPEED`: 30…55…85 % gas, och med pitotrör – **lufthastighet** 10…14…22 m/s). Rollspaken svänger (släpp – den håller den nya kursen), tippspaken ändrar höjden. Med pitotrör verkar överstegringsskyddet. Kursen kommer från GPS (vid en markhastighet > 3 m/s, tillbaka till kompassen under 2 m/s), annars från kompassen, annars från gyroskopet. I simulering, med 4 m/s sidvind, håller markspåret sig inom ±5° och höjden inom ±3 m.
**Behöver:** IMU + barometer; GPS eller kompass för en kurs utan avdrift.

### LOITER (LOIT) – ”cirkla här”
Cirklar medurs över punkten där läget slogs på, radie 50 m (ratten `LOITER_RADIUS` 25…150 m), på den aktuella höjden, med automatisk gas. Styrningen är ett vektorfält: på långt avstånd ansluter flygplanet till cirkeln längs en tangent, och på cirkeln håller det en framkopplad krängning. Utan GPS – bara en cirkel med konstant krängning på stället.
**Behöver:** IMU + barometer + GPS.

### RTH – ”hem”
Kurs mot hempunkten (ARM-punkten), höjd `RTH_ALTITUDE_M` = 40 m (under den – stiger på vägen, över den – stannar kvar). Över hempunkten – cirklar med LOITER-radien tills piloten tar över. Ingen GPS eller ingen hempunkt – cirklar på stället. Samma läge kopplas in vid förlorad förbindelse och av geofencen.
**Behöver:** IMU + barometer + GPS med hempunkt.

### AUTO_TAKEOFF (TKOFF) – ”start på gas”
Efter ARM händer ingenting förrän piloten höjer gasen över mitten. Sedan programmet: 1 s acceleration till 100 % gas med vingarna plana, 2 s rotation med tippning 15°, sedan stigning med tippning 10° tills piloten byter läge. Spakarna läggs ovanpå programmet – du kan räta upp kursen under startrullningen.
**Behöver:** IMU.

### LAUNCH (LNCH) – ”handstart”
1. Armerat, gas över mitten – starten är **förberedd**, motorn står still.
2. Kastet: acceleration framåt > 1,5 g i mer än 40 ms.
3. Efter 0,3 s (handen är borta från propellern) – gas 100 %, stigning med tippning 15°, vingarna plana – i 6 s eller till 30 m.
4. Därefter – som CRUISE på den nådda höjden.

Varje spakrörelse (> 150 µs) före kastet avbryter det: flygplanet är i pilotens händer.
**Behöver:** IMU (accelerometer). I simulering: ett kast i 9 m/s från handhöjd – flygplanet rör aldrig marken och stiger mer än 15 m på 15 s.

### AUTO_LAND (LAND) – ”landning”
Motorn är av, glidflykt på kurs med tippning −4°; under 3 m enligt barometern – flare (+4°). Rollspaken justerar inflygningskursen. Koppla in det på en rak sträcka, mot vinden, på 20–40 m, med bana att ta av. I simulering sker sättningen med en vertikal hastighet under 1,5 m/s, vingarna plana, inte på nosen.
**Behöver:** IMU + barometer (nollställd på marken vid start).

### SOARING (SOAR) – ”termikflygning”
Motorn är av, glidflykt. En variometer (med pitotrör – totalenergi, utan falsk ”termik” av att dra spaken bakåt) över 0,5 m/s i mer än 1,5 s – termik: cirklar med 25° krängning. En genomsnittlig stigning över 8 s som sjunker under −0,2 m/s – ut ur termiken. Under 30 m – motorn tills 100 m; längre bort än 400 m från hempunkten – glider hem. I simulering hittar den en termik (en kärna på 3 m/s) och stiger mer än 50 m utan motor.
**Behöver:** IMU + barometer; GPS – för att återvända hem.

### RESCUE (RESQ) – ”rädda mig”
Vingarna plana, nosen +8°, gas 70 % – ur vilken spiral som helst. Tappat orienteringen? Slå om brytaren och andas ut. I simulering, från en spiral med 70° krängning och nosen på −40°, på 4 s – vingarna plana och stigning.
**Behöver:** IMU.

---

## Funktioner (brytare)

| Funktion | Vad den gör | Detaljer och siffror (`Config.h`) |
|---|---|---|
| `FLAPS` | båda skevrodren ned – flaperoner | `FLAPS_DEPLOYED_US` = 220 µs, mjukt på 1 s; roll verkar ovanpå |
| `AIRBRAKE` | båda skevrodren upp – en luftbroms, brantare glidbana | `AIRBRAKE_US` = 250; har företräde framför klaffarna |
| `AUTO_TRIM` | lär sig hålla flygplanet rakt utan spakarna: det stadiga roderkommandot i planflykt ”flyter över” i trimmet | 20 %/s, upp till ±120 µs; sparas i flash efter DISARM **på marken** |
| `TURN_COORDINATION` | sidroder in i svängen, nosen upp i krängningen | alltid på i navigeringslägena |
| `MOTOR_KILL` | motorn är av i alla lägen, även de automatiska | går före alla lägen och gasen |
| `BEEPER` | ”jag är här”-summern | utan brytare piper den av sig själv: på marken, förbindelsen bruten > 10 s |
| `PAYLOAD_DROP` | AUX1-servot är öppet så länge brytaren är på | 1000 µs stängt, 2000 öppet |
| `GEOFENCE` | längre bort än 500 m från hempunkten eller högre än 120 m – RTH | `GEOFENCE_ALWAYS_ON` – utan brytare |
| `HOME_RESET` | hempunkt = den aktuella punkten (i det ögonblick brytaren slås på) | bara med bra GPS |
| `CAMERA_STAB` | kameran på AUX2 håller sin vinkel mot horisonten | flygplanets tippning dras ifrån |

## Rattar

Mitten på en ratt = standardvärdet från `Config.h`; ändlägena är minimum och maximum. Om en ratt inte är bunden gäller standardvärdet.

| Ratt | Minimum … mitten … maximum | Var den verkar |
|---|---|---|
| `STAB_GAIN` | ×0,25 … ×1 … ×2 | alla lägen med stabilisering, och ACRO – ”mjukare/hårdare” |
| `MAX_BANK` | 15° … 45° … 60° | den största krängningen från spaken och från navigeringen |
| `CRUISE_SPEED` | gas 30 … 55 … 85 % (med pitotrör: 10 … 14 … 22 m/s) | CRUISE, LOITER, RTH, motorn i SOARING |
| `FLAPS` | 0 … 50 … 100 % klaffar | steglösa klaffar i stället för en brytare |
| `CAMERA_TILT` | −90° … 0° … +30° | kameravinkeln (AUX2) |
| `RATES` | 30 … 65 … 100 % av spakutslaget | alla lägen: spakkänslighet |
| `LOITER_RADIUS` | 25 … 50 … 150 m | LOITER och cirklar över hempunkten |

> Ett användbart knep: ratten `STAB_GAIN` på VrA är ”levande” justering av koefficienterna under flygning. Om det svänger – vrid ned; om det är trögt – vrid upp; för sedan över multiplikatorn till `Config.h`.

---

## Förlorad förbindelse, geofence, hem

**Hempunkten** registreras vid ARM om GPS:en är bra (3D-fix, ≥ 6 satelliter, noggrannhet ≤ 5 m). Om GPS:en inte har låst före ARM registreras hempunkten så snart den gör det. För att ändra den på fältet – funktionen `HOME_RESET`.

**Förlorad förbindelse** (inga iBUS-ramar på > 0,5 s, eller mottagaren skickade en gas under 950 µs – det är den failsafe som ställts in i sändaren; se `docs/PILOT_GUIDE.md`):

| Situation | Vad planet gör |
|---|---|
| på marken (inte armerat) | motor 0, ytor till neutralläge; efter 10 s – summern |
| i luften, GPS och hempunkt finns | **RTH** med motor, över hempunkten – cirklar på 40 m |
| i luften, ingen GPS | **glidflykt**: motorn av, vingarna plana, nosen −3° |
| förbindelsen är tillbaka | direkt läget från pilotens brytare |

En hemflygning som redan har börjat övergår inte till glidflykt på grund av en kort GPS-förlust. `FAILSAFE_RTH = false` – bara glidflykt.

**Geofence** (`GEOFENCE` eller `GEOFENCE_ALWAYS_ON`): flyger planet längre bort än `FENCE_RADIUS_M` (500 m) eller högre än `FENCE_ALTITUDE_M` (120 m) – RTH. För att ta tillbaka kontrollen, slå om en lägesbrytare till vilket annat läge som helst (ett läge slås på genom att läget ändras). Den löser ut igen när flygplanet har kommit tillbaka innanför med 10 % marginal.

---

## Ett hemmabyggt pitotrör

Lufthastighet utan en köpt differenstrycksensor: **två barometrar**.

```
     mötande luftström ─►  ┌────────────── rör (PVC/mässing, Ø4–6 mm) ──┐
                            │  BMP581 (I2C 0x47) – totaltryck         │  lufttätt
                            └─────────────────────────────────────────┘
   flygkropp: huvudbarometer (BMP581 0x46 / SPL06 / BMP388) – statiskt tryck

   hastighet  V = √(2·(P_rör − P_statiskt − noll) / ρ),   ρ – från statiskt tryck och temperatur
```

**Montering.** BMP581 (en modul med adress 0x47: stiftet SDO till VCC) limmas in i ett rör som bara är öppet framtill – kortet sitter i ett lufttätt hålrum, med trådarna utdragna genom tätningsmassa. Röret pekar framåt, utanför propellerströmmen (på vingen eller ovanför nosen). Den andra barometern sitter inne i flygkroppen, skyddad från den direkta luftströmmen (skumplast).

**Påslagning i firmwaren** – `sensors/SensorSelection.h`: `SENSOR_KIT_LSM6DSV_PITOT` eller `SENSOR_KIT_ICM45686_PITOT` (färdiga satser), eller `SENSOR_AIRSPEED = SENSOR_AIRSPEED_PITOT_BMP581` i din egen sats.

**Nollpunkt.** Två barometrar skiljer sig alltid lite åt: den absoluta noggrannheten hos var och en är tiotals pascal, och det är hela tryckskillnaden vid låg fart (10 m/s ≈ 60 Pa). Under den första sekunden efter start medelvärdesbildar firmwaren skillnaden och tar den som noll. **Flygplanet står stilla vid start, och röret är täckt med ett finger eller ett lock, eller vänt mot vinden.** På OLED-skärmen, i panelen och i telemetrin visas hastigheten efter nollställningen.

**Kalibrering av `PITOT_SCALE`.** Trycket inne i flygkroppen är inte strikt statiskt. Flyg i stilla luft en rak sträcka fram och tillbaka i CRUISE och jämför den genomsnittliga markhastigheten från GPS med rörets hastighet: `PITOT_SCALE = V_GPS / V_rör`.

**Skydd.** En starkt negativ tryckskillnad i mer än 2 s (förväxlade slangar, vatten) eller rörmätningar äldre än 0,2 s – ingen hastighet matas ut, och autopiloten faller tillbaka på gasen från ratten och flyger kursen utan lufthastighet. Verifierat i en simulering i sluten slinga med brus på båda barometrarna: hastighetsfelet under flygning är < 0,5 m/s.

Vad röret ger dig: CRUISE håller **lufthastigheten** i stället för gasen; överstegringsskydd; en totalenergivariometer för SOARING; en ärlig hastighet i telemetrin.

---

## Markstation: Wi-Fi-panel och MAVLink

**ESP32 – Wi-Fi-panel.** Åtkomstpunkten `OpenPlane-Debug`, lösenord `12345678`, adressen står i seriemonitorn. Kanaler, utgångar, alla sensorer, läget, navigering, de påslagna funktionerna; du kan ändra läget och PID. Detaljer – `docs/PILOT_GUIDE.md`.

**STM32H743 – MAVLink via radiomodem** (UART4: PD0 RX, PD1 TX, 57600 baud – SiK-standard). SiK 433/868/915 MHz, ELRS i MAVLink-läge och en ESP-01 som Wi-Fi-brygga fungerar alla. **QGroundControl** och **Mission Planner** ser planet som ett ArduPilot-flygplan:

- horisonten, en karta med hempunkten, hastighet (från pitotröret, om det finns), höjd, variometer, gas;
- lägen under ArduPlane-namnen: STABILIZE → FBWA, ALT_HOLD → FBWB, CRUISE → CRUISE, LOITER → LOITER, RTH → RTL, AUTO_TAKEOFF/LAUNCH → TAKEOFF, SOARING → THERMAL, RESCUE → STABILIZE, AUTO_LAND → AUTO; failsafe visas som RTL eller CIRCLE;
- ett meddelandeflöde: ARM/DISARM, lägesbyten (under vårt eget namn), förlorad förbindelse, geofence;
- **byte av läge från marken** – med lägesknappen i GCS:en (utom AUTO: OpenPlane har inga uppdrag);
- **parametrar** `RLL_KP … PTCH_KD` – PID för roll och tippning, läses och ändras från GCS:ens parameterfönster mitt under flygningen (de sparas inte över en omstart – för över bra värden till `Config.h`).

ARM/DISARM från marken **avvisas** – bara med sändarens brytare. För att kontrollera strömmen utan hårdvara: `OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink && python3 tools/check_mavlink.py /tmp/tlm.bin` (kräver `pip install pymavlink`).

---

## Inställning av ett nytt flygplan, steg för steg

1. **MANUAL på marken:** rodrens riktningar (`*_REVERSED` i `Config.h`), klaffar ned, AUX. Kontrollera utgångarna med `p` i konsolen (propellern av!).
2. **Sensorer på marken:** `b` – bussavsökning, `s` – status; `o` – kalibrering av IMU-monteringen (3 positioner, en gång), `m` – kompass.
3. **STABILIZE på marken:** luta flygplanet åt höger – det högra skevrodret ska gå **ned** (för att räta upp det). Nosen upp – höjdrodret ned. Om inte – reverseringarna eller IMU-monteringen är förväxlade.
4. **Den första flygningen i MANUAL**, stig till minst 50 m → STABILIZE. Om det svänger – sänk `STAB_GAIN`; om det är trögt – höj den.
5. **AUTO_TRIM** i planflykt i 20–30 s, landa, DISARM – trimmet sparas.
6. **ALT_HOLD**, sedan **CRUISE** – kontrollera höjd och kurs; med pitotrör – kalibrera `PITOT_SCALE`.
7. **LOITER** och **RTH** – på höjd, inom synhåll, fingret på MANUAL.
8. Först därefter – **failsafe-testet** (stäng av sändaren på höjd, flygplanet ska styra hemåt) och automatisk start/landning.

## Autopilotens kontroll före flygning

- [ ] Fördelningen som skrivs ut vid start är den du förväntar dig.
- [ ] Konsol/OLED: IMU ok, IMU-kontrollen före flygning godkänd (flygplanet stod stilla vid start).
- [ ] Barometern är nollställd på marken (höjd ~0 på OLED-skärmen).
- [ ] Med pitotrör: hastighet ~0 på marken, blås i röret – den stiger.
- [ ] GPS: 3D-fix, ≥ 6 satelliter **före ARM** – annars blir det ingen hempunkt och ingen RTH.
- [ ] STABILIZE på marken: skevrodren och höjdrodret rätar upp flygplanet i stället för att välta det.
- [ ] Failsafe är inställt i sändaren (gas under 950 vid förlorad förbindelse) och verifierat genom att stänga av sändaren **på marken** utan propeller.
- [ ] MANUAL – under fingret.

---

## Vad varje läge behöver

| Läge | IMU | Barometer | GPS | Kompass | Pitot | Gas | Utan den nödvändiga sensorn |
|---|:-:|:-:|:-:|:-:|:-:|---|---|
| MANUAL | | | | | | pilot | – |
| STABILIZE | ● | | | | | pilot | ytor = spakar |
| ALT_HOLD | ● | ● | | | | pilot | piloten håller höjden |
| ACRO | ● | | | | | pilot | ytor = spakar |
| CRUISE | ● | ● | ○ | ○ | ○ | auto | kurs från gyroskopet (driver), höjden hos piloten |
| LOITER | ● | ● | ● | | ○ | auto | en cirkel med krängning på stället |
| RTH | ● | ● | ● | | ○ | auto | cirklar på stället |
| AUTO_TAKEOFF | ● | | | | | program | ytor = spakar + pilotens gas |
| LAUNCH | ● | ○ | | | | program | ytor till neutralläge |
| AUTO_LAND | ● | ● | | ○ | | 0 | ingen flare |
| SOARING | ● | ● | ○ | | ○ | 0 / motor | ingen termik – glidflykt |
| RESCUE | ● | | | | | 70 % | ytor till neutralläge |

● – krävs, ○ – förbättrar. Kontroller i koden: `Autopilot.h` (`imuReady`, `baroReady`, `nav.gpsGood`); tester – `test/native/test_autopilot_modes` (hur lägena reagerar på varje sensor) och `test/native/test_sim` (flygningar i sluten slinga).
