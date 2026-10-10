# OpenPlaneProject – färdplan och presentation för investerare och partner

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../ROADMAP.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

> Repository: [github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject), gren `main`.
> Det här dokumentet är en fördjupad version av README-filens teaser, riktad till dem
> som överväger att investera pengar, tid eller partnerskap i projektet. Det beskriver
> var projektet står nu, vart och varför det är på väg och vad exakt i den kod
> som redan har skrivits som gör den här banan realistisk snarare än bara utlovad.

## 1. Nuläge – ärligt talat

OpenPlaneProject är i dag en prototyp till ett glidplan som har flugit men är omogen, med
egen firmware på ESP32 – inte en färdig produkt och inte en autonom drönare.
Hårdvara: spännvidd 1200 mm, korda 250 mm, profil NACA 4412, en PETG-konstruktion
(3D-utskriven), MG90S-servon (ett separat för varje skevroder), 3S LiPo-matning. Den första prototypen
(ESP32-C3, D2212 1000KV-motor, 40A ESC) har redan flugit med manuell styrning och visade efter
flygningen konkreta problem: otillräcklig hållfasthet i motor- och vingfästena
(de behöver förstärkas med kolfiber) och behovet av att justera servona.

Det nuvarande bygget har gått över till ESP32-S3 (N16R8) med en D3548 1100KV-motor och en
60–80A ESC, och alla autopilotsensorer är anslutna till det på bänken: en IMU (MPU6500),
en BMP388-barometer, en QMC5883P-kompass samt en OLED-statusskärm. Verifierat live:
manuell RC-styrning över iBUS med en mixer för skevroder, höjdroder, sidroder
(med styrbart hjul) och klaffar (flaperoner);
ARM med en separat brytare; failsafe med sändaren avstängd; en styrslinga på 500 Hz;
en levande webbpanel över Wi-Fi. På bordet reagerar stabiliseringen på lutning
åt rätt håll.

Sedan dess har firmwaren fått 12 autopilotlägen (stabilisering, höjdhållning,
marschflygning, cirklar och hemflygning med GPS, handstart, autolandning, termikflygning,
”räddning”), geofence, hemflygning vid förlorad förbindelse, lastsläpp,
ett pitotrör av två barometrar, stöd för nya sensorer (LSM6DSV, ICM-45686,
QMC6309, SPL06, BMP581) och en fullständig firmware för STM32H743 med MAVLink-telemetri
för QGroundControl. Allt detta har kontrollerats med 387 automatiska tester och flygsimuleringar
i sluten slinga (hela firmwaren flyger en flygplansmodell), men **det har ännu inte
provats i luften**. Med andra ord: hittills har bara manuell styrning flugit; autopiloten är
skriven, kontrollerad med allt som går att kontrollera den med utan att flyga, och väntar på
flygprov.

## 2. Varför detta är viktigt

Autonom småskalig luftfart med låg inträdesbarriär täcker uppgifter där det avgörande
inte är maximal last eller räckvidd utan reaktionssnabbhet och låg driftskostnad:

- **Leverans av läkemedel och blod till svåråtkomliga och katastrofdrabbade
  områden** – bortspolade vägar, ingen infrastruktur, förstörelse efter naturkatastrofer
  eller konflikter. Det avgörande här är inte lastförmågan
  (paketet är litet) utan reaktionstiden: minuter och timmar i stället för ett dygn med
  bil eller till fots.
- **Sök- och räddningsinsatser** – precisionssläpp av utrustning, förbandslådor,
  kommunikationsutrustning och flytmedel till nödställda innan markstyrkan kommer fram, i områden
  där en helikopter är orimligt dyr eller otillgänglig på grund av väder eller terräng.
- **Precisionsjordbruk** – övervakning av grödor och punktvis sprutning/spridning av medel
  där slutna drönarplattformar för dessa uppgifter kostar från tusentals dollar, vilket är
  olönsamt för små och medelstora gårdar.

I alla tre kategorierna är ekonomin densamma: skillnaden mellan ”en lösning finns,
men den är dyr och stängd” och ”det finns i praktiken ingen lösning, eftersom den är dyr” – och det är just
den nischen som en billig öppen plattform siktar på. Detta är en beskrivning av tillämpningsområdet
och marknadsproblemet, inte ett påstående om att OpenPlaneProject redan kan
leverera last – i det nuvarande skedet är det ett uttalande om målet och om varför
målet är värt att sträva efter.

## 3. Investeringstes: varför en öppen ESP32-arkitektur är en asymmetri

Kommersiella autonoma drönarplattformar för leverans/övervakning byggs vanligtvis
på slutna flygkontroller och sluten mjukvara, kostar från hundratals till tusentals
dollar per farkost och kräver licensavgifter eller serviceavtal för
drift av en flotta. OpenPlaneProject utgår från ett annat antagande:

- **En billig bas.** En ESP32-modul kostar i storleksordningen 5–15 $, och den omgivande hårdvaran (MG90S-servon,
  en ESC, en iBUS-mottagare) består av vanliga komponenter i hobbyklass. Det är en
  storleksordning billigare än inträdesbiljetten till slutna kommersiella plattformar, vilket är
  avgörande för pilotinföranden i miljöer med begränsad budget (frivilligorganisationer,
  småskaligt jordbruk, regionala räddningstjänster).
- **Öppen kod ändrar förtroendets ekonomi.** En organisation som inför
  en flotta för medicinsk leverans kan granska säkerhetslogiken (failsafe,
  ARM) och anpassa firmwaren till sina egna sensorer och regler i stället för att
  vara beroende av en enda leverantör och dess färdplan.
- **Arkitekturen är redan i dag utformad för utbyggnad, inte bara för det
  nuvarande glidplanet.** Det är ingen utfästelse utan en direkt följd av hur koden är uppbyggd:
  - En ny sensor läggs till som en klass som implementerar det befintliga gränssnittet
    `Sensor` → `ImuSensor`/`BarometerSensor` (`include/sensors/SensorInterface.h`),
    utan att kärnan ändras. Så har drivrutinerna för IMU:er (MPU6050/MPU6500,
    ICM-42688), barometrar (BMP388, BME280), kompasser (QMC5883P/L) och GPS
    (u-blox M10) redan gjorts – alla skrivna direkt via bussregistren
    (gränssnitten `II2CBus`/`ISpiBus`/`IUartPort`), utan tredjepartsbibliotek, det vill säga utan
    dolda beroenden av en viss leverantörs SDK.
  - Ett nytt kort läggs till med ett enda `#elif`-block i `include/config/Config.h` plus
    ett enda `[env:...]`-block i `platformio.ini` – byte av kort fungerar redan
    i dag för fyra mål (se tabellen nedan); det är ingen hypotetisk
    möjlighet.
  - `Autopilot.h` tar redan emot `ImuSensor*`/`BarometerSensor*` som
    parametrar (de får vara `nullptr`) – det vill säga, kontraktet mellan autopiloten och
    hårdvaran förutsätter att uppsättningen sensorer kommer att ändras (nästa
    steg är GPS som ytterligare en klass med samma mönster, se fas 3).
  - `WebDebugServer.h` levererar redan en enda sammanställd JSON (`GET
    /api/status`) och tar emot kommandon (`POST /api/setmode`, `/api/setpid`)
    – det vill säga, protokollet ”farkosten levererar telemetri och
    tar emot kommandon” finns redan, och markstationen ska växa fram ur
    det snarare än skrivas från grunden.

Asymmetrin består i att inträdesbarriären (pengar, tid för att anpassa till en ny
uppgift) för den här plattformen är en storleksordning lägre än för slutna motsvarigheter, medan
vägen till autonomi inte kräver att kärnan skrivs om – bara att nya
klasser läggs till ovanpå befintliga gränssnitt. Det här är en öppen ingenjörsplattform, inte
en färdig kommersiell produkt – följaktligen är tesen för en investerare/partner
inte ”köp en färdig lösning” utan ”kom in i ett skede då grunden redan är
verifierad och nästa steg är tekniskt klara”.

## 4. Arkitekturen i dag – grunden för faserna nedan

### 4.1 Kort som stöds

Valet av kort är ett enda byggalternativ i PlatformIO; byte av kort kräver inte att man
rör logiken (`include/config/Config.h` + `platformio.ini`):

| Miljö (`pio run -e ...`) | Kort | Status | aileron L / R | elevator | esc | ibus_rx | i2c sda / scl |
|---|---|---|---|---|---|---|---|
| `esp32-s3` (standard) | ESP32-S3 N16R8 (DevKitC-1) | **Primärt, verifierat på bänken med alla sensorer** | GPIO4 / GPIO5 | GPIO6 | GPIO7 | GPIO17 | GPIO41 / GPIO42 |
| `esp32-c3` | ESP32-C3 SuperMini | Första prototypen, flög med manuell styrning | GPIO5 / GPIO4 | GPIO6 | GPIO7 | GPIO8 | GPIO1 / GPIO3 |
| `esp32-dev` | klassisk ESP32 38-pin | För bänken, **inte verifierat på hårdvara** (hela firmwaren täcks av tester) | GPIO13 / GPIO14 | GPIO27 | GPIO26 | GPIO16 | GPIO21 / GPIO22 |
| `stm32h743` | STM32H743VIT6 (WeAct Mini) | Fullständig firmware + MAVLink, **inget kort ännu** (hela firmwaren körs i tester på en dator) | PA0 / PA1 | PA2 | PA3 | PE7 | PB11 / PB10 |

Ladda upp firmwaren: `pio run -t upload`. Monitor: `pio device monitor` (115200).

### 4.2 RC-kanalkarta (FS-i6 + FS-iA6B, iBUS, 10 kanaler, 1000–2000 µs)

| Kanal | Namn | Standardanvändning |
|---|---|---|
| CH1–CH4 | spakar | roll, tippning, gas, sidroder |
| CH5 | ARM | brytaren SwA: ARM med gasen i botten, DISARM omedelbart |
| CH6 | SWB | klaffar |
| CH7 | SWC | läge: MANUAL / STABILIZE / AUTO_TAKEOFF |
| CH8 | SWD | RTH – hem |
| CH9 | VRA | stabiliseringens styrka |
| CH10 | VRB | marschfart |

CH6–CH10 tilldelas med en enda rad i `include/config/Controls.h`: vilket som helst av de 12
lägena, 10 funktionerna (klaffar, broms, lastsläpp, geofence…) och 7 rattarna –
[AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

### 4.3 Styrflödet (`FlightController::update()`)

En enda orkestrerare med fast ordning; slingan körs på 500 Hz med fast
period: iBUS-mottagning → pilotens gas → läge från CH7 → **sensoravläsning och autopilotens
beräkning (alltid, även utan förbindelse)** → **kontroll av förlorad förbindelse med absolut
prioritet** (i luften – hem med GPS med motorn igång, eller glidflykt med plana vingar
när GPS saknas; på marken – roderytor till neutralläge) → ARM → spakarnas kommando +
autopilotens korrigeringar i enhetliga flygtekniska teckenkonventioner → mixer med servoreversering →
lägets gas → gasen blockerad utan ARM → skrivning till servon/ESC. Det är just den här
disciplinen i ordningsföljden (säkerhet först, sedan manuell styrning, sedan autopiloten som en
överbyggnad) som gör det möjligt att säkert bygga in alltmer
autonomt beteende utan att skriva om grundslingan. Wi-Fi, panelen och skärmen
körs på den andra kärnan och fördröjer inte styrningen.

### 4.4 Webbpanelen som fröet till en markstation

`WebDebugServer.h` startar redan i dag en åtkomstpunkt (SSID
`OpenPlane-Debug`, IP `192.168.4.1`) och levererar och tar emot JSON:

| Metod och sökväg | Vad den gör |
|---|---|
| `GET /api/status` | En enda sammanställd JSON: RC (10 kanaler), armed/failsafe, 7 utgångar (`us`, `attached`), IMU, barometer, kompass, GPS, pitotrör (var och en med `attached`/`available` + data), autopilot (läge, korrigeringar, PID, navigering, påslagna funktioner) |
| `POST /api/setmode` | `{mode: 0-11}` – byt autopilotläge |
| `POST /api/setpid` | `{kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch}` – alla fält är valfria |
| `GET /` | HTML-panelen: levande staplar för de 10 kanalerna, status för varje utgång/sensor, lägesknappar, ett PID-formulär |

Fälten `attached`/`available` finns alltid i JSON – panelen visar ärligt
”inte med i uppsättningen” separat från ”med i uppsättningen men svarar inte”, i stället för att
tiga om en saknad sensor. Det är samma ärlighetsprincip
som ligger till grund för hela detta dokument: utge inte det önskade för det verkliga.

## 5. En teknisk färdplan fas för fas

Nedan följer åtta faser, var och en beskriven som nästa logiska steg ovanpå de
klasser som redan finns, utan påhittade datum eller summor.

### Fas 1 – Manuell styrning och en säker bas [klar]

**Mål:** ett pålitligt, förutsägbart radiostyrt glidplan med genomskinlig felsökning. **Vad som redan finns tekniskt:** iBUS-tolkning med upptäckt av förlorad förbindelse i `IBusReceiver.h`, mixern `ControlMixer.h` (spakar → roll-/tipp-/klaffkommando → PWM med servoreversering, utan kunskap om UART/PWM), `ThrottleManager.h`, `ArmingManager.h` (ARM med en separat brytare med gasen i botten, omedelbar DISARM), failsafe med absolut prioritet i `FlightController.h`, utskrift till Serial (`DebugLogger.h`), webbpanelen (`WebDebugServer.h`) och OLED-skärmen (`OledDisplay.h`). **Varför detta är basen för allt annat:** det är det enda lagret som alltid måste fungera, även om alla andra faser ännu inte har genomförts eller deras sensorer är bortkopplade – och det är just därför failsafe och ARM skrevs först och verifierades på hårdvara (inklusive det verkliga beteendet hos FS-iA6B-mottagaren med sändaren avstängd).

### Fas 2 – IMU + barometer → autopilot [skriven och verifierad med tester och simulering, väntar på flygprov]

**Mål:** det första autonoma flygläget – horisontstabilisering, autostart, höjdhållning. **Vad som redan finns tekniskt:** `imu/MPU6050_Sensor.h` (MPU6050 och MPU6500, register som nås direkt, axelrotation för att matcha kortets montering, flygtekniska teckenkonventioner, ett komplementärfilter i `ImuSensorBase`), `baro/BMP388_Sensor.h` (I2C eller SPI, fullständig Bosch-kompensering, avläsning på flaggan data-klar, en filtrerad vertikal hastighet), `Autopilot.h` med `PidController` (D-termen från gyroskopet, integratorn ackumulerar bara efter ARM) och tolv lägen (från MANUAL till SOARING och RESCUE), som byts med brytare enligt tabellen i `Controls.h`, från webbpanelen och från QGroundControl, plus failsafe (hem eller glidflykt). Varje läge flyger i en simulering i sluten slinga av hela firmwaren med en flygplansmodell (`test/native/test_sim`). På bänken med ESP32-S3 svarar alla sensorer och tecknen har verifierats live: lutning → roderytorna korrigerar mot plant läge. **Vad som behövs för att avsluta fasen:** flytta elektroniken in i glidplanet, kontrollera roderytornas riktningar på den monterade farkosten och genomföra de första flygproven – med början i STABILIZE på säker höjd.

### Fas 2.5 – Återkoppling från det verkliga flygplanet [förarbete, verifierat i simulering]

**Mål:** att autopiloten inte ska bero på koefficienter som justerats för en enda hastighet, utan på hur det verkliga flygplanet reagerar på roderytan just nu. PID:en från fas 2 ger roderutslag ”enligt en formel” och kontrollerar inte resultatet; vid låg fart korrigerar den för lite, vid hög fart för mycket. **Vad som redan finns tekniskt** (`include/autopilot/feedback/`, **inte ansluten** till firmwaren): skattning under flygning av roderytornas verkan (rekursiva minsta kvadrater, omskalad med hastigheten ∝ V²), en regulator ”vinkel → rotationshastighet → roderyta” med uppföljande korrigering (”ytan vred inte hela vägen – vrid den längre”), skydd mot förlust av lufthastighet och överstegring (gas, nosen ned, vingarna plana), start från bana eller för hand och landning i steg som styrs av sensorerna. Allt har verifierats med en flygplanssimulering i sluten slinga på själva kortet (`pio test -e esp32-s3 -f test_feedback`, 10 scenarier) – inklusive ett förväxlat skevroder, turbulens, en höjd nos vid låg gas, start och landning. **Vad som behövs för att avsluta fasen:** efter de första flygningarna i fas 2 – ”skuggläge” (återkopplingen skriver bara i loggen vad den skulle ha gjort), sedan anslutning en axel i taget, en lufthastighetssensor (pitotrör) och en avståndsmätare för utflytning före sättningen. Detaljer – avsnittet ”Återkoppling” i [`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md).

### Fas 3 – GPS och kompass [i firmwaren, verifierat med simulering]

**Status (uppdaterad): navigeringen fungerar** – kurs från GPS/kompass/gyroskop med hysteres, hempunkt satt vid ARM, CRUISE, LOITER (ett vektorfält mot en cirkel), RTH, geofence; verifierat med simuleringar i sluten slinga. Nedan följer den ursprungliga texten för fasen. GPS (u-blox M10, `sensors/gps/UbloxM10_Gps.h`, protokollet UBX-NAV-PVT) och en magnetometer (två varianter av kortet ”GY-273”: QMC5883P – `sensors/mag/QMC5883P_Sensor.h`, monterad på den nuvarande bänken och verifierad live; QMC5883L – `sensors/mag/QMC5883L_Sensor.h`) lades till som nya klasser som implementerar gränssnitten `GpsSensor`/`MagnetometerSensor` (`include/sensors/SensorInterface.h`) enligt samma princip som IMU:n och barometern – `Autopilot.h` och `FlightController.h` skrevs inte om; de fick två datakällor till på samma sätt (en nullbar pekare i konstruktorn). Längs vägen kom ett HAL-lager (`include/hal/`) till, genom vilket sensorerna når bussarna – I2C/SPI/UART är inte längre direkt knutna till ESP32-specifika `Wire`/`SPI`/`HardwareSerial`.

Tills vidare är GPS-/kompassdata tillgängliga via `Autopilot::getGpsSensor()`/`getMagnetometerSensor()` och i `GET /api/status`, plus en engångsinställning av den initiala giren från kompassen vid start – **men de deltar inte i styrningen**. Öppna frågor för att avsluta fasen: ansluta GPS-modulen till ESP32-S3 (UART2-stiften är redan reserverade), kalibrera kompassen på den monterade farkosten och lägga till lutningskompensation för kursen. På ESP32-C3 är en fullvärdig GPS omöjlig – det finns inte tillräckligt med GPIO:er för TX (se stiftbeläggningen i `DEVELOPER_GUIDE.md`).

### Fas 4 – Flygning längs vägpunkter (waypoint navigation) [nästa steg]

**Status:** primitiverna är klara – `Guidance::rollForCourse`, en cirkel runt en punkt, återflygning till en punkt (RTH), en MAVLink-kanal för att ladda upp ett uppdrag (just nu svarar farkosten ärligt ”0 punkter” när den blir tillfrågad om ett uppdrag). Det som återstår: lagring av rutten, övergångar mellan punkter, MAVLink-protokollet MISSION_*.

**Mål:** farkosten flyger en given uppsättning koordinater utan att en operatör är inblandad på varje sträcka av rutten. **Hur detta passar in i arkitekturen:** det är ett nytt `AutopilotMode` i `Autopilot.h`, vid sidan av de befintliga MANUAL/STABILIZE/AUTO_TAKEOFF/ALT_HOLD – det vill säga, mekanismen för lägesbyte (via RC-platser och via `POST /api/setmode`) ändras inte; ett femte läge läggs till, som tar kurs och avstånd från GPS:en (fas 3) i stället för manuell inmatning via RC. **Vad som behövs tekniskt:** en algoritm för att beräkna kursen mot en punkt och logiken för att förflytta sig mellan ruttens punkter, plus ett sätt att ladda upp själva rutten till farkosten (den naturliga kandidaten är en utvidgning av samma HTTP-API som redan används för att styra lägena och PID).

### Fas 5 – Telemetri med lång räckvidd [klar i STM32-firmwaren]

**Status:** på STM32H743 – MAVLink 2 över ett radiomodem (SiK, ELRS i MAVLink-läge): attityd, position, hastighet, läge, PID-parametrar, lägesbyte från marken. Ramarna har kontrollerats mot referensen pymavlink. ESP32 har ingen ledig UART – den använder Wi-Fi-panelen. Nedan följer den ursprungliga texten för fasen.

**Mål:** en länk farkost↔mark på avstånd som är relevanta för verklig leverans, inte för bänken. **En ärlig bedömning av nuläget:** Wi-Fi-åtkomstpunkten i `WebDebugServer` överför redan i dag den fullständiga sammanställda statusen och styrkommandon, men räckvidden för en vanlig Wi-Fi-AP är tiotals meter, vilket räcker för felsökning på ett bord eller på ett flygfält, men inte för en autonom rutt bortom synhåll. **Vad som behövs tekniskt:** en separat radiokanal med längre räckvidd (till exempel en LoRa-modul eller ett dedikerat telemetriradiomodem) som transport för samma dataformat som redan definieras i `GET /api/status` – det vill säga, att ersätta eller komplettera transportlagret, inte att skriva om telemetriformatet.

### Fas 6 – Ett fullvärdigt grafiskt gränssnitt för markstyrning [delvis: QGroundControl / Mission Planner]

**Status:** tack vare MAVLink ser vanliga markstationer redan farkosten (karta, hempunkt, instrument, lägen under ArduPlane-namn). En egen markstation för en flotta är fortfarande ett mål. Nedan följer den ursprungliga texten för fasen.

**Mål:** en station för uppdragsplanering med karta, levande telemetri och flotthantering, snarare än en felsökningssida för en enda farkost. **Hur detta passar in i arkitekturen:** `WebDebugServer.h` är redan i dag ingen stubbe utan en fungerande webbserver med en sammanställd JSON-status och ett kommando-API (se tabellen i avsnitt 4.4); det är utgångspunkten, inte något som måste kastas. Nästa steg är en karta med den aktuella positionen (efter fas 3), visning och uppladdning av en rutt (efter fas 4), drift över en radiokanal med lång räckvidd (efter fas 5) och skalning av gränssnittet från en farkost till flera. Mer i avsnitt 6.

### Fas 7 – En mekanism för lastsläpp och skyddsåtgärder för leverans [klar i firmwaren, ännu inte flugen]

**Status:** lastsläpp (AUX1-servot, funktionen `PAYLOAD_DROP` på valfri brytare), geofence (radie och tak → RTH), hemflygning vid förlorad förbindelse, summern ”modellen borttappad”. Nedan följer den ursprungliga texten för fasen.

**Mål:** att förvandla plattformen från ”ett flygplan som flyger autonomt” till ”ett flygplan som levererar autonomt”. **Vad som behövs tekniskt:** ett extra servo för mekanismen som släpper/fördelar lasten, styrt enligt samma princip som de andra utgångarna i `FlightOutputs.h`; och skyddsåtgärder som är specifika för leverans snarare än för neutral flygning – geofence (en begränsning av flygområdet) och automatisk återgång till startpunkten vid förlorad förbindelse (i dag stänger `FlightController` av motorn och går över till glidflykt med plana vingar när signalen förloras i luften, vilket är rätt för ett manuellt fluget glidplan, men för autonom leverans är det logiska nästa steget att återvända till basen med GPS i stället för att bara glida).

### Fas 8 – Skalning till en flotta

**Mål:** att hantera flera farkoster samtidigt – uppgiftsfördelning, en driftpanel, flyghistorik. Det är den nivå där projektet slutar vara bara en ingenjörsmässig hobbyprototyp och blir ett operativt verktyg, intressant som affärsproblem: ruttplanering för flera farkoster, en uppgiftskö, status för varje farkost i realtid. Tekniskt är detta en överbyggnad över faserna 3–6 (GPS, telemetri, gränssnitt) – i praktiken samma `WebDebugServer`-API, men utvidgat till många telemetrikällor i stället för en.

## 6. Gränssnitt: från en felsökningssida till en markstation

Den här sektionens huvudtes: gränssnittet behöver inte byggas från grunden – det finns redan delvis och fungerar. I dag gör `WebDebugServer.h` följande:

- levererar en enda sammanställd JSON-ögonblicksbild av farkostens tillstånd (`GET /api/status`) – RC-kanalerna, flaggorna armed/failsafe, tillståndet för varje utgång, tillståndet för varje sensor (ärligt, med separata flaggor `attached` och `available`) och autopilotens tillstånd;
- tar emot styrkommandon i realtid (lägesbyte, ändring av PID) utan omflashning;
- levererar en färdig HTML-panel med levande kanalstaplar och styrknappar.

Vägen till en fullvärdig markstation är en stegvis utvidgning av det protokoll som redan fungerar, inte ett byte av arkitektur:

1. Lägg till en karta och den aktuella positionen i panelen – det kräver GPS (fas 3) som ytterligare ett fält i samma JSON-status.
2. Lägg till byggande och uppladdning av rutter – det kräver ett vägpunktsläge (fas 4) och en utvidgning av POST-API:t i likhet med `/api/setmode`/`/api/setpid`.
3. Flytta transporten från Wi-Fi-AP:n till en länk med lång räckvidd (fas 5), med samma meddelandeformat så att det befintliga gränssnittet inte behöver skrivas om.
4. Skala gränssnittet från en farkost till flera telemetrikällor (fas 8).

Med andra ord: den del av den framtida markstationen som är mest riskabel i fråga om ”kommer den att behöva skrivas från grunden” – serialiseringen av farkostens tillstånd och kommando-API:t – är redan implementerad och verifierad live på kortet.

## 7. Ärliga begränsningar – vad som ännu inte fungerar

För att färdplanen inte ska läsas som marknadsföring redovisar vi separat vad som ännu inte har gjorts eller bara har gjorts delvis:

- Autopiloten har verifierats på bänken, med 387 automatiska tester och simuleringar i sluten slinga, men den har aldrig provats under flygning – hittills har bara manuell styrning flugit (på den första prototypen). Flygplansmodellen i simuleringarna är förenklad, och koefficienterna är startvärden.
- De nya sensorerna (LSM6DSV, ICM-45686, QMC6309, SPL06, BMP581) har verifierats med registeremulatorer byggda efter databladen; på hårdvara ännu inte.
- STM32H743: hela firmwaren körs på en dator ovanpå STM32duino-attrapper; det finns ännu inget fysiskt kort.
- Horisonten för stabiliseringen är farkostens attityd vid start (IMU:n kalibreras vid varje start), eller monteringskalibreringen från tre positioner.
- Den fysiska anslutningen av ett servo syns inte för mjukvaran; det enda som syns är att pulsen verkligen kommer ut på stiftet (en självkontroll från konsolen).
- Stiftbeläggningen för den vanliga ESP32 med 38 stift valdes utifrån kretsens dokumentation och har inte verifierats på hårdvara.
- Licensen är [OpenPlane License](LICENSE.md): MIT med obligatorisk angivelse av upphovspersonen och förbud mot militär användning och mot avsiktlig skada på människor och egendom utan deras medgivande. På grund av dessa förbud räknas den inte som ”open source” i OSI:s mening.

## 8. Öppna frågor – en inbjudan till diskussion

Nedan följer de punkter där projektet ännu inte har något svar, avsiktligt formulerade som frågor till en potentiell partner eller investerare snarare än som fastställda fakta:

- Finansieringsmodellen och dess storlek – öppet för diskussion; det finns för närvarande inga konkreta summor eller datum, och inga kommer att hittas på i det här dokumentet.
- Projektets juridiska form (ett bolag, en stiftelse, en ren open source-gemenskap) – öppet för diskussion med dem som är intresserade av partnerskap.
- Teamets sammansättning – tills vidare drivs projektet öppet och är öppet för deltagande; inga specifika roller eller åtaganden slås fast i förväg.

Om något av detta är viktigt för dig som potentiell partner är rätt ställe att tala om det projektets GitHub Discussions (se avsnitt 9), inte antaganden i det här dokumentet.

## 9. Hur man tar kontakt och deltar

Projektets enda officiella kanal i dag är GitHub-repositoryt:
[github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject)
(gren `main`). Det finns för närvarande inga andra kontaktvägar (e-post, sociala nätverk, en juridisk person), och de anges avsiktligt inte här för att inte vilseleda.

- **Issues** – rapportera ett fel, föreslå en konkret teknisk ändring, rapportera resultaten av ett flygprov med ditt eget exemplar av prototypen.
- **Discussions** – diskutera färdplanen, partnerskap, användning i en specifik uppgift (leverans, sök och räddning, jordbruk) samt licens- och finansieringsfrågorna från avsnitt 8.
- **Pull requests** – lägg till en ny sensor via gränssnittet `Sensor`, ett nytt kort via ett block i `Config.h`, ett nytt `AutopilotMode`, förbättringar av webbpanelen – arkitekturen är utformad så att detta kan göras utan att röra kärnan.

Om du läser det här dokumentet som potentiell investerare eller partner: nästa meningsfulla steg är inte att skriva under något, utan att öppna en Discussion i repositoryt med en konkret fråga eller ett förslag. Färdplanen ovan är en inbjudan att diskutera den fas för fas, med full tillgång till den kod den bygger på.
