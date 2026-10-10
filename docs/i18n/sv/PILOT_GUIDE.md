# Pilothandbok för OpenPlaneProject

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../PILOT_GUIDE.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Det här är en praktisk handbok om ”vad som kopplas vart och hur man flyger” för dem som håller i en lödkolv och en sändare snarare än läser källkod. Vill du förstå kodens arkitektur, se de andra dokumenten i repositoryt. Här handlar det bara om hårdvara, kanaler, firmware och flygning.

Repository: https://github.com/damir-lebedev/OpenPlaneProject, gren `main`.

Låt oss vara ärliga från början: projektet är under aktiv utveckling och är **ingen färdig produkt**. Den första prototypen har redan flugit, men med förbehåll som beskrivs i ett eget avsnitt längre ned. Läs det före flygningen, inte efter.

---

## Innehåll

1. [Vilken hårdvara du behöver](#vilken-hårdvara-du-behöver)
2. [Val av kort och stiftbeläggning](#val-av-kort-och-stiftbeläggning)
3. [Anslutning av mottagaren](#anslutning-av-mottagaren)
4. [RC-kanalkarta](#rc-kanalkarta)
5. [Autopilotkanaler](#autopilotkanaler)
6. [ARM och failsafe](#arm-och-failsafe)
7. [Flashning av kortet](#flashning-av-kortet)
8. [Webbpanel på fältet](#webbpanel-på-fältet)
9. [Svart låda](#svart-låda)
10. [Checklista före flygning och säkerhet](#checklista-före-flygning-och-säkerhet)
11. [Felsökning](#felsökning)
12. [Aktuellt läge för prototypens flygplansstomme](#aktuellt-läge-för-prototypens-flygplansstomme)

---

## Vilken hårdvara du behöver

Satsen i det nuvarande bygget (firmwaren har verifierats på den):

- **Ett STM32H743 DevEBox H743-kort** (MCUDEV) – huvudkortet. Det tidigare huvudkortet, ESP32-S3 nedan, stöds också.
- **Ett ESP32-S3 N16R8-kort** (en DevKitC-1-klon med två USB-C-portar: ”USB” och ”COM”).
- **En FS-i6-sändare + FS-iA6B-mottagare** (iBUS-protokoll, 10 kanaler). Du behöver en datatråd – iBUS SERVO-porten. Sändaren måste kunna ställa in failsafe – den inställningen är obligatorisk, se avsnittet om failsafe.
- **2 MG90S-servon** för skevrodren – ett för varje vinghalva (två oberoende servon, inte ett för båda vingarna).
- **1 MG90S-servo** för höjdrodret.
- **1 MG90S-servo** för sidrodret – landningsställets styrhjul sitter på samma axel (styrning på marken).
- **Ett elektroniskt fartreglage (ESC)** 60–80 A med en 5 V BEC (BEC:en driver servona och mottagaren).
- **En D3548 1100KV-motor** + **en 10x5-propeller**.
- **Ett 3S LiPo-batteri**.

Autopilotsensorer (alla på I2C; utan dem flyger planet i manuellt läge):

- **GY-521** – gyroskop + accelerometer (kortet kan ha en MPU6050 eller, som i vårt fall, en MPU6500 – båda stöds).
- **BMP581** – barometer (den tidigare BMP388 stöds också).
- **GY-273** – kompass (vår har en QMC5883P; QMC5883L stöds också).
- Valfritt en **OLED 128×64 SSD1306** (I2C) – en statusskärm ombord.

Flygplansstomme: spännvidd 1200 mm, korda 250 mm, profil NACA 4412, konstruktion i PETG (3D-utskriven). Den första prototypen flög med en ESP32-C3, en D2212 1000KV-motor och en 40 A ESC.

---

## Val av kort och stiftbeläggning

Firmwaren stöder fyra kort; bytet kräver en byggparameter (`pio run -e <miljöns namn>`). Varje kort har sin egen stiftbeläggning, hårdkodad i firmwaren för den specifika miljön – flytta inte om trådar på eget bevåg; titta i tabellen för ditt kort.

> **Viktigt:** huvudkortet är nu **STM32H743 (DevEBox H743)** – uppstart, USB-konsolen, SD-kortet, iBUS, servona och motorn har verifierats på det; sensorerna kopplas in för första gången. **esp32-s3 (N16R8)** är det tidigare huvudkortet; dess stiftbeläggning har verifierats på bänken med alla sensorer. **esp32-c3** är den gamla prototypen som har flugit. Stiftbeläggningen för **esp32-dev** valdes utifrån kretsens dokumentation och **har inte provats på riktig hårdvara**.

### STM32H743 (DevEBox H743) – huvudkortet

`pio run -e stm32h743-devebox`, ett MCUDEV DevEBox H743-kort (STM32H743VIT6). Det här är standardkortet (`default_envs = stm32h743-devebox`). Uppstart, USB-konsolen, SD-kortet och den svarta lådan, iBUS-mottagning, ARM, servon och motorn från sändaren har redan verifierats på det; sensorerna kopplas in för första gången.

| Syfte | Stift |
|---|---|
| Skevroder, vänster vinghalva | PA0 |
| Skevroder, höger vinghalva | PA1 |
| Höjdroder | PA2 |
| ESC (gas) | PA3 |
| Sidroder + styrhjul | PD14 |
| iBUS från mottagaren (RX) | PE7 |
| Sensorernas I2C SDA / SCL (MPU, BMP581, kompass) | PB11 / PB10 |
| OLED I2C SDA / SCL (separat buss) | PB9 / PB8 |
| GPS: RX (← GPS TX) / TX (→ GPS RX) | PD9 / PD8 |
| MAVLink-telemetri (radiomodem): RX / TX | PD0 / PD1 |
| AUX1 / AUX2 (servon), summer | PD15 / PE9, PE15 |
| Sensorernas SPI SCK / MISO / MOSI, IMU CS / barometer CS | PB13 / PB14 / PB15, PB12 / PD10 |
| Batteri / strömsensor (ADC, registreras av den svarta lådan) | PC0 / PC1 |

Konsolen, loggen och hämtningen från den svarta lådan går via kortets USB-C (en virtuell COM-port). Håll fria: PA11/PA12 (USB), PA13/PA14 (SWD), PC8–PC12 och PD2 (µSD-platsen), PE3 och PC5 (knapparna K1/K2).

Anslutning av sensorerna på bänken (alla moduler drivs med **3,3 V**, inte 5 V):

| Modul | Stift |
|---|---|
| MPU-6050 / GY-521 (kortet kan ha en MPU6500 – det går bra) | VCC–3.3V, GND–GND, SCL–PB10, SDA–PB11, AD0–GND, INT/XDA/XCL – lämna oanslutna. En fristående MPU-6500-modul (10 stift): samma sak, plus **NCS–3.3V** (annars går kretsen över till SPI) och FSYNC–GND; EDA/ECL – lämna oanslutna. Kretsen uppåt, X-pilen mot nosen; rotationen av kretsens axlar är `IMU_ROTATION_CW_DEG` i `Config.h` (90 på vår klon) |
| BMP581 | VCC–3.3V (**endast 3.3V**: många moduler har ingen egen regulator), GND–GND, SCL–PB10, SDA–PB11, **SDO–GND** (adress 0x46; lämna den inte flytande), **CSB–3.3V** (annars går kretsen över till SPI), INT – lämna oansluten |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–PB10, SDA–PB11, DRDY – lämna oansluten. Håll den borta från servo-, ESC- och motortrådarna |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–PB8, SDA–PB9 |

Servona drivs **inte från kortet** utan från ESC:ns BEC (eller från en separat 5 V-källa på minst 2 A); alla källor har gemensam jord. Anslut inte ESC:ns röda tråd till kortets 5 V medan USB är inkopplad.

### esp32-s3 (N16R8) – det tidigare huvudkortet, verifierat på bänken

`pio run -e esp32-s3`, kort `esp32-s3-devkitc-1` med inställningar för N16R8-modulen (16 MB flash, 8 MB oktal PSRAM).

| Syfte | GPIO |
|---|---|
| Skevroder, vänster vinghalva | GPIO4 |
| Skevroder, höger vinghalva | GPIO5 |
| Höjdroder | GPIO6 |
| ESC (gas) | GPIO7 |
| Sidroder + styrhjul | GPIO18 |
| iBUS från mottagaren (RX) | GPIO17 |
| Sensorernas I2C SDA / SCL (MPU, BMP581, kompass) | GPIO41 / GPIO42 |
| OLED I2C SDA / SCL (separat buss) | GPIO1 / GPIO2 |
| Reserverat: GPS RX / TX | GPIO39 / GPIO40 |
| Reserverat: AUX1 / AUX2 (servon), AUX3, summer, LIGHT | GPIO15 / 16, 47, 38, 21 |
| Batteri / strömsensor (ADC, registreras av den svarta lådan); reserverat: telemetri TX / RX | GPIO8 / GPIO3, GPIO9 / GPIO10 |
| Endast bänk: SPI (ICM42688) SCK / MISO / MOSI / CS | GPIO12 / 13 / 11 / 14 (+ BMP388 CS – GPIO21) |

> Sensorbussen satt tidigare på GPIO8/9 – den flyttades till 41/42 för att stämma med flygkontrollerkortets layout.
> På bänken: SDA 8→41, SCL 9→42.

Använd inte: GPIO0/45/46 (startläget beror på dem), 19/20 (USB), 26–32 (flash), 33–37 (PSRAM på N16R8), 43/44 (”COM”-kontakten), 48 (RGB-lysdioden). De fria stiften är redan fördelade som reserver – ett bärarkort med kontakter för framtiden: [`FC_BOARD.md`](FC_BOARD.md).

Anslutning av sensorerna på bänken (alla moduler drivs med **3,3 V**, inte 5 V):

| Modul | Stift |
|---|---|
| MPU-6050 / GY-521 (kortet kan ha en MPU6500 – det går bra) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, AD0–GND, INT/XDA/XCL – lämna oanslutna. En fristående MPU-6500-modul (10 stift): samma sak, plus **NCS–3.3V** (annars går kretsen över till SPI) och FSYNC–GND; EDA/ECL – lämna oanslutna. Kretsen uppåt, X-pilen mot nosen; rotationen av kretsens axlar är `IMU_ROTATION_CW_DEG` i `Config.h` (90 på vår klon) |
| BMP581 | VCC–3.3V (**endast 3.3V**: många moduler har ingen egen regulator), GND–GND, SCL–GPIO42, SDA–GPIO41, **SDO–GND** (adress 0x46; lämna den inte flytande), **CSB–3.3V** (annars går kretsen över till SPI), INT – lämna oansluten |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, DRDY – lämna oansluten. Håll den borta från servo-, ESC- och motortrådarna |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–GPIO2, SDA–GPIO1 |

Servona drivs **inte från kortet** utan från ESC:ns BEC (eller från en separat 5 V-källa på minst 2 A); alla källor har gemensam jord. Anslut inte ESC:ns röda tråd till kortets 5 V medan USB är inkopplad.

### esp32-c3 – den gamla prototypen, har flugit

`pio run -e esp32-c3`, kort `esp32-c3-devkitm-1`.

| Syfte | GPIO |
|---|---|
| Skevroder, vänster vinghalva | GPIO5 |
| Skevroder, höger vinghalva | GPIO4 |
| Höjdroder | GPIO6 |
| ESC (gas) | GPIO7 |
| Sidroder | – (inga lediga stift) |
| iBUS från mottagaren (RX) | GPIO8 |
| I2C SDA (sensorer) | GPIO1 |
| I2C SCL (sensorer) | GPIO3 |

### esp32-dev (den vanliga klassiska ESP32, 38 stift) – för bänken och felsökning, HAR INTE FLUGIT

`pio run -e esp32-dev`, kort `esp32dev`.

| Syfte | GPIO |
|---|---|
| Skevroder, vänster vinghalva | GPIO13 |
| Skevroder, höger vinghalva | GPIO14 |
| Höjdroder | GPIO27 |
| ESC (gas) | GPIO26 |
| Sidroder | GPIO25 |
| iBUS från mottagaren (RX) | GPIO16 |
| I2C SDA (sensorer) | GPIO21 |
| I2C SCL (sensorer) | GPIO22 |

Fördelen är att det är det vanligaste och billigaste kortet i familjen – lämpligt för felsökning av firmwaren på bänken, men dess stiftbeläggning har inte provats på hårdvara.

---

## Anslutning av mottagaren

Allt du behöver från mottagaren är **en iBUS-datatråd**, som på de flesta FlySky-kompatibla mottagare är utdragen till en separat port (ofta märkt ”iBUS”, eller så är det den enda utgången som inte är PPM). Anslutning:

- **Mottagarens TX (iBUS-utgång)** → kortets **RX-stift** från tabellen ovan (GPIO8 på esp32-c3, GPIO17 på esp32-s3, GPIO16 på esp32-dev).
- **Mottagarens jord (GND)** → kortets **GND**. Det är obligatoriskt; utan gemensam jord fungerar inte protokollet.
- **Mottagarens strömförsörjning** – från en separat BEC/regulator eller från kortets 5 V, beroende på hur du brukar driva mottagaren i dina byggen; den är inte knuten till något särskilt stift i firmwaren.

Firmwaren skickar ingenting tillbaka till mottagaren – den bara lyssnar, så kortets TX-ledning behöver inte anslutas någonstans.

iBUS-portens hastighet i firmwaren är 115200 baud, vilket är standard för protokollet; det finns ingen anledning att ändra den, eftersom mottagaren själv håller den hastigheten.

---

## RC-kanalkarta

Kartan verifierades på bänken med en FS-i6-sändare (10 kanaler, mode 2) och en FS-iA6B-mottagare.

| Kanal | Reglage på sändaren | Namn | Vad den gör |
|---|---|---|---|
| CH1 | höger spak ←→ | AILERON | Roll – skevroder (2000 = höger) |
| CH2 | höger spak ↑↓ | ELEVATOR | Tippning – höjdroder (2000 = spaken framåt, nosen ned) |
| CH3 | vänster spak ↑↓ | THROTTLE | Gas. 1000 µs = av, 2000 µs = max, ingen begränsning |
| CH4 | vänster spak ←→ | RUDDER | Sidroder och landningsställets styrhjul (ett servo) |
| CH5 | SwA | ARM | ARM-brytaren – se avsnittet om ARM nedan |
| CH6 | SwB | SWB | Klaffar som standard: ned, mot dig – utfällda, upp – infällda (se nedan) |
| CH7 | SwC (3 lägen) | SWC | Läge som standard: upp MANUAL, mitten STABILIZE, ned AUTO_TAKEOFF |
| CH8 | SwD | SWD | RTH (hem) som standard, så länge den är påslagen |
| CH9 | VrA | VRA | Stabiliseringens styrka som standard |
| CH10 | VrB | VRB | Marschfart som standard |

CH6–CH10 kan vara **vad som helst, på en rad** i `include/config/Controls.h`: vilket som helst av de 12 lägena, 10 funktionerna (klaffar, broms, lastsläpp, geofence, summer…) och 7 rattarna. När kortet startar skrivs den faktiska fördelningen ut i seriemonitorn. Allt om lägen och bindningar finns i [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

Alla kanalvärden använder mottagarens standardpulsområde 1000–2000 µs, med 1500 µs som mitt/neutralläge.

### Klaffar (flaperoner)

Det finns inga separata klaffar – skevrodren fyller deras funktion. **SwB ned** (mot dig): båda skevrodren sänks mjukt, på ungefär en sekund, med samma vinkel (`FLAPS_DEPLOYED_US` = 220 µs ≈ 20° rotation av servoarmen – ändras i `include/config/Config.h`). Det ökar lyftkraften för start och landning i lägre fart. Roll från spaken och från autopiloten fungerar som vanligt – skevrodren rör sig åt motsatta håll, men nu runt det sänkta läget. **SwB upp** – de fälls in lika mjukt. På OLED-skärmen tänds `FL` på första raden medan klaffarna är utfällda.

Vid full roll med utfällda klaffar når det skevroder som rör sig nedåt sitt ändläge före det som rör sig uppåt – det är normalt och fungerar som skevroderdifferential.

Att fälla ut klaffarna lyfter oftast nosen – var beredd att trycka spaken lite framåt; om effekten är stark, justera genom att minska `FLAPS_DEPLOYED_US`.

---

## Autopilotkanaler

Kort sagt – standardfördelningen (`include/config/Controls.h`):

| Brytare | Vad den gör |
|---|---|
| **SwC** (CH7) | upp **MANUAL** · mitten **STABILIZE** · ned **AUTO_TAKEOFF** |
| **SwD** (CH8) | **RTH** – hem, så länge den är påslagen |
| **SwB** (CH6) | klaffar |
| **VrA / VrB** (CH9/10) | stabiliseringens styrka / marschfart |

- **STABILIZE – ”spaken anger vinkeln”.** Fullt spakutslag är 45° roll och 25° tippning; släpp spaken och flygplanet rätar upp sig självt. Gasen är din.
- **AUTO_TAKEOFF.** Efter ARM händer ingenting förrän du själv höjer gasen över hälften. Sedan: 0–1 s – gasen mjukt upp till 100 %, vingarna plana; 1–3 s – tippning +15°; därefter +10° tills du slår om SwC. Gas = det högsta av spaken och programmet.
- **RTH.** Kurs mot ARM-punkten, höjd 40 m, cirklar över hempunkten. Kräver GPS med 3D-fix **före ARM**.

Alla 12 lägen (ALT_HOLD, ACRO, CRUISE, LOITER, hand-LAUNCH, AUTO_LAND, SOARING, RESCUE…), vilka sensorer vart och ett kräver och hur det läggs på en brytare finns i [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md). Ett läge som valts från panelen eller markstationen gäller tills du slår om lägesbrytaren.

Stabiliseringen rör roderytorna även när planet **inte är armerat** – så kan du på bänken se åt vilket håll det reagerar på en lutning. Integratorn ackumulerar inte i det fallet.

Vinkeltecken (som på OLED-skärmen och i loggen): **roll R > 0 – höger vinge ned, tippning P > 0 – nosen upp.**

---

## ARM och failsafe

### ARM-procedur

- **ARM:** gasen (CH3) i botten → brytaren **SwA (CH5) ned, mot dig** (på FS-i6 är det CH5 = 2000; upp = 1000). `ArmingManager: ARM` visas i Serial och `ARMED` på OLED-skärmen.
- **DISARM:** SwA upp, bort från dig – omedelbart, när som helst; motorn stannar direkt.
- Om SwA förs till ARM med gasen inte i botten, eller om kontrollerna före flygning inte har gått igenom, sker **ingen** ARM (orsaken skrivs ut i Serial). Du måste föra tillbaka SwA uppåt, sänka gasen och föra den nedåt igen.
- Om kortet startades med SwA redan i ARM-läget (ned) armeras det **inte**: firmwaren måste först se SwA uppe (OFF).

**ARM blockerar verkligen gasen:** så länge planet inte är armerat hålls ESC:ns gas tvångsmässigt på minimum oavsett spak. Före armering kontrollerar firmwaren att de sensorer som det valda läget kräver svarar – till exempel armeras inte STABILIZE utan en levande IMU förrän du byter till MANUAL. Bete dig ändå som om propellern kan börja snurra när som helst efter ARM.

### Failsafe

Förlorad förbindelse har **absolut prioritet** över allt annat:

- **planet är armerat, GPS och hempunkt finns** – **hemflygning med motor** (som i ArduPilot/INAV): kurs mot hempunkten, höjd 40 m, cirklar över hempunkten tills förbindelsen återkommer. OLED-skärmen visar `FSRTH`. Stängs av med `FAILSAFE_RTH = false` i `Config.h`;
- **planet är armerat, ingen GPS** – **glidflykt**: motorn är av, autopiloten håller i vilket läge som helst, även MANUAL, vingarna plana och nosen något under horisonten (−3°), och klaffarna fälls in. OLED-skärmen visar `RX LOST ... GLIDE`. Om du sätter `FAILSAFE_GLIDE_ROLL_DEG` = 10–20 kommer flygplanet att cirkla ovanför dig i en flack spiral;
- **inte armerat** (på marken) eller IMU:n svarar inte – motorn är av, skevrodren, höjdrodret och sidrodret går till neutralläge (1500 µs); efter 10 s utan förbindelse piper ”jag är här”-summern (om den är monterad).

Firmwaren känner igen förlorad förbindelse på två sätt:

1. **Inga iBUS-ramar på mer än 500 ms** – en bruten tråd eller en mottagare utan ström.
2. **Gas under 950 µs** – det är så mottagaren rapporterar att den har tappat sändaren. **Det kräver att failsafe ställs in i sändaren** (se nedan): när förbindelsen bryts slutar FS-iA6B INTE skicka ramar utan upprepar de senaste spakvärdena – utan inställningen ser firmwaren inte att förbindelsen är bruten, och flygplanet fortsätter flyga med den senaste gasen.

ARM återställs **inte** vid failsafe: när förbindelsen återkommer lyder flygplanet åter spakarna och det valda läget utan ny armering (att slå om brytaren i luften med gasen på noll är farligare).

Kontroll på bänken (propellern borttagen): ARM → stäng av sändaren → OLED-skärmen visar `RX LOST ... GLIDE`, motorn har stannat; luta flygplanet – roderytorna ska föra tillbaka det till plant läge. Slå på sändaren – `RX ok`, och styrningen är tillbaka hos spakarna.

### Inställning av failsafe i FS-i6-sändaren (obligatoriskt, en gång)

Idén: när förbindelsen bryts ska mottagaren ge en gas på ~900 µs – under det normala minimumet 1000.

1. `Menu → Functions setup → End points` → kanal 3: sätt den nedre punkten (det vänstra värdet) till **120 %**. Spara (långt tryck på Cancel).
2. Gas – **helt i botten**.
3. `Menu → Functions setup → Failsafe` → Channel 3 → **On**, gasen fortfarande i botten → spara med ett långt tryck på Cancel. Mottagaren kommer ihåg ~900 µs.
4. Gå tillbaka till `End points` → kanal 3 → sätt tillbaka den nedre punkten till **100 %**. Spara.
5. Kontroll: ARM behövs inte. Stäng av sändaren – efter ungefär 1 s visas `RX=LOST(failsafe пульта)` i Serial och en inverterad rad `RX LOST` på OLED-skärmen. Slå på sändaren – `RX=OK`.

Håll gastrimmet i mitten: med trimmet långt nedsänkt kan gasen hamna under 950 och firmwaren tar det för förlorad förbindelse.

Här fanns tidigare en **boost på CH8** och en gasbegränsning på 40 % – de har tagits bort: begränsningen skyddade ett svagt 3S1P-bygge, och nya batterier är inte rädda för full gas. Boosten brukade också slå på sig själv om SwD var uppe när kortet startades.

---

## Flashning av kortet

Firmwaren byggs med **PlatformIO** (Arduino-ramverket, C++).

### Installation av PlatformIO

Det enklaste är att installera tillägget **PlatformIO IDE** i VS Code (Extensions → sök ”PlatformIO IDE” → Install); då får du både CLI:t och praktiska byggknappar i gränssnittet. Du kan också installera det med `pip install platformio` och arbeta från terminalen – båda alternativen använder samma `pio`-kommandon.

### Bygga och ladda upp

Öppna projektet (repositoryts mapp) i VS Code med PlatformIO installerat, anslut kortet via USB och kör kommandot för ditt kort i terminalen:

```bash
# STM32H743 DevEBox (huvudkort)
pio run -e stm32h743-devebox -t upload

# esp32-s3 N16R8 (tidigare huvudkort)
pio run -e esp32-s3 -t upload

# esp32-c3 (gammal prototyp)
pio run -e esp32-c3 -t upload

# esp32-dev (klassisk ESP32 38-pin, för bänken)
pio run -e esp32-dev -t upload
```

Om du inte anger `-e` alls byggs standardkortet – `stm32h743-devebox`.

**STM32 DevEBox:** den första flashningen går via USB DFU: en bygel BT0→3V3, tryck på RST, sedan kommandot ovan (Windows behöver WinUSB-drivrutinen för ”STM32 BOOTLOADER”, installerad med Zadig). Därefter startar tangenten `D` i konsolen om kortet i bootloadern av sig självt, och bygeln behövs inte längre. Konsolen går via samma USB-C.

**esp32-s3:** kortet har två USB-C-kontakter. Flashning och Serial går via **”COM”**-kontakten (en CH343-brygga; i Windows visas den som ”USB-Enhanced-SERIAL CH343”). ”USB”-kontakten (kretsens inbyggda USB) behövs inte för driften, men den får vara ansluten – den stör ingenting.

### Seriemonitor

För att se felsökningsutskrifterna (kanalernas tillstånd, ARM, sensorer) direkt i konsolen via USB:

```bash
pio device monitor -b 115200
```

Hastigheten måste vara 115200 – annars ser du oläsligt skräp i stället för text. Var tionde sekund skrivs en `SYS`-rad ut – slingans frekvens (den ska vara ~500 Hz), den genomsnittliga och sämsta slingtiden under 10 s, iBUS-räknarna, ledigt minne. Allt annat kommer via de kanaler som är påslagna i loggmenyn (se nedan).

### Konsol: meny och logg (seriemonitor)

Ett tangenttryck verkar direkt; Enter behövs inte (i en monitor som skickar radvis, skriv bokstaven + Enter). Kalibreringar och kontrollen av utgångar fungerar bara när planet inte är armerat.

| Tangent | Vad den gör |
|---|---|
| `h` | **huvudmeny** (textbaserad, punkterna är numrerade) |
| `l` | menyn ”vad som ska skrivas till loggen” |
| mellanslag | pausa loggen / återuppta |
| `s` | detaljerad status för alla sensorer (inklusive I2C-felräknarna) |
| `i` | kalibrera om gyroskopet och kör IMU-kontrollen före flygning – 2 s, rör inte flygplanet |
| `o` | **kalibrering av IMU-monteringen** – en gång, efter att kortet monterats i flygplanet (se nedan) |
| `m` | kompasskalibrering – rotera kortet/flygplanet runt alla axlar i 15 s. Resultatet sparas i flash och överlever en omstart |
| `p` | kontroll av utgångar: den faktiska pulsen på varje stift |

**Loggning per kanal.** Varje typ av data är en egen rad med eget prefix, och var och en har sitt eget läge: **av**, **vid ändring** (en rad visas bara när värdena verkligen har ändrats – spakdarrning och sensorbrus räknas inte) eller **kontinuerligt** (var 0,2 / 0,5 / 1 / 2 s – perioden ställs in i samma meny).

| Kanal | Vad den visar | Standard |
|---|---|---|
| `STAT` | förbindelse, ARM, läge, klaffar, om IMU/barometer är OK | vid ändring |
| `RC` | sändarkanaler, µs | av |
| `OUT` | utgångar till roderytorna och ESC:n, µs | av |
| `ATT` | roll, tippning, kurs | av |
| `AP` | autopilot: mål och korrigeringar | av |
| `ALT` | höjd, vertikal hastighet, ALT_HOLD-mål | av |
| `MAG` | kompasskurs | av |
| `GPS` | fix, satelliter, koordinater, hastighet | av |
| `IMU` | gyroskop och accelerometer | av |
| `SYS` | slingans frekvens och tid, minne (var 10:e s) | på |

Medan menyn är öppen är loggen tyst så att menyn inte rullar bort; när du lämnar den skrivs alla påslagna kanaler ut igen. Valet sparas i flash när du lämnar menyn och överlever en omstart. När planet är armerat verkar det direkt men skrivs först efter DISARM: en skrivning till flash stoppar flygslingan i ~0,4 s.

### IMU-montering: hur du vill, en kalibrering

Kortet med IMU:n kan monteras i flygplanet **hur det passar** – i vilken vinkel som helst, på sidan, upp och ned: firmwaren räknar själv ut var dess nos och ovansida är. Det görs **en gång** efter monteringen (och igen om kortet har flyttats):

1. Flygplanet på bordet, ingen sändare behövs, motorn inte armerad. Tryck `o` i seriemonitorn.
2. **Steg 1:** flygplanet står plant, som i planflykt (för ett flygplan med sporrhjul, lägg något under stjärten). Rör det inte i ~3 s.
3. **Steg 2:** höj **nosen** 30–60°, vingarna plana, och håll stilla i ~1 s.
4. **Steg 3:** nosen tillbaka ned, sänk den **högra vingen** 30–60° och håll i ~1 s.

Varje steg räknas av sig självt (loggen skriver ”засчитано”, det vill säga ”godkänt”). I slutet visas vad som kom fram (”нос = +Y чипа, верх = −Z чипа”, det vill säga ”nos = kretsens +Y, upp = kretsens −Z”) och ”установка сохранена” (”monteringen sparad”). Om du har blandat ihop något (sänkt nosen i stället för att höja den, vänster vinge i stället för höger) avvisas kalibreringen med en förklaring; upprepa bara `o`. Resultatet lagras i flash och överlever en omstart. Kontroll: nosen upp → P på OLED-skärmen blir positivt; höger vinge ned → R blir positivt.

Så länge ingen monteringskalibrering finns gäller den gamla metoden: kortet måste ligga med kretsen uppåt, och rotationen är `IMU_ROTATION_CW_DEG` i `Config.h`.

### Kontroll före flygning vid start

Vid varje start ägnar IMU:n ~2 s åt att kalibrera gyroskopet och kontrollerar samtidigt sig själv:

- **flygplanet står stilla** – om det hölls i händerna eller flyttades i det ögonblicket blir gyroskopets offset fel;
- accelerometern i vila visar 1g;
- **”upp” stämmer med monteringskalibreringen** – om kortet har flyttats eller vänts syns det direkt (ett flygplan på sporrhjulet eller i en sluttning är inget problem; toleransen är 45°).

**Slå på flygplanet medan det står stilla.** Det behöver inte stå plant om monteringen är kalibrerad (annars blir läget vid start horisonten). Resultatet syns i loggen: `предполётная проверка пройдена` (”kontrollen före flygning godkänd”) eller `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА — <причина>` (”KONTROLLEN FÖRE FLYGNING EJ GODKÄND” följt av orsaken). Om den misslyckades är **ARM i STABILIZE och AUTO_TAKEOFF förbjuden** (orsaken skrivs ut när du försöker), och autopiloten ger inga korrigeringar i något läge, inklusive glidflykt vid förlorad förbindelse: med fel vinklar skulle den styra åt fel håll. ARM i MANUAL är fortfarande möjlig. Så åtgärdar du det: ställ ned flygplanet orörligt och anslut batteriet på nytt (eller tryck `i`); om kortet har flyttats, tryck `o`.

### OLED-skärm

Om OLED-skärmen är ansluten (GPIO1/GPIO2) visar den följande 5 gånger per sekund:

```
RX ok disarm STAB        <- förbindelse / ARM / läge (vid förlorad förbindelse inverteras raden)
R  +1.2 P  -0.4          <- roll / tippning, °
Alt +0.3 Vz +0.1         <- höjd från startpunkten, m / vertikal hastighet, m/s
Hdg 123  Thr 1000        <- kompasskurs / gas till ESC:n, µs
L1500 R1500 E1500        <- PWM för skevrodren och höjdrodret, µs
Loop 500Hz max 1100us    <- frekvens och sämsta slingtid
```

---

## Webbpanel på fältet

Kortet startar en egen Wi-Fi-åtkomstpunkt – ingen router hemma eller internet behövs, och allt fungerar direkt på fältet från en telefon.

**Så ansluter du:**

1. Öppna listan över Wi-Fi-nätverk på en telefon eller bärbar dator.
2. Anslut till nätverket **`OpenPlane-Debug`**, lösenord **`12345678`**.
3. Öppna adressen **`http://192.168.4.1`** i en webbläsare.

Ingen app behöver installeras – det är en vanlig webbsida.

**Vad du kan göra från en telefon på fältet, helt utan programmering:**

- Se **levande staplar för alla 10 RC-kanaler** – praktiskt för att kontrollera att sändaren och mottagaren verkligen skickar det du rör på spaken, redan innan servona ansluts.
- Se status för varje utgång (vänster/höger skevroder, höjdroder, ESC) – om kanalen är ansluten i mjukvaran.
- Se om sensorerna (IMU, barometer) svarar, om du har dem monterade – den visar ärligt antingen riktiga data (roll/tippning/höjd) eller en tydlig notering om att sensorn fysiskt saknas eller inte svarar.
- **Byta autopilotläge** med knappar (manuellt / stabilisering / autostart / höjdhållning) direkt från sidan – utan sändaren.
- **Justera PID-regulatorns koefficienter** (för roll och tippning) via ett formulär på sidan – användbart för att stegvis trimma stabiliseringen utan att flasha om.

Räckvidden för den här åtkomstpunkten är i praktiken tiotals meter; det är vanlig ESP32-Wi-Fi, inte telemetri med lång räckvidd. Det är ett verktyg för justering på bordet, på bänken och bredvid fältet – inte för att styra planet under flygning på avstånd.

---

## Svart låda

ESP32-S3-firmwaren registrerar själv varje flygning i flash: allt som sensorerna såg, vad spakarna gjorde, vart servona gick och vad autopiloten bestämde. Det finns inget att göra:

- den **spelar in** från det ögonblick planet armeras och gasen höjs (plus 10 s dessförinnan);
- den **stannar** 10 s efter DISARM – eller om ett armerat plan står orörligt med motorn av i 30 s (det landade eller kraschade, och DISARM glömdes);
- förlorad förbindelse, motor på noll och glidflykt stoppar **inte** inspelningen.

Vid start visar seriemonitorn hur mycket utrymme det finns för en flygning (konsolen skriver på ryska; raden nedan betyder ”väntar på ARM och gas | raderat i förväg 12.9 MB (≈11 min) av 13.9 MB | flygningar 1”):

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

**Efter flygningen**, anslut USB (COM-kontakten), stäng seriemonitorn och hämta flygningen:

```bash
python tools/blackbox.py download
```

Flygningen hamnar i mappen `blackbox/` tillsammans med den avkodade utdatan: `summary.txt` (sammanfattningen och händelserna), `events.txt` och CSV-tabeller per sensor. Den svarta lådan raderar själv gamla flygningar när den behöver utrymme – **hämta efter varje flygning**. Konsolmenyn är tangenten `k`. Alla detaljer finns i [BLACKBOX.md](BLACKBOX.md).

**På STM32H743-kortet** spelar den svarta lådan in på ett SD-kort: kortet formateras som FAT32 och förbereds en gång på en dator (`python tools/blackbox.py sd-prepare E:`). Efter en flygning kan den hämtas via USB med samma `download`-kommando, eller så tar du ut kortet och avkodar filen direkt från det: `python tools/blackbox.py ring E:/BLACKBOX.BIN`.

---

## Checklista före flygning och säkerhet

Läs det här avsnittet i sin helhet **före** den första starten, inte efter en incident.

### Obligatoriskt före varje bänktest

- [ ] **Propellern är fysiskt BORTTAGEN** om du kontrollerar kanaler, ARM, panelen, justerar PID eller bara startar kortet för första gången med en ny stiftbeläggning. ESC:n kan rycka i motorn i vilket skede av ett test som helst – det är ingen hypotetisk risk utan normalt beteende vid första start.
- [ ] LiPo-batteriet har kontrollerats för svullnad och skador, laddats med en riktig LiPo-laddare och förvaras och laddas på ett obrännbart underlag.
- [ ] Sändaren är påslagen och dess kanaler har kontrollerats på panelen (`http://192.168.4.1`) **innan** batteriet ansluts till ESC:n.

### Före flygning

- [ ] Ett öppet område, utan människor eller byggnader inom en radie som räcker för ett flygplan med 1200 mm spännvidd under manuell styrning med onormalt beteende hos servona eller vingen (se avsnittet om prototypens begränsningar nedan – motorfästet och vingen har ännu inte förstärkts med kolfiber).
- [ ] Alla tre ytorna rör sig åt rätt håll – kontrollera på bordet före varje flygning och lita inte på minnet från förra gången:
  - höger spak åt höger → **höger skevroder upp, vänster skevroder ned**;
  - höger spak mot dig → **höjdrodret upp**;
  - vänster spak åt höger → **sidroder och hjul åt höger**;
  - SwB (klaffar) ned → **båda skevrodren mjukt ned**, och roll från spaken rör dem fortfarande åt motsatta håll;
  - i STABILIZE, luta flygplanet med höger vinge ned → **höger skevroder ned, vänster skevroder upp** (ytorna för tillbaka det till plant läge); nosen ned → **höjdrodret upp**.
  Om något är fel, ändra motsvarande `*_REVERSED` i `include/config/Config.h` (avsnittet ”Servo direction”), inte reverseringen på sändaren: annars blir spaken och autopiloten oense.
- [ ] Vid start stod flygplanet stilla, och loggen eller panelen visar inte `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА`; när du lutar flygplanet i händerna ändras P och R på OLED-skärmen åt rätt håll.
- [ ] Failsafe är inställt i sändaren och provat: stäng av sändaren → `RX LOST` på OLED-skärmen/i Serial (se avsnittet om failsafe).
- [ ] Sändarens räckvidd har kontrollerats, och sändarens batteri är laddat.
- [ ] Vid start visar loggen `BlackBox: ... стёрто впереди N МБ (≈M мин)` – det finns tillräckligt med utrymme för flygningen. Den föregående flygningen har hämtats (`python tools/blackbox.py download`), om du behöver den.
- [ ] Håll händer och ansikte borta från propellern så snart en LiPo är ansluten till ESC:n – efter ARM reagerar gasen på spaken direkt, utan fler varningar. DISARM – SwA upp.
- [ ] Se till att du fysiskt kan koppla bort strömmen snabbt (åtkomst till LiPo-kontakten) i stället för att bara förlita dig på failsafe vid signalförlust.

### Allmän LiPo-säkerhet

- Lämna aldrig en LiPo som laddas utan uppsikt.
- Anslut eller koppla inte bort LiPo:n från ESC:n medan du står i propellerns rotationsplan.
- Transportera och förvara LiPo-batterier i en skyddspåse eller låda.

---

## Felsökning

**”Ingen mottagarsignal” / RX ser bra ut, men kanalerna på panelen rör sig inte**
Kontrollera att mottagarens datatråd är ansluten exakt till iBUS RX-stiftet från ditt korts tabell (GPIO8/GPIO17/GPIO16), inte förväxlad med jord eller ström, och att mottagarens och kortets jord är sammankopplade. Om stiftbeläggningen stämmer och trådarna är hela men det fortfarande inte finns någon signal, kontrollera att mottagaren överhuvudtaget är bunden till sändaren och att mottagarens utgång är inställd på iBUS, inte PPM/SBUS.

**En sensor (IMU, barometer, kompass) visar ”svarar inte” / NO_RESPONSE**
Firmwaren rapporterar ärligt att sensorn inte svarar i stället för att mata ut nollor. Kontrollera: (1) modulens strömförsörjning – 3.3V och kortets jord; (2) SDA/SCL – på I2C-stiften för just ditt kort; (3) adressen på ledningen: MPU 0x68 (AD0 till GND), BMP581 0x46 (SDO till GND, CSB till 3.3V; 0x47 om SDO ligger på 3.3V), BMP388 0x76 (SDO till GND, **CSB till 3.3V** – annars är kretsen i SPI-läge), QMC5883P 0x2C, QMC5883L 0x0D. Om en sensor ibland svarar och ibland inte (eller svarar på någon annans adress) är det dålig kontakt på kopplingsdäcket: tryck till VCC/GND/SDA/SCL, och helst bör varje modul matas direkt från kortets 3.3V/GND. Kommandot `s` i konsolen visar I2C-felräknarna för varje sensor.

**Vinklarna på OLED-skärmen är förväxlade (nosen upp ändrar R, inte P) eller har fel tecken**
Gör kalibreringen av IMU-monteringen (`o`, se ”IMU-montering”) – den beror inte på hur kretsen är lödd på modulen eller hur modulen sitter i flygplanet. Utan den: på GY-521-kloner är kretsen ibland lödd vriden i förhållande till de tryckta pilarna – rotera axlarna i `Config.h` → `IMU_ROTATION_CW_DEG` (0/90/180/270). Kontroll: nosen upp → P blir positivt, höger vinge ned → R blir positivt.

**ARM nekad: ”IMU: ...”**
IMU-kontrollen före flygning gick inte igenom (se ”Kontroll före flygning vid start”): flygplanet rördes vid start – ställ ned det orörligt och anslut batteriet på nytt; ”upp” stämmer inte med kalibreringen – kortet har flyttats, gör `o`; ”kortet ligger inte med kretsen uppåt” – monteringen är inte kalibrerad, gör `o`.

**Ett servo eller ESC:n reagerar på fel spak / reagerar inte**
Kommandot `p` i konsolen mäter den verkliga pulsen på varje utgång (GPIO4–7) och jämför den med den förväntade. Om allt är ”OK” men servot inte rör sig ligger problemet utanför kortet: (1) servot får ingen ström (BEC/5V, gemensam jord); (2) signaltråden sitter på fel stift; (3) mekanismen har fastnat. ”НЕ СОВПАДАЕТ” (”STÄMMER INTE”) betyder att problemet finns i firmwaren eller kringutrustningen – rapportera det till utvecklaren.

**Motorn snurrar inte alls, fast gasen på OLED-skärmen/panelen följer spaken**
Kontrollera att LiPo:n är ansluten till ESC:n och att planet verkligen är armerat – före ARM hålls gasen till ESC:n tvångsmässigt på minimum, och det är inget fel. En ESC som såg en gas som inte var minimum vid start kan pipa hela tiden och vägra armera – anslut batteriet på nytt med gasen i botten. Om `ArmingManager: ARM` inte visas efter att SwA förts ned, titta i Serial – firmwaren skriver ut orsaken (gasen är inte i botten, en sensor svarar inte, en som behövs för det läge som för närvarande är valt på CH7); för tillbaka SwA uppåt, åtgärda orsaken och för den nedåt igen.

**Motorn stannar eller rycker vid uppvarvning**
Om ESC:n drivs från ett labbaggregat slår du i aggregatets strömgräns: även utan propeller drar motorn kortvarigt flera ampere, spänningen sjunker och ESC:n startar om. Höj strömgränsen eller använd en LiPo. Spikar från en sådan omstart kan låsa kortets USB-brygga (”COM”-porten slutar gå att öppna) – anslut kabeln på nytt.

**Efter flashning svarar inte kortet / åtkomstpunkten `OpenPlane-Debug` visas inte**
Se till att uppladdningen (`pio run -e <ditt kort> -t upload`) avslutades utan fel och att du flashade exakt den miljö du fysiskt håller i handen (esp32-c3 skiljer sig från esp32-s3 och esp32-dev inte bara i stiften utan också i kretsen – firmware för en annan krets installeras inte på kortet, eller installeras felaktigt). Kontrollera seriemonitorns utskrift (`pio device monitor -b 115200`) direkt efter att kortet startat om – den visar i vilket skede av setup() kortet befinner sig.

---

## Aktuellt läge för prototypens flygplansstomme

För att hålla förväntningarna ärliga:

- Den första prototypen **har redan flugit**. Problem upptäcktes: **otillräcklig hållfasthet i motorfästet** och **otillräcklig hållfasthet i vingen** – vingen behöver förstärkas med kolfiber. Servona behöver också justeras ytterligare. Ta hänsyn till detta när du planerar egna flygningar – det är ingen abstrakt friskrivning utan ett verkligt haveri som redan har inträffat på den här prototypen.
- **esp32-s3-bänken är byggd med alla sensorer** (GY-521 med en MPU6500, en BMP388 på I2C, en GY-273 med en QMC5883P, en OLED) – alla svarar, och slingan går på 500 Hz. Autopilotlägena har verifierats på bordet men **har ännu inte provats under flygning**.
- Standardbarometern är nu BMP581 (bänkens BMP388 verifierades live; BMP581 har ännu inte provats på hårdvara). Höjden är relativ till startpunkten. Absolut höjd över havet räknas fram ur standardatmosfären, utan väderkorrigering.
- QMC5883P-kompassen behöver kalibreras (`m` i konsolen) på det monterade flygplanet – bredvid motorn och trådarna skiljer sig offseten från dem på kopplingsdäcket. Kursen har ännu ingen lutningskompensation och används inte av något läge.
- Licensen är OpenPlane License: MIT med obligatorisk angivelse av upphovspersonen (Damir Lebedev), förbud mot militär användning och förbud mot att avsiktligt skada människor eller egendom utan deras medgivande, se [LICENSE](LICENSE.md).

Om du bygger ditt eget plan efter den här handboken, flyg det först med manuell styrning (MANUAL) och gå först därefter vidare till autopiloten.
