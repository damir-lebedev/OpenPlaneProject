# Flygkontrollerkort: kontaktblock

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../FC_BOARD.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Ett bärarkort för ESP32-S3 DevKitC-1 (N16R8): DevKit-kortet sätts i två hylslister, med block av JST-XH-kontakter runt omkring. Det här dokumentet besvarar tre frågor: vilka kontakter som ska samlas i block, var kondensatorerna ska sitta och vad som kopplas in var. Stiften stämmer med `include/config/Config.h` (blocket `BOARD_ESP32_S3`).

> Det här kortet är utlagt för ESP32-S3, det tidigare huvudkortet. Huvudkortet är nu STM32H743 (DevEBox H743): något bärarkort för det har ännu inte ritats, så tills vidare ansluts sensorerna till DevEBox-kortets stiftlister – stiftbeläggningen finns i [PILOT_GUIDE](PILOT_GUIDE.md).

Kortet är konstruerat för att vara **enkelsidigt**: GPIO:erna är valda så att stiften i varje block ligger i följd längs DevKit-kortets list, och signalbanorna sprids ut utan att korsa varandra. Jag har inte kontrollerat ledningsdragningen i ett CAD-verktyg. Om det inte går ihop någonstans, sätt en trådbygel på komponentsidan: en till tre sådana på ett kort som det här är helt okej.

**Alla kontakter är JST-XH, 1–5 stift.** Trådar löds inte på kortet: motsvarande kontaktdon krimpas på trådarna från servon, ESC, mottagare och moduler. XH är polariserad, så den kan inte sättas i bakvänt. Ett stift klarar ~3 A.

---

## 1. Layoutskiss

Vy ovanifrån, från komponentsidan. DevKit-kortets USB-kontakter sitter vid nederkanten.

```
                    överst: ESP-antennen, ingen koppar under den
+----------------------------------------------------------------------+
| GND-ring längs hela kanten                                           |
|  (470 µF)              3V3 --------------------+    +- C5 IMU        |
|  A1 AIL-L               ^  (bakom DevKit-änden)|    +- C4 BARO       |
|  A2 AIL-R               |                      |    +- C3 I2C-B      |
|  A3 ELE          +------+-------------+        |    +- C2 I2C-A      |
|  A4 ESC <-BEC    | J1-1,2: 3V3        |        +--->+- C1 OLED       |
|  A5 AUX1  <------| J1-4..11   J3-4..7 |-------------^  I2C-skena     |
|  A6 AUX2         |                    |                              |
|  A7 RC           |      DevKit        | J3-8..10 --> D1 GPS          |
|  A8 RUD          |     ESP32-S3       |              D2 BUZ   [Q1]   |
|  (470 µF)        |                    |                              |
|  B1 BAT   <------| J1-12,13           | J3-17,18 --> D3 AUX3         |
|  B2 TELEM <------| J1-15,16           |              D4 LIGHT [Q2]   |
|                  | J1-21: 5V          |                  ^           |
|                  +----[USB]--[COM]----+                  | logik 5V  |
|    [diod 1N5822] --> logik 5V --- nederst, under USB ----+           |
+----------------------------------------------------------------------+
```

Fyra zoner:

| Zon | Var | Vad som finns där | Matning |
|---|---|---|---|
| **A** Servon | vänsterkanten, mittemot J1-4…11 | 8 × XH-3: roderytor, ESC, AUX, mottagare | servo 5V (smutsig) |
| **B** Batteri, telemetri | nere till vänster, mittemot J1-12…16 | batteri- och strömsensorer, radiomodemet | logik 5V |
| **C** 3V3 | uppe till höger, mittemot J3-4…7 | OLED, sensorer på I2C-skenan, I2C-kontakter | 3V3 |
| **D** Logik 5V | nere till höger, mittemot J3-8…18 | GPS, summer, AUX3, lampor | logik 5V |

Kraftdelen sitter till vänster (servon, ESC, batteri), sensorerna och kommunikationen till höger. Servoströmmen stannar till vänster och passerar inte förbi sensorerna.

---

## 2. DevKit-kortets lister: vilket stift som går vart

Stiftordningen är den för ESP32-S3-DevKitC-1 (2 × 22). Kontrollera den mot tryckningen på ditt kort och mät avståndet mellan raderna innan du ritar något. Numreringen löper från antennen till USB.

| J1 (vänster rad) | GPIO | Till | | J3 (höger rad) | GPIO | Till |
|---|---|---|---|---|---|---|
| 1 | 3V3 | 3V3 över toppen → zon C | | 1 | GND | – |
| 2 | 3V3 | (samma) | | 2 | 43 | – (”COM”-konsol) |
| 3 | RST | – | | 3 | 44 | – (”COM”-konsol) |
| 4 | 4 | A1 AIL-L | | 4 | 1 | C1 OLED SDA |
| 5 | 5 | A2 AIL-R | | 5 | 2 | C1 OLED SCL |
| 6 | 6 | A3 ELE | | 6 | 42 | I2C-skena: SCL |
| 7 | 7 | A4 ESC | | 7 | 41 | I2C-skena: SDA |
| 8 | 15 | A5 AUX1 | | 8 | 40 | D1 GPS: TX |
| 9 | 16 | A6 AUX2 | | 9 | 39 | D1 GPS: RX |
| 10 | 17 | A7 RC (iBUS) | | 10 | 38 | D2 summer (via Q1) |
| 11 | 18 | A8 RUD | | 11 | 37 | – PSRAM |
| 12 | 8 | B1 VBAT (ADC) | | 12 | 36 | – PSRAM |
| 13 | 3 | B1 CURR (ADC) | | 13 | 35 | – PSRAM |
| 14 | 46 | – strapping | | 14 | 0 | – BOOT-knappen |
| 15 | 9 | B2 TELEM: TX | | 15 | 45 | – strapping |
| 16 | 10 | B2 TELEM: RX | | 16 | 48 | – RGB-lysdiod |
| 17 | 11 | ledig | | 17 | 47 | D3 AUX3 |
| 18 | 12 | ledig | | 18 | 21 | D4 LIGHT (via Q2) |
| 19 | 13 | ledig | | 19 | 20 | – USB |
| 20 | 14 | ledig | | 20 | 19 | – USB |
| 21 | 5V | ingång logik 5V (efter dioden) | | 21 | GND | jord för zon D |
| 22 | GND | jord för zon B | | 22 | GND | jord för zon D |

På bänken använder firmwaren de lediga GPIO11–14 för SPI (ICM-42688); på det här kortet är SPI inte draget.

---

## 3. Regler för att få plats i ett enda lager

1. **Hålmonterade komponenter ovanpå, SMD på undersidan.** DevKit-hylslisterna, XH-kontakterna, elektrolyterna och dioden sitter på komponentsidan. 0805-, 0603- och SOT-23-komponenter löds direkt på kopparn. Med toneröverföring skrivs kopparmönstret ut spegelvänt.
2. **I varje kontakt sitter signalen närmare DevKit-kortet, matningen längre bort, GND vid kanten.** Därför är **stift 1 i alla kontakter det som är närmast DevKit-kortet**. Signalbanorna korsar då inte matningen. Undantaget är I2C-skenan (punkt 5).
3. **GND är en kopparyta runt hela omkretsen** (en ring). Kontakternas yttersta stift går direkt till den.
4. **Bara två ledningar går från ena sidan av DevKit-kortet till den andra.** 3V3 går uppåt från J1-1/2, förbi DevKit-kortets övre ände och åt höger – utanför kanten på DevKit-kortet, inte under antennen. Logik 5V går från J1-21 nedåt under DevKit-kortet och längs nederkanten åt höger, under USB-kontakterna (där finns bara banor, kontakten sitter högre upp).
5. **I2C-skenan.** Fyra parallella banor med 2,54 mm delning, som löper utåt från DevKit-kortet: **3V3 · GND · SCL · SDA**. Det är stiftordningen hos GY-modulerna (VCC GND SCL SDA). Hylslisterna och kontakterna sitter tvärs över skenan som järnvägsvagnar: varje bana går genom sitt eget stift. Skenan börjar vid J3-6/7, dyker in under OLED-skärmen och löper uppåt längs den högra raden. Om den inte får plats på höjden, böj den åt vänster över DevKit-kortets övre ände; ledningarnas ordning bevaras genom svängen.
6. **Dra inga banor mellan DevKit-kortets stift**: delningen 2,54 är för trång för toneröverföring. Under själva DevKit-kortet går det bra med banor – där är det 11 mm upp till dess kort.
7. **Hålmonterade komponenter är gratis byglar.** En bana passerar bekvämt under diodens kropp (benavstånd 12,5–15 mm) och mellan benen på en elektrolyt (5 mm).
8. **0805 mellan kontaktens stift.** XH-delningen 2,5 mm – en 0805-kondensator löds direkt mellan de intilliggande +5V- och GND-stiften på kopparsidan.
9. **Banbredder:** servo 5V och servo GND – från 2 mm, logik 5V – från 1 mm, signaler – 0,4–0,5 mm.
10. **Tryckta etiketter:** kontaktnumret (A1, B2…), namnet och en pil vid stift 1.

---

## 4. Kontaktblock

### Block A – servon: 8 × XH-3, vänsterkanten, i en kolumn mittemot J1-4…11

Stiften i varje kontakt:
**1 – signal** (närmare DevKit-kortet) · **2 – servo +5V** · **3 – GND** (mot kanten).
Det är ordningen i en servokabel: orange, röd, brun.

| Kontakt | Etikett | Vad som kopplas in | GPIO (stift) |
|---|---|---|---|
| A1 | AIL-L | det vänstra skevrodret | 4 (J1-4) |
| A2 | AIL-R | det högra skevrodret | 5 (J1-5) |
| A3 | ELE | höjdrodret | 6 (J1-6) |
| A4 | ESC | regulatorn: gassignalen; **på den röda tråden – BEC 5V-ingången** | 7 (J1-7) |
| A5 | AUX1 | lastsläppsservot | 15 (J1-8) |
| A6 | AUX2 | klaffar (två servon via en Y-kabel) eller valfritt servo | 16 (J1-9) |
| A7 | RC | FS-iA6B-mottagaren, iBUS SERVO-porten (mottagaren matas härifrån) | 17 (J1-10) |
| A8 | RUD | sidrodret + hjulet | 18 (J1-11) |

Vad mer som löds i blocket:

- **Servo +5V-bussen** – den mittersta kolumnen av stift, en bana från 2 mm. **GND** – den yttre kolumnen, som också är en del av GND-ringen.
- **330 Ω (0603)** i ett avbrott på varje signalledning, vid kontakten. Om 5 V någon gång når ett signalstift (ett dåligt servo, en dålig krimpning) överlever ESP-stiftet.
- **10 kΩ (0603)** från A4 ESC-signalen till GND: medan ESP:n startar om når inget skräp fram till regulatorn.
- **100 nF (0805)** mellan stift 2 och 3 vid varje kontakt.
- **A4 ESC** har dessutom 10 µF + 100 pF (0805): matningsingången för hela kortet – lågfrekvent, högfrekvent och mikrovågsfiltrering.
- **A7 RC** har dessutom 10 µF (0805): mottagaren är känslig för spänningsfall.
- **2 × 470 µF 16 V** på servobussen, en i vardera änden av kolumnen (ovanför A1 och under A8): plus till bussen, minus till ringen. Inne i kolumnen når minus inte ringen, och 3 cm bred bana gör ingen skillnad för en elektrolyt.

Ett ESC-uttag klarar ~3 A. För 4–6 MG90S-servon räcker det. Om det blir fler servon och de är kraftigare, lägg till ett separat BEC-matningsuttag bredvid A4.

### Block B – batteri och telemetri, nere till vänster under servona

**B1 BAT – XH-5** (den enda XH-5 på kortet: en kabel med 12–17 V passar inte i någon annan kontakt)

| Stift | Vad | Var på kortet |
|---|---|---|
| 1 | **VBAT** – batteriets plus via en tunn tråd (det yttersta ”+” i balanskabeln, eller från batterikontakten, inte via ESC:n) | en spänningsdelare 56 kΩ / 10 kΩ → GPIO8 (J1-12) |
| 2 | tomt – ett mellanrum mellan batterispänningen och allt annat | – |
| 3 | **CURR** – strömsensorns utgång | en spänningsdelare 10 kΩ / 15 kΩ → GPIO3 (J1-13) |
| 4 | logik +5V – matning för strömsensorn | logik 5V-bussen |
| 5 | GND | ringen |

- **VBAT-delaren:** 56 kΩ överst, 10 kΩ nederst, 100 nF parallellt med den nedre. 3S (12,6 V) → 1,91 V, 4S (16,8 V) → 2,55 V – med marginal upp till ADC-gränsen (~3,1 V). En separat jordtråd behövs inte: jorden är gemensam via ESC:n, så stift 5 kan lämnas okrimpat.
- **CURR-delaren:** 10 kΩ överst, 15 kΩ nederst, 100 nF parallellt med den nedre. En 5 V Hall-sensor (ACS758 och liknande) ger högst 5 V → 3,0 V på stiftet. Om sensorns utgång är 3,3 V är det övre motståndet 0 Ω och det nedre löds inte i.
- Placera delarna direkt vid kontakten och ta jorden från dess stift 5 – då går bara en bana till ESP:n.
- Ingen strömsensor ännu? Krimpa helt enkelt inte stift 3–4.

**B2 TELEM – XH-4:** ett telemetriradiomodem eller iBUS-SENS.

| Stift | Vad | GPIO (stift) |
|---|---|---|
| 1 | TX → till modemets RX | 9 (J1-15) |
| 2 | RX ← från modemets TX | 10 (J1-16) |
| 3 | logik +5V | – |
| 4 | GND | – |

- 10 µF + 100 nF mellan stift 3 och 4. För ett modem på 1 W, lägg till en elektrolyt på 470 µF.
- Firmwaren stöder ännu inte TELEM: alla tre UART:er är upptagna (konsol, iBUS, GPS). För att aktivera den måste konsolen flyttas till den inbyggda USB:n. Det är en ändring i firmwaren; kontakten är redan dragen.

### Block C – 3V3: skärmen och sensorerna, uppe till höger

Allt i den här zonen sitter på **I2C-skenan** (avsnitt 3, punkt 5): fyra banor **3V3 · GND · SCL · SDA** som löper utåt från DevKit-kortet. SCL kommer från GPIO42 (J3-6), SDA från GPIO41 (J3-7). 3V3 kommer över toppen från J1-1/2. OLED-skärmen sitter lägst på skenan, med C2–C5 ovanför i ordning.

**C1 OLED – XH-4.** Den tar matning från skenan och data från sin egen buss (GPIO1/2), som kommer in från insidan.

| Stift | Vad | Från |
|---|---|---|
| 1 | SDA | GPIO1 (J3-4) |
| 2 | SCL | GPIO2 (J3-5) |
| 3 | 3V3 | skenan |
| 4 | GND | skenan |

Det är OLED-modulens stiftordning (GND VCC SCL SDA) bakvänd, så kabeln går utan vridning.

**C2 I2C-A och C3 I2C-B – XH-4**, identiska:

| Stift | Vad |
|---|---|
| 1 | 3V3 |
| 2 | GND |
| 3 | SCL |
| 4 | SDA |

- **C2** – kompassen: en GY-273 på en mast (en rak kabel, ordningen är densamma som på modulen) eller kompassen från en GPS-modul. För GPS:en krimpas bara GND, SCL och SDA: kompassen får matning via GPS-kabeln.
- **C3** – reserven: en lufthastighetssensor (MS4525DO), en avståndsmätare och så vidare.

**C4 BARO – en 1×4-hylslist för en BMP581-modul:** 1 – VCC, 2 – GND, 3 – SCL, 4 – SDA.

- Löd trådbyglar på själva modulen: **CSB→VCC** och **SDO→GND** (adress 0x46; 0x47 förblir ledig för pitotröret). Utan CSB→VCC går kretsen över till SPI-läge; med SDO flytande vandrar adressen. Modulens övriga stift hänger fritt.
- 3V3 endast från skenan: många BMP581-moduler har ingen egen regulator.
- Stiftordningen skiljer sig mellan moduler – kontrollera din. Om den inte stämmer går modulen via en kabel till C3, och hylslisten monteras inte.
- Ovanpå – en bit öppencellig skumplast (mot luftström och ljus).

**C5 IMU – en 1×8-hylslist för en GY-521:**

| Stift | Modulens stift | Till |
|---|---|---|
| 1 | VCC | 3V3-skenan |
| 2 | GND | GND-skenan |
| 3 | SCL | SCL-skenan |
| 4 | SDA | SDA-skenan |
| 5, 6 | XDA, XCL | ingenting |
| 7 | AD0 | till GND-ytan utanför skenan (adress 0x68) |
| 8 | INT | ingenting |

Hur IMU:n är vriden på kortet spelar ingen roll: monteringen bestäms av kalibreringen `o` (PILOT_GUIDE, ”IMU-montering”).

En fristående MPU-6500-modul (10 stift: VCC GND SCL SDA EDA ECL AD0 INT NCS FSYNC) passar inte i den här hylslisten – stiftordningen är annorlunda. Den går via en kabel till C3 (VCC, GND, SCL, SDA), med byglar på modulen: **NCS→VCC** (annars går kretsen över till SPI-läge), **AD0→GND** (adress 0x68) och **FSYNC→GND**.

Blockets kondensatorer: **10 µF + 100 nF** på skenan vid C1 (där börjar 3V3), **100 nF** mellan stift 1 och 2 vid C2–C5. I2C-pull-up-motstånden finns redan på modulerna; montera dem inte på kortet.

### Block D – logik 5V: GPS, summer, lampor, nere till höger

Logik 5V-bussen kommer nedifrån (från under DevKit-kortet längs nederkanten) och klättrar uppför högerkanten genom ”+5V”-stiften i alla kontakter i blocket.

**D1 GPS – XH-4.** Den sitter direkt under skenan, så att GPS-uttaget och uttaget för dess kompass (C2) sitter nära varandra.

| Stift | Vad | GPIO (stift) |
|---|---|---|
| 1 | TX → till GPS:ens RX | 40 (J3-8) |
| 2 | RX ← från GPS:ens TX | 39 (J3-9) |
| 3 | logik +5V | – |
| 4 | GND | – |

10 µF + 100 nF mellan stift 3 och 4.

**D2 BUZ – XH-2:** en aktiv 5 V-summer, för att hitta flygplanet i gräset och varna för batteri och ARM.

| Stift | Vad |
|---|---|
| 1 | summerns ”−” → transistorn Q1 |
| 2 | logik +5V → summerns ”+” |

**D3 AUX3 – XH-3:** 1 – GPIO47-signalen (J3-17) via 330 Ω, 2 – logik +5V, 3 – GND, plus 100 nF mellan 2 och 3. Lämplig för en knapp, data till en LED-slinga, en kamerautlösare. **Häng inte ett servo på den:** det här är logik 5V, och dess ström skulle gå genom dioden och störa ESP:ns matning.

**D4 LIGHT – XH-2:** en brytare för en last på upp till ~0,5–1 A – navigationsljus, en strålkastare, elektromagneten för lastsläpp.

| Stift | Vad |
|---|---|
| 1 | lastens ”−” → transistorn Q2 |
| 2 | logik +5V → lastens ”+” |

**Transistorerna Q1 och Q2** – samma SOT-23-fotavtryck. På BC817 och Si2302 stämmer stiften i funktion:

| SOT-23-stift | BC817 | Si2302 | Till |
|---|---|---|---|
| 1 | bas | gate | ← 1 kΩ ← GPIO (38 för Q1, 21 för Q2); 10 kΩ från stift 1 till GND |
| 2 | emitter | source | GND (zonens jord – J3-21/22) |
| 3 | kollektor | drain | stift 1 i kontakten (D2 / D4) |

- **Q1 (summer):** vilken som helst av de två duger.
- **Q2 (lampor):** **Si2302** – BC817 blir varm vid hundratals milliampere.
- 10 kΩ håller transistorn stängd medan ESP:n startar: summern skriker inte, lamporna blinkar inte.
- Om lasten är en spole (en elektromagnet, en magnetisk summer), montera en diod SS14 eller 1N4148 parallellt med kontakten, katoden mot +5V. Lämna plats för den mellan stift 1 och 2.

---

## 5. Matning och alla kondensatorer

```
 ESC (BEC 5V/5A) ──► A4 ──► servo 5V-buss ──┬──► A1…A8 (servon, mottagare)
                                           │    2×470 µF i kolumnens ändar
                                           │
                                           └──► diod 1N5822 ──► logik 5V ──┬──► J1-21 (5V DevKit)
                                                                            ├──► B1, B2 (strömsensor, modem)
                                                                            └──► D1…D4 (GPS, summer, AUX3, lampor)
 DevKit: egen 3V3-regulator ──► J1-1/2 ──► över toppen ──► skena C (OLED, sensorer, I2C-kontakter)
```

- **Det finns bara en ingång – A4 ESC.** En separat matningskontakt behövs inte.
- **Schottkydioden 1N5822** (3 A, hålmonterad; SMD-ersättaren är SS34) – köp den. Den gör tre saker:
  - USB och BEC:en slåss inte: du kan ha USB inkopplad med batteriet anslutet;
  - när servona drar ned bussen laddas logikens elektrolyt inte ur tillbaka in i servona, och ESP:n startar inte om;
  - den hålmonterade kroppen fungerar som en bygel över GND-banan i hörnet vid J1-22.

  Anoden går till servobussens nedre ände, katoden till J1-21. Ta inte en 1N5819 (1 A): ESP:n, modemet, GPS:en och lamporna drar alla ström genom dioden.
- Servona matas inte enbart från USB; dioden kopplar bort dem. Det är avsiktligt. GPS:en, modemet och summern fungerar från USB på skrivbordet bara om DevKit-kortets 5V-stift levererar ström från USB. Vissa kloner har en egen diod där, och då fungerar de inte – det är normalt.
- 3,3 V kommer bara från DevKit-kortets regulator och bara för zon C.

**Alla kondensatorer i en tabell** (keramiska – 0805, elektrolyter – 16 V):

| Var | Vad | Varför |
|---|---|---|
| Servobussen, ovanför A1 och under A8 | 470 µF + 470 µF | spänningsfall när alla servon rycker samtidigt |
| A4 ESC, mellan + och GND | 10 µF + 100 nF + 100 pF | matningsingången: låg-, hög- och ultrahögfrekvent |
| A1–A3, A5–A8 | 100 nF vid varje | störningar från servomotorer – vid källan |
| A7 RC | + 10 µF | mottagaren |
| Logik 5V, vid J1-21 | 470 µF + 10 µF + 100 nF | håller uppe ESP:n när BEC:en sviktar |
| 3V3-skenan, vid C1 | 10 µF + 100 nF | matning för sensorerna |
| C2–C5 | 100 nF vid varje | |
| B1: ADC-ingången för VBAT och CURR | 100 nF på varje, parallellt med det nedre motståndet | ADC-filtret |
| B1: strömsensorns +5V | 100 nF mellan stift 4 och 5 | |
| B2 TELEM | 10 µF + 100 nF (ett modem på 1 W – + 470 µF) | sändarens strömspikar |
| D1 GPS | 10 µF + 100 nF | |
| D3 AUX3 | 100 nF | |

En keramisk 10 µF 0805 förlorar upp till hälften av sin kapacitans vid 5 V – det är inräknat, och det finns elektrolyter i närheten ändå.

Mätpunkter: **5VS** (servobussen), **5VL** (logik 5V), **3V3**, **GND**. De är praktiska för mätning med multimeter.

---

## 6. Vad som kopplas in var

| Enhet | Kontakt | Anmärkningar |
|---|---|---|
| Servona för skevroder, höjdroder och sidroder | A1, A2, A3, A8 | signalen går till stift 1 |
| ESC | A4 | den röda tråden är BEC-ingången |
| FS-iA6B-mottagaren | A7 | iBUS SERVO-porten, en vanlig 3-trådig kabel |
| Lastsläppsservot, klaffar | A5, A6 | |
| GY-521 (MPU6500) | C5, hylslist | |
| BMP581 | C4, hylslist | byglar på modulen CSB→VCC, SDO→GND (0x46) |
| GY-273 (kompass) eller GPS-kompassen | C2 | bort från kraftledningarna, helst på en mast |
| OLED 128×64 | C1 | |
| Lufthastighetssensor och liknande | C3 | |
| u-blox M10 GPS | D1 + C2 | de 6 trådarna delas mellan två kontaktdon: D1 (matning, UART) och C2 (kompass) |
| Batterispänning, strömsensor | B1 | |
| Radiomodem / iBUS-SENS | B2 | |
| Summer | D2 | |
| Lampor, strålkastare | D4 | |

Placering i flygplanet:

- Montera kortet mjukt (skumplast, gelkuddar), närmare tyngdpunkten: motorvibrationer förstör vinklarna.
- Håll kraftledningarna (batteri → ESC → motor) borta från zon C och från kompassen.
- Barometern sitter under skumplast, IMU:n hur du vill (kalibreringen `o`).

---

## 7. Komponentlista

| Komponent | Antal | Var |
|---|---|---|
| Hylslist PBS 1×22 (för DevKit-kortet) | 2 | |
| XH-3, vinklad eller rak | 9 | A1–A8, D3 |
| XH-4 | 5 | B2, C1, C2, C3, D1 |
| XH-5 | 1 | B1 |
| XH-2 | 2 | D2, D4 |
| Hylslister PBS 1×8 och 1×4 | 1 av varje | C5, C4 |
| Schottkydiod 1N5822 (eller SS34) | 1 | **köp den** |
| Elektrolyt 470 µF 16 V | 3 (+1 för ett kraftigt modem) | servobussen ×2, logik 5V |
| 0805 10 µF | 6 | A4, A7, logik 5V, 3V3, B2, D1 |
| 0805 100 nF | 20 | se kondensatortabellen |
| 0805 100 pF | 1 | A4 |
| 0603 330 Ω | 9 | signaler A1–A8, D3 |
| 0603 10 kΩ | 5 | ESC till GND, nedre VBAT, övre CURR, stift 1 vid Q1 och Q2 |
| 0603 56 kΩ | 1 | övre VBAT |
| 0603 15 kΩ | 1 | nedre CURR |
| 0603 1 kΩ | 2 | till stift 1 vid Q1 och Q2 |
| BC817 eller Si2302 | 1 | Q1 (summer) |
| Si2302 | 1 | Q2 (lampor) |
| SS14 / 1N4148 | 0–2 | bara för spolar på D2 och D4 |

Kortet blir ungefär 80×95 mm: servokolumnen och sensorskenan sticker upp ovanför DevKit-kortets övre ände. Om det inte får plats i flygkroppen är det enklaste sättet att krympa det att flytta BARO och IMU:n till kablar mot C3 och korta skenan.

---

## 8. Vilka GPIO:er man inte ska röra

0, 45, 46 – startläget beror på dem; 19/20 – USB; 26–37 – flash och PSRAM i N16R8-modulen; 43/44 – ”COM”-kontakten (konsolen); 48 – RGB-lysdioden. Efter den här planen återstår bara GPIO11–14 lediga (på bänken – SPI).

---

## 9. Före den första starten

1. Utan DevKit-kortet och utan batteriet, mät igenom: servo +5V ↔ GND, logik 5V ↔ GND, 3V3 ↔ GND – det får inte finnas kortslutning någonstans.
2. Koppla på BEC:en (via A4), med DevKit-kortet ännu inte isatt. 5VS ska visa 5,0–5,2 V, och 5VL 0,3–0,5 V lägre (spänningsfallet över dioden).
3. Sätt i DevKit-kortet och anslut bara USB. 5VS visar 0 V: dioden släpper inte in USB i servona.
4. Allt tillsammans. Konsolens `s` visar om sensorerna svarar och hur många I2C-fel det finns.

---

## 10. Vad som ändrats jämfört med den tidigare planen

- **Stiften har kastats om för ett enda lager** (redan i `Config.h`):
  - sensorernas I2C 8/9 → **41/42**;
  - GPS 15/16 → **39/40**;
  - AUX1/AUX2 41/42 → **15/16**;
  - VBAT 3 → **8**, strömsensor 10 → **3**;
  - telemetri 39/40 → **9/10**;
  - en ny **LIGHT**-utgång på GPIO21.

  På bänken, flytta två trådar: SDA 8 → 41, SCL 9 → 42. De andra stiften är i reserv och inte anslutna på bänken.
- **En BEC via ESC:n**: den separata PWR-kontakten är borta.
- **Sensorerna på kortet sitter på I2C** (som på bänken). SPI-hylslisterna är borta, GPIO11–14 är lediga. ICM-42688 talar också I2C, men firmwaren kommer att behöva en I2C-variant av den i `SensorSelection.h`.
- **GPS-MAG är borta**: GPS-kompassen kopplas in i C2.
- **VBAT och strömsensorn är sammanslagna** i en enda XH-5 (B1).
