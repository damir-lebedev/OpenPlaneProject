# Svart låda

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../BLACKBOX.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Firmwaren registrerar själv varje flygning i kortets inbyggda flash: sensorer, spakar, servoutgångar, autopilotens beslut, händelser. Efter flygningen hämtas inspelningen via USB och avkodas till CSV-tabeller.

Den fungerar på två kort:

| Kort | Var den spelar in | Hur mycket som får plats (vid ~20 KB/s) |
|---|---|---|
| **ESP32-S3 N16R8** | en partition på 13,9 MB i det inbyggda flashminnet | ungefär **11 minuter** |
| **STM32H743** (DevEBox – huvudkort; WeAct) | en fil på ett SD-kort, [nedan](#sd-kort-stm32h743) | 64 MB – ungefär **55 minuter**, storleken bestäms av filen |

På de andra korten (ESP32-C3, den vanliga ESP32) finns inget lagringsutrymme: den svarta lådan är avstängd och stör inte flygningen.

---

## När den spelar in

| | Villkor |
|---|---|
| **Start** | Armerat **och** gasen höjd (spaken eller ESC:n över `THROTTLE_LOW_US`). **10 s dessförinnan** spelas också in – ögonblicket för ARM och väntan före start |
| | En omstart orsakad av ett fel (panic, watchdog, spänningsfall) – inspelning från första cykeln och i minst 60 s: om det hände i luften ser man vad som följde |
| | Manuellt från konsolen (`k` → `r`) – för bänken |
| **Stopp** | **10 s efter DISARM** |
| | Armerat, men motorn står still och flygplanet har varit **orörligt i 30 s** – det landade eller kraschade, och DISARM glömdes |
| | Manuellt (`k` → `r`) |
| **Inget stopp** | Förlorad förbindelse, failsafe, motorn på noll i luften, glidflykt, landning utan DISARM medan flygplanet fortfarande rullar |

”Orörligt” betyder allt detta på en gång: rotation under 5 °/s på varje axel, accelerometern visar 1g ± 0,1, nästan ingen vertikal hastighet enligt barometern, och enligt GPS och pitotröret (om det finns) långsammare än 2 m/s. Under flygning är det aldrig så stilla i 30 sekunder i sträck.

## Vad som spelas in

| Post | Frekvens | Vad den innehåller |
|---|---|---|
| `IMU` | varje cykel, 500 Hz | gyroskop (°/s), accelerometer (g), hur lång tid styrcykelns arbete tog (µs) |
| `CTRL` | 100 Hz | roll/tippning/kurs, autopilotens mål, pilotens spakar, de slutliga kommandona, **alla 7 utgångar** (µs), komponenterna i roll- och tipp-PID (P, I, D), pilotens och autopilotens gas, klaffar, läge, flaggor (ARM, förbindelse, failsafe, sensorer vid liv...), påslagna funktioner |
| `RC` | 50 Hz | alla 10 sändarkanaler, räknare för iBUS-ramar (bra och dåliga) |
| `BARO` | varje mätning (~50 Hz) | tryck, temperatur, höjd, vertikal hastighet, höjdmål |
| `MAG` | upp till 50 Hz | fältet på tre axlar, kurs |
| `GPS` | varje lösning | koordinater, höjd, hastighet, kurs, satelliter, fix, noggrannhet |
| `AIR` | upp till 50 Hz | pitotrör: tryckskillnad, indikerad och sann lufthastighet, densitet |
| `NAV` | 10 Hz | hempunkt (avstånd, bäring), kurs och kursmål, navigeringshastighet, kurskälla, steg för handstart och termikflygning, autotrimning |
| `POWER` | 10 Hz | batterispänning och strömsensorns utsignal (spänningsdelarna på flygkontrollerkortet, [FC_BOARD.md](FC_BOARD.md), block B) |
| `SYS` | 1 Hz | styrslingans frekvens och sämsta cykel, ledigt minne, iBUS-räknare, IMU-temperatur, den svarta lådans kö, förlorade poster, den längsta flashskrivningen, ledigt utrymme |
| `EVENT` | vid en händelse | ARM/DISARM, en ARM-vägran med orsak, lägesbyte, förbindelse förlorad/återställd, sensor felade/återhämtade sig, GPS-fix, hempunkt registrerad, brytarfunktioner, geofence, överstegringsskydd, steg för handstart och termikflygning |

I början av varje flygning kommer parametrarna: firmwaren (byggdatum), orsaken till starten och till den senaste omstarten, vilka sensorer som finns och om de klarade kontrollen före flygning, PID-koefficienterna (inklusive ändringar från panelen), trimmen, viktiga `Config`-värden och brytarbindningarna.

---

## Hur man använder den

### Före flygningen

Det finns inget att göra. Vid start visar seriemonitorn statusen (konsolen skriver på ryska; raden nedan betyder ”väntar på ARM och gas | raderat i förväg 12.9 MB (≈11 min) av 13.9 MB | flygningar 1”):

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

”Raderat i förväg” är hur mycket som får plats i nästa flygning. Efter start ägnar den svarta lådan några sekunder (efter en lång flygning – upp till en minut) åt att förbereda utrymme: den raderar gamla poster. Under den tiden fryser flygslingan på marken ibland i ~0,15 s – ytorna kan rycka till sent, vilket är normalt. **I luften raderas flashminnet aldrig.**

### Efter flygningen – hämtning

1. Anslut USB till **COM**-kontakten. Stäng seriemonitorn (den håller porten).
2. Kör i projektmappen:

   ```bash
   python tools/blackbox.py download          # den senaste flygningen
   python tools/blackbox.py download --all    # alla
   python tools/blackbox.py list              # vad som finns på kortet
   ```

   `pyserial` krävs: `pip install pyserial`. Eller använd PlatformIO:s Python, som redan har det: `%USERPROFILE%\.platformio\penv\Scripts\python tools\blackbox.py download`.

3. Flygningen hämtas till mappen `blackbox/` (~200 KB/s: 10 minuters flygning tar ungefär en minut) och avkodas direkt bredvid, till en mapp med samma namn.

Medan hämtningen pågår står flygslingan still, så den fungerar bara utan ARM.

### Vad som finns i en flygnings mapp

| Fil | Vad det är |
|---|---|
| `summary.txt` | Sammanfattningen: längd, frekvenser, intervall för vinklar, höjder, hastigheter, spänningar, den sämsta styrcykeln, förlorade poster, alla händelser |
| `events.txt` | Flygningens parametrar och alla händelser i tidsordning |
| `IMU.csv`, `CTRL.csv`, `RC.csv`, ... | En tabell per posttyp |

Tiden i alla tabeller är `time_s`, sekunder från inspelningens start (ARM och gas); förinspelningen är negativ. Värdena är redan i enheter: grader, g, meter, m/s, mikrosekunder puls. I `CTRL.csv` läggs läget till med namn (`mode_name`), funktionerna som en lista (`features_on`), och flaggorna delas upp i 0/1-kolumner (`armed`, `rx_lost`, `fs_glide`, `imu_ok`...).

CSV-filerna öppnas i Excel/LibreOffice, men för diagram över tid är [PlotJuggler](https://github.com/facontidavide/PlotJuggler) bekvämare: File → Load Data → CSV, tidskolumnen `time_s`.

En `.bbl`-fil är en rå avbild av flashminnet, och den kan avkodas igen: `python tools/blackbox.py decode blackbox/flight_001_....bbl`.

### Konsol: `k`

I seriemonitorn öppnar tangenten `k` den svarta lådans meny: status, listan över flygningar, `r` – starta/stoppa inspelning för hand (för kontroll på bänken), `e` – radera alla flygningar (med `y` för att bekräfta, ~40 s).

---

## Utrymme i flashminnet

- Flygningar skrivs i en ring. När utrymmet tar slut raderar den svarta lådan på marken **de äldsta flygningarna i sin helhet** tills 10 MB är ledigt framåt (`BLACKBOX_MIN_FREE_BYTES`, ~9 minuter).
- **Den senast inspelade flygningen raderas aldrig** – bara av nästa inspelning, om den fick slut på utrymme.
- Om det raderade utrymmet tar slut i luften fortsätter inspelningen i en kö i PSRAM (4 MB, ~3 minuter av de senaste data); efter landning och DISARM frigör den svarta lådan utrymme och skriver ut den. En flygning som är längre än hela partitionen (~11 min) får inte plats helt: början behålls och slutet går förlorat.
- Därför **ska du hämta flygningen efter varje tur** – särskilt den första.

## Tillförlitlighet

- Ett strömavbrott när som helst (en krasch, batteriet lossnade): allt bevaras utom de sista ~15 ms. Delvis skrivna poster kasseras med CRC – i `summary.txt` är det raden ”Недописанных записей (CRC)” (sammanfattningen är på ryska; den betyder ”Ofullständiga poster (CRC)”).
- Flygningsnumret, ringens huvud och listan över flygningar återställs från själva sektorerna: det finns ingen separat ”karta” som kan bli korrupt.
- Hämtningen kontrollerar CRC-32 för varje sektor och för hela flygningen.

## Påverkan på flygningen

- Flygslingan lägger bara en ögonblicksbild i en kö i PSRAM – några mikrosekunder. En separat uppgift på kärna 0 skriver till flash, en sida (256 byte) i taget, **direkt efter en styrcykel**: en flashskrivning stoppar båda ESP32-kärnorna i 0,6–0,9 ms, och den hamnar i pausen mellan cyklerna.
- Uppmätt på bänken (ett DevKit utan sensorer, två körningar med 30–40 s inspelning): intervallet mellan cyklerna är 2,00 ms, 99,2–99,7 % av intervallen ligger inom 1,9–2,1 ms, det längsta är 2,5 ms, och inte en enda cykel hoppades över; cykelns belastning är densamma som utan inspelning. Cykeln varierar märkbart bara på marken utan ARM, medan den svarta lådan verifierar och raderar utrymme (läsning av ett block på 64 KB – en paus på ~3 ms, en radering – ~0,15 s).
- Med sensorer tar en cykel ~0,7 ms, och en sidskrivning får fortfarande plats i de återstående 1,3 ms. Kontrollera efter den första flygningen: i `summary.txt` raderna ”Такт IMU” och ”Цикл: худший такт” (på ryska: ”IMU-cykel” och ”Slinga: sämsta cykel”).

---

## SD-kort (STM32H743)

På STM32H743 spelar den svarta lådan in på ett SD-kort (en µSD-plats på SDMMC1, 4 bitar, 24 MHz). Kortet förblir ett vanligt **FAT32**-kort: i dess rot ligger en i förväg skapad fil `BLACKBOX.BIN`, inuti vilken firmwaren skriver råa block, och den rör aldrig FAT-tabellen eller katalogen. Det finns alltså inget att förstöra när strömmen bryts under flygning, och filen kan helt enkelt kopieras till en dator.

### Förberedelse av kortet (en gång)

1. Formatera kortet som **FAT32** (inte exFAT; Windows erbjuder FAT32 för kort upp till 32 GB).
2. Med kortet i en kortläsare, på en dator:

   ```bash
   python tools/blackbox.py sd-prepare E:              # 64 MB, E: är kortets enhet
   python tools/blackbox.py sd-prepare E: --size 256   # eller större
   ```

   Filen skapas som ett sammanhängande stycke på ett tomt kort och fylls med `0xFF`; den första sektorn är en tjänsteetikett ”ringen är tom”. Om filen inte är sammanhängande (kortet är inte tomt och kraftigt fragmenterat) eller saknas visar konsolen orsaken vid start, och den svarta lådan är avstängd.
3. Sätt in kortet i kortet. Vid start (konsolen skriver på ryska: ”SD-kort: 15204 MB, SDMMC 24 MHz, 4 bitar; fil BLACKBOX.BIN: ok”, sedan statusraden, sedan ”klar på 300 ms”):

   ```
   SD-карта: 15204 МБ, SDMMC 24 МГц, 4 бита; файл BLACKBOX.BIN: ок
   BlackBox: ждёт ARM и газ | стёрто впереди 0.7 МБ из 64.0 МБ | полётов 0
   BlackBox: готов за 300 мс
   ```

   ”Raderat i förväg” växer i bakgrunden: kortet verifierar utrymmet med ~2,5 MB/s.

### Hämta flygningen

- **Via kortet över USB** – som på ESP32: `python tools/blackbox.py download` (STM32-konsolen är USB CDC, hämtningshastigheten är ~400 KB/s, 1 MB tar mindre än 3 s). `list`, `--all`, `--flight N` fungerar på samma sätt.
- **Genom att ta ut SD-kortet**: filen `BLACKBOX.BIN` från kortet avkodas direkt till CSV –

  ```bash
  python tools/blackbox.py ring E:/BLACKBOX.BIN              # alla flygningar -> blackbox/
  python tools/blackbox.py ring E:/BLACKBOX.BIN --list       # bara lista dem
  ```

  Filen är en ring av sektorer: verktyget sätter själv ihop flygningarna utifrån sektornumren, även sådana som slog runt förbi filens slut.

### Vad som mättes på kortet

DevEBox H743 + ett 16 GB-kort (testet `test_blackbox_sd`, [TESTING.md](TESTING.md#tester-på-stm32-kortet)):

| | |
|---|---|
| Identifiering av kortet | 12–18 ms, 4 bitar, 24 MHz |
| Skrivning av en sida på 256 B | 2,3–3,7 ms i genomsnitt, **sämst 60–190 ms**, ~75–110 KB/s uthålligt (~20 KB/s behövs) |
| Läsning | en sektor på 4 KB – 4,2 MB/s; ett slumpmässigt block – 0,6 ms |
| Radering | 64 KB – 13 ms; hela området på 64 MB – 20–28 s |
| Start | med en tom ring – 0 ms (via etiketten); med flygningar – 0,3 s (en stickprovskontroll med ~530 läsningar); en fullständig kontroll av 64 MB skulle ta ~20 s |
| 20 s inspelning i realtid (500 Hz IMU) | inte en enda förlorad post, 0 fel |
| Flyguppgiften under inspelning | periodavvikelse från 2 ms – **1 µs** (en simulatoruppgift med högsta prioritet bredvid inspelningen) |

Den sämsta sidskrivningen är kortets interna ”städning”; kön i RAM (384 KB ≈ 19 s av dataströmmen) klarar sådana pauser. Billiga kort skiljer sig mest just i detta: före flygning är det värt att kontrollera kortet med testet `test_blackbox_sd` (den sämsta skrivningen måste vara under 250 ms – gränsen i SD-specifikationen).

### Hur det skiljer sig från ESP32:s flash

- **Skrivaruppgiften** (`bbox`, prioritet 2) avbryts av flyguppgiften (5) mitt i en kortåtkomst – snarare än ”i cykelns paus” som på ESP32 med dess kärnstopp. Överföringen körs med SDMMC:s hårdvaruflödeskontroll: utan den flödade FIFO:n över vid avbrott (på kortet blev det `HAL_SD_ERROR_RX_OVERRUN` och att konsolen och inspelningen frös i sekunder).
- **Kön ligger i RAM**, 384 KB (`BLACKBOX_RING_STM32_BYTES`), inte 4 MB PSRAM.
- **Kontrollen vid start sker med stickprov**: de verkliga ringsektorerna bildar en sammanhängande båge, ~500 huvuden läses, och gränserna för bågen och flygningarna förfinas med bisektion. Resultatet är detsamma som vid en fullständig kontroll; om bilden inte går ihop – en fullständig.
- **Etiketten ”ringen är tom”** i filens första sektor: så att ett tomt område inte verifieras i sekunder vid varje start. Den sätts när allt är raderat och när en fullständig kontroll inte hittade något; den rensas före den första skrivningen.
- **Kortfel** (utdraget, ett bussfel) hamnar i loggen en gång per sekund som händelsen ”носитель: ошибок записи …” (på ryska: ”lagringsmedium: skrivfel …”); en förlorad sida lämnar ett `0xFF`-hål, och avkodningen av sektorn stannar vid det (precis som i `tools/blackbox.py`), medan de andra sektorerna är intakta.

## Inställningar (`include/config/Config.h`, avsnittet ”Black box”)

| Konstant | Standard | Betydelse |
|---|---|---|
| `BLACKBOX_RING_BYTES` | 4 MB | Kön i PSRAM (utan PSRAM – `BLACKBOX_RING_NO_PSRAM_BYTES`, 32 KB) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32: kön i RAM |
| `BLACKBOX_SD_FILE`, `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN`, 256 MB | STM32: filen på kortet och taket för den del som används |
| `BLACKBOX_PREROLL_MS` | 10 000 | Hur mycket som spelas in före starten |
| `BLACKBOX_POSTROLL_MS` | 10 000 | Hur mycket som spelas in efter DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Står orörligt medan armerat – stopp |
| `BLACKBOX_LANDED_GYRO_DPS`, `_ACCEL_G`, `_CLIMB_MS`, `_SPEED_MS` | 5, 0.1, 0.5, 2 | Vad som räknas som ”orörligt” |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Inspelning efter en felomstart – inte kortare än så |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | Hur mycket som hålls raderat för nästa flygning |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | Pausen mellan raderingar på marken |
| `BLACKBOX_IMU_DIVIDER` | 1 | IMU var N:te cykel: 2–250 Hz och ~25 % längre inspelning |
| `BLACKBOX_VBAT_DIVIDER`, `_CURRENT_DIVIDER` | 6.6, 1.667 | Spänningsdelarna för batteriet och strömsensorn på kortet |

Partitionstabellen är `partitions_blackbox.csv`: applikationen 2 MB (firmwaren är nu ~0,9 MB), den svarta lådan 13,9 MB, coredump 64 KB. NVS-partitionen stannade där den var – kalibreringarna av IMU och kompass samt trimmen bevaras efter bytet till den här tabellen. Det finns ingen andra plats för trådlösa (OTA) uppdateringar.

---

## För utvecklare

Koden är `include/telemetry/BlackBox*.h`:

| Fil | Vad |
|---|---|
| `BlackBoxFormat.h` | Formatet: sektorhuvudet, posttyper och strukturer, fältscheman, CRC-8/CRC-32 |
| `BlackBoxStorage.h` | En ring av sektorer på `IFlashRegion`: att hitta huvudet vid start, listan över flygningar, skrivning sida för sida, radering av gamla flygningar i steg |
| `BlackBoxRing.h` | En kö av poster mellan kärnorna (spinlock), kastar de äldsta |
| `BlackBox.h` | Ögonblicksbilder i slingan, start/stopp, händelser, skrivaruppgiften, hämtning över UART |
| `hal/esp32/Esp32FlashPartition.h` | `IFlashRegion` ovanpå `esp_partition` |
| `hal/SdFileRegion.h`, `storage/Fat32File.h` | `IFlashRegion` ovanpå en fil på ett FAT32-kort: att hitta filen (FAT är skrivskyddad), delblock, radering med `0xFF`, etiketten ”ringen är tom” |
| `hal/stm32/Stm32SdCard.h`, `src/stm32/sd_msp.cpp` | `IBlockDevice`: SDMMC1 på `HAL_SD` (polling, 4 bitar, hårdvaruflödeskontroll) och stiften |
| `hal/ResetCause.h` | Omstartsorsaken på ESP32 och STM32 (`RCC->RSR`) |

### Format i flashminnet

En sektor på 4 KB = ett huvud på 16 byte (`magic "OPBB"`, en löpande `seq`, `millis()` vid öppningen, flygningsnumret, formatversionen, en kontrollbyte) + poster efter varandra. En post korsar inte en sektorgräns; sektorns svans är `0xFF`.

En post: `[typ u8][längd u8][data][CRC-8]`, data börjar med `t_us` (`micros()`). De första posterna i en flygning är `SCHEMA`: texten `"16 IMU t_us:I gx:h/10 ..."` – fältnamnet, Python-tecknet för `struct`, divisorn. Avkodaren tar fälten från loggen, så ett nytt fält i en post innebär att man ändrar strukturen och schemasträngen i `BlackBoxFormat.h` (deras storlekar korskontrolleras med `static_assert`); avkodaren behöver inte ändras.

### Hämtningsprotokoll

Kommandon är en rad efter STX-byten (`0x02`), och konsolen lämnar den vidare till den svarta lådan:

```
PC:  \x02bb list\n
FC:  BB:STATE state=idle free_kb=... total_kb=... flights=... rate_bps=...
     BB:FLIGHT n=3 sectors=234 kb=936 seconds=41 start=1
     BB:END
PC:  \x02bb get 3 2000000\n
FC:  BB:SEND n=3 sectors=234 baud=2000000   (på 115200), växlar sedan till 2 Mbaud
PC:  växlar till 2 Mbaud, skickar 'G'
FC:  234 ramar: A5 5A, u16 nummer, 4096 byte av sektorn, u32 CRC-32
     återgår till 115200, BB:DONE n=3 crc=<CRC-32 för alla sektorer>
```

### Tester

`pio test -e native -f native/test_blackbox_scan` – en stickprovskontroll av ringen mot en fullständig på slumpmässiga historiker (300 ringar × 5 provsteg) och kostnaden på ett SD-område. `pio test -e native-stm32 -f native_stm32/test_blackbox_sd` och `test_app_stm32_*` – FAT32, området, kortdrivrutinen, den svarta lådan på kortet, hela STM32-firmwaren. På kortet – `pio test -e stm32h743-devebox -f test_blackbox_sd` (ett riktigt SD-kort).

`pio test -e native -f native/test_blackbox` – formatet, ringen på en NOR-attrapp av partitionen (radering per sektor, en skrivning sänker bara bitar – att höja en bit räknas som fel), omstarter, strömavbrott, inspelning av en flygning på riktiga `FlightController`/`Autopilot`, start/stopp, händelser, flashöverfyllnad i luften, hämtning. En flygavbild för att kontrollera avkodaren: `OPENPLANE_BLACKBOX_DUMP=/tmp/f.bbl pio test -e native -f native/test_blackbox`.
