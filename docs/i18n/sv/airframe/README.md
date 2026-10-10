# Flygplansstommen Astro-Cargo: modell och utskriftsfiler

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../../airframe/README.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Här finns själva flygplanet: Fusion 360-projektet och STL-filerna för 3D-utskrift. Elektroniken och flygkontrollerkortet beskrivs i [FC_BOARD.md](../FC_BOARD.md), och monteringen och den första flygningen i [pilothandboken](../PILOT_GUIDE.md).

## Modellversion: v2

Den här mappen innehåller **Astro-Cargo v2**. Någon v1 kommer inte att finnas i repositoryt: den första modellen publiceras inte, så den andra blev den första som släpptes.

- [Fusion 360-projekt](../../../../airframe/fusion360/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.f3d), 14 MB;
- [STL-fil för utskrift](../../../../airframe/stl/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.stl), 6 MB.

Filerna är namngivna så att det direkt syns på namnet vad som är fel med den här versionen (detaljer nedan).

> [!WARNING]
> **Kritiska konstruktionsbrister har hittats i v2:**
>
> 1. **Landningsstället är för svagt fäst i flygkroppen.** Fästet klarar inte flygplanets vikt, och flygkroppen spricker vid fästpunkten.
> 2. **Det saknas fäste för kardborrebandet som håller batteriet på plats.**
>
> Båda bristerna åtgärdas i nästa prototyp, **Astro-Cargo v3**. Dess modell dyker upp i den här mappen så snart den är klar. Flyg inte v2 utan ändringar innan dess: förstärk landningsställets fäste och gör själv plats för kardborrebandet.

## Vad som finns var

| Mapp | Vad den innehåller |
|---|---|
| [`fusion360/`](../../../../airframe/fusion360/) | Källprojektet: en `.f3d`-fil (eller ett `.f3z`-arkiv om projektet har flera filer). I det kan du ändra mått och exportera delarna på nytt |
| [`stl/`](../../../../airframe/stl/) | Utskriftsklara delar i STL-format |

## Hur filer ska namnges

- Namnen skrivs med latinska bokstäver och innehåller versionsnumret. Filerna för v2 är namngivna så att namnet berättar om bristerna, men framöver är det bättre att undvika mellanslag och parenteser: `astro-cargo_v3.f3d`, `astro-cargo_v3.stl`. Då blir det lättare att länka till en fil från dokumentationen. Om det finns flera delar får var och en sin egen fil: `fuselage_v3.stl`, `wing_left_v3.stl`.
- Filerna för v3 läggs bredvid (`astro-cargo_v3.f3d`), och filerna för v2 blir kvar, så att man ser exakt vad som rättades.
- Enheten är millimeter. Om projektet använder andra enheter, skriv det bredvid filen.

## Om en fil är för stor

GitHub tar inte emot filer större än 100 MB och börjar varna vid 50 MB. Kontrollera därför filstorleken innan du gör en commit. Lägg inte sådana filer i repositoryt; använd i stället något av följande:

- avsnittet **Releases** på GitHub: en fil kan bifogas till en release och får väga upp till 2 GB;
- [Git LFS](https://git-lfs.com), om filen ska ligga i själva repositoryt och ändras tillsammans med koden;
- extern lagring, med en länk till den i den här README-filen.

Git behandlar filerna `.stl`, `.f3d`, `.f3z`, `.step`, `.stp` och `.3mf` i den här mappen som binära (se [`.gitattributes`](../../../../.gitattributes)): det ändrar inte deras radslut och visar inte skillnader rad för rad.

## Licens

Modellen distribueras på samma villkor som hela projektet: under [OpenPlane License](../LICENSE.md), det vill säga MIT med obligatorisk angivelse av upphovspersonen, förbud mot militär användning och förbud mot att avsiktligt skada människor eller egendom utan deras medgivande. Inom dessa villkor får du skriva ut, ändra och förbättra den, men du måste ange upphovspersonen, Damir Lebedev (Damn / Проклятый).
