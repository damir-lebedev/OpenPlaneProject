# Boîte noire

> 🌐 Cette page est la traduction de l’[original en russe](../../BLACKBOX.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Le firmware enregistre lui-même chaque vol dans la mémoire flash intégrée de la carte : capteurs, sticks, sorties vers les servos, décisions du pilote automatique, événements. Après le vol, l’enregistrement est téléchargé par USB et décodé en tableaux CSV.

Elle fonctionne sur deux cartes :

| Carte | Où elle enregistre | Combien ça contient (à ~20 Ko/s) |
|---|---|---|
| **ESP32-S3 N16R8** | une partition de 13,9 Mo de la flash intégrée | environ **11 minutes** |
| **STM32H743** (DevEBox — principale ; WeAct) | un fichier sur une carte SD, [plus bas](#carte-sd-stm32h743) | 64 Mo — environ **55 minutes**, la taille est fixée par le fichier |

Sur les autres cartes (ESP32-C3, ESP32 ordinaire), il n’y a pas de support de stockage : la boîte noire est désactivée et ne gêne pas le vol.

---

## Quand elle enregistre

| | Condition |
|---|---|
| **Début** | Armé **et** gaz relevés (le stick ou l’ESC au-dessus de `THROTTLE_LOW_US`). Les **10 s précédentes** sont aussi enregistrées — le moment de l’ARM et l’attente avant le décollage |
| | Redémarrage dû à une défaillance (panique, chien de garde, chute d’alimentation) — enregistrement dès le premier cycle et pendant au moins 60 s : si cela s’est produit en vol, on voit ce qui s’est passé ensuite |
| | À la main depuis la console (`k` → `r`) — pour le banc |
| **Arrêt** | **10 s après le DISARM** |
| | Armé, mais le moteur est arrêté et l’avion est **immobile depuis 30 s** — il s’est posé ou écrasé, et le DISARM a été oublié |
| | À la main (`k` → `r`) |
| **Pas d’arrêt** | Perte de liaison, failsafe, moteur à zéro en vol, plané, atterrissage sans DISARM tant que l’avion roule |

« Immobile » signifie tout cela à la fois : rotation inférieure à 5 °/s sur tous les axes, l’accéléromètre indique 1g ± 0,1, presque aucune vitesse verticale d’après le baromètre, et d’après le GPS et le tube de Pitot (s’il y en a un) une vitesse inférieure à 2 m/s. En vol, un tel calme pendant 30 secondes d’affilée n’existe pas.

## Ce qui est enregistré

| Enregistrement | Fréquence | Ce qu’il contient |
|---|---|---|
| `IMU` | à chaque cycle, 500 Hz | gyroscope (°/s), accéléromètre (g), durée du travail du cycle de contrôle (µs) |
| `CTRL` | 100 Hz | roulis/tangage/cap, consignes du pilote automatique, sticks du pilote, commandes finales, **les 7 sorties** (µs), composantes du PID de roulis et de tangage (P, I, D), gaz du pilote et du pilote automatique, volets, mode, indicateurs (ARM, liaison, failsafe, capteurs vivants...), fonctions activées |
| `RC` | 50 Hz | les 10 canaux de la radio, compteurs de trames iBUS (valides et corrompues) |
| `BARO` | à chaque mesure (~50 Hz) | pression, température, altitude, vitesse verticale, consigne d’altitude |
| `MAG` | jusqu’à 50 Hz | le champ sur trois axes, cap |
| `GPS` | à chaque solution | coordonnées, altitude, vitesse, route, satellites, fix, précision |
| `AIR` | jusqu’à 50 Hz | tube de Pitot : différence de pression, vitesse indiquée et vraie, densité |
| `NAV` | 10 Hz | point de départ (distance, relèvement), cap et consigne de cap, vitesse de navigation, source du cap, phases du lancement à la main et du vol à voile, auto-trim |
| `POWER` | 10 Hz | tension de la batterie et sortie du capteur de courant (les diviseurs de la carte du contrôleur de vol, [FC_BOARD.md](FC_BOARD.md), bloc B) |
| `SYS` | 1 Hz | fréquence et pire cycle de la boucle de contrôle, mémoire libre, compteurs iBUS, température de l’IMU, file de la boîte noire, enregistrements perdus, l’écriture en flash la plus longue, espace libre |
| `EVENT` | sur événement | ARM/DISARM, refus d’ARM avec la cause, changement de mode, liaison perdue/rétablie, capteur en panne/revenu, fix GPS, point de départ enregistré, fonctions des interrupteurs, géorepérage, protection contre le décrochage, phases du lancement à la main et du vol à voile |

Au début de chaque vol figurent les paramètres : le firmware (date de compilation), la cause du démarrage et du dernier redémarrage, quels capteurs sont installés et s’ils ont réussi le contrôle avant vol, les coefficients du PID (y compris les modifications faites depuis le tableau de bord), les trims, les valeurs importantes de `Config` et les affectations des interrupteurs.

---

## Comment s’en servir

### Avant le vol

Il n’y a rien à faire. À la mise sous tension, le moniteur série affiche l’état (la console écrit en russe ; la ligne ci-dessous signifie « attend l’ARM et les gaz | effacé devant 12,9 Mo (≈ 11 min) sur 13,9 Mo | vols 1 ») :

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

« Effacé devant » indique ce qui tiendra dans le prochain vol. Après la mise sous tension, la boîte noire passe quelques secondes (après un long vol, jusqu’à une minute) à préparer de la place : elle efface les anciens enregistrements. Pendant ce temps, la boucle de vol au sol se fige parfois pendant ~0,15 s — les gouvernes peuvent donner un coup avec retard, c’est normal. **En vol, la flash n’est jamais effacée.**

### Après le vol : le téléchargement

1. Branchez l’USB au connecteur **COM**. Fermez le moniteur série (il retient le port).
2. Lancez dans le dossier du projet :

   ```bash
   python tools/blackbox.py download          # le dernier vol
   python tools/blackbox.py download --all    # tous
   python tools/blackbox.py list              # ce qui se trouve sur la carte
   ```

   Il faut `pyserial` : `pip install pyserial`. Ou le Python de PlatformIO, qui l’a déjà : `%USERPROFILE%\.platformio\penv\Scripts\python tools\blackbox.py download`.

3. Le vol est téléchargé dans le dossier `blackbox/` (~200 Ko/s : 10 minutes de vol prennent environ une minute) et est aussitôt décodé à côté, dans un dossier du même nom.

Pendant le téléchargement, la boucle de vol est arrêtée ; il ne fonctionne donc que sans ARM.

### Ce que contient le dossier du vol

| Fichier | Ce que c’est |
|---|---|
| `summary.txt` | Le résumé : durée, fréquences, plages d’angles, d’altitudes, de vitesses, de tensions, le pire cycle de contrôle, enregistrements perdus, tous les événements |
| `events.txt` | Les paramètres du vol et tous les événements dans l’ordre du temps |
| `IMU.csv`, `CTRL.csv`, `RC.csv`, ... | Un tableau par type d’enregistrement |

Le temps dans tous les tableaux est `time_s`, en secondes depuis le début de l’enregistrement (ARM et gaz) ; la pré-écriture est négative. Les valeurs sont déjà en unités : degrés, g, mètres, m/s, microsecondes d’impulsion. Dans `CTRL.csv`, le mode est ajouté par son nom (`mode_name`), les fonctions sous forme de liste (`features_on`), et les indicateurs sont répartis en colonnes 0/1 (`armed`, `rx_lost`, `fs_glide`, `imu_ok`...).

Les CSV s’ouvrent dans Excel/LibreOffice, mais pour tracer des courbes en fonction du temps, [PlotJuggler](https://github.com/facontidavide/PlotJuggler) est plus pratique : File → Load Data → CSV, colonne de temps `time_s`.

Le fichier `.bbl` est une image brute de la flash ; on peut le décoder de nouveau : `python tools/blackbox.py decode blackbox/flight_001_....bbl`.

### Console : `k`

Dans le moniteur série, la touche `k` ouvre le menu de la boîte noire : état, liste des vols, `r` — démarrer/arrêter l’enregistrement à la main (pour vérifier sur le banc), `e` — effacer tous les vols (avec confirmation par `y`, ~40 s).

---

## Place dans la flash

- Les vols sont écrits en anneau. Quand la place manque, la boîte noire, au sol, efface **les plus anciens vols en entier** jusqu’à ce qu’il y ait 10 Mo libres devant (`BLACKBOX_MIN_FREE_BYTES`, ~9 minutes).
- **Le dernier vol enregistré n’est jamais effacé** — seulement par l’enregistrement suivant, s’il n’a pas eu assez de place.
- Si la place effacée est épuisée en vol, l’enregistrement se poursuit dans une file en PSRAM (4 Mo, ~3 minutes des dernières données) ; après l’atterrissage et le DISARM, la boîte noire libère de la place et l’écrit. Un vol plus long que toute la partition (~11 min) ne tient pas en entier : le début est conservé, la fin est perdue.
- C’est pourquoi **il faut télécharger le vol après chaque sortie** — surtout la première.

## Fiabilité

- Une coupure d’alimentation à n’importe quel moment (chute, batterie débranchée) : tout est conservé, sauf les ~15 dernières ms. Les enregistrements incomplets sont écartés grâce au CRC — dans `summary.txt`, c’est la ligne « Недописанных записей (CRC) » (le résumé est en russe ; cela signifie « Enregistrements incomplets (CRC) »).
- Le numéro du vol, la tête de l’anneau et la liste des vols sont reconstitués à partir des secteurs eux-mêmes : il n’y a aucune « carte » à part qui pourrait être corrompue.
- Le téléchargement vérifie le CRC-32 de chaque secteur et de tout le vol.

## Effet sur le vol

- La boucle de vol se contente de déposer un instantané dans une file en PSRAM — quelques microsecondes. Une tâche distincte sur le cœur 0 écrit en flash, une page (256 octets) à la fois, **juste après un cycle de contrôle** : une écriture en flash arrête les deux cœurs de l’ESP32 pendant 0,6–0,9 ms, et elle tombe dans la pause entre les cycles.
- Mesure sur le banc (une DevKit sans capteurs, deux passes de 30–40 s d’enregistrement) : l’intervalle entre les cycles est de 2,00 ms, 99,2–99,7 % des intervalles sont dans 1,9–2,1 ms, le plus long est de 2,5 ms, et aucun cycle n’a été manqué ; le travail du cycle est le même que sans enregistrement. Le cycle ne tressaute sensiblement qu’au sol sans ARM, pendant que la boîte noire vérifie et efface de la place (lecture d’un bloc de 64 Ko : une pause de ~3 ms ; effacement : ~0,15 s).
- Avec des capteurs, un cycle prend ~0,7 ms, et l’écriture d’une page tient quand même dans les 1,3 ms restantes. Vérification après le premier vol : dans `summary.txt`, les lignes « Такт IMU » et « Цикл: худший такт » (en russe : « Cycle de l’IMU » et « Boucle : pire cycle »).

---

## Carte SD (STM32H743)

Sur la STM32H743, la boîte noire écrit sur une carte SD (emplacement µSD sur SDMMC1, 4 bits, 24 MHz). La carte reste une **FAT32** ordinaire : à sa racine se trouve un fichier créé à l’avance, `BLACKBOX.BIN`, dans lequel le firmware écrit des blocs bruts sans jamais toucher à la table FAT ni au répertoire. Il n’y a donc rien qui puisse être corrompu en cas de coupure d’alimentation en vol, et le fichier peut simplement être copié sur un PC.

### Préparation de la carte (une seule fois)

1. Formatez la carte en **FAT32** (pas en exFAT ; Windows propose FAT32 pour les cartes jusqu’à 32 Go).
2. Avec la carte dans un lecteur, sur un PC :

   ```bash
   python tools/blackbox.py sd-prepare E:              # 64 Mo, E: est le lecteur de la carte
   python tools/blackbox.py sd-prepare E: --size 256   # ou plus
   ```

   Le fichier est créé d’un seul tenant sur une carte vide et rempli de `0xFF` ; le premier secteur est un repère de service « l’anneau est vide ». Si le fichier n’est pas contigu (la carte n’est pas vide et très fragmentée) ou s’il n’existe pas, la console en indiquera la cause à la mise sous tension et la boîte noire sera désactivée.
3. Insérez la carte dans la carte. À la mise sous tension (la console écrit en russe : « Carte SD : 15204 Mo, SDMMC 24 MHz, 4 bits ; fichier BLACKBOX.BIN : ok », puis la ligne d’état, puis « prête en 300 ms ») :

   ```
   SD-карта: 15204 МБ, SDMMC 24 МГц, 4 бита; файл BLACKBOX.BIN: ок
   BlackBox: ждёт ARM и газ | стёрто впереди 0.7 МБ из 64.0 МБ | полётов 0
   BlackBox: готов за 300 мс
   ```

   « Effacé devant » augmente en arrière-plan : la carte vérifie la place à ~2,5 Mo/s.

### Récupérer le vol

- **Par la carte, via USB**, comme sur l’ESP32 : `python tools/blackbox.py download` (la console de la STM32 est en USB CDC, la vitesse de téléchargement est de ~400 Ko/s, 1 Mo prend moins de 3 s). `list`, `--all` et `--flight N` fonctionnent de la même façon.
- **En retirant la carte** : le fichier `BLACKBOX.BIN` de la carte est décodé directement en CSV —

  ```bash
  python tools/blackbox.py ring E:/BLACKBOX.BIN              # tous les vols -> blackbox/
  python tools/blackbox.py ring E:/BLACKBOX.BIN --list       # seulement les lister
  ```

  Le fichier est un anneau de secteurs : l’utilitaire reconstitue lui-même les vols d’après les numéros de secteur, y compris ceux qui passent par la fin du fichier.

### Ce qui a été mesuré sur la carte

DevEBox H743 + carte de 16 Go (le test `test_blackbox_sd`, [TESTING.md](TESTING.md#tests-sur-la-carte-stm32)) :

| | |
|---|---|
| Reconnaissance de la carte | 12–18 ms, 4 bits, 24 MHz |
| Écriture d’une page de 256 o | en moyenne 2,3–3,7 ms, **pire cas 60–190 ms**, ~75–110 Ko/s soutenus (il en faut ~20 Ko/s) |
| Lecture | un secteur de 4 Ko — 4,2 Mo/s ; un bloc aléatoire — 0,6 ms |
| Effacement | 64 Ko — 13 ms ; toute la zone de 64 Mo — 20–28 s |
| Mise sous tension | avec l’anneau vide — 0 ms (grâce au repère) ; avec des vols — 0,3 s (vérification par échantillonnage de ~530 lectures) ; une vérification complète de 64 Mo prendrait ~20 s |
| 20 s d’enregistrement en temps réel (IMU à 500 Hz) | aucun enregistrement perdu, 0 erreur |
| La tâche de vol pendant l’enregistrement | écart de période de 2 ms — **1 µs** (une tâche simulée de priorité maximale à côté de l’enregistrement) |

La pire écriture de page est le « nettoyage » interne de la carte ; la file en RAM (384 Ko ≈ 19 s de flux) survit à de telles pauses. Les cartes bon marché sont celles qui diffèrent le plus sur ce point : avant de voler, il vaut la peine de vérifier la carte avec le test `test_blackbox_sd` (la pire écriture doit être inférieure à 250 ms — la limite de la spécification SD).

### En quoi elle diffère de la flash de l’ESP32

- **La tâche d’écriture** (`bbox`, priorité 2) est préemptée par la tâche de vol (5) au milieu d’un accès à la carte — et non « dans la pause du cycle », comme sur l’ESP32 avec son arrêt des cœurs. Le transfert se fait avec le contrôle de flux matériel du SDMMC : sans lui, la FIFO débordait lors de la préemption (sur la carte, c’étaient `HAL_SD_ERROR_RX_OVERRUN` et la console et l’enregistrement figés pendant plusieurs secondes).
- **La file est en RAM**, de 384 Ko (`BLACKBOX_RING_STM32_BYTES`), et non dans 4 Mo de PSRAM.
- **La vérification à la mise sous tension se fait par échantillonnage** : les vrais secteurs de l’anneau forment un arc continu, ~500 en-têtes sont lus, et les limites de l’arc et des vols sont affinées par dichotomie. Le résultat est le même que celui d’une vérification complète ; si le tableau ne concorde pas — on fait la complète.
- **Le repère « anneau vide »** dans le premier secteur du fichier : pour ne pas vérifier pendant des secondes la zone vide à chaque mise sous tension. Il est posé quand tout est effacé et quand une vérification complète n’a rien trouvé ; il est retiré avant la première écriture.
- **Les erreurs de la carte** (retirée, défaut du bus) vont dans le journal une fois par seconde sous la forme de l’événement « носитель: ошибок записи … » (en russe : « support : erreurs d’écriture … ») ; une page perdue laisse un trou `0xFF`, et le décodage du secteur s’arrête là (comme dans `tools/blackbox.py`), les autres secteurs sont intacts.

## Réglages (`include/config/Config.h`, section « Boîte noire »)

| Constante | Par défaut | Signification |
|---|---|---|
| `BLACKBOX_RING_BYTES` | 4 Mo | La file en PSRAM (sans PSRAM : `BLACKBOX_RING_NO_PSRAM_BYTES`, 32 Ko) |
| `BLACKBOX_RING_STM32_BYTES` | 384 Ko | STM32 : la file en RAM |
| `BLACKBOX_SD_FILE`, `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN`, 256 Mo | STM32 : le fichier sur la carte et le plafond de la partie utilisée |
| `BLACKBOX_PREROLL_MS` | 10 000 | Combien enregistrer avant le début |
| `BLACKBOX_POSTROLL_MS` | 10 000 | Combien enregistrer après le DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Immobile avec l’ARM — arrêt |
| `BLACKBOX_LANDED_GYRO_DPS`, `_ACCEL_G`, `_CLIMB_MS`, `_SPEED_MS` | 5, 0,1, 0,5, 2 | Ce qui compte comme « immobile » |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Enregistrement après un redémarrage sur défaillance — pas moins que cela |
| `BLACKBOX_MIN_FREE_BYTES` | 10 Mo | Combien garder effacé pour le prochain vol |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | La pause entre les effacements au sol |
| `BLACKBOX_IMU_DIVIDER` | 1 | L’IMU tous les N cycles : 2 donne 250 Hz et un enregistrement ~25 % plus long |
| `BLACKBOX_VBAT_DIVIDER`, `_CURRENT_DIVIDER` | 6,6, 1,667 | Les diviseurs de la batterie et du capteur de courant sur la carte |

La table des partitions est `partitions_blackbox.csv` : l’application 2 Mo (le firmware fait maintenant ~0,9 Mo), la boîte noire 13,9 Mo, le coredump 64 Ko. La partition NVS est restée à sa place — les calibrations de l’IMU et de la boussole et les trims sont conservés après le passage à cette table. Il n’y a pas de second emplacement pour les mises à jour par voie hertzienne (OTA).

---

## Pour le développeur

Le code se trouve dans `include/telemetry/BlackBox*.h` :

| Fichier | Quoi |
|---|---|
| `BlackBoxFormat.h` | Le format : en-tête de secteur, types et structures des enregistrements, schémas des champs, CRC-8/CRC-32 |
| `BlackBoxStorage.h` | Un anneau de secteurs sur `IFlashRegion` : recherche de la tête à la mise sous tension, liste des vols, écriture par pages, effacement des anciens vols par étapes |
| `BlackBoxRing.h` | Une file d’enregistrements entre les cœurs (spinlock), qui écarte le plus ancien |
| `BlackBox.h` | Instantanés dans le cycle, démarrage/arrêt, événements, la tâche d’écriture, téléchargement par UART |
| `hal/esp32/Esp32FlashPartition.h` | `IFlashRegion` au-dessus d’`esp_partition` |
| `hal/SdFileRegion.h`, `storage/Fat32File.h` | `IFlashRegion` au-dessus d’un fichier sur une carte FAT32 : recherche du fichier (FAT en lecture seule), blocs incomplets, effacement par `0xFF`, le repère « anneau vide » |
| `hal/stm32/Stm32SdCard.h`, `src/stm32/sd_msp.cpp` | `IBlockDevice` : SDMMC1 sur `HAL_SD` (interrogation, 4 bits, contrôle de flux matériel) et les broches |
| `hal/ResetCause.h` | La cause du redémarrage sur ESP32 et STM32 (`RCC->RSR`) |

### Format dans la flash

Un secteur de 4 Ko = un en-tête de 16 octets (`magic "OPBB"`, un `seq` continu, `millis()` à l’ouverture, le numéro du vol, la version du format, un octet de contrôle) + les enregistrements à la suite. Un enregistrement ne franchit pas la limite du secteur ; la fin du secteur est `0xFF`.

Un enregistrement : `[type u8][longueur u8][données][CRC-8]`, les données commencent par `t_us` (`micros()`). Les premiers enregistrements d’un vol sont `SCHEMA` : le texte `"16 IMU t_us:I gx:h/10 ..."` — le nom du champ, le caractère `struct` de Python, le diviseur. Le décodeur prend les champs dans le journal ; un nouveau champ dans un enregistrement revient donc à modifier la structure et la chaîne du schéma dans `BlackBoxFormat.h` (leurs tailles sont recoupées par `static_assert`), sans toucher au décodeur.

### Protocole de téléchargement

Les commandes sont une ligne après l’octet STX (`0x02`), que la console transmet à la boîte noire :

```
PC:  \x02bb list\n
FC:  BB:STATE state=idle free_kb=... total_kb=... flights=... rate_bps=...
     BB:FLIGHT n=3 sectors=234 kb=936 seconds=41 start=1
     BB:END
PC:  \x02bb get 3 2000000\n
FC:  BB:SEND n=3 sectors=234 baud=2000000   (à 115200), puis passe à 2 Mbauds
PC:  passe à 2 Mbauds, envoie 'G'
FC:  234 trames : A5 5A, u16 numéro, 4096 octets du secteur, u32 CRC-32
     revient à 115200, BB:DONE n=3 crc=<CRC-32 de tous les secteurs>
```

### Tests

`pio test -e native -f native/test_blackbox_scan` — une vérification de l’anneau par échantillonnage comparée à une vérification complète sur des historiques aléatoires (300 anneaux × 5 pas de sondage), et le coût sur une zone de SD. `pio test -e native-stm32 -f native_stm32/test_blackbox_sd` et `test_app_stm32_*` — FAT32, la zone, le pilote de carte, la boîte noire sur la carte, tout le firmware de la STM32. Sur la carte — `pio test -e stm32h743-devebox -f test_blackbox_sd` (une vraie carte).

`pio test -e native -f native/test_blackbox` — le format, l’anneau sur un NOR simulé de la partition (effacement par secteurs, une écriture ne fait que baisser des bits — remonter un bit compte comme une erreur), redémarrages, coupures d’alimentation, enregistrement d’un vol sur les vrais `FlightController`/`Autopilot`, démarrage/arrêt, événements, débordement de la flash en vol, téléchargement. Une image de vol pour vérifier le décodeur : `OPENPLANE_BLACKBOX_DUMP=/tmp/f.bbl pio test -e native -f native/test_blackbox`.
