# Guide du pilote d’OpenPlaneProject

> 🌐 Cette page est la traduction de l’[original en russe](../../PILOT_GUIDE.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Ceci est un guide pratique « quoi brancher où et comment voler » pour ceux qui ont en main un fer à souder et une radio, et qui ne lisent pas le code source. Si vous voulez comprendre l’architecture du code, reportez-vous aux autres documents du dépôt. Ici, il n’est question que de matériel, de canaux, de firmware et de vol.

Dépôt : https://github.com/damir-lebedev/OpenPlaneProject, branche `main`.

Soyons francs d’emblée : le projet est en développement actif et **n’est pas un produit fini**. Le premier prototype a déjà volé, mais avec des réserves, détaillées plus bas dans une section à part. Lisez-la avant de voler, pas après.

---

## Sommaire

1. [Le matériel nécessaire](#le-matériel-nécessaire)
2. [Choix de la carte et brochage](#choix-de-la-carte-et-brochage)
3. [Branchement du récepteur](#branchement-du-récepteur)
4. [Table des canaux RC](#table-des-canaux-rc)
5. [Canaux du pilote automatique](#canaux-du-pilote-automatique)
6. [ARM et failsafe](#arm-et-failsafe)
7. [Flashage de la carte](#flashage-de-la-carte)
8. [Tableau de bord web sur le terrain](#tableau-de-bord-web-sur-le-terrain)
9. [Boîte noire](#boîte-noire)
10. [Check-list avant le vol et sécurité](#check-list-avant-vol-et-sécurité)
11. [Dépannage](#dépannage)
12. [État actuel de la cellule prototype](#état-actuel-de-la-cellule-du-prototype)

---

## Le matériel nécessaire

Le kit de la version actuelle (le firmware a été vérifié dessus) :

- **Une carte STM32H743 DevEBox H743** (MCUDEV) — la carte principale. L’ancienne carte principale, l’ESP32-S3 ci-dessous, reste prise en charge.
- **Une carte ESP32-S3 N16R8** (un clone de DevKitC-1 avec deux USB-C : « USB » et « COM »).
- **Une radio FS-i6 + un récepteur FS-iA6B** (protocole iBUS, 10 canaux). Il faut un seul fil de données : le port iBUS SERVO. La radio doit permettre de régler le failsafe ; ce réglage est obligatoire, voir la section sur le failsafe.
- **2 servos MG90S** pour les ailerons — un par demi-aile (deux servos indépendants, et non un seul pour les deux ailes).
- **1 servo MG90S** pour la gouverne de profondeur.
- **1 servo MG90S** pour la gouverne de direction — la roue directrice du train d’atterrissage est montée sur son axe (pour rouler au sol).
- **Un variateur (ESC)** de 60–80 A avec BEC 5 V (le BEC alimente les servos et le récepteur).
- **Un moteur D3548 1100KV** + **une hélice 10x5**.
- **Une batterie LiPo 3S**.

Capteurs du pilote automatique (tous en I2C ; sans eux, l’appareil vole en mode manuel) :

- **GY-521** — gyroscope + accéléromètre (la carte peut porter un MPU6050 ou, comme chez nous, un MPU6500 — les deux sont pris en charge).
- **BMP581** — baromètre (l’ancien BMP388 reste pris en charge).
- **GY-273** — boussole (chez nous, elle porte un QMC5883P ; le QMC5883L est lui aussi pris en charge).
- En option, un **écran OLED 128×64 SSD1306** (I2C) — un écran d’état embarqué.

Cellule : envergure 1200 mm, corde 250 mm, profil NACA 4412, construction en PETG (impression 3D). Le premier prototype volait avec un ESP32-C3, un moteur D2212 1000KV et un variateur de 40 A.

---

## Choix de la carte et brochage

Le firmware prend en charge quatre cartes ; pour passer de l’une à l’autre, il suffit d’un paramètre de compilation (`pio run -e <nom de l’environnement>`). Chaque carte a son propre brochage, fixé dans le firmware pour l’environnement concerné — ne déplacez pas les fils de votre propre initiative, consultez le tableau de votre carte.

> **Important :** la carte principale est désormais la **STM32H743 (DevEBox H743)** — le démarrage, la console USB, la carte SD, l’iBUS, les servos et le moteur y ont été vérifiés ; les capteurs sont branchés pour la première fois. L’**esp32-s3 (N16R8)** est l’ancienne carte principale ; son brochage a été vérifié sur le banc avec tous les capteurs. L’**esp32-c3** est l’ancien prototype qui a volé. Le brochage de l’**esp32-dev** a été choisi d’après la documentation de la puce et **n’a pas été vérifié sur du vrai matériel**.

### STM32H743 (DevEBox H743) — la carte principale

`pio run -e stm32h743-devebox`, carte MCUDEV DevEBox H743 (STM32H743VIT6). C’est la carte par défaut (`default_envs = stm32h743-devebox`). Le démarrage, la console USB, la carte SD et la boîte noire, la réception iBUS, l’ARM, les servos et le moteur depuis la radiocommande y sont déjà vérifiés ; les capteurs sont branchés pour la première fois.

| Fonction | Broche |
|---|---|
| Aileron, demi-aile gauche | PA0 |
| Aileron, demi-aile droite | PA1 |
| Gouverne de profondeur | PA2 |
| ESC (gaz) | PA3 |
| Gouverne de direction + roue directrice | PD14 |
| iBUS du récepteur (RX) | PE7 |
| I2C des capteurs SDA / SCL (MPU, BMP581, boussole) | PB11 / PB10 |
| I2C de l’OLED SDA / SCL (bus séparé) | PB9 / PB8 |
| GPS : RX (← TX du GPS) / TX (→ RX du GPS) | PD9 / PD8 |
| Télémétrie MAVLink (modem radio) : RX / TX | PD0 / PD1 |
| AUX1 / AUX2 (servos), buzzer | PD15 / PE9, PE15 |
| SPI des capteurs SCK / MISO / MOSI, CS de l’IMU / CS du baromètre | PB13 / PB14 / PB15, PB12 / PD10 |
| Réserve : batterie / capteur de courant (ADC ; le firmware STM32 ne les lit pas encore) | PC0 / PC1 |

La console, le journal et le téléchargement de la boîte noire passent par l’USB-C de la carte (port COM virtuel). Ne pas occuper : PA11/PA12 (USB), PA13/PA14 (SWD), PC8–PC12 et PD2 (emplacement µSD), PE3 et PC5 (boutons K1/K2).

Branchement des capteurs sur le banc (tous les modules fonctionnent en **3,3 V**, et non en 5 V) :

| Module | Broches |
|---|---|
| MPU-6050 / GY-521 (la carte peut porter un MPU6500 — c’est normal) | VCC–3.3V, GND–GND, SCL–PB10, SDA–PB11, AD0–GND, INT/XDA/XCL — ne pas brancher. Module MPU-6500 seul (10 broches) : pareil, plus **NCS–3.3V** (sinon la puce passe en SPI) et FSYNC–GND ; EDA/ECL — ne pas brancher. Puce vers le haut, flèche X vers le nez ; la rotation des axes de la puce se règle avec `IMU_ROTATION_CW_DEG` dans `Config.h` (90 sur notre clone) |
| BMP581 | VCC–3.3V (**3.3V uniquement** : beaucoup de modules n’ont pas leur propre régulateur), GND–GND, SCL–PB10, SDA–PB11, **SDO–GND** (adresse 0x46 ; ne pas la laisser en l’air), **CSB–3.3V** (sinon la puce passe en SPI), INT — ne pas brancher |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–PB10, SDA–PB11, DRDY — ne pas brancher. À l’écart des fils des servos, de l’ESC et du moteur |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–PB8, SDA–PB9 |

Les servos sont alimentés **non pas par la carte**, mais par le BEC du variateur (ou par une alimentation 5 V séparée d’au moins 2 A) ; la masse de toutes les sources est commune. Ne reliez pas le fil rouge de l’ESC au 5 V de la carte tant que l’USB est branché.

### esp32-s3 (N16R8) — l’ancienne carte principale, vérifiée sur le banc

`pio run -e esp32-s3`, carte `esp32-s3-devkitc-1` avec les réglages du module N16R8 (16 Mo de flash, 8 Mo de PSRAM octale).

| Fonction | GPIO |
|---|---|
| Aileron, demi-aile gauche | GPIO4 |
| Aileron, demi-aile droite | GPIO5 |
| Gouverne de profondeur | GPIO6 |
| ESC (gaz) | GPIO7 |
| Gouverne de direction + roue directrice | GPIO18 |
| iBUS du récepteur (RX) | GPIO17 |
| I2C des capteurs SDA / SCL (MPU, BMP581, boussole) | GPIO41 / GPIO42 |
| I2C de l’OLED SDA / SCL (bus séparé) | GPIO1 / GPIO2 |
| Réserve : GPS RX / TX | GPIO39 / GPIO40 |
| Réserve : AUX1 / AUX2 (servos), AUX3, buzzer, LIGHT | GPIO15 / 16, 47, 38, 21 |
| Batterie / capteur de courant (ADC, enregistré par la boîte noire) ; réserve : télémétrie TX / RX | GPIO8 / GPIO3, GPIO9 / GPIO10 |
| Banc uniquement : SPI (ICM42688) SCK / MISO / MOSI / CS | GPIO12 / 13 / 11 / 14 (+ CS du BMP388 — GPIO21) |

> Le bus des capteurs se trouvait auparavant sur GPIO8/9 ; il a été déplacé sur 41/42 pour suivre le tracé
> de la carte du contrôleur de vol. Sur le banc : SDA 8→41, SCL 9→42.

À ne pas utiliser : GPIO0/45/46 (le mode de démarrage en dépend), 19/20 (USB), 26–32 (flash), 33–37 (PSRAM sur la N16R8), 43/44 (connecteur « COM »), 48 (LED RVB). Les broches libres sont déjà réparties en réserve — une carte porteuse avec des connecteurs pour l’avenir : [`FC_BOARD.md`](FC_BOARD.md).

Branchement des capteurs sur le banc (tous les modules fonctionnent en **3,3 V**, et non en 5 V) :

| Module | Broches |
|---|---|
| MPU-6050 / GY-521 (la carte peut porter un MPU6500 — c’est normal) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, AD0–GND, INT/XDA/XCL — ne pas brancher. Module MPU-6500 seul (10 broches) : pareil, plus **NCS–3.3V** (sinon la puce passe en SPI) et FSYNC–GND ; EDA/ECL — ne pas brancher. Puce vers le haut, flèche X vers le nez ; la rotation des axes de la puce se règle avec `IMU_ROTATION_CW_DEG` dans `Config.h` (90 sur notre clone) |
| BMP581 | VCC–3.3V (**3.3V uniquement** : beaucoup de modules n’ont pas leur propre régulateur), GND–GND, SCL–GPIO42, SDA–GPIO41, **SDO–GND** (adresse 0x46 ; ne pas la laisser en l’air), **CSB–3.3V** (sinon la puce passe en SPI), INT — ne pas brancher |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, DRDY — ne pas brancher. À l’écart des fils des servos, de l’ESC et du moteur |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–GPIO2, SDA–GPIO1 |

Les servos sont alimentés **non pas par la carte**, mais par le BEC du variateur (ou par une alimentation 5 V séparée d’au moins 2 A) ; la masse de toutes les sources est commune. Ne reliez pas le fil rouge de l’ESC au 5 V de la carte tant que l’USB est branché.

### esp32-c3 — l’ancien prototype, a volé

`pio run -e esp32-c3`, carte `esp32-c3-devkitm-1`.

| Fonction | GPIO |
|---|---|
| Aileron, demi-aile gauche | GPIO5 |
| Aileron, demi-aile droite | GPIO4 |
| Gouverne de profondeur | GPIO6 |
| ESC (gaz) | GPIO7 |
| Gouverne de direction | — (aucune broche libre) |
| iBUS du récepteur (RX) | GPIO8 |
| I2C SDA (capteurs) | GPIO1 |
| I2C SCL (capteurs) | GPIO3 |

### esp32-dev (l’ESP32 classique ordinaire, 38 broches) — pour le banc et le débogage, N’A PAS VOLÉ

`pio run -e esp32-dev`, carte `esp32dev`.

| Fonction | GPIO |
|---|---|
| Aileron, demi-aile gauche | GPIO13 |
| Aileron, demi-aile droite | GPIO14 |
| Gouverne de profondeur | GPIO27 |
| ESC (gaz) | GPIO26 |
| Gouverne de direction | GPIO25 |
| iBUS du récepteur (RX) | GPIO16 |
| I2C SDA (capteurs) | GPIO21 |
| I2C SCL (capteurs) | GPIO22 |

Son avantage : c’est la carte la plus répandue et la moins chère de la gamme ; elle convient au débogage du firmware sur le banc, mais son brochage n’a pas été vérifié sur du matériel.

---

## Branchement du récepteur

Tout ce qu’il faut du récepteur, c’est **un fil de données iBUS**, qui, sur la plupart des récepteurs compatibles FlySky, est sorti sur un port à part (souvent marqué « iBUS », ou c’est la seule sortie qui n’est pas en PPM). Branchement :

- **TX du récepteur (sortie iBUS)** → **broche RX de la carte** du tableau ci-dessus (GPIO8 sur l’esp32-c3, GPIO17 sur l’esp32-s3, GPIO16 sur l’esp32-dev).
- **Masse (GND) du récepteur** → **GND de la carte**. C’est obligatoire ; sans masse commune, le protocole ne fonctionne pas.
- **Alimentation du récepteur** — par un BEC ou un régulateur séparé, ou par le 5 V de la carte, selon la façon dont vous alimentez habituellement le récepteur dans vos montages ; elle n’est liée à aucune broche précise du firmware.

Le firmware n’envoie rien en retour au récepteur : il se contente d’écouter, il n’est donc pas nécessaire de brancher la ligne TX de la carte.

La vitesse du port iBUS dans le firmware est de 115200 bauds, valeur standard du protocole ; inutile de la changer, car le récepteur maintient lui-même cette vitesse.

---

## Table des canaux RC

La table a été vérifiée sur le banc avec une radio FS-i6 (10 canaux, mode 2) et un récepteur FS-iA6B.

| Canal | Commande de la radio | Nom | Ce que cela fait |
|---|---|---|---|
| CH1 | stick droit ←→ | AILERON | Roulis — ailerons (2000 = vers la droite) |
| CH2 | stick droit ↑↓ | ELEVATOR | Tangage — gouverne de profondeur (2000 = stick vers l’avant, nez vers le bas) |
| CH3 | stick gauche ↑↓ | THROTTLE | Gaz. 1000 µs = coupé, 2000 µs = maximum, sans limitation |
| CH4 | stick gauche ←→ | RUDDER | Gouverne de direction et roue directrice du train d’atterrissage (un seul servo) |
| CH5 | SwA | ARM | Interrupteur ARM — voir la section sur l’ARM plus bas |
| CH6 | SwB | SWB | Par défaut, volets : vers le bas, vers soi — sortis, vers le haut — rentrés (voir plus bas) |
| CH7 | SwC (3 positions) | SWC | Par défaut, mode : en haut MANUAL, au milieu STABILIZE, en bas AUTO_TAKEOFF |
| CH8 | SwD | SWD | Par défaut, RTH (retour à la maison) tant qu’il est activé |
| CH9 | VrA | VRA | Par défaut, force de la stabilisation |
| CH10 | VrB | VRB | Par défaut, vitesse de croisière |

Les canaux CH6–CH10 peuvent être **n’importe quoi, en une seule ligne** de `include/config/Controls.h` : n’importe lequel des 12 modes, des 10 fonctions (volets, frein, largage de charge, géorepérage, buzzer…) et des 7 potentiomètres. À la mise sous tension de la carte, l’affectation réelle s’affiche dans le moniteur série. Tout ce qui concerne les modes et les affectations se trouve dans [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

Toutes les valeurs des canaux utilisent la plage d’impulsion standard du récepteur, 1000–2000 µs, avec 1500 µs pour le centre/neutre.

### Volets (flaperons)

Il n’y a pas de volets distincts : ce sont les ailerons qui jouent ce rôle. **SwB vers le bas** (vers soi) : les deux ailerons s’abaissent en douceur, en une seconde environ, du même angle (`FLAPS_DEPLOYED_US` = 220 µs ≈ 20° de rotation du palonnier du servo — modifiable dans `include/config/Config.h`). Cela augmente la portance pour décoller et atterrir à plus faible vitesse. Le roulis commandé au stick et par le pilote automatique fonctionne comme d’habitude — les ailerons se déplacent en sens opposés, mais désormais autour de la position abaissée. **SwB vers le haut** — ils rentrent avec la même douceur. Sur l’OLED, quand les volets sont sortis, `FL` s’allume sur la première ligne.

En plein roulis avec les volets sortis, l’aileron qui descend arrive en butée avant celui qui monte — c’est normal et cela fonctionne comme un différentiel d’ailerons.

La sortie des volets fait en général cabrer l’appareil — soyez prêt à pousser un peu le stick vers l’avant ; si l’effet est marqué, on le règle en réduisant `FLAPS_DEPLOYED_US`.

---

## Canaux du pilote automatique

En bref, l’affectation par défaut (`include/config/Controls.h`) :

| Interrupteur | Ce qu’il fait |
|---|---|
| **SwC** (CH7) | en haut **MANUAL** · au milieu **STABILIZE** · en bas **AUTO_TAKEOFF** |
| **SwD** (CH8) | **RTH** — retour à la maison tant qu’il est activé |
| **SwB** (CH6) | volets |
| **VrA / VrB** (CH9/10) | force de la stabilisation / vitesse de croisière |

- **STABILIZE — « le stick fixe l’angle ».** Stick à fond : roulis de 45°, tangage de 25° ; relâché, l’avion se remet à l’horizontale tout seul. Les gaz sont à vous.
- **AUTO_TAKEOFF.** Après l’ARM, rien ne se passe tant que vous n’avez pas poussé les gaz au-delà de la moitié. Ensuite : 0–1 s — gaz progressivement jusqu’à 100 %, ailes à plat ; 1–3 s — tangage +15° ; puis +10° jusqu’à ce que vous changiez SwC. Gaz = le maximum entre le stick et le programme.
- **RTH.** Cap sur le point de l’ARM, altitude de 40 m, cercles au-dessus du point de départ. Il faut un GPS avec un fix 3D **avant l’ARM**.

Les 12 modes (ALT_HOLD, ACRO, CRUISE, LOITER, LAUNCH à la main, AUTO_LAND, SOARING, RESCUE…), les capteurs qu’exige chacun d’eux et la façon de l’affecter à un interrupteur sont décrits dans [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md). Un mode choisi depuis le tableau de bord ou la station sol reste actif jusqu’à ce que vous actionniez l’interrupteur de mode.

La stabilisation fait bouger les gouvernes même quand l’appareil **n’est pas armé** — ainsi, sur la table, on voit dans quel sens elle réagit à une inclinaison. L’intégrateur, dans ce cas, ne s’accumule pas.

Signes des angles (comme sur l’OLED et dans le journal) : **roulis R > 0 — aile droite vers le bas, tangage P > 0 — nez vers le haut.**

---

## ARM et failsafe

### Procédure d’ARM

- **ARM :** gaz (CH3) en bas → interrupteur **SwA (CH5) vers le bas, vers soi** (sur la FS-i6, c’est CH5 = 2000 ; vers le haut = 1000). `ArmingManager: ARM` apparaît dans Serial, et `ARMED` sur l’OLED.
- **DISARM :** SwA vers le haut, à l’opposé de soi — instantanément, à tout moment ; le moteur s’arrête aussitôt.
- Si SwA est basculé sur ARM alors que les gaz ne sont pas en bas, ou que les vérifications avant vol n’ont pas été réussies, l’ARM n’a **pas** lieu (la raison est affichée dans Serial). Il faut remettre SwA vers le haut, couper les gaz et le rebasculer vers le bas.
- Si la carte est mise sous tension avec SwA déjà en position ARM (en bas), elle ne s’arme **pas** : le firmware doit d’abord voir SwA en haut (OFF).

**L’ARM bloque réellement les gaz :** tant que l’appareil n’est pas armé, les gaz de l’ESC sont maintenus de force au minimum, quelle que soit la position du stick. Avant l’armement, on vérifie que les capteurs requis par le mode choisi répondent — par exemple, STABILIZE sans IMU opérationnelle ne s’armera pas tant que vous n’aurez pas passé en MANUAL. Comportez-vous quand même comme si l’hélice pouvait se mettre à tourner à tout moment après l’ARM.

### Failsafe

La perte de liaison a une **priorité absolue** sur tout le reste :

- **appareil armé, GPS et point de départ disponibles** — **retour à la maison avec le moteur** (comme sur ArduPilot/INAV) : cap sur le point de départ, altitude de 40 m, cercles au-dessus jusqu’au retour de la liaison. L’OLED affiche `FSRTH`. Se désactive avec `FAILSAFE_RTH = false` dans `Config.h` ;
- **appareil armé, pas de GPS** — **plané** : moteur coupé ; le pilote automatique, dans n’importe quel mode, même MANUAL, maintient les ailes à plat et le nez légèrement sous l’horizon (−3°), et les volets rentrent. L’OLED affiche `RX LOST ... GLIDE`. Si vous réglez `FAILSAFE_GLIDE_ROLL_DEG` = 10–20, l’avion décrira une spirale peu inclinée au-dessus de vous ;
- **non armé** (au sol) ou IMU qui ne répond pas — moteur coupé, ailerons, profondeur et direction au neutre (1500 µs) ; au bout de 10 s sans liaison, le buzzer « je suis là » retentit (s’il est soudé).

Le firmware reconnaît la perte de liaison de deux façons :

1. **Aucune trame iBUS pendant plus de 500 ms** — fil coupé ou récepteur sans alimentation.
2. **Gaz sous 950 µs** — c’est ainsi que le récepteur signale qu’il a perdu la radio. **Cela exige de régler le failsafe dans la radio** (voir plus bas) : en cas de perte de liaison, le FS-iA6B NE cesse PAS d’envoyer des trames, il répète les dernières valeurs des sticks — sans ce réglage, le firmware ne verra pas la perte de liaison et l’avion continuera de voler avec les derniers gaz.

L’ARM n’est **pas** annulé par le failsafe : quand la liaison revient, l’avion obéit de nouveau aux sticks et au mode choisi sans réarmement (actionner l’interrupteur en vol avec les gaz à zéro est plus dangereux).

Contrôle sur la table (hélice démontée) : ARM → éteindre la radio → l’OLED affiche `RX LOST ... GLIDE`, le moteur s’est arrêté ; inclinez l’avion — les gouvernes doivent le ramener à l’horizontale. Rallumez la radio — `RX ok`, et la commande revient aux sticks.

### Réglage du failsafe dans la radio FS-i6 (obligatoire, une seule fois)

L’idée : en cas de perte de liaison, le récepteur doit délivrer des gaz d’environ 900 µs — sous le minimum normal de 1000.

1. `Menu → Functions setup → End points` → voie 3 : réglez le point bas (la valeur de gauche) sur **120 %**. Enregistrez (appui long sur Cancel).
2. Gaz **tout en bas**.
3. `Menu → Functions setup → Failsafe` → Channel 3 → **On**, gaz toujours en bas → enregistrez par un appui long sur Cancel. Le récepteur mémorisera environ 900 µs.
4. Revenez dans `End points` → voie 3 → remettez le point bas à **100 %**. Enregistrez.
5. Vérification : l’ARM n’est pas nécessaire. Éteignez la radio — au bout d’environ 1 s, `RX=LOST(failsafe пульта)` apparaît dans Serial, et une ligne `RX LOST` en inversé sur l’OLED. Rallumez la radio — `RX=OK`.

Gardez le trim des gaz au centre : avec un trim très baissé, les gaz peuvent descendre sous 950 et le firmware prendra cela pour une perte de liaison.

Il y avait ici auparavant un **boost sur CH8** et une limite des gaz à 40 % ; ils ont été supprimés : la limite ménageait un montage 3S1P faible, et les batteries neuves ne craignent pas les pleins gaz. Le boost se déclenchait en outre tout seul si SwD était en haut à la mise sous tension de la carte.

---

## Flashage de la carte

Le firmware se compile avec **PlatformIO** (framework Arduino, C++).

### Installation de PlatformIO

Le plus simple est d’installer l’extension **PlatformIO IDE** dans VS Code (Extensions → chercher « PlatformIO IDE » → Install) ; vous obtenez alors la ligne de commande ainsi que des boutons de compilation pratiques dans l’interface. On peut aussi l’installer avec `pip install platformio` et travailler depuis le terminal — les deux options utilisent les mêmes commandes `pio`.

### Compilation et téléversement

Ouvrez le projet (le dossier du dépôt) dans VS Code avec PlatformIO installé, branchez la carte en USB et exécutez dans le terminal la commande correspondant à votre carte :

```bash
# STM32H743 DevEBox (carte principale)
pio run -e stm32h743-devebox -t upload

# esp32-s3 N16R8 (ancienne carte principale)
pio run -e esp32-s3 -t upload

# esp32-c3 (ancien prototype)
pio run -e esp32-c3 -t upload

# esp32-dev (ESP32 classique 38 broches, pour le banc)
pio run -e esp32-dev -t upload
```

Si vous n’indiquez pas du tout `-e`, la carte par défaut est compilée — `stm32h743-devebox`.

**STM32 DevEBox :** le premier flashage passe par l’USB DFU : un strap BT0→3V3, appuyez sur RST, puis la commande ci-dessus (Windows a besoin du pilote WinUSB pour « STM32 BOOTLOADER », installé avec Zadig). Ensuite, la touche `D` dans la console redémarre la carte dans le bootloader toute seule, et le strap n’est plus nécessaire. La console passe par le même USB-C.

**esp32-s3 :** la carte a deux connecteurs USB-C. Le téléversement et Serial passent par le connecteur **« COM »** (pont CH343 ; sous Windows, « USB-Enhanced-SERIAL CH343 »). Le connecteur « USB » (l’USB natif de la puce) n’est pas nécessaire au travail, mais il peut rester branché — il ne gêne en rien.

### Moniteur série

Pour voir la sortie de débogage (état des canaux, ARM, capteurs) directement dans la console par USB :

```bash
pio device monitor -b 115200
```

La vitesse doit absolument être de 115200 ; sinon vous verrez un charabia illisible au lieu du texte. Toutes les 10 secondes, une ligne `SYS` est affichée : la fréquence de la boucle (elle doit être d’environ 500 Hz), le temps de boucle moyen et le pire sur 10 s, les compteurs iBUS, la mémoire libre. Le reste passe par les canaux activés dans le menu du journal (voir plus bas).

### Console : menu et journal (moniteur série)

Une touche agit immédiatement ; Entrée n’est pas nécessaire (dans un moniteur qui envoie ligne par ligne : la lettre + Entrée). Les calibrations et le contrôle des sorties ne fonctionnent que lorsque l’appareil n’est pas armé.

| Touche | Ce qu’elle fait |
|---|---|
| `h` | **menu principal** (textuel, entrées numérotées) |
| `l` | menu « quoi écrire dans le journal » |
| espace | mettre le journal en pause / reprendre |
| `s` | état détaillé de tous les capteurs (y compris les compteurs d’erreurs I2C) |
| `i` | recalibrer le gyroscope et faire le contrôle avant vol de l’IMU — 2 s, ne pas bouger l’avion |
| `o` | **calibration du montage de l’IMU** — une seule fois, après avoir installé la carte dans l’avion (voir plus bas) |
| `m` | calibration de la boussole — pendant 15 s, faire tourner la carte ou l’avion autour de tous les axes. Le résultat est enregistré en flash et survit à un redémarrage |
| `p` | contrôle des sorties : l’impulsion réelle sur chaque broche |

**Journal par canaux.** Chaque type de données forme sa propre ligne, avec son préfixe, et chacun a son mode : **désactivé**, **sur changement** (la ligne n’apparaît que lorsque les valeurs ont réellement changé — le tremblement des sticks et le bruit des capteurs ne comptent pas) ou **en continu** (toutes les 0,2 / 0,5 / 1 / 2 s — la période se règle dans le même menu).

| Canal | Ce qu’il affiche | Par défaut |
|---|---|---|
| `STAT` | liaison, ARM, mode, volets, IMU et baromètre en bon état ou non | sur changement |
| `RC` | canaux de la radio, µs | désactivé |
| `OUT` | sorties vers les gouvernes et l’ESC, µs | désactivé |
| `ATT` | roulis, tangage, cap | désactivé |
| `AP` | pilote automatique : consignes et corrections | désactivé |
| `ALT` | altitude, vitesse verticale, consigne d’ALT_HOLD | désactivé |
| `MAG` | cap d’après la boussole | désactivé |
| `GPS` | fix, satellites, coordonnées, vitesse | désactivé |
| `IMU` | gyroscope et accéléromètre | désactivé |
| `SYS` | fréquence et durée de la boucle, mémoire (toutes les 10 s) | activé |

Tant que le menu est ouvert, le journal reste muet pour que le menu ne défile pas ; après la sortie, tous les canaux activés sont réaffichés. Le choix est enregistré en flash à la sortie du menu et survit à un redémarrage. Quand l’appareil est armé, il s’applique tout de suite, mais n’est écrit qu’après le DISARM : une écriture en flash arrête la boucle de vol pendant environ 0,4 s.

### Montage de l’IMU : comme vous voulez, une seule calibration

La carte avec l’IMU peut être installée dans l’avion **comme cela vous arrange** — sous n’importe quel angle, sur le côté, à l’envers : le firmware détermine lui-même où se trouvent son nez et son dessus. On le fait **une seule fois** après l’installation (et de nouveau si la carte a été déplacée) :

1. L’avion sur la table, sans radio, moteur non armé. Dans le moniteur série, appuyez sur `o`.
2. **Étape 1 :** l’avion repose à plat, comme en vol horizontal (pour un avion à roulette de queue, calez quelque chose sous la queue). Ne pas toucher pendant environ 3 s.
3. **Étape 2 :** relevez le **nez** de 30 à 60°, ailes à plat, et maintenez immobile pendant environ 1 s.
4. **Étape 3 :** remettez le nez, abaissez l’**aile droite** de 30 à 60° et maintenez pendant environ 1 s.

Chaque étape est validée automatiquement (le journal affiche « засчитано », c’est-à-dire « compté »). À la fin s’affiche le résultat (« нос = +Y чипа, верх = −Z чипа », c’est-à-dire « nez = +Y de la puce, haut = −Z de la puce ») et « установка сохранена » (« montage enregistré »). Si vous vous êtes trompé (nez abaissé au lieu d’être relevé, aile gauche au lieu de la droite), la calibration est rejetée avec une explication ; il suffit de refaire `o`. Le résultat est stocké en flash et survit à un redémarrage. Vérification : nez vers le haut → P sur l’OLED devient positif ; aile droite vers le bas → R devient positif.

Tant qu’il n’y a pas de calibration du montage, l’ancienne méthode s’applique : la carte doit être posée puce vers le haut, et la rotation se règle avec `IMU_ROTATION_CW_DEG` dans `Config.h`.

### Contrôle avant vol à la mise sous tension

À chaque mise sous tension, l’IMU passe environ 2 s à calibrer le gyroscope et, en même temps, se vérifie lui-même :

- **l’avion est immobile** — si on le tenait dans les mains ou si on le déplaçait à ce moment-là, le décalage du gyroscope sera faux ;
- l’accéléromètre au repos indique 1g ;
- **le « haut » correspond à la calibration du montage** — si la carte a été déplacée ou retournée, cela se voit tout de suite (un avion posé sur sa roulette de queue ou sur une pente n’est pas un problème ; la tolérance est de 45°).

**Allumez l’avion alors qu’il est immobile.** Il n’a pas besoin d’être à plat si le montage est calibré (sinon, la position à la mise sous tension devient l’horizon). Le résultat figure dans le journal : `предполётная проверка пройдена` (« contrôle avant vol réussi ») ou `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА — <причина>` (« CONTRÔLE AVANT VOL ÉCHOUÉ », suivi de la cause). En cas d’échec, **l’ARM en STABILIZE et en AUTO_TAKEOFF est interdit** (la cause est affichée à la tentative), et le pilote automatique ne donne aucune correction dans aucun mode, y compris le plané en cas de perte de liaison : avec des angles faux, il commanderait dans le mauvais sens. L’ARM en MANUAL reste possible. Pour corriger : posez l’avion immobile et reconnectez la batterie (ou appuyez sur `i`) ; si la carte a été déplacée, appuyez sur `o`.

### Écran OLED

Si l’OLED est branché (GPIO1/GPIO2), il affiche ceci 5 fois par seconde :

```
RX ok disarm STAB        <- liaison / ARM / mode (si la liaison est perdue, la ligne est en inversé)
R  +1.2 P  -0.4          <- roulis / tangage, °
Alt +0.3 Vz +0.1         <- altitude depuis le point de mise sous tension, m / vitesse verticale, m/s
Hdg 123  Thr 1000        <- cap de la boussole / gaz envoyés à l’ESC, µs
L1500 R1500 E1500        <- PWM des ailerons et de la profondeur, µs
Loop 500Hz max 1100us    <- fréquence et pire durée de la boucle
```

---

## Tableau de bord web sur le terrain

La carte crée son propre point d’accès Wi-Fi — pas besoin d’un routeur domestique ni d’Internet, et tout fonctionne directement sur le terrain depuis un téléphone.

**Comment se connecter :**

1. Sur un téléphone ou un ordinateur portable, ouvrez la liste des réseaux Wi-Fi.
2. Connectez-vous au réseau **`OpenPlane-Debug`**, mot de passe **`12345678`**.
3. Ouvrez dans le navigateur l’adresse **`http://192.168.4.1`**.

Aucune application à installer — c’est une page web ordinaire.

**Ce qu’on peut faire depuis un téléphone sur le terrain, sans la moindre programmation :**

- Regarder les **barres en direct des 10 canaux RC** — pratique pour vérifier que la radio et le récepteur envoient bien ce que vous bougez sur le stick, avant même de brancher les servos.
- Voir l’état de chaque sortie (aileron gauche/droit, profondeur, ESC) — le canal est-il connecté côté logiciel.
- Voir si les capteurs (IMU, baromètre) répondent, s’ils sont soudés — le tableau de bord affiche honnêtement soit les données réelles (roulis/tangage/altitude), soit une mention explicite que le capteur est physiquement absent ou ne répond pas.
- **Changer le mode du pilote automatique** avec des boutons (manuel / stabilisation / décollage automatique / maintien d’altitude) directement depuis la page — sans la radio.
- **Régler les coefficients du régulateur PID** (pour le roulis et le tangage) via un formulaire sur la page — utile pour affiner peu à peu la stabilisation sans reflasher.

La portée de ce point d’accès est, en pratique, de quelques dizaines de mètres : c’est le Wi-Fi ordinaire d’un ESP32, pas une télémétrie longue portée. C’est un outil de réglage sur la table, sur le banc et à côté du terrain — pas pour piloter l’appareil en vol à distance.

---

## Boîte noire

Le firmware de l’ESP32-S3 enregistre lui-même chaque vol en flash : tout ce que les capteurs ont vu, ce qu’ont fait les sticks, où sont allés les servos et ce qu’a décidé le pilote automatique. Il n’y a rien à faire :

- elle **enregistre** à partir du moment où l’appareil est armé et où les gaz sont relevés (plus les 10 s précédentes) ;
- elle **s’arrête** 10 s après le DISARM — ou si un avion armé reste immobile, moteur coupé, pendant 30 s (il s’est posé ou écrasé, et le DISARM a été oublié) ;
- la perte de liaison, le moteur à zéro et le vol plané n’arrêtent **pas** l’enregistrement.

À la mise sous tension, le moniteur série indique combien de place il y a pour un vol (la console écrit en russe ; la ligne ci-dessous signifie « attend l’ARM et les gaz | effacé devant 12,9 Mo (≈ 11 min) sur 13,9 Mo | vols 1 ») :

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

**Après le vol**, branchez l’USB (le connecteur COM), fermez le moniteur série et téléchargez le vol :

```bash
python tools/blackbox.py download
```

Le vol atterrit dans le dossier `blackbox/` avec son analyse : `summary.txt` (résumé et événements), `events.txt` et des tableaux CSV par capteur. La boîte noire efface elle-même les anciens vols quand il lui faut de la place — **téléchargez après chaque sortie**. Le menu dans la console, c’est la touche `k`. Tous les détails sont dans [BLACKBOX.md](BLACKBOX.md).

**Sur la carte STM32H743**, la boîte noire écrit sur une carte SD : la carte est formatée en FAT32 et préparée une seule fois sur un PC (`python tools/blackbox.py sd-prepare E:`). Après un vol, on peut le télécharger par USB avec la même commande `download`, ou bien retirer la carte et décoder le fichier directement depuis elle : `python tools/blackbox.py ring E:/BLACKBOX.BIN`.

---

## Check-list avant vol et sécurité

Lisez cette section en entier **avant** la première mise sous tension, et non après un incident.

### Obligatoire avant tout essai sur le banc

- [ ] **L’hélice est physiquement RETIRÉE** si vous vérifiez les canaux, l’ARM, le tableau de bord, si vous réglez le PID, ou si vous mettez simplement la carte sous tension pour la première fois avec un nouveau brochage. L’ESC peut faire tressaillir le moteur à n’importe quelle étape d’un essai — ce n’est pas un risque hypothétique, c’est le comportement normal à la première mise sous tension.
- [ ] La batterie LiPo a été contrôlée (gonflement, dommages), chargée avec un chargeur LiPo adapté, et elle est stockée et chargée sur une surface ininflammable.
- [ ] La radio est allumée et ses canaux ont été vérifiés sur le tableau de bord (`http://192.168.4.1`) **avant** de brancher la batterie à l’ESC.

### Avant le vol

- [ ] Un terrain dégagé, sans personnes ni constructions dans un rayon suffisant pour un avion de 1200 mm d’envergure en pilotage manuel, en cas de comportement anormal des servos ou de l’aile (voir plus bas la section sur les limites du prototype — la fixation du moteur et l’aile ne sont pas encore renforcées au carbone).
- [ ] Les trois surfaces bougent dans le bon sens — vérifiez-le sur la table avant chaque sortie, sans vous fier à votre souvenir de la fois précédente :
  - stick droit vers la droite → **aileron droit vers le haut, gauche vers le bas** ;
  - stick droit vers soi → **profondeur vers le haut** ;
  - stick gauche vers la droite → **gouverne de direction et roue directrice vers la droite** ;
  - SwB (volets) vers le bas → **les deux ailerons descendent en douceur**, et le roulis au stick continue de les écarter en sens opposés ;
  - en STABILIZE, inclinez l’avion aile droite vers le bas → **aileron droit vers le bas, gauche vers le haut** (les gouvernes le ramènent à l’horizontale) ; nez vers le bas → **profondeur vers le haut**.
  Si quelque chose ne va pas, changez le `*_REVERSED` correspondant dans `include/config/Config.h` (section « Sens des servos »), et non l’inversion dans la radio : sinon le stick et le pilote automatique divergeront.
- [ ] À la mise sous tension, l’avion était immobile, et le journal ou le tableau de bord n’affiche pas `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА` ; quand vous inclinez l’avion dans vos mains, P et R sur l’OLED changent dans le bon sens.
- [ ] Le failsafe est réglé dans la radio et vérifié : éteignez la radio → `RX LOST` sur l’OLED ou dans Serial (voir la section sur le failsafe).
- [ ] La portée de la radio a été vérifiée et la batterie de la radio est chargée.
- [ ] À la mise sous tension, le journal affiche `BlackBox: ... стёрто впереди N МБ (≈M мин)` — il y a assez de place pour le vol. Le vol précédent a été téléchargé (`python tools/blackbox.py download`), si vous en avez besoin.
- [ ] Gardez les mains et le visage loin de l’hélice à tout moment où une LiPo est branchée à l’ESC — après l’ARM, les gaz répondent aussitôt au stick, sans autre avertissement. DISARM — SwA vers le haut.
- [ ] Assurez-vous de pouvoir physiquement couper l’alimentation rapidement (accès au connecteur de la LiPo), au lieu de ne compter que sur le failsafe en cas de perte de signal.

### Sécurité générale avec les LiPo

- Ne laissez jamais une LiPo en charge sans surveillance.
- Ne branchez ni ne débranchez la LiPo de l’ESC en vous tenant près du plan de rotation de l’hélice.
- Transportez et stockez les LiPo dans un sac ou un coffret de protection.

---

## Dépannage

**« Pas de signal du récepteur » / le RX semble normal, mais les canaux du tableau de bord ne bougent pas**
Vérifiez que le fil de données du récepteur est branché exactement sur la broche RX iBUS du tableau de votre carte (GPIO8/GPIO17/GPIO16), qu’il n’est pas confondu avec la masse ou l’alimentation, et que les masses du récepteur et de la carte sont reliées. Si le brochage correspond et que les fils sont intacts, mais qu’il n’y a toujours pas de signal, vérifiez que le récepteur est bien associé (bind) à la radio et que sa sortie est réglée sur iBUS, et non sur PPM/SBUS.

**Un capteur (IMU, baromètre, boussole) affiche « ne répond pas » / NO_RESPONSE**
Le firmware indique honnêtement que le capteur ne répond pas, au lieu de sortir des zéros. Vérifiez : (1) l’alimentation du module — 3.3V et la masse de la carte ; (2) SDA/SCL — sur les broches I2C de votre carte précisément ; (3) l’adresse sur la ligne : MPU 0x68 (AD0 à GND), BMP581 0x46 (SDO à GND, CSB à 3.3V ; 0x47 si SDO est à 3.3V), BMP388 0x76 (SDO à GND, **CSB à 3.3V** — sinon la puce est en mode SPI), QMC5883P 0x2C, QMC5883L 0x0D. Si le capteur répond tantôt oui, tantôt non (ou répond à une adresse étrangère), c’est un mauvais contact sur la plaque d’essai : appuyez sur VCC/GND/SDA/SCL et, de préférence, alimentez chaque module directement depuis le 3.3V/GND de la carte. La commande `s` de la console affiche les compteurs d’erreurs I2C de chaque capteur.

**Les angles sur l’OLED sont mélangés (nez en haut change R et non P) ou ont le mauvais signe**
Faites la calibration du montage de l’IMU (`o`, voir « Montage de l’IMU ») — elle ne dépend ni de la façon dont la puce est soudée sur le module, ni de la position du module dans l’avion. Sans elle : sur les clones du GY-521, la puce est parfois soudée tournée par rapport aux flèches imprimées — tournez les axes dans `Config.h` → `IMU_ROTATION_CW_DEG` (0/90/180/270). Vérification : nez en haut → P devient positif, aile droite vers le bas → R devient positif.

**ARM refusé : « IMU: ... »**
Le contrôle avant vol de l’IMU n’a pas été réussi (voir « Contrôle avant vol à la mise sous tension ») : l’avion a bougé à la mise sous tension — posez-le immobile et rebranchez la batterie ; le « haut » ne correspond pas à la calibration — la carte a été déplacée, faites `o` ; « la carte n’est pas posée puce vers le haut » — le montage n’est pas calibré, faites `o`.

**Un servo ou l’ESC réagit au mauvais stick / ne réagit pas**
La commande `p` de la console mesure l’impulsion réelle sur chaque sortie (GPIO4–7) et la compare à celle attendue. Si tout est « OK » et que le servo ne bouge pas, le problème est au-delà de la carte : (1) le servo n’est pas alimenté (BEC/5V, masse commune) ; (2) le fil de signal n’est pas sur la bonne broche ; (3) la mécanique est bloquée. « НЕ СОВПАДАЕТ » (« NE CORRESPOND PAS ») signifie que le problème est dans le firmware ou les périphériques — signalez-le au développeur.

**Le moteur ne tourne pas du tout, alors que les gaz sur l’OLED ou le tableau de bord suivent le stick**
Vérifiez que la LiPo est branchée à l’ESC et que l’appareil est réellement armé — avant l’ARM, les gaz vers l’ESC sont forcés au minimum, et ce n’est pas une panne. Un ESC qui a vu à la mise sous tension des gaz non minimaux peut bipper sans cesse et ne pas s’armer — rebranchez la batterie avec les gaz en bas. Si `ArmingManager: ARM` n’apparaît pas après avoir abaissé SwA, regardez Serial — le firmware imprime la cause (gaz pas en bas, un capteur ne répond pas, un capteur requis par le mode actuellement choisi sur CH7) ; remettez SwA en haut, éliminez la cause et rabaissez-le.

**Le moteur cale ou saccade à l’accélération**
Si l’ESC est alimenté par une alimentation de laboratoire, c’est la limite de courant de l’alimentation qui est atteinte : même sans hélice, le moteur tire brièvement plusieurs ampères, la tension s’effondre et l’ESC redémarre. Relevez la limite de courant ou utilisez une LiPo. Les parasites d’un tel redémarrage peuvent bloquer le pont USB de la carte (le port « COM » ne s’ouvre plus) — rebranchez le câble.

**Après le flashage, la carte ne répond pas / le point d’accès `OpenPlane-Debug` n’apparaît pas**
Assurez-vous que le téléversement (`pio run -e <votre carte> -t upload`) s’est terminé sans erreur et que vous avez flashé exactement l’environnement de la carte que vous tenez en main (l’esp32-c3 se distingue de l’esp32-s3 et de l’esp32-dev non seulement par les broches, mais aussi par la puce — un firmware pour une autre puce ne s’installera pas sur la carte, ou s’installera de travers). Vérifiez la sortie du moniteur série (`pio device monitor -b 115200`) juste après le redémarrage de la carte — elle y imprime à quelle étape de setup() elle en est.

---

## État actuel de la cellule du prototype

Pour que les attentes restent honnêtes :

- Le premier prototype **a déjà volé**. Des problèmes ont été relevés : **résistance insuffisante de la fixation du moteur** et **résistance insuffisante de l’aile** — l’aile doit être renforcée au carbone. Les servos demandent aussi un réglage supplémentaire. Tenez-en compte en planifiant vos propres vols — ce n’est pas une réserve abstraite, mais une défaillance réelle, déjà survenue sur ce prototype.
- **Le banc esp32-s3 est monté avec tous les capteurs** (GY-521 avec un MPU6500, BMP388 en I2C, GY-273 avec un QMC5883P, OLED) — tous répondent, et la boucle tourne à 500 Hz. Les modes du pilote automatique ont été vérifiés sur la table, mais **pas encore éprouvés en vol**.
- Le baromètre par défaut est désormais le BMP581 (le BMP388 du banc a été vérifié en réel ; le BMP581 n’a pas encore été testé sur le matériel). L’altitude est relative au point de mise sous tension. L’altitude absolue au-dessus de la mer se calcule d’après l’atmosphère standard, sans correction météo.
- La boussole QMC5883P doit être calibrée (`m` dans la console) sur l’avion déjà assemblé — près du moteur et des fils, les décalages diffèrent de ceux de la plaque d’essai. Le cap n’a pas encore de compensation d’inclinaison et n’est utilisé par aucun mode.
- La licence est l’OpenPlane License : MIT avec mention obligatoire de l’auteur (Damir Lebedev), interdiction de l’usage militaire et interdiction de nuire intentionnellement aux personnes et aux biens sans leur consentement, voir [LICENSE](LICENSE.md).

Si vous construisez votre propre appareil d’après ce guide, faites-le voler d’abord en pilotage manuel (MANUAL), et ensuite seulement passez au pilote automatique.
