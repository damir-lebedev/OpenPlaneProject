# Carte du contrôleur de vol : blocs de connecteurs

> 🌐 Cette page est la traduction de l’[original en russe](../../FC_BOARD.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Une carte porteuse pour l’ESP32-S3 DevKitC-1 (N16R8) : le DevKit s’enfiche dans deux barrettes femelles, et tout autour se trouvent des blocs de connecteurs JST-XH. Ce document répond à trois questions : quels connecteurs assembler en blocs, où placer les condensateurs et ce qu’il faut brancher où. Les broches correspondent à `include/config/Config.h` (le bloc `BOARD_ESP32_S3`).

> Cette carte est conçue pour l’ESP32-S3, l’ancienne carte principale. La carte principale est désormais la STM32H743 (DevEBox H743) : sa carte porteuse n’est pas encore dessinée, les capteurs se branchent donc pour l’instant sur les barrettes de la DevEBox — le brochage est dans [PILOT_GUIDE](PILOT_GUIDE.md).

La carte est conçue pour être **simple face** : les GPIO sont choisis de façon que les broches de chaque bloc se suivent le long de la barrette du DevKit et que les pistes de signal s’ouvrent en éventail sans se croiser. Je n’ai pas vérifié le routage dans un logiciel de CAO. Si cela ne passe pas quelque part, ajoutez un strap en fil côté composants : de un à trois sur une carte de ce type, c’est normal.

**Tous les connecteurs sont des JST-XH, de 1 à 5 contacts.** Les fils ne sont pas soudés à la carte : sur les fils des servos, de l’ESC, du récepteur et des modules, on sertit les boîtiers correspondants. Le XH est détrompé, on ne peut donc pas l’enficher à l’envers. Un contact supporte ~3 A.

---

## 1. Schéma d’implantation

Vue de dessus, côté composants. Les connecteurs USB du DevKit sont sur le bord inférieur.

```
                    haut : antenne de l’ESP, pas de cuivre en dessous
+----------------------------------------------------------------------+
| anneau de GND sur tout le bord                                       |
|  (470 µF)              3V3 --------------------+    +- C5 IMU        |
|  A1 AIL-L               ^  (derrière DevKit)   |    +- C4 BARO       |
|  A2 AIL-R               |                      |    +- C3 I2C-B      |
|  A3 ELE          +------+-------------+        |    +- C2 I2C-A      |
|  A4 ESC <-BEC    | J1-1,2: 3V3        |        +--->+- C1 OLED       |
|  A5 AUX1  <------| J1-4..11   J3-4..7 |-------------^  rail I2C      |
|  A6 AUX2         |                    |                              |
|  A7 RC           |      DevKit        | J3-8..10 --> D1 GPS          |
|  A8 RUD          |     ESP32-S3       |              D2 BUZ   [Q1]   |
|  (470 µF)        |                    |                              |
|  B1 BAT   <------| J1-12,13           | J3-17,18 --> D3 AUX3         |
|  B2 TELEM <------| J1-15,16           |              D4 LIGHT [Q2]   |
|                  | J1-21: 5V          |                  ^           |
|                  +----[USB]--[COM]----+                  | 5V logique|
|    [diode 1N5822] --> 5V logique --- en bas, sous l’USB -+           |
+----------------------------------------------------------------------+
```

Quatre zones :

| Zone | Où | Ce qu’il y a | Alimentation |
|---|---|---|---|
| **A** Servos | le bord gauche, en face de J1-4…11 | 8 × XH-3 : gouvernes, ESC, AUX, récepteur | 5V des servos (sale) |
| **B** Batterie, télémétrie | en bas à gauche, en face de J1-12…16 | capteurs de batterie et de courant, le radiomodem | 5V logique |
| **C** 3V3 | en haut à droite, en face de J3-4…7 | OLED, capteurs sur le rail I2C, connecteurs I2C | 3V3 |
| **D** 5V logique | en bas à droite, en face de J3-8…18 | GPS, buzzer, AUX3, feux | 5V logique |

À gauche se trouve la partie puissance (servos, ESC, batterie), à droite les capteurs et les communications. Le courant des servos reste à gauche et ne passe pas à côté des capteurs.

---

## 2. Les barrettes du DevKit : quelle patte va où

L’ordre des pattes est celui de l’ESP32-S3-DevKitC-1 (2 × 22). Comparez-le à la sérigraphie de votre carte et mesurez l’écart entre les rangées avant de dessiner quoi que ce soit. La numérotation va de l’antenne vers l’USB.

| J1 (rangée gauche) | GPIO | Vers | | J3 (rangée droite) | GPIO | Vers |
|---|---|---|---|---|---|---|
| 1 | 3V3 | 3V3 par le haut → zone C | | 1 | GND | — |
| 2 | 3V3 | (idem) | | 2 | 43 | — (console « COM ») |
| 3 | RST | — | | 3 | 44 | — (console « COM ») |
| 4 | 4 | A1 AIL-L | | 4 | 1 | C1 OLED SDA |
| 5 | 5 | A2 AIL-R | | 5 | 2 | C1 OLED SCL |
| 6 | 6 | A3 ELE | | 6 | 42 | rail I2C : SCL |
| 7 | 7 | A4 ESC | | 7 | 41 | rail I2C : SDA |
| 8 | 15 | A5 AUX1 | | 8 | 40 | D1 GPS : TX |
| 9 | 16 | A6 AUX2 | | 9 | 39 | D1 GPS : RX |
| 10 | 17 | A7 RC (iBUS) | | 10 | 38 | D2 buzzer (via Q1) |
| 11 | 18 | A8 RUD | | 11 | 37 | — PSRAM |
| 12 | 8 | B1 VBAT (CAN) | | 12 | 36 | — PSRAM |
| 13 | 3 | B1 CURR (CAN) | | 13 | 35 | — PSRAM |
| 14 | 46 | — strapping | | 14 | 0 | — bouton BOOT |
| 15 | 9 | B2 TELEM : TX | | 15 | 45 | — strapping |
| 16 | 10 | B2 TELEM : RX | | 16 | 48 | — LED RGB |
| 17 | 11 | libre | | 17 | 47 | D3 AUX3 |
| 18 | 12 | libre | | 18 | 21 | D4 LIGHT (via Q2) |
| 19 | 13 | libre | | 19 | 20 | — USB |
| 20 | 14 | libre | | 20 | 19 | — USB |
| 21 | 5V | entrée du 5V logique (après la diode) | | 21 | GND | masse de la zone D |
| 22 | GND | masse de la zone B | | 22 | GND | masse de la zone D |

Sur le banc, le firmware utilise les GPIO11–14 libres pour le SPI (ICM-42688) ; sur cette carte, le SPI n’est pas câblé.

---

## 3. Règles pour tenir sur une seule couche

1. **Composants traversants dessus, CMS dessous.** Les embases du DevKit, les XH, les électrolytiques et la diode se placent côté composants. Les 0805, 0603 et SOT-23 se soudent directement sur le cuivre. Avec le transfert de toner (méthode du fer à repasser), le dessin du cuivre s’imprime en miroir.
2. **Dans chaque connecteur, le signal est plus près du DevKit, l’alimentation plus loin et la masse vers le bord.** C’est pourquoi, dans tous les connecteurs, le **contact 1 est le plus proche du DevKit**. Les pistes de signal ne croisent alors pas l’alimentation. L’exception est le rail I2C (point 5).
3. **La masse est un plan de cuivre sur tout le pourtour** (un anneau). Les contacts extrêmes des connecteurs y arrivent directement.
4. **D’un côté du DevKit à l’autre, seules deux lignes passent.** Le 3V3 monte depuis J1-1/2, contourne l’extrémité supérieure du DevKit et part vers la droite — au-delà du bord de la carte du DevKit, pas sous l’antenne. Le 5V logique part de J1-21 sous le DevKit vers le bas, puis longe le bord inférieur vers la droite, sous les connecteurs USB (il n’y a là que des pistes, la fiche reste plus haut).
5. **Le rail I2C.** Quatre pistes parallèles au pas de 2,54 mm, qui partent du DevKit vers l’extérieur : **3V3 · GND · SCL · SDA**. C’est l’ordre des broches des modules GY (VCC GND SCL SDA). Les embases et les connecteurs se placent en travers du rail, comme des wagons : chaque piste passe par son propre contact. Le rail commence à J3-6/7, plonge sous l’OLED et remonte le long de la rangée de droite. S’il ne tient pas en hauteur, repliez-le vers la gauche au-dessus de l’extrémité supérieure du DevKit ; l’ordre des lignes est conservé dans le virage.
6. **Ne faites pas passer de pistes entre les pattes du DevKit** : le pas de 2,54 est trop étroit pour le transfert de toner. Sous le DevKit lui-même, c’est possible : il y a 11 mm jusqu’à sa carte.
7. **Les composants traversants sont des straps gratuits.** Une piste passe sans problème sous le corps de la diode (pas des pattes de 12,5–15 mm) et entre les pattes d’un électrolytique (5 mm).
8. **0805 entre les contacts du connecteur.** Avec le pas de 2,5 mm du XH, un condensateur 0805 se soude directement entre les contacts voisins +5V et GND, côté cuivre.
9. **Largeur des pistes :** 5V des servos et masse des servos, à partir de 2 mm ; 5V logique, à partir de 1 mm ; signaux, 0,4–0,5 mm.
10. **Légendes sur la sérigraphie :** le numéro du connecteur (A1, B2…), l’inscription et une flèche près du contact 1.

---

## 4. Blocs de connecteurs

### Bloc A — servos : 8 × XH-3, le bord gauche, en colonne en face de J1-4…11

Les contacts de chaque connecteur :
**1 — signal** (plus près du DevKit) · **2 — +5V des servos** · **3 — GND** (vers le bord).
C’est l’ordre du fil d’un servo : orange, rouge, marron.

| Connecteur | Inscription | Ce qu’il faut brancher | GPIO (patte) |
|---|---|---|---|
| A1 | AIL-L | l’aileron gauche | 4 (J1-4) |
| A2 | AIL-R | l’aileron droit | 5 (J1-5) |
| A3 | ELE | la gouverne de profondeur | 6 (J1-6) |
| A4 | ESC | le variateur : le signal des gaz ; **sur le fil rouge — l’entrée du BEC 5V** | 7 (J1-7) |
| A5 | AUX1 | le servo de largage de charge | 15 (J1-8) |
| A6 | AUX2 | les volets (deux servos par un câble en Y) ou n’importe quel servo | 16 (J1-9) |
| A7 | RC | le récepteur FS-iA6B, port iBUS SERVO (le récepteur est alimenté d’ici) | 17 (J1-10) |
| A8 | RUD | la gouverne de direction + la roue | 18 (J1-11) |

Ce qui se soude encore dans le bloc :

- **Le bus +5V des servos** — la colonne centrale de contacts, une piste de 2 mm minimum. **GND** — la colonne extérieure, qui fait aussi partie de l’anneau de masse.
- **330 Ω (0603)** en série avec chaque ligne de signal, près du connecteur. Si 5 V arrivent un jour sur un contact de signal (servo défectueux, sertissage de travers), la broche de l’ESP survivra.
- **10 kΩ (0603)** entre le signal de l’A4 ESC et la masse : pendant que l’ESP redémarre, aucun parasite n’atteint le variateur.
- **100 nF (0805)** entre les contacts 2 et 3 de chaque connecteur.
- L’**A4 ESC** porte en plus 10 µF + 100 pF (0805) : l’entrée d’alimentation de toute la carte — filtrage basse, haute et très haute fréquence.
- L’**A7 RC** porte en plus 10 µF (0805) : le récepteur est sensible aux chutes de tension.
- **2 × 470 µF 16 V** sur le bus des servos, un à chaque extrémité de la colonne (au-dessus de l’A1 et au-dessous de l’A8) : le plus sur le bus, le moins sur l’anneau. À l’intérieur de la colonne, le moins ne peut pas atteindre l’anneau, et 3 cm de piste large ne jouent aucun rôle pour un électrolytique.

Une embase pour l’ESC supporte ~3 A. Pour 4 à 6 servos MG90S, cela suffit. S’il y a plus de servos, et plus puissants, ajoutez à côté de l’A4 une embase d’alimentation distincte venant du BEC.

### Bloc B — batterie et télémétrie, en bas à gauche, sous les servos

**B1 BAT — XH-5** (le seul XH-5 de la carte : un câble de 12–17 V ne rentrera dans aucun autre connecteur)

| Contact | Quoi | Où sur la carte |
|---|---|---|
| 1 | **VBAT** — le plus de la batterie par un fil fin (le « + » extrême du connecteur d’équilibrage, ou celui du connecteur de la batterie, pas à travers l’ESC) | un diviseur 56 kΩ / 10 kΩ → GPIO8 (J1-12) |
| 2 | vide — un espace entre la tension de la batterie et tout le reste | — |
| 3 | **CURR** — la sortie du capteur de courant | un diviseur 10 kΩ / 15 kΩ → GPIO3 (J1-13) |
| 4 | +5V logique — alimentation du capteur de courant | le bus de 5V logique |
| 5 | GND | l’anneau |

- **Le diviseur VBAT :** 56 kΩ en haut, 10 kΩ en bas, 100 nF en parallèle sur celui du bas. 3S (12,6 V) → 1,91 V, 4S (16,8 V) → 2,55 V, avec de la marge jusqu’à la limite du CAN (~3,1 V). Un fil de masse séparé n’est pas nécessaire : la masse est commune par l’ESC, le contact 5 peut rester non serti.
- **Le diviseur CURR :** 10 kΩ en haut, 15 kΩ en bas, 100 nF en parallèle sur celui du bas. Un capteur à effet Hall de 5 V (ACS758 et similaires) donne au maximum 5 V → 3,0 V sur la broche. Si la sortie du capteur est de 3,3 V, la résistance du haut est de 0 Ω et celle du bas n’est pas soudée.
- Placez les diviseurs juste au niveau du connecteur et prenez la masse sur son contact 5 : une seule piste va alors vers l’ESP.
- Pas encore de capteur de courant ? Ne sertissez simplement pas les contacts 3–4.

**B2 TELEM — XH-4 :** un radiomodem de télémétrie ou iBUS-SENS.

| Contact | Quoi | GPIO (patte) |
|---|---|---|
| 1 | TX → vers le RX du modem | 9 (J1-15) |
| 2 | RX ← depuis le TX du modem | 10 (J1-16) |
| 3 | +5V logique | — |
| 4 | GND | — |

- 10 µF + 100 nF entre les contacts 3 et 4. Pour un modem de 1 W, ajoutez un électrolytique de 470 µF.
- Le firmware ne prend pas encore en charge TELEM : les trois UART sont occupés (console, iBUS, GPS). Pour l’activer, il faut déplacer la console sur l’USB intégré. C’est une modification du firmware ; le connecteur, lui, est déjà câblé.

### Bloc C — 3V3 : l’écran et les capteurs, en haut à droite

Tout ce qui se trouve dans cette zone est sur le **rail I2C** (section 3, point 5) : quatre pistes **3V3 · GND · SCL · SDA** qui partent du DevKit vers l’extérieur. SCL vient du GPIO42 (J3-6), SDA du GPIO41 (J3-7). Le 3V3 arrive par le haut depuis J1-1/2. L’OLED est le plus bas sur le rail, et au-dessus de lui, dans l’ordre, C2–C5.

**C1 OLED — XH-4.** Il prend l’alimentation sur le rail et les données sur son propre bus (GPIO1/2), qui arrivent par le côté intérieur.

| Contact | Quoi | D’où |
|---|---|---|
| 1 | SDA | GPIO1 (J3-4) |
| 2 | SCL | GPIO2 (J3-5) |
| 3 | 3V3 | le rail |
| 4 | GND | le rail |

C’est l’ordre des broches du module OLED (GND VCC SCL SDA) à l’envers, donc le câble passe sans se croiser.

**C2 I2C-A et C3 I2C-B — XH-4**, identiques :

| Contact | Quoi |
|---|---|
| 1 | 3V3 |
| 2 | GND |
| 3 | SCL |
| 4 | SDA |

- **C2** — la boussole : un GY-273 sur un mât (câble droit, l’ordre est celui du module) ou la boussole d’un module GPS. Sur le GPS, seuls GND, SCL et SDA sont sertis : la boussole reçoit son alimentation par le câble du GPS.
- **C3** — le port de réserve : un capteur de vitesse air (MS4525DO), un télémètre, etc.

**C4 BARO — une embase 1×4 pour un module BMP581 :** 1 — VCC, 2 — GND, 3 — SCL, 4 — SDA.

- Sur le module lui-même, soudez des straps en fil **CSB→VCC** et **SDO→GND** (adresse 0x46 ; 0x47 reste libre pour le tube de Pitot). Sans CSB→VCC, la puce passe en mode SPI ; avec SDO en l’air, l’adresse flotte. Les autres broches du module restent en l’air.
- Uniquement le 3V3 du rail : beaucoup de modules BMP581 n’ont pas leur propre régulateur.
- L’ordre des broches diffère selon les modules : vérifiez le vôtre. S’il ne correspond pas, le module va sur un câble vers C3 et l’embase n’est pas montée.
- Par-dessus, un morceau de mousse à pores ouverts (contre le flux d’air et la lumière).

**C5 IMU — une embase 1×8 pour un GY-521 :**

| Contact | Broche du module | Vers |
|---|---|---|
| 1 | VCC | le rail 3V3 |
| 2 | GND | le rail GND |
| 3 | SCL | le rail SCL |
| 4 | SDA | le rail SDA |
| 5, 6 | XDA, XCL | rien |
| 7 | AD0 | vers le plan de GND à l’extérieur du rail (adresse 0x68) |
| 8 | INT | rien |

La façon dont l’IMU est tourné sur la carte n’a pas d’importance : le montage est déterminé par la calibration `o` (PILOT_GUIDE, « Montage de l’IMU »).

Un module MPU-6500 seul (10 broches : VCC GND SCL SDA EDA ECL AD0 INT NCS FSYNC) ne rentre pas dans cette embase — l’ordre des broches est différent. Il va sur un câble vers C3 (VCC, GND, SCL, SDA), avec des straps sur le module : **NCS→VCC** (sinon la puce passe en mode SPI), **AD0→GND** (adresse 0x68) et **FSYNC→GND**.

Les condensateurs du bloc : **10 µF + 100 nF** sur le rail à hauteur de C1 (c’est là que commence le 3V3), **100 nF** entre les contacts 1 et 2 en C2–C5. Les résistances de rappel (pull-up) de l’I2C sont déjà sur les modules ; ne les placez pas sur la carte.

### Bloc D — 5V logique : GPS, buzzer, feux, en bas à droite

Le bus de 5V logique arrive par le bas (de sous le DevKit, le long du bord inférieur) et remonte le long du bord droit à travers les contacts « +5V » de tous les connecteurs du bloc.

**D1 GPS — XH-4.** Il se place juste sous le rail, pour que l’embase du GPS et celle de sa boussole (C2) soient proches.

| Contact | Quoi | GPIO (patte) |
|---|---|---|
| 1 | TX → vers le RX du GPS | 40 (J3-8) |
| 2 | RX ← depuis le TX du GPS | 39 (J3-9) |
| 3 | +5V logique | — |
| 4 | GND | — |

10 µF + 100 nF entre les contacts 3 et 4.

**D2 BUZ — XH-2 :** un buzzer actif de 5 V, pour retrouver l’avion dans l’herbe et pour avertir de l’état de la batterie et de l’ARM.

| Contact | Quoi |
|---|---|
| 1 | le « − » du buzzer → interrupteur Q1 |
| 2 | +5V logique → le « + » du buzzer |

**D3 AUX3 — XH-3 :** 1 — le signal du GPIO47 (J3-17) à travers 330 Ω, 2 — +5V logique, 3 — GND, plus 100 nF entre 2 et 3. Convient pour un bouton, les données d’un ruban de LED, le déclencheur d’une caméra. **N’y accrochez pas de servo :** c’est du 5V logique, son courant passerait par la diode et ferait osciller l’alimentation de l’ESP.

**D4 LIGHT — XH-2 :** un interrupteur pour une charge allant jusqu’à ~0,5–1 A — feux de navigation, phare, électroaimant de largage.

| Contact | Quoi |
|---|---|
| 1 | le « − » de la charge → interrupteur Q2 |
| 2 | +5V logique → le « + » de la charge |

**Les interrupteurs Q1 et Q2** — la même empreinte SOT-23. Sur le BC817 et le Si2302, les pattes se correspondent par leur rôle :

| Patte du SOT-23 | BC817 | Si2302 | Vers |
|---|---|---|---|
| 1 | base | grille | ← 1 kΩ ← GPIO (38 pour Q1, 21 pour Q2) ; 10 kΩ de la patte 1 à GND |
| 2 | émetteur | source | GND (la masse de la zone — J3-21/22) |
| 3 | collecteur | drain | contact 1 du connecteur (D2 / D4) |

- **Q1 (buzzer) :** l’un ou l’autre convient.
- **Q2 (feux) :** **Si2302** — le BC817 chauffe à plusieurs centaines de milliampères.
- Les 10 kΩ maintiennent l’interrupteur ouvert pendant que l’ESP démarre : le buzzer ne hurle pas, les feux ne clignotent pas.
- Si la charge est une bobine (électroaimant, buzzer magnétique), placez une diode SS14 ou 1N4148 en parallèle avec le connecteur, cathode vers le +5V. Prévoyez-lui de la place entre les contacts 1 et 2.

---

## 5. Alimentation et tous les condensateurs

```
 ESC (BEC 5V/5A) ──► A4 ──► bus 5V des servos ──┬──► A1…A8 (servos, récepteur)
                                                │    2×470 µF aux extrémités de la colonne
                                                │
                                                └──► diode 1N5822 ──► 5V logique ──┬──► J1-21 (5V DevKit)
                                                                                   ├──► B1, B2 (capteur de courant, modem)
                                                                                   └──► D1…D4 (GPS, buzzer, AUX3, feux)
 DevKit : son propre régulateur 3V3 ──► J1-1/2 ──► par le haut ──► rail C (OLED, capteurs, connecteurs I2C)
```

- **Il n’y a qu’une seule entrée — A4 ESC.** Un connecteur d’alimentation distinct n’est pas nécessaire.
- **La diode Schottky 1N5822** (3 A, traversante ; le remplaçant CMS est la SS34) — à acheter. Elle fait trois choses :
  - l’USB et le BEC ne se heurtent pas : on peut garder l’USB branché avec la batterie connectée ;
  - quand les servos font chuter le bus, l’électrolytique de la logique ne se décharge pas en retour dans les servos, et l’ESP ne redémarre pas ;
  - son boîtier traversant sert de strap au-dessus de la piste de GND dans le coin près de J1-22.

  L’anode va à l’extrémité basse du bus des servos, la cathode à J1-21. Ne prenez pas une 1N5819 (1 A) : l’ESP, le modem, le GPS et les feux passent tous par la diode.
- Les servos ne sont pas alimentés par l’USB seul ; la diode les coupe. C’est voulu. Le GPS, le modem et le buzzer fonctionnent sur table depuis l’USB seulement si la broche 5V du DevKit fournit l’alimentation depuis l’USB. Certains clones ont là une diode à eux, et alors ils ne fonctionnent pas — c’est normal.
- Les 3,3 V viennent uniquement du régulateur du DevKit et uniquement pour la zone C.

**Tous les condensateurs dans un seul tableau** (céramiques : 0805 ; électrolytiques : 16 V) :

| Où | Quoi | Pourquoi |
|---|---|---|
| Bus des servos, au-dessus de l’A1 et au-dessous de l’A8 | 470 µF + 470 µF | les chutes quand tous les servos sursautent en même temps |
| A4 ESC, entre + et GND | 10 µF + 100 nF + 100 pF | l’entrée d’alimentation : basse, haute et très haute fréquence |
| A1–A3, A5–A8 | 100 nF sur chacun | le bruit des moteurs de servos — à la source |
| A7 RC | + 10 µF | le récepteur |
| 5V logique, à J1-21 | 470 µF + 10 µF + 100 nF | soutient l’ESP quand le BEC chute |
| Rail 3V3, à C1 | 10 µF + 100 nF | alimentation des capteurs |
| C2–C5 | 100 nF sur chacun | |
| B1 : l’entrée du CAN de VBAT et CURR | 100 nF sur chacune, en parallèle avec la résistance du bas | le filtre du CAN |
| B1 : le +5V du capteur de courant | 100 nF entre les contacts 4 et 5 | |
| B2 TELEM | 10 µF + 100 nF (modem de 1 W : + 470 µF) | les pointes de courant de l’émetteur |
| D1 GPS | 10 µF + 100 nF | |
| D3 AUX3 | 100 nF | |

Une céramique 0805 de 10 µF perd jusqu’à la moitié de sa capacité sous 5 V — c’est pris en compte, et il y a de toute façon des électrolytiques à côté.

Points de test : **5VS** (le bus des servos), **5VL** (5V logique), **3V3**, **GND**. Ils sont pratiques pour mesurer au multimètre.

---

## 6. Ce qui se branche où

| Appareil | Connecteur | Remarques |
|---|---|---|
| Les servos des ailerons, de la profondeur et de la direction | A1, A2, A3, A8 | le signal va sur le contact 1 |
| ESC | A4 | le fil rouge est l’entrée du BEC |
| Le récepteur FS-iA6B | A7 | le port iBUS SERVO, un câble ordinaire à 3 fils |
| Le servo de largage de charge, les volets | A5, A6 | |
| GY-521 (MPU6500) | C5, embase | |
| BMP581 | C4, embase | straps sur le module CSB→VCC, SDO→GND (0x46) |
| GY-273 (boussole) ou la boussole du GPS | C2 | loin des fils de puissance, de préférence sur un mât |
| OLED 128×64 | C1 | |
| Capteur de vitesse air et similaires | C3 | |
| GPS u-blox M10 | D1 + C2 | les 6 fils se répartissent dans deux boîtiers : D1 (alimentation, UART) et C2 (boussole) |
| Tension de la batterie, capteur de courant | B1 | |
| Radiomodem / iBUS-SENS | B2 | |
| Buzzer | D2 | |
| Feux, phare | D4 | |

Implantation dans l’avion :

- Montez la carte sur une fixation souple (mousse, coussinets de gel), plus près du centre de gravité : les vibrations du moteur faussent les angles.
- Gardez les fils de puissance (batterie → ESC → moteur) loin de la zone C et de la boussole.
- Le baromètre se place sous de la mousse, l’IMU comme vous voulez (calibration `o`).

---

## 7. Liste des composants

| Composant | Qté | Où |
|---|---|---|
| Embase PBS 1×22 (pour le DevKit) | 2 | |
| XH-3 coudé ou droit | 9 | A1–A8, D3 |
| XH-4 | 5 | B2, C1, C2, C3, D1 |
| XH-5 | 1 | B1 |
| XH-2 | 2 | D2, D4 |
| Embases PBS 1×8 et 1×4 | 1 de chaque | C5, C4 |
| Diode Schottky 1N5822 (ou SS34) | 1 | **à acheter** |
| Électrolytique 470 µF 16 V | 3 (+1 pour un modem puissant) | bus des servos ×2, 5V logique |
| 0805 10 µF | 6 | A4, A7, 5V logique, 3V3, B2, D1 |
| 0805 100 nF | 20 | voir le tableau des condensateurs |
| 0805 100 pF | 1 | A4 |
| 0603 330 Ω | 9 | signaux A1–A8, D3 |
| 0603 10 kΩ | 5 | ESC vers GND, bas de VBAT, haut de CURR, patte 1 de Q1 et Q2 |
| 0603 56 kΩ | 1 | haut de VBAT |
| 0603 15 kΩ | 1 | bas de CURR |
| 0603 1 kΩ | 2 | vers la patte 1 de Q1 et Q2 |
| BC817 ou Si2302 | 1 | Q1 (buzzer) |
| Si2302 | 1 | Q2 (feux) |
| SS14 / 1N4148 | 0–2 | seulement pour des bobines sur D2 et D4 |

La carte fait environ 80×95 mm : la colonne des servos et le rail des capteurs dépassent au-dessus de l’extrémité supérieure du DevKit. Si elle ne tient pas dans le fuselage, le moyen le plus simple de la réduire est de déporter BARO et l’IMU sur des câbles vers C3 et de raccourcir le rail.

---

## 8. Quels GPIO ne pas toucher

0, 45, 46 — le mode de démarrage en dépend ; 19/20 — USB ; 26–37 — la flash et la PSRAM du module N16R8 ; 43/44 — le connecteur « COM » (console) ; 48 — la LED RGB. Après ce plan, seuls les GPIO11–14 restent libres (sur le banc — SPI).

---

## 9. Avant la première mise sous tension

1. Sans le DevKit et sans la batterie, faites un test de continuité : +5V des servos ↔ GND, 5V logique ↔ GND, 3V3 ↔ GND — il ne doit y avoir de court-circuit nulle part.
2. Appliquez le BEC (par l’A4), le DevKit n’étant pas encore enfiché. Sur 5VS, on doit lire 5,0–5,2 V, et sur 5VL 0,3–0,5 V de moins (la chute dans la diode).
3. Enfichez le DevKit et ne branchez que l’USB. Sur 5VS, il y a 0 V : la diode ne laisse pas l’USB entrer dans les servos.
4. Tout ensemble. Le `s` de la console montrera si les capteurs répondent et combien d’erreurs il y a sur l’I2C.

---

## 10. Ce qui a changé par rapport au plan précédent

- **Les broches ont été redistribuées pour une seule couche** (déjà dans `Config.h`) :
  - I2C des capteurs 8/9 → **41/42** ;
  - GPS 15/16 → **39/40** ;
  - AUX1/AUX2 41/42 → **15/16** ;
  - VBAT 3 → **8**, capteur de courant 10 → **3** ;
  - télémétrie 39/40 → **9/10** ;
  - une nouvelle sortie **LIGHT** sur le GPIO21.

  Sur le banc, déplacez deux fils : SDA 8 → 41, SCL 9 → 42. Les autres broches sont en réserve et ne sont pas connectées sur le banc.
- **Un seul BEC par l’ESC** : le connecteur PWR distinct a disparu.
- **Les capteurs de la carte sont sur l’I2C** (comme sur le banc). Les embases SPI ont disparu, les GPIO11–14 sont libres. L’ICM-42688 sait aussi parler en I2C, mais le firmware aura besoin d’une variante I2C pour lui dans `SensorSelection.h`.
- **GPS-MAG a disparu** : la boussole du GPS s’enfiche dans C2.
- **VBAT et le capteur de courant sont réunis** dans un seul XH-5 (B1).
