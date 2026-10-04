# Référence du pilote automatique d’OpenPlane

> 🌐 Cette page est la traduction de l’[original en russe](../../AUTOPILOT_GUIDE.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Ce que sait faire le pilote automatique, comment activer chaque fonction et comment l’affecter à n’importe quel interrupteur ou potentiomètre de la radio **en une seule ligne**.

> D’abord, en toute franchise. Tous les modes sont vérifiés par des tests unitaires et par des simulations de vol en boucle fermée (`test/native/test_sim` : tout le firmware pilote un modèle d’avion). Le modèle est simplifié, et les coefficients de `Config.h` sont des valeurs de départ : **chaque mode s’essaie d’abord à plus de 50 m d’altitude, le doigt sur l’interrupteur MANUAL**. Jusqu’ici, seul le mode manuel a volé (le premier prototype, sur un ESP32-C3) ; la stabilisation a été vérifiée sur le banc : les gouvernes réagissent aux inclinaisons dans le bon sens. La carte STM32H743 compile et passe les mêmes tests ; sur le matériel, la carte DevEBox a été vérifiée sans capteurs : la carte SD, la boîte noire et la commande manuelle des servos et du moteur depuis la radio (filmée en vidéo) ; aucun capteur n’y a encore été branché, et les modes du pilote automatique n’y ont pas été essayés.

---

## Sommaire

1. [Comment ça marche, en une minute](#comment-ça-marche-en-une-minute)
2. [Disposition par défaut de la radio](#disposition-par-défaut-de-la-radio)
3. [Affecter une fonction en une seule ligne](#affecter-une-fonction-en-une-seule-ligne)
4. [Modes](#modes)
5. [Fonctions (interrupteurs)](#fonctions-interrupteurs)
6. [Potentiomètres](#potentiomètres)
7. [Perte de liaison, géorepérage, point de départ](#perte-de-liaison-géorepérage-point-de-départ)
8. [Un tube de Pitot fait maison](#un-tube-de-pitot-fait-maison)
9. [Station sol : tableau de bord Wi-Fi et MAVLink](#station-sol-tableau-de-bord-wi-fi-et-mavlink)
10. [Ordre de réglage d’un nouvel avion](#ordre-de-réglage-dun-nouvel-avion)
11. [Contrôle avant vol du pilote automatique](#contrôle-avant-vol-du-pilote-automatique)
12. [Ce dont chaque mode a besoin](#ce-dont-chaque-mode-a-besoin)

---

## Comment ça marche, en une minute

```
sticks ──────┐
             ├─► PilotSwitches (config/Controls.h) ─► mode, fonctions, potentiomètres
interrupteurs┘                                               │
                                                             ▼
capteurs (IMU, baromètre, boussole, GPS, tube de Pitot) ─► Autopilot ─► commande des gouvernes et des gaz
                                                             │
                                FlightController : volets, charge, caméra, buzzer, failsafe
                                                             ▼
                                                ailerons · profondeur · direction · ESC · AUX1 · AUX2
```

- Le **mode** décide qui pilote : le pilote (MANUAL), le pilote avec un assistant (STABILIZE, ALT_HOLD, ACRO), le pilote automatique avec les corrections du pilote (CRUISE, LOITER, RTH…).
- Les **fonctions** s’activent par-dessus n’importe quel mode : volets, aérofrein, largage de charge, géorepérage…
- Les **potentiomètres** modifient en douceur une valeur : la force de la stabilisation, la vitesse de croisière, le rayon des cercles…
- Dans les modes avec stabilisation, **le stick fixe l’angle** et non le braquage de la gouverne : on lâche le stick, l’avion revient seul à l’horizontale.
- La panne d’un capteur ne « secoue » jamais l’avion : pas d’IMU — les gouvernes restent au pilote ; pas de baromètre — le pilote tient l’altitude ; pas de GPS — pas de navigation, et les modes qui en ont besoin se comportent sans danger (voir [le tableau](#ce-dont-chaque-mode-a-besoin)).

---

## Disposition par défaut de la radio

FS-i6 + FS-iA6B, iBUS, 10 canaux (`config/Channels.h`).

| Canal | Organe de la radio | Par défaut |
|---|---|---|
| CH1–CH4 | sticks | roulis, tangage, gaz, direction (ne peuvent pas être réaffectés) |
| CH5 | **SwA** | **ARM** (en bas = armé, seulement avec les gaz en bas ; ne peut pas être réaffecté) |
| CH6 | SwB | volets (`Feature::FLAPS`) |
| CH7 | **SwC** (3 positions) | en haut **MANUAL** · au milieu **STABILIZE** · en bas **AUTO_TAKEOFF** |
| CH8 | SwD | **RTH** — retour au point de départ, tant qu’il est actif |
| CH9 | VrA | force de la stabilisation (`Knob::STAB_GAIN`) |
| CH10 | VrB | vitesse de croisière (`Knob::CRUISE_SPEED`) |

À la mise sous tension de la carte, le moniteur série affiche la disposition réelle — ce qui est effectivement flashé (le firmware écrit en russe ; « вверх / середина / вниз » signifie en haut / au milieu / en bas) :

```
SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (вверх / середина / вниз)
SwB (CH6): FLAPS, пока включён
SwD (CH8): RTH, пока включён
VrA (CH9): крутилка STAB_GAIN
VrB (CH10): крутилка CRUISE_SPEED
```

> Les canaux 7 à 10 ne sont pas sortis par défaut sur la FS-i6. Menu de la radio : **Functions setup → Aux. channels**, puis affectez SwC, SwD, VrA, VrB.

---

## Affecter une fonction en une seule ligne

Tout se trouve dans un seul fichier : `include/config/Controls.h` :

```cpp
constexpr Binding BINDINGS[] = {
    Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_AUTO_TAKEOFF),
    Bind::feature(Channels::SWB, Feature::FLAPS),
    Bind::mode   (Channels::SWD, MODE_RTH),
    Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
    Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
};
```

| Forme de la ligne | Ce qu’elle fait |
|---|---|
| `Bind::modes(canal, haut, milieu, bas)` | un interrupteur à trois positions choisit le mode |
| `Bind::modes(canal, haut, bas)` | à deux positions — deux modes |
| `Bind::mode(canal, mode)` | le mode **prend le dessus** sur les autres tant que l’interrupteur est actif ; une fois coupé, le mode de l’interrupteur de modes revient |
| `Bind::feature(canal, fonction)` | la fonction agit tant que l’interrupteur est actif |
| `Bind::knob(canal, potentiomètre)` | potentiomètre : centre = valeur de `Config.h`, extrémités = minimum et maximum |

« Actif » signifie que le canal dépasse 1750 µs (sur la FS-i6 — interrupteur vers le bas, vers soi). Tant que la première trame du récepteur n’est pas arrivée, tous les canaux sont considérés comme inactifs : à la mise sous tension, rien ne sortira et rien ne sera largué.

### Recettes toutes prêtes

```cpp
// Planeur de thermiques : SwD — vol à voile, SwB — auto-trim, VrB — rayon des cercles
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_SOARING),
Bind::feature(Channels::SWB, Feature::AUTO_TRIM),
Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Élève : seulement la stabilisation, RESCUE sur le « bouton de panique », sticks doux
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_ALT_HOLD, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_RESCUE),
Bind::feature(Channels::SWB, Feature::GEOFENCE),
Bind::knob   (Channels::VRA, Knob::RATES),
Bind::knob   (Channels::VRB, Knob::MAX_BANK),

// Prise de vue et livraison : caméra stabilisée, largage de charge, cercles au-dessus d’un point
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_CRUISE, MODE_LOITER),
Bind::feature(Channels::SWB, Feature::PAYLOAD_DROP),
Bind::feature(Channels::SWD, Feature::CAMERA_STAB),
Bind::knob   (Channels::VRA, Knob::CAMERA_TILT),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Lancement à la main sans train d’atterrissage : SwD — LAUNCH, volets progressifs au potentiomètre
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_LAUNCH),
Bind::feature(Channels::SWB, Feature::AIRBRAKE),
Bind::knob   (Channels::VRA, Knob::FLAPS),
Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
```

### Le compilateur attrape les erreurs

Le tableau est vérifié à la compilation (`static_assert`), avant que le firmware n’arrive dans l’avion :

- les sticks et SwA (ARM) ne peuvent pas être affectés, et le numéro de canal doit être inférieur à `Channels::COUNT` ;
- un canal n’a qu’une seule affectation ;
- l’interrupteur de choix des modes (`Bind::modes`) ne peut pas être plus d’un.

Si plusieurs interrupteurs `Bind::mode` sont actifs en même temps, c’est la ligne la plus haute du tableau qui l’emporte.

---

## Modes

Le mode change quand un interrupteur est **basculé**. Un mode activé depuis le tableau de bord ou la station sol reste en vigueur jusqu’à ce que le pilote actionne de nouveau un interrupteur de mode. Sur l’OLED s’affiche un nom court (entre parenthèses).

### MANUAL (MAN)
Gouvernes = sticks, comme sans contrôleur de vol. Seules les fonctions (volets, aérofrein, charge) et l’auto-trim fonctionnent. **Le principal mode de sécurité** : gardez-le toujours sur un interrupteur sous le doigt.

### STABILIZE (STAB) — « le stick fixe l’angle »
Le stick de roulis fixe l’angle de roulis jusqu’à `MAX_BANK_DEG` (45°, potentiomètre `MAX_BANK` de 15…60°), et le stick de tangage fixe l’angle jusqu’à `STAB_MAX_PITCH_DEG` (25°). On lâche, et l’avion se remet seul à l’horizontale. Les gaz restent au pilote. L’intégrateur du PID ne s’accumule que près de la cible (±10°) ; c’est pourquoi, après une manœuvre brusque, l’avion ne « dépasse » pas l’horizon.
**Il faut :** l’IMU. Sans IMU — comme MANUAL.

### ALT_HOLD (ALT) — « garde l’altitude »
Comme STABILIZE en roulis, tandis que la gouverne de profondeur tient l’altitude grâce au baromètre. On bouge le stick de tangage — on pilote soi-même ; on le lâche — l’avion tient la **nouvelle** altitude. Les gaz sont au pilote (ajoutez des gaz, sinon la vitesse ne suffira pas pour monter).
**Il faut :** l’IMU + le baromètre.

### ACRO — « le stick fixe la vitesse de rotation »
Stick à fond — 180°/s. On lâche, et l’avion **garde l’assiette qu’il avait** (même sur le dos), et le gyroscope amortit les rafales. Pour la voltige.
**Il faut :** l’IMU. Sans IMU — gouvernes = sticks.

### CRUISE (CRZ) — « garde le cap, l’altitude et la vitesse »
L’avion vole en ligne droite à l’altitude actuelle, avec les gaz automatiques (potentiomètre `CRUISE_SPEED` : 30…55…85 % de gaz et, avec un tube de Pitot, **vitesse air** de 10…14…22 m/s). Le stick de roulis sert à virer (on lâche, et il garde le nouveau cap), celui de tangage à changer d’altitude. Avec un tube de Pitot, la protection contre le décrochage fonctionne. Le cap vient du GPS (à une vitesse sol > 3 m/s ; retour à la boussole sous 2 m/s), sinon de la boussole, sinon du gyroscope. En simulation, avec un vent de travers de 4 m/s, la route sol reste à ±5° et l’altitude à ±3 m.
**Il faut :** l’IMU + le baromètre ; le GPS ou la boussole pour un cap sans dérive.

### LOITER (LOIT) — « tourne ici »
Des cercles dans le sens horaire au-dessus du point où le mode a été activé, de rayon 50 m (potentiomètre `LOITER_RADIUS` de 25…150 m), à l’altitude actuelle, avec les gaz automatiques. Le guidage se fait par champ de vecteurs : de loin, l’avion rejoint le cercle par la tangente, et sur le cercle il tient une inclinaison anticipée. Sans GPS — simplement un cercle à inclinaison constante sur place.
**Il faut :** l’IMU + le baromètre + le GPS.

### RTH — « à la maison »
Cap sur le point de départ (le point de l’ARM), altitude `RTH_ALTITUDE_M` = 40 m (en dessous, il monte en route ; au-dessus, il y reste). Au-dessus du point de départ — des cercles du rayon LOITER, jusqu’à ce que le pilote reprenne les commandes. Sans GPS ou sans point de départ — des cercles sur place. Le même mode est enclenché par la perte de liaison et par le géorepérage.
**Il faut :** l’IMU + le baromètre + le GPS avec un point de départ.

### AUTO_TAKEOFF (TKOFF) — « décollage aux gaz »
Après l’ARM, rien ne se passe tant que le pilote n’a pas poussé les gaz au-delà du milieu. Ensuite, le programme : 1 s d’accélération jusqu’à 100 % de gaz, ailes à plat ; 2 s de rotation avec un tangage de 15° ; puis une montée avec un tangage de 10° jusqu’à ce que le pilote change de mode. Les sticks s’ajoutent au programme : on peut redresser le cap pendant la course au décollage.
**Il faut :** l’IMU.

### LAUNCH (LNCH) — « lancement à la main »
1. L’appareil est armé, les gaz au-delà du milieu : le lancement est **en attente**, le moteur est arrêté.
2. Le jet : surcharge vers l’avant > 1,5 g pendant plus de 40 ms.
3. Au bout de 0,3 s (la main s’est écartée de l’hélice) : gaz à 100 %, montée avec un tangage de 15°, ailes à plat — pendant 6 s ou jusqu’à 30 m.
4. Ensuite — comme CRUISE à l’altitude atteinte.

Tout mouvement de stick (> 150 µs) avant le jet annule le lancement : l’avion reste dans les mains du pilote.
**Il faut :** l’IMU (accéléromètre). En simulation : un jet à 9 m/s depuis la hauteur de la main — l’avion ne touche jamais le sol et gagne plus de 15 m en 15 s.

### AUTO_LAND (LAND) — « atterrissage »
Moteur coupé, plané sur le cap avec un tangage de −4° ; sous 3 m d’après le baromètre — arrondi (+4°). Le stick de roulis permet de corriger le cap d’approche. À activer sur une ligne droite, face au vent, à 20–40 m, avec de la piste en réserve. En simulation, le toucher se fait à une vitesse verticale inférieure à 1,5 m/s, ailes à plat, sans piquer du nez.
**Il faut :** l’IMU + le baromètre (mis à zéro au sol à la mise sous tension).

### SOARING (SOAR) — « vol à voile en thermiques »
Moteur coupé, plané. Un variomètre (avec un tube de Pitot, à énergie totale, sans faux « thermiques » dus à une traction sur le manche) au-dessus de 0,5 m/s pendant plus de 1,5 s — c’est un thermique : des cercles avec 25° d’inclinaison. Si la montée moyenne sur 8 s tombe sous −0,2 m/s, on sort du thermique. Sous 30 m — moteur jusqu’à 100 m ; à plus de 400 m du point de départ — plané vers le point de départ. En simulation, il trouve un thermique (cœur à 3 m/s) et gagne plus de 50 m sans moteur.
**Il faut :** l’IMU + le baromètre ; le GPS — pour revenir au point de départ.

### RESCUE (RESQ) — « au secours »
Ailes à plat, nez à +8°, gaz à 70 % — depuis n’importe quelle spirale. On a perdu l’orientation ? On bascule l’interrupteur et on respire. En simulation, depuis une spirale avec 70° d’inclinaison et le nez à −40°, en 4 s — ailes à plat et montée.
**Il faut :** l’IMU.

---

## Fonctions (interrupteurs)

| Fonction | Ce qu’elle fait | Détails et valeurs (`Config.h`) |
|---|---|---|
| `FLAPS` | les deux ailerons vers le bas — flaperons | `FLAPS_DEPLOYED_US` = 220 µs, progressivement en 1 s ; le roulis agit par-dessus |
| `AIRBRAKE` | les deux ailerons vers le haut — aérofrein, pente de descente plus raide | `AIRBRAKE_US` = 250 ; prioritaire sur les volets |
| `AUTO_TRIM` | apprend à tenir l’avion droit sans les sticks : la commande constante aux gouvernes en vol en palier « s’écoule » dans le trim | 20 %/s, jusqu’à ±120 µs ; enregistré en flash après le DISARM **au sol** |
| `TURN_COORDINATION` | direction dans le virage, nez en haut dans l’inclinaison | toujours active dans les modes de navigation |
| `MOTOR_KILL` | moteur coupé dans n’importe quel mode, même automatique | plus fort que n’importe quel mode et que les gaz |
| `BEEPER` | buzzer « je suis là » | sans interrupteur, il sonne tout seul : au sol, liaison perdue > 10 s |
| `PAYLOAD_DROP` | servo AUX1 ouvert tant que l’interrupteur est actif | 1000 µs fermé, 2000 ouvert |
| `GEOFENCE` | au-delà de 500 m du point de départ ou au-dessus de 120 m — RTH | `GEOFENCE_ALWAYS_ON` — sans interrupteur |
| `HOME_RESET` | point de départ = point actuel (au moment où l’on active l’interrupteur) | seulement avec un bon GPS |
| `CAMERA_STAB` | la caméra sur AUX2 garde son angle par rapport à l’horizon | on retranche le tangage de l’avion |

## Potentiomètres

Le centre du potentiomètre = valeur par défaut de `Config.h` ; les extrémités sont le minimum et le maximum. Non affecté, c’est la valeur par défaut qui s’applique.

| Potentiomètre | Minimum … centre … maximum | Où il agit |
|---|---|---|
| `STAB_GAIN` | ×0,25 … ×1 … ×2 | tous les modes avec stabilisation et ACRO — « plus doux/plus dur » |
| `MAX_BANK` | 15° … 45° … 60° | inclinaison maximale au stick et en navigation |
| `CRUISE_SPEED` | gaz 30 … 55 … 85 % (avec Pitot : 10 … 14 … 22 m/s) | CRUISE, LOITER, RTH, le moteur en SOARING |
| `FLAPS` | 0 … 50 … 100 % de volets | volets progressifs au lieu d’un interrupteur |
| `CAMERA_TILT` | −90° … 0° … +30° | angle de la caméra (AUX2) |
| `RATES` | 30 … 65 … 100 % de la course des sticks | tous les modes : sensibilité des sticks |
| `LOITER_RADIUS` | 25 … 50 … 150 m | LOITER et cercles au-dessus du point de départ |

> Une astuce utile : le potentiomètre `STAB_GAIN` sur VrA permet un réglage « en direct » des coefficients en vol. Si ça oscille — diminuez ; si c’est mou — augmentez ; puis reportez le multiplicateur dans `Config.h`.

---

## Perte de liaison, géorepérage, point de départ

**Le point de départ** est enregistré à l’ARM si le GPS est bon (fix 3D, ≥ 6 satellites, précision ≤ 5 m). Si le GPS n’a pas encore accroché à l’ARM, le point est enregistré dès qu’il accroche. Pour le changer sur le terrain — la fonction `HOME_RESET`.

**Perte de liaison** (pas de trames iBUS pendant > 0,5 s, ou le récepteur a envoyé des gaz sous 950 µs — c’est le failsafe réglé dans la radio ; voir `docs/PILOT_GUIDE.md`) :

| Situation | Ce que fait l’appareil |
|---|---|
| au sol (non armé) | moteur 0, gouvernes au neutre ; au bout de 10 s — le buzzer |
| en vol, GPS et point de départ disponibles | **RTH** avec moteur, au-dessus du point de départ — des cercles à 40 m |
| en vol, pas de GPS | **plané** : moteur coupé, ailes à plat, nez −3° |
| la liaison est revenue | aussitôt le mode de l’interrupteur du pilote |

Un retour déjà commencé ne passe pas en plané à cause d’une courte perte du GPS. `FAILSAFE_RTH = false` — plané uniquement.

**Géorepérage** (`GEOFENCE` ou `GEOFENCE_ALWAYS_ON`) : sortir au-delà de `FENCE_RADIUS_M` (500 m) ou au-dessus de `FENCE_ALTITUDE_M` (120 m) — RTH. Pour reprendre les commandes, il faut basculer un interrupteur de mode sur une autre position (un mode s’active par changement de position). Il se redéclenchera quand l’avion sera rentré à l’intérieur avec une marge de 10 %.

---

## Un tube de Pitot fait maison

Vitesse air sans capteur de pression différentielle acheté : **deux baromètres**.

```
       flux d’air incident ─►  ┌──────────── tube (PVC/laiton, Ø4–6 mm) ──┐
                               │  BMP581 (I2C 0x47) — pression totale     │  étanche
                               └──────────────────────────────────────────┘
   fuselage : baromètre principal (BMP581 0x46 / SPL06 / BMP388) — pression statique

   vitesse  V = √(2·(P_tube − P_statique − zéro) / ρ),   ρ — d’après la pression statique et la température
```

**Montage.** Le BMP581 (module d’adresse 0x47 : broche SDO au VCC) est collé dans un tube ouvert seulement vers l’avant — la carte est dans une cavité étanche, les fils sortent à travers du mastic d’étanchéité. Le tube pointe vers l’avant, hors du souffle de l’hélice (sur l’aile ou au-dessus du nez). Le second baromètre se trouve à l’intérieur du fuselage, à l’abri du flux direct (mousse).

**Activation dans le firmware** — `sensors/SensorSelection.h` : `SENSOR_KIT_LSM6DSV_PITOT` ou `SENSOR_KIT_ICM45686_PITOT` (kits tout prêts), ou `SENSOR_AIRSPEED = SENSOR_AIRSPEED_PITOT_BMP581` dans votre propre kit.

**Le zéro.** Deux baromètres divergent toujours un peu : la précision absolue de chacun est de quelques dizaines de pascals, et c’est toute la différence de pression à basse vitesse (10 m/s ≈ 60 Pa). Pendant la première seconde après la mise sous tension, le firmware fait la moyenne de l’écart et le prend pour zéro. **L’avion est immobile à la mise sous tension, et le tube est bouché avec le doigt ou un capuchon, ou orienté face au vent.** Sur l’OLED, dans le tableau de bord et dans la télémétrie, la vitesse apparaît après la mise à zéro.

**Calibration de `PITOT_SCALE`.** La pression dans le fuselage n’est pas strictement statique. Par temps calme, volez en ligne droite aller-retour en CRUISE et comparez la vitesse sol moyenne du GPS avec la vitesse du tube : `PITOT_SCALE = V_GPS / V_tube`.

**Protection.** Si l’écart est fortement négatif pendant plus de 2 s (tuyaux inversés, eau) ou si les mesures du tube datent de plus de 0,2 s, la vitesse n’est pas fournie : le pilote automatique passe aux gaz du potentiomètre et au cap sans elle. Vérifié dans une simulation en boucle fermée avec du bruit sur les deux baromètres : l’erreur de vitesse en vol est < 0,5 m/s.

Ce que donne le tube : CRUISE tient la **vitesse air** et non les gaz ; la protection contre le décrochage ; un variomètre à énergie totale pour SOARING ; une vitesse honnête dans la télémétrie.

---

## Station sol : tableau de bord Wi-Fi et MAVLink

**ESP32 : tableau de bord Wi-Fi.** Point d’accès `OpenPlane-Debug`, mot de passe `12345678`, l’adresse s’affiche dans le moniteur série. Canaux, sorties, tous les capteurs, le mode, la navigation, les fonctions activées ; on peut changer le mode et le PID. Détails dans `docs/PILOT_GUIDE.md`.

**STM32H743 : MAVLink par radiomodem** (UART4 : PD0 RX, PD1 TX, 57600 bauds, le réglage par défaut de SiK). Conviennent les SiK 433/868/915 MHz, l’ELRS en mode MAVLink et un ESP-01 comme pont Wi-Fi. **QGroundControl** et **Mission Planner** voient l’appareil comme un avion ArduPilot :

- horizon, carte avec le point de départ, vitesse (du tube de Pitot, s’il y en a un), altitude, variomètre, gaz ;
- les modes sous les noms d’ArduPlane : STABILIZE → FBWA, ALT_HOLD → FBWB, CRUISE → CRUISE, LOITER → LOITER, RTH → RTL, AUTO_TAKEOFF/LAUNCH → TAKEOFF, SOARING → THERMAL, RESCUE → STABILIZE, AUTO_LAND → AUTO ; le failsafe s’affiche comme RTL ou CIRCLE ;
- fil des messages : ARM/DISARM, changement de mode (sous notre nom), perte de liaison, géorepérage ;
- **changement de mode depuis le sol**, avec le bouton de mode de la GCS (sauf AUTO : OpenPlane n’a pas de missions) ;
- **paramètres** `RLL_KP … PTCH_KD` : le PID de roulis et de tangage, lus et modifiés depuis la fenêtre des paramètres de la GCS en plein vol (ils ne sont pas conservés après un redémarrage — reportez les bonnes valeurs dans `Config.h`).

L’ARM/DISARM depuis le sol est **refusé** : uniquement par l’interrupteur de la radio. Pour vérifier le flux sans matériel : `OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink && python3 tools/check_mavlink.py /tmp/tlm.bin` (il faut `pip install pymavlink`).

---

## Ordre de réglage d’un nouvel avion

1. **MANUAL au sol :** sens des gouvernes (`*_REVERSED` dans `Config.h`), volets vers le bas, AUX. Contrôle des sorties avec `p` dans la console (hélice retirée !).
2. **Capteurs au sol :** `b` : interrogation des bus ; `s` : état ; `o` : calibration du montage de l’IMU (3 positions, une seule fois) ; `m` : boussole.
3. **STABILIZE au sol :** inclinez l’avion vers la droite : l’aileron droit doit aller **vers le bas** (pour redresser). Nez en haut : gouverne de profondeur vers le bas. Sinon, les inversions ou le montage de l’IMU sont faux.
4. **Premier vol en MANUAL**, montée à plus de 50 m → STABILIZE. Si ça oscille, `STAB_GAIN` plus petit ; si c’est mou, plus grand.
5. **AUTO_TRIM** en vol en palier pendant 20–30 s, atterrissage, DISARM : le trim sera enregistré.
6. **ALT_HOLD** puis **CRUISE** : vérifiez l’altitude et le cap ; avec un tube de Pitot, calibrez `PITOT_SCALE`.
7. **LOITER** et **RTH** : en altitude, à portée de vue, le doigt sur MANUAL.
8. Seulement après cela, le **test de failsafe** (éteignez la radio en altitude ; l’avion doit rentrer à la maison) et le décollage et l’atterrissage automatiques.

## Contrôle avant vol du pilote automatique

- [ ] La disposition affichée à la mise sous tension est celle que vous attendez.
- [ ] Console/OLED : IMU ok, le contrôle avant vol de l’IMU est réussi (l’avion était immobile à la mise sous tension).
- [ ] Le baromètre est mis à zéro au sol (altitude ~0 sur l’OLED).
- [ ] Avec un tube de Pitot : vitesse ~0 au sol ; si vous soufflez dans le tube, elle augmente.
- [ ] GPS : fix 3D, ≥ 6 satellites **avant l’ARM** ; sinon, pas de point de départ ni de RTH.
- [ ] STABILIZE au sol : les ailerons et la gouverne de profondeur redressent l’avion, ils ne le font pas basculer.
- [ ] Le failsafe est réglé dans la radio (gaz sous 950 en cas de perte de liaison) et vérifié en éteignant la radio **au sol**, sans hélice.
- [ ] MANUAL, sous le doigt.

---

## Ce dont chaque mode a besoin

| Mode | IMU | Baromètre | GPS | Boussole | Pitot | Gaz | Sans le capteur nécessaire |
|---|:-:|:-:|:-:|:-:|:-:|---|---|
| MANUAL | | | | | | pilote | — |
| STABILIZE | ● | | | | | pilote | gouvernes = sticks |
| ALT_HOLD | ● | ● | | | | pilote | le pilote tient l’altitude |
| ACRO | ● | | | | | pilote | gouvernes = sticks |
| CRUISE | ● | ● | ○ | ○ | ○ | auto | cap au gyroscope (dérive), altitude au pilote |
| LOITER | ● | ● | ● | | ○ | auto | cercle incliné sur place |
| RTH | ● | ● | ● | | ○ | auto | cercles sur place |
| AUTO_TAKEOFF | ● | | | | | programme | gouvernes = sticks + gaz du pilote |
| LAUNCH | ● | ○ | | | | programme | gouvernes au neutre |
| AUTO_LAND | ● | ● | | ○ | | 0 | sans arrondi |
| SOARING | ● | ● | ○ | | ○ | 0 / moteur | sans thermiques — plané |
| RESCUE | ● | | | | | 70 % | gouvernes au neutre |

● signifie obligatoire et ○, que ça améliore. Les vérifications dans le code : `Autopilot.h` (`imuReady`, `baroReady`, `nav.gpsGood`) ; les tests : `test/native/test_autopilot_modes` (réaction des modes à chaque capteur) et `test/native/test_sim` (vols en boucle fermée).
