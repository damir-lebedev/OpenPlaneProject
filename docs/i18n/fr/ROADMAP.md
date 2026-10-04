# OpenPlaneProject — feuille de route et présentation pour les investisseurs et partenaires

> 🌐 Cette page est la traduction de l’[original en russe](../../ROADMAP.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels.

> Dépôt : [github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject), branche `main`.
> Ce document est une version plus approfondie de l’aperçu du README, destinée à ceux qui envisagent d’investir de l’argent,
> du temps ou un partenariat dans le projet. Il décrit où en est le projet aujourd’hui, où et pourquoi il va,
> et ce qui, dans le code déjà écrit, rend cette trajectoire réaliste et pas seulement déclarée.

## 1. État actuel — en toute franchise

OpenPlaneProject est aujourd’hui un prototype de planeur qui a déjà volé mais reste immature, avec son propre firmware sur l’ESP32 — ni un produit fini ni un drone autonome. Matériel : envergure de 1200 mm, corde de 250 mm, profil NACA 4412, structure en PETG (impression 3D), servos MG90S (un distinct pour chaque aileron), alimentation par LiPo 3S. Le premier prototype (ESP32-C3, moteur D2212 1000KV, ESC de 40A) a déjà volé en commande manuelle et, à l’issue du vol, a révélé des problèmes concrets : une solidité insuffisante des fixations du moteur et de l’aile (il faut les renforcer au carbone) et la nécessité de régler les servos.

La version actuelle est passée à l’ESP32-S3 (N16R8), avec un moteur D3548 1100KV et un ESC de 60–80A, et tous les capteurs du pilote automatique y sont connectés sur le banc : une IMU (MPU6500), un baromètre BMP388, une boussole QMC5883P, plus un écran OLED d’état. Vérifié en direct : commande manuelle par radio via l’iBUS avec un mixeur d’ailerons, de gouverne de profondeur, de gouverne de direction (avec la roulette orientable) et de volets (flaperons) ; ARM par un interrupteur dédié ; failsafe avec la radio éteinte ; une boucle de contrôle à 500 Hz ; un tableau de bord web en direct par Wi-Fi. Sur la table, la stabilisation répond aux inclinaisons dans le bon sens.

Depuis, le firmware a gagné 12 modes de pilote automatique (stabilisation, maintien d’altitude, croisière, cercles et retour au point de départ par GPS, lancement à la main, atterrissage automatique, vol à voile dans les thermiques, « sauvetage »), un géorepérage, le retour au point de départ en cas de perte de signal, le largage de charge, un tube de Pitot fait de deux baromètres, la prise en charge de nouveaux capteurs (LSM6DSV, ICM-45686, QMC6309, SPL06, BMP581) et un firmware complet pour la STM32H743 avec télémétrie MAVLink pour QGroundControl. Tout cela a été vérifié par 387 tests automatiques et des simulations de vol en boucle fermée (le firmware entier pilote un modèle d’avion), mais **n’a pas encore été éprouvé en vol**. Autrement dit : jusqu’ici, seule la commande manuelle a volé ; le pilote automatique est écrit, vérifié par tout ce qui permet de le vérifier sans voler, et attend les essais en vol.

## 2. Pourquoi c’est important

L’aviation légère autonome à faible barrière d’entrée couvre des tâches où ce qui décide n’est pas la charge utile ou la portée maximales, mais la rapidité de réaction et le faible coût d’exploitation :

- **Livraison de médicaments et de sang dans les zones difficiles d’accès et sinistrées** — routes emportées, absence d’infrastructures, destructions après des catastrophes naturelles ou des conflits. Ce qui décide ici, ce n’est pas la capacité d’emport (le colis est petit), mais le temps de réaction : des minutes et des heures au lieu d’une journée en voiture ou à pied.
- **Opérations de recherche et de sauvetage** — largage ciblé d’équipement, de trousses de secours, de moyens de communication et de moyens de flottaison aux victimes avant l’arrivée de l’équipe au sol, dans des zones où l’hélicoptère est trop coûteux ou inaccessible à cause de la météo ou du relief.
- **Agriculture de précision** — suivi des cultures et pulvérisation/épandage ciblés de produits là où les plateformes de drones fermées pour ces tâches coûtent des milliers de dollars, ce qui n’est pas rentable pour les petites et moyennes exploitations.

Dans les trois catégories, l’économie est la même : la différence entre « une solution existe, mais elle est chère et fermée » et « il n’y a en fait pas de solution, parce que c’est cher » — et c’est précisément cette niche que vise une plateforme ouverte et bon marché. Ceci décrit le domaine d’application et le problème du marché ; ce n’est pas l’affirmation qu’OpenPlaneProject sait déjà livrer des charges — au stade actuel, c’est une affirmation sur l’objectif et sur les raisons pour lesquelles il vaut la peine.

## 3. Thèse d’investissement : pourquoi une architecture ouverte sur ESP32 est une asymétrie

Les plateformes commerciales de drones autonomes de livraison/surveillance reposent en général sur des contrôleurs de vol fermés et un logiciel fermé, coûtent de quelques centaines à plusieurs milliers de dollars par appareil et exigent des redevances de licence ou des contrats de service pour exploiter la flotte. OpenPlaneProject part d’une autre hypothèse :

- **Une base bon marché.** Un module ESP32 coûte de l’ordre de 5 à 15 $, et le reste (servos MG90S, ESC, récepteur iBUS) se compose de composants standard de gamme loisir. C’est un ordre de grandeur moins cher que le ticket d’entrée des plateformes commerciales fermées, ce qui est crucial pour des déploiements pilotes dans des contextes à budget limité (ONG, petite agriculture, services de secours régionaux).
- **Le code ouvert change l’économie de la confiance.** Une organisation qui déploie une flotte pour la livraison médicale peut auditer la sécurité (failsafe, logique de l’ARM) et adapter le firmware à ses capteurs et à ses règlements, au lieu de dépendre d’un seul fournisseur et de sa feuille de route.
- **L’architecture est dès aujourd’hui conçue pour s’étendre, et pas seulement pour le planeur actuel.** Ce n’est pas une déclaration, mais la conséquence directe de la structure du code :
  - Un nouveau capteur s’ajoute sous la forme d’une classe qui implémente l’interface existante `Sensor` → `ImuSensor`/`BarometerSensor` (`include/sensors/SensorInterface.h`), sans toucher au cœur. C’est ainsi que l’on a déjà réalisé les pilotes d’IMU (MPU6050/MPU6500, ICM-42688), de baromètres (BMP388, BME280), de boussoles (QMC5883P/L) et de GPS (u-blox M10) — tous écrits directement via les registres du bus (interfaces `II2CBus`/`ISpiBus`/`IUartPort`), sans bibliothèques tierces, c’est-à-dire sans dépendance cachée à un SDK de fabricant précis.
  - Une nouvelle carte s’ajoute avec un seul bloc `#elif` dans `include/config/Config.h` plus un seul bloc `[env:...]` dans `platformio.ini` — le changement de carte fonctionne dès aujourd’hui pour quatre cibles (voir le tableau ci-dessous) ; ce n’est pas une possibilité hypothétique.
  - `Autopilot.h` accepte déjà `ImuSensor*`/`BarometerSensor*` en paramètres (ils peuvent valoir `nullptr`) — autrement dit, le contrat entre le pilote automatique et le matériel suppose que la composition des capteurs va changer (l’étape suivante est le GPS comme une classe de plus suivant le même schéma, voir la Phase 3).
  - `WebDebugServer.h` fournit déjà un JSON agrégé unique (`GET /api/status`) et accepte des commandes (`POST /api/setmode`, `/api/setpid`) — autrement dit, le protocole « l’appareil fournit la télémétrie et accepte des commandes » existe déjà, et la station sol doit en découler plutôt que d’être écrite de zéro.

L’asymétrie tient à ce que la barrière d’entrée (argent, temps d’adaptation à une nouvelle tâche) de cette plateforme est inférieure d’un ordre de grandeur à celle des équivalents fermés, alors que le chemin vers l’autonomie n’exige pas de réécrire le cœur — seulement d’ajouter de nouvelles classes au-dessus des interfaces existantes. C’est une plateforme d’ingénierie ouverte, pas un produit commercial fini — la thèse pour un investisseur/partenaire n’est donc pas « achetez une solution toute faite », mais « entrez à un stade où les fondations sont déjà vérifiées et où les étapes suivantes sont techniquement claires ».

## 4. L’architecture aujourd’hui — la base des phases suivantes

### 4.1 Cartes prises en charge

Le choix de la carte est une seule option de compilation de PlatformIO ; changer de carte n’oblige pas à modifier la logique (`include/config/Config.h` + `platformio.ini`) :

| Environnement (`pio run -e ...`) | Carte | État | aileron L / R | elevator | esc | ibus_rx | i2c sda / scl |
|---|---|---|---|---|---|---|---|
| `esp32-s3` (default) | ESP32-S3 N16R8 (DevKitC-1) | **Principale, vérifiée sur le banc avec tous les capteurs** | GPIO4 / GPIO5 | GPIO6 | GPIO7 | GPIO17 | GPIO41 / GPIO42 |
| `esp32-c3` | ESP32-C3 SuperMini | Premier prototype, a volé en commande manuelle | GPIO5 / GPIO4 | GPIO6 | GPIO7 | GPIO8 | GPIO1 / GPIO3 |
| `esp32-dev` | ESP32 classique 38 broches | Pour le banc, **non vérifiée sur le matériel** (le firmware entier est dans les tests) | GPIO13 / GPIO14 | GPIO27 | GPIO26 | GPIO16 | GPIO21 / GPIO22 |
| `stm32h743` | STM32H743VIT6 (WeAct Mini) | Firmware complet + MAVLink, **pas encore de carte** (le firmware entier tourne dans les tests sur PC) | PA0 / PA1 | PA2 | PA3 | PE7 | PB11 / PB10 |

Téléverser le firmware : `pio run -t upload`. Moniteur : `pio device monitor` (115200).

### 4.2 Carte des canaux RC (FS-i6 + FS-iA6B, iBUS, 10 canaux, 1000–2000 µs)

| Canal | Nom | Fonction par défaut |
|---|---|---|
| CH1–CH4 | sticks | roulis, tangage, gaz, gouverne de direction |
| CH5 | ARM | interrupteur SwA : ARM avec les gaz en bas, DISARM instantané |
| CH6 | SWB | volets |
| CH7 | SWC | mode : MANUAL / STABILIZE / AUTO_TAKEOFF |
| CH8 | SWD | RTH — retour au point de départ |
| CH9 | VRA | force de la stabilisation |
| CH10 | VRB | vitesse de croisière |

CH6–CH10 s’affectent avec une seule ligne dans `include/config/Controls.h` : n’importe lequel des 12 modes, 10 fonctions (volets, frein, largage de charge, géorepérage…) et 7 potentiomètres — [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

### 4.3 Flux de contrôle (`FlightController::update()`)

Un seul orchestrateur, un ordre fixe, une boucle à 500 Hz de période fixe : réception de l’iBUS → gaz du pilote → mode issu de CH7 → **lecture des capteurs et calcul du pilote automatique (toujours, même sans liaison)** → **vérification de la perte de signal avec une priorité absolue** (en l’air — retour au point de départ par GPS moteur en marche, ou plané ailes à plat sans GPS ; au sol — gouvernes au neutre) → ARM → commande des sticks + corrections du pilote automatique dans des signes aéronautiques unifiés → mixeur avec inversion des servos → gaz du mode → blocage des gaz sans ARM → écriture sur les servos/l’ESC. C’est précisément cette discipline d’ordre (la sécurité d’abord, puis la commande manuelle, puis le pilote automatique comme surcouche) qui permet d’y intégrer en toute sécurité un comportement toujours plus autonome sans réécrire la boucle de base. Le Wi-Fi, le tableau de bord et l’écran tournent sur le second cœur et ne retardent pas le contrôle.

### 4.4 Le tableau de bord web, embryon d’une station sol

`WebDebugServer.h` monte dès aujourd’hui un point d’accès (SSID `OpenPlane-Debug`, IP `192.168.4.1`) et sert et accepte du JSON :

| Méthode et chemin | Ce que cela fait |
|---|---|
| `GET /api/status` | Un seul JSON agrégé : RC (10 canaux), armed/failsafe, 7 sorties (`us`, `attached`), IMU, baromètre, boussole, GPS, tube de Pitot (chacun avec `attached`/`available` + données), pilote automatique (mode, corrections, PID, navigation, fonctions activées) |
| `POST /api/setmode` | `{mode: 0-11}` — changer le mode du pilote automatique |
| `POST /api/setpid` | `{kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch}` — chaque champ est facultatif |
| `GET /` | Le tableau de bord HTML : barres en direct des 10 canaux, état de chaque sortie/capteur, boutons de modes, formulaire PID |

Les champs `attached`/`available` sont toujours présents dans le JSON — le tableau de bord distingue honnêtement « absent de la configuration » de « présent dans la configuration, mais ne répond pas », au lieu de se taire sur un capteur absent. C’est le même principe d’honnêteté qui sous-tend tout le document : ne pas faire passer le souhaité pour l’existant.

## 5. Feuille de route technique, phase par phase

Voici huit phases, chacune décrite comme la prochaine étape logique au-dessus des classes déjà existantes, sans dates ni sommes inventées.

### Phase 1 — Commande manuelle et base sûre [terminée]

**Objectif :** un planeur radiocommandé fiable et prévisible, avec un débogage transparent. **Ce qui existe déjà techniquement :** l’analyse de l’iBUS avec détection de la perte de signal dans `IBusReceiver.h`, le mixeur `ControlMixer.h` (sticks → commande de roulis/tangage/volets → PWM avec inversion des servos, sans rien savoir d’UART/PWM), `ThrottleManager.h`, `ArmingManager.h` (ARM par un interrupteur dédié avec les gaz en bas, DISARM instantané), le failsafe à priorité absolue dans `FlightController.h`, la sortie sur Serial (`DebugLogger.h`), le tableau de bord web (`WebDebugServer.h`) et l’écran OLED (`OledDisplay.h`). **Pourquoi c’est la base de tout le reste :** c’est la seule couche qui doit toujours fonctionner, même si toutes les autres phases ne sont pas encore réalisées ou si leurs capteurs sont débranchés — c’est précisément pourquoi le failsafe et l’ARM ont été écrits en premier et vérifiés sur le matériel (y compris le comportement réel du récepteur FS-iA6B avec la radio éteinte).

### Phase 2 — IMU + baromètre → pilote automatique [écrite et vérifiée par des tests et la simulation, en attente des essais en vol]

**Objectif :** le premier mode de vol autonome — stabilisation de l’horizon, décollage automatique, maintien d’altitude. **Ce qui existe déjà techniquement :** `imu/MPU6050_Sensor.h` (MPU6050 et MPU6500, registres directement, rotation des axes selon le montage de la carte, signes aéronautiques, filtre complémentaire dans `ImuSensorBase`), `baro/BMP388_Sensor.h` (I2C ou SPI, compensation complète de Bosch, lecture sur l’indicateur de donnée prête, vitesse verticale filtrée), `Autopilot.h` avec `PidController` (le terme D issu du gyroscope, l’intégrateur n’accumule qu’après l’ARM) et douze modes (de MANUAL à SOARING et RESCUE), commutés par des interrupteurs selon la table de `Controls.h`, depuis le tableau de bord web et depuis QGroundControl, plus le failsafe (retour au point de départ ou plané). Chaque mode vole dans une simulation en boucle fermée de tout le firmware avec un modèle d’avion (`test/native/test_sim`). Sur le banc avec l’ESP32-S3, tous les capteurs répondent et les signes ont été vérifiés en direct : inclinaison → correction des gouvernes dans le sens de la remise à plat. **Ce qu’il faut pour clore la phase :** transférer l’électronique dans le planeur, vérifier les sens des gouvernes sur l’appareil assemblé et réaliser les premiers essais en vol — en commençant par STABILIZE à une altitude sûre.

### Phase 2.5 — Rétroaction à partir de l’avion réel [ébauche, vérifiée en simulation]

**Objectif :** que le pilote automatique ne dépende pas de coefficients réglés pour une seule vitesse, mais de la façon dont l’avion réel réagit à la gouverne à cet instant précis. Le PID de la phase 2 braque la gouverne « selon une formule » et ne vérifie pas le résultat ; à basse vitesse, il corrige trop peu, à grande vitesse, il en fait trop. **Ce qui existe déjà techniquement** (`include/autopilot/feedback/`, **non branché** au firmware) : l’estimation en vol de l’efficacité des gouvernes (moindres carrés récursifs, recalcul selon la vitesse ∝ V²), le régulateur « angle → vitesse de rotation → gouverne » avec correction complémentaire (« la gouverne n’a pas tourné jusqu’au bout — la tourner encore »), la protection contre la perte de vitesse et le décrochage (gaz, nez vers le bas, ailes à plat), le décollage depuis une piste ou à la main et l’atterrissage par étapes d’après les capteurs. Le tout a été vérifié par une simulation en boucle fermée de l’avion sur la carte elle-même (`pio test -e esp32-s3 -f test_feedback`, 10 scénarios) — y compris un aileron inversé, des turbulences, un nez relevé à bas régime, le décollage et l’atterrissage. **Ce qu’il faut pour clore la phase :** après les premiers vols de la phase 2 — un « mode fantôme » (la rétroaction écrit seulement dans le journal ce qu’elle aurait fait), puis un branchement axe par axe, un capteur de vitesse air (tube de Pitot) et un télémètre pour l’arrondi avant le toucher. Détails — la section « Rétroaction » du [`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md).

### Phase 3 — GPS et boussole [dans le firmware, vérifiée par simulation]

**État (mis à jour) : la navigation fonctionne** — cap d’après le GPS/la boussole/le gyroscope avec hystérésis, point de départ à l’ARM, CRUISE, LOITER (un champ vectoriel vers un cercle), RTH, géorepérage ; vérifié par des simulations en boucle fermée. Ci-dessous, l’entrée d’origine de la phase. Le GPS (u-blox M10, `sensors/gps/UbloxM10_Gps.h`, protocole UBX-NAV-PVT) et le magnétomètre (deux variantes de la carte « GY-273 » : QMC5883P — `sensors/mag/QMC5883P_Sensor.h`, installé sur le banc actuel et vérifié en direct ; QMC5883L — `sensors/mag/QMC5883L_Sensor.h`) ont été ajoutés comme de nouvelles classes implémentant les interfaces `GpsSensor`/`MagnetometerSensor` (`include/sensors/SensorInterface.h`) selon le même principe que l’IMU et le baromètre — `Autopilot.h` et `FlightController.h` n’ont pas été réécrits, ils ont reçu deux sources de données de plus de la même manière (un pointeur nullable dans le constructeur). Au passage est apparue la couche HAL (`include/hal/`), par laquelle les capteurs accèdent aux bus — I2C/SPI/UART ne sont plus liés directement aux `Wire`/`SPI`/`HardwareSerial` propres à l’ESP32.

Pour l’instant, les données du GPS/de la boussole sont disponibles via `Autopilot::getGpsSensor()`/`getMagnetometerSensor()` et dans `GET /api/status`, plus un réglage unique du yaw initial d’après la boussole au démarrage — **mais elles ne participent pas au contrôle**. Questions ouvertes pour terminer la phase : brancher le module GPS à l’ESP32-S3 (les broches de l’UART2 sont déjà réservées), étalonner la boussole sur l’appareil assemblé et ajouter la compensation d’inclinaison au cap. Sur l’ESP32-C3, un GPS complet est impossible — il manque des GPIO pour le TX (voir le brochage dans `DEVELOPER_GUIDE.md`).

### Phase 4 — Vol par points de cheminement (waypoint navigation) [étape suivante]

**État :** les primitives sont prêtes — `Guidance::rollForCourse`, un cercle autour d’un point, le retour à un point (RTH), un canal MAVLink pour téléverser la mission (pour l’instant, à une demande de mission, l’appareil répond honnêtement « 0 point »). Reste à faire : le stockage de l’itinéraire, le passage d’un point à l’autre, le protocole MISSION_* de MAVLink.

**Objectif :** l’appareil vole selon un ensemble donné de coordonnées sans intervention de l’opérateur sur chaque tronçon de l’itinéraire. **Comment cela s’insère dans l’architecture :** c’est un nouvel `AutopilotMode` dans `Autopilot.h`, au même titre que les MANUAL/STABILIZE/AUTO_TAKEOFF/ALT_HOLD existants — autrement dit, le mécanisme de changement de mode (via les emplacements RC et via `POST /api/setmode`) ne change pas, et l’on ajoute un cinquième mode, qui prend le cap et la distance du GPS (Phase 3) au lieu d’une saisie manuelle via la radio. **Ce qu’il faut techniquement :** un algorithme de calcul du cap vers un point et la logique de passage entre les points de l’itinéraire, plus un moyen de téléverser l’itinéraire lui-même dans l’appareil (le candidat naturel est l’extension de la même API HTTP qui sert déjà à piloter les modes et le PID).

### Phase 5 — Télémétrie longue portée [réalisée dans le firmware STM32]

**État :** sur la STM32H743 — MAVLink 2 par modem radio (SiK, ELRS en mode MAVLink) : attitude, position, vitesse, mode, paramètres du PID, changement de mode depuis le sol. Les trames ont été confrontées à la référence pymavlink. L’ESP32 n’a pas d’UART libre — on y utilise le tableau de bord Wi-Fi. Ci-dessous, l’entrée d’origine de la phase.

**Objectif :** une liaison appareil↔sol aux distances pertinentes pour une vraie livraison, et non pour le banc. **Une évaluation honnête de l’état actuel :** le point d’accès Wi-Fi de `WebDebugServer` transmet dès aujourd’hui l’état agrégé complet et les commandes de contrôle, mais la portée d’un point d’accès Wi-Fi ordinaire est de quelques dizaines de mètres, ce qui suffit pour déboguer sur une table ou sur un aérodrome, mais pas pour un itinéraire autonome au-delà de la vue directe. **Ce qu’il faut techniquement :** un canal radio distinct, de plus grande portée (par exemple un module LoRa ou un modem radio de télémétrie spécialisé), comme transport du même format de données que celui déjà défini dans `GET /api/status` — autrement dit, remplacer ou compléter la couche de transport, et non réécrire le format de la télémétrie.

### Phase 6 — Une interface complète de contrôle au sol [en partie : QGroundControl / Mission Planner]

**État :** grâce à MAVLink, les stations sol standard voient déjà l’appareil (carte, point de départ, instruments, modes sous les noms d’ArduPlane). Une station à nous pour une flotte reste un objectif. Ci-dessous, l’entrée d’origine de la phase.

**Objectif :** une station de planification de missions avec carte, télémétrie en direct et gestion de flotte, et non une page de débogage pour un seul appareil. **Comment cela s’insère dans l’architecture :** `WebDebugServer.h` n’est dès aujourd’hui pas une maquette, mais un serveur web qui fonctionne, avec un état JSON agrégé et une API de commandes (voir le tableau de la section 4.4) ; c’est le point de départ, pas quelque chose qu’il faudra jeter. Les étapes suivantes : une carte avec la position courante (après la Phase 3), l’affichage et le téléversement d’un itinéraire (après la Phase 4), le fonctionnement sur un canal radio longue portée (après la Phase 5), et le passage de l’interface d’un appareil à plusieurs. Plus de détails dans la section 6.

### Phase 7 — Mécanisme de largage de charge et mesures de protection pour la livraison [réalisée dans le firmware, pas encore volée]

**État :** largage de charge (servo AUX1, la fonction `PAYLOAD_DROP` sur n’importe quel interrupteur), géorepérage (rayon et plafond → RTH), retour au point de départ en cas de perte de signal, le buzzer « modèle perdu ». Ci-dessous, l’entrée d’origine de la phase.

**Objectif :** faire passer la plateforme d’« un avion qui vole de façon autonome » à « un avion qui livre de façon autonome ». **Ce qu’il faut techniquement :** un servo supplémentaire pour le mécanisme de largage/distribution de la charge, piloté selon le même principe que les autres sorties de `FlightOutputs.h` ; et des mesures de protection propres à la livraison, et non au vol neutre — des géorepérages (limite de la zone de vol) et le retour automatique au point de départ en cas de perte de signal (aujourd’hui, quand le signal est perdu en l’air, `FlightController` coupe le moteur et passe en plané ailes à plat, ce qui est correct pour un planeur piloté à la main, mais pour la livraison autonome l’étape logique suivante est le retour à la base par GPS au lieu d’un simple plané).

### Phase 8 — Passage à l’échelle d’une flotte

**Objectif :** gérer plusieurs appareils en même temps — répartition des tâches, tableau de bord opérationnel, historique des vols. C’est le niveau auquel le projet cesse d’être seulement un prototype d’ingénierie de loisir et devient un outil opérationnel, intéressant comme problème d’affaires : planification d’itinéraires pour plusieurs appareils, file de tâches, état de chaque appareil en temps réel. Techniquement, c’est une surcouche au-dessus des Phases 3–6 (GPS, télémétrie, interface) — en fait la même API de `WebDebugServer`, mais étendue à de nombreuses sources de télémétrie au lieu d’une seule.

## 6. Interface : d’une page de débogage à une station sol

La thèse clé de cette section : il n’est pas nécessaire de construire l’interface de zéro — elle existe déjà en partie et fonctionne. Aujourd’hui, `WebDebugServer.h` :

- fournit un instantané JSON agrégé unique de l’état de l’appareil (`GET /api/status`) — les canaux RC, les indicateurs armed/failsafe, l’état de chaque sortie, l’état de chaque capteur (honnêtement, par des indicateurs distincts `attached` et `available`) et l’état du pilote automatique ;
- accepte des commandes de contrôle en temps réel (changement de mode, modification du PID) sans reprogrammer ;
- sert un tableau de bord HTML prêt à l’emploi avec des barres de canaux en direct et des boutons de commande.

Le chemin vers une station sol complète est une extension successive du protocole qui fonctionne déjà, et non un changement d’architecture :

1. Ajouter au tableau de bord une carte et la position courante — cela demande le GPS (Phase 3) comme un champ de plus dans le même état JSON.
2. Ajouter la construction et le téléversement d’itinéraire — cela demande un mode à points de cheminement (Phase 4) et une extension de l’API POST sur le modèle de `/api/setmode`/`/api/setpid`.
3. Faire passer le transport du point d’accès Wi-Fi à une liaison longue portée (Phase 5), en conservant le même format de messages pour ne pas réécrire l’interface existante.
4. Étendre l’interface d’un appareil à plusieurs sources de télémétrie (Phase 8).

Autrement dit, l’élément de la future station sol le plus risqué du point de vue du « faudra-t-il l’écrire de zéro » — la sérialisation de l’état de l’appareil et l’API de commandes — est déjà réalisé et vérifié en direct sur la carte.

## 7. Limites en toute franchise — ce qui ne fonctionne pas encore

Pour que la feuille de route ne ressemble pas à du marketing, nous consignons à part ce qui n’est pas encore fait ou n’est fait que partiellement :

- Le pilote automatique a été vérifié sur le banc, par 387 tests automatiques et par des simulations en boucle fermée, mais il n’a jamais été éprouvé en vol — jusqu’ici, seule la commande manuelle a volé (sur le premier prototype). Le modèle d’avion des simulations est simplifié et les coefficients sont des valeurs de départ.
- Les nouveaux capteurs (LSM6DSV, ICM-45686, QMC6309, SPL06, BMP581) ont été vérifiés par des émulateurs de registres construits d’après les fiches techniques ; sur le matériel, pas encore.
- STM32H743 : tout le firmware tourne sur un PC au-dessus de fakes de STM32duino ; il n’y a pas encore de carte physique.
- L’horizon pour la stabilisation est l’attitude de l’appareil à la mise sous tension (l’IMU s’étalonne à chaque démarrage), ou l’étalonnage du montage à partir de trois positions.
- Le branchement physique d’un servo n’est pas visible du logiciel ; on voit seulement que l’impulsion sort réellement sur la broche (un autotest depuis la console).
- Le brochage de l’ESP32 ordinaire à 38 broches a été choisi d’après la documentation de la puce et n’a pas été vérifié sur le matériel.
- La licence est l’[OpenPlane License](LICENSE.md) : MIT avec mention obligatoire de l’auteur et interdictions de l’usage militaire et du préjudice intentionnel aux personnes et aux biens sans leur consentement. À cause de ces interdictions, elle n’est pas considérée comme « libre » au sens de l’OSI.

## 8. Questions ouvertes — une invitation à en discuter

Voici les points sur lesquels le projet n’a pas encore de réponse, formulés à dessein comme des questions pour un partenaire ou un investisseur potentiel, et non comme des faits tranchés :

- Le modèle de financement et son ampleur — négociable ; il n’y a pour l’instant ni sommes ni délais précis, et ce document n’en inventera pas.
- La forme juridique du projet (une société, une fondation, une communauté purement open source) — ouverte à la discussion avec ceux qui s’intéressent à un partenariat.
- La composition de l’équipe — pour l’instant le projet est mené publiquement et ouvert à la participation ; aucun rôle ni engagement précis n’est posé à l’avance.

Si l’un de ces points est important pour vous en tant que partenaire potentiel, le bon endroit pour en parler est la section Discussions de GitHub du projet (voir la section 9), et non les suppositions de ce document.

## 9. Comment nous contacter et participer

Le seul canal officiel du projet aujourd’hui est le dépôt GitHub :
[github.com/damir-lebedev/OpenPlaneProject](https://github.com/damir-lebedev/OpenPlaneProject)
(branche `main`). Il n’existe pour l’instant aucun autre contact (e-mail, réseaux sociaux, personne morale), et ils ne sont pas indiqués ici à dessein, pour ne pas induire en erreur.

- **Issues** — signaler un bogue, proposer une modification technique précise, rendre compte des résultats d’un essai en vol sur votre propre exemplaire du prototype.
- **Discussions** — discuter de la feuille de route, du partenariat, de l’usage pour une tâche précise (livraison, recherche et sauvetage, agriculture), des questions de licence et de financement de la section 8.
- **Pull requests** — ajouter un nouveau capteur via l’interface `Sensor`, une nouvelle carte via un bloc dans `Config.h`, un nouvel `AutopilotMode`, des améliorations du tableau de bord web — l’architecture est conçue pour que cela puisse se faire sans toucher au cœur.

Si vous lisez ce document en tant qu’investisseur ou partenaire potentiel : la prochaine étape qui ait du sens n’est pas de signer quoi que ce soit, mais d’ouvrir une Discussion dans le dépôt avec une question ou une proposition précise. La feuille de route ci-dessus est une invitation à en discuter phase par phase, avec un accès complet au code sur lequel elle repose.
