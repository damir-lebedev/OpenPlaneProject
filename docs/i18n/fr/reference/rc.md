# RC — réception des ordres de la radiocommande

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/rc.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Référence](README.md)

La couche RC transforme les octets de l’UART en valeurs de canaux et en un
indicateur « pas de liaison ». Elle ne sait rien de l’avion, de l’ARM, du
comportement de failsafe ni des servos. Le protocole du récepteur — iBUS, SBUS
ou CRSF — est fixé par `Config::RX_PROTOCOL` ; un nouveau protocole n’est qu’un
analyseur de trames de plus à côté des trois existants.

---

## `RcChannelState`

**Fichier :** `rc/RcChannelState.h` · **Dépend de :** `Config`, `Channels`

Un instantané des 10 canaux du récepteur (µs), sans logique de commande.

| Méthode | Description |
|---|---|
| `RcChannelState()` | Appelle `reset()` |
| `void reset()` | Valeurs sûres : tous les canaux à `PWM_CENTER`, les gaz à `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | La valeur du canal ; un indice hors limites → `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | Écrit un canal ; un indice hors limites est ignoré |
| `const uint16_t* data() const` | Le tableau entier (pour le débogage) |

---

## `RcInput`

**Fichier :** `rc/RcInput.h` · **Genre :** un ensemble de fonctions statiques · **Dépend de :** `Config`

Conversions courantes des signaux RC.

| Méthode | Description |
|---|---|
| `static uint16_t clamp(uint16_t value)` | Borne à `PWM_MIN..PWM_MAX` |
| `static uint16_t reversed(uint16_t value)` | Voie à l’envers : `3000 − clamp(value)` (1000 ↔ 2000) — pour `Config::CHANNEL_REVERSED` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Linéaire : 1000 → `−max`, 1500 → 0, 2000 → `+max` (l’entrée est d’abord bornée) ; `reverse` inverse le signe. Le résultat est borné à ±`max` |

Exemple : `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `RcReceiver`

**Fichier :** `rc/RcReceiver.h` · **Dépend de :** `IUartPort`, `IBusParser`, `SbusParser`, `CrsfParser`, `RcChannelState`, `RcInput`, `Config`, `Channels`

Le récepteur radio : ouvre l’UART au format du protocole, passe les octets à
l’analyseur et fait ce qui est commun à tous les protocoles — les
`RC_CHANNELS` = 10 premiers canaux, l’inversion des canaux, le délai des trames,
le failsafe, les compteurs.

| Protocole (`Config::RX_PROTOCOL`) | Récepteurs | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B, FS-iA10B | 115200, 8N1 |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper et la plupart des autres | 100000, 8E2, inversé — l’UART inverse lui-même |
| `CRSF` | ExpressLRS, TBS Crossfire et Tracer | 420000, 8N1 |

| Méthode | Description |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | Le port n’est pas ouvert dans le constructeur ; en test, tout protocole est possible |
| `void begin()` | Ouvrir l’UART au format du protocole (`IUartPort::begin(baud, format)`), compter le délai à partir de « maintenant » |
| `void update()` | Lire tout ce qui s’est accumulé dans l’UART ; à appeler à chaque cycle |
| `const RcChannelState& getState() const` | Les derniers canaux reçus |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Aucune trame encore **ou** la dernière est plus ancienne que `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | Le récepteur a signalé la perte de liaison : indicateur failsafe dans la trame SBUS, qualité de liaison 0 % en CRSF ou gaz < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` de la dernière trame valide |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Compteurs des trames acceptées et rejetées (CRC, fin de trame) |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | Le protocole ; son nom est `"iBUS"`, `"SBUS"` ou `"CRSF"` |
| `int16_t getLinkQuality() const` | Qualité de la liaison radio, % — CRSF seulement, sinon −1 |
| `bool isPortReady() const` | L’UART s’est ouvert au bon format ; `false` — le port ne sait pas le faire (SBUS sans inversion), il n’y aura pas de liaison |

La fonction libre `rxProtocolName(Config::RxProtocol)` donne le nom du protocole ;
c’est aussi ce qu’imprime le passeport du firmware (`@id`, champ `rx`).

Invariants :

- Jusqu’à la première trame valide, `isSignalLost() == true` — les valeurs par
  défaut (toutes à 1500) ne sont pas prises pour des ordres de la radio.
- L’indicateur de failsafe est recalculé à **chaque** trame de canaux — la
  liaison revient dès la première trame avec des gaz normaux (en CRSF, aussi avec
  une statistique de qualité de liaison supérieure à 0).
- Les canaux marqués dans `Config::CHANNEL_REVERSED` sont stockés inversés
  (`RcInput::reversed`) ; le failsafe est détecté avant, sur les gaz **bruts**,
  et l’inversion de CH3 est interdite par `static_assert`.

---

## `IBusParser`

**Fichier :** `rc/IBusParser.h` · **Dépend de :** `Config`

Analyseur octet par octet du FlySky iBUS : `bool feed(uint8_t)` renvoie `true` quand
une trame au CRC correct est complète ; les canaux sont dans `values()`. **Trame** (32 octets) : `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(30 premiers octets)` ; la valeur du canal tient dans les
**12 bits de poids faible** (dans les bits hauts, le FS-iA6B envoie des données de
service, p. ex. en failsafe `0x2384` → 900 µs). Il attend `0x20` ; l’octet suivant
doit être `0x40`, sinon la recherche repart (un second `0x20` peut lui-même
commencer une trame). L’iBUS n’a pas d’indicateur de failsafe : le FS-iA6B
signale la perte de liaison par des gaz sous la normale.

---

## `SbusParser`

**Fichier :** `rc/SbusParser.h` · **Dépend de :** `Packed11`

**Trame** (25 octets) : `0x0F | 16 canaux de 11 bits (22 octets) | indicateurs | fin` ;
indicateurs : bit 2 — une trame perdue par radio (ce n’est pas encore une perte
de liaison), bit 3 — failsafe ; fin — `0x00` en SBUS, `0x04`/`0x14`/`0x24`/`0x34`
en SBUS2. L’octet `0x0F` apparaît aussi dans les données, donc une trame n’est
acceptée que si le début et la fin sont à leur place ; sinon l’analyse reprend
au `0x0F` suivant dans ce qui a été reçu — le flux se synchronise en une ou deux
trames.

---

## `CrsfParser`

**Fichier :** `rc/CrsfParser.h` · **Dépend de :** `Packed11`

**Trame :** `0xC8 | longueur | type | données | CRC8` ; longueur = type + données +
CRC, une trame fait au plus 64 octets ; CRC8 de polynôme `0xD5` (DVB-S2) sur le
type et les données (`static uint8_t crc8(...)`). Deux types sont utiles : `0x16`
— les canaux (16 de 11 bits, comme en SBUS) et `0x14` — les statistiques de
liaison (le troisième octet est la qualité de liaison, %). À la perte de liaison,
ExpressLRS cesse par défaut d’envoyer les canaux — c’est le délai des trames ; en
mode « dernières positions », les canaux continuent d’arriver mais la qualité de
liaison est 0 — c’est `failsafe()`.

---

## `Packed11`

**Fichier :** `rc/Packed11.h` · **Genre :** espace de noms

16 canaux de 11 bits dans 22 octets, bits de poids faible d’abord — c’est ainsi
que SBUS et CRSF les empaquettent. `toMicros(raw)` : 172…1811 → 988…2012 µs,
992 → 1500 (`5/8 · valeur + 880`, comme dans Betaflight et INAV) ;
`unpack(data, out)` transforme 22 octets en 16 valeurs en µs.
