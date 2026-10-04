# RC — réception des ordres de la radiocommande

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/rc.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels. La traduction a été réalisée par une IA et n’a pas été relue par des locuteurs natifs. Pour signaler une erreur, écrivez à [Damir Lebedev](https://github.com/damir-lebedev) ou ouvrez un [ticket](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Référence](README.md)

La couche RC transforme les octets de l’UART en valeurs de canaux et en un
indicateur « pas de liaison ». Elle ne sait rien de l’avion, de l’ARM, du
comportement de failsafe ni des servos — remplacer le protocole (S-Bus, PPM)
n’affecte que cette couche.

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
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Linéaire : 1000 → `−max`, 1500 → 0, 2000 → `+max` (l’entrée est d’abord bornée) ; `reverse` inverse le signe. Le résultat est borné à ±`max` |

Exemple : `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `IBusReceiver`

**Fichier :** `rc/IBusReceiver.h` · **Dépend de :** `IUartPort`, `RcChannelState`, `Config`, `Channels`

Un analyseur octet par octet du protocole iBUS de FlySky.

**Format de la trame** (32 octets) : `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(first 30 bytes)`. On prend les `IBUS_CHANNELS` = 10 premiers
canaux ; la valeur du canal correspond aux **12 bits de poids faible** (dans
les bits de poids fort, le FS-iA6B transmet des données de service, par
exemple en failsafe `0x2384` → 900 µs).

| Méthode | Description |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | Le port n’est pas ouvert dans le constructeur |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`, le délai d’attente se compte à partir de « maintenant » |
| `void update()` | Lire tout ce qui s’est accumulé dans l’UART ; à appeler à chaque cycle |
| `const RcChannelState& getState() const` | Les derniers canaux reçus |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Aucune trame n’est encore arrivée **ou** la dernière est plus ancienne que `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | Dans la dernière trame, les gaz < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` de la dernière trame correcte |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Compteurs de trames correctes et d’erreurs de CRC |

L’automate d’analyse (`processByte`) : attend `0x20` ; l’octet suivant doit
être `0x40`, sinon la recherche recommence ; puis il accumule 32 octets et
appelle `processFrame()`. Une trame avec un CRC erroné est écartée en entier
(les canaux ne changent pas, `badFrames++`).

Invariants :

- Jusqu’à la première trame correcte, `isSignalLost() == true` — les valeurs
  par défaut (toutes à 1500) ne sont pas prises pour des ordres de la
  radiocommande.
- L’indicateur de failsafe est recalculé à **chaque** trame correcte — la
  liaison est rétablie dès la première trame avec des gaz normaux.
