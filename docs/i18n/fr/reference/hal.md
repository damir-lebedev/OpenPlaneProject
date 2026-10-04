# HAL — abstraction du matériel

> 🌐 Cette page est la traduction de l’[original en russe](../../../reference/hal.md). En cas de divergence entre la traduction et l’original, c’est l’original qui fait foi. Le firmware affiche les messages de la console en russe ; ils sont donc cités tels quels.

[← Référence](README.md)

Le HAL est la seule couche autorisée à connaître un MCU précis. Les interfaces
se trouvent dans `include/hal/` ; les implémentations sont :

- `include/hal/esp32/` — ESP32 (Arduino core 2.0.x), **l’implémentation principale** ;
- `include/hal/stm32/` — STM32H743 (STM32duino 3.x) : le micrologiciel complet
  se compile (`pio run -e stm32h743`) et tourne sur PC (`pio test -e
  native-stm32`) ; sur le matériel, la carte DevEBox nue a été vérifiée (carte
  SD, boîte noire), mais pas encore les capteurs ni les servos ;
- `hal/Rtos.h` — les tâches FreeRTOS, identiques sur les deux plateformes.

Tout ce qui se trouve au-dessus ne travaille qu’avec les interfaces ; passer à un
autre MCU, c’est donc écrire une nouvelle implémentation de `IBoard`, et non
réécrire les capteurs.

---

## namespace `ServoChannel`

**Fichier :** `hal/IBoard.h`

Les indices des sorties pour `IBoard::servo(channel)`. Une liste plate, et non
des méthodes nommées : ajouter une sortie ne change pas l’interface `IBoard`. L’ordre
est celui des lignes du tableau `FlightOutputs::outputInfo()`.

| Constante | Valeur |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 — largage de charge (`Feature::PAYLOAD_DROP`) |
| `AUX2` | 6 — caméra (`Knob::CAMERA_TILT`, `Feature::CAMERA_STAB`) |
| `COUNT` | 7 |

---

## `IBoard`

**Fichier :** `hal/IBoard.h` · **Genre :** interface · **Implémentations :** `Esp32Board`, `Stm32Board`

Le seul point d’entrée vers le matériel. Rien au-dessus n’inclut `<Wire.h>`,
`<SPI.h>` ni `HardwareSerial`, et rien n’appelle directement LEDC.

| Méthode | Description |
|---|---|
| `virtual void begin()` | Initialisation unique des bus I2C/SPI. Les UART sont ouverts par leurs propriétaires (`IBusReceiver`, GPS) à leur propre vitesse, le PWM par `FlightOutputs::begin()` |
| `virtual II2CBus& i2c()` | Le bus des capteurs |
| `virtual ISpiBus& spi()` | Le bus SPI |
| `virtual II2CBus* displayI2c()` | Un second bus I2C réservé à l’écran ; `nullptr` s’il n’y en a pas |
| `virtual IUartPort& rcUart()` | L’UART du récepteur iBUS |
| `virtual IUartPort& gpsUart()` | L’UART du GPS |
| `virtual IUartPort* telemetryUart()` | L’UART du modem radio MAVLink ; `nullptr` par défaut (l’ESP32 n’a pas d’UART libre) |
| `virtual IServoOutput& servo(uint8_t channel)` | Une sortie PWM d’après l’indice `ServoChannel::*` |
| `virtual void setBuzzer(bool on)` | Le buzzer `PIN_BUZZER` ; ne fait rien par défaut |

---

## `II2CBus`

**Fichier :** `hal/II2CBus.h` · **Genre :** interface avec des fonctions auxiliaires non virtuelles ·
**Implémentations :** `Esp32I2CBus`, `Stm32I2CBus`

Une abstraction du bus I2C calquée sur `Wire`. Les broches et la fréquence sont fixées
par l’implémentation dans le constructeur ; c’est pourquoi `begin()`/`setClock()` ne
prennent pas de broches — le bus est initialisé exactement une fois, même s’il porte
plusieurs périphériques.

| Méthode | Description |
|---|---|
| `begin()`, `setClock(hz)` | Initialisation, fréquence |
| `beginTransmission(addr)`, `write(byte)`, `write(data, len)`, `endTransmission(sendStop = true)` | Écriture ; `endTransmission` renvoie 0 en cas de succès (comme `Wire`) |
| `requestFrom(addr, n)`, `available()`, `read()` | Lecture |
| `bool writeRegister(addr, reg, value)` | Fonction auxiliaire : écrit un seul registre ; `false` — NACK |
| `bool readRegisters(addr, reg, buf, count)` | Fonction auxiliaire : redémarrage + lecture de `count` octets. `false` en cas de NACK **ou si moins de `count` octets sont arrivés** ; le tampon n’est alors pas touché |
| `int readRegister(addr, reg)` | La valeur du registre ou `-1` |
| `bool probe(addr)` | Le périphérique répond par un ACK à l’adresse |

Invariant : en cas d’échec, les fonctions auxiliaires n’écrivent pas dans le tampon —
le pilote conserve les données précédentes, et non des déchets (le `0xFF` que
`read()` renvoie sur un tampon vide).

---

## `ISpiBus`

**Fichier :** `hal/ISpiBus.h` · **Genre :** interface · **Implémentations :** `Esp32SpiBus`, `Stm32SpiBus`

Un bus SPI **sans gestion du CS** : plusieurs périphériques partagent un bus, et
c’est `SpiRegisterDevice` qui bascule le CS.

| Méthode | Description |
|---|---|
| `begin()` | Configure SCK/MISO/MOSI (les broches sont dans le constructeur de l’implémentation) |
| `beginTransaction(clockHz, spiMode)` | `spiMode` de 0 à 3 (CPOL/CPHA) |
| `uint8_t transfer(data)` | Échange d’un octet en full-duplex |
| `endTransaction()` | Fin de la transaction |

---

## `IUartPort`

**Fichier :** `hal/IUartPort.h` · **Genre :** interface · **Implémentations :** `Esp32UartPort`, `Stm32UartPort`

Un UART calqué sur `HardwareSerial`, mais `begin()` ne prend que la vitesse :
les broches et le format (8N1) sont fixés par l’implémentation.

| Méthode | Description |
|---|---|
| `begin(baud)` | Ouvrir le port |
| `int available()`, `int read()` | Réception |
| `size_t write(byte)`, `size_t write(buffer, size)` | Émission |
| `virtual int availableForWrite()` | Place libre dans le tampon d’émission ; `-1` — inconnue (par défaut). La télémétrie s’en sert pour reporter une trame plutôt que d’attendre |

---

## `IServoOutput`

**Fichier :** `hal/IServoOutput.h` · **Genre :** interface · **Implémentations :** `Esp32ServoOutput`, `Stm32ServoOutput`

Une seule sortie PWM. La broche est fixée par l’implémentation.

| Méthode | Description |
|---|---|
| `bool attach(minUs, maxUs)` | Alloue le canal/la minuterie et configure la broche ; la plage de limitation de l’impulsion. `true` indique seulement que le MCU a alloué les ressources, et **non** qu’un servo est branché |
| `writeMicroseconds(us)` | Largeur de l’impulsion, µs (bornée à la plage de `attach`) |
| `bool isAttached() const` | Le résultat de `attach()` |
| `virtual int32_t measurePulseUs()` | Diagnostic : la largeur réelle de l’impulsion sur la broche ou `-1`. L’implémentation par défaut renvoie `-1` |

---

## `IFlashRegion`

**Fichier :** `hal/IFlashRegion.h` · **Genre :** interface · **Implémentations :** `Esp32FlashPartition`, `SdFileRegion`

Une zone de flash NOR pour le journal (la boîte noire) : l’effacement ne se fait que
par secteurs de 4 Ko (ce qui est effacé se lit `0xFF`), l’écriture ne fait que
abaisser des bits — on peut écrire dans des octets effacés, y compris par morceaux
dans une même page. Sur l’ESP32, l’écriture comme l’effacement arrêtent les deux cœurs —
c’est à l’appelant de décider quand c’est acceptable.

| Méthode | Description |
|---|---|
| `uint32_t size() const` | Taille de la zone, en octets ; 0 — il n’y a pas de zone |
| `bool read(offset, data, length)` | Lire |
| `bool write(offset, data, length)` | Écrire (dans des octets effacés) |
| `bool erase(offset, length)` | Effacer ; l’adresse et la longueur sont des multiples de 4096 |

`Esp32FlashPartition(const char* name)` — une partition de données désignée par son nom
dans la table des partitions (`esp_partition_*`) ; `begin()` trouve la partition (une fois
le cœur démarré) et, si elle n’existe pas, renvoie `false` avec `size() == 0`.

---

## `IBlockDevice`

**Fichier :** `hal/IBlockDevice.h` · **Genre :** interface · **Implémentations :** `Stm32SdCard` (dans les tests — `fake::SdCardModel`)

Une carte SD vue comme un tableau de blocs de 512 octets. Il n’y a pas d’effacement : un bloc peut être réécrit.

| Méthode | Description |
|---|---|
| `uint32_t blockCount() const` | Taille en blocs ; 0 — il n’y a pas de carte |
| `bool read(block, data, count)` / `write(...)` | `count` blocs d’affilée, `data` — n’importe quelle adresse |

## `SdFileRegion`

**Fichier :** `hal/SdFileRegion.h` · **Hérite de :** `IFlashRegion` · **Dépend de :** `IBlockDevice`, `Fat32::locate`

La zone de la boîte noire sur la carte SD : un fichier à la racine du FAT32 (par
défaut `BLACKBOX.BIN`), créé à l’avance sur PC d’un seul tenant (`tools/blackbox.py
sd-prepare`) et rempli de `0xFF`. Le fichier est seulement **localisé** (les tables
FAT et le répertoire ne sont pas touchés) ; ensuite, des blocs bruts y sont écrits.
Pour `BlackBoxStorage`, c’est le même `IFlashRegion` que la partition de flash de l’ESP32.

| Méthode | Description |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes` est le plafond de la zone : le temps de vérification des secteurs à la mise sous tension croît avec lui |
| `Fat32::Result begin()` | Trouver le fichier. `Ok` — `size() > 0` ; sinon la raison (`Fat32::describe()`) : pas de carte, pas du FAT32, pas de fichier, fichier fragmenté, fichier vide |
| `size()` | Le fichier (au plus `maxBytes`), arrondi par défaut au secteur de 4 Ko ; 0 — il n’y a pas de zone |
| `read` / `write` | N’importe quel décalage et n’importe quelle longueur. Un bloc incomplet est lu, complété et écrit en entier ; le bloc qui vient d’être écrit est mémorisé (cache en écriture immédiate) : des pages de 256 octets consécutives ne relisent pas la carte. Une coupure d’alimentation ne fait rien perdre de ce qui est déjà revenu de `write()` |
| `erase(offset, length)` | Multiple de 4096 ; écrit `0xFF` (la carte a son propre effacement interne, l’extérieur n’en a pas besoin) |

## `IRegisterDevice`

**Fichier :** `hal/RegisterDevice.h` · **Genre :** interface ·
**Implémentations :** `I2cRegisterDevice`, `SpiRegisterDevice`

« Un ensemble de registres de 8 bits ». Le pilote d’un capteur s’écrit une seule fois,
et le bus est choisi à la création de l’objet dans `SensorSelection.h`.

| Méthode | Description |
|---|---|
| `virtual void begin()` | Prépare les lignes du périphérique (pour le SPI — le CS). Ne fait rien par défaut |
| `virtual bool probe()` | Le périphérique a répondu (pour le SPI, toujours `true` — il n’y a pas d’ACK, on vérifie le registre d’ID) |
| `virtual bool writeRegister(reg, value)` | Écriture d’un registre |
| `virtual bool writeRegisters(reg, data, count)` | Écriture consécutive (incrémentation automatique de l’adresse) |
| `virtual bool readRegisters(reg, buffer, count)` | Lecture de `count` octets consécutifs ; avec `false`, le tampon n’est pas touché |
| `int readRegister(reg)` | La valeur ou `-1` (une fonction auxiliaire non virtuelle) |

---

## `I2cRegisterDevice`

**Fichier :** `hal/RegisterDevice.h` · **Hérite de :** `IRegisterDevice`

Un périphérique sur un `II2CBus` à adresse de 7 bits. Toutes les opérations sont déléguées
aux fonctions auxiliaires de `II2CBus`.

| Méthode | Description |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress` est la seconde adresse de la puce (la broche SDO/SA0) : LSM6DSV 0x6A/0x6B, ICM-45686 0x68/0x69, SPL06 0x76/0x77, BMP581 0x46/0x47 |
| `begin()` | l’adresse principale ne répond pas mais l’adresse de secours répond — on continue alors avec l’adresse de secours |
| `probe()`, `writeRegister()`, `writeRegisters()`, `readRegisters()` | → les fonctions auxiliaires de `II2CBus(address, …)` |
| `uint8_t getAddress() const` | l’adresse actuelle du périphérique |

---

## `SpiRegisterDevice`

**Fichier :** `hal/RegisterDevice.h` · **Hérite de :** `IRegisterDevice`

Un périphérique sur un `ISpiBus` avec sa propre broche CS. Protocole Bosch/InvenSense :
la lecture, c’est l’adresse avec le bit `0x80` ; l’écriture, avec le bit 7 à zéro.

| Méthode | Description |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` est le nombre d’octets « poubelle » que la puce envoie après l’adresse avant les données (BMP388 — 1, ICM42688 — 0) ; `mode` est le mode SPI de 0 à 3 |
| `begin()` | `pinMode(cs, OUTPUT)`, CS = HIGH |
| `probe()` | Toujours `true` |
| `writeRegister(reg, value)` | CS↓, `reg & 0x7F`, `value`, CS↑ ; toujours `true` |
| `readRegisters(reg, buf, n)` | CS↓, `reg \| 0x80`, saut de `dummyReadBytes`, `n` octets, CS↑ ; toujours `true` |

Chaque opération est une transaction distincte `beginTransaction(clockHz, spiMode)` …
`endTransaction()`.

---

## `Esp32Board`

**Fichier :** `hal/esp32/Esp32Board.h` · **Hérite de :** `IBoard`

Le seul endroit qui crée les objets concrets des périphériques de l’ESP32 et qui
connaît les broches de `Config.h`.

| Champ | Type | Ce que c’est |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `Wire` sur `PIN_I2C_SDA/SCL`, 400 kHz |
| `displayBus` | `Esp32I2CBus` | `Wire1` sur `PIN_I2C2_SDA/SCL` — seulement si `SOC_I2C_NUM > 1` |
| `spiBus` | `Esp32SpiBus` | Le `SPI` global |
| `rcSerial`, `rcPort` | `HardwareSerial(1)`, `Esp32UartPort` | iBUS sur `PIN_IBUS`, RX seulement |
| `gpsSerial`, `gpsPort` | `HardwareSerial(UART_NUM_GPS)`, `Esp32UartPort` | GPS sur `PIN_GPS_RX/TX` |
| `servos[7]` | `Esp32ServoOutput` | Canaux LEDC 0 à 6 dans l’ordre de `ServoChannel` (AUX1/AUX2 — `PIN_AUX1/2`, si elles sont câblées) |

| Méthode | Description |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, puis `displayBus.begin()` si le second bus existe ; la broche du buzzer |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` si la broche est câblée |
| `displayI2c()` | `&displayBus` si `hasDisplayBus()`, sinon `nullptr` |
| `static constexpr bool hasDisplayBus()` | Les deux broches du second bus sont ≥ 0. Elle existe (comme le champ `displayBus`) seulement si `SOC_I2C_NUM > 1` — le C3 n’a qu’un seul contrôleur I2C |
| les autres | Renvoient les champs correspondants |

---

## `Esp32I2CBus`

**Fichier :** `hal/esp32/Esp32I2CBus.h` · **Hérite de :** `II2CBus`

Une fine enveloppe autour de `TwoWire` (`Wire` ou `Wire1`).

| Méthode | Description |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | Mémorise les paramètres |
| `begin()` | `wire.begin(sda, scl, hz)` et `wire.setTimeOut(TIMEOUT_MS)` — l’unique appel de `wire.begin()` |
| les autres | Délégation directe à `TwoWire` |

`TIMEOUT_MS = 5` : la lecture de 14 octets de l’IMU à 400 kHz prend ~0,4 ms ; une transaction
bloquée par une perturbation immobiliserait sinon la boucle pendant les 50 ms habituelles.

---

## `Esp32SpiBus`

**Fichier :** `hal/esp32/Esp32SpiBus.h` · **Hérite de :** `ISpiBus`

Une enveloppe autour du `SPI` global. `begin()` → `SPI.begin(sck, miso, mosi, -1)` (le CS
est géré par les périphériques). `beginTransaction()` construit `SPISettings(hz, MSBFIRST,
SPI_MODEn)` ; `spiModeOf()` convertit 0 à 3 en constantes Arduino, et une valeur
inconnue → `SPI_MODE0`.

---

## `Esp32UartPort`

**Fichier :** `hal/esp32/Esp32UartPort.h` · **Hérite de :** `IUartPort`

Une enveloppe autour de `HardwareSerial` : `begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)` ; `tx = -1` — réception seule. Le reste n’est que de la délégation.

---

## `Esp32ServoOutput`

**Fichier :** `hal/esp32/Esp32ServoOutput.h` · **Hérite de :** `IServoOutput`

Le PWM directement par LEDC (`ledcSetup/ledcAttachPin/ledcWrite` de l’Arduino core 2.x).
La bibliothèque ESP32Servo **n’est pas utilisée** : la version 3.2.1 sur le S3 confondait les blocs
MCPWM (GPIO6/7 répétaient GPIO4/5).

| Constante | Valeur |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384 (≈1,2 µs par pas) |

| Méthode | Description |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | Broche `< 0` — la sortie n’est pas câblée |
| `attach(minUs, maxUs)` | Mémorise la plage ; broche < 0 → `false` ; sinon `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | Pas attached — rien ; sinon `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | Active le tampon d’entrée du même GPIO (`PIN_INPUT_ENABLE`, la sortie n’est pas touchée) et mesure `pulseIn(pin, HIGH, 30 ms)` ; pas d’impulsion → `-1` |

Les canaux 2n et 2n+1 partagent une minuterie LEDC — toutes les sorties sont à 50 Hz, il n’y a donc pas de conflit.

---

# Implémentation pour le STM32H743

La carte de la génération suivante est la STM32H743VIT6 (Cortex-M7 à 480 MHz, 2 Mo de flash,
1 Mo de RAM). Le micrologiciel complet se compile (env `stm32h743` — la carte PlatformIO
`weact_mini_h743vitx`, et `stm32h743-devebox` — la DevEBox H743, console par
USB CDC), passe cppcheck et les tests sur PC (env `native-stm32` avec la couche de
simulation de STM32duino). Sur la carte DevEBox **sans capteurs**, ont été vérifiés : le démarrage, la carte SD, la boîte noire —
[tests sur la carte](../TESTING.md#tests-sur-la-carte-stm32) — ainsi que la réception iBUS, l’ARM, le PWM vers
les servos et le moteur : l’avion se pilote depuis la radiocommande en mode manuel (le lancement a été filmé).
Les capteurs n’ont pas encore été raccordés à la carte.
Le brochage se trouve dans le bloc `BOARD_STM32H743` de [`Config.h`](config.md#stm32h743vit6-board_stm32h743).

Les différences générales avec l’ESP32 que cette couche masque :

- **C’est le cœur qui choisit les périphériques.** STM32duino trouve lui-même le contrôleur
  (I2C1/I2C2, SPI2, USART3, UART4, UART7, TIMx) d’après les numéros de broches, grâce aux
  tableaux `PeripheralPins` de la variante ; il n’y a donc pas de numéros d’UART ni de canaux dans `Config.h`.
- **Les numéros de broches** sont les « broches Arduino » de la variante (`PA0`, `PD14`...), et non des GPIO ; pour
  les broches analogiques, ce sont `0xC0 + N`, d’où des broches de type `int16_t` dans le bloc STM32.
- **Les broches de l’UART** sont fixées à la création de l’objet `Uart(rx, tx)`, et non dans `begin()`.

## `Stm32Board`

**Fichier :** `hal/stm32/Stm32Board.h` · **Hérite de :** `IBoard`

La même chose que `Esp32Board`, par-dessus STM32duino.

| Champ | Type | Ce que c’est |
|---|---|---|
| `displayWire` | `TwoWire` | Le second contrôleur I2C (le `Wire` global est pris par les capteurs). Déclaré avant `displayBus`, qui en garde une référence |
| `i2cBus` | `Stm32I2CBus` | `Wire` sur `PIN_I2C_SDA/SCL` (I2C2 : PB11/PB10), 400 kHz |
| `displayBus` | `Stm32I2CBus` | `displayWire` sur `PIN_I2C2_SDA/SCL` (I2C1 : PB9/PB8) — le second bus est toujours présent |
| `spiBus` | `Stm32SpiBus` | Le `SPI` global sur `PIN_SENSOR_SPI_*` (SPI2) |
| `rcSerial`, `rcPort` | `Uart`, `Stm32UartPort` | iBUS : UART7, RX `PIN_IBUS` (PE7), TX `PIN_IBUS_TX` (PE8, réservé à l’iBUS-SENS) |
| `gpsSerial`, `gpsPort` | `Uart`, `Stm32UartPort` | GPS : USART3, `PIN_GPS_RX/TX` (PD9/PD8) |
| `telemetrySerial`, `telemetryPort` | `Uart`, `Stm32UartPort` | le modem radio MAVLink : UART4, `PIN_TELEM_RX/TX` (PD0/PD1) |
| `servos[7]` | `Stm32ServoOutput` | Dans l’ordre de `ServoChannel` (AUX1 — PD15/TIM4, AUX2 — PE9/TIM1) |

| Méthode | Description |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, `displayBus.begin()`, la broche du buzzer |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | Toujours `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | Convertit une broche de `Config.h` vers le type de l’API du cœur |
| les autres | Renvoient les champs correspondants |

## `Stm32I2CBus`

**Fichier :** `hal/stm32/Stm32I2CBus.h` · **Hérite de :** `II2CBus`

| Méthode | Description |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | Mémorise les paramètres |
| `begin()` | `setSDA()`/`setSCL()` (n’agissent qu’avant `begin()`), `wire.begin()`, `wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)` ; le résultat de type `size_t` est converti en `uint8_t` |
| les autres | Délégation directe à `TwoWire` |

Dans STM32duino, le délai d’attente de la transaction n’est pas une méthode mais la macro
`I2C_TIMEOUT_TICK` (ms, 100 par défaut). Dans l’env `stm32h743`, il est fixé par l’option
`-D I2C_TIMEOUT_TICK=5` — pour la même raison que `TIMEOUT_MS` dans `Esp32I2CBus`.

## `Stm32SpiBus`

**Fichier :** `hal/stm32/Stm32SpiBus.h` · **Hérite de :** `ISpiBus`

Une enveloppe autour de `SPIClass&`. `begin()` → `setSCLK/setMISO/setMOSI` + `spi.begin()` ;
le NSS matériel n’est pas utilisé — le CS est basculé par `SpiRegisterDevice`, comme sur l’ESP32.
`beginTransaction()` construit `SPISettings(hz, MSBFIRST, SPIMode)` ; `spiModeOf()`
convertit 0 à 3 en `SPI_MODEn`, et une valeur inconnue → `SPI_MODE0`.

## `Stm32UartPort`

**Fichier :** `hal/stm32/Stm32UartPort.h` · **Hérite de :** `IUartPort`

Une enveloppe autour de `HardwareSerial&` (dans STM32duino 3.x, c’est la base abstraite
`arduino::HardwareSerial` ; l’objet concret `Uart` est créé par `Stm32Board`).
`begin(baud)` → `serial.begin(baud, SERIAL_8N1)` ; `availableForWrite()` vient
de `HardwareSerial`. Les tampons (`SERIAL_RX/TX_BUFFER_SIZE` dans l’env) : réception 256
octets (une trame NAV-PVT en fait 100, les 64 standard sont trop peu), émission 1024 (lignes du journal et
trames MAVLink sans attente).

## `Stm32ServoOutput`

**Fichier :** `hal/stm32/Stm32ServoOutput.h` · **Hérite de :** `IServoOutput`

PWM matériel d’une minuterie via `HardwareTimer`, 50 Hz. L’impulsion est générée par la
minuterie sans interruptions et sans le CPU — contrairement à la bibliothèque `Servo` pour
STM32, qui fait basculer les broches depuis l’interruption d’une seule minuterie
et provoque de la gigue.

| Constante | Valeur |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4 — combien de minuteries différentes les sorties peuvent occuper (TIM2 et TIM4 le sont actuellement) |

| Méthode | Description |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | Broche `< 0` — la sortie n’est pas câblée |
| `attach(minUs, maxUs)` | La minuterie et le canal viennent de `PinMap_TIM` d’après la broche (`pinmap_peripheral`, `STM_PIN_CHANNEL`), comme pour `analogWrite()`. Pas de minuterie sur la broche ou réserve épuisée → `false`. Sinon `setMode(PWM1)`, comparaison 0 (pas d’impulsion avant la première écriture), `resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` → `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`. Le registre de comparaison est à préchargement — la valeur prend effet à la période suivante |
| `measurePulseUs()` | `pulseIn(pin, HIGH, 30 ms)` sans reconfigurer la broche : sur le STM32, le registre IDR voit le niveau même en mode de fonction alternative |
| `static acquireTimer(TIM_TypeDef*)` | Une réserve partagée : **un seul `HardwareTimer` par TIMx**. Un second objet sur la même minuterie écraserait le gestionnaire du cœur (`HardwareTimer_Handle[index]`). La période est fixée à la première sortie sur la minuterie ; `setOverflow(MICROSEC_FORMAT)` choisit le diviseur — un pas de ~0,3 µs avec une horloge de minuterie de 240 MHz |

## `Stm32FlashStorage`

**Fichier :** `hal/stm32/Stm32FlashStorage.h` · **Hérite de :** `IFlashStorage` ([storage.md](storage.md))

Le support du `KeyValueStore` sur STM32 : le dernier secteur de la flash (banque 2) via
l’émulation d’EEPROM de STM32duino (`eeprom_buffer_fill/flush`, un tampon de 8 Ko dont
les premiers `KeyValueStore::CAPACITY` octets sont utilisés).

| Méthode | Description |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` + lecture du tampon octet par octet |
| `write(src, n)` | **rapide** : copie l’image dans son propre tampon sous `noInterrupts()` et lève le drapeau « écriture en attente ». Appelée depuis `KvPreferences::end()` dans la tâche de vol |
| `bool service()` | **lente** : un instantané dans le tampon d’émulation (sous `noInterrupts()`) et `eeprom_buffer_flush()` — effacement d’un secteur de 128 Ko (des secondes) et écriture. Uniquement depuis la tâche de fond `storage` |
| `hasPending()`, `flushCount()` | diagnostic |
| `static instance()`, `static store()` | le support et le `KeyValueStore` commun du micrologiciel |

Pourquoi le vol ne se fige pas : le secteur des réglages est dans la banque 2, le code dans la
banque 1 ; la flash du H7 peut lire une banque pendant que l’autre est écrite ;
la tâche de vol préempte la tâche de fond.

## `compat/Preferences.h`

**Fichier :** `hal/stm32/compat/Preferences.h` — dans l’env `stm32h743` (et
`native-stm32`), le répertoire `compat/` figure dans `-I` avant les bibliothèques, et
le `#include <Preferences.h>` des pilotes de capteurs, de l’auto-trim et des réglages
du journal le trouve. `class Preferences : public KvPreferences` par-dessus
`Stm32FlashStorage::store()` — la même API que le NVS de l’ESP32 ([storage.md](storage.md#kvpreferences)).

## `Stm32SdCard`

**Fichier :** `hal/stm32/Stm32SdCard.h` · **Hérite de :** `IBlockDevice` · **Broches :** `src/stm32/sd_msp.cpp`

Une carte SD sur SDMMC1 : bus de 4 bits, `HAL_SD` en mode scrutation (sans DMA ni
interruptions) **avec contrôle de flux matériel** : la tâche de vol préempte la tâche
d’écriture au milieu d’un bloc, et sans lui la FIFO débordait
(`HAL_SD_ERROR_RX_OVERRUN`, 0x20) — sur la carte, cela se traduisait par une console et un
enregistrement figés pendant des secondes. Les broches PC8..PC11 (D0..D3), PC12 (CK), PD2 (CMD) sont le slot µSD de la
DevEBox et de WeAct. Le cœur SDMMC est cadencé par PLL1Q = 48 MHz, `ClockDiv = 1` →
**24 MHz** ; si la première lecture à 24 MHz échoue, on essaie 12 puis 6.

| Membre | Description |
|---|---|
| `bool begin()` | Monter le bus, identifier la carte, lecture d’essai. `false` — il n’y a pas de carte ; `initError()` — le code |
| `read` / `write` | Par morceaux d’au plus 4 Ko (courtes pauses) ; une adresse non multiple de 4 est copiée via un tampon aligné (le HAL lit la FIFO par mots). En cas d’échec — une nouvelle tentative |
| attente | Avant un accès faisant suite à une écriture, attend que la carte revienne à l’état de transfert (`Rtos::sleepMs(1)` : les tâches de fond ne sont pas affamées), jusqu’à 1 s. Après une lecture, aucune requête d’état superflue n’est envoyée — la vérification à la mise sous tension lit des dizaines de milliers de secteurs |
| `blockCount()`, `cardType()`, `clockDivider()`, `lastErrorCode()` | Pour la ligne d’état |
| `readOps`, `writeOps`, `errors`, `retries` | Compteurs |

## `ResetCause`

**Fichier :** `hal/ResetCause.h` · `readResetCause()`, `isCrashReset()`, `resetCauseName()`

La cause du redémarrage, identique sur les deux cartes. ESP32 — `esp_reset_reason()` ;
STM32 — les drapeaux de `RCC->RSR` (lus une fois puis effacés ; sur le H7, `PINRSTF`
est positionné à n’importe quelle réinitialisation, c’est pourquoi on vérifie d’abord les causes plus
précises : chien de garde → mise sous tension → chute de tension → réinitialisation logicielle).
Une panique, les chiens de garde et une chute de l’alimentation comptent comme une « défaillance » :
la boîte noire commence alors aussitôt à enregistrer.

## `Rtos`

**Fichier :** `hal/Rtos.h` · namespace

| Membre | Description |
|---|---|
| `PRIORITY_BACKGROUND` (1), `PRIORITY_TELEMETRY` (2), `PRIORITY_FLIGHT` (5) | priorités des tâches |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | ESP32 — `xTaskCreatePinnedToCore(..., cœur 0)`, pile en octets ; STM32 — `xTaskCreate`, la pile est convertie en mots ; `handle` sert à `xTaskNotifyGive` |
| `void sleepMs(ms)` | `vTaskDelay` ; avant le démarrage de l’ordonnanceur (le `setup()` du STM32) — `delay()` |
| `class CriticalSection` | `enter()`/`exit()` : ESP32 — le verrou tournant `portMUX`, STM32 — `taskENTER_CRITICAL()`. À l’intérieur, uniquement de la copie d’octets (la file de la boîte noire) |
| `uint32_t freeHeapBytes()` | ESP32 — `ESP.getFreeHeap()` ; STM32 — `xPortGetFreeHeapSize()` |

## Le point d’entrée `src/stm32/main.cpp`

Le micrologiciel complet : les mêmes objets que `src/main.cpp`, la télémétrie MAVLink à la place du
Wi-Fi, des tâches FreeRTOS à la place de `loop()` — [application.md](application.md#srcstm32maincpp--stm32h743).
