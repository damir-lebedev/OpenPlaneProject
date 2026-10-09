# HAL — abstração do hardware

> 🌐 Esta página é uma tradução do [original em russo](../../../reference/hal.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referência](README.md)

O HAL é a única camada autorizada a conhecer um MCU específico. As interfaces
ficam em `include/hal/`; as implementações são:

- `include/hal/esp32/` — ESP32 (Arduino core 2.0.x);
- `include/hal/stm32/` — STM32H743 (STM32duino 3.x), **a principal**: o firmware completo compila (`pio run -e stm32h743-devebox`) e roda no PC (`pio test -e native-stm32`); na placa DevEBox foram verificados o cartão SD, a caixa-preta, o iBUS e os servos, mas ainda não os sensores;
- `hal/Rtos.h` — tarefas do FreeRTOS, iguais nas duas plataformas.

Tudo o que está acima trabalha apenas com as interfaces; por isso, migrar para outro
MCU significa uma nova implementação de `IBoard`, e não reescrever os sensores.

---

## namespace `ServoChannel`

**Arquivo:** `hal/IBoard.h`

Índices das saídas para `IBoard::servo(channel)`. É uma lista plana, e não métodos
nomeados: acrescentar uma saída não altera a interface `IBoard`. A ordem coincide
com as linhas da tabela `FlightOutputs::outputInfo()`.

| Constante | Valor |
|---|---|
| `AILERON_LEFT` | 0 |
| `AILERON_RIGHT` | 1 |
| `ELEVATOR` | 2 |
| `ESC` | 3 |
| `RUDDER` | 4 |
| `AUX1` | 5 — lançamento de carga (`Feature::PAYLOAD_DROP`) |
| `AUX2` | 6 — câmera (`Knob::CAMERA_TILT`, `Feature::CAMERA_STAB`) |
| `COUNT` | 7 |

---

## `IBoard`

**Arquivo:** `hal/IBoard.h` · **Tipo:** interface · **Implementações:** `Esp32Board`, `Stm32Board`

O único ponto de entrada para o hardware. Nada acima inclui `<Wire.h>`,
`<SPI.h>` ou `HardwareSerial`, e nada chama o LEDC diretamente.

| Método | Descrição |
|---|---|
| `virtual void begin()` | Inicialização única dos barramentos I2C/SPI. As UARTs são abertas por seus donos (`IBusReceiver`, GPS) com a velocidade própria, e o PWM, por `FlightOutputs::begin()` |
| `virtual II2CBus& i2c()` | O barramento dos sensores |
| `virtual ISpiBus& spi()` | O barramento SPI |
| `virtual II2CBus* displayI2c()` | Um segundo barramento I2C só para a tela; `nullptr` se não houver |
| `virtual IUartPort& rcUart()` | A UART do receptor iBUS |
| `virtual IUartPort& gpsUart()` | A UART do GPS |
| `virtual IUartPort* telemetryUart()` | A UART do modem de rádio MAVLink; por padrão `nullptr` (o ESP32 não tem UART livre) |
| `virtual IServoOutput& servo(uint8_t channel)` | Uma saída PWM pelo índice `ServoChannel::*` |
| `virtual void setBuzzer(bool on)` | O buzzer `PIN_BUZZER`; por padrão não faz nada |

---

## `II2CBus`

**Arquivo:** `hal/II2CBus.h` · **Tipo:** interface com auxiliares não virtuais ·
**Implementações:** `Esp32I2CBus`, `Stm32I2CBus`

Uma abstração do barramento I2C no formato do `Wire`. Os pinos e a frequência são
fixados pela implementação no construtor; por isso `begin()`/`setClock()` não recebem
pinos — o barramento é inicializado exatamente uma vez, mesmo que haja vários dispositivos nele.

| Método | Descrição |
|---|---|
| `begin()`, `setClock(hz)` | Inicialização, frequência |
| `beginTransmission(addr)`, `write(byte)`, `write(data, len)`, `endTransmission(sendStop = true)` | Escrita; `endTransmission` retorna 0 em caso de sucesso (como o `Wire`) |
| `requestFrom(addr, n)`, `available()`, `read()` | Leitura |
| `bool writeRegister(addr, reg, value)` | Auxiliar: escreve um único registrador; `false` — NACK |
| `bool readRegisters(addr, reg, buf, count)` | Auxiliar: start repetido + leitura de `count` bytes. `false` se houver NACK **ou se chegarem menos de `count` bytes**; o buffer não é tocado |
| `int readRegister(addr, reg)` | O valor do registrador ou `-1` |
| `bool probe(addr)` | O dispositivo responde ao endereço com ACK |

Invariante: em caso de falha, os auxiliares não escrevem no buffer — o driver mantém
os dados anteriores, e não lixo (o `0xFF` que `read()` devolve com o buffer vazio).

---

## `ISpiBus`

**Arquivo:** `hal/ISpiBus.h` · **Tipo:** interface · **Implementações:** `Esp32SpiBus`, `Stm32SpiBus`

Um barramento SPI **sem gerenciamento de CS**: vários dispositivos dividem um barramento,
e é o `SpiRegisterDevice` que comuta o CS.

| Método | Descrição |
|---|---|
| `begin()` | Configura SCK/MISO/MOSI (os pinos estão no construtor da implementação) |
| `beginTransaction(clockHz, spiMode)` | `spiMode` de 0 a 3 (CPOL/CPHA) |
| `uint8_t transfer(data)` | Troca de um byte em full duplex |
| `endTransaction()` | Fim da transação |

---

## `IUartPort`

**Arquivo:** `hal/IUartPort.h` · **Tipo:** interface · **Implementações:** `Esp32UartPort`, `Stm32UartPort`

Uma UART no formato do `HardwareSerial`, mas o `begin()` recebe apenas a velocidade:
os pinos e o formato (8N1) são fixados pela implementação.

| Método | Descrição |
|---|---|
| `begin(baud)` | Abrir a porta |
| `int available()`, `int read()` | Recepção |
| `size_t write(byte)`, `size_t write(buffer, size)` | Transmissão |
| `virtual int availableForWrite()` | Espaço livre no buffer de transmissão; `-1` — desconhecido (o padrão). A telemetria o usa para adiar um quadro em vez de esperar |

---

## `IServoOutput`

**Arquivo:** `hal/IServoOutput.h` · **Tipo:** interface · **Implementações:** `Esp32ServoOutput`, `Stm32ServoOutput`

Uma única saída PWM. O pino é fixado pela implementação.

| Método | Descrição |
|---|---|
| `bool attach(minUs, maxUs)` | Aloca o canal/temporizador e configura o pino; a faixa de limitação do pulso. `true` indica apenas que o MCU alocou os recursos, e **não** que há um servo conectado |
| `writeMicroseconds(us)` | Largura do pulso, µs (limitada à faixa de `attach`) |
| `bool isAttached() const` | O resultado de `attach()` |
| `virtual int32_t measurePulseUs()` | Diagnóstico: a largura real do pulso no pino ou `-1`. A implementação padrão devolve `-1` |

---

## `IFlashRegion`

**Arquivo:** `hal/IFlashRegion.h` · **Tipo:** interface · **Implementações:** `Esp32FlashPartition`, `SdFileRegion`

Uma região de flash NOR para o registro (a caixa-preta): o apagamento é só por
setores de 4 KB (o que foi apagado é lido como `0xFF`), e a escrita apenas zera
bits — é possível escrever em bytes apagados, inclusive em partes dentro de uma mesma
página. No ESP32, tanto a escrita quanto o apagamento param os dois núcleos — quem
chama decide quando isso é aceitável.

| Método | Descrição |
|---|---|
| `uint32_t size() const` | Tamanho da região, em bytes; 0 — não há região |
| `bool read(offset, data, length)` | Ler |
| `bool write(offset, data, length)` | Escrever (em bytes apagados) |
| `bool erase(offset, length)` | Apagar; o endereço e o comprimento são múltiplos de 4096 |

`Esp32FlashPartition(const char* name)` — uma partição de dados pelo nome na
tabela de partições (`esp_partition_*`); `begin()` encontra a partição (depois que o
núcleo inicia) e, se ela não existir, retorna `false` e `size() == 0`.

---

## `IBlockDevice`

**Arquivo:** `hal/IBlockDevice.h` · **Tipo:** interface · **Implementações:** `Stm32SdCard` (nos testes — `fake::SdCardModel`)

Um cartão SD como um array de blocos de 512 bytes. Não há apagamento: um bloco pode ser sobrescrito.

| Método | Descrição |
|---|---|
| `uint32_t blockCount() const` | Tamanho em blocos; 0 — não há cartão |
| `bool read(block, data, count)` / `write(...)` | `count` blocos seguidos, `data` — qualquer endereço |

## `SdFileRegion`

**Arquivo:** `hal/SdFileRegion.h` · **Herda de:** `IFlashRegion` · **Depende de:** `IBlockDevice`, `Fat32::locate`

A região da caixa-preta no cartão SD: um arquivo na raiz do FAT32 (por padrão
`BLACKBOX.BIN`), criado antes no PC em um único trecho contíguo (`tools/blackbox.py
sd-prepare`) e preenchido com `0xFF`. O arquivo apenas é **localizado** (as tabelas
FAT e o diretório não são tocados); depois, blocos brutos são escritos dentro dele.
Para o `BlackBoxStorage`, é o mesmo `IFlashRegion` da partição de flash do ESP32.

| Método | Descrição |
|---|---|
| `SdFileRegion(device, fileName, maxBytes)` | `maxBytes` é o teto da região: o tempo de conferência dos setores na inicialização cresce junto com ele |
| `Fat32::Result begin()` | Encontrar o arquivo. `Ok` — `size() > 0`; caso contrário, o motivo (`Fat32::describe()`): não há cartão, não é FAT32, não há arquivo, está fragmentado, está vazio |
| `size()` | O arquivo (no máximo `maxBytes`), arredondado para baixo até um setor de 4 KB; 0 — não há região |
| `read` / `write` | Qualquer deslocamento e comprimento. Um bloco incompleto é lido, completado e escrito inteiro; o bloco que acabou de ser escrito é lembrado (cache write-through): páginas de 256 bytes em sequência não leem o cartão de novo. Uma queda de energia não perde nada do que já retornou de `write()` |
| `erase(offset, length)` | Múltiplo de 4096; escreve `0xFF` (o cartão tem seu próprio apagamento interno, e por fora ele não é necessário) |

## `IRegisterDevice`

**Arquivo:** `hal/RegisterDevice.h` · **Tipo:** interface ·
**Implementações:** `I2cRegisterDevice`, `SpiRegisterDevice`

“Um conjunto de registradores de 8 bits”. O driver de um sensor é escrito uma vez,
e o barramento é escolhido ao criar o objeto em `SensorSelection.h`.

| Método | Descrição |
|---|---|
| `virtual void begin()` | Prepara as linhas do dispositivo (no SPI — o CS). Por padrão não faz nada |
| `virtual bool probe()` | O dispositivo respondeu (no SPI é sempre `true` — não há ACK, verifica-se o registrador de ID) |
| `virtual bool writeRegister(reg, value)` | Escrever um registrador |
| `virtual bool writeRegisters(reg, data, count)` | Escrever em sequência (autoincremento do endereço) |
| `virtual bool readRegisters(reg, buffer, count)` | Ler `count` bytes em sequência; com `false`, o buffer não é tocado |
| `int readRegister(reg)` | O valor ou `-1` (um auxiliar não virtual) |

---

## `I2cRegisterDevice`

**Arquivo:** `hal/RegisterDevice.h` · **Herda de:** `IRegisterDevice`

Um dispositivo em um `II2CBus` com endereço de 7 bits. Todas as operações são delegadas
aos auxiliares do `II2CBus`.

| Método | Descrição |
|---|---|
| `I2cRegisterDevice(II2CBus& i2cBus, uint8_t deviceAddress, uint8_t alternateAddress = 0)` | `alternateAddress` é o segundo endereço do chip (o pino SDO/SA0): LSM6DSV 0x6A/0x6B, ICM-45686 0x68/0x69, SPL06 0x76/0x77, BMP581 0x46/0x47 |
| `begin()` | o principal não responde e o alternativo responde — daí em diante trabalha-se com o alternativo |
| `probe()`, `writeRegister()`, `writeRegisters()`, `readRegisters()` | → os auxiliares do `II2CBus(address, …)` |
| `uint8_t getAddress() const` | o endereço atual do dispositivo |

---

## `SpiRegisterDevice`

**Arquivo:** `hal/RegisterDevice.h` · **Herda de:** `IRegisterDevice`

Um dispositivo em um `ISpiBus` com pino CS próprio. Protocolo Bosch/InvenSense:
a leitura é o endereço com o bit `0x80`, e a escrita, com o bit 7 zerado.

| Método | Descrição |
|---|---|
| `SpiRegisterDevice(ISpiBus& spiBus, uint8_t chipSelectPin, uint32_t clockFrequencyHz = 8 MHz, uint8_t dummyBytesBeforeData = 0, uint8_t mode = 0)` | `dummyBytesBeforeData` é quantos bytes “lixo” o chip entrega após o endereço antes dos dados (BMP388 — 1, ICM42688 — 0); `mode` é o modo SPI de 0 a 3 |
| `begin()` | `pinMode(cs, OUTPUT)`, CS = HIGH |
| `probe()` | Sempre `true` |
| `writeRegister(reg, value)` | CS↓, `reg & 0x7F`, `value`, CS↑; sempre `true` |
| `readRegisters(reg, buf, n)` | CS↓, `reg \| 0x80`, pula `dummyReadBytes`, `n` bytes, CS↑; sempre `true` |

Cada operação é uma transação independente `beginTransaction(clockHz, spiMode)` …
`endTransaction()`.

---

## `Esp32Board`

**Arquivo:** `hal/esp32/Esp32Board.h` · **Herda de:** `IBoard`

O único lugar que cria os objetos concretos dos periféricos do ESP32 e conhece os
pinos de `Config.h`.

| Campo | Tipo | O que é |
|---|---|---|
| `i2cBus` | `Esp32I2CBus` | `Wire` em `PIN_I2C_SDA/SCL`, 400 kHz |
| `displayBus` | `Esp32I2CBus` | `Wire1` em `PIN_I2C2_SDA/SCL` — só se `SOC_I2C_NUM > 1` |
| `spiBus` | `Esp32SpiBus` | O `SPI` global |
| `rcSerial`, `rcPort` | `HardwareSerial(1)`, `Esp32UartPort` | iBUS em `PIN_IBUS`, somente RX |
| `gpsSerial`, `gpsPort` | `HardwareSerial(UART_NUM_GPS)`, `Esp32UartPort` | GPS em `PIN_GPS_RX/TX` |
| `servos[7]` | `Esp32ServoOutput` | Canais LEDC de 0 a 6 na ordem de `ServoChannel` (AUX1/AUX2 — `PIN_AUX1/2`, se estiverem ligados) |

| Método | Descrição |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, depois `displayBus.begin()` se o segundo barramento existir; o pino do buzzer |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` se o pino estiver ligado |
| `displayI2c()` | `&displayBus` se `hasDisplayBus()`, caso contrário `nullptr` |
| `static constexpr bool hasDisplayBus()` | Os dois pinos do segundo barramento são ≥ 0. Existe (assim como o campo `displayBus`) apenas com `SOC_I2C_NUM > 1` — o C3 tem um único controlador I2C |
| os demais | Retornam os campos correspondentes |

---

## `Esp32I2CBus`

**Arquivo:** `hal/esp32/Esp32I2CBus.h` · **Herda de:** `II2CBus`

Um invólucro fino sobre o `TwoWire` (`Wire` ou `Wire1`).

| Método | Descrição |
|---|---|
| `Esp32I2CBus(TwoWire& bus, int8_t sdaPin, int8_t sclPin, uint32_t frequencyHz = 400000)` | Guarda os parâmetros |
| `begin()` | `wire.begin(sda, scl, hz)` e `wire.setTimeOut(TIMEOUT_MS)` — a única chamada de `wire.begin()` |
| os demais | Delegação direta ao `TwoWire` |

`TIMEOUT_MS = 5`: ler 14 bytes da IMU a 400 kHz leva ~0,4 ms; uma transação
travada por interferência, de outro modo, pararia o laço pelos 50 ms padrão.

---

## `Esp32SpiBus`

**Arquivo:** `hal/esp32/Esp32SpiBus.h` · **Herda de:** `ISpiBus`

Um invólucro sobre o `SPI` global. `begin()` → `SPI.begin(sck, miso, mosi, -1)` (o CS
fica com os dispositivos). `beginTransaction()` monta `SPISettings(hz, MSBFIRST,
SPI_MODEn)`; `spiModeOf()` converte 0 a 3 nas constantes do Arduino, e um valor
desconhecido → `SPI_MODE0`.

---

## `Esp32UartPort`

**Arquivo:** `hal/esp32/Esp32UartPort.h` · **Herda de:** `IUartPort`

Um invólucro sobre o `HardwareSerial`: `begin(baud)` → `serial.begin(baud, SERIAL_8N1,
rx, tx)`; `tx = -1` — somente recepção. O resto é delegação.

---

## `Esp32ServoOutput`

**Arquivo:** `hal/esp32/Esp32ServoOutput.h` · **Herda de:** `IServoOutput`

PWM diretamente pelo LEDC (`ledcSetup/ledcAttachPin/ledcWrite` do Arduino core 2.x).
A biblioteca ESP32Servo **não é usada**: a versão 3.2.1 no S3 confundia os blocos
MCPWM (o GPIO6/7 repetia o GPIO4/5).

| Constante | Valor |
|---|---|
| `FREQUENCY_HZ` | 50 |
| `PERIOD_US` | 20 000 |
| `RESOLUTION_BITS` / `MAX_DUTY` | 14 / 16384 (≈1,2 µs por passo) |

| Método | Descrição |
|---|---|
| `Esp32ServoOutput(int8_t pin, uint8_t ledcChannel)` | Pino `< 0` — a saída não está ligada |
| `attach(minUs, maxUs)` | Guarda a faixa; pino < 0 → `false`; caso contrário `ledcSetup() != 0` → `ledcAttachPin()` |
| `writeMicroseconds(us)` | Se não estiver attached, nada; senão `constrain(us, min, max) * MAX_DUTY / PERIOD_US` → `ledcWrite` |
| `measurePulseUs()` | Ativa o buffer de entrada do mesmo GPIO (`PIN_INPUT_ENABLE`, a saída não é tocada) e mede `pulseIn(pin, HIGH, 30 ms)`; sem pulso → `-1` |

Os canais 2n e 2n+1 dividem um temporizador do LEDC — todas as saídas operam a 50 Hz, então não há conflito.

---

# Implementação para o STM32H743

A placa da próxima geração é a STM32H743VIT6 (Cortex-M7 de 480 MHz, 2 MB de flash,
1 MB de RAM). O firmware completo compila (env `stm32h743` — a placa do PlatformIO
`weact_mini_h743vitx`, e `stm32h743-devebox` — a DevEBox H743, console por
USB CDC), passa no cppcheck e nos testes no PC (env `native-stm32` com a camada de
simulação do STM32duino). Na placa DevEBox **sem sensores** foram verificados: a inicialização, o cartão SD, a caixa-preta —
[testes na placa](../TESTING.md#testes-na-placa-stm32) — e a recepção de iBUS, o ARM, o PWM para
os servos e o motor: o avião é controlado pelo rádio no modo manual (o lançamento foi gravado em vídeo).
Os sensores ainda não foram conectados à placa.
A distribuição dos pinos é o bloco `BOARD_STM32H743` em [`Config.h`](config.md#stm32h743vit6-board_stm32h743).

As diferenças gerais em relação ao ESP32 que esta camada esconde:

- **O núcleo escolhe os periféricos.** O STM32duino encontra sozinho o controlador
  (I2C1/I2C2, SPI2, USART3, UART4, UART7, TIMx) pelos números dos pinos, usando as
  tabelas `PeripheralPins` da variante; por isso não há números de UART/canais em `Config.h`.
- **Os números dos pinos** são os “pinos Arduino” da variante (`PA0`, `PD14`...), e não GPIOs; nos
  pinos analógicos são `0xC0 + N`, e por isso os pinos no bloco STM32 são `int16_t`.
- **Os pinos da UART** são definidos ao criar o objeto `Uart(rx, tx)`, e não em `begin()`.

## `Stm32Board`

**Arquivo:** `hal/stm32/Stm32Board.h` · **Herda de:** `IBoard`

O mesmo que o `Esp32Board`, sobre o STM32duino.

| Campo | Tipo | O que é |
|---|---|---|
| `displayWire` | `TwoWire` | O segundo controlador I2C (o `Wire` global está ocupado pelos sensores). Declarado antes de `displayBus`, que guarda uma referência a ele |
| `i2cBus` | `Stm32I2CBus` | `Wire` em `PIN_I2C_SDA/SCL` (I2C2: PB11/PB10), 400 kHz |
| `displayBus` | `Stm32I2CBus` | `displayWire` em `PIN_I2C2_SDA/SCL` (I2C1: PB9/PB8) — o segundo barramento está sempre presente |
| `spiBus` | `Stm32SpiBus` | O `SPI` global em `PIN_SENSOR_SPI_*` (SPI2) |
| `rcSerial`, `rcPort` | `Uart`, `Stm32UartPort` | iBUS: UART7, RX `PIN_IBUS` (PE7), TX `PIN_IBUS_TX` (PE8, reservado para o iBUS-SENS) |
| `gpsSerial`, `gpsPort` | `Uart`, `Stm32UartPort` | GPS: USART3, `PIN_GPS_RX/TX` (PD9/PD8) |
| `telemetrySerial`, `telemetryPort` | `Uart`, `Stm32UartPort` | o modem de rádio MAVLink: UART4, `PIN_TELEM_RX/TX` (PD0/PD1) |
| `servos[7]` | `Stm32ServoOutput` | Na ordem de `ServoChannel` (AUX1 — PD15/TIM4, AUX2 — PE9/TIM1) |

| Método | Descrição |
|---|---|
| `begin()` | `i2cBus.begin()`, `spiBus.begin()`, `displayBus.begin()`, o pino do buzzer |
| `telemetryUart()` | `&telemetryPort` |
| `setBuzzer(on)` | `digitalWrite(PIN_BUZZER)` |
| `displayI2c()` | Sempre `&displayBus` |
| `static constexpr pin_size_t pinOf(int16_t)` | Converte um pino de `Config.h` para o tipo da API do núcleo |
| os demais | Retornam os campos correspondentes |

## `Stm32I2CBus`

**Arquivo:** `hal/stm32/Stm32I2CBus.h` · **Herda de:** `II2CBus`

| Método | Descrição |
|---|---|
| `Stm32I2CBus(TwoWire& bus, pin_size_t sdaPin, pin_size_t sclPin, uint32_t frequencyHz = 400000)` | Guarda os parâmetros |
| `begin()` | `setSDA()`/`setSCL()` (só têm efeito antes de `begin()`), `wire.begin()`, `wire.setClock(hz)` |
| `requestFrom(address, n)` | `wire.requestFrom(address, size_t n)`; o resultado `size_t` é convertido para `uint8_t` |
| os demais | Delegação direta ao `TwoWire` |

No STM32duino, o tempo limite da transação não é um método, mas a macro
`I2C_TIMEOUT_TICK` (ms, 100 por padrão). No env `stm32h743` ela é definida pela flag
`-D I2C_TIMEOUT_TICK=5` — pelo mesmo motivo do `TIMEOUT_MS` no `Esp32I2CBus`.

## `Stm32SpiBus`

**Arquivo:** `hal/stm32/Stm32SpiBus.h` · **Herda de:** `ISpiBus`

Um invólucro sobre `SPIClass&`. `begin()` → `setSCLK/setMISO/setMOSI` + `spi.begin()`;
o NSS por hardware não é usado — o CS é comutado pelo `SpiRegisterDevice`, como no ESP32.
`beginTransaction()` monta `SPISettings(hz, MSBFIRST, SPIMode)`; `spiModeOf()`
converte 0 a 3 em `SPI_MODEn`, e um valor desconhecido → `SPI_MODE0`.

## `Stm32UartPort`

**Arquivo:** `hal/stm32/Stm32UartPort.h` · **Herda de:** `IUartPort`

Um invólucro sobre `HardwareSerial&` (no STM32duino 3.x, é a base abstrata
`arduino::HardwareSerial`; o objeto concreto `Uart` é criado pelo `Stm32Board`).
`begin(baud)` → `serial.begin(baud, SERIAL_8N1)`; `availableForWrite()` vem
do `HardwareSerial`. Os buffers (`SERIAL_RX/TX_BUFFER_SIZE` no env): recepção de 256
bytes (um quadro NAV-PVT tem 100, os 64 padrão são poucos), transmissão de 1024 (linhas do registro e
quadros MAVLink sem espera).

## `Stm32ServoOutput`

**Arquivo:** `hal/stm32/Stm32ServoOutput.h` · **Herda de:** `IServoOutput`

PWM por hardware de um temporizador, via `HardwareTimer`, a 50 Hz. O pulso é gerado
pelo temporizador, sem interrupções e sem a CPU — ao contrário da biblioteca `Servo`
para STM32, que alterna os pinos a partir da interrupção de um único temporizador
e causa jitter.

| Constante | Valor |
|---|---|
| `FREQUENCY_HZ` / `PERIOD_US` | 50 / 20 000 |
| `MAX_TIMERS` | 4 — quantos temporizadores diferentes as saídas podem ocupar (hoje estão ocupados o TIM2 e o TIM4) |

| Método | Descrição |
|---|---|
| `Stm32ServoOutput(int16_t pin)` | Pino `< 0` — a saída não está ligada |
| `attach(minUs, maxUs)` | O temporizador e o canal vêm de `PinMap_TIM` pelo pino (`pinmap_peripheral`, `STM_PIN_CHANNEL`), como no `analogWrite()`. Sem temporizador no pino ou com o conjunto esgotado → `false`. Caso contrário, `setMode(PWM1)`, comparação 0 (sem pulso até a primeira escrita), `resume()` |
| `writeMicroseconds(us)` | `constrain(us, min, max)` → `setCaptureCompare(..., MICROSEC_COMPARE_FORMAT)`. O registrador de comparação tem pré-carga — o valor passa a valer no próximo período |
| `measurePulseUs()` | `pulseIn(pin, HIGH, 30 ms)` sem reconfigurar o pino: no STM32, o registrador IDR enxerga o nível mesmo no modo de função alternativa |
| `static acquireTimer(TIM_TypeDef*)` | Um conjunto compartilhado: **um `HardwareTimer` por TIMx**. Um segundo objeto para o mesmo temporizador sobrescreveria o tratador do núcleo (`HardwareTimer_Handle[index]`). O período é definido na primeira saída do temporizador; `setOverflow(MICROSEC_FORMAT)` escolhe o divisor — passo de ~0,3 µs com o clock do temporizador a 240 MHz |

## `Stm32FlashStorage`

**Arquivo:** `hal/stm32/Stm32FlashStorage.h` · **Herda de:** `IFlashStorage` ([storage.md](storage.md))

O meio do `KeyValueStore` no STM32: o último setor da flash (banco 2) por meio da
emulação de EEPROM do STM32duino (`eeprom_buffer_fill/flush`, um buffer de 8 KB do qual
são usados os primeiros `KeyValueStore::CAPACITY` bytes).

| Método | Descrição |
|---|---|
| `capacity()` | `min(KeyValueStore::CAPACITY, E2END + 1)` |
| `read(dst, n)` | `eeprom_buffer_fill()` + leitura do buffer byte a byte |
| `write(src, n)` | **rápido**: copia a imagem para o próprio buffer sob `noInterrupts()` e levanta o sinalizador de “há escrita pendente”. Chamado de `KvPreferences::end()` na tarefa de voo |
| `bool service()` | **lento**: um instantâneo no buffer da emulação (sob `noInterrupts()`) e `eeprom_buffer_flush()` — apagamento de um setor de 128 KB (segundos) e escrita. Somente a partir da tarefa de segundo plano `storage` |
| `hasPending()`, `flushCount()` | diagnóstico |
| `static instance()`, `static store()` | o meio e o `KeyValueStore` compartilhado do firmware |

Por que o voo não trava: o setor de configurações está no banco 2 e o código no banco 1;
a flash do H7 lê um banco enquanto o outro é escrito; a tarefa de voo preempta a de segundo plano.

## `compat/Preferences.h`

**Arquivo:** `hal/stm32/compat/Preferences.h` — no env `stm32h743` (e em
`native-stm32`), o diretório `compat/` vem em `-I` antes das bibliotecas, e
o `#include <Preferences.h>` dos drivers de sensores, do autotrimmer e das configurações do
registro o encontra. `class Preferences : public KvPreferences` sobre
`Stm32FlashStorage::store()` — a mesma API do NVS do ESP32 ([storage.md](storage.md#kvpreferences)).

## `Stm32SdCard`

**Arquivo:** `hal/stm32/Stm32SdCard.h` · **Herda de:** `IBlockDevice` · **Pinos:** `src/stm32/sd_msp.cpp`

Um cartão SD no SDMMC1: barramento de 4 bits, `HAL_SD` em modo de polling (sem DMA e sem
interrupções) **com controle de fluxo por hardware**: a tarefa de voo preempta a tarefa
de gravação no meio de um bloco, e sem ele o FIFO estourava
(`HAL_SD_ERROR_RX_OVERRUN`, 0x20) — na placa isso aparecia como console e
gravação congelados por segundos. Os pinos PC8..PC11 (D0..D3), PC12 (CK), PD2 (CMD) são o slot µSD da
DevEBox e da WeAct. O núcleo SDMMC recebe o clock do PLL1Q = 48 MHz, `ClockDiv = 1` →
**24 MHz**; se a primeira leitura a 24 MHz falhar, tentam-se 12 e 6.

| Membro | Descrição |
|---|---|
| `bool begin()` | Subir o barramento, identificar o cartão, leitura de teste. `false` — não há cartão; `initError()` — o código |
| `read` / `write` | Em pedaços de no máximo 4 KB (pausas curtas); um endereço não múltiplo de 4 é copiado por um buffer alinhado (o HAL lê o FIFO por palavras). Em caso de falha — uma nova tentativa |
| espera | Antes de um acesso após uma escrita, espera o cartão voltar ao estado de transferência (`Rtos::sleepMs(1)`: as tarefas de segundo plano não ficam famintas), por até 1 s. Após uma leitura, nenhuma consulta de estado extra é enviada — a conferência na inicialização lê dezenas de milhares de setores |
| `blockCount()`, `cardType()`, `clockDivider()`, `lastErrorCode()` | Para a linha de status |
| `readOps`, `writeOps`, `errors`, `retries` | Contadores |

## `ResetCause`

**Arquivo:** `hal/ResetCause.h` · `readResetCause()`, `isCrashReset()`, `resetCauseName()`

A causa da reinicialização, igual nas duas placas. ESP32 — `esp_reset_reason()`;
STM32 — os sinalizadores de `RCC->RSR` (lidos uma vez e zerados; no H7 o `PINRSTF`
é ligado em qualquer reset, por isso as causas mais específicas são verificadas
primeiro: watchdog → ligação → queda de tensão → reset por software).
Um pânico, os watchdogs e uma queda da alimentação contam como “falha”: a caixa-preta
começa a gravar na hora por eles.

## `Rtos`

**Arquivo:** `hal/Rtos.h` · namespace

| Membro | Descrição |
|---|---|
| `PRIORITY_BACKGROUND` (1), `PRIORITY_TELEMETRY` (2), `PRIORITY_FLIGHT` (5) | prioridades das tarefas |
| `bool startTask(fn, name, stackBytes, arg, priority, handle)` | ESP32 — `xTaskCreatePinnedToCore(..., núcleo 0)`, a pilha em bytes; STM32 — `xTaskCreate`, a pilha é convertida em palavras; `handle` é para `xTaskNotifyGive` |
| `void sleepMs(ms)` | `vTaskDelay`; antes de o escalonador iniciar (o `setup()` do STM32) — `delay()` |
| `class CriticalSection` | `enter()`/`exit()`: ESP32 — o spinlock `portMUX`, STM32 — `taskENTER_CRITICAL()`. Dentro, apenas cópia de bytes (a fila da caixa-preta) |
| `uint32_t freeHeapBytes()` | ESP32 — `ESP.getFreeHeap()`; STM32 — `xPortGetFreeHeapSize()` |

## O ponto de entrada `src/stm32/main.cpp`

O firmware completo: os mesmos objetos de `src/main.cpp`, telemetria MAVLink no lugar do
Wi-Fi, tarefas do FreeRTOS no lugar de `loop()` — [application.md](application.md#srcstm32maincpp--stm32h743).
