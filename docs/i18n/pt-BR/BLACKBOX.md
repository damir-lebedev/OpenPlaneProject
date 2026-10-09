# Caixa-preta

> 🌐 Esta página é uma tradução do [original em russo](../../BLACKBOX.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão. A tradução foi feita por uma IA e não foi revisada por falantes nativos. Se encontrar erros, escreva para [Damir Lebedev](https://github.com/damir-lebedev) ou abra uma [issue](https://github.com/damir-lebedev/OpenPlaneProject/issues).

O firmware grava sozinho cada voo na flash integrada da placa: sensores, sticks, saídas para os servos, decisões do piloto automático, eventos. Depois do voo, a gravação é baixada por USB e decodificada em tabelas CSV.

Funciona em duas placas:

| Placa | Onde grava | Quanto cabe (a ~20 KB/s) |
|---|---|---|
| **ESP32-S3 N16R8** | uma partição de 13,9 MB da flash integrada | cerca de **11 minutos** |
| **STM32H743** (DevEBox, a principal; WeAct) | um arquivo em um cartão SD, [abaixo](#cartão-sd-stm32h743) | 64 MB: cerca de **55 minutos**, o tamanho é definido pelo arquivo |

Nas demais placas (ESP32-C3, ESP32 comum) não há mídia de armazenamento: a caixa-preta fica desligada e não interfere no voo.

---

## Quando grava

| | Condição |
|---|---|
| **Início** | Armado **e** com o acelerador levantado (o stick ou o ESC acima de `THROTTLE_LOW_US`). Os **10 s anteriores** também entram na gravação: o momento do ARM e a espera antes da decolagem |
| | Reinicialização causada por uma falha (pânico, watchdog, queda de energia): grava desde o primeiro ciclo e por pelo menos 60 s; se aconteceu no ar, dá para ver o que veio depois |
| | Manualmente pelo console (`k` → `r`): para a bancada |
| **Parada** | **10 s depois do DISARM** |
| | Armado, mas com o motor parado e o avião **imóvel por 30 s**: pousou ou caiu, e o DISARM foi esquecido |
| | Manualmente (`k` → `r`) |
| **Não é parada** | Perda de link, failsafe, motor em zero no ar, planeio, pouso sem DISARM enquanto o avião ainda está rolando |

"Imóvel" significa tudo isto ao mesmo tempo: rotação abaixo de 5 °/s em todos os eixos, o acelerômetro marca 1g ± 0,1, quase não há velocidade vertical pelo barômetro e, pelo GPS e pelo tubo de Pitot (se houver), a velocidade é menor que 2 m/s. Em voo, uma calma dessas por 30 segundos seguidos nunca acontece.

## O que é gravado

| Registro | Frequência | O que contém |
|---|---|---|
| `IMU` | a cada ciclo, 500 Hz | giroscópio (°/s), acelerômetro (g), quanto durou o trabalho do ciclo de controle (µs) |
| `CTRL` | 100 Hz | rolagem/arfagem/rumo, alvos do piloto automático, sticks do piloto, comandos finais, **todas as 7 saídas** (µs), componentes do PID de rolagem e de arfagem (P, I, D), acelerador do piloto e do piloto automático, flaps, modo, flags (ARM, link, failsafe, sensores vivos...), funções ligadas |
| `RC` | 50 Hz | todos os 10 canais do rádio, contadores de quadros iBUS (íntegros e corrompidos) |
| `BARO` | a cada amostra (~50 Hz) | pressão, temperatura, altitude, velocidade vertical, alvo de altitude |
| `MAG` | até 50 Hz | o campo nos três eixos, rumo |
| `GPS` | a cada solução | coordenadas, altitude, velocidade, rumo, satélites, fix, precisão |
| `AIR` | até 50 Hz | tubo de Pitot: diferença de pressão, velocidade indicada e verdadeira, densidade |
| `NAV` | 10 Hz | ponto de origem (distância, marcação), rumo e alvo de rumo, velocidade de navegação, fonte do rumo, estágios do lançamento manual e do voo planado, auto-trim |
| `POWER` | 10 Hz | tensão da bateria e saída do sensor de corrente (os divisores da placa da controladora de voo, [FC_BOARD.md](FC_BOARD.md), bloco B) |
| `SYS` | 1 Hz | frequência e pior ciclo do laço de controle, memória livre, contadores do iBUS, temperatura do IMU, fila da caixa-preta, registros perdidos, a gravação mais longa na flash, espaço livre |
| `EVENT` | por evento | ARM/DISARM, recusa de ARM com o motivo, troca de modo, link perdido/recuperado, sensor com falha/recuperado, fix do GPS, ponto de origem registrado, funções das chaves, geofence, proteção contra estol, estágios do lançamento manual e do voo planado |

No início de cada voo vêm os parâmetros: o firmware (data da compilação), o motivo da partida e da última reinicialização, quais sensores estão instalados e se passaram na verificação pré-voo, os coeficientes do PID (incluindo ajustes feitos pelo painel), os trims, valores importantes do `Config` e as atribuições das chaves.

---

## Como usar

### Antes do voo

Não é preciso fazer nada. Ao ligar, o monitor serial mostra o estado (o console imprime em russo; a linha abaixo diz "espera ARM e acelerador | apagado à frente 12,9 MB (≈11 min) de 13,9 MB | voos 1"):

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

"Apagado à frente" é quanto vai caber no próximo voo. Depois de ligada, a caixa-preta leva alguns segundos (depois de um voo longo, até um minuto) preparando espaço: apaga as gravações antigas. Nesse tempo, o laço de voo no solo às vezes congela por ~0,15 s; as superfícies podem dar um tranco atrasado, e isso é normal. **No ar a flash nunca é apagada.**

### Depois do voo: download

1. Conecte o USB ao conector **COM**. Feche o monitor serial (ele segura a porta).
2. Execute na pasta do projeto:

   ```bash
   python tools/blackbox.py download          # o último voo
   python tools/blackbox.py download --all    # todos
   python tools/blackbox.py list              # o que há na placa
   ```

   É preciso o `pyserial`: `pip install pyserial`. Ou o Python do PlatformIO, que já o tem: `%USERPROFILE%\.platformio\penv\Scripts\python tools\blackbox.py download`.

3. O voo é baixado para a pasta `blackbox/` (~200 KB/s: 10 minutos de voo levam cerca de um minuto) e é decodificado logo ao lado, em uma pasta com o mesmo nome.

Enquanto o download corre, o laço de voo fica parado; por isso ele só funciona sem ARM.

### O que há dentro da pasta do voo

| Arquivo | O que é |
|---|---|
| `summary.txt` | O resumo: duração, frequências, faixas de ângulos, altitudes, velocidades, tensões, o pior ciclo de controle, registros perdidos, todos os eventos |
| `events.txt` | Os parâmetros do voo e todos os eventos por ordem de tempo |
| `IMU.csv`, `CTRL.csv`, `RC.csv`, ... | Uma tabela para cada tipo de registro |

O tempo em todas as tabelas é `time_s`, segundos desde o início da gravação (ARM e acelerador); a pré-gravação vem com sinal de menos. Os valores já estão em unidades: graus, g, metros, m/s, microssegundos de pulso. No `CTRL.csv`, o modo é acrescentado pelo nome (`mode_name`), as funções como lista (`features_on`) e as flags são separadas em colunas 0/1 (`armed`, `rx_lost`, `fs_glide`, `imu_ok`...).

Os CSVs abrem no Excel/LibreOffice, mas, para gráficos em função do tempo, é mais prático o [PlotJuggler](https://github.com/facontidavide/PlotJuggler): File → Load Data → CSV, coluna de tempo `time_s`.

O arquivo `.bbl` é uma imagem bruta da flash e pode ser decodificado de novo: `python tools/blackbox.py decode blackbox/flight_001_....bbl`.

### Console: `k`

No monitor serial, a tecla `k` abre o menu da caixa-preta: estado, lista de voos, `r`: iniciar/parar a gravação manualmente (para verificar na bancada), `e`: apagar todos os voos (com confirmação `y`, ~40 s).

---

## Espaço na flash

- Os voos são gravados em anel. Quando o espaço está acabando, a caixa-preta, no solo, apaga **os voos mais antigos inteiros** até haver 10 MB livres à frente (`BLACKBOX_MIN_FREE_BYTES`, ~9 minutos).
- **O último voo gravado nunca é apagado**: só é sobrescrito pela gravação seguinte, se ela não tiver tido espaço suficiente.
- Se o espaço apagado acabar no ar, a gravação continua em uma fila na PSRAM (4 MB, ~3 minutos dos dados mais recentes); depois do pouso e do DISARM, a caixa-preta libera espaço e a grava. Um voo mais longo que a partição inteira (~11 min) não cabe todo: o começo é guardado e o fim se perde.
- Por isso, **baixe o voo depois de cada saída**, principalmente a primeira.

## Confiabilidade

- Uma queda de energia a qualquer momento (caiu, a bateria soltou): tudo é preservado, exceto os últimos ~15 ms. Os registros incompletos são descartados por CRC; no `summary.txt` é a linha "Недописанных записей (CRC)" (o resumo está em russo; significa "Registros incompletos (CRC)").
- O número do voo, a cabeça do anel e a lista de voos são restaurados a partir dos próprios setores: não existe um "mapa" separado que possa ser corrompido.
- O download verifica o CRC-32 de cada setor e do voo inteiro.

## Efeito sobre o voo

- O laço de voo apenas coloca um instantâneo em uma fila na PSRAM: alguns microssegundos. Uma tarefa separada no núcleo 0 grava na flash, uma página (256 bytes) por vez, **logo depois de um ciclo de controle**: uma gravação na flash para os dois núcleos do ESP32 por 0,6–0,9 ms, e ela cai na pausa entre os ciclos.
- Medição na bancada (uma DevKit sem sensores, duas rodadas de 30–40 s de gravação): o intervalo entre os ciclos é de 2,00 ms, 99,2–99,7% dos intervalos ficam entre 1,9 e 2,1 ms, o mais longo é de 2,5 ms e nenhum ciclo foi perdido; o trabalho do ciclo é o mesmo que sem gravação. O ciclo só treme de forma perceptível no solo, sem ARM, enquanto a caixa-preta verifica e apaga espaço (ler um bloco de 64 KB: uma pausa de ~3 ms; apagar: ~0,15 s).
- Com sensores, um ciclo leva ~0,7 ms, e a gravação de uma página ainda cabe nos 1,3 ms restantes. Verificação depois do primeiro voo: no `summary.txt`, as linhas "Такт IMU" e "Цикл: худший такт" (em russo: "Ciclo do IMU" e "Laço: pior ciclo").

---

## Cartão SD (STM32H743)

Na STM32H743, a caixa-preta grava em um cartão SD (slot µSD no SDMMC1, 4 bits, 24 MHz). O cartão continua sendo um **FAT32** comum: na raiz dele fica um arquivo criado de antemão, `BLACKBOX.BIN`, dentro do qual o firmware grava blocos brutos, sem nunca tocar na tabela FAT nem no diretório. Por isso não há o que corromper se a energia faltar em voo, e o arquivo pode simplesmente ser copiado para um PC.

### Preparação do cartão (uma única vez)

1. Formate o cartão em **FAT32** (não exFAT; o Windows oferece FAT32 para cartões de até 32 GB).
2. Com o cartão em um leitor, no PC:

   ```bash
   python tools/blackbox.py sd-prepare E:              # 64 MB, E: é a unidade do cartão
   python tools/blackbox.py sd-prepare E: --size 256   # ou mais
   ```

   O arquivo é criado em um único pedaço em um cartão vazio e preenchido com `0xFF`; o primeiro setor é uma marca de serviço "o anel está vazio". Se o arquivo não estiver contíguo (o cartão não está vazio e está muito fragmentado) ou não existir, ao ligar o console mostrará o motivo e a caixa-preta ficará desligada.
3. Insira o cartão na placa. Ao ligar (o console imprime em russo: "Cartão SD: 15204 MB, SDMMC 24 MHz, 4 bits; arquivo BLACKBOX.BIN: ok", depois a linha de estado e depois "pronto em 300 ms"):

   ```
   SD-карта: 15204 МБ, SDMMC 24 МГц, 4 бита; файл BLACKBOX.BIN: ок
   BlackBox: ждёт ARM и газ | стёрто впереди 0.7 МБ из 64.0 МБ | полётов 0
   BlackBox: готов за 300 мс
   ```

   "Apagado à frente" cresce em segundo plano: a placa verifica o espaço a ~2,5 MB/s.

### Pegar o voo

- **Pela placa, via USB**, como na ESP32: `python tools/blackbox.py download` (o console da STM32 é USB CDC, a velocidade do download é de ~400 KB/s, 1 MB leva menos de 3 s). `list`, `--all` e `--flight N` funcionam do mesmo jeito.
- **Tirando o cartão**: o arquivo `BLACKBOX.BIN` do cartão é decodificado direto em CSV:

  ```bash
  python tools/blackbox.py ring E:/BLACKBOX.BIN              # todos os voos -> blackbox/
  python tools/blackbox.py ring E:/BLACKBOX.BIN --list       # só listar
  ```

  O arquivo é um anel de setores: o utilitário monta os voos sozinho pelos números dos setores, inclusive os que passaram do fim do arquivo.

### O que foi medido na placa

DevEBox H743 + cartão de 16 GB (o teste `test_blackbox_sd`, [TESTING.md](TESTING.md#testes-na-placa-stm32)):

| | |
|---|---|
| Reconhecimento do cartão | 12–18 ms, 4 bits, 24 MHz |
| Gravação de uma página de 256 B | em média 2,3–3,7 ms, **a pior 60–190 ms**, ~75–110 KB/s sustentados (são necessários ~20 KB/s) |
| Leitura | um setor de 4 KB: 4,2 MB/s; um bloco aleatório: 0,6 ms |
| Apagamento | 64 KB: 13 ms; toda a área de 64 MB: 20–28 s |
| Ao ligar | com o anel vazio: 0 ms (pela marca); com voos: 0,3 s (verificação por amostragem de ~530 leituras); uma verificação completa de 64 MB levaria ~20 s |
| 20 s de gravação em tempo real (IMU a 500 Hz) | nenhum registro perdido, 0 erros |
| A tarefa de voo durante a gravação | desvio do período de 2 ms: **1 µs** (uma tarefa simuladora de prioridade máxima ao lado da gravação) |

A pior gravação de página é a "faxina" interna do cartão; a fila na RAM (384 KB ≈ 19 s de fluxo) aguenta essas pausas. Os cartões baratos são os que mais diferem nisso: antes de voar, vale testar o cartão com o teste `test_blackbox_sd` (a pior gravação deve ser menor que 250 ms, o limite da especificação SD).

### Em que difere da flash da ESP32

- **A tarefa de gravação** (`bbox`, prioridade 2) é preemptada pela de voo (5) no meio de um acesso ao cartão, e não "na pausa do ciclo", como na ESP32 com a sua parada dos núcleos. A transferência usa o controle de fluxo por hardware do SDMMC: sem ele, a FIFO transbordava na preempção (na placa eram `HAL_SD_ERROR_RX_OVERRUN` e o console e a gravação travados por segundos).
- **A fila fica na RAM**, de 384 KB (`BLACKBOX_RING_STM32_BYTES`), e não em 4 MB de PSRAM.
- **A verificação ao ligar é por amostragem**: os setores reais do anel formam um arco contínuo, são lidos ~500 cabeçalhos, e os limites do arco e dos voos são refinados por bisseção. O resultado é o mesmo da verificação completa; se o quadro não fechar, faz-se a completa.
- **A marca "anel vazio"** no primeiro setor do arquivo: para não verificar a área vazia por segundos a cada vez que se liga. Ela é colocada ao apagar tudo e quando uma verificação completa não achou nada; é removida antes da primeira gravação.
- **Os erros do cartão** (retirado, falha no barramento) vão para o log uma vez por segundo como o evento "носитель: ошибок записи …" (em russo: "mídia: erros de gravação …"); uma página perdida deixa um buraco `0xFF`, e a decodificação do setor para nele (como em `tools/blackbox.py`); os demais setores ficam intactos.

## Configurações (`include/config/Config.h`, seção "Caixa-preta")

| Constante | Padrão | Significado |
|---|---|---|
| `BLACKBOX_RING_BYTES` | 4 MB | A fila na PSRAM (sem PSRAM: `BLACKBOX_RING_NO_PSRAM_BYTES`, 32 KB) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32: a fila na RAM |
| `BLACKBOX_SD_FILE`, `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN`, 256 MB | STM32: o arquivo no cartão e o teto da parte usada |
| `BLACKBOX_PREROLL_MS` | 10 000 | Quanto gravar antes do início |
| `BLACKBOX_POSTROLL_MS` | 10 000 | Quanto gravar depois do DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Parado e com ARM: parada |
| `BLACKBOX_LANDED_GYRO_DPS`, `_ACCEL_G`, `_CLIMB_MS`, `_SPEED_MS` | 5, 0,1, 0,5, 2 | O que conta como "imóvel" |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | Gravação depois de uma reinicialização por falha: não menos que isso |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | Quanto manter apagado para o próximo voo |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | A pausa entre apagamentos no solo |
| `BLACKBOX_IMU_DIVIDER` | 1 | O IMU a cada N ciclos: 2 equivale a 250 Hz e a uma gravação ~25% mais longa |
| `BLACKBOX_VBAT_DIVIDER`, `_CURRENT_DIVIDER` | 6,6, 1,667 | Os divisores da bateria e do sensor de corrente na placa |

A tabela de partições é `partitions_blackbox.csv`: o aplicativo, 2 MB (o firmware ocupa agora ~0,9 MB), a caixa-preta, 13,9 MB, e o coredump, 64 KB. A partição NVS ficou no mesmo lugar: as calibrações do IMU e da bússola e os trims são preservados depois da mudança para esta tabela. Não há um segundo slot para atualizações pelo ar (OTA).

---

## Para o desenvolvedor

O código está em `include/telemetry/BlackBox*.h`:

| Arquivo | O quê |
|---|---|
| `BlackBoxFormat.h` | O formato: cabeçalho do setor, tipos e estruturas dos registros, esquemas dos campos, CRC-8/CRC-32 |
| `BlackBoxStorage.h` | Um anel de setores sobre o `IFlashRegion`: busca da cabeça ao ligar, lista de voos, gravação por páginas, apagamento dos voos antigos em etapas |
| `BlackBoxRing.h` | Uma fila de registros entre os núcleos (spinlock), que descarta o mais antigo |
| `BlackBox.h` | Instantâneos no ciclo, início/parada, eventos, a tarefa de gravação, download por UART |
| `hal/esp32/Esp32FlashPartition.h` | `IFlashRegion` sobre o `esp_partition` |
| `hal/SdFileRegion.h`, `storage/Fat32File.h` | `IFlashRegion` sobre um arquivo em um cartão FAT32: busca do arquivo (a FAT é só leitura), blocos incompletos, apagamento com `0xFF`, a marca "anel vazio" |
| `hal/stm32/Stm32SdCard.h`, `src/stm32/sd_msp.cpp` | `IBlockDevice`: SDMMC1 sobre o `HAL_SD` (polling, 4 bits, controle de fluxo por hardware) e os pinos |
| `hal/ResetCause.h` | A causa da reinicialização no ESP32 e na STM32 (`RCC->RSR`) |

### Formato na flash

Um setor de 4 KB = um cabeçalho de 16 bytes (`magic "OPBB"`, um `seq` corrido, o `millis()` no momento da abertura, o número do voo, a versão do formato, um byte de controle) + os registros em sequência. Um registro não atravessa o limite do setor; o fim do setor é `0xFF`.

Um registro: `[tipo u8][comprimento u8][dados][CRC-8]`; os dados começam com `t_us` (`micros()`). Os primeiros registros de um voo são `SCHEMA`: o texto `"16 IMU t_us:I gx:h/10 ..."`: o nome do campo, o caractere do `struct` do Python e o divisor. O decodificador pega os campos do log, então um campo novo em um registro significa editar a estrutura e a string do esquema em `BlackBoxFormat.h` (os tamanhos delas são conferidos por `static_assert`); não é preciso mexer no decodificador.

### Protocolo de download

Os comandos são uma linha depois do byte STX (`0x02`), e o console a repassa à caixa-preta:

```
PC:  \x02bb list\n
FC:  BB:STATE state=idle free_kb=... total_kb=... flights=... rate_bps=...
     BB:FLIGHT n=3 sectors=234 kb=936 seconds=41 start=1
     BB:END
PC:  \x02bb get 3 2000000\n
FC:  BB:SEND n=3 sectors=234 baud=2000000   (a 115200), depois passa para 2 Mbaud
PC:  passa para 2 Mbaud, envia 'G'
FC:  234 quadros: A5 5A, u16 número, 4096 bytes do setor, u32 CRC-32
     volta para 115200, BB:DONE n=3 crc=<CRC-32 de todos os setores>
```

### Testes

`pio test -e native -f native/test_blackbox_scan`: uma verificação do anel por amostragem contra uma completa, em históricos aleatórios (300 anéis × 5 passos de sondagem), e o custo em uma área de SD. `pio test -e native-stm32 -f native_stm32/test_blackbox_sd` e `test_app_stm32_*`: FAT32, a área, o driver do cartão, a caixa-preta no cartão, o firmware da STM32 inteiro. Na placa: `pio test -e stm32h743-devebox -f test_blackbox_sd` (um cartão de verdade).

`pio test -e native -f native/test_blackbox`: o formato, o anel sobre um NOR simulado da partição (apagamento por setores, uma gravação só abaixa bits; subir um bit conta como erro), reinicializações, quedas de energia, gravação de um voo com os `FlightController`/`Autopilot` reais, início/parada, eventos, estouro da flash no ar, download. Uma imagem de voo para verificar o decodificador: `OPENPLANE_BLACKBOX_DUMP=/tmp/f.bbl pio test -e native -f native/test_blackbox`.
