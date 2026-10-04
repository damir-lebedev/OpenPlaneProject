# Referência do piloto automático do OpenPlane

> 🌐 Esta página é uma tradução do [original em russo](../../AUTOPILOT_GUIDE.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão.

O que o piloto automático sabe fazer, como ativar cada função e como atribuí-la a qualquer chave ou potenciômetro do rádio **com uma única linha**.

> Primeiro, com honestidade. Todos os modos foram verificados com testes unitários e simulações de voo em malha fechada (`test/native/test_sim`: o firmware inteiro pilota um modelo de avião). O modelo é simplificado, e os coeficientes em `Config.h` são valores iniciais: **cada modo deve ser experimentado primeiro a uma altitude de mais de 50 m, com o dedo na chave MANUAL**. Até agora só o modo manual voou (o primeiro protótipo, com uma ESP32-C3); a estabilização foi verificada na bancada: as superfícies respondem às inclinações no sentido certo. A placa STM32H743 compila e passa nos mesmos testes; no hardware, a placa DevEBox foi verificada sem sensores: o cartão SD, a caixa-preta e o controle manual dos servos e do motor pelo rádio (gravado em vídeo); ainda não foram conectados sensores a ela, e os modos do piloto automático não foram testados nela.

---

## Sumário

1. [Como funciona, em um minuto](#como-funciona-em-um-minuto)
2. [Disposição padrão do rádio](#disposição-padrão-do-rádio)
3. [Atribuir uma função com uma única linha](#atribuir-uma-função-com-uma-única-linha)
4. [Modos](#modos)
5. [Funções (chaves)](#funções-chaves)
6. [Potenciômetros](#potenciômetros)
7. [Perda de link, geofence, ponto de origem](#perda-de-link-geofence-ponto-de-origem)
8. [Tubo de Pitot caseiro](#tubo-de-pitot-caseiro)
9. [Estação de solo: painel Wi-Fi e MAVLink](#estação-de-solo-painel-wi-fi-e-mavlink)
10. [Ordem de configuração de um avião novo](#ordem-de-configuração-de-um-avião-novo)
11. [Verificação pré-voo do piloto automático](#verificação-pré-voo-do-piloto-automático)
12. [O que cada modo precisa](#o-que-cada-modo-precisa)

---

## Como funciona, em um minuto

```
sticks ─┐
        ├─► PilotSwitches (config/Controls.h) ─► modo, funções, potenciômetros
chaves ─┘                                                   │
                                                            ▼
sensores (IMU, barômetro, bússola, GPS, tubo de Pitot) ─► Autopilot ─► comandos das superfícies e do acelerador
                                                            │
                               FlightController: flaps, carga, câmera, buzzer, failsafe
                                                            ▼
                                               ailerons · profundor · leme · ESC · AUX1 · AUX2
```

- O **modo** decide quem pilota: o piloto (MANUAL), o piloto com um ajudante (STABILIZE, ALT_HOLD, ACRO), o piloto automático com correções do piloto (CRUISE, LOITER, RTH…).
- As **funções** são ligadas por cima de qualquer modo: flaps, freio, lançamento de carga, geofence…
- Os **potenciômetros** mudam um número suavemente: a intensidade da estabilização, a velocidade de cruzeiro, o raio dos círculos…
- Nos modos com estabilização, **o stick define o ângulo**, e não a deflexão da superfície: soltou o stick, o avião volta sozinho ao horizonte.
- A falha de um sensor nunca "sacode" o avião: sem IMU, as superfícies ficam com o piloto; sem barômetro, a altitude fica com o piloto; sem GPS, não há navegação, e os modos que precisam dele se comportam com segurança (veja a [tabela](#o-que-cada-modo-precisa)).

---

## Disposição padrão do rádio

FS-i6 + FS-iA6B, iBUS, 10 canais (`config/Channels.h`).

| Canal | Controle do rádio | Padrão |
|---|---|---|
| CH1–CH4 | sticks | rolagem, arfagem, acelerador, leme (não podem ser reatribuídos) |
| CH5 | **SwA** | **ARM** (para baixo = armado, só com o acelerador embaixo; não pode ser reatribuído) |
| CH6 | SwB | flaps (`Feature::FLAPS`) |
| CH7 | **SwC** (3 posições) | em cima **MANUAL** · no meio **STABILIZE** · embaixo **AUTO_TAKEOFF** |
| CH8 | SwD | **RTH** — volta para casa, enquanto estiver ligada |
| CH9 | VrA | intensidade da estabilização (`Knob::STAB_GAIN`) |
| CH10 | VrB | velocidade de cruzeiro (`Knob::CRUISE_SPEED`) |

Ao ligar a placa, o monitor serial imprime a disposição real, ou seja, o que de fato foi gravado nela (o firmware imprime em russo; "вверх / середина / вниз" significa em cima / no meio / embaixo):

```
SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (вверх / середина / вниз)
SwB (CH6): FLAPS, пока включён
SwD (CH8): RTH, пока включён
VrA (CH9): крутилка STAB_GAIN
VrB (CH10): крутилка CRUISE_SPEED
```

> Os canais 7–10 no FS-i6 não vêm habilitados por padrão. Menu do rádio: **Functions setup → Aux. channels**; atribua SwC, SwD, VrA e VrB.

---

## Atribuir uma função com uma única linha

Tudo em um único arquivo: `include/config/Controls.h`:

```cpp
constexpr Binding BINDINGS[] = {
    Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_AUTO_TAKEOFF),
    Bind::feature(Channels::SWB, Feature::FLAPS),
    Bind::mode   (Channels::SWD, MODE_RTH),
    Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
    Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
};
```

| Forma da linha | O que faz |
|---|---|
| `Bind::modes(canal, cima, meio, baixo)` | uma chave de três posições escolhe o modo |
| `Bind::modes(canal, cima, baixo)` | de duas posições: dois modos |
| `Bind::mode(canal, modo)` | o modo **por cima** dos demais enquanto a chave estiver ligada; ao desligar, volta o modo da chave de modos |
| `Bind::feature(canal, função)` | a função age enquanto a chave estiver ligada |
| `Bind::knob(canal, potenciômetro)` | potenciômetro: centro = valor de `Config.h`, extremos = mínimo e máximo |

"Ligada" significa que o canal está acima de 1750 µs (no FS-i6, a chave para baixo, em sua direção). Até o primeiro quadro do receptor chegar, todos os canais são considerados desligados: ao dar partida, nada será liberado nem solto.

### Receitas prontas

```cpp
// Planador de térmicas: SwD — voo planado, SwB — auto-trim, VrB — raio dos círculos
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_SOARING),
Bind::feature(Channels::SWB, Feature::AUTO_TRIM),
Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Aluno: só estabilização, RESCUE no "botão de pânico", sticks suaves
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_ALT_HOLD, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_RESCUE),
Bind::feature(Channels::SWB, Feature::GEOFENCE),
Bind::knob   (Channels::VRA, Knob::RATES),
Bind::knob   (Channels::VRB, Knob::MAX_BANK),

// Filmagem e entrega: câmera com estabilização, lançamento de carga, círculos sobre um ponto
Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_CRUISE, MODE_LOITER),
Bind::feature(Channels::SWB, Feature::PAYLOAD_DROP),
Bind::feature(Channels::SWD, Feature::CAMERA_STAB),
Bind::knob   (Channels::VRA, Knob::CAMERA_TILT),
Bind::knob   (Channels::VRB, Knob::LOITER_RADIUS),

// Lançamento manual sem trem de pouso: SwD — LAUNCH, flaps suaves no potenciômetro
Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_CRUISE),
Bind::mode   (Channels::SWD, MODE_LAUNCH),
Bind::feature(Channels::SWB, Feature::AIRBRAKE),
Bind::knob   (Channels::VRA, Knob::FLAPS),
Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),
```

### O compilador pega os erros

A tabela é verificada na compilação (`static_assert`), antes de o firmware chegar ao avião:

- os sticks e o SwA (ARM) não podem ser atribuídos, e o número do canal deve ser menor que `Channels::COUNT`;
- um canal tem uma única atribuição;
- a chave de seleção de modos (`Bind::modes`) não pode ser mais de uma.

Se várias chaves `Bind::mode` estiverem ligadas ao mesmo tempo, vence a linha de cima da tabela.

---

## Modos

O modo muda quando uma chave é **acionada**. Um modo ligado pelo painel ou pela estação de solo vale até o piloto acionar de novo uma chave de modo. No OLED aparece um nome curto (entre parênteses).

### MANUAL (MAN)
Superfícies = sticks, como se não houvesse controladora de voo. Só funcionam as funções (flaps, freio, carga) e o auto-trim. **O principal modo de segurança**: mantenha-o sempre em uma chave sob o dedo.

### STABILIZE (STAB) — "o stick define o ângulo"
O stick de rolagem define o ângulo de rolagem até `MAX_BANK_DEG` (45°, potenciômetro `MAX_BANK` de 15…60°), e o stick de arfagem define o ângulo até `STAB_MAX_PITCH_DEG` (25°). Soltou, e o avião se nivela sozinho. O acelerador fica com o piloto. O integrador do PID só acumula perto do alvo (±10°); por isso, depois de uma manobra brusca, o avião não "passa direto" pelo horizonte.
**Precisa de:** IMU. Sem IMU, funciona como MANUAL.

### ALT_HOLD (ALT) — "mantenha a altitude"
Como o STABILIZE na rolagem, e o profundor mantém a altitude pelo barômetro. Mexeu o stick de arfagem, você pilota; soltou, e o avião mantém a **nova** altitude. O acelerador fica com o piloto (aumente o acelerador, senão a velocidade não basta para subir).
**Precisa de:** IMU + barômetro.

### ACRO — "o stick define a taxa de rotação"
Stick todo: 180°/s. Soltou, e o avião **mantém a atitude em que estava** (mesmo de cabeça para baixo), e o giroscópio amortece as rajadas. Para acrobacias.
**Precisa de:** IMU. Sem IMU, superfícies = sticks.

### CRUISE (CRZ) — "mantenha rumo, altitude e velocidade"
O avião voa em linha reta na altitude atual, com o acelerador automático (potenciômetro `CRUISE_SPEED`: 30…55…85 % de acelerador e, com tubo de Pitot, **velocidade do ar** de 10…14…22 m/s). O stick de rolagem faz a curva (soltou, e ele mantém o novo rumo), e o de arfagem muda a altitude. Com tubo de Pitot, funciona a proteção contra estol. O rumo vem do GPS (com velocidade em relação ao solo > 3 m/s; volta para a bússola abaixo de 2 m/s); senão, da bússola; senão, do giroscópio. Na simulação, com vento de través de 4 m/s, a trajetória sobre o solo se mantém dentro de ±5° e a altitude, dentro de ±3 m.
**Precisa de:** IMU + barômetro; GPS ou bússola para um rumo sem deriva.

### LOITER (LOIT) — "orbite aqui"
Círculos no sentido horário sobre o ponto onde o modo foi ligado, com raio de 50 m (potenciômetro `LOITER_RADIUS` de 25…150 m), na altitude atual, com o acelerador automático. A orientação é um campo vetorial: de longe, o avião entra no círculo pela tangente e, no círculo, mantém uma inclinação antecipada. Sem GPS, é só um círculo com inclinação constante no mesmo lugar.
**Precisa de:** IMU + barômetro + GPS.

### RTH — "para casa"
Rumo ao ponto de origem (o ponto do ARM), altitude `RTH_ALTITUDE_M` = 40 m (se estiver abaixo, sobe pelo caminho; se estiver acima, permanece). Sobre o ponto de origem, círculos com o raio do LOITER até o piloto retomar o controle. Sem GPS ou sem ponto de origem, círculos no mesmo lugar. O mesmo modo é acionado pela perda de link e pela geofence.
**Precisa de:** IMU + barômetro + GPS com ponto de origem.

### AUTO_TAKEOFF (TKOFF) — "decolagem no acelerador"
Depois do ARM, nada acontece até o piloto levantar o acelerador acima do meio. Daí o programa: 1 s de aceleração até 100 % de acelerador com as asas niveladas, 2 s de rotação com arfagem de 15° e depois subida com arfagem de 10° até o piloto trocar de modo. Os sticks se somam ao programa: dá para corrigir o rumo durante a corrida de decolagem.
**Precisa de:** IMU.

### LAUNCH (LNCH) — "lançamento manual"
1. Armado e com o acelerador acima do meio, o lançamento fica **preparado**, com o motor parado.
2. O arremesso: sobrecarga para a frente > 1,5 g por mais de 40 ms.
3. Após 0,3 s (a mão já saiu de perto da hélice): acelerador a 100 %, subida com arfagem de 15° e asas niveladas, por 6 s ou até 30 m.
4. Depois, como o CRUISE na altitude alcançada.

Qualquer movimento do stick (> 150 µs) antes do arremesso cancela o lançamento: o avião fica nas mãos do piloto.
**Precisa de:** IMU (acelerômetro). Na simulação: um arremesso a 9 m/s da altura da mão; o avião nunca toca o chão e, em 15 s, ganha mais de 15 m.

### AUTO_LAND (LAND) — "pouso"
Motor desligado, planeio no rumo com arfagem de −4°; abaixo de 3 m pelo barômetro, arredondamento (+4°). O stick de rolagem corrige o rumo da aproximação. Ative em um trecho reto, contra o vento, a 20–40 m, com pista de sobra. Na simulação, o toque ocorre com velocidade vertical menor que 1,5 m/s, asas niveladas e sem bicar o nariz.
**Precisa de:** IMU + barômetro (zerado no solo ao ligar).

### SOARING (SOAR) — "voo planado em térmicas"
Motor desligado, planeio. Um variômetro (com tubo de Pitot, de energia total, sem "térmicas" falsas causadas por puxar o stick) acima de 0,5 m/s por mais de 1,5 s indica uma térmica: círculos com inclinação de 25°. Se a subida média em 8 s cair abaixo de −0,2 m/s, sai da térmica. Abaixo de 30 m, motor até 100 m; a mais de 400 m do ponto de origem, planeia de volta. Na simulação, encontra uma térmica (núcleo de 3 m/s) e ganha mais de 50 m sem motor.
**Precisa de:** IMU + barômetro; GPS, para voltar ao ponto de origem.

### RESCUE (RESQ) — "socorro"
Asas niveladas, nariz a +8°, acelerador a 70 %: sai de qualquer espiral. Perdeu a orientação? Acione a chave e respire. Na simulação, a partir de uma espiral com inclinação de 70° e nariz a −40°, em 4 s: asas niveladas e subida.
**Precisa de:** IMU.

---

## Funções (chaves)

| Função | O que faz | Detalhes e valores (`Config.h`) |
|---|---|---|
| `FLAPS` | os dois ailerons para baixo: flaperons | `FLAPS_DEPLOYED_US` = 220 µs, de forma suave em 1 s; a rolagem age por cima |
| `AIRBRAKE` | os dois ailerons para cima: freio aerodinâmico, rampa de planeio mais íngreme | `AIRBRAKE_US` = 250; tem prioridade sobre os flaps |
| `AUTO_TRIM` | aprende a manter o avião reto sem os sticks: o comando constante às superfícies em voo nivelado "escorre" para o trim | 20 %/s, até ±120 µs; é salvo na flash após o DISARM **no solo** |
| `TURN_COORDINATION` | leme para dentro da curva, nariz para cima na inclinação | sempre ligada nos modos de navegação |
| `MOTOR_KILL` | motor desligado em qualquer modo, até nos automáticos | mais forte que qualquer modo e que o acelerador |
| `BEEPER` | buzzer "estou aqui" | sem chave, apita sozinho: no solo, com o link perdido > 10 s |
| `PAYLOAD_DROP` | servo AUX1 aberto enquanto a chave estiver ligada | 1000 µs fechado, 2000 aberto |
| `GEOFENCE` | a mais de 500 m do ponto de origem ou acima de 120 m: RTH | `GEOFENCE_ALWAYS_ON`: sem chave |
| `HOME_RESET` | ponto de origem = ponto atual (no momento em que a chave é ligada) | só com GPS bom |
| `CAMERA_STAB` | a câmera no AUX2 mantém o ângulo em relação ao horizonte | subtrai-se a arfagem do avião |

## Potenciômetros

O centro do potenciômetro = valor padrão de `Config.h`; os extremos são o mínimo e o máximo. Se não estiver atribuído, vale o valor padrão.

| Potenciômetro | Mínimo … centro … máximo | Onde atua |
|---|---|---|
| `STAB_GAIN` | ×0,25 … ×1 … ×2 | todos os modos com estabilização e o ACRO: "mais macio/mais duro" |
| `MAX_BANK` | 15° … 45° … 60° | inclinação máxima pelo stick e pela navegação |
| `CRUISE_SPEED` | acelerador 30 … 55 … 85 % (com Pitot: 10 … 14 … 22 m/s) | CRUISE, LOITER, RTH, o motor no SOARING |
| `FLAPS` | 0 … 50 … 100 % de flaps | flaps graduais no lugar da chave |
| `CAMERA_TILT` | −90° … 0° … +30° | ângulo da câmera (AUX2) |
| `RATES` | 30 … 65 … 100 % do curso dos sticks | todos os modos: sensibilidade dos sticks |
| `LOITER_RADIUS` | 25 … 50 … 150 m | LOITER e círculos sobre o ponto de origem |

> Um truque útil: o potenciômetro `STAB_GAIN` no VrA é um ajuste "ao vivo" dos coeficientes em voo. Se oscilar, reduza; se estiver mole, aumente; depois leve o multiplicador para o `Config.h`.

---

## Perda de link, geofence, ponto de origem

**O ponto de origem** é registrado no ARM se o GPS estiver bom (fix 3D, ≥ 6 satélites, precisão ≤ 5 m). Se no ARM o GPS ainda não tiver fixado, o ponto é registrado assim que ele fixar. Para mudá-lo em campo, use a função `HOME_RESET`.

**Perda de link** (sem quadros iBUS por > 0,5 s, ou o receptor enviou um acelerador abaixo de 950 µs: é o failsafe configurado no rádio; veja `docs/PILOT_GUIDE.md`):

| Situação | O que a aeronave faz |
|---|---|
| no solo (desarmada) | motor 0, superfícies em neutro; após 10 s, o buzzer |
| no ar, com GPS e ponto de origem | **RTH** com motor; sobre o ponto de origem, círculos a 40 m |
| no ar, sem GPS | **planeio**: motor desligado, asas niveladas, nariz −3° |
| o link voltou | na hora, o modo da chave do piloto |

Um retorno já iniciado não cai em planeio por uma perda curta de GPS. `FAILSAFE_RTH = false`: só planeio.

**Geofence** (`GEOFENCE` ou `GEOFENCE_ALWAYS_ON`): sair a mais de `FENCE_RADIUS_M` (500 m) ou acima de `FENCE_ALTITUDE_M` (120 m) aciona o RTH. Para retomar o controle, é preciso colocar a chave de modo em qualquer outra posição (o modo é ligado pela mudança de posição). Ela volta a atuar quando o avião retorna para dentro com uma folga de 10 %.

---

## Tubo de Pitot caseiro

Velocidade do ar sem um sensor de pressão diferencial comprado: **dois barômetros**.

```
       fluxo de ar incidente ─►  ┌──────────── tubo (PVC/latão, Ø4–6 mm) ──┐
                                 │  BMP581 (I2C 0x47) — pressão total      │  vedado
                                 └─────────────────────────────────────────┘
   fuselagem: barômetro principal (BMP581 0x46 / SPL06 / BMP388) — pressão estática

   velocidade  V = √(2·(P_tubo − P_estática − zero) / ρ),   ρ — da pressão estática e da temperatura
```

**Montagem.** O BMP581 (módulo com endereço 0x47: pino SDO no VCC) é colado dentro de um tubo aberto só para a frente: a placa fica dentro de uma cavidade vedada, com os fios saindo por selante. O tubo aponta para a frente, fora da esteira da hélice (na asa ou acima do nariz). O segundo barômetro fica dentro da fuselagem, protegido do fluxo direto (espuma).

**Ativação no firmware**: `sensors/SensorSelection.h`: `SENSOR_KIT_LSM6DSV_PITOT` ou `SENSOR_KIT_ICM45686_PITOT` (kits prontos), ou `SENSOR_AIRSPEED = SENSOR_AIRSPEED_PITOT_BMP581` em um kit próprio.

**Zero.** Dois barômetros sempre divergem um pouco: a precisão absoluta de cada um é de dezenas de pascals, e essa é toda a diferença de pressão em baixa velocidade (10 m/s ≈ 60 Pa). No primeiro segundo depois de ligar, o firmware tira a média da diferença e a toma como zero. **O avião fica parado ao ligar, e o tubo fica tampado com o dedo ou uma tampa, ou virado contra o vento.** No OLED, no painel e na telemetria, a velocidade aparece depois do zeramento.

**Calibração do `PITOT_SCALE`.** A pressão dentro da fuselagem não é estritamente estática. Com ar calmo, voe uma reta de ida e volta no CRUISE e compare a velocidade média em relação ao solo do GPS com a velocidade do tubo: `PITOT_SCALE = V_GPS / V_tubo`.

**Proteção.** Se a diferença for fortemente negativa por mais de 2 s (mangueiras trocadas, água) ou as leituras do tubo tiverem mais de 0,2 s, a velocidade não é fornecida: o piloto automático passa ao acelerador do potenciômetro e ao rumo sem ela. Verificado em simulação em malha fechada com ruído nos dois barômetros: o erro de velocidade em voo é < 0,5 m/s.

O que o tubo oferece: o CRUISE mantém a **velocidade do ar** e não o acelerador; proteção contra estol; variômetro de energia total para o SOARING; uma velocidade honesta na telemetria.

---

## Estação de solo: painel Wi-Fi e MAVLink

**ESP32: painel Wi-Fi.** Ponto de acesso `OpenPlane-Debug`, senha `12345678`, o endereço aparece no monitor serial. Canais, saídas, todos os sensores, o modo, a navegação e as funções ligadas; é possível trocar o modo e o PID. Detalhes em `docs/PILOT_GUIDE.md`.

**STM32H743: MAVLink por rádio modem** (UART4: PD0 RX, PD1 TX, 57600 baud, o padrão do SiK). Servem o SiK de 433/868/915 MHz, o ELRS em modo MAVLink e um ESP-01 como ponte Wi-Fi. O **QGroundControl** e o **Mission Planner** enxergam a aeronave como um avião ArduPilot:

- horizonte, mapa com o ponto de origem, velocidade (pelo tubo de Pitot, se houver), altitude, variômetro, acelerador;
- os modos, com os nomes do ArduPlane: STABILIZE → FBWA, ALT_HOLD → FBWB, CRUISE → CRUISE, LOITER → LOITER, RTH → RTL, AUTO_TAKEOFF/LAUNCH → TAKEOFF, SOARING → THERMAL, RESCUE → STABILIZE, AUTO_LAND → AUTO; o failsafe aparece como RTL ou CIRCLE;
- faixa de mensagens: ARM/DISARM, troca de modo (com o nosso nome), perda de link, geofence;
- **troca de modo a partir do solo**, com o botão de modo da GCS (exceto AUTO: o OpenPlane não tem missões);
- **parâmetros** `RLL_KP … PTCH_KD`: o PID de rolagem e de arfagem, que são lidos e alterados na janela de parâmetros da GCS em pleno voo (não são salvos após uma reinicialização: leve os valores bons para o `Config.h`).

O ARM/DISARM a partir do solo é **recusado**: só pela chave do rádio. Para verificar o fluxo sem hardware: `OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink && python3 tools/check_mavlink.py /tmp/tlm.bin` (é preciso `pip install pymavlink`).

---

## Ordem de configuração de um avião novo

1. **MANUAL no solo:** sentidos das superfícies (`*_REVERSED` em `Config.h`), flaps para baixo, AUX. Verificação das saídas com `p` no console (sem a hélice!).
2. **Sensores no solo:** `b`: varredura dos barramentos; `s`: status; `o`: calibração da instalação do IMU (3 posições, uma única vez); `m`: bússola.
3. **STABILIZE no solo:** incline o avião para a direita: o aileron direito deve ir **para baixo** (para nivelar). Nariz para cima: profundor para baixo. Se não for assim, as reversões ou a instalação do IMU estão erradas.
4. **Primeiro voo em MANUAL**, subida a mais de 50 m → STABILIZE. Se oscilar, reduza o `STAB_GAIN`; se estiver mole, aumente.
5. **AUTO_TRIM** em voo nivelado por 20–30 s, pouso, DISARM: o trim será salvo.
6. **ALT_HOLD** e depois **CRUISE**: verifique a altitude e o rumo; com tubo de Pitot, calibre o `PITOT_SCALE`.
7. **LOITER** e **RTH**: em altitude, dentro do alcance visual, com o dedo no MANUAL.
8. Só depois disso, o **teste de failsafe** (desligue o rádio em altitude; o avião deve voltar para casa) e a decolagem e o pouso automáticos.

## Verificação pré-voo do piloto automático

- [ ] A disposição impressa ao ligar é a que você espera.
- [ ] Console/OLED: IMU ok, a verificação pré-voo do IMU passou (o avião estava parado ao ligar).
- [ ] O barômetro está zerado no solo (altitude ~0 no OLED).
- [ ] Com tubo de Pitot: velocidade ~0 no solo; se soprar no tubo, ela sobe.
- [ ] GPS: fix 3D, ≥ 6 satélites **antes do ARM**; senão, não haverá ponto de origem nem RTH.
- [ ] STABILIZE no solo: os ailerons e o profundor nivelam o avião, e não o viram.
- [ ] O failsafe está configurado no rádio (acelerador abaixo de 950 na perda de link) e foi verificado desligando o rádio **no solo**, sem hélice.
- [ ] MANUAL, sob o dedo.

---

## O que cada modo precisa

| Modo | IMU | Barômetro | GPS | Bússola | Pitot | Acelerador | Sem o sensor necessário |
|---|:-:|:-:|:-:|:-:|:-:|---|---|
| MANUAL | | | | | | piloto | — |
| STABILIZE | ● | | | | | piloto | superfícies = sticks |
| ALT_HOLD | ● | ● | | | | piloto | o piloto mantém a altitude |
| ACRO | ● | | | | | piloto | superfícies = sticks |
| CRUISE | ● | ● | ○ | ○ | ○ | auto | rumo pelo giroscópio (deriva), altitude com o piloto |
| LOITER | ● | ● | ● | | ○ | auto | círculo com inclinação no mesmo lugar |
| RTH | ● | ● | ● | | ○ | auto | círculos no mesmo lugar |
| AUTO_TAKEOFF | ● | | | | | programa | superfícies = sticks + acelerador do piloto |
| LAUNCH | ● | ○ | | | | programa | superfícies em neutro |
| AUTO_LAND | ● | ● | | ○ | | 0 | sem arredondamento |
| SOARING | ● | ● | ○ | | ○ | 0 / motor | sem térmicas: planeio |
| RESCUE | ● | | | | | 70 % | superfícies em neutro |

● significa obrigatório e ○, que melhora. As verificações no código: `Autopilot.h` (`imuReady`, `baroReady`, `nav.gpsGood`); os testes: `test/native/test_autopilot_modes` (reação dos modos a cada sensor) e `test/native/test_sim` (voos em malha fechada).
