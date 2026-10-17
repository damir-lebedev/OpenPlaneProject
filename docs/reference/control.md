# CONTROL и COORDINATION — микшер, газ, ARM, выходы, оркестратор

[← Справочник](README.md)

Слой CONTROL — логика над данными без UART, PWM и Wi-Fi. `FlightController`
(COORDINATION) — единственный класс, который сводит все нижние слои в один
такт.

---

## `ControlCommand`

**Файл:** `control/ControlCommand.h` · **Вид:** struct

Команда на рули в **физических знаках**, мкс отклонения (±500 = полный ход).
Общий язык стиков, автопилота и микшера.

| Поле | «+» означает |
|---|---|
| `int16_t roll` | крен вправо (правый элерон вверх, левый вниз) |
| `int16_t pitch` | нос вверх (руль высоты вверх) |
| `int16_t yaw` | нос вправо (руль направления и колесо вправо) |
| `int16_t flaps` | закрылки вниз (оба элерона вниз); «−» — воздушный тормоз (оба вверх) |

Все поля по умолчанию 0.

---

## `FlightOutputState`

**Файл:** `control/FlightOutputState.h` · **Вид:** struct

Желаемые импульсы выходов, мкс PWM. По умолчанию — нейтраль рулей и газ `PWM_MIN`.
`throttle` — величина газа (1000..2000 = 0..100%) в любом направлении; в импульс
ESC её вместе с `motorReverse` переводит `FlightOutputs` (через `EscSignal`).

| Поле | По умолчанию |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `bool motorReverse` | `false` — мотор вперёд |
| `bool escPulsesOff` | `false`; `true` — импульсов на ESC нет вовсе (сигнал мотором, только без ARM) |
| `aux1` | `PAYLOAD_CLOSED_US` — сброс груза закрыт |
| `aux2` | `PWM_CENTER` — камера |

---

## `FlapsController`

**Файл:** `control/FlapsController.h` · **Зависит от:** `Config`

Плавный выпуск/уборка закрылков: положение идёт к цели (любое значение —
закрылки с тумблера, с крутилки, воздушный тормоз вверх) не быстрее полного
хода `FLAPS_DEPLOYED_US` за `FLAPS_TRANSITION_MS`. Время — параметром.

| Метод | Описание |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | Шаг к цели; возвращает текущее положение, мкс (+ вниз, − вверх) |
| `int16_t getPosition() const` | Текущее положение |

Инварианты:

- **Первый вызов** ставит положение сразу в цель — закрылки не «выезжают» на
  столе при включении.
- Шаг времени ограничен `MAX_STEP_MS = 20`: после долгой паузы (failsafe,
  калибровка) закрылки не прыгают к цели за один такт.

---

## `ControlMixer`

**Файл:** `control/ControlMixer.h` · **Зависит от:** `RcInput`, `RcChannelState`, `FlapsController`, `ControlCommand`, `FlightOutputState`, `Config`, `Channels`

Аэродинамическая логика в два шага. Владеет `FlapsController`.

| Метод | Описание |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll` (2000 = вправо); CH2 → `pitch` **с обратным знаком** (2000 = от себя = нос вниз); CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | цель закрылков выбирает `FlightController` (тормоз → тумблер закрылков → крутилка), здесь — плавный ход |
| `FlightOutputState mix(const ControlCommand& c) const` | Команда → PWM. Крен/тангаж/рысканье ограничиваются ходом (`*_MAX_US`); элероны: левый = `flaps + roll`, правый = `flaps − roll` (вниз = «+»); PWM = `1500 ± отклонение` со знаком из `Config::*_REVERSED`, знак закрылок — `FLAPS_REVERSED`, ограничение 1000..2000. `throttle` не заполняется |
| `int16_t getFlaps() const` | Текущее положение закрылков, мкс |

Флапероны: при выпуске оба элерона опускаются на `FLAPS_DEPLOYED_US` (новая
«нейтраль»), крен работает поверх. При полном крене опускающийся элерон
упирается в край хода раньше поднимающегося — это работает как дифференциал
элеронов.

---

## `ThrottleManager`

**Файл:** `control/ThrottleManager.h` · **Зависит от:** `RcInput`, `RcChannelState`, `Config`, `Channels`

| Метод | Описание |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | Газ CH3, ограниченный 1000..2000; при потере связи — `FAILSAFE_THROTTLE` |

Не знает про ARM и автопилот — их поправки применяет `FlightController`.

---

## `EscSignal`

**Файл:** `control/EscSignal.h` · **Вид:** namespace · **Зависит от:** `Config`

Газ (1000..2000 = 0..100%) и направление → импульс на пине ESC.

| Функция | Описание |
|---|---|
| `uint16_t pulseUs(uint16_t throttleUs, bool reverse)` | `ESC_BIDIRECTIONAL` (BLHeli 80A в 3D): 0% → `ESC_NEUTRAL_US` (1500), вперёд 1500..2000, назад 1500..1000. Однонаправленный ESC: 1000..2000 как есть, `reverse` игнорируется. Газ вне 1000..2000 обрезается |
| `uint16_t stopUs()` | Импульс «мотор стоит» (1500 у двунаправленного, 1000 у обычного) |

Только перевод, без защит: когда менять направление — решает `MotorReverse`,
предел газа назад — `FlightController`.

---

## `MotorReverse`

**Файл:** `control/MotorReverse.h` · **Зависит от:** `Config`

Направление мотора по тумблеру `Feature::MOTOR_REVERSE` (по умолчанию SwD) —
**только без ARM**.

| Метод | Описание |
|---|---|
| `void update(bool requested, bool armed, uint32_t nowMs)` | Не armed — направление сразу следует за тумблером (в консоль `MotorReverse: мотор НАЗАД` / `вперёд`). Armed — тумблер не действует, один раз печатается «реверс НЕ переключён», новое направление ждёт DISARM. С `ESC_BIDIRECTIONAL = false` реверс не включается никогда |
| `bool isReversed() const` | Мотор назад |
| `bool isPending() const` | Тумблер уже в другом положении, направление ждёт DISARM |
| `const char* armBlockReason(uint32_t nowMs) const` | Причина запретить ARM: с последней смены направления прошло меньше `REVERSE_SETTLE_MS` (мотор мог ещё крутиться); иначе `nullptr` |

---

## `ArmingManager`

**Файл:** `control/ArmingManager.h` · **Зависит от:** `Autopilot` (nullable), `RcChannelState`, `Config`, `Channels`

ARM отдельным тумблером SwA (CH5). Автомат — в
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Метод | Описание |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Без автопилота проверяется только газ |
| `void update(const RcChannelState& rc, bool receiverFailsafe, const char* blockReason = nullptr)` | При failsafe — ничего (тумблер в failsafe-кадре не отражает пилота). Тумблер OFF → DISARM, `switchSeenOff = true`. Переход OFF→ON → проверки → ARM или отказ. `blockReason` — внешняя причина отказа (от `FlightController`: `MotorReverse::armBlockReason()`) |
| `bool isArmed() const` | Заармлен |
| `const char* getLastRefusalReason() const` | Причина последнего отказа или `nullptr`; сбрасывается при выключении тумблера |

`checkFailureReason(rc)` — проверки ARM:

| Условие | Причина отказа |
|---|---|
| Газ ≥ `THROTTLE_LOW_US` | «газ не на минимуме» |
| Любой режим, кроме MANUAL, IMU есть, но не отвечает | «IMU не отвечает…» |
| Любой режим, кроме MANUAL, у IMU проблема предполётной проверки | текст `ImuSensor::getPreflightProblem()` |
| Режим с высотой (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), барометр есть, но не отвечает | «барометр не отвечает…» |
| Остальное прошло, но передан `blockReason` | `blockReason` (например, «направление мотора только что сменилось…») |

Датчик, которого нет в сборке (`nullptr`), ARM не блокирует; в MANUAL борт
армится вообще без датчиков. GPS-фикс в проверки намеренно не входит: без GPS
навигационные режимы ведут себя безопасно (круг на месте), а дом запишется,
когда GPS поймает спутники.

Инварианты: включение платы с тумблером в ON не армит; одна попытка на один
переход OFF→ON; потеря связи ARM не снимает.

---

## `FlightOutputs`

**Файл:** `control/FlightOutputs.h` · **Зависит от:** `IBoard`, `FlightOutputState`, `EscSignal`, `Config`

Единственный класс, который знает набор и порядок PWM-выходов. Все выходы
описаны одной таблицей; `begin()`, `write()`, статус и самопроверка проходят по
ней циклом.

### `FlightOutputs::OutputInfo`

| Поле | Описание |
|---|---|
| `const char* key` | Имя в JSON/логе (`aileronLeft`, …, `esc`, `rudder`, `aux1`, `aux2`) |
| `const char* label` | Имя для человека |
| `int16_t pin` | Номер пина; `-1` — не разведён. `int16_t`, потому что у STM32 номера аналоговых пинов — `0xC0 + N` |
| `bool required` | Без него борт не летит (руль направления — необязательный) |
| `uint16_t FlightOutputState::* field` | Указатель на поле состояния |

| Метод | Описание |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | Строка таблицы; порядок = `ServoChannel` |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` каждого выхода, печать статуса и типа ESC; `true`, если все **обязательные** получили канал |
| `bool isAttached(uint8_t ch) const` | Выход подключён (индекс вне диапазона → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | Импульс на пине выхода: поле по таблице, у ESC — через `EscSignal::pulseUs(throttle, motorReverse)`, при `escPulsesOff` — 0 |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | Измеренный импульс против ожидаемого на каждом разведённом пине; «OK» при расхождении ≤ 15 мкс |
| `void write(const FlightOutputState&)` | Записать все выходы и запомнить состояние; ESC при `escPulsesOff` — `IServoOutput::stopPulses()` |
| `void setFailsafe()` | Нейтраль рулей (`FAILSAFE_*`), газ `FAILSAFE_THROTTLE` (на пине двунаправленного ESC — 1500); AUX — как были (груз не сбрасывается от потери связи) |
| `void setBuzzer(bool on)` | пищалка платы (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | Последнее записанное состояние |

Добавить выход: строка таблицы + поле в `FlightOutputState` + индекс в
`ServoChannel` (+ пин и канал LEDC в `Esp32Board`).

---

## `FlightController`

**Файл:** `control/FlightController.h` · **Слой:** COORDINATION ·
**Зависит от:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`, `MotorReverse`, `GroundCalibration`, `SignalPlayer`

Единственный координатор цикла управления: сам не парсит UART, не трогает
PWM, не считает микшер — только вызывает остальных в правильном порядке.
Подробная диаграмма — [ARCHITECTURE.md §6](../ARCHITECTURE.md#6-такт-управления-flightcontrollerupdate).

| Метод | Описание |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | Без автопилота — чистое ручное управление; без тумблеров — только стики |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | Один такт (см. ниже) |
| `bool isReceiverFailsafe() const` | Связь потеряна |
| `const IBusReceiver& getReceiver() const` | Для лога (счётчики кадров, причина потери) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | Последнее записанное на выходы |
| `const RcChannelState& getRcState() const` | Каналы |
| `const FlightOutputs& getOutputs() const` | Таблица выходов и `attached` |
| `int16_t getFlapsUs() const` | Положение закрылков |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | Тумблеры и крутилки этого такта |
| `bool isLostModelBeeping() const` | Пищалка «я здесь» работает |
| `const MotorReverse& getMotorReverse() const` | Направление мотора и ожидающий реверс |
| `void toggleGroundCalibration()` | Консоль `c`: калибровка самолёта на земле — запустить (только без ARM), идёт — отменить |
| `void playSignal(uint8_t count)` | Консоль `e`: сигнал (без ARM — и мотором) |
| `const GroundCalibration& getCalibration() const`, `const SignalPlayer& getSignals() const` | Для OLED, лога и чёрного ящика |

Порядок `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. при живой связи — `switches->update(rc)` (режим, функции, крутилки);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. стики `mixer.fromSticks(rc)` (при живой связи) × `Knob::RATES`; закрылки
   `mixer.updateFlaps(цель)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → плавно, при потере связи — 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **всегда**; в
   реверсе вместо `pilotThrottle` — `PWM_MIN` (для автопилота мотор выключен);
6. калибровка: щелчок ARM во время неё — отмена; без ARM и при связи — жест
   тумблером `CAL_SWITCH`; шаг `GroundCalibration::update()`; её сигналы и
   сигнал отказа автовзлёта — в `SignalPlayer`; пищалка:
   `Beeper::update(BEEPER, armed, failsafe, now)` или свет сигнала;
7. связь потеряна → `applyLinkLoss()` и выход из такта;
8. `arming.update(rc, false, причина)` — ARM нельзя, пока идёт калибровка или
   звучит сигнал (и в такт, когда щелчок ARM отменил калибровку), иначе
   `reverse.armBlockReason(now)`; затем `reverse.update(MOTOR_REVERSE, armed, now)`
   — на DISARM направление меняется в том же такте;
9. `command = autopilot->getCommand()` (или стики без автопилота), закрылки —
   свои; кивок сигнала — к рулям высоты и направления;
10. `output = mixer.mix(command)`; `output.motorReverse = reverse.isReversed()`;
    вперёд — `output.throttle = autopilot->applyThrottle(pilotThrottle)` с пределом
    `THROTTLE_LIMIT_PCT` × крутилка `Knob::THROTTLE_LIMIT`, назад — газ пилота с пределом
    `min(REVERSE_THROTTLE_LIMIT_PCT, передний предел)`; разрешённое растягивается
    на весь ход стика;
11. не armed или `MOTOR_KILL` → `throttle = PWM_MIN` (последним); без ARM
    сигнал мотором — `escPulsesOff`;
12. AUX1 — груз (`PAYLOAD_DROP`), AUX2 — камера (`Knob::CAMERA_TILT`, `CAMERA_STAB` вычитает тангаж); `AUX1_REVERSED`/`AUX2_REVERSED` зеркалят импульс;
13. `outputs.write(output)`.

`applyLinkLoss()`: если автопилот в failsafe (armed: RTH или планирование) —
рули и газ по команде автопилота (закрылки плавно убираются, `MOTOR_KILL` и
реверс глушат мотор, AUX — как были); иначе `outputs.setFailsafe()`, а пока
звучит сигнал — нейтраль с кивком и паузой импульсов ESC (пульт положили —
калибровка идёт).

---

## `Beeper`

**Файл:** `control/Beeper.h` · **Зависит от:** `Config`

| Метод | Описание |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | состояние пищалки: 2 Гц, если `Feature::BEEPER` или «модель потеряна» (не заармлен, связи нет дольше `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | режим «ищите меня в траве» |

---

## `GroundCalibration`

**Файл:** `control/GroundCalibration.h` · **Зависит от:** `Config`, `RcChannelState`, `ImuSensor`, `ImuOrientation`

Наземная калибровка самолёта: шаг 1 — хвост поднят, фюзеляж горизонтально
(горизонт, `ImuOrientation::withLevel()`), шаг 2 — на колёсах (стояночный
угол). Шаг засчитывается сам: неподвижность `CAL_STILL_MS`; шаг 2 ждёт, пока
самолёт переставят, или `CAL_PARK_WAIT_MS`. Сохраняется только целиком
(`ImuSensor::setGroundCalibration()`), после проверки стояночного угла для
`LANDING_GEAR`. Не блокирует цикл.

`Stage`: `IDLE`, `LEVEL`, `PARK`, `DONE`, `FAILED`.

| Метод | Описание |
|---|---|
| `bool watchSwitch(const RcChannelState&, uint32_t nowMs)` | жест: тумблер `CAL_SWITCH` вниз-вверх `CAL_SWITCH_FLIPS` раз за `CAL_SWITCH_WINDOW_MS`, газ внизу; первый взгляд и газ не внизу — счёт заново |
| `void forgetSwitch()` | ARM или нет связи — счёт заново |
| `bool start(const ImuSensor*, uint32_t nowMs, const char* source)` | начать (сигнал 1); нет IMU или не умеет — ошибка (сигнал 3) |
| `void cancel(const char* why)` | отмена без сигнала |
| `void update(ImuSensor*, bool armed, bool signalBusy, uint32_t nowMs)` | шаг: ARM — отмена; пока звучит сигнал — не засчитывается; таймаут `CAL_STEP_TIMEOUT_MS` — ошибка |
| `isActive()`, `getStage()`, `getStageSinceMs()`, `getFailure()`, `getParkPitch()`, `getParkRoll()` | состояние и итог |
| `uint8_t takeSignal()` | сколько сигналов сыграть: 1 — начали / шаг 1, 2 — готово, 3 — ошибка (забирается один раз) |
| `static const char* stageShort(Stage)` | `CAL1`, `CAL2`, `CALOK`, `CALER` — для OLED |

---

## `SignalPlayer`

**Файл:** `control/SignalPlayer.h` · **Зависит от:** `Config`

Сигналы пилоту — функция времени от старта серии. Один сигнал: кивок рулей
высоты и направления (`SIGNAL_NOD_US` вверх, потом вниз, по `SIGNAL_NOD_MS`),
без ARM — пауза импульсов ESC `SIGNAL_ESC_PAUSE_MS` (регулятор потом играет
мелодию готовности), свет пищалки на время кивка; следующий — через
`SIGNAL_PERIOD_MS`.

| Метод | Описание |
|---|---|
| `void play(uint8_t count, uint32_t nowMs, bool motor)` | серия; `motor` — можно ли трогать ESC (и `SIGNAL_MOTOR`) |
| `void stop()` | тишина |
| `bool isBusy(uint32_t nowMs) const`, `uint8_t getCount(uint32_t nowMs) const` | серия звучит (ARM в это время запрещён) |
| `bool escPulsesOff(uint32_t nowMs) const` | импульсы на ESC убрать сейчас |
| `int16_t nodUs(uint32_t nowMs) const` | кивок рулей, мкс (`SIGNAL_SURFACES`) |
| `bool light(uint32_t nowMs) const` | пищалка / светодиод на `PIN_BUZZER` |
