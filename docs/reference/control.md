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

| Поле | По умолчанию |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
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
| `FlightOutputState mix(const ControlCommand& c) const` | Команда → PWM. Крен/тангаж/рысканье ограничиваются ходом (`*_MAX_US`); элероны: левый = `flaps + roll`, правый = `flaps − roll` (вниз = «+»); PWM = `1500 ± отклонение` со знаком из `Config::*_REVERSED`, ограничение 1000..2000. `throttle` не заполняется |
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

## `ArmingManager`

**Файл:** `control/ArmingManager.h` · **Зависит от:** `Autopilot` (nullable), `RcChannelState`, `Config`, `Channels`

ARM отдельным тумблером SwA (CH5). Автомат — в
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Метод | Описание |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Без автопилота проверяется только газ |
| `void update(const RcChannelState& rc, bool receiverFailsafe)` | При failsafe — ничего (тумблер в failsafe-кадре не отражает пилота). Тумблер OFF → DISARM, `switchSeenOff = true`. Переход OFF→ON → проверки → ARM или отказ |
| `bool isArmed() const` | Заармлен |
| `const char* getLastRefusalReason() const` | Причина последнего отказа или `nullptr`; сбрасывается при выключении тумблера |

`checkFailureReason(rc)` — проверки ARM:

| Условие | Причина отказа |
|---|---|
| Газ ≥ `THROTTLE_LOW_US` | «газ не на минимуме» |
| Любой режим, кроме MANUAL, IMU есть, но не отвечает | «IMU не отвечает…» |
| Любой режим, кроме MANUAL, у IMU проблема предполётной проверки | текст `ImuSensor::getPreflightProblem()` |
| Режим с высотой (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), барометр есть, но не отвечает | «барометр не отвечает…» |

Датчик, которого нет в сборке (`nullptr`), ARM не блокирует; в MANUAL борт
армится вообще без датчиков. GPS-фикс в проверки намеренно не входит: без GPS
навигационные режимы ведут себя безопасно (круг на месте), а дом запишется,
когда GPS поймает спутники.

Инварианты: включение платы с тумблером в ON не армит; одна попытка на один
переход OFF→ON; потеря связи ARM не снимает.

---

## `FlightOutputs`

**Файл:** `control/FlightOutputs.h` · **Зависит от:** `IBoard`, `FlightOutputState`, `Config`

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
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` каждого выхода, печать статуса; `true`, если все **обязательные** получили канал |
| `bool isAttached(uint8_t ch) const` | Выход подключён (индекс вне диапазона → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | Значение выхода из состояния по таблице |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | Измеренный импульс против ожидаемого на каждом разведённом пине; «OK» при расхождении ≤ 15 мкс |
| `void write(const FlightOutputState&)` | Записать все выходы и запомнить состояние |
| `void setFailsafe()` | Нейтраль рулей (`FAILSAFE_*`), газ `FAILSAFE_THROTTLE`; AUX — как были (груз не сбрасывается от потери связи) |
| `void setBuzzer(bool on)` | пищалка платы (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | Последнее записанное состояние |

Добавить выход: строка таблицы + поле в `FlightOutputState` + индекс в
`ServoChannel` (+ пин и канал LEDC в `Esp32Board`).

---

## `FlightController`

**Файл:** `control/FlightController.h` · **Слой:** COORDINATION ·
**Зависит от:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`

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

Порядок `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. при живой связи — `switches->update(rc)` (режим, функции, крутилки);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. стики `mixer.fromSticks(rc)` (при живой связи) × `Knob::RATES`; закрылки
   `mixer.updateFlaps(цель)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → плавно, при потере связи — 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **всегда**;
6. пищалка: `Beeper::update(BEEPER, armed, failsafe, now)`;
7. связь потеряна → `applyLinkLoss()` и выход из такта;
8. `arming.update(rc, false)`;
9. `command = autopilot->getCommand()` (или стики без автопилота), закрылки — свои;
10. `output = mixer.mix(command)`; `output.throttle = autopilot->applyThrottle(pilotThrottle)`;
11. не armed или `MOTOR_KILL` → `throttle = PWM_MIN` (последним);
12. AUX1 — груз (`PAYLOAD_DROP`), AUX2 — камера (`Knob::CAMERA_TILT`, `CAMERA_STAB` вычитает тангаж);
13. `outputs.write(output)`.

`applyLinkLoss()`: если автопилот в failsafe (armed: RTH или планирование) —
рули и газ по команде автопилота (закрылки плавно убираются, `MOTOR_KILL`
по-прежнему глушит мотор, AUX — как были); иначе `outputs.setFailsafe()`.

---

## `Beeper`

**Файл:** `control/Beeper.h` · **Зависит от:** `Config`

| Метод | Описание |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | состояние пищалки: 2 Гц, если `Feature::BEEPER` или «модель потеряна» (не заармлен, связи нет дольше `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | режим «ищите меня в траве» |
