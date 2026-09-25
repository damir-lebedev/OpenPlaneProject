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
| `int16_t flaps` | закрылки вниз (оба элерона вниз) |

Все поля по умолчанию 0.

---

## `FlightOutputState`

**Файл:** `control/FlightOutputState.h` · **Вид:** struct

Желаемые импульсы выходов, мкс PWM. По умолчанию — нейтраль рулей и газ `PWM_MIN`.

| Поле | По умолчанию |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |

---

## `FlapsController`

**Файл:** `control/FlapsController.h` · **Зависит от:** `Config`

Плавный выпуск/уборка закрылков: положение идёт к цели (0 или
`FLAPS_DEPLOYED_US`) не быстрее полного хода за `FLAPS_TRANSITION_MS`. Время —
параметром.

| Метод | Описание |
|---|---|
| `int16_t update(bool deployed, uint32_t nowMs)` | Шаг к цели; возвращает текущее положение, мкс вниз |
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
| `ControlCommand fromSticks(const RcChannelState& rc, uint32_t nowMs)` | CH1 → `roll` (2000 = вправо); CH2 → `pitch` **с обратным знаком** (2000 = от себя = нос вниз); CH4 → `yaw`; CH6 ≥ `FLAPS_SWITCH_ON_US` → цель закрылков, `flaps` = плавное положение |
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
| Режим STABILIZE/AUTO_TAKEOFF, IMU есть, но не отвечает | «IMU не отвечает…» |
| Режим STABILIZE/AUTO_TAKEOFF, у IMU проблема предполётной проверки | текст `ImuSensor::getPreflightProblem()` |
| Режим ALT_HOLD, барометр есть, но не отвечает | «барометр не отвечает…» |

Датчик, которого нет в сборке (`nullptr`), ARM не блокирует; в MANUAL борт
армится вообще без датчиков. GPS-фикс в проверки намеренно не входит — ни один
режим пока не использует GPS.

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
| `const char* key` | Имя в JSON/логе (`aileronLeft`, …, `esc`, `rudder`) |
| `const char* label` | Имя для человека |
| `int8_t pin` | GPIO; `-1` — не разведён |
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
| `void setFailsafe()` | Нейтраль рулей (`FAILSAFE_*`), газ `FAILSAFE_THROTTLE` |
| `const FlightOutputState& getLastState() const` | Последнее записанное состояние |

Добавить выход: строка таблицы + поле в `FlightOutputState` + индекс в
`ServoChannel` (+ пин и канал LEDC в `Esp32Board`).

---

## `FlightController`

**Файл:** `control/FlightController.h` · **Слой:** COORDINATION ·
**Зависит от:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `AutopilotModeSelector*`

Единственный координатор цикла управления: сам не парсит UART, не трогает
PWM, не считает микшер — только вызывает остальных в правильном порядке.
Подробная диаграмма — [ARCHITECTURE.md §6](../ARCHITECTURE.md#6-такт-управления-flightcontrollerupdate).

| Метод | Описание |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, AutopilotModeSelector* = nullptr)` | Без автопилота — чистое ручное управление |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | Один такт (см. ниже) |
| `bool isReceiverFailsafe() const` | Связь потеряна |
| `const IBusReceiver& getReceiver() const` | Для лога (счётчики кадров, причина потери) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | Последнее записанное на выходы |
| `const RcChannelState& getRcState() const` | Каналы |
| `const FlightOutputs& getOutputs() const` | Таблица выходов и `attached` |
| `int16_t getFlapsUs() const` | Положение закрылков |

Порядок `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. `pilotThrottle = throttle.update(rc, failsafe)`;
3. при живой связи — `modeSelector->update(rc)`;
4. `autopilot->update(armed, failsafe, pilotThrottle)` — **всегда**;
5. связь потеряна → `applyLinkLoss()` и выход из такта;
6. `arming.update(rc, false)`;
7. `command = mixer.fromSticks(rc, millis())` + коррекции автопилота, ограничение ±500 (`clampCommand`);
8. `output = mixer.mix(command)`;
9. `output.throttle = autopilot->applyThrottle(pilotThrottle)`; если не armed — `PWM_MIN`;
10. `outputs.write(output)`.

`applyLinkLoss()`: если автопилот в режиме планирования
(`isFailsafeGliding()`: armed и связь потеряна) — рули по коррекциям
планирования (закрылки убраны), газ `FAILSAFE_THROTTLE`; иначе
`outputs.setFailsafe()`.
