# AUTOPILOT / feedback — контур обратной связи (заготовка)

[← Справочник](README.md)

> ⚠️ **Заготовка, в прошивку не подключена.** Ни `FlightController`, ни
> `Autopilot`, ни `main.cpp` эти заголовки не включают. Проверяются замкнутой
> симуляцией (`test/test_feedback`, на ПК и на плате) и модульными нативными
> тестами. План подключения — в
> [DEVELOPER_GUIDE.md](../DEVELOPER_GUIDE.md#план-подключения).

Идея: вместо ПИД по углу с коэффициентами под одну скорость — регулятор,
замкнутый на **реакцию самолёта**, с моделью оси, изучаемой в полёте, защитой
от сваливания и этапами взлёта/посадки по датчикам. Единственный вход —
`FlightSnapshot`, единственный выход — `FeedbackOutput`.

Все модули header-only; `FeedbackModules.h` включает их одной строкой.

---

## namespace `FeedbackConfig`

**Файл:** `autopilot/feedback/FeedbackConfig.h`

Все константы контура (при подключении переедут в `Config.h`). Значения с
пометкой «прикидка» — для модели ~1 кг и размаха 1.2 м. Массивы `[AXIS_COUNT]`
индексируются осью.

| Группа | Константы |
|---|---|
| Общее | `GRAVITY = 9.80665`; оси `AXIS_ROLL = 0`, `AXIS_PITCH = 1`, `AXIS_YAW = 2`, `AXIS_COUNT = 3` |
| Скорость | `STALL_SPEED_MS = 8`, `REFERENCE_SPEED_MS = 14`, `ACCEL_FILTER_TAU_S = 0.3` |
| В воздухе/на земле | `AIRBORNE_HEIGHT_M = 3`, `AIRBORNE_CONFIRM_MS = 500`, `GROUND_STILL_MS = 2000`, `GROUND_ACCEL_TOLERANCE_G = 0.1` |
| Регулятор | `ANGLE_GAIN = {4, 4, 2}` 1/с, `MAX_RATE_DPS = {120, 60, 30}`, `RATE_TAU_S = {0.15, 0.20, 0.30}`, `RATE_INTEGRAL_GAIN = {2, 2, 1}`, `MAX_DEFLECTION_US = {400, 400, 400}`, `DAMPING_COMPENSATION = 0.5` |
| Эффективность рулей | `EFFECTIVENESS_PRIOR = {3, 1.5, 0.6}` °/с²/мкс, `EFFECTIVENESS_MIN = {0.2, 0.1, 0.05}`, `EFFECTIVENESS_MAX = {30, 15, 6}`, `RESPONSE_DELAY_MS = 40`, `RLS_FORGETTING = 0.995`, `ESTIMATOR_PERIOD_MS = 20`, `ESTIMATOR_PREFILTER_HZ = 2`, `MIN_EXCITATION_US = 30` |
| Сваливание | `DECEL_WARN_MS2 = 2`, `DECEL_CONFIRM_MS = 300`, `LOW_ENERGY_PITCH_DEG = 5`, `NOSE_DROP_RATE_DPS = 60`, `WING_DROP_RATE_DPS = 120`, `STALL_NOSE_UP_COMMAND_US = 50`, `LOW_EFFECTIVENESS_RATIO = 0.35`, `LOW_SPEED_MARGIN = 1.25`, `LOW_SPEED_EXIT_MARGIN = 1.5`, `LOW_ENERGY_THROTTLE_PERCENT = 80`, `LOW_ENERGY_MAX_PITCH_DEG = 5`, `STALL_THROTTLE_PERCENT = 100`, `STALL_MAX_PITCH_DEG = −5`, `STALL_MAX_BANK_DEG = 10`, `STALL_AILERON_LIMIT_US = 150`, `RECOVERY_HOLD_MS = 1000` |
| Взлёт | `TAKEOFF_HAND_LAUNCH = false`, `TAKEOFF_TRIGGER_THROTTLE_PERCENT = 50`, `TAKEOFF_THROTTLE_PERCENT = 100`, `LAUNCH_ACCEL_G = 1`, `LAUNCH_DETECT_MS = 50`, `ROTATE_SPEED_MS = 10`, `ROTATE_FALLBACK_MS = 1500`, `CLIMB_PITCH_DEG = 12`, `TAKEOFF_TARGET_ALTITUDE_M = 30`, `TAKEOFF_CLIMB_FALLBACK_MS = 10000`, `LAUNCH_TIMEOUT_MS = 8000`, `HEADING_HOLD_GAIN = 2` |
| Посадка | `APPROACH_SINK_RATE_MS = 1`, `APPROACH_THROTTLE_PERCENT = 25`, `APPROACH_BASE_PITCH_DEG = −3`, `APPROACH_MIN_PITCH_DEG = −10`, `APPROACH_MAX_BANK_DEG = 20`, `GO_AROUND_THROTTLE_PERCENT = 80`, `SINK_TO_PITCH_GAIN = 4`, `FLARE_HEIGHT_M = 2`, `FLARE_SINK_RATE_MS = 0.3`, `FLARE_MAX_PITCH_DEG = 8`, `TOUCHDOWN_ACCEL_G = 0.5`, `TOUCHDOWN_HEIGHT_M = 0.3`, `TOUCHDOWN_STILL_MS = 500`, `TOUCHDOWN_STILL_RATE_DPS = 5`, `ROLLOUT_MS = 5000` |

---

## namespace `FeedbackMath`

**Файл:** `autopilot/feedback/FeedbackMath.h` · **Зависит от:** `<math.h>`

| Функция | Описание |
|---|---|
| `float wrap180(float deg)` | Угол в `(−180, 180]`: разница курсов 350° и 10° — это −20° |
| `int8_t signOf(float x)` | −1 / 0 / +1 |
| `float clampAbs(float x, float limit)` | Ограничение `[−limit, limit]` |

---

## `FlightSnapshot`

**Файл:** `autopilot/feedback/FlightSnapshot.h` · **Вид:** struct

Всё, что контур знает о самолёте за такт. Знаки — авиационные.

| Группа | Поля |
|---|---|
| Время/статус | `timeUs`, `armed`, `linkLost` |
| Ориентация | `imuValid`, `rollDeg`, `pitchDeg`, `yawDeg`, `rollRateDps`, `pitchRateDps`, `yawRateDps`, `accelXg/Yg/Zg` |
| Высота | `baroValid`, `altitudeM` (от точки включения), `climbRateMs`, `heightAglValid`, `heightAglM` (будущий дальномер) |
| Скорость | `airspeedValid`, `airspeedMs` (будущая трубка Пито), `gpsValid`, `groundSpeedMs` |
| Цели режима | `stabilizationActive` (false = MANUAL: только обучение), `targetRollDeg`, `targetPitchDeg` |
| Команды, мкс | `stick*Us` — вклад пилота; `command*Us` — итог, реально ушедший на рули |
| Газ, % | `pilotThrottlePercent`, `throttlePercent` (реально на ESC) |
| Закрылки | `flapsUs`, `flapsMoving` |

---

## `FeedbackOutput`

**Файл:** `autopilot/feedback/FeedbackOutput.h` · **Вид:** struct

| Поле | Описание |
|---|---|
| `float deflectionUs[3]` | Отклонения рулей по осям, мкс (знаки `ControlCommand`) |
| `bool axisEnabled[3]` | `false` — ось не управляется, руль остаётся у пилота |
| `float throttleOverridePercent` | Абсолютный газ этапа полёта; `< 0` — не задан |
| `float throttleFloorPercent` | Нижняя граница газа (защита от сваливания); `< 0` — нет |
| `targetRollDeg`, `targetPitchDeg` | Итоговые цели после ограничений (отладка) |
| `const char* reason` | Краткое описание для лога/OLED |

---

## `PhaseTargets`

**Файл:** `autopilot/feedback/PhaseTargets.h` · **Вид:** struct

Общий выход `TakeoffSequencer` и `LandingSequencer` — «что», а не «как».

| Поле | По умолчанию | Описание |
|---|---|---|
| `active` | `false` | Этап сейчас управляет самолётом |
| `targetRollDeg`, `targetPitchDeg` | 0 | Цели |
| `controlRoll`, `controlPitch` | `true` | `false` — ось не трогать (на колёсах тангаж задаёт шасси) |
| `holdHeading`, `headingDeg` | `false`, 0 | Держать курс рулём направления и колесом |
| `throttlePercent` | −1 | −1 — газ пилота |
| `reason` | `""` | Описание |

---

## `SpeedEstimator`

**Файл:** `autopilot/feedback/SpeedEstimator.h`

Скорость (воздушная > путевая GPS > неизвестна) и продольное ускорение по IMU:
`dV/dt = g · (ax − sin θ)` через ФНЧ `ACCEL_FILTER_TAU_S` — «скорость падает»
видно даже без датчика скорости.

| Метод | Описание |
|---|---|
| `void update(const FlightSnapshot&)` | Шаг; `dt ≤ 0` или `> 0.5 с` от прошлого вызова (для первого — от `timeUs = 0`) — пропуск |
| `bool hasSpeed() const`, `float getSpeed() const`, `Source getSource() const` | `Source::{None, Gps, Airspeed}` |
| `bool hasAcceleration() const`, `float getAcceleration() const` | м/с², «+» — разгон; нет IMU — `hasAcceleration() == false` |
| `float effectivenessScale() const` | `(V / REFERENCE_SPEED)²`, ограничено `0.05..4`; без скорости — 1 |

---

## `AirborneDetector`

**Файл:** `autopilot/feedback/AirborneDetector.h`

В воздухе ли самолёт: учиться, копить интеграл и искать сваливание имеет смысл
только в полёте.

| Метод | Описание |
|---|---|
| `void update(const FlightSnapshot&, const SpeedEstimator&, uint32_t nowMs)` | Не armed — сброс в «на земле». Кандидат на смену состояния должен держаться `AIRBORNE_CONFIRM_MS` (взлёт) или `GROUND_STILL_MS` (посадка) |
| `void force(bool)` | Явно задать (знают взлёт/посадка) |
| `void reset()` | На земле |
| `bool isAirborne() const` | |

«Похоже на полёт»: высота по дальномеру или барометру > `AIRBORNE_HEIGHT_M`,
или скорость > `ROTATE_SPEED_MS`. «Похоже на землю»: низко, угловые скорости
всех осей < `TOUCHDOWN_STILL_RATE_DPS`, |a| ≈ 1g (± `GROUND_ACCEL_TOLERANCE_G`).

---

## `ControlEffectivenessEstimator`

**Файл:** `autopilot/feedback/ControlEffectivenessEstimator.h`

Одна ось. Модель: **угловое ускорение = b·руль(t − задержка) + a·ω + c**.
`b` — эффективность руля (°/с² на мкс, знак — направление реакции), `a` —
демпфирование (1/с, обычно < 0), `c` — постоянный момент (автотриммирование).
Учится `b` на опорной скорости: `b = b_ref · (V/V_ref)²`, `a = a_ref · V/V_ref`.
Оценка — рекурсивный МНК с забыванием (`λ = 0.995`, память ~4 с).

| Метод | Описание |
|---|---|
| `explicit ControlEffectivenessEstimator(uint8_t axis = AXIS_ROLL)` | Вызывает `reset()` |
| `void reset()` | θ = (prior, 0, 0); ковариация: b ± prior, a ± 5, c ± 100 |
| `void update(commandUs, rateDps, speedScale, learningAllowed, nowMs)` | Вызывать каждый такт: копит средние за интервал `ESTIMATOR_PERIOD_MS`, на конце интервала — ускорение по разности гироскопа, задержка команды, общий ФНЧ обеих сторон, шаг RLS (если можно учиться и есть раскачка) |
| `getEffectiveness()` | `b` на текущей скорости |
| `getReferenceEffectiveness()` | `b` на опорной скорости |
| `getDamping()` | `a` на текущей скорости |
| `getBias()` | `c` |
| `getTrimUs()` | `−c/b` (0, если `|b|` мал) |
| `getEffectivenessSigma()` | σ оценки `b` на текущей скорости |
| `bool isConfident() const` | ≥ 50 шагов RLS, `|b|` ≥ минимума и σ < 0.3·`|b|` |
| `getAngularAccel()` | Ускорение за последний интервал (отладка) |

Особенности:

- Интервал длиннее `MAX_GAP_MS = 200` (цикл стоял) — данные начинаются заново
  (история задержки и фильтр сбрасываются).
- Учится только при **раскачке**: размах средних команд за 16 интервалов
  (~0.3 с) ≥ `MIN_EXCITATION_US`; иначе оценка замирает.
- Гигиена float после шага: симметрия `P`, потолок дисперсий (×10 от
  начальных), ограничение `b` (`±EFFECTIVENESS_MAX`) и `a` (`−40..5`).

---

## `AxisModel`

**Файл:** `autopilot/feedback/AdaptiveRateController.h` · **Вид:** struct

Что известно о реакции оси для регулятора: `effectiveness` (b, по умолчанию
1), `damping` (a, 0 — не компенсировать), `bias` (c, 0).

---

## `AdaptiveRateController`

**Файл:** `autopilot/feedback/AdaptiveRateController.h`

Регулятор одной оси, три ступени:

```
ω* = clamp(ANGLE_GAIN · wrap180(цель − угол), MAX_RATE)
ε* = (ω* − ω + I) / RATE_TAU,     I += RATE_INTEGRAL_GAIN · (ω* − ω) · dt
руль = clamp((ε* − a·ω − c) / b, MAX_DEFLECTION)
```

| Метод | Описание |
|---|---|
| `explicit AdaptiveRateController(uint8_t axis = AXIS_ROLL)` | |
| `void reset()` | Интеграл, насыщение и выход — 0 |
| `float angleToRate(targetDeg, angleDeg) const` | Ступень 1 (кратчайший путь для курса) |
| `float update(desiredRateDps, rateDps, const AxisModel&, bool allowIntegral, float dt)` | Ступени 2–3, возвращает отклонение, мкс |
| `getDesiredRate()`, `getIntegral()`, `getOutput()`, `isSaturated()` | Состояние |

Инварианты: `|b|` не меньше `EFFECTIVENESS_MIN` (со знаком b); интеграл хранится
в °/с (остаётся верным при изменении `b`) и **не копится в сторону упора**
(anti-windup по направлению насыщения прошлого шага); `dt ≤ 0` — интеграл не
меняется.

---

## `StallGuard`

**Файл:** `autopilot/feedback/StallGuard.h`

Защита от потери скорости и сваливания. Уровни `Level::{Normal, LowEnergy, Stall}`.

| Метод | Описание |
|---|---|
| `void update(snapshot, speed, ControlState, bool airborne, nowMs)` | На земле или без IMU — сброс в Normal |
| `Level getLevel() const`, `const char* getLevelName() const` | `"OK"`, `"LOW_ENERGY"`, `"STALL"` |
| `const char* getReason() const` | Последний сработавший признак |
| `float maxPitchDeg() const` | Stall: −5°, LowEnergy: 5°, иначе 90° |
| `float maxBankDeg() const` | Stall: 10°, иначе 180° |
| `float maxAileronUs() const` | Stall: 150 мкс, иначе `MAX_DEFLECTION_US[ROLL]` |
| `float throttleFloorPercent(bool linkLost) const` | Stall 100 %, LowEnergy 80 %, иначе/без связи −1 |
| `void reset()` | Normal |

`StallGuard::ControlState` — `pitchEffectivenessKnown`, `pitchEffectiveness`
(модуль оценки b по тангажу).

Признаки **LowEnergy**: подтверждённое (`DECEL_CONFIRM_MS`) замедление
> `DECEL_WARN_MS2` при тангаже > 5°; скорость < `1.25·Vs`; уверенная
эффективность руля высоты < 35 % априорной. Признаки **Stall**: скорость < Vs;
нос падает быстрее 60 °/с при руле высоты «вверх» > 50 мкс; при малой энергии
крыло валится быстрее 120 °/с против элеронов. Выход из мер — через
`RECOVERY_HOLD_MS` и только когда энергия восстановлена (скорость ≥ `1.5·Vs`,
без датчика — ускорение ≥ 0).

---

## `TakeoffSequencer`

**Файл:** `autopilot/feedback/TakeoffSequencer.h`

Взлёт этапами. `State::{Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted}`.
Диаграмма — [ARCHITECTURE.md §7](../ARCHITECTURE.md#взлёт-и-посадка-контур-обратной-связи-не-подключён).

| Метод | Описание |
|---|---|
| `void request(nowMs)` | → `WaitThrottle` |
| `void cancel()` | Активный этап → `Aborted`; цели сброшены |
| `void update(snapshot, speed, nowMs)` | Не более одного перехода за такт, затем цели нового этапа |
| `void reset()` | → `Idle` |
| `getTargets()`, `getState()`, `getStateName()` | |
| `bool isActive() const` | WaitThrottle / WaitLaunch / GroundRoll / Climb |
| `bool isAirborne() const` | Climb / Complete |

Цели этапов: ожидание — газ 0, рули у пилота; разбег — газ 100 %, крылья ровно,
тангаж не трогать, держать курс, зафиксированный в момент старта; набор — газ
100 %, крылья ровно, тангаж `CLIMB_PITCH_DEG`. Бросок с руки: продольное
ускорение `ax − sin θ ≥ LAUNCH_ACCEL_G` дольше `LAUNCH_DETECT_MS`.

---

## `LandingSequencer`

**Файл:** `autopilot/feedback/LandingSequencer.h`

Посадка этапами. `State::{Idle, Approach, Flare, Rollout, Complete, Aborted}`.

| Метод | Описание |
|---|---|
| `void request(nowMs)` | → `Approach` |
| `void cancel()`, `void reset()`, `update(snapshot, nowMs)` | Как у взлёта |
| `bool isActive() const` | Approach / Flare / Rollout |
| `bool isNearGround() const` | Flare / Rollout — защиту от сваливания здесь выключают |
| `bool isOnGround() const` | Rollout / Complete |

Тангаж на снижении и выравнивании — от ошибки вертикальной скорости:
`θ = base + SINK_TO_PITCH_GAIN · (−sinkRate − climbRate)`, ограничено
`[min, FLARE_MAX_PITCH_DEG]`; без барометра — базовый угол. Высота — дальномер,
иначе барометр. Касание: всплеск |a − 1g| ≥ 0.5g или «низко и не вращается»
`TOUCHDOWN_STILL_MS`. На пробеге курс фиксируется в момент касания.

---

## `FeedbackSupervisor`

**Файл:** `autopilot/feedback/FeedbackSupervisor.h`

Контур целиком. Владеет `SpeedEstimator`, `AirborneDetector`, тремя
`ControlEffectivenessEstimator`, тремя `AdaptiveRateController`, `StallGuard`,
`TakeoffSequencer`, `LandingSequencer`.

| Метод | Описание |
|---|---|
| `bool requestTakeoff()` | Только armed, связь есть, на земле; отменяет посадку |
| `bool requestLanding()` | Только armed, связь есть, в воздухе; отменяет взлёт |
| `void cancelPhase()` | Отменить этап |
| `const FeedbackOutput& update(const FlightSnapshot&)` | Такт (порядок — [ARCHITECTURE.md §10](../ARCHITECTURE.md#10-контур-обратной-связи-не-подключён)) |
| `getOutput()`, `isAirborne()`, `getSpeedEstimator()`, `getEstimator(axis)`, `getController(axis)`, `getStallGuard()`, `getTakeoff()`, `getLanding()` | Состояние для лога и тестов |
| `void printStatus(Print& out) const` | Строка статуса + строка на ось (`b ± σ`, `*` — уверенная, `a`, `c`, `I`, выход) |

Ключевые правила:

- **Не armed** — все оси выключены, `reason = "не заармлен"`; ARM/DISARM
  (новый полёт) сбрасывает всё выученное.
- **Потеря связи** отменяет этапы; газ не трогается (действует failsafe
  прошивки).
- **Отрыв от земли** сбрасывает оценки и регуляторы (то, что «видели» на
  колёсах, не годится).
- Отрицательная уверенная оценка `b` **никогда** не идёт в регулятор — ось
  работает по априорной модели, а в `reason` предупреждение «… реагирует на
  руль наоборот? проверить на земле».
- Интеграл на земле заморожен, кроме курса на разбеге/пробеге.
- Координированный разворот (при известной скорости в воздухе): к желаемой
  скорости тангажа `+ g/V · sin φ · tg φ`, рысканья `g/V · sin φ` (крен ограничен ±60°).
- Приоритет `reason`: сваливание > мало энергии > этап > предупреждение о знаке
  > «стабилизация»/«ручное (обучение)».
