# AUTOPILOT — режимы, навигация, тумблеры

[← Справочник](README.md)

Автопилот получает стики пилота, тумблеры/крутилки (`PilotInputs`) и датчики,
а выдаёт **итоговую команду рулей** (`getCommand()`) и газ режима
(`applyThrottle()`). Без нужного датчика режим ведёт себя безопасно (рули у
пилота или в нейтрали), а не падает. Что делает каждый режим для пилота —
[AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md). Облётаны MANUAL и STABILIZE;
остальное проверено тестами и замкнутыми симуляциями (`test/native/test_sim`).

---

## `AutopilotMode`, `Feature`, `Knob`

**Файл:** `autopilot/AutopilotTypes.h`

`enum AutopilotMode : uint8_t` (нескоупный — числовые коды идут в JSON
`/api/setmode`, `/api/status` и в `MavlinkModes`):

| Значение | Код | Коротко (OLED) | Суть |
|---|---|---|---|
| `MODE_MANUAL` | 0 | MAN | рули = стики |
| `MODE_STABILIZE` | 1 | STAB | стик — угол крена/тангажа |
| `MODE_AUTO_TAKEOFF` | 2 | TKOFF | программа взлёта по газу пилота |
| `MODE_ALT_HOLD` | 3 | ALT | STABILIZE + высота рулём высоты |
| `MODE_ACRO` | 4 | ACRO | стик — угловая скорость |
| `MODE_CRUISE` | 5 | CRZ | курс + высота + автогаз |
| `MODE_LOITER` | 6 | LOIT | круги над точкой включения |
| `MODE_RTH` | 7 | RTH | домой, круги над домом |
| `MODE_LAUNCH` | 8 | LNCH | запуск с руки |
| `MODE_AUTO_LAND` | 9 | LAND | планирование + выравнивание |
| `MODE_SOARING` | 10 | SOAR | термики без мотора |
| `MODE_RESCUE` | 11 | RESQ | крылья ровно, нос вверх, газ |
| `MODE_COUNT` | 12 | | граница (`setMode()` игнорирует ≥) |

`enum class Feature : uint8_t` — функции тумблеров: `FLAPS`, `AIRBRAKE`,
`AUTO_TRIM`, `TURN_COORDINATION`, `MOTOR_KILL`, `BEEPER`, `PAYLOAD_DROP`,
`GEOFENCE`, `HOME_RESET`, `CAMERA_STAB`, `COUNT`.

`enum class Knob : uint8_t` — крутилки: `STAB_GAIN`, `MAX_BANK`,
`CRUISE_SPEED`, `FLAPS`, `CAMERA_TILT`, `RATES`, `LOITER_RADIUS`, `COUNT`.

`namespace AutopilotNames` — `mode()`, `modeShort()` (≤ 5 символов),
`feature()`, `knob()`: имена для лога, OLED, дашборда, MAVLink.

### `PilotInputs`

Состояние тумблеров и крутилок одного такта.

| Член | Описание |
|---|---|
| `bool has(Feature) const` | функция включена |
| `float knob(Knob) const` | положение крутилки −1…+1 |
| `bool isBound(Knob) const` | крутилка есть в таблице привязок |
| `float knobValue(Knob, min, default, max) const` | в единицах: центр — `default`, края — `min`/`max`; не привязана — `default` |

---

## `Binding`, `Bind`, `BindingCheck`

**Файл:** `autopilot/ControlBinding.h` · таблица — `config/Controls.h`

`struct Binding { Kind kind; uint8_t channel; AutopilotMode modes[3]; uint8_t modeCount; Feature feature; Knob knob; }`,
`Kind` = `MODES` / `MODE` / `FEATURE` / `KNOB`. Строки таблицы — фабрики
`namespace Bind` (все `constexpr`):

| Фабрика | Смысл |
|---|---|
| `modes(ch, up, middle, down)`, `modes(ch, up, down)` | тумблер выбора режима (зона по `PilotSwitches::zoneOf`) |
| `mode(ch, m)` | режим поверх, пока канал ≥ `SWITCH_ON_US` |
| `feature(ch, f)` | функция, пока канал ≥ `SWITCH_ON_US` |
| `knob(ch, k)` | крутилка, `(us − 1500) / 500`, ограничено ±1 |

`namespace BindingCheck` — рекурсивные `constexpr` (ядро ESP32 собирается
как C++11): `channelIsFree`, `channelsFree`, `channelsUnique`,
`modeSwitchCount`, `atMostOneModeSwitch`. Используются в `static_assert`
`Controls.h`.

---

## `PilotSwitches`

**Файл:** `autopilot/PilotSwitches.h` · **Зависит от:** `Autopilot*`, `RcChannelState`, таблица привязок

| Метод | Описание |
|---|---|
| `template <size_t N> PilotSwitches(Autopilot*, const Binding (&table)[N])` | своя таблица (тесты, симуляции) |
| `explicit PilotSwitches(Autopilot* = nullptr)` | таблица `Controls::BINDINGS` |
| `void update(const RcChannelState&)` | собрать `PilotInputs`, передать `autopilot->setInputs()`; `setMode()` — **только когда изменился итог** тумблеров (режим с дашборда/GCS не затирается каждый такт). Вызывается `FlightController` только при живой связи |
| `void printBindings() const` | раскладка в Serial при включении: `SwC (CH7): MANUAL / STABILIZE / AUTO_TAKEOFF (вверх / середина / вниз)` |
| `static const char* channelName(uint8_t)` | `"SwC (CH7)"`, `"VrA (CH9)"`… |
| `static uint8_t zoneOf(uint16_t us, uint8_t zones)` | 2 зоны: < 1500 / ≥ 1500; 3 зоны: < 1250 / < 1750 / ≥ 1750 |
| `getInputs()`, `binding(i)` | для телеметрии и тестов |

`Bind::mode` главнее `Bind::modes`; из нескольких включённых `Bind::mode`
побеждает верхняя строка.

---

## `Autopilot`

**Файл:** `autopilot/Autopilot.h` · **Зависит от:** `PidController`, `Navigation`, `AltitudeSpeedController`, `LaunchController`, `SoaringController`, `AutoTrim`, датчики (все nullable)

### Жизненный цикл

| Метод | Описание |
|---|---|
| `explicit Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr, AirspeedSensor* = nullptr)` | ПИД крена/тангажа Kp 5, Ki 0.5, Kd 0.5, выход ±500 мкс |
| `bool begin()` | загрузить триммер; `false` и сообщение, если нет IMU или барометра |
| `void setInputs(const PilotInputs&)` | тумблеры и крутилки этого такта (до `update`) |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs, const ControlCommand& sticks = {})` | раз за такт: датчики (всегда) → навигация и дом → сохранение триммера на земле → failsafe → геозабор → режим → координация разворота → автотриммер |
| `ControlCommand getCommand() const` | итоговые рули (roll/pitch/yaw, мкс) |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | газ режима: `PILOT` — газ пилота; `AUTO` — свой; `AT_LEAST` — не меньше своего (автовзлёт). ARM и `MOTOR_KILL` учитывает `FlightController` |

### Режимы

| Метод | Описание |
|---|---|
| `void setMode(AutopilotMode)` | тот же или ≥ `MODE_COUNT` — ничего; иначе сброс ПИД и автоматов, цели = текущие курс и высота, центр кругов = текущая точка (при GPS), RTH — высота возврата |
| `getMode()`, `getModeName()` | имя: `FAILSAFE_GLIDE` / `FAILSAFE_RTH` при потере связи, иначе режим |
| `isFailsafeActive()`, `isFailsafeGliding()`, `isFailsafeReturning()` | failsafe поверх режима |
| `isAutoThrottle()`, `getThrottleCorrection()` | газ режима (%, для лога и дашборда) |
| `getLaunchState()`, `getSoaringState()` | автоматы LAUNCH и SOARING |

### Выходы и диагностика

| Метод | Описание |
|---|---|
| `getRollCorrection()`, `getPitchCorrection()`, `getYawCorrection()` | команда − стики, мкс |
| `getDesiredRoll()`, `getDesiredPitch()`, `getTargetAltitude()` | цели |
| `const NavStatus& getNavStatus()` | GPS, дом, позиция, расстояние/пеленг на дом, курс и целевой курс, скорость для навигации, геозабор, сваливание |
| `getCourseSource()` | `CourseSource::NONE / GYRO / COMPASS / GPS` |
| `getAltitude()` | высота по барометру (м от точки включения) |
| `getInputs()`, `getAutoTrim()` | для телеметрии |
| `getImuSensor()` … `getAirspeedSensor()` | датчики (могут быть `nullptr`) |
| `getRollPid()`, `getPitchPid()`, `setPIDGains(...)` | ПИД (дашборд, параметры MAVLink) |

### Внутренняя механика

- `stabilize()` — ПИД угла с D по гироскопу, множитель `Knob::STAB_GAIN`;
  интегратор копится только при ARM и ошибке < `STAB_INTEGRATOR_ZONE_DEG`.
  `stabilizeOrManual()` — без IMU рули у пилота; `stabilizeOrNeutral()` — без
  IMU нейтраль (автоматические режимы).
- `imuReady()` = IMU есть, доступен и без предполётной проблемы.
- Скорость для навигации: трубка Пито → GPS → `NAV_ASSUMED_SPEED_MS`.
- `looksLanded()` — у земли по барометру, почти без вертикальной скорости,
  медленнее порога по трубке/GPS: только тогда триммер пишется во флеш.
- Failsafe: при GPS и доме — RTH с мотором, иначе планирование; начатый RTH
  не бросается от короткой потери GPS.

---

## `Geo`, `Guidance`, `GeoPoint`

**Файл:** `autopilot/Navigation.h`

Локальная плоскость «север/восток» в метрах (равнопромежуточная проекция —
для километров ошибка доли процента).

| Функция | Описание |
|---|---|
| `Geo::wrap180`, `Geo::wrap360` | нормализация углов |
| `Geo::offsetNE(a, b, north, east)`, `distance(a, b)`, `bearing(a, b)` | смещение, расстояние, пеленг 0..360 |
| `Geo::moved(a, north, east)` | точка со смещением |
| `Geo::fromGps(GpsData)` | `GeoPoint` из GPS |
| `Guidance::rollForCourse(target, course, bankLimit)` | крен на ошибку курса (`NAV_COURSE_GAIN`), ограничен |
| `Guidance::orbitCourse(bearingFromCenter, distance, radius, clockwise)` | курс векторного поля на окружность (`LOITER_CONVERGENCE`) |
| `Guidance::orbitBankDeg(speed, radius)` | упреждающий крен круга: atan(V²/(g·R)) |

## `AltitudeSpeedController`

**Файл:** `autopilot/AltitudeSpeedController.h` — TECS-lite.

| Метод | Описание |
|---|---|
| `float pitchFor(targetAlt, alt, climb, speed, dt, integrate)` | желаемая вертикальная скорость = `NAV_ALT_GAIN`·ошибка (≤ `NAV_MAX_CLIMB/SINK`); тангаж = упреждение asin(Vz/V) + ПИ по ошибке Vz, в пределах `NAV_MAX_CLIMB/DIVE_PITCH_DEG` |
| `float throttleFor(hasAirspeed, airspeed, targetAirspeed, cruisePct, dt, integrate)` | с трубкой — ПИ по воздушной скорости вокруг `cruisePct`; без — `cruisePct`; + `THROTTLE_PER_CLIMB_PCT` на требуемый набор |
| `reset()`, `getWantedClimb()` | |

## `LaunchController`

**Файл:** `autopilot/LaunchController.h`

`State`: `IDLE → READY` (газ поднят) `→ THROWN` (перегрузка > `LAUNCH_ACCEL_G`
дольше `LAUNCH_ACCEL_TIME_MS`) `→ CLIMB` (через `LAUNCH_MOTOR_DELAY_MS`: мотор,
тангаж `LAUNCH_CLIMB_PITCH_DEG`) `→ DONE` (`LAUNCH_CLIMB_MS` или
`LAUNCH_ALTITUDE_M`). Движение стиков до броска — отмена. Методы: `update(...)`,
`reset()`, `getState()`, `motorOn()`, `pitchTargetDeg()`, `stateName()`.

## `SoaringController`

**Файл:** `autopilot/SoaringController.h`

`State`: `GLIDE ⇄ THERMAL` (вариометр > `SOAR_THERMAL_CLIMB_MS` дольше
`SOAR_THERMAL_CONFIRM_MS` / среднее < `SOAR_EXIT_CLIMB_MS` за
`SOAR_EXIT_WINDOW_MS`), `→ MOTOR_CLIMB` (ниже `SOAR_MIN_ALTITUDE_M`, до
`SOAR_MAX_ALTITUDE_M`), `→ RETURN` (дальше `SOAR_MAX_DISTANCE_M`, до 70 % от
него). Методы: `update(climb, alt, distHome, dt, now)`, `reset(now)`,
`getState()`, `motorOn()`, `getAverageClimb()`, `stateName()`.

## `AutoTrim`

**Файл:** `autopilot/AutoTrim.h` · хранение — `Preferences` (NVS / флеш STM32), пространство `"autotrim"`

| Метод | Описание |
|---|---|
| `void load()` | триммер из NVS (нет — 0) |
| `void update(active, levelFlight, rollCmdUs, pitchCmdUs, dt)` | trim += команда · `AUTOTRIM_RATE` · dt, до ±`AUTOTRIM_MAX_US` |
| `bool saveIfChanged()` | записать, если менялся (вызывает `Autopilot` после DISARM на земле) |
| `reset()`, `getRoll()`, `getPitch()` | |

---

## `PidController`

**Файл:** `autopilot/PidController.h` · **Зависит от:** `Config` (номинальный `dt`)

| Метод | Описание |
|---|---|
| `explicit PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`, `getKp/Ki/Kd()` | коэффициенты |
| `setLimits(minOut, maxOut)` | ограничение выхода (по умолчанию ±500) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | выход, ограниченный `[min, max]` |
| `void reset()` | обнулить интегратор, `dt` — от «сейчас» |

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (скорость с датчика!)
out = constrain(P + I + D, min, max)
```

D — по скорости измеряемой величины (гироскоп °/с), а не по производной
ошибки: без шума дифференцирования и без скачка при смене уставки. `dt` — по
`micros()`; первый вызов после `reset()` или пауза > 0.1 с — номинальный
`LOOP_PERIOD_MS`.
