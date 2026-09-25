# AUTOPILOT — режимы, ПИД, выбор режима

[← Справочник](README.md)

Автопилот считает **коррекции**, которые `FlightController` прибавляет к
командам стиков, и газ для текущего режима. Без датчиков (или с неготовым IMU)
он просто не даёт коррекций. **В полёте не испытан.** Будущая замена
ПИД-стабилизации — контур обратной связи, см. [feedback.md](feedback.md).

---

## `AutopilotMode`

**Файл:** `autopilot/Autopilot.h` · **Вид:** `enum` (нескоупный, значения — в JSON `/api/setmode`)

| Значение | Код | Что делает | Как выбрать |
|---|---|---|---|
| `MODE_MANUAL` | 0 | Без коррекций | CH7 < 1250, дашборд |
| `MODE_STABILIZE` | 1 | ПИД крена/тангажа к горизонту поверх стиков | CH7 1250..1749, дашборд |
| `MODE_AUTO_TAKEOFF` | 2 | Программа газа и тангажа, крылья ровно | CH7 ≥ 1750, дашборд |
| `MODE_ALT_HOLD` | 3 | ПИД газа по высоте барометра | только дашборд |

---

## `PidController`

**Файл:** `autopilot/PidController.h` · **Зависит от:** `Config` (номинальный `dt`)

ПИД общего назначения.

| Метод | Описание |
|---|---|
| `PidController(kp = 1, ki = 0, kd = 0)` | |
| `setGains(kp, ki, kd)`, `getKp/Ki/Kd()` | Коэффициенты |
| `setLimits(minOut, maxOut)` | Ограничение выхода (по умолчанию ±500) |
| `float calculate(setpoint, feedback, feedbackRate, bool integrate = true)` | Коррекция, ограниченная `[min, max]` |
| `void reset()` | Обнулить интегратор, отсчёт `dt` — от «сейчас» |

Формулы (`calculate`):

```
error = setpoint − feedback
P = Kp · error
I = Ki · Σ(error·dt),  |Σ| ≤ INTEGRAL_LIMIT = 100      (integrate = false → Σ = 0)
D = −Kd · feedbackRate                                  (скорость с датчика!)
out = constrain(P + I + D, min, max)
```

Особенности:

- **D по скорости измеряемой величины** (гироскоп °/с, вариометр м/с), а не
  по производной ошибки: без шума численного дифференцирования и без скачка
  при смене уставки (derivative kick).
- `dt` меряется по `micros()`; первый вызов после `reset()` или пауза > 0.1 с —
  номинальный `LOOP_PERIOD_MS`.
- `integrate = false` (не заармлен) держит интегратор в нуле — на земле не
  копится поправка, которая дёрнет рули при взлёте.

---

## `Autopilot`

**Файл:** `autopilot/Autopilot.h` · **Зависит от:** `PidController`, `SensorInterface` (все сенсоры — nullable), `Config`

### Конструктор и жизненный цикл

| Метод | Описание |
|---|---|
| `Autopilot(ImuSensor* = nullptr, BarometerSensor* = nullptr, MagnetometerSensor* = nullptr, GpsSensor* = nullptr)` | Стартовые коэффициенты: крен/тангаж Kp 5, Ki 0.5, Kd 0.5, выход ±500 мкс; высота Kp 10, Ki 2, Kd 5, выход ±50 % |
| `bool begin()` | `false` и сообщение, если нет IMU или барометра |
| `void update(bool armed, bool linkLost, uint16_t pilotThrottleUs)` | Раз за такт: `update()` всех датчиков (всегда), затем планирование или обработчик режима |

### Режимы и газ

| Метод | Описание |
|---|---|
| `void setMode(AutopilotMode)` | Смена режима (повтор того же — ничего): сброс всех ПИД и состояния режима; ALT_HOLD запоминает текущую высоту как цель |
| `AutopilotMode getMode() const` | Текущий режим |
| `const char* getModeName() const` | `"FAILSAFE_GLIDE"` при планировании, иначе имя режима |
| `bool isFailsafeGliding() const` | Связь потеряна при ARM — держим планирование |
| `uint16_t applyThrottle(uint16_t pilotUs) const` | AUTO_TAKEOFF: `max(пилот, программа)`; ALT_HOLD: пилот + поправка % × 10 мкс, ограничение 1000..2000; иначе — газ пилота. ARM/failsafe здесь **не** учитываются |

### Выходы и диагностика

| Метод | Описание |
|---|---|
| `float getRollCorrection() const`, `getPitchCorrection() const` | Коррекции, мкс (знаки `ControlCommand`) |
| `float getThrottleCorrection() const` | ALT_HOLD: поправка −50..50 %; AUTO_TAKEOFF: программный газ 0..100 % |
| `getDesiredRoll()`, `getDesiredPitch()`, `getTargetAltitude()` | Текущие цели |
| `getImuSensor()`, `getBarometerSensor()`, `getMagnetometerSensor()`, `getGpsSensor()` | Датчики (могут быть `nullptr`) — для ARM, лога, дашборда |
| `getRollPid()`, `getPitchPid()` | ПИД (для дашборда) |
| `void setPIDGains(kpR, kiR, kdR, kpP, kiP, kdP)` | Перенастройка на ходу (`/api/setpid`) |

### Обработчики

| Обработчик | Логика |
|---|---|
| `handleManualMode()` | Все коррекции 0 |
| `handleStabilizeMode()` | Цели 0°/0°, `stabilize()` |
| `handleAutoTakeoffMode(pilotUs)` | Старт, когда armed и газ ≥ `TAKEOFF_TRIGGER_US` (1500); программа: 0–1 с газ 0→100 %, тангаж 0°; 1–3 с — 15°; далее 10°. Не armed — сброс старта |
| `handleAltHoldMode()` | Барометр недоступен — поправка 0; иначе ПИД(цель, высота, вертикальная скорость) |
| `handleFailsafeGlide()` | Цели `FAILSAFE_GLIDE_ROLL/PITCH_DEG`, газ-поправка 0, `stabilize()` |
| `stabilize()` | IMU не готов → коррекции 0; иначе ПИД(цель, угол, скорость гироскопа, `integrate = armed`) |

`imuReady()` = IMU есть, `isAvailable()` и нет `getPreflightProblem()` — с
перевёрнутой или переставленной платой коррекции пошли бы не туда, поэтому
лучше не корректировать совсем.

`setFailsafeGlide(active)` при смене состояния сбрасывает ПИД крена/тангажа и
старт автовзлёта (после восстановления связи — только заново) и печатает
событие.

---

## `AutopilotModeSelector`

**Файл:** `autopilot/AutopilotModeSelector.h` · **Зависит от:** `Autopilot*`, `RcChannelState`, `Channels`

CH7 (SwC, 3 положения) → режим.

| Метод | Описание |
|---|---|
| `explicit AutopilotModeSelector(Autopilot* = nullptr)` | |
| `void update(const RcChannelState& rc)` | Вызывает `autopilot->setMode()` **только при смене зоны** CH7 |
| `static AutopilotMode modeFor(uint16_t us)` (private) | `< 1250` MANUAL, `1250..1749` STABILIZE, `≥ 1750` AUTO_TAKEOFF |

Смена только по переходу зоны нужна, чтобы ALT_HOLD, выбранный с дашборда, не
перезаписывался положением тумблера каждый такт. Начальное «последнее» значение
— MANUAL, поэтому тумблер в MANUAL при старте ничего не вызывает.
