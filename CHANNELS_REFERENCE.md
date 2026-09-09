# 📻 RC Channels Quick Reference

Быстрый справочник по всем 10 RC каналам iBUS приёмника.

---

## Channel Mapping

| CH | Name | Russian | Диапазон | Нейтраль | Назначение |
|:--:|:----:|:--------:|:--------:|:--------:|-----------|
| **CH1** | **Aileron** | **Элероны** | 1000–2000 µs | 1500 µs | **Крены (Roll)** — управление креном самолёта |
| **CH2** | **Elevator** | **Лифт** | 1000–2000 µs | 1500 µs | **Тангаж (Pitch)** — управление высотой |
| **CH3** | **Throttle** | **Газ** | 1000–2000 µs | 1000 µs* | **Мощность мотора** — 0–100% |
| **CH4** | **Rudder** | **Руль** | 1000–2000 µs | 1500 µs | **Рысканье (Yaw)** — ⚠️ НЕ ИСПОЛЬЗУЕТСЯ |
| **CH5** | **Flaps** | **Закрылки** | 1000–2000 µs | 1500 µs | **Положение флапов** — 3 уровня |
| **CH6** | **Boost** | **Ускорение** | 1000–2000 µs | 1500 µs | **Режим Boost** — дополнительная мощность |
| CH7-CH10 | *Reserved* | *Зарезервированы* | 1000–2000 µs | 1500 µs | Для будущего: GPS, IMU, Telemetry |

---

## 🎮 Пульт → Значения Каналов

### CH1 — Aileron (Элероны)

```
LEFT STICK HORIZONTAL
┌─────────┬─────────┬─────────┐
│ 1000 µs │ 1500 µs │ 2000 µs │
│ влево   │ центр   │ вправо  │
│ -100%   │   0%    │  +100%  │
└─────────┴─────────┴─────────┘
       ↓
   LEFT AILERON:  1000 µs (вверх)
   RIGHT AILERON: 2000 µs (вниз)
```

### CH2 — Elevator (Лифт)

```
LEFT STICK VERTICAL
┌─────────┬─────────┬─────────┐
│ 1000 µs │ 1500 µs │ 2000 µs │
│  вверх  │ центр   │  вниз   │
│ -100%   │   0%    │  +100%  │
└─────────┴─────────┴─────────┘
       ↓
   ELEVATOR: 1000–2000 µs
```

### CH3 — Throttle (Газ)

```
THROTTLE SLIDER / STICK
┌─────────┬─────────┬─────────┐
│ 1000 µs │ 1500 µs │ 2000 µs │
│  MIN    │  50%    │  MAX    │
│   0%    │   50%   │  100%   │
└─────────┴─────────┴─────────┘
       ↓
   ESC (MOTOR): 1000–2000 µs
   
   ⚠️ ВАЖНО: 1000 µs = мотор ВЫКЛЮЧЕН!
            (не 0, а минимум 1000)
```

### CH5 — Flaps (Закрылки)

```
3-POSITION SWITCH
   ┌──────────┬──────────┬──────────┐
   │ < 1250   │ 1250-    │ > 1750   │
   │   µs     │  1750 µs │   µs     │
   │          │          │          │
   │ УБРАНЫ   │ ПОЛОВИНА │ ВЫПУЩЕНЫ │
   │ 0 µs     │ 50 µs    │ 100 µs   │
   └──────────┴──────────┴──────────┘
         ↓
   Add OFFSET to AILERONS
```

### CH6 — Boost (Ускорение)

```
2-POSITION OR ANALOG SWITCH
   ┌──────────────┬──────────────┐
   │ < 1250 µs    │ > 1750 µs    │
   │              │              │
   │ РАЗБЛОКИРОВАН│ АКТИВЕН      │
   │ (готов)      │ (максимум 5s) │
   └──────────────┴──────────────┘
         ↓
   Throttle += 250 µs (максимум 5 секунд)
   После → выключить CH6 в LOW для повтора
```

---

## 📊 Логика Каналов в Коде

### Где обрабатываются каналы?

```cpp
// src/main.cpp — главный loop
loop()
  ├─ flightController.update()
  │  │
  │  ├─ receiver.update()
  │  │  └─ Читаем все 10 каналов из UART
  │  │
  │  ├─ mixer.calculate(rc)
  │  │  ├─ CH1 (Aileron)  → Left/Right Aileron
  │  │  ├─ CH2 (Elevator) → Elevator
  │  │  └─ CH5 (Flaps)    → Offset для элеронов
  │  │
  │  └─ throttle.update(rc)
  │     ├─ CH3 (Throttle) → Motor PWM
  │     └─ CH6 (Boost)    → Дополнительная мощность
  │
  └─ debugLogger.update()
     └─ Выводит все 10 каналов в Serial
```

---

## 🛠️ Как Читать Канал в Коде

```cpp
// Получить текущее состояние каналов
const RcChannelState& rc = receiver.getState();

// Прочитать конкретный канал (в микросекундах)
uint16_t ch1 = rc.get(Channels::AILERON);     // 1000–2000 µs
uint16_t ch2 = rc.get(Channels::ELEVATOR);    // 1000–2000 µs
uint16_t ch3 = rc.get(Channels::THROTTLE);    // 1000–2000 µs
uint16_t ch4 = rc.get(Channels::RUDDER);      // 1000–2000 µs
uint16_t ch5 = rc.get(Channels::FLAPS);       // 1000–2000 µs
uint16_t ch6 = rc.get(Channels::BOOST);       // 1000–2000 µs
uint16_t ch7 = rc.get(Channels::AUX7);        // 1000–2000 µs
// ... CH8, CH9, CH10

// Преобразовать в проценты (0–100%)
uint16_t throttlePercent = (ch3 - 1000) / 10;  // 0–100
//  1000 µs → 0%
//  1500 µs → 50%
//  2000 µs → 100%
```

---

## ⚡ Быстрые Примеры

### Проверить, газ в минимуме?

```cpp
uint16_t throttle = rc.get(Channels::THROTTLE);
if (throttle < 1050)  // Рядом с минимумом
{
    // Газ выключен
}
```

### Включить boost?

```cpp
uint16_t boostSwitch = rc.get(Channels::BOOST);
if (boostSwitch > 1750)
{
    // Переключатель в HIGH → Включить boost
    // (максимум 5 секунд)
}
```

### Получить отклонение элерона

```cpp
uint16_t aileronRaw = rc.get(Channels::AILERON);
int16_t aileronDelta = aileronRaw - 1500;
// -500…0…+500 µs отклонение от центра
```

### Флапы выпущены?

```cpp
uint16_t flaps = rc.get(Channels::FLAPS);
if (flaps >= 1750)
{
    // Флапы полностью выпущены (100 µs offset)
}
```

---

## 🔍 Отладка Каналов

Чтобы увидеть все 10 каналов в реальном времени:

```
1. Откройте Arduino IDE
2. Подключите ESP32-C3 по USB
3. Tools → Serial Monitor (115200 baud)
4. Смотрите вывод каждые 500ms
```

Пример вывода:

```
CH1(Aileron)   1400 µs
CH2(Elevator)  1500 µs
CH3(Throttle)  1200 µs
CH4(Rudder)    1500 µs
CH5(Flaps)     1750 µs
CH6(Boost)     1000 µs
CH7-CH10: 1500 µs (default)

Armed: false
Failsafe: false
Boost: false
```

---

## 📋 Стандартные Значения

```
Минимум (min):    1000 µs  (0%)
Четверть (1Q):    1250 µs  (25%)
Половина (mid):   1500 µs  (50%)
Три четверти (3Q):1750 µs  (75%)
Максимум (max):   2000 µs  (100%)

Deadzone (мёртвая зона): ±50–100 µs от нейтрали
(диапазон, где stick считается в центре)
```

---

## 🛑 Failsafe Режим

Если сигнал потерян (> 500ms без кадров):

```cpp
// Все каналы выставляются на безопасные значения:

CH1 (Aileron)  → 1500 µs (нейтраль)
CH2 (Elevator) → 1500 µs (нейтраль)
CH3 (Throttle) → 1000 µs (ВЫКЛЮЧЕН!)
CH4 (Rudder)   → 1500 µs (нейтраль)
CH5 (Flaps)    → 1500 µs (убраны)
CH6 (Boost)    → 1500 µs (отключён)
```

---

## 🚀 Расширение (Future)

В будущем можно использовать CH7–CH10 для:

```
CH7 → GPS Mode / Auto-pilot Mode
CH8 → Camera Control / Recording
CH9 → Telemetry / Video Transmitter Power
CH10→ Second Motor / Auxiliary System
```

Код уже готов — просто используйте:

```cpp
uint16_t ch7 = rc.get(Channels::AUX7);
uint16_t ch8 = rc.get(Channels::AUX8);
uint16_t ch9 = rc.get(Channels::AUX9);
uint16_t ch10 = rc.get(Channels::AUX10);
```

---

**OpenPlaneProject — Управление от пульта до самолёта! 🛩️**
