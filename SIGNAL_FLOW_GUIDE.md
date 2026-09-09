# 📡 SIGNAL FLOW GUIDE — Полный поток сигнала управления

Этот документ описывает, как сигнал проходит от передатчика пульта управления до управления сервоприводами и мотором.

---

## 📊 Общая архитектура

```
┌─────────────────────────────────────────────────────────────┐
│                    RC TRANSMITTER (ПУЛЬТ)                  │
│         Отправляет iBUS кадры в эфир на 2.4GHz             │
└────────────────────────┬────────────────────────────────────┘
                         │ RF сигнал в эфире
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              RC RECEIVER (ПРИЁМНИК НА САМОЛЁТЕ)            │
│    Получает RF сигнал и выдает iBUS данные по UART          │
└────────────────────────┬────────────────────────────────────┘
                         │ UART (115200 baud, PIN GPIO3)
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                  ESP32-C3 MICROCONTROLLER                    │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ 1️⃣  IBusReceiver                                    │   │
│  │    • Парсит iBUS кадры из приёмника                │   │
│  │    • Преобразует в RcChannelState (10 каналов)     │   │
│  │    • Отслеживает потерю сигнала (failsafe)         │   │
│  └────────────┬─────────────────────────────────────────┘   │
│               │                                             │
│  ┌────────────▼─────────────────────────────────────────┐   │
│  │ 2️⃣  FlightController (ГЛАВНЫЙ КООРДИНАТОР)         │   │
│  │    • Вызывает все остальные компоненты             │   │
│  │    • Управляет приоритетом failsafe                │   │
│  │    • Координирует arming/disarming                 │   │
│  └────────────┬─────────────────────────────────────────┘   │
│               │                                             │
│  ┌────────────▼──────────┬──────────────┬───────────────┐   │
│  │                       │              │               │   │
│  │ 3️⃣  ArmingManager   │  3️⃣  Failsafe CHECK  │   │   │
│  │ Проверяет условия   │  (Signal lost?)     │   │   │
│  │ для ARM системы     └─────────────────────┘   │   │
│  └─────────────┬───────────────────────────────────┘   │
│                │                                       │
│  ┌─────────────▼──────────────────────────────────┐   │
│  │ 4️⃣  ControlMixer (АЭРОДИНАМИЧЕСКАЯ ЛОГИКА)   │   │
│  │    • Преобразует CH1, CH2, CH5 в:             │   │
│  │      - Left Aileron (левый элерон)           │   │
│  │      - Right Aileron (правый элерон)         │   │
│  │      - Elevator (лифт)                       │   │
│  │    • Применяет ограничения движения           │   │
│  └──────────────┬─────────────────────────────────┘   │
│                 │                                      │
│  ┌──────────────▼────────────────────────────────┐   │
│  │ 5️⃣  ThrottleManager (УПРАВЛЕНИЕ МОТОРОМ)   │   │
│  │    • Читает CH3 (Throttle)                  │   │
│  │    • Применяет ограничения мощности         │   │
│  │    • Управляет boost режимом                │   │
│  │    • При failsafe — прямо на failsafe       │   │
│  └──────────────┬────────────────────────────────┘   │
│                 │                                     │
│  ┌──────────────▼──────────────────────────────┐    │
│  │ 6️⃣  FlightOutputs (ВЫХОДНЫЕ СИГНАЛЫ PWM) │    │
│  │    • Отправляет PWM на GPIO пины            │    │
│  │    • Частота: 50 Hz (стандарт для Servo)    │    │
│  │    • Диапазон: 1000–2000 µs                 │    │
│  └──────────────┬──────────────────────────────┘    │
│                 │                                    │
└─────────────────┼────────────────────────────────────┘
                  │ PWM сигналы (50 Hz, 1-2 ms)
                  │
        ┌─────────┼─────────┬──────────┐
        │         │         │          │
        ▼         ▼         ▼          ▼
   ┌────────┐┌────────┐┌────────┐┌─────────┐
   │Left    ││Right   ││Elevator││ ESC     │
   │Aileron ││Aileron ││        ││ (Motor) │
   │Servo   ││Servo   ││Servo   ││ Control │
   └────────┘└────────┘└────────┘└─────────┘
        │         │         │          │
        │         │         │          │
        ▼         ▼         ▼          ▼
   ┌─────────────────────────────────────────┐
   │     САМОЛЁТ ПОЛУЧАЕТ УПРАВЛЕНИЕ 🛩️     │
   └─────────────────────────────────────────┘
```

---

## 🔄 Подробный поток данных (шаг за шагом)

### Шаг 1️⃣: ПРИЁМ СИГНАЛА ПРИЁМНИКОМ

```
┌─ Пульт отправляет iBUS кадр 2.4 GHz в эфир:
│
│  Кадр содержит 10 каналов:
│  CH1 = Aileron (элероны)      → Диапазон: 1000–2000 µs
│  CH2 = Elevator (лифт)        → Диапазон: 1000–2000 µs
│  CH3 = Throttle (газ)         → Диапазон: 1000–2000 µs
│  CH4 = Rudder (руль)          → Диапазон: 1000–2000 µs
│  CH5 = Flaps (закрылки)       → Диапазон: 1000–2000 µs
│  CH6-CH10 = Дополнительные    → Диапазон: 1000–2000 µs
│
└─ Приёмник декодирует кадр и выдает UART сигнал

   UART FORMAT:
   Baudrate:   115200 бод
   Data bits:  8
   Parity:     None
   Stop bits:  1
   PIN:        GPIO3 (RX1 на ESP32-C3)
```

**Код в `IBusReceiver.h`:**
```cpp
// IBusReceiver получает UART данные и парсит iBUS кадры
class IBusReceiver
{
    void update()  // Вызывается в loop() каждые 2ms
    {
        // Читаем все доступные байты из UART
        while (serial.available())
        {
            uint8_t byte = serial.read();
            processByte(byte);  // Парсим iBUS protokol
        }
    }

    // После полного кадра обновляется RcChannelState
    const RcChannelState& getState() const { return state; }
};
```

---

### Шаг 2️⃣: ГЛАВНЫЙ КОНТРОЛЛЕР (FlightController)

После получения данных от приёмника запускается главный координатор.

```cpp
// ============================================================
// ГЛАВНАЯ АРХИТЕКТУРА В MAIN.CPP
// ============================================================

void loop()
{
    // Главный объект координирует всё управление
    flightController.update();
    
    // Дебаг информация (отдельно)
    debugLogger.update();
    
    delay(2);  // Небольшая пауза
}
```

**Код в `FlightController.h`:**
```cpp
class FlightController
{
    void update()
    {
        // 1. Обновляем приёмник (получаем новые iBUS кадры)
        receiver.update();
        
        // 2. Проверяем, потеряли ли сигнал?
        const bool receiverFailsafe = receiver.isSignalLost();
        
        // 3. Получаем текущее состояние всех каналов
        const RcChannelState& rc = receiver.getState();
        
        // 4. ⚠️ FAILSAFE имеет АБСОЛЮТНЫЙ ПРИОРИТЕТ
        if (receiverFailsafe)
        {
            // При потере сигнала выставляем безопасные значения
            outputs.setFailsafe();  // Моторы выключены, сервы в ноль
            return;
        }
        
        // 5. Проверяем условия ARM системы (удержание газа в LOW)
        arming.update(rc.get(Channels::THROTTLE), receiverFailsafe);
        
        // 6. Считаем управляющие сигналы для поверхностей
        const FlightOutputState control = mixer.calculate(rc);
        
        // 7. Обновляем мотор (throttle с boost логикой)
        const uint16_t throttle = throttleManager.update(rc, receiverFailsafe);
        
        // 8. Отправляем PWM сигналы на сервоприводы и ESC
        outputs.update(control, throttle, arming.isArmed());
    }
};
```

---

### Шаг 3️⃣: ARMING MANAGER (Система вооружения)

Перед полётом самолёт нужно "вооружить" (ARM).

```
📋 Условия для ARM:
   ┌─────────────────────────────────────┐
   │ Газ (CH3) должен быть на МИНИМУМЕ   │
   │ (менее 1000 µs) в течение 2 секунд │
   │                                     │
   │ После этого система вооружена 🔓   │
   │ и готова к полёту                  │
   └─────────────────────────────────────┘
```

**Код в `ArmingManager.h`:**
```cpp
class ArmingManager
{
    void update(uint16_t throttle, bool receiverFailsafe)
    {
        // Потеря сигнала → DISARM
        if (receiverFailsafe)
        {
            armed = false;
            throttleLowSince = 0;
            return;
        }
        
        // Газ в минимуме?
        if (throttle < Config::THROTTLE_LOW_US)  // 1000 µs
        {
            // Засекаем время удержания газа на минимуме
            if (throttleLowSince == 0)
                throttleLowSince = millis();
            
            // Если газ низко 2 сек → ARMED!
            if (millis() - throttleLowSince >= Config::ARM_LOW_TIME_MS)
                armed = true;
        }
        else
        {
            // Газ поднят → сброс таймера
            throttleLowSince = 0;
        }
    }
    
    bool isArmed() const { return armed; }
};
```

---

### Шаг 4️⃣: FAILSAFE (Система безопасности)

Если сигнал потерян — система переходит в failsafe режим.

```
⚠️ ЧТО ПРОИСХОДИТ ПРИ FAILSAFE:

   Потеря сигнала (> 500ms без кадров)
            │
            ▼
   receiverFailsafe = true
            │
            ▼
   ┌───────────────────────────────────┐
   │ Все управляющие сигналы           │
   │ выставляются на НОЛЬ (нейтраль)  │
   │                                   │
   │ CH1-CH4 → 1500 µs (ноль)          │
   │ CH3 (Throttle) → 1000 µs (MIN)    │
   │ Motor → Выключен (failsafe)       │
   │ Servo → Нейтраль                  │
   │                                   │
   │ ✅ Самолёт теряет управление     │
   │    и должен планировать            │
   └───────────────────────────────────┘
```

---

### Шаг 5️⃣: CONTROL MIXER (Аэродинамическая обработка)

Преобразует RC inputs в управляющие сигналы для поверхностей.

```
INPUT CHANNELS:
┌────────────────────────────────────────┐
│ CH1 (Aileron)  → ±500 µs от центра     │
│ CH2 (Elevator) → ±500 µs от центра     │
│ CH3 (Throttle) → 0...1000 µs (минус)  │
│ CH5 (Flaps)    → 3 положения          │
└────────────────────────────────────────┘

MIXER LOGIC:
┌──────────────────────────────────────────────────┐
│                                                  │
│  LEFT AILERON = Aileron + Flaps Offset          │
│  RIGHT AILERON = -Aileron + Flaps Offset        │
│  ELEVATOR = Elevator                            │
│                                                  │
│  Логика:                                        │
│  • Если CH5 < 1250 µs → Flaps = 0 µs            │
│  • Если 1250..1750  → Flaps = 50 µs             │
│  • Если CH5 >= 1750 → Flaps = 100 µs            │
│                                                  │
└──────────────────────────────────────────────────┘

OUTPUT SURFACES:
┌────────────────────────────────────────┐
│ Left Aileron   → GPIO4  (Servo 1)      │
│ Right Aileron  → GPIO5  (Servo 2)      │
│ Elevator       → GPIO6  (Servo 3)      │
└────────────────────────────────────────┘
```

**Код в `ControlMixer.h`:**
```cpp
class ControlMixer
{
    FlightOutputState calculate(const RcChannelState& rc) const
    {
        FlightOutputState output;
        
        // Получаем входные сигналы
        uint16_t aileronInput = rc.get(Channels::AILERON);
        uint16_t elevatorInput = rc.get(Channels::ELEVATOR);
        uint16_t flapsInput = rc.get(Channels::FLAPS);
        
        // Преобразуем в ±500 µs отклонения
        int16_t aileron = RcInput::centered(
            aileronInput,
            Config::AILERON_MAX_US,
            false
        );
        
        int16_t elevator = RcInput::centered(
            elevatorInput,
            Config::ELEVATOR_MAX_US,
            false
        );
        
        // Рассчитываем положение закрылков (0, 50, 100 µs)
        uint16_t flapOffset = calculateFlapOffset(flapsInput);
        
        // LEFT AILERON = Aileron + Flaps
        output.leftAileron = 1500 + aileron + flapOffset;
        
        // RIGHT AILERON = -Aileron + Flaps (эффект кровельки)
        output.rightAileron = 1500 - aileron + flapOffset;
        
        // ELEVATOR = просто лифт
        output.elevator = 1500 + elevator;
        
        return output;
    }
};
```

---

### Шаг 6️⃣: THROTTLE MANAGER (Управление мотором)

Управляет двигателем, включая режим boost.

```
📊 THROTTLE LOGIC:

   CH3 Range: 1000-2000 µs
            │
            ├─ 1000-1250 µs → Минимум (0%)
            ├─ 1250-1750 µs → Линейно (0%..100%)
            └─ 1750-2000 µs → Максимум (100%)

   BOOST MODE:
   ┌─────────────────────────────────────┐
   │ CH6 < 1250 µs  → Boost РАЗБЛОКИРОВАН │
   │ CH6 >= 1750 µs → Boost АКТИВЕН      │
   │                                     │
   │ При активном boost:                 │
   │ Throttle += Config::THROTTLE_BOOST  │
   │ (максимум 5 секунд)                 │
   │                                     │
   │ После истечения таймера boost       │
   │ нужно вернуть CH6 в LOW              │
   │ чтобы повторно активировать boost   │
   └─────────────────────────────────────┘
```

**Код в `ThrottleManager.h`:**
```cpp
class ThrottleManager
{
    uint16_t update(const RcChannelState& rc, bool receiverFailsafe)
    {
        // При failsafe → прямо на безопасное значение
        if (receiverFailsafe)
        {
            boostActive = false;
            return Config::FAILSAFE_THROTTLE;  // Обычно 1000 µs
        }
        
        // Читаем gas из CH3
        uint16_t throttle = RcInput::clamp(
            rc.get(Channels::THROTTLE)
        );
        
        // Читаем boost switch из CH6
        uint16_t boostSwitch = rc.get(Channels::BOOST);
        
        // LOW на switch → разблокируем boost
        if (boostSwitch < 1250)
        {
            boostReady = true;
        }
        
        // HIGH на switch + разблокирован → активируем boost
        if (boostSwitch >= 1750 && boostReady && !boostActive)
        {
            boostActive = true;
            boostReady = false;
            boostStartTime = millis();
        }
        
        // Проверяем таймер (5 секунд)
        if (boostActive)
        {
            if (millis() - boostStartTime >= Config::THROTTLE_BOOST_TIME_MS)
            {
                boostActive = false;
            }
        }
        
        // Если boost активен → добавляем мощность
        if (boostActive)
        {
            throttle = min(
                throttle + Config::THROTTLE_BOOST,
                Config::PWM_MAX  // Не выше максимума
            );
        }
        
        return throttle;
    }
};
```

---

### Шаг 7️⃣: FLIGHT OUTPUTS (Отправка PWM сигналов)

Финальный этап — отправка PWM сигналов на сервоприводы и ESC.

```
🔌 PWM SIGNAL FORMAT:

   Частота:    50 Hz (период 20 ms)
   Длительность импульса: 1000–2000 µs
   
   Стандартная кодировка:
   ┌─────────────────────────────────────┐
   │ 1000 µs → Минимум положение (-100%) │
   │ 1500 µs → Нейтраль (0%)             │
   │ 2000 µs → Максимум положение (+100%)│
   └─────────────────────────────────────┘

   СЕРВОПРИВОДЫ:
   GPIO4 (PWM) ──────→ Left Aileron Servo  (MG90S)
   GPIO5 (PWM) ──────→ Right Aileron Servo (MG90S)
   GPIO6 (PWM) ──────→ Elevator Servo      (MG90S)
   GPIO7 (PWM) ──────→ ESC Motor Control   (40A BLHeli)

   🎚️ ESC специфично:
   Принимает PWM сигнал и преобразует в трёхфазный
   сигнал для бесщёточного двигателя.
```

**Код в `FlightOutputs.h`:**
```cpp
class FlightOutputs
{
    // Hardware servo objects (используют ESP32Servo библиотеку)
    ESP32Servo aileronLeft, aileronRight, elevator, esc;
    
    bool begin()
    {
        // Выделяем 4 hardware timer'а для PWM
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);
        
        // Устанавливаем частоту 50 Hz (стандарт для сервоприводов)
        aileronLeft.setPeriodHertz(50);
        aileronRight.setPeriodHertz(50);
        elevator.setPeriodHertz(50);
        esc.setPeriodHertz(50);
        
        // Подключаем сервоприводы к GPIO пинам
        aileronLeft.attach(GPIO4, 1000, 2000);    // MIN, MAX PWM
        aileronRight.attach(GPIO5, 1000, 2000);
        elevator.attach(GPIO6, 1000, 2000);
        esc.attach(GPIO7, 1000, 2000);
        
        return true;  // Если всё успешно
    }
    
    void update(
        const FlightOutputState& control,
        uint16_t throttle,
        bool armed
    )
    {
        // Отправляем сигналы на сервоприводы (в µs)
        aileronLeft.writeMicroseconds(control.leftAileron);
        aileronRight.writeMicroseconds(control.rightAileron);
        elevator.writeMicroseconds(control.elevator);
        
        // ESC получает throttle сигнал
        esc.writeMicroseconds(throttle);
    }
    
    void setFailsafe()
    {
        // При failsafe всё на нейтраль/минимум
        aileronLeft.writeMicroseconds(1500);
        aileronRight.writeMicroseconds(1500);
        elevator.writeMicroseconds(1500);
        esc.writeMicroseconds(1000);  // Мотор выключен
    }
};
```

---

## 📋 Каналы RC Приёмника (10 каналов iBUS)

```
CHANNEL MAPPING (src/main.cpp):

┌─────────────────────────────────────────────────────────────┐
│                   RC CHANNEL CONFIGURATION                   │
├─────────────────────────────────────────────────────────────┤
│ CH1 (Aileron)      → Управление креном (Roll)              │
│     Диапазон: 1000–2000 µs                                  │
│     Эффект: ±500 µs отклонение элеронов                     │
│     Пульт: Stick LEFT/RIGHT                                 │
│                                                             │
│ CH2 (Elevator)     → Управление тангажом (Pitch)           │
│     Диапазон: 1000–2000 µs                                  │
│     Эффект: ±500 µs отклонение лифта                       │
│     Пульт: Stick UP/DOWN                                    │
│                                                             │
│ CH3 (Throttle)     → Управление двигателем                 │
│     Диапазон: 1000–2000 µs                                  │
│     Эффект: 0%–100% мощности мотора                        │
│     Пульт: Throttle slider LEFT/RIGHT или throttle hold   │
│                                                             │
│ CH4 (Rudder)       → Управление рысканием (Yaw)            │
│     Диапазон: 1000–2000 µs                                  │
│     ⚠️ СЕЙЧАС НЕ ИСПОЛЬЗУЕТСЯ                              │
│     Пульт: Stick LEFT/RIGHT (нижний)                        │
│                                                             │
│ CH5 (Flaps)        → Положение закрылков/флапов            │
│     Диапазон: 1000–2000 µs                                  │
│     Логика:                                                │
│       < 1250 µs  → Flaps = 0 µs (убраны)                   │
│       1250–1750  → Flaps = 50 µs (полусбросок)            │
│       > 1750 µs  → Flaps = 100 µs (выпущены)              │
│     Пульт: Switch 3-position                               │
│                                                             │
│ CH6 (Boost)        → Активация режима boost (ускорение)    │
│     Диапазон: 1000–2000 µs                                  │
│     Логика:                                                │
│       < 1250 µs  → Boost РАЗБЛОКИРОВАН (готов)            │
│       > 1750 µs  → Boost АКТИВЕН (максимум 5 сек)         │
│     Эффект: Throttle += 250 µs (максимум 5 секунд)        │
│     Пульт: Switch 2-position (Arm/Disarm)                  │
│                                                             │
│ CH7–CH10           → Дополнительные каналы                 │
│     ⚠️ ЗАРЕЗЕРВИРОВАНЫ для будущих систем:               │
│     • Автопилот                                           │
│     • Второй мотор                                         │
│     • Дополнительные серво                                 │
│     • Системные флаги                                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 🔧 Конфигурация (include/Config.h)

```cpp
// Основные пины ESP32-C3

#define PIN_IBUS          GPIO3      // UART RX (приёмник iBUS)
#define PIN_AILERON_LEFT  GPIO4      // PWM left aileron servo
#define PIN_AILERON_RIGHT GPIO5      // PWM right aileron servo
#define PIN_ELEVATOR      GPIO6      // PWM elevator servo
#define PIN_ESC           GPIO7      // PWM ESC motor control

// iBUS протокол
#define IBUS_BAUDRATE     115200     // Скорость UART
#define IBUS_FRAME_LENGTH 32         // Длина кадра iBUS
#define RX_TIMEOUT_US     500000     // Timeout потери сигнала (500ms)

// PWM сигналы
#define PWM_MIN           1000       // Минимум PWM (µs)
#define PWM_MAX           2000       // Максимум PWM (µs)
#define PWM_CENTER        1500       // Нейтраль (µs)

// Arming логика
#define THROTTLE_LOW_US   1000       // Минимум газа для ARM
#define ARM_LOW_TIME_MS   2000       // Время удержания газа (2 сек)

// Failsafe
#define FAILSAFE_THROTTLE 1000       // Мотор выключен при failsafe

// Boost режим
#define THROTTLE_BOOST    250        // Дополнительно µs при boost
#define THROTTLE_BOOST_TIME_MS 5000  // Длительность boost (5 сек)

// Диапазоны серв
#define AILERON_MAX_US    500        // Максимальное отклонение элерона
#define ELEVATOR_MAX_US   500        // Максимальное отклонение лифта
```

---

## 🧪 Пример полного цикла управления

```
ВРЕМЕННАЯ ШКАЛА (каждые 2ms в loop()):

t=0ms:   loop() → flightController.update()
         │
         ├─ receiver.update()
         │  • Читаем UART кадр от приёмника
         │  • Парсим iBUS протокол
         │  • Обновляем RcChannelState (10 каналов)
         │
         ├─ Проверяем failsafe
         │  • isSignalLost()? (если > 500ms без кадров)
         │
         ├─ arming.update(throttle, failsafe)
         │  • Газ низко 2 сек? → Armed ✓
         │
         ├─ control = mixer.calculate(rc)
         │  • CH1 → Left Aileron = 1400 µs
         │  • CH1 → Right Aileron = 1600 µs
         │  • CH2 → Elevator = 1450 µs
         │
         ├─ throttle = throttleManager.update(rc, failsafe)
         │  • CH3 = 1300 µs (30% мощности)
         │  • Boost? Нет.
         │  • Output: 1300 µs
         │
         └─ outputs.update(control, throttle, armed)
            • Left Aileron Servo ← 1400 µs
            • Right Aileron Servo ← 1600 µs
            • Elevator Servo ← 1450 µs
            • ESC (Motor) ← 1300 µs

t=2ms:   loop() повторяется снова (50 Hz)
         ...
```

---

## 📡 iBUS Protocol (коротко)

```
iBUS Frame Format:

[HEAD][LEN][CH1-L][CH1-H]...[CH10-L][CH10-H][CHECKSUM-L][CHECKSUM-H]

HEAD        = 0x20            (1 байт, маркер начала)
LEN         = 0x40            (1 байт, длина данных, 64 байта)

CH1-CH10    = 2 байта каждый (Low byte, High byte)
              Каждый канал = 16-bit number (0x03E8...0x07D0)
              В микросекундах: 1000...2000 µs

CHECKSUM    = 2 байта         (контроль целостности кадра)

Пример кадра для CH1=1500, CH2=1500, ...:
20 40 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 XX XX
```

---

## ✅ Контрольный список для понимания

- [ ] Я понимаю, как пульт отправляет сигнал → приёмник → ESP32
- [ ] Я знаю 10 RC каналов и их назначение
- [ ] Я понимаю, что такое failsafe и когда он активируется
- [ ] Я знаю, как система вооружается (ARM логика)
- [ ] Я понимаю, как mixer преобразует RC inputs в серво сигналы
- [ ] Я знаю, как работает boost режим
- [ ] Я понимаю, что такое PWM 50Hz и диапазон 1000–2000 µs
- [ ] Я знаю, какие GPIO пины используются для каждого серво/ESC

---

## 🚀 Следующие шаги для расширения

Эта архитектура готова для добавления:

```
В будущем можно легко добавить:

📊 IMU (Инерциальное измерительное устройство)
   • MPU6050, BNO055
   • Даст gyroscope, accelerometer, compass

🛰️ GPS (Навигация)
   • NEO-6M, NEO-M8N
   • Координаты, высота, скорость

🤖 Autopilot (Автопилот)
   • Возьмёт mixer output
   • Добавит свои корректировки
   • Выдаст в outputs

📡 Telemetry (Телеметрия)
   • Отправляет данные обратно на пульт
   • Батарея, высота, скорость, GPS

🎮 GUI (Графический интерфейс)
   • Конфигурация параметров
   • Калибровка серв
   • Логирование полёта

⚙️ Flight Modes (Режимы полёта)
   • MANUAL (ручное управление)
   • STABILIZE (стабилизация)
   • AUTONOMOUS (полная автономия)

Все эти системы смогут работать независимо,
потому что интерфейсы хорошо разделены!
```

---

## 📚 Обозначения в коде

```
µs  = микросекунда (1/1,000,000 секунды)
ms  = миллисекунда (1/1,000 секунды)
Hz  = Герц (кол-во циклов в секунду)
PWM = Pulse Width Modulation (модуляция ширины импульса)

RC  = Radio Control (радиоуправление)
CH  = Channel (канал)
ESC = Electronic Speed Controller (регулятор оборотов)

ARM = Вооружение системы (готовность к полёту)
ARM vs DISARM = Вооружена vs Разоружена
failsafe = Безопасный режим при потере сигнала
```

---

## 📞 Общение компонентов

```
Пример сообщения между компонентами:

    IBusReceiver
    "Получил: CH1=1000, CH2=1500, CH3=1800"
           │
           ▼
    FlightController
    "Начинаю обработку"
           │
           ├─→ ArmingManager
           │   "Throttle = 1000 (LOW) 2 сек? → Armed!"
           │
           ├─→ ControlMixer
           │   "CH1=1000 → aileron = -500, CH2=1500 → elevator = 0"
           │   "Результат: левый элерон = 1000, правый = 2000"
           │
           ├─→ ThrottleManager
           │   "CH3=1800 → 80% мощности = 1800 µs"
           │
           └─→ FlightOutputs
               "Отправляю PWM сигналы:"
               "• Left Aileron ← 1000 µs"
               "• Right Aileron ← 2000 µs"
               "• Elevator ← 1500 µs"
               "• ESC ← 1800 µs"
                      │
                      ▼
                  🛩️ САМОЛЁТ УПРАВЛЯЕТСЯ!
```

---

**Создано для OpenPlaneProject**

Этот документ должен помочь любому программисту понять, как сигнал проходит от пульта до самолёта. Если что-то не ясно — смотрите код в соответствующих `.h` файлах, там много комментариев! 🚀
