# 🚀 Autopilot Integration Guide

## Overview

Система автопилота расширяет OpenPlaneProject с поддержкой:
- **Гиро-стабилизации** (удержание углов крена/тангажа)
- **Автоматического взлёта** с управлением тангажом
- **Удержания высоты** через барометр
- **Гибкого назначения каналов** без переписывания кода
- **Веб-отладки** через Wi-Fi (обзор датчиков, туёвша режимов)

## Architecture

```
RC Receiver (10 channels)
    ↓
FlightController
    ├─→ IBusReceiver (parsing)
    ├─→ FeatureManager (channels 7-10 mapping)
    │   └─→ Autopilot (modes: MANUAL, STABILIZE, AUTO_TAKEOFF, ALT_HOLD)
    │       ├─→ ImuSensor (MPU6050: roll, pitch, yaw)
    │       └─→ BarometerSensor (BME280: altitude, climb rate)
    ├─→ ControlMixer + Autopilot corrections
    ├─→ ThrottleManager
    └─→ FlightOutputs (PWM to servos/motor)
    ↓
WebDebugServer (optional)
    ├─→ JSON APIs
    └─→ HTML Dashboard
```

## Hardware Connections

### Sensors (I2C Bus)
- **SCL**: GPIO22 (clock)
- **SDA**: GPIO21 (data)
- **Pullup resistors**: 4.7kΩ (если не встроены в датчик)

### I2C Addresses
- MPU6050: **0x68** (AD0=GND, default) или 0x69 (AD0=VCC)
- BME280: **0x76** (SDO=GND, default) или 0x77 (SDO=VCC)

### Wiring Example
```
ESP32-C3          MPU6050         BME280
=====================================
GPIO21 (SDA) ←→ SDA            SDA
GPIO22 (SCL) ←→ SCL            SCL
GND          ←→ GND            GND
3.3V         ←→ VCC            VCC
             ←→ AD0=GND        SDO=GND
```

## PlatformIO Configuration

### platformio.ini

```ini
[env:esp32-c3-supermini]
; ... existing settings ...

lib_deps =
    ; Existing
    ESP32Servo
    espressif/esp-idf-lib
    
    ; New for sensors
    ; (используем встроенный Wire для I2C)
    
    ; For WiFi (встроено в ESP32)
    ; WiFi library is built-in
```

## Code Integration

### main.cpp Example

```cpp
#include <Arduino.h>
#include "FlightController.h"
#include "sensors/MPU6050_Sensor.h"
#include "sensors/BME280_Sensor.h"
#include "Autopilot.h"
#include "FeatureManager.h"
#include "WebDebugServer.h"

// Инициализация компонентов
// ... existing code ...

// Датчики
MPU6050_Sensor imuSensor(0x68);      // Gyro + Accel
BME280_Sensor baroSensor(0x76);      // Altitude

// Автопилот
Autopilot autopilot(&imuSensor, &baroSensor);

// Менеджер функций (CH7-CH10)
FeatureManager featureManager(&autopilot);

// Веб-сервер отладки
WebDebugServer webServer(&flightController, &autopilot, &featureManager);

void setup()
{
    // ... existing initialization ...
    
    // Инициализируем датчики
    if (!imuSensor.begin())
    {
        Serial.println("❌ IMU sensor failed to initialize!");
        // Но не выходим - система может работать без датчиков
    }
    
    if (!baroSensor.begin())
    {
        Serial.println("❌ Barometer sensor failed to initialize!");
    }
    
    // Инициализируем автопилот
    if (!autopilot.begin())
    {
        Serial.println("⚠️  Autopilot initialization warning");
    }
    
    // Инициализируем менеджер функций
    if (!featureManager.begin())
    {
        Serial.println("⚠️  Feature manager initialization warning");
    }
    
    // Калибруем датчики (датчик должен быть неподвижен)
    Serial.println("🔧 Calibrating sensors...");
    if (imuSensor.isAvailable())
    {
        imuSensor.calibrate();  // ~2 сек
    }
    if (baroSensor.isAvailable())
    {
        baroSensor.calibrateAltitude();  // ~1 сек
    }
    
    // Инициализируем веб-сервер
    if (!webServer.begin(0))  // 0 = AP mode
    {
        Serial.println("⚠️  WebServer failed to start");
    }
    
    // Запускаем main flight controller
    flightController.begin();
}

void loop()
{
    // Основной цикл управления полётом
    flightController.update();
    
    // Обновляем автопилот
    autopilot.update();
    
    // Обновляем менеджер функций
    if (!ibusReceiver.isSignalLost())
    {
        featureManager.update(rcState);
    }
    
    // Обновляем веб-сервер (обработка HTTP запросов)
    webServer.update();
    
    // Логирование (опционально)
    debugLogger.update();
    
    delay(2);  // 500 Hz main loop
}
```

## Configurating Features

### Default Mapping

По умолчанию:
- **CH7**: AUTO_TAKEOFF - автоматический взлёт
- **CH8**: ALT_HOLD - удержание высоты
- **CH9**: STABILIZE - гиро-стабилизация
- **CH10**: MANUAL - ручное управление

### Change Mapping at Runtime

```cpp
// Переназначить функции
featureManager.assignFeature(7, FEATURE_ALT_HOLD);
featureManager.assignFeature(8, FEATURE_STABILIZE);

// Сохранить конфигурацию
featureManager.saveConfiguration();
```

### RC Channel Logic

Для каждого канала:
- **< 1500 µs**: Функция выключена (OFF)
- **≥ 1500 µs**: Функция включена (ON)

Пример использования 3-позиционного переключателя:
```
Pos 1 (1000 µs) → OFF
Pos 2 (1500 µs) → ON (граница)
Pos 3 (2000 µs) → ON
```

## Autopilot Modes

### 1. MANUAL (MODE_MANUAL)
- **Включение**: CH10 > 1500 µs или по умолчанию
- **Описание**: Полностью управление с пульта
- **Автопилот вмешивается**: Никогда
- **Использование**: Обычное пилотирование

### 2. STABILIZE (MODE_STABILIZE)
- **Включение**: CH9 > 1500 µs
- **Описание**: Гиро-стабилизация углов крена и тангажа
- **ПИД контроллеры**: Roll и Pitch стремятся вернуться к 0°
- **Эффект**: Самолёт возвращается в уровень автоматически
- **Использование**: Для безопасного пилотирования неопытным пилотом

### 3. AUTO_TAKEOFF (MODE_AUTO_TAKEOFF)
- **Включение**: CH7 > 1500 µс
- **Фазы**:
  1. 0-1 сек: Разгон на земле (30% газа горизонтально)
  2. 1-3 сек: Взлёт (60% газа, 15° тангаж вверх)
  3. 3+ сек: Набор высоты (100% газа, 10° тангаж)
- **Стабилизация**: Крен удерживается на 0°, тангаж по плану
- **Использование**: Полностью автоматический взлёт

### 4. ALT_HOLD (MODE_ALT_HOLD)
- **Включение**: CH8 > 1500 µс
- **Описание**: Удержание текущей высоты
- **ПИД контроллер**: Throttle стремится держать текущую высоту
- **Условие**: Должна быть предварительная скорость (иначе дросель будет уменьшаться)
- **Использование**: После взлёта, для стабильного полёта на определённой высоте

## PID Tuning

### Структура ПИД контроллера

```
Output = Kp * error + Ki * integral(error) + Kd * derivative(error)
```

### Текущие коэффициенты (в Autopilot.h)

```cpp
// Roll стабилизация
pidRoll.setGains(0.05f, 0.01f, 0.02f);   // Kp=0.05, Ki=0.01, Kd=0.02

// Pitch стабилизация
pidPitch.setGains(0.05f, 0.01f, 0.02f);  // Kp=0.05, Ki=0.01, Kd=0.02

// Altitude hold
pidThrottle.setGains(0.1f, 0.05f, 0.01f); // Kp=0.1, Ki=0.05, Kd=0.01
```

### Настройка (простой подход)

1. **Начните с Kp только** (Ki=0, Kd=0)
   - Увеличивайте Kp пока не появится небольшое дрожание
   - Затем немного уменьшите

2. **Добавьте Ki** для устранения смещения
   - Начните с Ki = Kp / 10
   - Это добавляет интеграл ошибки со временем

3. **Добавьте Kd** для демпфирования
   - Начните с Kd = Kp / 2
   - Это уменьшает дрожание

### Настройка через Web UI

Текущая версия показывает коэффициенты в консоли.
Для реальной настройки через Web UI отредактируйте handleSetPID() в WebDebugServer.h.

## Web Debug Server

### Доступ

1. Включите самолёт (ESP32 включится)
2. Откройте Wi-Fi на телефоне/ноутбуке
3. Найдите сеть **OpenPlane-Debug**
4. Пароль: **12345678**
5. Откройте браузер: `http://192.168.4.1`

### Дашборд показывает

- **📊 Датчики**: Roll, Pitch, Yaw, Altitude, Climb Rate
- **🚀 Autopilot**: Current mode, desired angles, target altitude
- **🎛️ Features**: Status каждого канала (CH7-CH10)
- **⚙️ Controls**: Кнопки для переключения режимов

### API Endpoints

```
GET /api/sensors
→ {"roll": 0.00, "pitch": 0.00, "yaw": 0.00, "altitude": 0.00, "climb": 0.00}

GET /api/autopilot
→ {"mode": 0, "desired_roll": 0.0, "desired_pitch": 0.0, "target_alt": 0.0}

GET /api/features
→ {"ch7": "AUTO_TAKEOFF", "ch7_active": false, "ch8": "ALT_HOLD", ...}

POST /api/setmode
→ Body: {"mode": 1}
→ Response: {"status": "ok"}
```

## Sensor Calibration

### IMU (MPU6050)

Обязательно выполнить перед полётом!

```cpp
// Автоматическая калибровка в setup()
imuSensor.calibrate();  // Держите самолёт неподвижно, ~2 сек

// Или вручную:
// 1. Положите самолёт на ровную поверхность
// 2. Вызовите imuSensor.calibrate()
// 3. Дождитесь завершения
```

**Что делает калибровка:**
- Измеряет смещение (bias) гироскопа
- Сохраняет значение при нулевой угловой скорости
- Вычитает смещение из всех последующих измерений

### Barometer (BME280)

```cpp
// Автоматическая калибровка в setup()
baroSensor.calibrateAltitude();  // Держите на земле, ~1 сек

// Или вручную:
// 1. Включите систему на земле (в месте взлёта)
// 2. Вызовите baroSensor.calibrateAltitude()
// 3. Высота 0 м будет сохранена как базовая
```

**Что делает калибровка:**
- Измеряет начальную высоту
- Все последующие высоты рассчитываются относительно этой точки
- Позволяет использовать относительную высоту вместо абсолютной

## Troubleshooting

### Датчики не инициализируются

**Проблема**: `❌ BME280: No response from sensor!`

**Решение**:
1. Проверьте I2C адреса (используйте I2C сканер)
2. Проверьте провода (SDA/SCL)
3. Проверьте напряжение 3.3V на датчиках
4. Добавьте 4.7kΩ pull-up резисторы между SDA/SCL и 3.3V

### Углы прыгают / Дрожание

**Проблема**: Roll/Pitch постоянно меняются

**Решение**:
1. Проверьте калибровку IMU
2. Уменьшите Kp в PID контроллере
3. Добавьте Kd для демпфирования
4. Убедитесь, что самолёт может свободно двигаться

### Автопилот не реагирует

**Проблема**: Режим не меняется при переключении каналов

**Решение**:
1. Проверьте значения RC каналов (через Serial monitor)
2. Убедитесь, что Channel > 1500 µs для включения функции
3. Проверьте, что FeatureManager инициализирован
4. Проверьте логи в консоли

### Высота не удерживается

**Проблема**: Самолёт постоянно поднимается или падает

**Решение**:
1. Проверьте калибровку барометра
2. Проверьте PID коэффициенты (уменьшите Kp)
3. Убедитесь, что самолёт имеет скорость перед включением ALT_HOLD
4. Допустите больший диапазон колебаний (±1-2 м)

## Performance Considerations

### CPU Usage
- **Sensor updates**: ~1% (200 Hz × 3 sensors)
- **Autopilot PID**: ~2% (3 PID controllers × 200 Hz)
- **Web server**: ~5% (зависит от клиентов)
- **Total**: ~8% (нормально для 500 Hz main loop)

### Memory
- **Sensor classes**: ~1 KB each
- **Autopilot class**: ~2 KB
- **Feature Manager**: ~1 KB
- **Web server**: ~5 KB (HTML, buffers)
- **Total**: ~10 KB (достаточно для ESP32-C3)

### Latency
- **IMU update**: <5 ms
- **Barometer update**: <10 ms
- **PID calculation**: <1 ms per controller
- **Control output**: <2 ms
- **Total loop**: ~20 ms (50 Hz, достаточно)

## Next Steps

1. **Интегрируйте в main.cpp** следуя примеру выше
2. **Настройте PID коэффициенты** для вашего самолёта
3. **Проведите тесты в симуляторе** (Gazebo, CoppeliaSim)
4. **Летные тесты** начните с режима STABILIZE
5. **Постепенно добавляйте** AUTO_TAKEOFF и ALT_HOLD

## Support

Если возникли проблемы:
1. Проверьте консоль (Serial output)
2. Используйте веб-дашборд для визуализации
3. Проверьте это руководство в разделе Troubleshooting
4. Изучите логи DebugLogger (каждые 500 мс)
