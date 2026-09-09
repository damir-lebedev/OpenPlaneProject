#pragma once

// ============================================================
// 🚀 AUTOPILOT SYSTEM
//
// Основные режимы:
//   1. MANUAL - полностью управление с пульта
//   2. STABILIZE - гиро-стабилизация (удерживает углы крена/тангажа)
//   3. AUTO_TAKEOFF - автоматический взлёт с разгоном
//   4. ALT_HOLD - удержание высоты (после набора скорости)
//
// Архитектура:
//   • Режимы включаются через Feature Manager (каналы CH7-CH10)
//   • PID контроллеры для стабилизации (roll/pitch)
//   • Интеграция с IMU (гироскоп) и барометром (высота)
//   • Выход: коррекции к ControlMixer
// ============================================================

#include "sensors/SensorInterface.h"
#include "Config.h"
#include <Arduino.h>

// ============================================================
// PID КОНТРОЛЛЕР
// ============================================================

class PID_Controller
{
public:

    PID_Controller(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f)
        : Kp(kp), Ki(ki), Kd(kd),
          errorSum(0), lastError(0), lastTime(0),
          minOutput(-500), maxOutput(500)
    {
    }

    // ========================================================
    // УСТАНОВКА КОЭФФИЦИЕНТОВ
    // ========================================================

    void setGains(float kp, float ki, float kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    // ========================================================
    // УСТАНОВКА ЛИМИТОВ ВЫВОДА
    // ========================================================

    void setLimits(float minOut, float maxOut)
    {
        minOutput = minOut;
        maxOutput = maxOut;
    }

    // ========================================================
    // РАСЧЁТ ПИД ВЫВОДА
    // ========================================================
    // setpoint - желаемое значение (например, желаемый угол крена)
    // feedback - текущее значение (например, текущий угол крена)
    // returns: коррекция сигнала (-500 ... +500 µs)

    float calculate(float setpoint, float feedback)
    {
        unsigned long now = micros();
        float dt = (now - lastTime) / 1000000.0f;
        lastTime = now;

        // Защита от больших прыжков по времени
        if (dt < 0.001f || dt > 1.0f)
        {
            dt = 0.002f;  // 2 ms (стандартный цикл)
        }

        // Ошибка между желаемым и текущим значением
        float error = setpoint - feedback;

        // P (пропорциональный) термин
        float P = Kp * error;

        // I (интегральный) термин
        errorSum += error * dt;
        // Ограничиваем windup (переполнение интегратора)
        if (errorSum > 100) errorSum = 100;
        if (errorSum < -100) errorSum = -100;
        float I = Ki * errorSum;

        // D (дифференциальный) термин
        float dError = (error - lastError) / dt;
        float D = Kd * dError;

        lastError = error;

        // Сумма всех компонент
        float output = P + I + D;

        // Ограничиваем выход
        if (output > maxOutput) output = maxOutput;
        if (output < minOutput) output = minOutput;

        return output;
    }

    // ========================================================
    // СБРОС КОНТРОЛЛЕРА
    // ========================================================

    void reset()
    {
        errorSum = 0;
        lastError = 0;
        lastTime = micros();
    }

private:
    float Kp, Ki, Kd;
    float errorSum;
    float lastError;
    unsigned long lastTime;
    float minOutput, maxOutput;
};

// ============================================================
// РЕЖИМЫ АВТОПИЛОТА
// ============================================================

enum AutopilotMode
{
    MODE_MANUAL = 0,       // Полностью управление с пульта
    MODE_STABILIZE = 1,    // Стабилизация углов
    MODE_AUTO_TAKEOFF = 2, // Автоматический взлёт
    MODE_ALT_HOLD = 3      // Удержание высоты
};

// ============================================================
// ОСНОВНОЙ КЛАСС АВТОПИЛОТА
// ============================================================

class Autopilot
{
public:

    // ========================================================
    // КОНСТРУКТОР
    // ========================================================
    // Принимает указатели на датчики

    Autopilot(ImuSensor* imu = nullptr, BarometerSensor* baro = nullptr)
        : imuSensor(imu),
          baroSensor(baro),
          currentMode(MODE_MANUAL),
          previousMode(MODE_MANUAL),
          modeChangeTime(0),
          desiredRoll(0),
          desiredPitch(0),
          autoTakeoffThrottle(0),
          autoTakeoffStartTime(0),
          targetAltitude(0)
    {
        // Инициализируем PID контроллеры для крена и тангажа
        // Эти коэффициенты можно настраивать через Web UI
        pidRoll.setGains(0.05f, 0.01f, 0.02f);   // Kp, Ki, Kd для крена
        pidPitch.setGains(0.05f, 0.01f, 0.02f);  // Kp, Ki, Kd для тангажа
        pidThrottle.setGains(0.1f, 0.05f, 0.01f); // Для удержания высоты

        pidRoll.setLimits(-500, 500);
        pidPitch.setLimits(-500, 500);
        pidThrottle.setLimits(-100, 100);
    }

    // ========================================================
    // ИНИЦИАЛИЗАЦИЯ
    // ========================================================

    bool begin()
    {
        if (!imuSensor || !baroSensor)
        {
            Serial.println("❌ Autopilot: IMU or Barometer not attached!");
            return false;
        }

        Serial.println("✅ Autopilot: Initialized");
        return true;
    }

    // ========================================================
    // ОБНОВЛЕНИЕ АВТОПИЛОТА
    // ========================================================
    // Должна вызваться из FlightController::update()

    void update()
    {
        // Обновляем датчики
        if (imuSensor) imuSensor->update();
        if (baroSensor) baroSensor->update();

        // Обновляем режим
        updateMode();

        // Выполняем логику текущего режима
        switch (currentMode)
        {
            case MODE_MANUAL:
                handleManualMode();
                break;

            case MODE_STABILIZE:
                handleStabilizeMode();
                break;

            case MODE_AUTO_TAKEOFF:
                handleAutoTakeoffMode();
                break;

            case MODE_ALT_HOLD:
                handleAltHoldMode();
                break;
        }
    }

    // ========================================================
    // УСТАНОВИТЬ РЕЖИМ
    // ========================================================

    void setMode(AutopilotMode mode)
    {
        if (mode != currentMode)
        {
            Serial.print("🔄 Autopilot: Mode changed from ");
            Serial.print(modeToString(currentMode));
            Serial.print(" to ");
            Serial.println(modeToString(mode));

            previousMode = currentMode;
            currentMode = mode;
            modeChangeTime = millis();

            // Сбрасываем ПИД контроллеры при смене режима
            pidRoll.reset();
            pidPitch.reset();
            pidThrottle.reset();

            // Инициализируем переменные для нового режима
            initializeMode();
        }
    }

    // ========================================================
    // ПОЛУЧИТЬ ТЕКУЩИЙ РЕЖИМ
    // ========================================================

    AutopilotMode getMode() const
    {
        return currentMode;
    }

    // ========================================================
    // ПОЛУЧИТЬ ПОПРАВКИ УПРАВЛЕНИЯ
    // ========================================================
    // Возвращает поправки к сигналам с пульта (-500...+500 µs)
    // Интегрируются в ControlMixer

    float getRollCorrection() const
    {
        return rollCorrection;
    }

    float getPitchCorrection() const
    {
        return pitchCorrection;
    }

    float getThrottleCorrection() const
    {
        return throttleCorrection;
    }

    // ========================================================
    // ПОЛУЧИТЬ ИНФОРМАЦИЮ О СТАТУСЕ
    // ========================================================

    const char* getModeName() const
    {
        return modeToString(currentMode);
    }

    float getDesiredRoll() const { return desiredRoll; }
    float getDesiredPitch() const { return desiredPitch; }
    float getTargetAltitude() const { return targetAltitude; }
    // ========================================================
    // ДОСТУП К ДАТЧИКАМ (для WebDebugServer и диагностики)
    // ========================================================

    ImuSensor* getImuSensor() const
    {
        return imuSensor;
    }

    BarometerSensor* getBarometerSensor() const
    {
        return baroSensor;
    }

    // ========================================================
    // ДИАГНОСТИКА
    // ========================================================

    void printStatus() const
    {
        Serial.println("\n🚀 Autopilot Status:");
        Serial.print("  Mode: ");
        Serial.println(modeToString(currentMode));

        if (imuSensor)
        {
            const ImuData& imu = imuSensor->getImuData();
            Serial.print("  Roll: ");
            Serial.print(imu.roll, 1);
            Serial.print("° (desired: ");
            Serial.print(desiredRoll, 1);
            Serial.println("°)");

            Serial.print("  Pitch: ");
            Serial.print(imu.pitch, 1);
            Serial.print("° (desired: ");
            Serial.print(desiredPitch, 1);
            Serial.println("°)");

            Serial.print("  Yaw: ");
            Serial.print(imu.yaw, 1);
            Serial.println("°");
        }

        if (baroSensor)
        {
            const BarometerData& baro = baroSensor->getBarometerData();
            Serial.print("  Altitude: ");
            Serial.print(baro.altitude, 1);
            Serial.print("m (target: ");
            Serial.print(targetAltitude, 1);
            Serial.println("m)");

            Serial.print("  Climb rate: ");
            Serial.print(baro.verticalSpeed, 2);
            Serial.println(" m/s");
        }

        Serial.print("  Corrections: Roll=");
        Serial.print(rollCorrection);
        Serial.print(", Pitch=");
        Serial.print(pitchCorrection);
        Serial.print(", Throttle=");
        Serial.println(throttleCorrection);
    }

    // ========================================================
    // УСТАНОВИТЬ PID КОЭФФИЦИЕНТЫ
    // ========================================================
    // Используется для настройки через Web UI

    void setPIDGains(float kpRoll, float kiRoll, float kdRoll,
                     float kpPitch, float kiPitch, float kdPitch)
    {
        pidRoll.setGains(kpRoll, kiRoll, kdRoll);
        pidPitch.setGains(kpPitch, kiPitch, kdPitch);
    }

private:

    // ========================================================
    // ПРИВАТНЫЕ ПЕРЕМЕННЫЕ
    // ========================================================

    ImuSensor* imuSensor;
    BarometerSensor* baroSensor;

    AutopilotMode currentMode;
    AutopilotMode previousMode;
    unsigned long modeChangeTime;

    // PID контроллеры
    PID_Controller pidRoll;
    PID_Controller pidPitch;
    PID_Controller pidThrottle;

    // Вывод поправок
    float rollCorrection;
    float pitchCorrection;
    float throttleCorrection;

    // Параметры стабилизации
    float desiredRoll;
    float desiredPitch;

    // Параметры автоматического взлёта
    float autoTakeoffThrottle;
    unsigned long autoTakeoffStartTime;

    // Параметры удержания высоты
    float targetAltitude;


    // ========================================================
    // ИНИЦИАЛИЗАЦИЯ РЕЖИМА
    // ========================================================

    void initializeMode()
    {
        rollCorrection = 0;
        pitchCorrection = 0;
        throttleCorrection = 0;

        switch (currentMode)
        {
            case MODE_MANUAL:
                break;

            case MODE_STABILIZE:
                desiredRoll = 0;
                desiredPitch = 0;
                break;

            case MODE_AUTO_TAKEOFF:
                autoTakeoffThrottle = 0;
                autoTakeoffStartTime = millis();
                break;

            case MODE_ALT_HOLD:
                if (baroSensor)
                {
                    targetAltitude = baroSensor->getBarometerData().altitude;
                }
                break;
        }
    }


    // ========================================================
    // ОБНОВЛЕНИЕ РЕЖИМА (из внешних сигналов)
    // ========================================================

    void updateMode()
    {
        // Эта функция переопределяется Feature Manager
        // Пока оставляем пустой (переключение вручную через setMode())
    }


    // ========================================================
    // ОБРАБОТКА РЕЖИМА MANUAL
    // ========================================================

    void handleManualMode()
    {
        // В ручном режиме автопилот не вмешивается
        rollCorrection = 0;
        pitchCorrection = 0;
        throttleCorrection = 0;
    }


    // ========================================================
    // ОБРАБОТКА РЕЖИМА STABILIZE
    // ========================================================
    // Гиро-стабилизация: удерживаем углы крена и тангажа

    void handleStabilizeMode()
    {
        if (!imuSensor) return;

        const ImuData& imu = imuSensor->getImuData();

        // Используем ПИД контроллеры для стабилизации
        rollCorrection = pidRoll.calculate(desiredRoll, imu.roll);
        pitchCorrection = pidPitch.calculate(desiredPitch, imu.pitch);
    }


    // ========================================================
    // ОБРАБОТКА РЕЖИМА AUTO_TAKEOFF
    // ========================================================
    // Автоматический взлёт с управлением тангажом

    void handleAutoTakeoffMode()
    {
        if (!imuSensor) return;

        const ImuData& imu = imuSensor->getImuData();

        // Фазы взлёта:
        unsigned long elapsedTime = millis() - autoTakeoffStartTime;

        if (elapsedTime < 1000)
        {
            // Фаза 1: Разгон на земле (1 сек)
            autoTakeoffThrottle = 30;  // 30% газа
            desiredPitch = 0;          // Горизонтально
        }
        else if (elapsedTime < 3000)
        {
            // Фаза 2: Взлёт с углом в 15 градусов (2 сек)
            autoTakeoffThrottle = 60;  // 60% газа
            desiredPitch = 15;         // 15 градусов носом вверх
        }
        else
        {
            // Фаза 3: Набор высоты (100% газа)
            autoTakeoffThrottle = 100; // 100% газа
            desiredPitch = 10;         // Удерживаем 10 градусов
        }

        // Стабилизируем крен (убираем крен при взлёте)
        rollCorrection = pidRoll.calculate(0, imu.roll);

        // Стабилизируем тангаж
        pitchCorrection = pidPitch.calculate(desiredPitch, imu.pitch);

        // Поправка газа (кроме стабилизации используем также задаваемую мощность)
        throttleCorrection = autoTakeoffThrottle - 50;  // Центр = 50, max = 100
    }


    // ========================================================
    // ОБРАБОТКА РЕЖИМА ALT_HOLD
    // ========================================================
    // Удержание высоты автоматически

    void handleAltHoldMode()
    {
        if (!baroSensor) return;

        const BarometerData& baro = baroSensor->getBarometerData();

        // Используем ПИД для поддержания целевой высоты
        // Вывод контроллера = коррекция газа
        throttleCorrection = pidThrottle.calculate(targetAltitude, baro.altitude);
    }


    // ========================================================
    // ВСПОМОГАТЕЛЬНАЯ ФУНКЦИЯ
    // ========================================================

    const char* modeToString(AutopilotMode mode) const
    {
        switch (mode)
        {
            case MODE_MANUAL:       return "MANUAL";
            case MODE_STABILIZE:    return "STABILIZE";
            case MODE_AUTO_TAKEOFF: return "AUTO_TAKEOFF";
            case MODE_ALT_HOLD:     return "ALT_HOLD";
            default:                return "UNKNOWN";
        }
    }
};
