#pragma once
#include <Arduino.h>

#include "sensors/SensorInterface.h"
#include "Config.h"

// ============================================================
// АВТОПИЛОТ
//
// Режимы: MANUAL (без коррекции) / STABILIZE (ПИД по крену и
// тангажу) / AUTO_TAKEOFF (сценарий газ+тангаж по времени) /
// ALT_HOLD (ПИД газа по высоте барометра).
//
// Режим переключается снаружи (FeatureManager). Этот класс
// только считает коррекции, которые FlightController добавляет
// к сигналам микшера. Безопасно работает и без датчиков (imu/baro
// == nullptr или физически не отвечают на I2C) — тогда просто
// не даёт коррекции (0).
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

    void setGains(float kp, float ki, float kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    void setLimits(float minOut, float maxOut)
    {
        minOutput = minOut;
        maxOutput = maxOut;
    }

    // setpoint/feedback в одних единицах (например, градусы).
    // Возвращает коррекцию, ограниченную [minOutput, maxOutput].
    float calculate(float setpoint, float feedback)
    {
        unsigned long now = micros();
        float dt = (now - lastTime) / 1000000.0f;
        lastTime = now;

        // Первый вызов после reset() или долгая пауза (например,
        // I2C подвис) — берём номинальный период цикла, чтобы
        // D-член не выдал случайный всплеск.
        if (dt < 0.001f || dt > 1.0f)
        {
            dt = 0.002f;
        }

        float error = setpoint - feedback;

        float P = Kp * error;

        errorSum += error * dt;
        if (errorSum > 100) errorSum = 100;   // защита от переполнения интегратора
        if (errorSum < -100) errorSum = -100;
        float I = Ki * errorSum;

        float dError = (error - lastError) / dt;
        float D = Kd * dError;

        lastError = error;

        float output = P + I + D;

        if (output > maxOutput) output = maxOutput;
        if (output < minOutput) output = minOutput;

        return output;
    }

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

enum AutopilotMode
{
    MODE_MANUAL = 0,
    MODE_STABILIZE = 1,
    MODE_AUTO_TAKEOFF = 2,
    MODE_ALT_HOLD = 3
};

class Autopilot
{
public:

    Autopilot(ImuSensor* imu = nullptr, BarometerSensor* baro = nullptr)
        : imuSensor(imu),
          baroSensor(baro),
          currentMode(MODE_MANUAL),
          previousMode(MODE_MANUAL),
          modeChangeTime(0),
          rollCorrection(0.0f),
          pitchCorrection(0.0f),
          throttleCorrection(0.0f),
          desiredRoll(0),
          desiredPitch(0),
          autoTakeoffThrottle(0),
          autoTakeoffStartTime(0),
          targetAltitude(0)
    {
        // Значения по умолчанию, можно перенастроить через setPIDGains()/веб.
        pidRoll.setGains(0.05f, 0.01f, 0.02f);
        pidPitch.setGains(0.05f, 0.01f, 0.02f);
        pidThrottle.setGains(0.1f, 0.05f, 0.01f);

        pidRoll.setLimits(-500, 500);
        pidPitch.setLimits(-500, 500);
        pidThrottle.setLimits(-100, 100);
    }

    // Возвращает false, если датчики не подключены; STABILIZE/ALT_HOLD
    // в этом случае просто не дают коррекции (см. handle*Mode()).
    bool begin()
    {
        if (!imuSensor || !baroSensor)
        {
            Serial.println("Autopilot: IMU или барометр не подключены, STABILIZE/ALT_HOLD недоступны");
            return false;
        }

        Serial.println("Autopilot: инициализирован");
        return true;
    }

    // Вызывается один раз за цикл из FlightController::update().
    void update()
    {
        if (imuSensor) imuSensor->update();
        if (baroSensor) baroSensor->update();

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

    void setMode(AutopilotMode mode)
    {
        if (mode == currentMode) return;

        Serial.print("Autopilot: режим ");
        Serial.print(modeToString(currentMode));
        Serial.print(" -> ");
        Serial.println(modeToString(mode));

        previousMode = currentMode;
        currentMode = mode;
        modeChangeTime = millis();

        pidRoll.reset();
        pidPitch.reset();
        pidThrottle.reset();

        initializeMode();
    }

    AutopilotMode getMode() const { return currentMode; }

    // Коррекции для добавления к выходу микшера, в микросекундах.
    float getRollCorrection() const { return rollCorrection; }
    float getPitchCorrection() const { return pitchCorrection; }
    float getThrottleCorrection() const { return throttleCorrection; }

    const char* getModeName() const { return modeToString(currentMode); }

    float getDesiredRoll() const { return desiredRoll; }
    float getDesiredPitch() const { return desiredPitch; }
    float getTargetAltitude() const { return targetAltitude; }

    // Для WebDebugServer и диагностики; может быть nullptr.
    ImuSensor* getImuSensor() const { return imuSensor; }
    BarometerSensor* getBarometerSensor() const { return baroSensor; }

    void printStatus() const
    {
        Serial.print("Autopilot: mode=");
        Serial.print(modeToString(currentMode));

        if (!imuSensor)
        {
            Serial.print(" imu=NOT_ATTACHED");
        }
        else if (!imuSensor->isAvailable())
        {
            Serial.print(" imu=NO_RESPONSE");
        }
        else
        {
            const ImuData& imu = imuSensor->getImuData();
            Serial.print(" roll="); Serial.print(imu.roll, 1);
            Serial.print("(want "); Serial.print(desiredRoll, 1); Serial.print(")");
            Serial.print(" pitch="); Serial.print(imu.pitch, 1);
            Serial.print("(want "); Serial.print(desiredPitch, 1); Serial.print(")");
            Serial.print(" yaw="); Serial.print(imu.yaw, 1);
        }

        if (!baroSensor)
        {
            Serial.print(" baro=NOT_ATTACHED");
        }
        else if (!baroSensor->isAvailable())
        {
            Serial.print(" baro=NO_RESPONSE");
        }
        else
        {
            const BarometerData& baro = baroSensor->getBarometerData();
            Serial.print(" alt="); Serial.print(baro.altitude, 1);
            Serial.print("(want "); Serial.print(targetAltitude, 1); Serial.print(")");
            Serial.print(" climb="); Serial.print(baro.verticalSpeed, 2);
        }

        Serial.print(" corr(roll,pitch,thr)=");
        Serial.print(rollCorrection); Serial.print(",");
        Serial.print(pitchCorrection); Serial.print(",");
        Serial.println(throttleCorrection);
    }

    // Перенастройка коэффициентов стабилизации крена/тангажа на ходу (веб-интерфейс).
    void setPIDGains(float kpRoll, float kiRoll, float kdRoll,
                     float kpPitch, float kiPitch, float kdPitch)
    {
        pidRoll.setGains(kpRoll, kiRoll, kdRoll);
        pidPitch.setGains(kpPitch, kiPitch, kdPitch);
    }

private:

    ImuSensor* imuSensor;
    BarometerSensor* baroSensor;

    AutopilotMode currentMode;
    AutopilotMode previousMode;
    unsigned long modeChangeTime;

    PID_Controller pidRoll;
    PID_Controller pidPitch;
    PID_Controller pidThrottle;

    float rollCorrection;
    float pitchCorrection;
    float throttleCorrection;

    float desiredRoll;
    float desiredPitch;

    float autoTakeoffThrottle;
    unsigned long autoTakeoffStartTime;

    float targetAltitude;

    // Сбрасывает состояние режима, вызывается только при реальной смене режима.
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

    void handleManualMode()
    {
        rollCorrection = 0;
        pitchCorrection = 0;
        throttleCorrection = 0;
    }

    void handleStabilizeMode()
    {
        if (!imuSensor) return;

        const ImuData& imu = imuSensor->getImuData();

        rollCorrection = pidRoll.calculate(desiredRoll, imu.roll);
        pitchCorrection = pidPitch.calculate(desiredPitch, imu.pitch);
    }

    // Сценарий по времени: 1с разгон на земле, 2с набор с тангажом 15°,
    // затем крейсерский набор высоты с тангажом 10°. Крен всё время держим нулевым.
    void handleAutoTakeoffMode()
    {
        if (!imuSensor) return;

        const ImuData& imu = imuSensor->getImuData();

        unsigned long elapsedTime = millis() - autoTakeoffStartTime;

        if (elapsedTime < 1000)
        {
            autoTakeoffThrottle = 30;
            desiredPitch = 0;
        }
        else if (elapsedTime < 3000)
        {
            autoTakeoffThrottle = 60;
            desiredPitch = 15;
        }
        else
        {
            autoTakeoffThrottle = 100;
            desiredPitch = 10;
        }

        rollCorrection = pidRoll.calculate(0, imu.roll);
        pitchCorrection = pidPitch.calculate(desiredPitch, imu.pitch);

        // 50 — нейтральная точка коррекции газа (throttleCorrection в %).
        throttleCorrection = autoTakeoffThrottle - 50;
    }

    void handleAltHoldMode()
    {
        if (!baroSensor) return;

        const BarometerData& baro = baroSensor->getBarometerData();

        throttleCorrection = pidThrottle.calculate(targetAltitude, baro.altitude);
    }

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
