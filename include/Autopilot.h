#pragma once
#include <Arduino.h>

#include "sensors/SensorInterface.h"
#include "Config.h"

// ============================================================
// АВТОПИЛОТ
//
// Режимы: MANUAL (без коррекции) / STABILIZE (ПИД по крену и
// тангажу, выравнивание к горизонту поверх стиков) / AUTO_TAKEOFF
// (автовзлёт: газ + набор с выровненными крыльями) / ALT_HOLD (ПИД
// газа по высоте барометра).
//
// Режим переключается снаружи (AutopilotModeSelector, CH7). Этот класс
// только считает коррекции, которые FlightController добавляет к
// командам стиков (ControlCommand, те же физические знаки: roll > 0 —
// крен вправо, pitch > 0 — нос вверх), и газ для текущего режима.
// Безопасно работает и без датчиков (imu/baro == nullptr или не
// отвечают на I2C) — тогда просто не даёт коррекции (0).
//
// Углы IMU — в авиационных знаках (см. MPU6050_Sensor.h): roll > 0 —
// правое крыло вниз, pitch > 0 — нос вверх. Поэтому ПИД с ошибкой
// (цель - факт) сразу даёт команду нужного знака: крен вправо
// (roll > 0) -> ошибка < 0 -> команда крена влево.
//
// Пока самолёт не заармлен, стабилизация всё равно двигает рули
// (удобно на столе: наклонил — видно, в какую сторону отвечают
// рули), но интегратор держится на нуле, чтобы на земле не
// накопилась поправка, которая дёрнет рули в момент взлёта.
// ============================================================

class PID_Controller
{
public:

    PID_Controller(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f)
        : Kp(kp), Ki(ki), Kd(kd)
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

    float getKp() const { return Kp; }
    float getKi() const { return Ki; }
    float getKd() const { return Kd; }

    // setpoint/feedback в одних единицах (например, градусы),
    // feedbackRate — скорость изменения feedback (град/с с гироскопа,
    // м/с с барометра). D-член берётся по ней, а не по производной
    // ошибки: гироскоп даёт скорость напрямую и без шума численного
    // дифференцирования, а смена уставки (например, AUTO_TAKEOFF
    // 0° -> 15°) не даёт скачка D ("derivative kick").
    //
    // integrate = false — интегратор не копится и сбрасывается
    // (используется, пока самолёт не заармлен).
    //
    // Возвращает коррекцию, ограниченную [minOutput, maxOutput].
    float calculate(float setpoint, float feedback, float feedbackRate, bool integrate = true)
    {
        const uint32_t now = micros();
        float dt = (now - lastTime) / 1000000.0f;
        lastTime = now;

        // Первый вызов после reset() или долгая пауза — номинальный
        // период цикла, чтобы I-член не получил скачок.
        if (dt <= 0.0f || dt > 0.1f)
        {
            dt = Config::LOOP_PERIOD_MS / 1000.0f;
        }

        const float error = setpoint - feedback;

        const float P = Kp * error;

        if (integrate)
        {
            errorSum += error * dt;
            errorSum = constrain(errorSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);  // защита от раскрутки интегратора
        }
        else
        {
            errorSum = 0;
        }
        const float I = Ki * errorSum;

        const float D = -Kd * feedbackRate;

        return constrain(P + I + D, minOutput, maxOutput);
    }

    void reset()
    {
        errorSum = 0;
        lastTime = micros();
    }

private:
    static constexpr float INTEGRAL_LIMIT = 100.0f;

    float Kp, Ki, Kd;
    float errorSum = 0;
    uint32_t lastTime = 0;
    float minOutput = -500, maxOutput = 500;
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

    Autopilot(ImuSensor* imu = nullptr, BarometerSensor* baro = nullptr,
              MagnetometerSensor* mag = nullptr, GpsSensor* gps = nullptr)
        : imuSensor(imu),
          baroSensor(baro),
          magSensor(mag),
          gpsSensor(gps)
    {
        // Значения по умолчанию, можно перенастроить через setPIDGains()/веб
        // (см. WebDebugServer POST /api/setpid) без перепрошивки.
        //
        // Kp=5: 8° ошибки -> 40 мкс, 30° -> 150 мкс — заметное движение
        // поверхности, но не сразу упирается в потолок ±500 (см.
        // setLimits ниже). Kd теперь умножается на угловую скорость
        // гироскопа (град/с): 0.5 даёт 50 мкс демпфирования на 100°/с.
        //
        // ALT_HOLD: выход — поправка газа в %: 10%/м ошибки высоты,
        // D — 5% на 1 м/с вертикальной скорости.
        //
        // Это стартовая точка для полевой настройки, не финальный тюнинг —
        // регулировать на столе/в полёте через дашборд, не в коде.
        pidRoll.setGains(5.0f, 0.5f, 0.5f);
        pidPitch.setGains(5.0f, 0.5f, 0.5f);
        pidThrottle.setGains(10.0f, 2.0f, 5.0f);

        pidRoll.setLimits(-500, 500);
        pidPitch.setLimits(-500, 500);
        pidThrottle.setLimits(-50, 50);
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

    // Вызывается один раз за цикл из FlightController::update(), в том
    // числе во время failsafe — датчики читаются всегда, чтобы фильтры
    // углов не "застывали". pilotThrottleUs — газ со стика (нужен
    // AUTO_TAKEOFF, чтобы понять, что пилот дал команду на взлёт).
    void update(bool isArmed, uint16_t pilotThrottleUs)
    {
        armed = isArmed;

        if (imuSensor) imuSensor->update();
        if (baroSensor) baroSensor->update();
        if (magSensor) magSensor->update();
        if (gpsSensor) gpsSensor->update();

        switch (currentMode)
        {
            case MODE_MANUAL:
                handleManualMode();
                break;

            case MODE_STABILIZE:
                handleStabilizeMode();
                break;

            case MODE_AUTO_TAKEOFF:
                handleAutoTakeoffMode(pilotThrottleUs);
                break;

            case MODE_ALT_HOLD:
                handleAltHoldMode();
                break;
        }
    }

    // Газ на ESC для текущего режима, исходя из газа пилота (мкс).
    // ARM и failsafe учитывает FlightController, не этот метод.
    uint16_t applyThrottle(uint16_t pilotThrottleUs) const
    {
        switch (currentMode)
        {
            case MODE_AUTO_TAKEOFF:
                // Пилот всегда может добавить газ сверх программы, но
                // не убрать его ниже — выход из автовзлёта переключателем
                // CH7 или DISARM.
                return max(pilotThrottleUs, percentToUs(throttleCorrection));

            case MODE_ALT_HOLD:
            {
                const int32_t corrected = pilotThrottleUs +
                    static_cast<int32_t>(throttleCorrection * (Config::PWM_MAX - Config::PWM_MIN) / 100.0f);
                return static_cast<uint16_t>(constrain(corrected, (int32_t)Config::PWM_MIN, (int32_t)Config::PWM_MAX));
            }

            default:
                return pilotThrottleUs;
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

    // Коррекции для добавления к командам стиков, в мкс отклонения
    // (roll > 0 — крен вправо, pitch > 0 — нос вверх).
    float getRollCorrection() const { return rollCorrection; }
    float getPitchCorrection() const { return pitchCorrection; }

    // ALT_HOLD: поправка газа в % (-50..50) к газу пилота.
    // AUTO_TAKEOFF: программный газ в % (0..100), см. applyThrottle().
    float getThrottleCorrection() const { return throttleCorrection; }

    const char* getModeName() const { return modeToString(currentMode); }

    float getDesiredRoll() const { return desiredRoll; }
    float getDesiredPitch() const { return desiredPitch; }
    float getTargetAltitude() const { return targetAltitude; }

    // Для WebDebugServer и диагностики; может быть nullptr.
    ImuSensor* getImuSensor() const { return imuSensor; }
    BarometerSensor* getBarometerSensor() const { return baroSensor; }
    MagnetometerSensor* getMagnetometerSensor() const { return magSensor; }
    GpsSensor* getGpsSensor() const { return gpsSensor; }

    const PID_Controller& getRollPid() const { return pidRoll; }
    const PID_Controller& getPitchPid() const { return pidPitch; }

    // Печатает статус в переданный Print (обычно — буфер DebugLogger'а,
    // который потом сверяет кадр целиком и не шлёт в Serial, если ничего
    // не изменилось).
    //
    // roll/pitch/yaw/alt/climb/corr печатаются через shown*-копии с
    // допуском (applyPrintDeadband) — иначе шум датчика в десятые доли
    // градуса/метра меняет строку каждый DEBUG_INTERVAL_MS, и
    // DebugLogger шлёт новый кадр в Serial непрерывно, даже когда
    // самолёт лежит неподвижно.
    void printStatus(Print& out) const
    {
        out.print("Autopilot: mode=");
        out.print(modeToString(currentMode));

        if (currentMode == MODE_AUTO_TAKEOFF)
        {
            out.print(takeoffStarted ? "(RUN)" : "(WAIT THR>50%)");
        }

        if (!imuSensor)
        {
            out.print(" imu=NOT_ATTACHED");
        }
        else if (!imuSensor->isAvailable())
        {
            out.print(" imu=NO_RESPONSE");
        }
        else
        {
            const ImuData& imu = imuSensor->getImuData();
            applyPrintDeadband(imu.roll, shownRoll, 0.3f);
            applyPrintDeadband(imu.pitch, shownPitch, 0.3f);
            applyPrintDeadband(imu.yaw, shownYaw, 0.3f);

            out.print(" roll="); out.print(shownRoll, 1);
            out.print("(want "); out.print(desiredRoll, 1); out.print(")");
            out.print(" pitch="); out.print(shownPitch, 1);
            out.print("(want "); out.print(desiredPitch, 1); out.print(")");
            out.print(" yaw="); out.print(shownYaw, 1);
        }

        if (!baroSensor)
        {
            out.print(" baro=NOT_ATTACHED");
        }
        else if (!baroSensor->isAvailable())
        {
            out.print(" baro=NO_RESPONSE");
        }
        else
        {
            const BarometerData& baro = baroSensor->getBarometerData();
            applyPrintDeadband(baro.altitude, shownAltitude, 0.3f);
            applyPrintDeadband(baro.verticalSpeed, shownClimb, 0.5f);

            out.print(" alt="); out.print(shownAltitude, 1);
            out.print("(want "); out.print(targetAltitude, 1); out.print(")");
            out.print(" climb="); out.print(shownClimb, 2);
        }

        applyPrintDeadband(rollCorrection, shownRollCorr, 0.5f);
        applyPrintDeadband(pitchCorrection, shownPitchCorr, 0.5f);
        applyPrintDeadband(throttleCorrection, shownThrottleCorr, 0.5f);

        out.print(" corr(roll,pitch,thr)=");
        out.print(shownRollCorr, 0); out.print(",");
        out.print(shownPitchCorr, 0); out.print(",");
        out.println(shownThrottleCorr, 0);

        if (magSensor && magSensor->isAvailable())
        {
            applyPrintDeadband(magSensor->getMagData().headingDegrees, shownHeading, 1.0f);
            out.print("Autopilot: mag heading=");
            out.println(shownHeading, 0);
        }

        if (gpsSensor && gpsSensor->isAvailable())
        {
            const GpsData& gps = gpsSensor->getGpsData();
            out.print("Autopilot: gps fix="); out.print(gps.fixType);
            out.print(" numSV="); out.print(gps.numSatellites);
            out.print(" lat="); out.print(gps.latitude, 6);
            out.print(" lon="); out.println(gps.longitude, 6);
        }
    }

    // Перенастройка коэффициентов стабилизации крена/тангажа на ходу (веб-интерфейс).
    void setPIDGains(float kpRoll, float kiRoll, float kdRoll,
                     float kpPitch, float kiPitch, float kdPitch)
    {
        pidRoll.setGains(kpRoll, kiRoll, kdRoll);
        pidPitch.setGains(kpPitch, kiPitch, kdPitch);
    }

private:

    // Автовзлёт: программа стартует, когда самолёт заармлен и пилот
    // поднял газ выше этого порога (защита от раскрутки мотора сразу
    // после ARM, например на столе).
    static constexpr uint16_t TAKEOFF_TRIGGER_US = 1500;
    static constexpr uint32_t TAKEOFF_THROTTLE_RAMP_MS = 1000;

    ImuSensor* imuSensor;
    BarometerSensor* baroSensor;
    MagnetometerSensor* magSensor;
    GpsSensor* gpsSensor;

    AutopilotMode currentMode = MODE_MANUAL;
    AutopilotMode previousMode = MODE_MANUAL;
    uint32_t modeChangeTime = 0;

    bool armed = false;

    PID_Controller pidRoll;
    PID_Controller pidPitch;
    PID_Controller pidThrottle;

    float rollCorrection = 0;
    float pitchCorrection = 0;
    float throttleCorrection = 0;

    float desiredRoll = 0;
    float desiredPitch = 0;

    bool takeoffStarted = false;
    uint32_t takeoffStartTime = 0;

    float targetAltitude = 0;

    // "Отображаемые" версии шумных полей для printStatus() — см.
    // комментарий у printStatus(). mutable — печать не меняет
    // логическое состояние автопилота, только сглаживает вывод.
    mutable float shownRoll = 0, shownPitch = 0, shownYaw = 0;
    mutable float shownAltitude = 0, shownClimb = 0;
    mutable float shownRollCorr = 0, shownPitchCorr = 0, shownThrottleCorr = 0;
    mutable float shownHeading = 0;

    static void applyPrintDeadband(float raw, float& shown, float deadband)
    {
        if (fabsf(raw - shown) > deadband)
        {
            shown = raw;
        }
    }

    static uint16_t percentToUs(float percent)
    {
        const float clamped = constrain(percent, 0.0f, 100.0f);
        return static_cast<uint16_t>(Config::PWM_MIN + clamped * (Config::PWM_MAX - Config::PWM_MIN) / 100.0f);
    }

    bool imuReady() const
    {
        return imuSensor && imuSensor->isAvailable();
    }

    // Сбрасывает состояние режима, вызывается только при реальной смене режима.
    void initializeMode()
    {
        rollCorrection = 0;
        pitchCorrection = 0;
        throttleCorrection = 0;
        desiredRoll = 0;
        desiredPitch = 0;
        takeoffStarted = false;

        if (currentMode == MODE_ALT_HOLD && baroSensor)
        {
            targetAltitude = baroSensor->getBarometerData().altitude;
        }
    }

    void handleManualMode()
    {
        rollCorrection = 0;
        pitchCorrection = 0;
        throttleCorrection = 0;
    }

    // Крен/тангаж к desiredRoll/desiredPitch (обычно 0 — горизонт).
    void stabilize()
    {
        if (!imuReady())
        {
            rollCorrection = 0;
            pitchCorrection = 0;
            return;
        }

        const ImuData& imu = imuSensor->getImuData();

        rollCorrection = pidRoll.calculate(desiredRoll, imu.roll, imu.gyroX, armed);
        pitchCorrection = pidPitch.calculate(desiredPitch, imu.pitch, imu.gyroY, armed);
    }

    void handleStabilizeMode()
    {
        desiredRoll = 0;
        desiredPitch = 0;
        stabilize();
    }

    // Автовзлёт (бросок с руки или разбег): после ARM ждём, пока пилот
    // поднимет газ выше TAKEOFF_TRIGGER_US, затем:
    //   0..1 с — газ плавно до 100%, тангаж 0° (набор скорости);
    //   1..3 с — 100%, тангаж +15° (отрыв/уход от земли);
    //   дальше — 100%, тангаж +10° (набор высоты), пока пилот не
    //   переключит CH7 в другой режим.
    // Крылья всё время держатся ровно. Стики пилота по-прежнему
    // добавляются поверх (FlightController).
    void handleAutoTakeoffMode(uint16_t pilotThrottleUs)
    {
        if (!armed)
        {
            takeoffStarted = false;
        }
        else if (!takeoffStarted && pilotThrottleUs >= TAKEOFF_TRIGGER_US)
        {
            takeoffStarted = true;
            takeoffStartTime = millis();
            Serial.println("Autopilot: автовзлёт — старт");
        }

        desiredRoll = 0;

        if (!takeoffStarted)
        {
            desiredPitch = 0;
            throttleCorrection = 0;
        }
        else
        {
            const uint32_t elapsed = millis() - takeoffStartTime;

            throttleCorrection = elapsed < TAKEOFF_THROTTLE_RAMP_MS
                ? 100.0f * elapsed / TAKEOFF_THROTTLE_RAMP_MS
                : 100.0f;

            if (elapsed < 1000)      desiredPitch = 0;
            else if (elapsed < 3000) desiredPitch = 15;
            else                     desiredPitch = 10;
        }

        stabilize();
    }

    void handleAltHoldMode()
    {
        if (!baroSensor || !baroSensor->isAvailable())
        {
            throttleCorrection = 0;
            return;
        }

        const BarometerData& baro = baroSensor->getBarometerData();

        throttleCorrection = pidThrottle.calculate(targetAltitude, baro.altitude, baro.verticalSpeed, armed);
    }

    static const char* modeToString(AutopilotMode mode)
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
