#pragma once
#include <Arduino.h>

// ============================================================
// ATTITUDE ESTIMATOR — комплементарный фильтр ориентации
//
// Не зависит от конкретного чипа: принимает ускорение и угловые
// скорости уже в осях самолёта и выдаёт крен/тангаж/рысканье в
// авиационных знаках (roll > 0 — правое крыло вниз, pitch > 0 — нос
// вверх, yaw > 0 — нос вправо).
//
// Акселерометр даёт абсолютный угол, но шумит и "врёт" в манёвре;
// гироскоп даёт гладкую скорость без абсолютной опоры. Обе части
// взвешены одним ALPHA: ALPHA = 0.98 при цикле ~2 мс даёт постоянную
// времени tau = dt·ALPHA/(1−ALPHA) ≈ 0.1 с. Если в полёте углы
// "тормозят" относительно реального движения — ALPHA уменьшить.
//
// Рысканье — чистый интеграл гироскопа: абсолютной опоры нет,
// медленно уплывает. Начальное значение задаёт setYaw() (например,
// курс компаса при старте).
//
// Акселерометр видит не только тяжесть, но и разгон: на разбеге 0.5 g
// вперёд для него — нос задран на +27°. Поэтому автовзлёт на время
// разгона отключает его (setAccelCorrection(false)) — крен и тангаж ведёт
// только гироскоп от точно известных стояночных углов. Включили обратно —
// фильтр сам за ~0.1 с подтягивается к акселерометру.
// ============================================================

class AttitudeEstimator
{
public:

    // Сбросить фильтр: следующий update() начнёт сразу с угла по
    // акселерометру, а не будет "доезжать" до него с нуля.
    void reset()
    {
        initialized = false;
    }

    void setYaw(float yawDegrees)
    {
        yaw = wrap180(yawDegrees);
    }

    // false — акселерометр не опора горизонта, только гироскоп.
    void setAccelCorrection(bool enabled)
    {
        accelCorrection = enabled;
    }

    bool isAccelCorrectionEnabled() const { return accelCorrection; }

    // ax/ay/az — ускорение, g, оси самолёта (X к носу, Y влево, Z вверх).
    // rollRate/pitchRate/yawRate — °/с, авиационные знаки.
    void update(float ax, float ay, float az,
                float rollRate, float pitchRate, float yawRate,
                uint32_t nowUs)
    {
        // Нос вверх -> проекция "верха" на X положительна -> pitch > 0.
        // Правое крыло вниз -> проекция "верха" на Y (влево) > 0 -> roll > 0.
        const float accelRoll = atan2f(ay, az) * RAD_TO_DEG_F;
        const float accelPitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;

        const float dt = (nowUs - lastUpdateUs) / 1000000.0f;
        lastUpdateUs = nowUs;

        if (!initialized)
        {
            roll = accelRoll;
            pitch = accelPitch;
            initialized = true;
            return;
        }

        // Пауза (калибровка, зависание шины) — не интегрируем скачок dt.
        if (dt <= 0.0f || dt > MAX_DT_S)
        {
            return;
        }

        const float alpha = accelCorrection ? ALPHA : 1.0f;
        roll = wrap180(alpha * (roll + rollRate * dt) + (1.0f - alpha) * accelRoll);
        pitch = wrap180(alpha * (pitch + pitchRate * dt) + (1.0f - alpha) * accelPitch);
        yaw = wrap180(yaw + yawRate * dt);
    }

    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }


private:

    static constexpr float ALPHA = 0.98f;
    static constexpr float MAX_DT_S = 0.1f;

    // RAD_TO_DEG из Arduino.h — double: умножение на него шло бы в
    // программной двойной точности (у ESP32 аппаратная только float).
    static constexpr float RAD_TO_DEG_F = static_cast<float>(RAD_TO_DEG);

    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    uint32_t lastUpdateUs = 0;
    bool initialized = false;
    bool accelCorrection = true;

    static float wrap180(float angle)
    {
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }
};
