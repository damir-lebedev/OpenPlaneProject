#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// PID CONTROLLER
//
// ПИД-регулятор общего назначения для автопилота (крен, тангаж,
// высота). Особенности:
//   • D-член — по скорости изменения измеряемой величины, которую
//     даёт датчик (гироскоп, вариометр), а не по производной
//     ошибки: без шума численного дифференцирования и без скачка
//     при смене уставки ("derivative kick");
//   • интегратор ограничен (anti-windup) и может быть заморожен в
//     нуле (integrate = false), пока самолёт не заармлен;
//   • dt меряется сам, скачки после пауз заменяются номинальным
//     периодом цикла.
// ============================================================

class PidController
{
public:

    PidController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f)
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
