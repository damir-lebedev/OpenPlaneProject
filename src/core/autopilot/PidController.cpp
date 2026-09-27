// Реализация autopilot/PidController.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/PidController.h"


PidController::PidController(float kp, float ki, float kd)
: Kp(kp), Ki(ki), Kd(kd)
{
}

auto PidController::setGains(float kp, float ki, float kd) -> void
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

auto PidController::setLimits(float minOut, float maxOut) -> void
{
    minOutput = minOut;
    maxOutput = maxOut;
}

auto PidController::calculate(float setpoint, float feedback, float feedbackRate, bool integrate) -> float
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

auto PidController::reset() -> void
{
    errorSum = 0;
    lastTime = micros();
}
