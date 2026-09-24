#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FeedbackMath.h"

// ============================================================
// ADAPTIVE RATE CONTROLLER — угол → скорость вращения → руль
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// Будущая замена ПИД по углу из Autopilot, одна ось. Три ступени:
//
//   1. Ошибка угла → желаемая угловая скорость:
//        ω* = ANGLE_GAIN · (цель − угол), не больше MAX_RATE_DPS.
//      "Нос опущен на 10° — поднимать его со скоростью 40°/с".
//
//   2. Ошибка скорости → желаемое угловое ускорение:
//        ε* = (ω* − ω + I) / RATE_TAU,
//      ω — гироскоп, I — накопленная ошибка скорости. Это и есть
//      "руль подправлен не до конца — подправь ещё": пока самолёт
//      вращается медленнее, чем нужно (мало руля, ветер, центровка),
//      I растёт и добавляет руля — до тех пор, пока реальный самолёт
//      не начнёт вращаться как надо.
//
//   3. Желаемое ускорение → руль через модель самолёта:
//        руль = (ε* − a·ω − c) / b,
//      b, a, c — текущие эффективность, демпфирование и постоянный
//      момент (ControlEffectivenessEstimator). На малой скорости b
//      мала — руль больше, на большой — меньше: регулятор
//      подстраивается под реальные скорость и высоту, а не настроен
//      под одну точку, как ПИД.
//
// I хранится в °/с, а не в мкс руля, поэтому остаётся правильным,
// когда меняется оценка b (скорость, высота, обучение).
// ============================================================

// Что сейчас известно о реакции оси (собирает FeedbackSupervisor).
struct AxisModel
{
    float effectiveness = 1.0f;   // b со знаком оси, °/с² на мкс
    float damping = 0;            // a, 1/с; 0 — не компенсировать
    float bias = 0;               // c, °/с²; 0 — не компенсировать
};

class AdaptiveRateController
{
public:

    explicit AdaptiveRateController(uint8_t axis = FeedbackConfig::AXIS_ROLL)
        : axis(axis) {}

    void reset()
    {
        integral = 0;
        saturatedDirection = 0;
        desiredRate = 0;
        output = 0;
    }

    // Ступень 1. Для рысканья (курс) ошибка берётся по кратчайшему
    // пути: цель 350°, курс 10° — поворот на −20°, а не на 340°.
    float angleToRate(float targetDeg, float angleDeg) const
    {
        const float error = FeedbackMath::wrap180(targetDeg - angleDeg);
        return FeedbackMath::clampAbs(FeedbackConfig::ANGLE_GAIN[axis] * error,
                                      FeedbackConfig::MAX_RATE_DPS[axis]);
    }

    // Ступени 2–3: отклонение руля, мкс (физические знаки).
    // allowIntegral — false на земле и когда ось не управляет
    // самолётом: иначе I копит то, что рулём не исправить.
    float update(float desiredRateDps, float rateDps, const AxisModel& model,
                 bool allowIntegral, float dt)
    {
        desiredRate = FeedbackMath::clampAbs(desiredRateDps, FeedbackConfig::MAX_RATE_DPS[axis]);
        const float rateError = desiredRate - rateDps;

        // Модуль b не меньше минимума: иначе на почти нулевой
        // эффективности руль улетел бы в бесконечность.
        const float bMin = FeedbackConfig::EFFECTIVENESS_MIN[axis];
        float b = model.effectiveness;
        if (fabsf(b) < bMin) b = (b < 0) ? -bMin : bMin;

        // Интеграл. Не копить в сторону, куда руль уже упёрся
        // (anti-windup): иначе после упора он долго "отматывается".
        if (allowIntegral && dt > 0)
        {
            const float step = FeedbackConfig::RATE_INTEGRAL_GAIN[axis] * rateError * dt;
            const int8_t pushDirection = FeedbackMath::signOf(step) * FeedbackMath::signOf(b);
            if (saturatedDirection == 0 || pushDirection != saturatedDirection)
            {
                integral = FeedbackMath::clampAbs(integral + step, FeedbackConfig::MAX_RATE_DPS[axis]);
            }
        }

        const float desiredAccel = (rateError + integral) / FeedbackConfig::RATE_TAU_S[axis];
        const float deflection = (desiredAccel - model.damping * rateDps - model.bias) / b;

        const float limit = FeedbackConfig::MAX_DEFLECTION_US[axis];
        saturatedDirection = deflection > limit ? 1 : (deflection < -limit ? -1 : 0);
        output = FeedbackMath::clampAbs(deflection, limit);
        return output;
    }

    float getDesiredRate() const { return desiredRate; }
    float getIntegral() const { return integral; }
    float getOutput() const { return output; }
    bool isSaturated() const { return saturatedDirection != 0; }


private:

    uint8_t axis;

    float integral = 0;             // °/с
    int8_t saturatedDirection = 0;  // куда руль упёрся на прошлом шаге
    float desiredRate = 0;
    float output = 0;
};
