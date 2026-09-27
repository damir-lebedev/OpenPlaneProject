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

    explicit AdaptiveRateController(uint8_t axisIndex = FeedbackConfig::AXIS_ROLL);

    void reset();

    // Ступень 1. Для рысканья (курс) ошибка берётся по кратчайшему
    // пути: цель 350°, курс 10° — поворот на −20°, а не на 340°.
    float angleToRate(float targetDeg, float angleDeg) const;

    // Ступени 2–3: отклонение руля, мкс (физические знаки).
    // allowIntegral — false на земле и когда ось не управляет
    // самолётом: иначе I копит то, что рулём не исправить.
    float update(float desiredRateDps, float rateDps, const AxisModel& model,
                 bool allowIntegral, float dt);

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
