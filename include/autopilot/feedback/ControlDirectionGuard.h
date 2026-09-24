#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FeedbackMath.h"

// ============================================================
// CONTROL DIRECTION GUARD — ось работает наоборот? Выключить её
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// Одна ось. Если автопилот тянет нос вверх, а нос уверенно идёт вниз,
// значит, ось перепутана: неправильный *_REVERSED в Config.h, тяги
// перекинуты, IMU переставили без новой калибровки установки. Без
// защиты автопилот в таком случае не выравнивает самолёт, а сам его
// заваливает, всё сильнее с каждым тактом.
//
// Знак в полёте НЕ переворачивается. Направления определяются на
// земле: установка IMU — калибровкой 'o' и проверкой при включении
// (ImuSensorBase), рули — предполётной проверкой пилота. Переворот
// в воздухе по косвенным признакам может ошибиться (порыв, срыв), а
// ошибка там стоит самолёта. Здесь — только последняя страховка:
// ось, которая явно работает наоборот, выключается (коррекции 0,
// рули остаются на стиках пилота). Бороться в неправильную сторону
// хуже, чем не бороться.
//
// Два независимых признака "ось работает наоборот":
//   • оценка эффективности b (ControlEffectivenessEstimator) уверенно
//     держится отрицательной дольше REVERSAL_CONFIRM_MS;
//   • расходимость: коррекция упорно толкает к цели, а самолёт
//     вращается от цели, и ошибка за DIVERGENCE_WINDOW_MS выросла на
//     DIVERGENCE_ERROR_GROWTH_DEG. Не нужна обученная оценка —
//     срабатывает в первые секунды. Игнорируется, если оценка
//     уверенно показывает, что ось работает правильно (значит, порыв).
//
// Анализ — только когда можно доверять наблюдению: в воздухе, не на
// сваливании (там нос падает при любом руле), закрылки стоят.
// Расходимость — ещё и когда пилот этим стиком не двигает (он может
// вращать самолёт нарочно). Оценке b стик не мешает — наоборот, это
// хорошая раскачка: перепутанная ось видна даже в MANUAL, до
// включения стабилизации.
//
// Выключенная ось остаётся выключенной до дизарма; лог и OLED
// говорят, какая, — исправить Config.h и проверить на земле.
// ============================================================

class ControlDirectionGuard
{
public:

    struct Inputs
    {
        // Ошибка угла (цель − угол), если ось сейчас держит угол;
        // angleErrorValid = false — ось управляет скоростью (рысканье
        // в полёте), признак расходимости не проверяется.
        bool angleErrorValid = false;
        float errorDeg = 0;

        float rateDps = 0;           // угловая скорость по гироскопу
        float correctionUs = 0;      // вклад автопилота в руль (без стика), мкс

        float effectiveness = 0;     // оценка b со знаком
        bool estimatorConfident = false;
        float expectedEffectiveness = 0;   // априорная |b| на текущей скорости

        bool analysisAllowed = false;   // в воздухе, не сваливание, закрылки стоят
        bool stickQuiet = false;        // пилот этой осью не рулит
    };

    // Начало нового полёта (арминг).
    void reset()
    {
        disabled = false;
        reason = "";
        reversalPending = false;
        divergencePending = false;
    }

    void update(const Inputs& in, uint32_t nowMs)
    {
        if (disabled) return;

        if (!in.analysisAllowed)
        {
            reversalPending = false;
            divergencePending = false;
            return;
        }

        if (reversedByEstimator(in, nowMs))
        {
            disable("реакция на руль обратная (оценка b < 0)");
        }
        else if (divergenceDetected(in, nowMs))
        {
            disable("расходимость: рулит к цели, а уходит от неё");
        }
    }

    bool isEnabled() const { return !disabled; }
    const char* getStateName() const { return disabled ? "DISABLED" : "OK"; }

    // Почему ось выключена ("" — не выключена).
    const char* getReason() const { return reason; }


private:

    bool disabled = false;
    const char* reason = "";

    // Признак 1: оценка b отрицательная.
    bool reversalPending = false;
    uint32_t reversalSinceMs = 0;

    // Признак 2: расходимость.
    bool divergencePending = false;
    uint32_t divergenceSinceMs = 0;
    float divergenceStartError = 0;

    void disable(const char* why)
    {
        disabled = true;
        reason = why;
    }

    bool reversedByEstimator(const Inputs& in, uint32_t nowMs)
    {
        const bool reversed =
            in.estimatorConfident &&
            in.effectiveness < 0 &&
            -in.effectiveness >= FeedbackConfig::REVERSAL_EFFECTIVENESS_RATIO * in.expectedEffectiveness;

        if (!reversed)
        {
            reversalPending = false;
            return false;
        }
        if (!reversalPending)
        {
            reversalPending = true;
            reversalSinceMs = nowMs;
        }
        return nowMs - reversalSinceMs >= FeedbackConfig::REVERSAL_CONFIRM_MS;
    }

    bool divergenceDetected(const Inputs& in, uint32_t nowMs)
    {
        // Оценка уверенно говорит, что ось работает правильно, — это
        // не перепутанная ось, а внешнее возмущение.
        const bool estimatorAgrees = in.estimatorConfident && in.effectiveness > 0;

        const int8_t towardTarget = FeedbackMath::signOf(in.errorDeg);
        const bool pushingToTarget =
            fabsf(in.correctionUs) >= FeedbackConfig::DIVERGENCE_MIN_COMMAND_US &&
            FeedbackMath::signOf(in.correctionUs) == towardTarget;
        const bool rotatingAway = in.rateDps * towardTarget < -FeedbackConfig::DIVERGENCE_RATE_DPS;

        if (!in.angleErrorValid || !in.stickQuiet || estimatorAgrees || towardTarget == 0 ||
            !pushingToTarget || !rotatingAway)
        {
            divergencePending = false;
            return false;
        }

        const float error = fabsf(in.errorDeg);
        if (!divergencePending)
        {
            divergencePending = true;
            divergenceSinceMs = nowMs;
            divergenceStartError = error;
            return false;
        }

        return nowMs - divergenceSinceMs >= FeedbackConfig::DIVERGENCE_WINDOW_MS &&
               error - divergenceStartError >= FeedbackConfig::DIVERGENCE_ERROR_GROWTH_DEG;
    }
};
