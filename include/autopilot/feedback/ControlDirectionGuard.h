#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FeedbackMath.h"

// ============================================================
// CONTROL DIRECTION GUARD — рули работают в ту сторону? Автоинверсия
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// Одна ось. Если автопилот тянет нос вверх, а нос уверенно идёт вниз,
// значит, знак оси перепутан: неправильный *_REVERSED в Config.h,
// IMU повёрнут не так, как указано в IMU_ROTATION_CW_DEG, тяги
// перекинуты. Без защиты автопилот в таком случае не выравнивает
// самолёт, а сам его заваливает, всё сильнее с каждым тактом.
//
// Два независимых признака "знак неверный":
//   • оценка эффективности b (ControlEffectivenessEstimator) уверенно
//     держится с противоположным знаком дольше REVERSAL_CONFIRM_MS;
//   • расходимость: коррекция упорно толкает к цели, а самолёт
//     вращается от цели, и ошибка за DIVERGENCE_WINDOW_MS выросла на
//     DIVERGENCE_ERROR_GROWTH_DEG. Не нужна обученная оценка —
//     срабатывает даже в первые секунды. Игнорируется, если оценка
//     уверенно подтверждает текущий знак (значит, это порыв ветра).
//
// Что происходит дальше (состояния):
//
//   Normal ──признак──► Verifying ──ошибка уменьшается──► Locked (−1)
//     │                    │
//     │                    └──стало хуже──► Locked (знак вернули)
//     └──AUTO_INVERT_ENABLED = false──► Disabled
//   Locked ──снова признак──► Disabled
//
//   Verifying — знак перевёрнут. Первые INVERSION_GRACE_MS не судим
//               (серво доходит, самолёт по инерции докручивается), потом:
//               пошёл к цели — принято; уходит от цели не медленнее, чем
//               при перевороте, — переворот был ложным (например,
//               порыв), знак возвращается. Не ясно за
//               INVERSION_VERIFY_MS — решает, уменьшилась ли ошибка.
//   Locked    — знак определён, больше автоматически не меняется.
//   Disabled  — ни один знак не работает: коррекции по оси
//               выключаются, ось остаётся только на стиках пилота.
//               Бороться в неправильную сторону хуже, чем не бороться.
//
// Анализ — только когда можно доверять наблюдению: в воздухе, не на
// сваливании (там нос падает при любом руле), закрылки стоят.
// Расходимость и проверка по углу — ещё и когда пилот этим стиком не
// двигает (он может вращать самолёт нарочно). Оценке b стик не
// мешает — наоборот, это хорошая раскачка: знак определяется даже в
// MANUAL, до включения стабилизации.
//
// Автоинверсия — страховка, а не настройка: после полёта, в котором
// она сработала, надо исправить Config.h (лог говорит, какая ось).
// Стики пилота она не переворачивает.
// ============================================================

class ControlDirectionGuard
{
public:

    enum class State : uint8_t { Normal, Verifying, Locked, Disabled };

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

    // Начало нового полёта (арминг): знак — снова из Config.h.
    void reset()
    {
        state = State::Normal;
        sign = 1;
        lastEvent = "";
        resetDetectors();
    }

    void update(const Inputs& in, uint32_t nowMs)
    {
        if (state == State::Disabled) return;

        if (state == State::Verifying)
        {
            verify(in, nowMs);
            return;
        }

        if (!in.analysisAllowed)
        {
            resetDetectors();
            return;
        }

        if (reversedByEstimator(in, nowMs))
        {
            onReversalDetected(in, nowMs, "оценка b против знака");
        }
        else if (divergenceDetected(in, nowMs))
        {
            onReversalDetected(in, nowMs, "расходимость");
        }
    }

    // +1 — как в Config.h, −1 — ось инвертирована.
    int8_t getSign() const { return sign; }
    bool isEnabled() const { return state != State::Disabled; }
    State getState() const { return state; }

    const char* getStateName() const
    {
        switch (state)
        {
            case State::Normal:    return "NORMAL";
            case State::Verifying: return "VERIFY";
            case State::Locked:    return sign > 0 ? "LOCKED" : "INVERTED";
            case State::Disabled:  return "DISABLED";
        }
        return "?";
    }

    // Последнее событие ("" — не было) — для лога.
    const char* getLastEvent() const { return lastEvent; }


private:

    State state = State::Normal;
    int8_t sign = 1;
    const char* lastEvent = "";

    // Признак 1: оценка b против знака.
    bool reversalPending = false;
    uint32_t reversalSinceMs = 0;

    // Признак 2: расходимость.
    bool divergencePending = false;
    uint32_t divergenceSinceMs = 0;
    float divergenceStartError = 0;

    // Проверка после переворота.
    uint32_t verifySinceMs = 0;
    float verifyStartError = 0;
    float verifyStartRateAway = 0;

    void resetDetectors()
    {
        reversalPending = false;
        divergencePending = false;
    }

    bool reversedByEstimator(const Inputs& in, uint32_t nowMs)
    {
        const bool reversed =
            in.estimatorConfident &&
            in.effectiveness * sign < 0 &&
            fabsf(in.effectiveness) >= FeedbackConfig::REVERSAL_EFFECTIVENESS_RATIO * in.expectedEffectiveness;

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
        // Оценка уверенно за текущий знак — это не перепутанный знак,
        // а внешнее возмущение.
        const bool estimatorAgrees = in.estimatorConfident && in.effectiveness * sign > 0;

        const int8_t towardTarget = FeedbackMath::signOf(in.errorDeg);
        const bool pushingToTarget =
            fabsf(in.correctionUs) >= FeedbackConfig::DIVERGENCE_MIN_COMMAND_US &&
            FeedbackMath::signOf(in.correctionUs) * sign == towardTarget;
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

    void onReversalDetected(const Inputs& in, uint32_t nowMs, const char* why)
    {
        resetDetectors();
        lastEvent = why;

        if (state == State::Locked || !FeedbackConfig::AUTO_INVERT_ENABLED)
        {
            state = State::Disabled;
            return;
        }

        sign = -sign;
        state = State::Verifying;
        startVerifyWindow(in, nowMs);
    }

    void startVerifyWindow(const Inputs& in, uint32_t nowMs)
    {
        verifySinceMs = nowMs;
        verifyStartError = fabsf(in.errorDeg);
        verifyStartRateAway = rateAwayFromTarget(in);
    }

    // Скорость вращения ОТ цели (> 0 — уходит, < 0 — возвращается).
    static float rateAwayFromTarget(const Inputs& in)
    {
        return -in.rateDps * FeedbackMath::signOf(in.errorDeg);
    }

    void verify(const Inputs& in, uint32_t nowMs)
    {
        if (!in.analysisAllowed)
        {
            startVerifyWindow(in, nowMs);
            return;
        }

        // Уверенная оценка решает сразу.
        if (in.estimatorConfident)
        {
            if (in.effectiveness * sign > 0) { accept(); return; }
            if (in.effectiveness * sign < 0) { revert(); return; }
        }

        // По углу судить можно, только пока пилот не рулит этой осью и
        // ось держит угол. Иначе окно проверки начинается заново (для
        // рысканья в полёте остаётся только оценка b).
        if (!in.stickQuiet || !in.angleErrorValid)
        {
            startVerifyWindow(in, nowMs);
            return;
        }

        const uint32_t elapsed = nowMs - verifySinceMs;
        const float error = fabsf(in.errorDeg);
        const float rateAway = rateAwayFromTarget(in);

        // Стало лучше: ошибка заметно меньше, чем при перевороте.
        if (error <= verifyStartError - FeedbackConfig::DIVERGENCE_ERROR_GROWTH_DEG)
        {
            accept();
            return;
        }

        // Сразу после переворота самолёт ещё докручивается по инерции —
        // ждём, пока серво дойдёт и вращение успеет остановиться.
        if (elapsed < FeedbackConfig::INVERSION_GRACE_MS) return;

        // Самолёт уже возвращается к цели — переворот помог.
        if (rateAway < -FeedbackConfig::DIVERGENCE_RATE_DPS)
        {
            accept();
            return;
        }

        // Всё ещё уходит от цели не медленнее, чем при перевороте, —
        // стало не лучше, а хуже: переворот был ошибкой.
        if (rateAway > FeedbackConfig::DIVERGENCE_RATE_DPS && rateAway >= verifyStartRateAway)
        {
            revert();
            return;
        }

        // Окно прошло: решает, уменьшилась ли ошибка.
        if (elapsed >= FeedbackConfig::INVERSION_VERIFY_MS)
        {
            if (error < verifyStartError) accept();
            else revert();
        }
    }

    void accept()
    {
        state = State::Locked;
        lastEvent = sign > 0 ? "знак подтверждён" : "инверсия подтверждена";
    }

    void revert()
    {
        sign = -sign;
        state = State::Locked;
        lastEvent = "инверсия отменена";
    }
};
