#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FlightSnapshot.h"
#include "autopilot/feedback/PhaseTargets.h"

// ============================================================
// LANDING SEQUENCER — автоматическая посадка по датчикам
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
//   Approach: газ APPROACH_THROTTLE_PERCENT, снижение с постоянной
//     вертикальной скоростью APPROACH_SINK_RATE_MS. Тангаж — от
//     реального снижения: падаем быстрее нужного — нос чуть вверх,
//     медленнее — вниз. Крен — от пилота (он доворачивает на полосу),
//     не больше APPROACH_MAX_BANK_DEG.
//       ── высота ≤ FLARE_HEIGHT_M ──►
//   Flare (выравнивание): газ 0, крылья ровно, снижение гасится до
//     FLARE_SINK_RATE_MS — тем же правилом "тангаж от вертикальной
//     скорости", нос от 0 до FLARE_MAX_PITCH_DEG. По мере потери
//     скорости нос поднимается сам — ровно настолько, насколько нужно.
//       ── касание: удар по акселерометру или высота ≈ 0 и самолёт
//          не вращается TOUCHDOWN_STILL_MS ──►
//   Rollout (пробег): газ 0, крылья ровно, курс держат руль
//     направления и колесо, ROLLOUT_MS ──► Complete
//
// Высота: дальномер (heightAgl), если есть; иначе барометр от точки
// включения — только если садимся туда же, откуда взлетали, и с
// ошибкой ~метр, поэтому выравнивание по барометру грубое.
//
// Уход на второй круг: пилот даёт газ ≥ GO_AROUND_THROTTLE_PERCENT
// или cancel() — посадка отменяется, управляет пилот.
// ============================================================

class LandingSequencer
{
public:

    enum class State : uint8_t { Idle, Approach, Flare, Rollout, Complete, Aborted };

    void reset()
    {
        state = State::Idle;
        targets = PhaseTargets();
    }

    void request(uint32_t nowMs) { enter(State::Approach, nowMs); }

    void cancel()
    {
        if (isActive()) state = State::Aborted;
        targets = PhaseTargets();
    }

    // Сначала переход (не больше одного за такт), потом цели того
    // этапа, в котором оказались, — так в такт смены этапа не уходят
    // цели прошлого (например, газ снижения в первом такте выравнивания).
    void update(const FlightSnapshot& s, uint32_t nowMs)
    {
        advance(s, nowMs);
        targets = targetsFor(state, s);
    }

    const PhaseTargets& getTargets() const { return targets; }
    State getState() const { return state; }

    bool isActive() const
    {
        return state == State::Approach || state == State::Flare || state == State::Rollout;
    }

    // Выравнивание и пробег — у самой земли, где сваливание — это
    // и есть посадка: защите от сваливания здесь не место.
    bool isNearGround() const { return state == State::Flare || state == State::Rollout; }

    // Колёса уже на земле.
    bool isOnGround() const { return state == State::Rollout || state == State::Complete; }

    const char* getStateName() const
    {
        switch (state)
        {
            case State::Idle:     return "IDLE";
            case State::Approach: return "APPROACH";
            case State::Flare:    return "FLARE";
            case State::Rollout:  return "ROLLOUT";
            case State::Complete: return "COMPLETE";
            case State::Aborted:  return "ABORTED";
        }
        return "?";
    }


private:

    State state = State::Idle;
    uint32_t stateSinceMs = 0;
    float headingDeg = 0;
    PhaseTargets targets;

    bool stillPending = false;
    uint32_t stillSinceMs = 0;

    void enter(State next, uint32_t nowMs)
    {
        state = next;
        stateSinceMs = nowMs;
        stillPending = false;
    }

    void advance(const FlightSnapshot& s, uint32_t nowMs)
    {
        float height;
        switch (state)
        {
            case State::Approach:
                if (s.pilotThrottlePercent >= FeedbackConfig::GO_AROUND_THROTTLE_PERCENT)
                {
                    state = State::Aborted;
                }
                else if (heightAboveGround(s, height) && height <= FeedbackConfig::FLARE_HEIGHT_M)
                {
                    enter(State::Flare, nowMs);
                }
                return;

            case State::Flare:
                if (touchdownDetected(s, nowMs))
                {
                    headingDeg = s.yawDeg;
                    enter(State::Rollout, nowMs);
                }
                return;

            case State::Rollout:
                if (nowMs - stateSinceMs >= FeedbackConfig::ROLLOUT_MS) enter(State::Complete, nowMs);
                return;

            case State::Idle:
            case State::Complete:
            case State::Aborted:
                return;
        }
    }

    PhaseTargets targetsFor(State st, const FlightSnapshot& s) const
    {
        PhaseTargets t;
        switch (st)
        {
            case State::Approach:
                t.active = true;
                t.targetRollDeg = constrain(s.targetRollDeg,
                                            -FeedbackConfig::APPROACH_MAX_BANK_DEG,
                                            FeedbackConfig::APPROACH_MAX_BANK_DEG);
                t.targetPitchDeg = sinkRatePitch(s, FeedbackConfig::APPROACH_SINK_RATE_MS,
                                                 FeedbackConfig::APPROACH_BASE_PITCH_DEG,
                                                 FeedbackConfig::APPROACH_MIN_PITCH_DEG);
                t.throttlePercent = FeedbackConfig::APPROACH_THROTTLE_PERCENT;
                t.reason = "посадка: снижение";
                break;

            case State::Flare:
                t.active = true;
                t.targetRollDeg = 0;
                t.targetPitchDeg = sinkRatePitch(s, FeedbackConfig::FLARE_SINK_RATE_MS, 0.0f, 0.0f);
                t.throttlePercent = 0;
                t.reason = "посадка: выравнивание";
                break;

            case State::Rollout:
                t.active = true;
                t.targetRollDeg = 0;
                t.controlPitch = false;
                t.holdHeading = true;
                t.headingDeg = headingDeg;
                t.throttlePercent = 0;
                t.reason = "посадка: пробег";
                break;

            case State::Idle:
            case State::Complete:
            case State::Aborted:
                break;
        }
        return t;
    }

    static bool heightAboveGround(const FlightSnapshot& s, float& height)
    {
        if (s.heightAglValid) { height = s.heightAglM; return true; }
        if (s.baroValid) { height = s.altitudeM; return true; }
        return false;
    }

    // Тангаж по ошибке вертикальной скорости: снижаемся быстрее
    // нужного (climbRate −2 при нужных −1) — нос вверх на
    // SINK_TO_PITCH_GAIN° за каждый лишний м/с. Без барометра —
    // базовый угол.
    static float sinkRatePitch(const FlightSnapshot& s, float sinkRateMs,
                               float basePitchDeg, float minPitchDeg)
    {
        if (!s.baroValid) return basePitchDeg;

        const float sinkError = -sinkRateMs - s.climbRateMs;
        return constrain(basePitchDeg + FeedbackConfig::SINK_TO_PITCH_GAIN * sinkError,
                         minPitchDeg, FeedbackConfig::FLARE_MAX_PITCH_DEG);
    }

    bool touchdownDetected(const FlightSnapshot& s, uint32_t nowMs)
    {
        // Удар колёс о землю — всплеск перегрузки.
        const float gLoad = sqrtf(s.accelXg * s.accelXg + s.accelYg * s.accelYg + s.accelZg * s.accelZg);
        if (fabsf(gLoad - 1.0f) >= FeedbackConfig::TOUCHDOWN_ACCEL_G) return true;

        // Мягкое касание: высота ≈ 0, и самолёт перестал вращаться.
        float height;
        const bool low = heightAboveGround(s, height) && height <= FeedbackConfig::TOUCHDOWN_HEIGHT_M;
        const bool still = fabsf(s.rollRateDps) < FeedbackConfig::TOUCHDOWN_STILL_RATE_DPS &&
                           fabsf(s.pitchRateDps) < FeedbackConfig::TOUCHDOWN_STILL_RATE_DPS;
        if (!low || !still)
        {
            stillPending = false;
            return false;
        }
        if (!stillPending)
        {
            stillPending = true;
            stillSinceMs = nowMs;
        }
        return nowMs - stillSinceMs >= FeedbackConfig::TOUCHDOWN_STILL_MS;
    }
};
