// Реализация autopilot/feedback/LandingSequencer.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/feedback/LandingSequencer.h"


auto LandingSequencer::reset() -> void
{
    state = State::Idle;
    targets = PhaseTargets();
}

auto LandingSequencer::cancel() -> void
{
    if (isActive()) state = State::Aborted;
    targets = PhaseTargets();
}

auto LandingSequencer::update(const FlightSnapshot& s, uint32_t nowMs) -> void
{
    advance(s, nowMs);
    targets = targetsFor(state, s);
}

auto LandingSequencer::isActive() const -> bool
{
    return state == State::Approach || state == State::Flare || state == State::Rollout;
}

auto LandingSequencer::getStateName() const -> const char*
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

auto LandingSequencer::enter(State next, uint32_t nowMs) -> void
{
    state = next;
    stateSinceMs = nowMs;
    stillPending = false;
}

auto LandingSequencer::advance(const FlightSnapshot& s, uint32_t nowMs) -> void
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

auto LandingSequencer::targetsFor(State st, const FlightSnapshot& s) const -> PhaseTargets
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

auto LandingSequencer::heightAboveGround(const FlightSnapshot& s, float& height) -> bool
{
    if (s.heightAglValid) { height = s.heightAglM; return true; }
    if (s.baroValid) { height = s.altitudeM; return true; }
    return false;
}

auto LandingSequencer::sinkRatePitch(const FlightSnapshot& s, float sinkRateMs,
                               float basePitchDeg, float minPitchDeg) -> float
{
    if (!s.baroValid) return basePitchDeg;

    const float sinkError = -sinkRateMs - s.climbRateMs;
    return constrain(basePitchDeg + FeedbackConfig::SINK_TO_PITCH_GAIN * sinkError,
                     minPitchDeg, FeedbackConfig::FLARE_MAX_PITCH_DEG);
}

auto LandingSequencer::touchdownDetected(const FlightSnapshot& s, uint32_t nowMs) -> bool
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
