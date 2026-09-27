// Реализация autopilot/feedback/TakeoffSequencer.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/feedback/TakeoffSequencer.h"


auto TakeoffSequencer::reset() -> void
{
    state = State::Idle;
    targets = PhaseTargets();
}

auto TakeoffSequencer::request(uint32_t nowMs) -> void
{
    enter(State::WaitThrottle, nowMs);
    targets = targetsFor(state);
}

auto TakeoffSequencer::cancel() -> void
{
    if (isActive()) state = State::Aborted;
    targets = PhaseTargets();
}

auto TakeoffSequencer::update(const FlightSnapshot& s, const SpeedEstimator& speed, uint32_t nowMs) -> void
{
    advance(s, speed, nowMs);
    targets = targetsFor(state);
}

auto TakeoffSequencer::isActive() const -> bool
{
    return state == State::WaitThrottle || state == State::WaitLaunch ||
           state == State::GroundRoll || state == State::Climb;
}

auto TakeoffSequencer::getStateName() const -> const char*
{
    switch (state)
    {
        case State::Idle:         return "IDLE";
        case State::WaitThrottle: return "WAIT_THROTTLE";
        case State::WaitLaunch:   return "WAIT_LAUNCH";
        case State::GroundRoll:   return "GROUND_ROLL";
        case State::Climb:        return "CLIMB";
        case State::Complete:     return "COMPLETE";
        case State::Aborted:      return "ABORTED";
    }
    return "?";
}

auto TakeoffSequencer::enter(State next, uint32_t nowMs) -> void
{
    state = next;
    stateSinceMs = nowMs;
    launchPending = false;
}

auto TakeoffSequencer::advance(const FlightSnapshot& s, const SpeedEstimator& speed, uint32_t nowMs) -> void
{
    const uint32_t inState = nowMs - stateSinceMs;
    const bool throttleUp = s.pilotThrottlePercent >= FeedbackConfig::TAKEOFF_TRIGGER_THROTTLE_PERCENT;

    switch (state)
    {
        case State::WaitThrottle:
            if (throttleUp)
            {
                headingDeg = s.yawDeg;
                enter(FeedbackConfig::TAKEOFF_HAND_LAUNCH ? State::WaitLaunch : State::GroundRoll, nowMs);
            }
            return;

        case State::WaitLaunch:
            if (!throttleUp) enter(State::WaitThrottle, nowMs);
            else if (launchDetected(s, nowMs)) enter(State::Climb, nowMs);
            else if (inState > FeedbackConfig::LAUNCH_TIMEOUT_MS) state = State::Aborted;
            return;

        case State::GroundRoll:
            if (throttleUp && rotateReached(speed, inState)) enter(State::Climb, nowMs);
            else if (!throttleUp || inState > FeedbackConfig::LAUNCH_TIMEOUT_MS) state = State::Aborted;
            return;

        case State::Climb:
            if (climbComplete(s, inState)) enter(State::Complete, nowMs);
            return;

        case State::Idle:
        case State::Complete:
        case State::Aborted:
            return;
    }
}

auto TakeoffSequencer::targetsFor(State st) const -> PhaseTargets
{
    PhaseTargets t;
    switch (st)
    {
        case State::WaitThrottle:
        case State::WaitLaunch:
            // До старта: мотор стоит (при броске с руки — до самого
            // броска), рули у пилота.
            t.active = true;
            t.controlRoll = false;
            t.controlPitch = false;
            t.throttlePercent = 0;
            t.reason = st == State::WaitThrottle ? "взлёт: дайте газ" : "взлёт: бросайте";
            break;

        case State::GroundRoll:
            t.active = true;
            t.targetRollDeg = 0;
            t.controlPitch = false;
            t.holdHeading = true;
            t.headingDeg = headingDeg;
            t.throttlePercent = FeedbackConfig::TAKEOFF_THROTTLE_PERCENT;
            t.reason = "взлёт: разбег";
            break;

        case State::Climb:
            t.active = true;
            t.targetRollDeg = 0;
            t.targetPitchDeg = FeedbackConfig::CLIMB_PITCH_DEG;
            t.throttlePercent = FeedbackConfig::TAKEOFF_THROTTLE_PERCENT;
            t.reason = "взлёт: набор высоты";
            break;

        case State::Idle:
        case State::Complete:
        case State::Aborted:
            break;
    }
    return t;
}

auto TakeoffSequencer::launchDetected(const FlightSnapshot& s, uint32_t nowMs) -> bool
{
    const float longitudinalG = s.accelXg - sinf(s.pitchDeg * static_cast<float>(DEG_TO_RAD));
    if (longitudinalG < FeedbackConfig::LAUNCH_ACCEL_G)
    {
        launchPending = false;
        return false;
    }
    if (!launchPending)
    {
        launchPending = true;
        launchSinceMs = nowMs;
    }
    return nowMs - launchSinceMs >= FeedbackConfig::LAUNCH_DETECT_MS;
}

auto TakeoffSequencer::rotateReached(const SpeedEstimator& speed, uint32_t inStateMs) const -> bool
{
    if (speed.hasSpeed()) return speed.getSpeed() >= FeedbackConfig::ROTATE_SPEED_MS;
    return inStateMs >= FeedbackConfig::ROTATE_FALLBACK_MS;
}

auto TakeoffSequencer::climbComplete(const FlightSnapshot& s, uint32_t inStateMs) const -> bool
{
    if (s.baroValid) return s.altitudeM >= FeedbackConfig::TAKEOFF_TARGET_ALTITUDE_M;
    return inStateMs >= FeedbackConfig::TAKEOFF_CLIMB_FALLBACK_MS;
}
