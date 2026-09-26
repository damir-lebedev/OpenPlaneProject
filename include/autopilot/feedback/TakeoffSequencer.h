#pragma once
#include <Arduino.h>

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FlightSnapshot.h"
#include "autopilot/feedback/PhaseTargets.h"
#include "autopilot/feedback/SpeedEstimator.h"

// ============================================================
// TAKEOFF SEQUENCER — автоматический взлёт по датчикам
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// Будущая замена режима AUTO_TAKEOFF из Autopilot: там взлёт — это
// просто "тангаж 15° и газ", а здесь этапы переключаются по тому,
// что реально происходит с самолётом.
//
//   WaitThrottle ── пилот дал газ ≥ TAKEOFF_TRIGGER ──┐
//                                                    │
//   С полосы (TAKEOFF_HAND_LAUNCH = false):          │
//     GroundRoll: полный газ, крылья ровно, курс держат руль
//       направления и колесо, руль высоты свободен. Отрыв — по
//       скорости ROTATE_SPEED_MS (или через ROTATE_FALLBACK_MS без
//       датчика скорости) ──► Climb
//                                                    │
//   С руки (TAKEOFF_HAND_LAUNCH = true):             │
//     WaitLaunch: мотор стоит (винт у руки!), ждём бросок — продольное
//       ускорение ≥ LAUNCH_ACCEL_G дольше LAUNCH_DETECT_MS ──► Climb
//                                                    │
//   Climb: полный газ, крылья ровно, тангаж CLIMB_PITCH_DEG — пока
//     высота не достигнет TAKEOFF_TARGET_ALTITUDE_M ──► Complete
//
// Отмена (Aborted): пилот убрал газ до отрыва, за LAUNCH_TIMEOUT_MS
// так и не взлетели, cancel(). Дальше управляет пилот.
//
// Защита от сваливания (StallGuard) стоит выше: если в наборе
// высоты скорость уходит, она опустит нос, что бы ни просил взлёт.
// ============================================================

class TakeoffSequencer
{
public:

    enum class State : uint8_t
    {
        Idle, WaitThrottle, WaitLaunch, GroundRoll, Climb, Complete, Aborted
    };

    void reset()
    {
        state = State::Idle;
        targets = PhaseTargets();
    }

    void request(uint32_t nowMs)
    {
        enter(State::WaitThrottle, nowMs);
        targets = targetsFor(state);
    }

    void cancel()
    {
        if (isActive()) state = State::Aborted;
        targets = PhaseTargets();
    }

    // Сначала переход (не больше одного за такт), потом цели того
    // этапа, в котором оказались, — так в такт смены этапа не уходят
    // цели прошлого.
    void update(const FlightSnapshot& s, const SpeedEstimator& speed, uint32_t nowMs)
    {
        advance(s, speed, nowMs);
        targets = targetsFor(state);
    }

    const PhaseTargets& getTargets() const { return targets; }
    State getState() const { return state; }

    bool isActive() const
    {
        return state == State::WaitThrottle || state == State::WaitLaunch ||
               state == State::GroundRoll || state == State::Climb;
    }

    // Самолёт уже в воздухе по мнению взлёта.
    bool isAirborne() const { return state == State::Climb || state == State::Complete; }

    const char* getStateName() const
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


private:

    State state = State::Idle;
    uint32_t stateSinceMs = 0;
    float headingDeg = 0;
    PhaseTargets targets;

    bool launchPending = false;
    uint32_t launchSinceMs = 0;

    void enter(State next, uint32_t nowMs)
    {
        state = next;
        stateSinceMs = nowMs;
        launchPending = false;
    }

    void advance(const FlightSnapshot& s, const SpeedEstimator& speed, uint32_t nowMs)
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

    PhaseTargets targetsFor(State st) const
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

    // Бросок: акселерометр по X минус проекция тяжести.
    bool launchDetected(const FlightSnapshot& s, uint32_t nowMs)
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

    bool rotateReached(const SpeedEstimator& speed, uint32_t inStateMs) const
    {
        if (speed.hasSpeed()) return speed.getSpeed() >= FeedbackConfig::ROTATE_SPEED_MS;
        return inStateMs >= FeedbackConfig::ROTATE_FALLBACK_MS;
    }

    bool climbComplete(const FlightSnapshot& s, uint32_t inStateMs) const
    {
        if (s.baroValid) return s.altitudeM >= FeedbackConfig::TAKEOFF_TARGET_ALTITUDE_M;
        return inStateMs >= FeedbackConfig::TAKEOFF_CLIMB_FALLBACK_MS;
    }
};
