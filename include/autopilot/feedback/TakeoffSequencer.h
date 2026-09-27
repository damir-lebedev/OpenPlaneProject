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

    void reset();

    void request(uint32_t nowMs);

    void cancel();

    // Сначала переход (не больше одного за такт), потом цели того
    // этапа, в котором оказались, — так в такт смены этапа не уходят
    // цели прошлого.
    void update(const FlightSnapshot& s, const SpeedEstimator& speed, uint32_t nowMs);

    const PhaseTargets& getTargets() const { return targets; }
    State getState() const { return state; }

    bool isActive() const;

    // Самолёт уже в воздухе по мнению взлёта.
    bool isAirborne() const { return state == State::Climb || state == State::Complete; }

    const char* getStateName() const;


private:

    State state = State::Idle;
    uint32_t stateSinceMs = 0;
    float headingDeg = 0;
    PhaseTargets targets;

    bool launchPending = false;
    uint32_t launchSinceMs = 0;

    void enter(State next, uint32_t nowMs);

    void advance(const FlightSnapshot& s, const SpeedEstimator& speed, uint32_t nowMs);

    PhaseTargets targetsFor(State st) const;

    // Бросок: акселерометр по X минус проекция тяжести.
    bool launchDetected(const FlightSnapshot& s, uint32_t nowMs);

    bool rotateReached(const SpeedEstimator& speed, uint32_t inStateMs) const;

    bool climbComplete(const FlightSnapshot& s, uint32_t inStateMs) const;
};
