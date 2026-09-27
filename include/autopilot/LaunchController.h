#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// ЗАПУСК С РУКИ (MODE_LAUNCH)
//
//   IDLE ──ARM + газ поднят──► READY ("взведён", мотор стоит)
//   READY ──газ убран──► IDLE
//   READY ──бросок: ускорение вперёд > LAUNCH_ACCEL_G дольше
//           LAUNCH_ACCEL_TIME_MS──► THROWN (мотор ещё стоит —
//           рука уходит от винта)
//   THROWN ──LAUNCH_MOTOR_DELAY_MS──► CLIMB (газ LAUNCH_THROTTLE_PCT,
//           тангаж LAUNCH_CLIMB_PITCH_DEG, крылья ровно)
//   CLIMB ──LAUNCH_CLIMB_MS, или высота LAUNCH_ALTITUDE_M, или пилот
//           взял стики──► DONE (дальше автопилот держит курс и высоту)
//   любое ──DISARM──► IDLE
//
// Взвод газом — защита от случайного старта мотора, когда
// заармленный самолёт несут к месту запуска и трясут: без поднятого
// газа бросок не распознаётся вовсе (так же в INAV NAV LAUNCH).
// ============================================================

class LaunchController
{
public:

    enum class State : uint8_t { IDLE, READY, THROWN, CLIMB, DONE };

    void reset();

    void update(bool armed, bool throttleRaised, float forwardAccelG, bool pilotSticksMoved,
                float altitudeM, uint32_t nowMs);

    State getState() const { return state; }

    // Мотор разрешён только в наборе; после DONE газом управляет круиз.
    bool motorOn() const { return state == State::CLIMB; }

    // До броска — горизонт (самолёт в руке), после — угол набора.
    float pitchTargetDeg() const;

    static const char* stateName(State s);


private:

    State state = State::IDLE;
    uint32_t accelSinceMs = 0;
    uint32_t stateSinceMs = 0;
};
