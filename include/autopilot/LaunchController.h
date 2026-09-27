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

    void reset()
    {
        state = State::IDLE;
        accelSinceMs = 0;
    }

    void update(bool armed, bool throttleRaised, float forwardAccelG, bool pilotSticksMoved,
                float altitudeM, uint32_t nowMs)
    {
        if (!armed)
        {
            reset();
            return;
        }

        switch (state)
        {
            case State::IDLE:
                if (throttleRaised)
                {
                    state = State::READY;
                    Serial.println("Launch: взведён — бросайте");
                }
                break;

            case State::READY:
                if (!throttleRaised)
                {
                    reset();
                    break;
                }
                if (forwardAccelG < Config::LAUNCH_ACCEL_G)
                {
                    accelSinceMs = 0;
                    break;
                }
                if (accelSinceMs == 0) accelSinceMs = nowMs == 0 ? 1 : nowMs;
                if (nowMs - accelSinceMs >= Config::LAUNCH_ACCEL_TIME_MS)
                {
                    state = State::THROWN;
                    stateSinceMs = nowMs;
                    Serial.println("Launch: бросок — мотор через паузу");
                }
                break;

            case State::THROWN:
                if (nowMs - stateSinceMs >= Config::LAUNCH_MOTOR_DELAY_MS)
                {
                    state = State::CLIMB;
                    stateSinceMs = nowMs;
                    Serial.println("Launch: мотор, набор высоты");
                }
                break;

            case State::CLIMB:
                if (pilotSticksMoved || altitudeM >= Config::LAUNCH_ALTITUDE_M ||
                    nowMs - stateSinceMs >= Config::LAUNCH_CLIMB_MS)
                {
                    state = State::DONE;
                    Serial.println("Launch: запуск завершён — держу курс и высоту");
                }
                break;

            case State::DONE:
                break;
        }
    }

    State getState() const { return state; }

    // Мотор разрешён только в наборе; после DONE газом управляет круиз.
    bool motorOn() const { return state == State::CLIMB; }

    // До броска — горизонт (самолёт в руке), после — угол набора.
    float pitchTargetDeg() const
    {
        return state == State::THROWN || state == State::CLIMB ? Config::LAUNCH_CLIMB_PITCH_DEG : 0.0f;
    }

    static const char* stateName(State s)
    {
        switch (s)
        {
            case State::IDLE:   return "IDLE";
            case State::READY:  return "READY";
            case State::THROWN: return "THROWN";
            case State::CLIMB:  return "CLIMB";
            case State::DONE:   return "DONE";
            default:            return "?";
        }
    }


private:

    State state = State::IDLE;
    uint32_t accelSinceMs = 0;
    uint32_t stateSinceMs = 0;
};
