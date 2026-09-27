// Реализация autopilot/LaunchController.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/LaunchController.h"


auto LaunchController::reset() -> void
{
    state = State::IDLE;
    accelSinceMs = 0;
}

auto LaunchController::update(bool armed, bool throttleRaised, float forwardAccelG, bool pilotSticksMoved,
                float altitudeM, uint32_t nowMs) -> void
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

auto LaunchController::pitchTargetDeg() const -> float
{
    return state == State::THROWN || state == State::CLIMB ? Config::LAUNCH_CLIMB_PITCH_DEG : 0.0f;
}

auto LaunchController::stateName(State s) -> const char*
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
