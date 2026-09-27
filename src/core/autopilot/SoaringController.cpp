// Реализация autopilot/SoaringController.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/SoaringController.h"


auto SoaringController::reset(uint32_t nowMs) -> void
{
    state = State::GLIDE;
    stateSinceMs = nowMs;
    liftSinceMs = 0;
    averageClimb = 0;
}

auto SoaringController::update(float climbMs, float altitudeM, float distanceHomeM, float dtS, uint32_t nowMs) -> void
{
    if (state != State::MOTOR_CLIMB && altitudeM < Config::SOAR_MIN_ALTITUDE_M)
    {
        enter(State::MOTOR_CLIMB, nowMs, "низко — набор с мотором");
        return;
    }

    switch (state)
    {
        case State::MOTOR_CLIMB:
            if (altitudeM >= Config::SOAR_MAX_ALTITUDE_M)
            {
                enter(State::GLIDE, nowMs, "высота набрана — планирую");
            }
            break;

        case State::RETURN:
            if (distanceHomeM >= 0 && distanceHomeM < Config::SOAR_MAX_DISTANCE_M * 0.7f)
            {
                enter(State::GLIDE, nowMs, "снова рядом с домом");
            }
            break;

        case State::GLIDE:
            if (distanceHomeM > Config::SOAR_MAX_DISTANCE_M)
            {
                enter(State::RETURN, nowMs, "далеко — к дому");
                break;
            }
            if (climbMs > Config::SOAR_THERMAL_CLIMB_MS)
            {
                if (liftSinceMs == 0) liftSinceMs = nowMs == 0 ? 1 : nowMs;
                if (nowMs - liftSinceMs >= Config::SOAR_THERMAL_CONFIRM_MS)
                {
                    averageClimb = climbMs;
                    enter(State::THERMAL, nowMs, "термик! круги");
                }
            }
            else
            {
                liftSinceMs = 0;
            }
            break;

        case State::THERMAL:
        {
            const float tau = Config::SOAR_EXIT_WINDOW_MS / 3000.0f;
            averageClimb += dtS / (tau + dtS) * (climbMs - averageClimb);
            if (nowMs - stateSinceMs >= Config::SOAR_EXIT_WINDOW_MS &&
                averageClimb < Config::SOAR_EXIT_CLIMB_MS)
            {
                enter(State::GLIDE, nowMs, "термик кончился — планирую дальше");
            }
            break;
        }
    }
}

auto SoaringController::stateName(State s) -> const char*
{
    switch (s)
    {
        case State::GLIDE:       return "GLIDE";
        case State::THERMAL:     return "THERMAL";
        case State::MOTOR_CLIMB: return "MOTOR_CLIMB";
        case State::RETURN:      return "RETURN";
        default:                 return "?";
    }
}

auto SoaringController::enter(State next, uint32_t nowMs, const char* why) -> void
{
    state = next;
    stateSinceMs = nowMs;
    liftSinceMs = 0;
    Serial.print("Soaring: ");
    Serial.println(why);
}
