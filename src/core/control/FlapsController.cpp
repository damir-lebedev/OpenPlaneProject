// Реализация control/FlapsController.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "control/FlapsController.h"


auto FlapsController::update(float target, uint32_t nowMs) -> int16_t
{

    // Первый вызов (включение платы) — сразу в целевое положение,
    // без "выезда" закрылков на столе.
    if (!initialized)
    {
        positionUs = target;
        lastUpdateMs = nowMs;
        initialized = true;
        return getPosition();
    }

    // Шаг по времени ограничен: если update() долго не вызывался
    // (failsafe, калибровка из консоли), закрылки не должны
    // прыгнуть к цели за один цикл.
    const uint32_t elapsedMs = min<uint32_t>(nowMs - lastUpdateMs, MAX_STEP_MS);
    lastUpdateMs = nowMs;

    const float maxStep =
        static_cast<float>(Config::FLAPS_DEPLOYED_US) * elapsedMs / Config::FLAPS_TRANSITION_MS;

    positionUs += constrain(target - positionUs, -maxStep, maxStep);

    return getPosition();
}

auto FlapsController::getPosition() const -> int16_t
{
    return static_cast<int16_t>(positionUs);
}
