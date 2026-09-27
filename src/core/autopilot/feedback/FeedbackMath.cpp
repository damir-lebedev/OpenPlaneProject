// Реализация autopilot/feedback/FeedbackMath.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/feedback/FeedbackMath.h"

namespace FeedbackMath
{

float wrap180(float deg)
{
    deg = fmodf(deg, 360.0f);
    if (deg > 180.0f) deg -= 360.0f;
    if (deg <= -180.0f) deg += 360.0f;
    return deg;
}

int8_t signOf(float x)
{
    return static_cast<int8_t>(x > 0.0f ? 1 : (x < 0.0f ? -1 : 0));
}

float clampAbs(float x, float limit)
{
    return x > limit ? limit : (x < -limit ? -limit : x);
}

}  // namespace FeedbackMath
