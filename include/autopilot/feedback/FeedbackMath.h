#pragma once
#include <math.h>
#include <stdint.h>

// ============================================================
// FEEDBACK MATH — мелкие общие функции модулей обратной связи
// ============================================================

namespace FeedbackMath
{
    // Угол в диапазон (−180, 180]: разница курсов 350° и 10° — это
    // −20°, а не 340°.
    inline float wrap180(float deg)
    {
        deg = fmodf(deg, 360.0f);
        if (deg > 180.0f) deg -= 360.0f;
        if (deg <= -180.0f) deg += 360.0f;
        return deg;
    }

    inline int8_t signOf(float x)
    {
        return x > 0.0f ? 1 : (x < 0.0f ? -1 : 0);
    }

    inline float clampAbs(float x, float limit)
    {
        return x > limit ? limit : (x < -limit ? -limit : x);
    }
}
