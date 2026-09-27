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
    float wrap180(float deg);

    int8_t signOf(float x);

    float clampAbs(float x, float limit);
}
