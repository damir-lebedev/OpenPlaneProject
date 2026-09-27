// Реализация control/Beeper.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "control/Beeper.h"


auto Beeper::update(bool requested, bool armed, bool linkLost, uint32_t nowMs) -> bool
{
    if (linkLost && !armed)
    {
        if (lostSinceMs == 0) lostSinceMs = nowMs == 0 ? 1 : nowMs;
    }
    else
    {
        lostSinceMs = 0;
    }

    lostModel = lostSinceMs != 0 && nowMs - lostSinceMs >= Config::LOST_MODEL_BEEP_DELAY_MS;
    if (!requested && !lostModel) return false;

    return (nowMs / HALF_PERIOD_MS) % 2 == 0;
}
