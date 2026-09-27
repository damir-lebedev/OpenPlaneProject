// Реализация telemetry/LoopStats.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "telemetry/LoopStats.h"


auto LoopStats::record(uint32_t durationUs) -> void
{
    count++;
    sumUs += durationUs;
    if (durationUs > windowMaxUs) windowMaxUs = durationUs;
    if (durationUs > peakUs) peakUs = durationUs;

    const uint32_t now = millis();
    if (now - windowStartMs >= 1000)
    {
        hz = count * 1000 / (now - windowStartMs);
        avgUs = count ? sumUs / count : 0;
        maxUs = windowMaxUs;

        count = 0;
        sumUs = 0;
        windowMaxUs = 0;
        windowStartMs = now;
    }
}

auto LoopStats::takePeakUs() -> uint32_t
{
    const uint32_t peak = peakUs;
    peakUs = 0;
    return peak;
}
