#pragma once
#include <Arduino.h>

// ============================================================
// LOOP STATS
//
// Частота и длительность основного цикла (FlightController::
// update() и всё, что крутится рядом в loop()). main.cpp вызывает
// record() каждый цикл; раз в секунду значения публикуются в
// hz/avgUs/maxUs — их читают DebugLogger и OledDisplay (в том
// числе из другой задачи: поля 32-битные, рваного чтения нет).
// ============================================================

struct LoopStats
{
    volatile uint32_t hz = 0;
    volatile uint32_t avgUs = 0;
    volatile uint32_t maxUs = 0;

    void record(uint32_t durationUs)
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

    // Худшее время цикла с прошлого вызова — для строки SYS раз в 10 с:
    // maxUs — только за последнюю секунду, редкий затык в нём не виден.
    // Вызывать из той же задачи, что и record() (loop).
    uint32_t takePeakUs()
    {
        const uint32_t peak = peakUs;
        peakUs = 0;
        return peak;
    }

private:
    uint32_t count = 0;
    uint32_t sumUs = 0;
    uint32_t windowMaxUs = 0;
    uint32_t windowStartMs = 0;
    uint32_t peakUs = 0;
};
