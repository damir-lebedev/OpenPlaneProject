#pragma once
#include <stdint.h>

#include "autopilot/feedback/FeedbackConfig.h"

// ============================================================
// FEEDBACK OUTPUT — что контур обратной связи хочет сделать
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// Результат FeedbackSupervisor::update() за такт. При подключении
// FlightController будет применять его так:
//   • deflectionUs — отклонения рулей от контура обратной связи
//     (вместо коррекций ПИД из Autopilot, в тех же знаках, что
//     ControlCommand); ось с axisEnabled = false не трогается;
//   • throttle — throttleOverride (если задан) или не меньше
//     throttleFloorPercent;
//   • reason — короткое описание для лога и OLED.
// ============================================================

struct FeedbackOutput
{
    // Отклонения рулей по осям (крен, тангаж, рысканье), мкс.
    float deflectionUs[FeedbackConfig::AXIS_COUNT] = { 0, 0, 0 };

    // false — ось не управляется (режим MANUAL, этап полёта её не
    // трогает, или ось работает наоборот и выключена): рули этой оси
    // остаются на стиках пилота.
    bool axisEnabled[FeedbackConfig::AXIS_COUNT] = { true, true, true };

    // Газ, %: абсолютное значение (взлёт/посадка) или нижняя граница
    // (защита от сваливания). Отрицательное — не задано.
    float throttleOverridePercent = -1.0f;
    float throttleFloorPercent = -1.0f;

    // Итоговые цели по углам после всех ограничений — для отладки.
    float targetRollDeg = 0;
    float targetPitchDeg = 0;

    const char* reason = "";
};
