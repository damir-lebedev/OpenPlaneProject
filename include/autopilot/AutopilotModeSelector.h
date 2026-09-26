#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "config/Channels.h"
#include "rc/RcChannelState.h"

// ============================================================
// AUTOPILOT MODE SELECTOR
//
// CH7 (Channels::AUX_2, тумблер SwC) — единственный переключатель режима
// автопилота, 3 положения: <1250 -> MANUAL, 1250..1749 -> STABILIZE,
// >=1750 -> AUTO_TAKEOFF.
//
// Раньше режим выбирался 4 отдельными каналами (CH6/CH7/CH9/CH10)
// через FeatureManager, с приоритетом между ними на случай, если
// несколько активны одновременно. Один канал с тремя положениями
// снимает эту неоднозначность в принципе — включён всегда ровно
// один режим.
//
// ALT_HOLD с RC не выбирается (только через POST /api/setmode на
// дашборде) — поэтому setMode() дёргается только в момент, когда
// CH7 реально переходит в другую зону, а не каждый цикл. Иначе
// этот селектор переписывал бы режим обратно на CH7 каждые ~2мс
// (период FlightController::update()) и ALT_HOLD с дашборда никогда
// бы не удерживался дольше одного цикла.
// ============================================================

class AutopilotModeSelector
{
public:

    explicit AutopilotModeSelector(Autopilot* ap = nullptr)
        : autopilot(ap)
    {
    }

    // Вызывается из FlightController::update() с текущими RC-каналами.
    void update(const RcChannelState& rc)
    {
        if (!autopilot) return;

        const AutopilotMode target = modeFor(rc.get(Channels::AUX_2));

        if (target != lastChannelMode)
        {
            autopilot->setMode(target);
            lastChannelMode = target;
        }
    }


private:

    Autopilot* autopilot;
    AutopilotMode lastChannelMode = MODE_MANUAL;

    static AutopilotMode modeFor(uint16_t input)
    {
        if (input >= 1750) return MODE_AUTO_TAKEOFF;
        if (input >= 1250) return MODE_STABILIZE;
        return MODE_MANUAL;
    }
};
