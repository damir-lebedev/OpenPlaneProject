#pragma once
#include <Arduino.h>

#include "config/Channels.h"
#include "config/Config.h"
#include "rc/RcChannelState.h"
#include "rc/RcInput.h"

// ============================================================
// THROTTLE MANAGER
//
// Читает газ пилота (CH3) и отдаёт его в мкс для ESC; при
// потере связи — FAILSAFE_THROTTLE. Не знает про Servo, ARM и
// автопилот — их поправки применяет FlightController поверх.
//
// Раньше здесь был лимит газа 40% + форсаж на 5 секунд (CH8):
// лимит берёг слабую сборку аккумуляторов 3S1P. С новыми
// аккумуляторами полный газ допустим всегда, так что лимит и
// форсаж убраны, CH8 (SwD) освободился. Форсаж к тому же
// срабатывал сам, если SwD был поднят при включении.
// ============================================================

class ThrottleManager
{
public:

    uint16_t update(
        const RcChannelState& rc,
        bool receiverFailsafe
    ) const
    {
        if (receiverFailsafe)
        {
            return Config::FAILSAFE_THROTTLE;
        }

        return RcInput::clamp(rc.get(Channels::THROTTLE));
    }
};
