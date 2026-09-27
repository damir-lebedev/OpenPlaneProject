#pragma once
#include <Arduino.h>

#include "config/Config.h"

// ============================================================
// ПИЩАЛКА "ГДЕ САМОЛЁТ"
//
// Пищит прерывисто (2 Гц), когда:
//   • включён Feature::BEEPER (тумблер) — найти самолёт в траве;
//   • связь потеряна на земле (DISARM) дольше
//     LOST_MODEL_BEEP_DELAY_MS — самолёт упал, пульт далеко или
//     выключен.
// В воздухе (ARM) по потере связи не пищит: там работает failsafe,
// а пищалка только разряжала бы батарею.
//
// Пин — Config::PIN_BUZZER (через транзистор); на платах без него
// IBoard::setBuzzer() ничего не делает.
// ============================================================

class Beeper
{
public:

    bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs);

    bool isLostModel() const { return lostModel; }


private:

    static constexpr uint32_t HALF_PERIOD_MS = 250;

    uint32_t lostSinceMs = 0;
    bool lostModel = false;
};
