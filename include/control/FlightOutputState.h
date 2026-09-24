#pragma once
#include <stdint.h>

#include "config/Config.h"

// ============================================================
// FLIGHT OUTPUT STATE
//
// Желаемые положения поверхностей и газ (мкс PWM), без привязки
// к конкретному железу. ControlMixer и FlightController считают
// это, FlightOutputs превращает в реальный PWM.
// ============================================================

struct FlightOutputState
{
    uint16_t aileronLeft  = Config::PWM_CENTER;
    uint16_t aileronRight = Config::PWM_CENTER;
    uint16_t elevator     = Config::PWM_CENTER;
    uint16_t rudder       = Config::PWM_CENTER;
    uint16_t throttle     = Config::PWM_MIN;
};
