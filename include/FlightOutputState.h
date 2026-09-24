#pragma once

#include "Config.h"

// ============================================================
// FLIGHT OUTPUT STATE
//
// Логическое представление желаемых положений поверхностей и
// газа (в µs), без привязки к Servo/PWM. ControlMixer считает
// это; FlightOutputs превращает в реальный PWM.
// ============================================================

struct FlightOutputState
{
    uint16_t aileronLeft  = Config::PWM_CENTER;
    uint16_t aileronRight = Config::PWM_CENTER;
    uint16_t elevator     = Config::PWM_CENTER;
    uint16_t rudder       = Config::PWM_CENTER;
    uint16_t throttle     = Config::PWM_MIN;
};