#pragma once
#include <stdint.h>

#include "config/Config.h"

// ============================================================
// FLIGHT OUTPUT STATE
//
// Желаемые положения поверхностей и газ (мкс PWM), без привязки
// к конкретному железу. ControlMixer и FlightController считают
// это, FlightOutputs превращает в реальный PWM.
//
// throttle — величина газа 1000..2000 = 0..100% в любом направлении;
// в импульс ESC (у двунаправленного 1500 = стоп) её вместе с
// motorReverse переводит FlightOutputs (control/EscSignal.h).
// ============================================================

struct FlightOutputState
{
    uint16_t aileronLeft  = Config::PWM_CENTER;
    uint16_t aileronRight = Config::PWM_CENTER;
    uint16_t elevator     = Config::PWM_CENTER;
    uint16_t rudder       = Config::PWM_CENTER;
    uint16_t throttle     = Config::PWM_MIN;
    bool motorReverse     = false;                      // мотор назад (MotorReverse)
    uint16_t aux1         = Config::directedUs(Config::PAYLOAD_CLOSED_US, Config::AUX1_REVERSED);  // сброс груза
    uint16_t aux2         = Config::PWM_CENTER;         // камера
};
