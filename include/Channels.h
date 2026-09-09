#pragma once
// ============================================================
// 2. CHANNEL MAP
//
// Важно: здесь находится только логическое описание каналов.
// Если позже передатчик будет перенастроен, менять нужно будет
// только этот блок.
// ============================================================

namespace Channels
{
    constexpr uint8_t AILERON  = 0;  // CH1
    constexpr uint8_t ELEVATOR = 1;  // CH2
    constexpr uint8_t THROTTLE = 2;  // CH3
    constexpr uint8_t RUDDER   = 3;  // CH4

    constexpr uint8_t FLAPS    = 4;  // CH5

    constexpr uint8_t AUX_1    = 5;  // CH6
    constexpr uint8_t AUX_2    = 6;  // CH7

    // Текущая логика boost использует CH8.
    constexpr uint8_t BOOST    = 7;  // CH8

    constexpr uint8_t AUX_4    = 8;  // CH9
    constexpr uint8_t AUX_5    = 9;  // CH10
}