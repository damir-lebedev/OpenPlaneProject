#pragma once
// ============================================================
// CHANNEL MAP
//
// Единственное место, где физический номер канала (CH1-CH10)
// связывается с его назначением. Если передатчик перенастроят
// по-другому, менять нужно только этот файл.
// ============================================================

namespace Channels
{
    constexpr uint8_t AILERON  = 0;  // CH1
    constexpr uint8_t ELEVATOR = 1;  // CH2
    constexpr uint8_t THROTTLE = 2;  // CH3
    constexpr uint8_t RUDDER   = 3;  // CH4 (пока не используется)
    constexpr uint8_t FLAPS    = 4;  // CH5

    constexpr uint8_t AUX_1    = 5;  // CH6  - свободен
    constexpr uint8_t AUX_2    = 6;  // CH7  - свободен
    constexpr uint8_t BOOST    = 7;  // CH8  - используется ThrottleManager
    constexpr uint8_t AUX_4    = 8;  // CH9  - свободен
    constexpr uint8_t AUX_5    = 9;  // CH10 - свободен

    // --------------------------------------------------------
    // Каналы автопилота (FeatureManager).
    //
    // Специально обходят BOOST (CH8), чтобы не конфликтовать
    // с ThrottleManager. Порядок в массиве = порядок слотов
    // FeatureManager (slot 0..3).
    // --------------------------------------------------------

    constexpr uint8_t FEATURE_SLOTS[4] = { AUX_1, AUX_2, AUX_4, AUX_5 };
}