#pragma once
// ============================================================
// CHANNEL MAP
//
// Единственное место, где физический номер канала (CH1-CH10)
// связывается с его назначением. Если передатчик перенастроят
// по-другому, менять нужно только этот файл.
//
// Раскладка проверена на стенде (FS-i6, 10 каналов, режим 2):
// правый стик вправо -> CH1=2000, правый стик от себя -> CH2=2000,
// газ вверх -> CH3=2000; SwA=CH5, SwB=CH6, SwC(3 поз.)=CH7,
// SwD=CH8, VrA=CH9, VrB=CH10.
// ============================================================

namespace Channels
{
    constexpr uint8_t AILERON  = 0;  // CH1
    constexpr uint8_t ELEVATOR = 1;  // CH2
    constexpr uint8_t THROTTLE = 2;  // CH3
    constexpr uint8_t RUDDER   = 3;  // CH4 (пока не используется)
    constexpr uint8_t ARM      = 4;  // CH5  - SwA, тумблер ARM, см. ArmingManager.h

    constexpr uint8_t AUX_1    = 5;  // CH6  - SwB, свободен
    constexpr uint8_t AUX_2    = 6;  // CH7  - SwC, режим автопилота, см. AutopilotModeSelector.h
    constexpr uint8_t AUX_3    = 7;  // CH8  - SwD, свободен (раньше был BOOST)
    constexpr uint8_t FLAPS    = 8;  // CH9  - VrA, закрылки, линейно (крутилка на пульте)
    constexpr uint8_t AUX_5    = 9;  // CH10 - VrB, свободен
}
