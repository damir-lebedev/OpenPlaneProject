#pragma once
#include <stdint.h>
// ============================================================
// CHANNEL MAP
//
// Единственное место, где физический номер канала (CH1-CH10)
// связывается с органом управления на пульте. Если передатчик
// перенастроят по-другому, менять нужно только этот файл.
//
// Раскладка проверена на стенде (FS-i6, 10 каналов, режим 2):
// правый стик вправо -> CH1=2000, правый стик от себя -> CH2=2000,
// газ вверх -> CH3=2000; SwA=CH5, SwB=CH6, SwC(3 поз.)=CH7,
// SwD=CH8, VrA=CH9, VrB=CH10.
//
// Стики и ARM закреплены за своими каналами. Что делают тумблеры
// SwB/SwC/SwD и крутилки VrA/VrB — решает таблица привязок
// config/Controls.h (одна строка на тумблер).
// ============================================================

namespace Channels
{
    constexpr uint8_t AILERON  = 0;  // CH1  - правый стик ←→
    constexpr uint8_t ELEVATOR = 1;  // CH2  - правый стик ↑↓
    constexpr uint8_t THROTTLE = 2;  // CH3  - левый стик ↑↓
    constexpr uint8_t RUDDER   = 3;  // CH4  - левый стик ←→, руль направления + колесо
    constexpr uint8_t ARM      = 4;  // CH5  - SwA, тумблер ARM, см. ArmingManager.h

    // Тумблеры и крутилки FS-i6 — для таблицы привязок (config/Controls.h).
    constexpr uint8_t SWA = 4;       // CH5  - 2 положения (ARM)
    constexpr uint8_t SWB = 5;       // CH6  - 2 положения
    constexpr uint8_t SWC = 6;       // CH7  - 3 положения
    constexpr uint8_t SWD = 7;       // CH8  - 2 положения
    constexpr uint8_t VRA = 8;       // CH9  - крутилка
    constexpr uint8_t VRB = 9;       // CH10 - крутилка

    constexpr uint8_t COUNT = 10;
}
