#pragma once

#include "autopilot/AutopilotTypes.h"
#include "autopilot/ControlBinding.h"
#include "config/Channels.h"

// ============================================================
// 🎛️ ЧТО ДЕЛАЕТ КАЖДЫЙ ТУМБЛЕР И КРУТИЛКА ПУЛЬТА
//
// Одна строка — один канал. Хотите RTH на SwB вместо закрылок —
// поменяйте одну строку и перепрошейте. Что делает каждый режим,
// функция и крутилка — docs/AUTOPILOT_GUIDE.md.
//
//   Bind::modes  (канал, вверх, середина, вниз)  — тумблер выбирает режим
//   Bind::mode   (канал, режим)                  — режим поверх, пока тумблер включён
//   Bind::feature(канал, функция)                — функция, пока тумблер включён
//   Bind::knob   (канал, крутилка)               — плавная величина
//
// Каналы FS-i6: SWB, SWC (3 положения), SWD, VRA, VRB (Channels.h).
// SWA — ARM, стики — свои, их занять нельзя (проверка ниже).
// ============================================================

namespace Controls
{
    constexpr Binding BINDINGS[] = {
        Bind::modes  (Channels::SWC, MODE_MANUAL, MODE_STABILIZE, MODE_AUTO_TAKEOFF),
        Bind::feature(Channels::SWB, Feature::FLAPS),
        Bind::mode   (Channels::SWD, MODE_RTH),
        Bind::knob   (Channels::VRA, Knob::STAB_GAIN),
        Bind::knob   (Channels::VRB, Knob::CRUISE_SPEED),

        // Идеи — раскомментируйте, закомментировав строку с тем же каналом:
        // Bind::modes  (Channels::SWC, MODE_STABILIZE, MODE_CRUISE, MODE_LOITER),
        // Bind::mode   (Channels::SWD, MODE_RESCUE),          // "спасите": ровно и вверх
        // Bind::mode   (Channels::SWD, MODE_LAUNCH),          // запуск с руки
        // Bind::mode   (Channels::SWB, MODE_SOARING),         // парение в термиках
        // Bind::feature(Channels::SWB, Feature::PAYLOAD_DROP),// сброс груза
        // Bind::feature(Channels::SWB, Feature::AUTO_TRIM),   // автотриммер
        // Bind::feature(Channels::SWD, Feature::BEEPER),      // найти самолёт в траве
        // Bind::knob   (Channels::VRB, Knob::CAMERA_TILT),    // наклон камеры
        // Bind::knob   (Channels::VRB, Knob::FLAPS),          // закрылки плавно
    };

    static_assert(BindingCheck::channelsFree(BINDINGS),
                  "Controls.h: стики и SWA (ARM) привязывать нельзя, канал < Channels::COUNT");
    static_assert(BindingCheck::channelsUnique(BINDINGS),
                  "Controls.h: у канала может быть только одна привязка");
    static_assert(BindingCheck::atMostOneModeSwitch(BINDINGS),
                  "Controls.h: тумблер выбора режимов (Bind::modes) — не больше одного");
}
