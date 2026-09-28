#pragma once
#include <stdint.h>

// ============================================================
// РЕЖИМЫ, ФУНКЦИИ И КРУТИЛКИ АВТОПИЛОТА
//
// Три вида того, что можно повесить на тумблер или крутилку пульта
// (привязки — одна строка на канал в config/Controls.h):
//
//   • AutopilotMode — режим полёта. Включён всегда ровно один.
//     Выбирается многопозиционным тумблером (Bind::modes) или
//     включается "поверх" отдельным тумблером (Bind::mode, например
//     RTH на SwD — пока тумблер включён, режим принудительный).
//   • Feature — функция, которая включена, пока включён тумблер
//     (Bind::feature): закрылки, автотриммер, сброс груза...
//     Функции не исключают друг друга и работают в любом режиме.
//   • Knob — плавная величина с крутилки (Bind::knob): сила
//     стабилизации, скорость круиза, угол камеры... Центр крутилки
//     (1500 мкс) — значение по умолчанию из Config, края — пределы.
//
// Числовые значения AutopilotMode — часть API дашборда
// (POST /api/setmode), существующие номера не меняются.
// ============================================================

enum AutopilotMode : uint8_t
{
    MODE_MANUAL = 0,        // рули = стики
    MODE_STABILIZE = 1,     // стик задаёт крен/тангаж, отпустил — горизонт
    MODE_AUTO_TAKEOFF = 2,  // автовзлёт с полосы/с руки по газу пилота
    MODE_ALT_HOLD = 3,      // как STABILIZE, высоту держит руль высоты
    MODE_ACRO = 4,          // стик задаёт скорость вращения, отпустил — держит угол
    MODE_CRUISE = 5,        // держит курс и высоту, газ — сам; стики меняют курс/высоту
    MODE_LOITER = 6,        // круги над точкой включения (GPS)
    MODE_RTH = 7,           // домой на высоте RTH, затем круги над домом (GPS)
    MODE_LAUNCH = 8,        // запуск с руки: мотор после броска, набор высоты
    MODE_AUTO_LAND = 9,     // посадка: планирование, выравнивание у земли
    MODE_SOARING = 10,      // парение: мотор выключен, круги в термиках
    MODE_RESCUE = 11,       // "спасите": крылья ровно, набор высоты с мотором
    MODE_COUNT
};

enum class Feature : uint8_t
{
    FLAPS,              // закрылки (флапероны) выпущены полностью
    AIRBRAKE,           // воздушный тормоз: оба элерона вверх
    AUTO_TRIM,          // автотриммер: учится держать прямо без стиков
    TURN_COORDINATION,  // координация разворота: руль направления и тангаж в крене
    MOTOR_KILL,         // мотор выключен в любом режиме
    BEEPER,             // пищалка "где самолёт"
    PAYLOAD_DROP,       // сброс груза (серво AUX1)
    GEOFENCE,           // геозабор: вылет за радиус/высоту -> домой
    HOME_RESET,         // дом = текущая точка (по включению тумблера)
    CAMERA_STAB,        // камера (серво AUX2) держит угол к горизонту
    COUNT
};

enum class Knob : uint8_t
{
    STAB_GAIN,       // сила стабилизации: ×0.25 ... ×1 (центр) ... ×2
    MAX_BANK,        // предельный крен в режимах со стабилизацией, °
    CRUISE_SPEED,    // газ круиза, % (или воздушная скорость, если есть трубка Пито)
    FLAPS,           // закрылки плавно, 0 ... 100%
    CAMERA_TILT,     // угол камеры, °
    RATES,           // чувствительность стиков, %
    LOITER_RADIUS,   // радиус кругов, м
    COUNT
};

namespace AutopilotNames
{
    inline const char* mode(AutopilotMode m)
    {
        switch (m)
        {
            case MODE_MANUAL:       return "MANUAL";
            case MODE_STABILIZE:    return "STABILIZE";
            case MODE_AUTO_TAKEOFF: return "AUTO_TAKEOFF";
            case MODE_ALT_HOLD:     return "ALT_HOLD";
            case MODE_ACRO:         return "ACRO";
            case MODE_CRUISE:       return "CRUISE";
            case MODE_LOITER:       return "LOITER";
            case MODE_RTH:          return "RTH";
            case MODE_LAUNCH:       return "LAUNCH";
            case MODE_AUTO_LAND:    return "AUTO_LAND";
            case MODE_SOARING:      return "SOARING";
            case MODE_RESCUE:       return "RESCUE";
            default:                return "UNKNOWN";
        }
    }

    // Короткое имя (до 5 символов) — для OLED.
    inline const char* modeShort(AutopilotMode m)
    {
        switch (m)
        {
            case MODE_MANUAL:       return "MAN";
            case MODE_STABILIZE:    return "STAB";
            case MODE_AUTO_TAKEOFF: return "TKOFF";
            case MODE_ALT_HOLD:     return "ALT";
            case MODE_ACRO:         return "ACRO";
            case MODE_CRUISE:       return "CRZ";
            case MODE_LOITER:       return "LOIT";
            case MODE_RTH:          return "RTH";
            case MODE_LAUNCH:       return "LNCH";
            case MODE_AUTO_LAND:    return "LAND";
            case MODE_SOARING:      return "SOAR";
            case MODE_RESCUE:       return "RESQ";
            default:                return "?";
        }
    }

    inline const char* feature(Feature f)
    {
        switch (f)
        {
            case Feature::FLAPS:             return "FLAPS";
            case Feature::AIRBRAKE:          return "AIRBRAKE";
            case Feature::AUTO_TRIM:         return "AUTO_TRIM";
            case Feature::TURN_COORDINATION: return "TURN_COORD";
            case Feature::MOTOR_KILL:        return "MOTOR_KILL";
            case Feature::BEEPER:            return "BEEPER";
            case Feature::PAYLOAD_DROP:      return "PAYLOAD_DROP";
            case Feature::GEOFENCE:          return "GEOFENCE";
            case Feature::HOME_RESET:        return "HOME_RESET";
            case Feature::CAMERA_STAB:       return "CAMERA_STAB";
            default:                         return "?";
        }
    }

    inline const char* knob(Knob k)
    {
        switch (k)
        {
            case Knob::STAB_GAIN:     return "STAB_GAIN";
            case Knob::MAX_BANK:      return "MAX_BANK";
            case Knob::CRUISE_SPEED:  return "CRUISE_SPEED";
            case Knob::FLAPS:         return "FLAPS";
            case Knob::CAMERA_TILT:   return "CAMERA_TILT";
            case Knob::RATES:         return "RATES";
            case Knob::LOITER_RADIUS: return "LOITER_RADIUS";
            default:                  return "?";
        }
    }
}

// Что включено тумблерами и где стоят крутилки — снимок за такт.
// Крутилка: -1 (1000 мкс) ... 0 (центр) ... +1 (2000 мкс); без
// привязки — 0, то есть значение по умолчанию.
struct PilotInputs
{
    bool features[static_cast<uint8_t>(Feature::COUNT)] = {};
    float knobs[static_cast<uint8_t>(Knob::COUNT)] = {};
    bool knobBound[static_cast<uint8_t>(Knob::COUNT)] = {};

    bool has(Feature f) const { return features[static_cast<uint8_t>(f)]; }
    float knob(Knob k) const { return knobs[static_cast<uint8_t>(k)]; }
    bool isBound(Knob k) const { return knobBound[static_cast<uint8_t>(k)]; }

    // Значение крутилки в единицах: центр — defaultValue, края — minValue/maxValue.
    float knobValue(Knob k, float minValue, float defaultValue, float maxValue) const
    {
        const float x = knob(k);
        return x >= 0 ? defaultValue + x * (maxValue - defaultValue)
                      : defaultValue + x * (defaultValue - minValue);
    }
};
