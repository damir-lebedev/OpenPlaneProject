// Реализация autopilot/AutopilotTypes.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/AutopilotTypes.h"

namespace AutopilotNames
{

const char* mode(AutopilotMode m)
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

const char* modeShort(AutopilotMode m)
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

const char* feature(Feature f)
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

const char* knob(Knob k)
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

}  // namespace AutopilotNames


auto PilotInputs::knobValue(Knob k, float minValue, float defaultValue, float maxValue) const -> float
{
    const float x = knob(k);
    return x >= 0 ? defaultValue + x * (maxValue - defaultValue)
                  : defaultValue + x * (defaultValue - minValue);
}
