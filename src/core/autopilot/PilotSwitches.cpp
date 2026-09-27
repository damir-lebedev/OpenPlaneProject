// Реализация autopilot/PilotSwitches.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/PilotSwitches.h"


PilotSwitches::PilotSwitches(Autopilot* ap)
: PilotSwitches(ap, Controls::BINDINGS)
{
}

auto PilotSwitches::update(const RcChannelState& rc) -> void
{
    PilotInputs next;
    bool haveModeSwitch = false;
    AutopilotMode switchMode = MODE_MANUAL;
    bool haveOverride = false;
    AutopilotMode overrideMode = MODE_MANUAL;

    for (size_t i = 0; i < count; ++i)
    {
        const Binding& b = bindings[i];
        const uint16_t value = rc.get(b.channel);

        switch (b.kind)
        {
            case Binding::Kind::FEATURE:
                next.features[static_cast<uint8_t>(b.feature)] = value >= SWITCH_ON_US;
                break;

            case Binding::Kind::KNOB:
            {
                const float x = (static_cast<float>(value) - 1500.0f) / 500.0f;
                next.knobs[static_cast<uint8_t>(b.knob)] = constrain(x, -1.0f, 1.0f);
                next.knobBound[static_cast<uint8_t>(b.knob)] = true;
                break;
            }

            case Binding::Kind::MODES:
                haveModeSwitch = true;
                switchMode = b.modes[zoneOf(value, b.modeCount)];
                break;

            case Binding::Kind::MODE:
                if (!haveOverride && value >= SWITCH_ON_US)
                {
                    haveOverride = true;
                    overrideMode = b.modes[0];
                }
                break;
        }
    }

    inputs = next;

    if (!autopilot) return;
    autopilot->setInputs(inputs);

    const AutopilotMode target = haveOverride ? overrideMode : switchMode;
    if ((haveOverride || haveModeSwitch) && target != lastTarget)
    {
        autopilot->setMode(target);
        lastTarget = target;
    }
}

auto PilotSwitches::printBindings() const -> void
{
    for (size_t i = 0; i < count; ++i)
    {
        const Binding& b = bindings[i];
        Serial.print(channelName(b.channel));
        Serial.print(": ");
        switch (b.kind)
        {
            case Binding::Kind::MODES:
                for (uint8_t m = 0; m < b.modeCount; ++m)
                {
                    if (m) Serial.print(" / ");
                    Serial.print(AutopilotNames::mode(b.modes[m]));
                }
                Serial.println(b.modeCount == 3 ? " (вверх / середина / вниз)" : " (вверх / вниз)");
                break;
            case Binding::Kind::MODE:
                Serial.print(AutopilotNames::mode(b.modes[0]));
                Serial.println(", пока включён");
                break;
            case Binding::Kind::FEATURE:
                Serial.print(AutopilotNames::feature(b.feature));
                Serial.println(", пока включён");
                break;
            case Binding::Kind::KNOB:
                Serial.print("крутилка ");
                Serial.println(AutopilotNames::knob(b.knob));
                break;
        }
    }
}

auto PilotSwitches::channelName(uint8_t channel) -> const char*
{
    switch (channel)
    {
        case Channels::SWB: return "SwB (CH6)";
        case Channels::SWC: return "SwC (CH7)";
        case Channels::SWD: return "SwD (CH8)";
        case Channels::VRA: return "VrA (CH9)";
        case Channels::VRB: return "VrB (CH10)";
        default:            return "CH?";
    }
}

auto PilotSwitches::zoneOf(uint16_t value, uint8_t positions) -> uint8_t
{
    if (positions <= 2) return value >= TWO_POS_US ? 1 : 0;
    if (value >= SWITCH_ON_US) return 2;
    if (value >= THREE_POS_LOW_US) return 1;
    return 0;
}
