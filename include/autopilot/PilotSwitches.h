#pragma once
#include <Arduino.h>
#include <stddef.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "autopilot/ControlBinding.h"
#include "config/Channels.h"
#include "config/Config.h"
#include "config/Controls.h"
#include "rc/RcChannelState.h"

// ============================================================
// ТУМБЛЕРЫ И КРУТИЛКИ ПУЛЬТА -> РЕЖИМ, ФУНКЦИИ, КРУТИЛКИ
//
// Движок таблицы привязок (config/Controls.h). За такт:
//   • функции (Bind::feature) — включена, пока канал >= SWITCH_ON_US;
//   • крутилки (Bind::knob) — (канал − 1500) / 500, от −1 до +1;
//   • режим: тумблер режимов (Bind::modes) даёт режим по зоне,
//     включённый тумблер "поверх" (Bind::mode) — перекрывает его
//     (приоритет — у того, что выше в таблице).
//
// Режим меняется в Autopilot только когда тумблеры реально дают
// ДРУГОЙ режим, а не каждый такт: иначе режим, выбранный с дашборда
// (POST /api/setmode) или геозабором, тумблеры переписывали бы
// обратно каждые 2 мс.
//
// При потере связи не вызывается (FlightController): в failsafe-кадре
// каналы содержат значения failsafe, а не положения тумблеров.
// ============================================================

class PilotSwitches
{
public:

    static constexpr uint16_t SWITCH_ON_US = Config::SWITCH_ON_US;
    static constexpr uint16_t THREE_POS_LOW_US = 1250;
    static constexpr uint16_t TWO_POS_US = 1500;

    template <size_t N>
    PilotSwitches(Autopilot* ap, const Binding (&table)[N])
        : autopilot(ap),
          bindings(table),
          count(N)
    {
    }

    explicit PilotSwitches(Autopilot* ap = nullptr)
        : PilotSwitches(ap, Controls::BINDINGS)
    {
    }

    void update(const RcChannelState& rc)
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

    // Подсказка при включении: что на каком тумблере (из таблицы привязок).
    void printBindings() const
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

    static const char* channelName(uint8_t channel)
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

    const PilotInputs& getInputs() const { return inputs; }
    size_t size() const { return count; }
    const Binding& binding(size_t i) const { return bindings[i]; }

    // Положение тумблера на 2/3 позиции -> индекс режима.
    static uint8_t zoneOf(uint16_t value, uint8_t positions)
    {
        if (positions <= 2) return value >= TWO_POS_US ? 1 : 0;
        if (value >= SWITCH_ON_US) return 2;
        if (value >= THREE_POS_LOW_US) return 1;
        return 0;
    }


private:

    Autopilot* autopilot;
    const Binding* bindings;
    size_t count;

    PilotInputs inputs;
    AutopilotMode lastTarget = MODE_MANUAL;
};
