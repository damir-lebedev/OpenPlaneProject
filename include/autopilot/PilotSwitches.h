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

    explicit PilotSwitches(Autopilot* ap = nullptr);

    void update(const RcChannelState& rc);

    // Подсказка при включении: что на каком тумблере (из таблицы привязок).
    void printBindings() const;

    static const char* channelName(uint8_t channel);

    const PilotInputs& getInputs() const { return inputs; }
    size_t size() const { return count; }
    const Binding& binding(size_t i) const { return bindings[i]; }

    // Положение тумблера на 2/3 позиции -> индекс режима.
    static uint8_t zoneOf(uint16_t value, uint8_t positions);


private:

    Autopilot* autopilot;
    const Binding* bindings;
    size_t count;

    PilotInputs inputs;
    AutopilotMode lastTarget = MODE_MANUAL;
};
