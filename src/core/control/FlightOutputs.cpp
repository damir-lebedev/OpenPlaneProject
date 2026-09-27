// Реализация control/FlightOutputs.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "control/FlightOutputs.h"


auto FlightOutputs::outputInfo(uint8_t channel) -> const OutputInfo&
{
    static const OutputInfo table[ServoChannel::COUNT] = {
        { "aileronLeft",  "элерон L", static_cast<int16_t>(Config::PIN_AILERON_LEFT),  true,  &FlightOutputState::aileronLeft },
        { "aileronRight", "элерон R", static_cast<int16_t>(Config::PIN_AILERON_RIGHT), true,  &FlightOutputState::aileronRight },
        { "elevator",     "руль выс", static_cast<int16_t>(Config::PIN_ELEVATOR),      true,  &FlightOutputState::elevator },
        { "esc",          "ESC     ", static_cast<int16_t>(Config::PIN_ESC),           true,  &FlightOutputState::throttle },
        { "rudder",       "руль нап", static_cast<int16_t>(Config::PIN_RUDDER),        false, &FlightOutputState::rudder },
        { "aux1",         "AUX1 груз", static_cast<int16_t>(Config::PIN_AUX1),         false, &FlightOutputState::aux1 },
        { "aux2",         "AUX2 кам ", static_cast<int16_t>(Config::PIN_AUX2),         false, &FlightOutputState::aux2 },
    };
    return table[channel];
}

FlightOutputs::FlightOutputs(IBoard& hardware)
: board(hardware)
{
}

auto FlightOutputs::begin() -> bool
{
    bool allRequired = true;

    for (uint8_t ch = 0; ch < ServoChannel::COUNT; ++ch)
    {
        attached[ch] = board.servo(ch).attach(Config::PWM_MIN, Config::PWM_MAX);

        if (outputInfo(ch).required && !attached[ch])
        {
            allRequired = false;
        }
    }

    printStatus();
    return allRequired;
}

auto FlightOutputs::isAttached(uint8_t channel) const -> bool
{
    return channel < ServoChannel::COUNT && attached[channel];
}

auto FlightOutputs::valueOf(const FlightOutputState& state, uint8_t channel) -> uint16_t
{
    return state.*(outputInfo(channel).field);
}

auto FlightOutputs::printStatus() const -> void
{
    Serial.print("Outputs:");

    for (uint8_t ch = 0; ch < ServoChannel::COUNT; ++ch)
    {
        const OutputInfo& info = outputInfo(ch);

        Serial.print(' ');
        Serial.print(info.key);

        if (info.pin < 0)
        {
            Serial.print("(нет пина)");
            continue;
        }

        Serial.print("(GPIO");
        Serial.print(info.pin);
        Serial.print(")=");
        Serial.print(attached[ch] ? "OK" : "FAIL");
    }

    Serial.println();
}

auto FlightOutputs::printPulseSelfTest() -> void
{
    Serial.println("Выходы: GPIO -> измерено / ожидается (мкс)");

    for (uint8_t ch = 0; ch < ServoChannel::COUNT; ++ch)
    {
        const OutputInfo& info = outputInfo(ch);
        if (info.pin < 0) continue;

        const int32_t measured = board.servo(ch).measurePulseUs();
        const uint16_t expected = valueOf(lastState, ch);

        Serial.print("  "); Serial.print(info.label);
        Serial.print(" GPIO"); Serial.print(info.pin);
        Serial.print(": ");
        if (measured < 0) Serial.print("нет импульса"); else Serial.print(measured);
        Serial.print(" / "); Serial.print(expected);

        const bool ok = measured >= 0 && abs(measured - static_cast<int32_t>(expected)) <= 15;
        Serial.println(ok ? "  OK" : "  НЕ СОВПАДАЕТ");
    }
}

auto FlightOutputs::write(const FlightOutputState& state) -> void
{
    for (uint8_t ch = 0; ch < ServoChannel::COUNT; ++ch)
    {
        board.servo(ch).writeMicroseconds(valueOf(state, ch));
    }

    lastState = state;
}

auto FlightOutputs::setFailsafe() -> void
{
    FlightOutputState safe;

    safe.aux1 = lastState.aux1;
    safe.aux2 = lastState.aux2;
    safe.aileronLeft = Config::FAILSAFE_AILERON;
    safe.aileronRight = Config::FAILSAFE_AILERON;
    safe.elevator = Config::FAILSAFE_ELEVATOR;
    safe.rudder = Config::FAILSAFE_RUDDER;
    safe.throttle = Config::FAILSAFE_THROTTLE;

    write(safe);
}

auto FlightOutputs::getLastState() const -> const FlightOutputState&
{
    return lastState;
}

auto FlightOutputs::setBuzzer(bool on) -> void
{
    buzzer = on;
    board.setBuzzer(on);
}
