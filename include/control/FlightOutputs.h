#pragma once
#include <Arduino.h>

#include "config/Config.h"
#include "control/FlightOutputState.h"
#include "hal/IBoard.h"

// ============================================================
// FLIGHT OUTPUTS
//
// Единственный класс, который знает про набор и порядок
// PWM-выходов (элероны, руль высоты, ESC, руль направления) и их
// пины. Само железо спрятано за IBoard/IServoOutput — этот файл
// его не видит, поэтому смена MCU/драйвера сюда не проникает.
//
// Все выходы описаны одной таблицей (outputInfo()), и begin(),
// write(), вывод статуса и самопроверка проходят по ней циклом —
// новый выход добавляется одной строкой таблицы + полем в
// FlightOutputState + индексом в ServoChannel.
//
// attached означает только то, что плата смогла выделить канал и
// настроить пин. Подключён ли физический сервопривод — программно
// не видно; реальный импульс на пине проверяет printPulseSelfTest().
// ============================================================

class FlightOutputs
{
public:

    struct OutputInfo
    {
        const char* key;      // имя в JSON/логе
        const char* label;    // имя для человека
        int16_t pin;          // -1 — на этой плате не разведён (int16_t: у STM32 номера до 0xC0+N)
        bool required;        // без него борт не летит
        uint16_t FlightOutputState::* field;
    };

    // Порядок строк = индексы ServoChannel (hal/IBoard.h).
    static const OutputInfo& outputInfo(uint8_t channel)
    {
        static const OutputInfo table[ServoChannel::COUNT] = {
            { "aileronLeft",  "элерон L", static_cast<int16_t>(Config::PIN_AILERON_LEFT),  true,  &FlightOutputState::aileronLeft },
            { "aileronRight", "элерон R", static_cast<int16_t>(Config::PIN_AILERON_RIGHT), true,  &FlightOutputState::aileronRight },
            { "elevator",     "руль выс", static_cast<int16_t>(Config::PIN_ELEVATOR),      true,  &FlightOutputState::elevator },
            { "esc",          "ESC     ", static_cast<int16_t>(Config::PIN_ESC),           true,  &FlightOutputState::throttle },
            { "rudder",       "руль нап", static_cast<int16_t>(Config::PIN_RUDDER),        false, &FlightOutputState::rudder },
        };
        return table[channel];
    }

    explicit FlightOutputs(IBoard& hardware)
        : board(hardware)
    {
    }

    // true, если все обязательные выходы получили канал PWM.
    bool begin()
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

    bool isAttached(uint8_t channel) const
    {
        return channel < ServoChannel::COUNT && attached[channel];
    }

    // Значение выхода channel в состоянии state, мкс.
    static uint16_t valueOf(const FlightOutputState& state, uint8_t channel)
    {
        return state.*(outputInfo(channel).field);
    }

    void printStatus() const
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

    // Самопроверка выходов: на каждом GPIO измеряется реальный
    // импульс и сравнивается с тем, что туда пишет прошивка. Если
    // совпадает, а серво/ESC реагирует "не на тот" стик — значит,
    // провод воткнут не в тот пин.
    void printPulseSelfTest()
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

    // Применить рассчитанное состояние к физическим выходам.
    void write(const FlightOutputState& state)
    {
        for (uint8_t ch = 0; ch < ServoChannel::COUNT; ++ch)
        {
            board.servo(ch).writeMicroseconds(valueOf(state, ch));
        }

        lastState = state;
    }

    // Немедленно выставить безопасные значения (нейтраль, газ выключен).
    void setFailsafe()
    {
        FlightOutputState safe;

        safe.aileronLeft = Config::FAILSAFE_AILERON;
        safe.aileronRight = Config::FAILSAFE_AILERON;
        safe.elevator = Config::FAILSAFE_ELEVATOR;
        safe.rudder = Config::FAILSAFE_RUDDER;
        safe.throttle = Config::FAILSAFE_THROTTLE;

        write(safe);
    }

    const FlightOutputState& getLastState() const
    {
        return lastState;
    }


private:

    IBoard& board;
    bool attached[ServoChannel::COUNT] = {};
    FlightOutputState lastState;
};
