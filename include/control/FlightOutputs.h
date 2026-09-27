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
    static const OutputInfo& outputInfo(uint8_t channel);

    explicit FlightOutputs(IBoard& hardware);

    // true, если все обязательные выходы получили канал PWM.
    bool begin();

    bool isAttached(uint8_t channel) const;

    // Значение выхода channel в состоянии state, мкс.
    static uint16_t valueOf(const FlightOutputState& state, uint8_t channel);

    void printStatus() const;

    // Самопроверка выходов: на каждом GPIO измеряется реальный
    // импульс и сравнивается с тем, что туда пишет прошивка. Если
    // совпадает, а серво/ESC реагирует "не на тот" стик — значит,
    // провод воткнут не в тот пин.
    void printPulseSelfTest();

    // Применить рассчитанное состояние к физическим выходам.
    void write(const FlightOutputState& state);

    // Немедленно выставить безопасные значения (нейтраль, газ выключен).
    // Груз и камера остаются как были: потеря связи не должна
    // сбрасывать груз.
    void setFailsafe();

    const FlightOutputState& getLastState() const;

    void setBuzzer(bool on);

    bool isBuzzerOn() const { return buzzer; }


private:

    IBoard& board;
    bool attached[ServoChannel::COUNT] = {};
    FlightOutputState lastState;
    bool buzzer = false;
};
