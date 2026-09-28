#pragma once

// ============================================================
// Нативная замена ядра STM32duino 3.x — поверх фейков ESP32-ядра
// (test/native/support): время, GPIO, Serial, Wire, SPI те же, здесь —
// только то, что есть лишь у STM32duino:
//   - имена пинов PA0..PE15 (порт·16 + номер, 0x00..0x4F);
//   - pin_size_t, PinName, таблица таймеров PinMap_TIM для пинов
//     сервовыходов из Config.h (BOARD_STM32H743);
//   - HardwareTimer: ширина импульса видна тесту (fake::timerPulseUs)
//     и pulseIn() — как у настоящего таймера;
//   - Uart (= HardwareSerial с пинами), noInterrupts()/interrupts().
// ============================================================

#include "../support/Arduino.h"

#include <map>

typedef uint32_t pin_size_t;
typedef uint32_t PinName;

#define STM32_FAKE_PIN(port, n) (((port) << 4) | (n))
#define PA0  STM32_FAKE_PIN(0, 0)
#define PA1  STM32_FAKE_PIN(0, 1)
#define PA2  STM32_FAKE_PIN(0, 2)
#define PA3  STM32_FAKE_PIN(0, 3)
#define PA9  STM32_FAKE_PIN(0, 9)
#define PA10 STM32_FAKE_PIN(0, 10)
#define PB8  STM32_FAKE_PIN(1, 8)
#define PB9  STM32_FAKE_PIN(1, 9)
#define PB10 STM32_FAKE_PIN(1, 10)
#define PB11 STM32_FAKE_PIN(1, 11)
#define PB12 STM32_FAKE_PIN(1, 12)
#define PB13 STM32_FAKE_PIN(1, 13)
#define PB14 STM32_FAKE_PIN(1, 14)
#define PB15 STM32_FAKE_PIN(1, 15)
#define PC0  STM32_FAKE_PIN(2, 0)
#define PC1  STM32_FAKE_PIN(2, 1)
#define PD0  STM32_FAKE_PIN(3, 0)
#define PD1  STM32_FAKE_PIN(3, 1)
#define PD8  STM32_FAKE_PIN(3, 8)
#define PD9  STM32_FAKE_PIN(3, 9)
#define PD10 STM32_FAKE_PIN(3, 10)
#define PD14 STM32_FAKE_PIN(3, 14)
#define PD15 STM32_FAKE_PIN(3, 15)
#define PE7  STM32_FAKE_PIN(4, 7)
#define PE8  STM32_FAKE_PIN(4, 8)
#define PE9  STM32_FAKE_PIN(4, 9)
#define PE15 STM32_FAKE_PIN(4, 15)

using Uart = HardwareSerial;

inline void noInterrupts() { ++fake::tasks().criticalEntries; }
inline void interrupts() {}

// ------------------------------------------------------------
// Таймеры и таблица PinMap_TIM
// ------------------------------------------------------------

struct TIM_TypeDef
{
    int number;
};

namespace fake
{
    inline TIM_TypeDef* timerInstance(int number)
    {
        static TIM_TypeDef timers[8] = { { 0 }, { 1 }, { 2 }, { 3 }, { 4 }, { 5 }, { 6 }, { 7 } };
        return &timers[number];
    }

    // Ширина импульса на пине, мкс (−1 — таймер не настроен).
    inline std::map<uint32_t, long>& timerPulses()
    {
        static std::map<uint32_t, long> pulses;
        return pulses;
    }

    inline long timerPulseUs(uint32_t pin)
    {
        auto it = timerPulses().find(pin);
        return it == timerPulses().end() ? -1 : it->second;
    }
}

struct PinMap
{
    PinName pin;
    int timer;
    int channel;
};

// Пины сервовыходов Config.h (BOARD_STM32H743) — как в PeripheralPins
// варианта WeAct H743: TIM2 CH1..4, TIM4 CH3/CH4, TIM1 CH1.
inline const PinMap PinMap_TIM[] = {
    { PA0, 2, 1 }, { PA1, 2, 2 }, { PA2, 2, 3 }, { PA3, 2, 4 },
    { PD14, 4, 3 }, { PD15, 4, 4 }, { PE9, 1, 1 },
    { 0xFFFFFFFFu, 0, 0 },
};

inline PinName digitalPinToPinName(pin_size_t pin) { return pin; }

inline void* pinmap_peripheral(PinName pin, const PinMap* map)
{
    for (; map->pin != 0xFFFFFFFFu; ++map)
    {
        if (map->pin == pin) return fake::timerInstance(map->timer);
    }
    return nullptr;
}

inline int pinmap_function(PinName pin, const PinMap* map)
{
    for (; map->pin != 0xFFFFFFFFu; ++map)
    {
        if (map->pin == pin) return map->channel;
    }
    return 0;
}

#define STM_PIN_CHANNEL(function) (static_cast<uint32_t>(function))

enum TimerModes_t { TIMER_OUTPUT_COMPARE_PWM1 = 6 };
enum TimerFormat_t { MICROSEC_FORMAT, MICROSEC_COMPARE_FORMAT };

class HardwareTimer
{
public:
    void setup(TIM_TypeDef* timerInstance) { instance = timerInstance; }
    void setOverflow(uint32_t value, TimerFormat_t) { periodUs = value; }

    void setMode(uint32_t channel, TimerModes_t, PinName pin)
    {
        if (channel < 5) channelPin[channel] = pin;
        fake::timerPulses()[pin] = 0;
    }

    void setCaptureCompare(uint32_t channel, uint32_t value, TimerFormat_t)
    {
        if (channel >= 5) return;
        const uint32_t pin = channelPin[channel];
        fake::timerPulses()[pin] = static_cast<long>(value);
        // pulseIn() видит импульс таймера (вход GPIO STM32 читает пин в AF-режиме).
        if (pin < fake::GPIO_COUNT)
        {
            fake::gpio().pulseOverride[pin] = static_cast<long>(value);
            fake::gpio().inputEnabled[pin] = true;
        }
    }

    void resume() { running = true; }

    TIM_TypeDef* instance = nullptr;
    uint32_t periodUs = 0;
    bool running = false;
    uint32_t channelPin[5] = {};
};
