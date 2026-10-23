#pragma once

// ============================================================
// Нативная замена ядра STM32duino 3.x — поверх фейков ESP32-ядра
// (test/native/support): время, GPIO, Serial, Wire, SPI те же, здесь —
// только то, что есть лишь у STM32duino:
//   - имена пинов PA0..PE15 (порт·16 + номер, 0x00..0x4F);
//   - pin_size_t, PinName, таблица таймеров PinMap_TIM для пинов
//     сервовыходов из Config.h (BOARD_STM32H743, BOARD_MATEK_H743);
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
#define PA5  STM32_FAKE_PIN(0, 5)
#define PA6  STM32_FAKE_PIN(0, 6)
#define PA9  STM32_FAKE_PIN(0, 9)
#define PA10 STM32_FAKE_PIN(0, 10)
#define PA15 STM32_FAKE_PIN(0, 15)
#define PB0  STM32_FAKE_PIN(1, 0)
#define PB1  STM32_FAKE_PIN(1, 1)
#define PB6  STM32_FAKE_PIN(1, 6)
#define PB7  STM32_FAKE_PIN(1, 7)
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
#define PC6  STM32_FAKE_PIN(2, 6)
#define PC7  STM32_FAKE_PIN(2, 7)
#define PC15 STM32_FAKE_PIN(2, 15)
#define PD0  STM32_FAKE_PIN(3, 0)
#define PD1  STM32_FAKE_PIN(3, 1)
#define PD5  STM32_FAKE_PIN(3, 5)
#define PD6  STM32_FAKE_PIN(3, 6)
#define PD7  STM32_FAKE_PIN(3, 7)
#define PD8  STM32_FAKE_PIN(3, 8)
#define PD9  STM32_FAKE_PIN(3, 9)
#define PD10 STM32_FAKE_PIN(3, 10)
#define PD12 STM32_FAKE_PIN(3, 12)
#define PD13 STM32_FAKE_PIN(3, 13)
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
// RCC->RSR: причина перезагрузки (hal/ResetCause.h). Биты — как в CMSIS H7.
// Тест выставляет fake::rcc().RSR; запись RMVF в фейке ничего не сбрасывает.
// ------------------------------------------------------------

#define RCC_RSR_RMVF (1UL << 16)
#define RCC_RSR_BORRSTF (1UL << 21)
#define RCC_RSR_PINRSTF (1UL << 22)
#define RCC_RSR_PORRSTF (1UL << 23)
#define RCC_RSR_SFTRSTF (1UL << 24)
#define RCC_RSR_IWDG1RSTF (1UL << 26)
#define RCC_RSR_WWDG1RSTF (1UL << 28)

struct RCC_TypeDef
{
    uint32_t RSR = RCC_RSR_PORRSTF | RCC_RSR_PINRSTF;
};

namespace fake
{
    inline RCC_TypeDef& rcc()
    {
        static RCC_TypeDef regs;
        return regs;
    }
}
#define RCC (&::fake::rcc())

// ------------------------------------------------------------
// АЦП STM32duino: analogRead() в counts заданной разрядности (по
// умолчанию 10 бит), опорное 3.3 В; напряжение на пине — fake::gpio().analogMv.
// ------------------------------------------------------------

namespace fake
{
    inline uint8_t& adcBits()
    {
        static uint8_t bits = 10;
        return bits;
    }
}

inline void analogReadResolution(uint8_t bits) { fake::adcBits() = bits; }

inline int analogRead(uint32_t pin)
{
    if (pin >= fake::GPIO_COUNT) return 0;
    fake::gpio().analogReads[pin]++;
    const uint32_t full = (1u << fake::adcBits()) - 1;
    return static_cast<int>(static_cast<uint64_t>(fake::gpio().analogMv[pin]) * full / 3300);
}

// Карта SD и HAL_SD_* (STM32H7).
#include "hal_sd_fake.h"

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

// Пины сервовыходов Config.h — как в PeripheralPins вариантов WeAct и
// DevEBox H743: TIM2 CH1..4, TIM4 CH3/CH4, TIM1 CH1; у Matek H743 ещё
// PB0/PB1 (TIM1 CH2N/CH3N — N-канал фейку не важен) и PD12/PD13 (TIM4
// CH1/CH2).
inline const PinMap PinMap_TIM[] = {
    { PA0, 2, 1 }, { PA1, 2, 2 }, { PA2, 2, 3 }, { PA3, 2, 4 },
    { PD14, 4, 3 }, { PD15, 4, 4 }, { PE9, 1, 1 },
    { PB0, 1, 2 }, { PB1, 1, 3 }, { PD12, 4, 1 }, { PD13, 4, 2 },
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
enum TimerFormat_t { MICROSEC_FORMAT, MICROSEC_COMPARE_FORMAT, TICK_FORMAT, TICK_COMPARE_FORMAT };

class HardwareTimer
{
public:
    void setup(TIM_TypeDef* timerInstance) { instance = timerInstance; }
    void setOverflow(uint32_t value, TimerFormat_t format = TICK_FORMAT)
    {
        overflow = value;
        overflowFormat = format;
        periodUs = format == TICK_FORMAT ? value * prescaler / (getTimerClkFreq() / 1000000u) : value;
    }
    void setPrescaleFactor(uint32_t value) { prescaler = value; }
    uint32_t getTimerClkFreq() const { return 240000000u; }   // APB-таймеры H743 при 480 МГц

    void setMode(uint32_t channel, TimerModes_t, PinName pin)
    {
        if (channel < 5) channelPin[channel] = pin;
        fake::timerPulses()[pin] = 0;
    }

    void setCaptureCompare(uint32_t channel, uint32_t compare, TimerFormat_t format)
    {
        if (channel >= 5) return;
        const uint32_t pin = channelPin[channel];
        // в тиках (выходы мотора) — переводим в мкс по делителю
        const uint32_t value = format == TICK_COMPARE_FORMAT
                                   ? static_cast<uint32_t>((static_cast<uint64_t>(compare) * prescaler * 1000000u + getTimerClkFreq() / 2) / getTimerClkFreq())
                                   : compare;
        lastCompare[channel] = compare;
        fake::timerPulses()[pin] = static_cast<long>(value);
        // pulseIn() видит импульс таймера (вход GPIO STM32 читает пин в AF-режиме).
        // Как у настоящего ядра: на пинах с номером бита 8+ (PD14, PE9...)
        // pulseIn(HIGH) меряет низкий уровень — маска у него uint8_t.
        if (pin < fake::GPIO_COUNT)
        {
            const bool highBit = (pin & 0x0F) >= 8;
            const long seen = highBit && value > 0 && periodUs > value ? static_cast<long>(periodUs - value)
                                                                         : static_cast<long>(value);
            fake::gpio().pulseOverride[pin] = seen;
            fake::gpio().inputEnabled[pin] = true;
        }
    }

    void resume() { running = true; }

    TIM_TypeDef* instance = nullptr;
    uint32_t periodUs = 0;
    uint32_t overflow = 0;
    uint32_t prescaler = 1;
    TimerFormat_t overflowFormat = MICROSEC_FORMAT;
    bool running = false;
    uint32_t channelPin[5] = {};
    uint32_t lastCompare[5] = {};
};
