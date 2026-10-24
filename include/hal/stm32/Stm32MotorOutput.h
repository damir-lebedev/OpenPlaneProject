#pragma once
#include <Arduino.h>

#include "copter/EscProtocol.h"
#include "hal/stm32/Stm32Dshot.h"

// ============================================================
// Выходы на регуляторы мультикоптера (STM32duino 3.x, HardwareTimer):
// PWM 490 Гц, OneShot125, OneShot42, Multishot; DShot150/300/600 — через
// Stm32Dshot.h (таймер + DMA), тем же begin()/write().
//
// Импульс формирует таймер сам, без прерываний. Аналоговые протоколы
// здесь — непрерывные: таймер повторяет импульс с частотой протокола
// (OneShot125 — 2 кГц, OneShot42 — 8 кГц, Multishot — 25 кГц), а
// ширина обновляется каждый цикл стабилизации; регуляторы BLHeli_S/32
// и AM32 принимают так любой из них. Новая ширина — с предзагрузкой:
// импульс не рвётся посередине.
//
// Шаг таймера — 1/24 мкс (делитель под 24 МГц от такта таймера): у
// Multishot (5–25 мкс) это 480 ступеней газа, у PWM — 24000.
//
// Таймер на несколько моторов (CH1..CH4 одного TIMx) — один общий
// объект HardwareTimer, как у сервовыходов (Stm32ServoOutput.h): иначе
// второй объект на том же TIMx перезаписал бы обработчик ядра. Пул у
// моторов свой: частота у них другая, чем у серв (50 Гц).
//
// Пока коптер не заармлен, на выходах — импульс «стоп» (1000 мкс у
// PWM, 125 у OneShot125): регуляторы при включении ждут его, чтобы
// взвестись, и не крутят мотор.
// ============================================================

class Stm32MotorOutput
{
public:
    static constexpr uint8_t MAX_MOTORS = 8;

    // false — ни один пин не стал выходом.
    bool begin(const int16_t* pins, uint8_t count, copter::EscProtocol esc)
    {
        protocol = esc;
        motorCount = count > MAX_MOTORS ? MAX_MOTORS : count;
        attachedCount = 0;
        if (copter::isDshot(esc))
        {
            attachedCount = dshot.begin(pins, motorCount, esc);
            return attachedCount > 0;
        }

        const copter::PulseRange range = copter::pulseRange(esc);
        const uint32_t hz = outputRateHz(esc);
        for (uint8_t i = 0; i < motorCount; ++i)
        {
            Output& o = outputs[i];
            o.attached = false;
            if (pins[i] < 0) continue;
            const PinName name = digitalPinToPinName(static_cast<pin_size_t>(pins[i]));
            auto* instance = static_cast<TIM_TypeDef*>(pinmap_peripheral(name, PinMap_TIM));
            if (instance == nullptr) continue;
            Slot* slot = acquireTimer(instance, hz);
            if (slot == nullptr) continue;
            o.timer = &slot->timer;
            o.ticksPerUs = slot->ticksPerUs;
            o.channel = STM_PIN_CHANNEL(pinmap_function(name, PinMap_TIM));
            o.timer->setMode(o.channel, TIMER_OUTPUT_COMPARE_PWM1, name);
            o.timer->setCaptureCompare(o.channel, o.ticks(range.stopUs), TICK_COMPARE_FORMAT);
            o.timer->resume();
            o.attached = true;
            ++attachedCount;
        }
        return attachedCount > 0;
    }

    // Команды 0..1 (CopterOutputs::motor); spinning = false — стоп на всех.
    void write(const float* commands, bool spinning)
    {
        if (copter::isDshot(protocol))
        {
            dshot.write(commands, spinning);
            return;
        }
        for (uint8_t i = 0; i < motorCount; ++i)
        {
            const float us = copter::pulseUs(protocol, spinning ? commands[i] : 0.0f);
            const Output& o = outputs[i];
            if (o.attached) o.timer->setCaptureCompare(o.channel, o.ticks(us), TICK_COMPARE_FORMAT);
            lastUs[i] = us;
        }
    }

    void stop()
    {
        const float zero[MAX_MOTORS] = {};
        write(zero, false);
    }

    uint8_t attached() const { return attachedCount; }
    bool isAttached(uint8_t i) const
    {
        return copter::isDshot(protocol) ? dshot.isAttached(i) : i < motorCount && outputs[i].attached;
    }
    Stm32Dshot& dshotOutput() { return dshot; }
    const Stm32Dshot& dshotOutput() const { return dshot; }
    float pulseUs(uint8_t i) const { return i < motorCount ? lastUs[i] : 0.0f; }
    copter::EscProtocol getProtocol() const { return protocol; }

    // Частота повторения импульсов протокола, Гц.
    static uint32_t outputRateHz(copter::EscProtocol esc)
    {
        switch (esc)
        {
            case copter::EscProtocol::PWM: return 490;
            case copter::EscProtocol::ONESHOT125: return 2000;
            case copter::EscProtocol::ONESHOT42: return 8000;
            case copter::EscProtocol::MULTISHOT: return 25000;
            default: return 0;
        }
    }

private:
    static constexpr uint32_t TICK_HZ = 24000000;

    struct Output
    {
        HardwareTimer* timer = nullptr;
        uint32_t channel = 0;
        float ticksPerUs = 24.0f;
        bool attached = false;

        uint32_t ticks(float us) const { return static_cast<uint32_t>(us * ticksPerUs + 0.5f); }
    };

    struct Slot
    {
        TIM_TypeDef* instance = nullptr;
        HardwareTimer timer;
        float ticksPerUs = 24.0f;
    };

    // Один HardwareTimer на TIMx; делитель — под ~24 МГц от такта таймера
    // (точный шаг считается от настоящего такта).
    static Slot* acquireTimer(TIM_TypeDef* instance, uint32_t hz)
    {
        static Slot slots[4];
        for (Slot& slot : slots)
        {
            if (slot.instance == nullptr)
            {
                slot.instance = instance;
                slot.timer.setup(instance);
                const uint32_t clock = slot.timer.getTimerClkFreq();
                const uint32_t prescaler = clock > TICK_HZ ? clock / TICK_HZ : 1;
                const uint32_t tickHz = clock / prescaler;
                slot.ticksPerUs = tickHz / 1000000.0f;
                slot.timer.setPrescaleFactor(prescaler);
                slot.timer.setOverflow(tickHz / hz, TICK_FORMAT);
                return &slot;
            }
            if (slot.instance == instance) return &slot;
        }
        return nullptr;
    }

    Output outputs[MAX_MOTORS];
    Stm32Dshot dshot;
    float lastUs[MAX_MOTORS] = {};
    uint8_t motorCount = 0;
    uint8_t attachedCount = 0;
    copter::EscProtocol protocol = copter::EscProtocol::PWM;
};
