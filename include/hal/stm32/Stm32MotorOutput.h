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

    // false — ни один пин не стал выходом. af — AF таймеров из заголовка платы (-1 — любой).
    // pwmRateHz — motor_pwm_rate (только у PWM).
    bool begin(const int16_t* pins, uint8_t count, copter::EscProtocol esc, const int8_t* af = nullptr,
               uint8_t afCount = 0, uint32_t pwmRateHz = 490)
    {
        protocol = esc;
        motorCount = count > MAX_MOTORS ? MAX_MOTORS : count;
        attachedCount = 0;
        if (copter::isDshot(esc))
        {
            attachedCount = dshot.begin(pins, motorCount, esc, af, afCount);
            return attachedCount > 0;
        }

        const uint32_t hz = copter::analogRateHz(esc, pwmRateHz);
        for (uint8_t i = 0; i < motorCount; ++i)
        {
            Output& o = outputs[i];
            o.attached = false;
            if (pins[i] < 0) continue;
            const int8_t wantAf = (af && i < afCount) ? af[i] : -1;
            PinName name;
            if (!timerPin(digitalPinToPinName(static_cast<pin_size_t>(pins[i])), wantAf, name)) continue;
            auto* instance = static_cast<TIM_TypeDef*>(pinmap_peripheral(name, PinMap_TIM));
            Slot* slot = acquireTimer(instance, hz);
            if (slot == nullptr) continue;
            o.timer = &slot->timer;
            o.ticksPerUs = slot->ticksPerUs;
            o.channel = STM_PIN_CHANNEL(pinmap_function(name, PinMap_TIM));
            o.timer->setMode(o.channel, TIMER_OUTPUT_COMPARE_PWM1, name);
            o.timer->setCaptureCompare(o.channel, o.ticks(copter::stopPulseUs(esc, minCommandUs)), TICK_COMPARE_FORMAT);
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
            // стоп (не крутится или мотор стоит) — импульс min_command
            const float us = spinning && commands[i] > 0.0f ? copter::pulseUs(protocol, commands[i])
                                                            : copter::stopPulseUs(protocol, minCommandUs);
            const Output& o = outputs[i];
            if (o.attached) o.timer->setCaptureCompare(o.channel, o.ticks(us), TICK_COMPARE_FORMAT);
            lastUs[i] = us;
        }
    }

    // min_command, мкс в шкале PWM (BF 1000): импульс «стоп» аналоговых протоколов.
    void setMinCommand(float us) { minCommandUs = us; }

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

    // Частота повторения импульсов протокола, Гц (PWM — по умолчанию 490).
    static uint32_t outputRateHz(copter::EscProtocol esc) { return copter::analogRateHz(esc); }

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

    // Вариант пина (базовый, ALT1, ALT2…) с таймером: AF из заголовка платы —
    // ровно он (как у DShot: PB0 на SpeedyBee — TIM3, а не первый в таблице
    // TIM1 CH2N), иначе первый с таймером. false — такого нет.
    static bool timerPin(PinName base, int8_t wantAf, PinName& out)
    {
        for (uint32_t alt = 0; alt <= 3; ++alt)
        {
            const PinName name = static_cast<PinName>(base | (alt << 8));   // ALT1 = 0x100 у STM32duino
            if (pinmap_peripheral(name, PinMap_TIM) == nullptr) continue;
            const int function = pinmap_function(name, PinMap_TIM);
            if (wantAf >= 0 && static_cast<int>(STM_PIN_AFNUM(function)) != wantAf) continue;
            out = name;
            return true;
        }
        return false;
    }

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
    float minCommandUs = 1000.0f;
    uint8_t motorCount = 0;
    uint8_t attachedCount = 0;
    copter::EscProtocol protocol = copter::EscProtocol::PWM;
};
