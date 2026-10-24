#pragma once
#include <Arduino.h>
#include <stddef.h>

#include "copter/EscProtocol.h"

// ============================================================
// DShot150/300/600 на STM32F405 и STM32H743 (STM32duino 3.x): таймер в
// режиме PWM + DMA в режиме burst, без прерываний.
//
// Моторы группируются по таймерам (до 4 каналов CH1..CH4 у одного TIMx).
// Таймер тикает с полной частотой (делитель 1), период — один бит DShot;
// по каждому событию обновления DMA пишет в TIMx_DMAR ширины импульсов
// этого бита для всех каналов группы (CCRn…CCRm подряд — burst). Буфер на
// кадр — 18 слотов: 16 бит и 2 нуля (линия внизу до следующего кадра).
// Кадр уходит раз в цикл стабилизации: write() собирает буфер и запускает
// поток DMA заново. Поток ещё занят прошлым кадром (цикл короче кадра) —
// кадр пропускается и считается в framesSkipped(), буфер не трогается.
//
// Потоки DMA на событие обновления таймера (TIMx_UP):
//   F405 (RM0090, таблицы запросов DMA1/DMA2): TIM1 — DMA2 S5 канал 6,
//     TIM2 — DMA1 S1 к. 3, TIM3 — DMA1 S2 к. 5, TIM4 — DMA1 S6 к. 2,
//     TIM5 — DMA1 S0 к. 6, TIM8 — DMA2 S1 к. 7;
//   H743 — любой поток через DMAMUX: DMA2 S0…S5 по порядку TIM1, 2, 3, 4,
//     5, 8, запрос — DMA_REQUEST_TIMx_UP из HAL.
// Совместимость с DMA гироскопа, OSD и флеша — пункт 081 (наши драйверы
// SPI пока без DMA). Буферы — в обычной RAM (0x20000000 у F405, AXI SRAM
// у H743): CCM и DTCM для DMA1/DMA2 недоступны. Если у H743 включён
// D-кэш — буфер перед запуском выгружается в память.
//
// Пока коптер не заармлен — кадры «стоп» (0): регулятор взводится по ним.
// Команды DShot (sendCommand: направление, пищалка, сохранение) — только
// при остановленных моторах, с битом телеметрии, повтором (как в BF).
//
// На ПК (test/native/support_stm32) вместо DMA — фейк Stm32DshotDma:
// тест видит, какой буфер ушёл на каждый таймер.
// ============================================================

#if defined(ARDUINO_ARCH_STM32)
// Поток DMA, который по событию обновления таймера пишет буфер в TIMx_DMAR.
class Stm32DshotDma
{
public:
    // firstChannel 1..4, burst — сколько CCR подряд; false — у таймера нет потока.
    bool attach(TIM_TypeDef* tim, uint8_t firstChannel, uint8_t burst, const uint32_t* buf, uint16_t words)
    {
        Route r;
        if (!route(tim, r) || firstChannel < 1 || burst < 1 || firstChannel + burst > 5) return false;
        stream = r.stream;
        dma = r.dma;
        index = r.index;
        buffer = buf;
        count = words;
        if (dma == DMA1) __HAL_RCC_DMA1_CLK_ENABLE();
        else __HAL_RCC_DMA2_CLK_ENABLE();

        stream->CR &= ~DMA_SxCR_EN;
        while (stream->CR & DMA_SxCR_EN)
        {
        }
        clearFlags();
        stream->PAR = reinterpret_cast<uint32_t>(&tim->DMAR);
        stream->M0AR = reinterpret_cast<uint32_t>(buffer);
        stream->NDTR = count;
        stream->FCR = 0;   // прямой режим, без FIFO
        uint32_t cr = DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PL_1;
#if defined(STM32F4xx)
        cr |= r.channel << DMA_SxCR_CHSEL_Pos;
#elif defined(STM32H7xx)
        auto* mux = reinterpret_cast<DMAMUX_Channel_TypeDef*>(DMAMUX1_Channel0_BASE + 4u * ((dma == DMA2 ? 8u : 0u) + index));
        mux->CCR = r.channel;   // номер запроса DMAMUX
#endif
        stream->CR = cr;

        // CCR с предзагрузкой: значение бита начинает работать со следующего бита
        for (uint8_t ch = firstChannel; ch < firstChannel + burst; ++ch)
        {
            if (ch == 1) tim->CCMR1 |= TIM_CCMR1_OC1PE;
            if (ch == 2) tim->CCMR1 |= TIM_CCMR1_OC2PE;
            if (ch == 3) tim->CCMR2 |= TIM_CCMR2_OC3PE;
            if (ch == 4) tim->CCMR2 |= TIM_CCMR2_OC4PE;
        }
        // burst: DBA — CCRfirst (номер слова от начала таймера), DBL — burst − 1
        const uint32_t dba = static_cast<uint32_t>(offsetof(TIM_TypeDef, CCR1) / 4u) + (firstChannel - 1u);
        tim->DCR = ((burst - 1u) << TIM_DCR_DBL_Pos) | (dba << TIM_DCR_DBA_Pos);
        tim->DIER |= TIM_DIER_UDE;
        return true;
    }

    bool busy() const { return stream && (stream->CR & DMA_SxCR_EN); }

    // Отправить буфер (кадр). false — прошлый кадр ещё идёт.
    bool start()
    {
        if (!stream || busy()) return false;
        clearFlags();
#if defined(STM32H7xx) && defined(SCB_CCR_DC_Msk)
        if (SCB->CCR & SCB_CCR_DC_Msk)
        {
            const uint32_t from = reinterpret_cast<uint32_t>(buffer) & ~31u;
            const uint32_t to = reinterpret_cast<uint32_t>(buffer + count);
            SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(from), static_cast<int32_t>(to - from));
        }
#endif
        stream->M0AR = reinterpret_cast<uint32_t>(buffer);
        stream->NDTR = count;
        stream->CR |= DMA_SxCR_EN;
        return true;
    }

private:
    struct Route
    {
        DMA_TypeDef* dma;
        DMA_Stream_TypeDef* stream;
        uint8_t index;
        uint32_t channel;   // F4 — канал CHSEL, H7 — запрос DMAMUX
    };

    static bool route(TIM_TypeDef* t, Route& r)
    {
#if defined(STM32F4xx)
        if (t == TIM1) r = { DMA2, DMA2_Stream5, 5, 6 };
        else if (t == TIM2) r = { DMA1, DMA1_Stream1, 1, 3 };
        else if (t == TIM3) r = { DMA1, DMA1_Stream2, 2, 5 };
        else if (t == TIM4) r = { DMA1, DMA1_Stream6, 6, 2 };
        else if (t == TIM5) r = { DMA1, DMA1_Stream0, 0, 6 };
        else if (t == TIM8) r = { DMA2, DMA2_Stream1, 1, 7 };
        else return false;
        return true;
#elif defined(STM32H7xx)
        if (t == TIM1) r = { DMA2, DMA2_Stream0, 0, DMA_REQUEST_TIM1_UP };
        else if (t == TIM2) r = { DMA2, DMA2_Stream1, 1, DMA_REQUEST_TIM2_UP };
        else if (t == TIM3) r = { DMA2, DMA2_Stream2, 2, DMA_REQUEST_TIM3_UP };
        else if (t == TIM4) r = { DMA2, DMA2_Stream3, 3, DMA_REQUEST_TIM4_UP };
        else if (t == TIM5) r = { DMA2, DMA2_Stream4, 4, DMA_REQUEST_TIM5_UP };
        else if (t == TIM8) r = { DMA2, DMA2_Stream5, 5, DMA_REQUEST_TIM8_UP };
        else return false;
        return true;
#else
        (void)t;
        (void)r;
        return false;
#endif
    }

    // Все флаги потока (TC, HT, TE, DME, FE) — сброс.
    void clearFlags()
    {
        static const uint8_t SHIFT[4] = { 0, 6, 16, 22 };
        const uint32_t mask = 0x3Du << SHIFT[index & 3u];
        if (index < 4) dma->LIFCR = mask;
        else dma->HIFCR = mask;
    }

    DMA_TypeDef* dma = nullptr;
    DMA_Stream_TypeDef* stream = nullptr;
    uint8_t index = 0;
    const uint32_t* buffer = nullptr;
    uint16_t count = 0;
};
#endif   // ARDUINO_ARCH_STM32 (на ПК — фейк из test/native/support_stm32)

class Stm32Dshot
{
public:
    static constexpr uint8_t MAX_MOTORS = 8;
    static constexpr uint8_t MAX_TIMERS = 4;
    static constexpr uint8_t COMMAND_REPEATS = 10;   // как в BF: команду — 10 кадров подряд

    // Число моторов, которые стали выходами DShot (0 — ни одного).
    uint8_t begin(const int16_t* pins, uint8_t count, copter::EscProtocol esc)
    {
        protocol = esc;
        motorCount = count > MAX_MOTORS ? MAX_MOTORS : count;
        attachedCount = 0;
        groupCount = 0;
        for (Group& grp : groups)
        {
            grp.instance = nullptr;
            for (auto& m : grp.motorAt) m = -1;
            grp.ok = false;
        }
        if (!copter::isDshot(esc)) return 0;
        for (uint8_t i = 0; i < motorCount; ++i)
        {
            motors[i] = Motor();
            if (pins[i] < 0) continue;
            const PinName name = digitalPinToPinName(static_cast<pin_size_t>(pins[i]));
            auto* instance = static_cast<TIM_TypeDef*>(pinmap_peripheral(name, PinMap_TIM));
            const uint32_t channel = STM_PIN_CHANNEL(pinmap_function(name, PinMap_TIM));
            if (instance == nullptr || channel < 1 || channel > 4) continue;
            const int g = groupFor(instance);
            if (g < 0) continue;
            Group& grp = groups[g];
            if (grp.motorAt[channel - 1] >= 0) continue;   // канал уже занят другим мотором
            grp.motorAt[channel - 1] = static_cast<int8_t>(i);
            grp.pin[channel - 1] = name;
            motors[i].group = static_cast<int8_t>(g);
            motors[i].channel = static_cast<uint8_t>(channel);
        }
        for (uint8_t g = 0; g < groupCount; ++g)
        {
            Group& grp = groups[g];
            grp.first = 5;
            uint8_t last = 0;
            for (uint8_t ch = 1; ch <= 4; ++ch)
            {
                if (grp.motorAt[ch - 1] < 0) continue;
                if (ch < grp.first) grp.first = ch;
                last = ch;
            }
            grp.burst = static_cast<uint8_t>(last - grp.first + 1);
            grp.timer.setup(grp.instance);
            const uint32_t clock = grp.timer.getTimerClkFreq();
            grp.ticks = copter::dshot::bitTicks(esc, clock);
            grp.timer.setPrescaleFactor(1);
            grp.timer.setOverflow(grp.ticks.period, TICK_FORMAT);
            for (uint8_t ch = grp.first; ch <= last; ++ch)
            {
                if (grp.motorAt[ch - 1] < 0) continue;
                grp.timer.setMode(ch, TIMER_OUTPUT_COMPARE_PWM1, grp.pin[ch - 1]);
                grp.timer.setCaptureCompare(ch, 0, TICK_COMPARE_FORMAT);
            }
            grp.timer.resume();
            for (auto& w : grp.buffer) w = 0;
            grp.ok = grp.dma.attach(grp.instance, grp.first, grp.burst, grp.buffer,
                                    static_cast<uint16_t>(copter::dshot::FRAME_SLOTS * grp.burst));
            if (!grp.ok) continue;
            for (uint8_t ch = grp.first; ch <= last; ++ch)
            {
                const int8_t m = grp.motorAt[ch - 1];
                if (m >= 0)
                {
                    motors[m].attached = true;
                    ++attachedCount;
                }
            }
        }
        return attachedCount;
    }

    // Раз в цикл: команды 0..1 (CopterOutputs::motor); spinning = false — «стоп»
    // (или команда из очереди sendCommand).
    void write(const float* commands, bool spinning)
    {
        for (uint8_t i = 0; i < motorCount; ++i)
        {
            Motor& m = motors[i];
            bool telemetry = false;
            uint16_t value;
            if (!spinning && m.commandLeft > 0)
            {
                value = m.command;
                telemetry = true;   // команды BLHeli_32/Bluejay принимают только с битом телеметрии
                --m.commandLeft;
            }
            else
            {
                value = copter::dshot::throttleValue(spinning ? commands[i] : 0.0f, spinning);
            }
            m.value = value;
            m.frame = copter::dshot::frame(value, telemetry, false);
        }
        for (uint8_t g = 0; g < groupCount; ++g)
        {
            Group& grp = groups[g];
            if (!grp.ok) continue;
            if (grp.dma.busy())   // прошлый кадр ещё идёт — этот пропускаем
            {
                ++skipped;
                continue;
            }
            uint16_t frames[copter::dshot::BURST_MAX] = {};
            bool used[copter::dshot::BURST_MAX] = {};
            for (uint8_t k = 0; k < grp.burst; ++k)
            {
                const int8_t m = grp.motorAt[grp.first - 1 + k];
                used[k] = m >= 0;
                if (used[k]) frames[k] = motors[m].frame;
            }
            copter::dshot::burstBuffer(frames, used, grp.burst, grp.ticks, grp.buffer);
            if (grp.dma.start()) ++sent;
            else ++skipped;
        }
    }

    // Команда DShot (1..47) мотору index (0xFF — всем), повтор COMMAND_REPEATS
    // кадров; уходит, пока моторы стоят (write с spinning = false).
    void sendCommand(uint8_t index, uint16_t command, uint8_t repeats = COMMAND_REPEATS)
    {
        if (command == 0 || command >= copter::dshot::MIN_THROTTLE) return;
        for (uint8_t i = 0; i < motorCount; ++i)
        {
            if (index != 0xFF && i != index) continue;
            motors[i].command = command;
            motors[i].commandLeft = repeats;
        }
    }

    bool commandPending() const
    {
        for (uint8_t i = 0; i < motorCount; ++i)
        {
            if (motors[i].commandLeft) return true;
        }
        return false;
    }

    uint8_t attached() const { return attachedCount; }
    bool isAttached(uint8_t i) const { return i < motorCount && motors[i].attached; }
    uint16_t value(uint8_t i) const { return i < motorCount ? motors[i].value : 0; }    // 0..2047 последнего кадра
    uint16_t frame(uint8_t i) const { return i < motorCount ? motors[i].frame : 0; }    // кадр с CRC
    uint8_t timerCount() const { return groupCount; }
    uint32_t framesSent() const { return sent; }
    uint32_t framesSkipped() const { return skipped; }
    copter::dshot::BitTicks bitTicks(uint8_t timer) const { return timer < groupCount ? groups[timer].ticks : copter::dshot::BitTicks{ 0, 0, 0 }; }
    copter::EscProtocol getProtocol() const { return protocol; }

private:
    struct Motor
    {
        int8_t group = -1;
        uint8_t channel = 0;
        bool attached = false;
        uint16_t value = 0;
        uint16_t frame = 0;
        uint16_t command = 0;
        uint8_t commandLeft = 0;
    };

    struct Group
    {
        TIM_TypeDef* instance = nullptr;
        HardwareTimer timer;
        Stm32DshotDma dma;
        int8_t motorAt[4] = { -1, -1, -1, -1 };   // канал CH1..CH4 -> мотор
        PinName pin[4] = {};
        uint8_t first = 1, burst = 1;
        bool ok = false;
        copter::dshot::BitTicks ticks{ 0, 0, 0 };
        alignas(32) uint32_t buffer[copter::dshot::FRAME_SLOTS * copter::dshot::BURST_MAX] = {};
    };

    int groupFor(TIM_TypeDef* instance)
    {
        for (uint8_t g = 0; g < groupCount; ++g)
        {
            if (groups[g].instance == instance) return g;
        }
        if (groupCount >= MAX_TIMERS) return -1;
        groups[groupCount].instance = instance;
        return groupCount++;
    }

    Group groups[MAX_TIMERS];
    uint8_t groupCount = 0;
    Motor motors[MAX_MOTORS];
    uint8_t motorCount = 0;
    uint8_t attachedCount = 0;
    uint32_t sent = 0, skipped = 0;
    copter::EscProtocol protocol = copter::EscProtocol::DSHOT600;
};
