#pragma once
#include <Arduino.h>
#include <stddef.h>

#include "copter/EscProtocol.h"

// ============================================================
// DShot150/300/600 на STM32F405 и STM32H743 (STM32duino 3.x): таймер в
// режиме PWM + DMA в режиме burst, без прерываний.
//
// Моторы группируются по таймерам (до 4 каналов CH1..CH4 у одного TIMx,
// до 4 таймеров — M1–M8). Таймер тикает с полной частотой (делитель 1),
// период — один бит DShot; по каждому событию обновления DMA пишет в
// TIMx_DMAR ширины импульсов этого бита для всех каналов группы (CCRn…CCRm
// подряд — burst). Буфер на кадр — 18 слотов: 16 бит и 2 нуля (линия в
// покое до следующего кадра). Кадр уходит раз в цикл: write() сначала
// собирает буферы всех таймеров, потом запускает их потоки DMA подряд —
// кадры моторов начинаются почти одновременно (разница — запись регистров,
// доли микросекунды). Поток ещё занят прошлым кадром (цикл короче кадра) —
// кадр этого таймера пропускается (framesSkipped()), буфер не трогается.
//
// Какой таймер у пина. У пина бывает несколько таймеров (PC6 — TIM3 и
// TIM8). Заголовок платы (tools/bf_board.py по строкам `timer` дампа) даёт
// AF, который выбрал Betaflight: берётся таймер с этим AF — так же, как у
// BF на этой плате. AF неизвестен — первый таймер пина, у которого есть
// поток DMA и свободен канал; занят — следующий (ALT1, ALT2…).
//
// Потоки DMA на событие обновления таймера (TIMx_UP) — dshot_dma::route():
//   F405 (RM0090, таблицы запросов DMA1/DMA2): TIM1 — DMA2 S5 канал 6,
//     TIM2 — DMA1 S1 к. 3, TIM3 — DMA1 S2 к. 5, TIM4 — DMA1 S6 к. 2,
//     TIM5 — DMA1 S0 к. 6, TIM8 — DMA2 S1 к. 7;
//   H743 — любой поток через DMAMUX: DMA2 S0…S5 по порядку TIM1, 2, 3, 4,
//     5, 8, запрос — DMA_REQUEST_TIMx_UP из HAL.
// У разных таймеров потоки разные (тест), а остальные драйверы прошивки
// DMA не берут: SPI гироскопа, OSD и флеша, UART, АЦП — опросом, SD у
// H743 — своим IDMA. Буферы — в обычной RAM (0x20000000 у F405, AXI SRAM у
// H743): CCM и DTCM для DMA1/DMA2 недоступны. Если у H743 включён D-кэш —
// буфер перед запуском выгружается в память.
//
// Пока коптер не заармлен — кадры «стоп» (0): регулятор взводится по ним.
// Команды DShot (sendCommand: направление, пищалка, сохранение) — только
// при остановленных моторах, с битом телеметрии, повтором (как в BF).
//
// Двунаправленный DShot (setBidirectional, dshot_bidir): линия инвертирована
// (покой — высокий), CRC кадра инвертирован, после кадра регулятор отвечает
// оборотами (copter/EscProtocol.h). Ответ читает Stm32DshotCapture: отсчёты
// уровня линии каждого мотора -> dshot::samplesToLine -> eRPM. На железе
// захвата пока нет (нужен стенд: у F405 читать ножки умеет только DMA2,
// а его потоки заняты TIM1/TIM8 — у BF для этого свой bitbang-драйвер),
// поэтому там ответа нет и ошибки телеметрии — 100 %; кадры bidir при этом
// уходят, моторы крутятся.
//
// На ПК (test/native/support_stm32) вместо DMA и захвата — фейки
// Stm32DshotDma / Stm32DshotCapture: тест видит буфер каждого таймера и
// подкладывает ответы регуляторов.
// ============================================================

namespace dshot_dma
{
    struct Route
    {
        uint8_t dma;       // 1 или 2
        uint8_t stream;    // 0..7
        uint8_t channel;   // F405: канал CHSEL; H743 — не нужен (DMAMUX)
    };

    // Поток DMA события обновления TIMtimer. false — у таймера потока нет.
    inline bool f405(uint8_t timer, Route& r)
    {
        switch (timer)
        {
            case 1: r = { 2, 5, 6 }; return true;
            case 2: r = { 1, 1, 3 }; return true;
            case 3: r = { 1, 2, 5 }; return true;
            case 4: r = { 1, 6, 2 }; return true;
            case 5: r = { 1, 0, 6 }; return true;
            case 8: r = { 2, 1, 7 }; return true;
            default: return false;
        }
    }

    inline bool h743(uint8_t timer, Route& r)
    {
        switch (timer)
        {
            case 1: r = { 2, 0, 0 }; return true;
            case 2: r = { 2, 1, 0 }; return true;
            case 3: r = { 2, 2, 0 }; return true;
            case 4: r = { 2, 3, 0 }; return true;
            case 5: r = { 2, 4, 0 }; return true;
            case 8: r = { 2, 5, 0 }; return true;
            default: return false;
        }
    }

    inline bool route(uint8_t timer, Route& r)
    {
#if defined(BOARD_STM32F405) || defined(STM32F4xx)
        return f405(timer, r);
#else
        return h743(timer, r);
#endif
    }
}

#if defined(ARDUINO_ARCH_STM32)
// Номер таймера по адресу (1..8, 0 — не таймер DShot).
inline uint8_t dshotTimerNumber(TIM_TypeDef* t)
{
    if (t == TIM1) return 1;
    if (t == TIM2) return 2;
    if (t == TIM3) return 3;
    if (t == TIM4) return 4;
    if (t == TIM5) return 5;
    if (t == TIM8) return 8;
    return 0;
}

// Поток DMA, который по событию обновления таймера пишет буфер в TIMx_DMAR.
class Stm32DshotDma
{
public:
    // firstChannel 1..4, burst — сколько CCR подряд; inverted — покой высокий
    // (bidir). false — у таймера нет потока.
    bool attach(TIM_TypeDef* tim, uint8_t firstChannel, uint8_t burst, const uint32_t* buf, uint16_t words, bool inverted)
    {
        dshot_dma::Route r;
        if (!dshot_dma::route(dshotTimerNumber(tim), r) || firstChannel < 1 || burst < 1 || firstChannel + burst > 5) return false;
        dma = r.dma == 1 ? DMA1 : DMA2;
        index = r.stream;
        stream = streamOf(r.dma, r.stream);
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
        cr |= static_cast<uint32_t>(r.channel) << DMA_SxCR_CHSEL_Pos;
#elif defined(STM32H7xx)
        auto* mux = reinterpret_cast<DMAMUX_Channel_TypeDef*>(DMAMUX1_Channel0_BASE + 4u * ((dma == DMA2 ? 8u : 0u) + index));
        mux->CCR = requestOf(dshotTimerNumber(tim));
#endif
        stream->CR = cr;

        for (uint8_t ch = firstChannel; ch < firstChannel + burst; ++ch)
        {
            // CCR с предзагрузкой: значение бита начинает работать со следующего бита
            if (ch == 1) tim->CCMR1 |= TIM_CCMR1_OC1PE;
            if (ch == 2) tim->CCMR1 |= TIM_CCMR1_OC2PE;
            if (ch == 3) tim->CCMR2 |= TIM_CCMR2_OC3PE;
            if (ch == 4) tim->CCMR2 |= TIM_CCMR2_OC4PE;
            // bidir: активный уровень — низкий (и у N-выхода тоже)
            const uint32_t pol = (TIM_CCER_CC1P | TIM_CCER_CC1NP) << (4u * (ch - 1u));
            if (inverted) tim->CCER |= pol;
            else tim->CCER &= ~pol;
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
    static DMA_Stream_TypeDef* streamOf(uint8_t dmaNumber, uint8_t s)
    {
        static DMA_Stream_TypeDef* const DMA1S[8] = { DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
                                                      DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7 };
        static DMA_Stream_TypeDef* const DMA2S[8] = { DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
                                                      DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7 };
        return dmaNumber == 1 ? DMA1S[s & 7u] : DMA2S[s & 7u];
    }

#if defined(STM32H7xx)
    static uint32_t requestOf(uint8_t timer)
    {
        switch (timer)
        {
            case 1: return DMA_REQUEST_TIM1_UP;
            case 2: return DMA_REQUEST_TIM2_UP;
            case 3: return DMA_REQUEST_TIM3_UP;
            case 4: return DMA_REQUEST_TIM4_UP;
            case 5: return DMA_REQUEST_TIM5_UP;
            default: return DMA_REQUEST_TIM8_UP;
        }
    }
#endif

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

// Захват ответа регулятора (bidir). Пока не написан — нужен стенд (см. выше):
// attach() -> false, ответов нет.
class Stm32DshotCapture
{
public:
    bool attach(TIM_TypeDef*, const PinName*, float) { return false; }
    void start() {}
    // Отсчёты уровня линии канала CHn (1 — высокий) и сколько их на бит ответа.
    bool samples(uint8_t, const uint8_t*&, uint16_t&, float&) { return false; }
};
#endif   // ARDUINO_ARCH_STM32 (на ПК — фейки из test/native/support_stm32)

class Stm32Dshot : public copter::IMotorTelemetry
{
public:
    static constexpr uint8_t MAX_MOTORS = 8;
    static constexpr uint8_t MAX_TIMERS = 4;
    static constexpr uint8_t COMMAND_REPEATS = 10;     // как в BF: команду — 10 кадров подряд
    static constexpr uint16_t TELEMETRY_WINDOW = 1000;  // ошибки телеметрии — доля за столько кадров

    // Двунаправленный DShot — до begin().
    void setBidirectional(bool on) { bidir = on; }
    bool isBidirectional() const { return bidir; }

    // Число моторов, которые стали выходами DShot (0 — ни одного). af[i] — AF
    // таймера мотора i из заголовка платы (−1 или нет массива — любой).
    uint8_t begin(const int16_t* pins, uint8_t count, copter::EscProtocol esc, const int8_t* af = nullptr,
                  uint8_t afCount = 0)
    {
        protocol = esc;
        motorCount = count > MAX_MOTORS ? MAX_MOTORS : count;
        attachedCount = 0;
        groupCount = 0;
        sent = skipped = 0;
        for (Group& grp : groups)
        {
            grp.instance = nullptr;
            for (auto& m : grp.motorAt) m = -1;
            grp.ok = grp.captureOk = grp.armed = false;
        }
        for (uint8_t i = 0; i < MAX_MOTORS; ++i) motors[i] = Motor();
        if (!copter::isDshot(esc)) return 0;
        for (uint8_t i = 0; i < motorCount; ++i)
        {
            if (pins[i] < 0) continue;
            const int8_t wantAf = (af && i < afCount) ? af[i] : -1;
            placeMotor(i, digitalPinToPinName(static_cast<pin_size_t>(pins[i])), wantAf);
        }
        for (uint8_t g = 0; g < groupCount; ++g) setupGroup(groups[g], esc);
        return attachedCount;
    }

    // Раз в цикл: команды 0..1 (CopterOutputs::motor); spinning = false — «стоп»
    // (или команда из очереди sendCommand).
    void write(const float* commands, bool spinning)
    {
        if (bidir) collectTelemetry();
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
            m.frame = copter::dshot::frame(value, telemetry, bidir);
        }
        // сначала буферы всех таймеров, потом запуск подряд — кадры почти одновременно
        bool ready[MAX_TIMERS] = {};
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
            ready[g] = true;
        }
        for (uint8_t g = 0; g < groupCount; ++g)
        {
            if (!ready[g]) continue;
            Group& grp = groups[g];
            if (grp.dma.start())
            {
                ++sent;
                if (grp.captureOk) grp.capture.start();
                grp.armed = true;   // после этого кадра ждём ответ
            }
            else
            {
                ++skipped;
            }
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
    uint8_t timerOf(uint8_t i) const { return i < motorCount && motors[i].group >= 0 ? groups[motors[i].group].number : 0; }
    uint8_t channelOf(uint8_t i) const { return i < motorCount ? motors[i].channel : 0; }
    uint32_t framesSent() const { return sent; }
    uint32_t framesSkipped() const { return skipped; }
    copter::dshot::BitTicks bitTicks(uint8_t timer) const { return timer < groupCount ? groups[timer].ticks : copter::dshot::BitTicks{ 0, 0, 0 }; }
    copter::EscProtocol getProtocol() const { return protocol; }

    // ---- телеметрия оборотов (bidir), copter::IMotorTelemetry ----
    bool telemetryEnabled() const override { return bidir && attachedCount > 0; }
    uint8_t telemetryMotors() const override { return motorCount; }
    bool hasTelemetry(uint8_t i) const override { return i < motorCount && motors[i].telemetryValid; }
    uint32_t erpm(uint8_t i) const override { return i < motorCount ? motors[i].erpm : 0; }
    // Доля кадров без верного ответа, %, за последние TELEMETRY_WINDOW кадров
    // (пока окно не набралось — за то, что есть).
    float telemetryErrorPercent(uint8_t i) const override
    {
        if (i >= motorCount) return 100.0f;
        const Motor& m = motors[i];
        const uint32_t total = m.telOk + m.telBad;
        if (total == 0) return m.lastWindowPercent;
        return m.windowDone ? m.lastWindowPercent : 100.0f * m.telBad / total;
    }

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
        // телеметрия
        bool telemetryValid = false;
        uint32_t erpm = 0;
        uint16_t telOk = 0, telBad = 0;
        bool windowDone = false;
        float lastWindowPercent = 100.0f;
    };

    struct Group
    {
        TIM_TypeDef* instance = nullptr;
        uint8_t number = 0;
        HardwareTimer timer;
        Stm32DshotDma dma;
        Stm32DshotCapture capture;
        int8_t motorAt[4] = { -1, -1, -1, -1 };   // канал CH1..CH4 -> мотор
        PinName pin[4] = {};
        uint8_t first = 1, burst = 1;
        bool ok = false, captureOk = false, armed = false;
        copter::dshot::BitTicks ticks{ 0, 0, 0 };
        alignas(32) uint32_t buffer[copter::dshot::FRAME_SLOTS * copter::dshot::BURST_MAX] = {};
    };

    static uint8_t numberOf(TIM_TypeDef* t)
    {
#if defined(ARDUINO_ARCH_STM32)
        return dshotTimerNumber(t);
#else
        return static_cast<uint8_t>(t->number);
#endif
    }

    // Таймер и канал для мотора: AF из заголовка платы — ровно он; иначе
    // первый вариант пина (базовый, ALT1, ALT2…) с потоком DMA и свободным каналом.
    void placeMotor(uint8_t i, PinName base, int8_t wantAf)
    {
        for (uint32_t alt = 0; alt <= 3; ++alt)
        {
            const PinName name = static_cast<PinName>(base | (alt << 8));   // ALT1 = 0x100 у STM32duino
            auto* instance = static_cast<TIM_TypeDef*>(pinmap_peripheral(name, PinMap_TIM));
            if (instance == nullptr) continue;
            const int function = pinmap_function(name, PinMap_TIM);
            const uint32_t channel = STM_PIN_CHANNEL(function);
            if (channel < 1 || channel > 4) continue;
            if (wantAf >= 0 && static_cast<int>(STM_PIN_AFNUM(function)) != wantAf) continue;
            dshot_dma::Route r;
            const uint8_t number = numberOf(instance);
            if (!dshot_dma::route(number, r)) continue;
            const int g = groupFor(instance, number);
            if (g < 0) continue;
            Group& grp = groups[g];
            if (grp.motorAt[channel - 1] >= 0) continue;   // канал занят другим мотором — следующий вариант
            grp.motorAt[channel - 1] = static_cast<int8_t>(i);
            grp.pin[channel - 1] = name;
            motors[i].group = static_cast<int8_t>(g);
            motors[i].channel = static_cast<uint8_t>(channel);
            return;
        }
    }

    void setupGroup(Group& grp, copter::EscProtocol esc)
    {
        grp.first = 5;
        uint8_t last = 0;
        for (uint8_t ch = 1; ch <= 4; ++ch)
        {
            if (grp.motorAt[ch - 1] < 0) continue;
            if (ch < grp.first) grp.first = ch;
            last = ch;
        }
        if (last == 0) return;   // группа без моторов (все каналы ушли в другие варианты)
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
                                static_cast<uint16_t>(copter::dshot::FRAME_SLOTS * grp.burst), bidir);
        if (!grp.ok) return;
        if (bidir)
        {
            const float tickHz = static_cast<float>(clock);
            const float spb = tickHz / static_cast<float>(copter::dshot::telemetryBitRate(esc));
            grp.captureOk = grp.capture.attach(grp.instance, grp.pin, spb);
        }
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

    // Ответы на прошлый кадр: каждому мотору — обороты или ошибка.
    void collectTelemetry()
    {
        for (uint8_t g = 0; g < groupCount; ++g)
        {
            Group& grp = groups[g];
            if (!grp.ok || !grp.armed) continue;
            grp.armed = false;
            for (uint8_t ch = grp.first; ch < grp.first + grp.burst; ++ch)
            {
                const int8_t m = grp.motorAt[ch - 1];
                if (m < 0) continue;
                const uint8_t* data = nullptr;
                uint16_t n = 0;
                float spb = 0.0f;
                uint32_t line = 0;
                uint16_t value = 0;
                const bool ok = grp.captureOk && grp.capture.samples(ch, data, n, spb) &&
                                copter::dshot::samplesToLine(data, n, spb, line) &&
                                copter::dshot::decodeTelemetryLine(line, value);
                count(motors[m], ok, value);
            }
        }
    }

    static void count(Motor& m, bool ok, uint16_t value)
    {
        if (ok)
        {
            m.erpm = copter::dshot::telemetryToErpm(value);
            m.telemetryValid = true;
            ++m.telOk;
        }
        else
        {
            ++m.telBad;
        }
        if (m.telOk + m.telBad >= TELEMETRY_WINDOW)
        {
            m.lastWindowPercent = 100.0f * m.telBad / (m.telOk + m.telBad);
            m.windowDone = true;
            m.telOk = m.telBad = 0;
        }
    }

    int groupFor(TIM_TypeDef* instance, uint8_t number)
    {
        for (uint8_t g = 0; g < groupCount; ++g)
        {
            if (groups[g].instance == instance) return g;
        }
        if (groupCount >= MAX_TIMERS) return -1;
        groups[groupCount].instance = instance;
        groups[groupCount].number = number;
        return groupCount++;
    }

    Group groups[MAX_TIMERS];
    uint8_t groupCount = 0;
    Motor motors[MAX_MOTORS];
    uint8_t motorCount = 0;
    uint8_t attachedCount = 0;
    uint32_t sent = 0, skipped = 0;
    bool bidir = false;
    copter::EscProtocol protocol = copter::EscProtocol::DSHOT600;
};
