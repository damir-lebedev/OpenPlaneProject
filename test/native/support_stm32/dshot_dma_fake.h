#pragma once

// ============================================================
// Фейк потока DMA для DShot (hal/stm32/Stm32Dshot.h): вместо регистров
// DMA и TIMx_DMAR — запись того, что ушло бы на таймер.
//
//   fake::dshotDma(timer) — последний отправленный буфер таймера TIMx,
//                           сколько раз отправляли, первый канал, burst;
//   fake::dshotDmaSlow()  — true: поток «занят», пока тест не вызовет
//                           fake::dshotDmaFinish() (кадр длиннее цикла);
//   fake::dshotNoDma()    — номер таймера без потока DMA (attach -> false);
//   fake::dshotReply(timer, channel) — отсчёты линии, которые «ответит»
//                           регулятор этого канала (bidir), и отсчётов на бит.
// ============================================================

#include <stdint.h>

#include <map>
#include <vector>

namespace fake
{
    struct DshotDmaRecord
    {
        uint8_t firstChannel = 0;
        uint8_t burst = 0;
        uint32_t starts = 0;
        bool busy = false;
        bool inverted = false;
        std::vector<uint32_t> sent;   // буфер на момент запуска
    };

    inline std::map<int, DshotDmaRecord>& dshotDmaRecords()
    {
        static std::map<int, DshotDmaRecord> records;
        return records;
    }

    inline DshotDmaRecord* dshotDma(int timer)
    {
        auto it = dshotDmaRecords().find(timer);
        return it == dshotDmaRecords().end() ? nullptr : &it->second;
    }

    inline bool& dshotDmaSlow()
    {
        static bool slow = false;
        return slow;
    }

    inline int& dshotNoDma()
    {
        static int timer = -1;
        return timer;
    }

    struct DshotReply
    {
        std::vector<uint8_t> samples;
        float samplesPerBit = 0.0f;
    };

    inline std::map<int, DshotReply>& dshotReplies()
    {
        static std::map<int, DshotReply> replies;
        return replies;
    }

    inline DshotReply& dshotReply(int timer, int channel) { return dshotReplies()[timer * 10 + channel]; }

    inline void dshotDmaFinish()
    {
        for (auto& r : dshotDmaRecords()) r.second.busy = false;
    }

    inline void dshotDmaReset()
    {
        dshotDmaRecords().clear();
        dshotReplies().clear();
        dshotDmaSlow() = false;
        dshotNoDma() = -1;
    }
}

class Stm32DshotDma
{
public:
    bool attach(TIM_TypeDef* tim, uint8_t firstChannel, uint8_t burst, const uint32_t* buf, uint16_t words,
                bool inverted = false)
    {
        if (!tim || tim->number == fake::dshotNoDma() || firstChannel < 1 || firstChannel + burst > 5) return false;
        timer = tim->number;
        buffer = buf;
        count = words;
        fake::DshotDmaRecord& r = fake::dshotDmaRecords()[timer];
        r = fake::DshotDmaRecord();
        r.firstChannel = firstChannel;
        r.burst = burst;
        r.inverted = inverted;
        return true;
    }

    bool busy() const
    {
        const fake::DshotDmaRecord* r = fake::dshotDma(timer);
        return r && r->busy;
    }

    bool start()
    {
        fake::DshotDmaRecord* r = fake::dshotDma(timer);
        if (!r || r->busy) return false;
        r->sent.assign(buffer, buffer + count);
        ++r->starts;
        r->busy = fake::dshotDmaSlow();
        return true;
    }

private:
    int timer = -1;
    const uint32_t* buffer = nullptr;
    uint16_t count = 0;
};

// Захват ответа регулятора: отдаёт то, что тест положил в fake::dshotReply.
class Stm32DshotCapture
{
public:
    bool attach(TIM_TypeDef* tim, const PinName*, float spb)
    {
        timer = tim ? tim->number : -1;
        nominalSpb = spb;
        return timer >= 0;
    }
    void start() { ++starts; }
    bool samples(uint8_t channel, const uint8_t*& data, uint16_t& count, float& spb)
    {
        fake::DshotReply& r = fake::dshotReply(timer, channel);
        if (r.samples.empty()) return false;
        data = r.samples.data();
        count = static_cast<uint16_t>(r.samples.size());
        spb = r.samplesPerBit > 0.0f ? r.samplesPerBit : nominalSpb;
        return true;
    }
    uint32_t starts = 0;

private:
    int timer = -1;
    float nominalSpb = 0.0f;
};
