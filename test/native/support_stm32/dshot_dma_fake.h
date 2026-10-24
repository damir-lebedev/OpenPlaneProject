#pragma once

// ============================================================
// Фейк потока DMA для DShot (hal/stm32/Stm32Dshot.h): вместо регистров
// DMA и TIMx_DMAR — запись того, что ушло бы на таймер.
//
//   fake::dshotDma(timer) — последний отправленный буфер таймера TIMx,
//                           сколько раз отправляли, первый канал, burst;
//   fake::dshotDmaSlow()  — true: поток «занят», пока тест не вызовет
//                           fake::dshotDmaFinish() (кадр длиннее цикла);
//   fake::dshotNoDma()    — номер таймера без потока DMA (attach -> false).
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

    inline void dshotDmaFinish()
    {
        for (auto& r : dshotDmaRecords()) r.second.busy = false;
    }

    inline void dshotDmaReset()
    {
        dshotDmaRecords().clear();
        dshotDmaSlow() = false;
        dshotNoDma() = -1;
    }
}

class Stm32DshotDma
{
public:
    bool attach(TIM_TypeDef* tim, uint8_t firstChannel, uint8_t burst, const uint32_t* buf, uint16_t words)
    {
        if (!tim || tim->number == fake::dshotNoDma() || firstChannel < 1 || firstChannel + burst > 5) return false;
        timer = tim->number;
        buffer = buf;
        count = words;
        fake::DshotDmaRecord& r = fake::dshotDmaRecords()[timer];
        r = fake::DshotDmaRecord();
        r.firstChannel = firstChannel;
        r.burst = burst;
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
