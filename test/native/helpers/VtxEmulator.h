#pragma once
#include <stdint.h>

#include <initializer_list>
#include <vector>

#include "copter/Vtx.h"

// ============================================================
// Видеопередатчик на проводе управления — для тестов: SmartAudio (v1, v2,
// v2.1; 4800 бод, 8N2) или Tramp (9600, 8N1). Получает байты от полётника
// и возвращает ответ, как настоящий: на чужой скорости или формате кадра —
// молчит. Состояние открыто тесту (частота, мощность, PIT), его можно
// менять «кнопкой на передатчике».
// ============================================================

struct VtxEmulator
{
    enum Kind { SMARTAUDIO, TRAMP, ABSENT } kind = SMARTAUDIO;
    uint8_t saVersion = 21;
    int frames = 0;

    uint16_t freq = 5865;
    bool userFreq = false;
    uint8_t channel = 0;
    uint8_t dbm = 14;          // SA 2.1
    uint8_t powerIndex = 0;    // SA v1/v2
    bool pit = false;
    uint16_t trampMw = 25;
    uint16_t trampMax = 1600;
    std::vector<uint8_t> dbmLevels = { 14, 20, 26, 30, 33, 36 };   // 25 мВт … 4 Вт

    // Байты от полётника -> ответ (пусто — молчит).
    std::vector<uint8_t> receive(const uint8_t* data, size_t n, uint32_t baud, bool twoStop)
    {
        out.clear();
        if (kind == SMARTAUDIO && baud == 4800 && twoStop) smartAudio(data, n);
        if (kind == TRAMP && baud == 9600 && !twoStop) tramp(data, n);
        return out;
    }

private:
    std::vector<uint8_t> out;

    void saReply(uint8_t cmd, const std::vector<uint8_t>& payload)
    {
        std::vector<uint8_t> f = { 0xAA, 0x55, cmd, static_cast<uint8_t>(payload.size()) };
        f.insert(f.end(), payload.begin(), payload.end());
        f.push_back(copter::smartAudioCrc(f.data(), static_cast<uint8_t>(f.size())));
        out.insert(out.end(), f.begin(), f.end());
    }

    void saSettings()
    {
        static const uint8_t V1[4] = { 7, 16, 25, 40 };
        const uint8_t code = saVersion == 1 ? 0x01 : saVersion == 2 ? 0x09 : 0x11;
        const uint8_t mode = static_cast<uint8_t>((userFreq ? 0x01 : 0) | (pit ? 0x02 : 0) | 0x10);
        const uint8_t power = saVersion == 1 ? V1[powerIndex & 3] : powerIndex;
        std::vector<uint8_t> d = { channel, power, mode, static_cast<uint8_t>(freq >> 8), static_cast<uint8_t>(freq) };
        if (saVersion == 21)
        {
            d.push_back(dbm);
            d.push_back(static_cast<uint8_t>(dbmLevels.size()));
            d.insert(d.end(), dbmLevels.begin(), dbmLevels.end());
        }
        saReply(code, d);
    }

    void smartAudio(const uint8_t* d, size_t n)
    {
        // полётник шлёт 0x00 перед кадром
        size_t at = 0;
        while (at < n && d[at] == 0x00) ++at;
        d += at;
        n -= at;
        if (n < 5 || d[0] != 0xAA || d[1] != 0x55) return;
        const uint8_t len = d[3];
        if (n != static_cast<size_t>(len) + 5 || copter::smartAudioCrc(d, static_cast<uint8_t>(n - 1)) != d[n - 1]) return;
        ++frames;
        const uint8_t* p = d + 4;
        switch (d[2] >> 1)
        {
            case 0x01: saSettings(); break;
            case 0x02:
                if (saVersion == 21)
                {
                    dbm = p[0] & 0x7F;
                }
                else if (saVersion == 1)
                {
                    static const uint8_t V1[4] = { 7, 16, 25, 40 };
                    for (uint8_t i = 0; i < 4; ++i)
                        if (V1[i] == p[0]) powerIndex = i;
                }
                else
                {
                    powerIndex = p[0];
                }
                saReply(0x02, { p[0], 0x01 });
                break;
            case 0x03:
                channel = p[0];
                userFreq = false;
                freq = copter::vtxFrequency(channel / 8 + 1, channel % 8 + 1);
                saReply(0x03, { p[0], 0x01 });
                break;
            case 0x04:
                freq = static_cast<uint16_t>((p[0] << 8) | p[1]);
                userFreq = true;
                saReply(0x04, { p[0], p[1], 0x01 });
                break;
            case 0x05:
                if (p[0] & 0x01) pit = true;
                if (p[0] & 0x04) pit = false;
                saReply(0x05, { p[0], 0x01 });
                break;
            default: break;
        }
    }

    void trampAnswer(char c, std::initializer_list<uint8_t> bytes)
    {
        uint8_t f[16] = {};
        f[0] = 0x0F;
        f[1] = static_cast<uint8_t>(c);
        uint8_t i = 2;
        for (uint8_t b : bytes) f[i++] = b;
        f[14] = copter::trampChecksum(f);
        out.insert(out.end(), f, f + 16);
    }

    void tramp(const uint8_t* d, size_t n)
    {
        if (n != 16 || d[0] != 0x0F || d[15] != 0 || copter::trampChecksum(d) != d[14]) return;
        ++frames;
        const uint16_t v = static_cast<uint16_t>(d[2] | (d[3] << 8));
        switch (d[1])
        {
            case 'r':
                trampAnswer('r', { 0x40, 0x13, 0xE8, 0x17, static_cast<uint8_t>(trampMax), static_cast<uint8_t>(trampMax >> 8) });
                break;
            case 'v':
                trampAnswer('v', { static_cast<uint8_t>(freq), static_cast<uint8_t>(freq >> 8), static_cast<uint8_t>(trampMw),
                                   static_cast<uint8_t>(trampMw >> 8), 0, static_cast<uint8_t>(pit ? 1 : 0),
                                   static_cast<uint8_t>(trampMw), static_cast<uint8_t>(trampMw >> 8) });
                break;
            case 'F': freq = v; break;
            case 'P': trampMw = v; break;
            case 'I': pit = v == 0; break;
            default: break;
        }
    }
};
