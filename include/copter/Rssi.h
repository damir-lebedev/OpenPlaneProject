#pragma once
#include <stdint.h>

// ============================================================
// RSSI — СИЛА СИГНАЛА ПУЛЬТА, % (113)
//
// Откуда берётся — как в BF, по порядку:
//   • rssi_channel > 0 — из канала пульта (приёмник кладёт RSSI в AUX):
//     1000…2000 мкс → 0…100 %, затем rssi_scale (%), rssi_offset (%),
//     rssi_invert (100 − значение);
//   • rssi_src_frame_errors = ON — по доле битых кадров за полсекунды
//     (для приёмников без RSSI: SBUS, iBUS);
//   • иначе — от протокола: CRSF сообщает уровень сигнала в dBm,
//     −130…0 dBm → 0…100 % (так считает BF).
// Нет источника — −1 («неизвестно»): не показывается и не тревожит.
// Связи нет — 0 %.
// LQ (качество связи, доля дошедших пакетов) — отдельно, его даёт CRSF.
// ============================================================

namespace copter
{
    struct RssiConfig
    {
        uint8_t channel = 0;        // rssi_channel: 1… — номер канала, 0 — нет
        uint8_t scale = 100;        // rssi_scale, %
        int8_t offset = 0;          // rssi_offset, %
        bool invert = false;        // rssi_invert
        bool frameErrors = false;   // rssi_src_frame_errors
    };

    enum class RssiSource : uint8_t
    {
        NONE,
        CHANNEL,
        FRAME_ERRORS,
        PROTOCOL,
    };

    inline const char* rssiSourceName(RssiSource s)
    {
        switch (s)
        {
            case RssiSource::CHANNEL: return "channel";
            case RssiSource::FRAME_ERRORS: return "frames";
            case RssiSource::PROTOCOL: return "protocol";
            default: return "none";
        }
    }

    class Rssi
    {
    public:
        static constexpr uint32_t WINDOW_MS = 500;   // окно подсчёта битых кадров
        static constexpr int16_t DBM_MIN = -130;     // CRSF: 0 %

        void configure(const RssiConfig& c)
        {
            if (c.frameErrors != config.frameErrors) windowStartMs = 0;
            config = c;
        }

        const RssiConfig& getConfig() const { return config; }

        // channels — каналы в мкс (count штук); dbm — уровень от протокола
        // (0 — протокол не сообщает); good/bad — счётчики кадров приёмника.
        void update(const uint16_t* channels, uint8_t count, int16_t dbm, uint32_t good, uint32_t bad, bool linkOk,
                    uint32_t nowMs)
        {
            src = pickSource(count, dbm);
            switch (src)
            {
                case RssiSource::CHANNEL:
                    value = linkOk ? fromChannel(channels[config.channel - 1]) : 0;
                    break;
                case RssiSource::FRAME_ERRORS:
                    updateFrames(good, bad, linkOk, nowMs);
                    break;
                case RssiSource::PROTOCOL:
                    value = linkOk ? fromDbm(dbm) : 0;
                    break;
                default:
                    value = -1;
                    break;
            }
        }

        // %, −1 — неизвестно.
        int16_t percent() const { return value; }
        RssiSource source() const { return src; }

        // 1000…2000 мкс → 0…100 % с rssi_scale/offset/invert.
        int16_t fromChannel(uint16_t us) const
        {
            int32_t raw = us <= 1000 ? 0 : us >= 2000 ? 100 : (static_cast<int32_t>(us) - 1000) / 10;
            raw = raw * config.scale / 100 + config.offset;
            raw = clamp(raw);
            return static_cast<int16_t>(config.invert ? 100 - raw : raw);
        }

        static int16_t fromDbm(int16_t dbm)
        {
            if (dbm >= 0) return 100;
            return static_cast<int16_t>(clamp((static_cast<int32_t>(dbm) - DBM_MIN) * 100 / -DBM_MIN));
        }

    private:
        RssiConfig config;
        RssiSource src = RssiSource::NONE;
        int16_t value = -1;
        uint32_t windowStartMs = 0;
        uint32_t windowGood = 0, windowBad = 0;

        static int32_t clamp(int32_t v) { return v < 0 ? 0 : v > 100 ? 100 : v; }

        RssiSource pickSource(uint8_t count, int16_t dbm) const
        {
            if (config.channel > 0 && config.channel <= count) return RssiSource::CHANNEL;
            if (config.frameErrors) return RssiSource::FRAME_ERRORS;
            if (dbm < 0) return RssiSource::PROTOCOL;
            return RssiSource::NONE;
        }

        void updateFrames(uint32_t good, uint32_t bad, bool linkOk, uint32_t nowMs)
        {
            if (windowStartMs == 0)
            {
                windowStartMs = nowMs ? nowMs : 1;
                windowGood = good;
                windowBad = bad;
                if (value < 0) value = linkOk ? 100 : 0;
                return;
            }
            if (!linkOk) value = 0;
            if (nowMs - windowStartMs < WINDOW_MS) return;
            const uint32_t g = good - windowGood, b = bad - windowBad;
            value = !linkOk || g + b == 0 ? 0 : static_cast<int16_t>(g * 100 / (g + b));
            windowStartMs = nowMs ? nowMs : 1;
            windowGood = good;
            windowBad = bad;
        }
    };
}
