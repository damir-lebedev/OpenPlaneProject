#pragma once
#include <stdint.h>

// ============================================================
// Протоколы регуляторов (ESC) мультикоптера.
//
// Аналоговые — импульс заданной ширины, чем шире, тем больше газ:
//   PWM        1000–2000 мкс, 490 Гц (любой ESC, в т. ч. самолётный)
//   ONESHOT125  125–250 мкс   — BLHeli_S/32, AM32, KISS
//   ONESHOT42    42–84 мкс
//   MULTISHOT     5–25 мкс
// Импульс идёт каждый цикл стабилизации (OneShot и быстрее) или с
// частотой PWM; 0 % газа — минимальная ширина, мотор стоит.
//
// Цифровой DShot — 16 бит на кадр: 11 бит значения (0 — стоп, 1–47 —
// команды, 48–2047 — газ), бит запроса телеметрии и 4 бита CRC.
// Бит — импульс доли периода: «1» — 3/4, «0» — 3/8. Скорость: DShot150,
// 300, 600 (кбит/с). Двунаправленный DShot инвертирует CRC, а
// регулятор отвечает оборотами по тому же проводу.
//
// Здесь — расчёт импульсов и кадров; железо (таймер, DMA) — в hal/.
// ============================================================

namespace copter
{
    enum class EscProtocol : uint8_t
    {
        PWM,
        ONESHOT125,
        ONESHOT42,
        MULTISHOT,
        DSHOT150,
        DSHOT300,
        DSHOT600,
    };

    inline const char* escProtocolName(EscProtocol p)
    {
        switch (p)
        {
            case EscProtocol::PWM: return "PWM";
            case EscProtocol::ONESHOT125: return "OneShot125";
            case EscProtocol::ONESHOT42: return "OneShot42";
            case EscProtocol::MULTISHOT: return "Multishot";
            case EscProtocol::DSHOT150: return "DShot150";
            case EscProtocol::DSHOT300: return "DShot300";
            case EscProtocol::DSHOT600: return "DShot600";
        }
        return "?";
    }

    inline bool isDshot(EscProtocol p)
    {
        return p == EscProtocol::DSHOT150 || p == EscProtocol::DSHOT300 || p == EscProtocol::DSHOT600;
    }

    // Ширина импульса аналоговых протоколов, мкс: стоп и полный газ.
    struct PulseRange
    {
        float stopUs;
        float fullUs;
        float maxRateHz;   // чаще импульсы не повторять
    };

    inline PulseRange pulseRange(EscProtocol p)
    {
        switch (p)
        {
            case EscProtocol::PWM: return { 1000.0f, 2000.0f, 490.0f };
            case EscProtocol::ONESHOT125: return { 125.0f, 250.0f, 3900.0f };
            case EscProtocol::ONESHOT42: return { 41.667f, 83.333f, 11000.0f };
            case EscProtocol::MULTISHOT: return { 5.0f, 25.0f, 32000.0f };
            default: return { 0.0f, 0.0f, 0.0f };
        }
    }

    // Команда 0..1 -> ширина импульса, мкс.
    inline float pulseUs(EscProtocol p, float command)
    {
        const PulseRange r = pulseRange(p);
        if (command < 0.0f) command = 0.0f;
        if (command > 1.0f) command = 1.0f;
        return r.stopUs + (r.fullUs - r.stopUs) * command;
    }

    // Обороты от регуляторов (двунаправленный DShot) — для консоли и
    // приложения; реализация — hal/stm32/Stm32Dshot.h.
    class IMotorTelemetry
    {
    public:
        virtual ~IMotorTelemetry() = default;
        virtual bool telemetryEnabled() const = 0;
        virtual uint8_t telemetryMotors() const = 0;
        virtual bool hasTelemetry(uint8_t i) const = 0;
        virtual uint32_t erpm(uint8_t i) const = 0;
        virtual float telemetryErrorPercent(uint8_t i) const = 0;
    };

    // ---------------- DShot ----------------

    namespace dshot
    {
        constexpr uint16_t STOP = 0;
        constexpr uint16_t MIN_THROTTLE = 48;
        constexpr uint16_t MAX_THROTTLE = 2047;

        // Команды (значения 1–47, слать при остановленном моторе, обычно 6–10 раз подряд).
        enum Command : uint16_t
        {
            BEEP1 = 1,
            ESC_INFO = 6,
            SPIN_DIRECTION_1 = 7,        // обычное направление
            SPIN_DIRECTION_2 = 8,        // обратное
            MODE_3D_OFF = 9,
            MODE_3D_ON = 10,
            SAVE_SETTINGS = 12,
            EXTENDED_TELEMETRY_ON = 13,
            SPIN_DIRECTION_NORMAL = 20,
            SPIN_DIRECTION_REVERSED = 21,
            LED0_ON = 22,
        };

        // Команда 0..1 -> значение кадра (0 — мотор стоит).
        inline uint16_t throttleValue(float command, bool running)
        {
            if (!running) return STOP;
            if (command < 0.0f) command = 0.0f;
            if (command > 1.0f) command = 1.0f;
            return static_cast<uint16_t>(MIN_THROTTLE + command * (MAX_THROTTLE - MIN_THROTTLE) + 0.5f);
        }

        // 16-битный кадр: значение, бит телеметрии, CRC (инверсная у двунаправленного).
        inline uint16_t frame(uint16_t value, bool telemetry, bool bidirectional)
        {
            const uint16_t packet = static_cast<uint16_t>((value & 0x07FF) << 1) | (telemetry ? 1 : 0);
            uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
            if (bidirectional) crc = (~crc) & 0x0F;
            return static_cast<uint16_t>((packet << 4) | crc);
        }

        inline bool frameValid(uint16_t f, bool bidirectional)
        {
            return frame(static_cast<uint16_t>(f >> 5), (f >> 4) & 1, bidirectional) == f;
        }

        inline uint32_t bitRate(EscProtocol p)
        {
            switch (p)
            {
                case EscProtocol::DSHOT150: return 150000;
                case EscProtocol::DSHOT300: return 300000;
                case EscProtocol::DSHOT600: return 600000;
                default: return 0;
            }
        }

        // Длительности бита и импульсов «1»/«0» в тиках таймера.
        struct BitTicks
        {
            uint32_t period;
            uint32_t one;
            uint32_t zero;
        };

        inline BitTicks bitTicks(EscProtocol p, uint32_t timerHz)
        {
            const uint32_t rate = bitRate(p);
            if (rate == 0) return { 0, 0, 0 };
            const uint32_t period = (timerHz + rate / 2) / rate;
            return { period, (period * 3 + 2) / 4, (period * 3 + 4) / 8 };
        }

        // Кадр -> 16 длительностей импульсов (старший бит первым) + 0 в конце (пауза).
        inline void frameToPulses(uint16_t f, const BitTicks& t, uint32_t* out17)
        {
            for (int i = 0; i < 16; ++i) out17[i] = (f & (0x8000 >> i)) ? t.one : t.zero;
            out17[16] = 0;
        }

        // ---- буфер burst DMA (hal/stm32/Stm32Dshot.h) ----
        //
        // Один таймер ведёт до 4 моторов (каналы CH1..CH4). По каждому событию
        // обновления таймера (раз в бит) DMA в режиме burst пишет подряд CCRn…
        // CCRn+k−1 — ширины импульсов текущего бита всех каналов. Буфер —
        // [слот бита][канал]: 16 бит кадра и 2 слота нулей (линия внизу, пока
        // следующий кадр; CCR с предзагрузкой — новое значение работает со
        // следующего бита, последний ноль гасит линию после кадра).
        constexpr uint8_t FRAME_SLOTS = 18;
        constexpr uint8_t BURST_MAX = 4;

        // frames[k] — кадр канала firstChannel + k (used[k] = false — канал
        // не мотор: всегда 0, линия внизу).
        inline void burstBuffer(const uint16_t* frames, const bool* used, uint8_t burst, const BitTicks& t, uint32_t* out)
        {
            for (uint8_t slot = 0; slot < FRAME_SLOTS; ++slot)
            {
                for (uint8_t k = 0; k < burst; ++k)
                {
                    uint32_t w = 0;
                    if (slot < 16 && used[k]) w = (frames[k] & (0x8000 >> slot)) ? t.one : t.zero;
                    out[slot * burst + k] = w;
                }
            }
        }

        // Обратно: канал k буфера -> кадр (проверка в тестах). false — не DShot
        // (ширина не «1» и не «0» с допуском в такт, или нет паузы в конце).
        inline bool decodeBurst(const uint32_t* buffer, uint8_t burst, uint8_t k, const BitTicks& t, uint16_t& frame)
        {
            uint16_t f = 0;
            for (uint8_t slot = 0; slot < 16; ++slot)
            {
                const uint32_t w = buffer[slot * burst + k];
                const uint32_t dOne = w > t.one ? w - t.one : t.one - w;
                const uint32_t dZero = w > t.zero ? w - t.zero : t.zero - w;
                if (dOne <= 1) f = static_cast<uint16_t>(f | (0x8000 >> slot));
                else if (dZero > 1) return false;
            }
            for (uint8_t slot = 16; slot < FRAME_SLOTS; ++slot)
            {
                if (buffer[slot * burst + k] != 0) return false;
            }
            frame = f;
            return true;
        }

        // Длительность кадра (16 бит), мкс.
        inline float frameUs(EscProtocol p)
        {
            const uint32_t rate = bitRate(p);
            return rate ? 16.0e6f / static_cast<float>(rate) : 0.0f;
        }

        // ---- двунаправленный DShot: ответ регулятора (dshot_bidir) ----
        //
        // С dshot_bidir линия инвертирована (покой — высокий уровень, CRC
        // кадра инвертирован), и через ~30 мкс после кадра регулятор сам
        // отвечает по тому же проводу. Ответ — 16 бит: 12 бит периода
        // электрического оборота (3 бита сдвига e и 9 бит m: период = m << e
        // мкс; 0xFFF — мотор стоит) и 4 бита CRC (инверсный xor полубайтов).
        // 16 бит -> GCR 4->5 = 20 бит -> NRZI (единица — смена уровня) после
        // стартового бита (уровень вниз) = 21 бит, со скоростью 5/4 скорости
        // DShot. Смежные уровни xor-ом дают обратно GCR — неважно, какой
        // уровень у линии считать единицей.

        constexpr uint16_t TELEMETRY_STOPPED = 0x0FFF;
        constexpr uint8_t TELEMETRY_BITS = 21;

        inline uint8_t gcrEncode(uint8_t nibble)
        {
            static const uint8_t T[16] = { 0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
                                           0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F };
            return T[nibble & 0x0F];
        }

        // 5 бит GCR -> полубайт; 0xFF — такого кода нет.
        inline uint8_t gcrDecode(uint8_t code)
        {
            for (uint8_t n = 0; n < 16; ++n)
            {
                if (gcrEncode(n) == (code & 0x1F)) return n;
            }
            return 0xFF;
        }

        inline uint16_t telemetryCrcFrame(uint16_t value12)
        {
            const uint16_t v = value12 & 0x0FFF;
            const uint16_t crc = static_cast<uint16_t>(~(v ^ (v >> 4) ^ (v >> 8)) & 0x0F);
            return static_cast<uint16_t>((v << 4) | crc);
        }

        // Период оборота, мкс -> 12 бит (e, m); 0 и больше 65 мс — «стоит».
        inline uint16_t periodToTelemetry(uint32_t periodUs)
        {
            if (periodUs == 0 || periodUs > (0x1FFu << 7)) return TELEMETRY_STOPPED;
            uint8_t e = 0;
            while ((periodUs >> e) > 0x1FF) ++e;
            return static_cast<uint16_t>((e << 9) | (periodUs >> e));
        }

        // 12 бит -> электрические обороты в минуту (eRPM); стоит — 0.
        inline uint32_t telemetryToErpm(uint16_t value12)
        {
            if ((value12 & 0x0FFF) == TELEMETRY_STOPPED) return 0;
            const uint32_t period = static_cast<uint32_t>(value12 & 0x1FF) << ((value12 >> 9) & 0x07);
            return period ? (60000000u + period / 2) / period : 0;
        }

        // Механические обороты в минуту: motor_poles — магнитов ротора (14 у 2207).
        inline float erpmToRpm(uint32_t erpm, uint8_t poles)
        {
            return poles >= 2 ? static_cast<float>(erpm) * 2.0f / poles : 0.0f;
        }

        // Ответ -> уровни линии (21 бит, старший — стартовый, 0 = низкий).
        inline uint32_t telemetryLine(uint16_t value12)
        {
            const uint16_t f = telemetryCrcFrame(value12);
            uint32_t gcr = 0;
            for (int n = 3; n >= 0; --n) gcr = (gcr << 5) | gcrEncode(static_cast<uint8_t>(f >> (4 * n)));
            uint32_t line = 0, level = 0;   // стартовый бит — вниз
            for (int b = 19; b >= 0; --b)
            {
                level ^= (gcr >> b) & 1u;
                line = (line << 1) | level;
            }
            return line;   // 21 бит: стартовый 0 + 20 уровней
        }

        // Уровни линии -> 12 бит. false — код не GCR или CRC не сошлась.
        inline bool decodeTelemetryLine(uint32_t line21, uint16_t& value12)
        {
            const uint32_t gcr = (line21 ^ (line21 >> 1)) & 0xFFFFFu;
            uint16_t f = 0;
            for (int n = 3; n >= 0; --n)
            {
                const uint8_t nib = gcrDecode(static_cast<uint8_t>(gcr >> (5 * n)));
                if (nib == 0xFF) return false;
                f = static_cast<uint16_t>((f << 4) | nib);
            }
            uint16_t x = static_cast<uint16_t>(f ^ (f >> 8));
            x = static_cast<uint16_t>(x ^ (x >> 4));
            if ((x & 0x0F) != 0x0F) return false;
            value12 = static_cast<uint16_t>(f >> 4);
            return true;
        }

        // Отсчёты уровня линии (1 — высокий) с частотой samplesPerBit × скорость
        // ответа -> 21 бит уровней: от первого спада (стартовый бит) длины
        // участков одного уровня — в битах с округлением; хвост после
        // последнего фронта — высокий уровень покоя. false — спада нет или
        // бит не 21.
        inline bool samplesToLine(const uint8_t* samples, uint16_t count, float samplesPerBit, uint32_t& line21)
        {
            if (samplesPerBit < 1.5f) return false;
            uint16_t i = 1;
            while (i < count && !(samples[i - 1] && !samples[i])) ++i;   // спад: старт
            if (i >= count) return false;
            uint32_t line = 0;
            uint8_t bits = 0;
            uint32_t level = 0;
            uint16_t runStart = i;
            for (uint16_t k = static_cast<uint16_t>(i + 1); k < count && bits < TELEMETRY_BITS; ++k)
            {
                if ((samples[k] ? 1u : 0u) == level) continue;
                const uint32_t n = static_cast<uint32_t>((k - runStart) / samplesPerBit + 0.5f);
                if (n == 0) return false;   // иголка короче полбита — шум
                for (uint32_t b = 0; b < n && bits < TELEMETRY_BITS; ++b, ++bits) line = (line << 1) | level;
                level ^= 1u;
                runStart = k;
            }
            if (bits < TELEMETRY_BITS)
            {
                // последний участок до конца окна: высокий — это покой после ответа
                if (!level) return false;   // оборвалось на низком уровне
                while (bits < TELEMETRY_BITS)
                {
                    line = (line << 1) | 1u;
                    ++bits;
                }
            }
            line21 = line;
            return true;
        }

        // Скорость ответа регулятора: 5/4 скорости DShot.
        inline uint32_t telemetryBitRate(EscProtocol p) { return bitRate(p) * 5 / 4; }

        // Чаще цикл не поднимать: кадр и пауза не короче кадра (регулятору
        // нужно время разобрать кадр; у BF так же: DShot150 — до 4 кГц,
        // DShot300 — до 8, DShot600 — до 16). 0 — не DShot.
        // С dshot_bidir — ещё пауза ~30 мкс до ответа и сам ответ (21 бит на
        // 5/4 скорости): DShot600 — до 11.8 кГц, 300 — 7.2, 150 — 4.0.
        constexpr float TELEMETRY_GAP_US = 30.0f;
        inline uint32_t maxLoopHz(EscProtocol p, bool bidirectional = false)
        {
            const float us = frameUs(p);
            if (us <= 0.0f) return 0;
            if (!bidirectional) return static_cast<uint32_t>(1.0e6f / (2.0f * us));
            const float reply = TELEMETRY_BITS * 1.0e6f / static_cast<float>(telemetryBitRate(p));
            return static_cast<uint32_t>(1.0e6f / (us + TELEMETRY_GAP_US + reply));
        }
    }
}
