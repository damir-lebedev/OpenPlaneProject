#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

// ============================================================
// Видеопередатчик (аналоговый VTX): частота, мощность, PIT — по проводу
// управления, как vtx_* в Betaflight.
//
// Протоколы (по их открытым описаниям; код Betaflight не копируется):
//   SmartAudio (TBS) — 4800 бод, 8N2, один провод. Кадр:
//     [0x00] 0xAA 0x55 (команда<<1)|1 длина данные… CRC8 (полином 0xD5);
//     ответ: 0xAA 0x55 команда длина данные… CRC8. Версия — по ответу на
//     «настройки» (0x01 — v1, 0x09 — v2, 0x11 — v2.1); у v2.1 в ответе и
//     список мощностей в дБм — таблица мощностей берётся у самого VTX.
//   Tramp (ImmersionRC) — 9600 бод, 8N1, один провод. Кадр 16 байт:
//     0x0F команда данные (12 байт) сумма 0x00; мощность — в мВт.
// Какой протокол у передатчика — пробуем оба (vtx_protocol = AUTO).
//
// Мощность: уровень vtx_power (1 — самый слабый) или с тумблера (таблица
// `vtx` в CLI, как в BF), потолок vtx_max_power_mw — «в городе больше
// нельзя»: никакой уровень выше не уйдёт, а если передатчик сам (кнопкой)
// оказался выше, прошивка его опустит, как только увидит. Без ARM —
// слабее (vtx_low_power_disarm), режим VTX PIT MODE — PIT.
//
// Частоты — таблица этого передатчика (80 каналов 4.9–6.1 ГГц): A, B, E,
// F, R, L (у продавца — D), X, V, G, S. Шлем Skyzone Cobra X V4 принимает
// только A, B, E, F, R и L.
//
// Без Arduino: порт — интерфейс VtxPort, тест — эмуляторы передатчиков
// (test_copter).
// ============================================================

namespace copter
{
    // ---------------- таблица частот ----------------

    constexpr uint8_t VTX_BANDS = 10;
    constexpr uint8_t VTX_CHANNELS = 8;

    inline const char* vtxBandLetters() { return "ABEFRLXVGS"; }

    inline const char* vtxBandName(uint8_t band)
    {
        static const char* const names[VTX_BANDS] = { "A",  "B",  "E",  "F",  "R (RACEBAND)",
                                                      "L (LOWBAND)", "X", "V", "G", "S" };
        return band >= 1 && band <= VTX_BANDS ? names[band - 1] : "USER";
    }

    // МГц; band 1..10, channel 1..8; 0 — нет такого.
    inline uint16_t vtxFrequency(uint8_t band, uint8_t channel)
    {
        static const uint16_t table[VTX_BANDS][VTX_CHANNELS] = {
            { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 },   // A
            { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 },   // B
            { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 },   // E
            { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 },   // F
            { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 },   // R
            { 5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621 },   // L (D)
            { 4990, 5020, 5050, 5080, 5110, 5140, 5170, 5200 },   // X
            { 4867, 4884, 4921, 4958, 4995, 5032, 5069, 5099 },   // V
            { 5960, 5980, 6000, 6020, 6040, 6060, 6080, 6100 },   // G
            { 5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613 },   // S
        };
        if (band < 1 || band > VTX_BANDS || channel < 1 || channel > VTX_CHANNELS) return 0;
        return table[band - 1][channel - 1];
    }

    // Найти канал по частоте (для показа). false — частоты нет в таблице.
    inline bool vtxBandChannel(uint16_t mhz, uint8_t& band, uint8_t& channel)
    {
        for (uint8_t b = 1; b <= VTX_BANDS; ++b)
        {
            for (uint8_t c = 1; c <= VTX_CHANNELS; ++c)
            {
                if (vtxFrequency(b, c) == mhz)
                {
                    band = b;
                    channel = c;
                    return true;
                }
            }
        }
        return false;
    }

    // Шлем Cobra X V4 (и большинство аналоговых шлемов) — только 5.8 ГГц A, B, E, F, R, L.
    inline bool vtxBandCommonGoggles(uint8_t band) { return band >= 1 && band <= 6; }

    // дБм -> мВт, как их называют передатчики: 36 дБм — «4 Вт» (а не 3981 мВт).
    inline uint16_t dbmToMw(float dbm)
    {
        static const uint16_t nice[] = { 25,  50,  100, 200,  250,  300,  400,  500,  600,  800,  1000,
                                         1200, 1600, 2000, 2500, 3000, 4000, 5000, 6000, 8000, 10000 };
        const float mw = powf(10.0f, dbm / 10.0f);
        for (uint16_t n : nice)
        {
            if (fabsf(mw - n) <= 0.06f * n) return n;
        }
        return static_cast<uint16_t>(lroundf(mw));
    }

    // ---------------- контрольные суммы ----------------

    inline uint8_t smartAudioCrc(const uint8_t* data, uint8_t length)
    {
        uint8_t crc = 0;
        for (uint8_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (uint8_t b = 0; b < 8; ++b) crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
        }
        return crc;
    }

    inline uint8_t trampChecksum(const uint8_t* frame)   // байты 1..13
    {
        uint8_t sum = 0;
        for (uint8_t i = 1; i < 14; ++i) sum = static_cast<uint8_t>(sum + frame[i]);
        return sum;
    }

    // ---------------- порт ----------------

    class VtxPort
    {
    public:
        virtual ~VtxPort() = default;
        // Скорость и стоп-биты; половина дуплекса — один провод.
        virtual void open(uint32_t baud, bool twoStopBits) = 0;
        virtual void send(const uint8_t* data, uint8_t length) = 0;   // после отправки — сразу приём
        virtual int read() = 0;                                       // −1 — нет байта
    };

    enum class VtxProtocol : uint8_t
    {
        AUTO,
        SMARTAUDIO,
        TRAMP,
        NONE,
    };

    inline const char* vtxProtocolName(VtxProtocol p)
    {
        switch (p)
        {
            case VtxProtocol::SMARTAUDIO: return "SmartAudio";
            case VtxProtocol::TRAMP: return "Tramp";
            case VtxProtocol::NONE: return "нет";
            case VtxProtocol::AUTO:
            default: return "AUTO";
        }
    }

    // Что хотим от передатчика.
    struct VtxRequest
    {
        uint16_t frequencyMhz = 0;
        uint8_t band = 0, channel = 0;   // 0 — частота задана числом
        uint8_t powerLevel = 1;          // 1..N
        bool pit = false;
        bool control = true;             // false — режим VTX CONTROL DISABLE: не трогать
    };

    // Что передатчик сказал о себе.
    struct VtxState
    {
        bool online = false;
        VtxProtocol protocol = VtxProtocol::NONE;
        uint8_t version = 0;             // SmartAudio: 1, 2, 21
        uint16_t frequencyMhz = 0;
        uint16_t powerMw = 0;
        bool pit = false;
        bool unlocked = true;
        float temperatureC = NAN;        // Tramp
    };

    // ---------------- контроллер ----------------

    class VtxController
    {
    public:
        static constexpr uint8_t MAX_LEVELS = 12;
        static constexpr uint32_t RESPONSE_MS = 150;    // ответа нет дольше — команда потеряна
        static constexpr uint32_t POLL_MS = 1000;       // перечитать настройки
        static constexpr uint8_t PROBES = 3;            // попыток на протокол, потом — следующий
        static constexpr uint32_t SETTLE_MS = 100;      // после установки — перечитать не раньше

        explicit VtxController(VtxPort& p) : port(p) {}

        void setProtocol(VtxProtocol p)
        {
            wanted = p;
            restart();
        }

        // Потолок мощности, мВт; 0 — без потолка.
        void setMaxPowerMw(uint16_t mw) { maxPowerMw = mw; }

        void request(const VtxRequest& r) { desired = r; }

        const VtxState& state() const { return st; }
        uint8_t levelCount() const { return levelTotal; }
        uint16_t levelMw(uint8_t level) const { return level >= 1 && level <= levelTotal ? levels[level - 1] : 0; }
        uint8_t topAllowedLevel() const
        {
            uint8_t top = 1;
            for (uint8_t l = 1; l <= levelTotal; ++l)
            {
                if (maxPowerMw == 0 || levels[l - 1] <= maxPowerMw) top = l;
            }
            return top;
        }
        // Уровень, который будет послан: запрошенный, но не выше потолка.
        uint8_t effectiveLevel() const
        {
            uint8_t l = desired.powerLevel ? desired.powerLevel : 1;
            if (l > levelTotal) l = levelTotal ? levelTotal : 1;
            const uint8_t top = topAllowedLevel();
            return l > top ? top : l;
        }

        // Каждый цикл (или чаще 50 Гц): разбор ответов, следующая команда.
        void update(uint32_t nowMs)
        {
            if (wanted == VtxProtocol::NONE) return;
            receive();
            if (waiting)
            {
                if (static_cast<int32_t>(nowMs - sentMs) < static_cast<int32_t>(RESPONSE_MS)) return;
                waiting = false;   // ответа нет
                if (!st.online) nextProbe();
                else if (++misses >= 3) restart();   // пропал — искать заново
            }
            if (!st.online)
            {
                probe(nowMs);
                return;
            }
            // Не знаем, что сейчас у передатчика (Tramp после 'r'), или только что
            // что-то в нём меняли — сначала перечитать, потом решать дальше.
            if (!settingsKnown || stale)
            {
                if (static_cast<int32_t>(nowMs - sentMs) >= static_cast<int32_t>(SETTLE_MS)) askSettings(nowMs);
                return;
            }
            if (!desired.control) return;
            if (!sendCorrection(nowMs) && static_cast<int32_t>(nowMs - polledMs) >= static_cast<int32_t>(POLL_MS))
            {
                askSettings(nowMs);
            }
        }

    private:
        void restart()
        {
            st = VtxState();
            trying = wanted == VtxProtocol::TRAMP ? VtxProtocol::TRAMP : VtxProtocol::SMARTAUDIO;
            probesLeft = PROBES;
            opened = false;
            waiting = false;
            misses = 0;
            levelTotal = 0;
            rxLength = 0;
            settingsKnown = false;
            stale = false;
        }

        void nextProbe()
        {
            if (--probesLeft > 0) return;
            probesLeft = PROBES;
            if (wanted == VtxProtocol::AUTO)
            {
                trying = trying == VtxProtocol::SMARTAUDIO ? VtxProtocol::TRAMP : VtxProtocol::SMARTAUDIO;
                opened = false;
            }
        }

        void probe(uint32_t nowMs)
        {
            if (!opened)
            {
                if (trying == VtxProtocol::SMARTAUDIO) port.open(4800, true);
                else port.open(9600, false);
                opened = true;
            }
            if (trying == VtxProtocol::SMARTAUDIO) saSend(0x01, nullptr, 0, nowMs);
            else trampSend('r', 0, nowMs);
        }

        void askSettings(uint32_t nowMs)
        {
            polledMs = nowMs;
            if (st.protocol == VtxProtocol::SMARTAUDIO) saSend(0x01, nullptr, 0, nowMs);
            else trampSend('v', 0, nowMs);
        }

        // Одна команда, если передатчик не такой, как надо. true — послали.
        bool sendCorrection(uint32_t nowMs)
        {
            const uint16_t mhz = desired.frequencyMhz;
            const uint8_t level = effectiveLevel();
            const uint16_t mw = levelMw(level);
            if (st.protocol == VtxProtocol::SMARTAUDIO)
            {
                if (st.version >= 2 && st.pit != desired.pit)
                {
                    // режим: PIT в зоне (бит 0) или выход из PIT (бит 2); бит 3 — не блокировать
                    const uint8_t mode = desired.pit ? 0x01 | 0x08 : 0x04 | 0x08;
                    saSend(0x05, &mode, 1, nowMs);
                    return true;
                }
                if (mhz && st.frequencyMhz != mhz)
                {
                    const int ch = smartAudioChannel(desired.band, desired.channel);
                    if (ch >= 0 && vtxFrequency(desired.band, desired.channel) == mhz)
                    {
                        const uint8_t c = static_cast<uint8_t>(ch);
                        saSend(0x03, &c, 1, nowMs);
                    }
                    else
                    {
                        const uint8_t f[2] = { static_cast<uint8_t>(mhz >> 8), static_cast<uint8_t>(mhz) };
                        saSend(0x04, f, 2, nowMs);
                    }
                    return true;
                }
                if (mw && st.powerMw != mw)
                {
                    uint8_t value;
                    if (st.version == 21) value = static_cast<uint8_t>(dbm[level - 1] | 0x80);
                    else if (st.version == 2) value = static_cast<uint8_t>(level - 1);
                    else value = V1_POWER_VALUES[level - 1 < 4 ? level - 1 : 3];
                    saSend(0x02, &value, 1, nowMs);
                    return true;
                }
                return false;
            }
            // Tramp
            if (st.pit != desired.pit)
            {
                trampSend('I', desired.pit ? 0 : 1, nowMs);
                return true;
            }
            if (mhz && st.frequencyMhz != mhz)
            {
                trampSend('F', mhz, nowMs);
                return true;
            }
            if (mw && st.powerMw != mw)
            {
                trampSend('P', mw, nowMs);
                return true;
            }
            return false;
        }

        // SmartAudio: номер канала 0..39 для A, B, E, F, R; −1 — нет.
        static int smartAudioChannel(uint8_t band, uint8_t channel)
        {
            if (band < 1 || band > 5 || channel < 1 || channel > 8) return -1;
            return (band - 1) * 8 + (channel - 1);
        }

        // ---------------- SmartAudio ----------------

        void saSend(uint8_t command, const uint8_t* data, uint8_t length, uint32_t nowMs)
        {
            uint8_t frame[16];
            uint8_t n = 0;
            frame[n++] = 0x00;   // разбудить линию
            frame[n++] = 0xAA;
            frame[n++] = 0x55;
            frame[n++] = static_cast<uint8_t>((command << 1) | 1);
            frame[n++] = length;
            for (uint8_t i = 0; i < length; ++i) frame[n++] = data[i];
            frame[n] = smartAudioCrc(frame + 1, static_cast<uint8_t>(n - 1));
            ++n;
            port.send(frame, n);
            sentCommand = command;
            sentMs = nowMs;
            waiting = true;
            if (command != 0x01) stale = true;
        }

        void saFrame(const uint8_t* f, uint8_t length)
        {
            // f: 0xAA 0x55 команда длина данные… CRC
            const uint8_t cmd = f[2];
            const uint8_t* d = f + 4;
            const uint8_t len = f[3];
            if (cmd == 0x01 || cmd == 0x09 || cmd == 0x11)
            {
                if (len < 5) return;
                st.protocol = VtxProtocol::SMARTAUDIO;
                st.version = cmd == 0x01 ? 1 : cmd == 0x09 ? 2 : 21;
                const uint8_t channel = d[0];
                const uint8_t mode = d[2];
                const uint16_t freq = static_cast<uint16_t>((d[3] << 8) | d[4]);
                st.frequencyMhz = (mode & 0x01) ? freq : vtxFrequency(channel / 8 + 1, channel % 8 + 1);
                st.pit = (mode & 0x02) != 0;
                st.unlocked = (mode & 0x10) != 0;
                if (st.version == 21 && len >= 7)
                {
                    // текущая мощность в дБм, число уровней, уровни (у разных VTX —
                    // n или n+1 значений: берём, сколько пришло)
                    const uint8_t count = static_cast<uint8_t>(len - 7);
                    levelTotal = 0;
                    for (uint8_t i = 0; i < count && levelTotal < MAX_LEVELS; ++i)
                    {
                        const uint8_t v = d[7 + i];
                        if (levelTotal && v <= dbm[levelTotal - 1]) continue;   // по возрастанию
                        dbm[levelTotal] = v;
                        levels[levelTotal++] = dbmToMw(v);
                    }
                    st.powerMw = dbmToMw(d[5]);
                }
                else
                {
                    // v1/v2: четыре ступени, как у TBS Unify: 25, 200, 500, 800 мВт
                    levelTotal = 4;
                    const uint16_t mw[4] = { 25, 200, 500, 800 };
                    for (uint8_t i = 0; i < 4; ++i) levels[i] = mw[i];
                    uint8_t index = d[1];
                    if (st.version == 1)
                    {
                        index = 0;
                        for (uint8_t i = 0; i < 4; ++i)
                        {
                            if (d[1] >= V1_POWER_VALUES[i]) index = i;
                        }
                    }
                    st.powerMw = levels[index < 4 ? index : 3];
                }
                if (levelTotal == 0)
                {
                    levelTotal = 1;
                    levels[0] = st.powerMw;
                }
                settingsKnown = true;
                stale = false;
                online();
                return;
            }
            // ответы на установку (значение + резерв): передатчик принял — перечитать
            // настройки. Короче двух байт — это эхо нашего же запроса на линии.
            if (cmd >= 0x02 && cmd <= 0x05 && len >= 2) acknowledged();
            (void)length;
        }

        // ---------------- Tramp ----------------

        void trampSend(char command, uint16_t value, uint32_t nowMs)
        {
            uint8_t frame[16] = {};
            frame[0] = 0x0F;
            frame[1] = static_cast<uint8_t>(command);
            frame[2] = static_cast<uint8_t>(value);
            frame[3] = static_cast<uint8_t>(value >> 8);
            frame[14] = trampChecksum(frame);
            port.send(frame, 16);
            sentCommand = static_cast<uint8_t>(command);
            sentMs = nowMs;
            // на установку Tramp не отвечает — ответ ждём только на запросы
            waiting = command == 'r' || command == 'v' || command == 's';
            if (!waiting) stale = true;
        }

        void trampFrame(const uint8_t* f)
        {
            auto u16 = [f](uint8_t i) { return static_cast<uint16_t>(f[i] | (f[i + 1] << 8)); };
            switch (f[1])
            {
                case 'r':
                {
                    if (u16(2) == 0) break;   // эхо нашего запроса на линии: пределов в нём нет
                    st.protocol = VtxProtocol::TRAMP;
                    const uint16_t maxMw = u16(6);
                    const uint16_t steps[] = { 25, 100, 200, 400, 600, 800, 1000, 1600, 2000, 2500, 3000, 4000 };
                    levelTotal = 0;
                    for (uint16_t mw : steps)
                    {
                        if (maxMw && mw > maxMw) break;
                        levels[levelTotal++] = mw;
                    }
                    if (levelTotal == 0) levels[levelTotal++] = maxMw ? maxMw : 25;
                    online();
                    break;
                }
                case 'v':
                    if (u16(2) == 0) break;   // эхо
                    settingsKnown = true;
                    stale = false;
                    st.frequencyMhz = u16(2);
                    st.powerMw = u16(4);
                    st.pit = f[7] != 0;   // 1 — PIT (а команда 'I': 1 — передавать)
                    acknowledged();
                    break;
                case 's':
                    st.temperatureC = static_cast<int16_t>(u16(6));
                    acknowledged();
                    break;
                default: break;
            }
        }

        // ---------------- приём ----------------

        void receive()
        {
            for (int n = 0; n < 64; ++n)
            {
                const int b = port.read();
                if (b < 0) return;
                const uint8_t c = static_cast<uint8_t>(b);
                if (rxLength == 0)
                {
                    if (c == 0xAA || c == 0x0F) rx[rxLength++] = c;
                    continue;
                }
                rx[rxLength++] = c;
                if (rx[0] == 0xAA)
                {
                    if (rxLength == 2 && c != 0x55)
                    {
                        rxLength = 0;
                        continue;
                    }
                    if (rxLength >= 4 && rxLength == static_cast<uint8_t>(rx[3] + 5))
                    {
                        if (smartAudioCrc(rx, static_cast<uint8_t>(rxLength - 1)) == rx[rxLength - 1]) saFrame(rx, rxLength);
                        rxLength = 0;
                    }
                    else if (rxLength >= sizeof(rx))
                    {
                        rxLength = 0;
                    }
                }
                else if (rxLength == 16)
                {
                    if (rx[15] == 0 && trampChecksum(rx) == rx[14] && rx[1] >= 'a' && rx[1] <= 'z') trampFrame(rx);
                    rxLength = 0;
                }
            }
        }

        void online()
        {
            st.online = true;
            waiting = false;
            misses = 0;
        }

        void acknowledged()
        {
            waiting = false;
            misses = 0;
        }

        static constexpr uint8_t V1_POWER_VALUES[4] = { 7, 16, 25, 40 };

        VtxPort& port;
        VtxProtocol wanted = VtxProtocol::NONE;
        VtxProtocol trying = VtxProtocol::SMARTAUDIO;
        uint8_t probesLeft = PROBES;
        bool opened = false;
        bool waiting = false;
        bool settingsKnown = false;
        bool stale = false;   // что-то меняли и ещё не перечитали
        uint8_t misses = 0;
        uint8_t sentCommand = 0;
        uint32_t sentMs = 0, polledMs = 0;
        VtxRequest desired;
        VtxState st;
        uint16_t maxPowerMw = 0;
        uint16_t levels[MAX_LEVELS] = {};
        uint8_t dbm[MAX_LEVELS] = {};
        uint8_t levelTotal = 0;
        uint8_t rx[40] = {};
        uint8_t rxLength = 0;
    };
}
