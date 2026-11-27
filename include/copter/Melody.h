#pragma once
#include <ctype.h>
#include <stddef.h>
#include <stdint.h>

// ============================================================
// Мелодия по тумблеру (своё, в BF нет). Мелодия — строка RTTTL, как у
// рингтонов старых телефонов и в EdgeTX:
//
//   имя:d=4,o=5,b=120:8c,8e,8g,2c6,p,4a#.
//
// d — длительность по умолчанию (1, 2, 4, 8, 16, 32), o — октава (3..8),
// b — темп (четвертей в минуту). Нота: [длительность]буква[#][.][октава][.],
// буквы c d e f g a b (h = b), p — пауза, точка — в полтора раза длиннее.
// Имя и блок умолчаний можно не писать (тогда d=4, o=6, b=63 — как в RTTTL).
//
// Играет:
//   - пищалкой — пассивной (beeper_frequency > 0) нотами, активной — ритмом;
//   - моторами (beeper_melody_motors) — DShot-тонами BEACON1..5: высота
//     ноты раскладывается на пять тонов (грубо), не чаще раза в 0.28 с.
// Когда: режим MELODY (тумблер) — только без ARM и со связью, по кругу с
// паузой 1 с; «послушать» (приложение, CLI `melody`) — один раз; при
// включении (beeper_melody_startup) — один раз вместо писка SYSTEM_INIT.
// Тумблер ARM включили — мелодия замолкает; после тонов моторами ARM ждёт
// 1.2 с (регулятор, пока пищит, газ не слушает) — как у маяка DShot.
//
// Без Arduino: tick() каждые 10 мс; тесты — test_copter.
// ============================================================

namespace copter
{
    constexpr uint8_t MELODY_NOTES_MAX = 64;
    constexpr size_t MELODY_TEXT_MAX = 128;   // символов в beeper_melody

    struct MelodyNote
    {
        uint16_t hz = 0;   // 0 — пауза
        uint16_t ms = 0;
        uint8_t key = 0;   // номер MIDI (C4 = 60); у паузы 0
    };

    struct Melody
    {
        MelodyNote notes[MELODY_NOTES_MAX];
        uint8_t count = 0;

        uint32_t totalMs() const
        {
            uint32_t t = 0;
            for (uint8_t i = 0; i < count; ++i) t += notes[i].ms;
            return t;
        }
    };

    // Своя мелодия по умолчанию (пустая beeper_melody): короткий позывной.
    constexpr const char* MELODY_DEFAULT = "openplane:d=8,o=6,b=150:c,e,g,4c7,p,g,2c7";

    namespace melody
    {
        // Частоты октавы 8 (C8..B8), Гц; ниже — делением на 2.
        constexpr uint16_t OCTAVE8[12] = { 4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902 };

        inline uint16_t noteHz(uint8_t semitone, uint8_t octave)
        {
            if (semitone > 11 || octave < 1 || octave > 8) return 0;
            const uint8_t shift = static_cast<uint8_t>(8 - octave);
            return static_cast<uint16_t>((OCTAVE8[semitone] + (1u << shift) / 2) >> shift);
        }

        inline int letterSemitone(char c)
        {
            switch (tolower(static_cast<unsigned char>(c)))
            {
                case 'c': return 0;
                case 'd': return 2;
                case 'e': return 4;
                case 'f': return 5;
                case 'g': return 7;
                case 'a': return 9;
                case 'b':
                case 'h': return 11;
                default: return -1;
            }
        }

        inline bool validDuration(unsigned d) { return d == 1 || d == 2 || d == 4 || d == 8 || d == 16 || d == 32; }

        inline const char* skipSpaces(const char* p)
        {
            while (*p == ' ' || *p == '\t') ++p;
            return p;
        }

        inline unsigned readNumber(const char*& p, bool& any)
        {
            unsigned v = 0;
            any = false;
            while (isdigit(static_cast<unsigned char>(*p)) && v < 10000)
            {
                v = v * 10 + static_cast<unsigned>(*p - '0');
                any = true;
                ++p;
            }
            return v;
        }
    }

    // Ошибка разбора: что не так и с какого символа (0 — с начала).
    struct MelodyError
    {
        const char* text = nullptr;
        size_t at = 0;
    };

    // RTTTL -> ноты (out == nullptr — только проверить, без буфера нот).
    // false — строка не та (err — что и где).
    inline bool parseRtttlInto(const char* s, Melody* out, MelodyError* err)
    {
        using namespace melody;
        uint8_t count = 0;
        if (out) out->count = 0;
        auto fail = [&](const char* text, const char* where) {
            if (err)
            {
                err->text = text;
                err->at = where ? static_cast<size_t>(where - s) : 0;
            }
            if (out) out->count = 0;
            return false;
        };
        if (!s) return fail("пустая строка", nullptr);

        // Сколько двоеточий: 2 — имя:умолчания:ноты, 1 — умолчания:ноты, 0 — ноты.
        const char* colon1 = nullptr;
        const char* colon2 = nullptr;
        for (const char* p = s; *p; ++p)
        {
            if (*p != ':') continue;
            if (!colon1) colon1 = p;
            else if (!colon2) colon2 = p;
            else return fail("больше двух двоеточий", p);
        }
        const char* defaults = nullptr;
        const char* defaultsEnd = nullptr;
        const char* notes = s;
        if (colon2)
        {
            defaults = colon1 + 1;
            defaultsEnd = colon2;
            notes = colon2 + 1;
        }
        else if (colon1)
        {
            defaults = s;
            defaultsEnd = colon1;
            notes = colon1 + 1;
        }

        unsigned dur = 4, oct = 6, bpm = 63;
        if (defaults)
        {
            const char* p = defaults;
            while (p < defaultsEnd)
            {
                p = skipSpaces(p);
                if (p >= defaultsEnd) break;
                const char key = static_cast<char>(tolower(static_cast<unsigned char>(*p)));
                const char* at = p;
                ++p;
                p = skipSpaces(p);
                if (*p != '=') return fail("в умолчаниях нужно d=, o=, b=", at);
                ++p;
                p = skipSpaces(p);
                bool any = false;
                const unsigned v = readNumber(p, any);
                if (!any) return fail("после = нужно число", p);
                if (key == 'd')
                {
                    if (!validDuration(v)) return fail("d — 1, 2, 4, 8, 16 или 32", at);
                    dur = v;
                }
                else if (key == 'o')
                {
                    if (v < 3 || v > 8) return fail("o — октава 3..8", at);
                    oct = v;
                }
                else if (key == 'b')
                {
                    if (v < 10 || v > 900) return fail("b — темп 10..900", at);
                    bpm = v;
                }
                else return fail("в умолчаниях бывают только d, o, b", at);
                p = skipSpaces(p);
                if (p < defaultsEnd && *p != ',') return fail("умолчания — через запятую", p);
                if (p < defaultsEnd) ++p;
            }
        }

        const uint32_t wholeMs = 240000u / bpm;   // целая нота = 4 четверти
        const char* p = notes;
        while (true)
        {
            p = skipSpaces(p);
            if (!*p) break;
            const char* at = p;
            if (count >= MELODY_NOTES_MAX) return fail("больше 64 нот", at);
            bool any = false;
            unsigned d = readNumber(p, any);
            if (!any) d = dur;
            else if (!validDuration(d)) return fail("длительность — 1, 2, 4, 8, 16 или 32", at);
            const char letter = static_cast<char>(tolower(static_cast<unsigned char>(*p)));
            const bool pause = letter == 'p';
            int semitone = pause ? 0 : letterSemitone(letter);
            if (semitone < 0) return fail("нота — c d e f g a b или p", p);
            ++p;
            if (*p == '#')
            {
                if (pause) return fail("у паузы нет диеза", p);
                ++semitone;
                ++p;
            }
            bool dotted = false;
            if (*p == '.')
            {
                dotted = true;
                ++p;
            }
            unsigned o = readNumber(p, any);
            if (!any) o = oct;
            else if (o < 3 || o > 8) return fail("октава 3..8", at);
            if (*p == '.')
            {
                dotted = true;
                ++p;
            }
            p = skipSpaces(p);
            if (*p && *p != ',') return fail("ноты — через запятую", p);
            if (*p == ',') ++p;

            uint32_t ms = wholeMs / d;
            if (dotted) ms += ms / 2;
            MelodyNote n;
            n.ms = static_cast<uint16_t>(ms > 60000u ? 60000u : ms);
            if (!pause)
            {
                if (semitone > 11)   // b# — до следующей октавы
                {
                    semitone -= 12;
                    ++o;
                }
                if (o > 8) return fail("выше B8 не бывает", at);
                n.hz = noteHz(static_cast<uint8_t>(semitone), static_cast<uint8_t>(o));
                n.key = static_cast<uint8_t>(12 * (o + 1) + static_cast<unsigned>(semitone));
            }
            if (out) out->notes[count] = n;
            ++count;
        }
        if (count == 0) return fail("нет ни одной ноты", p);
        if (out) out->count = count;
        return true;
    }

    inline bool parseRtttl(const char* s, Melody& out, MelodyError* err = nullptr) { return parseRtttlInto(s, &out, err); }
    // Только проверить строку (настройка beeper_melody).
    inline bool checkRtttl(const char* s, MelodyError* err = nullptr) { return parseRtttlInto(s, nullptr, err); }

    // Нота -> тон маяка DShot 1..5: от самой низкой ноты мелодии до самой
    // высокой поровну (мелодия в одну высоту — тон 3). Паузе — 0.
    inline uint8_t melodyBeaconTone(const Melody& m, const MelodyNote& n)
    {
        if (n.hz == 0) return 0;
        uint8_t lo = 255, hi = 0;
        for (uint8_t i = 0; i < m.count; ++i)
        {
            if (m.notes[i].hz == 0) continue;
            if (m.notes[i].key < lo) lo = m.notes[i].key;
            if (m.notes[i].key > hi) hi = m.notes[i].key;
        }
        if (hi <= lo) return 3;
        return static_cast<uint8_t>(1 + ((n.key - lo) * 8 + (hi - lo)) / (2 * (hi - lo)));
    }

    // Что сейчас звучит.
    struct MelodyOut
    {
        bool active = false;     // мелодия играет (пищалка — её, а не событий)
        uint16_t hz = 0;         // 0 — тишина между нотами
        uint8_t dshotTone = 0;   // послать всем моторам BEACON1..5 сейчас (0 — нет)
    };

    class MelodyPlayer
    {
    public:
        static constexpr uint32_t TICK_MS = 10;
        static constexpr uint32_t REPEAT_PAUSE_MS = 1000;   // тумблер: пауза между повторами
        static constexpr uint32_t DSHOT_MIN_GAP_MS = 280;   // тон регулятора ~0.26 с
        static constexpr uint32_t GUARD_MS = 1200;          // после DISARM и после тонов моторами

        void setMelody(const Melody& m)
        {
            tune = &m;
            stop();
        }
        void configure(bool motorTones) { motors = motorTones; }

        // Один раз («послушать», при включении). false — нечего играть или ARM.
        bool playOnce(bool armed, bool withMotors = true)
        {
            if (armed || !tune || tune->count == 0) return false;
            begin(withMotors);
            once = true;
            return true;
        }
        void stop()
        {
            playing = false;
            once = false;
            index = 0;
            left = 0;
            gap = 0;
        }
        bool isPlaying() const { return playing; }

        // Каждые 10 мс. switchOn — режим MELODY (уже со связью), armSwitch —
        // пилот армится.
        MelodyOut tick(uint32_t nowMs, bool armed, bool armSwitch, bool switchOn)
        {
            MelodyOut o;
            if (armed || !seenDisarmed)
            {
                if (armed) stop();
                seenDisarmed = !armed;
                disarmedAt = nowMs;
                return o;
            }
            if (armSwitch && playing)
            {
                stop();
                return o;
            }
            if (!once)
            {
                if (switchOn && !armSwitch && !playing && (!repeatAt || static_cast<int32_t>(nowMs - repeatAt) >= 0)) begin(true);
                if (!switchOn && playing) stop();
                if (!switchOn) repeatAt = 0;
            }
            if (!playing || !tune) return o;

            o.active = true;
            if (left == 0)
            {
                if (index >= tune->count)
                {
                    const bool wasOnce = once;
                    stop();
                    o.active = false;
                    if (!wasOnce) repeatAt = nowMs + REPEAT_PAUSE_MS;
                    return o;
                }
                const MelodyNote& n = tune->notes[index++];
                left = n.ms < TICK_MS ? TICK_MS : n.ms;
                // в конце ноты — короткая тишина, чтобы одинаковые ноты не слились
                gap = left >= 4 * TICK_MS ? (left >= 160 ? 2 * TICK_MS : TICK_MS) : 0;
                hz = n.hz;
                if (hz && useMotors && nowMs - disarmedAt >= GUARD_MS && (!dshotSent || nowMs - lastDshotMs >= DSHOT_MIN_GAP_MS))
                {
                    o.dshotTone = melodyBeaconTone(*tune, n);
                    dshotSent = true;
                    lastDshotMs = nowMs;
                }
            }
            o.hz = left > gap ? hz : 0;
            left = left > TICK_MS ? left - TICK_MS : 0;
            return o;
        }

        // Тоны моторами были недавно — ARM отложить.
        bool holdArm(uint32_t nowMs) const { return dshotSent && nowMs - lastDshotMs < GUARD_MS; }

    private:
        void begin(bool withMotors)
        {
            playing = tune && tune->count > 0;
            once = false;
            index = 0;
            left = 0;
            gap = 0;
            repeatAt = 0;
            useMotors = motors && withMotors;
        }

        const Melody* tune = nullptr;
        bool motors = false;
        bool useMotors = false;
        bool playing = false;
        bool once = false;
        bool seenDisarmed = false;
        uint8_t index = 0;
        uint32_t left = 0;
        uint32_t gap = 0;
        uint16_t hz = 0;
        uint32_t repeatAt = 0;
        uint32_t disarmedAt = 0;
        bool dshotSent = false;
        uint32_t lastDshotMs = 0;
    };
}
