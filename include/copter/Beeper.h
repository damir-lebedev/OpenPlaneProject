#pragma once
#include <ctype.h>
#include <stdint.h>

// ============================================================
// Пищалка коптера — что и когда пищит. Условия — как `beeper` в Betaflight:
// те же имена и номера (бит условия = номер − 1, BF beeper_off_flags), каждое
// можно выключить (`beeper -RX_LOST`, `beeper_off_flags`).
//
//   события (один раз): GYRO_CALIBRATED, SYSTEM_INIT, READY_BEEP (проверки
//     прошли, у GPS — дом записан), ARMING / ARMING_GPS_FIX /
//     ARMING_GPS_NO_FIX, DISARMING, ACC_CALIBRATION(_FAIL), BLACKBOX_ERASE,
//     MULTI_BEEPS (N коротких — подтверждение команды стиками: профиль N);
//   тревоги (пока длятся): RX_SET (режим BEEPER — «поиск модели»: громко,
//     работает и без ARM, и без связи — тумблер держит последнее значение),
//     RX_LOST_LANDING (failsafe сажает), RX_LOST (нет связи — и на земле,
//     если связь хоть раз была), CRASH_FLIP (turtle mode), BAT_CRIT_LOW,
//     BAT_LOW, GPS_STATUS (режим BEEP GPS SATELLITE COUNT: столько коротких,
//     сколько спутников; без фикса — один длинный), DISARM_REPEAT (ARM
//     стиками: стик держат в положении DISARM);
//   ON_USB выключен — без батареи (питание от USB) молчит всё;
//   ARMED, CAM_CONNECTION_*, RC_SMOOTHING_INIT_FAIL — у нас не бывает
//     (моторы на холостых не стоят, управления камерой нет, сглаживание
//     не «инициализируется»): только имена, чтобы diff BF переносился.
// crash_recovery = BEEP пищит всегда (включается самой настройкой).
//
// Мелодия — строка «длительностей» по 10 мс: нечётные места — звук,
// чётные — пауза, 0 — конец (тревоги — с начала). Тревоги главнее
// событий; событие прерывает только тишину или повтор.
//
// DShot-маяк (dshot_beacon, beeper_dshot_beacon_tone): регуляторы пищат
// моторами, чтобы найти коптер в траве, — на RX_LOST и/или RX_SET (BF
// по умолчанию оба выключены). Только без ARM, через 1.2 с после DISARM,
// не чаще раза в 0.5 с и не при включённом ARM; ARM после маяка
// откладывается на 1.2 с (регулятор, пока пищит, газ не слушает) — как в BF.
//
// Без Arduino: tick() каждые 10 мс говорит, включить ли пищалку.
// ============================================================

namespace copter
{
    // Номера — как beeperMode_e в BF 4.5 (бит в beeper_off_flags = номер − 1).
    enum class BeeperCondition : uint8_t
    {
        NONE = 0,
        GYRO_CALIBRATED,
        RX_LOST,
        RX_LOST_LANDING,
        DISARMING,
        ARMING,
        ARMING_GPS_FIX,
        BAT_CRIT_LOW,
        BAT_LOW,
        GPS_STATUS,
        RX_SET,
        ACC_CALIBRATION,
        ACC_CALIBRATION_FAIL,
        READY_BEEP,
        MULTI_BEEPS,
        DISARM_REPEAT,
        ARMED,
        SYSTEM_INIT,
        ON_USB,
        BLACKBOX_ERASE,
        CRASH_FLIP,
        CAM_CONNECTION_OPEN,
        CAM_CONNECTION_CLOSE,
        RC_SMOOTHING_INIT_FAIL,
        ARMING_GPS_NO_FIX,
        COUNT,   // дальше — ALL в CLI
    };

    constexpr uint8_t BEEPER_CONDITION_COUNT = static_cast<uint8_t>(BeeperCondition::COUNT) - 1;   // 24
    constexpr uint32_t BEEPER_ALL_MASK = (1u << BEEPER_CONDITION_COUNT) - 1u;

    constexpr uint32_t beeperBit(BeeperCondition c)
    {
        return c == BeeperCondition::NONE || c >= BeeperCondition::COUNT ? 0u : 1u << (static_cast<uint8_t>(c) - 1);
    }

    // Маяк моторами — только на этих двух (как в BF).
    constexpr uint32_t DSHOT_BEACON_ALLOWED = beeperBit(BeeperCondition::RX_LOST) | beeperBit(BeeperCondition::RX_SET);

    // Имя как в CLI BF (beeper / beacon); c = 1..24.
    inline const char* beeperConditionName(BeeperCondition c)
    {
        static const char* const NAMES[] = { "",
                                             "GYRO_CALIBRATED",
                                             "RX_LOST",
                                             "RX_LOST_LANDING",
                                             "DISARMING",
                                             "ARMING",
                                             "ARMING_GPS_FIX",
                                             "BAT_CRIT_LOW",
                                             "BAT_LOW",
                                             "GPS_STATUS",
                                             "RX_SET",
                                             "ACC_CALIBRATION",
                                             "ACC_CALIBRATION_FAIL",
                                             "READY_BEEP",
                                             "MULTI_BEEPS",
                                             "DISARM_REPEAT",
                                             "ARMED",
                                             "SYSTEM_INIT",
                                             "ON_USB",
                                             "BLACKBOX_ERASE",
                                             "CRASH_FLIP",
                                             "CAM_CONNECTION_OPEN",
                                             "CAM_CONNECTION_CLOSE",
                                             "RC_SMOOTHING_INIT_FAIL",
                                             "ARMING_GPS_NO_FIX" };
        const uint8_t i = static_cast<uint8_t>(c);
        return i < sizeof(NAMES) / sizeof(NAMES[0]) ? NAMES[i] : "";
    }

    // Имя (без учёта регистра) -> условие; «ALL» — COUNT; нет такого — NONE.
    inline BeeperCondition beeperConditionByName(const char* name)
    {
        auto same = [](const char* a, const char* b) {
            while (*a && *b && toupper(static_cast<unsigned char>(*a)) == *b)
            {
                ++a;
                ++b;
            }
            return *a == '\0' && *b == '\0';
        };
        if (!name || !*name) return BeeperCondition::NONE;
        if (same(name, "ALL")) return BeeperCondition::COUNT;
        for (uint8_t i = 1; i <= BEEPER_CONDITION_COUNT; ++i)
        {
            const BeeperCondition c = static_cast<BeeperCondition>(i);
            if (same(name, beeperConditionName(c))) return c;
        }
        return BeeperCondition::NONE;
    }

    struct BeeperInputs
    {
        bool batteryWarning = false;    // BAT_LOW
        bool batteryCritical = false;   // BAT_CRIT_LOW
        bool rxLost = false;            // RX_LOST: связи нет (а была)
        bool rxLostLanding = false;     // RX_LOST_LANDING: failsafe этап 2 — сажает
        bool crashRecovery = false;     // crash_recovery = BEEP: выравнивается после удара
        bool crashFlip = false;         // turtle mode: заармлен, моторы наоборот (BF CRASH_FLIP)
        bool findMe = false;            // режим BEEPER (RX_SET)
        bool muted = false;             // режим BEEPER MUTE
        bool gpsCount = false;          // режим BEEP GPS SATELLITE COUNT (GPS_STATUS)
        bool gpsFix = false;
        uint8_t gpsSats = 0;
        bool disarmRepeat = false;      // DISARM_REPEAT: стик в положении DISARM (ARM стиками)
        bool usbOnly = false;           // батареи нет — питание от USB (ON_USB)
    };

    class Beeper
    {
    public:
        static constexpr uint8_t MAX_MULTI = 20;   // коротких подряд (спутники, профиль)

        // beeper_off_flags: выключенные условия (бит = номер − 1).
        void setOffFlags(uint32_t mask) { offFlags = mask & BEEPER_ALL_MASK; }
        uint32_t getOffFlags() const { return offFlags; }
        bool enabled(BeeperCondition c) const { return (offFlags & beeperBit(c)) == 0; }

        // Одноразовый сигнал; n — для MULTI_BEEPS (сколько коротких). Играет
        // другое событие — этот следом (одно место); ARM/DISARM — сразу.
        void event(BeeperCondition c, uint8_t n = 0)
        {
            if (!enabled(c) || !eventTune(c, n)) return;
            const bool urgent = c == BeeperCondition::ARMING || c == BeeperCondition::ARMING_GPS_FIX ||
                                c == BeeperCondition::ARMING_GPS_NO_FIX || c == BeeperCondition::DISARMING;
            if (tune && !repeat && !urgent)
            {
                pending = c;
                pendingN = n;
                return;
            }
            start(eventTune(c, n), c, 1);
        }

        // «Послушать» (приложение, без ARM): мелодия условия даже выключенного;
        // тревоги — двумя кругами.
        bool preview(BeeperCondition c)
        {
            const uint8_t* t = eventTune(c, 3);
            if (!t)
            {
                BeeperInputs in;
                in.gpsSats = 8;
                in.gpsFix = true;
                t = alarmTune(c, in);
            }
            if (!t) return false;
            start(t, c, eventTune(c, 3) ? 1 : 2);
            return true;
        }

        // Каждые 10 мс: true — пищать.
        bool tick(const BeeperInputs& in)
        {
            const BeeperCondition a = alarmFor(in);
            const uint8_t* alarm = alarmTune(a, in);
            // тревога главнее одноразового; одноразовое доигрывает поверх тишины
            if (alarm && current != a && (repeat || !tune)) start(alarm, a, 0);
            if (!alarm && repeat) stop();
            if (!tune) return false;
            if (left == 0)
            {
                ++step;
                if (tune[step] == 0)
                {
                    if (!repeat && --plays == 0)
                    {
                        stop();
                        if (pending != BeeperCondition::NONE)
                        {
                            start(eventTune(pending, pendingN), pending, 1);
                            pending = BeeperCondition::NONE;
                        }
                        return false;
                    }
                    // GPS_STATUS: спутников стало другое число — новая мелодия с начала круга
                    if (repeat && current == BeeperCondition::GPS_STATUS) gpsTune(in);
                    step = 0;
                }
                left = tune[step];
            }
            --left;
            const bool on = (step % 2) == 0;
            if (current == BeeperCondition::RX_SET) return on;   // «поиск модели» не глушится
            if (in.muted) return false;
            if (in.usbOnly && !enabled(BeeperCondition::ON_USB)) return false;
            return on;
        }

        bool busy() const { return tune != nullptr; }
        BeeperCondition playing() const { return tune ? current : BeeperCondition::NONE; }

    private:
        // Тревога сейчас (выключенные не играют): по важности.
        BeeperCondition alarmFor(const BeeperInputs& in) const
        {
            struct Row
            {
                bool on;
                BeeperCondition c;
            };
            const Row rows[] = {
                { in.findMe, BeeperCondition::RX_SET },
                { in.rxLostLanding, BeeperCondition::RX_LOST_LANDING },
                { in.rxLost, BeeperCondition::RX_LOST },
                { in.crashRecovery, BeeperCondition::NONE },   // crash_recovery = BEEP — без условия
                { in.crashFlip, BeeperCondition::CRASH_FLIP },
                { in.batteryCritical, BeeperCondition::BAT_CRIT_LOW },
                { in.batteryWarning, BeeperCondition::BAT_LOW },
                { in.gpsCount, BeeperCondition::GPS_STATUS },
                { in.disarmRepeat, BeeperCondition::DISARM_REPEAT },
            };
            for (const Row& r : rows)
            {
                if (!r.on) continue;
                if (r.c == BeeperCondition::NONE) return CRASH_RECOVERY;
                if (enabled(r.c)) return r.c;
            }
            return BeeperCondition::NONE;
        }

        const uint8_t* alarmTune(BeeperCondition c, const BeeperInputs& in)
        {
            switch (c)
            {
                case BeeperCondition::RX_SET: return FIND_ME_TUNE;
                case BeeperCondition::RX_LOST_LANDING: return LANDING_TUNE;
                case BeeperCondition::RX_LOST: return RX_LOST_TUNE;
                case CRASH_RECOVERY: return CRASH_TUNE;
                case BeeperCondition::CRASH_FLIP: return CRASH_FLIP_TUNE;
                case BeeperCondition::BAT_CRIT_LOW: return CRITICAL_TUNE;
                case BeeperCondition::BAT_LOW: return WARNING_TUNE;
                case BeeperCondition::GPS_STATUS:
                    if (current != BeeperCondition::GPS_STATUS || !tune) gpsTune(in);
                    return gpsBuf;
                case BeeperCondition::DISARM_REPEAT: return DISARM_REPEAT_TUNE;
                default: return nullptr;
            }
        }

        const uint8_t* eventTune(BeeperCondition c, uint8_t n)
        {
            switch (c)
            {
                case BeeperCondition::GYRO_CALIBRATED: return GYRO_TUNE;
                case BeeperCondition::SYSTEM_INIT: return INIT_TUNE;
                case BeeperCondition::READY_BEEP: return READY_TUNE;
                case BeeperCondition::ARMING: return ARMED_TUNE;
                case BeeperCondition::ARMING_GPS_FIX: return ARMED_GPS_TUNE;
                case BeeperCondition::ARMING_GPS_NO_FIX: return ARMED_NO_GPS_TUNE;
                case BeeperCondition::DISARMING: return DISARMED_TUNE;
                case BeeperCondition::ACC_CALIBRATION: return ACC_TUNE;
                case BeeperCondition::ACC_CALIBRATION_FAIL: return ACC_FAIL_TUNE;
                case BeeperCondition::BLACKBOX_ERASE: return ERASE_TUNE;
                case BeeperCondition::MULTI_BEEPS: return shorts(multiBuf, n ? n : 1, 15, 0);
                default: return nullptr;
            }
        }

        // N коротких по 50 мс, пауза между pause, в конце — тишина tail (0 — сразу конец).
        static const uint8_t* shorts(uint8_t* buf, uint8_t n, uint8_t pause, uint8_t tail)
        {
            if (n > MAX_MULTI) n = MAX_MULTI;
            uint8_t at = 0;
            for (uint8_t i = 0; i < n; ++i)
            {
                buf[at++] = 5;
                buf[at++] = i + 1 < n ? pause : (tail ? tail : 1);
            }
            buf[at] = 0;
            return buf;
        }

        // Спутники коротким писком (без фикса — один длинный), пауза 1.5 с.
        void gpsTune(const BeeperInputs& in)
        {
            if (in.gpsFix && in.gpsSats > 0)
            {
                shorts(gpsBuf, in.gpsSats, 15, 150);
                return;
            }
            gpsBuf[0] = 40;
            gpsBuf[1] = 150;
            gpsBuf[2] = 0;
        }

        void start(const uint8_t* t, BeeperCondition c, uint8_t times)
        {
            tune = t;
            current = c;
            repeat = times == 0;
            plays = times;
            step = 0;
            left = t[0];
        }

        void stop()
        {
            tune = nullptr;
            current = BeeperCondition::NONE;
            repeat = false;
        }

        // crash_recovery = BEEP: своё «условие» вне номеров BF
        static constexpr BeeperCondition CRASH_RECOVERY = static_cast<BeeperCondition>(0xFE);

        // длительности по 10 мс: звук, пауза, звук, …, 0
        static constexpr uint8_t GYRO_TUNE[] = { 3, 5, 3, 0 };
        static constexpr uint8_t INIT_TUNE[] = { 8, 6, 8, 6, 20, 0 };
        static constexpr uint8_t READY_TUNE[] = { 5, 0 };
        static constexpr uint8_t ARMED_TUNE[] = { 50, 0 };
        static constexpr uint8_t ARMED_GPS_TUNE[] = { 50, 10, 5, 5, 5, 0 };
        static constexpr uint8_t ARMED_NO_GPS_TUNE[] = { 50, 10, 25, 0 };
        static constexpr uint8_t DISARMED_TUNE[] = { 12, 10, 12, 0 };
        static constexpr uint8_t ACC_TUNE[] = { 5, 5, 5, 5, 5, 0 };
        static constexpr uint8_t ACC_FAIL_TUNE[] = { 40, 10, 40, 0 };
        static constexpr uint8_t ERASE_TUNE[] = { 25, 10, 5, 0 };
        static constexpr uint8_t WARNING_TUNE[] = { 10, 190, 0 };
        static constexpr uint8_t CRITICAL_TUNE[] = { 10, 10, 0 };
        static constexpr uint8_t RX_LOST_TUNE[] = { 30, 10, 10, 10, 0 };
        static constexpr uint8_t LANDING_TUNE[] = { 5, 5, 5, 5, 5, 15, 25, 5, 25, 5, 25, 15, 5, 5, 5, 5, 5, 60, 0 };
        static constexpr uint8_t FIND_ME_TUNE[] = { 20, 10, 20, 50, 0 };
        static constexpr uint8_t CRASH_TUNE[] = { 5, 5, 0 };
        static constexpr uint8_t CRASH_FLIP_TUNE[] = { 8, 8, 8, 76, 0 };
        static constexpr uint8_t DISARM_REPEAT_TUNE[] = { 5, 45, 0 };

        uint8_t multiBuf[MAX_MULTI * 2 + 1] = {};
        uint8_t gpsBuf[MAX_MULTI * 2 + 1] = {};
        uint32_t offFlags = 0;
        const uint8_t* tune = nullptr;
        BeeperCondition current = BeeperCondition::NONE;
        BeeperCondition pending = BeeperCondition::NONE;
        uint8_t pendingN = 0;
        bool repeat = false;
        uint8_t plays = 0;
        uint8_t step = 0;
        uint8_t left = 0;
    };

    // DShot-маяк: когда послать регуляторам команду «пискни» (BEACON1..5).
    class DshotBeacon
    {
    public:
        static constexpr uint32_t INTERVAL_MS = 500;   // не чаще (звук регулятора ~0.26 с)
        static constexpr uint32_t GUARD_MS = 1200;     // после DISARM — тишина; после маяка — ARM ждёт

        // dshot_beacon_off_flags (только RX_LOST и RX_SET) и тон 1..5.
        void configure(uint32_t offFlags, uint8_t toneNumber)
        {
            off = offFlags & DSHOT_BEACON_ALLOWED;
            tone = toneNumber < 1 ? 1 : toneNumber > 5 ? 5 : toneNumber;
        }
        bool enabled(BeeperCondition c) const { return (beeperBit(c) & DSHOT_BEACON_ALLOWED & ~off) != 0; }
        uint8_t toneCommand() const { return tone; }

        // Каждый цикл. Возвращает команду DShot (BEACON1..5), которую послать
        // всем моторам сейчас, или 0. armSwitch — тумблер ARM включён (пилот
        // армится — маяк молчит).
        uint8_t update(uint32_t nowMs, bool armed, bool armSwitch, bool rxLost, bool findMe)
        {
            if (armed || !seenDisarmed)
            {
                disarmedAt = nowMs;
                seenDisarmed = !armed;
                return 0;
            }
            const bool want = (rxLost && enabled(BeeperCondition::RX_LOST)) || (findMe && enabled(BeeperCondition::RX_SET));
            if (!want || armSwitch) return 0;
            if (nowMs - disarmedAt < GUARD_MS) return 0;
            if (sent && nowMs - lastMs < INTERVAL_MS) return 0;
            sent = true;
            lastMs = nowMs;
            return tone;
        }

        // Маяк был недавно — ARM отложить (регулятор, пока пищит, газ не слушает).
        bool holdArm(uint32_t nowMs) const { return sent && nowMs - lastMs < GUARD_MS; }

    private:
        uint32_t off = DSHOT_BEACON_ALLOWED;
        uint8_t tone = 1;
        bool seenDisarmed = false;
        bool sent = false;
        uint32_t disarmedAt = 0;
        uint32_t lastMs = 0;
    };
}
