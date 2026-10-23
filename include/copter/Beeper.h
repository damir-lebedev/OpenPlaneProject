#pragma once
#include <stdint.h>

// ============================================================
// Пищалка коптера — что и когда пищит (как в Betaflight по смыслу):
//
//   ARM          — один длинный;          DISARM — два коротких;
//   батарея WARNING — короткий раз в 2 с; CRITICAL — часто, без остановки;
//   failsafe     — тревожный, пока длится;
//   режим BEEPER — «поиск модели»: громко и часто (работает и без ARM, и
//                  без связи — тумблер держит последнее значение);
//   готов к ARM  — после включения, когда все проверки прошли, — короткий.
//
// Мелодия — строка «длительностей» по 10 мс: нечётные места — звук,
// чётные — пауза, 0 — конец (повторяющиеся — с начала). Тревоги главнее
// событий; событие прерывает только тишину или повтор.
//
// Без Arduino: tick() каждые 10 мс говорит, включить ли пищалку.
// ============================================================

namespace copter
{
    enum class BeeperEvent : uint8_t
    {
        NONE,
        ARMED,
        DISARMED,
        READY,
    };

    struct BeeperInputs
    {
        bool batteryWarning = false;
        bool batteryCritical = false;
        bool failsafe = false;
        bool findMe = false;    // режим BEEPER
        bool muted = false;     // режим BEEPER MUTE
    };

    class Beeper
    {
    public:
        // Одноразовый сигнал (ARM, DISARM, готов).
        void event(BeeperEvent e)
        {
            switch (e)
            {
                case BeeperEvent::ARMED: start(ARMED_TUNE, false); break;
                case BeeperEvent::DISARMED: start(DISARMED_TUNE, false); break;
                case BeeperEvent::READY: start(READY_TUNE, false); break;
                case BeeperEvent::NONE:
                default: break;
            }
        }

        // Каждые 10 мс: true — пищать.
        bool tick(const BeeperInputs& in)
        {
            const uint8_t* alarm = in.findMe            ? FIND_ME_TUNE
                                   : in.failsafe        ? FAILSAFE_TUNE
                                   : in.batteryCritical ? CRITICAL_TUNE
                                   : in.batteryWarning  ? WARNING_TUNE
                                                        : nullptr;
            // тревога главнее одноразового; одноразовое доигрывает поверх тишины
            if (alarm && tune != alarm && (repeat || !tune)) start(alarm, true);
            if (!alarm && repeat) stop();
            if (!tune) return false;
            if (left == 0)
            {
                ++step;
                if (tune[step] == 0)
                {
                    if (!repeat)
                    {
                        stop();
                        return false;
                    }
                    step = 0;
                }
                left = tune[step];
            }
            --left;
            const bool on = (step % 2) == 0;
            if (in.muted && !in.findMe) return false;   // «поиск модели» не глушится
            return on;
        }

        bool busy() const { return tune != nullptr; }

    private:
        void start(const uint8_t* t, bool repeating)
        {
            tune = t;
            repeat = repeating;
            step = 0;
            left = t[0];
        }

        void stop()
        {
            tune = nullptr;
            repeat = false;
        }

        // длительности по 10 мс: звук, пауза, звук, …, 0
        static constexpr uint8_t ARMED_TUNE[] = { 50, 0 };
        static constexpr uint8_t DISARMED_TUNE[] = { 12, 10, 12, 0 };
        static constexpr uint8_t READY_TUNE[] = { 5, 0 };
        static constexpr uint8_t WARNING_TUNE[] = { 10, 190, 0 };
        static constexpr uint8_t CRITICAL_TUNE[] = { 10, 10, 0 };
        static constexpr uint8_t FAILSAFE_TUNE[] = { 30, 10, 10, 10, 0 };
        static constexpr uint8_t FIND_ME_TUNE[] = { 20, 10, 20, 50, 0 };

        const uint8_t* tune = nullptr;
        bool repeat = false;
        uint8_t step = 0;
        uint8_t left = 0;
    };
}
