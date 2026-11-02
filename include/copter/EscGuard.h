#pragma once
#include <math.h>
#include <stdint.h>

// ============================================================
// Защита по железу: «регулятор может сгореть» и «батарея перегружена».
//
// Прошивка знает из описания коптера (Studio → «Что ещё подключено»):
//   esc_current_a        — длительный ток регулятора на мотор, А (0 — не следим);
//   esc_burst_current_a  — пиковый ток на мотор, А (0 — 1.25 × длительного);
//   esc_overload_time    — сколько можно выше длительного, 0.1 с (50 — 5 с);
//   battery_c_rating     — C батареи (0 — не следим): предел = C × ёмкость.
// Ток — с датчика (current_meter = ADC): общий ток ÷ число моторов. Это
// среднее: в резком манёвре один мотор тянет больше, поэтому порог стоит
// брать честный (длительный ток с коробки, не «пиковый маркетинговый»).
// Когда будет ток по мотору (EDT, ESC-телеметрия), берётся наибольший.
//
// Перегрузка регулятора: выше длительного дольше esc_overload_time или выше
// пикового дольше 0.3 с (не от одного выброса). Батареи: выше C × ёмкость
// дольше esc_overload_time. Предупреждение держится 2 с после конца — не
// мигает. Что делает прошивка: OSD «ESC OVERLOAD» / «BATT OVERLOAD», писк
// как LOW BATTERY, метка в чёрном ящике, после посадки — в статистике.
// Газ не отнимается: лучше предупредить, чем лишить тяги в воздухе.
//
// Без Arduino — тестируется на ПК (test_copter).
// ============================================================

namespace copter
{
    struct EscGuardConfig
    {
        float escContinuousA = 0.0f;   // esc_current_a
        float escBurstA = 0.0f;        // esc_burst_current_a: 0 — 1.25 × длительного
        float overloadS = 5.0f;        // esc_overload_time / 10
        float batteryC = 0.0f;         // battery_c_rating
        float batteryMah = 0.0f;       // battery_mah (описание коптера)
        uint8_t motors = 4;

        float burstA() const { return escBurstA > 0.0f ? escBurstA : escContinuousA * 1.25f; }
        float batteryMaxA() const { return batteryC > 0.0f && batteryMah > 0.0f ? batteryC * batteryMah / 1000.0f : 0.0f; }
        bool watchesEsc() const { return escContinuousA > 0.0f; }
        bool watchesBattery() const { return batteryMaxA() > 0.0f; }
    };

    enum class Overload : uint8_t
    {
        NONE,
        ESC,       // регулятор: выше длительного долго или выше пикового
        BATTERY,   // батарея: выше C × ёмкость долго
    };

    inline const char* overloadText(Overload o)
    {
        switch (o)
        {
            case Overload::ESC: return "ESC OVERLOAD";
            case Overload::BATTERY: return "BATT OVERLOAD";
            case Overload::NONE:
            default: return nullptr;
        }
    }

    // Итоги полёта (сбрасываются при ARM).
    struct OverloadStats
    {
        float maxMotorA = 0.0f;     // наибольший ток на мотор
        uint16_t escEvents = 0;     // сколько раз включалось ESC OVERLOAD
        uint16_t batteryEvents = 0;
        float escSeconds = 0.0f;    // сколько всего длилось
        float batterySeconds = 0.0f;
    };

    class EscGuard
    {
    public:
        static constexpr float BURST_S = 0.3f;   // выше пикового — не от одного выброса
        static constexpr float HOLD_S = 2.0f;    // предупреждение держится после конца

        void configure(const EscGuardConfig& c) { cfg = c; }
        const EscGuardConfig& config() const { return cfg; }

        // ARM — статистика полёта с нуля.
        void resetFlight()
        {
            stats = OverloadStats();
            escAboveS = burstAboveS = battAboveS = 0.0f;
            escHoldS = battHoldS = 0.0f;
            escOn = battOn = false;
        }

        // Раз в dtS: общий ток (А, с датчика) или наибольший ток мотора
        // (motorMaxA ≥ 0, когда он известен). Без датчика — hasCurrent = false.
        void update(bool armed, bool hasCurrent, float totalA, float dtS, float motorMaxA = -1.0f)
        {
            if (!armed || !hasCurrent || dtS <= 0.0f)
            {
                escAboveS = burstAboveS = battAboveS = 0.0f;
                escHoldS = battHoldS = 0.0f;
                escOn = battOn = false;
                return;
            }
            const float perMotor = motorMaxA >= 0.0f ? motorMaxA : totalA / (cfg.motors ? cfg.motors : 4);
            if (perMotor > stats.maxMotorA) stats.maxMotorA = perMotor;

            if (cfg.watchesEsc())
            {
                escAboveS = perMotor > cfg.escContinuousA ? escAboveS + dtS : 0.0f;
                burstAboveS = perMotor > cfg.burstA() ? burstAboveS + dtS : 0.0f;
                const bool over = escAboveS > cfg.overloadS || burstAboveS > BURST_S;
                step(over, escOn, escHoldS, stats.escEvents, stats.escSeconds, dtS);
            }
            if (cfg.watchesBattery())
            {
                battAboveS = totalA > cfg.batteryMaxA() ? battAboveS + dtS : 0.0f;
                step(battAboveS > cfg.overloadS, battOn, battHoldS, stats.batteryEvents, stats.batterySeconds, dtS);
            }
        }

        // Что показать: регулятор важнее батареи.
        Overload state() const
        {
            if (escOn) return Overload::ESC;
            if (battOn) return Overload::BATTERY;
            return Overload::NONE;
        }

        const OverloadStats& flightStats() const { return stats; }

    private:
        EscGuardConfig cfg;
        OverloadStats stats;
        float escAboveS = 0.0f, burstAboveS = 0.0f, battAboveS = 0.0f;
        float escHoldS = 0.0f, battHoldS = 0.0f;
        bool escOn = false, battOn = false;

        static void step(bool over, bool& on, float& holdS, uint16_t& events, float& seconds, float dtS)
        {
            if (over)
            {
                if (!on) ++events;
                on = true;
                holdS = HOLD_S;
            }
            else if (on)
            {
                holdS -= dtS;
                if (holdS <= 0.0f) on = false;
            }
            if (over) seconds += dtS;
        }
    };
}
