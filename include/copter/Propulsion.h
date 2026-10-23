#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/Battery.h"
#include "copter/Filters.h"

// ============================================================
// Винтомоторная группа одного мотора: физическая модель.
//
// Одна модель на всё: по ней Presets.h выводит настройку (обороты,
// тяга, запаздывание мотора на висении), а тестовый симулятор квадро
// по ней же летает (с намеренно искажёнными параметрами — настройка
// должна держать и «не тот» коптер).
//
// Мотор — бесколлекторный с противо-ЭДС:
//   ток     I = (u·V − ω/Kv) / R,      момент мотора = I / Kv
//   винт    тяга T = kT·ω²,             момент сопротивления Q = kQ·ω²
//   разгон  J·dω/dt = I/Kv − kQ·ω²
// u — скважность регулятора 0..1, V — напряжение батареи, Kv — в
// рад/с на вольт. Постоянная времени у рабочей точки
//   τ = J / (1/(R·Kv²) + 2·kQ·ω):
// её задаёт не аэродинамика, а то, насколько сильно мотор может
// разгонять/тормозить винт.
//
// Коэффициенты винта — из шага/диаметра и числа лопастей (оценка по
// таблицам статической тяги: 5×4.3×3 на 2207 1950KV 6S даёт ~1.7 кг на
// ~28 тыс. об/мин). Сопротивление обмотки — из KV и объёма статора:
// у геометрически подобных моторов Km² ∝ объём^(5/3), R = 1/(Kv²·Km²);
// опора — 2207 2400KV ≈ 0.11 Ом вместе с регулятором и проводами (так
// 5" 2207 1950KV на 6S выходит на ~28 тыс. об/мин, ~45 А и ~1.6 кг тяги
// на полном газе — как на стендах). Это оценки ±30 %, для настройки
// достаточно: контур выбирается с запасом по фазе.
// ============================================================

namespace copter
{
    constexpr float AIR_DENSITY = 1.225f;   // кг/м³, у земли
    constexpr float GRAVITY = 9.80665f;

    struct PropSpec
    {
        float diameterIn = 5.0f;
        float pitchIn = 4.3f;
        uint8_t blades = 3;
        float massG = 0.0f;       // 0 — оценка по размеру
    };

    // Мотор как его называют: 2207 — статор 22 мм × 7 мм; 2806.5 — 28 × 6.5.
    struct MotorSpec
    {
        float statorWidthMm = 22.0f;
        float statorHeightMm = 7.0f;
        float kv = 1950.0f;
        float massG = 0.0f;       // 0 — оценка по статору
    };

    struct BatterySpec
    {
        uint8_t cells = 6;
        uint16_t capacityMah = 1300;
        float voltsPerCellLoaded = 3.75f;   // под нагрузкой (LiPo); Li-ion — ~3.5 (cellVoltages())
        BatteryChemistry chemistry = BatteryChemistry::LIPO;
    };

    class Propulsion
    {
    public:
        static float bladeFactor(uint8_t blades)
        {
            switch (blades)
            {
                case 2: return 1.0f;
                case 3: return 1.3f;
                case 4: return 1.5f;
                case 5: return 1.65f;
                case 6: return 1.75f;
                default: return blades < 2 ? 0.7f : 1.8f;
            }
        }

        static float statorVolume(const MotorSpec& m)
        {
            const float r = m.statorWidthMm * 0.5f;
            return PI_F * r * r * m.statorHeightMm;   // мм³
        }

        static Propulsion from(const PropSpec& prop, const MotorSpec& motor)
        {
            Propulsion p;
            p.prop = prop;
            p.motor = motor;
            const float d = prop.diameterIn * 0.0254f;
            const float pd = fminf(1.5f, fmaxf(0.3f, prop.pitchIn / prop.diameterIn));
            const float bf = bladeFactor(prop.blades);
            const float ct = (0.07f + 0.11f * pd) * bf;
            const float cp = (0.025f + 0.12f * pd) * powf(bf, 1.1f);
            const float cq = cp / (2.0f * PI_F);
            const float fourPi2 = 4.0f * PI_F * PI_F;
            p.kT = ct * AIR_DENSITY * powf(d, 4) / fourPi2;
            p.kQ = cq * AIR_DENSITY * powf(d, 5) / fourPi2;

            p.volume = statorVolume(motor);
            p.kvRad = motor.kv * 2.0f * PI_F / 60.0f;
            p.resistance = 0.004f + 0.11f * powf(2400.0f / motor.kv, 2.0f) * powf(2661.0f / p.volume, 5.0f / 3.0f);
            p.motorMassG = motor.massG > 0.0f ? motor.massG : 1.2f + 0.0118f * p.volume;
            p.propMassG = prop.massG > 0.0f ? prop.massG
                                            : fmaxf(0.2f, 4.5f * powf(prop.diameterIn / 5.0f, 2.6f) * powf(prop.blades / 3.0f, 0.8f));
            const float bellR = (motor.statorWidthMm * 0.5f + 1.0f) * 0.001f;
            const float jRotor = 0.3f * p.motorMassG * 0.001f * bellR * bellR;   // колокол с магнитами ~ треть массы мотора
            const float jProp = p.propMassG * 0.001f * (d * 0.5f) * (d * 0.5f) * 0.25f;
            p.inertia = jRotor + jProp;
            // Длительная мощность: у мелких моторов ~ объём статора, у крупных растёт медленнее (охлаждение).
            p.ratedPowerW = p.volume < 2661.0f ? 0.26f * p.volume : 700.0f * powf(p.volume / 2661.0f, 0.43f);
            return p;
        }

        // Установившиеся обороты (рад/с) при скважности u и напряжении V.
        float steadyOmega(float u, float volts) const
        {
            const float drive = fmaxf(0.0f, u) * volts;
            // kQ·ω² + ω/(R·Kv²) − drive/(R·Kv) = 0
            const float a = kQ;
            const float b = 1.0f / (resistance * kvRad * kvRad);
            const float c = -drive / (resistance * kvRad);
            return (-b + sqrtf(b * b - 4.0f * a * c)) / (2.0f * a);
        }

        float thrust(float omega) const { return kT * omega * omega; }
        float torque(float omega) const { return kQ * omega * omega; }

        float current(float u, float volts, float omega) const
        {
            return (u * volts - omega / kvRad) / resistance;
        }

        // Ускорение ротора, рад/с².
        float omegaDot(float u, float volts, float omega) const
        {
            return (current(u, volts, omega) / kvRad - kQ * omega * omega) / inertia;
        }

        float timeConstant(float omega) const
        {
            return inertia / (1.0f / (resistance * kvRad * kvRad) + 2.0f * kQ * omega);
        }

        PropSpec prop;
        MotorSpec motor;
        float kT = 0.0f, kQ = 0.0f;   // Н/(рад/с)², Н·м/(рад/с)²
        float kvRad = 0.0f;           // рад/с на В
        float resistance = 0.0f;      // Ом (обмотка + регулятор)
        float inertia = 0.0f;         // кг·м², ротор + винт
        float volume = 0.0f;          // мм³ статора
        float motorMassG = 0.0f;
        float propMassG = 0.0f;
        float ratedPowerW = 0.0f;
    };
}
