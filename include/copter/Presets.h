#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "copter/EscProtocol.h"
#include "copter/Frames.h"
#include "copter/GyroFilter.h"
#include "copter/Mixer.h"
#include "copter/Propulsion.h"

// ============================================================
// Сборки мультикоптеров и настройка по их описанию.
//
// Вместо того чтобы подбирать PID, фильтры и расходы руками, пилот
// описывает коптер: раму, пропеллеры, моторы, батарею, массу и стиль
// полёта (или берёт готовую сборку из каталога). По физической модели
// (Propulsion.h) считается всё остальное:
//   • висение: газ, обороты, ток; полная тяга и тяговооружённость;
//   • запаздывание мотора на висении (τ) и момент инерции рамы;
//   • фильтры: частота вращения моторов на холостых, висении и полном
//     газе -> ФНЧ гироскопа и D, полоса режекторов оценки оборотов;
//   • контур угловой скорости: частота среза выбирается так, чтобы
//     запас по фазе с учётом τ мотора, фильтров, цикла и регулятора был
//     50° (рысканье — 55°), запас по усилению — не меньше 6 дБ; P, I, D,
//     упреждение (FF) — из неё и коэффициента «команда -> ускорение»;
//   • поправка усиления по газу (TPA) — из той же модели: на большом
//     газе мотор отзывчивее, на малом — слабее;
//   • расходы, наклон в режиме горизонта, холостые — по стилю и размеру;
//   • предупреждения: мало тяги, мотор перегружен этим винтом,
//     пропеллеры задевают друг друга, висит на большом газе, вибрации
//     выше частоты цикла.
//
// Каталог — заготовки от вупа 65 мм до шестилучевика с 15" винтами;
// любую можно взять за основу и поменять размер винта, мотор, батарею.
// ============================================================

namespace copter
{
    enum class FlyStyle : uint8_t
    {
        INDOOR,       // вупы в помещении
        FREESTYLE,
        RACE,
        CINEMATIC,    // плавно, для съёмки
        LONG_RANGE,
        HEAVY_LIFT,   // грузовые, большие винты
    };

    inline const char* flyStyleName(FlyStyle s)
    {
        switch (s)
        {
            case FlyStyle::INDOOR: return "в помещении";
            case FlyStyle::FREESTYLE: return "фристайл";
            case FlyStyle::RACE: return "гонки";
            case FlyStyle::CINEMATIC: return "съёмка";
            case FlyStyle::LONG_RANGE: return "дальние полёты";
            case FlyStyle::HEAVY_LIFT: return "грузовой";
        }
        return "?";
    }

    struct BuildSpec
    {
        const char* id;
        const char* title;
        FrameType frame;
        float wheelbaseMm;       // между осями противоположных моторов
        PropSpec prop;
        MotorSpec motor;
        BatterySpec battery;
        float massG;             // взлётная масса; 0 — оценка
        FlyStyle style;
        bool propsOut = false;   // пропеллеры наружу: все моторы крутятся наоборот
    };

    // Каталог: «заготовки» под типичные сборки.
    inline const BuildSpec* buildCatalog(uint8_t& count)
    {
        static const BuildSpec catalog[] = {
            { "whoop65", "Вуп 65 мм, 1S", FrameType::QUAD_X, 65,
              { 1.22f, 0.85f, 3, 0.3f }, { 7, 2, 23000, 0 }, { 1, 300, 3.6f }, 22, FlyStyle::INDOOR },
            { "whoop75", "Вуп 75 мм, 1S", FrameType::QUAD_X, 75,
              { 1.57f, 1.0f, 4, 0.45f }, { 8, 2, 19000, 0 }, { 1, 450, 3.6f }, 32, FlyStyle::INDOOR },
            { "toothpick25", "Тусклик 2.5\", 3S", FrameType::QUAD_X, 115,
              { 2.5f, 1.7f, 3, 1.2f }, { 11, 3, 8000, 0 }, { 3, 450, 3.7f }, 70, FlyStyle::FREESTYLE },
            { "micro3", "Микро 3\", 4S", FrameType::QUAD_X, 140,
              { 3.0f, 3.0f, 3, 1.8f }, { 14, 4, 4500, 0 }, { 4, 650, 3.75f }, 190, FlyStyle::FREESTYLE },
            { "cinewhoop3", "Синевуп 3\", 4S", FrameType::QUAD_X, 150,
              { 3.0f, 1.8f, 5, 2.4f }, { 15, 4, 3600, 0 }, { 4, 850, 3.75f }, 330, FlyStyle::CINEMATIC },
            { "freestyle5", "Фристайл 5\", 6S", FrameType::QUAD_X, 225,
              { 5.1f, 4.3f, 3, 4.5f }, { 22, 7, 1950, 0 }, { 6, 1300, 3.75f }, 680, FlyStyle::FREESTYLE },
            { "race5", "Гоночный 5\", 6S", FrameType::QUAD_X_STRETCHED, 215,
              { 5.1f, 4.8f, 3, 4.2f }, { 23, 6, 2000, 0 }, { 6, 1100, 3.75f }, 560, FlyStyle::RACE },
            { "longrange7", "Дальний 7\", 6S", FrameType::QUAD_X, 315,
              { 7.0f, 4.0f, 3, 8.0f }, { 28, 6.5f, 1300, 0 }, { 6, 2200, 3.75f }, 1000, FlyStyle::LONG_RANGE },
            // 7" на Li-ion: 2810 1300KV, двухлопастные 7040, 6S 4000 мА·ч (21700),
            // аналоговый VTX на 4 Вт с вентилятором — тяжелее Li-Po-сборки.
            { "longrange7li", "Дальний 7\" Li-ion, 6S", FrameType::QUAD_X, 315,
              { 7.0f, 4.0f, 2, 7.0f }, { 28, 10, 1300, 0 }, { 6, 4000, 3.5f, BatteryChemistry::LIION }, 1050,
              FlyStyle::LONG_RANGE },
            { "heavy10", "Тяжёлый 10\", 6S", FrameType::QUAD_X, 450,
              { 10.0f, 5.0f, 2, 15.0f }, { 31, 15, 900, 0 }, { 6, 5000, 3.75f }, 2200, FlyStyle::HEAVY_LIFT },
            { "cinelifter8", "Синелифтер X8 8\", 6S", FrameType::X8, 380,
              { 8.0f, 4.5f, 3, 10.0f }, { 28, 12, 1100, 0 }, { 6, 4000, 3.75f }, 3500, FlyStyle::CINEMATIC },
            { "hex13", "Гекса 13\", 6S", FrameType::HEX_X, 700,
              { 13.0f, 4.4f, 2, 24.0f }, { 35, 10, 600, 0 }, { 6, 8000, 3.75f }, 4000, FlyStyle::HEAVY_LIFT },
            { "hex15", "Гекса 15\", 6S", FrameType::HEX_X, 900,
              { 15.0f, 5.5f, 2, 35.0f }, { 41, 14, 400, 0 }, { 6, 10000, 3.75f }, 6000, FlyStyle::HEAVY_LIFT },
        };
        count = sizeof(catalog) / sizeof(catalog[0]);
        return catalog;
    }

    inline const BuildSpec* findBuild(const char* id)
    {
        uint8_t n = 0;
        const BuildSpec* c = buildCatalog(n);
        for (uint8_t i = 0; i < n; ++i)
        {
            if (strcmp(c[i].id, id) == 0) return &c[i];
        }
        return nullptr;
    }

    // ---------------- расходы ----------------

    // Кривая «стик -> угловая скорость», °/с: наклон в центре, максимум, экспо 0..1.
    struct RateCurve
    {
        float centerDps = 200.0f;
        float maxDps = 670.0f;
        float expo = 0.5f;

        float apply(float stick) const
        {
            const float x = fminf(1.0f, fmaxf(-1.0f, stick));
            const float a = fabsf(x);
            return centerDps * x + (maxDps - centerDps) * x * powf(a, 1.0f + 4.0f * expo);
        }
    };

    // ---------------- настройка ----------------

    struct AxisGains
    {
        float p = 0, i = 0, d = 0, ff = 0;   // ошибка в °/с -> команда смесителя
    };

    enum TuneWarning : uint16_t
    {
        WARN_LOW_THRUST = 1 << 0,       // тяговооружённость < 1.8: вяло, ветер не перетянет
        WARN_NO_LIFT_MARGIN = 1 << 1,   // < 1.2: опасно, может не удержаться
        WARN_MOTOR_OVERLOAD = 1 << 2,   // мотор не тянет этот винт на полном газе
        WARN_PROP_OVERLAP = 1 << 3,     // соседние винты задевают
        WARN_HEAVY_HOVER = 1 << 4,      // висит выше 60 % газа
        WARN_VIBRATION_ALIAS = 1 << 5,  // вращение моторов выше половины частоты цикла
    };

    constexpr uint8_t TPA_POINTS = 11;   // газ 0, 0.1 … 1.0

    // tpa_mode: PD и D — кривая BF (до tpa_breakpoint 1, дальше линейно до
    // 1 − tpa_rate на полном газе) на P и D или только на D; MODEL — таблица
    // из модели винтомоторной группы на P и D (наша).
    enum class TpaMode : uint8_t
    {
        PD,
        D,
        MODEL,
    };

    // anti-gravity: полный разгон — газ меняется со скоростью ~9.4 хода в
    // секунду (весь ход за 0.1 с); P растёт на эту долю anti_gravity_p_gain.
    constexpr float ANTI_GRAVITY_FULL_RATE = 3.0f * PI_F;
    constexpr float ANTI_GRAVITY_P_SHARE = 0.5f;

    // iterm_relax: на каких осях интеграл придерживается при резком движении
    // уставки; *_INC — только пока интеграл растёт (уменьшаться может всегда).
    enum class ItermRelax : uint8_t
    {
        OFF,
        RP,
        RPY,
        RP_INC,
        RPY_INC,
    };

    // iterm_relax_type: SETPOINT — интеграл копится медленнее, пока уставка
    // быстро меняется; GYRO — интеграл считает ошибку к сглаженной уставке,
    // а в полосе «насколько уставка убежала от сглаженной» — ноль.
    enum class ItermRelaxType : uint8_t
    {
        GYRO,
        SETPOINT,
    };

    // Тонкости PID, как в Betaflight (настройки PID-профиля, Params.h):
    // D, растущий на резком движении, I-relax, поворот интеграла и windup,
    // пределы суммы, anti-gravity, throttle boost, поправка по газу (TPA).
    // Здесь — умолчания модели: всё как было до этих настроек, кроме
    // anti-gravity — она как в BF 4.3+ (80, 5 Гц), но без добавки к P: прежняя
    // (I до ×4 по газу через ФНЧ 15 Гц) на рывке газом с висения до полного
    // у 5" с центром тяжести не в центре давала клевок 7–16° против 2–3.5°.
    struct PidExtras
    {
        float dMax[3] = {};               // D на резком движении (наш коэффициент); не больше D — D постоянный
        float dMaxGain = 37.0f;           // d_max_gain: чувствительность к угловому ускорению (гироскоп)
        float dMaxAdvance = 20.0f;        // d_max_advance: то же по скорости изменения уставки (стик)
        ItermRelax relax = ItermRelax::RPY;
        ItermRelaxType relaxType = ItermRelaxType::SETPOINT;
        float relaxCutoffHz = 10.0f;      // ФНЧ уставки для relax
        bool relaxError = true;           // наш: большая ошибка разгона (флип) не копится
        bool itermRotation = false;       // интеграл поворачивается вместе с рамой
        float itermWindup = 1.0f;         // разброс моторов (доля хода), выше которого I копится слабее; 1 — выкл
        float pidSumLimit = 1.0f;         // предел суммы PID крена и тангажа (доля смесителя)
        float pidSumLimitYaw = 1.0f;      // то же рысканья
        float antiGravityI = 8.0f;        // на полном рывке газа I сильнее в 1 + столько раз
        float antiGravityP = 0.0f;        // P крена и тангажа: + ANTI_GRAVITY_P_SHARE × столько на полном рывке
        float antiGravityHz = 5.0f;       // ФНЧ скорости газа для anti-gravity
        float throttleBoost = 0.0f;       // быстрая часть газа (рывок) усиливается в 1 + столько раз
        float throttleBoostHz = 15.0f;    // быстрая часть — газ минус его ФНЧ на этой частоте
        TpaMode tpaMode = TpaMode::MODEL;
        float tpaRate = 0.65f;            // PD, D: на полном газе PD (D) слабее на эту долю
        float tpaBreakpoint = 0.35f;      // газ, выше которого кривая BF начинает ослаблять
        float tpaLowRate = 0.0f;          // на нуле газа PD (D) слабее на эту долю (< 0 — сильнее)
        float tpaLowBreakpoint = 0.05f;   // газ, ниже которого работает tpaLowRate
        bool tpaLowAlways = false;        // false — только до первого подъёма газа выше tpaLowBreakpoint после ARM
        // упреждение (feedforward) — что делать с частью уставки от стика
        float ffTransition = 0.0f;        // ниже такого отклонения стика FF слабее (линейно к центру); 0 — выкл
        uint8_t ffAveraging = 1;          // скорость стика — средняя за столько кадров приёмника; 1 — выкл
        float ffSmoothFactor = 0.0f;      // сглаживание скорости стика между кадрами: доля старого за кадр
        float ffJitterFactor = 0.0f;      // стик сдвинулся меньше (BF-единицы за кадр, 500 — полстика) — FF слабее
        float ffBoost = 0.0f;             // добавка по ускорению стика: FF_BOOST_LEAD_S × столько упреждения
        float ffMaxRateLimit = 0.0f;      // у такой доли наибольшего расхода FF гаснет (нет перелёта на краю); 0 — выкл
        // рысканье через интеграл суммы PID (большие винты: рысканье — разгоном винтов)
        bool integratedYaw = false;
        float integratedYawRelax = 200.0f;   // утечка интеграла, единицы BF (200 — 16 1/с)
        // absolute control: накопленная ошибка угла в ACRO возвращается
        float absGain = 0.0f;             // (°/с) на градус накопленной ошибки; 0 — выкл
        float absLimit = 90.0f;           // предел поправки, °/с
        float absErrorLimit = 20.0f;      // предел накопленной ошибки, °
        float absCutoffHz = 11.0f;        // ФНЧ уставки: что считать быстрым движением стика
    };

    struct CopterTune
    {
        Frame frame;
        Propulsion prop;
        FlyStyle style = FlyStyle::FREESTYLE;
        float volts = 0, massKg = 0, armM = 0, loopHz = 1000;
        uint8_t cells = 0;              // банок в батарее (из описания)

        AxisGains roll, pitch, yaw;
        bool gainsFromModel = true;     // false — PID заданы руками: запасы ниже посчитаны не для них
        float iLimit = 0.3f;            // доля смесителя
        PidExtras pid;
        float gyroLpfHz = 0, dtermLpfHz = 0, yawLpfHz = 0;
        float notchQ = 4.0f;
        float minNotchHz = 80.0f;       // ниже режекторы не опускаются (запас по фазе)
        float levelGain = 8.0f;         // (°/с) на градус ошибки угла
        float maxAngleDeg = 50.0f;
        RateCurve rates[3];
        float idle = 0.055f;            // холостые: доля команды регулятору
        float thrustLinear = 0.25f;
        float motorOutputLimit = 1.0f;  // motor_output_limit: доля хода от холостых до полного
        float hoverThrust = 0.4f;       // доля тяги (выход смесителя) на висении
        float tpa[TPA_POINTS] = {};     // поправка PD по газу из модели (tpa_mode = MODEL)

        // для понимания и проверок
        float thrustToWeight = 0, motorTauS = 0, escDelayS = 0;
        float crossoverHzRP = 0, crossoverHzYaw = 0, phaseMarginDeg = 0, gainMarginDb = 0;
        float rotHzIdle = 0, rotHzHover = 0, rotHzFull = 0;
        float fullAmpsPerMotor = 0, fullWattsPerMotor = 0, hoverAmpsTotal = 0;
        float jRoll = 0, jPitch = 0, jYaw = 0;
        float kRoll = 0, kPitch = 0, kYaw = 0;   // рад/с² на единицу команды
        uint16_t warnings = 0;

        // Команда смесителя (доля тяги) -> скважность регулятора 0..1.
        float motorCommand(float thrustShare) const
        {
            return Mixer::withIdle(Mixer::linearize(thrustShare, thrustLinear), idle, motorOutputLimit);
        }

        // Частота вращения мотора, Гц, при доле тяги и напряжении (0 — номинальное).
        float rotationHz(float thrustShare, float measuredVolts = 0.0f) const
        {
            const float v = measuredVolts > 1.0f ? measuredVolts : volts;
            return prop.steadyOmega(motorCommand(thrustShare), v) / (2.0f * PI_F);
        }

        // Поправка PD по газу (tpa_mode). Малый газ (tpa_low) — отдельно:
        // работает ли она, решает контроллер (был ли газ выше точки после ARM).
        float tpaAt(float throttle) const
        {
            const float g = fminf(1.0f, fmaxf(0.0f, throttle));
            if (pid.tpaMode != TpaMode::MODEL)
            {
                const float bp = pid.tpaBreakpoint;
                return g <= bp || bp >= 1.0f ? 1.0f : 1.0f - pid.tpaRate * (g - bp) / (1.0f - bp);
            }
            const float x = g * (TPA_POINTS - 1);
            const uint8_t i = static_cast<uint8_t>(x);
            if (i >= TPA_POINTS - 1) return tpa[TPA_POINTS - 1];
            const float f = x - i;
            return tpa[i] * (1.0f - f) + tpa[i + 1] * f;
        }

        // tpa_low: ниже точки — от 1 у точки до 1 − tpa_low_rate на нуле газа.
        float tpaLowAt(float throttle) const
        {
            const float bp = pid.tpaLowBreakpoint;
            if (pid.tpaLowRate == 0.0f || bp <= 0.0f || throttle >= bp) return 1.0f;
            return 1.0f - pid.tpaLowRate * (1.0f - fmaxf(0.0f, throttle) / bp);
        }
    };

    inline float escDelaySeconds(EscProtocol p)
    {
        switch (p)
        {
            case EscProtocol::PWM: return 0.0015f;      // кадр 2 мс в среднем ждёт половину + обработка
            case EscProtocol::ONESHOT125: return 0.0004f;
            case EscProtocol::ONESHOT42:
            case EscProtocol::MULTISHOT: return 0.00025f;
            default: return 0.0002f;                    // DShot
        }
    }

    inline float estimateMassG(const BuildSpec& b, const Propulsion& p, uint8_t motors)
    {
        if (b.massG > 0.0f) return b.massG;
        const float wb = b.wheelbaseMm;
        const float frameAndElectronics = 0.0031f * wb * wb + 8.0f;
        const float battery = b.battery.cells * b.battery.capacityMah * 0.0275f;
        return frameAndElectronics + battery + motors * (p.motorMassG + p.propMassG);
    }

    namespace detail
    {
        struct Cx
        {
            float re, im;
            Cx operator*(Cx o) const { return { re * o.re - im * o.im, re * o.im + im * o.re }; }
            Cx operator/(Cx o) const
            {
                const float d = o.re * o.re + o.im * o.im;
                return { (re * o.re + im * o.im) / d, (im * o.re - re * o.im) / d };
            }
            Cx operator+(Cx o) const { return { re + o.re, im + o.im }; }
            float abs() const { return sqrtf(re * re + im * im); }
            float arg() const { return atan2f(im, re); }
        };

        struct LoopShape
        {
            float k, tauM, delayS, gyroHz, dtermHz, outHz, dRatio, iRatio;
            float loopHz = 1000.0f;
            float notchHz[NOTCH_HARMONICS] = {};
            float notchQ = 4.0f;

            // Разомкнутый контур при P = 1: C(jω)·K/(jω(1+jωτ))·e^(−jωT)·фильтры.
            Cx at(float w) const
            {
                const Cx jw = { 0.0f, w };
                const Cx one = { 1.0f, 0.0f };
                const float wd = 2.0f * PI_F * dtermHz;
                Cx c = one + Cx{ 0.0f, -iRatio };   // I: ωI = ω·iRatio -> ωI/(jω) = −j·iRatio
                if (dRatio > 0.0f)
                {
                    // производная — разность соседних отсчётов: ещё полцикла задержки
                    const float half = 0.5f / loopHz;
                    const Cx dDelay = { cosf(-w * half), sinf(-w * half) };
                    c = c + (Cx{ 0.0f, w * dRatio * tauM } / (one + Cx{ 0.0f, w / wd })) * dDelay;
                }
                Cx plant = Cx{ k, 0.0f } / (jw * (one + Cx{ 0.0f, w * tauM }));
                const float wg = 2.0f * PI_F * gyroHz * 1.553774f;
                const Cx g = one + Cx{ 0.0f, w / wg };
                plant = plant / (g * g);
                for (uint8_t h = 0; h < NOTCH_HARMONICS; ++h)
                {
                    if (notchHz[h] <= 0.0f) continue;
                    const float x = w / (2.0f * PI_F * notchHz[h]);
                    const float re = 1.0f - x * x;
                    plant = plant * (Cx{ re, 0.0f } / Cx{ re, x / notchQ });
                }
                if (outHz > 0.0f) plant = plant / (one + Cx{ 0.0f, w / (2.0f * PI_F * outHz) });
                const Cx delay = { cosf(-w * delayS), sinf(-w * delayS) };
                return c * plant * delay;
            }

            // Фаза с развёрткой (непрерывная по частоте), градусы.
            float phaseDeg(float w) const
            {
                const int steps = 64;
                float prev = at(0.5f).arg(), total = prev;
                for (int i = 1; i <= steps; ++i)
                {
                    const float wi = 0.5f * powf(w / 0.5f, static_cast<float>(i) / steps);
                    const float a = at(wi).arg();
                    float d = a - prev;
                    while (d > PI_F) d -= 2.0f * PI_F;
                    while (d < -PI_F) d += 2.0f * PI_F;
                    total += d;
                    prev = a;
                }
                return total * 180.0f / PI_F;
            }
        };

        // Частота среза (рад/с) под запасы по фазе и по усилению, P (на рад/с), итоговые запасы.
        inline void shapeLoop(const LoopShape& s, float pmTargetDeg, float gmTargetDb, float wMax, float& wc, float& p, float& pm,
                              float& gmDb)
        {
            // наибольшая частота с запасом по фазе не меньше цели
            float lo = 1.0f, hi = wMax;
            if (180.0f + s.phaseDeg(lo) < pmTargetDeg) hi = lo;
            for (int it = 0; it < 40 && hi > lo * 1.001f; ++it)
            {
                const float mid = sqrtf(lo * hi);
                (180.0f + s.phaseDeg(mid) >= pmTargetDeg ? lo : hi) = mid;
            }
            wc = lo;

            // частота, где фаза −180°: от P не зависит
            float w180 = 0.0f;
            float a = wc, b = wMax * 8.0f;
            if (s.phaseDeg(b) < -180.0f)
            {
                for (int it = 0; it < 40; ++it)
                {
                    const float mid = sqrtf(a * b);
                    (s.phaseDeg(mid) > -180.0f ? a : b) = mid;
                }
                w180 = a;
            }

            // запас по усилению = |L(wc)| / |L(w180)| (при P = 1/|L(wc)|): падает с ростом wc
            if (w180 > 0.0f)
            {
                const float need = powf(10.0f, gmTargetDb / 20.0f);
                const float at180 = s.at(w180).abs();
                if (s.at(wc).abs() / at180 < need)
                {
                    float l = 0.5f, h = wc;
                    for (int it = 0; it < 40; ++it)
                    {
                        const float mid = sqrtf(l * h);
                        (s.at(mid).abs() / at180 >= need ? l : h) = mid;
                    }
                    wc = l;
                }
            }
            p = 1.0f / s.at(wc).abs();
            pm = 180.0f + s.phaseDeg(wc);
            gmDb = w180 > 0.0f ? -20.0f * log10f(p * s.at(w180).abs()) : 60.0f;
        }
    }

    // Запас по усилению контура угловой скорости. Модель знает коптер с
    // точностью ±30 % по тяге винта и инерции рамы — до 2× по усилению
    // контура (6 дБ). 12 дБ оставляют ещё 6 дБ и на такой «не тот» коптер:
    // проверено симуляцией на всём каталоге (test_copter_sim).
    constexpr float GAIN_MARGIN_DB = 12.0f;

    // Настройка по описанию сборки. loopHz — частота цикла стабилизации.
    inline CopterTune deriveTune(const BuildSpec& b, float loopHz, EscProtocol esc)
    {
        CopterTune t;
        t.frame = buildFrame(b.frame, b.propsOut);
        t.prop = Propulsion::from(b.prop, b.motor);
        t.style = b.style;
        t.loopHz = loopHz;
        t.volts = b.battery.cells * b.battery.voltsPerCellLoaded;
        t.cells = b.battery.cells;
        const uint8_t n = t.frame.count;
        t.massKg = estimateMassG(b, t.prop, n) * 0.001f;
        t.armM = b.wheelbaseMm * 0.0005f;
        t.escDelayS = escDelaySeconds(esc);

        const float d = b.prop.diameterIn;
        t.idle = d <= 2.0f ? 0.07f : (d <= 6.5f ? 0.055f : 0.045f);
        t.thrustLinear = d < 3.0f ? 0.3f : (d <= 6.5f ? 0.25f : (d <= 9.0f ? 0.2f : 0.15f));

        auto omegaAt = [&](float m) { return t.prop.steadyOmega(t.motorCommand(m), t.volts); };
        auto thrustAt = [&](float m) { return t.prop.thrust(omegaAt(m)); };
        auto torqueAt = [&](float m) { return t.prop.torque(omegaAt(m)); };

        // висение
        const float weight = t.massKg * GRAVITY;
        const float maxThrust = n * thrustAt(1.0f);
        t.thrustToWeight = maxThrust / weight;
        if (maxThrust <= weight)
        {
            t.hoverThrust = 1.0f;
        }
        else
        {
            float lo = 0.0f, hi = 1.0f;
            for (int it = 0; it < 30; ++it)
            {
                const float mid = 0.5f * (lo + hi);
                (n * thrustAt(mid) < weight ? lo : hi) = mid;
            }
            t.hoverThrust = 0.5f * (lo + hi);
        }
        const float mh = t.hoverThrust;
        const float wHover = omegaAt(mh);
        const float dm = 0.01f;
        const float m0 = fmaxf(0.0f, mh - dm), m1 = fminf(1.0f, mh + dm);
        const float sT = (thrustAt(m1) - thrustAt(m0)) / (m1 - m0);
        const float sQ = (torqueAt(m1) - torqueAt(m0)) / (m1 - m0);

        t.rotHzIdle = omegaAt(0.0f) / (2.0f * PI_F);
        t.rotHzHover = wHover / (2.0f * PI_F);
        t.rotHzFull = omegaAt(1.0f) / (2.0f * PI_F);
        t.motorTauS = t.prop.timeConstant(wHover);
        const float wFull = omegaAt(1.0f);
        t.fullAmpsPerMotor = t.prop.current(t.motorCommand(1.0f), t.volts, wFull);
        t.fullWattsPerMotor = t.fullAmpsPerMotor * t.motorCommand(1.0f) * t.volts;
        t.hoverAmpsTotal = n * t.prop.current(t.motorCommand(mh), t.volts, wHover) * t.motorCommand(mh);

        // моменты инерции
        const float mMotor = (t.prop.motorMassG + t.prop.propMassG) * 0.001f;
        const float mCore = fmaxf(0.3f * t.massKg, t.massKg - n * mMotor);
        const float r = t.armM;
        float sx2 = 0, sy2 = 0, maxX = 0, maxY = 0;
        for (uint8_t i = 0; i < n; ++i)
        {
            const MotorPlace& pl = t.frame.place[i];
            sx2 += pl.x * pl.x;
            sy2 += pl.y * pl.y;
            maxX = fmaxf(maxX, fabsf(pl.x));
            maxY = fmaxf(maxY, fabsf(pl.y));
        }
        t.jRoll = mMotor * sy2 * r * r + 0.08f * mCore * r * r;
        t.jPitch = mMotor * sx2 * r * r + 0.08f * mCore * r * r;
        t.jYaw = mMotor * (sx2 + sy2) * r * r + 0.06f * mCore * r * r;
        t.kRoll = sT * r * sy2 / maxY / t.jRoll;
        t.kPitch = sT * r * sx2 / maxX / t.jPitch;
        t.kYaw = sQ * n / t.jYaw;

        // фильтры
        const float nyq = 0.5f * loopHz;
        t.gyroLpfHz = fminf(0.4f * loopHz, fmaxf(60.0f, 1.3f * t.rotHzHover));
        t.dtermLpfHz = fminf(0.3f * loopHz, fmaxf(40.0f, 0.75f * t.gyroLpfHz));
        t.yawLpfHz = fminf(100.0f, fmaxf(30.0f, 0.5f * t.gyroLpfHz));

        // контур угловой скорости
        // цикл (выборка + расчёт) + регулятор + фильтр в самом чипе гироскопа (~0.5 мс)
        const float delay = 1.5f / loopHz + t.escDelayS + 0.0005f;
        const float wMax = 2.0f * PI_F * 0.12f * loopHz;
        const float iRatio = 1.0f / 6.0f;
        // Доля D (в долях τ мотора) подбирается: больше D — больше фазы у
        // среза, но и усиления на высоких частотах (запас по усилению).
        // Берётся та, при которой частота среза выше всего.
        const float dRatios[] = { 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.85f, 1.0f };
        float notches[NOTCH_HARMONICS] = {};
        auto axis = [&](float k, bool isYaw, float wCap, AxisGains& g, float& wcOut, float& pmOut, float& gmOut) {
            float wc = 0, p = 0, pm = 0, gm = 0, dRatio = 0;
            for (float dr : dRatios)
            {
                if (isYaw && dr > dRatios[0]) break;
                const float dSel = isYaw ? 0.0f : dr;
                detail::LoopShape s{ k, t.motorTauS, delay, t.gyroLpfHz, t.dtermLpfHz, isYaw ? t.yawLpfHz : 0.0f, dSel, iRatio };
                s.loopHz = loopHz;
                s.notchQ = t.notchQ;
                for (uint8_t h = 0; h < NOTCH_HARMONICS; ++h) s.notchHz[h] = notches[h];
                float wcI, pI, pmI, gmI;
                detail::shapeLoop(s, isYaw ? 55.0f : 50.0f, GAIN_MARGIN_DB, wCap, wcI, pI, pmI, gmI);
                if (wcI > wc * 1.02f)
                {
                    wc = wcI;
                    p = pI;
                    pm = pmI;
                    gm = gmI;
                    dRatio = dSel;
                }
            }
            const float toDeg = PI_F / 180.0f;
            g.p = p * toDeg;
            g.i = p * wc * iRatio * toDeg;
            g.d = p * dRatio * t.motorTauS * toDeg;
            g.ff = 0.7f / k * toDeg;
            wcOut = wc;
            pmOut = pm;
            gmOut = gm;
        };
        float wcR, wcP, wcY, pmR, pmP, pmY, gmR, gmP, gmY;
        // Режекторы оборотов съедают фазу у полосы контура: сначала срез без
        // них, режекторы — не ниже 5 частот среза, и контур — уже с ними.
        axis(t.kRoll, false, wMax, t.roll, wcR, pmR, gmR);
        axis(t.kPitch, false, wMax, t.pitch, wcP, pmP, gmP);
        t.minNotchHz = fmaxf(30.0f, 5.0f * fminf(wcR, wcP) / (2.0f * PI_F));
        notchCenters(t.rotHzHover, b.prop.blades, t.minNotchHz, notches);
        axis(t.kRoll, false, wMax, t.roll, wcR, pmR, gmR);
        axis(t.kPitch, false, wMax, t.pitch, wcP, pmP, gmP);
        axis(t.kYaw, true, 0.6f * fminf(wcR, wcP), t.yaw, wcY, pmY, gmY);
        t.crossoverHzRP = fminf(wcR, wcP) / (2.0f * PI_F);
        t.crossoverHzYaw = wcY / (2.0f * PI_F);
        t.phaseMarginDeg = fminf(pmR, fminf(pmP, pmY));
        t.gainMarginDb = fminf(gmR, fminf(gmP, gmY));

        // TPA по модели
        for (uint8_t i = 0; i < TPA_POINTS; ++i)
        {
            const float m = fmaxf(0.03f, fminf(0.97f, i / float(TPA_POINTS - 1)));
            const float s = (thrustAt(m + 0.02f) - thrustAt(m - 0.02f)) / 0.04f;
            t.tpa[i] = s > 0.0f ? fminf(1.5f, fmaxf(0.6f, sT / s)) : 1.0f;
        }

        // уровень, расходы
        t.levelGain = fminf(12.0f, fmaxf(3.0f, fminf(wcR, wcP) / 7.0f));
        switch (b.style)
        {
            case FlyStyle::INDOOR:
                t.maxAngleDeg = 45;
                t.rates[0] = t.rates[1] = { 200, 800, 0.4f };
                t.rates[2] = { 180, 600, 0.3f };
                break;
            case FlyStyle::RACE:
                t.maxAngleDeg = 60;
                t.rates[0] = t.rates[1] = { 250, 800, 0.3f };
                t.rates[2] = { 220, 650, 0.3f };
                break;
            case FlyStyle::CINEMATIC:
                t.maxAngleDeg = 35;
                t.rates[0] = t.rates[1] = { 120, 360, 0.4f };
                t.rates[2] = { 90, 240, 0.3f };
                break;
            case FlyStyle::LONG_RANGE:
                t.maxAngleDeg = 40;
                t.rates[0] = t.rates[1] = { 140, 500, 0.4f };
                t.rates[2] = { 120, 360, 0.3f };
                break;
            case FlyStyle::HEAVY_LIFT:
                t.maxAngleDeg = 30;
                t.rates[0] = t.rates[1] = { 60, 180, 0.3f };
                t.rates[2] = { 45, 120, 0.3f };
                break;
            case FlyStyle::FREESTYLE:
            default:
                t.maxAngleDeg = 55;
                t.rates[0] = t.rates[1] = { 180, 720, 0.5f };
                t.rates[2] = { 160, 600, 0.4f };
                break;
        }

        // предупреждения
        if (t.thrustToWeight < 1.2f) t.warnings |= WARN_NO_LIFT_MARGIN;
        else if (t.thrustToWeight < 1.8f) t.warnings |= WARN_LOW_THRUST;
        // полный газ — рывками, и длительную мощность моторы 5" превышают в 1.5 раза штатно
        if (t.fullWattsPerMotor > 2.0f * t.prop.ratedPowerW) t.warnings |= WARN_MOTOR_OVERLOAD;
        if (t.hoverThrust > 0.6f) t.warnings |= WARN_HEAVY_HOVER;
        if (t.rotHzFull > nyq) t.warnings |= WARN_VIBRATION_ALIAS;
        float nearest = 1e9f;
        for (uint8_t i = 0; i < n; ++i)
        {
            for (uint8_t j = i + 1; j < n; ++j)
            {
                const float dx = t.frame.place[i].x - t.frame.place[j].x;
                const float dy = t.frame.place[i].y - t.frame.place[j].y;
                const float dist = sqrtf(dx * dx + dy * dy) * r;
                if (dist > 1e-4f) nearest = fminf(nearest, dist);   // соосные пары — на одной оси
            }
        }
        if (b.prop.diameterIn * 0.0254f > nearest) t.warnings |= WARN_PROP_OVERLAP;
        return t;
    }

    // ---------------- единицы и дефолты Betaflight ----------------
    //
    // Числа P/I/D/F в Betaflight — не «команда на °/с», а условные единицы:
    // член PID = число × масштаб × ошибка, сумма делится на 1000 и идёт в
    // смеситель (1 — весь ход мотора от холостых до полного, как и у нас).
    // Масштабы и дефолты — из документации и исходников BF 4.5 (это числа,
    // а не код): P 0.032029, I 0.244381 (у рысканья ещё ×2.5, если нет
    // integrated yaw), D 0.000529 (на °/с²), F 0.013754 × 0.01 (на °/с²
    // уставки).

    enum class PidTerm : uint8_t
    {
        P,
        I,
        D,
        F,
    };

    namespace betaflight
    {
        constexpr float P_SCALE = 0.032029f;
        constexpr float I_SCALE = 0.244381f;
        constexpr float D_SCALE = 0.000529f;
        constexpr float F_SCALE = 0.013754f * 0.01f;
        constexpr float YAW_I_FACTOR = 2.5f;
        constexpr float MIXER_SCALING = 1000.0f;
        constexpr uint8_t AXIS_YAW = 2;

        // BF 4.5: pid_roll/pitch/yaw = P, I, D, F
        constexpr uint16_t DEFAULT_PID[3][4] = { { 45, 80, 40, 120 }, { 47, 84, 46, 125 }, { 45, 80, 0, 120 } };
        // rates_type = ACTUAL: центр 70 °/с, максимум 670 °/с, экспо 0
        constexpr uint8_t DEFAULT_RC_RATE = 7, DEFAULT_SRATE = 67, DEFAULT_EXPO = 0;
        // BF 4.5: d_min_roll/pitch/yaw — D в покое (D из таблицы — на резком
        // движении), d_min_gain 37, d_min_advance 20; I: iterm_relax RP,
        // SETPOINT, срез 15 Гц, iterm_windup 85 %, iterm_limit 400
        constexpr uint8_t DEFAULT_D_MIN[3] = { 30, 34, 0 };
        constexpr uint8_t DEFAULT_ITERM_RELAX_CUTOFF = 15, DEFAULT_ITERM_WINDUP = 85;
        constexpr uint16_t DEFAULT_ITERM_LIMIT = 400;
        // pidsum_limit 500, pidsum_limit_yaw 400; anti_gravity_gain 80,
        // cutoff 5 Гц, p_gain 100; throttle_boost 5, cutoff 15 Гц;
        // TPA: D, 65 % от 1350; tpa_low 20 % ниже 1050, только до взлёта
        constexpr uint16_t DEFAULT_PIDSUM_LIMIT = 500, DEFAULT_PIDSUM_LIMIT_YAW = 400;
        constexpr uint8_t DEFAULT_ANTI_GRAVITY_GAIN = 80, DEFAULT_ANTI_GRAVITY_CUTOFF = 5, DEFAULT_ANTI_GRAVITY_P_GAIN = 100;
        constexpr uint8_t DEFAULT_THROTTLE_BOOST = 5, DEFAULT_THROTTLE_BOOST_CUTOFF = 15;
        constexpr uint8_t DEFAULT_TPA_RATE = 65, DEFAULT_TPA_LOW_RATE = 20;
        constexpr uint16_t DEFAULT_TPA_BREAKPOINT = 1350, DEFAULT_TPA_LOW_BREAKPOINT = 1050;
        // feedforward: averaging OFF, smooth_factor 25, jitter_factor 7,
        // boost 15, transition 0
        constexpr uint8_t DEFAULT_FF_SMOOTH_FACTOR = 25, DEFAULT_FF_JITTER_FACTOR = 7, DEFAULT_FF_BOOST = 15;
        // feedforward_max_rate_limit 90 %, yaw_lowpass_hz 100, thrust_linear 0
        constexpr uint8_t DEFAULT_FF_MAX_RATE_LIMIT = 90, DEFAULT_YAW_LOWPASS_HZ = 100, DEFAULT_THRUST_LINEAR = 0;

        // Газ в мкс (1000–2000) -> доля 0..1 и обратно.
        inline float throttleFromUs(float us) { return fminf(1.0f, fmaxf(0.0f, (us - 1000.0f) / 1000.0f)); }
        inline float usFromThrottle(float g) { return 1000.0f + 1000.0f * g; }
    }

    inline float pidTermScale(PidTerm term, uint8_t axis)
    {
        float s = betaflight::P_SCALE;
        switch (term)
        {
            case PidTerm::P: s = betaflight::P_SCALE; break;
            case PidTerm::I: s = betaflight::I_SCALE * (axis == betaflight::AXIS_YAW ? betaflight::YAW_I_FACTOR : 1.0f); break;
            case PidTerm::D: s = betaflight::D_SCALE; break;
            case PidTerm::F: s = betaflight::F_SCALE; break;
        }
        return s / betaflight::MIXER_SCALING;
    }

    // Число BF -> наш коэффициент (ось 0 крен, 1 тангаж, 2 рысканье) и обратно.
    inline float bfToGain(PidTerm term, uint8_t axis, float value) { return value * pidTermScale(term, axis); }
    inline float gainToBf(PidTerm term, uint8_t axis, float gain) { return gain / pidTermScale(term, axis); }

    inline float& gainTerm(AxisGains& g, PidTerm term)
    {
        switch (term)
        {
            case PidTerm::I: return g.i;
            case PidTerm::D: return g.d;
            case PidTerm::F: return g.ff;
            case PidTerm::P:
            default: return g.p;
        }
    }

    inline float gainTerm(const AxisGains& g, PidTerm term)
    {
        switch (term)
        {
            case PidTerm::I: return g.i;
            case PidTerm::D: return g.d;
            case PidTerm::F: return g.ff;
            case PidTerm::P:
            default: return g.p;
        }
    }

    // Всё, кроме PID и расходов, — как в BF 4.5 по умолчанию, в пересчёте на
    // нашу цепочку фильтров:
    //   • гироскоп: у BF два PT1 (динамический 250–500 Гц и 500 Гц) — у нас
    //     PT2 250 Гц: то же запаздывание на 50 Гц (~15°);
    //   • D: у BF два PT1 (динамический 75–150 Гц и 150 Гц) — у нас один PT1
    //     60 Гц: то же запаздывание у полосы контура (~19° на 20 Гц) и то же
    //     ослабление шума на 200 Гц (~0.28);
    //   • thrust_linear 0 (линеаризации тяги нет), feedforward_max_rate_limit 90 %;
    //   • рысканье: yaw_lowpass_hz 100; TPA только на D, 65 % от газа 1350
    //     (35 % хода) до полного, на малом газе −20 % ниже 1050 до взлёта;
    //     iterm_limit 400; ANGLE: angle_p_gain 50 (5 °/с на градус),
    //     angle_limit 60°;
    //   • D 30/34 в покое и до 40/46 на резком движении (d_min), I-relax RP
    //     15 Гц, iterm_windup 85 %, сумма PID ±0.5 (рысканье ±0.4),
    //     anti-gravity 80/5 Гц/100, throttle_boost 5; feedforward:
    //     smooth_factor 25, jitter_factor 7, boost 15. PID и D в покое
    //     (d_min) — из реестра настроек (Params.h), там же их дефолты BF.
    // Режекторы по оборотам остаются наши (у BF на их месте RPM-фильтр по
    // телеметрии DShot и динамический режектор по спектру).
    inline void applyBetaflightBase(CopterTune& t)
    {
        namespace bf = betaflight;
        t.gyroLpfHz = fminf(0.4f * t.loopHz, 250.0f);
        t.dtermLpfHz = fminf(0.3f * t.loopHz, 60.0f);
        t.yawLpfHz = bf::DEFAULT_YAW_LOWPASS_HZ;
        t.thrustLinear = bf::DEFAULT_THRUST_LINEAR / 100.0f;
        t.iLimit = bf::DEFAULT_ITERM_LIMIT / bf::MIXER_SCALING;
        t.pid.relax = ItermRelax::RP;
        t.pid.relaxType = ItermRelaxType::SETPOINT;
        t.pid.relaxCutoffHz = bf::DEFAULT_ITERM_RELAX_CUTOFF;
        t.pid.itermWindup = bf::DEFAULT_ITERM_WINDUP / 100.0f;
        t.pid.pidSumLimit = bf::DEFAULT_PIDSUM_LIMIT / bf::MIXER_SCALING;
        t.pid.pidSumLimitYaw = bf::DEFAULT_PIDSUM_LIMIT_YAW / bf::MIXER_SCALING;
        t.pid.antiGravityI = bf::DEFAULT_ANTI_GRAVITY_GAIN / 10.0f;
        t.pid.antiGravityHz = bf::DEFAULT_ANTI_GRAVITY_CUTOFF;
        t.pid.antiGravityP = bf::DEFAULT_ANTI_GRAVITY_P_GAIN / 100.0f;
        t.pid.throttleBoost = bf::DEFAULT_THROTTLE_BOOST / 10.0f;
        t.pid.throttleBoostHz = bf::DEFAULT_THROTTLE_BOOST_CUTOFF;
        t.pid.tpaMode = TpaMode::D;
        t.pid.tpaRate = bf::DEFAULT_TPA_RATE / 100.0f;
        t.pid.tpaBreakpoint = bf::throttleFromUs(bf::DEFAULT_TPA_BREAKPOINT);
        t.pid.tpaLowRate = bf::DEFAULT_TPA_LOW_RATE / 100.0f;
        t.pid.tpaLowBreakpoint = bf::throttleFromUs(bf::DEFAULT_TPA_LOW_BREAKPOINT);
        t.pid.tpaLowAlways = false;
        t.pid.ffSmoothFactor = bf::DEFAULT_FF_SMOOTH_FACTOR / 100.0f;
        t.pid.ffJitterFactor = bf::DEFAULT_FF_JITTER_FACTOR;
        t.pid.ffBoost = bf::DEFAULT_FF_BOOST;
        t.pid.ffMaxRateLimit = bf::DEFAULT_FF_MAX_RATE_LIMIT / 100.0f;
        t.levelGain = 5.0f;
        t.maxAngleDeg = 60.0f;
        t.gainsFromModel = false;
    }
}
