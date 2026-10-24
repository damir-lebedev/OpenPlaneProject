#pragma once
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "copter/Battery.h"
#include "copter/Presets.h"
#include "copter/RateController.h"
#include "copter/RcSetup.h"
#include "copter/Vtx.h"
#include "storage/KeyValueStore.h"

// ============================================================
// Реестр настроек коптера — то, что в Betaflight задаётся строкой
// `set имя = значение` в CLI.
//
// У каждой настройки: имя как в BF, тип (целое, дробное, выбор из
// списка, строка), границы, значение по умолчанию, единицы, группа
// (= вкладка приложения) и описание по-русски одной строкой. Значения —
// в единицах BF, где они у нас имеют смысл (P/I/D/F, расходы).
//
// Где хранится:
//   • общие (MASTER) — одно значение;
//   • PID-профиль и профиль расходов — по 4 значения (profile 0..3,
//     rateprofile 0..3, как в BF), работает выбранный профиль.
// Настройка, которую не меняли, берёт умолчание — а оно у многих не
// число из таблицы, а вывод из описания коптера (PID, расходы — по модели
// и стилю полёта) или дефолт BF 4.5 (tune_source = BETAFLIGHT). Поэтому
// смена сборки или источника двигает всё, что пилот не трогал, а то, что
// он задал руками, остаётся. Задать значение, равное умолчанию, — то же,
// что вернуть умолчание (в diff его нет, как в BF).
//
// Описание коптера (copter_build, масса, винт, мотор, батарея) — тоже
// настройки: умолчание — сборка из каталога с поправками «своего
// коптера» из CopterConfig.h, заданное в CLI — поверх. Сменили — модель
// (PID, фильтры) пересчитывается при save (updateModel()).
//
// Во флеш (KeyValueStore, пространство "copter", ключ "params") пишутся
// только заданные руками значения: [версия][профиль][профиль расходов],
// затем записи [хеш имени u16][профиль u8][значение f32] — строки
// [хеш][0x80|профиль][длина][символы]. Хеш, а не номер, — чтобы новые
// настройки и перестановка таблицы не сдвигали сохранённые; настройку,
// которой больше нет, загрузка пропускает, значение за границами —
// прижимает к ним. Таблицы пульта (map, rxrange, aux — RcSetup.h) — ключ
// "rc".
//
// Код переносимый (без Arduino) — тестируется на ПК (test_copter).
// ============================================================

namespace copter
{
    enum class ParamType : uint8_t
    {
        INT,
        FLOAT,
        ENUM,
        STRING,
    };

    enum class ParamScope : uint8_t
    {
        MASTER,
        PID_PROFILE,
        RATE_PROFILE,
    };

    // Группа = вкладка приложения.
    enum class ParamGroup : uint8_t
    {
        PID,
        RATES,
        FILTERS,
        MOTORS,
        SAFETY,
        RECEIVER,
        MODES,
        COPTER,
        POWER,
        VTX,
        GPS,
        BLACKBOX,
        OTHER,
    };

    inline const char* paramGroupName(ParamGroup g)
    {
        switch (g)
        {
            case ParamGroup::PID: return "pid";
            case ParamGroup::RATES: return "rates";
            case ParamGroup::FILTERS: return "filters";
            case ParamGroup::MOTORS: return "motors";
            case ParamGroup::SAFETY: return "safety";
            case ParamGroup::RECEIVER: return "receiver";
            case ParamGroup::MODES: return "modes";
            case ParamGroup::COPTER: return "copter";
            case ParamGroup::POWER: return "power";
            case ParamGroup::VTX: return "vtx";
            case ParamGroup::GPS: return "gps";
            case ParamGroup::BLACKBOX: return "blackbox";
            case ParamGroup::OTHER: return "other";
        }
        return "other";
    }

    enum class TuneSource : uint8_t
    {
        MODEL,        // PID, фильтры, TPA — из описания коптера (наш вывод)
        BETAFLIGHT,   // дефолты Betaflight 4.5
    };

    // Откуда умолчание настройки.
    enum class ParamDefault : uint8_t
    {
        TABLE,   // число из таблицы (прошивка может заменить: setTableDefault)
        TUNE,    // вывод из модели коптера или дефолт BF 4.5 (tune_source)
        BUILD,   // из сборки copter_build с поправками CopterConfig.h
        BATTERY, // по химии батареи (battery_chemistry)
    };

    // Сборки каталога по именам — варианты copter_build.
    inline const char* const* buildIds(uint8_t& count)
    {
        static const char* ids[24] = {};
        static uint8_t n = 0;
        if (n == 0)
        {
            uint8_t total = 0;
            const BuildSpec* c = buildCatalog(total);
            for (uint8_t i = 0; i < total && i < 24; ++i) ids[n++] = c[i].id;
        }
        count = n;
        return ids;
    }

    // Порядок — как в таблице paramTable(): P, I, D, F по осям подряд.
    enum ParamId : uint8_t
    {
        PARAM_TUNE_SOURCE,
        PARAM_PROFILE_NAME,
        PARAM_P_ROLL,
        PARAM_I_ROLL,
        PARAM_D_ROLL,
        PARAM_F_ROLL,
        PARAM_P_PITCH,
        PARAM_I_PITCH,
        PARAM_D_PITCH,
        PARAM_F_PITCH,
        PARAM_P_YAW,
        PARAM_I_YAW,
        PARAM_D_YAW,
        PARAM_F_YAW,
        PARAM_D_MAX_ROLL,
        PARAM_D_MAX_PITCH,
        PARAM_D_MAX_YAW,
        PARAM_D_MAX_GAIN,
        PARAM_D_MAX_ADVANCE,
        PARAM_D_MIN_ROLL,
        PARAM_D_MIN_PITCH,
        PARAM_D_MIN_YAW,
        PARAM_ITERM_RELAX,
        PARAM_ITERM_RELAX_TYPE,
        PARAM_ITERM_RELAX_CUTOFF,
        PARAM_ITERM_RELAX_ERROR,
        PARAM_ITERM_ROTATION,
        PARAM_ITERM_WINDUP,
        PARAM_ITERM_LIMIT,
        PARAM_PIDSUM_LIMIT,
        PARAM_PIDSUM_LIMIT_YAW,
        PARAM_ANTI_GRAVITY_GAIN,
        PARAM_ANTI_GRAVITY_CUTOFF_HZ,
        PARAM_ANTI_GRAVITY_P_GAIN,
        PARAM_THROTTLE_BOOST,
        PARAM_THROTTLE_BOOST_CUTOFF,
        PARAM_TPA_MODE,
        PARAM_TPA_RATE,
        PARAM_TPA_BREAKPOINT,
        PARAM_TPA_LOW_RATE,
        PARAM_TPA_LOW_BREAKPOINT,
        PARAM_TPA_LOW_ALWAYS,
        PARAM_FEEDFORWARD_TRANSITION,
        PARAM_FEEDFORWARD_AVERAGING,
        PARAM_FEEDFORWARD_SMOOTH_FACTOR,
        PARAM_FEEDFORWARD_JITTER_FACTOR,
        PARAM_FEEDFORWARD_BOOST,
        PARAM_FEEDFORWARD_MAX_RATE_LIMIT,
        PARAM_YAW_LOWPASS_HZ,
        PARAM_USE_INTEGRATED_YAW,
        PARAM_INTEGRATED_YAW_RELAX,
        PARAM_ABS_CONTROL_GAIN,
        PARAM_ABS_CONTROL_LIMIT,
        PARAM_ABS_CONTROL_ERROR_LIMIT,
        PARAM_ABS_CONTROL_CUTOFF,
        PARAM_THRUST_LINEAR,
        PARAM_MOTOR_OUTPUT_LIMIT,
        PARAM_RATEPROFILE_NAME,
        PARAM_ROLL_RC_RATE,
        PARAM_PITCH_RC_RATE,
        PARAM_YAW_RC_RATE,
        PARAM_ROLL_EXPO,
        PARAM_PITCH_EXPO,
        PARAM_YAW_EXPO,
        PARAM_ROLL_SRATE,
        PARAM_PITCH_SRATE,
        PARAM_YAW_SRATE,
        PARAM_SERIALRX_PROVIDER,
        PARAM_RX_MIN_USEC,
        PARAM_RX_MAX_USEC,
        PARAM_COPTER_BUILD,
        PARAM_COPTER_MASS_G,
        PARAM_COPTER_WHEELBASE_MM,
        PARAM_PROP_DIAMETER_IN,
        PARAM_PROP_PITCH_IN,
        PARAM_PROP_BLADES,
        PARAM_MOTOR_STATOR_MM,
        PARAM_MOTOR_HEIGHT_MM,
        PARAM_MOTOR_KV,
        PARAM_BATTERY_CELLS,
        PARAM_BATTERY_MAH,
        PARAM_PROPS_OUT,
        PARAM_BATTERY_CHEMISTRY,
        PARAM_VBAT_MIN_CELL_VOLTAGE,
        PARAM_VBAT_WARNING_CELL_VOLTAGE,
        PARAM_VBAT_MAX_CELL_VOLTAGE,
        PARAM_VBAT_SCALE,
        PARAM_CURRENT_METER,
        PARAM_IBATA_SCALE,
        PARAM_IBATA_OFFSET,
        PARAM_VTX_PROTOCOL,
        PARAM_VTX_BAND,
        PARAM_VTX_CHANNEL,
        PARAM_VTX_FREQ,
        PARAM_VTX_POWER,
        PARAM_VTX_MAX_POWER_MW,
        PARAM_VTX_LOW_POWER_DISARM,
        PARAM_GPS_MINIMUM_SATS,
        PARAM_CRSF_TELEMETRY,
        PARAM_BLACKBOX_DEVICE,
        PARAM_BLACKBOX_MODE,
        PARAM_BLACKBOX_SAMPLE_RATE,
        PARAM_COUNT
    };

    struct ParamInfo
    {
        const char* name;            // как в CLI Betaflight
        ParamType type;
        ParamScope scope;
        ParamGroup group;
        float min, max;              // STRING: max — наибольшая длина
        float def;                   // умолчание для ParamDefault::TABLE
        ParamDefault from;
        const char* units;
        const char* const* options;  // ENUM: имена значений 0, 1, …
        uint8_t optionCount;
        const char* text;            // что это, по-русски
    };

    inline const ParamInfo* paramTable()
    {
        static const char* const TUNE_SOURCES[] = { "MODEL", "BETAFLIGHT" };
        // порядок — как Config::RxProtocol (static_assert в main_copter.cpp)
        static const char* const RX_PROVIDERS[] = { "IBUS", "SBUS", "CRSF" };
        static const char* const OFF_ON[] = { "OFF", "ON" };
        // порядок — как BatteryChemistry
        static const char* const CHEMISTRIES[] = { "LIPO", "LIHV", "LIION" };
        static const char* const CURRENT_METERS[] = { "NONE", "ADC" };
        // порядок — как VtxProtocol
        static const char* const VTX_PROTOCOLS[] = { "AUTO", "SMARTAUDIO", "TRAMP", "NONE" };
        static const char* const LOW_POWER_DISARM[] = { "OFF", "ON", "UNTIL_FIRST_ARM" };
        // порядок — как ItermRelax и ItermRelaxType (Presets.h)
        static const char* const ITERM_RELAX[] = { "OFF", "RP", "RPY", "RP_INC", "RPY_INC" };
        static const char* const ITERM_RELAX_TYPES[] = { "GYRO", "SETPOINT" };
        static const char* const TPA_MODES[] = { "PD", "D", "MODEL" };   // как TpaMode
        static const char* const FF_AVERAGING[] = { "OFF", "2_POINT", "3_POINT", "4_POINT" };
        static const char* const BLACKBOX_DEVICES[] = { "NONE", "SDCARD" };
        static const char* const BLACKBOX_MODES[] = { "NORMAL", "MOTOR_TEST", "ALWAYS" };
        static const char* const BLACKBOX_RATES[] = { "1/1", "1/2", "1/4", "1/8", "1/16", "1/32" };
        uint8_t buildCount = 0;
        const char* const* builds = buildIds(buildCount);
        constexpr ParamType kInt = ParamType::INT, kFloat = ParamType::FLOAT, kEnum = ParamType::ENUM,
                            kText = ParamType::STRING;
        constexpr ParamScope kMaster = ParamScope::MASTER, kPid = ParamScope::PID_PROFILE, kRate = ParamScope::RATE_PROFILE;
        constexpr ParamGroup kGroupPid = ParamGroup::PID, kGroupRates = ParamGroup::RATES, kGroupRx = ParamGroup::RECEIVER,
                             kGroupCopter = ParamGroup::COPTER, kGroupPower = ParamGroup::POWER,
                             kGroupVtx = ParamGroup::VTX, kGroupGps = ParamGroup::GPS, kGroupBlackbox = ParamGroup::BLACKBOX,
                             kGroupFilters = ParamGroup::FILTERS, kGroupMotors = ParamGroup::MOTORS;
        constexpr ParamDefault kTable = ParamDefault::TABLE, kTune = ParamDefault::TUNE, kBuild = ParamDefault::BUILD,
                               kBattery = ParamDefault::BATTERY;
        static const ParamInfo table[] = {
            { "tune_source", kEnum, kMaster, kGroupPid, 0, 1, 0, kTable, "", TUNE_SOURCES, 2,
              "откуда PID и фильтры: MODEL — из описания коптера, BETAFLIGHT — дефолты BF 4.5" },
            { "profile_name", kText, kPid, kGroupPid, 0, 8, 0, kTable, "", nullptr, 0, "имя PID-профиля" },
            { "p_roll", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0, "P крена" },
            { "i_roll", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0, "I крена" },
            { "d_roll", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0, "D крена" },
            { "f_roll", kInt, kPid, kGroupPid, 0, 1000, 0, kTune, "BF", nullptr, 0, "упреждение (feedforward) крена" },
            { "p_pitch", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0, "P тангажа" },
            { "i_pitch", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0, "I тангажа" },
            { "d_pitch", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0, "D тангажа" },
            { "f_pitch", kInt, kPid, kGroupPid, 0, 1000, 0, kTune, "BF", nullptr, 0, "упреждение (feedforward) тангажа" },
            { "p_yaw", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0, "P рысканья" },
            { "i_yaw", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0, "I рысканья" },
            { "d_yaw", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0, "D рысканья" },
            { "f_yaw", kInt, kPid, kGroupPid, 0, 1000, 0, kTune, "BF", nullptr, 0, "упреждение (feedforward) рысканья" },
            { "d_max_roll", kInt, kPid, kGroupPid, 0, 250, 0, kTable, "BF", nullptr, 0,
              "D крена на резком движении (тогда d_roll — в покое); 0 — D постоянный" },
            { "d_max_pitch", kInt, kPid, kGroupPid, 0, 250, 0, kTable, "BF", nullptr, 0,
              "D тангажа на резком движении (тогда d_pitch — в покое); 0 — D постоянный" },
            { "d_max_yaw", kInt, kPid, kGroupPid, 0, 250, 0, kTable, "BF", nullptr, 0,
              "D рысканья на резком движении (тогда d_yaw — в покое); 0 — D постоянный" },
            { "d_max_gain", kInt, kPid, kGroupPid, 0, 100, 37, kTable, "", nullptr, 0,
              "как легко D растёт от рывков гироскопа (пропвош, манёвр)" },
            { "d_max_advance", kInt, kPid, kGroupPid, 0, 200, 20, kTable, "", nullptr, 0,
              "как рано D растёт от движения стика" },
            { "d_min_roll", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0,
              "как в BF 4.5: D крена в покое, d_roll — на резком движении; 0 — нет" },
            { "d_min_pitch", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0,
              "как в BF 4.5: D тангажа в покое, d_pitch — на резком движении; 0 — нет" },
            { "d_min_yaw", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "BF", nullptr, 0,
              "как в BF 4.5: D рысканья в покое, d_yaw — на резком движении; 0 — нет" },
            { "iterm_relax", kEnum, kPid, kGroupPid, 0, 4, 0, kTune, "", ITERM_RELAX, 5,
              "оси, на которых I ждёт, пока уставка резко меняется (нет отскока после флипа); _INC — только рост" },
            { "iterm_relax_type", kEnum, kPid, kGroupPid, 0, 1, 1, kTable, "", ITERM_RELAX_TYPES, 2,
              "SETPOINT — I копится медленнее; GYRO — I к сглаженной уставке" },
            { "iterm_relax_cutoff", kInt, kPid, kGroupPid, 1, 50, 0, kTune, "Гц", nullptr, 0,
              "сглаживание уставки для relax: меньше — дольше ждёт" },
            { "iterm_relax_error", kEnum, kPid, kGroupPid, 0, 1, 1, kTable, "", OFF_ON, 2,
              "наш: большая ошибка разгона к новой уставке (флип) не копится в I; без движения стика — копится" },
            { "iterm_rotation", kEnum, kPid, kGroupPid, 0, 1, 0, kTable, "", OFF_ON, 2,
              "I поворачивается вместе с рамой (пируэты: крен переходит в тангаж)" },
            { "iterm_windup", kInt, kPid, kGroupPid, 30, 100, 0, kTune, "%", nullptr, 0,
              "моторы разошлись больше — I копится слабее; 100 — выкл" },
            { "iterm_limit", kInt, kPid, kGroupPid, 0, 500, 0, kTune, "BF", nullptr, 0,
              "предел I (1000 — весь ход моторов)" },
            { "pidsum_limit", kInt, kPid, kGroupPid, 100, 1000, 0, kTune, "BF", nullptr, 0,
              "предел суммы PID крена и тангажа (1000 — весь ход моторов)" },
            { "pidsum_limit_yaw", kInt, kPid, kGroupPid, 100, 1000, 0, kTune, "BF", nullptr, 0,
              "предел суммы PID рысканья (1000 — весь ход моторов)" },
            { "anti_gravity_gain", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "", nullptr, 0,
              "рывок газом — I сильнее, нос не клюёт: на полном рывке в 1 + gain/10 раз" },
            { "anti_gravity_cutoff_hz", kInt, kPid, kGroupPid, 2, 50, 0, kTune, "Гц", nullptr, 0,
              "сглаживание скорости газа для anti-gravity: меньше — пик ниже, но дольше" },
            { "anti_gravity_p_gain", kInt, kPid, kGroupPid, 0, 250, 0, kTune, "%", nullptr, 0,
              "рывок газом — P крена и тангажа сильнее: при 100 — на 50 % на полном рывке" },
            { "throttle_boost", kInt, kPid, kGroupPid, 0, 100, 0, kTune, "x0.1", nullptr, 0,
              "рывок газа со стика усиливается в 1 + boost/10 раз: моторы догоняют стик раньше" },
            { "throttle_boost_cutoff", kInt, kPid, kGroupPid, 5, 50, 15, kTable, "Гц", nullptr, 0,
              "что считать рывком газа для throttle_boost: газ минус его ФНЧ на этой частоте" },
            { "tpa_mode", kEnum, kPid, kGroupPid, 0, 2, 0, kTune, "", TPA_MODES, 3,
              "поправка по газу: PD, D — как в BF (tpa_rate, tpa_breakpoint), MODEL — из модели на P и D" },
            { "tpa_rate", kInt, kPid, kGroupPid, 0, 100, 65, kTable, "%", nullptr, 0,
              "PD, D: насколько слабее на полном газе (выше tpa_breakpoint — линейно)" },
            { "tpa_breakpoint", kInt, kPid, kGroupPid, 750, 2250, 1350, kTable, "мкс", nullptr, 0,
              "PD, D: газ, выше которого работает TPA" },
            { "tpa_low_rate", kInt, kPid, kGroupPid, -128, 100, 0, kTune, "%", nullptr, 0,
              "насколько слабее PD (D) на нуле газа; меньше 0 — сильнее" },
            { "tpa_low_breakpoint", kInt, kPid, kGroupPid, 750, 2250, 1050, kTable, "мкс", nullptr, 0,
              "газ, ниже которого работает tpa_low_rate" },
            { "tpa_low_always", kEnum, kPid, kGroupPid, 0, 1, 0, kTable, "", OFF_ON, 2,
              "OFF — tpa_low только до первого подъёма газа выше tpa_low_breakpoint после ARM; ON — всегда" },
            { "feedforward_transition", kInt, kPid, kGroupPid, 0, 100, 0, kTable, "%", nullptr, 0,
              "ближе к центру стика, чем столько, упреждение слабее (плавнее у центра); 0 — выкл" },
            { "feedforward_averaging", kEnum, kPid, kGroupPid, 0, 3, 0, kTable, "", FF_AVERAGING, 4,
              "скорость стика для упреждения — средняя за 2–4 кадра приёмника (ровнее, но позже)" },
            { "feedforward_smooth_factor", kInt, kPid, kGroupPid, 0, 95, 0, kTune, "%", nullptr, 0,
              "сглаживание упреждения от кадра к кадру: больше — ровнее, но позже" },
            { "feedforward_jitter_factor", kInt, kPid, kGroupPid, 0, 20, 0, kTune, "", nullptr, 0,
              "стик за кадр сдвинулся меньше (500 — полстика) — упреждение слабее: нет дрожи от пальца" },
            { "feedforward_boost", kInt, kPid, kGroupPid, 0, 50, 0, kTune, "", nullptr, 0,
              "добавка по ускорению стика: на рывке стиком моторы раньше берут разгон" },
            { "feedforward_max_rate_limit", kInt, kPid, kGroupPid, 0, 150, 0, kTune, "%", nullptr, 0,
              "уставка у такой доли наибольшего расхода — упреждение гаснет: нет перелёта на флипе; 0 — выкл" },
            { "yaw_lowpass_hz", kInt, kPid, kGroupFilters, 0, 500, 0, kTune, "Гц", nullptr, 0,
              "ФНЧ выхода PID рысканья: меньше — тише моторы, но рысканье мягче; 0 — без фильтра" },
            { "use_integrated_yaw", kEnum, kPid, kGroupPid, 0, 1, 0, kTable, "", OFF_ON, 2,
              "рысканье через интеграл суммы PID — для больших винтов; PID рысканья из модели пересчитываются" },
            { "integrated_yaw_relax", kInt, kPid, kGroupPid, 0, 255, 200, kTable, "", nullptr, 0,
              "утечка интеграла рысканья (use_integrated_yaw): больше — меньше «отскок» после пируэта" },
            { "abs_control_gain", kInt, kPid, kGroupPid, 0, 20, 0, kTable, "°/с на °", nullptr, 0,
              "ACRO: угол, потерянный от порыва, возвращается (не «уплывает»); 0 — выкл" },
            { "abs_control_limit", kInt, kPid, kGroupPid, 10, 255, 90, kTable, "°/с", nullptr, 0,
              "предел поправки absolute control" },
            { "abs_control_error_limit", kInt, kPid, kGroupPid, 1, 45, 20, kTable, "°", nullptr, 0,
              "предел накопленной ошибки угла absolute control" },
            { "abs_control_cutoff", kInt, kPid, kGroupPid, 1, 45, 11, kTable, "Гц", nullptr, 0,
              "absolute control: ФНЧ уставки — отставание рамы от быстрого стика не копится" },
            { "thrust_linear", kInt, kPid, kGroupMotors, 0, 150, 0, kTune, "%", nullptr, 0,
              "линеаризация тяги: малый газ отвечает так же резко, как большой (у мелких винтов больше)" },
            { "motor_output_limit", kInt, kPid, kGroupMotors, 1, 100, 100, kTable, "%", nullptr, 0,
              "предел хода моторов выше холостых: батарея 6S на моторах под 4S — 66" },
            { "rateprofile_name", kText, kRate, kGroupRates, 0, 8, 0, kTable, "", nullptr, 0, "имя профиля расходов" },
            { "roll_rc_rate", kInt, kRate, kGroupRates, 1, 255, 0, kTune, "x10 °/с", nullptr, 0, "расход крена у центра стика" },
            { "pitch_rc_rate", kInt, kRate, kGroupRates, 1, 255, 0, kTune, "x10 °/с", nullptr, 0, "расход тангажа у центра стика" },
            { "yaw_rc_rate", kInt, kRate, kGroupRates, 1, 255, 0, kTune, "x10 °/с", nullptr, 0, "расход рысканья у центра стика" },
            { "roll_expo", kInt, kRate, kGroupRates, 0, 100, 0, kTune, "%", nullptr, 0, "экспо крена: мягче у центра" },
            { "pitch_expo", kInt, kRate, kGroupRates, 0, 100, 0, kTune, "%", nullptr, 0, "экспо тангажа: мягче у центра" },
            { "yaw_expo", kInt, kRate, kGroupRates, 0, 100, 0, kTune, "%", nullptr, 0, "экспо рысканья: мягче у центра" },
            { "roll_srate", kInt, kRate, kGroupRates, 0, 255, 0, kTune, "x10 °/с", nullptr, 0, "наибольший расход крена (стик до упора)" },
            { "pitch_srate", kInt, kRate, kGroupRates, 0, 255, 0, kTune, "x10 °/с", nullptr, 0, "наибольший расход тангажа (стик до упора)" },
            { "yaw_srate", kInt, kRate, kGroupRates, 0, 255, 0, kTune, "x10 °/с", nullptr, 0, "наибольший расход рысканья (стик до упора)" },
            { "serialrx_provider", kEnum, kMaster, kGroupRx, 0, 2, 0, kTable, "", RX_PROVIDERS, 3,
              "протокол приёмника: IBUS — FlySky, SBUS — FrSky и др., CRSF — ExpressLRS, Crossfire" },
            { "rx_min_usec", kInt, kMaster, kGroupRx, 750, 2250, 885, kTable, "мкс", nullptr, 0,
              "импульс короче — ошибка канала (стик — потеря связи, AUX — держится прежнее)" },
            { "rx_max_usec", kInt, kMaster, kGroupRx, 750, 2250, 2115, kTable, "мкс", nullptr, 0,
              "импульс длиннее — ошибка канала (стик — потеря связи, AUX — держится прежнее)" },
            { "copter_build", kEnum, kMaster, kGroupCopter, 0, static_cast<float>(buildCount - 1), 0, kTable, "", builds,
              buildCount, "сборка из каталога — основа описания коптера" },
            { "copter_mass_g", kInt, kMaster, kGroupCopter, 0, 30000, 0, kBuild, "г", nullptr, 0,
              "взлётная масса с батареей (0 — оценка по размеру)" },
            { "copter_wheelbase_mm", kInt, kMaster, kGroupCopter, 30, 2000, 0, kBuild, "мм", nullptr, 0,
              "рама: между осями противоположных моторов" },
            { "prop_diameter_in", kFloat, kMaster, kGroupCopter, 0.5f, 40, 0, kBuild, "дюйм", nullptr, 0, "диаметр винта" },
            { "prop_pitch_in", kFloat, kMaster, kGroupCopter, 0.3f, 30, 0, kBuild, "дюйм", nullptr, 0, "шаг винта" },
            { "prop_blades", kInt, kMaster, kGroupCopter, 2, 8, 0, kBuild, "", nullptr, 0, "лопастей у винта" },
            { "motor_stator_mm", kFloat, kMaster, kGroupCopter, 3, 120, 0, kBuild, "мм", nullptr, 0,
              "ширина статора мотора (2207 — 22)" },
            { "motor_height_mm", kFloat, kMaster, kGroupCopter, 0.5f, 60, 0, kBuild, "мм", nullptr, 0,
              "высота статора мотора (2207 — 7)" },
            { "motor_kv", kInt, kMaster, kGroupCopter, 50, 80000, 0, kBuild, "об/мин/В", nullptr, 0, "KV мотора" },
            { "battery_cells", kInt, kMaster, kGroupCopter, 1, 14, 0, kBuild, "S", nullptr, 0, "банок в батарее" },
            { "battery_mah", kInt, kMaster, kGroupCopter, 50, 50000, 0, kBuild, "мА·ч", nullptr, 0, "ёмкость батареи" },
            { "props_out", kEnum, kMaster, kGroupCopter, 0, 1, 0, kBuild, "", OFF_ON, 2,
              "пропеллеры наружу: все моторы крутятся наоборот" },
            { "battery_chemistry", kEnum, kMaster, kGroupCopter, 0, 2, 0, kBuild, "", CHEMISTRIES, 3,
              "химия батареи: пороги банки и напряжение под нагрузкой для модели тяги" },
            { "vbat_min_cell_voltage", kInt, kMaster, kGroupPower, 100, 500, 0, kBattery, "x0.01 В", nullptr, 0,
              "банка ниже — батарея пуста: тревога CRITICAL" },
            { "vbat_warning_cell_voltage", kInt, kMaster, kGroupPower, 100, 500, 0, kBattery, "x0.01 В", nullptr, 0,
              "банка ниже — пора домой: тревога WARNING; ниже без нагрузки — не армить" },
            { "vbat_max_cell_voltage", kInt, kMaster, kGroupPower, 100, 500, 0, kBattery, "x0.01 В", nullptr, 0,
              "полная банка (для остатка в процентах без датчика тока)" },
            { "vbat_scale", kInt, kMaster, kGroupPower, 0, 255, 110, kTable, "", nullptr, 0,
              "делитель батареи на плате x10 (110 — 1:11)" },
            { "current_meter", kEnum, kMaster, kGroupPower, 0, 1, 0, kTable, "", CURRENT_METERS, 2,
              "датчик тока: ADC — площадка CUR регулятора" },
            { "ibata_scale", kInt, kMaster, kGroupPower, -16000, 16000, 400, kTable, "x0.1 мВ/А", nullptr, 0,
              "датчик тока: мВ на ампер x10 (у BLS 100A — 100)" },
            { "ibata_offset", kInt, kMaster, kGroupPower, -32000, 32000, 0, kTable, "мВ", nullptr, 0,
              "датчик тока: напряжение при нуле ампер" },
            { "vtx_protocol", kEnum, kMaster, kGroupVtx, 0, 3, 3, kTable, "", VTX_PROTOCOLS, 4,
              "провод управления видеопередатчиком: AUTO — найти SmartAudio или Tramp" },
            { "vtx_band", kInt, kMaster, kGroupVtx, 0, VTX_BANDS, 5, kTable, "", nullptr, 0,
              "полоса: 1 A, 2 B, 3 E, 4 F, 5 R, 6 L, 7 X, 8 V, 9 G, 10 S; 0 — частота числом (vtx_freq)" },
            { "vtx_channel", kInt, kMaster, kGroupVtx, 1, VTX_CHANNELS, 1, kTable, "", nullptr, 0, "канал полосы 1-8" },
            { "vtx_freq", kInt, kMaster, kGroupVtx, 0, 6200, 0, kTable, "МГц", nullptr, 0,
              "частота при vtx_band = 0 (0 — не трогать)" },
            { "vtx_power", kInt, kMaster, kGroupVtx, 1, VtxController::MAX_LEVELS, 1, kTable, "", nullptr, 0,
              "уровень мощности: 1 — самый слабый (таблица — от передатчика, vtx_info)" },
            { "vtx_max_power_mw", kInt, kMaster, kGroupVtx, 0, 10000, 0, kTable, "мВт", nullptr, 0,
              "потолок мощности: выше не уйдёт ни с тумблера, ни кнопкой на передатчике (0 — без потолка)" },
            { "vtx_low_power_disarm", kEnum, kMaster, kGroupVtx, 0, 2, 0, kTable, "", LOW_POWER_DISARM, 3,
              "без ARM — самый слабый уровень: ON — всегда, UNTIL_FIRST_ARM — до первого ARM" },
            { "gps_minimum_sats", kInt, kMaster, kGroupGps, 4, 50, 8, kTable, "", nullptr, 0,
              "дом ставится при ARM, если спутников не меньше" },
            { "crsf_telemetry", kEnum, kMaster, kGroupRx, 0, 1, 1, kTable, "", OFF_ON, 2,
              "телеметрия в пульт по CRSF: батарея, GPS, ориентация, режим" },
            { "blackbox_device", kEnum, kMaster, kGroupBlackbox, 0, 1, 1, kTable, "", BLACKBOX_DEVICES, 2,
              "куда писать чёрный ящик: SDCARD — файл BLACKBOX.BIN на карте, NONE — не писать" },
            { "blackbox_mode", kEnum, kMaster, kGroupBlackbox, 0, 2, 0, kTable, "", BLACKBOX_MODES, 3,
              "когда писать: NORMAL — пока ARM, MOTOR_TEST — пока крутятся моторы, ALWAYS — всегда" },
            { "blackbox_sample_rate", kEnum, kMaster, kGroupBlackbox, 0, 5, 0, kTable, "", BLACKBOX_RATES, 6,
              "гироскоп, PID и моторы — каждый какой цикл (1/1 — каждый: шум виден до половины частоты цикла)" },
        };
        static_assert(sizeof(table) / sizeof(table[0]) == PARAM_COUNT, "таблица настроек и ParamId разошлись");
        return table;
    }

    inline const ParamInfo& paramInfo(ParamId id) { return paramTable()[id < PARAM_COUNT ? id : 0]; }

    // Старые имена BF (4.5 и раньше) — те же настройки.
    struct ParamAlias
    {
        const char* name;
        int id;
    };

    // Настройка по имени (без учёта регистра); -1 — нет такой.
    inline int findParam(const char* name)
    {
        if (!name) return -1;
        static const ParamAlias aliases[] = {
            { "d_min_gain", PARAM_D_MAX_GAIN },
            { "d_min_advance", PARAM_D_MAX_ADVANCE },
        };
        for (const ParamAlias& alias : aliases)
        {
            const char* a = alias.name;
            const char* b = name;
            while (*a && tolower(static_cast<unsigned char>(*b)) == *a)
            {
                ++a;
                ++b;
            }
            if (*a == '\0' && *b == '\0') return alias.id;
        }
        for (int i = 0; i < PARAM_COUNT; ++i)
        {
            const char* a = paramTable()[i].name;
            const char* b = name;
            while (*a && tolower(static_cast<unsigned char>(*b)) == *a)
            {
                ++a;
                ++b;
            }
            if (*a == '\0' && *b == '\0') return i;
        }
        return -1;
    }

    // FNV-1a, свёрнутый до 16 бит: ключ настройки во флеше.
    inline uint16_t paramNameHash(const char* name)
    {
        uint32_t h = 2166136261u;
        for (const char* p = name; *p; ++p)
        {
            h ^= static_cast<uint8_t>(*p);
            h *= 16777619u;
        }
        return static_cast<uint16_t>(h ^ (h >> 16));
    }

    inline ParamId pidParam(uint8_t axis, PidTerm term)
    {
        return static_cast<ParamId>(PARAM_P_ROLL + axis * 4 + static_cast<uint8_t>(term));
    }


    class CopterParams
    {
    public:
        static constexpr uint8_t PROFILES = 4;
        static constexpr uint8_t NAME_LENGTH = 8;
        static constexpr uint8_t STORE_VERSION = 1;
        static constexpr const char* STORE_SPACE = "copter";
        static constexpr const char* STORE_KEY = "params";
        static constexpr const char* RC_KEY = "rc";

        using BuildOverrides = void (*)(BuildSpec&);

        CopterParams()
        {
            for (uint8_t i = 0; i < PARAM_COUNT; ++i) tableDefaults[i] = paramTable()[i].def;
            uint8_t n = 0;
            fileBuild = buildCatalog(n)[0];
            clearValues();
            refreshDefaults();
        }

        // Описание коптера из прошивки (сборка с поправками CopterConfig.h) ->
        // модельная настройка (перебор контура — долго) и умолчания.
        // Заданное руками не трогается. overrides — те же поправки для
        // другой сборки, выбранной в CLI (copter_build).
        void begin(const BuildSpec& b, float loop, EscProtocol escProtocol, BuildOverrides overrides = nullptr)
        {
            fileBuild = b;
            fileOverrides = overrides;
            loopHz = loop;
            esc = escProtocol;
            uint8_t n = 0;
            const BuildSpec* c = buildCatalog(n);
            fileIndex = 0;
            for (uint8_t i = 0; i < n; ++i)
            {
                if (b.id && strcmp(c[i].id, b.id) == 0) fileIndex = i;
            }
            tableDefaults[PARAM_COPTER_BUILD] = fileIndex;
            begun = true;
            haveModel = false;
            refreshDefaults();
            updateModel();
        }

        // Умолчание из таблицы, которое знает только прошивка (протокол приёмника).
        void setTableDefault(ParamId id, float v)
        {
            if (id >= PARAM_COUNT || paramInfo(id).from != ParamDefault::TABLE) return;
            tableDefaults[id] = v;
            refreshDefaults();
        }

        const CopterTune& modelTune() const { return model; }

        // Описание коптера: сборка copter_build (с поправками прошивки) и
        // заданное в CLI поверх.
        BuildSpec buildSpec() const
        {
            const BuildSpec base = baseBuild(static_cast<uint8_t>(value(PARAM_COPTER_BUILD)));
            BuildSpec b = base;
            b.massG = value(PARAM_COPTER_MASS_G);
            b.wheelbaseMm = value(PARAM_COPTER_WHEELBASE_MM);
            b.prop.diameterIn = value(PARAM_PROP_DIAMETER_IN);
            b.prop.pitchIn = value(PARAM_PROP_PITCH_IN);
            b.prop.blades = static_cast<uint8_t>(value(PARAM_PROP_BLADES));
            b.motor.statorWidthMm = value(PARAM_MOTOR_STATOR_MM);
            b.motor.statorHeightMm = value(PARAM_MOTOR_HEIGHT_MM);
            b.motor.kv = value(PARAM_MOTOR_KV);
            b.battery.cells = static_cast<uint8_t>(value(PARAM_BATTERY_CELLS));
            b.battery.capacityMah = static_cast<uint16_t>(value(PARAM_BATTERY_MAH));
            b.propsOut = value(PARAM_PROPS_OUT) > 0.5f;
            b.battery.chemistry = chemistry();
            b.battery.voltsPerCellLoaded = cellVoltages(b.battery.chemistry).loaded;
            // другой винт или мотор — масса заново по размеру
            if (b.prop.diameterIn != base.prop.diameterIn || b.prop.blades != base.prop.blades) b.prop.massG = 0;
            if (b.motor.statorWidthMm != base.motor.statorWidthMm || b.motor.statorHeightMm != base.motor.statorHeightMm)
                b.motor.massG = 0;
            return b;
        }

        // Описание поменялось — модель заново (долго; только без ARM).
        // true — пересчитана.
        bool updateModel()
        {
            if (!begun) return false;
            const BuildSpec b = buildSpec();
            if (haveModel && sameBuild(b, modelSpec)) return false;
            model = deriveTune(b, loopHz, esc);
            modelSpec = b;
            haveModel = true;
            refreshDefaults();
            return true;
        }

        BatteryChemistry chemistry() const
        {
            return static_cast<BatteryChemistry>(static_cast<uint8_t>(value(PARAM_BATTERY_CHEMISTRY)));
        }

        // Батарея: масштабы, пороги, банки и ёмкость — монитору (Battery.h).
        BatteryConfig batteryConfig() const
        {
            BatteryConfig c;
            c.vbatScale = value(PARAM_VBAT_SCALE);
            c.ibataScale = value(PARAM_IBATA_SCALE);
            c.ibataOffsetMv = value(PARAM_IBATA_OFFSET);
            c.currentSensor = value(PARAM_CURRENT_METER) > 0.5f;
            c.cells = static_cast<uint8_t>(value(PARAM_BATTERY_CELLS));
            c.capacityMah = value(PARAM_BATTERY_MAH);
            c.minCellVolts = value(PARAM_VBAT_MIN_CELL_VOLTAGE) * 0.01f;
            c.warningCellVolts = value(PARAM_VBAT_WARNING_CELL_VOLTAGE) * 0.01f;
            c.maxCellVolts = value(PARAM_VBAT_MAX_CELL_VOLTAGE) * 0.01f;
            return c;
        }

        // Настройки контроллера, которые задаёт реестр: батарея ниже порога
        // «пора домой» без нагрузки — не армить.
        void applyTo(CopterSettings& s) const { s.minCellVolts = value(PARAM_VBAT_WARNING_CELL_VOLTAGE) * 0.01f; }

        // Пульт: карта каналов, rxrange, таблица aux.
        RcConfig& rc() { return rcConfig; }
        const RcConfig& rc() const { return rcConfig; }

        // ---- профили ----
        uint8_t pidProfile() const { return pidIndex; }
        uint8_t rateProfile() const { return rateIndex; }
        bool selectPidProfile(uint8_t n)
        {
            if (n >= PROFILES) return false;
            pidIndex = n;
            return true;
        }
        bool selectRateProfile(uint8_t n)
        {
            if (n >= PROFILES) return false;
            rateIndex = n;
            return true;
        }

        // Профиль, в котором сейчас работает настройка (общие — 0).
        uint8_t activeSlot(ParamId id) const
        {
            switch (paramInfo(id).scope)
            {
                case ParamScope::PID_PROFILE: return pidIndex;
                case ParamScope::RATE_PROFILE: return rateIndex;
                case ParamScope::MASTER:
                default: return 0;
            }
        }

        // ---- значения ----
        float defaultValue(ParamId id) const { return defaults[id]; }
        bool isSet(ParamId id, uint8_t slot) const
        {
            if (paramInfo(id).type == ParamType::STRING) return text(id, slot)[0] != '\0';
            return slot < PROFILES && (setMask[id] >> slot) & 1u;
        }
        float valueIn(ParamId id, uint8_t slot) const { return isSet(id, slot) ? values[id][slot] : defaults[id]; }
        float value(ParamId id) const { return valueIn(id, activeSlot(id)); }

        const char* text(ParamId id, uint8_t slot) const
        {
            const char* s = stringSlot(id, slot);
            return s ? s : "";
        }

        // Задать число (в границах, ENUM — номер значения). Равное
        // умолчанию — то же, что вернуть умолчание.
        bool setValue(ParamId id, uint8_t slot, float v)
        {
            const ParamInfo& info = paramInfo(id);
            if (slot >= PROFILES || info.type == ParamType::STRING || !(v >= info.min && v <= info.max)) return false;
            if (info.type != ParamType::FLOAT) v = roundf(v);
            if (v == defaults[id])
            {
                setMask[id] &= static_cast<uint8_t>(~(1u << slot));
            }
            else
            {
                values[id][slot] = v;
                setMask[id] |= static_cast<uint8_t>(1u << slot);
            }
            if (id == PARAM_TUNE_SOURCE || id == PARAM_COPTER_BUILD || id == PARAM_BATTERY_CHEMISTRY) refreshDefaults();
            return true;
        }

        bool setText(ParamId id, uint8_t slot, const char* s)
        {
            char* dst = stringSlot(id, slot);
            if (!dst || !s || strlen(s) > NAME_LENGTH) return false;
            strcpy(dst, s);
            return true;
        }

        void reset(ParamId id, uint8_t slot)
        {
            char* s = stringSlot(id, slot);
            if (s) s[0] = '\0';
            if (slot < PROFILES) setMask[id] &= static_cast<uint8_t>(~(1u << slot));
            if (id == PARAM_TUNE_SOURCE || id == PARAM_COPTER_BUILD || id == PARAM_BATTERY_CHEMISTRY) refreshDefaults();
        }

        // Всё по умолчанию, профили 0, таблицы пульта пусты (как `defaults`
        // в BF). Модель — при следующем updateModel().
        void resetAll()
        {
            clearValues();
            rcConfig.reset();
            refreshDefaults();
        }

        TuneSource source() const { return static_cast<TuneSource>(static_cast<uint8_t>(value(PARAM_TUNE_SOURCE))); }

        // ---- текст, как в CLI ----

        // Разобрать и задать. false — в error почему (по-русски).
        bool parse(ParamId id, uint8_t slot, const char* textIn, char* error, size_t errorSize)
        {
            const ParamInfo& info = paramInfo(id);
            char buf[48];
            trimCopy(textIn, buf, sizeof(buf));
            switch (info.type)
            {
                case ParamType::STRING:
                {
                    const char* s = strcmp(buf, "-") == 0 ? "" : buf;
                    if (setText(id, slot, s)) return true;
                    snprintf(error, errorSize, "не длиннее %d символов", static_cast<int>(info.max));
                    return false;
                }
                case ParamType::ENUM:
                {
                    for (uint8_t i = 0; i < info.optionCount; ++i)
                    {
                        if (sameText(buf, info.options[i])) return setValue(id, slot, i);
                    }
                    int n = snprintf(error, errorSize, "допустимо:");
                    for (uint8_t i = 0; i < info.optionCount && n > 0 && static_cast<size_t>(n) < errorSize; ++i)
                    {
                        n += snprintf(error + n, errorSize - n, "%s %s", i ? "," : "", info.options[i]);
                    }
                    return false;
                }
                case ParamType::INT:
                case ParamType::FLOAT:
                default:
                {
                    char* end = nullptr;
                    const float v = info.type == ParamType::INT ? static_cast<float>(strtol(buf, &end, 10)) : strtof(buf, &end);
                    if (end != buf && *end == '\0' && setValue(id, slot, v)) return true;
                    char lo[16], hi[16];
                    formatNumber(info, info.min, lo, sizeof(lo));
                    formatNumber(info, info.max, hi, sizeof(hi));
                    snprintf(error, errorSize, "допустимо %s - %s", lo, hi);
                    return false;
                }
            }
        }

        void format(ParamId id, uint8_t slot, char* out, size_t size) const
        {
            const ParamInfo& info = paramInfo(id);
            if (info.type == ParamType::STRING)
            {
                snprintf(out, size, "%s", text(id, slot));
                return;
            }
            formatNumber(info, valueIn(id, slot), out, size);
        }

        void formatDefault(ParamId id, char* out, size_t size) const
        {
            const ParamInfo& info = paramInfo(id);
            if (info.type == ParamType::STRING)
            {
                snprintf(out, size, "%s", "");
                return;
            }
            formatNumber(info, defaults[id], out, size);
        }

        static void formatNumber(const ParamInfo& info, float v, char* out, size_t size)
        {
            if (info.type == ParamType::ENUM)
            {
                const uint8_t i = static_cast<uint8_t>(v);
                snprintf(out, size, "%s", i < info.optionCount ? info.options[i] : "?");
            }
            else if (info.type == ParamType::FLOAT)
            {
                // 5.1, а не 5.100
                snprintf(out, size, "%.3f", static_cast<double>(v));
                char* dot = strchr(out, '.');
                if (dot)
                {
                    char* end = out + strlen(out);
                    while (end > dot + 1 && end[-1] == '0') *--end = '\0';
                    if (end == dot + 1) *dot = '\0';
                }
            }
            else
            {
                snprintf(out, size, "%ld", lroundf(v));
            }
        }

        // ---- настройка контроллеру ----

        // Модельная настройка + источник + выбранные профили.
        CopterTune tune() const
        {
            CopterTune t = model;
            const bool bf = source() == TuneSource::BETAFLIGHT;
            const bool integrated = valueIn(PARAM_USE_INTEGRATED_YAW, pidIndex) > 0.5f;
            t.pid.integratedYaw = integrated;
            t.pid.integratedYawRelax = valueIn(PARAM_INTEGRATED_YAW_RELAX, pidIndex);
            if (bf) applyBetaflightBase(t);
            AxisGains* gains[3] = { &t.roll, &t.pitch, &t.yaw };
            for (uint8_t axis = 0; axis < 3; ++axis)
            {
                for (uint8_t k = 0; k < 4; ++k)
                {
                    const PidTerm term = static_cast<PidTerm>(k);
                    const ParamId id = pidParam(axis, term);
                    // своё число BF — и в модельном режиме: задан руками, значит так и надо
                    if (bf || isSet(id, pidIndex))
                    {
                        gainTerm(*gains[axis], term) = bfToGain(term, axis, valueIn(id, pidIndex));
                        t.gainsFromModel = false;
                    }
                }
                if (axis == betaflight::AXIS_YAW && integrated) integratedYawTerms(t.yaw, model.yaw, bf);
                // D на резком движении: d_max (BF 2025.12; D из таблицы — в покое)
                // или d_min (BF 4.5; D из таблицы — на движении, d_min — в покое)
                const float dMax = valueIn(static_cast<ParamId>(PARAM_D_MAX_ROLL + axis), pidIndex);
                const float dMin = valueIn(static_cast<ParamId>(PARAM_D_MIN_ROLL + axis), pidIndex);
                const float dBase = gainToBf(PidTerm::D, axis, gains[axis]->d);
                t.pid.dMax[axis] = 0.0f;
                if (dMax > 0.0f)
                {
                    t.pid.dMax[axis] = bfToGain(PidTerm::D, axis, dMax);
                }
                else if (dMin > 0.0f && dMin < dBase)
                {
                    t.pid.dMax[axis] = gains[axis]->d;
                    gains[axis]->d = bfToGain(PidTerm::D, axis, dMin);
                }
                const float center = 10.0f * valueIn(static_cast<ParamId>(PARAM_ROLL_RC_RATE + axis), rateIndex);
                const float top = 10.0f * valueIn(static_cast<ParamId>(PARAM_ROLL_SRATE + axis), rateIndex);
                t.rates[axis].centerDps = center;
                t.rates[axis].maxDps = fmaxf(center, top);
                t.rates[axis].expo = valueIn(static_cast<ParamId>(PARAM_ROLL_EXPO + axis), rateIndex) / 100.0f;
            }
            t.pid.dMaxGain = valueIn(PARAM_D_MAX_GAIN, pidIndex);
            t.pid.dMaxAdvance = valueIn(PARAM_D_MAX_ADVANCE, pidIndex);
            t.pid.relax = static_cast<ItermRelax>(static_cast<uint8_t>(valueIn(PARAM_ITERM_RELAX, pidIndex)));
            t.pid.relaxType = static_cast<ItermRelaxType>(static_cast<uint8_t>(valueIn(PARAM_ITERM_RELAX_TYPE, pidIndex)));
            t.pid.relaxCutoffHz = valueIn(PARAM_ITERM_RELAX_CUTOFF, pidIndex);
            t.pid.relaxError = valueIn(PARAM_ITERM_RELAX_ERROR, pidIndex) > 0.5f;
            t.pid.itermRotation = valueIn(PARAM_ITERM_ROTATION, pidIndex) > 0.5f;
            t.pid.itermWindup = valueIn(PARAM_ITERM_WINDUP, pidIndex) / 100.0f;
            t.iLimit = valueIn(PARAM_ITERM_LIMIT, pidIndex) / betaflight::MIXER_SCALING;
            t.pid.pidSumLimit = valueIn(PARAM_PIDSUM_LIMIT, pidIndex) / betaflight::MIXER_SCALING;
            t.pid.pidSumLimitYaw = valueIn(PARAM_PIDSUM_LIMIT_YAW, pidIndex) / betaflight::MIXER_SCALING;
            t.pid.antiGravityI = valueIn(PARAM_ANTI_GRAVITY_GAIN, pidIndex) / 10.0f;
            t.pid.antiGravityHz = valueIn(PARAM_ANTI_GRAVITY_CUTOFF_HZ, pidIndex);
            t.pid.antiGravityP = valueIn(PARAM_ANTI_GRAVITY_P_GAIN, pidIndex) / 100.0f;
            t.pid.throttleBoost = valueIn(PARAM_THROTTLE_BOOST, pidIndex) / 10.0f;
            t.pid.throttleBoostHz = valueIn(PARAM_THROTTLE_BOOST_CUTOFF, pidIndex);
            t.pid.tpaMode = static_cast<TpaMode>(static_cast<uint8_t>(valueIn(PARAM_TPA_MODE, pidIndex)));
            t.pid.tpaRate = valueIn(PARAM_TPA_RATE, pidIndex) / 100.0f;
            t.pid.tpaBreakpoint = betaflight::throttleFromUs(valueIn(PARAM_TPA_BREAKPOINT, pidIndex));
            t.pid.tpaLowRate = valueIn(PARAM_TPA_LOW_RATE, pidIndex) / 100.0f;
            t.pid.tpaLowBreakpoint = betaflight::throttleFromUs(valueIn(PARAM_TPA_LOW_BREAKPOINT, pidIndex));
            t.pid.tpaLowAlways = valueIn(PARAM_TPA_LOW_ALWAYS, pidIndex) > 0.5f;
            t.pid.ffTransition = valueIn(PARAM_FEEDFORWARD_TRANSITION, pidIndex) / 100.0f;
            t.pid.ffAveraging = static_cast<uint8_t>(valueIn(PARAM_FEEDFORWARD_AVERAGING, pidIndex)) + 1;
            t.pid.ffSmoothFactor = valueIn(PARAM_FEEDFORWARD_SMOOTH_FACTOR, pidIndex) / 100.0f;
            t.pid.ffJitterFactor = valueIn(PARAM_FEEDFORWARD_JITTER_FACTOR, pidIndex);
            t.pid.ffBoost = valueIn(PARAM_FEEDFORWARD_BOOST, pidIndex);
            t.pid.ffMaxRateLimit = valueIn(PARAM_FEEDFORWARD_MAX_RATE_LIMIT, pidIndex) / 100.0f;
            t.pid.absGain = valueIn(PARAM_ABS_CONTROL_GAIN, pidIndex);
            t.pid.absLimit = valueIn(PARAM_ABS_CONTROL_LIMIT, pidIndex);
            t.pid.absErrorLimit = valueIn(PARAM_ABS_CONTROL_ERROR_LIMIT, pidIndex);
            t.pid.absCutoffHz = valueIn(PARAM_ABS_CONTROL_CUTOFF, pidIndex);
            // ФНЧ рысканья и линеаризация: не заданы — точное значение модели (BF — его)
            if (isSet(PARAM_YAW_LOWPASS_HZ, pidIndex)) t.yawLpfHz = valueIn(PARAM_YAW_LOWPASS_HZ, pidIndex);
            if (isSet(PARAM_THRUST_LINEAR, pidIndex)) t.thrustLinear = valueIn(PARAM_THRUST_LINEAR, pidIndex) / 100.0f;
            t.motorOutputLimit = valueIn(PARAM_MOTOR_OUTPUT_LIMIT, pidIndex) / 100.0f;
            return t;
        }

        // use_integrated_yaw: числа BF (заданные или tune_source = BETAFLIGHT) —
        // как в BF: I рысканья без множителя 2.5; незаданные из модели —
        // пересчитаны так, чтобы контур остался тем же (integratedYawGains).
        void integratedYawTerms(AxisGains& yaw, const AxisGains& modelYaw, bool bf) const
        {
            const AxisGains conv = integratedYawGains(modelYaw, valueIn(PARAM_INTEGRATED_YAW_RELAX, pidIndex));
            for (uint8_t k = 0; k < 4; ++k)
            {
                const PidTerm term = static_cast<PidTerm>(k);
                if (bf || isSet(pidParam(betaflight::AXIS_YAW, term), pidIndex))
                {
                    if (term == PidTerm::I) yaw.i /= betaflight::YAW_I_FACTOR;
                }
                else
                {
                    gainTerm(yaw, term) = gainTerm(conv, term);
                }
            }
        }

        // ---- флеш ----

        // Записать заданное руками (commit — за вызывающим).
        bool save(KeyValueStore& kv) const
        {
            uint8_t blob[3 + PARAM_COUNT * PROFILES * (4 + NAME_LENGTH)];
            size_t n = 0;
            blob[n++] = STORE_VERSION;
            blob[n++] = pidIndex;
            blob[n++] = rateIndex;
            bool any = pidIndex != 0 || rateIndex != 0;
            for (uint8_t i = 0; i < PARAM_COUNT; ++i)
            {
                const ParamId id = static_cast<ParamId>(i);
                const ParamInfo& info = paramInfo(id);
                const uint16_t h = paramNameHash(info.name);
                const uint8_t slots = info.scope == ParamScope::MASTER ? 1 : PROFILES;
                for (uint8_t slot = 0; slot < slots; ++slot)
                {
                    if (!isSet(id, slot)) continue;
                    any = true;
                    blob[n++] = static_cast<uint8_t>(h);
                    blob[n++] = static_cast<uint8_t>(h >> 8);
                    if (info.type == ParamType::STRING)
                    {
                        const char* s = text(id, slot);
                        const size_t len = strlen(s);
                        blob[n++] = static_cast<uint8_t>(0x80u | slot);
                        blob[n++] = static_cast<uint8_t>(len);
                        memcpy(blob + n, s, len);
                        n += len;
                    }
                    else
                    {
                        blob[n++] = slot;
                        memcpy(blob + n, &values[id][slot], 4);
                        n += 4;
                    }
                }
            }
            bool ok = true;
            if (!any) kv.remove(STORE_SPACE, STORE_KEY);
            else ok = kv.put(STORE_SPACE, STORE_KEY, blob, n);

            uint8_t rcBlob[RcConfig::STORE_MAX];
            const size_t rcLength = rcConfig.serialize(rcBlob, sizeof(rcBlob));
            if (rcConfig.isDefault()) kv.remove(STORE_SPACE, RC_KEY);
            else ok = kv.put(STORE_SPACE, RC_KEY, rcBlob, rcLength) && ok;
            return ok;
        }

        // Прочитать из флеша (и пересчитать модель, если описание коптера
        // там другое). false — там ничего нет или не та версия: всё по умолчанию.
        bool load(KeyValueStore& kv)
        {
            size_t rcLength = 0;
            const uint8_t* rcData = kv.get(STORE_SPACE, RC_KEY, &rcLength);
            const bool rcOk = rcConfig.deserialize(rcData, rcLength);

            clearValues();
            size_t length = 0;
            const uint8_t* data = kv.get(STORE_SPACE, STORE_KEY, &length);
            const bool ok = data && length >= 3 && data[0] == STORE_VERSION;
            if (ok)
            {
                pidIndex = data[1] < PROFILES ? data[1] : 0;
                rateIndex = data[2] < PROFILES ? data[2] : 0;
                size_t at = 3;
                while (at + 3 <= length)
                {
                    const uint16_t h = static_cast<uint16_t>(data[at] | (data[at + 1] << 8));
                    const uint8_t tag = data[at + 2];
                    const bool isText = (tag & 0x80u) != 0;
                    const uint8_t slot = tag & 0x7Fu;
                    at += 3;
                    const int found = findHash(h);
                    if (isText)
                    {
                        if (at >= length) break;
                        const uint8_t len = data[at++];
                        if (at + len > length) break;
                        char s[NAME_LENGTH + 1] = {};
                        if (found >= 0 && len <= NAME_LENGTH)
                        {
                            memcpy(s, data + at, len);
                            setText(static_cast<ParamId>(found), slot, s);
                        }
                        at += len;
                    }
                    else
                    {
                        if (at + 4 > length) break;
                        float v;
                        memcpy(&v, data + at, 4);
                        at += 4;
                        if (found < 0 || slot >= PROFILES) continue;   // настройки больше нет
                        const ParamInfo& info = paramInfo(static_cast<ParamId>(found));
                        if (info.type == ParamType::STRING || isnan(v)) continue;
                        if (info.type == ParamType::ENUM && (v < 0 || v >= info.optionCount)) continue;
                        values[found][slot] = fminf(info.max, fmaxf(info.min, v));
                        setMask[found] |= static_cast<uint8_t>(1u << slot);
                    }
                }
            }
            refreshDefaults();
            updateModel();
            return ok || rcOk;
        }

    private:
        void clearValues()
        {
            memset(values, 0, sizeof(values));
            memset(setMask, 0, sizeof(setMask));
            memset(names, 0, sizeof(names));
            pidIndex = 0;
            rateIndex = 0;
        }

        // Умолчания: из таблицы, PID и расходы — из модели или BF 4.5,
        // описание — из сборки copter_build.
        void refreshDefaults()
        {
            for (uint8_t i = 0; i < PARAM_COUNT; ++i) defaults[i] = tableDefaults[i];
            const BuildSpec base = baseBuild(static_cast<uint8_t>(value(PARAM_COPTER_BUILD)));
            defaults[PARAM_COPTER_MASS_G] = roundf(base.massG);
            defaults[PARAM_COPTER_WHEELBASE_MM] = roundf(base.wheelbaseMm);
            defaults[PARAM_PROP_DIAMETER_IN] = base.prop.diameterIn;
            defaults[PARAM_PROP_PITCH_IN] = base.prop.pitchIn;
            defaults[PARAM_PROP_BLADES] = base.prop.blades;
            defaults[PARAM_MOTOR_STATOR_MM] = base.motor.statorWidthMm;
            defaults[PARAM_MOTOR_HEIGHT_MM] = base.motor.statorHeightMm;
            defaults[PARAM_MOTOR_KV] = roundf(base.motor.kv);
            defaults[PARAM_BATTERY_CELLS] = base.battery.cells;
            defaults[PARAM_BATTERY_MAH] = base.battery.capacityMah;
            defaults[PARAM_PROPS_OUT] = base.propsOut ? 1.0f : 0.0f;
            defaults[PARAM_BATTERY_CHEMISTRY] = static_cast<float>(base.battery.chemistry);
            const CellVoltages cells = cellVoltages(chemistry());
            defaults[PARAM_VBAT_MIN_CELL_VOLTAGE] = roundf(cells.min * 100.0f);
            defaults[PARAM_VBAT_WARNING_CELL_VOLTAGE] = roundf(cells.warning * 100.0f);
            defaults[PARAM_VBAT_MAX_CELL_VOLTAGE] = roundf(cells.max * 100.0f);

            const bool bf = source() == TuneSource::BETAFLIGHT;
            const AxisGains* gains[3] = { &model.roll, &model.pitch, &model.yaw };
            for (uint8_t axis = 0; axis < 3; ++axis)
            {
                for (uint8_t k = 0; k < 4; ++k)
                {
                    const PidTerm term = static_cast<PidTerm>(k);
                    const ParamId id = pidParam(axis, term);
                    const ParamInfo& info = paramInfo(id);
                    const float v = bf ? betaflight::DEFAULT_PID[axis][k] : roundf(gainToBf(term, axis, gainTerm(*gains[axis], term)));
                    defaults[id] = fminf(info.max, fmaxf(info.min, v));
                }
                defaults[PARAM_D_MIN_ROLL + axis] = bf ? betaflight::DEFAULT_D_MIN[axis] : 0.0f;
                const RateCurve& r = model.rates[axis];
                defaults[PARAM_ROLL_RC_RATE + axis] = bf ? betaflight::DEFAULT_RC_RATE : fmaxf(1.0f, roundf(r.centerDps / 10.0f));
                defaults[PARAM_ROLL_SRATE + axis] = bf ? betaflight::DEFAULT_SRATE : fminf(255.0f, roundf(r.maxDps / 10.0f));
                defaults[PARAM_ROLL_EXPO + axis] = bf ? betaflight::DEFAULT_EXPO : roundf(r.expo * 100.0f);
            }
            CopterTune pidBase = model;
            if (bf) applyBetaflightBase(pidBase);
            defaults[PARAM_ITERM_RELAX] = static_cast<float>(pidBase.pid.relax);
            defaults[PARAM_ITERM_RELAX_CUTOFF] = roundf(pidBase.pid.relaxCutoffHz);
            defaults[PARAM_ITERM_WINDUP] = roundf(pidBase.pid.itermWindup * 100.0f);
            defaults[PARAM_ITERM_LIMIT] = roundf(pidBase.iLimit * betaflight::MIXER_SCALING);
            defaults[PARAM_PIDSUM_LIMIT] = roundf(pidBase.pid.pidSumLimit * betaflight::MIXER_SCALING);
            defaults[PARAM_PIDSUM_LIMIT_YAW] = roundf(pidBase.pid.pidSumLimitYaw * betaflight::MIXER_SCALING);
            defaults[PARAM_ANTI_GRAVITY_GAIN] = roundf(pidBase.pid.antiGravityI * 10.0f);
            defaults[PARAM_ANTI_GRAVITY_CUTOFF_HZ] = roundf(pidBase.pid.antiGravityHz);
            defaults[PARAM_ANTI_GRAVITY_P_GAIN] = roundf(pidBase.pid.antiGravityP * 100.0f);
            defaults[PARAM_THROTTLE_BOOST] = roundf(pidBase.pid.throttleBoost * 10.0f);
            defaults[PARAM_TPA_MODE] = static_cast<float>(pidBase.pid.tpaMode);
            defaults[PARAM_TPA_LOW_RATE] = roundf(pidBase.pid.tpaLowRate * 100.0f);
            defaults[PARAM_FEEDFORWARD_SMOOTH_FACTOR] = roundf(pidBase.pid.ffSmoothFactor * 100.0f);
            defaults[PARAM_FEEDFORWARD_JITTER_FACTOR] = roundf(pidBase.pid.ffJitterFactor);
            defaults[PARAM_FEEDFORWARD_BOOST] = roundf(pidBase.pid.ffBoost);
            defaults[PARAM_FEEDFORWARD_MAX_RATE_LIMIT] = roundf(pidBase.pid.ffMaxRateLimit * 100.0f);
            defaults[PARAM_YAW_LOWPASS_HZ] = roundf(pidBase.yawLpfHz);
            defaults[PARAM_THRUST_LINEAR] = roundf(pidBase.thrustLinear * 100.0f);
        }

        // Сборка каталога с поправками прошивки (для сборки прошивки — она сама).
        BuildSpec baseBuild(uint8_t index) const
        {
            if (index == fileIndex) return fileBuild;
            uint8_t n = 0;
            const BuildSpec* c = buildCatalog(n);
            BuildSpec b = c[index < n ? index : 0];
            if (fileOverrides) fileOverrides(b);
            return b;
        }

        static bool sameBuild(const BuildSpec& a, const BuildSpec& b)
        {
            return a.id == b.id && a.frame == b.frame && a.wheelbaseMm == b.wheelbaseMm &&
                   a.prop.diameterIn == b.prop.diameterIn && a.prop.pitchIn == b.prop.pitchIn &&
                   a.prop.blades == b.prop.blades && a.prop.massG == b.prop.massG &&
                   a.motor.statorWidthMm == b.motor.statorWidthMm && a.motor.statorHeightMm == b.motor.statorHeightMm &&
                   a.motor.kv == b.motor.kv && a.motor.massG == b.motor.massG && a.battery.cells == b.battery.cells &&
                   a.battery.capacityMah == b.battery.capacityMah &&
                   a.battery.voltsPerCellLoaded == b.battery.voltsPerCellLoaded && a.massG == b.massG &&
                   a.style == b.style && a.propsOut == b.propsOut;
        }

        int findHash(uint16_t h) const
        {
            for (int i = 0; i < PARAM_COUNT; ++i)
            {
                if (paramNameHash(paramTable()[i].name) == h) return i;
            }
            return -1;
        }

        char* stringSlot(ParamId id, uint8_t slot)
        {
            if (slot >= PROFILES) return nullptr;
            if (id == PARAM_PROFILE_NAME) return names[0][slot];
            if (id == PARAM_RATEPROFILE_NAME) return names[1][slot];
            return nullptr;
        }
        const char* stringSlot(ParamId id, uint8_t slot) const { return const_cast<CopterParams*>(this)->stringSlot(id, slot); }

        static bool sameText(const char* a, const char* b)
        {
            while (*a && *b && tolower(static_cast<unsigned char>(*a)) == tolower(static_cast<unsigned char>(*b)))
            {
                ++a;
                ++b;
            }
            return *a == '\0' && *b == '\0';
        }

        static void trimCopy(const char* in, char* out, size_t size)
        {
            if (!in) in = "";
            while (*in == ' ' || *in == '\t') ++in;
            size_t n = 0;
            while (in[n] && n + 1 < size)
            {
                out[n] = in[n];
                ++n;
            }
            while (n > 0 && (out[n - 1] == ' ' || out[n - 1] == '\t' || out[n - 1] == '\r' || out[n - 1] == '\n')) --n;
            out[n] = '\0';
        }

        CopterTune model;
        BuildSpec modelSpec = {};
        bool haveModel = false;
        bool begun = false;
        BuildSpec fileBuild = {};
        uint8_t fileIndex = 0;
        BuildOverrides fileOverrides = nullptr;
        float loopHz = 1000.0f;
        EscProtocol esc = EscProtocol::ONESHOT125;
        RcConfig rcConfig;
        float tableDefaults[PARAM_COUNT] = {};
        float defaults[PARAM_COUNT] = {};
        float values[PARAM_COUNT][PROFILES];
        uint8_t setMask[PARAM_COUNT];
        char names[2][PROFILES][NAME_LENGTH + 1];
        uint8_t pidIndex = 0;
        uint8_t rateIndex = 0;
    };
}
