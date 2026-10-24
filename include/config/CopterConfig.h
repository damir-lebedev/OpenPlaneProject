#pragma once
#include <Arduino.h>

#include "config/Channels.h"
#include "config/Config.h"
#include "copter/CopterController.h"
#include "copter/Presets.h"
#include "copter/RcSetup.h"

// ============================================================
// НАСТРОЙКИ МУЛЬТИКОПТЕРА (прошивка VEHICLE_MULTIROTOR:
// env matek-h743-quad, stm32h743-devebox-quad)
//
// PID, фильтры, холостые, расходы здесь не задаются: прошивка выводит
// их сама из описания коптера (copter/Presets.h — модель моторов и
// винтов, запасы контура проверены симуляцией). Описать коптер:
//
//   1. Взять ближайшую сборку из каталога (COPTER_BUILD ниже):
//        whoop65, whoop75     — вупы 65/75 мм, 1S
//        toothpick25          — 2.5", 3S
//        micro3, cinewhoop3   — 3", 4S
//        freestyle5, race5    — 5", 6S
//        longrange7           — 7", 6S
//        longrange7li         — 7" на Li-ion 6S (2810 1300KV, 7040, 4000 мА·ч)
//        heavy10              — 10", 6S, грузовой
//        cinelifter8          — X8 с 8" винтами
//        hex13, hex15         — шестилучевые с 13" и 15"
//   2. Поправить то, что у вашего коптера другое (блок «Свой коптер»,
//      0 — как в сборке): винт, мотор, батарея, масса, рама.
//      Главное — МАССА с батареей: взвесить.
//   3. После прошивки — 't' в консоли: что вывелось (тяговооружённость,
//      газ висения, полоса контура) и предупреждения (мотор не тянет
//      винт, винты задевают, мало тяги).
//
// Сборку можно выбрать и без правки файла: -D COPTER_BUILD=\"race5\" в
// build_flags.
//
// Поверх выведенного — настройки из CLI, как в Betaflight ('#' в консоли:
// set p_roll = 50, profile 1, tune_source = BETAFLIGHT, save): они во
// флеше и переживают перепрошивку; что не задано — из описания коптера.
// Описание тоже можно сменить без пересборки: set copter_build = race5,
// set copter_mass_g = 742 — тогда этот файл — только умолчание.
// ============================================================

#ifndef COPTER_BUILD
#define COPTER_BUILD "freestyle5"
#endif

namespace CopterConfig
{
    // --------------------------------------------------------
    // Свой коптер (0 — взять из сборки COPTER_BUILD)
    // --------------------------------------------------------
    constexpr float MASS_G = 0;               // взлётная масса с батареей, г
    constexpr float WHEELBASE_MM = 0;         // между осями противоположных моторов
    constexpr float PROP_DIAMETER_IN = 0;     // 5.1 — винт 5.1"
    constexpr float PROP_PITCH_IN = 0;        // 4.3
    constexpr uint8_t PROP_BLADES = 0;
    constexpr float MOTOR_STATOR_MM = 0;      // 2207 -> 22
    constexpr float MOTOR_HEIGHT_MM = 0;      // 2207 -> 7
    constexpr float MOTOR_KV = 0;
    constexpr uint8_t BATTERY_CELLS = 0;
    constexpr uint16_t BATTERY_MAH = 0;

    // Пропеллеры наружу (верх лопасти уходит от рамы): все моторы
    // крутятся наоборот. Направление каждого мотора меняется в
    // конфигураторе регулятора (BLHeliSuite, ESC-Configurator), а не
    // перестановкой проводов — проверка: 'm' в консоли.
    constexpr bool PROPS_OUT = false;

    // --------------------------------------------------------
    // Регуляторы
    // --------------------------------------------------------
    // PWM — любой регулятор (и самолётный); ONESHOT125/ONESHOT42/
    // MULTISHOT — BLHeli_S, BLHeli_32, AM32 (быстрее отклик — шире
    // контур). DShot — следующий этап (нужен драйвер с DMA).
    // Плата может задать свой: -D COPTER_ESC_PROTOCOL=MULTISHOT в env.
#ifndef COPTER_ESC_PROTOCOL
#define COPTER_ESC_PROTOCOL ONESHOT125
#endif
    constexpr copter::EscProtocol ESC_PROTOCOL = copter::EscProtocol::COPTER_ESC_PROTOCOL;

    // Цикл стабилизации. 1000 Гц — частота тика FreeRTOS; вупам (вращение
    // выше 500 Гц) нужен цикл быстрее — следующий этап.
    constexpr float LOOP_HZ = 1000.0f;

    // Airmode: на нулевом газе коптер управляется (флипы, спуск на сброшенном газе).
    constexpr bool AIRMODE = true;

    // --------------------------------------------------------
    // Пульт (каналы — config/Channels.h, Mode 2)
    // --------------------------------------------------------
    // CH1 крен, CH2 тангаж (стик от себя — нос вниз), CH3 газ, CH4 рысканье.
    // ARM — SwA (CH5), как у самолёта. Режим — SwC (CH7), три положения.
    // Это умолчание: таблица режимов в CLI (`aux`, как в Betaflight) его
    // заменяет целиком, другой порядок каналов — `map TAER1234`.
    constexpr uint8_t MODE_CHANNEL = Channels::SWC;
    constexpr copter::CopterMode MODE_UP = copter::CopterMode::ANGLE;       // CH7 ~1000
    constexpr copter::CopterMode MODE_MIDDLE = copter::CopterMode::ALT_HOLD; // ~1500
    constexpr copter::CopterMode MODE_DOWN = copter::CopterMode::ACRO;       // ~2000
    static_assert(Channels::ARM >= 4 && MODE_CHANNEL >= 4, "ARM и режим — на AUX (CH5 и дальше), не на стиках");

    // --------------------------------------------------------
    // Выходы на моторы — по порядку номеров Betaflight (Frames.h):
    // квадро 1 — задний правый, 2 — передний правый, 3 — задний левый,
    // 4 — передний левый.
    // --------------------------------------------------------
#if defined(BOARD_MATEK_H743)
    // Площадки S1–S8: S1/S2 — TIM1, S3–S6 — TIM2, S7/S8 — TIM4.
    constexpr int16_t MOTOR_PINS[] = { PB0, PB1, PA0, PA1, PA2, PA3, PD12, PD13 };
#elif defined(BOARD_STM32F405)
    // Полётник под Betaflight: M1…M8 — как в заголовке платы (config/boards/*.h).
    constexpr int16_t MOTOR_PINS[] = { OPB_MOTOR_PINS };
#elif defined(BOARD_STM32H743)
    // Сервовыходы самолётной распиновки: PA0–PA3 (TIM2), PD14/PD15 (TIM4), PE9 (TIM1).
    constexpr int16_t MOTOR_PINS[] = { PA0, PA1, PA2, PA3, PD14, PD15, PE9 };
#else
    constexpr int16_t MOTOR_PINS[] = { -1 };
#endif
    constexpr uint8_t MOTOR_PIN_COUNT = sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]);

    // AF таймера каждого выхода (DShot берёт тот же таймер, что Betaflight на
    // этой плате, — по строкам timer дампа); −1 — первый таймер пина.
#if defined(BOARD_STM32F405) && defined(OPB_MOTOR_AF)
    constexpr int8_t MOTOR_AF[] = { OPB_MOTOR_AF };
#else
    constexpr int8_t MOTOR_AF[] = { -1 };
#endif
    constexpr uint8_t MOTOR_AF_COUNT = sizeof(MOTOR_AF) / sizeof(MOTOR_AF[0]);

    // Поправки «своего коптера» к сборке (и к выбранной в CLI copter_build).
    inline void applyOverrides(copter::BuildSpec& out)
    {
        if (MASS_G > 0) out.massG = MASS_G;
        if (WHEELBASE_MM > 0) out.wheelbaseMm = WHEELBASE_MM;
        if (PROP_DIAMETER_IN > 0) out.prop.diameterIn = PROP_DIAMETER_IN;
        if (PROP_PITCH_IN > 0) out.prop.pitchIn = PROP_PITCH_IN;
        if (PROP_BLADES > 0) out.prop.blades = PROP_BLADES;
        if (PROP_DIAMETER_IN > 0 || PROP_BLADES > 0) out.prop.massG = 0;   // масса винта — заново по размеру
        if (MOTOR_STATOR_MM > 0) out.motor.statorWidthMm = MOTOR_STATOR_MM;
        if (MOTOR_HEIGHT_MM > 0) out.motor.statorHeightMm = MOTOR_HEIGHT_MM;
        if (MOTOR_KV > 0) out.motor.kv = MOTOR_KV;
        if (BATTERY_CELLS > 0) out.battery.cells = BATTERY_CELLS;
        if (BATTERY_MAH > 0) out.battery.capacityMah = BATTERY_MAH;
        out.propsOut = PROPS_OUT;
    }

    // Сборка с поправками «своего коптера». false — в каталоге нет такой.
    inline bool build(copter::BuildSpec& out)
    {
        const copter::BuildSpec* base = copter::findBuild(COPTER_BUILD);
        if (base == nullptr) return false;
        out = *base;
        applyOverrides(out);
        return true;
    }

    inline copter::CopterSettings settings()
    {
        copter::CopterSettings s;
        s.airmode = AIRMODE;
        return s;
    }

    // Тумблеры выше в виде таблицы aux — работает, пока своя в CLI пуста.
    // Возвращает число строк.
    inline uint8_t defaultAux(copter::AuxRange* out, uint8_t capacity)
    {
        uint8_t n = 0;
        auto add = [&](uint8_t box, uint8_t channel, uint16_t from, uint16_t to) {
            if (n >= capacity || box >= copter::BOX_COUNT) return;
            copter::AuxRange& a = out[n++];
            a.mode = box;
            a.channel = static_cast<uint8_t>(channel - 4);
            a.startUs = from;
            a.endUs = to;
        };
        add(copter::BOX_ARM, Channels::ARM, 1700, 2100);
        add(copter::boxForMode(MODE_UP), MODE_CHANNEL, 900, 1300);
        add(copter::boxForMode(MODE_MIDDLE), MODE_CHANNEL, 1300, 1700);
        add(copter::boxForMode(MODE_DOWN), MODE_CHANNEL, 1700, 2100);
        return n;
    }
}
