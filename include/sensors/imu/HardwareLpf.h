#pragma once
#include <stdint.h>

// ============================================================
// Фильтр в самом чипе гироскопа — gyro_hardware_lpf (пункт 065)
//
// Как у Betaflight: NORMAL / OPTION_1 / OPTION_2 / EXPERIMENTAL. Выбор
// читается при запуске IMU (begin), поэтому настройка действует после
// save (плата перезапускается). Только у коптера (Config::
// IMU_WIDE_BANDWIDTH): самолёт держит свой узкий фильтр чипа.
//
// NORMAL — полоса, на которой проверены наши драйверы (как было до 065).
// Остальные — шире и с меньшей задержкой, но пропускают больше шума и
// дают наложение частот (гироскоп читается 1 раз в цикл): включать,
// посмотрев спектр во вкладке «Фильтры». Где чип шире не умеет, пункт
// повторяет предыдущий.
//
// Таблица у каждого драйвера — полоса (Гц, примерно, по датащиту) и
// задержка (мкс, примерно). Задержка идёт в модель контура
// (CopterTune::gyroChipDelayS): у NORMAL модель всегда считала ~0.5 мс.
// Заголовок без Arduino — его включает и логика коптера (copter/Params.h).
// ============================================================

namespace imu
{
    enum class HardwareLpf : uint8_t
    {
        NORMAL = 0,
        OPTION_1 = 1,
        OPTION_2 = 2,
        EXPERIMENTAL = 3,
    };
    constexpr uint8_t HARDWARE_LPF_COUNT = 4;

    struct HardwareLpfSpec
    {
        uint16_t hz;        // полоса −3 дБ, примерно
        uint16_t delayUs;   // задержка, примерно
    };

    // Задержка фильтра чипа в модели, когда чип неизвестен (как до 065).
    constexpr float DEFAULT_CHIP_DELAY_S = 0.0005f;

    namespace detail
    {
        inline HardwareLpf& hardwareLpfChoice()
        {
            static HardwareLpf choice = HardwareLpf::NORMAL;
            return choice;
        }
    }

    // Выбор из настройки — до begin() датчика (склейка коптера).
    inline void setHardwareLpf(uint8_t index)
    {
        detail::hardwareLpfChoice() = index < HARDWARE_LPF_COUNT ? static_cast<HardwareLpf>(index) : HardwareLpf::NORMAL;
    }

    // Пункт для драйвера: у самолёта (wide = false) всегда NORMAL.
    inline uint8_t hardwareLpfIndex(bool wide)
    {
        return wide ? static_cast<uint8_t>(detail::hardwareLpfChoice()) : 0;
    }
}
