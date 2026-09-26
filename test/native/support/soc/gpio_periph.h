#pragma once

// Нативная замена soc/gpio_periph.h. На плате GPIO_PIN_MUX_REG[n] —
// адрес регистра IO_MUX пина n; здесь это просто номер пина, чтобы
// PIN_INPUT_ENABLE() (soc/io_mux_reg.h) знал, какой пин включать.

#include <cstdint>

#include "soc/soc_caps.h"

inline const uint32_t GPIO_PIN_MUX_REG[SOC_GPIO_PIN_COUNT] = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16,
    17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
    34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
};
