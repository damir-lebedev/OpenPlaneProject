#pragma once

// Нативная замена soc/io_mux_reg.h: PIN_INPUT_ENABLE() включает
// входной буфер пина в симулированном GPIO (fake::gpio()).

#include "Arduino.h"

#define PIN_INPUT_ENABLE(PIN_NAME) (::fake::gpio().inputEnabled[(PIN_NAME)] = true)
