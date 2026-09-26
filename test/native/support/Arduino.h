#pragma once

// ============================================================
// Нативная (ПК) замена Arduino.h из Arduino core ESP32 2.0.x.
//
// Нужна двум вещам:
//   • нативным тестам (pio test -e native) — заголовки прошивки
//     собираются на ПК без изменений, а железо заменено
//     управляемыми фейками (namespace fake);
//   • статическому анализу (clang-tidy), который не разбирает
//     заголовки ESP-IDF под хост-архитектуру.
//
// Макросы и сигнатуры повторяют настоящий Arduino.h (constrain и sq
// — макросы, min/max/abs — из std, map() — целочисленный). Макрос
// ARDUINO здесь намеренно НЕ определяется: по нему тесты отличают
// сборку на плате (setup/loop) от нативной (main).
// ============================================================

// Как настоящий Arduino.h: стандартные заголовки C/C++ — часть его
// API (скетчи и библиотеки берут uint8_t, fabsf, snprintf и т.п.
// из него).
#include <algorithm>  // IWYU pragma: export
#include <cmath>      // IWYU pragma: export
#include <cstdarg>    // IWYU pragma: export
#include <cstddef>    // IWYU pragma: export
#include <cstdint>    // IWYU pragma: export
#include <cstdio>     // IWYU pragma: export
#include <cstdlib>    // IWYU pragma: export
#include <cstring>    // IWYU pragma: export
#include <math.h>     // IWYU pragma: export
#include <stdint.h>   // IWYU pragma: export
#include <stdio.h>    // IWYU pragma: export
#include <stdlib.h>   // IWYU pragma: export
#include <string.h>   // IWYU pragma: export

#include "Print.h"            // IWYU pragma: export
#include "Stream.h"           // IWYU pragma: export
#include "WString.h"          // IWYU pragma: export
#include "HardwareSerial.h"   // IWYU pragma: export
#include "esp32-hal-fake.h"   // IWYU pragma: export

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))

#define PROGMEM
#define PGM_P const char*

typedef uint8_t byte;
typedef bool boolean;

using std::abs;
using std::isinf;
using std::isnan;
using std::max;
using std::min;
using std::round;

// Как WMath.cpp: целочисленная арифметика, пустой входной диапазон — -1.
inline long map(long x, long inMin, long inMax, long outMin, long outMax)
{
    const long run = inMax - inMin;
    if (run == 0) return -1;
    return (x - inMin) * (outMax - outMin) / run + outMin;
}

inline bool isDigit(int c) { return c >= '0' && c <= '9'; }
