#pragma once
#include <Arduino.h>

#if !defined(BOARD_STM32H743)
#include <esp_system.h>
#endif

// ============================================================
// Причина последней перезагрузки — одинаково на ESP32 и STM32.
// Чёрный ящик по ней решает, начинать ли запись сразу: перезагрузка
// из-за сбоя в воздухе — это как раз то, что нужно записать.
//
// ESP32 — esp_reset_reason().
// STM32H743 — флаги RCC->RSR. Читать нужно один раз при включении: флаги
// сбрасываются здесь же (RMVF), иначе они остались бы и после следующей
// перезагрузки. PINRSTF у H7 выставляется при ЛЮБОМ сбросе (внутренние
// источники тянут NRST), поэтому проверяются сначала более
// конкретные причины.
// ============================================================

enum class ResetCause : uint8_t
{
    Unknown,
    PowerOn,
    External,
    Software,
    Panic,          // ESP32: исключение / abort
    IntWatchdog,    // ESP32
    TaskWatchdog,   // ESP32
    Watchdog,       // ESP32 RTC WDT; STM32 IWDG/WWDG
    DeepSleep,
    Brownout,
    Sdio,
};

inline bool isCrashReset(ResetCause cause)
{
    return cause == ResetCause::Panic || cause == ResetCause::IntWatchdog || cause == ResetCause::TaskWatchdog ||
           cause == ResetCause::Watchdog || cause == ResetCause::Brownout;
}

// Имена — как у констант ESP-IDF (ESP_RST_*): их видно в параметрах полёта.
inline const char* resetCauseName(ResetCause cause)
{
    switch (cause)
    {
        case ResetCause::PowerOn:      return "POWERON";
        case ResetCause::External:     return "EXT";
        case ResetCause::Software:     return "SW";
        case ResetCause::Panic:        return "PANIC";
        case ResetCause::IntWatchdog:  return "INT_WDT";
        case ResetCause::TaskWatchdog: return "TASK_WDT";
        case ResetCause::Watchdog:     return "WDT";
        case ResetCause::DeepSleep:    return "DEEPSLEEP";
        case ResetCause::Brownout:     return "BROWNOUT";
        case ResetCause::Sdio:         return "SDIO";
        case ResetCause::Unknown:      break;
    }
    return "UNKNOWN";
}

#if defined(BOARD_STM32H743)

inline ResetCause readResetCause()
{
    const uint32_t flags = RCC->RSR;
    RCC->RSR |= RCC_RSR_RMVF;

    if (flags & (RCC_RSR_IWDG1RSTF | RCC_RSR_WWDG1RSTF)) return ResetCause::Watchdog;
    if (flags & RCC_RSR_PORRSTF) return ResetCause::PowerOn;
    if (flags & RCC_RSR_BORRSTF) return ResetCause::Brownout;
    if (flags & RCC_RSR_SFTRSTF) return ResetCause::Software;
    if (flags & RCC_RSR_PINRSTF) return ResetCause::External;
    return ResetCause::Unknown;
}

#else

inline ResetCause readResetCause()
{
    switch (esp_reset_reason())
    {
        case ESP_RST_POWERON:   return ResetCause::PowerOn;
        case ESP_RST_EXT:       return ResetCause::External;
        case ESP_RST_SW:        return ResetCause::Software;
        case ESP_RST_PANIC:     return ResetCause::Panic;
        case ESP_RST_INT_WDT:   return ResetCause::IntWatchdog;
        case ESP_RST_TASK_WDT:  return ResetCause::TaskWatchdog;
        case ESP_RST_WDT:       return ResetCause::Watchdog;
        case ESP_RST_DEEPSLEEP: return ResetCause::DeepSleep;
        case ESP_RST_BROWNOUT:  return ResetCause::Brownout;
        case ESP_RST_SDIO:      return ResetCause::Sdio;
        default:                return ResetCause::Unknown;
    }
}

#endif
