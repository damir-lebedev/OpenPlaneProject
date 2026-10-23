// ============================================================
// Перезагрузка в системный загрузчик STM32 (USB DFU) прямо из прошивки:
// консольная клавиша 'D' (telemetry/DebugConsole.h). Без этого каждую
// перепрошивку по USB пришлось бы начинать с проводка BT0 -> 3V3 и RST
// (или кнопки BOOT на полётнике).
//
// Прыжок прямо из работающей прошивки (с FreeRTOS, USB, кэшами, PLL) —
// ненадёжен: загрузчик получает периферию в произвольном состоянии и
// зависает. Поэтому две ступени, как у ST в примерах:
//   1) stm32RebootToBootloader() кладёт метку в ОЗУ, которое линкер не
//      использует и программный сброс не стирает, и делает
//      NVIC_SystemReset();
//   2) конструктор ниже выполняется сразу после сброса, до main() и до
//      настройки тактирования, в состоянии "как после включения": если
//      метка стоит — стирает её и прыгает в системную память.
//
//   H743 — метка в последнем слове DTCM, загрузчик с 0x1FF09800;
//   F405 — метка в последнем слове CCM-ОЗУ (64 КБ с 0x10000000; ядро его не
//          занимает), загрузчик с 0x1FFF0000; перед прыжком системная память
//          отображается на адрес 0 (SYSCFG MEMRMP), как требует F4.
// Если прыжок не удался, достаточно нажать RST.
//
// Отдельный .cpp: функции с внешней связью, нативные тесты их не собирают.
// ============================================================

#include <Arduino.h>

#include "hal/Platform.h"

#if defined(ARDUINO_ARCH_STM32) && defined(OP_STM32)

namespace
{
    constexpr uint32_t MAGIC = 0xB007DF11u;

#if defined(BOARD_STM32H743)
    constexpr uint32_t SYSTEM_BOOTLOADER = 0x1FF09800;   // STM32H742/743/753
    // Последнее слово DTCM (128 КБ с 0x20000000): линкер его не занимает.
    volatile uint32_t* const flag = reinterpret_cast<volatile uint32_t*>(0x2001FFFCu);
#else
    constexpr uint32_t SYSTEM_BOOTLOADER = 0x1FFF0000;   // STM32F405/407/415/417
    // Последнее слово CCM-ОЗУ: секций там нет, стек и куча — в основном ОЗУ.
    volatile uint32_t* const flag = reinterpret_cast<volatile uint32_t*>(0x1000FFFCu);
#endif

    __attribute__((constructor(101))) void jumpToBootloaderIfRequested()
    {
        if (*flag != MAGIC) return;
        *flag = 0;

#if defined(BOARD_STM32F405)
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
        (void)RCC->APB2ENR;
        SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;   // системная память — на адрес 0
#endif
        const uint32_t stack = *reinterpret_cast<const volatile uint32_t*>(SYSTEM_BOOTLOADER);
        const uint32_t entry = *reinterpret_cast<const volatile uint32_t*>(SYSTEM_BOOTLOADER + 4);
        SCB->VTOR = SYSTEM_BOOTLOADER;
        __set_MSP(stack);
        __DSB();
        __ISB();
        reinterpret_cast<void (*)()>(entry)();
        for (;;) {}
    }
}

void stm32RebootToBootloader()
{
    Serial.flush();
    delay(20);   // дать последним байтам уйти в USB

    __disable_irq();
    *flag = MAGIC;
    __DSB();
    NVIC_SystemReset();
}

#endif
