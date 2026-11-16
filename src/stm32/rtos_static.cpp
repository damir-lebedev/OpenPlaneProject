// ============================================================
// Память задач idle и таймеров FreeRTOS, когда включено статическое
// создание задач (configSUPPORT_STATIC_ALLOCATION = 1 — env F405:
// стеки коптера лежат в CCM-ОЗУ, hal/Rtos.h startTaskStatic). Без этих
// двух функций FreeRTOS не соберётся. Размеры — как у динамических.
// Отдельный .cpp: функции с внешней связью, нативные тесты их не собирают.
// ============================================================

#include <Arduino.h>

#include "hal/Platform.h"

#if defined(ARDUINO_ARCH_STM32) && defined(OP_STM32)
#include <STM32FreeRTOS.h>

#if configSUPPORT_STATIC_ALLOCATION

// configMINIMAL_STACK_SIZE у STM32duino — адрес символа линкера (_Min_Stack_Size / 8),
// размером массива быть не может; 0x400 / 8 = 128 слов, таймеры — вдвое больше.
constexpr uint32_t IDLE_STACK_WORDS = 128;
constexpr uint32_t TIMER_STACK_WORDS = 256;

extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t** tcb, StackType_t** stack, uint32_t* words)
{
    static StaticTask_t idleTcb;
    static StackType_t idleStack[IDLE_STACK_WORDS];
    *tcb = &idleTcb;
    *stack = idleStack;
    *words = IDLE_STACK_WORDS;
}

#if configUSE_TIMERS
extern "C" void vApplicationGetTimerTaskMemory(StaticTask_t** tcb, StackType_t** stack, uint32_t* words)
{
    static StaticTask_t timerTcb;
    static StackType_t timerStack[TIMER_STACK_WORDS];
    *tcb = &timerTcb;
    *stack = timerStack;
    *words = TIMER_STACK_WORDS;
}
#endif

#endif
#endif
