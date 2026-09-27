// Реализация hal/Rtos.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/Rtos.h"

namespace Rtos
{

bool startTask(TaskFunction_t function, const char* name, uint32_t stackBytes, void* arg,
                          UBaseType_t priority)
{
#if defined(BOARD_STM32H743)
    const uint32_t words = stackBytes / sizeof(StackType_t);
    return xTaskCreate(function, name, static_cast<configSTACK_DEPTH_TYPE>(words), arg, priority, nullptr) == pdPASS;
#else
    // Ядро 0: ядро 1 целиком у полётного цикла (loopTask).
    return xTaskCreatePinnedToCore(function, name, stackBytes, arg, priority, nullptr, 0) == pdPASS;
#endif
}

uint32_t freeHeapBytes()
{
#if defined(BOARD_STM32H743)
    return static_cast<uint32_t>(xPortGetFreeHeapSize());
#else
    return ESP.getFreeHeap();
#endif
}

}  // namespace Rtos
