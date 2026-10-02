#pragma once

// ============================================================
// Тонкая прослойка над FreeRTOS: одинаковый код задач на ESP32 и STM32.
//
// ESP32 — FreeRTOS встроен в ядро Arduino, loop() уже крутится
// задачей на ядре 1, фоновые задачи (экран, веб) — на ядре 0.
// Размер стека в xTaskCreate* у ESP-IDF — в байтах.
//
// STM32H743 — одно ядро, FreeRTOS из библиотеки STM32duino FreeRTOS
// (platformio.ini, env stm32h743). Полётная задача получает высший
// приоритет и вытесняет фоновые (экран, телеметрия, запись флеша) —
// они работают только в паузах полётного цикла. Стек у ванильного
// FreeRTOS — в словах (StackType_t), перевод здесь.
// ============================================================

#if defined(BOARD_STM32H743)
#include <STM32FreeRTOS.h>
#else
#include <Arduino.h>
#endif

namespace Rtos
{
    // Приоритеты (0 — idle). На STM32 configMAX_PRIORITIES = 7.
    constexpr UBaseType_t PRIORITY_BACKGROUND = 1;   // экран, запись флеша
    constexpr UBaseType_t PRIORITY_TELEMETRY = 2;    // радиомодем
    constexpr UBaseType_t PRIORITY_FLIGHT = 5;       // полётный цикл (только STM32)

    // handle — куда положить хэндл задачи (для xTaskNotifyGive), можно nullptr.
    inline bool startTask(TaskFunction_t function, const char* name, uint32_t stackBytes, void* arg,
                          UBaseType_t priority = PRIORITY_BACKGROUND, TaskHandle_t* handle = nullptr)
    {
#if defined(BOARD_STM32H743)
        const uint32_t words = stackBytes / sizeof(StackType_t);
        return xTaskCreate(function, name, static_cast<configSTACK_DEPTH_TYPE>(words), arg, priority, handle) == pdPASS;
#else
        // Ядро 0: ядро 1 целиком у полётного цикла (loopTask).
        return xTaskCreatePinnedToCore(function, name, stackBytes, arg, priority, handle, 0) == pdPASS;
#endif
    }

    // Подождать ms, отдав процессор другим задачам. До запуска планировщика
    // (setup() на STM32) — обычный delay().
    inline void sleepMs(uint32_t ms)
    {
#if defined(BOARD_STM32H743)
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        {
            vTaskDelay(pdMS_TO_TICKS(ms));
        }
        else
        {
            delay(ms);
        }
#else
        vTaskDelay(pdMS_TO_TICKS(ms));
#endif
    }

    // Короткая критическая секция между задачами (и ядрами ESP32): внутри —
    // только копирование нескольких байт, никакого ввода-вывода.
    class CriticalSection
    {
    public:
        void enter()
        {
#if defined(BOARD_STM32H743)
            taskENTER_CRITICAL();
#else
            portENTER_CRITICAL(&mux);
#endif
        }

        void exit()
        {
#if defined(BOARD_STM32H743)
            taskEXIT_CRITICAL();
#else
            portEXIT_CRITICAL(&mux);
#endif
        }

#if !defined(BOARD_STM32H743)
    private:
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif
    };

    // Свободная куча, байт — для строки состояния в логе.
    inline uint32_t freeHeapBytes()
    {
#if defined(BOARD_STM32H743)
        return static_cast<uint32_t>(xPortGetFreeHeapSize());
#else
        return ESP.getFreeHeap();
#endif
    }
}
