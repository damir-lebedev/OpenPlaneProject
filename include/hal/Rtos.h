#pragma once

// ============================================================
// Тонкая прослойка над FreeRTOS: одинаковый код задач на ESP32 и STM32.
//
// ESP32 — FreeRTOS встроен в ядро Arduino, loop() уже крутится
// задачей на ядре 1, фоновые задачи (экран, веб) — на ядре 0.
// Размер стека в xTaskCreate* у ESP-IDF — в байтах.
//
// STM32 (H743, F405) — одно ядро, FreeRTOS из библиотеки STM32duino
// FreeRTOS (platformio.ini). Полётная задача получает высший
// приоритет и вытесняет фоновые (экран, телеметрия, запись флеша) —
// они работают только в паузах полётного цикла. Стек у ванильного
// FreeRTOS — в словах (StackType_t), перевод здесь.
// ============================================================

#include "hal/Platform.h"

#if defined(OP_STM32)
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
#if defined(OP_STM32)
        const uint32_t words = stackBytes / sizeof(StackType_t);
        return xTaskCreate(function, name, static_cast<configSTACK_DEPTH_TYPE>(words), arg, priority, handle) == pdPASS;
#else
        // Ядро 0: ядро 1 целиком у полётного цикла (loopTask).
        return xTaskCreatePinnedToCore(function, name, stackBytes, arg, priority, handle, 0) == pdPASS;
#endif
    }

#if defined(OP_STM32) && configSUPPORT_STATIC_ALLOCATION
    // Задача со стеком в заданной памяти (F405: CCM-ОЗУ — куча в основных
    // 128 КБ не вмещает стеки коптера). TCB — из пула здесь. Нужен
    // configSUPPORT_STATIC_ALLOCATION = 1 (platformio.ini, env F405) и
    // память задач idle/таймеров (src/stm32/rtos_static.cpp).
    inline bool startTaskStatic(TaskFunction_t function, const char* name, StackType_t* stack, uint32_t stackBytes,
                                void* arg, UBaseType_t priority = PRIORITY_BACKGROUND, TaskHandle_t* handle = nullptr)
    {
        static StaticTask_t tcbs[6];
        static uint8_t used = 0;
        if (used >= sizeof(tcbs) / sizeof(tcbs[0])) return false;
        TaskHandle_t h = xTaskCreateStatic(function, name, stackBytes / sizeof(StackType_t), arg, priority, stack, &tcbs[used]);
        if (!h) return false;
        ++used;
        if (handle) *handle = h;
        return true;
    }

    // Нарезка области памяти под стеки (по 8 байт, как требует ARM EABI).
    class StackArena
    {
    public:
        StackArena(uintptr_t base, uint32_t bytes) : next(base), end(base + bytes) {}

        StackType_t* take(uint32_t bytes)
        {
            bytes = (bytes + 7u) & ~7u;
            if (next + bytes > end) return nullptr;
            StackType_t* p = reinterpret_cast<StackType_t*>(next);
            next += bytes;
            return p;
        }

        uint32_t left() const { return static_cast<uint32_t>(end - next); }

    private:
        uintptr_t next;
        uintptr_t end;
    };
#endif

    // Подождать ms, отдав процессор другим задачам. До запуска планировщика
    // (setup() на STM32) — обычный delay().
    inline void sleepMs(uint32_t ms)
    {
#if defined(OP_STM32)
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
#if defined(OP_STM32)
            taskENTER_CRITICAL();
#else
            portENTER_CRITICAL(&mux);
#endif
        }

        void exit()
        {
#if defined(OP_STM32)
            taskEXIT_CRITICAL();
#else
            portEXIT_CRITICAL(&mux);
#endif
        }

#if !defined(OP_STM32)
    private:
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif
    };

    // Рекурсивный мьютекс (шина I2C, общая у задач экрана и полёта).
    // create() — один раз, до задач. На STM32 до запуска планировщика
    // (setup()) задача одна — take()/give() там ничего не делают.
    class RecursiveMutex
    {
    public:
        void create()
        {
            if (!handle) handle = xSemaphoreCreateRecursiveMutex();
        }

        void take()
        {
            if (handle && running()) xSemaphoreTakeRecursive(handle, portMAX_DELAY);
        }

        void give()
        {
            if (handle && running()) xSemaphoreGiveRecursive(handle);
        }

    private:
        SemaphoreHandle_t handle = nullptr;

        static bool running()
        {
#if defined(OP_STM32)
            return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
#else
            return true;
#endif
        }
    };

    // Свободная куча, байт — для строки состояния в логе.
    inline uint32_t freeHeapBytes()
    {
#if defined(OP_STM32)
        return static_cast<uint32_t>(xPortGetFreeHeapSize());
#else
        return ESP.getFreeHeap();
#endif
    }
}
