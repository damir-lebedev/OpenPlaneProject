#pragma once

// ============================================================
// Нативная замена библиотеки STM32duino FreeRTOS: задачи
// записываются в тот же реестр, что у фейка ESP32
// (fake::tasks(), fake::runTask()); стек — в словах, как у ванильного
// FreeRTOS. vTaskStartScheduler() в тесте возвращается сразу —
// задачи тест запускает сам (fake::runTask()).
// ============================================================

#include <Arduino.h>

typedef uint32_t StackType_t;
#define configSTACK_DEPTH_TYPE uint16_t

namespace fake
{
    inline bool& schedulerStarted()
    {
        static bool started = false;
        return started;
    }
}

inline BaseType_t xTaskCreate(TaskFunction_t function, const char* name, configSTACK_DEPTH_TYPE stackWords,
                              void* parameter, UBaseType_t priority, TaskHandle_t* handle)
{
    fake::TaskRecord record;
    record.function = function;
    record.name = name ? name : "";
    record.stackDepth = static_cast<uint32_t>(stackWords) * sizeof(StackType_t);   // в байтах, как у ESP32
    record.parameter = parameter;
    record.priority = priority;
    record.core = -1;
    fake::tasks().created.push_back(record);
    if (handle) *handle = nullptr;
    return pdPASS;
}

inline void vTaskStartScheduler() { fake::schedulerStarted() = true; }

inline size_t xPortGetFreeHeapSize() { return fake::chip().freeHeap; }
