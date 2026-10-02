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
    // Ненулевой и у каждой задачи свой — как настоящий хэндл.
    if (handle) *handle = reinterpret_cast<TaskHandle_t>(fake::tasks().created.size());
    return pdPASS;
}

inline void vTaskStartScheduler() { fake::schedulerStarted() = true; }

#define taskSCHEDULER_RUNNING 2
#define taskSCHEDULER_NOT_STARTED 1
inline BaseType_t xTaskGetSchedulerState() { return fake::schedulerStarted() ? taskSCHEDULER_RUNNING : taskSCHEDULER_NOT_STARTED; }

// Критические секции STM32 FreeRTOS — без аргумента (у ESP32 — portMUX).
#define taskENTER_CRITICAL() (++::fake::tasks().criticalEntries, ++::fake::tasks().criticalDepth)
#define taskEXIT_CRITICAL() (--::fake::tasks().criticalDepth)

inline size_t xPortGetFreeHeapSize() { return fake::chip().freeHeap; }
