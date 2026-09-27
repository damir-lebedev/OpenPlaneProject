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

    bool startTask(TaskFunction_t function, const char* name, uint32_t stackBytes, void* arg,
                          UBaseType_t priority = PRIORITY_BACKGROUND);

    // Свободная куча, байт — для строки состояния в логе.
    uint32_t freeHeapBytes();
}
