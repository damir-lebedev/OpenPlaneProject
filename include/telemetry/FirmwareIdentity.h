#pragma once
#include <Arduino.h>

// ============================================================
// ПАСПОРТ ПРОШИВКИ — для приложения OpenPlane Studio
//
// ПК шлёт STX (0x02), "id", '\n' — так же, как команды "bb" чёрного
// ящика (telemetry/DebugConsole.h), — плата отвечает одной строкой:
//
//   @id {"fw":"openplane","proto":1,"board":"stm32h743-devebox",
//        "commit":"75d8784","config":"3f9a…","plane":"Astro-Cargo #1",
//        "armed":false}
//
// proto — версия этого обмена: растёт, когда меняются поля.
// board, commit, config и plane вшивает приложение при сборке — пишет
// config/Project.h в свою копию исходников (tools/opconfig.py apply). В
// репозитории этого файла нет: тогда board — по макросу платы,
// commit = "dev", config и plane пустые.
// ============================================================

#if __has_include("config/Project.h")
#include "config/Project.h"
#endif

#ifndef OP_BOARD_ENV
    #if defined(BOARD_STM32H743)
        #define OP_BOARD_ENV "stm32h743"
    #elif defined(BOARD_ESP32_S3)
        #define OP_BOARD_ENV "esp32-s3"
    #elif defined(BOARD_ESP32_C3)
        #define OP_BOARD_ENV "esp32-c3"
    #elif defined(BOARD_ESP32_CLASSIC)
        #define OP_BOARD_ENV "esp32-dev"
    #else
        #define OP_BOARD_ENV ""
    #endif
#endif
#ifndef OP_SOURCE_COMMIT
#define OP_SOURCE_COMMIT "dev"
#endif
#ifndef OP_CONFIG_HASH
#define OP_CONFIG_HASH ""
#endif
#ifndef OP_PLANE_NAME
#define OP_PLANE_NAME ""
#endif

namespace FirmwareIdentity
{
    constexpr int PROTOCOL_VERSION = 1;

    // Строка JSON в кавычках: " и \ экранируются, управляющие символы
    // пропускаются, UTF-8 (кириллица в имени самолёта) — как есть.
    inline void printJsonString(const char* text)
    {
        Serial.print('"');
        for (const char* p = text; *p; ++p)
        {
            const char c = *p;
            if (c == '"' || c == '\\')
            {
                Serial.print('\\');
                Serial.print(c);
            }
            else if (static_cast<unsigned char>(c) >= 0x20)
            {
                Serial.print(c);
            }
        }
        Serial.print('"');
    }

    inline void print(bool armed, const char* board = OP_BOARD_ENV, const char* commit = OP_SOURCE_COMMIT,
                      const char* config = OP_CONFIG_HASH, const char* plane = OP_PLANE_NAME)
    {
        Serial.print("@id {\"fw\":\"openplane\",\"proto\":");
        Serial.print(PROTOCOL_VERSION);
        Serial.print(",\"board\":");
        printJsonString(board);
        Serial.print(",\"commit\":");
        printJsonString(commit);
        Serial.print(",\"config\":");
        printJsonString(config);
        Serial.print(",\"plane\":");
        printJsonString(plane);
        Serial.print(",\"armed\":");
        Serial.print(armed ? "true" : "false");
        Serial.println("}");
    }
}
