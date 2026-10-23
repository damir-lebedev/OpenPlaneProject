#pragma once
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

// ============================================================
// Строка JSON в буфер — для строк приложению ("@live {...}",
// "@param {...}"): дописывает с проверкой границ; не влезло —
// overflow, строку целиком не отправлять.
// ============================================================

struct JsonWriter
{
    JsonWriter(char* out, size_t size) : buffer(out), capacity(size)
    {
        if (capacity) buffer[0] = '\0';
    }

    char* buffer;
    size_t capacity;
    size_t length = 0;
    bool overflow = false;

    void add(const char* format, ...) __attribute__((format(printf, 2, 3)))
    {
        if (overflow) return;
        va_list args;
        va_start(args, format);
        const int n = vsnprintf(buffer + length, capacity - length, format, args);
        va_end(args);
        if (n < 0 || static_cast<size_t>(n) >= capacity - length)
        {
            overflow = true;
            return;
        }
        length += static_cast<size_t>(n);
    }

    // Строка JSON: " и \ экранируются, управляющие символы — пропуск,
    // UTF-8 (причины ARM по-русски) — как есть.
    void str(const char* text)
    {
        add("\"");
        for (const char* p = text; *p && !overflow; ++p)
        {
            const char c = *p;
            if (c == '"' || c == '\\') add("\\%c", c);
            else if (static_cast<unsigned char>(c) >= 0x20) add("%c", c);
        }
        add("\"");
    }
};
