#pragma once

// ============================================================
// Нативная замена Print/Printable из Arduino core ESP32 2.0.x.
//
// Набор перегрузок print()/println() и форматирование чисел
// повторяют настоящий Print.cpp (printNumber/printFloat), чтобы код
// прошивки выбирал те же перегрузки и печатал то же, что на плате.
// ============================================================

#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class String;
class Print;

class Printable
{
public:
    virtual ~Printable() = default;
    virtual size_t printTo(Print& p) const = 0;
};

class Print
{
public:
    virtual ~Print() = default;

    virtual size_t write(uint8_t c) = 0;

    virtual size_t write(const uint8_t* buffer, size_t size)
    {
        size_t n = 0;
        while (size--)
        {
            n += write(*buffer++);
        }
        return n;
    }

    size_t write(const char* str)
    {
        return str ? write(reinterpret_cast<const uint8_t*>(str), strlen(str)) : 0;
    }

    size_t write(const char* buffer, size_t size)
    {
        return write(reinterpret_cast<const uint8_t*>(buffer), size);
    }

    size_t printf(const char* format, ...) __attribute__((format(printf, 2, 3)))
    {
        va_list args;
        va_start(args, format);
        va_list copy;
        va_copy(copy, args);
        const int length = vsnprintf(nullptr, 0, format, copy);
        va_end(copy);
        if (length < 0)
        {
            va_end(args);
            return 0;
        }
        std::string text(static_cast<size_t>(length) + 1, '\0');
        vsnprintf(&text[0], text.size(), format, args);
        va_end(args);
        return write(reinterpret_cast<const uint8_t*>(text.data()), static_cast<size_t>(length));
    }

    size_t print(const String& s);
    size_t print(const char str[]) { return write(str); }
    size_t print(char c) { return write(static_cast<uint8_t>(c)); }
    size_t print(unsigned char n, int base = DEC) { return print(static_cast<unsigned long>(n), base); }
    size_t print(int n, int base = DEC) { return print(static_cast<long>(n), base); }
    size_t print(unsigned int n, int base = DEC) { return print(static_cast<unsigned long>(n), base); }

    size_t print(long n, int base = DEC)
    {
        if (base == 0) return write(static_cast<uint8_t>(n));
        if (base == 10 && n < 0)
        {
            const size_t t = print('-');
            return t + printNumber(static_cast<unsigned long>(-n), 10);
        }
        return printNumber(static_cast<unsigned long>(n), static_cast<uint8_t>(base));
    }

    size_t print(unsigned long n, int base = DEC)
    {
        if (base == 0) return write(static_cast<uint8_t>(n));
        return printNumber(n, static_cast<uint8_t>(base));
    }

    size_t print(long long n, int base = DEC) { return print(static_cast<long>(n), base); }
    size_t print(unsigned long long n, int base = DEC) { return print(static_cast<unsigned long>(n), base); }
    size_t print(double n, int digits = 2) { return printFloat(n, static_cast<uint8_t>(digits)); }
    size_t print(const Printable& x) { return x.printTo(*this); }

    size_t println() { return print("\r\n"); }
    size_t println(const String& s);
    size_t println(const char c[]) { return print(c) + println(); }
    size_t println(char c) { return print(c) + println(); }
    size_t println(unsigned char n, int base = DEC) { return print(n, base) + println(); }
    size_t println(int n, int base = DEC) { return print(n, base) + println(); }
    size_t println(unsigned int n, int base = DEC) { return print(n, base) + println(); }
    size_t println(long n, int base = DEC) { return print(n, base) + println(); }
    size_t println(unsigned long n, int base = DEC) { return print(n, base) + println(); }
    size_t println(long long n, int base = DEC) { return print(n, base) + println(); }
    size_t println(unsigned long long n, int base = DEC) { return print(n, base) + println(); }
    size_t println(double n, int digits = 2) { return print(n, digits) + println(); }
    size_t println(const Printable& x) { return print(x) + println(); }

    virtual void flush() {}

private:
    // Как Print::printNumber(): цифры старше 9 — заглавными.
    size_t printNumber(unsigned long n, uint8_t base)
    {
        char buf[8 * sizeof(unsigned long) + 1];
        char* str = &buf[sizeof(buf) - 1];
        *str = '\0';
        if (base < 2) base = 10;
        do
        {
            const unsigned long m = n;
            n /= base;
            const char c = static_cast<char>(m - base * n);
            *--str = c < 10 ? c + '0' : c + 'A' - 10;
        } while (n);
        return write(str);
    }

    // Как Print::printFloat().
    size_t printFloat(double number, uint8_t digits)
    {
        if (std::isnan(number)) return print("nan");
        if (std::isinf(number)) return print("inf");
        if (number > 4294967040.0 || number < -4294967040.0) return print("ovf");

        size_t n = 0;
        if (number < 0.0)
        {
            n += print('-');
            number = -number;
        }

        double rounding = 0.5;
        for (uint8_t i = 0; i < digits; ++i) rounding /= 10.0;
        number += rounding;

        const unsigned long intPart = static_cast<unsigned long>(number);
        double remainder = number - static_cast<double>(intPart);
        n += print(intPart);
        if (digits > 0) n += print(".");

        while (digits-- > 0)
        {
            remainder *= 10.0;
            const int toPrint = static_cast<int>(remainder);
            n += print(toPrint);
            remainder -= toPrint;
        }
        return n;
    }
};
