#pragma once

// ============================================================
// Нативная замена String (WString.h) из Arduino core ESP32 2.0.x.
//
// Хранит текст в std::string. Конструкторы из чисел — explicit, как
// в оригинале; числа с плавающей точкой форматируются как dtostrf
// ("%.*f"), целые в системе счисления base — строчными буквами, как
// itoa/ltoa.
// ============================================================

#include <cstdint>
#include <cstdlib>
#include <string>

#include "Print.h"

class String
{
public:
    String(const char* cstr = "") : text(cstr ? cstr : "") {}
    String(const char* cstr, unsigned int length) : text(cstr ? std::string(cstr, length) : std::string()) {}
    String(const String&) = default;
    String(String&&) = default;
    String& operator=(const String&) = default;
    String& operator=(String&&) = default;
    String& operator=(const char* cstr)
    {
        text = cstr ? cstr : "";
        return *this;
    }

    explicit String(char c) : text(1, c) {}
    explicit String(unsigned char value, unsigned char base = 10) : text(toBase(value, base)) {}
    explicit String(int value, unsigned char base = 10) : text(toBaseSigned(value, base)) {}
    explicit String(unsigned int value, unsigned char base = 10) : text(toBase(value, base)) {}
    explicit String(long value, unsigned char base = 10) : text(toBaseSigned(value, base)) {}
    explicit String(unsigned long value, unsigned char base = 10) : text(toBase(value, base)) {}
    explicit String(long long value, unsigned char base = 10) : text(toBaseSigned(value, base)) {}
    explicit String(unsigned long long value, unsigned char base = 10) : text(toBase(value, base)) {}
    explicit String(float value, unsigned int decimalPlaces = 2) : text(fixed(value, decimalPlaces)) {}
    explicit String(double value, unsigned int decimalPlaces = 2) : text(fixed(value, decimalPlaces)) {}

    unsigned int length() const { return static_cast<unsigned int>(text.size()); }
    bool isEmpty() const { return text.empty(); }
    const char* c_str() const { return text.c_str(); }
    bool reserve(unsigned int size)
    {
        text.reserve(size);
        return true;
    }

    String& operator+=(const String& rhs) { text += rhs.text; return *this; }
    String& operator+=(const char* cstr) { if (cstr) text += cstr; return *this; }
    String& operator+=(char c) { text += c; return *this; }
    String& operator+=(unsigned char n) { return *this += String(n); }
    String& operator+=(int n) { return *this += String(n); }
    String& operator+=(unsigned int n) { return *this += String(n); }
    String& operator+=(long n) { return *this += String(n); }
    String& operator+=(unsigned long n) { return *this += String(n); }
    String& operator+=(long long n) { return *this += String(n); }
    String& operator+=(unsigned long long n) { return *this += String(n); }
    String& operator+=(float n) { return *this += String(n); }
    String& operator+=(double n) { return *this += String(n); }

    char operator[](unsigned int index) const { return index < text.size() ? text[index] : '\0'; }
    char& operator[](unsigned int index) { return text[index]; }
    char charAt(unsigned int index) const { return (*this)[index]; }

    int indexOf(char ch, unsigned int fromIndex = 0) const { return find(text.find(ch, fromIndex)); }
    int indexOf(const String& str, unsigned int fromIndex = 0) const { return find(text.find(str.text, fromIndex)); }
    int indexOf(const char* str, unsigned int fromIndex = 0) const { return find(text.find(str, fromIndex)); }

    String substring(unsigned int beginIndex) const { return substring(beginIndex, length()); }
    String substring(unsigned int beginIndex, unsigned int endIndex) const
    {
        if (beginIndex > endIndex) std::swap(beginIndex, endIndex);
        if (beginIndex >= text.size()) return String();
        if (endIndex > text.size()) endIndex = length();
        return String(text.substr(beginIndex, endIndex - beginIndex).c_str());
    }

    long toInt() const { return atol(text.c_str()); }
    float toFloat() const { return static_cast<float>(atof(text.c_str())); }
    double toDouble() const { return atof(text.c_str()); }

    bool equals(const String& other) const { return text == other.text; }
    bool operator==(const String& other) const { return text == other.text; }
    bool operator==(const char* cstr) const { return text == (cstr ? cstr : ""); }
    bool operator!=(const String& other) const { return !(*this == other); }
    bool operator!=(const char* cstr) const { return !(*this == cstr); }
    bool startsWith(const String& prefix) const { return text.compare(0, prefix.text.size(), prefix.text) == 0; }

    const std::string& str() const { return text; }

private:
    std::string text;

    static int find(size_t position) { return position == std::string::npos ? -1 : static_cast<int>(position); }

    static std::string toBase(unsigned long long value, unsigned char base)
    {
        if (base < 2 || base > 36) base = 10;
        std::string digits;
        do
        {
            const unsigned d = static_cast<unsigned>(value % base);
            digits.insert(digits.begin(), static_cast<char>(d < 10 ? '0' + d : 'a' + d - 10));
            value /= base;
        } while (value);
        return digits;
    }

    static std::string toBaseSigned(long long value, unsigned char base)
    {
        if (base == 10 && value < 0) return "-" + toBase(static_cast<unsigned long long>(-value), 10);
        return toBase(static_cast<unsigned long long>(value), base);
    }

    static std::string fixed(double value, unsigned int decimals)
    {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%.*f", static_cast<int>(decimals), value);
        return buffer;
    }
};

inline String operator+(const String& lhs, const String& rhs)
{
    String result(lhs);
    result += rhs;
    return result;
}

inline String operator+(const String& lhs, const char* rhs)
{
    String result(lhs);
    result += rhs;
    return result;
}

inline String operator+(const char* lhs, const String& rhs)
{
    String result(lhs);
    result += rhs;
    return result;
}

inline String operator+(const String& lhs, char rhs)
{
    String result(lhs);
    result += rhs;
    return result;
}

inline size_t Print::print(const String& s)
{
    return write(reinterpret_cast<const uint8_t*>(s.c_str()), s.length());
}

inline size_t Print::println(const String& s)
{
    return print(s) + println();
}
