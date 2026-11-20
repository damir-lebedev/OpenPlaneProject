#pragma once

// ============================================================
// Нативная замена Update из Arduino core ESP32 (прошивка по WiFi,
// telemetry/WifiOta.h). Пишет «во вторую область» — в память; образ
// без магического байта 0xE9 отвергается при первой записи, как у
// настоящего Update. Тест смотрит fake::update().
// ============================================================

#include <cstddef>
#include <cstdint>
#include <vector>

#define UPDATE_SIZE_UNKNOWN 0xFFFFFFFF
#define U_FLASH 0

namespace fake
{
    struct UpdateState
    {
        std::vector<uint8_t> image;   // принятое
        bool begun = false;
        bool finished = false;        // end(true) прошёл — загрузочной станет новая
        unsigned aborts = 0;
        const char* error = "No Error";
    };

    inline UpdateState& update()
    {
        static UpdateState state;
        return state;
    }
}

class UpdateClass
{
public:
    bool begin(size_t /*size*/ = UPDATE_SIZE_UNKNOWN, int /*command*/ = U_FLASH)
    {
        fake::UpdateState& s = fake::update();
        s.image.clear();
        s.begun = true;
        s.finished = false;
        s.error = "No Error";
        return true;
    }

    size_t write(const uint8_t* data, size_t len)
    {
        fake::UpdateState& s = fake::update();
        if (!s.begun) return 0;
        if (s.image.empty() && len > 0 && data[0] != 0xE9)
        {
            s.error = "Magic byte is wrong, not 0xE9";
            return 0;
        }
        s.image.insert(s.image.end(), data, data + len);
        return len;
    }

    bool end(bool /*evenIfRemaining*/ = false)
    {
        fake::UpdateState& s = fake::update();
        if (!s.begun || s.image.empty()) return false;
        s.begun = false;
        s.finished = true;
        return true;
    }

    void abort()
    {
        fake::UpdateState& s = fake::update();
        s.begun = false;
        s.aborts++;
    }

    bool isFinished() const { return fake::update().finished; }
    const char* errorString() const { return fake::update().error; }
};

inline UpdateClass Update;
