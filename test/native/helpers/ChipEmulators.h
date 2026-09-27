#pragma once

// ============================================================
// Эмуляторы чипов для тестов "прошивка целиком".
//
// Каждый чип — регистровая карта (fake::RegisterMapDevice на I2C или
// fake::SpiRegisterMapDevice на SPI), которую драйвер видит так же,
// как настоящий чип: ID, флаги готовности, сброс, данные в своём
// формате (порядок байт, масштаб — по датащитам, как в
// test_sensors_new). Данные берутся из "мира" (World): углы и
// угловые скорости самолёта, высота, воздушная скорость, курс,
// координаты. update(world) переписывает регистры данных.
//
// Оси. Мир — в осях самолёта FLU (X вперёд, Y влево, Z вверх), углы и
// скорости — авиационные знаки (+крен — правое крыло вниз, +тангаж —
// нос вверх, +рысканье — нос вправо). В оси чипа — поворотом вокруг
// вертикали на Config::IMU_ROTATION_CW_DEG / MAG_ROTATION_CW_DEG (куда
// смотрит ось X чипа, по часовой от носа), как в SensorMounting.
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "config/Config.h"

struct World
{
    // Ориентация, °, и угловые скорости, °/с (авиационные знаки).
    float rollDeg = 0, pitchDeg = 0, headingDeg = 0;
    float rollRateDps = 0, pitchRateDps = 0, yawRateDps = 0;

    float altitudeM = 0;        // над точкой включения
    float airspeedMs = 0;       // воздушная, для трубки Пито
    float temperatureC = 15;
    float groundPressurePa = 101325;
    float tubeOffsetPa = 150;   // барометр в трубке "врёт" на столько относительно статики

    double lat = 55.75, lon = 37.61;
    float groundSpeedMs = 0, courseDeg = 0;
    uint8_t satellites = 12;
    uint8_t fixType = 3;

    double staticPa() const
    {
        return groundPressurePa * pow(1.0 - altitudeM / 44330.0, 5.255);
    }

    double density() const { return staticPa() / (287.05 * (temperatureC + 273.15)); }

    double tubePa() const
    {
        return staticPa() + 0.5 * density() * airspeedMs * airspeedMs + tubeOffsetPa;
    }

    // Вектор из осей самолёта (FLU) в оси чипа, повёрнутого на rotationCwDeg.
    static void toChip(uint16_t rotationCwDeg, float x, float y, float z, float out[3])
    {
        const float r = rotationCwDeg * 0.01745329252f;
        out[0] = x * cosf(r) - y * sinf(r);
        out[1] = x * sinf(r) + y * cosf(r);
        out[2] = z;
    }

    // Показания акселерометра в покое (g) — вектор "вверх" в осях самолёта.
    void accelBody(float& x, float& y, float& z) const
    {
        const float roll = rollDeg * 0.01745329252f, pitch = pitchDeg * 0.01745329252f;
        x = sinf(pitch);
        y = cosf(pitch) * sinf(roll);
        z = cosf(pitch) * cosf(roll);
    }

    // Угловые скорости в правой системе FLU, °/с.
    void gyroBody(float& x, float& y, float& z) const
    {
        x = rollRateDps;
        y = -pitchRateDps;
        z = -yawRateDps;
    }
};

namespace chips
{
    inline void putLe16(uint8_t* regs, uint8_t reg, float value)
    {
        const long v = lroundf(value);
        const int16_t s = static_cast<int16_t>(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
        regs[reg] = static_cast<uint8_t>(static_cast<uint16_t>(s) & 0xFF);
        regs[reg + 1] = static_cast<uint8_t>(static_cast<uint16_t>(s) >> 8);
    }

    inline void putBe24(uint8_t* regs, uint8_t reg, int32_t value)
    {
        const uint32_t v = static_cast<uint32_t>(value) & 0xFFFFFFu;
        regs[reg] = static_cast<uint8_t>(v >> 16);
        regs[reg + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
        regs[reg + 2] = static_cast<uint8_t>(v & 0xFF);
    }

    // Общая часть: карта регистров на I2C или SPI.
    struct Chip
    {
        fake::RegisterMapDevice i2c;
        fake::SpiRegisterMapDevice spi;
        bool onSpi = false;

        uint8_t* regs() { return onSpi ? spi.regs : i2c.regs; }

        void attach(TwoWire& wire, uint8_t address) { onSpi = false; wire.attach(address, &i2c); }
        void attach(SPIClass& bus, uint8_t csPin, uint8_t dummyBytes = 0)
        {
            onSpi = true;
            spi.dummyBytes = dummyBytes;
            bus.attach(csPin, &spi);
        }

        void onWrite(std::function<void(uint8_t, uint8_t)> handler)
        {
            i2c.onRegisterWrite = handler;
            spi.onRegisterWrite = handler;
        }

        void beforeRead(std::function<void(uint8_t)> handler)
        {
            i2c.beforeRead = [handler](uint8_t reg, size_t) { handler(reg); };
            spi.beforeRead = handler;
        }
    };

    // LSM6DSV (16X): WHO_AM_I 0x70, самосброс SW_RESET, данные LE с 0x20.
    struct Lsm6dsv : Chip
    {
        void install()
        {
            regs()[0x0F] = 0x70;
            regs()[0x17] = 0x00;   // 16X
            onWrite([this](uint8_t reg, uint8_t value) {
                if (reg == 0x12 && (value & 0x01)) regs()[0x12] = value & ~0x01;
            });
        }

        void update(const World& w)
        {
            float ax, ay, az, gx, gy, gz, a[3], g[3];
            w.accelBody(ax, ay, az);
            w.gyroBody(gx, gy, gz);
            World::toChip(Config::IMU_ROTATION_CW_DEG, ax, ay, az, a);
            World::toChip(Config::IMU_ROTATION_CW_DEG, gx, gy, gz, g);
            uint8_t* r = regs();
            putLe16(r, 0x20, (w.temperatureC - 25.0f) * 256.0f);
            for (int i = 0; i < 3; ++i) putLe16(r, static_cast<uint8_t>(0x22 + 2 * i), g[i] * 1000.0f / 70.0f);
            for (int i = 0; i < 3; ++i) putLe16(r, static_cast<uint8_t>(0x28 + 2 * i), a[i] * 1000.0f / 0.488f);
        }
    };

    // ICM-45686: WHO_AM_I 0xE9 (0x72), косвенные регистры IPREG через 0x7C..0x7E.
    struct Icm45686 : Chip
    {
        std::map<uint16_t, uint8_t> indirect;
        uint16_t address = 0;

        void install()
        {
            regs()[0x72] = 0xE9;
            onWrite([this](uint8_t reg, uint8_t value) {
                if (reg == 0x7C) address = static_cast<uint16_t>((value << 8) | (address & 0xFF));
                else if (reg == 0x7D) address = static_cast<uint16_t>((address & 0xFF00) | value);
                else if (reg == 0x7E) indirect[address++] = value;
            });
            beforeRead([this](uint8_t reg) {
                if (reg == 0x7E) regs()[0x7E] = indirect[address];
            });
        }

        void update(const World& w)
        {
            float ax, ay, az, gx, gy, gz, a[3], g[3];
            w.accelBody(ax, ay, az);
            w.gyroBody(gx, gy, gz);
            World::toChip(Config::IMU_ROTATION_CW_DEG, ax, ay, az, a);
            World::toChip(Config::IMU_ROTATION_CW_DEG, gx, gy, gz, g);
            uint8_t* r = regs();
            for (int i = 0; i < 3; ++i) putLe16(r, static_cast<uint8_t>(0x00 + 2 * i), a[i] * 2048.0f);
            for (int i = 0; i < 3; ++i) putLe16(r, static_cast<uint8_t>(0x06 + 2 * i), g[i] * 16.4f);
            putLe16(r, 0x0C, (w.temperatureC - 25.0f) * 132.48f);
        }
    };

    // QMC6309 (I2C 0x7C): ID 0x90, NVM готова, данные LE с 0x01, 40.96 LSB/мкТл.
    struct Qmc6309 : Chip
    {
        float horizontalUt = 20.0f;   // горизонтальная составляющая поля Земли
        float downUt = 45.0f;         // вертикальная, вниз (средние широты)

        void install()
        {
            regs()[0x00] = 0x90;
            regs()[0x09] = 0x19;   // DRDY | NVM_RDY | NVM_LOAD_DONE
        }

        // Самолёт ровно, нос на курсе headingDeg: север — под углом −курс от носа.
        void update(const World& w)
        {
            const float h = w.headingDeg * 0.01745329252f;
            float m[3];
            World::toChip(Config::MAG_ROTATION_CW_DEG, horizontalUt * cosf(h), horizontalUt * sinf(h), -downUt, m);
            uint8_t* r = regs();
            for (int i = 0; i < 3; ++i) putLe16(r, static_cast<uint8_t>(0x01 + 2 * i), m[i] * 40.96f);
            regs()[0x09] = 0x19;
        }
    };

    // SPL06-001: ID 0x10, коэффициенты подобраны так, что давление
    // линейно по сырому отсчёту: P = c00 + c10·(raw / 253952), T = c0/2.
    struct Spl06 : Chip
    {
        static constexpr int32_t C0 = 30;        // 15 °C
        static constexpr int32_t C00 = 100000;   // Па
        static constexpr int32_t C10 = 20000;    // Па на единицу масштабированного отсчёта

        void install()
        {
            uint8_t* r = regs();
            r[0x0D] = 0x10;
            r[0x08] = 0xF0;   // COEF_RDY | SENSOR_RDY | TMP_RDY | PRS_RDY
            r[0x28] = 0x80;
            auto u = [](int32_t v, int bits) { return static_cast<uint32_t>(v) & ((1u << bits) - 1); };
            const uint32_t c0 = u(C0, 12), c1 = 0, c00 = u(C00, 20), c10 = u(C10, 20);
            uint8_t* k = r + 0x10;
            k[0] = static_cast<uint8_t>(c0 >> 4);
            k[1] = static_cast<uint8_t>(((c0 & 0x0F) << 4) | (c1 >> 8));
            k[2] = static_cast<uint8_t>(c1 & 0xFF);
            k[3] = static_cast<uint8_t>(c00 >> 12);
            k[4] = static_cast<uint8_t>((c00 >> 4) & 0xFF);
            k[5] = static_cast<uint8_t>(((c00 & 0x0F) << 4) | (c10 >> 16));
            k[6] = static_cast<uint8_t>((c10 >> 8) & 0xFF);
            k[7] = static_cast<uint8_t>(c10 & 0xFF);
            for (int i = 8; i < 18; ++i) k[i] = 0;   // c01, c11, c20, c21, c30 = 0
            onWrite([this](uint8_t reg, uint8_t value) {
                if (reg == 0x08) regs()[0x08] = static_cast<uint8_t>(0xF0 | (value & 0x07));
            });
        }

        void setPressure(double pa)
        {
            putBe24(regs(), 0x00, static_cast<int32_t>(lround((pa - C00) / C10 * 253952.0)));
            putBe24(regs(), 0x03, 0);
        }

        void update(const World& w) { setPressure(w.staticPa()); }
    };

    // BMP581: ID 0x50, данные LE: температура /65536, давление /64.
    struct Bmp581 : Chip
    {
        void install()
        {
            uint8_t* r = regs();
            r[0x01] = 0x50;
            r[0x27] = 0x11;   // POR (сброс завершён) | DRDY
            r[0x28] = 0x02;   // NVM готова
            r[0x30] = 0x03;
            r[0x38] = 0x80;
        }

        void setSample(double pa, float temperatureC)
        {
            uint8_t* r = regs();
            const uint32_t t = static_cast<uint32_t>(static_cast<int32_t>(lroundf(temperatureC * 65536.0f))) & 0xFFFFFFu;
            const uint32_t p = static_cast<uint32_t>(lround(pa * 64.0));
            r[0x1D] = static_cast<uint8_t>(t & 0xFF);
            r[0x1E] = static_cast<uint8_t>((t >> 8) & 0xFF);
            r[0x1F] = static_cast<uint8_t>(t >> 16);
            r[0x20] = static_cast<uint8_t>(p & 0xFF);
            r[0x21] = static_cast<uint8_t>((p >> 8) & 0xFF);
            r[0x22] = static_cast<uint8_t>(p >> 16);
            r[0x27] = 0x11;
        }
    };

    // Кадр UBX NAV-PVT из мира (u-blox M10).
    inline std::vector<uint8_t> navPvt(const World& w)
    {
        std::vector<uint8_t> p(92, 0);
        auto put32 = [&p](size_t at, int32_t v) {
            for (int i = 0; i < 4; ++i) p[at + i] = static_cast<uint8_t>(static_cast<uint32_t>(v) >> (8 * i));
        };
        p[20] = w.fixType;
        p[21] = 0x01;   // gnssFixOK
        p[23] = w.satellites;
        put32(24, static_cast<int32_t>(lround(w.lon * 1e7)));
        put32(28, static_cast<int32_t>(lround(w.lat * 1e7)));
        put32(36, static_cast<int32_t>(lroundf((150.0f + w.altitudeM) * 1000.0f)));
        put32(40, 1500);   // hAcc, мм
        put32(44, 2500);
        put32(60, static_cast<int32_t>(lroundf(w.groundSpeedMs * 1000.0f)));
        put32(64, static_cast<int32_t>(lroundf(w.courseDeg * 1e5f)));

        std::vector<uint8_t> frame = { 0xB5, 0x62, 0x01, 0x07, 92, 0 };
        frame.insert(frame.end(), p.begin(), p.end());
        uint8_t a = 0, b = 0;
        for (size_t i = 2; i < frame.size(); ++i)
        {
            a = static_cast<uint8_t>(a + frame[i]);
            b = static_cast<uint8_t>(b + a);
        }
        frame.push_back(a);
        frame.push_back(b);
        return frame;
    }
}
