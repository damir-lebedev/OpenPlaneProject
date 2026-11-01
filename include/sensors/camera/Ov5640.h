#pragma once
#include <stddef.h>
#include <stdint.h>

// ============================================================
// Камера OV5640 (5 Мп, модули с FPC-24 под разъём DCMI платы DevEBox
// H743): настройка по SCCB — это I2C с 16-битными адресами регистров,
// адрес 0x3C (7 бит). Кадры идут по параллельной шине DCMI
// (hal/stm32/Stm32Camera.h); здесь — только регистры.
//
// Режим — аппаратный JPEG: камера сама сжимает кадр (обычно 6–25 КБ на
// 640×480), МК лишь складывает байты в память — так кадр помещается в
// чёрный ящик и уходит в приложение по USB за доли секунды.
//
// Порядок: probe() — ID чипа (0x300A/0x300B = 0x5640); begin() —
// программный сброс и таблица настроек: такты (PLL от XCLK 24 МГц),
// окно сенсора с биннингом 2×2 (1280×960 → масштаб ISP до нужного
// размера), JPEG режим 3 (VSYNC — кадр, HREF — «данные идут»), качество
// (0x4407), автоэкспозиция, баланс белого, гамма, цвет. Значения — из
// документации OmniVision (OV5640 datasheet, раздел настроек JPEG).
//
// Без Arduino — тестируется на ПК (test_camera) с фейком SCCB.
// ============================================================

// Регистры камеры: запись и чтение одного байта по 16-битному адресу.
class ISccb
{
public:
    virtual ~ISccb() = default;
    virtual bool write(uint16_t reg, uint8_t value) = 0;
    virtual bool read(uint16_t reg, uint8_t& value) = 0;
    virtual void delayMs(uint32_t ms) = 0;
};

struct CameraSettings
{
    uint16_t width = 640;    // кратно 8; не больше 1280×960 (окно с биннингом)
    uint16_t height = 480;
    uint8_t quality = 12;    // 0x4407: масштаб квантования JPEG, 2 — лучше и больше, 40 — хуже и меньше
    bool flip = false;       // перевернуть (модуль стоит вверх ногами)
    bool mirror = false;
};

class Ov5640
{
public:
    static constexpr uint8_t ADDRESS = 0x3C;
    static constexpr uint16_t CHIP_ID = 0x5640;

    struct Reg
    {
        uint16_t reg;
        uint8_t value;
    };
    static constexpr uint16_t DELAY = 0xFFFF;   // в таблице: пауза value мс

    // ID чипа; 0 — не ответил по SCCB.
    static uint16_t probe(ISccb& bus)
    {
        uint8_t hi = 0, lo = 0;
        if (!bus.read(0x300A, hi) || !bus.read(0x300B, lo)) return 0;
        return static_cast<uint16_t>(hi << 8 | lo);
    }

    // Сброс и настройка JPEG. false — не ответил или не та камера; что
    // случилось — error().
    bool begin(ISccb& bus, const CameraSettings& s)
    {
        failedReg = 0;
        chip = probe(bus);
        if (chip == 0)
        {
            err = "нет ответа по SCCB (0x3C)";
            return false;
        }
        if (chip != CHIP_ID)
        {
            err = "это не OV5640";
            return false;
        }
        size_t n = 0;
        const Reg* table = jpegTable(n);
        if (!writeTable(bus, table, n)) return false;
        if (!setFrame(bus, s)) return false;
        err = nullptr;
        return true;
    }

    // Размер, качество и ориентация — на ходу (камера уже настроена).
    bool setFrame(ISccb& bus, const CameraSettings& s)
    {
        const uint16_t w = clamp(s.width, 64, 1280) & ~7u;
        const uint16_t h = clamp(s.height, 48, 960) & ~7u;
        // 0x3820 бит 1–2 — переворот (с биннингом бит 0 оставить), 0x3821
        // бит 1–2 — зеркало, бит 5 — JPEG, бит 0 — биннинг по горизонтали
        const uint8_t r3820 = static_cast<uint8_t>(0x41 | (s.flip ? 0x06 : 0x00));
        const uint8_t r3821 = static_cast<uint8_t>(0x21 | (s.mirror ? 0x00 : 0x06));
        const Reg regs[] = {
            { 0x3808, static_cast<uint8_t>(w >> 8) }, { 0x3809, static_cast<uint8_t>(w & 0xFF) },
            { 0x380A, static_cast<uint8_t>(h >> 8) }, { 0x380B, static_cast<uint8_t>(h & 0xFF) },
            { 0x3820, r3820 }, { 0x3821, r3821 },
            { 0x4407, clampQuality(s.quality) },
        };
        if (!writeTable(bus, regs, sizeof(regs) / sizeof(regs[0]))) return false;
        width = w;
        height = h;
        return true;
    }

    uint16_t chipId() const { return chip; }
    uint16_t frameWidth() const { return width; }
    uint16_t frameHeight() const { return height; }
    const char* error() const { return err; }
    uint16_t failedRegister() const { return failedReg; }

    // Таблица настройки JPEG (без размера кадра — его ставит setFrame).
    static const Reg* jpegTable(size_t& count)
    {
        static const Reg table[] = {
            // сброс; такты от XCLK, PLL: 24 МГц / 3 × 70 / 4 (sysclk) — ~15 кадров/с VGA
            { 0x3103, 0x11 }, { 0x3008, 0x82 }, { DELAY, 10 }, { 0x3008, 0x42 }, { 0x3103, 0x03 },
            { 0x3017, 0xFF }, { 0x3018, 0xFF },   // выходы D[9:0], VSYNC, HREF, PCLK
            { 0x3034, 0x18 }, { 0x3035, 0x41 }, { 0x3036, 0x46 }, { 0x3037, 0x13 }, { 0x3108, 0x01 },
            // аналоговая часть (значения OmniVision)
            { 0x3630, 0x36 }, { 0x3631, 0x0E }, { 0x3632, 0xE2 }, { 0x3633, 0x12 }, { 0x3621, 0xE0 },
            { 0x3704, 0xA0 }, { 0x3703, 0x5A }, { 0x3715, 0x78 }, { 0x3717, 0x01 }, { 0x370B, 0x60 },
            { 0x3705, 0x1A }, { 0x3905, 0x02 }, { 0x3906, 0x10 }, { 0x3901, 0x0A }, { 0x3731, 0x12 },
            { 0x3600, 0x08 }, { 0x3601, 0x33 }, { 0x302D, 0x60 }, { 0x3620, 0x52 }, { 0x371B, 0x20 },
            { 0x471C, 0x50 }, { 0x3A13, 0x43 }, { 0x3A18, 0x00 }, { 0x3A19, 0xF8 }, { 0x3635, 0x13 },
            { 0x3636, 0x03 }, { 0x3634, 0x40 }, { 0x3622, 0x01 },
            // 50/60 Гц
            { 0x3C01, 0x34 }, { 0x3C04, 0x28 }, { 0x3C05, 0x98 }, { 0x3C06, 0x00 }, { 0x3C07, 0x08 },
            { 0x3C08, 0x00 }, { 0x3C09, 0x1C }, { 0x3C0A, 0x9C }, { 0x3C0B, 0x40 },
            // окно сенсора 2624×1952 с биннингом 2×2 → 1280×960 на входе ISP
            { 0x3814, 0x31 }, { 0x3815, 0x31 },
            { 0x3800, 0x00 }, { 0x3801, 0x00 }, { 0x3802, 0x00 }, { 0x3803, 0x04 },
            { 0x3804, 0x0A }, { 0x3805, 0x3F }, { 0x3806, 0x07 }, { 0x3807, 0x9B },
            { 0x380C, 0x07 }, { 0x380D, 0x68 }, { 0x380E, 0x03 }, { 0x380F, 0xD8 },   // HTS 1896, VTS 984
            { 0x3810, 0x00 }, { 0x3811, 0x10 }, { 0x3812, 0x00 }, { 0x3813, 0x06 },
            { 0x3618, 0x00 }, { 0x3612, 0x29 }, { 0x3708, 0x64 }, { 0x3709, 0x52 }, { 0x370C, 0x03 },
            // автоэкспозиция
            { 0x3A02, 0x03 }, { 0x3A03, 0xD8 }, { 0x3A08, 0x01 }, { 0x3A09, 0x27 }, { 0x3A0A, 0x00 },
            { 0x3A0B, 0xF6 }, { 0x3A0E, 0x03 }, { 0x3A0D, 0x04 }, { 0x3A14, 0x03 }, { 0x3A15, 0xD8 },
            { 0x3A0F, 0x30 }, { 0x3A10, 0x28 }, { 0x3A1B, 0x30 }, { 0x3A1E, 0x26 }, { 0x3A11, 0x60 },
            { 0x3A1F, 0x14 }, { 0x3503, 0x00 },
            { 0x4001, 0x02 }, { 0x4004, 0x02 },
            // блоки: всё включено, JPEG — с тактами
            { 0x3000, 0x00 }, { 0x3002, 0x00 }, { 0x3004, 0xFF }, { 0x3006, 0xFF },
            { 0x300E, 0x58 }, { 0x302E, 0x00 },
            // формат: YUV422 → JPEG, режим 3 (VSYNC — кадр, HREF — данные)
            { 0x4300, 0x30 }, { 0x501F, 0x00 }, { 0x4713, 0x03 }, { 0x4407, 0x0C }, { 0x440E, 0x00 },
            { 0x460B, 0x35 }, { 0x460C, 0x22 }, { 0x4837, 0x22 }, { 0x3824, 0x04 },
            { 0x4740, 0x21 },   // полярности: PCLK — по фронту, VSYNC — активна высоким
            // ISP: коррекция линзы, гамма, пиксели, цвет, масштаб, баланс белого
            { 0x5000, 0xA7 }, { 0x5001, 0xA3 },
            { 0x5180, 0xFF }, { 0x5181, 0xF2 }, { 0x5182, 0x00 }, { 0x5183, 0x14 }, { 0x5184, 0x25 },
            { 0x5185, 0x24 }, { 0x5186, 0x09 }, { 0x5187, 0x09 }, { 0x5188, 0x09 }, { 0x5189, 0x75 },
            { 0x518A, 0x54 }, { 0x518B, 0xE0 }, { 0x518C, 0xB2 }, { 0x518D, 0x42 }, { 0x518E, 0x3D },
            { 0x518F, 0x56 }, { 0x5190, 0x46 }, { 0x5191, 0xF8 }, { 0x5192, 0x04 }, { 0x5193, 0x70 },
            { 0x5194, 0xF0 }, { 0x5195, 0xF0 }, { 0x5196, 0x03 }, { 0x5197, 0x01 }, { 0x5198, 0x04 },
            { 0x5199, 0x12 }, { 0x519A, 0x04 }, { 0x519B, 0x00 }, { 0x519C, 0x06 }, { 0x519D, 0x82 },
            { 0x519E, 0x38 },
            { 0x5381, 0x1E }, { 0x5382, 0x5B }, { 0x5383, 0x08 }, { 0x5384, 0x0A }, { 0x5385, 0x7E },
            { 0x5386, 0x88 }, { 0x5387, 0x7C }, { 0x5388, 0x6C }, { 0x5389, 0x10 }, { 0x538A, 0x01 },
            { 0x538B, 0x98 },
            { 0x5300, 0x08 }, { 0x5301, 0x30 }, { 0x5302, 0x10 }, { 0x5303, 0x00 }, { 0x5304, 0x08 },
            { 0x5305, 0x30 }, { 0x5306, 0x08 }, { 0x5307, 0x16 }, { 0x5309, 0x08 }, { 0x530A, 0x30 },
            { 0x530B, 0x04 }, { 0x530C, 0x06 },
            { 0x5480, 0x01 }, { 0x5481, 0x08 }, { 0x5482, 0x14 }, { 0x5483, 0x28 }, { 0x5484, 0x51 },
            { 0x5485, 0x65 }, { 0x5486, 0x71 }, { 0x5487, 0x7D }, { 0x5488, 0x87 }, { 0x5489, 0x91 },
            { 0x548A, 0x9A }, { 0x548B, 0xAA }, { 0x548C, 0xB8 }, { 0x548D, 0xCD }, { 0x548E, 0xDD },
            { 0x548F, 0xEA }, { 0x5490, 0x1D },
            { 0x5580, 0x06 }, { 0x5583, 0x40 }, { 0x5584, 0x10 }, { 0x5589, 0x10 }, { 0x558A, 0x00 },
            { 0x558B, 0xF8 }, { 0x501D, 0x40 },
            // проснуться
            { 0x3008, 0x02 }, { DELAY, 50 },
        };
        count = sizeof(table) / sizeof(table[0]);
        return table;
    }

private:
    static uint16_t clamp(uint16_t v, uint16_t lo, uint16_t hi) { return v < lo ? lo : v > hi ? hi : v; }
    static uint8_t clampQuality(uint8_t q) { return q < 2 ? 2 : q > 63 ? 63 : q; }

    bool writeTable(ISccb& bus, const Reg* regs, size_t n)
    {
        for (size_t i = 0; i < n; ++i)
        {
            if (regs[i].reg == DELAY)
            {
                bus.delayMs(regs[i].value);
                continue;
            }
            if (!bus.write(regs[i].reg, regs[i].value))
            {
                failedReg = regs[i].reg;
                err = "регистр не записался";
                return false;
            }
        }
        return true;
    }

    uint16_t chip = 0;
    uint16_t width = 0, height = 0;
    uint16_t failedReg = 0;
    const char* err = "не настроена";
};
