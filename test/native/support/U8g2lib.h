#pragma once

// ============================================================
// Нативная замена U8g2 (olikraus/U8g2) — только то, что использует
// OledDisplay: настройка SSD1306 с пользовательской функцией
// передачи байтов, шрифт, строки, прямоугольник, цвет, буфер кадра.
//
// Вместо пикселей фейк запоминает, что было нарисовано (drawStr,
// drawBox) в текущем и последнем отправленном кадре. begin() и
// sendBuffer() гоняют байты через пользовательский byte-callback
// так же, как настоящая библиотека: INIT, START_TRANSFER, SEND,
// SET_DC, END_TRANSFER.
// ============================================================

#include <cstdint>
#include <string>
#include <vector>

#include "Arduino.h"

#define U8X8_MSG_CAD_INIT 20
#define U8X8_MSG_CAD_SEND_DATA 23
#define U8X8_MSG_CAD_START_TRANSFER 24
#define U8X8_MSG_CAD_END_TRANSFER 25
#define U8X8_MSG_BYTE_INIT U8X8_MSG_CAD_INIT
#define U8X8_MSG_BYTE_SET_DC 32
#define U8X8_MSG_BYTE_SEND U8X8_MSG_CAD_SEND_DATA
#define U8X8_MSG_BYTE_START_TRANSFER U8X8_MSG_CAD_START_TRANSFER
#define U8X8_MSG_BYTE_END_TRANSFER U8X8_MSG_CAD_END_TRANSFER

typedef struct u8x8_struct u8x8_t;
typedef uint8_t (*u8x8_msg_cb)(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr);
typedef uint16_t u8g2_uint_t;

struct u8x8_struct
{
    u8x8_msg_cb byte_cb;
    u8x8_msg_cb gpio_and_delay_cb;
    uint8_t i2c_address;
};

struct u8g2_cb_struct
{
    int rotation;
};
typedef struct u8g2_cb_struct u8g2_cb_t;

struct u8g2_struct
{
    u8x8_t u8x8;
    const u8g2_cb_t* cb;
};
typedef struct u8g2_struct u8g2_t;

#define u8x8_GetI2CAddress(u8x8) ((u8x8)->i2c_address)

inline const u8g2_cb_t u8g2_cb_r0 = { 0 };
#define U8G2_R0 (&u8g2_cb_r0)

inline const uint8_t u8g2_font_6x10_tr[] = { 0 };

inline uint8_t u8x8_gpio_and_delay_arduino(u8x8_t*, uint8_t, uint8_t, void*) { return 1; }

inline void u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2_t* u8g2, const u8g2_cb_t* rotation, u8x8_msg_cb byte_cb,
                                                   u8x8_msg_cb gpio_and_delay_cb)
{
    u8g2->cb = rotation;
    u8g2->u8x8.byte_cb = byte_cb;
    u8g2->u8x8.gpio_and_delay_cb = gpio_and_delay_cb;
}

class U8G2 : public Print
{
public:
    struct DrawnText
    {
        int x;
        int y;
        std::string text;
        uint8_t color;
    };

    struct DrawnBox
    {
        int x, y, w, h;
    };

    U8G2() { u8g2 = u8g2_t{ { nullptr, nullptr, 0 }, nullptr }; }

    u8g2_t* getU8g2() { return &u8g2; }
    void setI2CAddress(uint8_t address) { u8g2.u8x8.i2c_address = address; }

    bool begin()
    {
        const uint8_t init[] = { 0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xAF };   // фрагмент инициализации SSD1306
        call(U8X8_MSG_BYTE_INIT, 0, nullptr);
        call(U8X8_MSG_BYTE_START_TRANSFER, 0, nullptr);
        call(U8X8_MSG_BYTE_SET_DC, 0, nullptr);
        call(U8X8_MSG_BYTE_SEND, sizeof(init), const_cast<uint8_t*>(init));
        call(U8X8_MSG_BYTE_END_TRANSFER, 0, nullptr);
        // Сообщение, которого передача байтов не знает, — должна вернуть 0.
        unknownMessageResult = call(0xFF, 0, nullptr);
        begun = true;
        return true;
    }

    void setFont(const uint8_t* f) { font = f; }
    void setDrawColor(uint8_t color) { drawColor = color; }

    void clearBuffer()
    {
        frame.clear();
        boxes.clear();
    }

    u8g2_uint_t drawStr(u8g2_uint_t x, u8g2_uint_t y, const char* s)
    {
        frame.push_back(DrawnText{ x, y, s ? s : "", drawColor });
        return static_cast<u8g2_uint_t>(s ? strlen(s) * 6 : 0);
    }

    void drawBox(u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h)
    {
        boxes.push_back(DrawnBox{ x, y, w, h });
    }

    void sendBuffer()
    {
        uint8_t page[16] = {};
        call(U8X8_MSG_BYTE_START_TRANSFER, 0, nullptr);
        call(U8X8_MSG_BYTE_SEND, sizeof(page), page);
        call(U8X8_MSG_BYTE_END_TRANSFER, 0, nullptr);
        sentFrame = frame;
        sentBoxes = boxes;
        framesSent++;
    }

    size_t write(uint8_t) override { return 1; }

    // --- для тестов ---
    const std::vector<DrawnText>& lastFrame() const { return sentFrame; }
    const std::vector<DrawnBox>& lastBoxes() const { return sentBoxes; }
    unsigned sentFrames() const { return framesSent; }
    bool isBegun() const { return begun; }
    const uint8_t* currentFont() const { return font; }
    uint8_t lastUnknownMessageResult() const { return unknownMessageResult; }

private:
    u8g2_t u8g2;
    const uint8_t* font = nullptr;
    uint8_t drawColor = 1;
    std::vector<DrawnText> frame;
    std::vector<DrawnBox> boxes;
    std::vector<DrawnText> sentFrame;
    std::vector<DrawnBox> sentBoxes;
    unsigned framesSent = 0;
    bool begun = false;
    uint8_t unknownMessageResult = 0xAA;

    uint8_t call(uint8_t msg, uint8_t argInt, void* argPtr)
    {
        return u8g2.u8x8.byte_cb ? u8g2.u8x8.byte_cb(&u8g2.u8x8, msg, argInt, argPtr) : 0;
    }
};
