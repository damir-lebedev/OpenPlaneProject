#pragma once
#include <Arduino.h>
#include <U8g2lib.h>

#include "autopilot/Autopilot.h"
#include "control/FlightController.h"
#include "hal/II2CBus.h"
#include "telemetry/LoopStats.h"

// ============================================================
// OLED DISPLAY (SSD1306 128x64, I2C 0x3C)
//
// Отладочный экран состояния на отдельной шине I2C
// (IBoard::displayI2c()), чтобы отрисовка кадра (~25 мс на 400 кГц)
// не задерживала опрос датчиков. Рисуется в своей FreeRTOS-задаче
// на ядре 0 раз в REFRESH_MS; данные только читает
// (FlightController/Autopilot/LoopStats), ничего в них не меняет.
//
// U8g2 передаёт байты через собственную функцию поверх II2CBus
// (byteCallback), поэтому экран не знает, что за шиной стоит Wire1
// ESP32. Callback у U8g2 — обычная C-функция без контекста, а
// user_ptr в библиотеке включается только глобальным флагом сборки,
// меняющим её структуры, — поэтому шина хранится в статическом поле
// (экран на борту один).
//
// Контроллер проверен на стенде: SSD1306 (картинка встала ровно,
// без сдвига на 2 столбца, как было бы у SH1106).
//
//   RX ok ARM STAB FL       связь / ARM / режим / закрылки
//   R  +1.2 P  -0.4         крен / тангаж, °
//   Alt +0.3 Vz +0.1        высота, м / вертикальная скорость, м/с
//   H123 T1000 Y1500        курс / газ / руль направления, мкс
//   L1500 R1500 E1500       элероны / руль высоты, мкс
//   Loop 500Hz max 1100us   частота и худший такт цикла
// ============================================================

class OledDisplay
{
public:

    OledDisplay(FlightController& controller, Autopilot* autopilot, const LoopStats& loopStats)
        : controller(controller),
          autopilot(autopilot),
          loopStats(loopStats)
    {
    }

    // bus == nullptr (на плате нет второй шины) — экрана просто нет.
    bool begin(II2CBus* displayBus)
    {
        if (!displayBus)
        {
            return false;
        }

        if (!displayBus->probe(I2C_ADDRESS))
        {
            Serial.println("OLED: не отвечает, экран отключён");
            return false;
        }

        busSlot() = displayBus;
        u8g2_Setup_ssd1306_i2c_128x64_noname_f(display.getU8g2(), U8G2_R0,
                                               byteCallback, u8x8_gpio_and_delay_arduino);
        display.setI2CAddress(I2C_ADDRESS << 1);
        display.begin();
        display.setFont(u8g2_font_6x10_tr);

        xTaskCreatePinnedToCore(displayTask, "oled", 4096, this, 1, nullptr, 0);

        Serial.println("OLED: подключён (SSD1306)");
        return true;
    }


private:

    static constexpr uint8_t I2C_ADDRESS = 0x3C;
    static constexpr uint32_t REFRESH_MS = 200;

    FlightController& controller;
    Autopilot* autopilot;
    const LoopStats& loopStats;

    U8G2 display;

    // Шина экрана для byteCallback (C-колбэк U8g2 не знает об объекте).
    // Статическая локальная переменная, а не static inline член: тот
    // требует C++17, а ядро Arduino собирается с gnu++11.
    static II2CBus*& busSlot()
    {
        static II2CBus* bus = nullptr;
        return bus;
    }

    // Передача байтов U8g2 поверх II2CBus.
    static uint8_t byteCallback(u8x8_t* u8x8, uint8_t msg, uint8_t argInt, void* argPtr)
    {
        II2CBus* bus = busSlot();
        switch (msg)
        {
            case U8X8_MSG_BYTE_SEND:
                bus->write(static_cast<const uint8_t*>(argPtr), argInt);
                break;
            case U8X8_MSG_BYTE_START_TRANSFER:
                bus->beginTransmission(u8x8_GetI2CAddress(u8x8) >> 1);
                break;
            case U8X8_MSG_BYTE_END_TRANSFER:
                bus->endTransmission();
                break;
            case U8X8_MSG_BYTE_INIT:      // шину уже подняла плата (IBoard::begin())
            case U8X8_MSG_BYTE_SET_DC:
                break;
            default:
                return 0;
        }
        return 1;
    }

    static void displayTask(void* arg)
    {
        OledDisplay* self = static_cast<OledDisplay*>(arg);
        TickType_t lastWake = xTaskGetTickCount();

        for (;;)
        {
            self->draw();
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(REFRESH_MS));
        }
    }

    void draw()
    {
        display.clearBuffer();

        drawStatusLine();
        drawLine(21, attitudeText());
        drawLine(32, baroText());

        const FlightOutputState& out = controller.getOutputState();
        char line[32];

        snprintf(line, sizeof(line), "%s T%4u Y%4u", headingText().c_str(), out.throttle, out.rudder);
        drawLine(43, line);

        snprintf(line, sizeof(line), "L%4u R%4u E%4u", out.aileronLeft, out.aileronRight, out.elevator);
        drawLine(54, line);

        snprintf(line, sizeof(line), "Loop %3luHz max%4luus",
                 (unsigned long)loopStats.hz, (unsigned long)loopStats.maxUs);
        drawLine(64, line);

        display.sendBuffer();
    }

    // Связь / ARM / режим / закрылки. Потеря связи — инверсией строки.
    void drawStatusLine()
    {
        const bool rxLost = controller.isReceiverFailsafe();
        char line[32];

        snprintf(line, sizeof(line), "%s %s %s%s",
                 rxLost ? "RX LOST" : "RX ok",
                 controller.isArmed() ? "ARM" : "safe",
                 !autopilot ? "MAN" : autopilot->isFailsafeGliding() ? "GLIDE" : shortMode(autopilot->getMode()),
                 controller.getFlapsUs() > 0 ? " FL" : "");

        if (rxLost)
        {
            display.drawBox(0, 0, 128, 11);
            display.setDrawColor(0);
        }
        display.drawStr(1, 9, line);
        display.setDrawColor(1);
    }

    void drawLine(uint8_t baselineY, const String& text)
    {
        display.drawStr(0, baselineY, text.c_str());
    }

    String attitudeText() const
    {
        const ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
        if (!imu || !imu->isAvailable()) return "IMU --";

        char line[32];
        const ImuData& d = imu->getImuData();
        snprintf(line, sizeof(line), "R%+6.1f P%+6.1f", d.roll, d.pitch);
        return line;
    }

    String baroText() const
    {
        const BarometerSensor* baro = autopilot ? autopilot->getBarometerSensor() : nullptr;
        if (!baro || !baro->isAvailable()) return "BARO --";

        char line[32];
        const BarometerData& d = baro->getBarometerData();
        snprintf(line, sizeof(line), "Alt%+6.1f Vz%+5.1f", d.altitude, d.verticalSpeed);
        return line;
    }

    String headingText() const
    {
        const MagnetometerSensor* mag = autopilot ? autopilot->getMagnetometerSensor() : nullptr;
        if (!mag || !mag->isAvailable()) return "H---";

        char text[8];
        snprintf(text, sizeof(text), "H%3.0f", mag->getMagData().headingDegrees);
        return text;
    }

    static const char* shortMode(AutopilotMode mode)
    {
        switch (mode)
        {
            case MODE_MANUAL:       return "MAN";
            case MODE_STABILIZE:    return "STAB";
            case MODE_AUTO_TAKEOFF: return "TKOFF";
            case MODE_ALT_HOLD:     return "ALT";
            default:                return "?";
        }
    }
};
