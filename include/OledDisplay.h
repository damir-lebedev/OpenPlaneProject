#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#include "Config.h"
#include "LoopStats.h"

// ============================================================
// OLED DISPLAY (SSD1306 128x64, I2C 0x3C)
//
// Отладочный экран состояния на отдельной шине Wire1 (Config::
// PIN_I2C2_SDA/SCL), чтобы отрисовка кадра (~25 мс на 400 кГц) не
// задерживала опрос датчиков на основной шине. Рисуется в своей
// FreeRTOS-задаче на ядре 0 раз в REFRESH_MS; данные только читает
// (FlightController/Autopilot/LoopStats), ничего в них не меняет.
//
// Контроллер проверен на стенде: SSD1306 (картинка встала ровно,
// без сдвига на 2 столбца, как было бы у SH1106).
//
// На плате без второй I2C-шины (PIN_I2C2_SDA < 0) begin() ничего не
// делает — экран просто отсутствует.
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

    bool begin()
    {
        if (Config::PIN_I2C2_SDA < 0 || Config::PIN_I2C2_SCL < 0)
        {
            return false;
        }

        Wire1.begin(Config::PIN_I2C2_SDA, Config::PIN_I2C2_SCL, 400000);

        Wire1.beginTransmission(I2C_ADDRESS);
        if (Wire1.endTransmission() != 0)
        {
            Serial.println("OLED: не отвечает на Wire1, экран отключён");
            return false;
        }

        // Своя функция передачи байтов поверх Wire1: штатные
        // конструкторы U8g2 *_HW_I2C работают только с Wire.
        u8g2_Setup_ssd1306_i2c_128x64_noname_f(display.getU8g2(), U8G2_R0,
                                               byteCallback, u8x8_gpio_and_delay_arduino);
        display.setI2CAddress(I2C_ADDRESS << 1);
        display.begin();
        display.setFont(u8g2_font_6x10_tr);

        xTaskCreatePinnedToCore(displayTask, "oled", 4096, this, 1, nullptr, 0);

        Serial.println("OLED: подключён (SSD1306, Wire1)");
        return true;
    }


private:

    static constexpr uint8_t I2C_ADDRESS = 0x3C;
    static constexpr uint32_t REFRESH_MS = 200;

    FlightController& controller;
    Autopilot* autopilot;
    const LoopStats& loopStats;

    U8G2 display;

    static uint8_t byteCallback(u8x8_t* u8x8, uint8_t msg, uint8_t argInt, void* argPtr)
    {
        switch (msg)
        {
            case U8X8_MSG_BYTE_SEND:
                Wire1.write(static_cast<const uint8_t*>(argPtr), argInt);
                break;
            case U8X8_MSG_BYTE_START_TRANSFER:
                Wire1.beginTransmission(u8x8_GetI2CAddress(u8x8) >> 1);
                break;
            case U8X8_MSG_BYTE_END_TRANSFER:
                Wire1.endTransmission();
                break;
            case U8X8_MSG_BYTE_INIT:      // Wire1.begin() уже вызван в begin()
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
        char line[32];

        display.clearBuffer();

        // Строка 1: связь / ARM / режим. Потеря связи — инверсией.
        const bool rxLost = controller.isReceiverFailsafe();
        const bool armed = controller.isArmed();

        if (rxLost)
        {
            display.drawBox(0, 0, 128, 11);
            display.setDrawColor(0);
        }
        snprintf(line, sizeof(line), "%s %s %s",
                 rxLost ? "RX LOST" : "RX ok",
                 armed ? "ARMED" : "disarm",
                 autopilot ? shortMode(autopilot->getMode()) : "MAN");
        display.drawStr(1, 9, line);
        display.setDrawColor(1);

        ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
        if (imu && imu->isAvailable())
        {
            const ImuData& d = imu->getImuData();
            snprintf(line, sizeof(line), "R%+6.1f P%+6.1f", d.roll, d.pitch);
        }
        else
        {
            snprintf(line, sizeof(line), "IMU --");
        }
        display.drawStr(0, 21, line);

        BarometerSensor* baro = autopilot ? autopilot->getBarometerSensor() : nullptr;
        if (baro && baro->isAvailable())
        {
            const BarometerData& d = baro->getBarometerData();
            snprintf(line, sizeof(line), "Alt%+6.1f Vz%+5.1f", d.altitude, d.verticalSpeed);
        }
        else
        {
            snprintf(line, sizeof(line), "BARO --");
        }
        display.drawStr(0, 32, line);

        MagnetometerSensor* mag = autopilot ? autopilot->getMagnetometerSensor() : nullptr;
        const FlightOutputState& out = controller.getOutputState();
        if (mag && mag->isAvailable())
        {
            snprintf(line, sizeof(line), "Hdg %3.0f  Thr %4u", mag->getMagData().headingDegrees, out.throttle);
        }
        else
        {
            snprintf(line, sizeof(line), "Hdg --   Thr %4u", out.throttle);
        }
        display.drawStr(0, 43, line);

        snprintf(line, sizeof(line), "L%4u R%4u E%4u", out.aileronLeft, out.aileronRight, out.elevator);
        display.drawStr(0, 54, line);

        snprintf(line, sizeof(line), "Loop %3luHz max%4luus",
                 (unsigned long)loopStats.hz, (unsigned long)loopStats.maxUs);
        display.drawStr(0, 64, line);

        display.sendBuffer();
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
