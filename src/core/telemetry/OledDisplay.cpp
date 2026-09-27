// Реализация telemetry/OledDisplay.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "telemetry/OledDisplay.h"


OledDisplay::OledDisplay(FlightController& flightController, Autopilot* ap, const LoopStats& stats)
: controller(flightController),
      autopilot(ap),
      loopStats(stats)
{
}

auto OledDisplay::begin(II2CBus* displayBus) -> bool
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

    Rtos::startTask(displayTask, "oled", 4096, this, Rtos::PRIORITY_BACKGROUND);

    Serial.println("OLED: подключён (SSD1306)");
    return true;
}

auto OledDisplay::busSlot() -> II2CBus*&
{
    static II2CBus* bus = nullptr;
    return bus;
}

auto OledDisplay::byteCallback(u8x8_t* u8x8, uint8_t msg, uint8_t argInt, void* argPtr) -> uint8_t
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

auto OledDisplay::displayTask(void* arg) -> void
{
    OledDisplay* self = static_cast<OledDisplay*>(arg);
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        self->draw();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(REFRESH_MS));
    }
}

auto OledDisplay::draw() -> void
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

    // uint32_t — до 10 цифр каждое: в худшем случае строка займёт
    // 34 байта с нулём, поэтому буфер на 40 (экран покажет первые 21).
    char stats[40];
    snprintf(stats, sizeof(stats), "Loop %3uHz max%4uus",
             static_cast<unsigned>(loopStats.hz), static_cast<unsigned>(loopStats.maxUs));
    drawLine(64, stats);

    display.sendBuffer();
}

auto OledDisplay::drawStatusLine() -> void
{
    const bool rxLost = controller.isReceiverFailsafe();
    char line[32];

    snprintf(line, sizeof(line), "%s %s %s%s",
             rxLost ? "RX LOST" : "RX ok",
             controller.isArmed() ? "ARM" : "safe",
             !autopilot                        ? "MAN"
             : autopilot->isFailsafeGliding()   ? "GLIDE"
             : autopilot->isFailsafeReturning() ? "FSRTH"
                                                : AutopilotNames::modeShort(autopilot->getMode()),
             controller.getFlapsUs() > 0 ? " FL" : "");

    if (rxLost)
    {
        display.drawBox(0, 0, 128, 11);
        display.setDrawColor(0);
    }
    display.drawStr(1, 9, line);
    display.setDrawColor(1);
}

auto OledDisplay::drawLine(uint8_t baselineY, const String& text) -> void
{
    display.drawStr(0, baselineY, text.c_str());
}

auto OledDisplay::attitudeText() const -> String
{
    const ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
    if (!imu || !imu->isAvailable()) return "IMU --";

    char line[32];
    const ImuData& d = imu->getImuData();
    snprintf(line, sizeof(line), "R%+6.1f P%+6.1f", d.roll, d.pitch);
    return line;
}

auto OledDisplay::baroText() const -> String
{
    const BarometerSensor* baro = autopilot ? autopilot->getBarometerSensor() : nullptr;
    if (!baro || !baro->isAvailable()) return "BARO --";

    char line[40];
    const BarometerData& d = baro->getBarometerData();
    const AirspeedSensor* airspeed = autopilot->getAirspeedSensor();
    if (airspeed && airspeed->isAvailable())
    {
        // С трубкой Пито — воздушная скорость в конце строки (21 символ экрана).
        snprintf(line, sizeof(line), "Alt%+6.1f Vz%+4.1f A%2.0f", d.altitude, d.verticalSpeed,
                 airspeed->getAirspeedData().indicatedMs);
        return line;
    }
    snprintf(line, sizeof(line), "Alt%+6.1f Vz%+5.1f", d.altitude, d.verticalSpeed);
    return line;
}

auto OledDisplay::headingText() const -> String
{
    const MagnetometerSensor* mag = autopilot ? autopilot->getMagnetometerSensor() : nullptr;
    if (!mag || !mag->isAvailable()) return "H---";

    char text[8];
    snprintf(text, sizeof(text), "H%3.0f", mag->getMagData().headingDegrees);
    return text;
}
