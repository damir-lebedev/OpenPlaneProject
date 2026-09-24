// ============================================================
// RC JOYSTICK — пульт FS-i6 как USB-джойстик для симулятора
//
// Отдельная прошивка, с полётником не связана (своя папка, свой
// platformio.ini, ни одного общего файла). ESP32-S3 читает каналы
// пульта и отдаёт их компьютеру как обычный USB-джойстик: 8 осей по
// 16 бит (полное разрешение пульта, 1 мкс) + 8 кнопок. Драйверы не
// нужны — Windows видит стандартный HID-джойстик.
//
// Источники сигнала (слушаются оба, iBUS главнее):
//   • iBUS от приёмника FS-iA6B — как на стенде полётника: сигнал
//     iBUS SERVO на GPIO17, приёмник от 5V/GND платы. Ничего паять
//     не надо — стенд уже так подключён.
//   • PPM с тренерского разъёма пульта на GPIO14 (через 1 кОм, земля
//     общая). В тренерском разъёме FS-i6 — PPM (импульсы), а не UART,
//     поэтому USB-UART переходник его не прочитает.
//
// Разъёмы платы: "USB" — в компьютер, это и есть джойстик; "COM" —
// прошивка и лог (что за сигнал, значения каналов).
//
// Оси:    CH1..CH8 -> X, Y, Z, Rx, Ry, Rz, ползунок 1, ползунок 2
//         (0..1000 = 1000..2000 мкс).
// Кнопки: 1..6 = CH5..CH10 выше 1750 мкс — тумблеры можно назначить
//         в симуляторе как кнопки (сброс, смена вида).
// Нет сигнала — оси в центре, газ (CH3) внизу, кнопки отпущены.
// ============================================================

#include <Arduino.h>

#include "USB.h"
#include "USBHID.h"

namespace
{
    constexpr uint8_t PIN_IBUS = 17;
    constexpr uint8_t PIN_PPM = 14;

    constexpr uint8_t CHANNELS = 10;
    constexpr uint8_t AXES = 8;
    constexpr uint8_t BUTTON_FIRST_CHANNEL = 4;    // CH5
    constexpr uint16_t BUTTON_ON_US = 1750;
    constexpr uint8_t THROTTLE_CHANNEL = 2;        // CH3

    constexpr uint32_t SIGNAL_TIMEOUT_MS = 200;
    constexpr uint32_t REPORT_MIN_INTERVAL_MS = 5;    // не чаще 200 Гц
    constexpr uint32_t REPORT_KEEPALIVE_MS = 100;
    constexpr uint32_t LOG_INTERVAL_MS = 1000;
    constexpr uint16_t LOG_DEADBAND_US = 5;

    constexpr uint8_t REPORT_ID = 1;

    // Джойстик: 8 осей 0..1000 по 16 бит + 8 кнопок.
    const uint8_t REPORT_DESCRIPTOR[] = {
        0x05, 0x01,             // Usage Page (Generic Desktop)
        0x09, 0x04,             // Usage (Joystick)
        0xA1, 0x01,             // Collection (Application)
        0x85, REPORT_ID,        //   Report ID
        0x09, 0x30, 0x09, 0x31, 0x09, 0x32,   //   X, Y, Z
        0x09, 0x33, 0x09, 0x34, 0x09, 0x35,   //   Rx, Ry, Rz
        0x09, 0x36, 0x09, 0x36,               //   Slider, Slider
        0x15, 0x00,             //   Logical Minimum (0)
        0x26, 0xE8, 0x03,       //   Logical Maximum (1000)
        0x75, 0x10,             //   Report Size (16)
        0x95, AXES,             //   Report Count (8)
        0x81, 0x02,             //   Input (Data, Variable, Absolute)
        0x05, 0x09,             //   Usage Page (Button)
        0x19, 0x01,             //   Usage Minimum (1)
        0x29, 0x08,             //   Usage Maximum (8)
        0x15, 0x00,             //   Logical Minimum (0)
        0x25, 0x01,             //   Logical Maximum (1)
        0x75, 0x01,             //   Report Size (1)
        0x95, 0x08,             //   Report Count (8)
        0x81, 0x02,             //   Input (Data, Variable, Absolute)
        0xC0                    // End Collection
    };

    struct __attribute__((packed)) JoystickReport
    {
        uint16_t axes[AXES];
        uint8_t buttons;
    };

    class UsbJoystick : public USBHIDDevice
    {
    public:
        UsbJoystick()
        {
            static bool registered = false;
            if (!registered)
            {
                registered = true;
                USBHID::addDevice(this, sizeof(REPORT_DESCRIPTOR));
            }
        }

        void begin() { hid.begin(); }

        // false — компьютер не подключён или не готов принять отчёт.
        bool send(const JoystickReport& report)
        {
            if (!USB || !hid.ready()) return false;
            return hid.SendReport(REPORT_ID, &report, sizeof(report));
        }

        uint16_t _onGetDescriptor(uint8_t* buffer) override
        {
            memcpy(buffer, REPORT_DESCRIPTOR, sizeof(REPORT_DESCRIPTOR));
            return sizeof(REPORT_DESCRIPTOR);
        }

    private:
        USBHID hid;
    };

    // --- iBUS: кадр 32 байта, 0x20 0x40, 14 каналов, контрольная сумма ---

    class IbusReader
    {
    public:
        void begin()
        {
            Serial1.begin(115200, SERIAL_8N1, PIN_IBUS, -1);
        }

        void poll()
        {
            while (Serial1.available())
            {
                const uint8_t b = static_cast<uint8_t>(Serial1.read());
                if ((position == 0 && b != 0x20) || (position == 1 && b != 0x40))
                {
                    position = 0;
                    continue;
                }
                frame[position++] = b;
                if (position == FRAME_SIZE)
                {
                    position = 0;
                    decode();
                }
            }
        }

        bool alive() const { return received && millis() - lastFrameMs < SIGNAL_TIMEOUT_MS; }
        const uint16_t* channels() const { return values; }

    private:
        static constexpr uint8_t FRAME_SIZE = 32;

        uint8_t frame[FRAME_SIZE] = {};
        uint8_t position = 0;
        uint16_t values[CHANNELS] = {};
        bool received = false;
        uint32_t lastFrameMs = 0;

        void decode()
        {
            uint16_t sum = 0xFFFF;
            for (uint8_t i = 0; i < FRAME_SIZE - 2; ++i) sum -= frame[i];
            if (sum != (frame[30] | (frame[31] << 8))) return;

            for (uint8_t ch = 0; ch < CHANNELS; ++ch)
            {
                values[ch] = (frame[2 + ch * 2] | (frame[3 + ch * 2] << 8)) & 0x0FFF;
            }
            received = true;
            lastFrameMs = millis();
        }
    };

    // --- PPM: длительность между фронтами = канал, пауза > 3 мс = конец кадра ---
    //
    // Меряется от фронта до фронта одного направления — так всё равно,
    // прямой у пульта PPM или инверсный.

    constexpr uint32_t PPM_SYNC_US = 3000;
    constexpr uint32_t PPM_MIN_US = 800;
    constexpr uint32_t PPM_MAX_US = 2200;

    volatile uint32_t ppmLastEdgeUs = 0;
    volatile uint8_t ppmIndex = 0;
    volatile uint16_t ppmPending[CHANNELS];
    volatile uint16_t ppmFrame[CHANNELS];
    volatile uint8_t ppmFrameChannels = 0;
    volatile bool ppmFrameReady = false;

    void IRAM_ATTR onPpmEdge()
    {
        const uint32_t now = micros();
        const uint32_t width = now - ppmLastEdgeUs;
        ppmLastEdgeUs = now;

        if (width > PPM_SYNC_US)
        {
            // Конец кадра: хотя бы 4 канала — кадр настоящий.
            if (ppmIndex >= 4 && ppmIndex <= CHANNELS)
            {
                for (uint8_t i = 0; i < ppmIndex; ++i) ppmFrame[i] = ppmPending[i];
                ppmFrameChannels = ppmIndex;
                ppmFrameReady = true;
            }
            ppmIndex = 0;
            return;
        }

        if (width < PPM_MIN_US || width > PPM_MAX_US || ppmIndex >= CHANNELS)
        {
            ppmIndex = CHANNELS + 1;   // кадр испорчен — до следующей паузы
            return;
        }
        ppmPending[ppmIndex++] = width;
    }

    class PpmReader
    {
    public:
        void begin()
        {
            pinMode(PIN_PPM, INPUT_PULLDOWN);
            attachInterrupt(digitalPinToInterrupt(PIN_PPM), onPpmEdge, RISING);
        }

        void poll()
        {
            if (!ppmFrameReady) return;

            noInterrupts();
            count = ppmFrameChannels;
            for (uint8_t i = 0; i < count; ++i) values[i] = ppmFrame[i];
            ppmFrameReady = false;
            interrupts();

            lastFrameMs = millis();
            received = true;
        }

        bool alive() const { return received && millis() - lastFrameMs < SIGNAL_TIMEOUT_MS; }
        const uint16_t* channels() const { return values; }
        uint8_t channelCount() const { return count; }

    private:
        uint16_t values[CHANNELS] = {};
        uint8_t count = 0;
        bool received = false;
        uint32_t lastFrameMs = 0;
    };

    UsbJoystick joystick;
    IbusReader ibus;
    PpmReader ppm;

    JoystickReport lastSent = {};
    uint32_t lastSendMs = 0;

    const char* lastLoggedSource = "";
    uint16_t lastLogged[CHANNELS] = {};
    uint32_t lastLogMs = 0;

    JoystickReport buildReport(const uint16_t* channels, uint8_t count)
    {
        JoystickReport report = {};
        for (uint8_t axis = 0; axis < AXES; ++axis)
        {
            if (channels && axis < count)
            {
                report.axes[axis] = constrain(channels[axis], 1000, 2000) - 1000;
            }
            else
            {
                // Нет сигнала: стики в центре, газ внизу.
                report.axes[axis] = axis == THROTTLE_CHANNEL ? 0 : 500;
            }
        }

        for (uint8_t button = 0; button < 6 && channels; ++button)
        {
            const uint8_t ch = BUTTON_FIRST_CHANNEL + button;
            if (ch < count && channels[ch] > BUTTON_ON_US) report.buttons |= 1 << button;
        }
        return report;
    }

    // Лог в "COM": раз в секунду, только если что-то поменялось.
    void logState(const char* source, const uint16_t* channels, uint8_t count, bool usbConnected)
    {
        const uint32_t now = millis();
        if (now - lastLogMs < LOG_INTERVAL_MS) return;

        bool changed = source != lastLoggedSource;
        for (uint8_t ch = 0; ch < CHANNELS && channels && !changed; ++ch)
        {
            const uint16_t value = ch < count ? channels[ch] : 0;
            changed = abs(int(value) - int(lastLogged[ch])) > LOG_DEADBAND_US;
        }
        if (!changed) return;

        lastLogMs = now;
        lastLoggedSource = source;
        Serial.printf("%-5s USB=%s |", source, usbConnected ? "OK" : "нет");
        for (uint8_t ch = 0; ch < CHANNELS; ++ch)
        {
            const uint16_t value = (channels && ch < count) ? channels[ch] : 0;
            lastLogged[ch] = value;
            if (channels && ch < count) Serial.printf(" %u:%u", ch + 1, value);
        }
        Serial.println();
    }
}


void setup()
{
    Serial.begin(115200);

    joystick.begin();
    USB.productName("FS-i6 RC Joystick");
    USB.manufacturerName("OpenPlaneProject");
    USB.begin();

    ibus.begin();
    ppm.begin();

    Serial.println();
    Serial.println("RC JOYSTICK: пульт как USB-джойстик. Разъём \"USB\" — в компьютер.");
    Serial.println("Сигнал: iBUS от приёмника на GPIO17 или PPM тренерского разъёма на GPIO14.");
}

void loop()
{
    ibus.poll();
    ppm.poll();

    const uint16_t* channels = nullptr;
    uint8_t count = 0;
    const char* source = "нет";

    if (ibus.alive())
    {
        channels = ibus.channels();
        count = CHANNELS;
        source = "iBUS";
    }
    else if (ppm.alive())
    {
        channels = ppm.channels();
        count = ppm.channelCount();
        source = "PPM";
    }

    const JoystickReport report = buildReport(channels, count);
    const uint32_t now = millis();
    const bool changed = memcmp(&report, &lastSent, sizeof(report)) != 0;
    if ((changed && now - lastSendMs >= REPORT_MIN_INTERVAL_MS) || now - lastSendMs >= REPORT_KEEPALIVE_MS)
    {
        if (joystick.send(report)) lastSent = report;
        lastSendMs = now;
    }

    logState(source, channels, count, static_cast<bool>(USB));
    delay(1);
}
