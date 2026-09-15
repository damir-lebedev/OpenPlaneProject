#pragma once
// ============================================================
// iBUS RECEIVER
//
// Парсит iBUS-кадры из UART в 10 RC-каналов. Ничего не знает
// про failsafe-поведение самолёта, Servo или ARM — только UART
// -> RcChannelState. Замена протокола (S-Bus, PWM) требует
// правки только этого файла.
//
// Формат кадра (32 байта):
//   [0x20][0x40] [CH1 low][CH1 high] ... [CH10 low][CH10 high] [CRC low][CRC high]
//   CRC = 0xFFFF - (сумма первых 30 байт).
// ============================================================

class IBusReceiver
{
public:

    explicit IBusReceiver(HardwareSerial& serial)
        : serial(serial)
    {
    }

    void begin()
    {
        serial.begin(
            Config::IBUS_BAUDRATE,
            SERIAL_8N1,
            Config::PIN_IBUS,
            -1
        );

        lastFrameTime = micros();
    }

    // Вызывать каждый цикл: вычитывает всё, что накопилось в UART-буфере.
    void update()
    {
        while (serial.available())
        {
            const uint8_t byte = serial.read();
            processByte(byte);
        }
    }

    const RcChannelState& getState() const
    {
        return state;
    }

    // true, если корректный кадр не приходил дольше Config::RX_TIMEOUT_US.
    bool isSignalLost() const
    {
        return (micros() - lastFrameTime) > Config::RX_TIMEOUT_US;
    }

    uint32_t getLastFrameTime() const
    {
        return lastFrameTime;
    }


private:

    HardwareSerial& serial;

    RcChannelState state;

    uint8_t frame[Config::IBUS_FRAME_LENGTH] = {};

    uint8_t frameIndex = 0;

    uint32_t lastFrameTime = 0;

    // Побайтовый разбор кадра: байты могут приходить порциями,
    // поэтому нельзя ждать весь кадр за один serial.available().
    void processByte(uint8_t byte)
    {
        if (frameIndex == 0)
        {
            if (byte != Config::IBUS_HEADER_0)
            {
                return;
            }

            frame[frameIndex++] = byte;
            return;
        }

        if (frameIndex == 1)
        {
            if (byte != Config::IBUS_HEADER_1)
            {
                frameIndex = 0;  // 0x20 был случайным, начинаем заново
                return;
            }

            frame[frameIndex++] = byte;
            return;
        }

        frame[frameIndex++] = byte;

        if (frameIndex >= Config::IBUS_FRAME_LENGTH)
        {
            processFrame();
            frameIndex = 0;
        }
    }

    void processFrame()
    {
        uint16_t checksum = 0xFFFF;

        for (uint8_t i = 0; i < 30; ++i)
        {
            checksum -= frame[i];
        }

        const uint16_t receivedChecksum =
            static_cast<uint16_t>(frame[30]) |
            (static_cast<uint16_t>(frame[31]) << 8);

        if (checksum != receivedChecksum)
        {
            return;  // помехи в эфире — игнорируем кадр
        }

        for (uint8_t channel = 0;
             channel < Config::IBUS_CHANNELS;
             ++channel)
        {
            const uint8_t lowByte  = frame[2 + channel * 2];
            const uint8_t highByte = frame[3 + channel * 2];

            const uint16_t value =
                static_cast<uint16_t>(lowByte) |
                (static_cast<uint16_t>(highByte) << 8);

            state.set(channel, value);
        }

        lastFrameTime = micros();
    }
};
