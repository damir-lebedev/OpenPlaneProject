// Реализация rc/IBusReceiver.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "rc/IBusReceiver.h"


IBusReceiver::IBusReceiver(IUartPort& port)
: serial(port)
{
}

auto IBusReceiver::begin() -> void
{
    serial.begin(Config::IBUS_BAUDRATE);

    lastFrameTime = micros();
}

auto IBusReceiver::update() -> void
{
    while (serial.available())
    {
        processByte(static_cast<uint8_t>(serial.read()));
    }
}

auto IBusReceiver::getState() const -> const RcChannelState&
{
    return state;
}

auto IBusReceiver::isSignalLost() const -> bool
{
    return isFrameTimeout() || failsafeReported;
}

auto IBusReceiver::isFrameTimeout() const -> bool
{
    return !receivedAnyFrame || (micros() - lastFrameTime) > Config::RX_TIMEOUT_US;
}

auto IBusReceiver::isFailsafeReported() const -> bool
{
    return failsafeReported;
}

auto IBusReceiver::getLastFrameTime() const -> uint32_t
{
    return lastFrameTime;
}

auto IBusReceiver::processByte(uint8_t value) -> void
{
    if (frameIndex == 0)
    {
        if (value != Config::IBUS_HEADER_0)
        {
            return;
        }

        frame[frameIndex++] = value;
        return;
    }

    if (frameIndex == 1)
    {
        if (value != Config::IBUS_HEADER_1)
        {
            // 0x20 был случайным, начинаем заново — но этот байт сам
            // может быть началом кадра (0x20 0x20 0x40 ...).
            frameIndex = (value == Config::IBUS_HEADER_0) ? 1 : 0;
            return;
        }

        frame[frameIndex++] = value;
        return;
    }

    frame[frameIndex++] = value;

    if (frameIndex >= Config::IBUS_FRAME_LENGTH)
    {
        processFrame();
        frameIndex = 0;
    }
}

auto IBusReceiver::processFrame() -> void
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
        badFrames++;
        return;  // помехи на линии — игнорируем кадр
    }

    for (uint8_t channel = 0;
         channel < Config::IBUS_CHANNELS;
         ++channel)
    {
        const uint8_t lowByte  = frame[2 + channel * 2];
        const uint8_t highByte = frame[3 + channel * 2];

        // Значение канала — только младшие 12 бит. В старших 4 битах
        // FS-iA6B передаёт служебные данные (так кодируются каналы
        // 15-18), и они не нулевые, например, в failsafe: на стенде
        // CH3 = 0x2384 -> 900 мкс, а без маски читалось 9092, и
        // failsafe по газу (< 950) не срабатывал.
        const uint16_t value =
            (static_cast<uint16_t>(lowByte) |
             (static_cast<uint16_t>(highByte) << 8)) & 0x0FFF;

        state.set(channel, value);
    }

    failsafeReported =
        state.get(Channels::THROTTLE) < Config::RX_FAILSAFE_THROTTLE_US;

    goodFrames++;
    receivedAnyFrame = true;
    lastFrameTime = micros();
}
