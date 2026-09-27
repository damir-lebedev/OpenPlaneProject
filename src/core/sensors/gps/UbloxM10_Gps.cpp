// Реализация sensors/gps/UbloxM10_Gps.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "sensors/gps/UbloxM10_Gps.h"


UbloxM10_Gps::UbloxM10_Gps(IUartPort& port)
: uart(port)
{
}

auto UbloxM10_Gps::begin() -> bool
{
    // Шаг 1: заводские 9600 бод -> 115200.
    uart.begin(FACTORY_BAUD);
    delay(100);

    // Без TX-пина (ESP32-C3) модулю ничего не отправить — он
    // остаётся на заводских 9600 бод и настройках, слушаем как есть.
    if (Config::PIN_GPS_TX < 0)
    {
        return true;
    }

    ValsetBuilder baud;
    baud.addU4(KEY_UART1_BAUDRATE, WORK_BAUD);
    sendUbxMessage(UBX_CLASS_CFG, UBX_ID_VALSET, baud.data(), baud.size());
    delay(50);  // дать модулю дослать ответ и переключиться

    // Шаг 2: на рабочей скорости — частота и набор сообщений.
    uart.begin(WORK_BAUD);
    delay(50);

    ValsetBuilder config;
    config.addU2(KEY_RATE_MEAS, 100);              // 100 мс = 10 Гц
    config.addU2(KEY_RATE_NAV, 1);                 // решение на каждое измерение
    config.addU1(KEY_MSGOUT_NAV_PVT_UART1, 1);     // NAV-PVT на каждое решение
    config.addU1(KEY_UART1OUTPROT_UBX, 1);
    config.addU1(KEY_UART1OUTPROT_NMEA, 0);        // NMEA не нужен — меньше трафика
    sendUbxMessage(UBX_CLASS_CFG, UBX_ID_VALSET, config.data(), config.size());

    return true;  // best-effort: без ACK нечего проверять программно
}

auto UbloxM10_Gps::isAvailable() const -> bool
{
    return hasValidFrame && (micros() - gpsData.timestamp) <= Config::GPS_TIMEOUT_US;
}

auto UbloxM10_Gps::update() -> void
{
    while (uart.available())
    {
        feedParser((uint8_t)uart.read());
    }
}

auto UbloxM10_Gps::getGpsData() const -> const GpsData&
{
    return gpsData;
}

auto UbloxM10_Gps::hasFix() const -> bool
{
    return isAvailable() && gpsData.fixType >= 2;
}

auto UbloxM10_Gps::getSensorType() const -> const char*
{
    return "u-blox M10 (UBX-M10050-KB)";
}

auto UbloxM10_Gps::printStatus() const -> void
{
    Serial.print("GPS: available="); Serial.print(isAvailable() ? "YES" : "NO");
    Serial.print(" fix="); Serial.print(gpsData.fixType);
    Serial.print(" numSV="); Serial.print(gpsData.numSatellites);
    Serial.print(" lat="); Serial.print(gpsData.latitude, 6);
    Serial.print(" lon="); Serial.print(gpsData.longitude, 6);
    Serial.print(" alt="); Serial.print(gpsData.altitude, 1);
    Serial.print(" hAcc="); Serial.print(gpsData.horizontalAccuracy, 1);
    Serial.println("m");
}

auto UbloxM10_Gps::ValsetBuilder::put(uint32_t value, uint8_t bytes) -> void
{
    for (uint8_t i = 0; i < bytes && length < sizeof(buffer); ++i)
    {
        buffer[length++] = (uint8_t)(value >> (8 * i));
    }
}

auto UbloxM10_Gps::feedParser(uint8_t b) -> void
{
    switch (state)
    {
        case ParseState::SYNC1:
            if (b == 0xB5) state = ParseState::SYNC2;
            break;

        case ParseState::SYNC2:
            state = (b == 0x62) ? ParseState::CLASS : ParseState::SYNC1;
            break;

        case ParseState::CLASS:
            msgClass = b;
            ckA = b; ckB = b;
            state = ParseState::ID;
            break;

        case ParseState::ID:
            msgId = b;
            ckA += b; ckB += ckA;
            state = ParseState::LEN1;
            break;

        case ParseState::LEN1:
            payloadLen = b;
            ckA += b; ckB += ckA;
            state = ParseState::LEN2;
            break;

        case ParseState::LEN2:
            payloadLen |= ((uint16_t)b << 8);
            ckA += b; ckB += ckA;

            payloadIndex = 0;
            isNavPvt = (msgClass == UBX_CLASS_NAV && msgId == UBX_ID_NAV_PVT &&
                        payloadLen == NAV_PVT_LEN);

            if (payloadLen > MAX_PAYLOAD_LEN)
            {
                state = ParseState::SYNC1;  // не кадр, а сбой синхронизации
                break;
            }

            state = (payloadLen == 0) ? ParseState::CK_A : ParseState::PAYLOAD;
            break;

        case ParseState::PAYLOAD:
            ckA += b; ckB += ckA;

            if (isNavPvt && payloadIndex < NAV_PVT_LEN)
            {
                payload[payloadIndex] = b;
            }
            payloadIndex++;

            if (payloadIndex >= payloadLen) state = ParseState::CK_A;
            break;

        case ParseState::CK_A:
            ckARecv = b;
            state = ParseState::CK_B;
            break;

        case ParseState::CK_B:
            ckBRecv = b;

            if (isNavPvt && ckARecv == ckA && ckBRecv == ckB)
            {
                parseNavPvt();
            }

            state = ParseState::SYNC1;
            break;
    }
}

auto UbloxM10_Gps::parseNavPvt() -> void
{
    gpsData.fixType = payload[20];
    gpsData.numSatellites = payload[23];

    gpsData.longitude = readI32(24) * 1e-7;
    gpsData.latitude  = readI32(28) * 1e-7;

    gpsData.altitude = readI32(36) / 1000.0f;             // hMSL, мм -> м
    gpsData.horizontalAccuracy = readU32(40) / 1000.0f;   // hAcc, мм -> м
    gpsData.verticalAccuracy  = readU32(44) / 1000.0f;    // vAcc, мм -> м

    gpsData.groundSpeed = readI32(60) / 1000.0f;          // gSpeed, мм/с -> м/с

    float heading = readI32(64) * 1e-5f;                  // headMot, 1e-5 град
    if (heading < 0) heading += 360.0f;
    gpsData.heading = heading;

    gpsData.timestamp = micros();
    hasValidFrame = true;
}

auto UbloxM10_Gps::readI32(uint16_t offset) const -> int32_t
{
    return (int32_t)(
        (uint32_t)payload[offset] |
        ((uint32_t)payload[offset + 1] << 8) |
        ((uint32_t)payload[offset + 2] << 16) |
        ((uint32_t)payload[offset + 3] << 24));
}

auto UbloxM10_Gps::readU32(uint16_t offset) const -> uint32_t
{
    return (uint32_t)readI32(offset);
}

auto UbloxM10_Gps::sendUbxMessage(uint8_t msgClassOut, uint8_t msgIdOut, const uint8_t* msgPayload, uint16_t len) -> void
{
    uint8_t header[6] = {
        0xB5, 0x62, msgClassOut, msgIdOut,
        (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)
    };

    uint8_t sumA = 0, sumB = 0;
    for (uint8_t i = 2; i < 6; ++i) { sumA += header[i]; sumB += sumA; }
    for (uint16_t i = 0; i < len; ++i) { sumA += msgPayload[i]; sumB += sumA; }

    uart.write(header, 6);
    if (len > 0) uart.write(msgPayload, len);

    const uint8_t checksum[2] = { sumA, sumB };
    uart.write(checksum, 2);
}
