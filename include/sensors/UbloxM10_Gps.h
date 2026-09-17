#pragma once
#include <Arduino.h>

// ============================================================
// u-blox M10 (QUESCAN, чип UBX-M10050-KB) — GPS/ГЛОНАСС/Galileo/BeiDou
//
// UART, протокол UBX binary — разбирается только NAV-PVT (класс
// 0x01, id 0x07, фиксированные 92 байта payload), этого одного
// сообщения достаточно для координат/высоты/скорости/курса/фикса/
// числа спутников/оценок точности разом.
//
// Формат кадра: 0xB5 0x62 [class][id][len_lo][len_hi][payload][ck_a][ck_b]
// Контрольная сумма — 8-битная Флетчера по class+id+len+payload.
//
// begin() отправляет UBX-CFG-RATE (частота решений) и UBX-CFG-MSG
// (включить вывод NAV-PVT) — legacy-команды конфигурации, БЕЗ
// ожидания ACK: модуль может быть подключён без ответа (или вообще
// без TX-пина, см. ниже), и это не должно блокировать setup().
//
// isAvailable() здесь означает не "чип отвечает по шине" (как у
// I2C-датчиков — тут нет ACK на уровне протокола), а "хотя бы один
// валидный кадр NAV-PVT успешно разобран после begin()".
//
// На ESP32-C3 SuperMini физически не хватило пина под GPS TX (см.
// Config::PIN_GPS_TX == -1 и комментарий в Config.h) — GPS там
// работает только на приём, конфигурация (CFG-RATE/CFG-MSG) не
// уходит, и модуль отдаёт то, что выводит по заводским настройкам.
// ============================================================

#include "SensorInterface.h"
#include "../hal/IUartPort.h"

class UbloxM10_Gps : public GpsSensor
{
public:

    explicit UbloxM10_Gps(IUartPort& port)
        : uart(port),
          hasValidFrame(false)
    {
        memset(&gpsData, 0, sizeof(gpsData));
        resetParser();
    }

    bool begin() override
    {
        uart.begin(9600);  // заводская скорость u-blox по умолчанию

        delay(100);

        // UBX-CFG-RATE: measRate=100мс (10Гц — с запасом от заявленных
        // модулем до 25Гц), navRate=1 цикл, timeRef=1 (GPS-время).
        const uint8_t rateMsg[6] = { 0x64, 0x00,  0x01, 0x00,  0x01, 0x00 };
        sendUbxMessage(0x06, 0x08, rateMsg, sizeof(rateMsg));

        delay(50);

        // UBX-CFG-MSG: включить вывод NAV-PVT на текущем порту,
        // rate=1 (каждое навигационное решение).
        const uint8_t msgMsg[3] = { 0x01, 0x07, 0x01 };
        sendUbxMessage(0x06, 0x01, msgMsg, sizeof(msgMsg));

        return true;  // best-effort: без ACK нечего проверять программно
    }

    bool isAvailable() const override
    {
        return hasValidFrame;
    }

    void update() override
    {
        while (uart.available())
        {
            feedParser((uint8_t)uart.read());
        }
    }

    const GpsData& getGpsData() const override
    {
        return gpsData;
    }

    bool hasFix() const override
    {
        return hasValidFrame && gpsData.fixType >= 2;
    }

    const char* getSensorType() const override
    {
        return "u-blox M10 (UBX-M10050-KB)";
    }

    void printStatus() const override
    {
        Serial.print("GPS: available="); Serial.print(hasValidFrame ? "YES" : "NO");
        Serial.print(" fix="); Serial.print(gpsData.fixType);
        Serial.print(" numSV="); Serial.print(gpsData.numSatellites);
        Serial.print(" lat="); Serial.print(gpsData.latitude, 6);
        Serial.print(" lon="); Serial.print(gpsData.longitude, 6);
        Serial.print(" alt="); Serial.print(gpsData.altitude, 1);
        Serial.print(" hAcc="); Serial.print(gpsData.horizontalAccuracy, 1);
        Serial.println("m");
    }


private:

    enum class ParseState : uint8_t
    {
        SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B
    };

    static constexpr uint16_t NAV_PVT_LEN = 92;
    static constexpr uint8_t UBX_CLASS_NAV = 0x01;
    static constexpr uint8_t UBX_ID_NAV_PVT = 0x07;

    IUartPort& uart;

    bool hasValidFrame;
    GpsData gpsData;

    // Состояние побайтового парсера UBX-кадра.
    ParseState state;
    uint8_t msgClass, msgId;
    uint16_t payloadLen;
    uint16_t payloadIndex;
    uint8_t ckA, ckB;           // накапливаемая контрольная сумма
    uint8_t ckARecv, ckBRecv;
    bool isNavPvt;
    uint8_t payload[NAV_PVT_LEN];

    void resetParser()
    {
        state = ParseState::SYNC1;
    }

    // Побайтовый разбор — как в IBusReceiver::processByte(), кадры
    // приходят из UART порциями произвольного размера.
    void feedParser(uint8_t b)
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

    void parseNavPvt()
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

    int32_t readI32(uint16_t offset) const
    {
        return (int32_t)(
            (uint32_t)payload[offset] |
            ((uint32_t)payload[offset + 1] << 8) |
            ((uint32_t)payload[offset + 2] << 16) |
            ((uint32_t)payload[offset + 3] << 24));
    }

    uint32_t readU32(uint16_t offset) const
    {
        return (uint32_t)readI32(offset);
    }

    void sendUbxMessage(uint8_t msgClassOut, uint8_t msgIdOut, const uint8_t* msgPayload, uint16_t len)
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
};
