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
// begin() настраивает модуль через UBX-CFG-VALSET (в RAM, без ACK):
// у M10 legacy-команды UBX-CFG-RATE/CFG-MSG/CFG-PRT больше не
// поддерживаются — только интерфейс конфигурации ключ-значение.
//   1. На заводских 9600 бод: UART1 -> 115200 бод. 10 решений в
//      секунду по ~100 байт NAV-PVT — это ~8 кбит/с сверх NMEA, в
//      9600 бод не помещается.
//   2. На 115200: 10 Гц, NAV-PVT на UART1, NMEA на выходе выключен.
// Если модуль уже работал на 115200 (конфиг в RAM пережил
// перезагрузку платы), первый шаг он просто проигнорирует, а второй
// примет. Ответа не ждём: модуль может быть подключён без TX-пина
// (см. ниже), и это не должно блокировать setup().
//
// isAvailable() здесь означает не "чип отвечает по шине" (как у
// I2C-датчиков — тут нет ACK на уровне протокола), а "хотя бы один
// валидный кадр NAV-PVT успешно разобран, и последний пришёл не
// позднее Config::GPS_TIMEOUT_US назад" — если модуль перестанет
// слать данные (потеря питания, обрыв провода), isAvailable() честно
// перестанет врать, что GPS на связи, вместо того чтобы навсегда
// застрять в last-known-good состоянии.
//
// На ESP32-C3 SuperMini физически не хватило пина под GPS TX (см.
// Config::PIN_GPS_TX == -1 и комментарий в Config.h) — GPS там
// работает только на приём, конфигурация (CFG-VALSET) не уходит, и
// модуль отдаёт то, что выводит по заводским настройкам (обычно NMEA
// на 9600 бод, без NAV-PVT).
// ============================================================

#include "config/Config.h"
#include "hal/IUartPort.h"
#include "sensors/SensorInterface.h"

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

    // hasValidFrame один раз становится true после первого разобранного
    // кадра и дальше не сбрасывается сам по себе — если модуль отключат
    // или он перестанет слать данные, isAvailable() должен перестать
    // врать, что GPS на связи. Поэтому дополнительно проверяем, что
    // последний кадр пришёл не более Config::GPS_TIMEOUT_US назад (тот
    // же принцип, что IBusReceiver::isSignalLost() для RC).
    bool isAvailable() const override
    {
        return hasValidFrame && (micros() - gpsData.timestamp) <= Config::GPS_TIMEOUT_US;
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
        return isAvailable() && gpsData.fixType >= 2;
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
    static constexpr uint8_t UBX_CLASS_CFG = 0x06;
    static constexpr uint8_t UBX_ID_VALSET = 0x8A;

    // Больше этого payload нам не встречается — кадр с большей
    // длиной считаем мусором (сбой синхронизации), а не ждём 64 КБ.
    static constexpr uint16_t MAX_PAYLOAD_LEN = 512;

    static constexpr uint32_t FACTORY_BAUD = 9600;
    static constexpr uint32_t WORK_BAUD = 115200;

    // Ключи конфигурации u-blox M10 (Interface Description, CFG-*).
    static constexpr uint32_t KEY_UART1_BAUDRATE        = 0x40520001;  // U4
    static constexpr uint32_t KEY_RATE_MEAS             = 0x30210001;  // U2, мс
    static constexpr uint32_t KEY_RATE_NAV              = 0x30210002;  // U2
    static constexpr uint32_t KEY_MSGOUT_NAV_PVT_UART1  = 0x20910007;  // U1
    static constexpr uint32_t KEY_UART1OUTPROT_UBX      = 0x10740001;  // L
    static constexpr uint32_t KEY_UART1OUTPROT_NMEA     = 0x10740002;  // L

    // Payload UBX-CFG-VALSET: version=0, layers=RAM, 2 резервных
    // байта, затем пары ключ (U4 LE) — значение (1/2/4 байта LE).
    class ValsetBuilder
    {
    public:
        ValsetBuilder() { buffer[0] = 0x00; buffer[1] = 0x01; buffer[2] = 0; buffer[3] = 0; length = 4; }

        void addU1(uint32_t key, uint8_t value)  { addKey(key); put(value, 1); }
        void addU2(uint32_t key, uint16_t value) { addKey(key); put(value, 2); }
        void addU4(uint32_t key, uint32_t value) { addKey(key); put(value, 4); }

        const uint8_t* data() const { return buffer; }
        uint16_t size() const { return length; }

    private:
        uint8_t buffer[64];
        uint16_t length;

        void addKey(uint32_t key) { put(key, 4); }
        void put(uint32_t value, uint8_t bytes)
        {
            for (uint8_t i = 0; i < bytes && length < sizeof(buffer); ++i)
            {
                buffer[length++] = (uint8_t)(value >> (8 * i));
            }
        }
    };

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
