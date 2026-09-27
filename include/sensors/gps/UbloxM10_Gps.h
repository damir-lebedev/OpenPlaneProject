#pragma once
#include <Arduino.h>

#include "config/Config.h"
#include "hal/IUartPort.h"
#include "sensors/SensorInterface.h"

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

class UbloxM10_Gps : public GpsSensor
{
public:

    explicit UbloxM10_Gps(IUartPort& port);

    bool begin() override;

    // hasValidFrame один раз становится true после первого разобранного
    // кадра и дальше не сбрасывается сам по себе — если модуль отключат
    // или он перестанет слать данные, isAvailable() должен перестать
    // врать, что GPS на связи. Поэтому дополнительно проверяем, что
    // последний кадр пришёл не более Config::GPS_TIMEOUT_US назад (тот
    // же принцип, что IBusReceiver::isSignalLost() для RC).
    bool isAvailable() const override;

    void update() override;

    const GpsData& getGpsData() const override;

    bool hasFix() const override;

    const char* getSensorType() const override;

    void printStatus() const override;


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
        uint8_t buffer[64] = {};
        uint16_t length = 0;

        void addKey(uint32_t key) { put(key, 4); }
        void put(uint32_t value, uint8_t bytes);
    };

    IUartPort& uart;

    bool hasValidFrame = false;
    GpsData gpsData = {};

    // Состояние побайтового парсера UBX-кадра.
    ParseState state = ParseState::SYNC1;
    uint8_t msgClass = 0, msgId = 0;
    uint16_t payloadLen = 0;
    uint16_t payloadIndex = 0;
    uint8_t ckA = 0, ckB = 0;           // накапливаемая контрольная сумма
    uint8_t ckARecv = 0, ckBRecv = 0;
    bool isNavPvt = false;
    uint8_t payload[NAV_PVT_LEN] = {};

    // Побайтовый разбор — как в IBusReceiver::processByte(), кадры
    // приходят из UART порциями произвольного размера.
    void feedParser(uint8_t b);

    void parseNavPvt();

    int32_t readI32(uint16_t offset) const;

    uint32_t readU32(uint16_t offset) const;

    void sendUbxMessage(uint8_t msgClassOut, uint8_t msgIdOut, const uint8_t* msgPayload, uint16_t len);
};
