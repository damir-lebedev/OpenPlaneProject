#pragma once
#include <Arduino.h>
#include <math.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "autopilot/PidController.h"
#include "config/Config.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "hal/IBoard.h"
#include "hal/IUartPort.h"
#include "rc/RcChannelState.h"
#include "sensors/SensorInterface.h"
#include "sensors/airspeed/AirspeedSensor.h"
#include "telemetry/LoopStats.h"
#include "telemetry/MavlinkCodec.h"

// ============================================================
// ТЕЛЕМЕТРИЯ MAVLINK: борт -> наземная станция по радиомодему
//
// QGroundControl / Mission Planner видят самолёт ArduPilot
// (MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_ARDUPILOTMEGA): горизонт,
// карта с домом, скорость/высота, режим именем ArduPlane, каналы
// пульта и выходы сервоприводов, сообщения о событиях.
//
// Потоки (Гц): ATTITUDE 10; GLOBAL_POSITION_INT, VFR_HUD 5;
// GPS_RAW_INT, RC_CHANNELS, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT 2;
// HEARTBEAT, SYS_STATUS 1; HOME_POSITION 0.2 — около 1.2 КБ/с,
// с запасом для SiK на 57600 бод.
//
// С земли принимается:
//   - смена режима (SET_MODE, COMMAND_LONG DO_SET_MODE) — как с
//     дашборда: действует, пока пилот не щёлкнет тумблером режима;
//     AUTO не принимается (у OpenPlane нет миссий);
//   - параметры: ПИД крена и тангажа (RLL_KP ... PTCH_KD) — чтение и
//     запись из окна параметров GCS. Не сохраняются: удачные значения
//     переносятся в Config.h;
//   - запросы миссий — ответ "0 точек", чтобы GCS не ждала.
// ARM/DISARM с земли отклоняется: армится только тумблером пульта.
//
// update() вызывается из полётного цикла: разбор входящих байтов и
// отправка не больше двух кадров за такт — десятки микросекунд.
// Если в буфере UART нет места под кадр, кадр откладывается до
// следующего такта: запись в порт никогда не блокирует цикл.
// ============================================================

namespace MavlinkModes
{
    // Номера режимов ArduPlane (custom_mode).
    constexpr uint32_t PLANE_MANUAL = 0;
    constexpr uint32_t PLANE_CIRCLE = 1;
    constexpr uint32_t PLANE_STABILIZE = 2;
    constexpr uint32_t PLANE_ACRO = 4;
    constexpr uint32_t PLANE_FBWA = 5;
    constexpr uint32_t PLANE_FBWB = 6;
    constexpr uint32_t PLANE_CRUISE = 7;
    constexpr uint32_t PLANE_AUTO = 10;
    constexpr uint32_t PLANE_RTL = 11;
    constexpr uint32_t PLANE_LOITER = 12;
    constexpr uint32_t PLANE_TAKEOFF = 13;
    constexpr uint32_t PLANE_THERMAL = 24;

    // Ближайший по смыслу режим ArduPlane. Failsafe показывается тем,
    // что самолёт делает на самом деле: домой (RTL) или круги (CIRCLE).
    uint32_t toCustomMode(AutopilotMode mode, bool failsafeReturning, bool failsafeGliding);

    // Режим, который можно включить с земли. false — такого нет или нельзя.
    bool fromCustomMode(uint32_t custom, AutopilotMode& mode);

    // Автономные режимы (самолёт сам выбирает курс).
    bool isAutonomous(AutopilotMode mode);
}


class MavlinkTelemetry
{
public:

    // MAV_SEVERITY
    static constexpr uint8_t SEVERITY_CRITICAL = 2;
    static constexpr uint8_t SEVERITY_WARNING = 4;
    static constexpr uint8_t SEVERITY_NOTICE = 5;
    static constexpr uint8_t SEVERITY_INFO = 6;

    // Параметры, доступные из GCS.
    static constexpr uint8_t PARAM_COUNT = 6;

    MavlinkTelemetry(IUartPort& uart, FlightController& flightController, Autopilot* ap,
                     const LoopStats* stats = nullptr);

    void begin(uint32_t baud = Config::TELEM_BAUDRATE);

    void update();

    // Сообщение в ленту GCS (до 50 символов). Очередь на 4 строки.
    void statusText(uint8_t severity, const char* text);

    bool isGcsConnected() const { return gcsSeen && millis() - lastGcsHeartbeatMs < 3000; }
    uint32_t getSentFrames() const { return sentFrames; }
    uint32_t getDeferredFrames() const { return deferredFrames; }
    const Mavlink::Parser& getParser() const { return parser; }

    static const char* paramName(uint8_t index);


private:

    // MAV_CMD / MAV_RESULT
    static constexpr uint16_t CMD_DO_SET_MODE = 176;
    static constexpr uint16_t CMD_COMPONENT_ARM_DISARM = 400;
    static constexpr uint16_t CMD_REQUEST_MESSAGE = 512;
    static constexpr uint8_t RESULT_ACCEPTED = 0;
    static constexpr uint8_t RESULT_DENIED = 2;
    static constexpr uint8_t RESULT_UNSUPPORTED = 3;

    static constexpr uint8_t TEXT_QUEUE = 4;

    struct Stream
    {
        uint32_t id;
        uint32_t periodMs;
        uint32_t nextMs;
    };

    static constexpr uint8_t STREAM_COUNT = 10;

    struct TextEntry
    {
        uint8_t severity = SEVERITY_INFO;
        char text[51] = {};
    };

    IUartPort& port;
    FlightController& controller;
    Autopilot* autopilot;
    const LoopStats* loopStats;

    Mavlink::Encoder encoder;
    Mavlink::Parser parser;
    uint8_t frame[Mavlink::MAX_FRAME] = {};

    Stream streams[STREAM_COUNT] = {
        { Mavlink::Msg::HEARTBEAT, 1000, 0 },
        { Mavlink::Msg::ATTITUDE, 100, 0 },
        { Mavlink::Msg::GLOBAL_POSITION_INT, 200, 0 },
        { Mavlink::Msg::VFR_HUD, 200, 0 },
        { Mavlink::Msg::SYS_STATUS, 1000, 0 },
        { Mavlink::Msg::GPS_RAW_INT, 500, 0 },
        { Mavlink::Msg::RC_CHANNELS, 500, 0 },
        { Mavlink::Msg::SERVO_OUTPUT_RAW, 500, 0 },
        { Mavlink::Msg::NAV_CONTROLLER_OUTPUT, 500, 0 },
        { Mavlink::Msg::HOME_POSITION, 5000, 0 },
    };

    TextEntry texts[TEXT_QUEUE];
    uint8_t textHead = 0;
    uint8_t textCount = 0;

    int16_t paramToSend = -1;       // следующий параметр списка, -1 — нет
    int16_t singleParam = -1;       // ответ на PARAM_REQUEST_READ / PARAM_SET
    bool ackPending = false;
    uint16_t ackCommand = 0;
    uint8_t ackResult = 0;
    uint8_t ackTargetSystem = 0;
    uint8_t ackTargetComponent = 0;
    bool missionCountPending = false;
    uint8_t missionType = 0;
    uint8_t missionTargetSystem = 0;
    uint8_t missionTargetComponent = 0;

    bool gcsSeen = false;
    uint32_t lastGcsHeartbeatMs = 0;
    uint32_t sentFrames = 0;
    uint32_t deferredFrames = 0;

    // Для сообщений о событиях.
    bool eventsPrimed = false;
    bool wasArmed = false;
    bool wasFailsafe = false;
    bool wasFence = false;
    AutopilotMode lastMode = MODE_MANUAL;


    // ---------------- отправка ----------------

    bool send(uint32_t msgid, const Mavlink::Payload& payload);

    bool sendStream(uint32_t id, uint32_t now);

    AutopilotMode mode() const { return autopilot ? autopilot->getMode() : MODE_MANUAL; }

    bool sendHeartbeat();

    bool sendAttitude(uint32_t now);

    const GpsData* gpsFix() const;

    float relativeAltitude() const { return autopilot ? autopilot->getAltitude() : 0.0f; }

    float climbRate() const;

    float headingDeg() const;

    bool sendGlobalPosition(uint32_t now);

    bool sendVfrHud();

    bool sendSysStatus();

    bool sendGpsRaw();

    bool sendRcChannels(uint32_t now);

    bool sendServoOutputs();

    bool sendNavController();

    bool sendHome();

    bool sendParam(uint8_t index);

    bool sendPendingParam();

    bool sendPendingAck();

    bool sendPendingMissionCount();

    bool sendPendingStatusText();


    // ---------------- параметры ----------------

    float paramValue(uint8_t index) const;

    int16_t paramIndex(const char* name) const;

    // Коэффициенты ПИД: конечные, не отрицательные, не безумные.
    bool setParam(uint8_t index, float value);


    // ---------------- приём ----------------

    void receive();

    bool addressedToUs(uint8_t targetSystem) const;

    void handle(const Mavlink::Message& m);

    bool applyMode(uint32_t custom);

    void handleCommand(const Mavlink::Message& m);

    // Внеочередная отправка потокового сообщения.
    bool requestMessage(uint32_t id);


    // ---------------- события ----------------

    void watchEvents();


    // ---------------- мелочи ----------------

    static float wrap360(float degrees);

    // Курс 0..35999 сотых градуса.
    static uint16_t centiDegrees(float degrees);

    static int16_t clamp16(float value);
};
