// Реализация telemetry/MavlinkTelemetry.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "telemetry/MavlinkTelemetry.h"

namespace MavlinkModes
{

uint32_t toCustomMode(AutopilotMode mode, bool failsafeReturning, bool failsafeGliding)
{
    if (failsafeReturning) return PLANE_RTL;
    if (failsafeGliding) return PLANE_CIRCLE;

    switch (mode)
    {
        case MODE_MANUAL: return PLANE_MANUAL;
        case MODE_STABILIZE: return PLANE_FBWA;
        case MODE_AUTO_TAKEOFF: return PLANE_TAKEOFF;
        case MODE_ALT_HOLD: return PLANE_FBWB;
        case MODE_ACRO: return PLANE_ACRO;
        case MODE_CRUISE: return PLANE_CRUISE;
        case MODE_LOITER: return PLANE_LOITER;
        case MODE_RTH: return PLANE_RTL;
        case MODE_LAUNCH: return PLANE_TAKEOFF;
        case MODE_AUTO_LAND: return PLANE_AUTO;
        case MODE_SOARING: return PLANE_THERMAL;
        case MODE_RESCUE: return PLANE_STABILIZE;
        default: return PLANE_MANUAL;
    }
}

bool fromCustomMode(uint32_t custom, AutopilotMode& mode)
{
    switch (custom)
    {
        case PLANE_MANUAL: mode = MODE_MANUAL; return true;
        case PLANE_STABILIZE: mode = MODE_RESCUE; return true;
        case PLANE_ACRO: mode = MODE_ACRO; return true;
        case PLANE_FBWA: mode = MODE_STABILIZE; return true;
        case PLANE_FBWB: mode = MODE_ALT_HOLD; return true;
        case PLANE_CRUISE: mode = MODE_CRUISE; return true;
        case PLANE_RTL: mode = MODE_RTH; return true;
        case PLANE_LOITER: mode = MODE_LOITER; return true;
        case PLANE_TAKEOFF: mode = MODE_AUTO_TAKEOFF; return true;
        case PLANE_THERMAL: mode = MODE_SOARING; return true;
        default: return false;   // AUTO, CIRCLE, GUIDED... — нет у OpenPlane
    }
}

bool isAutonomous(AutopilotMode mode)
{
    return mode == MODE_LOITER || mode == MODE_RTH || mode == MODE_AUTO_TAKEOFF || mode == MODE_LAUNCH ||
           mode == MODE_AUTO_LAND || mode == MODE_SOARING;
}

}  // namespace MavlinkModes


MavlinkTelemetry::MavlinkTelemetry(IUartPort& uart, FlightController& flightController, Autopilot* ap,
                     const LoopStats* stats)
: port(uart),
      controller(flightController),
      autopilot(ap),
      loopStats(stats),
      encoder(Config::MAVLINK_SYSTEM_ID, Config::MAVLINK_COMPONENT_ID)
{
}

auto MavlinkTelemetry::begin(uint32_t baud) -> void
{
    port.begin(baud);
    statusText(SEVERITY_INFO, "OpenPlane online");
}

auto MavlinkTelemetry::update() -> void
{
    const uint32_t now = millis();

    receive();
    watchEvents();

    // Не больше двух кадров за такт: ответы на запросы GCS — первыми.
    uint8_t budget = 2;
    if (sendPendingParam()) --budget;
    if (budget && sendPendingAck()) --budget;
    if (budget && sendPendingMissionCount()) --budget;
    if (budget && sendPendingStatusText()) --budget;

    for (uint8_t i = 0; i < STREAM_COUNT && budget > 0; ++i)
    {
        Stream& s = streams[i];
        if (static_cast<int32_t>(now - s.nextMs) < 0) continue;
        if (!sendStream(s.id, now)) continue;   // нет места или нечего слать
        s.nextMs = now + s.periodMs;
        --budget;
    }
}

auto MavlinkTelemetry::statusText(uint8_t severity, const char* text) -> void
{
    if (textCount >= TEXT_QUEUE) return;
    TextEntry& e = texts[(textHead + textCount) % TEXT_QUEUE];
    e.severity = severity;
    strncpy(e.text, text, sizeof(e.text) - 1);
    e.text[sizeof(e.text) - 1] = '\0';
    ++textCount;
}

auto MavlinkTelemetry::paramName(uint8_t index) -> const char*
{
    static const char* const names[PARAM_COUNT] = { "RLL_KP", "RLL_KI", "RLL_KD", "PTCH_KP", "PTCH_KI", "PTCH_KD" };
    return index < PARAM_COUNT ? names[index] : "";
}

auto MavlinkTelemetry::send(uint32_t msgid, const Mavlink::Payload& payload) -> bool
{
    const size_t length = encoder.encode(frame, msgid, payload);
    if (length == 0) return false;

    const int room = port.availableForWrite();
    if (room >= 0 && static_cast<size_t>(room) < length)
    {
        ++deferredFrames;
        return false;
    }
    port.write(frame, length);
    ++sentFrames;
    return true;
}

auto MavlinkTelemetry::sendStream(uint32_t id, uint32_t now) -> bool
{
    switch (id)
    {
        case Mavlink::Msg::HEARTBEAT: return sendHeartbeat();
        case Mavlink::Msg::ATTITUDE: return sendAttitude(now);
        case Mavlink::Msg::GLOBAL_POSITION_INT: return sendGlobalPosition(now);
        case Mavlink::Msg::VFR_HUD: return sendVfrHud();
        case Mavlink::Msg::SYS_STATUS: return sendSysStatus();
        case Mavlink::Msg::GPS_RAW_INT: return sendGpsRaw();
        case Mavlink::Msg::RC_CHANNELS: return sendRcChannels(now);
        case Mavlink::Msg::SERVO_OUTPUT_RAW: return sendServoOutputs();
        case Mavlink::Msg::NAV_CONTROLLER_OUTPUT: return sendNavController();
        case Mavlink::Msg::HOME_POSITION: return sendHome();
        default: return false;
    }
}

auto MavlinkTelemetry::sendHeartbeat() -> bool
{
    const bool armed = controller.isArmed();
    const bool failsafe = controller.isReceiverFailsafe() || (autopilot && autopilot->isFailsafeActive());
    const AutopilotMode m = mode();

    uint8_t baseMode = 1 | 64;                     // CUSTOM_MODE_ENABLED | MANUAL_INPUT_ENABLED
    if (m != MODE_MANUAL) baseMode |= 16;          // STABILIZE_ENABLED
    if (MavlinkModes::isAutonomous(m)) baseMode |= 4;   // AUTO_ENABLED
    if (armed) baseMode |= 128;                    // SAFETY_ARMED

    const uint32_t custom = MavlinkModes::toCustomMode(
        m, autopilot && autopilot->isFailsafeReturning(), autopilot && autopilot->isFailsafeGliding());

    Mavlink::Payload p;
    p.u32(custom)
     .u8(1)                                        // MAV_TYPE_FIXED_WING
     .u8(3)                                        // MAV_AUTOPILOT_ARDUPILOTMEGA
     .u8(baseMode)
     .u8(failsafe ? 5 : (armed ? 4 : 3))           // CRITICAL / ACTIVE / STANDBY
     .u8(3);                                       // версия MAVLink
    return send(Mavlink::Msg::HEARTBEAT, p);
}

auto MavlinkTelemetry::sendAttitude(uint32_t now) -> bool
{
    const ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
    if (!imu || !imu->isAvailable()) return false;

    const ImuData& d = imu->getImuData();
    constexpr float RAD = 0.01745329252f;
    Mavlink::Payload p;
    p.u32(now)
     .f32(d.roll * RAD).f32(d.pitch * RAD).f32(d.yaw * RAD)
     .f32(d.gyroX * RAD).f32(d.gyroY * RAD).f32(d.gyroZ * RAD);
    return send(Mavlink::Msg::ATTITUDE, p);
}

auto MavlinkTelemetry::gpsFix() const -> const GpsData*
{
    const GpsSensor* gps = autopilot ? autopilot->getGpsSensor() : nullptr;
    if (!gps || !gps->isAvailable()) return nullptr;
    const GpsData& d = gps->getGpsData();
    return d.fixType >= 2 ? &d : nullptr;
}

auto MavlinkTelemetry::climbRate() const -> float
{
    const BarometerSensor* baro = autopilot ? autopilot->getBarometerSensor() : nullptr;
    return (baro && baro->isAvailable()) ? baro->getBarometerData().verticalSpeed : 0.0f;
}

auto MavlinkTelemetry::headingDeg() const -> float
{
    const ImuSensor* imu = autopilot ? autopilot->getImuSensor() : nullptr;
    const MagnetometerSensor* mag = autopilot ? autopilot->getMagnetometerSensor() : nullptr;
    if (mag && mag->isAvailable()) return mag->getMagData().headingDegrees;
    if (imu && imu->isAvailable()) return wrap360(imu->getImuData().yaw);
    return 0.0f;
}

auto MavlinkTelemetry::sendGlobalPosition(uint32_t now) -> bool
{
    const GpsData* gps = gpsFix();
    if (!gps) return false;

    const float course = gps->heading * 0.01745329252f;
    const float speedCm = gps->groundSpeed * 100.0f;
    Mavlink::Payload p;
    p.u32(now)
     .i32(static_cast<int32_t>(lround(gps->latitude * 1e7)))
     .i32(static_cast<int32_t>(lround(gps->longitude * 1e7)))
     .i32(static_cast<int32_t>(lroundf(gps->altitude * 1000.0f)))
     .i32(static_cast<int32_t>(lroundf(relativeAltitude() * 1000.0f)))
     .i16(clamp16(speedCm * cosf(course)))
     .i16(clamp16(speedCm * sinf(course)))
     .i16(clamp16(-climbRate() * 100.0f))
     .u16(centiDegrees(headingDeg()));
    return send(Mavlink::Msg::GLOBAL_POSITION_INT, p);
}

auto MavlinkTelemetry::sendVfrHud() -> bool
{
    const AirspeedSensor* airspeed = autopilot ? autopilot->getAirspeedSensor() : nullptr;
    const GpsData* gps = gpsFix();
    const float ground = gps ? gps->groundSpeed : 0.0f;
    const float air = (airspeed && airspeed->isAvailable()) ? airspeed->getAirspeedData().indicatedMs : ground;

    const uint16_t throttleUs = controller.getOutputState().throttle;
    const long throttlePct = constrain(static_cast<long>(throttleUs) - 1000, 0L, 1000L) / 10;

    Mavlink::Payload p;
    p.f32(air).f32(ground).f32(relativeAltitude()).f32(climbRate())
     .i16(static_cast<int16_t>(centiDegrees(headingDeg()) / 100))
     .u16(static_cast<uint16_t>(throttlePct));
    return send(Mavlink::Msg::VFR_HUD, p);
}

auto MavlinkTelemetry::sendSysStatus() -> bool
{
    // MAV_SYS_STATUS_SENSOR: есть в сборке / исправен.
    uint32_t present = 0, health = 0;
    auto sensor = [&](const Sensor* s, uint32_t bits) {
        if (!s) return;
        present |= bits;
        if (s->isAvailable()) health |= bits;
    };
    if (autopilot)
    {
        sensor(autopilot->getImuSensor(), 0x01 | 0x02);    // 3D_GYRO | 3D_ACCEL
        sensor(autopilot->getMagnetometerSensor(), 0x04);  // 3D_MAG
        sensor(autopilot->getBarometerSensor(), 0x08);     // ABSOLUTE_PRESSURE
        sensor(autopilot->getAirspeedSensor(), 0x10);      // DIFFERENTIAL_PRESSURE
        sensor(autopilot->getGpsSensor(), 0x20);           // GPS

        present |= 0x400 | 0x800;                          // ANGULAR_RATE_CONTROL | ATTITUDE_STABILIZATION
        health |= 0x400 | 0x800;
        if (autopilot->getInputs().has(Feature::GEOFENCE))
        {
            present |= 0x100000;                           // GEOFENCE
            if (!autopilot->getNavStatus().fenceBreached) health |= 0x100000;
        }
    }
    present |= 0x8000 | 0x10000;                           // MOTOR_OUTPUTS | RC_RECEIVER
    health |= 0x8000;
    if (!controller.isReceiverFailsafe()) health |= 0x10000;

    uint16_t load = 0;   // загрузка цикла, промилле
    if (loopStats && loopStats->avgUs > 0)
    {
        const uint32_t permille = loopStats->avgUs / Config::LOOP_PERIOD_MS;   // мкс / (период, мс) = ‰
        load = static_cast<uint16_t>(permille > 1000 ? 1000 : permille);
    }

    Mavlink::Payload p;
    p.u32(present).u32(present).u32(health)
     .u16(load)
     .u16(UINT16_MAX)       // напряжение не измеряется
     .i16(-1)               // ток не измеряется
     .u16(0)                // drop_rate_comm
     .u16(static_cast<uint16_t>(parser.badCrcCount() > UINT16_MAX ? UINT16_MAX : parser.badCrcCount()))
     .u16(0).u16(0).u16(0).u16(0)
     .i8(-1);               // остаток батареи неизвестен
    return send(Mavlink::Msg::SYS_STATUS, p);
}

auto MavlinkTelemetry::sendGpsRaw() -> bool
{
    const GpsSensor* gps = autopilot ? autopilot->getGpsSensor() : nullptr;
    if (!gps) return false;

    const GpsData& d = gps->getGpsData();
    const bool ok = gps->isAvailable();
    Mavlink::Payload p;
    p.u64(static_cast<uint64_t>(millis()) * 1000ULL)
     .i32(ok ? static_cast<int32_t>(lround(d.latitude * 1e7)) : 0)
     .i32(ok ? static_cast<int32_t>(lround(d.longitude * 1e7)) : 0)
     .i32(ok ? static_cast<int32_t>(lroundf(d.altitude * 1000.0f)) : 0)
     .u16(UINT16_MAX)       // HDOP неизвестен (точность — в h_acc ниже)
     .u16(UINT16_MAX)
     .u16(ok ? static_cast<uint16_t>(lroundf(d.groundSpeed * 100.0f)) : 0)
     .u16(ok ? centiDegrees(d.heading) : UINT16_MAX)
     .u8(ok ? d.fixType : 0)
     .u8(ok ? d.numSatellites : 0)
     .i32(0)                                                   // alt_ellipsoid
     .u32(ok ? static_cast<uint32_t>(lroundf(d.horizontalAccuracy * 1000.0f)) : 0)   // h_acc, мм
     .u32(ok ? static_cast<uint32_t>(lroundf(d.verticalAccuracy * 1000.0f)) : 0);    // v_acc, мм
    return send(Mavlink::Msg::GPS_RAW_INT, p);
}

auto MavlinkTelemetry::sendRcChannels(uint32_t now) -> bool
{
    const RcChannelState& rc = controller.getRcState();
    Mavlink::Payload p;
    p.u32(now);
    for (uint8_t i = 0; i < 18; ++i)
    {
        p.u16(i < Config::IBUS_CHANNELS ? rc.get(i) : UINT16_MAX);
    }
    p.u8(Config::IBUS_CHANNELS).u8(controller.isReceiverFailsafe() ? 0 : UINT8_MAX);
    return send(Mavlink::Msg::RC_CHANNELS, p);
}

auto MavlinkTelemetry::sendServoOutputs() -> bool
{
    const FlightOutputState& state = controller.getOutputState();
    Mavlink::Payload p;
    p.u32(micros());
    for (uint8_t ch = 0; ch < 8; ++ch)
    {
        p.u16(ch < ServoChannel::COUNT ? FlightOutputs::valueOf(state, ch) : 0);
    }
    p.u8(0);   // port
    return send(Mavlink::Msg::SERVO_OUTPUT_RAW, p);
}

auto MavlinkTelemetry::sendNavController() -> bool
{
    if (!autopilot) return false;

    const NavStatus& nav = autopilot->getNavStatus();
    const float altError = autopilot->getTargetAltitude() - autopilot->getAltitude();
    Mavlink::Payload p;
    p.f32(autopilot->getDesiredRoll())
     .f32(autopilot->getDesiredPitch())
     .f32(mode() == MODE_MANUAL ? 0.0f : altError)
     .f32(0.0f)             // aspd_error
     .f32(0.0f)             // xtrack_error
     .i16(static_cast<int16_t>(lroundf(nav.targetCourseDeg)))
     .i16(static_cast<int16_t>(lroundf(nav.bearingHomeDeg)))
     .u16(nav.distanceHomeM >= 0 ? static_cast<uint16_t>(nav.distanceHomeM > 65535.0f ? 65535.0f : nav.distanceHomeM) : 0);
    return send(Mavlink::Msg::NAV_CONTROLLER_OUTPUT, p);
}

auto MavlinkTelemetry::sendHome() -> bool
{
    if (!autopilot) return false;
    const NavStatus& nav = autopilot->getNavStatus();
    if (!nav.homeValid) return false;

    // Высота дома над морем ≈ высота GPS сейчас − барометрическая над точкой старта.
    const GpsData* gps = gpsFix();
    const float homeMsl = gps ? gps->altitude - relativeAltitude() : 0.0f;

    Mavlink::Payload p;
    p.i32(static_cast<int32_t>(lround(nav.home.lat * 1e7)))
     .i32(static_cast<int32_t>(lround(nav.home.lon * 1e7)))
     .i32(static_cast<int32_t>(lroundf(homeMsl * 1000.0f)))
     .f32(0).f32(0).f32(0)                  // x, y, z
     .f32(1).f32(0).f32(0).f32(0)           // q — единичный
     .f32(0).f32(0).f32(0);                 // approach
    return send(Mavlink::Msg::HOME_POSITION, p);
}

auto MavlinkTelemetry::sendParam(uint8_t index) -> bool
{
    Mavlink::Payload p;
    p.f32(paramValue(index))
     .u16(PARAM_COUNT)
     .u16(index)
     .chars(paramName(index), 16)
     .u8(9);                                // MAV_PARAM_TYPE_REAL32
    return send(Mavlink::Msg::PARAM_VALUE, p);
}

auto MavlinkTelemetry::sendPendingParam() -> bool
{
    if (singleParam >= 0)
    {
        if (!sendParam(static_cast<uint8_t>(singleParam))) return false;
        singleParam = -1;
        return true;
    }
    if (paramToSend < 0) return false;
    if (!sendParam(static_cast<uint8_t>(paramToSend))) return false;
    if (++paramToSend >= PARAM_COUNT) paramToSend = -1;
    return true;
}

auto MavlinkTelemetry::sendPendingAck() -> bool
{
    if (!ackPending) return false;
    Mavlink::Payload p;
    p.u16(ackCommand).u8(ackResult).u8(0).i32(0).u8(ackTargetSystem).u8(ackTargetComponent);
    if (!send(Mavlink::Msg::COMMAND_ACK, p)) return false;
    ackPending = false;
    return true;
}

auto MavlinkTelemetry::sendPendingMissionCount() -> bool
{
    if (!missionCountPending) return false;
    Mavlink::Payload p;
    p.u16(0).u8(missionTargetSystem).u8(missionTargetComponent).u8(missionType);
    if (!send(Mavlink::Msg::MISSION_COUNT, p)) return false;
    missionCountPending = false;
    return true;
}

auto MavlinkTelemetry::sendPendingStatusText() -> bool
{
    if (textCount == 0) return false;
    const TextEntry& e = texts[textHead];
    Mavlink::Payload p;
    p.u8(e.severity).chars(e.text, 50);
    if (!send(Mavlink::Msg::STATUSTEXT, p)) return false;
    textHead = static_cast<uint8_t>((textHead + 1) % TEXT_QUEUE);
    --textCount;
    return true;
}

auto MavlinkTelemetry::paramValue(uint8_t index) const -> float
{
    if (!autopilot) return 0.0f;
    const PidController& pid = index < 3 ? autopilot->getRollPid() : autopilot->getPitchPid();
    switch (index % 3)
    {
        case 0: return pid.getKp();
        case 1: return pid.getKi();
        default: return pid.getKd();
    }
}

auto MavlinkTelemetry::paramIndex(const char* name) const -> int16_t
{
    for (uint8_t i = 0; i < PARAM_COUNT; ++i)
    {
        if (strncmp(name, paramName(i), 16) == 0) return i;
    }
    return -1;
}

auto MavlinkTelemetry::setParam(uint8_t index, float value) -> bool
{
    if (!autopilot || !isfinite(value) || value < 0.0f || value > 100.0f) return false;

    float gains[PARAM_COUNT];
    for (uint8_t i = 0; i < PARAM_COUNT; ++i) gains[i] = paramValue(i);
    gains[index] = value;
    autopilot->setPIDGains(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]);
    return true;
}

auto MavlinkTelemetry::receive() -> void
{
    // Не больше 128 байт за такт: полный кадр PARAM_SET — 35 байт.
    for (int n = 0; n < 128 && port.available() > 0; ++n)
    {
        const int value = port.read();
        if (value < 0) break;
        if (parser.feed(static_cast<uint8_t>(value))) handle(parser.message());
    }
}

auto MavlinkTelemetry::addressedToUs(uint8_t targetSystem) const -> bool
{
    return targetSystem == 0 || targetSystem == encoder.systemId();
}

auto MavlinkTelemetry::handle(const Mavlink::Message& m) -> void
{
    switch (m.msgid)
    {
        case Mavlink::Msg::HEARTBEAT:
            // Наземная станция: MAV_TYPE_GCS (6).
            if (m.u8(4) == 6)
            {
                gcsSeen = true;
                lastGcsHeartbeatMs = millis();
            }
            break;

        case Mavlink::Msg::PARAM_REQUEST_LIST:
            if (addressedToUs(m.u8(0))) paramToSend = 0;
            break;

        case Mavlink::Msg::PARAM_REQUEST_READ:
        {
            if (!addressedToUs(m.u8(2))) break;
            int16_t index = m.i16(0);   // −1 — искать по имени
            if (index < 0)
            {
                char name[17];
                m.chars(4, 16, name);
                index = paramIndex(name);
            }
            if (index >= 0 && index < PARAM_COUNT) singleParam = index;
            break;
        }

        case Mavlink::Msg::PARAM_SET:
        {
            if (!addressedToUs(m.u8(4))) break;
            char name[17];
            m.chars(6, 16, name);
            const int16_t index = paramIndex(name);
            if (index < 0) break;
            setParam(static_cast<uint8_t>(index), m.f32(0));
            singleParam = index;   // ответ — текущее значение (новое или отвергнутое)
            break;
        }

        case Mavlink::Msg::SET_MODE:
            if (addressedToUs(m.u8(4))) applyMode(m.u32(0));
            break;

        case Mavlink::Msg::COMMAND_LONG:
            if (addressedToUs(m.u8(30))) handleCommand(m);
            break;

        case Mavlink::Msg::MISSION_REQUEST_LIST:
            if (!addressedToUs(m.u8(0))) break;
            missionCountPending = true;
            missionTargetSystem = m.sysid;
            missionTargetComponent = m.compid;
            missionType = m.u8(2);
            break;

        default:
            break;   // REQUEST_DATA_STREAM и прочее: частоты фиксированы
    }
}

auto MavlinkTelemetry::applyMode(uint32_t custom) -> bool
{
    AutopilotMode target;
    if (!autopilot || !MavlinkModes::fromCustomMode(custom, target)) return false;
    autopilot->setMode(target);
    return true;
}

auto MavlinkTelemetry::handleCommand(const Mavlink::Message& m) -> void
{
    const uint16_t command = m.u16(28);
    uint8_t result = RESULT_UNSUPPORTED;

    switch (command)
    {
        case CMD_DO_SET_MODE:
            result = applyMode(static_cast<uint32_t>(m.f32(4))) ? RESULT_ACCEPTED : RESULT_DENIED;
            break;
        case CMD_COMPONENT_ARM_DISARM:
            result = RESULT_DENIED;   // только тумблером пульта
            statusText(SEVERITY_WARNING, "Arm/disarm: only from the transmitter");
            break;
        case CMD_REQUEST_MESSAGE:
            result = requestMessage(static_cast<uint32_t>(m.f32(0))) ? RESULT_ACCEPTED : RESULT_UNSUPPORTED;
            break;
        default:
            break;
    }

    ackPending = true;
    ackCommand = command;
    ackResult = result;
    ackTargetSystem = m.sysid;
    ackTargetComponent = m.compid;
}

auto MavlinkTelemetry::requestMessage(uint32_t id) -> bool
{
    // cppcheck-suppress useStlAlgorithm
    for (Stream& s : streams)
    {
        if (s.id == id)
        {
            s.nextMs = millis();
            return true;
        }
    }
    return false;
}

auto MavlinkTelemetry::watchEvents() -> void
{
    const bool armed = controller.isArmed();
    const bool failsafe = controller.isReceiverFailsafe() || (autopilot && autopilot->isFailsafeActive());
    const bool fence = autopilot && autopilot->getNavStatus().fenceBreached;
    const AutopilotMode m = mode();

    if (!eventsPrimed)
    {
        eventsPrimed = true;
        wasArmed = armed;
        wasFailsafe = failsafe;
        wasFence = fence;
        lastMode = m;
        return;
    }

    if (armed != wasArmed) statusText(SEVERITY_NOTICE, armed ? "ARMED" : "DISARMED");
    if (failsafe != wasFailsafe)
    {
        statusText(failsafe ? SEVERITY_CRITICAL : SEVERITY_NOTICE,
                   failsafe ? (autopilot && autopilot->isFailsafeReturning() ? "FAILSAFE: return home"
                                                                             : "FAILSAFE: link lost")
                            : "Link restored");
    }
    if (fence && !wasFence) statusText(SEVERITY_WARNING, "Geofence breached: returning");
    if (m != lastMode && autopilot)
    {
        char text[40];
        snprintf(text, sizeof(text), "Mode %s", AutopilotNames::mode(m));
        statusText(SEVERITY_INFO, text);
    }

    wasArmed = armed;
    wasFailsafe = failsafe;
    wasFence = fence;
    lastMode = m;
}

auto MavlinkTelemetry::wrap360(float degrees) -> float
{
    const float d = fmodf(degrees, 360.0f);
    return d < 0 ? d + 360.0f : d;
}

auto MavlinkTelemetry::centiDegrees(float degrees) -> uint16_t
{
    const long value = lroundf(wrap360(degrees) * 100.0f);
    return static_cast<uint16_t>(value >= 36000 ? value - 36000 : value);
}

auto MavlinkTelemetry::clamp16(float value) -> int16_t
{
    return static_cast<int16_t>(lroundf(constrain(value, -32767.0f, 32767.0f)));
}
