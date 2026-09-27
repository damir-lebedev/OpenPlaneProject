// Реализация autopilot/Autopilot.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/Autopilot.h"


Autopilot::Autopilot(ImuSensor* imu, BarometerSensor* baro,
                       MagnetometerSensor* mag, GpsSensor* gps,
                       AirspeedSensor* airspeed)
: imuSensor(imu),
      baroSensor(baro),
      magSensor(mag),
      gpsSensor(gps),
      airspeedSensor(airspeed)
{
    // Стартовая точка для полевой настройки (дашборд, POST
    // /api/setpid), не финальный тюнинг. Kp=5: 8° ошибки — 40 мкс,
    // 30° — 150 мкс; Kd умножается на угловую скорость (°/с).
    pidRoll.setGains(5.0f, 0.5f, 0.5f);
    pidPitch.setGains(5.0f, 0.5f, 0.5f);
    pidRoll.setLimits(-500, 500);
    pidPitch.setLimits(-500, 500);
}

auto Autopilot::begin() -> bool
{
    autoTrim.load();

    if (!imuSensor || !baroSensor)
    {
        Serial.println("Autopilot: IMU или барометр не подключены, стабилизация/высота недоступны");
        return false;
    }

    Serial.println("Autopilot: инициализирован");
    return true;
}

auto Autopilot::setInputs(const PilotInputs& pilotInputs) -> void
{
    inputs = pilotInputs;
}

auto Autopilot::update(bool isArmed, bool linkLost, uint16_t pilotThrottleUs,
                const ControlCommand& sticks) -> void
{
    const uint32_t nowUs = micros();
    dtS = lastUpdateUs == 0 ? 0.0f : min((nowUs - lastUpdateUs) / 1000000.0f, MAX_DT_S);
    lastUpdateUs = nowUs;
    nowMs = millis();

    const bool armedEdge = isArmed && !armed;
    const bool disarmedEdge = !isArmed && armed;
    armed = isArmed;
    pilot = sticks;
    pilotThrottle = pilotThrottleUs;

    if (imuSensor) imuSensor->update();
    if (baroSensor) baroSensor->update();
    if (magSensor) magSensor->update();
    if (gpsSensor) gpsSensor->update();
    if (airspeedSensor) airspeedSensor->update();

    updateNavigation(armedEdge);
    if (disarmedEdge) trimSavePending = true;
    if (trimSavePending && !armed && looksLanded())
    {
        autoTrim.saveIfChanged();
        trimSavePending = false;
    }

    setFailsafe(linkLost && armed);
    if (failsafe == Failsafe::NONE) checkGeofence();

    output = pilot;
    throttleMode = ThrottleMode::PILOT;
    autoThrottlePct = 0;
    nav.stallWarning = false;

    if (failsafe == Failsafe::RTH)        runReturnHome();
    else if (failsafe == Failsafe::GLIDE) runFailsafeGlide();
    else                                  runMode();

    applyTurnCoordination();
    applyAutoTrim();

    nav.targetCourseDeg = targetCourse;
    previousInputs = inputs;
}

auto Autopilot::applyThrottle(uint16_t pilotThrottleUs) const -> uint16_t
{
    switch (throttleMode)
    {
        case ThrottleMode::AUTO:
            return percentToUs(autoThrottlePct);
        case ThrottleMode::AT_LEAST:
            return max(pilotThrottleUs, percentToUs(autoThrottlePct));
        default:
            return pilotThrottleUs;
    }
}

auto Autopilot::setMode(AutopilotMode mode) -> void
{
    if (mode == currentMode || mode >= MODE_COUNT) return;

    Serial.print("Autopilot: режим ");
    Serial.print(AutopilotNames::mode(currentMode));
    Serial.print(" -> ");
    Serial.println(AutopilotNames::mode(mode));

    previousMode = currentMode;
    currentMode = mode;
    initializeMode();
}

auto Autopilot::getModeName() const -> const char*
{
    if (failsafe == Failsafe::GLIDE) return "FAILSAFE_GLIDE";
    if (failsafe == Failsafe::RTH) return "FAILSAFE_RTH";
    return AutopilotNames::mode(currentMode);
}

auto Autopilot::setPIDGains(float kpRoll, float kiRoll, float kdRoll,
                     float kpPitch, float kiPitch, float kdPitch) -> void
{
    pidRoll.setGains(kpRoll, kiRoll, kdRoll);
    pidPitch.setGains(kpPitch, kiPitch, kdPitch);
}

auto Autopilot::getAltitude() const -> float
{
    return baroReady() ? baroSensor->getBarometerData().altitude : 0.0f;
}

auto Autopilot::imuReady() const -> bool
{
    return imuSensor && imuSensor->isAvailable() && !imuSensor->getPreflightProblem();
}

auto Autopilot::baroReady() const -> bool
{
    return baroSensor && baroSensor->isAvailable();
}

auto Autopilot::airspeedReady() const -> bool
{
    return airspeedSensor && airspeedSensor->isAvailable();
}

auto Autopilot::looksLanded() const -> bool
{
    if (baroReady() && (fabsf(altitude()) > Config::AUTOTRIM_SAVE_MAX_ALT_M ||
                        fabsf(climbRate()) > Config::AUTOTRIM_SAVE_MAX_CLIMB_MS))
    {
        return false;
    }
    if (airspeedReady() && airspeedSensor->getAirspeedData().trueMs > Config::AUTOTRIM_SAVE_MAX_SPEED_MS)
    {
        return false;
    }
    return !(nav.gpsGood && gpsSensor->getGpsData().groundSpeed > Config::AUTOTRIM_SAVE_MAX_SPEED_MS);
}

auto Autopilot::climbRate() const -> float
{
    return baroReady() ? baroSensor->getBarometerData().verticalSpeed : 0.0f;
}

auto Autopilot::percentToUs(float percent) -> uint16_t
{
    const float clamped = constrain(percent, 0.0f, 100.0f);
    return static_cast<uint16_t>(Config::PWM_MIN + clamped * (Config::PWM_MAX - Config::PWM_MIN) / 100.0f);
}

auto Autopilot::stabGain() const -> float
{
    return inputs.knobValue(Knob::STAB_GAIN, Config::STAB_GAIN_MIN, 1.0f, Config::STAB_GAIN_MAX);
}

auto Autopilot::maxBank() const -> float
{
    return inputs.knobValue(Knob::MAX_BANK, Config::MAX_BANK_MIN_DEG, Config::MAX_BANK_DEG,
                            Config::MAX_BANK_MAX_DEG);
}

auto Autopilot::loiterRadius() const -> float
{
    return inputs.knobValue(Knob::LOITER_RADIUS, Config::LOITER_RADIUS_MIN_M, Config::LOITER_RADIUS_M,
                            Config::LOITER_RADIUS_MAX_M);
}

auto Autopilot::updateNavigation(bool armedEdge) -> void
{
    const GpsData* gps = gpsSensor && gpsSensor->isAvailable() ? &gpsSensor->getGpsData() : nullptr;
    nav.gpsGood = gps && gps->fixType >= 3 && gps->numSatellites >= Config::HOME_MIN_SATELLITES &&
                  gps->horizontalAccuracy <= Config::HOME_MAX_HACC_M;
    if (nav.gpsGood) nav.position = Geo::fromGps(*gps);

    if (nav.gpsGood && (armedEdge || (armed && !nav.homeValid)))
    {
        setHome("ARM");
    }

    if (inputs.has(Feature::HOME_RESET) && !previousInputs.has(Feature::HOME_RESET))
    {
        if (nav.gpsGood) setHome("тумблер HOME_RESET");
        else Serial.println("Autopilot: HOME_RESET — нет хорошего GPS, дом не изменён");
    }

    if (nav.homeValid && nav.gpsGood)
    {
        nav.distanceHomeM = Geo::distance(nav.position, nav.home);
        nav.bearingHomeDeg = Geo::bearing(nav.position, nav.home);
    }
    else
    {
        nav.distanceHomeM = -1;
    }

    // Курс: GPS (путевой) на скорости, иначе компас, иначе гироскоп.
    // У порога скорости — гистерезис: на GPS переходим выше порога,
    // уходим с него на 1 м/с ниже, иначе в сильный встречный ветер
    // источник "дребезжит" и цель курса сбрасывается каждый раз.
    CourseSource source = CourseSource::NONE;
    float course = nav.courseDeg;
    const float gpsCourseSpeed = courseSource == CourseSource::GPS
        ? Config::NAV_GPS_COURSE_MIN_SPEED_MS - 1.0f
        : Config::NAV_GPS_COURSE_MIN_SPEED_MS;
    if (nav.gpsGood && gps->groundSpeed >= gpsCourseSpeed)
    {
        source = CourseSource::GPS;
        course = gps->heading;
    }
    else if (magSensor && magSensor->isAvailable())
    {
        source = CourseSource::COMPASS;
        course = magSensor->getMagData().headingDegrees;
    }
    else if (imuReady())
    {
        source = CourseSource::GYRO;
        course = Geo::wrap360(imuSensor->getImuData().yaw);
    }
    nav.courseDeg = course;
    if (source != courseSource)
    {
        // Другой источник — другой ноль отсчёта: цель берём заново.
        courseSource = source;
        targetCourse = course;
    }

    // Скорость: трубка Пито, иначе GPS, иначе типичная.
    float speed = Config::NAV_ASSUMED_SPEED_MS;
    if (airspeedReady()) speed = airspeedSensor->getAirspeedData().trueMs;
    else if (nav.gpsGood) speed = gps->groundSpeed;
    nav.speedMs = speed;

    // Производная воздушной скорости — для вариометра полной энергии.
    if (airspeedReady() && dtS > 0)
    {
        const float v = airspeedSensor->getAirspeedData().trueMs;
        const float rate = (v - lastAirspeed) / dtS;
        airspeedRate += dtS / (1.0f + dtS) * (rate - airspeedRate);
        lastAirspeed = v;
    }
    else
    {
        airspeedRate = 0;
        if (airspeedReady()) lastAirspeed = airspeedSensor->getAirspeedData().trueMs;
    }
}

auto Autopilot::setHome(const char* why) -> void
{
    nav.home = nav.position;
    nav.homeValid = true;
    Serial.print("Autopilot: дом записан (");
    Serial.print(why);
    Serial.print(") ");
    Serial.print(nav.home.lat, 7);
    Serial.print(", ");
    Serial.println(nav.home.lon, 7);
}

auto Autopilot::checkGeofence() -> void
{
    const bool enabled = inputs.has(Feature::GEOFENCE) || Config::GEOFENCE_ALWAYS_ON;
    if (!enabled || !armed || !nav.homeValid || !nav.gpsGood)
    {
        nav.fenceBreached = false;
        return;
    }

    const bool outside = nav.distanceHomeM > Config::FENCE_RADIUS_M || altitude() > Config::FENCE_ALTITUDE_M;
    if (outside && !nav.fenceBreached)
    {
        nav.fenceBreached = true;
        Serial.println("Autopilot: геозабор — вылет за радиус/высоту, возврат домой");
        if (currentMode != MODE_RTH && currentMode != MODE_AUTO_LAND) setMode(MODE_RTH);
    }
    // Внутрь с запасом 10% — снова можно сработать.
    const bool wellInside = nav.distanceHomeM < Config::FENCE_RADIUS_M * 0.9f &&
                            altitude() < Config::FENCE_ALTITUDE_M * 0.9f;
    if (wellInside) nav.fenceBreached = false;
}

auto Autopilot::setFailsafe(bool active) -> void
{
    Failsafe next = Failsafe::NONE;
    if (active)
    {
        next = (Config::FAILSAFE_RTH && nav.gpsGood && nav.homeValid) ? Failsafe::RTH : Failsafe::GLIDE;
        // Уже возвращаемся — короткая потеря GPS не бросает в планирование.
        if (failsafe == Failsafe::RTH) next = Failsafe::RTH;
    }
    if (next == failsafe) return;

    const Failsafe previous = failsafe;
    failsafe = next;
    takeoffStarted = false;  // автовзлёт после восстановления связи — только заново
    launch.reset();
    pidRoll.reset();
    pidPitch.reset();

    if (failsafe == Failsafe::GLIDE)
    {
        Serial.println("Autopilot: связь потеряна — планирование (мотор выключен, крылья ровно)");
    }
    else if (failsafe == Failsafe::RTH)
    {
        Serial.println("Autopilot: связь потеряна — возврат домой");
        beginReturnHome();
    }
    else if (previous != Failsafe::NONE)
    {
        Serial.print("Autopilot: связь восстановлена — снова режим ");
        Serial.println(AutopilotNames::mode(currentMode));
        initializeMode();
    }
}

auto Autopilot::runFailsafeGlide() -> void
{
    desiredRoll = Config::FAILSAFE_GLIDE_ROLL_DEG;
    desiredPitch = Config::FAILSAFE_GLIDE_PITCH_DEG;
    output = ControlCommand();
    stabilizeOrNeutral();
    throttleMode = ThrottleMode::AUTO;
    autoThrottlePct = 0;
}

auto Autopilot::initializeMode() -> void
{
    output = pilot;   // коррекции прошлого режима больше не действуют
    pidRoll.reset();
    pidPitch.reset();
    altitudeControl.reset();
    launch.reset();
    soaring.reset(millis());

    desiredRoll = 0;
    desiredPitch = 0;
    takeoffStarted = false;
    launchCaptured = false;
    rthArrived = false;
    targetCourse = nav.courseDeg;
    targetAltitude = altitude();

    loiterHasCenter = nav.gpsGood;
    if (loiterHasCenter) loiterCenter = nav.position;

    if (currentMode == MODE_RTH) beginReturnHome();
}

auto Autopilot::beginReturnHome() -> void
{
    rthArrived = false;
    targetAltitude = max(altitude(), Config::RTH_ALTITUDE_M);
    altitudeControl.reset();
}

auto Autopilot::runMode() -> void
{
    switch (currentMode)
    {
        case MODE_MANUAL:       break;   // output = стики
        case MODE_STABILIZE:    runStabilize(); break;
        case MODE_ACRO:         runAcro(); break;
        case MODE_ALT_HOLD:     runAltHold(); break;
        case MODE_CRUISE:       runCruise(); break;
        case MODE_LOITER:       runLoiter(); break;
        case MODE_RTH:          runReturnHome(); break;
        case MODE_AUTO_TAKEOFF: runAutoTakeoff(); break;
        case MODE_LAUNCH:       runLaunch(); break;
        case MODE_AUTO_LAND:    runAutoLand(); break;
        case MODE_SOARING:      runSoaring(); break;
        case MODE_RESCUE:       runRescue(); break;
        default:                break;
    }
}

auto Autopilot::stickRollAngle() const -> float
{
    return static_cast<float>(pilot.roll) / Config::AILERON_MAX_US * maxBank();
}

auto Autopilot::stickPitchAngle() const -> float
{
    return static_cast<float>(pilot.pitch) / Config::ELEVATOR_MAX_US * Config::STAB_MAX_PITCH_DEG;
}

auto Autopilot::stickActive(int16_t value) -> bool
{
    return value > STICK_DEADBAND_US || value < -STICK_DEADBAND_US;
}

auto Autopilot::runStabilize() -> void
{
    desiredRoll = stickRollAngle();
    desiredPitch = stickPitchAngle();
    stabilizeOrManual();
}

auto Autopilot::runAcro() -> void
{
    if (!imuReady()) return;   // рули = стики

    const ImuData& imu = imuSensor->getImuData();
    const float wantRollRate = static_cast<float>(pilot.roll) / Config::AILERON_MAX_US * Config::ACRO_MAX_RATE_DPS;
    const float wantPitchRate = static_cast<float>(pilot.pitch) / Config::ELEVATOR_MAX_US * Config::ACRO_MAX_RATE_DPS;
    const float gain = Config::ACRO_RATE_GAIN_US_PER_DPS * stabGain();

    output.roll = clampCommand(pilot.roll + gain * (wantRollRate - imu.gyroX));
    output.pitch = clampCommand(pilot.pitch + gain * (wantPitchRate - imu.gyroY));
    desiredRoll = imu.roll;
    desiredPitch = imu.pitch;
}

auto Autopilot::holdAltitudeWithStickOverride() -> void
{
    if (stickActive(pilot.pitch) || !baroReady())
    {
        desiredPitch = stickPitchAngle();
        targetAltitude = altitude();
        altitudeControl.reset();
        return;
    }
    desiredPitch = altitudeControl.pitchFor(targetAltitude, altitude(), climbRate(), nav.speedMs, dtS, armed);
}

auto Autopilot::holdCourseWithStickOverride() -> void
{
    if (stickActive(pilot.roll) || courseSource == CourseSource::NONE)
    {
        desiredRoll = stickRollAngle();
        targetCourse = nav.courseDeg;
        return;
    }
    desiredRoll = Guidance::rollForCourse(targetCourse, nav.courseDeg, navBankLimit());
}

auto Autopilot::runAltHold() -> void
{
    desiredRoll = stickRollAngle();
    holdAltitudeWithStickOverride();
    stabilizeOrManual();
}

auto Autopilot::runCruise() -> void
{
    holdCourseWithStickOverride();
    holdAltitudeWithStickOverride();
    autoThrottle();
    protectFromStall();
    stabilizeOrManual();
}

auto Autopilot::orbit(const GeoPoint& center, bool haveCenter) -> void
{
    const float radius = loiterRadius();
    const float bankLimit = navBankLimit();
    const float feedForward = min(Guidance::orbitBankDeg(nav.speedMs, radius), bankLimit);

    if (!haveCenter || !nav.gpsGood)
    {
        desiredRoll = feedForward;
        targetCourse = nav.courseDeg;
        return;
    }

    const float distance = Geo::distance(center, nav.position);
    const float fromCenter = Geo::bearing(center, nav.position);
    targetCourse = Guidance::orbitCourse(fromCenter, distance, radius, true);

    // Упреждающий крен круга — у окружности; издалека — только курс.
    const float nearCircle = constrain(1.0f - fabsf(distance - radius) / radius, 0.0f, 1.0f);
    desiredRoll = constrain(Guidance::rollForCourse(targetCourse, nav.courseDeg, bankLimit) +
                                feedForward * nearCircle,
                            -bankLimit, bankLimit);
}

auto Autopilot::runLoiter() -> void
{
    orbit(loiterCenter, loiterHasCenter);
    holdAltitudeWithStickOverride();
    autoThrottle();
    protectFromStall();
    stabilizeOrNeutral();
}

auto Autopilot::runReturnHome() -> void
{
    if (nav.gpsGood && nav.homeValid && !rthArrived &&
        nav.distanceHomeM > loiterRadius() * 1.5f)
    {
        targetCourse = nav.bearingHomeDeg;
        desiredRoll = Guidance::rollForCourse(targetCourse, nav.courseDeg, navBankLimit());
    }
    else
    {
        if (!rthArrived && nav.homeValid && nav.gpsGood)
        {
            rthArrived = true;
            Serial.println("Autopilot: RTH — над домом, круги");
        }
        orbit(nav.home, nav.homeValid);
    }

    if (baroReady())
    {
        desiredPitch = altitudeControl.pitchFor(targetAltitude, altitude(), climbRate(), nav.speedMs, dtS, armed);
    }
    else
    {
        desiredPitch = 0;
    }

    autoThrottle();
    protectFromStall();
    stabilizeOrNeutral();
}

auto Autopilot::runAutoTakeoff() -> void
{
    if (!armed)
    {
        takeoffStarted = false;
    }
    else if (!takeoffStarted && pilotThrottle >= TAKEOFF_TRIGGER_US)
    {
        takeoffStarted = true;
        takeoffStartMs = nowMs;
        Serial.println("Autopilot: автовзлёт — старт");
    }

    desiredRoll = 0;
    if (!takeoffStarted)
    {
        desiredPitch = 0;
        autoThrottlePct = 0;
    }
    else
    {
        const uint32_t elapsed = nowMs - takeoffStartMs;
        autoThrottlePct = elapsed < TAKEOFF_THROTTLE_RAMP_MS ? 100.0f * elapsed / TAKEOFF_THROTTLE_RAMP_MS : 100.0f;

        if (elapsed < 1000)      desiredPitch = 0;
        else if (elapsed < 3000) desiredPitch = 15;
        else                     desiredPitch = 10;
    }
    throttleMode = ThrottleMode::AT_LEAST;

    // Стики пилота — поверх программы (как раньше).
    if (imuReady())
    {
        stabilize();
        output.roll = clampCommand(output.roll + pilot.roll);
        output.pitch = clampCommand(output.pitch + pilot.pitch);
    }
}

auto Autopilot::runLaunch() -> void
{
    const bool sticksMoved = abs(pilot.roll) > LAUNCH_ABORT_STICK_US || abs(pilot.pitch) > LAUNCH_ABORT_STICK_US;
    const float forwardG = imuSensor ? imuSensor->getImuData().accelX : 0.0f;
    launch.update(armed, pilotThrottle >= TAKEOFF_TRIGGER_US, forwardG, sticksMoved, altitude(), nowMs);

    if (launch.getState() == LaunchController::State::DONE)
    {
        if (!launchCaptured)
        {
            launchCaptured = true;
            targetCourse = nav.courseDeg;
            targetAltitude = altitude();
            altitudeControl.reset();
        }
        runCruise();
        return;
    }

    desiredRoll = 0;
    desiredPitch = launch.pitchTargetDeg();
    throttleMode = ThrottleMode::AUTO;
    autoThrottlePct = launch.motorOn() ? Config::LAUNCH_THROTTLE_PCT : 0.0f;
    stabilizeOrNeutral();
}

auto Autopilot::runAutoLand() -> void
{
    holdCourseWithStickOverride();
    desiredPitch = (baroReady() && altitude() <= Config::LAND_FLARE_ALTITUDE_M)
        ? Config::LAND_FLARE_PITCH_DEG
        : Config::LAND_GLIDE_PITCH_DEG;
    throttleMode = ThrottleMode::AUTO;
    autoThrottlePct = 0;
    stabilizeOrNeutral();
}

auto Autopilot::runSoaring() -> void
{
    // Вариометр полной энергии: набор + V·dV/dt / g.
    float climb = climbRate();
    if (airspeedReady()) climb += nav.speedMs * airspeedRate / Geo::GRAVITY;

    soaring.update(climb, altitude(), nav.distanceHomeM, dtS, nowMs);

    throttleMode = ThrottleMode::AUTO;
    autoThrottlePct = 0;

    switch (soaring.getState())
    {
        case SoaringController::State::THERMAL:
            desiredRoll = min(Config::SOAR_CIRCLE_BANK_DEG, navBankLimit());
            desiredPitch = Config::SOAR_CIRCLE_PITCH_DEG;
            targetCourse = nav.courseDeg;
            break;

        case SoaringController::State::MOTOR_CLIMB:
            steerHomeIfFarOrHoldCourse();
            targetAltitude = Config::SOAR_MAX_ALTITUDE_M;
            desiredPitch = baroReady()
                ? altitudeControl.pitchFor(targetAltitude, altitude(), climbRate(), nav.speedMs, dtS, armed)
                : Config::LAUNCH_CLIMB_PITCH_DEG;
            autoThrottle();
            break;

        case SoaringController::State::RETURN:
            targetCourse = nav.bearingHomeDeg;
            desiredRoll = Guidance::rollForCourse(targetCourse, nav.courseDeg, navBankLimit());
            desiredPitch = Config::SOAR_GLIDE_PITCH_DEG;
            break;

        default:
            holdCourseWithStickOverride();
            desiredPitch = Config::SOAR_GLIDE_PITCH_DEG;
            break;
    }

    protectFromStall();
    stabilizeOrNeutral();
}

auto Autopilot::steerHomeIfFarOrHoldCourse() -> void
{
    if (nav.distanceHomeM > Config::SOAR_MAX_DISTANCE_M)
    {
        targetCourse = nav.bearingHomeDeg;
        desiredRoll = Guidance::rollForCourse(targetCourse, nav.courseDeg, navBankLimit());
    }
    else
    {
        holdCourseWithStickOverride();
    }
}

auto Autopilot::runRescue() -> void
{
    desiredRoll = 0;
    desiredPitch = Config::RESCUE_PITCH_DEG;
    throttleMode = ThrottleMode::AUTO;
    autoThrottlePct = Config::RESCUE_THROTTLE_PCT;
    stabilizeOrNeutral();
}

auto Autopilot::autoThrottle() -> void
{
    const float cruisePct = inputs.knobValue(Knob::CRUISE_SPEED, Config::CRUISE_THROTTLE_MIN_PCT,
                                             Config::CRUISE_THROTTLE_PCT, Config::CRUISE_THROTTLE_MAX_PCT);
    const float targetAirspeed = inputs.knobValue(Knob::CRUISE_SPEED, Config::CRUISE_AIRSPEED_MIN_MS,
                                                  Config::CRUISE_AIRSPEED_MS, Config::CRUISE_AIRSPEED_MAX_MS);
    const float airspeed = airspeedReady() ? airspeedSensor->getAirspeedData().indicatedMs : 0.0f;

    throttleMode = ThrottleMode::AUTO;
    autoThrottlePct = altitudeControl.throttleFor(airspeedReady(), airspeed, targetAirspeed, cruisePct,
                                                  dtS, armed);
}

auto Autopilot::protectFromStall() -> void
{
    if (!airspeedReady() || !armed) return;

    const float ias = airspeedSensor->getAirspeedData().indicatedMs;
    if (ias >= Config::STALL_SPEED_MS + Config::STALL_MARGIN_MS) return;

    nav.stallWarning = true;
    desiredPitch = min(desiredPitch, 0.0f);
    desiredRoll = constrain(desiredRoll, -Config::STALL_BANK_LIMIT_DEG, Config::STALL_BANK_LIMIT_DEG);
    if (throttleMode == ThrottleMode::AUTO && autoThrottlePct > 0) autoThrottlePct = Config::AUTO_THROTTLE_MAX_PCT;
}

auto Autopilot::stabilize() -> void
{
    const ImuData& imu = imuSensor->getImuData();
    const float gain = stabGain();
    const bool integrate = armed && !waitingForStart();
    // Интегратор — только у цели (I-зона): при выходе из большого
    // крена он иначе накопил бы поправку, которая потом секундами
    // тянет самолёт за горизонт.
    const bool nearRoll = fabsf(desiredRoll - imu.roll) < Config::STAB_INTEGRATOR_ZONE_DEG;
    const bool nearPitch = fabsf(desiredPitch - imu.pitch) < Config::STAB_INTEGRATOR_ZONE_DEG;
    output.roll = clampCommand(gain * pidRoll.calculate(desiredRoll, imu.roll, imu.gyroX, integrate && nearRoll));
    output.pitch = clampCommand(gain * pidPitch.calculate(desiredPitch, imu.pitch, imu.gyroY, integrate && nearPitch));
}

auto Autopilot::waitingForStart() const -> bool
{
    if (failsafe != Failsafe::NONE) return false;
    if (currentMode == MODE_AUTO_TAKEOFF) return !takeoffStarted;
    if (currentMode == MODE_LAUNCH)
    {
        const LaunchController::State s = launch.getState();
        return s == LaunchController::State::IDLE || s == LaunchController::State::READY ||
               s == LaunchController::State::THROWN;
    }
    return false;
}

auto Autopilot::stabilizeOrManual() -> void
{
    if (imuReady()) stabilize();
}

auto Autopilot::stabilizeOrNeutral() -> void
{
    if (imuReady())
    {
        stabilize();
        return;
    }
    output.roll = 0;
    output.pitch = 0;
}

auto Autopilot::isNavigationMode(AutopilotMode mode) -> bool
{
    return mode == MODE_CRUISE || mode == MODE_LOITER || mode == MODE_RTH || mode == MODE_LAUNCH ||
           mode == MODE_AUTO_LAND || mode == MODE_SOARING || mode == MODE_RESCUE;
}

auto Autopilot::applyTurnCoordination() -> void
{
    const bool navigating = failsafe == Failsafe::RTH || isNavigationMode(currentMode);
    const bool enabled = inputs.has(Feature::TURN_COORDINATION) ||
                         (Config::TURN_COORD_IN_NAV_MODES && navigating && failsafe != Failsafe::GLIDE);
    if (!enabled) return;

    output.yaw = clampCommand(output.yaw + Config::TURN_COORD_RUDDER_MIX * output.roll);

    if (imuReady())
    {
        const float bankRad = constrain(fabsf(imuSensor->getImuData().roll), 0.0f, 60.0f) * static_cast<float>(Geo::DEG);
        const float lift = 1.0f / cosf(bankRad) - 1.0f;
        output.pitch = clampCommand(output.pitch + Config::TURN_COORD_PITCH_US * lift);
    }
}

auto Autopilot::applyAutoTrim() -> void
{
    const bool pilotedMode = currentMode == MODE_MANUAL || currentMode == MODE_STABILIZE ||
                             currentMode == MODE_ACRO || currentMode == MODE_ALT_HOLD;
    bool levelFlight = false;
    if (imuReady() && armed && pilotedMode && failsafe == Failsafe::NONE)
    {
        const ImuData& imu = imuSensor->getImuData();
        levelFlight = fabsf(imu.roll) < Config::AUTOTRIM_MAX_ROLL_DEG &&
                      fabsf(imu.gyroX) < Config::AUTOTRIM_MAX_RATE_DPS &&
                      fabsf(imu.gyroY) < Config::AUTOTRIM_MAX_RATE_DPS;
    }
    autoTrim.update(inputs.has(Feature::AUTO_TRIM), levelFlight, output.roll, output.pitch, dtS);

    output.roll = clampCommand(output.roll + autoTrim.getRoll());
    output.pitch = clampCommand(output.pitch + autoTrim.getPitch());
}

auto Autopilot::clampCommand(float value) -> int16_t
{
    return static_cast<int16_t>(constrain(value, -500.0f, 500.0f));
}
