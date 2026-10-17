#pragma once
#include <Arduino.h>
#include <math.h>

#include "config/Config.h"

// ============================================================
// АВТОВЗЛЁТ С ПОЛОСЫ (MODE_AUTO_TAKEOFF)
//
// Этапы переключаются по тому, что делает самолёт, а не по таймеру.
// Тангаж — от горизонта наземной калибровки (control/GroundCalibration.h),
// стояночный угол — оттуда же.
//
//   IDLE ──газ выше TAKEOFF_TRIGGER_US──► CHECK (мотор стоит)
//   CHECK: самолёт неподвижен TAKEOFF_CHECK_MS, углы = стояночные
//          (± TAKEOFF_PARK_*_TOL_DEG); уточняется ноль гироскопа,
//          запоминаются курс и высота. Не так — ABORTED (мотор не
//          стартовал, сигнал пилоту).
//   Хвостовое колесо (TAIL_WHEEL):
//     TAIL_UP: газ плавно до TAKEOFF_THROTTLE_PCT, руль высоты от себя —
//          поток от винта поднимает хвост; по мере выравнивания отдача
//          уменьшается, ниже цели — на себя (не встать на винт).
//          ──тангаж у горизонта──► ROLL
//     ROLL: разбег на основных колёсах, хвост в воздухе, тангаж
//          TD_ROLL_PITCH_DEG ──скорость отрыва / TD_ROLL_MS──► LIFTOFF
//   Носовое колесо (NOSE_WHEEL):
//     ROLL: руль высоты TRI_ROLL_ELEVATOR_US, колесо прижато и рулит
//          ──скорость отрыва / TRI_ROLL_MS──► LIFTOFF
//   LIFTOFF: цель тангажа медленно растёт, но, пока отрыв не виден, —
//          не выше стояночного угла минус TD_TAIL_CLEARANCE_DEG (носовое:
//          TRI_MAX_GROUND_PITCH_DEG): хвост не коснётся полосы, отрывает
//          крыло ──отрыв по барометру, или предел + TAKEOFF_LIFTOFF_MS──► CLIMB
//   CLIMB: тангаж до TAKEOFF_CLIMB_PITCH_DEG, крылья ровно, курс рулём
//          направления ──TAKEOFF_SAFE_ALTITUDE_M (без барометра —
//          TAKEOFF_CLIMB_MS)──► DONE (дальше Autopilot держит курс и высоту)
//
// На земле (CHECK…LIFTOFF) — прерванный взлёт (ABORTED: мотор стоп, руль
// высоты в нейтраль, курс и крылья держатся, пока катится): газ стиком
// вниз, IMU отказал, крен, уход курса, нос ниже TD_MIN_PITCH_DEG, нет
// отрыва за TAKEOFF_GROUND_TIMEOUT_MS; потерю связи сообщает Autopilot
// (abort()). Снова — газ вниз на секунду и снова вверх. После отрыва газ
// стиком взлёт уже не прерывает: забрать управление — сменить режим.
//
// Класс только решает; углы, датчики и рули — у Autopilot (Inputs и
// геттеры ниже).
// ============================================================

class TakeoffController
{
public:

    enum class Stage : uint8_t { IDLE, CHECK, TAIL_UP, ROLL, LIFTOFF, CLIMB, DONE, ABORTED };

    // Всё, что взлёту нужно знать за такт. Углы — °, авиационные знаки
    // (как ImuData), скорости вращения — °/с.
    struct Inputs
    {
        bool armed = false;
        bool linkLost = false;
        uint16_t pilotThrottleUs = Config::PWM_MIN;
        bool imuOk = false;
        float pitchDeg = 0, rollDeg = 0, yawDeg = 0;
        float pitchRateDps = 0, rollRateDps = 0, yawRateDps = 0;
        float accelG = 1.0f;           // модуль перегрузки — для неподвижности
        bool parkingValid = false;     // калибровка шасси есть
        float parkingPitchDeg = 0, parkingRollDeg = 0;
        bool altitudeValid = false;    // барометр
        float altitudeM = 0;
        bool speedValid = false;       // трубка Пито или GPS
        float speedMs = 0;
        float gain = 1.0f;             // крутилка STAB_GAIN
        uint32_t nowMs = 0;
    };

    explicit TakeoffController(Config::LandingGear landingGear = Config::LANDING_GEAR)
        : gear(landingGear)
    {
    }

    void setGear(Config::LandingGear landingGear) { gear = landingGear; }
    Config::LandingGear getGear() const { return gear; }

    // Всё сначала (смена режима, DISARM): газ снова надо увидеть внизу.
    void reset()
    {
        stage = Stage::IDLE;
        triggerArmed = false;
        abortReason = nullptr;
        refusalPending = false;
        gyroTrimPending = false;
        lowThrottle = false;
        clearOutputs();
    }

    void update(const Inputs& in)
    {
        const float dt = haveLast ? min((in.nowMs - lastMs) / 1000.0f, 0.1f) : 0.0f;
        haveLast = true;
        lastMs = in.nowMs;
        now = in.nowMs;

        if (!in.armed)
        {
            // Газ внизу помним и без ARM: ARM всё равно только с газом
            // внизу, а ARM автопилот видит на такт позже.
            reset();
            triggerArmed = !in.linkLost && in.pilotThrottleUs < Config::THROTTLE_LOW_US;
            return;
        }
        // Без связи газ — значение failsafe, а не стик: взлёт не стартует и
        // не перезапускается, а после связи газ надо снова увидеть внизу.
        if (in.linkLost) triggerArmed = false;
        else if (in.pilotThrottleUs < Config::THROTTLE_LOW_US) triggerArmed = true;

        switch (stage)
        {
            case Stage::IDLE:
                if (!in.linkLost && triggerArmed && in.pilotThrottleUs >= Config::TAKEOFF_TRIGGER_US)
                {
                    enterCheck();
                }
                break;

            case Stage::CHECK:
                runCheck(in);
                break;

            case Stage::TAIL_UP:
                if (groundAbort(in)) break;
                if (in.pitchDeg <= Config::TD_ROLL_PITCH_DEG + Config::TD_TAIL_UP_DONE_DEG)
                {
                    enter(Stage::ROLL);
                    Serial.println("Автовзлёт: хвост поднят — разбег ровно");
                }
                else if (sinceStage() > Config::TD_TAIL_UP_TIMEOUT_MS)
                {
                    abort("хвост не поднялся", now);
                }
                break;

            case Stage::ROLL:
                if (groundAbort(in)) break;
                if (rotateReady(in))
                {
                    enter(Stage::LIFTOFF);
                    capReached = false;
                    Serial.println("Автовзлёт: скорость — отрыв");
                }
                break;

            case Stage::LIFTOFF:
                if (groundAbort(in)) break;
                rampPitch(dt);
                watchLiftoff(in);
                if (airborne || (!haveAltitude && capReached && now - capReachedMs >= Config::TAKEOFF_LIFTOFF_MS))
                {
                    enter(Stage::CLIMB);
                    Serial.println("Автовзлёт: набор высоты");
                }
                break;

            case Stage::CLIMB:
                rampPitch(dt);
                watchLiftoff(in);
                if (climbDone(in))
                {
                    enter(Stage::DONE);
                    Serial.println("Автовзлёт: набор окончен — держу курс и высоту");
                }
                break;

            case Stage::ABORTED:
                // Снова — газ вниз на секунду (и снова вверх).
                if (in.linkLost || in.pilotThrottleUs >= Config::THROTTLE_LOW_US)
                {
                    lowThrottle = false;
                }
                else if (!lowThrottle)
                {
                    lowThrottle = true;
                    lowThrottleSinceMs = now;
                }
                else if (now - lowThrottleSinceMs >= RESTART_LOW_THROTTLE_MS)
                {
                    enter(Stage::IDLE);
                }
                break;

            case Stage::DONE:
                break;
        }

        computeOutputs(in);
    }

    // Прервать взлёт извне (потеря связи). До старта мотора — просто ждать заново.
    void abort(const char* reason, uint32_t nowMs)
    {
        if (stage == Stage::IDLE || stage == Stage::DONE || stage == Stage::ABORTED) return;
        now = nowMs;
        const bool motorWasOn = stage != Stage::CHECK;
        stage = Stage::ABORTED;
        stageSinceMs = nowMs;
        abortReason = reason;
        lowThrottle = false;
        clearOutputs();
        Serial.print(motorWasOn ? "Автовзлёт ПРЕРВАН: " : "Автовзлёт: старт отменён — ");
        Serial.println(reason);
    }

    Stage getStage() const { return stage; }
    const char* getAbortReason() const { return abortReason; }
    bool isDone() const { return stage == Stage::DONE; }

    // Самолёт ещё на земле (или только что оторвался, но отрыв не виден):
    // потеря связи здесь — прерванный взлёт, а не failsafe; интеграторы
    // ПИД держатся на нуле — колёса держат угол, интеграл накопил бы
    // "ошибку", которой в полёте нет.
    bool holdsGround() const
    {
        return stage == Stage::IDLE || stage == Stage::CHECK || stage == Stage::TAIL_UP ||
               stage == Stage::ROLL || stage == Stage::LIFTOFF || stage == Stage::ABORTED;
    }

    // Набор или набор окончен — самолёт в воздухе.
    bool isAirborneStage() const { return stage == Stage::CLIMB || stage == Stage::DONE; }

    // --- что делать в этом такте ---

    // Газ программы, %. Стик газа взлёту — только "старт" и "прервать":
    // max(стик, программа) сразу дал бы мотору газ стика, без плавного разгона.
    float getThrottlePct() const { return throttlePct; }
    float getRollTargetDeg() const { return rollTarget; }
    float getPitchTargetDeg() const { return pitchTarget; }
    // На колёсах руль высоты — напрямую (законы хвоста/разбега).
    bool hasElevatorCommand() const { return elevatorCommand; }
    float getElevatorUs() const { return elevatorUs; }
    bool hasRudderCommand() const { return rudderCommand; }
    float getRudderUs() const { return rudderUs; }

    // Проверка перед стартом не пройдена — сыграть пилоту сигнал (один раз).
    bool takeRefusal()
    {
        const bool r = refusalPending;
        refusalPending = false;
        return r;
    }

    // Остаточный ноль гироскопа, замеренный на проверке (один раз).
    bool takeGyroTrim(float residualDps[3])
    {
        if (!gyroTrimPending) return false;
        gyroTrimPending = false;
        for (uint8_t i = 0; i < 3; ++i) residualDps[i] = gyroTrim[i];
        return true;
    }

    static const char* stageName(Stage s)
    {
        switch (s)
        {
            case Stage::IDLE:    return "IDLE";
            case Stage::CHECK:   return "CHECK";
            case Stage::TAIL_UP: return "TAIL_UP";
            case Stage::ROLL:    return "ROLL";
            case Stage::LIFTOFF: return "LIFTOFF";
            case Stage::CLIMB:   return "CLIMB";
            case Stage::DONE:    return "DONE";
            case Stage::ABORTED: return "ABORTED";
        }
        return "?";
    }

    // До 5 символов — для OLED.
    static const char* stageShort(Stage s)
    {
        switch (s)
        {
            case Stage::IDLE:    return "WAIT";
            case Stage::CHECK:   return "CHECK";
            case Stage::TAIL_UP: return "TAIL";
            case Stage::ROLL:    return "ROLL";
            case Stage::LIFTOFF: return "LIFT";
            case Stage::CLIMB:   return "CLIMB";
            case Stage::DONE:    return "DONE";
            case Stage::ABORTED: return "ABORT";
        }
        return "?";
    }


private:

    // Сколько держать газ внизу после прерванного взлёта, чтобы начать заново.
    static constexpr uint32_t RESTART_LOW_THROTTLE_MS = 1000;

    const char* abortReason = nullptr;
    uint32_t stageSinceMs = 0;
    uint32_t now = 0;
    uint32_t lastMs = 0;
    uint32_t lowThrottleSinceMs = 0;  // ABORTED: с какого момента газ внизу

    // Проверка перед стартом: средние за неподвижность.
    uint32_t checkCount = 0;
    uint32_t stillSinceMs = 0;
    float sumGyro[3] = {};
    float sumPitch = 0, sumRoll = 0;
    float gyroTrim[3] = {};

    // Что запомнено на старте.
    float headingDeg = 0;
    float startRollDeg = 0;
    float parkingPitch = 0;
    float startAltitudeM = 0;
    uint32_t motorStartMs = 0;
    uint32_t climbStartMs = 0;

    // Тангаж и отрыв.
    float pitchTarget = 0;
    uint32_t capReachedMs = 0;
    uint32_t liftoffSinceMs = 0;
    uint32_t noseDownSinceMs = 0;

    // Выходы такта.
    float throttlePct = 0;
    float rollTarget = 0;
    float elevatorUs = 0;
    float rudderUs = 0;

    // Однобайтовые поля — в конце, без лишнего выравнивания.
    Config::LandingGear gear;
    Stage stage = Stage::IDLE;
    bool haveLast = false;
    bool triggerArmed = false;        // газ видели внизу — подъём газа стартует взлёт
    bool refusalPending = false;
    bool lowThrottle = false;         // ABORTED: газ внизу (с lowThrottleSinceMs)
    bool gyroTrimPending = false;
    bool haveAltitude = false;
    bool capReached = false;
    bool airborne = false;            // отрыв виден по барометру
    bool liftoffPending = false;
    bool noseDown = false;
    bool elevatorCommand = false;
    bool rudderCommand = false;

    uint32_t sinceStage() const { return now - stageSinceMs; }

    void enter(Stage next)
    {
        stage = next;
        stageSinceMs = now;
        if (next == Stage::CLIMB) climbStartMs = now;
    }

    void clearOutputs()
    {
        throttlePct = 0;
        rollTarget = 0;
        elevatorCommand = false;
        elevatorUs = 0;
        rudderCommand = false;
        rudderUs = 0;
    }

    void enterCheck()
    {
        enter(Stage::CHECK);
        abortReason = nullptr;
        resetCheck();
        Serial.println("Автовзлёт: проверка — самолёт должен стоять неподвижно на колёсах");
    }

    void resetCheck()
    {
        checkCount = 0;
        sumGyro[0] = sumGyro[1] = sumGyro[2] = 0;
        sumPitch = sumRoll = 0;
    }

    void refuse(const char* reason)
    {
        abort(reason, now);
        refusalPending = true;
    }

    void runCheck(const Inputs& in)
    {
        if (in.linkLost) return;   // Autopilot уже прервал (abort())
        if (in.pilotThrottleUs < Config::THROTTLE_LOW_US)
        {
            enter(Stage::IDLE);
            Serial.println("Автовзлёт: газ убран до старта — жду");
            return;
        }
        if (!in.imuOk)
        {
            refuse("IMU не отвечает");
            return;
        }

        const float rate = max(max(fabsf(in.rollRateDps), fabsf(in.pitchRateDps)), fabsf(in.yawRateDps));
        const bool still = rate < Config::CAL_STILL_DPS && fabsf(in.accelG - 1.0f) < Config::CAL_STILL_G;
        if (!still)
        {
            resetCheck();
            if (sinceStage() > Config::TAKEOFF_CHECK_TIMEOUT_MS) refuse("самолёт не стоит неподвижно");
            return;
        }

        if (checkCount == 0) stillSinceMs = now;
        sumGyro[0] += in.rollRateDps;
        sumGyro[1] += in.pitchRateDps;
        sumGyro[2] += in.yawRateDps;
        sumPitch += in.pitchDeg;
        sumRoll += in.rollDeg;
        checkCount++;
        if (now - stillSinceMs < Config::TAKEOFF_CHECK_MS) return;

        const float pitch = sumPitch / checkCount;
        const float roll = sumRoll / checkCount;
        const char* problem = attitudeProblem(pitch, roll, in);
        if (problem)
        {
            Serial.print("Автовзлёт: стоит с тангажом ");
            Serial.print(pitch, 1);
            Serial.print("°, крен ");
            Serial.print(roll, 1);
            Serial.println("°");
            refuse(problem);
            return;
        }

        for (uint8_t i = 0; i < 3; ++i) gyroTrim[i] = sumGyro[i] / checkCount;
        gyroTrimPending = true;
        headingDeg = in.yawDeg;
        startRollDeg = roll;
        parkingPitch = in.parkingValid ? in.parkingPitchDeg : pitch;
        haveAltitude = in.altitudeValid;
        startAltitudeM = in.altitudeM;
        motorStartMs = now;
        airborne = false;
        liftoffPending = false;
        noseDown = false;
        capReached = false;

        if (gear == Config::LandingGear::TAIL_WHEEL)
        {
            pitchTarget = Config::TD_ROLL_PITCH_DEG;
            enter(Stage::TAIL_UP);
            Serial.println("Автовзлёт: проверка пройдена — мотор, хвост вверх");
        }
        else
        {
            pitchTarget = pitch;
            enter(Stage::ROLL);
            Serial.println("Автовзлёт: проверка пройдена — мотор, разбег");
        }
    }

    const char* attitudeProblem(float pitch, float roll, const Inputs& in) const
    {
        if (in.parkingValid)
        {
            if (fabsf(pitch - in.parkingPitchDeg) > Config::TAKEOFF_PARK_PITCH_TOL_DEG)
            {
                return "тангаж не стояночный — не на колёсах, склон или сломано колесо? (или плату сдвинули)";
            }
            if (fabsf(roll - in.parkingRollDeg) > Config::TAKEOFF_PARK_ROLL_TOL_DEG)
            {
                return "крен не стояночный — склон или сломано колесо?";
            }
            return nullptr;
        }
        if (gear == Config::LandingGear::TAIL_WHEEL)
        {
            return "нет калибровки шасси — с пульта без ARM или 'c' в консоли";
        }
        if (fabsf(pitch) > Config::TAKEOFF_UNCALIBRATED_TILT_DEG || fabsf(roll) > Config::TAKEOFF_UNCALIBRATED_TILT_DEG)
        {
            return "самолёт стоит не ровно";
        }
        return nullptr;
    }

    // Прерванный взлёт на земле. true — прерван.
    bool groundAbort(const Inputs& in)
    {
        if (in.linkLost) return false;   // это прерывает Autopilot (abort())
        if (in.pilotThrottleUs < Config::THROTTLE_LOW_US)
        {
            abort("газ убран", now);
            return true;
        }
        if (!in.imuOk)
        {
            abort("IMU не отвечает", now);
            return true;
        }
        if (fabsf(in.rollDeg - startRollDeg) > Config::TAKEOFF_ABORT_ROLL_DEG)
        {
            abort("крен на разбеге — крыло у земли", now);
            return true;
        }
        if (fabsf(headingError(in)) > Config::TAKEOFF_ABORT_HEADING_DEG)
        {
            abort("ушёл с курса", now);
            return true;
        }
        if (in.pitchDeg < Config::TD_MIN_PITCH_DEG)
        {
            if (!noseDown)
            {
                noseDown = true;
                noseDownSinceMs = now;
            }
            else if (now - noseDownSinceMs >= Config::TAKEOFF_NOSE_DOWN_MS)
            {
                abort("нос ниже горизонта — винт у земли", now);
                return true;
            }
        }
        else
        {
            noseDown = false;
        }
        if (now - motorStartMs > Config::TAKEOFF_GROUND_TIMEOUT_MS)
        {
            abort("нет отрыва", now);
            return true;
        }
        return false;
    }

    bool rotateReady(const Inputs& in) const
    {
        if (in.speedValid) return in.speedMs >= Config::TAKEOFF_ROTATE_SPEED_MS;
        const uint32_t rollMs = gear == Config::LandingGear::TAIL_WHEEL ? Config::TD_ROLL_MS : Config::TRI_ROLL_MS;
        return sinceStage() >= rollMs;
    }

    // Предел цели тангажа: набор, а пока отрыв не виден — ещё и предел у
    // земли (хвост не коснётся полосы).
    float pitchLimit() const
    {
        float limit = Config::TAKEOFF_CLIMB_PITCH_DEG;
        if (!airborne)
        {
            const float ground = gear == Config::LandingGear::TAIL_WHEEL
                                     ? parkingPitch - Config::TD_TAIL_CLEARANCE_DEG
                                     : Config::TRI_MAX_GROUND_PITCH_DEG;
            limit = min(limit, ground);
        }
        if (gear == Config::LandingGear::TAIL_WHEEL) limit = max(limit, Config::TD_ROLL_PITCH_DEG);
        return limit;
    }

    void rampPitch(float dt)
    {
        const float rate = gear == Config::LandingGear::TAIL_WHEEL ? Config::TD_ROTATE_RATE_DPS : Config::TRI_ROTATE_RATE_DPS;
        const float limit = pitchLimit();
        pitchTarget = min(limit, pitchTarget + rate * dt);
        if (!capReached && pitchTarget >= limit - 0.01f)
        {
            capReached = true;
            capReachedMs = now;
        }
    }

    void watchLiftoff(const Inputs& in)
    {
        if (airborne || !haveAltitude || !in.altitudeValid) return;
        if (in.altitudeM - startAltitudeM < Config::TAKEOFF_LIFTOFF_ALT_M)
        {
            liftoffPending = false;
            return;
        }
        if (!liftoffPending)
        {
            liftoffPending = true;
            liftoffSinceMs = now;
            return;
        }
        if (now - liftoffSinceMs >= Config::TAKEOFF_LIFTOFF_CONFIRM_MS)
        {
            airborne = true;
            Serial.println("Автовзлёт: отрыв (барометр) — предел тангажа у земли снят");
        }
    }

    bool climbDone(const Inputs& in) const
    {
        const uint32_t inClimb = now - climbStartMs;
        if (haveAltitude && in.altitudeValid)
        {
            // Высоту не набрал и за двойное время — всё равно дальше: CRUISE
            // держит ту, что есть.
            return in.altitudeM - startAltitudeM >= Config::TAKEOFF_SAFE_ALTITUDE_M ||
                   inClimb >= 2 * Config::TAKEOFF_CLIMB_MS;
        }
        return inClimb >= Config::TAKEOFF_CLIMB_MS;
    }

    float headingError(const Inputs& in) const
    {
        float error = headingDeg - in.yawDeg;
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;
        return error;
    }

    // Курс рулём направления и колесом: + — нос вправо.
    float headingLaw(const Inputs& in) const
    {
        const float rudder = in.gain * (Config::TAKEOFF_HEADING_KP * headingError(in) -
                                        Config::TAKEOFF_YAW_RATE_KD * in.yawRateDps);
        return constrain(rudder, -static_cast<float>(Config::RUDDER_MAX_US), static_cast<float>(Config::RUDDER_MAX_US));
    }

    // Руль высоты на колёсах: к цели тангажа, демпфирование по скорости
    // вращения; от себя — не больше TD_TAIL_UP_ELEVATOR_US.
    float pitchLaw(const Inputs& in, float kp, float target) const
    {
        const float elevator = in.gain * (kp * (target - in.pitchDeg) - Config::TD_PITCH_KD * in.pitchRateDps);
        return constrain(elevator, -static_cast<float>(Config::TD_TAIL_UP_ELEVATOR_US),
                         static_cast<float>(Config::ELEVATOR_MAX_US));
    }

    float rampedThrottle() const
    {
        const float k = min(static_cast<float>(now - motorStartMs) / Config::TAKEOFF_THROTTLE_RAMP_MS, 1.0f);
        return Config::TAKEOFF_THROTTLE_PCT * k;
    }

    void computeOutputs(const Inputs& in)
    {
        clearOutputs();
        switch (stage)
        {
            case Stage::IDLE:
            case Stage::CHECK:
            case Stage::DONE:
                break;

            case Stage::ABORTED:
                // Катится без мотора: крылья ровно, курс, руль высоты в нейтраль.
                rollTarget = startRollDeg;
                elevatorCommand = true;
                rudderCommand = true;
                rudderUs = headingLaw(in);
                break;

            case Stage::TAIL_UP:
                throttlePct = rampedThrottle();
                rollTarget = startRollDeg;
                elevatorCommand = true;
                elevatorUs = pitchLaw(in, Config::TD_PITCH_KP, Config::TD_ROLL_PITCH_DEG);
                rudderCommand = true;
                rudderUs = headingLaw(in);
                break;

            case Stage::ROLL:
                throttlePct = rampedThrottle();
                rollTarget = startRollDeg;
                elevatorCommand = true;
                elevatorUs = gear == Config::LandingGear::TAIL_WHEEL
                                 ? pitchLaw(in, Config::TD_ROLL_PITCH_KP, pitchTarget)
                                 : static_cast<float>(Config::TRI_ROLL_ELEVATOR_US);
                rudderCommand = true;
                rudderUs = headingLaw(in);
                break;

            case Stage::LIFTOFF:
                throttlePct = rampedThrottle();
                rollTarget = startRollDeg;
                elevatorCommand = true;
                elevatorUs = pitchLaw(in, Config::TD_ROLL_PITCH_KP, pitchTarget);
                rudderCommand = true;
                rudderUs = headingLaw(in);
                break;

            case Stage::CLIMB:
                // Тангаж — ПИД автопилота к pitchTarget; крылья ровно.
                throttlePct = rampedThrottle();
                rudderCommand = true;
                rudderUs = headingLaw(in);
                break;
        }
    }
};
