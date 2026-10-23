#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/AltitudeHold.h"
#include "copter/Attitude.h"
#include "copter/GyroFilter.h"
#include "copter/Mixer.h"
#include "copter/Presets.h"
#include "copter/RateController.h"

// ============================================================
// Мультикоптер целиком: датчики и ручки пульта -> команды моторам.
//
// Без железа и без Arduino: на входе — показания гироскопа,
// акселерометра, барометра, напряжение и пульт, на выходе — команды
// регуляторам 0..1. Один и тот же код работает на плате (цикл 1 кГц,
// src/stm32/main_copter.cpp) и в тестовом симуляторе.
//
// Цепочка за один цикл:
//   гироскоп -> режекторы по оборотам -> ФНЧ -> контур угловой скорости
//   (RateController.h) -> смеситель (Mixer.h) -> линеаризация тяги,
//   поправка на просадку батареи, холостые -> регуляторы.
//
// Режимы (уставка контуру угловой скорости):
//   ACRO     — стик задаёт угловую скорость (кривая расходов);
//   HORIZON  — у центра стика — как ANGLE, у края — как ACRO (флип);
//   ANGLE    — стик задаёт наклон, отпустил — коптер выравнивается;
//   ALT_HOLD — ANGLE + высота: газ в центре — висит, выше/ниже —
//              набор/снижение.
//
// Защиты:
//   • ARM — только когда всё в порядке (armBlockers()): связь есть,
//     газ внизу, коптер стоит ровно и не качается, гироскоп жив,
//     батарея не пустая, проверка моторов выключена; ARM, включённый
//     раньше времени, надо выключить и включить заново;
//   • выключение ARM — моторы стоп в любой момент (аварийная кнопка);
//   • потеря связи: на земле — сразу DISARM; в воздухе — ровно и
//     снижение 0.7 м/с (с барометром — по нему, без — по
//     акселерометру), касание земли -> DISARM. Связь вернулась на 1 с —
//     управление снова у пилота;
//   • авария: в ANGLE/ALT_HOLD наклон больше 90° дольше 0.7 с или не тот,
//     что просит стик, больше 30° дольше 2 с (отказал мотор, лежит на
//     боку); в любом режиме лежит вверх ногами 1 с — моторы стоп (а не
//     молотят винтами по земле), повторный ARM — после выключения тумблера.
// ============================================================

namespace copter
{
    enum class CopterMode : uint8_t
    {
        ACRO,
        HORIZON,
        ANGLE,
        ALT_HOLD,
    };

    inline const char* copterModeName(CopterMode m)
    {
        switch (m)
        {
            case CopterMode::ACRO: return "ACRO";
            case CopterMode::HORIZON: return "HORIZON";
            case CopterMode::ANGLE: return "ANGLE";
            case CopterMode::ALT_HOLD: return "ALT_HOLD";
        }
        return "?";
    }

    // Почему не армится (битовая маска).
    enum ArmBlock : uint16_t
    {
        ARM_BLOCK_NO_LINK = 1 << 0,
        ARM_BLOCK_THROTTLE = 1 << 1,
        ARM_BLOCK_TILT = 1 << 2,
        ARM_BLOCK_SWITCH = 1 << 3,
        ARM_BLOCK_IMU = 1 << 4,
        ARM_BLOCK_SETTLING = 1 << 5,
        ARM_BLOCK_MOVING = 1 << 6,
        ARM_BLOCK_BATTERY = 1 << 7,
        ARM_BLOCK_MOTOR_TEST = 1 << 8,
        ARM_BLOCK_CRASH = 1 << 9,
        ARM_BLOCK_EXTERNAL = 1 << 10,   // снаружи: калибровка установки, настройки
    };

    inline const char* armBlockText(uint16_t bit)
    {
        switch (bit)
        {
            case ARM_BLOCK_NO_LINK: return "нет связи с пультом";
            case ARM_BLOCK_THROTTLE: return "газ не внизу";
            case ARM_BLOCK_TILT: return "коптер стоит не ровно (больше 25°)";
            case ARM_BLOCK_SWITCH: return "ARM был включён заранее — выключить и включить снова";
            case ARM_BLOCK_IMU: return "гироскоп/акселерометр не отвечает";
            case ARM_BLOCK_SETTLING: return "ориентация ещё не определилась (2 с после включения)";
            case ARM_BLOCK_MOVING: return "коптер качают — поставить на землю";
            case ARM_BLOCK_BATTERY: return "батарея разряжена";
            case ARM_BLOCK_MOTOR_TEST: return "идёт проверка моторов";
            case ARM_BLOCK_CRASH: return "был переворот — выключить ARM";
            case ARM_BLOCK_EXTERNAL: return "нужна калибровка или настройка (см. консоль)";
        }
        return "?";
    }

    enum class CopterState : uint8_t
    {
        DISARMED,
        ARMED,       // пилот управляет
        FAILSAFE,    // нет связи: выравнивание и снижение
    };

    struct CopterSensors
    {
        float gyro[3] = {};       // °/с: крен, тангаж, рысканье (авиационные знаки)
        float accel[3] = {};      // g: X к носу, Y влево, Z вверх
        bool imuOk = true;
        float baroAltM = 0.0f;
        bool baroOk = false;
        float batteryVolts = 0.0f;   // 0 — не измеряется
    };

    struct CopterRc
    {
        bool linkOk = false;
        float throttle = 0.0f;   // 0..1
        float roll = 0.0f;       // −1..1, + вправо
        float pitch = 0.0f;      // −1..1, + на себя (нос вверх)
        float yaw = 0.0f;        // −1..1, + нос вправо
        bool armSwitch = false;
        CopterMode mode = CopterMode::ANGLE;
    };

    struct CopterOutputs
    {
        float motor[MAX_MOTORS] = {};   // команда регулятору 0..1 (0 — стоп)
        uint8_t count = 0;
        bool spinning = false;          // false — все моторы стоят
    };

    struct CopterSettings
    {
        bool airmode = true;
        bool voltageCompensation = true;
        bool rpmNotches = true;
        float rcSmoothingHz = 40.0f;
        float horizonTransition = 0.75f;   // HORIZON: с какого отклонения стика — чистое ACRO
        float failsafeDescent = 0.7f;      // м/с
        float failsafeRecoveryS = 1.0f;    // связь держится столько — управление пилоту
        float failsafeGuardS = 0.3f;       // столько связи нет — failsafe
        float failsafeMaxS = 120.0f;       // не села за это время — DISARM
        float minCellVolts = 3.3f;         // ниже (без нагрузки) — не армить
        bool crashDetection = true;
        AltitudeController::Settings alt;
    };

    struct CopterStatus
    {
        CopterState state = CopterState::DISARMED;
        CopterMode mode = CopterMode::ANGLE;
        uint16_t armBlockers = 0;
        float roll = 0, pitch = 0, yaw = 0;     // °
        float rates[3] = {};                    // °/с после фильтров
        float setpoint[3] = {};                 // °/с
        float altitude = 0, climbRate = 0;      // м, м/с (от точки ARM)
        float altTarget = 0;
        float throttle = 0;                     // доля тяги на входе смесителя
        float hoverLearned = 0;
        float volts = 0;
        float rotationHz = 0;
        bool airborne = false;
        bool saturated = false;
        bool crashed = false;
        uint32_t armCount = 0;
        uint32_t loops = 0;
    };

    class CopterController
    {
    public:
        void configure(const CopterTune& t, const CopterSettings& s = CopterSettings())
        {
            tune = t;
            set = s;
            dt = 1.0f / t.loopHz;
            mixer.setAirmode(s.airmode);
            gyroFilter.configure(t.loopHz, t.gyroLpfHz, t.notchQ, t.prop.prop.blades, t.minNotchHz);
            axes[0].configure(t.roll, t.dtermLpfHz, 0.0f, t.iLimit, dt);
            axes[1].configure(t.pitch, t.dtermLpfHz, 0.0f, t.iLimit, dt);
            axes[2].configure(t.yaw, t.dtermLpfHz, t.yawLpfHz, t.iLimit, dt);
            for (auto& ax : axes) ax.setTpaMode(t.tpaMode);
            for (int a = 0; a < 4; ++a) stickLpf[a].configure(s.rcSmoothingHz, dt);
            throttleHpLpf.configure(15.0f, dt);
            for (auto& f : accelLpf) f.configure(20.0f, dt);
            motionLpf.configure(3.0f, dt);
            voltLpf.configure(2.0f, dt);
            attitude.configure(2.0f, 0.2f);
            attitude.reset();
            vertical.configure(2.5f);
            altCtl.configure(s.alt, t.hoverThrust);
            hover = t.hoverThrust;
            disarm();
            status = CopterStatus();
            uptime = 0.0f;
            motionTimer = 0.0f;
        }

        // Другой профиль PID и расходов — на ходу, и в полёте: интегралы,
        // фильтры, ориентация, ARM не сбрасываются. Фильтры, TPA и всё
        // остальное берутся из configure().
        void applyProfile(const CopterTune& t)
        {
            tune.roll = t.roll;
            tune.pitch = t.pitch;
            tune.yaw = t.yaw;
            tune.gainsFromModel = t.gainsFromModel;
            for (int a = 0; a < 3; ++a) tune.rates[a] = t.rates[a];
            axes[0].setGains(t.roll);
            axes[1].setGains(t.pitch);
            axes[2].setGains(t.yaw);
        }

        // Внешняя причина не армить (калибровка установки платы, ошибка настроек).
        void setExternalBlock(bool blocked) { externalBlock = blocked; }

        // Проверка моторов (только DISARM): мотор index, доля тяги, сколько секунд.
        bool startMotorTest(uint8_t index, float thrust, float seconds)
        {
            if (status.state != CopterState::DISARMED || index >= tune.frame.count) return false;
            testMotor = index;
            testThrust = fminf(0.3f, fmaxf(0.0f, thrust));
            testLeft = fminf(5.0f, fmaxf(0.0f, seconds));
            return true;
        }

        void stopMotorTest() { testLeft = 0.0f; }
        bool motorTestActive() const { return testLeft > 0.0f; }

        // Один цикл стабилизации.
        CopterOutputs update(const CopterSensors& sens, const CopterRc& rcIn)
        {
            CopterOutputs out;
            out.count = tune.frame.count;
            uptime += dt;
            ++status.loops;

            // ---- датчики ----
            float rates[3] = { sens.gyro[0], sens.gyro[1], sens.gyro[2] };
            if (sens.batteryVolts > 1.0f)
            {
                if (status.volts <= 1.0f) voltLpf.reset(sens.batteryVolts);
                status.volts = voltLpf.apply(sens.batteryVolts);
            }
            // обороты не меняются быстрее, чем мотор успевает разогнать винт:
            // оценка идёт за командой с постоянной времени мотора — режекторы
            // едут плавно, а не прыгают вслед за каждым циклом смесителя
            float rotHz = 0.0f;
            if (spinning())
            {
                const float target = tune.rotationHz(avgThrust, status.volts);
                rotEstimate += (target - rotEstimate) * dt / (fmaxf(0.005f, tune.motorTauS) + dt);
                rotHz = rotEstimate;
            }
            else
            {
                rotEstimate = tune.rotHzIdle;
            }
            status.rotationHz = rotHz;
            if (set.rpmNotches) gyroFilter.setRotationHz(rotHz);
            gyroFilter.apply(rates);
            // акселерометр на коптере трясётся на 1–2 g от моторов: для
            // горизонта и высоты нужно только медленное
            float accel[3];
            for (int a = 0; a < 3; ++a) accel[a] = accelLpf[a].apply(sens.accel[a]);
            if (!attitude.isInitialized())
            {
                for (int a = 0; a < 3; ++a)
                {
                    accelLpf[a].reset(sens.accel[a]);
                    accel[a] = sens.accel[a];
                }
            }
            for (int a = 0; a < 3; ++a) accelG[a] = accel[a];
            attitude.setGrounded(!status.airborne);
            if (sens.imuOk) attitude.update(sens.gyro, accel, dt);
            const float accelUp = attitude.isInitialized() ? attitude.verticalAccel(accel) : 0.0f;
            // без барометра смещение акселерометра учится, пока коптер стоит
            // на земле до ARM: ускорение тогда точно ноль
            vertical.setBiasLearning(status.state == CopterState::DISARMED && motionTimer > 0.5f);
            vertical.update(accelUp, sens.baroAltM - baroHome, sens.baroOk, dt);
            status.roll = attitude.rollDeg();
            status.pitch = attitude.pitchDeg();
            status.yaw = attitude.yawDeg();
            status.altitude = vertical.altitude();
            status.climbRate = vertical.climbRate();
            for (int a = 0; a < 3; ++a) status.rates[a] = rates[a];

            // качают ли коптер (проверка перед ARM), крутится ли (посадка, переворот)
            const float motion = fabsf(rates[0]) + fabsf(rates[1]) + fabsf(rates[2]);
            motionLpf.apply(motion);
            motionTimer = motion > 15.0f ? 0.0f : motionTimer + dt;

            // ---- связь, ARM, failsafe ----
            CopterRc rc = rcIn;
            const bool rcValid = handleLink(rc);
            handleArming(rc, sens, rcValid);
            if (status.state != CopterState::DISARMED) detectCrash();

            if (status.state == CopterState::DISARMED)
            {
                resetFlight(sens);
                if (testLeft > 0.0f)
                {
                    testLeft -= dt;
                    out.motor[testMotor] = tune.motorCommand(testThrust);
                    out.spinning = true;
                }
                avgThrust = 0.0f;
                status.throttle = 0.0f;
                return out;
            }

            // ---- уставки ----
            const bool failsafe = status.state == CopterState::FAILSAFE;
            const CopterMode mode = failsafe ? CopterMode::ALT_HOLD : rc.mode;
            if (mode != status.mode && (mode == CopterMode::ALT_HOLD))
            {
                altCtl.reset(vertical.altitude(), status.throttle);
            }
            status.mode = mode;

            const float sticks[4] = { stickLpf[0].apply(rc.roll), stickLpf[1].apply(rc.pitch),
                                      stickLpf[2].apply(rc.yaw), stickLpf[3].apply(rc.throttle) };
            float sp[3];
            rateSetpoints(mode, sticks, sp);
            for (int a = 0; a < 3; ++a) status.setpoint[a] = sp[a];

            // ---- газ ----
            float thrust;
            const float cosTilt = attitude.cosTilt();
            if (mode == CopterMode::ALT_HOLD)
            {
                float climb = 0.0f;
                if (failsafe)
                {
                    climb = -set.failsafeDescent;
                }
                else
                {
                    const float s = sticks[3] - 0.5f;   // центр — держать, края — набор/снижение
                    const float dead = 0.08f;
                    if (s > dead) climb = (s - dead) / (0.5f - dead) * set.alt.maxClimb;
                    if (s < -dead) climb = (s + dead) / (0.5f - dead) * set.alt.maxDescent;
                }
                altCtl.setHover(hover);
                thrust = altCtl.update(vertical.altitude(), vertical.climbRate(), climb, vertical.baroUsed(), cosTilt, dt);
                status.altTarget = altCtl.getTarget();
            }
            else
            {
                thrust = sticks[3];
                status.altTarget = vertical.altitude();
            }

            // в воздухе ли: газ выше 60 % висения (сел — ниже 55 %, checkLanded)
            if (!status.airborne && thrust > 0.6f * hover) status.airborne = true;

            // газ висения: учится, пока коптер спокойно висит (любой режим);
            // при снижении по команде — нет: севший коптер «висит» на
            // падающем газе, и оценка ушла бы за ним вниз
            const bool holding = mode != CopterMode::ALT_HOLD || fabsf(altCtl.climbTarget()) < 0.2f;
            const bool plausible = thrust * cosTilt > 0.4f * hover;   // на земле газ ниже — не висение
            if (status.airborne && holding && plausible && fabsf(vertical.climbRate()) < 0.5f && cosTilt > 0.95f &&
                fabsf(accelUp) < 1.0f)
            {
                const float k = dt / 3.0f;
                hover += k * (thrust * cosTilt - hover);
                hover = fminf(0.9f, fmaxf(0.03f, hover));
            }
            status.hoverLearned = hover;

            // anti-gravity: рывок газом -> интеграл временно сильнее
            const float thrHp = fabsf(thrust - throttleHpLpf.apply(thrust));
            const float iBoost = 1.0f + fminf(3.0f, thrHp * 30.0f);

            // ---- контур угловой скорости ----
            const float tpa = tune.tpaAt(lastMixThrottle);
            const bool holdI = !status.airborne;
            MixInput mi;
            mi.throttle = thrust;
            mi.roll = axes[0].update(sp[0], rates[0], tpa, iBoost, holdI, status.saturated);
            mi.pitch = axes[1].update(sp[1], rates[1], tpa, iBoost, holdI, status.saturated);
            mi.yaw = axes[2].update(sp[2], rates[2], 1.0f, iBoost, holdI, status.saturated);
            if (holdI)
            {
                for (auto& ax : axes) ax.resetIntegral();
            }

            const MixResult mr = mixer.mix(tune.frame, mi);
            status.saturated = mr.saturated;
            status.throttle = thrust;
            lastMixThrottle = mr.throttle;

            // ---- моторы ----
            float vComp = 1.0f;
            if (set.voltageCompensation && status.volts > 1.0f)
            {
                vComp = fminf(1.3f, fmaxf(1.0f, tune.volts / status.volts));
            }
            float sum = 0.0f;
            for (uint8_t i = 0; i < out.count; ++i)
            {
                sum += mr.motor[i];
                const float signal = Mixer::linearize(mr.motor[i], tune.thrustLinear) * vComp;
                out.motor[i] = Mixer::withIdle(signal, tune.idle);
            }
            avgThrust = out.count ? sum / out.count : 0.0f;
            out.spinning = true;

            checkLanded(thrust);
            if (!isArmed())
            {
                // сел в failsafe — моторы стоп в этом же цикле
                out = CopterOutputs();
                out.count = tune.frame.count;
            }
            return out;
        }

        const CopterStatus& getStatus() const { return status; }
        const CopterTune& getTune() const { return tune; }
        const CopterSettings& getSettings() const { return set; }
        const AxisTerms& axisTerms(uint8_t a) const { return axes[a < 3 ? a : 0].lastTerms(); }
        const GyroFilter& getGyroFilter() const { return gyroFilter; }
        bool isArmed() const { return status.state != CopterState::DISARMED; }

        // Разоружить немедленно (консоль, ошибка).
        void disarm()
        {
            status.state = CopterState::DISARMED;
            status.airborne = false;
        }

    private:
        bool spinning() const { return status.state != CopterState::DISARMED; }

        void resetFlight(const CopterSensors& sens)
        {
            for (auto& ax : axes) ax.reset();
            for (auto& f : stickLpf) f.reset();
            throttleHpLpf.reset();
            status.airborne = false;
            status.saturated = false;
            lastMixThrottle = 0.0f;
            failsafeTimer = 0.0f;
            landingTimer = 0.0f;
            crashTimer = 0.0f;
            lostControlTimer = 0.0f;
            levelErrorDeg = 0.0f;
            if (sens.baroOk) baroHome = sens.baroAltM;
            vertical.reset(sens.baroOk ? sens.baroAltM - baroHome : 0.0f);
            altCtl.reset(0.0f, 0.0f);
        }

        // Возвращает false, если ручкам пульта верить нельзя (связи нет).
        bool handleLink(CopterRc& rc)
        {
            if (rc.linkOk)
            {
                lostTimer = 0.0f;
                lastGoodRc = rc;
                if (status.state == CopterState::FAILSAFE)
                {
                    recoverTimer += dt;
                    if (recoverTimer >= set.failsafeRecoveryS)
                    {
                        status.state = CopterState::ARMED;
                        recoverTimer = 0.0f;
                        // режим — какой стоит на пульте; ALT_HOLD подхватит текущий газ
                        altCtl.reset(vertical.altitude(), status.throttle);
                    }
                }
                return true;
            }

            recoverTimer = 0.0f;
            lostTimer += dt;
            // короткий провал: держим последние ручки
            if (lostTimer < set.failsafeGuardS && status.state == CopterState::ARMED)
            {
                rc = lastGoodRc;
                rc.linkOk = false;
                return true;
            }
            rc.roll = rc.pitch = rc.yaw = 0.0f;
            rc.throttle = 0.0f;
            if (status.state == CopterState::ARMED)
            {
                if (!status.airborne)
                {
                    disarm();
                }
                else
                {
                    status.state = CopterState::FAILSAFE;
                    failsafeTimer = 0.0f;
                    landingTimer = 0.0f;
                    altCtl.reset(vertical.altitude(), status.throttle);
                }
            }
            return false;
        }

        uint16_t computeBlockers(const CopterRc& rc, const CopterSensors& sens) const
        {
            uint16_t b = 0;
            if (!rc.linkOk) b |= ARM_BLOCK_NO_LINK;
            if (rc.throttle > 0.05f && rc.mode != CopterMode::ALT_HOLD) b |= ARM_BLOCK_THROTTLE;
            if (rc.mode == CopterMode::ALT_HOLD && rc.throttle > 0.42f) b |= ARM_BLOCK_THROTTLE;
            if (!sens.imuOk) b |= ARM_BLOCK_IMU;
            if (uptime < 2.0f || !attitude.isInitialized()) b |= ARM_BLOCK_SETTLING;
            else if (attitude.tiltDeg() > 25.0f) b |= ARM_BLOCK_TILT;
            if (motionTimer < 0.3f) b |= ARM_BLOCK_MOVING;
            if (status.volts > 1.0f && tune.volts > 1.0f)
            {
                const float cells = roundf(tune.volts / 3.75f);
                if (cells >= 1.0f && status.volts / cells < set.minCellVolts) b |= ARM_BLOCK_BATTERY;
            }
            if (testLeft > 0.0f) b |= ARM_BLOCK_MOTOR_TEST;
            if (status.crashed) b |= ARM_BLOCK_CRASH;
            if (externalBlock) b |= ARM_BLOCK_EXTERNAL;
            if (!switchSeenOff) b |= ARM_BLOCK_SWITCH;
            return b;
        }

        void handleArming(const CopterRc& rc, const CopterSensors& sens, bool rcValid)
        {
            status.armBlockers = computeBlockers(rc, sens);
            if (!rcValid)
            {
                // тумблер не виден: после возврата связи ARM — только новым щелчком
                switchSeenOff = false;
                prevSwitch = true;
                return;
            }
            if (!rc.armSwitch)
            {
                switchSeenOff = true;
                status.crashed = false;
                if (status.state != CopterState::DISARMED) disarm();
                prevSwitch = false;
                return;
            }
            const bool rising = !prevSwitch;
            prevSwitch = true;
            if (status.state != CopterState::DISARMED) return;
            if (rising && status.armBlockers == 0)
            {
                status.state = CopterState::ARMED;
                status.mode = rc.mode;
                ++status.armCount;
                testLeft = 0.0f;
                attitude.setGrounded(true);
                if (rc.mode == CopterMode::ALT_HOLD) altCtl.reset(vertical.altitude(), 0.0f);
            }
            // ARM включён, но армить нельзя — нужен новый щелчок тумблером
            switchSeenOff = false;
        }

        void detectCrash()
        {
            if (!set.crashDetection) return;
            const float tilt = attitude.tiltDeg();
            const float motion = motionLpf.value();
            const bool levelMode = status.mode == CopterMode::ANGLE || status.mode == CopterMode::ALT_HOLD;
            const bool overturned = levelMode && tilt > 90.0f;
            // наклон не тот, что просит стик, больше 30° и дольше 2 с — коптер
            // не управляется (отказал мотор, слетел винт, лежит в траве на боку)
            lostControlTimer = (levelMode && levelErrorDeg > 30.0f) ? lostControlTimer + dt : 0.0f;
            if (lostControlTimer > 2.0f)
            {
                status.crashed = true;
                switchSeenOff = false;
                disarm();
                return;
            }
            // лежит на спине: акселерометр видит силу по оси −Z рамы — в
            // воздухе такой нет (тяга винтов — всегда по +Z), её даёт только земля
            const bool upsideDownStill = tilt > 110.0f && motion < 60.0f && accelG[2] < -0.6f;
            if (overturned || upsideDownStill)
            {
                crashTimer += dt;
                const float limit = overturned ? 0.7f : 1.0f;
                if (crashTimer >= limit)
                {
                    status.crashed = true;
                    switchSeenOff = false;
                    disarm();
                }
            }
            else
            {
                crashTimer = 0.0f;
            }
        }

        // Сел: тяга заметно меньше висения, а коптер не падает и не крутится.
        // Интеграл снова держится (не накручивается о землю), в failsafe — DISARM.
        void checkLanded(float thrust)
        {
            const bool calm = motionLpf.value() < 20.0f;
            const bool lowThrust = thrust < 0.55f * hover;
            // Стоит на земле: акселерометр видит 1 g, а тяга винтов много
            // меньше веса — в воздухе он показывает только тягу, 1 g при такой
            // тяге даёт лишь опора. Оценка скорости после удара о землю ещё
            // долго «падает» (акселерометр на ударе зашкаливает), поэтому
            // одной её мало.
            const float g = sqrtf(accelG[0] * accelG[0] + accelG[1] * accelG[1] + accelG[2] * accelG[2]);
            const bool resting = g > 0.85f && g < 1.15f && thrust < 0.3f * hover;
            const bool landed = calm && ((fabsf(vertical.climbRate()) < 0.3f && lowThrust) || resting);
            landingTimer = (status.airborne && landed) ? landingTimer + dt : 0.0f;
            if (landingTimer > 1.0f)
            {
                status.airborne = false;
                landingTimer = 0.0f;
                vertical.reset(vertical.altitude());
                if (status.state == CopterState::FAILSAFE) disarm();
            }
            if (status.state == CopterState::FAILSAFE)
            {
                failsafeTimer += dt;
                if (failsafeTimer > set.failsafeMaxS) disarm();
            }
        }

        void rateSetpoints(CopterMode mode, const float sticks[4], float sp[3])
        {
            float acro[3];
            for (int a = 0; a < 3; ++a) acro[a] = tune.rates[a].apply(sticks[a]);
            if (mode == CopterMode::ACRO)
            {
                for (int a = 0; a < 3; ++a) sp[a] = acro[a];
                return;
            }

            // Угол: желаемый «верх» в осях рамы против текущего -> поворот
            // кратчайшим путём; рысканье — вокруг вертикали Земли.
            const float maxA = tune.maxAngleDeg * PI_F / 180.0f;
            const float tr = sticks[0] * maxA, tp = sticks[1] * maxA;
            const Vec3 target = { sinf(tp), -sinf(tr) * cosf(tp), -cosf(tr) * cosf(tp) };
            const Vec3 up = attitude.upBody();
            Vec3 axis = cross(target, up);
            const float s = length(axis);
            const float angle = atan2f(s, dot(target, up));
            levelErrorDeg = angle * 180.0f / PI_F;
            float level[3] = { 0.0f, 0.0f, 0.0f };
            if (s > 1e-6f)
            {
                const float k = tune.levelGain * angle * 180.0f / PI_F / s;
                level[0] = axis.x * k;
                level[1] = axis.y * k;
                level[2] = axis.z * k;
            }
            const Vec3 down = attitude.downBody();
            const float yawRate = acro[2];
            float angleSp[3] = { level[0] + yawRate * down.x, level[1] + yawRate * down.y, level[2] + yawRate * down.z };
            const float cap = 0.9f * tune.rates[0].maxDps;
            for (int a = 0; a < 2; ++a) angleSp[a] = fminf(cap, fmaxf(-cap, angleSp[a]));

            if (mode == CopterMode::HORIZON)
            {
                const float deflection = fmaxf(fabsf(sticks[0]), fabsf(sticks[1]));
                const float h = fminf(1.0f, deflection / fmaxf(0.05f, set.horizonTransition));
                const float blend = h * h;
                for (int a = 0; a < 3; ++a) sp[a] = (1.0f - blend) * angleSp[a] + blend * acro[a];
                return;
            }
            for (int a = 0; a < 3; ++a) sp[a] = angleSp[a];
        }

        CopterTune tune;
        CopterSettings set;
        float dt = 0.001f;
        Mixer mixer;
        GyroFilter gyroFilter;
        RateAxis axes[3];
        Pt1 stickLpf[4];
        Pt1 throttleHpLpf;
        Pt1 voltLpf;
        Pt2 accelLpf[3];
        float accelG[3] = { 0.0f, 0.0f, 1.0f };   // после ФНЧ
        Pt1 motionLpf;
        AttitudeQuat attitude;
        VerticalEstimator vertical;
        AltitudeController altCtl;
        CopterStatus status;
        CopterRc lastGoodRc;

        float hover = 0.4f;
        float avgThrust = 0.0f;
        float rotEstimate = 0.0f;
        float lastMixThrottle = 0.0f;
        float uptime = 0.0f;
        float motionTimer = 0.0f;
        float lostTimer = 0.0f;
        float recoverTimer = 0.0f;
        float failsafeTimer = 0.0f;
        float landingTimer = 0.0f;
        float crashTimer = 0.0f;
        float lostControlTimer = 0.0f;
        float levelErrorDeg = 0.0f;
        float baroHome = 0.0f;
        bool switchSeenOff = false;
        bool prevSwitch = false;
        bool externalBlock = false;

        uint8_t testMotor = 0;
        float testThrust = 0.0f;
        float testLeft = 0.0f;
    };
}
