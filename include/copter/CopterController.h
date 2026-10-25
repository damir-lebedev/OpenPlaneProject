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
//     газ внизу, коптер стоит ровно (small_angle) и не качается,
//     гироскоп жив, батарея не пустая, проверка моторов выключена; ARM,
//     включённый раньше времени, надо выключить и включить заново;
//   • выключение ARM — моторы стоп в любой момент (аварийная кнопка);
//   • заармлен, но стоит на земле с газом внизу auto_disarm_delay — DISARM;
//   • runaway takeoff: после ARM сумма PID упёрлась (больше 60 % хода
//     моторов) и коптер при этом вращается — 75 мс подряд, — DISARM
//     (перепутаны моторы, винты, установка платы). Проверка гаснет, когда
//     коптер полсекунды спокойно летит на газе;
//   • потеря связи (или режим FAILSAFE с тумблера): первые 0.3 с —
//     последние ручки; дальше на земле — сразу DISARM, в воздухе этап 1 —
//     ровно и держит высоту до failsafe_delay (стик с `rxfail` HOLD —
//     последнее его положение, газ HOLD — последний газ вместо высоты),
//     затем этап 2 по
//     failsafe_procedure: AUTO-LAND — ровно и снижение 0.7 м/с (с
//     барометром — по нему, без — по акселерометру), касание земли —
//     DISARM; DROP — моторы стоп. Связь вернулась: на этапе 1 — управление
//     пилоту сразу, на этапе 2 — через failsafe_recovery_delay; после
//     DISARM защитой ARM — только когда связь продержалась столько же и
//     тумблер выключили;
//   • авария: в ANGLE/ALT_HOLD наклон больше 90° дольше 0.7 с или не тот,
//     что просит стик, больше 30° дольше 2 с (отказал мотор, лежит на
//     боку); в любом режиме лежит вверх ногами 1 с или рысканье разошлось
//     с уставкой больше чем на 200 °/с дольше 1.5 с (волчок на трёх
//     моторах) — моторы стоп (а не молотят винтами по земле), повторный
//     ARM — после выключения тумблера.
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

    // Почему не армится (битовая маска). Биты 0–25 — номера флагов запрета
    // ARM Betaflight 4.5 (`status`, MSP_STATUS_EX: конфигуратор BF читает
    // маску по номерам), коды — их имена. Наши проверки, которых в BF нет, —
    // с бита 26. Флаги BF, которые пока ставить нечему (CMS, PARALYZE, GPS…),
    // зарезервированы — их ставит снаружи то, что появится (setExternalBlockers).
    enum ArmBlock : uint32_t
    {
        ARM_BLOCK_IMU = 1u << 0,              // NOGYRO
        ARM_BLOCK_FAILSAFE = 1u << 1,         // failsafe был — ждём связь failsafe_recovery_delay
        ARM_BLOCK_NO_LINK = 1u << 2,          // RXLOSS
        ARM_BLOCK_NOT_DISARMED = 1u << 3,     // после DISARM защитой тумблер ARM так и включён
        ARM_BLOCK_FAILSAFE_SWITCH = 1u << 4,  // BOXFAILSAFE
        ARM_BLOCK_RUNAWAY = 1u << 5,
        ARM_BLOCK_CRASH = 1u << 6,
        ARM_BLOCK_THROTTLE = 1u << 7,
        ARM_BLOCK_TILT = 1u << 8,             // ANGLE: больше small_angle
        ARM_BLOCK_BOOT_GRACE = 1u << 9,
        ARM_BLOCK_NO_PREARM = 1u << 10,
        ARM_BLOCK_LOAD = 1u << 11,
        ARM_BLOCK_SETTLING = 1u << 12,        // CALIB
        ARM_BLOCK_CLI = 1u << 13,
        ARM_BLOCK_CMS = 1u << 14,
        ARM_BLOCK_BST = 1u << 15,
        ARM_BLOCK_MSP = 1u << 16,
        ARM_BLOCK_PARALYZE = 1u << 17,
        ARM_BLOCK_GPS = 1u << 18,
        ARM_BLOCK_RESCUE_SWITCH = 1u << 19,   // RESC
        ARM_BLOCK_RPM_FILTER = 1u << 20,
        ARM_BLOCK_REBOOT = 1u << 21,
        ARM_BLOCK_DSHOT_BITBANG = 1u << 22,
        ARM_BLOCK_NO_ACC_CAL = 1u << 23,
        ARM_BLOCK_MOTOR_PROTOCOL = 1u << 24,
        ARM_BLOCK_SWITCH = 1u << 25,          // ARMSWITCH: включён заранее
        // наши, в BF таких нет
        ARM_BLOCK_MOVING = 1u << 26,
        ARM_BLOCK_BATTERY = 1u << 27,
        ARM_BLOCK_MOTOR_TEST = 1u << 28,
        ARM_BLOCK_NO_ARM_SWITCH = 1u << 29,
        ARM_BLOCK_CONFIG = 1u << 30,
        ARM_BLOCK_LAST = ARM_BLOCK_CONFIG,
    };

    // Флаги, у которых есть номер в Betaflight 4.5 (биты 0–25).
    constexpr uint8_t ARM_BLOCK_BF_COUNT = 26;
    constexpr uint32_t ARM_BLOCK_BF_MASK = (1u << ARM_BLOCK_BF_COUNT) - 1u;

    constexpr uint32_t ARM_BLOCK_EXTERNAL_MASK = ARM_BLOCK_CLI | ARM_BLOCK_MSP | ARM_BLOCK_NO_ACC_CAL | ARM_BLOCK_REBOOT |
                                                 ARM_BLOCK_CONFIG | ARM_BLOCK_LOAD | ARM_BLOCK_BOOT_GRACE | ARM_BLOCK_CMS |
                                                 ARM_BLOCK_BST | ARM_BLOCK_PARALYZE | ARM_BLOCK_GPS | ARM_BLOCK_RESCUE_SWITCH |
                                                 ARM_BLOCK_RPM_FILTER | ARM_BLOCK_DSHOT_BITBANG | ARM_BLOCK_MOTOR_PROTOCOL;

    inline const char* armBlockText(uint32_t bit)
    {
        switch (bit)
        {
            case ARM_BLOCK_IMU: return "гироскоп/акселерометр не отвечает";
            case ARM_BLOCK_FAILSAFE: return "был failsafe — связь должна продержаться failsafe_recovery_delay";
            case ARM_BLOCK_NO_LINK: return "нет связи с пультом";
            case ARM_BLOCK_NOT_DISARMED: return "моторы остановила защита — выключить ARM и включить снова";
            case ARM_BLOCK_FAILSAFE_SWITCH: return "включён режим FAILSAFE (тумблер)";
            case ARM_BLOCK_RUNAWAY: return "коптер «убегал» на взлёте (моторы, винты, установка платы?) — выключить ARM";
            case ARM_BLOCK_CRASH: return "была авария (переворот, потеря управления) — выключить ARM";
            case ARM_BLOCK_THROTTLE: return "газ не внизу";
            case ARM_BLOCK_TILT: return "коптер стоит не ровно (больше small_angle)";
            case ARM_BLOCK_BOOT_GRACE: return "сразу после включения";
            case ARM_BLOCK_NO_PREARM: return "PREARM не включён";
            case ARM_BLOCK_LOAD: return "цикл не успевает (загрузка больше 100 %)";
            case ARM_BLOCK_SETTLING: return "ориентация ещё не определилась (2 с после включения)";
            case ARM_BLOCK_CLI: return "открыт CLI";
            case ARM_BLOCK_CMS: return "открыто меню OSD";
            case ARM_BLOCK_BST: return "шина BST";
            case ARM_BLOCK_MSP: return "в приложении несохранённые настройки";
            case ARM_BLOCK_PARALYZE: return "включён режим PARALYZE";
            case ARM_BLOCK_GPS: return "нет GPS для возврата домой";
            case ARM_BLOCK_RESCUE_SWITCH: return "включён тумблер возврата домой";
            case ARM_BLOCK_RPM_FILTER: return "нет оборотов от регуляторов для фильтра";
            case ARM_BLOCK_REBOOT: return "нужна перезагрузка (сменилась рама или выходы)";
            case ARM_BLOCK_DSHOT_BITBANG: return "DShot не запустился";
            case ARM_BLOCK_NO_ACC_CAL: return "не откалибрована установка платы ('o' в консоли)";
            case ARM_BLOCK_MOTOR_PROTOCOL: return "протокол моторов не выбран или не работает";
            case ARM_BLOCK_SWITCH: return "ARM был включён заранее — выключить и включить снова";
            case ARM_BLOCK_MOVING: return "коптер качают — поставить на землю";
            case ARM_BLOCK_BATTERY: return "батарея разряжена";
            case ARM_BLOCK_MOTOR_TEST: return "идёт проверка моторов";
            case ARM_BLOCK_NO_ARM_SWITCH: return "ARM не назначен ни на один тумблер (aux)";
            case ARM_BLOCK_CONFIG: return "описание коптера не сходится с платой (см. консоль)";
        }
        return "?";
    }

    // Код как в `status` Betaflight; наши проверки — своими словами.
    inline const char* armBlockCode(uint32_t bit)
    {
        switch (bit)
        {
            case ARM_BLOCK_IMU: return "NOGYRO";
            case ARM_BLOCK_FAILSAFE: return "FAILSAFE";
            case ARM_BLOCK_NO_LINK: return "RXLOSS";
            case ARM_BLOCK_NOT_DISARMED: return "NOT_DISARMED";
            case ARM_BLOCK_FAILSAFE_SWITCH: return "BOXFAILSAFE";
            case ARM_BLOCK_RUNAWAY: return "RUNAWAY";
            case ARM_BLOCK_CRASH: return "CRASH";
            case ARM_BLOCK_THROTTLE: return "THROTTLE";
            case ARM_BLOCK_TILT: return "ANGLE";
            case ARM_BLOCK_BOOT_GRACE: return "BOOTGRACE";
            case ARM_BLOCK_NO_PREARM: return "NOPREARM";
            case ARM_BLOCK_LOAD: return "LOAD";
            case ARM_BLOCK_SETTLING: return "CALIB";
            case ARM_BLOCK_CLI: return "CLI";
            case ARM_BLOCK_CMS: return "CMS";
            case ARM_BLOCK_BST: return "BST";
            case ARM_BLOCK_MSP: return "MSP";
            case ARM_BLOCK_PARALYZE: return "PARALYZE";
            case ARM_BLOCK_GPS: return "GPS";
            case ARM_BLOCK_RESCUE_SWITCH: return "RESC";
            case ARM_BLOCK_RPM_FILTER: return "RPMFILTER";
            case ARM_BLOCK_REBOOT: return "REBOOT_REQD";
            case ARM_BLOCK_DSHOT_BITBANG: return "DSHOT_BBANG";
            case ARM_BLOCK_NO_ACC_CAL: return "NO_ACC_CAL";
            case ARM_BLOCK_MOTOR_PROTOCOL: return "MOTOR_PROTO";
            case ARM_BLOCK_SWITCH: return "ARMSWITCH";
            case ARM_BLOCK_MOVING: return "MOVING";
            case ARM_BLOCK_BATTERY: return "BATTERY";
            case ARM_BLOCK_MOTOR_TEST: return "MOTOR_TEST";
            case ARM_BLOCK_NO_ARM_SWITCH: return "NO_ARM_AUX";
            case ARM_BLOCK_CONFIG: return "CONFIG";
        }
        return "?";
    }

    // Что делать на втором этапе failsafe (failsafe_procedure, порядок — как в BF).
    enum class FailsafeProcedure : uint8_t
    {
        AUTO_LAND,    // ровно и снижение (у нас — по высоте; failsafe_throttle > 1000 — фиксированный газ, как в BF)
        DROP,         // моторы стоп сразу
        GPS_RESCUE,   // возврата по GPS пока нет — снижение, как AUTO_LAND; вернуть управление — стиком
    };

    // Режим FAILSAFE с тумблера (failsafe_switch_mode, порядок — как в BF).
    enum class FailsafeSwitchMode : uint8_t
    {
        STAGE1,   // как потеря связи: этап 1, затем процедура
        KILL,     // моторы стоп сразу
        STAGE2,   // сразу процедура
    };

    enum class CopterState : uint8_t
    {
        DISARMED,
        ARMED,       // пилот управляет
        FAILSAFE,    // нет связи: этап 1 — ровно и высота, этап 2 — снижение (status.failsafeStage)
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
        bool airmode = false;          // режим AIR MODE с тумблера (поверх настройки)
        bool failsafeSwitch = false;   // режим FAILSAFE: как потеря связи
        bool prearm = true;            // PREARM включён или его нет в таблице
        bool armAssigned = true;       // ARM есть в таблице режимов
        uint32_t frame = 0;            // счётчик кадров приёмника; 0 — неизвестно (кадр — раз в 1/FF_NOMINAL_FRAME_HZ)
        // rxfail стиков = HOLD: на этапе 1 failsafe — последнее верное значение
        // (биты: 0 — крен, 1 — тангаж, 2 — рысканье, 3 — газ); AUTO — автомат
        uint8_t stickHold = 0;
    };

    // Кадров приёмника не видно (симулятор, тесты) — для feedforward_* они
    // приходят с такой частотой (CRSF 250 Гц).
    constexpr float FF_NOMINAL_FRAME_HZ = 250.0f;

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
        // failsafe (BF: failsafe_*). Связи нет FAILSAFE_HOLD_S — этап 1:
        // ровно и держит высоту; нет failsafeDelayS — этап 2 (процедура)
        float failsafeDescent = 0.7f;      // м/с: снижение AUTO-LAND
        float failsafeDelayS = 1.5f;       // failsafe_delay: столько связи нет — этап 2
        float failsafeLandingS = 120.0f;   // failsafe_landing_time: не села за это время — DISARM
        float failsafeThrottle = 0.0f;     // failsafe_throttle: 0 — снижение по высоте, иначе газ 0..1 как в BF
        float failsafeThrottleLowS = 10.0f;   // failsafe_throttle_low_delay: газ внизу дольше — сразу DISARM; 0 — нет
        float failsafeRecoveryS = 0.5f;    // failsafe_recovery_delay: связь держится столько — управление пилоту
        float failsafeStickThreshold = 0.3f;   // failsafe_stick_threshold: GPS-RESCUE отдаёт управление по стику
        FailsafeProcedure failsafeProcedure = FailsafeProcedure::AUTO_LAND;
        FailsafeSwitchMode failsafeSwitchMode = FailsafeSwitchMode::STAGE1;
        float minCellVolts = 3.5f;         // ниже (без нагрузки) — не армить: vbat_warning_cell_voltage
        float smallAngleDeg = 25.0f;       // small_angle: наклон больше — не армить; 180 — проверки нет
        float autoDisarmS = 5.0f;          // auto_disarm_delay: на земле с газом внизу столько — DISARM; 0 — нет
        // runaway_takeoff_*: «убегает» на взлёте — DISARM; проверка гаснет,
        // когда коптер столько продержался в воздухе спокойно
        bool runawayPrevention = true;
        float runawayDeactivateS = 0.5f;
        float runawayDeactivateThrottle = 0.2f;
        bool crashDetection = true;
        AltitudeController::Settings alt;
    };

    struct CopterStatus
    {
        CopterState state = CopterState::DISARMED;
        CopterMode mode = CopterMode::ANGLE;
        uint32_t armBlockers = 0;
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
        bool runaway = false;          // DISARM защитой runaway takeoff (до выключения тумблера)
        bool runawayCheck = false;     // проверка runaway takeoff ещё работает (после ARM, до уверенного взлёта)
        uint8_t failsafeStage = 0;     // 0 — нет, 1 — ровно и держит высоту, 2 — процедура
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
            for (uint8_t a = 0; a < 3; ++a)
            {
                axes[a].setExtras(t.pid, a);
                axes[a].setMaxRate(t.rates[a].apply(1.0f));
                absCtl[a].configure(t.pid.absCutoffHz, dt);
            }
            for (int a = 0; a < 4; ++a) stickLpf[a].configure(s.rcSmoothingHz, dt);
            throttleHpLpf.configure(t.pid.antiGravityHz, dt);
            throttleBoostLpf.configure(t.pid.throttleBoostHz, dt);
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
        // фильтры, ориентация, ARM не сбрасываются. TPA, anti-gravity,
        // throttle boost, ФНЧ рысканья, линеаризация и предел моторов — тоже
        // из профиля (как в BF); фильтры гироскопа и D и всё остальное
        // берутся из configure().
        void applyProfile(const CopterTune& t)
        {
            tune.roll = t.roll;
            tune.pitch = t.pitch;
            tune.yaw = t.yaw;
            tune.gainsFromModel = t.gainsFromModel;
            tune.iLimit = t.iLimit;
            tune.pid = t.pid;
            tune.yawLpfHz = t.yawLpfHz;
            tune.thrustLinear = t.thrustLinear;
            tune.motorOutputLimit = t.motorOutputLimit;
            tune.vbatSagCompensation = t.vbatSagCompensation;
            for (int a = 0; a < 3; ++a) tune.rates[a] = t.rates[a];
            axes[0].setGains(t.roll);
            axes[1].setGains(t.pitch);
            axes[2].setGains(t.yaw);
            axes[2].setOutputLpf(t.yawLpfHz);
            for (uint8_t a = 0; a < 3; ++a)
            {
                axes[a].setLimit(t.iLimit);
                axes[a].setExtras(t.pid, a);
                axes[a].setMaxRate(t.rates[a].apply(1.0f));
                absCtl[a].setCutoff(t.pid.absCutoffHz);
            }
            throttleHpLpf.configure(t.pid.antiGravityHz, dt);
            throttleBoostLpf.configure(t.pid.throttleBoostHz, dt);
        }

        // Внешние причины не армить (ARM_BLOCK_EXTERNAL_MASK): CLI, приложение,
        // калибровка установки платы, ошибка описания, нужна перезагрузка.
        void setExternalBlockers(uint32_t mask) { externalBlockers = mask & ARM_BLOCK_EXTERNAL_MASK; }

        // vbat_cutoff_percent (copter/Battery.h): газ со стика × limit (0..1). Удержанию
        // высоты и автомату failsafe — нет: им газ нужен целиком, чтобы сесть.
        void setThrottleLimit(float limit) { throttleLimit = fminf(1.0f, fmaxf(0.0f, limit)); }

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
            trackFrames(rc.frame);
            const bool rcValid = handleLink(rc);
            handleArming(rc, sens, rcValid);
            if (status.state == CopterState::ARMED) autoDisarm(rc);
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
            // этап 2 с failsafe_throttle — ровно на постоянном газе, как в BF
            const bool fixedThrottle = failsafe && status.failsafeStage >= 2 && set.failsafeThrottle > 0.0f;
            // этап 1 с rxfail газа HOLD — ровно на последнем газе, как в BF
            const CopterMode mode = failsafe ? (fixedThrottle || heldThrottle ? CopterMode::ANGLE : CopterMode::ALT_HOLD) : rc.mode;
            if (mode != status.mode && (mode == CopterMode::ALT_HOLD))
            {
                altCtl.reset(vertical.altitude(), status.throttle);
            }
            status.mode = mode;

            const float sticks[4] = { stickLpf[0].apply(rc.roll), stickLpf[1].apply(rc.pitch),
                                      stickLpf[2].apply(rc.yaw), stickLpf[3].apply(rc.throttle) };
            float sp[3], stickSp[3];
            rateSetpoints(mode, sticks, sp, stickSp);
            const float raw[3] = { rc.roll, rc.pitch, rc.yaw };
            for (uint8_t a = 0; a < 3; ++a)
            {
                status.setpoint[a] = sp[a];
                FeedforwardInput ff;
                ff.stickSetpoint = stickSp[a];
                ff.deflection = fabsf(sticks[a]);
                ff.rawStick = raw[a];
                ff.newFrame = frameNow;
                ff.framePeriodS = framePeriod;
                axes[a].setFeedforwardInput(ff);
            }
            // absolute control: в воздухе; крен и тангаж — в ACRO (в ANGLE и
            // HORIZON угол держит горизонт), рысканье — в любом режиме
            for (uint8_t a = 0; a < 3; ++a)
            {
                if (tune.pid.absGain > 0.0f && status.airborne && (a == 2 || mode == CopterMode::ACRO))
                    sp[a] += absCtl[a].update(sp[a], rates[a], tune.pid.absGain, tune.pid.absLimit, tune.pid.absErrorLimit);
                else
                    absCtl[a].reset();
            }

            // ---- газ ----
            float thrust;   // что просит пилот (или удержание высоты), доля тяги
            const float cosTilt = attitude.cosTilt();
            const float stickSlow = throttleBoostLpf.apply(sticks[3]);
            if (mode == CopterMode::ALT_HOLD)
            {
                float climb = 0.0f;
                if (failsafe)
                {
                    climb = status.failsafeStage >= 2 ? -set.failsafeDescent : 0.0f;   // этап 1 — держит высоту
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
                thrust = fixedThrottle ? set.failsafeThrottle : sticks[3] * (failsafe ? 1.0f : throttleLimit);
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

            // anti-gravity: рывок газом -> интеграл (и P крена и тангажа)
            // временно сильнее: перекос тяги (центр тяжести не в центре,
            // моторы разные) растёт вместе с газом, и I успевает за ним.
            // Скорость газа — газ минус его ФНЧ, в ходах за секунду: для ФНЧ
            // anti_gravity_cutoff_hz это скорость, сглаженная на той же частоте.
            // iterm_windup: моторы разошлись (прошлый цикл) — I слабее
            const float thrRate = fabsf(thrust - throttleHpLpf.apply(thrust)) * 2.0f * PI_F * tune.pid.antiGravityHz;
            const float ag = fminf(1.0f, thrRate / ANTI_GRAVITY_FULL_RATE);
            const float iBoost = (1.0f + tune.pid.antiGravityI * ag) * itermWindupScale(lastMixSpread, tune.pid.itermWindup);
            const float pBoost = 1.0f + ANTI_GRAVITY_P_SHARE * tune.pid.antiGravityP * ag;

            // throttle_boost: быстрая часть газа со стика (рывок) усиливается —
            // моторы догоняют стик раньше, плавный газ не меняется. Удержанию
            // высоты — нет: у него свой контур, рассчитанный без добавки
            float mixThrust = thrust;
            if (mode != CopterMode::ALT_HOLD && tune.pid.throttleBoost > 0.0f)
            {
                mixThrust = fminf(1.0f, fmaxf(0.0f, thrust + tune.pid.throttleBoost * (sticks[3] - stickSlow)));
            }

            // ---- контур угловой скорости ----
            // TPA — по газу смесителя (прошлый цикл). tpa_low: по умолчанию —
            // только пока после ARM газ не поднимали выше точки (на земле,
            // перед взлётом), с tpa_low_always — всегда
            if (thrust >= tune.pid.tpaLowBreakpoint) tpaLowDone = true;
            float tpa = tune.tpaAt(lastMixThrottle);
            if (tune.pid.tpaLowAlways || !tpaLowDone) tpa *= tune.tpaLowAt(lastMixThrottle);
            const bool holdI = !status.airborne;
            if (tune.pid.itermRotation && !holdI)
            {
                float v[3] = { axes[0].integral(), axes[1].integral(), axes[2].integral() };
                rotateIterm(v, rates, dt);
                for (uint8_t a = 0; a < 3; ++a) axes[a].setIntegral(v[a]);
            }
            MixInput mi;
            mi.throttle = mixThrust;
            mi.roll = axes[0].update(sp[0], rates[0], tpa, iBoost, holdI, status.saturated, pBoost);
            mi.pitch = axes[1].update(sp[1], rates[1], tpa, iBoost, holdI, status.saturated, pBoost);
            mi.yaw = axes[2].update(sp[2], rates[2], 1.0f, iBoost, holdI, status.saturated);
            if (holdI)
            {
                for (auto& ax : axes) ax.resetIntegral();
            }

            mixer.setAirmode(set.airmode || rc.airmode);
            const MixResult mr = mixer.mix(tune.frame, mi);
            status.saturated = mr.saturated;
            status.throttle = thrust;
            lastMixThrottle = mr.throttle;
            float lo = 1.0f, hi = 0.0f;
            for (uint8_t i = 0; i < tune.frame.count; ++i)
            {
                lo = fminf(lo, mr.motor[i]);
                hi = fmaxf(hi, mr.motor[i]);
            }
            lastMixSpread = tune.frame.count ? hi - lo : 0.0f;
            checkRunaway(thrust, rates);

            // ---- моторы ----
            float vComp = 1.0f;
            if (set.voltageCompensation && status.volts > 1.0f)
            {
                // vbat_sag_compensation: доля поправки (100 % — моторы отвечают как на полной)
                vComp = 1.0f + (fminf(1.3f, fmaxf(1.0f, tune.volts / status.volts)) - 1.0f) * tune.vbatSagCompensation;
            }
            float sum = 0.0f;
            for (uint8_t i = 0; i < out.count; ++i)
            {
                sum += mr.motor[i];
                const float signal = Mixer::linearize(mr.motor[i], tune.thrustLinear) * vComp;
                out.motor[i] = Mixer::withIdle(signal, tune.idle, tune.motorOutputLimit);
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
        // Насколько D оси поднят к d_max: 0 — в покое, 1 — d_max.
        float dBoost(uint8_t a) const { return axes[a < 3 ? a : 0].dBoost(); }
        const GyroFilter& getGyroFilter() const { return gyroFilter; }
        bool isArmed() const { return status.state != CopterState::DISARMED; }

        // Разоружить немедленно (консоль, ошибка).
        void disarm()
        {
            status.state = CopterState::DISARMED;
            status.airborne = false;
            status.failsafeStage = 0;
            status.runawayCheck = false;
        }

    private:
        static constexpr float FAILSAFE_HOLD_S = 0.3f;        // провал связи короче — последние ручки (как в BF)
        static constexpr float FAILSAFE_KILL_LOCK_S = 1.0f;   // после KILL с тумблера ARM не раньше
        // runaway takeoff: сумма PID оси больше доли хода моторов, а гироскоп
        // по ней быстрее порога — RUNAWAY_TRIGGER_S подряд; «спокойно» —
        // сумма PID всех осей меньше RUNAWAY_CALM_PIDSUM
        static constexpr float RUNAWAY_PIDSUM = 0.6f;
        static constexpr float RUNAWAY_CALM_PIDSUM = 0.1f;
        static constexpr float RUNAWAY_GYRO_RP_DPS = 15.0f;
        static constexpr float RUNAWAY_GYRO_YAW_DPS = 50.0f;
        static constexpr float RUNAWAY_TRIGGER_S = 0.075f;

        // Газ внизу: для ARM, автодисарма. В ALT_HOLD газ у центра — висеть,
        // ниже середины — «не взлетать».
        static bool throttleLow(const CopterRc& rc)
        {
            return rc.mode == CopterMode::ALT_HOLD ? rc.throttle <= 0.42f : rc.throttle <= 0.05f;
        }

        // DISARM защитой: снова ARM — только выключив и включив тумблер.
        void forcedDisarm()
        {
            disarm();
            switchSeenOff = false;
            forcedOff = true;
        }

        // DISARM failsafe'ом: ARM ещё и после того, как связь продержится
        // failsafe_recovery_delay (KILL — 1 с).
        void failsafeDisarm(bool kill = false)
        {
            forcedDisarm();
            failsafeLatch = true;
            killLatch = kill;
        }

        // auto_disarm_delay: заармлен, стоит на земле, газ внизу — DISARM.
        void autoDisarm(const CopterRc& rc)
        {
            const bool idle = !status.airborne && rc.linkOk && throttleLow(rc);
            groundIdleTimer = idle ? groundIdleTimer + dt : 0.0f;
            if (set.autoDisarmS > 0.0f && groundIdleTimer >= set.autoDisarmS) forcedDisarm();
        }

        // Runaway takeoff: регулятор упёрся, а коптер всё равно вращается —
        // контур работает наоборот (моторы не на тех выходах, винты не той
        // стороной, плата повёрнута). Проверка — от ARM до уверенного взлёта.
        void checkRunaway(float thrust, const float rates[3])
        {
            if (!status.runawayCheck) return;
            bool runaway = false, calm = true;
            for (uint8_t a = 0; a < 3; ++a)
            {
                const AxisTerms& t = axes[a].lastTerms();
                const float sum = fabsf(t.p + t.i + t.d + t.ff);   // до pidsum_limit
                const float gyroLimit = a == 2 ? RUNAWAY_GYRO_YAW_DPS : RUNAWAY_GYRO_RP_DPS;
                if (sum > RUNAWAY_PIDSUM && fabsf(rates[a]) > gyroLimit) runaway = true;
                if (sum > RUNAWAY_CALM_PIDSUM) calm = false;
            }
            runawayTimer = runaway ? runawayTimer + dt : 0.0f;
            if (runawayTimer >= RUNAWAY_TRIGGER_S)
            {
                status.runaway = true;
                forcedDisarm();
                return;
            }
            // газ выше порога (или коптер уже поднялся выше метра — висение
            // у мощного коптера бывает ниже порога)
            const bool flying = thrust >= set.runawayDeactivateThrottle || (status.airborne && vertical.altitude() > 1.0f);
            if (flying && calm)
            {
                runawayFlown += dt;
                if (runawayFlown >= set.runawayDeactivateS) status.runawayCheck = false;
            }
        }

        bool spinning() const { return status.state != CopterState::DISARMED; }

        // Кадры приёмника: пришёл ли новый в этом цикле и их период
        // (сглаженный — кадры приходят неровно, иногда два за цикл).
        void trackFrames(uint32_t frame)
        {
            ++loopsSinceFrame;
            if (frame)
            {
                frameNow = frame != lastRcFrame;
                lastRcFrame = frame;
            }
            else
            {
                frameNow = static_cast<float>(loopsSinceFrame) * dt >= 1.0f / FF_NOMINAL_FRAME_HZ - 0.5f * dt;
            }
            if (!frameNow) return;
            const float p = fminf(0.1f, fmaxf(dt, static_cast<float>(loopsSinceFrame) * dt));
            framePeriod = framePeriod > 0.0f ? framePeriod + 0.1f * (p - framePeriod) : p;
            loopsSinceFrame = 0;
        }

        void resetFlight(const CopterSensors& sens)
        {
            for (auto& ax : axes) ax.reset();
            for (auto& f : stickLpf) f.reset();
            throttleHpLpf.reset();
            throttleBoostLpf.reset();
            tpaLowDone = false;
            status.airborne = false;
            status.saturated = false;
            lastMixThrottle = 0.0f;
            lastMixSpread = 0.0f;
            failsafeTimer = 0.0f;
            landingTimer = 0.0f;
            crashTimer = 0.0f;
            lostControlTimer = 0.0f;
            yawLostTimer = 0.0f;
            levelErrorDeg = 0.0f;
            runawayTimer = 0.0f;
            groundIdleTimer = 0.0f;
            if (sens.baroOk) baroHome = sens.baroAltM;
            vertical.reset(sens.baroOk ? sens.baroAltM - baroHome : 0.0f);
            altCtl.reset(0.0f, 0.0f);
        }

        // Возвращает false, если ручкам пульта верить нельзя (связи нет).
        bool handleLink(CopterRc& rc)
        {
            heldThrottle = false;
            if (rc.linkOk && !rc.failsafeSwitch)   // режим FAILSAFE — как потеря связи
            {
                lostTimer = 0.0f;
                lastGoodRc = rc;
                goodTimer += dt;
                if (failsafeLatch && goodTimer >= (killLatch ? FAILSAFE_KILL_LOCK_S : set.failsafeRecoveryS))
                {
                    failsafeLatch = killLatch = false;
                }
                if (status.state == CopterState::FAILSAFE) recoverFromFailsafe(rc);
                if (status.state == CopterState::FAILSAFE)
                {
                    // управление ещё у автомата: стики не трогают ни наклон, ни газ
                    rc.roll = rc.pitch = rc.yaw = 0.0f;
                    rc.throttle = 0.0f;
                }
                // failsafe_throttle_low_delay: газ внизу так долго — коптер, видно, на земле
                const bool low = rc.mode != CopterMode::ALT_HOLD && rc.throttle <= 0.05f;
                lowThrottleTimer = (status.state == CopterState::ARMED && low) ? lowThrottleTimer + dt : 0.0f;
                return true;
            }

            goodTimer = 0.0f;
            recoverTimer = 0.0f;
            const bool bySwitch = rc.failsafeSwitch && rc.linkOk;
            if (lostTimer == 0.0f || !rc.linkOk) failsafeBySwitch = bySwitch;
            lostTimer += dt;
            if (bySwitch && set.failsafeSwitchMode == FailsafeSwitchMode::KILL)
            {
                if (isArmed()) failsafeDisarm(true);
            }
            else if (status.state == CopterState::ARMED)
            {
                // короткий провал связи: последние ручки (тумблер — сразу)
                if (!bySwitch && lostTimer < FAILSAFE_HOLD_S)
                {
                    rc = lastGoodRc;
                    rc.linkOk = false;
                    return true;
                }
                const bool longLow = set.failsafeThrottleLowS > 0.0f && lowThrottleTimer >= set.failsafeThrottleLowS;
                if (!status.airborne || longLow)
                {
                    failsafeDisarm();   // на земле — просто DISARM
                }
                else
                {
                    status.state = CopterState::FAILSAFE;
                    status.failsafeStage = 1;
                    failsafeTimer = 0.0f;
                    landingTimer = 0.0f;
                    altCtl.reset(vertical.altitude(), status.throttle);
                }
            }
            if (status.state == CopterState::FAILSAFE && status.failsafeStage == 1)
            {
                const bool skip = bySwitch && set.failsafeSwitchMode == FailsafeSwitchMode::STAGE2;
                if (skip || lostTimer >= set.failsafeDelayS)
                {
                    if (set.failsafeProcedure == FailsafeProcedure::DROP)
                    {
                        failsafeDisarm();
                    }
                    else
                    {
                        status.failsafeStage = 2;
                        failsafeTimer = 0.0f;
                    }
                }
            }
            // этап 1: стики с rxfail HOLD — последние верные, прочие — автомат
            const bool stage1 = status.state == CopterState::FAILSAFE && status.failsafeStage == 1;
            const uint8_t hold = stage1 ? rc.stickHold : 0;
            rc.roll = (hold & 1u) ? lastGoodRc.roll : 0.0f;
            rc.pitch = (hold & 2u) ? lastGoodRc.pitch : 0.0f;
            rc.yaw = (hold & 4u) ? lastGoodRc.yaw : 0.0f;
            heldThrottle = (hold & 8u) != 0;
            rc.throttle = heldThrottle ? lastGoodRc.throttle : 0.0f;
            return false;
        }

        // Связь (или тумблер) вернулась во время failsafe. Этап 1 или режим
        // FAILSAFE с тумблера — управление пилоту сразу; этап 2 — когда связь
        // продержится failsafe_recovery_delay, а у GPS-RESCUE ещё и пилот
        // отклонит стик дальше failsafe_stick_threshold.
        void recoverFromFailsafe(const CopterRc& rc)
        {
            bool back = failsafeBySwitch || status.failsafeStage < 2;
            if (!back)
            {
                recoverTimer += dt;
                back = recoverTimer >= set.failsafeRecoveryS;
                if (back && set.failsafeProcedure == FailsafeProcedure::GPS_RESCUE)
                {
                    const float stick = fmaxf(fabsf(rc.roll), fmaxf(fabsf(rc.pitch), fabsf(rc.yaw)));
                    back = stick > set.failsafeStickThreshold;
                }
            }
            if (!back) return;
            status.state = CopterState::ARMED;
            status.failsafeStage = 0;
            recoverTimer = 0.0f;
            // режим — какой стоит на пульте; ALT_HOLD подхватит текущий газ
            altCtl.reset(vertical.altitude(), status.throttle);
        }

        uint32_t computeBlockers(const CopterRc& rc, const CopterSensors& sens) const
        {
            uint32_t b = 0;
            if (!rc.linkOk) b |= ARM_BLOCK_NO_LINK;
            if (failsafeLatch || status.state == CopterState::FAILSAFE) b |= ARM_BLOCK_FAILSAFE;
            if (!throttleLow(rc)) b |= ARM_BLOCK_THROTTLE;
            if (!sens.imuOk) b |= ARM_BLOCK_IMU;
            if (uptime < 2.0f || !attitude.isInitialized()) b |= ARM_BLOCK_SETTLING;
            else if (set.smallAngleDeg < 180.0f && attitude.tiltDeg() > set.smallAngleDeg) b |= ARM_BLOCK_TILT;
            if (motionTimer < 0.3f) b |= ARM_BLOCK_MOVING;
            if (status.volts > 1.0f && tune.volts > 1.0f)
            {
                const float cells = tune.cells ? tune.cells : roundf(tune.volts / 3.75f);
                if (cells >= 1.0f && status.volts / cells < set.minCellVolts) b |= ARM_BLOCK_BATTERY;
            }
            if (testLeft > 0.0f) b |= ARM_BLOCK_MOTOR_TEST;
            if (status.crashed) b |= ARM_BLOCK_CRASH;
            if (status.runaway) b |= ARM_BLOCK_RUNAWAY;
            if (rc.failsafeSwitch) b |= ARM_BLOCK_FAILSAFE_SWITCH;
            if (!rc.prearm) b |= ARM_BLOCK_NO_PREARM;
            if (!rc.armAssigned) b |= ARM_BLOCK_NO_ARM_SWITCH;
            b |= externalBlockers;
            if (!switchSeenOff) b |= forcedOff ? ARM_BLOCK_NOT_DISARMED : ARM_BLOCK_SWITCH;
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
                forcedOff = false;
                status.crashed = false;
                status.runaway = false;
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
                status.runawayCheck = set.runawayPrevention;
                runawayTimer = runawayFlown = 0.0f;
                groundIdleTimer = lowThrottleTimer = 0.0f;
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
            // рысканье не слушается: уставка и гироскоп разошлись больше чем на
            // 200 °/с дольше 1.5 с (любой режим) — отказал мотор, и коптер
            // волчком крутится на трёх: наклон он при этом может и держать
            yawLostTimer = fabsf(status.setpoint[2] - status.rates[2]) > 200.0f ? yawLostTimer + dt : 0.0f;
            if (lostControlTimer > 2.0f || yawLostTimer > 1.5f)
            {
                status.crashed = true;
                forcedDisarm();
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
                    forcedDisarm();
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
                if (status.state == CopterState::FAILSAFE) failsafeDisarm();
            }
            // failsafe_landing_time: этап 2 не кончился посадкой — DISARM
            if (status.state == CopterState::FAILSAFE && status.failsafeStage >= 2)
            {
                failsafeTimer += dt;
                if (failsafeTimer > set.failsafeLandingS) failsafeDisarm();
            }
        }

        // Уставки угловой скорости; stick — их часть от стика (для упреждения:
        // в ANGLE — наклон стиком, без поправки горизонта).
        void rateSetpoints(CopterMode mode, const float sticks[4], float sp[3], float stick[3])
        {
            float acro[3];
            for (int a = 0; a < 3; ++a) acro[a] = tune.rates[a].apply(sticks[a]);
            if (mode == CopterMode::ACRO)
            {
                for (int a = 0; a < 3; ++a) sp[a] = stick[a] = acro[a];
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
            // наклон стиком: у ровного коптера — levelGain × угол стика
            float angleStick[3] = { tune.levelGain * tune.maxAngleDeg * sticks[0], tune.levelGain * tune.maxAngleDeg * sticks[1],
                                    acro[2] };
            for (int a = 0; a < 2; ++a) angleStick[a] = fminf(cap, fmaxf(-cap, angleStick[a]));

            if (mode == CopterMode::HORIZON)
            {
                const float deflection = fmaxf(fabsf(sticks[0]), fabsf(sticks[1]));
                const float h = fminf(1.0f, deflection / fmaxf(0.05f, set.horizonTransition));
                const float blend = h * h;
                for (int a = 0; a < 3; ++a)
                {
                    sp[a] = (1.0f - blend) * angleSp[a] + blend * acro[a];
                    stick[a] = (1.0f - blend) * angleStick[a] + blend * acro[a];
                }
                return;
            }
            for (int a = 0; a < 3; ++a)
            {
                sp[a] = angleSp[a];
                stick[a] = angleStick[a];
            }
        }

        CopterTune tune;
        CopterSettings set;
        float dt = 0.001f;
        Mixer mixer;
        GyroFilter gyroFilter;
        RateAxis axes[3];
        AbsoluteControl absCtl[3];
        Pt1 stickLpf[4];
        Pt1 throttleHpLpf;               // anti-gravity
        Pt1 throttleBoostLpf;            // throttle_boost
        Pt1 voltLpf;
        bool heldThrottle = false;   // этап 1 failsafe на последнем газе (rxfail 3 h)
        float throttleLimit = 1.0f;   // vbat_cutoff_percent
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
        float lastMixSpread = 0.0f;      // разброс моторов прошлого цикла (iterm_windup)
        bool tpaLowDone = false;         // газ после ARM был выше tpa_low_breakpoint
        uint32_t lastRcFrame = 0;        // кадры приёмника (feedforward_*)
        uint32_t loopsSinceFrame = 0;
        bool frameNow = true;
        float framePeriod = 0.0f;
        float uptime = 0.0f;
        float motionTimer = 0.0f;
        float lostTimer = 0.0f;
        float goodTimer = 0.0f;          // связь держится (failsafe_recovery_delay после DISARM)
        float recoverTimer = 0.0f;
        float lowThrottleTimer = 0.0f;   // газ внизу (failsafe_throttle_low_delay)
        float groundIdleTimer = 0.0f;    // на земле с газом внизу (auto_disarm_delay)
        float runawayTimer = 0.0f;
        float runawayFlown = 0.0f;       // спокойного полёта после ARM (гасит runaway takeoff)
        bool failsafeLatch = false;      // DISARM failsafe'ом: ARM после связи failsafe_recovery_delay
        bool killLatch = false;          // ... а после KILL — 1 с
        bool failsafeBySwitch = false;   // failsafe включил тумблер, а не потеря связи
        bool forcedOff = false;          // DISARM защитой — код NOT_DISARMED
        float failsafeTimer = 0.0f;
        float landingTimer = 0.0f;
        float crashTimer = 0.0f;
        float lostControlTimer = 0.0f;
        float yawLostTimer = 0.0f;
        float levelErrorDeg = 0.0f;
        float baroHome = 0.0f;
        bool switchSeenOff = false;
        bool prevSwitch = false;
        uint32_t externalBlockers = 0;

        uint8_t testMotor = 0;
        float testThrust = 0.0f;
        float testLeft = 0.0f;
    };
}
