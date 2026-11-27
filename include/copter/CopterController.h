#pragma once
#include <math.h>
#include <stdint.h>

#include "copter/AltitudeHold.h"
#include "copter/Attitude.h"
#include "copter/Chirp.h"
#include "copter/GpsRescue.h"
#include "copter/GyroFilter.h"
#include "copter/ImuAlign.h"
#include "copter/LaunchControl.h"
#include "copter/Mixer.h"
#include "copter/MotorMusic.h"
#include "copter/MotorTrack.h"
#include "copter/PositionEstimator.h"
#include "copter/GpsIntegrity.h"
#include "copter/DeadReckonHome.h"
#include "copter/PositionHold.h"
#include "copter/Presets.h"
#include "copter/RateController.h"
#include "copter/RcSmoothing.h"
#include "copter/TrainingLimits.h"
#include "copter/Geofence.h"
#include "copter/Turtle.h"

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
//              набор/снижение;
//   POS_HOLD — ALT_HOLD + место по GPS: стики в центре — стоит над точкой,
//              стик — летит со скоростью, отпустил — тормозит и держит
//              (copter/PositionHold.h). Без годного GPS или курса в режим не
//              войти, а пропали в полёте — остаётся ALT_HOLD (ровно и высота,
//              стики — наклон) и предупреждение в очках; вернулись — снова
//              держит. В failsafe (этап 1 и посадка) место держится само.
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
//
// Turtle mode: ARM с режимом FLIP OVER AFTER CRASH (только DShot) —
// регуляторы крутят наоборот, стик выбирает сторону переворота
// (copter/Turtle.h); наклон при ARM не проверяется, PID и защиты от удара,
// runaway и автодисарм не работают. Выход — DISARM.
// ============================================================

namespace copter
{
    enum class CopterMode : uint8_t
    {
        ACRO,
        HORIZON,
        ANGLE,
        ALT_HOLD,
        POS_HOLD,
    };

    inline const char* copterModeName(CopterMode m)
    {
        switch (m)
        {
            case CopterMode::ACRO: return "ACRO";
            case CopterMode::HORIZON: return "HORIZON";
            case CopterMode::ANGLE: return "ANGLE";
            case CopterMode::ALT_HOLD: return "ALT_HOLD";
            case CopterMode::POS_HOLD: return "POS_HOLD";
        }
        return "?";
    }

    // Режимы, где газ — это высота (центр стика — висеть).
    inline bool altitudeMode(CopterMode m) { return m == CopterMode::ALT_HOLD || m == CopterMode::POS_HOLD; }

    // HEADFREE: стики крена и тангажа — в осях курса refDeg, а не носа.
    // deltaDeg — на сколько нос ушёл вправо от того курса: «от себя» ведёт
    // туда, куда смотрел нос при ARM (или HEADADJ), как бы коптер ни повернулся.
    // roll + вправо, pitch + на себя; каждая ось прижата к ±1.
    inline void headfreeRotate(float& roll, float& pitch, float deltaDeg)
    {
        const float d = deltaDeg * 3.14159265f / 180.0f, c = cosf(d), s = sinf(d);
        const float r = roll * c + pitch * s;
        const float p = pitch * c - roll * s;
        roll = r > 1.0f ? 1.0f : r < -1.0f ? -1.0f : r;
        pitch = p > 1.0f ? 1.0f : p < -1.0f ? -1.0f : p;
    }

    // FPV ANGLE MIX (BF fpv_mix_degrees): камера наклонена вверх на camDeg, и
    // стики крена и рысканья вращают коптер вокруг осей камеры, а не рамы —
    // в кадре разворот не заваливает горизонт. Скорости в °/с, оси FRD (крен +
    // вправо, рысканье + нос вправо); каждая прижата к своему потолку.
    inline void fpvMixRotate(float& rollDps, float& yawDps, float camDeg, float rollMax, float yawMax)
    {
        const float d = camDeg * 3.14159265f / 180.0f, c = cosf(d), s = sinf(d);
        const float r = rollDps * c + yawDps * s;
        const float y = yawDps * c - rollDps * s;
        rollDps = r > rollMax ? rollMax : r < -rollMax ? -rollMax : r;
        yawDps = y > yawMax ? yawMax : y < -yawMax ? -yawMax : y;
    }

    // Почему не армится (битовая маска). Биты 0–25 — номера флагов запрета
    // ARM Betaflight 4.5 (`status`, MSP_STATUS_EX: конфигуратор BF читает
    // маску по номерам), коды — их имена. Наши проверки, которых в BF нет, —
    // с бита 26. Флаги BF, которые пока ставить нечему (CMS, BST…),
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
        ARM_BLOCK_FENCE = 1u << 31,           // поле геозабора: коптер вне поля или в запретной зоне
        ARM_BLOCK_LAST = ARM_BLOCK_FENCE,     // циклы по битам — с «bit &&»: после 1u << 31 сдвиг даёт 0
    };

    // Флаги, у которых есть номер в Betaflight 4.5 (биты 0–25).
    constexpr uint8_t ARM_BLOCK_BF_COUNT = 26;
    constexpr uint32_t ARM_BLOCK_BF_MASK = (1u << ARM_BLOCK_BF_COUNT) - 1u;

    constexpr uint32_t ARM_BLOCK_EXTERNAL_MASK = ARM_BLOCK_CLI | ARM_BLOCK_MSP | ARM_BLOCK_NO_ACC_CAL | ARM_BLOCK_REBOOT |
                                                 ARM_BLOCK_CONFIG | ARM_BLOCK_LOAD | ARM_BLOCK_BOOT_GRACE | ARM_BLOCK_CMS |
                                                 ARM_BLOCK_BST | ARM_BLOCK_GPS | ARM_BLOCK_RESCUE_SWITCH |
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
            case ARM_BLOCK_PARALYZE: return "PARALYZE: ARM запрещён до перезагрузки";
            case ARM_BLOCK_GPS: return "нет GPS для возврата домой";
            case ARM_BLOCK_RESCUE_SWITCH: return "включён тумблер возврата домой";
            case ARM_BLOCK_RPM_FILTER: return "нет оборотов от регуляторов для фильтра";
            case ARM_BLOCK_REBOOT: return "нужна перезагрузка (сменилась рама или выходы)";
            case ARM_BLOCK_DSHOT_BITBANG: return "DShot не запустился";
            case ARM_BLOCK_NO_ACC_CAL: return "не откалибрована установка платы ('o' в консоли; плата из BF — нуль 'a' на ровном)";
            case ARM_BLOCK_MOTOR_PROTOCOL: return "протокол моторов не выбран или не работает";
            case ARM_BLOCK_SWITCH: return "ARM был включён заранее — выключить и включить снова";
            case ARM_BLOCK_MOVING: return "коптер качают — поставить на землю";
            case ARM_BLOCK_BATTERY: return "батарея разряжена";
            case ARM_BLOCK_MOTOR_TEST: return "идёт проверка моторов";
            case ARM_BLOCK_NO_ARM_SWITCH: return "ARM не назначен ни на один тумблер (aux)";
            case ARM_BLOCK_CONFIG: return "описание коптера не сходится с платой (см. консоль)";
            case ARM_BLOCK_FENCE: return "вне поля геозабора или в запретной зоне — перенести коптер внутрь";
        }
        return "?";
    }

    // Слово для очков (P10): что сделать, а не код BF; до 11 знаков, шрифт OSD.
    // Те же слова — в приложении рядом с кнопкой «исправить» (core/armWhy.ts).
    inline const char* armBlockWord(uint32_t bit)
    {
        switch (bit)
        {
            case ARM_BLOCK_IMU: return "NO GYRO";
            case ARM_BLOCK_FAILSAFE: return "WAIT LINK";
            case ARM_BLOCK_NO_LINK: return "NO RADIO";
            case ARM_BLOCK_NOT_DISARMED: return "ARM SW OFF";
            case ARM_BLOCK_FAILSAFE_SWITCH: return "FS SWITCH";
            case ARM_BLOCK_RUNAWAY: return "RUNAWAY";
            case ARM_BLOCK_CRASH: return "CRASH";
            case ARM_BLOCK_THROTTLE: return "THR DOWN";
            case ARM_BLOCK_TILT: return "NOT LEVEL";
            case ARM_BLOCK_BOOT_GRACE: return "WAIT";
            case ARM_BLOCK_NO_PREARM: return "PREARM";
            case ARM_BLOCK_LOAD: return "CPU LOAD";
            case ARM_BLOCK_SETTLING: return "WAIT GYRO";
            case ARM_BLOCK_CLI: return "CLI OPEN";
            case ARM_BLOCK_CMS: return "MENU OPEN";
            case ARM_BLOCK_BST: return "BST";
            case ARM_BLOCK_MSP: return "NOT SAVED";
            case ARM_BLOCK_PARALYZE: return "PARALYZE";
            case ARM_BLOCK_GPS: return "NO GPS";
            case ARM_BLOCK_RESCUE_SWITCH: return "RESCUE SW";
            case ARM_BLOCK_RPM_FILTER: return "NO RPM";
            case ARM_BLOCK_REBOOT: return "REBOOT";
            case ARM_BLOCK_DSHOT_BITBANG: return "DSHOT FAIL";
            case ARM_BLOCK_NO_ACC_CAL: return "CALIBRATE";
            case ARM_BLOCK_MOTOR_PROTOCOL: return "MOTOR PROTO";
            case ARM_BLOCK_SWITCH: return "ARM SW OFF";
            case ARM_BLOCK_MOVING: return "MOVING";
            case ARM_BLOCK_BATTERY: return "LOW BATT";
            case ARM_BLOCK_MOTOR_TEST: return "MOTOR TEST";
            case ARM_BLOCK_NO_ARM_SWITCH: return "NO ARM SW";
            case ARM_BLOCK_CONFIG: return "CONFIG";
            case ARM_BLOCK_FENCE: return "FENCE";
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
            case ARM_BLOCK_FENCE: return "FENCE";
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
        GpsInput gps;                // GPS и курс — для возврата домой (copter/GpsRescue.h)
        // обороты моторов по телеметрии DShot (dshot_bidir), Гц, в порядке смесителя;
        // false — режекторы по модели (команда мотора -> обороты)
        bool motorHzOk = false;
        float motorHz[MAX_MOTORS] = {};
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
        bool acroTrainer = false;      // режим ACRO TRAINER: в ACRO наклон не больше acro_trainer_angle_limit
        bool launchControl = false;    // режим LAUNCH CONTROL: на земле ждать рывка газом (copter/LaunchControl.h)
        bool gpsRescue = false;        // режим GPS RESCUE: возврат домой с тумблера
        bool failsafeSwitch = false;   // режим FAILSAFE: как потеря связи
        bool flipOverAfterCrash = false;   // режим FLIP OVER AFTER CRASH: ARM — turtle mode (copter/Turtle.h)
        bool chirp = false;            // режим CHIRP: качание частоты по одной оси (copter/Chirp.h)
        bool prearm = true;            // PREARM включён или его нет в таблице
        bool paralyze = false;         // режим PARALYZE на тумблере (защёлкивается в контроллере)
        bool headfree = false;         // режим HEADFREE: стики — от курса при ARM
        bool headAdjust = false;       // режим HEADADJ: курс для HEADFREE — нынешний
        bool fpvAngleMix = false;      // режим FPV ANGLE MIX: крен и рысканье — в осях камеры
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
        bool reversed = false;          // turtle mode: регуляторы крутят наоборот (команда DShot)
    };

    struct CopterSettings
    {
        bool airmode = true;
        float airmodeStartThrottle = 0.25f;   // airmode_start_throttle_percent: airmode — после газа выше (с ARM)
        MixerType mixerType = MixerType::LEGACY;   // mixer_type
        bool voltageCompensation = true;
        bool rpmNotches = true;
        RcSmoothingSettings rcSmoothing;   // rc_smoothing_*, deadband (copter/RcSmoothing.h)
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
        // acc_trim_roll / acc_trim_pitch: «ровно» для ANGLE/HORIZON — столько
        // вправо и нос вниз (как в BF: летит вперёд — тангаж назад)
        float levelTrimRollDeg = 0.0f;
        float levelTrimPitchDeg = 0.0f;
        // attitude_filter: MEKF (оценивает и смещение гироскопа) или Махони (copter/Attitude.h)
        bool attitudeEkf = true;
        // attitude_drag: сопротивление винтов для модели фильтра, 1/с; 0 — не
        // измерено (фильтр берёт 0.3 и учит по GPS в полёте)
        float dragCoef = 0.0f;
        float accLpfHz = 20.0f;            // acc_lpf_hz: ФНЧ акселерометра (PT2), 0 — без фильтра
        ImuAlignSettings imuAlign;         // align_board_*, acc_calibration, gyro_offset_yaw (copter/ImuAlign.h)
        float dcmKp = 0.5f;                // imu_dcm_kp × 0.0001: Махони в полёте, 1/с (на земле ×10)
        float dcmKi = 0.0f;                // imu_dcm_ki × 0.0001: интеграл Махони
        // yaw_spin_recovery / yaw_spin_threshold: раскрутка по рысканью (069)
        YawSpinMode yawSpin = YawSpinMode::AUTO;
        float yawSpinThresholdDps = 1950.0f;
        float autoDisarmS = 5.0f;          // auto_disarm_delay: на земле с газом внизу столько — DISARM; 0 — нет
        // runaway_takeoff_*: «убегает» на взлёте — DISARM; проверка гаснет,
        // когда коптер столько продержался в воздухе спокойно
        bool runawayPrevention = true;
        float runawayDeactivateS = 0.5f;
        float runawayDeactivateThrottle = 0.2f;
        bool crashDetection = true;
        bool turtleAvailable = false;      // регуляторы DShot: реверс командой есть — turtle mode работает
        TurtleSettings turtle;             // crashflip_*
        AltitudeController::Settings alt;
        GpsRescueSettings rescue;          // gps_rescue_*
        DeadReckonSettings deadReck;       // deadreck_*: домой вслепую без GPS (copter/DeadReckonHome.h)
        PosHoldSettings posHold;           // pos_hold_*, autopilot_position_* (copter/PositionHold.h)
        uint8_t posHoldMinSats = 8;        // gps_minimum_sats: спутников меньше — позицию не держать
        float gpsDelayS = 0.1f;            // фикс GPS старше на столько (copter/PositionEstimator.h)
        bool gpsSpoofDetect = true;        // gps_spoof_detect: подмена GPS — GPS из навигации вон (copter/GpsIntegrity.h)
        float hoverThrottle = 0.0f;        // autopilot_hover_throttle: газ висения до обучения; 0 — из описания коптера
        TrainingSettings training;         // training_* (copter/TrainingLimits.h)
        GeofenceSettings geofence;         // geofence_*: граница, за которую не вылетит (copter/Geofence.h)
        ChirpSettings chirp;               // chirp_* (copter/Chirp.h)
    };

    struct CopterStatus
    {
        CopterState state = CopterState::DISARMED;
        CopterMode mode = CopterMode::ANGLE;
        uint32_t armBlockers = 0;
        float roll = 0, pitch = 0, yaw = 0;     // °
        float rates[3] = {};                    // °/с после фильтров
        float gyroIn[3] = {};                   // °/с в осях рамы, до фильтров (вкладка «Датчики»)
        float accIn[3] = {};                    // g в осях рамы, после нуля акселерометра
        bool imuOk = false;                     // гироскоп отвечает
        float setpoint[3] = {};                 // °/с
        float altitude = 0, climbRate = 0;      // м, м/с (от точки ARM)
        float altTarget = 0;
        float throttle = 0;                     // доля тяги на входе смесителя
        float hoverLearned = 0;
        float volts = 0;
        float rotationHz = 0;                   // средние обороты моторов, Гц
        bool rpmTelemetry = false;              // обороты — по DShot, а не по модели
        float rpmModelScale = 1;                // поправка модели оборотов по DShot (090): настоящие / модель
        float dynIdleIncrease = 0;              // dynamic idle: добавка к холостым сейчас, доля команды
        float dynIdleSlowestHz = 0;             // dynamic idle: самый медленный мотор, об/с (0 — нет телеметрии)
        bool airborne = false;
        bool saturated = false;
        bool crashed = false;
        bool crashRecovery = false;    // удар в воздухе — выравнивается (crash_recovery)
        bool yawSpin = false;          // раскрутка по рысканью — останавливается (yaw_spin_recovery)
        bool gyroSaturated = false;    // гироскоп в упоре сейчас или недавно (3 с): горизонт под вопросом
        float rcFrameHz = 0;           // частота кадров приёмника (измерена)
        float rcSetpointCutoffHz = 0;  // сглаживание стиков: срез PT3 (0 — выключено)
        float rcThrottleCutoffHz = 0;  // и газа
        bool runaway = false;          // DISARM защитой runaway takeoff (до выключения тумблера)
        bool runawayCheck = false;     // проверка runaway takeoff ещё работает (после ARM, до уверенного взлёта)
        bool headfree = false;         // HEADFREE действует (режим с горизонтом, не автомат)
        float headfreeRefDeg = 0.0f;   // курс, от которого считаются стики HEADFREE
        bool fpvMix = false;           // FPV ANGLE MIX действует (ACRO/HORIZON, fpv_mix_degrees > 0)
        bool turtle = false;           // заармлен в turtle mode: моторы наоборот, стик — какой стороной переворачиваться
        LaunchState launch = LaunchState::OFF;   // launch control: ждёт рывка (ACTIVE) или уже стартовал
        uint8_t failsafeStage = 0;     // 0 — нет, 1 — ровно и держит высоту, 2 — процедура
        RescuePhase rescuePhase = RescuePhase::IDLE;   // возврат домой: этап
        RescueFail rescueFail = RescueFail::NONE;      // и почему прерван
        float rescueHeadingFix = 0.0f;                  // без компаса: поправка курса гироскопа по GPS, °
        // домой вслепую (copter/DeadReckonHome.h): этап, место по своим датчикам и его неуверенность
        DeadReckonPhase deadReckPhase = DeadReckonPhase::IDLE;
        float deadReckN = 0.0f, deadReckE = 0.0f, deadReckErrM = 0.0f;
        uint8_t trainingFlags = 0;     // учебные ограничения: биты TrainingFlag (что сейчас держит)
        uint8_t fenceFlags = 0;        // геозабор: биты GeofenceFlag
        float fenceMarginM = 0.0f;     // до края забора, м (< 0 — за ним; без забора — 0)
        // положение по GPS и акселерометру (copter/PositionEstimator.h): от дома, м и м/с
        bool posValid = false;
        float posN = 0.0f, posE = 0.0f, velN = 0.0f, velE = 0.0f;
        float heading = 0.0f;                                   // курс носа, 0…360 (без источника — рысканье гироскопа)
        HeadingSource headingSource = HeadingSource::NONE;      // компас, полёт (GPS) или ничего
        bool magDisagrees = false;                              // компас расходится с полётом — не верим
        GpsSpoof gpsSpoof = GpsSpoof::NONE;                     // подмена GPS (copter/GpsIntegrity.h): GPS выключен из навигации
        uint8_t gpsTrust = 100;                                 // доверие GPS, 0…100
        PosHoldPhase posHoldPhase = PosHoldPhase::OFF;          // удержание позиции: что делает
        PosHoldFail posHoldFail = PosHoldFail::NONE;            // и почему не держит (режим включён)
        float posTargetN = 0.0f, posTargetE = 0.0f;             // точка удержания
        // chirp — качание частоты для измерения коптера (copter/Chirp.h)
        bool chirpActive = false;
        uint8_t chirpAxis = 0;                  // ось свипа; пока не идёт — следующая
        uint8_t chirpRun = 0;                   // номер свипа с включения
        float chirpHz = 0.0f;
        float chirpExcitation = 0.0f;           // добавка к команде оси, доля хода смесителя
        float chirpCommand = 0.0f;              // команда этой оси на входе смесителя вместе с ней
        ChirpEnd chirpEnd = ChirpEnd::NONE;     // чем кончился прошлый свип
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
            mixer.setType(s.mixerType);
            mixer.setEzLanding(t.ezLanding);
            launch.configure(t.launch);
            gyroFilter.configure(t.loopHz, t.gyroLpfHz, t.notchQ, t.prop.prop.blades, t.minNotchHz);
            axes[0].configure(t.roll, t.dtermLpfHz, 0.0f, t.iLimit, dt);
            axes[1].configure(t.pitch, t.dtermLpfHz, 0.0f, t.iLimit, dt);
            axes[2].configure(t.yaw, t.dtermLpfHz, t.yawLpfHz, t.iLimit, dt);
            configureFilterChain(t.filters);
            for (uint8_t a = 0; a < 3; ++a)
            {
                axes[a].setExtras(t.pid, a);
                axes[a].setMaxRate(t.rates[a].apply(1.0f));
                absCtl[a].configure(t.pid.absCutoffHz, dt);
            }
            rcSmooth.configure(s.rcSmoothing, dt);
            throttleHpLpf.configure(t.pid.antiGravityHz, dt);
            throttleBoostLpf.configure(t.pid.throttleBoostHz, dt);
            configureLevelFilters(t.pid);
            for (auto& f : accelLpf) f.configure(s.accLpfHz, dt);
            motionLpf.configure(3.0f, dt);
            voltLpf.configure(2.0f, dt);
            attitude.configure(2.0f, 0.2f);
            attitude.setMahonyGains(s.dcmKp, s.dcmKp * 10.0f, s.dcmKi);
            imuAlign.configure(s.imuAlign);
            attitude.reset();
            attitude.setEkf(s.attitudeEkf);
            attitude.setDragCoef(s.dragCoef);
            vertical.configure(2.5f);
            hover = s.hoverThrottle > 0.01f ? s.hoverThrottle : t.hoverThrust;
            altCtl.configure(s.alt, hover);
            rescue.configure(s.rescue);
            deadReck.configure(s.deadReck, s.rescue);
            training.configure(s.training);
            fence.configure(s.geofence);
            position.configure(s.gpsDelayS);
            position.setMinSats(s.posHoldMinSats);
            gpsIntegrity.configure(s.gpsSpoofDetect);
            position.reset();
            posHold.configure(s.posHold, dt);
            posHold.stop();
            posHold.clearFail();
            chirp.configure(s.chirp, t.loopHz);
            chirpAxisNext = 0;
            chirpSwitch = true;
            posOkTimer = 0.0f;
            posLastFail = PosHoldFail::NO_GPS;
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
        // Потолок моторов от ограничения тока (copter/EscGuard.h): 1 — нет.
        void setOutputScale(float s) { outputScale = fminf(1.0f, fmaxf(0.0f, s)); }

        void applyProfile(const CopterTune& t)
        {
            // фильтры — только если поменялись: перенастройка сбрасывает их состояние
            if (!sameFilters(t.filters, tune.filters))
            {
                tune.filters = t.filters;
                tune.gyroLpfHz = t.gyroLpfHz;
                tune.dtermLpfHz = t.dtermLpfHz;
                if (t.filters.valid) configureFilterChain(t.filters);
                else
                {
                    gyroFilter.configure(tune.loopHz, t.gyroLpfHz, tune.notchQ, tune.prop.prop.blades, tune.minNotchHz);
                    for (uint8_t a = 0; a < 3; ++a)
                    {
                        FilterSpec legacy;
                        legacy.dterm1 = { LpfType::PT1, t.dtermLpfHz };
                        axes[a].configureDterm(legacy);
                    }
                }
            }
            tune.roll = t.roll;
            tune.pitch = t.pitch;
            tune.yaw = t.yaw;
            tune.gainsFromModel = t.gainsFromModel;
            tune.iLimit = t.iLimit;
            tune.pid = t.pid;
            tune.yawLpfHz = t.yawLpfHz;
            tune.thrustLinear = t.thrustLinear;
            tune.motorOutputLimit = t.motorOutputLimit;
            tune.ezLanding = t.ezLanding;
            mixer.setEzLanding(t.ezLanding);
            tune.launch = t.launch;
            launch.configure(t.launch);
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
            tune.levelGain = t.levelGain;
            tune.maxAngleDeg = t.maxAngleDeg;
            configureLevelFilters(t.pid);
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
            for (float& t : testThrust) t = 0.0f;
            return setMotorTest(index, thrust, seconds);
        }

        // Ползунок мотора (вкладка «Моторы» приложения, `motor` в CLI,
        // MSP_SET_MOTOR): остальные моторы — как были; MOTOR_TEST_ALL — все.
        // Каждый вызов продлевает проверку на seconds: нового нет — стоп.
        static constexpr uint8_t MOTOR_TEST_ALL = 0xFF;
        static constexpr float MOTOR_TEST_MAX = 0.3f;
        bool setMotorTest(uint8_t index, float thrust, float seconds)
        {
            if (status.state != CopterState::DISARMED) return false;
            if (mp) mp->music.stop();
            if (index != MOTOR_TEST_ALL && index >= tune.frame.count) return false;
            const float v = fminf(MOTOR_TEST_MAX, fmaxf(0.0f, thrust));
            for (uint8_t i = 0; i < MAX_MOTORS; ++i)
            {
                if (index == MOTOR_TEST_ALL || i == index) testThrust[i] = i < tune.frame.count ? v : 0.0f;
            }
            bool any = false;
            for (float t : testThrust) any = any || t > 0.0f;
            testLeft = any ? fminf(5.0f, fmaxf(0.0f, seconds)) : 0.0f;
            return true;
        }

        void stopMotorTest()
        {
            testLeft = 0.0f;
            for (float& t : testThrust) t = 0.0f;
            if (mp) mp->music.stop();
        }
        bool motorTestActive() const { return testLeft > 0.0f || musicActive(); }
        float motorTestValue(uint8_t index) const
        {
            if (musicActive())
            {
                const MotorMusic& m = mp->music;
                return index < m.count() && m.targetHz(index) >= 0.0f ? m.command(index) : 0.0f;
            }
            return testLeft > 0.0f && index < MAX_MOTORS ? testThrust[index] : 0.0f;
        }

        // Музыка моторами по оборотам (copter/MotorMusic.h, 176c): только без
        // ARM и без винтов. mult — звук / обороты, 0 — пар полюсов (poles / 2).
        bool startMusic(uint8_t poles, float mult)
        {
            if (!mp || status.state != CopterState::DISARMED) return false;
            testLeft = 0.0f;
            for (float& t : testThrust) t = 0.0f;
            const float volts = status.volts > 1.0f ? status.volts : tune.volts;
            const float hzPerCommand = tune.prop.kvRad * volts / (2.0f * PI_F);
            if (!(mult > 0.0f)) mult = poles >= 2 ? poles * 0.5f : 7.0f;
            return mp->music.start(tune.frame.count, hzPerCommand, mult);
        }
        void stopMusic()
        {
            if (!mp) return;
            mp->track.stop();
            mp->music.stop();
        }

        // Трек на плате (copter/MotorTrack.h): диапазон — от верхней ноты по
        // KV и напряжению вниз на 2,2 октавы (ниже — холостые регулятора).
        bool playTrack(MotorTrack::Fetch fetch, void* ctx, uint16_t chunk, uint16_t size, uint8_t poles, bool loop)
        {
            if (!startMusic(poles, 0.0f)) return false;
            const float hi = fminf(1400.0f, mp->music.maxSoundHz() * 0.9f);
            if (!mp->track.start(fetch, ctx, chunk, size, fmaxf(150.0f, hi / 4.5f), hi, loop))
            {
                mp->music.stop();
                return false;
            }
            return true;
        }
        // nullptr — музыки на этой плате нет (F405: ещё не выделена в CCM).
        const MotorTrack* motorTrack() const { return mp ? &mp->track : nullptr; }
        bool musicActive() const { return mp && mp->music.active(); }
        // F405: музыка — в CCM (main_copter), в основных 128 КБ места нет.
        void attachMusic(MotorMusicParts* parts) { mp = parts; }
        bool musicNote(uint8_t motor, float soundHz)
        {
            return mp && status.state == CopterState::DISARMED && mp->music.note(motor, soundHz);
        }
        const MotorMusic* motorMusic() const { return mp ? &mp->music : nullptr; }

        // Один цикл стабилизации.
        CopterOutputs update(const CopterSensors& sensIn, const CopterRc& rcIn)
        {
            // подмена GPS: сырой фикс против акселерометра, ветра, барометра —
            // до того, как фикс увидят навигация и фильтр (ускорение и ветер — с
            // прошлого такта)
            {
                GpsIntegrityInput gi;
                gi.fresh = sensIn.gps.fresh;
                gi.fix = sensIn.gps.fix;
                gi.latitude = sensIn.gps.latitude;
                gi.longitude = sensIn.gps.longitude;
                gi.velN = sensIn.gps.velN;
                gi.velE = sensIn.gps.velE;
                gi.altitudeM = sensIn.gps.altitudeM;
                gi.receiverSpoof = sensIn.gps.receiverSpoof;
                gi.baroAltM = sensIn.baroAltM;
                gi.baroOk = sensIn.baroOk;
                // сырое ускорение, без курса и выученного по GPS смещения: подмена не должна учить сверку
                gi.accelOk = attitude.isInitialized();
                gi.accelX = earthAccelX;
                gi.accelY = earthAccelY;
                gi.windOk = status.airborne && attitude.dragSigma() <= GPS_WIND_SIGMA;
                gi.windN = attitude.windNorth();
                gi.windE = attitude.windEast();
                gi.armed = status.state != CopterState::DISARMED;
                gi.pathRatio = deadReck.pathRatio();   // с прошлого такта
                gpsIntegrity.update(gi, dt);
                status.gpsSpoof = gpsIntegrity.reason();
                status.gpsTrust = gpsIntegrity.trust();
            }
            // подмена GPS (copter/GpsIntegrity.h): навигация, возврат и фильтр
            // ориентации GPS не видят — как будто фикса нет
            const bool gpsMasked = gpsIntegrity.spoofed();
            CopterSensors maskedSens;   // на стеке задачи (на F405 — в CCM), не в общей ОЗУ
            if (gpsMasked)
            {
                maskedSens = sensIn;
                maskedSens.gps.fix = false;
                maskedSens.gps.home = false;
                maskedSens.gps.sats = 0;
                maskedSens.gps.velN = maskedSens.gps.velE = 0.0f;
            }
            const CopterSensors& sens = gpsMasked ? maskedSens : sensIn;
            CopterOutputs out;
            out.count = tune.frame.count;
            uptime += dt;
            ++status.loops;

            // ---- датчики ----
            // плата -> рама: align_board_*, нуль акселерометра, gyro_offset_yaw
            float gyroIn[3], accIn[3];
            imuAlign.apply(sens.gyro, sens.accel, gyroIn, accIn);
            for (int i = 0; i < 3; ++i)
            {
                status.gyroIn[i] = gyroIn[i];
                status.accIn[i] = accIn[i];
            }
            status.imuOk = sens.imuOk;
            float rates[3] = { gyroIn[0], gyroIn[1], gyroIn[2] };
            if (sens.batteryVolts > 1.0f)
            {
                if (status.volts <= 1.0f) voltLpf.reset(sens.batteryVolts);
                status.volts = voltLpf.apply(sens.batteryVolts);
            }
            // обороты: по телеметрии DShot — каждого мотора; иначе по модели —
            // средние: обороты не меняются быстрее, чем мотор успевает разогнать
            // винт: оценка идёт за командой с постоянной времени мотора —
            // режекторы едут плавно, а не прыгают вслед за каждым циклом смесителя
            const uint8_t motors = tune.frame.count < MAX_MOTORS ? tune.frame.count : MAX_MOTORS;
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
            if (sens.motorHzOk)
            {
                float sum = 0.0f;
                for (uint8_t i = 0; i < motors; ++i) sum += sens.motorHz[i];
                rotHz = motors ? sum / motors : 0.0f;
                // модель учится у телеметрии (090): винт не тот, KV, износ —
                // отношение настоящих оборотов к модели, медленно (2 с).
                // Пропала телеметрия — режекторы едут по поправленной модели,
                // а не прыгают на её ошибку
                if (spinning() && rotEstimate > RPM_LEARN_MIN_HZ && rotHz > RPM_LEARN_MIN_HZ)
                {
                    const float ratio = fminf(RPM_SCALE_MAX, fmaxf(RPM_SCALE_MIN, rotHz / rotEstimate));
                    status.rpmModelScale += (ratio - status.rpmModelScale) * dt / (RPM_LEARN_TAU_S + dt);
                }
            }
            else if (spinning())
            {
                rotHz *= status.rpmModelScale;
            }
            status.rotationHz = rotHz;
            status.rpmTelemetry = sens.motorHzOk;
            // dynamic idle — только по настоящим оборотам и пока моторы крутятся
            status.dynIdleIncrease = dynIdle.update(tune.dynIdle, sens.motorHz, motors, sens.motorHzOk && spinning(),
                                                    airmodeStarted, dt);
            status.dynIdleSlowestHz = dynIdle.slowestHz();
            // по телеметрии — режекторы на каждый мотор (063); по модели — один
            // набор на средние обороты: каждый режектор стоит фазы у полосы
            // контура, а модель моторы между собой всё равно не различит
            if (set.rpmNotches)
            {
                if (sens.motorHzOk) gyroFilter.setMotorHz(sens.motorHz, motors);
                else gyroFilter.setRotationHz(rotHz);
            }
            // динамические ФНЧ гироскопа и D — по газу (как в BF)
            gyroFilter.setThrottle(status.throttle);
            for (uint8_t a = 0; a < 3; ++a) axes[a].setThrottle(status.throttle);
            gyroFilter.apply(rates);
            // акселерометр на коптере трясётся на 1–2 g от моторов: для
            // горизонта и высоты нужно только медленное
            float accel[3];
            for (int a = 0; a < 3; ++a) accel[a] = accelLpf[a].apply(accIn[a]);
            if (!attitude.isInitialized())
            {
                for (int a = 0; a < 3; ++a)
                {
                    accelLpf[a].reset(accIn[a]);
                    accel[a] = accIn[a];
                }
            }
            for (int a = 0; a < 3; ++a) accelG[a] = accel[a];
            attitude.setGrounded(!status.airborne);
            gpsAccel.update(sens.gps.fresh, sens.gps.fix, sens.gps.velN, sens.gps.velE, dt);
            attitude.setGroundAccel(gpsAccel.accel());
            // курс известен — ускорение по GPS вычитается из акселерометра
            // (вираж, разгон: «верх» наклонён на него по праву)
            if (gpsAccel.accel() >= 0.0f && position.headingSource() != HeadingSource::NONE)
                attitude.setGroundAccelVector(gpsAccel.accelN(), gpsAccel.accelE(), position.headingDeg() - attitude.yawDeg());
            else
                attitude.clearGroundAccelVector();
            // новый фикс при известном курсе — скорость фильтру: по ней и по
            // акселерометру он учит сопротивление винтов и ветер. Только в
            // настоящем полёте: «в воздухе» включается уже на раскрутке, а пока
            // коптер стоит, акселерометр показывает опору, а не сопротивление —
            // фильтр «узнал» бы, что ветра нет
            if (sens.gps.fresh && sens.gps.fix && position.headingSource() != HeadingSource::NONE && status.airborne &&
                (vertical.altitude() > DRAG_MIN_HEIGHT_M ||
                 sens.gps.velN * sens.gps.velN + sens.gps.velE * sens.gps.velE > DRAG_MIN_SPEED_MS * DRAG_MIN_SPEED_MS))
                attitude.setDragVelocity(sens.gps.velN, sens.gps.velE, -vertical.climbRate(), position.headingDeg() - attitude.yawDeg());
            // компас, которому есть вера, — фильтру: рысканье не уплывает, и
            // находится смещение гироскопа по нему (курс минус рысканье — как
            // их считали в прошлом цикле: status.yaw)
            if (sens.gps.headingTrusted && !status.magDisagrees) attitude.setCompass(sens.gps.headingDeg - status.yaw);
            else attitude.clearCompass();
            if (sens.imuOk)
            {
                // упор — по сырому отсчёту чипа (до поворота платы: поворот
                // смешал бы оси и спрятал предел)
                bool sat = false;
                for (int a = 0; a < 3; ++a) sat = sat || fabsf(sens.gyro[a]) >= GYRO_SATURATED_DPS;
                attitude.setGyroSaturated(sat);
                attitude.update(gyroIn, accel, dt);
            }
            status.gyroSaturated = attitude.gyroSaturatedRecently();
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
            // положение по горизонтали: акселерометр в осях «от включения» + GPS
            {
                const Vec3 f = attitude.isInitialized() ? attitude.toEarth({ accel[0], -accel[1], -accel[2] }) : Vec3();
                earthAccelX = f.x * 9.80665f;
                earthAccelY = f.y * 9.80665f;
                position.update(f.x * 9.80665f, f.y * 9.80665f, status.yaw, sens.gps, status.airborne, dt);
                status.posValid = position.valid();
                status.posN = position.north();
                status.posE = position.east();
                status.velN = position.velN();
                status.velE = position.velE();
                status.heading = position.headingDeg();
                status.headingSource = position.headingSource();
                // компас отброшен — отказ удержания «унесло» был из-за него: можно пробовать снова
                if (position.magDisagrees() && !status.magDisagrees) posHold.clearFail();
                status.magDisagrees = position.magDisagrees();
            }

            // качают ли коптер (проверка перед ARM), крутится ли (посадка, переворот)
            const float motion = fabsf(rates[0]) + fabsf(rates[1]) + fabsf(rates[2]);
            motionLpf.apply(motion);
            motionTimer = motion > 15.0f ? 0.0f : motionTimer + dt;

            // след для возврата (P45 а): только достоверный фикс и в воздухе
            rescue.track(sens.gps, vertical.altitude(), dt,
                         status.state != CopterState::DISARMED && status.airborne && !gpsIntegrity.spoofed());
            // поле геозабора: где коптер (и без ARM — ARM только внутри)
            if (sens.gps.fresh && sens.gps.fix && !gpsIntegrity.spoofed())
                fence.updatePosition(sens.gps.latitude, sens.gps.longitude, sens.gps.velN, sens.gps.velE);
            // путь по своим датчикам — на случай, если GPS подменят или заглушат
            {
                DeadReckonInput di;
                di.gpsGood = sens.gps.fresh && rescue.ready() == RescueFail::NONE;
                di.gpsN = sens.gps.northM;
                di.gpsE = sens.gps.eastM;
                di.modelOk = attitude.ekfEnabled() && attitude.isInitialized() && status.airborne;
                di.airX = attitude.airVelX();
                di.airY = attitude.airVelY();
                di.headingOk = position.headingSource() != HeadingSource::NONE;
                di.headingOffsetDeg = position.headingDeg() - attitude.yawDeg();
                di.windN = attitude.windNorth();
                di.windE = attitude.windEast();
                di.windSigma = attitude.windSigma();
                di.dragCoef = attitude.dragCoef();
                di.dragSigma = attitude.dragSigma();
                deadReck.track(di, dt);
            }

            // ---- связь, ARM, failsafe ----
            CopterRc rc = rcIn;
            trackFrames(rc.frame);
            const bool rcValid = handleLink(rc);
            handleArming(rc, sens, rcValid);
            if (status.state == CopterState::ARMED && !status.turtle && !launch.active()) autoDisarm(rc);
            if (status.state != CopterState::DISARMED && !status.turtle) detectCrash();

            if (status.state == CopterState::DISARMED)
            {
                resetFlight(sens);
                if (musicActive())
                {
                    if (mp->track.isPlaying()) mp->track.tick(dt, mp->music);
                    float cmd[MAX_MOTORS];
                    if (mp->music.update(dt, sens.motorHz, sens.motorHzOk, cmd))
                    {
                        for (uint8_t i = 0; i < out.count && i < MAX_MOTORS; ++i) out.motor[i] = cmd[i];
                        out.spinning = true;
                    }
                }
                else if (testLeft > 0.0f)
                {
                    testLeft -= dt;
                    // остальные — «стоп»: 0 у DShot — минимальный газ (48), регулятор
                    // крутил бы их на холостых (стенд 2026-11-26: крутились все четыре)
                    for (uint8_t i = 0; i < out.count && i < MAX_MOTORS; ++i)
                        out.motor[i] = testThrust[i] > 0.0f ? tune.motorCommand(testThrust[i]) : MOTOR_STOP;
                    out.spinning = true;
                    if (testLeft <= 0.0f) stopMotorTest();
                }
                avgThrust = 0.0f;
                status.throttle = 0.0f;
                return out;
            }
            if (status.turtle) return turtleOutputs(sens, rc);

            // ---- учебные ограничения (copter/TrainingLimits.h) ----
            training.update(vertical.altitude(), vertical.baroUsed(), rescue.distanceHome(), rescue.outwardSpeed(),
                            rescue.ready() == RescueFail::NONE, rescue.groundSpeed(), rescue.gpsOk(), dt);
            status.trainingFlags = training.flags();
            // ---- геозабор (copter/Geofence.h): в любом режиме, и в failsafe ----
            fence.update(vertical.altitude(), vertical.baroUsed(), rescue.distanceHome(), rescue.outwardSpeed(),
                         rescue.groundSpeed(),
                         !gpsMasked && rescue.ready() == RescueFail::NONE && rescue.fixAgeS() < Geofence::GPS_STALE_S,
                         status.airborne, dt);

            // ---- уставки ----
            const bool failsafe = status.state == CopterState::FAILSAFE;
            // этап 2 с failsafe_throttle — ровно на постоянном газе, как в BF
            const bool fixedThrottle = failsafe && status.failsafeStage >= 2 && set.failsafeThrottle > 0.0f;
            // этап 1 с rxfail газа HOLD — ровно на последнем газе, как в BF
            // возврат домой: этап 2 failsafe с GPS-RESCUE или режим GPS RESCUE в воздухе
            const bool wantRescue = (failsafe && status.failsafeStage >= 2 && set.failsafeProcedure == FailsafeProcedure::GPS_RESCUE) ||
                                    ((rc.gpsRescue || training.wantReturn()) && status.state == CopterState::ARMED &&
                                     status.airborne) ||
                                    (fence.wantReturn() && status.airborne);
            // прямая домой через запретную зону или край поля — по следу (P44 б2)
            // (на старте и на свежем фиксе: стороны зон — до 128 отрезков)
            {
                const bool need = wantRescue || rescue.active();
                if (!need || sens.gps.fresh || !rescue.active()) rescue.setDirectBlocked(need && fence.lineBlocked(-rescue.fixNorth(), -rescue.fixEast()));
            }
            if (wantRescue && !rescue.active()) rescue.start(vertical.altitude(), failsafe, rescueCeiling());
            if (!wantRescue && rescue.active()) rescue.stop();
            if (!wantRescue) deadReck.stop();
            // GPS подменяют — по нему не лететь, даже если проверки выключены
            if (gpsMasked) rescue.fail(RescueFail::GPS_SPOOF);
            RescueCommand rescueCmd;
            // возврат летит по общей оценке положения и курса (P39)
            {
                NavEstimate ne;
                ne.valid = position.valid();
                ne.northM = position.north();
                ne.eastM = position.east();
                ne.velN = position.velN();
                ne.velE = position.velE();
                ne.headingKnown = position.headingSource() != HeadingSource::NONE;
                ne.headingDeg = position.headingDeg();
                rescue.setEstimate(ne);
            }
            if (rescue.active() && !deadReck.active()) rescueCmd = rescue.update(vertical.altitude(), status.yaw, dt);
            // GPS не годится (подмена, глушение) — домой вслепую по своим датчикам
            if (wantRescue && !deadReck.active() && rescue.phase() == RescuePhase::ABORT && gpsFailure(rescue.failure()) &&
                deadReck.available())
            {
                // скачок (точки или высоты) виден сразу — последний честный фикс годится;
                // медленный увод ловят не сразу — точка и ветер за LOOKBACK_S до него
                const GpsSpoof why = gpsIntegrity.reason();
                const bool slow = gpsMasked && why != GpsSpoof::JUMP && why != GpsSpoof::ALTITUDE;
                deadReck.start(slow, rescue.returnAltitude(vertical.altitude(), rescueCeiling()));
                blindGpsBack = 0.0f;
            }
            // GPS вернулся и годен BLIND_GPS_BACK_S подряд (глушили, не подменяли) — снова по GPS
            if (deadReck.active())
            {
                blindGpsBack = !gpsMasked && rescue.ready() == RescueFail::NONE ? blindGpsBack + dt : 0.0f;
                if (blindGpsBack >= BLIND_GPS_BACK_S)
                {
                    deadReck.stop();
                    rescue.start(vertical.altitude(), failsafe, rescueCeiling());
                    if (rescue.active()) rescueCmd = rescue.update(vertical.altitude(), status.yaw, dt);
                }
            }
            const bool blind = deadReck.active();
            if (blind) rescueCmd = deadReck.update(vertical.altitude(), status.yaw, dt);
            bool rescuing = blind || (rescue.active() && rescue.phase() != RescuePhase::ABORT);
            // забор ведёт назад, а вернуть нечем (нет GPS и модели для полёта вслепую) — сесть
            if (fence.wantReturn() && !rescuing) fence.forceLand();
            const bool fenceLanding = fence.wantLand() && !rescuing && status.airborne;
            RescueCommand fenceLandCmd;
            if (fenceLanding)
            {
                fenceLandCmd.climbRate = -set.failsafeDescent;
                rescueCmd = fenceLandCmd;
                rescuing = true;   // ровно и вниз, стики не слушаются
            }
            status.fenceFlags = fence.flags();
            status.fenceMarginM = fence.marginM();
            status.deadReckPhase = deadReck.phase();
            status.deadReckN = deadReck.northM();
            status.deadReckE = deadReck.eastM();
            status.deadReckErrM = deadReck.errorM();
            status.rescuePhase = rescue.phase();
            status.rescueFail = rescue.failure();
            status.rescueHeadingFix = rescue.headingCorrection();
            CopterMode mode = rescuing ? CopterMode::ALT_HOLD
                              : failsafe ? (fixedThrottle || heldThrottle ? CopterMode::ANGLE : CopterMode::ALT_HOLD)
                                         : rc.mode;

            // ---- удержание позиции (copter/PositionHold.h) ----
            // с тумблера (POS HOLD) и в failsafe, пока коптер сам висит или
            // садится: с GPS он стоит над точкой, а не плывёт по ветру
            const bool pilotPos = mode == CopterMode::POS_HOLD;
            const bool failsafePos = failsafe && !rescuing && mode == CopterMode::ALT_HOLD && rc.roll == 0.0f &&
                                     rc.pitch == 0.0f && vertical.altitude() > POS_GROUND_M;
            const PosHoldFail posCheck = posHold.check(position);
            if (posCheck != PosHoldFail::NONE)
            {
                posOkTimer = 0.0f;
                posLastFail = posCheck;
            }
            else if (posOkTimer < POS_READY_S)
            {
                posOkTimer += dt;
            }
            // годится — когда GPS и курс в порядке POS_READY_S подряд (после отказа — тоже)
            const PosHoldFail posFail = posOkTimer >= POS_READY_S ? PosHoldFail::NONE : posLastFail;
            if (pilotPos || failsafePos)
            {
                if (posFail == PosHoldFail::NONE && status.airborne)
                {
                    if (!posHold.active()) posHold.start(position, status.roll, status.pitch);
                }
                else
                {
                    posHold.stop();
                }
                // держать нечем — ровно и высота (ALT_HOLD), пилоту — предупреждение
                if (pilotPos && posFail != PosHoldFail::NONE) mode = CopterMode::ALT_HOLD;
                if (failsafePos && posHold.active()) mode = CopterMode::POS_HOLD;
            }
            else
            {
                posHold.stop();
                posHold.clearFail();
            }
            status.posHoldFail = pilotPos ? posFail : PosHoldFail::NONE;

            if (altitudeMode(mode) && !altitudeMode(status.mode))
            {
                altCtl.reset(vertical.altitude(), status.throttle);
            }
            status.mode = mode;

            rcSmooth.track(framePeriod);
            float sticks[4] = { rc.roll, rc.pitch, rc.yaw, rc.throttle };
            rcSmooth.apply(sticks);
            // HEADFREE (BF): в ACRO стик — скорость вращения, поворачивать её
            // по курсу бессмысленно — только в режимах с горизонтом
            if (rc.headAdjust) status.headfreeRefDeg = attitude.yawDeg();
            status.headfree = rc.headfree && mode != CopterMode::ACRO && !rescuing && !failsafe && !status.turtle;
            if (status.headfree) headfreeRotate(sticks[0], sticks[1], attitude.yawDeg() - status.headfreeRefDeg);
            // FPV ANGLE MIX: только где стик — скорость (в ANGLE крен — угол), не с HEADFREE и не автоматом
            status.fpvMix = rc.fpvAngleMix && tune.fpvMixDeg > 0.0f && (mode == CopterMode::ACRO || mode == CopterMode::HORIZON) &&
                            !status.headfree && !rescuing && !failsafe && !status.turtle;
            status.rcFrameHz = rcSmooth.frameRateHz();
            status.rcSetpointCutoffHz = rcSmooth.setpointCutoffHz();
            status.rcThrottleCutoffHz = rcSmooth.throttleCutoffHz();
            float sp[3], stickSp[3];
            // удержание позиции: наклон задаёт автомат, рысканье — стик
            RescueCommand posCmd;
            if (posHold.active())
            {
                const float speedCap = set.training.enabled ? set.training.maxSpeedMs : 0.0f;
                const PosHoldCommand pc = posHold.update(position, failsafe ? 0.0f : sticks[0], failsafe ? 0.0f : sticks[1],
                                                         fence.angleCap(training.angleCap(set.posHold.maxAngleDeg)), speedCap, dt);
                posCmd.rollDeg = pc.rollDeg;
                posCmd.pitchDeg = pc.pitchDeg;
                posCmd.yawRateDps = tune.rates[2].apply(sticks[2]);
            }
            status.posHoldPhase = posHold.phase();
            status.posTargetN = posHold.targetNorth();
            status.posTargetE = posHold.targetEast();
            const RescueCommand* autoCmd = rescuing ? &rescueCmd : posHold.active() ? &posCmd : nullptr;
            rateSetpoints(mode, sticks, sp, stickSp, rc.acroTrainer, autoCmd);
            // launch control: только ACRO с тумблера, без автоматов
            launch.update(rc.launchControl,
                          mode == CopterMode::ACRO && !failsafe && !rescuing && !status.turtle && !status.crashRecovery,
                          rc.throttle, status.airborne);
            const bool launching = launch.active();
            if (launching)
            {
                launch.setpoints(sticks, status.pitch, sp);
                for (uint8_t a = 0; a < 3; ++a) stickSp[a] = sp[a];
            }
            // рывок газом: пока не в воздухе, наклон держит накопленный I
            if (launching) launchCarry = false;
            else if (launch.state() == LaunchState::TRIGGERED && wasLaunching) launchCarry = true;
            if (status.airborne || rc.throttle <= LaunchControl::RESET_THROTTLE || launch.state() == LaunchState::OFF)
                launchCarry = false;
            wasLaunching = launching;
            status.launch = launch.state();
            yawSpinCheck(rates[2]);
            if (!crashRecovery(mode, sp, rates))
            {
                out = CopterOutputs();   // crash_recovery = DISARM
                out.count = tune.frame.count;
                return out;
            }
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
            float climbCmd = 0.0f;   // что просят у удержания высоты, м/с (ниже — «стоит ли уже на земле»)
            if (altitudeMode(mode))
            {
                float climb = 0.0f;
                if (rescuing)
                {
                    climb = rescueCmd.climbRate;
                }
                else if (failsafe)
                {
                    climb = status.failsafeStage >= 2 ? -set.failsafeDescent : 0.0f;   // этап 1 — держит высоту
                }
                else
                {
                    const float s = sticks[3] - 0.5f;   // центр — держать, края — набор/снижение
                    const float dead = fminf(0.45f, fmaxf(0.0f, set.alt.deadband));   // alt_hold_deadband
                    if (s > dead) climb = (s - dead) / (0.5f - dead) * set.alt.maxClimb;
                    if (s < -dead) climb = (s + dead) / (0.5f - dead) * set.alt.maxDescent;
                    climb = fence.limitClimb(training.limitClimb(climb));
                }
                climbCmd = climb;
                altCtl.setHover(hover);
                thrust = altCtl.update(vertical.altitude(), vertical.climbRate(), climb, vertical.baroUsed(), cosTilt, dt);
                status.altTarget = altCtl.getTarget();
            }
            else
            {
                // кривая и предел газа (thr_mid/thr_expo/throttle_limit) — только у стика пилота
                thrust = fixedThrottle ? set.failsafeThrottle : tune.throttle.apply(sticks[3]) * (failsafe ? 1.0f : throttleLimit);
                if (!failsafe) thrust = training.limitThrust(thrust, vertical.climbRate(), hover, cosTilt);
                thrust = fence.limitThrust(thrust, vertical.climbRate(), hover, cosTilt);   // забор — и в failsafe
                status.altTarget = vertical.altitude();
            }

            // launch control: до рывка газ коптер не поднимает — работают только
            // поправки регулятора (airmode)
            if (launching) thrust = 0.0f;

            // в воздухе ли: газ выше 60 % висения (сел — ниже 55 %, checkLanded)
            if (!status.airborne && thrust > 0.6f * hover) status.airborne = true;

            // газ висения: учится, пока коптер спокойно висит (любой режим);
            // при снижении по команде — нет: севший коптер «висит» на
            // падающем газе, и оценка ушла бы за ним вниз
            const bool holding = !altitudeMode(mode) || fabsf(altCtl.climbTarget()) < 0.2f;
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
            if (!altitudeMode(mode) && tune.pid.throttleBoost > 0.0f && !launching)
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
            // Снижение просят (посадка в failsafe, возврат домой, ALT HOLD с
            // газом вниз), а высота не меняется и коптер не качается — он уже
            // стоит, хотя «в воздухе» снимется только через секунды, когда
            // упадёт тяга. Земля раму держит, и интеграл копился бы на любой
            // градус между «ровно» и землёй (склон, подстройка горизонта,
            // ошибка оценки), пока моторы не разойдутся и коптер не запрыгает.
            // Интеграл замирает — не сбрасывается: ошиблись в воздухе (начало
            // снижения, восходящий поток) — рывка нет.
            const bool sinking = climbCmd < -STANDING_CLIMB_MS;
            const bool still = fabsf(vertical.climbRate()) < STANDING_CLIMB_MS && motionLpf.value() < 20.0f;
            standingTimer = (status.airborne && sinking && still) ? standingTimer + dt : 0.0f;
            const bool standing = standingTimer > STANDING_S;
            const bool holdI = !status.airborne || standing;
            if (tune.pid.itermRotation && !holdI)
            {
                float v[3] = { axes[0].integral(), axes[1].integral(), axes[2].integral() };
                rotateIterm(v, rates, dt);
                for (uint8_t a = 0; a < 3; ++a) axes[a].setIntegral(v[a]);
            }
            // launch control: I копится на земле с launch_control_gain (тангаж,
            // в FULL — все оси); после рывка, пока не в воздухе, — замирает
            bool axisHoldI[3] = { holdI, holdI, holdI };
            float axisIBoost[3] = { iBoost, iBoost, iBoost };
            if (launching || launchCarry)
            {
                const AxisGains* g[3] = { &tune.roll, &tune.pitch, &tune.yaw };
                for (uint8_t a = 0; a < 3; ++a)
                {
                    axisHoldI[a] = launchCarry || !launch.integrates(a);
                    if (launching && g[a]->i > 0.0f)
                        axisIBoost[a] = bfToGain(PidTerm::I, a, tune.launch.gainBf) / g[a]->i;
                }
            }
            MixInput mi;
            mi.throttle = mixThrust;
            // EZLANDING: насколько пилот «просит» — газ или любой стик
            mi.stick = fmaxf(sticks[3], fmaxf(fabsf(sticks[0]), fmaxf(fabsf(sticks[1]), fabsf(sticks[2]))));
            mi.speedMs = rescue.gpsOk() ? rescue.groundSpeed() : -1.0f;
            mi.roll = axes[0].update(sp[0], rates[0], tpa, axisIBoost[0], axisHoldI[0], status.saturated, pBoost);
            mi.pitch = axes[1].update(sp[1], rates[1], tpa, axisIBoost[1], axisHoldI[1], status.saturated, pBoost);
            mi.yaw = axes[2].update(sp[2], rates[2], 1.0f, axisIBoost[2], axisHoldI[2], status.saturated);
            // PITCHONLY: только задние моторы — нос вниз, крен и рысканье молчат
            if (launching && tune.launch.mode == LaunchControlMode::PITCHONLY)
            {
                mi.roll = mi.yaw = 0.0f;
                mi.pitch = fminf(0.0f, mi.pitch);
            }
            // раскрутка по рысканью: крен и тангаж без PID (их гироскоп при
            // таком вращении врёт, а D и FF только мешали бы), рысканье —
            // одно P против вращения, всем ходом (как в BF)
            if (status.yawSpin)
            {
                mi.roll = 0.0f;
                mi.pitch = 0.0f;
                mi.yaw = fminf(1.0f, fmaxf(-1.0f, axes[2].lastTerms().p));
            }
            // chirp: качание частоты поверх суммы PID одной оси (168)
            {
                const float excite = chirpStep(rc, mode, failsafe || rescue.active(), thrust);
                float* const axisCommand[3] = { &mi.roll, &mi.pitch, &mi.yaw };
                const uint8_t ca = chirp.axis();
                *axisCommand[ca] += excite;
                status.chirpExcitation = excite;
                status.chirpCommand = *axisCommand[ca];
            }
            // на земле — и при выравнивании после удара (как в BF: накопленное
            // за кувырок потом докручивало бы раму мимо горизонта)
            if (!status.airborne || status.crashRecovery || status.yawSpin)
            {
                for (uint8_t a = 0; a < 3; ++a)
                {
                    // launch control держит наклон интегралом — его не трогать
                    const bool keep = (launching || launchCarry) && launch.integrates(a) && !status.crashRecovery &&
                                      !status.yawSpin;
                    if (!keep) axes[a].resetIntegral();
                }
            }

            // airmode — когда после ARM стик газа хоть раз был выше
            // airmode_start_throttle_percent (как в BF: на земле с газом внизу
            // моторы не раскачивают коптер) или коптер уже в воздухе (наше)
            if (sticks[3] >= set.airmodeStartThrottle || status.airborne) airmodeStarted = true;
            mixer.setAirmode(((set.airmode || rc.airmode) && airmodeStarted) || status.crashRecovery || status.yawSpin ||
                             launching || launchCarry);
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
            if (!launching && !launchCarry) checkRunaway(thrust, rates);

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
                out.motor[i] = Mixer::withIdle(signal, tune.idle + status.dynIdleIncrease, tune.motorOutputLimit * outputScale);
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
        // ориентация — компасу: поле из осей рамы в земные (copter/Compass.h)
        const AttitudeQuat& getAttitude() const { return attitude; }
        // нуль гироскопа откалиброван заново — фильтр забывает выученное смещение
        void gyroRecalibrated() { attitude.forgetGyroBias(); }
        const Chirp& getChirp() const { return chirp; }
        const PositionEstimator& getPosition() const { return position; }
        const GpsIntegrity& getGpsIntegrity() const { return gpsIntegrity; }
        const DeadReckonHome& getDeadReckon() const { return deadReck; }
        const Geofence& getGeofence() const { return fence; }
        bool isArmed() const { return status.state != CopterState::DISARMED; }

        // Разоружить немедленно (консоль, ошибка).
        void disarm()
        {
            status.state = CopterState::DISARMED;
            status.airborne = false;
            status.failsafeStage = 0;
            status.runawayCheck = false;
            status.turtle = false;
            launch.disarm();
            launchCarry = wasLaunching = false;
            status.launch = LaunchState::OFF;
        }

    private:
        // ФНЧ и режекторы как в BF поверх простых PT2/PT1 из configure().
        void configureFilterChain(const FilterSpec& f)
        {
            if (!f.valid) return;
            gyroFilter.configureChain(f);
            for (uint8_t a = 0; a < 3; ++a) axes[a].configureDterm(f);
        }

        static constexpr float FAILSAFE_HOLD_S = 0.3f;        // провал связи короче — последние ручки (как в BF)
        static constexpr float FAILSAFE_KILL_LOCK_S = 1.0f;   // после KILL с тумблера ARM не раньше
        static constexpr float POS_READY_S = 1.0f;            // GPS и курс годятся столько подряд — позицию можно держать
        static constexpr float POS_GROUND_M = 1.0f;           // failsafe: ниже — ровно (наклон в землю мешает понять, что сел)
        static constexpr float STANDING_CLIMB_MS = 0.3f;      // просят снижаться быстрее, а скорость меньше — стоит
        static constexpr float STANDING_S = 0.3f;             // столько подряд
        // chirp: начать — наклон меньше 25° и коптер не крутится; прервать —
        // наклон больше 45° в ANGLE и 70° в ACRO, моторы упёрты дольше 0.25 с
        static constexpr float CHIRP_START_COS = 0.9063f;
        static constexpr float CHIRP_START_MOTION_DPS = 60.0f;
        static constexpr float CHIRP_ABORT_COS_ANGLE = 0.7071f;
        static constexpr float CHIRP_ABORT_COS_ACRO = 0.3420f;
        static constexpr float CHIRP_SATURATED_S = 0.25f;
        static constexpr float CHIRP_THRUST_SHARE = 0.5f;     // размах не больше этой доли газа
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
            return altitudeMode(rc.mode) ? rc.throttle <= 0.42f : rc.throttle <= 0.05f;
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

        // Turtle mode (FLIP OVER AFTER CRASH): PID, удержание, проверки
        // удара, runaway и автодисарма не работают — коптер лежит на спине,
        // а моторы стороны, выбранной стиком, крутят наоборот (газ не нужен).
        // Связи нет — DISARM (на земле, handleLink). Выйти — DISARM.
        CopterOutputs turtleOutputs(const CopterSensors& sens, const CopterRc& rc)
        {
            resetFlight(sens);
            CopterOutputs out;
            out.count = tune.frame.count;
            float cmd[MAX_MOTORS];
            turtleMix(tune.frame, rc.roll, rc.pitch, rc.yaw, set.turtle, cmd);
            for (uint8_t i = 0; i < out.count && i < MAX_MOTORS; ++i)
            {
                out.motor[i] = cmd[i] < 0.0f ? MOTOR_STOP : Mixer::withIdle(cmd[i], tune.idle, tune.motorOutputLimit);
            }
            out.spinning = true;
            out.reversed = true;
            avgThrust = 0.0f;
            status.throttle = 0.0f;
            return out;
        }

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
            rcSmooth.reset();
            for (auto& f : angleFfLpf) f.reset();
            angleTargetValid = false;
            horizonStick.reset(1.0f);
            throttleHpLpf.reset();
            throttleBoostLpf.reset();
            tpaLowDone = false;
            airmodeStarted = false;
            status.airborne = false;
            status.saturated = false;
            lastMixThrottle = 0.0f;
            lastMixSpread = 0.0f;
            failsafeTimer = 0.0f;
            landingTimer = 0.0f;
            crashTimer = 0.0f;
            status.crashRecovery = false;
            crashPrevValid = false;
            status.yawSpin = false;
            yawSpinCalmS = 0.0f;
            if (chirp.active()) chirp.stop(ChirpEnd::NOT_FLYING);
            chirpAxisNext = 0;
            chirpSwitch = true;   // тумблер, включённый до ARM, — не команда: выключить и включить
            status.chirpActive = false;
            status.chirpAxis = 0;
            status.chirpHz = 0.0f;
            status.chirpExcitation = status.chirpCommand = 0.0f;
            lostControlTimer = 0.0f;
            yawLostTimer = 0.0f;
            levelErrorDeg = 0.0f;
            runawayTimer = 0.0f;
            groundIdleTimer = 0.0f;
            if (sens.baroOk) baroHome = sens.baroAltM;
            vertical.reset(sens.baroOk ? sens.baroAltM - baroHome : 0.0f);
            altCtl.reset(0.0f, 0.0f);
            rescue.resetFlight();
            deadReck.resetFlight();
            blindGpsBack = 0.0f;
            training.reset();
            status.trainingFlags = 0;
            fence.reset();
            status.fenceFlags = 0;
            status.fenceMarginM = 0.0f;
            posHold.stop();
            posHold.clearFail();
            status.posHoldPhase = PosHoldPhase::OFF;
            status.posHoldFail = PosHoldFail::NONE;
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
                const bool low = !altitudeMode(rc.mode) && rc.throttle <= 0.05f;
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
            else if (set.smallAngleDeg < 180.0f && attitude.tiltDeg() > set.smallAngleDeg && !turtleWanted(rc)) b |= ARM_BLOCK_TILT;
            if (motionTimer < 0.3f) b |= ARM_BLOCK_MOVING;
            if (status.volts > 1.0f && tune.volts > 1.0f)
            {
                const float cells = tune.cells ? tune.cells : roundf(tune.volts / 3.75f);
                if (cells >= 1.0f && status.volts / cells < set.minCellVolts) b |= ARM_BLOCK_BATTERY;
            }
            if (testLeft > 0.0f || musicActive()) b |= ARM_BLOCK_MOTOR_TEST;
            if (status.crashed) b |= ARM_BLOCK_CRASH;
            if (status.runaway) b |= ARM_BLOCK_RUNAWAY;
            if (rc.failsafeSwitch) b |= ARM_BLOCK_FAILSAFE_SWITCH;
            if (!rc.prearm) b |= ARM_BLOCK_NO_PREARM;
            if (paralyzed) b |= ARM_BLOCK_PARALYZE;
            if (!rc.armAssigned) b |= ARM_BLOCK_NO_ARM_SWITCH;
            // GPS-RESCUE по потере связи без GPS не вернёт — не армить (как BF)
            if (set.failsafeProcedure == FailsafeProcedure::GPS_RESCUE && !set.rescue.allowArmingWithoutFix && !rescue.gpsOk())
                b |= ARM_BLOCK_GPS;
            // геозабор без GPS не знает, где край, — не армить
            if (set.geofence.area() && !rescue.gpsOk()) b |= ARM_BLOCK_GPS;
            // поле: только внутри и не в запретной зоне
            if (fence.armOutside() && rescue.gpsOk()) b |= ARM_BLOCK_FENCE;
            b |= externalBlockers;
            if (!switchSeenOff) b |= forcedOff ? ARM_BLOCK_NOT_DISARMED : ARM_BLOCK_SWITCH;
            return b;
        }

        // ARM в turtle mode: режим включён и регуляторы DShot (иначе тумблер
        // ничего не делает — как в BF). Лежит на спине — это и нужно: наклон не проверяется.
        bool turtleWanted(const CopterRc& rc) const { return rc.flipOverAfterCrash && set.turtleAvailable; }

        // PARALYZE (как в BF): после посадки или аварии тумблер навсегда, до
        // перезагрузки, запрещает ARM — коптер не взлетит, пока его подбирают.
        // Срабатывает только на земле, только после хотя бы одного ARM и только
        // если тумблер после включения питания был выключен: включённый при
        // загрузке он не запирает коптер на столе. В полёте не действует —
        // защёлкнется после DISARM, если так и включён.
        void updateParalyze(const CopterRc& rc, bool rcValid)
        {
            if (paralyzed || !rcValid) return;
            if (!rc.paralyze)
            {
                paralyzeSeenOff = true;
                return;
            }
            if (paralyzeSeenOff && status.armCount > 0 && status.state == CopterState::DISARMED) paralyzed = true;
        }

        void handleArming(const CopterRc& rc, const CopterSensors& sens, bool rcValid)
        {
            updateParalyze(rc, rcValid);
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
            // тумблер ARM под музыкой моторами — сначала тишина, армит следующее включение
            if (rising && musicActive()) stopMusic();
            prevSwitch = true;
            if (status.state != CopterState::DISARMED) return;
            if (rising && status.armBlockers == 0)
            {
                status.state = CopterState::ARMED;
                status.mode = rc.mode;
                ++status.armCount;
                status.headfreeRefDeg = attitude.yawDeg();
                stopMotorTest();
                status.turtle = turtleWanted(rc);
                status.runawayCheck = set.runawayPrevention && !status.turtle;
                launch.arm(rc.launchControl, rc.mode == CopterMode::ACRO && !status.turtle);
                launchCarry = wasLaunching = false;
                status.launch = launch.state();
                runawayTimer = runawayFlown = 0.0f;
                groundIdleTimer = lowThrottleTimer = 0.0f;
                attitude.setGrounded(true);
                if (altitudeMode(rc.mode)) altCtl.reset(vertical.altitude(), 0.0f);
            }
            // ARM включён, но армить нельзя — нужен новый щелчок тумблером
            switchSeenOff = false;
        }

        void detectCrash()
        {
            if (!set.crashDetection) return;
            const float tilt = attitude.tiltDeg();
            const float motion = motionLpf.value();
            const bool levelMode = status.mode == CopterMode::ANGLE || altitudeMode(status.mode);
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

        // Crash recovery (как в BF): в ACRO и HORIZON в воздухе удар — моторы в
        // упоре, скорость по крену или тангажу скачком (crash_dthreshold) и
        // разошлась с уставкой (crash_gthreshold), а стик спокоен
        // (crash_setpoint_threshold). Тогда уставка — к горизонту, рысканье не
        // быстрее crash_limit_yaw, I в нуле, airmode; ровно и спокойно (после
        // crash_delay) или прошло crash_time — управление пилоту. DISARM —
        // моторы стоп сразу. false — разоружился.
        bool crashRecovery(CopterMode mode, float sp[3], const float r[3])
        {
            const PidExtras& px = tune.pid;
            float jump[2];
            for (uint8_t a = 0; a < 2; ++a)
            {
                jump[a] = crashPrevValid ? fabsf(r[a] - crashPrevRate[a]) / dt : 0.0f;
                crashPrevRate[a] = r[a];
            }
            crashPrevValid = true;
            const bool allowed = px.crashRecovery != CrashRecovery::OFF && status.airborne &&
                                 (mode == CopterMode::ACRO || mode == CopterMode::HORIZON);
            if (!allowed)
            {
                status.crashRecovery = false;
                return true;
            }
            if (!status.crashRecovery)
            {
                if (!status.saturated) return true;
                bool hit = false;
                for (uint8_t a = 0; a < 2; ++a)
                {
                    hit = hit || (jump[a] > px.crashDThreshold && fabsf(sp[a] - r[a]) > px.crashGThreshold &&
                                  fabsf(sp[a]) < px.crashSetpointThreshold);
                }
                if (!hit) return true;
                if (px.crashRecovery == CrashRecovery::DISARM)
                {
                    status.crashed = true;
                    forcedDisarm();
                    return false;
                }
                status.crashRecovery = true;
                crashRecoveryTimer = 0.0f;
            }
            crashRecoveryTimer += dt;
            float level[3];
            levelTo(0.0f, 0.0f, attitude.upBody(), level);
            const float cap = 0.9f * tune.rates[0].top();
            sp[0] = fminf(cap, fmaxf(-cap, level[0]));
            sp[1] = fminf(cap, fmaxf(-cap, level[1]));
            sp[2] = fminf(px.crashLimitYaw, fmaxf(-px.crashLimitYaw, sp[2]));
            // после упора гироскопа горизонт ищется заново (P54): пока MEKF в нём
            // не уверен, «ровно» по оценке — ещё не ровно; держит до crash_time
            const bool calm = !status.saturated && fabsf(r[0]) < px.crashRecoveryRate && fabsf(r[1]) < px.crashRecoveryRate &&
                              attitude.tiltDeg() < px.crashRecoveryAngleDeg && !(attitude.ekfEnabled() && attitude.horizonUnsure());
            if ((calm && crashRecoveryTimer >= px.crashDelayS) || crashRecoveryTimer >= px.crashTimeS) status.crashRecovery = false;
            return true;
        }

        // Порог раскрутки: ON — yaw_spin_threshold; AUTO — рейт рысканья на
        // полном стике × 1.25 + 200 °/с, в пределах 500…1950 (быстрее своих
        // рейтов честно не крутят — значит, вращение не от пилота)
        float yawSpinThreshold() const
        {
            if (set.yawSpin == YawSpinMode::ON) return set.yawSpinThresholdDps;
            return fminf(YAW_SPIN_MAX_DPS, fmaxf(YAW_SPIN_MIN_DPS, 1.25f * tune.rates[2].top() + 200.0f));
        }

        // Yaw spin recovery (069): рысканье быстрее порога — режим остановки;
        // выход — когда 20 мс подряд на 100 °/с ниже порога
        void yawSpinCheck(float yawRate)
        {
            if (set.yawSpin == YawSpinMode::OFF)
            {
                status.yawSpin = false;
                return;
            }
            const float th = yawSpinThreshold();
            const float r = fabsf(yawRate);
            if (r > th)
            {
                status.yawSpin = true;
                yawSpinCalmS = 0.0f;
            }
            else if (status.yawSpin)
            {
                yawSpinCalmS = r < th - 100.0f ? yawSpinCalmS + dt : 0.0f;
                if (yawSpinCalmS >= YAW_SPIN_EXIT_S) status.yawSpin = false;
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
                else if (rescue.landing() || deadReck.landing()) forcedDisarm();   // GPS RESCUE с тумблера сел
            }
            // failsafe_landing_time: этап 2 не кончился посадкой — DISARM
            // (полёт домой — не посадка: время идёт с этапа LAND или без возврата)
            const bool flyingHome = (rescue.active() && !rescue.landing() && rescue.phase() != RescuePhase::ABORT && !deadReck.active()) ||
                                    (deadReck.active() && !deadReck.landing());
            if (status.state == CopterState::FAILSAFE && status.failsafeStage >= 2 && !flyingHome)
            {
                failsafeTimer += dt;
                if (failsafeTimer > set.failsafeLandingS) failsafeDisarm();
            }
        }

        // CHIRP (copter/Chirp.h): свип начинается по фронту тумблера (включён
        // заранее, до ARM, — не считается), только в воздухе, в ANGLE или ACRO,
        // когда коптер висит спокойно и моторы не упёрты. Пройденный свип
        // переводит на следующую ось: крен, тангаж, рысканье; прерванный —
        // повторится на той же. Прерывают: тумблер, смена режима и любой
        // автомат (failsafe, возврат домой), наклон больше предела, упор
        // моторов дольше CHIRP_SATURATED_S, посадка. Стики работают как
        // обычно — управление всё время у пилота.
        float chirpStep(const CopterRc& rc, CopterMode mode, bool automatic, float thrust)
        {
            const bool modeOk = !automatic && status.state == CopterState::ARMED &&
                                (mode == CopterMode::ANGLE || mode == CopterMode::ACRO);
            const bool rising = rc.chirp && !chirpSwitch;
            chirpSwitch = rc.chirp;
            const bool was = chirp.active();
            if (!was)
            {
                if (rising && modeOk && status.airborne && !status.crashRecovery && !status.saturated &&
                    attitude.cosTilt() > CHIRP_START_COS && motionLpf.value() < CHIRP_START_MOTION_DPS)
                {
                    chirp.start(chirpAxisNext);
                    chirpSaturated = 0.0f;
                }
            }
            else
            {
                chirpSaturated = status.saturated ? chirpSaturated + dt : 0.0f;
                const float limit = mode == CopterMode::ANGLE ? CHIRP_ABORT_COS_ANGLE : CHIRP_ABORT_COS_ACRO;
                ChirpEnd end = ChirpEnd::NONE;
                if (!rc.chirp) end = ChirpEnd::SWITCH;
                else if (!status.airborne) end = ChirpEnd::NOT_FLYING;
                else if (!modeOk) end = ChirpEnd::MODE;
                else if (status.crashRecovery || attitude.cosTilt() < limit) end = ChirpEnd::TILT;
                else if (chirpSaturated > CHIRP_SATURATED_S) end = ChirpEnd::SATURATED;
                if (end != ChirpEnd::NONE) chirp.stop(end);
            }
            const float value = chirp.update(dt, CHIRP_THRUST_SHARE * thrust);
            if (was && !chirp.active())
            {
                status.chirpEnd = chirp.lastEnd();
                if (chirp.lastEnd() == ChirpEnd::DONE) chirpAxisNext = static_cast<uint8_t>((chirp.axis() + 1) % 3);
            }
            status.chirpActive = chirp.active();
            status.chirpAxis = chirp.active() ? chirp.axis() : chirpAxisNext;
            status.chirpRun = chirp.runCount();
            status.chirpHz = chirp.hz();
            return value;
        }

        // Сглаживание упреждения ANGLE и возврата выравнивания HORIZON (BF 4.5).
        void configureLevelFilters(const PidExtras& p)
        {
            const float ffHz = p.angleFfSmoothingS > 0.0f ? 1.0f / (2.0f * PI_F * p.angleFfSmoothingS) : 0.0f;
            for (auto& f : angleFfLpf) f.configure(ffHz, dt);
            horizonStick.configure(p.horizonDelayS > 0.0f ? 1.0f / (2.0f * PI_F * p.horizonDelayS) : 0.0f, dt);
        }

        // Уставки угловой скорости; stick — их часть от стика (для упреждения:
        // в ANGLE — наклон стиком, без поправки горизонта).
        //   ACRO     — кривая расходов; с ACRO TRAINER наклон крена и тангажа не
        //              больше acro_trainer_angle_limit (с упреждением по скорости);
        //   ANGLE    — стик задаёт угол до angle_limit; выравнивание angle_p_gain,
        //              упреждение angle_feedforward по скорости целевого угла,
        //              рысканье вокруг вертикали Земли (angle_earth_ref);
        //   HORIZON  — как в BF 4.5: ACRO плюс выравнивание к горизонту силой
        //              horizon_level_strength, гаснущее к horizon_limit_sticks
        //              стика и к horizon_limit_degrees наклона, после флипа
        //              возвращается за horizon_delay_ms.
        void rateSetpoints(CopterMode mode, const float sticks[4], float sp[3], float stick[3], bool trainer = false,
                           const RescueCommand* rescueCmd = nullptr)
        {
            const PidExtras& px = tune.pid;
            float acro[3];
            for (int a = 0; a < 3; ++a) acro[a] = tune.rates[a].apply(sticks[a]);
            if (status.fpvMix && !rescueCmd) fpvMixRotate(acro[0], acro[2], tune.fpvMixDeg, tune.rates[0].top(), tune.rates[2].top());
            // ANGLE и ALT_HOLD (и автомат failsafe) — целевой угол стиком; иначе упреждения нет
            if (mode == CopterMode::ACRO || mode == CopterMode::HORIZON) angleTargetValid = false;
            if (mode == CopterMode::ACRO)
            {
                for (int a = 0; a < 3; ++a) sp[a] = stick[a] = acro[a];
                if (trainer) acroTrainer(sp);
                return;
            }

            const Vec3 up = attitude.upBody();
            const Vec3 down = attitude.downBody();
            const float cap = 0.9f * tune.rates[0].top();
            // рысканье: вокруг вертикали Земли или оси рамы (angle_earth_ref)
            const float er = fminf(1.0f, fmaxf(0.0f, px.angleEarthRef));
            const float yawRate = rescueCmd ? rescueCmd->yawRateDps : acro[2];
            const Vec3 yawAxis = { er * down.x, er * down.y, er * down.z + (1.0f - er) };

            if (mode == CopterMode::HORIZON)
            {
                // выравнивание к горизонту, как в ANGLE с центральным стиком
                float level[3];
                levelTo(0.0f, 0.0f, up, level);
                const float deflection = fmaxf(fabsf(sticks[0]), fabsf(sticks[1]));
                const float stickAtt = fmaxf(0.0f, 1.0f - deflection / fmaxf(0.05f, px.horizonLimitSticks));
                // гаснет сразу, возвращается плавно (horizon_delay_ms)
                const float smooth = stickAtt < horizonStick.value() ? (horizonStick.reset(stickAtt), stickAtt)
                                                                     : horizonStick.apply(stickAtt);
                const float tiltDeg = acosf(fminf(1.0f, fmaxf(-1.0f, -up.z))) * 180.0f / PI_F;
                const float angleAtt = fmaxf(0.0f, 1.0f - tiltDeg / fmaxf(1.0f, px.horizonLimitDeg));
                const float k = px.horizonStrength * smooth * angleAtt;
                for (int a = 0; a < 2; ++a)
                {
                    sp[a] = acro[a] + k * fminf(cap, fmaxf(-cap, level[a]));
                    stick[a] = acro[a];
                }
                sp[2] = acro[2] + k * level[2];
                stick[2] = acro[2];
                return;
            }

            // ANGLE: целевой угол стиком; упреждение — по его скорости
            const float maxA = fence.angleCap(training.angleCap(tune.maxAngleDeg));   // учебный предел и забор
            float target[2] = { sticks[0] * maxA, sticks[1] * maxA };
            if (rescueCmd)
            {
                // возврат домой: наклон задаёт автомат (copter/GpsRescue.h)
                target[0] = rescueCmd->rollDeg;
                target[1] = rescueCmd->pitchDeg;
                angleTargetValid = false;
            }
            float ff[2] = { 0.0f, 0.0f };
            if (px.angleFf > 0.0f && !rescueCmd)
            {
                for (int a = 0; a < 2; ++a)
                {
                    const float speed = angleTargetValid ? (target[a] - angleTarget[a]) / dt : 0.0f;
                    ff[a] = px.angleFf * angleFfLpf[a].apply(speed);
                }
            }
            angleTarget[0] = target[0];
            angleTarget[1] = target[1];
            angleTargetValid = true;
            float level[3];
            levelTo(target[0], target[1], up, level);
            float angleSp[3] = { level[0] + yawRate * yawAxis.x + ff[0], level[1] + yawRate * yawAxis.y + ff[1],
                                 level[2] + yawRate * yawAxis.z };
            for (int a = 0; a < 2; ++a) angleSp[a] = fminf(cap, fmaxf(-cap, angleSp[a]));
            // наклон стиком: у ровного коптера — levelGain × угол стика (+ упреждение)
            float angleStick[3] = { tune.levelGain * target[0] + ff[0], tune.levelGain * target[1] + ff[1], yawRate };
            for (int a = 0; a < 2; ++a) angleStick[a] = fminf(cap, fmaxf(-cap, angleStick[a]));
            for (int a = 0; a < 3; ++a)
            {
                sp[a] = angleSp[a];
                stick[a] = angleStick[a];
            }
        }

        // Желаемый «верх» в осях рамы против текущего -> поворот кратчайшим
        // путём, levelGain (°/с) на градус ошибки.
        void levelTo(float rollDeg, float pitchDeg, const Vec3& up, float level[3])
        {
            rollDeg += set.levelTrimRollDeg;
            pitchDeg -= set.levelTrimPitchDeg;
            const float tr = rollDeg * PI_F / 180.0f, tp = pitchDeg * PI_F / 180.0f;
            const Vec3 target = { sinf(tp), -sinf(tr) * cosf(tp), -cosf(tr) * cosf(tp) };
            Vec3 axis = cross(target, up);
            const float s = length(axis);
            const float angle = atan2f(s, dot(target, up));
            levelErrorDeg = angle * 180.0f / PI_F;
            level[0] = level[1] = level[2] = 0.0f;
            if (s > 1e-6f)
            {
                const float k = tune.levelGain * angle * 180.0f / PI_F / s;
                level[0] = axis.x * k;
                level[1] = axis.y * k;
                level[2] = axis.z * k;
            }
        }

        // ACRO TRAINER: наклон (с упреждением по скорости) за пределом — уставка
        // не дальше, чем возвращает к пределу (acro_trainer_gain).
        void acroTrainer(float sp[3]) const
        {
            const PidExtras& px = tune.pid;
            const float angle[2] = { status.roll, status.pitch };
            for (int a = 0; a < 2; ++a)
            {
                const float projected = angle[a] + status.rates[a] * px.acroTrainerLookaheadS;
                const float limit = px.acroTrainerLimitDeg;
                if (projected > limit) sp[a] = fminf(sp[a], px.acroTrainerGain * (limit - projected));
                else if (projected < -limit) sp[a] = fmaxf(sp[a], px.acroTrainerGain * (-limit - projected));
            }
        }

        CopterTune tune;
        CopterSettings set;
        float dt = 0.001f;
        Mixer mixer;
        GyroFilter gyroFilter;
        RateAxis axes[3];
        AbsoluteControl absCtl[3];
        GpsRescue rescue;
        DeadReckonHome deadReck;
        float blindGpsBack = 0.0f;                       // вслепую: GPS снова годен подряд, с
        static constexpr float BLIND_GPS_BACK_S = 3.0f;  // столько — и обратно к GPS Rescue
        PositionEstimator position;
        GpsIntegrity gpsIntegrity;
        float earthAccelX = 0.0f, earthAccelY = 0.0f;    // горизонтальное ускорение в осях «от включения», м/с²
        static constexpr float GPS_WIND_SIGMA = 0.12f;   // сопротивление выучено — ветру верим (подмена по ветру)
        PositionHold posHold;
        float posOkTimer = 0.0f;                         // GPS и курс годятся подряд, с
        PosHoldFail posLastFail = PosHoldFail::NO_GPS;   // чего не хватало в последний раз
        TrainingLimits training;
        Geofence fence;

        // Потолок для возврата домой: учебный и забора — ниже из двух (0 — нет).
        float rescueCeiling() const
        {
            const float a = training.rescueCeilingM(), b = fence.rescueCeilingM();
            return a > 0.0f && b > 0.0f ? fminf(a, b) : fmaxf(a, b);
        }
        RcSmoothing rcSmooth;
        Pt1 throttleHpLpf;               // anti-gravity
        Pt1 throttleBoostLpf;            // throttle_boost
        Pt1 angleFfLpf[2];               // angle_feedforward_smoothing_ms
        Pt1 horizonStick;                // horizon_delay_ms
        float angleTarget[2] = {};
        bool angleTargetValid = false;
        Pt1 voltLpf;
        bool heldThrottle = false;   // этап 1 failsafe на последнем газе (rxfail 3 h)
        float throttleLimit = 1.0f;   // vbat_cutoff_percent
        Pt2 accelLpf[3];
        float accelG[3] = { 0.0f, 0.0f, 1.0f };   // после ФНЧ
        Pt1 motionLpf;
        AttitudeQuat attitude;
        ImuAlign imuAlign;               // плата -> рама: align_board_*, acc_calibration, gyro_offset_yaw
        // сопротивление винтов меряется выше (над точкой ARM) или на ходу
        static constexpr float DRAG_MIN_HEIGHT_M = 2.0f, DRAG_MIN_SPEED_MS = 2.0f;
        // гироскоп в упоре: все наши драйверы ставят чип на ±2000 °/с; запас на округление
        static constexpr float GYRO_SATURATED_DPS = 1960.0f;
        GpsAccel gpsAccel;               // в разгоне по GPS горизонт не тянется к акселерометру (P32)
        VerticalEstimator vertical;
        AltitudeController altCtl;
        CopterStatus status;
        CopterRc lastGoodRc;

        float hover = 0.4f;
        float avgThrust = 0.0f;
        float outputScale = 1.0f;        // current_limit_a (EscGuard::outputScale)
        float rotEstimate = 0.0f;
        static constexpr float RPM_LEARN_MIN_HZ = 20.0f;   // ниже — холостые на столе, модель там грубая
        static constexpr float RPM_LEARN_TAU_S = 2.0f;
        static constexpr float RPM_SCALE_MIN = 0.6f, RPM_SCALE_MAX = 1.6f;
        float lastMixThrottle = 0.0f;
        float lastMixSpread = 0.0f;      // разброс моторов прошлого цикла (iterm_windup)
        bool tpaLowDone = false;         // газ после ARM был выше tpa_low_breakpoint
        bool airmodeStarted = false;     // газ после ARM был выше airmode_start_throttle_percent
        LaunchControl launch;            // режим LAUNCH CONTROL
        DynamicIdle dynIdle;             // dyn_idle_*: подъём холостых по оборотам
        bool launchCarry = false;        // рывок уже был, а коптер ещё на земле: I тангажа держит наклон
        bool wasLaunching = false;
        float yawSpinCalmS = 0.0f;       // раскрутка: сколько рысканье уже ниже порога выхода
        static constexpr float YAW_SPIN_MIN_DPS = 500.0f, YAW_SPIN_MAX_DPS = 1950.0f, YAW_SPIN_EXIT_S = 0.02f;
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
        float standingTimer = 0.0f;      // автомат снижается, а коптер стоит (интеграл замирает)
        Chirp chirp;                     // качание частоты (168)
        uint8_t chirpAxisNext = 0;
        bool chirpSwitch = true;         // тумблер CHIRP в прошлом цикле (фронт)
        float chirpSaturated = 0.0f;
        float crashTimer = 0.0f;
        float lostControlTimer = 0.0f;
        float crashPrevRate[2] = {};
        bool crashPrevValid = false;
        float crashRecoveryTimer = 0.0f;
        float yawLostTimer = 0.0f;
        float levelErrorDeg = 0.0f;
        float baroHome = 0.0f;
        bool switchSeenOff = false;
        bool prevSwitch = false;
        bool paralyzeSeenOff = false;   // PARALYZE: тумблер был выключен после загрузки
        bool paralyzed = false;         // PARALYZE защёлкнут — до перезагрузки
        uint32_t externalBlockers = 0;

        float testThrust[MAX_MOTORS] = {};
        float testLeft = 0.0f;
#if defined(BOARD_STM32F405)
        MotorMusicParts* mp = nullptr;
#else
        MotorMusicParts ownMusic;
        MotorMusicParts* mp = &ownMusic;
#endif
    };
}
