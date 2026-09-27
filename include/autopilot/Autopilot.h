#pragma once
#include <Arduino.h>
#include <math.h>

#include "autopilot/AltitudeSpeedController.h"
#include "autopilot/AutoTrim.h"
#include "autopilot/AutopilotTypes.h"
#include "autopilot/LaunchController.h"
#include "autopilot/Navigation.h"
#include "autopilot/PidController.h"
#include "autopilot/SoaringController.h"
#include "config/Config.h"
#include "control/ControlCommand.h"
#include "sensors/SensorInterface.h"
#include "sensors/airspeed/AirspeedSensor.h"

// ============================================================
// АВТОПИЛОТ
//
// Получает за такт стики пилота (ControlCommand, мкс отклонения рулей,
// физические знаки: roll > 0 — крен вправо, pitch > 0 — нос вверх,
// yaw > 0 — нос вправо), газ пилота, тумблеры и крутилки
// (PilotInputs, см. config/Controls.h) и отдаёт итоговую команду
// рулей (getCommand()) и газ (applyThrottle()) для текущего режима.
// FlightController только передаёт их на выходы.
//
// Режимы (AutopilotTypes.h, подробно — docs/AUTOPILOT_GUIDE.md):
//   MANUAL      рули = стики (+ автотриммер/координация, если включены)
//   STABILIZE   стик задаёт крен/тангаж (до MAX_BANK/STAB_MAX_PITCH),
//               отпустил — горизонт
//   ACRO        стик — скорость вращения, гироскоп гасит лишнее
//   ALT_HOLD    крен как STABILIZE, высоту держит руль высоты,
//               газ — пилот
//   CRUISE      держит курс и высоту, газ сам; стик крена/тангажа —
//               новый курс/высота
//   LOITER      круги над точкой включения (без GPS — круги на месте)
//   RTH         домой на высоте RTH_ALTITUDE_M, над домом — круги
//   AUTO_TAKEOFF автовзлёт по газу пилота (программа тангажа и газа)
//   LAUNCH      запуск с руки (LaunchController)
//   AUTO_LAND   планирование с выключенным мотором и выравнивание
//   SOARING     парение в термиках (SoaringController)
//   RESCUE      крылья ровно, нос вверх, газ — выйти из любой беды
//
// Углы IMU — в авиационных знаках: ПИД с ошибкой (цель − факт) сразу
// даёт команду нужного знака. Без IMU (или IMU не прошёл предполётную
// проверку) режимы со стабилизацией работают как MANUAL — рули по
// стикам, без коррекций: с перевёрнутой платой коррекции пошли бы
// в обратную сторону.
//
// Пока самолёт не заармлен, стабилизация двигает рули (видно на
// столе, в какую сторону отвечают рули), но интеграторы на нуле.
//
// Потеря связи в воздухе (armed): при живом GPS и известном доме —
// возврат домой с мотором (Config::FAILSAFE_RTH), иначе —
// планирование с выключенным мотором (FAILSAFE_GLIDE_*).
// ============================================================

struct NavStatus
{
    bool gpsGood = false;
    bool homeValid = false;
    GeoPoint home;
    GeoPoint position;
    float distanceHomeM = -1;   // -1 — дом или GPS неизвестны
    float bearingHomeDeg = 0;
    float courseDeg = 0;        // курс, по которому ведёт навигация
    float targetCourseDeg = 0;
    float speedMs = 0;          // скорость для навигации (трубка / GPS / по умолчанию)
    bool fenceBreached = false;
    bool stallWarning = false;
};

class Autopilot
{
public:

    enum class CourseSource : uint8_t { NONE, GYRO, COMPASS, GPS };

    explicit Autopilot(ImuSensor* imu = nullptr, BarometerSensor* baro = nullptr,
                       MagnetometerSensor* mag = nullptr, GpsSensor* gps = nullptr,
                       AirspeedSensor* airspeed = nullptr);

    bool begin();

    // Тумблеры и крутилки этого такта — до update().
    void setInputs(const PilotInputs& pilotInputs);

    // Один раз за цикл (FlightController::update()), в том числе во
    // время потери связи — датчики читаются всегда, чтобы фильтры не
    // "застывали". sticks — команды стиков (в failsafe — нули).
    void update(bool isArmed, bool linkLost, uint16_t pilotThrottleUs,
                const ControlCommand& sticks = ControlCommand());

    // Итоговые рули (roll/pitch/yaw), мкс; закрылки — поле пилота.
    ControlCommand getCommand() const { return output; }

    // Газ на ESC для текущего режима, исходя из газа пилота (мкс).
    // ARM и выключение мотора учитывает FlightController.
    uint16_t applyThrottle(uint16_t pilotThrottleUs) const;

    void setMode(AutopilotMode mode);

    AutopilotMode getMode() const { return currentMode; }

    const char* getModeName() const;

    // Итоговая команда минус стики пилота (мкс) — "что добавил автопилот".
    float getRollCorrection() const { return static_cast<float>(output.roll - pilot.roll); }
    float getPitchCorrection() const { return static_cast<float>(output.pitch - pilot.pitch); }
    float getYawCorrection() const { return static_cast<float>(output.yaw - pilot.yaw); }

    // Газ автоматического режима, % (0, если газом управляет пилот).
    float getThrottleCorrection() const { return autoThrottlePct; }
    bool isAutoThrottle() const { return throttleMode != ThrottleMode::PILOT; }

    // Связь потеряна в воздухе и автопилот ведёт самолёт сам.
    bool isFailsafeActive() const { return failsafe != Failsafe::NONE; }
    bool isFailsafeGliding() const { return failsafe == Failsafe::GLIDE; }
    bool isFailsafeReturning() const { return failsafe == Failsafe::RTH; }

    float getDesiredRoll() const { return desiredRoll; }
    float getDesiredPitch() const { return desiredPitch; }
    float getTargetAltitude() const { return targetAltitude; }
    const NavStatus& getNavStatus() const { return nav; }
    const PilotInputs& getInputs() const { return inputs; }

    LaunchController::State getLaunchState() const { return launch.getState(); }
    SoaringController::State getSoaringState() const { return soaring.getState(); }
    const AutoTrim& getAutoTrim() const { return autoTrim; }
    CourseSource getCourseSource() const { return courseSource; }

    // Для телеметрии и ARM; могут быть nullptr.
    ImuSensor* getImuSensor() const { return imuSensor; }
    BarometerSensor* getBarometerSensor() const { return baroSensor; }
    MagnetometerSensor* getMagnetometerSensor() const { return magSensor; }
    GpsSensor* getGpsSensor() const { return gpsSensor; }
    AirspeedSensor* getAirspeedSensor() const { return airspeedSensor; }

    const PidController& getRollPid() const { return pidRoll; }
    const PidController& getPitchPid() const { return pidPitch; }

    void setPIDGains(float kpRoll, float kiRoll, float kdRoll,
                     float kpPitch, float kiPitch, float kdPitch);

    // Высота (м, относительно точки включения), на которой режим
    // с удержанием высоты окажется, если его включить сейчас.
    float getAltitude() const;


private:

    enum class ThrottleMode : uint8_t { PILOT, AUTO, AT_LEAST };
    enum class Failsafe : uint8_t { NONE, GLIDE, RTH };

    // Автовзлёт: программа стартует, когда пилот поднял газ выше этого
    // порога (защита от раскрутки мотора сразу после ARM). Тот же порог
    // взводит запуск с руки.
    static constexpr uint16_t TAKEOFF_TRIGGER_US = 1500;
    static constexpr uint32_t TAKEOFF_THROTTLE_RAMP_MS = 1000;

    // Стик ближе к центру — "пилот отпустил".
    static constexpr int16_t STICK_DEADBAND_US = 25;
    // Стик дальше — пилот вмешался в запуск с руки.
    static constexpr int16_t LAUNCH_ABORT_STICK_US = 150;

    static constexpr float MAX_DT_S = 0.1f;

    ImuSensor* imuSensor;
    BarometerSensor* baroSensor;
    MagnetometerSensor* magSensor;
    GpsSensor* gpsSensor;
    AirspeedSensor* airspeedSensor;

    AutopilotMode currentMode = MODE_MANUAL;
    AutopilotMode previousMode = MODE_MANUAL;

    PidController pidRoll;
    PidController pidPitch;
    AltitudeSpeedController altitudeControl;
    LaunchController launch;
    SoaringController soaring;
    AutoTrim autoTrim;
    bool trimSavePending = false;   // DISARM был, запись ждёт, пока самолёт встанет

    PilotInputs inputs;
    PilotInputs previousInputs;
    ControlCommand pilot;
    ControlCommand output;
    uint16_t pilotThrottle = 1000;

    bool armed = false;
    Failsafe failsafe = Failsafe::NONE;

    uint32_t lastUpdateUs = 0;
    uint32_t nowMs = 0;
    float dtS = 0;

    ThrottleMode throttleMode = ThrottleMode::PILOT;
    float autoThrottlePct = 0;

    float desiredRoll = 0;
    float desiredPitch = 0;
    float targetAltitude = 0;
    float targetCourse = 0;

    NavStatus nav;
    CourseSource courseSource = CourseSource::NONE;
    GeoPoint loiterCenter;
    bool loiterHasCenter = false;
    bool rthArrived = false;

    bool takeoffStarted = false;
    uint32_t takeoffStartMs = 0;
    bool launchCaptured = false;

    float lastAirspeed = 0;
    float airspeedRate = 0;

    // --------------------------------------------------------
    // Датчики
    // --------------------------------------------------------

    bool imuReady() const;

    bool baroReady() const;

    bool airspeedReady() const;

    float altitude() const { return getAltitude(); }

    // Стоит на земле (для записи во флеш): каждый имеющийся датчик
    // согласен; отсутствующий не мешает.
    bool looksLanded() const;

    float climbRate() const;

    static uint16_t percentToUs(float percent);

    // --------------------------------------------------------
    // Крутилки
    // --------------------------------------------------------

    float stabGain() const;

    float maxBank() const;

    float navBankLimit() const { return min(maxBank(), Config::NAV_BANK_LIMIT_DEG); }

    float loiterRadius() const;

    // --------------------------------------------------------
    // Навигация: позиция, дом, курс, скорость
    // --------------------------------------------------------

    void updateNavigation(bool armedEdge);

    void setHome(const char* why);

    void checkGeofence();

    // --------------------------------------------------------
    // Потеря связи
    // --------------------------------------------------------

    void setFailsafe(bool active);

    void runFailsafeGlide();

    // --------------------------------------------------------
    // Режимы
    // --------------------------------------------------------

    void initializeMode();

    void beginReturnHome();

    void runMode();

    float stickRollAngle() const;

    float stickPitchAngle() const;

    static bool stickActive(int16_t value);

    void runStabilize();

    void runAcro();

    // Руль высоты держит высоту; стик тангажа — вручную, отпустил —
    // держит новую высоту.
    void holdAltitudeWithStickOverride();

    // Курс держит крен; стик крена — поворачивать вручную, отпустил —
    // держит новый курс.
    void holdCourseWithStickOverride();

    void runAltHold();

    void runCruise();

    // Круги вокруг center; без GPS — круг постоянным креном на месте.
    void orbit(const GeoPoint& center, bool haveCenter);

    void runLoiter();

    void runReturnHome();

    // Автовзлёт (разбег или с руки по газу): после ARM ждём, пока пилот
    // поднимет газ выше TAKEOFF_TRIGGER_US, затем:
    //   0..1 с — газ плавно до 100%, тангаж 0° (набор скорости);
    //   1..3 с — 100%, тангаж +15° (отрыв);
    //   дальше — 100%, тангаж +10° (набор высоты), пока пилот не
    //   переключит режим. Крылья ровно; стики добавляются поверх.
    void runAutoTakeoff();

    void runLaunch();

    void runAutoLand();

    void runSoaring();

    void steerHomeIfFarOrHoldCourse();

    void runRescue();

    // --------------------------------------------------------
    // Газ, сваливание, стабилизация
    // --------------------------------------------------------

    void autoThrottle();

    // С трубкой Пито: на малой приборной скорости нос не выше
    // горизонта, крен ограничен, газ (в автоматических режимах) — полный.
    void protectFromStall();

    // Крен/тангаж к desiredRoll/desiredPitch; руль направления — стик.
    // Интеграторы копятся, только когда заармлен и самолёт уже летит:
    // в руке до броска и на старте до разбега они накопили бы "крен",
    // которого в полёте нет, и дёрнули бы рули в момент старта.
    void stabilize();

    bool waitingForStart() const;

    // Режимы пилотирования без IMU — просто рули по стикам.
    void stabilizeOrManual();

    // Автоматические режимы без IMU — рули в нейтраль: стики в них не
    // главные (или их нет — failsafe), а без углов вести нечем.
    void stabilizeOrNeutral();

    static bool isNavigationMode(AutopilotMode mode);

    void applyTurnCoordination();

    void applyAutoTrim();

    static int16_t clampCommand(float value);
};
