#pragma once
#include <Arduino.h>

#include "config/Channels.h"
#include "config/Config.h"
#include "rc/RcChannelState.h"
#include "sensors/SensorInterface.h"
#include "sensors/imu/ImuOrientation.h"

// ============================================================
// НАЗЕМНАЯ КАЛИБРОВКА САМОЛЁТА
//
// Самолёт с хвостовым колесом стоит не ровно, а носом вверх, и на
// сколько — заранее не знает никто. Калибровка меряет это сама, в два
// шага, и запоминает во флеше (рядом с установкой IMU):
//
//   1. ГОРИЗОНТ — хвост поднят, фюзеляж горизонтально, как в полёте
//      (основные колёса на земле). Это 0° тангажа и крена для всех
//      режимов: горизонт больше не берётся из положения при включении.
//      Где у платы нос, не меняется (ImuOrientation::withLevel()).
//   2. СТОЯНКА — самолёт на колёсах, руки убраны. Стояночный угол:
//      "стоит носом вверх на 9.4°" — от него считает автовзлёт
//      (autopilot/TakeoffController.h): насколько поднять хвост, где
//      хвостовое колесо коснётся полосы, тот ли самолёт стоит перед
//      стартом.
//
// Шаг засчитывается сам, когда самолёт неподвижен CAL_STILL_MS подряд.
// Шаг 2 ждёт, пока самолёт переставят (движение после сигнала), или
// CAL_PARK_WAIT_MS — у носового колеса стоянка бывает той же позой.
// Сигналы (control/SignalPlayer.h, их играет FlightController): 1 —
// началось / шаг 1 засчитан, 2 — готово, 3 — ошибка, ничего не сохранено.
//
// Запуск — с пульта без ARM (watchSwitch(): тумблер CAL_SWITCH вниз-вверх
// CAL_SWITCH_FLIPS раз за CAL_SWITCH_WINDOW_MS, газ внизу) или 'c' в
// консоли. Не блокирует полётный цикл: приёмник и failsafe работают,
// ARM на время калибровки запрещён, щелчок ARM её отменяет.
// ============================================================

class GroundCalibration
{
public:

    enum class Stage : uint8_t { IDLE, LEVEL, PARK, DONE, FAILED };

    explicit GroundCalibration(Config::LandingGear landingGear = Config::LANDING_GEAR)
        : gear(landingGear)
    {
    }

    void setGear(Config::LandingGear landingGear) { gear = landingGear; }

    // Жест с пульта. Вызывать каждый такт, пока связь есть и нет ARM;
    // true — тумблер щёлкнули нужное число раз (вниз и обратно вверх).
    bool watchSwitch(const RcChannelState& rc, uint32_t nowMs)
    {
        const bool on = rc.get(Config::CAL_SWITCH) >= Config::SWITCH_ON_US;
        const bool throttleLow = rc.get(Channels::THROTTLE) < Config::THROTTLE_LOW_US;
        if (!switchKnown || !throttleLow)
        {
            switchKnown = true;
            switchOn = on;
            flips = 0;
            return false;
        }

        const bool flipped = switchOn && !on;   // вниз и обратно вверх
        switchOn = on;
        if (!flipped) return false;

        if (flips == 0 || nowMs - firstFlipMs > Config::CAL_SWITCH_WINDOW_MS)
        {
            flips = 0;
            firstFlipMs = nowMs;
        }
        if (++flips < Config::CAL_SWITCH_FLIPS) return false;

        flips = 0;
        return true;
    }

    // ARM, потеря связи — счёт щелчков заново.
    void forgetSwitch()
    {
        switchKnown = false;
        flips = 0;
    }

    // false — не началась (причина в Serial, сигнал ошибки).
    bool start(const ImuSensor* imu, uint32_t nowMs, const char* source)
    {
        Serial.print("Калибровка самолёта (");
        Serial.print(source);
        Serial.println(")");

        ImuOrientation probe;
        if (!imu || !imu->isAvailable())
        {
            fail("IMU не отвечает", nowMs);
            return false;
        }
        if (!imu->getOrientation(probe))
        {
            fail("этот IMU не умеет калибровку шасси", nowMs);
            return false;
        }

        failure = nullptr;
        enter(Stage::LEVEL, nowMs);
        pendingSignal = 1;
        if (gear == Config::LandingGear::TAIL_WHEEL)
        {
            Serial.println("  Шаг 1/2 — ГОРИЗОНТ: поднимите хвост, пока фюзеляж не станет горизонтально,");
            Serial.println("  как в полёте (основные колёса на земле), и держите неподвижно ~2 с.");
        }
        else
        {
            Serial.println("  Шаг 1/2 — ГОРИЗОНТ: поставьте самолёт ровно, как в горизонтальном полёте,");
            Serial.println("  и держите неподвижно ~2 с.");
        }
        return true;
    }

    void cancel(const char* why)
    {
        if (!isActive()) return;
        stage = Stage::IDLE;
        Serial.print("Калибровка самолёта отменена: ");
        Serial.println(why);
    }

    // Каждый такт. signalBusy — сигнал ещё звучит: рули кивают, пилот
    // слушает — шаг не засчитывается.
    void update(ImuSensor* imu, bool armed, bool signalBusy, uint32_t nowMs)
    {
        if (!isActive()) return;
        if (armed)
        {
            cancel("ARM");
            return;
        }
        if (!imu || !imu->isAvailable())
        {
            fail("IMU перестал отвечать", nowMs);
            return;
        }
        if (nowMs - stageSinceMs > Config::CAL_STEP_TIMEOUT_MS)
        {
            fail("самолёт так и не замер неподвижно", nowMs);
            return;
        }
        if (signalBusy)
        {
            // Ожидание шага 2 — от конца последнего сигнала.
            resetStill();
            quietSinceValid = false;
            return;
        }
        if (!quietSinceValid)
        {
            quietSinceValid = true;
            quietSinceMs = nowMs;
        }

        float up[3];
        if (!stillSample(*imu, up))
        {
            moved = true;
            resetStill();
            return;
        }
        if (stage == Stage::PARK && !moved && nowMs - quietSinceMs < Config::CAL_PARK_WAIT_MS)
        {
            return;   // ещё не переставили
        }

        if (count == 0) stillSinceMs = nowMs;
        for (uint8_t i = 0; i < 3; ++i) sum[i] += up[i];
        count++;
        if (nowMs - stillSinceMs < Config::CAL_STILL_MS) return;

        float average[3];
        for (uint8_t i = 0; i < 3; ++i) average[i] = sum[i] / count;
        if (stage == Stage::LEVEL) captureLevel(*imu, average, nowMs);
        else captureParking(*imu, average, nowMs);
    }

    bool isActive() const { return stage == Stage::LEVEL || stage == Stage::PARK; }
    Stage getStage() const { return stage; }
    uint32_t getStageSinceMs() const { return stageSinceMs; }
    const char* getFailure() const { return failure; }
    float getParkPitch() const { return parkPitch; }
    float getParkRoll() const { return parkRoll; }

    // Сколько сигналов сыграть (0 — нечего) — забирается один раз.
    uint8_t takeSignal()
    {
        const uint8_t n = pendingSignal;
        pendingSignal = 0;
        return n;
    }

    // Коротко — для OLED (до 5 символов).
    static const char* stageShort(Stage s)
    {
        switch (s)
        {
            case Stage::LEVEL:  return "CAL1";
            case Stage::PARK:   return "CAL2";
            case Stage::DONE:   return "CALOK";
            case Stage::FAILED: return "CALER";
            default:            return "";
        }
    }


private:

    Config::LandingGear gear;
    Stage stage = Stage::IDLE;
    uint32_t stageSinceMs = 0;
    const char* failure = nullptr;
    uint8_t pendingSignal = 0;

    // Неподвижность и среднее "верха" за неё.
    float sum[3] = {};
    uint32_t count = 0;
    uint32_t stillSinceMs = 0;
    bool moved = false;               // шаг 2: самолёт переставляли после сигнала
    bool quietSinceValid = false;     // сигнал отзвучал...
    uint32_t quietSinceMs = 0;        // ...вот когда

    ImuOrientation level;             // горизонт шага 1 (ещё не сохранён)
    float parkPitch = 0;
    float parkRoll = 0;

    // Жест тумблером.
    bool switchKnown = false;
    bool switchOn = false;
    uint8_t flips = 0;
    uint32_t firstFlipMs = 0;

    void enter(Stage next, uint32_t nowMs)
    {
        stage = next;
        stageSinceMs = nowMs;
        moved = false;
        quietSinceValid = false;
        resetStill();
    }

    void resetStill()
    {
        sum[0] = sum[1] = sum[2] = 0;
        count = 0;
    }

    // Самолёт неподвижен: не вращается и видит только тяжесть.
    static bool stillSample(const ImuSensor& imu, float up[3])
    {
        if (!imu.readChipUp(up)) return false;
        const ImuData& d = imu.getImuData();
        const float rate = max(max(fabsf(d.gyroX), fabsf(d.gyroY)), fabsf(d.gyroZ));
        const float g = sqrtf(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
        return rate < Config::CAL_STILL_DPS && fabsf(g - 1.0f) < Config::CAL_STILL_G;
    }

    void captureLevel(const ImuSensor& imu, const float levelUp[3], uint32_t nowMs)
    {
        ImuOrientation current;
        imu.getOrientation(current);
        const char* error = current.withLevel(levelUp, level);
        if (error)
        {
            fail(error, nowMs);
            return;
        }

        enter(Stage::PARK, nowMs);
        pendingSignal = 1;
        Serial.println("  Шаг 1 засчитан: горизонт.");
        Serial.println("  Шаг 2/2 — СТОЯНКА: поставьте самолёт на колёса на ровное место и уберите руки.");
    }

    void captureParking(ImuSensor& imu, const float parkUp[3], uint32_t nowMs)
    {
        float pitch, roll;
        level.attitudeOf(parkUp, pitch, roll);

        Serial.print("  Стоянка: нос ");
        Serial.print(pitch >= 0 ? "вверх на " : "вниз на ");
        Serial.print(fabsf(pitch), 1);
        Serial.print("°, крен ");
        Serial.print(roll, 1);
        Serial.println("°");

        const char* problem = checkParking(pitch, roll);
        if (problem)
        {
            fail(problem, nowMs);
            return;
        }
        if (!imu.setGroundCalibration(level, parkUp))
        {
            fail("IMU не сохранил калибровку", nowMs);
            return;
        }

        parkPitch = pitch;
        parkRoll = roll;
        stage = Stage::DONE;
        stageSinceMs = nowMs;
        pendingSignal = 2;
        Serial.println("Калибровка самолёта сохранена: горизонт и стоянка.");
    }

    const char* checkParking(float pitch, float roll) const
    {
        if (fabsf(roll) > Config::CAL_PARK_MAX_ROLL_DEG)
        {
            return "на стоянке большой крен (больше CAL_PARK_MAX_ROLL_DEG) — нужно ровное место";
        }
        if (gear == Config::LandingGear::TAIL_WHEEL)
        {
            if (pitch < Config::CAL_TAIL_PARK_MIN_DEG)
            {
                return "нос почти не задран — хвост не опустили? У хвостового колеса на стоянке "
                       "нос вверх на CAL_TAIL_PARK_MIN_DEG..CAL_TAIL_PARK_MAX_DEG";
            }
            if (pitch > Config::CAL_TAIL_PARK_MAX_DEG)
            {
                return "нос задран больше CAL_TAIL_PARK_MAX_DEG — в шаге 1 хвост был ниже горизонта?";
            }
            return nullptr;
        }
        if (fabsf(pitch) > Config::CAL_NOSE_PARK_MAX_DEG)
        {
            return "носовое колесо, а самолёт стоит не ровно (больше CAL_NOSE_PARK_MAX_DEG) — "
                   "проверьте LANDING_GEAR в Config.h";
        }
        return nullptr;
    }

    void fail(const char* reason, uint32_t nowMs)
    {
        stage = Stage::FAILED;
        stageSinceMs = nowMs;
        failure = reason;
        pendingSignal = 3;
        Serial.print("Калибровка самолёта: ОШИБКА — ");
        Serial.print(reason);
        Serial.println(". Ничего не сохранено.");
    }
};
