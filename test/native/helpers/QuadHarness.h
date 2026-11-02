#pragma once

// ============================================================
// Стенд для замкнутых полётов мультикоптера: настоящий
// CopterController (copter/CopterController.h) + модель
// (helpers/QuadSim.h). Пульт — поля rc; шаг — один цикл стабилизации.
// По ходу копится статистика, по которой тесты судят о полёте.
// ============================================================

#include <cmath>
#include <functional>

#include "copter/CopterController.h"
#include "helpers/QuadSim.h"

struct QuadStats
{
    double maxTilt = 0;
    double maxAbsRollErr = 0;
    double sumRate2 = 0;          // для СКЗ угловой скорости (качание)
    double sumMotorHp2 = 0;       // СКЗ «дрожи» команд моторов
    long samples = 0;
    long saturated = 0;
    double minHeight = 1e9, maxHeight = -1e9;

    void reset() { *this = QuadStats(); }
    double rateRms() const { return samples ? std::sqrt(sumRate2 / samples) : 0; }
    double motorNoiseRms() const { return samples ? std::sqrt(sumMotorHp2 / samples) : 0; }
};

class QuadHarness
{
public:
    QuadHarness(const copter::BuildSpec& build, const QuadPerturb& perturb, uint32_t seed = 1,
                copter::EscProtocol esc = copter::EscProtocol::DSHOT600, float loopHz = 1000.0f,
                const copter::CopterSettings& settings = copter::CopterSettings())
        : tune(copter::deriveTune(build, loopHz, esc)), sim(quadParamsFrom(tune, build.battery, perturb, seed), loopHz)
    {
        ctl.configure(tune, settings);
        rc.linkOk = true;
        rc.mode = copter::CopterMode::ANGLE;
        for (uint8_t i = 0; i < copter::MAX_MOTORS; ++i) wiring[i] = i;
    }

    // Другая настройка контроллера на той же физике (дефолты Betaflight, свои PID).
    void useTune(const copter::CopterTune& t, const copter::CopterSettings& settings = copter::CopterSettings())
    {
        tune = t;
        ctl.configure(t, settings);
    }

    void step()
    {
        copter::CopterSensors s;
        sim.readImu(s.gyro, s.accel);
        s.imuOk = imuOk;
        s.baroOk = baroOk;
        s.baroAltM = baroOk ? sim.readBaro() + 120.0f : 0.0f;   // над уровнем моря — не ноль
        s.batteryVolts = sim.readVolts();
        feedGps(s.gps);
        out = ctl.update(s, rc);
        lastSens = s;
        float motor[copter::MAX_MOTORS] = {};
        for (uint8_t i = 0; i < out.count; ++i) motor[wiring[i]] = out.motor[i];
        sim.setReversed(out.reversed);
        sim.step(motor, out.spinning);

        // статистика
        const auto w = sim.rates();
        stats.sumRate2 += w.x * w.x + w.y * w.y;
        double hp = 0;
        for (uint8_t i = 0; i < out.count; ++i)
        {
            const double d = out.motor[i] - motorLpf[i];
            motorLpf[i] += 0.05 * d;   // ~8 Гц
            hp += d * d;
        }
        stats.sumMotorHp2 += out.count ? hp / out.count : 0;
        stats.samples++;
        if (ctl.getStatus().saturated) stats.saturated++;
        stats.maxTilt = std::fmax(stats.maxTilt, sim.tiltDeg());
        stats.minHeight = std::fmin(stats.minHeight, sim.height());
        stats.maxHeight = std::fmax(stats.maxHeight, sim.height());
        ++steps;
        if (afterStep) afterStep();
    }

    void run(double seconds)
    {
        const long n = std::lround(seconds * tune.loopHz);
        for (long i = 0; i < n; ++i) step();
    }

    // Встать на землю, дождаться готовности и заармиться.
    bool armOnGround(copter::CopterMode mode = copter::CopterMode::ANGLE)
    {
        rc.mode = mode;
        rc.throttle = mode == copter::CopterMode::ALT_HOLD ? 0.3f : 0.0f;
        rc.armSwitch = false;
        run(2.5);
        rc.armSwitch = true;
        run(0.05);
        return ctl.isArmed();
    }

    // Подняться на высоту (ALT_HOLD) и повисеть.
    bool takeoffTo(double height)
    {
        if (!armOnGround(copter::CopterMode::ALT_HOLD)) return false;
        rc.throttle = 0.9f;
        for (int i = 0; i < 30000 && sim.height() < height; ++i) step();
        rc.throttle = 0.5f;
        run(3.0);
        return std::fabs(sim.height() - height) < 2.0;
    }

    // Модель GPS: фикс 10 Гц, шум положения и скорости, дом — где заармились;
    // курс носа — компас (точно) или нет (контроллер учит его по курсу GPS).
    void feedGps(copter::GpsInput& g)
    {
        if (!gpsOn) return;
        const bool armed = ctl.isArmed();
        if (armed && !wasArmedGps)
        {
            homeN = sim.position().x;
            homeE = sim.position().y;
            gpsHomeSet = true;
        }
        wasArmedGps = armed;
        g = gpsLast;
        g.fresh = false;
        if (gpsLost || steps % std::lround(tune.loopHz / 10.0) != 0) return;
        auto noise = [this](double sigma) {
            gpsSeed = gpsSeed * 1103515245u + 12345u;
            return sigma * (static_cast<double>((gpsSeed >> 8) & 0xFFFF) / 32768.0 - 1.0);
        };
        g.fresh = true;
        g.fix = true;
        g.sats = gpsSats;
        g.home = gpsHomeSet;
        g.northM = static_cast<float>(sim.position().x - homeN + noise(0.4));
        g.eastM = static_cast<float>(sim.position().y - homeE + noise(0.4));
        g.velN = static_cast<float>(sim.velocity().x + noise(0.1));
        g.velE = static_cast<float>(sim.velocity().y + noise(0.1));
        double yaw = sim.yawDeg();
        if (yaw < 0) yaw += 360.0;
        g.headingDeg = static_cast<float>(yaw);
        g.headingTrusted = compass;
        gpsLast = g;
    }

    bool gpsOn = false;        // есть GPS
    bool compass = true;       // и компас
    bool gpsLost = false;      // GPS замолчал
    uint8_t gpsSats = 14;
    bool gpsHomeSet = false, wasArmedGps = false;
    double homeN = 0, homeE = 0;
    uint32_t gpsSeed = 12345;
    copter::GpsInput gpsLast;

    copter::CopterTune tune;
    QuadSim sim;
    copter::CopterController ctl;
    copter::CopterRc rc;
    copter::CopterOutputs out;
    QuadStats stats;
    copter::CopterSensors lastSens;          // что видел контроллер в последнем шаге
    std::function<void()> afterStep;         // после каждого шага (чёрный ящик в тестах)
    bool baroOk = true;
    bool imuOk = true;
    uint8_t wiring[copter::MAX_MOTORS];      // выход i -> мотор wiring[i] (перепутанные провода)
    long steps = 0;

private:
    double motorLpf[copter::MAX_MOTORS] = {};
};
