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
        if (rpmTelemetry)
        {
            // двунаправленный DShot: настоящие обороты каждого мотора
            s.motorHzOk = true;
            for (uint8_t i = 0; i < tune.frame.count && i < copter::MAX_MOTORS; ++i) s.motorHz[i] = static_cast<float>(sim.rotorHz(wiring[i]));
        }
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
        // как пилот: газ вверх, у нужной высоты стик плавно к центру (отпущенный
        // на скорости стик — торможение, и высота запомнится выше)
        const double edge = 0.5 + ctl.getSettings().alt.deadband;
        for (int i = 0; i < 40000 && sim.height() < height - 0.05; ++i)
        {
            const double left = (height - sim.height()) / 3.0;
            rc.throttle = static_cast<float>(edge + (0.9 - edge) * std::fmin(1.0, std::fmax(0.03, left)));
            step();
        }
        rc.throttle = 0.5f;
        run(3.0);
        return std::fabs(sim.height() - height) < 2.0;
    }

    // Модель GPS: фикс 10 Гц, шум положения и скорости, дом — где заармились;
    // курс носа — компас (точно; compassErrorDeg — врёт на столько) или нет
    // (контроллер учит его по полёту). gpsDelayS — фикс описывает то, что было
    // столько назад (приёмник считает и шлёт решение не мгновенно).
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
        // прошлое — для задержки фикса
        gpsPast[steps % GPS_PAST] = { sim.position().x, sim.position().y, sim.velocity().x, sim.velocity().y };
        g = gpsLast;
        g.fresh = false;
        // компас отвечает каждый цикл и GPS ему не нужен
        double yaw = sim.yawDeg() + compassErrorDeg;
        while (yaw < 0) yaw += 360.0;
        while (yaw >= 360.0) yaw -= 360.0;
        g.headingDeg = static_cast<float>(yaw);
        g.headingTrusted = compass;
        gpsLast.headingDeg = g.headingDeg;
        gpsLast.headingTrusted = g.headingTrusted;
        if (gpsLost || steps % std::lround(tune.loopHz / 10.0) != 0) return;
        auto noise = [this](double sigma) {
            gpsSeed = gpsSeed * 1103515245u + 12345u;
            return sigma * (static_cast<double>((gpsSeed >> 8) & 0xFFFF) / 32768.0 - 1.0);
        };
        g.fresh = true;
        g.fix = true;
        g.sats = gpsSats;
        g.home = gpsHomeSet;
        long back = std::lround(gpsDelayS * tune.loopHz);
        if (back > steps) back = steps;
        if (back > GPS_PAST - 1) back = GPS_PAST - 1;
        const GpsPast& was = gpsPast[(steps - back) % GPS_PAST];
        g.northM = static_cast<float>(was.n - homeN + noise(gpsNoisePosM));
        g.eastM = static_cast<float>(was.e - homeE + noise(gpsNoisePosM));
        g.velN = static_cast<float>(was.vn + noise(gpsNoiseVelMs));
        g.velE = static_cast<float>(was.ve + noise(gpsNoiseVelMs));
        gpsLast = g;
    }

    bool gpsOn = false;        // есть GPS
    bool compass = true;       // и компас
    bool gpsLost = false;      // GPS замолчал
    double gpsDelayS = 0.0;        // фикс старше на столько
    double gpsNoisePosM = 0.4;     // шум положения (±, равномерный)
    double gpsNoiseVelMs = 0.1;    // и скорости
    double compassErrorDeg = 0.0;  // компас врёт на столько
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
    bool rpmTelemetry = false;               // обороты моторов по DShot (иначе контроллер берёт модель)
    uint8_t wiring[copter::MAX_MOTORS];      // выход i -> мотор wiring[i] (перепутанные провода)
    long steps = 0;

private:
    struct GpsPast
    {
        double n, e, vn, ve;
    };
    static constexpr long GPS_PAST = 4096;
    GpsPast gpsPast[GPS_PAST] = {};
    double motorLpf[copter::MAX_MOTORS] = {};
};
