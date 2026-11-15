#pragma once

// ============================================================
// Chirp на модели (helpers/QuadSim.h): полёт со свипом, запись того же,
// что пишет чёрный ящик (QCHIRP), и «правда» — параметры модели, которые
// идентификация (copter/SysId.h) должна восстановить.
// ============================================================

#include <cmath>
#include <vector>

#include "copter/SysId.h"
#include "helpers/QuadHarness.h"

struct ChirpLog
{
    std::vector<double> r, u, y, yf;   // возбуждение, команда оси, гироскоп до и после фильтров
    int axis = -1;
    copter::ChirpEnd end = copter::ChirpEnd::NONE;
    double maxTilt = 0, minHeight = 1e9, maxHeight = -1e9, maxRate = 0;
    bool saturated = false;
};

// Свип по следующей оси: тумблер CHIRP выключить и включить, «пилот» держит
// высоту газом. Режим и висение — забота вызывающего.
inline ChirpLog flyChirp(QuadHarness& h)
{
    ChirpLog log;
    const double hold = h.sim.height();
    const double hover = h.ctl.getStatus().hoverLearned;
    h.rc.chirp = false;
    h.run(0.05);
    h.rc.chirp = true;
    bool started = false;
    for (long i = 0; i < 200000; ++i)
    {
        const double thr = hover + 0.03 * (hold - h.sim.height()) - 0.05 * h.sim.climbRate();
        h.rc.throttle = static_cast<float>(std::fmin(0.95, std::fmax(0.05, thr)));
        h.step();
        const copter::CopterStatus& st = h.ctl.getStatus();
        if (st.chirpActive)
        {
            started = true;
            log.axis = st.chirpAxis;
            log.r.push_back(st.chirpExcitation);
            log.u.push_back(st.chirpCommand);
            log.y.push_back(h.lastSens.gyro[st.chirpAxis]);
            log.yf.push_back(st.rates[st.chirpAxis]);
            log.maxTilt = std::fmax(log.maxTilt, h.sim.tiltDeg());
            log.minHeight = std::fmin(log.minHeight, h.sim.height());
            log.maxHeight = std::fmax(log.maxHeight, h.sim.height());
            log.maxRate = std::fmax(log.maxRate, std::fabs(static_cast<double>(h.lastSens.gyro[st.chirpAxis])));
            log.saturated = log.saturated || st.saturated;
        }
        else if (started || i > 300)
        {
            break;
        }
    }
    log.end = h.ctl.getStatus().chirpEnd;
    h.rc.chirp = false;
    h.run(0.3);
    return log;
}

// Параметры модели у точки висения: то, что должна найти идентификация.
// Объект — от команды оси на входе смесителя до гироскопа в том же цикле.
struct TruePlant
{
    double gain = 0;    // (°/с²) на единицу команды
    double tau = 0;     // запаздывание мотора, с
    double delay = 0;   // регулятор + цикл + фильтр чипа, с
    double zero = 0;    // рысканье: момент разгона ротора, с
};

inline TruePlant truePlant(const QuadHarness& h, int axis)
{
    const QuadSimParams& sp = h.sim.params();
    const copter::CopterTune& t = h.tune;
    const copter::CopterStatus& st = h.ctl.getStatus();
    const int n = sp.frame.count;
    const double volts = h.sim.batteryVolts();
    double vComp = 1.0;
    if (h.ctl.getSettings().voltageCompensation && st.volts > 1.0f)
        vComp = 1.0 + (std::fmin(1.3, std::fmax(1.0, static_cast<double>(t.volts / st.volts))) - 1.0) * t.vbatSagCompensation;

    double torque = 0, tauSum = 0, zeroSum = 0;
    for (int i = 0; i < n; ++i)
    {
        const auto& m = sp.motor[i];
        const double w = h.sim.rotorHz(i) * 2.0 * M_PI;
        // скважность -> доля тяги смесителя (обратно к линеаризации), её наклон
        const double span = (1.0 - t.idle) * t.motorOutputLimit;
        const double signal = std::fmin(1.0, std::fmax(1e-4, (h.out.motor[i] - t.idle) / span / vComp));
        double lo = 0.0, hi = 1.0;
        for (int it = 0; it < 40; ++it)
        {
            const double mid = 0.5 * (lo + hi);
            (copter::Mixer::linearize(static_cast<float>(mid), t.thrustLinear) < signal ? lo : hi) = mid;
        }
        const double share = std::fmax(1e-3, 0.5 * (lo + hi));
        const double dDuty = span * vComp * ((1.0 - t.thrustLinear) + t.thrustLinear * 0.5 / std::sqrt(share));
        // мотор: установившиеся обороты по скважности и постоянная времени
        const double stiff = 1.0 / (m.R * m.kv * m.kv) + 2.0 * m.kQ * w;
        const double dOmega = volts / (m.R * m.kv) / stiff;
        tauSum += m.J / stiff;
        zeroSum += m.J / (2.0 * m.kQ * w);
        const double rx = sp.frame.place[i].x * sp.armM - sp.cgX;
        const double ry = sp.frame.place[i].y * sp.armM - sp.cgY;
        const double spin = static_cast<double>(static_cast<int8_t>(sp.frame.place[i].spin));
        const double dThrust = 2.0 * m.kT * w * dOmega * dDuty;
        const double dDrag = 2.0 * m.kQ * w * dOmega * dDuty;
        if (axis == 0) torque += -ry * dThrust * sp.frame.mix[i].roll;
        else if (axis == 1) torque += rx * dThrust * sp.frame.mix[i].pitch;
        else torque += spin * dDrag * sp.frame.mix[i].yaw;
    }
    const double inertia = axis == 0 ? sp.jx : axis == 1 ? sp.jy : sp.jz;
    const double dt = 1.0 / t.loopHz;
    TruePlant p;
    p.gain = std::fabs(torque) / inertia * 57.29578;
    p.tau = tauSum / n;
    p.zero = axis == 2 ? zeroSum / n : 0.0;
    // команда цикла k действует с цикла k + d (очередь регулятора) и держится
    // цикл (ещё полцикла в среднем); фильтр чипа — два звена PT1
    const double steps = std::lround(sp.escDelayS / dt) + 1;
    p.delay = (steps + 0.5) * dt + 2.0 / (2.0 * M_PI * sp.imuAafHz * 1.553774);
    return p;
}

inline copter::sysid::Response chirpResponse(const ChirpLog& log, double loopHz)
{
    return copter::sysid::estimate(log.r, log.u, log.y, log.yf, loopHz);
}

// ---------------- автонастройка на модели ----------------

constexpr double STEP_HOLD_S = 0.25, STEP_WINDOW_S = 0.45;
constexpr float STEP_STICK = 0.4f;   // ступенька — 40 % стика

#include "copter/AutoTune.h"

// Регулятор оси как он задан настройкой (D — наибольший: d_max).
inline copter::autotune::Controller controllerOf(const copter::CopterTune& t, int axis)
{
    namespace at = copter::autotune;
    const copter::AxisGains& g = axis == 0 ? t.roll : axis == 1 ? t.pitch : t.yaw;
    at::Controller c;
    c.p = g.p;
    c.i = g.i;
    c.d = std::fmax(g.d, t.pid.dMax[axis]);
    c.ff = g.ff;
    c.dt = 1.0 / t.loopHz;
    if (t.filters.valid)
    {
        const copter::FilterSpec& f = t.filters;
        c.dLpf1 = { static_cast<int>(f.dterm1.type), f.dtermDyn.on() ? f.dtermDyn.minHz : f.dterm1.hz };
        c.dLpf2 = { static_cast<int>(f.dterm2.type), f.dterm2.hz };
        if (f.dtermNotch.on())
        {
            c.dNotchHz = f.dtermNotch.hz;
            c.dNotchCutoffHz = f.dtermNotch.cutoffHz;
        }
    }
    else
    {
        c.dLpf1 = { at::PT1, t.dtermLpfHz };
    }
    c.outputLpfHz = axis == 2 ? t.yawLpfHz : 0.0;
    c.levelGain = axis == 2 ? 0.0 : t.levelGain;
    return c;
}

struct Measured
{
    ChirpLog log[3];
    copter::sysid::Response response[3];
    copter::autotune::AxisData data[3];
    bool ok = true;
};

// Три свипа подряд (крен, тангаж, рысканье) и всё, что по ним считается.
inline Measured measureByChirp(QuadHarness& h)
{
    Measured m;
    for (int a = 0; a < 3; ++a)
    {
        m.log[a] = flyChirp(h);
        m.ok = m.ok && m.log[a].axis == a && m.log[a].end == copter::ChirpEnd::DONE;
        m.response[a] = chirpResponse(m.log[a], h.tune.loopHz);
        m.data[a].plant = copter::sysid::fit(m.response[a], a == 2);
        m.data[a].gyroFilter = copter::autotune::gyroFilterCurve(m.response[a]);
        m.data[a].noise = copter::autotune::noiseCurve(m.response[a]);
        m.data[a].stepDps = h.tune.rates[a].apply(STEP_STICK);
        m.ok = m.ok && m.data[a].plant.valid();
    }
    return m;
}

// Настройка с предложенными коэффициентами и срезом ФНЧ D.
inline copter::CopterTune applyProposal(const copter::CopterTune& t, const copter::autotune::Proposal& p)
{
    copter::CopterTune n = t;
    copter::AxisGains* g[3] = { &n.roll, &n.pitch, &n.yaw };
    for (int a = 0; a < 3; ++a)
    {
        const copter::autotune::Controller& c = p.axis[a].controller;
        g[a]->p = static_cast<float>(c.p);
        g[a]->i = static_cast<float>(c.i);
        g[a]->d = static_cast<float>(c.d);
        g[a]->ff = static_cast<float>(c.ff);
        n.pid.dMax[a] = 0.0f;
    }
    n.gainsFromModel = false;
    n.dtermLpfHz = static_cast<float>(p.dtermLpfHz);
    n.filters.dterm1.hz = n.dtermLpfHz;
    n.filters.dtermDyn = copter::DynLpf();
    return n;
}

// Ступенька стиком в ACRO на модели коптера: стик на STEP_HOLD_S и обратно.
// Время до 90 %, перелёт и интеграл модуля ошибки за всё окно (туда и обратно).
struct RateStep
{
    double t90 = -1, overshoot = 0;
    double error = 0;   // интеграл |уставка − отклик|, доля ступеньки × с
    bool flying = false;
};

// Перед ступенькой: набрать высоту и повисеть ровно (ступенька по крену —
// это наклон на десятки градусов и потеря высоты).
inline void settleAt(QuadHarness& h, double height)
{
    h.rc.mode = copter::CopterMode::ALT_HOLD;
    h.rc.roll = h.rc.pitch = h.rc.yaw = 0.0f;
    for (int i = 0; i < 30000 && h.sim.height() < height - 0.3; ++i)
    {
        h.rc.throttle = 0.85f;
        h.step();
    }
    h.rc.throttle = 0.5f;
    h.run(2.5);
}

inline RateStep acroStep(QuadHarness& h, int axis)
{
    RateStep r;
    settleAt(h, 15.0);
    h.rc.mode = copter::CopterMode::ACRO;
    h.rc.throttle = h.ctl.getStatus().hoverLearned;
    h.run(0.3);
    float* sticks[3] = { &h.rc.roll, &h.rc.pitch, &h.rc.yaw };
    const double target = h.tune.rates[axis].apply(STEP_STICK);
    const double t0 = h.sim.now();
    double peak = 0;
    const int hold = static_cast<int>(STEP_HOLD_S * h.tune.loopHz), steps = static_cast<int>(STEP_WINDOW_S * h.tune.loopHz);
    for (int i = 0; i < steps; ++i)
    {
        *sticks[axis] = i < hold ? STEP_STICK : 0.0f;
        h.step();
        const QuadSim::Vec w = h.sim.rates();
        const double rate = (axis == 0 ? w.x : axis == 1 ? w.y : w.z) * 57.29578;
        r.error += std::fabs(h.ctl.getStatus().setpoint[axis] - rate) / target / h.tune.loopHz;
        if (i < hold)
        {
            peak = std::fmax(peak, rate / target);
            if (r.t90 < 0 && rate >= 0.9 * target) r.t90 = h.sim.now() - t0;
        }
    }
    r.overshoot = std::fmax(0.0, peak - 1.0);
    r.flying = h.ctl.isArmed() && !h.sim.grounded();
    h.rc.mode = copter::CopterMode::ANGLE;
    h.run(1.0);
    return r;
}
