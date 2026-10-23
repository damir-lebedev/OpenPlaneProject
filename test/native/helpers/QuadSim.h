#pragma once

// ============================================================
// Модель мультикоптера для замкнутых симуляций (только тесты).
//
// Твёрдое тело с 6 степенями свободы и честными моторами:
//   • каждый мотор — та же модель, что в copter/Propulsion.h (ток через
//     сопротивление и противо-ЭДС, тяга kT·ω², момент kQ·ω², разгон
//     ротора с винтом), но с параметрами, искажёнными относительно
//     тех, по которым прошивка выводила настройку: «не тот» мотор,
//     «не тот» винт, тяжелее, инерционнее, центр тяжести смещён,
//     моторы разбросаны между собой;
//   • батарея проседает под током (внутреннее сопротивление);
//   • реактивный момент винтов и момент разгона роторов — по рысканью;
//   • сопротивление воздуха (линейное от винтов и квадратичное от
//     рамы), ветер с порывами;
//   • регулятор отвечает с задержкой (кадр протокола + цикл);
//   • датчики: гироскоп и акселерометр с шумом и вибрацией моторов
//     (частота вращения, удвоенная, проход лопастей — амплитуда растёт
//     с оборотами), с фильтром самого чипа и выборкой на частоте цикла
//     (вибрация выше половины частоты цикла заворачивается, как на
//     плате); барометр с шумом и выборкой 50 Гц;
//   • земля: стоит ровно, пока тяги не хватает оторваться; лежит на
//     боку/спине, если опрокинут.
//
// Оси: Земля — NED (север, восток, вниз), рама — FRD (вперёд, вправо,
// вниз). Наружу — в соглашениях прошивки (ImuData: гироскоп в
// авиационных знаках, акселерометр X нос, Y влево, Z вверх).
// ============================================================

#include <cmath>
#include <random>

#include "copter/Presets.h"

struct QuadPerturb
{
    double thrust = 1.0;          // kT винта
    double torque = 1.0;          // kQ
    double resistance = 1.0;      // сопротивление мотора
    double rotorInertia = 1.0;
    double bodyInertia = 1.0;
    double mass = 1.0;
    double motorSpread = 0.0;     // разброс моторов между собой (доля, ±)
    double cgX = 0.0, cgY = 0.0;  // смещение центра тяжести, доля плеча
    double vibration = 1.0;       // множитель вибраций
    double escDelayS = 0.0;       // сверх цикла
};

struct QuadSimParams
{
    copter::Frame frame;
    double armM = 0.1;
    double massKg = 0.5;
    double jx = 1e-3, jy = 1e-3, jz = 2e-3;
    struct Motor
    {
        double kT, kQ, R, kv, J;
    } motor[copter::MAX_MOTORS] = {};
    double cgX = 0, cgY = 0;     // м
    int cells = 4;
    double cellOcv = 3.95;       // В, без нагрузки
    double packR = 0.06;         // Ом
    double escDelayS = 0.0002;
    double dragLin = 0.25;       // (Н/(м/с)) на кг массы: сопротивление винтов
    double dragQuad = 0.004;     // ½ρ·Cd·S рамы, кг/м
    double gyroNoiseDps = 0.15;
    double accelNoiseG = 0.01;
    double vibDpsAtFull = 40.0;  // вибрация гироскопа на полном газе (амплитуда по оси)
    double vibGAtFull = 1.0;
    double imuAafHz = 400.0;     // фильтр чипа
    double baroNoiseM = 0.12;
    int blades = 3;
    uint32_t seed = 1;
};

// Параметры модели по настройке прошивки, с искажением.
inline QuadSimParams quadParamsFrom(const copter::CopterTune& t, const copter::BatterySpec& battery, const QuadPerturb& p,
                                    uint32_t seed)
{
    QuadSimParams s;
    s.seed = seed;
    s.frame = t.frame;
    s.armM = t.armM;
    s.massKg = t.massKg * p.mass;
    // инерция растёт вместе с массой (тяжелее батарея — на той же раме)
    s.jx = t.jRoll * p.bodyInertia * (0.5 + 0.5 * p.mass);
    s.jy = t.jPitch * p.bodyInertia * (0.5 + 0.5 * p.mass);
    s.jz = t.jYaw * p.bodyInertia * (0.5 + 0.5 * p.mass);
    std::mt19937 rng(seed * 7919u + 13u);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (uint8_t i = 0; i < t.frame.count; ++i)
    {
        const double k = 1.0 + p.motorSpread * u(rng);
        s.motor[i].kT = t.prop.kT * p.thrust * k;
        s.motor[i].kQ = t.prop.kQ * p.torque * k;
        s.motor[i].R = t.prop.resistance * p.resistance * (1.0 + p.motorSpread * u(rng));
        s.motor[i].kv = t.prop.kvRad * (1.0 + 0.5 * p.motorSpread * u(rng));
        s.motor[i].J = t.prop.inertia * p.rotorInertia;
    }
    s.cgX = p.cgX * t.armM;
    s.cgY = p.cgY * t.armM;
    s.cells = battery.cells;
    // банка LiPo: ~9 мОм на 1 А·ч ёмкости (1300 мА·ч — ~7 мОм) + провода и разъём
    const double ah = std::fmax(0.1, battery.capacityMah / 1000.0);
    s.packR = s.cells * (0.009 / ah) + 0.004;
    s.escDelayS = t.escDelayS + p.escDelayS;
    s.blades = t.prop.prop.blades;
    s.vibDpsAtFull *= p.vibration;
    s.vibGAtFull *= p.vibration;
    const double d = t.prop.prop.diameterIn * 0.0254;
    s.dragQuad = 0.5 * 1.225 * 1.0 * (0.25 * d * d + 0.004 * t.massKg);
    return s;
}

class QuadSim
{
public:
    struct Vec
    {
        double x = 0, y = 0, z = 0;
    };

    explicit QuadSim(const QuadSimParams& p, double controlHz = 1000.0, int substeps = 8)
        : par(p), rng(p.seed), gauss(0.0, 1.0)
    {
        dtCtl = 1.0 / controlHz;
        sub = substeps;
        dt = dtCtl / sub;
        delaySteps = static_cast<int>(std::lround(par.escDelayS / dtCtl)) + 1;
        if (delaySteps >= MAX_DELAY) delaySteps = MAX_DELAY - 1;
        std::uniform_real_distribution<double> u(-1.0, 1.0);
        for (int i = 0; i < copter::MAX_MOTORS; ++i)
        {
            for (int h = 0; h < 3; ++h)
            {
                vibDir[i][h] = { u(rng), u(rng), 0.6 * u(rng) };
                vibPhase[i][h] = 3.14159 * u(rng);
            }
        }
        reset();
    }

    void reset()
    {
        pos = vel = omega = Vec();
        qw = 1;
        qx = qy = qz = 0;
        for (auto& w : rotor) w = 0;
        for (auto& c : queue)
        {
            for (auto& m : c) m = 0;
        }
        head = 0;
        onGround = true;
        aaf1 = aaf2 = aafA1 = aafA2 = Vec();
        specific = { 0, 0, -9.80665 };
        baroHeld = 0;
        baroTimer = 0;
        time = 0;
        volts = par.cells * par.cellOcv;
    }

    // Начать в воздухе: высота, моторы на тяге висения.
    void startHover(double height)
    {
        reset();
        pos.z = -height;
        onGround = false;
        const double perMotor = par.massKg * 9.80665 / par.frame.count;
        for (uint8_t i = 0; i < par.frame.count; ++i) rotor[i] = std::sqrt(perMotor / par.motor[i].kT);
    }

    // Ориентация по углам Эйлера (°), авиационные знаки.
    void setAttitude(double rollDeg, double pitchDeg, double yawDeg)
    {
        const double r = rollDeg * M_PI / 360.0, p = pitchDeg * M_PI / 360.0, y = yawDeg * M_PI / 360.0;
        const double cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p), cy = std::cos(y), sy = std::sin(y);
        qw = cr * cp * cy + sr * sp * sy;
        qx = sr * cp * cy - cr * sp * sy;
        qy = cr * sp * cy + sr * cp * sy;
        qz = cr * cp * sy - sr * sp * cy;
    }

    // Мотор отказал (сгорел, слетел винт): ни тяги, ни момента.
    void failMotor(int index)
    {
        if (index >= 0 && index < par.frame.count) failed[index] = true;
    }

    void setWind(double north, double east, double gustiness)
    {
        windBase = { north, east, 0 };
        gustLevel = gustiness;
    }

    // Один цикл управления: команды регуляторам (0..1; spinning=false — стоп).
    void step(const float* commands, bool spinning)
    {
        auto& slot = queue[head];
        for (uint8_t i = 0; i < par.frame.count; ++i) slot[i] = spinning ? commands[i] : 0.0;
        const auto& apply = queue[(head + MAX_DELAY - delaySteps) % MAX_DELAY];
        head = (head + 1) % MAX_DELAY;
        for (int s = 0; s < sub; ++s) physics(apply);
    }

    // ---------- датчики (как их видит прошивка) ----------

    void readImu(float gyro[3], float accel[3])
    {
        gyro[0] = static_cast<float>(aaf2.x * 57.29578 + par.gyroNoiseDps * gauss(rng));
        gyro[1] = static_cast<float>(aaf2.y * 57.29578 + par.gyroNoiseDps * gauss(rng));
        gyro[2] = static_cast<float>(aaf2.z * 57.29578 + par.gyroNoiseDps * gauss(rng));
        accel[0] = static_cast<float>(aafA2.x / 9.80665 + par.accelNoiseG * gauss(rng));
        accel[1] = static_cast<float>(-aafA2.y / 9.80665 + par.accelNoiseG * gauss(rng));
        accel[2] = static_cast<float>(-aafA2.z / 9.80665 + par.accelNoiseG * gauss(rng));
    }

    float readBaro()
    {
        return static_cast<float>(baroHeld);
    }

    float readVolts() const { return static_cast<float>(volts); }

    // ---------- истинное состояние ----------

    double height() const { return -pos.z; }
    double climbRate() const { return -vel.z; }
    Vec position() const { return pos; }
    Vec velocity() const { return vel; }
    Vec rates() const { return omega; }   // рад/с, FRD
    double rollDeg() const { return std::atan2(2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)) * 57.29578; }
    double pitchDeg() const { return std::asin(std::fmax(-1.0, std::fmin(1.0, 2 * (qw * qy - qx * qz)))) * 57.29578; }
    double yawDeg() const { return std::atan2(2 * (qx * qy + qw * qz), 1 - 2 * (qy * qy + qz * qz)) * 57.29578; }
    double tiltDeg() const { return std::acos(std::fmax(-1.0, std::fmin(1.0, 1 - 2 * (qx * qx + qy * qy)))) * 57.29578; }
    double rotorHz(int i) const { return rotor[i] / (2 * M_PI); }
    double batteryVolts() const { return volts; }
    double totalCurrent() const { return current; }
    bool grounded() const { return onGround; }
    double now() const { return time; }
    const QuadSimParams& params() const { return par; }

private:
    static constexpr int MAX_DELAY = 8;

    Vec toEarth(const Vec& v) const
    {
        return { (1 - 2 * (qy * qy + qz * qz)) * v.x + 2 * (qx * qy - qw * qz) * v.y + 2 * (qx * qz + qw * qy) * v.z,
                 2 * (qx * qy + qw * qz) * v.x + (1 - 2 * (qx * qx + qz * qz)) * v.y + 2 * (qy * qz - qw * qx) * v.z,
                 2 * (qx * qz - qw * qy) * v.x + 2 * (qy * qz + qw * qx) * v.y + (1 - 2 * (qx * qx + qy * qy)) * v.z };
    }

    Vec toBody(const Vec& v) const
    {
        return { (1 - 2 * (qy * qy + qz * qz)) * v.x + 2 * (qx * qy + qw * qz) * v.y + 2 * (qx * qz - qw * qy) * v.z,
                 2 * (qx * qy - qw * qz) * v.x + (1 - 2 * (qx * qx + qz * qz)) * v.y + 2 * (qy * qz + qw * qx) * v.z,
                 2 * (qx * qz + qw * qy) * v.x + 2 * (qy * qz - qw * qx) * v.y + (1 - 2 * (qx * qx + qy * qy)) * v.z };
    }

    void physics(const double* u)
    {
        const int n = par.frame.count;
        const double g = 9.80665;

        // Напряжение под нагрузкой — из той же формулы тока, что у моторов:
        // V = Uхх − Rбат·Σ dᵢ·(dᵢ·V − ωᵢ/Kv)/Rᵢ — линейно по V, решается сразу
        // (иначе ток и напряжение раскачивают друг друга с шагом счёта).
        {
            const double ocv = par.cells * par.cellOcv;
            double num = ocv, den = 1.0;
            for (int i = 0; i < n; ++i)
            {
                const auto& m = par.motor[i];
                const double duty = std::fmin(1.0, std::fmax(0.0, u[i]));
                if (duty <= 0.0 || duty * volts < rotor[i] / m.kv) continue;   // мотор не потребляет
                num += par.packR * duty * rotor[i] / (m.kv * m.R);
                den += par.packR * duty * duty / m.R;
            }
            volts = std::fmax(0.5 * ocv, num / den);
        }

        double totalI = 0, fz = 0, tx = 0, ty = 0, tz = 0;
        for (int i = 0; i < n; ++i)
        {
            const auto& m = par.motor[i];
            const double duty = failed[i] ? 0.0 : std::fmin(1.0, std::fmax(0.0, u[i]));
            const double I = (duty * volts - rotor[i] / m.kv) / m.R;
            const double load = m.kQ * rotor[i] * rotor[i];
            double wdot = (I / m.kv - load) / m.J;
            if (duty <= 0.0 && wdot < 0) wdot = -load / m.J - rotor[i] * 2.0;   // стоп: выбег
            rotor[i] = std::fmax(0.0, rotor[i] + wdot * dt);
            if (duty > 0.0) totalI += std::fmax(0.0, I) * duty;

            const double T = failed[i] ? 0.0 : m.kT * rotor[i] * rotor[i];
            const double rx = par.frame.place[i].x * par.armM - par.cgX;
            const double ry = par.frame.place[i].y * par.armM - par.cgY;
            fz -= T;
            tx += -ry * T;
            ty += rx * T;
            const double spin = static_cast<double>(static_cast<int8_t>(par.frame.place[i].spin));
            tz += spin * (load + m.J * wdot);
        }
        current = totalI;

        // ветер с порывами (случайное блуждание, возвращается к среднему)
        gust.x += (-gust.x * 0.5 * dt) + gustLevel * std::sqrt(dt) * gauss(rng);
        gust.y += (-gust.y * 0.5 * dt) + gustLevel * std::sqrt(dt) * gauss(rng);
        const Vec air = { vel.x - windBase.x - gust.x, vel.y - windBase.y - gust.y, vel.z };
        const Vec airB = toBody(air);
        const double sp = std::sqrt(airB.x * airB.x + airB.y * airB.y + airB.z * airB.z);
        Vec fB = { -par.dragLin * par.massKg * airB.x - par.dragQuad * sp * airB.x,
                   -par.dragLin * par.massKg * airB.y - par.dragQuad * sp * airB.y,
                   fz - 0.3 * par.dragLin * par.massKg * airB.z - par.dragQuad * sp * airB.z };

        // вращение: J·ω̇ = τ − ω × Jω − демпфирование
        const double damp = 0.02 * par.massKg * par.armM * par.armM;
        tx -= damp * omega.x;
        ty -= damp * omega.y;
        tz -= damp * omega.z;
        const Vec jw = { par.jx * omega.x, par.jy * omega.y, par.jz * omega.z };
        const Vec gyroT = { omega.y * jw.z - omega.z * jw.y, omega.z * jw.x - omega.x * jw.z, omega.x * jw.y - omega.y * jw.x };
        omega.x += (tx - gyroT.x) / par.jx * dt;
        omega.y += (ty - gyroT.y) / par.jy * dt;
        omega.z += (tz - gyroT.z) / par.jz * dt;

        // поступательное движение
        const Vec fE = toEarth(fB);
        Vec acc = { fE.x / par.massKg, fE.y / par.massKg, fE.z / par.massKg + g };

        // земля
        if (pos.z >= 0.0 && (vel.z >= 0.0 || onGround) && acc.z >= 0.0)
        {
            onGround = true;
            pos.z = 0.0;
            vel = Vec();
            acc = Vec();
            const double tilt = tiltDeg();
            omega = Vec();
            if (tilt < 60.0)
            {
                // встаёт на ноги, курс сохраняется
                const double yaw = yawDeg();
                setAttitude(0, 0, yaw);
            }
        }
        else
        {
            onGround = false;
        }

        vel.x += acc.x * dt;
        vel.y += acc.y * dt;
        vel.z += acc.z * dt;
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;
        pos.z += vel.z * dt;
        if (pos.z > 0.0 && !onGround)
        {
            // удар о землю
            pos.z = 0.0;
            vel = Vec();
            onGround = true;
        }

        // ориентация
        const double hw = 0.5 * dt;
        const double nw = qw + hw * (-qx * omega.x - qy * omega.y - qz * omega.z);
        const double nx = qx + hw * (qw * omega.x + qy * omega.z - qz * omega.y);
        const double ny = qy + hw * (qw * omega.y - qx * omega.z + qz * omega.x);
        const double nz = qz + hw * (qw * omega.z + qx * omega.y - qy * omega.x);
        const double nn = std::sqrt(nw * nw + nx * nx + ny * ny + nz * nz);
        qw = nw / nn;
        qx = nx / nn;
        qy = ny / nn;
        qz = nz / nn;

        // удельная сила (то, что видит акселерометр), рама
        specific = toBody({ acc.x, acc.y, acc.z - g });

        // вибрации моторов
        Vec vibG, vibA;
        for (int i = 0; i < n; ++i)
        {
            const auto& m = par.motor[i];
            // амплитуда ~ квадрат оборотов (дисбаланс), 1 — на полных
            const double frac = rotor[i] / (m.kv * par.cells * par.cellOcv * 0.85);
            const double amp = frac * frac;
            const double mult[3] = { 1.0, 2.0, static_cast<double>(par.blades) };
            const double share[3] = { 1.0, 0.5, 0.35 };
            for (int h = 0; h < 3; ++h)
            {
                vibPhase[i][h] += rotor[i] * mult[h] * dt;
                const double s = std::sin(vibPhase[i][h]) * amp * share[h];
                vibG.x += s * vibDir[i][h].x;
                vibG.y += s * vibDir[i][h].y;
                vibG.z += s * vibDir[i][h].z;
                vibA.x += s * vibDir[i][h].y;
                vibA.y += s * vibDir[i][h].z;
                vibA.z += s * vibDir[i][h].x;
            }
        }
        const double vg = par.vibDpsAtFull / 57.29578 / std::sqrt(static_cast<double>(n));
        const double va = par.vibGAtFull * g / std::sqrt(static_cast<double>(n));
        const Vec gyroRaw = { omega.x + vg * vibG.x, omega.y + vg * vibG.y, omega.z + vg * vibG.z };
        const Vec accRaw = { specific.x + va * vibA.x, specific.y + va * vibA.y, specific.z + va * vibA.z };

        // фильтр чипа: два PT1
        const double k = dt / (dt + 1.0 / (2 * M_PI * par.imuAafHz * 1.553774));
        aaf1.x += k * (gyroRaw.x - aaf1.x);
        aaf1.y += k * (gyroRaw.y - aaf1.y);
        aaf1.z += k * (gyroRaw.z - aaf1.z);
        aaf2.x += k * (aaf1.x - aaf2.x);
        aaf2.y += k * (aaf1.y - aaf2.y);
        aaf2.z += k * (aaf1.z - aaf2.z);
        aafA1.x += k * (accRaw.x - aafA1.x);
        aafA1.y += k * (accRaw.y - aafA1.y);
        aafA1.z += k * (accRaw.z - aafA1.z);
        aafA2.x += k * (aafA1.x - aafA2.x);
        aafA2.y += k * (aafA1.y - aafA2.y);
        aafA2.z += k * (aafA1.z - aafA2.z);

        // барометр 50 Гц
        baroTimer += dt;
        if (baroTimer >= 0.02)
        {
            baroTimer -= 0.02;
            baroHeld = height() + par.baroNoiseM * gauss(rng);
        }
        time += dt;
    }

    QuadSimParams par;
    std::mt19937 rng;
    std::normal_distribution<double> gauss;
    double dtCtl = 0.001, dt = 0.000125;
    int sub = 8;
    int delaySteps = 1;
    double queue[MAX_DELAY][copter::MAX_MOTORS] = {};
    int head = 0;

    Vec pos, vel, omega;
    double qw = 1, qx = 0, qy = 0, qz = 0;
    double rotor[copter::MAX_MOTORS] = {};
    bool failed[copter::MAX_MOTORS] = {};
    bool onGround = true;
    Vec specific;
    Vec aaf1, aaf2, aafA1, aafA2;
    Vec vibDir[copter::MAX_MOTORS][3];
    double vibPhase[copter::MAX_MOTORS][3] = {};
    Vec windBase, gust;
    double gustLevel = 0;
    double baroHeld = 0, baroTimer = 0;
    double volts = 0, current = 0;
    double time = 0;
};
