#pragma once

// ============================================================
// Модель самолёта для замкнутых симуляций (только тесты).
//
// Точка с массой + вращение по крену и тангажу первым порядком — не
// полная 6-DOF модель, но всё, что важно автопилоту, в ней есть:
//   • подъёмная сила L = ½ρV²S·CL(α), CL = CL0 + CLα·α, сваливание
//     после ALPHA_STALL (подъёмная сила падает);
//   • сопротивление D = ½ρV²S·(CD0 + k·CL²) (+ закрылки, тормоз);
//   • тяга мотора падает со скоростью, до нуля на V_MAX;
//   • крен вызывает разворот (ψ̇ = L·sinφ / (m·V)), энергия
//     перетекает между скоростью и высотой;
//   • статическая устойчивость по тангажу (самолёт сам держит угол
//     атаки балансировки), демпфирование крена/тангажа, эффективность
//     рулей растёт как V²;
//   • ветер, термики (восходящие потоки), земля с трением качения;
//   • шасси (gear): LEGACY — прежняя земля (нос не ниже горизонта, без
//     рысканья); TAIL_WHEEL — самолёт качается на основных колёсах между
//     "хвост на земле" (parkPitch) и "винт у земли" (propStrikePitch),
//     ЦТ за колёсами тянет хвост вниз, хвост поднимает руль высоты в
//     потоке от винта; считаются нагрузка на хвостовое колесо, удары
//     хвостом и касания винтом; NOSE_WHEEL — нос лежит на колесе, хвост
//     чиркает на tailStrikePitch. На земле — рысканье: руль направления,
//     момент винта (нос влево), рулевое колесо, пока касается.
//
// Параметры — модель ~1.2 кг, размах ~1.2 м, крейсер ~15 м/с.
// Рули: -1..+1 (доля полного хода), газ 0..1.
// ============================================================

#include <algorithm>
#include <cmath>
#include <functional>

enum class SimGear { LEGACY, TAIL_WHEEL, NOSE_WHEEL };

struct PlaneState
{
    double north = 0, east = 0;  // м от старта
    double height = 0;           // м над землёй
    double speed = 0;            // воздушная, м/с
    double heading = 0;          // рад, по часовой от севера
    double gamma = 0;            // угол наклона траектории, рад
    double roll = 0;             // рад, + правое крыло вниз
    double pitch = 0;            // рад, + нос вверх
    double rollRate = 0;         // рад/с
    double pitchRate = 0;        // рад/с
    double yawRate = 0;          // рад/с
    double climbRate = 0;        // м/с (относительно земли)
    double forwardAccelG = 0;    // продольная перегрузка
    bool onGround = true;
};

struct PlaneControls
{
    double aileron = 0;   // + крен вправо
    double elevator = 0;  // + нос вверх
    double rudder = 0;    // + нос вправо
    double flaps = 0;     // + вниз (закрылки), − вверх (тормоз); доля от FLAPS_DEPLOYED
    double throttle = 0;  // 0..1
};

class PlaneSim
{
public:
    static constexpr double G = 9.80665;
    static constexpr double RHO = 1.225;
    static constexpr double MASS = 1.2;
    static constexpr double AREA = 0.3;
    static constexpr double CL0 = 0.3;
    static constexpr double CL_ALPHA = 4.5;
    static constexpr double ALPHA_STALL = 0.22;
    static constexpr double CD0 = 0.04;
    static constexpr double K_INDUCED = 0.06;
    static constexpr double THRUST_MAX = 12.0;
    static constexpr double V_MAX = 30.0;
    static constexpr double V_REF = 15.0;

    // Вращение: разгон от руля, демпфирование, устойчивость по тангажу.
    static constexpr double ROLL_POWER = 12.0;
    static constexpr double ROLL_DAMPING = 6.0;
    static constexpr double PITCH_POWER = 25.0;
    static constexpr double PITCH_DAMPING = 8.0;
    static constexpr double PITCH_STABILITY = 40.0;
    static constexpr double ALPHA_TRIM = 0.0;
    static constexpr double RUDDER_YAW_RATE = 0.4;

    // Шасси (gear != LEGACY).
    static constexpr double DEG = 0.017453293;
    static constexpr double WEIGHT_MOMENT = 6.0;     // рад/с²: ЦТ за основными колёсами — хвост вниз
    static constexpr double PROPWASH = 0.5;          // хвост в потоке от винта на полном газу, доля от V_REF
    static constexpr double TAIL_ARM_INERTIA = 0.15; // I / плечо хвостового колеса, кг·м: нагрузка = это · ускорение
    static constexpr double GROUND_RUDDER_POWER = 4.0;
    static constexpr double P_FACTOR = -0.4;         // рад/с² на полном газу: нос влево
    static constexpr double GROUND_YAW_DAMPING = 3.0;
    static constexpr double WHEELBASE = 0.8;
    static constexpr double MAX_STEER = 0.35;        // рулевое колесо на полном руле, рад
    static constexpr double WHEEL_GRIP = 20.0;       // 1/с: как быстро колесо навязывает свой курс

    PlaneState s;
    SimGear gear = SimGear::LEGACY;
    double parkPitch = 10 * DEG;        // хвостовое: тангаж на трёх колёсах
    double propStrikePitch = -4 * DEG;  // хвостовое: ниже — винт в земле
    double tailStrikePitch = 12 * DEG;  // носовое: выше — хвост чиркнул

    double tailLoadN = 0;               // нагрузка на хвостовое колесо сейчас, Н
    double maxTailImpactDps = 0;        // хвост упал на полосу с такой скоростью вращения
    int propStrikes = 0;
    int tailStrikes = 0;

    // Нагрузка на хвостовое колесо стоящего самолёта.
    static constexpr double staticTailLoadN() { return TAIL_ARM_INERTIA * WEIGHT_MOMENT; }

    // Поставить на полосу (хвостовое — на три колеса).
    void placeOnGround()
    {
        s = PlaneState();
        s.onGround = true;
        s.pitch = gear == SimGear::TAIL_WHEEL ? parkPitch : 0.0;
    }
    double windNorth = 0, windEast = 0;
    double aileronBias = 0;  // перекос: самолёт "тянет" в сторону без стика
    // Восходящий поток (м/с) в точке — для парения.
    std::function<double(double north, double east, double height)> updraft;

    double alpha() const { return s.pitch - s.gamma; }

    double liftCoefficient(double flaps) const
    {
        const double a = alpha();
        double cl = CL0 + CL_ALPHA * a + 0.35 * flaps;
        if (a > ALPHA_STALL)
        {
            const double peak = CL0 + CL_ALPHA * ALPHA_STALL;
            cl = peak * std::max(0.3, 1.0 - 3.0 * (a - ALPHA_STALL)) + 0.35 * flaps;
        }
        return cl;
    }

    void step(const PlaneControls& c, double dt)
    {
        const double v = std::max(s.speed, 0.1);
        const double q = 0.5 * RHO * v * v;
        const double qRatio = (v / V_REF) * (v / V_REF);

        const double cl = liftCoefficient(c.flaps);
        const double lift = q * AREA * cl;
        const double brake = c.flaps < 0 ? -c.flaps * 0.08 : c.flaps * 0.03;
        const double drag = q * AREA * (CD0 + brake + K_INDUCED * cl * cl);
        const double thrust = THRUST_MAX * std::clamp(c.throttle, 0.0, 1.0) * std::max(0.0, 1.0 - v / V_MAX);

        // Вращение.
        const bool gearModel = s.onGround && gear != SimGear::LEGACY;
        const double aileron = std::clamp(c.aileron + aileronBias, -1.0, 1.0);
        s.rollRate += (ROLL_POWER * aileron * qRatio - ROLL_DAMPING * s.rollRate * (v / V_REF)) * dt;
        if (gearModel)
        {
            groundPitch(c, dt, lift, qRatio, v);
        }
        else
        {
            s.pitchRate += (PITCH_POWER * c.elevator * qRatio - PITCH_DAMPING * s.pitchRate * (v / V_REF) -
                            PITCH_STABILITY * (alpha() - ALPHA_TRIM) * qRatio) * dt;
        }

        if (s.onGround)
        {
            // На колёсах: крен ноль, нос не ниже горизонта, трение.
            s.roll = 0;
            s.rollRate = 0;
            if (gearModel)
            {
                groundYaw(c, dt, v, qRatio);
                s.heading = fmod(s.heading + s.yawRate * dt + 2 * M_PI, 2 * M_PI);
            }
            else
            {
                s.pitch = std::max(0.0, s.pitch + s.pitchRate * dt);
                if (s.pitch == 0.0) s.pitchRate = std::max(0.0, s.pitchRate);
                s.yawRate = 0;
            }
            s.gamma = 0;
            const double normal = std::max(0.0, MASS * G - lift);
            const double accel = (thrust - drag - 0.05 * normal) / MASS;
            s.speed = std::max(0.0, s.speed + accel * dt);
            s.forwardAccelG = accel / G;
            if (lift > MASS * G * 1.02 && s.speed > 5)
            {
                s.onGround = false;
                tailLoadN = 0;
            }
        }
        else
        {
            s.roll += s.rollRate * dt;
            s.pitch += s.pitchRate * dt;
            s.pitch = std::clamp(s.pitch, -1.5, 1.5);

            const double accel = (thrust - drag) / MASS - G * sin(s.gamma);
            s.speed = std::max(0.5, s.speed + accel * dt);
            s.forwardAccelG = (thrust - drag) / (MASS * G);

            s.gamma += (lift * cos(s.roll) - MASS * G * cos(s.gamma)) / (MASS * v) * dt;
            s.gamma = std::clamp(s.gamma, -1.4, 1.4);
            s.yawRate = lift * sin(s.roll) / (MASS * v * std::max(cos(s.gamma), 0.2)) + RUDDER_YAW_RATE * c.rudder;
            s.heading = fmod(s.heading + s.yawRate * dt + 2 * M_PI, 2 * M_PI);
        }

        const double up = updraft ? updraft(s.north, s.east, s.height) : 0.0;
        const double horizontal = s.speed * cos(s.gamma);
        s.north += (horizontal * cos(s.heading) + windNorth) * dt;
        s.east += (horizontal * sin(s.heading) + windEast) * dt;
        s.climbRate = s.onGround ? 0.0 : s.speed * sin(s.gamma) + up;
        s.height += s.climbRate * dt;

        if (s.height <= 0 && !s.onGround && s.climbRate < 0)
        {
            touchdownVerticalSpeed = s.climbRate;
            touchdownPitch = s.pitch;
            touchdownRoll = s.roll;
            touchdowns++;
            s.height = 0;
            s.onGround = true;
            s.gamma = 0;
        }
        if (s.onGround || s.height < 0) s.height = 0;
    }

    // Тангаж на колёсах: самолёт качается на основных колёсах.
    void groundPitch(const PlaneControls& c, double dt, double lift, double qRatio, double v)
    {
        const double throttle = std::clamp(c.throttle, 0.0, 1.0);
        const double qTail = qRatio + PROPWASH * throttle;
        const double unloaded = std::max(0.0, 1.0 - lift / (MASS * G));
        const double weight = (gear == SimGear::TAIL_WHEEL ? WEIGHT_MOMENT : -WEIGHT_MOMENT) * unloaded;
        double acc = PITCH_POWER * c.elevator * qTail - PITCH_DAMPING * s.pitchRate * std::max(v / V_REF, 0.3) -
                     PITCH_STABILITY * alpha() * qRatio + weight;

        const double upper = gear == SimGear::TAIL_WHEEL ? parkPitch : tailStrikePitch;
        const double lower = gear == SimGear::TAIL_WHEEL ? propStrikePitch : 0.0;
        const bool resting = s.pitch >= upper - 1e-9;

        tailLoadN = 0;
        if (resting && acc > 0)
        {
            // Хвост лежит на земле: она его и держит.
            if (gear == SimGear::TAIL_WHEEL) tailLoadN = TAIL_ARM_INERTIA * acc;
            s.pitchRate = 0;
            return;
        }

        s.pitchRate += acc * dt;
        double next = s.pitch + s.pitchRate * dt;
        if (next >= upper)
        {
            if (!resting && s.pitchRate > 0)
            {
                if (gear == SimGear::TAIL_WHEEL) maxTailImpactDps = std::max(maxTailImpactDps, s.pitchRate / DEG);
                else tailStrikes++;
            }
            next = upper;
            s.pitchRate = 0;
        }
        if (next <= lower)
        {
            if (gear == SimGear::TAIL_WHEEL && s.pitch > lower + 1e-9) propStrikes++;
            next = lower;
            s.pitchRate = std::max(0.0, s.pitchRate);
        }
        s.pitch = next;
    }

    // Рысканье на колёсах: руль направления в потоке, момент винта,
    // рулевое колесо, пока касается полосы.
    void groundYaw(const PlaneControls& c, double dt, double v, double qRatio)
    {
        const double throttle = std::clamp(c.throttle, 0.0, 1.0);
        const double qTail = qRatio + PROPWASH * throttle;
        s.yawRate += (GROUND_RUDDER_POWER * c.rudder * qTail + P_FACTOR * throttle - GROUND_YAW_DAMPING * s.yawRate) * dt;

        const bool wheelDown = gear == SimGear::NOSE_WHEEL ? s.pitch <= 0.5 * DEG : s.pitch >= parkPitch - 0.5 * DEG;
        if (wheelDown)
        {
            const double wheelRate = v * c.rudder * MAX_STEER / WHEELBASE;
            s.yawRate += (wheelRate - s.yawRate) * std::min(1.0, WHEEL_GRIP * dt);
        }
    }

    // Путевая скорость и курс относительно земли (что видит GPS).
    double groundSpeed() const
    {
        const double h = s.speed * cos(s.gamma);
        const double vn = h * cos(s.heading) + windNorth;
        const double ve = h * sin(s.heading) + windEast;
        return sqrt(vn * vn + ve * ve);
    }

    double groundCourse() const
    {
        const double h = s.speed * cos(s.gamma);
        const double vn = h * cos(s.heading) + windNorth;
        const double ve = h * sin(s.heading) + windEast;
        return fmod(atan2(ve, vn) + 2 * M_PI, 2 * M_PI);
    }

    // Лететь ровно на скорости speed на высоте height.
    void setCruise(double height, double speed, double headingRad)
    {
        s = PlaneState();
        s.onGround = false;
        s.height = height;
        s.speed = speed;
        s.heading = headingRad;
        // Балансировка: CL на горизонтальный полёт.
        const double clNeeded = MASS * G / (0.5 * RHO * speed * speed * AREA);
        s.pitch = (clNeeded - CL0) / CL_ALPHA;
    }

    double touchdownVerticalSpeed = 0;
    double touchdownPitch = 0;
    double touchdownRoll = 0;
    int touchdowns = 0;
};
