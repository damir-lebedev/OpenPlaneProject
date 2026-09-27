#pragma once

// ============================================================
// Стенд замкнутого полёта: вся прошивка управления в цикле с
// моделью самолёта (PlaneSim.h).
//
//   пульт (RcChannels) -> кадр iBUS -> IBusReceiver -> PilotSwitches
//   -> Autopilot -> FlightController -> ШИМ серво (FakeBoard)
//   -> отклонения рулей -> PlaneSim -> датчики (IMU, барометр, компас,
//   GPS 10 Гц, воздушная скорость) -> следующий такт
//
// Такт — 2 мс, как у настоящего полётного цикла (Config::LOOP_PERIOD_MS),
// физика — 4 подшага по 0.5 мс. Воздушная скорость — либо "идеальная"
// (FakeAirspeed), либо настоящая трубка Пито на двух шумных барометрах
// (PitotDualBaroAirspeed) — useRealPitot.
//
// Если задана переменная окружения OPENPLANE_SIM_DIR, траектория
// пишется в <dir>/<name>.csv — из этих файлов рисуются графики
// (tools/plot_sim.py, картинки в docs/images/sim).
// ============================================================

#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>

#include "autopilot/Autopilot.h"
#include "autopilot/Navigation.h"
#include "autopilot/PilotSwitches.h"
#include "control/ArmingManager.h"
#include "control/ControlMixer.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"
#include "control/ThrottleManager.h"
#include "rc/IBusReceiver.h"
#include "sensors/airspeed/PitotDualBaroAirspeed.h"
#include "helpers/PlaneSim.h"
#include "helpers/TestSupport.h"

template <size_t N>
class SimHarness
{
public:
    FakeBoard board;
    FakeImu imu;
    FakeBaro baro;
    FakeMag mag;
    FakeGps gps;
    FakeAirspeed idealAirspeed;
    FakeBaro pitotTotal;
    FakeBaro pitotStatic;
    PitotDualBaroAirspeed pitot{ pitotTotal, pitotStatic };

    IBusReceiver receiver{ board.rcUart() };
    ControlMixer mixer;
    ThrottleManager throttle;
    FlightOutputs outputs{ board };
    Autopilot autopilot;
    PilotSwitches switches;
    ArmingManager arming{ &autopilot };
    FlightController controller{ receiver, mixer, throttle, arming, outputs, &autopilot, &switches };

    PlaneSim plane;
    RcChannels rc;
    GeoPoint origin;
    bool linkUp = true;
    bool gpsWorking = true;
    double time = 0;
    double throwAccelG = 0;   // бросок с руки: перегрузка вперёд, пока > 0
    double throwUntil = 0;
    bool frozen = false;      // самолёт в руке: физика стоит

    SimHarness(const Binding (&table)[N], bool withAirspeed, bool useRealPitot = false, const char* traceName = nullptr)
        : autopilot(&imu, &baro, &mag, &gps,
                    withAirspeed ? (useRealPitot ? static_cast<AirspeedSensor*>(&pitot) : &idealAirspeed) : nullptr),
          switches(&autopilot, table),
          realPitot(withAirspeed && useRealPitot)
    {
        origin.lat = 55.75;
        origin.lon = 37.61;
        outputs.begin();
        controller.begin();
        autopilot.begin();
        if (realPitot)
        {
            pitotTotal.data.pressure = 101325;
            pitotStatic.data.pressure = 101325;
            pitot.begin();
        }
        openTrace(traceName);
    }

    ~SimHarness()
    {
        if (trace) fclose(trace);
    }

    SimHarness(const SimHarness&) = delete;
    SimHarness& operator=(const SimHarness&) = delete;

    // Заармить в текущем положении тумблеров (газ внизу).
    void arm()
    {
        const uint16_t throttleWas = rc.value[Channels::THROTTLE];
        rc.set(Channels::THROTTLE, 1000).set(Channels::ARM, 1000);
        run(0.1);
        rc.set(Channels::ARM, 2000);
        run(0.1);
        rc.set(Channels::THROTTLE, throttleWas);
    }

    void run(double seconds)
    {
        const int ticks = static_cast<int>(seconds / 0.002 + 0.5);
        for (int i = 0; i < ticks; ++i) tick();
    }

    // Лететь, пока pred() не станет true, но не дольше maxSeconds.
    template <typename Pred>
    bool runUntil(Pred pred, double maxSeconds)
    {
        const int ticks = static_cast<int>(maxSeconds / 0.002);
        for (int i = 0; i < ticks; ++i)
        {
            tick();
            if (pred()) return true;
        }
        return false;
    }

    void tick()
    {
        if (linkUp) board.rc.push(ibusFrame(rc));
        feedSensors();
        fake::advanceMs(2);
        controller.update();

        const PlaneControls c = controls();
        if (!frozen)
        {
            for (int i = 0; i < 4; ++i) plane.step(c, 0.0005);
        }
        time += 0.002;
        lastControls = c;

        if (trace && ++traceDivider % 25 == 0) writeTrace();   // 20 Гц
        if (++logDivider % 500 == 0) takeSerial();              // лог не копится
    }

    // Отклонения рулей из ШИМ — обратная функция ControlMixer::mix().
    PlaneControls controls() const
    {
        const FlightOutputState& o = outputs.getLastState();
        auto deflection = [](uint16_t pwm, bool reversed) {
            const double d = static_cast<double>(pwm) - 1500.0;
            return reversed ? -d : d;
        };
        const double left = deflection(o.aileronLeft, Config::AILERON_LEFT_REVERSED);
        const double right = deflection(o.aileronRight, Config::AILERON_RIGHT_REVERSED);

        PlaneControls c;
        c.aileron = (left - right) / 2.0 / 500.0;
        c.flaps = (left + right) / 2.0 / Config::FLAPS_DEPLOYED_US;
        c.elevator = deflection(o.elevator, Config::ELEVATOR_REVERSED) / 500.0;
        c.rudder = deflection(o.rudder, Config::RUDDER_REVERSED) / 500.0;
        c.throttle = (static_cast<double>(o.throttle) - 1000.0) / 1000.0;
        return c;
    }

    GeoPoint position() const { return Geo::moved(origin, static_cast<float>(plane.s.north), static_cast<float>(plane.s.east)); }
    double distanceFrom(double north, double east) const { return hypot(plane.s.north - north, plane.s.east - east); }
    double distanceHome() const { return distanceFrom(0, 0); }
    double rollDeg() const { return plane.s.roll * 57.29578; }
    double pitchDeg() const { return plane.s.pitch * 57.29578; }
    double headingDeg() const { return plane.s.heading * 57.29578; }
    const PlaneControls& lastControl() const { return lastControls; }

    // Пилот: крен/тангаж стиками, мкс отклонения (как ControlCommand).
    void sticks(int roll, int pitch)
    {
        rc.set(Channels::AILERON, static_cast<uint16_t>(1500 + roll));
        rc.set(Channels::ELEVATOR, static_cast<uint16_t>(1500 - pitch));   // на себя = нос вверх
    }

    void throw_(double accelG, double seconds)
    {
        throwAccelG = accelG;
        throwUntil = time + seconds;
    }


private:

    bool realPitot;
    FILE* trace = nullptr;
    unsigned traceDivider = 0;
    unsigned logDivider = 0;
    double lastGpsTime = -1;
    PlaneControls lastControls;
    std::mt19937 rng{ 7 };
    std::normal_distribution<float> baroNoise{ 0.0f, 0.05f };
    std::normal_distribution<float> tubeNoise{ 0.0f, 0.3f };
    std::normal_distribution<float> staticNoise{ 0.0f, 1.0f };
    double nextTube = 0, nextStatic = 0;

    void feedSensors()
    {
        const PlaneState& s = plane.s;
        const uint32_t now = micros();

        imu.data.roll = static_cast<float>(s.roll * 57.29578);
        imu.data.pitch = static_cast<float>(s.pitch * 57.29578);
        imu.data.yaw = Geo::wrap180(static_cast<float>(s.heading * 57.29578));
        imu.data.gyroX = static_cast<float>(s.rollRate * 57.29578);
        imu.data.gyroY = static_cast<float>(s.pitchRate * 57.29578);
        imu.data.gyroZ = static_cast<float>(s.yawRate * 57.29578);
        imu.data.accelX = static_cast<float>(time < throwUntil ? throwAccelG : s.forwardAccelG);
        imu.data.accelZ = 1.0f;
        imu.data.timestamp = now;

        baro.data.altitude = static_cast<float>(s.height) + baroNoise(rng);
        baro.data.verticalSpeed = static_cast<float>(s.climbRate);
        baro.data.timestamp = now;

        mag.data.headingDegrees = Geo::wrap360(static_cast<float>(s.heading * 57.29578));
        mag.data.timestamp = now;

        gps.available = gpsWorking;
        if (gpsWorking && time - lastGpsTime >= 0.1)
        {
            lastGpsTime = time;
            const GeoPoint p = position();
            gps.data.latitude = p.lat;
            gps.data.longitude = p.lon;
            gps.data.altitude = static_cast<float>(s.height);
            gps.data.groundSpeed = static_cast<float>(plane.groundSpeed());
            gps.data.heading = static_cast<float>(plane.groundCourse() * 57.29578);
            gps.data.fixType = 3;
            gps.data.numSatellites = 12;
            gps.data.horizontalAccuracy = 1.5f;
            gps.data.timestamp = now;
        }

        idealAirspeed.setSpeed(static_cast<float>(s.speed));

        if (realPitot)
        {
            // Статика ~32 Гц, трубка 50 Гц, у барометров свои шумы и
            // смещение 150 Па между чипами.
            const double staticPa = 101325.0 * pow(1.0 - s.height / 44330.0, 5.255);
            const double rho = staticPa / (287.05 * 288.15);
            if (time >= nextStatic)
            {
                nextStatic = time + 0.031;
                pitotStatic.data.pressure = static_cast<float>(staticPa) + staticNoise(rng);
                pitotStatic.data.temperature = 15.0f;
                pitotStatic.data.timestamp = now;
            }
            if (time >= nextTube)
            {
                nextTube = time + 0.02;
                pitotTotal.data.pressure = static_cast<float>(staticPa + 0.5 * rho * s.speed * s.speed + 150.0) + tubeNoise(rng);
                pitotTotal.data.temperature = 15.0f;
                pitotTotal.data.timestamp = now;
            }
        }
    }

    void openTrace(const char* name)
    {
        const char* dir = getenv("OPENPLANE_SIM_DIR");
        if (!dir || !name) return;
        const std::string path = std::string(dir) + "/" + name + ".csv";
        trace = fopen(path.c_str(), "w");
        if (trace)
        {
            fprintf(trace, "t,north,east,height,speed,roll,pitch,heading,throttle,mode,airspeed_est,target_alt,soaring,home_dist\n");
        }
    }

    void writeTrace()
    {
        const AirspeedSensor* a = autopilot.getAirspeedSensor();
        const float est = (a && a->isAvailable()) ? a->getAirspeedData().trueMs : -1.0f;
        fprintf(trace, "%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%s,%.2f,%.1f,%d,%.1f\n",
                time, plane.s.north, plane.s.east, plane.s.height, plane.s.speed, rollDeg(), pitchDeg(),
                headingDeg(), lastControls.throttle, autopilot.getModeName(), est, autopilot.getTargetAltitude(),
                static_cast<int>(autopilot.getSoaringState()), autopilot.getNavStatus().distanceHomeM);
    }
};
