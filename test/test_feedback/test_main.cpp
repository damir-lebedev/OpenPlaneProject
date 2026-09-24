// ============================================================
// Замкнутая симуляция контура обратной связи (autopilot/feedback/)
//
// Заготовка обратной связи в прошивку не подключена — эти тесты
// единственное место, где она работает. Модель самолёта ниже —
// грубая (оси независимы, подъёмная сила — через угол атаки от
// скорости), но в ней есть то, ради чего обратная связь затевалась:
// задержка серво, демпфирование, эффективность рулей ∝ V²,
// постоянный момент, сваливание, шасси.
//
// Запуск (прошивает плату тестовой прошивкой, потом залейте обычную):
//   pio test -e esp32-s3 -f test_feedback
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "autopilot/feedback/FeedbackModules.h"

using FeedbackConfig::AXIS_PITCH;
using FeedbackConfig::AXIS_ROLL;
using FeedbackConfig::AXIS_YAW;

namespace
{
    constexpr float SIM_DT = 0.002f;                  // 500 Гц, как цикл прошивки
    constexpr int SERVO_DELAY_STEPS = 10;             // 20 мс транспортной задержки
    constexpr float SERVO_TAU_S = 0.03f;              // + инерция серво
    constexpr float REFERENCE_SPEED = 14.0f;
    constexpr float THRUST_ACCEL_MS2 = 6.0f;          // при 100 % газа
    constexpr float DRAG_COEFF = 0.0184f;             // 60 % газа — 14 м/с в горизонте
    constexpr float GROUND_FRICTION_MS2 = 0.5f;
    constexpr float SIM_STALL_SPEED = 8.0f;
    constexpr float LEVEL_ALPHA_DEG = 4.0f;           // угол атаки в горизонте на 14 м/с
    constexpr float LIFTOFF_SPEED = 9.5f;

    struct PlantAxis
    {
        float bRef = 1;          // эффективность руля на 14 м/с, °/с² на мкс
        float damping = 0;       // 1/с на 14 м/с
        float bias = 0;          // постоянный момент, °/с²
        int8_t sign = 1;         // −1 — руль перепутан (реверс серво / IMU)
        float angle = 0, rate = 0;
        float actuator = 0;
        float delayLine[SERVO_DELAY_STEPS] = {};
        int delayIndex = 0;
    };

    // Детерминированный "случайный" генератор для порывов.
    uint32_t rngState = 12345;
    float randomUnit()   // −1..1
    {
        rngState = rngState * 1664525u + 1013904223u;
        return ((rngState >> 8) & 0xFFFF) / 32767.5f - 1.0f;
    }

    class SimPlane
    {
    public:

        PlantAxis axes[3];
        float speed = REFERENCE_SPEED;
        float altitude = 50;
        bool onGround = false;
        bool freezeSpeed = false;
        bool airspeedSensor = true;

        bool armed = true;
        bool linkLost = false;
        bool stabilization = true;
        float pilotThrottle = 60;
        float targetRoll = 0, targetPitch = 0;
        float stick[3] = {};
        float disturbance[3] = {};

        float command[3] = {};
        float appliedThrottle = 60;
        uint32_t timeUs = 1000000;

        SimPlane()
        {
            axes[AXIS_ROLL].bRef = 3.0f;  axes[AXIS_ROLL].damping = -6.0f;
            axes[AXIS_PITCH].bRef = 1.5f; axes[AXIS_PITCH].damping = -4.0f;
            axes[AXIS_YAW].bRef = 0.6f;   axes[AXIS_YAW].damping = -2.0f;
        }

        void putOnGround()
        {
            onGround = true;
            altitude = 0;
            speed = 0;
            for (PlantAxis& a : axes) { a.angle = 0; a.rate = 0; }
        }

        float flightPathDeg() const
        {
            const float v = max(speed, 1.0f);
            return axes[AXIS_PITCH].angle - LEVEL_ALPHA_DEG * (sq(REFERENCE_SPEED / v) - 1.0f);
        }

        FlightSnapshot snapshot() const
        {
            FlightSnapshot s;
            s.timeUs = timeUs;
            s.armed = armed;
            s.linkLost = linkLost;

            s.imuValid = true;
            s.rollDeg = FeedbackMath::wrap180(axes[AXIS_ROLL].angle);
            s.pitchDeg = FeedbackMath::wrap180(axes[AXIS_PITCH].angle);
            s.yawDeg = FeedbackMath::wrap180(axes[AXIS_YAW].angle);
            s.rollRateDps = axes[AXIS_ROLL].rate;
            s.pitchRateDps = axes[AXIS_PITCH].rate;
            s.yawRateDps = axes[AXIS_YAW].rate;

            const float pitchRad = axes[AXIS_PITCH].angle * DEG_TO_RAD;
            const float rollRad = axes[AXIS_ROLL].angle * DEG_TO_RAD;
            s.accelXg = speedDot / FeedbackConfig::GRAVITY + sinf(pitchRad);
            s.accelYg = 0;
            s.accelZg = cosf(pitchRad) * cosf(rollRad) + (touchdownSpike ? 0.9f : 0.0f);

            s.baroValid = true;
            s.altitudeM = altitude;
            s.climbRateMs = onGround ? 0 : speed * sinf(flightPathDeg() * DEG_TO_RAD);
            s.airspeedValid = airspeedSensor;
            s.airspeedMs = speed;

            s.stabilizationActive = stabilization;
            s.targetRollDeg = targetRoll;
            s.targetPitchDeg = targetPitch;

            s.stickRollUs = stick[AXIS_ROLL];
            s.stickPitchUs = stick[AXIS_PITCH];
            s.stickYawUs = stick[AXIS_YAW];
            s.commandRollUs = command[AXIS_ROLL];
            s.commandPitchUs = command[AXIS_PITCH];
            s.commandYawUs = command[AXIS_YAW];

            s.pilotThrottlePercent = pilotThrottle;
            s.throttlePercent = appliedThrottle;
            return s;
        }

        // Выход обратной связи → рули и газ, как это сделает
        // FlightController при подключении.
        void apply(const FeedbackOutput& out)
        {
            for (uint8_t axis = 0; axis < 3; ++axis)
            {
                command[axis] = stick[axis] + (out.axisEnabled[axis] ? out.deflectionUs[axis] : 0.0f);
            }
            float throttle = pilotThrottle;
            if (out.throttleOverridePercent >= 0) throttle = out.throttleOverridePercent;
            if (out.throttleFloorPercent >= 0) throttle = max(throttle, out.throttleFloorPercent);
            appliedThrottle = linkLost ? 0 : throttle;
        }

        void step()
        {
            const float speedScale = sq(speed / REFERENCE_SPEED);
            const bool stalled = !onGround && speed < SIM_STALL_SPEED;

            for (uint8_t axis = 0; axis < 3; ++axis)
            {
                PlantAxis& a = axes[axis];
                const float delayed = a.delayLine[a.delayIndex];
                a.delayLine[a.delayIndex] = command[axis];
                a.delayIndex = (a.delayIndex + 1) % SERVO_DELAY_STEPS;
                a.actuator += (delayed - a.actuator) * SIM_DT / SERVO_TAU_S;

                float effectiveness = a.bRef * speedScale * (stalled ? 0.4f : 1.0f);
                float bias = a.bias;
                if (axis == AXIS_YAW && onGround)
                {
                    // На земле курс держит колесо: действует уже на малой
                    // скорости, а снос (винт, ветер) растёт с разгоном.
                    const float rolling = min(1.0f, speed / 5.0f);
                    effectiveness = max(effectiveness, a.bRef * rolling);
                    bias *= rolling;
                }
                float accel = a.sign * effectiveness * a.actuator +
                              a.damping * max(speed / REFERENCE_SPEED, 0.3f) * a.rate +
                              bias + disturbance[axis];
                if (axis == AXIS_PITCH && stalled) accel -= 150.0f;   // нос валится

                a.rate += accel * SIM_DT;
                a.angle += a.rate * SIM_DT;
            }

            // Скорость: тяга − сопротивление − составляющая веса.
            const float gamma = flightPathDeg() * DEG_TO_RAD;
            speedDot = THRUST_ACCEL_MS2 * appliedThrottle / 100.0f - DRAG_COEFF * speed * speed;
            if (onGround) speedDot -= (speed > 0.1f) ? GROUND_FRICTION_MS2 : 0.0f;
            else speedDot -= FeedbackConfig::GRAVITY * sinf(gamma);
            if (freezeSpeed) speedDot = 0;
            speed = max(0.0f, speed + speedDot * SIM_DT);

            touchdownSpike = false;
            if (onGround)
            {
                // На колёсах: крен и тангаж держит шасси; отрыв — когда
                // хватает скорости и руль высоты тянет нос вверх.
                PlantAxis& pitch = axes[AXIS_PITCH];
                const bool liftoff = speed >= LIFTOFF_SPEED && pitch.rate > 0;
                if (!liftoff)
                {
                    pitch.angle = 0; pitch.rate = 0;
                    axes[AXIS_ROLL].angle = 0; axes[AXIS_ROLL].rate = 0;
                }
                else
                {
                    onGround = false;
                }
            }
            else
            {
                altitude += speed * sinf(gamma) * SIM_DT;
                if (altitude <= 0)
                {
                    altitude = 0;
                    onGround = true;
                    touchdownSpike = true;
                }
            }

            timeUs += (uint32_t)(SIM_DT * 1000000.0f);
        }

    private:

        float speedDot = 0;
        bool touchdownSpike = false;
    };

    // Прогнать seconds секунд; beforeStep(номер шага) меняет условия.
    template <typename F>
    void run(SimPlane& sim, FeedbackSupervisor& fb, float seconds, F beforeStep)
    {
        const int steps = (int)(seconds / SIM_DT);
        for (int i = 0; i < steps; ++i)
        {
            beforeStep(i);
            sim.apply(fb.update(sim.snapshot()));
            sim.step();
        }
    }

    void run(SimPlane& sim, FeedbackSupervisor& fb, float seconds)
    {
        run(sim, fb, seconds, [](int) {});
    }

    int stepsFor(float seconds) { return (int)(seconds / SIM_DT); }
}


// ------------------------------------------------------------
// Выравнивание и "доправь ещё"
// ------------------------------------------------------------

// Крен 30°, нос −15°, и постоянный момент по крену (кривое крыло):
// самолёт выравнивается, а интеграл убирает остаточную ошибку.
void test_levels_after_upset_and_trims_constant_moment()
{
    SimPlane sim;
    sim.axes[AXIS_ROLL].angle = 30;
    sim.axes[AXIS_PITCH].angle = -15;
    sim.axes[AXIS_ROLL].bias = 60;     // требует ~20 мкс элерона постоянно
    FeedbackSupervisor fb;

    float maxOvershoot = 0;
    run(sim, fb, 3.0f, [&](int) { maxOvershoot = max(maxOvershoot, -sim.axes[AXIS_ROLL].angle); });
    const float rollAt3s = sim.axes[AXIS_ROLL].angle;
    run(sim, fb, 5.0f);

    Serial.printf("  roll@3s=%.2f roll=%.2f pitch=%.2f overshoot=%.1f I=%.1f trim=%.0f\n",
                  rollAt3s, sim.axes[AXIS_ROLL].angle, sim.axes[AXIS_PITCH].angle, maxOvershoot,
                  fb.getController(AXIS_ROLL).getIntegral(), fb.getOutput().deflectionUs[AXIS_ROLL]);

    TEST_ASSERT_FLOAT_WITHIN(3.0f, 0.0f, rollAt3s);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, sim.axes[AXIS_ROLL].angle);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, sim.axes[AXIS_PITCH].angle);
    TEST_ASSERT_LESS_THAN_FLOAT(10.0f, maxOvershoot);
}


// ------------------------------------------------------------
// Эффективность рулей изучается и следует за скоростью
// ------------------------------------------------------------

void test_estimator_learns_effectiveness_and_speed_scaling()
{
    SimPlane sim;
    sim.freezeSpeed = true;
    sim.axes[AXIS_ROLL].bRef = 6.0f;   // вдвое эффективнее априорной оценки
    FeedbackSupervisor fb;

    // Раскачка: цель по крену ±15° каждую секунду.
    auto excite = [&](int i) { sim.targetRoll = ((i / stepsFor(1.0f)) % 2) ? 15.0f : -15.0f; };

    run(sim, fb, 12.0f, excite);
    const ControlEffectivenessEstimator& roll = fb.getEstimator(AXIS_ROLL);
    Serial.printf("  14 m/s: b=%.2f±%.2f (true 6.00) a=%.1f confident=%d\n",
                  roll.getEffectiveness(), roll.getEffectivenessSigma(), roll.getDamping(), roll.isConfident());
    TEST_ASSERT_TRUE(roll.isConfident());
    TEST_ASSERT_FLOAT_WITHIN(6.0f * 0.3f, 6.0f, roll.getEffectiveness());
    TEST_ASSERT_LESS_THAN_FLOAT(0.0f, roll.getDamping());

    // Разгон до 20 м/с: руль стал в (20/14)² ≈ 2 раза сильнее — оценка
    // пересчитывается по скорости сразу, без переобучения.
    sim.speed = 20.0f;
    run(sim, fb, 0.1f, excite);
    const float expected = 6.0f * sq(20.0f / REFERENCE_SPEED);
    Serial.printf("  20 m/s, instantly: b=%.2f (true %.2f)\n", roll.getEffectiveness(), expected);
    TEST_ASSERT_FLOAT_WITHIN(expected * 0.3f, expected, roll.getEffectiveness());

    run(sim, fb, 10.0f, excite);
    Serial.printf("  20 m/s, after 10 s: b=%.2f±%.2f\n", roll.getEffectiveness(), roll.getEffectivenessSigma());
    TEST_ASSERT_FLOAT_WITHIN(expected * 0.3f, expected, roll.getEffectiveness());
    TEST_ASSERT_EQUAL_INT(1, fb.getGuard(AXIS_ROLL).getSign());

    // Без датчика скорости масштаба нет — b учится напрямую и всё
    // равно приходит к правде (медленнее).
    sim.airspeedSensor = false;
    run(sim, fb, 20.0f, excite);
    Serial.printf("  20 m/s, no airspeed sensor, after 20 s: b=%.2f±%.2f\n",
                  roll.getEffectiveness(), roll.getEffectivenessSigma());
    TEST_ASSERT_FLOAT_WITHIN(expected * 0.35f, expected, roll.getEffectiveness());
}


// ------------------------------------------------------------
// Автоинверсия
// ------------------------------------------------------------

// Элероны перепутаны, стабилизацию включили в полёте, самолёт
// накренило: автопилот сначала валит его дальше, замечает
// расходимость, переворачивает знак и выравнивает.
void test_auto_inversion_recovers_reversed_aileron()
{
    SimPlane sim;
    sim.axes[AXIS_ROLL].sign = -1;
    sim.stabilization = false;
    FeedbackSupervisor fb;

    run(sim, fb, 1.0f);                // в воздухе, пилот летит сам
    sim.axes[AXIS_ROLL].angle = 15;    // порыв
    sim.stabilization = true;

    float maxBank = 0;
    run(sim, fb, 8.0f, [&](int) { maxBank = max(maxBank, fabsf(sim.axes[AXIS_ROLL].angle)); });

    const ControlDirectionGuard& guard = fb.getGuard(AXIS_ROLL);
    Serial.printf("  guard=%s sign=%d event='%s' maxBank=%.0f roll=%.2f\n",
                  guard.getStateName(), guard.getSign(), guard.getLastEvent(), maxBank,
                  sim.axes[AXIS_ROLL].angle);

    TEST_ASSERT_EQUAL_INT(-1, guard.getSign());
    TEST_ASSERT_TRUE(guard.getState() == ControlDirectionGuard::State::Locked);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, FeedbackMath::wrap180(sim.axes[AXIS_ROLL].angle));
    TEST_ASSERT_LESS_THAN_FLOAT(120.0f, maxBank);
}

// Тот же перепутанный элерон, но пилот летит в MANUAL и качает
// крыльями: оценка b находит обратный знак ещё до включения
// стабилизации — включение проходит без броска.
void test_estimator_finds_reversal_in_manual_flight()
{
    SimPlane sim;
    sim.axes[AXIS_ROLL].sign = -1;
    sim.stabilization = false;
    FeedbackSupervisor fb;

    run(sim, fb, 8.0f, [&](int i) {
        sim.stick[AXIS_ROLL] = 120.0f * sinf(2.0f * PI * 0.7f * i * SIM_DT);
    });
    sim.stick[AXIS_ROLL] = 0;

    const ControlDirectionGuard& guard = fb.getGuard(AXIS_ROLL);
    Serial.printf("  after manual: b=%.2f guard=%s sign=%d event='%s'\n",
                  fb.getEstimator(AXIS_ROLL).getEffectiveness(), guard.getStateName(),
                  guard.getSign(), guard.getLastEvent());
    TEST_ASSERT_EQUAL_INT(-1, guard.getSign());

    // Включаем стабилизацию в крене 20° — сразу в правильную сторону.
    sim.axes[AXIS_ROLL].angle = 20;
    sim.axes[AXIS_ROLL].rate = 0;
    sim.stabilization = true;
    float maxBank = 0;
    run(sim, fb, 3.0f, [&](int) { maxBank = max(maxBank, fabsf(sim.axes[AXIS_ROLL].angle)); });
    Serial.printf("  stabilized: maxBank=%.1f roll=%.2f\n", maxBank, sim.axes[AXIS_ROLL].angle);
    TEST_ASSERT_LESS_THAN_FLOAT(22.0f, maxBank);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, sim.axes[AXIS_ROLL].angle);
}

// Правильные знаки и болтанка 30 с — ни одного ложного переворота.
void test_no_false_inversion_in_gusts()
{
    SimPlane sim;
    FeedbackSupervisor fb;
    rngState = 12345;

    int gustLeft = 0;
    float worstRoll = 0;
    run(sim, fb, 30.0f, [&](int) {
        if (gustLeft > 0)
        {
            if (--gustLeft == 0) { sim.disturbance[AXIS_ROLL] = 0; sim.disturbance[AXIS_PITCH] = 0; }
        }
        else if (randomUnit() > 0.995f)   // в среднем раз в ~1 с
        {
            gustLeft = stepsFor(0.2f);
            sim.disturbance[AXIS_ROLL] = 500.0f * randomUnit();
            sim.disturbance[AXIS_PITCH] = 300.0f * randomUnit();
        }
        worstRoll = max(worstRoll, fabsf(sim.axes[AXIS_ROLL].angle));
    });

    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        Serial.printf("  axis %d: guard=%s sign=%d event='%s'\n", axis, fb.getGuard(axis).getStateName(),
                      fb.getGuard(axis).getSign(), fb.getGuard(axis).getLastEvent());
        TEST_ASSERT_TRUE(fb.getGuard(axis).getState() == ControlDirectionGuard::State::Normal);
    }
    Serial.printf("  worst roll in gusts: %.1f\n", worstRoll);
}


// ------------------------------------------------------------
// Защита от сваливания
// ------------------------------------------------------------

// Пилот (режим) просит нос 15° вверх на малом газу. Выравнивание
// справилось бы с углом — но скорость уходит. Защита должна
// ограничивать тангаж и добавлять газ так, чтобы скорость ни разу не
// упала до сваливания. (Без защиты этот сценарий кончается
// сваливанием: 15° на 20 % газа самолёт не тянет.)
static void runLowEnergyScenario(bool airspeedSensor)
{
    SimPlane sim;
    sim.airspeedSensor = airspeedSensor;
    sim.pilotThrottle = 20;
    sim.targetPitch = 15;
    FeedbackSupervisor fb;

    float minSpeed = sim.speed;
    bool sawLowEnergy = false;
    bool sawStall = false;
    run(sim, fb, 20.0f, [&](int) {
        minSpeed = min(minSpeed, sim.speed);
        sawLowEnergy |= fb.getStallGuard().getLevel() == StallGuard::Level::LowEnergy;
        sawStall |= fb.getStallGuard().getLevel() == StallGuard::Level::Stall;
    });

    Serial.printf("  airspeed=%d: minSpeed=%.1f speed=%.1f pitch=%.1f lowEnergy=%d stall=%d level=%s reason='%s'\n",
                  airspeedSensor, minSpeed, sim.speed, sim.axes[AXIS_PITCH].angle, sawLowEnergy, sawStall,
                  fb.getStallGuard().getLevelName(), fb.getOutput().reason);

    TEST_ASSERT_TRUE(sawLowEnergy);
    TEST_ASSERT_FALSE(sawStall);
    TEST_ASSERT_GREATER_THAN_FLOAT(SIM_STALL_SPEED * 1.1f, minSpeed);
}

void test_stall_guard_with_airspeed() { runLowEnergyScenario(true); }
void test_stall_guard_without_airspeed() { runLowEnergyScenario(false); }


// ------------------------------------------------------------
// Взлёт и посадка
// ------------------------------------------------------------

void test_takeoff_from_runway()
{
    SimPlane sim;
    sim.putOnGround();
    sim.pilotThrottle = 0;
    sim.axes[AXIS_YAW].bias = 15;       // реактивный момент винта тянет с курса
    FeedbackSupervisor fb;

    run(sim, fb, 0.1f);
    TEST_ASSERT_TRUE(fb.requestTakeoff());

    float worstHeading = 0;
    bool sawGroundRoll = false, sawClimb = false;
    sim.pilotThrottle = 60;             // пилот двинул газ — старт
    run(sim, fb, 40.0f, [&](int) {
        const TakeoffSequencer::State st = fb.getTakeoff().getState();
        sawGroundRoll |= st == TakeoffSequencer::State::GroundRoll;
        sawClimb |= st == TakeoffSequencer::State::Climb;
        if (st == TakeoffSequencer::State::GroundRoll)
        {
            worstHeading = max(worstHeading, fabsf(FeedbackMath::wrap180(sim.axes[AXIS_YAW].angle)));
        }
    });

    Serial.printf("  takeoff=%s alt=%.1f speed=%.1f worstHeading=%.1f airborne=%d\n",
                  fb.getTakeoff().getStateName(), sim.altitude, sim.speed, worstHeading, fb.isAirborne());
    TEST_ASSERT_TRUE(sawGroundRoll);
    TEST_ASSERT_TRUE(sawClimb);
    TEST_ASSERT_TRUE(fb.getTakeoff().getState() == TakeoffSequencer::State::Complete);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(FeedbackConfig::TAKEOFF_TARGET_ALTITUDE_M, sim.altitude);
    TEST_ASSERT_LESS_THAN_FLOAT(5.0f, worstHeading);
}

void test_landing_sequence()
{
    SimPlane sim;
    sim.altitude = 15;
    sim.pilotThrottle = 50;
    FeedbackSupervisor fb;

    run(sim, fb, 1.0f);
    TEST_ASSERT_TRUE(fb.isAirborne());
    TEST_ASSERT_TRUE(fb.requestLanding());

    bool sawFlare = false, sawRollout = false, throttleInFlare = false;
    float touchdownSink = 0;
    run(sim, fb, 45.0f, [&](int) {
        const LandingSequencer::State st = fb.getLanding().getState();
        if (st == LandingSequencer::State::Flare)
        {
            sawFlare = true;
            throttleInFlare |= sim.appliedThrottle > 0;
        }
        if (!sim.onGround) touchdownSink = sim.snapshot().climbRateMs;
        if (st == LandingSequencer::State::Rollout)
        {
            sawRollout = true;
            sim.pilotThrottle = 0;       // пилот убрал газ после касания
        }
    });

    Serial.printf("  landing=%s sawFlare=%d sawRollout=%d sinkAtTouchdown=%.2f speed=%.1f airborne=%d\n",
                  fb.getLanding().getStateName(), sawFlare, sawRollout, touchdownSink, sim.speed,
                  fb.isAirborne());
    TEST_ASSERT_TRUE(sawFlare);
    TEST_ASSERT_TRUE(sawRollout);
    TEST_ASSERT_FALSE(throttleInFlare);   // защита от сваливания не дала газ у земли
    TEST_ASSERT_GREATER_THAN_FLOAT(-1.5f, touchdownSink);
    TEST_ASSERT_TRUE(fb.getLanding().getState() == LandingSequencer::State::Complete);
    TEST_ASSERT_FALSE(fb.isAirborne());
}


// ------------------------------------------------------------
// Безопасность
// ------------------------------------------------------------

void test_link_loss_cancels_takeoff_and_keeps_throttle()
{
    SimPlane sim;
    sim.putOnGround();
    FeedbackSupervisor fb;
    run(sim, fb, 0.1f);
    TEST_ASSERT_TRUE(fb.requestTakeoff());
    sim.pilotThrottle = 60;
    run(sim, fb, 1.0f);
    TEST_ASSERT_TRUE(fb.getTakeoff().getState() == TakeoffSequencer::State::GroundRoll);

    sim.linkLost = true;
    run(sim, fb, 0.1f);
    TEST_ASSERT_TRUE(fb.getTakeoff().getState() == TakeoffSequencer::State::Aborted);
    TEST_ASSERT_LESS_THAN_FLOAT(0.0f, fb.getOutput().throttleOverridePercent);
    TEST_ASSERT_LESS_THAN_FLOAT(0.0f, fb.getOutput().throttleFloorPercent);
    TEST_ASSERT_FALSE(fb.requestTakeoff());
}

void test_disarmed_and_manual_do_not_touch_controls()
{
    SimPlane sim;
    sim.axes[AXIS_ROLL].angle = 30;
    FeedbackSupervisor fb;

    sim.armed = false;
    run(sim, fb, 1.0f);
    for (uint8_t axis = 0; axis < 3; ++axis) TEST_ASSERT_FALSE(fb.getOutput().axisEnabled[axis]);

    sim.armed = true;
    sim.stabilization = false;
    run(sim, fb, 1.0f);
    for (uint8_t axis = 0; axis < 3; ++axis) TEST_ASSERT_FALSE(fb.getOutput().axisEnabled[axis]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fb.getController(AXIS_ROLL).getIntegral());
}


void setUp() {}
void tearDown() {}

void setup()
{
    delay(2000);   // дать монитору порта подключиться

    UNITY_BEGIN();
    RUN_TEST(test_levels_after_upset_and_trims_constant_moment);
    RUN_TEST(test_estimator_learns_effectiveness_and_speed_scaling);
    RUN_TEST(test_auto_inversion_recovers_reversed_aileron);
    RUN_TEST(test_estimator_finds_reversal_in_manual_flight);
    RUN_TEST(test_no_false_inversion_in_gusts);
    RUN_TEST(test_stall_guard_with_airspeed);
    RUN_TEST(test_stall_guard_without_airspeed);
    RUN_TEST(test_takeoff_from_runway);
    RUN_TEST(test_landing_sequence);
    RUN_TEST(test_link_loss_cancels_takeoff_and_keeps_throttle);
    RUN_TEST(test_disarmed_and_manual_do_not_touch_controls);
    UNITY_END();
}

void loop() {}
