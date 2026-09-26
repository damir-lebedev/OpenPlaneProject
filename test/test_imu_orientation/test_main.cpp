// ============================================================
// Калибровка установки IMU (ImuOrientation) на синтетических позах
//
// Для случайных установок платы (включая "вверх ногами" и "на
// боку") считается, что акселерометр показал бы в трёх позах
// калибровки, — и проверяется, что найденная установка совпадает с
// истинной, а ошибки пилота (нос опустил вместо подъёма, наклонил не
// ту ось, наклонил слишком мало) отклоняются.
//
// Запуск: pio test -e native -f test_imu_orientation (на ПК)
//         pio test -e esp32-s3 -f test_imu_orientation (на плате)
// ============================================================

#include <Arduino.h>
#include <unity.h>

#include "sensors/imu/ImuOrientation.h"

namespace
{
    struct Matrix
    {
        float m[3][3];
    };

    // body = R · chip; строки R — оси самолёта в осях чипа.
    Matrix rotation(float axisX, float axisY, float axisZ, float angleDeg)
    {
        const float n = sqrtf(axisX * axisX + axisY * axisY + axisZ * axisZ);
        const float x = axisX / n, y = axisY / n, z = axisZ / n;
        const float a = angleDeg * DEG_TO_RAD, c = cosf(a), s = sinf(a), t = 1 - c;
        Matrix r = { { { t * x * x + c,     t * x * y - s * z, t * x * z + s * y },
                       { t * x * y + s * z, t * y * y + c,     t * y * z - s * x },
                       { t * x * z - s * y, t * y * z + s * x, t * z * z + c } } };
        return r;
    }

    // Что покажет акселерометр (оси чипа), если "верх" в осях самолёта — up.
    void chipReading(const Matrix& truth, const float up[3], const float bias[3], float out[3])
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            out[i] = truth.m[0][i] * up[0] + truth.m[1][i] * up[1] + truth.m[2][i] * up[2] + bias[i];
        }
    }

    // Три позы калибровки: ровно, нос +noseDeg, правое крыло вниз wingDeg.
    const char* calibrate(const Matrix& truth, float noseDeg, float wingDeg, const float bias[3],
                          ImuOrientation& result)
    {
        const float levelUp[3] = { 0, 0, 1 };
        const float noseUp[3] = { sinf(noseDeg * DEG_TO_RAD), 0, cosf(noseDeg * DEG_TO_RAD) };
        const float wingUp[3] = { 0, sinf(wingDeg * DEG_TO_RAD), cosf(wingDeg * DEG_TO_RAD) };

        float level[3], nose[3], wing[3];
        chipReading(truth, levelUp, bias, level);
        chipReading(truth, noseUp, bias, nose);
        chipReading(truth, wingUp, bias, wing);
        return ImuOrientation::fromPoses(level, nose, wing, result);
    }

    // Наибольший угол, °, между осью самолёта по истинной установке и
    // по найденной: берём каждую ось самолёта в осях чипа (строку
    // истины) и смотрим, куда её отправляет найденная установка.
    float worstAxisErrorDeg(const Matrix& truth, const ImuOrientation& found)
    {
        float worst = 0;
        for (uint8_t axis = 0; axis < 3; ++axis)
        {
            float mapped[3];
            found.apply(truth.m[axis], mapped);
            worst = max(worst, float(acosf(constrain(mapped[axis], -1.0f, 1.0f)) * RAD_TO_DEG));
        }
        return worst;
    }

    uint32_t rngState = 777;
    float randomUnit()   // −1..1
    {
        rngState = rngState * 1664525u + 1013904223u;
        return ((rngState >> 8) & 0xFFFF) / 32767.5f - 1.0f;
    }

    const float NO_BIAS[3] = { 0, 0, 0 };
}


void test_known_mountings()
{
    struct Case { const char* name; Matrix truth; };
    const Case cases[] = {
        { "как в Config (0°)",           rotation(0, 0, 1, 0) },
        { "стенд: поворот 90°",           rotation(0, 0, 1, -90) },
        { "вверх ногами",                 rotation(1, 0, 0, 180) },
        { "на боку",                      rotation(1, 0, 0, 90) },
        { "стоймя, чипом к носу",         rotation(0, 1, 0, 90) },
        { "криво: 17° и 33° и 120°",      rotation(0.3f, -0.8f, 0.5f, 123) },
    };

    for (const Case& c : cases)
    {
        ImuOrientation found;
        const char* error = calibrate(c.truth, 40, 35, NO_BIAS, found);
        const float axisError = error ? 999 : worstAxisErrorDeg(c.truth, found);
        Serial.printf("  %-28s error=%s axisError=%.3f°\n", c.name, error ? error : "-", axisError);
        TEST_ASSERT_NULL_MESSAGE(error, c.name);
        TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(0.5f, axisError, c.name);
    }
}

void test_random_mountings()
{
    rngState = 777;
    float worst = 0;
    for (int i = 0; i < 300; ++i)
    {
        const Matrix truth = rotation(randomUnit(), randomUnit(), randomUnit() + 0.01f, 180 * randomUnit());
        const float noseDeg = 30 + 30 * (randomUnit() + 1) / 2;   // 30..60°
        const float wingDeg = 30 + 30 * (randomUnit() + 1) / 2;

        ImuOrientation found;
        const char* error = calibrate(truth, noseDeg, wingDeg, NO_BIAS, found);
        TEST_ASSERT_NULL(error);
        worst = max(worst, worstAxisErrorDeg(truth, found));
    }
    Serial.printf("  300 случайных установок: худшая ошибка оси %.3f°\n", worst);
    TEST_ASSERT_LESS_THAN_FLOAT(0.5f, worst);
}

// Смещение нуля акселерометра (~0.05g) и неровные позы: самолёт в
// позе "ровно" всё равно показывает 0°, а наклоны читаются с
// ошибкой в пределах нескольких градусов.
void test_bias_and_sloppy_poses()
{
    const Matrix truth = rotation(0.2f, 0.9f, -0.3f, 140);
    const float bias[3] = { 0.04f, -0.03f, 0.03f };

    ImuOrientation found;
    TEST_ASSERT_NULL(calibrate(truth, 45, 40, bias, found));

    const float levelUp[3] = { 0, 0, 1 };
    float level[3];
    chipReading(truth, levelUp, bias, level);
    Serial.printf("  level reads %.2f° from level\n", found.tiltFromLevelDeg(level));
    TEST_ASSERT_LESS_THAN_FLOAT(0.1f, found.tiltFromLevelDeg(level));

    // Нос поднят на 20°: прочитанный наклон ≈ 20°.
    const float pitchedUp[3] = { sinf(20 * DEG_TO_RAD), 0, cosf(20 * DEG_TO_RAD) };
    float pitched[3];
    chipReading(truth, pitchedUp, bias, pitched);
    const float read = found.tiltFromLevelDeg(pitched);
    Serial.printf("  20° pitch reads %.2f°\n", read);
    TEST_ASSERT_FLOAT_WITHIN(4.0f, 20.0f, read);

    // Нос поднят с попутным креном 8° — перекос одной позы гасится
    // второй наполовину.
    const float sloppyNose[3] = { sinf(40 * DEG_TO_RAD), sinf(8 * DEG_TO_RAD), cosf(40 * DEG_TO_RAD) };
    float level2[3], nose2[3], wing2[3];
    const float wingUp[3] = { 0, sinf(35 * DEG_TO_RAD), cosf(35 * DEG_TO_RAD) };
    chipReading(truth, levelUp, NO_BIAS, level2);
    chipReading(truth, sloppyNose, NO_BIAS, nose2);
    chipReading(truth, wingUp, NO_BIAS, wing2);
    TEST_ASSERT_NULL(ImuOrientation::fromPoses(level2, nose2, wing2, found));
    const float axisError = worstAxisErrorDeg(truth, found);
    Serial.printf("  nose pose with 8° accidental roll: axis error %.2f°\n", axisError);
    TEST_ASSERT_LESS_THAN_FLOAT(8.0f, axisError);
}

void test_pilot_mistakes_are_rejected()
{
    const Matrix truth = rotation(1, 0, 0, 180);   // вверх ногами
    ImuOrientation found;

    const char* noseDown = calibrate(truth, -40, 35, NO_BIAS, found);
    const char* leftWing = calibrate(truth, 40, -35, NO_BIAS, found);
    const char* tooLittle = calibrate(truth, 10, 35, NO_BIAS, found);
    const char* tooMuch = calibrate(truth, 40, 85, NO_BIAS, found);

    // Шаг 2 и шаг 3 вокруг одной оси: оба раза подняли нос.
    const float levelUp[3] = { 0, 0, 1 };
    const float noseUp[3] = { sinf(40 * DEG_TO_RAD), 0, cosf(40 * DEG_TO_RAD) };
    const float noseUpMore[3] = { sinf(60 * DEG_TO_RAD), 0, cosf(60 * DEG_TO_RAD) };
    float level[3], nose[3], nose2[3];
    chipReading(truth, levelUp, NO_BIAS, level);
    chipReading(truth, noseUp, NO_BIAS, nose);
    chipReading(truth, noseUpMore, NO_BIAS, nose2);
    const char* sameAxis = ImuOrientation::fromPoses(level, nose, nose2, found);

    Serial.printf("  nose down:  %s\n  left wing:  %s\n  10°:        %s\n  85°:        %s\n  same axis:  %s\n",
                  noseDown ? noseDown : "(accepted!)", leftWing ? leftWing : "(accepted!)",
                  tooLittle ? tooLittle : "(accepted!)", tooMuch ? tooMuch : "(accepted!)",
                  sameAxis ? sameAxis : "(accepted!)");
    TEST_ASSERT_NOT_NULL(noseDown);
    TEST_ASSERT_NOT_NULL(leftWing);
    TEST_ASSERT_NOT_NULL(tooLittle);
    TEST_ASSERT_NOT_NULL(tooMuch);
    TEST_ASSERT_NOT_NULL(sameAxis);
}

// Старый способ (Config::IMU_ROTATION_CW_DEG) даёт ту же матрицу, что
// и раньше делал SensorMounting::rotateToBody.
void test_yaw_steps_match_legacy_mounting()
{
    const uint16_t steps[] = { 0, 90, 180, 270 };
    for (uint16_t step : steps)
    {
        const ImuOrientation o = ImuOrientation::fromYawSteps(step);
        const float chip[3] = { 0.3f, -0.7f, 0.64f };
        float body[3], legacyX, legacyY;
        o.apply(chip, body);
        SensorMounting::rotateToBody(step, chip[0], chip[1], legacyX, legacyY);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, legacyX, body[0]);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, legacyY, body[1]);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, chip[2], body[2]);
    }
}


void setUp() {}
void tearDown() {}

static int runAllTests()
{
    UNITY_BEGIN();
    RUN_TEST(test_known_mountings);
    RUN_TEST(test_random_mountings);
    RUN_TEST(test_bias_and_sloppy_poses);
    RUN_TEST(test_pilot_mistakes_are_rejected);
    RUN_TEST(test_yaw_steps_match_legacy_mounting);
    return UNITY_END();
}

#ifdef ARDUINO
void setup()
{
    delay(2000);
    runAllTests();
}

void loop() {}
#else
int main()
{
    fake::setSerialEcho(true);
    return runAllTests();
}
#endif
