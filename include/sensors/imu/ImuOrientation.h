#pragma once
#include <Arduino.h>
#include <Preferences.h>

#include "sensors/SensorMounting.h"

// ============================================================
// IMU ORIENTATION — как IMU стоит в самолёте
//
// Матрица поворота из осей чипа в оси самолёта (X к носу, Y влево,
// Z вверх): body = R · chip. Строки R — оси самолёта, записанные в
// осях чипа.
//
// Два источника:
//   • Config::IMU_ROTATION_CW_DEG — поворот вокруг вертикали шагами
//     по 90°, плата обязательно чипом вверх (fromYawSteps);
//   • калибровка по трём позам (fromPoses) — плата стоит КАК УГОДНО:
//     под любым углом, вверх ногами, на боку. Акселерометр в покое
//     показывает направление "вверх" в осях чипа:
//       ровно                 -> ось Z самолёта (и заодно горизонт);
//       нос поднят            -> "верх" отклонился к носу: его часть,
//                                перпендикулярная Z, — ось X;
//       правое крыло опущено  -> "верх" отклонился влево: ось Y.
//     X по носу и X по крылу (Y × Z) должны совпасть — это проверка,
//     что наклоняли то, что просили, и в ту сторону. Итог — среднее
//     двух оценок, то есть небольшой перекос в одной позе
//     наполовину гасится другой.
//
// Горизонт при этом тоже берётся из позы "ровно": смещение нуля
// акселерометра входит в неё и потому не мешает.
// ============================================================

class ImuOrientation
{
public:

    ImuOrientation();

    // Поворот вокруг вертикали шагами по 90° (Config::IMU_ROTATION_CW_DEG),
    // плата чипом вверх.
    static ImuOrientation fromYawSteps(uint16_t rotationCwDeg);

    // level, noseUp, rightWingDown — "верх" (среднее показание
    // акселерометра в покое, g) в осях чипа в трёх позах.
    // nullptr — успех (результат в out), иначе — причина отказа.
    static const char* fromPoses(const float level[3], const float noseUp[3],
                                 const float rightWingDown[3], ImuOrientation& out);

    void apply(const float chip[3], float body[3]) const;

    // Угол между измеренным "верхом" (оси чипа) и осью Z самолёта, °.
    float tiltFromLevelDeg(const float chipUp[3]) const;

    // --- NVS ---

    bool load(const char* nvsNamespace);

    void save(const char* nvsNamespace) const;

    // Словами: какая ось чипа смотрит в нос и какая вверх.
    void describe(Print& out) const;


private:

    struct NoInit {};
    explicit ImuOrientation(NoInit) {}

    static constexpr float MIN_TILT_DEG = 20.0f;
    static constexpr float MAX_TILT_DEG = 80.0f;

    // Оценки носа по шагам 2 и 3 должны совпасть с точностью ~25°.
    static constexpr float AGREEMENT_MIN_COS = 0.9f;

    float r[3][3] = {};

    static float dot(const float a[3], const float b[3]);

    static void cross(const float a[3], const float b[3], float out[3]);

    static bool normalize(const float v[3], float out[3]);

    // Единичная часть v, перпендикулярная единичному axis.
    static void perpendicularPart(const float v[3], const float axis[3], float out[3]);

    static bool tiltInRange(const float levelUnit[3], const float poseUnit[3]);

    // Строки ортонормированы и тройка правая — иначе в NVS мусор.
    bool isRotation() const;

    static void describeAxis(Print& out, const float axis[3]);
};
