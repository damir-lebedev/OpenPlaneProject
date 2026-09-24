#pragma once

// ============================================================
// PHASE TARGETS — чего хочет автоматический этап полёта
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
//
// Общий выход TakeoffSequencer и LandingSequencer: цели по углам и
// газ. Сами рули считает AdaptiveRateController — этапы полёта
// говорят только "что", а не "как", поэтому им не важно, какой у
// самолёта руль и на какой он скорости.
// ============================================================

struct PhaseTargets
{
    bool active = false;             // этап сейчас управляет самолётом

    float targetRollDeg = 0;
    float targetPitchDeg = 0;

    // false — ось не трогать, руль остаётся у пилота. Тангаж на земле
    // (разбег, пробег) задаёт шасси, а не руль: регулятор упёрся бы в
    // предел, пытаясь выровнять стоящий на колёсах самолёт.
    bool controlRoll = true;
    bool controlPitch = true;

    // Держать курс рулём направления и колесом (разбег, пробег).
    bool holdHeading = false;
    float headingDeg = 0;

    float throttlePercent = -1.0f;   // −1 — газ пилота

    const char* reason = "";
};
