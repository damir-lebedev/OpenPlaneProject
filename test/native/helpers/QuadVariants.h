#pragma once

// ============================================================
// «Не та» физика для замкнутых полётов (helpers/QuadSim.h): модель
// нарочно не совпадает с описанием сборки, по которому выведена настройка.
//   • «вялая»: винт тянет на 20 % меньше, мотор и рама на 30 % инерционнее,
//     коптер на 20 % тяжелее, центр тяжести сдвинут вперёд;
//   • «резвая»: тяга +20 %, инерция −30 %, масса −15 %, центр тяжести
//     сдвинут вбок — усиление контура почти вдвое выше расчётного.
// Моторы при этом разбросаны между собой на ±5 %.
// ============================================================

#include "helpers/QuadSim.h"

inline QuadPerturb sluggish()
{
    QuadPerturb p;
    p.thrust = 0.8;
    p.resistance = 1.3;
    p.rotorInertia = 1.3;
    p.bodyInertia = 1.3;
    p.mass = 1.2;
    p.motorSpread = 0.05;
    p.cgX = 0.08;
    return p;
}

inline QuadPerturb twitchy()
{
    QuadPerturb p;
    p.thrust = 1.2;
    p.torque = 1.2;
    p.resistance = 0.75;
    p.rotorInertia = 0.7;
    p.bodyInertia = 0.7;
    p.mass = 0.85;
    p.motorSpread = 0.05;
    p.cgY = 0.05;
    return p;
}
