#pragma once
#include <Arduino.h>
#include <Preferences.h>

#include "config/Config.h"

// ============================================================
// АВТОТРИММЕР (Feature::AUTO_TRIM)
//
// Самолёт после сборки почти никогда не летит ровно "сам": то крыло
// тяжелее, то руль высоты чуть не в нуле. Пилот держит стик чуть в
// стороне — это и есть нужный триммер. Пока тумблер автотриммера
// включён и самолёт летит ровно, постоянная команда рулей
// (стик в MANUAL, выход ПИД в режимах со стабилизацией) медленно
// "перетекает" в триммер: trim += команда · AUTOTRIM_RATE · dt.
// Пилот при этом отпускает стик — через несколько секунд самолёт
// летит ровно без его участия. Так же работает SERVO AUTOTRIM в INAV.
//
// Триммер добавляется к рулям во всех режимах (это настройка
// самолёта, а не режима) и хранится в NVS (на STM32 — во флеше).
// Запись — после DISARM, когда самолёт стоит (Autopilot::looksLanded):
// на ESP32 запись NVS останавливает оба ядра на ~0.4 с.
// ============================================================

class AutoTrim
{
public:

    void load();

    // levelFlight — самолёт летит ровно и не вращается; rollCommandUs/
    // pitchCommandUs — команда рулей без триммера.
    void update(bool active, bool levelFlight, float rollCommandUs, float pitchCommandUs, float dtS);

    // Вызывается после DISARM на земле: записывает, только если триммер менялся.
    bool saveIfChanged();

    void reset();

    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }


private:

    static constexpr const char* NVS_NAMESPACE = "autotrim";

    float roll = 0;
    float pitch = 0;
    bool dirty = false;

    static float clampTrim(float value);
};
