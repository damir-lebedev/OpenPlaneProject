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

    void load()
    {
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, true)) return;
        roll = prefs.getFloat("roll", 0.0f);
        pitch = prefs.getFloat("pitch", 0.0f);
        prefs.end();
        roll = clampTrim(roll);
        pitch = clampTrim(pitch);
    }

    // levelFlight — самолёт летит ровно и не вращается; rollCommandUs/
    // pitchCommandUs — команда рулей без триммера.
    void update(bool active, bool levelFlight, float rollCommandUs, float pitchCommandUs, float dtS)
    {
        if (!active || !levelFlight || dtS <= 0) return;

        roll = clampTrim(roll + rollCommandUs * Config::AUTOTRIM_RATE * dtS);
        pitch = clampTrim(pitch + pitchCommandUs * Config::AUTOTRIM_RATE * dtS);
        dirty = true;
    }

    // Вызывается после DISARM на земле: записывает, только если триммер менялся.
    bool saveIfChanged()
    {
        if (!dirty) return false;

        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, false)) return false;
        prefs.putFloat("roll", roll);
        prefs.putFloat("pitch", pitch);
        prefs.end();
        dirty = false;

        Serial.print("AutoTrim: сохранён крен ");
        Serial.print(roll, 0);
        Serial.print(" мкс, тангаж ");
        Serial.print(pitch, 0);
        Serial.println(" мкс");
        return true;
    }

    void reset()
    {
        roll = 0;
        pitch = 0;
        dirty = true;
    }

    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }


private:

    static constexpr const char* NVS_NAMESPACE = "autotrim";

    float roll = 0;
    float pitch = 0;
    bool dirty = false;

    static float clampTrim(float value)
    {
        return constrain(value, -Config::AUTOTRIM_MAX_US, Config::AUTOTRIM_MAX_US);
    }
};
