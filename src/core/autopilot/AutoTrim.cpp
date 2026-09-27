// Реализация autopilot/AutoTrim.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "autopilot/AutoTrim.h"


auto AutoTrim::load() -> void
{
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) return;
    roll = prefs.getFloat("roll", 0.0f);
    pitch = prefs.getFloat("pitch", 0.0f);
    prefs.end();
    roll = clampTrim(roll);
    pitch = clampTrim(pitch);
}

auto AutoTrim::update(bool active, bool levelFlight, float rollCommandUs, float pitchCommandUs, float dtS) -> void
{
    if (!active || !levelFlight || dtS <= 0) return;

    roll = clampTrim(roll + rollCommandUs * Config::AUTOTRIM_RATE * dtS);
    pitch = clampTrim(pitch + pitchCommandUs * Config::AUTOTRIM_RATE * dtS);
    dirty = true;
}

auto AutoTrim::saveIfChanged() -> bool
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

auto AutoTrim::reset() -> void
{
    roll = 0;
    pitch = 0;
    dirty = true;
}

auto AutoTrim::clampTrim(float value) -> float
{
    return constrain(value, -Config::AUTOTRIM_MAX_US, Config::AUTOTRIM_MAX_US);
}
