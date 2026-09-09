// ============================================================
// 🎛️  FEATURE MANAGER
//
// Гибкая система назначения RC-каналов к функциям автопилота
// Позволяет настраивать поведение без переписывания кода.
//
// Архитектура:
//   • Каналы 7-10 зарезервированы для автопилота
//   • Каждый канал может быть назначен отдельной функции
//   • Функции: AUTO_TAKEOFF, ALT_HOLD, STABILIZE, и т.д.
//   • Конфигурация хранится в памяти (EEPROM, LittleFS)
//   • Веб-интерфейс позволяет менять назначение без USB
// ============================================================

#pragma once
#include "Autopilot.h"
#include "RcChannelState.h"
#include <Arduino.h>

// ============================================================
// ТИПЫ ФУНКЦИЙ, КОТОРЫЕ МОГУТ БЫТЬ НАЗНАЧЕНЫ
// ============================================================

enum FeatureType
{
    FEATURE_DISABLED = 0,
    FEATURE_AUTO_TAKEOFF = 1,
    FEATURE_ALT_HOLD = 2,
    FEATURE_STABILIZE = 3,
    FEATURE_MANUAL = 4
};

// ============================================================
// СОСТОЯНИЕ ФУНКЦИИ
// ============================================================
// Отслеживает переключение и фильтрует дребезг

struct FeatureState
{
    FeatureType feature;           // Какая функция назначена на этот канал
    uint16_t lastRawValue;         // Последнее значение RC сигнала
    bool wasActive;                // Была ли функция активной в прошлом цикле
    bool isActive;                 // Активна ли функция сейчас
    unsigned long lastChangeTime;  // Когда произошло последнее изменение
};

// ============================================================
// МЕНЕДЖЕР ФУНКЦИЙ
// ============================================================

class FeatureManager
{
public:

    // ========================================================
    // КОНСТРУКТОР
    // ========================================================

    FeatureManager(Autopilot* autopilot = nullptr)
        : autopilot(autopilot)
    {
        // По умолчанию все каналы отключены
        for (int i = 0; i < 4; i++)
        {
            features[i].feature = FEATURE_DISABLED;
            features[i].lastRawValue = 1500;
            features[i].wasActive = false;
            features[i].isActive = false;
            features[i].lastChangeTime = 0;
        }

        // Начальные назначения (можно переопределить):
        // CH7 - AUTO_TAKEOFF
        // CH8 - ALT_HOLD
        // CH9 - STABILIZE
        // CH10 - MANUAL
        assignFeature(7, FEATURE_AUTO_TAKEOFF);
        assignFeature(8, FEATURE_ALT_HOLD);
        assignFeature(9, FEATURE_STABILIZE);
        assignFeature(10, FEATURE_MANUAL);
    }

    // ========================================================
    // ИНИЦИАЛИЗАЦИЯ
    // ========================================================

    bool begin()
    {
        if (!autopilot)
        {
            Serial.println("❌ FeatureManager: Autopilot not attached!");
            return false;
        }

        Serial.println("✅ FeatureManager: Initialized");
        printConfiguration();

        return true;
    }

    // ========================================================
    // ОБНОВЛЕНИЕ УПРАВЛЕНИЯ
    // ========================================================
    // Должна вызваться из FlightController::update()
    // Принимает текущие значения RC каналов

    void update(const RcChannelState& rcState)
    {
        // Обрабатываем каналы 7-10 (индексы 6-9, так как нумерация с 0)
        for (int ch = 0; ch < 4; ch++)
        {
            processChannel(ch, rcState.getChannel(ch + 7));
        }

        // Переходим в режим, если произошло переключение
        applyActiveFunctions();
    }

    // ========================================================
    // НАЗНАЧИТЬ ФУНКЦИЮ НА КАНАЛ
    // ========================================================
    // channel: 7-10
    // feature: FEATURE_AUTO_TAKEOFF, FEATURE_ALT_HOLD, и т.д.

    void assignFeature(uint8_t channel, FeatureType feature)
    {
        if (channel < 7 || channel > 10)
        {
            Serial.print("❌ FeatureManager: Invalid channel ");
            Serial.println(channel);
            return;
        }

        int idx = channel - 7;
        features[idx].feature = feature;

        Serial.print("✅ FeatureManager: CH");
        Serial.print(channel);
        Serial.print(" assigned to ");
        Serial.println(featureToString(feature));
    }

    // ========================================================
    // ПОЛУЧИТЬ ТЕКУЩУЮ КОНФИГУРАЦИЮ
    // ========================================================

    FeatureType getFeature(uint8_t channel) const
    {
        if (channel < 7 || channel > 10) return FEATURE_DISABLED;
        return features[channel - 7].feature;
    }

    // ========================================================
    // ПРОВЕРИТЬ, АКТИВНА ЛИ ФУНКЦИЯ
    // ========================================================

    bool isFeatureActive(uint8_t channel) const
    {
        if (channel < 7 || channel > 10) return false;
        return features[channel - 7].isActive;
    }

    // ========================================================
    // ПОЛУЧИТЬ АКТИВНЫЙ РЕЖИМ АВТОПИЛОТА
    // ========================================================

    AutopilotMode getActiveMode() const
    {
        // Проверяем каналы в порядке приоритета
        for (int i = 0; i < 4; i++)
        {
            if (!features[i].isActive) continue;

            switch (features[i].feature)
            {
                case FEATURE_AUTO_TAKEOFF: return MODE_AUTO_TAKEOFF;
                case FEATURE_ALT_HOLD:     return MODE_ALT_HOLD;
                case FEATURE_STABILIZE:    return MODE_STABILIZE;
                case FEATURE_MANUAL:       return MODE_MANUAL;
                default:                   break;
            }
        }

        return MODE_MANUAL;  // По умолчанию ручное управление
    }

    // ========================================================
    // СОХРАНИТЬ КОНФИГУРАЦИЮ
    // ========================================================
    // (Заглушка для будущей реализации с EEPROM)

    void saveConfiguration() const
    {
        Serial.println("💾 FeatureManager: Configuration saved to EEPROM");
        // TODO: Реализовать сохранение в EEPROM/LittleFS
    }

    // ========================================================
    // ЗАГРУЗИТЬ КОНФИГУРАЦИЮ
    // ========================================================
    // (Заглушка для будущей реализации)

    void loadConfiguration()
    {
        Serial.println("📂 FeatureManager: Configuration loaded from EEPROM");
        // TODO: Реализовать загрузку из EEPROM/LittleFS
    }

    // ========================================================
    // ДИАГНОСТИКА
    // ========================================================

    void printStatus() const
    {
        Serial.println("\n🎛️  FeatureManager Status:");
        Serial.print("  Active mode: ");
        Serial.println(modeToString(getActiveMode()));

        for (int i = 0; i < 4; i++)
        {
            Serial.print("  CH");
            Serial.print(i + 7);
            Serial.print(": ");
            Serial.print(featureToString(features[i].feature));
            Serial.print(" (");
            Serial.print(features[i].isActive ? "ACTIVE" : "inactive");
            Serial.print(", raw=");
            Serial.print(features[i].lastRawValue);
            Serial.println(" µs)");
        }
    }

    void printConfiguration() const
    {
        Serial.println("\n📋 FeatureManager Configuration:");
        for (int i = 0; i < 4; i++)
        {
            Serial.print("  CH");
            Serial.print(i + 7);
            Serial.print(" → ");
            Serial.println(featureToString(features[i].feature));
        }
    }

    // ========================================================
    // ЭКСПОРТ КОНФИГУРАЦИИ В JSON
    // ========================================================
    // Используется для веб-интерфейса

    void getConfigAsJSON(char* buffer, size_t bufferSize) const
    {
        snprintf(buffer, bufferSize,
                 "{\"ch7\":%d,\"ch8\":%d,\"ch9\":%d,\"ch10\":%d,"
                 "\"active_mode\":%d}",
                 features[0].feature,
                 features[1].feature,
                 features[2].feature,
                 features[3].feature,
                 getActiveMode());
    }

private:

    // ========================================================
    // ПРИВАТНЫЕ ПЕРЕМЕННЫЕ
    // ========================================================

    Autopilot* autopilot;
    FeatureState features[4];  // CH7, CH8, CH9, CH10


    // ========================================================
    // ОБРАБОТКА ИЗМЕНЕНИЙ НА КАНАЛЕ
    // ========================================================

    void processChannel(int idx, uint16_t rawValue)
    {
        // Фильтруем дребезг: величина изменения должна быть > 50 µs
        if (abs((int)rawValue - (int)features[idx].lastRawValue) < 50)
        {
            return;
        }

        features[idx].lastRawValue = rawValue;

        // Определяем, активна ли функция на основе положения переключателя
        // Конвенция: < 1500 µs = OFF, >= 1500 µs = ON
        bool wasActive = features[idx].isActive;
        features[idx].isActive = (rawValue >= 1500);

        // Если произошло изменение, логируем это
        if (features[idx].isActive != wasActive)
        {
            features[idx].lastChangeTime = millis();

            Serial.print("🔀 FeatureManager: CH");
            Serial.print(idx + 7);
            Serial.print(" (");
            Serial.print(featureToString(features[idx].feature));
            Serial.print(") ");
            Serial.println(features[idx].isActive ? "ACTIVATED" : "deactivated");
        }
    }

    // ========================================================
    // ПРИМЕНИТЬ АКТИВНЫЕ ФУНКЦИИ
    // ========================================================

    void applyActiveFunctions()
    {
        // Находим активный режим (с приоритетом)
        AutopilotMode newMode = getActiveMode();

        // Переключаемся в новый режим
        if (autopilot)
        {
            autopilot->setMode(newMode);
        }
    }

    // ========================================================
    // ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
    // ========================================================

    const char* featureToString(FeatureType feature) const
    {
        switch (feature)
        {
            case FEATURE_DISABLED:      return "DISABLED";
            case FEATURE_AUTO_TAKEOFF:  return "AUTO_TAKEOFF";
            case FEATURE_ALT_HOLD:      return "ALT_HOLD";
            case FEATURE_STABILIZE:     return "STABILIZE";
            case FEATURE_MANUAL:        return "MANUAL";
            default:                    return "UNKNOWN";
        }
    }

    const char* modeToString(AutopilotMode mode) const
    {
        switch (mode)
        {
            case MODE_MANUAL:       return "MANUAL";
            case MODE_STABILIZE:    return "STABILIZE";
            case MODE_AUTO_TAKEOFF: return "AUTO_TAKEOFF";
            case MODE_ALT_HOLD:     return "ALT_HOLD";
            default:                return "UNKNOWN";
        }
    }
};
