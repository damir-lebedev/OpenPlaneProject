#pragma once

// ============================================================
// FEATURE MANAGER
//
// Назначает 4 функции автопилота на 4 свободных вспомогательных
// канала (Channels::FEATURE_SLOTS — CH6, CH7, CH9, CH10; CH8
// занят boost'ом в ThrottleManager). Переключатель: < 1500 µs —
// выключено, >= 1500 µs — включено. Если активно несколько
// слотов сразу, выигрывает слот с меньшим индексом.
// ============================================================

#include "Autopilot.h"
#include "RcChannelState.h"
#include <Arduino.h>

enum FeatureType
{
    FEATURE_DISABLED = 0,
    FEATURE_AUTO_TAKEOFF = 1,
    FEATURE_ALT_HOLD = 2,
    FEATURE_STABILIZE = 3,
    FEATURE_MANUAL = 4
};

constexpr uint8_t FEATURE_SLOT_COUNT = 4;

struct FeatureState
{
    FeatureType feature = FEATURE_DISABLED;
    uint16_t lastRawValue = 1500;
    bool isActive = false;
    unsigned long lastChangeTime = 0;
};

class FeatureManager
{
public:

    explicit FeatureManager(Autopilot* autopilot = nullptr)
        : autopilot(autopilot)
    {
        // Назначения по умолчанию, можно поменять через assignFeature().
        assignFeature(0, FEATURE_AUTO_TAKEOFF);
        assignFeature(1, FEATURE_ALT_HOLD);
        assignFeature(2, FEATURE_STABILIZE);
        assignFeature(3, FEATURE_MANUAL);
    }

    bool begin()
    {
        if (!autopilot)
        {
            Serial.println("FeatureManager: Autopilot не подключён, функции отключены");
            return false;
        }

        printConfiguration();
        return true;
    }

    // Вызывается из FlightController::update() с текущими RC-каналами.
    void update(const RcChannelState& rcState)
    {
        for (uint8_t slot = 0; slot < FEATURE_SLOT_COUNT; ++slot)
        {
            processSlot(slot, rcState.get(Channels::FEATURE_SLOTS[slot]));
        }

        if (autopilot)
        {
            autopilot->setMode(getActiveMode());
        }
    }

    // slot: 0..3, см. Channels::FEATURE_SLOTS.
    void assignFeature(uint8_t slot, FeatureType feature)
    {
        if (slot >= FEATURE_SLOT_COUNT) return;
        features[slot].feature = feature;
    }

    FeatureType getFeature(uint8_t slot) const
    {
        return slot < FEATURE_SLOT_COUNT ? features[slot].feature : FEATURE_DISABLED;
    }

    bool isFeatureActive(uint8_t slot) const
    {
        return slot < FEATURE_SLOT_COUNT && features[slot].isActive;
    }

    // Физический номер канала (CH1..CH10) для данного слота — для UI и логов.
    static uint8_t slotChannelNumber(uint8_t slot)
    {
        return slot < FEATURE_SLOT_COUNT
            ? static_cast<uint8_t>(Channels::FEATURE_SLOTS[slot] + 1)
            : 0;
    }

    AutopilotMode getActiveMode() const
    {
        for (uint8_t slot = 0; slot < FEATURE_SLOT_COUNT; ++slot)
        {
            if (!features[slot].isActive) continue;

            switch (features[slot].feature)
            {
                case FEATURE_AUTO_TAKEOFF: return MODE_AUTO_TAKEOFF;
                case FEATURE_ALT_HOLD:     return MODE_ALT_HOLD;
                case FEATURE_STABILIZE:    return MODE_STABILIZE;
                case FEATURE_MANUAL:       return MODE_MANUAL;
                default:                   break;
            }
        }

        return MODE_MANUAL;
    }

    void printStatus() const
    {
        Serial.print("Features: mode=");
        Serial.print(modeToString(getActiveMode()));

        for (uint8_t slot = 0; slot < FEATURE_SLOT_COUNT; ++slot)
        {
            Serial.print(" CH");
            Serial.print(slotChannelNumber(slot));
            Serial.print("=");
            Serial.print(featureToString(features[slot].feature));
            Serial.print(features[slot].isActive ? "(on)" : "(off)");
        }

        Serial.println();
    }

    void printConfiguration() const
    {
        Serial.println("FeatureManager: конфигурация каналов");

        for (uint8_t slot = 0; slot < FEATURE_SLOT_COUNT; ++slot)
        {
            Serial.print("  CH");
            Serial.print(slotChannelNumber(slot));
            Serial.print(" -> ");
            Serial.println(featureToString(features[slot].feature));
        }
    }

    // Для веб-интерфейса.
    void getConfigAsJSON(char* buffer, size_t bufferSize) const
    {
        snprintf(buffer, bufferSize,
                 "{\"ch\":[%d,%d,%d,%d],\"feature\":[%d,%d,%d,%d],"
                 "\"active\":[%s,%s,%s,%s],\"active_mode\":%d}",
                 slotChannelNumber(0), slotChannelNumber(1),
                 slotChannelNumber(2), slotChannelNumber(3),
                 features[0].feature, features[1].feature,
                 features[2].feature, features[3].feature,
                 features[0].isActive ? "true" : "false",
                 features[1].isActive ? "true" : "false",
                 features[2].isActive ? "true" : "false",
                 features[3].isActive ? "true" : "false",
                 getActiveMode());
    }

private:

    Autopilot* autopilot;
    FeatureState features[FEATURE_SLOT_COUNT];

    void processSlot(uint8_t slot, uint16_t rawValue)
    {
        // Фильтр дребезга: игнорируем изменения меньше 50 µs.
        if (abs((int)rawValue - (int)features[slot].lastRawValue) < 50)
        {
            return;
        }

        features[slot].lastRawValue = rawValue;

        const bool wasActive = features[slot].isActive;
        features[slot].isActive = (rawValue >= 1500);

        if (features[slot].isActive != wasActive)
        {
            features[slot].lastChangeTime = millis();

            Serial.print("Feature CH");
            Serial.print(slotChannelNumber(slot));
            Serial.print(" (");
            Serial.print(featureToString(features[slot].feature));
            Serial.println(features[slot].isActive ? ") ВКЛ" : ") ВЫКЛ");
        }
    }

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
