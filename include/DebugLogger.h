#pragma once
// ============================================================
// DEBUG LOGGER
//
// Периодический вывод состояния в Serial, полностью отдельно от
// flight logic. Autopilot/FeatureManager опциональны — без них
// печатается только RC/ARM/failsafe/выходы, как раньше.
// ============================================================

class DebugLogger
{
public:

    explicit DebugLogger(
        FlightController& controller,
        Autopilot* autopilot = nullptr,
        FeatureManager* featureManager = nullptr
    )
        : controller(controller),
          autopilot(autopilot),
          featureManager(featureManager)
    {
    }

    void update()
    {
        const uint32_t now = millis();

        if (now - lastDebugTime < Config::DEBUG_INTERVAL_MS)
        {
            return;
        }

        lastDebugTime = now;

        printState();
    }


private:

    FlightController& controller;
    Autopilot* autopilot;
    FeatureManager* featureManager;

    uint32_t lastDebugTime = 0;

    void printState()
    {
        const RcChannelState& rc = controller.getRcState();
        const FlightOutputState& output = controller.getOutputState();

        Serial.print("IBUS: ");

        for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
        {
            Serial.print("CH");
            Serial.print(i + 1);
            Serial.print("=");
            Serial.print(rc.get(i));
            Serial.print(" ");
        }

        Serial.print("| RX=");
        Serial.print(controller.isReceiverFailsafe() ? "LOST" : "OK");

        Serial.print(" | ARM=");
        Serial.print(controller.isArmed() ? "YES" : "NO");

        Serial.print(" | BOOST=");
        Serial.print(controller.isBoostActive() ? "ON" : "OFF");

        Serial.print(" | OUT LAIL=");
        Serial.print(output.aileronLeft);
        Serial.print(" RAIL=");
        Serial.print(output.aileronRight);
        Serial.print(" ELE=");
        Serial.print(output.elevator);
        Serial.print(" ESC=");
        Serial.println(output.throttle);

        if (autopilot)
        {
            autopilot->printStatus();
        }

        if (featureManager)
        {
            featureManager->printStatus();
        }
    }
};
