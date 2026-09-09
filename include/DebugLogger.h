// ============================================================
// 12. DEBUG LOGGER
//
// Debug полностью отделён от flight logic.
//
// В будущем этот класс можно заменить на:
//
// SerialLogger
// TelemetryLogger
// WiFiLogger
// WebSocketLogger
// SDLogger
//
// При этом FlightController менять не потребуется.
// ============================================================

class DebugLogger
{
public:

    explicit DebugLogger(
        FlightController& controller
    )
        : controller(controller)
    {
    }


    // --------------------------------------------------------
    // Периодический вывод состояния.
    // --------------------------------------------------------

    void update()
    {
        const uint32_t now = millis();

        if (
            now - lastDebugTime <
            Config::DEBUG_INTERVAL_MS
        )
        {
            return;
        }


        lastDebugTime = now;

        printState();
    }


private:

    FlightController& controller;

    uint32_t lastDebugTime = 0;


    // --------------------------------------------------------
    // Вывод полного текущего состояния.
    // --------------------------------------------------------

    void printState()
    {
        const RcChannelState& rc =
            controller.getRcState();

        const FlightOutputState& output =
            controller.getOutputState();


        // ----------------------------------------------------
        // RC channels.
        // ----------------------------------------------------

        Serial.print("IBUS: ");

        for (
            uint8_t i = 0;
            i < Config::IBUS_CHANNELS;
            ++i
        )
        {
            Serial.print("CH");
            Serial.print(i + 1);
            Serial.print("=");

            Serial.print(rc.get(i));

            Serial.print(" ");
        }


        // ----------------------------------------------------
        // Receiver status.
        // ----------------------------------------------------

        Serial.print("| RX=");

        Serial.print(
            controller.isReceiverFailsafe()
                ? "LOST"
                : "OK"
        );


        // ----------------------------------------------------
        // ARM status.
        // ----------------------------------------------------

        Serial.print(" | ARM=");

        Serial.print(
            controller.isArmed()
                ? "YES"
                : "NO"
        );


        // ----------------------------------------------------
        // Boost status.
        // ----------------------------------------------------

        Serial.print(" | BOOST=");

        Serial.print(
            controller.isBoostActive()
                ? "ON"
                : "OFF"
        );


        // ----------------------------------------------------
        // Calculated outputs.
        // ----------------------------------------------------

        Serial.print(" | OUT LAIL=");
        Serial.print(output.aileronLeft);

        Serial.print(" RAIL=");
        Serial.print(output.aileronRight);

        Serial.print(" ELE=");
        Serial.print(output.elevator);

        Serial.print(" ESC=");
        Serial.println(output.throttle);
    }
};