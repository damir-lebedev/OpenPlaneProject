#pragma once
// ============================================================
// DEBUG LOGGER
//
// Периодический вывод состояния в Serial, полностью отдельно от
// flight logic. Autopilot опционален — без него
// печатается только RC/ARM/failsafe/выходы, как раньше.
//
// Кадр сначала собирается в буфер и сравнивается с предыдущим —
// если ничего не изменилось (см. Config::DEBUG_ONLY_ON_CHANGE),
// он не отправляется в Serial. Иначе VSCode Serial Monitor (и
// любой другой простой лог-вьюер без поддержки ANSI) заваливает
// одинаковыми строками каждые DEBUG_INTERVAL_MS, даже когда
// самолёт просто лежит на столе.
// ============================================================

class DebugLogger
{
public:

    explicit DebugLogger(
        FlightController& controller,
        Autopilot* autopilot = nullptr,
        const LoopStats* loopStats = nullptr
    )
        : controller(controller),
          autopilot(autopilot),
          loopStats(loopStats)
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

        // Системная строка — раз в SYSTEM_INTERVAL_MS, независимо от
        // того, менялось ли что-то (частота цикла и счётчики ошибок
        // меняются постоянно и забили бы основной кадр).
        if (now - lastSystemTime >= SYSTEM_INTERVAL_MS)
        {
            lastSystemTime = now;
            printSystem();
        }
    }


private:

    // Print, который просто копит текст в буфер вместо отправки в Serial —
    // так можно собрать целый кадр и сравнить его целиком с предыдущим.
    //
    // Размер с запасом: 10 каналов + RX/ARM/OUT (~220) + Autopilot
    // mode/imu/baro/corr + mag + gps (~350 при всех датчиках сразу) —
    // реалистичный максимум около 600 символов.
    static constexpr size_t FRAME_BUFFER_SIZE = 900;

    class CapturePrint : public Print
    {
    public:
        size_t write(uint8_t c) override
        {
            if (length + 1 < sizeof(buffer))
            {
                buffer[length++] = static_cast<char>(c);
                buffer[length] = '\0';
            }

            return 1;
        }

        void reset()
        {
            length = 0;
            buffer[0] = '\0';
        }

        const char* c_str() const { return buffer; }

    private:
        char buffer[FRAME_BUFFER_SIZE];
        size_t length = 0;
    };

    static constexpr uint32_t SYSTEM_INTERVAL_MS = 10000;

    FlightController& controller;
    Autopilot* autopilot;
    const LoopStats* loopStats;

    uint32_t lastDebugTime = 0;
    uint32_t lastSystemTime = 0;

    CapturePrint capture;
    char previousFrame[FRAME_BUFFER_SIZE] = {0};
    bool hasPreviousFrame = false;

    // "Отображаемые" версии шумных RC/PWM величин — держат старое значение,
    // пока разница не превысит допуск, чтобы дребезг в 1-2 мкс не считался
    // изменением и не расталкивал кадр в Serial.
    uint16_t shownChannels[Config::IBUS_CHANNELS] = {0};
    FlightOutputState shownOutput;
    bool snapshotInitialized = false;

    static uint16_t absDiff(uint16_t a, uint16_t b)
    {
        return a > b ? a - b : b - a;
    }

    static void applyDeadband(uint16_t raw, uint16_t& shown)
    {
        if (absDiff(raw, shown) > Config::DEBUG_CHANGE_DEADBAND_US)
        {
            shown = raw;
        }
    }

    void printState()
    {
        const RcChannelState& rc = controller.getRcState();
        const FlightOutputState& output = controller.getOutputState();

        if (!snapshotInitialized)
        {
            for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
            {
                shownChannels[i] = rc.get(i);
            }

            shownOutput = output;
            snapshotInitialized = true;
        }

        capture.reset();
        capture.print("IBUS: ");

        for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
        {
            applyDeadband(rc.get(i), shownChannels[i]);

            capture.print("CH");
            capture.print(i + 1);
            capture.print("=");
            capture.print(shownChannels[i]);
            capture.print(" ");
        }

        const IBusReceiver& receiver = controller.getReceiver();

        capture.print("| RX=");
        if (receiver.isFrameTimeout())
        {
            capture.print("LOST(нет кадров)");
        }
        else if (receiver.isFailsafeReported())
        {
            capture.print("LOST(failsafe пульта)");
        }
        else
        {
            capture.print("OK");
        }

        capture.print(" | ARM=");
        capture.print(controller.isArmed() ? "YES" : "NO");

        applyDeadband(output.aileronLeft, shownOutput.aileronLeft);
        applyDeadband(output.aileronRight, shownOutput.aileronRight);
        applyDeadband(output.elevator, shownOutput.elevator);
        applyDeadband(output.throttle, shownOutput.throttle);

        capture.print(" | OUT LAIL=");
        capture.print(shownOutput.aileronLeft);
        capture.print(" RAIL=");
        capture.print(shownOutput.aileronRight);
        capture.print(" ELE=");
        capture.print(shownOutput.elevator);
        capture.print(" ESC=");
        capture.println(shownOutput.throttle);

        if (autopilot)
        {
            autopilot->printStatus(capture);
        }

        const bool changed = !hasPreviousFrame || strcmp(capture.c_str(), previousFrame) != 0;

        if (!Config::DEBUG_ONLY_ON_CHANGE || changed)
        {
            // Пустая строка перед кадром — чтобы соседние обновления не
            // сливались в одну кашу, когда печать идёт не каждый тик.
            Serial.println();
            Serial.print(capture.c_str());

            strncpy(previousFrame, capture.c_str(), sizeof(previousFrame) - 1);
            previousFrame[sizeof(previousFrame) - 1] = '\0';
            hasPreviousFrame = true;
        }
    }

    void printSystem() const
    {
        const IBusReceiver& receiver = controller.getReceiver();

        Serial.println();
        Serial.print("SYS: loop ");
        if (loopStats)
        {
            Serial.print(loopStats->hz); Serial.print(" Hz, avg ");
            Serial.print(loopStats->avgUs); Serial.print(" us, max ");
            Serial.print(loopStats->maxUs); Serial.print(" us");
        }
        else
        {
            Serial.print("n/a");
        }

        Serial.print(" | iBUS ok="); Serial.print(receiver.getGoodFrameCount());
        Serial.print(" crc_err="); Serial.print(receiver.getBadFrameCount());
        Serial.print(" | heap "); Serial.print(ESP.getFreeHeap() / 1024); Serial.print(" KB");
        Serial.print(" | uptime "); Serial.print(millis() / 1000); Serial.println(" s");
    }
};
