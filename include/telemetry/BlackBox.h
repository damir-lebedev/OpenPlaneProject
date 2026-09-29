#pragma once
#include <Arduino.h>
#include <esp_system.h>

#include "autopilot/Autopilot.h"
#include "autopilot/AutopilotTypes.h"
#include "autopilot/PilotSwitches.h"
#include "config/Channels.h"
#include "config/Config.h"
#include "control/FlightController.h"
#include "hal/Rtos.h"
#include "rc/IBusReceiver.h"
#include "sensors/SensorInterface.h"
#include "sensors/airspeed/AirspeedSensor.h"
#include "telemetry/BlackBoxFormat.h"
#include "telemetry/BlackBoxRing.h"
#include "telemetry/BlackBoxStorage.h"
#include "telemetry/IFlightRecorder.h"
#include "telemetry/LoopStats.h"

// ============================================================
// ЧЁРНЫЙ ЯЩИК — запись полёта во встроенный флеш
//
// Что пишется (формат — BlackBoxFormat.h, частоты при цикле 500 Гц):
//   IMU   каждый такт — гироскоп, акселерометр, время работы такта;
//   CTRL  100 Гц — углы и цели, стики, команды автопилота, все
//         выходы, составляющие ПИД, режим, флаги, функции;
//   RC    50 Гц — все каналы пульта, счётчики кадров iBUS;
//   BARO, MAG, GPS, AIR — каждый новый отсчёт датчика;
//   NAV   10 Гц; POWER 10 Гц — батарея и датчик тока (S3);
//   SYS   1 Гц — цикл, память, приёмник, сам ящик;
//   EVENT — ARM/DISARM, режимы, связь, отказы датчиков, GPS, функции;
//   в начале полёта — схема записей и параметры (ПИД, триммеры,
//   Config, датчики, привязки тумблеров, причина перезагрузки).
//
// Как устроено:
//   • полётный цикл (ядро 1) каждый такт вызывает update(): снимок
//     кладётся в очередь в PSRAM (BlackBoxRing) — микросекунды, флеш
//     не трогается;
//   • задача "bbox" (ядро 0) после каждого такта пишет во флеш одну
//     страницу (256 байт). Запись флеша останавливает оба ядра на
//     ~0.5 мс — поэтому она идёт сразу после такта, в паузе цикла;
//   • стирание — только в покое на земле: не записывается и не
//     заармлено (BlackBoxStorage::eraseStep). В воздухе флеш не
//     стирается никогда.
//
// Когда пишет: ARM + газ (стик или ESC выше THROTTLE_LOW_US), с
// предзаписью BLACKBOX_PREROLL_MS; после перезагрузки из-за сбоя —
// сразу; вручную — из консоли (стенд). Когда перестаёт — см. Config
// (BLACKBOX_*) и decideStop().
//
// Выгрузка: tools/blackbox.py (list / download / decode), команды
// "bb ..." по UART — handleHostCommand(). Консоль — меню 'k'.
// ============================================================

namespace BlackBoxRate
{
    // Каждый какой такт цикла (Config::LOOP_PERIOD_MS) — период ms.
    constexpr uint32_t every(uint32_t ms)
    {
        return ms / Config::LOOP_PERIOD_MS > 0 ? ms / Config::LOOP_PERIOD_MS : 1;
    }
}

class BlackBox : public IFlightRecorder
{
public:

    enum class State : uint8_t { Off, Idle, Recording, Stopping };

    BlackBox(FlightController& flightController, Autopilot& ap, LoopStats& stats, BlackBoxStorage& flightStorage,
             const PilotSwitches* pilotSwitches = nullptr)
        : controller(flightController),
          autopilot(ap),
          loopStats(stats),
          storage(flightStorage),
          switches(pilotSwitches)
    {
    }

    // Прочитать флеш, выделить очередь, запустить задачу записи.
    // Раздела нет — ящик выключен, полёту это не мешает.
    bool begin(bool startTask = true)
    {
        resetReason = esp_reset_reason();
        resetStartPending = isCrashReset(resetReason);

        if (!storage.begin())
        {
            Serial.println("BlackBox: нет раздела blackbox — запись выключена (partitions_blackbox.csv)");
            return false;
        }

        uint32_t size = Config::BLACKBOX_RING_BYTES;
        uint8_t* memory = psramFound() ? static_cast<uint8_t*>(ps_malloc(size)) : nullptr;
        if (!memory)
        {
            size = Config::BLACKBOX_RING_NO_PSRAM_BYTES;
            memory = static_cast<uint8_t*>(malloc(size));
        }
        if (!memory)
        {
            Serial.println("BlackBox: нет памяти под очередь — запись выключена");
            return false;
        }
        ring.begin(memory, size);

        lock = xSemaphoreCreateMutex();
        state = State::Idle;
#if defined(BOARD_ESP32_S3)
        analogReadMilliVolts(Config::PIN_VBAT_ADC);   // первое чтение — калибровка АЦП, мс
#endif

        // Стёртое впереди — сверить сразу (только чтение, ~0.1 мс на
        // сектор): заармят через секунду после включения — место уже
        // есть. Дальше — в фоне (maintain()).
        for (const uint32_t verifyStart = millis(); millis() - verifyStart < BOOT_VERIFY_MS;)
        {
            if (!storage.eraseStep(storage.totalSectors(), 0, false)) break;
        }

        printStatus(Serial);
        if (resetStartPending)
        {
            Serial.print("BlackBox: перезагрузка из-за сбоя (");
            Serial.print(resetName(resetReason));
            Serial.println(") — запись включается сразу");
        }

        if (startTask)
        {
            xTaskCreatePinnedToCore(writerTask, "bbox", 6144, this, Rtos::PRIORITY_TELEMETRY, &writerHandle, 0);
        }
        return true;
    }

    State getState() const { return state; }
    bool isRecording() const override { return state == State::Recording || state == State::Stopping; }

    // --------------------------------------------------------
    // Полётный цикл (ядро 1): после каждого такта. workUs — сколько
    // длилась работа такта (без ожидания).
    // --------------------------------------------------------

    void update(uint32_t workUs)
    {
        if (state == State::Off) return;

        nowUs = micros();
        nowMs = millis();
        const bool armed = controller.isArmed();
        armedFlag = armed;

        if (state != State::Stopping) detectEvents(armed);
        decideStartStop(armed);

        if (state == State::Idle) ring.trimOlderThan(nowUs, Config::BLACKBOX_PREROLL_MS * 1000UL);
        if (state != State::Stopping) sample(workUs);
        cycle++;

        if (writerHandle) xTaskNotifyGive(writerHandle);
    }

    // Запись вручную (стенд): начать/остановить. Ручная запись
    // останавливается только вручную, пока самолёт не заармят, —
    // дальше как обычная.
    void requestManualStart() override { manualStartRequested = true; }
    void requestManualStop() override { manualStopRequested = true; }
    bool isManual() const { return manualRecording; }

    // --------------------------------------------------------
    // Задача записи (ядро 0): один шаг — не больше одной-двух
    // записей страницы или одного стирания.
    // --------------------------------------------------------

    void writerStep()
    {
        if (state == State::Off || !takeLock(0)) return;

        const State s = state;
        if (s == State::Recording || s == State::Stopping) writeSome();
        else maintain();

        giveLock();
    }

    // --------------------------------------------------------
    // Консоль и ПК
    // --------------------------------------------------------

    void printStatus(Print& out) const override
    {
        out.print("BlackBox: ");
        switch (state)
        {
            case State::Off:       out.print("выключен"); break;
            case State::Idle:      out.print("ждёт ARM и газ"); break;
            case State::Recording: out.printf("ЗАПИСЬ полёта #%u%s", recordingFlight, manualRecording ? " (вручную)" : ""); break;
            case State::Stopping:  out.printf("дописывает полёт #%u", recordingFlight); break;
        }
        if (!storage.isReady())
        {
            out.println();
            return;
        }
        out.printf(" | стёрто впереди %.1f МБ", storage.freeBytes() / 1048576.0);
        if (bytesPerSecond > 0)
        {
            out.printf(" (≈%lu мин)", (unsigned long)(storage.freeBytes() / bytesPerSecond / 60));
        }
        out.printf(" из %.1f МБ | полётов %u",
                   static_cast<double>(storage.totalSectors()) * BlackBoxFormat::SECTOR_SIZE / 1048576.0,
                   (unsigned)storage.flightCount());
        out.println();
    }

    void printFlights(Print& out) override
    {
        takeLock(portMAX_DELAY);
        if (storage.flightCount() == 0) out.println("  полётов нет");
        for (size_t i = 0; i < storage.flightCount(); ++i)
        {
            const BlackBoxStorage::Flight& f = storage.flight(i);
            const uint32_t seconds = (f.lastMs - f.startMs) / 1000;
            out.printf("  #%-4u %6lu КБ  %2lu:%02lu%s\n", f.number,
                       (unsigned long)(f.sectors * BlackBoxFormat::SECTOR_SIZE / 1024),
                       (unsigned long)(seconds / 60), (unsigned long)(seconds % 60),
                       f.hasStart ? "" : "  (начало стёрто)");
        }
        giveLock();
    }

    // Стереть все полёты (консоль). Блокирует: ~40 с на весь раздел.
    bool eraseAll() override
    {
        if (isRecording() || controller.isArmed()) return false;
        takeLock(portMAX_DELAY);
        const bool ok = storage.eraseAll();
        giveLock();
        return ok;
    }

    // Команда с ПК (tools/blackbox.py) — строка после байта STX:
    //   bb list            — полёты: BB:FLIGHT ... и BB:END
    //   bb get <n> [бод]   — выгрузить полёт n кадрами (см. sendFlight)
    // Ответы начинаются с "BB:" — среди строк лога их легко найти.
    void handleHostCommand(const char* line) override
    {
        unsigned flight = 0;
        unsigned long baud = 0;
        if (strcmp(line, "bb list") == 0)
        {
            hostList();
        }
        else if (sscanf(line, "bb get %u %lu", &flight, &baud) >= 1)
        {
            sendFlight(static_cast<uint16_t>(flight), baud ? baud : Serial.baudRate());
        }
        else
        {
            Serial.println("BB:ERR неизвестная команда");
        }
    }


private:

    static constexpr uint32_t CTRL_EVERY = BlackBoxRate::every(10);
    static constexpr uint32_t RC_EVERY = BlackBoxRate::every(20);
    static constexpr uint32_t NAV_EVERY = BlackBoxRate::every(100);
    static constexpr uint32_t SYS_EVERY = BlackBoxRate::every(1000);
    static constexpr uint32_t MAG_MIN_US = 20000;
    static constexpr uint32_t AIR_MIN_US = 20000;

    static constexpr size_t HEADER_BUFFER = 8192;
    static constexpr uint32_t BOOT_VERIFY_MS = 300;
    static constexpr uint8_t FRAME_MAGIC_0 = 0xA5;
    static constexpr uint8_t FRAME_MAGIC_1 = 0x5A;

    FlightController& controller;
    Autopilot& autopilot;
    LoopStats& loopStats;
    BlackBoxStorage& storage;
    const PilotSwitches* switches;

    BlackBoxRing ring;
    SemaphoreHandle_t lock = nullptr;
    TaskHandle_t writerHandle = nullptr;

    volatile State state = State::Off;
    volatile bool armedFlag = false;
    volatile uint16_t flashMaxUs = 0;

    // --- сторона полётного цикла ---
    uint32_t nowUs = 0;
    uint32_t nowMs = 0;
    uint32_t cycle = 0;
    esp_reset_reason_t resetReason = ESP_RST_UNKNOWN;
    bool resetStartPending = false;
    bool resetRecording = false;
    bool manualStartRequested = false;
    bool manualStopRequested = false;
    bool manualRecording = false;
    uint16_t recordingFlight = 0;
    uint32_t recordingStartMs = 0;
    const char* startReason = "";
    uint32_t disarmedSinceMs = 0;
    uint32_t stillSinceMs = 0;
    uint32_t lastBaroTs = 0, lastMagTs = 0, lastGpsTs = 0, lastAirTs = 0;
    uint32_t lastMagUs = 0, lastAirUs = 0;
    uint32_t reportedDropped = 0;
    uint32_t dropReportMs = 0;

    // Скорость потока записей — для оценки "сколько минут влезет".
    uint32_t bytesThisSecond = 0;
    uint32_t bytesPerSecond = 0;
    uint32_t rateWindowMs = 0;

    struct Seen
    {
        bool valid = false;
        bool armed = false;
        const char* mode = nullptr;
        const char* refusal = nullptr;
        uint8_t link = 0;
        uint8_t sensors = 0;   // биты: IMU BARO MAG GPS AIR
        uint8_t gpsFix = 0;
        bool home = false;
        bool fence = false;
        bool stall = false;
        uint16_t features = 0;
        uint8_t launch = 0;
        uint8_t soaring = 0;
    } seen;

    // --- сторона записи ---
    uint8_t header[HEADER_BUFFER] = {};
    size_t headerLength = 0;
    size_t headerPosition = 0;
    uint8_t carry[BlackBoxFormat::MAX_RECORD] = {};
    size_t carryLength = 0;
    bool fullReported = false;
    uint32_t lastEraseMs = 0;
    bool erasing = false;
    uint32_t eraseOpsAtStart = 0;


    // ========================================================
    // Старт и остановка
    // ========================================================

    static bool isCrashReset(esp_reset_reason_t r)
    {
        return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT ||
               r == ESP_RST_BROWNOUT;
    }

    static const char* resetName(esp_reset_reason_t r)
    {
        switch (r)
        {
            case ESP_RST_POWERON:  return "POWERON";
            case ESP_RST_EXT:      return "EXT";
            case ESP_RST_SW:       return "SW";
            case ESP_RST_PANIC:    return "PANIC";
            case ESP_RST_INT_WDT:  return "INT_WDT";
            case ESP_RST_TASK_WDT: return "TASK_WDT";
            case ESP_RST_WDT:      return "WDT";
            case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
            case ESP_RST_BROWNOUT: return "BROWNOUT";
            case ESP_RST_SDIO:     return "SDIO";
            default:               return "UNKNOWN";
        }
    }

    bool motorOn() const
    {
        return controller.getOutputState().throttle > Config::THROTTLE_LOW_US ||
               controller.getRcState().get(Channels::THROTTLE) > Config::THROTTLE_LOW_US;
    }

    void decideStartStop(bool armed)
    {
        if (state == State::Idle)
        {
            if (manualStartRequested) start("вручную", true, false);
            else if (resetStartPending) start("перезагрузка после сбоя", false, true);
            else if (armed && motorOn()) start("ARM и газ", false, false);
            manualStartRequested = false;
            manualStopRequested = false;
            resetStartPending = false;
            return;
        }
        if (state != State::Recording) return;

        if (manualStopRequested)
        {
            manualStopRequested = false;
            stop("вручную");
            return;
        }
        if (armed)
        {
            manualRecording = false;
            resetRecording = false;
        }
        if (manualRecording) return;

        disarmedSinceMs = armed ? 0 : (disarmedSinceMs ? disarmedSinceMs : nowMs);
        const bool holdAfterReset = resetRecording && nowMs - recordingStartMs < Config::BLACKBOX_RESET_HOLD_MS;
        if (disarmedSinceMs && nowMs - disarmedSinceMs >= Config::BLACKBOX_POSTROLL_MS && !holdAfterReset)
        {
            stop("DISARM");
            return;
        }

        stillSinceMs = (armed && stillOnGround()) ? (stillSinceMs ? stillSinceMs : nowMs) : 0;
        if (stillSinceMs && nowMs - stillSinceMs >= Config::BLACKBOX_LANDED_STOP_MS)
        {
            stop("стоит на земле, мотор выключен");
        }
    }

    void start(const char* reason, bool manual, bool afterReset)
    {
        startReason = reason;
        manualRecording = manual;
        resetRecording = afterReset;
        recordingStartMs = nowMs;
        disarmedSinceMs = 0;
        stillSinceMs = 0;
        recordingFlight = storage.peekNextFlight();
        state = State::Recording;

        event("запись: старт (%s)", reason);
        snapshotEvent();
        Serial.printf("BlackBox: запись полёта #%u — %s\n", recordingFlight, reason);
    }

    // Последняя запись полёта — END: задача записи закроет полёт на ней.
    // Состояние — ДО записи END: иначе задача могла бы закрыть полёт
    // (Idle) раньше, чем цикл поставит Stopping поверх.
    void stop(const char* reason)
    {
        state = State::Stopping;
        pushText(BlackBoxFormat::REC_END, "%s", reason);
        Serial.printf("BlackBox: стоп записи полёта #%u — %s\n", recordingFlight, reason);
    }

    // Мотор стоит и самолёт неподвижен (см. Config::BLACKBOX_LANDED_*).
    bool stillOnGround() const
    {
        if (controller.getOutputState().throttle > Config::THROTTLE_LOW_US) return false;

        const ImuSensor* imu = autopilot.getImuSensor();
        if (!imu || !imu->isAvailable()) return false;
        const ImuData& d = imu->getImuData();
        const float gyroLimit = Config::BLACKBOX_LANDED_GYRO_DPS;
        if (fabsf(d.gyroX) > gyroLimit || fabsf(d.gyroY) > gyroLimit || fabsf(d.gyroZ) > gyroLimit) return false;
        const float g = sqrtf(d.accelX * d.accelX + d.accelY * d.accelY + d.accelZ * d.accelZ);
        if (fabsf(g - 1.0f) > Config::BLACKBOX_LANDED_ACCEL_G) return false;

        const BarometerSensor* baro = autopilot.getBarometerSensor();
        if (baro && baro->isAvailable() && fabsf(baro->getBarometerData().verticalSpeed) > Config::BLACKBOX_LANDED_CLIMB_MS)
        {
            return false;
        }
        const GpsSensor* gps = autopilot.getGpsSensor();
        if (gps && gps->isAvailable() && gps->hasFix() && gps->getGpsData().groundSpeed > Config::BLACKBOX_LANDED_SPEED_MS)
        {
            return false;
        }
        const AirspeedSensor* air = autopilot.getAirspeedSensor();
        if (air && air->isAvailable() && air->getAirspeedData().indicatedMs > Config::BLACKBOX_LANDED_SPEED_MS * 2)
        {
            return false;
        }
        return true;
    }


    // ========================================================
    // События
    // ========================================================

    uint8_t sensorBits() const
    {
        uint8_t bits = 0;
        if (autopilot.getImuSensor() && autopilot.getImuSensor()->isAvailable()) bits |= 1;
        if (autopilot.getBarometerSensor() && autopilot.getBarometerSensor()->isAvailable()) bits |= 2;
        if (autopilot.getMagnetometerSensor() && autopilot.getMagnetometerSensor()->isAvailable()) bits |= 4;
        if (autopilot.getGpsSensor() && autopilot.getGpsSensor()->isAvailable()) bits |= 8;
        if (autopilot.getAirspeedSensor() && autopilot.getAirspeedSensor()->isAvailable()) bits |= 16;
        return bits;
    }

    static uint16_t featureBits(const PilotInputs& in)
    {
        uint16_t bits = 0;
        for (uint8_t f = 0; f < static_cast<uint8_t>(Feature::COUNT); ++f)
        {
            if (in.features[f]) bits |= static_cast<uint16_t>(1u << f);
        }
        return bits;
    }

    uint8_t linkState() const
    {
        const IBusReceiver& rx = controller.getReceiver();
        return rx.isFrameTimeout() ? 1 : (rx.isFailsafeReported() ? 2 : 0);
    }

    void detectEvents(bool armed)
    {
        const bool first = !seen.valid;
        seen.valid = true;

        if (first || armed != seen.armed)
        {
            if (!first || armed) event(armed ? "ARM" : "DISARM");
            seen.armed = armed;
        }

        const char* refusal = controller.getArming().getLastRefusalReason();
        if (refusal != seen.refusal)
        {
            if (refusal) event("ARM отклонён: %s", refusal);
            seen.refusal = refusal;
        }

        const char* mode = autopilot.getModeName();
        if (!seen.mode || strcmp(mode, seen.mode) != 0)
        {
            event("режим %s", mode);
            seen.mode = mode;
        }

        const uint8_t link = linkState();
        if (first || link != seen.link)
        {
            static const char* const LINK[] = { "связь: есть", "связь: потеряна — нет кадров iBUS",
                                                 "связь: потеряна — failsafe пульта" };
            event("%s", LINK[link]);
            seen.link = link;
        }

        const uint8_t sensors = sensorBits();
        if (first || sensors != seen.sensors)
        {
            static const char* const NAMES[] = { "IMU", "BARO", "MAG", "GPS", "AIR" };
            char text[80] = "датчики:";
            for (uint8_t i = 0; i < 5; ++i)
            {
                const bool now = sensors & (1u << i);
                if (!first && now == static_cast<bool>(seen.sensors & (1u << i))) continue;
                strncat(text, now ? " +" : " -", sizeof(text) - strlen(text) - 1);
                strncat(text, NAMES[i], sizeof(text) - strlen(text) - 1);
            }
            event("%s", text);
            seen.sensors = sensors;
        }

        const GpsSensor* gps = autopilot.getGpsSensor();
        if (gps && gps->isAvailable() && gps->getGpsData().fixType != seen.gpsFix)
        {
            seen.gpsFix = gps->getGpsData().fixType;
            event("GPS: фикс %u, спутников %u", seen.gpsFix, gps->getGpsData().numSatellites);
        }

        const NavStatus& nav = autopilot.getNavStatus();
        if (nav.homeValid != seen.home)
        {
            if (nav.homeValid) event("дом: %.7f %.7f", nav.home.lat, nav.home.lon);
            else event("дом: сброшен");
            seen.home = nav.homeValid;
        }
        if (nav.fenceBreached != seen.fence)
        {
            event(nav.fenceBreached ? "геозабор: нарушен" : "геозабор: в пределах");
            seen.fence = nav.fenceBreached;
        }
        if (nav.stallWarning != seen.stall)
        {
            if (nav.stallWarning) event("малая скорость: защита от сваливания");
            seen.stall = nav.stallWarning;
        }

        const uint16_t features = featureBits(autopilot.getInputs());
        if (features != seen.features)
        {
            for (uint8_t f = 0; f < static_cast<uint8_t>(Feature::COUNT); ++f)
            {
                const uint16_t bit = static_cast<uint16_t>(1u << f);
                if ((features ^ seen.features) & bit)
                {
                    event("%c%s", (features & bit) ? '+' : '-', AutopilotNames::feature(static_cast<Feature>(f)));
                }
            }
            seen.features = features;
        }

        const uint8_t launch = static_cast<uint8_t>(autopilot.getLaunchState());
        if (launch != seen.launch)
        {
            event("запуск с руки: стадия %u", launch);
            seen.launch = launch;
        }
        const uint8_t soaring = static_cast<uint8_t>(autopilot.getSoaringState());
        if (soaring != seen.soaring)
        {
            event("парение: стадия %u", soaring);
            seen.soaring = soaring;
        }

        if (ring.dropped() != reportedDropped && nowMs - dropReportMs >= 1000)
        {
            event("очередь переполнена: потеряно записей %lu", (unsigned long)(ring.dropped() - reportedDropped));
            reportedDropped = ring.dropped();
            dropReportMs = nowMs;
        }
    }

    void snapshotEvent()
    {
        const uint8_t s = sensorBits();
        event("состояние: ARM=%s MODE=%s RX=%s IMU=%s BARO=%s MAG=%s GPS=%s AIR=%s",
              controller.isArmed() ? "YES" : "NO", autopilot.getModeName(), linkState() ? "LOST" : "OK",
              (s & 1) ? "OK" : "-", (s & 2) ? "OK" : "-", (s & 4) ? "OK" : "-", (s & 8) ? "OK" : "-",
              (s & 16) ? "OK" : "-");
    }

    void event(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        pushTextV(BlackBoxFormat::REC_EVENT, format, args);
        va_end(args);
    }

    void pushText(uint8_t type, const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        pushTextV(type, format, args);
        va_end(args);
    }

    void pushTextV(uint8_t type, const char* format, va_list args)
    {
        uint8_t record[BlackBoxFormat::MAX_RECORD];
        const size_t length = formatText(record, type, nowUs, format, args);
        push(record, length);
    }

    // [тип][длина][t_us][текст] — текст обрезается, чтобы влезть в запись.
    static size_t formatText(uint8_t* record, uint8_t type, uint32_t tUs, const char* format, va_list args)
    {
        constexpr size_t TEXT_AT = BlackBoxFormat::RECORD_HEADER + 4;
        char* text = reinterpret_cast<char*>(record + TEXT_AT);
        const int n = vsnprintf(text, BlackBoxFormat::MAX_RECORD - TEXT_AT, format, args);
        const size_t textLength = n < 0 ? 0 : min(static_cast<size_t>(n), BlackBoxFormat::MAX_PAYLOAD - 4);
        record[0] = type;
        record[1] = static_cast<uint8_t>(4 + textLength);
        memcpy(record + 2, &tUs, 4);
        return TEXT_AT + textLength;
    }


    // ========================================================
    // Снимки
    // ========================================================

    template <typename T>
    void pushRecord(uint8_t type, const T& data)
    {
        uint8_t record[BlackBoxFormat::RECORD_HEADER + sizeof(T)];
        record[0] = type;
        record[1] = static_cast<uint8_t>(sizeof(T));
        memcpy(record + BlackBoxFormat::RECORD_HEADER, &data, sizeof(T));
        push(record, sizeof(record));
    }

    void push(const uint8_t* record, size_t length)
    {
        ring.push(record, length);
        bytesThisSecond += static_cast<uint32_t>(length);
    }

    static int16_t i16(float value)
    {
        return static_cast<int16_t>(constrain(lroundf(value), -32768L, 32767L));
    }

    static uint16_t u16(float value)
    {
        return static_cast<uint16_t>(constrain(lroundf(value), 0L, 65535L));
    }

    static uint16_t angle100(float degrees)
    {
        float a = fmodf(degrees, 360.0f);
        if (a < 0) a += 360.0f;
        return u16(a * 100.0f);
    }

    void sample(uint32_t workUs)
    {
        if (nowMs - rateWindowMs >= 1000)
        {
            bytesPerSecond = bytesThisSecond;
            bytesThisSecond = 0;
            rateWindowMs = nowMs;
        }

        // Делитель — настройка Config, сейчас 1.
        // cppcheck-suppress knownConditionTrueFalse
        // cppcheck-suppress moduloofone
        if (cycle % Config::BLACKBOX_IMU_DIVIDER == 0) sampleImu(workUs);
        if (cycle % CTRL_EVERY == 0) sampleCtrl();
        if (cycle % RC_EVERY == 0) sampleRc();
        sampleBaro();
        sampleMag();
        sampleGps();
        sampleAir();
        if (cycle % NAV_EVERY == 0) sampleNav();
        if (cycle % NAV_EVERY == 1) samplePower();
        if (cycle % SYS_EVERY == 0) sampleSys();
    }

    void sampleImu(uint32_t workUs)
    {
        const ImuSensor* imu = autopilot.getImuSensor();
        if (!imu) return;
        const ImuData& d = imu->getImuData();
        BlackBoxFormat::ImuRecord r;
        r.tUs = nowUs;
        r.gyro[0] = i16(d.gyroX * 10.0f);
        r.gyro[1] = i16(d.gyroY * 10.0f);
        r.gyro[2] = i16(d.gyroZ * 10.0f);
        r.accel[0] = i16(d.accelX * 1000.0f);
        r.accel[1] = i16(d.accelY * 1000.0f);
        r.accel[2] = i16(d.accelZ * 1000.0f);
        r.workUs = static_cast<uint16_t>(min(workUs, static_cast<uint32_t>(65535)));
        pushRecord(BlackBoxFormat::REC_IMU, r);
    }

    uint16_t flags(bool armed) const
    {
        namespace F = BlackBoxFormat::Flag;
        const IBusReceiver& rx = controller.getReceiver();
        const NavStatus& nav = autopilot.getNavStatus();
        const uint8_t sensors = sensorBits();
        const GpsSensor* gps = autopilot.getGpsSensor();

        uint16_t f = 0;
        if (armed) f |= F::ARMED;
        if (rx.isSignalLost()) f |= F::RX_LOST;
        if (rx.isFrameTimeout()) f |= F::RX_TIMEOUT;
        if (rx.isFailsafeReported()) f |= F::RX_FAILSAFE;
        if (autopilot.isFailsafeActive()) f |= F::FS_ACTIVE;
        if (autopilot.isFailsafeGliding()) f |= F::FS_GLIDE;
        if (autopilot.isFailsafeReturning()) f |= F::FS_RTH;
        if (autopilot.isAutoThrottle()) f |= F::AUTO_THR;
        if (nav.stallWarning) f |= F::STALL;
        if (nav.fenceBreached) f |= F::FENCE;
        if (sensors & 1) f |= F::IMU_OK;
        if (sensors & 2) f |= F::BARO_OK;
        if (sensors & 4) f |= F::MAG_OK;
        if (sensors & 8) f |= F::GPS_OK;
        if (gps && gps->isAvailable() && gps->hasFix()) f |= F::GPS_FIX;
        if (sensors & 16) f |= F::AIR_OK;
        return f;
    }

    void sampleCtrl()
    {
        BlackBoxFormat::CtrlRecord r;
        r.tUs = nowUs;

        const ImuSensor* imu = autopilot.getImuSensor();
        const ImuData* d = imu ? &imu->getImuData() : nullptr;
        r.roll = d ? i16(d->roll * 100.0f) : 0;
        r.pitch = d ? i16(d->pitch * 100.0f) : 0;
        r.yaw = d ? i16(d->yaw * 100.0f) : 0;
        r.wantRoll = i16(autopilot.getDesiredRoll() * 100.0f);
        r.wantPitch = i16(autopilot.getDesiredPitch() * 100.0f);

        const ControlCommand& pilot = autopilot.getPilotCommand();
        const ControlCommand command = autopilot.getCommand();
        r.stickRoll = pilot.roll;
        r.stickPitch = pilot.pitch;
        r.stickYaw = pilot.yaw;
        r.cmdRoll = command.roll;
        r.cmdPitch = command.pitch;
        r.cmdYaw = command.yaw;
        r.flaps = controller.getFlapsUs();
        r.thrPilot = controller.getRcState().get(Channels::THROTTLE);
        r.thrAuto = i16(autopilot.getThrottleCorrection() * 10.0f);

        const FlightOutputState& out = controller.getOutputState();
        r.out[0] = out.aileronLeft;
        r.out[1] = out.aileronRight;
        r.out[2] = out.elevator;
        r.out[3] = out.rudder;
        r.out[4] = out.throttle;
        r.out[5] = out.aux1;
        r.out[6] = out.aux2;

        const PidController& pr = autopilot.getRollPid();
        const PidController& pp = autopilot.getPitchPid();
        r.pidRoll[0] = i16(pr.getLastP() * 10.0f);
        r.pidRoll[1] = i16(pr.getLastI() * 10.0f);
        r.pidRoll[2] = i16(pr.getLastD() * 10.0f);
        r.pidPitch[0] = i16(pp.getLastP() * 10.0f);
        r.pidPitch[1] = i16(pp.getLastI() * 10.0f);
        r.pidPitch[2] = i16(pp.getLastD() * 10.0f);

        r.mode = static_cast<uint8_t>(autopilot.getMode());
        r.flags = flags(armedFlag);
        r.features = featureBits(autopilot.getInputs());
        pushRecord(BlackBoxFormat::REC_CTRL, r);
    }

    void sampleRc()
    {
        const RcChannelState& rc = controller.getRcState();
        const IBusReceiver& rx = controller.getReceiver();
        BlackBoxFormat::RcRecord r = {};
        static_assert(Config::IBUS_CHANNELS <= sizeof(r.ch) / sizeof(r.ch[0]), "RC: каналов больше, чем в записи");
        r.tUs = nowUs;
        for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i) r.ch[i] = rc.get(i);
        r.status = static_cast<uint8_t>((rx.isFrameTimeout() ? 1 : 0) | (rx.isFailsafeReported() ? 2 : 0));
        r.good = static_cast<uint16_t>(rx.getGoodFrameCount());
        r.bad = static_cast<uint16_t>(rx.getBadFrameCount());
        pushRecord(BlackBoxFormat::REC_RC, r);
    }

    void sampleBaro()
    {
        const BarometerSensor* baro = autopilot.getBarometerSensor();
        if (!baro || !baro->isAvailable()) return;
        const BarometerData& d = baro->getBarometerData();
        if (d.timestamp == lastBaroTs) return;
        lastBaroTs = d.timestamp;

        BlackBoxFormat::BaroRecord r;
        r.tUs = nowUs;
        r.pressurePa = d.pressure;
        r.tempC = i16(d.temperature * 100.0f);
        r.altM = d.altitude;
        r.vzCms = i16(d.verticalSpeed * 100.0f);
        r.targetDm = i16(autopilot.getTargetAltitude() * 10.0f);
        pushRecord(BlackBoxFormat::REC_BARO, r);
    }

    void sampleMag()
    {
        const MagnetometerSensor* mag = autopilot.getMagnetometerSensor();
        if (!mag || !mag->isAvailable()) return;
        const MagData& d = mag->getMagData();
        if (d.timestamp == lastMagTs || nowUs - lastMagUs < MAG_MIN_US) return;
        lastMagTs = d.timestamp;
        lastMagUs = nowUs;

        BlackBoxFormat::MagRecord r;
        r.tUs = nowUs;
        r.mag[0] = i16(d.magX * 10.0f);
        r.mag[1] = i16(d.magY * 10.0f);
        r.mag[2] = i16(d.magZ * 10.0f);
        r.heading = angle100(d.headingDegrees);
        pushRecord(BlackBoxFormat::REC_MAG, r);
    }

    void sampleGps()
    {
        const GpsSensor* gps = autopilot.getGpsSensor();
        if (!gps || !gps->isAvailable()) return;
        const GpsData& d = gps->getGpsData();
        if (d.timestamp == lastGpsTs) return;
        lastGpsTs = d.timestamp;

        BlackBoxFormat::GpsRecord r;
        r.tUs = nowUs;
        r.lat = static_cast<int32_t>(llround(d.latitude * 1e7));
        r.lon = static_cast<int32_t>(llround(d.longitude * 1e7));
        r.altMm = static_cast<int32_t>(lroundf(d.altitude * 1000.0f));
        r.speedCms = i16(d.groundSpeed * 100.0f);
        r.course = angle100(d.heading);
        r.sats = d.numSatellites;
        r.fix = d.fixType;
        r.haccCm = u16(d.horizontalAccuracy * 100.0f);
        r.vaccCm = u16(d.verticalAccuracy * 100.0f);
        pushRecord(BlackBoxFormat::REC_GPS, r);
    }

    void sampleAir()
    {
        const AirspeedSensor* air = autopilot.getAirspeedSensor();
        if (!air || !air->isAvailable()) return;
        const AirspeedData& d = air->getAirspeedData();
        if (d.timestamp == lastAirTs || nowUs - lastAirUs < AIR_MIN_US) return;
        lastAirTs = d.timestamp;
        lastAirUs = nowUs;

        BlackBoxFormat::AirRecord r;
        r.tUs = nowUs;
        r.dpPa = d.differentialPressurePa;
        r.iasCms = i16(d.indicatedMs * 100.0f);
        r.tasCms = i16(d.trueMs * 100.0f);
        r.rho = u16(d.airDensity * 10000.0f);
        pushRecord(BlackBoxFormat::REC_AIR, r);
    }

    void sampleNav()
    {
        const NavStatus& nav = autopilot.getNavStatus();
        BlackBoxFormat::NavRecord r;
        r.tUs = nowUs;
        r.flags = static_cast<uint8_t>((nav.gpsGood ? 1 : 0) | (nav.homeValid ? 2 : 0) | (nav.fenceBreached ? 4 : 0) |
                                       (nav.stallWarning ? 8 : 0));
        r.homeDistM = nav.distanceHomeM;
        r.homeBearing = angle100(nav.bearingHomeDeg);
        r.course = angle100(nav.courseDeg);
        r.targetCourse = angle100(nav.targetCourseDeg);
        r.speedCms = i16(nav.speedMs * 100.0f);
        r.courseSource = static_cast<uint8_t>(autopilot.getCourseSource());
        r.launchState = static_cast<uint8_t>(autopilot.getLaunchState());
        r.soaringState = static_cast<uint8_t>(autopilot.getSoaringState());
        r.trimRoll = i16(autopilot.getAutoTrim().getRoll() * 10.0f);
        r.trimPitch = i16(autopilot.getAutoTrim().getPitch() * 10.0f);
        pushRecord(BlackBoxFormat::REC_NAV, r);
    }

    void samplePower()
    {
#if defined(BOARD_ESP32_S3)
        BlackBoxFormat::PowerRecord r;
        r.tUs = nowUs;
        r.vbatMv = u16(analogReadMilliVolts(Config::PIN_VBAT_ADC) * Config::BLACKBOX_VBAT_DIVIDER);
        r.currentMv = u16(analogReadMilliVolts(Config::PIN_CURRENT_ADC) * Config::BLACKBOX_CURRENT_DIVIDER);
        pushRecord(BlackBoxFormat::REC_POWER, r);
#endif
    }

    void sampleSys()
    {
        const IBusReceiver& rx = controller.getReceiver();
        const ImuSensor* imu = autopilot.getImuSensor();
        BlackBoxFormat::SysRecord r;
        r.tUs = nowUs;
        const uint32_t hz = loopStats.hz;
        const uint32_t avgUs = loopStats.avgUs;
        const uint32_t maxUs = loopStats.maxUs;
        r.loopHz = static_cast<uint16_t>(min(hz, static_cast<uint32_t>(65535)));
        r.loopAvgUs = static_cast<uint16_t>(min(avgUs, static_cast<uint32_t>(65535)));
        r.loopMaxUs = static_cast<uint16_t>(min(maxUs, static_cast<uint32_t>(65535)));
        r.heapFree = Rtos::freeHeapBytes();
        r.ibusGood = rx.getGoodFrameCount();
        r.ibusBad = rx.getBadFrameCount();
        r.imuTempC = imu ? i16(imu->getImuData().temperature * 100.0f) : 0;
        r.ringUsed = ring.used();
        r.dropped = ring.dropped();
        r.flashMaxUs = flashMaxUs;
        flashMaxUs = 0;
        r.freeKb = static_cast<uint16_t>(min(storage.freeBytes() / 1024, static_cast<uint32_t>(65535)));
        pushRecord(BlackBoxFormat::REC_SYS, r);
    }


    // ========================================================
    // Задача записи (ядро 0)
    // ========================================================

    static void writerTask(void* arg)
    {
        BlackBox* self = static_cast<BlackBox*>(arg);
        for (;;)
        {
            // Будит полётный цикл после каждого такта; в покое — сам,
            // чтобы стирать на земле.
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
            self->writerStep();
        }
    }

    bool takeLock(TickType_t wait) { return lock && xSemaphoreTake(lock, wait) == pdTRUE; }
    void giveLock() { if (lock) xSemaphoreGive(lock); }

    // Одна-две страницы за шаг: запись флеша останавливает оба ядра,
    // и шаг должен уложиться в паузу между тактами цикла.
    void writeSome()
    {
        const uint32_t ms = millis();
        if (!storage.isFlightOpen())
        {
            buildHeader(storage.openFlight(), ms);
        }

        int writes = 0;
        while (writes == 0)
        {
            if (!carryLength && !nextRecord()) return;

            const uint32_t t0 = micros();
            const int result = storage.append(carry, carryLength, ms);
            if (result < 0)
            {
                onFull();
                return;
            }
            if (result > 0)
            {
                const uint32_t spent = micros() - t0;
                if (spent > flashMaxUs) flashMaxUs = static_cast<uint16_t>(min(spent, static_cast<uint32_t>(65535)));
            }
            writes += result;

            const bool end = carry[0] == BlackBoxFormat::REC_END;
            carryLength = 0;
            if (end)
            {
                finishFlight();
                return;
            }
        }
    }

    // Следующая запись: сначала заголовок полёта, потом очередь.
    bool nextRecord()
    {
        if (headerPosition < headerLength)
        {
            carryLength = BlackBoxFormat::RECORD_HEADER + header[headerPosition + 1];
            memcpy(carry, header + headerPosition, carryLength);
            headerPosition += carryLength;
            return true;
        }
        carryLength = ring.pop(carry);
        return carryLength > 0;
    }

    // Стёртого места не осталось. В воздухе — ждать (очередь держит
    // последние минуты); на земле без ARM — стереть старый полёт.
    void onFull()
    {
        if (!fullReported)
        {
            fullReported = true;
            uint8_t record[BlackBoxFormat::MAX_RECORD];
            const size_t length = textRecord(record, BlackBoxFormat::REC_EVENT, "флеш: стёртое место кончилось, пишу в очередь");
            ring.push(record, length);
        }
        if (millis() - lastEraseMs < Config::BLACKBOX_ERASE_PAUSE_MS) return;
        lastEraseMs = millis();

        // В воздухе — только сверить уже стёртое (чтение), не стирать.
        if (storage.eraseStep(storage.freeSectors() + 1, storage.currentFlight(), !armedFlag) || armedFlag) return;

        // На земле стирать больше нечего: впереди — сам этот полёт, он
        // длиннее всего раздела. Закрыть его; хвост в очереди не влез.
        if (state == State::Stopping)
        {
            Serial.printf("BlackBox: полёт #%u длиннее раздела — конец не поместился\n", storage.currentFlight());
            ring.clear();
            carryLength = 0;
            finishFlight();
        }
    }

    void finishFlight()
    {
        storage.closeFlight();
        headerLength = headerPosition = 0;
        fullReported = false;
        state = State::Idle;

        if (const BlackBoxStorage::Flight* f = storage.newestFlight())
        {
            const uint32_t seconds = (f->lastMs - f->startMs) / 1000;
            Serial.printf("BlackBox: полёт #%u записан — %lu КБ, ~%lu:%02lu\n", f->number,
                          (unsigned long)(f->sectors * BlackBoxFormat::SECTOR_SIZE / 1024),
                          (unsigned long)(seconds / 60), (unsigned long)(seconds % 60));
        }
    }

    // В покое на земле: держать стёртое место наготове.
    void maintain()
    {
        const uint32_t ms = millis();
        if (armedFlag || ms - lastEraseMs < Config::BLACKBOX_ERASE_PAUSE_MS) return;

        const BlackBoxStorage::Flight* newest = storage.newestFlight();
        const uint32_t target = Config::BLACKBOX_MIN_FREE_BYTES / BlackBoxFormat::SECTOR_PAYLOAD + 1;
        const uint32_t opsBefore = storage.eraseOps;
        const bool worked = storage.eraseStep(target, newest ? newest->number : 0);

        if (worked && !erasing)
        {
            erasing = true;
            eraseOpsAtStart = opsBefore;
        }
        if (storage.eraseOps != opsBefore) lastEraseMs = millis();
        if (!worked && erasing)
        {
            erasing = false;
            if (storage.eraseOps != eraseOpsAtStart) printStatus(Serial);
        }
    }

    static size_t textRecord(uint8_t* record, uint8_t type, const char* text)
    {
        const size_t n = min(strlen(text), BlackBoxFormat::MAX_PAYLOAD - 4);
        const uint32_t t = micros();
        record[0] = type;
        record[1] = static_cast<uint8_t>(4 + n);
        memcpy(record + 2, &t, 4);
        memcpy(record + 6, text, n);
        return 6 + n;
    }

    // --- заголовок полёта: схема и параметры ---

    // Print в записи INFO "bind=<строка>" — для привязок тумблеров.
    class InfoLines : public Print
    {
    public:
        explicit InfoLines(BlackBox& owner, const char* key) : box(owner), prefix(key) {}

        size_t write(uint8_t c) override
        {
            if (c == '\n')
            {
                line[length] = '\0';
                box.info("%s=%s", prefix, line);
                length = 0;
            }
            else if (c != '\r' && length + 1 < sizeof(line))
            {
                line[length++] = static_cast<char>(c);
            }
            return 1;
        }

    private:
        BlackBox& box;
        const char* prefix;
        char line[160] = {};
        size_t length = 0;
    };

    void appendHeader(const uint8_t* record, size_t length)
    {
        if (headerLength + length > sizeof(header)) return;
        memcpy(header + headerLength, record, length);
        headerLength += length;
    }

    void info(const char* format, ...)
    {
        uint8_t record[BlackBoxFormat::MAX_RECORD];
        va_list args;
        va_start(args, format);
        const size_t length = formatText(record, BlackBoxFormat::REC_INFO, micros(), format, args);
        va_end(args);
        appendHeader(record, length);
    }

    // Схема типа: "<id> <имя> <поля>", длинная — кусками "<id> + ...".
    void schema(const BlackBoxFormat::Schema& s)
    {
        constexpr size_t CHUNK = 200;
        const char* fields = s.fields;
        bool first = true;
        while (*fields)
        {
            size_t n = strlen(fields);
            if (n > CHUNK)
            {
                n = CHUNK;
                while (n > 0 && fields[n] != ' ') n--;
            }
            uint8_t record[BlackBoxFormat::MAX_RECORD];
            char text[BlackBoxFormat::MAX_PAYLOAD];
            snprintf(text, sizeof(text), "%u %s %.*s", static_cast<unsigned>(s.id), first ? s.name : "+",
                     static_cast<int>(n), fields);
            const size_t length = textRecord(record, BlackBoxFormat::REC_SCHEMA, text);
            appendHeader(record, length);
            fields += n;
            while (*fields == ' ') fields++;
            first = false;
        }
    }

    static const char* sensorName(const Sensor* sensor) { return sensor ? sensor->getSensorType() : "нет"; }

    void buildHeader(uint16_t flight, uint32_t ms)
    {
        headerLength = headerPosition = 0;
        for (const BlackBoxFormat::Schema& s : BlackBoxFormat::SCHEMAS) schema(s);

        info("format=%u", BlackBoxFormat::VERSION);
        info("flight=%u", flight);
        info("firmware=%s %s", __DATE__, __TIME__);
        info("board=%s", boardName());
        info("start=%s", startReason);
        info("reset_reason=%s", resetName(resetReason));
        info("uptime_ms=%lu", (unsigned long)ms);
        info("loop_period_ms=%lu", (unsigned long)Config::LOOP_PERIOD_MS);
        info("rate.imu_hz=%lu", (unsigned long)(1000 / Config::LOOP_PERIOD_MS / Config::BLACKBOX_IMU_DIVIDER));
        info("rate.ctrl_hz=%lu", (unsigned long)(1000 / (Config::LOOP_PERIOD_MS * CTRL_EVERY)));
        info("bits.flags=%s", BlackBoxFormat::Flag::NAMES);

        char names[200] = "";
        for (uint8_t m = 0; m < MODE_COUNT; ++m)
        {
            if (m) strncat(names, " ", sizeof(names) - strlen(names) - 1);
            strncat(names, AutopilotNames::mode(static_cast<AutopilotMode>(m)), sizeof(names) - strlen(names) - 1);
        }
        info("names.mode=%s", names);
        names[0] = '\0';
        for (uint8_t f = 0; f < static_cast<uint8_t>(Feature::COUNT); ++f)
        {
            if (f) strncat(names, " ", sizeof(names) - strlen(names) - 1);
            strncat(names, AutopilotNames::feature(static_cast<Feature>(f)), sizeof(names) - strlen(names) - 1);
        }
        info("bits.features=%s", names);
        info("names.course_source=NONE GYRO COMPASS GPS");

        const ImuSensor* imu = autopilot.getImuSensor();
        info("sensor.imu=%s", sensorName(imu));
        info("sensor.baro=%s", sensorName(autopilot.getBarometerSensor()));
        info("sensor.mag=%s", sensorName(autopilot.getMagnetometerSensor()));
        info("sensor.gps=%s", sensorName(autopilot.getGpsSensor()));
        info("sensor.airspeed=%s", sensorName(autopilot.getAirspeedSensor()));
        if (imu) info("imu.preflight=%s", imu->getPreflightProblem() ? imu->getPreflightProblem() : "OK");

        const PidController& pr = autopilot.getRollPid();
        const PidController& pp = autopilot.getPitchPid();
        info("pid.roll=%.4f %.4f %.4f", pr.getKp(), pr.getKi(), pr.getKd());
        info("pid.pitch=%.4f %.4f %.4f", pp.getKp(), pp.getKi(), pp.getKd());
        info("trim.roll_us=%.1f", autopilot.getAutoTrim().getRoll());
        info("trim.pitch_us=%.1f", autopilot.getAutoTrim().getPitch());

        infoConfig();
#if defined(BOARD_ESP32_S3)
        info("power.vbat_divider=%.4f", static_cast<double>(Config::BLACKBOX_VBAT_DIVIDER));
        info("power.current_divider=%.4f", static_cast<double>(Config::BLACKBOX_CURRENT_DIVIDER));
#endif

        if (switches)
        {
            InfoLines lines(*this, "bind");
            switches->printBindings(lines);
        }
        if (headerLength + 64 > sizeof(header)) info("header=обрезан");
    }

    static const char* boardName()
    {
#if defined(BOARD_ESP32_S3)
        return "ESP32-S3";
#elif defined(BOARD_ESP32_C3)
        return "ESP32-C3";
#elif defined(BOARD_ESP32_CLASSIC)
        return "ESP32";
#else
        return "?";
#endif
    }

    // Настройки, от которых зависит поведение в полёте.
    void infoConfig()
    {
#define BB_CFG_INT(name) info("cfg." #name "=%ld", static_cast<long>(Config::name))
#define BB_CFG_FLT(name) info("cfg." #name "=%.3f", static_cast<double>(Config::name))
        BB_CFG_INT(AILERON_MAX_US);
        BB_CFG_INT(ELEVATOR_MAX_US);
        BB_CFG_INT(RUDDER_MAX_US);
        BB_CFG_INT(AILERON_LEFT_REVERSED);
        BB_CFG_INT(AILERON_RIGHT_REVERSED);
        BB_CFG_INT(ELEVATOR_REVERSED);
        BB_CFG_INT(RUDDER_REVERSED);
        BB_CFG_INT(FLAPS_DEPLOYED_US);
        BB_CFG_INT(AIRBRAKE_US);
        BB_CFG_INT(IMU_ROTATION_CW_DEG);
        BB_CFG_INT(MAG_ROTATION_CW_DEG);
        BB_CFG_INT(THROTTLE_LOW_US);
        BB_CFG_INT(RX_FAILSAFE_THROTTLE_US);
        BB_CFG_FLT(MAX_BANK_DEG);
        BB_CFG_FLT(STAB_MAX_PITCH_DEG);
        BB_CFG_FLT(STAB_INTEGRATOR_ZONE_DEG);
        BB_CFG_FLT(ACRO_MAX_RATE_DPS);
        BB_CFG_FLT(ACRO_RATE_GAIN_US_PER_DPS);
        BB_CFG_FLT(FAILSAFE_GLIDE_ROLL_DEG);
        BB_CFG_FLT(FAILSAFE_GLIDE_PITCH_DEG);
        BB_CFG_INT(FAILSAFE_RTH);
        BB_CFG_FLT(RTH_ALTITUDE_M);
        BB_CFG_FLT(CRUISE_THROTTLE_PCT);
        BB_CFG_FLT(CRUISE_AIRSPEED_MS);
        BB_CFG_FLT(STALL_SPEED_MS);
        BB_CFG_FLT(LOITER_RADIUS_M);
        BB_CFG_FLT(NAV_ALT_GAIN);
        BB_CFG_FLT(NAV_CLIMB_KP_DEG);
        BB_CFG_FLT(NAV_CLIMB_KI_DEG);
        BB_CFG_FLT(TURN_COORD_RUDDER_MIX);
        BB_CFG_FLT(TURN_COORD_PITCH_US);
        BB_CFG_INT(GEOFENCE_ALWAYS_ON);
        BB_CFG_FLT(FENCE_RADIUS_M);
        BB_CFG_FLT(FENCE_ALTITUDE_M);
        BB_CFG_FLT(LAUNCH_ACCEL_G);
        BB_CFG_FLT(LAUNCH_CLIMB_PITCH_DEG);
        BB_CFG_FLT(LAND_GLIDE_PITCH_DEG);
        BB_CFG_FLT(AUTOTRIM_RATE);
        BB_CFG_FLT(PITOT_SCALE);
#undef BB_CFG_INT
#undef BB_CFG_FLT
    }


    // ========================================================
    // Выгрузка по UART
    // ========================================================

    void hostList()
    {
        takeLock(portMAX_DELAY);
        static const char* const STATES[] = { "off", "idle", "recording", "stopping" };
        Serial.printf("BB:STATE state=%s free_kb=%lu total_kb=%lu flights=%u rate_bps=%lu\n",
                      STATES[static_cast<uint8_t>(state)], (unsigned long)(storage.freeBytes() / 1024),
                      (unsigned long)(storage.totalSectors() * BlackBoxFormat::SECTOR_SIZE / 1024),
                      (unsigned)storage.flightCount(), (unsigned long)bytesPerSecond);
        for (size_t i = 0; i < storage.flightCount(); ++i)
        {
            const BlackBoxStorage::Flight& f = storage.flight(i);
            Serial.printf("BB:FLIGHT n=%u sectors=%lu kb=%lu seconds=%lu start=%u\n", f.number,
                          (unsigned long)f.sectors, (unsigned long)(f.sectors * BlackBoxFormat::SECTOR_SIZE / 1024),
                          (unsigned long)((f.lastMs - f.startMs) / 1000), f.hasStart ? 1u : 0u);
        }
        Serial.println("BB:END");
        giveLock();
    }

    static void setSerialBaud(unsigned long baud)
    {
#if ARDUINO_USB_CDC_ON_BOOT
        (void)baud;   // Serial — USB CDC (ESP32-C3): скорость ни на что не влияет
#else
        Serial.updateBaudRate(baud);
#endif
    }

    // Выгрузка полёта. Полётный цикл стоит (консоль вызывает из
    // loop()), поэтому только без ARM и без записи.
    //   ФК:  BB:SEND n=<n> sectors=<k> baud=<b>   (на текущей скорости)
    //        переключается на b и ждёт от ПК байт 'G' (3 с)
    //   ФК:  k кадров: A5 5A, u16 номер, 4096 байт сектора, u32 CRC-32
    //        возвращает скорость, BB:DONE n=<n> crc=<CRC всех секторов>
    void sendFlight(uint16_t number, unsigned long baud)
    {
        if (isRecording() || controller.isArmed())
        {
            Serial.println("BB:ERR занято: идёт запись или заармлено");
            return;
        }
        takeLock(portMAX_DELAY);
        const BlackBoxStorage::Flight* found = storage.findFlight(number);
        if (!found)
        {
            giveLock();
            Serial.printf("BB:ERR нет полёта %u\n", number);
            return;
        }
        const BlackBoxStorage::Flight f = *found;

        Serial.printf("BB:SEND n=%u sectors=%lu baud=%lu\n", f.number, (unsigned long)f.sectors, baud);
        Serial.flush();
        const unsigned long oldBaud = Serial.baudRate();
        if (baud != oldBaud) setSerialBaud(baud);

        bool go = false;
        for (const uint32_t waitStart = millis(); millis() - waitStart < 3000 && !go;)
        {
            while (Serial.available()) go = go || Serial.read() == 'G';
            if (!go) delay(1);
        }

        uint32_t crcAll = 0;
        if (go)
        {
            for (uint32_t k = 0; k < f.sectors; ++k)
            {
                uint8_t* data = header;   // в покое заголовок не нужен — буфер сектора
                if (!storage.readSector(storage.sectorOf(f, k), data)) memset(data, 0xFF, BlackBoxFormat::SECTOR_SIZE);
                const uint8_t frame[4] = { FRAME_MAGIC_0, FRAME_MAGIC_1, static_cast<uint8_t>(k & 0xFF),
                                           static_cast<uint8_t>(k >> 8) };
                const uint32_t crc = BlackBoxFormat::crc32(data, BlackBoxFormat::SECTOR_SIZE);
                crcAll = BlackBoxFormat::crc32(data, BlackBoxFormat::SECTOR_SIZE, crcAll);
                Serial.write(frame, sizeof(frame));
                Serial.write(data, BlackBoxFormat::SECTOR_SIZE);
                Serial.write(reinterpret_cast<const uint8_t*>(&crc), sizeof(crc));
            }
            Serial.flush();
        }
        // ПК дочитывает хвост через USB (задержка драйвера моста — до
        // десятков мс) и только потом сам возвращает скорость.
        delay(150);
        if (baud != oldBaud) setSerialBaud(oldBaud);
        delay(50);
        giveLock();

        if (go) Serial.printf("BB:DONE n=%u crc=%08lx\n", f.number, (unsigned long)crcAll);
        else Serial.println("BB:ERR ПК не ответил на новой скорости");
    }
};
