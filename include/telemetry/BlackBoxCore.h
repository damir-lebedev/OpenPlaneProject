#pragma once
#include <Arduino.h>
#include <stdarg.h>

#include "config/Config.h"
#include "hal/Rtos.h"
#include "telemetry/BlackBoxFormat.h"
#include "telemetry/BlackBoxRing.h"
#include "telemetry/BlackBoxStorage.h"
#include "telemetry/IFlightRecorder.h"

// ============================================================
// ЧЁРНЫЙ ЯЩИК — общее у самолёта (telemetry/BlackBox.h) и коптера
// (telemetry/CopterBlackBox.h): очередь, задача записи на носитель,
// заголовок полёта (схема и параметры), стирание в покое, консоль и
// выгрузка на ПК ("bb list", "bb get"). Что и когда писать — решает
// наследник: снимки кладёт в очередь (pushRecord, event), запись
// включает start()/stop(), параметры полёта дописывает headerInfo().
// ============================================================

class BlackBoxCore : public IFlightRecorder
{
public:

    enum class State : uint8_t { Off, Idle, Recording, Stopping };

    explicit BlackBoxCore(BlackBoxStorage& flightStorage) : storage(flightStorage) {}

    State getState() const { return state; }
    bool isRecording() const override { return state == State::Recording || state == State::Stopping; }

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
        writerSteps++;

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
        // Консоль работает в полётной задаче: задача записи держит замок на время
        // обращения к карте (иногда десятки мс), поэтому ждём недолго, а не вечно.
        if (!takeLock(pdMS_TO_TICKS(PRINT_LOCK_WAIT_MS)))
        {
            out.println("  носитель занят записью — список позже");
            return;
        }
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
        out.printf("  носитель: страниц записано %lu, стираний %lu, ошибок записи/стирания %lu/%lu, шагов задачи %lu\n",
                   (unsigned long)storage.pageWrites, (unsigned long)storage.eraseOps,
                   (unsigned long)storage.writeErrors, (unsigned long)storage.eraseErrors,
                   (unsigned long)writerSteps);
        giveLock();
    }

    // Стереть все полёты (консоль). Блокирует: ~40 с на весь раздел.
    bool eraseAll() override
    {
        if (isRecording() || vehicleArmed()) return false;
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
            sendFlight(static_cast<uint16_t>(flight), baud ? baud : consoleBaud());
        }
        else
        {
            Serial.println("BB:ERR неизвестная команда");
        }
    }

    // Сколько стёртого держать наготове (по умолчанию BLACKBOX_MIN_FREE_BYTES;
    // маленькому носителю — меньше, иначе стирались бы все полёты).
    void setMinFreeBytes(uint32_t bytes) { minFreeBytes = bytes; }
    uint32_t minFree() const { return minFreeBytes; }

protected:

#if defined(BOARD_STM32H743) || defined(BOARD_STM32F405)
    static constexpr uint32_t WRITER_STACK_BYTES = 8192;   // printf с float и заголовок полёта
#else
    static constexpr uint32_t WRITER_STACK_BYTES = 6144;
#endif
    static constexpr size_t HEADER_BUFFER = 8192;
    static constexpr uint32_t BOOT_VERIFY_MS = 300;
    static constexpr uint32_t PRINT_LOCK_WAIT_MS = 300;
    static constexpr uint8_t FRAME_MAGIC_0 = 0xA5;
    static constexpr uint8_t FRAME_MAGIC_1 = 0x5A;

    // Заармлен ли аппарат: стирать и выгружать — только без ARM.
    virtual bool vehicleArmed() const = 0;
    // Параметры полёта в заголовок (info(...)) — после схем, format и flight.
    virtual void headerInfo(uint16_t flight, uint32_t ms) = 0;
    // Запись началась (событие уже записано) — снимок состояния.
    virtual void onRecordingStart() {}

    // Носитель, очередь ringBytes, мьютекс, сверка стёртого, задача записи.
    // false — места для записи нет (ящик выключен) или нет памяти.
    bool beginCore(uint32_t ringBytes, uint8_t* ringMemory)
    {
        if (!storage.begin())
        {
            Serial.println("BlackBox: нет места для записи — выключен (ESP32: раздел blackbox, "
                           "partitions_blackbox.csv; H743: файл на SD-карте, tools/blackbox.py sd-prepare; "
                           "F405: SPI-флеш платы)");
            return false;
        }
        if (!ringMemory)
        {
            Serial.println("BlackBox: нет памяти под очередь — запись выключена");
            return false;
        }
        ring.begin(ringMemory, ringBytes);
        lock = xSemaphoreCreateMutex();
        state = State::Idle;

        // Стёртое впереди — сверить сразу (только чтение, ~0.1 мс на
        // сектор): заармят через секунду после включения — место уже
        // есть. Дальше — в фоне (maintain()).
        for (const uint32_t verifyStart = millis(); millis() - verifyStart < BOOT_VERIFY_MS;)
        {
            if (!storage.eraseStep(storage.totalSectors(), 0, false)) break;
        }
        return true;
    }

    void startWriterTask()
    {
        Rtos::startTask(writerTask, "bbox", WRITER_STACK_BYTES, this, Rtos::PRIORITY_TELEMETRY, &writerHandle);
    }

    // Полётный цикл: снимки положены — разбудить задачу записи.
    void wakeWriter()
    {
        if (writerHandle) xTaskNotifyGive(writerHandle);
    }

    // Скорость потока записей (раз в секунду) — для «сколько минут влезет».
    void countRate()
    {
        if (nowMs - rateWindowMs >= 1000)
        {
            bytesPerSecond = bytesThisSecond;
            bytesThisSecond = 0;
            rateWindowMs = nowMs;
        }
    }

    BlackBoxStorage& storage;
    BlackBoxRing ring;
    SemaphoreHandle_t lock = nullptr;
    uint32_t minFreeBytes = Config::BLACKBOX_MIN_FREE_BYTES;
    TaskHandle_t writerHandle = nullptr;

    volatile State state = State::Off;
    volatile bool armedFlag = false;
    volatile uint16_t flashMaxUs = 0;
    volatile uint32_t writerSteps = 0;   // сколько шагов сделала задача записи: жива ли она

    // --- сторона полётного цикла ---
    uint32_t nowUs = 0;
    uint32_t nowMs = 0;
    uint32_t cycle = 0;
    bool resetRecording = false;
    bool manualStartRequested = false;
    bool manualStopRequested = false;
    bool manualRecording = false;
    uint16_t recordingFlight = 0;
    uint32_t recordingStartMs = 0;
    const char* startReason = "";
    uint32_t disarmedSinceMs = 0;
    uint32_t stillSinceMs = 0;
    uint32_t reportedDropped = 0;
    uint32_t dropReportMs = 0;

    // Скорость потока записей — для оценки "сколько минут влезет".
    uint32_t bytesThisSecond = 0;
    uint32_t bytesPerSecond = 0;
    uint32_t rateWindowMs = 0;

    // --- сторона записи ---
    uint8_t header[HEADER_BUFFER] = {};
    size_t headerLength = 0;
    size_t headerPosition = 0;
    uint8_t carry[BlackBoxFormat::MAX_RECORD] = {};
    size_t carryLength = 0;
    bool fullReported = false;
    uint32_t reportedMediaErrors = 0;
    uint32_t mediaReportMs = 0;
    uint32_t lastEraseMs = 0;
    bool erasing = false;
    uint32_t eraseOpsAtStart = 0;

    // ========================================================
    // Старт и остановка
    // ========================================================

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
        onRecordingStart();
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


    // ========================================================
    // События и снимки
    // ========================================================

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

    // ========================================================
    // Задача записи (ядро 0)
    // ========================================================

    static void writerTask(void* arg)
    {
        BlackBoxCore* self = static_cast<BlackBoxCore*>(arg);
        for (;;)
        {
            // Будит полётный цикл после каждого такта; в покое — сам,
            // чтобы стирать на земле.
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
            self->writerStep();
        }
    }

    bool takeLock(TickType_t wait) { return lock && xSemaphoreTake(lock, wait) == pdTRUE; }
    // cppcheck-suppress cstyleCast
    void giveLock() { if (lock) xSemaphoreGive(lock); }   // макрос FreeRTOS STM32 раскрывается в приведение в стиле C

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
            reportMediaErrors();

            const bool end = carry[0] == BlackBoxFormat::REC_END;
            carryLength = 0;
            if (end)
            {
                finishFlight();
                return;
            }
        }
    }

    // Носитель отказал (карту вынуло вибрацией, сбой шины): записать в
    // журнал — не чаще раза в секунду; полёту это не мешает.
    void reportMediaErrors()
    {
        const uint32_t errors = storage.writeErrors + storage.eraseErrors;
        if (errors == reportedMediaErrors) return;
        const uint32_t ms = millis();
        if (mediaReportMs && ms - mediaReportMs < 1000) return;

        char text[BlackBoxFormat::MAX_PAYLOAD];
        snprintf(text, sizeof(text), "носитель: ошибок записи %lu, стирания %lu",
                 static_cast<unsigned long>(storage.writeErrors), static_cast<unsigned long>(storage.eraseErrors));
        uint8_t record[BlackBoxFormat::MAX_RECORD];
        ring.push(record, textRecord(record, BlackBoxFormat::REC_EVENT, text));
        reportedMediaErrors = errors;
        mediaReportMs = ms ? ms : 1;
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
        const uint32_t target = minFreeBytes / BlackBoxFormat::SECTOR_PAYLOAD + 1;
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
        explicit InfoLines(BlackBoxCore& owner, const char* key) : box(owner), prefix(key) {}

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
        BlackBoxCore& box;
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

    void buildHeader(uint16_t flight, uint32_t ms)
    {
        headerLength = headerPosition = 0;
        for (const BlackBoxFormat::Schema& s : BlackBoxFormat::SCHEMAS) schema(s);
        info("format=%u", BlackBoxFormat::VERSION);
        info("flight=%u", flight);
        headerInfo(flight, ms);
        if (headerLength + 64 > sizeof(header)) info("header=обрезан");
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

#if defined(BOARD_STM32H743) || defined(BOARD_STM32F405)
    // Скорость консоли. STM32duino не умеет её спрашивать — помним сами.
    static unsigned long& consoleBaudStore()
    {
        static unsigned long baud = 115200;
        return baud;
    }
#endif

    static unsigned long consoleBaud()
    {
#if defined(BOARD_STM32H743) || defined(BOARD_STM32F405)
        return consoleBaudStore();
#else
        return Serial.baudRate();
#endif
    }

    static void setSerialBaud(unsigned long baud)
    {
#if defined(BOARD_STM32H743) || defined(BOARD_STM32F405)
#if defined(USBCON) && defined(USBD_USE_CDC)
        (void)baud;   // Serial — USB CDC: скорость ни на что не влияет
#else
        Serial.end();
        Serial.begin(baud);
        consoleBaudStore() = baud;
#endif
#elif ARDUINO_USB_CDC_ON_BOOT
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
        if (isRecording() || vehicleArmed())
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
        const unsigned long oldBaud = consoleBaud();
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
