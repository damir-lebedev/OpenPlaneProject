#pragma once

// ============================================================
// Нативная замена "железных" функций Arduino core ESP32 2.0.x и
// FreeRTOS: время, GPIO, LEDC, pulseIn, задачи, критические секции.
//
// Всё состояние симулированного мира — в namespace fake, тесты
// читают и меняют его напрямую (fake::advanceMs(), fake::gpio(),
// fake::ledc(), fake::tasks() ...). fake::resetHal() возвращает мир
// в исходное состояние.
//
// Время не идёт само: только fake::advance*(), delay() и задержки
// FreeRTOS. millis()/micros() возвращают uint32_t, как unsigned long
// на ESP32, поэтому переполнение счётчиков ведёт себя как на плате.
// ============================================================

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

namespace fake
{
    // --------------------------------------------------------
    // Время
    // --------------------------------------------------------

    struct ClockState
    {
        uint64_t nowUs = 0;
    };

    inline ClockState& clock()
    {
        static ClockState state;
        return state;
    }

    inline void setTimeUs(uint64_t us) { clock().nowUs = us; }
    inline void advanceUs(uint64_t us) { clock().nowUs += us; }
    inline void advanceMs(uint64_t ms) { clock().nowUs += ms * 1000ULL; }
    inline uint64_t nowUs() { return clock().nowUs; }

    // --------------------------------------------------------
    // GPIO
    // --------------------------------------------------------

    constexpr uint8_t GPIO_COUNT = 96;   // ESP32 — до 48, фейк STM32 — порты A..E (0x00..0x4F)

    struct GpioState
    {
        uint8_t mode[GPIO_COUNT] = {};
        uint8_t level[GPIO_COUNT] = {};
        bool inputEnabled[GPIO_COUNT] = {};
        uint32_t writes[GPIO_COUNT] = {};
        // pulseIn(): >= 0 — вернуть это значение вместо измерения по LEDC.
        long pulseOverride[GPIO_COUNT];
        uint32_t analogMv[GPIO_COUNT] = {};   // напряжение на пине для АЦП
        uint32_t analogReads[GPIO_COUNT] = {};

        GpioState()
        {
            for (long& p : pulseOverride) p = -1;
        }
    };

    inline GpioState& gpio()
    {
        static GpioState state;
        return state;
    }

    // --------------------------------------------------------
    // LEDC
    // --------------------------------------------------------

    constexpr uint8_t LEDC_CHANNELS = 16;

    struct LedcChannel
    {
        bool configured = false;
        double frequency = 0;
        uint8_t resolutionBits = 0;
        int pin = -1;
        uint32_t duty = 0;
        uint32_t writeCount = 0;
    };

    struct LedcState
    {
        LedcChannel channel[LEDC_CHANNELS];
        bool failSetup[LEDC_CHANNELS] = {};
    };

    inline LedcState& ledc()
    {
        static LedcState state;
        return state;
    }

    // Ширина импульса на пине по настройкам LEDC, мкс; -1 — на пине нет PWM.
    inline long ledcPulseUs(uint8_t pin)
    {
        for (const LedcChannel& ch : ledc().channel)
        {
            if (ch.configured && ch.pin == pin && ch.frequency > 0)
            {
                const double periodUs = 1000000.0 / ch.frequency;
                return static_cast<long>(periodUs * ch.duty / (1u << ch.resolutionBits) + 0.5);
            }
        }
        return -1;
    }

    // --------------------------------------------------------
    // Задачи FreeRTOS
    // --------------------------------------------------------

    // Бросается из задержек FreeRTOS, когда исчерпан бюджет итераций
    // runTask(): так тест прерывает бесконечный цикл задачи.
    struct TaskStop {};

    struct TaskRecord
    {
        void (*function)(void*) = nullptr;
        std::string name;
        uint32_t stackDepth = 0;
        void* parameter = nullptr;
        unsigned priority = 0;
        int core = -1;
    };

    struct TaskState
    {
        std::vector<TaskRecord> created;
        int delayBudget = -1;      // < 0 — без ограничений
        uint32_t delayCalls = 0;
        int criticalDepth = 0;
        uint32_t criticalEntries = 0;
        uint32_t notifications = 0;   // xTaskNotifyGive()
    };

    inline TaskState& tasks()
    {
        static TaskState state;
        return state;
    }

    inline void consumeDelay()
    {
        TaskState& t = tasks();
        t.delayCalls++;
        if (t.delayBudget < 0) return;
        if (t.delayBudget == 0) throw TaskStop{};
        t.delayBudget--;
    }

    // Выполнить тело задачи iterations раз (по числу задержек внутри
    // её бесконечного цикла) и вернуться.
    inline void runTask(const TaskRecord& task, int iterations)
    {
        tasks().delayBudget = iterations - 1;
        try
        {
            task.function(task.parameter);
        }
        catch (const TaskStop&)
        {
        }
        tasks().delayBudget = -1;
    }

    inline const TaskRecord* findTask(const std::string& name)
    {
        for (const TaskRecord& t : tasks().created)
        {
            if (t.name == name) return &t;
        }
        return nullptr;
    }

    // --------------------------------------------------------
    // Прочее
    // --------------------------------------------------------

    struct SystemState
    {
        uint32_t freeHeap = 200 * 1024;
        bool psram = true;            // psramFound(): N16R8 — 8 МБ
        int resetReason = 1;          // esp_reset_reason(): ESP_RST_POWERON
    };

    inline SystemState& chip()
    {
        static SystemState state;
        return state;
    }

    inline void resetHal()
    {
        clock() = ClockState();
        gpio() = GpioState();
        ledc() = LedcState();
        tasks() = TaskState();
        chip() = SystemState();
    }
}

// ------------------------------------------------------------
// Arduino: время
// ------------------------------------------------------------

inline uint32_t millis() { return static_cast<uint32_t>(fake::nowUs() / 1000ULL); }
inline uint32_t micros() { return static_cast<uint32_t>(fake::nowUs()); }
inline void delay(uint32_t ms) { fake::advanceMs(ms); }
inline void delayMicroseconds(uint32_t us) { fake::advanceUs(us); }
inline void yield() {}

// ------------------------------------------------------------
// Arduino: GPIO
// ------------------------------------------------------------

#define LOW    0x0
#define HIGH   0x1
#define INPUT  0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05

inline void pinMode(uint8_t pin, uint8_t mode)
{
    if (pin < fake::GPIO_COUNT) fake::gpio().mode[pin] = mode;
}

inline void digitalWrite(uint8_t pin, uint8_t value)
{
    if (pin >= fake::GPIO_COUNT) return;
    fake::gpio().level[pin] = value ? HIGH : LOW;
    fake::gpio().writes[pin]++;
}

inline int digitalRead(uint8_t pin)
{
    return pin < fake::GPIO_COUNT ? fake::gpio().level[pin] : LOW;
}

// Импульс видно, только если у пина включён входной буфер (как на
// ESP32: выход LEDC сам по себе не читается digitalRead/pulseIn).
inline unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L)
{
    if (pin >= fake::GPIO_COUNT)
    {
        return 0;
    }
    const long override = fake::gpio().pulseOverride[pin];
    long width = override >= 0 ? override : fake::ledcPulseUs(pin);
    if (!fake::gpio().inputEnabled[pin] || state != HIGH || width <= 0)
    {
        fake::advanceUs(timeout);
        return 0;
    }
    fake::advanceUs(20000);
    return static_cast<unsigned long>(width);
}

// Arduino core ESP32 2.x: АЦП, напряжение на пине в мВ.
inline uint32_t analogReadMilliVolts(uint8_t pin)
{
    if (pin >= fake::GPIO_COUNT) return 0;
    fake::gpio().analogReads[pin]++;
    return fake::gpio().analogMv[pin];
}

// ------------------------------------------------------------
// Arduino core ESP32 2.x: LEDC
// ------------------------------------------------------------

inline double ledcSetup(uint8_t channel, double frequency, uint8_t resolutionBits)
{
    if (channel >= fake::LEDC_CHANNELS || fake::ledc().failSetup[channel] || resolutionBits > 20)
    {
        return 0;
    }
    fake::LedcChannel& ch = fake::ledc().channel[channel];
    ch.configured = true;
    ch.frequency = frequency;
    ch.resolutionBits = resolutionBits;
    return frequency;
}

inline void ledcAttachPin(uint8_t pin, uint8_t channel)
{
    if (channel < fake::LEDC_CHANNELS) fake::ledc().channel[channel].pin = pin;
}

inline void ledcWrite(uint8_t channel, uint32_t duty)
{
    if (channel >= fake::LEDC_CHANNELS) return;
    fake::ledc().channel[channel].duty = duty;
    fake::ledc().channel[channel].writeCount++;
}

inline uint32_t ledcRead(uint8_t channel)
{
    return channel < fake::LEDC_CHANNELS ? fake::ledc().channel[channel].duty : 0;
}

// ------------------------------------------------------------
// FreeRTOS (тик 1 мс, как CONFIG_FREERTOS_HZ=1000 в Arduino ESP32)
// ------------------------------------------------------------

typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef void* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);

#define pdPASS 1
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))
#define portTICK_PERIOD_MS 1

inline TickType_t xTaskGetTickCount() { return millis(); }

inline void vTaskDelay(TickType_t ticks)
{
    fake::advanceMs(ticks);
    fake::consumeDelay();
}

// Как в FreeRTOS: проснуться в *previousWake + increment; если это
// время уже прошло — вернуться сразу.
inline void vTaskDelayUntil(TickType_t* previousWake, TickType_t increment)
{
    const TickType_t wake = *previousWake + increment;
    const TickType_t now = xTaskGetTickCount();
    if (static_cast<int32_t>(wake - now) > 0)
    {
        fake::advanceMs(wake - now);
    }
    *previousWake = wake;
    fake::consumeDelay();
}

inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t function, const char* name, uint32_t stackDepth,
                                          void* parameter, UBaseType_t priority, TaskHandle_t* handle,
                                          BaseType_t core)
{
    fake::TaskRecord record;
    record.function = function;
    record.name = name ? name : "";
    record.stackDepth = stackDepth;
    record.parameter = parameter;
    record.priority = priority;
    record.core = core;
    fake::tasks().created.push_back(record);
    // Ненулевой и у каждой задачи свой — как настоящий хэндл.
    if (handle) *handle = reinterpret_cast<TaskHandle_t>(fake::tasks().created.size());
    return pdPASS;
}

#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY 0xFFFFFFFFu

// Уведомления задач: в фейке — счётчик; ожидание — как задержка
// (fake::runTask() считает проходы цикла задачи по ним).
inline void xTaskNotifyGive(TaskHandle_t handle)
{
    if (handle) fake::tasks().notifications++;
}

inline uint32_t ulTaskNotifyTake(BaseType_t clearOnExit, TickType_t ticks)
{
    (void)clearOnExit;
    (void)ticks;
    fake::consumeDelay();
    return 1;
}

// Мьютекс FreeRTOS: в однопоточном фейке — флаг "занят".
namespace fake
{
    struct Mutex
    {
        bool taken = false;
        uint32_t takes = 0;
    };
}

typedef fake::Mutex* SemaphoreHandle_t;

inline SemaphoreHandle_t xSemaphoreCreateMutex()
{
    static std::vector<fake::Mutex*> all;   // живут до конца теста, как у задач
    all.push_back(new fake::Mutex());
    return all.back();
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t ticks)
{
    (void)ticks;
    if (!mutex || mutex->taken) return pdFALSE;
    mutex->taken = true;
    mutex->takes++;
    return pdTRUE;
}

inline BaseType_t xSemaphoreGive(SemaphoreHandle_t mutex)
{
    if (!mutex || !mutex->taken) return pdFALSE;
    mutex->taken = false;
    return pdTRUE;
}

struct portMUX_TYPE
{
    uint32_t owner;
    uint32_t count;
};

#define portMUX_INITIALIZER_UNLOCKED { 0xB33FFFFF, 0 }

namespace fake
{
    inline void enterCritical(portMUX_TYPE* mux)
    {
        mux->count++;
        tasks().criticalDepth++;
        tasks().criticalEntries++;
    }

    inline void exitCritical(portMUX_TYPE* mux)
    {
        mux->count--;
        tasks().criticalDepth--;
    }
}

#define portENTER_CRITICAL(mux) (::fake::enterCritical(mux))
#define portEXIT_CRITICAL(mux) (::fake::exitCritical(mux))

// ------------------------------------------------------------
// ESP
// ------------------------------------------------------------

class EspClass
{
public:
    uint32_t getFreeHeap() const { return fake::chip().freeHeap; }
};

inline EspClass ESP;

// ------------------------------------------------------------
// PSRAM (esp32-hal-psram.h)
// ------------------------------------------------------------

inline bool psramFound() { return fake::chip().psram; }
inline void* ps_malloc(size_t size) { return fake::chip().psram ? malloc(size) : nullptr; }
