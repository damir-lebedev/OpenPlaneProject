#pragma once
#include <stdint.h>

// ============================================================
// Время частей полётного цикла — для `tasks` в CLI коптера (как в
// Betaflight): сколько раз в секунду часть работала, сколько длилась в
// среднем и в худший раз. Окно — 1 с: publish() раз в секунду
// переносит накопленное в rateHz/avgUs/maxUs (их читает консоль из той
// же задачи). Без Arduino: время передаёт вызывающий.
// ============================================================

class TaskStats
{
public:
    static constexpr uint8_t MAX_TASKS = 10;

    struct Task
    {
        const char* name = nullptr;
        uint32_t rateHz = 0, avgUs = 0, maxUs = 0;   // за прошлую секунду
        uint32_t peakUs = 0;                         // худшее с включения
        uint32_t count = 0, sumUs = 0, windowMaxUs = 0;
    };

    // Номер новой части; MAX_TASKS — места нет.
    uint8_t add(const char* name)
    {
        if (taskCount >= MAX_TASKS) return MAX_TASKS;
        tasks[taskCount].name = name;
        return taskCount++;
    }

    void record(uint8_t id, uint32_t us)
    {
        if (id >= taskCount) return;
        Task& t = tasks[id];
        ++t.count;
        t.sumUs += us;
        if (us > t.windowMaxUs) t.windowMaxUs = us;
        if (us > t.peakUs) t.peakUs = us;
    }

    // Каждый цикл: раз в секунду — новые цифры.
    void publish(uint32_t nowMs)
    {
        const uint32_t elapsed = nowMs - windowStartMs;
        if (elapsed < 1000) return;
        for (uint8_t i = 0; i < taskCount; ++i)
        {
            Task& t = tasks[i];
            t.rateHz = static_cast<uint32_t>(static_cast<uint64_t>(t.count) * 1000u / elapsed);
            t.avgUs = t.count ? t.sumUs / t.count : 0;
            t.maxUs = t.windowMaxUs;
            t.count = t.sumUs = t.windowMaxUs = 0;
        }
        windowStartMs = nowMs;
    }

    uint8_t size() const { return taskCount; }
    const Task& task(uint8_t id) const { return tasks[id < taskCount ? id : 0]; }

    // Доля процессора, %: средняя и в худший раз (как avgload/maxload в BF).
    static float avgLoad(const Task& t) { return static_cast<float>(t.avgUs) * t.rateHz / 10000.0f; }
    static float maxLoad(const Task& t) { return static_cast<float>(t.maxUs) * t.rateHz / 10000.0f; }

private:
    Task tasks[MAX_TASKS];
    uint8_t taskCount = 0;
    uint32_t windowStartMs = 0;
};
