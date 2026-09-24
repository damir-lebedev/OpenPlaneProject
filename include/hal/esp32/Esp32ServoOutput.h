#pragma once
#include <Arduino.h>
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#include "../IServoOutput.h"

// ============================================================
// Реализация IServoOutput для ESP32 — аппаратный LEDC напрямую
// (ledcSetup/ledcAttachPin/ledcWrite из Arduino core 2.x).
//
// Раньше здесь была обёртка над библиотекой ESP32Servo. Версия
// 3.2.1 на ESP32-S3 раскладывает сервы по MCPWM и в attachPin()
// путает номер блока MCPWM с номером таймера: пины второго таймера
// получают сигнал первого. На стенде это выглядело так: GPIO6
// (руль высоты) повторял GPIO4 (левый элерон), а GPIO7 (ESC) —
// GPIO5 (правый элерон), то есть мотор управлялся правым стиком,
// а стик газа ни на что не влиял. LEDC есть на всех ESP32 (S3 — 8
// каналов, C3 — 6, classic — 16), четырём выходам его хватает.
//
// Каждому выходу — свой канал LEDC (задаёт Esp32Board). Каналы
// 2n и 2n+1 делят один таймер; у всех выходов одинаковые 50 Гц,
// так что это не мешает.
// ============================================================

class Esp32ServoOutput : public IServoOutput
{
public:

    Esp32ServoOutput(uint8_t servoPin, uint8_t ledcChannel)
        : pin(servoPin),
          channel(ledcChannel)
    {
    }

    bool attach(uint16_t minUs, uint16_t maxUs) override
    {
        rangeMinUs = minUs;
        rangeMaxUs = maxUs;

        // ledcSetup() возвращает реально выставленную частоту, 0 — ошибка.
        attached = ledcSetup(channel, FREQUENCY_HZ, RESOLUTION_BITS) != 0;
        if (attached)
        {
            ledcAttachPin(pin, channel);
        }
        return attached;
    }

    // Как и у Servo::writeMicroseconds(), значение ограничивается
    // диапазоном, заданным в attach().
    void writeMicroseconds(uint16_t us) override
    {
        if (!attached) return;

        const uint32_t clamped = constrain(us, rangeMinUs, rangeMaxUs);
        ledcWrite(channel, clamped * MAX_DUTY / PERIOD_US);
    }

    bool isAttached() const override { return attached; }

    // Включаем входной буфер того же GPIO (бит FUN_IE в IO_MUX) —
    // выход PWM при этом не трогается, а digitalRead()/pulseIn()
    // видят уровень, который пин реально выдаёт.
    int32_t measurePulseUs() override
    {
        if (!attached) return -1;

        PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[pin]);
        const unsigned long width = pulseIn(pin, HIGH, 30000);
        return width ? static_cast<int32_t>(width) : -1;
    }


private:

    static constexpr uint32_t FREQUENCY_HZ = 50;
    static constexpr uint32_t PERIOD_US = 1000000 / FREQUENCY_HZ;

    // 14 бит — максимум LEDC на S3/C3: 20000 мкс / 16384 ≈ 1.2 мкс на шаг.
    static constexpr uint8_t RESOLUTION_BITS = 14;
    static constexpr uint32_t MAX_DUTY = (1u << RESOLUTION_BITS);

    uint8_t pin;
    uint8_t channel;
    uint16_t rangeMinUs = 1000;
    uint16_t rangeMaxUs = 2000;
    bool attached = false;
};
