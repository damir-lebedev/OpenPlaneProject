#pragma once
#include <Arduino.h>

#include "hal/IServoOutput.h"

// ============================================================
// Реализация IServoOutput для STM32 (STM32duino 3.x) — аппаратный
// PWM таймера (HardwareTimer), 50 Гц. ЗАГОТОВКА: на железе не
// проверялась.
//
// Таймер и его канал определяются по пину из таблицы PinMap_TIM
// варианта (как это делает analogWrite() ядра): PA0 -> TIM2_CH1,
// PD14 -> TIM4_CH3 и т.д. Импульс формирует сам таймер, без
// прерываний и без участия CPU — в отличие от библиотеки Servo
// для STM32, которая дёргает пины из прерывания одного таймера и
// даёт джиттер под нагрузкой.
//
// Несколько выходов на одном таймере (CH1..CH4 одного TIMx) обязаны
// делить ОДИН объект HardwareTimer: каждый новый объект на тот же
// TIMx перезаписал бы обработчик прерываний ядра
// (HardwareTimer_Handle[index]). Поэтому таймеры берутся из общего
// пула acquireTimer(), а не создаются каждым выходом. Период 20 мс
// у всех выходов одинаковый, так что общий таймер ничему не мешает.
//
// Пин < 0 — выход на этой плате не разведён: attach() вернёт
// false, запись будет пропускаться (как у Esp32ServoOutput).
// ============================================================

class Stm32ServoOutput : public IServoOutput
{
public:

    explicit Stm32ServoOutput(int16_t servoPin);

    bool attach(uint16_t minUs, uint16_t maxUs) override;

    // Как и у Servo::writeMicroseconds(), значение ограничивается
    // диапазоном, заданным в attach(). Регистр сравнения с
    // предзагрузкой: новое значение вступает в силу со следующего
    // периода, импульс не рвётся посередине.
    void writeMicroseconds(uint16_t us) override;

    bool isAttached() const override { return attached; }

    // На STM32 входной регистр GPIO (IDR) отражает уровень пина и в
    // режиме альтернативной функции, так что pulseIn() видит
    // импульс таймера без перенастройки пина (на ESP32 для этого
    // пришлось включать входной буфер вручную).
    int32_t measurePulseUs() override;


private:

    static constexpr uint32_t FREQUENCY_HZ = 50;
    static constexpr uint32_t PERIOD_US = 1000000 / FREQUENCY_HZ;

    // Сколько разных таймеров могут занять сервовыходы. Пять выходов
    // текущей распиновки сидят на двух (TIM2, TIM4).
    static constexpr uint8_t MAX_TIMERS = 4;

    int16_t pin;
    HardwareTimer* timer = nullptr;
    uint32_t channel = 0;
    uint16_t rangeMinUs = 1000;
    uint16_t rangeMaxUs = 2000;
    bool attached = false;

    // Общий для всех выходов пул: один HardwareTimer на TIMx.
    // Период задаётся один раз, при первом выходе на этом таймере;
    // setOverflow() в MICROSEC_FORMAT сам подбирает делитель под
    // 16-битный счётчик (и для 32-битного TIM2 тоже): при тактовой
    // таймера 240 МГц шаг ~0.3 мкс — вчетверо точнее LEDC на ESP32.
    static HardwareTimer* acquireTimer(TIM_TypeDef* instance);
};
