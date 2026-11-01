#pragma once
#include <stdint.h>

#include "hal/II2CBus.h"
#include "sensors/camera/Ov5640.h"

// ============================================================
// Камера по DCMI на STM32H743 (разъём FPC-24 платы DevEBox H743):
//   D0–D7  PC6 PC7 PE0 PE1 PE4 PD3 PE5 PE6 (AF13)
//   PCLK PA6, HREF PA4, VSYNC PB7 (AF13)
//   XCLK   PA8 = MCO1: HSI48 / 2 = 24 МГц
//   PWDN   PA7 (низкий — работать), RESET PC4 (низкий — сброс)
//   SCCB   — шина датчиков I2C2 (PB10/PB11), адрес 0x3C
// (схема DevEBox STM32H7XX_M, разъём J6; пины — Config::PIN_CAM_*).
//
// Захват — снимок одного кадра JPEG (DCMI snapshot + JPEG): DMA2
// поток 7 (DMAMUX, запрос DCMI) кладёт слова в буфер, пока не кончится
// кадр (VSYNC); длина — сколько слов успело прийти, конец JPEG — метка
// FFD9. Буфер — в AXI SRAM (DMA2 не видит DTCM); кэш данных после DMA
// сбрасывается по адресу буфера.
//
// Регистры — без HAL (модуль DCMI в HAL STM32duino выключен), по RM0433.
// На ПК — фейк в тестах (Stm32Camera не собирается вне STM32H7).
// ============================================================

// SCCB поверх шины I2C платы: 16-битный адрес регистра.
class I2cSccb : public ISccb
{
public:
    explicit I2cSccb(II2CBus& bus, uint8_t address = Ov5640::ADDRESS) : i2c(bus), addr(address) {}

    bool write(uint16_t reg, uint8_t value) override
    {
        const uint8_t b[3] = { static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF), value };
        i2c.beginTransmission(addr);
        i2c.write(b, 3);
        return i2c.endTransmission(true) == 0;
    }

    bool read(uint16_t reg, uint8_t& value) override
    {
        const uint8_t b[2] = { static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF) };
        i2c.beginTransmission(addr);
        i2c.write(b, 2);
        // SCCB не любит повторный старт — сначала стоп
        if (i2c.endTransmission(true) != 0) return false;
        if (i2c.requestFrom(addr, static_cast<uint8_t>(1)) != 1 || i2c.available() < 1) return false;
        value = static_cast<uint8_t>(i2c.read());
        return true;
    }

    void delayMs(uint32_t ms) override;

private:
    II2CBus& i2c;
    uint8_t addr;
};

#if defined(ARDUINO_ARCH_STM32) && defined(STM32H7xx)
#include <Arduino.h>

inline void I2cSccb::delayMs(uint32_t ms) { delay(ms); }

class Stm32Camera
{
public:
    // Включить такты, ножки и XCLK; камера — из PWDN и RESET.
    void powerUp(int16_t pwdnPin, int16_t resetPin)
    {
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN |
                        RCC_AHB4ENR_GPIOEEN;
        RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        (void)RCC->AHB2ENR;

        // DCMI: AF13
        af(GPIOC, 6, 13);
        af(GPIOC, 7, 13);
        af(GPIOE, 0, 13);
        af(GPIOE, 1, 13);
        af(GPIOE, 4, 13);
        af(GPIOD, 3, 13);
        af(GPIOE, 5, 13);
        af(GPIOE, 6, 13);
        af(GPIOA, 6, 13);   // PIXCLK
        af(GPIOA, 4, 13);   // HSYNC (HREF)
        af(GPIOB, 7, 13);   // VSYNC

        // XCLK: MCO1 = HSI48 / 2 на PA8 (AF0)
        RCC->CR |= RCC_CR_HSI48ON;
        for (uint32_t i = 0; i < 100000 && !(RCC->CR & RCC_CR_HSI48RDY); ++i) {}
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE)) | (4u << RCC_CFGR_MCO1_Pos) |
                    (2u << RCC_CFGR_MCO1PRE_Pos);
        af(GPIOA, 8, 0);

        if (pwdnPin >= 0)
        {
            pinMode(pwdnPin, OUTPUT);
            digitalWrite(pwdnPin, HIGH);
        }
        if (resetPin >= 0)
        {
            pinMode(resetPin, OUTPUT);
            digitalWrite(resetPin, LOW);
        }
        delay(5);
        if (pwdnPin >= 0) digitalWrite(pwdnPin, LOW);   // проснуться
        delay(2);
        if (resetPin >= 0) digitalWrite(resetPin, HIGH);
        delay(25);   // после сброса SCCB отвечает через ~20 мс
    }

    // Один кадр JPEG в buf (выровнен на 32, в AXI SRAM). Длина кадра (до
    // FFD9 включительно) или 0 — кадра нет за timeoutMs.
    uint32_t snapshot(uint8_t* buf, uint32_t capacity, uint32_t timeoutMs)
    {
        const uint32_t words = capacity / 4 > 65535 ? 65535 : capacity / 4;
        DMA_Stream_TypeDef* s = DMA2_Stream7;
        s->CR &= ~DMA_SxCR_EN;
        while (s->CR & DMA_SxCR_EN) {}
        DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;
        DMAMUX1_Channel15->CCR = DCMI_DMAMUX_REQUEST;   // DMA2 поток 7 = канал 8 + 7
        s->PAR = reinterpret_cast<uint32_t>(&DCMI->DR);
        s->M0AR = reinterpret_cast<uint32_t>(buf);
        s->NDTR = words;
        s->CR = DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC;   // периферия → память, слова
        s->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1;
        s->CR |= DMA_SxCR_EN;

        DCMI->CR = 0;
        DCMI->ICR = 0x1F;
        // JPEG, снимок, PCLK — по фронту; VSYNC «пауза» — высокий, HREF «пауза» — низкий
        DCMI->CR = DCMI_CR_JPEG | DCMI_CR_CM | DCMI_CR_PCKPOL | DCMI_CR_VSPOL | polarityExtra;
        DCMI->CR |= DCMI_CR_ENABLE;
        DCMI->CR |= DCMI_CR_CAPTURE;

        const uint32_t start = millis();
        bool done = false;
        while (millis() - start < timeoutMs)
        {
            if (DCMI->RISR & DCMI_RIS_FRAME_RIS)
            {
                done = true;
                break;
            }
            if (DCMI->RISR & DCMI_RIS_OVR_RIS) break;
            delay(1);
        }
        lastStatus = DCMI->RISR;
        DCMI->CR &= ~DCMI_CR_CAPTURE;
        DCMI->CR &= ~DCMI_CR_ENABLE;
        s->CR &= ~DMA_SxCR_EN;
        while (s->CR & DMA_SxCR_EN) {}
        const uint32_t got = (words - s->NDTR) * 4;
        lastBytes = got;
        if (SCB->CCR & SCB_CCR_DC_Msk) SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(buf), static_cast<int32_t>((got + 31) & ~31u));
        if (!done || got < 4) return 0;
        return jpegLength(buf, got);
    }

    // Длина JPEG в принятом: от FFD8 в начале до FFD9; 0 — не JPEG.
    static uint32_t jpegLength(const uint8_t* b, uint32_t n)
    {
        if (n < 4 || b[0] != 0xFF || b[1] != 0xD8) return 0;
        for (uint32_t i = n - 1; i > 2; --i)
        {
            if (b[i] == 0xD9 && b[i - 1] == 0xFF) return i + 1;
        }
        return 0;
    }

    uint32_t lastRawBytes() const { return lastBytes; }
    uint32_t lastRis() const { return lastStatus; }
    uint32_t polarityExtra = 0;   // для подбора полярностей на стенде (DCMI_CR_HSPOL)

private:
    static constexpr uint32_t DCMI_DMAMUX_REQUEST = 75;   // dcmi_dma (RM0433, DMAMUX1)

    static void af(GPIO_TypeDef* port, uint8_t pin, uint8_t function)
    {
        port->MODER = (port->MODER & ~(3u << (pin * 2))) | (2u << (pin * 2));
        port->OSPEEDR |= 3u << (pin * 2);
        port->PUPDR &= ~(3u << (pin * 2));
        volatile uint32_t& afr = port->AFR[pin >> 3];
        afr = (afr & ~(0xFu << ((pin & 7) * 4))) | (static_cast<uint32_t>(function) << ((pin & 7) * 4));
    }

    uint32_t lastBytes = 0;
    uint32_t lastStatus = 0;
};
#else
#include <chrono>
#include <thread>
inline void I2cSccb::delayMs(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
#endif
