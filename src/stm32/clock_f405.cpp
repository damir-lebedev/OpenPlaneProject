// ============================================================
// Такты STM32F405 (полётники F405: env *-f405*): кварц HSE_VALUE (у всех
// полётников F405 под Betaflight — 8 МГц, иначе дамп платы покажет
// system_hse_mhz) вместо внутреннего HSI варианта genericSTM32F405RG:
//   PLL: HSE / M = 2 МГц × 168 = VCO 336 МГц; P / 2 = 168 МГц — ядро;
//        Q / 7 = 48 МГц — USB (от HSI USB работает ненадёжно: точность
//        HSI ~1 %, USB нужно 0.25 %);
//   шины: AHB 168 МГц, APB1 42 МГц (таймеры 84), APB2 84 МГц (таймеры 168).
// Кварц не запустился (другая плата, брак) — те же 168 МГц от HSI, как у
// варианта ядра: прошивка работает, консоль по USB — как повезёт.
// ============================================================

#if defined(BOARD_STM32F405) && defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>

static_assert(HSE_VALUE % 2000000UL == 0 && HSE_VALUE >= 4000000UL && HSE_VALUE <= 26000000UL,
              "F405: кварц — чётное число МГц от 4 до 26 (флаг -D HSE_VALUE в platformio.ini)");

namespace
{
    bool configurePll(uint32_t source, uint32_t m)
    {
        RCC_OscInitTypeDef osc = {};
        if (source == RCC_PLLSOURCE_HSE)
        {
            osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
            osc.HSEState = RCC_HSE_ON;
        }
        else
        {
            osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
            osc.HSIState = RCC_HSI_ON;
            osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        }
        osc.PLL.PLLState = RCC_PLL_ON;
        osc.PLL.PLLSource = source;
        osc.PLL.PLLM = m;                   // 2 МГц на входе PLL
        osc.PLL.PLLN = 168;                 // VCO 336 МГц
        osc.PLL.PLLP = RCC_PLLP_DIV2;       // 168 МГц
        osc.PLL.PLLQ = 7;                   // 48 МГц: USB
        return HAL_RCC_OscConfig(&osc) == HAL_OK;
    }
}

extern "C" void SystemClock_Config(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    if (!configurePll(RCC_PLLSOURCE_HSE, HSE_VALUE / 2000000UL))
    {
        if (!configurePll(RCC_PLLSOURCE_HSI, HSI_VALUE / 2000000UL)) Error_Handler();
    }

    RCC_ClkInitTypeDef clk = {};
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

#endif
