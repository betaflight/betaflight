/**
  ******************************************************************************
  * @file    system_stm32g4xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include <string.h>

#include "stm32g4xx.h"
#include "drivers/system.h"
#include "platform.h"
#include "drivers/persistent.h"

#if !defined  (HSE_VALUE)
  #define HSE_VALUE     8000000U /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    16000000U /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

  /* The SystemCoreClock variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
  uint32_t SystemCoreClock = HSI_VALUE;

  const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
  const uint8_t APBPrescTable[8] =  {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

void SystemClock_Config(void); // Forward

void SystemInit(void)
{
  systemCheckResetReason();

  initialiseMemorySections();

  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
  #endif

  /* Configure the Vector Table location add offset address ------------------*/

    extern uint8_t isr_vector_table_flash_base;
    extern uint8_t isr_vector_table_base;
    extern uint8_t isr_vector_table_end;

    if (&isr_vector_table_base != &isr_vector_table_flash_base) {
        memcpy(&isr_vector_table_base, &isr_vector_table_flash_base, (size_t) (&isr_vector_table_end - &isr_vector_table_base));
    }

    SCB->VTOR = (uint32_t)&isr_vector_table_base;

#ifdef USE_HAL_DRIVER
    HAL_Init();
#endif

    SystemClock_Config();
    SystemCoreClockUpdate();
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(**)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(***)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(***)
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (**) HSI_VALUE is a constant defined in stm32g4xx_hal.h file (default value
  *              16 MHz) but the real value may vary depending on the variations
  *              in voltage and temperature.
  *
  *         (***) HSE_VALUE is a constant defined in stm32g4xx_hal.h file (default value
  *              8 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);

  uint32_t tmp, pllvco, pllr, pllsource, pllm;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (RCC->CFGR & RCC_CFGR_SWS)
  {
    case 0x04:  /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;

    case 0x08:  /* HSE used as system clock source */
      SystemCoreClock = hse_value;
      break;

    case 0x0C:  /* PLL used as system clock  source */
      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
         SYSCLK = PLL_VCO / PLLR
         */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
      pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 4) + 1U ;
      if (pllsource == 0x02UL) /* HSI used as PLL clock source */
      {
        pllvco = (HSI_VALUE / pllm);
      }
      else                   /* HSE used as PLL clock source */
      {
        pllvco = (hse_value / pllm);
      }
      pllvco = pllvco * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 8);
      pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 25) + 1U) * 2U;
      SystemCoreClock = pllvco/pllr;
      break;

    default:
      break;
  }
  /* Compute HCLK clock frequency --------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
}

// SystemSYSCLKSource
//   0: HSI
//   1: HSE
//   (2: PLLP)
//   3: PLLR

int SystemSYSCLKSource(void)
{
    uint32_t rawSrc = RCC->CFGR & RCC_CFGR_SW;
    int src = 0;

    switch (rawSrc) {
    case 0: // can't happen, fall through
        FALLTHROUGH;
    case 1:
        src = 0; // HSI 
        break;

    case 2:
        src = 1; // HSE
        break;

    case 3:
        src = 3; // PLL-R
    }

    return src;
}

// SystemPLLSource
//   0: HSI
//   1: HSE

int SystemPLLSource(void)
{
    return (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) & 1; // LSB determines HSI(0) or HSE(1)
}

void Error_Handler(void)
{
    while (1) {
    }
}

/*
 * G4 is capable of fine granularity overclocking thanks to a separate 48MHz source,
 * but we keep the overclocking capability to match that of F405 (clocks at 168, 192, 216, 240).
 *
 * However, due to restrictions on MCO source, designs that wishes to generate 27MHz on MCO
 * must use a 27MHz HSE source. The 27MHz HSE source will produce slightly different series of clocks
 * due to restriction on PLL multipler range.
 *
 * If mhz == 8, 16 or 24 then scale it down to 4 and start with PLLN=42 for base 168MHz,
 * with PLLN increment of 6 (4 * 6 = 24MHz a part)
 *
 * If mhz == 27 then scale it down to 9 with PLL=19 for base 171MHz with PLLN increment of 3 (9 * 3 = 27MHz a part)
 *
 * We don't prepare a separate frequency selection for 27MHz series in CLI, so what is set with "cpu_overclock" 
 * will result in slightly higher clock when examined with "status" command.
 */

// Target frequencies for cpu_overclock (Levels 0 through 3)

uint16_t sysclkSeries8[] =  { 168, 192, 216, 240 };
uint16_t sysclkSeries27[] = { 171, 198, 225, 252 };
#define OVERCLOCK_LEVELS ARRAYLEN(sysclkSeries8)

// Generic fine granularity PLL parameter calculation

static bool systemComputePLLParameters(uint8_t src, uint16_t target, int *sysclk, int *pllm, int *plln, int *pllr)
{
    int vcoTarget = target * 2;
    int vcoBase;
    int vcoDiff;
    int multDiff;
    int vcoFreq;

    *pllr = 2;

    if (src == 8 || src == 16 || src == 24) {
        *pllm = src / 8;
        vcoBase = 168 * 2;
        vcoDiff = vcoTarget - vcoBase;
        multDiff = vcoDiff / 16 * 2;
        *plln = 42 + multDiff;
        vcoFreq = 8 * *plln;
    } else if (src == 27) {
        *pllm = 3;
        vcoBase = 171 * 2;
        vcoDiff = vcoTarget - vcoBase;
        multDiff = vcoDiff / 18 * 2;
        *plln = 38 + multDiff;
        vcoFreq = 9 * *plln;
    } else {
        return false;
    }

    // VCO seems to top out at 590MHz or so. Give it some margin.

    if (vcoFreq >= 560) {
        return false;
    }
    *sysclk = vcoFreq / 2;

    return true;
}

static int pll_m;
static int pll_n;
static int pll_r;
static uint32_t pllSrc;
static int sysclkMhz;

static bool systemClock_PLLConfig(int overclockLevel)
{
    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);
    int pllInput;
    int targetMhz;

    if (hse_value == 0) {
        pllInput = 16; // HSI
        pllSrc = RCC_PLLSOURCE_HSI;
        targetMhz = 168;
    } else {
        pllInput = hse_value / 1000000;
        pllSrc = RCC_PLLSOURCE_HSE;
        if (pllInput == 8 || pllInput == 16 || pllInput == 24) {
            targetMhz = sysclkSeries8[overclockLevel];
        } else if (pllInput == 27) {
            targetMhz = sysclkSeries8[overclockLevel];
        } else {
            return false;
        }
    }

    return systemComputePLLParameters(pllInput, targetMhz, &sysclkMhz, &pll_m, &pll_n, &pll_r);
}

void systemClockSetHSEValue(uint32_t frequency)
{
    uint32_t freqMhz = frequency / 1000000;

    // Only supported HSE crystal/resonator is 27MHz or integer multiples of 8MHz

    if (freqMhz != 27 && (freqMhz / 8) * 8 != freqMhz) {
        return;
    } 

    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);

    if (hse_value != frequency) {
        persistentObjectWrite(PERSISTENT_OBJECT_HSE_VALUE, frequency);

        if (hse_value != 0) {
            __disable_irq();
            NVIC_SystemReset();
        }
    }
}

void OverclockRebootIfNecessary(unsigned requestedOverclockLevel)
{
    uint32_t currentOverclockLevel = persistentObjectRead(PERSISTENT_OBJECT_OVERCLOCK_LEVEL);

    if (requestedOverclockLevel >= OVERCLOCK_LEVELS ) {
        requestedOverclockLevel = 0;
    }

    // If we are not running at the requested speed or
    // we are running on PLL-HSI even HSE has been set,
    // then remember the requested clock and issue soft reset

    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);

    if ((currentOverclockLevel != requestedOverclockLevel) ||
            (hse_value &&
             SystemSYSCLKSource() == 3 /* PLL-R */ &&
             SystemPLLSource() != 1 /* HSE */)) {

        // Make sure we can configure the requested clock.
        if (!systemClock_PLLConfig(requestedOverclockLevel)) {
            return;
        }
        persistentObjectWrite(PERSISTENT_OBJECT_OVERCLOCK_LEVEL, requestedOverclockLevel);
        __disable_irq();
        NVIC_SystemReset();
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
// Extracted from MX generated main.c 

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_CRSInitTypeDef pInit = {0};

  systemClock_PLLConfig(persistentObjectRead(PERSISTENT_OBJECT_OVERCLOCK_LEVEL));

  // Configure the main internal regulator output voltage 

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  // Initializes the CPU, AHB and APB busses clocks 

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;

  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = pllSrc;
  RCC_OscInitStruct.PLL.PLLM = pll_m;
  RCC_OscInitStruct.PLL.PLLN = pll_n;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Initializes the CPU, AHB and APB busses clocks 

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }

  // Initializes the peripherals clocks 

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_I2C4|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC345
                              |RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_LPUART1
                              ;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  // Configures CRS 

  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);
}
