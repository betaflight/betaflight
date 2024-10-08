/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
  *          This file contains the system clock configuration for STM32F4xx devices.
  *
  * 1.  This file provides two functions and one global variable to be called from
  *     user application:
  *      - SystemInit(): Setups the system clock (System clock source, PLL Multiplier
  *                      and Divider factors, AHB/APBx prescalers and Flash settings),
  *                      depending on the configuration made in the clock xls tool.
  *                      This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  * 2. After each device reset the HSI (16 MHz) is used as system clock source.
  *    Then SystemInit() function is called, in "startup_stm32f4xx.s" file, to
  *    configure the system clock before to branch to main program.
  *
  * 3. If the system clock source selected by user fails to startup, the SystemInit()
  *    function will do nothing and HSI still used as system clock source. User can
  *    add some code to deal with this issue inside the SetSysClock() function.
  *
  * 4. The default value of HSE crystal is set to 25MHz, refer to "HSE_VALUE" define
  *    in "stm32f4xx.h" file. When HSE is used as system clock source, directly or
  *    through PLL, and you are using different crystal you have to adapt the HSE
  *    value to your own configuration.
  *
  * 5. This file configures the system clock as follows:
  *=============================================================================
  *=============================================================================
  *                    Supported STM32F40xxx/41xxx devices
  *-----------------------------------------------------------------------------
  *        System Clock source                    | PLL (HSE)
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 168000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 168000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 4
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 2
  *-----------------------------------------------------------------------------
  *        HSE Frequency(Hz)                      | 8000000
  *-----------------------------------------------------------------------------
  *        PLL_M                                  | 10
  *-----------------------------------------------------------------------------
  *        PLL_N                                  | 420
  *-----------------------------------------------------------------------------
  *        PLL_P                                  | 2
  *-----------------------------------------------------------------------------
  *        PLL_Q                                  | 7
  *-----------------------------------------------------------------------------
  *        PLLI2S_N                               | NA
  *-----------------------------------------------------------------------------
  *        PLLI2S_R                               | NA
  *-----------------------------------------------------------------------------
  *        I2S input clock                        | NA
  *-----------------------------------------------------------------------------
  *        VDD(V)                                 | 3.3
  *-----------------------------------------------------------------------------
  *        Main regulator output voltage          | Scale1 mode
  *-----------------------------------------------------------------------------
  *        Flash Latency(WS)                      | 5
  *-----------------------------------------------------------------------------
  *        Prefetch Buffer                        | ON
  *-----------------------------------------------------------------------------
  *        Instruction cache                      | ON
  *-----------------------------------------------------------------------------
  *        Data cache                             | ON
  *-----------------------------------------------------------------------------
  *        Require 48MHz for USB OTG FS,          | Disabled
  *        SDIO and RNG clock                     |
  *-----------------------------------------------------------------------------
  *=============================================================================
  *=============================================================================
  *                    Supported STM32F42xxx/43xxx devices
  *-----------------------------------------------------------------------------
  *        System Clock source                    | PLL (HSE)
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 180000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 180000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 4
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 2
  *-----------------------------------------------------------------------------
  *        HSE Frequency(Hz)                      | 25000000
  *-----------------------------------------------------------------------------
  *        PLL_M                                  | 25
  *-----------------------------------------------------------------------------
  *        PLL_N                                  | 360
  *-----------------------------------------------------------------------------
  *        PLL_P                                  | 2
  *-----------------------------------------------------------------------------
  *        PLL_Q                                  | 7
  *-----------------------------------------------------------------------------
  *        PLLI2S_N                               | NA
  *-----------------------------------------------------------------------------
  *        PLLI2S_R                               | NA
  *-----------------------------------------------------------------------------
  *        I2S input clock                        | NA
  *-----------------------------------------------------------------------------
  *        VDD(V)                                 | 3.3
  *-----------------------------------------------------------------------------
  *        Main regulator output voltage          | Scale1 mode
  *-----------------------------------------------------------------------------
  *        Flash Latency(WS)                      | 5
  *-----------------------------------------------------------------------------
  *        Prefetch Buffer                        | ON
  *-----------------------------------------------------------------------------
  *        Instruction cache                      | ON
  *-----------------------------------------------------------------------------
  *        Data cache                             | ON
  *-----------------------------------------------------------------------------
  *        Require 48MHz for USB OTG FS,          | Disabled
  *        SDIO and RNG clock                     |
  *-----------------------------------------------------------------------------
  *=============================================================================
  *=============================================================================
  *                         Supported STM32F401xx devices
  *-----------------------------------------------------------------------------
  *        System Clock source                    | PLL (HSE)
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 84000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 84000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 2
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        HSE Frequency(Hz)                      | 25000000
  *-----------------------------------------------------------------------------
  *        PLL_M                                  | 25
  *-----------------------------------------------------------------------------
  *        PLL_N                                  | 336
  *-----------------------------------------------------------------------------
  *        PLL_P                                  | 4
  *-----------------------------------------------------------------------------
  *        PLL_Q                                  | 7
  *-----------------------------------------------------------------------------
  *        PLLI2S_N                               | NA
  *-----------------------------------------------------------------------------
  *        PLLI2S_R                               | NA
  *-----------------------------------------------------------------------------
  *        I2S input clock                        | NA
  *-----------------------------------------------------------------------------
  *        VDD(V)                                 | 3.3
  *-----------------------------------------------------------------------------
  *        Main regulator output voltage          | Scale1 mode
  *-----------------------------------------------------------------------------
  *        Flash Latency(WS)                      | 2
  *-----------------------------------------------------------------------------
  *        Prefetch Buffer                        | ON
  *-----------------------------------------------------------------------------
  *        Instruction cache                      | ON
  *-----------------------------------------------------------------------------
  *        Data cache                             | ON
  *-----------------------------------------------------------------------------
  *        Require 48MHz for USB OTG FS,          | Disabled
  *        SDIO and RNG clock                     |
  *-----------------------------------------------------------------------------
  *=============================================================================
  *=============================================================================
  *                Supported STM32F411xx/STM32F410xx devices
  *-----------------------------------------------------------------------------
  *        System Clock source                    | PLL (HSI)
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 100000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 100000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 2
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        HSI Frequency(Hz)                      | 16000000
  *-----------------------------------------------------------------------------
  *        PLL_M                                  | 16
  *-----------------------------------------------------------------------------
  *        PLL_N                                  | 400
  *-----------------------------------------------------------------------------
  *        PLL_P                                  | 4
  *-----------------------------------------------------------------------------
  *        PLL_Q                                  | 7
  *-----------------------------------------------------------------------------
  *        PLLI2S_N                               | NA
  *-----------------------------------------------------------------------------
  *        PLLI2S_R                               | NA
  *-----------------------------------------------------------------------------
  *        I2S input clock                        | NA
  *-----------------------------------------------------------------------------
  *        VDD(V)                                 | 3.3
  *-----------------------------------------------------------------------------
  *        Main regulator output voltage          | Scale1 mode
  *-----------------------------------------------------------------------------
  *        Flash Latency(WS)                      | 3
  *-----------------------------------------------------------------------------
  *        Prefetch Buffer                        | ON
  *-----------------------------------------------------------------------------
  *        Instruction cache                      | ON
  *-----------------------------------------------------------------------------
  *        Data cache                             | ON
  *-----------------------------------------------------------------------------
  *        Require 48MHz for USB OTG FS,          | Disabled
  *        SDIO and RNG clock                     |
  *-----------------------------------------------------------------------------
  *=============================================================================
  *=============================================================================
  *                         Supported STM32F446xx devices
  *-----------------------------------------------------------------------------
  *        System Clock source                    | PLL (HSE)
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 180000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 180000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 4
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 2
  *-----------------------------------------------------------------------------
  *        HSE Frequency(Hz)                      | 8000000
  *-----------------------------------------------------------------------------
  *        PLL_M                                  | 8
  *-----------------------------------------------------------------------------
  *        PLL_N                                  | 360
  *-----------------------------------------------------------------------------
  *        PLL_P                                  | 2
  *-----------------------------------------------------------------------------
  *        PLL_Q                                  | 7
  *-----------------------------------------------------------------------------
  *        PLL_R                                  | NA
  *-----------------------------------------------------------------------------
  *        PLLI2S_M                               | NA
  *-----------------------------------------------------------------------------
  *        PLLI2S_N                               | NA
  *-----------------------------------------------------------------------------
  *        PLLI2S_P                               | NA
  *-----------------------------------------------------------------------------
  *        PLLI2S_Q                               | NA
  *-----------------------------------------------------------------------------
  *        PLLI2S_R                               | NA
  *-----------------------------------------------------------------------------
  *        I2S input clock                        | NA
  *-----------------------------------------------------------------------------
  *        VDD(V)                                 | 3.3
  *-----------------------------------------------------------------------------
  *        Main regulator output voltage          | Scale1 mode
  *-----------------------------------------------------------------------------
  *        Flash Latency(WS)                      | 5
  *-----------------------------------------------------------------------------
  *        Prefetch Buffer                        | ON
  *-----------------------------------------------------------------------------
  *        Instruction cache                      | ON
  *-----------------------------------------------------------------------------
  *        Data cache                             | ON
  *-----------------------------------------------------------------------------
  *        Require 48MHz for USB OTG FS,          | Disabled
  *        SDIO and RNG clock                     |
  *-----------------------------------------------------------------------------
  *=============================================================================
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f4xx_system
  * @{
  */

/** @addtogroup STM32F4xx_System_Private_Includes
  * @{
  */

#include <string.h>
#include "stm32f4xx.h"
#include "drivers/system.h"
#include "system_stm32f4xx.h"
#include "platform.h"
#include "drivers/persistent.h"


/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to use external SRAM or SDRAM mounted
     on STM324xG_EVAL/STM324x7I_EVAL/STM324x9I_EVAL boards as data memory  */
#if defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F469_479xx)
/* #define DATA_IN_ExtSRAM */
#endif /* STM32F40_41xxx || STM32F427_437x || STM32F429_439xx || STM32F469_479xx */

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
/* #define DATA_IN_ExtSDRAM */
#endif /* STM32F427_437x || STM32F429_439xx || STM32F446xx || STM32F469_479xx */

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
#define VECT_TAB_SRAM
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */

__I uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
  * @{
  */

void SetSysClock(void);

#if defined(DATA_IN_ExtSRAM) || defined(DATA_IN_ExtSDRAM)
static void SystemInit_ExtMemCtl(void);
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the
  *         SystemFrequency variable.
  * @param  None
  * @retval None
  */

uint32_t SystemCoreClock;
uint32_t pll_src, pll_input, pll_m, pll_p, pll_n, pll_q;

// SystemSYSCLKSource
// 0: HSI
// 1; HSE
// 2: PLLP
// 3: PLLR (F446 only)

int SystemSYSCLKSource(void)
{
    return (RCC->CFGR & RCC_CFGR_SWS) >> 2;
}

// SystemPLLSource
//   0: HSI
//   1: HSE

int SystemPLLSource(void)
{
    return (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
}

typedef struct pllConfig_s {
  uint16_t mhz; // target SYSCLK
  uint16_t n;
  uint16_t p;
  uint16_t q;
} pllConfig_t;

// PLL parameters for PLL input = 1MHz.
// For PLL input = 2MHz, divide n by 2; see SystemInitPLLParameters below.

static const pllConfig_t overclockLevels[] = {
#if defined(STM32F40_41xxx)
  { 168, 336, 2,  7 },  // 168 MHz
  { 192, 384, 2,  8 },  // 192 MHz
  { 216, 432, 2,  9 },  // 216 MHz
  { 240, 480, 2,  10 }  // 240 MHz
#elif defined(STM32F411xE)
  {  96, 384, 4,  8 },  // 96 MHz
  { 108, 432, 4,  9 },  // 108 MHz
  { 120, 480, 4, 10 },  // 120 MHz
#elif defined(STM32F446xx)
  // Main PLL for F446 is not constrained by USB clock generation,
  // as we generate it with PLLSAI.
  // Here, for the moment, we start with default 180MHz and increment in steps of 24MHz.
  // May be made variable in steps of 1MHz in the future...
  { 180, 360, 2,  2 },  // 180 MHz
  { 202, 404, 2,  2 },  // 202 MHz
  { 226, 452, 2,  2 },  // 226 MHz
  { 250, 500, 2,  2 },  // 250 MHz, operation not verified
#endif
};

#if defined(STM32F446xx)
#define PLL_R      7 // PLL_R output is not used, can be any descent number
#endif

void SystemInitPLLParameters(void)
{
    /* PLL setting for overclocking */

    uint32_t currentOverclockLevel = persistentObjectRead(PERSISTENT_OBJECT_OVERCLOCK_LEVEL);

    if (currentOverclockLevel >= ARRAYLEN(overclockLevels)) {
      return;
    }

    const pllConfig_t * const pll = overclockLevels + currentOverclockLevel;

    pll_n = pll->n / pll_input;
    pll_p = pll->p;
    pll_q = pll->q;
}

void OverclockRebootIfNecessary(uint32_t overclockLevel)
{
  if (overclockLevel >= ARRAYLEN(overclockLevels)) {
    return;
  }

  const pllConfig_t * const pll = overclockLevels + overclockLevel;

  // Reboot to adjust overclock frequency
  if (SystemCoreClock != pll->mhz * 1000000U) {
    persistentObjectWrite(PERSISTENT_OBJECT_OVERCLOCK_LEVEL, overclockLevel);
    __disable_irq();
    NVIC_SystemReset();
  }
}

void systemClockSetHSEValue(uint32_t frequency)
{
    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);

    if (hse_value != frequency) {
        persistentObjectWrite(PERSISTENT_OBJECT_HSE_VALUE, frequency);
        __disable_irq();
        NVIC_SystemReset();
    }
}

void SystemInit(void)
{
  initialiseMemorySections();

  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

#if defined(DATA_IN_ExtSRAM) || defined(DATA_IN_ExtSDRAM)
  SystemInit_ExtMemCtl();
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

  /* Configure the System clock source, PLL Multiplier and Divider factors,
     AHB/APBx prescalers and Flash settings ----------------------------------*/
  //SetSysClock();

  /* Configure the Vector Table location add offset address ------------------*/

#ifdef VECT_TAB_SRAM
  // Copy vector table from isr_vector_table_flash_base to isr_vector_table_base.
  // If these two regions are the same, the copy will have no effect
  // (Happens when linker script aliases VECTAB to FLASH).

  extern uint8_t isr_vector_table_flash_base;
  extern uint8_t isr_vector_table_base;
  extern uint8_t isr_vector_table_end;

  memcpy(&isr_vector_table_base, &isr_vector_table_flash_base, &isr_vector_table_end - &isr_vector_table_base);
  SCB->VTOR = (uint32_t)&isr_vector_table_base;
#else
  SCB->VTOR = (uint32_t)&isr_vector_table_flash_base;
#endif

#ifdef USE_HAL_DRIVER
    HAL_Init();
#endif

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
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**)
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (*) HSI_VALUE is a constant defined in stm32f4xx.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSE_VALUE is a constant defined in stm32f4xx.h file (default value
  *              25 MHz), user has to ensure that HSE_VALUE is same as the real
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

  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
#if defined(STM32F446xx)
  uint32_t pllr = 2;
#endif /* STM32F446xx */
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock source */
      SystemCoreClock = hse_value;
      break;
    case 0x08:  /* PLL P used as system clock source */
       /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        pllvco = (hse_value / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      SystemCoreClock = pllvco/pllp;
      break;
#if defined(STM32F446xx)
      case 0x0C:  /* PLL R used as system clock source */
       /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_R
         */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        pllvco = (hse_value / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >>28) + 1 ) *2;
      SystemCoreClock = pllvco/pllr;
      break;
#endif /* STM32F446xx */
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
}

static int StartHSx(uint32_t onBit, uint32_t readyBit, int maxWaitCount)
{
    RCC->CR |= onBit;
    for (int waitCounter = 0 ; waitCounter < maxWaitCount ; waitCounter++) {
        if (RCC->CR & readyBit) {
            return 1;
        }
    }
    return 0;
}

/**
  * @brief  Configures the System clock source, PLL Multiplier and Divider factors,
  *         AHB/APBx prescalers and Flash settings
  * @Note   This function should be called only once the RCC clock configuration
  *         is reset to the default reset state (done in SystemInit() function).
  * @param  None
  * @retval None
  */
void SetSysClock(void)
{
    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);
    uint32_t hse_mhz = hse_value / 1000000;

    // Switch to HSI during clock manipulation

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_HSI);

    // We want to use 2MHz input to PLL, as it will provide greater
    // flexibility in choice of PLL_N and compatible with generation
    // of 48MHz for USB requirement at the same time.
    //
    // Here, if the frequency (in MHz) is multiples of 2, then pll_m is
    // set to a value that derives 2MHz as input to PLL.
    // Otherwise, pll_m is set to the frequency (in MHz) to derive
    // 1MHz as input to PLL.

    if (hse_value == 0) {
        // HSE frequency unknown; use PLL with HSI as source
        if (!StartHSx(RCC_CR_HSION, RCC_CR_HSIRDY, 5000)) {
            return;
        }

        pll_src = RCC_PLLCFGR_PLLSRC_HSI;

        // HSI is fixed at 16MHz.
        pll_m = 8;
        pll_input = 2;
    } else {
        // HSE frequency is given.

        if (!StartHSx(RCC_CR_HSEON, RCC_CR_HSERDY, 5000)) {
            return;
        }

        pll_src = RCC_PLLCFGR_PLLSRC_HSE;

        pll_m = hse_mhz / 2;
        if (pll_m * 2 != hse_mhz) {
            pll_m = hse_mhz;
        }
        pll_input = hse_mhz / pll_m;
    }

    SystemInitPLLParameters();

    /* Select regulator voltage output Scale 1 mode */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    /* HCLK = SYSCLK / 1*/
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

#if defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
    /* PCLK2 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    /* PCLK1 = HCLK / 4*/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
#endif /* STM32F40_41xxx || STM32F427_437x || STM32F429_439xx  || STM32F446xx || STM32F469_479xx */

#if defined(STM32F401xx)
    /* PCLK2 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    /* PCLK1 = HCLK / 4*/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
#endif /* STM32F401xx */

#if defined(STM32F410xx) || defined(STM32F411xE)
    /* PCLK2 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    /* PCLK1 = HCLK / 4*/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
#endif /* STM32F410xx || STM32F411xE */

#if defined(STM32F446xx)
    /* Configure the main PLL */
    RCC->PLLCFGR = pll_m | (pll_n << 6) | (((pll_p >> 1) -1) << 16) |
                   (pll_src) | (pll_q << 24) | (PLL_R << 28);
#else
    /* Configure the main PLL */
    RCC->PLLCFGR = pll_m | (pll_n << 6) | (((pll_p >> 1) -1) << 16) |
                   (pll_src) | (pll_q << 24);
#endif /* STM32F446xx */

    /* Enable the main PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till the main PLL is ready */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
    /* Enable the Over-drive to extend the clock frequency to 180 Mhz */
    PWR->CR |= PWR_CR_ODEN;
    while ((PWR->CSR & PWR_CSR_ODRDY) == 0)
    {
    }
    PWR->CR |= PWR_CR_ODSWEN;
    while ((PWR->CSR & PWR_CSR_ODSWRDY) == 0)
    {
    }
#endif /* STM32F427_437x || STM32F429_439xx || STM32F446xx || STM32F469_479xx */

#if defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
#endif /* STM32F40_41xxx || STM32F427_437x || STM32F429_439xx  || STM32F446xx || STM32F469_479xx */

#if defined(STM32F401xx) || defined(STM32F410xx) || defined(STM32F411xE)
    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_2WS;
#endif /* STM32F401xx || STM32F410xx || STM32F411xE*/

    /* Select the main PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait till the main PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);

#if defined(STM32F446xx)
// Always use PLLSAI to derive USB 48MHz clock.
// - This also works under arbitral overclocking situations.
// - Only handles HSE case.

uint32_t pllsai_m;

#ifdef TARGET_XTAL_MHZ
#define PLLSAI_M      TARGET_XTAL_MHZ
#else
#define PLLSAI_M      8
#endif
#define PLLSAI_N      192
#define PLLSAI_P      4
#define PLLSAI_Q      2

#define RCC_PLLSAI_IS_READY() ((RCC->CR & (RCC_CR_PLLSAIRDY)) == (RCC_CR_PLLSAIRDY))

    // Scale PLLSAI input to 1MHz.
    if (hse_value) {
        pllsai_m = hse_value / 1000000U;
    } else {
        pllsai_m = 16;
    }

    // Set 48MHz clock source
    RCC_48MHzClockSourceConfig(RCC_48MHZCLKSource_PLLSAI);

    // Enable PLLSAI
    RCC_PLLSAICmd(DISABLE);

    // wait for PLLSAI to be disabled
    while (RCC_PLLSAI_IS_READY()) {}

    RCC_PLLSAIConfig(pllsai_m, PLLSAI_N, PLLSAI_P, PLLSAI_Q);

    RCC_PLLSAICmd(ENABLE);

    // wait for PLLSAI to be enabled
    while (!RCC_PLLSAI_IS_READY()) {}

    RCC->DCKCFGR2 |= RCC_DCKCFGR2_CK48MSEL;

#undef  RCC_PLLSAI_GET_FLAG
#endif /* STM32F446xx */

    // Configure PLLI2S for 27MHz operation
    // Use pll_input (1 or 2) to derive multiplier N for
    // 108MHz (27 * 4) PLLI2SCLK with R divider fixed at 2.
    // 108MHz will further be prescaled by 4 by mcoInit.

#define PLLI2S_TARGET_FREQ_MHZ (27 * 4)
#define PLLI2S_R               2

    uint32_t plli2s_n = (PLLI2S_TARGET_FREQ_MHZ * PLLI2S_R) / pll_input;

#ifdef STM32F40_41xxx
    RCC_PLLI2SConfig(plli2s_n, PLLI2S_R);
#elif defined(STM32F411xE)
    RCC_PLLI2SConfig(plli2s_n, PLLI2S_R, pll_m);
#elif defined(STM32F446xx)
    RCC_PLLI2SConfig(pll_m, plli2s_n, 2, 2, PLLI2S_R); // M, N, P, Q, R
#else
#error Unsupported MCU
#endif

    RCC_PLLI2SCmd(ENABLE);

    SystemCoreClockUpdate();
}

/**
  * @brief  Setup the external memory controller. Called in startup_stm32f4xx.s
  *          before jump to __main
  * @param  None
  * @retval None
  */
#ifdef DATA_IN_ExtSRAM
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f4xx.s before jump to main.
  *         This function configures the external SRAM mounted on STM324xG_EVAL/STM324x7I boards
  *         This SRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
/*-- GPIOs Configuration -----------------------------------------------------*/
/*
 +-------------------+--------------------+------------------+--------------+
 +                       SRAM pins assignment                               +
 +-------------------+--------------------+------------------+--------------+
 | PD0  <-> FMC_D2  | PE0  <-> FMC_NBL0 | PF0  <-> FMC_A0 | PG0 <-> FMC_A10 |
 | PD1  <-> FMC_D3  | PE1  <-> FMC_NBL1 | PF1  <-> FMC_A1 | PG1 <-> FMC_A11 |
 | PD4  <-> FMC_NOE | PE3  <-> FMC_A19  | PF2  <-> FMC_A2 | PG2 <-> FMC_A12 |
 | PD5  <-> FMC_NWE | PE4  <-> FMC_A20  | PF3  <-> FMC_A3 | PG3 <-> FMC_A13 |
 | PD8  <-> FMC_D13 | PE7  <-> FMC_D4   | PF4  <-> FMC_A4 | PG4 <-> FMC_A14 |
 | PD9  <-> FMC_D14 | PE8  <-> FMC_D5   | PF5  <-> FMC_A5 | PG5 <-> FMC_A15 |
 | PD10 <-> FMC_D15 | PE9  <-> FMC_D6   | PF12 <-> FMC_A6 | PG9 <-> FMC_NE2 |
 | PD11 <-> FMC_A16 | PE10 <-> FMC_D7   | PF13 <-> FMC_A7 |-----------------+
 | PD12 <-> FMC_A17 | PE11 <-> FMC_D8   | PF14 <-> FMC_A8 |
 | PD13 <-> FMC_A18 | PE12 <-> FMC_D9   | PF15 <-> FMC_A9 |
 | PD14 <-> FMC_D0  | PE13 <-> FMC_D10  |-----------------+
 | PD15 <-> FMC_D1  | PE14 <-> FMC_D11  |
 |                  | PE15 <-> FMC_D12  |
 +------------------+------------------+
*/
   /* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
  RCC->AHB1ENR   |= 0x00000078;

  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x00cc00cc;
  GPIOD->AFR[1]  = 0xcccccccc;
  /* Configure PDx pins in Alternate function mode */
  GPIOD->MODER   = 0xaaaa0a0a;
  /* Configure PDx pins speed to 100 MHz */
  GPIOD->OSPEEDR = 0xffff0f0f;
  /* Configure PDx pins Output type to push-pull */
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xcccccccc;
  GPIOE->AFR[1]  = 0xcccccccc;
  /* Configure PEx pins in Alternate function mode */
  GPIOE->MODER   = 0xaaaaaaaa;
  /* Configure PEx pins speed to 100 MHz */
  GPIOE->OSPEEDR = 0xffffffff;
  /* Configure PEx pins Output type to push-pull */
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0x00cccccc;
  GPIOF->AFR[1]  = 0xcccc0000;
  /* Configure PFx pins in Alternate function mode */
  GPIOF->MODER   = 0xaa000aaa;
  /* Configure PFx pins speed to 100 MHz */
  GPIOF->OSPEEDR = 0xff000fff;
  /* Configure PFx pins Output type to push-pull */
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0x00cccccc;
  GPIOG->AFR[1]  = 0x000000c0;
  /* Configure PGx pins in Alternate function mode */
  GPIOG->MODER   = 0x00080aaa;
  /* Configure PGx pins speed to 100 MHz */
  GPIOG->OSPEEDR = 0x000c0fff;
  /* Configure PGx pins Output type to push-pull */
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */
  GPIOG->PUPDR   = 0x00000000;

/*-- FMC Configuration ------------------------------------------------------*/
  /* Enable the FMC/FSMC interface clock */
  RCC->AHB3ENR         |= 0x00000001;

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427_437xx || STM32F429_439xx || STM32F446xx || STM32F469_479xx */

#if defined(STM32F40_41xxx)
  /* Configure and enable Bank1_SRAM2 */
  FSMC_Bank1->BTCR[2]  = 0x00001011;
  FSMC_Bank1->BTCR[3]  = 0x00000201;
  FSMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif  /* STM32F40_41xxx */

/*
  Bank1_SRAM2 is configured as follow:
  In case of FSMC configuration
  NORSRAMTimingStructure.FSMC_AddressSetupTime = 1;
  NORSRAMTimingStructure.FSMC_AddressHoldTime = 0;
  NORSRAMTimingStructure.FSMC_DataSetupTime = 2;
  NORSRAMTimingStructure.FSMC_BusTurnAroundDuration = 0;
  NORSRAMTimingStructure.FSMC_CLKDivision = 0;
  NORSRAMTimingStructure.FSMC_DataLatency = 0;
  NORSRAMTimingStructure.FSMC_AccessMode = FMC_AccessMode_A;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &NORSRAMTimingStructure;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &NORSRAMTimingStructure;

  In case of FMC configuration
  NORSRAMTimingStructure.FMC_AddressSetupTime = 1;
  NORSRAMTimingStructure.FMC_AddressHoldTime = 0;
  NORSRAMTimingStructure.FMC_DataSetupTime = 2;
  NORSRAMTimingStructure.FMC_BusTurnAroundDuration = 0;
  NORSRAMTimingStructure.FMC_CLKDivision = 0;
  NORSRAMTimingStructure.FMC_DataLatency = 0;
  NORSRAMTimingStructure.FMC_AccessMode = FMC_AccessMode_A;

  FMC_NORSRAMInitStructure.FMC_Bank = FMC_Bank1_NORSRAM2;
  FMC_NORSRAMInitStructure.FMC_DataAddressMux = FMC_DataAddressMux_Disable;
  FMC_NORSRAMInitStructure.FMC_MemoryType = FMC_MemoryType_SRAM;
  FMC_NORSRAMInitStructure.FMC_MemoryDataWidth = FMC_MemoryDataWidth_16b;
  FMC_NORSRAMInitStructure.FMC_BurstAccessMode = FMC_BurstAccessMode_Disable;
  FMC_NORSRAMInitStructure.FMC_AsynchronousWait = FMC_AsynchronousWait_Disable;
  FMC_NORSRAMInitStructure.FMC_WaitSignalPolarity = FMC_WaitSignalPolarity_Low;
  FMC_NORSRAMInitStructure.FMC_WrapMode = FMC_WrapMode_Disable;
  FMC_NORSRAMInitStructure.FMC_WaitSignalActive = FMC_WaitSignalActive_BeforeWaitState;
  FMC_NORSRAMInitStructure.FMC_WriteOperation = FMC_WriteOperation_Enable;
  FMC_NORSRAMInitStructure.FMC_WaitSignal = FMC_WaitSignal_Disable;
  FMC_NORSRAMInitStructure.FMC_ExtendedMode = FMC_ExtendedMode_Disable;
  FMC_NORSRAMInitStructure.FMC_WriteBurst = FMC_WriteBurst_Disable;
  FMC_NORSRAMInitStructure.FMC_ContinousClock = FMC_CClock_SyncOnly;
  FMC_NORSRAMInitStructure.FMC_ReadWriteTimingStruct = &NORSRAMTimingStructure;
  FMC_NORSRAMInitStructure.FMC_WriteTimingStruct = &NORSRAMTimingStructure;
*/

}
#endif /* DATA_IN_ExtSRAM */

#ifdef DATA_IN_ExtSDRAM
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f4xx.s before jump to main.
  *         This function configures the external SDRAM mounted on STM324x9I_EVAL board
  *         This SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register uint32_t index;

  /* Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface
      clock */
  RCC->AHB1ENR |= 0x000001FC;

  /* Connect PCx pins to FMC Alternate function */
  GPIOC->AFR[0]  = 0x0000000c;
  GPIOC->AFR[1]  = 0x00007700;
  /* Configure PCx pins in Alternate function mode */
  GPIOC->MODER   = 0x00a00002;
  /* Configure PCx pins speed to 50 MHz */
  GPIOC->OSPEEDR = 0x00a00002;
  /* Configure PCx pins Output type to push-pull */
  GPIOC->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PCx pins */
  GPIOC->PUPDR   = 0x00500000;

  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x000000CC;
  GPIOD->AFR[1]  = 0xCC000CCC;
  /* Configure PDx pins in Alternate function mode */
  GPIOD->MODER   = 0xA02A000A;
  /* Configure PDx pins speed to 50 MHz */
  GPIOD->OSPEEDR = 0xA02A000A;
  /* Configure PDx pins Output type to push-pull */
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00000CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */
  GPIOE->MODER   = 0xAAAA800A;
  /* Configure PEx pins speed to 50 MHz */
  GPIOE->OSPEEDR = 0xAAAA800A;
  /* Configure PEx pins Output type to push-pull */
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0xcccccccc;
  GPIOF->AFR[1]  = 0xcccccccc;
  /* Configure PFx pins in Alternate function mode */
  GPIOF->MODER   = 0xAA800AAA;
  /* Configure PFx pins speed to 50 MHz */
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* Configure PFx pins Output type to push-pull */
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0xcccccccc;
  GPIOG->AFR[1]  = 0xcccccccc;
  /* Configure PGx pins in Alternate function mode */
  GPIOG->MODER   = 0xaaaaaaaa;
  /* Configure PGx pins speed to 50 MHz */
  GPIOG->OSPEEDR = 0xaaaaaaaa;
  /* Configure PGx pins Output type to push-pull */
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */
  GPIOG->PUPDR   = 0x00000000;

  /* Connect PHx pins to FMC Alternate function */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* Configure PHx pins in Alternate function mode */
  GPIOH->MODER   = 0xAAAA08A0;
  /* Configure PHx pins speed to 50 MHz */
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* Configure PHx pins Output type to push-pull */
  GPIOH->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PHx pins */
  GPIOH->PUPDR   = 0x00000000;

  /* Connect PIx pins to FMC Alternate function */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* Configure PIx pins in Alternate function mode */
  GPIOI->MODER   = 0x0028AAAA;
  /* Configure PIx pins speed to 50 MHz */
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* Configure PIx pins Output type to push-pull */
  GPIOI->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PIx pins */
  GPIOI->PUPDR   = 0x00000000;

/*-- FMC Configuration ------------------------------------------------------*/
  /* Enable the FMC interface clock */
  RCC->AHB3ENR |= 0x00000001;

  /* Configure and enable SDRAM bank1 */
  FMC_Bank5_6->SDCR[0] = 0x000039D0;
  FMC_Bank5_6->SDTR[0] = 0x01115351;

  /* SDRAM initialization sequence */
  /* Clock enable command */
  FMC_Bank5_6->SDCMR = 0x00000011;
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  while ((tmpreg != 0) & (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  }

  /* Delay */
  for (index = 0; index<1000; index++);

  /* PALL command */
  FMC_Bank5_6->SDCMR = 0x00000012;
  timeout = 0xFFFF;
  while ((tmpreg != 0) & (timeout-- > 0))
  {
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  }

  /* Auto refresh command */
  FMC_Bank5_6->SDCMR = 0x00000073;
  timeout = 0xFFFF;
  while ((tmpreg != 0) & (timeout-- > 0))
  {
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  }

  /* MRD register program */
  FMC_Bank5_6->SDCMR = 0x00046014;
  timeout = 0xFFFF;
  while ((tmpreg != 0) & (timeout-- > 0))
  {
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  }

  /* Set refresh count */
  tmpreg = FMC_Bank5_6->SDRTR;
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));

  /* Disable write protection */
  tmpreg = FMC_Bank5_6->SDCR[0];
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);

/*
  Bank1_SDRAM is configured as follow:

  FMC_SDRAMTimingInitStructure.FMC_LoadToActiveDelay = 2;
  FMC_SDRAMTimingInitStructure.FMC_ExitSelfRefreshDelay = 6;
  FMC_SDRAMTimingInitStructure.FMC_SelfRefreshTime = 4;
  FMC_SDRAMTimingInitStructure.FMC_RowCycleDelay = 6;
  FMC_SDRAMTimingInitStructure.FMC_WriteRecoveryTime = 2;
  FMC_SDRAMTimingInitStructure.FMC_RPDelay = 2;
  FMC_SDRAMTimingInitStructure.FMC_RCDDelay = 2;

  FMC_SDRAMInitStructure.FMC_Bank = SDRAM_BANK;
  FMC_SDRAMInitStructure.FMC_ColumnBitsNumber = FMC_ColumnBits_Number_8b;
  FMC_SDRAMInitStructure.FMC_RowBitsNumber = FMC_RowBits_Number_11b;
  FMC_SDRAMInitStructure.FMC_SDMemoryDataWidth = FMC_SDMemory_Width_16b;
  FMC_SDRAMInitStructure.FMC_InternalBankNumber = FMC_InternalBank_Number_4;
  FMC_SDRAMInitStructure.FMC_CASLatency = FMC_CAS_Latency_3;
  FMC_SDRAMInitStructure.FMC_WriteProtection = FMC_Write_Protection_Disable;
  FMC_SDRAMInitStructure.FMC_SDClockPeriod = FMC_SDClock_Period_2;
  FMC_SDRAMInitStructure.FMC_ReadBurst = FMC_Read_Burst_disable;
  FMC_SDRAMInitStructure.FMC_ReadPipeDelay = FMC_ReadPipe_Delay_1;
  FMC_SDRAMInitStructure.FMC_SDRAMTimingStruct = &FMC_SDRAMTimingInitStructure;
*/

}
#endif /* DATA_IN_ExtSDRAM */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
