/**
 *
 * @file        system_apm32f4xx.c
 *
 * @brief       CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
 *
 * @version     V1.0.0
 *
 * @date        2023-07-31
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup apm32f4xx_system
  * @{
  */  
  
/** @addtogroup APM32F4xx_System_Private_Includes
  * @{
  */

#include "apm32f4xx.h"
#include "system_apm32f4xx.h"
#include "platform.h"
#include "drivers/system.h"
#include "drivers/persistent.h"

#include <string.h>

/* Value of the external oscillator in Hz */
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)8000000U)
#endif /* HSE_VALUE */

/* Value of the internal oscillator in Hz */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U)
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_Defines
  * @{
  */
/* Uncomment the following line if you need to relocate your vector table in internal SRAM */
/* #define VECT_TAB_SRAM */

/* Vector table base offset field. This value must be a multiple of 0x200 */
#define VECT_TAB_OFFSET  0x00

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_Macros
  * @{
  */
#define PLLI2S_TARGET_FREQ_MHZ (27 * 4)
#define PLLI2S_R               2

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_Variables
  * @{
  */
uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

uint32_t pll_src, pll_input, pll_m, pll_p, pll_n, pll_q;

typedef struct pllConfig_s {
  uint16_t mhz; // target SYSCLK
  uint16_t n;
  uint16_t p;
  uint16_t q;
} pllConfig_t;

// PLL parameters for PLL input = 1MHz.
// For PLL input = 2MHz, divide n by 2; see SystemInitPLLParameters below.

static const pllConfig_t overclockLevels[] = {
  { 168, 336, 2,  7 },  // 168 MHz
  { 192, 384, 2,  8 },  // 192 MHz
  { 216, 432, 2,  9 },  // 216 MHz
  { 240, 480, 2,  10 }  // 240 MHz
};

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_Functions
  * @{
  */

static void SystemInitPLLParameters(void);
void DAL_SysClkConfig(void);
void DAL_ErrorHandler(void);

/**
 * @brief     Setup the microcontroller system
 *
 * @param     None
 *
 * @retval    None
 */
void SystemInit(void)
{
    initialiseMemorySections();

    /* FPU settings */
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
      SCB->CPACR |= ((3UL << 10U * 2U)|(3UL << 11U * 2U));  /* set CP10 and CP11 Full Access */
#endif

    /* Reset the RCM clock configuration to the default reset state */
    /* Set HSIEN bit */
    RCM->CTRL |= (uint32_t)0x00000001;

    /* Reset CFG register */
    RCM->CFG = 0x00000000;

    /* Reset HSEEN, CSSEN and PLL1EN bits */
    RCM->CTRL &= (uint32_t)0xFEF6FFFF;

    /* Reset PLL1CFG register */
    RCM->PLL1CFG = 0x24003010;

    /* Reset HSEBCFG bit */
    RCM->CTRL &= (uint32_t)0xFFFBFFFF;

    /* Disable all interrupts */
    RCM->INT = 0x00000000;

    /* Configure the Vector Table location add offset address */
    extern uint8_t isr_vector_table_flash_base;
    extern uint8_t isr_vector_table_base;
    extern uint8_t isr_vector_table_end;

    if (&isr_vector_table_base != &isr_vector_table_flash_base) {
        memcpy(&isr_vector_table_base, &isr_vector_table_flash_base, (size_t) (&isr_vector_table_end - &isr_vector_table_base));
    }

    SCB->VTOR = (uint32_t)&isr_vector_table_base;
}

/**
   * @brief Update SystemCoreClock variable according to clock register values
 *          The SystemCoreClock variable contains the core clock (HCLK)
  *     
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
    uint32_t sysClock = 0, pllvco = 0, pllc, pllClock, pllb;

    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);

    /* Get SYSCLK source */
    sysClock = RCM->CFG & RCM_CFG_SCLKSWSTS;

    switch (sysClock)
    {
        case 0x00:  /* HSI used as system clock source */
            SystemCoreClock = HSI_VALUE;
            break;

        case 0x04:  /* HSE used as system clock source */
            SystemCoreClock = hse_value;
            break;

        case 0x08:  /* PLL used as system clock source */
            pllClock = (RCM->PLL1CFG & RCM_PLL1CFG_PLL1CLKS) >> 22;
            pllb = RCM->PLL1CFG & RCM_PLL1CFG_PLLB;
            
            if (pllClock != 0)
            {
                /* HSE used as PLL clock source */
                pllvco = (hse_value / pllb) * ((RCM->PLL1CFG & RCM_PLL1CFG_PLL1A) >> 6);
            }
            else
            {
                /* HSI used as PLL clock source */
                pllvco = (HSI_VALUE / pllb) * ((RCM->PLL1CFG & RCM_PLL1CFG_PLL1A) >> 6);
            }

            pllc = (((RCM->PLL1CFG & RCM_PLL1CFG_PLL1C) >> 16) + 1 ) * 2;
            SystemCoreClock = pllvco / pllc;
            break;

        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }

    /* Compute HCLK frequency --------------------------------------------------*/
    /* Get HCLK prescaler */
    sysClock = AHBPrescTable[((RCM->CFG & RCM_CFG_AHBPSC) >> 4)];
    /* HCLK frequency */
    SystemCoreClock >>= sysClock;
}

/**
 * @brief   System clock configuration
 *
 * @param   None
 *
 * @retval  None
 */
void DAL_SysClkConfig(void)
{
    RCM_ClkInitTypeDef RCM_ClkInitStruct = {0U};
    RCM_OscInitTypeDef RCM_OscInitStruct = {0U};
    RCM_PeriphCLKInitTypeDef PeriphClk_InitStruct = {0U};

    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);
    uint32_t hse_mhz = hse_value / 1000000;

    if (hse_value == 0) {
        pll_src = RCM_PLLSOURCE_HSI;

        // HSI is fixed at 16MHz
        pll_m = 8;
        pll_input = 2;
    }
    else {
        // HSE frequency is given
        pll_src = RCM_PLLSOURCE_HSE;
        
        pll_m = hse_mhz /  2;
        if (pll_m * 2 != hse_mhz) {
            pll_m = hse_mhz;
        }
        pll_input = hse_mhz / pll_m;
    }

    SystemInitPLLParameters();

    /* Enable PMU clock */
    __DAL_RCM_PMU_CLK_ENABLE();

    /* Configure the voltage scaling value */
    __DAL_PMU_VOLTAGESCALING_CONFIG(PMU_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCM_OscInitStruct.OscillatorType    = RCM_OSCILLATORTYPE_HSI | RCM_OSCILLATORTYPE_HSE | RCM_OSCILLATORTYPE_LSI;
    RCM_OscInitStruct.HSEState          = RCM_HSE_ON;
    RCM_OscInitStruct.HSIState          = RCM_HSI_ON;
    RCM_OscInitStruct.LSIState          = RCM_LSI_ON;
    RCM_OscInitStruct.PLL.PLLState      = RCM_PLL_ON;
    RCM_OscInitStruct.PLL.PLLSource     = pll_src;
    RCM_OscInitStruct.PLL.PLLB          = pll_m;
    RCM_OscInitStruct.PLL.PLL1A         = pll_n;
    RCM_OscInitStruct.PLL.PLL1C         = pll_p;
    RCM_OscInitStruct.PLL.PLLD          = pll_q;
    RCM_OscInitStruct.HSICalibrationValue = 0x10;
    if(DAL_RCM_OscConfig(&RCM_OscInitStruct) != DAL_OK)
    {
        DAL_ErrorHandler();
    }

    /* Configure clock */
    RCM_ClkInitStruct.ClockType         = (RCM_CLOCKTYPE_SYSCLK | RCM_CLOCKTYPE_HCLK | RCM_CLOCKTYPE_PCLK1 | RCM_CLOCKTYPE_PCLK2);
    RCM_ClkInitStruct.SYSCLKSource      = RCM_SYSCLKSOURCE_PLLCLK;
    RCM_ClkInitStruct.AHBCLKDivider     = RCM_SYSCLK_DIV1;
    RCM_ClkInitStruct.APB1CLKDivider    = RCM_HCLK_DIV4;  
    RCM_ClkInitStruct.APB2CLKDivider    = RCM_HCLK_DIV2;  
    if(DAL_RCM_ClockConfig(&RCM_ClkInitStruct, FLASH_LATENCY_5) != DAL_OK)
    {
        DAL_ErrorHandler();
    }

    /* I2S clock */
    // Configure PLLI2S for 27MHz operation
    // Use pll_input (1 or 2) to derive multiplier N for
    // 108MHz (27 * 4) PLLI2SCLK with R divider fixed at 2.
    // 108MHz will further be prescaled by 4 by mcoInit.

    uint32_t plli2s_n = (PLLI2S_TARGET_FREQ_MHZ * PLLI2S_R) / pll_input;

    PeriphClk_InitStruct.PeriphClockSelection   = RCM_PERIPHCLK_I2S;
    PeriphClk_InitStruct.PLLI2S.PLL2A           = plli2s_n;
    PeriphClk_InitStruct.PLLI2S.PLL2C           = PLLI2S_R;
    if (DAL_RCMEx_PeriphCLKConfig(&PeriphClk_InitStruct) != DAL_OK)
    {
        DAL_ErrorHandler();
    }
}

/**
 * @brief     Error handler
 *
 * @param     None
 *
 * @retval    None
 */
void DAL_ErrorHandler(void)
{
    /* When the function is needed, this function 
       could be implemented in the user file
    */
    while(1)
    {
    }
}

void AssertFailedHandler(uint8_t *file, uint32_t line)
{ 
    /* When the function is needed, this function 
       could be implemented in the user file
    */
    UNUSED(file);
    UNUSED(line);
    while(1)
    {
    }
}

/**
 * @brief     Get the SYSCLK source
 *
 * @param     None
 *
 * @retval    The SYSCLK source
 *           - 0x00: HSI used as system clock source
 *           - 0x01: HSE used as system clock source
 *           - 0x02: PLL used as system clock source
 */
int SystemSYSCLKSource(void)
{
    return (int)((RCM->CFG & RCM_CFG_SCLKSWSTS) >> 2);
}

/**
 * @brief     Get the PLL source
 *
 * @param     None
 *
 * @retval    The PLL source
 *           - 0x00: HSI used as PLL clock source
 *           - 0x01: HSE used as PLL clock source
 */
int SystemPLLSource(void)
{
    return (int)((RCM->PLL1CFG & RCM_PLL1CFG_PLL1CLKS) >> 22);
}

/**
 * @brief Reboot the system if necessary after changing the overclock level
 * 
 * @param overclockLevel 
 * 
 * @retval None
 */
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

/**
 * @brief Set the HSE value
 * 
 * @param frequency 
 * 
 * @retval None
 */
void systemClockSetHSEValue(uint32_t frequency)
{
    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);

    if (hse_value != frequency) {
        persistentObjectWrite(PERSISTENT_OBJECT_HSE_VALUE, frequency);
        __disable_irq();
        NVIC_SystemReset();
    }
}

/**
 * @brief    Initialize the PLL parameters
 * 
 * @param    None
 * 
 * @retval   None
 * 
 */
static void SystemInitPLLParameters(void)
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

/**
  * @}
  */

/**
  * @}
  */
  
/**
  * @}
  */    
