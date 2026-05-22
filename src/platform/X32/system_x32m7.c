/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

#include "x32m7xx_otpc.h"

#define X32_OTPC_UCID_BASE      0x00000300U
#define X32_OTPC_UID_WORDS      3U
#define X32_OTPC_READ_TIMEOUT   1000000U

uint32_t systemUniqueId[3] = {
    0,
    0,
    0
};

void SystemCoreClockUpdate(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    SystemCoreClock = clocks.M7ClkFreq;
}

void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
    while (1) {
    }
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    switch (requestType) {
    case BOOTLOADER_REQUEST_FLASH:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_FLASH);
        break;

    case BOOTLOADER_REQUEST_ROM:
    default:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);
        break;
    }

    systemReset();
}

void systemProcessResetReason(void)
{
    const uint32_t resetReason = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (resetReason) {
    case RESET_BOOTLOADER_REQUEST_FLASH:
    case RESET_BOOTLOADER_REQUEST_ROM:
        // Keep bootloader requests one-shot until the X32 ROM boot path is wired in.
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_POST);
        break;

    case RESET_BOOTLOADER_POST:
    case RESET_FORCED:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        break;

    case RESET_MSC_REQUEST:
    case RESET_NONE:
    default:
        break;
    }
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    RCC_EnableAHB5PeriphClk1(
        RCC_AHB5_PERIPHEN_M7_GPIOA |
        RCC_AHB5_PERIPHEN_M7_GPIOB |
        RCC_AHB5_PERIPHEN_M7_GPIOC |
        RCC_AHB5_PERIPHEN_M7_GPIOD |
        RCC_AHB5_PERIPHEN_M7_GPIOE |
        RCC_AHB5_PERIPHEN_M7_GPIOF |
        RCC_AHB5_PERIPHEN_M7_GPIOG |
        RCC_AHB5_PERIPHEN_M7_GPIOH,
        ENABLE);

    RCC_EnableAHB5PeriphClk2(
        RCC_AHB5_PERIPHEN_M7_GPIOI |
        RCC_AHB5_PERIPHEN_M7_GPIOJ |
        RCC_AHB5_PERIPHEN_M7_GPIOK |
        RCC_AHB5_PERIPHEN_M7_AFIO,
        ENABLE);
}

bool isMPUSoftReset(void)
{
    return (cachedResetFlags & RCC_CTRLSTS_CM7SFTRSTF) != 0;
}

static void systemUniqueIdSetFallback(void)
{
    systemUniqueId[0] = SCB->CPUID;
    systemUniqueId[1] = 0x5833324dU;
    systemUniqueId[2] = 0x37425549U;
}

static bool systemUniqueIdIsProgrammed(const uint32_t *uid)
{
    return ((uid[0] | uid[1] | uid[2]) != 0U) && ((uid[0] & uid[1] & uid[2]) != 0xffffffffU);
}

static bool systemUniqueIdIsMultiword(const uint32_t *uid)
{
    return (uid[0] != uid[1]) || (uid[1] != uid[2]);
}

static bool x32OtpcWaitReady(void)
{
    for (uint32_t timeout = X32_OTPC_READ_TIMEOUT; timeout > 0U; timeout--) {
        if ((OTPC->STS & OTPC_FLAG_BUSY) == 0U) {
            return (OTPC->STS & OTPC_ALLERROR_STS) == 0U;
        }
    }

    return false;
}

static bool x32OtpcReadWord(uint32_t address, uint32_t *data)
{
    OTPC_ClearFlag(OTPC_ALLERROR_STS);

    if (!x32OtpcWaitReady()) {
        return false;
    }

    OTPC_ReadEnable();
    OTPC_SetAddr(address);

    if (!x32OtpcWaitReady()) {
        return false;
    }

    *data = OTPC->RDATA;
    return true;
}

static void systemUniqueIdInit(void)
{
    uint32_t uid[X32_OTPC_UID_WORDS];
    bool readOk = true;

    systemUniqueIdSetFallback();

    RCC_EnableAXIPeriphClk1(RCC_AXI_PERIPHEN_M7_OTPC, ENABLE);

    OTPC_Unlock();

    uint32_t otpcUsCount = SystemCoreClock / 1000000U;
    if (otpcUsCount > OTPC_USC_MASK) {
        otpcUsCount = OTPC_USC_MASK;
    }
    OTPC_SetUsCount(otpcUsCount);

    for (uint32_t i = 0; i < X32_OTPC_UID_WORDS; i++) {
        readOk = x32OtpcReadWord(X32_OTPC_UCID_BASE + (i * sizeof(uint32_t)), &uid[i]) && readOk;
    }

    OTPC_Lock();

    if (readOk && systemUniqueIdIsProgrammed(uid) && systemUniqueIdIsMultiword(uid)) {
        systemUniqueId[0] = uid[0];
        systemUniqueId[1] = uid[1];
        systemUniqueId[2] = uid[2];
    } else if (readOk && systemUniqueIdIsProgrammed(uid)) {
        systemUniqueId[0] = uid[0];
    }
}


/**
*\*\name    SetSysClockToPLL1.
*\*\fun     Selects PLL as sys_clk source and Configure the M7 clock is 400MHz,
*\*\       M4, AXI, AHB(1,2,5,6,9) clock is 200MHz,
*\*\       APB(1,2,5,6) clock is 100MHz. 
*\*\param   PLL_source(PLL entry clock source):
*\*\   		  - RCC_PLL_SRC_HSI     
*\*\   		  - RCC_PLL_SRC_HSE
*\*\   		  - RCC_PLL_SRC_MSI   
*\*\param   sysclk_freq(sys_clk frequency):
*\*\         200000000 ~ 600000000 (Hz)
*\*\return  ErrorStatus:
 *\*\         - SUCCESS    
 *\*\         - ERROR   
*\*\note    M7 from PLL1A, AXI from PLL1A, M4 from PLL1A.  
**/
ErrorStatus SetSysClockToPLL1(uint32_t PLL_source, uint64_t sysclk_freq)
{
    ErrorStatus ClockStatus;
    uint64_t input_freq, pll_freq;
    uint32_t PLL1A_divider;
    uint32_t timeout_value = 0xFFFFFFFF;
    
    RCC_DeInit();
    
    if(PLL_source == RCC_PLL_SRC_HSE)
    {
        input_freq = 25000000;
        /* Enable HSE */
        RCC_ConfigHse(RCC_HSE_ENABLE);

        /* Wait till HSE is ready */
        ClockStatus = RCC_WaitHseStable();
           
    }
    else if(PLL_source == RCC_PLL_SRC_MSI)
    {
        input_freq = MSI_VALUE;
        /* Enable MSI */
        RCC_EnableMsi(ENABLE);

        /* Wait till MSI is ready */
        ClockStatus = RCC_WaitMsiStable();
           
    }   else
    {
        input_freq = HSI_VALUE;
        /* Enable HSI */
        RCC_EnableHsi(ENABLE);

        /* Wait till HSI is ready */
        ClockStatus = RCC_WaitHsiStable();
    }
    
    if(ClockStatus == SUCCESS)
    {
        /* Configure sys_div_clk is sys_clk(PLL1A) = M7 clock*/
        RCC_ConfigSysclkDivider(RCC_SYSCLK_DIV1);
        /* Configure sys_bus_div_clk is sys_div_clk/2 = M4 clock = AHB1\2\5\9*/
        RCC_ConfigSysbusDivider(RCC_BUSCLK_DIV2);
        /* Configure AXI clock is sys_div_clk/2 = AHB6*/
        RCC_ConfigAXIclkDivider(RCC_AXICLK_DIV2);
        RCC_ConfigAXIHyperDivider(RCC_AXICLK_HYP_DIV2);
        
        // systick_delay_us(1);
        if(sysclk_freq >= 400000000)
        {
            pll_freq = sysclk_freq;
            PLL1A_divider = RCC_PLLA_DIV1;
        }
        else if(sysclk_freq >= (400000000/2))
        {
            pll_freq = sysclk_freq*2;
            PLL1A_divider = RCC_PLLA_DIV2;
        }
        else
        {
            pll_freq = sysclk_freq*3;
            PLL1A_divider = RCC_PLLA_DIV3;
        }
        
        if(sysclk_freq <= 300000000)
        {
            /* Configure APB clock is AHB/1  */
            RCC_ConfigAPBclkDivider(RCC_APB1CLK_DIV1,RCC_APB2CLK_DIV1,RCC_APB5CLK_DIV1,RCC_APB6CLK_DIV1);
        }
        else
        {
            /* Configure APB clock is AHB/2  */
            RCC_ConfigAPBclkDivider(RCC_APB1CLK_DIV2,RCC_APB2CLK_DIV2,RCC_APB5CLK_DIV2,RCC_APB6CLK_DIV2);
        }
        
        /* configure PLL1 source and frequency */
        RCC_ConfigPll1(PLL_source,input_freq,pll_freq,ENABLE);
        
        // systick_delay_us(1);
        
        /* configure PLL1A is PLL1 */
        RCC_ConfigPLL1ADivider(PLL1A_divider);
        
        /* configure sys_clk source is PLL1A */
        RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLL1A);

        /* Check if sys_clk source is PLL1A */
        while(RCC_GetSysclkSrc() != RCC_SYSCLK_STS_PLL1A)
        {
            if ((timeout_value--) == 0)
            {
                return ERROR;
            }
        }

    }
    else
    {
        /* clock source fails to start-up */
        return ERROR;
    }
    return SUCCESS;
    
}

void systemInit(void)
{
    ErrorStatus status;
    persistentObjectInit();
    systemProcessResetReason();

    status = SetSysClockToPLL1(RCC_PLL_SRC_HSE, 600000000);
    if (status != SUCCESS) {
        // Clock configuration failed - halt or fall back
        while (1) {
        }
    }

    SystemCoreClockUpdate();
    //USE for SD Card
    RCC_ConfigPll2(RCC_PLL_SRC_HSI,64000000,500000000,ENABLE);
    /* configure PLL1A is PLL1 */
    RCC_ConfigPLL2ADivider(RCC_PLLA_DIV5);

    NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);

    cachedResetFlags = RCC->CTRLSTS;

#ifdef VECT_TAB_SRAM
    extern uint8_t isr_vector_table_base;
    SCB->VTOR = (uint32_t)&isr_vector_table_base;
#endif
    
    RCC_ClearResetFlag();

    enableGPIOPowerUsageAndNoiseReductions();

    systemUniqueIdInit();

    cycleCounterInit();

    SysTick_Config(SystemCoreClock / 1000);
}
