/*
 * CMSIS system file for the FSBL stub. The stub does not build with
 * -mcmse and never transitions to non-secure state, so the usual
 * TrustZone bring-up (SAU region clear, SCB_NS->CPACR, CMSE_NS_ENTRY
 * exports) is omitted.
 */

#include "stm32n6xx.h"
#include <math.h>

#if !defined(HSE_VALUE)
#define HSE_VALUE      48000000UL
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE      64000000UL
#endif
#if !defined(MSI_VALUE)
#define MSI_VALUE       4000000UL
#endif
#if !defined(EXTERNAL_I2S_CLOCK_VALUE)
#define EXTERNAL_I2S_CLOCK_VALUE  12288000UL
#endif

uint32_t SystemCoreClock = HSI_VALUE;

extern void *g_pfnVectors;
#define INTVECT_START ((uint32_t)&g_pfnVectors)

void SystemInit(void)
{
    SCB->VTOR = INTVECT_START;

    /* Hold the debug subsystem clock on first thing so SWD is attachable
     * even if a later init step faults. The boot ROM closes AP1 on
     * BOOT0=USER paths; DBGCLKEN reopens it on OPEN-lifecycle silicon
     * without going through Debug Authentication. DBG_SLEEP/STOP/STANDBY
     * keep the core debuggable across low-power transitions too. */
    RCC->MISCENSR = RCC_MISCENSR_DBGENS;
    (void)RCC->MISCENR;
    DBGMCU->CR |= DBGMCU_CR_DBGCLKEN
                | DBGMCU_CR_DBG_SLEEP
                | DBGMCU_CR_DBG_STOP
                | DBGMCU_CR_DBG_STANDBY;
    (void)DBGMCU->CR;

    /* RNG reset + clock-disable. The boot ROM may have left it ticking. */
    RCC->AHB3RSTSR = RCC_AHB3RSTSR_RNGRSTS;
    RCC->AHB3RSTCR = RCC_AHB3RSTCR_RNGRSTC;
    RCC->AHB3ENCR  = RCC_AHB3ENCR_RNGENC;

    /* SYSCFG clock + VDDIOx supply rails (errata ES0620). */
    RCC->APB4ENSR2 = RCC_APB4ENSR2_SYSCFGENS;
    (void)RCC->APB4ENR2;

    SYSCFG->INITSVTORCR = SCB->VTOR;

    PWR->SVMCR1 |= PWR_SVMCR1_VDDIO4SV;
    PWR->SVMCR2 |= PWR_SVMCR2_VDDIO5SV;
    PWR->SVMCR3 |= PWR_SVMCR3_VDDIO2SV | PWR_SVMCR3_VDDIO3SV;

    SYSCFG->VDDIO2CCCR = 0x00000287UL;
    SYSCFG->VDDIO3CCCR = 0x00000287UL;
    SYSCFG->VDDIO4CCCR = 0x00000287UL;
    SYSCFG->VDDIO5CCCR = 0x00000287UL;
    SYSCFG->VDDCCCR    = 0x00000287UL;

    /* VDDADC clamp + VREF buffer. */
    PWR->SVMCR3 |= PWR_SVMCR3_ASV;
    PWR->SVMCR3 |= PWR_SVMCR3_AVMEN;
    (void)PWR->SVMCR3;
    RCC->APB4ENR1 |= RCC_APB4ENR1_VREFBUFEN;

    /* RCC fix per ST template — pulse a reserved bit to lower power. */
    RCC->APB4ENR2 |= 0x00000010UL;
    (void)RCC->APB4ENR2;
    RCC->APB4ENR2 &= ~(0x00000010UL);

    /* Reset XSPI2 + XSPIM so we start from a known state regardless of how
     * boot ROM left them after loading us. */
    RCC->AHB5RSTSR = RCC_AHB5RSTSR_XSPIMRSTS | RCC_AHB5RSTSR_XSPI2RSTS;
    RCC->AHB5RSTCR = RCC_AHB5RSTCR_XSPIMRSTC | RCC_AHB5RSTCR_XSPI2RSTC;

    /* TIM2 reset + clock-disable. */
    RCC->APB1RSTSR1 = RCC_APB1RSTSR1_TIM2RSTS;
    RCC->APB1RSTCR1 = RCC_APB1RSTCR1_TIM2RSTC;
    RCC->APB1ENCR1  = RCC_APB1ENCR1_TIM2ENC;

    /* Boot ROM left GPIOG clocked; we'll re-enable in xspi MSP if needed. */
    RCC->AHB4ENCR = RCC_AHB4ENCR_GPIOGENC;

    /* Enable AXISRAM1..6 clocks. Boot ROM only clocks AXISRAM2; the rest
     * (notably AXISRAM1 where the app's stack and .text live) must be
     * brought up here or the first store from the app's Reset_Handler
     * busfaults silently. */
    RCC->MEMENSR = RCC_MEMENSR_AXISRAM1ENS | RCC_MEMENSR_AXISRAM2ENS
                 | RCC_MEMENSR_AXISRAM3ENS | RCC_MEMENSR_AXISRAM4ENS
                 | RCC_MEMENSR_AXISRAM5ENS | RCC_MEMENSR_AXISRAM6ENS;
    (void)RCC->MEMENR;

    (void)SYSCFG->INITSVTORCR;
    RCC->APB4ENCR2 = RCC_APB4ENCR2_SYSCFGENC;

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 20U) | (3UL << 22U));   /* CP10/CP11 full access */
#endif
}

void SystemCoreClockUpdate(void)
{
    uint32_t sysclk = 0;
    uint32_t pllm = 0;
    uint32_t plln = 0;
    uint32_t pllfracn = 0;
    uint32_t pllp1 = 0;
    uint32_t pllp2 = 0;
    uint32_t pllcfgr;
    uint32_t pllsource = 0;
    uint32_t pllbypass = 0;
    uint32_t ic_divider;
    float_t pllvco;

    switch (RCC->CFGR1 & RCC_CFGR1_CPUSWS) {
    case 0:
        sysclk = HSI_VALUE >> ((RCC->HSICFGR & RCC_HSICFGR_HSIDIV) >> RCC_HSICFGR_HSIDIV_Pos);
        break;

    case RCC_CFGR1_CPUSWS_0:
        sysclk = (READ_BIT(RCC->MSICFGR, RCC_MSICFGR_MSIFREQSEL) == 0UL) ? MSI_VALUE : 16000000UL;
        break;

    case RCC_CFGR1_CPUSWS_1:
        sysclk = HSE_VALUE;
        break;

    case (RCC_CFGR1_CPUSWS_1 | RCC_CFGR1_CPUSWS_0):
        switch (READ_BIT(RCC->IC1CFGR, RCC_IC1CFGR_IC1SEL)) {
        case 0:
            pllcfgr = READ_REG(RCC->PLL1CFGR1);
            pllsource = pllcfgr & RCC_PLL1CFGR1_PLL1SEL;
            pllbypass = pllcfgr & RCC_PLL1CFGR1_PLL1BYP;
            if (pllbypass == 0U) {
                pllm = (pllcfgr & RCC_PLL1CFGR1_PLL1DIVM) >> RCC_PLL1CFGR1_PLL1DIVM_Pos;
                plln = (pllcfgr & RCC_PLL1CFGR1_PLL1DIVN) >> RCC_PLL1CFGR1_PLL1DIVN_Pos;
                pllfracn = READ_BIT(RCC->PLL1CFGR2, RCC_PLL1CFGR2_PLL1DIVNFRAC) >> RCC_PLL1CFGR2_PLL1DIVNFRAC_Pos;
                pllcfgr = READ_REG(RCC->PLL1CFGR3);
                pllp1 = (pllcfgr & RCC_PLL1CFGR3_PLL1PDIV1) >> RCC_PLL1CFGR3_PLL1PDIV1_Pos;
                pllp2 = (pllcfgr & RCC_PLL1CFGR3_PLL1PDIV2) >> RCC_PLL1CFGR3_PLL1PDIV2_Pos;
            }
            break;
        case RCC_IC1CFGR_IC1SEL_0:
            pllcfgr = READ_REG(RCC->PLL2CFGR1);
            pllsource = pllcfgr & RCC_PLL2CFGR1_PLL2SEL;
            pllbypass = pllcfgr & RCC_PLL2CFGR1_PLL2BYP;
            if (pllbypass == 0U) {
                pllm = (pllcfgr & RCC_PLL2CFGR1_PLL2DIVM) >> RCC_PLL2CFGR1_PLL2DIVM_Pos;
                plln = (pllcfgr & RCC_PLL2CFGR1_PLL2DIVN) >> RCC_PLL2CFGR1_PLL2DIVN_Pos;
                pllfracn = READ_BIT(RCC->PLL2CFGR2, RCC_PLL2CFGR2_PLL2DIVNFRAC) >> RCC_PLL2CFGR2_PLL2DIVNFRAC_Pos;
                pllcfgr = READ_REG(RCC->PLL2CFGR3);
                pllp1 = (pllcfgr & RCC_PLL2CFGR3_PLL2PDIV1) >> RCC_PLL2CFGR3_PLL2PDIV1_Pos;
                pllp2 = (pllcfgr & RCC_PLL2CFGR3_PLL2PDIV2) >> RCC_PLL2CFGR3_PLL2PDIV2_Pos;
            }
            break;
        case RCC_IC1CFGR_IC1SEL_1:
            pllcfgr = READ_REG(RCC->PLL3CFGR1);
            pllsource = pllcfgr & RCC_PLL3CFGR1_PLL3SEL;
            pllbypass = pllcfgr & RCC_PLL3CFGR1_PLL3BYP;
            if (pllbypass == 0U) {
                pllm = (pllcfgr & RCC_PLL3CFGR1_PLL3DIVM) >> RCC_PLL3CFGR1_PLL3DIVM_Pos;
                plln = (pllcfgr & RCC_PLL3CFGR1_PLL3DIVN) >> RCC_PLL3CFGR1_PLL3DIVN_Pos;
                pllfracn = READ_BIT(RCC->PLL3CFGR2, RCC_PLL3CFGR2_PLL3DIVNFRAC) >> RCC_PLL3CFGR2_PLL3DIVNFRAC_Pos;
                pllcfgr = READ_REG(RCC->PLL3CFGR3);
                pllp1 = (pllcfgr & RCC_PLL3CFGR3_PLL3PDIV1) >> RCC_PLL3CFGR3_PLL3PDIV1_Pos;
                pllp2 = (pllcfgr & RCC_PLL3CFGR3_PLL3PDIV2) >> RCC_PLL3CFGR3_PLL3PDIV2_Pos;
            }
            break;
        default:
            pllcfgr = READ_REG(RCC->PLL4CFGR1);
            pllsource = pllcfgr & RCC_PLL4CFGR1_PLL4SEL;
            pllbypass = pllcfgr & RCC_PLL4CFGR1_PLL4BYP;
            if (pllbypass == 0U) {
                pllm = (pllcfgr & RCC_PLL4CFGR1_PLL4DIVM) >> RCC_PLL4CFGR1_PLL4DIVM_Pos;
                plln = (pllcfgr & RCC_PLL4CFGR1_PLL4DIVN) >> RCC_PLL4CFGR1_PLL4DIVN_Pos;
                pllfracn = READ_BIT(RCC->PLL4CFGR2, RCC_PLL4CFGR2_PLL4DIVNFRAC) >> RCC_PLL4CFGR2_PLL4DIVNFRAC_Pos;
                pllcfgr = READ_REG(RCC->PLL4CFGR3);
                pllp1 = (pllcfgr & RCC_PLL4CFGR3_PLL4PDIV1) >> RCC_PLL4CFGR3_PLL4PDIV1_Pos;
                pllp2 = (pllcfgr & RCC_PLL4CFGR3_PLL4PDIV2) >> RCC_PLL4CFGR3_PLL4PDIV2_Pos;
            }
            break;
        }

        switch (pllsource) {
        case 0:
            sysclk = HSI_VALUE >> ((RCC->HSICFGR & RCC_HSICFGR_HSIDIV) >> RCC_HSICFGR_HSIDIV_Pos);
            break;
        case RCC_PLL1CFGR1_PLL1SEL_0:
            sysclk = (READ_BIT(RCC->MSICFGR, RCC_MSICFGR_MSIFREQSEL) == 0UL) ? MSI_VALUE : 16000000UL;
            break;
        case RCC_PLL1CFGR1_PLL1SEL_1:
            sysclk = HSE_VALUE;
            break;
        case (RCC_PLL1CFGR1_PLL1SEL_1 | RCC_PLL1CFGR1_PLL1SEL_0):
            sysclk = EXTERNAL_I2S_CLOCK_VALUE;
            break;
        default:
            break;
        }

        if (pllbypass == 0U) {
            pllvco = ((float_t)sysclk * ((float_t)plln + ((float_t)pllfracn / (float_t)0x1000000UL))) / (float_t)pllm;
            sysclk = (uint32_t)((float_t)(pllvco / (((float_t)pllp1) * ((float_t)pllp2))));
        }
        ic_divider = (READ_BIT(RCC->IC1CFGR, RCC_IC1CFGR_IC1INT) >> RCC_IC1CFGR_IC1INT_Pos) + 1UL;
        sysclk = sysclk / ic_divider;
        break;

    default:
        break;
    }

    SystemCoreClock = sysclk;
}
