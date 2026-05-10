/*
 * CMSIS system file for the Betaflight N6 OpenBootloader. Mirrors the
 * FSBL stub's bring-up (debug clock + VDDIO supplies + AXISRAM1..6
 * enable + XSPI2 reset) and adds SystemClock_Config_Impl, called from
 * main() after HAL_Init to crank the chip to a clock fast enough for
 * USB OTG_HS at FS speeds. Boot ROM hands us off at HSI/4 ≈ 16 MHz
 * which is fine for HAL_Init but too slow for the USB SOF stream.
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

    /* Caches stay off throughout OBL: the boot decision reads XSPI flash
     * and we want fresh fetches; BF re-enables caches in its own startup. */
    SCB_DisableICache();
    SCB_InvalidateICache();
    SCB_DisableDCache();
    SCB_InvalidateDCache();

    /* Debug subsystem clock — RCC_S->MISCENSR.DBGENS at 0x56028A48 bit 0.
     * Hardcoded secure-alias address; the CMSIS RCC macro resolves to the
     * NS alias in this no-mcmse build and an NS-tagged AHB transaction
     * gets gated to DBGMCU/TAMP/RIFSC. */
    *(volatile uint32_t *)0x56028A48UL = 1UL;
    (void)*(volatile uint32_t *)0x56028248UL;

    /* APB3 bus clock — DBGMCU is at APB3+0x1000 (0x54001000 secure /
     * 0x44001000 NS); without APB3 ungated all loads/stores to that
     * window read 0 / drop. RCC_S->BUSENSR.APB3ENS at 0x56028A44 bit 10. */
    *(volatile uint32_t *)0x56028A44UL = (1UL << 10);
    (void)*(volatile uint32_t *)0x56028244UL;

    /* RIFSC: every peripheral slave NS+Unpriv, debug-AP master CID 0.
     *   RISC_SECCFGRx[0..5]  at 0x54024010..0x54024024
     *   RISC_PRIVCFGRx[0..5] at 0x54024030..0x54024044
     *   RIMC_CR              at 0x54024C00 */
    {
        volatile uint32_t * const seccfg  = (volatile uint32_t *)0x54024010UL;
        volatile uint32_t * const privcfg = (volatile uint32_t *)0x54024030UL;
        for (uint32_t i = 0; i < 6U; i++) {
            seccfg[i]  = 0;
            privcfg[i] = 0;
        }
        volatile uint32_t * const rimc_cr = (volatile uint32_t *)0x54024C00UL;
        *rimc_cr &= ~0x7UL;
        (void)*rimc_cr;
    }

    /* DBGMCU.CR: hold the M55 debuggable across run / sleep / stop /
     * standby so SWD attach to running user code works on OPEN-lifecycle
     * silicon. DBGMCU_S->CR at 0x54001004; bit 20 DBGCLKEN, bits 0/1/2
     * DBG_SLEEP / DBG_STOP / DBG_STANDBY. NS-alias mirror written too
     * because RIFSC tagging follows the address alias. */
    {
        const uint32_t cr_val = (1UL << 20)
                              | (1UL <<  0)
                              | (1UL <<  1)
                              | (1UL <<  2);
        *(volatile uint32_t *)0x54001004UL = cr_val;
        (void)*(volatile uint32_t *)0x54001004UL;
        *(volatile uint32_t *)0x44001004UL = cr_val;
        (void)*(volatile uint32_t *)0x44001004UL;
    }

    /* TAMP is RIF-aware: its security/priv gating lives in its own
     * SECCFGR/PRIVCFGR (not RIFSC). Opening to NS+Unpriv exposes all
     * 32 BKPxR for read/write from NS world. */
    *(volatile uint32_t *)0x56004420UL = 0;
    *(volatile uint32_t *)0x56004424UL = 0;
    (void)*(volatile uint32_t *)0x56004420UL;

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

    /* Pulse APB4ENR2 bit 4 — required by ST's reference bring-up to
     * lower power; no documented register name. */
    RCC->APB4ENR2 |= 0x00000010UL;
    (void)RCC->APB4ENR2;
    RCC->APB4ENR2 &= ~(0x00000010UL);

    /* LSI on. IWDG is clocked from LSI exclusively; without this the
     * watchdog hardware never ticks and a wedged BF never resets back
     * into OBL recovery. Boot ROM doesn't reliably leave LSI on. N6 RCC
     * uses set/clear registers: CSR.LSIONS sets, SR.LSIRDY reads ready. */
    RCC->CSR = RCC_CSR_LSIONS;
    while ((RCC->SR & RCC_SR_LSIRDY) == 0U) {
        ;
    }

    /* Reset XSPI2 + XSPIM so we start from a known state regardless of how
     * boot ROM left them after loading us. */
    RCC->AHB5RSTSR = RCC_AHB5RSTSR_XSPIMRSTS | RCC_AHB5RSTSR_XSPI2RSTS;
    RCC->AHB5RSTCR = RCC_AHB5RSTCR_XSPIMRSTC | RCC_AHB5RSTCR_XSPI2RSTC;

    /* TIM2 reset + clock-disable. */
    RCC->APB1RSTSR1 = RCC_APB1RSTSR1_TIM2RSTS;
    RCC->APB1RSTCR1 = RCC_APB1RSTCR1_TIM2RSTC;
    RCC->APB1ENCR1  = RCC_APB1ENCR1_TIM2ENC;

    /* Enable AXISRAM1..6 clocks. Boot ROM only clocks AXISRAM2; the rest
     * must be brought up here or the first store from a consumer of
     * those banks busfaults silently. */
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

/*
 * SystemClock_Config_Impl — bring the CPU up to a USB-friendly speed.
 *
 * Lifted from CubeN6's OBL reference (Projects/STM32N6570-DK/.../OBL/
 * Core/Src/main.c::SystemClock_Config). Targets:
 *   PLL1 VCO    = HSE 48 MHz / PLLM 3 * PLLN 50 = 800 MHz
 *   PLL1 output = VCO / (PLLP1 1 * PLLP2 1) = 800 MHz
 *   CPU         = IC1 /2 = 400 MHz
 *   SYSCLK      = IC2/IC6/IC11 /2 = 400 MHz (AXI / NPU / AXISRAM3..6)
 *   HCLK        = SYSCLK /2 = 200 MHz
 *   PCLK1..5    = HCLK / 1 = 200 MHz
 *   XSPI2 kclk  = IC3 = PLL1 / 24 = 33 MHz (sufficient for indirect-mode
 *                 program / erase; XIP reads happen via the same kclk
 *                 once memory-mapped mode is on)
 *   USB1_OTG_HS = HSE_DIRECT / 2 (handled in usbd_conf.c::HAL_PCD_MspInit)
 *   USBPHYC_CR.FSEL = 0b010 (24 MHz) — set in HAL_PCD_MspInit
 *
 * Called from main() after HAL_Init. The boot ROM hands us off at HSI/4
 * (~16 MHz CPU) which is enough for HAL_Init but too slow for USB SOFs.
 */
void SystemClock_Config_Impl(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef pclk = {0};

    /* Boot ROM may hand off with PLL1 already running (it cranks PLL1
     * for fast XSPI staging on the FSBL-load path). HAL_RCC_OscConfig
     * refuses to reconfigure PLL1 while it's the active source, so
     * we tear the clock tree back to defaults first. Caches must be
     * disabled across the teardown — boot ROM leaves stale lines, and
     * an instruction-prefetch refill during the CPU-clock switch from
     * PLL1 to HSI bus-faults and escalates to LOCKUP. */
    SCB_DisableICache();
    SCB_DisableDCache();
    System_DeInit();

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSIDiv              = RCC_HSI_DIV1;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.HSEState            = RCC_HSE_ON;

    osc.PLL1.PLLState       = RCC_PLL_ON;
    osc.PLL1.PLLSource      = RCC_PLLSOURCE_HSE;
    osc.PLL1.PLLM           = 3;
    osc.PLL1.PLLN           = 50;
    osc.PLL1.PLLP1          = 1;
    osc.PLL1.PLLP2          = 1;
    osc.PLL1.PLLFractional  = 0;

    osc.PLL2.PLLState = RCC_PLL_OFF;
    osc.PLL3.PLLState = RCC_PLL_OFF;
    osc.PLL4.PLLState = RCC_PLL_OFF;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1) {}
    }

    clk.ClockType = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1
                  | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK4
                  | RCC_CLOCKTYPE_PCLK5;
    clk.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;

    clk.IC1Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL1;
    clk.IC1Selection.ClockDivider    = 2;
    clk.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
    clk.IC11Selection.ClockDivider   = 2;
    clk.IC2Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL1;
    clk.IC2Selection.ClockDivider    = 2;
    clk.IC6Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL1;
    clk.IC6Selection.ClockDivider    = 2;

    clk.AHBCLKDivider  = RCC_HCLK_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV1;
    clk.APB2CLKDivider = RCC_APB2_DIV1;
    clk.APB4CLKDivider = RCC_APB4_DIV1;
    clk.APB5CLKDivider = RCC_APB5_DIV1;

    if (HAL_RCC_ClockConfig(&clk) != HAL_OK) {
        while (1) {}
    }

    /* XSPI2 kernel clock from IC3 = PLL1 / 24 = 33 MHz. Slow vs the
     * 133 MHz the chip can handle, but reliable for command-mode
     * programming and we don't need XIP throughput in OBL. */
    pclk.PeriphClockSelection                = RCC_PERIPHCLK_XSPI2;
    pclk.Xspi2ClockSelection                 = RCC_XSPI2CLKSOURCE_IC3;
    pclk.ICSelection[RCC_IC3].ClockSelection = RCC_ICCLKSOURCE_PLL1;
    pclk.ICSelection[RCC_IC3].ClockDivider   = 24;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) {
        while (1) {}
    }
    __HAL_RCC_XSPI2_CLK_ENABLE();

    SystemCoreClockUpdate();
}
