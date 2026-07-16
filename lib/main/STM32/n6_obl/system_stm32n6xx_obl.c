/*
 * CMSIS system file for the Betaflight N6 OpenBootloader.
 *
 * Built with -mcmse so __ARM_FEATURE_CMSE=3 and the CMSIS headers resolve
 * every peripheral pointer (RCC, RIFSC, GPIOx, DBGMCU, TAMP, PWR, RTC,
 * EXTI, GPDMA1, HPDMA1, RNG, SYSCFG, ...) to its secure-alias base. OBL
 * is the only S-state code on the device, so all of its peripheral
 * accesses must go through the secure aliases.
 *
 * SystemInit opens every relevant slave + master so the NS application
 * can reach RAM, XSPI memory-mapped flash and AHB/APB peripherals after
 * the BXNS hand-off in jump_to_bf().
 *
 * SystemClock_Config_Impl is called from main() to lift the CPU from
 * the boot-ROM hand-off clock (HSI/4 ≈ 16 MHz) to 400 MHz so USB OTG_HS
 * gets a clean SOF stream when the OBL DFU branch runs.
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

    /* Debug subsystem clock. */
    RCC->MISCENSR = RCC_MISCENSR_DBGENS;
    (void)RCC->MISCENR;

    /* APB3 bus clock — DBGMCU lives on APB3 and accesses there read 0 /
     * silently drop until the bus is ungated. */
    RCC->BUSENSR = RCC_BUSENSR_APB3ENS;
    (void)RCC->BUSENR;

    /* RIFSC: every peripheral slave NS+Unpriv, debug-AP master CID 0. */
    for (uint32_t i = 0; i < 6U; i++) {
        RIFSC->RISC_SECCFGRx[i]  = 0;
        RIFSC->RISC_PRIVCFGRx[i] = 0;
    }
    RIFSC->RIMC_CR &= ~0x7UL;
    (void)RIFSC->RIMC_CR;

    /* DBGMCU.CR: hold the M55 debuggable across run / sleep / stop /
     * standby so SWD attach to running user code works on OPEN-lifecycle
     * silicon. */
    DBGMCU->CR = DBGMCU_CR_DBGCLKEN
               | DBGMCU_CR_DBG_SLEEP
               | DBGMCU_CR_DBG_STOP
               | DBGMCU_CR_DBG_STANDBY;
    (void)DBGMCU->CR;

    /* TAMP is RIF-aware: its security/priv gating lives in its own
     * SECCFGR/PRIVCFGR (not RIFSC). Opening to NS+Unpriv exposes all
     * 32 BKPxR for read/write from NS world. */
    TAMP->SECCFGR  = 0;
    TAMP->PRIVCFGR = 0;
    (void)TAMP->SECCFGR;

    /* GPIO SECCFGR + PRIVCFGR per port — every pin NS + Unpriv. Without
     * these the NS application's pinmux for SPI / I2C / UART / etc. is
     * silently rejected even with RIFSC opened. The bank-enable on
     * AHB4ENSR has to happen first: writes to a GPIO bank with its
     * clock gated off silently drop on the N6, so the SECCFGR loop
     * below would no-op for any port the boot ROM hadn't already
     * ungated. */
    RCC->AHB4ENSR = RCC_AHB4ENR_GPIOAEN_Msk | RCC_AHB4ENR_GPIOBEN_Msk
                  | RCC_AHB4ENR_GPIOCEN_Msk | RCC_AHB4ENR_GPIODEN_Msk
                  | RCC_AHB4ENR_GPIOEEN_Msk | RCC_AHB4ENR_GPIOFEN_Msk
                  | RCC_AHB4ENR_GPIOGEN_Msk | RCC_AHB4ENR_GPIOHEN_Msk
                  | RCC_AHB4ENR_GPIONEN_Msk | RCC_AHB4ENR_GPIOOEN_Msk
                  | RCC_AHB4ENR_GPIOPEN_Msk | RCC_AHB4ENR_GPIOQEN_Msk;
    (void)RCC->AHB4ENR;

    static GPIO_TypeDef * const gpio_banks[] = {
        GPIOA, GPIOB, GPIOC, GPIOD,
        GPIOE, GPIOF, GPIOG, GPIOH,
        GPION, GPIOO, GPIOP, GPIOQ,
    };
    for (unsigned i = 0; i < sizeof(gpio_banks) / sizeof(gpio_banks[0]); i++) {
        gpio_banks[i]->SECCFGR  = 0;
        gpio_banks[i]->PRIVCFGR = 0;
    }

    /* GPDMA1 + HPDMA1: clock-enable first (boot ROM hands them off
     * ungated) then open per-controller SECCFGR + PRIVCFGR. SECCFGR is
     * a bit-per-channel register at controller +0x00; writing 0 routes
     * every channel under NS attribution. */
    RCC->AHB1ENSR = RCC_AHB1ENSR_GPDMA1ENS;
    RCC->AHB5ENSR = RCC_AHB5ENSR_HPDMA1ENS;
    GPDMA1->SECCFGR  = 0;
    GPDMA1->PRIVCFGR = 0;
    HPDMA1->SECCFGR  = 0;
    HPDMA1->PRIVCFGR = 0;

    /* Per-peripheral SECCFGR + PRIVCFGR — peripherals carrying their
     * own security config above RIFSC. Clearing each = NS + Unpriv. */
    RCC->SECCFGR0  = 0;
    RCC->SECCFGR1  = 0;
    RCC->SECCFGR2  = 0;
    RCC->SECCFGR3  = 0;
    RCC->SECCFGR4  = 0;
    PWR->SECCFGR   = 0;
    PWR->PRIVCFGR  = 0;
    EXTI->SECCFGR1  = 0;
    EXTI->PRIVCFGR1 = 0;
    EXTI->SECCFGR2  = 0;
    EXTI->PRIVCFGR2 = 0;
    EXTI->SECCFGR3  = 0;
    EXTI->PRIVCFGR3 = 0;
    RTC->SECCFGR   = 0;

    /* RIMC master attributes — every bus master (CPU, DMAs, USB, ETH, …)
     * presents as NS + Unpriv + CID 0 by default. */
    for (unsigned i = 0; i < 13U; i++) {
        RIFSC->RIMC_ATTRx[i] = 0;
    }

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
    RCC->APB4ENR2 |=  0x00000010UL;
    (void)RCC->APB4ENR2;
    RCC->APB4ENR2 &= ~0x00000010UL;

    /* LSI on. IWDG is clocked from LSI exclusively; without this the
     * watchdog hardware never ticks and a wedged BF never resets back
     * into OBL recovery. Boot ROM doesn't reliably leave LSI on. */
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
    SCB->CPACR |= ((3UL << 20U) | (3UL << 22U));   /* CP10/CP11 full S access */
#endif

    /* NSACR — bits 10/11 grant NS access to CP10/CP11 (FPU + MVE).
     * 0x0FFF is the broadest "NS can touch every implemented CP" set;
     * the harmless bits 0..7 hand future-proof access. NSACR is
     * Secure-only-writable so it has to be set here, before the BXNS
     * hand-off. (AIRCR.BFHFNMINS is deferred to jump_to_bf — flipping it
     * here routes any S-side fault during OBL's own remaining execution
     * to an NS state that doesn't yet have a vector table or stack,
     * which deterministically wedges the chip into LOCKUP.) */
    SCB->NSACR = 0x00000FFFUL;
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
