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
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

// System core clock variable required by CMSIS and HAL
uint32_t SystemCoreClock = 64000000; // Default to HSI frequency

void SystemCoreClockUpdate(void)
{
    // Compute CPU frequency directly from RCC registers. HAL_RCC_GetCpuClockFreq
    // returned a stale HSI-based value on the CubeN6 v1.3.0 drop after the
    // CPU was successfully switched to IC1/PLL1 (post-switch the CFGR1.CPUSWS
    // bits read as IC1 and the PLL1 lock + IC1 enable bits all confirm the
    // switch, but the function still returned HSI frequency). Read the live
    // PLL1 + IC1 + HSI dividers instead.
    const uint32_t cpusws = (RCC->CFGR1 & RCC_CFGR1_CPUSWS) >> RCC_CFGR1_CPUSWS_Pos;
    switch (cpusws) {
    case 0: { // HSI (with HSIDIV)
        const uint32_t hsidiv = (RCC->HSICFGR & RCC_HSICFGR_HSIDIV) >> RCC_HSICFGR_HSIDIV_Pos;
        SystemCoreClock = HSI_VALUE >> hsidiv;
        break;
    }
    case 2: // HSE
        SystemCoreClock = HSE_VALUE;
        break;
    case 3: { // IC1
        const uint32_t ic1cfgr = RCC->IC1CFGR;
        const uint32_t ic_div = ((ic1cfgr & RCC_IC1CFGR_IC1INT) >> RCC_IC1CFGR_IC1INT_Pos) + 1U;
        const uint32_t ic_sel = (ic1cfgr & RCC_IC1CFGR_IC1SEL) >> RCC_IC1CFGR_IC1SEL_Pos;
        const uint32_t pllcfgr1 = (ic_sel == 0U) ? RCC->PLL1CFGR1 :
                                  (ic_sel == 1U) ? RCC->PLL2CFGR1 :
                                  (ic_sel == 2U) ? RCC->PLL3CFGR1 : RCC->PLL4CFGR1;
        const uint32_t pll_src  = (pllcfgr1 & RCC_PLL1CFGR1_PLL1SEL) >> RCC_PLL1CFGR1_PLL1SEL_Pos;
        const uint32_t pll_n    = (pllcfgr1 & RCC_PLL1CFGR1_PLL1DIVN) >> RCC_PLL1CFGR1_PLL1DIVN_Pos;
        const uint32_t pll_m    = (pllcfgr1 & RCC_PLL1CFGR1_PLL1DIVM) >> RCC_PLL1CFGR1_PLL1DIVM_Pos;
        // Source: 0=HSI, 1=MSI, 2=HSE (we only spin PLLs from HSE in the
        // betaflight bring-up; other paths fall back to HSI).
        const uint32_t pll_in   = (pll_src == 2U) ? HSE_VALUE : HSI_VALUE;
        if (pll_m != 0U) {
            SystemCoreClock = (pll_in / pll_m) * pll_n / ic_div;
        }
        break;
    }
    default:
        // MSI (case 1) and any unexpected encoding — leave as last good value.
        break;
    }
}

// Minimal _init for C runtime
void _init(void) {}

// PLL1 VCO target. 1200 MHz produces clean integer PLLN dividers for the HSE
// crystal frequencies that NUCLEO and DK boards ship with: 8 / 16 / 24 / 25 /
// 48 MHz → PLLN of 150 / 75 / 50 / 48 / 25. With IC1 divider 2 the CPU clock
// lands at 600 MHz, matching ST's reference example projects.
#define N6_PLL1_VCO_HZ                  1200000000U

static void SystemClock_Config(void);

bool isMPUSoftReset(void)
{
    if (cachedResetFlags & RCC_RSR_SFTRSTF)
        return true;
    else
        return false;
}

static bool memoryMappedModeEnabledOnBoot = false;

bool isMemoryMappedModeEnabledOnBoot(void)
{
    return memoryMappedModeEnabledOnBoot;
}

void memoryMappedModeInit(void)
{
#if defined(USE_OCTOSPI) && defined(FLASH_OCTOSPI_INSTANCE)
    // The XSPI2 peripheral clock has to be live before we touch its CR. On a
    // production LRUN boot the ROM has already enabled it; on a RAM-only debug
    // load (firmware loaded by gdb directly into AXISRAM) we may be jumping in
    // before the ROM completes, so the read would otherwise stall the AHB bus
    // indefinitely (defeats both bus-fault recovery and `halt`).
    __HAL_RCC_XSPI2_CLK_ENABLE();
    memoryMappedModeEnabledOnBoot = READ_BIT(FLASH_OCTOSPI_INSTANCE->CR, XSPI_CR_FMODE) == XSPI_CR_FMODE;
#endif
}

extern uint32_t isr_vector_table_base;

void systemInit(void)
{
    // Debug subsystem clock + APB3 bus clock + DBGMCU.CR all configured
    // via the secure alias. RCC and DBGMCU are RIFSC-gated and an
    // NS-tagged transaction (which the CMSIS RCC->... and DBGMCU->...
    // macros emit in this no-mcmse build) gets dropped silently — the
    // bits read back as not set, the debug clock never enables, and
    // SWD attach to a running BF fails AP1 examination ("Failed to
    // read memory at 0xE000ED00"). Use the same secure-alias addresses
    // OBL writes (system_stm32n6xx_obl.c).
    //   RCC_S->MISCENSR  @ 0x56028A48 bit 0  — DBGENS
    //   RCC_S->BUSENSR   @ 0x56028A44 bit 10 — APB3ENS
    //   DBGMCU_S->CR     @ 0x54001004        — DBGCLKEN + DBG_SLEEP/STOP/STANDBY
    *(volatile uint32_t *)0x56028A48UL = 1UL;
    (void)*(volatile uint32_t *)0x56028248UL;
    *(volatile uint32_t *)0x56028A44UL = (1UL << 10);
    (void)*(volatile uint32_t *)0x56028244UL;
    *(volatile uint32_t *)0x54001004UL = (1UL << 20)
                                       | (1UL <<  0)
                                       | (1UL <<  1)
                                       | (1UL <<  2);
    (void)*(volatile uint32_t *)0x54001004UL;

    // Vector table redirect first, so a pending IRQ enabled below routes
    // through our handlers (not whatever the boot ROM / FSBL had).
    // On a normal LRUN boot the ROM/loader hands control to Reset_Handler
    // at 0x70000000 with VTOR already pointing there; on a debug-load
    // build vectors live in AXISRAM instead. The linker symbol
    // `isr_vector_table_base` is defined at the start of the .isr_vector
    // section in either script, so VTOR ends up at the correct location
    // regardless of layout.
    SCB->VTOR = (uint32_t)&isr_vector_table_base;
    __DSB();

    // Disable + clear every NVIC IRQ and SysTick before re-enabling
    // interrupts. SWD-load reflashes .text without resetting the NVIC, so
    // any IRQ left enabled by a prior boot (notably USB1_OTG_HS, which a
    // running betaflight enables once VCP comes up) stays pending and
    // fires on __enable_irq() — the new handler then dereferences its
    // pcdHandle while it's still NULL in the freshly-zeroed .bss.
    SysTick->CTRL = 0;
    SysTick->VAL  = 0;
    for (unsigned i = 0; i < (sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0])); i++) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    __DSB();
    __ISB();

    // Reset USB1_OTG_HS — clearing NVIC isn't enough on its own when the
    // peripheral itself is left running by the prior BF (e.g. across an
    // SWD-load): the IRQ source line keeps re-asserting and pending fires
    // the moment NVIC re-enables it, before pcdHandle is wired up. Pulse
    // the AHB5 reset for the controller via the dedicated set/clear regs.
    SET_BIT(RCC->AHB5RSTSR, RCC_AHB5RSTSR_OTG1RSTS_Msk);
    (void)RCC->AHB5RSTR;
    SET_BIT(RCC->AHB5RSTCR, RCC_AHB5RSTCR_OTG1RSTC_Msk);
    (void)RCC->AHB5RSTR;

    // The OTG_HS IRQ source can re-assert between the initial ICPR clear
    // above and the AHB5 reset (the controller is still running until the
    // pulse takes effect). Drop the pending bits again so __enable_irq()
    // below doesn't immediately re-vector into a half-initialised handler.
    for (unsigned i = 0; i < (sizeof(NVIC->ICPR) / sizeof(NVIC->ICPR[0])); i++) {
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    __DSB();
    __ISB();

    // Normalise PRIMASK regardless of what the boot ROM / FSBL handed us.
    // Without this, an FSBL that disables IRQs before the jump leaves
    // SysTick masked, and HAL_Delay()/PLL-lock waits in HAL_RCC_OscConfig
    // hang forever.
    __enable_irq();

    // Cortex-M55 caches: enable I-cache and D-cache before any HAL work.
    SCB_EnableICache();
    SCB_EnableDCache();

    // FPU access. The N6 startup .s does not call CMSIS SystemInit, so the
    // CP10/CP11 enable that ST's standard SystemInit normally provides has
    // to be done here before HAL/clock code emits FPU instructions.
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
    SCB->CPACR |= ((3U << (10U * 2U)) | (3U << (11U * 2U)));
    __DSB();
    __ISB();
#endif

    memoryMappedModeInit();

    // Analog supply + I/O voltage monitors. Normally done by the FSBL, but
    // when BOOT1=1 bypasses the FSBL (or when systemInit is reached via a
    // SWD-loaded RAM_ONLY image) these bits are still at reset: ASV / AVMEN
    // off → ADC analog regulator never goes ready and HAL_ADC_Init's
    // ADC_Enable polling times out; VDDIOxSV off → some I/O domains stay
    // gated. Setting them again from BF is harmless when the FSBL has
    // already done it.
    {
        // SYSCFG clock for VDDIOx compensation cells.
        SET_BIT(RCC->APB4ENSR2, RCC_APB4ENSR2_SYSCFGENS);
        (void)RCC->APB4ENR2;

        PWR->SVMCR1 |= PWR_SVMCR1_VDDIO4SV;
        PWR->SVMCR2 |= PWR_SVMCR2_VDDIO5SV;
        PWR->SVMCR3 |= PWR_SVMCR3_VDDIO2SV | PWR_SVMCR3_VDDIO3SV;

        SYSCFG->VDDIO2CCCR = 0x00000287UL;
        SYSCFG->VDDIO3CCCR = 0x00000287UL;
        SYSCFG->VDDIO4CCCR = 0x00000287UL;
        SYSCFG->VDDIO5CCCR = 0x00000287UL;
        SYSCFG->VDDCCCR    = 0x00000287UL;

        // ADC analog supply + VREF buffer.
        PWR->SVMCR3 |= PWR_SVMCR3_ASV | PWR_SVMCR3_AVMEN;
        (void)PWR->SVMCR3;
        SET_BIT(RCC->APB4ENSR1, RCC_APB4ENSR1_VREFBUFENS);
        (void)RCC->APB4ENR1;
    }

    HAL_Init();

    // Open-everything RIFSC + GPIO config for OPEN-lifecycle dev silicon.
    // BF runs as secure-privileged without -mcmse, so HAL macros emit NS
    // alias addresses; SECCFGR/PRIVCFGR=0 lets those accesses through and
    // RIMC master attributes mark every RIF-tracked master as MCID=1 +
    // SEC + PRIV (LTDC/GPDMA/HPDMA/OTG/SDMMC/etc.). Production builds
    // should replace this with the per-driver granular model.
    {
        SET_BIT(RCC->AHB3ENSR, RCC_AHB3ENR_RIFSCEN_Msk);
        (void)RCC->AHB3ENR;

        for (unsigned i = 0; i < 6U; i++) {
            RIFSC->RISC_SECCFGRx[i]  = 0U;
            RIFSC->RISC_PRIVCFGRx[i] = 0U;
        }
        for (unsigned i = 0; i < 13U; i++) {
            RIFSC->RIMC_ATTRx[i] = ((1U << 8) | (1U << 4) | (1U << 5));
        }

        SET_BIT(RCC->AHB4ENSR,
            RCC_AHB4ENR_GPIOAEN_Msk | RCC_AHB4ENR_GPIOBEN_Msk |
            RCC_AHB4ENR_GPIOCEN_Msk | RCC_AHB4ENR_GPIODEN_Msk |
            RCC_AHB4ENR_GPIOEEN_Msk | RCC_AHB4ENR_GPIOFEN_Msk |
            RCC_AHB4ENR_GPIOGEN_Msk | RCC_AHB4ENR_GPIOHEN_Msk |
            RCC_AHB4ENR_GPIONEN_Msk | RCC_AHB4ENR_GPIOOEN_Msk |
            RCC_AHB4ENR_GPIOPEN_Msk | RCC_AHB4ENR_GPIOQEN_Msk);
        (void)RCC->AHB4ENR;

        GPIO_TypeDef * const gpios[] = {
            GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF,
            GPIOG, GPIOH, GPION, GPIOO, GPIOP, GPIOQ,
        };
        for (unsigned i = 0; i < sizeof(gpios) / sizeof(gpios[0]); i++) {
            gpios[i]->SECCFGR = 0x00000000U;
        }
    }

    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->RSR value to use it in isMPUSoftReset() and others
    cachedResetFlags = RCC->RSR;

    // Enable AXI SRAM clocks
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM1EN);
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM2EN);
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM3EN);
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM4EN);
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM5EN);
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM6EN);

    // Copy .fastram_data, .dmaram_data, etc. from flash to RAM.
    // On other platforms this runs from SystemInit (assembly startup), but the
    // N6 LRUN bootstrap only copies the main _stext.._etext range.
    initialiseMemorySections();

    // Spin PLL1 up off HSE, route IC1 → CPU at 600 MHz, USBPHY1 ← HSE/2.
    SystemClock_Config();

    // Re-assert APB3 bus clock + DBGMCU.CR via the secure alias after the
    // clock-tree reconfig. HAL_RCC_OscConfig / HAL_RCC_ClockConfig drop
    // bus-enable bits while they retune PLL1 and switch the CPU/SYSCLK
    // source; without this post-config re-assert DBGCLKEN is gone by
    // the time SWD tries to attach to a running BF.
    *(volatile uint32_t *)0x56028A44UL = (1UL << 10);   // APB3ENS
    (void)*(volatile uint32_t *)0x56028244UL;
    *(volatile uint32_t *)0x54001004UL = (1UL << 20)    // DBGCLKEN
                                       | (1UL <<  0)    // DBG_SLEEP
                                       | (1UL <<  1)    // DBG_STOP
                                       | (1UL <<  2);   // DBG_STANDBY
    (void)*(volatile uint32_t *)0x54001004UL;

    // Init cycle counter — depends on SystemCoreClock being current
    // (HAL_RCC_ClockConfig calls SystemCoreClockUpdate internally).
    cycleCounterInit();
}

static void SystemClock_Config(void)
{
#if HSE_VALUE == 0
    // No HSE crystal configured for this board — stay on HSI 64 MHz.
    return;
#else
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    // Power supply / voltage scaling: SMPS step-down disabled (VCORE supplied
    // externally on the DK), VOS0 for full-frequency operation.
    if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK) {
        while (1);
    }
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK) {
        while (1);
    }

    // Make sure HSI is on and the PLLs are stopped before disturbing them.
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1);
    }

    // If CPU/SYSCLK are currently driven from an IC (PLL-fed), park them on
    // HSI before retuning the PLL — otherwise HAL_RCC_OscConfig will refuse
    // to disable a PLL that's still feeding the CPU.
    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
    if ((RCC_ClkInitStruct.CPUCLKSource != RCC_CPUCLKSOURCE_HSI) ||
        (RCC_ClkInitStruct.SYSCLKSource != RCC_SYSCLKSOURCE_HSI)) {
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK;
        RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) {
            while (1);
        }
    }

    // Bring up HSE, drive PLL1 to N6_PLL1_VCO_HZ.
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL1.PLLM = 1;
    RCC_OscInitStruct.PLL1.PLLN = N6_PLL1_VCO_HZ / HSE_VALUE;
    RCC_OscInitStruct.PLL1.PLLFractional = 0;
    RCC_OscInitStruct.PLL1.PLLP1 = 1;
    RCC_OscInitStruct.PLL1.PLLP2 = 1;
    RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1);
    }

    // Distribute PLL1 across the IC matrix:
    //   IC1  PLL1/2  → CPU clock          (600 MHz)
    //   IC2  PLL1/3  → SYSB system bus    (400 MHz)
    //   IC6  PLL1/4  → SYSC               (300 MHz)
    //   IC11 PLL1/3  → SYSD               (400 MHz)
    // HCLK and the four PCLK domains are scaled from SYSCLK.
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_HCLK
                                | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
                                | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK4
                                | RCC_CLOCKTYPE_PCLK5;
    RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
    RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
    RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
    RCC_ClkInitStruct.IC1Selection.ClockDivider = 2;
    RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
    RCC_ClkInitStruct.IC2Selection.ClockDivider = 3;
    RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
    RCC_ClkInitStruct.IC6Selection.ClockDivider = 4;
    RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
    RCC_ClkInitStruct.IC11Selection.ClockDivider = 3;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) {
        while (1);
    }

    // USB1 OTG_HS PHY kernel: HSE/2 (24 MHz reference, internally multiplied
    // up). Required for the FS PHY to enumerate. ADC kernel: HCLK / 2 = 150
    // MHz (well within the 75 MHz max so we apply a /2 prescaler) so
    // HAL_ADC_Init / Calibration_Start polling completes.
    HAL_PWREx_EnableVddUSB();
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USBPHY1
                                             | RCC_PERIPHCLK_USBOTGHS1
                                             | RCC_PERIPHCLK_ADC
                                             | RCC_PERIPHCLK_SDMMC1
                                             | RCC_PERIPHCLK_SDMMC2
                                             | RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.UsbPhy1ClockSelection    = RCC_USBPHY1CLKSOURCE_HSE_DIV2;
    PeriphClkInitStruct.UsbOtgHs1ClockSelection  = RCC_USBOTGHS1CLKSOURCE_OTGPHY1;
    PeriphClkInitStruct.AdcClockSelection        = RCC_ADCCLKSOURCE_HCLK;
    PeriphClkInitStruct.AdcDivider               = 2;
    PeriphClkInitStruct.Sdmmc1ClockSelection     = RCC_SDMMC1CLKSOURCE_HCLK;
    PeriphClkInitStruct.Sdmmc2ClockSelection     = RCC_SDMMC2CLKSOURCE_HCLK;
    // USART1 kernel: pick HSI (64 MHz, stable, decoupled from bus clocks) so
    // ST-LINK V3 VCOM bring-up works regardless of how PCLK2 is scaled.
    PeriphClkInitStruct.Usart1ClockSelection     = RCC_USART1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        while (1);
    }

    // Recompute SystemCoreClock from live registers — HAL_RCC_ClockConfig
    // wrote it via HAL_RCC_GetCpuClockFreq() which returned a stale HSI
    // value on N6, leaving SysTick reloaded for HSI not the new CPU clock.
    SystemCoreClockUpdate();

    // Reconfigure SysTick at the (now correct) CPU clock so HAL_GetTick /
    // HAL_Delay match wall-clock time.
    HAL_InitTick(TICK_INT_PRIORITY);
#endif // HSE_VALUE
}

void systemReset(void)
{
    SCB_DisableDCache();
    SCB_DisableICache();

    __disable_irq();
    NVIC_SystemReset();
}

void systemResetWithoutDisablingCaches(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
#if ENABLE_BF_OBL
    // OBL routes to DFU on RCC->RSR.IWDGRSTF; no on-chip primitive
    // survives a soft reset on this board, so a clean SYSRESETREQ
    // would leave SFTRSTF set and OBL would just relaunch BF. Mask all
    // interrupts and spin — TASK_SERIAL stops running, the watchdog
    // refresh in taskHandleSerial stops, IWDG times out (~10 s — see
    // OBL's main.c reload), the chip resets, RCC->RSR.IWDGRSTF is set,
    // and OBL routes to DFU.
    UNUSED(requestType);
    __disable_irq();
    while (1) { /* wait for IWDG */ }
#else
    switch (requestType) {
#if defined(USE_FLASH_BOOT_LOADER)
    case BOOTLOADER_REQUEST_FLASH:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_FLASH);

        break;
#endif
    case BOOTLOADER_REQUEST_ROM:
    default:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);

        break;
    }

    __disable_irq();
    NVIC_SystemReset();
#endif
}

#define SYSMEMBOOT_VECTOR_TABLE ((uint32_t *)0x0BF00000)

typedef void *(*bootJumpPtr)(void);

static void systemJumpToBootloader(void)
{
    __SYSCFG_CLK_ENABLE();

    uint32_t bootStack =  SYSMEMBOOT_VECTOR_TABLE[0];

    bootJumpPtr SysMemBootJump = (bootJumpPtr)SYSMEMBOOT_VECTOR_TABLE[1];

    __set_MSP(bootStack); //Set the main stack pointer to its default values

    SysMemBootJump();

    while (1);
}

void systemProcessResetReason(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (bootloaderRequest) {
#if defined(USE_FLASH_BOOT_LOADER)
    case RESET_BOOTLOADER_REQUEST_FLASH:
#endif
    case RESET_BOOTLOADER_REQUEST_ROM:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_POST);
        systemJumpToBootloader();

        break;

    case RESET_FORCED:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        break;

    case RESET_BOOTLOADER_POST:
        // Boot loader activity magically prevents SysTick from interrupting.
        // Issue a soft reset to prevent the condition.
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_FORCED);
        systemResetWithoutDisablingCaches(); // observed that disabling dcache after cold boot with BOOT pin high causes segfault.

        break;

    case RESET_MSC_REQUEST:
    case RESET_NONE:
    default:
        break;

    }
}
