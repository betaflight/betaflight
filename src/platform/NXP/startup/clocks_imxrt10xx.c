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

/*
 * Clock tree setup for the i.MXRT 10xx: park on the crystal, raise VDD_SOC, PLLs, dividers,
 * then back to the ARM PLL. The FlexSPI roots and PLL3 carry the XIP flash and are left alone.
 */

#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "platform/nxp_clock.h"

// The oscillator block is specified for 24 MHz.
#define XTAL_FREQ_HZ            24000000u

// ARM PLL (PLL1) runs at crystal * DIV_SELECT / 2 = 1152 MHz; CACRR and
// AHB_PODF divide it to the 576 MHz core clock, which keeps IPG at 144 MHz, a
// multiple of 16 that flexpwm_imxrt.h depends on.
#define ARM_PLL_DIV_SELECT      96
#define ARM_PODF_DIVISOR        2

// IPG at a quarter of the core clock: 144 MHz, under its 150 MHz maximum.
#define AHB_PODF_DIVISOR        1
#define IPG_PODF_DIVISOR        4

#define CORE_CLOCK_HZ           ((XTAL_FREQ_HZ * ARM_PLL_DIV_SELECT / 2) / ARM_PODF_DIVISOR / AHB_PODF_DIVISOR)

// The SEMC root is limited to 166 MHz.
#define SEMC_PODF_DIVISOR       4

// PERCLK feeds GPT and PIT from the crystal; the system.c time base needs 24 MHz.
#define PERCLK_CLK_SEL_OSC      1
#define PERCLK_PODF_DIVISOR     1
#define PERCLK_HZ               (XTAL_FREQ_HZ / PERCLK_PODF_DIVISOR)

STATIC_ASSERT(PERCLK_HZ == 24000000u, "PERCLK is not 24 MHz, the microsecond time base would skew");

// System PLL (PLL2): DIV_SELECT 1 selects the 22x multiplier, so 528 MHz.
#define SYS_PLL_DIV_SELECT      1

// USB1 PLL (PLL3): DIV_SELECT 0 selects the 20x multiplier, so 480 MHz.
#define USB1_PLL_DIV_SELECT     0
#define PLL_USB1_HZ             480000000u

// A PFD multiplies its PLL by 18 and divides by its own FRAC field.
#define PFD_MULTIPLIER          18u

// The four LPSPI instances share one branch, selected and divided in CBCMR.
// PLL2 at 528 MHz divided by eight gives 66 MHz.
#define LPSPI_CLK_SEL_PLL2      2u
#define LPSPI_PODF_DIVISOR      8u

// USDHC1 runs from PLL2 PFD2 (396 MHz, left as found) halved, inside its
// 200 MHz limit.
#define USDHC_CLK_SEL_PLL2_PFD2 0u
#define USDHC_PODF_DIVISOR      2u

// The three FlexCAN instances share one branch, selected and divided in CSCMR2.
// Selection 2 is PLL3 / 6 = 80 MHz, the rate the can_nxp.c bit timing assumes.
#define CAN_CLK_SEL_PLL3_80M    2u
#define CAN_PODF_DIVISOR        1u
#define CAN_CLOCK_HZ            ((PLL_USB1_HZ / 6u) / CAN_PODF_DIVISOR)

STATIC_ASSERT(CAN_CLOCK_HZ == 80000000u,
              "the FlexCAN branch is not 80 MHz, every CAN bit rate would be wrong");

// FlexIO runs DShot at 108 MHz, which divides into every DShot bit rate:
// PFD2 makes 432 MHz and the FlexIO root divides it by 4.
#define PLL3_PFD2_FRAC          20
// Divide first: 480 MHz times 18 overflows 32 bits.
#define PLL3_PFD2_HZ            ((PLL_USB1_HZ / PLL3_PFD2_FRAC) * PFD_MULTIPLIER)
#define FLEXIO_CLK_SEL_PLL3_PFD2 1u
#define FLEXIO_CLOCK_DIV        4
#define FLEXIO_CLOCK_HZ         (PLL3_PFD2_HZ / FLEXIO_CLOCK_DIV)

STATIC_ASSERT(PLL3_PFD2_FRAC <= (CCM_ANALOG_PFD_480_PFD2_FRAC_MASK >> CCM_ANALOG_PFD_480_PFD2_FRAC_SHIFT),
              "PLL3 PFD2 fraction does not fit its field");
STATIC_ASSERT(FLEXIO_CLOCK_HZ == 108000000u,
              "FlexIO clock is not 108 MHz, DShot bit timing will be wrong");

// DCDC target voltage, in units of 25 mV above 800 mV. 576 MHz is in the
// overdrive range and needs VDD_SOC at 1.25 V: (1250 - 800) / 25 = 18.
#define VDD_SOC_TRG             18

// PMU 3.0 V regulator output trim. The field runs from 2.625 V at 0 in 25 mV
// steps, so 0x17 is 3.2 V.
#define PMU_REG_3P0_OUTPUT_3V2  0x17

// Divider register fields hold (divisor - 1).
#define PODF(divisor)           ((divisor) - 1)

// nxpClockInit() programs exactly this rate before main().
uint32_t SystemCoreClock = CORE_CLOCK_HZ;

// WDOG1/2 reset with WMCR[PDE] set, a 16 s counter that pulls WDOG_B low. RTWDOG resets
// enabled; its CS is write once unless UPDATE is set, and the unlock window is 128 bus
// clocks, so CS is read before the key.
static void quietBootRom(void)
{
    WDOG1->WMCR &= ~(uint16_t)WDOG_WMCR_PDE_MASK;
    WDOG2->WMCR &= ~(uint16_t)WDOG_WMCR_PDE_MASK;

    const uint32_t rtwdogCs = RTWDOG->CS;

    if (rtwdogCs & RTWDOG_CS_CMD32EN_MASK) {
        RTWDOG->CNT = RTWDOG_UPDATE_KEY;             // unlock key, as one 32 bit write
    } else {
        RTWDOG->CNT = (uint16_t)RTWDOG_UPDATE_KEY;   // unlock key, as two 16 bit halves
        RTWDOG->CNT = (uint16_t)(RTWDOG_UPDATE_KEY >> 16);
    }
    RTWDOG->TOVAL = RTWDOG_TOVAL_TOVALHIGH_MASK | RTWDOG_TOVAL_TOVALLOW_MASK;   // longest timeout
    RTWDOG->CS = (rtwdogCs & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;

    SysTick->CTRL = 0;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

    for (unsigned i = 0; i < ARRAYLEN(NVIC->ICER); i++) {
        NVIC->ICER[i] = 0xffffffffu;
        NVIC->ICPR[i] = 0xffffffffu;
    }
}

// The waits below have no timeout: no steady clock exists yet to time against.
static void parkPeripheralClockOnCrystal(void)
{
    // periph_clk2 <- osc_clk
    CCM->CBCMR = (CCM->CBCMR & ~CCM_CBCMR_PERIPH_CLK2_SEL_MASK) | CCM_CBCMR_PERIPH_CLK2_SEL(1);
    while (CCM->CDHIPR & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY_MASK) {
    }

    // periph_clk <- periph_clk2, so the ARM PLL is now unused
    CCM->CBCDR = (CCM->CBCDR & ~CCM_CBCDR_PERIPH_CLK_SEL_MASK) | CCM_CBCDR_PERIPH_CLK_SEL(1);
    while (CCM->CDHIPR & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY_MASK) {
    }
}

static void setCoreVoltage(void)
{
    DCDC->REG3 = (DCDC->REG3 & ~DCDC_REG3_TRG_MASK) | DCDC_REG3_TRG(VDD_SOC_TRG);
    while (!(DCDC->REG0 & DCDC_REG0_STS_DC_OK_MASK)) {
    }
}

// Runs from ITCM: the FlexSPI clock hangs off the PLLs reprogrammed here.
FAST_CODE static void initPlls(void)
{
    // ARM PLL (PLL1). Writing the register with POWERDOWN clear also powers it up.
    CCM_ANALOG->PLL_ARM = CCM_ANALOG_PLL_ARM_DIV_SELECT(ARM_PLL_DIV_SELECT)
                        | CCM_ANALOG_PLL_ARM_ENABLE_MASK;

    while (!(CCM_ANALOG->PLL_ARM & CCM_ANALOG_PLL_ARM_LOCK_MASK)) {
    }

    // System PLL (PLL2), 528 MHz, feeding the PFDs of FlexSPI, LPSPI and SEMC.
    CCM_ANALOG->PLL_SYS = CCM_ANALOG_PLL_SYS_DIV_SELECT(SYS_PLL_DIV_SELECT)
                        | CCM_ANALOG_PLL_SYS_ENABLE_MASK;

    while (!(CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_LOCK_MASK)) {
    }

    // USB1 PLL (PLL3) already clocks the flash through PFD0, so a running
    // 480 MHz PLL only gets its USB output enabled through the SET register;
    // a full rewrite would make it re-lock under the flash.
    STATIC_ASSERT(USB1_PLL_DIV_SELECT == 0, "the PLL3 check below assumes the 480 MHz setting is zero");

    const uint32_t usb1PllState = CCM_ANALOG->PLL_USB1
                                & (CCM_ANALOG_PLL_USB1_DIV_SELECT_MASK
                                   | CCM_ANALOG_PLL_USB1_BYPASS_MASK
                                   | CCM_ANALOG_PLL_USB1_POWER_MASK);

    if (usb1PllState == CCM_ANALOG_PLL_USB1_POWER_MASK) {
        CCM_ANALOG->PLL_USB1_SET = CCM_ANALOG_PLL_USB1_ENABLE_MASK
                                 | CCM_ANALOG_PLL_USB1_EN_USB_CLKS_MASK;
    } else {
        CCM_ANALOG->PLL_USB1 = CCM_ANALOG_PLL_USB1_DIV_SELECT(USB1_PLL_DIV_SELECT)
                             | CCM_ANALOG_PLL_USB1_ENABLE_MASK
                             | CCM_ANALOG_PLL_USB1_EN_USB_CLKS_MASK
                             | CCM_ANALOG_PLL_USB1_POWER_MASK;

        while (!(CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_LOCK_MASK)) {
        }
    }

    // PFD2 to 432 MHz. The fraction may only change while PFD2 is gated.
    CCM_ANALOG->PFD_480_SET = CCM_ANALOG_PFD_480_PFD2_CLKGATE_MASK;
    CCM_ANALOG->PFD_480 = (CCM_ANALOG->PFD_480 & ~(CCM_ANALOG_PFD_480_PFD2_FRAC_MASK
                                                   | CCM_ANALOG_PFD_480_PFD2_CLKGATE_MASK))
                        | CCM_ANALOG_PFD_480_PFD2_FRAC(PLL3_PFD2_FRAC);
}

static void setDividers(void)
{
    CCM->CACRR = CCM_CACRR_ARM_PODF(PODF(ARM_PODF_DIVISOR));
    while (CCM->CDHIPR & CCM_CDHIPR_ARM_PODF_BUSY_MASK) {
    }

    CCM->CBCDR = (CCM->CBCDR & ~CCM_CBCDR_AHB_PODF_MASK) | CCM_CBCDR_AHB_PODF(PODF(AHB_PODF_DIVISOR));
    while (CCM->CDHIPR & CCM_CDHIPR_AHB_PODF_BUSY_MASK) {
    }

    // IPG has no handshake bit.
    CCM->CBCDR = (CCM->CBCDR & ~CCM_CBCDR_IPG_PODF_MASK) | CCM_CBCDR_IPG_PODF(PODF(IPG_PODF_DIVISOR));

    CCM->CBCDR = (CCM->CBCDR & ~CCM_CBCDR_SEMC_PODF_MASK) | CCM_CBCDR_SEMC_PODF(PODF(SEMC_PODF_DIVISOR));
    while (CCM->CDHIPR & CCM_CDHIPR_SEMC_PODF_BUSY_MASK) {
    }

    CCM->CSCMR1 = (CCM->CSCMR1 & ~(CCM_CSCMR1_PERCLK_PODF_MASK | CCM_CSCMR1_PERCLK_CLK_SEL_MASK))
                | CCM_CSCMR1_PERCLK_PODF(PODF(PERCLK_PODF_DIVISOR))
                | CCM_CSCMR1_PERCLK_CLK_SEL(PERCLK_CLK_SEL_OSC);   // not the IPG root
}

static void selectArmPll(void)
{
    // pre_periph_clk <- divided PLL1
    CCM->CBCMR = (CCM->CBCMR & ~CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK) | CCM_CBCMR_PRE_PERIPH_CLK_SEL(3);

    // periph_clk <- pre_periph_clk
    CCM->CBCDR = (CCM->CBCDR & ~CCM_CBCDR_PERIPH_CLK_SEL_MASK) | CCM_CBCDR_PERIPH_CLK_SEL(0);
    while (CCM->CDHIPR & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY_MASK) {
    }
}

static void initPeripheralClockRoots(void)
{
    // Every LPUART runs from pll3_80m undivided, 80 MHz.
    CCM->CSCDR1 = (CCM->CSCDR1 & ~(CCM_CSCDR1_UART_CLK_SEL_MASK | CCM_CSCDR1_UART_CLK_PODF_MASK))
                | CCM_CSCDR1_UART_CLK_SEL(0)
                | CCM_CSCDR1_UART_CLK_PODF(PODF(1));

    // Both FlexIO roots from PLL3 PFD2 / 4, 108 MHz. FlexIO1 is set in CDCDR;
    // FlexIO2 has its select in CSCMR2 and dividers in CS1CDR.
    CCM->CDCDR = (CCM->CDCDR & ~(CCM_CDCDR_FLEXIO1_CLK_SEL_MASK
                                 | CCM_CDCDR_FLEXIO1_CLK_PRED_MASK
                                 | CCM_CDCDR_FLEXIO1_CLK_PODF_MASK))
               | CCM_CDCDR_FLEXIO1_CLK_SEL(FLEXIO_CLK_SEL_PLL3_PFD2)
               | CCM_CDCDR_FLEXIO1_CLK_PRED(PODF(FLEXIO_CLOCK_DIV))
               | CCM_CDCDR_FLEXIO1_CLK_PODF(PODF(1));

    CCM->CSCMR2 = (CCM->CSCMR2 & ~CCM_CSCMR2_FLEXIO2_CLK_SEL_MASK)
                | CCM_CSCMR2_FLEXIO2_CLK_SEL(FLEXIO_CLK_SEL_PLL3_PFD2);

    CCM->CS1CDR = (CCM->CS1CDR & ~(CCM_CS1CDR_FLEXIO2_CLK_PRED_MASK
                                   | CCM_CS1CDR_FLEXIO2_CLK_PODF_MASK))
                | CCM_CS1CDR_FLEXIO2_CLK_PRED(PODF(FLEXIO_CLOCK_DIV))
                | CCM_CS1CDR_FLEXIO2_CLK_PODF(PODF(1));

    // The LPSPI branch, shared by all four instances.
    CCM->CBCMR = (CCM->CBCMR & ~(CCM_CBCMR_LPSPI_CLK_SEL_MASK | CCM_CBCMR_LPSPI_PODF_MASK))
               | CCM_CBCMR_LPSPI_CLK_SEL(LPSPI_CLK_SEL_PLL2)
               | CCM_CBCMR_LPSPI_PODF(PODF(LPSPI_PODF_DIVISOR));

    // The FlexCAN branch, shared by all three controllers, also in CSCMR2.
    CCM->CSCMR2 = (CCM->CSCMR2 & ~(CCM_CSCMR2_CAN_CLK_SEL_MASK | CCM_CSCMR2_CAN_CLK_PODF_MASK))
                | CCM_CSCMR2_CAN_CLK_SEL(CAN_CLK_SEL_PLL3_80M)
                | CCM_CSCMR2_CAN_CLK_PODF(PODF(CAN_PODF_DIVISOR));

    // ERR050235: with CAN_CLK_SEL 2 the FlexCAN root passes through the LPUART1
    // gate (CCGR5 CG12), so that gate must be open for the CAN engine to run.
#if !defined(FSL_FEATURE_CCM_HAS_ERRATA_50235)
#error "no ERR050235 applicability flag: the FlexCAN clock workaround cannot be decided"
#endif
#if FSL_FEATURE_CCM_HAS_ERRATA_50235
    CCM->CCGR5 |= CCM_CCGR5_CG12_MASK;
#endif

    // USDHC1 select and divider are in different registers.
    CCM->CSCMR1 = (CCM->CSCMR1 & ~CCM_CSCMR1_USDHC1_CLK_SEL_MASK)
                | CCM_CSCMR1_USDHC1_CLK_SEL(USDHC_CLK_SEL_PLL2_PFD2);

    CCM->CSCDR1 = (CCM->CSCDR1 & ~CCM_CSCDR1_USDHC1_PODF_MASK)
                | CCM_CSCDR1_USDHC1_PODF(PODF(USDHC_PODF_DIVISOR));
}

// The CCGR gate of each peripheral. Both bits of a field set keep the clock on
// in run and wait mode; two fields gate the bus and functional clocks apart.
typedef struct clockGate_s {
    volatile uint32_t *ccgr;
    uint32_t mask;
} clockGate_t;

static const clockGate_t clockGates[NXP_CLK_COUNT] = {
    [NXP_CLK_LPUART1]  = { &CCM->CCGR5, CCM_CCGR5_CG12_MASK },
    [NXP_CLK_LPUART2]  = { &CCM->CCGR0, CCM_CCGR0_CG14_MASK },
    [NXP_CLK_LPUART3]  = { &CCM->CCGR0, CCM_CCGR0_CG6_MASK },
    [NXP_CLK_LPUART4]  = { &CCM->CCGR1, CCM_CCGR1_CG12_MASK },
    [NXP_CLK_LPUART5]  = { &CCM->CCGR3, CCM_CCGR3_CG1_MASK },
    [NXP_CLK_LPUART6]  = { &CCM->CCGR3, CCM_CCGR3_CG3_MASK },
    [NXP_CLK_LPUART7]  = { &CCM->CCGR5, CCM_CCGR5_CG13_MASK },
    [NXP_CLK_LPUART8]  = { &CCM->CCGR6, CCM_CCGR6_CG7_MASK },

    [NXP_CLK_LPSPI1]   = { &CCM->CCGR1, CCM_CCGR1_CG0_MASK },
    [NXP_CLK_LPSPI2]   = { &CCM->CCGR1, CCM_CCGR1_CG1_MASK },
    [NXP_CLK_LPSPI3]   = { &CCM->CCGR1, CCM_CCGR1_CG2_MASK },
    [NXP_CLK_LPSPI4]   = { &CCM->CCGR1, CCM_CCGR1_CG3_MASK },

    [NXP_CLK_LPI2C1]   = { &CCM->CCGR2, CCM_CCGR2_CG3_MASK },
    [NXP_CLK_LPI2C2]   = { &CCM->CCGR2, CCM_CCGR2_CG4_MASK },
    [NXP_CLK_LPI2C3]   = { &CCM->CCGR2, CCM_CCGR2_CG5_MASK },
    [NXP_CLK_LPI2C4]   = { &CCM->CCGR6, CCM_CCGR6_CG12_MASK },

    [NXP_CLK_FLEXIO1]  = { &CCM->CCGR5, CCM_CCGR5_CG1_MASK },
    [NXP_CLK_FLEXIO2]  = { &CCM->CCGR3, CCM_CCGR3_CG0_MASK },

    [NXP_CLK_FLEXPWM1] = { &CCM->CCGR4, CCM_CCGR4_CG8_MASK },
    [NXP_CLK_FLEXPWM2] = { &CCM->CCGR4, CCM_CCGR4_CG9_MASK },
    [NXP_CLK_FLEXPWM3] = { &CCM->CCGR4, CCM_CCGR4_CG10_MASK },
    [NXP_CLK_FLEXPWM4] = { &CCM->CCGR4, CCM_CCGR4_CG11_MASK },

    [NXP_CLK_ADC1]     = { &CCM->CCGR1, CCM_CCGR1_CG8_MASK },
    [NXP_CLK_ADC2]     = { &CCM->CCGR1, CCM_CCGR1_CG4_MASK },

    // Bus and serial clock of each FlexCAN controller; it is dead without both.
    [NXP_CLK_CAN1]     = { &CCM->CCGR0, CCM_CCGR0_CG7_MASK | CCM_CCGR0_CG8_MASK },
    [NXP_CLK_CAN2]     = { &CCM->CCGR0, CCM_CCGR0_CG9_MASK | CCM_CCGR0_CG10_MASK },
    [NXP_CLK_CAN3]     = { &CCM->CCGR7, CCM_CCGR7_CG3_MASK | CCM_CCGR7_CG4_MASK },

    // Bus and serial clock of each counter.
    [NXP_CLK_GPT1]     = { &CCM->CCGR1, CCM_CCGR1_CG10_MASK | CCM_CCGR1_CG11_MASK },
    [NXP_CLK_GPT2]     = { &CCM->CCGR0, CCM_CCGR0_CG12_MASK | CCM_CCGR0_CG13_MASK },

    [NXP_CLK_USDHC1]   = { &CCM->CCGR6, CCM_CCGR6_CG1_MASK },

    // One gate covers the eDMA engine and the DMAMUX, which has none of its own.
    [NXP_CLK_EDMA]     = { &CCM->CCGR5, CCM_CCGR5_CG3_MASK },

    // The SNVS high power and low power halves.
    [NXP_CLK_SNVS]     = { &CCM->CCGR5, CCM_CCGR5_CG14_MASK | CCM_CCGR5_CG15_MASK },

    [NXP_CLK_USBOH3]   = { &CCM->CCGR6, CCM_CCGR6_CG0_MASK },

    // QuadTimer3 and crossbar A feed the FlexPWM external clock; gate only.
    [NXP_CLK_TMR3]     = { &CCM->CCGR6, CCM_CCGR6_CG15_MASK },
    [NXP_CLK_XBARA1]   = { &CCM->CCGR2, CCM_CCGR2_CG11_MASK },
};

void nxpClockGateOpen(nxpClock_e peripheral)
{
    if ((unsigned)peripheral >= ARRAYLEN(clockGates) || !clockGates[peripheral].ccgr) {
        return;
    }

    *clockGates[peripheral].ccgr |= clockGates[peripheral].mask;

    // The first peripheral access must not reach the bus before the gate opens.
    __DSB();
    const uint32_t mask = clockGates[peripheral].mask;
    for (unsigned spin = 0; spin < 1000 && (*clockGates[peripheral].ccgr & mask) != mask; spin++) {
    }
}

// A PFD puts out its PLL times 18 over its FRAC field; FRAC 0 reads as 0 Hz.
static uint32_t pfdHz(uint32_t baseHz, uint32_t frac)
{
    return frac ? (uint32_t)(((uint64_t)baseHz * PFD_MULTIPLIER) / frac) : 0;
}

// System PLL (PLL2). DIV_SELECT picks a multiplier of 20 or 22 on the crystal.
static uint32_t pllSysHz(void)
{
    return XTAL_FREQ_HZ * ((CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_DIV_SELECT_MASK) ? 22u : 20u);
}

// IPG_CLK_ROOT: SystemCoreClock (the AHB root) divided by IPG_PODF.
static uint32_t ipgClockHz(void)
{
    const uint32_t podf = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;

    return SystemCoreClock / podf;
}

// PERCLK_CLK_ROOT, which both GPTs count.
static uint32_t perclkHz(void)
{
    const uint32_t sel = (CCM->CSCMR1 & CCM_CSCMR1_PERCLK_CLK_SEL_MASK) >> CCM_CSCMR1_PERCLK_CLK_SEL_SHIFT;
    const uint32_t root = (sel == PERCLK_CLK_SEL_OSC) ? XTAL_FREQ_HZ : ipgClockHz();
    const uint32_t podf = ((CCM->CSCMR1 & CCM_CSCMR1_PERCLK_PODF_MASK) >> CCM_CSCMR1_PERCLK_PODF_SHIFT) + 1;

    return root / podf;
}

// The shared LPUART branch. Select 0 is pll3_80m, the USB1 PLL divided by six
// inside the CCM.
static uint32_t uartClockHz(void)
{
    const uint32_t root = (CCM->CSCDR1 & CCM_CSCDR1_UART_CLK_SEL_MASK) ? XTAL_FREQ_HZ : PLL_USB1_HZ / 6u;
    const uint32_t podf = ((CCM->CSCDR1 & CCM_CSCDR1_UART_CLK_PODF_MASK) >> CCM_CSCDR1_UART_CLK_PODF_SHIFT) + 1;

    return root / podf;
}

// The shared LPI2C branch. Select 0 is pll3_60m, the USB1 PLL divided by eight
// inside the CCM.
static uint32_t lpi2cClockHz(void)
{
    const uint32_t root = (CCM->CSCDR2 & CCM_CSCDR2_LPI2C_CLK_SEL_MASK) ? XTAL_FREQ_HZ : PLL_USB1_HZ / 8u;
    const uint32_t podf = ((CCM->CSCDR2 & CCM_CSCDR2_LPI2C_CLK_PODF_MASK) >> CCM_CSCDR2_LPI2C_CLK_PODF_SHIFT) + 1;

    return root / podf;
}

// The shared LPSPI branch, selected and divided in CBCMR.
static uint32_t lpspiClockHz(void)
{
    const uint32_t pfd480 = CCM_ANALOG->PFD_480;
    uint32_t root;

    switch ((CCM->CBCMR & CCM_CBCMR_LPSPI_CLK_SEL_MASK) >> CCM_CBCMR_LPSPI_CLK_SEL_SHIFT) {
    case 0:
        root = pfdHz(PLL_USB1_HZ, (pfd480 & CCM_ANALOG_PFD_480_PFD1_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD1_FRAC_SHIFT);
        break;
    case 1:
        root = pfdHz(PLL_USB1_HZ, (pfd480 & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT);
        break;
    case LPSPI_CLK_SEL_PLL2:
        root = pllSysHz();
        break;
    default:
        root = pfdHz(pllSysHz(),
                     (CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD2_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT);
        break;
    }

    const uint32_t podf = ((CCM->CBCMR & CCM_CBCMR_LPSPI_PODF_MASK) >> CCM_CBCMR_LPSPI_PODF_SHIFT) + 1;

    return root / podf;
}

// The shared FlexCAN branch, selected and divided in CSCMR2.
static uint32_t canClockHz(void)
{
    uint32_t root;

    switch ((CCM->CSCMR2 & CCM_CSCMR2_CAN_CLK_SEL_MASK) >> CCM_CSCMR2_CAN_CLK_SEL_SHIFT) {
    case 0:
        root = PLL_USB1_HZ / 8u;        // pll3_60m
        break;
    case 1:
        root = XTAL_FREQ_HZ;
        break;
    case CAN_CLK_SEL_PLL3_80M:
        root = PLL_USB1_HZ / 6u;        // pll3_80m
        break;
    default:
        return 0;
    }

    const uint32_t podf = ((CCM->CSCMR2 & CCM_CSCMR2_CAN_CLK_PODF_MASK) >> CCM_CSCMR2_CAN_CLK_PODF_SHIFT) + 1;

    return root / podf;
}

// The USDHC1 root, which hangs off one of the two PLL2 fractional dividers.
static uint32_t usdhcClockHz(void)
{
    const uint32_t pfd528 = CCM_ANALOG->PFD_528;
    uint32_t frac;

    if (CCM->CSCMR1 & CCM_CSCMR1_USDHC1_CLK_SEL_MASK) {
        frac = (pfd528 & CCM_ANALOG_PFD_528_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD0_FRAC_SHIFT;
    } else {
        frac = (pfd528 & CCM_ANALOG_PFD_528_PFD2_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT;
    }

    const uint32_t podf = ((CCM->CSCDR1 & CCM_CSCDR1_USDHC1_PODF_MASK) >> CCM_CSCDR1_USDHC1_PODF_SHIFT) + 1;

    return pfdHz(pllSysHz(), frac) / podf;
}

// A FlexIO root; the audio and video PLL selections (0 and 2) read as 0 Hz.
static uint32_t flexioClockHz(nxpClock_e peripheral)
{
    uint32_t sel;
    uint32_t pred;
    uint32_t podf;

    if (peripheral == NXP_CLK_FLEXIO1) {
        const uint32_t cdcdr = CCM->CDCDR;
        sel  = (cdcdr & CCM_CDCDR_FLEXIO1_CLK_SEL_MASK) >> CCM_CDCDR_FLEXIO1_CLK_SEL_SHIFT;
        pred = ((cdcdr & CCM_CDCDR_FLEXIO1_CLK_PRED_MASK) >> CCM_CDCDR_FLEXIO1_CLK_PRED_SHIFT) + 1;
        podf = ((cdcdr & CCM_CDCDR_FLEXIO1_CLK_PODF_MASK) >> CCM_CDCDR_FLEXIO1_CLK_PODF_SHIFT) + 1;
    } else {
        const uint32_t cs1cdr = CCM->CS1CDR;
        sel  = (CCM->CSCMR2 & CCM_CSCMR2_FLEXIO2_CLK_SEL_MASK) >> CCM_CSCMR2_FLEXIO2_CLK_SEL_SHIFT;
        pred = ((cs1cdr & CCM_CS1CDR_FLEXIO2_CLK_PRED_MASK) >> CCM_CS1CDR_FLEXIO2_CLK_PRED_SHIFT) + 1;
        podf = ((cs1cdr & CCM_CS1CDR_FLEXIO2_CLK_PODF_MASK) >> CCM_CS1CDR_FLEXIO2_CLK_PODF_SHIFT) + 1;
    }

    uint32_t root;

    switch (sel) {
    case FLEXIO_CLK_SEL_PLL3_PFD2:
        root = pfdHz(PLL_USB1_HZ,
                     (CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD2_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD2_FRAC_SHIFT);
        break;
    case 3:
        root = PLL_USB1_HZ;
        break;
    default:
        return 0;
    }

    return root / (pred * podf);
}

uint32_t nxpClockHz(nxpClock_e peripheral)
{
    // The enum is grouped by clock root, so a range decides which root to read.
    if (peripheral <= NXP_CLK_LPUART8) {
        return uartClockHz();
    }
    if (peripheral <= NXP_CLK_LPSPI4) {
        return lpspiClockHz();
    }
    if (peripheral <= NXP_CLK_LPI2C4) {
        return lpi2cClockHz();
    }
    if (peripheral <= NXP_CLK_FLEXIO2) {
        return flexioClockHz(peripheral);
    }
    if (peripheral <= NXP_CLK_ADC2) {
        // FlexPWM counts the IPBus clock and the ADC divides it down further.
        return ipgClockHz();
    }
    if (peripheral <= NXP_CLK_CAN3) {
        return canClockHz();
    }
    if (peripheral <= NXP_CLK_GPT2) {
        return perclkHz();
    }
    if (peripheral == NXP_CLK_USDHC1) {
        return usdhcClockHz();
    }
    if (peripheral == NXP_CLK_EDMA) {
        return ipgClockHz();
    }

    // SNVS and USB are gate only, with no CCM root rate.
    return 0;
}

void imxrtUsbPhyPllInit(void)
{
    // The PLL3 USB clocks are enabled at startup; this sets up the PHY supply,
    // the 3.0 V regulator.
    PMU->REG_3P0 = (PMU->REG_3P0 & ~PMU_REG_3P0_OUTPUT_TRG_MASK)
                 | PMU_REG_3P0_OUTPUT_TRG(PMU_REG_3P0_OUTPUT_3V2)
                 | PMU_REG_3P0_ENABLE_LINREG_MASK;

    // Disable charger detect, whose reset pull downs on D+ and D- hide the
    // device; both bits are active low.
    USB_ANALOG->INSTANCE[0].CHRG_DETECT_SET = USB_ANALOG_CHRG_DETECT_CHK_CHRG_B_MASK
                                            | USB_ANALOG_CHRG_DETECT_EN_B_MASK;
}

FAST_CODE void nxpClockInit(void)
{
    quietBootRom();

    parkPeripheralClockOnCrystal();
    setCoreVoltage();
    initPlls();
    setDividers();
    selectArmPll();
    initPeripheralClockRoots();
}
