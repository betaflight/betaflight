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
 * Clock tree setup for the i.MXRT 11xx: supplies, oscillators with the M7 on the RC clock,
 * PLLs, then the roots with the M7 last. The FlexSPI1 root is the boot ROM's. LPUART, LPSPI
 * and LPI2C instances share a rate, as spiCalculateDivider() has no bus.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/system.h"

#include "platform/nxp_clock.h"

// The oscillator block is specified for 24 MHz.
#define XTAL_FREQ_HZ            24000000u

// The three RC oscillators, untrimmed, so nothing timing critical uses them.
#define OSC_RC_16M_HZ           16000000u
#define OSC_RC_48M_DIV2_HZ      24000000u
#define OSC_RC_400M_HZ          400000000u

// ARM PLL: crystal * loop_div / (2 * post_div), 24 MHz * 166 / 4 = 996 MHz.
// POST_DIV_SEL 0..2 divide by 2, 4, 8 and 3 divides by one.
#define ARM_PLL_LOOP_DIV        166
#define ARM_PLL_POST_DIV_SEL    0
#define ARM_PLL_POST_DIV        2

#define CORE_CLOCK_HZ           ((XTAL_FREQ_HZ / (2u * ARM_PLL_POST_DIV)) * ARM_PLL_LOOP_DIV)

STATIC_ASSERT(CORE_CLOCK_HZ == 996000000u, "the ARM PLL setting does not give a 996 MHz core");

// The system PLL rates: PLL2 and PLL3 are fixed, and PLL1 is
// 24 MHz * (41 + 178956970 / 268435455) = 1 GHz.
#define SYS_PLL1_HZ             1000000000u
#define SYS_PLL1_DIV            41
#define SYS_PLL1_NUM            178956970u
#define SYS_PLL1_DENOM          268435455u

#define SYS_PLL2_HZ             528000000u
#define SYS_PLL3_HZ             480000000u

// A PFD multiplies its PLL by 18 and divides by its own FRAC field.
#define PFD_MULTIPLIER          18u

// PLL2 at 352 / 594 / 396 / 432 MHz. PFD2 carries USDHC and PFD3 carries FlexIO.
#define SYS_PLL2_PFD0_FRAC      27
#define SYS_PLL2_PFD1_FRAC      16
#define SYS_PLL2_PFD2_FRAC      24
#define SYS_PLL2_PFD3_FRAC      22

// PLL3 at 664.6 / 508.2 / 270 / 392.7 MHz. PFD0 may carry the boot flash.
#define SYS_PLL3_PFD0_FRAC      13
#define SYS_PLL3_PFD1_FRAC      17
#define SYS_PLL3_PFD2_FRAC      32
#define SYS_PLL3_PFD3_FRAC      22

// DCDC target for VDD_SOC, in units of 25 mV above 600 mV. 996 MHz is in the
// overdrive range and needs 1.15 V: (1150 - 600) / 25 = 22.
#define VDD_SOC_TRG             22

#define OCOTP_FUSE_FBB_WORD     7
#define OCOTP_FUSE_FBB_DISABLE  0x10u

// PMU_BIAS_CTRL well configuration bits: well select and voltage threshold.
#define PMU_BIAS_WELL_SELECT    (1u << 0)
#define PMU_BIAS_VOLTAGE_THRESHOLD (1u << 1)

// Analog interface registers, reached through an address, a data word and a
// toggle bit; each has a set alias at +4 and a clear alias at +8.
#define AI_ADDR_SET             0x04u
#define AI_ADDR_CLR             0x08u

#define AI_PHY_LDO_CTRL0        0x00u
#define AI_PHY_LDO_CTRL0_LINREG_EN  (1u << 0)
#define AI_PHY_LDO_CTRL0_LIMIT_EN   (1u << 2)
#define AI_PHY_LDO_CTRL0_OUTPUT_TRG(x) (((x) & 0x1fu) << 4)
#define AI_PHY_LDO_CTRL0_ENABLED    (AI_PHY_LDO_CTRL0_OUTPUT_TRG(0x10) \
                                     | AI_PHY_LDO_CTRL0_LINREG_EN \
                                     | AI_PHY_LDO_CTRL0_LIMIT_EN)

#define AI_PLL1G_CTRL0          0x00u
#define AI_PLL1G_CTRL2          0x20u
#define AI_PLL1G_CTRL3          0x30u
#define AI_PLL1G_CTRL0_DIV_MASK     0x7fu
#define AI_PLL1G_CTRL0_HOLD_RING_OFF (1u << 13)
#define AI_PLL1G_CTRL0_POWER_UP     (1u << 14)
#define AI_PLL1G_CTRL0_ENABLE       (1u << 15)
#define AI_PLL1G_CTRL0_BYPASS       (1u << 16)
#define AI_PLL1G_CTRL0_PLL_REG_EN   (1u << 22)

// nxpClockInit() programs exactly this rate before main().
uint32_t SystemCoreClock = CORE_CLOCK_HZ;

// A counted wait sized for the 996 MHz core, so it is never shorter than asked.
#define SPINS_PER_US            (CORE_CLOCK_HZ / 1000000u / 4u)

FAST_CODE static void delayUs(uint32_t us)
{
    for (volatile uint32_t spins = us * SPINS_PER_US; spins > 0; spins--) {
    }
}

// The flag waits in this file have no timeout: no steady clock exists yet.

// WDOG1/2 reset with WMCR[PDE] set, a 16 s counter that pulls WDOG_B low. RTWDOG3/4 reset
// enabled; CS is write once unless UPDATE is set, and the unlock window is 128 bus clocks,
// so CS is read before the key.
static void quietBootRom(void)
{
    WDOG1->WMCR &= ~(uint16_t)WDOG_WMCR_PDE_MASK;
    WDOG2->WMCR &= ~(uint16_t)WDOG_WMCR_PDE_MASK;

    RTWDOG_Type *const rtwdogs[] = { RTWDOG3, RTWDOG4 };

    for (unsigned i = 0; i < ARRAYLEN(rtwdogs); i++) {
        RTWDOG_Type *const rtwdog = rtwdogs[i];
        const uint32_t cs = rtwdog->CS;

        if (cs & RTWDOG_CS_CMD32EN_MASK) {
            rtwdog->CNT = RTWDOG_UPDATE_KEY;             // unlock key, as one 32 bit write
        } else {
            rtwdog->CNT = (uint16_t)RTWDOG_UPDATE_KEY;   // unlock key, as two 16 bit halves
            rtwdog->CNT = (uint16_t)(RTWDOG_UPDATE_KEY >> 16);
        }
        rtwdog->TOVAL = RTWDOG_TOVAL_TOVALHIGH_MASK | RTWDOG_TOVAL_TOVALLOW_MASK;   // longest timeout
        rtwdog->CS = (cs & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;
    }

    SysTick->CTRL = 0;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

    for (unsigned i = 0; i < ARRAYLEN(NVIC->ICER); i++) {
        NVIC->ICER[i] = 0xffffffffu;
        NVIC->ICPR[i] = 0xffffffffu;
    }
}

// The regulator side of the analog interface; it has no completion flag, the
// transfer is done when the toggle write retires.
FAST_CODE static void vddsocAiWrite(uint8_t addr, uint32_t wdata)
{
    ANADIG_MISC->VDDSOC_AI_CTRL &= ~ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AIRWB_MASK;
    ANADIG_MISC->VDDSOC_AI_CTRL = (ANADIG_MISC->VDDSOC_AI_CTRL & ~ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AI_ADDR_MASK)
                                | ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AI_ADDR(addr);
    ANADIG_MISC->VDDSOC_AI_WDATA = wdata;

    ANADIG_PMU->PMU_LDO_PLL ^= ANADIG_PMU_PMU_LDO_PLL_LDO_PLL_AI_TOGGLE_MASK;
}

FAST_CODE static uint32_t vddsocAiRead(uint8_t addr)
{
    ANADIG_MISC->VDDSOC_AI_CTRL |= ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AIRWB_MASK;
    ANADIG_MISC->VDDSOC_AI_CTRL = (ANADIG_MISC->VDDSOC_AI_CTRL & ~ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AI_ADDR_MASK)
                                | ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AI_ADDR(addr);

    ANADIG_PMU->PMU_LDO_PLL ^= ANADIG_PMU_PMU_LDO_PLL_LDO_PLL_AI_TOGGLE_MASK;

    return ANADIG_MISC->VDDSOC_AI_RDATA;
}

// The PLL1 side flips a done flag on completion.
FAST_CODE static void pll1gAiWrite(uint8_t addr, uint32_t wdata)
{
    const uint32_t donePre = ANADIG_MISC->VDDSOC2PLL_AI_CTRL_1G
                           & ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AITOGGLE_DONE_1G_MASK;

    ANADIG_MISC->VDDSOC2PLL_AI_CTRL_1G &= ~ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AIRWB_1G_MASK;
    ANADIG_MISC->VDDSOC2PLL_AI_CTRL_1G = (ANADIG_MISC->VDDSOC2PLL_AI_CTRL_1G
                                          & ~ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AIADDR_1G_MASK)
                                       | ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AIADDR_1G(addr);
    ANADIG_MISC->VDDSOC2PLL_AI_WDATA_1G = wdata;

    ANADIG_MISC->VDDSOC2PLL_AI_CTRL_1G ^= ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AITOGGLE_1G_MASK;

    while ((ANADIG_MISC->VDDSOC2PLL_AI_CTRL_1G
            & ANADIG_MISC_VDDSOC2PLL_AI_CTRL_1G_VDDSOC2PLL_AITOGGLE_DONE_1G_MASK) == donePre) {
    }
}

// Set and clear bits of one PLL1 register through its set and clear aliases.
FAST_CODE static void modifyPll1g(uint8_t addr, uint32_t set, uint32_t clr)
{
    if (set) {
        pll1gAiWrite(addr + AI_ADDR_SET, set);
    }
    if (clr) {
        pll1gAiWrite(addr + AI_ADDR_CLR, clr);
    }
}

// VDD_SOC, with the step limiter cleared so the regulator ramps.
static void setCoreVoltage(void)
{
    DCDC->REG3 &= ~DCDC_REG3_VDD1P0CTRL_DISABLE_STEP_MASK;

    DCDC->CTRL1 = (DCDC->CTRL1 & ~DCDC_CTRL1_VDD1P0CTRL_TRG_MASK) | DCDC_CTRL1_VDD1P0CTRL_TRG(VDD_SOC_TRG);

    while (!(DCDC->REG0 & DCDC_REG0_STS_DC_OK_MASK)) {
    }
}

// Forward body bias on the M7 well for 996 MHz, unless fuse word 7 bit 4 is set.
static void setBodyBias(void)
{
    const bool enable = (OCOTP->FUSEN[OCOTP_FUSE_FBB_WORD].FUSE & OCOTP_FUSE_FBB_DISABLE) == 0;

    if (enable) {
        ANADIG_PMU->PMU_BIAS_CTRL = (ANADIG_PMU->PMU_BIAS_CTRL & ~PMU_BIAS_WELL_SELECT)
                                  | PMU_BIAS_VOLTAGE_THRESHOLD;

        ANADIG_PMU->PMU_BIAS_CTRL2 = (ANADIG_PMU->PMU_BIAS_CTRL2 & ~ANADIG_PMU_PMU_BIAS_CTRL2_WB_PWR_SW_EN_1P8_MASK)
                                   | ANADIG_PMU_PMU_BIAS_CTRL2_WB_PWR_SW_EN_1P8(1)
                                   | ANADIG_PMU_PMU_BIAS_CTRL2_WB_EN_MASK;

        while (!(ANADIG_PMU->PMU_BIAS_CTRL2 & ANADIG_PMU_PMU_BIAS_CTRL2_WB_OK_MASK)) {
        }
    } else {
        ANADIG_PMU->PMU_BIAS_CTRL2 &= ~(ANADIG_PMU_PMU_BIAS_CTRL2_WB_PWR_SW_EN_1P8_MASK
                                        | ANADIG_PMU_PMU_BIAS_CTRL2_WB_EN_MASK);
    }
}

// Put the two low power domain regulators into bypass; each step must settle.
static void bypassLpsrRegulators(void)
{
    ANADIG_LDO_SNVS->PMU_LDO_LPSR_ANA &= ~ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_REG_LP_EN_MASK;
    delayUs(1000);
    ANADIG_LDO_SNVS->PMU_LDO_LPSR_ANA |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_TRACK_MODE_EN_MASK;
    delayUs(1000);
    ANADIG_LDO_SNVS->PMU_LDO_LPSR_ANA |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_BYPASS_MODE_EN_MASK;
    delayUs(1000);
    ANADIG_LDO_SNVS->PMU_LDO_LPSR_ANA |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_REG_DISABLE_MASK;

    ANADIG_LDO_SNVS->PMU_LDO_LPSR_DIG |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_TRACKING_MODE_MASK;
    delayUs(1000);
    ANADIG_LDO_SNVS->PMU_LDO_LPSR_DIG |= ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_BYPASS_MODE_MASK;
    delayUs(1000);
    ANADIG_LDO_SNVS->PMU_LDO_LPSR_DIG &= ~ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_REG_EN_MASK;
}

// The PLL regulator and the reference buffer every PLL needs before locking.
FAST_CODE static void enablePllLdo(void)
{
    if (vddsocAiRead(AI_PHY_LDO_CTRL0) == AI_PHY_LDO_CTRL0_ENABLED) {
        return;
    }

    vddsocAiWrite(AI_PHY_LDO_CTRL0, AI_PHY_LDO_CTRL0_ENABLED);
    delayUs(1000);

    ANADIG_PMU->PMU_REF_CTRL |= ANADIG_PMU_PMU_REF_CTRL_EN_PLL_VOL_REF_BUFFER_MASK;
}

// This and the PLL setup below run from ITCM: the FlexSPI clock may hang off a PLL.
FAST_CODE static void oscSetup(void)
{
    ANADIG_OSC->OSC_16M_CTRL |= ANADIG_OSC_OSC_16M_CTRL_EN_IRC4M16M_MASK;

    ANADIG_OSC->OSC_400M_CTRL1 &= ~ANADIG_OSC_OSC_400M_CTRL1_PWD_MASK;
    ANADIG_OSC->OSC_400M_CTRL2 |= ANADIG_OSC_OSC_400M_CTRL2_ENABLE_CLK_MASK;
    ANADIG_OSC->OSC_400M_CTRL1 |= ANADIG_OSC_OSC_400M_CTRL1_CLKGATE_400MEG_MASK;

    ANADIG_OSC->OSC_48M_CTRL |= ANADIG_OSC_OSC_48M_CTRL_TEN_MASK
                              | ANADIG_OSC_OSC_48M_CTRL_RC_48M_DIV2_EN_MASK;

    ANADIG_OSC->OSC_24M_CTRL |= ANADIG_OSC_OSC_24M_CTRL_OSC_EN_MASK
                              | ANADIG_OSC_OSC_24M_CTRL_LP_EN_MASK;

    while (!(ANADIG_OSC->OSC_24M_CTRL & ANADIG_OSC_OSC_24M_CTRL_OSC_24M_STABLE_MASK)) {
    }

    // Park the M7 and SysTick roots on OSC_RC_48M_DIV2 before the ARM PLL moves.
    CCM->CLOCK_ROOT[0].CONTROL = 0;
    CCM->CLOCK_ROOT[8].CONTROL = 0;

    // The FlexRAM clock group divided by three, within its rating at 996 MHz.
    CCM->CLOCK_GROUP[0].CONTROL = CCM_CLOCK_GROUP_CONTROL_RSTDIV(3 - 1) | CCM_CLOCK_GROUP_CONTROL_DIV0(3 - 1);

    // Keep the TCM clock running while the M7 sleeps, for DMA into TCM.
    IOMUXC_GPR->GPR16 |= IOMUXC_GPR_GPR16_CM7_FORCE_HCLK_EN_MASK;
}

FAST_CODE static void initArmPll(void)
{
    uint32_t ctrl = ANADIG_PLL->ARM_PLL_CTRL;

    const uint32_t wanted = ANADIG_PLL_ARM_PLL_CTRL_DIV_SELECT(ARM_PLL_LOOP_DIV)
                          | ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL(ARM_PLL_POST_DIV_SEL);
    const uint32_t setting = ctrl & (ANADIG_PLL_ARM_PLL_CTRL_DIV_SELECT_MASK
                                     | ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL_MASK);

    if ((ctrl & ANADIG_PLL_ARM_PLL_CTRL_POWERUP_MASK) && setting == wanted) {
        // Already at the wanted rate: only open the output, without re-locking.
        ANADIG_PLL->ARM_PLL_CTRL = (ctrl | ANADIG_PLL_ARM_PLL_CTRL_ENABLE_CLK_MASK)
                                 & ~ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_GATE_MASK;
        return;
    }

    enablePllLdo();

    // Power down and gate before the divider moves, then power up with the new
    // setting in the same write.
    ctrl &= ~(ANADIG_PLL_ARM_PLL_CTRL_POWERUP_MASK | ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_STABLE_MASK);
    ANADIG_PLL->ARM_PLL_CTRL = ctrl | ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_GATE_MASK;

    ctrl &= ~(ANADIG_PLL_ARM_PLL_CTRL_DIV_SELECT_MASK | ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL_MASK);
    ctrl |= wanted | ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_GATE_MASK | ANADIG_PLL_ARM_PLL_CTRL_POWERUP_MASK;
    ANADIG_PLL->ARM_PLL_CTRL = ctrl;

    delayUs(30);

    while (!(ANADIG_PLL->ARM_PLL_CTRL & ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_STABLE_MASK)) {
    }

    ctrl |= ANADIG_PLL_ARM_PLL_CTRL_ENABLE_CLK_MASK;
    ctrl &= ~ANADIG_PLL_ARM_PLL_CTRL_ARM_PLL_GATE_MASK;
    ANADIG_PLL->ARM_PLL_CTRL = ctrl;
}

// PLL1, the 1 GHz fractional synthesiser, set through the analog interface.
// SYS_PLL1_GATE is cleared at the end because the LPSPI roots use it.
FAST_CODE static void initSysPll1(void)
{
    modifyPll1g(AI_PLL1G_CTRL0, AI_PLL1G_CTRL0_BYPASS, 0);

    ANADIG_PLL->SYS_PLL1_CTRL |= ANADIG_PLL_SYS_PLL1_CTRL_ENABLE_CLK_MASK;

    modifyPll1g(AI_PLL1G_CTRL3, SYS_PLL1_DENOM, ~SYS_PLL1_DENOM);
    modifyPll1g(AI_PLL1G_CTRL2, SYS_PLL1_NUM, ~SYS_PLL1_NUM);
    modifyPll1g(AI_PLL1G_CTRL0, SYS_PLL1_DIV, AI_PLL1G_CTRL0_DIV_MASK & ~SYS_PLL1_DIV);

    modifyPll1g(AI_PLL1G_CTRL0,
                AI_PLL1G_CTRL0_ENABLE | AI_PLL1G_CTRL0_POWER_UP
                | AI_PLL1G_CTRL0_HOLD_RING_OFF | AI_PLL1G_CTRL0_PLL_REG_EN,
                0);

    delayUs(250);
    modifyPll1g(AI_PLL1G_CTRL0, 0, AI_PLL1G_CTRL0_HOLD_RING_OFF);

    while (!(ANADIG_PLL->SYS_PLL1_CTRL & ANADIG_PLL_SYS_PLL1_CTRL_SYS_PLL1_STABLE_MASK)) {
    }

    ANADIG_PLL->SYS_PLL1_CTRL = (ANADIG_PLL->SYS_PLL1_CTRL & ~ANADIG_PLL_SYS_PLL1_CTRL_SYS_PLL1_GATE_MASK)
                              | ANADIG_PLL_SYS_PLL1_CTRL_SYS_PLL1_DIV2_MASK
                              | ANADIG_PLL_SYS_PLL1_CTRL_SYS_PLL1_DIV5_MASK;

    modifyPll1g(AI_PLL1G_CTRL0, 0, AI_PLL1G_CTRL0_BYPASS);
}

// PLL2 and PLL3 registers share a layout apart from two bits.
#define PLL_CTRL(pll3)          (*((pll3) ? &ANADIG_PLL->SYS_PLL3_CTRL   : &ANADIG_PLL->SYS_PLL2_CTRL))
#define PLL_PFD(pll3)           (*((pll3) ? &ANADIG_PLL->SYS_PLL3_PFD    : &ANADIG_PLL->SYS_PLL2_PFD))
#define PLL_UPDATE(pll3)        (*((pll3) ? &ANADIG_PLL->SYS_PLL3_UPDATE : &ANADIG_PLL->SYS_PLL2_UPDATE))

#define PLL_REG_EN(pll3)        ((pll3) ? ANADIG_PLL_SYS_PLL3_CTRL_PLL_REG_EN_MASK \
                                        : ANADIG_PLL_SYS_PLL2_CTRL_PLL_REG_EN_MASK)
#define PLL_POWERUP(pll3)       ((pll3) ? ANADIG_PLL_SYS_PLL3_CTRL_POWERUP_MASK \
                                        : ANADIG_PLL_SYS_PLL2_CTRL_POWERUP_MASK)

#define PLL_GATE                ANADIG_PLL_SYS_PLL2_CTRL_SYS_PLL2_GATE_MASK
#define PLL_HOLD_RING_OFF       ANADIG_PLL_SYS_PLL2_CTRL_HOLD_RING_OFF_MASK
#define PLL_STABLE              ANADIG_PLL_SYS_PLL2_CTRL_SYS_PLL2_STABLE_MASK
#define PLL_ENABLE_CLK          ANADIG_PLL_SYS_PLL2_CTRL_ENABLE_CLK_MASK

#define PLL_PFD_GATES           (ANADIG_PLL_SYS_PLL2_PFD_PFD0_DIV1_CLKGATE_MASK \
                                 | ANADIG_PLL_SYS_PLL2_PFD_PFD1_DIV1_CLKGATE_MASK \
                                 | ANADIG_PLL_SYS_PLL2_PFD_PFD2_DIV1_CLKGATE_MASK \
                                 | ANADIG_PLL_SYS_PLL2_PFD_PFD3_DIV1_CLKGATE_MASK)

// Reference manual order: gate the PFDs, regulator, power up with hold, release, wait, open.
FAST_CODE static void initFixedPll(bool pll3)
{
    uint32_t ctrl = PLL_CTRL(pll3);

    if (ctrl & PLL_POWERUP(pll3)) {
        // Already running and maybe clocking the flash: only open the output.
        PLL_CTRL(pll3) = (ctrl | PLL_ENABLE_CLK) & ~PLL_GATE;
        return;
    }

    enablePllLdo();

    PLL_PFD(pll3) |= PLL_PFD_GATES;

    PLL_CTRL(pll3) = PLL_REG_EN(pll3) | PLL_GATE;
    delayUs(30);

    ctrl = PLL_CTRL(pll3) | PLL_POWERUP(pll3) | PLL_HOLD_RING_OFF;
    PLL_CTRL(pll3) = ctrl;
    delayUs(250);

    ctrl &= ~PLL_HOLD_RING_OFF;
    PLL_CTRL(pll3) = ctrl;

    while (!(PLL_CTRL(pll3) & PLL_STABLE)) {
    }

    ctrl |= PLL_ENABLE_CLK;
    PLL_CTRL(pll3) = ctrl;

    ctrl &= ~PLL_GATE;
    PLL_CTRL(pll3) = ctrl;
}

// One phase fractional divider. The fraction may only change while gated, and
// an update flips the stable flag. A PFD already set and open is left alone.
FAST_CODE static void initPfd(bool pll3, unsigned pfd, uint32_t frac)
{
    // All four dividers have the same three fields, one byte apart.
    const unsigned shift = 8u * pfd;
    const uint32_t fracMask = ANADIG_PLL_SYS_PLL2_PFD_PFD0_FRAC_MASK << shift;
    const uint32_t gateMask = ANADIG_PLL_SYS_PLL2_PFD_PFD0_DIV1_CLKGATE_MASK << shift;
    const uint32_t stableMask = ANADIG_PLL_SYS_PLL2_PFD_PFD0_STABLE_MASK << shift;

    if ((PLL_PFD(pll3) & (fracMask | gateMask)) == (frac << shift)) {
        return;
    }

    const uint32_t stablePre = PLL_PFD(pll3) & stableMask;

    PLL_PFD(pll3) |= gateMask;
    PLL_PFD(pll3) = (PLL_PFD(pll3) & ~fracMask) | (frac << shift);

    PLL_UPDATE(pll3) ^= ANADIG_PLL_SYS_PLL2_UPDATE_PFD0_UPDATE_MASK << pfd;

    PLL_PFD(pll3) &= ~gateMask;

    while ((PLL_PFD(pll3) & stableMask) == stablePre) {
    }
}

FAST_CODE static void initPlls(void)
{
    initArmPll();
    initSysPll1();

    initFixedPll(false);
    initPfd(false, 0, SYS_PLL2_PFD0_FRAC);
    initPfd(false, 1, SYS_PLL2_PFD1_FRAC);
    initPfd(false, 2, SYS_PLL2_PFD2_FRAC);
    initPfd(false, 3, SYS_PLL2_PFD3_FRAC);

    initFixedPll(true);

    // Enable SYS_PLL3_DIV2, which the CAN and other roots use.
    ANADIG_PLL->SYS_PLL3_CTRL |= ANADIG_PLL_SYS_PLL3_CTRL_SYS_PLL3_DIV2_MASK;

    initPfd(true, 0, SYS_PLL3_PFD0_FRAC);
    initPfd(true, 1, SYS_PLL3_PFD1_FRAC);
    initPfd(true, 2, SYS_PLL3_PFD2_FRAC);
    initPfd(true, 3, SYS_PLL3_PFD3_FRAC);
}

// Every root this port sets, in CCM order: the root's mux value and the real
// divisor (the register holds one less).
typedef struct clockRootSetting_s {
    uint8_t root;
    uint8_t mux;
    uint8_t div;
} clockRootSetting_t;

// CCM clock root numbers.
#define ROOT_M7                 0
#define ROOT_BUS                2
#define ROOT_SEMC               4
#define ROOT_M7_SYSTICK         8
#define ROOT_ADC1               9
#define ROOT_FLEXIO1            12
#define ROOT_GPT1               14
#define ROOT_CAN1               22
#define ROOT_LPUART1            25
#define ROOT_LPI2C1             37
#define ROOT_LPSPI1             43
#define ROOT_USDHC1             58

#define ROOT_NONE               0xff

static const clockRootSetting_t clockRootSettings[] = {
    // The M4 is never started, but its roots must be legal.
    { 1,  4, 1 },                   // m4          SYS_PLL3_PFD3, 392 MHz
    { ROOT_BUS, 4, 2 },             // bus         SYS_PLL3 / 2, 240 MHz
    { 3,  5, 3 },                   // bus_lpsr    SYS_PLL3 / 3, 160 MHz
    { 5,  0, 1 },                   // cssys       OSC_RC_48M_DIV2
    { 6,  7, 4 },                   // cstrace     SYS_PLL2 / 4, 132 MHz
    { 7,  0, 1 },                   // m4_systick  OSC_RC_48M_DIV2
    { ROOT_M7_SYSTICK, 0, 240 },    // m7_systick  OSC_RC_48M_DIV2 / 240, 100 kHz
    { ROOT_ADC1, 6, 6 },            // adc1        SYS_PLL2 / 6, 88 MHz
    { 10, 6, 6 },                   // adc2        SYS_PLL2 / 6, 88 MHz
    { 11, 0, 1 },                   // acmp        OSC_RC_48M_DIV2
    // Both FlexIO roots at the same rate, which the DShot timing assumes.
    { ROOT_FLEXIO1, 7, 4 },         // flexio1     SYS_PLL2_PFD3 / 4, 108 MHz
    { 13, 7, 4 },                   // flexio2     SYS_PLL2_PFD3 / 4, 108 MHz

    // GPT1 and GPT2 on the crystal: micros() needs an accurate 24 MHz.
    { ROOT_GPT1, 1, 1 },            // gpt1        OSC_24M
    { 15, 1, 1 },                   // gpt2        OSC_24M
    { 16, 1, 1 },                   // gpt3        OSC_24M
    { 17, 0, 1 },                   // gpt4        OSC_RC_48M_DIV2
    { 18, 1, 1 },                   // gpt5        OSC_24M
    { 19, 0, 1 },                   // gpt6        OSC_RC_48M_DIV2

    // Root 20 is flexspi1, owned by the boot ROM.
    { 21, 0, 1 },                   // flexspi2    OSC_RC_48M_DIV2
    // All three FlexCAN roots at the 80 MHz can_nxp.c timing assumes.
    { ROOT_CAN1, 4, 3 },            // can1        SYS_PLL3_DIV2 / 3, 80 MHz
    { 23, 4, 3 },                   // can2        SYS_PLL3_DIV2 / 3, 80 MHz
    { 24, 5, 6 },                   // can3        SYS_PLL3 / 6, 80 MHz

    // The last two LPUART, LPI2C and LPSPI instances are in the low power
    // domain, whose muxes offer different sources.
    { ROOT_LPUART1, 6, 22 },        // lpuart1     SYS_PLL2 / 22, 24 MHz
    { 26, 6, 11 },                  // lpuart2     SYS_PLL2 / 11, 48 MHz
    { 27, 6, 11 },                  // lpuart3     SYS_PLL2 / 11, 48 MHz
    { 28, 6, 11 },                  // lpuart4     SYS_PLL2 / 11, 48 MHz
    { 29, 6, 11 },                  // lpuart5     SYS_PLL2 / 11, 48 MHz
    { 30, 6, 11 },                  // lpuart6     SYS_PLL2 / 11, 48 MHz
    { 31, 6, 11 },                  // lpuart7     SYS_PLL2 / 11, 48 MHz
    { 32, 6, 11 },                  // lpuart8     SYS_PLL2 / 11, 48 MHz
    { 33, 6, 11 },                  // lpuart9     SYS_PLL2 / 11, 48 MHz
    { 34, 6, 11 },                  // lpuart10    SYS_PLL2 / 11, 48 MHz
    { 35, 5, 10 },                  // lpuart11    SYS_PLL3 / 10, 48 MHz
    { 36, 5, 10 },                  // lpuart12    SYS_PLL3 / 10, 48 MHz

    { ROOT_LPI2C1, 6, 22 },         // lpi2c1      SYS_PLL2 / 22, 24 MHz
    { 38, 6, 22 },                  // lpi2c2      SYS_PLL2 / 22, 24 MHz
    { 39, 6, 22 },                  // lpi2c3      SYS_PLL2 / 22, 24 MHz
    { 40, 6, 22 },                  // lpi2c4      SYS_PLL2 / 22, 24 MHz
    { 41, 5, 20 },                  // lpi2c5      SYS_PLL3 / 20, 24 MHz
    { 42, 5, 20 },                  // lpi2c6      SYS_PLL3 / 20, 24 MHz

    { ROOT_LPSPI1, 5, 2 },          // lpspi1      SYS_PLL1_DIV5 / 2, 100 MHz
    { 44, 5, 2 },                   // lpspi2      SYS_PLL1_DIV5 / 2, 100 MHz
    { 45, 5, 2 },                   // lpspi3      SYS_PLL1_DIV5 / 2, 100 MHz
    { 46, 5, 2 },                   // lpspi4      SYS_PLL1_DIV5 / 2, 100 MHz
    { 47, 7, 2 },                   // lpspi5      SYS_PLL1_DIV5 / 2, 100 MHz
    { 48, 7, 2 },                   // lpspi6      SYS_PLL1_DIV5 / 2, 100 MHz

    { 54, 0, 1 },                   // enet_25m    OSC_RC_48M_DIV2
    { ROOT_USDHC1, 4, 2 },          // usdhc1      SYS_PLL2_PFD2 / 2, 198 MHz

    // Last: this takes the core to 996 MHz.
    { ROOT_M7, 4, 1 },              // m7          PLL_ARM_CLK
};

FAST_CODE static void initClockRoots(void)
{
    // SEMC is unused and its root resets enabled, so switch it off.
    CCM->CLOCK_ROOT[ROOT_SEMC].CONTROL = CCM_CLOCK_ROOT_CONTROL_OFF_MASK;

    for (unsigned i = 0; i < ARRAYLEN(clockRootSettings); i++) {
        const clockRootSetting_t *const setting = &clockRootSettings[i];

        CCM->CLOCK_ROOT[setting->root].CONTROL = CCM_CLOCK_ROOT_CONTROL_MUX(setting->mux)
                                               | CCM_CLOCK_ROOT_CONTROL_DIV(setting->div - 1u);
    }
}

// The low power clock gates and the clock root of each peripheral; count is
// the number of neighbouring gates to open.
typedef struct clockEntry_s {
    uint8_t lpcg;
    uint8_t lpcgCount;
    uint8_t root;
    uint8_t muxGroup;
} clockEntry_t;

// The source list a root's mux picks from: four oscillators on mux 0 to 3, then
// four PLL outputs that differ per list.
typedef enum {
    MUX_PERIPH = 0,     // LPUART1..10, LPI2C1..4, FlexIO, ADC
    MUX_LPSPI,          // LPSPI1..4
    MUX_LPSR,           // LPUART11..12, LPI2C5..6, the low power domain
    MUX_LPSR_LPSPI,     // LPSPI5..6, the low power domain again
    MUX_GPT,            // GPT1, GPT4, GPT5, GPT6
    MUX_GPT_AV,         // GPT2 and GPT3, which reach the audio and video PLLs
    MUX_BUS,            // the bus root
    MUX_USDHC,
    MUX_GROUP_COUNT
} muxGroup_e;

// The clock sources a mux can select; SRC_NONE covers the unused audio and
// video PLLs.
typedef enum {
    SRC_NONE = 0,
    SRC_OSC_RC_48M_DIV2,
    SRC_OSC_24M,
    SRC_OSC_RC_400M,
    SRC_OSC_RC_16M,
    SRC_ARM_PLL,
    SRC_SYS_PLL1_DIV5,
    SRC_SYS_PLL2,
    SRC_SYS_PLL2_PFD0,
    SRC_SYS_PLL2_PFD2,
    SRC_SYS_PLL2_PFD3,
    SRC_SYS_PLL3,
    SRC_SYS_PLL3_DIV2,
    SRC_SYS_PLL3_PFD2,
    SRC_SYS_PLL3_PFD3,
} clockSource_e;

static const uint8_t muxSources[MUX_GROUP_COUNT][8] = {
    [MUX_PERIPH] = {
        SRC_OSC_RC_48M_DIV2, SRC_OSC_24M, SRC_OSC_RC_400M, SRC_OSC_RC_16M,
        SRC_SYS_PLL3_DIV2, SRC_SYS_PLL1_DIV5, SRC_SYS_PLL2, SRC_SYS_PLL2_PFD3,
    },
    [MUX_LPSPI] = {
        SRC_OSC_RC_48M_DIV2, SRC_OSC_24M, SRC_OSC_RC_400M, SRC_OSC_RC_16M,
        SRC_SYS_PLL3_PFD2, SRC_SYS_PLL1_DIV5, SRC_SYS_PLL2, SRC_SYS_PLL2_PFD3,
    },
    [MUX_LPSR] = {
        SRC_OSC_RC_48M_DIV2, SRC_OSC_24M, SRC_OSC_RC_400M, SRC_OSC_RC_16M,
        SRC_SYS_PLL3_PFD3, SRC_SYS_PLL3, SRC_SYS_PLL2_PFD3, SRC_SYS_PLL1_DIV5,
    },
    [MUX_LPSR_LPSPI] = {
        SRC_OSC_RC_48M_DIV2, SRC_OSC_24M, SRC_OSC_RC_400M, SRC_OSC_RC_16M,
        SRC_SYS_PLL3_PFD3, SRC_SYS_PLL3, SRC_SYS_PLL3_PFD2, SRC_SYS_PLL1_DIV5,
    },
    [MUX_GPT] = {
        SRC_OSC_RC_48M_DIV2, SRC_OSC_24M, SRC_OSC_RC_400M, SRC_OSC_RC_16M,
        SRC_SYS_PLL3_DIV2, SRC_SYS_PLL1_DIV5, SRC_SYS_PLL3_PFD2, SRC_SYS_PLL3_PFD3,
    },
    [MUX_GPT_AV] = {
        SRC_OSC_RC_48M_DIV2, SRC_OSC_24M, SRC_OSC_RC_400M, SRC_OSC_RC_16M,
        SRC_SYS_PLL3_DIV2, SRC_SYS_PLL1_DIV5, SRC_NONE, SRC_NONE,
    },
    [MUX_BUS] = {
        SRC_OSC_RC_48M_DIV2, SRC_OSC_24M, SRC_OSC_RC_400M, SRC_OSC_RC_16M,
        SRC_SYS_PLL3, SRC_SYS_PLL1_DIV5, SRC_SYS_PLL2, SRC_SYS_PLL2_PFD3,
    },
    [MUX_USDHC] = {
        SRC_OSC_RC_48M_DIV2, SRC_OSC_24M, SRC_OSC_RC_400M, SRC_OSC_RC_16M,
        SRC_SYS_PLL2_PFD2, SRC_SYS_PLL2_PFD0, SRC_SYS_PLL1_DIV5, SRC_ARM_PLL,
    },
};

static const clockEntry_t clockEntries[NXP_CLK_COUNT] = {
    [NXP_CLK_LPUART1]  = {  86, 1, ROOT_LPUART1 + 0,  MUX_PERIPH },
    [NXP_CLK_LPUART2]  = {  87, 1, ROOT_LPUART1 + 1,  MUX_PERIPH },
    [NXP_CLK_LPUART3]  = {  88, 1, ROOT_LPUART1 + 2,  MUX_PERIPH },
    [NXP_CLK_LPUART4]  = {  89, 1, ROOT_LPUART1 + 3,  MUX_PERIPH },
    [NXP_CLK_LPUART5]  = {  90, 1, ROOT_LPUART1 + 4,  MUX_PERIPH },
    [NXP_CLK_LPUART6]  = {  91, 1, ROOT_LPUART1 + 5,  MUX_PERIPH },
    [NXP_CLK_LPUART7]  = {  92, 1, ROOT_LPUART1 + 6,  MUX_PERIPH },
    [NXP_CLK_LPUART8]  = {  93, 1, ROOT_LPUART1 + 7,  MUX_PERIPH },
    [NXP_CLK_LPUART9]  = {  94, 1, ROOT_LPUART1 + 8,  MUX_PERIPH },
    [NXP_CLK_LPUART10] = {  95, 1, ROOT_LPUART1 + 9,  MUX_PERIPH },
    [NXP_CLK_LPUART11] = {  96, 1, ROOT_LPUART1 + 10, MUX_LPSR },
    [NXP_CLK_LPUART12] = {  97, 1, ROOT_LPUART1 + 11, MUX_LPSR },

    [NXP_CLK_LPSPI1]   = { 104, 1, ROOT_LPSPI1 + 0, MUX_LPSPI },
    [NXP_CLK_LPSPI2]   = { 105, 1, ROOT_LPSPI1 + 1, MUX_LPSPI },
    [NXP_CLK_LPSPI3]   = { 106, 1, ROOT_LPSPI1 + 2, MUX_LPSPI },
    [NXP_CLK_LPSPI4]   = { 107, 1, ROOT_LPSPI1 + 3, MUX_LPSPI },
    [NXP_CLK_LPSPI5]   = { 108, 1, ROOT_LPSPI1 + 4, MUX_LPSR_LPSPI },
    [NXP_CLK_LPSPI6]   = { 109, 1, ROOT_LPSPI1 + 5, MUX_LPSR_LPSPI },

    [NXP_CLK_LPI2C1]   = {  98, 1, ROOT_LPI2C1 + 0, MUX_PERIPH },
    [NXP_CLK_LPI2C2]   = {  99, 1, ROOT_LPI2C1 + 1, MUX_PERIPH },
    [NXP_CLK_LPI2C3]   = { 100, 1, ROOT_LPI2C1 + 2, MUX_PERIPH },
    [NXP_CLK_LPI2C4]   = { 101, 1, ROOT_LPI2C1 + 3, MUX_PERIPH },
    [NXP_CLK_LPI2C5]   = { 102, 1, ROOT_LPI2C1 + 4, MUX_LPSR },
    [NXP_CLK_LPI2C6]   = { 103, 1, ROOT_LPI2C1 + 5, MUX_LPSR },

    [NXP_CLK_FLEXIO1]  = {  53, 1, ROOT_FLEXIO1 + 0, MUX_PERIPH },
    [NXP_CLK_FLEXIO2]  = {  54, 1, ROOT_FLEXIO1 + 1, MUX_PERIPH },

    // The FlexPWM counters count the bus clock.
    [NXP_CLK_FLEXPWM1] = {  79, 1, ROOT_BUS, MUX_BUS },
    [NXP_CLK_FLEXPWM2] = {  80, 1, ROOT_BUS, MUX_BUS },
    [NXP_CLK_FLEXPWM3] = {  81, 1, ROOT_BUS, MUX_BUS },
    [NXP_CLK_FLEXPWM4] = {  82, 1, ROOT_BUS, MUX_BUS },

    [NXP_CLK_ADC1]     = {  55, 1, ROOT_ADC1 + 0, MUX_PERIPH },
    [NXP_CLK_ADC2]     = {  56, 1, ROOT_ADC1 + 1, MUX_PERIPH },

    // CAN3 is in the low power domain, with another mux list.
    [NXP_CLK_CAN1]     = {  83, 1, ROOT_CAN1 + 0, MUX_PERIPH },
    [NXP_CLK_CAN2]     = {  84, 1, ROOT_CAN1 + 1, MUX_PERIPH },
    [NXP_CLK_CAN3]     = {  85, 1, ROOT_CAN1 + 2, MUX_LPSR },

    [NXP_CLK_GPT1]     = {  64, 1, ROOT_GPT1 + 0, MUX_GPT },
    [NXP_CLK_GPT2]     = {  65, 1, ROOT_GPT1 + 1, MUX_GPT_AV },
    [NXP_CLK_GPT3]     = {  66, 1, ROOT_GPT1 + 2, MUX_GPT_AV },
    [NXP_CLK_GPT4]     = {  67, 1, ROOT_GPT1 + 3, MUX_GPT },
    [NXP_CLK_GPT5]     = {  68, 1, ROOT_GPT1 + 4, MUX_GPT },
    [NXP_CLK_GPT6]     = {  69, 1, ROOT_GPT1 + 5, MUX_GPT },

    [NXP_CLK_USDHC1]   = { 117, 1, ROOT_USDHC1, MUX_USDHC },

    [NXP_CLK_EDMA]     = {  22, 1, ROOT_BUS, MUX_BUS },

    // The SNVS high power and low power halves.
    [NXP_CLK_SNVS]     = {  38, 2, ROOT_NONE, 0 },

    [NXP_CLK_USBOH3]   = { 115, 1, ROOT_NONE, 0 },

    // QuadTimer3 and crossbar A feed the FlexPWM external clock; gate only.
    [NXP_CLK_TMR3]     = {  72, 1, ROOT_NONE, 0 },
    [NXP_CLK_XBARA1]   = {  43, 1, ROOT_NONE, 0 },
};

void nxpClockGateOpen(nxpClock_e peripheral)
{
    if ((unsigned)peripheral >= ARRAYLEN(clockEntries)) {
        return;
    }

    const clockEntry_t *const entry = &clockEntries[peripheral];

    for (unsigned i = 0; i < entry->lpcgCount; i++) {
        CCM->LPCG[entry->lpcg + i].DIRECT = CCM_LPCG_DIRECT_ON_MASK;
    }

    // Wait for each gate to report on; an early register access bus faults.
    __DSB();
    __ISB();

    for (unsigned i = 0; i < entry->lpcgCount; i++) {
        unsigned spin = 0;
        while (!(CCM->LPCG[entry->lpcg + i].STATUS0 & CCM_LPCG_STATUS0_ON_MASK)) {
            if (++spin == 100000) {
                failureMode(FAILURE_DEVELOPER);
                return;
            }
        }
    }
}

#if SERIAL_PIOUART_COUNT > 0
// The PIOUART FlexIO root: SYS_PLL3_DIV2, mux 4 of the FlexIO roots, divided by 5.
#define FLEXIO_UART_MUX         4
#define FLEXIO_UART_DIV         5

STATIC_ASSERT(SYS_PLL3_HZ / 2u / FLEXIO_UART_DIV == NXP_FLEXIO_UART_CLOCK_HZ,
              "the PIOUART FlexIO root is not NXP_FLEXIO_UART_CLOCK_HZ");

void nxpClockFlexioUartRoot(nxpClock_e flexio)
{
    if (flexio != NXP_CLK_FLEXIO1 && flexio != NXP_CLK_FLEXIO2) {
        return;
    }

    CCM->CLOCK_ROOT[clockEntries[flexio].root].CONTROL = CCM_CLOCK_ROOT_CONTROL_MUX(FLEXIO_UART_MUX)
                                                       | CCM_CLOCK_ROOT_CONTROL_DIV(FLEXIO_UART_DIV - 1);
}
#endif

// A PFD puts out its PLL times 18 over its FRAC field; FRAC 0 reads as 0 Hz.
static uint32_t pfdHz(uint32_t baseHz, uint32_t frac)
{
    return frac ? (uint32_t)(((uint64_t)baseHz * PFD_MULTIPLIER) / frac) : 0;
}

// All four FRAC fields of one PLL sit a byte apart in the same register.
static uint32_t pfdFrac(uint32_t pfdReg, unsigned pfd)
{
    return (pfdReg >> (8u * pfd)) & ANADIG_PLL_SYS_PLL2_PFD_PFD0_FRAC_MASK;
}

// The ARM PLL is the one PLL whose rate can be read back from the bus.
static uint32_t armPllHz(void)
{
    const uint32_t ctrl = ANADIG_PLL->ARM_PLL_CTRL;
    const uint32_t loopDiv = (ctrl & ANADIG_PLL_ARM_PLL_CTRL_DIV_SELECT_MASK)
                           >> ANADIG_PLL_ARM_PLL_CTRL_DIV_SELECT_SHIFT;
    const uint32_t postDivSel = (ctrl & ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL_MASK)
                              >> ANADIG_PLL_ARM_PLL_CTRL_POST_DIV_SEL_SHIFT;

    // POST_DIV_SEL 3 divides by one.
    const uint32_t postDiv = (postDivSel == 3u) ? 1u : (2u << postDivSel);

    return (XTAL_FREQ_HZ / (2u * postDiv)) * loopDiv;
}

static uint32_t sourceHz(uint8_t source)
{
    switch (source) {
    case SRC_OSC_RC_48M_DIV2:
        return OSC_RC_48M_DIV2_HZ;
    case SRC_OSC_24M:
        return XTAL_FREQ_HZ;
    case SRC_OSC_RC_400M:
        return OSC_RC_400M_HZ;
    case SRC_OSC_RC_16M:
        return OSC_RC_16M_HZ;
    case SRC_ARM_PLL:
        return armPllHz();
    case SRC_SYS_PLL1_DIV5:
        return SYS_PLL1_HZ / 5u;
    case SRC_SYS_PLL2:
        return SYS_PLL2_HZ;
    case SRC_SYS_PLL2_PFD0:
        return pfdHz(SYS_PLL2_HZ, pfdFrac(ANADIG_PLL->SYS_PLL2_PFD, 0));
    case SRC_SYS_PLL2_PFD2:
        return pfdHz(SYS_PLL2_HZ, pfdFrac(ANADIG_PLL->SYS_PLL2_PFD, 2));
    case SRC_SYS_PLL2_PFD3:
        return pfdHz(SYS_PLL2_HZ, pfdFrac(ANADIG_PLL->SYS_PLL2_PFD, 3));
    case SRC_SYS_PLL3:
        return SYS_PLL3_HZ;
    case SRC_SYS_PLL3_DIV2:
        return SYS_PLL3_HZ / 2u;
    case SRC_SYS_PLL3_PFD2:
        return pfdHz(SYS_PLL3_HZ, pfdFrac(ANADIG_PLL->SYS_PLL3_PFD, 2));
    case SRC_SYS_PLL3_PFD3:
        return pfdHz(SYS_PLL3_HZ, pfdFrac(ANADIG_PLL->SYS_PLL3_PFD, 3));
    default:
        return 0;
    }
}

uint32_t nxpClockHz(nxpClock_e peripheral)
{
    if ((unsigned)peripheral >= ARRAYLEN(clockEntries)) {
        return 0;
    }

    const clockEntry_t *const entry = &clockEntries[peripheral];

    if (entry->root == ROOT_NONE) {
        return 0;
    }

    const uint32_t control = CCM->CLOCK_ROOT[entry->root].CONTROL;

    if (control & CCM_CLOCK_ROOT_CONTROL_OFF_MASK) {
        return 0;
    }

    const uint32_t mux = (control & CCM_CLOCK_ROOT_CONTROL_MUX_MASK) >> CCM_CLOCK_ROOT_CONTROL_MUX_SHIFT;
    const uint32_t div = (control & CCM_CLOCK_ROOT_CONTROL_DIV_MASK) + 1u;

    return sourceHz(muxSources[entry->muxGroup][mux]) / div;
}

// The USB PHY's own PLL makes 480 MHz (DIV_SEL 3 is times 20). The usboh3 gate
// must settle before these writes.
void imxrtUsbPhyPllInit(void)
{
    delayUs(1000);

    USBPHY1->PLL_SIC_SET = USBPHY_PLL_SIC_PLL_POWER_MASK | USBPHY_PLL_SIC_PLL_REG_ENABLE_MASK;
    USBPHY1->PLL_SIC_CLR = USBPHY_PLL_SIC_PLL_DIV_SEL_MASK;
    USBPHY1->PLL_SIC_SET = USBPHY_PLL_SIC_PLL_DIV_SEL(3);
    USBPHY1->PLL_SIC_CLR = USBPHY_PLL_SIC_PLL_BYPASS_MASK;
    USBPHY1->PLL_SIC_SET = USBPHY_PLL_SIC_PLL_EN_USB_CLKS_MASK;

    // Release the PHY from reset before ungating its clock.
    USBPHY1->CTRL_CLR = USBPHY_CTRL_SFTRST_MASK;
    USBPHY1->CTRL_CLR = USBPHY_CTRL_CLKGATE_MASK;

    while (!(USBPHY1->PLL_SIC & USBPHY_PLL_SIC_PLL_LOCK_MASK)) {
    }
}

FAST_CODE void nxpClockInit(void)
{
    quietBootRom();

    setCoreVoltage();
    setBodyBias();
    bypassLpsrRegulators();

    oscSetup();
    initPlls();
    initClockRoots();
}
