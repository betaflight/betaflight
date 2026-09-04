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

// FlexPWM instance and submodule setup; the per update calls are inline in
// flexpwm_imxrt.h.

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "flexpwm_imxrt.h"

uint32_t flexpwmExtClockDivisor(void)
{
    return nxpClockHz(NXP_CLK_FLEXPWM1) / FLEXPWM_EXT_CLK_HZ;
}

uint32_t flexpwmClockHz(void)
{
    const uint32_t divisor = flexpwmExtClockDivisor();

    return divisor ? nxpClockHz(NXP_CLK_FLEXPWM1) / divisor : 0;
}

bool flexpwmDeriveTiming(uint32_t clockHz, uint32_t rateHz, uint8_t *prescaler, uint16_t *modulo)
{
    if (clockHz == 0 || rateHz == 0) {
        return false;
    }

    for (uint8_t prsc = 0; prsc <= FLEXPWM_PRESCALER_MAX; prsc++) {
        const uint32_t tickHz = flexpwmTickRate(clockHz, prsc);
        const uint32_t ticks = (tickHz + rateHz / 2) / rateHz;

        if (ticks > FLEXPWM_MODULO_MAX) {
            continue;   // period too long for the counter, divide the clock more
        }

        if (ticks < FLEXPWM_MODULO_MIN) {
            return false;   // already at the fastest tick and the period is still too short
        }

        *prescaler = prsc;
        *modulo = (uint16_t)ticks;
        return true;
    }

    return false;
}

static nxpClock_e flexpwmClock(const PWM_Type *pwm)
{
    if (pwm == PWM2) {
        return NXP_CLK_FLEXPWM2;
    } else if (pwm == PWM3) {
        return NXP_CLK_FLEXPWM3;
    } else if (pwm == PWM4) {
        return NXP_CLK_FLEXPWM4;
    }
    return NXP_CLK_FLEXPWM1;
}

// XBARA1 input index of the QTimer3 channel 0 output, without the vendor
// enum's 0x100 group tag.
#if defined(IMXRT11XX)
#define XBARA1_IN_QTIMER3_TMR0      (kXBARA1_InputQtimer3Timer0 & 0xFF)
#elif defined(IMXRT10XX)
#define XBARA1_IN_QTIMER3_TMR0      (kXBARA1_InputQtimer3Tmr0Output & 0xFF)
#else
#error "Unsupported i.MXRT family: no XBARA1 input index for the QuadTimer3 output"
#endif

void flexpwmExtClockInit(void)
{
    // An odd divisor gives COMP1 the longer half.
    const uint32_t divisor = flexpwmExtClockDivisor();
    if (divisor < 2) {
        return;
    }

    nxpClockGateEnable(NXP_CLK_TMR3);
    nxpClockGateEnable(NXP_CLK_XBARA1);

    TMR_Type *qtimer = TMR3;

    // Stop the channel before staging. COMP registers hold (N - 1) counts.
    qtimer->CHANNEL[0].CTRL = 0;
    qtimer->CHANNEL[0].COMP1 = (uint16_t)((divisor + 1) / 2 - 1);
    qtimer->CHANNEL[0].COMP2 = (uint16_t)(divisor / 2 - 1);
    qtimer->CHANNEL[0].CAPT = 0;
    qtimer->CHANNEL[0].LOAD = 0;
    qtimer->CHANNEL[0].CNTR = 0;
    qtimer->CHANNEL[0].SCTRL = TMR_SCTRL_OEN(1);        // drive the output pin
    qtimer->CHANNEL[0].CMPLD1 = 0;
    qtimer->CHANNEL[0].CMPLD2 = 0;
    qtimer->CHANNEL[0].CSCTRL = 0;
    qtimer->CHANNEL[0].FILT = 0;
    qtimer->CHANNEL[0].DMA = 0;

    // Count the peripheral clock (CM 1, PCS 8), reinitialise at each compare
    // and toggle the output on alternating compares (OUTMODE 4).
    qtimer->CHANNEL[0].CTRL = TMR_CTRL_CM(1)
                            | TMR_CTRL_PCS(8)
                            | TMR_CTRL_LENGTH(1)
                            | TMR_CTRL_OUTMODE(4);

    // Two crossbar output selects share each SEL register, the even output in
    // the low half and the odd one in the high half.
#if defined(IMXRT11XX)
    // EXT_CLK outputs: FLEXPWM1 57, FLEXPWM2 71, FLEXPWM3 and 4 share 79.
    XBARA1->SEL28 = (XBARA1->SEL28 & ~(uint16_t)XBARA_SEL0_SEL1_MASK)
                  | XBARA_SEL0_SEL1(XBARA1_IN_QTIMER3_TMR0);
    XBARA1->SEL35 = (XBARA1->SEL35 & ~(uint16_t)XBARA_SEL0_SEL1_MASK)
                  | XBARA_SEL0_SEL1(XBARA1_IN_QTIMER3_TMR0);
    XBARA1->SEL39 = (XBARA1->SEL39 & ~(uint16_t)XBARA_SEL0_SEL1_MASK)
                  | XBARA_SEL0_SEL1(XBARA1_IN_QTIMER3_TMR0);
#elif defined(IMXRT10XX)
    // EXT_CLK outputs: FLEXPWM1 34, FLEXPWM2 to 4 share 48.
    XBARA1->SEL17 = (XBARA1->SEL17 & ~(uint16_t)XBARA_SEL0_SEL0_MASK)
                  | XBARA_SEL0_SEL0(XBARA1_IN_QTIMER3_TMR0);
    XBARA1->SEL24 = (XBARA1->SEL24 & ~(uint16_t)XBARA_SEL0_SEL0_MASK)
                  | XBARA_SEL0_SEL0(XBARA1_IN_QTIMER3_TMR0);
#else
#error "Unsupported i.MXRT family: no XBARA1 output selects for the FlexPWM EXT_CLK inputs"
#endif
}

void flexpwmInstanceInit(PWM_Type *pwm)
{
    // The external clock is chip wide, so bring it up before any submodule.
    static bool extClockReady = false;
    if (!extClockReady) {
        flexpwmExtClockInit();
        extClockReady = true;
    }

    nxpClockGateEnable(flexpwmClock(pwm));

    // Clear any latched fault and drive the pins from the PWM generator.
    pwm->FSTS = PWM_FSTS_FFLAG_MASK;
    pwm->DTSRCSEL = 0;
}

bool flexpwmSubmoduleInit(PWM_Type *pwm, uint8_t sm, uint8_t prescaler, uint16_t modulo)
{
    const uint16_t smBit = 1u << sm;

    if (pwm->MCTRL & PWM_MCTRL_RUN(smBit)) {
        return (pwm->SM[sm].CTRL & PWM_CTRL_PRSC_MASK) == PWM_CTRL_PRSC(prescaler)
            && pwm->SM[sm].VAL1 == (uint16_t)(modulo - 1);
    }

    // Hold off any load that is still pending while the values are staged.
    pwm->MCTRL |= PWM_MCTRL_CLDOK(smBit);

    // EXT_CLK clock, independent PWMA and PWMB, counting while halted.
    pwm->SM[sm].CTRL2 = PWM_CTRL2_CLK_SEL(1) | PWM_CTRL2_INDEP_MASK | PWM_CTRL2_DBGEN_MASK;

    // Values set with LDOK load at the end of the PWM period.
    pwm->SM[sm].CTRL = PWM_CTRL_FULL_MASK | PWM_CTRL_PRSC(prescaler);

    // Both outputs active high, nothing forced.
    pwm->SM[sm].OCTRL = 0;

    // Deadtime counters reset to 0x07FF.
    pwm->SM[sm].DTCNT0 = 0;
    pwm->SM[sm].DTCNT1 = 0;

    pwm->SM[sm].INIT = 0;
    pwm->SM[sm].VAL0 = 0;
    pwm->SM[sm].VAL1 = modulo - 1;
    pwm->SM[sm].VAL2 = 0;
    pwm->SM[sm].VAL3 = 0;
    pwm->SM[sm].VAL4 = 0;
    pwm->SM[sm].VAL5 = 0;

    // Unmap the fault sources, which at reset hold the pins low; the top
    // nibble is reserved and reads as ones.
    pwm->SM[sm].DISMAP[0] = (uint16_t)~(PWM_DISMAP_DIS0A_MASK | PWM_DISMAP_DIS0B_MASK | PWM_DISMAP_DIS0X_MASK);

    return true;
}

void flexpwmSubmoduleForceInitEnable(PWM_Type *pwm, uint8_t sm)
{
    pwm->SM[sm].CTRL2 = (pwm->SM[sm].CTRL2 & ~PWM_CTRL2_FORCE_SEL_MASK) | PWM_CTRL2_FRCEN_MASK;
}

void flexpwmOutputEnable(PWM_Type *pwm, uint8_t sm, bool channelB)
{
    const uint16_t smBit = 1u << sm;
    pwm->OUTEN |= channelB ? PWM_OUTEN_PWMB_EN(smBit) : PWM_OUTEN_PWMA_EN(smBit);
}
