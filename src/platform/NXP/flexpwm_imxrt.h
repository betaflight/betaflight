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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "platform/nxp_clock.h"

/*
 * FlexPWM submodule setup for the motor, servo and beeper outputs. Edge aligned: VAL1 is
 * the period, VAL3/VAL5 the PWMA/PWMB width. VAL and CTRL load on MCTRL[LDOK], so clear it
 * with CLDOK, stage, then set it. Submodules count EXT_CLK, as only submodule 0 has IPBus.
 */

// CTRL[PRSC] divides the counter clock by 1, 2, 4 ... 128.
#define FLEXPWM_PRESCALER_MAX       7

// Counter is signed 16 bit and INIT is zero, so the period tops out here.
#define FLEXPWM_MODULO_MAX          32768u

// Shortest period with any duty cycle resolution.
#define FLEXPWM_MODULO_MIN          2u

#define FLEXPWM_INSTANCE_COUNT      4

// The counter tick both i.MXRT families run the FlexPWM submodules at.
#define FLEXPWM_EXT_CLK_HZ          16000000u

static inline uint32_t flexpwmTickRate(uint32_t clockHz, uint8_t prescaler)
{
    return clockHz >> prescaler;
}

// Bus cycles in one EXT_CLK period, which is COMP1 + COMP2 counts of QuadTimer3
// channel 0: 144 MHz / 9 or 240 MHz / 15. Zero when the clock is unknown.
uint32_t flexpwmExtClockDivisor(void);

// Counter tick rate of the submodules.
uint32_t flexpwmClockHz(void);

// Pick the fastest counter tick that still fits one period of rateHz into the
// counter. False when no prescaler works.
bool flexpwmDeriveTiming(uint32_t clockHz, uint32_t rateHz, uint8_t *prescaler, uint16_t *modulo);

// Bring up the chip wide FlexPWM external clock.
void flexpwmExtClockInit(void);

// Instance wide setup; brings the external clock up on the first call.
void flexpwmInstanceInit(PWM_Type *pwm);

// Start with flexpwmLoadOk() and flexpwmRun(). A running submodule is left alone and
// the return value says whether it already has this period.
bool flexpwmSubmoduleInit(PWM_Type *pwm, uint8_t sm, uint8_t prescaler, uint16_t modulo);

// Arm a submodule for one pulse per update: FRCEN turns its local CTRL2[FORCE]
// into a counter init.
void flexpwmSubmoduleForceInitEnable(PWM_Type *pwm, uint8_t sm);

// Connect a submodule output to its pin. Muxing the pad is a separate step.
void flexpwmOutputEnable(PWM_Type *pwm, uint8_t sm, bool channelB);

// Stage a pulse width, in counter ticks. Nothing moves on the pin until LDOK.
static inline void flexpwmStagePulse(PWM_Type *pwm, uint8_t sm, bool channelB, uint16_t ticks)
{
    if (channelB) {
        pwm->SM[sm].VAL5 = ticks;
    } else {
        pwm->SM[sm].VAL3 = ticks;
    }
}

// Drop any pending load; CLDOK reads as zero, so read modify write is safe.
static inline void flexpwmClearLoadOk(PWM_Type *pwm, uint16_t smMask)
{
    pwm->MCTRL |= PWM_MCTRL_CLDOK(smMask);
}

// Let the staged values take effect at the end of the current PWM period.
static inline void flexpwmLoadOk(PWM_Type *pwm, uint16_t smMask)
{
    pwm->MCTRL |= PWM_MCTRL_LDOK(smMask);
}

// Fire one pulse now: stage the width and set LDOK first, since the forced
// init also loads the buffered values (RM i.MX RT1060 Rev. 4, 55.8.4.3).
static inline void flexpwmForceInit(PWM_Type *pwm, uint16_t smMask)
{
    for (uint8_t sm = 0; sm < PWM_SM_COUNT; sm++) {
        if (smMask & (1u << sm)) {
            pwm->SM[sm].CTRL2 |= PWM_CTRL2_FORCE_MASK;
        }
    }
}

// Stage a zero pulse on both outputs, so a reload before the next synced
// update repeats nothing.
static inline void flexpwmStageOff(PWM_Type *pwm, uint16_t smMask)
{
    for (uint8_t sm = 0; sm < PWM_SM_COUNT; sm++) {
        if (smMask & (1u << sm)) {
            pwm->SM[sm].VAL3 = 0;
            pwm->SM[sm].VAL5 = 0;
        }
    }
}

static inline void flexpwmRun(PWM_Type *pwm, uint16_t smMask)
{
    pwm->MCTRL |= PWM_MCTRL_RUN(smMask);
}

static inline void flexpwmStop(PWM_Type *pwm, uint16_t smMask)
{
    pwm->MCTRL &= ~PWM_MCTRL_RUN(smMask);
}
