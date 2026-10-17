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

// Beeper tone from FlexPWM, or from a GPT output compare when the board
// defines BEEPER_GPT_INSTANCE for a pad with no FlexPWM output.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_BEEPER) && defined(USE_PWM_OUTPUT)

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/sound_beeper.h"

#if defined(BEEPER_GPT_INSTANCE)

/*
 * GPT output compare tone: channel 1 resets the counter every half wave and
 * the buzzer channel (2 or 3, compare below OCR1) toggles the pin once per
 * reset. Off switches the compare action from toggle to clear.
 */

#include "platform/nxp_clock.h"

#include "iomux_nxp.h"
#include "iomux_signal_def.h"

STATIC_ASSERT(DEFIO_TAG_GPIOID(IO_TAG(BEEPER_PIN)) < NXP_IOMUX_PORT_COUNT,
              beeper_pin_is_not_on_a_port_the_pad_table_covers);

STATIC_ASSERT(BEEPER_GPT_CHANNEL == 2 || BEEPER_GPT_CHANNEL == 3,
              beeper_gpt_channel_1_sets_the_period);

// GPT2 is the free running microsecond counter every delay reads.
STATIC_ASSERT(BEEPER_GPT_INSTANCE != 2, gpt2_is_the_delay_time_base);

#if defined(IMXRT10XX)
STATIC_ASSERT(BEEPER_GPT_INSTANCE == 1 || BEEPER_GPT_INSTANCE == 2,
              the_10xx_parts_have_only_gpt1_and_gpt2);
#else
STATIC_ASSERT(BEEPER_GPT_INSTANCE >= 1 && BEEPER_GPT_INSTANCE <= 6,
              the_part_has_gpt1_to_gpt6);
#endif

#define BEEPER_GPT              CONCAT(GPT, BEEPER_GPT_INSTANCE)
#define BEEPER_GPT_CLOCK        CONCAT(NXP_CLK_GPT, BEEPER_GPT_INSTANCE)
#define BEEPER_GPT_OM(x)        CONCAT(GPT_CR_OM, BEEPER_GPT_CHANNEL)(x)
#define BEEPER_GPT_OM_MASK      CONCAT3(GPT_CR_OM, BEEPER_GPT_CHANNEL, _MASK)
#define BEEPER_GPT_FORCE(x)     CONCAT(GPT_CR_FO, BEEPER_GPT_CHANNEL)(x)

// GPT_CR[CLKSRC]: the instance's clock root.
#define GPT_CLKSRC_PERIPHERAL   1

// GPT_CR[OMn]: what a compare event does to the output pin.
#define GPT_OUTPUT_TOGGLE       1
#define GPT_OUTPUT_CLEAR        2

static bool beeperReady;
static bool beeperOn;

void beeperPwmInit(const ioTag_t tag, uint16_t frequency)
{
    beeperReady = false;

    const IO_t beeperIO = IOGetByTag(tag);

    // BEEPER_GPT_MUX_MODE is valid for BEEPER_PIN only.
    if (!beeperIO || tag != IO_TAG(BEEPER_PIN) || !frequency || !IOIsFreeOrPreinit(beeperIO)) {
        return;
    }

    // A GPT compare is an output, so it needs no input select register.
    const uint32_t padBase = nxpIomuxPortBase[DEFIO_TAG_GPIOID(tag)];
    const nxpGpioPad_t *pad = &nxpGpioPads[DEFIO_TAG_GPIOID(tag)][DEFIO_TAG_PIN(tag)];

    const nxpPinMux_t beeperMux = {
        .muxRegister = padBase + pad->muxRegOffset,
        .muxMode = BEEPER_GPT_MUX_MODE,
        .configRegister = padBase + pad->padCtlRegOffset,
    };

    nxpClockGateEnable(BEEPER_GPT_CLOCK);

    // Half a wave in counter ticks; every audible tone fits the 32 bit counter.
    const uint32_t clockHz = nxpClockHz(BEEPER_GPT_CLOCK);
    const uint32_t ticks = (clockHz + frequency) / (2u * frequency);

    if (ticks < 2) {
        return;
    }

    GPT_Type *gpt = BEEPER_GPT;

    gpt->CR = 0;                        // stop the counter before staging
    gpt->IR = 0;
    gpt->SR = gpt->SR;                  // status bits are write-one-to-clear
    gpt->PR = GPT_PR_PRESCALER(0);      // divide by one

    gpt->OCR[BEEPER_GPT_CHANNEL - 1] = ticks / 2;
    gpt->OCR[0] = ticks - 1;

    IOInit(beeperIO, OWNER_BEEPER, 0);
    nxpIomuxConfigPin(&beeperMux, IOCFG_AF_PP);

    // Start with the tone off, so the first compare takes the pin low.
    gpt->CR = GPT_CR_CLKSRC(GPT_CLKSRC_PERIPHERAL) | GPT_CR_ENMOD_MASK
            | BEEPER_GPT_OM(GPT_OUTPUT_CLEAR);
    gpt->CR |= GPT_CR_EN_MASK;

    beeperReady = true;
    beeperOn = false;
}

void pwmWriteBeeper(bool onoff)
{
    if (!beeperReady) {
        return;
    }

    GPT_Type *gpt = BEEPER_GPT;

    uint32_t cr = (gpt->CR & ~(uint32_t)BEEPER_GPT_OM_MASK)
                | BEEPER_GPT_OM(onoff ? GPT_OUTPUT_TOGGLE : GPT_OUTPUT_CLEAR);

    if (!onoff) {
        // Take the pin low now; the force bit clears itself.
        cr |= BEEPER_GPT_FORCE(1);
    }

    gpt->CR = cr;

    beeperOn = onoff;
}

void pwmToggleBeeper(void)
{
    pwmWriteBeeper(!beeperOn);
}

#else // BEEPER_GPT_INSTANCE

// FlexPWM tone at the tone frequency, switched by a pulse width of half the
// period or zero.

#include "drivers/pwm_output.h"

#include "flexpwm_imxrt.h"
#include "flexpwm_pins_imxrt.h"

static const imxrtFlexpwmPin_t *beeperPad;
static uint16_t beeperHalfPeriod;
static bool beeperOn;

void beeperPwmInit(const ioTag_t tag, uint16_t frequency)
{
    beeperPad = NULL;

    const IO_t beeperIO = IOGetByTag(tag);
    const imxrtFlexpwmPin_t *pad = flexpwmFindPad(tag);

    if (!beeperIO || !pad || !frequency || !IOIsFreeOrPreinit(beeperIO)) {
        return;
    }

    const uint32_t clockHz = flexpwmClockHz();

    uint8_t prescaler;
    uint16_t modulo;
    if (!flexpwmDeriveTiming(clockHz, frequency, &prescaler, &modulo)) {
        return;
    }

    PWM_Type *pwm = flexpwmPadReg(pad);

    flexpwmInstanceInit(pwm);

    if (!flexpwmSubmoduleInit(pwm, pad->submodule, prescaler, modulo)) {
        return;
    }

    IOInit(beeperIO, OWNER_BEEPER, 0);
    flexpwmOutputEnable(pwm, pad->submodule, flexpwmPadIsChannelB(pad));
    nxpIomuxConfigPin(&pad->mux, IOCFG_AF_PP);

    beeperPad = pad;
    beeperHalfPeriod = modulo / 2;
    beeperOn = false;

    // Starts low: submodule init staged a zero pulse.
    const uint16_t smBit = 1u << pad->submodule;
    flexpwmLoadOk(pwm, smBit);
    flexpwmRun(pwm, smBit);
}

void pwmWriteBeeper(bool onoff)
{
    if (!beeperPad) {
        return;
    }

    const uint16_t smBit = 1u << beeperPad->submodule;

    PWM_Type *pwm = flexpwmPadReg(beeperPad);

    flexpwmClearLoadOk(pwm, smBit);
    flexpwmStagePulse(pwm, beeperPad->submodule, flexpwmPadIsChannelB(beeperPad), onoff ? beeperHalfPeriod : 0);
    flexpwmLoadOk(pwm, smBit);

    beeperOn = onoff;
}

void pwmToggleBeeper(void)
{
    pwmWriteBeeper(!beeperOn);
}

#endif // BEEPER_GPT_INSTANCE

#endif // USE_BEEPER && USE_PWM_OUTPUT
