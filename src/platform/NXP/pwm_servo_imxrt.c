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

// FlexPWM servo outputs; each servo loads its own pulse width when written.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/servo_impl.h"

#include "flexpwm_imxrt.h"
#include "flexpwm_pins_imxrt.h"

typedef struct flexpwmServo_s {
    const imxrtFlexpwmPin_t *pad;
    uint16_t modulo;            // period in counter ticks
    float ticksPerMicrosecond;
} flexpwmServo_t;

static FAST_DATA_ZERO_INIT flexpwmServo_t flexpwmServos[MAX_SUPPORTED_SERVOS];

void servoDevInit(const servoDevConfig_t *servoDevConfig)
{
    if (!servoDevConfig) {
        return;
    }

    const uint32_t clockHz = flexpwmClockHz();

    uint8_t prescaler;
    uint16_t modulo;
    if (!flexpwmDeriveTiming(clockHz, servoDevConfig->servoPwmRate, &prescaler, &modulo)) {
        return;
    }

    const uint32_t tickHz = flexpwmTickRate(clockHz, prescaler);

    for (unsigned index = 0; index < MAX_SUPPORTED_SERVOS; index++) {
        const ioTag_t tag = servoDevConfig->ioTags[index];
        const imxrtFlexpwmPin_t *pad = flexpwmFindPad(tag);
        const IO_t servoIO = IOGetByTag(tag);

        if (!pad || !servoIO || !IOIsFreeOrPreinit(servoIO)) {
            continue;
        }

        PWM_Type *pwm = flexpwmPadReg(pad);

        flexpwmInstanceInit(pwm);

        // A submodule already running at another rate drops the servo.
        if (!flexpwmSubmoduleInit(pwm, pad->submodule, prescaler, modulo)) {
            continue;
        }

        IOInit(servoIO, OWNER_SERVO, RESOURCE_INDEX(index));

        flexpwmOutputEnable(pwm, pad->submodule, flexpwmPadIsChannelB(pad));
        nxpIomuxConfigPin(&pad->mux, IOCFG_AF_PP);

        flexpwmServos[index].pad = pad;
        flexpwmServos[index].modulo = modulo;
        flexpwmServos[index].ticksPerMicrosecond = (float)tickHz / 1e6f;

        // Start at the centre pulse so a servo does not slam to an end stop.
        servoWrite(index, servoDevConfig->servoCenterPulse);

        flexpwmRun(pwm, 1u << pad->submodule);
    }
}

void servoWrite(uint8_t index, float value)
{
    if (index >= MAX_SUPPORTED_SERVOS) {
        return;
    }

    const flexpwmServo_t *servo = &flexpwmServos[index];
    if (!servo->pad) {
        return;
    }

    const float pulseUs = constrainf(value, PWM_SERVO_MIN, PWM_SERVO_MAX);
    const uint32_t ticks = constrain(lrintf(pulseUs * servo->ticksPerMicrosecond), 0, servo->modulo);

    const uint16_t smBit = 1u << servo->pad->submodule;
    PWM_Type *pwm = flexpwmPadReg(servo->pad);

    flexpwmClearLoadOk(pwm, smBit);
    flexpwmStagePulse(pwm, servo->pad->submodule, flexpwmPadIsChannelB(servo->pad), ticks);
    flexpwmLoadOk(pwm, smBit);
}

#endif // USE_SERVOS
