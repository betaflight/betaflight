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
 * FlexPWM motor outputs: PWM, Brushed, OneShot125, OneShot42 and Multishot.
 * Continuous mode loads the pulse width at each period boundary; synced mode
 * runs the counter at its longest period and fires one pulse per update with
 * a software FORCE (see flexpwmForceInit).
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_PWM_OUTPUT

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/motor_types.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_impl.h"

#include "pg/motor.h"

#include "flexpwm_imxrt.h"
#include "flexpwm_pins_imxrt.h"

typedef struct flexpwmMotor_s {
    const imxrtFlexpwmPin_t *pad;
    PWM_Type *pwm;              // kept here so the write path reads no flash table
    uint8_t submodule;
    bool channelB;
    uint16_t modulo;            // period in counter ticks, so also the full on level
} flexpwmMotor_t;

// One entry per FlexPWM instance carrying motors.
typedef struct flexpwmGroup_s {
    PWM_Type *pwm;
    uint16_t smMask;
} flexpwmGroup_t;

static FAST_DATA_ZERO_INIT flexpwmMotor_t flexpwmMotors[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT flexpwmGroup_t flexpwmGroups[FLEXPWM_INSTANCE_COUNT];
static FAST_DATA_ZERO_INIT uint8_t flexpwmGroupCount;

// One pulse per update rather than a free running period.
static FAST_DATA_ZERO_INIT bool flexpwmSynced;

static void flexpwmAddToGroup(PWM_Type *pwm, uint8_t submodule)
{
    for (unsigned i = 0; i < flexpwmGroupCount; i++) {
        if (flexpwmGroups[i].pwm == pwm) {
            flexpwmGroups[i].smMask |= 1u << submodule;
            return;
        }
    }

    flexpwmGroups[flexpwmGroupCount].pwm = pwm;
    flexpwmGroups[flexpwmGroupCount].smMask = 1u << submodule;
    flexpwmGroupCount++;
}

static FAST_CODE void pwmWriteStandard(uint8_t index, float value)
{
    const flexpwmMotor_t *motor = &flexpwmMotors[index];

    if (!motor->pwm) {
        return;
    }

    const uint32_t ticks = constrain(lrintf((value * pwmMotors[index].pulseScale) + pwmMotors[index].pulseOffset),
                                     0, motor->modulo);

    flexpwmClearLoadOk(motor->pwm, 1u << motor->submodule);
    flexpwmStagePulse(motor->pwm, motor->submodule, motor->channelB, ticks);
}

static FAST_CODE void pwmCompleteMotorUpdate(void)
{
    // Synced mode needs LDOK set before the FORCE below.
    for (unsigned i = 0; i < flexpwmGroupCount; i++) {
        flexpwmLoadOk(flexpwmGroups[i].pwm, flexpwmGroups[i].smMask);
    }

    if (!flexpwmSynced) {
        return;
    }

    for (unsigned i = 0; i < flexpwmGroupCount; i++) {
        PWM_Type *pwm = flexpwmGroups[i].pwm;
        const uint16_t smMask = flexpwmGroups[i].smMask;

        flexpwmForceInit(pwm, smMask);

        // Overrun guard: a reload before the next update outputs nothing.
        flexpwmClearLoadOk(pwm, smMask);
        flexpwmStageOff(pwm, smMask);
        flexpwmLoadOk(pwm, smMask);
    }
}

void pwmShutdownPulsesForAllMotors(void)
{
    for (unsigned index = 0; index < pwmMotorCount; index++) {
        const flexpwmMotor_t *motor = &flexpwmMotors[index];
        if (motor->pwm) {
            flexpwmClearLoadOk(motor->pwm, 1u << motor->submodule);
            flexpwmStagePulse(motor->pwm, motor->submodule, motor->channelB, 0);
        }
    }

    pwmCompleteMotorUpdate();
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors();
}

// ESC passthrough leaves the pads muxed to GPIO, so mux them back.
static bool pwmEnableMotorOutputs(void)
{
    for (unsigned index = 0; index < pwmMotorCount; index++) {
        const flexpwmMotor_t *motor = &flexpwmMotors[index];
        if (motor->pad) {
            nxpIomuxConfigPin(&motor->pad->mux, IOCFG_AF_PP);
        }
    }

    return pwmEnableMotors();
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static FAST_RODATA const motorVTable_t motorPwmVTable = {
    .postInit = NULL,
    .enable = pwmEnableMotorOutputs,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .convertExternalToMotor = pwmConvertFromExternal,
    .convertMotorToExternal = pwmConvertToExternal,
    .write = pwmWriteStandard,
    .decodeTelemetry = NULL,
    .updateComplete = pwmCompleteMotorUpdate,
    .requestTelemetry = NULL,
    .isMotorIdle = NULL,
    .getMotorIO = pwmGetMotorIO,
};

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t idlePulse)
{
    UNUSED(idlePulse);

    if (!device || !motorConfig) {
        return false;
    }

    memset(pwmMotors, 0, sizeof(pwmMotors));
    memset(flexpwmMotors, 0, sizeof(flexpwmMotors));
    memset(flexpwmGroups, 0, sizeof(flexpwmGroups));
    flexpwmGroupCount = 0;
    pwmMotorCount = 0;

    // sMin is the pulse length at throttle 1000, sLen the extra at 2000.
    float sMin;
    float sLen;
    bool continuous = motorConfig->useContinuousUpdate;

    switch (motorConfig->motorProtocol) {
    case MOTOR_PROTOCOL_PWM:
        sMin = 1e-3f;
        sLen = 1e-3f;
        continuous = true;
        break;
    case MOTOR_PROTOCOL_BRUSHED:
        sMin = 0.0f;
        sLen = 0.0f;
        continuous = true;
        break;
    case MOTOR_PROTOCOL_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case MOTOR_PROTOCOL_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case MOTOR_PROTOCOL_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    default:
        return false;
    }

    flexpwmSynced = !continuous;

    const uint32_t clockHz = flexpwmClockHz();

    // Synced mode takes its prescaler from four longest pulses per period.
    const uint32_t pwmRateHz = continuous ? motorConfig->motorPwmRate
                                          : (uint32_t)ceilf(1.0f / ((sMin + sLen) * 4.0f));

    uint8_t prescaler;
    uint16_t modulo;
    if (!flexpwmDeriveTiming(clockHz, pwmRateHz, &prescaler, &modulo)) {
        return false;
    }

    // Synced mode runs to the longest period, at least 2 ms at 16 MHz.
    if (flexpwmSynced) {
        modulo = FLEXPWM_MODULO_MAX;
    }

    const uint32_t tickHz = flexpwmTickRate(clockHz, prescaler);

    for (unsigned motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < device->count; motorIndex++) {
        const unsigned reorderedMotorIndex = motorConfig->motorOutputReordering[motorIndex];
        const ioTag_t tag = motorConfig->ioTags[reorderedMotorIndex];

        const imxrtFlexpwmPin_t *pad = flexpwmFindPad(tag);
        pwmMotors[motorIndex].io = IOGetByTag(tag);

        bool padUsable = pad && pwmMotors[motorIndex].io && IOIsFreeOrPreinit(pwmMotors[motorIndex].io);

        PWM_Type *pwm = NULL;
        if (padUsable) {
            pwm = flexpwmPadReg(pad);
            flexpwmInstanceInit(pwm);
            padUsable = flexpwmSubmoduleInit(pwm, pad->submodule, prescaler, modulo);
        }

        if (!padUsable) {
            // Release the pads claimed so far.
            while (motorIndex--) {
                IORelease(pwmMotors[motorIndex].io);
            }

            device->vTable = NULL;
            pwmMotorCount = 0;
            return false;
        }

        IOInit(pwmMotors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));

        flexpwmOutputEnable(pwm, pad->submodule, flexpwmPadIsChannelB(pad));

        if (flexpwmSynced) {
            flexpwmSubmoduleForceInitEnable(pwm, pad->submodule);
        }

        nxpIomuxConfigPin(&pad->mux, IOCFG_AF_PP);

        flexpwmMotors[motorIndex].pad = pad;
        flexpwmMotors[motorIndex].pwm = pwm;
        flexpwmMotors[motorIndex].submodule = pad->submodule;
        flexpwmMotors[motorIndex].channelB = flexpwmPadIsChannelB(pad);
        flexpwmMotors[motorIndex].modulo = modulo;
        flexpwmAddToGroup(pwm, pad->submodule);

        // Brushed spans the whole period, so its scale is the modulo.
        pwmMotors[motorIndex].pulseScale = ((motorConfig->motorProtocol == MOTOR_PROTOCOL_BRUSHED)
                                            ? (float)modulo
                                            : (sLen * tickHz)) / 1000.0f;
        pwmMotors[motorIndex].pulseOffset = (sMin * tickHz) - (pwmMotors[motorIndex].pulseScale * 1000.0f);
        pwmMotors[motorIndex].enabled = true;

        pwmMotorCount++;
    }

    if (pwmMotorCount == 0) {
        return false;
    }

    // Invert only each motor pin, after every submodule setup has cleared OCTRL.
    for (unsigned i = 0; motorConfig->motorInversion && i < pwmMotorCount; i++) {
        const flexpwmMotor_t *motor = &flexpwmMotors[i];
        motor->pwm->SM[motor->submodule].OCTRL |= motor->channelB ? PWM_OCTRL_POLB_MASK : PWM_OCTRL_POLA_MASK;
    }

    // Start with a zero pulse staged.
    for (unsigned i = 0; i < flexpwmGroupCount; i++) {
        flexpwmLoadOk(flexpwmGroups[i].pwm, flexpwmGroups[i].smMask);
        flexpwmRun(flexpwmGroups[i].pwm, flexpwmGroups[i].smMask);
    }

    device->vTable = &motorPwmVTable;

    return true;
}

#endif // USE_PWM_OUTPUT
