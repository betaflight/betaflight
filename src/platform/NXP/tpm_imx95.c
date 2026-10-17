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
 * TPM motor, servo and beeper outputs, edge aligned: MOD is the period minus one, CnV the pulse width.
 * The channels of a TPM share its counter and period; CnV loads on the wrap, so no pulse is cut short.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_PWM_OUTPUT) || defined(USE_SERVOS) || defined(USE_BEEPER)

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/motor_types.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_impl.h"
#include "drivers/servo_impl.h"
#include "drivers/sound_beeper.h"

#include "pg/motor.h"

#include "platform/nxp_clock.h"

#include "iomux_nxp.h"
#include "iomux_signal_def.h"

// Shortest period with any duty cycle resolution.
#define TPM_PERIOD_MIN          2u

// SC[CMOD]: count every tick of the clock root.
#define TPM_CMOD_COUNT          1

// CONF[DBGMODE]: keep counting while the core is halted.
#define TPM_DBGMODE_RUN         3

// The TPM channel row of a pad, or NULL; EXTCLK rows are skipped.
static const nxpSignalPin_t *tpmFindPad(ioTag_t tag)
{
    if (!tag) {
        return NULL;
    }

    for (unsigned i = 0; i < ARRAYLEN(nxpTpmPins); i++) {
        const nxpSignalPin_t *pad = &nxpTpmPins[i];
        if (pad->port == DEFIO_TAG_GPIOID(tag) && pad->pin == DEFIO_TAG_PIN(tag)
            && pad->role >= NXP_ROLE_TPM_CH0 && pad->role <= NXP_ROLE_TPM_CH3
            && ((NXP_TPM_INSTANCE_MASK >> pad->instance) & 1)) {
            return pad;
        }
    }

    return NULL;
}

static uint8_t tpmPadChannel(const nxpSignalPin_t *pad)
{
    return pad->role - NXP_ROLE_TPM_CH0;
}

// The vendor table leads with a null entry, so the instance number indexes it.
static TPM_Type *tpmPadReg(const nxpSignalPin_t *pad)
{
    static TPM_Type *const instances[] = TPM_BASE_PTRS;

    return instances[pad->instance];
}

static nxpClock_e tpmPadClock(const nxpSignalPin_t *pad)
{
    return (nxpClock_e)(NXP_CLK_TPM1 + (pad->instance - 1));
}

static uint32_t tpmPadClockHz(const nxpSignalPin_t *pad)
{
    return nxpClockHz(tpmPadClock(pad));
}

// The 32 bit counter holds one period of any rate undivided; false when the period is too short.
static bool tpmDeriveTiming(uint32_t clockHz, uint32_t rateHz, uint8_t *prescaler, uint32_t *period)
{
    if (clockHz == 0 || rateHz == 0) {
        return false;
    }

    const uint32_t ticks = (clockHz + rateHz / 2) / rateHz;

    if (ticks < TPM_PERIOD_MIN) {
        return false;
    }

    *prescaler = 0;
    *period = ticks;
    return true;
}

// Starts the TPM at this period with the channel at zero width; a running counter must match the period.
static bool tpmChannelInit(const nxpSignalPin_t *pad, uint8_t prescaler, uint32_t period, bool inverted)
{
    TPM_Type *tpm = tpmPadReg(pad);

    if (!nxpClockGateEnable(tpmPadClock(pad))) {
        return false;
    }

    if (tpm->SC & TPM_SC_CMOD_MASK) {
        if ((tpm->SC & TPM_SC_PS_MASK) != TPM_SC_PS(prescaler) || tpm->MOD != period - 1) {
            return false;
        }
    } else {
        tpm->SC = 0;
        tpm->CONF = TPM_CONF_DBGMODE(TPM_DBGMODE_RUN);
        tpm->CNT = 0;
        tpm->MOD = period - 1;
        tpm->SC = TPM_SC_CMOD(TPM_CMOD_COUNT) | TPM_SC_PS(prescaler);
    }

    // Edge aligned PWM: ELSB drives high from the wrap to the match, ELSA low.
    tpm->CONTROLS[tpmPadChannel(pad)].CnV = 0;
    tpm->CONTROLS[tpmPadChannel(pad)].CnSC = TPM_CnSC_MSB_MASK
                                     | (inverted ? TPM_CnSC_ELSA_MASK : TPM_CnSC_ELSB_MASK);

    return true;
}

static void tpmSetPulse(const nxpSignalPin_t *pad, uint32_t ticks)
{
    tpmPadReg(pad)->CONTROLS[tpmPadChannel(pad)].CnV = ticks;
}

#ifdef USE_PWM_OUTPUT

typedef struct {
    const nxpSignalPin_t *pad;
    uint32_t period;            // in counter ticks, so also the full on level
} tpmMotor_t;

static FAST_DATA_ZERO_INIT tpmMotor_t tpmMotors[MAX_SUPPORTED_MOTORS];

static FAST_CODE void pwmWriteStandard(uint8_t index, float value)
{
    const tpmMotor_t *motor = &tpmMotors[index];

    if (!motor->pad) {
        return;
    }

    const uint32_t ticks = constrain(lrintf((value * pwmMotors[index].pulseScale) + pwmMotors[index].pulseOffset),
                                     0, motor->period);

    tpmSetPulse(motor->pad, ticks);
}

static FAST_CODE void pwmCompleteMotorUpdate(void)
{
}

void pwmShutdownPulsesForAllMotors(void)
{
    for (unsigned index = 0; index < pwmMotorCount; index++) {
        if (tpmMotors[index].pad) {
            tpmSetPulse(tpmMotors[index].pad, 0);
        }
    }
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors();
}

// ESC passthrough leaves the pads muxed to GPIO, so mux them back.
static bool pwmEnableMotorOutputs(void)
{
    for (unsigned index = 0; index < pwmMotorCount; index++) {
        if (tpmMotors[index].pad) {
            nxpIomuxConfigPin(&tpmMotors[index].pad->mux, IOCFG_AF_PP);
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
    memset(tpmMotors, 0, sizeof(tpmMotors));
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

    // Without a forced reload, synced protocols run free at four longest pulses per period.
    const uint32_t pwmRateHz = continuous ? motorConfig->motorPwmRate
                                          : (uint32_t)ceilf(1.0f / ((sMin + sLen) * 4.0f));

    for (unsigned motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < device->count; motorIndex++) {
        const unsigned reorderedMotorIndex = motorConfig->motorOutputReordering[motorIndex];
        const ioTag_t tag = motorConfig->ioTags[reorderedMotorIndex];

        const nxpSignalPin_t *pad = tpmFindPad(tag);
        pwmMotors[motorIndex].io = IOGetByTag(tag);

        uint32_t clockHz = 0;
        uint8_t prescaler = 0;
        uint32_t period = 0;

        bool padUsable = pad && pwmMotors[motorIndex].io && IOIsFreeOrPreinit(pwmMotors[motorIndex].io);

        if (padUsable) {
            clockHz = tpmPadClockHz(pad);
            padUsable = tpmDeriveTiming(clockHz, pwmRateHz, &prescaler, &period)
                     && tpmChannelInit(pad, prescaler, period, motorConfig->motorInversion);
        }

        if (!padUsable) {
            // Stop the channels started so far and release their pads.
            while (motorIndex--) {
                const nxpSignalPin_t *started = tpmMotors[motorIndex].pad;
                tpmPadReg(started)->CONTROLS[tpmPadChannel(started)].CnSC = 0;
                tpmMotors[motorIndex].pad = NULL;
                IORelease(pwmMotors[motorIndex].io);
            }

            device->vTable = NULL;
            pwmMotorCount = 0;
            return false;
        }

        IOInit(pwmMotors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));
        nxpIomuxConfigPin(&pad->mux, IOCFG_AF_PP);

        tpmMotors[motorIndex].pad = pad;
        tpmMotors[motorIndex].period = period;

        const uint32_t tickHz = clockHz >> prescaler;

        // Brushed spans the whole period, so its scale is the period.
        pwmMotors[motorIndex].pulseScale = ((motorConfig->motorProtocol == MOTOR_PROTOCOL_BRUSHED)
                                            ? (float)period
                                            : (sLen * tickHz)) / 1000.0f;
        pwmMotors[motorIndex].pulseOffset = (sMin * tickHz) - (pwmMotors[motorIndex].pulseScale * 1000.0f);
        pwmMotors[motorIndex].enabled = true;

        pwmMotorCount++;
    }

    if (pwmMotorCount == 0) {
        return false;
    }

    device->vTable = &motorPwmVTable;

    return true;
}

#endif // USE_PWM_OUTPUT

#ifdef USE_SERVOS

typedef struct {
    const nxpSignalPin_t *pad;
    uint32_t period;            // in counter ticks
    float ticksPerMicrosecond;
} tpmServo_t;

static FAST_DATA_ZERO_INIT tpmServo_t tpmServos[MAX_SUPPORTED_SERVOS];

void servoDevInit(const servoDevConfig_t *servoDevConfig)
{
    if (!servoDevConfig) {
        return;
    }

    for (unsigned index = 0; index < MAX_SUPPORTED_SERVOS; index++) {
        const ioTag_t tag = servoDevConfig->ioTags[index];
        const nxpSignalPin_t *pad = tpmFindPad(tag);
        const IO_t servoIO = IOGetByTag(tag);

        if (!pad || !servoIO || !IOIsFreeOrPreinit(servoIO)) {
            continue;
        }

        const uint32_t clockHz = tpmPadClockHz(pad);

        uint8_t prescaler;
        uint32_t period;

        // A TPM already running at another rate drops the servo.
        if (!tpmDeriveTiming(clockHz, servoDevConfig->servoPwmRate, &prescaler, &period)
            || !tpmChannelInit(pad, prescaler, period, false)) {
            continue;
        }

        IOInit(servoIO, OWNER_SERVO, RESOURCE_INDEX(index));
        nxpIomuxConfigPin(&pad->mux, IOCFG_AF_PP);

        tpmServos[index].pad = pad;
        tpmServos[index].period = period;
        tpmServos[index].ticksPerMicrosecond = (float)(clockHz >> prescaler) / 1e6f;

        // Start at the centre pulse so a servo does not slam to an end stop.
        servoWrite(index, servoDevConfig->servoCenterPulse);
    }
}

void servoWrite(uint8_t index, float value)
{
    if (index >= MAX_SUPPORTED_SERVOS || !tpmServos[index].pad) {
        return;
    }

    const tpmServo_t *servo = &tpmServos[index];
    const float pulseUs = constrainf(value, PWM_SERVO_MIN, PWM_SERVO_MAX);

    tpmSetPulse(servo->pad, constrain(lrintf(pulseUs * servo->ticksPerMicrosecond), 0, servo->period));
}

#endif // USE_SERVOS

#if defined(USE_BEEPER) && defined(USE_PWM_OUTPUT)

// A tone at the beeper frequency, switched by a pulse width of half the period or zero.

static const nxpSignalPin_t *beeperPad;
static uint32_t beeperHalfPeriod;
static bool beeperOn;

void beeperPwmInit(const ioTag_t tag, uint16_t frequency)
{
    beeperPad = NULL;

    const IO_t beeperIO = IOGetByTag(tag);
    const nxpSignalPin_t *pad = tpmFindPad(tag);

    if (!beeperIO || !pad || !frequency || !IOIsFreeOrPreinit(beeperIO)) {
        return;
    }

    uint8_t prescaler;
    uint32_t period;
    if (!tpmDeriveTiming(tpmPadClockHz(pad), frequency, &prescaler, &period)
        || !tpmChannelInit(pad, prescaler, period, false)) {
        return;
    }

    IOInit(beeperIO, OWNER_BEEPER, 0);
    nxpIomuxConfigPin(&pad->mux, IOCFG_AF_PP);

    beeperPad = pad;
    beeperHalfPeriod = period / 2;
    beeperOn = false;
}

void pwmWriteBeeper(bool onoff)
{
    if (!beeperPad) {
        return;
    }

    tpmSetPulse(beeperPad, onoff ? beeperHalfPeriod : 0);
    beeperOn = onoff;
}

void pwmToggleBeeper(void)
{
    pwmWriteBeeper(!beeperOn);
}

#endif // USE_BEEPER && USE_PWM_OUTPUT

#endif // USE_PWM_OUTPUT || USE_SERVOS || USE_BEEPER
