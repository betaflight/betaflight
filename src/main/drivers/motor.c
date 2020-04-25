/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MOTOR

#include "common/maths.h"

#include "config/feature.h"

#include "drivers/dshot.h" // for DSHOT_ constants in initEscEndpoints; may be gone in the future

#include "drivers/motor.h"
#include "drivers/pwm_output.h" // for PWM_TYPE_* and others
#include "drivers/time.h"
#include "drivers/dshot_bitbang.h"
#include "drivers/dshot_dpwm.h"

#include "fc/rc_controls.h" // for flight3DConfig_t

#include "pg/motor.h"

static FAST_RAM_ZERO_INIT motorDevice_t *motorDevice;

void motorShutdown(void)
{
    motorDevice->vTable.shutdown();
    motorDevice->enabled = false;
    motorDevice->motorEnableTimeMs = 0;
    motorDevice->initialized = false;
    delayMicroseconds(1500);
}

void motorWriteAll(float *values)
{
#ifdef USE_PWM_OUTPUT
    if (motorDevice->enabled) {
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
        if (!motorDevice->vTable.updateStart()) {
            return;
        }
#endif
        for (int i = 0; i < motorDevice->count; i++) {
            motorDevice->vTable.write(i, values[i]);
        }
        motorDevice->vTable.updateComplete();
    }
#endif
}

int motorDeviceCount(void)
{
    return motorDevice->count;
}

// This is not motor generic anymore; should be moved to analog pwm module
static void analogInitEndpoints(float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow) {
    if (featureIsEnabled(FEATURE_3D)) {
        float outputLimitOffset = (flight3DConfig()->limit3d_high - flight3DConfig()->limit3d_low) * (1 - outputLimit) / 2;
        *disarm = flight3DConfig()->neutral3d;
        *outputLow = flight3DConfig()->limit3d_low + outputLimitOffset;
        *outputHigh = flight3DConfig()->limit3d_high - outputLimitOffset;
        *deadbandMotor3dHigh = flight3DConfig()->deadband3d_high;
        *deadbandMotor3dLow = flight3DConfig()->deadband3d_low;
    } else {
        *disarm = motorConfig()->mincommand;
        *outputLow = motorConfig()->minthrottle;
        *outputHigh = motorConfig()->maxthrottle - ((motorConfig()->maxthrottle - motorConfig()->minthrottle) * (1 - outputLimit));
    }
}

// End point initialization is called from mixerInit before motorDevInit; can't use vtable...
void motorInitEndpoints(float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow)
{
    switch (motorConfig()->dev.motorPwmProtocol) {
#ifdef USE_DSHOT
    case PWM_TYPE_PROSHOT1000:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT150:
        dshotInitEndpoints(outputLimit, outputLow, outputHigh, disarm, deadbandMotor3dHigh, deadbandMotor3dLow);
        break;
#endif
    default:
        analogInitEndpoints(outputLimit, outputLow, outputHigh, disarm, deadbandMotor3dHigh, deadbandMotor3dLow);
        break;
    }
}

float motorConvertFromExternal(uint16_t externalValue)
{
    return motorDevice->vTable.convertExternalToMotor(externalValue);
}

uint16_t motorConvertToExternal(float motorValue)
{
    return motorDevice->vTable.convertMotorToExternal(motorValue);
}

static bool isDshot = false; // XXX Should go somewhere else

void motorPostInit()
{
    motorDevice->vTable.postInit();
}

void motorPostInitNull(void)
{
}

static bool motorEnableNull(void)
{
    return false;
}

static void motorDisableNull(void)
{
}

static bool motorIsEnabledNull(uint8_t index)
{
    UNUSED(index);

    return false;
}

bool motorUpdateStartNull(void)
{
    return true;
}

void motorWriteNull(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}

static void motorWriteIntNull(uint8_t index, uint16_t value)
{
    UNUSED(index);
    UNUSED(value);
}

void motorUpdateCompleteNull(void)
{
}

static void motorShutdownNull(void)
{
}

static float motorConvertFromExternalNull(uint16_t value)
{
    UNUSED(value);
    return 0.0f ;
}

static uint16_t motorConvertToExternalNull(float value)
{
    UNUSED(value);
    return 0;
}

static const motorVTable_t motorNullVTable = {
    .postInit = motorPostInitNull,
    .enable = motorEnableNull,
    .disable = motorDisableNull,
    .isMotorEnabled = motorIsEnabledNull,
    .updateStart = motorUpdateStartNull,
    .write = motorWriteNull,
    .writeInt = motorWriteIntNull,
    .updateComplete = motorUpdateCompleteNull,
    .convertExternalToMotor = motorConvertFromExternalNull,
    .convertMotorToExternal = motorConvertToExternalNull,
    .shutdown = motorShutdownNull,
};

static motorDevice_t motorNullDevice = {
    .initialized = false,
    .enabled = false,
};

void motorDevInit(const motorDevConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount) {
    memset(motors, 0, sizeof(motors));

    bool useUnsyncedPwm = motorConfig->useUnsyncedPwm;

    switch (motorConfig->motorPwmProtocol) {
    default:
    case PWM_TYPE_STANDARD:
    case PWM_TYPE_ONESHOT125:
    case PWM_TYPE_ONESHOT42:
    case PWM_TYPE_MULTISHOT:
    case PWM_TYPE_BRUSHED:
        motorDevice = motorPwmDevInit(motorConfig, idlePulse, motorCount, useUnsyncedPwm);
        break;

#ifdef USE_DSHOT
    case PWM_TYPE_DSHOT150:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_PROSHOT1000:
#ifdef USE_DSHOT_BITBANG
        if (isDshotBitbangActive(motorConfig)) {
            motorDevice = dshotBitbangDevInit(motorConfig, motorCount);
        } else
#endif
        {
            motorDevice = dshotPwmDevInit(motorConfig, idlePulse, motorCount, useUnsyncedPwm);
        }

        isDshot = true;
        break;
#endif

#if 0 // not yet
    case PWM_TYPE_DSHOT_UART:
        //motorDevice = dshotSerialInit(motorConfig, idlePulse, motorCount, useUnsyncedPwm);
        break;
#endif
    }

    if (motorDevice) {
        motorDevice->count = motorCount;
        motorDevice->initialized = true;
        motorDevice->motorEnableTimeMs = 0;
        motorDevice->enabled = false;
    } else {
        motorNullDevice.vTable = motorNullVTable;
        motorDevice = &motorNullDevice;
    }
}

void motorDisable(void)
{
    motorDevice->vTable.disable();
    motorDevice->enabled = false;
    motorDevice->motorEnableTimeMs = 0;
}

void motorEnable(void)
{
    if (motorDevice->initialized && motorDevice->vTable.enable()) {
        motorDevice->enabled = true;
        motorDevice->motorEnableTimeMs = millis();
    }
}

bool motorIsEnabled(void)
{
    return motorDevice->enabled;
}

bool motorIsMotorEnabled(uint8_t index)
{
    return motorDevice->vTable.isMotorEnabled(index);
}

bool isMotorProtocolDshot(void)
{
    return isDshot;
}

#ifdef USE_DSHOT
timeMs_t motorGetMotorEnableTimeMs(void)
{
    return motorDevice->motorEnableTimeMs;
}
#endif

#ifdef USE_DSHOT_BITBANG
bool isDshotBitbangActive(const motorDevConfig_t *motorConfig) {
    return motorConfig->useDshotBitbang == DSHOT_BITBANG_ON ||
        (motorConfig->useDshotBitbang == DSHOT_BITBANG_AUTO && motorConfig->useDshotTelemetry && motorConfig->motorPwmProtocol != PWM_TYPE_PROSHOT1000);
}
#endif

#endif // USE_MOTOR
