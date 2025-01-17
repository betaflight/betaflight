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

#include "drivers/dshot.h"
#include "drivers/dshot_bitbang.h" // TODO: bitbang should be behind the veil of dshot (it is an implementation)
#include "drivers/pwm_output.h"

#include "drivers/time.h"

#include "fc/rc_controls.h" // for flight3DConfig_t

#include "sensors/battery.h"

#include "motor.h"

static FAST_DATA_ZERO_INIT motorDevice_t motorDevice;

static bool motorProtocolEnabled = false;
static bool motorProtocolDshot = false;

void motorShutdown(void)
{
    uint32_t shutdownDelayUs = 1500;
    motorDevice.vTable->shutdown();
    motorDevice.enabled = false;
    motorDevice.motorEnableTimeMs = 0;
    motorDevice.initialized = false;

    switch (motorConfig()->dev.motorProtocol) {
    case MOTOR_PROTOCOL_PWM :
    case MOTOR_PROTOCOL_ONESHOT125:
    case MOTOR_PROTOCOL_ONESHOT42:
    case MOTOR_PROTOCOL_MULTISHOT:
        // Delay 500ms will disarm esc which can prevent motor spin while reboot
        shutdownDelayUs += 500 * 1000;
        break;
    default:
        break;
    }

    delayMicroseconds(shutdownDelayUs);
}

void motorWriteAll(float *values)
{
#ifdef USE_PWM_OUTPUT
    if (motorDevice.enabled) {
#ifdef USE_DSHOT_BITBANG
        if (isDshotBitbangActive(&motorConfig()->dev)) {
            // Initialise the output buffers
            if (motorDevice.vTable->updateInit) {
                motorDevice.vTable->updateInit();
            }

            // Update the motor data
            for (int i = 0; i < motorDevice.count; i++) {
                motorDevice.vTable->write(i, values[i]);
            }

            // Don't attempt to write commands to the motors if telemetry is still being received
            if (motorDevice.vTable->telemetryWait) {
                (void)motorDevice.vTable->telemetryWait();
            }

            // Trigger the transmission of the motor data
            motorDevice.vTable->updateComplete();

            // Perform the decode of the last data received
            // New data will be received once the send of motor data, triggered above, completes
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
            if (motorDevice.vTable->decodeTelemetry) {
                motorDevice.vTable->decodeTelemetry();
            }
#endif
        } else
#endif
        {
            // Perform the decode of the last data received
            // New data will be received once the send of motor data, triggered above, completes
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
            motorDevice.vTable->decodeTelemetry();
#endif

            // Update the motor data
            for (int i = 0; i < motorDevice.count; i++) {
                motorDevice.vTable->write(i, values[i]);
            }

            // Trigger the transmission of the motor data
            motorDevice.vTable->updateComplete();
        }
    }
#else
    UNUSED(values);
#endif
}

void motorRequestTelemetry(unsigned index)
{
    if (index >= motorDevice.count) {
        return;
    }

    if (motorDevice.vTable->requestTelemetry) {
        motorDevice.vTable->requestTelemetry(index);
    }
}

unsigned motorDeviceCount(void)
{
    return motorDevice.count;
}

const motorVTable_t *motorGetVTable(void)
{
    return motorDevice.vTable;
}

bool checkMotorProtocolEnabled(const motorDevConfig_t *motorDevConfig, bool *isProtocolDshot)
{
    bool enabled = false;
    bool isDshot = false;

    switch (motorDevConfig->motorProtocol) {
    case MOTOR_PROTOCOL_PWM :
    case MOTOR_PROTOCOL_ONESHOT125:
    case MOTOR_PROTOCOL_ONESHOT42:
    case MOTOR_PROTOCOL_MULTISHOT:
    case MOTOR_PROTOCOL_BRUSHED:
        enabled = true;
        break;

#ifdef USE_DSHOT
    case MOTOR_PROTOCOL_DSHOT150:
    case MOTOR_PROTOCOL_DSHOT300:
    case MOTOR_PROTOCOL_DSHOT600:
    case MOTOR_PROTOCOL_PROSHOT1000:
        enabled = true;
        isDshot = true;
        break;
#endif
    default:
        break;
    }

    if (isProtocolDshot) {
        *isProtocolDshot = isDshot;
    }

    return enabled;
}

motorProtocolFamily_e motorGetProtocolFamily(void)
{
    switch (motorConfig()->dev.motorProtocol) {
#ifdef USE_PWMOUTPUT
    case MOTOR_PROTOCOL_PWM :
    case MOTOR_PROTOCOL_ONESHOT125:
    case MOTOR_PROTOCOL_ONESHOT42:
    case MOTOR_PROTOCOL_MULTISHOT:
    case MOTOR_PROTOCOL_BRUSHED:
        return MOTOR_PROTOCOL_FAMILY_PWM;
#endif
#ifdef USE_DSHOT
    case MOTOR_PROTOCOL_DSHOT150:
    case MOTOR_PROTOCOL_DSHOT300:
    case MOTOR_PROTOCOL_DSHOT600:
    case MOTOR_PROTOCOL_PROSHOT1000:
        return MOTOR_PROTOCOL_FAMILY_DSHOT;
#endif
    default:
        return MOTOR_PROTOCOL_FAMILY_UNKNOWN;
    }
}

static void checkMotorProtocol(const motorDevConfig_t *motorDevConfig)
{
    motorProtocolEnabled = checkMotorProtocolEnabled(motorDevConfig, &motorProtocolDshot);
}

// End point initialization is called from mixerInit before motorDevInit; can't use vtable...
void motorInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow)
{
    checkMotorProtocol(&motorConfig->dev);

    if (isMotorProtocolEnabled()) {
        switch (motorGetProtocolFamily()) {
#ifdef USE_PWM_OUTPUT
        case MOTOR_PROTOCOL_FAMILY_PWM:
            analogInitEndpoints(motorConfig, outputLimit, outputLow, outputHigh, disarm, deadbandMotor3dHigh, deadbandMotor3dLow);
            break;
#endif
#ifdef USE_DSHOT
        case MOTOR_PROTOCOL_FAMILY_DSHOT:
            dshotInitEndpoints(motorConfig, outputLimit, outputLow, outputHigh, disarm, deadbandMotor3dHigh, deadbandMotor3dLow);
            break;
#endif
        default:
            // TODO: perhaps a failure mode here?
            break;
        }
    }
}

float motorConvertFromExternal(uint16_t externalValue)
{
    return motorDevice.vTable->convertExternalToMotor(externalValue);
}

uint16_t motorConvertToExternal(float motorValue)
{
    return motorDevice.vTable->convertMotorToExternal(motorValue);
}

void motorPostInit(void)
{
    if (motorDevice.vTable->postInit) {
        motorDevice.vTable->postInit();
    }
}

bool isMotorProtocolEnabled(void)
{
    return motorProtocolEnabled;
}

bool isMotorProtocolDshot(void)
{
    return motorProtocolDshot;
}

bool isMotorProtocolBidirDshot(void)
{
    return isMotorProtocolDshot() && useDshotTelemetry;
}

void motorNullDevInit(motorDevice_t *device);

void motorDevInit(unsigned motorCount)
{
#if defined(USE_PWM_OUTPUT) || defined(USE_DSHOT)
    const motorDevConfig_t *motorDevConfig = &motorConfig()->dev;
#endif

#if defined(USE_PWM_OUTPUT)
    uint16_t idlePulse = motorConfig()->mincommand;
    if (featureIsEnabled(FEATURE_3D)) {
        idlePulse = flight3DConfig()->neutral3d;
    }
    if (motorConfig()->dev.motorProtocol == MOTOR_PROTOCOL_BRUSHED) {
        idlePulse = 0; // brushed motors
    }
#endif

    motorDevice.count = motorCount;
    if (isMotorProtocolEnabled()) {
        do {
            if (!isMotorProtocolDshot()) {
#ifdef USE_PWM_OUTPUT
                motorPwmDevInit(&motorDevice, motorDevConfig, idlePulse);
#endif
                break;
            }
#ifdef USE_DSHOT
#ifdef USE_DSHOT_BITBANG
            if (isDshotBitbangActive(motorDevConfig)) {
                dshotBitbangDevInit(&motorDevice, motorDevConfig);
                break;
            }
#endif
            dshotPwmDevInit(&motorDevice, motorDevConfig);
#endif
        } while(0);
    }

    // if the VTable has been populated, the device is initialized.
    if (motorDevice.vTable) {
        motorDevice.initialized = true;
        motorDevice.motorEnableTimeMs = 0;
        motorDevice.enabled = false;
    } else {
        motorNullDevInit(&motorDevice);
    }
}

void motorDisable(void)
{
    motorDevice.vTable->disable();
    motorDevice.enabled = false;
    motorDevice.motorEnableTimeMs = 0;
}

void motorEnable(void)
{
    if (motorDevice.initialized && motorDevice.vTable->enable()) {
        motorDevice.enabled = true;
        motorDevice.motorEnableTimeMs = millis();
    }
}

float motorEstimateMaxRpm(void)
{
    // Empirical testing found this relationship between estimated max RPM without props attached
    // (unloaded) and measured max RPM with props attached (loaded), independent from prop size
    float unloadedMaxRpm = 0.01f * getBatteryVoltage() * motorConfig()->kv;
    float loadDerating = -5.44e-6f * unloadedMaxRpm + 0.944f;

    return unloadedMaxRpm * loadDerating;
}

bool motorIsEnabled(void)
{
    return motorDevice.enabled;
}

bool motorIsMotorEnabled(unsigned index)
{
    return motorDevice.vTable->isMotorEnabled(index);
}

bool motorIsMotorIdle(unsigned index)
{
    return motorDevice.vTable->isMotorIdle ? motorDevice.vTable->isMotorIdle(index) : false;
}

#ifdef USE_DSHOT
timeMs_t motorGetMotorEnableTimeMs(void)
{
    return motorDevice.motorEnableTimeMs;
}
#endif

/* functions below for empty methods and no active motors */
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

static bool motorIsEnabledNull(unsigned index)
{
    UNUSED(index);
    return false;
}

bool motorDecodeTelemetryNull(void)
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
    .decodeTelemetry = motorDecodeTelemetryNull,
    .write = motorWriteNull,
    .writeInt = motorWriteIntNull,
    .updateComplete = motorUpdateCompleteNull,
    .convertExternalToMotor = motorConvertFromExternalNull,
    .convertMotorToExternal = motorConvertToExternalNull,
    .shutdown = motorShutdownNull,
    .requestTelemetry = NULL,
    .isMotorIdle = NULL,
};

void motorNullDevInit(motorDevice_t *device)
{
    device->vTable = &motorNullVTable;
    device->count = 0;
}


#endif // USE_MOTOR
