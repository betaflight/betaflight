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
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/atomic.h"

#include "common/maths.h"

#include "config/feature.h"

#include "drivers/motor.h"
#include "drivers/timer.h"

#include "drivers/dshot_command.h"
#include "drivers/nvic.h"

#include "flight/mixer.h"

#include "rx/rx.h"
#include "dshot.h"

void dshotInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow)
{
    float outputLimitOffset = DSHOT_RANGE * (1 - outputLimit);
    *disarm = DSHOT_CMD_MOTOR_STOP;
    if (featureIsEnabled(FEATURE_3D)) {
        *outputLow = DSHOT_MIN_THROTTLE + getDigitalIdleOffset(motorConfig) * (DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - DSHOT_MIN_THROTTLE);
        *outputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset / 2;
        *deadbandMotor3dHigh = DSHOT_3D_FORWARD_MIN_THROTTLE + getDigitalIdleOffset(motorConfig) * (DSHOT_MAX_THROTTLE - DSHOT_3D_FORWARD_MIN_THROTTLE);
        *deadbandMotor3dLow = DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - outputLimitOffset / 2;
    } else {
        *outputLow = DSHOT_MIN_THROTTLE + getDigitalIdleOffset(motorConfig) * DSHOT_RANGE;
        *outputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset;
    }
}

float dshotConvertFromExternal(uint16_t externalValue)
{
    float motorValue;

    externalValue = constrain(externalValue, PWM_RANGE_MIN, PWM_RANGE_MAX);

    if (featureIsEnabled(FEATURE_3D)) {
        if (externalValue == PWM_RANGE_MIDDLE) {
            motorValue = DSHOT_CMD_MOTOR_STOP;
        } else if (externalValue < PWM_RANGE_MIDDLE) {
            motorValue = scaleRangef(externalValue, PWM_RANGE_MIN, PWM_RANGE_MIDDLE - 1, DSHOT_3D_FORWARD_MIN_THROTTLE - 1, DSHOT_MIN_THROTTLE);
        } else {
            motorValue = scaleRangef(externalValue, PWM_RANGE_MIDDLE + 1, PWM_RANGE_MAX, DSHOT_3D_FORWARD_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
        }
    } else {
        motorValue = (externalValue == PWM_RANGE_MIN) ? DSHOT_CMD_MOTOR_STOP : scaleRangef(externalValue, PWM_RANGE_MIN + 1, PWM_RANGE_MAX, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    }

    return motorValue;
}

uint16_t dshotConvertToExternal(float motorValue)
{
    float externalValue;

    if (featureIsEnabled(FEATURE_3D)) {
        if (motorValue == DSHOT_CMD_MOTOR_STOP || motorValue < DSHOT_MIN_THROTTLE) {
            externalValue = PWM_RANGE_MIDDLE;
        } else if (motorValue <= DSHOT_3D_FORWARD_MIN_THROTTLE - 1) {
            externalValue = scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_3D_FORWARD_MIN_THROTTLE - 1, PWM_RANGE_MIDDLE - 1, PWM_RANGE_MIN);
        } else {
            externalValue = scaleRangef(motorValue, DSHOT_3D_FORWARD_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIDDLE + 1, PWM_RANGE_MAX);
        }
    } else {
        externalValue = (motorValue < DSHOT_MIN_THROTTLE) ? PWM_RANGE_MIN : scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIN + 1, PWM_RANGE_MAX);
    }

    return lrintf(externalValue);
}

FAST_CODE uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb)
{
    uint16_t packet;

    ATOMIC_BLOCK(NVIC_PRIO_DSHOT_DMA) {
        packet = (pcb->value << 1) | (pcb->requestTelemetry ? 1 : 0);
        pcb->requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row
    }

    // compute checksum
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    // append checksum
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        csum = ~csum;
    }
#endif
    csum &= 0xf;
    packet = (packet << 4) | csum;

    return packet;
}

#ifdef USE_DSHOT_TELEMETRY


FAST_DATA_ZERO_INIT dshotTelemetryState_t dshotTelemetryState;

static uint32_t dshot_decode_eRPM_telemetry_value(uint16_t value)
{
    // eRPM range
    if (value == 0x0fff) {
        return 0;
    }

    // Convert value to 16 bit from the GCR telemetry format (eeem mmmm mmmm)
    value = (value & 0x01ff) << ((value & 0xfe00) >> 9);
    if (!value) {
        return DSHOT_TELEMETRY_INVALID;
    }

    // Convert period to erpm * 100
    return (1000000 * 60 / 100 + value / 2) / value;
}

static void dshot_decode_telemetry_value(uint8_t motorIndex, uint32_t *pDecoded, dshotTelemetryType_t *pType)
{
    uint16_t value = dshotTelemetryState.motorState[motorIndex].rawValue;

    if (dshotTelemetryState.motorState[motorIndex].telemetryTypes == DSHOT_NORMAL_TELEMETRY_MASK) {   /* Check DSHOT_TELEMETRY_TYPE_eRPM mask */
        // Decode eRPM telemetry
        *pDecoded = dshot_decode_eRPM_telemetry_value(value);

        // Set telemetry type
        *pType = DSHOT_TELEMETRY_TYPE_eRPM;
    } else {
        // Decode Extended DSHOT telemetry
        switch (value & 0x0f00) {

        case 0x0200:
            // Temperature range (in degree Celsius, just like Blheli_32 and KISS)
            *pDecoded = value & 0x00ff;

            // Set telemetry type
            *pType = DSHOT_TELEMETRY_TYPE_TEMPERATURE;
            break;

        case 0x0400:
            // Voltage range (0-63,75V step 0,25V)
            *pDecoded = value & 0x00ff;

            // Set telemetry type
            *pType = DSHOT_TELEMETRY_TYPE_VOLTAGE;
            break;

        case 0x0600:
            // Current range (0-255A step 1A)
            *pDecoded = value & 0x00ff;

            // Set telemetry type
            *pType = DSHOT_TELEMETRY_TYPE_CURRENT;
            break;

        case 0x0800:
            // Debug 1 value
            *pDecoded = value & 0x00ff;

            // Set telemetry type
            *pType = DSHOT_TELEMETRY_TYPE_DEBUG1;
            break;

        case 0x0A00:
            // Debug 2 value
            *pDecoded = value & 0x00ff;

            // Set telemetry type
            *pType = DSHOT_TELEMETRY_TYPE_DEBUG2;
            break;

        case 0x0C00:
            // Debug 3 value
            *pDecoded = value & 0x00ff;

            // Set telemetry type
            *pType = DSHOT_TELEMETRY_TYPE_DEBUG3;
            break;

        case 0x0E00:
            // State / events
            *pDecoded = value & 0x00ff;

            // Set telemetry type
            *pType = DSHOT_TELEMETRY_TYPE_STATE_EVENTS;
            break;

        default:
            // Decode as eRPM
            *pDecoded = dshot_decode_eRPM_telemetry_value(value);

            // Set telemetry type
            *pType = DSHOT_TELEMETRY_TYPE_eRPM;
            break;

        }
    }
}

static void dshotUpdateTelemetryData(uint8_t motorIndex, dshotTelemetryType_t type, uint32_t value)
{
    // Update telemetry data
    dshotTelemetryState.motorState[motorIndex].telemetryData[type] = value;
    dshotTelemetryState.motorState[motorIndex].telemetryTypes |= (1 << type);

    // Update max temp
    if ((type == DSHOT_TELEMETRY_TYPE_TEMPERATURE) && (value > dshotTelemetryState.motorState[motorIndex].maxTemp)) {
        dshotTelemetryState.motorState[motorIndex].maxTemp = value;
    }
}

uint16_t getDshotTelemetry(uint8_t index)
{
    // Process telemetry in case it haven´t been processed yet
    if (dshotTelemetryState.rawValueState == DSHOT_RAW_VALUE_STATE_NOT_PROCESSED) {
        const unsigned motorCount = motorDeviceCount();
        uint32_t rpmTotal = 0;
        uint32_t rpmSamples = 0;

        // Decode all telemetry data now to discharge interrupt from this task
        for (uint8_t k = 0; k < motorCount; k++) {
            dshotTelemetryType_t type;
            uint32_t value;

            dshot_decode_telemetry_value(k, &value, &type);

            if (value != DSHOT_TELEMETRY_INVALID) {
                dshotUpdateTelemetryData(k, type, value);

                if (type == DSHOT_TELEMETRY_TYPE_eRPM) {
                    rpmTotal += value;
                    rpmSamples++;
                }
            }
        }

        // Update average
        if (rpmSamples > 0) {
            dshotTelemetryState.averageRpm = rpmTotal / rpmSamples;
        }

        // Set state to processed
        dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_PROCESSED;
    }

    return dshotTelemetryState.motorState[index].telemetryData[DSHOT_TELEMETRY_TYPE_eRPM];
}

bool isDshotMotorTelemetryActive(uint8_t motorIndex)
{
    return (dshotTelemetryState.motorState[motorIndex].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_eRPM)) != 0;
}

bool isDshotTelemetryActive(void)
{
    const unsigned motorCount = motorDeviceCount();
    if (motorCount) {
        for (unsigned i = 0; i < motorCount; i++) {
            if (!isDshotMotorTelemetryActive(i)) {
                return false;
            }
        }
        return true;
    }
    return false;
}

void dshotCleanTelemetryData(void)
{
    memset(&dshotTelemetryState, 0, sizeof(dshotTelemetryState));
}

uint32_t erpmToRpm(uint16_t erpm)
{
    //  rpm = (erpm * 100) / (motorConfig()->motorPoleCount / 2)
    return (erpm * 200) / motorConfig()->motorPoleCount;
}

uint32_t getDshotAverageRpm(void)
{
    return dshotTelemetryState.averageRpm;
}

#endif // USE_DSHOT_TELEMETRY

#ifdef USE_DSHOT_TELEMETRY_STATS

FAST_DATA_ZERO_INIT dshotTelemetryQuality_t dshotTelemetryQuality[MAX_SUPPORTED_MOTORS];

int16_t getDshotTelemetryMotorInvalidPercent(uint8_t motorIndex)
{
    int16_t invalidPercent = 0;

    if (isDshotMotorTelemetryActive(motorIndex)) {
        const uint32_t totalCount = dshotTelemetryQuality[motorIndex].packetCountSum;
        const uint32_t invalidCount = dshotTelemetryQuality[motorIndex].invalidCountSum;
        if (totalCount > 0) {
            invalidPercent = lrintf(invalidCount * 10000.0f / totalCount);
        }
    } else {
        invalidPercent = 10000;  // 100.00%
    }
    return invalidPercent;
}

void updateDshotTelemetryQuality(dshotTelemetryQuality_t *qualityStats, bool packetValid, timeMs_t currentTimeMs)
{
    uint8_t statsBucketIndex = (currentTimeMs / DSHOT_TELEMETRY_QUALITY_BUCKET_MS) % DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT;
    if (statsBucketIndex != qualityStats->lastBucketIndex) {
        qualityStats->packetCountSum -= qualityStats->packetCountArray[statsBucketIndex];
        qualityStats->invalidCountSum -= qualityStats->invalidCountArray[statsBucketIndex];
        qualityStats->packetCountArray[statsBucketIndex] = 0;
        qualityStats->invalidCountArray[statsBucketIndex] = 0;
        qualityStats->lastBucketIndex = statsBucketIndex;
    }
    qualityStats->packetCountSum++;
    qualityStats->packetCountArray[statsBucketIndex]++;
    if (!packetValid) {
        qualityStats->invalidCountSum++;
        qualityStats->invalidCountArray[statsBucketIndex]++;
    }
}
#endif // USE_DSHOT_TELEMETRY_STATS

#endif // USE_DSHOT

// temporarily here, needs to be moved during refactoring
void validateAndfixMotorOutputReordering(uint8_t *array, const unsigned size)
{
    bool invalid = false;

    for (unsigned i = 0; i < size; i++) {
        if (array[i] >= size) {
            invalid = true;
            break;
        }
    }

    int valuesAsIndexes[size];

    for (unsigned i = 0; i < size; i++) {
        valuesAsIndexes[i] = -1;
    }

    if (!invalid) {
        for (unsigned i = 0; i < size; i++) {
            if (-1 != valuesAsIndexes[array[i]]) {
                invalid = true;
                break;
            }

            valuesAsIndexes[array[i]] = array[i];
        }
    }

    if (invalid) {
        for (unsigned i = 0; i < size; i++) {
            array[i] = i;
        }
    }
}
