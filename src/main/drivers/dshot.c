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
 *
 * Follows the extended dshot telemetry documentation found at https://github.com/bird-sanctuary/extended-dshot-telemetry
 */

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/debug.h"
#include "build/atomic.h"
#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/motor.h"
#include "drivers/timer.h"

#include "drivers/dshot_command.h"
#include "drivers/nvic.h"

#include "flight/mixer.h"

#include "pg/rpm_filter.h"

#include "rx/rx.h"

#include "dshot.h"

#define ERPM_PER_LSB                        100.0f

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

FAST_DATA_ZERO_INIT static pt1Filter_t motorFreqLpf[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT static float motorFrequencyHz[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT static float minMotorFrequencyHz;
FAST_DATA_ZERO_INIT static float erpmToHz;
FAST_DATA_ZERO_INIT static float dshotRpmAverage;
FAST_DATA_ZERO_INIT static float dshotRpm[MAX_SUPPORTED_MOTORS];

void initDshotTelemetry(const timeUs_t looptimeUs)
{
    // if bidirectional DShot is not available
    if (!motorConfig()->dev.useDshotTelemetry && !featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return;
    }

    // erpmToHz is used by bidir dshot and ESC telemetry
    erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE / (motorConfig()->motorPoleCount / 2.0f);

    if (motorConfig()->dev.useDshotTelemetry) {
        // init LPFs for RPM data
        for (int i = 0; i < getMotorCount(); i++) {
            pt1FilterInit(&motorFreqLpf[i], pt1FilterGain(rpmFilterConfig()->rpm_filter_lpf_hz, looptimeUs * 1e-6f));
        }
    }
}

static uint32_t dshotDecodeErpmTelemetryValue(uint16_t value)
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

static void dshotStoreDebugWithStatus(unsigned motorIndex, uint32_t decodedValue, debugType_e debugType) {
    if (motorIndex < DEBUG16_VALUE_COUNT) {
        uint16_t pHighByte = dshotTelemetryState.motorState[motorIndex].telemetryData[DSHOT_TELEMETRY_TYPE_STATUS] << (sizeof(uint8_t) * 8);
        DEBUG_SET(debugType, motorIndex, pHighByte | decodedValue);
    }
}

static bool dshotDecodeTelemetryValue(uint8_t motorIndex, uint32_t *pDecoded, dshotTelemetryType_e *pType)
{
    bool useDebug = false;
    debugType_e debugType;
    const uint16_t value = dshotTelemetryState.motorState[motorIndex].rawValue;

    *pDecoded = DSHOT_TELEMETRY_INVALID;
    if (dshotTelemetryState.motorState[motorIndex].telemetryTypes == DSHOT_NORMAL_TELEMETRY_MASK) {   /* Check DSHOT_TELEMETRY_TYPE_eRPM mask */
        // Decode eRPM telemetry
        *pDecoded = dshotDecodeErpmTelemetryValue(value);

        // Update debug buffer (motorIndex < motorCount guaranteed by caller)
        if (motorIndex < DEBUG16_VALUE_COUNT) {
            DEBUG_SET(DEBUG_DSHOT_RPM_TELEMETRY, motorIndex, *pDecoded);
        }

        *pType = DSHOT_TELEMETRY_TYPE_eRPM;
    } else {
        // Decode Extended DSHOT telemetry
        *pDecoded = value & DSHOT_TELEMETRY_VALUE_MASK;

        switch (value & DSHOT_TELEMETRY_RANGE_MASK) {

        case DSHOT_TELEMETRY_RANGE_TEMPERATURE:
            // Temperature frame (in degree Celsius, just like Blheli_32 and KISS)
            debugType = DEBUG_DSHOT_STATUS_N_TEMPERATURE;
            *pType = DSHOT_TELEMETRY_TYPE_TEMPERATURE;
            useDebug = true;
            break;

        case DSHOT_TELEMETRY_RANGE_VOLTAGE:
            // Voltage frame (0-63,75V step 0,25V)
            debugType = DEBUG_DSHOT_STATUS_N_VOLTAGE;
            *pType = DSHOT_TELEMETRY_TYPE_VOLTAGE;
            useDebug = true;
            break;

        case DSHOT_TELEMETRY_RANGE_CURRENT:
            // Current frame (0-255A step 1A)
            debugType = DEBUG_DSHOT_STATUS_N_CURRENT;
            *pType = DSHOT_TELEMETRY_TYPE_CURRENT;
            useDebug = true;
            break;

        case DSHOT_TELEMETRY_RANGE_DEBUG1:
            // Debug 1 frame
            debugType = DEBUG_DSHOT_STATUS_N_DEBUG1;
            *pType = DSHOT_TELEMETRY_TYPE_DEBUG1;
            useDebug = true;
            break;

        case DSHOT_TELEMETRY_RANGE_DEBUG2:
            // Debug 2 frame
            debugType = DEBUG_DSHOT_STATUS_N_DEBUG2;
            *pType = DSHOT_TELEMETRY_TYPE_DEBUG2;
            useDebug = true;
            break;

        case DSHOT_TELEMETRY_RANGE_STRESS_LEVEL:
            // Stress level frame
            debugType = DEBUG_DSHOT_STATUS_N_STRESS_LVL;
            *pType = DSHOT_TELEMETRY_TYPE_STRESS_LEVEL;
            useDebug = true;
            break;

        case DSHOT_TELEMETRY_RANGE_STATUS:
            // State / events frame
            *pType = DSHOT_TELEMETRY_TYPE_STATUS;
            break;

        default:
            // Decode as eRPM
            *pDecoded = dshotDecodeErpmTelemetryValue(value);

            // Update debug buffer (motorIndex < motorCount guaranteed by caller)
            if (motorIndex < DEBUG16_VALUE_COUNT) {
                // In this case two debug options to maximize logging info
                uint16_t dshotDebugHighByte = dshotTelemetryState.motorState[motorIndex].telemetryData[DSHOT_TELEMETRY_TYPE_STATUS] << (sizeof(uint8_t) * 8);
                DEBUG_SET(DEBUG_DSHOT_STATUS_N_ERPM_FRACTION_18, motorIndex, dshotDebugHighByte | ((*pDecoded) / 18));
                DEBUG_SET(DEBUG_DSHOT_RPM_TELEMETRY, motorIndex, *pDecoded);
            }

            *pType = DSHOT_TELEMETRY_TYPE_eRPM;
            break;
        }

        if(useDebug) {
          dshotStoreDebugWithStatus(motorIndex, *pDecoded, debugType);
        }
    }

    return *pDecoded != DSHOT_TELEMETRY_INVALID;
}

static void dshotUpdateTelemetryData(uint8_t motorIndex, dshotTelemetryType_e type, uint32_t value)
{
    // Update telemetry data
    dshotTelemetryState.motorState[motorIndex].telemetryData[type] = value;
    dshotTelemetryState.motorState[motorIndex].telemetryTypes |= (1 << type);

    // Update max temp
    if ((type == DSHOT_TELEMETRY_TYPE_TEMPERATURE) && (value > dshotTelemetryState.motorState[motorIndex].maxTemp)) {
        dshotTelemetryState.motorState[motorIndex].maxTemp = value;
    }
}

FAST_CODE_NOINLINE void updateDshotTelemetry(void)
{
    if (!useDshotTelemetry) {
        return;
    }

    // Only process telemetry in case it hasn´t been processed yet
    if (dshotTelemetryState.rawValueState != DSHOT_RAW_VALUE_STATE_NOT_PROCESSED) {
        return;
    }

    const unsigned motorCount = motorDeviceCount();
    uint32_t erpmTotal = 0;
    uint32_t rpmSamples = 0;

    // Decode all telemetry data now to discharge interrupt from this task
    for (uint8_t k = 0; k < motorCount; k++) {
        dshotTelemetryType_e type;
        uint32_t value;

        if (dshotDecodeTelemetryValue(k, &value, &type)) {
            dshotUpdateTelemetryData(k, type, value);

            if (type == DSHOT_TELEMETRY_TYPE_eRPM) {
                dshotRpm[k] = erpmToRpm(value);
                erpmTotal += value;
                rpmSamples++;
            }
        }
    }

    // Update average
    if (rpmSamples > 0) {
        dshotRpmAverage = erpmToRpm(erpmTotal) / (float)rpmSamples;
    }

    // update filtered rotation speed of motors for features (e.g. "RPM filter")
    minMotorFrequencyHz = FLT_MAX;
    for (int motor = 0; motor < getMotorCount(); motor++) {
        motorFrequencyHz[motor] = pt1FilterApply(&motorFreqLpf[motor], erpmToHz * getDshotErpm(motor));
        minMotorFrequencyHz = MIN(minMotorFrequencyHz, motorFrequencyHz[motor]);
    }

    // Set state to processed
    dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_PROCESSED;
}

uint16_t getDshotErpm(uint8_t motorIndex)
{
    return dshotTelemetryState.motorState[motorIndex].telemetryData[DSHOT_TELEMETRY_TYPE_eRPM];
}

float getDshotRpm(uint8_t motorIndex)
{
    return dshotRpm[motorIndex];
}

float getDshotRpmAverage(void)
{
    return dshotRpmAverage;
}

float getMotorFrequencyHz(uint8_t motorIndex)
{
    return motorFrequencyHz[motorIndex];
}

float getMinMotorFrequencyHz(void)
{
    return minMotorFrequencyHz;
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

#endif // USE_DSHOT_TELEMETRY

#if defined(USE_ESC_SENSOR) || defined(USE_DSHOT_TELEMETRY)

// Used with serial esc telem as well as dshot telem
float erpmToRpm(uint32_t erpm)
{
    // rpm = (erpm * ERPM_PER_LSB) / (motorConfig()->motorPoleCount / 2)
    return erpm * erpmToHz * SECONDS_PER_MINUTE;
}

#endif // USE_ESC_SENSOR || USE_DSHOT_TELEMETRY

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
