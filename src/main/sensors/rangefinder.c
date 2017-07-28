/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/io.h"
#include "drivers/logging.h"
#include "drivers/time.h"
#include "drivers/rangefinder_hcsr04.h"
#include "drivers/rangefinder_srf10.h"
#include "drivers/rangefinder_hcsr04_i2c.h"
#include "drivers/rangefinder_vl53l0x.h"
#include "drivers/rangefinder.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/rangefinder.h"
#include "sensors/battery.h"

#include "scheduler/scheduler.h"

#include "build/debug.h"

rangefinder_t rangefinder;

#define RANGEFINDER_HARDWARE_TIMEOUT_MS         500     // Accept 500ms of non-responsive sensor, report HW failure otherwise

#define RANGEFINDER_DYNAMIC_THRESHOLD           600     //Used to determine max. usable rangefinder disatance
#define RANGEFINDER_DYNAMIC_FACTOR              75    

#ifdef USE_RANGEFINDER
PG_REGISTER_WITH_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig, PG_RANGEFINDER_CONFIG, 0);

PG_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig,
    .rangefinder_hardware = RANGEFINDER_NONE,
);

const rangefinderHardwarePins_t * rangefinderGetHardwarePins(void)
{
    static rangefinderHardwarePins_t rangefinderHardwarePins;

#if defined(RANGEFINDER_HCSR04_PWM_TRIGGER_PIN)
    // If we are using softserial, parallel PWM or ADC current sensor, then use motor pins for sonar, otherwise use RC pins
    if (feature(FEATURE_SOFTSERIAL)
            || feature(FEATURE_RX_PARALLEL_PWM )
            || (feature(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC)) {
        rangefinderHardwarePins.triggerTag = IO_TAG(RANGEFINDER_HCSR04_TRIGGER_PIN_PWM);
        rangefinderHardwarePins.echoTag = IO_TAG(RANGEFINDER_HCSR04_ECHO_PIN_PWM);
    } else {
        rangefinderHardwarePins.triggerTag = IO_TAG(RANGEFINDER_HCSR04_TRIGGER_PIN);
        rangefinderHardwarePins.echoTag = IO_TAG(RANGEFINDER_HCSR04_ECHO_PIN);
    }
#elif defined(RANGEFINDER_HCSR04_TRIGGER_PIN)
    rangefinderHardwarePins.triggerTag = IO_TAG(RANGEFINDER_HCSR04_TRIGGER_PIN);
    rangefinderHardwarePins.echoTag = IO_TAG(RANGEFINDER_HCSR04_ECHO_PIN);
#endif
    return &rangefinderHardwarePins;
}

/*
 * Detect which rangefinder is present
 */
static bool rangefinderDetect(rangefinderDev_t * dev, uint8_t rangefinderHardwareToUse)
{
    rangefinderType_e rangefinderHardware = RANGEFINDER_NONE;
    requestedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderHardwareToUse;

    switch (rangefinderHardwareToUse) {
        case RANGEFINDER_HCSR04:
#ifdef USE_RANGEFINDER_HCSR04
            {
                const rangefinderHardwarePins_t *rangefinderHardwarePins = rangefinderGetHardwarePins();
                if (hcsr04Detect(dev, rangefinderHardwarePins)) {   // FIXME: Do actual detection if HC-SR04 is plugged in
                    rangefinderHardware = RANGEFINDER_HCSR04;
                    rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_HCSR04_TASK_PERIOD_MS));
                }
            }
#endif
            break;

        case RANGEFINDER_SRF10:
#ifdef USE_RANGEFINDER_SRF10
            if (srf10Detect(dev)) {
                rangefinderHardware = RANGEFINDER_SRF10;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_SRF10_TASK_PERIOD_MS));
            }
#endif
            break;

            case RANGEFINDER_HCSR04I2C:
#ifdef USE_RANGEFINDER_HCSR04_I2C
            if (hcsr04i2c0Detect(dev)) {
                rangefinderHardware = RANGEFINDER_HCSR04I2C;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_HCSR04_i2C_TASK_PERIOD_MS));
            }
#endif
            break;

            case RANGEFINDER_VL53L0X:
#if defined(USE_RANGEFINDER_VL53L0X)
            if (vl53l0xDetect(dev)) {
                rangefinderHardware = RANGEFINDER_VL53L0X;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_VL53L0X_TASK_PERIOD_MS));
            }
#endif
            break;

        case RANGEFINDER_NONE:
            rangefinderHardware = RANGEFINDER_NONE;
            break;
    }

    addBootlogEvent6(BOOT_EVENT_RANGEFINDER_DETECTION, BOOT_EVENT_FLAGS_NONE, rangefinderHardware, 0, 0, 0);

    if (rangefinderHardware == RANGEFINDER_NONE) {
        sensorsClear(SENSOR_RANGEFINDER);
        return false;
    }

    detectedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderHardware;
    sensorsSet(SENSOR_RANGEFINDER);
    return true;
}

void rangefinderResetDynamicThreshold(void) {
    rangefinder.snrThresholdReached = false;
    rangefinder.dynamicDistanceThreshold = 0;
}

bool rangefinderInit(void)
{
    if (!rangefinderDetect(&rangefinder.dev, rangefinderConfig()->rangefinder_hardware)) {
        return false;
    }

    rangefinder.dev.init();
    rangefinder.rawAltitude = RANGEFINDER_OUT_OF_RANGE;
    rangefinder.calculatedAltitude = RANGEFINDER_OUT_OF_RANGE;
    rangefinder.maxTiltCos = cos_approx(DECIDEGREES_TO_RADIANS(rangefinder.dev.detectionConeExtendedDeciDegrees / 2.0f));
    rangefinder.lastValidResponseTimeMs = millis();
    rangefinder.snr = 0;

    rangefinderResetDynamicThreshold();

    return true;
}

static int32_t applyMedianFilter(int32_t newReading)
{
    #define DISTANCE_SAMPLES_MEDIAN 5
    static int32_t filterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int filterSampleIndex = 0;
    static bool medianFilterReady = false;

    if (newReading > RANGEFINDER_OUT_OF_RANGE) {// only accept samples that are in range
        filterSamples[filterSampleIndex] = newReading;
        ++filterSampleIndex;
        if (filterSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            filterSampleIndex = 0;
            medianFilterReady = true;
        }
    }
    return medianFilterReady ? quickMedianFilter5(filterSamples) : newReading;
}

static int16_t computePseudoSnr(int32_t newReading) {
    #define SNR_SAMPLES 5
    static int16_t snrSamples[SNR_SAMPLES];
    static uint8_t snrSampleIndex = 0;
    static int32_t previousReading = RANGEFINDER_OUT_OF_RANGE;
    static bool snrReady = false;
    int16_t pseudoSnr = 0;

    snrSamples[snrSampleIndex] = constrain((int)(pow(newReading - previousReading, 2) / 10), 0, 6400);
    ++snrSampleIndex;
    if (snrSampleIndex == SNR_SAMPLES) {
        snrSampleIndex = 0;
        snrReady = true;
    }

    previousReading = newReading;

    if (snrReady) {

        for (uint8_t i = 0; i < SNR_SAMPLES; i++) {
            pseudoSnr += snrSamples[i];
        }

        return constrain(pseudoSnr, 0, 32000);
    } else {
        return RANGEFINDER_OUT_OF_RANGE;
    }
}

/*
 * This is called periodically by the scheduler
 */
timeDelta_t rangefinderUpdate(void)
{
    if (rangefinder.dev.update) {
        rangefinder.dev.update();
    }

    return rangefinder.dev.delayMs * 1000;  // to microseconds
}

bool isSurfaceAltitudeValid() {

    /*
     * Preconditions: raw and calculated altidude > 0
     * SNR lower than threshold
     */ 
    if (
        rangefinder.calculatedAltitude > 0 &&
        rangefinder.rawAltitude > 0 &&
        rangefinder.snr < RANGEFINDER_DYNAMIC_THRESHOLD
    ) {

        /*
         * When critical altitude was determined, distance reported by rangefinder
         * has to be lower than it to assume healthy readout
         */
        if (rangefinder.snrThresholdReached) {
            return (rangefinder.rawAltitude < rangefinder.dynamicDistanceThreshold);
        } else {
            return true;
        }

    } else {
        return false;
    }

}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, RANGEFINDER_OUT_OF_RANGE is returned.
 */
bool rangefinderProcess(float cosTiltAngle)
{
    if (rangefinder.dev.read) {
        const int32_t distance = rangefinder.dev.read();

        DEBUG_SET(DEBUG_RANGEFINDER, 0, distance);

        // If driver reported no new measurement - don't do anything
        if (distance == RANGEFINDER_NO_NEW_DATA) {
            return false;
        }

        if (distance >= 0) {
            rangefinder.lastValidResponseTimeMs = millis();
            rangefinder.rawAltitude = applyMedianFilter(distance);
        }
        else if (distance == RANGEFINDER_OUT_OF_RANGE) {
            rangefinder.lastValidResponseTimeMs = millis();
            rangefinder.rawAltitude = RANGEFINDER_OUT_OF_RANGE;
        }
        else {
            // Invalid response / hardware failure
            rangefinder.rawAltitude = RANGEFINDER_HARDWARE_FAILURE;
        }

        rangefinder.snr = computePseudoSnr(distance);

        if (rangefinder.snrThresholdReached == false && rangefinder.rawAltitude > 0) {

            if (rangefinder.snr < RANGEFINDER_DYNAMIC_THRESHOLD && rangefinder.dynamicDistanceThreshold < rangefinder.rawAltitude) {
                rangefinder.dynamicDistanceThreshold = rangefinder.rawAltitude * RANGEFINDER_DYNAMIC_FACTOR / 100;
            }

            if (rangefinder.snr >= RANGEFINDER_DYNAMIC_THRESHOLD) {
                rangefinder.snrThresholdReached = true;
            }

        }

        DEBUG_SET(DEBUG_RANGEFINDER, 3, rangefinder.snr);

        DEBUG_SET(DEBUG_RANGEFINDER_QUALITY, 0, rangefinder.rawAltitude);
        DEBUG_SET(DEBUG_RANGEFINDER_QUALITY, 1, rangefinder.snrThresholdReached);
        DEBUG_SET(DEBUG_RANGEFINDER_QUALITY, 2, rangefinder.dynamicDistanceThreshold);
        DEBUG_SET(DEBUG_RANGEFINDER_QUALITY, 3, isSurfaceAltitudeValid());

    }
    else {
        // Bad configuration
        rangefinder.rawAltitude = RANGEFINDER_OUT_OF_RANGE;
    }

    /**
    * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
    * the altitude. Returns the computed altitude in centimeters.
    *
    * When the ground is too far away or the tilt is too large, RANGEFINDER_OUT_OF_RANGE is returned.
    */
    if (cosTiltAngle < rangefinder.maxTiltCos || rangefinder.rawAltitude < 0) {
        rangefinder.calculatedAltitude = RANGEFINDER_OUT_OF_RANGE;
    } else {
        rangefinder.calculatedAltitude = rangefinder.rawAltitude * cosTiltAngle;
    }

    DEBUG_SET(DEBUG_RANGEFINDER, 1, rangefinder.rawAltitude);
    DEBUG_SET(DEBUG_RANGEFINDER, 2, rangefinder.calculatedAltitude);

    return true;
}

/**
 * Get the latest altitude that was computed, or RANGEFINDER_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t rangefinderGetLatestAltitude(void)
{
    return rangefinder.calculatedAltitude;
}

int32_t rangefinderGetLatestRawAltitude(void) {
    return rangefinder.rawAltitude;
}

bool rangefinderIsHealthy(void)
{
    return (millis() - rangefinder.lastValidResponseTimeMs) < RANGEFINDER_HARDWARE_TIMEOUT_MS;
}
#endif

