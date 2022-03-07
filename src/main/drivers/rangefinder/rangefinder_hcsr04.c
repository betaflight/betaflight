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
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RANGEFINDER_HCSR04)

#include "common/time.h"

#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/time.h"

#include "drivers/rangefinder/rangefinder.h"

#include "rangefinder_hcsr04.h"

#define HCSR04_MAX_RANGE_CM 400 // 4m, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_DECIDEGREES 300 // recommended cone angle30 degrees, from HC-SR04 spec sheet
#define HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES 450 // in practice 45 degrees seems to work well


/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When triggered it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

static volatile timeDelta_t hcsr04SonarPulseTravelTime = 0;
static volatile timeMs_t lastMeasurementReceivedAt;
static int32_t lastCalculatedDistance = RANGEFINDER_OUT_OF_RANGE;
static timeMs_t lastMeasurementStartedAt = 0;

static extiCallbackRec_t hcsr04_extiCallbackRec;

static IO_t echoIO;
static IO_t triggerIO;

#if !defined(UNIT_TEST)
void hcsr04_extiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    static timeUs_t timing_start;

    const timeUs_t currentTimeUs = microsISR();

    if (IORead(echoIO) != 0) {
        timing_start = currentTimeUs;
    } else {
        const timeUs_t timing_stop = currentTimeUs;
        if (timing_stop > timing_start) {
            lastMeasurementReceivedAt = currentTimeUs / 1000;
            hcsr04SonarPulseTravelTime = timing_stop - timing_start;
        }
    }
}
#endif

void hcsr04_init(rangefinderDev_t *dev)
{
    UNUSED(dev);
}

#define HCSR04_MINIMUM_FIRING_INTERVAL_MS 60

/*
 * Start a range reading
 * Called periodically by the scheduler
 * Measurement reading is done asynchronously, using interrupt
 */
void hcsr04_start_reading(void)
{
#if !defined(UNIT_TEST)
    IOHi(triggerIO);
    delayMicroseconds(11);
    IOLo(triggerIO);
#endif
}

void hcsr04_update(rangefinderDev_t *dev)
{
    UNUSED(dev);

    const timeMs_t timeNowMs = millis();

    // We should have a valid measurement within 60ms of trigger
    if (lastMeasurementReceivedAt > lastMeasurementStartedAt && (lastMeasurementReceivedAt - lastMeasurementStartedAt) <= HCSR04_MINIMUM_FIRING_INTERVAL_MS) {
        // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
        // The ping travels out and back, so to find the distance of the
        // object we take half of the distance traveled.
        // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59

        lastCalculatedDistance = hcsr04SonarPulseTravelTime / 59;
        if (lastCalculatedDistance > HCSR04_MAX_RANGE_CM) {
            lastCalculatedDistance = RANGEFINDER_OUT_OF_RANGE;
        }
    } else if ((lastMeasurementReceivedAt - lastMeasurementStartedAt) > HCSR04_MINIMUM_FIRING_INTERVAL_MS) {
        // No measurement within reasonable time - indicate failure
        lastCalculatedDistance = RANGEFINDER_HARDWARE_FAILURE;
    }

    // the firing interval of the trigger signal should be greater than 60ms
    // to avoid interference between consecutive measurements
    if (timeNowMs > lastMeasurementStartedAt + HCSR04_MINIMUM_FIRING_INTERVAL_MS) {
        // Trigger a new measurement
        lastMeasurementStartedAt = timeNowMs;
        hcsr04_start_reading();
    }
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t hcsr04_get_distance(rangefinderDev_t *dev)
{
    UNUSED(dev);

    const int32_t result = lastCalculatedDistance;

    lastCalculatedDistance = RANGEFINDER_NO_NEW_DATA;

    return result;
}

bool hcsr04Detect(rangefinderDev_t *dev, const sonarConfig_t * rangefinderHardwarePins)
{
    bool detected = false;

    triggerIO = IOGetByTag(rangefinderHardwarePins->triggerTag);
    echoIO = IOGetByTag(rangefinderHardwarePins->echoTag);

    if (IOGetOwner(triggerIO) || IOGetOwner(echoIO)) {
        return false;
    }

    // trigger pin
    IOInit(triggerIO, OWNER_SONAR_TRIGGER, 0);
    IOConfigGPIO(triggerIO, IOCFG_OUT_PP);
    IOLo(triggerIO);
    delay(100);

    // echo pin
    IOInit(echoIO, OWNER_SONAR_ECHO, 0);
    IOConfigGPIO(echoIO, IOCFG_IN_FLOATING);

    // HC-SR04 echo line should be low by default and should return a response pulse when triggered
    if (IORead(echoIO) == false) {
        for (int i = 0; i < 5 && !detected; i++) {
            timeMs_t requestTime = millis();
            hcsr04_start_reading();

            while ((millis() - requestTime) < HCSR04_MINIMUM_FIRING_INTERVAL_MS) {
                if (IORead(echoIO) == true) {
                    detected = true;
                    break;
                }
            }
        }
    }

    if (detected) {
        // Hardware detected - configure the driver
        EXTIHandlerInit(&hcsr04_extiCallbackRec, hcsr04_extiHandler);
        EXTIConfig(echoIO, &hcsr04_extiCallbackRec, NVIC_PRIO_SONAR_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_BOTH); // TODO - priority!
        EXTIEnable(echoIO);

        dev->delayMs = 100;
        dev->maxRangeCm = HCSR04_MAX_RANGE_CM;
        dev->detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;
        dev->detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;

        dev->init = &hcsr04_init;
        dev->update = &hcsr04_update;
        dev->read = &hcsr04_get_distance;

        return true;
    }
    else {
        // Not detected - free resources
        IORelease(triggerIO);
        IORelease(echoIO);
        return false;
    }
}
#endif
