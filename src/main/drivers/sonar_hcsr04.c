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

#include "platform.h"

#if defined(USE_SONAR)

#include "build/build_config.h"

#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/time.h"
#include "drivers/altimeter.h"

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When triggered it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

STATIC_UNIT_TESTED volatile int32_t measurement = -1;
static uint32_t lastMeasurementAt;

extiCallbackRec_t hcsr04_extiCallbackRec;

static IO_t echoIO;
static IO_t triggerIO;

static const altimeterDevice_t hcsr04Device; // Forward

void hcsr04_extiHandler(extiCallbackRec_t* cb)
{
    static uint32_t timing_start;
    uint32_t timing_stop;
    UNUSED(cb);

    if (IORead(echoIO) != 0) {
        timing_start = micros();
    }
    else {
        timing_stop = micros();
        if (timing_stop > timing_start) {
            measurement = timing_stop - timing_start;
        }
    }
}

const altimeterDevice_t *hcsr04_init(const sonarConfig_t *sonarConfig)
{
#if !defined(UNIT_TEST)

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#if defined(STM32F3) || defined(STM32F4)
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

    // trigger and echo pin
    triggerIO = IOGetByTag(sonarConfig->triggerTag);
    echoIO = IOGetByTag(sonarConfig->echoTag);

    if (!triggerIO || !echoIO) {
        return NULL;
    }

    IOInit(triggerIO, OWNER_SONAR_TRIGGER, 0);
    IOConfigGPIO(triggerIO, IOCFG_OUT_PP);

    IOInit(echoIO, OWNER_SONAR_ECHO, 0);
    IOConfigGPIO(echoIO, IOCFG_IN_FLOATING);

#ifdef USE_EXTI
    EXTIHandlerInit(&hcsr04_extiCallbackRec, hcsr04_extiHandler);
    EXTIConfig(echoIO, &hcsr04_extiCallbackRec, NVIC_PRIO_SONAR_EXTI, EXTI_Trigger_Rising_Falling); // TODO - priority!
    EXTIEnable(echoIO, true);
#endif

    lastMeasurementAt = millis() - 60; // force 1st measurement in hcsr04_get_distance()
#else
    UNUSED(lastMeasurementAt); // to avoid "unused" compiler warning
#endif

    return (&hcsr04Device);
}

// measurement reading is done asynchronously, using interrupt
void hcsr04_start_reading(void)
{
#if !defined(UNIT_TEST)
    uint32_t now = millis();

    if (now < (lastMeasurementAt + 60)) {
        // the repeat interval of trig signal should be greater than 60ms
        // to avoid interference between connective measurements.
        return;
    }

    lastMeasurementAt = now;

    IOHi(triggerIO);
    //  The width of trig signal must be greater than 10us
    delayMicroseconds(11);
    IOLo(triggerIO);
#endif
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t hcsr04_get_distance(void)
{
    // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance traveled.
    //
    // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
    int32_t distance = measurement / 59;

    if (distance > HCSR04_MAX_RANGE_CM) {
        distance = ALTIMETER_OUT_OF_RANGE;
    }

    return distance;
}

static const altimeterVTable_t hcsr04VTable = {
    hcsr04_start_reading,
    hcsr04_get_distance,
};

static const altimeterRange_t hcsr04Range = {
    .maxRangeCm = HCSR04_MAX_RANGE_CM,
    .detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES,
    .detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES
};

static const altimeterDevice_t hcsr04Device = {
    &hcsr04Range,
    &hcsr04VTable,
};
#endif
