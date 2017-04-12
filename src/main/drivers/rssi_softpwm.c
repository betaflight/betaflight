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

/*
 * Interrupt based FrSky-style PWM encoded RSSI
 *
 * Created by jflyper
 */

#include <stdbool.h>
#include <stdint.h>

#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/atomic.h"
#include "build/debug.h"

#include "common/maths.h"

#ifdef USE_RSSI_SOFTPWM

#ifndef USE_EXTI
# error Configuration problem: USE_RSSI_SOFTPWM requires USE_EXTI
#endif

#include "system.h"
#include "exti.h"
#include "io.h"
#include "gpio.h"
#include "nvic.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/rssi_softpwm.h"

// Basic resources

PG_REGISTER_WITH_RESET_TEMPLATE(rssiSoftPwmConfig_t, rssiSoftPwmConfig, PG_RSSI_SOFTPWM_CONFIG, 0);

#ifndef RSSI_SOFTPWM_PIN
#define RSSI_SOFTPWM_PIN NONE
#endif

PG_RESET_TEMPLATE(rssiSoftPwmConfig_t, rssiSoftPwmConfig,
    .ioTag = IO_TAG(RSSI_SOFTPWM_PIN),
    .min = 0,
    .minFollow = 0,
    .monitor = 0,
);

static const rssiSoftPwmConfig_t *pRspConfig;
static IO_t rspIO = 0;
static extiCallbackRec_t rsp_extiCallbackRec;

// Output variable
static uint16_t rspValue;
#define RSSI_OUTPUT_MIN     0
#define RSSI_OUTPUT_MAX  1023

// Measurement Control
static volatile bool rspInProgress = false;

// Basic measurement by EXTI
// Take three consequtive durations between interrupts.
// timerCount counts the number of interrupts,
// tstamp array stores us time stamp of each interrupts,
// and tbits record the state of the pin at each interrupt.

static volatile uint8_t timerCount = 0;
static volatile uint32_t tstamp[3];
static volatile uint8_t tbits;

// tbits normally ends up in '101' bit pattern, but if high or low period
// is too short to capture, then it will be other patterns.

#define TBITS_IS_NORMAL(bits)         ((bits) == 5)           // '101' Normal
#define TBITS_MISSED_1ST_RISING(bits) (((bits) & 0x4) == 0)   // '0xx' Short high period
#define TBITS_MISSED_FALLING(bits)    (((bits) & 0x2) == 1)   // 'x1x' Short low
#define TBITS_MISSED_2ND_RISING(bits) (((bits) & 0x1) == 0)   // 'xx0' Short high period

// Smoothing
#define MEDIAN_TAPCOUNT 5
#define QUICKMEDIANFILTER quickMedianFilter5
static int32_t dutyFilterArray[MEDIAN_TAPCOUNT];
static int dutyFilterPos;

static void rspExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    if (!rspInProgress)
        return;

    switch (timerCount) {
    case 0:
        tbits = IORead(rspIO);
        tstamp[0] = microsISR();
        EXTISetTrigger(rspIO, EXTI_Trigger_Rising_Falling);
        ++timerCount;
        break;

    case 1:
        tbits = (tbits << 1) | IORead(rspIO);
        tstamp[1] = microsISR();
        ++timerCount;
        break;

    case 2:
        tbits = (tbits << 1) | IORead(rspIO);
        tstamp[2] = microsISR();
        ++timerCount;
        EXTIEnable(rspIO, false);
        rspInProgress = false;
        break;
    }
}

void rssiSoftPwmInit(const rssiSoftPwmConfig_t *configToUse)
{
    pRspConfig = configToUse;

    rspIO = IOGetByTag(pRspConfig->ioTag);

    if (!rspIO)
        return;

    IOInit(rspIO, OWNER_RSSI_SOFTPWM, 0);
    IOConfigGPIO(rspIO, IOCFG_IPD);

    EXTIHandlerInit(&rsp_extiCallbackRec, rspExtiHandler);

    for (int i = 0 ; i < MEDIAN_TAPCOUNT ; i++)
        dutyFilterArray[i] = 0;

    dutyFilterPos = 0;

    // Configure EXTI; will trigger on the first rising edge.

    rspInProgress = true;
    EXTIConfig(rspIO, &rsp_extiCallbackRec, NVIC_PRIO_SOFTPWM_EXTI, EXTI_Trigger_Rising);
}

//
// rssiSoftPwmUpdate: Scheduler task
//

void rssiSoftPwmUpdate(uint32_t currentTime)
{
    // Exception monitoring
    static uint16_t badtimer = 0;

    UNUSED(currentTime);

    if (!rspIO)
        return;

    uint32_t tCycle;
    uint32_t tHigh;
    uint32_t duty;

    uint16_t minDuty = 100;

    ATOMIC_BLOCK(NVIC_PRIO_SOFTPWM_EXTI) {
        if (rspInProgress) {
            // Did not finish the current measurement.
            rspInProgress = false;
            EXTIEnable(rspIO, false);
        }
    }

    // For monitoring
    tCycle = 0;
    tHigh = 0;

    if (timerCount != 3) {  // Did not finish, assume no signal
        duty = 0;
    } else if (TBITS_IS_NORMAL(tbits)) {
        tCycle = tstamp[2] - tstamp[0];
        tHigh = tstamp[1] - tstamp[0];

        // Reject bad measurement (should not happen)

        if (tCycle > 2000) {
            ++badtimer;
            goto restart;
        }

        duty = (tHigh * 100 * 10) / tCycle;
    } else if (TBITS_MISSED_1ST_RISING(tbits) || TBITS_MISSED_2ND_RISING(tbits)) {
        // Assume very short pulse
        duty = 1;
    } else { // including TBITS_MISSED_FALLING(tbits) case
        // Assume very long pulse
        duty = 999;
    }

    // Apply median filter to smooth out rapid changes

    dutyFilterArray[dutyFilterPos] = duty;
    dutyFilterPos = (dutyFilterPos + 1) % MEDIAN_TAPCOUNT;
    duty = (uint16_t)QUICKMEDIANFILTER(dutyFilterArray);

    if (duty < minDuty)
        minDuty = duty;

    rspValue = scaleRange(duty, pRspConfig->minFollow ? minDuty : pRspConfig->min, 999, RSSI_OUTPUT_MIN, RSSI_OUTPUT_MAX);

restart:
    if (pRspConfig->monitor) {
        debug[0] = tCycle;
        debug[1] = tHigh;
        debug[2] = duty;
        debug[3] = badtimer * 1000 + minduty; // minduty in low 3 digits
    }

    // Start a new measurement

    EXTISetTrigger(rspIO, EXTI_Trigger_Rising);
    rspInProgress = true;
    timerCount = 0;
    EXTIEnable(rspIO, true);
}

bool rssiSoftPwmActive(void)
{
    return (rspIO != IO_NONE);
}

uint16_t rssiSoftPwmRead(void)
{
    return rspValue;
}

#endif // USE_RSSI_SOFTPWM
