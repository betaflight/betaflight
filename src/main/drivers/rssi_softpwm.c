/* Copyright spell goes here */

//
// Interrupt based (sloppy) FrSky PWM RSSI
//

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
static volatile uint8_t timerState = 0;
static volatile uint16_t tstamp[3];
static volatile uint8_t tbits;

// Smoothing
#define MEDIAN_TAPCOUNT 5
#define QUICKMEDIANFILTER quickMedianFilter5
static int32_t dutyFilterArray[MEDIAN_TAPCOUNT];
static int dutyFilterPos;

// Exception monitoring
static uint16_t exceptions;

static void rspExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    if (!rspInProgress)
        return;

    switch (timerState) {
    case 0:
        tbits = IORead(rspIO);
        tstamp[0] = micros();
        EXTISetTrigger(rspIO, EXTI_Trigger_Rising_Falling);
        ++timerState;
        break;

    case 1:
        tbits = (tbits << 1) | IORead(rspIO);
        tstamp[1] = micros();
        ++timerState;
        break;

    case 2:
        tbits = (tbits << 1) | IORead(rspIO);
        tstamp[2] = micros();
        ++timerState;
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
    UNUSED(currentTime);

    if (!rspIO)
        return;

    int32_t tCycle;
    int32_t tHigh;
    int32_t duty;

    uint16_t minDuty = 100;

    ATOMIC_BLOCK(NVIC_PRIO_SOFTPWM_EXTI) {
        if (rspInProgress) {
            // Did not finish the current measurement.
            rspInProgress = false;
            EXTIEnable(rspIO, false);
        }
    }

    if (timerState != 3) {  // Did not finish, assume no signal
        duty = 0;
    } else if (tbits == 5) {  // '101': Normal pattern
        tCycle = tstamp[2] - tstamp[0];
        tHigh = tstamp[1] - tstamp[0];

        // Reject bad measurement
        // XXX Should find out why this happens (especially the negative case)
        if (tCycle > 2000 || tCycle < 0) {
            exceptions = (exceptions & 0xff00) | ((exceptions & 0xff) + 1);
            goto restart;
        }

        duty = (tHigh * 100 * 10) / tCycle;
    } else if (!(tbits & 0x4)) {  // '0xx': Missed falling edge
        // Assume very short pulse
        duty = 1;
    } else if ((tbits & 0x6) == 0x6) {  // '11x': Missed 2nd rising edge
        // Assume very long pulse
        duty = 999;
    } else {
        // Can't happen!?
        exceptions = ((exceptions & 0xff00) + 1) | (exceptions & 0xff);
        goto restart;
    }

    // Apply median filter to smooth out rapid changes

    dutyFilterArray[dutyFilterPos] = duty;
    dutyFilterPos = (dutyFilterPos + 1) % MEDIAN_TAPCOUNT;
    duty = (uint16_t)QUICKMEDIANFILTER(dutyFilterArray);

    if (duty < minDuty)
        minDuty = duty;

    rspValue = scaleRange(duty, pRspConfig->minFollow ? minDuty : pRspConfig->min, 999, RSSI_OUTPUT_MIN, RSSI_OUTPUT_MAX);

    if (pRspConfig->monitor) {
        debug[0] = tCycle;
        debug[1] = tHigh;
        debug[2] = duty;
        debug[3] = exceptions;
    }

restart:
    // Start a new measurement

    EXTISetTrigger(rspIO, EXTI_Trigger_Rising);
    rspInProgress = true;
    timerState = 0;
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
