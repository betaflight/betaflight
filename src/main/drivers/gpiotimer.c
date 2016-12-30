/* Copyright spell goes here */

//
// Simple timer measuring duration between two pulses and store it as aigtimeed.
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

#ifdef USE_GPIOTIMER

#ifndef USE_EXTI
# error Configuration problem: USE_GPIOTIMER requires USE_EXTI
#endif

#include "system.h"
#include "exti.h"
#include "io.h"
#include "gpio.h"
#include "nvic.h"

#include "drivers/gpiotimer.h"

static IO_t gtimIO;
static extiCallbackRec_t gtim_extiCallbackRec;

static uint32_t lastPulseMs = 0;
uint32_t gpioTimerValue = 0;

static void gpioTimerExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    uint32_t newPulseMs = millis();

    gpioTimerValue = newPulseMs - lastPulseMs;
    lastPulseMs = newPulseMs;

    debug[0] = gpioTimerValue;
}

bool gpioTimerInit(gpioTimerConfig_t *gpioTimerConfig)
{
    gtimIO = IOGetByTag(gpioTimerConfig->ioTag);

    if (!gtimIO)
        return false;

    IOInit(gtimIO, OWNER_GPIOTIMER, 0);
    IOConfigGPIO(gtimIO, IOCFG_IPD);

    EXTIHandlerInit(&gtim_extiCallbackRec, gpioTimerExtiHandler);

    // Configure EXTI according to polarity config.
    EXTIConfig(gtimIO, &gtim_extiCallbackRec, NVIC_PRIO_GPIOTIMER, gpioTimerConfig->polarity ? EXTI_Trigger_Rising : EXTI_Trigger_Falling);

    return true;
}

#endif // USE_GPIOTIMER
