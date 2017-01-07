/* Copyright spell goes here */

//
// Simple timer measuring duration between two pulses and store it as aigtimeed.
//

#include <stdbool.h>
#include <stdint.h>

#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

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
uint32_t gpioTimerValueMs = 0;

uint16_t guardTimeMs;

static void gpioTimerExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    uint32_t newPulseMs = millis();

    gpioTimerValueMs = newPulseMs - lastPulseMs;
    lastPulseMs = newPulseMs;

    debug[0] = gpioTimerValueMs;

    if (guardTimeMs)
        EXTIEnable(gtimIO, false);
}

void gpioTimerReset(void)
{
    lastPulseMs = 0;
}

bool gpioTimerInit(gpioTimerConfig_t *gpioTimerConfig)
{
    gtimIO = IOGetByTag(gpioTimerConfig->ioTag);

    if (!gtimIO)
        return false;

    guardTimeMs = gpioTimerConfig->guardTimeMs;

    IOInit(gtimIO, OWNER_GPIOTIMER, 0);
    IOConfigGPIO(gtimIO, IOCFG_IPU);

    EXTIHandlerInit(&gtim_extiCallbackRec, gpioTimerExtiHandler);

    // Configure EXTI according to polarity config.
    EXTIConfig(gtimIO, &gtim_extiCallbackRec, NVIC_PRIO_GPIOTIMER, gpioTimerConfig->polarity ? EXTI_Trigger_Rising : EXTI_Trigger_Falling);

    return true;
}

void gpioTimerRearm(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    debug[1]++;

    if (millis() - lastPulseMs > guardTimeMs) {
        debug[2]++;
        EXTIEnable(gtimIO, true);
    }
}

#endif // USE_GPIOTIMER
