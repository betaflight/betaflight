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
#include "io/beeper.h" // XXX Upward call!!!

// XXX Should be a io/laptimer.c or something that make use of driver level timer !?

#include "drivers/gpiotimer.h"

static IO_t gtimIO;
static extiCallbackRec_t gtim_extiCallbackRec;

static uint32_t lastPulseMs = 0;
uint32_t gpioTimerLapMs = 0;
uint32_t gpioTimerRunningMs;

uint16_t guardTimeMs;

static void gpioTimerExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    uint32_t newPulseMs = millis();

    gpioTimerLapMs = newPulseMs - lastPulseMs;
    lastPulseMs = newPulseMs;

    debug[0] = gpioTimerLapMs;

    if (guardTimeMs)
        EXTIEnable(gtimIO, false);

#ifdef BEEPER
    beeper(BEEPER_LAP_TRIGGER);
#endif
}

void gpioTimerReset(void)
{
    lastPulseMs = millis();
}

bool gpioTimerInit(gpioTimerConfig_t *gpioTimerConfig)
{
    gtimIO = IOGetByTag(gpioTimerConfig->ioTag);

debug[2]++;
    if (!gtimIO)
        return false;

debug[3]++;
    guardTimeMs = gpioTimerConfig->guardTimeMs;

    IOInit(gtimIO, OWNER_GPIOTIMER, 0);
    IOConfigGPIO(gtimIO, IOCFG_IPU);

    EXTIHandlerInit(&gtim_extiCallbackRec, gpioTimerExtiHandler);

    // Configure EXTI according to polarity config.
#ifdef STM32F7
#  ifdef notyet
    // Doesn't work for F7 (yet); EXTIConfig() doesn't have edge selection.
    EXTIConfig(gtimIO, &gtim_extiCallbackRec, NVIC_PRIO_GPIOTIMER,
        gpioTimerConfig->polarity ? 
        ioConfig_t ioconfig ) ; // XXX ioConfig_t encodes edge type???
#  endif
#else
    EXTIConfig(gtimIO, &gtim_extiCallbackRec, NVIC_PRIO_GPIOTIMER, gpioTimerConfig->polarity ? EXTI_Trigger_Rising : EXTI_Trigger_Falling);
#endif

    return true;
}

void gpioTimerRearm(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    gpioTimerRunningMs = millis();

    if (gpioTimerRunningMs - lastPulseMs >= guardTimeMs) {
        EXTIEnable(gtimIO, true);
    }
}

#endif // USE_GPIOTIMER
