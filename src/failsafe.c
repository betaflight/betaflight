#include <stdbool.h>
#include <stdint.h>

#include "rx_common.h"

#include "common/axis.h"
#include "flight_common.h"
#include "runtime_config.h"

#include "failsafe.h"

/*
 * Usage:
 *
 * failsafeInit() and useFailsafeConfig() must be called before the other methods are used.
 *
 * failsafeInit() and useFailsafeConfig() can be called in any order.
 * failsafeInit() should only be called once.
 */

static failsafe_t failsafe;

static failsafeConfig_t *failsafeConfig;

static rxConfig_t *rxConfig;

const failsafeVTable_t failsafeVTable[];

void reset(void)
{
    failsafe.counter = 0;
}

/*
 * Should called when the failsafe config needs to be changed - e.g. a different profile has been selected.
 */
void useFailsafeConfig(failsafeConfig_t *failsafeConfigToUse)
{
    failsafeConfig = failsafeConfigToUse;
    reset();
}

failsafe_t* failsafeInit(rxConfig_t *intialRxConfig)
{
    rxConfig = intialRxConfig;

    failsafe.vTable = failsafeVTable;
    failsafe.events = 0;

    return &failsafe;
}

bool isIdle(void)
{
    return failsafe.counter == 0;
}

bool hasTimerElapsed(void)
{
    return failsafe.counter > (5 * failsafeConfig->failsafe_delay);
}

bool shouldForceLanding(bool armed)
{
    return hasTimerElapsed() && armed;
}

bool shouldHaveCausedLandingByNow(void)
{
    return failsafe.counter > 5 * (failsafeConfig->failsafe_delay + failsafeConfig->failsafe_off_delay);
}

void failsafeAvoidRearm(void)
{
    mwDisarm();             // This will prevent the automatic rearm if failsafe shuts it down and prevents
    f.OK_TO_ARM = 0;        // to restart accidently by just reconnect to the tx - you will have to switch off first to rearm
}

void onValidDataReceived(void)
{
    if (failsafe.counter > 20)
        failsafe.counter -= 20;
    else
        failsafe.counter = 0;
}

void updateState(void)
{
    uint8_t i;

    if (hasTimerElapsed()) {

        if (shouldForceLanding(f.ARMED)) { // Stabilize, and set Throttle to specified level
            for (i = 0; i < 3; i++) {
                rcData[i] = rxConfig->midrc;      // after specified guard time after RC signal is lost (in 0.1sec)
            }
            rcData[THROTTLE] = failsafeConfig->failsafe_throttle;
            failsafe.events++;
        }

        if (shouldHaveCausedLandingByNow() || !f.ARMED) {
            failsafeAvoidRearm();
        }
    }
    failsafe.counter++;
}

const failsafeVTable_t failsafeVTable[] = {
    {
        reset,
        onValidDataReceived,
        shouldForceLanding,
        hasTimerElapsed,
        shouldHaveCausedLandingByNow,
        updateState,
        isIdle
    }
};


