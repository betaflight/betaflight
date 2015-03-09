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

#include "common/axis.h"

#include "rx/rx.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "config/runtime_config.h"

#include "flight/failsafe.h"

/*
 * Usage:
 *
 * failsafeInit() and useFailsafeConfig() must be called before the other methods are used.
 *
 * failsafeInit() and useFailsafeConfig() can be called in any order.
 * failsafeInit() should only be called once.
 *
 * enable() should be called after system initialisation.
 */

static failsafeState_t failsafeState;

static failsafeConfig_t *failsafeConfig;

static rxConfig_t *rxConfig;

void failsafeReset(void)
{
    failsafeState.counter = 0;
}

/*
 * Should called when the failsafe config needs to be changed - e.g. a different profile has been selected.
 */
void useFailsafeConfig(failsafeConfig_t *failsafeConfigToUse)
{
    failsafeConfig = failsafeConfigToUse;
    failsafeReset();
}

failsafeState_t* failsafeInit(rxConfig_t *intialRxConfig)
{
    rxConfig = intialRxConfig;

    failsafeState.events = 0;
    failsafeState.enabled = false;

    return &failsafeState;
}

bool failsafeIsIdle(void)
{
    return failsafeState.counter == 0;
}

bool failsafeIsEnabled(void)
{
    return failsafeState.enabled;
}

void failsafeEnable(void)
{
    failsafeState.enabled = true;
}

bool failsafeHasTimerElapsed(void)
{
    return failsafeState.counter > (5 * failsafeConfig->failsafe_delay);
}

bool failsafeShouldForceLanding(bool armed)
{
    return failsafeHasTimerElapsed() && armed;
}

bool failsafeShouldHaveCausedLandingByNow(void)
{
    return failsafeState.counter > 5 * (failsafeConfig->failsafe_delay + failsafeConfig->failsafe_off_delay);
}

static void failsafeAvoidRearm(void)
{
    // This will prevent the automatic rearm if failsafe shuts it down and prevents
    // to restart accidently by just reconnect to the tx - you will have to switch off first to rearm
    ENABLE_ARMING_FLAG(PREVENT_ARMING);
}

static void failsafeOnValidDataReceived(void)
{
    if (failsafeState.counter > 20)
        failsafeState.counter -= 20;
    else
        failsafeState.counter = 0;
}

void failsafeUpdateState(void)
{
    uint8_t i;

    if (!failsafeHasTimerElapsed()) {
        return;
    }

    if (!failsafeIsEnabled()) {
        failsafeReset();
        return;
    }

    if (failsafeShouldForceLanding(ARMING_FLAG(ARMED))) { // Stabilize, and set Throttle to specified level
        failsafeAvoidRearm();

        for (i = 0; i < 3; i++) {
            rcData[i] = rxConfig->midrc;      // after specified guard time after RC signal is lost (in 0.1sec)
        }
        rcData[THROTTLE] = failsafeConfig->failsafe_throttle;
        failsafeState.events++;
    }

    if (failsafeShouldHaveCausedLandingByNow() || !ARMING_FLAG(ARMED)) {
        mwDisarm();
    }
}

/**
 * Should be called once each time RX data is processed by the system.
 */
void failsafeOnRxCycle(void)
{
    failsafeState.counter++;
}

#define REQUIRED_CHANNEL_MASK 0x0F // first 4 channels

// pulse duration is in micro secons (usec)
void failsafeCheckPulse(uint8_t channel, uint16_t pulseDuration)
{
    static uint8_t goodChannelMask = 0;

    if (channel < 4 &&
        pulseDuration > failsafeConfig->failsafe_min_usec &&
        pulseDuration < failsafeConfig->failsafe_max_usec
    ) {
        // if signal is valid - mark channel as OK
        goodChannelMask |= (1 << channel);
    }

    if (goodChannelMask == REQUIRED_CHANNEL_MASK) {
        goodChannelMask = 0;
        failsafeOnValidDataReceived();
    }
}

