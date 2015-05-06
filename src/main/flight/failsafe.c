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

#include "debug.h"

#include "common/axis.h"

#include "rx/rx.h"
#include "io/beeper.h"
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

static void failsafeReset(void)
{
    failsafeState.counter = 0;
    failsafeState.phase = FAILSAFE_IDLE;
}

/*
 * Should called when the failsafe config needs to be changed - e.g. a different profile has been selected.
 */
void useFailsafeConfig(failsafeConfig_t *failsafeConfigToUse)
{
    failsafeConfig = failsafeConfigToUse;
    failsafeReset();
}

void failsafeInit(rxConfig_t *intialRxConfig)
{
    rxConfig = intialRxConfig;

    failsafeState.events = 0;
    failsafeState.monitoring = false;

    return;
}

failsafePhase_e failsafePhase()
{
    return failsafeState.phase;
}

#define FAILSAFE_COUNTER_THRESHOLD 20

bool failsafeIsReceivingRxData(void)
{
    return failsafeState.counter <= FAILSAFE_COUNTER_THRESHOLD;
}

bool failsafeIsMonitoring(void)
{
    return failsafeState.monitoring;
}

bool failsafeIsActive(void)
{
    return failsafeState.active;
}

void failsafeStartMonitoring(void)
{
    failsafeState.monitoring = true;
}

static bool failsafeHasTimerElapsed(void)
{
    return failsafeState.counter > (5 * failsafeConfig->failsafe_delay);
}

static bool failsafeShouldForceLanding(bool armed)
{
    return failsafeHasTimerElapsed() && armed;
}

static bool failsafeShouldHaveCausedLandingByNow(void)
{
    return failsafeState.counter > 5 * (failsafeConfig->failsafe_delay + failsafeConfig->failsafe_off_delay);
}

static void failsafeActivate(void)
{
    failsafeState.active = true;
    failsafeState.phase = FAILSAFE_LANDING;

    failsafeState.events++;
}

static void failsafeApplyControlInput(void)
{
    for (int i = 0; i < 3; i++) {
        rcData[i] = rxConfig->midrc;
    }
    rcData[THROTTLE] = failsafeConfig->failsafe_throttle;
}

void failsafeOnValidDataReceived(void)
{
    if (failsafeState.counter > FAILSAFE_COUNTER_THRESHOLD)
        failsafeState.counter -= FAILSAFE_COUNTER_THRESHOLD;
    else
        failsafeState.counter = 0;
}

void failsafeUpdateState(void)
{
    bool receivingRxData = failsafeIsReceivingRxData();
    bool armed = ARMING_FLAG(ARMED);
    beeperMode_e beeperMode = BEEPER_SILENCE;

    if (receivingRxData) {
        failsafeState.phase = FAILSAFE_IDLE;
        failsafeState.active = false;
    } else {
        beeperMode = BEEPER_RX_LOST;
    }


    bool reprocessState;

    do {
        reprocessState = false;

        switch (failsafeState.phase) {
            case FAILSAFE_IDLE:
                if (!receivingRxData && armed) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;

                    reprocessState = true;
                }
                break;

            case FAILSAFE_RX_LOSS_DETECTED:

                if (failsafeShouldForceLanding(armed)) {
                    // Stabilize, and set Throttle to specified level
                    failsafeActivate();

                    reprocessState = true;
                }
                break;

            case FAILSAFE_LANDING:
                if (armed) {
                    failsafeApplyControlInput();
                    beeperMode = BEEPER_RX_LOST_LANDING;
                }

                if (failsafeShouldHaveCausedLandingByNow() || !armed) {

                    failsafeState.phase = FAILSAFE_LANDED;

                    reprocessState = true;

                }
                break;

            case FAILSAFE_LANDED:

                if (!armed) {
                    break;
                }

                // This will prevent the automatic rearm if failsafe shuts it down and prevents
                // to restart accidently by just reconnect to the tx - you will have to switch off first to rearm
                ENABLE_ARMING_FLAG(PREVENT_ARMING);

                failsafeState.active = false;
                mwDisarm();
                break;

            default:
                break;
        }
    } while (reprocessState);

    if (beeperMode != BEEPER_SILENCE) {
        beeper(beeperMode);
    }
}

/**
 * Should be called once when RX data is processed by the system.
 */
void failsafeOnRxCycleStarted(void)
{
    failsafeState.counter++;
}
