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

#include "build/debug.h"

#include "common/axis.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/time.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"

#include "io/beeper.h"

#include "rx/rx.h"

#include "flight/pid.h"

/*
 * Usage:
 *
 * failsafeInit() and failsafeReset() must be called before the other methods are used.
 *
 * failsafeInit() and failsafeReset() can be called in any order.
 * failsafeInit() should only be called once.
 *
 * enable() should be called after system initialisation.
 */

static failsafeState_t failsafeState;

PG_REGISTER_WITH_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 2);

PG_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig,
    .failsafe_throttle = 1000,                       // default throttle off.
    .failsafe_throttle_low_delay = 100,              // default throttle low delay for "just disarm" on failsafe condition
    .failsafe_delay = 10,                                // 1 sec, can regain control on signal recovery, at idle in drop mode
    .failsafe_off_delay = 10,                            // 1 sec in landing phase, if enabled
    .failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE1, // default failsafe switch action is identical to rc link loss
    .failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT,// default full failsafe procedure is 0: auto-landing
    .failsafe_recovery_delay = 10,                       // 1 sec of valid rx data needed to allow recovering from failsafe procedure
    .failsafe_stick_threshold = 30                   // 30 percent of stick deflection to exit GPS Rescue procedure
);

const char * const failsafeProcedureNames[FAILSAFE_PROCEDURE_COUNT] = {
    "AUTO-LAND",
    "DROP",
#ifdef USE_GPS_RESCUE
    "GPS-RESCUE",
#endif
};

/*
 * Should called when the failsafe config needs to be changed - e.g. a different profile has been selected.
 */
void failsafeReset(void)
{
    failsafeState.rxDataFailurePeriod = failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND; // time to start stage2
    if (failsafeState.rxDataFailurePeriod < PERIOD_RXDATA_RECOVERY) {
        // PERIOD_RXDATA_RECOVERY (200ms) - avoid transients and ensure reliable arming
        failsafeState.rxDataFailurePeriod = PERIOD_RXDATA_RECOVERY;
    }
    failsafeState.rxDataRecoveryPeriod = failsafeConfig()->failsafe_recovery_delay * MILLIS_PER_TENTH_SECOND;
    if (failsafeState.rxDataRecoveryPeriod < PERIOD_RXDATA_RECOVERY) {
        // PERIOD_RXDATA_RECOVERY (200ms) is the minimum allowed RxData recovery time
        failsafeState.rxDataRecoveryPeriod = PERIOD_RXDATA_RECOVERY;
    }
    failsafeState.validRxDataReceivedAt = 0;
    failsafeState.validRxDataFailedAt = 0;
    failsafeState.throttleLowPeriod = 0;
    failsafeState.landingShouldBeFinishedAt = 0;
    failsafeState.receivingRxDataPeriod = 0;
    failsafeState.receivingRxDataPeriodPreset = 0;
    failsafeState.phase = FAILSAFE_IDLE;
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
}

void failsafeInit(void)
{
    failsafeState.events = 0;
    failsafeState.monitoring = false;

    return;
}

failsafePhase_e failsafePhase(void)
{
    return failsafeState.phase;
}

bool failsafeIsMonitoring(void)
{
    return failsafeState.monitoring;
}

bool failsafeIsActive(void)
{
    return failsafeState.active;
}

#ifdef USE_GPS_RESCUE
bool failsafePhaseIsGpsRescue(void)
{
    return failsafeState.phase == FAILSAFE_GPS_RESCUE;
}
#endif

void failsafeStartMonitoring(void)
{
    failsafeState.monitoring = true;
}

static bool failsafeShouldHaveCausedLandingByNow(void)
{
    return (millis() > failsafeState.landingShouldBeFinishedAt);
}

static void failsafeActivate(void)
{
    failsafeState.active = true;

    failsafeState.phase = FAILSAFE_LANDING;

    ENABLE_FLIGHT_MODE(FAILSAFE_MODE);

    failsafeState.landingShouldBeFinishedAt = millis() + failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;

    failsafeState.events++;
}

bool failsafeIsReceivingRxData(void)
{
    return (failsafeState.rxLinkState == FAILSAFE_RXLINK_UP);
}

void failsafeOnRxSuspend(uint32_t usSuspendPeriod)
{
    failsafeState.validRxDataReceivedAt += (usSuspendPeriod / 1000);    // / 1000 to convert micros to millis
}

void failsafeOnRxResume(void)
{
    failsafeState.validRxDataReceivedAt = millis();                     // prevent RX link down trigger, restart rx link up
    failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;                     // do so while rx link is up
}

void failsafeOnValidDataReceived(void)
{
    failsafeState.validRxDataReceivedAt = millis();
    if ((cmp32(failsafeState.validRxDataReceivedAt, failsafeState.validRxDataFailedAt) > (int32_t)failsafeState.rxDataRecoveryPeriod) ||
        (cmp32(failsafeState.validRxDataFailedAt, failsafeState.validRxDataReceivedAt) > 0)) {
        // rxDataRecoveryPeriod defaults to 1.0s with minimum of PERIOD_RXDATA_RECOVERY (200ms)
        failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
        // Allow arming now we have an RX link
        unsetArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
    }
}

void failsafeOnValidDataFailed(void)
{
    failsafeState.validRxDataFailedAt = millis();
    if ((cmp32(failsafeState.validRxDataFailedAt, failsafeState.validRxDataReceivedAt) > (int32_t)failsafeState.rxDataFailurePeriod)) {
        // rxDataFailurePeriod is stage 1 guard time
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
        // Prevent arming with no RX link
        setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
    }
}

void failsafeCheckDataFailurePeriod(void)
// forces link down directly from scheduler?
{
    if (cmp32(millis(), failsafeState.validRxDataReceivedAt) > (int32_t)failsafeState.rxDataFailurePeriod) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
        // Prevent arming with no RX link
        setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
    }
}

uint32_t failsafeFailurePeriodMs(void)
{
    return failsafeState.rxDataFailurePeriod;
}

FAST_CODE_NOINLINE void failsafeUpdateState(void)
// triggered directly, and ONLY, by the cheduler, at 10ms = PERIOD_RXDATA_FAILURE - intervals
{
    if (!failsafeIsMonitoring()) {
        return;
    }

    bool receivingRxData = failsafeIsReceivingRxData();
    // should be true when FAILSAFE_RXLINK_UP
    // FAILSAFE_RXLINK_UP is set in failsafeOnValidDataReceived
    // failsafeOnValidDataReceived runs from detectAndApplySignalLossBehaviour

    bool armed = ARMING_FLAG(ARMED);
    bool failsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    beeperMode_e beeperMode = BEEPER_SILENCE;

    if (failsafeSwitchIsOn && (failsafeConfig()->failsafe_switch_mode == FAILSAFE_SWITCH_MODE_STAGE2)) {
        // Aux switch set to failsafe stage2 emulates loss of signal without waiting
        failsafeOnValidDataFailed();
        receivingRxData = false;
    }

    // Beep RX lost only if we are not seeing data and we have been armed earlier
    if (!receivingRxData && (armed || ARMING_FLAG(WAS_EVER_ARMED))) {
        beeperMode = BEEPER_RX_LOST;
    }

    bool reprocessState;

    do {
        reprocessState = false;

        switch (failsafeState.phase) {
            case FAILSAFE_IDLE:
                if (armed) {
                    // Track throttle command below minimum time
                    if (THROTTLE_HIGH == calculateThrottleStatus()) {
                        failsafeState.throttleLowPeriod = millis() + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                    }
                    if (failsafeSwitchIsOn && (failsafeConfig()->failsafe_switch_mode == FAILSAFE_SWITCH_MODE_KILL)) {
                        // Failsafe switch is configured as KILL switch and is switched ON
                        failsafeActivate();
                        // Skip auto-landing procedure
                        failsafeState.phase = FAILSAFE_LANDED;
                        // Require 3 seconds of valid rxData
                        failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS;
                        reprocessState = true;
                    } else if (!receivingRxData) {
                        if (millis() > failsafeState.throttleLowPeriod
#ifdef USE_GPS_RESCUE
                            && failsafeConfig()->failsafe_procedure != FAILSAFE_PROCEDURE_GPS_RESCUE
#endif
                            ) {
                            // JustDisarm: throttle was LOW for at least 'failsafe_throttle_low_delay' seconds before failsafe
                            // protects against false arming when the Tx is powered up after the quad
                            failsafeActivate();
                            // skip auto-landing procedure
                            failsafeState.phase = FAILSAFE_LANDED;
                            // re-arm at rxDataRecoveryPeriod - default is 1.0 seconds
                            failsafeState.receivingRxDataPeriodPreset = failsafeState.rxDataRecoveryPeriod;
                        } else {
                            failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;
                        }
                        reprocessState = true;
                    }
                } else {
                    // When NOT armed, show rxLinkState of failsafe switch in GUI (failsafe mode)
                    if (failsafeSwitchIsOn) {
                        ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    } else {
                        DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    }
                    // Throttle low period expired (= low long enough for JustDisarm)
                    failsafeState.throttleLowPeriod = 0;
                }
                break;

            case FAILSAFE_RX_LOSS_DETECTED:
                if (receivingRxData) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                } else {
                    switch (failsafeConfig()->failsafe_procedure) {
                        case FAILSAFE_PROCEDURE_AUTO_LANDING:
                            // Stabilize, and set Throttle to specified level
                            failsafeActivate();
                            break;

                        case FAILSAFE_PROCEDURE_DROP_IT:
                            // Drop the craft
                            failsafeActivate();
                            // re-arm at rxDataRecoveryPeriod - default is 1.0 seconds
                            failsafeState.receivingRxDataPeriodPreset = failsafeState.rxDataRecoveryPeriod;
                            // skip auto-landing procedure
                            failsafeState.phase = FAILSAFE_LANDED;
                            break;
#ifdef USE_GPS_RESCUE
                        case FAILSAFE_PROCEDURE_GPS_RESCUE:
                            failsafeActivate();
                            failsafeState.phase = FAILSAFE_GPS_RESCUE;
                            break;
#endif
                    }
                }
                reprocessState = true;
                break;

            case FAILSAFE_LANDING:
                if (receivingRxData) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = true;
                } else {
                    if (armed) {
                        beeperMode = BEEPER_RX_LOST_LANDING;
                    }
                    if (failsafeShouldHaveCausedLandingByNow() || crashRecoveryModeActive() || !armed) {
                        // require 3 seconds of valid rxData
                        failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS;
                        failsafeState.phase = FAILSAFE_LANDED;
                        reprocessState = true;
                    }
                }
                break;
#ifdef USE_GPS_RESCUE
            case FAILSAFE_GPS_RESCUE:
                if (receivingRxData) {
                    if (areSticksActive(failsafeConfig()->failsafe_stick_threshold)) {
                        //  hence we must allow stick inputs during FAILSAFE_GPS_RESCUE see PR #7936 for rationale
                        failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                        reprocessState = true;
                    }
                } else {
                    if (armed) {
                        ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
                        beeperMode = BEEPER_RX_LOST_LANDING;
                    } else {
                        // require 3 seconds of valid rxData
                        failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS;
                        failsafeState.phase = FAILSAFE_LANDED;
                        reprocessState = true;
                    }
                }
                break;
#endif
            case FAILSAFE_LANDED:
                // Prevent accidently rearming by an intermittent rx link
                setArmingDisabled(ARMING_DISABLED_FAILSAFE);
                disarm(DISARM_REASON_FAILSAFE);
                failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset; // set required period of valid rxData
                failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
                reprocessState = true;
                break;

            case FAILSAFE_RX_LOSS_MONITORING:
                // Monitoring the rx link to allow rearming when it has become good for > `receivingRxDataPeriodPreset` time.
                if (receivingRxData) {
                    if (millis() > failsafeState.receivingRxDataPeriod) {
                        // rx link is good now
                        failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                        reprocessState = true;
                    }
                } else {
                    failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset;
                }
                break;

            case FAILSAFE_RX_LOSS_RECOVERED:
                // Entering IDLE with the requirement that throttle first must be at min_check for failsafe_throttle_low_delay period.
                // This is to prevent that JustDisarm is activated on the next iteration.
                // Because that would have the effect of shutting down failsafe handling on intermittent connections.
                failsafeState.throttleLowPeriod = millis() + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                failsafeState.phase = FAILSAFE_IDLE;
                failsafeState.active = false;
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                unsetArmingDisabled(ARMING_DISABLED_FAILSAFE);
                reprocessState = true;
                break;

            default:
                break;
        }
    } while (reprocessState);

    if (beeperMode != BEEPER_SILENCE) {
        beeper(beeperMode);
    }
}
