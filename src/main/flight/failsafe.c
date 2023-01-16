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
    .failsafe_throttle = 1000,                           // default throttle off.
    .failsafe_throttle_low_delay = 100,                  // default throttle low delay for "just disarm" on failsafe condition
    .failsafe_delay = 15,                                // 1.5 sec stage 1 period, can regain control on signal recovery, at idle in drop mode
    .failsafe_off_delay = 10,                            // 1 sec in landing phase, if enabled
    .failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE1, // default failsafe switch action is identical to rc link loss
    .failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT,    // default full failsafe procedure is 0: auto-landing
    .failsafe_recovery_delay = 10,                       // 1 sec of valid rx data needed to allow recovering from failsafe procedure
    .failsafe_stick_threshold = 30                       // 30 percent of stick deflection to exit GPS Rescue procedure
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
    failsafeState.rxDataFailurePeriod = failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND;
    if (failsafeState.rxDataFailurePeriod < PERIOD_RXDATA_RECOVERY){
        // avoid transients and ensure reliable arming for minimum of PERIOD_RXDATA_RECOVERY (200ms)
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
    failsafeState.receivingRxDataPeriodPreset = failsafeState.rxDataRecoveryPeriod;
    failsafeState.phase = FAILSAFE_IDLE;
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
    failsafeState.failsafeSwitchWasOn = false;
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

bool failsafeIsActive(void) // real or switch-induced stage 2 failsafe
{
    return failsafeState.active;
}

void failsafeStartMonitoring(void)
{
    failsafeState.monitoring = true;
}

static bool failsafeShouldHaveCausedLandingByNow(void)
{
    return (millis() > failsafeState.landingShouldBeFinishedAt);
}

bool failsafeIsReceivingRxData(void)
{
    return (failsafeState.rxLinkState == FAILSAFE_RXLINK_UP);
    // False with failsafe switch or when no valid packets for 100ms or any flight channel invalid for 300ms,
    // stays false until after recovery period expires
    // Link down is the trigger for the various failsafe stage 2 outcomes.
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
// runs when packets are received for more than the signal validation period (100ms)
{
    unsetArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
    // clear RXLOSS in OSD immediately we get a good packet, and un-set its arming block

    failsafeState.validRxDataReceivedAt = millis();

    if (failsafeState.validRxDataFailedAt == 0) {
        // after initialisation, we sometimes only receive valid packets,
        // then we don't know how long the signal has been valid for
        // in this setting, the time the signal first valid is also time it was last valid, so
        // initialise validRxDataFailedAt to the time of the first valid data
        failsafeState.validRxDataFailedAt = failsafeState.validRxDataReceivedAt;
        setArmingDisabled(ARMING_DISABLED_BST);
        // prevent arming until we have valid data for rxDataRecoveryPeriod after initialisation
        // using the BST flag since no other suitable name....
    }

    if (cmp32(failsafeState.validRxDataReceivedAt, failsafeState.validRxDataFailedAt) > (int32_t)failsafeState.receivingRxDataPeriodPreset) {
        // receivingRxDataPeriodPreset is rxDataRecoveryPeriod unless set to zero to allow immediate control recovery after switch induced failsafe
        // rxDataRecoveryPeriod defaults to 1.0s with minimum of PERIOD_RXDATA_RECOVERY (200ms)
        // link is not considered 'up', after it has been 'down', until that recovery period has expired
        failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
        unsetArmingDisabled(ARMING_DISABLED_BST);
    }
}

void failsafeOnValidDataFailed(void)
// runs when packets are lost for more than the signal validation period (100ms)
{
    setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
    //  set RXLOSS in OSD and block arming after 100ms of signal loss (is restored in rx.c immediately signal returns)

    failsafeState.validRxDataFailedAt = millis();
    if ((cmp32(failsafeState.validRxDataFailedAt, failsafeState.validRxDataReceivedAt) > (int32_t)failsafeState.rxDataFailurePeriod)) {
        // sets rxLinkState = DOWN to initiate stage 2 failsafe, if no validated signal for the stage 1 period
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
        // show RXLOSS and block arming
    }
}

void failsafeCheckDataFailurePeriod(void)
// runs directly from scheduler, every 10ms, to validate the link
{
    if (cmp32(millis(), failsafeState.validRxDataReceivedAt) > (int32_t)failsafeState.rxDataFailurePeriod) {
        // sets link DOWN after the stage 1 failsafe period, initiating stage 2
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
// triggered directly, and ONLY, by the scheduler, at 10ms = PERIOD_RXDATA_FAILURE - intervals
{
    if (!failsafeIsMonitoring()) {
        return;
    }

    bool receivingRxData = failsafeIsReceivingRxData();
    // returns state of FAILSAFE_RXLINK_UP
    // FAILSAFE_RXLINK_UP is set in failsafeOnValidDataReceived, after the various Stage 1 and recovery delays
    // failsafeOnValidDataReceived runs from detectAndApplySignalLossBehaviour

    DEBUG_SET(DEBUG_FAILSAFE, 2, receivingRxData); // from Rx alone, not considering switch

    bool armed = ARMING_FLAG(ARMED);
    bool failsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    beeperMode_e beeperMode = BEEPER_SILENCE;

    if (failsafeSwitchIsOn && (failsafeConfig()->failsafe_switch_mode == FAILSAFE_SWITCH_MODE_STAGE2)) {
        // Aux switch set to failsafe stage2 emulates immediate loss of signal without waiting
        failsafeOnValidDataFailed();
        receivingRxData = false;
    }

    // Beep RX lost only if we are not seeing data and are armed or have been armed earlier
    if (!receivingRxData && (armed || ARMING_FLAG(WAS_EVER_ARMED))) {
        beeperMode = BEEPER_RX_LOST;
    }

    bool reprocessState;

    do {
        reprocessState = false;

        switch (failsafeState.phase) {
            case FAILSAFE_IDLE:
                failsafeState.failsafeSwitchWasOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
                if (armed) {
                    // Track throttle command below minimum time
                    if (calculateThrottleStatus() != THROTTLE_LOW) {
                        failsafeState.throttleLowPeriod = millis() + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                    }
                    if (failsafeState.failsafeSwitchWasOn && (failsafeConfig()->failsafe_switch_mode == FAILSAFE_SWITCH_MODE_KILL)) {
                        // Failsafe switch is configured as KILL switch and is switched ON
                        failsafeState.active = true;
                        failsafeState.events++;
                        ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                        failsafeState.phase = FAILSAFE_LANDED;
                        //  go to landed immediately
                        failsafeState.receivingRxDataPeriodPreset = failsafeState.rxDataRecoveryPeriod;
                        //  allow re-arming 1 second after Rx recovery, customisable
                        reprocessState = true;
                    } else if (!receivingRxData) {
                        if (millis() > failsafeState.throttleLowPeriod
#ifdef USE_GPS_RESCUE
                            && failsafeConfig()->failsafe_procedure != FAILSAFE_PROCEDURE_GPS_RESCUE
#endif
                            ) {
                            //  JustDisarm if throttle was LOW for at least 'failsafe_throttle_low_delay' before failsafe
                            //  protects against false arming when the Tx is powered up after the quad
                            failsafeState.active = true;
                            failsafeState.events++;
                            ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                            failsafeState.phase = FAILSAFE_LANDED;
                            //  go directly to FAILSAFE_LANDED
                            failsafeState.receivingRxDataPeriodPreset = failsafeState.rxDataRecoveryPeriod;
                            //  allow re-arming 1 second after Rx recovery, customisable
                        } else {
                            failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;
                        }
                        reprocessState = true;
                    }
                } else {
                    // When NOT armed, enable failsafe mode to show warnings in OSD
                    if (failsafeState.failsafeSwitchWasOn) {
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
                    failsafeState.active = true;
                    failsafeState.events++;
                    switch (failsafeConfig()->failsafe_procedure) {
                        case FAILSAFE_PROCEDURE_AUTO_LANDING:
                            //  Enter Stage 2 with settings for landing mode
                            ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                            failsafeState.phase = FAILSAFE_LANDING;
                            failsafeState.landingShouldBeFinishedAt = millis() + failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;
                            break;

                        case FAILSAFE_PROCEDURE_DROP_IT:
                            ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                            failsafeState.phase = FAILSAFE_LANDED;
                            //  go directly to FAILSAFE_LANDED
                            break;
#ifdef USE_GPS_RESCUE
                        case FAILSAFE_PROCEDURE_GPS_RESCUE:
                            ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
                            failsafeState.phase = FAILSAFE_GPS_RESCUE;
                            break;
#endif
                    }
                    if (failsafeState.failsafeSwitchWasOn) {
                        failsafeState.receivingRxDataPeriodPreset = 0;
                        // recover immediately if failsafe was triggered by a switch
                    } else {
                        failsafeState.receivingRxDataPeriodPreset = failsafeState.rxDataRecoveryPeriod;
                        // recover from true link loss failsafe 1 second after RC Link recovers
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
                        // to manually disarm while Landing, aux channels must be enabled
                        // note also that disarming via arm box must be possible during failsafe in rc_controls.c
                        // this should be blocked during signal not received periods, to avoid false disarms
                        // but should be allowed otherwise, eg after signal recovers, or during switch initiated failsafe
                        failsafeState.phase = FAILSAFE_LANDED;
                        reprocessState = true;
                    }
                }
                break;
#ifdef USE_GPS_RESCUE
            case FAILSAFE_GPS_RESCUE:
                if (receivingRxData) {
                    if (areSticksActive(failsafeConfig()->failsafe_stick_threshold) || failsafeState.failsafeSwitchWasOn) {
                        // exits the rescue immediately if failsafe was initiated by switch, otherwise 
                        // requires stick input to exit the rescue after a true Rx loss failsafe
                        // NB this test requires stick inputs to be received during GPS Rescue see PR #7936 for rationale
                        failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                        reprocessState = true;
                    }
                } else {
                    if (armed) {
                        beeperMode = BEEPER_RX_LOST_LANDING;
                    } else {
                        // to manually disarm while in GPS Rescue, aux channels must be enabled
                        failsafeState.phase = FAILSAFE_LANDED;
                        reprocessState = true;
                    }
                }
                break;
#endif
            case FAILSAFE_LANDED:
                disarm(DISARM_REASON_FAILSAFE);
                setArmingDisabled(ARMING_DISABLED_FAILSAFE);
                //  prevent accidently rearming by an intermittent rx link
                failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset;
                //  customise receivingRxDataPeriod according to type of failsafe
                failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
                reprocessState = true;
                break;

            case FAILSAFE_RX_LOSS_MONITORING:
                // receivingRxData is true when we get valid Rx Data and the recovery period has expired
                // for switch initiated failsafes, the recovery period is zero
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
                // Entering IDLE, terminating failsafe, reset throttle low timer
                failsafeState.throttleLowPeriod = millis() + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                failsafeState.phase = FAILSAFE_IDLE;
                failsafeState.active = false;
#ifdef USE_GPS_RESCUE
                DISABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
#endif
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                unsetArmingDisabled(ARMING_DISABLED_FAILSAFE);
                reprocessState = true;
                break;

            default:
                break;
        }

    DEBUG_SET(DEBUG_FAILSAFE, 0, failsafeState.failsafeSwitchWasOn);
    DEBUG_SET(DEBUG_FAILSAFE, 3, failsafeState.phase);

    } while (reprocessState);

    if (beeperMode != BEEPER_SILENCE) {
        beeper(beeperMode);
    }
}
