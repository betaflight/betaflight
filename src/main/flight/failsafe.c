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

#include "platform.h"

#include "build/build_config.h"

#include "build/debug.h"

#include "common/axis.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#include "io/beeper.h"

#include "fc/fc_core.h"
#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/controlrate_profile.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

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

PG_REGISTER_WITH_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);

PG_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig,
    .failsafe_delay = 5,                // 0.5 sec
    .failsafe_recovery_delay = 5,       // 0.5 seconds (plus 200ms explicit delay)
    .failsafe_off_delay = 200,          // 20sec
    .failsafe_throttle = 1000,          // default throttle off.
    .failsafe_throttle_low_delay = 100, // default throttle low delay for "just disarm" on failsafe condition
    .failsafe_procedure = 0,            // default full failsafe procedure is 0: auto-landing, 1: drop, 2 : RTH
    .failsafe_fw_roll_angle = -200,     // 20 deg left
    .failsafe_fw_pitch_angle = 100,     // 10 deg dive (yes, positive means dive)
    .failsafe_fw_yaw_rate = -45,        // 45 deg/s left yaw (left is negative, 8s for full turn)
    .failsafe_stick_motion_threshold = 50,
);

typedef enum {
    FAILSAFE_CHANNEL_HOLD,      // Hold last known good value
    FAILSAFE_CHANNEL_NEUTRAL,   // RPY = zero, THR = zero
    FAILSAFE_CHANNEL_AUTO,      // Defined by failsafe configured values
} failsafeChannelBehavior_e;

typedef struct {
    bool                        forceAngleMode;
    failsafeChannelBehavior_e   channelBehavior[4];
} failsafeProcedureLogic_t;

static const failsafeProcedureLogic_t failsafeProcedureLogic[] = {
    [FAILSAFE_PROCEDURE_AUTO_LANDING] = {
            .forceAngleMode = true,
            .channelBehavior = {
                FAILSAFE_CHANNEL_AUTO,          // ROLL
                FAILSAFE_CHANNEL_AUTO,          // PITCH
                FAILSAFE_CHANNEL_AUTO,          // YAW
                FAILSAFE_CHANNEL_AUTO           // THROTTLE
            }
    },

    [FAILSAFE_PROCEDURE_DROP_IT] = {
            .forceAngleMode = true,
            .channelBehavior = {
                FAILSAFE_CHANNEL_NEUTRAL,       // ROLL
                FAILSAFE_CHANNEL_NEUTRAL,       // PITCH
                FAILSAFE_CHANNEL_NEUTRAL,       // YAW
                FAILSAFE_CHANNEL_NEUTRAL        // THROTTLE
            }
    },

    [FAILSAFE_PROCEDURE_RTH] = {
            .forceAngleMode = true,
            .channelBehavior = {
                FAILSAFE_CHANNEL_NEUTRAL,       // ROLL
                FAILSAFE_CHANNEL_NEUTRAL,       // PITCH
                FAILSAFE_CHANNEL_NEUTRAL,       // YAW
                FAILSAFE_CHANNEL_HOLD           // THROTTLE
            }
    },

    [FAILSAFE_PROCEDURE_NONE] = {
            .forceAngleMode = false,
            .channelBehavior = {
                FAILSAFE_CHANNEL_HOLD,          // ROLL
                FAILSAFE_CHANNEL_HOLD,          // PITCH
                FAILSAFE_CHANNEL_HOLD,          // YAW
                FAILSAFE_CHANNEL_HOLD           // THROTTLE
            }
    }
};

/*
 * Should called when the failsafe config needs to be changed - e.g. a different profile has been selected.
 */
void failsafeReset(void)
{
    failsafeState.rxDataFailurePeriod = PERIOD_RXDATA_FAILURE + failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND;
    failsafeState.rxDataRecoveryPeriod = PERIOD_RXDATA_RECOVERY + failsafeConfig()->failsafe_recovery_delay * MILLIS_PER_TENTH_SECOND;
    failsafeState.validRxDataReceivedAt = 0;
    failsafeState.validRxDataFailedAt = 0;
    failsafeState.throttleLowPeriod = 0;
    failsafeState.landingShouldBeFinishedAt = 0;
    failsafeState.receivingRxDataPeriod = 0;
    failsafeState.receivingRxDataPeriodPreset = 0;
    failsafeState.phase = FAILSAFE_IDLE;
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;

    failsafeState.lastGoodRcCommand[ROLL] = 0;
    failsafeState.lastGoodRcCommand[PITCH] = 0;
    failsafeState.lastGoodRcCommand[YAW] = 0;
    failsafeState.lastGoodRcCommand[THROTTLE] = 1000;
}

void failsafeInit(void)
{
    failsafeState.events = 0;
    failsafeState.monitoring = false;

    return;
}

#ifdef NAV
bool failsafeMayRequireNavigationMode(void)
{
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_RTH;
}
#endif

failsafePhase_e failsafePhase()
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

bool failsafeShouldApplyControlInput(void)
{
    return failsafeState.controlling;
}

bool failsafeRequiresAngleMode(void)
{
    return failsafeState.active && failsafeState.controlling && failsafeProcedureLogic[failsafeConfig()->failsafe_procedure].forceAngleMode;
}

bool failsafeRequiresMotorStop(void)
{
    return failsafeState.active &&
           failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_AUTO_LANDING &&
           failsafeConfig()->failsafe_throttle < motorConfig()->minthrottle;
}

void failsafeStartMonitoring(void)
{
    failsafeState.monitoring = true;
}

static bool failsafeShouldHaveCausedLandingByNow(void)
{
    return failsafeConfig()->failsafe_off_delay && (millis() > failsafeState.landingShouldBeFinishedAt);
}

static void failsafeActivate(failsafePhase_e newPhase)
{
    failsafeState.active = true;
    failsafeState.controlling = true;
    failsafeState.phase = newPhase;
    ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
    failsafeState.landingShouldBeFinishedAt = millis() + failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;

    failsafeState.events++;
}

void failsafeUpdateRcCommandValues(void)
{
    if (!failsafeState.active) {
        for (int idx = 0; idx < 4; idx++) {
            failsafeState.lastGoodRcCommand[idx] = rcCommand[idx];
        }
    }
}

void failsafeApplyControlInput(void)
{
    // Prepare FAILSAFE_CHANNEL_AUTO values for rcCommand
    int16_t autoRcCommand[4];
    if (STATE(FIXED_WING)) {
        autoRcCommand[ROLL] = pidAngleToRcCommand(failsafeConfig()->failsafe_fw_roll_angle, pidProfile()->max_angle_inclination[FD_ROLL]);
        autoRcCommand[PITCH] = pidAngleToRcCommand(failsafeConfig()->failsafe_fw_pitch_angle, pidProfile()->max_angle_inclination[FD_PITCH]);
        autoRcCommand[YAW] = -pidRateToRcCommand(failsafeConfig()->failsafe_fw_yaw_rate, currentControlRateProfile->rates[FD_YAW]);
        autoRcCommand[THROTTLE] = failsafeConfig()->failsafe_throttle;
    }
    else {
        for (int i = 0; i < 3; i++) {
            autoRcCommand[i] = 0;
        }
        autoRcCommand[THROTTLE] = failsafeConfig()->failsafe_throttle;
    }

    // Apply channel values
    for (int idx = 0; idx < 4; idx++) {
        switch (failsafeProcedureLogic[failsafeConfig()->failsafe_procedure].channelBehavior[idx]) {
            case FAILSAFE_CHANNEL_HOLD:
                rcCommand[idx] = failsafeState.lastGoodRcCommand[idx];
                break;

            case FAILSAFE_CHANNEL_NEUTRAL:
                switch (idx) {
                    case ROLL:
                    case PITCH:
                    case YAW:
                        rcCommand[idx] = 0;
                        break;

                    case THROTTLE:
                        rcCommand[idx] = feature(FEATURE_3D) ? rxConfig()->midrc : motorConfig()->minthrottle;
                        break;
                }
                break;

            case FAILSAFE_CHANNEL_AUTO:
                rcCommand[idx] = autoRcCommand[idx];
                break;
        }
    }
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
    if ((failsafeState.validRxDataReceivedAt - failsafeState.validRxDataFailedAt) > failsafeState.rxDataRecoveryPeriod) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
    }
}

void failsafeOnValidDataFailed(void)
{
    failsafeState.validRxDataFailedAt = millis();
    if ((failsafeState.validRxDataFailedAt - failsafeState.validRxDataReceivedAt) > failsafeState.rxDataFailurePeriod) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
    }
}

static bool failsafeCheckStickMotion(void)
{
    if (failsafeConfig()->failsafe_stick_motion_threshold > 0) {
        uint32_t totalRcDelta = 0;

        totalRcDelta += ABS(rcData[ROLL] - rxConfig()->midrc);
        totalRcDelta += ABS(rcData[PITCH] - rxConfig()->midrc);
        totalRcDelta += ABS(rcData[YAW] - rxConfig()->midrc);

        return totalRcDelta >= failsafeConfig()->failsafe_stick_motion_threshold;
    }
    else {
        return true;
    }
}

void failsafeUpdateState(void)
{
    if (!failsafeIsMonitoring()) {
        return;
    }

    const bool receivingRxData = failsafeIsReceivingRxData();
    const bool armed = ARMING_FLAG(ARMED);
    const bool sticksAreMoving = failsafeCheckStickMotion();
    beeperMode_e beeperMode = BEEPER_SILENCE;

    // Beep RX lost only if we are not seeing data and we have been armed earlier
    if (!receivingRxData && ARMING_FLAG(WAS_EVER_ARMED)) {
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
                    if (!receivingRxData) {
                        if ((failsafeConfig()->failsafe_throttle_low_delay && (millis() > failsafeState.throttleLowPeriod)) || STATE(NAV_MOTOR_STOP_OR_IDLE)) {
                            // JustDisarm: throttle was LOW for at least 'failsafe_throttle_low_delay' seconds or waiting for launch
                            // Don't disarm at all if `failsafe_throttle_low_delay` is set to zero
                            failsafeActivate(FAILSAFE_LANDED);  // skip auto-landing procedure
                            failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; // require 3 seconds of valid rxData
                        } else {
                            failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;
                        }
                        reprocessState = true;
                    }
                } else {
                    // When NOT armed, show rxLinkState of failsafe switch in GUI (failsafe mode)
                    if (IS_RC_MODE_ACTIVE(BOXFAILSAFE) || !receivingRxData) {
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
                            failsafeActivate(FAILSAFE_LANDING);
                            break;

                        case FAILSAFE_PROCEDURE_DROP_IT:
                            // Drop the craft
                            failsafeActivate(FAILSAFE_LANDED);      // skip auto-landing procedure
                            failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; // require 3 seconds of valid rxData
                            break;

#if defined(NAV)
                        case FAILSAFE_PROCEDURE_RTH:
                            // Proceed to handling & monitoring RTH navigation
                            failsafeActivate(FAILSAFE_RETURN_TO_HOME);
                            activateForcedRTH();
                            break;
#endif
                        case FAILSAFE_PROCEDURE_NONE:
                        default:
                            // Do nothing procedure
                            failsafeActivate(FAILSAFE_RX_LOSS_IDLE);
                            break;
                    }
                }
                reprocessState = true;
                break;

            /* A very simple do-nothing failsafe procedure. The only thing it will do is monitor the receiver state and switch out of FAILSAFE condition */
            case FAILSAFE_RX_LOSS_IDLE:
                if (receivingRxData && sticksAreMoving) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = true;
                }
                break;

#if defined(NAV)
            case FAILSAFE_RETURN_TO_HOME:
                if (receivingRxData && sticksAreMoving) {
                    abortForcedRTH();
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = true;
                }
                else {
                    if (armed) {
                        beeperMode = BEEPER_RX_LOST_LANDING;
                    }
                    bool rthLanded = false;
                    switch (getStateOfForcedRTH()) {
                        case RTH_IN_PROGRESS:
                            break;

                        case RTH_HAS_LANDED:
                            rthLanded = true;
                            break;

                        case RTH_IDLE:
                        default:
                            // This shouldn't happen. If RTH was somehow aborted during failsafe - fallback to FAILSAFE_LANDING procedure
                            abortForcedRTH();
                            failsafeActivate(FAILSAFE_LANDING);
                            reprocessState = true;
                            break;
                    }
                    if (rthLanded || !armed) {
                        failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS; // require 30 seconds of valid rxData
                        failsafeState.phase = FAILSAFE_LANDED;
                        reprocessState = true;
                    }
                }
                break;
#endif

            case FAILSAFE_LANDING:
                if (receivingRxData && sticksAreMoving) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = true;
                }
                if (armed) {
                    beeperMode = BEEPER_RX_LOST_LANDING;
                }
                if (failsafeShouldHaveCausedLandingByNow() || !armed) {
                    failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS; // require 30 seconds of valid rxData
                    failsafeState.phase = FAILSAFE_LANDED;
                    reprocessState = true;
                }
                break;

            case FAILSAFE_LANDED:
                ENABLE_ARMING_FLAG(ARMING_DISABLED_FAILSAFE_SYSTEM); // To prevent accidently rearming by an intermittent rx link
                mwDisarm(DISARM_FAILSAFE);
                failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset; // set required period of valid rxData
                failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
                failsafeState.controlling = false;  // Failsafe no longer in control of the machine - release control to pilot
                reprocessState = true;
                break;

            case FAILSAFE_RX_LOSS_MONITORING:
                // Monitoring the rx link to allow rearming when it has become good for > `receivingRxDataPeriodPreset` time.
                if (receivingRxData) {
                    if (millis() > failsafeState.receivingRxDataPeriod) {
                        // rx link is good now, when arming via ARM switch, it must be OFF first
                        if (!(!isUsingSticksForArming() && IS_RC_MODE_ACTIVE(BOXARM))) {
                            DISABLE_ARMING_FLAG(ARMING_DISABLED_FAILSAFE_SYSTEM);
                            failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                            reprocessState = true;
                        }
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
                failsafeState.controlling = false;
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
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
