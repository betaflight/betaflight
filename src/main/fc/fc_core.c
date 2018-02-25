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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "blackbox/blackbox.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/transponder_ir.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/fc_rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "msp/msp_serial.h"

#include "interface/cli.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/gps.h"
#include "io/motors.h"
#include "io/servos.h"
#include "io/serial.h"
#include "io/statusindicator.h"
#include "io/transponder_ir.h"
#include "io/vtx_control.h"
#include "io/vtx_rtc6705.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "telemetry/telemetry.h"

#include "flight/altitude.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/pid.h"
#include "flight/servos.h"


// June 2013     V2.2-dev

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};


#define GYRO_WATCHDOG_DELAY 80 //  delay for gyro sync

#ifdef USE_RUNAWAY_TAKEOFF
#define RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD         600   // The pidSum threshold required to trigger - corresponds to a pidSum value of 60% (raw 600) in the blackbox viewer
#define RUNAWAY_TAKEOFF_ACTIVATE_DELAY           75000 // (75ms) Time in microseconds where pidSum is above threshold to trigger
#define RUNAWAY_TAKEOFF_DEACTIVATE_STICK_PERCENT 15    // 15% - minimum stick deflection during deactivation phase
#define RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT  100   // 10.0% - pidSum limit during deactivation phase
#define RUNAWAY_TAKEOFF_GYRO_LIMIT_RP            15    // Roll/pitch 15 deg/sec threshold to prevent triggering during bench testing without props
#define RUNAWAY_TAKEOFF_GYRO_LIMIT_YAW           50    // Yaw 50 deg/sec threshold to prevent triggering during bench testing without props
#define RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT    75    // High throttle limit to accelerate deactivation (halves the deactivation delay)

#define DEBUG_RUNAWAY_TAKEOFF_ENABLED_STATE      0
#define DEBUG_RUNAWAY_TAKEOFF_ACTIVATING_DELAY   1
#define DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY 2
#define DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME  3

#define DEBUG_RUNAWAY_TAKEOFF_TRUE  1
#define DEBUG_RUNAWAY_TAKEOFF_FALSE 0
#endif


#if defined(USE_GPS) || defined(USE_MAG)
int16_t magHold;
#endif

static bool flipOverAfterCrashMode = false;

static uint32_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

bool isRXDataNew;
static int lastArmingDisabledReason = 0;

#ifdef USE_RUNAWAY_TAKEOFF
static timeUs_t runawayTakeoffDeactivateUs = 0;
static timeUs_t runawayTakeoffAccumulatedUs = 0;
static bool runawayTakeoffCheckDisabled = false;
static timeUs_t runawayTakeoffTriggerUs = 0;
static bool runawayTakeoffTemporarilyDisabled = false;
#endif


PG_REGISTER_WITH_RESET_TEMPLATE(throttleCorrectionConfig_t, throttleCorrectionConfig, PG_THROTTLE_CORRECTION_CONFIG, 0);

PG_RESET_TEMPLATE(throttleCorrectionConfig_t, throttleCorrectionConfig,
    .throttle_correction_value = 0,      // could 10 with althold or 40 for fpv
    .throttle_correction_angle = 800     // could be 80.0 deg with atlhold or 45.0 for fpv
);

void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
{
    accelerometerConfigMutable()->accelerometerTrims.values.roll += rollAndPitchTrimsDelta->values.roll;
    accelerometerConfigMutable()->accelerometerTrims.values.pitch += rollAndPitchTrimsDelta->values.pitch;

    saveConfigAndNotify();
}

static bool isCalibrating(void)
{
    
#ifdef USE_BARO
    if (sensors(SENSOR_BARO) && !isBaroCalibrationComplete()) {
        return true;
    }
#endif
#ifdef USE_GYRO_IMUF9001
    return false;
#endif
    // Note: compass calibration is handled completely differently, outside of the main loop, see f.CALIBRATE_MAG

    return (!accIsCalibrationComplete() && sensors(SENSOR_ACC)) || (!isGyroCalibrationComplete());
}

void resetArmingDisabled(void)
{
    lastArmingDisabledReason = 0;
}

void updateArmingStatus(void)
{
    if (ARMING_FLAG(ARMED)) {
        LED0_ON;
    } else {
        // Check if the power on arming grace time has elapsed
        if ((getArmingDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME) && (millis() >= systemConfig()->powerOnArmingGraceTime * 1000)) {
            // If so, unset the grace time arming disable flag
            unsetArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
        }

        // If switch is used for arming then check it is not defaulting to on when the RX link recovers from a fault
        if (!isUsingSticksForArming()) {
            static bool hadRx = false;
            const bool haveRx = rxIsReceivingSignal();

            const bool justGotRxBack = !hadRx && haveRx;

            if (justGotRxBack && IS_RC_MODE_ACTIVE(BOXARM)) {
                // If the RX has just started to receive a signal again and the arm switch is on, apply arming restriction
                setArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            } else if (haveRx && !IS_RC_MODE_ACTIVE(BOXARM)) {
                // If RX signal is OK and the arm switch is off, remove arming restriction
                unsetArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            }

            hadRx = haveRx;
        }

        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        }

        if (calculateThrottleStatus() != THROTTLE_LOW) {
            setArmingDisabled(ARMING_DISABLED_THROTTLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_THROTTLE);
        }

        if (!STATE(SMALL_ANGLE) && !IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH)) {
            setArmingDisabled(ARMING_DISABLED_ANGLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ANGLE);
        }

        if (averageSystemLoadPercent > 100) {
            setArmingDisabled(ARMING_DISABLED_LOAD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_LOAD);
        }

        if (isCalibrating()) {
            setArmingDisabled(ARMING_DISABLED_CALIBRATING);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_CALIBRATING);
        }

        if (isModeActivationConditionPresent(BOXPREARM)) {
            if (IS_RC_MODE_ACTIVE(BOXPREARM) && !ARMING_FLAG(WAS_ARMED_WITH_PREARM)) {
                unsetArmingDisabled(ARMING_DISABLED_NOPREARM);
            } else {
                setArmingDisabled(ARMING_DISABLED_NOPREARM);
            }
        }

        if (!isUsingSticksForArming()) {
          /* Ignore ARMING_DISABLED_CALIBRATING if we are going to calibrate gyro on first arm */
          bool ignoreGyro = armingConfig()->gyro_cal_on_first_arm
                         && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_CALIBRATING));

          /* Ignore ARMING_DISABLED_THROTTLE (once arm switch is on) if we are in 3D mode */
          bool ignoreThrottle = feature(FEATURE_3D)
                             && !IS_RC_MODE_ACTIVE(BOX3D)
                             && !flight3DConfig()->switched_mode3d
                             && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_THROTTLE));

#ifdef USE_RUNAWAY_TAKEOFF
           if (!IS_RC_MODE_ACTIVE(BOXARM)) {
               unsetArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
           }
#endif

          // If arming is disabled and the ARM switch is on
          if (isArmingDisabled()
              && !ignoreGyro
              && !ignoreThrottle
              && IS_RC_MODE_ACTIVE(BOXARM)) {
              setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
          } else if (!IS_RC_MODE_ACTIVE(BOXARM)) {
              unsetArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
          }
        }

        if (isArmingDisabled()) {
            warningLedFlash();
        } else {
            warningLedDisable();
        }

        warningLedUpdate();
    }
}

void disarm(void)
{
    if (ARMING_FLAG(ARMED)) {
        DISABLE_ARMING_FLAG(ARMED);

#ifdef USE_BLACKBOX
        if (blackboxConfig()->device) {
            blackboxFinish();
        }
#endif
        BEEP_OFF;
        // if ARMING_DISABLED_RUNAWAY_TAKEOFF is set then we want to play it's beep pattern instead
        if (!(getArmingDisableFlags() & ARMING_DISABLED_RUNAWAY_TAKEOFF)) {
            beeper(BEEPER_DISARMING);      // emit disarm tone
        }
    }
}

void tryArm(void)
{
    if (armingConfig()->gyro_cal_on_first_arm) {
        gyroStartCalibration(true);
    }

    updateArmingStatus();

    if (!isArmingDisabled()) {
        if (ARMING_FLAG(ARMED)) {
            return;
        }
#ifdef USE_DSHOT
        if (isMotorProtocolDshot() && isModeActivationConditionPresent(BOXFLIPOVERAFTERCRASH)) {
            pwmDisableMotors();
            delay(1);

            if (!IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH)) {
                flipOverAfterCrashMode = false;
                if (!feature(FEATURE_3D)) {
                    pwmWriteDshotCommand(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL);
                }
            } else {
                flipOverAfterCrashMode = true;
#ifdef USE_RUNAWAY_TAKEOFF
                runawayTakeoffCheckDisabled = false;
#endif
                if (!feature(FEATURE_3D)) {
                    pwmWriteDshotCommand(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_REVERSED);
                }
            }

            pwmEnableMotors();
        }
#endif

        ENABLE_ARMING_FLAG(ARMED);
        ENABLE_ARMING_FLAG(WAS_EVER_ARMED);

        if (isModeActivationConditionPresent(BOXPREARM)) {
            ENABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
        }
        imuQuaternionHeadfreeOffsetSet();

        disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;   // start disarm timeout, will be extended when throttle is nonzero

        lastArmingDisabledReason = 0;

        //beep to indicate arming
#ifdef USE_GPS
        if (feature(FEATURE_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= 5) {
            beeper(BEEPER_ARMING_GPS_FIX);
        } else {
            beeper(BEEPER_ARMING);
        }
#else
        beeper(BEEPER_ARMING);
#endif

#ifdef USE_RUNAWAY_TAKEOFF
        runawayTakeoffDeactivateUs = 0;
        runawayTakeoffAccumulatedUs = 0;
        runawayTakeoffTriggerUs = 0;
#endif
    } else {
        if (!isFirstArmingGyroCalibrationRunning()) {
            int armingDisabledReason = ffs(getArmingDisableFlags());
            if (lastArmingDisabledReason != armingDisabledReason) {
                lastArmingDisabledReason = armingDisabledReason;

                beeperWarningBeeps(armingDisabledReason);
            }
        }
    }
}

// Automatic ACC Offset Calibration
bool AccInflightCalibrationArmed = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm = false;
bool AccInflightCalibrationActive = false;
uint16_t InflightcalibratingA = 0;

void handleInflightCalibrationStickPosition(void)
{
    if (AccInflightCalibrationMeasurementDone) {
        // trigger saving into eeprom after landing
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    } else {
        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
        if (AccInflightCalibrationArmed) {
            beeper(BEEPER_ACC_CALIBRATION);
        } else {
            beeper(BEEPER_ACC_CALIBRATION_FAIL);
        }
    }
}

static void updateInflightCalibrationState(void)
{
    if (AccInflightCalibrationArmed && ARMING_FLAG(ARMED) && rcData[THROTTLE] > rxConfig()->mincheck && !IS_RC_MODE_ACTIVE(BOXARM)) {   // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = false;
    }
    if (IS_RC_MODE_ACTIVE(BOXCALIB)) {      // Use the Calib Option to activate : Calib = TRUE measurement started, Land and Calib = 0 measurement stored
        if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
            InflightcalibratingA = 50;
        AccInflightCalibrationActive = true;
    } else if (AccInflightCalibrationMeasurementDone && !ARMING_FLAG(ARMED)) {
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    }
}

#if defined(USE_GPS) || defined(USE_MAG)
void updateMagHold(void)
{
    if (ABS(rcCommand[YAW]) < 15 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
        if (STATE(SMALL_ANGLE))
            rcCommand[YAW] -= dif * currentPidProfile->pid[PID_MAG].P / 30;    // 18 deg
    } else
        magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}
#endif

#ifdef USE_VTX_CONTROL
static bool canUpdateVTX(void)
{
#ifdef USE_VTX_RTC6705
    return vtxRTC6705CanUpdate();
#endif
    return true;
}
#endif

#ifdef USE_RUNAWAY_TAKEOFF
// determine if the R/P/Y stick deflection exceeds the specified limit - integer math is good enough here.
bool areSticksActive(uint8_t stickPercentLimit)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis ++) {
        const uint8_t deadband = axis == FD_YAW ? rcControlsConfig()->yaw_deadband : rcControlsConfig()->deadband;
        uint8_t stickPercent = 0;
        if ((rcData[axis] >= PWM_RANGE_MAX) || (rcData[axis] <= PWM_RANGE_MIN)) {
            stickPercent = 100;
        } else {
            if (rcData[axis] > (rxConfig()->midrc + deadband)) {
                stickPercent = ((rcData[axis] - rxConfig()->midrc - deadband) * 100) / (PWM_RANGE_MAX - rxConfig()->midrc - deadband);
            } else if (rcData[axis] < (rxConfig()->midrc - deadband)) {
                stickPercent = ((rxConfig()->midrc - deadband - rcData[axis]) * 100) / (rxConfig()->midrc - deadband - PWM_RANGE_MIN);
            }
        }
        if (stickPercent >= stickPercentLimit) {
            return true;
        }
    }
    return false;
}


// allow temporarily disabling runaway takeoff prevention if we are connected
// to the configurator and the ARMING_DISABLED_MSP flag is cleared.
void runawayTakeoffTemporaryDisable(uint8_t disableFlag)
{
    runawayTakeoffTemporarilyDisabled = disableFlag;
}
#endif


// calculate the throttle stick percent - integer math is good enough here.
uint8_t calculateThrottlePercent(void)
{
    uint8_t ret = 0;
    if (feature(FEATURE_3D)
        && !IS_RC_MODE_ACTIVE(BOX3D)
        && !flight3DConfig()->switched_mode3d) {

        if ((rcData[THROTTLE] >= PWM_RANGE_MAX) || (rcData[THROTTLE] <= PWM_RANGE_MIN)) {
            ret = 100;
        } else {
            if (rcData[THROTTLE] > (rxConfig()->midrc + flight3DConfig()->deadband3d_throttle)) {
                ret = ((rcData[THROTTLE] - rxConfig()->midrc - flight3DConfig()->deadband3d_throttle) * 100) / (PWM_RANGE_MAX - rxConfig()->midrc - flight3DConfig()->deadband3d_throttle);
            } else if (rcData[THROTTLE] < (rxConfig()->midrc - flight3DConfig()->deadband3d_throttle)) {
                ret = ((rxConfig()->midrc - flight3DConfig()->deadband3d_throttle - rcData[THROTTLE]) * 100) / (rxConfig()->midrc - flight3DConfig()->deadband3d_throttle - PWM_RANGE_MIN);
            }
        }
    } else {
        ret = constrain(((rcData[THROTTLE] - rxConfig()->mincheck) * 100) / (PWM_RANGE_MAX - rxConfig()->mincheck), 0, 100);
    }
    return ret;
}


/*
 * processRx called from taskUpdateRxMain
 */
bool processRx(timeUs_t currentTimeUs)
{
    static bool armedBeeperOn = false;
    static bool airmodeIsActivated;

    if (!calculateRxChannelsAndUpdateFailsafe(currentTimeUs)) {
        return false;
    }

    // in 3D mode, we need to be able to disarm by switch at any time
    if (feature(FEATURE_3D)) {
        if (!IS_RC_MODE_ACTIVE(BOXARM))
            disarm();
    }

    updateRSSI(currentTimeUs);

    if (currentTimeUs > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
        failsafeStartMonitoring();
    }
    failsafeUpdateState();

    const throttleStatus_e throttleStatus = calculateThrottleStatus();
    const uint8_t throttlePercent = calculateThrottlePercent();

    if (isAirmodeActive() && ARMING_FLAG(ARMED)) {
        if (throttlePercent >= rxConfig()->airModeActivateThreshold) {
            airmodeIsActivated = true; // Prevent Iterm from being reset
        }
    } else {
        airmodeIsActivated = false;
    }

    /* In airmode Iterm should be prevented to grow when Low thottle and Roll + Pitch Centered.
     This is needed to prevent Iterm winding on the ground, but keep full stabilisation on 0 throttle while in air */
    if (throttleStatus == THROTTLE_LOW && !airmodeIsActivated) {
        pidResetITerm();
        if (currentPidProfile->pidAtMinThrottle)
            pidStabilisationState(PID_STABILISATION_ON);
        else
            pidStabilisationState(PID_STABILISATION_OFF);
    } else {
        pidStabilisationState(PID_STABILISATION_ON);
    }

#ifdef USE_RUNAWAY_TAKEOFF
    // If runaway_takeoff_prevention is enabled, accumulate the amount of time that throttle
    // is above runaway_takeoff_deactivate_throttle with the any of the R/P/Y sticks deflected
    // to at least runaway_takeoff_stick_percent percent while the pidSum on all axis is kept low.
    // Once the amount of accumulated time exceeds runaway_takeoff_deactivate_delay then disable
    // prevention for the remainder of the battery.

    if (ARMING_FLAG(ARMED)
        && pidConfig()->runaway_takeoff_prevention
        && !runawayTakeoffCheckDisabled
        && !flipOverAfterCrashMode
        && !runawayTakeoffTemporarilyDisabled
        && !STATE(FIXED_WING)) {

        // Determine if we're in "flight"
        //   - motors running
        //   - throttle over runaway_takeoff_deactivate_throttle_percent
        //   - sticks are active and have deflection greater than runaway_takeoff_deactivate_stick_percent
        //   - pidSum on all axis is less then runaway_takeoff_deactivate_pidlimit
        bool inStableFlight = false;
        if (!feature(FEATURE_MOTOR_STOP) || isAirmodeActive() || (throttleStatus != THROTTLE_LOW)) { // are motors running?
            const uint8_t lowThrottleLimit = pidConfig()->runaway_takeoff_deactivate_throttle;
            const uint8_t midThrottleLimit = constrain(lowThrottleLimit * 2, lowThrottleLimit * 2, RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT);
            if ((((throttlePercent >= lowThrottleLimit) && areSticksActive(RUNAWAY_TAKEOFF_DEACTIVATE_STICK_PERCENT)) || (throttlePercent >= midThrottleLimit))
                && (fabsf(axisPIDSum[FD_PITCH]) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)
                && (fabsf(axisPIDSum[FD_ROLL]) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)
                && (fabsf(axisPIDSum[FD_YAW]) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)) {
                
                inStableFlight = true;
                if (runawayTakeoffDeactivateUs == 0) {
                    runawayTakeoffDeactivateUs = currentTimeUs;
                }
            }
        }

        // If we're in flight, then accumulate the time and deactivate once it exceeds runaway_takeoff_deactivate_delay milliseconds
        if (inStableFlight) {
            if (runawayTakeoffDeactivateUs == 0) {
                runawayTakeoffDeactivateUs = currentTimeUs;
            }
            uint16_t deactivateDelay = pidConfig()->runaway_takeoff_deactivate_delay;
            // at high throttle levels reduce deactivation delay by 50%
            if (throttlePercent >= RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT) {
                deactivateDelay = deactivateDelay / 2;
            }
            if ((cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs) + runawayTakeoffAccumulatedUs) > deactivateDelay * 1000) {
                runawayTakeoffCheckDisabled = true;
            }

        } else {
            if (runawayTakeoffDeactivateUs != 0) {
                runawayTakeoffAccumulatedUs += cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs);
            }
            runawayTakeoffDeactivateUs = 0;
        }
        if (runawayTakeoffDeactivateUs == 0) {
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_FALSE);
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME, runawayTakeoffAccumulatedUs / 1000);
        } else {
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_TRUE);
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME, (cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs) + runawayTakeoffAccumulatedUs) / 1000);
        }
    } else {
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_FALSE);
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME, DEBUG_RUNAWAY_TAKEOFF_FALSE);
    }
#endif

    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    if (ARMING_FLAG(ARMED)
        && feature(FEATURE_MOTOR_STOP)
        && !STATE(FIXED_WING)
        && !feature(FEATURE_3D)
        && !isAirmodeActive()
    ) {
        if (isUsingSticksForArming()) {
            if (throttleStatus == THROTTLE_LOW) {
                if (armingConfig()->auto_disarm_delay != 0
                    && (int32_t)(disarmAt - millis()) < 0
                ) {
                    // auto-disarm configured and delay is over
                    disarm();
                    armedBeeperOn = false;
                } else {
                    // still armed; do warning beeps while armed
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = true;
                }
            } else {
                // throttle is not low
                if (armingConfig()->auto_disarm_delay != 0) {
                    // extend disarm time
                    disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;
                }

                if (armedBeeperOn) {
                    beeperSilence();
                    armedBeeperOn = false;
                }
            }
        } else {
            // arming is via AUX switch; beep while throttle low
            if (throttleStatus == THROTTLE_LOW) {
                beeper(BEEPER_ARMED);
                armedBeeperOn = true;
            } else if (armedBeeperOn) {
                beeperSilence();
                armedBeeperOn = false;
            }
        }
    }

    processRcStickPositions();

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        updateInflightCalibrationState();
    }

    updateActivatedModes();

#ifdef USE_DSHOT
    /* Enable beep warning when the crash flip mode is active */
    if (isMotorProtocolDshot() && isModeActivationConditionPresent(BOXFLIPOVERAFTERCRASH) && IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH)) {
        beeper(BEEPER_CRASH_FLIP_MODE);
    }
#endif

    if (!cliMode) {
        updateAdjustmentStates();
        processRcAdjustments(currentControlRateProfile);
    }

    bool canUseHorizonMode = true;

    if ((IS_RC_MODE_ACTIVE(BOXANGLE) || failsafeIsActive()) && (sensors(SENSOR_ACC))) {
        // bumpless transfer to Level mode
        canUseHorizonMode = false;

        if (!FLIGHT_MODE(ANGLE_MODE)) {
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ANGLE_MODE); // failsafe support
    }

    if (IS_RC_MODE_ACTIVE(BOXHORIZON) && canUseHorizonMode) {

        DISABLE_FLIGHT_MODE(ANGLE_MODE);

        if (!FLIGHT_MODE(HORIZON_MODE)) {
            ENABLE_FLIGHT_MODE(HORIZON_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(HORIZON_MODE);
    }

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        LED1_ON;
        // increase frequency of attitude task to reduce drift when in angle or horizon mode
        rescheduleTask(TASK_ATTITUDE, TASK_PERIOD_HZ(500));
    } else {
        LED1_OFF;
        rescheduleTask(TASK_ATTITUDE, TASK_PERIOD_HZ(100));
    }

    if (!IS_RC_MODE_ACTIVE(BOXPREARM) && ARMING_FLAG(WAS_ARMED_WITH_PREARM)) {
        DISABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
    }

#if defined(USE_ACC) || defined(USE_MAG)
    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
#if defined(USE_GPS) || defined(USE_MAG)
        if (IS_RC_MODE_ACTIVE(BOXMAG)) {
            if (!FLIGHT_MODE(MAG_MODE)) {
                ENABLE_FLIGHT_MODE(MAG_MODE);
                magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            }
        } else {
            DISABLE_FLIGHT_MODE(MAG_MODE);
        }
#endif
        if (IS_RC_MODE_ACTIVE(BOXHEADFREE)) {
            if (!FLIGHT_MODE(HEADFREE_MODE)) {
                ENABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
        if (IS_RC_MODE_ACTIVE(BOXHEADADJ)) {
            if (imuQuaternionHeadfreeOffsetSet()){
               beeper(BEEPER_RX_SET);
            }
        }
    }
#endif

#ifdef USE_NAV
    if (sensors(SENSOR_GPS)) {
        updateGpsWaypointsAndMode();
    }
#endif

    if (IS_RC_MODE_ACTIVE(BOXPASSTHRU)) {
        ENABLE_FLIGHT_MODE(PASSTHRU_MODE);
    } else {
        DISABLE_FLIGHT_MODE(PASSTHRU_MODE);
    }

    if (mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_AIRPLANE) {
        DISABLE_FLIGHT_MODE(HEADFREE_MODE);
    }

#ifdef USE_TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        if ((!telemetryConfig()->telemetry_switch && ARMING_FLAG(ARMED)) ||
                (telemetryConfig()->telemetry_switch && IS_RC_MODE_ACTIVE(BOXTELEMETRY))) {

            releaseSharedTelemetryPorts();
        } else {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspSerialAllocatePorts();
        }
    }
#endif

#ifdef USE_VTX_CONTROL
    vtxUpdateActivatedChannel();

    if (canUpdateVTX()) {
        handleVTXControlButton();
    }
#endif

    return true;
}

static void subTaskPidController(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_PIDLOOP) {startTime = micros();}
    // PID - note this is function pointer set by setPIDController()
    pidController(currentPidProfile, &accelerometerConfig()->accelerometerTrims, currentTimeUs);
    DEBUG_SET(DEBUG_PIDLOOP, 1, micros() - startTime);

#ifdef USE_RUNAWAY_TAKEOFF
    // Check to see if runaway takeoff detection is active (anti-taz), the pidSum is over the threshold,
    // and gyro rate for any axis is above the limit for at least the activate delay period.
    // If so, disarm for safety
    if (ARMING_FLAG(ARMED)
        && !STATE(FIXED_WING)
        && pidConfig()->runaway_takeoff_prevention
        && !runawayTakeoffCheckDisabled
        && !flipOverAfterCrashMode
        && !runawayTakeoffTemporarilyDisabled
        && (!feature(FEATURE_MOTOR_STOP) || isAirmodeActive() || (calculateThrottleStatus() != THROTTLE_LOW))) {

        if (((fabsf(axisPIDSum[FD_PITCH]) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD)
            || (fabsf(axisPIDSum[FD_ROLL]) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD)
            || (fabsf(axisPIDSum[FD_YAW]) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD))
            && ((ABS(gyroAbsRateDps(FD_PITCH)) > RUNAWAY_TAKEOFF_GYRO_LIMIT_RP)
                || (ABS(gyroAbsRateDps(FD_ROLL)) > RUNAWAY_TAKEOFF_GYRO_LIMIT_RP)
                || (ABS(gyroAbsRateDps(FD_YAW)) > RUNAWAY_TAKEOFF_GYRO_LIMIT_YAW))) {

            if (runawayTakeoffTriggerUs == 0) {
                runawayTakeoffTriggerUs = currentTimeUs + RUNAWAY_TAKEOFF_ACTIVATE_DELAY;
            } else if (currentTimeUs > runawayTakeoffTriggerUs) {
                setArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
                disarm();
            }
        } else {
            runawayTakeoffTriggerUs = 0;
        }
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ENABLED_STATE, DEBUG_RUNAWAY_TAKEOFF_TRUE);
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ACTIVATING_DELAY, runawayTakeoffTriggerUs == 0 ? DEBUG_RUNAWAY_TAKEOFF_FALSE : DEBUG_RUNAWAY_TAKEOFF_TRUE);
    } else {
        runawayTakeoffTriggerUs = 0;
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ENABLED_STATE, DEBUG_RUNAWAY_TAKEOFF_FALSE);
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_FALSE);
    }
#endif
}

static void subTaskMainSubprocesses(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_PIDLOOP) {startTime = micros();}

    // Read out gyro temperature if used for telemmetry
    if (feature(FEATURE_TELEMETRY)) {
        gyroReadTemperature();
    }

#ifdef USE_MAG
    if (sensors(SENSOR_MAG)) {
        updateMagHold();
    }
#endif

#if defined(USE_ALT_HOLD)
    // updateRcCommands sets rcCommand, which is needed by updateAltHoldState and updateSonarAltHoldState
    updateRcCommands();
    if (sensors(SENSOR_BARO) || sensors(SENSOR_RANGEFINDER)) {
        if (FLIGHT_MODE(BARO_MODE) || FLIGHT_MODE(RANGEFINDER_MODE)) {
            applyAltHold();
        }
    }
#endif

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if (isUsingSticksForArming() && rcData[THROTTLE] <= rxConfig()->mincheck
#ifndef USE_QUAD_MIXER_ONLY
#ifdef USE_SERVOS
                && !((mixerConfig()->mixerMode == MIXER_TRI || mixerConfig()->mixerMode == MIXER_CUSTOM_TRI) && servoConfig()->tri_unarmed_servo)
#endif
                && mixerConfig()->mixerMode != MIXER_AIRPLANE
                && mixerConfig()->mixerMode != MIXER_FLYING_WING
#endif
    ) {
        resetYawAxis();
    }

    if (throttleCorrectionConfig()->throttle_correction_value && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
        rcCommand[THROTTLE] += calculateThrottleAngleCorrection(throttleCorrectionConfig()->throttle_correction_value);
    }

    processRcCommand();

#ifdef USE_NAV
    if (sensors(SENSOR_GPS)) {
        if ((FLIGHT_MODE(GPS_HOME_MODE) || FLIGHT_MODE(GPS_HOLD_MODE)) && STATE(GPS_FIX_HOME)) {
            updateGpsStateForHomeAndHoldMode();
        }
    }
#endif

#ifdef USE_SDCARD
    afatfs_poll();
#endif

#ifdef USE_BLACKBOX
    if (!cliMode && blackboxConfig()->device) {
        blackboxUpdate(currentTimeUs);
    }
#else
    UNUSED(currentTimeUs);
#endif

#ifdef USE_TRANSPONDER
    transponderUpdate(currentTimeUs);
#endif
    DEBUG_SET(DEBUG_PIDLOOP, 3, micros() - startTime);
}

static void subTaskMotorUpdate(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_CYCLETIME) {
        startTime = micros();
        static uint32_t previousMotorUpdateTime;
        const uint32_t currentDeltaTime = startTime - previousMotorUpdateTime;
        debug[2] = currentDeltaTime;
        debug[3] = currentDeltaTime - targetPidLooptime;
        previousMotorUpdateTime = startTime;
    } else if (debugMode == DEBUG_PIDLOOP) {
        startTime = micros();
    }

    mixTable(currentTimeUs, currentPidProfile->vbatPidCompensation);

#ifdef USE_SERVOS
    // motor outputs are used as sources for servo mixing, so motors must be calculated using mixTable() before servos.
    if (isMixerUsingServos()) {
        writeServos();
    }
#endif

    writeMotors();

    DEBUG_SET(DEBUG_PIDLOOP, 2, micros() - startTime);
}

// Function for loop trigger
void taskMainPidLoop(timeUs_t currentTimeUs)
{
    static uint8_t pidUpdateCountdown = 0;

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_GYROPID_SYNC)
    if (lockMainPID() != 0) return;
#endif
    //__disable_irq();

    // DEBUG_PIDLOOP, timings for:
    // 0 - gyroUpdate()
    // 1 - pidController()
    // 2 - subTaskMotorUpdate()
    // 3 - subTaskMainSubprocesses()
    //when using dma, this shouldn't run until the dma spi transfer flag is complete
    //gyroUpdateSensor in gyro.c is called by gyroUpdate
    gyroUpdate(currentTimeUs);
    DEBUG_SET(DEBUG_PIDLOOP, 0, micros() - currentTimeUs);

    if (pidUpdateCountdown) {
        pidUpdateCountdown--;
    } else {
        pidUpdateCountdown = pidConfig()->pid_process_denom - 1;
        subTaskPidController(currentTimeUs);
        subTaskMotorUpdate(currentTimeUs);
        subTaskMainSubprocesses(currentTimeUs);
    }

    if (debugMode == DEBUG_CYCLETIME) {
        debug[0] = getTaskDeltaTime(TASK_SELF);
        debug[1] = averageSystemLoadPercent;
    }
    //__enable_irq();
}


bool isFlipOverAfterCrashMode(void)
{
    return flipOverAfterCrashMode;
}
