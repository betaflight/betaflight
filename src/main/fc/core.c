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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"

#include "build/debug.h"

#include "cli/cli.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/light_led.h"
#include "drivers/motor.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/transponder_ir.h"

#include "fc/controlrate_profile.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"

#include "flight/failsafe.h"
#include "flight/autopilot.h"
#include "flight/launch_wing.h"
#include "flight/gps_rescue.h"
#include "flight/alt_hold.h"
#include "flight/pos_hold.h"

#if ENABLE_FLIGHT_PLAN && !defined(USE_WING)
#include "flight/flight_plan_capture.h"
#endif
#include "flight/flight_plan_nav.h"

#ifdef USE_OSD_NAV_MAP
#include "flight/nav_trail.h"
#endif

#if defined(USE_DYN_NOTCH_FILTER)
#include "flight/dyn_notch_filter.h"
#endif

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/pidaudio.h"
#include "io/serial.h"
#include "io/statusindicator.h"
#include "io/transponder_ir.h"
#include "io/vtx_control.h"
#include "io/vtx_rtc6705.h"

#include "msp/msp_serial.h"

#include "osd/osd.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rc_stats.h"
#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#include "core.h"

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

enum {
    ARMING_DELAYED_DISARMED = 0,
    ARMING_DELAYED_NORMAL = 1,
    ARMING_DELAYED_CRASHFLIP = 2,
    ARMING_DELAYED_LAUNCH_CONTROL = 3,
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

static FAST_DATA_ZERO_INIT uint8_t pidUpdateCounter;

static bool crashFlipModeActive = false;
static timeUs_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero
static int lastArmingDisabledReason = 0;
static timeUs_t lastDisarmTimeUs;
static int tryingToArm = ARMING_DELAYED_DISARMED;

#ifdef USE_RUNAWAY_TAKEOFF
static timeUs_t runawayTakeoffDeactivateUs = 0;
static timeUs_t runawayTakeoffAccumulatedUs = 0;
static bool runawayTakeoffCheckDisabled = false;
static timeUs_t runawayTakeoffTriggerUs = 0;
static bool runawayTakeoffTemporarilyDisabled = false;
#endif

#if defined(USE_WING) && defined(USE_LAUNCH_WING)
// A hold or a mission selected alongside a launch takes over when the launch ends, so it may be on at arm.
MAYBE_UNUSED static inline bool holdFollowsLaunch(void)
{
    return isFixedWing() && IS_RC_MODE_ACTIVE(BOXLAUNCH);
}

MAYBE_UNUSED static inline bool launchOwnsAircraft(void)
{
    return launchWingOwnsAircraft();
}
#else
MAYBE_UNUSED static inline bool holdFollowsLaunch(void)
{
    return false;
}

MAYBE_UNUSED static inline bool launchOwnsAircraft(void)
{
    return FLIGHT_MODE(LAUNCH_MODE);
}
#endif

#ifdef USE_WING
// A wing holds a position by circling it, which it cannot do without holding its height as well.
MAYBE_UNUSED static inline bool altHoldSelected(void)
{
    return IS_RC_MODE_ACTIVE(BOXALTHOLD) || IS_RC_MODE_ACTIVE(BOXPOSHOLD);
}

MAYBE_UNUSED static inline bool posHoldHasAltHold(void)
{
    return FLIGHT_MODE(ALT_HOLD_MODE);
}

// Nor can it stop to wait for the GPS: a mission it is flying carries on through the loss.
MAYBE_UNUSED static inline bool missionKeepsWithoutFix(void)
{
    return isFixedWing() && FLIGHT_MODE(AUTOPILOT_MODE);
}

// Nor come straight down: a rescue's emergency descent circles down.
MAYBE_UNUSED static inline bool rescueDescentCircles(void)
{
    return isFixedWing() && flightPlanNavIsRescueDescentActive();
}
#else
MAYBE_UNUSED static inline bool altHoldSelected(void)
{
    return IS_RC_MODE_ACTIVE(BOXALTHOLD);
}

MAYBE_UNUSED static inline bool posHoldHasAltHold(void)
{
    return true;
}

MAYBE_UNUSED static inline bool missionKeepsWithoutFix(void)
{
    return false;
}

MAYBE_UNUSED static inline bool rescueDescentCircles(void)
{
    return false;
}
#endif

#ifdef USE_LAUNCH_CONTROL
static launchControlState_e launchControlState = LAUNCH_CONTROL_DISABLED;
static bool launchControlPreStaged = false;           // OSD snapshot: disarmed, and launch control will engage on arming
#ifdef USE_LAUNCH_CONTROL_LIFT
static timeUs_t launchControlLiftStartUs = 0;
static timeUs_t launchControlLiftEndUs = 0;
static bool launchControlLiftTriggerSeenOff = false;  // LIFT launches on an off->on edge of the trigger only
static timeUs_t launchControlLiftTroubleStartUs = 0;  // when it started failing to hold level, 0 if holding
static launchControlLiftEndReason_e launchControlLiftEndReason = LIFT_END_TIME;   // why the last lift handed over
static bool launchControlLiftNotLevel = false;        // OSD snapshot: too tilted to send
static uint8_t launchControlLiftStoppedMotor = 0;     // OSD snapshot: staged with this motor (1-based) not turning, 0 = all turning
static float launchControlLiftImpactJerk = 0.0f;      // impact threshold for this lift [g/s], 0 = off
static float launchControlLiftPeakJerk = 0.0f;        // highest jerk seen in the last lift [g/s]
static bool launchControlLiftRan = false;             // a lift actually flew this arm (not skipped by a stick takeoff)
static char launchControlLiftUsedMessage[32];         // "LIFT USED PEAK 250+: SWITCH OFF"
// launch_lift_brake: the end of a lift is a flip upside down and a powered stop
typedef enum {
    LIFT_PHASE_CLIMB = 0,   // the lift itself: lift throttle, self-levelled
    LIFT_PHASE_FLIP,        // time's up: throttle off, rolling upside down
    LIFT_PHASE_BRAKE,       // upside down: brake throttle until the climb stops
} launchControlLiftPhase_e;
static launchControlLiftPhase_e launchControlLiftPhase = LIFT_PHASE_CLIMB;
static timeUs_t launchControlLiftPhaseStartUs = 0;
static timeUs_t launchControlLiftLastUpdateUs = 0;
static float launchControlLiftBrakeFraction = 0.0f;   // brake throttle for this lift (0..1), 0 = no brake
static uint8_t launchControlLiftFlipDirection = LAUNCH_LIFT_FLIP_AUTO;
static float launchControlLiftFlipRate = LAUNCH_CONTROL_LIFT_FLIP_RATE_DEFAULT_DPS;   // [deg/s]
static timeDelta_t launchControlLiftFlipTimeoutUs = 0;
static timeDelta_t launchControlLiftBrakeTimeUs = 0;
static float launchControlLiftRestG = 1.0f;           // vertical specific force while staged and still [g]
static float launchControlLiftClimbSpeed = 0.0f;      // vertical speed since the trigger, from the accelerometer [m/s]
static float launchControlLiftHeight = 0.0f;          // height since the trigger, from the accelerometer [m]
static float launchControlLiftTop = 0.0f;             // highest point of the lift so far [m]
static bool launchControlLiftRising = false;          // handed over still climbing: following the coast to its top
static timeUs_t launchControlLiftTopUs = 0;           // when the top was reached (the OSD's figure is final), 0 = not yet
static float launchControlLiftMotorThrottle = 0.0f;   // what the mixer is given right now (0..1)
static float launchControlLiftRate[XYZ_AXIS_COUNT];   // rate setpoints for the flip and brake [deg/s]
// motor health in the climb
static float launchControlLiftMotorCommand[MAX_SUPPORTED_MOTORS];   // filtered, 0..1
static float launchControlLiftMotorRpm[MAX_SUPPORTED_MOTORS];       // filtered
static bool launchControlLiftMotorFiltersPrimed = false;
static timeUs_t launchControlLiftMotorFaultStartUs = 0;
static timeUs_t launchControlLiftMotorStoppedStartUs = 0;
// Snapshot of the LIFT settings and status, taken in the flight controller tasks. The OSD
// reads these instead of currentPidProfile or canUseLaunchControl(), so no display task
// dereferences the profile or re-runs the arming checks.
static uint16_t launchControlLiftTimeMs = 0;
static float launchControlLiftThrottleFraction = 0.0f;
static bool launchControlLiftStaged = false;
static bool launchControlLiftAwaitingTrigger = false;
static const char *launchControlLiftPreArmMessage = NULL;
static void updateLaunchControlLiftStatus(void);
#endif // USE_LAUNCH_CONTROL_LIFT

const char * const osdLaunchControlModeNames[] = {
    "NORMAL",
    "PITCHONLY",
    "FULL",
#ifdef USE_LAUNCH_CONTROL_LIFT
    "LIFT",
#endif
};
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(throttleCorrectionConfig_t, throttleCorrectionConfig, PG_THROTTLE_CORRECTION_CONFIG, 0);

PG_RESET_TEMPLATE(throttleCorrectionConfig_t, throttleCorrectionConfig,
    .throttle_correction_value = 0,      // could 10 with althold or 40 for fpv
    .throttle_correction_angle = 800     // could be 80.0 deg with atlhold or 45.0 for fpv
);

static bool isCalibrating(void)
{
    return (sensors(SENSOR_GYRO) && !gyroIsCalibrationComplete())
#ifdef USE_ACC
        || (sensors(SENSOR_ACC) && !accIsCalibrationComplete())
#endif
#ifdef USE_BARO
        || (sensors(SENSOR_BARO) && !baroIsCalibrated())
#endif
#ifdef USE_MAG
        || (sensors(SENSOR_MAG) && !compassIsCalibrationComplete())
#endif
        ;
}

#ifdef USE_LAUNCH_CONTROL
static bool canUseLaunchControl(void)
{
    if (!isFixedWing()
        && !isUsingSticksForArming()     // require switch arming for safety
        && IS_RC_MODE_ACTIVE(BOXLAUNCHCONTROL)
#ifdef USE_LAUNCH_CONTROL_LIFT
        // LIFT is sent by its own LAUNCH TRIGGER mode, so it must be set up on a switch
        && (currentPidProfile->launchControlMode != LAUNCH_CONTROL_MODE_LIFT || isModeActivationConditionPresent(BOXLAUNCHTRIGGER))
#endif
        && (!featureIsEnabled(FEATURE_MOTOR_STOP) || isAirmodeEnabled()) // either not using motor_stop, or motor_stop is blocked by Airmode
        && !featureIsEnabled(FEATURE_3D) // pitch control is not 3D aware
        && (flightModeFlags == 0)) {     // don't want to use unless in acro mode
        return true;
    }
    return false;
}
#endif

#ifdef USE_LAUNCH_CONTROL_LIFT
// LIFT self-levels, so it needs a trustworthy attitude, and it only corrects a mild tilt
// from the ground: steeper than that and a leg digs in, or it leaves at an angle.
// No accelerometer, or no attitude yet, can't prove level, so the answer is no.
static bool isLaunchControlLiftLevel(void)
{
#ifdef USE_ACC
    return sensors(SENSOR_ACC)
        && isUpright()   // attitude established (and within small_angle)
        && getCosTiltAngle() > cos_approx(DEGREES_TO_RADIANS(LAUNCH_CONTROL_LIFT_MAX_START_TILT_DEG));
#else
    return false;
#endif
}

// The first motor (1-based) that isn't turning, by RPM telemetry. 0 when every motor is
// turning, or when there's no RPM telemetry to tell.
static uint8_t launchControlLiftFindStoppedMotor(void)
{
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry && isDshotTelemetryActive()) {
        const int motorCount = MIN(getMotorCount(), MAX_SUPPORTED_MOTORS);
        for (int i = 0; i < motorCount; i++) {
            if (getDshotRpm(i) < LAUNCH_CONTROL_LIFT_MOTOR_TURNING_RPM) {
                return i + 1;
            }
        }
    }
#endif
    return 0;
}
#endif

#ifdef USE_DSHOT
static void setMotorSpinDirection(uint8_t spinDirection)
{
    if (isMotorProtocolDshot() && !featureIsEnabled(FEATURE_3D)) {
        dshotCommandWrite(ALL_MOTORS, getMotorCount(), spinDirection, DSHOT_CMD_TYPE_INLINE);
    }
}
#endif

void resetArmingDisabled(void)
{
    lastArmingDisabledReason = 0;
}

#ifdef USE_ACC
static bool accNeedsCalibration(void)
{
    if (sensors(SENSOR_ACC)) {

        // Check to see if the ACC has already been calibrated
        if (accHasBeenCalibrated()) {
            return false;
        }

        // We've determined that there's a detected ACC that has not
        // yet been calibrated. Check to see if anything is using the
        // ACC that would be affected by the lack of calibration.

        // Check for any configured modes that use the ACC
        if (isModeActivationConditionPresent(BOXANGLE) ||
            isModeActivationConditionPresent(BOXHORIZON) ||
            isModeActivationConditionPresent(BOXALTHOLD) ||
            isModeActivationConditionPresent(BOXPOSHOLD) ||
            isModeActivationConditionPresent(BOXGPSRESCUE) ||
#if defined(USE_WING) && defined(USE_LAUNCH_WING)
            (isFixedWing() && isModeActivationConditionPresent(BOXLAUNCH)) ||
#endif
            isModeActivationConditionPresent(BOXCAMSTAB) ||
            isModeActivationConditionPresent(BOXCALIB) ||
            isModeActivationConditionPresent(BOXACROTRAINER)) {

            return true;
        }

        // Launch Control only requires the ACC if a angle limit is set, or for a LIFT, which
        // self-levels, checks the tilt before launching and measures the climb with it
        if (isModeActivationConditionPresent(BOXLAUNCHCONTROL)
            && (currentPidProfile->launchControlAngleLimit
#ifdef USE_LAUNCH_CONTROL_LIFT
                || currentPidProfile->launchControlMode == LAUNCH_CONTROL_MODE_LIFT
#endif
            )) {
            return true;
        }

#ifdef USE_OSD
        // Check for any enabled OSD elements that need the ACC
        if (featureIsEnabled(FEATURE_OSD)) {
            if (osdNeedsAccelerometer()) {
                return true;
            }
        }
#endif

#ifdef USE_GPS_RESCUE
        // Check if failsafe will use GPS Rescue
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE) {
            return true;
        }
#endif
    }

    return false;
}
#endif

void updateArmingStatus(void)
{

    if (ARMING_FLAG(ARMED)) {
        LED0_ON;

#ifdef USE_DSHOT
// --- handle crashFlip behaviours while armed ---
if (crashFlipModeActive) {
    if (!IS_RC_MODE_ACTIVE(BOXCRASHFLIP)) {
        // Pilot has reverted the crash flip switch while crashflip is active and craft is  armed
        if (!mixerConfig()->crashflip_auto_rearm) {
            // we are in manual re-arm mode:  block arming until manual re-arm:
            setArmingDisabled(ARMING_DISABLED_CRASHFLIP); // block tryArm() until user disarms manually
            clearWasLastDisarmUserRequested();
            // tell disarm() that this was not a user generated disarm
            // also clear the flag in rc_controls so that it will only be true when the pilot makes a new user manual disarm
            disarm(DISARM_REASON_CRASHFLIP); // stop the motors, revert crashflipMode and set motor direction normal
        } else {
            // we are in auto re-arm mode, terminate crashflip mode and set motor direction normal
            setMotorSpinDirection(DSHOT_CMD_SPIN_DIRECTION_NORMAL);
            crashFlipModeActive = false;
        }
    }
}
#endif // USE_DSHOT
    } else {
        // arming switch on, but not yet armed; currently DISARMED
        // identify things that should delay, or prevent, arming, then arm

        // Check if the power-on arming grace time has elapsed
        bool graceTimeElapsed =
            (getArmingDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME)
            && (millis() >= systemConfig()->powerOnArmingGraceTime * 1000);

#ifdef USE_DSHOT
        // With DSHOT, we also require DSHOT to be ready
        graceTimeElapsed = graceTimeElapsed && (!isMotorProtocolDshot() || dshotStreamingCommandsAreEnabled());
#endif

        if (graceTimeElapsed) {
            unsetArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
        }
        // If switch is used for arming then check it is not defaulting to on when the RX link recovers from a fault
        if (!isUsingSticksForArming()) {
            static bool hadRx = false;
            const bool haveRx = isRxReceivingSignal();

            const bool justGotRxBack = !hadRx && haveRx;

            if (justGotRxBack && IS_RC_MODE_ACTIVE(BOXARM)) {
                // If the RX has just started to receive a signal again and the arm switch is on, apply arming restriction
                setArmingDisabled(ARMING_DISABLED_NOT_DISARMED);
            } else if (haveRx && !IS_RC_MODE_ACTIVE(BOXARM)) {
                // If RX signal is OK and the arm switch is off, remove arming restriction
                unsetArmingDisabled(ARMING_DISABLED_NOT_DISARMED);
            }
        hadRx = haveRx;
        }

        if (getArmingDisableFlags() & ARMING_DISABLED_CRASHFLIP) {
             // if, while disarmed, arming was blocked due to crashflip, eg in manual mode and the crashflip switch was reverted, watch for a manual user disarm
             if (wasLastDisarmUserRequested()) {
             // if there has been a new manual user disarm since the last time its flag was cleared, eg since the crashflip switch was reverted in manual mode
                unsetArmingDisabled(ARMING_DISABLED_CRASHFLIP);
                // clear the armingDisabled block to permit re-arming
            }
        }

        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        }

        if (IS_RC_MODE_ACTIVE(BOXALTHOLD) && !holdFollowsLaunch()) {
            setArmingDisabled(ARMING_DISABLED_ALTHOLD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ALTHOLD);
        }

        if (IS_RC_MODE_ACTIVE(BOXPOSHOLD) && !holdFollowsLaunch()) {
            setArmingDisabled(ARMING_DISABLED_POSHOLD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_POSHOLD);
        }

#if ENABLE_FLIGHT_PLAN
        if (IS_RC_MODE_ACTIVE(BOXAUTOPILOT) && !holdFollowsLaunch()) {
            setArmingDisabled(ARMING_DISABLED_AUTOPILOT);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_AUTOPILOT);
        }
#endif

        if (calculateThrottleStatus() != THROTTLE_LOW) {
            setArmingDisabled(ARMING_DISABLED_THROTTLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_THROTTLE);
        }

        if (!isUpright() && !IS_RC_MODE_ACTIVE(BOXCRASHFLIP)) {
            setArmingDisabled(ARMING_DISABLED_ANGLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ANGLE);
        }

#if defined(USE_LATE_TASK_STATISTICS)
        if ((getCpuPercentageLate() > schedulerConfig()->cpuLatePercentageLimit)) {
            setArmingDisabled(ARMING_DISABLED_LOAD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_LOAD);
        }
#endif // USE_LATE_TASK_STATISTICS

        if (isCalibrating()) {
            setArmingDisabled(ARMING_DISABLED_CALIBRATING);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_CALIBRATING);
        }

        if (isModeActivationConditionPresent(BOXPREARM)) {
            if (IS_RC_MODE_ACTIVE(BOXPREARM) && (!ARMING_FLAG(WAS_ARMED_WITH_PREARM) || armingConfig()->prearm_allow_rearm) ) {
                unsetArmingDisabled(ARMING_DISABLED_NOPREARM);
            } else {
                setArmingDisabled(ARMING_DISABLED_NOPREARM);
            }
        }

#ifdef USE_GPS_RESCUE
        if (gpsRescueIsConfigured()) {
            if (gpsRescueConfig()->allowArmingWithoutFix || (STATE(GPS_FIX) && (gpsSol.numSat >= gpsRescueConfig()->minSats)) ||
            ARMING_FLAG(WAS_EVER_ARMED) || IS_RC_MODE_ACTIVE(BOXCRASHFLIP)) {
                unsetArmingDisabled(ARMING_DISABLED_GPS);
            } else {
                setArmingDisabled(ARMING_DISABLED_GPS);
            }
            if (IS_RC_MODE_ACTIVE(BOXGPSRESCUE)) {
                setArmingDisabled(ARMING_DISABLED_RESC);
            } else {
                unsetArmingDisabled(ARMING_DISABLED_RESC);
            }
        }
#endif

#ifdef USE_DSHOT_TELEMETRY
        // If Dshot Telemetry is enabled and any motor isn't providing telemetry, then disable arming
        if (useDshotTelemetry && !isDshotTelemetryActive()) {
            setArmingDisabled(ARMING_DISABLED_DSHOT_TELEM);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_DSHOT_TELEM);
        }
#endif

#ifdef USE_DSHOT_BITBANG
        if (isDshotBitbangActive(&motorConfig()->dev) && dshotBitbangGetStatus() != DSHOT_BITBANG_STATUS_OK) {
            setArmingDisabled(ARMING_DISABLED_DSHOT_BITBANG);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_DSHOT_BITBANG);
        }
#endif

        if (IS_RC_MODE_ACTIVE(BOXPARALYZE)) {
            setArmingDisabled(ARMING_DISABLED_PARALYZE);
        }

#ifdef USE_ACC
        if (accNeedsCalibration()) {
            setArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        }
#endif

        if (!isMotorProtocolEnabled()) {
            setArmingDisabled(ARMING_DISABLED_MOTOR_PROTOCOL);
        }

        if (!isUsingSticksForArming()) {
            if (!IS_RC_MODE_ACTIVE(BOXARM)) {
#ifdef USE_RUNAWAY_TAKEOFF
                unsetArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
#endif
                unsetArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
            }

            /* Ignore ARMING_DISABLED_CALIBRATING if we are going to calibrate gyro on first arm */
            bool ignoreGyro = armingConfig()->gyro_cal_on_first_arm
                && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_CALIBRATING));

            /* Ignore ARMING_DISABLED_THROTTLE (once arm switch is on) if we are in 3D mode */
            bool ignoreThrottle = featureIsEnabled(FEATURE_3D)
                 && !IS_RC_MODE_ACTIVE(BOX3D)
                 && !flight3DConfig()->switched_mode3d
                 && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_THROTTLE));

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

void disarm(flightLogDisarmReason_e reason)
{
#if ENABLE_TELEMETRY_MAVLINK_COMMANDS
    // Drop any MAVLink-commanded mode override so it can never persist into a
    // disarmed state and re-assert modes on the next arm.
    rcModeClearExternalOverrides();
#endif

    if (!wasLastDisarmUserRequested()) {
        // Non-user disarm, clear the user-initated flag in rc_controls.c
        clearWasLastDisarmUserRequested();
    }

    if (ARMING_FLAG(ARMED)) {
        if (!crashFlipModeActive) {
            ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
        }
        DISABLE_ARMING_FLAG(ARMED); // disarm now
        lastDisarmTimeUs = micros();
        launchWingDisarm();

#ifdef USE_OSD
        if (IS_RC_MODE_ACTIVE(BOXCRASHFLIP) || isLaunchControlActive()) {
            osdSuppressStats(true);
        }
#endif

#ifdef USE_BLACKBOX
        flightLogEvent_disarm_t eventData;
        eventData.reason = reason;
        blackboxLogEvent(FLIGHT_LOG_EVENT_DISARM, (flightLogEventData_t*)&eventData);

        if (blackboxConfig()->device && blackboxConfig()->mode != BLACKBOX_MODE_ALWAYS_ON) { // Close the log upon disarm except when logging mode is ALWAYS ON
            blackboxFinish();
        }
#else
        UNUSED(reason);
#endif

        BEEP_OFF;

#ifdef USE_PERSISTENT_STATS
        if (!crashFlipModeActive) {
            statsOnDisarm();
        }
#endif
    // Terminate crashflip mode in any disarm
    if (crashFlipModeActive) {
        crashFlipModeActive = false;
    }

        // always set motor direction to normal on disarming
#ifdef USE_DSHOT
        setMotorSpinDirection(DSHOT_CMD_SPIN_DIRECTION_NORMAL);
#endif

        // make disarming beeps, but not if ARMING_DISABLED (RUNAWAY_TAKEOFF or CRASH_DETECTED)
        if (!(getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))) {
            beeper(BEEPER_DISARMING);
        }
    }
}

void tryArm(void)
{
    if (armingConfig()->gyro_cal_on_first_arm) {
        gyroStartCalibration(true);
    }


    updateArmingStatus();
    // set or clear armingDisabled flags, while arming is requested, whether armed or disarmed,


    if (!isArmingDisabled()) {
        if (ARMING_FLAG(ARMED)) {
            return;
            // don't allow arming if there are active armingDisabled flags or already armed
        }
        const timeUs_t currentTimeUs = micros();

#ifdef USE_DSHOT
// Handle DShot factors that should delay arming
timeDelta_t beaconTimeDiff = cmpTimeUs(currentTimeUs, getLastDshotBeaconCommandTimeUs());
if (beaconTimeDiff < DSHOT_BEACON_GUARD_DELAY_US && beaconTimeDiff >= 0) {
    if (tryingToArm == ARMING_DELAYED_DISARMED) {
        #ifdef USE_LAUNCH_CONTROL
        if (canUseLaunchControl()) {
            tryingToArm = ARMING_DELAYED_LAUNCH_CONTROL;
        } else
        #endif
        {
            tryingToArm = ARMING_DELAYED_NORMAL;
        }
    }
    return;
}

if (isMotorProtocolDshot()) {
    #if defined(USE_ESC_SENSOR) && defined(USE_DSHOT_TELEMETRY)
    // Try to activate extended DSHOT telemetry only if no esc sensor exists and dshot telemetry is active
    if (!featureIsEnabled(FEATURE_ESC_SENSOR) && useDshotTelemetry) {
        dshotCleanTelemetryData();
        if (motorConfig()->dev.useDshotEdt) {
            dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE, DSHOT_CMD_TYPE_INLINE);
        }
    }
    #endif

    crashFlipModeActive = IS_RC_MODE_ACTIVE(BOXCRASHFLIP);
    setMotorSpinDirection(crashFlipModeActive ? DSHOT_CMD_SPIN_DIRECTION_REVERSED : DSHOT_CMD_SPIN_DIRECTION_NORMAL);
}
#endif // USE_DSHOT

#ifdef USE_LAUNCH_CONTROL
#ifdef USE_LAUNCH_CONTROL_LIFT
        launchControlLiftEndUs = 0;
#endif
        if (!crashFlipModeActive && (canUseLaunchControl() || (tryingToArm == ARMING_DELAYED_LAUNCH_CONTROL))) {
            if (launchControlState == LAUNCH_CONTROL_DISABLED) {  // only activate if it hasn't already been triggered
                launchControlState = LAUNCH_CONTROL_ACTIVE;
#ifdef USE_LAUNCH_CONTROL_LIFT
                // LIFT: armed with the trigger already on, it must go off and on again
                // to launch, so arming can never be what sends the quad
                launchControlLiftTriggerSeenOff = !IS_RC_MODE_ACTIVE(BOXLAUNCHTRIGGER);
                launchControlLiftRan = false;
                launchControlLiftRestG = 1.0f;   // relearned while it sits staged
#endif
            }
        }
#endif

#ifdef USE_OSD
        osdSuppressStats(false);
#endif
#ifdef USE_RPM_LIMIT
        mixerResetRpmLimiter();
#endif
        // Latched once, here, and nowhere else: a launch can never be switched
        // on mid-air.
        launchWingArm();
        ENABLE_ARMING_FLAG(ARMED);  // ***ARM NOW ***

#ifdef USE_RC_STATS
        NotifyRcStatsArming();
#endif

        resetTryingToArm();

#ifdef USE_ACRO_TRAINER
        pidAcroTrainerInit();
#endif // USE_ACRO_TRAINER

        if (isModeActivationConditionPresent(BOXPREARM)) {
            ENABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
        }
        imuQuaternionHeadfreeOffsetSet();

#if defined(USE_DYN_NOTCH_FILTER)
        resetMaxFFT();
#endif

        disarmAt = currentTimeUs + armingConfig()->auto_disarm_delay * 1000 * 1000;   // start disarm timeout, will be extended when throttle is nonzero

        lastArmingDisabledReason = 0;

#ifdef USE_GPS
        //beep to indicate arming
        if (featureIsEnabled(FEATURE_GPS)) {
            GPS_reset_home_position();
            canUseGPSHeading = false; // block use of GPS Heading in position hold after each arm, until quad can set IMU to GPS COG
            if (STATE(GPS_FIX) && gpsSol.numSat >= gpsRescueConfig()->minSats) {
                beeper(BEEPER_ARMING_GPS_FIX);
            } else {
                beeper(BEEPER_ARMING_GPS_NO_FIX);
            }
        } else {
            beeper(BEEPER_ARMING);
        }
#else
        beeper(BEEPER_ARMING);
#endif

#ifdef USE_PERSISTENT_STATS
        statsOnArm();
#endif

#ifdef USE_RUNAWAY_TAKEOFF
        runawayTakeoffDeactivateUs = 0;
        runawayTakeoffAccumulatedUs = 0;
        runawayTakeoffTriggerUs = 0;
#endif
    } else {
       resetTryingToArm();
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
    if (AccInflightCalibrationArmed && ARMING_FLAG(ARMED) && rcGetChannel(THROTTLE) > rcUsToNorm(rxConfig()->mincheck) && !IS_RC_MODE_ACTIVE(BOXARM)) {   // Copter is airborne and you are turning it off via boxarm : start measurement
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

#ifdef USE_VTX_CONTROL
static bool canUpdateVTX(void)
{
#ifdef USE_VTX_RTC6705
    return vtxRTC6705CanUpdate();
#endif
    return true;
}
#endif

#if defined(USE_RUNAWAY_TAKEOFF) || defined(USE_GPS_RESCUE)
// determine if the R/P/Y stick deflection exceeds the specified limit - integer math is good enough here.
bool areSticksActive(uint8_t stickPercentLimit)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis ++) {
        if (getRcDeflectionAbs(axis) * 100.f >= stickPercentLimit) {
            return true;
        }
    }
    return false;
}
#endif

#ifdef USE_RUNAWAY_TAKEOFF
// allow temporarily disabling runaway takeoff prevention if we are connected
// to the configurator and the ARMING_DISABLED_MSP flag is cleared.
void runawayTakeoffTemporaryDisable(uint8_t disableFlag)
{
    runawayTakeoffTemporarilyDisabled = disableFlag;
}
#endif

// returns negative values for reversed thrust in 3D mode
int8_t calculateThrottlePercent(void)
{
    const float throttle = constrainf(rcGetChannel(THROTTLE), NORMALISED_RANGE_MIN, NORMALISED_RANGE_MAX);
    float fraction = 0.0f;

    if (featureIsEnabled(FEATURE_3D)
        && !IS_RC_MODE_ACTIVE(BOX3D)
        && !flight3DConfig()->switched_mode3d) {

        const float midrc = rcUsToNorm(rxConfig()->midrc);
        const float deadband = rcUsSpanToNorm(flight3DConfig()->deadband3d_throttle);
        if (throttle > midrc + deadband) {
            fraction = (throttle - midrc - deadband) / (NORMALISED_RANGE_MAX - midrc - deadband);
        } else if (throttle < midrc - deadband) {
            fraction = (throttle - midrc + deadband) / (midrc - deadband - NORMALISED_RANGE_MIN);
        }
    } else {
        const float mincheck = rcUsToNorm(rxConfig()->mincheck);
        fraction = constrainf((throttle - mincheck) / (NORMALISED_RANGE_MAX - mincheck), 0.0f, 1.0f);
        if (featureIsEnabled(FEATURE_3D)
            && IS_RC_MODE_ACTIVE(BOX3D)
            && flight3DConfig()->switched_mode3d) {

            fraction = -fraction;  // 3D on a switch is active
        }
    }
    return fraction * 100;
}

uint8_t calculateThrottlePercentAbs(void)
{
    return abs(calculateThrottlePercent());
}

static bool throttleRaised = false;
bool wasThrottleRaised(void)
{
    return throttleRaised;
}

/*
 * processRx called from taskUpdateRxMain
 */
bool processRx(timeUs_t currentTimeUs)
{
    if (!calculateRxChannelsAndUpdateFailsafe(currentTimeUs)) {
        return false;
    }

    updateRcRefreshRate(currentTimeUs, isRxReceivingSignal());

    // in 3D mode, we need to be able to disarm by switch at any time
    if (featureIsEnabled(FEATURE_3D)) {
        if (!IS_RC_MODE_ACTIVE(BOXARM))
            disarm(DISARM_REASON_SWITCH);
    }

    updateRSSI(currentTimeUs);

    if (currentTimeUs > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
        failsafeStartMonitoring();
    }

    const bool throttleActive = calculateThrottleStatus() != THROTTLE_LOW;
    const uint8_t throttlePercent = calculateThrottlePercentAbs();
    const bool launchControlActive = isLaunchControlActive();
    static bool isAirmodeActive;

    if (ARMING_FLAG(ARMED)) {
        if (throttlePercent >= rxConfig()->airModeActivateThreshold) {
            throttleRaised = true; // Latch true until disarm
        }
        if (isAirmodeEnabled() && !launchControlActive) {
            isAirmodeActive = throttleRaised;
        }
    } else {
        throttleRaised = false;
        isAirmodeActive = false;
    }

    // Note: If Airmode is enabled, on arming, iTerm and PIDs will be off until throttle exceeds the threshold (OFF while disarmed)
    // If not, iTerm will be off at low throttle, with pidStabilisationState determining whether PIDs will be active
    // A LIFT holds the motors at the lift throttle whatever the stick says, so the
    // PIDs must stay on even when the pilot's throttle stick is at the bottom.
    if (ARMING_FLAG(ARMED) && (isAirmodeActive || throttleActive || launchControlActive || isLaunchControlLifting() || isFixedWing())) {
        // the wing launch holds iTerm off until spin-up, but must keep the rest
        // of the stabilisation running through the motor delay
        pidSetItermReset(launchWingHoldsIterm());
        pidStabilisationState(PID_STABILISATION_ON);
    } else {
        pidSetItermReset(true);
        pidStabilisationState(currentPidProfile->pidAtMinThrottle ? PID_STABILISATION_ON : PID_STABILISATION_OFF);
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
        && !crashFlipModeActive
        && !runawayTakeoffTemporarilyDisabled
        && !isFixedWing()) {

        // Determine if we're in "flight"
        //   - motors running
        //   - throttle over runaway_takeoff_deactivate_throttle_percent
        //   - sticks are active and have deflection greater than runaway_takeoff_deactivate_stick_percent
        //   - pidSum on all axis is less then runaway_takeoff_deactivate_pidlimit
        bool inStableFlight = false;
        if (!featureIsEnabled(FEATURE_MOTOR_STOP) || isAirmodeEnabled() || throttleActive) { // are motors running?
            const uint8_t lowThrottleLimit = pidConfig()->runaway_takeoff_deactivate_throttle;
            const uint8_t midThrottleLimit = constrain(lowThrottleLimit * 2, lowThrottleLimit * 2, RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT);
            if ((((throttlePercent >= lowThrottleLimit) && areSticksActive(RUNAWAY_TAKEOFF_DEACTIVATE_STICK_PERCENT)) || (throttlePercent >= midThrottleLimit))
                && (fabsf(pidData[FD_PITCH].Sum) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)
                && (fabsf(pidData[FD_ROLL].Sum) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)
                && (fabsf(pidData[FD_YAW].Sum) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)) {

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
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_FALSE);  //!< Deactivation Delay Running
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME, runawayTakeoffAccumulatedUs / 1000);  //!< Accumulated Trigger Time [unit:ms]
        } else {
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_TRUE);  //!< Deactivation Delay Running
            DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME, (cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs) + runawayTakeoffAccumulatedUs) / 1000);  //!< Accumulated Trigger Time [unit:ms]
        }
    } else {
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_FALSE);  //!< Deactivation Delay Running
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME, DEBUG_RUNAWAY_TAKEOFF_FALSE);   //!< Accumulated Trigger Time [unit:ms]
    }
#endif

#ifdef USE_LAUNCH_CONTROL
    if (ARMING_FLAG(ARMED)) {
        if (launchControlActive && (throttlePercent > currentPidProfile->launchControlThrottlePercent)) {
            // throttle limit trigger reached, launch triggered
            // reset the iterms as they may be at high values from holding the launch position
            // (in LIFT mode this means the pilot took off on the sticks: the lift is skipped)
            launchControlState = LAUNCH_CONTROL_TRIGGERED;
            pidResetIterm();
#ifdef USE_LAUNCH_CONTROL_LIFT
        } else if (launchControlActive && (currentPidProfile->launchControlMode == LAUNCH_CONTROL_MODE_LIFT)) {
            // LIFT is staged at arming like every launch control mode; LAUNCH TRIGGER sends it.
            // On one 3-position switch: LAUNCH CONTROL on mid+high, LAUNCH TRIGGER on high.
            const bool triggerOn = IS_RC_MODE_ACTIVE(BOXLAUNCHTRIGGER);
            if (!IS_RC_MODE_ACTIVE(BOXLAUNCHCONTROL)) {
                // switched to regular before launching: stand down for the rest of this arm
                launchControlState = LAUNCH_CONTROL_DISABLED;
            } else if (!triggerOn) {
                launchControlLiftTriggerSeenOff = true;
            } else if (launchControlLiftTriggerSeenOff) {
                // the trigger has just come on: this is the launch
                launchControlLiftTriggerSeenOff = false;  // consumed, whether or not it launches
                // only from acro, as for arming launch control: the flip-and-brake
                // recovery needs full rotation. And only near level: the lift self-levels,
                // but not from a steep slope, a stand at an angle, or upside down. And only
                // with every motor turning: a prop held by grass at start-up stays stopped
                // even at full throttle, and three motors tip it straight over. A refused
                // press is still used up, so fixing it afterwards can never send it.
                if (flightModeFlags == 0 && isLaunchControlLiftLevel() && launchControlLiftFindStoppedMotor() == 0) {
                    launchControlState = LAUNCH_CONTROL_LIFTING;
                    launchControlLiftStartUs = currentTimeUs;
                    launchControlLiftEndUs = 0;
                    launchControlLiftTroubleStartUs = 0;
                    launchControlLiftEndReason = LIFT_END_TIME;
                    launchControlLiftMotorFaultStartUs = 0;
                    launchControlLiftMotorStoppedStartUs = 0;
                    launchControlLiftMotorFiltersPrimed = false;
                    launchControlLiftPeakJerk = 0.0f;
                    launchControlLiftRan = true;
                    // A lift is a takeoff. When it hands over, the quad is flying on whatever the
                    // sticks say, exactly as if the pilot had flown it up: airmode and the I-term
                    // are live even with the throttle left at zero, and runaway takeoff (built for
                    // a quad still on the ground) can't disarm a flip-and-brake at the top.
                    throttleRaised = true;
#ifdef USE_RUNAWAY_TAKEOFF
                    runawayTakeoffCheckDisabled = true;
#endif
                    launchControlLiftImpactJerk = currentPidProfile->launchControlLiftImpact * 10.0f;
                    launchControlLiftTimeMs = currentPidProfile->launchControlLiftTime;
                    launchControlLiftThrottleFraction = currentPidProfile->launchControlLiftThrottle / 100.0f;
                    launchControlLiftBrakeFraction = MIN(currentPidProfile->launchControlLiftBrake, 100) / 100.0f;
                    launchControlLiftFlipDirection = currentPidProfile->launchControlLiftFlipDirection < LAUNCH_LIFT_FLIP_COUNT
                        ? currentPidProfile->launchControlLiftFlipDirection : LAUNCH_LIFT_FLIP_AUTO;
                    launchControlLiftFlipRate = constrain(currentPidProfile->launchControlLiftFlipRate,
                        LAUNCH_CONTROL_LIFT_FLIP_RATE_MIN_DPS, LAUNCH_CONTROL_LIFT_FLIP_RATE_MAX_DPS);
                    // a slow flip gets longer before it's taken to have failed
                    launchControlLiftFlipTimeoutUs = LAUNCH_CONTROL_LIFT_FLIP_TIMEOUT_MS * 1000
                        + (timeDelta_t)(2.0f * 180.0f / launchControlLiftFlipRate * 1e6f);
                    launchControlLiftBrakeTimeUs = constrain(currentPidProfile->launchControlLiftBrakeTime,
                        LAUNCH_CONTROL_LIFT_BRAKE_TIME_MIN_MS, LAUNCH_CONTROL_LIFT_BRAKE_TIME_MAX_MS) * 1000;
                    launchControlLiftMotorThrottle = launchControlLiftThrottleFraction;
                    launchControlLiftPhase = LIFT_PHASE_CLIMB;
                    launchControlLiftPhaseStartUs = currentTimeUs;
                    launchControlLiftClimbSpeed = 0.0f;
                    launchControlLiftHeight = 0.0f;
                    launchControlLiftTop = 0.0f;
                    launchControlLiftRising = false;
                    launchControlLiftTopUs = 0;
                    launchControlLiftLastUpdateUs = currentTimeUs;
                    pidResetIterm();
                }
            }
#endif // USE_LAUNCH_CONTROL_LIFT
        }
    } else {
#ifdef USE_LAUNCH_CONTROL_LIFT
        if (launchControlState == LAUNCH_CONTROL_LIFTING) {
            // disarmed mid-lift: the launch has been used, exactly as if it had completed
            launchControlState = LAUNCH_CONTROL_TRIGGERED;
            launchControlLiftEndReason = LIFT_END_DISARM;
            launchControlLiftPhase = LIFT_PHASE_CLIMB;
        }
#endif
        if (launchControlState == LAUNCH_CONTROL_TRIGGERED) {
            // If trigger mode is MULTIPLE then reset the state when disarmed
            // and the mode switch is turned off.
            // For trigger mode SINGLE we never reset the state and only a single
            // launch is allowed until a reboot.
            if (currentPidProfile->launchControlAllowTriggerReset && !IS_RC_MODE_ACTIVE(BOXLAUNCHCONTROL)) {
                launchControlState = LAUNCH_CONTROL_DISABLED;
            }
        } else {
            launchControlState = LAUNCH_CONTROL_DISABLED;
        }
#ifdef USE_LAUNCH_CONTROL_LIFT
        // the OSD's "how high" is for the flight it happened in
        launchControlLiftRising = false;
        launchControlLiftTopUs = 0;
#endif
    }
    // the OSD's PREP: disarmed, and launch control will engage on arming
    launchControlPreStaged = !ARMING_FLAG(ARMED)
        && launchControlState == LAUNCH_CONTROL_DISABLED
        && canUseLaunchControl();
#ifdef USE_LAUNCH_CONTROL_LIFT
    updateLaunchControlLiftStatus();
#endif
#endif

    return true;
}

void processRxModes(timeUs_t currentTimeUs)
{
    static bool armedBeeperOn = false;
#ifdef USE_TELEMETRY
    static bool sharedPortTelemetryEnabled = false;
#endif
    const throttleStatus_e throttleStatus = calculateThrottleStatus();

    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    const timeUs_t autoDisarmDelayUs = armingConfig()->auto_disarm_delay * 1e6f;
    if (ARMING_FLAG(ARMED)
        && featureIsEnabled(FEATURE_MOTOR_STOP) // enable auto-disarm if motor_stop is enabled
        && !isFixedWing()
        && !featureIsEnabled(FEATURE_3D)
        && !isAirmodeEnabled()
        && !FLIGHT_MODE(GPS_RESCUE_MODE)  // disable auto-disarm when GPS Rescue is active
        && !flightPlanNavIsRescuePlanActive()  // ... or a plan rescue / fallback descent is flying
        && !flightPlanNavIsRescueDescentActive()
    ) {
        if (isUsingSticksForArming()) {
            if (throttleStatus == THROTTLE_LOW) {
                if ((autoDisarmDelayUs > 0) && (currentTimeUs > disarmAt)) {
                    // auto-disarm configured and delay is over
                    disarm(DISARM_REASON_THROTTLE_TIMEOUT);
                    armedBeeperOn = false;
                } else {
                    // still armed; do warning beeps while armed
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = true;
                }
            } else {
                // throttle is not low - extend disarm time
                disarmAt = currentTimeUs + autoDisarmDelayUs;

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
    } else {
        disarmAt = currentTimeUs + autoDisarmDelayUs;  // extend auto-disarm timer
    }

    if (!(IS_RC_MODE_ACTIVE(BOXPARALYZE) && !ARMING_FLAG(ARMED))
#ifdef USE_CMS
        && !cmsInMenu
#endif
        ) {
        processRcStickPositions();
    }

    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        updateInflightCalibrationState();
    }

    updateActivatedModes();

#ifdef USE_DSHOT
    if (crashFlipModeActive) {
        beeper(BEEPER_CRASHFLIP_MODE);
    }
#endif

    if (!cliMode && !(IS_RC_MODE_ACTIVE(BOXPARALYZE) && !ARMING_FLAG(ARMED))) {
        processRcAdjustments(currentControlRateProfile);
    }

    bool canUseHorizonMode = true;
    if ((IS_RC_MODE_ACTIVE(BOXANGLE)
        || failsafeIsActive()
#ifdef USE_ALTITUDE_HOLD
        || FLIGHT_MODE(ALT_HOLD_MODE)
#endif
#ifdef USE_POSITION_HOLD
        || FLIGHT_MODE(POS_HOLD_MODE)
#endif
#if defined(USE_WING) && defined(USE_LAUNCH_WING)
        || FLIGHT_MODE(LAUNCH_MODE)
#endif
        ) && (sensors(SENSOR_ACC))) {
        // bumpless transfer to Level mode
        canUseHorizonMode = false;

        if (!FLIGHT_MODE(ANGLE_MODE)) {
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ANGLE_MODE); // failsafe support
    }

#if defined(USE_GPS_RESCUE) && !ENABLE_RESCUE_PLAN
    // Legacy: the pilot's switch and the failsafe procedure both fly the legacy
    // GPS_RESCUE_MODE controller. (With ENABLE_RESCUE_PLAN both are flown as an
    // autopilot rescue mission instead - staged below and in failsafe.c.)
    if (ARMING_FLAG(ARMED) && gpsRescueIsConfigured()
        && (IS_RC_MODE_ACTIVE(BOXGPSRESCUE)
        || (failsafeIsActive() && failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE))) {
        if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
            ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
    }
#endif

#if ENABLE_FLIGHT_PLAN
#ifndef USE_WING
    // Waypoint capture runs whether or not the mission is engaged - the whole
    // point is marking waypoints while flying around before engaging. The
    // channel-validity guard keeps rxfail aux substitution from ghost-editing
    // the plan during signal loss.
    flightPlanCaptureUpdate(currentTimeUs,
                            IS_RC_MODE_ACTIVE(BOXWPCAPTURE),
                            rxAreFlightChannelsValid());
#endif

    // During failsafe the mission flies only while the failsafe state machine
    // has chosen to (rx-loss policy). In the stage-1 window before failsafe
    // activates, rxfail substitution already drives the aux channels, so the
    // switch is only pilot intent while channel data is real — otherwise hold
    // the current mode and let the policy decide at stage-2 entry.
    // A BOXGPSRESCUE switch flies the rescue as an autopilot mission: stage the
    // rescue plan once on activation (it replaces any mission already flying),
    // then request the autopilot to fly it. Only requests once the plan actually
    // stages (needs home + GPS fix), so a switch without a fix can't engage a
    // stale mission.
    // A held BOXGPSRESCUE switch that cannot stage a plan (no home/fix), or whose
    // plan aborts, degrades to an altitude-only descent rather than doing nothing
    // or hovering indefinitely - legacy emergency-descent parity. Failsafe owns
    // its own degrade path (failsafe.c), so this is switch-only.
    bool rescueSwitchRequest = false;
#if defined(USE_GPS_RESCUE) && ENABLE_RESCUE_PLAN
    bool rescueDescentRequest = false;
    static bool rescueSwitchStaged = false;
    static bool rescueSwitchDescending = false;
    if (ARMING_FLAG(ARMED) && IS_RC_MODE_ACTIVE(BOXGPSRESCUE) && !failsafeIsActive()) {
        if (!rescueSwitchStaged) {
            rescueSwitchStaged = flightPlanNavStageRescuePlan();
        }
        // Latch the descent once the staged plan aborts: re-engaging would only
        // re-fly into the same abort. A staging failure (no fix) is not latched -
        // if a fix returns, staging succeeds next cycle and the real plan flies.
        if (rescueSwitchStaged && flightPlanNavGetState() == FP_NAV_ABORTED) {
            rescueSwitchDescending = true;
        }
        if (rescueSwitchDescending || !rescueSwitchStaged) {
            rescueDescentRequest = true;
        } else {
            rescueSwitchRequest = true;
        }
    } else {
        rescueSwitchStaged = false;
        rescueSwitchDescending = false;
    }
    flightPlanNavRescueDescent(rescueDescentRequest, currentTimeUs);
#endif

    bool autopilotRequested;
    if (failsafeIsActive()) {
        autopilotRequested = (failsafePhase() == FAILSAFE_AUTOPILOT);
    } else if (rxAreFlightChannelsValid()) {
        autopilotRequested = (IS_RC_MODE_ACTIVE(BOXAUTOPILOT) && !launchWingHoldWaits()) || rescueSwitchRequest;
    } else {
        autopilotRequested = FLIGHT_MODE(AUTOPILOT_MODE);
    }

    // Before the hold modes: AUTOPILOT_MODE activates both, so they must see
    // its state from the same cycle.
    if (ARMING_FLAG(ARMED)
        // GPS Rescue has priority over the mission
        && !FLIGHT_MODE(GPS_RESCUE_MODE)
        && !launchWingOwnsAircraft()
        && !launchWingAwaitingThrow()
        && autopilotRequested
        && sensors(SENSOR_ACC)
        && ((sensors(SENSOR_GPS) && STATE(GPS_FIX)) || missionKeepsWithoutFix())
        // waypoints carry altitude; without altitude data the mission must not run
        && isAltitudeAvailable()
        && wasThrottleRaised()) {
        if (!FLIGHT_MODE(AUTOPILOT_MODE)) {
            ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
            flightPlanNavEngage();
        }
        flightPlanNavUpdate(currentTimeUs);
    } else {
        if (FLIGHT_MODE(AUTOPILOT_MODE)) {
            DISABLE_FLIGHT_MODE(AUTOPILOT_MODE);
            flightPlanNavDisengage();
        }
    }
#endif

#ifdef USE_OSD_NAV_MAP
    // record the flown trail for the OSD minimap; internally rate-limited and
    // needs only the GPS solution, home and the ARMED state
    navTrailUpdate(currentTimeUs);
#endif

#if defined(USE_WING) && defined(USE_LAUNCH_WING)
    // The latch is the mid-air guard: it is set only at the arm transition, so
    // the box alone can never engage a launch in flight. A failsafe ends the
    // launch through the launch itself, which hands an airborne aircraft over.
    if (ARMING_FLAG(ARMED)
        && launchWingLatched()
        && IS_RC_MODE_ACTIVE(BOXLAUNCH)
        && sensors(SENSOR_ACC)
        && !launchWingIsTerminal()) {
        if (!FLIGHT_MODE(LAUNCH_MODE)) {
            ENABLE_FLIGHT_MODE(LAUNCH_MODE);
        }
    } else if (FLIGHT_MODE(LAUNCH_MODE)) {
        // Cancelling before the throw would drop the launch's hold on the motor
        // and hand the raised throttle stick straight to it, with the aircraft
        // still in the pilot's hands. Disarm instead, and make them cycle the
        // arm switch. Only the box going inactive is treated this way; failsafe
        // and the terminal states reach here too and keep their own handling.
        const bool cancelledBeforeTheThrow = !IS_RC_MODE_ACTIVE(BOXLAUNCH)
            && launchWingIsPreLaunch()
            && ARMING_FLAG(ARMED);
        DISABLE_FLIGHT_MODE(LAUNCH_MODE);
        launchWingSwitchOff();
        if (cancelledBeforeTheThrow) {
            // Stick arming needs throttle low to re-arm, so it cannot repeat
            // this, and nothing on that path would ever clear the interlock.
            if (!isUsingSticksForArming()) {
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            }
            disarm(DISARM_REASON_LAUNCH_ABORT);
        }
    }
#endif

#ifdef USE_ALTITUDE_HOLD
    // only if armed; can coexist with position hold
    if (ARMING_FLAG(ARMED)
        // and not in GPS_RESCUE_MODE, to give it priority over Altitude Hold
        && !FLIGHT_MODE(GPS_RESCUE_MODE)
        && !launchOwnsAircraft()
        // nor with the aircraft still in the hand
        && !launchWingAwaitingThrow()
        // and either the alt_hold switch is activated, or are in failsafe landing mode,
        // or an autopilot mission needs altitude control, or a switch-rescue fallback descent
        && ((altHoldSelected() && !launchWingHoldWaits()) || failsafeIsActive() || FLIGHT_MODE(AUTOPILOT_MODE) || flightPlanNavIsRescueDescentActive())
        // and we have Acc for self-levelling
        && sensors(SENSOR_ACC)
        // and this platform actually has an altitude control law
        && autopilotAltitudeControlAvailable()
        // and we have altitude data
        && isAltitudeAvailable()
        // but not until throttle is raised
        && wasThrottleRaised()) {
        if (!FLIGHT_MODE(ALT_HOLD_MODE)) {
            ENABLE_FLIGHT_MODE(ALT_HOLD_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ALT_HOLD_MODE);
    }
#endif

#ifdef USE_POSITION_HOLD
    // only if armed; can coexist with altitude hold
    if (ARMING_FLAG(ARMED)
        // and not in GPS_RESCUE_MODE, to give it priority over Position Hold
        && !FLIGHT_MODE(GPS_RESCUE_MODE)
        && !launchOwnsAircraft()
        // and either the pos_hold switch is activated, or are in failsafe landing mode,
        // or an autopilot mission needs the position controller
        && ((IS_RC_MODE_ACTIVE(BOXPOSHOLD) && !launchWingHoldWaits()) || failsafeIsActive() || FLIGHT_MODE(AUTOPILOT_MODE)
            || rescueDescentCircles())
        // and altitude hold is flying, on a platform that needs it to hold a position
        && posHoldHasAltHold()
        // and we have Acc for self-levelling
        && sensors(SENSOR_ACC)
        // and this platform actually has a position control law
        && autopilotPositionControlAvailable()
        // but not until throttle is raised
        && wasThrottleRaised()) {
        if (!FLIGHT_MODE(POS_HOLD_MODE)) {
            ENABLE_FLIGHT_MODE(POS_HOLD_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(POS_HOLD_MODE);
    }
#endif

    if (IS_RC_MODE_ACTIVE(BOXHORIZON) && canUseHorizonMode && sensors(SENSOR_ACC)) {
        DISABLE_FLIGHT_MODE(ANGLE_MODE);
        if (!FLIGHT_MODE(HORIZON_MODE)) {
            ENABLE_FLIGHT_MODE(HORIZON_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(HORIZON_MODE);
    }

#ifdef USE_CHIRP
    if (IS_RC_MODE_ACTIVE(BOXCHIRP) && !FLIGHT_MODE(FAILSAFE_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
        if (!FLIGHT_MODE(CHIRP_MODE)) {
            ENABLE_FLIGHT_MODE(CHIRP_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(CHIRP_MODE);
    }
#endif

    if (FLIGHT_MODE(ANGLE_MODE | ALT_HOLD_MODE | POS_HOLD_MODE | HORIZON_MODE)) {
        LED1_ON;
        // increase frequency of attitude task to reduce drift when in angle or horizon mode
        rescheduleTask(TASK_ATTITUDE, TASK_PERIOD_HZ(acc.sampleRateHz / (float)imuConfig()->imu_process_denom));
    } else {
        LED1_OFF;
        rescheduleTask(TASK_ATTITUDE, TASK_PERIOD_HZ(100));
    }

    if (!IS_RC_MODE_ACTIVE(BOXPREARM) && ARMING_FLAG(WAS_ARMED_WITH_PREARM)) {
        DISABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
    }

#if defined(USE_ACC) || defined(USE_MAG)
    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        if (IS_RC_MODE_ACTIVE(BOXHEADFREE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
            if (!FLIGHT_MODE(HEADFREE_MODE)) {
                ENABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
        if (IS_RC_MODE_ACTIVE(BOXHEADADJ) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
            if (imuQuaternionHeadfreeOffsetSet()) {
               beeper(BEEPER_RX_SET);
            }
        }
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
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        bool enableSharedPortTelemetry = (!isModeActivationConditionPresent(BOXTELEMETRY) && ARMING_FLAG(ARMED)) || (isModeActivationConditionPresent(BOXTELEMETRY) && IS_RC_MODE_ACTIVE(BOXTELEMETRY));
        if (enableSharedPortTelemetry && !sharedPortTelemetryEnabled) {
            mspSerialReleaseSharedTelemetryPorts();
            telemetryCheckState();

            sharedPortTelemetryEnabled = true;
        } else if (!enableSharedPortTelemetry && sharedPortTelemetryEnabled) {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspSerialAllocatePorts();

            sharedPortTelemetryEnabled = false;
        }
    }
#endif

#ifdef USE_VTX_CONTROL
    vtxUpdateActivatedChannel();

    if (canUpdateVTX()) {
        handleVTXControlButton();
    }
#endif

#ifdef USE_ACRO_TRAINER
    pidSetAcroTrainerState(IS_RC_MODE_ACTIVE(BOXACROTRAINER) && sensors(SENSOR_ACC));
#endif // USE_ACRO_TRAINER

    pidSetAntiGravityState(IS_RC_MODE_ACTIVE(BOXANTIGRAVITY) || featureIsEnabled(FEATURE_ANTI_GRAVITY));
}

static FAST_CODE_NOINLINE_CRITICAL void subTaskPidController(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_PIDLOOP) {startTime = micros();}
    // PID - note this is function pointer set by setPIDController()
    pidController(currentPidProfile, currentTimeUs);
    DEBUG_SET(DEBUG_PIDLOOP, 1, micros() - startTime);  //!< PID Controller Time [unit:us]

#ifdef USE_RUNAWAY_TAKEOFF
    // Check to see if runaway takeoff detection is active (anti-taz), the pidSum is over the threshold,
    // and gyro rate for any axis is above the limit for at least the activate delay period.
    // If so, disarm for safety
    if (ARMING_FLAG(ARMED)
        && !isFixedWing()
        && pidConfig()->runaway_takeoff_prevention
        && !runawayTakeoffCheckDisabled
        && !crashFlipModeActive
        && !runawayTakeoffTemporarilyDisabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable Runaway Takeoff triggering if GPS Rescue is active
        // Same for a LIFT: the flight controller is flying, the sticks are ignored, and the
        // stick throttle stays low, so the deactivation check below can never clear it.
        && !isLaunchControlLifting()
        // check that motors are running
        && (!featureIsEnabled(FEATURE_MOTOR_STOP) || isAirmodeEnabled() || (calculateThrottleStatus() != THROTTLE_LOW))) {

        if (((fabsf(pidData[FD_PITCH].Sum) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD)
            || (fabsf(pidData[FD_ROLL].Sum) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD)
            || (fabsf(pidData[FD_YAW].Sum) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD))
            && ((gyroAbsRateDps(FD_PITCH) > RUNAWAY_TAKEOFF_GYRO_LIMIT_RP)
                || (gyroAbsRateDps(FD_ROLL) > RUNAWAY_TAKEOFF_GYRO_LIMIT_RP)
                || (gyroAbsRateDps(FD_YAW) > RUNAWAY_TAKEOFF_GYRO_LIMIT_YAW))) {

            if (runawayTakeoffTriggerUs == 0) {
                runawayTakeoffTriggerUs = currentTimeUs + RUNAWAY_TAKEOFF_ACTIVATE_DELAY;
            } else if (currentTimeUs > runawayTakeoffTriggerUs) {
                setArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
                disarm(DISARM_REASON_RUNAWAY_TAKEOFF);
            }
        } else {
            runawayTakeoffTriggerUs = 0;
        }
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ENABLED_STATE, DEBUG_RUNAWAY_TAKEOFF_TRUE);  //!< Runaway Takeoff Enabled
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ACTIVATING_DELAY, runawayTakeoffTriggerUs == 0 ? DEBUG_RUNAWAY_TAKEOFF_FALSE : DEBUG_RUNAWAY_TAKEOFF_TRUE);  //!< Activation Delay Running
    } else {
        runawayTakeoffTriggerUs = 0;
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ENABLED_STATE, DEBUG_RUNAWAY_TAKEOFF_FALSE);     //!< Runaway Takeoff Enabled
        DEBUG_SET(DEBUG_RUNAWAY_TAKEOFF, DEBUG_RUNAWAY_TAKEOFF_ACTIVATING_DELAY, DEBUG_RUNAWAY_TAKEOFF_FALSE);  //!< Activation Delay Running
    }
#endif

#ifdef USE_PID_AUDIO
    if (isModeActivationConditionPresent(BOXPIDAUDIO)) {
        pidAudioUpdate();
    }
#endif
}

static FAST_CODE_NOINLINE void subTaskPidSubprocesses(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_PIDLOOP) {
        startTime = micros();
    }

#ifdef USE_BLACKBOX
    if (!cliMode && blackboxConfig()->device) {
        blackboxUpdate(currentTimeUs);
    }
#else
    UNUSED(currentTimeUs);
#endif

    DEBUG_SET(DEBUG_PIDLOOP, 3, micros() - startTime);  //!< PID Subprocess Time [unit:us]
}

#ifdef USE_TELEMETRY
#define GYRO_TEMP_READ_DELAY_US (3 * 1000 * 1000)    // Only read the gyro temp every 3 seconds
void subTaskTelemetryPollSensors(timeUs_t currentTimeUs)
{
    static timeUs_t lastGyroTempTimeUs = 0;

    if (cmpTimeUs(currentTimeUs, lastGyroTempTimeUs) >= GYRO_TEMP_READ_DELAY_US) {
        // Read out gyro temperature if used for telemmetry
        gyroReadTemperature();
        lastGyroTempTimeUs = currentTimeUs;
    }
}
#endif

static FAST_CODE void subTaskMotorUpdate(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_CYCLETIME) {
        startTime = micros();
        static uint32_t previousMotorUpdateTime;
        const uint32_t currentDeltaTime = startTime - previousMotorUpdateTime;
        DEBUG_SET(DEBUG_CYCLETIME, 2, currentDeltaTime);                      //!< Motor Update Interval [unit:us]
        DEBUG_SET(DEBUG_CYCLETIME, 3, currentDeltaTime - targetPidLooptime);  //!< Motor Update Interval Error [unit:us]
        previousMotorUpdateTime = startTime;
    } else if (debugMode == DEBUG_PIDLOOP) {
        startTime = micros();
    }

    mixTable(currentTimeUs);

#ifdef USE_SERVOS
    // motor outputs are used as sources for servo mixing, so motors must be calculated using mixTable() before servos.
    if (isMixerUsingServos()) {
        writeServos();
    }
#endif

    writeMotors();

#ifdef USE_DSHOT_TELEMETRY_STATS
    if (debugMode == DEBUG_DSHOT_RPM_ERRORS && useDshotTelemetry) {
        const uint8_t motorCount = MIN(getMotorCount(), 4);
        for (uint8_t i = 0; i < motorCount; i++) {
            DEBUG_SET(DEBUG_DSHOT_RPM_ERRORS, i, getDshotTelemetryMotorInvalidPercent(i));  //!< [index:0..3] Motor {1|2|3|4} Invalid [unit:%]
        }
    }
#endif

    DEBUG_SET(DEBUG_PIDLOOP, 2, micros() - startTime);  //!< Motor Update Time [unit:us]
}

static FAST_CODE_NOINLINE void subTaskRcCommand(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if (isUsingSticksForArming() && rcGetChannel(THROTTLE) <= rcUsToNorm(rxConfig()->mincheck)
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

    processRcCommand();
}

FAST_CODE void taskGyroSample(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    gyroUpdate();
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        pidUpdateCounter = 0;
    }
    pidUpdateCounter++;
}

FAST_CODE bool gyroFilterReady(void)
{
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        return true;
    } else {
        return false;
    }
}

FAST_CODE bool pidLoopReady(void)
{
    if ((pidUpdateCounter % activePidLoopDenom) == (activePidLoopDenom / 2)) {
        return true;
    }
    return false;
}

FAST_CODE void taskFiltering(timeUs_t currentTimeUs)
{
#ifdef USE_DSHOT_TELEMETRY
    updateDshotTelemetry();  // decode and update Dshot telemetry
#endif
    gyroFiltering(currentTimeUs);
}

// Function for loop trigger
FAST_CODE void taskMainPidLoop(timeUs_t currentTimeUs)
{

#if ENABLE_SIMULATOR_GYROPID_SYNC
    if (lockMainPID() != 0) return;
#endif

    // DEBUG_PIDLOOP, timings for:
    // 0 - gyroUpdate()
    // 1 - subTaskPidController()
    // 2 - subTaskMotorUpdate()
    // 3 - subTaskPidSubprocesses()
    DEBUG_SET(DEBUG_PIDLOOP, 0, micros() - currentTimeUs);  //!< Gyro Update Time [unit:us]

    subTaskRcCommand(currentTimeUs);
#ifdef USE_LAUNCH_CONTROL_LIFT
    // Runs in the PID loop, not the RX task, so the lift ends on time even if RX frames stop.
    launchControlLiftUpdate(currentTimeUs);
#endif
#if defined(USE_WING) && defined(USE_LAUNCH_WING)
    // After rcCommand so the abort test and the hand-back blend see fresh stick
    // data, and before the PID controller so it reads this cycle's angle target.
    launchWingUpdate(currentTimeUs);
#endif
    subTaskPidController(currentTimeUs);
    subTaskMotorUpdate(currentTimeUs);
    subTaskPidSubprocesses(currentTimeUs);

    DEBUG_SET(DEBUG_CYCLETIME, 0, getTaskDeltaTimeUs(TASK_SELF));  //!< Cycle Time [unit:us]
    DEBUG_SET(DEBUG_CYCLETIME, 1, getAverageSystemLoadPercent());  //!< CPU Load [unit:%]
}

bool isCrashFlipModeActive(void)
{
    return crashFlipModeActive;
}

timeUs_t getLastDisarmTimeUs(void)
{
    return lastDisarmTimeUs;
}

bool isTryingToArm(void)
{
    return (tryingToArm != ARMING_DELAYED_DISARMED);
}

void resetTryingToArm(void)
{
    tryingToArm = ARMING_DELAYED_DISARMED;
}

bool isLaunchControlActive(void)
{
#ifdef USE_LAUNCH_CONTROL
    return launchControlState == LAUNCH_CONTROL_ACTIVE;
#else
    return false;
#endif
}

// LIFT mode: true from the trigger until the lift hands control back to the sticks
bool isLaunchControlLifting(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return launchControlState == LAUNCH_CONTROL_LIFTING && ARMING_FLAG(ARMED);
#else
    return false;
#endif
}

// LIFT is over and the flight controller is stopping the climb: rolling upside down,
// then braking (launch_lift_brake). The sticks are still ignored.
bool isLaunchControlLiftBraking(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return isLaunchControlLifting() && launchControlLiftPhase != LIFT_PHASE_CLIMB;
#else
    return false;
#endif
}

// Rate setpoint [deg/s] for the flip and brake, in place of the sticks
float getLaunchControlLiftRate(int axis)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return launchControlLiftRate[axis];
#else
    UNUSED(axis);
    return 0.0f;
#endif
}

// Vertical speed since the trigger [m/s], from the accelerometer
float getLaunchControlLiftClimbSpeed(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return launchControlLiftClimbSpeed;
#else
    return 0.0f;
#endif
}

// Motor throttle (0..1) the mixer uses in place of the stick while lifting
float getLaunchControlLiftThrottle(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return launchControlLiftMotorThrottle;
#else
    return 0.0f;
#endif
}

// Time left before the sticks take over, for the OSD countdown
uint32_t getLaunchControlLiftRemainingMs(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    if (!isLaunchControlLifting() || launchControlLiftPhase != LIFT_PHASE_CLIMB) {
        return 0;
    }
    const timeDelta_t elapsedUs = cmpTimeUs(micros(), launchControlLiftStartUs);
    const timeDelta_t remainingUs = (timeDelta_t)launchControlLiftTimeMs * 1000 - MAX(elapsedUs, 0);
    return remainingUs > 0 ? (uint32_t)remainingUs / 1000 : 0;
#else
    return 0;
#endif
}

// LIFT staged with its trigger already on: it has to go off before it can launch
bool isLaunchControlLiftAwaitingTriggerOff(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return launchControlLiftAwaitingTrigger;
#else
    return false;
#endif
}

// Armed and holding for a LIFT (as opposed to one of the other launch control modes)
bool isLaunchControlLiftStaged(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return launchControlLiftStaged;
#else
    return false;
#endif
}

#ifdef USE_LAUNCH_CONTROL_LIFT
// Works out what the OSD should say, in the RX task where currentPidProfile is valid.
// Everything the display asks for afterwards is a plain read of these variables.
static void updateLaunchControlLiftStatus(void)
{
    const bool liftMode = currentPidProfile->launchControlMode == LAUNCH_CONTROL_MODE_LIFT;
    launchControlLiftStaged = liftMode && launchControlState == LAUNCH_CONTROL_ACTIVE;
    launchControlLiftAwaitingTrigger = launchControlLiftStaged && !launchControlLiftTriggerSeenOff;
    launchControlLiftNotLevel = liftMode && !isLaunchControlLiftLevel();
    launchControlLiftStoppedMotor = launchControlLiftStaged ? launchControlLiftFindStoppedMotor() : 0;

    launchControlLiftPreArmMessage = NULL;
    if (!ARMING_FLAG(ARMED) && liftMode && IS_RC_MODE_ACTIVE(BOXLAUNCHCONTROL)) {
        if (launchControlState == LAUNCH_CONTROL_TRIGGERED && !launchControlLiftRan) {
            launchControlLiftPreArmMessage = currentPidProfile->launchControlAllowTriggerReset
                ? "LIFT USED: SWITCH OFF" : "LIFT USED: REBOOT";
        } else if (launchControlState == LAUNCH_CONTROL_TRIGGERED) {
            // the peak jerk of the lift, in launch_lift_impact units: set it a little above
            // what a clean lift reads, and a hit ends the lift while a clean one never does
            const unsigned peak = (unsigned)(launchControlLiftPeakJerk / 10.0f + 0.999f);   // rounded up
            tfp_sprintf(launchControlLiftUsedMessage, "LIFT USED PEAK %u%s: %s",
                MIN(peak, (unsigned)LAUNCH_CONTROL_LIFT_IMPACT_MAX),
                peak > (unsigned)LAUNCH_CONTROL_LIFT_IMPACT_MAX ? "+" : "",
                currentPidProfile->launchControlAllowTriggerReset ? "SWITCH OFF" : "REBOOT");
            launchControlLiftPreArmMessage = launchControlLiftUsedMessage;
        } else if (!isModeActivationConditionPresent(BOXLAUNCHTRIGGER)) {
            launchControlLiftPreArmMessage = "LIFT: NO TRIGGER MODE";
        } else if (flightModeFlags) {
            launchControlLiftPreArmMessage = "LIFT: ACRO ONLY";
        } else if (!canUseLaunchControl()) {
            launchControlLiftPreArmMessage = "LIFT: UNAVAILABLE";
        } else if (IS_RC_MODE_ACTIVE(BOXLAUNCHTRIGGER)) {
            launchControlLiftPreArmMessage = "LIFT: TRIGGER OFF FIRST";
#ifdef USE_ACC
        } else if (!sensors(SENSOR_ACC)) {
            launchControlLiftPreArmMessage = "LIFT: NEEDS ACC";
#endif
        } else if (launchControlLiftNotLevel) {
            launchControlLiftPreArmMessage = "LIFT: NOT LEVEL";
        } else {
            launchControlLiftPreArmMessage = "LIFT PRE-STAGE";
        }
    }
}
#endif // USE_LAUNCH_CONTROL_LIFT

// Disarmed with the Launch Control switch on, and launch control will engage on arming
// (the flight mode display shows LNCH, as it does once armed and holding)
bool isLaunchControlPreStaged(void)
{
#ifdef USE_LAUNCH_CONTROL
    return launchControlPreStaged;
#else
    return false;
#endif
}

// The LIFT pre-arm checklist for the OSD: disarmed with the Launch Control switch on,
// what will arming do? NULL when not in LIFT mode or the switch is off.
const char *getLaunchControlLiftPreArmMessage(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return launchControlLiftPreArmMessage;
#else
    return NULL;
#endif
}

// True for a moment after a lift ends while still armed, however it ended (failsafe too).
// The PID loop keeps landing disarm and crash detection out of this second: what the
// flight controller was doing in the lift must not read as a landing or a crash.
bool isLaunchControlLiftJustEnded(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    if (launchControlLiftEndUs == 0
        || launchControlState != LAUNCH_CONTROL_TRIGGERED
        || !ARMING_FLAG(ARMED)) {
        return false;
    }
    // cmpTimeUs is signed: without the lower bound the notice would return after the
    // timer wraps, and cover the warnings that come after it
    const timeDelta_t sinceHandoverUs = cmpTimeUs(micros(), launchControlLiftEndUs);
    return sinceHandoverUs >= 0 && sinceHandoverUs < LAUNCH_CONTROL_LIFT_HANDOVER_NOTICE_MS * 1000;
#else
    return false;
#endif
}

// True for a moment after a lift hands over, so the pilot sees the sticks are live.
// Not in a failsafe: failsafe is flying the quad then, not the sticks.
bool isLaunchControlLiftHandoverRecent(void)
{
    return isLaunchControlLiftJustEnded() && !failsafeIsActive();
}

// The last lift was ended early because it couldn't hold level
bool isLaunchControlLiftAborted(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    switch (launchControlLiftEndReason) {
    case LIFT_END_CONTROL:
    case LIFT_END_FLIP:
    case LIFT_END_UPSET:
    case LIFT_END_MOTOR:
    case LIFT_END_IMPACT:
        return true;
    default:
        return false;
    }
#else
    return false;
#endif
}

// The last lift was ended by an impact (launch_lift_impact)
bool isLaunchControlLiftImpacted(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return launchControlLiftEndReason == LIFT_END_IMPACT;
#else
    return false;
#endif
}

// LIFT is selected but the quad is too tilted (or has no attitude) to send
bool isLaunchControlLiftNotLevel(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return launchControlLiftNotLevel;
#else
    return false;
#endif
}

// How high the lift has gone, for the OSD [cm], from the accelerometer: live while it climbs and
// coasts, then the top of the lift (*atTop set) for a while after. -1 when there's nothing to show.
int32_t getLaunchControlLiftHeightCm(bool *atTop)
{
    *atTop = false;
#if defined(USE_LAUNCH_CONTROL_LIFT) && defined(USE_ACC)
    if (!ARMING_FLAG(ARMED) || !sensors(SENSOR_ACC)) {
        return -1;
    }
    if (launchControlState == LAUNCH_CONTROL_LIFTING || launchControlLiftRising) {
        return lrintf(MAX(launchControlLiftHeight, 0.0f) * 100.0f);
    }
    if (launchControlLiftTopUs) {
        // cmpTimeUs is signed: the lower bound keeps it from coming back when the timer wraps
        const timeDelta_t sinceTopUs = cmpTimeUs(micros(), launchControlLiftTopUs);
        if (sinceTopUs >= 0 && sinceTopUs < LAUNCH_CONTROL_LIFT_TOP_NOTICE_MS * 1000) {
            *atTop = true;
            return lrintf(launchControlLiftTop * 100.0f);
        }
    }
#endif
    return -1;
}

// Staged for a LIFT with a motor that isn't turning: its number (1-based), or 0
uint8_t getLaunchControlLiftStoppedMotor(void)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    return launchControlLiftStoppedMotor;
#else
    return 0;
#endif
}


#if defined(USE_LAUNCH_CONTROL_LIFT) && defined(USE_ACC)
// Specific force along earth-up [g]: 1 sitting still, more while thrust is speeding the
// climb up, less (below zero) while braking upside down. Earth-up in the body frame is
// row NWU_U of rMat.
static float launchControlLiftVerticalG(void)
{
    return (rMat.m[NWU_U][X] * acc.accADC.x + rMat.m[NWU_U][Y] * acc.accADC.y + rMat.m[NWU_U][Z] * acc.accADC.z)
        * acc.dev.acc_1G_rec;
}

// Steers the quad upside down: rotates about the axis that turns earth-up (u, in the body
// frame) onto body down, at a rate proportional to the angle left, so it rolls over fast
// and settles. Returns that angle [deg].
static float launchControlLiftSteerInverted(void)
{
    const float ux = rMat.m[NWU_U][X];
    const float uy = rMat.m[NWU_U][Y];
    const float uz = rMat.m[NWU_U][Z];
    const float angleToGoDeg = RADIANS_TO_DEGREES(acos_approx(constrainf(-uz, -1.0f, 1.0f)));

    // the axis (target x u) = (uy, -ux, 0): no yaw, roll and pitch only
    float axisRoll = uy;
    float axisPitch = -ux;
    float axisNorm = sqrtf(sq(axisRoll) + sq(axisPitch));
    bool forced = false;
    if (uz > 0.0f && launchControlLiftFlipDirection != LAUNCH_LIFT_FLIP_AUTO) {
        // the first half of the turn goes the way it was told, whatever lean it started
        // with; past sideways the same steering as AUTO takes it the rest of the way
        // (positive roll and pitch rates are what right and forward stick ask for)
        axisRoll = launchControlLiftFlipDirection == LAUNCH_LIFT_FLIP_RIGHT ? 1.0f
            : launchControlLiftFlipDirection == LAUNCH_LIFT_FLIP_LEFT ? -1.0f : 0.0f;
        axisPitch = launchControlLiftFlipDirection == LAUNCH_LIFT_FLIP_FORWARD ? 1.0f
            : launchControlLiftFlipDirection == LAUNCH_LIFT_FLIP_BACK ? -1.0f : 0.0f;
        axisNorm = 1.0f;
        forced = true;
    } else if (uz > 0.0f && axisNorm < 0.17f) {
        // AUTO, still nearly level, so no tilt to say which way: roll over, toward any lean
        axisRoll = uy >= 0.0f ? 1.0f : -1.0f;
        axisPitch = 0.0f;
        axisNorm = 1.0f;
    }
    const float rate = forced ? launchControlLiftFlipRate
        : MIN(LAUNCH_CONTROL_LIFT_FLIP_GAIN * angleToGoDeg, launchControlLiftFlipRate);
    if (axisNorm > 1e-4f) {
        launchControlLiftRate[FD_ROLL] = rate * axisRoll / axisNorm;
        launchControlLiftRate[FD_PITCH] = rate * axisPitch / axisNorm;
    } else {
        launchControlLiftRate[FD_ROLL] = 0.0f;   // exactly upside down
        launchControlLiftRate[FD_PITCH] = 0.0f;
    }
    launchControlLiftRate[FD_YAW] = 0.0f;
    return angleToGoDeg;
}
#endif

#ifdef USE_LAUNCH_CONTROL_LIFT
// One motor not pulling its weight, seen two ways (filtered, so a moment's difference
// doesn't count):
// - the mixer has to drive one motor much harder than another to hold level, more than
//   any balance or CoG difference needs (healthy lifts: up to about 12 %)
// - with RPM telemetry, a motor turning much faster or slower for its command than the
//   others: a slipping or loose prop unloads it, a failing motor drags (healthy: about 2 %)
static bool launchControlLiftMotorsUnhealthy(float dt)
{
    const int motorCount = getMotorCount();
    const float outputLow = getMotorOutputLow();
    const float outputRange = getMotorOutputHigh() - outputLow;
    if (motorCount < 2 || motorCount > MAX_SUPPORTED_MOTORS || outputRange <= 0.0f) {
        return false;
    }
#ifdef USE_DSHOT_TELEMETRY
    const bool haveRpm = useDshotTelemetry && isDshotTelemetryActive();
#else
    const bool haveRpm = false;
#endif
    const float k = launchControlLiftMotorFiltersPrimed ? dt / (LAUNCH_CONTROL_LIFT_MOTOR_FILTER_S + dt) : 1.0f;
    launchControlLiftMotorFiltersPrimed = true;

    float lowest = 1.0f;
    float highest = 0.0f;
    float rpmPerCommand[MAX_SUPPORTED_MOTORS];
    for (int i = 0; i < motorCount; i++) {
        const float command = constrainf((motor[i] - outputLow) / outputRange, 0.0f, 1.0f);
        launchControlLiftMotorCommand[i] += (command - launchControlLiftMotorCommand[i]) * k;
        lowest = MIN(lowest, launchControlLiftMotorCommand[i]);
        highest = MAX(highest, launchControlLiftMotorCommand[i]);
#ifdef USE_DSHOT_TELEMETRY
        if (haveRpm) {
            launchControlLiftMotorRpm[i] += (getDshotRpm(i) - launchControlLiftMotorRpm[i]) * k;
        }
#endif
        rpmPerCommand[i] = launchControlLiftMotorRpm[i] / MAX(launchControlLiftMotorCommand[i], 0.2f);
    }
    if (highest - lowest > LAUNCH_CONTROL_LIFT_MOTOR_SPREAD_PERCENT / 100.0f) {
        return true;
    }
    if (!haveRpm) {
        return false;
    }

    // compare each motor with the median, so one bad motor can't drag the reference
    float sorted[MAX_SUPPORTED_MOTORS];
    for (int i = 0; i < motorCount; i++) {
        float value = rpmPerCommand[i];
        int j = i;
        for (; j > 0 && sorted[j - 1] > value; j--) {
            sorted[j] = sorted[j - 1];
        }
        sorted[j] = value;
    }
    const float median = (motorCount & 1) ? sorted[motorCount / 2]
        : 0.5f * (sorted[motorCount / 2 - 1] + sorted[motorCount / 2]);
    if (median <= 0.0f) {
        return false;   // no RPM yet
    }
    for (int i = 0; i < motorCount; i++) {
        if (fabsf(rpmPerCommand[i] / median - 1.0f) > LAUNCH_CONTROL_LIFT_MOTOR_RPM_PERCENT / 100.0f) {
            return true;
        }
    }
    return false;
}

// What the OSD says for a moment after a lift hands over
const char *getLaunchControlLiftHandoverMessage(void)
{
    switch (launchControlLiftEndReason) {
    case LIFT_END_SWITCH:
        return "LIFT ABORT: STICKS LIVE";    // the pilot switched it off
    case LIFT_END_CONTROL:
    case LIFT_END_FLIP:
    case LIFT_END_UPSET:
        return "LIFT FAULT: STICKS LIVE";    // couldn't hold level, flip or stay upside down
    case LIFT_END_MOTOR:
        return "LIFT MOTOR: STICKS LIVE";    // a motor not pulling its weight
    case LIFT_END_IMPACT:
        return "LIFT IMPACT: STICKS LIVE";
    default:
        return "STICKS LIVE";
    }
}

// For the blackbox log: what LIFT is doing now, or how the last lift ended
uint8_t getLaunchControlLiftLogState(void)
{
    if (launchControlState == LAUNCH_CONTROL_LIFTING) {
        return launchControlLiftPhase == LIFT_PHASE_FLIP ? LAUNCH_LIFT_LOG_FLIP
            : launchControlLiftPhase == LIFT_PHASE_BRAKE ? LAUNCH_LIFT_LOG_BRAKE : LAUNCH_LIFT_LOG_CLIMB;
    }
    if (launchControlLiftStaged) {
        return LAUNCH_LIFT_LOG_STAGED;
    }
    if (launchControlLiftRan && launchControlState == LAUNCH_CONTROL_TRIGGERED) {
        return LAUNCH_LIFT_LOG_ENDED + launchControlLiftEndReason;
    }
    return LAUNCH_LIFT_LOG_NONE;
}

static void launchControlLiftEnd(timeUs_t currentTimeUs, launchControlLiftEndReason_e reason)
{
    launchControlState = LAUNCH_CONTROL_TRIGGERED;
    launchControlLiftEndUs = currentTimeUs ? currentTimeUs : 1;
    launchControlLiftEndReason = reason;
    launchControlLiftPhase = LIFT_PHASE_CLIMB;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        launchControlLiftRate[axis] = 0.0f;
    }
    // Handed over still climbing (no brake, the brake time ran out, or an abort): follow the
    // coast to its top, so how high the lift went is the real top, not where it handed over
    launchControlLiftRising = launchControlLiftClimbSpeed > 0.0f && reason != LIFT_END_DISARM;
    launchControlLiftTopUs = launchControlLiftRising ? 0 : launchControlLiftEndUs;
}
#endif

// Runs the LIFT. Nothing but time, the trigger (or launch control) going off, losing the
// ability to hold level, a disarm or a failsafe ends the climb: stick movement deliberately
// does not, so the pilot can set the sticks for the handover while the quad climbs. On a
// momentary trigger this makes the lift hold-to-run: letting go hands the quad back to the
// sticks.
// With launch_lift_brake set, time up doesn't hand over straight away: the throttle drops,
// the quad rolls upside down and brakes at that throttle until the climb has stopped, and
// then the sticks are live. The coast after a lift is what would carry it highest, so this
// keeps the top of every lift a few metres above where the lift time ends it.
// NOINLINE: called from the PID loop, but kept out of its fast code (ITCM RAM on F7 is only 16 KB).
NOINLINE void launchControlLiftUpdate(timeUs_t currentTimeUs)
{
#ifdef USE_LAUNCH_CONTROL_LIFT
    const timeDelta_t sinceLastUs = launchControlLiftLastUpdateUs ? cmpTimeUs(currentTimeUs, launchControlLiftLastUpdateUs) : 0;
    const float dt = MAX(sinceLastUs, 0) * 1e-6f;
    launchControlLiftLastUpdateUs = currentTimeUs;

#ifdef USE_ACC
    if (launchControlState == LAUNCH_CONTROL_ACTIVE && sensors(SENSOR_ACC)) {
        // staged and still: learn what "not moving" reads, so the climb speed starts from zero
        launchControlLiftRestG += (launchControlLiftVerticalG() - launchControlLiftRestG)
            * dt / (LAUNCH_CONTROL_LIFT_REST_TIME_S + dt);
    }
    if (launchControlLiftRising && sensors(SENSOR_ACC)) {
        // handed over while still climbing: keep integrating until the coast tops out
        launchControlLiftClimbSpeed += (launchControlLiftVerticalG() - launchControlLiftRestG) * G_ACCELERATION * dt;
        launchControlLiftHeight += launchControlLiftClimbSpeed * dt;
        launchControlLiftTop = MAX(launchControlLiftTop, launchControlLiftHeight);
        if (launchControlLiftClimbSpeed <= 0.0f || !ARMING_FLAG(ARMED)
            || cmpTimeUs(currentTimeUs, launchControlLiftEndUs) >= LAUNCH_CONTROL_LIFT_TOP_FOLLOW_MS * 1000) {
            launchControlLiftRising = false;
            launchControlLiftTopUs = currentTimeUs ? currentTimeUs : 1;
        }
    }
#endif
    // Forget the handover and top times once their notices are over. cmpTimeUs is 32 bits, so
    // about 72 minutes later a stale time would read as recent again and bring a notice back.
    // This runs every PID loop, so it doesn't depend on the OSD asking.
    if (launchControlLiftEndUs && !launchControlLiftRising
        && cmpTimeUs(currentTimeUs, launchControlLiftEndUs) >= LAUNCH_CONTROL_LIFT_HANDOVER_NOTICE_MS * 1000) {
        launchControlLiftEndUs = 0;
    }
    if (launchControlLiftTopUs
        && cmpTimeUs(currentTimeUs, launchControlLiftTopUs) >= LAUNCH_CONTROL_LIFT_TOP_NOTICE_MS * 1000) {
        launchControlLiftTopUs = 0;
    }
    if (launchControlState != LAUNCH_CONTROL_LIFTING) {
        return;
    }

    const bool switchAbort = !IS_RC_MODE_ACTIVE(BOXLAUNCHTRIGGER) || !IS_RC_MODE_ACTIVE(BOXLAUNCHCONTROL);
    if (switchAbort || failsafeIsActive() || !ARMING_FLAG(ARMED)) {
        // the switch always gives the pilot the sticks, whatever stage the lift is at
        launchControlLiftEnd(currentTimeUs, switchAbort ? LIFT_END_SWITCH
            : failsafeIsActive() ? LIFT_END_FAILSAFE : LIFT_END_DISARM);
        return;
    }

#ifdef USE_ACC
    launchControlLiftClimbSpeed += (launchControlLiftVerticalG() - launchControlLiftRestG) * G_ACCELERATION * dt;
    launchControlLiftHeight += launchControlLiftClimbSpeed * dt;
    launchControlLiftTop = MAX(launchControlLiftTop, launchControlLiftHeight);
#endif
    const timeDelta_t inPhaseUs = cmpTimeUs(currentTimeUs, launchControlLiftPhaseStartUs);
    UNUSED(inPhaseUs);

    if (launchControlLiftPhase == LIFT_PHASE_CLIMB) {
        const bool timeUp = cmpTimeUs(currentTimeUs, launchControlLiftStartUs) >= (timeDelta_t)launchControlLiftTimeMs * 1000;

        // Can it still hold level? Self-level keeps a healthy quad within a few degrees and
        // barely rotating. A loose prop, a motor or ESC fault or a strike shows up as a tilt
        // it can't correct, or a spin, and full throttle would drive it wherever it points.
        // Brief upsets (a gust, leaving uneven ground) are ridden out; a sustained one hands
        // the quad straight back to the pilot rather than finishing the climb.
        bool lostControl = false;
        bool impact = false;
#ifdef USE_ACC
        bool troubled = getCosTiltAngle() < cos_approx(DEGREES_TO_RADIANS(LAUNCH_CONTROL_LIFT_ABORT_TILT_DEG));
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            troubled = troubled || fabsf(gyro.gyroADCf[axis]) > LAUNCH_CONTROL_LIFT_ABORT_RATE_DPS;
        }
        // An impact (a ceiling, a bird, a branch) is a spike in jerk that thrust alone doesn't
        // make, however hard the quad accelerates. Off unless set: it ends the lift at once.
        launchControlLiftPeakJerk = fmaxf(launchControlLiftPeakJerk, acc.jerkMagnitude);
        impact = launchControlLiftImpactJerk > 0.0f && acc.jerkMagnitude > launchControlLiftImpactJerk;
        if (!troubled) {
            launchControlLiftTroubleStartUs = 0;
        } else if (launchControlLiftTroubleStartUs == 0) {
            launchControlLiftTroubleStartUs = currentTimeUs ? currentTimeUs : 1;
        } else {
            lostControl = cmpTimeUs(currentTimeUs, launchControlLiftTroubleStartUs) >= LAUNCH_CONTROL_LIFT_ABORT_TIME_MS * 1000;
        }
#endif

        // Is every motor pulling its weight? Holding level isn't enough: a loose prop or a
        // failing motor can be levelled out by the other three at full throttle, and the
        // lift would carry on. Not in the spin-up, where the RPM lags the command.
        bool motorFault = false;
        if (launchControlLiftMotorsUnhealthy(dt)
            && cmpTimeUs(currentTimeUs, launchControlLiftStartUs) >= LAUNCH_CONTROL_LIFT_MOTOR_SETTLE_MS * 1000) {
            if (launchControlLiftMotorFaultStartUs == 0) {
                launchControlLiftMotorFaultStartUs = currentTimeUs ? currentTimeUs : 1;
            } else {
                motorFault = cmpTimeUs(currentTimeUs, launchControlLiftMotorFaultStartUs) >= LAUNCH_CONTROL_LIFT_MOTOR_FAULT_MS * 1000;
            }
        } else {
            launchControlLiftMotorFaultStartUs = 0;
        }
        // A motor that has stopped (stalled, or an ESC that has reset or given up) is a fault
        // straight away, whatever the other three are doing. It turned at the trigger, so
        // the spin-up can't be mistaken for it.
        if (launchControlLiftFindStoppedMotor()) {
            if (launchControlLiftMotorStoppedStartUs == 0) {
                launchControlLiftMotorStoppedStartUs = currentTimeUs ? currentTimeUs : 1;
            } else if (cmpTimeUs(currentTimeUs, launchControlLiftMotorStoppedStartUs) >= LAUNCH_CONTROL_LIFT_MOTOR_STOPPED_MS * 1000) {
                motorFault = true;
            }
        } else {
            launchControlLiftMotorStoppedStartUs = 0;
        }

        if (lostControl || impact || motorFault) {
            // no flip for a quad that's hurt: the pilot gets it as it is
            launchControlLiftEnd(currentTimeUs, impact ? LIFT_END_IMPACT : lostControl ? LIFT_END_CONTROL : LIFT_END_MOTOR);
        } else if (timeUp) {
#ifdef USE_ACC
            if (launchControlLiftBrakeFraction > 0.0f && launchControlLiftClimbSpeed >= LAUNCH_CONTROL_LIFT_BRAKE_MIN_SPEED) {
                // the run is over: throttle off and roll upside down
                launchControlLiftPhase = LIFT_PHASE_FLIP;
                launchControlLiftPhaseStartUs = currentTimeUs;
                launchControlLiftTroubleStartUs = 0;
                launchControlLiftMotorThrottle = 0.0f;
                launchControlLiftSteerInverted();
            } else
#endif
            {
                launchControlLiftEnd(currentTimeUs, LIFT_END_TIME);
            }
        }
        return;
    }

#ifdef USE_ACC
    const float angleToGoDeg = launchControlLiftSteerInverted();
    if (launchControlLiftPhase == LIFT_PHASE_FLIP) {
        if (angleToGoDeg <= LAUNCH_CONTROL_LIFT_BRAKE_START_DEG) {
            // upside down: thrust now pushes down, against the climb
            launchControlLiftPhase = LIFT_PHASE_BRAKE;
            launchControlLiftPhaseStartUs = currentTimeUs;
            launchControlLiftMotorThrottle = launchControlLiftBrakeFraction;
        } else if (inPhaseUs >= launchControlLiftFlipTimeoutUs) {
            // it isn't turning over as told
            launchControlLiftEnd(currentTimeUs, LIFT_END_FLIP);
        }
        return;
    }

    // LIFT_PHASE_BRAKE
    if (angleToGoDeg <= LAUNCH_CONTROL_LIFT_BRAKE_ABORT_DEG) {
        launchControlLiftTroubleStartUs = 0;
    } else if (launchControlLiftTroubleStartUs == 0) {
        launchControlLiftTroubleStartUs = currentTimeUs ? currentTimeUs : 1;
    } else if (cmpTimeUs(currentTimeUs, launchControlLiftTroubleStartUs) >= LAUNCH_CONTROL_LIFT_ABORT_TIME_MS * 1000) {
        launchControlLiftEnd(currentTimeUs, LIFT_END_UPSET);   // can't hold it upside down
        return;
    }
    if (launchControlLiftClimbSpeed <= 0.0f) {
        // stopped: the top of the jump. Throttle back to the stick, sticks live.
        launchControlLiftEnd(currentTimeUs, LIFT_END_STOPPED);
    } else if (inPhaseUs >= launchControlLiftBrakeTimeUs) {
        launchControlLiftEnd(currentTimeUs, LIFT_END_BRAKE_LIMIT);
    }
#else
    launchControlLiftEnd(currentTimeUs, LIFT_END_TIME);
#endif
#else
    UNUSED(currentTimeUs);
#endif
}
