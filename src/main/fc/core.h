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

#pragma once

#include "common/time.h"
#include "pg/pg.h"

typedef struct throttleCorrectionConfig_s {
    uint16_t throttle_correction_angle;     // the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
    uint8_t throttle_correction_value;      // the correction that will be applied at throttle_correction_angle.
} throttleCorrectionConfig_t;

typedef enum {
    LAUNCH_CONTROL_DISABLED = 0,
    LAUNCH_CONTROL_ACTIVE,
    LAUNCH_CONTROL_LIFTING,     // LIFT mode only: timed, stick-independent climb between trigger and handover
    LAUNCH_CONTROL_TRIGGERED,
} launchControlState_e;

// launch_lift_flip_direction: which way the quad turns over for the brake
typedef enum {
    LAUNCH_LIFT_FLIP_AUTO = 0,  // roll, toward any lean it has at the end of the lift
    LAUNCH_LIFT_FLIP_RIGHT,     // roll right, as the roll stick would
    LAUNCH_LIFT_FLIP_LEFT,
    LAUNCH_LIFT_FLIP_FORWARD,   // pitch forward (nose down), as the pitch stick would
    LAUNCH_LIFT_FLIP_BACK,
    LAUNCH_LIFT_FLIP_COUNT
} launchLiftFlipDirection_e;

// Why a LIFT handed the sticks back. These numbers are written to the blackbox log
// (launchLiftState = LAUNCH_LIFT_LOG_ENDED + reason), so keep them stable.
typedef enum {
    LIFT_END_TIME = 0,          // finished: lift time up, no brake (off, or too slow to need it)
    LIFT_END_STOPPED = 1,       // finished: the brake stopped the climb
    LIFT_END_BRAKE_LIMIT = 2,   // finished: launch_lift_brake_time ran out before the climb stopped
    LIFT_END_SWITCH = 3,        // the pilot switched it off
    LIFT_END_FAILSAFE = 4,      // failsafe
    LIFT_END_DISARM = 5,        // disarmed mid-lift
    LIFT_END_CONTROL = 6,       // couldn't hold level in the climb
    LIFT_END_FLIP = 7,          // didn't turn over for the brake
    LIFT_END_UPSET = 8,         // knocked off upside down while braking
    LIFT_END_MOTOR = 9,         // a motor not pulling its weight
    LIFT_END_IMPACT = 10,       // launch_lift_impact
} launchControlLiftEndReason_e;

// What the blackbox log records about LIFT (slow frame field launchLiftState), written the
// moment it changes, so the log has the exact time of each step
typedef enum {
    LAUNCH_LIFT_LOG_NONE = 0,       // not a LIFT, or not armed in LNCH
    LAUNCH_LIFT_LOG_STAGED = 1,     // armed in LNCH, waiting for the trigger
    LAUNCH_LIFT_LOG_CLIMB = 2,      // sent: the lift
    LAUNCH_LIFT_LOG_FLIP = 3,       // lift time up, turning over
    LAUNCH_LIFT_LOG_BRAKE = 4,      // upside down, braking
    LAUNCH_LIFT_LOG_ENDED = 16,     // sticks live: 16 + launchControlLiftEndReason_e
} launchLiftLogState_e;

typedef enum {
    LAUNCH_CONTROL_MODE_NORMAL = 0,
    LAUNCH_CONTROL_MODE_PITCHONLY,
    LAUNCH_CONTROL_MODE_FULL,
#ifdef USE_LAUNCH_CONTROL_LIFT
    LAUNCH_CONTROL_MODE_LIFT,   // fixed throttle for a fixed time after the trigger, sticks ignored, then hand over
#endif
    LAUNCH_CONTROL_MODE_COUNT // must be the last element
} launchControlMode_e;

typedef enum {
    DISARM_REASON_ARMING_DISABLED   = 0,
    DISARM_REASON_FAILSAFE          = 1,
    DISARM_REASON_THROTTLE_TIMEOUT  = 2,
    DISARM_REASON_STICKS            = 3,
    DISARM_REASON_SWITCH            = 4,
    DISARM_REASON_CRASH_PROTECTION  = 5,
    DISARM_REASON_RUNAWAY_TAKEOFF   = 6,
    DISARM_REASON_GPS_RESCUE        = 7,
    DISARM_REASON_SERIAL_COMMAND    = 8,
    DISARM_REASON_LANDING           = 9,
    DISARM_REASON_CRASHFLIP         = 10,
    DISARM_REASON_LAUNCH_ABORT      = 11,
#ifdef UNIT_TEST
    DISARM_REASON_SYSTEM            = 255,
#endif
} flightLogDisarmReason_e;

// LIFT defaults, outside USE_LAUNCH_CONTROL_LIFT: the PID profile reset uses them in every build
#define LAUNCH_CONTROL_LIFT_FLIP_RATE_DEFAULT_DPS 1200 // launch_lift_flip_rate: fastest it turns over [deg/s]
#define LAUNCH_CONTROL_LIFT_BRAKE_TIME_DEFAULT_MS 1500 // launch_lift_brake_time: longest brake, whatever the speed estimate says

#ifdef USE_LAUNCH_CONTROL
#define LAUNCH_CONTROL_THROTTLE_TRIGGER_MAX 90
extern const char * const osdLaunchControlModeNames[LAUNCH_CONTROL_MODE_COUNT];
#endif

#ifdef USE_LAUNCH_CONTROL_LIFT
#define LAUNCH_CONTROL_LIFT_TIME_MIN_MS 100
#define LAUNCH_CONTROL_LIFT_TIME_MAX_MS 10000
#define LAUNCH_CONTROL_LIFT_THROTTLE_MIN 25
#define LAUNCH_CONTROL_LIFT_HANDOVER_NOTICE_MS 1000  // how long the OSD confirms the sticks are live again
// LIFT self-levels on roll and pitch, so it can be sent from slightly uneven ground
#define LAUNCH_CONTROL_LIFT_MAX_START_TILT_DEG 20    // the trigger is refused beyond this tilt
#define LAUNCH_CONTROL_LIFT_LEVEL_MAX_RATE_DPS 300   // cap on the self-level correction rate
// It can't hold level (loose prop, motor/ESC fault, strike): tilted past the angle, or
// rotating faster than the rate on any axis, for the time, ends the lift early
#define LAUNCH_CONTROL_LIFT_ABORT_TILT_DEG     45
#define LAUNCH_CONTROL_LIFT_ABORT_RATE_DPS     500
#define LAUNCH_CONTROL_LIFT_ABORT_TIME_MS      150
// launch_lift_impact: jerk (the rate of change of acceleration, filtered as for EZ landing)
// that counts as a hit, in units of 10 g/s like landing_disarm_threshold. 0 = off.
#define LAUNCH_CONTROL_LIFT_IMPACT_MAX         250
// launch_lift_brake: when the lift time is up, the throttle drops, the quad rolls upside
// down and the brake throttle stops the climb; the sticks go live once it has stopped.
// The coast after a lift is what carries it highest (about 35 m from 40 m/s); braking
// inverted stops it in about a quarter of that.
#define LAUNCH_CONTROL_LIFT_FLIP_RATE_MIN_DPS  200
// Flights at 1200 deg/s peaked 12-18 % above the asked rate as the flip started. 1600 plus that
// stays inside the 2000 deg/s gyro range: a clipped gyro would throw off the attitude the flip
// and brake steer by.
#define LAUNCH_CONTROL_LIFT_FLIP_RATE_MAX_DPS  1600
#define LAUNCH_CONTROL_LIFT_FLIP_GAIN          15.0f // rotation rate per degree still to go [deg/s per deg]
#define LAUNCH_CONTROL_LIFT_BRAKE_START_DEG    30    // brake throttle comes on within this of upside down
#define LAUNCH_CONTROL_LIFT_FLIP_TIMEOUT_MS    300   // the flip gets this, plus twice the time a half turn takes at the flip rate,
                                                     // before something is taken to be wrong and the sticks are handed back
#define LAUNCH_CONTROL_LIFT_BRAKE_ABORT_DEG    60    // knocked this far off upside down while braking, for the abort time: hand back
#define LAUNCH_CONTROL_LIFT_BRAKE_TIME_MIN_MS  100
#define LAUNCH_CONTROL_LIFT_BRAKE_TIME_MAX_MS  3000
#define LAUNCH_CONTROL_LIFT_BRAKE_MIN_SPEED    5.0f  // climbing slower than this [m/s] at the end: nothing to brake, no flip
#define LAUNCH_CONTROL_LIFT_REST_TIME_S        0.25f // how quickly the staged reading of "still" settles [s]
// Motor health in the climb: one motor not pulling its weight (a loose or slipping prop,
// a failing motor or ESC) ends the lift even if the other three can still hold it level.
// Healthy lifts measured: command spread up to about 12 %, RPM per command within 2.5 %.
#define LAUNCH_CONTROL_LIFT_MOTOR_SETTLE_MS     300   // not in the spin-up: the RPM lags the command
#define LAUNCH_CONTROL_LIFT_MOTOR_FILTER_S      0.025f
#define LAUNCH_CONTROL_LIFT_MOTOR_SPREAD_PERCENT 25   // one motor this much harder driven than another
#define LAUNCH_CONTROL_LIFT_MOTOR_RPM_PERCENT   10    // a motor's RPM per command this far from the others' (needs RPM telemetry)
#define LAUNCH_CONTROL_LIFT_MOTOR_FAULT_MS      200   // for this long
// A motor that isn't turning (needs RPM telemetry). A 5" idles at 1500 RPM or more; a stopped
// motor reads 0, or about 200 on ESCs that report a minimum. Props held by long grass at
// start-up stay stopped even at full throttle, because the ESC limits power until it sees
// the motor turn. LIFT won't trigger with a motor stopped, and a motor that stops in the
// climb ends it at once (no settle time: the RPM only rises after the trigger).
#define LAUNCH_CONTROL_LIFT_MOTOR_TURNING_RPM   500
#define LAUNCH_CONTROL_LIFT_MOTOR_STOPPED_MS    100
// How high it went, for the OSD (from the same accelerometer integration as the climb speed)
#define LAUNCH_CONTROL_LIFT_TOP_FOLLOW_MS       5000  // handed over still climbing: follow the coast to its top for up to this
#define LAUNCH_CONTROL_LIFT_TOP_NOTICE_MS       10000 // then show the top for this long
#endif // USE_LAUNCH_CONTROL_LIFT

PG_DECLARE(throttleCorrectionConfig_t, throttleCorrectionConfig);

union rollAndPitchTrims_u;
void handleInflightCalibrationStickPosition(void);

void resetArmingDisabled(void);

void disarm(flightLogDisarmReason_e reason);
void tryArm(void);

bool processRx(timeUs_t currentTimeUs);
void processRxModes(timeUs_t currentTimeUs);
void updateArmingStatus(void);

void taskGyroSample(timeUs_t currentTimeUs);
bool gyroFilterReady(void);
bool pidLoopReady(void);
void taskFiltering(timeUs_t currentTimeUs);
void taskMainPidLoop(timeUs_t currentTimeUs);

bool isCrashFlipModeActive(void);
int8_t calculateThrottlePercent(void);
uint8_t calculateThrottlePercentAbs(void);
bool areSticksActive(uint8_t stickPercentLimit);
void runawayTakeoffTemporaryDisable(uint8_t disableFlag);
bool wasThrottleRaised(void);
timeUs_t getLastDisarmTimeUs(void);
bool isTryingToArm(void);
void resetTryingToArm(void);

void subTaskTelemetryPollSensors(timeUs_t currentTimeUs);

bool isLaunchControlActive(void);
bool isLaunchControlLifting(void);
bool isLaunchControlLiftBraking(void);
float getLaunchControlLiftRate(int axis);
float getLaunchControlLiftClimbSpeed(void);
float getLaunchControlLiftThrottle(void);
uint32_t getLaunchControlLiftRemainingMs(void);
bool isLaunchControlLiftJustEnded(void);
bool isLaunchControlLiftHandoverRecent(void);
bool isLaunchControlLiftAborted(void);
bool isLaunchControlLiftImpacted(void);
bool isLaunchControlLiftNotLevel(void);
uint8_t getLaunchControlLiftStoppedMotor(void);
int32_t getLaunchControlLiftHeightCm(bool *atTop);
bool isLaunchControlLiftAwaitingTriggerOff(void);
bool isLaunchControlLiftStaged(void);
bool isLaunchControlPreStaged(void);
const char *getLaunchControlLiftPreArmMessage(void);
const char *getLaunchControlLiftHandoverMessage(void);
uint8_t getLaunchControlLiftLogState(void);
void launchControlLiftUpdate(timeUs_t currentTimeUs);
