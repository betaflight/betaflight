/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#if defined(USE_WING) && defined(USE_LAUNCH_WING)

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "io/gps.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "pg/launch_wing.h"

#include "launch_wing.h"

#define LAUNCH_IDLE_RAMP_MS         1500
#define LAUNCH_SWING_MIN_YAW_DPS    100.0f
#define LAUNCH_GRAVITY_CMSS         980.665f
#define LAUNCH_STATIONARY_SPEED_CMS 50
#define LAUNCH_STATIONARY_ACC_BAND  0.15f
#define LAUNCH_STATIONARY_GYRO_DPS  25.0f
#define LAUNCH_STATIONARY_VARIO_CMS 100.0f
#define LAUNCH_MIN_SATS             5

typedef struct {
    launchWingState_e state;
    bool latched;
    timeUs_t stateEnteredAtUs;
    timeUs_t detectedAtUs;
    float pitchTargetDeg;
    float throttle;
    float handover;   // 0 while the launch owns the aircraft, 1 once the pilot does
    timeUs_t attitudeLostAtUs;
    launchWingExit_e exit;
} launchWingRuntime_t;

static launchWingRuntime_t launchWing;

static void setState(launchWingState_e state, timeUs_t currentTimeUs)
{
    launchWing.state = state;
    launchWing.stateEnteredAtUs = currentTimeUs;
}

// Every exit from launch control hands the aircraft straight back. pidLevel and
// mixTable keep blending on the handover factor until the mode bit clears on
// the next rx cycle, so a partial factor would leave the pilot fighting a stale
// launch demand for that window.
static void endLaunch(launchWingState_e terminalState, launchWingExit_e exit)
{
    launchWing.state = terminalState;
    launchWing.handover = 1.0f;
    launchWing.exit = exit;
}

static float elapsedMs(timeUs_t currentTimeUs)
{
    return cmpTimeUs(currentTimeUs, launchWing.stateEnteredAtUs) * 1e-3f;
}

// Linear 0..1 progress through a ramp of the given duration. A zero-length ramp
// is complete on the first evaluation.
static float rampProgress(timeUs_t currentTimeUs, float durationMs)
{
    if (durationMs <= 0.0f) {
        return 1.0f;
    }
    return constrainf(elapsedMs(currentTimeUs) / durationMs, 0.0f, 1.0f);
}

static bool isStationary(void)
{
    if (fabsf(acc.accMagnitude - 1.0f) > LAUNCH_STATIONARY_ACC_BAND) {
        return false;
    }
    // Specific force alone cannot separate sitting on the ground from a steady
    // glide, so every other motion signal the airframe has must agree too.
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        if (fabsf(gyro.gyroADCf[axis]) > LAUNCH_STATIONARY_GYRO_DPS) {
            return false;
        }
    }
    if (isAltitudeAvailable() && fabsf(getAltitudeDerivative()) > LAUNCH_STATIONARY_VARIO_CMS) {
        return false;
    }
#ifdef USE_GPS
    if (STATE(GPS_FIX) && gpsSol.numSat >= LAUNCH_MIN_SATS
        && gpsSol.groundSpeed >= LAUNCH_STATIONARY_SPEED_CMS) {
        return false;
    }
#endif
    return true;
}

// Body frame is NWU: x forward, y left, z up. acc.accADC is rotated but
// unscaled, and still contains gravity, so a level stationary airframe reads
// zero on x.
static bool launchDetected(timeUs_t currentTimeUs)
{
    const launchWingConfig_t *cfg = launchWingConfig();
    const float accScale = acc.dev.acc_1G_rec;
    const float axG = acc.accADC.x * accScale;
    const float ayG = acc.accADC.y * accScale;
    const float yawDps = gyro.gyroADCf[FD_YAW];

    const bool isLevel = getCosTiltAngle() >= cos_approx(DEGREES_TO_RADIANS((float)cfg->maxAngleDeg));
    const bool bungee = (axG >= cfg->detectAccelDecig * 0.1f) && isLevel;

    // Tangential speed of an airframe swung about the thrower's shoulder. The
    // magnitude form avoids depending on the accel and gyro sign conventions
    // agreeing; the forward-acceleration term already rejects a backswing.
    float swingCmS = 0.0f;
    if (fabsf(yawDps) > LAUNCH_SWING_MIN_YAW_DPS) {
        swingCmS = fabsf(ayG) * LAUNCH_GRAVITY_CMSS / DEGREES_TO_RADIANS(fabsf(yawDps));
    }
    const bool swing = (swingCmS > cfg->detectVelocityCmS) && (axG > 0.0f);

    bool forward = false;
#ifdef USE_GPS
    forward = STATE(GPS_FIX) && gpsSol.numSat >= LAUNCH_MIN_SATS
        && gpsSol.groundSpeed > cfg->detectVelocityCmS && (axG > 0.0f);
#endif

    DEBUG_SET(DEBUG_LAUNCH, 3, lrintf(axG * 100.0f));                //!< Forward Acceleration [unit:0.01g]
    DEBUG_SET(DEBUG_LAUNCH, 4, lrintf(swingCmS));                    //!< Swing Speed [unit:cm/s]
    DEBUG_SET(DEBUG_LAUNCH, 6, (bungee ? 1 : 0) | (swing ? 2 : 0) | (forward ? 4 : 0));  //!< Launch Detectors [flags:Bungee|Swing|Ground Speed]
    UNUSED(currentTimeUs);

    return bungee || swing || forward;
}

static bool sticksMoved(void)
{
    const float deadband = launchWingConfig()->abortDeadbandPercent / 100.0f;
    return getRcDeflectionAbs(FD_ROLL) > deadband || getRcDeflectionAbs(FD_PITCH) > deadband;
}

// The inhibit window runs from detection, not from state entry, so it spans the
// motor delay, the spin-up and the climb-out as one period.
static bool abortRequested(timeUs_t currentTimeUs)
{
    if (cmpTimeUs(currentTimeUs, launchWing.detectedAtUs) < (timeDelta_t)(launchWingConfig()->minTimeMs * 1000)) {
        return false;
    }
    return sticksMoved();
}

static bool maxAltitudeReached(void)
{
    const uint16_t maxAltitudeM = launchWingConfig()->maxAltitudeM;
    return maxAltitudeM > 0 && isAltitudeAvailable() && getAltitudeCm() >= maxAltitudeM * 100.0f;
}

// Bank and dive are tested separately rather than as one tilt magnitude: the
// launch commands a climb, and attitude.values.pitch is positive nose-down, so a
// magnitude bound would abort on the very thing the launch is there to fly.
// The bound must also hold continuously - a throw can slap the airframe past it
// for a few frames - so any frame inside the bound re-seeds the clock, the same
// rule detection uses.
static bool attitudeLost(timeUs_t currentTimeUs)
{
    const int32_t limit = launchWingConfig()->abortAngleDeg * 10;
    const bool lost = limit > 0
        && (ABS(attitude.values.roll) > limit || attitude.values.pitch > limit);

    if (!lost) {
        launchWing.attitudeLostAtUs = currentTimeUs;
        return false;
    }
    return cmpTimeUs(currentTimeUs, launchWing.attitudeLostAtUs) >= LAUNCH_ATTITUDE_HOLD_MS * 1000;
}

void launchWingInit(void)
{
    launchWing.state = LAUNCH_WING_IDLE;
    launchWing.latched = false;
    launchWing.pitchTargetDeg = 0.0f;
    launchWing.throttle = 0.0f;
    launchWing.handover = 0.0f;
    launchWing.attitudeLostAtUs = 0;
    launchWing.exit = LAUNCH_WING_EXIT_NONE;
}

void launchWingArm(void)
{
    launchWingInit();
    pidResetTpaSpeed();
    launchWing.latched = IS_RC_MODE_ACTIVE(BOXLAUNCH)
        && isFixedWing()
        && sensors(SENSOR_ACC)
        && isStationary();
}

void launchWingDisarm(void)
{
    launchWingInit();
}

void launchWingSwitchOff(void)
{
    if (launchWingIsActive()) {
        endLaunch(LAUNCH_WING_ABORTED, LAUNCH_WING_EXIT_MODE_OFF);
    }
}

bool launchWingLatched(void)
{
    return launchWing.latched;
}

bool launchWingIsTerminal(void)
{
    return launchWing.state == LAUNCH_WING_FLYING || launchWing.state == LAUNCH_WING_ABORTED;
}

bool launchWingIsActive(void)
{
    return launchWing.state > LAUNCH_WING_IDLE && !launchWingIsTerminal();
}

// Before detection the aircraft is still in the pilot's hands, and the throttle
// stick is already up because that is what let the launch leave WAIT_THROTTLE.
// That is what makes cancelling here different from cancelling in the air.
bool launchWingIsPreLaunch(void)
{
    return launchWing.state > LAUNCH_WING_IDLE
        && launchWing.state <= LAUNCH_WING_WAIT_DETECTION;
}

bool launchWingThrottleValid(void)
{
    return launchWingIsActive();
}

bool launchWingHoldsIterm(void)
{
    return launchWingIsActive() && launchWing.state < LAUNCH_WING_SPINUP;
}

float launchWingGetThrottle(void)
{
    return launchWing.throttle;
}

float launchWingHandoverFactor(void)
{
    return launchWing.handover;
}

launchWingState_e launchWingGetState(void)
{
    return launchWing.state;
}

void launchWingUpdate(timeUs_t currentTimeUs)
{
    const launchWingConfig_t *cfg = launchWingConfig();
    const float idleThrottle = cfg->idleThrottlePercent * 0.01f;
    const float launchThrottle = cfg->throttlePercent * 0.01f;
    const float climbAngleDeg = cfg->climbAngleDeg;

    // Failsafe is checked here as well as in the mode gate: the gate only
    // re-evaluates on the rx task, and the launch must not keep commanding
    // throttle and attitude over the failsafe procedure in the meantime.
    if (!FLIGHT_MODE(LAUNCH_MODE) || failsafeIsActive()) {
        if (launchWing.state != LAUNCH_WING_IDLE && !launchWingIsTerminal()) {
            endLaunch(LAUNCH_WING_ABORTED, LAUNCH_WING_EXIT_MODE_OFF);
        }
        return;
    }

    // One test for every state where the launch is airborne and still
    // commanding. It hands back rather than cutting the motor: a dead-stick at
    // launch altitude is its own accident, and the pilot's throttle is already
    // up.
    if (launchWing.state >= LAUNCH_WING_MOTOR_DELAY && !launchWingIsTerminal()
        && attitudeLost(currentTimeUs)) {
        endLaunch(LAUNCH_WING_ABORTED, LAUNCH_WING_EXIT_ATTITUDE);
    }

    switch (launchWing.state) {
    case LAUNCH_WING_IDLE:
        setState(LAUNCH_WING_WAIT_THROTTLE, currentTimeUs);
        FALLTHROUGH;

    case LAUNCH_WING_WAIT_THROTTLE:
        launchWing.throttle = 0.0f;
        launchWing.pitchTargetDeg = 0.0f;
        if (calculateThrottleStatus() != THROTTLE_LOW
            && elapsedMs(currentTimeUs) >= cfg->idleDelayMs) {
            setState(LAUNCH_WING_MOTOR_IDLE, currentTimeUs);
        } else if (calculateThrottleStatus() == THROTTLE_LOW) {
            launchWing.stateEnteredAtUs = currentTimeUs;
        }
        break;

    case LAUNCH_WING_MOTOR_IDLE: {
        if (calculateThrottleStatus() == THROTTLE_LOW) {
            setState(LAUNCH_WING_WAIT_THROTTLE, currentTimeUs);
            break;
        }
        const float k = rampProgress(currentTimeUs, LAUNCH_IDLE_RAMP_MS);
        launchWing.throttle = idleThrottle * k;
        launchWing.pitchTargetDeg = climbAngleDeg * k;
        if (k >= 1.0f) {
            setState(LAUNCH_WING_WAIT_DETECTION, currentTimeUs);
        }
        break;
    }

    case LAUNCH_WING_WAIT_DETECTION:
        launchWing.throttle = idleThrottle;
        launchWing.pitchTargetDeg = climbAngleDeg;
        if (calculateThrottleStatus() == THROTTLE_LOW) {
            setState(LAUNCH_WING_WAIT_THROTTLE, currentTimeUs);
            break;
        }
        // The hold must be continuous: any frame that fails re-seeds the clock,
        // so a transient bump while carrying the aircraft cannot accumulate.
        if (!launchDetected(currentTimeUs)) {
            launchWing.stateEnteredAtUs = currentTimeUs;
        } else if (elapsedMs(currentTimeUs) >= cfg->detectTimeMs) {
            launchWing.detectedAtUs = currentTimeUs;
            // Start the attitude hold clock here, not on the first frame that
            // consults it: a throw that leaves the airframe past the bound
            // would otherwise measure its hold against a stale timestamp and
            // abort immediately.
            launchWing.attitudeLostAtUs = currentTimeUs;
            setState(LAUNCH_WING_MOTOR_DELAY, currentTimeUs);
        }
        break;

    case LAUNCH_WING_MOTOR_DELAY:
        launchWing.throttle = idleThrottle;
        launchWing.pitchTargetDeg = climbAngleDeg;
        if (abortRequested(currentTimeUs)) {
            endLaunch(LAUNCH_WING_ABORTED, LAUNCH_WING_EXIT_STICKS);
        } else if (elapsedMs(currentTimeUs) >= cfg->motorDelayMs) {
            setState(LAUNCH_WING_SPINUP, currentTimeUs);
        }
        break;

    case LAUNCH_WING_SPINUP: {
        launchWing.pitchTargetDeg = climbAngleDeg;
        const float k = rampProgress(currentTimeUs, cfg->spinupTimeMs);
        launchWing.throttle = idleThrottle + (launchThrottle - idleThrottle) * k;
        if (abortRequested(currentTimeUs)) {
            endLaunch(LAUNCH_WING_ABORTED, LAUNCH_WING_EXIT_STICKS);
        } else if (k >= 1.0f) {
            setState(LAUNCH_WING_IN_PROGRESS, currentTimeUs);
        }
        break;
    }

    case LAUNCH_WING_IN_PROGRESS:
        launchWing.throttle = launchThrottle;
        launchWing.pitchTargetDeg = climbAngleDeg;
        if (abortRequested(currentTimeUs)) {
            endLaunch(LAUNCH_WING_ABORTED, LAUNCH_WING_EXIT_STICKS);
        } else if (maxAltitudeReached()) {
            launchWing.exit = LAUNCH_WING_EXIT_ALTITUDE;
            setState(LAUNCH_WING_FINISH, currentTimeUs);
        } else if (cfg->timeoutMs > 0 && elapsedMs(currentTimeUs) >= cfg->timeoutMs) {
            launchWing.exit = LAUNCH_WING_EXIT_TIMEOUT;
            setState(LAUNCH_WING_FINISH, currentTimeUs);
        }
        break;

    case LAUNCH_WING_FINISH:
        // The launch-side demands are held; the blend towards the pilot's own
        // throttle and angle target happens where those values actually live,
        // in mixTable and pidLevel.
        launchWing.throttle = launchThrottle;
        launchWing.pitchTargetDeg = climbAngleDeg;
        launchWing.handover = rampProgress(currentTimeUs, cfg->endTimeMs);
        // Any stick input ends the handover early - the pilot has taken over.
        if (sticksMoved()) {
            endLaunch(LAUNCH_WING_FLYING, LAUNCH_WING_EXIT_STICKS);
        } else if (launchWing.handover >= 1.0f) {
            endLaunch(LAUNCH_WING_FLYING, LAUNCH_WING_EXIT_HANDOVER);
        }
        break;

    case LAUNCH_WING_FLYING:
    case LAUNCH_WING_ABORTED:
        break;
    }

    if (launchWingIsActive()) {
        autopilotAngle[AI_ROLL] = 0.0f;
        // pitchTargetDeg is the user-facing climb angle, positive nose-up; the
        // angle target convention is positive nose-down.
        autopilotAngle[AI_PITCH] = -launchWing.pitchTargetDeg;
    }

    DEBUG_SET(DEBUG_LAUNCH, 0, launchWing.state);                            //!< Launch State [enum:launchWingState_e]
    DEBUG_SET(DEBUG_LAUNCH, 1, lrintf(launchWing.pitchTargetDeg * 10.0f));   //!< Climb Angle Target [unit:0.1deg]
    DEBUG_SET(DEBUG_LAUNCH, 2, lrintf(launchWing.throttle * 1000.0f));       //!< Launch Throttle [unit:0.001]
    DEBUG_SET(DEBUG_LAUNCH, 5, lrintf(launchWing.handover * 1000.0f));       //!< Pilot Handover [unit:0.001]
    DEBUG_SET(DEBUG_LAUNCH, 7, launchWing.exit);                             //!< Launch Exit Reason [enum:launchWingExit_e]
}

#endif // USE_WING && USE_LAUNCH_WING
