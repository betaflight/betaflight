/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/vector.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position_estimator.h"
#include "flight/position_filter.h"
#include "flight/position.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#ifdef USE_BARO
#include "sensors/barometer.h"
#endif

#ifdef USE_RANGEFINDER
#include "sensors/rangefinder.h"
#endif

#ifdef USE_OPTICALFLOW
#include "sensors/opticalflow.h"
#endif

#ifdef USE_GPS
#include "io/gps.h"
#endif

#if defined(USE_POSITION_HOLD)
#include "pg/autopilot.h"
#endif
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
#include "pg/pos_hold.h"
#endif

// DEBUG_POSITION_EST
// Slots 0, 1, 2 and 5 report the axis selected by gyro_filter_debug_axis:
// East by default, North when the axis is set to pitch.
// 0 - KF position cm
// 1 - KF velocity cm/s, including the ACCEL_VELOCITY_LEAD_TIME_XY lead on East and North
// 2 - KF acceleration cm/s^2
// 3 - optical flow East velocity cm/s   (written only while optical flow is fused)
// 4 - optical flow North velocity cm/s  (written only while optical flow is fused)
// 5 - earth-frame linear acceleration from the IMU cm/s^2, before the KF
// 6 - GPS position R cm^2               (written only while GPS XY is fused)
// 7 - GPS velocity R (cm/s)^2           (written only while GPS XY is fused)
// Slots 3, 4, 6 and 7 hold their last value when their source is not being fused.

// Constant-acceleration 3 state model tuning
// Q or Jerk (process noise): higher allows faster adaptations to data change / offset
// R or Measurement noise:  higher means less trust in that input

#define Q_JERK_XY       1500.0f  // lower gives smoother, damped acceleration but doesn't smooth  velocity much; below 500 attenuates output acceleration
#define R_ACCEL_XY      1500.0f  // higher reduces accel influence
#define R_GPS_VEL_BASE   200.0f  // increases as sAcc increases, higher allows more accel influence (not always good)
#define R_GPS_POS_BASE   160.0f   // cm^2 , increases as hAcc increases to allow more optical flow influence

#define R_OPTICALFLOW_VEL 400.0f   // (cm/s)^2 at max quality

#define Q_JERK_Z          3000.0f
#define R_ACCEL_Z          500.0f
#define R_BARO_ALT        150.0f   // cm^2 lower value favours baro data vs others, reduced by higher user prefer baro values
#define R_GPS_VEL_Z_BASE  100.0f   // (cm/s)^2, increases as sAcc increases to allow more baro / accelerometer influence
#define R_GPS_ALT_BASE    200.0f   // cm^2 , increases as vAcc increases to allow more baro / accel influence
#define R_RANGEFINDER_ALT 100.0f   // cm^2

#define ACCEL_VELOCITY_LEAD_TIME_XY      0.12f // seconds of Kalman acceleration added to Kalman velocity, as a lead compensator
#define GPS_VEL_ACCURACY_DENOM     300.0f   // sAcc mm/s: at or below this, use base velocity R values (same sAcc for all axes)
#define GPS_POS_XY_ACCURACY_DENOM 1000.0f  // hAcc mm: at or below this, use R_GPS_POS_BASE, increase when less accurate, up to 10x
#define GPS_ALT_ACCURACY_DENOM 1500.0f  // vAcc mm: at or below this, use R_GPS_ALT_BASE, increase when less accurate up to 10x
// Initial covariance values
#define INITIAL_POS_VAR     10000.0f    // cm^2  (1m uncertainty)
#define INITIAL_VEL_VAR     10000.0f    // (cm/s)^2
#define INITIAL_ACCEL_VAR   10000.0f  // (cm/s^2)^2; acquire IMU acceleration quickly

#define GRAVITY_CMSS        980.665f

// Each GPS fix is reused across the estimator steps that fall inside its
// source interval, so a slow receiver still corrects every step.

// Timeout: if no measurement for this long, mark invalid
#define MEASUREMENT_TIMEOUT_US  2000000  // 2 seconds

#define RANGEFINDER_MIN_ALT_CM  10

// Below RANGEFINDER_MIN_ALT_CM a reading is less trustworthy, but it is still a direct
// measurement of height above the ground, and it is the *only* trustworthy Z source
// down there - see GROUND_EFFECT_ALT_CM. Discarding it outright left the baro as the
// sole source at exactly the wrong moment. The driver reports RANGEFINDER_OUT_OF_RANGE
// (-1) when it has nothing, so any reading at or above zero is a real measurement.
#define RANGEFINDER_NEAR_FIELD_MIN_CM    0.0f
#define RANGEFINDER_NEAR_FIELD_R_SCALE   4.0f  // 2x the noise sigma; still far better than the baro

// Prop wash recirculating off the ground raises and destabilises static pressure, so
// close to the ground the baro is not measuring height at all. Logged on a descent to
// ~8 cm: the baro's relative altitude swung to -280 cm while the rangefinder sat
// steady at 8-11 cm, and it dragged the fused estimate to -59 cm and the vertical
// velocity to -47 cm/s. The step gate cannot catch this - the excursion is a fast ramp
// rather than a discontinuity, and no single sample came close to its allowance - so it
// has to be handled by trust instead.
//
// Measured baro error against the rangefinder, by height above ground:
//     >100 cm     median  +2 cm, worst  -23 cm
//     50-100 cm   median  +4 cm, worst  -29 cm
//     20-50 cm    median -25 cm, worst  -49 cm
//     <20 cm      median -13 cm, worst -280 cm
// so the onset is around 50 cm and it becomes severe below 20 cm.
#define GROUND_EFFECT_ALT_CM         50.0f  // baro starts losing validity here
#define GROUND_EFFECT_FULL_ALT_CM    20.0f  // and is worthless below here
#define GROUND_EFFECT_BARO_R_SCALE  100.0f  // R multiplier once fully in ground effect

// The rangefinder is what tells us we are in ground effect, so a dropout would otherwise
// switch the protection off at the moment it is most needed: lose the anchor and the
// knowledge simultaneously. Logged during a 140 ms dropout at 9 cm AGL, the baro was at
// -193 cm and got full trust, pulling the estimate to -29 cm. Rangefinders drop out near
// their minimum range, so this correlates with exactly the regime that needs covering -
// every invalid run in that flight began below 40 cm.
//
// So the last known height above ground is latched. A craft that was 9 cm up 140 ms ago
// cannot have left ground effect, and if it climbs out the rangefinder comes back in range
// and reports it directly.
//
// Releasing that latch has two separate jobs, and they want different timescales:
//
//   Hold  covers a blink. The sensor is momentarily unable to read, but everything we
//         know still applies, so the derate stays at full strength.
//   Fade  covers a real loss. The rangefinder is gone and the baro is all there is, so
//         the derate has to let go - a degraded baro altitude beats none, and with no
//         rangefinder the ground-effect question is not answerable anyway.
//
// A single fade conflates them. Sized at 500 ms from one observed 140 ms dropout, it was
// then met with a 421 ms dropout during a touchdown, which consumed 84% of the fade: the
// derate decayed 100x -> 17x and the estimate followed the baro from -23 cm to -52 cm in
// lockstep. Measured dropout durations were 18 to 20 ms in flight and up to 886 ms
// sitting on the ground with the motors running, so the hold clears the longest seen -
// but only just, and a longer one would start to fade. Shortening it needs new evidence.
#define GROUND_EFFECT_LATCH_HOLD_US  1000000  // full derate while the sensor blinks
#define GROUND_EFFECT_LATCH_FADE_US  1000000  // then release, for a rangefinder that is gone

// How close to the origin the Z estimate has to be for the craft to be treated as
// still on the ground. Above any multirotor's ground clearance, well below a baro
// excursion worth correcting. It shares a value with GROUND_EFFECT_ALT_CM by
// coincidence, not by derivation: this one is about the estimator's distance from its
// own origin, that one about height above the terrain. They are free to diverge.
#define Z_ON_GROUND_TOLERANCE_CM  50.0f

#ifdef USE_RANGEFINDER
// Valid rangefinder sample for Z fusion and optical-flow scaling (driver returns cm).
// minRangeCm is the caller's floor: Z fusion accepts near-field readings and derates
// them, while optical-flow scaling keeps the nominal minimum because a near-field
// height error scales straight into horizontal velocity.
static bool rangefinderSampleAltitudeCm(float *altCm, float maxRangeCm, float minRangeCm)
{
    if (!sensors(SENSOR_RANGEFINDER) || !rangefinderIsHealthy()) {
        return false;
    }
    const float alt = rangefinderGetLatestAltitude();
    if (alt < minRangeCm || alt > maxRangeCm) {
        return false;
    }
    *altCm = alt;
    return true;
}

// Whether the rangefinder is a Z source at all under the configured altitude_source.
static bool rangefinderFeedsZAltitude(void)
{
    const uint8_t altSource = positionConfig()->altitude_source;

    return altSource != ALTITUDE_SOURCE_GPS_ONLY &&
           altSource != ALTITUDE_SOURCE_BARO_ONLY;
}
#endif

#ifdef USE_GPS
static bool positionEstimatorGpsAltitudeAllowed(void)
{
    const uint8_t altSource = positionConfig()->altitude_source;

    return altSource == ALTITUDE_SOURCE_DEFAULT ||
           altSource == ALTITUDE_SOURCE_GPS_ONLY ||
           altSource == ALTITUDE_SOURCE_RANGEFINDER_PREFER;
}
#endif

// True when the Z position sources must also drive the velocity state, which is
// whenever nothing measures vertical velocity directly. GPS velned is the only
// direct vertical velocity measurement; baro and rangefinder report height alone.
//
// This is deliberately one answer for the whole axis rather than a per-sensor
// choice, and every Z position source must use it. Baro and rangefinder measure
// height against independent origins that drift relative to each other, so their
// innovations settle at non-zero values of opposite sign. Letting only one of them
// reach velocity rectifies that standing disagreement into an unbounded velocity
// bias: the position-only source repeatedly displaces the position state and only
// the pull-back is allowed to write velocity, so the correction accumulates in one
// direction. Measured on a hover with the baro drifting up at 10 cm/s against a
// steady rangefinder, the vertical velocity estimate ramped to -95 cm/s at
// -9.1 cm/s^2 while the craft did not move.
//
// Applying the same form to every Z position source cancels this exactly. The
// velocity-to-position gain ratio of kalmanUpdatePositionToVelocity() is
// P_vp / P_pp for every source, independent of its R, so the net velocity drive is
// that ratio times the weighted sum of the position corrections - and at the
// position fixed point that sum is zero by definition.
#if defined(USE_RANGEFINDER) || defined(USE_BARO)
static bool zPositionInnovationDrivesVelocity(void)
{
#ifdef USE_GPS
    if (positionEstimatorGpsAltitudeAllowed() &&
        sensors(SENSOR_GPS) &&
        STATE(GPS_FIX)) {
        return false;
    }
#endif

    return true;
}
#endif

// True while armed if horizontal fusion should run (POS_HOLD, rescue, GPS, and/or optical flow).
static bool positionEstimatorWantXYFusion(void)
{
    if (!ARMING_FLAG(ARMED)) {
        return false;
    }

#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        return true;
    }
#endif

#ifdef USE_GPS_RESCUE
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        return true;
    }
#endif

#ifdef USE_GPS
    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
        if (posHoldConfig()->positionSource != POSHOLD_SOURCE_OPTICALFLOW_ONLY) {
            return true;
        }
#else
        return true;
#endif
    }
#endif

#if defined(USE_OPTICALFLOW) && defined(USE_RANGEFINDER)
    if (sensors(SENSOR_OPTICALFLOW) && sensors(SENSOR_RANGEFINDER) &&
        isOpticalflowHealthy() && rangefinderIsHealthy()) {
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
        if (posHoldConfig()->positionSource != POSHOLD_SOURCE_GPS_ONLY) {
            return true;
        }
#else
        return true;
#endif
    }
#endif

    return false;
}

// Generic cross-calibration: correct drifting sensor offsets against the KF estimate
// whenever at least one non-drifting sensor is active.
//
// The correction is a first-order lag toward the ideal offset, so it nulls a static
// offset but leaves a standing error of driftRate * tau against a *drifting* one -
// and baro drift is a ramp, not a step. That standing error is a position
// disagreement between the two Z sources, so tau directly sets how much baro drift
// the estimate carries. A logged indoor hover drifted at 10 cm/s, which at the
// original 100 s time constant meant a 10 m standing error; the disagreement was
// still growing through 130 cm when the flight ended.
//
// The right tau depends on how good the anchor is, because a short tau slaves the
// drifting source to the anchor and the drifting source then stops contributing:
//
//   Rangefinder anchor  Low noise and absolute, and we want it to win outright, so
//                       re-zero the baro against it quickly. The point of fusing the
//                       baro here is dropout cover and high-rate response, not
//                       absolute reference, and a continuously re-zeroed baro hands
//                       over cleanly the moment the rangefinder goes out of range.
//   GPS anchor          Metre-level altitude noise, so the baro is the better
//                       short-term reference. Correct it slowly and let it keep
//                       smoothing GPS rather than being slaved to it.
//
// A genuine climb moves the anchor too, so kfPosition moves with the raw reading and
// the ideal offset barely changes: this loop only ever chases disagreement between
// the sources, never shared motion.
#define CROSS_CAL_TAU_RANGEFINDER_S   2.0f
#define CROSS_CAL_TAU_GPS_S          60.0f

typedef struct {
    float rawReading;   // only populated by drifting sources; unused unless offsetPtr is set
    float *offsetPtr;   // NULL for sources whose offset is not corrected
    float anchorTauS;   // cross-calibration time constant to use when this source anchors
    bool active;        // fed this cycle; non-drifting sources use this to anchor
    bool drifts;
} sensorCalEntry_t;

enum { CAL_Z_BARO = 0, CAL_Z_GPS, CAL_Z_RF, CAL_Z_COUNT };

// Step gating for the Z position sources (baro and rangefinder).
//
// Both report height with no vertical velocity, so they normally take the wider
// kalmanUpdatePositionToVelocity() form and let their position innovation correct
// velocity as well. The velocity gain on that path is several times the position
// gain - measured at ~3.4 cm/s per cm against ~0.6 cm per cm on a converged filter
// - so a discontinuity in the measurement is differentiated into a large phantom
// vertical velocity. A rangefinder losing and reacquiring lock over changing
// terrain, hitting rangefinder_max_range_cm and returning, or a baro spiking on
// prop wash all produce exactly that discontinuity.
//
// The obvious gate here would be a normalised innovation (chi-square) test on
// innovation^2 against S = P_pos + R. It does not work for this filter. Measured
// peak innovations, with the Z filter running on baro alone:
//
//     hover, +/-8 cm baro dither     1.8 sigma
//     1 m/s climb                    2.3 sigma
//     5 m/s climb                   11.9 sigma
//     10 m/s climb                  23.6 sigma
//     a 2 m position step           ~17 sigma
//
// A genuine fast climb produces a larger normalised innovation than the faults
// worth catching, so no threshold separates them. Innovation magnitude is simply
// not the discriminant: what makes a fault a fault is that it is a
// *discontinuity*. A climb, however fast, moves the reading by a bounded amount
// per unit time; a lock reacquire moves it hundreds of cm between one sample and
// the next.
//
// So the gate is a rate-of-change test on the sensor's own reading, bounded by
// what the airframe could physically have flown between two of that sensor's
// samples. That is independent of the filter's covariance, and independent of
// whether the accelerometer happens to be tracking - which matters, because an
// accelerometer fault is one of the things that could make the state diverge in
// the first place.
//
// A plain gate can lock out permanently, though. If the state has genuinely
// diverged, every future sample disagrees and is rejected forever. So persistent
// disagreement is read as evidence that our reference is stale rather than the
// sensor being wrong: after MAX_REJECTS consecutive rejections the sample is
// applied through the narrow kalmanUpdatePosition() form, which re-anchors
// position while still keeping the step out of velocity. That is precisely what
// the gate exists to prevent, so the escape hatch cannot reintroduce the spike it
// is protecting against.
//
// GPS altitude and optical flow are deliberately not gated. GPS altitude already
// uses the narrow position-only form, so it cannot drive velocity from a position
// step, and optical flow measures velocity directly rather than a position that
// could step.
#if defined(USE_BARO) || defined(USE_RANGEFINDER)
// Generous vs any real airframe, so the gate only ever catches the physically
// impossible: 40 m/s vertical, plus headroom for sensor noise and quantisation.
// The interval cap stops a slow or stalled sensor from opening the gate arbitrarily
// wide.
#define Z_MAX_VERTICAL_SPEED_CMS    4000.0f
#define Z_STEP_ALLOWANCE_CM           50.0f
#define Z_STEP_MAX_INTERVAL_S          0.2f
#define Z_STEP_GATE_MAX_REJECTS         10

typedef enum {
    Z_UPDATE_FULL = 0,        // in gate: fuse as the source normally would
    Z_UPDATE_POSITION_ONLY,   // gate held off too long: re-anchor position only
    Z_UPDATE_REJECT,
} zUpdateAction_e;

typedef struct {
    float lastAcceptedCm;     // last reading allowed through, relative to this source's offset
    bool hasReference;
    uint8_t consecutiveRejects;
} zStepGate_t;
#endif

// Independent 1-D Kalman filters, one per ENU axis (East, North, Up).
static positionKalman_t kfEast;
static positionKalman_t kfNorth;
static positionKalman_t kfUp;

static positionEstimate3d_t estimate;

static bool xyEnabled = false;
static float qJerkXY = Q_JERK_XY;
static timeUs_t lastXYMeasurementUs = 0;
static timeUs_t lastZMeasurementUs = 0;
static unsigned debugAxis; // 0 for East, 1 for North; selected by gyro_filter_debug_axis

static sensorCalEntry_t zCal[CAL_Z_COUNT];

#ifdef USE_GPS
static uint16_t gpsStamp = 0;
static bool gpsDataAvailable = false;
static timeUs_t gpsDataReceivedAtUs = 0;
static timeDelta_t gpsDataHoldDurationUs = 0;
static gpsLocation_t armLocationGps;
static bool gpsArmLocationSet = false;
static float gpsAltOffsetCm = 0.0f;
static bool gpsAltOffsetSet = false;
#endif

#ifdef USE_BARO
static float baroAltOffsetCm = 0.0f;
static bool baroOffsetSet = false;
static bool baroDataAvailable = false;
static timeUs_t baroTimestampUs = 0;
static timeDelta_t baroHoldDurationUs = 0;
static zStepGate_t baroStepGate;
#endif

#ifdef USE_RANGEFINDER
static float rangefinderAltOffsetCm = 0.0f;
static bool rangefinderOffsetSet = false;
static bool rangefinderDataAvailable = false;
static timeUs_t rangefinderTimestampUs = 0;
static timeDelta_t rangefinderHoldDurationUs = 0;
static zStepGate_t rangefinderStepGate;
#endif

#if defined(USE_BARO) && defined(USE_RANGEFINDER)
// Last height above ground the rangefinder reported, so the ground-effect derate
// survives a dropout. See GROUND_EFFECT_LATCH_US.
static float groundEffectAglCm = 0.0f;
static timeUs_t groundEffectAglTimeUs = 0;
static bool groundEffectAglValid = false;
#endif

#ifdef USE_OPTICALFLOW
static bool opticalFlowDataAvailable = false;
static timeUs_t opticalFlowTimestampUs = 0;
static timeDelta_t opticalFlowHoldDurationUs = 0;
#endif

static void initZCalEntries(void)
{
    for (int i = 0; i < CAL_Z_COUNT; i++) {
        zCal[i].rawReading = 0.0f;
        zCal[i].offsetPtr = NULL;
        zCal[i].anchorTauS = 0.0f;
        zCal[i].active = false;
        zCal[i].drifts = false;
    }
#ifdef USE_BARO
    zCal[CAL_Z_BARO].offsetPtr = &baroAltOffsetCm;
    zCal[CAL_Z_BARO].drifts = true;
#endif
#ifdef USE_GPS
    zCal[CAL_Z_GPS].anchorTauS = CROSS_CAL_TAU_GPS_S;
#endif
#ifdef USE_RANGEFINDER
    zCal[CAL_Z_RF].anchorTauS = CROSS_CAL_TAU_RANGEFINDER_S;
#endif
}

void positionEstimatorInit(void)
{

    kalmanInit(&kfEast, 0.0f, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, INITIAL_ACCEL_VAR, Q_JERK_XY);
    kalmanInit(&kfNorth, 0.0f, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, INITIAL_ACCEL_VAR, Q_JERK_XY);
    kalmanInit(&kfUp, 0.0f, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, INITIAL_ACCEL_VAR, Q_JERK_Z);
    estimate.position = (vector3_t){{0, 0, 0}};
    estimate.velocity = (vector3_t){{0, 0, 0}};
    estimate.acceleration = (vector3_t){{0, 0, 0}};
    estimate.trustXY = 0.0f;
    estimate.trustZ = 0.0f;
    estimate.isValidXY = false;
    estimate.isValidZ = false;

    xyEnabled = false;
    lastXYMeasurementUs = 0;
    lastZMeasurementUs = 0;
    debugAxis = (gyroConfig()->gyro_filter_debug_axis == FD_PITCH) ? 1 : 0; //  0 for East, 1 for North


#ifdef USE_GPS
    gpsStamp = 0;
    gpsDataAvailable = false;
    gpsDataReceivedAtUs = 0;
    gpsDataHoldDurationUs = 0;
    gpsArmLocationSet = false;
    gpsAltOffsetCm = 0.0f;
    gpsAltOffsetSet = false;
#endif
#ifdef USE_BARO
    baroAltOffsetCm = 0.0f;
    baroOffsetSet = false;
    baroDataAvailable = false;
    baroTimestampUs = 0;
    baroHoldDurationUs = 0;
    baroStepGate = (zStepGate_t){ 0 };
#endif
#ifdef USE_RANGEFINDER
    rangefinderAltOffsetCm = 0.0f;
    rangefinderOffsetSet = false;
    rangefinderDataAvailable = false;
    rangefinderTimestampUs = 0;
    rangefinderHoldDurationUs = 0;
    rangefinderStepGate = (zStepGate_t){ 0 };
#endif
#if defined(USE_BARO) && defined(USE_RANGEFINDER)
    groundEffectAglCm = 0.0f;
    groundEffectAglTimeUs = 0;
    groundEffectAglValid = false;
#endif
#ifdef USE_OPTICALFLOW
    opticalFlowDataAvailable = false;
    opticalFlowTimestampUs = 0;
    opticalFlowHoldDurationUs = 0;
#endif

    initZCalEntries();
}

// Discards the horizontal filter state without touching the GPS origin. The origin is
// what GPS Rescue and waypoint nav measure home against, and it has independent meaning
// that losing sight of the ground has no business redefining - see positionEstimatorResetXY()
// for the full reset that does move it.
static void resetXYFilterState(void)
{
    kalmanInit(&kfEast, 0.0f, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, INITIAL_ACCEL_VAR, qJerkXY);
    kalmanInit(&kfNorth, 0.0f, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, INITIAL_ACCEL_VAR, qJerkXY);
    estimate.position.v[ENU_E] = 0.0f;
    estimate.position.v[ENU_N] = 0.0f;
    estimate.velocity.v[ENU_E] = 0.0f;
    estimate.velocity.v[ENU_N] = 0.0f;
    estimate.acceleration.v[ENU_E] = 0.0f;
    estimate.acceleration.v[ENU_N] = 0.0f;
    estimate.isValidXY = false;
}

// True when nothing has measured horizontal position or velocity for longer than the
// timeout, or has never measured it at all this XY session.
static bool xyMeasurementsStale(timeUs_t nowUs)
{
    return lastXYMeasurementUs == 0 ||
           cmpTimeUs(nowUs, lastXYMeasurementUs) >= MEASUREMENT_TIMEOUT_US;
}

void positionEstimatorEnableXY(bool enable)
{
    if (enable && !xyEnabled) {
        kalmanInit(&kfEast, 0.0f, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, INITIAL_ACCEL_VAR, qJerkXY);
        kalmanInit(&kfNorth, 0.0f, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, INITIAL_ACCEL_VAR, qJerkXY);
        estimate.position.v[ENU_E] = 0.0f;
        estimate.position.v[ENU_N] = 0.0f;
        estimate.velocity.v[ENU_E] = 0.0f;
        estimate.velocity.v[ENU_N] = 0.0f;
        estimate.acceleration.v[ENU_E] = 0.0f;
        estimate.acceleration.v[ENU_N] = 0.0f;
        lastXYMeasurementUs = 0;
        estimate.isValidXY = false;
#ifdef USE_GPS
        // Clear before recapture: a stale origin from a prior XY session must
        // not survive into a new one, otherwise the late-capture path (see
        // positionEstimatorUpdate) will skip and we'd target waypoints
        // against the previous flight's baseline.
        gpsArmLocationSet = false;
        if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
            armLocationGps = gpsSol.llh;
            gpsArmLocationSet = true;
        }
#endif
    }
    xyEnabled = enable;
}

// Compute earth-frame linear acceleration from IMU (gravity removed), in cm/s^2 ENU
STATIC_UNIT_TESTED void getLinearAccelENU(float *accelEast, float *accelNorth, float *accelUp)
{
    const float accScale = acc.dev.acc_1G_rec;
    vector3_t accBF = {{ acc.accADC.x * accScale,
                         acc.accADC.y * accScale,
                         acc.accADC.z * accScale }};

    // rMat rotates body -> earth NWU (North-West-Up); this is the same convention
    // used throughout imu.c (imuComputeRotationMatrix, attitude extraction, mag fusion).
    vector3_t accEF_NWU;
    matrixVectorMul(&accEF_NWU, &rMat, &accBF);

    // NWU -> ENU named outputs. Indexing by NWU_W makes the East = -West sign
    // explicit; gravity (a steady +1 g on NWU_U) is removed from Up.
    *accelEast  = -accEF_NWU.v[NWU_W] * GRAVITY_CMSS;
    *accelNorth =  accEF_NWU.v[NWU_N] * GRAVITY_CMSS;
    *accelUp    = (accEF_NWU.v[NWU_U] - 1.0f) * GRAVITY_CMSS;
}

#ifdef USE_GPS
// Convert GPS accuracy estimate (mm or mm/s) to Kalman measurement noise (R) scaler
//  returns (cm)^2 or (cm/s)^2, constrained to the configured limits.
static float gpsAccuracyR(uint32_t accuracyMm, float accuracyDenom, float baseR, float maxR)
{
    const float scale = constrainf((float)accuracyMm / accuracyDenom, 1.0f, maxR / baseR);
    return baseR * scale;
}

STATIC_UNIT_TESTED bool gpsMeasurementReadyForFusion(timeUs_t nowUs)
{
    const bool hasNewData = gpsHasNewData(&gpsStamp);
    if (hasNewData) {
        gpsDataAvailable = true;
        gpsDataReceivedAtUs = nowUs;
        const float gpsFrequencyHz = getGpsDataFrequencyHz();

        if (gpsFrequencyHz > 0.0f) {
            gpsDataHoldDurationUs = (timeUs_t)lrintf(1000000.0f / gpsFrequencyHz);
        } else {
            // An invalid frequency cannot define a safe hold interval.
            gpsDataHoldDurationUs = 0;
        }
    }

    const bool withinHoldInterval = gpsDataAvailable &&
        (gpsDataHoldDurationUs > 0) &&
        (cmpTimeUs(nowUs, gpsDataReceivedAtUs) < gpsDataHoldDurationUs);

    return hasNewData || withinHoldInterval;
}
#endif

#ifdef USE_BARO
// A complete pressure-conversion cycle can be slower than the barometer task's
// scheduler rate. Use timestamps from valid altitude samples so conversion
// delays and target-specific barometer rates are both accounted for.
STATIC_UNIT_TESTED bool baroMeasurementReadyForFusion(timeUs_t nowUs)
{
    const timeUs_t sampleTimeUs = getBaroLatestSampleTimeUs();
    const bool hasNewData = !baroDataAvailable || sampleTimeUs != baroTimestampUs;
    if (hasNewData) {
        baroDataAvailable = true;
        baroTimestampUs = sampleTimeUs;
        baroHoldDurationUs = getBaroSampleIntervalUs();
    }

    const timeDelta_t sampleAgeUs = cmpTimeUs(nowUs, baroTimestampUs);
    const bool withinHoldInterval = baroDataAvailable &&
        baroHoldDurationUs > 0 &&
        sampleAgeUs >= 0 &&
        sampleAgeUs < baroHoldDurationUs;

    // A new sample with no known interval has no hold window to fall inside, so
    // accept it on arrival; otherwise it must sit within its interval.
    return (hasNewData && baroHoldDurationUs <= 0) || withinHoldInterval;
}
#endif

#ifdef USE_RANGEFINDER
// rangefinderProcess() timestamps only actual device samples, so its measured
// interval remains correct when a driver task polls faster than the hardware
// data rate (the UP-T1 polls at 100 Hz for a 50 Hz data stream, for example).
STATIC_UNIT_TESTED bool rangefinderMeasurementReadyForFusion(timeUs_t nowUs)
{
    const timeUs_t sampleTimeUs = rangefinderGetLatestSampleTimeUs();
    const bool hasNewData = !rangefinderDataAvailable || sampleTimeUs != rangefinderTimestampUs;
    if (hasNewData) {
        rangefinderDataAvailable = true;
        rangefinderTimestampUs = sampleTimeUs;
        rangefinderHoldDurationUs = rangefinderGetSampleIntervalUs();
    }
    const timeDelta_t sampleAgeUs = cmpTimeUs(nowUs, rangefinderTimestampUs);
    const bool withinHoldInterval = rangefinderDataAvailable &&
        rangefinderHoldDurationUs > 0 &&
        sampleAgeUs >= 0 &&
        sampleAgeUs < rangefinderHoldDurationUs;

    // As for the barometer: a new sample with no known interval is accepted on
    // arrival, otherwise it must sit within its interval.
    return (hasNewData && rangefinderHoldDurationUs <= 0) || withinHoldInterval;
}
#endif



#ifdef USE_OPTICALFLOW
static float opticalFlowR(int16_t quality)
{
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    const int minQuality = posHoldConfig()->opticalflowQualityMin;
#else
    const int minQuality = 30;
#endif
    if (quality <= minQuality) {
        return -1.0f;  // signal: do not use
    }
    // Scale R inversely with quality: better quality = lower noise
    const float qualityNorm = constrainf((float)(quality - minQuality) / (100.0f - minQuality), 0.01f, 1.0f);
    return R_OPTICALFLOW_VEL / qualityNorm;
}

// Optical-flow drivers expose the timestamp of the last processed sensor
// sample. Use its measured cadence after the first sample; delayMs is only the
// nominal first-sample fallback. This avoids mistaking a driver's faster
// polling rate (such as the UP-T1 rangefinder's 2x polling) for its data rate.
STATIC_UNIT_TESTED bool opticalFlowMeasurementReadyForFusion(timeUs_t nowUs, const opticalflow_t *flow)
{
    const bool hasNewData = !opticalFlowDataAvailable || flow->timeStampUs != opticalFlowTimestampUs;
    if (hasNewData) {
        timeDelta_t sourceIntervalUs = (timeDelta_t)flow->dev.delayMs * 1000;
        if (opticalFlowDataAvailable) {
            const timeDelta_t measuredIntervalUs = cmpTimeUs(flow->timeStampUs, opticalFlowTimestampUs);
            if (measuredIntervalUs > 0 && measuredIntervalUs <= OPTICALFLOW_HARDWARE_TIMEOUT_US) {
                sourceIntervalUs = measuredIntervalUs;
            }
        }

        opticalFlowDataAvailable = true;
        opticalFlowTimestampUs = flow->timeStampUs;
        opticalFlowHoldDurationUs = sourceIntervalUs;
    }

    const timeDelta_t sampleAgeUs = cmpTimeUs(nowUs, opticalFlowTimestampUs);
    const bool withinHoldInterval = opticalFlowDataAvailable &&
        opticalFlowHoldDurationUs > 0 &&
        sampleAgeUs >= 0 &&
        sampleAgeUs < opticalFlowHoldDurationUs;

    return hasNewData || withinHoldInterval;
}
#endif

static void feedGPSMeasurements(timeUs_t nowUs)
{
#ifdef USE_GPS
    if (!sensors(SENSOR_GPS) || !STATE(GPS_FIX)) {
        gpsDataAvailable = false;
        return;
    }
    if (!gpsMeasurementReadyForFusion(nowUs)) {
        return;
    }

#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    const uint8_t posSource = posHoldConfig()->positionSource;
    const bool gpsXYAllowed = (posSource != POSHOLD_SOURCE_OPTICALFLOW_ONLY);
#else
    const bool gpsXYAllowed = true;
#endif
    const bool gpsAltAllowed = positionEstimatorGpsAltitudeAllowed();

    if (xyEnabled && !gpsArmLocationSet && gpsXYAllowed) {
        armLocationGps = gpsSol.llh;
        gpsArmLocationSet = true;
    }

    if (xyEnabled && gpsXYAllowed && gpsArmLocationSet) {
        vector2_t gpsDistCm;
        GPS_distance2d(&armLocationGps, &gpsSol.llh, &gpsDistCm);

        const float rGpsPosMax = 10.0f * R_GPS_POS_BASE;
        const float rGpsPos = gpsAccuracyR(gpsSol.acc.hAcc, GPS_POS_XY_ACCURACY_DENOM, R_GPS_POS_BASE, rGpsPosMax);


        // GPS velocity (NED from UBX) -> ENU
        const float rGpsVelMax = 10.0f * R_GPS_VEL_BASE;
        const float rGpsVel = gpsAccuracyR(gpsSol.acc.sAcc, GPS_VEL_ACCURACY_DENOM, R_GPS_VEL_BASE, rGpsVelMax);

        DEBUG_SET(DEBUG_POSITION_EST, 6, lrintf(rGpsPos));
        DEBUG_SET(DEBUG_POSITION_EST, 7, lrintf(rGpsVel));


        kalmanUpdateVelocityToPosition(&kfEast, (float)gpsSol.velned.velE, rGpsVel);
        kalmanUpdateVelocityToPosition(&kfNorth, (float)gpsSol.velned.velN, rGpsVel);

        kalmanUpdatePosition(&kfEast, gpsDistCm.v[EF_EAST], rGpsPos);
        kalmanUpdatePosition(&kfNorth, gpsDistCm.v[EF_NORTH], rGpsPos);

        lastXYMeasurementUs = nowUs;
    }

    // Z velocity and altitude measurements
    if (gpsAltAllowed) {
        if (!gpsAltOffsetSet) {
            gpsAltOffsetCm = gpsSol.llh.altCm;

            gpsAltOffsetSet = true;
        }
        const float gpsVelZRMax = 10.0f * R_GPS_VEL_Z_BASE;
        const float gpsVelUpR = gpsAccuracyR(gpsSol.acc.sAcc, GPS_VEL_ACCURACY_DENOM, R_GPS_VEL_Z_BASE, gpsVelZRMax);
        const float gpsAltRMax = 10.0f * R_GPS_ALT_BASE;
        const float gpsAltR = gpsAccuracyR(gpsSol.acc.vAcc, GPS_ALT_ACCURACY_DENOM, R_GPS_ALT_BASE, gpsAltRMax);
        const float gpsVelocityUp = -(float)gpsSol.velned.velD;
        const float gpsRelativeAltCm = gpsSol.llh.altCm - gpsAltOffsetCm;

        DEBUG_SET(DEBUG_ALTITUDE, 2, lrintf(gpsRelativeAltCm));
        DEBUG_SET(DEBUG_ALTITUDE, 4, lrintf(gpsVelocityUp));

        kalmanUpdateVelocityToPosition(&kfUp, gpsVelocityUp, gpsVelUpR); // always update velocity and position from GPS velocity innovation
        kalmanUpdatePosition(&kfUp, gpsRelativeAltCm, gpsAltR); // always update position from GPS position innovation

        lastZMeasurementUs = nowUs;

        zCal[CAL_Z_GPS].active = true;
    }
#else
    UNUSED(nowUs);
#endif
}

#if defined(USE_BARO) || defined(USE_RANGEFINDER)
// sampleIntervalUs is the source's own nominal interval, which is what bounds how
// far the height could have moved between one reading and the next. Elapsed wall
// time is deliberately not used: a slow sensor is held and re-fused across several
// estimator steps, and conversely a long steady hover would otherwise accumulate a
// vast allowance and wave the fault straight through. A reading unchanged since the
// last accepted one always passes, so held samples need no special case.
static zUpdateAction_e gateZPositionStep(zStepGate_t *gate, float measuredPositionCm,
                                         timeDelta_t sampleIntervalUs)
{
    if (gate->hasReference) {
        const float intervalS = constrainf(sampleIntervalUs * 1e-6f, 0.0f, Z_STEP_MAX_INTERVAL_S);
        const float allowanceCm = Z_STEP_ALLOWANCE_CM + Z_MAX_VERTICAL_SPEED_CMS * intervalS;

        if (fabsf(measuredPositionCm - gate->lastAcceptedCm) > allowanceCm) {
            if (++gate->consecutiveRejects < Z_STEP_GATE_MAX_REJECTS) {
                return Z_UPDATE_REJECT;
            }

            // Held off long enough that our reference must be the stale one.
            gate->consecutiveRejects = 0;
            gate->lastAcceptedCm = measuredPositionCm;
            return Z_UPDATE_POSITION_ONLY;
        }
    }

    gate->consecutiveRejects = 0;
    gate->lastAcceptedCm = measuredPositionCm;
    gate->hasReference = true;

    return Z_UPDATE_FULL;
}
#endif

#if defined(USE_BARO) && defined(USE_RANGEFINDER)
// R multiplier for the baro, from how far into ground effect the rangefinder says we are.
// Only applies where the rangefinder is actually a Z source: if the user asked for
// baro-only altitude, distrusting the baro here would leave nothing driving Z at all.
//
// This deliberately works in raw height above ground - rangefinderSampleAltitudeCm() returns
// the driver's tilt-compensated distance to the terrain, before rangefinderAltOffsetCm is
// subtracted - and not in the estimator's frame. Ground effect is caused by prop wash off the
// physical ground, so the band below has to be measured against the terrain directly below.
// Substituting the frame-relative altitude that feedRangefinderMeasurements() fuses would be
// a bug: the two diverge across a frame rebase and over any terrain step, and only the raw
// value predicts whether the craft is actually sitting in its own wash. Being frame-free also
// means the latch cannot be poisoned by the offset seed or rebase running later in the cycle.
//
// Refreshes the ground-effect latch as a side effect. Its only caller sits after the baro's
// ready-for-fusion gate, so this runs at the baro's own rate rather than the estimator's -
// around 14 Hz against 100 Hz on the hardware this was measured on. The fade is computed
// from timestamps, so its shape is unaffected, but the latched sample can be up to one baro
// interval old when a dropout starts, which shortens the effective hold by that much. At
// ~70 ms against a 1 s hold that is immaterial; it would not be if the hold were much
// shorter or the baro much slower.
static float baroGroundEffectRScale(timeUs_t nowUs)
{
    if (!rangefinderFeedsZAltitude()) {
        return 1.0f;
    }

    float aglCm;
    float fade = 1.0f;

    if (rangefinderSampleAltitudeCm(&aglCm, positionConfig()->rangefinder_max_range_cm,
                                    RANGEFINDER_NEAR_FIELD_MIN_CM)) {
        groundEffectAglCm = aglCm;
        groundEffectAglTimeUs = nowUs;
        groundEffectAglValid = true;
    } else if (groundEffectAglValid) {
        // No reading this step, so hold the last height: at full strength through the
        // hold, then released across the fade.
        const timeDelta_t ageUs = cmpTimeUs(nowUs, groundEffectAglTimeUs);
        if (ageUs < 0 || ageUs >= GROUND_EFFECT_LATCH_HOLD_US + GROUND_EFFECT_LATCH_FADE_US) {
            groundEffectAglValid = false;
            return 1.0f;
        }
        aglCm = groundEffectAglCm;
        const timeDelta_t fadeAgeUs = ageUs - GROUND_EFFECT_LATCH_HOLD_US;
        if (fadeAgeUs > 0) {
            fade = 1.0f - (float)fadeAgeUs / (float)GROUND_EFFECT_LATCH_FADE_US;
        }
    } else {
        return 1.0f;  // never had a reading, so nothing to act on
    }

    // Interpolate trust (1/scale) rather than the scale itself, so the onset is gentle
    // through the top of the band and only turns severe near the bottom, where the
    // measured errors do the same.
    const float t = constrainf((aglCm - GROUND_EFFECT_FULL_ALT_CM) /
                               (GROUND_EFFECT_ALT_CM - GROUND_EFFECT_FULL_ALT_CM), 0.0f, 1.0f);
    const float minTrust = 1.0f / GROUND_EFFECT_BARO_R_SCALE;
    const float scale = 1.0f / (minTrust + t * (1.0f - minTrust));

    return 1.0f + (scale - 1.0f) * fade;
}
#endif

static void feedBaroMeasurements(timeUs_t nowUs)
{
#ifdef USE_BARO
    if (!sensors(SENSOR_BARO)) {
        baroDataAvailable = false;
        return;
    }

    const uint8_t altSource = positionConfig()->altitude_source;
    if (altSource == ALTITUDE_SOURCE_GPS_ONLY ||
        altSource == ALTITUDE_SOURCE_RANGEFINDER_ONLY) {
        return;
    }

    const float baroAltCm = getBaroAltitude();

    if (!baroMeasurementReadyForFusion(nowUs)) {
        return;
    }

    if (!baroOffsetSet) {
        // Capture disarmed baseline once; keep live relative altitude while disarmed.
        baroAltOffsetCm = baroAltCm;
        baroOffsetSet = true;
    }

    // Scale R based on altitude_prefer_baro:
    // 100 = normal trust, 50 = 2x R, 20 = 5x R, 10 or below = 10x R.
    const float baroTrust = constrainf(positionConfig()->altitude_prefer_baro * 0.01f, 0.1f, 1.0f);
    float baroR = R_BARO_ALT / baroTrust;

#ifdef USE_RANGEFINDER
    // Static pressure is not a height measurement inside ground effect.
    baroR *= baroGroundEffectRScale(nowUs);
#endif

    const float baroRelativeAltCm = baroAltCm - baroAltOffsetCm;

    // Logged before the gate, so a rejected run is still visible against the estimate.
    DEBUG_SET(DEBUG_ALTITUDE, 1, lrintf(baroRelativeAltCm));

    const zUpdateAction_e action = gateZPositionStep(&baroStepGate, baroRelativeAltCm,
                                                    getBaroSampleIntervalUs());
    if (action == Z_UPDATE_REJECT) {
        // Not a measurement, so it neither refreshes the Z validity timeout nor
        // anchors the cross-calibration.
        return;
    }

    if (action == Z_UPDATE_FULL && zPositionInnovationDrivesVelocity()) {
        kalmanUpdatePositionToVelocity(&kfUp, baroRelativeAltCm, baroR);
    } else {
        kalmanUpdatePosition(&kfUp, baroRelativeAltCm, baroR);
    }

    lastZMeasurementUs = nowUs;

    zCal[CAL_Z_BARO].rawReading = baroAltCm;
    zCal[CAL_Z_BARO].active = true;
#else
    UNUSED(nowUs);
#endif
}

#ifdef USE_RANGEFINDER
// A rebase shifts every absolute altitude reading at once. Anything holding an
// absolute altitude target captured that target in the old frame and would chase the
// shift as though it were real vertical movement, so the rebase has to wait until no
// such consumer is active. Altitude hold is relative - it holds whatever height it
// was engaged at - so declining to rebase costs only the absolute-altitude consumers
// (OSD readout, landing detection, rescue altitudes), never stability.
static bool zFrameMayBeRebased(void)
{
    return !FLIGHT_MODE(ALT_HOLD_MODE | POS_HOLD_MODE | GPS_RESCUE_MODE | AUTOPILOT_MODE);
}

// Redefine the Z origin so the estimate reads newPositionCm, without disturbing any
// source's innovation or the velocity state.
static void rebaseZFrame(float newPositionCm)
{
    const float deltaCm = newPositionCm - kalmanGetPosition(&kfUp);

    kalmanShiftPosition(&kfUp, deltaCm);

    // Relative altitude for the other Z sources is (raw - offset), so lowering each
    // offset by the same delta carries that source's measurement along with the frame
    // and leaves its innovation exactly as it was. Without this the baro would drag
    // the state straight back until cross-calibration caught up. The step gate holds
    // its reference in relative terms, so it moves with the measurement.
#ifdef USE_BARO
    baroAltOffsetCm -= deltaCm;
    baroStepGate.lastAcceptedCm += deltaCm;
#endif
#ifdef USE_GPS
    gpsAltOffsetCm -= deltaCm;
#endif
}
#endif

static void feedRangefinderMeasurements(timeUs_t nowUs)
{
#ifdef USE_RANGEFINDER
    const uint8_t altSource = positionConfig()->altitude_source;
    const bool rangefinderFeedsZ = rangefinderFeedsZAltitude();
    float altCm;
    if (!rangefinderSampleAltitudeCm(&altCm, positionConfig()->rangefinder_max_range_cm,
                                     RANGEFINDER_NEAR_FIELD_MIN_CM)) {
        rangefinderDataAvailable = false;
        return;
    }

    if (!rangefinderMeasurementReadyForFusion(nowUs)) {
        return;
    }

    if (!rangefinderOffsetSet) {
        // The rangefinder measures distance to the ground, not height above the
        // estimator origin, so the offset stands for the ground height beneath us.
        //
        // On the ground the raw reading is the craft's ground clearance and the origin
        // is that same ground, so the offset is the reading itself and relative
        // altitude correctly starts at zero. "On the ground" is read off the estimate
        // rather than the arming flag, because a craft can sit armed on the ground for
        // some time: while the estimate still says we are at the origin, the two
        // candidate seeds below agree to within that ground clearance anyway.
        //
        // Once the estimate has left the origin, neither obvious seed works.
        // RANGEFINDER_MIN_ALT_CM means the first valid sample can arrive well after
        // takeoff. Taking the raw reading there declares us back at the origin.
        // Anchoring to the estimate inherits whatever that estimate is worth - and
        // until the rangefinder locks, the only Z source is usually the baro, which
        // spikes on prop wash at throttle-up: a logged takeoff put the baro 4 m high
        // inside 0.8 s, the seed landed in the middle of it, and the whole altitude
        // frame was 2.6 m out for the rest of the flight.
        //
        // So airborne we trust the rangefinder over the estimate: offset zero, so
        // relative altitude is AGL directly. Whenever the rangefinder is in range its
        // AGL reading is the best absolute height available, and if the estimate was
        // in fact correct the rebase below is simply a no-op. The cost is the
        // flat-ground assumption, which a rangefinder-referenced altitude makes
        // regardless.
        const bool estimateAtOrigin = fabsf(kalmanGetPosition(&kfUp)) < Z_ON_GROUND_TOLERANCE_CM;

        if (!ARMING_FLAG(ARMED) || !estimate.isValidZ || estimateAtOrigin) {
            rangefinderAltOffsetCm = altCm;
        } else if (rangefinderFeedsZ && zFrameMayBeRebased()) {
            // Rebase onto the AGL frame rather than letting an innovation drag the
            // state there, which would differentiate into a phantom vertical velocity.
            rangefinderAltOffsetCm = 0.0f;
            rebaseZFrame(altCm);
        } else {
            // Rebasing was declined, so keep the frame as it stands and absorb the
            // difference into the offset - the ideal-offset form crossCalibrateOffsets()
            // already uses. The first innovation is zero, so there is no velocity
            // spike, and altitude hold is relative: an offset frame still holds the
            // correct physical height. Only the absolute-altitude consumers lose out.
            rangefinderAltOffsetCm = altCm - kalmanGetPosition(&kfUp);
        }
        rangefinderOffsetSet = true;
    }

    const float RangeFinderAltitude = altCm - rangefinderAltOffsetCm;

    DEBUG_SET(DEBUG_ALTITUDE, 0, lrintf(RangeFinderAltitude));

    // Logged above this point, so the rangefinder's altitude stays visible in
    // DEBUG_ALTITUDE even under a source that does not fuse it.
    if (!rangefinderFeedsZ) {
        return;
    }

    // Rangefinder has low noise; give it very low R when source prefers it
    float rfR = R_RANGEFINDER_ALT;
    if (altSource == ALTITUDE_SOURCE_RANGEFINDER_ONLY ||
        altSource == ALTITUDE_SOURCE_RANGEFINDER_PREFER) {
        rfR *= 0.25f;  // even lower noise -> stronger pull
    }

    if (altCm < RANGEFINDER_MIN_ALT_CM) {
        // Near field: derated rather than discarded. This is where the baro is at its
        // worst, so a noisier rangefinder is still by far the best anchor available.
        rfR *= RANGEFINDER_NEAR_FIELD_R_SCALE;
    }


    // The offset seed above establishes this gate's first reference, so the first
    // sample always passes. The gate covers the steps the seed cannot: lock lost
    // and reacquired over different terrain, or a return from beyond max range.
    const zUpdateAction_e action = gateZPositionStep(&rangefinderStepGate, RangeFinderAltitude,
                                                    rangefinderGetSampleIntervalUs());
    if (action == Z_UPDATE_REJECT) {
        return;
    }

    if (action == Z_UPDATE_FULL && zPositionInnovationDrivesVelocity()) {
        kalmanUpdatePositionToVelocity(&kfUp, RangeFinderAltitude, rfR);
    } else {
        kalmanUpdatePosition(&kfUp, RangeFinderAltitude, rfR);
    }

    zCal[CAL_Z_RF].active = true;
    lastZMeasurementUs = nowUs;

#else
    UNUSED(nowUs);
#endif
}

static void feedOpticalFlowMeasurements(timeUs_t nowUs)
{
#ifdef USE_OPTICALFLOW
    if (!xyEnabled) {
        return;
    }

    if (!sensors(SENSOR_OPTICALFLOW) || !isOpticalflowHealthy()) {
        opticalFlowDataAvailable = false;
        return;
    }

#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    const uint8_t posSource = posHoldConfig()->positionSource;
    if (posSource == POSHOLD_SOURCE_GPS_ONLY) {
        return;
    }
#endif

#ifdef USE_RANGEFINDER
    float maxRangeCm = positionConfig()->rangefinder_max_range_cm;
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    maxRangeCm = fminf(maxRangeCm, (float)posHoldConfig()->opticalflowMaxRange);
#endif
    float altitudeCmF;
    // Nominal minimum, not the near-field floor the Z path uses: flow is scaled by this
    // height, so a near-field height error would land straight in horizontal velocity.
    if (!rangefinderSampleAltitudeCm(&altitudeCmF, maxRangeCm, RANGEFINDER_MIN_ALT_CM)) {
        return;
    }
    const float altitudeCm = altitudeCmF;
#else
    UNUSED(altSource);
    return;  // Optical flow requires rangefinder for altitude scaling
#endif

    const opticalflow_t *flow = getOpticalFlowData();
    if (flow == NULL) {
        return;
    }

    const float flowR = opticalFlowR(flow->quality);
    if (flowR < 0.0f) {
        return;  // quality too low
    }

    // Called after ReadyForFusion so its sample bookkeeping still advances: this
    // sample is being discarded, not deferred.
    if (!opticalFlowMeasurementReadyForFusion(nowUs, flow)) {
        return;
    }

    // Both body axes are needed even for one earth axis, because the pair is
    // rotated into ENU by heading. So an axis the rotation compensation could not
    // resolve costs the whole sample; coasting on the accelerometer for one
    // interval beats fusing a rate that is not a measurement of anything.
    if (!flow->flowRateValid[0] || !flow->flowRateValid[1]) {
        return;
    }

    // Convert flow rates (rad/s) to velocity (cm/s) in body frame, scaled by rangefinder height.
    // Flow sensor X axis measures rightward motion; Y axis measures forward motion.
    const float flowRight   = flow->processedFlowRates.x * altitudeCm;
    const float flowForward = flow->processedFlowRates.y * altitudeCm;

    // Project to the horizontal plane by removing the tilt-induced component.
    const float cosPitch = cos_approx(DECIDEGREES_TO_RADIANS(attitude.values.pitch));
    const float cosRoll  = cos_approx(DECIDEGREES_TO_RADIANS(attitude.values.roll));
    const float velRight   = flowRight   * cosRoll;
    const float velForward = flowForward * cosPitch;

    // Rotate from body heading frame to ENU earth frame.
    const float yawRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    const float cosYaw = cos_approx(yawRad);
    const float sinYaw = sin_approx(yawRad);
    const float velEast  =  velForward * sinYaw - velRight * cosYaw;
    const float velNorth =  velForward * cosYaw + velRight * sinYaw;

    kalmanUpdateVelocityToPosition(&kfEast, velEast, flowR);
    kalmanUpdateVelocityToPosition(&kfNorth, velNorth, flowR);

    DEBUG_SET(DEBUG_POSITION_EST, 3, lrintf(velEast));
    DEBUG_SET(DEBUG_POSITION_EST, 4, lrintf(velNorth));

    lastXYMeasurementUs = nowUs;
#else
    UNUSED(nowUs);
#endif
}

static void crossCalibrateOffsets(sensorCalEntry_t *sources, int count, float kfPosition, float dt)
{
    // Fastest active anchor sets the rate: a rangefinder in range should re-zero the
    // baro at its own rate even when GPS is anchoring at the same time.
    float tauS = 0.0f;
    for (int i = 0; i < count; i++) {
        if (sources[i].active && !sources[i].drifts && sources[i].anchorTauS > 0.0f) {
            if (tauS == 0.0f || sources[i].anchorTauS < tauS) {
                tauS = sources[i].anchorTauS;
            }
        }
    }

    const float alpha = (tauS > 0.0f) ? constrainf(dt / tauS, 0.0f, 1.0f) : 0.0f;

    for (int i = 0; i < count; i++) {
        if (alpha > 0.0f && ARMING_FLAG(ARMED) &&
            sources[i].active && sources[i].drifts && sources[i].offsetPtr) {
            const float idealOffset = sources[i].rawReading - kfPosition;
            *sources[i].offsetPtr += alpha * (idealOffset - *sources[i].offsetPtr);
        }
        sources[i].active = false;
    }
}

void positionEstimatorUpdate(void)
{
    const timeUs_t nowUs = micros();
    const float dt = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

    const bool wantXY = positionEstimatorWantXYFusion();
    if (wantXY != xyEnabled) {
        positionEstimatorEnableXY(wantXY);
    }

    // Compute earth-frame linear acceleration from IMU
    float accelEast, accelNorth, accelUp;
    getLinearAccelENU(&accelEast, &accelNorth, &accelUp);

    const float accelToLog = (debugAxis == 0) ? accelEast : accelNorth;
    DEBUG_SET(DEBUG_POSITION_EST, 5, lrintf(accelToLog));

    // Z-axis: always runs (for altitude hold, OSD, vario). While disarmed,
    // measure zero acceleration so covariance continues to evolve without
    // integrating small gravity-removal errors.

    kalmanUpdateAcceleration(&kfUp, ARMING_FLAG(ARMED) ? accelUp : 0.0f, R_ACCEL_Z);
    kalmanPredict(&kfUp, dt);

    // XY axes: only when a consumer is active
    if (xyEnabled && ARMING_FLAG(ARMED)) {
        if (xyMeasurementsStale(nowUs)) {
            // Nothing has anchored the horizontal state for longer than the timeout, which
            // makes it worthless rather than merely stale: with no measurement of either
            // position or velocity, both are an open-loop double integral of accelerometer
            // error. So hold it discarded instead of extending it.
            //
            // Reconciling it on reacquisition is not enough, because optical flow measures
            // velocity alone and the velocity gain is Pvv / (Pvv + R). Pvv grows to only
            // the order of the flow R, so the first sample moves velocity roughly half way
            // and leaves the remainder for the controller to fly out. Logged across a 22 s
            // flow outage that remainder was several m/s of phantom velocity, and because
            // the position controller's D term acts on the velocity estimate directly, the
            // first sample after the rangefinder came back into range commanded a
            // maximum-lean brake against motion that was not happening.
            //
            // Discarded here rather than in positionEstimatorResetXY() so the GPS origin
            // survives: home must not move because we lost sight of the ground.
            resetXYFilterState();
        } else {
            kalmanUpdateAcceleration(&kfEast, accelEast, R_ACCEL_XY);
            kalmanUpdateAcceleration(&kfNorth, accelNorth, R_ACCEL_XY);
            kalmanPredict(&kfEast, dt);
            kalmanPredict(&kfNorth, dt);
        }
    }

    // Feed sensor measurements (order does not matter)
    feedGPSMeasurements(nowUs);
    feedBaroMeasurements(nowUs);
    feedRangefinderMeasurements(nowUs);
    feedOpticalFlowMeasurements(nowUs);

    // Calibrate drifting sensor offsets against KF estimate anchored by non-drifting sources
    crossCalibrateOffsets(zCal, CAL_Z_COUNT, kalmanGetPosition(&kfUp), dt);

    // Extract state into the unified estimate (kfEast/kfNorth/kfUp are the East/North/Up filters)
    estimate.position.v[ENU_E] = kalmanGetPosition(&kfEast);
    estimate.position.v[ENU_N] = kalmanGetPosition(&kfNorth);
    estimate.position.v[ENU_U] = kalmanGetPosition(&kfUp);

    // XY velocity carries a lead term; Up does not, as the vertical loop has its
    // own acceleration feedforward in altitudeControl().
    estimate.velocity.v[ENU_E] = kalmanGetVelocity(&kfEast) + ACCEL_VELOCITY_LEAD_TIME_XY * kalmanGetAcceleration(&kfEast);
    estimate.velocity.v[ENU_N] = kalmanGetVelocity(&kfNorth) + ACCEL_VELOCITY_LEAD_TIME_XY * kalmanGetAcceleration(&kfNorth);
    estimate.velocity.v[ENU_U] = kalmanGetVelocity(&kfUp);

    estimate.acceleration.v[ENU_E] = kalmanGetAcceleration(&kfEast);
    estimate.acceleration.v[ENU_N] = kalmanGetAcceleration(&kfNorth);
    estimate.acceleration.v[ENU_U] = kalmanGetAcceleration(&kfUp);

    DEBUG_SET(DEBUG_POSITION_EST, 0, lrintf(estimate.position.v[debugAxis]));
    DEBUG_SET(DEBUG_POSITION_EST, 1, lrintf(estimate.velocity.v[debugAxis]));
    DEBUG_SET(DEBUG_POSITION_EST, 2, lrintf(estimate.acceleration.v[debugAxis]));

    DEBUG_SET(DEBUG_ALTITUDE, 6, lrintf(accelUp));

    // Validity: based on recent measurement updates
    if (xyEnabled) {
        estimate.isValidXY = (lastXYMeasurementUs > 0) &&
                             (cmpTimeUs(nowUs, lastXYMeasurementUs) < MEASUREMENT_TIMEOUT_US);
    } else {
        estimate.isValidXY = false;
    }
    estimate.isValidZ = (lastZMeasurementUs > 0) &&
                        (cmpTimeUs(nowUs, lastZMeasurementUs) < MEASUREMENT_TIMEOUT_US);

    // Trust: derived from position covariance (lower variance = higher trust)
    // Map variance to 0-1: trust = 1 / (1 + variance/scale)
    const float xyVar = (kalmanGetPositionVariance(&kfEast) + kalmanGetPositionVariance(&kfNorth)) * 0.5f;
    estimate.trustXY = 1.0f / (1.0f + xyVar / 10000.0f);
    estimate.trustZ = 1.0f / (1.0f + kalmanGetPositionVariance(&kfUp) / 10000.0f);
}

const positionEstimate3d_t *positionEstimatorGetEstimate(void)
{
    return &estimate;
}

float positionEstimatorGetAltitudeCm(void)
{
    return estimate.position.v[ENU_U];
}

float positionEstimatorGetVerticalVelocity(void)
{
    return estimate.velocity.v[ENU_U];
}

float positionEstimatorGetVerticalAcceleration(void)
{
    return estimate.acceleration.v[ENU_U];
}

bool positionEstimatorIsValidXY(void)
{
    return estimate.isValidXY;
}

bool positionEstimatorIsValidZ(void)
{
    return estimate.isValidZ;
}

float positionEstimatorGetTrustZ(void)
{
    return estimate.trustZ;
}

float positionEstimatorGetTrustXY(void)
{
    return estimate.trustXY;
}

bool positionEstimatorIsHeadingRequired(void)
{
    // If configured for optical flow only, heading is not required
#if defined(USE_GPS) && defined(USE_POSITION_HOLD) && !defined(USE_WING)
    if (posHoldConfig()->positionSource == POSHOLD_SOURCE_OPTICALFLOW_ONLY) {
        return false;
    }
#endif

    // If optical flow is active and spatial tracking is healthy, heading is optional
    if (sensors(SENSOR_OPTICALFLOW) && positionEstimatorIsValidXY()) {
        return false;
    }

    // Otherwise we strictly require a valid heading to prevent a flyaway
#ifdef USE_GPS
    return sensors(SENSOR_GPS) && STATE(GPS_FIX);
#else
    return false;
#endif
}

void positionEstimatorResetZ(void)
{
    kalmanInit(&kfUp, 0.0f, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, INITIAL_ACCEL_VAR, Q_JERK_Z);
    estimate.position.v[ENU_U] = 0.0f;
    estimate.velocity.v[ENU_U] = 0.0f;
    estimate.acceleration.v[ENU_U] = 0.0f;
    estimate.isValidZ = false;
    lastZMeasurementUs = 0;
#ifdef USE_GPS
    gpsAltOffsetCm = 0.0f;
    gpsAltOffsetSet = false;
#endif
    // Each source's offset is cleared, which redefines the relative altitude it reports, so
    // its step gate has to go with it: a reference measured in the old frame would gate the
    // new one against a height that no longer means anything.
#ifdef USE_BARO
    baroAltOffsetCm = 0.0f;
    baroOffsetSet = false;
    baroStepGate = (zStepGate_t){ 0 };
#endif
#ifdef USE_RANGEFINDER
    rangefinderAltOffsetCm = 0.0f;
    rangefinderOffsetSet = false;
    rangefinderStepGate = (zStepGate_t){ 0 };
#endif
#if defined(USE_BARO) && defined(USE_RANGEFINDER)
    // Dropped rather than carried: while Z re-converges the baro is what re-establishes
    // altitude, so it must not be held at a stale derate. A live rangefinder refreshes
    // this on the next step anyway.
    groundEffectAglValid = false;
#endif

    initZCalEntries();
}

void positionEstimatorResetXY(void)
{
    resetXYFilterState();
    lastXYMeasurementUs = 0;
#ifdef USE_GPS
    gpsArmLocationSet = false;
    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
        armLocationGps = gpsSol.llh;
        gpsArmLocationSet = true;
    }
#endif
}

bool positionEstimatorGetGpsOrigin(gpsLocation_t *out)
{
#ifdef USE_GPS
    if (!gpsArmLocationSet || out == NULL) {
        return false;
    }
    *out = armLocationGps;
    return true;
#else
    UNUSED(out);
    return false;
#endif
}
