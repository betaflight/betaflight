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

#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
#include "pg/pos_hold.h"
#endif

// Accelerometer process noise in (cm/s^2)^2.
// Accounts for vibration, bias drift, attitude errors.
// Higher = less trust in accel dead-reckoning, more reliance on sensor corrections.
#define Q_ACCEL_XY          50000.0f
#define Q_ACCEL_Z           20000.0f

// Initial covariance values
#define INITIAL_POS_VAR     10000.0f    // cm^2  (1m uncertainty)
#define INITIAL_VEL_VAR     10000.0f    // (cm/s)^2

// Measurement noise base values (R)
#define R_GPS_POS_BASE      10000.0f    // cm^2 at pDOP=1.0
#define R_GPS_VEL_BASE      2500.0f     // (cm/s)^2 at pDOP=1.0
#define R_GPS_ALT_BASE      40000.0f    // cm^2 at pDOP=1.0
#define R_BARO_ALT          2500.0f     // cm^2
#define R_RANGEFINDER_ALT   100.0f      // cm^2
#define R_OPTICALFLOW_VEL   400.0f      // (cm/s)^2 at max quality

#define GRAVITY_CMSS        980.665f

// Timeout: if no measurement for this long, mark invalid
#define MEASUREMENT_TIMEOUT_US  2000000  // 2 seconds

#define RANGEFINDER_MIN_ALT_CM  10

#ifdef USE_RANGEFINDER
// Valid rangefinder sample for Z fusion and optical-flow scaling (driver returns cm).
static bool rangefinderSampleAltitudeCm(float *altCm, float maxRangeCm)
{
    if (!sensors(SENSOR_RANGEFINDER) || !rangefinderIsHealthy()) {
        return false;
    }
    const float alt = rangefinderGetLatestAltitude();
    if (alt < RANGEFINDER_MIN_ALT_CM || alt > maxRangeCm) {
        return false;
    }
    *altCm = alt;
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
// whenever at least one non-drifting sensor is active.  The KF dynamics naturally
// scale the effective correction rate (fast when rangefinder anchors, slow when
// only GPS is available) so a single alpha suffices.
#define CROSS_CAL_ALPHA  0.005f

typedef struct {
    float rawReading;
    float *offsetPtr;
    bool active;
    bool drifts;
} sensorCalEntry_t;

enum { CAL_Z_BARO = 0, CAL_Z_GPS, CAL_Z_RF, CAL_Z_COUNT };

static positionKalman_t kfX;
static positionKalman_t kfY;
static positionKalman_t kfZ;

static positionEstimate3d_t estimate;

static bool xyEnabled = false;

static timeUs_t lastXYMeasurementUs = 0;
static timeUs_t lastZMeasurementUs = 0;

static sensorCalEntry_t zCal[CAL_Z_COUNT];

#ifdef USE_GPS
static uint16_t gpsStamp = 0;
static gpsLocation_t armLocationGps;
static bool gpsArmLocationSet = false;
static float gpsAltOffsetCm = 0.0f;
static bool gpsAltOffsetSet = false;
#endif

#ifdef USE_BARO
static float baroAltOffsetCm = 0.0f;
static float baroAltAccumulator = 0.0f;
static bool baroOffsetSet = false;
#endif

#ifdef USE_RANGEFINDER
static float rangefinderAltOffsetCm = 0.0f;
static bool rangefinderOffsetSet = false;
#endif

static void initZCalEntries(void)
{
    for (int i = 0; i < CAL_Z_COUNT; i++) {
        zCal[i].rawReading = 0.0f;
        zCal[i].offsetPtr = NULL;
        zCal[i].active = false;
        zCal[i].drifts = false;
    }
#ifdef USE_BARO
    zCal[CAL_Z_BARO].offsetPtr = &baroAltOffsetCm;
    zCal[CAL_Z_BARO].drifts = true;
#endif
}

void positionEstimatorInit(void)
{
    kalmanInit(&kfX, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_XY);
    kalmanInit(&kfY, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_XY);
    kalmanInit(&kfZ, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_Z);

    estimate.position = (vector3_t){{0, 0, 0}};
    estimate.velocity = (vector3_t){{0, 0, 0}};
    estimate.trustXY = 0.0f;
    estimate.trustZ = 0.0f;
    estimate.isValidXY = false;
    estimate.isValidZ = false;

    xyEnabled = false;
    lastXYMeasurementUs = 0;
    lastZMeasurementUs = 0;

#ifdef USE_GPS
    gpsStamp = 0;
    gpsArmLocationSet = false;
    gpsAltOffsetCm = 0.0f;
    gpsAltOffsetSet = false;
#endif
#ifdef USE_BARO
    baroAltOffsetCm = 0.0f;
    baroAltAccumulator = 0.0f;
    baroOffsetSet = false;
#endif
#ifdef USE_RANGEFINDER
    rangefinderAltOffsetCm = 0.0f;
    rangefinderOffsetSet = false;
#endif

    initZCalEntries();
}

void positionEstimatorEnableXY(bool enable)
{
    if (enable && !xyEnabled) {
        kalmanInit(&kfX, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_XY);
        kalmanInit(&kfY, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_XY);
        estimate.position.x = 0.0f;
        estimate.position.y = 0.0f;
        estimate.velocity.x = 0.0f;
        estimate.velocity.y = 0.0f;
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
static void getLinearAccelENU(float *accelEast, float *accelNorth, float *accelUp)
{
    const float accScale = acc.dev.acc_1G_rec;
    vector3_t accBF = {{ acc.accADC.x * accScale,
                         acc.accADC.y * accScale,
                         acc.accADC.z * accScale }};

    // rMat rotates body -> earth NED
    vector3_t accEF_NED;
    matrixVectorMul(&accEF_NED, &rMat, &accBF);

    // NED -> ENU, subtract gravity (NED gravity = [0,0,+1g]), convert G -> cm/s^2
    *accelEast  =  accEF_NED.y * GRAVITY_CMSS;
    *accelNorth =  accEF_NED.x * GRAVITY_CMSS;
    *accelUp    = -(accEF_NED.z - 1.0f) * GRAVITY_CMSS;
}

#ifdef USE_GPS
// GPS measurement noise scaled by pDOP
static float gpsR(float baseR)
{
    float pdop = gpsSol.dop.pdop * 0.01f;  // pDOP * 100 stored, so /100 to real value
    if (pdop < 1.0f) {
        pdop = 1.0f;
    }
    return baseR * pdop * pdop;
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
#endif

static void feedGPSMeasurements(timeUs_t nowUs)
{
#ifdef USE_GPS
    if (!sensors(SENSOR_GPS) || !STATE(GPS_FIX)) {
        return;
    }

    if (!gpsHasNewData(&gpsStamp)) {
        return;
    }

    // Determine which measurements are allowed by source settings
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    const uint8_t posSource = posHoldConfig()->positionSource;
    const bool gpsXYAllowed = (posSource != POSHOLD_SOURCE_OPTICALFLOW_ONLY);
#else
    const bool gpsXYAllowed = true;
#endif
    const uint8_t altSource = positionConfig()->altitude_source;
    const bool gpsAltAllowed = (altSource == ALTITUDE_SOURCE_DEFAULT ||
                                altSource == ALTITUDE_SOURCE_GPS_ONLY ||
                                altSource == ALTITUDE_SOURCE_RANGEFINDER_PREFER);

    // Late origin capture: if XY fusion was enabled before GPS_FIX became
    // available (e.g. opticalflow-only arm), grab the origin the first time
    // a valid fix arrives so downstream consumers (flight plan) can proceed.
    if (xyEnabled && !gpsArmLocationSet && gpsXYAllowed) {
        armLocationGps = gpsSol.llh;
        gpsArmLocationSet = true;
    }

    // XY position + velocity measurements
    if (xyEnabled && gpsXYAllowed && gpsArmLocationSet) {
        vector2_t gpsDistCm;
        GPS_distance2d(&gpsSol.llh, &armLocationGps, &gpsDistCm);
        // gpsDistCm: X = East-West (lon), Y = North-South (lat) relative to arm position

        const float rPos = gpsR(R_GPS_POS_BASE);
        kalmanUpdatePosition(&kfX, gpsDistCm.x, rPos);
        kalmanUpdatePosition(&kfY, gpsDistCm.y, rPos);

        // GPS velocity (NED from UBX) -> ENU
        const float rVel = gpsR(R_GPS_VEL_BASE);
        kalmanUpdateVelocity(&kfX, (float)gpsSol.velned.velE, rVel);
        kalmanUpdateVelocity(&kfY, (float)gpsSol.velned.velN, rVel);

        lastXYMeasurementUs = nowUs;
    }

    // Z altitude measurement
    if (gpsAltAllowed) {
        if (!gpsAltOffsetSet) {
            gpsAltOffsetCm = gpsSol.llh.altCm;
            gpsAltOffsetSet = true;
        }

        // Use altitude_prefer_baro to scale GPS altitude R:
        // altitude_prefer_baro=100 means strongly prefer baro, so GPS R should be higher
        const float baroPreference = positionConfig()->altitude_prefer_baro * 0.01f;
        const float gpsAltR = gpsR(R_GPS_ALT_BASE) * (1.0f + baroPreference * 2.0f);

        const float gpsRelativeAltCm = gpsSol.llh.altCm - gpsAltOffsetCm;
        kalmanUpdatePosition(&kfZ, gpsRelativeAltCm, gpsAltR);
        lastZMeasurementUs = nowUs;

        zCal[CAL_Z_GPS].rawReading = gpsSol.llh.altCm;
        zCal[CAL_Z_GPS].active = true;
    }
#else
    UNUSED(nowUs);
#endif
}

static void feedBaroMeasurements(timeUs_t nowUs)
{
#ifdef USE_BARO
    if (!sensors(SENSOR_BARO)) {
        return;
    }

    const uint8_t altSource = positionConfig()->altitude_source;
    if (altSource == ALTITUDE_SOURCE_GPS_ONLY ||
        altSource == ALTITUDE_SOURCE_RANGEFINDER_ONLY) {
        return;
    }

    const float baroAltCm = getBaroAltitude();

    if (!baroOffsetSet) {
        // Capture disarmed baseline once; keep live relative altitude while disarmed.
        baroAltAccumulator = baroAltCm;
        baroAltOffsetCm = baroAltCm;
        baroOffsetSet = true;
    }

    // Scale R based on altitude_prefer_baro: higher value = lower baro R = more trust
    const float baroPreference = constrainf(positionConfig()->altitude_prefer_baro * 0.01f, 0.01f, 1.0f);
    const float baroR = R_BARO_ALT / baroPreference;

    kalmanUpdatePosition(&kfZ, baroAltCm - baroAltOffsetCm, baroR);
    lastZMeasurementUs = nowUs;

    zCal[CAL_Z_BARO].rawReading = baroAltCm;
    zCal[CAL_Z_BARO].active = true;
#else
    UNUSED(nowUs);
#endif
}

static void feedRangefinderMeasurements(timeUs_t nowUs)
{
#ifdef USE_RANGEFINDER
    const uint8_t altSource = positionConfig()->altitude_source;
    if (altSource == ALTITUDE_SOURCE_GPS_ONLY ||
        altSource == ALTITUDE_SOURCE_BARO_ONLY) {
        return;
    }

    float altCm;
    if (!rangefinderSampleAltitudeCm(&altCm, positionConfig()->rangefinder_max_range_cm)) {
        return;
    }

    if (!rangefinderOffsetSet) {
        rangefinderAltOffsetCm = altCm;
        rangefinderOffsetSet = true;
    }

    // Rangefinder has low noise; give it very low R when source prefers it
    float rfR = R_RANGEFINDER_ALT;
    if (altSource == ALTITUDE_SOURCE_RANGEFINDER_ONLY ||
        altSource == ALTITUDE_SOURCE_RANGEFINDER_PREFER) {
        rfR *= 0.25f;  // even lower noise -> stronger pull
    }

    kalmanUpdatePosition(&kfZ, altCm - rangefinderAltOffsetCm, rfR);
    lastZMeasurementUs = nowUs;

    zCal[CAL_Z_RF].rawReading = altCm;
    zCal[CAL_Z_RF].active = true;
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
        return;
    }

#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    const uint8_t posSource = posHoldConfig()->positionSource;
    if (posSource == POSHOLD_SOURCE_GPS_ONLY) {
        return;
    }
#endif

    const uint8_t altSource = positionConfig()->altitude_source;
    if (altSource == ALTITUDE_SOURCE_BARO_ONLY || altSource == ALTITUDE_SOURCE_GPS_ONLY) {
        return;
    }

#ifdef USE_RANGEFINDER
    float maxRangeCm = positionConfig()->rangefinder_max_range_cm;
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    maxRangeCm = fminf(maxRangeCm, (float)posHoldConfig()->opticalflowMaxRange);
#endif
    float altitudeCmF;
    if (!rangefinderSampleAltitudeCm(&altitudeCmF, maxRangeCm)) {
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

    // Convert flow rates (rad/s) to velocity (cm/s) in body frame, scaled by rangefinder height
    const float vBFx = flow->processedFlowRates.x * altitudeCm;
    const float vBFy = flow->processedFlowRates.y * altitudeCm;

    // Project body-frame velocity to earth frame using heading
    // Flow X (roll axis) corresponds to lateral movement, flow Y (pitch axis) to longitudinal
    const float yawRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    const float cosYaw = cos_approx(yawRad);
    const float sinYaw = sin_approx(yawRad);

    // Also project through pitch/roll to horizontal plane
    const float cosPitch = cos_approx(DECIDEGREES_TO_RADIANS(attitude.values.pitch));
    const float cosRoll  = cos_approx(DECIDEGREES_TO_RADIANS(attitude.values.roll));
    const float vBodyX = vBFx * cosRoll;   // lateral velocity, tilt-corrected
    const float vBodyY = vBFy * cosPitch;  // longitudinal velocity, tilt-corrected

    // Rotate body-heading-frame velocity to ENU
    // Body X (roll) = rightward, Body Y (pitch) = forward
    // ENU East  =  bodyY * sinYaw + bodyX * cosYaw
    // ENU North =  bodyY * cosYaw - bodyX * sinYaw
    const float velEast  =  vBodyY * sinYaw + vBodyX * cosYaw;
    const float velNorth =  vBodyY * cosYaw - vBodyX * sinYaw;

    kalmanUpdateVelocity(&kfX, velEast, flowR);
    kalmanUpdateVelocity(&kfY, velNorth, flowR);

    lastXYMeasurementUs = nowUs;
#else
    UNUSED(nowUs);
#endif
}

static void crossCalibrateOffsets(sensorCalEntry_t *sources, int count, float kfPosition)
{
    bool hasAnchor = false;
    for (int i = 0; i < count; i++) {
        if (sources[i].active && !sources[i].drifts) {
            hasAnchor = true;
            break;
        }
    }

    for (int i = 0; i < count; i++) {
        if (hasAnchor && ARMING_FLAG(ARMED) &&
            sources[i].active && sources[i].drifts && sources[i].offsetPtr) {
            const float idealOffset = sources[i].rawReading - kfPosition;
            *sources[i].offsetPtr += CROSS_CAL_ALPHA * (idealOffset - *sources[i].offsetPtr);
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

    // Z-axis: always runs (for altitude hold, OSD, vario).
    // While disarmed, predict with zero acceleration so covariance continues to evolve
    // and incoming baro/rangefinder updates remain responsive.
    kalmanPredict(&kfZ, dt, ARMING_FLAG(ARMED) ? accelUp : 0.0f);

    // XY axes: only when a consumer is active
    if (xyEnabled && ARMING_FLAG(ARMED)) {
        kalmanPredict(&kfX, dt, accelEast);
        kalmanPredict(&kfY, dt, accelNorth);
    }

    // Feed sensor measurements (order does not matter)
    feedGPSMeasurements(nowUs);
    feedBaroMeasurements(nowUs);
    feedRangefinderMeasurements(nowUs);
    feedOpticalFlowMeasurements(nowUs);

    // Calibrate drifting sensor offsets against KF estimate anchored by non-drifting sources
    crossCalibrateOffsets(zCal, CAL_Z_COUNT, kalmanGetPosition(&kfZ));

    // Extract state into the unified estimate
    estimate.position.x = kalmanGetPosition(&kfX);
    estimate.position.y = kalmanGetPosition(&kfY);
    estimate.position.z = kalmanGetPosition(&kfZ);

    estimate.velocity.x = kalmanGetVelocity(&kfX);
    estimate.velocity.y = kalmanGetVelocity(&kfY);
    estimate.velocity.z = kalmanGetVelocity(&kfZ);

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
    const float xyVar = (kalmanGetPositionVariance(&kfX) + kalmanGetPositionVariance(&kfY)) * 0.5f;
    estimate.trustXY = 1.0f / (1.0f + xyVar / 10000.0f);
    estimate.trustZ = 1.0f / (1.0f + kalmanGetPositionVariance(&kfZ) / 10000.0f);
}

const positionEstimate3d_t *positionEstimatorGetEstimate(void)
{
    return &estimate;
}

float positionEstimatorGetAltitudeCm(void)
{
    return estimate.position.z;
}

float positionEstimatorGetAltitudeDerivative(void)
{
    return estimate.velocity.z;
}

bool positionEstimatorIsValidXY(void)
{
    return estimate.isValidXY;
}

bool positionEstimatorIsValidZ(void)
{
    return estimate.isValidZ;
}

float positionEstimatorGetTrustXY(void)
{
    return estimate.trustXY;
}

void positionEstimatorResetZ(void)
{
    kalmanInit(&kfZ, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_Z);
    estimate.position.z = 0.0f;
    estimate.velocity.z = 0.0f;
    estimate.isValidZ = false;
    lastZMeasurementUs = 0;
#ifdef USE_GPS
    gpsAltOffsetCm = 0.0f;
    gpsAltOffsetSet = false;
#endif
#ifdef USE_BARO
    baroAltOffsetCm = 0.0f;
    baroAltAccumulator = 0.0f;
    baroOffsetSet = false;
#endif
#ifdef USE_RANGEFINDER
    rangefinderAltOffsetCm = 0.0f;
    rangefinderOffsetSet = false;
#endif

    initZCalEntries();
}

void positionEstimatorResetXY(void)
{
    kalmanInit(&kfX, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_XY);
    kalmanInit(&kfY, 0.0f, 0.0f, INITIAL_POS_VAR, INITIAL_VEL_VAR, Q_ACCEL_XY);
    estimate.position.x = 0.0f;
    estimate.position.y = 0.0f;
    estimate.velocity.x = 0.0f;
    estimate.velocity.y = 0.0f;
    estimate.isValidXY = false;
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
