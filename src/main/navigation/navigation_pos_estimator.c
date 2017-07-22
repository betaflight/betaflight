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
#include <math.h>
#include <string.h>

#include "platform.h"

#if defined(NAV)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/pitotmeter.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"


/**
 * Model-identification based position estimator
 * Based on INAV position estimator for PX4 by Anton Babushkin <anton.babushkin@me.com>
 * @author Konstantin Sharlaimov <konstantin.sharlaimov@gmail.com>
 */
#define INAV_GPS_DEFAULT_EPH                200.0f  // 2m GPS HDOP  (gives about 1.6s of dead-reckoning if GPS is temporary lost)
#define INAV_GPS_DEFAULT_EPV                500.0f  // 5m GPS VDOP

#define INAV_GPS_ACCEPTANCE_EPE             500.0f  // 5m acceptance radius

#define INAV_ACC_BIAS_ACCEPTANCE_VALUE      (GRAVITY_CMSS * 0.25f)   // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway

#define INAV_GPS_GLITCH_RADIUS              250.0f  // 2.5m GPS glitch radius
#define INAV_GPS_GLITCH_ACCEL               1000.0f // 10m/s/s max possible acceleration for GPS glitch detection

#define INAV_POSITION_PUBLISH_RATE_HZ       50      // Publish position updates at this rate
#define INAV_PITOT_UPDATE_RATE              10

#define INAV_GPS_TIMEOUT_MS                 1500    // GPS timeout
#define INAV_BARO_TIMEOUT_MS                200     // Baro timeout
#define INAV_SURFACE_TIMEOUT_MS               300     // Surface timeout    (missed 3 readings in a row)

#define INAV_HISTORY_BUF_SIZE               (INAV_POSITION_PUBLISH_RATE_HZ / 2)     // Enough to hold 0.5 sec historical data

typedef struct {
    timeUs_t    lastTriggeredTime;
    timeUs_t    deltaTime;
} navigationTimer_t;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)
#if defined(NAV_GPS_GLITCH_DETECTION)
    bool        glitchDetected;
    bool        glitchRecovery;
#endif
    t_fp_vector pos;            // GPS position in NEU coordinate system (cm)
    t_fp_vector vel;            // GPS velocity (cms)
    float       eph;
    float       epv;
} navPositionEstimatorGPS_t;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)
    float       alt;            // Raw barometric altitude (cm)
    float       epv;
} navPositionEstimatorBARO_t;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)
    float       airspeed;            // airspeed (cm/s)
} navPositionEstimatorPITOT_t;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)
    float       alt;            // Raw altitude measurement (cm)
} navPositionEstimatorSURFACE_t;

typedef struct {
    timeUs_t    lastUpdateTime; // Last update time (us)
    // 3D position, velocity and confidence
    t_fp_vector pos;
    t_fp_vector vel;
    float       eph;
    float       epv;
    // Surface offset
    float       surface;
    float       surfaceVel;
    bool        surfaceValid;
} navPositionEstimatorESTIMATE_t;

typedef struct {
    timeUs_t    baroGroundTimeout;
    float       baroGroundAlt;
    bool        isBaroGroundValid;
} navPositionEstimatorSTATE_t;

typedef struct {
    uint8_t     index;
    t_fp_vector pos[INAV_HISTORY_BUF_SIZE];
    t_fp_vector vel[INAV_HISTORY_BUF_SIZE];
} navPosisitonEstimatorHistory_t;

typedef struct {
    t_fp_vector     accelNEU;
    t_fp_vector     accelBias;
    bool            gravityCalibrationComplete;
} navPosisitonEstimatorIMU_t;

typedef struct {
    // Data sources
    navPositionEstimatorGPS_t   gps;
    navPositionEstimatorBARO_t  baro;
    navPositionEstimatorSURFACE_t surface;
    navPositionEstimatorPITOT_t pitot;

    // IMU data
    navPosisitonEstimatorIMU_t  imu;

    // Estimate
    navPositionEstimatorESTIMATE_t  est;

    // Estimation history
    navPosisitonEstimatorHistory_t  history;

    // Extra state variables
    navPositionEstimatorSTATE_t state;
} navigationPosEstimator_s;

static navigationPosEstimator_s posEstimator;

PG_REGISTER_WITH_RESET_TEMPLATE(positionEstimationConfig_t, positionEstimationConfig, PG_POSITION_ESTIMATION_CONFIG, 1);

PG_RESET_TEMPLATE(positionEstimationConfig_t, positionEstimationConfig,
        // Inertial position estimator parameters
        .automatic_mag_declination = 1,
        .reset_altitude_type = NAV_RESET_ALTITUDE_ON_FIRST_ARM,
        .gps_delay_ms = 200,
        .gravity_calibration_tolerance = 5,     // 5 cm/s/s calibration error accepted (0.5% of gravity)
        .use_gps_velned = 1,         // "Disabled" is mandatory with gps_dyn_model = Pedestrian

        .max_surface_altitude = 200,

        .w_z_baro_p = 0.35f,

        .w_z_surface_p = 3.500f,
        .w_z_surface_v = 6.100f,

        .w_z_gps_p = 0.2f,
        .w_z_gps_v = 0.5f,

        .w_xy_gps_p = 1.0f,
        .w_xy_gps_v = 2.0f,

        .w_z_res_v = 0.5f,
        .w_xy_res_v = 0.5f,

        .w_acc_bias = 0.01f,

        .max_eph_epv = 1000.0f,
        .baro_epv = 100.0f
);

/* Inertial filter, implementation taken from PX4 implementation by Anton Babushkin <rk3dov@gmail.com> */
static void inavFilterPredict(int axis, float dt, float acc)
{
    posEstimator.est.pos.A[axis] += posEstimator.est.vel.A[axis] * dt + acc * dt * dt / 2.0f;
    posEstimator.est.vel.A[axis] += acc * dt;
}

static void inavFilterCorrectPos(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    posEstimator.est.pos.A[axis] += ewdt;
    posEstimator.est.vel.A[axis] += w * ewdt;
}

static void inavFilterCorrectVel(int axis, float dt, float e, float w)
{
    posEstimator.est.vel.A[axis] += e * w * dt;
}

#define resetTimer(tim, currentTimeUs) { (tim)->deltaTime = 0; (tim)->lastTriggeredTime = currentTimeUs; }
#define getTimerDeltaMicros(tim) ((tim)->deltaTime)
static bool updateTimer(navigationTimer_t * tim, timeUs_t interval, timeUs_t currentTimeUs)
{
    if ((currentTimeUs - tim->lastTriggeredTime) >= interval) {
        tim->deltaTime = currentTimeUs - tim->lastTriggeredTime;
        tim->lastTriggeredTime = currentTimeUs;
        return true;
    }
    else {
        return false;
    }
}

static bool shouldResetReferenceAltitude(void)
{
    switch (positionEstimationConfig()->reset_altitude_type) {
        case NAV_RESET_ALTITUDE_NEVER:
            return false;
        case NAV_RESET_ALTITUDE_ON_FIRST_ARM:
            return !ARMING_FLAG(ARMED) && !ARMING_FLAG(WAS_EVER_ARMED);
        case NAV_RESET_ALTITUDE_ON_EACH_ARM:
            return !ARMING_FLAG(ARMED);
    }

    return false;
}

#if defined(GPS)
/* Why is this here: Because GPS will be sending at quiet a nailed rate (if not overloaded by junk tasks at the brink of its specs)
 * but we might read out with timejitter because Irq might be off by a few us so we do a +-10% margin around the time between GPS
 * datasets representing the most common Hz-rates today. You might want to extend the list or find a smarter way.
 * Don't overload your GPS in its config with trash, choose a Hz rate that it can deliver at a sustained rate.
 * (c) CrashPilot1000
 */
static timeUs_t getGPSDeltaTimeFilter(timeUs_t dTus)
{
    if (dTus >= 225000 && dTus <= 275000) return HZ2US(4);       //  4Hz Data 250ms
    if (dTus >= 180000 && dTus <= 220000) return HZ2US(5);       //  5Hz Data 200ms
    if (dTus >=  90000 && dTus <= 110000) return HZ2US(10);      // 10Hz Data 100ms
    if (dTus >=  45000 && dTus <=  55000) return HZ2US(20);      // 20Hz Data  50ms
    if (dTus >=  30000 && dTus <=  36000) return HZ2US(30);      // 30Hz Data  33ms
    if (dTus >=  23000 && dTus <=  27000) return HZ2US(40);      // 40Hz Data  25ms
    if (dTus >=  18000 && dTus <=  22000) return HZ2US(50);      // 50Hz Data  20ms
    return dTus;                                                 // Filter failed. Set GPS Hz by measurement
}

#if defined(NAV_GPS_GLITCH_DETECTION)
static bool detectGPSGlitch(timeUs_t currentTimeUs)
{
    static timeUs_t previousTime = 0;
    static t_fp_vector lastKnownGoodPosition;
    static t_fp_vector lastKnownGoodVelocity;

    bool isGlitching = false;

    if (previousTime == 0) {
        isGlitching = false;
    }
    else {
        t_fp_vector predictedGpsPosition;
        float gpsDistance;
        float dT = US2S(currentTimeUs - previousTime);

        /* We predict new position based on previous GPS velocity and position */
        predictedGpsPosition.V.X = lastKnownGoodPosition.V.X + lastKnownGoodVelocity.V.X * dT;
        predictedGpsPosition.V.Y = lastKnownGoodPosition.V.Y + lastKnownGoodVelocity.V.Y * dT;

        /* New pos is within predefined radius of predicted pos, radius is expanded exponentially */
        gpsDistance = sqrtf(sq(predictedGpsPosition.V.X - lastKnownGoodPosition.V.X) + sq(predictedGpsPosition.V.Y - lastKnownGoodPosition.V.Y));
        if (gpsDistance <= (INAV_GPS_GLITCH_RADIUS + 0.5f * INAV_GPS_GLITCH_ACCEL * dT * dT)) {
            isGlitching = false;
        }
        else {
            isGlitching = true;
        }
    }

    if (!isGlitching) {
        previousTime = currentTimeUs;
        lastKnownGoodPosition = posEstimator.gps.pos;
        lastKnownGoodVelocity = posEstimator.gps.vel;
    }

    return isGlitching;
}
#endif

/**
 * Update GPS topic
 *  Function is called on each GPS update
 */
void onNewGPSData(void)
{
    static timeUs_t lastGPSNewDataTime;
    static int32_t previousLat;
    static int32_t previousLon;
    static int32_t previousAlt;
    static bool isFirstGPSUpdate = true;

    gpsLocation_t newLLH;
    const timeUs_t currentTimeUs = micros();

    newLLH.lat = gpsSol.llh.lat;
    newLLH.lon = gpsSol.llh.lon;
    newLLH.alt = gpsSol.llh.alt;

    if (sensors(SENSOR_GPS)) {
        if (!STATE(GPS_FIX)) {
            isFirstGPSUpdate = true;
            return;
        }

        if ((currentTimeUs - lastGPSNewDataTime) > MS2US(INAV_GPS_TIMEOUT_MS)) {
            isFirstGPSUpdate = true;
        }

#if defined(NAV_AUTO_MAG_DECLINATION)
        /* Automatic magnetic declination calculation - do this once */
        static bool magDeclinationSet = false;
        if (positionEstimationConfig()->automatic_mag_declination && !magDeclinationSet) {
            mag.magneticDeclination = geoCalculateMagDeclination(&newLLH) * 10.0f; // heading is in 0.1deg units
            magDeclinationSet = true;
        }
#endif

        /* Process position update if GPS origin is already set, or precision is good enough */
        // FIXME: Add HDOP check for acquisition of GPS origin
        /* Set GPS origin or reset the origin altitude - keep initial pre-arming altitude at zero */
        if (!posControl.gpsOrigin.valid) {
            geoSetOrigin(&posControl.gpsOrigin, &newLLH, GEO_ORIGIN_SET);
        }
        else if (shouldResetReferenceAltitude()) {
            /* If we were never armed - keep altitude at zero */
            geoSetOrigin(&posControl.gpsOrigin, &newLLH, GEO_ORIGIN_RESET_ALTITUDE);
        }

        if (posControl.gpsOrigin.valid) {
            /* Convert LLH position to local coordinates */
            geoConvertGeodeticToLocal(&posControl.gpsOrigin, &newLLH, & posEstimator.gps.pos, GEO_ALT_ABSOLUTE);

            /* If not the first update - calculate velocities */
            if (!isFirstGPSUpdate) {
                float dT = US2S(getGPSDeltaTimeFilter(currentTimeUs - lastGPSNewDataTime));

                /* Use VELNED provided by GPS if available, calculate from coordinates otherwise */
                float gpsScaleLonDown = constrainf(cos_approx((ABS(gpsSol.llh.lat) / 10000000.0f) * 0.0174532925f), 0.01f, 1.0f);
                if (positionEstimationConfig()->use_gps_velned && gpsSol.flags.validVelNE) {
                    posEstimator.gps.vel.V.X = gpsSol.velNED[0];
                    posEstimator.gps.vel.V.Y = gpsSol.velNED[1];
                }
                else {
                    posEstimator.gps.vel.V.X = (posEstimator.gps.vel.V.X + (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * (gpsSol.llh.lat - previousLat) / dT)) / 2.0f;
                    posEstimator.gps.vel.V.Y = (posEstimator.gps.vel.V.Y + (gpsScaleLonDown * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * (gpsSol.llh.lon - previousLon) / dT)) / 2.0f;
                }

                if (positionEstimationConfig()->use_gps_velned && gpsSol.flags.validVelD) {
                    posEstimator.gps.vel.V.Z = - gpsSol.velNED[2];   // NEU
                }
                else {
                    posEstimator.gps.vel.V.Z = (posEstimator.gps.vel.V.Z + (gpsSol.llh.alt - previousAlt) / dT) / 2.0f;
                }

#if defined(NAV_GPS_GLITCH_DETECTION)
                /* GPS glitch protection. We have local coordinates and local velocity for current GPS update. Check if they are sane */
                if (detectGPSGlitch(currentTimeUs)) {
                    posEstimator.gps.glitchRecovery = false;
                    posEstimator.gps.glitchDetected = true;
                }
                else {
                    /* Store previous glitch flag in glitchRecovery to indicate a valid reading after a glitch */
                    posEstimator.gps.glitchRecovery = posEstimator.gps.glitchDetected;
                    posEstimator.gps.glitchDetected = false;
                }
#endif

                /* FIXME: use HDOP/VDOP */
                if (gpsSol.flags.validEPE) {
                    posEstimator.gps.eph = gpsSol.eph;
                    posEstimator.gps.epv = gpsSol.epv;
                }
                else {
                    posEstimator.gps.eph = INAV_GPS_DEFAULT_EPH;
                    posEstimator.gps.epv = INAV_GPS_DEFAULT_EPV;
                }

                /* Indicate a last valid reading of Pos/Vel */
                posEstimator.gps.lastUpdateTime = currentTimeUs;
            }

            previousLat = gpsSol.llh.lat;
            previousLon = gpsSol.llh.lon;
            previousAlt = gpsSol.llh.alt;
            isFirstGPSUpdate = false;

            lastGPSNewDataTime = currentTimeUs;
        }
    }
    else {
        posEstimator.gps.lastUpdateTime = 0;
    }
}
#endif

#if defined(BARO)
/**
 * Read BARO and update alt/vel topic
 *  Function is called from TASK_BARO
 */
void updatePositionEstimator_BaroTopic(timeUs_t currentTimeUs)
{
    static float initialBaroAltitudeOffset = 0.0f;
    float newBaroAlt = baroCalculateAltitude();

    /* If we are required - keep altitude at zero */
    if (shouldResetReferenceAltitude()) {
        initialBaroAltitudeOffset = newBaroAlt;
    }

    if (sensors(SENSOR_BARO) && baroIsCalibrationComplete()) {
        posEstimator.baro.alt = newBaroAlt - initialBaroAltitudeOffset;
        posEstimator.baro.epv = positionEstimationConfig()->baro_epv;
        posEstimator.baro.lastUpdateTime = currentTimeUs;
    }
    else {
        posEstimator.baro.alt = 0;
        posEstimator.baro.lastUpdateTime = 0;
    }
}
#endif

#if defined(PITOT)
/**
 * Read Pitot and update airspeed topic
 *  Function is called at main loop rate, updates happen at reduced rate
 */
static void updatePitotTopic(timeUs_t currentTimeUs)
{
    static navigationTimer_t pitotUpdateTimer;

    if (updateTimer(&pitotUpdateTimer, HZ2US(INAV_PITOT_UPDATE_RATE), currentTimeUs)) {
        float newTAS = pitotCalculateAirSpeed();
        if (sensors(SENSOR_PITOT) && pitotIsCalibrationComplete()) {
            posEstimator.pitot.airspeed = newTAS;
        }
        else {
            posEstimator.pitot.airspeed = 0;
        }
    }
}
#endif

#ifdef USE_RANGEFINDER
/**
 * Read surface and update alt/vel topic
 *  Function is called from TASK_RANGEFINDER at arbitrary rate - as soon as new measurements are available
 */
void updatePositionEstimator_SurfaceTopic(timeUs_t currentTimeUs, float newSurfaceAlt)
{
    if (newSurfaceAlt > 0 && newSurfaceAlt <= positionEstimationConfig()->max_surface_altitude) {
        posEstimator.surface.alt = newSurfaceAlt;
        posEstimator.surface.lastUpdateTime = currentTimeUs;
    }
}
#endif

/**
 * Update IMU topic
 *  Function is called at main loop rate
 */
static void updateIMUTopic(void)
{
    static float calibratedGravityCMSS = GRAVITY_CMSS;
    static timeMs_t gravityCalibrationTimeout = 0;

    if (!isImuReady()) {
        posEstimator.imu.accelNEU.V.X = 0;
        posEstimator.imu.accelNEU.V.Y = 0;
        posEstimator.imu.accelNEU.V.Z = 0;

        gravityCalibrationTimeout = millis();
        posEstimator.imu.gravityCalibrationComplete = false;
    }
    else {
        t_fp_vector accelBF;

        /* Read acceleration data in body frame */
        accelBF.V.X = imuMeasuredAccelBF.V.X;
        accelBF.V.Y = imuMeasuredAccelBF.V.Y;
        accelBF.V.Z = imuMeasuredAccelBF.V.Z;

        /* Correct accelerometer bias */
        accelBF.V.X -= posEstimator.imu.accelBias.V.X;
        accelBF.V.Y -= posEstimator.imu.accelBias.V.Y;
        accelBF.V.Z -= posEstimator.imu.accelBias.V.Z;

        /* Rotate vector to Earth frame - from Forward-Right-Down to North-East-Up*/
        imuTransformVectorBodyToEarth(&accelBF);

        /* Read acceleration data in NEU frame from IMU */
        posEstimator.imu.accelNEU.V.X = accelBF.V.X;
        posEstimator.imu.accelNEU.V.Y = accelBF.V.Y;
        posEstimator.imu.accelNEU.V.Z = accelBF.V.Z;

        /* When unarmed, assume that accelerometer should measure 1G. Use that to correct accelerometer gain */
        if (!ARMING_FLAG(ARMED) && !posEstimator.imu.gravityCalibrationComplete) {
            // Slowly converge on calibrated gravity while level
            const float gravityOffsetError = posEstimator.imu.accelNEU.V.Z - calibratedGravityCMSS;
            calibratedGravityCMSS += gravityOffsetError * 0.0025f;

            if (ABS(gravityOffsetError) < positionEstimationConfig()->gravity_calibration_tolerance) {  // Error should be within 0.5% of calibrated gravity
                if ((millis() - gravityCalibrationTimeout) > 250) {
                    posEstimator.imu.gravityCalibrationComplete = true;
                }
            }
            else {
                gravityCalibrationTimeout = millis();
            }
        }

        /* If calibration is incomplete - report zero acceleration */
        if (posEstimator.imu.gravityCalibrationComplete) {
            posEstimator.imu.accelNEU.V.Z -= calibratedGravityCMSS;
        }
        else {
            posEstimator.imu.accelNEU.V.X = 0;
            posEstimator.imu.accelNEU.V.Y = 0;
            posEstimator.imu.accelNEU.V.Z = 0;
        }

#if defined(NAV_BLACKBOX)
        /* Update blackbox values */
        navAccNEU[X] = posEstimator.imu.accelNEU.A[X];
        navAccNEU[Y] = posEstimator.imu.accelNEU.A[Y];
        navAccNEU[Z] = posEstimator.imu.accelNEU.A[Z];
#endif
    }
}

static float updateEPE(const float oldEPE, const float dt, const float newEPE, const float w)
{
    return oldEPE + (newEPE - oldEPE) * w * dt;
}

/**
 * Calculate next estimate using IMU and apply corrections from reference sensors (GPS, BARO etc)
 *  Function is called at main loop rate
 */
static void updateEstimatedTopic(timeUs_t currentTimeUs)
{
    t_fp_vector accelBiasCorr;
    float dt = US2S(currentTimeUs - posEstimator.est.lastUpdateTime);
    posEstimator.est.lastUpdateTime = currentTimeUs;

    /* If IMU is not ready we can't estimate anything */
    if (!isImuReady()) {
        posEstimator.est.eph = positionEstimationConfig()->max_eph_epv + 0.001f;
        posEstimator.est.epv = positionEstimationConfig()->max_eph_epv + 0.001f;
        return;
    }

    /* Calculate new EPH and EPV for the case we didn't update postion */
    float newEPH = posEstimator.est.eph;
    float newEPV = posEstimator.est.epv;

    if (newEPH <= positionEstimationConfig()->max_eph_epv) {
        newEPH *= 1.0f + dt;
    }

    if (newEPV <= positionEstimationConfig()->max_eph_epv) {
        newEPV *= 1.0f + dt;
    }


    /* Figure out if we have valid position data from our data sources */
    bool isGPSValid = sensors(SENSOR_GPS) &&
                      posControl.gpsOrigin.valid &&
                      ((currentTimeUs - posEstimator.gps.lastUpdateTime) <= MS2US(INAV_GPS_TIMEOUT_MS)) &&
                      (posEstimator.gps.eph < positionEstimationConfig()->max_eph_epv);   // EPV is checked later
    bool isBaroValid = sensors(SENSOR_BARO) && ((currentTimeUs - posEstimator.baro.lastUpdateTime) <= MS2US(INAV_BARO_TIMEOUT_MS));
    bool isSurfaceValid = sensors(SENSOR_RANGEFINDER) && ((currentTimeUs - posEstimator.surface.lastUpdateTime) <= MS2US(INAV_SURFACE_TIMEOUT_MS));

    /* Do some preparations to data */
    if (isBaroValid) {
        if (!ARMING_FLAG(ARMED)) {
            posEstimator.state.baroGroundAlt = posEstimator.est.pos.V.Z;
            posEstimator.state.isBaroGroundValid = true;
            posEstimator.state.baroGroundTimeout = currentTimeUs + 250000;   // 0.25 sec
        }
        else {
            if (posEstimator.est.vel.V.Z > 15) {
                if (currentTimeUs > posEstimator.state.baroGroundTimeout) {
                    posEstimator.state.isBaroGroundValid = false;
                }
            }
            else {
                posEstimator.state.baroGroundTimeout = currentTimeUs + 250000;   // 0.25 sec
            }
        }
    }
    else {
        posEstimator.state.isBaroGroundValid = false;
    }

    /* We might be experiencing air cushion effect - use sonar or baro groung altitude to detect it */
    bool isAirCushionEffectDetected = ARMING_FLAG(ARMED) &&
                                        ((isSurfaceValid && posEstimator.surface.alt < 20.0f && posEstimator.state.isBaroGroundValid) ||
                                         (isBaroValid && posEstimator.state.isBaroGroundValid && posEstimator.baro.alt < posEstimator.state.baroGroundAlt));

#if defined(NAV_GPS_GLITCH_DETECTION)
    //isGPSValid = isGPSValid && !posEstimator.gps.glitchDetected;
#endif

    /* Validate EPV for GPS and calculate altitude/climb rate correction flags */
    const bool isGPSZValid = isGPSValid && (posEstimator.gps.epv < positionEstimationConfig()->max_eph_epv);
    const bool useGpsZPos = STATE(FIXED_WING) && !sensors(SENSOR_BARO) && isGPSValid && isGPSZValid;
    const bool useGpsZVel = isGPSValid && isGPSZValid;

    /* Estimate validity */
    const bool isEstXYValid = (posEstimator.est.eph < positionEstimationConfig()->max_eph_epv);
    const bool isEstZValid = (posEstimator.est.epv < positionEstimationConfig()->max_eph_epv);

    /* Handle GPS loss and recovery */
    if (isGPSValid) {
        bool positionWasReset = false;

        /* If GPS is valid and our estimate is NOT valid - reset it to GPS coordinates and velocity */
        if (!isEstXYValid) {
            posEstimator.est.pos.V.X = posEstimator.gps.pos.V.X;
            posEstimator.est.pos.V.Y = posEstimator.gps.pos.V.Y;
            posEstimator.est.vel.V.X = posEstimator.gps.vel.V.X;
            posEstimator.est.vel.V.Y = posEstimator.gps.vel.V.Y;
            newEPH = posEstimator.gps.eph;
            positionWasReset = true;
        }

        if (!isEstZValid && useGpsZPos) {
            posEstimator.est.pos.V.Z = posEstimator.gps.pos.V.Z;
            posEstimator.est.vel.V.Z = posEstimator.gps.vel.V.Z;
            newEPV = posEstimator.gps.epv;
            positionWasReset = true;
        }

        /* If position was reset we need to reset history as well */
        if (positionWasReset) {
            for (int i = 0; i < INAV_HISTORY_BUF_SIZE; i++) {
                posEstimator.history.pos[i] = posEstimator.est.pos;
                posEstimator.history.vel[i] = posEstimator.est.vel;
            }

            posEstimator.history.index = 0;
        }
    }

    /* Pre-calculate history index for GPS delay compensation */
    int gpsHistoryIndex = (posEstimator.history.index - 1) - constrain(((int)positionEstimationConfig()->gps_delay_ms / (1000 / INAV_POSITION_PUBLISH_RATE_HZ)), 0, INAV_HISTORY_BUF_SIZE - 1);
    if (gpsHistoryIndex < 0) {
        gpsHistoryIndex += INAV_HISTORY_BUF_SIZE;
    }

    /* Prediction step: Z-axis */
    if (isEstZValid) {
        inavFilterPredict(Z, dt, posEstimator.imu.accelNEU.V.Z);
    }

    /* Prediction step: XY-axis */
    if (isEstXYValid) {
        if (isImuHeadingValid()) {
            inavFilterPredict(X, dt, posEstimator.imu.accelNEU.V.X);
            inavFilterPredict(Y, dt, posEstimator.imu.accelNEU.V.Y);
        }
        else {
            inavFilterPredict(X, dt, 0.0f);
            inavFilterPredict(Y, dt, 0.0f);
        }
    }

    /* Calculate residual */
    float gpsResidual[3][2];
    float baroResidual;
    float gpsResidualXYMagnitude;

    if (isGPSValid) {
        gpsResidual[X][0] = posEstimator.gps.pos.V.X - posEstimator.history.pos[gpsHistoryIndex].V.X;
        gpsResidual[Y][0] = posEstimator.gps.pos.V.Y - posEstimator.history.pos[gpsHistoryIndex].V.Y;
        gpsResidual[Z][0] = posEstimator.gps.pos.V.Z - posEstimator.history.pos[gpsHistoryIndex].V.Z;
        gpsResidual[X][1] = posEstimator.gps.vel.V.X - posEstimator.history.vel[gpsHistoryIndex].V.X;
        gpsResidual[Y][1] = posEstimator.gps.vel.V.Y - posEstimator.history.vel[gpsHistoryIndex].V.Y;
        gpsResidual[Z][1] = posEstimator.gps.vel.V.Z - posEstimator.history.vel[gpsHistoryIndex].V.Z;

        gpsResidualXYMagnitude = sqrtf(sq(gpsResidual[X][0]) + sq(gpsResidual[Y][0]));
    }

    if (isBaroValid) {
        /* If we are going to use GPS Z-position - calculate and apply barometer offset */
        baroResidual = (isAirCushionEffectDetected ? posEstimator.state.baroGroundAlt : posEstimator.baro.alt) - posEstimator.est.pos.V.Z;
    }

    /* Correction step: Z-axis */
    if (useGpsZPos || isBaroValid) {
        float gpsWeightScaler = 1.0f;

#if defined(BARO)
        if (isBaroValid) {
            /* Apply only baro correction, no sonar */
            inavFilterCorrectPos(Z, dt, baroResidual, positionEstimationConfig()->w_z_baro_p);

            /* Adjust EPV */
            newEPV = updateEPE(posEstimator.est.epv, dt, posEstimator.baro.epv, positionEstimationConfig()->w_z_baro_p);
        }
#endif

        /* Apply GPS correction to altitude */
        if (useGpsZPos) {
            /*
            gpsWeightScaler = scaleRangef(bellCurve(gpsResidual[Z][0], INAV_GPS_ACCEPTANCE_EPE), 0.0f, 1.0f, 0.1f, 1.0f);
            inavFilterCorrectPos(Z, dt, gpsResidual[Z][0], positionEstimationConfig()->w_xy_gps_p * gpsWeightScaler);
            */
            inavFilterCorrectPos(Z, dt, gpsResidual[Z][0], positionEstimationConfig()->w_xy_gps_p * gpsWeightScaler);

            /* Adjust EPV */
            newEPV = updateEPE(posEstimator.est.epv, dt, MAX(posEstimator.gps.epv, gpsResidual[Z][0]), positionEstimationConfig()->w_z_gps_p);
        }

        /* Apply GPS correction to climb rate */
        if (useGpsZVel) {
            inavFilterCorrectVel(Z, dt, gpsResidual[Z][1], positionEstimationConfig()->w_z_gps_v * sq(gpsWeightScaler));
        }
    }
    else {
        inavFilterCorrectVel(Z, dt, 0.0f - posEstimator.est.vel.V.Z, positionEstimationConfig()->w_z_res_v);
    }

    /* Correct position from GPS - always if GPS is valid */
    if (isGPSValid) {
        //const float gpsWeightScaler = scaleRangef(bellCurve(gpsResidualXYMagnitude, INAV_GPS_ACCEPTANCE_EPE), 0.0f, 1.0f, 0.1f, 1.0f);
        const float gpsWeightScaler = 1.0f;

        const float w_xy_gps_p = positionEstimationConfig()->w_xy_gps_p * gpsWeightScaler;
        const float w_xy_gps_v = positionEstimationConfig()->w_xy_gps_v * sq(gpsWeightScaler);

        inavFilterCorrectPos(X, dt, gpsResidual[X][0], w_xy_gps_p);
        inavFilterCorrectPos(Y, dt, gpsResidual[Y][0], w_xy_gps_p);

        inavFilterCorrectVel(X, dt, gpsResidual[X][1], w_xy_gps_v);
        inavFilterCorrectVel(Y, dt, gpsResidual[Y][1], w_xy_gps_v);

        /* Adjust EPH */
        newEPH = updateEPE(posEstimator.est.eph, dt, MAX(posEstimator.gps.eph, gpsResidualXYMagnitude), positionEstimationConfig()->w_xy_gps_p);
    }
    else {
        inavFilterCorrectVel(X, dt, 0.0f - posEstimator.est.vel.V.X, positionEstimationConfig()->w_xy_res_v);
        inavFilterCorrectVel(Y, dt, 0.0f - posEstimator.est.vel.V.Y, positionEstimationConfig()->w_xy_res_v);
    }

    /* Correct accelerometer bias */
    if (positionEstimationConfig()->w_acc_bias > 0) {
        accelBiasCorr.V.X = 0;
        accelBiasCorr.V.Y = 0;
        accelBiasCorr.V.Z = 0;

        /* accelerometer bias correction for GPS */
        if (isGPSValid) {
            accelBiasCorr.V.X -= gpsResidual[X][0] * sq(positionEstimationConfig()->w_xy_gps_p);
            accelBiasCorr.V.Y -= gpsResidual[Y][0] * sq(positionEstimationConfig()->w_xy_gps_p);

            if (useGpsZPos) {
                accelBiasCorr.V.Z -= gpsResidual[Z][0] * sq(positionEstimationConfig()->w_z_gps_p);
            }
        }

        /* accelerometer bias correction for baro */
        if (isBaroValid && !isAirCushionEffectDetected) {
            accelBiasCorr.V.Z -= baroResidual * sq(positionEstimationConfig()->w_z_baro_p);
        }

        const float accelBiasCorrMagnitudeSq = sq(accelBiasCorr.V.X) + sq(accelBiasCorr.V.Y) + sq(accelBiasCorr.V.Z);
        if (accelBiasCorrMagnitudeSq < sq(INAV_ACC_BIAS_ACCEPTANCE_VALUE)) {
            /* transform error vector from NEU frame to body frame */
            imuTransformVectorEarthToBody(&accelBiasCorr);

            /* Correct accel bias */
            posEstimator.imu.accelBias.V.X += accelBiasCorr.V.X * positionEstimationConfig()->w_acc_bias * dt;
            posEstimator.imu.accelBias.V.Y += accelBiasCorr.V.Y * positionEstimationConfig()->w_acc_bias * dt;
            posEstimator.imu.accelBias.V.Z += accelBiasCorr.V.Z * positionEstimationConfig()->w_acc_bias * dt;
        }
    }

    /* Surface offset */
#ifdef USE_RANGEFINDER
    posEstimator.est.surface = posEstimator.est.surface + posEstimator.est.surfaceVel * dt;

    if (isSurfaceValid) {
        const float surfaceResidual = posEstimator.surface.alt - posEstimator.est.surface;
        const float bellCurveScaler = scaleRangef(bellCurve(surfaceResidual, 50.0f), 0.0f, 1.0f, 0.1f, 1.0f);

        posEstimator.est.surface += surfaceResidual * positionEstimationConfig()->w_z_surface_p * bellCurveScaler * dt;
        posEstimator.est.surfaceVel += surfaceResidual * positionEstimationConfig()->w_z_surface_v * sq(bellCurveScaler) * dt;
        posEstimator.est.surfaceValid = true;
    }
    else {
        posEstimator.est.surfaceVel = 0; // Zero out velocity to prevent estimate to drift away
        posEstimator.est.surfaceValid = false;
    }

#else
    posEstimator.est.surface = -1;
    posEstimator.est.surfaceVel = 0;
    posEstimator.est.surfaceValid = false;
#endif

    /* Update uncertainty */
    posEstimator.est.eph = newEPH;
    posEstimator.est.epv = newEPV;
}

/**
 * Examine estimation error and update navigation system if estimate is good enough
 *  Function is called at main loop rate, but updates happen less frequently - at a fixed rate
 */
static void publishEstimatedTopic(timeUs_t currentTimeUs)
{
    static navigationTimer_t posPublishTimer;

    /* IMU operates in decidegrees while INAV operates in deg*100 */
    updateActualHeading(DECIDEGREES_TO_CENTIDEGREES(attitude.values.yaw));

    /* Position and velocity are published with INAV_POSITION_PUBLISH_RATE_HZ */
    if (updateTimer(&posPublishTimer, HZ2US(INAV_POSITION_PUBLISH_RATE_HZ), currentTimeUs)) {
        /* Publish position update */
        if (posEstimator.est.eph < positionEstimationConfig()->max_eph_epv) {
            updateActualHorizontalPositionAndVelocity(true, posEstimator.est.pos.V.X, posEstimator.est.pos.V.Y, posEstimator.est.vel.V.X, posEstimator.est.vel.V.Y);
        }
        else {
            updateActualHorizontalPositionAndVelocity(false, posEstimator.est.pos.V.X, posEstimator.est.pos.V.Y, 0, 0);
        }

        /* Publish altitude update and set altitude validity */
        if (posEstimator.est.epv < positionEstimationConfig()->max_eph_epv) {
            updateActualAltitudeAndClimbRate(true, posEstimator.est.pos.V.Z, posEstimator.est.vel.V.Z);
        }
        else {
            updateActualAltitudeAndClimbRate(false, posEstimator.est.pos.V.Z, 0);
        }

        /* Publish surface distance */
        updateActualSurfaceDistance(posEstimator.est.surfaceValid, posEstimator.est.surface, posEstimator.est.surfaceVel);

        /* Store history data */
        posEstimator.history.pos[posEstimator.history.index] = posEstimator.est.pos;
        posEstimator.history.vel[posEstimator.history.index] = posEstimator.est.vel;
        posEstimator.history.index++;
        if (posEstimator.history.index >= INAV_HISTORY_BUF_SIZE) {
            posEstimator.history.index = 0;
        }

#if defined(NAV_BLACKBOX)
        navEPH = posEstimator.est.eph;
        navEPV = posEstimator.est.epv;
#endif
    }
}

#if defined(NAV_GPS_GLITCH_DETECTION)
bool isGPSGlitchDetected(void)
{
    return posEstimator.gps.glitchDetected;
}
#endif

/**
 * Initialize position estimator
 *  Should be called once before any update occurs
 */
void initializePositionEstimator(void)
{
    int axis;

    posEstimator.est.eph = positionEstimationConfig()->max_eph_epv + 0.001f;
    posEstimator.est.epv = positionEstimationConfig()->max_eph_epv + 0.001f;

    posEstimator.gps.lastUpdateTime = 0;
    posEstimator.baro.lastUpdateTime = 0;
    posEstimator.surface.lastUpdateTime = 0;

    posEstimator.est.surface = 0;
    posEstimator.est.surfaceVel = 0;

    posEstimator.history.index = 0;

    posEstimator.imu.gravityCalibrationComplete = false;

    for (axis = 0; axis < 3; axis++) {
        posEstimator.imu.accelBias.A[axis] = 0;
        posEstimator.est.pos.A[axis] = 0;
        posEstimator.est.vel.A[axis] = 0;
    }

    memset(&posEstimator.history.pos[0], 0, sizeof(posEstimator.history.pos));
    memset(&posEstimator.history.vel[0], 0, sizeof(posEstimator.history.vel));
}

/**
 * Update estimator
 *  Update rate: loop rate (>100Hz)
 */
void updatePositionEstimator(void)
{
    static bool isInitialized = false;

    if (!isInitialized) {
        initializePositionEstimator();
        isInitialized = true;
    }

    const timeUs_t currentTimeUs = micros();

    /* Periodic sensor updates */
#if defined(PITOT)
    updatePitotTopic(currentTimeUs);
#endif

    /* Read updates from IMU, preprocess */
    updateIMUTopic();

    /* Update estimate */
    updateEstimatedTopic(currentTimeUs);

    /* Publish estimate */
    publishEstimatedTopic(currentTimeUs);
}

bool navIsCalibrationComplete(void)
{
    return posEstimator.imu.gravityCalibrationComplete;
}

#endif
