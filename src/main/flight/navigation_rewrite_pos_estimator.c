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

#include "build_config.h"
#include "platform.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "io/gps.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"

#include "config/runtime_config.h"
#include "config/config.h"

#if defined(NAV)

/**
 * Model-identification based position estimator
 * Based on INAV position estimator for PX4 by Anton Babushkin <anton.babushkin@me.com>
 * @author Konstantin Sharlaimov <konstantin.sharlaimov@gmail.com>
 */
#define INAV_GPS_EPV        500.0f  // 5m GPS VDOP
#define INAV_GPS_EPH        200.0f  // 2m GPS HDOP  (gives about 1.6s of dead-reckoning if GPS is temporary lost)

#define INAV_GPS_GLITCH_RADIUS              250.0f  // 2.5m GPS glitch radius
#define INAV_GPS_GLITCH_ACCEL               1000.0f // 10m/s/s max possible acceleration for GPS glitch detection

#define INAV_POSITION_PUBLISH_RATE_HZ       50      // Publish position updates at this rate
#define INAV_BARO_UPDATE_RATE               20
#define INAV_SONAR_UPDATE_RATE              15      // Sonar is limited to 1/60ms update rate, go lower that that

#define INAV_GPS_TIMEOUT_MS                 1500    // GPS timeout
#define INAV_BARO_TIMEOUT_MS                200     // Baro timeout
#define INAV_SONAR_TIMEOUT_MS               200     // Sonar timeout    (missed 3 readings in a row)

#define INAV_SONAR_W1                       0.8461f // Sonar predictive filter gain for altitude
#define INAV_SONAR_W2                       6.2034f // Sonar predictive filter gain for velocity
#define INAV_SONAR_MAX_DISTANCE             70      // Sonar is unreliable above 70cm due to noise from propellers

#define INAV_HISTORY_BUF_SIZE               (INAV_POSITION_PUBLISH_RATE_HZ / 2)     // Enough to hold 0.5 sec historical data

extern float magneticDeclination;

typedef struct {
    uint32_t    lastTriggeredTime;
    uint32_t    deltaTime;
} navigationTimer_t;

typedef struct {
    uint32_t    lastUpdateTime; // Last update time (us)
#if defined(INAV_ENABLE_GPS_GLITCH_DETECTION)
    bool        glitchDetected;
    bool        glitchRecovery;
#endif
    t_fp_vector pos;            // GPS position in NEU coordinate system (cm)
    t_fp_vector vel;            // GPS velocity (cms)
    float       eph;
    float       epv;
} navPositionEstimatorGPS_t;

typedef struct {
    uint32_t    lastUpdateTime; // Last update time (us)
    float       alt;            // Raw barometric altitude (cm)
    float       epv;
} navPositionEstimatorBARO_t;

typedef struct {
    uint32_t    lastUpdateTime; // Last update time (us)
    float       alt;            // Raw altitude measurement (cm)
    float       vel;
} navPositionEstimatorSONAR_t;

typedef struct {
    uint32_t    lastUpdateTime; // Last update time (us)
    t_fp_vector pos;
    t_fp_vector vel;
    float       surface;
    float       surfaceVel;
    float       eph;
    float       epv;
} navPositionEstimatorESTIMATE_t;

typedef struct {
    uint8_t     index;
    t_fp_vector pos[INAV_HISTORY_BUF_SIZE];
    t_fp_vector vel[INAV_HISTORY_BUF_SIZE];
} navPosisitonEstimatorHistory_t;

typedef struct {
    t_fp_vector     accelNEU;
    t_fp_vector     accelBias;
} navPosisitonEstimatorIMU_t;

typedef struct {
    // Data sources
    navPositionEstimatorGPS_t   gps;
    navPositionEstimatorBARO_t  baro;
    navPositionEstimatorSONAR_t sonar;

    // IMU data
    navPosisitonEstimatorIMU_t  imu;

    // Estimate
    navPositionEstimatorESTIMATE_t  est;

    // Estimation history
    navPosisitonEstimatorHistory_t  history;
} navigationPosEstimator_s;

static navigationPosEstimator_s posEstimator;

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

#define resetTimer(tim, currentTime) { (tim)->deltaTime = 0; (tim)->lastTriggeredTime = currentTime; }
#define getTimerDeltaMicros(tim) ((tim)->deltaTime)
static bool updateTimer(navigationTimer_t * tim, uint32_t interval, uint32_t currentTime)
{
    if ((currentTime - tim->lastTriggeredTime) >= interval) {
        tim->deltaTime = currentTime - tim->lastTriggeredTime;
        tim->lastTriggeredTime = currentTime;
        return true;
    }
    else {
        return false;
    }
}

#if defined(GPS)
/* Why is this here: Because GPS will be sending at quiet a nailed rate (if not overloaded by junk tasks at the brink of its specs)
 * but we might read out with timejitter because Irq might be off by a few us so we do a +-10% margin around the time between GPS
 * datasets representing the most common Hz-rates today. You might want to extend the list or find a smarter way.
 * Don't overload your GPS in its config with trash, choose a Hz rate that it can deliver at a sustained rate.
 * (c) CrashPilot1000
 */
static uint32_t getGPSDeltaTimeFilter(uint32_t dTus)
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

#if defined(INAV_ENABLE_GPS_GLITCH_DETECTION)
static bool detectGPSGlitch(t_fp_vector * newLocalPos, float dT)
{
    t_fp_vector predictedGpsPosition;
    float gpsDistance;

    /* We predict new position based on previous GPS velocity and position */
    predictedGpsPosition.V.X = posEstimator.gps.pos.V.X + posEstimator.gps.vel.V.X * dT;
    predictedGpsPosition.V.Y = posEstimator.gps.pos.V.Y + posEstimator.gps.vel.V.Y * dT;

    /* Calculate position error */
    gpsDistance = sqrtf(sq(predictedGpsPosition.V.X - newLocalPos->V.X) + sq(predictedGpsPosition.V.Y - newLocalPos->V.Y));

    /* Condition 1: New pos is within predefined radius of predicted pos */
    if (gpsDistance > INAV_GPS_GLITCH_RADIUS)
        return true;

    /* Condition 2: New position must be reachable within INAV_GPS_GLITCH_ACCEL * dt * dt from previous position */
    if (gpsDistance > (0.5f * INAV_GPS_GLITCH_ACCEL * dT * dT))
        return true;

    return false;
}
#endif

/**
 * Update GPS topic
 *  Function is called on each GPS update
 */
void onNewGPSData(void)
{
    static uint32_t lastGPSNewDataTime;
    static int32_t previousLat;
    static int32_t previousLon;
    static int32_t previousAlt;
    static bool isFirstGPSUpdate = true;

    gpsLocation_t newLLH;
    uint32_t currentTime = micros();

    newLLH.lat = gpsSol.llh.lat;
    newLLH.lon = gpsSol.llh.lon;
    newLLH.alt = gpsSol.llh.alt;

    if (sensors(SENSOR_GPS)) {
        if (!(STATE(GPS_FIX) && gpsSol.numSat >= posControl.navConfig->inav.gps_min_sats)) {
            isFirstGPSUpdate = true;
            return;
        }

        if ((currentTime - lastGPSNewDataTime) > MS2US(INAV_GPS_TIMEOUT_MS)) {
            isFirstGPSUpdate = true;
        }

#if defined(INAV_ENABLE_AUTO_MAG_DECLINATION)
        /* Automatic magnetic declination calculation - do this once */
        static bool magDeclinationSet = false;
        if (posControl.navConfig->inav.automatic_mag_declination && !magDeclinationSet) {
            magneticDeclination = geoCalculateMagDeclination(&newLLH) * 10.0f; // heading is in 0.1deg units
            magDeclinationSet = true;
        }
#endif

        /* Process position update if GPS origin is already set, or precision is good enough */
        // FIXME: use HDOP here
        if ((posControl.gpsOrigin.valid) || (gpsSol.numSat >= posControl.navConfig->inav.gps_min_sats)) {
            /* Convert LLH position to local coordinates */
            t_fp_vector newLocalPos;
            geoConvertGeodeticToLocal(&posControl.gpsOrigin, &newLLH, &newLocalPos, GEO_ALT_ABSOLUTE);

            /* If not the first update - calculate velocities */
            if (!isFirstGPSUpdate) {
                float dT = US2S(getGPSDeltaTimeFilter(currentTime - lastGPSNewDataTime));

#if defined(INAV_ENABLE_GPS_GLITCH_DETECTION)
                /* GPS glitch protection */
                if (detectGPSGlitch(&newLocalPos, dT)) {
                    posEstimator.gps.glitchRecovery = false;
                    posEstimator.gps.glitchDetected = true;
                }
                else {
                    /* Store previous glitch flag in glitchRecovery to indicate a valid reading after a glitch */
                    posEstimator.gps.glitchRecovery = posEstimator.gps.glitchDetected;
                    posEstimator.gps.glitchDetected = false;
                }
#endif

                /* Even if GPS glitch is detected we continue to update GPS position and velocity to always have a previous reading */
                posEstimator.gps.pos = newLocalPos;

                /* Use VELNED provided by GPS if available, calculate from coordinates otherwise */
                float gpsScaleLonDown = constrainf(cos_approx((ABS(gpsSol.llh.lat) / 10000000.0f) * 0.0174532925f), 0.01f, 1.0f);
                if (posControl.navConfig->inav.use_gps_velned && gpsSol.flags.validVelNE) {
                    posEstimator.gps.vel.V.X = gpsSol.velNED[0];
                    posEstimator.gps.vel.V.Y = gpsSol.velNED[1];
                }
                else {
                    posEstimator.gps.vel.V.X = (posEstimator.gps.vel.V.X + (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * (gpsSol.llh.lat - previousLat) / dT)) / 2.0f;
                    posEstimator.gps.vel.V.Y = (posEstimator.gps.vel.V.Y + (gpsScaleLonDown * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * (gpsSol.llh.lon - previousLon) / dT)) / 2.0f;
                }

                if (posControl.navConfig->inav.use_gps_velned && gpsSol.flags.validVelD) {
                    posEstimator.gps.vel.V.Z = - gpsSol.velNED[2];   // NEU
                }
                else {
                    posEstimator.gps.vel.V.Z = (posEstimator.gps.vel.V.Z + (gpsSol.llh.alt - previousAlt) / dT) / 2.0f;
                }

                /* FIXME: use HDOP/VDOP */
                posEstimator.gps.eph = INAV_GPS_EPH;
                posEstimator.gps.epv = INAV_GPS_EPV;

                /* Indicate a last valid reading of Pos/Vel */
                posEstimator.gps.lastUpdateTime = currentTime;
            }

            previousLat = gpsSol.llh.lat;
            previousLon = gpsSol.llh.lon;
            previousAlt = gpsSol.llh.alt;
            isFirstGPSUpdate = false;

            lastGPSNewDataTime = currentTime;
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
 *  Function is called at main loop rate, updates happen at reduced rate
 */
static void updateBaroTopic(uint32_t currentTime)
{
    static navigationTimer_t baroUpdateTimer;

    if (updateTimer(&baroUpdateTimer, HZ2US(INAV_BARO_UPDATE_RATE), currentTime)) {
        float newBaroAlt = baroCalculateAltitude();
        if (sensors(SENSOR_BARO) && isBaroCalibrationComplete()) {
            posEstimator.baro.alt = newBaroAlt;
            posEstimator.baro.epv = posControl.navConfig->inav.baro_epv;
            posEstimator.baro.lastUpdateTime = currentTime;
        }
        else {
            posEstimator.baro.alt = 0;
            posEstimator.baro.lastUpdateTime = 0;
        }
    }
}
#endif

#if defined(SONAR)
/**
 * Read sonar and update alt/vel topic
 *  Function is called at main loop rate, updates happen at reduced rate
 */
static void updateSonarTopic(uint32_t currentTime)
{
    static navigationTimer_t sonarUpdateTimer;

    if (updateTimer(&sonarUpdateTimer, HZ2US(INAV_SONAR_UPDATE_RATE), currentTime)) {
        if (sensors(SENSOR_SONAR)) {
            /* Read sonar */
            float newSonarAlt = sonarRead();
            newSonarAlt = sonarCalculateAltitude(newSonarAlt, calculateCosTiltAngle());

            /* Apply predictive filter to sonar readings (inspired by PX4Flow) */
            if (posEstimator.sonar.alt > 0 && posEstimator.sonar.alt <= INAV_SONAR_MAX_DISTANCE) {
                float sonarPredVel, sonarPredAlt;
                float sonarDt = (currentTime - posEstimator.sonar.lastUpdateTime) * 1e-6;
                posEstimator.sonar.lastUpdateTime = currentTime;

                sonarPredVel = (sonarDt < 0.25f) ? posEstimator.sonar.vel : 0.0f;
                sonarPredAlt = posEstimator.sonar.alt + sonarPredVel * sonarDt;

                posEstimator.sonar.alt = sonarPredAlt + INAV_SONAR_W1 * (newSonarAlt - sonarPredAlt);
                posEstimator.sonar.vel = sonarPredVel + INAV_SONAR_W2 * (newSonarAlt - sonarPredAlt);
            }
        }
        else {
            /* No sonar */
            posEstimator.sonar.alt = 0;
            posEstimator.sonar.vel = 0;
            posEstimator.sonar.lastUpdateTime = 0;
        }
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

    if (!isImuReady()) {
        posEstimator.imu.accelNEU.V.X = 0;
        posEstimator.imu.accelNEU.V.Y = 0;
        posEstimator.imu.accelNEU.V.Z = 0;
    }
    else {
        t_fp_vector accelBF;

        /* Read acceleration data in body frame */
        accelBF.V.X = imuAccelInBodyFrame.V.X;
        accelBF.V.Y = imuAccelInBodyFrame.V.Y;
        accelBF.V.Z = imuAccelInBodyFrame.V.Z;

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
        //if (!ARMING_FLAG(ARMED) && imuRuntimeConfig->acc_unarmedcal) {
        if (!ARMING_FLAG(ARMED) && posControl.navConfig->inav.accz_unarmed_cal) {
            // Slowly converge on calibrated gravity while level
            calibratedGravityCMSS += (posEstimator.imu.accelNEU.V.Z - calibratedGravityCMSS) * 0.0025f;
        }

        posEstimator.imu.accelNEU.V.Z -= calibratedGravityCMSS;
    }
}

/**
 * Calculate next estimate using IMU and apply corrections from reference sensors (GPS, BARO etc)
 *  Function is called at main loop rate
 */
static void updateEstimatedTopic(uint32_t currentTime)
{
    t_fp_vector accelBiasCorr;
    float dt = US2S(currentTime - posEstimator.est.lastUpdateTime);
    posEstimator.est.lastUpdateTime = currentTime;

    /* If IMU is not ready we can't estimate anything */
    if (!isImuReady()) {
        posEstimator.est.eph = posControl.navConfig->inav.max_eph_epv + 0.001f;
        posEstimator.est.epv = posControl.navConfig->inav.max_eph_epv + 0.001f;
        return;
    }

    /* increase EPH/EPV on each iteration */
    if (posEstimator.est.eph <= posControl.navConfig->inav.max_eph_epv) {
        posEstimator.est.eph *= 1.0f + dt;
    }

    if (posEstimator.est.epv <= posControl.navConfig->inav.max_eph_epv) {
        posEstimator.est.epv *= 1.0f + dt;
    }

    /* Figure out if we have valid position data from our data sources */
    bool isGPSValid = sensors(SENSOR_GPS) && posControl.gpsOrigin.valid && ((currentTime - posEstimator.gps.lastUpdateTime) <= MS2US(INAV_GPS_TIMEOUT_MS));
    bool isBaroValid = sensors(SENSOR_BARO) && ((currentTime - posEstimator.baro.lastUpdateTime) <= MS2US(INAV_BARO_TIMEOUT_MS));
    bool isSonarValid = sensors(SENSOR_SONAR) && ((currentTime - posEstimator.sonar.lastUpdateTime) <= MS2US(INAV_SONAR_TIMEOUT_MS));

#if defined(INAV_ENABLE_GPS_GLITCH_DETECTION)
    //isGPSValid = isGPSValid && !posEstimator.gps.glitchDetected;
#endif

    /* Apply GPS altitude corrections only on fixed wing aircrafts */
    bool useGpsZ = STATE(FIXED_WING) && isGPSValid;

    /* Pre-calculate history index for GPS delay compensation */
    int gpsHistoryIndex = (posEstimator.history.index - 1) - constrain(((int)posControl.navConfig->inav.gps_delay_ms / (1000 / INAV_POSITION_PUBLISH_RATE_HZ)), 0, INAV_HISTORY_BUF_SIZE - 1);
    if (gpsHistoryIndex < 0) {
        gpsHistoryIndex += INAV_HISTORY_BUF_SIZE;
    }

    /* Correct accelerometer bias */
    if (posControl.navConfig->inav.w_acc_bias > 0) {
        accelBiasCorr.V.X = 0;
        accelBiasCorr.V.Y = 0;
        accelBiasCorr.V.Z = 0;

        /* accelerometer bias correction for GPS */
        if (isGPSValid) {
            accelBiasCorr.V.X -= (posEstimator.gps.pos.V.X - posEstimator.history.pos[gpsHistoryIndex].V.X) * sq(posControl.navConfig->inav.w_xy_gps_p);
            accelBiasCorr.V.X -= (posEstimator.gps.vel.V.X - posEstimator.history.vel[gpsHistoryIndex].V.X) * posControl.navConfig->inav.w_xy_gps_v;
            accelBiasCorr.V.Y -= (posEstimator.gps.pos.V.Y - posEstimator.history.pos[gpsHistoryIndex].V.Y) * sq(posControl.navConfig->inav.w_xy_gps_p);
            accelBiasCorr.V.Y -= (posEstimator.gps.vel.V.Y - posEstimator.history.vel[gpsHistoryIndex].V.Y) * posControl.navConfig->inav.w_xy_gps_v;

            if (useGpsZ) {
                accelBiasCorr.V.Z -= (posEstimator.gps.pos.V.Z - posEstimator.history.pos[gpsHistoryIndex].V.Z) * sq(posControl.navConfig->inav.w_z_gps_p);
                accelBiasCorr.V.Z -= (posEstimator.gps.vel.V.Z - posEstimator.history.vel[gpsHistoryIndex].V.Z) * posControl.navConfig->inav.w_z_gps_v;
            }
        }

        /* accelerometer bias correction for baro */
        if (isBaroValid) {
            accelBiasCorr.V.Z -= (posEstimator.baro.alt - posEstimator.est.pos.V.Z) * sq(posControl.navConfig->inav.w_z_baro_p);
        }

        /* transform error vector from NEU frame to body frame */
        imuTransformVectorEarthToBody(&accelBiasCorr);

        /* Correct accel bias */
        posEstimator.imu.accelBias.V.X += accelBiasCorr.V.X * posControl.navConfig->inav.w_acc_bias * dt;
        posEstimator.imu.accelBias.V.Y += accelBiasCorr.V.Y * posControl.navConfig->inav.w_acc_bias * dt;
        posEstimator.imu.accelBias.V.Z += accelBiasCorr.V.Z * posControl.navConfig->inav.w_acc_bias * dt;
    }

    /* Estimate Z-axis */
    if ((posEstimator.est.epv < posControl.navConfig->inav.max_eph_epv) || useGpsZ || isBaroValid) {
        /* Predict position/velocity based on acceleration */
        inavFilterPredict(Z, dt, posEstimator.imu.accelNEU.V.Z);

#if defined(BARO)
        if (isBaroValid) {
            /* Apply only baro correction, no sonar */
            inavFilterCorrectPos(Z, dt, posEstimator.baro.alt - posEstimator.est.pos.V.Z, posControl.navConfig->inav.w_z_baro_p);

            /* Adjust EPV */
            posEstimator.est.epv = MIN(posEstimator.est.epv, posEstimator.baro.epv);
        }
#endif

        /* Apply GPS correction to altitude */
        if (useGpsZ) {
            inavFilterCorrectPos(Z, dt, posEstimator.gps.pos.V.Z - posEstimator.history.pos[gpsHistoryIndex].V.Z, posControl.navConfig->inav.w_z_gps_p);
            inavFilterCorrectVel(Z, dt, posEstimator.gps.vel.V.Z - posEstimator.history.vel[gpsHistoryIndex].V.Z, posControl.navConfig->inav.w_z_gps_v);

            /* Adjust EPV */
            posEstimator.est.epv = MIN(posEstimator.est.epv, posEstimator.gps.epv);
        }
    }
    else {
        inavFilterCorrectVel(Z, dt, 0.0f - posEstimator.est.vel.V.Z, posControl.navConfig->inav.w_z_res_v);
    }

    /* Estimate XY-axis only if heading is valid (X-Y acceleration is North-East)*/
    if ((posEstimator.est.eph < posControl.navConfig->inav.max_eph_epv) || isGPSValid) {
        if (isImuHeadingValid()) {
            inavFilterPredict(X, dt, posEstimator.imu.accelNEU.V.X);
            inavFilterPredict(Y, dt, posEstimator.imu.accelNEU.V.Y);
        }

        /* Correct position from GPS - always if GPS is valid */
        if (isGPSValid) {
            inavFilterCorrectPos(X, dt, posEstimator.gps.pos.V.X - posEstimator.history.pos[gpsHistoryIndex].V.X, posControl.navConfig->inav.w_xy_gps_p);
            inavFilterCorrectPos(Y, dt, posEstimator.gps.pos.V.Y - posEstimator.history.pos[gpsHistoryIndex].V.Y, posControl.navConfig->inav.w_xy_gps_p);

            inavFilterCorrectVel(X, dt, posEstimator.gps.vel.V.X - posEstimator.history.vel[gpsHistoryIndex].V.X, posControl.navConfig->inav.w_xy_gps_v);
            inavFilterCorrectVel(Y, dt, posEstimator.gps.vel.V.Y - posEstimator.history.vel[gpsHistoryIndex].V.Y, posControl.navConfig->inav.w_xy_gps_v);

            /* Adjust EPH */
            posEstimator.est.eph = MIN(posEstimator.est.eph, posEstimator.gps.eph);
        }
    }
    else {
        inavFilterCorrectVel(X, dt, 0.0f - posEstimator.est.vel.V.X, posControl.navConfig->inav.w_xy_res_v);
        inavFilterCorrectVel(Y, dt, 0.0f - posEstimator.est.vel.V.Y, posControl.navConfig->inav.w_xy_res_v);
    }

    /* Surface offset */
#if defined(SONAR)
    if (isSonarValid) {
        posEstimator.est.surface = posEstimator.sonar.alt;
        posEstimator.est.surfaceVel = posEstimator.sonar.vel;
    }
    else {
        posEstimator.est.surface = -1;
        posEstimator.est.surfaceVel = 0;
    }
#else
    posEstimator.est.surface = -1;
    posEstimator.est.surfaceVel = 0;
#endif
}

/**
 * Examine estimation error and update navigation system if estimate is good enough
 *  Function is called at main loop rate, but updates happen less frequently - at a fixed rate
 */
static void publishEstimatedTopic(uint32_t currentTime)
{
    static navigationTimer_t posPublishTimer;

    /* IMU operates in decidegrees while INAV operates in deg*100 */
    updateActualHeading(DECIDEGREES_TO_CENTIDEGREES(attitude.values.yaw));

    /* Position and velocity are published with INAV_POSITION_PUBLISH_RATE_HZ */
    if (updateTimer(&posPublishTimer, HZ2US(INAV_POSITION_PUBLISH_RATE_HZ), currentTime)) {
        /* Publish position update */
        if (posEstimator.est.eph < posControl.navConfig->inav.max_eph_epv) {
            updateActualHorizontalPositionAndVelocity(true, posEstimator.est.pos.V.X, posEstimator.est.pos.V.Y, posEstimator.est.vel.V.X, posEstimator.est.vel.V.Y);
        }
        else {
            updateActualHorizontalPositionAndVelocity(false, posEstimator.est.pos.V.X, posEstimator.est.pos.V.Y, 0, 0);
        }

        /* Publish altitude update and set altitude validity */
        if (posEstimator.est.epv < posControl.navConfig->inav.max_eph_epv) {
            updateActualAltitudeAndClimbRate(true, posEstimator.est.pos.V.Z, posEstimator.est.vel.V.Z);
        }
        else {
            updateActualAltitudeAndClimbRate(false, posEstimator.est.pos.V.Z, 0);
        }

        /* Publish surface distance */
        if (posEstimator.est.surface > 0) {
            updateActualSurfaceDistance(true, posEstimator.est.surface, posEstimator.est.surfaceVel);
        }
        else {
            updateActualSurfaceDistance(false, -1, 0);
        }

        /* Store history data */
        posEstimator.history.pos[posEstimator.history.index] = posEstimator.est.pos;
        posEstimator.history.vel[posEstimator.history.index] = posEstimator.est.vel;
        posEstimator.history.index++;
        if (posEstimator.history.index >= INAV_HISTORY_BUF_SIZE) {
            posEstimator.history.index = 0;
        }
    }
}

bool isGPSGlitchDetected(void)
{
    return posEstimator.gps.glitchDetected;
}

/**
 * Initialize position estimator
 *  Should be called once before any update occurs
 */
void initializePositionEstimator(void)
{
    int axis;

    posEstimator.est.eph = posControl.navConfig->inav.max_eph_epv + 0.001f;
    posEstimator.est.epv = posControl.navConfig->inav.max_eph_epv + 0.001f;

    posEstimator.gps.lastUpdateTime = 0;
    posEstimator.baro.lastUpdateTime = 0;
    posEstimator.sonar.lastUpdateTime = 0;

    posEstimator.history.index = 0;

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

    uint32_t currentTime = micros();

    /* Periodic sensor updates */
#if defined(BARO)
    updateBaroTopic(currentTime);
#endif

#if defined(SONAR)
    updateSonarTopic(currentTime);
#endif

    /* Read updates from IMU, preprocess */
    updateIMUTopic();

    /* Update estimate */
    updateEstimatedTopic(currentTime);

    /* Publish estimate */
    publishEstimatedTopic(currentTime);
}

#endif