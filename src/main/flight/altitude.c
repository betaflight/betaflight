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
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "platform.h"

#if defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER)

#include "build/debug.h"

#include "drivers/system.h"
#include "drivers/time.h"

#include "common/maths.h"
#include "common/filter.h"
#include "common/time.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/rangefinder.h"


#include "fc/runtime_config.h"

#include "io/gps.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/kalman_filter_1d.h"
#include "flight/altitude.h"


#define ACC_VAR_COEFF (1.0f)
#define ACC_CONST_VAR (10.0f)

#define BARO_VAR_ALT_COEFF  (0.05f)
#define BARO_VAR_VEL_COEFF  (0.01f)
#define BARO_VAR_TEMP_COEFF (0.5f)

#define GPS_VAR_VDOP_COEFF (1.0f)

#define RANGEFINDER_ACC_ERROR_THRESH (50.0f) 
#define RANGEFINDER_RAPID_ERR_THRESH (100.0f)

typedef struct sensorState_s {
    float currentAltReadingCm;
    float previousAltReadingCm;
    float zeroAltOffsetCm;
    float filteredAltReadingCm;
    float variance;
    timeMs_t deltaTimeMs;
    float VelocityCmSec;
    bool isValid;
    void (*updateReading)(struct sensorState_s *sensor);
    void (*updateVariance)(struct sensorState_s *sensor);
    void (*updateOffset)(struct sensorState_s *sensor);
} sensorState_t;

typedef struct velocity3D_s {
    uint16_t value;
    bool isValid;
} velocity3D_t;

// #ifdef USE_ACC
// void updateAccReading(sensorState_t *sensor, bool ARMING_FLAG(ARMED));
// void updateAccVariance(sensorState_t *sensor, bool ARMING_FLAG(ARMED));
// float updateAccFilter(float value);
// void updateAccItegralCallback(timeUs_t currentTimeUs);

// typedef struct accIntegral_s {
//     float vel[3];
//     timeDelta_t deltaTimeUs;
// } accIntegral_t;
// accIntegral_t accIntegral;
// #endif

#ifdef USE_BARO
void updateBaroReading(sensorState_t *sensor);
void updateBaroVariance(sensorState_t *sensor);
#endif

#ifdef USE_GPS
void updateGpsReading(sensorState_t *sensor);
void updateGpsVariance(sensorState_t *sensor);
#endif

#ifdef USE_RANGEFINDER
void updateRangefinderReading(sensorState_t *sensor);
void updateRangefinderVariance(sensorState_t *sensor);
void updateRangefinderOffset(sensorState_t *sensor);
#endif

void updateSensorOffset(sensorState_t *sensor);

typedef enum {
#ifdef USE_BARO
    SF_BARO,
#endif
#ifdef USE_GPS
    SF_GPS,
#endif
// the only must for the order of the sensors is that the local sensors like the rangefinder should be last after the global sensors like the GPS and Baro
#ifdef USE_RANGEFINDER
    SF_RANGEFINDER,
#endif
    SENSOR_COUNT
    } altSensor_e;

sensorState_t altSenFusSources[SENSOR_COUNT];
altitudeState_t altitudeState;
velocity3D_t velocityDm3D; // for Decimeter/sec 0.1m/s, coming from the GPS if available
static KalmanFilter kf;

void altSensorFusionInit(void) {
#ifdef USE_BARO
    altSenFusSources[SF_BARO].updateReading  = updateBaroReading;
    altSenFusSources[SF_BARO].updateVariance = updateBaroVariance;
    altSenFusSources[SF_BARO].updateOffset   = updateSensorOffset;
    altSenFusSources[SF_BARO].currentAltReadingCm = 0;
    altSenFusSources[SF_BARO].variance = 0.0f;
    altSenFusSources[SF_BARO].zeroAltOffsetCm = 0.0f;
    altSenFusSources[SF_BARO].isValid = isBaroReady();
#endif

#ifdef USE_GPS
    altSenFusSources[SF_GPS].updateReading  = updateGpsReading;
    altSenFusSources[SF_GPS].updateVariance = updateGpsVariance;
    altSenFusSources[SF_GPS].updateOffset   = updateSensorOffset;
    altSenFusSources[SF_GPS].currentAltReadingCm = 0;
    altSenFusSources[SF_GPS].variance = 0.0f;
    altSenFusSources[SF_GPS].zeroAltOffsetCm = 0.0f;
    altSenFusSources[SF_GPS].isValid = false;
#endif

#ifdef USE_RANGEFINDER
    altSenFusSources[SF_RANGEFINDER].updateReading  = updateRangefinderReading;
    altSenFusSources[SF_RANGEFINDER].updateVariance = updateRangefinderVariance;
    altSenFusSources[SF_RANGEFINDER].updateOffset   = updateRangefinderOffset;
    altSenFusSources[SF_RANGEFINDER].currentAltReadingCm = 0;
    altSenFusSources[SF_RANGEFINDER].variance = 0.0f;
    altSenFusSources[SF_RANGEFINDER].zeroAltOffsetCm = 0.0f;
    altSenFusSources[SF_RANGEFINDER].isValid = false;
#endif

    kf_init(&kf, 0.0f, 1.0f, 1.0f);

    velocityDm3D.value = 0;
    velocityDm3D.isValid = false;

}

void altSensorFusionUpdate(void) {
    kf_update_variance(&kf);
    SensorMeasurement tempSensorMeas;
    float previousAltitude = altitudeState.value;
    for (altSensor_e sensor = 0; sensor < SENSOR_COUNT; sensor++) {
        altSenFusSources[sensor].updateReading(&altSenFusSources[sensor]);
        altSenFusSources[sensor].updateVariance(&altSenFusSources[sensor]);
        altSenFusSources[sensor].updateOffset(&altSenFusSources[sensor]);
        
        tempSensorMeas.value    = altSenFusSources[sensor].currentAltReadingCm - altSenFusSources[sensor].zeroAltOffsetCm;
        tempSensorMeas.variance = altSenFusSources[sensor].variance;
        kf_update(&kf, tempSensorMeas);
        altitudeState.value = kf.estimatedValue; // update the altitude state every loop since it maybe used in the variance estimate
    }
    altitudeState.variance = kf.estimatedVariance;
    altitudeState.velocity = (altitudeState.value - previousAltitude) * HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

    DEBUG_SET(DEBUG_ALTITUDE, 4, (int32_t)(altitudeState.value));
    DEBUG_SET(DEBUG_ALTITUDE, 5, (int32_t)(altSenFusSources[SF_BARO].currentAltReadingCm));
    DEBUG_SET(DEBUG_ALTITUDE, 6, (int32_t)(altSenFusSources[SF_GPS].currentAltReadingCm));
    DEBUG_SET(DEBUG_ALTITUDE, 7, (int32_t)(altSenFusSources[SF_RANGEFINDER].currentAltReadingCm));
}

// void updateAccItegralCallback(timeUs_t currentTimeUs) {
//     static bool firstRun = true;
//     static timeUs_t prevTimeUs = 0;
//     if (firstRun) {
//         prevTimeUs = currentTimeUs;
//         firstRun = false;
//         return;
//     }

//     timeDelta_t deltaTimeUs = cmpTimeUs(currentTimeUs, prevTimeUs);
//     for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
//         accIntegral.vel[i] += acc.accADC.v[i] * (float)deltaTimeUs / 1e6f;
//     }
//     accIntegral.deltaTimeUs += deltaTimeUs;
//     prevTimeUs = currentTimeUs;
// }

// void updateAccReading(sensorState_t *sensor, bool ARMING_FLAG(ARMED)) {
//     static float velOffset = 0;
//     static float velAccumlator = 0;
//     static float velocityDecay = 0.99f;
//     sensor->previousAltReadingCm = sensor->currentAltReadingCm;
//     // given the attiude roll and pitch angles of the drone, and the integrated acceleration in x,y and z
//     // calculate the integrated acceleration in the z direction in the world frame
//     float roll  = DEGREES_TO_RADIANS((float)attitude.values.roll / 10.0f); // integer devision to reduce noise
//     float pitch = DEGREES_TO_RADIANS((float)attitude.values.pitch / 10.0f);
    
//     float cosPitch = cosf(pitch);

//     float velWorldZ = - accIntegral.vel[0] * sinf(pitch) 
//                     +   accIntegral.vel[1] * cosPitch * sinf(roll)
//                     +   accIntegral.vel[2] * cosPitch * cosf(roll);

//     sensor->deltaTimeMs  = (float)accIntegral.deltaTimeUs / 1e3f;

//     float gravityVel = (float)accIntegral.deltaTimeUs / 1e6f; // g/Us to g/S
    
//     float velCmSec = ((velWorldZ * acc.dev.acc_1G_rec) - gravityVel) * 981.0f; // g to cm/s

//     velCmSec -= velOffset;

//     velCmSec = updateAccFilter(velCmSec);

//     if (!ARMING_FLAG(ARMED)) {
//         velOffset = 0.99f * velOffset + 0.01f * velCmSec;
//         sensor->currentAltReadingCm = 0;
//         velAccumlator = 0;
//     }        

//     // use the average velocity and cut the decimals after the first decimal
//     sensor->VelocityCmSec = (int)((velCmSec) * 100 / 2.0f) / 100.0f;
//     velAccumlator += sensor->VelocityCmSec;

//     sensor->currentAltReadingCm += velAccumlator * ((float)accIntegral.deltaTimeUs/1e6f);

//     DEBUG_SET(DEBUG_ALTITUDE, 4, (int32_t)(accIntegral.vel[2] * 1000));
//     DEBUG_SET(DEBUG_ALTITUDE, 5, (int32_t)(sensor->VelocityCmSec * 1000));
//     DEBUG_SET(DEBUG_ALTITUDE, 6, (int32_t)(velAccumlator * 1000));
//     DEBUG_SET(DEBUG_ALTITUDE, 7, (int32_t)(sensor->currentAltReadingCm * 1000));

//     accIntegral.vel[0] = 0;
//     accIntegral.vel[1] = 0;
//     accIntegral.vel[2] = 0;
//     accIntegral.deltaTimeUs = 0;

// }

// void updateAccVariance(sensorState_t *sensor, bool ARMING_FLAG(ARMED)) { // maybe we don't need this since the altitude quality is not good anyway
//     static float unVar = 0;
//     if (!armed) {
//         float newVar = ACC_VAR_COEFF * sq(sensor->currentAltReadingCm - sensor->previousAltReadingCm);
//         unarmedVar = (unarmedVar + newVar)/2;
//     } 
//     sensor->variance = unarmedVar + ACC_CONST_VAR;
// }

// float updateAccFilter(float value) {
//     static pt3Filter_t velFilter;
//     static bool firstRun = true;

//     if (firstRun) {
//         pt3FilterInit(&velFilter, 0.5);
//         firstRun = false;
//     }
    
//     return pt3FilterApply(&velFilter, value);
// }


#ifdef USE_BARO

void updateBaroReading(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    static pt2Filter_t baroLpfFilter;
    static bool firstRun = true;
    static timeMs_t prevTime = 0;
    
    if (firstRun) { // init
        const float sampleTimeS = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

        const float altitudeCutoffHz = positionConfig()->altitude_lpf / 100.0f;
        const float altitudeGain     = pt2FilterGain(altitudeCutoffHz, sampleTimeS);
        pt2FilterInit(&baroLpfFilter, altitudeGain);

        prevTime = millis();
        firstRun = false;
    }

    sensor->previousAltReadingCm = sensor->currentAltReadingCm;
    sensor->currentAltReadingCm = pt2FilterApply(&baroLpfFilter, getBaroAltitude());
    
    sensor->deltaTimeMs = millis() - prevTime;
    prevTime = millis();
}

void updateBaroVariance(sensorState_t *sensor) {
    // the baro variance can be calculated as initial variance while the drone is stationary (not armed)
    // and while armed as a funtion of the altitude, velocity and the baro temprature 

    if (!sensor->isValid) {
        return;
    }

    static float contantVariance = 0;
    static uint16_t n = 0;

    if (!ARMING_FLAG(ARMED)) {
        contantVariance = (sensor->variance * n + sq(sensor->currentAltReadingCm)) / (n + 1);
        n++;
    } else {
        sensor->variance = contantVariance
                         + BARO_VAR_VEL_COEFF  * fabsf(altitudeState.velocity) //todo: use the global velocity, but from where?
                         + BARO_VAR_TEMP_COEFF * fabsf((float)getBaroTemperature()) 
                         + BARO_VAR_ALT_COEFF  * fabsf(sensor->currentAltReadingCm);
    }
}

#endif // USE_BARO

#ifdef USE_GPS
void updateGpsReading(sensorState_t *sensor) {
    static timeMs_t prevTime = 0;
    sensor->isValid = gpsIsHealthy() && sensors(SENSOR_GPS) && STATE(GPS_FIX);
    if (!sensor->isValid) {
        velocityDm3D.isValid = false;
        return;
    }

    sensor->previousAltReadingCm = sensor->currentAltReadingCm;
    sensor->currentAltReadingCm = gpsSol.llh.altCm;
    velocityDm3D.value = gpsSol.speed3d;
    velocityDm3D.isValid = true;

    sensor->deltaTimeMs = gpsSol.time - prevTime;
    prevTime = gpsSol.time;
}

void updateGpsVariance(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    if (gpsSol.dop.vdop != 0) {
        sensor->variance = GPS_VAR_VDOP_COEFF * gpsSol.dop.vdop;
    } else {
        sensor->variance = 1000.0f; 
    }   
}

#endif // USE_GPS

#ifdef USE_RANGEFINDER

void updateRangefinderReading(sensorState_t *sensor) {
    static timeMs_t prevTime = 0;
    static bool firstRun = true;
    int32_t rfAlt = rangefinderGetLatestAltitude();

    sensor->isValid = rangefinderIsHealthy() && sensors(SENSOR_RANGEFINDER) && rfAlt >= 0;

    if (!sensor->isValid) {
        return;
    }

    if (firstRun) {
        prevTime = millis();
        firstRun = false;
    }

    sensor->previousAltReadingCm = sensor->currentAltReadingCm;
    sensor->currentAltReadingCm  = rfAlt;
    sensor->deltaTimeMs = millis() - prevTime;
    prevTime = millis();
}

void updateRangefinderVariance(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    sensor->variance = 1.0f; // is there a better way to get the variance of the rangefinder?
}

void updateRangefinderOffset(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    if (!ARMING_FLAG(ARMED)) { // default offset update when not armed
        updateSensorOffset(sensor);
    } else { // when armed the offset should be updated using the altitude state 
        static float accError = 0;
        float zeroedAlt = sensor->currentAltReadingCm - sensor->zeroAltOffsetCm;
        float error = (zeroedAlt - altitudeState.value);

        if (fabsf(error) > RANGEFINDER_RAPID_ERR_THRESH) { // if the error is too high, update the offset directly
            sensor->zeroAltOffsetCm = sensor->zeroAltOffsetCm + error;
            return;
        } else { // otherwise, add the error to the accumlated error and smoothly update the offset if the error is high enough
            accError += error;
            if (accError > RANGEFINDER_ACC_ERROR_THRESH) {
                sensor->zeroAltOffsetCm = 0.99f * sensor->zeroAltOffsetCm + 0.01f * (sensor->currentAltReadingCm - altitudeState.value);
            }
        }
    }
}
#endif // USE_RANGEFINDER

void updateSensorOffset(sensorState_t *sensor) {
    if (!sensor->isValid || ARMING_FLAG(ARMED)) {
        return;
    }
    sensor->zeroAltOffsetCm = 0.2f * sensor->currentAltReadingCm + 0.8f * sensor->zeroAltOffsetCm;
}

#endif // USE_BARO || USE_GPS || USE_RANGEFINDER