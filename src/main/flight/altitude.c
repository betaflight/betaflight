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
#include "sensors/acceleration.h"
#include "sensors/rangefinder.h"


#include "fc/runtime_config.h"

#include "io/gps.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/kalman_filter_1d.h"
#include "flight/altitude.h"


#define SENSOR_VEL_ERROR_THRESH   (1000.0f) // the error above this value excludes the sensor from the fusion and changes the offset
#define SENSOR_VEL_MAX_ERROR      (10000.0f)
#define SENSOR_MAX_PENALITY_ITERS (100)
#define SENSOR_MAX_OFFSET_ERROR   (1000.0f)

#define BARO_VAR_ALT_COEFF       (0.01f)
#define BARO_VAR_VEL_COEFF       (0.01f)
#define BARO_VAR_TEMP_COEFF      (0.01f)
#define BARO_VAR_VEL_ERROR_COEFF (1.00f)


#define RANGEFINDER_ACC_ERROR_THRESH    (50.0f) 
#define RANGEFINDER_RAPID_ERR_THRESH    (100.0f)
#define RANGEFINDER_VAR_VEL_ERROR_COEFF (2.0f)
#define RANGEFINDER_CONST_VAR           (10.0f)

#define GPS_VAR_DOP_COEFF       (1.0f)
#define GPS_VAR_VEL_ERROR_COEFF (2.0f)
#define GPS_RAPID_ERR_THRESH    (100.0f)
#define GPS_ACC_ERROR_THRESH    (200.0f)

typedef enum {
#ifdef USE_BARO
    SF_BARO,
#endif
#ifdef USE_GPS
    SF_GPS,
#endif
// the only must for the order of the sensors is that the local sensors like the rangefinder should be last after the global sensors like the GPS and Baro
#ifdef USE_ACC
    SF_ACC,
#endif
#ifdef USE_RANGEFINDER
    SF_RANGEFINDER,
#endif
    SENSOR_COUNT
    } altSensor_e;

typedef struct sensorState_s {
    float currentAltReadingCm;
    float zeroAltOffsetCm;
    float velocityCmS;
    float variance;
    float offsetError;
    uint32_t velError;
    altSensor_e type;
    uint8_t penalityIters;
    bool isValid;
    bool toFuse;
    void (*updateReading)(struct sensorState_s *sensor);
    void (*updateVariance)(struct sensorState_s *sensor);
    void (*updateOffset)(struct sensorState_s *sensor);
} sensorState_t;

typedef struct velocity3D_s {
    float value;
    bool isValid;
} velocity3D_t;

#ifdef USE_ACC
void updateAccReading(sensorState_t *sensor);
void applyAccVelFilter(float *velocity);
void updateAccItegralCallback(timeUs_t currentTimeUs);

typedef struct accIntegral_s {
    float vel[XYZ_AXIS_COUNT + 1]; // 3 axis + 1 for the 3D magnitude
    timeDelta_t deltaTimeUs;
} accIntegral_t;
accIntegral_t accIntegral;
#endif

#ifdef USE_BARO
void updateBaroReading(sensorState_t *sensor);
void updateBaroVariance(sensorState_t *sensor);
void applyBaroFilter(float *value);
#endif

#ifdef USE_GPS
void updateGpsReading(sensorState_t *sensor);
void updateGpsVariance(sensorState_t *sensor);
void updateGpsOffset(sensorState_t *sensor);
void applyGpsFilter(float *value);
#endif

#ifdef USE_RANGEFINDER
void updateRangefinderReading(sensorState_t *sensor);
void updateRangefinderVariance(sensorState_t *sensor);
void updateRangefinderOffset(sensorState_t *sensor);
void applyRangefinderFilter(float *value);
#endif

void updateSensorOffset(sensorState_t *sensor);
void updateVelError(sensorState_t *sensor);
void updateSensorFusability(sensorState_t *sensor);
void doNothing(sensorState_t *sensor);

sensorState_t altSenFusSources[SENSOR_COUNT];
altitudeState_t altitudeState;
velocity3D_t velocityCm3D;
static KalmanFilter kf;

void altSensorFusionInit(void) {
#ifdef USE_ACC
    altSenFusSources[SF_ACC].updateReading  = updateAccReading;
    altSenFusSources[SF_ACC].updateVariance = doNothing;
    altSenFusSources[SF_ACC].updateOffset   = doNothing;
    altSenFusSources[SF_ACC].type = SF_ACC;
    altSenFusSources[SF_ACC].isValid = false;
    altSenFusSources[SF_ACC].toFuse = false;
    velocityCm3D.isValid = true;
#endif
    
#ifdef USE_BARO
    altSenFusSources[SF_BARO].updateReading  = updateBaroReading;
    altSenFusSources[SF_BARO].updateVariance = updateBaroVariance;
    altSenFusSources[SF_BARO].updateOffset   = updateSensorOffset;
    altSenFusSources[SF_BARO].type = SF_BARO;
    altSenFusSources[SF_BARO].isValid = isBaroReady();
    altSenFusSources[SF_BARO].toFuse = true;
#endif

#ifdef USE_GPS
    altSenFusSources[SF_GPS].updateReading  = updateGpsReading;
    altSenFusSources[SF_GPS].updateVariance = updateGpsVariance;
    altSenFusSources[SF_GPS].updateOffset   = updateSensorOffset;
    altSenFusSources[SF_GPS].type = SF_GPS;
    altSenFusSources[SF_GPS].isValid = false;
    altSenFusSources[SF_GPS].toFuse = true;
#endif

#ifdef USE_RANGEFINDER
    altSenFusSources[SF_RANGEFINDER].updateReading  = updateRangefinderReading;
    altSenFusSources[SF_RANGEFINDER].updateVariance = updateRangefinderVariance;
    altSenFusSources[SF_RANGEFINDER].updateOffset   = updateSensorOffset;
    altSenFusSources[SF_RANGEFINDER].type = SF_RANGEFINDER;
    altSenFusSources[SF_RANGEFINDER].isValid = false;
    altSenFusSources[SF_RANGEFINDER].toFuse = true;
#endif

    kf_init(&kf, 0.0f, 1.0f, 1.0f);

    velocityCm3D.value = 0;
    velocityCm3D.isValid = false;
}

void altSensorFusionUpdate(void) {
    kf_update_variance(&kf);
    SensorMeasurement tempSensorMeas;
    float previousAltitude = altitudeState.distCm;
    for (sensorState_t * sensor = altSenFusSources; sensor < altSenFusSources + SENSOR_COUNT; sensor++) {
        sensor->updateReading(sensor);
        updateVelError(sensor);
        updateSensorFusability(sensor); // this should handle the case of sudden jumps in the sensor readings compared to the accelerometer velocity estimation
        sensor->updateVariance(sensor);
        sensor->updateOffset(sensor);
        
        if (sensor->isValid && sensor->toFuse) {
            tempSensorMeas.value    = sensor->currentAltReadingCm - sensor->zeroAltOffsetCm;
            tempSensorMeas.variance = sensor->variance;
            kf_update(&kf, tempSensorMeas);
        }
    }
    altitudeState.distCm     = kf.estimatedValue; 
    altitudeState.variance   = kf.estimatedVariance;
    altitudeState.velocityCm = (altitudeState.distCm - previousAltitude) * TASK_ALTITUDE_RATE_HZ;
    previousAltitude         = altitudeState.distCm;

   
    DEBUG_SET(DEBUG_ALTITUDE, 0, altSenFusSources[SF_BARO].isValid        ? lrintf(altSenFusSources[SF_BARO].currentAltReadingCm)        : -1);
    DEBUG_SET(DEBUG_ALTITUDE, 1, lrintf(altSenFusSources[SF_GPS].currentAltReadingCm));
    DEBUG_SET(DEBUG_ALTITUDE, 2, altSenFusSources[SF_RANGEFINDER].isValid ? lrintf(altSenFusSources[SF_RANGEFINDER].currentAltReadingCm) : -1);
    DEBUG_SET(DEBUG_ALTITUDE, 3, lrintf(altitudeState.distCm));
    DEBUG_SET(DEBUG_ALTITUDE, 4, lrintf(altSenFusSources[SF_BARO].variance));
    DEBUG_SET(DEBUG_ALTITUDE, 5, lrintf(altSenFusSources[SF_GPS].variance));
    DEBUG_SET(DEBUG_ALTITUDE, 6, lrintf(altSenFusSources[SF_RANGEFINDER].variance));
    DEBUG_SET(DEBUG_ALTITUDE, 7, lrintf(altitudeState.variance));
}


void updateSensorOffset(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    if (!ARMING_FLAG(ARMED)) {// default offset update when not armed
        sensor->zeroAltOffsetCm = 0.2f * sensor->currentAltReadingCm + 0.8f * sensor->zeroAltOffsetCm;
    } else { // when armed the offset should be updated according to the velocity error value
        // float estAlt = sensor->currentAltReadingCm - sensor->zeroAltOffsetCm;
        float newOffset = sensor->currentAltReadingCm - kf.estimatedValue;
        if (sensor->velError > SENSOR_VEL_ERROR_THRESH) {
            sensor->zeroAltOffsetCm = newOffset;
            sensor->velError = 0;
        } else { // update the offset smoothly using the error value, if the error is 0 then no update is done
            // float correctnessRatio = (float)(SENSOR_VEL_MAX_ERROR - sensor->velError) / SENSOR_VEL_MAX_ERROR;
            // sensor->zeroAltOffsetCm = (correctnessRatio * sensor->zeroAltOffsetCm) 
            //                         + ((1.0f - correctnessRatio) * newOffset);
            sensor->offsetError += newOffset - sensor->zeroAltOffsetCm;
            if (fabsf(sensor->offsetError) > SENSOR_MAX_OFFSET_ERROR) {
                sensor->zeroAltOffsetCm = 0.01f * newOffset + 0.99f * sensor->zeroAltOffsetCm;
                // decaying the error
                sensor->offsetError = 0.99f * sensor->offsetError;
            }
        }
    }
}

void updateVelError(sensorState_t *sensor) {
    if (!sensor->isValid || sensor->type == SF_ACC) {
        return;
    }
    sensor->velError = 0.1 * (sq(altSenFusSources[SF_ACC].velocityCmS - sensor->velocityCmS) / (100.f))
                     + 0.9 * sensor->velError;

    sensor->velError = constrain(sensor->velError, 0, SENSOR_VEL_MAX_ERROR);
}

void updateSensorFusability(sensorState_t *sensor) {
    if (!sensor->isValid || sensor->type == SF_ACC) {
        return;
    }
    
    if (sensor->velError > SENSOR_VEL_ERROR_THRESH) {
        sensor->penalityIters = SENSOR_MAX_PENALITY_ITERS;
    } else if (sensor->penalityIters > 0) {
        sensor->penalityIters--;
    }
    
    sensor->toFuse = (sensor->penalityIters == 0);
}

void doNothing(sensorState_t *sensor) {
    UNUSED(sensor);
}

// ======================================================================================================
// ==================================== Sensor specific functions =======================================
// ======================================================================================================
void updateAccItegralCallback(timeUs_t currentTimeUs) {
    static bool firstRun = true;
    static timeUs_t prevTimeUs = 0;
    if (firstRun) {
        prevTimeUs = currentTimeUs;
        firstRun = false;
        return;
    }

    timeDelta_t deltaTimeUs = cmpTimeUs(currentTimeUs, prevTimeUs);
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        accIntegral.vel[i] += acc.accADC.v[i] * (float)deltaTimeUs / 1e6f;
    }
    accIntegral.vel[XYZ_AXIS_COUNT] += (acc.accMagnitude - 1.0f) * (float)deltaTimeUs / 1e6f;
    accIntegral.deltaTimeUs += deltaTimeUs;
    prevTimeUs = currentTimeUs;
}

void updateAccReading(sensorState_t *sensor) {
    static float velDriftZ = 0;

    static float accVelZ = 0;
    // given the attiude roll and pitch angles of the drone, and the integrated acceleration in x,y and z
    // calculate the integrated acceleration in the z direction in the world frame
    float roll  = DEGREES_TO_RADIANS((float)attitude.values.roll / 10.0f); // integer devision to reduce noise
    float pitch = DEGREES_TO_RADIANS((float)attitude.values.pitch / 10.0f);
    
    float cosPitch = cosf(pitch);

    float velWorldZ = - accIntegral.vel[0] * sinf(pitch) 
                    +   accIntegral.vel[1] * cosPitch * sinf(roll)
                    +   accIntegral.vel[2] * cosPitch * cosf(roll);

    // sensor->deltaTimeMs  = (float)accIntegral.deltaTimeUs / 1e3f;

    float gravityVel = (float)accIntegral.deltaTimeUs / 1e6f; // g/Us to g/S
    
    float velCmSecZ = ((velWorldZ * acc.dev.acc_1G_rec) - gravityVel) * 981.0f; // g to cm/s

    velCmSecZ = (int)(velCmSecZ * 10) / 10.0f;

    accVelZ += velCmSecZ;

    velDriftZ  = 0.005 * accVelZ  + (1.0f - 0.005) * velDriftZ;

    sensor->velocityCmS = accVelZ  - velDriftZ;

    velocityCm3D.value  = fabsf(accIntegral.vel[XYZ_AXIS_COUNT] * 981.0f);
    // applyAccVelFilter(&sensor->velocityCmS);
    // sensor->currentAltReadingCm += sensor->velocityCmS * ((float)accIntegral.deltaTimeUs/1e6f);
    DEBUG_SET(DEBUG_ALTITUDE, 0, lrintf(accIntegral.vel[2] * 981.0f));
    DEBUG_SET(DEBUG_ALTITUDE, 1, lrintf(accVelZ));
    DEBUG_SET(DEBUG_ALTITUDE, 2, lrintf(velDriftZ));
    DEBUG_SET(DEBUG_ALTITUDE, 3, lrintf(sensor->velocityCmS));
    DEBUG_SET(DEBUG_ALTITUDE, 4, lrintf(accIntegral.vel[XYZ_AXIS_COUNT] * 981.0f));
    DEBUG_SET(DEBUG_ALTITUDE, 7, lrintf(velocityCm3D.value));

    for (int i = 0; i <= XYZ_AXIS_COUNT; i++) {
        accIntegral.vel[i] = 0;
    }
    accIntegral.deltaTimeUs = 0;

}

void applyAccVelFilter(float *velocity) {
    static pt2Filter_t velFilter;
    static bool firstRun = true;

    if (firstRun) {
        pt2FilterInit(&velFilter, 0.5);
        firstRun = false;
    }
    
    *velocity = pt2FilterApply(&velFilter, *velocity);
}


#ifdef USE_BARO

void updateBaroReading(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    static bool firstRun = true;
    static float previousAltitude = 0.0f;
    
    if (firstRun) { // init
        previousAltitude = getBaroAltitude();
        sensor->zeroAltOffsetCm = previousAltitude; // init the offset with the first reading
        firstRun = false;
    }

    sensor->currentAltReadingCm = getBaroAltitude();
    applyBaroFilter(&sensor->currentAltReadingCm);
    sensor->velocityCmS = (sensor->currentAltReadingCm - previousAltitude) * TASK_ALTITUDE_RATE_HZ;
    previousAltitude = sensor->currentAltReadingCm;
}

void updateBaroVariance(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    static float stationaryVariance = 0;
    static float stationaryMean = 0;
    static uint16_t n = 0;
    
    float velocity = 0;

    if (!ARMING_FLAG(ARMED)) { // approximating the mean and variance of the baro readings during the stationary phase
        stationaryMean     +=    (sensor->currentAltReadingCm - stationaryMean)                       / (n + 1);
        stationaryVariance += (sq(sensor->currentAltReadingCm - stationaryMean) - stationaryVariance) / (n + 1);
        n++;        
        velocity = 0;
    } else {
        velocity = velocityCm3D.isValid ? (float)velocityCm3D.value : (altitudeState.velocityCm);
    }

    float newVariance = stationaryVariance
                      + BARO_VAR_VEL_ERROR_COEFF * (float)sensor->velError
                      + BARO_VAR_VEL_COEFF       * fabsf(velocity)
                      + BARO_VAR_TEMP_COEFF      * fabsf((float)getBaroTemperature()) 
                      + BARO_VAR_ALT_COEFF       * fabsf(sensor->currentAltReadingCm);

    sensor->variance = 0.9f * sensor->variance + 0.1f * newVariance;
}

void applyBaroFilter(float *value) {
    static pt2Filter_t baroLpfFilter;
    static bool firstRun = true;

    if (firstRun) {
        const float altitudeCutoffHz = positionConfig()->altitude_lpf / 100.0f;
        const float altitudeGain     = pt2FilterGain(altitudeCutoffHz, HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ));
        pt2FilterInit(&baroLpfFilter, altitudeGain);
        firstRun = false;
    }

    *value = pt2FilterApply(&baroLpfFilter, *value);
}
#endif // USE_BARO

#ifdef USE_GPS
void updateGpsReading(sensorState_t *sensor) {
    static uint32_t prevTimeStamp = 0;
    static bool firstRun = true;
    static float previousAltitude = 0.0f;
    bool hasNewData = gpsSol.time != prevTimeStamp;
    
    sensor->isValid = gpsIsHealthy() && sensors(SENSOR_GPS) && STATE(GPS_FIX) && hasNewData;
    if (!sensor->isValid) {
#ifndef USE_ACC
        velocityCm3D.isValid = false;
#endif
        return;
    }

    if (firstRun) {
        previousAltitude = gpsSol.llh.altCm;
        sensor->zeroAltOffsetCm = previousAltitude;
        prevTimeStamp = gpsSol.time;
        firstRun = false;
    }

    sensor->currentAltReadingCm = gpsSol.llh.altCm;
    applyGpsFilter(&sensor->currentAltReadingCm);

#ifndef USE_ACC
    velocityCm3D.value = gpsSol.speed3d * 10;
    velocityCm3D.isValid = true;
#endif
    timeDelta_t deltaTime = cmpTimeUs(gpsSol.time, prevTimeStamp);
    sensor->velocityCmS = ((sensor->currentAltReadingCm - previousAltitude) * deltaTime) / 1000.0f;
    previousAltitude = sensor->currentAltReadingCm;
    prevTimeStamp = gpsSol.time;
}

void updateGpsVariance(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }
    
    float newVariance = GPS_VAR_VEL_ERROR_COEFF * sensor->velError;

    if (gpsSol.dop.vdop != 0) {
        newVariance += GPS_VAR_DOP_COEFF * gpsSol.dop.vdop;
    } else if (gpsSol.dop.pdop != 0) {
        newVariance += GPS_VAR_DOP_COEFF * gpsSol.dop.pdop;
    } else {
        newVariance += 10000.0f; 
    }

    sensor->variance = 0.9f * sensor->variance + 0.1f * newVariance;
}

void updateGpsOffset(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    if (!ARMING_FLAG(ARMED)) { // default offset update when not armed
        updateSensorOffset(sensor);
    } else { 
        // the offset calculated when not armed is not accurate anymore, in case of getting more satalites, the offset should be updated.
        static float accError = 0;
        float estAlt = sensor->currentAltReadingCm - sensor->zeroAltOffsetCm;
        float error = (estAlt - kf.estimatedValue);

        if (fabsf(error) > GPS_RAPID_ERR_THRESH) { // if the error is too high, update the offset directly
            sensor->zeroAltOffsetCm = sensor->zeroAltOffsetCm + error;
            return;
        } else { // otherwise, add the error to the accumlated error and smoothly update the offset if the error is high enough
            accError += error;
            if (accError > GPS_ACC_ERROR_THRESH) {
                sensor->zeroAltOffsetCm = 0.99f * sensor->zeroAltOffsetCm + 0.01f * (estAlt - kf.estimatedValue);
            }
        }
    }
}

void applyGpsFilter(float *value) {
    static pt2Filter_t filter;
    static bool firstRun = true;

    if (firstRun) {
        pt2FilterInit(&filter, 0.5);
        firstRun = false;
    }

    *value = pt2FilterApply(&filter, *value);
}

#endif // USE_GPS

#ifdef USE_RANGEFINDER

void updateRangefinderReading(sensorState_t *sensor) {
    static bool firstRun = true;
    static float previousAltitude = 0.0f;
    int32_t rfAlt = rangefinderGetLatestAltitude();
    
    sensor->isValid = rangefinderIsHealthy() && sensors(SENSOR_RANGEFINDER) && rfAlt >= 0;

    if (!sensor->isValid) {
        return;
    }

    if (firstRun) {
        previousAltitude = rfAlt;
        sensor->zeroAltOffsetCm = rfAlt;
        firstRun = false;
    }

    sensor->currentAltReadingCm = rfAlt;
    // applyRangefinderFilter(&sensor->currentAltReadingCm);
    sensor->velocityCmS = (sensor->currentAltReadingCm - previousAltitude) * TASK_ALTITUDE_RATE_HZ;
    previousAltitude = sensor->currentAltReadingCm;
}

void updateRangefinderVariance(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    float newVariance = RANGEFINDER_VAR_VEL_ERROR_COEFF * sensor->velError + RANGEFINDER_CONST_VAR; // is there a better way to get the variance of the rangefinder?
    sensor->variance = 0.9f * sensor->variance + 0.1f * newVariance;
}

void updateRangefinderOffset(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    if (!ARMING_FLAG(ARMED)) { // default offset update when not armed
        sensor->zeroAltOffsetCm = 0.2f * sensor->currentAltReadingCm + 0.8f * sensor->zeroAltOffsetCm;
    } else { // when armed the offset should be updated according to the velocity error value
        float estAlt = sensor->currentAltReadingCm - sensor->zeroAltOffsetCm;
        float error = (estAlt - kf.estimatedValue);
        if (sensor->velError > SENSOR_VEL_ERROR_THRESH) {
            sensor->zeroAltOffsetCm += error;
        } else { // update the offset smoothly using the error value, if the error is 0 then no update is done
            // this is not effective, we should find a way to fix the position drift of the rangefinder or gps

            float correctnessRatio = (float)(SENSOR_VEL_MAX_ERROR - sensor->velError) / SENSOR_VEL_MAX_ERROR;
            sensor->zeroAltOffsetCm = (correctnessRatio * sensor->zeroAltOffsetCm) 
                                    + ((1.0f - correctnessRatio) * (estAlt - kf.estimatedValue));
        }
    }
}

void applyRangefinderFilter(float *value) {
    static pt2Filter_t filter;
    static bool firstRun = true;

    if (firstRun) {
        pt2FilterInit(&filter, 0.5);
        firstRun = false;
    }

    *value = pt2FilterApply(&filter, *value);
}
#endif // USE_RANGEFINDER

#endif // USE_BARO || USE_GPS || USE_RANGEFINDER