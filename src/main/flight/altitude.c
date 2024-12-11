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


#define SENSOR_VEL_ERROR_THRESH   1000.0f // the error above this value excludes the sensor from the fusion and changes the offset
#define SENSOR_VEL_MAX_ERROR      10000.0f
#define SENSOR_MAX_PENALITY_ITERS 10
#define SENSOR_MAX_OFFSET_ERROR   1000.0f

#define BARO_VAR_ALT_COEFF       0.01f
#define BARO_VAR_VEL_COEFF       0.01f
#define BARO_VAR_TEMP_COEFF      0.01f
#define BARO_VAR_VEL_ERROR_COEFF 1.00f
#define BARO_TASK_INTERVAL_CALC_ITER 10 

#define RANGEFINDER_ACC_ERROR_THRESH    50.0f
#define RANGEFINDER_RAPID_ERR_THRESH    100.0f
#define RANGEFINDER_VAR_VEL_ERROR_COEFF 2.0f
#define RANGEFINDER_CONST_VAR           10.0f

#define GPS_VAR_DOP_COEFF       1.0f
#define GPS_VAR_VEL_ERROR_COEFF 2.0f
#define GPS_RAPID_ERR_THRESH    100.0f
#define GPS_ACC_ERROR_THRESH    200.0f
#define GPS_PDOP_MIN_THRESHOLD  400.0f

typedef enum {
#ifdef USE_ACC
    SF_ACC,
#endif
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

typedef struct sensorState_s {
    float currentAltReadingCm;
    float zeroAltOffsetCm;
    float velocityAltCmS;
    float variance;
    uint32_t deltaTimeMs;
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
void updateBaroOffset(sensorState_t *sensor);
void applyBaroFilter(float *dst, float newValue);
#endif

#ifdef USE_GPS
void updateGpsReading(sensorState_t *sensor);
void updateGpsVariance(sensorState_t *sensor);
#endif

#ifdef USE_RANGEFINDER
void updateRangefinderReading(sensorState_t *sensor);
void updateRangefinderVariance(sensorState_t *sensor);
#endif

void updateSensorOffset(sensorState_t *sensor);
void updateVelError(sensorState_t *sensor);
void updateSensorFusability(sensorState_t *sensor);
bool sensorUpdateIteration(sensorState_t *sensor);
void doNothing(sensorState_t *sensor);

static sensorState_t altSenFusSources[SENSOR_COUNT];
static KalmanFilter kf;
static velocity3D_t velocity3DCmS;
static altitudeState_t altitudeState;

void altSensorFusionInit(void) {
velocity3DCmS.value = 0;
velocity3DCmS.isValid = false;

#ifdef USE_ACC
    altSenFusSources[SF_ACC].updateReading  = updateAccReading;
    altSenFusSources[SF_ACC].updateVariance = doNothing;
    altSenFusSources[SF_ACC].updateOffset   = doNothing;
    altSenFusSources[SF_ACC].type = SF_ACC;
    altSenFusSources[SF_ACC].isValid = false;
    altSenFusSources[SF_ACC].toFuse = false;
    velocity3DCmS.isValid = true;
#endif
    
#ifdef USE_BARO
    altSenFusSources[SF_BARO].updateReading  = updateBaroReading;
    altSenFusSources[SF_BARO].updateVariance = updateBaroVariance;
    altSenFusSources[SF_BARO].updateOffset   = updateBaroOffset;
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
    velocity3DCmS.isValid = true;
#endif

#ifdef USE_RANGEFINDER
    altSenFusSources[SF_RANGEFINDER].updateReading  = updateRangefinderReading;
    altSenFusSources[SF_RANGEFINDER].updateVariance = updateRangefinderVariance;
    altSenFusSources[SF_RANGEFINDER].updateOffset   = updateSensorOffset;
    altSenFusSources[SF_RANGEFINDER].type = SF_RANGEFINDER;
    altSenFusSources[SF_RANGEFINDER].isValid = false;
    altSenFusSources[SF_RANGEFINDER].toFuse = true;
#endif

    kf_init(&kf, 0.0f, 1.0f, 10.0f);
}

bool sensorUpdateIteration(sensorState_t *sensor) {

    sensor->updateReading(sensor);
    updateVelError(sensor);
    updateSensorFusability(sensor); // this should handle the case of sudden jumps in the sensor readings compared to the accelerometer velocity estimation
    sensor->updateVariance(sensor);
    sensor->updateOffset(sensor);
    
    if (sensor->isValid && sensor->toFuse) {
        SensorMeasurement tempSensorMeas;
        tempSensorMeas.value    = sensor->currentAltReadingCm - sensor->zeroAltOffsetCm;
        tempSensorMeas.variance = sensor->variance;
#ifdef USE_GPS
        if (sensor->type == SF_GPS) { // ignore the GPS for now, TODO: add the gps to the fusion
            return false;
        }
#endif
        kf_update(&kf, tempSensorMeas);
        return true;
    }
    return false;
}

bool altSensorFusionUpdate(void) {
    static timeMs_t prevTimeMs = 0;
    timeMs_t deltaTimeMs = millis() - prevTimeMs;
    prevTimeMs = millis();

    kf_update_variance(&kf);
    float previousAltitude = altitudeState.distCm;
    bool haveAltitude = false;
    for (sensorState_t * sensor = altSenFusSources; sensor < altSenFusSources + SENSOR_COUNT; sensor++) {
        haveAltitude |= sensorUpdateIteration(sensor);
    }

    altitudeState.distCm     = kf.estimatedValue; 
    altitudeState.variance   = kf.estimatedVariance;
    altitudeState.velocityCm = (altitudeState.distCm - previousAltitude) * 1000 / deltaTimeMs;
    previousAltitude         = altitudeState.distCm;

#ifdef USE_ACC
    DEBUG_SET(DEBUG_ALTITUDE, 3, lrintf(altSenFusSources[SF_ACC].velocityAltCmS));
#endif

#ifdef USE_BARO    
    DEBUG_SET(DEBUG_ALTITUDE, 1, lrintf(altSenFusSources[SF_BARO].currentAltReadingCm - altSenFusSources[SF_BARO].zeroAltOffsetCm));
    DEBUG_SET(DEBUG_ALTITUDE, 5, lrintf(altSenFusSources[SF_BARO].zeroAltOffsetCm));
#endif

#ifdef USE_GPS
    DEBUG_SET(DEBUG_ALTITUDE, 0, lrintf(10000 - gpsSol.dop.pdop));
    DEBUG_SET(DEBUG_ALTITUDE, 2, lrintf(altSenFusSources[SF_GPS].currentAltReadingCm  - altSenFusSources[SF_GPS].zeroAltOffsetCm));
#endif

#ifdef USE_RANGEFINDER
    DEBUG_SET(DEBUG_ALTITUDE, 6, lrintf(altSenFusSources[SF_RANGEFINDER].zeroAltOffsetCm));
    DEBUG_SET(DEBUG_ALTITUDE, 4, lrintf(altSenFusSources[SF_RANGEFINDER].currentAltReadingCm - altSenFusSources[SF_RANGEFINDER].zeroAltOffsetCm));
#endif
    DEBUG_SET(DEBUG_ALTITUDE, 7, lrintf(altitudeState.distCm));


    return haveAltitude;
}

void updateSensorOffset(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    if (!ARMING_FLAG(ARMED)) {// default offset update when not armed
        sensor->zeroAltOffsetCm = 0.2f * sensor->currentAltReadingCm + 0.8f * sensor->zeroAltOffsetCm;
    } else { // when armed the offset should be updated according to the velocity error value
        float newOffset = sensor->currentAltReadingCm - kf.estimatedValue;
        if (sensor->penalityIters > 0) {
            sensor->zeroAltOffsetCm = 0.5f * (newOffset + sensor->zeroAltOffsetCm);
        }
        else { // detect a ramp in the sensor readings by accumulating the error
            sensor->offsetError += newOffset - sensor->zeroAltOffsetCm;
            if (fabsf(sensor->offsetError) > SENSOR_MAX_OFFSET_ERROR) {
                sensor->zeroAltOffsetCm = 0.01f * newOffset + 0.99f * sensor->zeroAltOffsetCm;
                sensor->offsetError = 0.99f * sensor->offsetError; // decaying the error
            }
        }
    }
}

void updateVelError(sensorState_t *sensor) {
    if (!sensor->isValid || sensor->type == SF_ACC) {
        return;
    }
    sensor->velError = 0.1f * (sq(altSenFusSources[SF_ACC].velocityAltCmS - sensor->velocityAltCmS) / (100.f))
                     + 0.9f * sensor->velError;

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

altitudeState_t *getAltitudeState(void) {
    return &altitudeState;
}

void doNothing(sensorState_t *sensor) {
    UNUSED(sensor);
}

// ======================================================================================================
// ==================================== Sensor specific functions =======================================
// ======================================================================================================
void updateAccItegralCallback(timeUs_t currentTimeUs) { // this is called in the acc update task
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
    static float velDriftZ = 0.0f;

    static float accVelZ = 0.0f;
    // given the attiude roll and pitch angles of the drone, and the integrated acceleration in x,y and z
    // calculate the integrated acceleration in the z direction in the world frame
    float roll  = DEGREES_TO_RADIANS((float)attitude.values.roll / 10.0f); // integer devision to reduce noise
    float pitch = DEGREES_TO_RADIANS((float)attitude.values.pitch / 10.0f);
    
    float cosPitch = cosf(pitch);

    float velWorldZ = - accIntegral.vel[0] * sinf(pitch) 
                    +   accIntegral.vel[1] * cosPitch * sinf(roll)
                    +   accIntegral.vel[2] * cosPitch * cosf(roll);

    float gravityVel = (float)accIntegral.deltaTimeUs / 1e6f; // g/Us to g/S
    
    float velCmSecZ = ((velWorldZ * acc.dev.acc_1G_rec) - gravityVel) * 981.0f; // g to cm/s

    velCmSecZ = (int)(velCmSecZ * 10) / 10.0f;

    accVelZ += velCmSecZ;

    velDriftZ  = 0.005f * accVelZ  + (1.0f - 0.005f) * velDriftZ;

    sensor->velocityAltCmS = accVelZ  - velDriftZ;

    velocity3DCmS.value  = (0.1f * fabsf(accIntegral.vel[XYZ_AXIS_COUNT] * 981.0f)) + (0.9f * velocity3DCmS.value);
    // applyAccVelFilter(&sensor->velocityAltCmS);
    // sensor->currentAltReadingCm += sensor->velocityAltCmS * ((float)accIntegral.deltaTimeUs/1e6f);

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
void updateBaroStateCallback(void) {
    applyBaroFilter(&altSenFusSources[SF_BARO].currentAltReadingCm, getBaroAltitude());
}

void updateBaroReading(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    static bool firstRun = true;
    static float previousAltitude = 0.0f;
    static uint32_t prevTimeMs = 0;

    if (firstRun) { // init
        previousAltitude        = sensor->currentAltReadingCm;
        sensor->zeroAltOffsetCm = sensor->currentAltReadingCm; // init the offset with the first reading
        firstRun = false;
        prevTimeMs = millis();
    }

    sensor->deltaTimeMs = millis() - prevTimeMs;

    sensor->velocityAltCmS = (sensor->currentAltReadingCm - previousAltitude) * 1000 / sensor->deltaTimeMs;
    previousAltitude = sensor->currentAltReadingCm;

    prevTimeMs = millis();
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
        velocity = velocity3DCmS.isValid ? (float)velocity3DCmS.value : (altitudeState.velocityCm);
    }

    float newVariance = stationaryVariance
                      + BARO_VAR_VEL_ERROR_COEFF * (float)sensor->velError
                      + BARO_VAR_VEL_COEFF       * fabsf(velocity)
                      + BARO_VAR_TEMP_COEFF      * fabsf((float)getBaroTemperature()) 
                      + BARO_VAR_ALT_COEFF       * fabsf(sensor->currentAltReadingCm);

    sensor->variance = 0.9f * sensor->variance + 0.1f * newVariance;
}

void updateBaroOffset(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    if (!ARMING_FLAG(ARMED)) { // default offset update when not armed
        sensor->zeroAltOffsetCm = 0.2f * sensor->currentAltReadingCm + 0.8f * sensor->zeroAltOffsetCm;
    }
}

void applyBaroFilter(float *dst, float newValue) {
    static pt2Filter_t baroLpfFilter;
    static bool firstRun = true;
    // calculate the task interval for the first few iterations in ms (can this be done better ? from the baro dev ?)
    static int8_t taskIntervalIter = BARO_TASK_INTERVAL_CALC_ITER;
    static uint16_t taskInterval = 0;
    if (taskIntervalIter > 0) {
        static bool firstIter = true;
        static uint16_t prevTimeMs = 0;
        if (firstIter) {
            prevTimeMs = millis();
            firstIter = false;
            return;
        }
        taskInterval += millis() - prevTimeMs;
        taskIntervalIter--;
        prevTimeMs = millis();
        if (taskIntervalIter == 0) {
            taskInterval /= BARO_TASK_INTERVAL_CALC_ITER;
        }
        return;
    }
    if (firstRun) {
        const float altitudeCutoffHz = positionConfig()->altitude_lpf / 100.0f;
        const float altitudeGain     = pt2FilterGain(altitudeCutoffHz,  taskInterval);
        pt2FilterInit(&baroLpfFilter, altitudeGain);
        firstRun = false;
    }

    *dst = pt2FilterApply(&baroLpfFilter, newValue);
}
#endif // USE_BARO

#ifdef USE_GPS
void updateGpsReading(sensorState_t *sensor) {
    static uint32_t prevTimeStamp = 0;
    static bool firstRun = true;
    static float previousAltitude = 0.0f;
    bool hasNewData = gpsSol.time != prevTimeStamp;
    bool hdopIsGood = (gpsSol.dop.pdop > 0 && gpsSol.dop.pdop < GPS_PDOP_MIN_THRESHOLD)
                   || (gpsSol.dop.hdop > 0 && gpsSol.dop.hdop < GPS_PDOP_MIN_THRESHOLD);
    
    sensor->isValid = gpsIsHealthy()
                   && sensors(SENSOR_GPS)
                   && STATE(GPS_FIX) 
                   && hasNewData
                   && hdopIsGood;

    if (!sensor->isValid) {
#ifndef USE_ACC
        velocity3DCmS.isValid = false;
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

#ifndef USE_ACC
    velocity3DCmS.value = gpsSol.speed3d * 10;
    velocity3DCmS.isValid = true;
#endif
    sensor->deltaTimeMs = gpsSol.time - prevTimeStamp;
    sensor->velocityAltCmS = ((sensor->currentAltReadingCm - previousAltitude) * 1000.0f) / sensor->deltaTimeMs;
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

    sensor->variance = newVariance;
}

#endif // USE_GPS

#ifdef USE_RANGEFINDER

void updateRangefinderReading(sensorState_t *sensor) {
    static bool firstRun = true;
    static float previousAltitude = 0.0f;
    static float prevReadingTime = 0;
    int32_t rfAlt   = getRangefinder()->calculatedAltitude;
    bool hasNewData = getRangefinder()->lastValidResponseTimeMs != prevReadingTime;

    sensor->isValid = rangefinderIsHealthy() 
                   && sensors(SENSOR_RANGEFINDER) 
                   && rfAlt >= 0 
                   && hasNewData;

    if (!sensor->isValid) {
        return;
    }

    if (firstRun) {
        previousAltitude = rfAlt;
        firstRun = false;
        prevReadingTime = getRangefinder()->lastValidResponseTimeMs;
    }

    sensor->deltaTimeMs = getRangefinder()->lastValidResponseTimeMs - prevReadingTime;
    sensor->currentAltReadingCm = rfAlt;
    sensor->velocityAltCmS = (sensor->currentAltReadingCm - previousAltitude) * 1000.0f / sensor->deltaTimeMs;
    previousAltitude = sensor->currentAltReadingCm;
    prevReadingTime = getRangefinder()->lastValidResponseTimeMs;
}

void updateRangefinderVariance(sensorState_t *sensor) {
    if (!sensor->isValid) {
        return;
    }

    float newVariance = RANGEFINDER_VAR_VEL_ERROR_COEFF * sensor->velError + RANGEFINDER_CONST_VAR;
    sensor->variance = 0.9f * sensor->variance + 0.1f * newVariance;
}

#endif // USE_RANGEFINDER
#endif // USE_BARO || USE_GPS || USE_RANGEFINDER