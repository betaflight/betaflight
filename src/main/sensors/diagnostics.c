#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "flight/navigation_rewrite.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/diagnostics.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/rangefinder.h"

extern uint8_t requestedSensors[SENSOR_INDEX_COUNT];
extern uint8_t detectedSensors[SENSOR_INDEX_COUNT];

hardwareSensorStatus_e getHwGyroStatus(void)
{
    // Gyro is assumed to be always healthy
    return HW_SENSOR_OK;
}

hardwareSensorStatus_e getHwAccelerometerStatus(void)
{
    if (detectedSensors[SENSOR_INDEX_ACC] != ACC_NONE) {
        if (true) { // isAccelerometerHealthy()
            return HW_SENSOR_OK;
        }
        else {
            return HW_SENSOR_UNHEALTHY;
        }
    }
    else {
        if (requestedSensors[SENSOR_INDEX_ACC] != ACC_NONE) {
            // Selected but not detected
            return HW_SENSOR_UNAVAILABLE;
        }
        else {
            // Not selected and not detected
            return HW_SENSOR_NONE;
        }
    }
}

hardwareSensorStatus_e getHwCompassStatus(void)
{
    if (detectedSensors[SENSOR_INDEX_MAG] != MAG_NONE) {
        if (isCompassHealthy()) {
            return HW_SENSOR_OK;
        }
        else {
            return HW_SENSOR_UNHEALTHY;
        }
    }
    else {
        if (requestedSensors[SENSOR_INDEX_MAG] != MAG_NONE) {
            // Selected but not detected
            return HW_SENSOR_UNAVAILABLE;
        }
        else {
            // Not selected and not detected
            return HW_SENSOR_NONE;
        }
    }
}

hardwareSensorStatus_e getHwBarometerStatus(void)
{
    if (detectedSensors[SENSOR_INDEX_BARO] != BARO_NONE) {
        if (true) { // isBarometerHealthy()
            return HW_SENSOR_OK;
        }
        else {
            return HW_SENSOR_UNHEALTHY;
        }
    }
    else {
        if (requestedSensors[SENSOR_INDEX_BARO] != BARO_NONE) {
            // Selected but not detected
            return HW_SENSOR_UNAVAILABLE;
        }
        else {
            // Not selected and not detected
            return HW_SENSOR_NONE;
        }
    }
}

hardwareSensorStatus_e getHwRangefinderStatus(void)
{
    if (detectedSensors[SENSOR_INDEX_RANGEFINDER] != RANGEFINDER_NONE) {
        if (true) { // isBarometerHealthy()
            return HW_SENSOR_OK;
        }
        else {
            return HW_SENSOR_UNHEALTHY;
        }
    }
    else {
        if (requestedSensors[SENSOR_INDEX_RANGEFINDER] != RANGEFINDER_NONE) {
            // Selected but not detected
            return HW_SENSOR_UNAVAILABLE;
        }
        else {
            // Not selected and not detected
            return HW_SENSOR_NONE;
        }
    }
}

hardwareSensorStatus_e getHwGPSStatus(void)
{
    if (sensors(SENSOR_GPS)) {
        if (true) { // isGPSHealthy()
            return HW_SENSOR_OK;
        }
        else {
            return HW_SENSOR_UNHEALTHY;
        }
    }
    else {
        if (feature(FEATURE_GPS)) {
            // Selected but not detected
            return HW_SENSOR_UNAVAILABLE;
        }
        else {
            // Not selected and not detected
            return HW_SENSOR_NONE;
        }
    }
}

bool isHardwareHealthy(void)
{
    const hardwareSensorStatus_e gyroStatus = getHwGyroStatus();
    const hardwareSensorStatus_e accStatus = getHwAccelerometerStatus();
    const hardwareSensorStatus_e baroStatus = getHwBarometerStatus();
    const hardwareSensorStatus_e magStatus = getHwCompassStatus();
    const hardwareSensorStatus_e rangefinderStatus = getHwRangefinderStatus();
    const hardwareSensorStatus_e gpsStatus = getHwGPSStatus();

    // Sensor is considered failing if it's either unavailable (selected but not detected) or unhealthy (returning invalid readings)
    if (gyroStatus == HW_SENSOR_UNAVAILABLE || gyroStatus == HW_SENSOR_UNHEALTHY)
        return false;

    if (accStatus == HW_SENSOR_UNAVAILABLE || accStatus == HW_SENSOR_UNHEALTHY)
        return false;

    if (baroStatus == HW_SENSOR_UNAVAILABLE || baroStatus == HW_SENSOR_UNHEALTHY)
        return false;

    if (magStatus == HW_SENSOR_UNAVAILABLE || magStatus == HW_SENSOR_UNHEALTHY)
        return false;

    if (rangefinderStatus == HW_SENSOR_UNAVAILABLE || rangefinderStatus == HW_SENSOR_UNHEALTHY)
        return false;

    if (gpsStatus == HW_SENSOR_UNAVAILABLE || gpsStatus == HW_SENSOR_UNHEALTHY)
        return false;

	return true;
}
