/*
 * gyro_sync.c
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"
#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "config/runtime_config.h"
#include "config/config.h"

extern gyro_t gyro;

uint32_t targetLooptime;
static uint8_t mpuDividerDrops;

bool getMpuDataStatus(gyro_t *gyro)
{
    bool mpuDataStatus;

    gyro->intStatus(&mpuDataStatus);
    return mpuDataStatus;
}

bool gyroSyncCheckUpdate(void) {
    return getMpuDataStatus(&gyro);
}

void gyroUpdateSampleRate(uint8_t lpf) {
    int gyroSamplePeriod, gyroSyncDenominator;

    if (!lpf) {
        gyroSamplePeriod = 125;
#ifdef STM32F303xC
#ifdef COLIBRI_RACE // Leave out LUX target for now.  Need to test 2.6Khz
        gyroSyncDenominator = 3; // Sample every 3d gyro measurement 2,6khz
#else
        gyroSyncDenominator = 4; // Sample every 4th gyro measurement 2khz
#endif
#else
        if (!sensors(SENSOR_ACC) && !sensors(SENSOR_BARO) && !sensors(SENSOR_MAG)) {
            gyroSyncDenominator = 4; // Sample every 4th gyro measurement 2khz
        } else {
            gyroSyncDenominator = 8; // Sample every 8th gyro measurement 1khz
        }
#endif
    } else {
        gyroSamplePeriod = 1000;
        gyroSyncDenominator = 1; // Full Sampling 1khz
    }

    // calculate gyro divider and targetLooptime (expected cycleTime)
    mpuDividerDrops  = gyroSyncDenominator - 1;
    targetLooptime = (mpuDividerDrops + 1) * gyroSamplePeriod;
}

uint8_t gyroMPU6xxxGetDividerDrops(void) {
    return mpuDividerDrops;
}
