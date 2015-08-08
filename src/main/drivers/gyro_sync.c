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
#include "drivers/accgyro_mpu6050.h"
#include "drivers/accgyro_spi_mpu6000.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "config/runtime_config.h"
#include "config/config.h"

extern gyro_t gyro;

uint32_t targetLooptime;
static uint8_t mpuDivider;

bool getMpuInterrupt(gyro_t *gyro)
{
    bool gyroIsUpdated;

    gyro->intStatus(&gyroIsUpdated);
    return gyroIsUpdated;
}

bool gyroSyncCheckUpdate(void) {
    return getMpuInterrupt(&gyro);
}

void gyroUpdateSampleRate(uint32_t looptime, uint8_t lpf) {
    int gyroSamplePeriod;
    int minLooptime;

#ifdef STM32F303xC
    if (lpf == INV_FILTER_256HZ_NOLPF2) {
        gyroSamplePeriod = 125;

        if(!sensors(SENSOR_ACC)) {
            minLooptime = 500;   // Max refresh 2khz
        }
        else {
            minLooptime = 625;   // Max refresh 1,6khz
        }
    }
    else {
        gyroSamplePeriod = 1000;
        minLooptime = 1000;      // Full sampling
    }
#elif STM32F10X
    if (lpf == INV_FILTER_256HZ_NOLPF2) {
        gyroSamplePeriod = 125;

        if(!sensors(SENSOR_ACC)) {
            minLooptime = 625;   // Max refresh 1,33khz
        }
        else {
            minLooptime = 1625;  // Max refresh 615hz
        }
    }
    else {
        gyroSamplePeriod = 1000;
        if(!sensors(SENSOR_ACC)) {
            minLooptime = 1000;  // Full sampling without ACC
        }
        else {
            minLooptime = 2000;
        }
    }
#endif
    looptime = constrain(looptime, minLooptime, 4000);
    mpuDivider  = (looptime + gyroSamplePeriod -1 ) / gyroSamplePeriod - 1;
    targetLooptime = (mpuDivider + 1) * gyroSamplePeriod;
}

uint8_t gyroMPU6xxxGetDivider(void) {
    return mpuDivider;
}
