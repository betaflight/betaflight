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
    int gyroSamplePeriod;
    int minLooptime;

#if defined(SPRACINGF3) || defined(ALIENWIIF3) || defined(NAZE32PRO) || defined(STM32F3DISCOVERY) || defined(CHEBUZZF3) || defined(PORT103R) || defined(MOTOLAB)  || defined(SPARKY)
        if (lpf == INV_FILTER_256HZ_NOLPF2) {
            gyroSamplePeriod = 125;

            if(!sensors(SENSOR_ACC)) {
                minLooptime = 500;   // Max refresh 2khz
            } else {
                minLooptime = 625;   // Max refresh 1,6khz
            }
        } else {
            gyroSamplePeriod = 1000;
            minLooptime = 1000;      // Full sampling
        }
#elif defined(CC3D)
        if (lpf == INV_FILTER_256HZ_NOLPF2) {
            gyroSamplePeriod = 125;

            if(!sensors(SENSOR_ACC)) {
                minLooptime = 890;   // Max refresh 1,12khz
            } else {
                minLooptime = 1000;  // Max refresh 1khz
            }
        } else {
            gyroSamplePeriod = 1000;
            minLooptime = 1000;      // Full sampling
        }
#elif defined(COLIBRI_RACE) || defined(EUSTM32F103RC)
        if (lpf == INV_FILTER_256HZ_NOLPF2) {
            gyroSamplePeriod = 125;

            if(!sensors(SENSOR_ACC)) { // TODO - increase to 8khz when oneshot125 can be limited
                minLooptime = 250;   // Max refresh 4khz
            } else {
                minLooptime = 250;   // Max refresh 4khz
            }
        } else {
            gyroSamplePeriod = 1000;
            minLooptime = 1000;      // Full sampling
        }
#else
        if (lpf == INV_FILTER_256HZ_NOLPF2) {
            gyroSamplePeriod = 125;
            minLooptime = 625;      // Max refresh 1,6khz
        } else {
            gyroSamplePeriod = 1000;
            minLooptime = 1000;     // Full sampling without ACC
        }
#endif
        mpuDividerDrops  = (minLooptime + gyroSamplePeriod -1 ) / gyroSamplePeriod - 1;
        targetLooptime = (mpuDividerDrops + 1) * gyroSamplePeriod;
}

uint8_t gyroMPU6xxxGetDividerDrops(void) {
    return mpuDividerDrops;
}
