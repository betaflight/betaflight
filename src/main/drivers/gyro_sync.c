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

void gyroUpdateSampleRate(void) {
    int gyroSamplePeriod;
    int minLooptime;

    gyroSamplePeriod = 1000; // gyro sampling rate 1khz
    minLooptime = 1000;      // Full 1khz sampling

    // calculate gyro divider and targetLooptime (expected cycleTime)
    mpuDividerDrops  = (minLooptime + gyroSamplePeriod -1 ) / gyroSamplePeriod - 1;
    targetLooptime = (mpuDividerDrops + 1) * gyroSamplePeriod;
}

uint8_t gyroMPU6xxxGetDividerDrops(void) {
    return mpuDividerDrops;
}
