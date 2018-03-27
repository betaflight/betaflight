/*
 * gyro_sync.c
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/gyro_sync.h"


bool gyroSyncCheckUpdate(gyroDev_t *gyro)
{
    bool ret;
    if (gyro->dataReady) {
        ret = true;
        gyro->dataReady= false;
    } else {
        ret = false;
    }
    return ret;
}

uint32_t gyroSetSampleRate(gyroDev_t *gyro, uint8_t lpf, uint8_t gyroSyncDenominator, bool gyro_use_32khz)
{
    bool lpfNoneOr256 = (lpf == GYRO_LPF_256HZ || lpf == GYRO_LPF_NONE);
    if (!lpfNoneOr256) {
        gyroSyncDenominator = 1; // Always full Sampling
    }
    gyro->mpuDividerDrops = gyroSyncDenominator - 1;
    gyro->gyroRateKHz = lpfNoneOr256 ? GYRO_RATE_8_kHz : GYRO_RATE_1_kHz;
    //20649 is a weird gyro
    if (gyro->mpuDetectionResult.sensor == ICM_20649_SPI) {
        gyro->gyroRateKHz = lpfNoneOr256 ? GYRO_RATE_9_kHz : GYRO_RATE_1100_Hz;
    } else if (gyro->mpuDetectionResult.sensor == BMI_160_SPI && lpfNoneOr256) { 
        //brainFPV is also a weird gyro
        gyro->gyroRateKHz = GYRO_RATE_3200_Hz;
    } else if (gyro_use_32khz) {
        //use full 32k
        gyro->gyroRateKHz = GYRO_RATE_32_kHz;
    }

    // return the targetLooptime (expected cycleTime)
    return (uint32_t)(gyroSyncDenominator * gyro->gyroRateKHz);
}
