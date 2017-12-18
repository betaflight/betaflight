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
    float gyroSamplePeriod;

    if (lpf == GYRO_LPF_256HZ || lpf == GYRO_LPF_NONE) {
        if (gyro_use_32khz) {
            gyro->gyroRateKHz = GYRO_RATE_32_kHz;
            gyroSamplePeriod = 31.5f;
        } else {
            switch (gyro->mpuDetectionResult.sensor) {
            case BMI_160_SPI:
                gyro->gyroRateKHz = GYRO_RATE_3200_Hz;
                gyroSamplePeriod = 312.0f;
                break;
            case ICM_20649_SPI:
                gyro->gyroRateKHz = GYRO_RATE_9_kHz;
                gyroSamplePeriod = 1000000.0f / 9000.0f;
                break;
            default:
                gyro->gyroRateKHz = GYRO_RATE_8_kHz;
                gyroSamplePeriod = 125.0f;
                break;
            }
        }
    } else {
        switch (gyro->mpuDetectionResult.sensor) {
        case ICM_20649_SPI:
            gyro->gyroRateKHz = GYRO_RATE_1100_Hz;
            gyroSamplePeriod = 1000000.0f / 1100.0f;
            break;
        default:
            gyro->gyroRateKHz = GYRO_RATE_1_kHz;
            gyroSamplePeriod = 1000.0f;
            break;
        }
        gyroSyncDenominator = 1; // Always full Sampling 1khz
    }

    // calculate gyro divider and targetLooptime (expected cycleTime)
    gyro->mpuDividerDrops  = gyroSyncDenominator - 1;
    const uint32_t targetLooptime = (uint32_t)(gyroSyncDenominator * gyroSamplePeriod);
    return targetLooptime;
}
