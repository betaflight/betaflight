/*
 * gyro_sync.c
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "sensor.h"
#include "accgyro.h"
#include "gyro_sync.h"


bool gyroSyncCheckUpdate(gyroDev_t *gyro)
{
    if (!gyro->intStatus)
        return false;
    return gyro->intStatus(gyro);
}

uint32_t gyroSetSampleRate(gyroDev_t *gyro, uint8_t lpf, uint8_t gyroSyncDenominator, bool gyro_use_32khz)
{
    float gyroSamplePeriod;

    if (lpf == GYRO_LPF_256HZ || lpf == GYRO_LPF_NONE) {
        if (gyro_use_32khz) {
            gyro->gyroRateKHz = GYRO_RATE_32_kHz;
            gyroSamplePeriod = 31.5f;
        } else {
            gyro->gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSamplePeriod = 125.0f;
        }
    } else {
        gyro->gyroRateKHz = GYRO_RATE_1_kHz;
        gyroSamplePeriod = 1000.0f;
        gyroSyncDenominator = 1; // Always full Sampling 1khz
    }

    // calculate gyro divider and targetLooptime (expected cycleTime)
    gyro->mpuDividerDrops  = gyroSyncDenominator - 1;
    const uint32_t targetLooptime = (uint32_t)(gyroSyncDenominator * gyroSamplePeriod);
    return targetLooptime;
}

uint8_t gyroMPU6xxxGetDividerDrops(const gyroDev_t *gyro)
{
    return gyro->mpuDividerDrops;
}
