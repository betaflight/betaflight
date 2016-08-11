/*
 * gyro_sync.c
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "build/debug.h"

#include "sensor.h"
#include "accgyro.h"
#include "gyro_sync.h"
#include "system.h"

#include "common/axis.h"

#include "sensors/gyro.h"

#define GYRO_LPF_256HZ      0
#define GYRO_LPF_188HZ      1
#define GYRO_LPF_98HZ       2
#define GYRO_LPF_42HZ       3
#define GYRO_LPF_20HZ       4
#define GYRO_LPF_10HZ       5
#define GYRO_LPF_5HZ        6
#define GYRO_LPF_NONE       7

static int mpuDividerDrops;
static volatile bool isPidScheduledToRun;
static int pidProcessDenom;
uint32_t lastGyroInterruptCallDelta;

bool pidScheduledToRun(void)
{
    bool ret;
    if (isPidScheduledToRun) {
        ret = true;
        isPidScheduledToRun= false;
    } else {
        ret = false;
    }
    return ret;
}

uint32_t gyroSetSampleRate(uint8_t lpf, uint8_t gyroSyncDenominator, uint8_t pidDenom)
{
    int gyroSamplePeriod;
    pidProcessDenom = pidDenom;

    if (lpf == GYRO_LPF_256HZ || lpf == GYRO_LPF_NONE) {
        gyroSamplePeriod = 125;
    } else {
        gyroSamplePeriod = 1000;
        gyroSyncDenominator = 1; // Always full Sampling 1khz
    }

    // calculate gyro divider and targetLooptime (expected cycleTime)
    mpuDividerDrops  = gyroSyncDenominator - 1;
    const uint32_t targetLooptime = gyroSyncDenominator * gyroSamplePeriod;
    return targetLooptime;
}

uint8_t gyroMPU6xxxGetDividerDrops(void)
{
    return mpuDividerDrops;
}

void gyroHandleInterrupt(void) {
	static int16_t gyroADCRaw[XYZ_AXIS_COUNT];
	static int pidProcessCountDown;

//#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    static uint32_t lastGyroInterruptCallAt = 0;
    uint32_t now = micros();
    lastGyroInterruptCallDelta = now - lastGyroInterruptCallAt;
    debug[0] = lastGyroInterruptCallDelta;
    lastGyroInterruptCallAt = now;

    if (!gyro.read(gyroADCRaw)) {
        return;
    }

    processGyroData(gyroADCRaw);

    if (pidProcessCountDown) {
        pidProcessCountDown--;
    } else {
        pidProcessCountDown = pidProcessDenom - 1;
        isPidScheduledToRun = true;
    }
}
