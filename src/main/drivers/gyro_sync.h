/*
 * gyro_sync.h
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#define GYRO_LPF_256HZ      0
#define GYRO_LPF_188HZ      1
#define GYRO_LPF_98HZ       2
#define GYRO_LPF_42HZ       3
#define GYRO_LPF_20HZ       4
#define GYRO_LPF_10HZ       5
#define GYRO_LPF_5HZ        6
#define GYRO_LPF_NONE       7

struct gyro_s;
extern uint32_t lastGyroInterruptCallDelta;
bool pidScheduledToRun(void);
uint8_t gyroMPU6xxxGetDividerDrops(void);
uint32_t gyroSetSamplingInterval(uint8_t lpf, uint8_t gyroSyncDenominator, uint8_t pidDenom);
void gyroHandleInterrupt(void);
