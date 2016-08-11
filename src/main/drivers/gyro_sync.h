/*
 * gyro_sync.h
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

struct gyro_s;
extern uint32_t lastGyroInterruptCallDelta;
bool pidScheduledToRun(void);
uint8_t gyroMPU6xxxGetDividerDrops(void);
uint32_t gyroSetSampleRate(uint8_t lpf, uint8_t gyroSyncDenominator, uint8_t pidDenom);
void gyroHandleInterrupt(void);
