/*
 * gyro_sync.h
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

extern uint32_t targetLooptime;

bool gyroSyncCheckUpdate(void);
uint8_t gyroMPU6xxxGetDividerDrops(void);
void gyroUpdateSampleRate(void);
