/*
 * gyro_sync.h
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

struct gyroDev_s;

bool gyroSyncCheckUpdate(struct gyroDev_s *gyro);
uint8_t gyroMPU6xxxGetDividerDrops(void);
uint32_t gyroSetSampleRate(uint8_t lpf, uint8_t gyroSyncDenominator);
