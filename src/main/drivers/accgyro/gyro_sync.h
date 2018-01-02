/*
 * gyro_sync.h
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#pragma once

#include "drivers/accgyro/accgyro.h"

bool gyroSyncCheckUpdate(gyroDev_t *gyro);
uint32_t gyroSetSampleRate(gyroDev_t *gyro, uint8_t lpf, uint8_t gyroSyncDenominator, bool gyro_use_32khz);
