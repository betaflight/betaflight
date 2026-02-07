/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

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

uint16_t gyroSetSampleRate(gyroDev_t *gyro)
{
    uint16_t gyroSampleRateHz;
    uint16_t accSampleRateHz;

    switch (gyro->mpuDetectionResult.sensor) {
        case BMI_160_SPI:
            gyro->gyroRateKHz = GYRO_RATE_3200_Hz;
            gyroSampleRateHz = 3200;
            accSampleRateHz = 800;
            break;
        case BMI_270_SPI:
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
            if (gyro->hardware_lpf == GYRO_HARDWARE_LPF_EXPERIMENTAL) {
                // 6.4KHz sampling, but data is unfiltered (no hardware DLPF)
                gyro->gyroRateKHz = GYRO_RATE_6400_Hz;
                gyroSampleRateHz = 6400;
            } else
#endif
            {
                gyro->gyroRateKHz = GYRO_RATE_3200_Hz;
                gyroSampleRateHz = 3200;
            }
            accSampleRateHz = 800;
            break;
        case ICM_20649_SPI:
            gyro->gyroRateKHz = GYRO_RATE_9_kHz;
            gyroSampleRateHz = 9000;
            accSampleRateHz = 1125;
            break;

       case ICM_20948_SPI: 
            gyro->gyroRateKHz = GYRO_RATE_8_kHz;  // Start with 8kHz for safety
            gyroSampleRateHz = 8000;
            accSampleRateHz = 1000;               // 1kHz is a safe Accel rate
            break;

#ifdef USE_ACCGYRO_LSM6DSO
        case LSM6DSO_SPI:
            gyro->gyroRateKHz = GYRO_RATE_6664_Hz;
            gyroSampleRateHz = 6664;   // Yes, this is correct per the datasheet. Will effectively round to 150us and 6.67KHz.
            accSampleRateHz = 833;
            break;
#endif
        case ICM_45686_SPI:
        case ICM_45605_SPI:
            gyro->gyroRateKHz = GYRO_RATE_6400_Hz;
            gyroSampleRateHz = 6400;
            accSampleRateHz = 1600;
            break;
        default:
            gyro->gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSampleRateHz = 8000;
            accSampleRateHz = 1000;
            break;
    }

    gyro->mpuDividerDrops  = 0; // we no longer use the gyro's sample divider
    gyro->accSampleRateHz = accSampleRateHz;
    return gyroSampleRateHz;
}
