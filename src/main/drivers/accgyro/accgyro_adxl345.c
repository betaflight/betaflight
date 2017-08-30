/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/system.h"
#include "drivers/bus_i2c.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_adxl345.h"

// ADXL345, Alternative address mode 0x53
#define ADXL345_ADDRESS     0x53

// Registers
#define ADXL345_BW_RATE     0x2C
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_INT_ENABLE  0x2E
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATA_OUT    0x32
#define ADXL345_FIFO_CTL    0x38

// BW_RATE values
#define ADXL345_RATE_50     0x09
#define ADXL345_RATE_100    0x0A
#define ADXL345_RATE_200    0x0B
#define ADXL345_RATE_400    0x0C
#define ADXL345_RATE_800    0x0D
#define ADXL345_RATE_1600   0x0E
#define ADXL345_RATE_3200   0x0F

// various register values
#define ADXL345_POWER_MEAS  0x08
#define ADXL345_FULL_RANGE  0x08
#define ADXL345_RANGE_2G    0x00
#define ADXL345_RANGE_4G    0x01
#define ADXL345_RANGE_8G    0x02
#define ADXL345_RANGE_16G   0x03
#define ADXL345_FIFO_STREAM 0x80

static bool useFifo = false;

static void adxl345Init(accDev_t *acc)
{
    if (useFifo) {
        uint8_t fifoDepth = 16;
        i2cWrite(MPU_I2C_INSTANCE, ADXL345_ADDRESS, ADXL345_POWER_CTL, ADXL345_POWER_MEAS);
        i2cWrite(MPU_I2C_INSTANCE, ADXL345_ADDRESS, ADXL345_DATA_FORMAT, ADXL345_FULL_RANGE | ADXL345_RANGE_8G);
        i2cWrite(MPU_I2C_INSTANCE, ADXL345_ADDRESS, ADXL345_BW_RATE, ADXL345_RATE_400);
        i2cWrite(MPU_I2C_INSTANCE, ADXL345_ADDRESS, ADXL345_FIFO_CTL, (fifoDepth & 0x1F) | ADXL345_FIFO_STREAM);
    } else {
        i2cWrite(MPU_I2C_INSTANCE, ADXL345_ADDRESS, ADXL345_POWER_CTL, ADXL345_POWER_MEAS);
        i2cWrite(MPU_I2C_INSTANCE, ADXL345_ADDRESS, ADXL345_DATA_FORMAT, ADXL345_FULL_RANGE | ADXL345_RANGE_8G);
        i2cWrite(MPU_I2C_INSTANCE, ADXL345_ADDRESS, ADXL345_BW_RATE, ADXL345_RATE_100);
    }
    acc->acc_1G = 256; // 3.3V operation
}

uint8_t acc_samples = 0;

static bool adxl345Read(accDev_t *acc)
{
    uint8_t buf[8];

    if (useFifo) {
        int32_t x = 0;
        int32_t y = 0;
        int32_t z = 0;
        uint8_t i = 0;
        uint8_t samples_remaining;

        do {
            i++;

            if (!i2cRead(MPU_I2C_INSTANCE, ADXL345_ADDRESS, ADXL345_DATA_OUT, 8, buf)) {
                return false;
            }

            x += (int16_t)(buf[0] + (buf[1] << 8));
            y += (int16_t)(buf[2] + (buf[3] << 8));
            z += (int16_t)(buf[4] + (buf[5] << 8));
            samples_remaining = buf[7] & 0x7F;
        } while ((i < 32) && (samples_remaining > 0));
        acc->ADCRaw[0] = x / i;
        acc->ADCRaw[1] = y / i;
        acc->ADCRaw[2] = z / i;
        acc_samples = i;
    } else {

        if (!i2cRead(MPU_I2C_INSTANCE, ADXL345_ADDRESS, ADXL345_DATA_OUT, 6, buf)) {
            return false;
        }

        acc->ADCRaw[0] = buf[0] + (buf[1] << 8);
        acc->ADCRaw[1] = buf[2] + (buf[3] << 8);
        acc->ADCRaw[2] = buf[4] + (buf[5] << 8);
    }

    return true;
}

bool adxl345Detect(accDev_t *acc, drv_adxl345_config_t *init)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(MPU_I2C_INSTANCE, ADXL345_ADDRESS, 0x00, 1, &sig);
    if (!ack || sig != 0xE5)
        return false;

    // use ADXL345's fifo to filter data or not
    useFifo = init->useFifo;

    acc->initFn = adxl345Init;
    acc->readFn = adxl345Read;
    return true;
}
