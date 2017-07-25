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
#include <string.h>

#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "drivers/compass/compass.h"
#include "drivers/compass/compass_ak8963.h"
#include "drivers/compass/compass_spi_ak8963.h"

#ifdef USE_MAG_SPI_AK8963

static float magGain[3] = { 1.0f, 1.0f, 1.0f };
static busDevice_t *bus = NULL;

static bool ak8963SpiInit(void)
{
    uint8_t calibration[3];
    uint8_t status;

    UNUSED(status);

    spiBusWriteRegister(bus, AK8963_MAG_REG_I2CDIS, I2CDIS_DISABLE_MASK); // disable I2C
    delay(10);

    spiBusWriteRegister(bus, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN); // power down before entering fuse mode
    delay(20);

    spiBusWriteRegister(bus, AK8963_MAG_REG_CNTL1, CNTL1_MODE_FUSE_ROM); // Enter Fuse ROM access mode
    delay(10);

    spiBusReadRegisterBuffer(bus, AK8963_MAG_REG_ASAX, calibration, sizeof(calibration)); // Read the x-, y-, and z-axis calibration values
    delay(10);

    magGain[X] = ((((float)(int8_t)calibration[X] - 128) / 256) + 1) * 30;
    magGain[Y] = ((((float)(int8_t)calibration[Y] - 128) / 256) + 1) * 30;
    magGain[Z] = ((((float)(int8_t)calibration[Z] - 128) / 256) + 1) * 30;

    spiBusWriteRegister(bus, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN); // power down after reading.
    delay(10);

    // Clear status registers
    status = spiBusReadRegister(bus, AK8963_MAG_REG_ST1);
    status = spiBusReadRegister(bus, AK8963_MAG_REG_ST2);

    // Trigger first measurement
    spiBusWriteRegister(bus, AK8963_MAG_REG_CNTL1, CNTL1_MODE_ONCE);
    return true;
}

static bool ak8963SpiRead(int16_t *magData)
{
    bool ack = false;
    uint8_t buf[7];

    uint8_t status = spiBusReadRegister(bus, AK8963_MAG_REG_ST1);

    if (!ack || (status & ST1_DATA_READY) == 0) {
        return false;
    }

    ack = spiBusReadRegisterBuffer(bus, AK8963_MAG_REG_HXL, &buf[0], 7);
    uint8_t status2 = buf[6];
    if (!ack || (status2 & ST2_DATA_ERROR) || (status2 & ST2_MAG_SENSOR_OVERFLOW)) {
        return false;
    }

    magData[X] = -(int16_t)(buf[1] << 8 | buf[0]) * magGain[X];
    magData[Y] = -(int16_t)(buf[3] << 8 | buf[2]) * magGain[Y];
    magData[Z] = -(int16_t)(buf[5] << 8 | buf[4]) * magGain[Z];

    return spiBusWriteRegister(bus, AK8963_MAG_REG_CNTL1, CNTL1_MODE_ONCE); // start reading again
}

bool ak8963SpiDetect(magDev_t *mag)
{
    uint8_t sig = 0;

    // check for SPI AK8963
    bool ack = spiBusReadRegisterBuffer(&mag->bus, AK8963_MAG_REG_WIA, &sig, 1);
    if (ack && sig == AK8963_Device_ID) // 0x48 / 01001000 / 'H'
    {
        bus = &mag->bus;

        mag->init = ak8963SpiInit;
        mag->read = ak8963SpiRead;

        return true;
    }
    return false;
}
#endif
