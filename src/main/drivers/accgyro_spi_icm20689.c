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
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "exti.h"
#include "io.h"
#include "bus_spi.h"
#include "gpio.h"
#include "gyro_sync.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_spi_icm20689.h"

#define DISABLE_ICM20689       IOHi(icmSpi20689CsPin)
#define ENABLE_ICM20689        IOLo(icmSpi20689CsPin)

static IO_t icmSpi20689CsPin = IO_NONE;

bool icm20689WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_ICM20689;
    spiTransferByte(ICM20689_SPI_INSTANCE, reg);
    spiTransferByte(ICM20689_SPI_INSTANCE, data);
    DISABLE_ICM20689;

    return true;
}

bool icm20689ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_ICM20689;
    spiTransferByte(ICM20689_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(ICM20689_SPI_INSTANCE, data, NULL, length);
    DISABLE_ICM20689;

    return true;
}

static void icm20689SpiInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

    icmSpi20689CsPin = IOGetByTag(IO_TAG(ICM20689_CS_PIN));
    IOInit(icmSpi20689CsPin, OWNER_MPU_CS, 0);
    IOConfigGPIO(icmSpi20689CsPin, SPI_IO_CS_CFG);

    spiSetDivisor(ICM20689_SPI_INSTANCE, SPI_CLOCK_STANDARD);

    hardwareInitialised = true;
}

bool icm20689SpiDetect(void)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 20;

    icm20689SpiInit();

    spiSetDivisor(ICM20689_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON); //low speed

    icm20689WriteRegister(MPU_RA_PWR_MGMT_1, ICM20689_BIT_RESET);

    do {
        delay(150);

        icm20689ReadRegister(MPU_RA_WHO_AM_I, 1, &tmp);
        if (tmp == ICM20689_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return false;
        }
    } while (attemptsRemaining--);

    spiSetDivisor(ICM20689_SPI_INSTANCE, SPI_CLOCK_STANDARD);

    return true;

}

void icm20689AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool icm20689SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != ICM_20689_SPI) {
        return false;
    }

    acc->init = icm20689AccInit;
    acc->read = mpuAccRead;

    return true;
}

void icm20689GyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    spiSetDivisor(ICM20689_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON);

    gyro->mpuConfiguration.write(MPU_RA_PWR_MGMT_1, ICM20689_BIT_RESET);
    delay(100);
    gyro->mpuConfiguration.write(MPU_RA_SIGNAL_PATH_RESET, 0x03);
    delay(100);
//    gyro->mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0);
//    delay(100);
    gyro->mpuConfiguration.write(MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    const uint8_t raGyroConfigData = gyro->gyroRateKHz > GYRO_RATE_8_kHz ? (INV_FSR_2000DPS << 3 | FCB_3600_32) : (INV_FSR_2000DPS << 3 | FCB_DISABLED);
    gyro->mpuConfiguration.write(MPU_RA_GYRO_CONFIG, raGyroConfigData);
    delay(15);
    gyro->mpuConfiguration.write(MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delay(15);
    gyro->mpuConfiguration.write(MPU_RA_CONFIG, gyro->lpf);
    delay(15);
    gyro->mpuConfiguration.write(MPU_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops(gyro)); // Get Divider Drops
    delay(100);

    // Data ready interrupt configuration
//    gyro->mpuConfiguration.write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
    gyro->mpuConfiguration.write(MPU_RA_INT_PIN_CFG, 0x10);  // INT_ANYRD_2CLEAR, BYPASS_EN

    delay(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    gyro->mpuConfiguration.write(MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
#endif

    spiSetDivisor(ICM20689_SPI_INSTANCE, SPI_CLOCK_STANDARD);
}

bool icm20689SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != ICM_20689_SPI) {
        return false;
    }

    gyro->init = icm20689GyroInit;
    gyro->read = mpuGyroRead;
    gyro->intStatus = mpuCheckDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}
