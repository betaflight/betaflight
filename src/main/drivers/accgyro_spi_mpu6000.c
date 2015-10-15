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


/*
 * Authors:
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "gpio.h"
#include "exti.h"
#include "bus_spi.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_spi_mpu6000.h"

static void mpu6000AccAndGyroInit(void);

extern uint8_t mpuLowPassFilter;

static bool mpuSpi6000InitDone = false;

// Bits
#define BIT_SLEEP				    0x40
#define BIT_H_RESET				    0x80
#define BITS_CLKSEL				    0x07
#define MPU_CLK_SEL_PLLGYROX	    0x01
#define MPU_CLK_SEL_PLLGYROZ	    0x03
#define MPU_EXT_SYNC_GYROX		    0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN			    0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA		    0x01
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

// Product ID Description for MPU6000
// high 4 bits low 4 bits
// Product Name Product Revision
#define MPU6000ES_REV_C4 0x14
#define MPU6000ES_REV_C5 0x15
#define MPU6000ES_REV_D6 0x16
#define MPU6000ES_REV_D7 0x17
#define MPU6000ES_REV_D8 0x18
#define MPU6000_REV_C4 0x54
#define MPU6000_REV_C5 0x55
#define MPU6000_REV_D6 0x56
#define MPU6000_REV_D7 0x57
#define MPU6000_REV_D8 0x58
#define MPU6000_REV_D9 0x59
#define MPU6000_REV_D10 0x5A

#define DISABLE_MPU6000       GPIO_SetBits(MPU6000_CS_GPIO,   MPU6000_CS_PIN)
#define ENABLE_MPU6000        GPIO_ResetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN)


bool mpu6000WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, reg);
    spiTransferByte(MPU6000_SPI_INSTANCE, data);
    DISABLE_MPU6000;

    return true;
}

bool mpu6000ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU6000_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU6000;

    return true;
}

void mpu6000SpiGyroInit(uint16_t lpf)
{
    mpuIntExtiInit();

    uint8_t mpuLowPassFilter = determineMPULPF(lpf);

    mpu6000AccAndGyroInit();

    spiResetErrorCounter(MPU6000_SPI_INSTANCE);

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_0_5625MHZ_CLOCK_DIVIDER);

    // Accel and Gyro DLPF Setting
    mpu6000WriteRegister(MPU6000_CONFIG, mpuLowPassFilter);
    delayMicroseconds(1);

    int16_t data[3];
    mpuGyroRead(data);

    if ((((int8_t)data[1]) == -1 && ((int8_t)data[0]) == -1) || spiGetErrorCounter(MPU6000_SPI_INSTANCE) != 0) {
        spiResetErrorCounter(MPU6000_SPI_INSTANCE);
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

void mpu6000SpiAccInit(void)
{
    mpuIntExtiInit();

    acc_1G = 512 * 8;
}

bool mpu6000SpiDetect(void)
{
    uint8_t in;
    uint8_t attemptsRemaining = 5;

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_0_5625MHZ_CLOCK_DIVIDER);

    mpu6000WriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET);

    do {
        delay(150);

        mpu6000ReadRegister(MPU_RA_WHO_AM_I, 1, &in);
        if (in == MPU6000_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return false;
        }
    } while (attemptsRemaining--);


    mpu6000ReadRegister(MPU_RA_PRODUCT_ID, 1, &in);

    /* look for a product ID we recognise */

    // verify product revision
    switch (in) {
        case MPU6000ES_REV_C4:
        case MPU6000ES_REV_C5:
        case MPU6000_REV_C4:
        case MPU6000_REV_C5:
        case MPU6000ES_REV_D6:
        case MPU6000ES_REV_D7:
        case MPU6000ES_REV_D8:
        case MPU6000_REV_D6:
        case MPU6000_REV_D7:
        case MPU6000_REV_D8:
        case MPU6000_REV_D9:
        case MPU6000_REV_D10:
            return true;
    }

    return false;
}

static void mpu6000AccAndGyroInit(void) {

    if (mpuSpi6000InitDone) {
        return;
    }

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_0_5625MHZ_CLOCK_DIVIDER);

    // Device Reset
    mpu6000WriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    delay(150);

    mpu6000WriteRegister(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay(150);

    // Clock Source PPL with Z axis gyro reference
    mpu6000WriteRegister(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayMicroseconds(1);

    // Disable Primary I2C Interface
    mpu6000WriteRegister(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(1);

    mpu6000WriteRegister(MPU_RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(1);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    mpu6000WriteRegister(MPU_RA_SMPLRT_DIV, 0x00);
    delayMicroseconds(1);

    // Gyro +/- 1000 DPS Full Scale
    mpu6000WriteRegister(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delayMicroseconds(1);

    // Accel +/- 8 G Full Scale
    mpu6000WriteRegister(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    delayMicroseconds(1);


    mpu6000WriteRegister(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    delayMicroseconds(1);

#ifdef USE_MPU_DATA_READY_SIGNAL
    mpu6000WriteRegister(MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    delayMicroseconds(1);
#endif

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_18MHZ_CLOCK_DIVIDER);  // 18 MHz SPI clock

    mpuSpi6000InitDone = true;
}

bool mpu6000SpiAccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    acc->init = mpu6000SpiAccInit;
    acc->read = mpuAccRead;

    return true;
}

bool mpu6000SpiGyroDetect(gyro_t *gyro)
{
    if (mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    gyro->init = mpu6000SpiGyroInit;
    gyro->read = mpuGyroRead;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}
