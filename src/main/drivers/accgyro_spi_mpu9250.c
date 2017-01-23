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
 * Kalyn Doerr (RS2K) - Robust rewrite
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "io.h"

#include "system.h"
#include "exti.h"
#include "bus_spi.h"
#include "gyro_sync.h"
#include "light_led.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_spi_mpu9250.h"

static void mpu9250AccAndGyroInit(gyroDev_t *gyro);

static bool mpuSpi9250InitDone = false;

static IO_t mpuSpi9250CsPin = IO_NONE;

#define DISABLE_MPU9250       IOHi(mpuSpi9250CsPin)
#define ENABLE_MPU9250        IOLo(mpuSpi9250CsPin)

void mpu9250ResetGyro(void)
{
    // Device Reset
    mpu9250WriteRegister(MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
    delay(150);
}

bool mpu9250WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_MPU9250;
    delayMicroseconds(1);
    spiTransferByte(MPU9250_SPI_INSTANCE, reg);
    spiTransferByte(MPU9250_SPI_INSTANCE, data);
    DISABLE_MPU9250;
    delayMicroseconds(1);

    return true;
}

bool mpu9250ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_MPU9250;
    spiTransferByte(MPU9250_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU9250_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU9250;

    return true;
}

bool mpu9250SlowReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_MPU9250;
    delayMicroseconds(1);
    spiTransferByte(MPU9250_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU9250_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU9250;
    delayMicroseconds(1);

    return true;
}

void mpu9250SpiGyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    mpu9250AccAndGyroInit(gyro);

    spiResetErrorCounter(MPU9250_SPI_INSTANCE);

    spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_FAST); //high speed now that we don't need to write to the slow registers

    mpuGyroRead(gyro);

    if ((((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1) || spiGetErrorCounter(MPU9250_SPI_INSTANCE) != 0) {
        spiResetErrorCounter(MPU9250_SPI_INSTANCE);
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

void mpu9250SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 8;
}

bool verifympu9250WriteRegister(uint8_t reg, uint8_t data)
{
    uint8_t in;
    uint8_t attemptsRemaining = 20;

    mpu9250WriteRegister(reg, data);
    delayMicroseconds(100);

    do {
        mpu9250SlowReadRegister(reg, 1, &in);
        if (in == data) {
            return true;
        } else {
            debug[3]++;
            mpu9250WriteRegister(reg, data);
            delayMicroseconds(100);
        }
    } while (attemptsRemaining--);
    return false;
}

static void mpu9250AccAndGyroInit(gyroDev_t *gyro) {

    if (mpuSpi9250InitDone) {
        return;
    }

    spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON); //low speed for writing to slow registers

    mpu9250WriteRegister(MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
    delay(50);

    verifympu9250WriteRegister(MPU_RA_PWR_MGMT_1, INV_CLK_PLL);

    //Fchoice_b defaults to 00 which makes fchoice 11
    const uint8_t raGyroConfigData = gyro->gyroRateKHz > GYRO_RATE_8_kHz ? (INV_FSR_2000DPS << 3 | FCB_3600_32) : (INV_FSR_2000DPS << 3 | FCB_DISABLED);
    verifympu9250WriteRegister(MPU_RA_GYRO_CONFIG, raGyroConfigData);

    if (gyro->lpf == 4) {
        verifympu9250WriteRegister(MPU_RA_CONFIG, 1); //1KHz, 184DLPF
    } else if (gyro->lpf < 4) {
        verifympu9250WriteRegister(MPU_RA_CONFIG, 7); //8KHz, 3600DLPF
    } else if (gyro->lpf > 4) {
        verifympu9250WriteRegister(MPU_RA_CONFIG, 0); //8KHz, 250DLPF
    }

    verifympu9250WriteRegister(MPU_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops(gyro));

    verifympu9250WriteRegister(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    verifympu9250WriteRegister(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN

#if defined(USE_MPU_DATA_READY_SIGNAL)
    verifympu9250WriteRegister(MPU_RA_INT_ENABLE, 0x01); //this resets register MPU_RA_PWR_MGMT_1 and won't read back correctly.
#endif

    spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_FAST);

    mpuSpi9250InitDone = true; //init done
}

bool mpu9250SpiDetect(void)
{
    uint8_t in;
    uint8_t attemptsRemaining = 20;

    /* not the best place for this - should really have an init method */
#ifdef MPU9250_CS_PIN
    mpuSpi9250CsPin = IOGetByTag(IO_TAG(MPU9250_CS_PIN));
#endif
    IOInit(mpuSpi9250CsPin, OWNER_MPU_CS, 0);
    IOConfigGPIO(mpuSpi9250CsPin, SPI_IO_CS_CFG);

    spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON); //low speed
    mpu9250WriteRegister(MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);

    do {
        delay(150);

        mpu9250ReadRegister(MPU_RA_WHO_AM_I, 1, &in);
        if (in == MPU9250_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return false;
        }
    } while (attemptsRemaining--);

    spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_FAST);

    return true;
}

bool mpu9250SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_9250_SPI) {
        return false;
    }

    acc->init = mpu9250SpiAccInit;
    acc->read = mpuAccRead;

    return true;
}

bool mpu9250SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_9250_SPI) {
        return false;
    }

    gyro->init = mpu9250SpiGyroInit;
    gyro->read = mpuGyroRead;
    gyro->intStatus = mpuCheckDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}
