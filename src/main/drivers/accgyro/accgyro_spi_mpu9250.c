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

#include "drivers/io.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/exti.h"
#include "drivers/bus_spi.h"
#include "drivers/gyro_sync.h"
#include "drivers/light_led.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"

static bool mpuSpi9250InitDone = false;

#define DISABLE_MPU9250(spiCsnPin)       IOHi(spiCsnPin)
#define ENABLE_MPU9250(spiCsnPin)        IOLo(spiCsnPin)

bool mpu9250SpiReadRegister(const busDevice_t *bus, uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_MPU9250(bus->spi.csnPin);
    spiTransferByte(MPU9250_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU9250_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU9250(bus->spi.csnPin);

    return true;
}

bool mpu9250SpiSlowReadRegister(const busDevice_t *bus, uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_MPU9250(bus->spi.csnPin);
    delayMicroseconds(1);
    spiTransferByte(MPU9250_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU9250_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU9250(bus->spi.csnPin);
    delayMicroseconds(1);

    return true;
}

bool mpu9250SpiWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    ENABLE_MPU9250(bus->spi.csnPin);
    delayMicroseconds(1);
    spiTransferByte(MPU9250_SPI_INSTANCE, reg);
    spiTransferByte(MPU9250_SPI_INSTANCE, data);
    DISABLE_MPU9250(bus->spi.csnPin);
    delayMicroseconds(1);

    return true;
}

bool verifympu9250SpiWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    mpu9250SpiWriteRegister(bus, reg, data);
    delayMicroseconds(100);

    uint8_t attemptsRemaining = 20;
    do {
        uint8_t in;
        mpu9250SpiSlowReadRegister(bus, reg, 1, &in);
        if (in == data) {
            return true;
        } else {
            debug[3]++;
            mpu9250SpiWriteRegister(bus, reg, data);
            delayMicroseconds(100);
        }
    } while (attemptsRemaining--);
    return false;
}

void mpu9250SpiResetGyro(void)
{
    // Device Reset
#ifdef MPU9250_CS_PIN
    busDevice_t bus = { .spi = { .csnPin = IOGetByTag(IO_TAG(MPU9250_CS_PIN)) } };
    mpu9250SpiWriteRegister(&bus, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
    delay(150);
#endif
}

static void mpu9250AccAndGyroInit(gyroDev_t *gyro)
{
    if (mpuSpi9250InitDone) {
        return;
    }

    spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON); //low speed for writing to slow registers

    mpu9250SpiWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
    delay(50);

    verifympu9250SpiWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);

    //Fchoice_b defaults to 00 which makes fchoice 11
    const uint8_t raGyroConfigData = gyro->gyroRateKHz > GYRO_RATE_8_kHz ? (INV_FSR_2000DPS << 3 | FCB_3600_32) : (INV_FSR_2000DPS << 3 | FCB_DISABLED);
    verifympu9250SpiWriteRegister(&gyro->bus, MPU_RA_GYRO_CONFIG, raGyroConfigData);

    if (gyro->lpf == 4) {
        verifympu9250SpiWriteRegister(&gyro->bus, MPU_RA_CONFIG, 1); //1KHz, 184DLPF
    } else if (gyro->lpf < 4) {
        verifympu9250SpiWriteRegister(&gyro->bus, MPU_RA_CONFIG, 7); //8KHz, 3600DLPF
    } else if (gyro->lpf > 4) {
        verifympu9250SpiWriteRegister(&gyro->bus, MPU_RA_CONFIG, 0); //8KHz, 250DLPF
    }

    verifympu9250SpiWriteRegister(&gyro->bus, MPU_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops(gyro));

    verifympu9250SpiWriteRegister(&gyro->bus, MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    verifympu9250SpiWriteRegister(&gyro->bus, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN

#if defined(USE_MPU_DATA_READY_SIGNAL)
    verifympu9250SpiWriteRegister(&gyro->bus, MPU_RA_INT_ENABLE, 0x01); //this resets register MPU_RA_PWR_MGMT_1 and won't read back correctly.
#endif

    spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_FAST);

    mpuSpi9250InitDone = true; //init done
}

bool mpu9250SpiDetect(const busDevice_t *bus)
{
    /* not the best place for this - should really have an init method */
    IOInit(bus->spi.csnPin, OWNER_MPU, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(bus->spi.csnPin, SPI_IO_CS_CFG);

    spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON); //low speed
    mpu9250SpiWriteRegister(bus, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);

    uint8_t attemptsRemaining = 20;
    do {
        delay(150);
        uint8_t in;
        mpu9250SpiReadRegister(bus, MPU_RA_WHO_AM_I, 1, &in);
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

static void mpu9250SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 8;
}

bool mpu9250SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_9250_SPI) {
        return false;
    }

    acc->initFn = mpu9250SpiAccInit;
    acc->readFn = mpuAccRead;

    return true;
}

static void mpu9250SpiGyroInit(gyroDev_t *gyro)
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

bool mpu9250SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_9250_SPI) {
        return false;
    }

    gyro->initFn = mpu9250SpiGyroInit;
    gyro->readFn = mpuGyroRead;
    gyro->intStatusFn = mpuCheckDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}
