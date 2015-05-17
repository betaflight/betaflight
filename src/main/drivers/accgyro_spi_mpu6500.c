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
#include "gpio.h"
#include "bus_spi.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_spi_mpu6500.h"

enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

#define DISABLE_MPU6500       GPIO_SetBits(MPU6500_CS_GPIO,   MPU6500_CS_PIN)
#define ENABLE_MPU6500        GPIO_ResetBits(MPU6500_CS_GPIO, MPU6500_CS_PIN)

static uint8_t mpuLowPassFilter = INV_FILTER_42HZ;

static void mpu6500AccInit(void);
static void mpu6500AccRead(int16_t *accData);
static void mpu6500GyroInit(void);
static void mpu6500GyroRead(int16_t *gyroADC);

extern uint16_t acc_1G;

static void mpu6500WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_MPU6500;
    spiTransferByte(MPU6500_SPI_INSTANCE, reg);
    spiTransferByte(MPU6500_SPI_INSTANCE, data);
    DISABLE_MPU6500;
}

static void mpu6500ReadRegister(uint8_t reg, uint8_t *data, int length)
{
    ENABLE_MPU6500;
    spiTransferByte(MPU6500_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU6500_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU6500;
}

static bool mpu6500Detect(void)
{
    uint8_t tmp;

    mpu6500ReadRegister(MPU6500_RA_WHOAMI, &tmp, 1);
    if (tmp != MPU6500_WHO_AM_I_CONST)
        return false;

    return true;
}

bool mpu6500SpiAccDetect(acc_t *acc)
{
    if (!mpu6500Detect()) {
        return false;
    }

    acc->init = mpu6500AccInit;
    acc->read = mpu6500AccRead;

    return true;
}

bool mpu6500SpiGyroDetect(gyro_t *gyro, uint16_t lpf)
{
    if (!mpu6500Detect()) {
        return false;
    }

    gyro->init = mpu6500GyroInit;
    gyro->read = mpu6500GyroRead;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;
    //gyro->scale = (4.0f / 16.4f) * (M_PIf / 180.0f) * 0.000001f;

    // default lpf is 42Hz
    if (lpf >= 188)
        mpuLowPassFilter = INV_FILTER_188HZ;
    else if (lpf >= 98)
        mpuLowPassFilter = INV_FILTER_98HZ;
    else if (lpf >= 42)
        mpuLowPassFilter = INV_FILTER_42HZ;
    else if (lpf >= 20)
        mpuLowPassFilter = INV_FILTER_20HZ;
    else if (lpf >= 10)
        mpuLowPassFilter = INV_FILTER_10HZ;
    else
        mpuLowPassFilter = INV_FILTER_5HZ;

    return true;
}

static void mpu6500AccInit(void)
{
    acc_1G = 512 * 8;
}

static void mpu6500AccRead(int16_t *accData)
{
    uint8_t buf[6];

    mpu6500ReadRegister(MPU6500_RA_ACCEL_XOUT_H, buf, 6);

    accData[X] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[Y] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[Z] = (int16_t)((buf[4] << 8) | buf[5]);
}

static void mpu6500GyroInit(void)
{

#ifdef NAZE
    gpio_config_t gpio;
    // MPU_INT output on rev5 hardware (PC13). rev4 was on PB13, conflicts with SPI devices
    if (hse_value == 12000000) {
        gpio.pin = Pin_13;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_IN_FLOATING;
        gpioInit(GPIOC, &gpio);
    }
#endif

    mpu6500WriteRegister(MPU6500_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
    delay(100);
    mpu6500WriteRegister(MPU6500_RA_PWR_MGMT_1, 0);
    delay(100);
    mpu6500WriteRegister(MPU6500_RA_PWR_MGMT_1, INV_CLK_PLL);
    mpu6500WriteRegister(MPU6500_RA_GYRO_CFG, INV_FSR_2000DPS << 3);
    mpu6500WriteRegister(MPU6500_RA_ACCEL_CFG, INV_FSR_8G << 3);
    mpu6500WriteRegister(MPU6500_RA_LPF, mpuLowPassFilter);
    mpu6500WriteRegister(MPU6500_RA_RATE_DIV, 0); // 1kHz S/R
}

static void mpu6500GyroRead(int16_t *gyroADC)
{
    uint8_t buf[6];

    mpu6500ReadRegister(MPU6500_RA_GYRO_XOUT_H, buf, 6);

    gyroADC[X] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroADC[Y] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroADC[Z] = (int16_t)((buf[4] << 8) | buf[5]);
}
