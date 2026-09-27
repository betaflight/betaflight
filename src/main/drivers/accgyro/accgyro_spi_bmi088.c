/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * BMI088 register definitions and timings are from Bosch Sensortec's
 * BMI088 data sheet BST-BMI088-DS000-19, revision 1.9 (January 2024).
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ACCGYRO_BMI088

#include "common/axis.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi088.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/resource.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

#define BMI088_MAX_SPI_CLK_HZ             10000000
#define BMI088_DETECT_SPI_CLK_HZ          1000000
#define BMI088_SPI_READ                   0x80
#define BMI088_AXIS_DATA_LENGTH           6

// Require sustained pulses during startup before declaring the EXTI usable.
#define BMI088_GYRO_EXTI_DETECT_THRESHOLD 1000

#define BMI088_ACC_REG_CHIP_ID            0x00
#define BMI088_ACC_REG_DATA               0x12
#define BMI088_ACC_REG_CONF               0x40
#define BMI088_ACC_REG_RANGE              0x41
#define BMI088_ACC_REG_PWR_CONF           0x7C
#define BMI088_ACC_REG_PWR_CTRL           0x7D
#define BMI088_ACC_REG_SOFTRESET          0x7E

#define BMI088_ACC_CONF_OSR2_1600HZ       0x9C
#define BMI088_ACC_RANGE_12G              0x02
#define BMI088_ACC_RANGE_24G              0x03
#define BMI088_ACC_PWR_ACTIVE             0x00
#define BMI088_ACC_PWR_ENABLE             0x04

#define BMI088_GYRO_REG_CHIP_ID           0x00
#define BMI088_GYRO_REG_DATA              0x02
#define BMI088_GYRO_REG_RANGE             0x0F
#define BMI088_GYRO_REG_BANDWIDTH         0x10
#define BMI088_GYRO_REG_LPM1              0x11
#define BMI088_GYRO_REG_SOFTRESET         0x14
#define BMI088_GYRO_REG_INT_CTRL          0x15
#define BMI088_GYRO_REG_INT_IO_CONF       0x16
#define BMI088_GYRO_REG_INT_IO_MAP        0x18

#define BMI088_GYRO_RANGE_2000DPS         0x00
// Complete reset-compatible values with the ODR/bandwidth selection in bits 3:0.
#define BMI088_GYRO_BW_532_ODR_2000HZ     0x80
#define BMI088_GYRO_BW_230_ODR_2000HZ     0x81
#define BMI088_GYRO_PM_NORMAL             0x00
#define BMI088_GYRO_DRDY_ENABLE           0x80
#define BMI088_GYRO_INT3_PUSH_PULL_HIGH   0x01
#define BMI088_GYRO_MAP_DRDY_INT3         0x01

#define BMI088_SOFTRESET                  0xB6

STATIC_UNIT_TESTED uint16_t bmi088AccOneG(bool highFsr)
{
    // Bosch specifies typical sensitivities of 2730 LSB/g at 12 g and
    // 1365 LSB/g at 24 g. Betaflight represents this as an integer.
    return highFsr ? 1365 : 2730;
}

STATIC_UNIT_TESTED uint8_t bmi088AccRange(bool highFsr)
{
    return highFsr ? BMI088_ACC_RANGE_24G : BMI088_ACC_RANGE_12G;
}

STATIC_UNIT_TESTED uint8_t bmi088GyroBandwidth(uint8_t hardwareLpf)
{
    switch (hardwareLpf) {
    case GYRO_HARDWARE_LPF_NORMAL:
        return BMI088_GYRO_BW_230_ODR_2000HZ;
    case GYRO_HARDWARE_LPF_OPTION_1:
        return BMI088_GYRO_BW_230_ODR_2000HZ;
    case GYRO_HARDWARE_LPF_OPTION_2:
        FALLTHROUGH;
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    case GYRO_HARDWARE_LPF_EXPERIMENTAL:
#endif
    default:
        return BMI088_GYRO_BW_532_ODR_2000HZ;
    }
}

STATIC_UNIT_TESTED void bmi088DecodeLittleEndian(const uint8_t raw[6], int16_t data[XYZ_AXIS_COUNT])
{
    data[X] = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    data[Y] = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    data[Z] = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
}

static uint8_t bmi088AccReadReg(const extDevice_t *dev, uint8_t reg)
{
    uint8_t data[2];
    spiReadRegBuf(dev, reg | BMI088_SPI_READ, data, sizeof(data));
    return data[1];
}

static bool bmi088AccReadData(const extDevice_t *dev, uint8_t data[BMI088_AXIS_DATA_LENGTH])
{
    if (spiIsBusy(dev)) {
        return false;
    }

    // The accelerometer inserts one dummy byte after the register address.
    uint8_t raw[BMI088_AXIS_DATA_LENGTH + 1];
    spiReadRegBuf(dev, BMI088_ACC_REG_DATA | BMI088_SPI_READ, raw, sizeof(raw));
    for (uint8_t i = 0; i < BMI088_AXIS_DATA_LENGTH; i++) {
        data[i] = raw[i + 1];
    }
    return true;
}

uint8_t bmi088GyroDetect(const extDevice_t *dev)
{
    return spiReadRegMsk(dev, BMI088_GYRO_REG_CHIP_ID) == BMI088_GYRO_CHIP_ID ? BMI_088_SPI : MPU_NONE;
}

static void bmi088GyroConfig(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    spiWriteReg(dev, BMI088_GYRO_REG_SOFTRESET, BMI088_SOFTRESET);
    delay(30);
    spiWriteReg(dev, BMI088_GYRO_REG_LPM1, BMI088_GYRO_PM_NORMAL);
    delay(30);
    spiWriteReg(dev, BMI088_GYRO_REG_RANGE, BMI088_GYRO_RANGE_2000DPS);
    delayMicroseconds(2);
    spiWriteReg(dev, BMI088_GYRO_REG_BANDWIDTH, bmi088GyroBandwidth(gyro->hardware_lpf));
    delayMicroseconds(2);
    spiWriteReg(dev, BMI088_GYRO_REG_INT_IO_CONF, BMI088_GYRO_INT3_PUSH_PULL_HIGH);
    delayMicroseconds(2);
    spiWriteReg(dev, BMI088_GYRO_REG_INT_IO_MAP, BMI088_GYRO_MAP_DRDY_INT3);
    delayMicroseconds(2);
    spiWriteReg(dev, BMI088_GYRO_REG_INT_CTRL, BMI088_GYRO_DRDY_ENABLE);
}

static void bmi088GyroExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    const uint32_t nowCycles = getCycleCounter();
    gyro->gyroSyncEXTI = gyro->gyroLastEXTI;
    gyro->gyroLastEXTI = nowCycles;
    gyro->detectedEXTI++;
}

static void bmi088GyroExtiInit(gyroDev_t *gyro)
{
    gyro->gyroModeSPI = GYRO_EXTI_INIT;

    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        return;
    }

    IO_t intIO = IOGetByTag(gyro->mpuIntExtiTag);
    IOInit(intIO, OWNER_GYRO_EXTI, RESOURCE_INDEX(gyro->deviceIndex));
    EXTIHandlerInit(&gyro->exti, bmi088GyroExtiHandler);
    EXTIConfig(intIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(intIO);
}

static bool bmi088GyroRead(gyroDev_t *gyro)
{
    if (gyro->gyroModeSPI == GYRO_EXTI_INIT) {
        gyro->gyroModeSPI = gyro->detectedEXTI > BMI088_GYRO_EXTI_DETECT_THRESHOLD
            ? GYRO_EXTI_INT
            : GYRO_EXTI_NO_INT;
    }

    uint8_t raw[6];
    if (!spiReadRegMskBufRB(&gyro->dev, BMI088_GYRO_REG_DATA, raw, sizeof(raw))) {
        return false;
    }
    bmi088DecodeLittleEndian(raw, gyro->gyroADCRaw);
    return true;
}

static void bmi088GyroInit(gyroDev_t *gyro)
{
    bmi088GyroConfig(gyro);
    bmi088GyroExtiInit(gyro);
    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));
}

bool bmi088SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_088_SPI) {
        return false;
    }

    gyro->initFn = bmi088GyroInit;
    gyro->readFn = bmi088GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;
    return true;
}

static bool bmi088AccDeviceInit(accDev_t *acc)
{
    gyroDev_t *gyro = acc->gyro;
    extDevice_t *dev = &acc->dev;

    if (!gyro->accCsnTag || !spiSetBusInstance(dev, gyro->spiBus)) {
        return false;
    }

    dev->busType_u.spi.csnPin = IOGetByTag(gyro->accCsnTag);
    IOInit(dev->busType_u.spi.csnPin, OWNER_ACC_CS, RESOURCE_INDEX(gyro->deviceIndex));
    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(dev->busType_u.spi.csnPin);
    spiSetClkDivisor(dev, spiCalculateDivider(BMI088_DETECT_SPI_CLK_HZ));

    // A rising CS edge followed by a dummy read switches only the accelerometer
    // die from its power-on I2C state into SPI mode.
    (void)bmi088AccReadReg(dev, BMI088_ACC_REG_CHIP_ID);
    delay(1);
    if (bmi088AccReadReg(dev, BMI088_ACC_REG_CHIP_ID) != BMI088_ACC_CHIP_ID) {
        ioPreinitByIO(dev->busType_u.spi.csnPin, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
        return false;
    }

    busDeviceRegister(dev);
    return true;
}

static bool bmi088AccRead(accDev_t *acc)
{
    uint8_t raw[BMI088_AXIS_DATA_LENGTH];
    if (!bmi088AccReadData(&acc->dev, raw)) {
        return false;
    }
    bmi088DecodeLittleEndian(raw, acc->ADCRaw);
    return true;
}

static void bmi088AccInit(accDev_t *acc)
{
    const extDevice_t *dev = &acc->dev;

    spiWriteReg(dev, BMI088_ACC_REG_SOFTRESET, BMI088_SOFTRESET);
    delay(1);
    (void)bmi088AccReadReg(dev, BMI088_ACC_REG_CHIP_ID);
    delay(1);
    spiWriteReg(dev, BMI088_ACC_REG_PWR_CONF, BMI088_ACC_PWR_ACTIVE);
    // Writes while the accelerometer is suspended require 1 ms idle time.
    delay(1);
    spiWriteReg(dev, BMI088_ACC_REG_PWR_CTRL, BMI088_ACC_PWR_ENABLE);
    delay(1);
    spiWriteReg(dev, BMI088_ACC_REG_CONF, BMI088_ACC_CONF_OSR2_1600HZ);
    delayMicroseconds(2);
    spiWriteReg(dev, BMI088_ACC_REG_RANGE, bmi088AccRange(acc->acc_high_fsr));

    acc->acc_1G = bmi088AccOneG(acc->acc_high_fsr);
    spiSetClkDivisor(dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));
}

bool bmi088SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_088_SPI || !bmi088AccDeviceInit(acc)) {
        return false;
    }

    acc->initFn = bmi088AccInit;
    acc->readFn = bmi088AccRead;
    return true;
}

#endif // USE_ACCGYRO_BMI088
