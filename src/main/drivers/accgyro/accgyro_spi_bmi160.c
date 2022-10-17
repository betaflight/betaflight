/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_BMI160 BMI160 Functions
 * @brief Hardware functions to deal with the 6DOF gyro / accel sensor
 * @{
 *
 * @file       pios_bmi160.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2016
 * @brief      BMI160 Gyro / Accel Sensor Routines
 * @see        The GNU Public License (GPL) Version 3
 ******************************************************************************/

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_BMI160

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

// 10 MHz max SPI frequency
#define BMI160_MAX_SPI_CLK_HZ 10000000

/* BMI160 Registers */
#define BMI160_REG_CHIPID 0x00
#define BMI160_REG_PMU_STAT 0x03
#define BMI160_REG_GYR_DATA_X_LSB 0x0C
#define BMI160_REG_ACC_DATA_X_LSB 0x12
#define BMI160_REG_STATUS 0x1B
#define BMI160_REG_TEMPERATURE_0 0x20
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_ACC_RANGE 0x41
#define BMI160_REG_GYR_CONF 0x42
#define BMI160_REG_GYR_RANGE 0x43
#define BMI160_REG_INT_EN1 0x51
#define BMI160_REG_INT_OUT_CTRL 0x53
#define BMI160_REG_INT_MAP1 0x56
#define BMI160_REG_FOC_CONF 0x69
#define BMI160_REG_CONF 0x6A
#define BMI160_REG_OFFSET_0 0x77
#define BMI160_REG_CMD 0x7E

/* Register values */
#define BMI160_PMU_CMD_PMU_ACC_NORMAL 0x11
#define BMI160_PMU_CMD_PMU_GYR_NORMAL 0x15
#define BMI160_INT_EN1_DRDY 0x10
#define BMI160_INT_OUT_CTRL_INT1_CONFIG 0x0A
#define BMI160_REG_INT_MAP1_INT1_DRDY 0x80
#define BMI160_CMD_START_FOC 0x03
#define BMI160_CMD_PROG_NVM 0xA0
#define BMI160_REG_STATUS_NVM_RDY 0x10
#define BMI160_REG_STATUS_FOC_RDY 0x08
#define BMI160_REG_CONF_NVM_PROG_EN 0x02
#define BMI160_VAL_GYRO_CONF_BWP_OSR4 0x00
#define BMI160_VAL_GYRO_CONF_BWP_OSR2 0x10
#define BMI160_VAL_GYRO_CONF_BWP_NORM 0x20
#define BMI160_VAL_ACC_CONF_BWP_OSR4 0x00
#define BMI160_VAL_ACC_CONF_BWP_OSR2 0x10
#define BMI160_VAL_ACC_CONF_BWP_NORM 0x20
#define BMI160_VAL_ACC_CONF_US_HP 0x00

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// Global Variables
static volatile bool BMI160InitDone = false;
static volatile bool BMI160Detected = false;

//! Private functions
static int32_t BMI160_Config(const extDevice_t *dev);
static int32_t BMI160_do_foc(const extDevice_t *dev);

uint8_t bmi160Detect(const extDevice_t *dev)
{
    if (BMI160Detected) {
        return BMI_160_SPI;
    }

    // Toggle CS to activate SPI (see https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf section 3.2.1)
    spiWrite(dev, 0xFF);

    delay(100); // Give SPI some time to start up

    // Check the chip ID
    if (spiReadRegMsk(dev, BMI160_REG_CHIPID) != 0xd1) {
        return MPU_NONE;
    }

    BMI160Detected = true;

    return BMI_160_SPI;
}

/**
 * @brief Initialize the BMI160 6-axis sensor.
 * @return 0 for success, -1 for failure to allocate, -10 for failure to get irq
 */
static void BMI160_Init(const extDevice_t *dev)
{
    if (BMI160InitDone || !BMI160Detected) {
        return;
    }

    /* Configure the BMI160 Sensor */
    if (BMI160_Config(dev) != 0) {
        return;
    }

    bool do_foc = false;

    /* Perform fast offset compensation if requested */
    if (do_foc) {
        BMI160_do_foc(dev);
    }

    BMI160InitDone = true;
}

static uint8_t getBmiOsrMode(void)
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return BMI160_VAL_GYRO_CONF_BWP_OSR4;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return BMI160_VAL_GYRO_CONF_BWP_OSR2;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return BMI160_VAL_GYRO_CONF_BWP_NORM;
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return BMI160_VAL_GYRO_CONF_BWP_NORM;
#endif
        default:
            return BMI160_VAL_GYRO_CONF_BWP_OSR4;
    }
}

/**
 * @brief Configure the sensor
 */
static int32_t BMI160_Config(const extDevice_t *dev)
{
    // Set normal power mode for gyro and accelerometer
    spiWriteReg(dev, BMI160_REG_CMD, BMI160_PMU_CMD_PMU_GYR_NORMAL);
    delay(100); // can take up to 80ms

    spiWriteReg(dev, BMI160_REG_CMD, BMI160_PMU_CMD_PMU_ACC_NORMAL);
    delay(5); // can take up to 3.8ms

    // Verify that normal power mode was entered
    uint8_t pmu_status = spiReadRegMsk(dev, BMI160_REG_PMU_STAT);
    if ((pmu_status & 0x3C) != 0x14) {
        return -3;
    }

    // Set odr and ranges
    // Set acc_us = 0 & acc_bwp = 0b001 for high performance and OSR2 mode
    spiWriteReg(dev, BMI160_REG_ACC_CONF, BMI160_VAL_ACC_CONF_US_HP | BMI160_VAL_ACC_CONF_BWP_OSR2 | BMI160_ODR_800_Hz);
    delay(1);

    spiWriteReg(dev, BMI160_REG_GYR_CONF, getBmiOsrMode() | BMI160_ODR_3200_Hz);
    delay(1);

    spiWriteReg(dev, BMI160_REG_ACC_RANGE, BMI160_RANGE_8G);
    delay(1);

    spiWriteReg(dev, BMI160_REG_GYR_RANGE, BMI160_RANGE_2000DPS);
    delay(1);

    // Enable offset compensation
    uint8_t val = spiReadRegMsk(dev, BMI160_REG_OFFSET_0);
    spiWriteReg(dev, BMI160_REG_OFFSET_0, val | 0xC0);

    // Enable data ready interrupt
    spiWriteReg(dev, BMI160_REG_INT_EN1, BMI160_INT_EN1_DRDY);
    delay(1);

    // Enable INT1 pin
    spiWriteReg(dev, BMI160_REG_INT_OUT_CTRL, BMI160_INT_OUT_CTRL_INT1_CONFIG);
    delay(1);

    // Map data ready interrupt to INT1 pin
    spiWriteReg(dev, BMI160_REG_INT_MAP1, BMI160_REG_INT_MAP1_INT1_DRDY);
    delay(1);

    return 0;
}

static int32_t BMI160_do_foc(const extDevice_t *dev)
{
    // assume sensor is mounted on top
    uint8_t val = 0x7D;
    spiWriteReg(dev, BMI160_REG_FOC_CONF, val);

    // Start FOC
    spiWriteReg(dev, BMI160_REG_CMD, BMI160_CMD_START_FOC);

    // Wait for FOC to complete
    for (int i=0; i<50; i++) {
        val = spiReadRegMsk(dev, BMI160_REG_STATUS);
        if (val & BMI160_REG_STATUS_FOC_RDY) {
            break;
        }
        delay(10);
    }
    if (!(val & BMI160_REG_STATUS_FOC_RDY)) {
        return -3;
    }

    // Program NVM
    val = spiReadRegMsk(dev, BMI160_REG_CONF);
    spiWriteReg(dev, BMI160_REG_CONF, val | BMI160_REG_CONF_NVM_PROG_EN);

    spiWriteReg(dev, BMI160_REG_CMD, BMI160_CMD_PROG_NVM);

    // Wait for NVM programming to complete
    for (int i=0; i<50; i++) {
        val = spiReadRegMsk(dev, BMI160_REG_STATUS);
        if (val & BMI160_REG_STATUS_NVM_RDY) {
            break;
        }
        delay(10);
    }
    if (!(val & BMI160_REG_STATUS_NVM_RDY)) {
        return -6;
    }

    return 0;
}

extiCallbackRec_t bmi160IntCallbackRec;

// Called in ISR context
// Gyro read has just completed
busStatus_e bmi160Intcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

void bmi160ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    extDevice_t *dev = &gyro->dev;

    // Ideally we'd use a timer to capture such information, but unfortunately the port used for EXTI interrupt does
    // not have an associated timer
    uint32_t nowCycles = getCycleCounter();
    gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    gyro->gyroLastEXTI = nowCycles;

    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        spiSequence(dev, gyro->segments);
    }

    gyro->detectedEXTI++;

}

static void bmi160IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi160ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}

static bool bmi160AccRead(accDev_t *acc)
{
    extDevice_t *dev = &acc->gyro->dev;

    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        dev->txBuf[0] = BMI160_REG_ACC_DATA_X_LSB | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = &dev->txBuf[1];
        segments[0].u.buffers.rxData = &dev->rxBuf[1];

        spiSequence(&acc->gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&acc->gyro->dev);

        int16_t *accData = (int16_t *)dev->rxBuf;
        acc->ADCRaw[X] = accData[1];
        acc->ADCRaw[Y] = accData[2];
        acc->ADCRaw[Z] = accData[3];
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.

        // This data was read from the gyro, which is the same SPI device as the acc
        int16_t *accData = (int16_t *)dev->rxBuf;
        acc->ADCRaw[X] = accData[4];
        acc->ADCRaw[Y] = accData[5];
        acc->ADCRaw[Z] = accData[6];
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}


static bool bmi160GyroRead(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    int16_t *gyroData = (int16_t *)dev->rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0x00
        memset(dev->txBuf, 0x00, 14);

        // Check that minimum number of interrupts have been detected

        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        // Using DMA for gyro access upsets the scheduler on the F4
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(dev)) {
                dev->callbackArg = (uint32_t)gyro;
                dev->txBuf[1] = BMI160_REG_GYR_DATA_X_LSB | 0x80;
                gyro->segments[0].len = 13;
                gyro->segments[0].callback = bmi160Intcallback;
                gyro->segments[0].u.buffers.txData = &dev->txBuf[1];
                gyro->segments[0].u.buffers.rxData = &dev->rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else {
                // Interrupts are present, but no DMA
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        dev->txBuf[1] = BMI160_REG_GYR_DATA_X_LSB | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = &dev->txBuf[1];
        segments[0].u.buffers.rxData = &dev->rxBuf[1];

        spiSequence(dev, &segments[0]);

        // Wait for completion
        spiWait(dev);

        // Fall through
        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.
        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];
        break;
    }

    default:
        break;
    }

    return true;
}


void bmi160SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    BMI160_Init(dev);
    bmi160IntExtiInit(gyro);

    spiSetClkDivisor(dev, spiCalculateDivider(BMI160_MAX_SPI_CLK_HZ));
}

void bmi160SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 8;
}


bool bmi160SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_160_SPI) {
        return false;
    }

    acc->initFn = bmi160SpiAccInit;
    acc->readFn = bmi160AccRead;

    return true;
}


bool bmi160SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_160_SPI) {
        return false;
    }

    gyro->initFn = bmi160SpiGyroInit;
    gyro->readFn = bmi160GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
#endif // USE_ACCGYRO_BMI160
