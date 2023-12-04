/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if defined(USE_ACCGYRO_LSM6DSV16X)

#include "drivers/accgyro/accgyro_spi_lsm6dsv16x.h"

// 10 MHz max SPI frequency
#define LSM6DSV16X_MAX_SPI_CLK_HZ 10000000

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

uint8_t lsm6dsv16xSpiDetect(const extDevice_t *dev)
{
    const uint8_t whoAmI = spiReadRegMsk(dev, LSM6DSV_WHO_AM_I);

    if (whoAmI != LSM6DSV16X_WHO_AM_I_CONST) {
        return MPU_NONE;
    }

    return LSM6DSV16X_SPI;
}

void lsm6dsv16xAccInit(accDev_t *acc)
{
    const extDevice_t *dev = &acc->gyro->dev;

    // Enable the accelerometer in high accuracy mode at 1kHz
    spiWriteReg(dev, LSM6DSV_CTRL1,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL1_OP_MODE_XL_HIGH_ACCURACY,
                                    LSM6DSV_CTRL1_OP_MODE_XL_MASK,
                                    LSM6DSV_CTRL1_OP_MODE_XL_SHIFT) |
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL1_ODR_XL_1000HZ,
                                    LSM6DSV_CTRL1_ODR_XL_MASK,
                                    LSM6DSV_CTRL1_ODR_XL_SHIFT));

    // Enable 16G sensitivity
    spiWriteReg(dev, LSM6DSV_CTRL8,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL8_FS_XL_16G,
                                    LSM6DSV_CTRL8_FS_XL_MASK,
                                    LSM6DSV_CTRL8_FS_XL_SHIFT));

    // ±16G mode
    acc->acc_1G = 512 * 4;
}

static bool lsm6dsv16xAccReadSPI(accDev_t *acc)
{
    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        acc->gyro->dev.txBuf[0] = LSM6DSV_OUTX_L_A | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = acc->gyro->dev.txBuf;
        segments[0].u.buffers.rxData = &acc->gyro->dev.rxBuf[1];

        spiSequence(&acc->gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&acc->gyro->dev);

        int16_t *accData = (int16_t *)acc->gyro->dev.rxBuf;

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
        int16_t *accData = (int16_t *)acc->gyro->dev.rxBuf;

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

bool lsm6dsv16xSpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != LSM6DSV16X_SPI) {
        return false;
    }

    acc->initFn = lsm6dsv16xAccInit;
    acc->readFn = lsm6dsv16xAccReadSPI;

    return true;
}

void lsm6dsv16xGyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    spiSetClkDivisor(dev, spiCalculateDivider(LSM6DSV16X_MAX_SPI_CLK_HZ));

    // Perform a software reset
    spiWriteReg(dev, LSM6DSV_CTRL3, LSM6DSV_CTRL3_SW_RESET);

    // Select high-accuracy ODR mode 1 before leaving power-off mode
    spiWriteReg(dev, LSM6DSV_HAODR_CFG,
                LSM6DSV_ENCODE_BITS(LSM6DSV_HAODR_MODE1,
                                    LSM6DSV_HAODR_CFG_HAODR_SEL_MASK,
                                    LSM6DSV_HAODR_CFG_HAODR_SEL_SHIFT));

    // Enable the gyro in high accuracy mode at 8kHz
    spiWriteReg(dev, LSM6DSV_CTRL2,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL2_OP_MODE_G_HIGH_ACCURACY,
                                    LSM6DSV_CTRL2_OP_MODE_G_MASK,
                                    LSM6DSV_CTRL2_OP_MODE_G_SHIFT) |
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL2_ODR_G_8000HZ,
                                    LSM6DSV_CTRL2_ODR_G_MASK,
                                    LSM6DSV_CTRL2_ODR_G_SHIFT));

    // Enable 2000 deg/s sensitivity
    spiWriteReg(dev, LSM6DSV_CTRL6,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL6_FS_G_BW_407HZ,
                                    LSM6DSV_CTRL6_LPF1_G_BW_MASK,
                                    LSM6DSV_CTRL6_LPF1_G_BW_SHIFT) |
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL6_FS_G_2000DPS,
                                    LSM6DSV_CTRL6_FS_G_MASK,
                                    LSM6DSV_CTRL6_FS_G_SHIFT));

    // Autoincrement register address when doing block SPI reads and update continuously
    spiWriteReg(dev, LSM6DSV_CTRL3, LSM6DSV_CTRL3_IF_INC);

    // Generate pulse on interrupt line, not requiring a read to clear
    // TODO this pulse lasts 66us followed by a low of 66us, so we get 132us cycle time, not 125us
    spiWriteReg(dev, LSM6DSV_CTRL4, LSM6DSV_CTRL4_DRDY_PULSED);

    // From section 4.1, Mechanical characteristics, of the datasheet, G_So is 70mdps/LSB for FS = ±2000 dps.
    gyro->scale = 0.070f;

    // Enable the INT1 output to interrupt when new gyro data is ready
    spiWriteReg(dev, LSM6DSV_INT1_CTRL, LSM6DSV_INT1_CTRL_INT1_DRDY_G);

    mpuGyroInit(gyro);
}

bool lsm6dsv16xGyroReadSPI(gyroDev_t *gyro)
{
    int16_t *gyroData = (int16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0xff
        memset(gyro->dev.txBuf, 0xff, 16);

        // Check that minimum number of interrupts have been detected

        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uint32_t)gyro;
                gyro->dev.txBuf[0] = LSM6DSV_OUTX_L_G | 0x80;
                // Read three words of gyro data immediately followed by three bytes of acc data
                gyro->segments[0].len = sizeof(uint8_t) + 6 * sizeof(int16_t);
                gyro->segments[0].callback = mpuIntcallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                gyro->segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];
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
        gyro->dev.txBuf[0] = gyro->gyroDataReg | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = gyro->dev.txBuf;
        segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];

        spiSequence(&gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&gyro->dev);

        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];
        break;
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

bool lsm6dsv16xSpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != LSM6DSV16X_SPI) {
        return false;
    }

    gyro->initFn = lsm6dsv16xGyroInit;
    gyro->readFn = lsm6dsv16xGyroReadSPI;

    return true;
}
#endif // USE_ACCGYRO_LSM6DSV16X
