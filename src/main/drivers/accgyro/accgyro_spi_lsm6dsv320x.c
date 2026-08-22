/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_LSM6DSV320X

#include "drivers/accgyro/accgyro_spi_lsm6dsv320x.h"
#include "drivers/time.h"
#include "sensors/gyro.h"

// LSM6DSV320X datasheet DS14623, revision 1.
#define LSM6DSV320X_MAX_SPI_CLK_HZ          10000000
#define LSM6DSV320X_RESET_TIMEOUT_MS        100
#define LSM6DSV320X_GYRO_SCALE_2000DPS      0.070f
#define LSM6DSV320X_GYRO_EXTI_THRESHOLD     1000

#define LSM6DSV320X_IF_CFG                  0x03
#define LSM6DSV320X_IF_CFG_I2C_I3C_DISABLE  0x01
#define LSM6DSV320X_INT1_CTRL               0x0D
#define LSM6DSV320X_INT1_DRDY_G             0x02
#define LSM6DSV320X_WHO_AM_I                0x0F
#define LSM6DSV320X_CTRL1                   0x10
#define LSM6DSV320X_CTRL2                   0x11
#define LSM6DSV320X_CTRL3                   0x12
#define LSM6DSV320X_CTRL3_BDU               0x40
#define LSM6DSV320X_CTRL3_IF_INC            0x04
#define LSM6DSV320X_CTRL3_SW_RESET          0x01
#define LSM6DSV320X_CTRL4                   0x13
#define LSM6DSV320X_CTRL4_DRDY_PULSED       0x02
#define LSM6DSV320X_CTRL6                   0x15
#define LSM6DSV320X_CTRL6_LPF1_G_BW_SHIFT   4
#define LSM6DSV320X_CTRL6_RESERVED_3        0x08
#define LSM6DSV320X_CTRL6_FS_G_2000DPS      0x04
#define LSM6DSV320X_CTRL7                   0x16
#define LSM6DSV320X_CTRL7_LPF1_G_EN         0x01
#define LSM6DSV320X_CTRL8                   0x17
#define LSM6DSV320X_CTRL8_FS_XL_16G         0x03
#define LSM6DSV320X_CTRL9                   0x18
#define LSM6DSV320X_CTRL9_LPF2_XL_EN        0x08
#define LSM6DSV320X_OUTX_L_G                0x22
#define LSM6DSV320X_OUTX_L_A                0x28
#define LSM6DSV320X_CTRL1_XL_HG             0x4E
#define LSM6DSV320X_HAODR_CFG               0x62

#define LSM6DSV320X_OP_MODE_HIGH_ACCURACY   0x10
#define LSM6DSV320X_ODR_HA01_1000HZ         0x09
#define LSM6DSV320X_ODR_HA01_8000HZ         0x0C
#define LSM6DSV320X_HAODR_MODE1             0x01

typedef enum {
    LSM6DSV320X_LPF_281HZ = 0,
    LSM6DSV320X_LPF_213HZ,
    LSM6DSV320X_LPF_156HZ,
    LSM6DSV320X_LPF_407HZ,
} lsm6dsv320xLpf_e;

uint8_t lsm6dsv320xSpiDetect(const extDevice_t *dev)
{
    return spiReadRegMsk(dev, LSM6DSV320X_WHO_AM_I) == LSM6DSV320X_WHO_AM_I_CONST
        ? LSM6DSV320X_SPI
        : MPU_NONE;
}

static void lsm6dsv320xAccInit(accDev_t *acc)
{
    // The flight-control accelerometer uses the low-g UI channel at +/-16 g.
    acc->acc_1G = 512 * 4;
}

static bool lsm6dsv320xAccReadSPI(accDev_t *acc)
{
    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        acc->gyro->dev.txBuf[0] = LSM6DSV320X_OUTX_L_A | 0x80;

        busSegment_t segments[] = {
            { .u.buffers = { NULL, NULL }, 7, true, NULL },
            { .u.link = { NULL, NULL }, 0, true, NULL },
        };
        segments[0].u.buffers.txData = acc->gyro->dev.txBuf;
        segments[0].u.buffers.rxData = &acc->gyro->dev.rxBuf[1];

        spiSequence(&acc->gyro->dev, segments);
        spiWait(&acc->gyro->dev);

        const int16_t *accData = (const int16_t *)acc->gyro->dev.rxBuf;
        acc->ADCRaw[X] = accData[1];
        acc->ADCRaw[Y] = accData[2];
        acc->ADCRaw[Z] = accData[3];
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        const int16_t *accData = (const int16_t *)acc->gyro->dev.rxBuf;
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

bool lsm6dsv320xSpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != LSM6DSV320X_SPI) {
        return false;
    }

    acc->initFn = lsm6dsv320xAccInit;
    acc->readFn = lsm6dsv320xAccReadSPI;
    return true;
}

static void lsm6dsv320xGyroInit(gyroDev_t *gyro)
{
    static const uint8_t lpfBandwidth[GYRO_HARDWARE_LPF_COUNT] = {
        [GYRO_HARDWARE_LPF_NORMAL] = LSM6DSV320X_LPF_281HZ,
        [GYRO_HARDWARE_LPF_OPTION_1] = LSM6DSV320X_LPF_156HZ,
        [GYRO_HARDWARE_LPF_OPTION_2] = LSM6DSV320X_LPF_213HZ,
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        [GYRO_HARDWARE_LPF_EXPERIMENTAL] = LSM6DSV320X_LPF_407HZ,
#endif
    };
    const extDevice_t *dev = &gyro->dev;

    spiSetClkDivisor(dev, spiCalculateDivider(LSM6DSV320X_MAX_SPI_CLK_HZ));

    // ST's reset sequence powers down all three sensors first. This also makes
    // initialization safe after an MCU-only reset where the IMU remained on.
    spiWriteReg(dev, LSM6DSV320X_CTRL1, 0);
    spiWriteReg(dev, LSM6DSV320X_CTRL2, 0);
    spiWriteReg(dev, LSM6DSV320X_CTRL1_XL_HG, 0);
    spiWriteReg(dev, LSM6DSV320X_CTRL3, LSM6DSV320X_CTRL3_SW_RESET);
    for (unsigned timeout = 0; timeout < LSM6DSV320X_RESET_TIMEOUT_MS; timeout++) {
        delay(1);
        if (!(spiReadRegMsk(dev, LSM6DSV320X_CTRL3) & LSM6DSV320X_CTRL3_SW_RESET)) {
            break;
        }
    }

    // Use only the primary 4-wire SPI interface and make multi-byte samples atomic.
    spiWriteReg(dev, LSM6DSV320X_IF_CFG, LSM6DSV320X_IF_CFG_I2C_I3C_DISABLE);
    spiWriteReg(dev, LSM6DSV320X_CTRL3, LSM6DSV320X_CTRL3_BDU | LSM6DSV320X_CTRL3_IF_INC);

    // Select exact 1 kHz / 8 kHz rates before enabling high-accuracy mode on both sensors.
    spiWriteReg(dev, LSM6DSV320X_HAODR_CFG, LSM6DSV320X_HAODR_MODE1);
    // HP_LPF2_XL_BW=000 selects LPF2 at ODR/4 (250 Hz for the 1 kHz accelerometer).
    spiWriteReg(dev, LSM6DSV320X_CTRL8, LSM6DSV320X_CTRL8_FS_XL_16G);
    spiWriteReg(dev, LSM6DSV320X_CTRL6,
        (lpfBandwidth[gyroConfig()->gyro_hardware_lpf] << LSM6DSV320X_CTRL6_LPF1_G_BW_SHIFT)
        | LSM6DSV320X_CTRL6_RESERVED_3
        | LSM6DSV320X_CTRL6_FS_G_2000DPS);
    spiWriteReg(dev, LSM6DSV320X_CTRL7, LSM6DSV320X_CTRL7_LPF1_G_EN);
    spiWriteReg(dev, LSM6DSV320X_CTRL9, LSM6DSV320X_CTRL9_LPF2_XL_EN);

    spiWriteReg(dev, LSM6DSV320X_CTRL4, LSM6DSV320X_CTRL4_DRDY_PULSED);
    spiWriteReg(dev, LSM6DSV320X_INT1_CTRL, LSM6DSV320X_INT1_DRDY_G);

    spiWriteReg(dev, LSM6DSV320X_CTRL1,
        LSM6DSV320X_OP_MODE_HIGH_ACCURACY | LSM6DSV320X_ODR_HA01_1000HZ);
    spiWriteReg(dev, LSM6DSV320X_CTRL2,
        LSM6DSV320X_OP_MODE_HIGH_ACCURACY | LSM6DSV320X_ODR_HA01_8000HZ);

    gyro->scale = LSM6DSV320X_GYRO_SCALE_2000DPS;
    mpuGyroInit(gyro);
}

static bool lsm6dsv320xGyroReadSPI(gyroDev_t *gyro)
{
    const int16_t *gyroData = (const int16_t *)gyro->dev.rxBuf;

    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
        memset(gyro->dev.txBuf, 0xff, 16);
        gyro->gyroDmaMaxDuration = 5;

        if (gyro->detectedEXTI > LSM6DSV320X_GYRO_EXTI_THRESHOLD) {
#ifdef USE_DMA
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uintptr_t)gyro;
                gyro->dev.txBuf[0] = LSM6DSV320X_OUTX_L_G | 0x80;
                gyro->segments[0].len = sizeof(uint8_t) + 6 * sizeof(int16_t);
                gyro->segments[0].callback = mpuIntCallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                gyro->segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else
#endif
            {
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        gyro->dev.txBuf[0] = LSM6DSV320X_OUTX_L_G | 0x80;

        busSegment_t segments[] = {
            { .u.buffers = { NULL, NULL }, 7, true, NULL },
            { .u.link = { NULL, NULL }, 0, true, NULL },
        };
        segments[0].u.buffers.txData = gyro->dev.txBuf;
        segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];

        spiSequence(&gyro->dev, segments);
        spiWait(&gyro->dev);

        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];
        break;
    }

    case GYRO_EXTI_INT_DMA:
        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];
        break;

    default:
        break;
    }

    return true;
}

bool lsm6dsv320xSpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != LSM6DSV320X_SPI) {
        return false;
    }

    gyro->initFn = lsm6dsv320xGyroInit;
    gyro->readFn = lsm6dsv320xGyroReadSPI;
    return true;
}

#endif // USE_ACCGYRO_LSM6DSV320X
