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
 
#include "platform.h"
 
#if defined(USE_ACCGYRO_ICM45686)
 
#include "common/axis.h"
#include "common/utils.h"
#include "build/debug.h"
 
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm456xx.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
 
#include "sensors/gyro.h"
#include "pg/gyrodev.h"

// reference: https://github.com/tdk-invn-oss/motion.mcu.icm45686.driver
// Datashet: https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf

#define ICM45686_REG_BANK_SEL                   0x75
#define ICM45686_BANK_0                         0x00
#define ICM45686_BANK_1                         0x01
 
// Register map Bank 0
#define ICM45686_WHO_AM_REGISTER                0x72
#define ICM45686_REG_MISC2                      0x7F
#define ICM45686_INT1_CONFIG0                   0x16
#define ICM45686_INT1_CONFIG2                   0x18
#define ICM45686_INT1_STATUS0                   0x19
#define ICM45686_INT1_STATUS1                   0x1A
#define ICM45686_GYRO_CONFIG0                   0x1C
#define ICM45686_ACCEL_CONFIG0                  0x1B
#define ICM45686_RA_SREG_CTRL                   0x67
#define ICM45686_PWR_MGMT0                      0x10

// Register map Bank 1
// LPF/HPF Config
#define ICM45686_REG_GYRO_OIS_LPF1BW_SEL        0xAB  // IPREG_SYS1_REG_171
#define ICM45686_REG_GYRO_UI_LPFBW_SEL          0xAC  // IPREG_SYS1_REG_172

// SREG_CTRL - 0x67
#define ICM45686_SREG_DATA_ENDIAN_SEL_LITTLE    (0 << 1)
#define ICM45686_SREG_DATA_ENDIAN_SEL_BIG       (1 << 1)

// MGMT0 - 0x10 - Gyro 
#define ICM45686_GYRO_MODE_OFF                  (0x00 << 2)
#define ICM45686_GYRO_MODE_STANDBY              (0x01 << 2)
#define ICM45686_GYRO_MODE_LP                   (0x02 << 2)  // Low Power Mode
#define ICM45686_GYRO_MODE_LN                   (0x03 << 2)  // Low Noise Mode

// MGMT0 - 0x10 - Accel
#define ICM45686_ACCEL_MODE_OFF                 (0x00)
#define ICM45686_ACCEL_MODE_OFF2                (0x01)
#define ICM45686_ACCEL_MODE_LP                  (0x02) // Low Power Mode
#define ICM45686_ACCEL_MODE_LN                  (0x03) // Low Noise Mode

// INT1_CONFIG0 - 0x16
#define ICM45686_INT1_STATUS_EN_RESET_DONE      (1 << 7)
#define ICM45686_INT1_STATUS_EN_AUX1_AGC_RDY    (1 << 6)
#define ICM45686_INT1_STATUS_EN_AP_AGC_RDY      (1 << 5)
#define ICM45686_INT1_STATUS_EN_AP_FSYNC        (1 << 4)
#define ICM45686_INT1_STATUS_EN_AUX1_DRDY       (1 << 3)
#define ICM45686_INT1_STATUS_EN_DRDY            (1 << 2)
#define ICM45686_INT1_STATUS_EN_FIFO_THS        (1 << 1)
#define ICM45686_INT1_STATUS_EN_FIFO_FULL       (0 << 1)

// INT1_CONFIG2 - 0x18
#define ICM45686_INT1_MODE_PULSED               (0 << 0)
#define ICM45686_INT1_MODE_LATCHED              (1 << 0)
#define ICM45686_INT1_DRIVE_CIRCUIT_OD          (0 << 1)
#define ICM45686_INT1_DRIVE_CIRCUIT_PP          (1 << 1)
#define ICM45686_INT1_POLARITY_ACTIVE_LOW       (0 << 2)
#define ICM45686_INT1_POLARITY_ACTIVE_HIGH      (1 << 2)

// INT1_STATUS0 - 0x19
#define ICM45686_INT1_STATUS_RESET_DONE         (1 << 7)
#define ICM45686_INT1_STATUS_AUX1_AGC_RDY       (1 << 6)
#define ICM45686_INT1_STATUS_AP_AGC_RDY         (1 << 5)
#define ICM45686_INT1_STATUS_AP_FSYNC           (1 << 4)
#define ICM45686_INT1_STATUS_AUX1_DRDY          (1 << 3)
#define ICM45686_INT1_STATUS_DRDY               (1 << 2)
#define ICM45686_INT1_STATUS_FIFO_THS           (1 << 1)
#define ICM45686_INT1_STATUS_FIFO_FULL          (1 << 0)

#define ICM45686_SOFT_RESET                     (1 << 1)

#define ICM45686_ACCEL_DATA_X1_UI               0x00
#define ICM45686_GYRO_DATA_X1_UI                0x06

// ACCEL_CONFIG0 - 0x1B
#define ICM45686_ACCEL_FS_SEL_32G               (0x00 << 4)
#define ICM45686_ACCEL_FS_SEL_16G               (0x01 << 4)
#define ICM45686_ACCEL_FS_SEL_8G                (0x02 << 4)
#define ICM45686_ACCEL_FS_SEL_4G                (0x03 << 4)
#define ICM45686_ACCEL_FS_SEL_2G                (0x04 << 4)

// GYRO_CONFIG0 - 0x1C
#define ICM45686_ACCEL_ODR_6K4_LN               0x03
#define ICM45686_ACCEL_ODR_3K2_LN               0x04
#define ICM45686_ACCEL_ODR_1K6_LN               0x05
#define ICM45686_ACCEL_ODR_800_LN               0x06
#define ICM45686_ACCEL_ODR_400_LP_LN            0x07
#define ICM45686_ACCEL_ODR_200_LP_LN            0x08
#define ICM45686_ACCEL_ODR_100_LP_LN            0x09
#define ICM45686_ACCEL_ODR_50_LP_LN             0x0A
#define ICM45686_ACCEL_ODR_25_LP_LN             0x0B
#define ICM45686_ACCEL_ODR_12_5_LP_LN           0x0C
#define ICM45686_ACCEL_ODR_6_25_LP              0x0D
#define ICM45686_ACCEL_ODR_3_125_LP             0x0E
#define ICM45686_ACCEL_ODR_1_5625_LP            0x0F

// GYRO_CONFIG0 - 0x1C
#define ICM45686_GYRO_FS_SEL_4000DPS            (0x00 << 4)
#define ICM45686_GYRO_FS_SEL_2000DPS            (0x01 << 4)
#define ICM45686_GYRO_FS_SEL_1000DPS            (0x02 << 4)
#define ICM45686_GYRO_FS_SEL_500DPS             (0x03 << 4)
#define ICM45686_GYRO_FS_SEL_250DPS             (0x04 << 4)
#define ICM45686_GYRO_FS_SEL_125DPS             (0x05 << 4)
#define ICM45686_GYRO_FS_SEL_62_5DPS            (0x06 << 4)
#define ICM45686_GYRO_FS_SEL_31_25DPS           (0x07 << 4)
#define ICM45686_GYRO_FS_SEL_15_625DPS          (0x08 << 4)

// GYRO_CONFIG0 - 0x1C
#define ICM45686_GYRO_ODR_6K4_LN                0x03
#define ICM45686_GYRO_ODR_3K2_LN                0x04
#define ICM45686_GYRO_ODR_1K6_LN                0x05
#define ICM45686_GYRO_ODR_800_LN                0x06
#define ICM45686_GYRO_ODR_400_LP_LN             0x07
#define ICM45686_GYRO_ODR_200_LP_LN             0x08
#define ICM45686_GYRO_ODR_100_LP_LN             0x09
#define ICM45686_GYRO_ODR_50_LP_LN              0x0A
#define ICM45686_GYRO_ODR_25_LP_LN              0x0B
#define ICM45686_GYRO_ODR_12_5_LP_LN            0x0C
#define ICM45686_GYRO_ODR_6_25_LP               0x0D
#define ICM45686_GYRO_ODR_3_125_LP              0x0E
#define ICM45686_GYRO_ODR_1_5625_LP             0x0F

// IPREG_SYS1_REG_172 0xAC UI Path LPF Bandwidth Options
#define ICM45686_GYRO_UI_LPF_BYPASS             0x00
#define ICM45686_GYRO_UI_LPF_ODR_DIV_4          0x01
#define ICM45686_GYRO_UI_LPF_ODR_DIV_8          0x02
#define ICM45686_GYRO_UI_LPF_ODR_DIV_16         0x03
#define ICM45686_GYRO_UI_LPF_ODR_DIV_32         0x04
#define ICM45686_GYRO_UI_LPF_ODR_DIV_64         0x05
#define ICM45686_GYRO_UI_LPF_ODR_DIV_128        0x06

// HPF1 (AUX1 path) Enable/Bypass
#define ICM45686_GYRO_OIS_HPF1_ENABLE           (0 << 7)  // HPF active
#define ICM45686_GYRO_OIS_HPF1_BYPASS           (1 << 7)  // HPF bypassed

// GYRO_SRC_CTRL and ACCEL_SRC_CTRL bits
#define ICM45686_SRC_CTRL_AAF_ENABLE_BIT        (1 << 0) // Anti-Alias Filter - AAF
#define ICM45686_SRC_CTRL_INTERP_ENABLE_BIT     (1 << 1) // Interpolator

// IREG addresses
#define ICM45686_GYRO_SRC_CTRL_IREG_ADDR        0xA57B
#define ICM45686_ACCEL_SRC_CTRL_IREG_ADDR       0xA57C

// HOST INDIRECT ACCESS REGISTER (IREG)
#define ICM45686_REG_IREG_ADDR_15_8             0x7C
#define ICM45686_REG_IREG_ADDR_7_0              0x7D
#define ICM45686_REG_IREG_DATA                  0x7E

// IPREG_SYS1_REG_172
#define ICM45686_GYRO_UI_LPF_CFG_IREG_ADDR       0xA4AC

// HPF bypass 0xAC PREG_SYS1_REG_172 (bit 7)
#define ICM45686_GYRO_OIS_HPF1_BYPASS            (1 << 7)
#define ICM45686_GYRO_OIS_HPF1_ENABLE            (0 << 7)

// LPF UI - 0xAC PREG_SYS1_REG_172 (bits 2:0)
#define ICM45686_GYRO_UI_LPFBW_BYPASS            0x00
#define ICM45686_GYRO_UI_LPFBW_ODR_DIV_4         0x01
#define ICM45686_GYRO_UI_LPFBW_ODR_DIV_8         0x02
#define ICM45686_GYRO_UI_LPFBW_ODR_DIV_16        0x03
#define ICM45686_GYRO_UI_LPFBW_ODR_DIV_32        0x04
#define ICM45686_GYRO_UI_LPFBW_ODR_DIV_64        0x05
#define ICM45686_GYRO_UI_LPFBW_ODR_DIV_128       0x06

#ifndef ICM456XX_CLOCK
// Default: 24 MHz max SPI frequency
#define ICM456XX_MAX_SPI_CLK_HZ                 24000000
#else
#define ICM456XX_MAX_SPI_CLK_HZ                 ICM456XX_CLOCK
#endif

#define ICM45686_BIT_IREG_DONE                  (1 << 0)

// pdf DS-000577 section 14.4 IREG WRITE
static bool icm456xx_write_ireg(const extDevice_t *dev, uint16_t reg, uint8_t value)
{
    const uint8_t msb = (reg >> 8) & 0xFF;
    const uint8_t lsb = reg & 0xFF;

    spiWriteReg(dev, ICM45686_REG_IREG_ADDR_15_8, msb);
    spiWriteReg(dev, ICM45686_REG_IREG_ADDR_7_0, lsb);
    spiWriteReg(dev, ICM45686_REG_IREG_DATA, value);

    // Check IREG_DONE (bit 0 of REG_MISC2 = 0x7F)
    for (int i = 0; i < 100; i++) {
        const uint8_t misc2 = spiReadRegMsk(dev, ICM45686_REG_MISC2);
        if (misc2 & ICM45686_BIT_IREG_DONE) {
            return true;
        }
        delayMicroseconds(10);
    }

    return false; // timeout
}

static inline void icm456xx_enableAAFandInterpolator(const extDevice_t *dev, uint16_t reg, bool enableAAF, bool enableInterp)
{
    const uint8_t value = (enableAAF ? ICM45686_SRC_CTRL_AAF_ENABLE_BIT : 0)
                        | (enableInterp ? ICM45686_SRC_CTRL_INTERP_ENABLE_BIT : 0);
    icm456xx_write_ireg(dev, reg, value);
}

static bool icm456xx_configureGyroLPF(const extDevice_t *dev, bool bypassHPF, uint8_t lpfDiv)
{
    if (lpfDiv > 0x07) {
        return false; 
    }

    uint8_t value = 0;
    value |= (bypassHPF ? ICM45686_GYRO_OIS_HPF1_BYPASS : ICM45686_GYRO_OIS_HPF1_ENABLE);
    value |= (lpfDiv & 0x07); // only bits 2:0

    return icm456xx_write_ireg(dev, ICM45686_GYRO_UI_LPF_CFG_IREG_ADDR, value);
}
 
void icm456xxAccInit(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
        case ICM_45686_SPI:
            acc->acc_1G = 2048; // 16g scale = 2048 LSB/g
            break;
        default:
            acc->acc_1G = 2048;
            break;
        }
}

void icm456xxGyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    spiSetClkDivisor(dev, spiCalculateDivider(ICM456XX_MAX_SPI_CLK_HZ));

    mpuGyroInit(gyro);
    
    spiWriteReg(dev, ICM45686_REG_BANK_SEL, ICM45686_BANK_0);
    delay(1);

    spiWriteReg(dev, ICM45686_PWR_MGMT0, ICM45686_GYRO_MODE_LN | ICM45686_ACCEL_MODE_LN);

    // Enable Anti-Alias (AAF) Filter and Interpolator for both Gyro and Accel
    icm456xx_enableAAFandInterpolator(dev, ICM45686_GYRO_SRC_CTRL_IREG_ADDR, true, true);
    icm456xx_enableAAFandInterpolator(dev, ICM45686_ACCEL_SRC_CTRL_IREG_ADDR, true, true);

    // Set the Gyro UI HPF & LPF bandwidth cut-off
    icm456xx_configureGyroLPF(dev, true, ICM45686_GYRO_UI_LPFBW_ODR_DIV_64); // TODO: check cut-off bandwidth of this divider

#if 0 // TODO: check & test this
    spiWriteReg(dev, ICM45686_REG_BANK_SEL, ICM45686_BANK_1);
    delay(1);

    // Set the Gyro UI LPF bandwidth (AUX1 path)
    spiWriteReg(dev, ICM45686_REG_GYRO_UI_LPFBW_SEL, ICM45686_GYRO_UI_LPF_ODR_DIV_8 | ICM45686_GYRO_OIS_HPF1_BYPASS);

    spiWriteReg(dev, ICM45686_REG_BANK_SEL, ICM45686_BANK_0);
#endif

    spiWriteReg(dev, ICM45686_RA_SREG_CTRL, ICM45686_SREG_DATA_ENDIAN_SEL_BIG);

    spiWriteReg(dev, ICM45686_GYRO_CONFIG0, ICM45686_GYRO_FS_SEL_4000DPS | ICM45686_GYRO_ODR_6K4_LN);
    spiWriteReg(dev, ICM45686_ACCEL_CONFIG0, ICM45686_ACCEL_FS_SEL_16G | ICM45686_ACCEL_ODR_1K6_LN);

    spiWriteReg(dev, ICM45686_INT1_CONFIG2, ICM45686_INT1_MODE_PULSED | ICM45686_INT1_DRIVE_CIRCUIT_PP |
                                            ICM45686_INT1_POLARITY_ACTIVE_HIGH);

    spiWriteReg(dev, ICM45686_INT1_CONFIG0, ICM45686_INT1_STATUS_EN_DRDY);

    delay(1);
    
}
 
uint8_t icm456xxSpiDetect(const extDevice_t *dev)
{
    uint8_t icmDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;

    spiWriteReg(dev, ICM45686_REG_MISC2, ICM45686_SOFT_RESET);
    delay(100);

    spiWriteReg(dev, ICM45686_REG_BANK_SEL, ICM45686_BANK_0);

    do {
        delay(150);
        const uint8_t whoAmI = spiReadRegMsk(dev, ICM45686_WHO_AM_REGISTER);
        switch (whoAmI) {
            case ICM45686_WHO_AM_I_CONST:
                icmDetected = ICM_45686_SPI;
                break;
            default:
                icmDetected = MPU_NONE;
                break;
        }
        if (icmDetected != MPU_NONE) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);

    return icmDetected;

}

bool icm456xxAccReadSPI(accDev_t *acc)
{
    uint8_t raw[6] = {0};

    const bool ack = spiReadRegMskBufRB(&acc->gyro->dev, ICM45686_ACCEL_DATA_X1_UI, raw, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((raw[1] << 8) | raw[0]);
    acc->ADCRaw[Y] = (int16_t)((raw[3] << 8) | raw[2]);
    acc->ADCRaw[Z] = (int16_t)((raw[5] << 8) | raw[4]);
    return true;
}
 
bool icm456xxSpiAccDetect(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
        case ICM_45686_SPI:
            acc->initFn = icm456xxAccInit;
            acc->readFn = icm456xxAccReadSPI;
            break;
        default:
            return false;
        }

        return true;
}

bool icm456xxGyroReadSPI(gyroDev_t *gyro)
{
    uint8_t raw[6] = {0};

    const bool ack = spiReadRegMskBufRB(&gyro->dev, ICM45686_GYRO_DATA_X1_UI, raw, 6);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((raw[1] << 8) | raw[0]);
    gyro->gyroADCRaw[Y] = (int16_t)((raw[3] << 8) | raw[2]);
    gyro->gyroADCRaw[Z] = (int16_t)((raw[5] << 8) | raw[4]);
    return true;
}


bool icm456xxSpiGyroDetect(gyroDev_t *gyro)
{
    switch (gyro->mpuDetectionResult.sensor) {
        case ICM_45686_SPI:
            gyro->scale = GYRO_SCALE_4000DPS;
            gyro->initFn = icm456xxGyroInit;
            gyro->readFn = icm456xxGyroReadSPI;
            break;
        default:
            return false;
        }
    
        return true;
}
 
#endif // USE_ACCGYRO_ICM45686
