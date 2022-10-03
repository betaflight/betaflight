/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_GYRO_SPI_ICM42605) || defined(USE_GYRO_SPI_ICM42688P)

#include "common/axis.h"
#include "common/maths.h"
#include "build/debug.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm426xx.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

// 24 MHz max SPI frequency
#define ICM426XX_MAX_SPI_CLK_HZ 24000000

#define ICM426XX_RA_PWR_MGMT0                       0x4E
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05

#define ICM426XX_AAF_258HZ_DELT                     6
#define ICM426XX_AAF_258HZ_BITSHIFT                 10
#define ICM426XX_AAF_536HZ_DELT                     12
#define ICM426XX_AAF_536HZ_BITSHIFT                 8
#define ICM426XX_AAF_997HZ_DELT                     21
#define ICM426XX_AAF_997HZ_BITSHIFT                 6
#define ICM426XX_AAF_1962HZ_DELT                    37
#define ICM426XX_AAF_1962HZ_BITSHIFT                4

#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (14 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (14 << 0)

#define ICM426XX_RA_GYRO_DATA_X1                    0x25
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F

#define ICM426XX_RA_INT_CONFIG                      0x14
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)


#define ICM426XX_RA_INT_SOURCE0                     0x65
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

uint8_t icm426xxSpiDetect(const extDevice_t *dev)
{
    spiWriteReg(dev, ICM426XX_RA_PWR_MGMT0, 0x00);

    uint8_t icmDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;
    do {
        delay(150);
        const uint8_t whoAmI = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);
        switch (whoAmI) {
        case ICM42605_WHO_AM_I_CONST:
            icmDetected = ICM_42605_SPI;
            break;
        case ICM42688P_WHO_AM_I_CONST:
            icmDetected = ICM_42688P_SPI;
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

void icm426xxAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool icm426xxSpiAccDetect(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
    case ICM_42605_SPI:
        break;
    case ICM_42688P_SPI:
        break;
    default:
        return false;
    }

    acc->initFn = icm426xxAccInit;
    acc->readFn = mpuAccReadSPI;

    return true;
}

typedef struct odrConfig_s {
    uint8_t odr;            // ODR: Output data rate (gyro sample rate in kHz)
    uint8_t odr_idx;        // See GYRO_ODR in datasheet
} odrConfig_t;

static odrConfig_t odrConfigLUT[] = {
    { 8, 3 },
    { 4, 4 },
    { 2, 5 },
    { 1, 6 },
};

void icm426xxGyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    spiSetClkDivisor(dev, spiCalculateDivider(ICM426XX_MAX_SPI_CLK_HZ));

    mpuGyroInit(gyro);
    gyro->accDataReg = ICM426XX_RA_ACCEL_DATA_X1;
    gyro->gyroDataReg = ICM426XX_RA_GYRO_DATA_X1;

    spiWriteReg(dev, ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN);
    delay(15);

    uint8_t odrIdx = 0;
    bool supportedODRFound = false;

    if (gyro->gyroRateKHz) {
        const uint8_t gyroSyncDenominator = gyro->mpuDividerDrops + 1; // rebuild it in here, see gyro_sync.c
        const uint8_t desiredODR = 8 / gyroSyncDenominator;
        for (unsigned i = 0; i < ARRAYLEN(odrConfigLUT); i++) {
            if (odrConfigLUT[i].odr == desiredODR) {
                odrIdx = odrConfigLUT[i].odr_idx;
                supportedODRFound = true;
                break;
            }
        }
    }

    if (!supportedODRFound) {
        odrIdx = 6;
        gyro->gyroRateKHz = GYRO_RATE_1_kHz;
    }

    STATIC_ASSERT(INV_FSR_2000DPS == 3, "INV_FSR_2000DPS must be 3 to generate correct value");
    spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG0, (3 - INV_FSR_2000DPS) << 5 | (odrIdx & 0x0F));
    delay(15);

    STATIC_ASSERT(INV_FSR_16G == 3, "INV_FSR_16G must be 3 to generate correct value");
    spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG0, (3 - INV_FSR_16G) << 5 | (odrIdx & 0x0F));
    delay(15);

    // Select desired AAF settings
    uint8_t aafDelt = 0;
    uint8_t aafBitshift = 0;
    switch (gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:  // 258 Hz
            aafDelt = ICM426XX_AAF_258HZ_DELT;
            aafBitshift = ICM426XX_AAF_258HZ_BITSHIFT;
            break;
        case GYRO_HARDWARE_LPF_OPTION_1:  // 536 Hz
            aafDelt = ICM426XX_AAF_536HZ_DELT;
            aafBitshift = ICM426XX_AAF_536HZ_BITSHIFT;
            break;
        case GYRO_HARDWARE_LPF_OPTION_2:  // 997 Hz
            aafDelt = ICM426XX_AAF_997HZ_DELT;
            aafBitshift = ICM426XX_AAF_997HZ_BITSHIFT;
            break;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:  // 1962 Hz
            aafDelt = ICM426XX_AAF_1962HZ_DELT;
            aafBitshift = ICM426XX_AAF_1962HZ_BITSHIFT;
            break;
    }

    // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
    spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC3, aafDelt);
    spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC4, sq(aafDelt) & 0xFF);
    spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC5, (sq(aafDelt) >> 8) | (aafBitshift << 4));

    // Configure acc Anti-Alias Filter for 1kHz sample rate (see tasks.c)
    spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC2, ICM426XX_AAF_258HZ_DELT << 1);
    spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC3, sq(ICM426XX_AAF_258HZ_DELT) & 0xFF);
    spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC4, (sq(ICM426XX_AAF_258HZ_DELT) >> 8) | (ICM426XX_AAF_258HZ_BITSHIFT << 4));

    // Configure gyro and acc UI Filters
    spiWriteReg(dev, ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);

    spiWriteReg(dev, ICM426XX_RA_INT_CONFIG, ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH);
    spiWriteReg(dev, ICM426XX_RA_INT_CONFIG0, ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR);

    spiWriteReg(dev, ICM426XX_RA_INT_SOURCE0, ICM426XX_UI_DRDY_INT1_EN_ENABLED);

    uint8_t intConfig1Value = spiReadRegMsk(dev, ICM426XX_RA_INT_CONFIG1);
    // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
    intConfig1Value &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);

    spiWriteReg(dev, ICM426XX_RA_INT_CONFIG1, intConfig1Value);
}

bool icm426xxSpiGyroDetect(gyroDev_t *gyro)
{
    switch (gyro->mpuDetectionResult.sensor) {
    case ICM_42605_SPI:
        break;
    case ICM_42688P_SPI:
        break;
    default:
        return false;
    }

    gyro->initFn = icm426xxGyroInit;
    gyro->readFn = mpuGyroReadSPI;

    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
#endif
