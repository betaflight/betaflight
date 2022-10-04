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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

// 10 MHz max SPI frequency
#define ICM20689_MAX_SPI_CLK_HZ 8000000

// Register 0x37 - INT_PIN_CFG / Pin Bypass Enable Configuration
#define ICM20689_INT_ANYRD_2CLEAR   0x10

// Register 0x68 - SIGNAL_PATH_RESET / Pin Bypass Enable Configuration
#define ICM20689_ACCEL_RST          0x02
#define ICM20689_TEMP_RST           0x01

// Register 0x6a - USER_CTRL / User Control
#define ICM20689_I2C_IF_DIS         0x10

// Register 0x6b - PWR_MGMT_1 / Power Management 1
#define ICM20689_BIT_RESET          0x80

/* Allow CLKSEL setting time to settle when PLL is selected
 *
 * Not specified in the ICM-20689 datasheet, but in the ICM-20690 datasheet,
 *
 * https://invensense.tdk.com/wp-content/uploads/2016/10/DS-000178-ICM-20690-v1.0.pdf
 *
 * says (section 10.11) that the clock selection takes ~20us to settle. Testing
 * has shown that 60us is required, so double to allow a margin
 */
#define ICM20689_CLKSEL_SETTLE_US   120

/* Not specified in the ICM-20689 datasheet, but in the MPU-6000 datasheet,
 *
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 * says (section 4.28) suggest a delay of 100ms after a reset
 */
#define ICM20689_RESET_DELAY_MS     100

/* Not specified in the ICM-20689 datasheet, but in the MPU-6000 datasheet,
 *
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 * says (section 4.28) suggest a delay of 100ms after a path reset
 */
#define ICM20689_PATH_RESET_DELAY_MS 100

uint8_t icm20689SpiDetect(const extDevice_t *dev)
{
    // Note that the following reset is being done repeatedly by each MPU6000
    // compatible device being probed

    // reset the device configuration
    spiWriteReg(dev, MPU_RA_PWR_MGMT_1, ICM20689_BIT_RESET);
    delay(ICM20689_RESET_DELAY_MS);

    uint8_t icmDetected;

    const uint8_t whoAmI = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);

    switch (whoAmI) {
    case ICM20601_WHO_AM_I_CONST:
        icmDetected = ICM_20601_SPI;
        break;
    case ICM20602_WHO_AM_I_CONST:
        icmDetected = ICM_20602_SPI;
        break;
    case ICM20608G_WHO_AM_I_CONST:
        icmDetected = ICM_20608_SPI;
        break;
    case ICM20689_WHO_AM_I_CONST:
        icmDetected = ICM_20689_SPI;
        break;
    default:
        icmDetected = MPU_NONE;
        return icmDetected;
    }

    // We now know the device is recognised so it's safe to perform device
    // specific register accesses

    // Disable Primary I2C Interface
    spiWriteReg(dev, MPU_RA_USER_CTRL, ICM20689_I2C_IF_DIS);

    // Reset the device signal paths
    spiWriteReg(dev, MPU_RA_SIGNAL_PATH_RESET, ICM20689_ACCEL_RST | ICM20689_TEMP_RST);

    delay(ICM20689_PATH_RESET_DELAY_MS);

    return icmDetected;
}

void icm20689AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool icm20689SpiAccDetect(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
    case ICM_20602_SPI:
    case ICM_20689_SPI:
        break;
    default:
        return false;
    }

    acc->initFn = icm20689AccInit;
    acc->readFn = mpuAccReadSPI;

    return true;
}

void icm20689GyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    spiSetClkDivisor(dev, spiCalculateDivider(ICM20689_MAX_SPI_CLK_HZ));

    mpuGyroInit(gyro);

    // Device was already reset during detection so proceed with configuration

    spiWriteReg(dev, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delayMicroseconds(ICM20689_CLKSEL_SETTLE_US);
    spiWriteReg(dev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    spiWriteReg(dev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    spiWriteReg(dev, MPU_RA_CONFIG, mpuGyroDLPF(gyro));
    spiWriteReg(dev, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);

    // Data ready interrupt configuration
    spiWriteReg(dev, MPU_RA_INT_PIN_CFG, ICM20689_INT_ANYRD_2CLEAR);

    spiWriteReg(dev, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
}

bool icm20689SpiGyroDetect(gyroDev_t *gyro)
{
    switch (gyro->mpuDetectionResult.sensor) {
    case ICM_20601_SPI:
    case ICM_20602_SPI:
    case ICM_20608_SPI:
    case ICM_20689_SPI:
        break;
    default:
        return false;
    }

    gyro->initFn = icm20689GyroInit;
    gyro->readFn = mpuGyroReadSPI;

    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
