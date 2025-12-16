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

#ifdef USE_GYRO_SPI_ICM20948

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm20948.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

// 8 MHz max SPI frequency for register access. High speed reads can be faster.
#define ICM20948_MAX_SPI_CLK_HZ 8000000

uint8_t icm20948SpiDetect(const extDevice_t *dev) {

    spiWriteReg(dev, ICM20948_RA_PWR_MGMT_1, ICM20948_BIT_RESET);
    delay(100); 

    spiWriteReg(dev, ICM20948_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(20);

    // Select Bank 0 (WHO_AM_I is here)
    spiWriteReg(dev, ICM20948_RA_REG_BANK_SEL, 0x00);
    delay(20); 

    uint8_t icmDetected = MPU_NONE;
    for (int i = 0; i < 2; i++) {
        if (i > 0) delay(10);
        const uint8_t whoAmI = spiReadRegMsk(dev, ICM20948_RA_WHO_AM_I);

        if (whoAmI == ICM20948_WHO_AM_I_CONST) {
            icmDetected = ICM_20948_SPI;
            break;
        }
    }
    return icmDetected;
}

// --- ACCELEROMETER FUNCTIONS ---

void icm20948AccInit(accDev_t *acc)
{
    acc->acc_1G = acc->acc_high_fsr ? 2048 : 16384;

    // Switch to Bank 2 to configure Accelerometer
    spiWriteReg(&acc->gyro->dev, ICM20948_RA_REG_BANK_SEL, 2 << 4);
    delay(15);

    const uint8_t acc_fsr = acc->acc_high_fsr ? ICM20948_FSR_16G : ICM20948_FSR_2G;

    spiWriteReg(&acc->gyro->dev, ICM20948_RA_ACCEL_CONFIG, (acc_fsr << 1) | 1);
    delay(15);

    // Return to Bank 0 for data reading
    spiWriteReg(&acc->gyro->dev, ICM20948_RA_REG_BANK_SEL, 0 << 4);
    delay(15);
}

bool icm20948AccRead(accDev_t *acc)
{
    uint8_t dataToSend[7] = {ICM20948_RA_ACCEL_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[7];

    // Perform the transfer
    const bool ack = spiReadWriteBufRB(&acc->gyro->dev, dataToSend, data, 7);
    if (!ack) {
        return false;
    }
    
    acc->ADCRaw[X] = (int16_t)(((int16_t)data[1] << 8) | data[2]);
    acc->ADCRaw[Y] = (int16_t)(((int16_t)data[3] << 8) | data[4]);
    acc->ADCRaw[Z] = (int16_t)(((int16_t)data[5] << 8) | data[6]);

    return true;
}

bool icm20948SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != ICM_20948_SPI) {
        return false;
    }

    acc->initFn = icm20948AccInit;
    acc->readFn = icm20948AccRead;

    return true;
}

// --- GYRO FUNCTIONS ---

void icm20948GyroInit(gyroDev_t *gyro) {

    // bprintf("ICM20948: Starting Full Gyro Initialization.\n");
    gyro->gyroDataReg = ICM20948_RA_GYRO_XOUT_H;
    gyro->accDataReg = ICM20948_RA_ACCEL_XOUT_H;

    // Set SPI clock speed for initialization (8MHz max)
    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(ICM20948_MAX_SPI_CLK_HZ));

    // --- Bank 0: Reset and Clock Configuration ---
    spiWriteReg(&gyro->dev, ICM20948_RA_REG_BANK_SEL, 0 << 4);    // Ensure Bank 0
    delay(15);
    spiWriteReg(&gyro->dev, ICM20948_RA_PWR_MGMT_1, ICM20948_BIT_RESET);
    delay(100); // Wait for reset
    spiWriteReg(&gyro->dev, ICM20948_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);

    spiWriteReg(&gyro->dev, ICM20948_RA_LP_CONFIG, 0x00);
    delay(15);

    spiWriteReg(&gyro->dev, ICM20948_RA_PWR_MGMT_2, 0x00);
    delay(15);

    // --- Bank 2: Gyro Configuration (FSR, DLPF, Sample Rate) ---
    spiWriteReg(&gyro->dev, ICM20948_RA_REG_BANK_SEL, 2 << 4);    // Switch to Bank 2
    delay(15);

    // Gyro Config 1: FSR and DLPF
    const uint8_t gyro_fsr = ICM20948_FSR_2000DPS; // ICM20948 max range is ±2000 dps
    
    // DLPF Config: 
    const uint8_t DLPF_CONFIG_VALUE = 0x00; 

    uint8_t raGyroConfigData = 0;
    raGyroConfigData |= gyro_fsr << 1;           // Bits 2:1 = FSR
    raGyroConfigData |= DLPF_CONFIG_VALUE << 3;  // Bits 5:3 = DLPF Config
    raGyroConfigData |= 0x01;                    // Bit 0    = 1 (Enable DLPF)

    spiWriteReg(&gyro->dev, ICM20948_RA_GYRO_CONFIG_1, raGyroConfigData);
    delay(15);

    spiWriteReg(&gyro->dev, ICM20948_RA_GYRO_SMPLRT_DIV, gyro->mpuDividerDrops);
    delay(100);

    // --- Bank 0: Interrupts and Finalize ---
    spiWriteReg(&gyro->dev, ICM20948_RA_REG_BANK_SEL, 0 << 4);
    delay(15);

    spiWriteReg(&gyro->dev, ICM20948_RA_INT_PIN_CFG, 0x11);
    delay(15);
    spiWriteReg(&gyro->dev, ICM20948_RA_INT_ENABLE_1, 0x01);
                                                               
}

bool icm20948GyroReadSPI(gyroDev_t *gyro) {

    uint8_t dataToSend[7] = {ICM20948_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[7];

    const bool ack = spiReadWriteBufRB(&gyro->dev, dataToSend, data, 7);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)(((int16_t)data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)(((int16_t)data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)(((int16_t)data[5] << 8) | data[6]);

    return true;
}

bool icm20948SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != ICM_20948_SPI)
        return false;

    gyro->initFn = icm20948GyroInit;
    gyro->readFn = icm20948GyroReadSPI;

    gyro->scale = ICM20948_GYRO_SCALE_2000DPS; // ICM20948 max range is ±2000 dps

    return true;
}
#endif
