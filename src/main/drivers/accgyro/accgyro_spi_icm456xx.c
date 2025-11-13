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

#if defined(USE_ACCGYRO_ICM45686) || defined(USE_ACCGYRO_ICM45605)

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
#include "drivers/system.h"

#include "fc/runtime_config.h"

#include "sensors/gyro.h"
#include "pg/gyrodev.h"

/*
reference: https://github.com/tdk-invn-oss/motion.mcu.icm45686.driver
Datasheet: https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf
Datasheet: https://invensense.tdk.com/wp-content/uploads/documentation/DS-000576_ICM-45605.pdf

Note: ICM456xx has two modes of operation: Low-Power Mode Low-Noise Mode
Note: Now implemented only UI Interface with Low-Noise Mode

 The following diagram shows the signal path for each mode:
 The cut-off frequency of the filters is determined by the ODR setting.

                   Low-Noise Mode
     +------+     +--------------+    +-------------+    +--------------+    +------------------+
     | ADC  |---->| Anti-Alias   |--->| Interpolator|--->|     LPF      |--->| Sensor Registers |---> UI Interface
     |      |     | Filter (AAF) |    |             | +->| & ODR Select |    |                  |
     +--|---+     +--------------+    +-------------+ |  +--------------+    +------------------+
        |                                             |
        |           Low-Power Mode                    |
        |         +--------------+                    |
        |-------->| Notch Filter |--------------------|
        |         |              |
        |         +--------------+
        |
        |
     +--|---+           +--------------+       +------+       +------+      +------------------+
     | ADC  | --------> | Notch Filter | --->  | HPF  | --->  | LPF  | ---> | Sensor Registers | ---> AUX1 Interface
     |      |           |              |       |      |       |      |      |                  |
     +------+           +--------------+       +------+       +------+      +------------------+

 The AUX1 interface default configuration can be checked by read only register IOC_PAD_SCENARIO through host interface.
 By default, AUX1 interface is enabled, and default interface for AUX1 is SPI3W or I3CSM.

 In Low-Noise Mode, the ADC output is sent through an Anti-Alias Filter (AAF). The AAF is an FIR filter with fixed
 coefficients (not user configurable). The AAF can be enabled or disabled by the user using GYRO_SRC_CTRL and
 ACCEL_SRC_CTRL.

 The AUX1 signal path includes a Notch Filter. The notch filter is not user programmable. The usage of the notch
 filter in the auxiliary path is recommended for sharper roll-off and for the cases where user is asynchronously
 sampling the auxiliary interface data output at integer multiples of 1 kHz rate. The notch filter may be bypassed
 using GYRO_OIS_M6_BYP.

 The notch filter is followed by an HPF on the AUX1 signal path. HPF cut-off frequency can be selected using
 GYRO_OIS_HPFBW_SEL and ACCEL_OIS_HPFBW_SEL. HPF can be bypassed using GYRO_OIS_HPF1_BYP and
 ACCEL_OIS_HPF1_BYP.

 The HPF is followed by LPF on the AUX1 signal path. The AUX1 LPF BW is set by register bit field
 GYRO_OIS_LPF1BW_SEL and ACCEL_OIS_LPF1BW_SEL for gyroscope and accelerometer respectively. This is
 followed by Full Scale Range (FSR) selection based on user configurable settings for register fields
 GYRO_AUX1_FS_SEL and ACCEL_AUX1_FS_SEL. AUX1 output is fixed at 6.4kHz ODR.
*/

// NOTE: ICM-45686 does NOT have a bank select register like ICM-426xx
// The ICM-45686 uses Indirect Register (IREG) access for internal registers
// Register 0x75 is RESERVED/UNDEFINED in the ICM-45686 datasheet
// DO NOT use bank switching on this device

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// Register map User Bank 0 (UI Interface)
#define ICM456XX_WHO_AM_REGISTER                0x72
#define ICM456XX_REG_MISC2                      0x7F
#define ICM456XX_INT1_CONFIG0                   0x16
#define ICM456XX_INT1_CONFIG2                   0x18
#define ICM456XX_INT1_STATUS0                   0x19
#define ICM456XX_INT1_STATUS1                   0x1A
#define ICM456XX_GYRO_CONFIG0                   0x1C
#define ICM456XX_ACCEL_CONFIG0                  0x1B
#define ICM456XX_PWR_MGMT0                      0x10

// Register map IPREG_TOP1 (for future use)
// Note: SREG_CTRL register provides endian selection capability. 
// Currently not utilized as UI interface reads are in device's native endianness.
// Kept as reference for potential future optimization.
#define ICM456XX_RA_SREG_CTRL                   0xA267 // To access: 0xA200 + 0x67

// SREG_CTRL - 0x67 (bits 1:0 select data endianness)
#define ICM456XX_SREG_DATA_ENDIAN_SEL_LITTLE    (0 << 1) // Little endian (native)
#define ICM456XX_SREG_DATA_ENDIAN_SEL_BIG       (1 << 1) // Big endian (requires IREG write)

// MGMT0 - 0x10 - Gyro
#define ICM456XX_GYRO_MODE_OFF                  (0x00 << 2)
#define ICM456XX_GYRO_MODE_STANDBY              (0x01 << 2)
#define ICM456XX_GYRO_MODE_LP                   (0x02 << 2)  // Low Power Mode
#define ICM456XX_GYRO_MODE_LN                   (0x03 << 2)  // Low Noise Mode

// MGMT0 - 0x10 - Accel
#define ICM456XX_ACCEL_MODE_OFF                 (0x00)
#define ICM456XX_ACCEL_MODE_OFF2                (0x01)
#define ICM456XX_ACCEL_MODE_LP                  (0x02) // Low Power Mode
#define ICM456XX_ACCEL_MODE_LN                  (0x03) // Low Noise Mode

// INT1_CONFIG0 - 0x16
#define ICM456XX_INT1_STATUS_EN_RESET_DONE      (1 << 7)
#define ICM456XX_INT1_STATUS_EN_AUX1_AGC_RDY    (1 << 6)
#define ICM456XX_INT1_STATUS_EN_AP_AGC_RDY      (1 << 5)
#define ICM456XX_INT1_STATUS_EN_AP_FSYNC        (1 << 4)
#define ICM456XX_INT1_STATUS_EN_AUX1_DRDY       (1 << 3)
#define ICM456XX_INT1_STATUS_EN_DRDY            (1 << 2)
#define ICM456XX_INT1_STATUS_EN_FIFO_THS        (1 << 1)
#define ICM456XX_INT1_STATUS_EN_FIFO_FULL       (1 << 0)

// INT1_CONFIG2 - 0x18
#define ICM456XX_INT1_DRIVE_CIRCUIT_PP          (0 << 2)
#define ICM456XX_INT1_DRIVE_CIRCUIT_OD          (1 << 2)
#define ICM456XX_INT1_MODE_PULSED               (0 << 1)
#define ICM456XX_INT1_MODE_LATCHED              (1 << 1)
#define ICM456XX_INT1_POLARITY_ACTIVE_LOW       (0 << 0)
#define ICM456XX_INT1_POLARITY_ACTIVE_HIGH      (1 << 0)

// INT1_STATUS0 - 0x19
#define ICM456XX_INT1_STATUS_RESET_DONE         (1 << 7)
#define ICM456XX_INT1_STATUS_AUX1_AGC_RDY       (1 << 6)
#define ICM456XX_INT1_STATUS_AP_AGC_RDY         (1 << 5)
#define ICM456XX_INT1_STATUS_AP_FSYNC           (1 << 4)
#define ICM456XX_INT1_STATUS_AUX1_DRDY          (1 << 3)
#define ICM456XX_INT1_STATUS_DRDY               (1 << 2)
#define ICM456XX_INT1_STATUS_FIFO_THS           (1 << 1)
#define ICM456XX_INT1_STATUS_FIFO_FULL          (1 << 0)

// REG_MISC2 - 0x7F
#define ICM456XX_SOFT_RESET                     (1 << 1)
#define ICM456XX_RESET_TIMEOUT_US               20000  // 20 ms

#define ICM456XX_ACCEL_DATA_X1_UI               0x00
#define ICM456XX_GYRO_DATA_X1_UI                0x06

// ACCEL_CONFIG0 - 0x1B
#define ICM456XX_ACCEL_FS_SEL_32G               (0x00 << 4)
#define ICM456XX_ACCEL_FS_SEL_16G               (0x01 << 4)
#define ICM456XX_ACCEL_FS_SEL_8G                (0x02 << 4)
#define ICM456XX_ACCEL_FS_SEL_4G                (0x03 << 4)
#define ICM456XX_ACCEL_FS_SEL_2G                (0x04 << 4)

// ACCEL_CONFIG0 - 0x1B
#define ICM456XX_ACCEL_ODR_6K4_LN               0x03
#define ICM456XX_ACCEL_ODR_3K2_LN               0x04
#define ICM456XX_ACCEL_ODR_1K6_LN               0x05
#define ICM456XX_ACCEL_ODR_800_LN               0x06
#define ICM456XX_ACCEL_ODR_400_LP_LN            0x07
#define ICM456XX_ACCEL_ODR_200_LP_LN            0x08
#define ICM456XX_ACCEL_ODR_100_LP_LN            0x09
#define ICM456XX_ACCEL_ODR_50_LP_LN             0x0A
#define ICM456XX_ACCEL_ODR_25_LP_LN             0x0B
#define ICM456XX_ACCEL_ODR_12_5_LP_LN           0x0C
#define ICM456XX_ACCEL_ODR_6_25_LP              0x0D
#define ICM456XX_ACCEL_ODR_3_125_LP             0x0E
#define ICM456XX_ACCEL_ODR_1_5625_LP            0x0F

// GYRO_CONFIG0 - 0x1C
#define ICM456XX_GYRO_FS_SEL_4000DPS            (0x00 << 4)
#define ICM456XX_GYRO_FS_SEL_2000DPS            (0x01 << 4)
#define ICM456XX_GYRO_FS_SEL_1000DPS            (0x02 << 4)
#define ICM456XX_GYRO_FS_SEL_500DPS             (0x03 << 4)
#define ICM456XX_GYRO_FS_SEL_250DPS             (0x04 << 4)
#define ICM456XX_GYRO_FS_SEL_125DPS             (0x05 << 4)
#define ICM456XX_GYRO_FS_SEL_62_5DPS            (0x06 << 4)
#define ICM456XX_GYRO_FS_SEL_31_25DPS           (0x07 << 4)
#define ICM456XX_GYRO_FS_SEL_15_625DPS          (0x08 << 4)

// GYRO_CONFIG0 - 0x1C
#define ICM456XX_GYRO_ODR_6K4_LN                0x03
#define ICM456XX_GYRO_ODR_3K2_LN                0x04
#define ICM456XX_GYRO_ODR_1K6_LN                0x05
#define ICM456XX_GYRO_ODR_800_LN                0x06
#define ICM456XX_GYRO_ODR_400_LP_LN             0x07
#define ICM456XX_GYRO_ODR_200_LP_LN             0x08
#define ICM456XX_GYRO_ODR_100_LP_LN             0x09
#define ICM456XX_GYRO_ODR_50_LP_LN              0x0A
#define ICM456XX_GYRO_ODR_25_LP_LN              0x0B
#define ICM456XX_GYRO_ODR_12_5_LP_LN            0x0C
#define ICM456XX_GYRO_ODR_6_25_LP               0x0D
#define ICM456XX_GYRO_ODR_3_125_LP              0x0E
#define ICM456XX_GYRO_ODR_1_5625_LP             0x0F

// Accel IPREG_SYS2_REG_123 - 0x7B
#define ICM456XX_SRC_CTRL_AAF_ENABLE_BIT        (1 << 0) // Anti-Alias Filter - AAF
#define ICM456XX_SRC_CTRL_INTERP_ENABLE_BIT     (1 << 1) // Interpolator

// IPREG_SYS2_REG_123 - 0x7B
#define ICM456XX_ACCEL_SRC_CTRL_IREG_ADDR       0xA57B // To access register in IPREG_SYS2, add base address 0xA500 + offset

// IPREG_SYS1_REG_166 - 0xA6
#define ICM456XX_GYRO_SRC_CTRL_IREG_ADDR        0xA4A6 // To access register in IPREG_SYS1, add base address 0xA400 + offset

// HOST INDIRECT ACCESS REGISTER (IREG)
#define ICM456XX_REG_IREG_ADDR_15_8             0x7C
#define ICM456XX_REG_IREG_ADDR_7_0              0x7D
#define ICM456XX_REG_IREG_DATA                  0x7E


// IPREG_SYS1_REG_172 - 0xAC
#define ICM456XX_GYRO_UI_LPF_CFG_IREG_ADDR       0xA4AC // To access register in IPREG_SYS1, add base address 0xA400 + offset

// LPF UI - 0xAC PREG_SYS1_REG_172 (bits 2:0)
#define ICM456XX_GYRO_UI_LPFBW_BYPASS            0x00
#define ICM456XX_GYRO_UI_LPFBW_ODR_DIV_4         0x01 // 1600 Hz ODR = 6400 Hz:
#define ICM456XX_GYRO_UI_LPFBW_ODR_DIV_8         0x02 // 800 Hz ODR = 6400 Hz:
#define ICM456XX_GYRO_UI_LPFBW_ODR_DIV_16        0x03 // 400 Hz ODR = 6400 Hz:
#define ICM456XX_GYRO_UI_LPFBW_ODR_DIV_32        0x04 // 200 Hz ODR = 6400 Hz
#define ICM456XX_GYRO_UI_LPFBW_ODR_DIV_64        0x05 // 100 Hz ODR = 6400 Hz
#define ICM456XX_GYRO_UI_LPFBW_ODR_DIV_128       0x06 // 50 Hz ODR = 6400 Hz

// IPREG_SYS2_REG_131 - 0x83
#define ICM456XX_ACCEL_UI_LPF_CFG_IREG_ADDR      0xA583 // To access register in IPREG_SYS2, add base address 0xA500 + offset

// Accel UI path LPF - 0x83 IPREG_SYS2_REG_131 (bits 2:0)
#define ICM456XX_ACCEL_UI_LPFBW_BYPASS           0x00
#define ICM456XX_ACCEL_UI_LPFBW_ODR_DIV_4        0x01 // 400 Hz ODR = 1600 Hz:
#define ICM456XX_ACCEL_UI_LPFBW_ODR_DIV_8        0x02 // 200 Hz ODR = 1600 Hz:
#define ICM456XX_ACCEL_UI_LPFBW_ODR_DIV_16       0x03 // 100 Hz ODR = 1600 Hz:
#define ICM456XX_ACCEL_UI_LPFBW_ODR_DIV_32       0x04 // 50 Hz ODR = 1600 Hz
#define ICM456XX_ACCEL_UI_LPFBW_ODR_DIV_64       0x05 // 25 Hz ODR = 1600 Hz
#define ICM456XX_ACCEL_UI_LPFBW_ODR_DIV_128      0x06 // 12.5 Hz ODR = 1600 Hz


#ifndef ICM456XX_CLOCK
// Default: 24 MHz max SPI frequency
#define ICM456XX_MAX_SPI_CLK_HZ                 24000000
#else
#define ICM456XX_MAX_SPI_CLK_HZ                 ICM456XX_CLOCK
#endif

#define HZ_TO_US(hz)                            ((int32_t)((1000 * 1000) / (hz)))

#define ICM456XX_BIT_IREG_DONE                  (1 << 0)

// Startup timing constants (DS-000577 Table 9-6)
#define ICM456XX_ACCEL_STARTUP_TIME_MS          10  // Min accel startup from OFF/STANDBY/LP
#define ICM456XX_GYRO_STARTUP_TIME_MS           35  // Min gyro startup from OFF/STANDBY/LP
#define ICM456XX_SENSOR_ENABLE_DELAY_MS         1   // Allow sensors to power on and stabilize
#define ICM456XX_IREG_TIMEOUT_US                5000 // IREG operation timeout (5ms max)

#define ICM456XX_DATA_LENGTH                    6  // 3 axes * 2 bytes per axis
#define ICM456XX_SPI_BUFFER_SIZE                (1 + ICM456XX_DATA_LENGTH) // 1 byte register + 6 bytes data

// Combined read constants for accel+gyro data (12 bytes total sensor data)
#define ICM456XX_COMBINED_DATA_LENGTH           12  // 6 bytes accel + 6 bytes gyro
#define ICM456XX_COMBINED_SPI_BUFFER_SIZE       (1 + ICM456XX_COMBINED_DATA_LENGTH) // 13 bytes total

// static uint8_t getGyroLpfConfig(const gyroHardwareLpf_e hardwareLpf)
// {
//     switch (hardwareLpf) {
//     case GYRO_HARDWARE_LPF_NORMAL:
//         // Default to 800 Hz (ODR/8) for lowest latency by default on ICM456xx
//         return ICM456XX_GYRO_UI_LPFBW_ODR_DIV_8;
//     case GYRO_HARDWARE_LPF_OPTION_1:
//         return ICM456XX_GYRO_UI_LPFBW_ODR_DIV_16;
//     case GYRO_HARDWARE_LPF_OPTION_2:
//         return ICM456XX_GYRO_UI_LPFBW_ODR_DIV_8;
// #ifdef USE_GYRO_DLPF_EXPERIMENTAL
//     case GYRO_HARDWARE_LPF_EXPERIMENTAL:
//         return ICM456XX_GYRO_UI_LPFBW_ODR_DIV_4;
// #endif
//     default:
//         return ICM456XX_GYRO_UI_LPFBW_BYPASS;
//     }
// }

/**
 * @brief This function follows the IREG WRITE procedure (Section 14.1-14.4 of the datasheet)
 * using indirect addressing via IREG_ADDR_15_8, IREG_ADDR_7_0, and IREG_DATA registers.
 * After writing, an internal operation transfers the data to the target IREG address.
 * Ensures compliance with the required minimum time gap and checks the IREG_DONE bit.
 *
 * @param dev   Pointer to the SPI device structure.
 * @param reg   16-bit internal IREG register address.
 * @param value Value to be written to the register.
 * @return true if the write was successful
 */
static bool icm456xx_write_ireg(const extDevice_t *dev, uint16_t reg, uint8_t value)
{
    // IREG writes should not be performed while armed to avoid disrupting flight
    if (ARMING_FLAG(ARMED)) {
        return false;
    }

    const uint8_t msb = (reg >> 8) & 0xFF;
    const uint8_t lsb = reg & 0xFF;

    spiWriteReg(dev, ICM456XX_REG_IREG_ADDR_15_8, msb);
    spiWriteReg(dev, ICM456XX_REG_IREG_ADDR_7_0, lsb);
    spiWriteReg(dev, ICM456XX_REG_IREG_DATA, value);

    // Check IREG_DONE (bit 0 of REG_MISC2 = 0x7F) with elapsed-time tracking
    for (uint32_t waited_us = 0; waited_us < ICM456XX_IREG_TIMEOUT_US; waited_us += 10) {
        const uint8_t misc2 = spiReadRegMsk(dev, ICM456XX_REG_MISC2);
        if (misc2 & ICM456XX_BIT_IREG_DONE) {
            return true;
        }
        delayMicroseconds(10);
    }

    return false; // timeout
}

static bool icm456xx_enableAAFandInterpolator(const extDevice_t *dev, uint16_t reg, bool enableAAF, bool enableInterp)
{
    const uint8_t value = (enableAAF ? ICM456XX_SRC_CTRL_AAF_ENABLE_BIT : 0)
                        | (enableInterp ? ICM456XX_SRC_CTRL_INTERP_ENABLE_BIT : 0);
    return icm456xx_write_ireg(dev, reg, value);
}

static bool icm456xx_configureLPF(const extDevice_t *dev, uint16_t reg, uint8_t lpfDiv)
{
    if (lpfDiv > 0x07) {
        return false;
    }

    return icm456xx_write_ireg(dev, reg, lpfDiv & 0x07);
}

static void icm456xx_enableSensors(const extDevice_t *dev, bool enable)
{
    uint8_t value = enable
        ? (ICM456XX_GYRO_MODE_LN | ICM456XX_ACCEL_MODE_LN)
        : (ICM456XX_GYRO_MODE_OFF | ICM456XX_ACCEL_MODE_OFF);

    spiWriteReg(dev, ICM456XX_PWR_MGMT0, value);
}

void icm456xxAccInit(accDev_t *acc)
{
    // ICM-45686 does not use bank switching (register 0x75 is reserved)
    // Accelerometer configuration is done in icm456xxGyroInit()
    // This function only sets the scale factor and sample rate

    switch (acc->mpuDetectionResult.sensor) {
    case ICM_45686_SPI:
        acc->acc_1G = 1024; // 32g scale (32768 / 32g = 1024 LSB/g)
        acc->gyro->accSampleRateHz = 1600;
        break;
    case ICM_45605_SPI:
    default:
        acc->acc_1G = 2048; // 16g scale (32768 / 16g = 2048 LSB/g) - ICM-45605 max is ±16g
        acc->gyro->accSampleRateHz = 1600;
        break;
    }

    // Set up register addresses for combined DMA reads
    // Accel and gyro data are contiguous: accel at 0x00, gyro at 0x06
    acc->gyro->accDataReg = ICM456XX_ACCEL_DATA_X1_UI;  // 0x00
}

void icm456xxGyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    spiSetClkDivisor(dev, spiCalculateDivider(ICM456XX_MAX_SPI_CLK_HZ));

    mpuGyroInit(gyro);

    // ICM-45686 does not use bank switching (register 0x75 is reserved)
    // Step 1: Enable sensors FIRST in Low-Noise mode (required for IREG and ODR/FS writes)
    icm456xx_enableSensors(dev, true);
    delay(ICM456XX_SENSOR_ENABLE_DELAY_MS); // Allow sensors to enter Low-Noise mode

    // Step 2: Configure IREG features FIRST (AAF, LPF) - requires sensors to be ON but BEFORE ODR/FS config
    // This prevents IREG operations from interfering with ODR/FS settings
    
    // Enable Anti-Alias (AAF) Filter and Interpolator for Gyro for stable timing
    if (!icm456xx_enableAAFandInterpolator(dev, ICM456XX_GYRO_SRC_CTRL_IREG_ADDR, true, true)) {
        // AAF/Interpolator initialization failed, fallback to disabled state
        icm456xx_enableAAFandInterpolator(dev, ICM456XX_GYRO_SRC_CTRL_IREG_ADDR, false, false);
    }

    // Configure Gyro UI LPF to 800 Hz (ODR/8 for 6.4 kHz ODR)
    if (!icm456xx_configureLPF(dev, ICM456XX_GYRO_UI_LPF_CFG_IREG_ADDR, ICM456XX_GYRO_UI_LPFBW_ODR_DIV_8)) {
        // If LPF configuration fails, fallback to BYPASS
        icm456xx_configureLPF(dev, ICM456XX_GYRO_UI_LPF_CFG_IREG_ADDR, ICM456XX_GYRO_UI_LPFBW_BYPASS);
    }

    // Enable Anti-Alias (AAF) Filter for Accel; disable Interpolator to reduce latency
    if (!icm456xx_enableAAFandInterpolator(dev, ICM456XX_ACCEL_SRC_CTRL_IREG_ADDR, true, false)) {
        // AAF/Interpolator initialization failed, fallback to disabled state
        icm456xx_enableAAFandInterpolator(dev, ICM456XX_ACCEL_SRC_CTRL_IREG_ADDR, false, false);
    }

    // Set the Accel UI LPF bandwidth cut-off (Section 7.3 of datasheet)
    if (!icm456xx_configureLPF(dev, ICM456XX_ACCEL_UI_LPF_CFG_IREG_ADDR, ICM456XX_ACCEL_UI_LPFBW_ODR_DIV_8)) {
        // If LPF configuration fails, fallback to BYPASS
        icm456xx_configureLPF(dev, ICM456XX_ACCEL_UI_LPF_CFG_IREG_ADDR, ICM456XX_ACCEL_UI_LPFBW_BYPASS);
    }

    // Step 3: Configure gyro full-scale range and ODR (after IREG setup)
    switch (gyro->mpuDetectionResult.sensor) {
    case ICM_45686_SPI:
    case ICM_45605_SPI:
    default:
        gyro->scale = GYRO_SCALE_2000DPS;
        gyro->gyroRateKHz = GYRO_RATE_6400_Hz;
        gyro->gyroSampleRateHz = 6400;
        spiWriteReg(dev, ICM456XX_GYRO_CONFIG0, ICM456XX_GYRO_FS_SEL_2000DPS | ICM456XX_GYRO_ODR_6K4_LN);
        break;
    }

    // Step 4: Configure accelerometer full-scale range and ODR
    switch (gyro->mpuDetectionResult.sensor) {
    case ICM_45686_SPI:
        // ICM-45686 supports ±32g
        spiWriteReg(dev, ICM456XX_ACCEL_CONFIG0, ICM456XX_ACCEL_FS_SEL_32G | ICM456XX_ACCEL_ODR_1K6_LN);
        break;
    case ICM_45605_SPI:
        // ICM-45605 max is ±16g
        spiWriteReg(dev, ICM456XX_ACCEL_CONFIG0, ICM456XX_ACCEL_FS_SEL_16G | ICM456XX_ACCEL_ODR_1K6_LN);
        break;
    default:
        break;
    }

    // Step 5: Re-enable sensors after ODR/FS configuration to ensure data path starts
    // Per ICM-456xx datasheet, ODR changes may require sensor restart
    icm456xx_enableSensors(dev, false);
    delay(1);
    icm456xx_enableSensors(dev, true);
    delay(ICM456XX_GYRO_STARTUP_TIME_MS); // Allow full gyro startup time (35ms)

    gyro->gyroShortPeriod = clockMicrosToCycles(HZ_TO_US(gyro->gyroSampleRateHz));

    // Step 6: Configure interrupt pin (pulsed, active-high, push-pull)
    spiWriteReg(dev, ICM456XX_INT1_CONFIG2, ICM456XX_INT1_MODE_PULSED | ICM456XX_INT1_DRIVE_CIRCUIT_PP |
                                            ICM456XX_INT1_POLARITY_ACTIVE_HIGH);

    spiWriteReg(dev, ICM456XX_INT1_CONFIG0, ICM456XX_INT1_STATUS_EN_DRDY);

    // Set up register addresses for combined DMA reads
    // ICM456xx data is contiguous: accel at 0x00, gyro at 0x06
    gyro->accDataReg = ICM456XX_ACCEL_DATA_X1_UI;  // 0x00
    gyro->gyroDataReg = ICM456XX_GYRO_DATA_X1_UI;  // 0x06
    gyro->gyroDmaMaxDuration = 0;  // Data ready interrupt ensures timely reads
}

uint8_t icm456xxSpiDetect(const extDevice_t *dev)
{
    uint8_t icmDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;
    uint32_t waited_us = 0;

    // ICM-45686 does not use bank switching (register 0x75 is reserved)
    // Perform soft reset directly

    // Soft reset
    spiWriteReg(dev, ICM456XX_REG_MISC2, ICM456XX_SOFT_RESET);

    // Wait for reset to complete (bit 1 of REG_MISC2 becomes 0)
    while ((spiReadRegMsk(dev, ICM456XX_REG_MISC2) & ICM456XX_SOFT_RESET) != 0) {
        delayMicroseconds(10);
        waited_us += 10;

        if (waited_us >= ICM456XX_RESET_TIMEOUT_US) {
            return MPU_NONE;
        }
    }

    // Initialize power management to a known state after reset
    // This ensures sensors are off and ready for proper initialization
    spiWriteReg(dev, ICM456XX_PWR_MGMT0, 0x00);

    do {
        delay(1);
        const uint8_t whoAmI = spiReadRegMsk(dev, ICM456XX_WHO_AM_REGISTER);
        switch (whoAmI) {
        case ICM45686_WHO_AM_I_CONST:
            icmDetected = ICM_45686_SPI;
            break;
        case ICM45605_WHO_AM_I_CONST:
            icmDetected = ICM_45605_SPI;
            break;
        default:
            icmDetected = MPU_NONE;
            break;
        }

    } while (icmDetected == MPU_NONE && attemptsRemaining--);

    return icmDetected;

}

bool icm456xxAccReadSPI(accDev_t *acc)
{
    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT_DMA:
        // DMA mode: gyro-only transfer in this driver; perform a direct accel read
        {
            uint8_t raw[ICM456XX_DATA_LENGTH];
            const bool ack = spiReadRegMskBufRB(&acc->gyro->dev, ICM456XX_ACCEL_DATA_X1_UI, raw, ICM456XX_DATA_LENGTH);
            if (!ack) {
                return false;
            }
            acc->ADCRaw[X] = (int16_t)((raw[1] << 8) | raw[0]);
            acc->ADCRaw[Y] = (int16_t)((raw[3] << 8) | raw[2]);
            acc->ADCRaw[Z] = (int16_t)((raw[5] << 8) | raw[4]);
        }
        break;
        
    default:
    {
        // Non-DMA modes: perform separate accel read (little endian)
        uint8_t raw[ICM456XX_DATA_LENGTH];
        const bool ack = spiReadRegMskBufRB(&acc->gyro->dev, ICM456XX_ACCEL_DATA_X1_UI, raw, ICM456XX_DATA_LENGTH);
        if (!ack) {
            return false;
        }
        // Extract little endian data (LSB at raw[0], MSB at raw[1])
        acc->ADCRaw[X] = (int16_t)((raw[1] << 8) | raw[0]);
        acc->ADCRaw[Y] = (int16_t)((raw[3] << 8) | raw[2]);
        acc->ADCRaw[Z] = (int16_t)((raw[5] << 8) | raw[4]);
        break;
    }
    }
    return true;
}

bool icm456xxSpiAccDetect(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
    case ICM_45686_SPI:
    case ICM_45605_SPI:
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
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the transfer buffer to 0xFF
        memset(gyro->dev.txBuf, 0xff, ICM456XX_SPI_BUFFER_SIZE);

        // Provide a small initial DMA duration estimate for EXTI sync heuristics
        gyro->gyroDmaMaxDuration = 5;

        // Only enable EXTI-driven DMA once we have confirmed interrupts are firing
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
#ifdef USE_DMA
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uintptr_t)gyro;
                // GYRO-ONLY READ: Start from gyro register (0x06), read gyro (7 bytes total including reg)
                // Buffer layout after read:
                //   rxBuf[0]   = dummy byte (register echo)
                //   rxBuf[1-6] = gyro X,Y,Z (little endian, 6 bytes)
                gyro->dev.txBuf[0] = ICM456XX_GYRO_DATA_X1_UI | 0x80;  // Read from gyro register
                gyro->segments[0].len = ICM456XX_SPI_BUFFER_SIZE;  // 7 bytes: 1 reg + 6 data
                gyro->segments[0].callback = mpuIntCallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                gyro->segments[0].u.buffers.rxData = gyro->dev.rxBuf;
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else
#endif
            {
                // Interrupts are present, but no DMA
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            // EXTI not confirmed yet: operate without EXTI to guarantee fresh samples
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }

        break;
    }

    case GYRO_EXTI_INT:
    {
        // Interrupt-driven non-DMA mode: read gyro data only
        uint8_t raw[ICM456XX_DATA_LENGTH];
        const bool ack = spiReadRegMskBufRB(&gyro->dev, ICM456XX_GYRO_DATA_X1_UI, raw, ICM456XX_DATA_LENGTH);
        if (!ack) {
            return false;
        }

        gyro->gyroADCRaw[X] = (int16_t)((raw[1] << 8) | raw[0]);
        gyro->gyroADCRaw[Y] = (int16_t)((raw[3] << 8) | raw[2]);
        gyro->gyroADCRaw[Z] = (int16_t)((raw[5] << 8) | raw[4]);

        // Pulsed mode: no need to read INT1_STATUS0 here
        break;
    }

    case GYRO_EXTI_NO_INT:
    {
        // Non-interrupt mode: read gyro data each call
        uint8_t raw[ICM456XX_DATA_LENGTH];
        const bool ack = spiReadRegMskBufRB(&gyro->dev, ICM456XX_GYRO_DATA_X1_UI, raw, ICM456XX_DATA_LENGTH);
        if (!ack) {
            return false;
        }

        gyro->gyroADCRaw[X] = (int16_t)((raw[1] << 8) | raw[0]);
        gyro->gyroADCRaw[Y] = (int16_t)((raw[3] << 8) | raw[2]);
        gyro->gyroADCRaw[Z] = (int16_t)((raw[5] << 8) | raw[4]);

        // Pulsed mode: no need to read INT1_STATUS0 here
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // DMA mode: Gyro-only read was performed from gyro register (0x06)
        // ICM456xx data is LITTLE ENDIAN (no byte swap needed)
        // Buffer layout:
        //   rxBuf[0]   = dummy
        //   rxBuf[1-2] = gyro X (LSB, MSB)
        //   rxBuf[3-4] = gyro Y
        //   rxBuf[5-6] = gyro Z

        // Extract gyro data (little endian format, LSB first)
        gyro->gyroADCRaw[X] = (int16_t)((gyro->dev.rxBuf[2] << 8) | gyro->dev.rxBuf[1]);
        gyro->gyroADCRaw[Y] = (int16_t)((gyro->dev.rxBuf[4] << 8) | gyro->dev.rxBuf[3]);
        gyro->gyroADCRaw[Z] = (int16_t)((gyro->dev.rxBuf[6] << 8) | gyro->dev.rxBuf[5]);

        // Pulsed mode: no need to read INT1_STATUS0 here
        break;
    }

    default:
        break;
    }

    return true;
}


bool icm456xxSpiGyroDetect(gyroDev_t *gyro)
{
    switch (gyro->mpuDetectionResult.sensor) {
    case ICM_45686_SPI:
    case ICM_45605_SPI:
        gyro->initFn = icm456xxGyroInit;
        gyro->readFn = icm456xxGyroReadSPI;
        break;
    default:
        return false;
    }

    return true;
}

#endif // USE_ACCGYRO_ICM45686 || USE_ACCGYRO_ICM45605
