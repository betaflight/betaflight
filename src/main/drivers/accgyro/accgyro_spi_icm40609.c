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
#include <math.h>

#include "platform.h"

#if defined(USE_ACCGYRO_ICM40609D)

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm40609.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "sensors/gyro.h"
#include "pg/gyrodev.h"

// Datasheet: https://invensense.tdk.com/wp-content/uploads/2022/07/DS-000330-ICM-40609-D-v1.2.pdf

#define ICM40609_WHO_AM_I_REG               0x75

#define ICM40609_REG_BANK_SEL               0x76

#define ICM40609_USER_BANK_0                0x00
#define ICM40609_USER_BANK_1                0x01
#define ICM40609_USER_BANK_2                0x02
#define ICM40609_USER_BANK_3                0x03
#define ICM40609_USER_BANK_4                0x04

// Registers in BANK 0
#define ICM40609_REG_INT_CONFIG             0x14
#define ICM40609_REG_INTF_CONFIG0           0x4C
#define ICM40609_REG_PWR_MGMT0              0x4E
#define ICM40609_REG_GYRO_CONFIG0           0x4F
#define ICM40609_REG_ACCEL_CONFIG0          0x50
#define ICM40609_REG_GYRO_CONFIG1           0x51 // Bandwidth of the temperature signal DLPF
#define ICM40609_REG_GYRO_ACCEL_CONFIG0     0x52 // Bandwidth for Gyro & Accel LPF
#define ICM40609_REG_GYRO_ACCEL_CONFIG1     0x53 // Bandwidth for Gyro & Accel LPF
#define ICM40609_REG_INT_CONFIG0            0x63
#define ICM40609_REG_INT_CONFIG1            0x64
#define ICM40609_REG_INT_SOURCE0            0x65

// Registers in BANK 1
#define ICM40609_REG_GYRO_CONFIG_STATIC2    0x0B
#define ICM40609_REG_GYRO_CONFIG_STATIC3    0x0C
#define ICM40609_REG_GYRO_CONFIG_STATIC4    0x0D
#define ICM40609_REG_GYRO_CONFIG_STATIC5    0x0E
#define ICM40609_REG_GYRO_CONFIG_STATIC6    0x0F
#define ICM40609_REG_GYRO_CONFIG_STATIC7    0x10
#define ICM40609_REG_GYRO_CONFIG_STATIC8    0x11
#define ICM40609_REG_GYRO_CONFIG_STATIC9    0x12
#define ICM40609_REG_GYRO_CONFIG_STATIC10   0x13 // gyro notch filter

// Registers in BANK 2
#define ICM40609_REG_ACCEL_CONFIG_STATIC2   0x03
#define ICM40609_REG_ACCEL_CONFIG_STATIC3   0x04
#define ICM40609_REG_ACCEL_CONFIG_STATIC4   0x05

// PWR_MGMT0_REG - 0x4E
#define ICM40609_TEMP_SENSOR_ENABLED        (0 << 5)
#define ICM40609_TEMP_SENSOR_DISABLED       (1 << 5)
#define ICM40609_IDLE                       (0 << 4)
#define ICM40609_RC_ON                      (1 << 4)

// // PWR_MGMT0_REG - 0x4E bits [3:2]
#define ICM40609_GYRO_MODE_OFF              (0 << 2)
#define ICM40609_GYRO_MODE_STANDBY          (1 << 2)
#define ICM40609_GYRO_MODE_LN               (3 << 2)

// // PWR_MGMT0_REG - 0x4E bits [1:0]
#define ICM40609_ACCEL_MODE_OFF             (0 << 0) // Of course, this is joke, but it's so easy to check bits orders in datasheet
#define ICM40609_ACCEL_MODE_PWR_OFF         (1 << 0)
#define ICM40609_ACCEL_MODE_LP              (2 << 0)
#define ICM40609_ACCEL_MODE_LN              (3 << 0)

// GYRO_CONFIG0_REG - 0x4F bits [7:5]
#define ICM40609_GYRO_FS_SEL_2000DPS        (0 << 5)  // default)
#define ICM40609_GYRO_FS_SEL_1000DPS        (1 << 5)
#define ICM40609_GYRO_FS_SEL_500DPS         (2 << 5)
#define ICM40609_GYRO_FS_SEL_250DPS         (3 << 5)
#define ICM40609_GYRO_FS_SEL_125DPS         (4 << 5)
#define ICM40609_GYRO_FS_SEL_62_5DPS        (5 << 5)
#define ICM40609_GYRO_FS_SEL_31_25DPS       (6 << 5)
#define ICM40609_GYRO_FS_SEL_15_625DPS      (7 << 5)

// GYRO_CONFIG0_REG - 0x4F bits [3:0]
#define ICM40609_GYRO_ODR_32KHZ             0x01
#define ICM40609_GYRO_ODR_16KHZ             0x02
#define ICM40609_GYRO_ODR_8KHZ              0x03
#define ICM40609_GYRO_ODR_4KHZ              0x04
#define ICM40609_GYRO_ODR_2KHZ              0x05
#define ICM40609_GYRO_ODR_1KHZ              0x06  // default
#define ICM40609_GYRO_ODR_200HZ             0x07
#define ICM40609_GYRO_ODR_100HZ             0x08
#define ICM40609_GYRO_ODR_50HZ              0x09
#define ICM40609_GYRO_ODR_25HZ              0x0A
#define ICM40609_GYRO_ODR_12_5HZ            0x0B
#define ICM40609_GYRO_ODR_RESERVED_C        0x0C
#define ICM40609_GYRO_ODR_RESERVED_D        0x0D
#define ICM40609_GYRO_ODR_RESERVED_E        0x0E
#define ICM40609_GYRO_ODR_500HZ             0x0F

// ACCEL_CONFIG0_REG - 0x50 bits [7:5]
// Per ICM-40609-D datasheet DS-000330 v1.2 Table 6.1
#define ICM40609_ACCEL_FS_SEL_32G           (0 << 5)  // ±32g (1024 LSB/g)
#define ICM40609_ACCEL_FS_SEL_16G           (1 << 5)  // ±16g (2048 LSB/g)
#define ICM40609_ACCEL_FS_SEL_8G            (2 << 5)  // ±8g (4096 LSB/g)
#define ICM40609_ACCEL_FS_SEL_4G            (3 << 5)  // ±4g (8192 LSB/g)
#define ICM40609_ACCEL_FS_SEL_2G            (4 << 5)  // ±2g (16384 LSB/g)
#define ICM40609_ACCEL_FS_SEL_1G            (5 << 5)  // ±1g (32768 LSB/g)
#define ICM40609_ACCEL_FS_SEL_0_5G          (6 << 5)  // ±0.5g (65536 LSB/g)
#define ICM40609_ACCEL_FS_SEL_0_125G        (7 << 5)  // ±0.125g (262144 LSB/g)

// ACCEL_CONFIG0_REG - 0x50 bits [3:0]
#define ICM40609_ACCEL_ODR_32KHZ            0x01
#define ICM40609_ACCEL_ODR_16KHZ            0x02
#define ICM40609_ACCEL_ODR_8KHZ             0x03
#define ICM40609_ACCEL_ODR_4KHZ             0x04
#define ICM40609_ACCEL_ODR_2KHZ             0x05
#define ICM40609_ACCEL_ODR_1KHZ             0x06  // 1kHz (LN mode) (default)
#define ICM40609_ACCEL_ODR_200HZ            0x07
#define ICM40609_ACCEL_ODR_100HZ            0x08
#define ICM40609_ACCEL_ODR_50HZ             0x09
#define ICM40609_ACCEL_ODR_25HZ             0x0A
#define ICM40609_ACCEL_ODR_12_5HZ           0x0B
#define ICM40609_ACCEL_ODR_500HZ            0x0F

// INT_CONFIG_REG - 0x14
#define ICM40609_INT1_MODE_PULSED           (0 << 2)
#define ICM40609_INT1_MODE_LATCHED          (1 << 2)

#define ICM40609_INT1_DRIVE_OPEN_DRAIN      (0 << 1)
#define ICM40609_INT1_DRIVE_PUSH_PULL       (1 << 1)

#define ICM40609_INT1_POLARITY_ACTIVE_LOW   (0 << 0)
#define ICM40609_INT1_POLARITY_ACTIVE_HIGH  (1 << 0)

// NT_SOURCE0_REG - 0x65
#define ICM40609_UI_FSYNC_INT1_EN           (1 << 6)
#define ICM40609_PLL_RDY_INT1_EN            (1 << 5)
#define ICM40609_RESET_DONE_INT1_EN         (1 << 4)
#define ICM40609_UI_DRDY_INT1_EN            (1 << 3)
#define ICM40609_FIFO_THS_INT1_EN           (1 << 2)
#define ICM40609_FIFO_FULL_INT1_EN          (1 << 1)
#define ICM40609_UI_AGC_RDY_INT1_EN         (1 << 0)

// INT_CONFIG0 - 0x63
#define ICM40609_UI_DRDY_INT_CLEAR_STATUS   (0 << 4)

// INT_CONFIG1 - 0x64
#define ICM40609_INT_TPULSE_100US           (0 << 6) // ODR < 4kHz, optional
#define ICM40609_INT_TPULSE_8US             (1 << 6) // ODR ≥ 4kHz
#define ICM40609_INT_TDEASSERT_ENABLED      (0 << 5)
#define ICM40609_INT_TDEASSERT_DISABLED     (1 << 5)
#define ICM40609_INT_ASYNC_RESET_ENABLED    (1 << 4)
#define ICM40609_INT_ASYNC_RESET_DISABLED   (0 << 4)

// REG_GYRO_CONFIG1 - 0x51 bits [3:2]
#define ICM40609_GYRO_UI_FILT_ORDER_1ST     (0 << 2)
#define ICM40609_GYRO_UI_FILT_ORDER_2ND     (1 << 2)
#define ICM40609_GYRO_UI_FILT_ORDER_3RD     (2 << 2)

// REG_GYRO_CONFIG1 - 0x51 bits [1:0]
#define ICM40609_GYRO_DEC2_M2_ORDER_3RD     (2 << 0)

// REG_GYRO_ACCEL_CONFIG0 - 0x52 bits [7:4]
#define ICM40609_ACCEL_UI_FILT_BW_ODR_DIV2   (0 << 4)
#define ICM40609_ACCEL_UI_FILT_BW_ODR_DIV4   (1 << 4) // default
#define ICM40609_ACCEL_UI_FILT_BW_ODR_DIV5   (2 << 4)
#define ICM40609_ACCEL_UI_FILT_BW_ODR_DIV8   (3 << 4)
#define ICM40609_ACCEL_UI_FILT_BW_ODR_DIV10  (4 << 4)
#define ICM40609_ACCEL_UI_FILT_BW_ODR_DIV16  (5 << 4)
#define ICM40609_ACCEL_UI_FILT_BW_ODR_DIV20  (6 << 4)
#define ICM40609_ACCEL_UI_FILT_BW_ODR_DIV40  (7 << 4)
#define ICM40609_ACCEL_UI_FILT_BW_LP_TRIVIAL_400HZ_ODR   (14 << 4) // Bit[7:4] - Low Latency
#define ICM40609_ACCEL_UI_FILT_BW_LP_TRIVIAL_200HZ_8XODR (15 << 4) // Bit[7:4] - Low Latency

// REG_GYRO_ACCEL_CONFIG0 - 0x52 bits [3:0]
#define ICM40609_GYRO_UI_FILT_BW_ODR_DIV2    (0 << 0)
#define ICM40609_GYRO_UI_FILT_BW_ODR_DIV4    (1 << 0) // default
#define ICM40609_GYRO_UI_FILT_BW_ODR_DIV5    (2 << 0)
#define ICM40609_GYRO_UI_FILT_BW_ODR_DIV8    (3 << 0)
#define ICM40609_GYRO_UI_FILT_BW_ODR_DIV10   (4 << 0)
#define ICM40609_GYRO_UI_FILT_BW_ODR_DIV16   (5 << 0)
#define ICM40609_GYRO_UI_FILT_BW_ODR_DIV20   (6 << 0)
#define ICM40609_GYRO_UI_FILT_BW_ODR_DIV40   (7 << 0)
#define ICM40609_GYRO_UI_FILT_BW_LP_TRIVIAL_400HZ_ODR   (14 << 0) // Bit[3:0] - Low Latency
#define ICM40609_GYRO_UI_FILT_BW_LP_TRIVIAL_200HZ_8XODR (15 << 0) // Bit[3:0] - Low Latency

// REG_ACCEL_CONFIG_STATIC2 - 0x03 bit [0]
#define ICM40609_ACCEL_AAF_DIS              (1 << 0)

// REG_GYRO_CONFIG_STATIC2 - 0x0B bit [1]
#define ICM40609_GYRO_AAF_DIS               (1 << 1)

//REG_GYRO_CONFIG_STATIC2 - 0x0B bit [0]
#define ICM40609_GYRO_NF_DIS_BIT            (1 << 0)

// GYRO_HPF_BW_IND - 0x13 bit [3:1] — High-pass filter 3dB cutoff frequency selection
#define ICM40609_GYRO_HPF_BW_IND_MASK       (0x07 << 1) // bits [3:1]

// GYRO_HPF_ORD_IND [0] — High-pass filter order (1st or 2nd)
#define ICM40609_GYRO_HPF_ORD_IND_MASK      (1 << 0)

// GYRO_CONFIG1 (0x51)
#define ICM40609_GYRO_UI_FILT_ORD_MASK      (0x03 << 2) // bits [3:2]
#define ICM40609_GYRO_DEC2_M2_ORD_MASK      (0x03 << 0) // bits [1:0]

// ACCEL_CONFIG1 (0x53)
#define ICM40609_ACCEL_UI_FILT_ORD_MASK     (0x03 << 3) // bits [4:3]
#define ICM40609_ACCEL_DEC2_M2_ORD_MASK     (0x03 << 1) // bits [2:1]

#define ICM40609_ACCEL_DATA_X1_UI           0x1F
#define ICM40609_GYRO_DATA_X1_UI            0x25

#define ICM40609_RESET_REG                  0x11
#define ICM40609_SOFT_RESET_VAL             0x01

#ifndef ICM40609_LOCK
// Default: 24 MHz max SPI frequency
#define ICM40609_MAX_SPI_CLK_HZ 24000000
#else
// Use the supplied value
#define ICM40609_MAX_SPI_CLK_HZ  ICM40609_LOCK
#endif

#define ICM40609_AAF_PROFILE_COUNT          63

// Table 5.2 Bandwidth (Hz)
typedef struct {
    uint16_t hz;
    uint8_t delt;
    uint16_t deltsqr;
    uint8_t bitshift;
} ICM40609_AafProfile;


static const ICM40609_AafProfile aafProfiles[ICM40609_AAF_PROFILE_COUNT] = {
    { 42, 1, 1, 15 },
    { 84, 2, 4, 13 },
    { 126, 3, 9, 12 },
    { 170, 4, 16, 11 },
    { 213, 5, 25, 10 },
    { 258, 6, 36, 10 },
    { 303, 7, 49, 9 },
    { 348, 8, 64, 9 },
    { 394, 9, 81, 9 },
    { 441, 10, 100, 8 },
    { 488, 11, 122, 8 },
    { 536, 12, 144, 8 },
    { 585, 13, 170, 8 },
    { 634, 14, 196, 8 },
    { 684, 15, 224, 7 },
    { 734, 16, 256, 7 },
    { 785, 17, 288, 7 },
    { 837, 18, 324, 7 },
    { 890, 19, 360, 6 },
    { 943, 20, 400, 6 },
    { 997, 21, 440, 6 },
    { 1051, 22, 488, 6 },
    { 1107, 23, 528, 6 },
    { 1163, 24, 576, 6 },
    { 1220, 25, 624, 6 },
    { 1277, 26, 680, 6 },
    { 1336, 27, 736, 5 },
    { 1395, 28, 784, 5 },
    { 1454, 29, 848, 5 },
    { 1515, 30, 896, 5 },
    { 1577, 31, 960, 5 },
    { 1639, 32, 1024, 5 },
    { 1702, 33, 1088, 5 },
    { 1766, 34, 1152, 5 },
    { 1830, 35, 1232, 5 },
    { 1896, 36, 1296, 5 },
    { 1962, 37, 1376, 4 },
    { 2029, 38, 1440, 4 },
    { 2097, 39, 1536, 4 },
    { 2166, 40, 1600, 4 },
    { 2235, 41, 1696, 4 },
    { 2306, 42, 1760, 4 },
    { 2377, 43, 1856, 4 },
    { 2449, 44, 1952, 4 },
    { 2522, 45, 2016, 4 },
    { 2596, 46, 2112, 4 },
    { 2671, 47, 2208, 4 },
    { 2746, 48, 2304, 4 },
    { 2823, 49, 2400, 4 },
    { 2900, 50, 2496, 4 },
    { 2978, 51, 2592, 4 },
    { 3057, 52, 2720, 4 },
    { 3137, 53, 2816, 4 },
    { 3217, 54, 2944, 3 },
    { 3299, 55, 3008, 3 },
    { 3381, 56, 3136, 3 },
    { 3464, 57, 3264, 3 },
    { 3548, 58, 3392, 3 },
    { 3633, 59, 3456, 3 },
    { 3718, 60, 3584, 3 },
    { 3805, 61, 3712, 3 },
    { 3892, 62, 3840, 3 },
    { 3979, 63, 3968, 3 },
};

/*
 * ICM-40609D Group Delay @DC (ODR = 8000 Hz)
 *
 * +-------------------+--------------------+----------+
 * | Filter Order      | Delay (UI_FILT_BW) | NBW (Hz) |
 * +-------------------+--------------------+----------+
 * | 1st order filter  | 0.2 ms             |  2204.6  |
 * | 2nd order filter  | 0.2 ms             |  2204.6  |
 * | 3rd order filter  | 0.2 ms             |  2096.3  |
 * +-------------------+--------------------+----------+
 *
 * Note:
 *  Delay is independent of UI_FILT_BW when ODR = 8000Hz.
 *  5.4 UI FILTER BLOCK TDK ICM-40609D Datasheet Rev 1.2 (2023)
 *
 *  Filter order (standard DSP behavior):
 *  1st order -6 dB/octave
 *  2nd order -12 dB/octave
 *  3rd order -18 dB/octave
 *  These roll-off rates are typical for LPF/HPF filters in digital signal processing (DSP).
 */
typedef enum {
    ICM40609_UI_FILT_ORDER_1ST = 0,
    ICM40609_UI_FILT_ORDER_2ND = 1,
    ICM40609_UI_FILT_ORDER_3RD = 2
} icm40609UiFiltOrder_e;

typedef enum {
    ICM40609_HPF_ORDER_1ST = 0,
    ICM40609_HPF_ORDER_2ND = 1
} icm40609HpfOrder_e;

// Bandwidth selection for High-Pass Filter
// ICM40609_REG_GYRO_CONFIG_STATIC10 - 0x13 bits [3:1]
// NOTE: clarify with new datasheet, section 5.6 not found in V1.2
typedef enum {
    ICM40609_HPF_BW_0 = 0,
    ICM40609_HPF_BW_1 = 1,
    ICM40609_HPF_BW_2 = 2,
    ICM40609_HPF_BW_3 = 3,
    ICM40609_HPF_BW_4 = 4,
    ICM40609_HPF_BW_5 = 5,
    ICM40609_HPF_BW_6 = 6,
    ICM40609_HPF_BW_7 = 7,
} icm40609HpfBw_e;

// Bandwidth selection for Notch Filter GYRO_NF_BW_SEL
// ICM40609_REG_GYRO_CONFIG_STATIC10 - 0x13 bits [6:4]
typedef enum {
    ICM40609_GYRO_NF_BW_1449HZ = 0,
    ICM40609_GYRO_NF_BW_680HZ  = 1,
    ICM40609_GYRO_NF_BW_329HZ  = 2,
    ICM40609_GYRO_NF_BW_162HZ  = 3,
    ICM40609_GYRO_NF_BW_80HZ   = 4,
    ICM40609_GYRO_NF_BW_40HZ   = 5,
    ICM40609_GYRO_NF_BW_20HZ   = 6,
    ICM40609_GYRO_NF_BW_10HZ   = 7,
} icm40609GyroNfBw_e;

typedef enum {
    ICM40609_TEMP_FILT_BW_4000HZ = 0, // 4000Hz, 0.125ms latency (default)
    ICM40609_TEMP_FILT_BW_170HZ  = 1, // 170Hz, 1ms latency
    ICM40609_TEMP_FILT_BW_82HZ   = 2, // 82Hz, 2ms latency
    ICM40609_TEMP_FILT_BW_40HZ   = 3, // 40Hz, 4ms latency
    ICM40609_TEMP_FILT_BW_20HZ   = 4, // 20Hz, 8ms latency
    ICM40609_TEMP_FILT_BW_10HZ   = 5, // 10Hz, 16ms latency
    ICM40609_TEMP_FILT_BW_5HZ    = 6, // 5Hz, 32ms latency
} icm40609TempFiltBw_e;

static void icm40609SelectUserBank(const extDevice_t *dev, uint8_t bank)
{
    if (bank > 4) {
        return; // out of range
    }
    spiWriteReg(dev, ICM40609_REG_BANK_SEL, bank & 0x07); // bit [2:0]
}

static void setGyroAccPowerMode(const extDevice_t *dev, bool enable)
{
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);

    spiWriteReg(dev, ICM40609_REG_PWR_MGMT0,
        ICM40609_RC_ON |
        ICM40609_TEMP_SENSOR_ENABLED |
        (enable ? ICM40609_GYRO_MODE_LN | ICM40609_ACCEL_MODE_LN
                : ICM40609_GYRO_MODE_OFF | ICM40609_ACCEL_MODE_OFF));

    if (enable) {
        delayMicroseconds(200);
    } else {
        delay(50);
    }
}

static void icm40609GetAafParams(uint16_t targetHz, ICM40609_AafProfile* res)
{
    uint16_t i = 0;
    while (i < ICM40609_AAF_PROFILE_COUNT && targetHz >  aafProfiles[i].hz) {
         i++;
    }
    if (i < ICM40609_AAF_PROFILE_COUNT) {
        *res = aafProfiles[i];
    } else {
        // not found - Requested frequency is higher than max available
        *res = aafProfiles[ICM40609_AAF_PROFILE_COUNT - 1];
    }
}

static void icm40609SetAccelAafByHz(const extDevice_t *dev, bool aafEnable, uint16_t targetHz)
{
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_2);

    if (aafEnable && targetHz > 0) {
        ICM40609_AafProfile aafProfile;

        icm40609GetAafParams(targetHz, &aafProfile);

        uint8_t reg03 = spiReadRegMsk(dev, ICM40609_REG_ACCEL_CONFIG_STATIC2);

        reg03 &= ~ICM40609_ACCEL_AAF_DIS; // Clear ACCEL_AAF_DIS to enable AAF
        reg03 = (reg03 & 0x81) | (aafProfile.delt << 1); // Keep reserved bit 7, set ACCEL_AAF_DELT
        spiWriteReg(dev, ICM40609_REG_ACCEL_CONFIG_STATIC2, reg03);

        uint8_t reg04 = aafProfile.deltsqr & 0xFF;
        uint8_t reg05 = ((aafProfile.bitshift & 0x0F) << 4) | ((aafProfile.deltsqr >> 8) & 0x0F);

        spiWriteReg(dev, ICM40609_REG_ACCEL_CONFIG_STATIC3, reg04);
        spiWriteReg(dev, ICM40609_REG_ACCEL_CONFIG_STATIC4, reg05);

    } else {
        uint8_t reg03 = spiReadRegMsk(dev, ICM40609_REG_ACCEL_CONFIG_STATIC2);
        reg03 |= ICM40609_ACCEL_AAF_DIS; // Set ACCEL_AAF_DIS to disable AAF
        spiWriteReg(dev, ICM40609_REG_ACCEL_CONFIG_STATIC2, reg03);
    }

    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);
}

static void icm40609SetGyroAafByHz(const extDevice_t *dev, bool aafEnable, uint16_t targetHz)
{
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_1);

    if (aafEnable && targetHz > 0) {
        ICM40609_AafProfile aafProfile;

        icm40609GetAafParams(targetHz, &aafProfile);

        uint8_t reg0C = aafProfile.delt & 0x3F;
        uint8_t reg0D = aafProfile.deltsqr & 0xFF;
        uint8_t reg0E = ((aafProfile.bitshift & 0x0F) << 4) | ((aafProfile.deltsqr >> 8) & 0x0F);

        uint8_t reg0B = spiReadRegMsk(dev, ICM40609_REG_GYRO_CONFIG_STATIC2);
        reg0B &= ~ICM40609_GYRO_AAF_DIS; // Clear AAF_DIS (bit1) to enable AAF
        spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC2, reg0B);

        spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC3, reg0C);
        spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC4, reg0D);
        spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC5, reg0E);

    } else {
        uint8_t reg0B = spiReadRegMsk(dev, ICM40609_REG_GYRO_CONFIG_STATIC2);
        reg0B |= ICM40609_GYRO_AAF_DIS; // Set AAF_DIS (bit1) to disable AAF
        spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC2, reg0B);
    }

    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);
}

static void icm40609SetGyroHPF(const extDevice_t *dev, bool hpfEnable, icm40609HpfBw_e hpfBwInd, icm40609HpfOrder_e hpfOrder)
{
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_1);

    uint8_t reg13 = spiReadRegMsk(dev, ICM40609_REG_GYRO_CONFIG_STATIC10);

    reg13 &= ~(ICM40609_GYRO_HPF_BW_IND_MASK | ICM40609_GYRO_HPF_ORD_IND_MASK); // clear HPF bits

    if (hpfEnable) {
        reg13 |= (hpfBwInd << 1) | (hpfOrder << 0);
    }

    spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC10, reg13);

    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);
}

static void icm40609SetGyroNotch(const extDevice_t *dev, bool notchEnable, icm40609GyroNfBw_e bwSel, float fdesiredKhz)
{
    if (fdesiredKhz < 1.0f || fdesiredKhz > 3.0f) {
        return; // (1kHz to 3kHz) Operating the notch filter outside this range is not supported.
    }

    icm40609SelectUserBank(dev, ICM40609_USER_BANK_1);

    // Enable/disable Notch filter
    uint8_t reg2 = spiReadRegMsk(dev, ICM40609_REG_GYRO_CONFIG_STATIC2);
    if (notchEnable) {
        reg2 &= ~ICM40609_GYRO_NF_DIS_BIT;
    } else {
        reg2 |= ICM40609_GYRO_NF_DIS_BIT; // Bypass
    }
    spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC2, reg2);

    if (notchEnable) {
        // Set Bandwidth in STATIC10 (0x13)
        uint8_t reg13 = spiReadRegMsk(dev, ICM40609_REG_GYRO_CONFIG_STATIC10);
        reg13 &= ~(0x07 << 4);
        reg13 |= (bwSel & 0x07) << 4; // GYRO_NF_BW_SEL [6:4]
        spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC10, reg13);

        // Section 5.1.1 (v1.2) Frequency of Notch Filter (each axis)
        // Calculate COSWZ and SEL based on desired frequency
        float coswz = cosf(2.0f * M_PIf * fdesiredKhz / 32.0f);
        uint8_t nf_coswz_lsb = 0;
        uint8_t nf_coswz_msb = 0;
        uint8_t nf_coswz_sel = 0;

        if (fabsf(coswz) <= 0.875f) {
            int16_t nf_coswz = (int16_t)roundf(coswz * 256.0f);
            nf_coswz_lsb = nf_coswz & 0xFF;
            nf_coswz_msb = (nf_coswz >> 8) & 0x01;
            nf_coswz_sel = 0;
        } else {
            nf_coswz_sel = 1;
            int16_t nf_coswz;
            if (coswz > 0.875f) {
                nf_coswz = (int16_t)roundf(8.0f * (1.0f - coswz) * 256.0f);
            } else {
                nf_coswz = (int16_t)roundf(8.0f * (1.0f + coswz) * 256.0f);
            }
            nf_coswz_lsb = nf_coswz & 0xFF;
            nf_coswz_msb = (nf_coswz >> 8) & 0x01;
        }

        // Write NF_COSWZ[7:0] for X, Y, Z
        spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC6, nf_coswz_lsb); // X
        spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC7, nf_coswz_lsb); // Y
        spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC8, nf_coswz_lsb); // Z

        // Write NF_COSWZ[8] and NF_COSWZ_SEL into STATIC9 (0x12)
        uint8_t reg9 = 0;
        reg9 |= (nf_coswz_msb << 0); // X NF_COSWZ[8]
        reg9 |= (nf_coswz_msb << 1); // Y NF_COSWZ[8]
        reg9 |= (nf_coswz_msb << 2); // Z NF_COSWZ[8]
        reg9 |= (nf_coswz_sel << 3); // X COSWZ_SEL
        reg9 |= (nf_coswz_sel << 4); // Y COSWZ_SEL
        reg9 |= (nf_coswz_sel << 5); // Z COSWZ_SEL
        spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG_STATIC9, reg9);
    }

    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);
}

static void icm40609SetTempFiltBw(const extDevice_t *dev, icm40609TempFiltBw_e bw)
{
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);

    uint8_t reg51 = spiReadRegMsk(dev, ICM40609_REG_GYRO_CONFIG1);
    reg51 &= ~(0x07 << 5);
    reg51 |= (bw << 5);
    spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG1, reg51);
}

static void icm40609SetGyroUiFiltOrder(const extDevice_t *dev, icm40609UiFiltOrder_e order)
{
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);

    uint8_t reg51 = spiReadRegMsk(dev, ICM40609_REG_GYRO_CONFIG1);
    reg51 &= ~ICM40609_GYRO_UI_FILT_ORD_MASK;
    reg51 |= (order << 2); // Write GYRO_UI_FILT_ORD to bits [3:2]
    spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG1, reg51);
}

static void icm40609SetAccelUiFiltOrder(const extDevice_t *dev, icm40609UiFiltOrder_e order)
{
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);

    uint8_t reg53 = spiReadRegMsk(dev, ICM40609_REG_GYRO_ACCEL_CONFIG1);
    reg53 &= ~ICM40609_ACCEL_UI_FILT_ORD_MASK;
    reg53 |= (order << 3); // Write ACCEL_UI_FILT_ORD to bits [4:3]
    spiWriteReg(dev, ICM40609_REG_GYRO_ACCEL_CONFIG1, reg53);
}

static void icm40609SetGyroDec2M2(const extDevice_t *dev, bool enable)
{
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);

    uint8_t reg51 = spiReadRegMsk(dev, ICM40609_REG_GYRO_CONFIG1);
    reg51 &= ~ICM40609_GYRO_DEC2_M2_ORD_MASK;

    if (enable) {
        reg51 |= (2 << 0);
    }

    spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG1, reg51);
}

// Set endianness for sensor data (bit 4 of INTF_CONFIG0, reg 0x4C)
// true  = Big Endian
// false = Little Endian
static void icm40609SetEndianess(const extDevice_t *dev, bool bigEndian)
{
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);

    uint8_t reg4C = spiReadRegMsk(dev, ICM40609_REG_INTF_CONFIG0);
    reg4C &= ~(1 << 4);
    reg4C |= bigEndian << 4;

    spiWriteReg(dev, ICM40609_REG_INTF_CONFIG0, reg4C);
}

void icm40609AccInit(accDev_t *acc)
{
    acc->acc_1G = 2048; // 16g scale
    acc->gyro->accSampleRateHz = 1000;

}

void icm40609GyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;
    spiSetClkDivisor(dev, spiCalculateDivider(ICM40609_MAX_SPI_CLK_HZ));

    mpuGyroInit(gyro);
    gyro->accDataReg = ICM40609_ACCEL_DATA_X1_UI;
    gyro->gyroDataReg = ICM40609_GYRO_DATA_X1_UI;

    // Enable sensors before configuration - registers ignored when powered off
    setGyroAccPowerMode(dev, true);
    delay(35); // Sensor power-on stabilization time

    icm40609SetEndianess(dev, true);

    // Configure accelerometer: 16g full-scale range, 1kHz ODR
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);
    spiWriteReg(dev, ICM40609_REG_ACCEL_CONFIG0, ICM40609_ACCEL_FS_SEL_16G | ICM40609_ACCEL_ODR_1KHZ);
    delay(10); // Accelerometer configuration delay

    // Configure filters before gyro
    icm40609SetTempFiltBw(dev, ICM40609_TEMP_FILT_BW_4000HZ); // 4000Hz, 0.125ms latency
    icm40609SetGyroUiFiltOrder(dev, ICM40609_UI_FILT_ORDER_3RD);
    icm40609SetAccelUiFiltOrder(dev, ICM40609_UI_FILT_ORDER_3RD);
    icm40609SetGyroDec2M2(dev, true);

    // Set filter bandwidth: Low Latency
    spiWriteReg(&gyro->dev, ICM40609_REG_GYRO_ACCEL_CONFIG0,
                    ICM40609_ACCEL_UI_FILT_BW_LP_TRIVIAL_200HZ_8XODR |
                    ICM40609_GYRO_UI_FILT_BW_LP_TRIVIAL_200HZ_8XODR);

    uint16_t gyroHWLpf; // Anti-Alias Filter (AAF) in Hz
    switch (gyroConfig()->gyro_hardware_lpf) {
    case GYRO_HARDWARE_LPF_NORMAL:
        gyroHWLpf = 213;
        break;
    case GYRO_HARDWARE_LPF_OPTION_1:
        gyroHWLpf = 488;
        break;
    case GYRO_HARDWARE_LPF_OPTION_2:
        gyroHWLpf = 997;
        break;
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    case GYRO_HARDWARE_LPF_EXPERIMENTAL:
        gyroHWLpf = 1962;
        break;
#endif
        default:
        gyroHWLpf = 213;
    }

    icm40609SetGyroAafByHz(dev, true, gyroHWLpf);
    icm40609SetAccelAafByHz(dev, true, gyroHWLpf);

    icm40609SetGyroNotch(dev, true, ICM40609_GYRO_NF_BW_1449HZ, 1.5f);

    icm40609SetGyroHPF(dev, true, ICM40609_HPF_BW_1, ICM40609_HPF_ORDER_1ST);

    // Configure gyro: 2000dps full-scale range, 8kHz ODR
    spiWriteReg(dev, ICM40609_REG_GYRO_CONFIG0, ICM40609_GYRO_FS_SEL_2000DPS | ICM40609_GYRO_ODR_8KHZ);
    gyro->scale = GYRO_SCALE_2000DPS;
    gyro->gyroRateKHz = GYRO_RATE_8_kHz;
    gyro->gyroSampleRateHz = 8000;
    delay(35); // Gyro startup time per DS-000330 Table 9-6

    // Enable interrupt
    spiWriteReg(dev, ICM40609_REG_INT_SOURCE0, ICM40609_UI_DRDY_INT1_EN);

    // Set INT1 to pulse mode, push-pull, active high
    spiWriteReg(dev, ICM40609_REG_INT_CONFIG,
        ICM40609_INT1_MODE_PULSED |
        ICM40609_INT1_DRIVE_PUSH_PULL |
        ICM40609_INT1_POLARITY_ACTIVE_HIGH);

    spiWriteReg(dev, ICM40609_REG_INT_CONFIG0, ICM40609_UI_DRDY_INT_CLEAR_STATUS); // Auto-clear on read

    // INT1: 8us pulse width, de-assertion enabled, async reset disabled
    spiWriteReg(dev, ICM40609_REG_INT_CONFIG1,
        ICM40609_INT_TPULSE_8US |
        ICM40609_INT_TDEASSERT_DISABLED |
        ICM40609_INT_ASYNC_RESET_DISABLED);

}

uint8_t icm40609SpiDetect(const extDevice_t *dev)
{
    icm40609SelectUserBank(dev, ICM40609_USER_BANK_0);

    spiWriteReg(dev, ICM40609_RESET_REG, ICM40609_SOFT_RESET_VAL);
    delay(1);

    uint8_t whoAmI = spiReadRegMsk(dev, ICM40609_WHO_AM_I_REG);
    if (whoAmI == ICM40609_WHO_AM_I_CONST) {
        return ICM_40609_SPI;
    }
    return MPU_NONE;
}

bool icm40609SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor == ICM_40609_SPI) {
        acc->initFn = icm40609AccInit;
        acc->readFn = mpuAccReadSPI;
        return true;
    }
    return false;
}

bool icm40609SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor == ICM_40609_SPI) {
        gyro->initFn = icm40609GyroInit;
        gyro->readFn = mpuGyroReadSPI;
        return true;
    }
    return false;
}

#endif // USE_ACCGYRO_ICM40609D
