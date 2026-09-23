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

#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_ASM330LHH

#include "accgyro_spi_asm330lhh.h"

#include "sensors/gyro.h"
#include "drivers/time.h"

/* See datasheet
 *
 *     STMicroelectronics ASM330LHH, automotive 6-axis IMU (DocID031239)
 *
 * The ASM330LHH shares much of its register layout with the LSM6DSV/LSM6DSO family,
 * but it lacks the embedded functions (FSM/MLC/sensor-hub), HAODR modes and dual
 * accelerometer channel found on the newer parts.
 */

// 10 MHz max SPI frequency
#define ASM330LHH_MAX_SPI_CLK_HZ 10000000

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// Macros to encode/decode multi-bit values
#define ASM330LHH_ENCODE_BITS(val, mask, shift)   ((val << shift) & mask)
#define ASM330LHH_DECODE_BITS(val, mask, shift)   ((val & mask) >> shift)

// SDO pin pull-up enable/disable register (R/W)
#define ASM330LHH_PIN_CTRL                  0x02
#define ASM330LHH_PIN_CTRL_SDO_PU_EN                    0x40

// FIFO control register 1 (R/W)
#define ASM330LHH_FIFO_CTRL1                0x07

// FIFO control register 2 (R/W)
#define ASM330LHH_FIFO_CTRL2                0x08
#define ASM330LHH_FIFO_CTRL2_STOP_ON_WTM                0x80
#define ASM330LHH_FIFO_CTRL2_ODRCHG_EN                   0x10
#define ASM330LHH_FIFO_CTRL2_WTM8                        0x01

// FIFO control register 3 (R/W)
#define ASM330LHH_FIFO_CTRL3                0x09
#define ASM330LHH_FIFO_CTRL3_BDR_GY_MASK                 0xf0
#define ASM330LHH_FIFO_CTRL3_BDR_GY_SHIFT                4
#define ASM330LHH_FIFO_CTRL3_BDR_XL_MASK                 0x0f
#define ASM330LHH_FIFO_CTRL3_BDR_XL_SHIFT                0

// FIFO control register 4 (R/W)
#define ASM330LHH_FIFO_CTRL4                0x0A
#define ASM330LHH_FIFO_CTRL4_DEC_TS_BATCH_MASK           0xc0
#define ASM330LHH_FIFO_CTRL4_DEC_TS_BATCH_SHIFT          6
#define ASM330LHH_FIFO_CTRL4_ODR_T_BATCH_MASK            0x30
#define ASM330LHH_FIFO_CTRL4_ODR_T_BATCH_SHIFT           4
#define ASM330LHH_FIFO_CTRL4_FIFO_MODE_MASK              0x07
#define ASM330LHH_FIFO_CTRL4_FIFO_MODE_SHIFT             0
#define ASM330LHH_FIFO_CTRL4_FIFO_MODE_BYPASS            0
#define ASM330LHH_FIFO_CTRL4_FIFO_MODE_FIFO_MODE         1
#define ASM330LHH_FIFO_CTRL4_FIFO_MODE_CONT_TO_FIFO      3
#define ASM330LHH_FIFO_CTRL4_FIFO_MODE_BYPASS_TO_CONT    4
#define ASM330LHH_FIFO_CTRL4_FIFO_MODE_CONT              6
#define ASM330LHH_FIFO_CTRL4_FIFO_MODE_BYPASS_TO_FIFO    7

// Counter batch data rate register 1 (R/W)
#define ASM330LHH_COUNTER_BDR_REG1          0x0B
#define ASM330LHH_COUNTER_BDR_REG1_DATAREADY_PULSED     0x80
#define ASM330LHH_COUNTER_BDR_REG1_RST_COUNTER_BDR      0x40
#define ASM330LHH_COUNTER_BDR_REG1_TRIG_COUNTER_BDR     0x20

// Counter batch data rate register 2 (R/W)
#define ASM330LHH_COUNTER_BDR_REG2          0x0C

// INT1 pin control register (R/W)
#define ASM330LHH_INT1_CTRL                 0x0D
#define ASM330LHH_INT1_CTRL_DEN_DRDY_FLAG                0x80
#define ASM330LHH_INT1_CTRL_INT1_CNT_BDR                 0x40
#define ASM330LHH_INT1_CTRL_INT1_FIFO_FULL               0x20
#define ASM330LHH_INT1_CTRL_INT1_FIFO_OVR                0x10
#define ASM330LHH_INT1_CTRL_INT1_FIFO_TH                 0x08
#define ASM330LHH_INT1_CTRL_INT1_BOOT                    0x04
#define ASM330LHH_INT1_CTRL_INT1_DRDY_G                  0x02
#define ASM330LHH_INT1_CTRL_INT1_DRDY_XL                 0x01

// INT2 pin control register (R/W)
#define ASM330LHH_INT2_CTRL                 0x0E
#define ASM330LHH_INT2_CTRL_INT2_CNT_BDR                 0x40
#define ASM330LHH_INT2_CTRL_INT2_FIFO_FULL               0x20
#define ASM330LHH_INT2_CTRL_INT2_FIFO_OVR                0x10
#define ASM330LHH_INT2_CTRL_INT2_FIFO_TH                 0x08
#define ASM330LHH_INT2_CTRL_INT2_DRDY_TEMP               0x04
#define ASM330LHH_INT2_CTRL_INT2_DRDY_G                  0x02
#define ASM330LHH_INT2_CTRL_INT2_DRDY_XL                 0x01

// WHO_AM_I register (R)
#define ASM330LHH_WHO_AM_I                  0x0F

// Accelerometer control register 1 (R/W)
#define ASM330LHH_CTRL1_XL                  0x10
#define ASM330LHH_CTRL1_XL_ODR_XL_MASK                   0xf0
#define ASM330LHH_CTRL1_XL_ODR_XL_SHIFT                  4
#define ASM330LHH_CTRL1_XL_ODR_XL_POWERDOWN              0x00
#define ASM330LHH_CTRL1_XL_ODR_XL_12_5HZ                 0x01
#define ASM330LHH_CTRL1_XL_ODR_XL_26HZ                   0x02
#define ASM330LHH_CTRL1_XL_ODR_XL_52HZ                   0x03
#define ASM330LHH_CTRL1_XL_ODR_XL_104HZ                  0x04
#define ASM330LHH_CTRL1_XL_ODR_XL_208HZ                  0x05
#define ASM330LHH_CTRL1_XL_ODR_XL_417HZ                  0x06
#define ASM330LHH_CTRL1_XL_ODR_XL_833HZ                  0x07
#define ASM330LHH_CTRL1_XL_ODR_XL_1667HZ                 0x08
#define ASM330LHH_CTRL1_XL_ODR_XL_3333HZ                 0x09
#define ASM330LHH_CTRL1_XL_ODR_XL_6667HZ                 0x0a
#define ASM330LHH_CTRL1_XL_FS_XL_MASK                    0x0c
#define ASM330LHH_CTRL1_XL_FS_XL_SHIFT                   2
#define ASM330LHH_CTRL1_XL_FS_XL_2G                      0x00
#define ASM330LHH_CTRL1_XL_FS_XL_16G                     0x01
#define ASM330LHH_CTRL1_XL_FS_XL_4G                      0x02
#define ASM330LHH_CTRL1_XL_FS_XL_8G                      0x03
#define ASM330LHH_CTRL1_XL_LPF2_XL_EN                    0x02

// Gyroscope control register 2 (R/W)
#define ASM330LHH_CTRL2_G                   0x11
#define ASM330LHH_CTRL2_G_ODR_G_MASK                     0xf0
#define ASM330LHH_CTRL2_G_ODR_G_SHIFT                    4
#define ASM330LHH_CTRL2_G_ODR_G_POWERDOWN                0x00
#define ASM330LHH_CTRL2_G_ODR_G_12_5HZ                   0x01
#define ASM330LHH_CTRL2_G_ODR_G_26HZ                     0x02
#define ASM330LHH_CTRL2_G_ODR_G_52HZ                     0x03
#define ASM330LHH_CTRL2_G_ODR_G_104HZ                    0x04
#define ASM330LHH_CTRL2_G_ODR_G_208HZ                    0x05
#define ASM330LHH_CTRL2_G_ODR_G_417HZ                    0x06
#define ASM330LHH_CTRL2_G_ODR_G_833HZ                    0x07
#define ASM330LHH_CTRL2_G_ODR_G_1667HZ                   0x08
#define ASM330LHH_CTRL2_G_ODR_G_3333HZ                   0x09
#define ASM330LHH_CTRL2_G_ODR_G_6667HZ                   0x0a
#define ASM330LHH_CTRL2_G_FS_G_MASK                      0x0c
#define ASM330LHH_CTRL2_G_FS_G_SHIFT                     2
#define ASM330LHH_CTRL2_G_FS_G_250DPS                    0x00
#define ASM330LHH_CTRL2_G_FS_G_500DPS                    0x01
#define ASM330LHH_CTRL2_G_FS_G_1000DPS                   0x02
#define ASM330LHH_CTRL2_G_FS_G_2000DPS                   0x03
#define ASM330LHH_CTRL2_G_FS_125                         0x02
#define ASM330LHH_CTRL2_G_FS_4000                        0x01

// Control register 3 (R/W)
#define ASM330LHH_CTRL3_C                   0x12
#define ASM330LHH_CTRL3_C_BOOT                           0x80
#define ASM330LHH_CTRL3_C_BDU                            0x40
#define ASM330LHH_CTRL3_C_H_LACTIVE                      0x20
#define ASM330LHH_CTRL3_C_PP_OD                          0x10
#define ASM330LHH_CTRL3_C_SIM                            0x08
#define ASM330LHH_CTRL3_C_IF_INC                         0x04
#define ASM330LHH_CTRL3_C_SW_RESET                       0x01

// Control register 4 (R/W)
#define ASM330LHH_CTRL4_C                   0x13
#define ASM330LHH_CTRL4_C_SLEEP_G                        0x40
#define ASM330LHH_CTRL4_C_INT2_ON_INT1                   0x20
#define ASM330LHH_CTRL4_C_DRDY_MASK                      0x08
#define ASM330LHH_CTRL4_C_I2C_DISABLE                    0x04
#define ASM330LHH_CTRL4_C_LPF1_SEL_G                     0x02

// Control register 5 (R/W)
#define ASM330LHH_CTRL5_C                   0x14
#define ASM330LHH_CTRL5_C_ROUNDING_MASK                  0x60
#define ASM330LHH_CTRL5_C_ROUNDING_SHIFT                 5
#define ASM330LHH_CTRL5_C_ST_G_MASK                      0x0c
#define ASM330LHH_CTRL5_C_ST_G_SHIFT                     2
#define ASM330LHH_CTRL5_C_ST_XL_MASK                     0x03
#define ASM330LHH_CTRL5_C_ST_XL_SHIFT                    0

// Control register 6 (R/W)
#define ASM330LHH_CTRL6_C                   0x15
#define ASM330LHH_CTRL6_C_TRIG_EN                        0x80
#define ASM330LHH_CTRL6_C_LVL1_EN                        0x40
#define ASM330LHH_CTRL6_C_LVL2_EN                        0x20
#define ASM330LHH_CTRL6_C_USR_OFF_W                      0x08
#define ASM330LHH_CTRL6_C_FTYPE_MASK                     0x07
#define ASM330LHH_CTRL6_C_FTYPE_SHIFT                    0

// Control register 7 (R/W)
#define ASM330LHH_CTRL7_G                   0x16
#define ASM330LHH_CTRL7_G_HP_EN_G                        0x40
#define ASM330LHH_CTRL7_G_HPM_G_MASK                     0x30
#define ASM330LHH_CTRL7_G_HPM_G_SHIFT                    4
#define ASM330LHH_CTRL7_G_USR_OFF_ON_OUT                 0x02

// Control register 8 (R/W)
#define ASM330LHH_CTRL8_XL                  0x17
#define ASM330LHH_CTRL8_XL_HPCF_XL_MASK                  0xe0
#define ASM330LHH_CTRL8_XL_HPCF_XL_SHIFT                 5
#define ASM330LHH_CTRL8_XL_HPCF_XL_ODR_DIV_4              0
#define ASM330LHH_CTRL8_XL_HPCF_XL_ODR_DIV_10             1
#define ASM330LHH_CTRL8_XL_HPCF_XL_ODR_DIV_20             2
#define ASM330LHH_CTRL8_XL_HPCF_XL_ODR_DIV_45             3
#define ASM330LHH_CTRL8_XL_HPCF_XL_ODR_DIV_100            4
#define ASM330LHH_CTRL8_XL_HPCF_XL_ODR_DIV_200             5
#define ASM330LHH_CTRL8_XL_HPCF_XL_ODR_DIV_400            6
#define ASM330LHH_CTRL8_XL_HPCF_XL_ODR_DIV_800            7
#define ASM330LHH_CTRL8_XL_HP_REF_MODE_XL                0x10
#define ASM330LHH_CTRL8_XL_FASTSETTL_MODE_XL             0x08
#define ASM330LHH_CTRL8_XL_HP_SLOPE_XL_EN                0x04
#define ASM330LHH_CTRL8_XL_LOW_PASS_ON_6D                0x01

// Control register 9 (R/W)
#define ASM330LHH_CTRL9_XL                  0x18
#define ASM330LHH_CTRL9_XL_DEN_X                         0x80
#define ASM330LHH_CTRL9_XL_DEN_Y                         0x40
#define ASM330LHH_CTRL9_XL_DEN_Z                         0x20
#define ASM330LHH_CTRL9_XL_DEN_XL_G                      0x10
#define ASM330LHH_CTRL9_XL_DEN_XL_EN                     0x08
#define ASM330LHH_CTRL9_XL_DEN_LH                        0x04
#define ASM330LHH_CTRL9_XL_DEVICE_CONF                   0x02

// Control register 10 (R/W)
#define ASM330LHH_CTRL10_C                  0x19
#define ASM330LHH_CTRL10_C_TIMESTAMP_EN                  0x20

// Source register for all interrupts (R)
#define ASM330LHH_ALL_INT_SRC               0x1A
#define ASM330LHH_ALL_INT_SRC_TIMESTAMP_ENDCOUNT         0x80
#define ASM330LHH_ALL_INT_SRC_SLEEP_CHANGE_IA            0x20
#define ASM330LHH_ALL_INT_SRC_D6D_IA                     0x10
#define ASM330LHH_ALL_INT_SRC_WU_IA                      0x02
#define ASM330LHH_ALL_INT_SRC_FF_IA                      0x01

// Wake-up interrupt source register (R)
#define ASM330LHH_WAKE_UP_SRC               0x1B
#define ASM330LHH_WAKE_UP_SRC_SLEEP_CHANGE_IA            0x40
#define ASM330LHH_WAKE_UP_SRC_FF_IA                      0x20
#define ASM330LHH_WAKE_UP_SRC_SLEEP_STATE                0x10
#define ASM330LHH_WAKE_UP_SRC_WU_IA                      0x08

// Portrait, landscape, face-up and face-down source register (R)
#define ASM330LHH_D6D_SRC                   0x1D
#define ASM330LHH_D6D_SRC_DEN_DRDY                       0x80
#define ASM330LHH_D6D_SRC_D6D_IA                         0x40

// Status register (R)
#define ASM330LHH_STATUS_REG                0x1E
#define ASM330LHH_STATUS_REG_TDA                         0x04
#define ASM330LHH_STATUS_REG_GDA                         0x02
#define ASM330LHH_STATUS_REG_XLDA                        0x01

// Temperature data output register (R)
#define ASM330LHH_OUT_TEMP_L                0x20
#define ASM330LHH_OUT_TEMP_H                0x21

// Angular rate sensor pitch axis (X) angular rate output register (R)
#define ASM330LHH_OUTX_L_G                  0x22
#define ASM330LHH_OUTX_H_G                  0x23

// Angular rate sensor roll axis (Y) angular rate output register (R)
#define ASM330LHH_OUTY_L_G                  0x24
#define ASM330LHH_OUTY_H_G                  0x25

// Angular rate sensor yaw axis (Z) angular rate output register (R)
#define ASM330LHH_OUTZ_L_G                  0x26
#define ASM330LHH_OUTZ_H_G                  0x27

// Linear acceleration sensor X-axis output register (R)
#define ASM330LHH_OUTX_L_A                  0x28
#define ASM330LHH_OUTX_H_A                  0x29

// Linear acceleration sensor Y-axis output register (R)
#define ASM330LHH_OUTY_L_A                  0x2A
#define ASM330LHH_OUTY_H_A                  0x2B

// Linear acceleration sensor Z-axis output register (R)
#define ASM330LHH_OUTZ_L_A                  0x2C
#define ASM330LHH_OUTZ_H_A                  0x2D

// FIFO status register 1 (R)
#define ASM330LHH_FIFO_STATUS1              0x3A

// FIFO status register 2 (R)
#define ASM330LHH_FIFO_STATUS2              0x3B
#define ASM330LHH_FIFO_STATUS2_FIFO_WTM_IA               0x80
#define ASM330LHH_FIFO_STATUS2_FIFO_OVR_IA               0x40
#define ASM330LHH_FIFO_STATUS2_FIFO_FULL_IA              0x20
#define ASM330LHH_FIFO_STATUS2_COUNTER_BDR_IA            0x10
#define ASM330LHH_FIFO_STATUS2_FIFO_OVR_LATCHED          0x08

// Timestamp output registers (R)
#define ASM330LHH_TIMESTAMP0                0x40
#define ASM330LHH_TIMESTAMP1                0x41
#define ASM330LHH_TIMESTAMP2                0x42
#define ASM330LHH_TIMESTAMP3                0x43

// Interrupt/filtering/latch configuration register (R/W)
#define ASM330LHH_INT_CFG0                  0x56
#define ASM330LHH_INT_CFG0_INT_CLR_ON_READ                0x40
#define ASM330LHH_INT_CFG0_SLEEP_STATUS_ON_INT            0x20
#define ASM330LHH_INT_CFG0_SLOPE_FDS                      0x10
#define ASM330LHH_INT_CFG0_LIR                            0x01

// Enables interrupt function register (R/W)
#define ASM330LHH_INT_CFG1                  0x58
#define ASM330LHH_INT_CFG1_INTERRUPTS_ENABLE              0x80
#define ASM330LHH_INT_CFG1_INACT_EN_MASK                  0x60
#define ASM330LHH_INT_CFG1_INACT_EN_SHIFT                 5

// Portrait/landscape position register (R/W)
#define ASM330LHH_THS_6D                    0x59
#define ASM330LHH_THS_6D_D4D_EN                           0x80
#define ASM330LHH_THS_6D_SIXD_THS_MASK                    0x60
#define ASM330LHH_THS_6D_SIXD_THS_SHIFT                   5

// Wake-up configuration register (R/W)
#define ASM330LHH_WAKE_UP_THS                0x5B
#define ASM330LHH_WAKE_UP_THS_USR_OFF_ON_WU                0x40
#define ASM330LHH_WAKE_UP_THS_WK_THS_MASK                  0x3f
#define ASM330LHH_WAKE_UP_THS_WK_THS_SHIFT                 0

// Free-fall, wake-up and sleep mode functions duration setting register (R/W)
#define ASM330LHH_WAKE_UP_DUR                0x5C
#define ASM330LHH_WAKE_UP_DUR_FF_DUR5                      0x80
#define ASM330LHH_WAKE_UP_DUR_WAKE_DUR_MASK                0x60
#define ASM330LHH_WAKE_UP_DUR_WAKE_DUR_SHIFT               5
#define ASM330LHH_WAKE_UP_DUR_WAKE_THS_W                   0x10
#define ASM330LHH_WAKE_UP_DUR_SLEEP_DUR_MASK                0x0f
#define ASM330LHH_WAKE_UP_DUR_SLEEP_DUR_SHIFT               0

// Free-fall function duration setting register (R/W)
#define ASM330LHH_FREE_FALL                  0x5D
#define ASM330LHH_FREE_FALL_FF_DUR_MASK                     0xf8
#define ASM330LHH_FREE_FALL_FF_DUR_SHIFT                    3
#define ASM330LHH_FREE_FALL_FF_THS_MASK                     0x07
#define ASM330LHH_FREE_FALL_FF_THS_SHIFT                    0

// Functions routing on INT1 register (R/W)
#define ASM330LHH_MD1_CFG                    0x5E
#define ASM330LHH_MD1_CFG_INT1_SLEEP_CHANGE                 0x80
#define ASM330LHH_MD1_CFG_INT1_WU                           0x40
#define ASM330LHH_MD1_CFG_INT1_FF                           0x20
#define ASM330LHH_MD1_CFG_INT1_6D                           0x08

// Functions routing on INT2 register (R/W)
#define ASM330LHH_MD2_CFG                    0x5F
#define ASM330LHH_MD2_CFG_INT2_SLEEP_CHANGE                 0x80
#define ASM330LHH_MD2_CFG_INT2_WU                           0x40
#define ASM330LHH_MD2_CFG_INT2_FF                           0x20
#define ASM330LHH_MD2_CFG_INT2_6D                           0x08
#define ASM330LHH_MD2_CFG_INT2_TIMESTAMP                    0x01

// Internal frequency register (R)
#define ASM330LHH_INTERNAL_FREQ_FINE          0x63

// Accelerometer user offset correction registers (R/W)
#define ASM330LHH_X_OFS_USR                  0x73
#define ASM330LHH_Y_OFS_USR                   0x74
#define ASM330LHH_Z_OFS_USR                   0x75

// FIFO tag register (R)
#define ASM330LHH_FIFO_DATA_OUT_TAG           0x78
#define ASM330LHH_FIFO_DATA_OUT_TAG_SENSOR_MASK             0xf8
#define ASM330LHH_FIFO_DATA_OUT_TAG_SENSOR_SHIFT            3
#define ASM330LHH_FIFO_DATA_OUT_TAG_SENSOR_GYRO              0x01
#define ASM330LHH_FIFO_DATA_OUT_TAG_SENSOR_ACC               0x02
#define ASM330LHH_FIFO_DATA_OUT_TAG_SENSOR_TEMP              0x03
#define ASM330LHH_FIFO_DATA_OUT_TAG_SENSOR_TIMESTAMP         0x04
#define ASM330LHH_FIFO_DATA_OUT_TAG_SENSOR_CFG_CHANGE        0x05

// FIFO data output X/Y/Z (R)
#define ASM330LHH_FIFO_DATA_OUT_X_L            0x79
#define ASM330LHH_FIFO_DATA_OUT_X_H            0x7A
#define ASM330LHH_FIFO_DATA_OUT_Y_L            0x7B
#define ASM330LHH_FIFO_DATA_OUT_Y_H            0x7C
#define ASM330LHH_FIFO_DATA_OUT_Z_L            0x7D
#define ASM330LHH_FIFO_DATA_OUT_Z_H            0x7E

#define ASM330LHH_WHO_AM_I_CONST              (0x6B)

uint8_t asm330lhhSpiDetect(const extDevice_t *dev)
{
    const uint8_t whoAmI = spiReadRegMsk(dev, ASM330LHH_WHO_AM_I);

    if (whoAmI != ASM330LHH_WHO_AM_I_CONST) {
        return MPU_NONE;
    }

    return ASM330LHH_SPI;
}

static void asm330lhhAccInit(accDev_t *acc)
{
    // ±16G mode
    acc->acc_1G = 512 * 4;
}

static bool asm330lhhAccReadSPI(accDev_t *acc)
{
    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        acc->gyro->dev.txBuf[0] = acc->gyro->accDataReg | 0x80;

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

        // The ASM330LHH stores gyro data before accel data in the burst, so compute the accel offset from the
        // start of the burst instead of hard-coding the array index.
        const uint8_t accDataIndex = ((acc->gyro->accDataReg - acc->gyro->dmaReadRegStart) >> 1) + 1;
        int16_t *accData = (int16_t *)acc->gyro->dev.rxBuf;

        acc->ADCRaw[X] = accData[accDataIndex];
        acc->ADCRaw[Y] = accData[accDataIndex + 1];
        acc->ADCRaw[Z] = accData[accDataIndex + 2];
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

bool asm330lhhSpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != ASM330LHH_SPI) {
        return false;
    }

    acc->initFn = asm330lhhAccInit;
    acc->readFn = asm330lhhAccReadSPI;

    return true;
}

static void asm330lhhGyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;
    // Set default LPF1 filter bandwidth to be as close as possible to MPU6000's 250Hz cutoff
    // Bandwidth values are for a gyro ODR of 6667Hz, see Table 53 of the datasheet
    uint8_t asm330lhhLPF1BandwidthOptions[GYRO_HARDWARE_LPF_COUNT] = {
            [GYRO_HARDWARE_LPF_NORMAL] = 0, // 297Hz
            [GYRO_HARDWARE_LPF_OPTION_1] = 2, // 154Hz
            [GYRO_HARDWARE_LPF_OPTION_2] = 1, // 223Hz
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
            [GYRO_HARDWARE_LPF_EXPERIMENTAL] = 3, // 470Hz
#endif
    };

    spiSetClkDivisor(dev, spiCalculateDivider(ASM330LHH_MAX_SPI_CLK_HZ));

    // Perform a software reset
    spiWriteReg(dev, ASM330LHH_CTRL3_C, ASM330LHH_CTRL3_C_SW_RESET);

    // Wait for the device to be ready
    while (spiReadRegMsk(dev, ASM330LHH_CTRL3_C) & ASM330LHH_CTRL3_C_SW_RESET) {}

    // Autoincrement register address when doing block SPI reads and update continuously
    spiWriteReg(dev, ASM330LHH_CTRL3_C, ASM330LHH_CTRL3_C_IF_INC | ASM330LHH_CTRL3_C_BDU);

    // Disable I2C and confirm the device configuration, as recommended in the application hints
    spiWriteReg(dev, ASM330LHH_CTRL9_XL, ASM330LHH_CTRL9_XL_DEVICE_CONF);
    spiWriteReg(dev, ASM330LHH_CTRL4_C, ASM330LHH_CTRL4_C_I2C_DISABLE | ASM330LHH_CTRL4_C_LPF1_SEL_G);

    // Select the accelerometer LPF2 bandwidth of ODR/4
    spiWriteReg(dev, ASM330LHH_CTRL8_XL,
                ASM330LHH_ENCODE_BITS(ASM330LHH_CTRL8_XL_HPCF_XL_ODR_DIV_4,
                                      ASM330LHH_CTRL8_XL_HPCF_XL_MASK,
                                      ASM330LHH_CTRL8_XL_HPCF_XL_SHIFT));

    // Enable 16G sensitivity, the accelerometer composite (LPF2) filter and the accelerometer odr at 1667Hz
    spiWriteReg(dev, ASM330LHH_CTRL1_XL,
                ASM330LHH_ENCODE_BITS(ASM330LHH_CTRL1_XL_ODR_XL_1667HZ,
                                      ASM330LHH_CTRL1_XL_ODR_XL_MASK,
                                      ASM330LHH_CTRL1_XL_ODR_XL_SHIFT) |
                ASM330LHH_ENCODE_BITS(ASM330LHH_CTRL1_XL_FS_XL_16G,
                                      ASM330LHH_CTRL1_XL_FS_XL_MASK,
                                      ASM330LHH_CTRL1_XL_FS_XL_SHIFT) |
                ASM330LHH_CTRL1_XL_LPF2_XL_EN);

    // Select the gyro LPF1 filter setting
    spiWriteReg(dev, ASM330LHH_CTRL6_C,
                ASM330LHH_ENCODE_BITS(asm330lhhLPF1BandwidthOptions[gyroConfig()->gyro_hardware_lpf],
                                      ASM330LHH_CTRL6_C_FTYPE_MASK,
                                      ASM330LHH_CTRL6_C_FTYPE_SHIFT));

    // Enable 2000 deg/s sensitivity and the gyro odr at 6667Hz
    spiWriteReg(dev, ASM330LHH_CTRL2_G,
                ASM330LHH_ENCODE_BITS(ASM330LHH_CTRL2_G_ODR_G_6667HZ,
                                      ASM330LHH_CTRL2_G_ODR_G_MASK,
                                      ASM330LHH_CTRL2_G_ODR_G_SHIFT) |
                ASM330LHH_ENCODE_BITS(ASM330LHH_CTRL2_G_FS_G_2000DPS,
                                      ASM330LHH_CTRL2_G_FS_G_MASK,
                                      ASM330LHH_CTRL2_G_FS_G_SHIFT));

    // Generate a pulse on the interrupt line, not requiring a read to clear
    spiWriteReg(dev, ASM330LHH_COUNTER_BDR_REG1, ASM330LHH_COUNTER_BDR_REG1_DATAREADY_PULSED);

    // From section 4.1, Mechanical characteristics, of the datasheet, G_So is 70mdps/LSB for FS = ±2000 dps.
    gyro->scale = 0.070f;

    // Enable the INT1 output to interrupt when new gyro data is ready
    spiWriteReg(dev, ASM330LHH_INT1_CTRL, ASM330LHH_INT1_CTRL_INT1_DRDY_G);

    mpuGyroInit(gyro);
    gyro->accDataReg = ASM330LHH_OUTX_L_A;
    gyro->gyroDataReg = ASM330LHH_OUTX_L_G;
    gyro->tempDataReg = ASM330LHH_OUT_TEMP_L;
    gyro->dmaReadRegStart = gyro->gyroDataReg;
}

static bool asm330lhhGyroReadSPI(gyroDev_t *gyro)
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
#ifdef USE_DMA
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uintptr_t)gyro;
                gyro->dev.txBuf[0] = gyro->dmaReadRegStart | 0x80;
                // Read three words of gyro data immediately followed by three bytes of acc data
                gyro->segments[0].len = sizeof(uint8_t) + 6 * sizeof(int16_t);
                gyro->segments[0].callback = mpuIntCallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                gyro->segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else
#endif
            {
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
        gyro->dev.txBuf[0] = gyro->dmaReadRegStart | 0x80;

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
        const uint8_t gyroDataIndex = ((gyro->gyroDataReg - gyro->dmaReadRegStart) >> 1) + 1;
        gyro->gyroADCRaw[X] = gyroData[gyroDataIndex];
        gyro->gyroADCRaw[Y] = gyroData[gyroDataIndex + 1];
        gyro->gyroADCRaw[Z] = gyroData[gyroDataIndex + 2];
        break;
    }

    default:
        break;
    }

    return true;
}

bool asm330lhhSpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != ASM330LHH_SPI) {
        return false;
    }

    gyro->initFn = asm330lhhGyroInit;
    gyro->readFn = asm330lhhGyroReadSPI;

    return true;
}

#endif // USE_ACCGYRO_ASM330LHH
