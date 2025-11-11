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

#if defined(USE_ACCGYRO_LSM6DSV16X)

#include "accgyro_spi_lsm6dsv16x.h"

#include "sensors/gyro.h"

/* See datasheet
 *
 *     https://www.st.com/content/ccc/resource/technical/document/datasheet/group3/47/03/b2/44/47/32/4b/76/DM00741844/files/DM00741844.pdf/jcr:content/translations/en.DM00741844.pdf
 *
 */

// 10 MHz max SPI frequency
#define LSM6DSV16X_MAX_SPI_CLK_HZ 10000000

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// Macros to encode/decode multi-bit values
#define LSM6DSV_ENCODE_BITS(val, mask, shift)   ((val << shift) & mask)
#define LSM6DSV_DECODE_BITS(val, mask, shift)   ((val & mask) >> shift)

// Enable embedded functions register (R/W)
#define LSM6DSV_FUNC_CFG_ACCESS             0x01
#define LSM6DSV_EMB_FUNC_REG_ACCESS_EMB_FUNC_REG_ACCESS 0x80
#define LSM6DSV_EMB_FUNC_REG_ACCESS_SHUB_REG_ACCESS     0x40
#define LSM6DSV_EMB_FUNC_REG_ACCESS_FSM_WR_CTRL_EN      0x08
#define LSM6DSV_EMB_FUNC_REG_ACCESS_SW_POR              0x04
#define LSM6DSV_EMB_FUNC_REG_ACCESS_SPI2_RESET          0x02
#define LSM6DSV_EMB_FUNC_REG_ACCESS_OIS_CTRL_FROM_UI    0x01

// SDO, OCS_Aux, SDO_Aux pins pull-up register (R/W)
#define LSM6DSV_PIN_CTRL                    0x02
#define LSM6DSV_PIN_CTRL_OIRS_PU_DIS                    0x80
#define LSM6DSV_PIN_CTRL_DSO_PU_EN                      0x40
#define LSM6DSV_PIN_CTRL_IBHR_POR_EN                    0x20
#define LSM6DSV_PIN_CTRL_RESDV                          0x03

// Interface configuration register (R/W)
#define LSM6DSV_IF_CFG                      0x03
#define LSM6DSV_IF_CFG_SDA_PU_EN                        0x80
#define LSM6DSV_IF_CFG_SHUB_PU_EN                       0x40
#define LSM6DSV_IF_CFG_ASF_CTRL                         0x20
#define LSM6DSV_IF_CFG_H_LACTIVE                        0x10
#define LSM6DSV_IF_CFG_PP_OD                            0x08
#define LSM6DSV_IF_CFG_SIM                              0x04
#define LSM6DSV_IF_CFG_I2C_I3C_DISABLE                  0x01

// ODR-triggered mode configuration register (R/W)
#define LSM6DSV_ODR_TRIG_CFG                0x06

// FIFO control register 1 (R/W)
#define LSM6DSV_FIFO_CTRL1                  0x07

// FIFO control register 2 (R/W)
#define LSM6DSV_FIFO_CTRL2                  0x08
#define LSM6DSV_FIFO_CTRL2_STOP_ON_WTM                  0x80
#define LSM6DSV_FIFO_CTRL2_FIFO_COMPR_RT_EN             0x40
#define LSM6DSV_FIFO_CTRL2_ODR_CHG_EN                   0x10
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_MASK            0x06
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_SHIFT           1
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_NOT_FORCED      0
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_8               1
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_16              2
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_32              3
#define LSM6DSV_FIFO_CTRL2_XL_DUALC_BATCH_FROM_FS       0x01

// FIFO control register 3 (R/W)
#define LSM6DSV_FIFO_CTRL3                  0x09
#define LSM6DSV_FIFO_CTRL3_BDR_GY_MASK                  0xf0
#define LSM6DSV_FIFO_CTRL3_BDR_GY_SHIFT                 4
#define LSM6DSV_FIFO_CTRL3_BDR_GY_1875HZ                0x01
#define LSM6DSV_FIFO_CTRL3_BDR_GY_7_5HZ                 0x02
#define LSM6DSV_FIFO_CTRL3_BDR_GY_15HZ                  0x03
#define LSM6DSV_FIFO_CTRL3_BDR_GY_30HZ                  0x04
#define LSM6DSV_FIFO_CTRL3_BDR_GY_60HZ                  0x05
#define LSM6DSV_FIFO_CTRL3_BDR_GY_120HZ                 0x06
#define LSM6DSV_FIFO_CTRL3_BDR_GY_240HZ                 0x07
#define LSM6DSV_FIFO_CTRL3_BDR_GY_480HZ                 0x08
#define LSM6DSV_FIFO_CTRL3_BDR_GY_960HZ                 0x09
#define LSM6DSV_FIFO_CTRL3_BDR_GY_1920HZ                0x0a
#define LSM6DSV_FIFO_CTRL3_BDR_GY_3840HZ                0x0b
#define LSM6DSV_FIFO_CTRL3_BDR_GY_7680HZ                0x0c
#define LSM6DSV_FIFO_CTRL3_BDR_XL_MASK                  0xff
#define LSM6DSV_FIFO_CTRL3_BDR_XL_SHIFT                 0
#define LSM6DSV_FIFO_CTRL3_BDR_XL_1875HZ                0x01
#define LSM6DSV_FIFO_CTRL3_BDR_XL_7_5HZ                 0x02
#define LSM6DSV_FIFO_CTRL3_BDR_XL_15HZ                  0x03
#define LSM6DSV_FIFO_CTRL3_BDR_XL_30HZ                  0x04
#define LSM6DSV_FIFO_CTRL3_BDR_XL_60HZ                  0x05
#define LSM6DSV_FIFO_CTRL3_BDR_XL_120HZ                 0x06
#define LSM6DSV_FIFO_CTRL3_BDR_XL_240HZ                 0x07
#define LSM6DSV_FIFO_CTRL3_BDR_XL_480HZ                 0x08
#define LSM6DSV_FIFO_CTRL3_BDR_XL_960HZ                 0x09
#define LSM6DSV_FIFO_CTRL3_BDR_XL_1920HZ                0x0a
#define LSM6DSV_FIFO_CTRL3_BDR_XL_3840HZ                0x0b
#define LSM6DSV_FIFO_CTRL3_BDR_XL_7680HZ                0x0c

// FIFO control register 4 (R/W)
#define LSM6DSV_FIFO_CTRL4                  0x0A
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_MASK            0xc0
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_SHIFT           6
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_NONE            0
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_DECIMATION_1    1
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_DECIMATION_8    2
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_DECIMATION_32   3
#define LSM6DSV_FIFO_CTRL4_ODR_T_BATCH_MASK             0x30
#define LSM6DSV_FIFO_CTRL4_ODR_T_BATCH_SHIFT            4
#define LSM6DSV_FIFO_CTRL4_ODR_T_BATCH_NONE             0
#define LSM6DSV_FIFO_CTRL4_ODR_T_BATCH_1875HZ           1
#define LSM6DSV_FIFO_CTRL4_ODR_T_BATCH_15HZ             2
#define LSM6DSV_FIFO_CTRL4_ODR_T_BATCH_60HZ             3
#define LSM6DSV_FIFO_CTRL4_G_EIS_FIFO_EN                0x08
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_MASK               0x07
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_SHIFT              0
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_BYPASS             0
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_FIFO_MODE          1
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_CONT_WTM           2
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_CONT_FIFO          3
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_BYPASS_CONT        4
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_CONT               6
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_BYPASS_FIFO        7

// Counter batch data rate register 1 (R/W)
#define LSM6DSV_COUNTER_BDR_REG1            0x0B
#define LSM6DSV_COUNTER_BDR_REG1_TRIG_COUNTER_BDR_MASK      0x60
#define LSM6DSV_COUNTER_BDR_REG1_TRIG_COUNTER_BDR_SHIFT     5
#define LSM6DSV_COUNTER_BDR_REG1_TRIG_COUNTER_BDR_ACC       0
#define LSM6DSV_COUNTER_BDR_REG1_TRIG_COUNTER_BDR_GYRO      1
#define LSM6DSV_COUNTER_BDR_REG1_TRIG_COUNTER_BDR_GYRO_EIS  2
#define LSM6DSV_COUNTER_BDR_REG2_CNT_BDR_TH_HI_MASK         0x03
#define LSM6DSV_COUNTER_BDR_REG2_CNT_BDR_TH_HI_SHIFT        0

// Counter batch data rate register 2 (R/W)
#define LSM6DSV_COUNTER_BDR_REG2            0x0C

// INT1 pin control register (R/W)
#define LSM6DSV_INT1_CTRL                   0x0D
#define LSM6DSV_INT1_CTRL_INT1_CNT_BDR                  0x40
#define LSM6DSV_INT1_CTRL_INT1_FIFO_FULL                0x20
#define LSM6DSV_INT1_CTRL_INT1_FIFO_OVR                 0x10
#define LSM6DSV_INT1_CTRL_INT1_FIFO_TH                  0x08
#define LSM6DSV_INT1_CTRL_INT1_DRDY_G                   0x02
#define LSM6DSV_INT1_CTRL_INT1_DRDY_XL                  0x01

// INT2 pin control register (R/W)
#define LSM6DSV_INT2_CTRL                   0x0E
#define LSM6DSV_INT2_CTRL_INT2_EMB_FUNC_ENDOP           0x80
#define LSM6DSV_INT2_CTRL_INT2_CNT_BDR                  0x40
#define LSM6DSV_INT2_CTRL_INT2_FIFO_FULL                0x20
#define LSM6DSV_INT2_CTRL_INT2_FIFO_OVR                 0x10
#define LSM6DSV_INT2_CTRL_INT2_FIFO_TH                  0x08
#define LSM6DSV_INT2_CTRL_INT2_DRDY_G_EIS               0x04
#define LSM6DSV_INT2_CTRL_INT2_DRDY_G                   0x02
#define LSM6DSV_INT2_CTRL_INT2_DRDY_XL                  0x01

// WHO_AM_I register (R)
#define LSM6DSV_WHO_AM_I                    0x0F

// Accelerometer control register 1 (R/W)
#define LSM6DSV_CTRL1                       0x10
#define LSM6DSV_CTRL1_OP_MODE_XL_MASK                   0x70
#define LSM6DSV_CTRL1_OP_MODE_XL_SHIFT                  4
#define LSM6DSV_CTRL1_OP_MODE_XL_HIGH_PERF              0
#define LSM6DSV_CTRL1_OP_MODE_XL_HIGH_ACCURACY          1
#define LSM6DSV_CTRL1_OP_MODE_XL_ODR_TRIG               3
#define LSM6DSV_CTRL1_OP_MODE_XL_LOW_PWR_MODE1          4
#define LSM6DSV_CTRL1_OP_MODE_XL_LOW_PWR_MODE2          5
#define LSM6DSV_CTRL1_OP_MODE_XL_LOW_PWR_MODE3          6
#define LSM6DSV_CTRL1_OP_MODE_XL_NORMAL                 7
#define LSM6DSV_CTRL1_ODR_XL_MASK                       0x0f
#define LSM6DSV_CTRL1_ODR_XL_SHIFT                      0
#define LSM6DSV_CTRL1_ODR_XL_POWERDOWN                  0
#define LSM6DSV_CTRL1_ODR_XL_1875HZ                     1
#define LSM6DSV_CTRL1_ODR_XL_7_5HZ                      2

// Values to use in high-accuracy ODR mode with HAODR_SEL[] = 00
// or in high-performance mode mode
#define LSM6DSV_CTRL1_ODR_XL_15HZ                       3
#define LSM6DSV_CTRL1_ODR_XL_30HZ                       4
#define LSM6DSV_CTRL1_ODR_XL_60HZ                       5
#define LSM6DSV_CTRL1_ODR_XL_120HZ                      6
#define LSM6DSV_CTRL1_ODR_XL_240HZ                      7
#define LSM6DSV_CTRL1_ODR_XL_480HZ                      8
#define LSM6DSV_CTRL1_ODR_XL_960HZ                      9
#define LSM6DSV_CTRL1_ODR_XL_1920HZ                     10
#define LSM6DSV_CTRL1_ODR_XL_3840HZ                     11
#define LSM6DSV_CTRL1_ODR_XL_7680HZ                     12

// Values to use in high-accuracy ODR mode with HAODR_SEL[] = 01
#define LSM6DSV_CTRL1_ODR_XL_15_625HZ                   3
#define LSM6DSV_CTRL1_ODR_XL_31_25HZ                    4
#define LSM6DSV_CTRL1_ODR_XL_62_5HZ                     5
#define LSM6DSV_CTRL1_ODR_XL_125HZ                      6
#define LSM6DSV_CTRL1_ODR_XL_250HZ                      7
#define LSM6DSV_CTRL1_ODR_XL_500HZ                      8
#define LSM6DSV_CTRL1_ODR_XL_1000HZ                     9
#define LSM6DSV_CTRL1_ODR_XL_2000HZ                     10
#define LSM6DSV_CTRL1_ODR_XL_4000HZ                     11
#define LSM6DSV_CTRL1_ODR_XL_8000HZ                     12

// Values to use in high-accuracy ODR mode with HAODR_SEL[] = 10
#define LSM6DSV_CTRL1_ODR_XL_12_5HZ                     3
#define LSM6DSV_CTRL1_ODR_XL_25HZ                       4
#define LSM6DSV_CTRL1_ODR_XL_50HZ                       5
#define LSM6DSV_CTRL1_ODR_XL_100HZ                      6
#define LSM6DSV_CTRL1_ODR_XL_200HZ                      7
#define LSM6DSV_CTRL1_ODR_XL_400HZ                      8
#define LSM6DSV_CTRL1_ODR_XL_800HZ                     9
#define LSM6DSV_CTRL1_ODR_XL_1600HZ                     10
#define LSM6DSV_CTRL1_ODR_XL_3200HZ                     11
#define LSM6DSV_CTRL1_ODR_XL_6400HZ                     12

// Gyroscope control register 2 (R/W)
#define LSM6DSV_CTRL2                       0x11
#define LSM6DSV_CTRL2_OP_MODE_G_MASK                    0x70
#define LSM6DSV_CTRL2_OP_MODE_G_SHIFT                   4
#define LSM6DSV_CTRL2_OP_MODE_G_HIGH_PERF               0
#define LSM6DSV_CTRL2_OP_MODE_G_HIGH_ACCURACY           1
#define LSM6DSV_CTRL2_OP_MODE_G_ODR_TRIG                3
#define LSM6DSV_CTRL2_OP_MODE_G_SLEEP                   4
#define LSM6DSV_CTRL2_OP_MODE_G_LOW_PWR_MODE            5
#define LSM6DSV_CTRL2_ODR_G_MASK                        0x0f
#define LSM6DSV_CTRL2_ODR_G_SHIFT                       0
#define LSM6DSV_CTRL2_ODR_G_POWERDOWN                   0
#define LSM6DSV_CTRL2_ODR_G_1875HZ                      1
#define LSM6DSV_CTRL2_ODR_G_7_5HZ                       2

// Values to use in high-accuracy ODR mode with HAODR_SEL[] = 00
// or in high-performance mode mode
#define LSM6DSV_CTRL2_ODR_G_15HZ                        3
#define LSM6DSV_CTRL2_ODR_G_30HZ                        4
#define LSM6DSV_CTRL2_ODR_G_60HZ                        5
#define LSM6DSV_CTRL2_ODR_G_120HZ                       6
#define LSM6DSV_CTRL2_ODR_G_240HZ                       7
#define LSM6DSV_CTRL2_ODR_G_480HZ                       8
#define LSM6DSV_CTRL2_ODR_G_960HZ                       9
#define LSM6DSV_CTRL2_ODR_G_1920HZ                      10
#define LSM6DSV_CTRL2_ODR_G_3840HZ                      11
#define LSM6DSV_CTRL2_ODR_G_7680HZ                      12

// Values to use in high-accuracy ODR mode with HAODR_SEL[] = 01
#define LSM6DSV_CTRL2_ODR_G_15_625HZ                    3
#define LSM6DSV_CTRL2_ODR_G_31_25HZ                     4
#define LSM6DSV_CTRL2_ODR_G_62_5HZ                      5
#define LSM6DSV_CTRL2_ODR_G_125HZ                       6
#define LSM6DSV_CTRL2_ODR_G_250HZ                       7
#define LSM6DSV_CTRL2_ODR_G_500HZ                       8
#define LSM6DSV_CTRL2_ODR_G_1000HZ                      9
#define LSM6DSV_CTRL2_ODR_G_2000HZ                      10
#define LSM6DSV_CTRL2_ODR_G_4000HZ                      11
#define LSM6DSV_CTRL2_ODR_G_8000HZ                      12

// Values to use in high-accuracy ODR mode with HAODR_SEL[] = 10
#define LSM6DSV_CTRL2_ODR_G_12_5HZ                      3
#define LSM6DSV_CTRL2_ODR_G_25HZ                        4
#define LSM6DSV_CTRL2_ODR_G_50HZ                        5
#define LSM6DSV_CTRL2_ODR_G_100HZ                       6
#define LSM6DSV_CTRL2_ODR_G_200HZ                       7
#define LSM6DSV_CTRL2_ODR_G_400HZ                       8
#define LSM6DSV_CTRL2_ODR_G_800HZ                       9
#define LSM6DSV_CTRL2_ODR_G_1600HZ                      10
#define LSM6DSV_CTRL2_ODR_G_3200HZ                      11
#define LSM6DSV_CTRL2_ODR_G_6400HZ                      12

// Control register 3 (R/W)
#define LSM6DSV_CTRL3                       0x12
#define LSM6DSV_CTRL3_BOOT                              0x80
#define LSM6DSV_CTRL3_BDU                               0x40
#define LSM6DSV_CTRL3_IF_INC                            0x04
#define LSM6DSV_CTRL3_SW_RESET                          0x01

// Control register 4 (R/W)
#define LSM6DSV_CTRL4                       0x13
#define LSM6DSV_CTRL4_INT2_ON_INT1                      0x10
#define LSM6DSV_CTRL4_DRDY_MASK                         0x08
#define LSM6DSV_CTRL4_INT2_DRDY_TEMP                    0x04
#define LSM6DSV_CTRL4_DRDY_PULSED                       0x02
#define LSM6DSV_CTRL4_INT2_IN_LH                        0x01

 // Control register 5 (R/W)
#define LSM6DSV_CTRL5                       0x14
#define LSM6DSV_CTRL5_BUS_ACT_SEL_MASK                  0x06
#define LSM6DSV_CTRL5_BUS_ACT_SEL_SHIFT                 1
#define LSM6DSV_CTRL5_BUS_ACT_SEL_2US                   0
#define LSM6DSV_CTRL5_BUS_ACT_SEL_50US                  1
#define LSM6DSV_CTRL5_BUS_ACT_SEL_1US                   2
#define LSM6DSV_CTRL5_BUS_ACT_SEL_25US                  3
#define LSM6DSV_CTRL5_INT_EN_I3C                        0x01

// Control register 6 (R/W)
#define LSM6DSV_CTRL6                       0x15
#define LSM6DSV_CTRL6_LPF1_G_BW_MASK                    0x70 // See table 64
#define LSM6DSV_CTRL6_LPF1_G_BW_SHIFT                   4

// Gyro LPF1 + LPF2 bandwidth selection when ODR=7.68kHz
// Note that these figures were advised by STmicro tech support and differ from the datasheet
#define LSM6DSV_CTRL6_FS_G_BW_288HZ                     0
#define LSM6DSV_CTRL6_FS_G_BW_215HZ                     1
#define LSM6DSV_CTRL6_FS_G_BW_157HZ                     2
#define LSM6DSV_CTRL6_FS_G_BW_455HZ                     3
#define LSM6DSV_CTRL6_FS_G_BW_102HZ                     4
#define LSM6DSV_CTRL6_FS_G_BW_58HZ                      5
#define LSM6DSV_CTRL6_FS_G_BW_28_8HZ                    6
#define LSM6DSV_CTRL6_FS_G_BW_14_4HZ                    7

#define LSM6DSV_CTRL6_FS_G_MASK                         0x0f
#define LSM6DSV_CTRL6_FS_G_SHIFT                        0
#define LSM6DSV_CTRL6_FS_G_125DPS                       0x00
#define LSM6DSV_CTRL6_FS_G_250DPS                       0x01
#define LSM6DSV_CTRL6_FS_G_500DPS                       0x02
#define LSM6DSV_CTRL6_FS_G_1000DPS                      0x03
#define LSM6DSV_CTRL6_FS_G_2000DPS                      0x04
#define LSM6DSV_CTRL6_FS_G_4000DPS                      0xc0

// Control register 7 (R/W)
#define LSM6DSV_CTRL7                       0x16
#define LSM6DSV_CTRL7_AH_QVAR_EN                        0x80
#define LSM6DSV_CTRL7_INT2_DRDY_AH_QVAR                 0x40
#define LSM6DSV_CTRL7_AH_QVAR_C_ZIN_MASK                0x30
#define LSM6DSV_CTRL7_AH_QVAR_C_ZIN_SHIFT               4
#define LSM6DSV_CTRL7_AH_QVAR_C_ZIN_2_4G                0
#define LSM6DSV_CTRL7_AH_QVAR_C_ZIN_730M                1
#define LSM6DSV_CTRL7_AH_QVAR_C_ZIN_300M                2
#define LSM6DSV_CTRL7_AH_QVAR_C_ZIN_235M                3
#define LSM6DSV_CTRL7_LPF1_G_EN                         0x01

// Control register 8 (R/W)
#define LSM6DSV_CTRL8                       0x17
#define LSM6DSV_CTRL8_HP_LPF2_XL_BW_2_MASK              0xe0 // See table 69
#define LSM6DSV_CTRL8_HP_LPF2_XL_BW_2_SHIFT             5
#define LSM6DSV_CTRL8_XL_DUALC_EN                       0x08
#define LSM6DSV_CTRL8_FS_XL_MASK                        0x03
#define LSM6DSV_CTRL8_FS_XL_SHIFT                       0
#define LSM6DSV_CTRL8_FS_XL_2G                          0
#define LSM6DSV_CTRL8_FS_XL_4G                          1
#define LSM6DSV_CTRL8_FS_XL_8G                          2
#define LSM6DSV_CTRL8_FS_XL_16G                         3
#define LSM6DSV_CTRL8_FS_XL_BW_4                        0
#define LSM6DSV_CTRL8_FS_XL_BW_10                       1
#define LSM6DSV_CTRL8_FS_XL_BW_20                       2
#define LSM6DSV_CTRL8_FS_XL_BW_45                       3

// Control register 9 (R/W)
#define LSM6DSV_CTRL9                       0x18
#define LSM6DSV_CTRL9_HP_REF_MODE_XL                    0x40
#define LSM6DSV_CTRL9_XL_FASTSETTL_MODE                 0x20
#define LSM6DSV_CTRL9_HP_SLOPE_XL_EN                    0x10
#define LSM6DSV_CTRL9_LPF2_XL_EN                        0x08
#define LSM6DSV_CTRL9_USR_OFF_W                         0x02
#define LSM6DSV_CTRL9_USR_OFF_ON_OUT                    0x01

// Control register 10 (R/W)
#define LSM6DSV_CTRL10                      0x19
#define LSM6DSV_CTRL10_EMB_FUNC_DEBUG                   0x40
#define LSM6DSV_CTRL10_ST_G_MASK                        0x0c
#define LSM6DSV_CTRL10_ST_G_SHIFT                       2
#define LSM6DSV_CTRL10_ST_G_NORMAL                      0
#define LSM6DSV_CTRL10_ST_G_POS_SELFTEST                1
#define LSM6DSV_CTRL10_ST_G_NEG_SELFTEST                2
#define LSM6DSV_CTRL10_ST_XL_MASK                       0x03
#define LSM6DSV_CTRL10_ST_XL_SHIFT                      0
#define LSM6DSV_CTRL10_ST_XL_NORMAL                     0
#define LSM6DSV_CTRL10_ST_XL_POS_SELFTEST               1
#define LSM6DSV_CTRL10_ST_XL_NEG_SELFTEST               2

// Control Status (R)
#define LSM6DSV_CTRL_STATUS                 0x1A
#define LSM6DSV_CTRL_STATUS_FSM_WR_CTRL_STATUS          0x04

// FIFO status register 1 (R)
#define LSM6DSV_FIFO_STATUS1                0x1B

// FIFO status register 2 (R)
#define LSM6DSV_FIFO_STATUS2                0x1C
#define LSM6DSV_FIFO_STATUS2_FIFO_WTM_IA                0x80
#define LSM6DSV_FIFO_STATUS2_FIFO_OVR_IA                0x40
#define LSM6DSV_FIFO_STATUS2_FIFO_FULL_IA               0x20
#define LSM6DSV_FIFO_STATUS2_COUNTER_BDR_IA             0x10
#define LSM6DSV_FIFO_STATUS2_FIFO_OVR_LATCHED           0x08
#define LSM6DSV_FIFO_STATUS2_DIFF_FIFO_8                0x01

// Source register for all interrupts (R)
#define LSM6DSV_ALL_INT_SRC                 0x1D
#define LSM6DSV_ALL_INT_SRC_EMB_FUNC_IA                 0x80
#define LSM6DSV_ALL_INT_SRC_SHUB_IA                     0x40
#define LSM6DSV_ALL_INT_SRC_SLEEP_CHANGE_IA             0x20
#define LSM6DSV_ALL_INT_SRC_D6D_IA                      0x10
#define LSM6DSV_ALL_INT_SRC_TAP_IA                      0x04
#define LSM6DSV_ALL_INT_SRC_WU_IA                       0x02
#define LSM6DSV_ALL_INT_SRC_FF_IA                       0x01

// The STATUS_REG register is read by the primary interface SPI/I²C & MIPI I3C® (R)
#define LSM6DSV_STATUS_REG                  0x1E
#define LSM6DSV_STATUS_REG_TIMESTAMP_ENDCOUNT           0x80
#define LSM6DSV_STATUS_REG_OIS_DRDY                     0x20
#define LSM6DSV_STATUS_REG_GDA_EIS                      0x10
#define LSM6DSV_STATUS_REG_AH_QVARDA                    0x80
#define LSM6DSV_STATUS_REG_TDA                          0x40
#define LSM6DSV_STATUS_REG_GDA                          0x20
#define LSM6DSV_STATUS_REG_XLDA                         0x10

// RESERVED - 1F

// Temperature data output register (R)
#define LSM6DSV_OUT_TEMP_L                  0x20
#define LSM6DSV_OUT_TEMP_H                  0x21

// Angular rate sensor pitch axis (X) angular rate output register (R)
#define LSM6DSV_OUTX_L_G                    0x22
#define LSM6DSV_OUTX_H_G                    0x23

// Angular rate sensor roll axis (Y) angular rate output register (R)
#define LSM6DSV_OUTY_L_G                    0x24
#define LSM6DSV_OUTY_H_G                    0x25

// Angular rate sensor yaw axis (Z) angular rate output register (R)
#define LSM6DSV_OUTZ_L_G                    0x26
#define LSM6DSV_OUTZ_H_G                    0x27

// Linear acceleration sensor X-axis output register (R)
#define LSM6DSV_OUTX_L_A                    0x28
#define LSM6DSV_OUTX_H_A                    0x29

// Linear acceleration sensor Y-axis output register (R)
#define LSM6DSV_OUTY_L_A                    0x2A
#define LSM6DSV_OUTY_H_A                    0x2B

// Linear acceleration sensor Z-axis output register (R)
#define LSM6DSV_OUTZ_L_A                    0x2C
#define LSM6DSV_OUTZ_H_A                    0x2D

// Angular rate sensor pitch axis (X) angular rate output register (R)
#define LSM6DSV_UI_OUTX_L_G_OIS_EIS         0x2E
#define LSM6DSV_UI_OUTX_H_G_OIS_EIS         0x2F

// Angular rate sensor roll axis (Y) angular rate output register (R)
#define LSM6DSV_UI_OUTY_L_G_OIS_EIS         0x30
#define LSM6DSV_UI_OUTY_H_G_OIS_EIS         0x31

// Angular rate sensor yaw axis (Z) angular rate output register (R)
#define LSM6DSV_UI_OUTZ_L_G_OIS_EIS         0x32
#define LSM6DSV_UI_OUTZ_H_G_OIS_EIS         0x33

// Linear acceleration sensor X-axis output register (R)
#define UI_OUTX_L_A_OIS_DualC               0x34
#define UI_OUTX_H_A_OIS_DualC               0x35

// Linear acceleration sensor Y-axis output register (R)
#define UI_OUTY_L_A_OIS_DualC               0x36
#define UI_OUTY_H_A_OIS_DualC               0x37

// Linear acceleration sensor Z-axis output register (R)
#define UI_OUTZ_L_A_OIS_DualC               0x38
#define UI_OUTZ_H_A_OIS_DualC               0x39

// Analog hub and Qvar data output register (R)
#define LSM6DSV_AH_QVAR_OUT_L               0x3A
#define LSM6DSV_AH_QVAR_OUT_H               0x3B

// RESERVED - 3C-3F

// Timestamp first data output register (R)
#define LSM6DSV_TIMESTAMP0                  0x40
#define LSM6DSV_TIMESTAMP1                  0x41
#define LSM6DSV_TIMESTAMP2                  0x42
#define LSM6DSV_TIMESTAMP3                  0x43

// Status regsiter
#define LSM6DSV_UI_STATUS_REG_OIS           0x44
#define LSM6DSV_UI_STATUS_REG_OIS_GYRO_SETTLING             0x04
#define LSM6DSV_UI_STATUS_REG_OIS_GDA_OIS                   0x02
#define LSM6DSV_UI_STATUS_REG_OIS_XLDA_OIS                  0x01

// Wake-up interrupt source register (R)
#define LSM6DSV_WAKE_UP_SRC                 0x45
#define LSM6DSV_WAKE_UP_SRC_SLEEP_CHANGE_IA                 0x40
#define LSM6DSV_WAKE_UP_SRC_FF_IA                           0x20
#define LSM6DSV_WAKE_UP_SRC_SLEEP_STATE                     0x10
#define LSM6DSV_WAKE_UP_SRC_WU_IA                           0x08
#define LSM6DSV_WAKE_UP_SRC_X_WU                            0x04
#define LSM6DSV_WAKE_UP_SRC_Y_WU                            0x02
#define LSM6DSV_WAKE_UP_SRC_Z_WU                            0x01

//Tap source register (R)
#define LSM6DSV_TAP_SRC                     0x46
#define LSM6DSV_TAP_SRC_TAP_IA                              0x40
#define LSM6DSV_TAP_SRC_SINGLE_TAP                          0x20
#define LSM6DSV_TAP_SRC_DOUBLE_TAP                          0x10
#define LSM6DSV_TAP_SRC_TAP_SIGN                            0x08
#define LSM6DSV_TAP_SRC_X_TAP                               0x04
#define LSM6DSV_TAP_SRC_Y_TAP                               0x02
#define LSM6DSV_TAP_SRC_Z_TAP                               0x01

// Portrait, landscape, face-up and face-down source register (R)
#define LSM6DSV_D6D_SRC                     0x47
#define LSM6DSV_D6D_SRC_D6D_IA                              0x40
#define LSM6DSV_D6D_SRC_ZH                                  0x20
#define LSM6DSV_D6D_SRC_ZL                                  0x10
#define LSM6DSV_D6D_SRC_YH                                  0x08
#define LSM6DSV_D6D_SRC_YL                                  0x04
#define LSM6DSV_D6D_SRC_XH                                  0x02
#define LSM6DSV_D6D_SRC_XL                                  0x01

// Sensor hub source register (R)
#define LSM6DSV_STATUS_MASTER_MAINPAGE      0x48
#define LSM6DSV_STATUS_MASTER_MAINPAGE_WR_ONCE_DONE         0x80
#define LSM6DSV_STATUS_MASTER_MAINPAGE_SLAVE3_NACK          0x40
#define LSM6DSV_STATUS_MASTER_MAINPAGE_SLAVE2_NACK          0x20
#define LSM6DSV_STATUS_MASTER_MAINPAGE_SLAVE1_NACK          0x10
#define LSM6DSV_STATUS_MASTER_MAINPAGE_SLAVE0_NACK          0x08
#define LSM6DSV_STATUS_MASTER_MAINPAGE_SENS_HUB_ENDOP       0x01

// Embedded function status register (R)
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE    0x49
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE_IS_FSM_LC          0x80
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE_IS_SIGMOT          0x20
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE_IS_TILT            0x10
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE_IS_STEP_DET        0x08

// Finite state machine status register (R)
#define LSM6DSV_FSM_STATUS_MAINPAGE         0x4A
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM8                 0x80
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM7                 0x40
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM6                 0x20
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM5                 0x10
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM4                 0x08
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM3                 0x04
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM2                 0x02
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM1                 0x01

// Embedded function status register (R)
#define LSM6DSV_MLC_STATUS_MAINPAGE         0x4B
#define LSM6DSV_MLC_STATUS_MAINPAGE_IS_MLC4                 0x08
#define LSM6DSV_MLC_STATUS_MAINPAGE_IS_MLC3                 0x04
#define LSM6DSV_MLC_STATUS_MAINPAGE_IS_MLC2                 0x02
#define LSM6DSV_MLC_STATUS_MAINPAGE_IS_MLC1                 0x01

// RESERVED - 4C-4E

// Internal frequency register (R)
#define LSM6DSV_INTERNAL_FREQ_FINE          0x4F

// Enable interrupt functions register (R/W)
#define LSM6DSV_FUNCTIONS_ENABLE            0x50
#define LSM6DSV_FUNCTIONS_ENABLE_INTERRUPTS_ENABLE          0x80
#define LSM6DSV_FUNCTIONS_ENABLE_TIMESTAMP_EN               0x40
#define LSM6DSV_FUNCTIONS_ENABLE_DIS_RST_LIR_ALL_INT        0x08
#define LSM6DSV_FUNCTIONS_ENABLE_INACT_EN_MASK              0x03
#define LSM6DSV_FUNCTIONS_ENABLE_INACT_EN_SHIFT             0

// DEN configuration register (R/W)
#define LSM6DSV_DEN                         0x51
#define LSM6DSV_DEN_LVL1_EN                                 0x40
#define LSM6DSV_DEN_LVL2_EN                                 0x20
#define LSM6DSV_DEN_DEN_XL_EN                               0x10
#define LSM6DSV_DEN_DEN_X                                   0x08
#define LSM6DSV_DEN_DEN_Y                                   0x04
#define LSM6DSV_DEN_DEN_Z                                   0x02
#define LSM6DSV_DEN_DEN_XL_G                                0x01

// Activity/inactivity configuration register (R/W)
#define LSM6DSV_INACTIVITY_DUR              0x54
#define LSM6DSV_INACTIVITY_DUR_SLEEP_STATUS_ON_INT          0x80
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_MASK          0x70
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_SHIFT         4
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_7_8125MG_LSB  0
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_15_625MG_LSB  1
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_31_25MG_LSB   2
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_62_5MG_LSB    3
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_125MG_LSB     4
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_250MG_LSB     5
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_MASK            0x0c
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_SHIFT           2
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_1_875HZ         0
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_15HZ            1
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_30HZ            2
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_60HZ            3
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_MASK               0x03
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_SHIFT              0
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_1                  0
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_2                  1
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_3                  2
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_4                  3

// Activity/inactivity threshold setting register (R/W)
#define LSM6DSV_INACTIVITY_THS              0x55
#define LSM6DSV_INACTIVITY_THS_MASK                         0x3f

// Tap configuration register 0 (R/W)
#define LSM6DSV_TAP_CFG0                    0x56
#define LSM6DSV_TAP_CFG0_LOW_PASS_ON_6D                     0x40
#define LSM6DSV_TAP_CFG0_HW_FUNC_MASK_XL_SETTL              0x20
#define LSM6DSV_TAP_CFG0_SLOPE_FDS                          0x10
#define LSM6DSV_TAP_CFG0_TAP_X_EN                           0x08
#define LSM6DSV_TAP_CFG0_TAP_Y_EN                           0x04
#define LSM6DSV_TAP_CFG0_TAP_Z_EN                           0x02
#define LSM6DSV_TAP_CFG0_LIR                                0x01

// Tap configuration register 1 (R/W)
#define LSM6DSV_TAP_CFG1                    0x57
#define LSM6DSV_TAP_CFG1_TAP_PRIORITY_MASK                  0xe0
#define LSM6DSV_TAP_CFG1_TAP_PRIORITY_SHIFT                 5
#define LSM6DSV_TAP_CFG1_TAP_THS_X_MASK                     0x1f
#define LSM6DSV_TAP_CFG1_TAP_THS_X_SHIFT                    0

// Tap configuration register 2 (R/W)
#define LSM6DSV_TAP_CFG2                    0x58
#define LSM6DSV_TAP_CFG2_TAP_THS_X_MASK                     0x1f
#define LSM6DSV_TAP_CFG2_TAP_THS_X_SHIFT                    0

// Portrait/landscape position and tap function threshold register (R/W)
#define LSM6DSV_TAP_THS_6D                  0x59
#define LSM6DSV_TAP_THS_6D_D4D_EN                           0x80
#define LSM6DSV_TAP_THS_6D_SIXD_THS_MASK                    0x60
#define LSM6DSV_TAP_THS_6D_SIXD_THS_SHIFT                   5
#define LSM6DSV_TAP_THS_6D_TAP_THS_Z_MASK                   0x1f
#define LSM6DSV_TAP_THS_6D_TAP_THS_Z_SHIFT                  0

// Tap recognition function setting register (R/W)
#define LSM6DSV_TAP_DUR                     0x5A
#define LSM6DSV_TAP_DUR_DUR_MASK                    0xf0
#define LSM6DSV_TAP_DUR_DUR_SHIFT                   4
#define LSM6DSV_TAP_DUR_QUIET_MASK                  0x0c
#define LSM6DSV_TAP_DUR_QUIET_SHIFT                 2
#define LSM6DSV_TAP_DUR_SHOCK_MASK                  0x03
#define LSM6DSV_TAP_DUR_SHOCK_SHIFT                 0

// Single/double-tap selection and wake-up configuration (R/W)
#define LSM6DSV_WAKE_UP_THS                 0x5B
#define LSM6DSV_WAKE_UP_THS_SINGLE_DOUBLE_TAP               0x80
#define LSM6DSV_WAKE_UP_THS_USR_OFF_ON_WU                   0x40
#define LSM6DSV_WAKE_UP_THS_WK_THS_MASK                     0x3f
#define LSM6DSV_WAKE_UP_THS_WK_THS_SHIFT                    6

// Free-fall, wake-up, and sleep mode functions duration setting register (R/W)
#define LSM6DSV_WAKE_UP_DUR                 0x5C
#define LSM6DSV_WAKE_UP_DUR_FF_DUR_5                        0x80
#define LSM6DSV_WAKE_UP_DUR_WAKE_DUR_MASK                   0x60
#define LSM6DSV_WAKE_UP_DUR_WAKE_DUR_SHIFT                  5
#define LSM6DSV_WAKE_UP_DUR_SLEEP_DUR_MASK                  0x0f
#define LSM6DSV_WAKE_UP_DUR_SLEEP_DUR_SHIFT                 0

// Free-fall function duration setting register (R/W)
#define LSM6DSV_FREE_FALL                   0x5D
#define LSM6DSV_FREE_FALL_FF_DUR_MASK                       0xf8
#define LSM6DSV_FREE_FALL_FF_DUR_SHIFT                      3
#define LSM6DSV_FREE_FALL_FF_THS_MASK                       0x07
#define LSM6DSV_FREE_FALL_FF_THS_SHIFT                      0

// Functions routing to INT1 pin register (R/W)
#define LSM6DSV_MD1_CFG                     0x5E
#define LSM6DSV_MD1_CFG_INT1_SLEEP_CHANG                    0x80
#define LSM6DSV_MD1_CFG_INT1_SINGLE_TAP                     0x40
#define LSM6DSV_MD1_CFG_INT1_WU                             0x20
#define LSM6DSV_MD1_CFG_INT1_FF                             0x10
#define LSM6DSV_MD1_CFG_INT1_DOUBLE_TAP                     0x08
#define LSM6DSV_MD1_CFG_INT1_6D                             0x04
#define LSM6DSV_MD1_CFG_INT1_EMB_FUNC                       0x02
#define LSM6DSV_MD1_CFG_INT1_SHUB                           0x01

// Functions routing to INT2 pin register (R/W)
#define LSM6DSV_MD2_CFG                     0x5F
#define LSM6DSV_MD2_CFG_INT2_SLEEP_CHANG                    0x80
#define LSM6DSV_MD2_CFG_INT2_SINGLE_TAP                     0x40
#define LSM6DSV_MD2_CFG_INT2_WU                             0x20
#define LSM6DSV_MD2_CFG_INT2_FF                             0x10
#define LSM6DSV_MD2_CFG_INT2_DOUBLE_TAP                     0x08
#define LSM6DSV_MD2_CFG_INT2_6D                             0x04
#define LSM6DSV_MD2_CFG_INT2_EMB_FUNC                       0x02
#define LSM6DSV_MD2_CFG_INT2_TIMESTAMP                      0x01

// RESERVED - 60-61

// HAODR data rate configuration register (R/W)
#define LSM6DSV_HAODR_CFG                   0x62
#define LSM6DSV_HAODR_CFG_HAODR_SEL_MASK                    0x03
#define LSM6DSV_HAODR_CFG_HAODR_SEL_SHIFT                   0
#define LSM6DSV_HAODR_MODE0                                 0
#define LSM6DSV_HAODR_MODE1                                 1
#define LSM6DSV_HAODR_MODE2                                 2

// Embedded functions configuration register (R/W)
#define LSM6DSV_EMB_FUNC_CFG                0x63
#define LSM6DSV_EMB_FUNC_CFG_XL_DUALC_BATCH_FROM_IF         0x80
#define LSM6DSV_EMB_FUNC_CFG_EMB_FUNC_IRQ_MASK_G_SETTL      0x20
#define LSM6DSV_EMB_FUNC_CFG_EMB_FUNC_IRQ_MASK_XL_SETTL     0x10
#define LSM6DSV_EMB_FUNC_CFG_EMB_FUNC_DISABLE               0x08

// Control register (UI side) for UI / SPI2 shared registers (R/W)
#define LSM6DSV_UI_HANDSHAKE_CTRL           0x64
#define LSM6DSV_UI_HANDSHAKE_CTRL_UI_SHARED_ACK             0x02
#define LSM6DSV_UI_HANDSHAKE_CTRL_UI_SHARED_REQ             0x01

// UI / SPI2 shared register 0 (R/W)
#define LSM6DSV_UI_SPI2_SHARED_0            0x65

// UI / SPI2 shared register 1 (R/W)
#define LSM6DSV_UI_SPI2_SHARED_1            0x66

// UI / SPI2 shared register 2 (R/W)
#define LSM6DSV_UI_SPI2_SHARED_2            0x67

// UI / SPI2 shared register 3 (R/W)
#define LSM6DSV_UI_SPI2_SHARED_3            0x68

// UI / SPI2 shared register 4 (R/W)
#define LSM6DSV_UI_SPI2_SHARED_4            0x69

// UI / SPI2 shared register 5 (R/W)
#define LSM6DSV_UI_SPI2_SHARED_5            0x6A

// Gyroscope EIS channel control register (R/W)
#define LSM6DSV_CTRL_EIS                    0x6B
#define LSM6DSV_CTRL_EIS_ODR_G_EIS_MASK                     0xc0
#define LSM6DSV_CTRL_EIS_ODR_G_EIS_SHIFT                    6
#define LSM6DSV_CTRL_EIS_ODR_G_EIS_OFF                      0
#define LSM6DSV_CTRL_EIS_ODR_G_EIS_1920HZ                   1
#define LSM6DSV_CTRL_EIS_ODR_G_EIS_960HZ                    2
#define LSM6DSV_CTRL_EIS_LPF_G_EIS_BW                       0x10
#define LSM6DSV_CTRL_EIS_G_EIS_ON_G_OIS_OUT_REG             0x08
#define LSM6DSV_CTRL_EIS_FS_G_EIS_MASK                      0x07
#define LSM6DSV_CTRL_EIS_FS_G_EIS_SHIFT                     0
#define LSM6DSV_CTRL_EIS_FS_G_EIS_125DPS                    0
#define LSM6DSV_CTRL_EIS_FS_G_EIS_250DPS                    1
#define LSM6DSV_CTRL_EIS_FS_G_EIS_500DPS                    2
#define LSM6DSV_CTRL_EIS_FS_G_EIS_1000DPS                   3
#define LSM6DSV_CTRL_EIS_FS_G_EIS_2000DPS                   4

// RESERVED - 6C - 6E

// OIS interrupt configuration register (R/W)
#define LSM6DSV_UI_INT_OIS                  0x6F
#define LSM6DSV_UI_INT_OIS_INT2_DRDY_OIS                    0x80
#define LSM6DSV_UI_INT_OIS_DRDY_MASK_OIS                    0x40
#define LSM6DSV_UI_INT_OIS_ST_OIS_CLAMPDIS                  0x10

// OIS configuration register (R/W)
#define LSM6DSV_UI_CTRL1_OIS                0x70
#define LSM6DSV_UI_CTRL1_OIS_SIM_OIS                        0x20
#define LSM6DSV_UI_CTRL1_OIS_OIS_XL_EN                      0x04
#define LSM6DSV_UI_CTRL1_OIS_OIS_G_EN                       0x02
#define LSM6DSV_UI_CTRL1_OIS_SPI2_READ_EN                   0x01

// OIS configuration register (R/W)
#define LSM6DSV_UI_CTRL2_OIS                0x71
#define LSM6DSV_UI_CTRL2_OIS_LPF1_G_OIS_BW_MASK             0x18
#define LSM6DSV_UI_CTRL2_OIS_LPF1_G_OIS_BW_SHIFT            3
#define LSM6DSV_UI_CTRL2_OIS_FS_G_OIS_MASK                  0x07
#define LSM6DSV_UI_CTRL2_OIS_FS_G_OIS_SHIFT                 0
#define LSM6DSV_UI_CTRL2_FS_G_EIS_125DPS                    0
#define LSM6DSV_UI_CTRL2_FS_G_EIS_250DPS                    1
#define LSM6DSV_UI_CTRL2_FS_G_EIS_500DPS                    2
#define LSM6DSV_UI_CTRL2_FS_G_EIS_1000DPS                   3
#define LSM6DSV_UI_CTRL2_FS_G_EIS_2000DPS                   4

// OIS configuration register (R/W)
#define LSM6DSV_UI_CTRL3_OIS                0x72
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_MASK             0x38
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_SHIFT            3
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_2G               0
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_4G               1
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_8G               2
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_16G              3
#define LSM6DSV_UI_CTRL3_OIS_FS_XL_OIS_MASK                 0x03
#define LSM6DSV_UI_CTRL3_OIS_FS_XL_OIS_SHIFT                0

// Accelerometer X-axis user offset correction (R/W)
#define LSM6DSV_X_OFS_USR                   0x73

// Accelerometer Y-axis user offset correction (R/W)
#define LSM6DSV_Y_OFS_USR                   0x74

// Accelerometer Z-axis user offset correction (R/W)
#define LSM6DSV_Z_OFS_USR                   0x75

// RESERVED - 76-77

// FIFO tag register (R)
#define LSM6DSV_FIFO_DATA_OUT_TAG           0x78
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_MASK               0xf8
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_SHIFT              3
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_EMPTY                 0x00
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_GYRO_NC               0x01
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_ACC_NC                0x02
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_TEMP                  0x03
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_TIMESTAMP             0x04
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_CFG_CHANGE            0x05
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_ACC_NC_T_2            0x06
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_ACC_NC_T_1            0x07
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_ACC_2XC               0x08
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_ACC_3XC               0x09
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_GYRO_NC_T_2           0x0a
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_GYRO_NC_T_1           0x0b
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_GYRO_2XC              0x0c
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_GYRO_3XC              0x0d
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_SENSOR_HUB_SLAVE_0    0x0e
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_SENSOR_HUB_SLAVE_1    0x0f
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_SENSOR_HUB_SLAVE_2    0x10
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_SENSOR_HUB_SLAVE_3    0x11
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_STEP_COUNT            0x12
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_SFLP_GAME_ROT_VEC     0x13
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_SFLP_GYRO_BIAS        0x16
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_SFLP_GRAVITY_VEC      0x17
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_SENSOR_HUB_NACK       0x19
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_MLC_RESULT            0x1a
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_MLC_FILTER            0x1b
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_MLC_FEATURE           0x1c
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_ACC_DUALC             0x1d
#define LSM6DSV_FIFO_DATA_OUT_TAG_SENSOR_FIFO_ENHANCED_EIS_GYRO     0x1e
#define LSM6DSV_FIFO_DATA_OUT_TAG_CNT_MASK                  0x07
#define LSM6DSV_FIFO_DATA_OUT_TAG_CNT_SHIFT                 0

// FIFO data output X (R)
#define LSM6DSV_FIFO_DATA_OUT_X_L           0x79
#define LSM6DSV_FIFO_DATA_OUT_X_H           0x7A

// FIFO data output Y (R)
#define LSM6DSV_FIFO_DATA_OUT_Y_L           0x7B
#define LSM6DSV_FIFO_DATA_OUT_Y_H           0x7C

// FIFO data output Z (R)
#define LSM6DSV_FIFO_DATA_OUT_Z_L           0x7D
#define LSM6DSV_FIFO_DATA_OUT_Z_H           0x7E

#define LSM6DSV16X_READY                    0

#define LSM6DSK320X_WHO_AM_I_CONST          (0x75)

uint8_t lsm6dsv16xSpiDetect(const extDevice_t *dev)
{
    const uint8_t whoAmI = spiReadRegMsk(dev, LSM6DSV_WHO_AM_I);
    if ((whoAmI == LSM6DSV16X_WHO_AM_I_CONST) || (whoAmI == LSM6DSK320X_WHO_AM_I_CONST)) {
        return LSM6DSV16X_SPI;
    }

    return MPU_NONE;
}

static void lsm6dsv16xAccInit(accDev_t *acc)
{
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

static void lsm6dsv16xGyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;
    // Set default LPF1 filter bandwidth to be as close as possible to MPU6000's 250Hz cutoff
    uint8_t lsm6dsv16xLPF1BandwidthOptions[GYRO_HARDWARE_LPF_COUNT] = {
            [GYRO_HARDWARE_LPF_NORMAL] = LSM6DSV_CTRL6_FS_G_BW_288HZ,
            [GYRO_HARDWARE_LPF_OPTION_1] = LSM6DSV_CTRL6_FS_G_BW_157HZ,
            [GYRO_HARDWARE_LPF_OPTION_2] = LSM6DSV_CTRL6_FS_G_BW_215HZ,
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
            [GYRO_HARDWARE_LPF_EXPERIMENTAL] = LSM6DSV_CTRL6_FS_G_BW_455HZ
#endif
    };

    spiSetClkDivisor(dev, spiCalculateDivider(LSM6DSV16X_MAX_SPI_CLK_HZ));

     // Perform a software reset
    spiWriteReg(dev, LSM6DSV_CTRL3, LSM6DSV_CTRL3_SW_RESET);

    // Wait for the device to be ready
    while (spiReadRegMsk(dev, LSM6DSV_CTRL3) & LSM6DSV_CTRL3_SW_RESET) {}
    // Autoincrement register address when doing block SPI reads and update continuously
    spiWriteReg(dev, LSM6DSV_CTRL3, LSM6DSV_CTRL3_IF_INC | LSM6DSV_CTRL3_BDU);      /*BDU bit need to be set*/
	
	// Select high-accuracy ODR mode 1 before leaving power-off mode
    spiWriteReg(dev, LSM6DSV_HAODR_CFG,
                LSM6DSV_ENCODE_BITS(LSM6DSV_HAODR_MODE1,
                                    LSM6DSV_HAODR_CFG_HAODR_SEL_MASK,
                                    LSM6DSV_HAODR_CFG_HAODR_SEL_SHIFT));

    // Enable 16G full-scale range and set accelerometer LPF2 bandwidth
    spiWriteReg(dev, LSM6DSV_CTRL8,
    			LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL8_FS_XL_BW_4,
                                    LSM6DSV_CTRL8_HP_LPF2_XL_BW_2_MASK,
                                    LSM6DSV_CTRL8_HP_LPF2_XL_BW_2_SHIFT) |
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL8_FS_XL_16G,
                                    LSM6DSV_CTRL8_FS_XL_MASK,
                                    LSM6DSV_CTRL8_FS_XL_SHIFT));
	// Enable 2000 deg/s sensitivity and selected LPF1 filter setting
    // Set the LPF1 filter bandwidth
    spiWriteReg(dev, LSM6DSV_CTRL6,
                LSM6DSV_ENCODE_BITS(lsm6dsv16xLPF1BandwidthOptions[gyroConfig()->gyro_hardware_lpf],
                                    LSM6DSV_CTRL6_LPF1_G_BW_MASK,
                                    LSM6DSV_CTRL6_LPF1_G_BW_SHIFT) |
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL6_FS_G_2000DPS,
                                    LSM6DSV_CTRL6_FS_G_MASK,
                                    LSM6DSV_CTRL6_FS_G_SHIFT));

    // Enable the accelerometer odr at 1kHz, in high accuracy
    spiWriteReg(dev, LSM6DSV_CTRL1,
           LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL1_OP_MODE_XL_HIGH_ACCURACY,
                                    LSM6DSV_CTRL1_OP_MODE_XL_MASK,
                                    LSM6DSV_CTRL1_OP_MODE_XL_SHIFT) |
           LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL1_ODR_XL_1000HZ,
                                    LSM6DSV_CTRL1_ODR_XL_MASK,
                                    LSM6DSV_CTRL1_ODR_XL_SHIFT));

    // Enable the gyro odr at 8kHz, in high accuracy
    spiWriteReg(dev, LSM6DSV_CTRL2,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL2_OP_MODE_G_HIGH_ACCURACY,
                                    LSM6DSV_CTRL2_OP_MODE_G_MASK,
                                    LSM6DSV_CTRL2_OP_MODE_G_SHIFT) |
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL2_ODR_G_8000HZ,
                                    LSM6DSV_CTRL2_ODR_G_MASK,
                                    LSM6DSV_CTRL2_ODR_G_SHIFT));
                             
    // Enable the gyro digital LPF1 filter
    spiWriteReg(dev, LSM6DSV_CTRL7, LSM6DSV_CTRL7_LPF1_G_EN);

	// Enable the acc digital LPF2 filter
    spiWriteReg(dev, LSM6DSV_CTRL9, LSM6DSV_CTRL9_LPF2_XL_EN);

    // Generate pulse on interrupt line, not requiring a read to clear
    spiWriteReg(dev, LSM6DSV_CTRL4, LSM6DSV_CTRL4_DRDY_PULSED);

    // From section 4.1, Mechanical characteristics, of the datasheet, G_So is 70mdps/LSB for FS = ±2000 dps.
    gyro->scale = 0.070f;

    // Enable the INT1 output to interrupt when new gyro data is ready
    spiWriteReg(dev, LSM6DSV_INT1_CTRL, LSM6DSV_INT1_CTRL_INT1_DRDY_G);

    mpuGyroInit(gyro);
}

static bool lsm6dsv16xGyroReadSPI(gyroDev_t *gyro)
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
                gyro->dev.txBuf[0] = LSM6DSV_OUTX_L_G | 0x80;
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
        gyro->dev.txBuf[0] = LSM6DSV_OUTX_L_G | 0x80;

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
