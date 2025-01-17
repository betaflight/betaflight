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

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "barometer.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "barometer_lps22df.h"

// 10 MHz max SPI frequency
#define LPS22DF_MAX_SPI_CLK_HZ 10000000

#if defined(USE_BARO) && (defined(USE_BARO_LPS22DF) || defined(USE_BARO_SPI_LPS22DF))

/* See datasheet
 *
 *     https://www.st.com/resource/en/datasheet/lps22df.pdf
 */

// Macros to encode/decode multi-bit values
#define LSM6DSV_ENCODE_BITS(val, mask, shift)   ((val << shift) & mask)
#define LSM6DSV_DECODE_BITS(val, mask, shift)   ((val & mask) >> shift)

// RESERVED - 00-0A

// Interrupt mode for pressure acquisition configuration (R/W)
#define LPS22DF_INTERRUPT_CFG               0x0B
#define LPS22DF_INTERRUPT_CFG_AUTOREFP                  0x80
#define LPS22DF_INTERRUPT_CFG_RESET_ARP                 0x40
#define LPS22DF_INTERRUPT_CFG_AUTOZERO                  0x20
#define LPS22DF_INTERRUPT_CFG_RESET_AZ                  0x10
#define LPS22DF_INTERRUPT_CFG_LIR                       0x04
#define LPS22DF_INTERRUPT_CFG_PLE                       0x02
#define LPS22DF_INTERRUPT_CFG_PHE                       0x01

// Threshold value for pressure interrupt event (least significant bits) (R/W)
#define LPS22DF_THS_P_L                     0x0C
#define LPS22DF_THS_P_H                     0x0D

// Interface control register (R/W)
#define LPS22DF_IF_CTRL                     0x0E
#define LPS22DF_IF_CTRL_INT_EN_I3C                      0x80
#define LPS22DF_IF_CTRL_I2C_I3C_DIS                     0x40
#define LPS22DF_IF_CTRL_SIM                             0x20
#define LPS22DF_IF_CTRL_SDA_PU_EN                       0x10
#define LPS22DF_IF_CTRL_SDO_PU_EN                       0x08
#define LPS22DF_IF_CTRL_INT_PD_DIS                      0x04
#define LPS22DF_IF_CTRL_CS_PU_DIS                       0x02

// Device Who am I (R)
#define LPS22DF_WHO_AM_I                    0x0F
#define LPS22DF_CHIP_ID                                 0xB4

// Control register 1 (R/W)
#define LPS22DF_CTRL_REG1                   0x10
#define LPS22DF_CTRL_REG1_ODR_MASK                      0x78
#define LPS22DF_CTRL_REG1_ODR_SHIFT                     3
#define LPS22DF_CTRL_REG1_ODR_ONE_SHOT                  0
#define LPS22DF_CTRL_REG1_ODR_1HZ                       1
#define LPS22DF_CTRL_REG1_ODR_4HZ                       2
#define LPS22DF_CTRL_REG1_ODR_10HZ                      3
#define LPS22DF_CTRL_REG1_ODR_25HZ                      4
#define LPS22DF_CTRL_REG1_ODR_50HZ                      5
#define LPS22DF_CTRL_REG1_ODR_75HZ                      6
#define LPS22DF_CTRL_REG1_ODR_100HZ                     7
#define LPS22DF_CTRL_REG1_ODR_200HZ                     8
#define LPS22DF_CTRL_REG1_AVG_MASK                      0x03
#define LPS22DF_CTRL_REG1_AVG_SHIFT                     0
#define LPS22DF_CTRL_REG1_AVG_4                         0
#define LPS22DF_CTRL_REG1_AVG_8                         1
#define LPS22DF_CTRL_REG1_AVG_16                        2
#define LPS22DF_CTRL_REG1_AVG_32                        3
#define LPS22DF_CTRL_REG1_AVG_64                        4
#define LPS22DF_CTRL_REG1_AVG_128                       5
#define LPS22DF_CTRL_REG1_AVG_512                       7

// Control register 2 (R/W)
#define LPS22DF_CTRL_REG2                   0x11
#define LPS22DF_CTRL_REG2_BOOT                          0x80
#define LPS22DF_CTRL_REG2_LFPF_CFG                      0x20
#define LPS22DF_CTRL_REG2_EN_LPFP                       0x10
#define LPS22DF_CTRL_REG2_BDU                           0x08
#define LPS22DF_CTRL_REG2_SWRESET                       0x04
#define LPS22DF_CTRL_REG2_ONESHOT                       0x01

// Control register 3 (R/W)
#define LPS22DF_CTRL_REG3                   0x12
#define LPS22DF_CTRL_REG3_INT_HL                        0x08
#define LPS22DF_CTRL_REG3_PP_OD                         0x02
#define LPS22DF_CTRL_REG3_IF_ADD_INC                    0x01

// Control register 4 (R/W)
#define LPS22DF_CTRL_REG4                   0x13
#define LPS22DF_CTRL_REG4_DRDY_PLS                      0x40
#define LPS22DF_CTRL_REG4_DRDY                          0x20
#define LPS22DF_CTRL_REG4_INT_EN                        0x10
#define LPS22DF_CTRL_REG4_INT_F_FULL                    0x04
#define LPS22DF_CTRL_REG4_INT_F_WTM                     0x02
#define LPS22DF_CTRL_REG4_INT_F_OVR                     0x01

//FIFO control register (R/W)
#define LPS22DF_FIFO_CTRL                   0x14
#define LPS22DF_FIFO_CTRL_STOP_ON_WTM                   0x08
#define LPS22DF_FIFO_CTRL_TRIG_MODES                    0x04
#define LPS22DF_FIFO_CTRL_F_MODE_MASK                   0x03
#define LPS22DF_FIFO_CTRL_F_MODE_SHIFT                  0
#define LPS22DF_FIFO_CTRL_F_MODE_BYPASS                 0
#define LPS22DF_FIFO_CTRL_F_MODE_FIFO_MODE              1
#define LPS22DF_FIFO_CTRL_F_MODE_CONT                   2
#define LPS22DF_FIFO_CTRL_F_MODE_CONT_TO_FIFO           3

// FIFO threshold setting register (R/W)
#define LPS22DF_FIFO_WTM                    0x15

// Reference pressure LSB data (R)
#define LPS22DF_REF_P_L                     0x16
#define LPS22DF_REF_P_H                     0x17

// RESERVED - 18

// Control register (R/W)
#define LPS22DF_I3C_IF_CTRL_ADD             0x19
#define LPS22DF_I3C_IF_CTRL_ADD_RESVD                   0x80
#define LPS22DF_I3C_IF_CTRL_ADD_ASF_ON                  0x20
#define LPS22DF_I3C_IF_CTRL_ADD_I3C_BUS_AVB_SEL_MASK    0x03
#define LPS22DF_I3C_IF_CTRL_ADD_I3C_BUS_AVB_SEL_SHIFT   0
#define LPS22DF_I3C_IF_CTRL_ADD_I3C_BUS_AVB_SEL_50US    0
#define LPS22DF_I3C_IF_CTRL_ADD_I3C_BUS_AVB_SEL_2US     1
#define LPS22DF_I3C_IF_CTRL_ADD_I3C_BUS_AVB_SEL_1MS     2
#define LPS22DF_I3C_IF_CTRL_ADD_I3C_BUS_AVB_SEL_25MS    3

// Pressure offset (R/W)
#define LPS22DF_RPDS_L                      0x1A
#define LPS22DF_RPDS_H                      0x1B

// RESERVED - 1C-23

// Interrupt source (R) register for differential pressure. A read at this address clears the INT_SOURCE register itself.
#define LPS22DF_INT_SOURCE                  0x24
#define LPS22DF_INT_SOURCE_BOOT_ON                      0x80
#define LPS22DF_INT_SOURCE_IA                           0x04
#define LPS22DF_INT_SOURCE_PL                           0x02
#define LPS22DF_INT_SOURCE_PH                           0x01

// FIFO status register (R)
#define LPS22DF_FIFO_STATUS1                0x25

// FIFO status register (R)
#define LPS22DF_FIFO_STATUS2                0x26
#define LPS22DF_FIFO_STATUS2_WTM_IA                     0x80
#define LPS22DF_FIFO_STATUS2_OVR_IA                     0x40
#define LPS22DF_FIFO_STATUS2_FULL_IA                    0x20

// Status register (R)
#define LPS22DF_STATUS                      0x27
#define LPS22DF_STATUS_T_OR                             0x20
#define LPS22DF_STATUS_P_OR                             0x10
#define LPS22DF_STATUS_T_DA                             0x02
#define LPS22DF_STATUS_P_DA                             0x01

// Pressure output value (R)
#define LPS22DF_PRESSURE_OUT_XL             0x28
#define LPS22DF_PRESSURE_OUT_L              0x29
#define LPS22DF_PRESSURE_OUT_H              0x2A

// Temperature output value (R)
#define LPS22DF_TEMP_OUT_L                  0x2B
#define LPS22DF_TEMP_OUT_H                  0x2C

// RESERVED - 2D-77

// FIFO pressure output (R)
#define LPS22DF_FIFO_DATA_OUT_PRESS_XL      0x78
#define LPS22DF_FIFO_DATA_OUT_PRESS_L       0x79
#define LPS22DF_FIFO_DATA_OUT_PRESS_H       0x7A

#define LPS22DF_I2C_ADDR              0x5D

static uint8_t lps22df_chip_id = 0;

// uncompensated pressure and temperature
static int32_t lps22df_up = 0;
static int32_t lps22df_ut = 0;

// 3 bytes of pressure followed by two bytes of temperature
#define LPS22DF_DATA_FRAME_SIZE (LPS22DF_TEMP_OUT_H - LPS22DF_PRESSURE_OUT_XL + 1)

static DMA_DATA_ZERO_INIT uint8_t sensor_data[LPS22DF_DATA_FRAME_SIZE];

static bool lps22dfStartUT(baroDev_t *baro);
static bool lps22dfReadUT(baroDev_t *baro);
static bool lps22dfGetUT(baroDev_t *baro);
static bool lps22dfStartUP(baroDev_t *baro);
static bool lps22dfReadUP(baroDev_t *baro);
static bool lps22dfGetUP(baroDev_t *baro);

STATIC_UNIT_TESTED void lps22dfCalculate(int32_t *pressure, int32_t *temperature);

void lps22dfBusInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_LPS22DF
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin); // Disable
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(LPS22DF_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
#endif
}

void lps22dfBusDeinit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_LPS22DF
    if (dev->bus->busType == BUS_TYPE_SPI) {
        ioPreinitByIO(dev->busType_u.spi.csnPin, IOCFG_IPU, PREINIT_PIN_STATE_LOW);
    }
#else
    UNUSED(dev);
#endif
}

bool lps22dfDetect(baroDev_t *baro)
{
    delay(20);

    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    lps22dfBusInit(dev);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for LPS22DF
        dev->busType_u.i2c.address = LPS22DF_I2C_ADDR;
        defaultAddressApplied = true;
    }

    busReadRegisterBuffer(dev, LPS22DF_WHO_AM_I, &lps22df_chip_id, 1);  /* read Chip Id */

    if ((lps22df_chip_id != LPS22DF_CHIP_ID)) {
        lps22dfBusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    // Reset the device
    busWriteRegister(dev, LPS22DF_CTRL_REG2, LPS22DF_CTRL_REG2_SWRESET);

    // Enable one-shot ODR and averaging at 16
    busWriteRegister(dev, LPS22DF_CTRL_REG1, LSM6DSV_ENCODE_BITS(LPS22DF_CTRL_REG1_ODR_ONE_SHOT,
                                                                 LPS22DF_CTRL_REG1_ODR_MASK,
                                                                 LPS22DF_CTRL_REG1_ODR_SHIFT) |
                                             LSM6DSV_ENCODE_BITS(LPS22DF_CTRL_REG1_AVG_16,
                                                                 LPS22DF_CTRL_REG1_AVG_MASK,
                                                                 LPS22DF_CTRL_REG1_AVG_SHIFT));

    // these are dummy as temperature is measured as part of pressure
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->start_ut = lps22dfStartUT;
    baro->get_ut = lps22dfGetUT;
    baro->read_ut = lps22dfReadUT;
    // only _up part is executed, and gets both temperature and pressure
    baro->start_up = lps22dfStartUP;
    baro->get_up = lps22dfGetUP;
    baro->read_up = lps22dfReadUP;
    baro->up_delay = 10000; // 10ms
    baro->calculate = lps22dfCalculate;

    return true;
}

static bool lps22dfStartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy

    return true;
}

static bool lps22dfReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
    return true;
}

static bool lps22dfGetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
    return true;
}

static bool lps22dfStartUP(baroDev_t *baro)
{
    // start measurement
    // Trigger one-shot enable block data update to ensure LSB/MSB are coherent
    return busWriteRegister(&baro->dev, LPS22DF_CTRL_REG2, LPS22DF_CTRL_REG2_ONESHOT | LPS22DF_CTRL_REG2_BDU);
}

static bool lps22dfReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // Read data from sensor
    return busReadRegisterBufferStart(&baro->dev, LPS22DF_PRESSURE_OUT_XL, sensor_data, LPS22DF_DATA_FRAME_SIZE);
}

static bool lps22dfGetUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    lps22df_up = (int32_t)(sensor_data[0] | sensor_data[1] << 8 | sensor_data[2] << 16);
    lps22df_ut = (int32_t)(sensor_data[3] | sensor_data[4] << 8);

    return true;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static int32_t lps22dfCompensateTemperature(int32_t adc_T)
{
    return adc_T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t lps22dfCompensatePressure(int32_t adc_P)
{
    return (uint32_t)(adc_P * 100.0f / 16);
}

STATIC_UNIT_TESTED void lps22dfCalculate(int32_t *pressure, int32_t *temperature)
{
    // calculate
    int32_t t;
    uint32_t p;
    t = lps22dfCompensateTemperature(lps22df_ut);
    p = lps22dfCompensatePressure(lps22df_up);

    if (pressure)
        *pressure = (int32_t)(p / 256);
    if (temperature)
        *temperature = t;
}

#endif
