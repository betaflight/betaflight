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

#include "barometer_lps22h.h"


// 10 MHz max SPI frequency
#define LPS22H_MAX_SPI_CLK_HZ 10000000

#if defined(USE_BARO) && (defined(USE_BARO_LPS22H) || defined(USE_BARO_SPI_LPS22H))

/* See datasheet
 *
 *     https://www.st.com/resource/en/datasheet/lps22hb.pdf
 */

// Macros to encode/decode multi-bit values
#define LPS22H_ENCODE_BITS(val, mask, shift)   ((val << shift) & mask)
#define LPS22H_DECODE_BITS(val, mask, shift)   ((val & mask) >> shift)

// RESERVED - 00-0A

// Interrupt mode for pressure acquisition configuration (R/W)
#define LPS22H_INTERRUPT_CFG               0x0B
#define LPS22H_INTERRUPT_CFG_AUTOREFP                  0x80
#define LPS22H_INTERRUPT_CFG_RESET_ARP                 0x40
#define LPS22H_INTERRUPT_CFG_AUTOZERO                  0x20
#define LPS22H_INTERRUPT_CFG_RESET_AZ                  0x10
#define LPS22H_INTERRUPT_CFG_LIR                       0x04
#define LPS22H_INTERRUPT_CFG_PLE                       0x02
#define LPS22H_INTERRUPT_CFG_PHE                       0x01

// Threshold value for pressure interrupt event (least significant bits) (R/W)
#define LPS22H_THS_P_L                     0x0C
#define LPS22H_THS_P_H                     0x0D


// Device Who am I (R)
#define LPS22H_WHO_AM_I                    0x0F
#define LPS22HB_CHIP_ID                     0xB1 // LPS22HB
#define LPS22HH_CHIP_ID                     0xB3 // LPS22HH

// Control register 1 (R/W)
#define LPS22H_CTRL_REG1                   0x10
#define LPS22H_CTRL_REG1_ODR_MASK               0x70 // bits 4-6
#define LPS22H_CTRL_REG1_ODR_SHIFT              4
#define LPS22H_CTRL_REG1_ODR_POWER_DOWN         0
#define LPS22H_CTRL_REG1_ODR_1HZ                1
#define LPS22H_CTRL_REG1_ODR_10HZ               2
#define LPS22H_CTRL_REG1_ODR_25HZ               3
#define LPS22H_CTRL_REG1_ODR_50HZ               4
#define LPS22H_CTRL_REG1_ODR_75HZ               5

#define LPS22H_CTRL_REG1_BDU                    0x02
#define LPS22H_CTRL_REG1_SIM                    0x01

// Control register 2 (R/W)
#define LPS22H_CTRL_REG2                   0x11
#define LPS22H_CTRL_REG2_BOOT                   0x80
#define LPS22H_CTRL_REG2_FIFO_EN                0x40 
#define LPS22H_CTRL_REG2_STOP_ON_FTH            0x20
#define LPS22H_CTRL_REG2_IF_ADD_INC             0x10
#define LPS22H_CTRL_REG2_I2C_DIS                0x08
#define LPS22H_CTRL_REG2_SWRESET                0x04 
#define LPS22H_CTRL_REG2_ONE_SHOT               0x01 


// Reference pressure LSB data (R)
#define LPS22H_REF_P_L                     0x16
#define LPS22H_REF_P_H                     0x17


// Pressure offset (R/W)
#define LPS22H_RPDS_L                      0x18
#define LPS22H_RPDS_H                      0x19


// Status register (R)
#define LPS22H_STATUS                      0x27
#define LPS22H_STATUS_T_OR                             0x20
#define LPS22H_STATUS_P_OR                             0x10
#define LPS22H_STATUS_T_DA                             0x02
#define LPS22H_STATUS_P_DA                             0x01

// Pressure output value (R)
#define LPS22H_PRESSURE_OUT_XL             0x28
#define LPS22H_PRESSURE_OUT_L              0x29
#define LPS22H_PRESSURE_OUT_H              0x2A

// Temperature output value (R)
#define LPS22H_TEMP_OUT_L                  0x2B
#define LPS22H_TEMP_OUT_H                  0x2C

// RESERVED - 2D-77

#define LPS22H_I2C_ADDR              0x5D

static uint8_t lps22h_chip_id = 0;

// uncompensated pressure and temperature
static int32_t lps22h_up = 0;
static int32_t lps22h_ut = 0;

// 3 bytes of pressure followed by two bytes of temperature
#define LPS22H_DATA_FRAME_SIZE (LPS22H_TEMP_OUT_H - LPS22H_PRESSURE_OUT_XL + 1)

static DMA_DATA_ZERO_INIT uint8_t sensor_data[LPS22H_DATA_FRAME_SIZE];

static bool lps22hStartUT(baroDev_t *baro);
static bool lps22hReadUT(baroDev_t *baro);
static bool lps22hGetUT(baroDev_t *baro);
static bool lps22hStartUP(baroDev_t *baro);
static bool lps22hReadUP(baroDev_t *baro);
static bool lps22hGetUP(baroDev_t *baro);

STATIC_UNIT_TESTED void lps22hCalculate(int32_t *pressure, int32_t *temperature);

static void lps22hBusInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_LPS22H
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin); // Disable
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(LPS22H_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
#endif
}

static void lps22hBusDeinit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_LPS22H
    if (dev->bus->busType == BUS_TYPE_SPI) {
        ioPreinitByIO(dev->busType_u.spi.csnPin, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
    }
#else
    UNUSED(dev);
#endif
}

bool lps22hDetect(baroDev_t *baro)
{
    delay(20);

    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    lps22hBusInit(dev);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for LPS22H
        dev->busType_u.i2c.address = LPS22H_I2C_ADDR;
        defaultAddressApplied = true;
    }

    busReadRegisterBuffer(dev, LPS22H_WHO_AM_I, &lps22h_chip_id, 1);  /* read Chip Id */

    if ((lps22h_chip_id != LPS22HB_CHIP_ID) && (lps22h_chip_id != LPS22HH_CHIP_ID)) {
        lps22hBusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    // Reset the device
    busWriteRegister(dev, LPS22H_CTRL_REG2, LPS22H_CTRL_REG2_SWRESET);
    busWriteRegister(dev, LPS22H_CTRL_REG1, LPS22H_ENCODE_BITS(
    LPS22H_CTRL_REG1_ODR_POWER_DOWN, LPS22H_CTRL_REG1_ODR_MASK, LPS22H_CTRL_REG1_ODR_SHIFT
    ));

    // these are dummy as temperature is measured as part of pressure
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->start_ut = lps22hStartUT;
    baro->get_ut = lps22hGetUT;
    baro->read_ut = lps22hReadUT;
    // only _up part is executed, and gets both temperature and pressure
    baro->start_up = lps22hStartUP;
    baro->get_up = lps22hGetUP;
    baro->read_up = lps22hReadUP;
    baro->up_delay = 10000; // 10ms
    baro->calculate = lps22hCalculate;

    return true;
}

static bool lps22hStartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy

    return true;
}

static bool lps22hReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
    return true;
}

static bool lps22hGetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
    return true;
}

static bool lps22hStartUP(baroDev_t *baro)
{
    // start measurement
    // Trigger a new conversion
    return busWriteRegister(&baro->dev, LPS22H_CTRL_REG2, LPS22H_CTRL_REG2_ONE_SHOT|LPS22H_CTRL_REG2_IF_ADD_INC);
}

static bool lps22hReadUP(baroDev_t *baro)
{
    uint8_t status;
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }
    busReadRegisterBuffer(&baro->dev, LPS22H_STATUS, &status, 1);
    if (!(status & LPS22H_STATUS_P_DA)) {
        return false; // Data not ready
    }

    // Read data from sensor
    return busReadRegisterBufferStart(&baro->dev, LPS22H_PRESSURE_OUT_XL, sensor_data, LPS22H_DATA_FRAME_SIZE);
}

static bool lps22hGetUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    lps22h_up = (int32_t)(sensor_data[0] | sensor_data[1] << 8 | sensor_data[2] << 16);
    // Sign extend 24-bit to 32-bit
    if (lps22h_up & 0x800000) {
        lps22h_up |= 0xFF000000; // Extend sign bit
    }
    
    lps22h_ut = (int32_t)(sensor_data[3] | sensor_data[4] << 8);
    if (lps22h_ut & 0x8000) {
        lps22h_ut |= 0xFFFF0000; // Sign extend 16-bit to 32-bit
    }

    return true;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static int32_t lps22hCompensateTemperature(int32_t adc_T)
{
    return adc_T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t lps22hCompensatePressure(int32_t adc_P)
{
    return (uint32_t)(adc_P * 100.0f / 16);
}

STATIC_UNIT_TESTED void lps22hCalculate(int32_t *pressure, int32_t *temperature)
{
    // calculate
    int32_t t;
    uint32_t p;
    t = lps22hCompensateTemperature(lps22h_ut);
    p = lps22hCompensatePressure(lps22h_up);

    if (pressure)
        *pressure = (int32_t)(p / 256);
    if (temperature)
        *temperature = t;
}

// Just for debug purpose, not used in production
/*
void lps22hDebug(baroDev_t *baro)
{
    uint8_t status;
    uint8_t id;
    uint8_t reg1, reg2;
    int32_t pressure;
    int32_t temperature;
    uint8_t raw_data[LPS22H_DATA_FRAME_SIZE];

    // Step 1: Read chip ID and configuration registers
    cliPrintf("LPS22H Debug Start...\r\n");
    
    busReadRegisterBuffer(&baro->dev, LPS22H_WHO_AM_I, &id, 1);
    busReadRegisterBuffer(&baro->dev, LPS22H_CTRL_REG1, &reg1, 1);
    busReadRegisterBuffer(&baro->dev, LPS22H_CTRL_REG2, &reg2, 1);
    
    cliPrintf("Chip ID: 0x%02X (should be 0xB1)\r\n", id);
    cliPrintf("CTRL_REG1: 0x%02X\r\n", reg1);
    cliPrintf("CTRL_REG2: 0x%02X\r\n", reg2);
    
    // Step 2: Trigger measurement
    cliPrintf("Triggering measurement...\r\n");
    busWriteRegister(&baro->dev, LPS22H_CTRL_REG2, LPS22H_CTRL_REG2_ONE_SHOT);
    
    // Step 3: Wait for data ready
    cliPrintf("Waiting for data ready...\r\n");
    uint32_t startTime = millis();
    bool dataReady = false;
    
    while ((millis() - startTime) < 100) { // Wait max 100ms
        busReadRegisterBuffer(&baro->dev, LPS22H_STATUS, &status, 1);
        cliPrintf("Status register: 0x%02X\r\n", status);
        
        if (status & LPS22H_STATUS_P_DA) {
            dataReady = true;
            cliPrintf("Data ready! Time taken: %dms\r\n", (int)(millis() - startTime));
            break;
        }
        delay(5);
    }
    
    if (!dataReady) {
        cliPrintf("Timeout! Data not ready\r\n");
        return;
    }
    
    // Step 4: Read raw data
    busReadRegisterBuffer(&baro->dev, LPS22H_PRESSURE_OUT_XL, raw_data, LPS22H_DATA_FRAME_SIZE);
    cliPrintf("Raw data: [0x%02X,0x%02X,0x%02X,0x%02X,0x%02X]\r\n", 
              raw_data[0], raw_data[1], raw_data[2], raw_data[3], raw_data[4]);
    
    // Step 5: Calculate 24-bit pressure and 16-bit temperature values
    int32_t raw_pressure = (int32_t)(raw_data[0] | raw_data[1] << 8 | raw_data[2] << 16);
    // Add sign extension
    if (raw_pressure & 0x800000) {
        raw_pressure |= 0xFF000000;
    }
    
    int16_t raw_temp = (int16_t)(raw_data[3] | raw_data[4] << 8);
    
    cliPrintf("Parsed 24-bit pressure: %d (0x%08X)\r\n", raw_pressure, raw_pressure);
    cliPrintf("Parsed 16-bit temperature: %d (0x%04X)\r\n", raw_temp, (uint16_t)raw_temp);
    
    // Step 6: Apply conversion formulas
    // Fix all calculations to use double throughout
    float current_p_hpa = (raw_pressure * 100.0) / (16.0 * 256.0);
    float correct_p_hpa = (raw_pressure / 4096.0) * 100.0;
    
    int32_t current_p_pa = (int32_t)(current_p_hpa * 100.0);
    int32_t correct_p_pa = (int32_t)(correct_p_hpa * 100.0);
    
    cliPrintf("Current formula pressure: %d Pa (%.2f hPa)\r\n", 
              current_p_pa, (double)current_p_hpa);
    cliPrintf("Correct formula pressure: %d Pa (%.2f hPa)\r\n", 
              correct_p_pa, (double)correct_p_hpa);
    
    // Step 7: Calculate using driver functions
    lps22h_up = raw_pressure;
    lps22h_ut = raw_temp;
    lps22hCalculate(&pressure, &temperature);
    
    float driver_p_hpa = pressure / 100.0;
    float driver_t_c = temperature / 100.0;
    
    cliPrintf("Driver calculated pressure: %d Pa (%.2f hPa)\r\n", 
              pressure, (double)driver_p_hpa);
    cliPrintf("Driver calculated temperature: %.2f C\r\n", 
              (double)driver_t_c);
    
    cliPrintf("LPS22H Debug End\r\n");
}
*/
#endif
