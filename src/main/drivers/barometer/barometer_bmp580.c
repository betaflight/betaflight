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
 *
 * BMP580 Driver
 *
 * References:
 * BMP580 datasheet - https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp580/
 * BMP5-Sensor-API - https://github.com/boschsensortec/BMP5-Sensor-API
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_BARO) && (defined(USE_BARO_BMP580) || defined(USE_BARO_SPI_BMP580))

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/barometer/barometer.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "barometer_bmp580.h"

// 10 MHz max SPI frequency
#define BMP580_MAX_SPI_CLK_HZ 10000000

// I2C addresses
#define BMP580_I2C_ADDR_PRIMARY     (0x47)  // SDO = HIGH (default)
#define BMP580_I2C_ADDR_SECONDARY   (0x46)  // SDO = LOW

// Chip IDs
#define BMP580_CHIP_ID              (0x50)
#define BMP581_CHIP_ID              (0x51)

// Register addresses
#define BMP580_REG_CHIP_ID          (0x01)
#define BMP580_REG_REV_ID           (0x02)
#define BMP580_REG_CHIP_STATUS      (0x11)
#define BMP580_REG_DRIVE_CONFIG     (0x13)
#define BMP580_REG_INT_CONFIG       (0x14)
#define BMP580_REG_INT_SOURCE       (0x15)
#define BMP580_REG_FIFO_CONFIG      (0x16)
#define BMP580_REG_FIFO_COUNT       (0x17)
#define BMP580_REG_FIFO_SEL         (0x18)
#define BMP580_REG_TEMP_DATA_XLSB   (0x1D)
#define BMP580_REG_TEMP_DATA_LSB    (0x1E)
#define BMP580_REG_TEMP_DATA_MSB    (0x1F)
#define BMP580_REG_PRESS_DATA_XLSB  (0x20)
#define BMP580_REG_PRESS_DATA_LSB   (0x21)
#define BMP580_REG_PRESS_DATA_MSB   (0x22)
#define BMP580_REG_INT_STATUS       (0x27)
#define BMP580_REG_STATUS           (0x28)
#define BMP580_REG_FIFO_DATA        (0x29)
#define BMP580_REG_NVM_ADDR         (0x2B)
#define BMP580_REG_NVM_DATA_LSB     (0x2C)
#define BMP580_REG_NVM_DATA_MSB     (0x2D)
#define BMP580_REG_DSP_CONFIG       (0x30)
#define BMP580_REG_DSP_IIR          (0x31)
#define BMP580_REG_OOR_THR_P_LSB    (0x32)
#define BMP580_REG_OOR_THR_P_MSB    (0x33)
#define BMP580_REG_OOR_RANGE        (0x34)
#define BMP580_REG_OOR_CONFIG       (0x35)
#define BMP580_REG_OSR_CONFIG       (0x36)
#define BMP580_REG_ODR_CONFIG       (0x37)
#define BMP580_REG_OSR_EFF          (0x38)
#define BMP580_REG_CMD              (0x7E)

// Commands
#define BMP580_CMD_NOP              (0x00)
#define BMP580_CMD_EXTMODE_EN_MID   (0x34)
#define BMP580_CMD_EXTMODE_EN_LOW   (0xB4)
#define BMP580_CMD_EXTMODE_EN_HIGH  (0xF4)
#define BMP580_CMD_FIFO_FLUSH       (0xB0)
#define BMP580_CMD_SOFT_RESET       (0xB6)

// Power modes (ODR_CONFIG bits 0-1)
#define BMP580_MODE_STANDBY         (0x00)
#define BMP580_MODE_NORMAL          (0x01)  // continuous with ODR
#define BMP580_MODE_FORCED          (0x02)  // single shot, returns to standby
#define BMP580_MODE_NON_STOP        (0x03)

// ODR_CONFIG register bitfields
#define BMP580_ODR_STANDBY          (0x00)  // Standby mode
#define BMP580_ODR_240_HZ           (0x00 << 2)
#define BMP580_ODR_218_HZ           (0x01 << 2)
#define BMP580_ODR_199_HZ           (0x02 << 2)
#define BMP580_ODR_179_HZ           (0x03 << 2)
#define BMP580_ODR_160_HZ           (0x04 << 2)
#define BMP580_ODR_149_HZ           (0x05 << 2)
#define BMP580_ODR_140_HZ           (0x06 << 2)
#define BMP580_ODR_129_HZ           (0x07 << 2)
#define BMP580_ODR_120_HZ           (0x08 << 2)
#define BMP580_ODR_110_HZ           (0x09 << 2)
#define BMP580_ODR_100_HZ           (0x0A << 2)
#define BMP580_ODR_89_HZ            (0x0B << 2)
#define BMP580_ODR_80_HZ            (0x0C << 2)
#define BMP580_ODR_70_HZ            (0x0D << 2)
#define BMP580_ODR_60_HZ            (0x0E << 2)
#define BMP580_ODR_50_HZ            (0x0F << 2)
#define BMP580_ODR_45_HZ            (0x10 << 2)
#define BMP580_ODR_40_HZ            (0x11 << 2)
#define BMP580_ODR_35_HZ            (0x12 << 2)
#define BMP580_ODR_30_HZ            (0x13 << 2)
#define BMP580_ODR_25_HZ            (0x14 << 2)
#define BMP580_ODR_20_HZ            (0x15 << 2)
#define BMP580_ODR_15_HZ            (0x16 << 2)
#define BMP580_ODR_10_HZ            (0x17 << 2)
#define BMP580_ODR_5_HZ             (0x18 << 2)
#define BMP580_ODR_4_HZ             (0x19 << 2)
#define BMP580_ODR_3_HZ             (0x1A << 2)
#define BMP580_ODR_2_HZ             (0x1B << 2)
#define BMP580_ODR_1_HZ             (0x1C << 2)
#define BMP580_ODR_0_5_HZ           (0x1D << 2)
#define BMP580_ODR_0_25_HZ          (0x1E << 2)
#define BMP580_ODR_0_125_HZ         (0x1F << 2)

// OSR_CONFIG register bitfields
#define BMP580_OSR_PRESS_1X         (0x00)
#define BMP580_OSR_PRESS_2X         (0x01)
#define BMP580_OSR_PRESS_4X         (0x02)
#define BMP580_OSR_PRESS_8X         (0x03)
#define BMP580_OSR_PRESS_16X        (0x04)
#define BMP580_OSR_PRESS_32X        (0x05)
#define BMP580_OSR_PRESS_64X        (0x06)
#define BMP580_OSR_PRESS_128X       (0x07)

#define BMP580_OSR_TEMP_1X          (0x00 << 3)
#define BMP580_OSR_TEMP_2X          (0x01 << 3)
#define BMP580_OSR_TEMP_4X          (0x02 << 3)
#define BMP580_OSR_TEMP_8X          (0x03 << 3)
#define BMP580_OSR_TEMP_16X         (0x04 << 3)
#define BMP580_OSR_TEMP_32X         (0x05 << 3)
#define BMP580_OSR_TEMP_64X         (0x06 << 3)
#define BMP580_OSR_TEMP_128X        (0x07 << 3)

#define BMP580_OSR_PRESS_EN         (0x01 << 6)

// DSP_CONFIG register bitfields
#define BMP580_DSP_COMP_PRESS_TEMP  (0x03)  // Enable pressure and temperature compensation
#define BMP580_DSP_SHDW_SEL_IIR     (0x01 << 3)  // IIR output to shadow registers

// INT_CONFIG register bitfields
#define BMP580_INT_MODE_PULSED      (0x00)
#define BMP580_INT_MODE_LATCHED     (0x01)
#define BMP580_INT_POL_ACTIVE_LOW   (0x00 << 1)
#define BMP580_INT_POL_ACTIVE_HIGH  (0x01 << 1)
#define BMP580_INT_OD_PUSHPULL      (0x00 << 2)
#define BMP580_INT_OD_OPENDRAIN     (0x01 << 2)
#define BMP580_INT_EN               (0x01 << 3)

// INT_SOURCE register bitfields
#define BMP580_INT_SRC_DRDY         (0x01)
#define BMP580_INT_SRC_FIFO_FULL    (0x01 << 1)
#define BMP580_INT_SRC_FIFO_THS     (0x01 << 2)
#define BMP580_INT_SRC_OOR          (0x01 << 3)

// STATUS register bitfields
#define BMP580_STATUS_DRDY_PRESS    (0x01 << 5)
#define BMP580_STATUS_DRDY_TEMP     (0x01 << 6)

// INT_STATUS register bitfields
#define BMP580_INT_STATUS_DRDY      (0x01)
#define BMP580_INT_STATUS_FIFO_FULL (0x01 << 1)
#define BMP580_INT_STATUS_FIFO_THS  (0x01 << 2)
#define BMP580_INT_STATUS_OOR       (0x01 << 3)
#define BMP580_INT_STATUS_POR       (0x01 << 4)

// Oversampling settings for measurement
// 128X = maximum oversampling, lowest noise, ~10Hz update rate
// Good for stable altitude hold where low latency is not critical
#define BMP580_PRESSURE_OSR         BMP580_OSR_PRESS_128X
#define BMP580_TEMPERATURE_OSR      BMP580_OSR_TEMP_128X

// Data frame size: temperature (3 bytes) + pressure (3 bytes)
#define BMP580_DATA_FRAME_SIZE      6

// Chip ID
static uint8_t bmp580_chip_id = 0;

// Uncompensated pressure and temperature (raw 24-bit values)
static uint32_t bmp580_up = 0;
static uint32_t bmp580_ut = 0;

// DMA buffer for sensor data (extra byte for SPI dummy)
static DMA_DATA_ZERO_INIT uint8_t sensor_data_buffer[BMP580_DATA_FRAME_SIZE + 1];
static uint8_t *sensor_data = sensor_data_buffer;

// Debug: last raw values for troubleshooting
static uint32_t bmp580_last_raw_temp = 0;
static uint32_t bmp580_last_raw_press = 0;

static bool bmp580StartUT(baroDev_t *baro);
static bool bmp580GetUT(baroDev_t *baro);
static bool bmp580ReadUT(baroDev_t *baro);
static bool bmp580StartUP(baroDev_t *baro);
static bool bmp580GetUP(baroDev_t *baro);
static bool bmp580ReadUP(baroDev_t *baro);

static void bmp580Calculate(int32_t *pressure, int32_t *temperature);

static bool bmp580ReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (dev->bus->busType == BUS_TYPE_SPI) {
        // For SPI: first byte is dummy on BMP5xx read
        uint8_t buf[length + 1];
        bool ret = busReadRegisterBuffer(dev, reg, buf, length + 1);
        if (ret) {
            memcpy(data, buf + 1, length);
        }
        return ret;
    } else {
        return busReadRegisterBuffer(dev, reg, data, length);
    }
}

static void bmp580_extiHandler(extiCallbackRec_t *cb)
{
#ifdef DEBUG
    static uint32_t bmp580ExtiCallbackCounter = 0;
    bmp580ExtiCallbackCounter++;
#endif

    baroDev_t *baro = container_of(cb, baroDev_t, exti);

    // Clear interrupt status by reading INT_STATUS register
    uint8_t intStatus = 0;
    bmp580ReadRegisterBuffer(&baro->dev, BMP580_REG_INT_STATUS, &intStatus, 1);
}

static void bmp580BusInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_BMP580
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin);
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(BMP580_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
#endif
}

static void bmp580BusDeinit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_BMP580
    if (dev->bus->busType == BUS_TYPE_SPI) {
        ioPreinitByIO(dev->busType_u.spi.csnPin, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
    }
#else
    UNUSED(dev);
#endif
}

static bool bmp580BeginForcedMeasurement(const extDevice_t *dev)
{
    UNUSED(dev);
    // In normal mode, measurements run continuously - nothing to start
    return true;
}

bool bmp580Detect(const bmp580Config_t *config, baroDev_t *baro)
{
    delay(20);

    IO_t baroIntIO = IOGetByTag(config->eocTag);
    if (baroIntIO) {
        IOInit(baroIntIO, OWNER_BARO_EOC, 0);
        EXTIHandlerInit(&baro->exti, bmp580_extiHandler);
        EXTIConfig(baroIntIO, &baro->exti, NVIC_PRIO_BARO_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
        EXTIEnable(baroIntIO);
    }

    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    bmp580BusInit(dev);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default I2C address for BMP580
        dev->busType_u.i2c.address = BMP580_I2C_ADDR_PRIMARY;
        defaultAddressApplied = true;
    }

    if (dev->bus->busType == BUS_TYPE_SPI) {
        // For SPI, first byte is dummy on read
        sensor_data = sensor_data_buffer + 1;
    }

    // Read chip ID
    bmp580ReadRegisterBuffer(dev, BMP580_REG_CHIP_ID, &bmp580_chip_id, 1);

    if (bmp580_chip_id != BMP580_CHIP_ID && bmp580_chip_id != BMP581_CHIP_ID) {
        // Try secondary I2C address
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = BMP580_I2C_ADDR_SECONDARY;
            bmp580ReadRegisterBuffer(dev, BMP580_REG_CHIP_ID, &bmp580_chip_id, 1);

            if (bmp580_chip_id != BMP580_CHIP_ID && bmp580_chip_id != BMP581_CHIP_ID) {
                bmp580BusDeinit(dev);
                dev->busType_u.i2c.address = 0;
                return false;
            }
        } else {
            bmp580BusDeinit(dev);
            return false;
        }
    }

    busDeviceRegister(dev);

    // Soft reset
    busWriteRegister(dev, BMP580_REG_CMD, BMP580_CMD_SOFT_RESET);
    delay(5);  // Wait for reset to complete

    // Configure interrupt if available
    if (baroIntIO) {
        // Configure interrupt: pulsed, active high, push-pull, enabled
        uint8_t intConfig = BMP580_INT_MODE_PULSED |
                           BMP580_INT_POL_ACTIVE_HIGH |
                           BMP580_INT_OD_PUSHPULL |
                           BMP580_INT_EN;
        busWriteRegister(dev, BMP580_REG_INT_CONFIG, intConfig);

        // Enable data ready interrupt
        busWriteRegister(dev, BMP580_REG_INT_SOURCE, BMP580_INT_SRC_DRDY);
    }

    // Configure DSP: enable pressure and temperature compensation, IIR output
    busWriteRegister(dev, BMP580_REG_DSP_CONFIG,
        BMP580_DSP_COMP_PRESS_TEMP | BMP580_DSP_SHDW_SEL_IIR);

    // Configure oversampling
    busWriteRegister(dev, BMP580_REG_OSR_CONFIG,
        BMP580_PRESSURE_OSR | BMP580_TEMPERATURE_OSR | BMP580_OSR_PRESS_EN);

    // Set normal mode with 50Hz ODR for continuous measurement
    // This way data is always ready when we want to read it
    uint8_t odrConfig = BMP580_ODR_50_HZ | BMP580_MODE_NORMAL;
    busWriteRegister(dev, BMP580_REG_ODR_CONFIG, odrConfig);

    // Set up baro device callbacks
    // Temperature and pressure are read together, so UT callbacks are dummy
    baro->ut_delay = 0;
    baro->start_ut = bmp580StartUT;
    baro->get_ut = bmp580GetUT;
    baro->read_ut = bmp580ReadUT;

    // Only UP part executes, reads both temperature and pressure
    baro->start_up = bmp580StartUP;
    baro->get_up = bmp580GetUP;
    baro->read_up = bmp580ReadUP;

    // Calculate measurement delay based on oversampling settings
    // Base conversion time ~5ms, plus OSR overhead
    // From datasheet: typical with 8x press OSR and 1x temp OSR ≈ 10ms
    baro->up_delay = 15000;  // 15ms to be safe

    baro->calculate = bmp580Calculate;

    while (busBusy(&baro->dev, NULL));

    return true;
}

static bool bmp580StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy - temperature is read with pressure
    return true;
}

static bool bmp580ReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy - temperature is read with pressure
    return true;
}

static bool bmp580GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy - temperature is read with pressure
    return true;
}

static bool bmp580StartUP(baroDev_t *baro)
{
    // In normal mode, measurements run continuously - nothing to start
    return bmp580BeginForcedMeasurement(&baro->dev);
}

static bool bmp580ReadUP(baroDev_t *baro)
{
    // Use synchronous read for I2C - async reads can get stuck on PICO
    // Read temperature and pressure data (6 bytes starting from TEMP_DATA_XLSB)
    // Registers are: TEMP_XLSB (0x1D), TEMP_LSB (0x1E), TEMP_MSB (0x1F),
    //                PRESS_XLSB (0x20), PRESS_LSB (0x21), PRESS_MSB (0x22)
    return bmp580ReadRegisterBuffer(&baro->dev, BMP580_REG_TEMP_DATA_XLSB, 
        sensor_data, BMP580_DATA_FRAME_SIZE);
}

static bool bmp580GetUP(baroDev_t *baro)
{
    UNUSED(baro);
    
    // Parse temperature (24-bit, signed)
    bmp580_ut = (uint32_t)sensor_data[0] |
                ((uint32_t)sensor_data[1] << 8) |
                ((uint32_t)sensor_data[2] << 16);

    // Parse pressure (24-bit, unsigned)
    bmp580_up = (uint32_t)sensor_data[3] |
                ((uint32_t)sensor_data[4] << 8) |
                ((uint32_t)sensor_data[5] << 16);

    // Store for debug
    bmp580_last_raw_temp = bmp580_ut;
    bmp580_last_raw_press = bmp580_up;

    return true;
}

static void bmp580Calculate(int32_t *pressure, int32_t *temperature)
{
    // BMP580 outputs already compensated data
    // Temperature: raw value / 65536 = °C
    // Pressure: raw value / 64 = Pa

    // Handle signed temperature (24-bit to 32-bit sign extension)
    int32_t temp_raw = (int32_t)bmp580_ut;
    if (temp_raw & 0x800000) {  // If negative (sign bit set)
        temp_raw |= 0xFF000000;  // Sign extend to 32-bit
    }

    // Temperature in centidegrees (for Betaflight compatibility)
    // raw / 65536 * 100 = raw / 655.36 ≈ raw * 100 / 65536
    int64_t temp_centi = ((int64_t)temp_raw * 100) / 65536;

    // Pressure in Pa (Betaflight expects Pa * 256 for some baros, but using Pa directly)
    // raw / 64 = pressure in Pa
    // Betaflight barometer interface expects pressure in Pa
    uint32_t press_pa = bmp580_up / 64;

    if (temperature) {
        *temperature = (int32_t)temp_centi;
    }
    if (pressure) {
        *pressure = (int32_t)press_pa;
    }
}

#endif // defined(USE_BARO) && (defined(USE_BARO_BMP580) || defined(USE_BARO_SPI_BMP580))
