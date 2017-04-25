/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/io.h"
#include "drivers/bus_spi.h"

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp280.h"

#ifdef USE_BARO_SPI_BMP280
#define DISABLE_BMP280       IOHi(bmp280CsPin)
#define ENABLE_BMP280        IOLo(bmp280CsPin)

extern int32_t bmp280_up;
extern int32_t bmp280_ut;

static IO_t bmp280CsPin = IO_NONE;

bool bmp280WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_BMP280;
    spiTransferByte(BMP280_SPI_INSTANCE, reg & 0x7F);
    spiTransferByte(BMP280_SPI_INSTANCE, data);
    DISABLE_BMP280;

    return true;
}

bool bmp280ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_BMP280;
    spiTransferByte(BMP280_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(BMP280_SPI_INSTANCE, data, NULL, length);
    DISABLE_BMP280;

    return true;
}

void bmp280SpiInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

    bmp280CsPin = IOGetByTag(IO_TAG(BMP280_CS_PIN));
    IOInit(bmp280CsPin, OWNER_BARO, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(bmp280CsPin, IOCFG_OUT_PP);

    DISABLE_BMP280;

    spiSetDivisor(BMP280_SPI_INSTANCE, SPI_CLOCK_STANDARD);

    hardwareInitialised = true;
}

void bmp280_spi_start_up(void)
{
    // start measurement
    // set oversampling + power mode (forced), and start sampling
    bmp280WriteRegister(BMP280_CTRL_MEAS_REG, BMP280_MODE);
}

void bmp280_spi_get_up(void)
{
    uint8_t data[BMP280_DATA_FRAME_SIZE];

    // read data from sensor
    bmp280ReadRegister(BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);
    bmp280_up = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
    bmp280_ut = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
}
#endif
