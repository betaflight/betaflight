/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/io.h"
#include "drivers/bus_spi.h"

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_ms56xx.h"

#if defined(USE_BARO_SPI_MS5611) || defined(USE_BARO_SPI_MS5607)

#define DISABLE_MS56XX(spiCsnPin)   IOHi(spiCsnPin)
#define ENABLE_MS56XX(spiCsnPin)    IOLo(spiCsnPin)

static IO_t spiCsnPin = IO_NONE;

bool ms56xxSpiWriteCommand(uint8_t reg, uint8_t data)
{
    ENABLE_MS56XX(spiCsnPin);
    spiTransferByte(MS56XX_SPI_INSTANCE, reg);
    spiTransferByte(MS56XX_SPI_INSTANCE, data);
    DISABLE_MS56XX(spiCsnPin);

    return true;
}

bool ms56xxSpiReadCommand(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_MS56XX(spiCsnPin);
    spiTransferByte(MS56XX_SPI_INSTANCE, reg);
    spiTransfer(MS56XX_SPI_INSTANCE, data, NULL, length);
    DISABLE_MS56XX(spiCsnPin);

    return true;
}

void ms56xxSpiInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

    spiCsnPin = IOGetByTag(IO_TAG(MS56XX_CS_PIN));
    IOInit(spiCsnPin, OWNER_BARO, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(spiCsnPin, IOCFG_OUT_PP);

    DISABLE_MS56XX(spiCsnPin);

    spiSetDivisor(MS56XX_SPI_INSTANCE, SPI_CLOCK_STANDARD);

    hardwareInitialised = true;
}
#endif
