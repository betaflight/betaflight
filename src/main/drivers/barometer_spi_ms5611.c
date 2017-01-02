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

#include "io.h"
#include "bus_spi.h"

#include "barometer.h"
#include "barometer_ms5611.h"

#ifdef USE_BARO_SPI_MS5611

#define DISABLE_MS5611      IOHi(ms5611CsPin)
#define ENABLE_MS5611       IOLo(ms5611CsPin)

static IO_t ms5611CsPin = IO_NONE;

bool ms5611SpiWriteCommand(uint8_t reg, uint8_t data)
{
    ENABLE_MS5611;
    spiTransferByte(MS5611_SPI_INSTANCE, reg);
    spiTransferByte(MS5611_SPI_INSTANCE, data);
    DISABLE_MS5611;

    return true;
}

bool ms5611SpiReadCommand(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_MS5611;
    spiTransferByte(MS5611_SPI_INSTANCE, reg);
    spiTransfer(MS5611_SPI_INSTANCE, data, NULL, length);
    DISABLE_MS5611;

    return true;
}

void ms5611SpiInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

    ms5611CsPin = IOGetByTag(IO_TAG(MS5611_CS_PIN));
    IOInit(ms5611CsPin, OWNER_BARO, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(ms5611CsPin, IOCFG_OUT_PP);

    DISABLE_MS5611;

    spiSetDivisor(MS5611_SPI_INSTANCE, SPI_CLOCK_STANDARD);

    hardwareInitialised = true;
}
#endif
