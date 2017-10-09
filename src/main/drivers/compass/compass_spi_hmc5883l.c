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
#include <string.h>

#include <platform.h>

#include "drivers/io.h"
#include "drivers/bus_spi.h"

#include "compass.h"
#include "compass_hmc5883l.h"

#ifdef USE_MAG_SPI_HMC5883

#define DISABLE_HMC5883      IOHi(hmc5883CsPin)
#define ENABLE_HMC5883       IOLo(hmc5883CsPin)

static IO_t hmc5883CsPin = IO_NONE;

bool hmc5883SpiWriteCommand(uint8_t reg, uint8_t data)
{
    uint8_t buf[32];
    buf[0] = reg & 0x7F;
    buf[1] = data;

    ENABLE_HMC5883;

    //spiTransferByte(HMC5883_SPI_INSTANCE, reg & 0x7F);
    //spiTransferByte(HMC5883_SPI_INSTANCE, data);
    spiTransfer(HMC5883_SPI_INSTANCE, buf, NULL, 2);
    DISABLE_HMC5883;

    return true;
}

bool hmc5883SpiReadCommand(uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t buf[32];

    buf[0] = reg | 0x80 | 0x40;

    ENABLE_HMC5883;
    spiTransfer(HMC5883_SPI_INSTANCE, buf, buf, length + 1);
    DISABLE_HMC5883;

    memcpy(data, &buf[1], length);

    return true;
}

void hmc5883SpiInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

    hmc5883CsPin = IOGetByTag(IO_TAG(HMC5883_CS_PIN));
    IOInit(hmc5883CsPin, OWNER_COMPASS_CS, 0);
    IOConfigGPIO(hmc5883CsPin, IOCFG_OUT_PP);

    DISABLE_HMC5883;

    spiSetDivisor(HMC5883_SPI_INSTANCE, SPI_CLOCK_STANDARD);

    hardwareInitialised = true;
}
#endif
