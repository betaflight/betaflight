/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// This file is copied with modifications from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_RX_SPI

#include "build/build_config.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "drivers/system.h"

#include "rx_spi.h"

#define DISABLE_RX()    {IOHi(DEFIO_IO(RX_NSS_PIN));}
#define ENABLE_RX()     {IOLo(DEFIO_IO(RX_NSS_PIN));}

void rxSpiDeviceInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

    const SPIDevice rxSPIDevice = spiDeviceByInstance(RX_SPI_INSTANCE);
    const IO_t rxCsPin = DEFIO_IO(RX_NSS_PIN);
    IOInit(rxCsPin, OWNER_RX_SPI_CS, rxSPIDevice + 1);
    IOConfigGPIO(rxCsPin, SPI_IO_CS_CFG);

    DISABLE_RX();

#ifdef RX_SPI_INSTANCE
    spiSetDivisor(RX_SPI_INSTANCE, SPI_CLOCK_STANDARD);
#endif
    hardwareInitialised = true;
}

uint8_t rxSpiTransferByte(uint8_t data)
{
#ifdef RX_SPI_INSTANCE
    return spiTransferByte(RX_SPI_INSTANCE, data);
#else
    return 0;
#endif
}

uint8_t rxSpiWriteByte(uint8_t data)
{
    ENABLE_RX();
    const uint8_t ret = rxSpiTransferByte(data);
    DISABLE_RX();
    return ret;
}

uint8_t rxSpiWriteCommand(uint8_t command, uint8_t data)
{
    ENABLE_RX();
    const uint8_t ret = rxSpiTransferByte(command);
    rxSpiTransferByte(data);
    DISABLE_RX();
    return ret;
}

uint8_t rxSpiWriteCommandMulti(uint8_t command, const uint8_t *data, uint8_t length)
{
    ENABLE_RX();
    const uint8_t ret = rxSpiTransferByte(command);
    for (uint8_t i = 0; i < length; i++) {
        rxSpiTransferByte(data[i]);
    }
    DISABLE_RX();
    return ret;
}

uint8_t rxSpiReadCommand(uint8_t command, uint8_t data)
{
    ENABLE_RX();
    rxSpiTransferByte(command);
    const uint8_t ret = rxSpiTransferByte(data);
    DISABLE_RX();
    return ret;
}

uint8_t rxSpiReadCommandMulti(uint8_t command, uint8_t commandData, uint8_t *retData, uint8_t length)
{
    ENABLE_RX();
    const uint8_t ret = rxSpiTransferByte(command);
    for (uint8_t i = 0; i < length; i++) {
        retData[i] = rxSpiTransferByte(commandData);
    }
    DISABLE_RX();
    return ret;
}
#endif
