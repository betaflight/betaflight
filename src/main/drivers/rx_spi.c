/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// This file is copied with modifications from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#ifdef USE_RX_SPI

#include "build/build_config.h"

#include "bus_spi.h"
#include "bus_spi_soft.h"
#include "gpio.h"
#include "io.h"
#include "io_impl.h"
#include "rcc.h"
#include "rx_spi.h"
#include "system.h"

#define DISABLE_RX()    {IOHi(DEFIO_IO(RX_NSS_PIN));}
#define ENABLE_RX()     {IOLo(DEFIO_IO(RX_NSS_PIN));}

#ifdef USE_RX_SOFTSPI
static const softSPIDevice_t softSPIDevice = {
    .sckTag = IO_TAG(RX_SCK_PIN),
    .mosiTag = IO_TAG(RX_MOSI_PIN),
    .misoTag = IO_TAG(RX_MISO_PIN),
    // Note: Nordic Semiconductor uses 'CSN', STM uses 'NSS'
    .nssTag = IO_TAG(RX_NSS_PIN),
};
static bool useSoftSPI = false;
#endif // USE_RX_SOFTSPI

void rxSpiDeviceInit(rx_spi_type_e spiType)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

#ifdef USE_RX_SOFTSPI
    if (spiType == RX_SPI_SOFTSPI) {
        useSoftSPI = true;
        softSpiInit(&softSPIDevice);
    }
    const SPIDevice rxSPIDevice = SOFT_SPIDEV_1;
#else
    UNUSED(spiType);
    const SPIDevice rxSPIDevice = spiDeviceByInstance(RX_SPI_INSTANCE);
    IOInit(DEFIO_IO(RX_NSS_PIN), OWNER_SPI_CS, rxSPIDevice + 1);
#endif // USE_RX_SOFTSPI

    DISABLE_RX();

#ifdef RX_SPI_INSTANCE
    spiSetDivisor(RX_SPI_INSTANCE, SPI_CLOCK_STANDARD);
#endif
    hardwareInitialised = true;
}

uint8_t rxSpiTransferByte(uint8_t data)
{
#ifdef USE_RX_SOFTSPI
    if (useSoftSPI) {
        return softSpiTransferByte(&softSPIDevice, data);
    } else
#endif
    {
#ifdef RX_SPI_INSTANCE
        return spiTransferByte(RX_SPI_INSTANCE, data);
#else
        return 0;
#endif
    }
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

