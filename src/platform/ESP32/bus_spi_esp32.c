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

#ifdef USE_SPI

#include "common/utils.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/io.h"

const spiHardware_t spiHardware[SPIDEV_COUNT] = { 0 };

void spiPinConfigure(const struct spiPinConfig_s *pConfig)
{
    UNUSED(pConfig);
    // TODO: configure SPI pins via ESP-IDF spi_bus_initialize()
}

void spiPreinit(void)
{
    // NOOP
}

void spiPreinitRegister(ioTag_t iotag, uint32_t iocfg, uint8_t init)
{
    UNUSED(iotag);
    UNUSED(iocfg);
    UNUSED(init);
}

void spiPreinitByIO(IO_t io)
{
    UNUSED(io);
}

void spiPreinitByTag(ioTag_t tag)
{
    UNUSED(tag);
}

void spiInitDevice(spiDevice_e device)
{
    UNUSED(device);
    // TODO: spi_bus_initialize()
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    UNUSED(bus);
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    UNUSED(descriptor);
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    UNUSED(dev);
}

void spiInternalStopDMA(const extDevice_t *dev)
{
    UNUSED(dev);
}

void spiInternalInitStream(const extDevice_t *dev, volatile busSegment_t *segment)
{
    UNUSED(dev);
    UNUSED(segment);
}

bool spiInternalReadWriteBufPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    UNUSED(instance);
    UNUSED(txData);
    UNUSED(rxData);
    UNUSED(len);
    return false;
}

void spiSequenceStart(const extDevice_t *dev)
{
    UNUSED(dev);
}

uint16_t spiCalculateDivider(uint32_t freq)
{
    UNUSED(freq);
    return 0;
}

void spiInitBusDMA(void)
{
}

#endif // USE_SPI
