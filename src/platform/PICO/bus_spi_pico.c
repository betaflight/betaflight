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
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"

void spiPinConfigure(const struct spiPinConfig_s *pConfig)
{
    UNUSED(pConfig);
}

void spiInitDevice(SPIDevice device)
{
    //TODO: implement
    UNUSED(device);
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    //TODO: implement
    UNUSED(bus);
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    //TODO: implement
    UNUSED(descriptor);
}

void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
    //TODO: implement
    UNUSED(dev);
    UNUSED(preInit);
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    //TODO: implement
    UNUSED(dev);
}

void spiInternalStopDMA (const extDevice_t *dev)
{
    //TODO: implement
    UNUSED(dev);
}

// DMA transfer setup and start
void spiSequenceStart(const extDevice_t *dev)
{
    //TODO: implement
    UNUSED(dev);
}

uint16_t spiCalculateDivider(uint32_t freq)
{
    UNUSED(freq);
    return 0;
}
#endif
