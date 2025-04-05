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
#include "drivers/io_def.h"
#include "drivers/io_impl.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "pg/bus_spi.h"

#define SPI_SPEED_20MHZ 2000000

const spiHardware_t spiHardware[] = {
#ifdef RP2350B
    {
        .device = SPIDEV_0,
        .reg = SPI0,
        .sckPins = {
            { DEFIO_TAG_E(P6) },
            { DEFIO_TAG_E(P18) },
            { DEFIO_TAG_E(P22) },
        },
        .misoPins = {
            { DEFIO_TAG_E(P0) },
            { DEFIO_TAG_E(P4) },
            { DEFIO_TAG_E(P16) },
            { DEFIO_TAG_E(P20) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(P3) },
            { DEFIO_TAG_E(P7) },
            { DEFIO_TAG_E(P19) },
            { DEFIO_TAG_E(P23) },
        },
    },
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(P10) },
            { DEFIO_TAG_E(P14) },
        },
        .misoPins = {
            { DEFIO_TAG_E(P12) },
            { DEFIO_TAG_E(P28) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(P11) },
            { DEFIO_TAG_E(P15) },
        },
    },
#endif
};

void spiPinConfigure(const struct spiPinConfig_s *pConfig)
{
    for (size_t hwindex = 0 ; hwindex < ARRAYLEN(spiHardware) ; hwindex++) {
        const spiHardware_t *hw = &spiHardware[hwindex];

        if (!hw->reg) {
            continue;
        }

        SPIDevice device = hw->device;
        spiDevice_t *pDev = &spiDevice[device];

        for (int pindex = 0 ; pindex < MAX_SPI_PIN_SEL ; pindex++) {
            if (pConfig[device].ioTagSck == hw->sckPins[pindex].pin) {
                pDev->sck = hw->sckPins[pindex].pin;
            }
            if (pConfig[device].ioTagMiso == hw->misoPins[pindex].pin) {
                pDev->miso = hw->misoPins[pindex].pin;
            }
            if (pConfig[device].ioTagMosi == hw->mosiPins[pindex].pin) {
                pDev->mosi = hw->mosiPins[pindex].pin;
            }
        }

        if (pDev->sck && pDev->miso && pDev->mosi) {
            pDev->dev = hw->reg;
            pDev->leadingEdge = false;
        }
    }
}

static spi_inst_t *getSpiInstanceByDevice(SPI0_Type *spi)
{
    if (spi == SPI0) {
        return spi0;
    } else if (spi == SPI1) {
        return spi1;
    }
    return NULL;
}

void spiInitDevice(SPIDevice device)
{
    const spiDevice_t *spi = &spiDevice[device];

    if (!spi->dev) {
        return;
    }

    spi_init(getSpiInstanceByDevice(spi->dev), SPI_SPEED_20MHZ);

    gpio_set_function(IO_PINBYTAG(spi->miso), GPIO_FUNC_SPI);
    gpio_set_function(IO_PINBYTAG(spi->mosi), GPIO_FUNC_SPI);
    gpio_set_function(IO_PINBYTAG(spi->sck), GPIO_FUNC_SPI);
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
    UNUSED(preInit);

    int dma_tx = dma_claim_unused_channel(true);
    int dma_rx = dma_claim_unused_channel(true);

    dev->bus->dmaTx->channel = dma_tx;
    dev->bus->dmaRx->channel = dma_rx;
    dev->bus->dmaTx->irqHandlerCallback = NULL;
    dev->bus->dmaRx->irqHandlerCallback = spiInternalResetStream;

    const spiDevice_t *spi = &spiDevice[spiDeviceByInstance(dev->bus->busType_u.spi.instance)];
    dma_channel_config config = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_dreq(&config, spi_get_dreq(SPI_INST(spi->dev), true));

    dma_channel_configure(dma_tx, &config, &spi_get_hw(SPI_INST(spi->dev))->dr, dev->txBuf, 0, false);

    config = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_dreq(&config, spi_get_dreq(SPI_INST(spi->dev), false));

    dma_channel_configure(dma_rx, &config, dev->rxBuf, &spi_get_hw(SPI_INST(spi->dev))->dr, 0, false);
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    dma_channel_set_trans_count(dev->bus->dmaTx->channel, dev->bus->curSegment->len + 1, false);
    dma_channel_set_trans_count(dev->bus->dmaRx->channel, dev->bus->curSegment->len + 1, false);

    dma_channel_start(dev->bus->dmaTx->channel);
    dma_channel_start(dev->bus->dmaRx->channel);
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
    /*
        Divider is probably not needed for the PICO as the baud rate on the
        SPI bus can be set directly.

        In anycase max SPI clock is half the system clock frequency.
        Therefore the minimum divider is 2.
    */
    return MAX(2, (((clock_get_hz(clk_sys) + (freq / 2)) / freq) + 1) & ~1);
}
#endif
