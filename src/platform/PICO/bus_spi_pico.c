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

/*
 * Clock divider code based on pico-sdk/src/rp2_common/hardware_spi.c
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPI
//#define TESTING_NO_DMA 1

#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_def.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "pg/bus_spi.h"

#define SPI_SPEED_20MHZ 20000000
#define SPI_DATAWIDTH 8
#define SPI_DMA_THRESHOLD 8

const spiHardware_t spiHardware[] = {
    {
        .device = SPIDEV_0,
        .reg = SPI0,
        .sckPins = {
            { DEFIO_TAG_E(PA2) },
            { DEFIO_TAG_E(PA6) },
            { DEFIO_TAG_E(PA18) },
            { DEFIO_TAG_E(PA22) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA34) },
            { DEFIO_TAG_E(PA38) },
#endif
        },
        .misoPins = {
            { DEFIO_TAG_E(PA0) },
            { DEFIO_TAG_E(PA4) },
            { DEFIO_TAG_E(PA16) },
            { DEFIO_TAG_E(PA20) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA32) },
            { DEFIO_TAG_E(PA36) },
#endif
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA3) },
            { DEFIO_TAG_E(PA7) },
            { DEFIO_TAG_E(PA19) },
            { DEFIO_TAG_E(PA23) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA35) },
            { DEFIO_TAG_E(PA39) },
#endif
        },
    },
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA10) },
            { DEFIO_TAG_E(PA14) },
            { DEFIO_TAG_E(PA26) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA30) },
            { DEFIO_TAG_E(PA42) },
            { DEFIO_TAG_E(PA46) },
#endif
        },
        .misoPins = {
            { DEFIO_TAG_E(PA8) },
            { DEFIO_TAG_E(PA12) },
            { DEFIO_TAG_E(PA24) },
            { DEFIO_TAG_E(PA28) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA40) },
            { DEFIO_TAG_E(PA44) },
#endif
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA11) },
            { DEFIO_TAG_E(PA15) },
            { DEFIO_TAG_E(PA27) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA31) },
            { DEFIO_TAG_E(PA43) },
            { DEFIO_TAG_E(PA47) },
#endif
        },
    },
};

extern busDevice_t spiBusDevice[SPIDEV_COUNT];

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

static void spiSetClockFromSpeed(spi_inst_t *spi, uint16_t speed)
{
    uint32_t freq = spiCalculateClock(speed);
    spi_set_baudrate(spi, freq);
}

/*
  
enum spi_cpha_t { SPI_CPHA_0 = 0, SPI_CPHA_1 = 1 }
Enumeration of SPI CPHA (clock phase) values.

enum spi_cpol_t { SPI_CPOL_0 = 0, SPI_CPOL_1 = 1 }
Enumeration of SPI CPOL (clock polarity) values.

enum spi_order_t { SPI_LSB_FIRST = 0, SPI_MSB_FIRST = 1 }
Enumeration of SPI bit-order values.


static void spi_set_format (spi_inst_t * spi, uint data_bits, spi_cpol_t cpol, spi_cpha_t cpha, __unused spi_order_t order) [inline], [static]

Configure SPI.

Configure how the SPI serialises and deserialises data on the wire

Parameters

spi
SPI instance specifier, either spi0 or spi1

data_bits
Number of data bits per transfer. Valid values 4..16.

cpol
SSPCLKOUT polarity, applicable to Motorola SPI frame format only.

cpha
SSPCLKOUT phase, applicable to Motorola SPI frame format only

order
Must be SPI_MSB_FIRST, no other values supported on the PL022


*/


void spiInitDevice(SPIDevice device)
{
    const spiDevice_t *spi = &spiDevice[device];

    if (!spi->dev) {
        return;
    }

    // Set owners
    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    spi_init(SPI_INST(spi->dev), SPI_SPEED_20MHZ);

    gpio_set_function(IO_PINBYTAG(spi->miso), GPIO_FUNC_SPI);
    gpio_set_function(IO_PINBYTAG(spi->mosi), GPIO_FUNC_SPI);
    gpio_set_function(IO_PINBYTAG(spi->sck), GPIO_FUNC_SPI);
}

void spiInternalStopDMA (const extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;

    if (dmaRx && dma_channel_is_busy(dmaRx->channel)) {
        // Abort active DMA -  this should never happen
        dma_channel_abort(dmaRx->channel);
    }

    if (dmaTx && dma_channel_is_busy(dmaTx->channel)) {
        // Abort active DMA -  this should never happen as Tx should complete before Rx
        dma_channel_abort(dmaTx->channel);
    }
}


// Interrupt handler for SPI receive DMA completion
FAST_IRQ_HANDLER static void spiRxIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    const extDevice_t *dev = (const extDevice_t *)descriptor->userParam;

    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;

    if (bus->curSegment->negateCS) {
        // Negate Chip Select
        IOHi(dev->busType_u.spi.csnPin);
    }

    spiInternalStopDMA(dev);

    spiIrqHandler(dev);
}

extern dmaChannelDescriptor_t dmaDescriptors[];

void spiInitBusDMA(void)
{
    for (uint32_t device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = &spiBusDevice[device];
        int32_t channel_tx;
        int32_t channel_rx;

        if (bus->busType != BUS_TYPE_SPI) {
            // This bus is not in use
            continue;
        }

        channel_tx = dma_claim_unused_channel(true);
        if (channel_tx == -1) {
            // no more available channels so give up
            return;
        }
        channel_rx = dma_claim_unused_channel(true);
        if (channel_rx == -1) {
            // no more available channels so give up, first releasing the one
            // channel we did claim
            dma_channel_unclaim(channel_tx);
            return;
        }

        if (!dmaAllocate(DMA_CHANNEL_TO_IDENTIFIER(channel_tx), OWNER_SPI_SDO, device + 1) ||
            !dmaAllocate(DMA_CHANNEL_TO_IDENTIFIER(channel_rx), OWNER_SPI_SDI, device + 1)) {
            // This should never happen if all allocated channels are claimed
            dma_channel_unclaim(channel_tx);
            dma_channel_unclaim(channel_rx);
            return;
        }

        bus->dmaTx = &dmaDescriptors[DMA_CHANNEL_TO_INDEX(channel_tx)];
        bus->dmaTx->channel = channel_tx;

        bus->dmaRx = &dmaDescriptors[DMA_CHANNEL_TO_INDEX(channel_rx)];
        bus->dmaRx->channel = channel_rx;

        // The transaction concludes when the data has been received which will be after transmission is complete
        dmaSetHandler(DMA_CHANNEL_TO_IDENTIFIER(bus->dmaRx->channel), spiRxIrqHandler, NVIC_PRIO_SPI_DMA, 0);

        // We got the required resources, so we can use DMA on this bus
        bus->useDMA = true;
    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    //TODO: implement
    UNUSED(descriptor);
}

bool spiInternalReadWriteBufPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    // TODO optimise with 16-bit transfers as per stm bus_spi_ll code
    int bytesProcessed = spi_write_read_blocking(SPI_INST(instance), txData, rxData, len);
    return bytesProcessed == len;
}

// Initialise DMA before first segment transfer
void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
#ifndef USE_DMA
    UNUSED(dev);
    UNUSED(preInit);
#else
    UNUSED(preInit);

    busDevice_t *bus = dev->bus;

    volatile busSegment_t *segment = bus->curSegment;

    const spiDevice_t *spi = &spiDevice[spiDeviceByInstance(dev->bus->busType_u.spi.instance)];
    dma_channel_config config = dma_channel_get_default_config(dev->bus->dmaTx->channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, spi_get_dreq(SPI_INST(spi->dev), true));

    dma_channel_configure(dev->bus->dmaTx->channel, &config, &spi_get_hw(SPI_INST(spi->dev))->dr, segment->u.buffers.txData, 0, false);

    config = dma_channel_get_default_config(dev->bus->dmaRx->channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config, spi_get_dreq(SPI_INST(spi->dev), false));

    dma_channel_configure(dev->bus->dmaRx->channel, &config, segment->u.buffers.rxData, &spi_get_hw(SPI_INST(spi->dev))->dr, 0, false);
#endif
}

// Start DMA transfer for the current segment
void spiInternalStartDMA(const extDevice_t *dev)
{
#ifndef USE_DMA
    UNUSED(dev);
#else
    dev->bus->dmaRx->userParam = (uint32_t)dev;

    // TODO check correct, was len + 1 now len
    dma_channel_set_trans_count(dev->bus->dmaTx->channel, dev->bus->curSegment->len, false);
    dma_channel_set_trans_count(dev->bus->dmaRx->channel, dev->bus->curSegment->len, false);

    dma_start_channel_mask((1 << dev->bus->dmaTx->channel) | (1 << dev->bus->dmaRx->channel));
#endif
}
    
// DMA transfer setup and start
void spiSequenceStart(const extDevice_t *dev)
{
    //TODO: implementation for PICO
  // base on STM32/bus_spi_ll.c
  
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = bus->busType_u.spi.instance;
    spiDevice_t *spi = &spiDevice[spiDeviceByInstance(instance)];
    bool dmaSafe = dev->useDMA;
#if TESTING_NO_DMA
    dmaSafe = false;
#endif
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    // 
    bus->initSegment = true;

    
    // Switch bus speed
    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        spiSetClockFromSpeed(SPI_INST(instance), dev->busType_u.spi.speed);
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    // Switch SPI clock polarity/phase if necessary
    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        uint8_t sckPin = IO_PINBYTAG(spi->sck);
        gpio_set_slew_rate(sckPin, GPIO_SLEW_RATE_FAST);
        // Betaflight busDevice / SPI supports modes 0 (leadingEdge True), 3 (leadingEdge False)
        // 0: (CPOL 0, CPHA 0)
        // 3: (CPOL 1, CPHA 1)
        if (dev->busType_u.spi.leadingEdge) {
            spi_set_format(SPI_INST(instance), SPI_DATAWIDTH, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
        }
        else {
            spi_set_format(SPI_INST(instance), SPI_DATAWIDTH, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        }

        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    // NB for RP2350 targets, heap and stack will be in SRAM (single-cycle),
    // so there are no cache issues with DMA.
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        segmentCount++;
        xferLen += checkSegment->len;
    }

    // Use DMA if possible
    // If there are more than one segments, or a single segment with negateCS negated in the list terminator then force DMA irrespective of length
    if (bus->useDMA && dmaSafe && ((segmentCount > 1) ||
                                   (xferLen >= SPI_DMA_THRESHOLD) ||
                                   !bus->curSegment[segmentCount].negateCS)) {
        spiProcessSegmentsDMA(dev);
    } else {
        spiProcessSegmentsPolled(dev);
    }
}

uint16_t spiCalculateDivider(uint32_t freq)
{
    /*
      SPI clock is set in Betaflight code by calling spiSetClkDivisor, which records a uint16_t value into a .speed field.
      In order to maintain this code (for simplicity), record the prescale and postdiv numbers as calculated in
      pico-sdk/src/rp2_common/hardware_spi.c: spi_set_baudrate()

      prescale and postdiv are in range 1..255 and are packed into the return value.
    */

    uint32_t spiClock = clock_get_hz(clk_peri);
    uint32_t prescale, postdiv;
    // Find smallest prescale value which puts output frequency in range of
    // post-divide. Prescale is an even number from 2 to 254 inclusive.
    for (prescale = 2; prescale <= 254; prescale += 2) {
        if (spiClock < prescale * 256 * (uint64_t) freq)
            break;
    }
    if (prescale > 254) {
      prescale = 254;
    }

    // Find largest post-divide which makes output <= freq. Post-divide is
    // an integer in the range 1 to 256 inclusive.
    for (postdiv = 256; postdiv > 1; --postdiv) {
        if (spiClock / (prescale * (postdiv - 1)) > freq)
            break;
    }

    // Store prescale, (postdiv - 1), both in range 0 to 255.
    return (uint16_t)((prescale << 8) + (postdiv - 1));
}

uint32_t spiCalculateClock(uint16_t speed)
{
    /*
      speed contains packed values of prescale and postdiv.
      Retrieve a frequency which will recreate the same prescale and postdiv on a call to spi_set_baudrate().
    */
    uint32_t spiClock = clock_get_hz(clk_peri);
    uint32_t prescale = speed >> 8;
    uint32_t postdivMinusOne = speed & 0xFF;

    // Set freq to reverse the calculation, so that we would end up with the same prescale and postdiv,
    // hence the same frequency as if we had requested directly from spiCalculateDivider().
    uint32_t freq = 1 + (spiClock/prescale)/(postdivMinusOne + 1);

    return freq;
}


#endif
