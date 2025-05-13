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
#define TESTING_NO_DMA 1

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

#define SPI_SPEED_20MHZ 20000000
#define SPI_DATAWIDTH 8
#define SPI_DMA_THRESHOLD 8

const spiHardware_t spiHardware[] = {
    {
        .device = SPIDEV_0,
        .reg = SPI0,
        .sckPins = {
            { DEFIO_TAG_E(P2) },
            { DEFIO_TAG_E(P6) },
            { DEFIO_TAG_E(P18) },
            { DEFIO_TAG_E(P22) },
#ifdef RP2350B
            { DEFIO_TAG_E(P34) },
            { DEFIO_TAG_E(P38) },
#endif
        },
        .misoPins = {
            { DEFIO_TAG_E(P0) },
            { DEFIO_TAG_E(P4) },
            { DEFIO_TAG_E(P16) },
            { DEFIO_TAG_E(P20) },
#ifdef RP2350B
            { DEFIO_TAG_E(P32) },
            { DEFIO_TAG_E(P36) },
#endif
        },
        .mosiPins = {
            { DEFIO_TAG_E(P3) },
            { DEFIO_TAG_E(P7) },
            { DEFIO_TAG_E(P19) },
            { DEFIO_TAG_E(P23) },
#ifdef RP2350B
            { DEFIO_TAG_E(P35) },
            { DEFIO_TAG_E(P39) },
#endif
        },
    },
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(P10) },
            { DEFIO_TAG_E(P14) },
            { DEFIO_TAG_E(P26) },
#ifdef RP2350B
            { DEFIO_TAG_E(P30) },
            { DEFIO_TAG_E(P42) },
            { DEFIO_TAG_E(P46) },
#endif
        },
        .misoPins = {
            { DEFIO_TAG_E(P8) },
            { DEFIO_TAG_E(P12) },
            { DEFIO_TAG_E(P24) },
            { DEFIO_TAG_E(P28) },
#ifdef RP2350B
            { DEFIO_TAG_E(P40) },
            { DEFIO_TAG_E(P44) },
#endif
        },
        .mosiPins = {
            { DEFIO_TAG_E(P11) },
            { DEFIO_TAG_E(P15) },
            { DEFIO_TAG_E(P27) },
#ifdef RP2350B
            { DEFIO_TAG_E(P31) },
            { DEFIO_TAG_E(P43) },
            { DEFIO_TAG_E(P47) },
#endif
        },
    },
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

/*
  static spi_inst_t *getSpiInstanceByDevice(SPI_TypeDef *spi)
{
    if (spi == SPI0) {
        return spi0;
    } else if (spi == SPI1) {
        return spi1;
    }
    return NULL;
}
*/

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
  // maybe here set getSpiInstanceByDevice(spi->dev) SPI device with
  // settings like
  // STM does
  //SetRXFIFOThreshold ...QF (1/4 full presumably)
  //         Init -> full duplex, master, 8biut, baudrate, MSBfirst, no CRC,
  //                  Clock = PolarityHigh, Phase_2Edge

  
    const spiDevice_t *spi = &spiDevice[device];

    if (!spi->dev) {
        return;
    }

    spi_init(SPI_INST(spi->dev), SPI_SPEED_20MHZ);

    gpio_set_function(IO_PINBYTAG(spi->miso), GPIO_FUNC_SPI);
    gpio_set_function(IO_PINBYTAG(spi->mosi), GPIO_FUNC_SPI);
    gpio_set_function(IO_PINBYTAG(spi->sck), GPIO_FUNC_SPI);
}

void spiInitBusDMA(void)
{
  //TODO: implement
  // if required to set up mappings of peripherals to DMA instances?
  // can just start off with dma_claim_unused_channel in spiInternalInitStream?
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
    UNUSED(preInit);

    int dma_tx = dma_claim_unused_channel(true);
    int dma_rx = dma_claim_unused_channel(true);

    dev->bus->dmaTx->channel = dma_tx;
    dev->bus->dmaRx->channel = dma_rx;
    dev->bus->dmaTx->irqHandlerCallback = NULL;
    dev->bus->dmaRx->irqHandlerCallback = spiInternalResetStream; // TODO: implement - correct callback

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

// Start DMA transfer for the current segment
void spiInternalStartDMA(const extDevice_t *dev)
{
    // TODO check correct, was len + 1 now len
    dma_channel_set_trans_count(dev->bus->dmaTx->channel, dev->bus->curSegment->len, false);
    dma_channel_set_trans_count(dev->bus->dmaRx->channel, dev->bus->curSegment->len, false);

    dma_channel_start(dev->bus->dmaTx->channel);
    dma_channel_start(dev->bus->dmaRx->channel);
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
