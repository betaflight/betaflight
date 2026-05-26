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
 * SPI low-level driver for HAL2 (Cube 2.0) families.
 *
 * Fork of bus_spi_ll.c. The SPI peripheral is register-compatible with H5
 * (same CR1/CFG1/CFG2/SR/TXDR/RXDR), but the DMA API is fundamentally
 * different: HAL2 LL DMA functions take DMA_Channel_TypeDef* directly
 * instead of the (DMA_TypeDef*, stream_index) pair used by GPDMA on H5.
 *
 * Named generically so future HAL2 families can reuse this file.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_SPI)

#include "common/utils.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_spi_types.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "platform/rcc.h"

// Use DMA if possible if this many bytes are to be transferred
#define SPI_DMA_THRESHOLD 8

#ifndef SPI1_NSS_PIN
#define SPI1_NSS_PIN NONE
#endif
#ifndef SPI2_NSS_PIN
#define SPI2_NSS_PIN NONE
#endif
#ifndef SPI3_NSS_PIN
#define SPI3_NSS_PIN NONE
#endif
#ifndef SPI4_NSS_PIN
#define SPI4_NSS_PIN NONE
#endif

#define SPI_DEFAULT_TIMEOUT 10

// No DTCM/CCM restrictions for DMA on HAL2 families
#define IS_DTCM(p) (0)

static LL_SPI_InitTypeDef defaultInit =
{
    .TransferDirection = LL_SPI_FULL_DUPLEX,
    .Mode = LL_SPI_MODE_MASTER,
    .DataWidth = LL_SPI_DATA_WIDTH_8_BIT,
    .NSS = LL_SPI_NSS_SOFT,
    .BaudRate = LL_SPI_BAUD_RATE_PRESCALER_8,
    .BitOrder = LL_SPI_MSB_FIRST,
    .CRCCalculation = 0,
    .ClockPolarity = LL_SPI_CLOCK_POLARITY_HIGH,
    .ClockPhase = LL_SPI_CLOCK_PHASE_2_EDGE,
};

static uint32_t spiDivisorToBRbits(const SPI_TypeDef *instance, uint16_t divisor)
{
    UNUSED(instance);

    divisor = constrain(divisor, 2, 256);

    const uint32_t baudRatePrescaler[8] = {
        LL_SPI_BAUD_RATE_PRESCALER_2,
        LL_SPI_BAUD_RATE_PRESCALER_4,
        LL_SPI_BAUD_RATE_PRESCALER_8,
        LL_SPI_BAUD_RATE_PRESCALER_16,
        LL_SPI_BAUD_RATE_PRESCALER_32,
        LL_SPI_BAUD_RATE_PRESCALER_64,
        LL_SPI_BAUD_RATE_PRESCALER_128,
        LL_SPI_BAUD_RATE_PRESCALER_256,
    };
    int prescalerIndex = ffs(divisor) - 2;

    return baudRatePrescaler[prescalerIndex];
}

void spiInitDevice(spiDevice_e device)
{
    spiDevice_t *spi = &spiDevice[device];

    if (!spi->dev) {
        return;
    }

    SPI_TypeDef *dev = (SPI_TypeDef *)spi->dev;

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

#ifdef STM32C5
    if (dev == SPI1) {
        LL_RCC_SetSPIClockSource(LL_RCC_SPI1_CLKSOURCE_HSIK);
    } else if (dev == SPI2) {
        LL_RCC_SetSPIClockSource(LL_RCC_SPI2_CLKSOURCE_HSIK);
    } else if (dev == SPI3) {
        LL_RCC_SetSPIClockSource(LL_RCC_SPI3_CLKSOURCE_HSIK);
    }
#endif

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_SDI_CFG, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->mosiAF);
    IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);

    LL_SPI_Disable(dev);
    LL_SPI_DeInit(dev);

    // Prevent glitching when SPI is disabled
    LL_SPI_EnableGPIOControl(dev);

    LL_SPI_SetFIFOThreshold(dev, LL_SPI_FIFO_THRESHOLD_1_DATA);
    LL_SPI_Init(dev, &defaultInit);

#ifdef STM32C5
    // Master + NSS_SOFT requires CR1.SSI=1; otherwise the peripheral reads
    // its internal NSS as low, raises a mode fault, and atomically clears
    // CFG2.MASTER (silently dropping to slave mode -- driver then hangs
    // waiting for RXP). The compat-shim LL_SPI_Init in stm32c5xx_hal2_compat.h
    // skips the SSI write that LL_SPI_SetConfig would do, so set it here
    // and re-arm MASTER in case a prior MODF cleared it.
    dev->IFCR = SPI_IFCR_MODFC;
    dev->CR1 |= SPI_CR1_SSI;
    dev->CFG2 |= SPI_CFG2_MASTER;
#endif
}

// Helper: get the DMA_Channel_TypeDef* from a dmaChannelDescriptor
static inline DMA_Channel_TypeDef *dmaChPtr(dmaChannelDescriptor_t *d)
{
    return (DMA_Channel_TypeDef *)d->ref;
}

// Configure an LPDMA channel for SPI transfer
static void dmaConfigChannel(DMA_Channel_TypeDef *ch, uint32_t request,
                              uint32_t srcAddr, uint32_t destAddr,
                              uint32_t srcInc, uint32_t destInc,
                              uint32_t length)
{
    // CTR1: byte data widths, increment modes
    ch->CTR1 = srcInc | destInc;

    // CTR2: DMA request selection
    ch->CTR2 = (request & DMA_CTR2_REQSEL);

    // CBR1: block data length
    ch->CBR1 = length & DMA_CBR1_BNDT;

    // Addresses
    ch->CSAR = srcAddr;
    ch->CDAR = destAddr;

    // No linked-list (single shot)
    ch->CLLR = 0;
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;
    LL_DMA_InitTypeDef *dmaInitTx = bus->dmaInitTx;

    memset(dmaInitTx, 0, sizeof(*dmaInitTx));
    dmaInitTx->Request = bus->dmaTx->channel;
    dmaInitTx->Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dmaInitTx->SrcIncMode = DMA_CTR1_SINC;
    dmaInitTx->DestIncMode = 0;
    dmaInitTx->DestAddress = (uint32_t)&instance->TXDR;

    if (bus->dmaRx) {
        LL_DMA_InitTypeDef *dmaInitRx = bus->dmaInitRx;

        memset(dmaInitRx, 0, sizeof(*dmaInitRx));
        dmaInitRx->Request = bus->dmaRx->channel;
        dmaInitRx->Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
        dmaInitRx->SrcIncMode = 0;
        dmaInitRx->DestIncMode = DMA_CTR1_DINC;
        dmaInitRx->SrcAddress = (uint32_t)&instance->RXDR;
    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    DMA_Channel_TypeDef *ch = dmaChPtr(descriptor);

    // Disable the channel
    LL_DMA_DisableChannel(ch);
    while (LL_DMA_IsEnabledChannel(ch));

    // Clear any pending interrupt flags
    DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
}

FAST_CODE bool spiInternalReadWriteBufPolled(spiResource_t *spiInstance, const uint8_t *txData, uint8_t *rxData, int len)
{
    SPI_TypeDef *instance = (SPI_TypeDef *)spiInstance;

    LL_SPI_SetTransferSize(instance, len);
    LL_SPI_Enable(instance);
    LL_SPI_StartMasterTransfer(instance);
    while (len) {
        while (!LL_SPI_IsActiveFlag_TXP(instance));
        uint8_t b = txData ? *(txData++) : 0xFF;
        LL_SPI_TransmitData8(instance, b);

        while (!LL_SPI_IsActiveFlag_RXP(instance));
        b = LL_SPI_ReceiveData8(instance);
        if (rxData) {
            *(rxData++) = b;
        }
        --len;
    }
    while (!LL_SPI_IsActiveFlag_EOT(instance));
    LL_SPI_ClearFlag_TXTF(instance);
    LL_SPI_Disable(instance);

    return true;
}

void spiInternalInitStream(const extDevice_t *dev, volatile busSegment_t *segment)
{
    STATIC_DMA_DATA_AUTO uint8_t dummyTxByte = 0xff;
    STATIC_DMA_DATA_AUTO uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;
    int len = segment->len;

    uint8_t *txData = segment->u.buffers.txData;
    LL_DMA_InitTypeDef *dmaInitTx = bus->dmaInitTx;

    if (txData) {
        dmaInitTx->SrcAddress = (uint32_t)txData;
        dmaInitTx->SrcIncMode = DMA_CTR1_SINC;
    } else {
        dmaInitTx->SrcAddress = (uint32_t)&dummyTxByte;
        dmaInitTx->SrcIncMode = 0;
    }
    dmaInitTx->BlkDataLength = len;

    uint8_t *rxData = segment->u.buffers.rxData;
    LL_DMA_InitTypeDef *dmaInitRx = bus->dmaInitRx;

    if (rxData) {
        dmaInitRx->DestAddress = (uint32_t)rxData;
        dmaInitRx->DestIncMode = DMA_CTR1_DINC;
    } else {
        dmaInitRx->DestAddress = (uint32_t)&dummyRxByte;
        dmaInitRx->DestIncMode = 0;
    }
    dmaInitRx->BlkDataLength = len;
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;

    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    LL_DMA_InitTypeDef *initTx = bus->dmaInitTx;
    LL_DMA_InitTypeDef *initRx = bus->dmaInitRx;

    DMA_Channel_TypeDef *chTx = dmaChPtr(dmaTx);
    DMA_Channel_TypeDef *chRx = dmaChPtr(dmaRx);

    // Use the correct callback argument
    dmaRx->userParam = (uint32_t)dev;

    // Clear transfer flags
    DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
    DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

    // Disable channels to enable reconfiguration
    LL_DMA_DisableChannel(chTx);
    LL_DMA_DisableChannel(chRx);

    // Configure TX DMA channel
    dmaConfigChannel(chTx, initTx->Request,
                     initTx->SrcAddress, initTx->DestAddress,
                     initTx->SrcIncMode, initTx->DestIncMode,
                     initTx->BlkDataLength);

    // Configure RX DMA channel
    dmaConfigChannel(chRx, initRx->Request,
                     initRx->SrcAddress, initRx->DestAddress,
                     initRx->SrcIncMode, initRx->DestIncMode,
                     initRx->BlkDataLength);

    // Enable RX TC interrupt (fires when SPI operation is fully complete)
    LL_DMA_EnableIT_TC(chRx);

    // Setup SPI transfer size, enable DMA channels, then SPI
    LL_SPI_SetTransferSize(instance, dev->bus->curSegment->len);
    LL_DMA_EnableChannel(chTx);
    LL_DMA_EnableChannel(chRx);
    SET_BIT(instance->CFG1, SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);
    LL_SPI_Enable(instance);
    LL_SPI_StartMasterTransfer(instance);
}

void spiInternalStopDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;

    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;

    // Disable the DMA channels
    LL_DMA_DisableChannel(dmaChPtr(dmaTx));
    LL_DMA_DisableChannel(dmaChPtr(dmaRx));

    // Clear transfer flags
    DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

    LL_SPI_DisableDMAReq_TX(instance);
    LL_SPI_DisableDMAReq_RX(instance);
    LL_SPI_ClearFlag_TXTF(instance);
    LL_SPI_Disable(instance);
}

// DMA transfer setup and start
FAST_CODE void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;
    spiDevice_t *spi = &spiDevice[spiDeviceByInstance(bus->busType_u.spi.instance)];
    bool dmaSafe = dev->useDMA;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    bus->initSegment = true;

    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        LL_SPI_SetBaudRatePrescaler(instance, spiDivisorToBRbits(instance, dev->busType_u.spi.speed));
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    // Switch SPI clock polarity/phase if necessary
    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        if (dev->busType_u.spi.leadingEdge) {
            IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_LOW, spi->sckAF);
            LL_SPI_SetClockPhase(instance, LL_SPI_CLOCK_PHASE_1_EDGE);
            LL_SPI_SetClockPolarity(instance, LL_SPI_CLOCK_POLARITY_LOW);
        }
        else {
            IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);
            LL_SPI_SetClockPhase(instance, LL_SPI_CLOCK_PHASE_2_EDGE);
            LL_SPI_SetClockPolarity(instance, LL_SPI_CLOCK_POLARITY_HIGH);
        }

        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    // Check that any reads are cache aligned and of multiple cache lines in length
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        // Check there is no receive data as only transmit DMA is available
        if ((checkSegment->u.buffers.rxData) && (bus->dmaRx == (dmaChannelDescriptor_t *)NULL)) {
            dmaSafe = false;
            break;
        }
        // Note that these counts are only valid if dmaSafe is true
        segmentCount++;
        xferLen += checkSegment->len;
    }

    // Use DMA if possible
    if (bus->useDMA && dmaSafe && ((segmentCount > 1) ||
                                   (xferLen >= SPI_DMA_THRESHOLD) ||
                                   !bus->curSegment[segmentCount].negateCS)) {
        spiProcessSegmentsDMA(dev);
    } else {
        spiProcessSegmentsPolled(dev);
    }
}
#endif
