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

#if defined(USE_SPI)

#include "common/utils.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "platform/rcc.h"

// Use DMA if possible if this many bytes are to be transferred
#define SPI_DMA_THRESHOLD 8

// APM32F405 can't DMA to/from FASTRAM (CCM SRAM)
#define IS_CCM(p) (((uint32_t)p & 0xffff0000) == 0x10000000)

static DDL_SPI_InitTypeDef defaultInit =
{
    .TransferDirection = DDL_SPI_FULL_DUPLEX,
    .Mode = DDL_SPI_MODE_MASTER,
    .DataWidth = DDL_SPI_DATAWIDTH_8BIT,
    .NSS = DDL_SPI_NSS_SOFT,
    .BaudRate = DDL_SPI_BAUDRATEPRESCALER_DIV8,
    .BitOrder = DDL_SPI_MSB_FIRST,
    .CRCCalculation = DDL_SPI_CRCCALCULATION_DISABLE,
    .ClockPolarity = DDL_SPI_POLARITY_HIGH,
    .ClockPhase = DDL_SPI_PHASE_2EDGE,
};

static uint32_t spiDivisorToBRbits(const SPI_TypeDef *instance, uint16_t divisor)
{
    // SPI2 and SPI3 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.
    if (instance == SPI2 || instance == SPI3) {
        divisor /= 2; // Safe for divisor == 0 or 1
    }

    divisor = constrain(divisor, 2, 256);

    return (ffs(divisor) - 2) << 3;// SPI_CR1_BR_Pos
}

void spiInitDevice(SPIDevice device)
{
    spiDevice_t *spi = &spiDevice[device];

    if (!spi->dev) {
        return;
    }

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_SDI_CFG, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->mosiAF);
    IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);

    DDL_SPI_Disable(spi->dev);
    DDL_SPI_DeInit(spi->dev);

    DDL_SPI_DisableDMAReq_RX(spi->dev);
    DDL_SPI_DisableDMAReq_TX(spi->dev);

    DDL_SPI_Init(spi->dev, &defaultInit);
    DDL_SPI_Enable(spi->dev);
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    DDL_DMA_InitTypeDef *dmaInitTx = bus->dmaInitTx;

    DDL_DMA_StructInit(dmaInitTx);

    dmaInitTx->Channel = bus->dmaTx->channel;
    dmaInitTx->Mode = DDL_DMA_MODE_NORMAL;
    dmaInitTx->Direction = DDL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dmaInitTx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->DATA;
    dmaInitTx->Priority = DDL_DMA_PRIORITY_LOW;
    dmaInitTx->PeriphOrM2MSrcIncMode  = DDL_DMA_PERIPH_NOINCREMENT;
    dmaInitTx->PeriphOrM2MSrcDataSize = DDL_DMA_PDATAALIGN_BYTE;
    dmaInitTx->MemoryOrM2MDstDataSize = DDL_DMA_MDATAALIGN_BYTE;

    if (bus->dmaRx) {
        DDL_DMA_InitTypeDef *dmaInitRx = bus->dmaInitRx;

        DDL_DMA_StructInit(dmaInitRx);

        dmaInitRx->Channel = bus->dmaRx->channel;
        dmaInitRx->Mode = DDL_DMA_MODE_NORMAL;
        dmaInitRx->Direction = DDL_DMA_DIRECTION_PERIPH_TO_MEMORY;
        dmaInitRx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->DATA;
        dmaInitRx->Priority = DDL_DMA_PRIORITY_LOW;
        dmaInitRx->PeriphOrM2MSrcIncMode  = DDL_DMA_PERIPH_NOINCREMENT;
        dmaInitRx->PeriphOrM2MSrcDataSize = DDL_DMA_PDATAALIGN_BYTE;
    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    // Disable the stream
    DDL_DMA_DisableStream(descriptor->dma, descriptor->stream);
    while (DDL_DMA_IsEnabledStream(descriptor->dma, descriptor->stream));

    // Clear any pending interrupt flags
    DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
}

FAST_CODE bool spiInternalReadWriteBufPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    while (len) {
        while (!DDL_SPI_IsActiveFlag_TXE(instance));
        uint8_t b = txData ? *(txData++) : 0xFF;
        DDL_SPI_TransmitData8(instance, b);

        while (!DDL_SPI_IsActiveFlag_RXNE(instance));
        b = DDL_SPI_ReceiveData8(instance);
        if (rxData) {
            *(rxData++) = b;
        }
        --len;
    }

    return true;
}

void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
    STATIC_DMA_DATA_AUTO uint8_t dummyTxByte = 0xff;
    STATIC_DMA_DATA_AUTO uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;

    busSegment_t *segment = (busSegment_t *)bus->curSegment;

    if (preInit) {
        // Prepare the init structure for the next segment to reduce inter-segment interval
        segment++;
        if(segment->len == 0) {
            // There's no following segment
            return;
        }
    }

    int len = segment->len;

    uint8_t *txData = segment->u.buffers.txData;
    DDL_DMA_InitTypeDef *dmaInitTx = bus->dmaInitTx;

    if (txData) {
        dmaInitTx->MemoryOrM2MDstAddress = (uint32_t)txData;
        dmaInitTx->MemoryOrM2MDstIncMode = DDL_DMA_MEMORY_INCREMENT;
    } else {
        dummyTxByte = 0xff;
        dmaInitTx->MemoryOrM2MDstAddress = (uint32_t)&dummyTxByte;
        dmaInitTx->MemoryOrM2MDstIncMode = DDL_DMA_MEMORY_NOINCREMENT;
    }
    dmaInitTx->NbData = len;

    if (dev->bus->dmaRx) {
        uint8_t *rxData = segment->u.buffers.rxData;
        DDL_DMA_InitTypeDef *dmaInitRx = bus->dmaInitRx;

        if (rxData) {
            /* Flush the D cache for the start and end of the receive buffer as
             * the cache will be invalidated after the transfer and any valid data
             * just before/after must be in memory at that point
             */
            dmaInitRx->MemoryOrM2MDstAddress = (uint32_t)rxData;
            dmaInitRx->MemoryOrM2MDstIncMode = DDL_DMA_MEMORY_INCREMENT;
        } else {
            dmaInitRx->MemoryOrM2MDstAddress = (uint32_t)&dummyRxByte;
            dmaInitRx->MemoryOrM2MDstIncMode = DDL_DMA_MEMORY_NOINCREMENT;
        }
        dmaInitRx->NbData = len;
    }
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;

    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;

    DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;

    if (dmaRx) {
        DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;

        // Use the correct callback argument
        dmaRx->userParam = (uint32_t)dev;

        // Clear transfer flags
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        // Disable streams to enable update
        DDL_DMA_WriteReg(streamRegsTx, SCFG, 0U);
        DDL_DMA_WriteReg(streamRegsRx, SCFG, 0U);

        /* Use the Rx interrupt as this occurs once the SPI operation is complete whereas the Tx interrupt
         * occurs earlier when the Tx FIFO is empty, but the SPI operation is still in progress
         */
        DDL_EX_DMA_EnableIT_TC(streamRegsRx);

        // Update streams
        DDL_DMA_Init(dmaTx->dma, dmaTx->stream, bus->dmaInitTx);
        DDL_DMA_Init(dmaRx->dma, dmaRx->stream, bus->dmaInitRx);

        /* Note from AN4031
         *
         * If the user enables the used peripheral before the corresponding DMA stream, a FEIF
         * (FIFO Error Interrupt Flag) may be set due to the fact the DMA is not ready to provide
         * the first required data to the peripheral (in case of memory-to-peripheral transfer).
         */

        // Enable the SPI DMA Tx & Rx requests

        // Enable streams
        DDL_DMA_EnableStream(dmaTx->dma, dmaTx->stream);
        DDL_DMA_EnableStream(dmaRx->dma, dmaRx->stream);

        SET_BIT(dev->bus->busType_u.spi.instance->CTRL2, SPI_CTRL2_TXDEN | SPI_CTRL2_RXDEN);
    } else {
        // Use the correct callback argument
        dmaTx->userParam = (uint32_t)dev;

        // Clear transfer flags
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        // Disable streams to enable update
        DDL_DMA_WriteReg(streamRegsTx, SCFG, 0U);

        DDL_EX_DMA_EnableIT_TC(streamRegsTx);

        // Update streams
        DDL_DMA_Init(dmaTx->dma, dmaTx->stream, bus->dmaInitTx);

        /* Note from AN4031
         *
         * If the user enables the used peripheral before the corresponding DMA stream, a FEIF
         * (FIFO Error Interrupt Flag) may be set due to the fact the DMA is not ready to provide
         * the first required data to the peripheral (in case of memory-to-peripheral transfer).
         */

        // Enable the SPI DMA Tx request
        // Enable streams
        DDL_DMA_EnableStream(dmaTx->dma, dmaTx->stream);

        SET_BIT(dev->bus->busType_u.spi.instance->CTRL2, SPI_CTRL2_TXDEN);
    }
}

void spiInternalStopDMA (const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;

    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    SPI_TypeDef *instance = bus->busType_u.spi.instance;

    if (dmaRx) {
        // Disable the DMA engine and SPI interface
        DDL_DMA_DisableStream(dmaRx->dma, dmaRx->stream);
        DDL_DMA_DisableStream(dmaTx->dma, dmaTx->stream);

        // Clear transfer flags
        DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        DDL_SPI_DisableDMAReq_TX(instance);
        DDL_SPI_DisableDMAReq_RX(instance);
    } else {
        SPI_TypeDef *instance = bus->busType_u.spi.instance;

        // Ensure the current transmission is complete
        while (DDL_SPI_IsActiveFlag_BSY(instance));

        // Drain the RX buffer
        while (DDL_SPI_IsActiveFlag_RXNE(instance)) {
            instance->DATA;
        }

        // Disable the DMA engine and SPI interface
        DDL_DMA_DisableStream(dmaTx->dma, dmaTx->stream);

        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        DDL_SPI_DisableDMAReq_TX(instance);
    }
}

// DMA transfer setup and start
FAST_CODE void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = bus->busType_u.spi.instance;
    spiDevice_t *spi = &spiDevice[spiDeviceByInstance(instance)];
    bool dmaSafe = dev->useDMA;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    bus->initSegment = true;

    DDL_SPI_Disable(instance);

    // Switch bus speed
    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        DDL_SPI_SetBaudRatePrescaler(instance, spiDivisorToBRbits(instance, dev->busType_u.spi.speed));
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    // Switch SPI clock polarity/phase if necessary
    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        if (dev->busType_u.spi.leadingEdge) {
            IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_LOW, spi->sckAF);
            DDL_SPI_SetClockPhase(instance, DDL_SPI_PHASE_1EDGE);
            DDL_SPI_SetClockPolarity(instance, DDL_SPI_POLARITY_LOW);
        }
        else {
            IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);
            DDL_SPI_SetClockPhase(instance, DDL_SPI_PHASE_2EDGE);
            DDL_SPI_SetClockPolarity(instance, DDL_SPI_POLARITY_HIGH);
        }

        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    DDL_SPI_Enable(instance);

    // Check that any reads are cache aligned and of multiple cache lines in length
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        // Check there is no receive data as only transmit DMA is available
        if (((checkSegment->u.buffers.rxData) && (IS_CCM(checkSegment->u.buffers.rxData) || (bus->dmaRx == (dmaChannelDescriptor_t *)NULL))) ||
            ((checkSegment->u.buffers.txData) && IS_CCM(checkSegment->u.buffers.txData))) {
            dmaSafe = false;
            break;
        }
        // Note that these counts are only valid if dmaSafe is true
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
#endif
