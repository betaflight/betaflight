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
#include "drivers/rcc.h"

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

#ifdef STM32H7
#define IS_DTCM(p) (((uint32_t)p & 0xfffe0000) == 0x20000000)
#elif defined(STM32F7)
#define IS_DTCM(p) (((uint32_t)p & 0xffff0000) == 0x20000000)
#elif defined(STM32G4)
#define IS_CCM(p) ((((uint32_t)p & 0xffff8000) == 0x10000000) || (((uint32_t)p & 0xffff8000) == 0x20018000))
#endif
static LL_SPI_InitTypeDef defaultInit =
{
    .TransferDirection = LL_SPI_FULL_DUPLEX,
    .Mode = LL_SPI_MODE_MASTER,
    .DataWidth = LL_SPI_DATAWIDTH_8BIT,
    .NSS = LL_SPI_NSS_SOFT,
    .BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8,
    .BitOrder = LL_SPI_MSB_FIRST,
    .CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
    .ClockPolarity = LL_SPI_POLARITY_HIGH,
    .ClockPhase = LL_SPI_PHASE_2EDGE,
};

static uint32_t spiDivisorToBRbits(const SPI_TypeDef *instance, uint16_t divisor)
{
#if !defined(STM32H7)
    // SPI2 and SPI3 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.

    if (instance == SPI2 || instance == SPI3) {
        divisor /= 2; // Safe for divisor == 0 or 1
    }
#else
    UNUSED(instance);
#endif

    divisor = constrain(divisor, 2, 256);

#if defined(STM32H7)
    const uint32_t baudRatePrescaler[8] = {
        LL_SPI_BAUDRATEPRESCALER_DIV2,
        LL_SPI_BAUDRATEPRESCALER_DIV4,
        LL_SPI_BAUDRATEPRESCALER_DIV8,
        LL_SPI_BAUDRATEPRESCALER_DIV16,
        LL_SPI_BAUDRATEPRESCALER_DIV32,
        LL_SPI_BAUDRATEPRESCALER_DIV64,
        LL_SPI_BAUDRATEPRESCALER_DIV128,
        LL_SPI_BAUDRATEPRESCALER_DIV256,
    };
    int prescalerIndex = ffs(divisor) - 2; // prescaler begins at "/2"

    return baudRatePrescaler[prescalerIndex];
#else
    return (ffs(divisor) - 2) << SPI_CR1_BR_Pos;
#endif
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

    LL_SPI_Disable(spi->dev);
    LL_SPI_DeInit(spi->dev);

#if defined(STM32H7)
    // Prevent glitching when SPI is disabled
    LL_SPI_EnableGPIOControl(spi->dev);

    LL_SPI_SetFIFOThreshold(spi->dev, LL_SPI_FIFO_TH_01DATA);
    LL_SPI_Init(spi->dev, &defaultInit);
#else
    LL_SPI_SetRxFIFOThreshold(spi->dev, SPI_RXFIFO_THRESHOLD_QF);

    LL_SPI_Init(spi->dev, &defaultInit);
    LL_SPI_Enable(spi->dev);
#endif
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    LL_DMA_InitTypeDef *initTx = bus->initTx;

    LL_DMA_StructInit(initTx);
#if defined(STM32G4) || defined(STM32H7)
    initTx->PeriphRequest = bus->dmaTx->channel;
#else
    initTx->Channel = bus->dmaTx->channel;
#endif
    initTx->Mode = LL_DMA_MODE_NORMAL;
    initTx->Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
#if defined(STM32H7)
    initTx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->TXDR;
#else
    initTx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->DR;
#endif
    initTx->Priority = LL_DMA_PRIORITY_LOW;
    initTx->PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    initTx->PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    initTx->MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;

    if (bus->dmaRx) {
        LL_DMA_InitTypeDef *initRx = bus->initRx;

        LL_DMA_StructInit(initRx);
#if defined(STM32G4) || defined(STM32H7)
        initRx->PeriphRequest = bus->dmaRx->channel;
#else
        initRx->Channel = bus->dmaRx->channel;
#endif
        initRx->Mode = LL_DMA_MODE_NORMAL;
        initRx->Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
#if defined(STM32H7)
        initRx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->RXDR;
#else
        initRx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->DR;
#endif
        initRx->Priority = LL_DMA_PRIORITY_LOW;
        initRx->PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
        initRx->PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    // Disable the stream
#if defined(STM32G4)
    LL_DMA_DisableChannel(descriptor->dma, descriptor->stream);
    while (LL_DMA_IsEnabledChannel(descriptor->dma, descriptor->stream));
#else
    LL_DMA_DisableStream(descriptor->dma, descriptor->stream);
    while (LL_DMA_IsEnabledStream(descriptor->dma, descriptor->stream));
#endif

    // Clear any pending interrupt flags
    DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
}


FAST_CODE static bool spiInternalReadWriteBufPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
#if defined(STM32H7)
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
#else
    // set 16-bit transfer
    CLEAR_BIT(instance->CR2, SPI_RXFIFO_THRESHOLD);
    while (len > 1) {
        while (!LL_SPI_IsActiveFlag_TXE(instance));
        uint16_t w;
        if (txData) {
            w = *((uint16_t *)txData);
            txData += 2;
        } else {
            w = 0xFFFF;
        }
        LL_SPI_TransmitData16(instance, w);

        while (!LL_SPI_IsActiveFlag_RXNE(instance));
        w = LL_SPI_ReceiveData16(instance);
        if (rxData) {
            *((uint16_t *)rxData) = w;
            rxData += 2;
        }
        len -= 2;
    }
    // set 8-bit transfer
    SET_BIT(instance->CR2, SPI_RXFIFO_THRESHOLD);
    if (len) {
        while (!LL_SPI_IsActiveFlag_TXE(instance));
        uint8_t b = txData ? *(txData++) : 0xFF;
        LL_SPI_TransmitData8(instance, b);

        while (!LL_SPI_IsActiveFlag_RXNE(instance));
        b = LL_SPI_ReceiveData8(instance);
        if (rxData) {
            *(rxData++) = b;
        }
        --len;
    }
#endif

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
    LL_DMA_InitTypeDef *initTx = bus->initTx;

    if (txData) {
#ifdef __DCACHE_PRESENT
#ifdef STM32H7
        if ((txData < &_dmaram_start__) || (txData >= &_dmaram_end__)) {
#else
        // No need to flush DTCM memory
        if (!IS_DTCM(txData)) {
#endif
            // Flush the D cache to ensure the data to be written is in main memory
            SCB_CleanDCache_by_Addr(
                    (uint32_t *)((uint32_t)txData & ~CACHE_LINE_MASK),
                    (((uint32_t)txData & CACHE_LINE_MASK) + len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
        }
#endif // __DCACHE_PRESENT
        initTx->MemoryOrM2MDstAddress = (uint32_t)txData;
        initTx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    } else {
        initTx->MemoryOrM2MDstAddress = (uint32_t)&dummyTxByte;
        initTx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT;
    }
    initTx->NbData = len;

#if !defined(STM32G4) && !defined(STM32H7) 
    if (dev->bus->dmaRx) {
#endif
        uint8_t *rxData = segment->u.buffers.rxData;
        LL_DMA_InitTypeDef *initRx = bus->initRx;

        if (rxData) {
            /* Flush the D cache for the start and end of the receive buffer as
             * the cache will be invalidated after the transfer and any valid data
             * just before/after must be in memory at that point
             */
#ifdef __DCACHE_PRESENT
            // No need to flush/invalidate DTCM memory
#ifdef STM32H7
            if ((rxData < &_dmaram_start__) || (rxData >= &_dmaram_end__)) {
#else
            // No need to flush DTCM memory
            if (!IS_DTCM(rxData)) {
#endif
                SCB_CleanInvalidateDCache_by_Addr(
                        (uint32_t *)((uint32_t)rxData & ~CACHE_LINE_MASK),
                        (((uint32_t)rxData & CACHE_LINE_MASK) + len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
            }
#endif // __DCACHE_PRESENT
        initRx->MemoryOrM2MDstAddress = (uint32_t)rxData;
        initRx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    } else {
        initRx->MemoryOrM2MDstAddress = (uint32_t)&dummyRxByte;
        initRx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT;
    }
    initRx->NbData = len;
#if !defined(STM32G4) && !defined(STM32H7) 
    }
#endif
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;

    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;

#if !defined(STM32G4) && !defined(STM32H7) 
    if (dmaRx) {
#endif
        // Use the correct callback argument
        dmaRx->userParam = (uint32_t)dev;

        // Clear transfer flags
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

#ifdef STM32G4
        // Disable channels to enable update
        LL_DMA_DisableChannel(dmaTx->dma, dmaTx->stream);
        LL_DMA_DisableChannel(dmaRx->dma, dmaRx->stream);

        /* Use the Rx interrupt as this occurs once the SPI operation is complete whereas the Tx interrupt
         * occurs earlier when the Tx FIFO is empty, but the SPI operation is still in progress
         */
        LL_DMA_EnableIT_TC(dmaRx->dma, dmaRx->stream);

        // Update channels
        LL_DMA_Init(dmaTx->dma, dmaTx->stream, bus->initTx);
        LL_DMA_Init(dmaRx->dma, dmaRx->stream, bus->initRx);

        // Enable channels
        LL_DMA_EnableChannel(dmaTx->dma, dmaTx->stream);
        LL_DMA_EnableChannel(dmaRx->dma, dmaRx->stream);

        SET_BIT(dev->bus->busType_u.spi.instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
#else
        DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;
        DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;

        // Disable streams to enable update
        LL_DMA_WriteReg(streamRegsTx, CR, 0U);
        LL_DMA_WriteReg(streamRegsRx, CR, 0U);

        /* Use the Rx interrupt as this occurs once the SPI operation is complete whereas the Tx interrupt
         * occurs earlier when the Tx FIFO is empty, but the SPI operation is still in progress
         */
        LL_EX_DMA_EnableIT_TC(streamRegsRx);

        // Update streams
        LL_DMA_Init(dmaTx->dma, dmaTx->stream, bus->initTx);
        LL_DMA_Init(dmaRx->dma, dmaRx->stream, bus->initRx);

        /* Note from AN4031
         *
         * If the user enables the used peripheral before the corresponding DMA stream, a FEIF
         * (FIFO Error Interrupt Flag) may be set due to the fact the DMA is not ready to provide
         * the first required data to the peripheral (in case of memory-to-peripheral transfer).
         */

        // Enable the SPI DMA Tx & Rx requests
#if defined(STM32H7)
        LL_SPI_SetTransferSize(dev->bus->busType_u.spi.instance, dev->bus->curSegment->len);
        LL_DMA_EnableStream(dmaTx->dma, dmaTx->stream);
        LL_DMA_EnableStream(dmaRx->dma, dmaRx->stream);
        SET_BIT(dev->bus->busType_u.spi.instance->CFG1, SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);
        LL_SPI_Enable(dev->bus->busType_u.spi.instance);
        LL_SPI_StartMasterTransfer(dev->bus->busType_u.spi.instance);
#else
        // Enable streams
        LL_DMA_EnableStream(dmaTx->dma, dmaTx->stream);
        LL_DMA_EnableStream(dmaRx->dma, dmaRx->stream);

        SET_BIT(dev->bus->busType_u.spi.instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
#endif
#if !defined(STM32G4) && !defined(STM32H7) 
    } else {
        DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;

        // Use the correct callback argument
        dmaTx->userParam = (uint32_t)dev;

        // Clear transfer flags
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        // Disable streams to enable update
        LL_DMA_WriteReg(streamRegsTx, CR, 0U);

        LL_EX_DMA_EnableIT_TC(streamRegsTx);

        // Update streams
        LL_DMA_Init(dmaTx->dma, dmaTx->stream, bus->initTx);

        /* Note from AN4031
         *
         * If the user enables the used peripheral before the corresponding DMA stream, a FEIF
         * (FIFO Error Interrupt Flag) may be set due to the fact the DMA is not ready to provide
         * the first required data to the peripheral (in case of memory-to-peripheral transfer).
         */

        // Enable the SPI DMA Tx request
        // Enable streams
        LL_DMA_EnableStream(dmaTx->dma, dmaTx->stream);

        SET_BIT(dev->bus->busType_u.spi.instance->CR2, SPI_CR2_TXDMAEN);
    }
#endif
#endif
}

void spiInternalStopDMA (const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;

    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    SPI_TypeDef *instance = bus->busType_u.spi.instance;

#if !defined(STM32G4) && !defined(STM32H7)
    if (dmaRx) {
#endif
        // Disable the DMA engine and SPI interface
#ifdef STM32G4
        LL_DMA_DisableChannel(dmaTx->dma, dmaTx->stream);
        LL_DMA_DisableChannel(dmaRx->dma, dmaRx->stream);
#else
        LL_DMA_DisableStream(dmaRx->dma, dmaRx->stream);
        LL_DMA_DisableStream(dmaTx->dma, dmaTx->stream);
#endif

        // Clear transfer flags
        DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        LL_SPI_DisableDMAReq_TX(instance);
        LL_SPI_DisableDMAReq_RX(instance);
#if defined(STM32H7)
        LL_SPI_ClearFlag_TXTF(dev->bus->busType_u.spi.instance);
        LL_SPI_Disable(dev->bus->busType_u.spi.instance);
#endif
#if !defined(STM32G4) && !defined(STM32H7) 
    } else {
        SPI_TypeDef *instance = bus->busType_u.spi.instance;

        // Ensure the current transmission is complete
        while (LL_SPI_IsActiveFlag_BSY(instance));

        // Drain the RX buffer
        while (LL_SPI_IsActiveFlag_RXNE(instance)) {
            instance->DR;
        }

        // Disable the DMA engine and SPI interface
        LL_DMA_DisableStream(dmaTx->dma, dmaTx->stream);

        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        LL_SPI_DisableDMAReq_TX(instance);
#endif
#if !defined(STM32G4) && !defined(STM32H7)
    }
#endif
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

    // Switch bus speed
#if !defined(STM32H7)
    LL_SPI_Disable(instance);
#endif

    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        LL_SPI_SetBaudRatePrescaler(instance, spiDivisorToBRbits(instance, dev->busType_u.spi.speed));
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    // Switch SPI clock polarity/phase if necessary
    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        if (dev->busType_u.spi.leadingEdge) {
            IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_LOW, spi->sckAF);
            LL_SPI_SetClockPhase(instance, LL_SPI_PHASE_1EDGE);
            LL_SPI_SetClockPolarity(instance, LL_SPI_POLARITY_LOW);
        }
        else {
            IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);
            LL_SPI_SetClockPhase(instance, LL_SPI_PHASE_2EDGE);
            LL_SPI_SetClockPolarity(instance, LL_SPI_POLARITY_HIGH);
        }

        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

#if !defined(STM32H7)
    LL_SPI_Enable(instance);
#endif

    /* Where data is being read into a buffer which is cached, where the start or end of that
     * buffer is not cache aligned, there is a risk of corruption of other data in that cache line.
     * After the read is complete, the cache lines covering the structure will be invalidated to ensure
     * that the processor sees the read data, not what was in cache previously. Unfortunately if
     * there is any other data in the area covered by those cache lines, at the start or end of the
     * buffer, it too will be invalidated, so had the processor written to those locations during the DMA
     * operation those written values will be lost.
     */

    // Check that any reads are cache aligned and of multiple cache lines in length
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        // Check there is no receive data as only transmit DMA is available
        if ((checkSegment->u.buffers.rxData) && (bus->dmaRx == (dmaChannelDescriptor_t *)NULL)) {
            dmaSafe = false;
            break;
        }
#ifdef STM32H7
        // Check if RX data can be DMAed
        if ((checkSegment->u.buffers.rxData) &&
            // DTCM can't be accessed by DMA1/2 on the H7
            (IS_DTCM(checkSegment->u.buffers.rxData) ||
             // Memory declared as DMA_RAM will have an address between &_dmaram_start__ and &_dmaram_end__
             (((checkSegment->u.buffers.rxData < &_dmaram_start__) || (checkSegment->u.buffers.rxData >= &_dmaram_end__)) &&
             (((uint32_t)checkSegment->u.buffers.rxData & (CACHE_LINE_SIZE - 1)) || (checkSegment->len & (CACHE_LINE_SIZE - 1)))))) {
            dmaSafe = false;
            break;
        }
        // Check if TX data can be DMAed
        else if ((checkSegment->u.buffers.txData) && IS_DTCM(checkSegment->u.buffers.txData)) {
            dmaSafe = false;
            break;
        }
#elif defined(STM32F7)
        if ((checkSegment->u.buffers.rxData) &&
            // DTCM is accessible and uncached on the F7
            (!IS_DTCM(checkSegment->u.buffers.rxData) &&
            (((uint32_t)checkSegment->u.buffers.rxData & (CACHE_LINE_SIZE - 1)) || (checkSegment->len & (CACHE_LINE_SIZE - 1))))) {
            dmaSafe = false;
            break;
        }
#elif defined(STM32G4)
        // Check if RX data can be DMAed
        if ((checkSegment->u.buffers.rxData) &&
            // CCM can't be accessed by DMA1/2 on the G4
            IS_CCM(checkSegment->u.buffers.rxData)) {
            dmaSafe = false;
            break;
        }
        if ((checkSegment->u.buffers.txData) &&
            // CCM can't be accessed by DMA1/2 on the G4
            IS_CCM(checkSegment->u.buffers.txData)) {
            dmaSafe = false;
            break;
        }
#endif
        // Note that these counts are only valid if dmaSafe is true
        segmentCount++;
        xferLen += checkSegment->len;
    }

    // Use DMA if possible
    // If there are more than one segments, or a single segment with negateCS negated in the list terminator then force DMA irrespective of length
    if (bus->useDMA && dmaSafe && ((segmentCount > 1) ||
                                   (xferLen >= SPI_DMA_THRESHOLD) ||
                                   !bus->curSegment[segmentCount].negateCS)) {
        // Intialise the init structures for the first transfer
        spiInternalInitStream(dev, false);

        // Assert Chip Select
        IOLo(dev->busType_u.spi.csnPin);

        // Start the transfers
        spiInternalStartDMA(dev);
    } else {
        busSegment_t *lastSegment = NULL;
        bool segmentComplete;

        // Manually work through the segment list performing a transfer for each
        while (bus->curSegment->len) {
            if (!lastSegment || lastSegment->negateCS) {
                // Assert Chip Select if necessary - it's costly so only do so if necessary
                IOLo(dev->busType_u.spi.csnPin);
            }

            spiInternalReadWriteBufPolled(
                    bus->busType_u.spi.instance,
                    bus->curSegment->u.buffers.txData,
                    bus->curSegment->u.buffers.rxData,
                    bus->curSegment->len);

            if (bus->curSegment->negateCS) {
                // Negate Chip Select
                IOHi(dev->busType_u.spi.csnPin);
            }

            segmentComplete = true;
            if (bus->curSegment->callback) {
                switch(bus->curSegment->callback(dev->callbackArg)) {
                case BUS_BUSY:
                    // Repeat the last DMA segment
                    segmentComplete = false;
                    break;

                case BUS_ABORT:
                    bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
                    segmentComplete = false;
                    return;

                case BUS_READY:
                default:
                    // Advance to the next DMA segment
                    break;
                }
            }
            if (segmentComplete) {
                lastSegment = (busSegment_t *)bus->curSegment;
                bus->curSegment++;
            }
        }

        // If a following transaction has been linked, start it
        if (bus->curSegment->u.link.dev) {
            busSegment_t *endSegment = (busSegment_t *)bus->curSegment;
            const extDevice_t *nextDev = endSegment->u.link.dev;
            busSegment_t *nextSegments = (busSegment_t *)endSegment->u.link.segments;
            bus->curSegment = nextSegments;
            endSegment->u.link.dev = NULL;
            endSegment->u.link.segments = NULL;
            spiSequenceStart(nextDev);
        } else {
            // The end of the segment list has been reached, so mark transactions as complete
            bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
        }
    }
}
#endif
