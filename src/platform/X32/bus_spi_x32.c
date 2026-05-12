/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_SPI

#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_spi_types.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "pg/bus_spi.h"

#define SPI_DMA_THRESHOLD 8

extern spiDevice_t spiDevice[SPIDEV_COUNT];
extern busDevice_t spiBusDevice[SPIDEV_COUNT];

static const SPI_InitType defaultInit = {
    .DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX,
    .SpiMode = SPI_MODE_MASTER,
    .DataLen = SPI_DATA_SIZE_8BITS,
    .CLKPOL = SPI_CLKPOL_HIGH,
    .CLKPHA = SPI_CLKPHA_SECOND_EDGE,
    .NSS = SPI_NSS_SOFT,
    .BaudRatePres = SPI_BR_PRESCALER_8,
    .FirstBit = SPI_FB_MSB,
    .CRCPoly = 7,
};

FAST_IRQ_HANDLER static void spiRxIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    const extDevice_t *dev = (const extDevice_t *)descriptor->userParam;

    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;

    if (bus->curSegment->negateCS) {
        IOHi(dev->busType_u.spi.csnPin);
    }

    spiInternalStopDMA(dev);

#ifdef __DCACHE_PRESENT
    if (bus->curSegment->u.buffers.rxData) {
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)((uint32_t)bus->curSegment->u.buffers.rxData & ~CACHE_LINE_MASK),
            (((uint32_t)bus->curSegment->u.buffers.rxData & CACHE_LINE_MASK) +
             bus->curSegment->len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
    }
#endif

    spiIrqHandler(dev);
}

static uint32_t spiMaxClockSource(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);

    uint32_t maxClock = clocks.APB1ClkFreq;
    if (clocks.APB2ClkFreq > maxClock) {
        maxClock = clocks.APB2ClkFreq;
    }
    if (clocks.APB5ClkFreq > maxClock) {
        maxClock = clocks.APB5ClkFreq;
    }

    return maxClock;
}

static uint16_t spiDivisorToPrescaler(uint16_t divisor)
{
    divisor = constrain(divisor, 2, 256);

    switch (divisor) {
    case 2:
        return SPI_BR_PRESCALER_2;
    case 4:
        return SPI_BR_PRESCALER_4;
    case 8:
        return SPI_BR_PRESCALER_8;
    case 16:
        return SPI_BR_PRESCALER_16;
    case 32:
        return SPI_BR_PRESCALER_32;
    case 64:
        return SPI_BR_PRESCALER_64;
    case 128:
        return SPI_BR_PRESCALER_128;
    default:
        return SPI_BR_PRESCALER_256;
    }
}

static void spiApplyConfig(SPI_TypeDef *instance, uint16_t divisor, bool leadingEdge)
{
    SPI_InitType init = defaultInit;
    init.BaudRatePres = spiDivisorToPrescaler(divisor);
    init.CLKPOL = leadingEdge ? SPI_CLKPOL_LOW : SPI_CLKPOL_HIGH;
    init.CLKPHA = leadingEdge ? SPI_CLKPHA_FIRST_EDGE : SPI_CLKPHA_SECOND_EDGE;

    SPI_Enable(instance, DISABLE);
    SPI_Init(instance, &init);
    SPI_I2S_EnableDma(instance, SPI_I2S_DMA_TX | SPI_I2S_DMA_RX, DISABLE);
    SPI_SSOutputEnable(instance, ENABLE);
    SPI_SetNssLevel(instance, SPI_NSS_HIGH);
    SPI_Enable(instance, ENABLE);
}

void spiInitDevice(spiDevice_e device)
{
    spiDevice_t *spi = &spiDevice[device];

    if (!spi->dev) {
        return;
    }

    SPI_TypeDef *dev = (SPI_TypeDef *)spi->dev;

    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck), OWNER_SPI_SCK, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG, spi->sckAF);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_SDI_CFG, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->mosiAF);

    SPI_I2S_DeInit(dev);
    spiApplyConfig(dev, 8, false);
}

uint16_t spiCalculateDivider(uint32_t freq)
{
    uint32_t spiClk = spiMaxClockSource();
    uint16_t divisor = 2;

    while ((spiClk / divisor) > freq && divisor < 256) {
        divisor <<= 1;
    }

    return divisor;
}

uint32_t spiCalculateClock(uint16_t spiClkDivisor)
{
    if (spiClkDivisor < 2) {
        spiClkDivisor = 2;
    }

    return spiMaxClockSource() / spiClkDivisor;
}

void spiInitBusDMA(void)
{
    for (uint32_t device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = &spiBusDevice[device];

        if (bus->busType != BUS_TYPE_SPI) {
            continue;
        }

        dmaIdentifier_e dmaTxIdentifier = DMA_NONE;
        dmaIdentifier_e dmaRxIdentifier = DMA_NONE;

        int8_t txDmaopt = spiPinConfig(device)->txDmaopt;
        uint8_t txDmaoptMin = 0;
        uint8_t txDmaoptMax = MAX_PERIPHERAL_DMA_OPTIONS - 1;

        if (txDmaopt != DMA_OPT_UNUSED) {
            txDmaoptMin = txDmaopt;
            txDmaoptMax = txDmaopt;
        }

        for (uint8_t opt = txDmaoptMin; opt <= txDmaoptMax; opt++) {
            const dmaChannelSpec_t *dmaTxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDO, device, opt);

            if (!dmaTxChannelSpec) {
                continue;
            }

            dmaTxIdentifier = dmaGetIdentifier(dmaTxChannelSpec->ref);
            if (!dmaAllocate(dmaTxIdentifier, OWNER_SPI_SDO, device + 1)) {
                dmaTxIdentifier = DMA_NONE;
                continue;
            }

            bus->dmaTx = dmaGetDescriptorByIdentifier(dmaTxIdentifier);
            dmaEnable(dmaTxIdentifier);
            dmaMuxEnable(dmaTxIdentifier, dmaTxChannelSpec->dmaMuxId);
            break;
        }

        int8_t rxDmaopt = spiPinConfig(device)->rxDmaopt;
        uint8_t rxDmaoptMin = 0;
        uint8_t rxDmaoptMax = MAX_PERIPHERAL_DMA_OPTIONS - 1;

        if (rxDmaopt != DMA_OPT_UNUSED) {
            rxDmaoptMin = rxDmaopt;
            rxDmaoptMax = rxDmaopt;
        }

        for (uint8_t opt = rxDmaoptMin; opt <= rxDmaoptMax; opt++) {
            const dmaChannelSpec_t *dmaRxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDI, device, opt);

            if (!dmaRxChannelSpec) {
                continue;
            }

            dmaRxIdentifier = dmaGetIdentifier(dmaRxChannelSpec->ref);
            if (!dmaAllocate(dmaRxIdentifier, OWNER_SPI_SDI, device + 1)) {
                dmaRxIdentifier = DMA_NONE;
                continue;
            }

            bus->dmaRx = dmaGetDescriptorByIdentifier(dmaRxIdentifier);
            dmaEnable(dmaRxIdentifier);
            dmaMuxEnable(dmaRxIdentifier, dmaRxChannelSpec->dmaMuxId);
            break;
        }

        if (dmaTxIdentifier && dmaRxIdentifier) {
            spiInternalResetStream(bus->dmaTx);
            spiInternalResetStream(bus->dmaRx);
            spiInternalResetDescriptors(bus);
            dmaSetHandler(dmaRxIdentifier, spiRxIrqHandler, NVIC_PRIO_SPI_DMA, 0);
            bus->useDMA = true;
        } else {
            bus->dmaTx = NULL;
            bus->dmaRx = NULL;
            bus->useDMA = false;
        }
    }
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;
    DMA_InitTypeDef *dmaInitTx = bus->dmaInitTx;

    DMA_ChannelStructInit(dmaInitTx);
    dmaInitTx->TfrType = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
    dmaInitTx->TfrTypeFlowCtrl = DMA_CH_TRANSFER_FLOW_M2P_DMA;
    dmaInitTx->ChannelPriority = DMA_CH_PRIORITY_2;
    dmaInitTx->SrcAddr = 0;
    dmaInitTx->DstAddr = (uint32_t)&instance->DAT;
    dmaInitTx->SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
    dmaInitTx->DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
    dmaInitTx->DstBurstLen      = DMA_CH_BURST_LENGTH_1;
    dmaInitTx->SrcBurstLen      = DMA_CH_BURST_LENGTH_1;
    dmaInitTx->SrcTfrWidth = DMA_CH_TRANSFER_WIDTH_8;
    dmaInitTx->DstTfrWidth = DMA_CH_TRANSFER_WIDTH_8;
    dmaInitTx->SrcHandshaking = DMA_CH_SRC_HANDSHAKING_SOFTWARE;
    dmaInitTx->DstHandshaking = DMA_CH_DST_HANDSHAKING_HARDWARE;
    dmaInitTx->DstHsInterface = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)bus->dmaTx->ref);
    dmaInitTx->DstHsInterfacePol = DMA_CH_HANDSHAKING_IF_POL_H;
    dmaInitTx->BlkTfrSize = 0;

    if (bus->dmaRx) {
        DMA_InitTypeDef *dmaInitRx = bus->dmaInitRx;

        DMA_ChannelStructInit(dmaInitRx);
        dmaInitRx->TfrType = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
        dmaInitRx->TfrTypeFlowCtrl = DMA_CH_TRANSFER_FLOW_P2M_DMA;
        dmaInitRx->ChannelPriority = DMA_CH_PRIORITY_3;
        dmaInitRx->SrcAddr = (uint32_t)&instance->DAT;
        dmaInitRx->DstAddr = 0;
        dmaInitRx->SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        dmaInitRx->DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
        dmaInitRx->SrcTfrWidth = DMA_CH_TRANSFER_WIDTH_8;
        dmaInitRx->DstTfrWidth = DMA_CH_TRANSFER_WIDTH_8;
        dmaInitRx->DstBurstLen    = DMA_CH_BURST_LENGTH_1;
        dmaInitRx->SrcBurstLen    = DMA_CH_BURST_LENGTH_1;
        dmaInitRx->SrcHandshaking = DMA_CH_SRC_HANDSHAKING_HARDWARE;
        dmaInitRx->SrcHsInterface = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)bus->dmaRx->ref);
        dmaInitRx->SrcHsInterfacePol = DMA_CH_HANDSHAKING_IF_POL_H;
        dmaInitRx->DstHandshaking = DMA_CH_DST_HANDSHAKING_SOFTWARE;
        dmaInitRx->BlkTfrSize = 0;
    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    xDMA_Cmd(descriptor->ref, DISABLE);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF);
}

void spiInternalInitStream(const extDevice_t *dev, volatile busSegment_t *segment)
{
    STATIC_DMA_DATA_AUTO uint8_t dummyTxByte = 0xFF;
    STATIC_DMA_DATA_AUTO uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;
    const int len = segment->len;

    DMA_InitTypeDef *dmaInitTx = bus->dmaInitTx;
    if (segment->u.buffers.txData) {
        dmaInitTx->SrcAddr = (uint32_t)segment->u.buffers.txData;
        dmaInitTx->SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
    } else {
        dummyTxByte = 0xFF;
        dmaInitTx->SrcAddr = (uint32_t)&dummyTxByte;
        dmaInitTx->SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
    }
    dmaInitTx->BlkTfrSize = len;

    if (bus->dmaRx) {
        DMA_InitTypeDef *dmaInitRx = bus->dmaInitRx;
        if (segment->u.buffers.rxData) {
            dmaInitRx->DstAddr = (uint32_t)segment->u.buffers.rxData;
            dmaInitRx->DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
        } else {
            dmaInitRx->DstAddr = (uint32_t)&dummyRxByte;
            dmaInitRx->DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        }
        dmaInitRx->BlkTfrSize = len;
    }
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;

    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;

    while (SPI_I2S_GetStatus(instance, SPI_I2S_BUSY_FLAG) == SET) {
    }

    dmaTx->userParam = (uint32_t)dev;
    dmaRx->userParam = (uint32_t)dev;

    DMA_CLEAR_FLAG(dmaTx, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF);
    DMA_CLEAR_FLAG(dmaRx, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF);

    xDMA_Cmd(dmaTx->ref, DISABLE);
    xDMA_Cmd(dmaRx->ref, DISABLE);

    xDMA_ITConfig(dmaRx->ref, DMA_IT_TCIF | DMA_IT_TEIF, ENABLE);
    xDMA_Init(dmaTx->ref, dev->bus->dmaInitTx);
    xDMA_Init(dmaRx->ref, dev->bus->dmaInitRx);

    xDMA_Cmd(dmaRx->ref, ENABLE);
    xDMA_Cmd(dmaTx->ref, ENABLE);

    SPI_I2S_EnableDma(instance, SPI_I2S_DMA_RX, ENABLE);
    SPI_I2S_EnableDma(instance, SPI_I2S_DMA_TX, ENABLE);
}

void spiInternalStopDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;

    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;

    xDMA_Cmd(dmaTx->ref, DISABLE);
    xDMA_Cmd(dmaRx->ref, DISABLE);

    DMA_CLEAR_FLAG(dmaTx, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF);
    DMA_CLEAR_FLAG(dmaRx, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF);

    while (SPI_I2S_GetStatus(instance, SPI_I2S_BUSY_FLAG) == SET) {
    }
    while (SPI_I2S_GetStatus(instance, SPI_I2S_RNE_FLAG) == SET) {
        (void)SPI_I2S_ReceiveData(instance);
    }

    SPI_I2S_EnableDma(instance, SPI_I2S_DMA_TX | SPI_I2S_DMA_RX, DISABLE);
}

bool spiInternalReadWriteBufPolled(spiResource_t *spiInstance, const uint8_t *txData, uint8_t *rxData, int len)
{
    SPI_TypeDef *instance = (SPI_TypeDef *)spiInstance;
    while (len--) {
        const uint8_t tx = txData ? *(txData++) : 0xFF;

        while (SPI_I2S_GetStatus(instance, SPI_I2S_TE_FLAG) == RESET) {
        }
        SPI_I2S_TransmitData(instance, tx);

        while (SPI_I2S_GetStatus(instance, SPI_I2S_RNE_FLAG) == RESET) {
        }

        const uint8_t rx = (uint8_t)SPI_I2S_ReceiveData(instance);
        if (rxData) {
            *(rxData++) = rx;
        }
    }

    return true;
}

void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    bus->initSegment = true;

    if ((bus->busType_u.spi.speed != dev->busType_u.spi.speed) ||
        (bus->busType_u.spi.leadingEdge != dev->busType_u.spi.leadingEdge)) {
        spiApplyConfig(instance, dev->busType_u.spi.speed, dev->busType_u.spi.leadingEdge);
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        segmentCount++;
        xferLen += checkSegment->len;
    }

    if (bus->useDMA &&
        ((segmentCount > 1) ||
         (xferLen >= SPI_DMA_THRESHOLD) ||
         !bus->curSegment[segmentCount].negateCS)) {
        spiProcessSegmentsDMA(dev);
    } else {
        spiProcessSegmentsPolled(dev);
    }
}

#endif
