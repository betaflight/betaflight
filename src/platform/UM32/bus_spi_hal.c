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
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "platform/rcc.h"
#if SPI_TRAIT_HANDLE
#include "platform/bus_spi_hal.h"
#endif

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

#if SPI_TRAIT_HANDLE
static struct spiHalHandle_s spiHalHandles[SPIDEV_COUNT];
#endif

void Error_Handler(void);

static uint32_t spiDivisorToBRbits(const SPI_TypeDef *instance, uint16_t divisor)
{
    UNUSED(instance);
    divisor = constrain(divisor, 2, 0xffff);
    return divisor; 
}

static void spiSetDivisorBRreg(SPI_TypeDef *instance, uint16_t divisor)
{
    instance->SPBRG = spiDivisorToBRbits(instance, divisor);
}

void spiInitDevice(spiDevice_e device)
{
    spiDevice_t *spi = &spiDevice[device];
/*    tfp_printf("spi->dev: 0x%08X\n", spi->dev);  */

    if (!spi->dev) {
        return;
    }

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_SDI_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->af); 


#if SPI_TRAIT_HANDLE
    spi->halHandle = &spiHalHandles[device];

    SPI_HandleTypeDef *pHandle = &spi->halHandle->hal;

    memset(pHandle, 0, sizeof(*pHandle));

    pHandle->Instance = (SPI_TypeDef *)spi->dev;

    pHandle->Init.Direction_Mode    = SPI_EX_FULL_DUPLEX;
    pHandle->Init.RXTLF             = SPI_EX_RXTLF_1;
    pHandle->Init.TXTLF             = SPI_EX_TXTLF_1;
    pHandle->Init.DMA_Mode          = SPI_EX_DMAMODE_CLOSE;
    pHandle->Init.NSS               = SPI_EX_NSS_SOFT;   
    pHandle->Init.SPI_Len           = SPI_EX_SPILEN_BIT_8;
    pHandle->Init.BaudRatePrescaler = 60;
    pHandle->Init.CLKPhase          = SPI_EX_PHASE_1EDGE;
    pHandle->Init.CLKPolarity       = SPI_EX_POLARITY_LOW;
    pHandle->Init.DataSize          = SPI_EX_DATASIZE_8BIT;
    pHandle->Init.FirstBit          = SPI_EX_FIRSTBIT_MSB;
    pHandle->Init.TIMode            = SPI_EX_TIMODE_MOTOROLA;
    pHandle->Init.TXEDGE            = SPI_EX_TXEDGE_PCLK;
    pHandle->Init.Mode              = SPI_EX_MODE_MASTER;

    if(HAL_SPI_EX_Init(pHandle) != HAL_OK)
    {
        /* Initialization Error */
/*        tfp_printf("HAL_SPI_EX_Init Error!\n");  */
        Error_Handler();
    } 

    __HAL_SPI_EX_ENABLE(pHandle);
#endif
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    const dmaChannelSpec_t *dmaTxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDO, spiDeviceByInstance(bus->busType_u.spi.instance), 0);
    const dmaChannelSpec_t *dmaRxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDI, spiDeviceByInstance(bus->busType_u.spi.instance), 0);

    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;
    LL_DMA_InitTypeDef *dmaInitTx = bus->dmaInitTx;

    dmaInitTx->SrcAddress  = 0x00000000U;
    dmaInitTx->DstAddress  = 0x00000000U;
    dmaInitTx->SrcPer = 0;
    dmaInitTx->DstPer = 0;
    dmaInitTx->NbData   = 0x00000000U;

    if (dmaTxChannelSpec) {
        dmaInitTx->SrcPer = DMA_SRC_HANDSHAKING(0);
        dmaInitTx->DstPer = DMA_DST_HANDSHAKING(dmaTxChannelSpec->code);
    }

    dmaInitTx->DstAddress = (uint32_t)&(instance->TXREG);
    dmaInitTx->Direction = LL_DMA_MEMORY_TO_PERIPH;                /* M2P transfer mode            */
    dmaInitTx->SrcDataAlignment = LL_DMA_SRCDATAALIGN_BYTE;            /* source data size: Byte       */
    dmaInitTx->DstDataAlignment = LL_DMA_DSTDATAALIGN_BYTE;            /* destination data size: Byte  */
   
    dmaInitTx->FIFOMode = LL_DMA_FIFOMODE_DISABLE;                /* FIFO mode disabled           */
    dmaInitTx->FCMode = LL_DMA_FCMODE_ENABLE;                      /* FC mode enabled              */

    dmaInitTx->SrcMSize = DMA_BURST_SRC_NUM_1;                   /* source burst                 */
    dmaInitTx->DstMSize = DMA_BURST_DST_NUM_1;                   /* dest burst                   */
    dmaInitTx->SrcHsSel = LL_DMA_SRC_HS_HW;                /* dst shake hand               */
    dmaInitTx->DstHsSel = LL_DMA_DST_HS_HW;                /* src shake hand               */
    dmaInitTx->DstReload = LL_DMA_DST_RELOAD_DISABLE;             /* dst auto reload              */
    dmaInitTx->SrcReload = LL_DMA_SRC_RELOAD_DISABLE;             /* src auto reload              */
	dmaInitTx->SrcInc = LL_DMA_SRCINC_INC;							  /* src address increase         */
	dmaInitTx->DstInc = LL_DMA_DSTINC_NOC;							  /* dst address unchanged        */
    dmaInitTx->Priority = LL_DMA_PRIORITY_7;
    if (bus->dmaInitRx) {
        LL_DMA_InitTypeDef *dmaInitRx = bus->dmaInitRx;

        dmaInitRx->SrcAddress  = 0x00000000U;
        dmaInitRx->DstAddress  = 0x00000000U;
        dmaInitRx->SrcPer = 0;
        dmaInitRx->DstPer = 0;
        dmaInitRx->NbData   = 0x00000000U;

        if (dmaRxChannelSpec) {
            dmaInitRx->SrcPer  = DMA_SRC_HANDSHAKING(dmaRxChannelSpec->code);
            dmaInitRx->DstPer  = DMA_SRC_HANDSHAKING(0);
        }

        dmaInitRx->SrcAddress = (uint32_t)&(instance->RXREG);
        dmaInitRx->Direction = LL_DMA_PERIPH_TO_MEMORY;         /* M2P transfer mode            */
        dmaInitRx->SrcDataAlignment = LL_DMA_SRCDATAALIGN_BYTE; /* source data size: Byte       */
        dmaInitRx->DstDataAlignment = LL_DMA_DSTDATAALIGN_BYTE; /* destination data size: Byte  */
        dmaInitRx->FIFOMode = LL_DMA_FIFOMODE_DISABLE;            /* FIFO mode disabled           */
        dmaInitRx->FCMode = LL_DMA_FCMODE_ENABLE;                 /* FC mode enabled              */

        dmaInitRx->SrcMSize = LL_DMA_BURST_SRC_NUM_1;             /* source burst                 */
        dmaInitRx->DstMSize = LL_DMA_BURST_DST_NUM_1;             /* dest burst                   */
        dmaInitRx->SrcHsSel = LL_DMA_SRC_HS_HW;                   /* dst shake hand               */
        dmaInitRx->DstHsSel = LL_DMA_DST_HS_SW;                   /* src shake hand               */
        dmaInitRx->DstReload = LL_DMA_DST_RELOAD_DISABLE;         /* dst auto reload              */
        dmaInitRx->SrcReload = LL_DMA_SRC_RELOAD_DISABLE;         /* src auto reload              */
        dmaInitRx->SrcInc = LL_DMA_SRCINC_NOC;                    /* src address increase         */
        dmaInitRx->DstInc = LL_DMA_DSTINC_INC;                    /* dst address unchanged        */
        dmaInitRx->Priority = LL_DMA_PRIORITY_7;
    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    DMA_Stream_TypeDef *streamRegs = (DMA_Stream_TypeDef *)descriptor->ref;
    
    LL_EX_DMA_DisableResource(streamRegs);

    // Clear any pending interrupt flags
    DMA_CLEAR_FLAG(descriptor, CLEARBLOCK);
}


FAST_CODE bool spiInternalReadWriteBufPolled(spiResource_t *spiInstance, const uint8_t *txData, uint8_t *rxData, int len)
{
    uint8_t b;
    SPI_TypeDef *instance = (SPI_TypeDef *)spiInstance;

    while (len--) {
        b = txData ? *(txData++) : 0xFF;
        instance->TXREG = b;
        
        while ((instance->CSTAT & SPI_EX_STATE_TXEPT) != SPI_EX_STATE_TXEPT);

        while ((instance->CSTAT & SPI_EX_STATE_RXAVL) != SPI_EX_STATE_RXAVL);
      
        b = instance->RXREG;
        if (rxData) {
            *(rxData++) = b;
        }
    }

    return true;
}

void spiInternalInitStream(const extDevice_t *dev, volatile busSegment_t *segment)
{
    STATIC_DMA_DATA_AUTO uint8_t dummyTxByte = 0xff;
    STATIC_DMA_DATA_AUTO uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;
    // volatile busSegment_t *segment = bus->curSegment;
    
    // if (preInit) {
    //     // Prepare the init structure for the next segment to reduce inter-segment interval
    //     segment++;
    //     if(segment->len == 0) {
    //         // There's no following segment
    //         return;
    //     }
    // }

    int len = segment->len;
    uint8_t *txData = segment->u.buffers.txData;
    LL_DMA_InitTypeDef *dmaInitTx = bus->dmaInitTx;
    if (txData) {
        dmaInitTx->SrcAddress = (uint32_t)txData;
        dmaInitTx->DstAddress = (uint32_t)&(instance->TXREG);
        dmaInitTx->SrcInc = LL_DMA_SRCINC_INC;							  /* src address increase         */
	    dmaInitTx->DstInc = LL_DMA_DSTINC_NOC;	
    } else {
        dummyTxByte = 0xff;
        dmaInitTx->SrcAddress = (uint32_t)&dummyTxByte;
        dmaInitTx->DstAddress = (uint32_t)&(instance->TXREG);
        dmaInitTx->SrcInc = LL_DMA_SRCINC_NOC;							  /* src address increase         */
	    dmaInitTx->DstInc = LL_DMA_DSTINC_NOC;	
    }
    dmaInitTx->NbData = len;


    if (dev->bus->dmaRx) {
        uint8_t *rxData = segment->u.buffers.rxData;
        LL_DMA_InitTypeDef *dmaInitRx = bus->dmaInitRx;

        if (rxData) {
            /* Flush the D cache for the start and end of the receive buffer as
             * the cache will be invalidated after the transfer and any valid data
             * just before/after must be in memory at that point
             */
            dmaInitRx->SrcAddress = (uint32_t)&(instance->RXREG);
            dmaInitRx->DstAddress = (uint32_t)rxData;
            dmaInitRx->SrcInc = LL_DMA_SRCINC_NOC;                    /* src address increase         */
            dmaInitRx->DstInc = LL_DMA_DSTINC_INC;                    /* dst address unchanged        */
         } else {
            dmaInitRx->SrcAddress = (uint32_t)&(instance->RXREG);
            dmaInitRx->DstAddress = (uint32_t)&dummyRxByte;
            dmaInitRx->SrcInc = LL_DMA_SRCINC_NOC;                    /* src address increase         */
            dmaInitRx->DstInc = LL_DMA_DSTINC_NOC;                    /* dst address unchanged        */
        }
        dmaInitRx->NbData = len;
    }
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;

    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    LL_DMA_InitTypeDef *initTx = bus->dmaInitTx;
    LL_DMA_InitTypeDef *initRx = bus->dmaInitRx;

    DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;
    if (dmaRx) {
         DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;

         // Use the correct callback argument
        dmaRx->userParam = (uint32_t)dev;

        xLL_EX_DMA_Init(streamRegsTx, initTx);
        xLL_EX_DMA_Init(streamRegsRx, initRx);

        xLL_EX_DMA_EnableIT_TC(streamRegsRx);
        
        CLEAR_BIT(instance->GCTL,SPI_EX_GCTL_TXEN|SPI_EX_GCTL_RXEN); 
        SET_BIT(instance->GCTL,SPI_EX_GCTL_DMAMODE);
        SET_BIT(instance->GCTL,SPI_EX_GCTL_TXEN|SPI_EX_GCTL_RXEN);

        xLL_EX_DMA_EnableResource(streamRegsRx);
        xLL_EX_DMA_EnableResource(streamRegsTx);
    }
    else
    {
        // Use the correct callback argument
        dmaTx->userParam = (uint32_t)dev;

        xLL_EX_DMA_Init(streamRegsTx, initTx);

        CLEAR_BIT(instance->GCTL,SPI_EX_GCTL_TXEN); 
        SET_BIT(instance->GCTL,SPI_EX_GCTL_DMAMODE);
        SET_BIT(instance->GCTL,SPI_EX_GCTL_TXEN);
        
        xLL_EX_DMA_EnableResource(streamRegsTx);
    }
}

void spiInternalStopDMA (const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    
    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;

    DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;
    DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;
    // Disable the SPI DMA Mode 
    CLEAR_BIT(instance->GCTL,SPI_EX_GCTL_DMAMODE);

    if (dmaRx) {
    // Disable the DMA engine and SPI interface    
        LL_EX_DMA_DisableResource(streamRegsTx);
        LL_EX_DMA_DisableResource(streamRegsRx);

        DMA_CLEAR_FLAG(dmaTx, CLEARBLOCK);
        DMA_CLEAR_FLAG(dmaRx, CLEARBLOCK);
    }
    else
    {
        DMA_CLEAR_FLAG(dmaTx, CLEARBLOCK);
        DMA_CLEAR_FLAG(dmaRx, CLEARBLOCK);
    }
}

// DMA transfer setup and start
FAST_CODE void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = (SPI_TypeDef *)bus->busType_u.spi.instance;
/*    spiDevice_t *spi = &spiDevice[spiDeviceByInstance(bus->busType_u.spi.instance)];  */
    bool dmaSafe = dev->useDMA;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    bus->initSegment = true;

    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        spiSetDivisorBRreg(instance, dev->busType_u.spi.speed);
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    // Switch SPI clock polarity/phase if necessary
    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        // Switch SPI clock polarity/phase
        instance->CCTL &= ~(SPI_EX_CCTL_CKPH | SPI_EX_CCTL_CKPL);

        // Apply setting
        if (dev->busType_u.spi.leadingEdge) {
            instance->CCTL |= SPI_EX_POLARITY_LOW | SPI_EX_PHASE_1EDGE;
        } else
        {
            instance->CCTL |= SPI_EX_POLARITY_HIGH | SPI_EX_PHASE_2EDGE;
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
