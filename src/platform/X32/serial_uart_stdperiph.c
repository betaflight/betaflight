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
 *
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

#ifdef USE_UART

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/serial.h"
#include "drivers/nvic.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "platform/dma.h"

static void handleUsartTxDma(uartPort_t *uartPort)
{
    uartTryStartTxDMA(uartPort);

    if (uartPort->txDMAEmpty) {
        uartTxMonitor(uartPort);
    }
}

void uartReconfigure(uartPort_t *uartPort)
{
    uartDevice_t *uart = container_of(uartPort, uartDevice_t, port);
    USART_TypeDef *USARTx = (USART_TypeDef *)uartPort->USARTx;

    const bool useRxDma = ((uartPort->port.mode & MODE_RX) != 0) && (uartPort->rxDMAResource != 0);
    const bool useTxDma = ((uartPort->port.mode & MODE_TX) != 0) && (uartPort->txDMAResource != 0);

    USART_InitType init;
    USART_StructInit(&init);
    init.BaudRate            = uartPort->port.baudRate;
    init.WordLength          = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_WL_9B : USART_WL_8B;
    init.StopBits            = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_STPB_2 : USART_STPB_1;
    init.Parity              = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_PE_EVEN : USART_PE_NO;
    init.HardwareFlowControl = USART_HFCTRL_NONE;
    init.OverSampling        = USART_16OVER;
    init.Mode                = 0;

    if (uartPort->port.mode & MODE_RX) {
        init.Mode |= USART_MODE_RX;
    }
    if (uartPort->port.mode & MODE_TX) {
        init.Mode |= USART_MODE_TX;
    }

    USART_DeInit(USARTx);
    USART_Init(USARTx, &init);

#if UART_TRAIT_PINSWAP
    if (uart->pinSwap) {
        SET_BIT(USARTx->CTRL1, USART_CTRL1_SWAP);
    } else {
        CLEAR_BIT(USARTx->CTRL1, USART_CTRL1_SWAP);
    }
#endif

    USART_EnableHalfDuplex(USARTx, (uartPort->port.options & SERIAL_BIDIR) ? ENABLE : DISABLE);

    DMA_InitTypeDef dmaInit;
    if (uartPort->port.mode & MODE_RX) {
#ifdef USE_DMA
        if (useRxDma) {
            DMA_ChannelStructInit(&dmaInit);

            dmaInit.IntEn              = 0x0U;
            dmaInit.SrcAddr            = uartPort->rxDMAPeripheralBaseAddr;
            dmaInit.DstAddr            = (uint32_t)uartPort->port.rxBuffer;
            dmaInit.SrcTfrWidth        = DMA_CH_TRANSFER_WIDTH_8;
            dmaInit.DstTfrWidth        = DMA_CH_TRANSFER_WIDTH_8;
            dmaInit.DstAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
            dmaInit.SrcAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
            dmaInit.DstBurstLen        = DMA_CH_BURST_LENGTH_1;
            dmaInit.SrcBurstLen        = DMA_CH_BURST_LENGTH_1;
            dmaInit.SrcGatherEn        = 0x0U;
            dmaInit.DstScatterEn       = 0x0U;
            dmaInit.TfrTypeFlowCtrl    = DMA_CH_TRANSFER_FLOW_P2M_DMA;
            dmaInit.BlkTfrSize         = uartPort->port.rxBufferSize;
            dmaInit.pLinkListItem      = NULL;
            dmaInit.SrcGatherInterval  = 0x0U;
            dmaInit.SrcGatherCount     = 0x0U;
            dmaInit.DstScatterInterval = 0x0U;
            dmaInit.DstScatterCount    = 0x0U;
            dmaInit.TfrType            = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
            dmaInit.ChannelPriority    = DMA_CH_PRIORITY_3;
            dmaInit.SrcHandshaking     = DMA_CH_SRC_HANDSHAKING_HARDWARE;
            dmaInit.DstHandshaking     = DMA_CH_DST_HANDSHAKING_SOFTWARE;
            dmaInit.SrcHsInterface     = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)uartPort->rxDMAResource);
            dmaInit.SrcHsInterfacePol = DMA_CH_HANDSHAKING_IF_POL_H;

            xDMA_DeInit(uartPort->rxDMAResource);
            xDMA_Init(uartPort->rxDMAResource, &dmaInit);
            xDMA_DestinationAddressReloadCmd(uartPort->rxDMAResource, ENABLE);
            /* Clean dirty lines first so adjacent variables sharing a cache line are
             * not silently discarded by the subsequent Invalidate. Then Invalidate so
             * the CPU re-fetches DMA-written data from RAM on next access. */
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
            if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
                const uint32_t rxBufAddr = (uint32_t)uartPort->port.rxBuffer & ~(uint32_t)CACHE_LINE_MASK;
                const int32_t  rxBufSize = (int32_t)(((uint32_t)uartPort->port.rxBuffer - rxBufAddr
                                                       + uartPort->port.rxBufferSize + CACHE_LINE_SIZE - 1)
                                                      & ~(uint32_t)CACHE_LINE_MASK);
                SCB_CleanDCache_by_Addr((uint32_t *)rxBufAddr, rxBufSize);
                SCB_InvalidateDCache_by_Addr((uint32_t *)rxBufAddr, rxBufSize);
            }
#endif
            xDMA_Cmd(uartPort->rxDMAResource, ENABLE);
            USART_EnableDMA(USARTx, USART_DMAREQ_RX, ENABLE);
            // rxDMAPos starts at rxBufferSize (DOWN-counting: "nothing consumed yet"),
            // matching serial_uart.c's formula: bytes_waiting = rxDMAPos - dmaGetDataLength().
            uartPort->rxDMAPos = uartPort->port.rxBufferSize;

            // IDLE interrupt only needed for rxCallback mode (uartProcessRxDMA delivery).
            if (uartPort->port.rxCallback) {
                USART_ConfigInt(USARTx, USART_INT_IDLEF, ENABLE);
            }
        } else
#endif
        {
            if (uartPort->rxDMAResource) {
                xDMA_DeInit(uartPort->rxDMAResource);
            }
            USART_EnableDMA(USARTx, USART_DMAREQ_RX, DISABLE);

            USART_ClrFlag(USARTx, USART_FLAG_RXDNE);
            USART_ConfigInt(USARTx, USART_INT_RXDNE, ENABLE);
            USART_ConfigInt(USARTx, USART_INT_IDLEF, ENABLE);
        }
    }

    if (uartPort->port.mode & MODE_TX) {
#ifdef USE_DMA
        if (useTxDma) {
            DMA_ChannelStructInit(&dmaInit);

            dmaInit.IntEn              = 0x1U;
            dmaInit.DstAddr            = uartPort->txDMAPeripheralBaseAddr;
            dmaInit.SrcAddr            = (uint32_t)uartPort->port.txBuffer;
            dmaInit.SrcTfrWidth        = DMA_CH_TRANSFER_WIDTH_8;
            dmaInit.DstTfrWidth        = DMA_CH_TRANSFER_WIDTH_8;
            dmaInit.DstAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
            dmaInit.SrcAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
            dmaInit.DstBurstLen        = DMA_CH_BURST_LENGTH_1;
            dmaInit.SrcBurstLen        = DMA_CH_BURST_LENGTH_1;
            dmaInit.SrcGatherEn        = 0x0U;
            dmaInit.DstScatterEn       = 0x0U;
            dmaInit.TfrTypeFlowCtrl    = DMA_CH_TRANSFER_FLOW_M2P_DMA;
            dmaInit.BlkTfrSize         = 0U;
            dmaInit.pLinkListItem      = NULL;
            dmaInit.SrcGatherInterval  = 0x0U;
            dmaInit.SrcGatherCount     = 0x0U;
            dmaInit.DstScatterInterval = 0x0U;
            dmaInit.DstScatterCount    = 0x0U;
            dmaInit.TfrType            = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
            dmaInit.ChannelPriority    = DMA_CH_PRIORITY_2;
            dmaInit.SrcHandshaking     = DMA_CH_SRC_HANDSHAKING_SOFTWARE;
            dmaInit.DstHandshaking     = DMA_CH_DST_HANDSHAKING_HARDWARE;
            dmaInit.DstHsInterface     = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)uartPort->txDMAResource);
            dmaInit.DstHsInterfacePol = DMA_CH_HANDSHAKING_IF_POL_H;

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
            if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
                const uint32_t txBufAddr = (uint32_t)uartPort->port.txBuffer & ~(uint32_t)CACHE_LINE_MASK;
                const int32_t  txBufSize = (int32_t)(((uint32_t)uartPort->port.txBuffer - txBufAddr
                                                       + uartPort->port.txBufferSize + CACHE_LINE_SIZE - 1)
                                                      & ~(uint32_t)CACHE_LINE_MASK);
                SCB_CleanInvalidateDCache_by_Addr((uint32_t *)txBufAddr, txBufSize);
            }
#endif
            xDMA_DeInit(uartPort->txDMAResource);
            xDMA_Init(uartPort->txDMAResource, &dmaInit);
            xDMA_SourceAddressReloadCmd(uartPort->txDMAResource, DISABLE);
            xDMA_ITConfig(uartPort->txDMAResource, DMA_IT_TCIF | DMA_IT_TEIF, ENABLE);
            xDMA_SetCurrDataCounter(uartPort->txDMAResource, 0);
            USART_EnableDMA(USARTx, USART_DMAREQ_TX, ENABLE);
            uartPort->txDMAEmpty = true;
        } else 
#endif
        {
            if (uartPort->txDMAResource) {
                xDMA_DeInit(uartPort->txDMAResource);
            }
            USART_EnableDMA(USARTx, USART_DMAREQ_TX, DISABLE);

            USART_ConfigInt(USARTx, USART_INT_TXDE, ENABLE);
        }

        USART_ConfigInt(USARTx, USART_INT_TXC, ENABLE);
    }

    USART_ConfigInt(USARTx, USART_INT_ERRF, ENABLE);

    USART_Enable(USARTx, ENABLE);
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *uartPort)
{
    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
        if (IS_DMA_ENABLED(uartPort->txDMAResource)) {
            return;
        }

        if (uartPort->port.txBufferHead == uartPort->port.txBufferTail) {
            uartPort->txDMAEmpty = true;
            return;
        }

        // X32 DMA auto-clears CHEN on block-transfer-complete, but the
        // SA/BTS fields retain their last values. Reprogram SA and BTS each time
        // before re-enabling. Save txTail now so the cache-clean address below
        // matches the SA programmed into the DMA channel.
        const uint32_t txTail = uartPort->port.txBufferTail;
        xDMA_MemoryTargetConfig(uartPort->txDMAResource,
            (uint32_t)&uartPort->port.txBuffer[txTail], 0);

        unsigned chunk;
        if (uartPort->port.txBufferHead > txTail) {
            chunk = uartPort->port.txBufferHead - txTail;
            uartPort->port.txBufferTail = uartPort->port.txBufferHead;
        } else {
            chunk = uartPort->port.txBufferSize - txTail;
            uartPort->port.txBufferTail = 0;
        }

        xDMA_SetCurrDataCounter(uartPort->txDMAResource, chunk);

        uartPort->txDMAEmpty = false;
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
        if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
            const uint32_t txChunkAddr = (uint32_t)&uartPort->port.txBuffer[txTail] & ~(uint32_t)CACHE_LINE_MASK;
            const int32_t  txChunkSize = (int32_t)(((uint32_t)&uartPort->port.txBuffer[txTail] - txChunkAddr
                                                     + chunk + CACHE_LINE_SIZE - 1)
                                                    & ~(uint32_t)CACHE_LINE_MASK);
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *)txChunkAddr, txChunkSize);
        }
#endif
        // Clear any stale event flags (BTC/STC/DTC/ERR) left from the previous
        // transfer before re-enabling, so the IRQ handler does not fire spuriously.
        xDMA_ClearFlag(uartPort->txDMAResource, DMA_IT_TCIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
        __DSB();
        xDMA_Cmd(uartPort->txDMAResource, ENABLE);
    }
}

void uartDmaIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    uartPort_t *uartPort = &(((uartDevice_t *)descriptor->userParam)->port);

    if ((DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF) & DMA_IT_TCIF) == DMA_IT_TCIF) {
        /* Clear Interrupt flag */
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        /* DMA interrupt callback */
        handleUsartTxDma(uartPort);
    }

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
    }
}
#endif

#endif
