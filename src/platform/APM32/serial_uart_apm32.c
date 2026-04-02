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
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/inverter.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "platform/dma.h"
#include "platform/serial_uart_hal.h"

struct uartHalHandle_s uartHalHandles[UARTDEV_COUNT];
struct dmaHalHandle_s uartRxDmaHalHandles[UARTDEV_COUNT];
struct dmaHalHandle_s uartTxDmaHalHandles[UARTDEV_COUNT];

void uartReconfigure(uartPort_t *uartPort)
{
    USART_TypeDef *USARTx = (USART_TypeDef *)uartPort->USARTx;

#ifdef USE_DMA
    // Disable USART DMA requests and stop DMA streams before reconfiguring
    if (uartPort->rxDMAResource) {
        DDL_USART_DisableDMAReq_RX(USARTx);
        xDDL_EX_DMA_DisableResource(uartPort->rxDMAResource);
    }
    if (uartPort->txDMAResource) {
        DDL_USART_DisableDMAReq_TX(USARTx);
        xDDL_EX_DMA_DisableResource(uartPort->txDMAResource);
    }
#endif

    DDL_USART_Disable(USARTx);
    DDL_USART_DeInit(USARTx);

    DDL_USART_InitTypeDef usartInit;
    DDL_USART_StructInit(&usartInit);

    usartInit.BaudRate = uartPort->port.baudRate;
    // according to the stm32 documentation wordlen has to be 9 for parity bits
    // this does not seem to matter for rx but will give bad data on tx!
    usartInit.DataWidth = (uartPort->port.options & SERIAL_PARITY_EVEN) ? DDL_USART_DATAWIDTH_9B : DDL_USART_DATAWIDTH_8B;
    usartInit.StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? DDL_USART_STOPBITS_2 : DDL_USART_STOPBITS_1;
    usartInit.Parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? DDL_USART_PARITY_EVEN : DDL_USART_PARITY_NONE;
    usartInit.HardwareFlowControl = DDL_USART_HWCONTROL_NONE;
    usartInit.OverSampling = DDL_USART_OVERSAMPLING_16;

    uint32_t direction = 0;
    if (uartPort->port.mode & MODE_RX)
        direction |= DDL_USART_DIRECTION_RX;
    if (uartPort->port.mode & MODE_TX)
        direction |= DDL_USART_DIRECTION_TX;
    usartInit.TransferDirection = direction;

    DDL_USART_Init(USARTx, &usartInit);

    // config external pin inverter (no internal pin inversion available on APM32F4)
    uartConfigureExternalPinInversion(uartPort);

    DDL_USART_ConfigAsyncMode(USARTx);

#ifdef TARGET_USART_CONFIG
    // TODO: move declaration into header file
    void usartTargetConfigure(uartPort_t *);
    usartTargetConfigure(uartPort);
#endif

    if (uartPort->port.options & SERIAL_BIDIR) {
        DDL_USART_EnableHalfDuplex(USARTx);
    }

    DDL_USART_Enable(USARTx);

    // Receive DMA or IRQ
    if (uartPort->port.mode & MODE_RX) {
#ifdef USE_DMA
        if (uartPort->rxDMAResource) {
            DDL_DMA_InitTypeDef dmaInit;

            dmaInit.Channel = uartPort->rxDMAChannel;
            dmaInit.Direction = DDL_DMA_DIRECTION_PERIPH_TO_MEMORY;
            dmaInit.PeriphOrM2MSrcAddress = DDL_USART_DMA_GetRegAddr(USARTx);
            dmaInit.MemoryOrM2MDstAddress = (uint32_t)uartPort->port.rxBuffer;
            dmaInit.PeriphOrM2MSrcIncMode = DDL_DMA_PERIPH_NOINCREMENT;
            dmaInit.MemoryOrM2MDstIncMode = DDL_DMA_MEMORY_INCREMENT;
            dmaInit.PeriphOrM2MSrcDataSize = DDL_DMA_PDATAALIGN_BYTE;
            dmaInit.MemoryOrM2MDstDataSize = DDL_DMA_MDATAALIGN_BYTE;
            dmaInit.Mode = DDL_DMA_MODE_CIRCULAR;
            dmaInit.NbData = uartPort->port.rxBufferSize;
            dmaInit.Priority = DDL_DMA_PRIORITY_MEDIUM;
            dmaInit.FIFOMode = DDL_DMA_FIFOMODE_DISABLE;
            dmaInit.FIFOThreshold = DDL_DMA_FIFOTHRESHOLD_1_4;
            dmaInit.MemBurst = DDL_DMA_MBURST_SINGLE;
            dmaInit.PeriphBurst = DDL_DMA_PBURST_SINGLE;

            xDDL_EX_DMA_DeInit(uartPort->rxDMAResource);
            xDDL_EX_DMA_Init(uartPort->rxDMAResource, &dmaInit);
            xDDL_EX_DMA_EnableResource(uartPort->rxDMAResource);
            DDL_USART_EnableDMAReq_RX(USARTx);

            uartPort->rxDMAPos = xDDL_EX_DMA_GetDataLength(uartPort->rxDMAResource);
        } else
#endif
        {
            /* Enable the UART Parity Error Interrupt */
            SET_BIT(USARTx->CTRL1, USART_CTRL1_PEIEN);

            /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            SET_BIT(USARTx->CTRL3, USART_CTRL3_ERRIEN);

            /* Enable the UART Data Register not empty Interrupt */
            SET_BIT(USARTx->CTRL1, USART_CTRL1_RXBNEIEN);

            /* Enable Idle Line detection */
            SET_BIT(USARTx->CTRL1, USART_CTRL1_IDLEIEN);
        }
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
#ifdef USE_DMA
        if (uartPort->txDMAResource) {
            DDL_DMA_InitTypeDef dmaInit;

            dmaInit.Channel = uartPort->txDMAChannel;
            dmaInit.Direction = DDL_DMA_DIRECTION_MEMORY_TO_PERIPH;
            dmaInit.PeriphOrM2MSrcAddress = DDL_USART_DMA_GetRegAddr(USARTx);
            dmaInit.MemoryOrM2MDstAddress = 0;
            dmaInit.PeriphOrM2MSrcIncMode = DDL_DMA_PERIPH_NOINCREMENT;
            dmaInit.MemoryOrM2MDstIncMode = DDL_DMA_MEMORY_INCREMENT;
            dmaInit.PeriphOrM2MSrcDataSize = DDL_DMA_PDATAALIGN_BYTE;
            dmaInit.MemoryOrM2MDstDataSize = DDL_DMA_MDATAALIGN_BYTE;
            dmaInit.Mode = DDL_DMA_MODE_NORMAL;
            dmaInit.NbData = 0;
            dmaInit.Priority = DDL_DMA_PRIORITY_MEDIUM;
            dmaInit.FIFOMode = DDL_DMA_FIFOMODE_DISABLE;
            dmaInit.FIFOThreshold = DDL_DMA_FIFOTHRESHOLD_1_4;
            dmaInit.MemBurst = DDL_DMA_MBURST_SINGLE;
            dmaInit.PeriphBurst = DDL_DMA_PBURST_SINGLE;

            xDDL_EX_DMA_DeInit(uartPort->txDMAResource);
            xDDL_EX_DMA_Init(uartPort->txDMAResource, &dmaInit);
            xDDL_EX_DMA_EnableIT_TC(uartPort->txDMAResource);
            SET_BIT(USARTx->CTRL1, USART_CTRL1_TXCIEN);
        } else
#endif
        {
            /* Enable the UART Transmit Data Register Empty Interrupt */
            SET_BIT(USARTx->CTRL1, USART_CTRL1_TXBEIEN);
            SET_BIT(USARTx->CTRL1, USART_CTRL1_TXCIEN);
        }
    }
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    USART_TypeDef *USARTx = (USART_TypeDef *)s->USARTx;

    // uartTryStartTxDMA must be protected, since it is called from
    // uartWrite and handleUsartTxDma (an ISR).

    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
        if (IS_DMA_ENABLED(s->txDMAResource)) {
            // DMA is already in progress
            return;
        }

        if (s->port.txBufferHead == s->port.txBufferTail) {
            // No more data to transmit
            s->txDMAEmpty = true;
            return;
        }

        uint16_t size;
        uint32_t fromwhere = s->port.txBufferTail;

        if (s->port.txBufferHead > s->port.txBufferTail) {
            size = s->port.txBufferHead - s->port.txBufferTail;
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            size = s->port.txBufferSize - s->port.txBufferTail;
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;

        // Clear TC flag before starting DMA
        DDL_USART_ClearFlag_TC(USARTx);

        xDDL_EX_DMA_SetMemoryAddress(s->txDMAResource, (uint32_t)&s->port.txBuffer[fromwhere]);
        xDDL_EX_DMA_SetDataLength(s->txDMAResource, size);
        xDDL_EX_DMA_EnableResource(s->txDMAResource);
        DDL_USART_EnableDMAReq_TX(USARTx);
    }
}
#endif
#endif // USE_UART
