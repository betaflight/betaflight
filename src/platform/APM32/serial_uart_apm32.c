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

// XXX uartReconfigure does not handle resource management properly.

void uartReconfigure(uartPort_t *uartPort)
{
    DAL_UART_DeInit(&uartPort->halHandle->hal);
    uartPort->halHandle->hal.Init.BaudRate = uartPort->port.baudRate;
    // according to the stm32 documentation wordlen has to be 9 for parity bits
    // this does not seem to matter for rx but will give bad data on tx!
    uartPort->halHandle->hal.Init.WordLength = (uartPort->port.options & SERIAL_PARITY_EVEN) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
    uartPort->halHandle->hal.Init.StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_STOPBITS_2 : USART_STOPBITS_1;
    uartPort->halHandle->hal.Init.Parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_PARITY_EVEN : USART_PARITY_NONE;
    uartPort->halHandle->hal.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartPort->halHandle->hal.Init.OverSampling = UART_OVERSAMPLING_16;
    uartPort->halHandle->hal.Init.Mode = 0;

    if (uartPort->port.mode & MODE_RX)
        uartPort->halHandle->hal.Init.Mode |= UART_MODE_RX;
    if (uartPort->port.mode & MODE_TX)
        uartPort->halHandle->hal.Init.Mode |= UART_MODE_TX;

    // config external pin inverter (no internal pin inversion available)
    uartConfigureExternalPinInversion(uartPort);

#ifdef TARGET_USART_CONFIG
    // TODO: move declaration into header file
    void usartTargetConfigure(uartPort_t *);
    usartTargetConfigure(uartPort);
#endif

    if (uartPort->port.options & SERIAL_BIDIR) {
        DAL_HalfDuplex_Init(&uartPort->halHandle->hal);
    } else {
        DAL_UART_Init(&uartPort->halHandle->hal);
    }

    // Receive DMA or IRQ
    if (uartPort->port.mode & MODE_RX) {
        do {
#ifdef USE_DMA
            if (uartPort->rxDMAResource) {
                uartPort->rxDmaHalHandle->hal.Instance = (DMA_ARCH_TYPE *)uartPort->rxDMAResource;
                uartPort->txDmaHalHandle->hal.Init.Channel = uartPort->rxDMAChannel;
                uartPort->rxDmaHalHandle->hal.Init.Direction = DMA_PERIPH_TO_MEMORY;
                uartPort->rxDmaHalHandle->hal.Init.PeriphInc = DMA_PINC_DISABLE;
                uartPort->rxDmaHalHandle->hal.Init.MemInc = DMA_MINC_ENABLE;
                uartPort->rxDmaHalHandle->hal.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
                uartPort->rxDmaHalHandle->hal.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
                uartPort->rxDmaHalHandle->hal.Init.Mode = DMA_CIRCULAR;
#if defined(APM32F4)
                uartPort->rxDmaHalHandle->hal.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
                uartPort->rxDmaHalHandle->hal.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
                uartPort->rxDmaHalHandle->hal.Init.PeriphBurst = DMA_PBURST_SINGLE;
                uartPort->rxDmaHalHandle->hal.Init.MemBurst = DMA_MBURST_SINGLE;
#endif
                uartPort->rxDmaHalHandle->hal.Init.Priority = DMA_PRIORITY_MEDIUM;

                DAL_DMA_DeInit(&uartPort->rxDmaHalHandle->hal);
                DAL_DMA_Init(&uartPort->rxDmaHalHandle->hal);
                /* Associate the initialized DMA handle to the UART handle */
                __DAL_LINKDMA(&uartPort->halHandle->hal, hdmarx, uartPort->rxDmaHalHandle->hal);

                DAL_UART_Receive_DMA(&uartPort->halHandle->hal, (uint8_t*)uartPort->port.rxBuffer, uartPort->port.rxBufferSize);

                uartPort->rxDMAPos = __DAL_DMA_GET_COUNTER(&uartPort->rxDmaHalHandle->hal);
                break;
            }
#endif

            /* Enable the UART Parity Error Interrupt */
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CTRL1, USART_CTRL1_PEIEN);

            /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CTRL3, USART_CTRL3_ERRIEN);

            /* Enable the UART Data Register not empty Interrupt */
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CTRL1, USART_CTRL1_RXBNEIEN);

            /* Enable Idle Line detection */
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CTRL1, USART_CTRL1_IDLEIEN);
        } while(false);
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
        do {
#ifdef USE_DMA
            if (uartPort->txDMAResource) {
                uartPort->txDmaHalHandle->hal.Instance = (DMA_ARCH_TYPE *)uartPort->txDMAResource;
                uartPort->txDmaHalHandle->hal.Init.Channel = uartPort->txDMAChannel;
                uartPort->txDmaHalHandle->hal.Init.Direction = DMA_MEMORY_TO_PERIPH;
                uartPort->txDmaHalHandle->hal.Init.PeriphInc = DMA_PINC_DISABLE;
                uartPort->txDmaHalHandle->hal.Init.MemInc = DMA_MINC_ENABLE;
                uartPort->txDmaHalHandle->hal.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
                uartPort->txDmaHalHandle->hal.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
                uartPort->txDmaHalHandle->hal.Init.Mode = DMA_NORMAL;
                uartPort->txDmaHalHandle->hal.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
                uartPort->txDmaHalHandle->hal.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
                uartPort->txDmaHalHandle->hal.Init.PeriphBurst = DMA_PBURST_SINGLE;
                uartPort->txDmaHalHandle->hal.Init.MemBurst = DMA_MBURST_SINGLE;
                uartPort->txDmaHalHandle->hal.Init.Priority = DMA_PRIORITY_MEDIUM;

                DAL_DMA_DeInit(&uartPort->txDmaHalHandle->hal);
                DAL_StatusTypeDef status = DAL_DMA_Init(&uartPort->txDmaHalHandle->hal);
                if (status != DAL_OK) {
                    while (1);
                }
                /* Associate the initialized DMA handle to the UART handle */
                __DAL_LINKDMA(&uartPort->halHandle->hal, hdmatx, uartPort->txDmaHalHandle->hal);

                __DAL_DMA_SET_COUNTER(&uartPort->txDmaHalHandle->hal, 0);
                break;
            }
#endif
            /* Enable the UART Transmit Data Register Empty Interrupt */
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CTRL1, USART_CTRL1_TXBEIEN);
            SET_BIT(((USART_TypeDef *)uartPort->USARTx)->CTRL1, USART_CTRL1_TXCIEN);
        } while(false);
    }
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    // uartTryStartTxDMA must be protected, since it is called from
    // uartWrite and handleUsartTxDma (an ISR).

    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
        if (IS_DMA_ENABLED(s->txDMAResource)) {
            // DMA is already in progress
            return;
        }

        DAL_UART_StateTypeDef state = DAL_UART_GetState(&s->halHandle->hal);
        if ((state & DAL_UART_STATE_BUSY_TX) == DAL_UART_STATE_BUSY_TX) {
            // UART is still transmitting
            return;
        }

        if (s->port.txBufferHead == s->port.txBufferTail) {
            // No more data to transmit
            s->txDMAEmpty = true;
            return;
        }

        unsigned chunk;
        uint32_t fromwhere = s->port.txBufferTail;

        if (s->port.txBufferHead > s->port.txBufferTail) {
            chunk = s->port.txBufferHead - s->port.txBufferTail;
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            chunk = s->port.txBufferSize - s->port.txBufferTail;
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;
        DAL_UART_Transmit_DMA(&s->halHandle->hal, (uint8_t*)s->port.txBuffer + fromwhere, chunk);
    }
}
#endif
#endif // USE_UART
