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
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_UART

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/inverter.h"
#include "drivers/dma.h"
#include "platform/dma.h"
#include "platform/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

static void usartConfigurePinInversion(uartPort_t *uartPort)
{
    USART_TypeDef *USARTx = (USART_TypeDef *)uartPort->USARTx;
    bool inverted = uartPort->port.options & SERIAL_INVERTED;

    if (inverted) {
        if (uartPort->port.mode & MODE_RX) {
            LL_USART_SetRXPinLevel(USARTx, LL_USART_RXPIN_LEVEL_INVERTED);
        }
        if (uartPort->port.mode & MODE_TX) {
            LL_USART_SetTXPinLevel(USARTx, LL_USART_TXPIN_LEVEL_INVERTED);
        }
    }
}

#if UART_TRAIT_PINSWAP
static void uartConfigurePinSwap(uartPort_t *uartPort)
{
    uartDevice_t *uartDevice = container_of(uartPort, uartDevice_t, port);
    if (uartDevice->pinSwap) {
        LL_USART_SetTXRXSwap((USART_TypeDef *)uartPort->USARTx, LL_USART_TXRX_SWAPPED);
    }
}
#endif

// XXX uartReconfigure does not handle resource management properly.

void uartReconfigure(uartPort_t *uartPort)
{
    USART_TypeDef *USARTx = (USART_TypeDef *)uartPort->USARTx;

    LL_USART_Disable(USARTx);
    LL_USART_DeInit(USARTx);

    LL_USART_InitTypeDef usartInit;
    LL_USART_StructInit(&usartInit);

    usartInit.BaudRate = uartPort->port.baudRate;
    usartInit.DataWidth = (uartPort->port.options & SERIAL_PARITY_EVEN) ? LL_USART_DATAWIDTH_9B : LL_USART_DATAWIDTH_8B;
    usartInit.StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? LL_USART_STOPBITS_2 : LL_USART_STOPBITS_1;
    usartInit.Parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? LL_USART_PARITY_EVEN : LL_USART_PARITY_NONE;
    usartInit.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    usartInit.OverSampling = LL_USART_OVERSAMPLING_16;

    uint32_t direction = 0;
    if (uartPort->port.mode & MODE_RX) {
        direction |= LL_USART_DIRECTION_RX;
    }
    if (uartPort->port.mode & MODE_TX) {
        direction |= LL_USART_DIRECTION_TX;
    }
    usartInit.TransferDirection = direction;

#if defined(STM32G4) || defined(STM32H7) || defined(STM32N6)
    if (USARTx == LPUART1) {
        usartInit.PrescalerValue = LL_USART_PRESCALER_DIV8;
    }
#endif

    LL_USART_Init(USARTx, &usartInit);

    usartConfigurePinInversion(uartPort);
#if UART_TRAIT_PINSWAP
    uartConfigurePinSwap(uartPort);
#endif

    LL_USART_DisableOverrunDetect(USARTx);
#if defined(STM32H7) || defined(STM32G4) || defined(STM32N6)
    LL_USART_DisableFIFO(USARTx);
#endif
    LL_USART_ConfigAsyncMode(USARTx);

#ifdef TARGET_USART_CONFIG
    void usartTargetConfigure(uartPort_t *);
    usartTargetConfigure(uartPort);
#endif

    if (uartPort->port.options & SERIAL_BIDIR) {
        LL_USART_EnableHalfDuplex(USARTx);
    }

    LL_USART_Enable(USARTx);

    // Receive DMA or IRQ
    if (uartPort->port.mode & MODE_RX) {
#ifdef USE_DMA
        if (uartPort->rxDMAResource) {
            xLL_EX_DMA_ConfigStream(uartPort->rxDMAResource,
                uartPort->rxDMAChannel, LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
                LL_USART_DMA_GetRegAddr(USARTx, LL_USART_DMA_REG_DATA_RECEIVE),
                (uint32_t)uartPort->port.rxBuffer,
                uartPort->port.rxBufferSize, LL_DMA_MODE_CIRCULAR);
            xLL_EX_DMA_EnableResource(uartPort->rxDMAResource);
            LL_USART_EnableDMAReq_RX(USARTx);

            uartPort->rxDMAPos = xLL_EX_DMA_GetDataLength(uartPort->rxDMAResource);
        } else
#endif
        {
            /* Enable the UART Parity Error Interrupt */
            SET_BIT(USARTx->CR1, USART_CR1_PEIE);

            /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            SET_BIT(USARTx->CR3, USART_CR3_EIE);

            /* Enable the UART Data Register not empty Interrupt */
            SET_BIT(USARTx->CR1, USART_CR1_RXNEIE);

            /* Enable Idle Line detection */
            SET_BIT(USARTx->CR1, USART_CR1_IDLEIE);
        }
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
#ifdef USE_DMA
        if (uartPort->txDMAResource) {
            xLL_EX_DMA_ConfigStream(uartPort->txDMAResource,
                uartPort->txDMAChannel, LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
                LL_USART_DMA_GetRegAddr(USARTx, LL_USART_DMA_REG_DATA_TRANSMIT),
                0, 0, LL_DMA_MODE_NORMAL);
            xLL_EX_DMA_EnableIT_TC(uartPort->txDMAResource);
            SET_BIT(USARTx->CR1, USART_CR1_TCIE);
        } else
#endif
        {
            /* Enable the UART Transmit Data Register Empty Interrupt */
            SET_BIT(USARTx->CR1, USART_CR1_TXEIE);
            SET_BIT(USARTx->CR1, USART_CR1_TCIE);
        }
    }
}

bool checkUsartTxOutput(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);
    IO_t txIO = IOGetByTag(uart->tx.pin);

    if ((uart->txPinState == TX_PIN_MONITOR) && txIO) {
        if (IORead(txIO)) {
            // TX is high so we're good to transmit

            // Enable USART TX output
            uart->txPinState = TX_PIN_ACTIVE;
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uart->tx.af);

            // Enable the UART transmitter
            SET_BIT(((USART_TypeDef *)s->USARTx)->CR1, USART_CR1_TE);

            return true;
        } else {
            // TX line is pulled low so don't enable USART TX
            return false;
        }
    }

    return true;
}

void uartTxMonitor(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);

    if (uart->txPinState == TX_PIN_ACTIVE) {
        IO_t txIO = IOGetByTag(uart->tx.pin);

        // Disable the UART transmitter
        CLEAR_BIT(((USART_TypeDef *)s->USARTx)->CR1, USART_CR1_TE);

        // Switch TX to an input with pullup so it's state can be monitored
        uart->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(txIO, IOCFG_IPU);
    }
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    USART_TypeDef *USARTx = (USART_TypeDef *)s->USARTx;

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
        LL_USART_ClearFlag_TC(USARTx);

        xLL_EX_DMA_SetMemoryAddress(s->txDMAResource, (uint32_t)&s->port.txBuffer[fromwhere]);
        xLL_EX_DMA_SetDataLength(s->txDMAResource, size);
        xLL_EX_DMA_EnableResource(s->txDMAResource);
        LL_USART_EnableDMAReq_TX(USARTx);
    }
}

static void handleUsartTxDma(uartPort_t *s)
{
    uartTryStartTxDMA(s);
}

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);

    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

#ifdef STM32G4
    // G4's DMA does not auto-clear EN bit upon completion.
    // Clear it explicitly so IS_DMA_ENABLED works in uartTryStartTxDMA().
    xLL_EX_DMA_DisableResource(s->txDMAResource);
#endif

    handleUsartTxDma(s);
}
#endif

FAST_IRQ_HANDLER void uartIrqHandler(uartPort_t *s)
{
    USART_TypeDef *USARTx = (USART_TypeDef *)s->USARTx;

    /* UART in mode Receiver ---------------------------------------------------*/
    if (LL_USART_IsActiveFlag_RXNE(USARTx)) {
        uint8_t rbyte = (uint8_t)(USARTx->RDR & 0xFF);

        if (s->port.rxCallback) {
            s->port.rxCallback(rbyte, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = rbyte;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    /* UART parity error interrupt occurred -------------------------------------*/
    if (LL_USART_IsActiveFlag_PE(USARTx)) {
        LL_USART_ClearFlag_PE(USARTx);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if (LL_USART_IsActiveFlag_FE(USARTx)) {
        LL_USART_ClearFlag_FE(USARTx);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if (LL_USART_IsActiveFlag_NE(USARTx)) {
        LL_USART_ClearFlag_NE(USARTx);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if (LL_USART_IsActiveFlag_ORE(USARTx)) {
        LL_USART_ClearFlag_ORE(USARTx);
    }

    // UART transmission completed
    if (LL_USART_IsActiveFlag_TC(USARTx)) {
        LL_USART_ClearFlag_TC(USARTx);

        // Switch TX to an input with pull-up so it's state can be monitored
        uartTxMonitor(s);

#ifdef USE_DMA
        if (s->txDMAResource) {
            handleUsartTxDma(s);
        }
#endif
    }

    if (
#ifdef USE_DMA
        !s->txDMAResource &&
#endif
        LL_USART_IsActiveFlag_TXE(USARTx)) {
        /* Check that a Tx process is ongoing */
        if (s->port.txBufferTail == s->port.txBufferHead) {
            /* Disable the UART Transmit Data Register Empty Interrupt */
            CLEAR_BIT(USARTx->CR1, USART_CR1_TXEIE);
        } else {
            if ((USARTx->CR1 & USART_CR1_M) && !(USARTx->CR1 & USART_CR1_PS)) {
                // 9-bit word length without parity
                USARTx->TDR = ((uint16_t)s->port.txBuffer[s->port.txBufferTail]) & 0x01FFU;
            } else {
                USARTx->TDR = (uint8_t)(s->port.txBuffer[s->port.txBufferTail]);
            }
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        }
    }

    // UART reception idle detected

    if (LL_USART_IsActiveFlag_IDLE(USARTx)) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }

        LL_USART_ClearFlag_IDLE(USARTx);
    }

}
#endif // USE_UART
