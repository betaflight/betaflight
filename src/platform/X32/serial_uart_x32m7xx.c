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

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "platform/rcc.h"

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_USART1,
        .reg = (usartResource_t *)USART1,
        .rxPins = {
            { DEFIO_TAG_E(PA10), GPIO_AF5 },
            { DEFIO_TAG_E(PB7),  GPIO_AF8 },
            { DEFIO_TAG_E(PB15), GPIO_AF9 },
            { DEFIO_TAG_E(PF1),  GPIO_AF9 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA9),  GPIO_AF7 },
            { DEFIO_TAG_E(PB6),  GPIO_AF7 },
            { DEFIO_TAG_E(PF0),  GPIO_AF8 },
            { DEFIO_TAG_E(PB14), GPIO_AF9 },
        },
        .rcc = RCC_APB1_3(USART1),
        .irqn = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif
#ifdef USE_UART2
    {
        .identifier = SERIAL_PORT_USART2,
        .reg = (usartResource_t *)USART2,
        .rxPins = {
            { DEFIO_TAG_E(PA3), GPIO_AF11 },
            { DEFIO_TAG_E(PD6), GPIO_AF7 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA2), GPIO_AF7 },
            { DEFIO_TAG_E(PD5), GPIO_AF6 },
        },
        .rcc = RCC_APB1_3(USART2),
        .irqn = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2,
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif
#ifdef USE_UART3
    {
        .identifier = SERIAL_PORT_USART3,
        .reg = (usartResource_t *)USART3,
        .rxPins = {
            { DEFIO_TAG_E(PB11), GPIO_AF6 },
            { DEFIO_TAG_E(PC11), GPIO_AF7 },
            { DEFIO_TAG_E(PD9),  GPIO_AF5 },
        },
        .txPins = {
            { DEFIO_TAG_E(PB10), GPIO_AF9 },
            { DEFIO_TAG_E(PC10), GPIO_AF7 },
            { DEFIO_TAG_E(PD8),  GPIO_AF6 },
        },
        .rcc = RCC_APB1_3(USART3),
        .irqn = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3,
        .txBuffer = uart3TxBuffer,
        .rxBuffer = uart3RxBuffer,
        .txBufferSize = sizeof(uart3TxBuffer),
        .rxBufferSize = sizeof(uart3RxBuffer),
    },
#endif
#ifdef USE_UART4
    {
        .identifier = SERIAL_PORT_UART4,
        .reg = (usartResource_t *)USART4,
        .rxPins = {
            { DEFIO_TAG_E(PC6),  GPIO_AF10 },
            { DEFIO_TAG_E(PG14), GPIO_AF10 },
        },
        .txPins = {
            { DEFIO_TAG_E(PC7),  GPIO_AF10 },
            { DEFIO_TAG_E(PG9),  GPIO_AF7 },
        },
        .rcc = RCC_APB1_3(USART4),
        .irqn = USART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4,
        .txBuffer = uart4TxBuffer,
        .rxBuffer = uart4RxBuffer,
        .txBufferSize = sizeof(uart4TxBuffer),
        .rxBufferSize = sizeof(uart4RxBuffer),
    },
#endif
#ifdef USE_UART5
    {
        .identifier = SERIAL_PORT_UART5,
        .reg = (usartResource_t *)USART5,
        .rxPins = {
            { DEFIO_TAG_E(PE2),  GPIO_AF6 },
            { DEFIO_TAG_E(PG11), GPIO_AF9 },
        },
        .txPins = {
            { DEFIO_TAG_E(PE3),  GPIO_AF4 },
            { DEFIO_TAG_E(PG12), GPIO_AF12 },
        },
        .rcc = RCC_APB2_3(USART5),
        .irqn = USART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART5,
        .txBuffer = uart5TxBuffer,
        .rxBuffer = uart5RxBuffer,
        .txBufferSize = sizeof(uart5TxBuffer),
        .rxBufferSize = sizeof(uart5RxBuffer),
    },
#endif
#ifdef USE_UART6
    {
        .identifier = SERIAL_PORT_USART6,
        .reg = (usartResource_t *)USART6,
        .rxPins = {
            { DEFIO_TAG_E(PE4),  GPIO_AF6 },
            { DEFIO_TAG_E(PH14), GPIO_AF7 },
        },
        .txPins = {
            { DEFIO_TAG_E(PE5),  GPIO_AF6 },
            { DEFIO_TAG_E(PH15), GPIO_AF8 },
        },
        .rcc = RCC_APB2_3(USART6),
        .irqn = USART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif
#ifdef USE_UART7
    {
        .identifier = SERIAL_PORT_USART7,
        .reg = (usartResource_t *)USART7,
        .rxPins = {
            { DEFIO_TAG_E(PF15), GPIO_AF7 },
            { DEFIO_TAG_E(PH5),  GPIO_AF6 },
        },
        .txPins = {
            { DEFIO_TAG_E(PF14), GPIO_AF6 },
            { DEFIO_TAG_E(PH4),  GPIO_AF6 },
        },
        .rcc = RCC_APB2_3(USART7),
        .irqn = USART7_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART7_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART7,
        .txBuffer = uart7TxBuffer,
        .rxBuffer = uart7RxBuffer,
        .txBufferSize = sizeof(uart7TxBuffer),
        .rxBufferSize = sizeof(uart7RxBuffer),
    },
#endif
#ifdef USE_UART8
    {
        .identifier = SERIAL_PORT_USART8,
        .reg = (usartResource_t *)USART8,
        .rxPins = {
            { DEFIO_TAG_E(PI2), GPIO_AF8 },
            { DEFIO_TAG_E(PI4), GPIO_AF8 },
        },
        .txPins = {
            { DEFIO_TAG_E(PI3), GPIO_AF8 },
            { DEFIO_TAG_E(PI5), GPIO_AF9 },
        },
        .rcc = RCC_APB2_3(USART8),
        .irqn = USART8_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART8_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART8,
        .txBuffer = uart8TxBuffer,
        .rxBuffer = uart8RxBuffer,
        .txBufferSize = sizeof(uart8TxBuffer),
        .rxBufferSize = sizeof(uart8RxBuffer),
    },
#endif
#ifdef USE_UART9
    {
        .identifier = SERIAL_PORT_UART9,
        .reg = (usartResource_t *)UART9,
        .rxPins = {
            { DEFIO_TAG_E(PA1),  GPIO_AF9 },
            { DEFIO_TAG_E(PA11), GPIO_AF7 },
            { DEFIO_TAG_E(PB8),  GPIO_AF9 },
            { DEFIO_TAG_E(PC11), GPIO_AF8 },
            { DEFIO_TAG_E(PD0),  GPIO_AF6 },
            { DEFIO_TAG_E(PH14), GPIO_AF8 },
            { DEFIO_TAG_E(PI9),  GPIO_AF7 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA0),  GPIO_AF11 },
            { DEFIO_TAG_E(PA12), GPIO_AF7 },
            { DEFIO_TAG_E(PB9),  GPIO_AF8 },
            { DEFIO_TAG_E(PC10), GPIO_AF8 },
            { DEFIO_TAG_E(PD1),  GPIO_AF5 },
            { DEFIO_TAG_E(PH13), GPIO_AF6 },
        },
        .rcc = RCC_APB1_3(UART9),
        .irqn = UART9_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART9_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART9,
        .txBuffer = uart9TxBuffer,
        .rxBuffer = uart9RxBuffer,
        .txBufferSize = sizeof(uart9TxBuffer),
        .rxBufferSize = sizeof(uart9RxBuffer),
    },
#endif
#ifdef USE_UART10
    {
        .identifier = SERIAL_PORT_USART10,
        .reg = (usartResource_t *)UART10,
        .rxPins = {
            { DEFIO_TAG_E(PB5),  GPIO_AF10 },
            { DEFIO_TAG_E(PB12), GPIO_AF9 },
            { DEFIO_TAG_E(PD2),  GPIO_AF10 },
            { DEFIO_TAG_E(PH11), GPIO_AF7 },
        },
        .txPins = {
            { DEFIO_TAG_E(PB6),  GPIO_AF8 },
            { DEFIO_TAG_E(PB13), GPIO_AF9 },
            { DEFIO_TAG_E(PC12), GPIO_AF11 },
            { DEFIO_TAG_E(PH12), GPIO_AF9 },
        },
        .rcc = RCC_APB1_3(UART10),
        .irqn = UART10_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART10_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART10,
        .txBuffer = uart10TxBuffer,
        .rxBuffer = uart10RxBuffer,
        .txBufferSize = sizeof(uart10TxBuffer),
        .rxBufferSize = sizeof(uart10RxBuffer),
    },
#endif
};

bool checkUsartTxOutput(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);
    IO_t txIO = IOGetByTag(uart->tx.pin);

    if ((uart->txPinState == TX_PIN_MONITOR) && txIO) {
        if (IORead(txIO)) {
            uart->txPinState = TX_PIN_ACTIVE;
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uart->tx.af);
            SET_BIT(((USART_Module*)s->USARTx)->CTRL1, USART_MODE_TX);
            return true;
        }

        return false;
    }

    return true;
}

void uartTxMonitor(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);

    if (uart->txPinState == TX_PIN_ACTIVE) {
        IO_t txIO = IOGetByTag(uart->tx.pin);
        CLEAR_BIT(((USART_Module*)s->USARTx)->CTRL1, USART_MODE_TX);
        uart->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(txIO, IOCFG_IPU);
    }
}

void uartIrqHandler(uartPort_t *s)
{
    /*
     * Snapshot STS and the enable registers once. Reading STS is the first
     * half of the documented clear sequence for IDLE / OREF / FEF / NEF /
     * PEF flags; the matching DAT read below completes the clear.
     *
     * We also snapshot CTRL1/CTRL3 to preserve the "only act on an event
     * when its interrupt is enabled" semantics that USART_GetIntStatus()
     * used to provide. This matters because:
     *   - TXDE flag is latched-high whenever the TX register is empty, so
     *     a raw STS test would spuriously enter the TX block on every IRQ.
     *   - IDLE is a sticky flag; in DMA RX mode the IDLE interrupt is
     *     disabled and we must NOT read DAT (the DMA controller owns it).
     *   - Keeps ERRF handling from firing when error interrupt is masked.
     */
    const uint32_t sts   = ((USART_Module*)s->USARTx)->STS;
    const uint32_t ctrl1 = ((USART_Module*)s->USARTx)->CTRL1;
    const uint32_t ctrl3 = ((USART_Module*)s->USARTx)->CTRL3;

    const bool rxneSet = (sts & USART_FLAG_RXDNE)
                      && (ctrl1 & USART_CTRL1_RXDNEIEN)
                      && !s->rxDMAResource;
    const bool idleSet = (sts & USART_FLAG_IDLEF)
                      && (ctrl1 & USART_CTRL1_IDLEIEN);
    const bool errSet  = (sts & (USART_FLAG_OREF | USART_FLAG_FEF | USART_FLAG_NEF | USART_FLAG_PEF))
                      && (ctrl3 & USART_CTRL3_ERRIEN);

    /*
     * Read DAT at most ONCE per IRQ entry. A single DAT read both consumes
     * the received byte (if RXNE was set) and clears all sticky error/IDLE
     * flags that were set when STS was snapshotted.
     *
     * The previous implementation read DAT separately in the RXNE, ERRF
     * and IDLE blocks. If a new byte arrived between the RXNE read and one
     * of the later reads (very likely at >= 420 kbps when rxCallback work
     * takes tens of microseconds), the second/third DAT read silently
     * consumed that new byte without delivering it to the rxCallback,
     * breaking frame-oriented protocols like CRSF.
     */
    if (rxneSet || idleSet || errSet) {
        const uint16_t byte = ((USART_Module*)s->USARTx)->DAT;

        if (rxneSet) {
            if (s->port.rxCallback) {
                s->port.rxCallback(byte, s->port.rxCallbackData);
            } else {
                s->port.rxBuffer[s->port.rxBufferHead] = byte;
                s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
            }
        }
        /* If RXNE was not set, 'byte' is stale data read solely to clear
         * IDLE/OREF/FEF/NEF/PEF and is intentionally discarded. */

        if (idleSet && s->port.idleCallback) {
            s->port.idleCallback();
        }
    }

    /* UART Transmission Complete Interrupt */
    if ((sts & USART_FLAG_TXC) && (ctrl1 & USART_CTRL1_TXCIEN)) {
        uartTxMonitor(s);
        USART_ClrFlag((USART_Module*)(s->USARTx), USART_FLAG_TXC);
    }

    if (!s->txDMAResource
        && (sts & USART_FLAG_TXDE)
        && (ctrl1 & USART_CTRL1_TXDEIEN)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            USART_SendData((USART_Module*)(s->USARTx), s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            USART_ConfigInt((USART_Module*)(s->USARTx), USART_INT_TXDE, DISABLE);
        }
    }
}

#endif
