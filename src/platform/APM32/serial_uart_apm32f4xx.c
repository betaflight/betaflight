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

/*
 * jflyper - Refactoring, cleanup and made pin-configurable
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/debug.h"

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_USART1,
        .reg = USART1,
        .rxDMAChannel = DMA_CHANNEL_4,
        .txDMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART1_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA2_Stream5,
#endif
#ifdef USE_UART1_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA2_Stream7,
#endif
        .rxPins = { { DEFIO_TAG_E(PA10) }, { DEFIO_TAG_E(PB7) },
            },
        .txPins = { { DEFIO_TAG_E(PA9) }, { DEFIO_TAG_E(PB6) },
            },
        .af = GPIO_AF7_USART1,
        .rcc = RCC_APB2(USART1),
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
        .reg = USART2,
        .rxDMAChannel = DMA_CHANNEL_4,
        .txDMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART2_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA1_Stream5,
#endif
#ifdef USE_UART2_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA1_Stream6,
#endif
        .rxPins = { { DEFIO_TAG_E(PA3) }, { DEFIO_TAG_E(PD6) } },
        .txPins = { { DEFIO_TAG_E(PA2) }, { DEFIO_TAG_E(PD5) } },
        .af = GPIO_AF7_USART2,
        .rcc = RCC_APB1(USART2),
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
        .reg = USART3,
        .rxDMAChannel = DMA_CHANNEL_4,
        .txDMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART3_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA1_Stream1,
#endif
#ifdef USE_UART3_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA1_Stream3,
#endif
        .rxPins = { { DEFIO_TAG_E(PB11) }, { DEFIO_TAG_E(PC11) }, { DEFIO_TAG_E(PD9) } },
        .txPins = { { DEFIO_TAG_E(PB10) }, { DEFIO_TAG_E(PC10) }, { DEFIO_TAG_E(PD8) } },
        .af = GPIO_AF7_USART3,
        .rcc = RCC_APB1(USART3),
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
        .reg = UART4,
        .rxDMAChannel = DMA_CHANNEL_4,
        .txDMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART4_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA1_Stream2,
#endif
#ifdef USE_UART4_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA1_Stream4,
#endif
        .rxPins = { { DEFIO_TAG_E(PA1) }, { DEFIO_TAG_E(PC11) } },
        .txPins = { { DEFIO_TAG_E(PA0) }, { DEFIO_TAG_E(PC10) } },
        .af = GPIO_AF8_UART4,
        .rcc = RCC_APB1(UART4),
        .irqn = UART4_IRQn,
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
        .reg = UART5,
        .rxDMAChannel = DMA_CHANNEL_4,
        .txDMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART5_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA1_Stream0,
#endif
#ifdef USE_UART5_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA1_Stream7,
#endif
        .rxPins = { { DEFIO_TAG_E(PD2) } },
        .txPins = { { DEFIO_TAG_E(PC12) } },
        .af = GPIO_AF8_UART5,
        .rcc = RCC_APB1(UART5),
        .irqn = UART5_IRQn,
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
        .reg = USART6,
        .rxDMAChannel = DMA_CHANNEL_5,
        .txDMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART6_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA2_Stream1,
#endif
#ifdef USE_UART6_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA2_Stream6,
#endif
        .rxPins = { { DEFIO_TAG_E(PC7) }, { DEFIO_TAG_E(PG9) } },
        .txPins = { { DEFIO_TAG_E(PC6) }, { DEFIO_TAG_E(PG14) } },
        .af = GPIO_AF8_USART6,
        .rcc = RCC_APB2(USART6),
        .irqn = USART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif
};

bool checkUsartTxOutput(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);
    IO_t txIO = IOGetByTag(uart->tx.pin);

    if ((uart->txPinState == TX_PIN_MONITOR) && txIO) {
        if (IORead(txIO)) {
            // TX is high so we're good to transmit

            // Enable USART TX output
            uart->txPinState = TX_PIN_ACTIVE;
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uart->hardware->af);

            // Enable the UART transmitter
            SET_BIT(s->Handle.Instance->CTRL1, USART_CTRL1_TXEN);

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
        CLEAR_BIT(s->Handle.Instance->CTRL1, USART_CTRL1_TXEN);

        // Switch TX to an input with pullup so it's state can be monitored
        uart->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(txIO, IOCFG_IPU);
    }
}

static void handleUsartTxDma(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);

    uartTryStartTxDMA(s);

    if (s->txDMAEmpty && (uart->txPinState != TX_PIN_IGNORE)) {
        // Switch TX to an input with pullup so it's state can be monitored
        uartTxMonitor(s);
    }
}

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
        if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_FEIF))
        {
            DMA_CLEAR_FLAG(descriptor, DMA_IT_FEIF);
        }
        handleUsartTxDma(s);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_DMEIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_DMEIF);
    }
}

FAST_IRQ_HANDLER void uartIrqHandler(uartPort_t *s)
{
    UART_HandleTypeDef *huart = &s->Handle;
    uint32_t isrflags = READ_REG(huart->Instance->STS);
    uint32_t cr1its = READ_REG(huart->Instance->CTRL1);
    uint32_t cr3its = READ_REG(huart->Instance->CTRL3);
    /* UART in mode Receiver ---------------------------------------------------*/
    if (!s->rxDMAResource && (((isrflags & USART_STS_RXBNEFLG) != RESET) && ((cr1its & USART_CTRL1_RXBNEIEN) != RESET))) {
        if (s->port.rxCallback) {
            s->port.rxCallback(huart->Instance->DATA, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = huart->Instance->DATA;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    // Detect completion of transmission
    if (((isrflags & USART_STS_TXCFLG) != RESET) && ((cr1its & USART_CTRL1_TXCIEN) != RESET)) {
        // Switch TX to an input with pullup so it's state can be monitored
        uartTxMonitor(s);

        __DAL_UART_CLEAR_FLAG(huart, UART_IT_TC);
    }

    if (!s->txDMAResource && (((isrflags & USART_STS_TXBEFLG) != RESET) && ((cr1its & USART_CTRL1_TXBEIEN) != RESET))) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            huart->Instance->DATA = (((uint16_t) s->port.txBuffer[s->port.txBufferTail]) & (uint16_t) 0x01FFU);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            __DAL_UART_DISABLE_IT(huart, UART_IT_TXE);
        }
    }

    if (((isrflags & USART_STS_OVREFLG) != RESET) && (((cr1its & USART_CTRL1_RXBNEIEN) != RESET)
                                                 || ((cr3its & USART_CTRL3_ERRIEN) != RESET))) {
        __DAL_UART_CLEAR_OREFLAG(huart);
    }

    if (((isrflags & USART_STS_IDLEFLG) != RESET) && ((cr1its & USART_STS_IDLEFLG) != RESET)) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }

        // clear
        (void) huart->Instance->STS;
        (void) huart->Instance->DATA;
    }
}

#endif // USE_UART
