/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "rcc.h"
#include "drivers/nvic.h"
#include "dma.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

static void handleUsartTxDma(uartPort_t *s);

#define UART_RX_BUFFER_SIZE UART1_RX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE UART1_TX_BUFFER_SIZE

typedef enum UARTDevice {
    UARTDEV_1 = 0,
    UARTDEV_2 = 1,
    UARTDEV_3 = 2,
    UARTDEV_4 = 3,
    UARTDEV_5 = 4,
    UARTDEV_6 = 5,
    UARTDEV_7 = 6,
    UARTDEV_8 = 7
} UARTDevice;

typedef struct uartDevice_s {
    USART_TypeDef* dev;
    uartPort_t port;
    uint32_t DMAChannel;
    DMA_Stream_TypeDef *txDMAStream;
    DMA_Stream_TypeDef *rxDMAStream;
    ioTag_t rx;
    ioTag_t tx;
    volatile uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
    volatile uint8_t txBuffer[UART_TX_BUFFER_SIZE];
    uint32_t rcc_ahb1;
    rccPeriphTag_t rcc_apb2;
    rccPeriphTag_t rcc_apb1;
    uint8_t af;
    uint8_t txIrq;
    uint8_t rxIrq;
    uint32_t txPriority;
    uint32_t rxPriority;
} uartDevice_t;

//static uartPort_t uartPort[MAX_UARTS];
#ifdef USE_UART1
static uartDevice_t uart1 =
{
    .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART1_RX_DMA
    .rxDMAStream = DMA2_Stream5,
#endif
    .txDMAStream = DMA2_Stream7,
    .dev = USART1,
    .rx = IO_TAG(UART1_RX_PIN),
    .tx = IO_TAG(UART1_TX_PIN),
    .af = GPIO_AF7_USART1,
#ifdef UART1_AHB1_PERIPHERALS
    .rcc_ahb1 = UART1_AHB1_PERIPHERALS,
#endif
    .rcc_apb2 = RCC_APB2(USART1),
    .txIrq = DMA2_ST7_HANDLER,
    .rxIrq = USART1_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART1
};
#endif

#ifdef USE_UART2
static uartDevice_t uart2 =
{
    .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART2_RX_DMA
    .rxDMAStream = DMA1_Stream5,
#endif
    .txDMAStream = DMA1_Stream6,
    .dev = USART2,
    .rx = IO_TAG(UART2_RX_PIN),
    .tx = IO_TAG(UART2_TX_PIN),
    .af = GPIO_AF7_USART2,
#ifdef UART2_AHB1_PERIPHERALS
    .rcc_ahb1 = UART2_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(USART2),
    .txIrq = DMA1_ST6_HANDLER,
    .rxIrq = USART2_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART2
};
#endif

#ifdef USE_UART3
static uartDevice_t uart3 =
{
    .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART3_RX_DMA
    .rxDMAStream = DMA1_Stream1,
#endif
    .txDMAStream = DMA1_Stream3,
    .dev = USART3,
    .rx = IO_TAG(UART3_RX_PIN),
    .tx = IO_TAG(UART3_TX_PIN),
    .af = GPIO_AF7_USART3,
#ifdef UART3_AHB1_PERIPHERALS
    .rcc_ahb1 = UART3_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(USART3),
    .txIrq = DMA1_ST3_HANDLER,
    .rxIrq = USART3_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART3
};
#endif

#ifdef USE_UART4
static uartDevice_t uart4 =
{
    .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART4_RX_DMA
    .rxDMAStream = DMA1_Stream2,
#endif
    .txDMAStream = DMA1_Stream4,
    .dev = UART4,
    .rx = IO_TAG(UART4_RX_PIN),
    .tx = IO_TAG(UART4_TX_PIN),
    .af = GPIO_AF8_UART4,
#ifdef UART4_AHB1_PERIPHERALS
    .rcc_ahb1 = UART4_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(UART4),
    .txIrq = DMA1_ST4_HANDLER,
    .rxIrq = UART4_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART4
};
#endif

#ifdef USE_UART5
static uartDevice_t uart5 =
{
    .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART5_RX_DMA
    .rxDMAStream = DMA1_Stream0,
#endif
    .txDMAStream = DMA1_Stream7,
    .dev = UART5,
    .rx = IO_TAG(UART5_RX_PIN),
    .tx = IO_TAG(UART5_TX_PIN),
    .af = GPIO_AF8_UART5,
#ifdef UART5_AHB1_PERIPHERALS
    .rcc_ahb1 = UART5_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(UART5),
    .txIrq = DMA1_ST7_HANDLER,
    .rxIrq = UART5_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART5
};
#endif

#ifdef USE_UART6
static uartDevice_t uart6 =
{
    .DMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART6_RX_DMA
    .rxDMAStream = DMA2_Stream1,
#endif
    .txDMAStream = DMA2_Stream6,
    .dev = USART6,
    .rx = IO_TAG(UART6_RX_PIN),
    .tx = IO_TAG(UART6_TX_PIN),
    .af = GPIO_AF8_USART6,
#ifdef UART6_AHB1_PERIPHERALS
    .rcc_ahb1 = UART6_AHB1_PERIPHERALS,
#endif
    .rcc_apb2 = RCC_APB2(USART6),
    .txIrq = DMA2_ST6_HANDLER,
    .rxIrq = USART6_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART6
};
#endif

#ifdef USE_UART7
static uartDevice_t uart7 =
{
    .DMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART7_RX_DMA
    .rxDMAStream = DMA1_Stream3,
#endif
    .txDMAStream = DMA1_Stream1,
    .dev = UART7,
    .rx = IO_TAG(UART7_RX_PIN),
    .tx = IO_TAG(UART7_TX_PIN),
    .af = GPIO_AF8_UART7,
#ifdef UART7_AHB1_PERIPHERALS
    .rcc_ahb1 = UART7_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(UART7),
    .txIrq = DMA1_ST1_HANDLER,
    .rxIrq = UART7_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART7_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART7
};
#endif
#ifdef USE_UART8
static uartDevice_t uart8 =
{
    .DMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART8_RX_DMA
    .rxDMAStream = DMA1_Stream6,
#endif
    .txDMAStream = DMA1_Stream0,
    .dev = UART8,
    .rx = IO_TAG(UART8_RX_PIN),
    .tx = IO_TAG(UART8_TX_PIN),
    .af = GPIO_AF8_UART8,
#ifdef UART8_AHB1_PERIPHERALS
    .rcc_ahb1 = UART8_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(UART8),
    .txIrq = DMA1_ST0_HANDLER,
    .rxIrq = UART8_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART8_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART8
};
#endif



static uartDevice_t* uartHardwareMap[] = {
#ifdef USE_UART1
    &uart1,
#else
    NULL,
#endif
#ifdef USE_UART2
    &uart2,
#else
    NULL,
#endif
#ifdef USE_UART3
    &uart3,
#else
    NULL,
#endif
#ifdef USE_UART4
    &uart4,
#else
    NULL,
#endif
#ifdef USE_UART5
    &uart5,
#else
    NULL,
#endif
#ifdef USE_UART6
    &uart6,
#else
    NULL,
#endif
#ifdef USE_UART7
    &uart7,
#else
    NULL,
#endif
#ifdef USE_UART8
    &uart8,
#else
    NULL,
#endif
};

void uartIrqHandler(uartPort_t *s)
{
    UART_HandleTypeDef *huart = &s->Handle;
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET))
    {
        uint8_t rbyte = (uint8_t)(huart->Instance->RDR & (uint8_t)0xff);

        if (s->port.rxCallback) {
            s->port.rxCallback(rbyte);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = rbyte;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE));

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    }

    /* UART parity error interrupt occurred -------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    }

    /* UART in mode Transmitter ------------------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_TXE) != RESET))
    {
        HAL_UART_IRQHandler(huart);
    }

    /* UART in mode Transmitter (transmission end) -----------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET))
    {
        HAL_UART_IRQHandler(huart);
        handleUsartTxDma(s);
    }

}

static void handleUsartTxDma(uartPort_t *s)
{
    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
    {
        s->txDMAEmpty = true;
    }
}

void dmaIRQHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
    HAL_DMA_IRQHandler(&s->txDMAHandle);
}

uartPort_t *serialUART(UARTDevice device, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;

    uartDevice_t *uart = uartHardwareMap[device];
    if (!uart) return NULL;

    s = &(uart->port);
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uart->rxBuffer;
    s->port.txBuffer = uart->txBuffer;
    s->port.rxBufferSize = sizeof(uart->rxBuffer);
    s->port.txBufferSize = sizeof(uart->txBuffer);

    s->USARTx = uart->dev;
    if (uart->rxDMAStream) {
        s->rxDMAChannel = uart->DMAChannel;
        s->rxDMAStream = uart->rxDMAStream;
    }
    s->txDMAChannel = uart->DMAChannel;
    s->txDMAStream = uart->txDMAStream;

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->TDR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->RDR;

    s->Handle.Instance = uart->dev;

    IO_t tx = IOGetByTag(uart->tx);
    IO_t rx = IOGetByTag(uart->rx);

    if (options & SERIAL_BIDIR) {
        IOInit(tx, OWNER_SERIAL, RESOURCE_UART_TXRX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(tx, IOCFG_AF_PP, uart->af);
    }
    else {
        if (mode & MODE_TX) {
            IOInit(tx, OWNER_SERIAL, RESOURCE_UART_TX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(tx, IOCFG_AF_PP, uart->af);
        }

        if (mode & MODE_RX) {
            IOInit(rx, OWNER_SERIAL, RESOURCE_UART_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rx, IOCFG_AF_PP, uart->af);
        }
    }

    // DMA TX Interrupt
    dmaSetHandler(uart->txIrq, dmaIRQHandler, uart->txPriority, (uint32_t)uart);


    //HAL_NVIC_SetPriority(uart->txIrq, NVIC_PRIORITY_BASE(uart->txPriority), NVIC_PRIORITY_SUB(uart->txPriority));
    //HAL_NVIC_EnableIRQ(uart->txIrq);

    if (!s->rxDMAChannel)
    {
        HAL_NVIC_SetPriority(uart->rxIrq, NVIC_PRIORITY_BASE(uart->rxPriority), NVIC_PRIORITY_SUB(uart->rxPriority));
        HAL_NVIC_EnableIRQ(uart->rxIrq);
    }

    return s;
}

#ifdef USE_UART1
uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_1, baudRate, mode, options);
}

// USART1 Rx/Tx IRQ Handler
void USART1_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_1]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART2
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_2, baudRate, mode, options);
}

// USART2 Rx/Tx IRQ Handler
void USART2_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_2]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART3
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_3, baudRate, mode, options);
}

// USART3 Rx/Tx IRQ Handler
void USART3_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_3]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART4
uartPort_t *serialUART4(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_4, baudRate, mode, options);
}

// UART4 Rx/Tx IRQ Handler
void UART4_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_4]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART5
uartPort_t *serialUART5(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_5, baudRate, mode, options);
}

// UART5 Rx/Tx IRQ Handler
void UART5_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_5]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART6
uartPort_t *serialUART6(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_6, baudRate, mode, options);
}

// USART6 Rx/Tx IRQ Handler
void USART6_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_6]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART7
uartPort_t *serialUART7(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_7, baudRate, mode, options);
}

// UART7 Rx/Tx IRQ Handler
void UART7_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_7]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART8
uartPort_t *serialUART8(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_8, baudRate, mode, options);
}

// UART8 Rx/Tx IRQ Handler
void UART8_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_8]->port);
    uartIrqHandler(s);
}
#endif
