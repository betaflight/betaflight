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

/*
 * jflyper - Refactoring, cleanup and made pin-configurable
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#ifdef USE_UART

static void handleUsartTxDma(uartPort_t *s);

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .reg = USART1,
        .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART1_RX_DMA
        .rxDMAStream = DMA2_Stream5,
#endif
#ifdef USE_UART1_TX_DMA
        .txDMAStream = DMA2_Stream7,
#endif
        .rxPins = { DEFIO_TAG_E(PA10), DEFIO_TAG_E(PB7), IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PA9), DEFIO_TAG_E(PB6), IO_TAG_NONE },
        .af = GPIO_AF7_USART1,
#ifdef UART1_AHB1_PERIPHERALS
        .rcc_ahb1 = UART1_AHB1_PERIPHERALS,
#endif
        .rcc_apb2 = RCC_APB2(USART1),
        .txIrq = DMA2_ST7_HANDLER,
        .rxIrq = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1
    },
#endif

#ifdef USE_UART2
    {
        .device = UARTDEV_2,
        .reg = USART2,
        .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART2_RX_DMA
        .rxDMAStream = DMA1_Stream5,
#endif
#ifdef USE_UART2_TX_DMA
        .txDMAStream = DMA1_Stream6,
#endif
        .rxPins = { DEFIO_TAG_E(PA3), DEFIO_TAG_E(PD6), IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PA2), DEFIO_TAG_E(PD5), IO_TAG_NONE },
        .af = GPIO_AF7_USART2,
#ifdef UART2_AHB1_PERIPHERALS
        .rcc_ahb1 = UART2_AHB1_PERIPHERALS,
#endif
        .rcc_apb1 = RCC_APB1(USART2),
        .txIrq = DMA1_ST6_HANDLER,
        .rxIrq = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2
    },
#endif

#ifdef USE_UART3
    {
        .device = UARTDEV_3,
        .reg = USART3,
        .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART3_RX_DMA
        .rxDMAStream = DMA1_Stream1,
#endif
#ifdef USE_UART3_TX_DMA
        .txDMAStream = DMA1_Stream3,
#endif
        .rxPins = { DEFIO_TAG_E(PB11), DEFIO_TAG_E(PC11), DEFIO_TAG_E(PD9) },
        .txPins = { DEFIO_TAG_E(PB10), DEFIO_TAG_E(PC10), DEFIO_TAG_E(PD8) },
        .af = GPIO_AF7_USART3,
#ifdef UART3_AHB1_PERIPHERALS
        .rcc_ahb1 = UART3_AHB1_PERIPHERALS,
#endif
        .rcc_apb1 = RCC_APB1(USART3),
        .txIrq = DMA1_ST3_HANDLER,
        .rxIrq = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3
    },
#endif

#ifdef USE_UART4
    {
        .device = UARTDEV_4,
        .reg = UART4,
        .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART4_RX_DMA
        .rxDMAStream = DMA1_Stream2,
#endif
#ifdef USE_UART4_TX_DMA
        .txDMAStream = DMA1_Stream4,
#endif
        .rxPins = { DEFIO_TAG_E(PA1), DEFIO_TAG_E(PC11), IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PA0), DEFIO_TAG_E(PC10), IO_TAG_NONE },
        .af = GPIO_AF8_UART4,
#ifdef UART4_AHB1_PERIPHERALS
        .rcc_ahb1 = UART4_AHB1_PERIPHERALS,
#endif
        .rcc_apb1 = RCC_APB1(UART4),
        .txIrq = DMA1_ST4_HANDLER,
        .rxIrq = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4
    },
#endif

#ifdef USE_UART5
    {
        .device = UARTDEV_5,
        .reg = UART5,
        .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART5_RX_DMA
        .rxDMAStream = DMA1_Stream0,
#endif
#ifdef USE_UART5_TX_DMA
        .txDMAStream = DMA1_Stream7,
#endif
        .rxPins = { DEFIO_TAG_E(PD2), IO_TAG_NONE, IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PC12), IO_TAG_NONE, IO_TAG_NONE },
        .af = GPIO_AF8_UART5,
#ifdef UART5_AHB1_PERIPHERALS
        .rcc_ahb1 = UART5_AHB1_PERIPHERALS,
#endif
        .rcc_apb1 = RCC_APB1(UART5),
        .txIrq = DMA1_ST7_HANDLER,
        .rxIrq = UART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART5
    },
#endif

#ifdef USE_UART6
    {
        .device = UARTDEV_6,
        .reg = USART6,
        .DMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART6_RX_DMA
        .rxDMAStream = DMA2_Stream1,
#endif
#ifdef USE_UART6_TX_DMA
        .txDMAStream = DMA2_Stream6,
#endif
        .rxPins = { DEFIO_TAG_E(PC7), DEFIO_TAG_E(PG9), IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PC6), DEFIO_TAG_E(PG14), IO_TAG_NONE },
        .af = GPIO_AF8_USART6,
#ifdef UART6_AHB1_PERIPHERALS
        .rcc_ahb1 = UART6_AHB1_PERIPHERALS,
#endif
        .rcc_apb2 = RCC_APB2(USART6),
        .txIrq = DMA2_ST6_HANDLER,
        .rxIrq = USART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6
    },
#endif

#ifdef USE_UART7
    {
        .device = UARTDEV_7,
        .reg = UART7,
        .DMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART7_RX_DMA
        .rxDMAStream = DMA1_Stream3,
#endif
#ifdef USE_UART7_TX_DMA
        .txDMAStream = DMA1_Stream1,
#endif
        .rxPins = { DEFIO_TAG_E(PE7), DEFIO_TAG_E(PF6), IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PE8), DEFIO_TAG_E(PF7), IO_TAG_NONE },
        .af = GPIO_AF8_UART7,
#ifdef UART7_AHB1_PERIPHERALS
        .rcc_ahb1 = UART7_AHB1_PERIPHERALS,
#endif
        .rcc_apb1 = RCC_APB1(UART7),
        .txIrq = DMA1_ST1_HANDLER,
        .rxIrq = UART7_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART7_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART7
    },
#endif

#ifdef USE_UART8
    {
        .device = UARTDEV_8,
        .reg = UART8,
        .DMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART8_RX_DMA
        .rxDMAStream = DMA1_Stream6,
#endif
#ifdef USE_UART8_TX_DMA
        .txDMAStream = DMA1_Stream0,
#endif
        .rxPins = { DEFIO_TAG_E(PE0), IO_TAG_NONE, IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PE1), IO_TAG_NONE, IO_TAG_NONE },
        .af = GPIO_AF8_UART8,
#ifdef UART8_AHB1_PERIPHERALS
        .rcc_ahb1 = UART8_AHB1_PERIPHERALS,
#endif
        .rcc_apb1 = RCC_APB1(UART8),
        .txIrq = DMA1_ST0_HANDLER,
        .rxIrq = UART8_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART8_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART8
    },
#endif
};

void uartIrqHandler(uartPort_t *s)
{
    UART_HandleTypeDef *huart = &s->Handle;
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET)) {
        uint8_t rbyte = (uint8_t)(huart->Instance->RDR & (uint8_t) 0xff);

        if (s->port.rxCallback) {
            s->port.rxCallback(rbyte, s->port.rxCallbackData);
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
    if ((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    }

    /* UART in mode Transmitter ------------------------------------------------*/
    if (!s->txDMAStream && (__HAL_UART_GET_IT(huart, UART_IT_TXE) != RESET)) {
        /* Check that a Tx process is ongoing */
        if (huart->gState != HAL_UART_STATE_BUSY_TX) {
            if (s->port.txBufferTail == s->port.txBufferHead) {
                huart->TxXferCount = 0;
                /* Disable the UART Transmit Data Register Empty Interrupt */
                CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
            } else {
                if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE)) {
                    huart->Instance->TDR = (((uint16_t) s->port.txBuffer[s->port.txBufferTail]) & (uint16_t) 0x01FFU);
                } else {
                    huart->Instance->TDR = (uint8_t)(s->port.txBuffer[s->port.txBufferTail]);
                }
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            }
        }
    }

    /* UART in mode Transmitter (transmission end) -----------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET)) {
        HAL_UART_IRQHandler(huart);
        if (s->txDMAStream) {
            handleUsartTxDma(s);
        }
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

// XXX Should serialUART be consolidated?

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uartdev = uartDevmap[device];
    if (!uartdev) {
        return NULL;
    }

    uartPort_t *s = &(uartdev->port);

    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uartdev->rxBuffer;
    s->port.txBuffer = uartdev->txBuffer;
    s->port.rxBufferSize = ARRAYLEN(uartdev->rxBuffer);
    s->port.txBufferSize = ARRAYLEN(uartdev->txBuffer);

    const uartHardware_t *hardware = uartdev->hardware;

    s->USARTx = hardware->reg;

    if (hardware->rxDMAStream) {
        s->rxDMAChannel = hardware->DMAChannel;
        s->rxDMAStream = hardware->rxDMAStream;
    }

    if (hardware->txDMAStream) {
        s->txDMAChannel = hardware->DMAChannel;
        s->txDMAStream = hardware->txDMAStream;

        // DMA TX Interrupt
        dmaInit(hardware->txIrq, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        dmaSetHandler(hardware->txIrq, dmaIRQHandler, hardware->txPriority, (uint32_t)uartdev);
    }

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->TDR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->RDR;

    s->Handle.Instance = hardware->reg;

    IO_t txIO = IOGetByTag(uartdev->tx);
    IO_t rxIO = IOGetByTag(uartdev->rx);

    if ((options & SERIAL_BIDIR) && txIO) {
        ioConfig_t ioCfg = IO_CONFIG(
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_MODE_AF_PP : GPIO_MODE_AF_OD,
            GPIO_SPEED_FREQ_HIGH,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_PULLDOWN : GPIO_PULLUP
        );

        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(txIO, ioCfg, hardware->af);
    }
    else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, hardware->af);
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP, hardware->af);
        }
    }

    if (!s->rxDMAChannel) {
        HAL_NVIC_SetPriority(hardware->rxIrq, NVIC_PRIORITY_BASE(hardware->rxPriority), NVIC_PRIORITY_SUB(hardware->rxPriority));
        HAL_NVIC_EnableIRQ(hardware->rxIrq);
    }

    return s;
}
#endif // USE_UART
