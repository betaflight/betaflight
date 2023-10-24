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

#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#include "stm32f7xx_ll_usart.h"

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .reg = USART1,
        .rxDMAChannel = DMA_CHANNEL_4,
        .txDMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART1_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA2_Stream5,
#endif
#ifdef USE_UART1_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA2_Stream7,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA10), GPIO_AF7_USART1 },
            { DEFIO_TAG_E(PB7), GPIO_AF7_USART1 },
#ifdef STM32F765xx
            { DEFIO_TAG_E(PB15), GPIO_AF4_USART1 }
#endif
        },
        .txPins = {
            { DEFIO_TAG_E(PA9), GPIO_AF7_USART1 },
            { DEFIO_TAG_E(PB6), GPIO_AF7_USART1 },
#ifdef STM32F765xx
            { DEFIO_TAG_E(PB14), GPIO_AF4_USART1 }
#endif
        },
        .rcc = RCC_APB2(USART1),
        .rxIrq = USART1_IRQn,
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
        .device = UARTDEV_2,
        .reg = USART2,
        .rxDMAChannel = DMA_CHANNEL_4,
        .txDMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART2_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA1_Stream5,
#endif
#ifdef USE_UART2_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA1_Stream6,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA3), GPIO_AF7_USART2 },
            { DEFIO_TAG_E(PD6), GPIO_AF7_USART2 }
        },
        .txPins = {
            { DEFIO_TAG_E(PA2), GPIO_AF7_USART2 },
            { DEFIO_TAG_E(PD5), GPIO_AF7_USART2 }
        },
        .rcc = RCC_APB1(USART2),
        .rxIrq = USART2_IRQn,
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
        .device = UARTDEV_3,
        .reg = USART3,
        .rxDMAChannel = DMA_CHANNEL_4,
        .txDMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART3_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA1_Stream1,
#endif
#ifdef USE_UART3_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA1_Stream3,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PB11), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PC11), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PD9), GPIO_AF7_USART3 }
        },
        .txPins = {
            { DEFIO_TAG_E(PB10), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PC10), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PD8), GPIO_AF7_USART3 }
        },
        .rcc = RCC_APB1(USART3),
        .rxIrq = USART3_IRQn,
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
        .device = UARTDEV_4,
        .reg = UART4,
        .rxDMAChannel = DMA_CHANNEL_4,
        .txDMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART4_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA1_Stream2,
#endif
#ifdef USE_UART4_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA1_Stream4,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA1), GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PC11), GPIO_AF8_UART4 },
#ifdef STM32F765xx
            { DEFIO_TAG_E(PA11), GPIO_AF6_UART4 },
            { DEFIO_TAG_E(PD0), GPIO_AF8_UART4 }
#endif
        },
        .txPins = {
            { DEFIO_TAG_E(PA0), GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PC10), GPIO_AF8_UART4 },
#ifdef STM32F765xx
            { DEFIO_TAG_E(PA12), GPIO_AF6_UART4 },
            { DEFIO_TAG_E(PD1), GPIO_AF8_UART4 }
#endif
        },
        .rcc = RCC_APB1(UART4),
        .rxIrq = UART4_IRQn,
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
        .device = UARTDEV_5,
        .reg = UART5,
        .rxDMAChannel = DMA_CHANNEL_4,
        .txDMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART5_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA1_Stream0,
#endif
#ifdef USE_UART5_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA1_Stream7,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PD2), GPIO_AF8_UART5 },
#ifdef STM32F765xx
            { DEFIO_TAG_E(PB5), GPIO_AF1_UART5 },
            { DEFIO_TAG_E(PB8), GPIO_AF7_UART5 },
            { DEFIO_TAG_E(PB12), GPIO_AF8_UART5 }
#endif
        },
        .txPins = {
            { DEFIO_TAG_E(PC12), GPIO_AF8_UART5 },
#ifdef STM32F765xx
            { DEFIO_TAG_E(PB6), GPIO_AF1_UART5 },
            { DEFIO_TAG_E(PB9), GPIO_AF7_UART5 },
            { DEFIO_TAG_E(PB13), GPIO_AF8_UART5 }
#endif
        },
        .rcc = RCC_APB1(UART5),
        .rxIrq = UART5_IRQn,
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
        .device = UARTDEV_6,
        .reg = USART6,
        .rxDMAChannel = DMA_CHANNEL_5,
        .txDMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART6_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA2_Stream1,
#endif
#ifdef USE_UART6_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA2_Stream6,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PC7), GPIO_AF8_USART6  },
            { DEFIO_TAG_E(PG9), GPIO_AF8_USART6 }
        },
        .txPins = {
            { DEFIO_TAG_E(PC6), GPIO_AF8_USART6 },
            { DEFIO_TAG_E(PG14), GPIO_AF8_USART6 }
        },
        .rcc = RCC_APB2(USART6),
        .rxIrq = USART6_IRQn,
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
        .device = UARTDEV_7,
        .reg = UART7,
        .rxDMAChannel = DMA_CHANNEL_5,
        .txDMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART7_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA1_Stream3,
#endif
#ifdef USE_UART7_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA1_Stream1,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PE7), GPIO_AF8_UART7 },
            { DEFIO_TAG_E(PF6), GPIO_AF8_UART7 },
#ifdef STM32F765xx
            { DEFIO_TAG_E(PA8), GPIO_AF12_UART7 },
            { DEFIO_TAG_E(PB3), GPIO_AF12_UART7 }
#endif
        },
        .txPins = {
            { DEFIO_TAG_E(PE8), GPIO_AF8_UART7 },
            { DEFIO_TAG_E(PF7), GPIO_AF8_UART7 },
#ifdef STM32F765xx
            { DEFIO_TAG_E(PA15), GPIO_AF12_UART7 },
            { DEFIO_TAG_E(PB4), GPIO_AF12_UART7 }
#endif
        },
        .rcc = RCC_APB1(UART7),
        .rxIrq = UART7_IRQn,
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
        .device = UARTDEV_8,
        .reg = UART8,
        .rxDMAChannel = DMA_CHANNEL_5,
        .txDMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART8_RX_DMA
        .rxDMAResource = (dmaResource_t *)DMA1_Stream6,
#endif
#ifdef USE_UART8_TX_DMA
        .txDMAResource = (dmaResource_t *)DMA1_Stream0,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PE0), GPIO_AF8_UART8 }
        },
        .txPins = {
            { DEFIO_TAG_E(PE1), GPIO_AF8_UART8 }
        },
        .rcc = RCC_APB1(UART8),
        .rxIrq = UART8_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART8_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART8,
        .txBuffer = uart8TxBuffer,
        .rxBuffer = uart8RxBuffer,
        .txBufferSize = sizeof(uart8TxBuffer),
        .rxBufferSize = sizeof(uart8RxBuffer),
    },
#endif
};

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

    const uartHardware_t *hardware = uartdev->hardware;

    s->USARTx = hardware->reg;

    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;

    s->checkUsartTxOutput = checkUsartTxOutput;

#ifdef USE_DMA
    uartConfigureDma(uartdev);
#endif

    s->Handle.Instance = hardware->reg;

    if (hardware->rcc) {
        RCC_ClockCmd(hardware->rcc, ENABLE);
    }

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);

    uartdev->txPinState = TX_PIN_IGNORE;

    if ((options & SERIAL_BIDIR) && txIO) {
        ioConfig_t ioCfg = IO_CONFIG(
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP) || (options & SERIAL_BIDIR_PP_PD)) ? GPIO_MODE_AF_PP : GPIO_MODE_AF_OD,
            GPIO_SPEED_FREQ_HIGH,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP_PD)) ? GPIO_PULLDOWN : GPIO_PULLUP
        );

        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(txIO, ioCfg, uartdev->tx.af);
    }
    else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));

            if (options & SERIAL_CHECK_TX) {
                uartdev->txPinState = TX_PIN_MONITOR;
                // Switch TX to UART output whilst UART sends idle preamble
                checkUsartTxOutput(s);
            } else {
                IOConfigGPIOAF(txIO, IOCFG_AF_PP, uartdev->tx.af);
            }
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP, uartdev->rx.af);
        }
    }

#ifdef USE_DMA
    if (!s->rxDMAResource) {
        HAL_NVIC_SetPriority(hardware->rxIrq, NVIC_PRIORITY_BASE(hardware->rxPriority), NVIC_PRIORITY_SUB(hardware->rxPriority));
        HAL_NVIC_EnableIRQ(hardware->rxIrq);
    }
#endif

    return s;
}
#endif // USE_UART
