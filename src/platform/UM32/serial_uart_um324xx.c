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
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"



// UM32 UARTs use interrupts. Leave the hardware DMA resources unset.
const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_UART1,
        .reg = (usartResource_t *)UART1,
        .rxPins = { { DEFIO_TAG_E(PA10) }, { DEFIO_TAG_E(PB7) },},
        .txPins = { { DEFIO_TAG_E(PA9) }, { DEFIO_TAG_E(PB6) },},
        .af = GPIO_AF7_UART1,
        .rcc = RCC_APB2(UART1),
        .irqn = UART1_IRQn,
        .rxPriority = NVIC_PRIO_SERIALUART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif

#ifdef USE_UART2
    {
        .identifier = SERIAL_PORT_UART2,
        .reg = (usartResource_t *)UART2,
        .rxPins = { { DEFIO_TAG_E(PA3) }, { DEFIO_TAG_E(PD6) } },
        .txPins = { { DEFIO_TAG_E(PA2) }, { DEFIO_TAG_E(PD5) } },
        .af = GPIO_AF7_UART2,
        .rcc = RCC_APB0(UART2),
        .irqn = UART2_IRQn,
        .rxPriority = NVIC_PRIO_SERIALUART2,
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif

#ifdef USE_UART3
    {
        .identifier = SERIAL_PORT_UART3,
        .reg = (usartResource_t *)UART3,
        .rxPins = { { DEFIO_TAG_E(PB11) }, { DEFIO_TAG_E(PC11) }, { DEFIO_TAG_E(PD9) } },
        .txPins = { { DEFIO_TAG_E(PB10) }, { DEFIO_TAG_E(PC10) }, { DEFIO_TAG_E(PD8) } },
        .af = GPIO_AF7_UART3,
        .rcc = RCC_APB1(UART3),
        .irqn = UART3_IRQn,
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
        .reg = (usartResource_t *)UART4,
        .rxPins = { { DEFIO_TAG_E(PA1) }, { DEFIO_TAG_E(PC11) } },
        .txPins = { { DEFIO_TAG_E(PA0) }, { DEFIO_TAG_E(PC10) } },
        .af = GPIO_AF8_UART4,
        .rcc = RCC_APB1(UART4),
        .irqn = UART4_IRQn,
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
        .reg = (usartResource_t *)UART5,
        .rxPins = { { DEFIO_TAG_E(PD2) } },
        .txPins = { { DEFIO_TAG_E(PC12) } },
        .af = GPIO_AF8_UART5,
        .rcc = RCC_APB1(UART5),
        .irqn = UART5_IRQn,
        .rxPriority = NVIC_PRIO_SERIALUART5,
        .txBuffer = uart5TxBuffer,
        .rxBuffer = uart5RxBuffer,
        .txBufferSize = sizeof(uart5TxBuffer),
        .rxBufferSize = sizeof(uart5RxBuffer),
    },
#endif

#ifdef USE_UART6
    {
        .identifier = SERIAL_PORT_UART6,
        .reg = (usartResource_t *)UART6,
        .rxPins = { { DEFIO_TAG_E(PC7) },},
        .txPins = { { DEFIO_TAG_E(PC6) },},
        .af = GPIO_AF8_UART6,
        .rcc = RCC_APB1(UART6),
        .irqn = UART6_IRQn,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif

#ifdef USE_UART7
    {
        // USART7/8 are a different IP from UART1-6: SAM-style USART registers
        // (CR/MR/IER/IDR/IMR/CSR/RHR/THR/BRGR), not the 16550-style UART_EX.
        // serial_uart_hal.c branches on these instances for init/IRQ handling.
        // The SDK HAL selects the clock per bridge (USART7/PCLK0, USART8/PCLK2).
        // Note UART8 (PC6/PC7, AF9) shares its pins with UART6 (AF8).
        .identifier = SERIAL_PORT_USART7,
        .reg = (usartResource_t *)USART7,
        .rxPins = { { DEFIO_TAG_E(PC3) } },
        .txPins = { { DEFIO_TAG_E(PC2) } },
        .af = GPIO_AF9_USART7,
        .rcc = RCC_APB0(USART7),
        .irqn = USART7_IRQn,
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
        .rxPins = { { DEFIO_TAG_E(PC7) } },
        .txPins = { { DEFIO_TAG_E(PC6) } },
        .af = GPIO_AF9_USART8,
        .rcc = RCC_APB2(USART8),
        .irqn = USART8_IRQn,
        .rxPriority = NVIC_PRIO_SERIALUART8,
        .txBuffer = uart8TxBuffer,
        .rxBuffer = uart8RxBuffer,
        .txBufferSize = sizeof(uart8TxBuffer),
        .rxBufferSize = sizeof(uart8RxBuffer),
    },
#endif
};

#endif // USE_UART
