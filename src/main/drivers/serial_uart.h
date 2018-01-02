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

#pragma once

// Since serial ports can be used for any function these buffer sizes should be equal
// The two largest things that need to be sent are: 1, MSP responses, 2, UBLOX SVINFO packet.

// Size must be a power of two due to various optimizations which use 'and' instead of 'mod'
// Various serial routines return the buffer occupied size as uint8_t which would need to be extended in order to
// increase size further.

#if defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6) || defined(USE_UART7) || defined(USE_UART8)
#define USE_UART
#endif

typedef enum {
    UARTDEV_1 = 0,
    UARTDEV_2 = 1,
    UARTDEV_3 = 2,
    UARTDEV_4 = 3,
    UARTDEV_5 = 4,
    UARTDEV_6 = 5,
    UARTDEV_7 = 6,
    UARTDEV_8 = 7
} UARTDevice_e;

typedef struct uartPort_s {
    serialPort_t port;

#if defined(STM32F7)
    DMA_HandleTypeDef rxDMAHandle;
    DMA_HandleTypeDef txDMAHandle;
#endif
#if defined(STM32F4) || defined(STM32F7)
    DMA_Stream_TypeDef *rxDMAStream;
    DMA_Stream_TypeDef *txDMAStream;
    uint32_t rxDMAChannel;
    uint32_t txDMAChannel;
#else
    DMA_Channel_TypeDef *rxDMAChannel;
    DMA_Channel_TypeDef *txDMAChannel;
#endif
    uint32_t rxDMAIrq;
    uint32_t txDMAIrq;

    uint32_t rxDMAPos;

    uint32_t txDMAPeripheralBaseAddr;
    uint32_t rxDMAPeripheralBaseAddr;

#ifdef USE_HAL_DRIVER
    // All USARTs can also be used as UART, and we use them only as UART.
    UART_HandleTypeDef Handle;
#endif
    USART_TypeDef *USARTx;
    bool txDMAEmpty;
} uartPort_t;

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig);
serialPort_t *uartOpen(UARTDevice_e device, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options);
