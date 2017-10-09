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

// Configuration constants

#if defined(STM32F1)
#define UARTDEV_COUNT_MAX 3
#define UARTHARDWARE_MAX_PINS 3
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     256
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     256
#endif
#elif defined(STM32F3)
#define UARTDEV_COUNT_MAX 5
#define UARTHARDWARE_MAX_PINS 4
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     256
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     256
#endif
#elif defined(STM32F4)
#define UARTDEV_COUNT_MAX 6
#define UARTHARDWARE_MAX_PINS 4
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     512
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     512
#endif
#elif defined(STM32F7)
#define UARTDEV_COUNT_MAX 8
#define UARTHARDWARE_MAX_PINS 3
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     512
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     512
#endif
#else
#error unknown MCU family
#endif

// Count number of configured UARTs

#ifdef USE_UART1
#define UARTDEV_COUNT_1 1
#else
#define UARTDEV_COUNT_1 0
#endif

#ifdef USE_UART2
#define UARTDEV_COUNT_2 1
#else
#define UARTDEV_COUNT_2 0
#endif

#ifdef USE_UART3
#define UARTDEV_COUNT_3 1
#else
#define UARTDEV_COUNT_3 0
#endif

#ifdef USE_UART4
#define UARTDEV_COUNT_4 1
#else
#define UARTDEV_COUNT_4 0
#endif

#ifdef USE_UART5
#define UARTDEV_COUNT_5 1
#else
#define UARTDEV_COUNT_5 0
#endif

#ifdef USE_UART6
#define UARTDEV_COUNT_6 1
#else
#define UARTDEV_COUNT_6 0
#endif

#ifdef USE_UART7
#define UARTDEV_COUNT_7 1
#else
#define UARTDEV_COUNT_7 0
#endif

#ifdef USE_UART8
#define UARTDEV_COUNT_8 1
#else
#define UARTDEV_COUNT_8 0
#endif

#define UARTDEV_COUNT (UARTDEV_COUNT_1 + UARTDEV_COUNT_2 + UARTDEV_COUNT_3 + UARTDEV_COUNT_4 + UARTDEV_COUNT_5 + UARTDEV_COUNT_6 + UARTDEV_COUNT_7 + UARTDEV_COUNT_8)

typedef struct uartHardware_s {
    UARTDevice_e device;    // XXX Not required for full allocation
    USART_TypeDef* reg;
#if defined(STM32F1) || defined(STM32F3)
    DMA_Channel_TypeDef *txDMAChannel;
    DMA_Channel_TypeDef *rxDMAChannel;
#elif defined(STM32F4) || defined(STM32F7)
    uint32_t DMAChannel;
    DMA_Stream_TypeDef *txDMAStream;
    DMA_Stream_TypeDef *rxDMAStream;
#endif
    ioTag_t rxPins[UARTHARDWARE_MAX_PINS];
    ioTag_t txPins[UARTHARDWARE_MAX_PINS];
#if defined(STM32F7)
    uint32_t rcc_ahb1;
    rccPeriphTag_t rcc_apb2;
    rccPeriphTag_t rcc_apb1;
#else
    rccPeriphTag_t rcc;
#endif
    uint8_t af;
#if defined(STM32F7)
    uint8_t txIrq;
    uint8_t rxIrq;
#else
    uint8_t irqn;
#endif
    uint8_t txPriority;
    uint8_t rxPriority;
} uartHardware_t;

extern const uartHardware_t uartHardware[];

// uartDevice_t is an actual device instance.
// XXX Instances are allocated for uarts defined by USE_UARTx atm.

typedef struct uartDevice_s {
    uartPort_t port;
    const uartHardware_t *hardware;
    ioTag_t rx;
    ioTag_t tx;
    volatile uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
    volatile uint8_t txBuffer[UART_TX_BUFFER_SIZE];
} uartDevice_t;

extern uartDevice_t *uartDevmap[];

extern const struct serialPortVTable uartVTable[];

#ifdef USE_HAL_DRIVER
void uartStartTxDMA(uartPort_t *s);
#else
void uartTryStartTxDMA(uartPort_t *s);
#endif

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options);

void uartIrqHandler(uartPort_t *s);

void uartReconfigure(uartPort_t *uartPort);
