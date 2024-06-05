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

#pragma once

#include "serial_uart_impl_mcu.h"

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

#ifdef USE_UART9
#define UARTDEV_COUNT_9 1
#else
#define UARTDEV_COUNT_9 0
#endif

#ifdef USE_UART10
#define UARTDEV_COUNT_10 1
#else
#define UARTDEV_COUNT_10 0
#endif

#ifdef USE_LPUART1
#define LPUARTDEV_COUNT_1 1
#else
#define LPUARTDEV_COUNT_1 0
#endif

#define UARTDEV_COUNT (UARTDEV_COUNT_1 + UARTDEV_COUNT_2 + UARTDEV_COUNT_3 + UARTDEV_COUNT_4 + UARTDEV_COUNT_5 + UARTDEV_COUNT_6 + UARTDEV_COUNT_7 + UARTDEV_COUNT_8 + UARTDEV_COUNT_9 + UARTDEV_COUNT_10 + LPUARTDEV_COUNT_1)

typedef struct uartPinDef_s {
    ioTag_t pin;
#if defined(UART_USES_PIN_AF)
    uint8_t af;
#endif
} uartPinDef_t;

typedef struct uartHardware_s {
    UARTDevice_e device;    // XXX Not required for full allocation
    USART_TypeDef* reg;

#ifdef USE_DMA
    dmaResource_t *txDMAResource;
    dmaResource_t *rxDMAResource;
#if defined(UART_USES_DMAMUX)
    uint32_t txDMAMuxId;
    uint32_t rxDMAMuxId;
#else
    uint32_t txDMAChannel;
    uint32_t rxDMAChannel;
#endif

#endif // USE_DMA

    uartPinDef_t rxPins[UARTHARDWARE_MAX_PINS];
    uartPinDef_t txPins[UARTHARDWARE_MAX_PINS];

    rccPeriphTag_t rcc;

#if defined(UART_USES_PIN_AF)
    uint8_t af;
#endif

#if defined(UART_USES_TXRX_IRQ)
    uint8_t txIrq;
    uint8_t rxIrq;
#else
    uint8_t irqn;
#endif
    uint8_t txPriority;
    uint8_t rxPriority;

    volatile uint8_t *txBuffer;
    volatile uint8_t *rxBuffer;
    uint16_t txBufferSize;
    uint16_t rxBufferSize;
} uartHardware_t;

extern const uartHardware_t uartHardware[];

// uartDevice_t is an actual device instance.
// XXX Instances are allocated for uarts defined by USE_UARTx atm.

typedef enum {
    TX_PIN_ACTIVE,
    TX_PIN_MONITOR,
    TX_PIN_IGNORE
} txPinState_t;

typedef struct uartDevice_s {
    uartPort_t port;
    const uartHardware_t *hardware;
    uartPinDef_t rx;
    uartPinDef_t tx;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
#if defined(UART_PINSWAP_CAPABLE)
    bool pinSwap;
#endif
    txPinState_t txPinState;
} uartDevice_t;

extern uartDevice_t *uartDevmap[];

extern const struct serialPortVTable uartVTable[];

void uartTryStartTxDMA(uartPort_t *s);

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options);

void uartIrqHandler(uartPort_t *s);

void uartReconfigure(uartPort_t *uartPort);

void uartConfigureDma(uartDevice_t *uartdev);

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor);

bool checkUsartTxOutput(uartPort_t *s);
void uartTxMonitor(uartPort_t *s);

#define UART_BUFFER(type, n, rxtx) type volatile uint8_t uart ## n ## rxtx ## xBuffer[UART_ ## rxtx ## X_BUFFER_SIZE]

#define UART_BUFFERS_EXTERN(n) \
    UART_BUFFER(extern, n, R); \
    UART_BUFFER(extern, n, T); struct dummy_s

#define LPUART_BUFFER(type, n, rxtx) type volatile uint8_t lpuart ## n ## rxtx ## xBuffer[UART_ ## rxtx ## X_BUFFER_SIZE]

#define LPUART_BUFFERS_EXTERN(n) \
    LPUART_BUFFER(extern, n, R); \
    LPUART_BUFFER(extern, n, T); struct dummy_s

#ifdef USE_UART1
UART_BUFFERS_EXTERN(1);
#endif

#ifdef USE_UART2
UART_BUFFERS_EXTERN(2);
#endif

#ifdef USE_UART3
UART_BUFFERS_EXTERN(3);
#endif

#ifdef USE_UART4
UART_BUFFERS_EXTERN(4);
#endif

#ifdef USE_UART5
UART_BUFFERS_EXTERN(5);
#endif

#ifdef USE_UART6
UART_BUFFERS_EXTERN(6);
#endif

#ifdef USE_UART7
UART_BUFFERS_EXTERN(7);
#endif

#ifdef USE_UART8
UART_BUFFERS_EXTERN(8);
#endif

#ifdef USE_UART9
UART_BUFFERS_EXTERN(9);
#endif

#ifdef USE_UART10
UART_BUFFERS_EXTERN(10);
#endif

#ifdef USE_LPUART1
LPUART_BUFFERS_EXTERN(1);
#endif

#undef UART_BUFFERS_EXTERN
