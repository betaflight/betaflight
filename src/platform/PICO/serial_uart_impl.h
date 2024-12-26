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

#pragma once

// Configuration constants

#define UART_TX_BUFFER_ATTRIBUTE                    // NONE
#define UART_RX_BUFFER_ATTRIBUTE                    // NONE

#define UARTHARDWARE_MAX_PINS   3
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     256
#endif

#ifndef UART_TX_BUFFER_SIZE
#ifdef USE_MSP_DISPLAYPORT
#define UART_TX_BUFFER_SIZE     1280
#else
#define UART_TX_BUFFER_SIZE     256
#endif
#endif

// compressed index of UART/LPUART. Direct index into uartDevice[]
typedef enum {
    UARTDEV_INVALID = -1,
#ifdef USE_UART0
    UARTDEV_0,
#endif
#ifdef USE_UART1
    UARTDEV_1,
#endif
    UARTDEV_COUNT
} uartDeviceIdx_e;

typedef struct uartPinDef_s {
    ioTag_t pin;
} uartPinDef_t;

typedef struct uartHardware_s {
    serialPortIdentifier_e identifier;
    uart_inst_t* reg;
    uartPinDef_t rxPins[UARTHARDWARE_MAX_PINS];
    uartPinDef_t txPins[UARTHARDWARE_MAX_PINS];

    uint8_t irqn;

    volatile uint8_t *txBuffer;
    volatile uint8_t *rxBuffer;
    uint16_t txBufferSize;
    uint16_t rxBufferSize;
} uartHardware_t;

extern const uartHardware_t uartHardware[UARTDEV_COUNT];

// uartDevice_t is an actual device instance.
// XXX Instances are allocated for uarts defined by USE_UARTx atm.

typedef enum {
    TX_PIN_ACTIVE,
    TX_PIN_MONITOR,
    TX_PIN_IGNORE
} txPinState_t;

// TODO: merge uartPort_t and uartDevice_t
typedef struct uartDevice_s {
    uartPort_t port;
    const uartHardware_t *hardware;
    uartPinDef_t rx;
    uartPinDef_t tx;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    txPinState_t txPinState;
} uartDevice_t;

extern uartDevice_t uartDevice[UARTDEV_COUNT];  // indexed by uartDeviceIdx_e;

uartDeviceIdx_e uartDeviceIdxFromIdentifier(serialPortIdentifier_e identifier);
uartDevice_t* uartDeviceFromIdentifier(serialPortIdentifier_e identifier);

extern const struct serialPortVTable uartVTable[];

uartPort_t *serialUART(uartDevice_t *uart, uint32_t baudRate, portMode_e mode, portOptions_e options);

void uartConfigureExternalPinInversion(uartPort_t *uartPort);
void uartReconfigure(uartPort_t *uartPort);
void uartConfigureDma(uartDevice_t *uartdev);
void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor);
bool checkUsartTxOutput(uartPort_t *s);
void uartTxMonitor(uartPort_t *s);

#define UART_BUFFER(type, n, rxtx) type volatile uint8_t uart ## n ## rxtx ## xBuffer[UART_ ## rxtx ## X_BUFFER_SIZE]

#define UART_BUFFERS_EXTERN(n)                \
    UART_BUFFER(extern, n, R);                \
    UART_BUFFER(extern, n, T); struct dummy_s \
/**/

#ifdef USE_UART0
UART_BUFFERS_EXTERN(0);
#endif

#ifdef USE_UART1
UART_BUFFERS_EXTERN(1);
#endif

#undef UART_BUFFERS_EXTERN
