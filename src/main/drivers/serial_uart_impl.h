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

// Configuration constants

// TODO: this comment is obsolete

// Since serial ports can be used for any function these buffer sizes should be equal
// The two largest things that need to be sent are: 1, MSP responses, 2, UBLOX SVINFO packet.

// Size must be a power of two due to various optimizations which use 'and' instead of 'mod'
// Various serial routines return the buffer occupied size as uint8_t which would need to be extended in order to
// increase size further.

// define some common UART features
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(AT32F43x)
#define UART_TRAIT_AF_PIN 1        // pin AF mode is configured for each pin individually
#else
#define UART_TRAIT_AF_PORT 1       // all pins on given uart use same AF
#endif

#if !defined(STM32F4) || !defined(APM32F4) // all others support pinswap
#define UART_TRAIT_PINSWAP 1
#endif

#if defined(STM32F4)

#define UARTHARDWARE_MAX_PINS 4
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

#elif defined(STM32F7)

#define UARTHARDWARE_MAX_PINS 4
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

#elif defined(STM32H7)

#define UARTHARDWARE_MAX_PINS 5
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

#elif defined(STM32G4)

#define UARTHARDWARE_MAX_PINS 3
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

#elif defined(AT32F4)

#define UARTHARDWARE_MAX_PINS 5
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
#elif defined(APM32F4)

#define UARTHARDWARE_MAX_PINS 4
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

#else
#error unknown MCU family
#endif

// compressed index of UART/LPUART. Direct index into uartDevice[]
typedef enum {
    UARTDEV_INVALID = -1,
#ifdef USE_UART1
    UARTDEV_1,
#endif
#ifdef USE_UART2
    UARTDEV_2,
#endif
#ifdef USE_UART3
    UARTDEV_3,
#endif
#ifdef USE_UART4
    UARTDEV_4,
#endif
#ifdef USE_UART5
    UARTDEV_5,
#endif
#ifdef USE_UART6
    UARTDEV_6,
#endif
#ifdef USE_UART7
    UARTDEV_7,
#endif
#ifdef USE_UART8
    UARTDEV_8,
#endif
#ifdef USE_UART9
    UARTDEV_9,
#endif
#ifdef USE_UART10
    UARTDEV_10,
#endif
#ifdef USE_LPUART1
    UARTDEV_LP1,
#endif
    UARTDEV_COUNT
} uartDeviceIdx_e;

typedef struct uartPinDef_s {
    ioTag_t pin;
#if UART_TRAIT_AF_PIN
    uint8_t af;
#endif
} uartPinDef_t;

typedef struct uartHardware_s {
    serialPortIdentifier_e identifier;
    USART_TypeDef* reg;

#ifdef USE_DMA
    dmaResource_t *txDMAResource;
    dmaResource_t *rxDMAResource;
    // For H7 and G4  , {tx|rx}DMAChannel are DMAMUX input index for  peripherals (DMA_REQUEST_xxx); H7:RM0433 Table 110, G4:RM0440 Table 80.
    // For F4 and F7, these are 32-bit channel identifiers (DMA_CHANNEL_x)
    // For at32f435/7 DmaChannel is the dmamux, need to call dmamuxenable using dmamuxid
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(APM32F4)
    uint32_t txDMAChannel;
    uint32_t rxDMAChannel;
#elif defined(AT32F4)
    uint32_t txDMAMuxId;//for dmaspec->dmamux  and using dmaMuxEnable(dmax,muxid)
    uint32_t rxDMAMuxId;
#endif

#endif // USE_DMA

    uartPinDef_t rxPins[UARTHARDWARE_MAX_PINS];
    uartPinDef_t txPins[UARTHARDWARE_MAX_PINS];

    rccPeriphTag_t rcc;

#if UART_TRAIT_AF_PORT
    uint8_t af;
#endif

    uint8_t irqn;                  // uart IRQ. One shared IRQ per uart

    uint8_t txPriority;
    uint8_t rxPriority;

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
#if UART_TRAIT_PINSWAP
    bool pinSwap;
#endif
    txPinState_t txPinState;
} uartDevice_t;

extern uartDevice_t uartDevice[UARTDEV_COUNT];  // indexed by uartDeviceIdx_e;

uartDeviceIdx_e uartDeviceIdxFromIdentifier(serialPortIdentifier_e identifier);
uartDevice_t* uartDeviceFromIdentifier(serialPortIdentifier_e identifier);

extern const struct serialPortVTable uartVTable[];

void uartTryStartTxDMA(uartPort_t *s);

uartPort_t *serialUART(uartDevice_t *uart, uint32_t baudRate, portMode_e mode, portOptions_e options);

void uartConfigureExternalPinInversion(uartPort_t *uartPort);

void uartIrqHandler(uartPort_t *s);

void uartReconfigure(uartPort_t *uartPort);

void uartConfigureDma(uartDevice_t *uartdev);

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor);

bool checkUsartTxOutput(uartPort_t *s);
void uartTxMonitor(uartPort_t *s);

#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define UART_REG_RXD(base) ((base)->RDR)
#define UART_REG_TXD(base) ((base)->TDR)
#elif defined(STM32F4)
#define UART_REG_RXD(base) ((base)->DR)
#define UART_REG_TXD(base) ((base)->DR)
#elif defined(AT32F43x)
#define UART_REG_RXD(base) ((base)->dt)
#define UART_REG_TXD(base) ((base)->dt)
#elif defined(APM32F4)
#define UART_REG_RXD(base) ((base)->DATA)
#define UART_REG_TXD(base) ((base)->DATA)
#endif

#define UART_BUFFER(type, n, rxtx) type volatile uint8_t uart ## n ## rxtx ## xBuffer[UART_ ## rxtx ## X_BUFFER_SIZE]

#define UART_BUFFERS_EXTERN(n)                \
    UART_BUFFER(extern, n, R);                \
    UART_BUFFER(extern, n, T); struct dummy_s \
/**/

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
UART_BUFFERS_EXTERN(Lp1);
#endif

#undef UART_BUFFERS_EXTERN
