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
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"

#include <common/maths.h>
#include "common/utils.h"

#include "io/serial.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/rcc.h"
#include "drivers/serial.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#include "pg/serial_uart.h"

#if defined(STM32H7)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM            // D2 SRAM
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM            // D2 SRAM
#elif defined(STM32G4)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM_W          // SRAM MPU NOT_BUFFERABLE
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM_R          // SRAM MPU NOT CACHABLE
#elif defined(STM32F7)
#define UART_TX_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT // DTCM RAM
#define UART_RX_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT // DTCM RAM
#elif defined(STM32F4) || defined(AT32F4) || defined(APM32F4)
#define UART_TX_BUFFER_ATTRIBUTE                    // NONE
#define UART_RX_BUFFER_ATTRIBUTE                    // NONE
#else
#error Undefined UART_{TX,RX}_BUFFER_ATTRIBUTE for this MCU
#endif

#define UART_BUFFERS(n)                                         \
    UART_BUFFER(UART_TX_BUFFER_ATTRIBUTE, n, T);                \
    UART_BUFFER(UART_RX_BUFFER_ATTRIBUTE, n, R); struct dummy_s \
/**/

#ifdef USE_UART1
UART_BUFFERS(1);
#endif

#ifdef USE_UART2
UART_BUFFERS(2);
#endif

#ifdef USE_UART3
UART_BUFFERS(3);
#endif

#ifdef USE_UART4
UART_BUFFERS(4);
#endif

#ifdef USE_UART5
UART_BUFFERS(5);
#endif

#ifdef USE_UART6
UART_BUFFERS(6);
#endif

#ifdef USE_UART7
UART_BUFFERS(7);
#endif

#ifdef USE_UART8
UART_BUFFERS(8);
#endif

#ifdef USE_UART9
UART_BUFFERS(9);
#endif

#ifdef USE_UART10
UART_BUFFERS(10);
#endif

#ifdef USE_LPUART1
UART_BUFFERS(Lp1);  // TODO - maybe some other naming scheme ?
#endif

#undef UART_BUFFERS

// store only devices configured for target (USE_UARTx)
// some entries may be unused, for example because of pin configuration
// uartDeviceIdx_e is direct index into this table
FAST_DATA_ZERO_INIT uartDevice_t uartDevice[UARTDEV_COUNT];

// map serialPortIdentifier_e to uartDeviceIdx_e
uartDeviceIdx_e uartDeviceIdxFromIdentifier(serialPortIdentifier_e identifier)
{
#ifdef USE_LPUART1
    if (identifier == SERIAL_PORT_LPUART1) {
        return UARTDEV_LP1;
    }
#endif

#if 1 // TODO ...
    // store +1 in table - unset values default to 0
    // table is for UART only to save space (LPUART is handled separately)
#define _R(id, dev) [id] = (dev) + 1
    static const uartDeviceIdx_e uartMap[] = {
#ifdef USE_UART1
        _R(SERIAL_PORT_USART1, UARTDEV_1),
#endif
#ifdef USE_UART2
        _R(SERIAL_PORT_USART2, UARTDEV_2),
#endif
#ifdef USE_UART3
        _R(SERIAL_PORT_USART3, UARTDEV_3),
#endif
#ifdef USE_UART4
        _R(SERIAL_PORT_UART4, UARTDEV_4),
#endif
#ifdef USE_UART5
        _R(SERIAL_PORT_UART5, UARTDEV_5),
#endif
#ifdef USE_UART6
        _R(SERIAL_PORT_USART6, UARTDEV_6),
#endif
#ifdef USE_UART7
        _R(SERIAL_PORT_USART7, UARTDEV_7),
#endif
#ifdef USE_UART8
        _R(SERIAL_PORT_USART8, UARTDEV_8),
#endif
#ifdef USE_UART9
        _R(SERIAL_PORT_UART9, UARTDEV_9),
#endif
#ifdef USE_UART10
        _R(SERIAL_PORT_USART10, UARTDEV_10),
#endif
    };
#undef _R
    if (identifier >= 0 && identifier < (int)ARRAYLEN(uartMap)) {
        // UART case, but given USE_UARTx may not be defined
        return uartMap[identifier] ? uartMap[identifier] - 1 : UARTDEV_INVALID;
    }
#else
    {
        int idx = identifier - SERIAL_PORT_USART1;
        if (idx >= 0 && idx < SERIAL_UART_MAX) {
            if (BIT(idx) & SERIAL_UART_MASK) {
                // retrun number of enabled UART ports smaller than idx
                return popcount((BIT(idx) - 1) & SERIAL_UART_MASK);
            } else {
                return UARTDEV_INVALID;
            }
        }
    }
#endif
    // neither LPUART, nor UART
    return UARTDEV_INVALID;
}

uartDevice_t *uartDeviceFromIdentifier(serialPortIdentifier_e identifier)
{
    const uartDeviceIdx_e deviceIdx = uartDeviceIdxFromIdentifier(identifier);
    return deviceIdx != UARTDEV_INVALID ? &uartDevice[deviceIdx] : NULL;
}

serialPort_t *uartOpen(serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uartDevice = uartDeviceFromIdentifier(identifier);
    if (!uartDevice) {
        return NULL;
    }
    // fill identifier early, so initialization code can use it
    uartDevice->port.port.identifier = identifier;

    uartPort_t *uartPort = serialUART(uartDevice, baudRate, mode, options);
    if (!uartPort) {
        return NULL;
    }

    // common serial initialisation code should move to serialPort::init()
    uartPort->port.rxBufferHead = uartPort->port.rxBufferTail = 0;
    uartPort->port.txBufferHead = uartPort->port.txBufferTail = 0;
    // callback works for IRQ-based RX ONLY
    uartPort->port.rxCallback = rxCallback;
    uartPort->port.rxCallbackData = rxCallbackData;
    uartPort->port.mode = mode;
    uartPort->port.baudRate = baudRate;
    uartPort->port.options = options;

    uartReconfigure(uartPort);

    return (serialPort_t *)uartPort;
}

static void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.baudRate = baudRate;
    uartReconfigure(uartPort);
}

static void uartSetMode(serialPort_t *instance, portMode_e mode)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.mode = mode;
    uartReconfigure(uartPort);
}

static uint32_t uartTotalRxBytesWaiting(const serialPort_t *instance)
{
    const uartPort_t *uartPort = (const uartPort_t*)instance;

#ifdef USE_DMA
    if (uartPort->rxDMAResource) {
        // XXX Could be consolidated
#ifdef USE_HAL_DRIVER
        uint32_t rxDMAHead = __HAL_DMA_GET_COUNTER(uartPort->Handle.hdmarx);
#else
        uint32_t rxDMAHead = xDMA_GetCurrDataCounter(uartPort->rxDMAResource);
#endif

        // uartPort->rxDMAPos and rxDMAHead represent distances from the end
        // of the buffer.  They count DOWN as they advance.
        if (uartPort->rxDMAPos >= rxDMAHead) {
            return uartPort->rxDMAPos - rxDMAHead;
        } else {
            return uartPort->port.rxBufferSize + uartPort->rxDMAPos - rxDMAHead;
        }
    }
#endif

    if (uartPort->port.rxBufferHead >= uartPort->port.rxBufferTail) {
        return uartPort->port.rxBufferHead - uartPort->port.rxBufferTail;
    } else {
        return uartPort->port.rxBufferSize + uartPort->port.rxBufferHead - uartPort->port.rxBufferTail;
    }
}

static uint32_t uartTotalTxBytesFree(const serialPort_t *instance)
{
    const uartPort_t *uartPort = (const uartPort_t*)instance;

    uint32_t bytesUsed;

    if (uartPort->port.txBufferHead >= uartPort->port.txBufferTail) {
        bytesUsed = uartPort->port.txBufferHead - uartPort->port.txBufferTail;
    } else {
        bytesUsed = uartPort->port.txBufferSize + uartPort->port.txBufferHead - uartPort->port.txBufferTail;
    }

#ifdef USE_DMA
    if (uartPort->txDMAResource) {
        /*
         * When we queue up a DMA request, we advance the Tx buffer tail before the transfer finishes, so we must add
         * the remaining size of that in-progress transfer here instead:
         */
#ifdef USE_HAL_DRIVER
        bytesUsed += __HAL_DMA_GET_COUNTER(uartPort->Handle.hdmatx);
#else
        bytesUsed += xDMA_GetCurrDataCounter(uartPort->txDMAResource);
#endif

        /*
         * If the Tx buffer is being written to very quickly, we might have advanced the head into the buffer
         * space occupied by the current DMA transfer. In that case the "bytesUsed" total will actually end up larger
         * than the total Tx buffer size, because we'll end up transmitting the same buffer region twice. (So we'll be
         * transmitting a garbage mixture of old and new bytes).
         *
         * Be kind to callers and pretend like our buffer can only ever be 100% full.
         */
        if (bytesUsed >= uartPort->port.txBufferSize - 1) {
            return 0;
        }
    }
#endif

    return (uartPort->port.txBufferSize - 1) - bytesUsed;
}

static bool isUartTransmitBufferEmpty(const serialPort_t *instance)
{
    const uartPort_t *uartPort = (const uartPort_t *)instance;
#ifdef USE_DMA
    if (uartPort->txDMAResource) {
        return uartPort->txDMAEmpty;
    } else
#endif
    {
        return uartPort->port.txBufferTail == uartPort->port.txBufferHead;
    }
}

static uint8_t uartRead(serialPort_t *instance)
{
    uint8_t ch;
    uartPort_t *uartPort = (uartPort_t *)instance;

#ifdef USE_DMA
    if (uartPort->rxDMAResource) {
        ch = uartPort->port.rxBuffer[uartPort->port.rxBufferSize - uartPort->rxDMAPos];
        if (--uartPort->rxDMAPos == 0)
            uartPort->rxDMAPos = uartPort->port.rxBufferSize;
    } else
#endif
    {
        ch = uartPort->port.rxBuffer[uartPort->port.rxBufferTail];
        if (uartPort->port.rxBufferTail + 1 >= uartPort->port.rxBufferSize) {
            uartPort->port.rxBufferTail = 0;
        } else {
            uartPort->port.rxBufferTail++;
        }
    }

    return ch;
}

static void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *uartPort = (uartPort_t *)instance;

    // Check if the TX line is being pulled low by an unpowered peripheral
    if (uartPort->checkUsartTxOutput && !uartPort->checkUsartTxOutput(uartPort)) {
        // TX line is being pulled low, so don't transmit
        return;
    }

    uartPort->port.txBuffer[uartPort->port.txBufferHead] = ch;

    if (uartPort->port.txBufferHead + 1 >= uartPort->port.txBufferSize) {
        uartPort->port.txBufferHead = 0;
    } else {
        uartPort->port.txBufferHead++;
    }

#ifdef USE_DMA
    if (uartPort->txDMAResource) {
        uartTryStartTxDMA(uartPort);
    } else
#endif
    {
#if defined(USE_HAL_DRIVER)
        __HAL_UART_ENABLE_IT(&uartPort->Handle, UART_IT_TXE);
#elif defined(USE_ATBSP_DRIVER)
        usart_interrupt_enable(uartPort->USARTx, USART_TDBE_INT, TRUE);
#else
        USART_ITConfig(uartPort->USARTx, USART_IT_TXE, ENABLE);
#endif
    }
}

static void uartBeginWrite(serialPort_t *instance)
{
    uartPort_t *uartPort = (uartPort_t *)instance;

    // Check if the TX line is being pulled low by an unpowered peripheral
    if (uartPort->checkUsartTxOutput) {
        uartPort->checkUsartTxOutput(uartPort);
    }
}

static void uartWriteBuf(serialPort_t *instance, const void *data, int count)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartDevice_t *uart = container_of(uartPort, uartDevice_t, port);
    const uint8_t *bytePtr = (const uint8_t*)data;

    // Test if checkUsartTxOutput() detected TX line being pulled low by an unpowered peripheral
    if (uart->txPinState == TX_PIN_MONITOR) {
        // TX line is being pulled low, so don't transmit
        return;
    }

    while (count > 0) {
        // Calculate the available space to the end of the buffer
        const int spaceToEnd = uartPort->port.txBufferSize - uartPort->port.txBufferHead;
        // Determine the amount to copy in this iteration
        const int chunkSize = MIN(spaceToEnd, count);
        // Copy the chunk
        memcpy((void *)&uartPort->port.txBuffer[uartPort->port.txBufferHead], bytePtr, chunkSize);
        // Advance source pointer
        bytePtr += chunkSize;
        // Advance head, wrapping if necessary
        uartPort->port.txBufferHead = (uartPort->port.txBufferHead + chunkSize) % uartPort->port.txBufferSize;
        // Decrease remaining count
        count -= chunkSize;
    }
}

static void uartEndWrite(serialPort_t *instance)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartDevice_t *uart = container_of(uartPort, uartDevice_t, port);

    // Check if the TX line is being pulled low by an unpowered peripheral
    if (uart->txPinState == TX_PIN_MONITOR) {
        // TX line is being pulled low, so don't transmit
        return;
    }

#ifdef USE_DMA
    if (uartPort->txDMAResource) {
        uartTryStartTxDMA(uartPort);
    } else
#endif
    {
#if defined(USE_HAL_DRIVER)
        __HAL_UART_ENABLE_IT(&uartPort->Handle, UART_IT_TXE);
#elif defined(USE_ATBSP_DRIVER)
        usart_interrupt_enable(uartPort->USARTx, USART_TDBE_INT, TRUE);
#else
        USART_ITConfig(uartPort->USARTx, USART_IT_TXE, ENABLE);
#endif
    }
}

const struct serialPortVTable uartVTable[] = {
    {
        .serialWrite = uartWrite,
        .serialTotalRxWaiting = uartTotalRxBytesWaiting,
        .serialTotalTxFree = uartTotalTxBytesFree,
        .serialRead = uartRead,
        .serialSetBaudRate = uartSetBaudRate,
        .isSerialTransmitBufferEmpty = isUartTransmitBufferEmpty,
        .setMode = uartSetMode,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = uartWriteBuf,
        .beginWrite = uartBeginWrite,
        .endWrite = uartEndWrite,
    }
};

// TODO - move to serial_uart_hw.c
#ifdef USE_DMA
void uartConfigureDma(uartDevice_t *uartdev)
{
    uartPort_t *uartPort = &(uartdev->port);
    const uartHardware_t *hardware = uartdev->hardware;

#ifdef USE_DMA_SPEC
    const serialPortIdentifier_e uartPortIdentifier = hardware->identifier;
    const uartDeviceIdx_e uartDeviceIdx = uartDeviceIdxFromIdentifier(uartPortIdentifier);
    if (uartDeviceIdx == UARTDEV_INVALID) {
        return;
    }
    const int resourceIdx = serialResourceIndex(uartPortIdentifier);
    const int ownerIndex = serialOwnerIndex(uartPortIdentifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(uartPortIdentifier); // rx is always +1

    const dmaChannelSpec_t *dmaChannelSpec;

    if (serialUartConfig(resourceIdx)->txDmaopt != DMA_OPT_UNUSED) {
        dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, uartDeviceIdx, serialUartConfig(resourceIdx)->txDmaopt);
        if (dmaChannelSpec) {
            uartPort->txDMAResource = dmaChannelSpec->ref;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(APM32F4)
            uartPort->txDMAChannel = dmaChannelSpec->channel;
#elif defined(AT32F4)
            uartPort->txDMAMuxId = dmaChannelSpec->dmaMuxId;
#endif
        }
    }

    if (serialUartConfig(resourceIdx)->rxDmaopt != DMA_OPT_UNUSED) {
        dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, uartDeviceIdx, serialUartConfig(resourceIdx)->txDmaopt);
        if (dmaChannelSpec) {
            uartPort->rxDMAResource = dmaChannelSpec->ref;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(APM32F4)
            uartPort->rxDMAChannel = dmaChannelSpec->channel;
#elif defined(AT32F4)
            uartPort->rxDMAMuxId = dmaChannelSpec->dmaMuxId;
#endif
        }
    }
#else /* USE_DMA_SPEC */
    // Non USE_DMA_SPEC does not support configurable ON/OFF of UART DMA

    if (hardware->rxDMAResource) {
        uartPort->rxDMAResource = hardware->rxDMAResource;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(APM32F4)
        uartPort->rxDMAChannel = hardware->rxDMAChannel;
#elif defined(AT32F4)
        uartPort->rxDMAMuxId = hardware->rxDMAMuxId;
#endif
    }

    if (hardware->txDMAResource) {
        uartPort->txDMAResource = hardware->txDMAResource;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(APM32F4)
        uartPort->txDMAChannel = hardware->txDMAChannel;
#elif defined(AT32F4)
        uartPort->txDMAMuxId = hardware->txDMAMuxId;
#endif
    }
#endif /* USE_DMA_SPEC */

    if (uartPort->txDMAResource) {
        const dmaIdentifier_e identifier = dmaGetIdentifier(uartPort->txDMAResource);
        if (dmaAllocate(identifier, ownerTxRx, ownerIndex)) {
            dmaEnable(identifier);
#if defined(AT32F4)
            dmaMuxEnable(identifier, uartPort->txDMAMuxId);
#endif
            dmaSetHandler(identifier, uartDmaIrqHandler, hardware->txPriority, (uint32_t)uartdev);
            uartPort->txDMAPeripheralBaseAddr = (uint32_t)&UART_REG_TXD(hardware->reg);
        }
    }

    if (uartPort->rxDMAResource) {
        const dmaIdentifier_e identifier = dmaGetIdentifier(uartPort->rxDMAResource);
        if (dmaAllocate(identifier, ownerTxRx + 1, ownerIndex)) {
            dmaEnable(identifier);
#if defined(AT32F4)
            dmaMuxEnable(identifier, uartPort->rxDMAMuxId);
#endif
            uartPort->rxDMAPeripheralBaseAddr = (uint32_t)&UART_REG_RXD(hardware->reg);
        }
    }
}
#endif

#define UART_IRQHandler(type, number, dev)                      \
    FAST_IRQ_HANDLER void type ## number ## _IRQHandler(void)   \
    {                                                           \
        uartPort_t *uartPort = &(uartDevice[(dev)].port);       \
        uartIrqHandler(uartPort);                               \
    }                                                           \
/**/

#ifdef USE_UART1
UART_IRQHandler(USART, 1, UARTDEV_1) // USART1 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART2
UART_IRQHandler(USART, 2, UARTDEV_2) // USART2 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART3
UART_IRQHandler(USART, 3, UARTDEV_3) // USART3 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART4
UART_IRQHandler(UART, 4, UARTDEV_4)  // UART4 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART5
UART_IRQHandler(UART, 5, UARTDEV_5)  // UART5 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART6
UART_IRQHandler(USART, 6, UARTDEV_6) // USART6 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART7
UART_IRQHandler(UART, 7, UARTDEV_7)  // UART7 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART8
UART_IRQHandler(UART, 8, UARTDEV_8)  // UART8 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART9
UART_IRQHandler(UART, 9, UARTDEV_9)  // UART9 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART10
UART_IRQHandler(UART, 10, UARTDEV_10) // UART10 Rx/Tx IRQ Handler
#endif

#ifdef USE_LPUART1
UART_IRQHandler(LPUART, 1, UARTDEV_LP1) // LPUART1 Rx/Tx IRQ Handler
#endif


#endif // USE_UART
