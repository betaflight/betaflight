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
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_UART

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/inverter.h"
#include "drivers/dma.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

static void usartConfigurePinInversion(uartPort_t *uartPort)
{
#if !defined(USE_INVERTER)
    UNUSED(uartPort);
#else
    bool inverted = uartPort->port.options & SERIAL_INVERTED;

#ifdef USE_INVERTER
    if (inverted) {
        // Enable hardware inverter if available.
        enableInverter(uartPort->USARTx, true);
    }
#endif
#endif
}

// XXX uartReconfigure does not handle resource management properly.

void uartReconfigure(uartPort_t *uartPort)
{
    DAL_UART_DeInit(&uartPort->Handle);
    uartPort->Handle.Init.BaudRate = uartPort->port.baudRate;
    // according to the stm32 documentation wordlen has to be 9 for parity bits
    // this does not seem to matter for rx but will give bad data on tx!
    uartPort->Handle.Init.WordLength = (uartPort->port.options & SERIAL_PARITY_EVEN) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
    uartPort->Handle.Init.StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_STOPBITS_2 : USART_STOPBITS_1;
    uartPort->Handle.Init.Parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_PARITY_EVEN : USART_PARITY_NONE;
    uartPort->Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartPort->Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    uartPort->Handle.Init.Mode = 0;

    if (uartPort->port.mode & MODE_RX)
        uartPort->Handle.Init.Mode |= UART_MODE_RX;
    if (uartPort->port.mode & MODE_TX)
        uartPort->Handle.Init.Mode |= UART_MODE_TX;


    usartConfigurePinInversion(uartPort);

#ifdef TARGET_USART_CONFIG
    void usartTargetConfigure(uartPort_t *);
    usartTargetConfigure(uartPort);
#endif

    if (uartPort->port.options & SERIAL_BIDIR)
    {
        DAL_HalfDuplex_Init(&uartPort->Handle);
    }
    else
    {
        DAL_UART_Init(&uartPort->Handle);
    }

    // Receive DMA or IRQ
    if (uartPort->port.mode & MODE_RX)
    {
#ifdef USE_DMA
        if (uartPort->rxDMAResource)
        {
            uartPort->rxDMAHandle.Instance = (DMA_ARCH_TYPE *)uartPort->rxDMAResource;
            uartPort->txDMAHandle.Init.Channel = uartPort->rxDMAChannel;
            uartPort->rxDMAHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
            uartPort->rxDMAHandle.Init.PeriphInc = DMA_PINC_DISABLE;
            uartPort->rxDMAHandle.Init.MemInc = DMA_MINC_ENABLE;
            uartPort->rxDMAHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            uartPort->rxDMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            uartPort->rxDMAHandle.Init.Mode = DMA_CIRCULAR;
#if defined(APM32F4)
            uartPort->rxDMAHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            uartPort->rxDMAHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            uartPort->rxDMAHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            uartPort->rxDMAHandle.Init.MemBurst = DMA_MBURST_SINGLE;
#endif
            uartPort->rxDMAHandle.Init.Priority = DMA_PRIORITY_MEDIUM;


            DAL_DMA_DeInit(&uartPort->rxDMAHandle);
            DAL_DMA_Init(&uartPort->rxDMAHandle);
            /* Associate the initialized DMA handle to the UART handle */
            __DAL_LINKDMA(&uartPort->Handle, hdmarx, uartPort->rxDMAHandle);

            DAL_UART_Receive_DMA(&uartPort->Handle, (uint8_t*)uartPort->port.rxBuffer, uartPort->port.rxBufferSize);

            uartPort->rxDMAPos = __DAL_DMA_GET_COUNTER(&uartPort->rxDMAHandle);
        } else
#endif
        {
            /* Enable the UART Parity Error Interrupt */
            SET_BIT(uartPort->USARTx->CTRL1, USART_CTRL1_PEIEN);

            /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            SET_BIT(uartPort->USARTx->CTRL3, USART_CTRL3_ERRIEN);

            /* Enable the UART Data Register not empty Interrupt */
            SET_BIT(uartPort->USARTx->CTRL1, USART_CTRL1_RXBNEIEN);

            /* Enable Idle Line detection */
            SET_BIT(uartPort->USARTx->CTRL1, USART_CTRL1_IDLEIEN);
        }
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
#ifdef USE_DMA
        if (uartPort->txDMAResource) {
            uartPort->txDMAHandle.Instance = (DMA_ARCH_TYPE *)uartPort->txDMAResource;
            uartPort->txDMAHandle.Init.Channel = uartPort->txDMAChannel;
            uartPort->txDMAHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
            uartPort->txDMAHandle.Init.PeriphInc = DMA_PINC_DISABLE;
            uartPort->txDMAHandle.Init.MemInc = DMA_MINC_ENABLE;
            uartPort->txDMAHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            uartPort->txDMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            uartPort->txDMAHandle.Init.Mode = DMA_NORMAL;
            uartPort->txDMAHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            uartPort->txDMAHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            uartPort->txDMAHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            uartPort->txDMAHandle.Init.MemBurst = DMA_MBURST_SINGLE;
            uartPort->txDMAHandle.Init.Priority = DMA_PRIORITY_MEDIUM;


            DAL_DMA_DeInit(&uartPort->txDMAHandle);
            DAL_StatusTypeDef status = DAL_DMA_Init(&uartPort->txDMAHandle);
            if (status != DAL_OK)
            {
                while (1);
            }
            /* Associate the initialized DMA handle to the UART handle */
            __DAL_LINKDMA(&uartPort->Handle, hdmatx, uartPort->txDMAHandle);

            __DAL_DMA_SET_COUNTER(&uartPort->txDMAHandle, 0);
        } else
#endif
        {

            /* Enable the UART Transmit Data Register Empty Interrupt */
            SET_BIT(uartPort->USARTx->CTRL1, USART_CTRL1_TXBEIEN);
            SET_BIT(uartPort->USARTx->CTRL1, USART_CTRL1_TXCIEN);
        }
    }
    return;
}

bool checkUsartTxOutput(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);
    IO_t txIO = IOGetByTag(uart->tx.pin);

    if ((uart->txPinState == TX_PIN_MONITOR) && txIO) {
        if (IORead(txIO)) {
            // TX is high so we're good to transmit

            // Enable USART TX output
            uart->txPinState = TX_PIN_ACTIVE;
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uart->tx.af);

            // Enable the UART transmitter
            SET_BIT(s->Handle.Instance->CTRL1, USART_CTRL1_TXEN);

            return true;
        } else {
            // TX line is pulled low so don't enable USART TX
            return false;
        }
    }

    return true;
}

void uartTxMonitor(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);

    if (uart->txPinState == TX_PIN_ACTIVE) {
        IO_t txIO = IOGetByTag(uart->tx.pin);

        // Disable the UART transmitter
        CLEAR_BIT(s->Handle.Instance->CTRL1, USART_CTRL1_TXEN);

        // Switch TX to an input with pullup so it's state can be monitored
        uart->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(txIO, IOCFG_IPU);
    }
}

#ifdef USE_DMA

void uartTryStartTxDMA(uartPort_t *s)
{
    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
        if (IS_DMA_ENABLED(s->txDMAResource)) {
            // DMA is already in progress
            return;
        }

        DAL_UART_StateTypeDef state = DAL_UART_GetState(&s->Handle);
        if ((state & DAL_UART_STATE_BUSY_TX) == DAL_UART_STATE_BUSY_TX) {
            // UART is still transmitting
            return;
        }

        if (s->port.txBufferHead == s->port.txBufferTail) {
            // No more data to transmit
            s->txDMAEmpty = true;
            return;
        }

        uint16_t size;
        uint32_t fromwhere = s->port.txBufferTail;

        if (s->port.txBufferHead > s->port.txBufferTail) {
            size = s->port.txBufferHead - s->port.txBufferTail;
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            size = s->port.txBufferSize - s->port.txBufferTail;
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;

        DAL_UART_Transmit_DMA(&s->Handle, (uint8_t *)&s->port.txBuffer[fromwhere], size);
    }
}

static void handleUsartTxDma(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);

    uartTryStartTxDMA(s);

    if (s->txDMAEmpty && (uart->txPinState != TX_PIN_IGNORE)) {
        // Switch TX to an input with pullup so it's state can be monitored
        uartTxMonitor(s);
    }
}

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    UNUSED(descriptor);
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
        if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_FEIF))
        {
            DMA_CLEAR_FLAG(descriptor, DMA_IT_FEIF);
        }
        handleUsartTxDma(s);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_DMEIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_DMEIF);
    }
}
#endif // USE_DMA

FAST_IRQ_HANDLER void uartIrqHandler(uartPort_t *s)
{
    UART_HandleTypeDef *huart = &s->Handle;
    uint32_t isrflags = READ_REG(huart->Instance->STS);
    uint32_t cr1its = READ_REG(huart->Instance->CTRL1);
    uint32_t cr3its = READ_REG(huart->Instance->CTRL3);
    /* UART in mode Receiver ---------------------------------------------------*/
    if (!s->rxDMAResource && (((isrflags & USART_STS_RXBNEFLG) != RESET) && ((cr1its & USART_CTRL1_RXBNEIEN) != RESET))) {
        if (s->port.rxCallback) {
            s->port.rxCallback(huart->Instance->DATA, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = huart->Instance->DATA;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    // Detect completion of transmission
    if (((isrflags & USART_STS_TXCFLG) != RESET) && ((cr1its & USART_CTRL1_TXCIEN) != RESET)) {
        // Switch TX to an input with pullup so it's state can be monitored
        uartTxMonitor(s);

        __DAL_UART_CLEAR_FLAG(huart, UART_IT_TC);
    }

    if (!s->txDMAResource && (((isrflags & USART_STS_TXBEFLG) != RESET) && ((cr1its & USART_CTRL1_TXBEIEN) != RESET))) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            huart->Instance->DATA = (((uint16_t) s->port.txBuffer[s->port.txBufferTail]) & (uint16_t) 0x01FFU);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            __DAL_UART_DISABLE_IT(huart, UART_IT_TXE);
        }
    }

    if (((isrflags & USART_STS_OVREFLG) != RESET) && (((cr1its & USART_CTRL1_RXBNEIEN) != RESET)
                                                 || ((cr3its & USART_CTRL3_ERRIEN) != RESET))) {
        __DAL_UART_CLEAR_OREFLAG(huart);
    }

    if (((isrflags & USART_STS_IDLEFLG) != RESET) && ((cr1its & USART_STS_IDLEFLG) != RESET)) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }

        // clear
        (void) huart->Instance->STS;
        (void) huart->Instance->DATA;
    }
}

#endif // USE_UART
