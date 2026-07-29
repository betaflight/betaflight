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
#include "platform/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#ifdef USE_HAL_DRIVER
#include "platform/serial_uart_hal.h"
struct uartHalHandle_s uartHalHandles[UARTDEV_COUNT];
#endif

#define __HAL_UART_EX_GET_IT(__HANDLE__, __INTERRUPT__)   ((READ_BIT((__HANDLE__)->Instance->IER, __INTERRUPT__) == (__INTERRUPT__)) ? SET : RESET) //((((__HANDLE__)->Instance->IER) &  (__INTERRUPT__)) ? SET : RESET) 

#if(0)
static void usartConfigurePinInversion(uartPort_t *uartPort)
{
    UNUSED(uartPort);
}
#endif

#if UART_TRAIT_PINSWAP
static void uartConfigurePinSwap(uartPort_t *uartPort)
{
    UNUSED(uartPort);
}
#endif

// XXX uartReconfigure does not handle resource management properly.

void uartReconfigure(uartPort_t *uartPort)
{
    const serialPortIdentifier_e id = uartPort->port.identifier;
    UART_HandleTypeDef *pHandle = &uartHalHandles[id - SERIAL_PORT_UART_FIRST].hal;

    pHandle->Instance = (UART_TypeDef *)uartPort->USARTx;
    pHandle->Init.BaudRate = uartPort->port.baudRate;
    pHandle->Init.WordLength = UART_EX_WORDLENGTH_8B; //(uartPort->port.options & SERIAL_PARITY_EVEN) ? UART_EX_WORDLENGTH_9B : UART_EX_WORDLENGTH_8B; //polo: STM32 must set 9-bit.
    pHandle->Init.StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? UART_EX_STOPBITS_2 : UART_EX_STOPBITS_1;
    pHandle->Init.Parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? UART_EX_PARITY_EVEN : UART_EX_PARITY_NONE;
    pHandle->Init.HwFlowCtl = UART_EX_HWCONTROL_NONE;

    HAL_UART_EX_Init(pHandle);

    // Receive DMA or IRQ
    if (uartPort->port.mode & MODE_RX)
    {
#ifdef USE_DMA
        if (uartPort->rxDMAResource)
        {
            //TODO: if use DMA for uart.
        } else
#endif
        {
            /* Enable the UART Data Register not empty Interrupt */
            __HAL_UART_EX_ENABLE_IT(pHandle, UART_EX_IT_ERBFI);
        }
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
#ifdef USE_DMA
        if (uartPort->txDMAResource) {
            //TODO: if use DMA for uart.
        } else
#endif
        {
            /* Enable the UART Transmit Data Register Empty Interrupt */
            // __HAL_UART_EX_ENABLE_IT(&uartPort->Handle, UART_EX_IT_ETBEI); //polo: 4xF RTBEI must enabled when ready to write.
        }
    }
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
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uart->hardware->af);

            // Enable the UART transmitter
            //SET_BIT(s->USARTx->CR1, USART_CR1_TE);

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
        //CLEAR_BIT(s->USARTx->CR1, USART_CR1_TE);

        // Switch TX to an input with pullup so it's state can be monitored
        uart->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(txIO, IOCFG_IPU);
    }
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    UNUSED(s);
}

#if(0)
static void handleUsartTxDma(uartPort_t *s)
{
    UNUSED(s);
}
#endif

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    UNUSED(descriptor);
}
#endif

void uartIrqHandler(uartPort_t *s)
{   
    const serialPortIdentifier_e id = s->port.identifier;
    UART_HandleTypeDef *pHandle = &uartHalHandles[id - SERIAL_PORT_UART_FIRST].hal;

    pHandle->Instance = (UART_TypeDef *)s->USARTx;

	uint32_t usrflags   = READ_REG(pHandle->Instance->USR);
    uint32_t lsrflags   = READ_REG(pHandle->Instance->LSR);
    uint32_t ierits     = READ_REG(pHandle->Instance->IER);
    uint32_t errorflags = 0x00U;
//    uint32_t dmarequest = 0x00U;

    /* If no error occurs */
    uint8_t rbyte = 0;
    errorflags = (lsrflags & (uint32_t)(UART_EX_LSR_OE | UART_EX_LSR_PE | UART_EX_LSR_FE | UART_EX_LSR_RFE));
    if (errorflags == RESET){
        /* UART in mode Receiver ---------------------------------------------------*/
        if (!s->rxDMAResource && (((usrflags & UART_EX_USR_RFNE) != RESET) && ((ierits & UART_EX_IER_ERBFI) != RESET))) {
            rbyte = (uint8_t)(pHandle->Instance->RBR & (uint8_t)0xff);
            
            if (s->port.rxCallback) {
                s->port.rxCallback(rbyte, s->port.rxCallbackData);
            } else {
                s->port.rxBuffer[s->port.rxBufferHead] = rbyte;
                s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
            }
        }
    }else{
        rbyte = (uint8_t)(pHandle->Instance->RBR & (uint8_t)0xff);
    }


    if (!s->txDMAResource && (((usrflags & UART_EX_USR_TFE) != RESET) && ((ierits & UART_EX_IER_ETBEI) != RESET))) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            pHandle->Instance->THR = (uint8_t)(s->port.txBuffer[s->port.txBufferTail] & (uint8_t)0xff);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            __HAL_UART_EX_DISABLE_IT(pHandle, UART_EX_IT_ETBEI);
        }
    }
}
#endif // USE_UART
