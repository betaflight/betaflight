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

/*
 * porting for ch32h41x by Temperslee
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_UART

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/inverter.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"

#include "drivers/dma.h"
#include "platform/dma.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"



void uartReconfigure(uartPort_t *uartPort)
{
    // usart_enable(uartPort->USARTx, FALSE);
    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(uartPort->USARTx, DISABLE);
    //init
    USART_InitStructure.USART_BaudRate = uartPort->port.baudRate;
    
    // this does not seem to matter for rx but will give bad data on tx!
    if (uartPort->port.options & SERIAL_PARITY_EVEN) {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    } else {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }

    USART_InitStructure.USART_StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_StopBits_2 : USART_StopBits_1;
    USART_InitStructure.USART_Parity   = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_Parity_Even : USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (uartPort->port.mode & MODE_RX)
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    if (uartPort->port.mode & MODE_TX)
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;

    USART_Init(uartPort->USARTx, &USART_InitStructure);
    // config external pin inverter (no internal pin inversion available)
    uartConfigureExternalPinInversion(uartPort);

    if (uartPort->port.options & SERIAL_BIDIR)
        USART_HalfDuplexCmd(uartPort->USARTx, ENABLE);
    else
        USART_HalfDuplexCmd(uartPort->USARTx, DISABLE);

    USART_Cmd(uartPort->USARTx, ENABLE);


    // Receive DMA or IRQ
    DMA_InitTypeDef DMA_InitStructure;
    if (uartPort->port.mode & MODE_RX) {
 #ifdef USE_DMA  
        if (uartPort->rxDMAResource) {

            DMA_StructInit(&DMA_InitStructure);
            DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
            DMA_InitStructure.DMA_PeripheralBaseAddr = uartPort->rxDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority  = DMA_Priority_Medium;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
            DMA_InitStructure.DMA_BufferSize = uartPort->port.rxBufferSize;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
            DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uartPort->port.rxBuffer;

            xDMA_DeInit(uartPort->rxDMAResource);
            xDMA_Init(uartPort->rxDMAResource, &DMA_InitStructure);
            xDMA_Cmd(uartPort->rxDMAResource, ENABLE);
            USART_DMACmd(uartPort->USARTx, USART_DMAReq_Rx, ENABLE);
            uartPort->rxDMAPos = xDMA_GetCurrDataCounter(uartPort->rxDMAResource);
        } else 
 #endif       
        {
            USART_ClearITPendingBit(uartPort->USARTx, USART_IT_RXNE);
            USART_ITConfig(uartPort->USARTx, USART_IT_RXNE, ENABLE);
            USART_ITConfig(uartPort->USARTx, USART_IT_IDLE, ENABLE);
        }
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
  #ifdef USE_DMA      
        if (uartPort->txDMAResource) {
            DMA_StructInit(&DMA_InitStructure);
            DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
            DMA_InitStructure.DMA_PeripheralBaseAddr = uartPort->txDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
            DMA_InitStructure.DMA_BufferSize = uartPort->port.txBufferSize;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;     
            DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uartPort->port.txBuffer;

            xDMA_DeInit(uartPort->txDMAResource);
            xDMA_Init(uartPort->txDMAResource, &DMA_InitStructure);
            xDMA_ITConfig(uartPort->txDMAResource, DMA_IT_TC, ENABLE);
            xDMA_SetCurrDataCounter(uartPort->txDMAResource, 0);
            USART_DMACmd(uartPort->USARTx, USART_DMAReq_Tx, ENABLE);

        } else 
  #endif      
        {
            USART_ITConfig(uartPort->USARTx, USART_IT_TXE, ENABLE);
        }
       USART_ITConfig(uartPort->USARTx, USART_IT_TC, ENABLE);
    }
    // TODO: usart_enable is called twice
   USART_Cmd(uartPort->USARTx, ENABLE); 
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
            // usart_transmitter_enable(s->USARTx, true);
            SET_BIT(s->USARTx->CTLR1, USART_CTLR1_TE);
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
        CLEAR_BIT(s->USARTx->CTLR1, USART_CTLR1_TE);
        // Switch TX to an input with pullup so it's state can be monitored
        uart->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(txIO, IOCFG_IPU);
    }
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    // uartTryStartTxDMA must be protected, since it is called from
    // uartWrite and handleUsartTxDma (an ISR).

    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
        if (IS_DMA_ENABLED(s->txDMAResource)) {
            // DMA is already in progress
            return;
        }

        // For F4, there are cases that NDTR is non-zero upon TC interrupt.
        // We couldn't find out the root cause, so mask the case here.
        if (xDMA_GetCurrDataCounter(s->txDMAResource)) {
            // Possible premature TC case.
            goto reenable;
        }

        if (s->port.txBufferHead == s->port.txBufferTail) {
            // No more data to transmit.
            s->txDMAEmpty = true;
            return;
        }

        // Start a new transaction.
        ((DMA_ARCH_TYPE*)s->txDMAResource)->MADDR =(uint32_t)&s->port.txBuffer[s->port.txBufferTail];

        if (s->port.txBufferHead > s->port.txBufferTail) {
            xDMA_SetCurrDataCounter(s->txDMAResource, s->port.txBufferHead - s->port.txBufferTail);
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            xDMA_SetCurrDataCounter(s->txDMAResource, s->port.txBufferSize - s->port.txBufferTail);
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;

    reenable:
        xDMA_Cmd(s->txDMAResource, ENABLE);
    }
}
#endif

static void handleUsartTxDma(uartPort_t *s)
{
    uartTryStartTxDMA(s);

    if (s->txDMAEmpty) {
        // Switch TX to an input with pullup so it's state can be monitored
        uartTxMonitor(s);
    }
}

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
        handleUsartTxDma(s);
    }

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    } 
}

FAST_IRQ_HANDLER void uartIrqHandler(uartPort_t *s)
{
    if (!s->rxDMAResource && (USART_GetFlagStatus(s->USARTx, USART_FLAG_RXNE) == SET)) {
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->DATAR, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->DATAR;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    // UART transmission completed
    if ((USART_GetFlagStatus(s->USARTx, USART_FLAG_TC) != RESET)) {
        USART_ClearFlag(s->USARTx, USART_FLAG_TC);

        // Switch TX to an input with pull-up so it's state can be monitored
        uartTxMonitor(s);
    }

    if (!s->txDMAResource && (USART_GetFlagStatus(s->USARTx, USART_FLAG_TXE) == SET)) {
        if (s->port.txBufferTail != s->port.txBufferHead) { 
            USART_SendData(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }

    if (USART_GetFlagStatus(s->USARTx, USART_FLAG_ORE) == SET) {
        USART_ClearFlag(s->USARTx, USART_FLAG_ORE); //clear by read DATAR?
        (void) s->USARTx->STATR;
        (void) s->USARTx->DATAR;
    }

    if (USART_GetFlagStatus(s->USARTx, USART_FLAG_IDLE) == SET) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }
        (void) s->USARTx->STATR;
        (void) s->USARTx->DATAR;
    }
}
#endif // USE_UART
