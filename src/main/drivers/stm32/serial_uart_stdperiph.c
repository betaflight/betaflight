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
 * Initialization part of serial_uart.c
 */

/*
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/inverter.h"
#include "drivers/nvic.h"
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

void uartReconfigure(uartPort_t *uartPort)
{
    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(uartPort->USARTx, DISABLE);

    USART_InitStructure.USART_BaudRate = uartPort->port.baudRate;

    // according to the stm32 documentation wordlen has to be 9 for parity bits
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

    usartConfigurePinInversion(uartPort);

    if (uartPort->port.options & SERIAL_BIDIR)
        USART_HalfDuplexCmd(uartPort->USARTx, ENABLE);
    else
        USART_HalfDuplexCmd(uartPort->USARTx, DISABLE);

    USART_Cmd(uartPort->USARTx, ENABLE);

    // Receive DMA or IRQ
    DMA_InitTypeDef DMA_InitStructure;
    if (uartPort->port.mode & MODE_RX) {
        if (uartPort->rxDMAResource) {
            DMA_StructInit(&DMA_InitStructure);
            DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
            DMA_InitStructure.DMA_PeripheralBaseAddr = uartPort->rxDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
            DMA_InitStructure.DMA_BufferSize = uartPort->port.rxBufferSize;
#ifdef STM32F4
            DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
            DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
            DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
            DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

            DMA_InitStructure.DMA_Channel = uartPort->rxDMAChannel;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
            DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uartPort->port.rxBuffer;
#else
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uartPort->port.rxBuffer;
#endif

            xDMA_DeInit(uartPort->rxDMAResource);
            xDMA_Init(uartPort->rxDMAResource, &DMA_InitStructure);
            xDMA_Cmd(uartPort->rxDMAResource, ENABLE);
            USART_DMACmd(uartPort->USARTx, USART_DMAReq_Rx, ENABLE);
            uartPort->rxDMAPos = xDMA_GetCurrDataCounter(uartPort->rxDMAResource);
        } else {
            USART_ClearITPendingBit(uartPort->USARTx, USART_IT_RXNE);
            USART_ITConfig(uartPort->USARTx, USART_IT_RXNE, ENABLE);
            USART_ITConfig(uartPort->USARTx, USART_IT_IDLE, ENABLE);
        }
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
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

#ifdef STM32F4
            DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
            DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
            DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
            DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

            DMA_InitStructure.DMA_Channel = uartPort->txDMAChannel;
            DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
#else
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
#endif

            xDMA_DeInit(uartPort->txDMAResource);
            xDMA_Init(uartPort->txDMAResource, &DMA_InitStructure);

#ifdef STM32F4
            xDMA_ITConfig(uartPort->txDMAResource, DMA_IT_TC | DMA_IT_FE | DMA_IT_TE | DMA_IT_DME, ENABLE);
#else
            xDMA_ITConfig(uartPort->txDMAResource, DMA_IT_TC, ENABLE);
#endif

            xDMA_SetCurrDataCounter(uartPort->txDMAResource, 0);
            USART_DMACmd(uartPort->USARTx, USART_DMAReq_Tx, ENABLE);
        } else {
            USART_ITConfig(uartPort->USARTx, USART_IT_TXE, ENABLE);
        }
    }

    USART_Cmd(uartPort->USARTx, ENABLE);
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

        // For F4 (and F1), there are cases that NDTR (CNDTR for F1) is non-zero upon TC interrupt.
        // We couldn't find out the root cause, so mask the case here.
        // F3 is not confirmed to be vulnerable, but not excluded as a safety.

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

#ifdef STM32F4
        xDMA_MemoryTargetConfig(s->txDMAResource, (uint32_t)&s->port.txBuffer[s->port.txBufferTail], DMA_Memory_0);
#else
        DMAx_SetMemoryAddress(s->txDMAResource, (uint32_t)&s->port.txBuffer[s->port.txBufferTail]);
#endif

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

#endif // USE_UART
