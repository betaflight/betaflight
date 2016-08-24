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

/*
 * Authors:
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build_config.h"

#include "common/utils.h"
#include "io.h"
#include "nvic.h"
#include "inverter.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

static void usartConfigurePinInversion(uartPort_t *uartPort) {
    bool inverted = uartPort->port.options & SERIAL_INVERTED;

    if(inverted)
    {
        if (uartPort->port.mode & MODE_RX)
        {
            uartPort->Handle.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
            uartPort->Handle.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
        }
        if (uartPort->port.mode & MODE_TX)
        {
            uartPort->Handle.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_TXINVERT_INIT;
            uartPort->Handle.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
        }
    }
}

static void uartReconfigure(uartPort_t *uartPort)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    /*RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3|
            RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_UART8;
    RCC_PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Usart6ClockSelection = RCC_USART6CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Uart7ClockSelection = RCC_UART7CLKSOURCE_SYSCLK;
    RCC_PeriphClkInit.Uart8ClockSelection = RCC_UART8CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);*/

    HAL_UART_DeInit(&uartPort->Handle);
    uartPort->Handle.Init.BaudRate = uartPort->port.baudRate;
    uartPort->Handle.Init.WordLength = UART_WORDLENGTH_8B;
    uartPort->Handle.Init.StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_STOPBITS_2 : USART_STOPBITS_1;
    uartPort->Handle.Init.Parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_PARITY_EVEN : USART_PARITY_NONE;
    uartPort->Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartPort->Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uartPort->Handle.Init.Mode = 0;

    if (uartPort->port.mode & MODE_RX)
        uartPort->Handle.Init.Mode |= UART_MODE_RX;
    if (uartPort->port.mode & MODE_TX)
        uartPort->Handle.Init.Mode |= UART_MODE_TX;

    usartConfigurePinInversion(uartPort);

    HAL_UART_IRQHandler(&uartPort->Handle);
    if(uartPort->port.options & SERIAL_BIDIR)
    {
        status = HAL_HalfDuplex_Init(&uartPort->Handle);
    }
    else
    {
        status = HAL_UART_Init(&uartPort->Handle);
    }
    return;
}

serialPort_t *uartOpen(USART_TypeDef *USARTx, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s = NULL;

    if (USARTx == USART1) {
        s = serialUART1(baudRate, mode, options);
#ifdef USE_UART2
    } else if (USARTx == USART2) {
        s = serialUART2(baudRate, mode, options);
#endif
#ifdef USE_UART3
    } else if (USARTx == USART3) {
        s = serialUART3(baudRate, mode, options);
#endif
#ifdef USE_UART4
    } else if (USARTx == UART4) {
        s = serialUART4(baudRate, mode, options);
#endif
#ifdef USE_UART5
    } else if (USARTx == UART5) {
        s = serialUART5(baudRate, mode, options);
#endif
#ifdef USE_UART6
    } else if (USARTx == USART6) {
        s = serialUART6(baudRate, mode, options);
#endif
#ifdef USE_UART7
    } else if (USARTx == UART7) {
        s = serialUART7(baudRate, mode, options);
#endif
#ifdef USE_UART8
    } else if (USARTx == UART8) {
        s = serialUART8(baudRate, mode, options);
#endif
    } else {
        return (serialPort_t *)s;
    }


    s->txDMAEmpty = true;
    
    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    // callback works for IRQ-based RX ONLY
    s->port.callback = callback;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    uartReconfigure(s);

    // Receive DMA or IRQ
    if (mode & MODE_RX) 
    {
        if (s->rxDMAStream) 
        {
            s->rxDMAHandle.Instance = s->rxDMAStream;
            s->rxDMAHandle.Init.Channel = s->rxDMAChannel;
            s->rxDMAHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
            s->rxDMAHandle.Init.PeriphInc = DMA_PINC_DISABLE;
            s->rxDMAHandle.Init.MemInc = DMA_MINC_ENABLE;
            s->rxDMAHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            s->rxDMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            s->rxDMAHandle.Init.Mode = DMA_CIRCULAR;
            s->rxDMAHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            s->rxDMAHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            s->rxDMAHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            s->rxDMAHandle.Init.MemBurst = DMA_MBURST_SINGLE;
            s->rxDMAHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
            
            
            HAL_DMA_DeInit(&s->rxDMAHandle);
            HAL_DMA_Init(&s->rxDMAHandle);
            /* Associate the initialized DMA handle to the UART handle */
            __HAL_LINKDMA(&s->Handle, hdmarx, s->rxDMAHandle);
            
            HAL_UART_Receive_DMA(&s->Handle, (uint8_t*)s->port.rxBuffer, s->port.rxBufferSize);

            s->rxDMAPos = __HAL_DMA_GET_COUNTER(&s->rxDMAHandle);

        } 
        else 
        {
//            __HAL_UART_CLEAR_IT(&cfg.uartport->Handle, UART_FLAG_RXNE);
//            __HAL_UART_ENABLE_IT(&cfg.uartport->Handle, UART_IT_RXNE);
            HAL_UART_Receive_IT(&s->Handle, (uint8_t*)s->port.rxBuffer, s->port.rxBufferSize);
        }
    }

    // Transmit DMA or IRQ
    if (mode & MODE_TX) {

        if (s->txDMAStream) {
            s->txDMAHandle.Instance = s->txDMAStream;
            s->txDMAHandle.Init.Channel = s->txDMAChannel;
            s->txDMAHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
            s->txDMAHandle.Init.PeriphInc = DMA_PINC_DISABLE;
            s->txDMAHandle.Init.MemInc = DMA_MINC_ENABLE;
            s->txDMAHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            s->txDMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            s->txDMAHandle.Init.Mode = DMA_NORMAL;
            s->txDMAHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            s->txDMAHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            s->txDMAHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            s->txDMAHandle.Init.MemBurst = DMA_MBURST_SINGLE;
            s->txDMAHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
            
            
            HAL_DMA_DeInit(&s->txDMAHandle);
            HAL_StatusTypeDef status = HAL_DMA_Init(&s->txDMAHandle);
            if(status != HAL_OK)
            {
                while(1);
            }
            /* Associate the initialized DMA handle to the UART handle */
            __HAL_LINKDMA(&s->Handle, hdmatx, s->txDMAHandle);
            
//            __HAL_DMA_ENABLE_IT(s->txDMAHandle, DMA_IT_TC|DMA_IT_FE|DMA_IT_TE|DMA_IT_DME);
            __HAL_DMA_SET_COUNTER(&s->txDMAHandle, 0);
        } else {
            __HAL_UART_ENABLE_IT(&s->Handle, UART_IT_TXE);
        }
    }

    return (serialPort_t *)s;
}

void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.baudRate = baudRate;
    uartReconfigure(uartPort);
}

void uartSetMode(serialPort_t *instance, portMode_t mode)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.mode = mode;
    uartReconfigure(uartPort);
}

void uartStartTxDMA(uartPort_t *s)
{
    HAL_UART_StateTypeDef state = HAL_UART_GetState(&s->Handle);
    if((state & HAL_UART_STATE_BUSY_TX) == HAL_UART_STATE_BUSY_TX)
        return;

    uint16_t size = 0;
//    HAL_UART_DMAStop(&s->Handle);
   
    if (s->port.txBufferHead > s->port.txBufferTail) {
        size = s->port.txBufferHead - s->port.txBufferTail;
        HAL_UART_Transmit_DMA(&s->Handle, (uint8_t *)&s->port.txBuffer[s->port.txBufferTail], size);
        s->port.txBufferTail = s->port.txBufferHead;
    } else {
        size = s->port.txBufferSize - s->port.txBufferTail;
        HAL_UART_Transmit_DMA(&s->Handle, (uint8_t *)&s->port.txBuffer[s->port.txBufferTail], size);
        s->port.txBufferTail = 0;
    }
    s->txDMAEmpty = false;
}

uint32_t uartTotalRxBytesWaiting(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t*)instance;

    if (s->rxDMAStream) {
        uint32_t rxDMAHead = __HAL_DMA_GET_COUNTER(s->Handle.hdmarx);

        if (rxDMAHead >= s->rxDMAPos) {
            return rxDMAHead - s->rxDMAPos;
        } else {
            return s->port.rxBufferSize + rxDMAHead - s->rxDMAPos;
        }
    }

    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        return s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        return s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
}

uint8_t uartTotalTxBytesFree(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t*)instance;

    uint32_t bytesUsed;

    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }

    if (s->txDMAStream) {
        /*
         * When we queue up a DMA request, we advance the Tx buffer tail before the transfer finishes, so we must add
         * the remaining size of that in-progress transfer here instead:
         */
        bytesUsed += __HAL_DMA_GET_COUNTER(s->Handle.hdmatx);

        /*
         * If the Tx buffer is being written to very quickly, we might have advanced the head into the buffer
         * space occupied by the current DMA transfer. In that case the "bytesUsed" total will actually end up larger
         * than the total Tx buffer size, because we'll end up transmitting the same buffer region twice. (So we'll be
         * transmitting a garbage mixture of old and new bytes).
         *
         * Be kind to callers and pretend like our buffer can only ever be 100% full.
         */
        if (bytesUsed >= s->port.txBufferSize - 1) {
            return 0;
        }
    }

    return (s->port.txBufferSize - 1) - bytesUsed;
}

bool isUartTransmitBufferEmpty(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t *)instance;
    if (s->txDMAStream)

        return s->txDMAEmpty;
    else
        return s->port.txBufferTail == s->port.txBufferHead;
}

uint8_t uartRead(serialPort_t *instance)
{
    uint8_t ch;
    uartPort_t *s = (uartPort_t *)instance;


    if (s->rxDMAStream) {

        ch = s->port.rxBuffer[s->port.rxBufferSize - s->rxDMAPos];
        if (--s->rxDMAPos == 0)
            s->rxDMAPos = s->port.rxBufferSize;
    } else {
        ch = s->port.rxBuffer[s->port.rxBufferTail];
        if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferTail = 0;
        } else {
            s->port.rxBufferTail++;
        }
    }

    return ch;
}

void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *s = (uartPort_t *)instance;
    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }

    if (s->txDMAStream) {
        if (!(s->txDMAStream->CR & 1))
            uartStartTxDMA(s);
    } else {
        __HAL_UART_ENABLE_IT(&s->Handle, UART_IT_TXE);
    }
}

const struct serialPortVTable uartVTable[] = {
    {
        uartWrite,
        uartTotalRxBytesWaiting,
        uartTotalTxBytesFree,
        uartRead,
        uartSetBaudRate,
        isUartTransmitBufferEmpty,
        uartSetMode,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
    }
};

