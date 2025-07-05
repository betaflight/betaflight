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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/inverter.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

void uartReconfigure(uartPort_t *uartPort)
{
    usart_deinit((uint32_t)uartPort->USARTx);
    usart_baudrate_set((uint32_t)uartPort->USARTx, uartPort->port.baudRate);

    if(uartPort->port.options & SERIAL_PARITY_EVEN) {
        usart_word_length_set((uint32_t)uartPort->USARTx, USART_WL_9BIT);
    } else {
        usart_word_length_set((uint32_t)uartPort->USARTx, USART_WL_8BIT);
    }

    if(uartPort->port.options & SERIAL_STOPBITS_2) {
        usart_stop_bit_set((uint32_t)uartPort->USARTx, USART_STB_2BIT);
    } else {
        usart_stop_bit_set((uint32_t)uartPort->USARTx, USART_STB_1BIT);
    }

    if(uartPort->port.options & SERIAL_PARITY_EVEN) {
        usart_parity_config((uint32_t)uartPort->USARTx, USART_PM_EVEN);
    } else {
        usart_parity_config((uint32_t)uartPort->USARTx, USART_PM_NONE);
    }

    usart_hardware_flow_rts_config((uint32_t)uartPort->USARTx, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config((uint32_t)uartPort->USARTx, USART_CTS_DISABLE);

    if (uartPort->port.mode & MODE_RX)
        usart_receive_config((uint32_t)uartPort->USARTx, USART_RECEIVE_ENABLE);
    if (uartPort->port.mode & MODE_TX)
        usart_transmit_config((uint32_t)uartPort->USARTx, USART_TRANSMIT_ENABLE);

     uartConfigureExternalPinInversion(uartPort);

    if (uartPort->port.options & SERIAL_BIDIR) {
        usart_halfduplex_enable((uint32_t)uartPort->USARTx);
    } else {
        usart_halfduplex_disable((uint32_t)uartPort->USARTx);
    }

    usart_enable((uint32_t)uartPort->USARTx);

    if (uartPort->port.mode & MODE_RX) {
#ifdef USE_DMA
        if (uartPort->rxDMAResource) {
            dma_single_data_parameter_struct dma_init_struct;
            dma_single_data_para_struct_init(&dma_init_struct);
            
            dma_init_struct.periph_addr = uartPort->rxDMAPeripheralBaseAddr;
            dma_init_struct.memory0_addr = (uint32_t)uartPort->port.rxBuffer;
            dma_init_struct.number = uartPort->port.rxBufferSize;
            dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
            dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
            dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
            dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
            dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
            dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
            
            xDMA_DeInit(uartPort->rxDMAResource);
            gd32_dma_init((uint32_t)uartPort->rxDMAResource, &dma_init_struct);
            xDMA_Cmd(uartPort->rxDMAResource, ENABLE);

            usart_dma_receive_config((uint32_t)uartPort->USARTx, USART_RECEIVE_DMA_ENABLE);

            uartPort->rxDMAPos = xDMA_GetCurrDataCounter(uartPort->rxDMAResource);
        } else
#endif
        {
            usart_interrupt_flag_clear((uint32_t)uartPort->USARTx, USART_INT_FLAG_RBNE);
            usart_interrupt_enable((uint32_t)uartPort->USARTx, USART_INT_RBNE);
            usart_interrupt_enable((uint32_t)uartPort->USARTx, USART_INT_IDLE);
        }
    }

    if (uartPort->port.mode & MODE_TX) {
#ifdef USE_DMA
        if (uartPort->txDMAResource) {
            dma_single_data_parameter_struct dma_init_struct;
            dma_single_data_para_struct_init(&dma_init_struct);
            
            dma_init_struct.periph_addr = uartPort->txDMAPeripheralBaseAddr;
            dma_init_struct.memory0_addr = (uint32_t)uartPort->port.txBuffer;
            dma_init_struct.number = uartPort->port.txBufferSize;
            dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
            dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
            dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
            dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
            dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
            dma_init_struct.priority = DMA_PRIORITY_MEDIUM;


            xDMA_DeInit(uartPort->txDMAResource);
            gd32_dma_init((uint32_t)uartPort->txDMAResource, &dma_init_struct);

            xDMA_ITConfig(uartPort->txDMAResource, DMA_INT_FTF | DMA_INT_FEE | DMA_INT_TAE | DMA_INT_SDE, ENABLE);
            xDMA_SetCurrDataCounter(uartPort->txDMAResource, 0);
            
            usart_dma_transmit_config((uint32_t)uartPort->USARTx, USART_TRANSMIT_DMA_ENABLE);
        } else
#endif
        {
            usart_interrupt_enable((uint32_t)uartPort->USARTx, USART_INT_TBE);
        }
        
        usart_interrupt_enable((uint32_t)uartPort->USARTx, USART_INT_TC);
    }

    usart_enable((uint32_t)uartPort->USARTx);
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{

    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
        if (IS_DMA_ENABLED(s->txDMAResource)) {
            return;
        }

        if (xDMA_GetCurrDataCounter(s->txDMAResource)) {
            goto reenable;
        }

        if (s->port.txBufferHead == s->port.txBufferTail) {
            s->txDMAEmpty = true;
            return;
        }

        xDMA_MemoryTargetConfig(s->txDMAResource, (uint32_t)&s->port.txBuffer[s->port.txBufferTail], DMA_MEMORY_0);

        unsigned chunk;
        if (s->port.txBufferHead > s->port.txBufferTail) {
            chunk = s->port.txBufferHead - s->port.txBufferTail;
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            chunk = s->port.txBufferSize - s->port.txBufferTail;
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;
        xDMA_SetCurrDataCounter(s->txDMAResource, chunk);
    reenable:
        xDMA_Cmd(s->txDMAResource, ENABLE);
    }
}
#endif

#endif // USE_UART
