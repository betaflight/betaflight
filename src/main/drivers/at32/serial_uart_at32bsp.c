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
 * Initialization part of serial_uart.c using at32 bsp driver
 *
 * Authors:
 * emsr ports the code to at32f435/7
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
#include "drivers/rcc.h"

#include "drivers/dma.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

static void usartConfigurePinInversion(uartPort_t *uartPort) {
#if !defined(USE_INVERTER) && !defined(STM32F303xC)
    UNUSED(uartPort);
#else
    bool inverted = uartPort->port.options & SERIAL_INVERTED;

#ifdef USE_INVERTER
    if (inverted) {
        // Enable hardware inverter if available.
        enableInverter(uartPort->USARTx, TRUE);
    }
#endif
#endif
}

static uartDevice_t *uartFindDevice(uartPort_t *uartPort)
{
    for (uint32_t i = 0; i < UARTDEV_COUNT_MAX; i++) {
        uartDevice_t *candidate = uartDevmap[i];

        if (&candidate->port == uartPort) {
            return candidate;
        }
    }
    return NULL;
}

static void uartConfigurePinSwap(uartPort_t *uartPort)
{
    uartDevice_t *uartDevice = uartFindDevice(uartPort);
    if (!uartDevice) {
        return;
    }

    if (uartDevice->pinSwap) {
        usart_transmit_receive_pin_swap(uartDevice->port.USARTx, TRUE);
    }
}

void uartReconfigure(uartPort_t *uartPort)
{
    usart_enable(uartPort->USARTx, FALSE);
    //init
    usart_init(uartPort->USARTx,
               uartPort->port.baudRate,
               USART_DATA_8BITS,
              (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_STOP_2_BIT : USART_STOP_1_BIT);

    //set parity
    usart_parity_selection_config(uartPort->USARTx,
            (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_PARITY_EVEN : USART_PARITY_NONE);

    //set hardware control
    usart_hardware_flow_control_set(uartPort->USARTx, USART_HARDWARE_FLOW_NONE);

    //set mode rx or tx
    if (uartPort->port.mode & MODE_RX) {
        usart_receiver_enable(uartPort->USARTx, TRUE);
    }
    
    if (uartPort->port.mode & MODE_TX) {
        usart_transmitter_enable(uartPort->USARTx, TRUE);
    }

    //config pin inverter
    usartConfigurePinInversion(uartPort);

    //config pin swap
    uartConfigurePinSwap(uartPort);

    if (uartPort->port.options & SERIAL_BIDIR) {
        usart_single_line_halfduplex_select(uartPort->USARTx, TRUE);
    } else {
        usart_single_line_halfduplex_select(uartPort->USARTx, FALSE);
    }
    //enable usart
    usart_enable(uartPort->USARTx, TRUE);

    // Receive DMA or IRQ
    dma_init_type DMA_InitStructure;
    if (uartPort->port.mode & MODE_RX) {
        if (uartPort->rxDMAResource) {

            dma_default_para_init(&DMA_InitStructure);
            DMA_InitStructure.loop_mode_enable=TRUE;
            DMA_InitStructure.peripheral_base_addr=uartPort->rxDMAPeripheralBaseAddr;
            DMA_InitStructure.priority  = DMA_PRIORITY_MEDIUM;
            DMA_InitStructure.peripheral_inc_enable =FALSE;
            DMA_InitStructure.peripheral_data_width =DMA_PERIPHERAL_DATA_WIDTH_BYTE;
            DMA_InitStructure.memory_inc_enable =TRUE;
            DMA_InitStructure.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
            DMA_InitStructure.memory_base_addr=(uint32_t)uartPort->port.rxBuffer;
            DMA_InitStructure.buffer_size = uartPort->port.rxBufferSize;
            DMA_InitStructure.direction= DMA_DIR_PERIPHERAL_TO_MEMORY;

            xDMA_DeInit(uartPort->rxDMAResource);
            xDMA_Init(uartPort->rxDMAResource, &DMA_InitStructure);
            xDMA_Cmd(uartPort->rxDMAResource, TRUE);
            usart_dma_receiver_enable(uartPort->USARTx,TRUE);
            uartPort->rxDMAPos = xDMA_GetCurrDataCounter(uartPort->rxDMAResource);
        } else {
            usart_flag_clear(uartPort->USARTx, USART_RDBF_FLAG);
            usart_interrupt_enable(uartPort->USARTx, USART_RDBF_INT, TRUE);
            usart_interrupt_enable(uartPort->USARTx, USART_IDLE_INT, TRUE);
        }
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
        if (uartPort->txDMAResource) {
            dma_default_para_init(&DMA_InitStructure);
            DMA_InitStructure.loop_mode_enable=FALSE;
            DMA_InitStructure.peripheral_base_addr=uartPort->txDMAPeripheralBaseAddr;
            DMA_InitStructure.priority  = DMA_PRIORITY_MEDIUM;
            DMA_InitStructure.peripheral_inc_enable =FALSE;
            DMA_InitStructure.peripheral_data_width =DMA_PERIPHERAL_DATA_WIDTH_BYTE;
            DMA_InitStructure.memory_inc_enable =TRUE;
            DMA_InitStructure.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
            DMA_InitStructure.memory_base_addr=(uint32_t)uartPort->port.txBuffer;
            DMA_InitStructure.buffer_size = uartPort->port.txBufferSize;
            DMA_InitStructure.direction= DMA_DIR_MEMORY_TO_PERIPHERAL;

            xDMA_DeInit(uartPort->txDMAResource);
            xDMA_Init(uartPort->txDMAResource, &DMA_InitStructure);
            xDMA_ITConfig(uartPort->txDMAResource, DMA_IT_TCIF, TRUE);
            xDMA_SetCurrDataCounter(uartPort->txDMAResource, 0);
            usart_dma_transmitter_enable(uartPort->USARTx, TRUE);

        } else {
            usart_interrupt_enable(uartPort->USARTx, USART_TDBE_INT, TRUE);
        }
        usart_interrupt_enable(uartPort->USARTx, USART_TDC_INT, TRUE);
    }

    usart_enable(uartPort->USARTx,TRUE);
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
            usart_transmitter_enable(s->USARTx, true);

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
        usart_transmitter_enable(s->USARTx, false);

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
        ((DMA_ARCH_TYPE*)s->txDMAResource) -> maddr =(uint32_t)&s->port.txBuffer[s->port.txBufferTail];

        if (s->port.txBufferHead > s->port.txBufferTail) {
            xDMA_SetCurrDataCounter(s->txDMAResource, s->port.txBufferHead - s->port.txBufferTail);
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            xDMA_SetCurrDataCounter(s->txDMAResource, s->port.txBufferSize - s->port.txBufferTail);
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;

    reenable:
        xDMA_Cmd(s->txDMAResource, TRUE);
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

void uartIrqHandler(uartPort_t *s)
{
    if (!s->rxDMAResource && (usart_flag_get(s->USARTx, USART_RDBF_FLAG) == SET)) {
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->dt, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->dt;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    // UART transmission completed
    if ((usart_flag_get(s->USARTx, USART_TDC_FLAG) != RESET)) {
        usart_flag_clear(s->USARTx, USART_TDC_FLAG);

        // Switch TX to an input with pull-up so it's state can be monitored
        uartTxMonitor(s);
    }

    if (!s->txDMAResource && (usart_flag_get(s->USARTx, USART_TDBE_FLAG) == SET)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            usart_data_transmit(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            usart_interrupt_enable(s->USARTx, USART_TDBE_INT, FALSE);
        }
    }

    if (usart_flag_get(s->USARTx, USART_ROERR_FLAG) == SET) {
        usart_flag_clear(s->USARTx, USART_ROERR_FLAG);
    }
    
    if (usart_flag_get(s->USARTx, USART_IDLEF_FLAG) == SET) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }

        (void) s->USARTx->sts;
        (void) s->USARTx->dt;
    }
}
#endif // USE_UART
