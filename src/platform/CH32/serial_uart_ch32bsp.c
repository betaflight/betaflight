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
#include "platform/dma.h"
#include "platform/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"



void uartReconfigure(uartPort_t *uartPort)
{
    USART_TypeDef *USARTx = (USART_TypeDef *)uartPort->USARTx;
    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(USARTx, DISABLE);
    //init
    USART_InitStructure.USART_BaudRate = uartPort->port.baudRate;

    // this does not seem to matter for rx but will give bad data on tx!
    if (uartPort->port.options & SERIAL_PARITY_EVEN) {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    } else {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }

    USART_InitStructure.USART_StopBits =
        (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_StopBits_2 : USART_StopBits_1;
    USART_InitStructure.USART_Parity =
        (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_Parity_Even : USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (uartPort->port.mode & MODE_RX) {
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    }
    if (uartPort->port.mode & MODE_TX) {
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;
    }

    USART_Init(USARTx, &USART_InitStructure);
    // config external pin inverter (no internal pin inversion available)
    uartConfigureExternalPinInversion(uartPort);

    if (uartPort->port.options & SERIAL_BIDIR) {
        USART_HalfDuplexCmd(USARTx, ENABLE);
    } else{
        USART_HalfDuplexCmd(USARTx, DISABLE);
    }

    USART_Cmd(USARTx, ENABLE);


    // Receive DMA or IRQ
    DMA_InitTypeDef DMA_InitStructure;
    if (uartPort->port.mode & MODE_RX) {
 #ifdef USE_DMA
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
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
            DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uartPort->port.rxBuffer;

            xDMA_DeInit(uartPort->rxDMAResource);
            xDMA_Init(uartPort->rxDMAResource, &DMA_InitStructure);
            xDMA_Cmd(uartPort->rxDMAResource, ENABLE);
            USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
            uartPort->rxDMAPos = xDMA_GetCurrDataCounter(uartPort->rxDMAResource);
        } else
 #endif
        {
            USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
            USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
            USART_ITConfig(USARTx, USART_IT_IDLE, uartPort->port.idleCallback ? ENABLE : DISABLE);
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
            USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
        } else
  #endif
        {
            USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
        }
        USART_ITConfig(USARTx, USART_IT_TC, ENABLE);
    }
    // TODO: usart_enable is called twice
    USART_Cmd(USARTx, ENABLE);
}

void uartSetIdleCallback(serialPort_t *instance, serialIdleCallbackPtr callback)
{
    uartPort_t *uartPort = container_of(instance, uartPort_t, port);
    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART1) {
        instance->idleCallback = callback;
        USART_ITConfig((USART_TypeDef *)uartPort->USARTx, USART_IT_IDLE,
            callback && (instance->mode & MODE_RX) && !uartPort->rxDMAResource ? ENABLE : DISABLE);
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
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uart->tx.af);

            // Enable the UART transmitter
            // usart_transmitter_enable(s->USARTx, true);
            SET_BIT(((USART_TypeDef *)s->USARTx)->CTLR1, USART_CTLR1_TE);
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
        CLEAR_BIT(((USART_TypeDef *)s->USARTx)->CTLR1, USART_CTLR1_TE);
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

    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA)
    {
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
        ((DMA_ARCH_TYPE *)s->txDMAResource)->MADDR = (uint32_t)&s->port.txBuffer[s->port.txBufferTail];

        if (s->port.txBufferHead > s->port.txBufferTail) {
            xDMA_SetCurrDataCounter(s->txDMAResource, s->port.txBufferHead - s->port.txBufferTail);
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            xDMA_SetCurrDataCounter(s->txDMAResource, s->port.txBufferSize - s->port.txBufferTail);
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;

 reenable:
        s->txCompletePending = false;
        USART_ClearFlag((USART_TypeDef *)s->USARTx, USART_FLAG_TC);
        ch32DmaBarrier();
        xDMA_Cmd(s->txDMAResource, ENABLE);
    }
}
#endif

static void uartTxMonitorIfComplete(uartPort_t *s)
{
    if (!s->txCompletePending || s->port.txBufferHead != s->port.txBufferTail) {
        return;
    }
    if (!s->txDMAResource || !IS_DMA_ENABLED(s->txDMAResource)) {
        s->txCompletePending = false;
        uartTxMonitor(s);
    }
}

static void handleUsartTxDma(uartPort_t *s)
{
    uartTryStartTxDMA(s);
    uartTxMonitorIfComplete(s);
}

void uartDmaIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    uartPort_t *s = &(((uartDevice_t *)(descriptor->userParam))->port);
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
        xDMA_Cmd(s->txDMAResource, DISABLE);
        ch32DmaBarrier();
        handleUsartTxDma(s);
    }

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    }
}

static inline void uartStoreRxByte(uartPort_t *s, uint16_t data)
{
    if (s->port.rxCallback) {
        s->port.rxCallback(data, s->port.rxCallbackData);
    } else {
        s->port.rxBuffer[s->port.rxBufferHead] = data;
        s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
    }
}

FAST_IRQ_HANDLER void uartIrqHandler(uartPort_t *s)
{
    USART_TypeDef *USARTx = (USART_TypeDef *)s->USARTx;
    uint16_t statr = USARTx->STATR;
    const uint16_t ctlr1 = USARTx->CTLR1;
    if (statr & (USART_STATR_RXNE | USART_STATR_ORE)) {
        if (!s->rxDMAResource || (statr & USART_STATR_ORE)) {
            const uint16_t data = USARTx->DATAR;
            if (!s->rxDMAResource && (statr & USART_STATR_RXNE)) {
                uartStoreRxByte(s, data);
            }
        }
    }

    if ((ctlr1 & USART_CTLR1_TCIE) && (statr & USART_STATR_TC)) {
        USARTx->STATR = (uint16_t)~USART_STATR_TC;
        s->txCompletePending = s->port.txBufferHead == s->port.txBufferTail;
        uartTxMonitorIfComplete(s);
    }

    if (!s->txDMAResource && (ctlr1 & USART_CTLR1_TXEIE) && (statr & USART_STATR_TXE)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            s->txCompletePending = false;
            USARTx->DATAR = s->port.txBuffer[s->port.txBufferTail];
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            CLEAR_BIT(USARTx->CTLR1, USART_CTLR1_TXEIE);
        }
    }

    if ((ctlr1 & USART_CTLR1_IDLEIE) && (statr & USART_STATR_IDLE)) {
        const uint16_t idleStatr = USARTx->STATR;
        if (idleStatr & USART_STATR_IDLE) {
            const uint16_t data = USARTx->DATAR;
            if (!s->rxDMAResource && (idleStatr & USART_STATR_RXNE)) {
                uartStoreRxByte(s, data);
            }
        }
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }
    }
}

#define CH32_UART_IRQ_HANDLER(number, dev)                \
    __FAST_INTERRUPT FAST_IRQ_HANDLER                     \
    void USART ## number ## _IRQHandler(void)             \
    {                                                     \
        uartPort_t *uartPort = &(uartDevice[(dev)].port); \
        uartIrqHandler(uartPort);                         \
    }

#ifdef USE_UART1
CH32_UART_IRQ_HANDLER(1, UARTDEV_1)
#endif
#ifdef USE_UART2
CH32_UART_IRQ_HANDLER(2, UARTDEV_2)
#endif
#ifdef USE_UART3
CH32_UART_IRQ_HANDLER(3, UARTDEV_3)
#endif
#ifdef USE_UART4
CH32_UART_IRQ_HANDLER(4, UARTDEV_4)
#endif
#ifdef USE_UART5
CH32_UART_IRQ_HANDLER(5, UARTDEV_5)
#endif
#ifdef USE_UART6
CH32_UART_IRQ_HANDLER(6, UARTDEV_6)
#endif
#ifdef USE_UART7
CH32_UART_IRQ_HANDLER(7, UARTDEV_7)
#endif
#ifdef USE_UART8
CH32_UART_IRQ_HANDLER(8, UARTDEV_8)
#endif
#endif // USE_UART
