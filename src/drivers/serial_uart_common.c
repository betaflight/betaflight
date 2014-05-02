#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "serial_common.h"
#include "serial_uart_common.h"

uartPort_t *serialUSART1(uint32_t baudRate, portMode_t mode);
uartPort_t *serialUSART2(uint32_t baudRate, portMode_t mode);

serialPort_t *uartOpen(USART_TypeDef *USARTx, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode)
{
    DMA_InitTypeDef DMA_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    uartPort_t *s = NULL;

    if (USARTx == USART1) {
        s = serialUSART1(baudRate, mode);
    } else if (USARTx == USART2) {
        s = serialUSART2(baudRate, mode);
    } else {
        return (serialPort_t *)s;
    }
    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    // callback for IRQ-based RX ONLY
    s->port.callback = callback;
    s->port.mode = mode;
    s->port.baudRate = baudRate;

    USART_InitStructure.USART_BaudRate = baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    if (mode & MODE_SBUS) {
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    } else {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
    }
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (mode & MODE_RX)
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    if (mode & MODE_TX)
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;
    USART_Init(s->USARTx, &USART_InitStructure);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = s->rxDMAPeripheralBaseAddr;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    // Receive DMA or IRQ
    if (mode & MODE_RX) {
        if (s->rxDMAChannel) {
            DMA_InitStructure.DMA_BufferSize = s->port.rxBufferSize;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)s->port.rxBuffer;
            DMA_DeInit(s->rxDMAChannel);
            DMA_Init(s->rxDMAChannel, &DMA_InitStructure);
            DMA_Cmd(s->rxDMAChannel, ENABLE);
            USART_DMACmd(s->USARTx, USART_DMAReq_Rx, ENABLE);
            s->rxDMAPos = DMA_GetCurrDataCounter(s->rxDMAChannel);
        } else {
            USART_ClearITPendingBit(s->USARTx, USART_IT_RXNE);
            USART_ITConfig(s->USARTx, USART_IT_RXNE, ENABLE);
        }
    }

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = s->txDMAPeripheralBaseAddr;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    // Transmit DMA or IRQ
    if (mode & MODE_TX) {
        if (s->txDMAChannel) {
            DMA_InitStructure.DMA_BufferSize = s->port.txBufferSize;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
            DMA_DeInit(s->txDMAChannel);
            DMA_Init(s->txDMAChannel, &DMA_InitStructure);
            DMA_ITConfig(s->txDMAChannel, DMA_IT_TC, ENABLE);
            DMA_SetCurrDataCounter(s->txDMAChannel, 0);
            s->txDMAChannel->CNDTR = 0;
            USART_DMACmd(s->USARTx, USART_DMAReq_Tx, ENABLE);
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
        }
    }

    USART_Cmd(s->USARTx, ENABLE);

    return (serialPort_t *)s;
}

void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    USART_InitTypeDef USART_InitStructure;
    uartPort_t *s = (uartPort_t *)instance;

    USART_InitStructure.USART_BaudRate = baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (s->port.mode & MODE_RX)
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    if (s->port.mode & MODE_TX)
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;
    USART_Init(s->USARTx, &USART_InitStructure);

    s->port.baudRate = baudRate;
}

void uartSetMode(serialPort_t *s, portMode_t mode)
{
    // not implemented.
}


void uartStartTxDMA(uartPort_t *s)
{
    s->txDMAChannel->CMAR = (uint32_t)&s->port.txBuffer[s->port.txBufferTail];
    if (s->port.txBufferHead > s->port.txBufferTail) {
        s->txDMAChannel->CNDTR = s->port.txBufferHead - s->port.txBufferTail;
        s->port.txBufferTail = s->port.txBufferHead;
    } else {
        s->txDMAChannel->CNDTR = s->port.txBufferSize - s->port.txBufferTail;
        s->port.txBufferTail = 0;
    }
    s->txDMAEmpty = false;
    DMA_Cmd(s->txDMAChannel, ENABLE);
}

uint8_t uartTotalBytesWaiting(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t*)instance;
    // FIXME always returns 1 or 0, not the amount of bytes waiting
    if (s->rxDMAChannel)
        return s->rxDMAChannel->CNDTR != s->rxDMAPos;
    else
        return s->port.rxBufferTail != s->port.rxBufferHead;
}

// BUGBUG TODO TODO FIXME - What is the bug?
bool isUartTransmitBufferEmpty(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t *)instance;
    if (s->txDMAChannel)
        return s->txDMAEmpty;
    else
        return s->port.txBufferTail == s->port.txBufferHead;
}

uint8_t uartRead(serialPort_t *instance)
{
    uint8_t ch;
    uartPort_t *s = (uartPort_t *)instance;

    if (s->rxDMAChannel) {
        ch = s->port.rxBuffer[s->port.rxBufferSize - s->rxDMAPos];
        if (--s->rxDMAPos == 0)
            s->rxDMAPos = s->port.rxBufferSize;
    } else {
        ch = s->port.rxBuffer[s->port.rxBufferTail];
        s->port.rxBufferTail = (s->port.rxBufferTail + 1) % s->port.rxBufferSize;
    }

    return ch;
}

void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *s = (uartPort_t *)instance;
    s->port.txBuffer[s->port.txBufferHead] = ch;
    s->port.txBufferHead = (s->port.txBufferHead + 1) % s->port.txBufferSize;

    if (s->txDMAChannel) {
        if (!(s->txDMAChannel->CCR & 1))
            uartStartTxDMA(s);
    } else {
        USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
    }
}

const struct serialPortVTable uartVTable[] = {
    {
        uartWrite,
        uartTotalBytesWaiting,
        uartRead,
        uartSetBaudRate,
        isUartTransmitBufferEmpty,
        uartSetMode,
    }
};
