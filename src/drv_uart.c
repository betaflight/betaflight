#include "board.h"

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/
#define UART_BUFFER_SIZE    256

// Receive buffer, circular DMA
volatile uint8_t rxBuffer[UART_BUFFER_SIZE];
uint32_t rxDMAPos = 0;

volatile uint8_t txBuffer[UART_BUFFER_SIZE];
uint32_t txBufferTail = 0;
uint32_t txBufferHead = 0;

static void uartTxDMA(void)
{
    DMA1_Channel4->CMAR = (uint32_t)&txBuffer[txBufferTail];
    if (txBufferHead > txBufferTail) {
        DMA1_Channel4->CNDTR = txBufferHead - txBufferTail;
        txBufferTail = txBufferHead;
    } else {
        DMA1_Channel4->CNDTR = UART_BUFFER_SIZE - txBufferTail;
        txBufferTail = 0;
    }

    DMA_Cmd(DMA1_Channel4, ENABLE);
}

void DMA1_Channel4_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);

    if (txBufferHead != txBufferTail)
        uartTxDMA();
}

void uartInit(uint32_t speed)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // UART
    // USART1_TX    PA9
    // USART1_RX    PA10

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate = speed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    // Receive DMA into a circular buffer
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_BufferSize = UART_BUFFER_SIZE;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel5, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    rxDMAPos = DMA_GetCurrDataCounter(DMA1_Channel5);

    // Transmit DMA
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    DMA1_Channel4->CNDTR = 0;
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

uint16_t uartAvailable(void)
{
    return (DMA_GetCurrDataCounter(DMA1_Channel5) != rxDMAPos) ? true : false;
}

bool uartTransmitEmpty(void)
{
    return (txBufferTail == txBufferHead);
}

uint8_t uartRead(void)
{
    uint8_t ch;

    ch = rxBuffer[UART_BUFFER_SIZE - rxDMAPos];
    // go back around the buffer
    if (--rxDMAPos == 0)
        rxDMAPos = UART_BUFFER_SIZE;

    return ch;
}

uint8_t uartReadPoll(void)
{
    while (!uartAvailable()); // wait for some bytes
    return uartRead();
}

void uartWrite(uint8_t ch)
{
    txBuffer[txBufferHead] = ch;
    txBufferHead = (txBufferHead + 1) % UART_BUFFER_SIZE;

    // if DMA wasn't enabled, fire it up
    if (!(DMA1_Channel4->CCR & 1))
        uartTxDMA();
}

void uartPrint(char *str)
{
    while (*str)
        uartWrite(*(str++));
}

/* -------------------------- UART2 (Spektrum, GPS) ----------------------------- */
uartReceiveCallbackPtr uart2Callback = NULL;

void uart2Init(uint32_t speed, uartReceiveCallbackPtr func)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = speed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
    uart2Callback = func;
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
        if (uart2Callback)
            uart2Callback(USART_ReceiveData(USART2));
    }
}
