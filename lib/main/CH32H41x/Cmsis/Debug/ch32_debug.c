/********************************** (C) COPYRIGHT  *******************************
* File Name          : debug.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for UART
*                      Printf , Delay functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32_debug.h"

static uint16_t  p_us = 0;
static uint32_t p_ms = 0;

/*********************************************************************
 * @fn      Delay_Init
 *
 * @brief   Initializes Delay Funcation.
 *
 * @return  none
 */
void Delay_Init(void)
{
    p_us = HCLKClock / 1000000;
    p_ms = (uint16_t)p_us * 1000;
}

/*********************************************************************
 * @fn      Delay_Us
 *
 * @brief   Microsecond Delay Time.
 *
 * @param   n - Microsecond number.
 *
 * @return  none
 */
void Delay_Us(uint32_t n)
{
    uint32_t i;
#ifdef Core_V3F
    SysTick0->ISR &= ~(1 << 0);
    i = (uint32_t)n * p_us;

    SysTick0->CNT = 0;
    SysTick0->CMP = i;
    SysTick0->CTLR = (1 << 2);
    SysTick0->CTLR |= (1 << 0);

    while((SysTick0->ISR & (1 << 0)) != (1 << 0))
        ;
    SysTick0->CTLR &= ~(1 << 0);

#elif defined(Core_V5F)
    SysTick0->ISR &= ~(1 << 1);
    i = (uint32_t)n * p_us;

    SysTick1->CNT = 0;
    SysTick1->CMP = i;
    SysTick1->CTLR = (1 << 2);
    SysTick1->CTLR |= (1 << 0);

    while((SysTick0->ISR & (1 << 1)) != (1 << 1))
        ;
    SysTick1->CTLR &= ~(1 << 0);
#endif
}

/*********************************************************************
 * @fn      Delay_Ms
 *
 * @brief   Millisecond Delay Time.
 *
 * @param   n - Millisecond number.
 *
 * @return  none
 */
void Delay_Ms(uint32_t n)
{
    uint32_t i;
#ifdef Core_V3F
    SysTick0->ISR &= ~(1 << 0);
    i = (uint32_t)n * p_ms;

    SysTick0->CNT = 0;
    SysTick0->CMP = i;
    SysTick0->CTLR = (1 << 2);
    SysTick0->CTLR |= (1 << 0);

    while((SysTick0->ISR & (1 << 0)) != (1 << 0))
        ;
    SysTick0->CTLR &= ~(1 << 0);
#elif defined(Core_V5F)
    SysTick0->ISR &= ~(1 << 1);
    i = (uint32_t)n * p_ms;

    SysTick1->CNT = 0;
    SysTick1->CMP = i;
    SysTick1->CTLR = (1 << 2);
    SysTick1->CTLR |= (1 << 0);

    while((SysTick0->ISR & (1 << 1)) != (1 << 1))
        ;
    SysTick1->CTLR &= ~(1 << 0);
#endif
}

/*********************************************************************
 * @fn      USART_Printf_Init
 *
 * @brief   Initializes the USARTx peripheral.
 *
 * @param   baudrate - USART communication baud rate.
 *
 * @return  none
 */
void USART_Printf_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

#if(DEBUG == DEBUG_UART1)  
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_USART1 | RCC_HB2Periph_GPIOA, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF7);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

#elif(DEBUG == DEBUG_UART2)
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART8, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF11);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

#elif(DEBUG == DEBUG_UART3)
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART6, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

#endif

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;

#if(DEBUG == DEBUG_UART1)
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

#elif(DEBUG == DEBUG_UART2)
    USART_Init(USART8, &USART_InitStructure);
    USART_Cmd(USART8, ENABLE);
    
#elif(DEBUG == DEBUG_UART3)
    USART_Init(USART6, &USART_InitStructure);
    USART_Cmd(USART6, ENABLE);

#endif
}

/*********************************************************************
 * @fn      _write
 *
 * @brief   Support Printf Function
 *
 * @param   *buf - UART send Data.
 *          size - Data length
 *
 * @return  size: Data length
 */

 #if 0
__attribute__((used)) int _write(int fd, char *buf, int size)
{
    int i = 0;

    for(i = 0; i < size; i++)
    {
// #if(DEBUG == DEBUG_UART1)
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        USART_SendData(USART1, *buf++);
// #elif(DEBUG == DEBUG_UART2)
//         while(USART_GetFlagStatus(USART8, USART_FLAG_TC) == RESET);
//         USART_SendData(USART8, *buf++);
// #elif(DEBUG == DEBUG_UART3)
//         while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
//         USART_SendData(USART6, *buf++);
// #endif
    }

    return size;
}
#endif

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
__attribute__((used)) void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}


void _fini() {}
void _init() {}


