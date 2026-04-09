/********************************** (C) COPYRIGHT  *******************************
* File Name          : debug.h
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
#ifndef __CH32_DEBUG_H
#define __CH32_DEBUG_H

#ifdef __cplusplus
 extern "C" {
#endif

// #include "stdio.h"
#include "ch32h417.h"

/* UART Printf Definition */
#define DEBUG_UART1    1
#define DEBUG_UART2    2
#define DEBUG_UART3    3

/* DEBUG UATR Definition */
#ifndef DEBUG

#ifdef Core_V3F
// #define DEBUG   DEBUG_UART1

#elif defined(Core_V5F)
// #define DEBUG   DEBUG_UART2

#endif

#endif

/* Run Core Definition */
#define Run_Core_V3F         0
#define Run_Core_V5F         1
#define Run_Core_V3FandV5F   2

#ifndef Run_Core
#define Run_Core   Run_Core_V3FandV5F
// #define Run_Core   Run_Core_V5F
// #define Run_Core   Run_Core_V3F
#endif

/* Core start addrress Definition */
#ifndef Core_V3F_StartAddr
#define Core_V3F_StartAddr   0x00000000
#endif

#ifndef Core_V5F_StartAddr
#define Core_V5F_StartAddr   0x00010000
#endif


void Delay_Init(void);
void Delay_Us (uint32_t n);
void Delay_Ms (uint32_t n);
void USART_Printf_Init(uint32_t baudrate);

#ifdef __cplusplus
}
#endif

#endif 



