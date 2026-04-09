/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_swpmi.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the  
*                      SWPMI firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_SWPMI_H
#define __CH32H417_SWPMI_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

typedef struct 
{
    uint32_t BitRate; /* This member configures the SWPMI communication baud rate.
                         SWPMI_freq = SWPMI_clk / (((BitRate) + 1)  * 4).
                         The BitRate must be in the following range: 100000 to 2000000 */

    uint8_t TxBufferingMode; /* Specifies which transmission buffering mode to select.
                                This parameter can be a value of @ref Tx_Buffering_Mode */

    uint8_t RxBufferingMode; /* Specifies which reception buffering mode to select.
                                This parameter can be a value of @ref Rx_Buffering_Mode */

    FunctionalState LoopBackMode; /* Specifies whether the loop back mode is enabled.
                                     This parameter can be set to ENABLE or DISABLE. */ 

} SWPMI_InitTypeDef;


/* Tx_Buffering_Mode */
#define SWPMI_TxMode_Buffering_None                 ((uint32_t)0x0)
#define SWPMI_TxMode_Buffering_Single               ((uint32_t)0x2)
#define SWPMI_TxMode_Buffering_Multi                ((uint32_t)0xA)

/* Rx_Buffering_Mode */
#define SWPMI_RxMode_Buffering_None                 ((uint32_t)0x0)
#define SWPMI_RxMode_Buffering_Single               ((uint32_t)0x1)
#define SWPMI_RxMode_Buffering_Multi                ((uint32_t)0x5)

/* SWPMI interrupts definition */
#define SWPMI_IT_RXBF                               ((uint16_t)0x0001)
#define SWPMI_IT_TXBE                               ((uint16_t)0x0002)
#define SWPMI_IT_RXBER                              ((uint16_t)0x0004)
#define SWPMI_IT_RXOVR                              ((uint16_t)0x0008)
#define SWPMI_IT_TXUNR                              ((uint16_t)0x0010)
#define SWPMI_IT_RXNE                               ((uint16_t)0x0020)
#define SWPMI_IT_TXE                                ((uint16_t)0x0040)
#define SWPMI_IT_TC                                 ((uint16_t)0x0080)
#define SWPMI_IT_SR                                 ((uint16_t)0x0100)
#define SWPMI_IT_RDY                                ((uint16_t)0x0800)

/* SWPMI flags definition */
#define SWPMI_FLAG_RXBF                             ((uint16_t)0x0001)
#define SWPMI_FLAG_TXBE                             ((uint16_t)0x0002)
#define SWPMI_FLAG_RXBER                            ((uint16_t)0x0004)
#define SWPMI_FLAG_RXOVR                            ((uint16_t)0x0008)
#define SWPMI_FLAG_TXUNR                            ((uint16_t)0x0010)
#define SWPMI_FLAG_RXNE                             ((uint16_t)0x0020)
#define SWPMI_FLAG_TXE                              ((uint16_t)0x0040)
#define SWPMI_FLAG_TC                               ((uint16_t)0x0080)
#define SWPMI_FLAG_SR                               ((uint16_t)0x0100)
#define SWPMI_FLAG_SUSP                             ((uint16_t)0x0200)
#define SWPMI_FLAG_DEACT                            ((uint16_t)0x0400)
#define SWPMI_FLAG_RDY                              ((uint16_t)0x0800)


void SWPMI_DeInit(void);
void SWPMI_Init(SWPMI_InitTypeDef* SWPMI_InitStruct);
void SWPMI_StructInit(SWPMI_InitTypeDef* SWPMI_InitStruct);
void SWPMI_Cmd(FunctionalState NewState);
void SWPMI_ActivateCmd(FunctionalState NewState);
void SWPMI_TxBufferModeConfig(uint32_t TxBuffer_mode);
void SWPMI_RxBufferModeConfig(uint32_t RxBuffer_mode);
void SWPMI_LoopbackCmd(FunctionalState NewState);
uint8_t SWPMI_GetReceiveFrameLength(void);
void SWPMI_TransmitData32(uint32_t TxData);
uint32_t SWPMI_ReceiveData32(void);
void SWPMI_BypassCmd(FunctionalState NewState);
FlagStatus SWPMI_GetFlagStatus(uint32_t SWPMI_FLAG);
void SWPMI_ClearFlag(uint32_t SWPMI_FLAG);
void SWPMI_ITConfig(uint32_t SWPMI_IT, FunctionalState NewState);
ITStatus SWPMI_GetITStatus(uint32_t SWPMI_IT);
void SWPMI_ClearITPendingBit(uint32_t SWPMI_IT);


#ifdef __cplusplus
}
#endif

#endif 

