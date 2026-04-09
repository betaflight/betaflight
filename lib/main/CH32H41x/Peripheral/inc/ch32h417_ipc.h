/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_ipc.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the IPC 
*                      firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_IPC_H
#define __CH32H417_IPC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

/* IPC_Channel_enumeration */
typedef enum
{
    IPC_CH0 = 0,
    IPC_CH1,
    IPC_CH2,
    IPC_CH3
} IPC_Channel_TypeDef;

/* PC_TxCID_enumeration */
typedef enum
{
    IPC_TxCID0 = 0,
    IPC_TxCID1,
} IPC_TxCID_TypeDef;

/* PC_RxCID_enumeration */
typedef enum
{
    IPC_RxCID0 = 0,
    IPC_RxCID1,
} IPC_RxCID_TypeDef;

/* IPC Init Structure definition */
typedef struct
{
    IPC_Channel_TypeDef  IPC_CH;
    IPC_TxCID_TypeDef  TxCID;      
    IPC_RxCID_TypeDef  RxCID;      
    FunctionalState  TxIER;         
    FunctionalState  RxIER;    
    FunctionalState  AutoEN;    
}IPC_InitTypeDef;

/* IPC_CH_Sta_enumeration */
typedef enum
{
    IPC_CH_Sta_Bit0 = 0,
    IPC_CH_Sta_Bit1,
    IPC_CH_Sta_Bit2,
    IPC_CH_Sta_Bit3,
    IPC_CH_Sta_Bit4,
    IPC_CH_Sta_Bit5,
    IPC_CH_Sta_Bit6,
    IPC_CH_Sta_Bit7
} IPC_ChannelStateBit_TypeDef;

/* IPC_MSG_enumeration */
typedef enum
{
    IPC_MSG0 = 0,
    IPC_MSG1,
    IPC_MSG2,
    IPC_MSG3
} IPC_MSG_TypeDef;


void IPC_DeInit(void);
void IPC_Init(IPC_InitTypeDef* IPC_InitStruct);
void IPC_StructInit(IPC_InitTypeDef* IPC_InitStruct);
void IPC_CH0_Lock(void);
void IPC_CH1_Lock(void);
void IPC_CH2_Lock(void);
void IPC_CH3_Lock(void);
ITStatus IPC_GetITStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit);
ITStatus IPC_GetITMask(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit);
void IPC_ITConfig(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit, FunctionalState NewState);
FlagStatus IPC_GetFlagStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit);
void IPC_SetFlagStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit);
void IPC_ClearFlagStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit);
void IPC_WriteMSG(IPC_MSG_TypeDef IPC_MSG, uint32_t Data);
uint32_t IPC_ReadMSG(IPC_MSG_TypeDef IPC_MSG);

#ifdef __cplusplus
}
#endif

#endif 

