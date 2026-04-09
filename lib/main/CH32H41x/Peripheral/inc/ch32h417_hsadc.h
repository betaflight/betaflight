/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_hsadc.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the 
*                      HSADC firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_HSADC_H
#define __CH32H417_HSADC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"


/* ADC Init structure definition */
typedef struct
{
  uint32_t HSADC_FirstConversionCycle;       /* Specifies the HSADC first conversion cycle.
                                           This parameter can be a value of @ref HSADC_First_Conversion_Cycle */

  uint32_t HSADC_DataSize;                  /* Specifies the HSADC data size.
                                           This parameter can be a value of @ref HSADC_data_size */

  uint32_t HSADC_ClockDivision;             /* Specifies the HSADC clock division
                                           This parameter must be a number between 0x00 and 0x3F */
  
  uint32_t HSADC_RxAddress0;                /* Specifies the HSADC receive address0.
                                          This parameter can be a 32-bit address. */  

  uint32_t HSADC_RxAddress1;                /* Specifies the HSADC receive address1.
                                          This parameter can be a 32-bit address. */  

  uint16_t HSADC_BurstMode_TransferLen;     /* Specifies the HSADC burst mode transfer lenth.
                                           This parameter must be a number between 0x00 and 0xFFFF */

  uint16_t HSADC_BurstMode_DMA_LastTransferLen;/* Specifies the HSADC burst mode DMA last transfer lenth.
                                           This parameter must be a number between 0x00 and 0xFFFF */  

  uint16_t HSADC_DMA_TransferLen;          /* Specifies the HSADC DMA transfer lenth.
                                          This parameter must be a number between 0x00 and 0xFFFF */

  FunctionalState HSADC_DMA;               /* Specifies the HSADC DMA.
                                           This parameter can be set to ENABLE or DISABLE */   

  FunctionalState HSADC_DualBuffer;        /* Specifies the HSADC dual buffer.
                                           This parameter can be set to ENABLE or DISABLE */  
                                                                                 
  FunctionalState HSADC_BurstMode;        /* Specifies the HSADC burst mode.
                                           This parameter can be set to ENABLE or DISABLE */ 
}HSADC_InitTypeDef;

/* HSADC_First_Conversion_Cycle */
#define HSADC_First_Conversion_Cycle_8    ((uint32_t)0x00000000)
#define HSADC_First_Conversion_Cycle_9    ((uint32_t)0x00000020)
#define HSADC_First_Conversion_Cycle_10   ((uint32_t)0x00000040)
#define HSADC_First_Conversion_Cycle_11   ((uint32_t)0x00000060)

/* HSADC_data_size */
#define HSADC_DataSize_16b                ((uint32_t)0x00000000)
#define HSADC_DataSize_8b                 ((uint32_t)0x00000080)

/* HSADC_channels */
#define HSADC_Channel_0                   ((uint8_t)0x00)
#define HSADC_Channel_1                   ((uint8_t)0x01)
#define HSADC_Channel_2                   ((uint8_t)0x02)
#define HSADC_Channel_3                   ((uint8_t)0x03)
#define HSADC_Channel_4                   ((uint8_t)0x04)
#define HSADC_Channel_5                   ((uint8_t)0x05)
#define HSADC_Channel_6                   ((uint8_t)0x06)

/* HSADC_interrupts_definition */
#define HSADC_IT_EOC                      ((uint16_t)0x0100)
#define HSADC_IT_DMAEnd                   ((uint16_t)0x0200)
#define HSADC_IT_BurstEnd                 ((uint16_t)0x0400)

/* HSADC_flags_definition */
#define HSADC_FLAG_EOC                    ((uint16_t)0x0001)
#define HSADC_FLAG_DMAEnd                 ((uint16_t)0x0002)
#define HSADC_FLAG_BurstEnd               ((uint16_t)0x0004)
#define HSADC_FLAG_RXNE                   ((uint16_t)0x0008)
#define HSADC_FLAG_DualBufferAddr1        ((uint16_t)0x0010)
#define HSADC_FLAG_FIFO_NE                ((uint16_t)0x0100)
#define HSADC_FLAG_FIFO_Full              ((uint16_t)0x0200)
#define HSADC_FLAG_FIFO_OV                ((uint16_t)0x0400)

void HSADC_DeInit(void);
void HSADC_Init(HSADC_InitTypeDef *HSADC_InitStruct);
void HSADC_StructInit(HSADC_InitTypeDef *HSADC_InitStruct);
void HSADC_Cmd(FunctionalState NewState);
void HSADC_DMACmd(FunctionalState NewState);
void HSADC_DualBufferCmd(FunctionalState NewState);
void HSADC_BurstModeCmd(FunctionalState NewState);
void HSADC_ChannelConfig(uint8_t HSADC_Channel);
void HSADC_BurstEndCmd(FunctionalState NewState);
void HSADC_SoftwareStartConvCmd(FunctionalState NewState);
void HSADC_ITConfig(uint16_t HSADC_IT, FunctionalState NewState);
FlagStatus HSADC_GetFlagStatus(uint16_t HSADC_FLAG);
void HSADC_ClearFlag(uint16_t HSADC_FLAG);
ITStatus HSADC_GetITStatus(uint16_t HSADC_IT);
void HSADC_ClearITPendingBit(uint16_t HSADC_IT);
uint8_t HSADC_GetRxFIFO_count(void);
uint16_t HSADC_GetConversionValue(void);

#ifdef __cplusplus
}
#endif

#endif 






