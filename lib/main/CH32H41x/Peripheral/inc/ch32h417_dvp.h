/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_dvp.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the 
*                      DVP firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_DVP_H
#define __CH32H417_DVP_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

/* SPI Init structure definition */
typedef struct
{
    uint16_t DVP_ROW_NUM;             /* Specifies the DVP ROW number.
                                        This parameter can be a value */

    uint16_t DVP_COL_NUM;             /* Specifies the DVP COL number.
                                        This parameter can be a value */

    uint32_t DVP_DMA_BUF0_Addr;       /* Specifies the DVP Buffer0 address.
                                        This parameter can be a value */

    uint32_t DVP_DMA_BUF1_Addr;       /* Specifies the DVP Buffer1 address.
                                        This parameter can be a value */  

    uint16_t DVP_Window_HOFFCNT;       /* Specifies the DVP window horizontal displacement count */                                

    uint16_t DVP_Window_VST;           /* Specifies the DVP window Starting Vertical */   

    uint16_t DVP_Window_CAPCNT;       /* Specifies the DVP window capture count */                             

    uint16_t DVP_Window_VLine;           /* Specifies the DVP window Vertical count */ 

    uint8_t DVP_DataSize;             /* Specifies the DVP data size.
                                        This parameter can be a value of @ref DVP_data_size */

    uint8_t DVP_HCLK_P;              /* Specifies the DVP HCLK Polarity.
                                        This parameter can be a value of @ref DVP_Hclk_P */

    uint8_t DVP_HSYNC_P;              /* Specifies the DVP HSYNC Polarity.
                                        This parameter can be a value of @ref DVP_Hsync_P */ 

    uint8_t DVP_VSYNC_P;              /* Specifies the DVP VSYNC Polarity.
                                        This parameter can be a value of @ref DVP_Vsync_P */   

    uint8_t DVP_FrameCapRate;         /* Specifies the DVP frame capture rate control.
                                        This parameter can be a value of @ref DVP_FrameCapRate */

    uint8_t DVP_CaptureMode;          /* Specifies the DVP frame capture mode.
                                        This parameter can be a value of @ref DVP_CaptureMode */                                                                              

    FunctionalState DVP_Crop;         /* Specifies the DVP Crop feature mode.
                                        This parameter can be a value of @ref DVP_Crop */  

    FunctionalState DVP_JPEGMode;     /* Specifies whether the DVP JPEG mode.
                                        This parameter can be set to ENABLE or DISABLE. */
}DVP_InitTypeDef;

/* DVP_data_size */
#define DVP_DataSize_8b                ((uint8_t)0x00)
#define DVP_DataSize_10b               ((uint8_t)0x10)
#define DVP_DataSize_12b               ((uint8_t)0x30)

/* DVP_Hclk_P */
#define DVP_Hclk_P_Rising              ((uint8_t)0x00)
#define DVP_Hclk_P_Falling             ((uint8_t)0x08)

/* DVP_Hsync_P */
#define DVP_Hsync_P_High               ((uint8_t)0x00)
#define DVP_Hsync_P_Low                ((uint8_t)0x04)

/* DVP_Vsync_P */
#define DVP_Vsync_P_Low                ((uint8_t)0x00)
#define DVP_Vsync_P_High               ((uint8_t)0x02)

/* DVP_FrameCapRate */
#define DVP_FrameCapRate_100P          ((uint8_t)0x00)
#define DVP_FrameCapRate_50P           ((uint8_t)0x40)
#define DVP_FrameCapRate_25P           ((uint8_t)0x80)

/* DVP_CaptureMode */
#define DVP_CaptureMode_Snapshot       ((uint8_t)0x00)
#define DVP_CaptureMode_Continuous     ((uint8_t)0x10)

/* DVP_interrupts_definition */
#define DVP_IT_STR_FRM                 ((uint8_t)0x00)
#define DVP_IT_ROW_DONE                ((uint8_t)0x02)
#define DVP_IT_FRM_DONE                ((uint8_t)0x04)
#define DVP_IT_FIFO_OV                 ((uint8_t)0x08)
#define DVP_IT_STP_FRM                 ((uint8_t)0x10)

/* DVP_flags_definition */
#define DVP_FLAG_STR_FRM               ((uint8_t)0x00)
#define DVP_FLAG_ROW_DONE              ((uint8_t)0x02)
#define DVP_FLAG_FRM_DONE              ((uint8_t)0x04)
#define DVP_FLAG_FIFO_OV               ((uint8_t)0x08)
#define DVP_FLAG_STP_FRM               ((uint8_t)0x10)
#define DVP_FLAG_FIFO_RDY              ((uint8_t)0x80)
#define DVP_FLAG_FIFO_FULL             ((uint8_t)0x82)
#define DVP_FLAG_FIFO_OV_1             ((uint8_t)0x84)


void DVP_DeInit(void);
void DVP_Init(DVP_InitTypeDef *DVP_InitStruct);
void DVP_StructInit(DVP_InitTypeDef *DVP_InitStruct);
void DVP_Cmd(FunctionalState NewState);
void DVP_DMACmd(FunctionalState NewState);
void DVP_ReceiveCircuitResetCmd(FunctionalState NewState);
void DVP_FIFO_ResetCmd(FunctionalState NewState);
void DVP_ITConfig(uint8_t DVP_IT, FunctionalState NewState);
FlagStatus DVP_GetFlagStatus(uint8_t DVP_FLAG);
void DVP_ClearFlag(uint8_t DVP_FLAG);
ITStatus DVP_GetITStatus(uint8_t DVP_IT);
void DVP_ClearITPendingBit(uint8_t DVP_IT);
uint8_t DVP_GetRxFIFO_count(void);
uint16_t DVP_GetReceiveROW_count(void);
uint32_t DVP_GetReceiveData(void);

#ifdef __cplusplus
}
#endif

#endif 






