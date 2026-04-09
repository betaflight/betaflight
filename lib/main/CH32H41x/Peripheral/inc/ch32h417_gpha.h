/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_gpha.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the
*                      GPHA firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_GPHA_H
#define __CH32H417_GPHA_H

#ifdef __cplusplus
  extern "C" {
#endif
#include "ch32h417.h"

/* GPHA Init structure definition  */
typedef struct
{
  uint32_t GPHA_Mode; /* configures the GPHA transfer mode.
                                            This parameter can be one value of @ref GPHA_MODE */

  uint32_t GPHA_CMode; /* configures the color format of the output image.
                                            This parameter can be one value of @ref GPHA_CMODE */

  uint32_t GPHA_OutputBlue; /* configures the blue value of the output image. 
                                            This parameter must range:
                                            - from 0x00 to 0xFF if ARGB8888 color mode is slected
                                            - from 0x00 to 0xFF if RGB888 color mode is slected
                                            - from 0x00 to 0x1F if RGB565 color mode is slected
                                            - from 0x00 to 0x1F if ARGB1555 color mode is slected
                                            - from 0x00 to 0x0F if ARGB4444 color mode is slected  */

  uint32_t GPHA_OutputGreen; /* configures the green value of the output image. 
                                            This parameter must range:
                                            - from 0x00 to 0xFF if ARGB8888 color mode is selected
                                            - from 0x00 to 0xFF if RGB888 color mode is selected
                                            - from 0x00 to 0x2F if RGB565 color mode is selected
                                            - from 0x00 to 0x1F if ARGB1555 color mode is selected
                                            - from 0x00 to 0x0F if ARGB4444 color mode is selected  */

  uint32_t GPHA_OutputRed; /* configures the red value of the output image. 
                                            This parameter must range:
                                            - from 0x00 to 0xFF if ARGB8888 color mode is slected
                                            - from 0x00 to 0xFF if RGB888 color mode is slected
                                            - from 0x00 to 0x1F if RGB565 color mode is slected
                                            - from 0x00 to 0x1F if ARGB1555 color mode is slected
                                            - from 0x00 to 0x0F if ARGB4444 color mode is slected  */

  uint32_t GPHA_OutputAlpha; /* configures the alpha channel of the output color. 
                                            This parameter must range:
                                            - from 0x00 to 0xFF if ARGB8888 color mode is selected
                                            - from 0x00 to 0x01 if ARGB1555 color mode is selected
                                            - from 0x00 to 0x0F if ARGB4444 color mode is selected  */

  uint32_t GPHA_OutputMemoryAdd; /* Specifies the memory address. This parameter 
                                            must be range from 0x00000000 to 0xFFFFFFFF. */

  uint32_t GPHA_OutputOffset; /* Specifies the Offset value. This parameter must be range from
                                            0x0000 to 0x3FFF. */

  uint32_t GPHA_NumberOfLine; /* Configures the number of line of the area to be transfered.
                                            This parameter must range from 0x0000 to 0xFFFF */

  uint32_t GPHA_PixelPerLine; /* Configures the number pixel per line of the area to be transferred.
                                            This parameter must range from 0x0000 to 0x3FFF */
} GPHA_InitTypeDef;

/* GPHA foreground Init structure definition  */
typedef struct
{
  uint32_t GPHA_FGMA; /* configures the GPHA foreground memory address.
                                            This parameter must be range from 0x00000000 to 0xFFFFFFFF. */

  uint32_t GPHA_FGO; /* configures the GPHA foreground offset.
                                            This parameter must be range from 0x0000 to 0x3FFF. */

  uint32_t GPHA_FGCM; /* configures the GPHA foreground color mode . 
                                            This parameter can be one value of @ref GPHA_FGCM */

  uint32_t GPHA_FG_CLUT_CM; /* configures the GPHA foreground CLUT color mode. 
                                            This parameter can be one value of @ref GPHA_FG_CLUT_CM */

  uint32_t GPHA_FG_CLUT_SIZE; /* configures the GPHA foreground CLUT size. 
                                            This parameter must range from 0x00 to 0xFF. */

  uint32_t GPHA_FGPFC_ALPHA_MODE; /* configures the GPHA foreground alpha mode. 
                                            This parameter can be one value of @ref GPHA_FGPFC_ALPHA_MODE */

  uint32_t GPHA_FGPFC_ALPHA_VALUE; /* Specifies the GPHA foreground alpha value 
                                            must be range from 0x00 to 0xFF. */

  uint32_t GPHA_FGC_BLUE; /* Specifies the GPHA foreground blue value 
                                            must be range from 0x00 to 0xFF. */

  uint32_t GPHA_FGC_GREEN; /* Specifies the GPHA foreground green value 
                                            must be range from 0x00 to 0xFF. */

  uint32_t GPHA_FGC_RED; /* Specifies the GPHA foreground red value 
                                            must be range from 0x00 to 0xFF. */

  uint32_t GPHA_FGCMAR; /* Configures the GPHA foreground CLUT memory address.
                                            This parameter must range from 0x00000000 to 0xFFFFFFFF. */
} GPHA_FG_InitTypeDef;

/* GPHA background Init structure definition  */
typedef struct
{
  uint32_t GPHA_BGMA; /* configures the GPHA background memory address.
                                            This parameter must be range from 0x00000000 to 0xFFFFFFFF. */

  uint32_t GPHA_BGO; /* configures the GPHA background offset.
                                            This parameter must be range from 0x0000 to 0x3FFF. */

  uint32_t GPHA_BGCM; /* configures the GPHA background color mode . 
                                            This parameter can be one value of @ref GPHA_FGCM */

  uint32_t GPHA_BG_CLUT_CM; /* configures the GPHA background CLUT color mode. 
                                            This parameter can be one value of @ref GPHA_FG_CLUT_CM */

  uint32_t GPHA_BG_CLUT_SIZE; /* configures the GPHA background CLUT size. 
                                            This parameter must range from 0x00 to 0xFF. */

  uint32_t GPHA_BGPFC_ALPHA_MODE; /* configures the GPHA background alpha mode. 
                                            This parameter can be one value of @ref GPHA_FGPFC_ALPHA_MODE */

  uint32_t GPHA_BGPFC_ALPHA_VALUE; /* Specifies the GPHA background alpha value 
                                            must be range from 0x00 to 0xFF. */

  uint32_t GPHA_BGC_BLUE; /* Specifies the GPHA background blue value 
                                            must be range from 0x00 to 0xFF. */

  uint32_t GPHA_BGC_GREEN; /* Specifies the GPHA background green value 
                                            must be range from 0x00 to 0xFF. */

  uint32_t GPHA_BGC_RED; /* Specifies the GPHA background red value 
                                            must be range from 0x00 to 0xFF. */

  uint32_t GPHA_BGCMAR; /* Configures the GPHA background CLUT memory address.
                                            This parameter must range from 0x00000000 to 0xFFFFFFFF. */
} GPHA_BG_InitTypeDef;

/* GPHA_MODE  */
#define GPHA_M2M_PFC         ((uint32_t)0x00010000)
#define GPHA_M2M_BLEND       ((uint32_t)0x00020000)
#define GPHA_R2M             ((uint32_t)0x00030000)

/* GPHA_CMODE */
#define GPHA_ARGB8888        ((uint32_t)0x00000000)
#define GPHA_RGB888          ((uint32_t)0x00000001)
#define GPHA_RGB565          ((uint32_t)0x00000002)
#define GPHA_ARGB1555        ((uint32_t)0x00000003)
#define GPHA_ARGB4444        ((uint32_t)0x00000004)

/* GPHA_FGCM */
#define CM_ARGB8888          ((uint32_t)0x00000000)
#define CM_RGB888            ((uint32_t)0x00000001)
#define CM_RGB565            ((uint32_t)0x00000002)
#define CM_ARGB1555          ((uint32_t)0x00000003)
#define CM_ARGB4444          ((uint32_t)0x00000004)
#define CM_L8                ((uint32_t)0x00000005)
#define CM_AL44              ((uint32_t)0x00000006)
#define CM_AL88              ((uint32_t)0x00000007)
#define CM_L4                ((uint32_t)0x00000008)
#define CM_A8                ((uint32_t)0x00000009)
#define CM_A4                ((uint32_t)0x0000000A)

/* GPHA_FG_CLUT_CM */
#define CLUT_CM_ARGB8888     ((uint32_t)0x00000000)
#define CLUT_CM_RGB888       ((uint32_t)0x00000001)

/* GPHA_FGPFC_ALPHA_MODE */
#define NO_MODIF_ALPHA_VALUE ((uint32_t)0x00000000)
#define REPLACE_ALPHA_VALUE  ((uint32_t)0x00000001)
#define COMBINE_ALPHA_VALUE  ((uint32_t)0x00000002)

/* GPHA_Interrupts_definition */
#define GPHA_IT_CE           GPHA_CTLR_CEIE
#define GPHA_IT_CTC          GPHA_CTLR_CTCIE
#define GPHA_IT_CAE          GPHA_CTLR_CAEIE
#define GPHA_IT_TW           GPHA_CTLR_TWIE
#define GPHA_IT_TC           GPHA_CTLR_TCIE

/* GPHA_Flag_definition */
#define GPHA_FLAG_CE         GPHA_ISR_CEIF
#define GPHA_FLAG_CTC        GPHA_ISR_CTCIF
#define GPHA_FLAG_CAE        GPHA_ISR_CAEIF
#define GPHA_FLAG_TW         GPHA_ISR_TWIF
#define GPHA_FLAG_TC         GPHA_ISR_TCIF

void GPHA_DeInit(void);
void GPHA_Init(GPHA_InitTypeDef* GPHA_InitStruct);
void GPHA_StructInit(GPHA_InitTypeDef* GPHA_InitStruct);
void GPHA_StartTransfer(void);
void GPHA_AbortTransfer(void);
void GPHA_Suspend(FunctionalState NewState);
void GPHA_FGConfig(GPHA_FG_InitTypeDef* GPHA_FG_InitStruct);
void GPHA_FG_StructInit(GPHA_FG_InitTypeDef* GPHA_FG_InitStruct);
void GPHA_BGConfig(GPHA_BG_InitTypeDef* GPHA_BG_InitStruct);
void GPHA_BG_StructInit(GPHA_BG_InitTypeDef* GPHA_BG_InitStruct);
void GPHA_FGStart(FunctionalState NewState);
void GPHA_BGStart(FunctionalState NewState);
void GPHA_DeadTimeConfig(uint32_t GPHA_DeadTime, FunctionalState NewState);
void GPHA_LineWatermarkConfig(uint32_t GPHA_LWatermarkConfig);
void GPHA_ITConfig(uint32_t GPHA_IT, FunctionalState NewState);
FlagStatus GPHA_GetFlagStatus(uint32_t GPHA_FLAG);
void GPHA_ClearFlag(uint32_t GPHA_FLAG);
ITStatus GPHA_GetITStatus(uint32_t GPHA_IT);
void GPHA_ClearITPendingBit(uint32_t GPHA_IT);

#ifdef __cplusplus
}
#endif

#endif
