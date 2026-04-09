/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_ltdc.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the
*                      LTDC firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_LTDC_H
#define __CH32H417_LTDC_H

#ifdef __cplusplus
 extern "C" {
#endif
#include "ch32h417.h"

/* LTDC Init structure definition   */
typedef struct
{
  uint32_t LTDC_HSPolarity;  /* configures the horizontal synchronization polarity.
                                              This parameter can be one value of @ref LTDC_HSPolarity */

  uint32_t LTDC_VSPolarity;  /* configures the vertical synchronization polarity.
                                              This parameter can be one value of @ref LTDC_VSPolarity */

  uint32_t LTDC_DEPolarity;  /* configures the data enable polarity. This parameter can
                                              be one of value of @ref LTDC_DEPolarity */

  uint32_t LTDC_PCPolarity;  /* configures the pixel clock polarity. This parameter can
                                              be one of value of @ref LTDC_PCPolarity */

  uint32_t LTDC_HorizontalSync;  /* configures the number of Horizontal synchronization 
                                              width. This parameter must range from 0x000 to 0xFFF. */

  uint32_t LTDC_VerticalSync;  /* configures the number of Vertical synchronization 
                                              height. This parameter must range from 0x000 to 0x7FF. */

  uint32_t LTDC_AccumulatedHBP;  /* configures the accumulated horizontal back porch width.
                                              This parameter must range from LTDC_HorizontalSync to 0xFFF. */

  uint32_t LTDC_AccumulatedVBP;  /* configures the accumulated vertical back porch height.
                                              This parameter must range from LTDC_VerticalSync to 0x7FF. */

  uint32_t LTDC_AccumulatedActiveW;  /* configures the accumulated active width. This parameter 
                                              must range from LTDC_AccumulatedHBP to 0xFFF. */

  uint32_t LTDC_AccumulatedActiveH;  /* configures the accumulated active height. This parameter 
                                              must range from LTDC_AccumulatedVBP to 0x7FF. */

  uint32_t LTDC_TotalWidth;  /* configures the total width. This parameter 
                                              must range from LTDC_AccumulatedActiveW to 0xFFF. */

  uint32_t LTDC_TotalHeigh;  /* configures the total height. This parameter 
                                              must range from LTDC_AccumulatedActiveH to 0x7FF. */

  uint32_t LTDC_BackgroundRedValue;  /* configures the background red value.
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_BackgroundGreenValue;  /* configures the background green value.
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_BackgroundBlueValue;  /* configures the background blue value.
                                              This parameter must range from 0x00 to 0xFF. */
} LTDC_InitTypeDef;

/* LTDC Layer structure definition   */
typedef struct
{
  uint32_t LTDC_HorizontalStart;  /* Configures the Window Horizontal Start Position.
                                              This parameter must range from 0x000 to 0xFFF. */

  uint32_t LTDC_HorizontalStop;  /* Configures the Window Horizontal Stop Position.
                                              This parameter must range from 0x0000 to 0xFFFF. */

  uint32_t LTDC_VerticalStart;  /* Configures the Window vertical Start Position.
                                              This parameter must range from 0x000 to 0xFFF. */

  uint32_t LTDC_VerticalStop;  /* Configures the Window vaertical Stop Position.
                                              This parameter must range from 0x0000 to 0xFFFF. */

  uint32_t LTDC_PixelFormat;  /* Specifies the pixel format. This parameter can be 
                                              one of value of @ref LTDC_Pixelformat */

  uint32_t LTDC_ConstantAlpha;  /* Specifies the constant alpha used for blending.
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_DefaultColorBlue;  /* Configures the default blue value.
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_DefaultColorGreen;  /* Configures the default green value.
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_DefaultColorRed;  /* Configures the default red value.
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_DefaultColorAlpha;  /* Configures the default alpha value.
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_BlendingFactor_1;  /* Select the blending factor 1. This parameter 
                                              can be one of value of @ref LTDC_BlendingFactor1 */

  uint32_t LTDC_BlendingFactor_2;  /* Select the blending factor 2. This parameter 
                                              can be one of value of @ref LTDC_BlendingFactor2 */

  uint32_t LTDC_CFBStartAdress;  /* Configures the color frame buffer address */

  uint32_t LTDC_CFBLineLength;  /* Configures the color frame buffer line length. 
                                              This parameter must range from 0x0000 to 0x1FFF. */

  uint32_t LTDC_CFBPitch;  /* Configures the color frame buffer pitch in bytes.
                                              This parameter must range from 0x0000 to 0x1FFF. */

  uint32_t LTDC_CFBLineNumber;  /* Specifies the number of line in frame buffer. 
                                              This parameter must range from 0x000 to 0x7FF. */
} LTDC_Layer_InitTypeDef;

/* LTDC Position structure definition   */
typedef struct
{
  uint32_t LTDC_POSX;  /*  Current X Position */
  uint32_t LTDC_POSY;  /*  Current Y Position */
} LTDC_PosTypeDef;

/* LTDC RGB structure definition   */
typedef struct
{
  uint32_t LTDC_BlueWidth;   /* Blue width */
  uint32_t LTDC_GreenWidth;  /* Green width */
  uint32_t LTDC_RedWidth;    /* Red width */
} LTDC_RGBTypeDef;

/* LTDC Color Keying structure definition   */
typedef struct
{
  uint32_t LTDC_ColorKeyBlue;  /* Configures the color key blue value. 
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_ColorKeyGreen;  /* Configures the color key green value. 
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_ColorKeyRed;  /* Configures the color key red value. 
                                              This parameter must range from 0x00 to 0xFF. */
} LTDC_ColorKeying_InitTypeDef;

/* LTDC CLUT structure definition   */
typedef struct
{
  uint32_t LTDC_CLUTAdress;  /* Configures the CLUT address.
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_BlueValue;  /* Configures the blue value. 
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_GreenValue;  /* Configures the green value. 
                                              This parameter must range from 0x00 to 0xFF. */

  uint32_t LTDC_RedValue;  /* Configures the red value.
                                              This parameter must range from 0x00 to 0xFF. */
} LTDC_CLUT_InitTypeDef;


/* LTDC_HSPolarity */
#define LTDC_HSPolarity_AL         ((uint32_t)0x00000000) 
#define LTDC_HSPolarity_AH         LTDC_GCR_HSPOL      

/* LTDC_VSPolarity */
#define LTDC_VSPolarity_AL         ((uint32_t)0x00000000) 
#define LTDC_VSPolarity_AH         LTDC_GCR_VSPOL       

/* LTDC_DEPolarity */
#define LTDC_DEPolarity_AL         ((uint32_t)0x00000000) 
#define LTDC_DEPolarity_AH         LTDC_GCR_DEPOL    

/* LTDC_PCPolarity */
#define LTDC_PCPolarity_IPC        ((uint32_t)0x00000000) 
#define LTDC_PCPolarity_IIPC       LTDC_GCR_PCPOL      

/* LTDC_Reload */
#define LTDC_IMReload              LTDC_SRCR_IMR 
#define LTDC_VBReload              LTDC_SRCR_VBR


/* LTDC_Position */
#define LTDC_POS_CY                LTDC_CPSR_CYPOS
#define LTDC_POS_CX                LTDC_CPSR_CXPOS


/* LTDC_CurrentStatus */
#define LTDC_CD_VDES               LTDC_CDSR_VDES
#define LTDC_CD_HDES               LTDC_CDSR_HDES
#define LTDC_CD_VSYNC              LTDC_CDSR_VSYNCS
#define LTDC_CD_HSYNC              LTDC_CDSR_HSYNCS

/* LTDC_Interrupts_definition */
#define LTDC_IT_LI                 LTDC_IER_LIE
#define LTDC_IT_FU                 LTDC_IER_FUIE
#define LTDC_IT_RR                 LTDC_IER_RRIE

/* LTDC_Flag_definition */
#define LTDC_FLAG_LI               LTDC_ISR_LIF
#define LTDC_FLAG_FU               LTDC_ISR_FUIF
#define LTDC_FLAG_RR               LTDC_ISR_RRIF

/* LTDC_Pixelformat */
#define LTDC_Pixelformat_ARGB8888  ((uint32_t)0x00000000)
#define LTDC_Pixelformat_RGB888    ((uint32_t)0x00000001)
#define LTDC_Pixelformat_RGB565    ((uint32_t)0x00000002)
#define LTDC_Pixelformat_ARGB1555  ((uint32_t)0x00000003)
#define LTDC_Pixelformat_ARGB4444  ((uint32_t)0x00000004)
#define LTDC_Pixelformat_L8        ((uint32_t)0x00000005)
#define LTDC_Pixelformat_AL44      ((uint32_t)0x00000006)
#define LTDC_Pixelformat_AL88      ((uint32_t)0x00000007)

/* LTDC_BlendingFactor1 */
#define LTDC_BlendingFactor1_CA    ((uint32_t)0x00000400)
#define LTDC_BlendingFactor1_PAxCA ((uint32_t)0x00000600)

/* LTDC_BlendingFactor2 */
#define LTDC_BlendingFactor2_CA    ((uint32_t)0x00000005)
#define LTDC_BlendingFactor2_PAxCA ((uint32_t)0x00000007)


void LTDC_DeInit(void);
void LTDC_Init(LTDC_InitTypeDef* LTDC_InitStruct);
void LTDC_StructInit(LTDC_InitTypeDef* LTDC_InitStruct);
void LTDC_Cmd(FunctionalState NewState);
void LTDC_DitherCmd(FunctionalState NewState);
LTDC_RGBTypeDef LTDC_GetRGBWidth(void);
void LTDC_RGBStructInit(LTDC_RGBTypeDef* LTDC_RGB_InitStruct);
void LTDC_LIPConfig(uint32_t LTDC_LIPositionConfig);
void LTDC_ReloadConfig(uint32_t LTDC_Reload);
void LTDC_LayerInit(LTDC_Layer_TypeDef* LTDC_Layerx, LTDC_Layer_InitTypeDef* LTDC_Layer_InitStruct);
void LTDC_LayerStructInit(LTDC_Layer_InitTypeDef* LTDC_Layer_InitStruct);
void LTDC_LayerCmd(LTDC_Layer_TypeDef* LTDC_Layerx, FunctionalState NewState);
LTDC_PosTypeDef LTDC_GetPosStatus(void);
void LTDC_PosStructInit(LTDC_PosTypeDef* LTDC_Pos_InitStruct);
FlagStatus LTDC_GetCDStatus(uint32_t LTDC_CD);
void LTDC_ColorKeyingConfig(LTDC_Layer_TypeDef* LTDC_Layerx, LTDC_ColorKeying_InitTypeDef* LTDC_colorkeying_InitStruct, FunctionalState NewState);
void LTDC_ColorKeyingStructInit(LTDC_ColorKeying_InitTypeDef* LTDC_colorkeying_InitStruct);
void LTDC_CLUTCmd(LTDC_Layer_TypeDef* LTDC_Layerx, FunctionalState NewState);
void LTDC_CLUTInit(LTDC_Layer_TypeDef* LTDC_Layerx, LTDC_CLUT_InitTypeDef* LTDC_CLUT_InitStruct);
void LTDC_CLUTStructInit(LTDC_CLUT_InitTypeDef* LTDC_CLUT_InitStruct);
void LTDC_LayerPosition(LTDC_Layer_TypeDef* LTDC_Layerx, uint16_t OffsetX, uint16_t OffsetY);
void LTDC_LayerAlpha(LTDC_Layer_TypeDef* LTDC_Layerx, uint8_t ConstantAlpha);
void LTDC_LayerAddress(LTDC_Layer_TypeDef* LTDC_Layerx, uint32_t Address);
void LTDC_LayerSize(LTDC_Layer_TypeDef* LTDC_Layerx, uint32_t Width, uint32_t Height);
void LTDC_LayerPixelFormat(LTDC_Layer_TypeDef* LTDC_Layerx, uint32_t PixelFormat);
void LTDC_ITConfig(uint32_t LTDC_IT, FunctionalState NewState);
FlagStatus LTDC_GetFlagStatus(uint32_t LTDC_FLAG);
void LTDC_ClearFlag(uint32_t LTDC_FLAG);
ITStatus LTDC_GetITStatus(uint32_t LTDC_IT);
void LTDC_ClearITPendingBit(uint32_t LTDC_IT);

#ifdef __cplusplus
}
#endif

#endif
