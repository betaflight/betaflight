/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_opa.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the 
*                      OPA firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_OPA_H
#define __CH32H417_OPA_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

/* OPA member enumeration */
typedef enum
{
   OPA1 = 0,
   OPA2,
   OPA3,
}OPA_Num_TypeDef;

/* OPA PSEL enumeration */
typedef enum
{
   CHP0 = 0,
   CHP1
}OPA_PSEL_TypeDef;

/* OPA NSEL enumeration */
typedef enum
{
   CHN0 = 0,
   CHN1,
   CHN_PGA_8xIN,
   CHN_PGA_16xIN,
   CHN_PGA_32xIN,
   CHN_PGA_64xIN,
   CHN_OFF = 0x7
}OPA_NSEL_TypeDef;

/* OPA out channel enumeration */
typedef enum
{
   OUT_IO_OUT0 = 0,
   OUT_IO_OUT1,
   OUT_TO_CMP /* only for OPA1 */
}OPA_Mode_TypeDef;

/* OPA_FB_enumeration */
typedef enum
{
    FB_OFF = 0,
    FB_ON
} OPA_FB_TypeDef;

/* OPA_HS_enumeration */
typedef enum
{
    HS_OFF = 0,
    HS_ON
} OPA_HS_TypeDef;

/* OPA_PFG_DIF_enumeration */
typedef enum
{
    DIF_OFF = 0,
    DIF_ON
} OPA_PGADIF_TypeDef;

/* OPA Init Structure definition */
typedef struct
{
  OPA_PSEL_TypeDef PSEL;         /* Specifies the positive channel of OPA */
  OPA_NSEL_TypeDef NSEL;         /* Specifies the negative channel of OPA */
  OPA_Mode_TypeDef Mode;         /* Specifies the mode of OPA */
  OPA_FB_TypeDef FB;             /* Specifies the internal feedback resistor of OPA */
  OPA_PGADIF_TypeDef PGADIF;     /* Specifies the internal PGADIF of OPA */
  OPA_HS_TypeDef HS;             /* specifies high speed mode enable of OPA */
}OPA_InitTypeDef;

/* CMP_out_channel_enumeration */
typedef enum
{
    OUT_TO_IO = 0,
    OUT_TIM1_BKIN,
    OUT_TIM8_BKIN,
    OUT_TIM1_CH4,
    OUT_TIM2_CH4,
    OUT_TIM3_CH4,
    OUT_TIM4_CH4,
    OUT_TIM5_CH4,
    OUT_TIM8_CH4,
    OUT_TIM9_CH4,
    OUT_TIM10_CH4,
    OUT_TIM11_CH4,
    OUT_TIM12_CH4,
    OUT_LPTIM1_CH1,
    OUT_LPTIM2_CH1,
    OUT_FLOAT
} CMP_Mode_TypeDef;

/* CMP_NSEL_enumeration */
typedef enum
{
    CMP_CHN0 = 0,
    CMP_CHN1,
    CMP_DAC2,
    CMP_VREF,
} CMP_NSEL_TypeDef;

/* CMP_PSEL_enumeration */
typedef enum
{
    CMP_CHP1 = 0,
    CMP_CHP2,
    CMP_OPA1
} CMP_PSEL_TypeDef;

/* CMP_VREF_enumeration */
typedef enum
{
    CMP_VREF_OFF = 0,
    CMP_VREF_25PER_VDD,
    CMP_VREF_50PERT_VDD,
    CMP_VREF_75PERC_VDD
} CMP_VREF_TypeDef;

/* CMP_HYPSEL_enumeration */
typedef enum
{
    CMP_HYPSEL_OFF = 0,
    CMP_HYPSEL_10mV,
    CMP_HYPSEL_20mV,
    CMP_HYPSEL_30mV,
} CMP_HYPSEL_TypeDef;

/* CMP Init structure definition */
typedef struct
{
    CMP_Mode_TypeDef   Mode;     /* Specifies the mode of CMP */
    CMP_NSEL_TypeDef   NSEL;     /* Specifies the negative channel of CMP */
    CMP_PSEL_TypeDef   PSEL;     /* Specifies the positive channel of CMP */
    CMP_VREF_TypeDef   VREF;     /* Specifies the positive VREF of CMP */
    CMP_HYPSEL_TypeDef HYPSEL;     /* Specifies the HYEN of CMP */
} CMP_InitTypeDef;

/* CMP FILT_TimeBase division definition */
#define CMP_FILT_TimeBase_Div1   ((uint8_t)0x00)
#define CMP_FILT_TimeBase_Div2   ((uint8_t)0x01)
#define CMP_FILT_TimeBase_Div3   ((uint8_t)0x02)
#define CMP_FILT_TimeBase_Div4   ((uint8_t)0x03)


void OPA_CMP_DeInit(void);
void OPA_Init(OPA_Num_TypeDef OPAx,OPA_InitTypeDef *OPA_InitStruct);
void OPA_StructInit(OPA_InitTypeDef *OPA_InitStruct);
void OPA_Cmd(OPA_Num_TypeDef OPAx, FunctionalState NewState);
void OPA_CMP_Init(CMP_InitTypeDef *CMP_InitStruct);
void OPA_CMP_StructInit(CMP_InitTypeDef *CMP_InitStruct);
void OPA_CMP_Cmd(FunctionalState NewState);
void OPA_CMP_FILT_Cmd(FunctionalState NewState);
void OPA_CMP_FILTConfig(uint8_t FILT_TimeBase, uint16_t FILT_TimeInterval);
FlagStatus OPA_CMP_GetOutStatus(void);

#ifdef __cplusplus
}
#endif

#endif 

