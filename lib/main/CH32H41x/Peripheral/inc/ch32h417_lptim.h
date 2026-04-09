/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32h417_lptim.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/03/01
 * Description        : This file contains all the functions prototypes for the
 *                      TIM firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_LPTIM_H
#define __CH32H417_LPTIM_H


#ifdef __cplusplus
extern "C" {
#endif

#include "ch32h417.h"

typedef struct
{
    uint32_t LPTIM_ClockSource; /* Selects the clock source.
                                 This parameter can be a value of @ref LPTIM_Clock_Source */

    union{
        uint32_t LPTIM_ClockPolarity; /* Configures Clock Polarity(No Encoder mode).
                                       This parameter can be a value of @ref LPTIM_ClockPolarity */

        uint32_t LPTIM_EncoderMode; /* Configures Encoder mode.
                                     This parameter can be a value of @ref LPTIM_EncoderMode */
    };

    uint32_t LPTIM_ClockSampleTime; /* Configures Clock Sample Time.
                                     This parameter can be a value of @ref LPTIM_ClockSampleTime */

    uint32_t LPTIM_TriggerSampleTime; /* Configures Trigger Sample Time.
                                       This parameter can be a value of @ref LPTIM_TriggerSampleTime */

    uint32_t LPTIM_ClockPrescaler; /* Configures the clock Prescaler.
                                    This parameter can be a value of @ref LPTIM_ClockPrescaler */

    uint32_t LPTIM_TriggerSource; /* Configures trigger source.
                                    This parameter can be a value of @ref LPTIM_TriggerSource */

    uint32_t LPTIM_ExTriggerPolarity; /* Configures external trigger polarity.
                                       This parameter can be a value of @ref LPTIM_ExTriggerPolarity  */

    FunctionalState LPTIM_TimeOut; /* Specifies whether the time out function.
                             This parameter can be set to ENABLE or DISABLE */

    FunctionalState LPTIM_OnePulseMode; /* Specifies whether the PWM out one pulse.
                                         This parameter can be set to ENABLE or DISABLE */

    uint32_t LPTIM_OutputPolarity; /* Configures output polarity.
                                    This parameter can be a value of @ref LPTIM_OutputPolarity */

    uint32_t LPTIM_UpdateMode; /* Configures update mode.
                                This parameter can be a value of @ref LPTIM_UpdateMode */

    uint32_t LPTIM_CountSource; /* Configures Counter Source.
                                 This parameter can be a value of @ref LPTIM_CountSource */

    FunctionalState LPTIM_Encoder; /* Specifies whether open Encoder function.
                                    This parameter can be set to ENABLE or DISABLE */

    uint32_t LPTIM_InClockSource; /* Specifies Internal clock source.
                                   This parameter can be a value of @ref LPTIM_InClockSource */

    FunctionalState LPTIM_ForceOutHigh; /* Specifies whether the PWM out high level.
                                         This parameter can be set to ENABLE or DISABLE */

    FunctionalState LPTIM_SingleMode; /* Specifies whether single mode.
                                       This parameter can be set to ENABLE or DISABLE */

    FunctionalState LPTIM_ContinuousMode; /* Specifies whether continuous mode.
                                           This parameter can be set to ENABLE or DISABLE */

    FunctionalState LPTIM_PWMOut; /* Specifies whether PWM out function.
                                   This parameter can be set to ENABLE or DISABLE */

    FunctionalState LPTIM_CounterDirIndicat; /* Specifies whether counter direction indicate function.
                                              This parameter can be set to ENABLE or DISABLE */

    uint16_t LPTIM_Pulse;         /* Specifies the pulse value to be loaded into the Capture Compare Register.
                                   This parameter can be a number between 0x0000 and 0xFFFF */

    uint16_t LPTIM_Period;          /* Specifies the period value to be loaded into the active
                                     This parameter must be a number between 0x0000 and 0xFFFF.  */
} LPTIM_TimeBaseInitTypeDef;


/* LPTIM_Clock_Source */
#define LPTIM_ClockSource_In                      ((uint32_t)0x00000000)
#define LPTIM_ClockSource_Ex                      ((uint32_t)0x00000001)

/* LPTIM_ClockPolarity */
#define LPTIM_ClockPolarity_Rising                ((uint32_t)0x00000000)
#define LPTIM_ClockPolarity_Falling               ((uint32_t)0x00000002)
#define LPTIM_ClockPolarity_Rising_Falling        ((uint32_t)0x00000004)

/* LPTIM_ClockPrescalerTime */
#define LPTIM_ClockSampleTime_0T                  ((uint32_t)0x00000000)
#define LPTIM_ClockSampleTime_2T                  ((uint32_t)0x00000008)
#define LPTIM_ClockSampleTime_4T                  ((uint32_t)0x00000010)
#define LPTIM_ClockSampleTime_8T                  ((uint32_t)0x00000018)

/* LPTIM_TriggerSampleTime */
#define LPTIM_TriggerSampleTime_0T                ((uint32_t)0x00000000)
#define LPTIM_TriggerSampleTime_2T                ((uint32_t)0x00000040)
#define LPTIM_TriggerSampleTime_4T                ((uint32_t)0x00000080)
#define LPTIM_TriggerSampleTime_8T                ((uint32_t)0x000000C0)

/* LPTIM_ClockPrescaler */
#define LPTIM_TClockPrescaler_DIV1                ((uint32_t)0x00000000)
#define LPTIM_TClockPrescaler_DIV2                ((uint32_t)0x00000200)
#define LPTIM_TClockPrescaler_DIV4                ((uint32_t)0x00000400)
#define LPTIM_TClockPrescaler_DIV8                ((uint32_t)0x00000600)
#define LPTIM_TClockPrescaler_DIV16               ((uint32_t)0x00000800)
#define LPTIM_TClockPrescaler_DIV32               ((uint32_t)0x00000A00)
#define LPTIM_TClockPrescaler_DIV64               ((uint32_t)0x00000C00)
#define LPTIM_TClockPrescaler_DIV128              ((uint32_t)0x00000E00)

/* LPTIM_TriggerSource */
#define LPTIM_TriggerSource_ETR                   ((uint32_t)0x00000000)
#define LPTIM_TriggerSource_RTC_ALARM             ((uint32_t)0x00002000)
#define LPTIM_TriggerSource_TAMP                  ((uint32_t)0x00004000)

/* LPTIM_ExTriggerPolarity */
#define LPTIM_ExTriggerPolarity_Disable           ((uint32_t)0x00000000)
#define LPTIM_ExTriggerPolarity_Rising            ((uint32_t)0x00020000)
#define LPTIM_ExTriggerPolarity_Falling           ((uint32_t)0x00040000)
#define LPTIM_ExTriggerPolarity_Rising_Falling    ((uint32_t)0x00060000)

/* LPTIM_OutputPolarity */
#define LPTIM_OutputPolarity_High                 ((uint32_t)0x00000000)
#define LPTIM_OutputPolarity_Low                  ((uint32_t)0x00200000)

/* LPTIM_UpdateMode */
#define LPTIM_UpdateMode0                         ((uint32_t)0x00000000)
#define LPTIM_UpdateMode1                         ((uint32_t)0x00400000)

/* LPTIM_CountSource */
#define LPTIM_CountSource_Internal                ((uint32_t)0x00000000)
#define LPTIM_CountSource_External                ((uint32_t)0x00800000)

/* LPTIM_InClockSource */
#define LPTIM_InClockSource_PCLK1                 ((uint32_t)0x00000000)
#define LPTIM_InClockSource_HSI                   ((uint32_t)0x02000000)
#define LPTIM_InClockSource_LSE                   ((uint32_t)0x04000000)
#define LPTIM_InClockSource_LSI                   ((uint32_t)0x06000000)

/* LPTIM_Flag_Definition */
#define LPTIM_FLAG_DIR_SYNC                       ((uint32_t)0x00000080)
#define LPTIM_FLAG_DOWN                           ((uint32_t)0x00000040)
#define LPTIM_FLAG_UP                             ((uint32_t)0x00000020)
#define LPTIM_FLAG_ARROK                          ((uint32_t)0x00000010)
#define LPTIM_FLAG_CMPOK                          ((uint32_t)0x00000008)
#define LPTIM_FLAG_EXTTRIG                        ((uint32_t)0x00000004)
#define LPTIM_FLAG_ARRM                           ((uint32_t)0x00000002)
#define LPTIM_FLAG_CMPM                           ((uint32_t)0x00000001)

/* LPTIM_Interrupts_Definition */
#define LPTIM_IT_DOWN                             ((uint32_t)0x00000040)
#define LPTIM_IT_UP                               ((uint32_t)0x00000020)
#define LPTIM_IT_ARROK                            ((uint32_t)0x00000010)
#define LPTIM_IT_CMPOK                            ((uint32_t)0x00000008)
#define LPTIM_IT_EXTTRIG                          ((uint32_t)0x00000004)
#define LPTIM_IT_ARRM                             ((uint32_t)0x00000002)
#define LPTIM_IT_CMPM                             ((uint32_t)0x00000001)


void LPTIM_DeInit(LPTIM_TypeDef *LPTIMx);
void LPTIM_TimeBaseInit(LPTIM_TypeDef *LPTIMx, LPTIM_TimeBaseInitTypeDef* LPTIM_TimeBaseInitStruct);
void LPTIM_TimeBaseStructInit(LPTIM_TimeBaseInitTypeDef* LPTIM_TimeBaseInitStruct);
void LPTIM_CounterDirIndicat_Cmd(LPTIM_TypeDef *LPTIMx, FunctionalState NewState);
void LPTIM_OutCmd(LPTIM_TypeDef *LPTIMx, FunctionalState NewState);
void LPTIM_Cmd(LPTIM_TypeDef *LPTIMx, FunctionalState NewState);
uint16_t LPTIM_GetCounter(LPTIM_TypeDef *LPTIMx);
void LPTIM_SetAutoreload(LPTIM_TypeDef *LPTIMx, uint16_t Autoreload);
void LPTIM_SetCompare(LPTIM_TypeDef *LPTIMx, uint16_t Compare);
uint16_t LPTIM_GetCapture(LPTIM_TypeDef *LPTIMx);
void LPTIM_ITConfig(LPTIM_TypeDef *LPTIMx, uint32_t LPTIM_IT, FunctionalState NewState);
FlagStatus LPTIM_GetFlagStatus(LPTIM_TypeDef *LPTIMx, uint32_t LPTIM_FLAG);
void LPTIM_ClearFlag(LPTIM_TypeDef *LPTIMx, uint32_t LPTIM_FLAG);
ITStatus LPTIM_GetITStatus(LPTIM_TypeDef *LPTIMx, uint32_t LPTIM_IT);
void LPTIM_ClearITPendingBit(LPTIM_TypeDef *LPTIMx, uint32_t LPTIM_IT);


#ifdef __cplusplus
}
#endif

#endif
