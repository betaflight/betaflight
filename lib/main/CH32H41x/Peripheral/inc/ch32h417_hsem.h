/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_hsem.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the HSEM 
*                      firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_HSEM_H
#define __CH32H417_HSEM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

/* HSEM_ID_enumeration */
typedef enum
{
    HSEM_ID0 = 0,
    HSEM_ID1,
    HSEM_ID2,
    HSEM_ID3,
    HSEM_ID4,
    HSEM_ID5,
    HSEM_ID6,
    HSEM_ID7,
    HSEM_ID8,
    HSEM_ID9,
    HSEM_ID10,
    HSEM_ID11,
    HSEM_ID12,
    HSEM_ID13,
    HSEM_ID14,
    HSEM_ID15,
    HSEM_ID16,
    HSEM_ID17,
    HSEM_ID18,
    HSEM_ID19,
    HSEM_ID20,
    HSEM_ID21,
    HSEM_ID22,
    HSEM_ID23,
    HSEM_ID24,
    HSEM_ID25,
    HSEM_ID26,
    HSEM_ID27,
    HSEM_ID28,
    HSEM_ID29,
    HSEM_ID30,
    HSEM_ID31
} HSEM_ID_TypeDef;

/* HSEM_Core_ID */
#define HSEM_Core_ID_V3F   ((uint32_t)0x00000000)
#define HSEM_Core_ID_V5F   ((uint32_t)0x00000100)


ErrorStatus HSEM_Take(HSEM_ID_TypeDef HSEM_ID, uint32_t ProcessID);
ErrorStatus HSEM_FastTake(HSEM_ID_TypeDef HSEM_ID);
FunctionalState HSEM_GetOneSemTakenState(HSEM_ID_TypeDef HSEM_ID);
uint32_t HSEM_GetAllSemTakenState(void);
uint32_t HSEM_OwnCoreGetAllSemTakenState(void);
void HSEM_ReleaseOneSem(HSEM_ID_TypeDef HSEM_ID, uint32_t ProcessID);
void HSEM_ReleaseAllSem(void);
void HSEM_ReleaseSem_MatchCID_PID(uint32_t CoreID, uint32_t ProcessID);
void HSEM_ReleaseSem_MatchCID(uint32_t CoreID);
void HSEM_ReleaseSem_MatchPID(uint32_t ProcessID);
void HSEM_SetClearKey(uint32_t Key);
uint32_t HSEM_GetClearKey(void);
void HSEM_ITConfig(HSEM_ID_TypeDef HSEM_ID, FunctionalState NewState);
FlagStatus HSEM_GetFlagStatus(HSEM_ID_TypeDef HSEM_ID);
void HSEM_ClearFlag(HSEM_ID_TypeDef HSEM_ID);
ITStatus HSEM_GetITStatus(HSEM_ID_TypeDef HSEM_ID);
void HSEM_ClearITPendingBit(HSEM_ID_TypeDef HSEM_ID);



#ifdef __cplusplus
}
#endif

#endif 

