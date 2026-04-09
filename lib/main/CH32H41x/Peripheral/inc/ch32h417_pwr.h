/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_pwr.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the PWR  
*                      firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_PWR_H
#define __CH32H417_PWR_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

/* PVD_detection_level  */
#define PWR_PVDLevel_MODE0          ((uint32_t)0x00000000)
#define PWR_PVDLevel_MODE1          ((uint32_t)0x00000020)
#define PWR_PVDLevel_MODE2          ((uint32_t)0x00000040)
#define PWR_PVDLevel_MODE3          ((uint32_t)0x00000060)
#define PWR_PVDLevel_MODE4          ((uint32_t)0x00000080)
#define PWR_PVDLevel_MODE5          ((uint32_t)0x000000A0)
#define PWR_PVDLevel_MODE6          ((uint32_t)0x000000C0)
#define PWR_PVDLevel_MODE7          ((uint32_t)0x000000E0)

/* PWR_detection_level  */
#define PWR_VIO18Level_MODE0        ((uint32_t)0x00000000)
#define PWR_VIO18Level_MODE1        ((uint32_t)0x00000400)
#define PWR_VIO18Level_MODE2        ((uint32_t)0x00000800)
#define PWR_VIO18Level_MODE3        ((uint32_t)0x00000C00)
#define PWR_VIO18Level_MODE4        ((uint32_t)0x00001000)
#define PWR_VIO18Level_MODE5        ((uint32_t)0x00001400)

/* Regulator_state_is_STOP_mode */
#define PWR_Regulator_ON            ((uint32_t)0x00000000)
#define PWR_Regulator_LowPower      ((uint32_t)0x00000001)

/* STOP_mode_entry */
#define PWR_STOPEntry_WFI           ((uint8_t)0x01)
#define PWR_STOPEntry_WFE           ((uint8_t)0x02)
 
/* PWR_Flag */
#define PWR_FLAG_PVDO               ((uint32_t)0x00000001)

/* PWR_VIO18CFGMODE */
#define PWR_VIO18CFGMODE_HW         ((uint32_t)0x00000000)
#define PWR_VIO18CFGMODE_SW         ((uint32_t)0x00000200)

/* VIO18InitialStatus */
typedef enum
{
    PWR_VIO18InitialStatus_0 = 0,
    PWR_VIO18InitialStatus_1,
    PWR_VIO18InitialStatus_2,
    PWR_VIO18InitialStatus_3  
}PWR_VIO18InitialStatus;


void PWR_DeInit(void);
void PWR_BackupAccessCmd(FunctionalState NewState);
void PWR_PVDCmd(FunctionalState NewState);
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_VIO18ModeCfg(uint32_t PWR_VIO18CfgMode);
void PWR_VIO18LevelCfg(uint16_t VIO18Level);
PWR_VIO18InitialStatus PWR_GetVIO18InitialStatus(void);

#ifdef __cplusplus
}
#endif

#endif 

