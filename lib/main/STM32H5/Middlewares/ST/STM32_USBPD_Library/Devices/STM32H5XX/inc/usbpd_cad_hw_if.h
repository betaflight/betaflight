/**
  ******************************************************************************
  * @file    usbpd_cad_hw_if.h
  * @author  MCD Application Team
  * @brief   This file contains the headers of usbpd_cad_hw_if.c for Cable Attach-Detach
  *          controls.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef __USBPD_CAD_HW_IF_H_
#define __USBPD_CAD_HW_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbpd_core.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_CAD_HW_IF
  * @{
  */


/* Exported types ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#if defined(_LOW_POWER) || defined(USBPDM1_VCC_FEATURE_ENABLED)
#define CAD_DELAY_READ_CC_STATUS         (300U)
#endif /* _LOW_POWER || USBPDM1_VCC_FEATURE_ENABLED */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#if defined(USBPDCORE_DRP) || defined(USBPDCORE_SRC)
/* Keep for legacy */
uint32_t                          CAD_Set_ResistorRp(uint8_t PortNum, CAD_RP_Source_Current_Adv_Typedef RpValue);
#endif /* USBPDCORE_DRP || USBPDCORE_SRC */

/** @addtogroup USBPD_DEVICE_CAD_HW_IF_Exported_Functions
  * @{
  */
void                              CAD_Init(uint8_t PortNum, USBPD_SettingsTypeDef *Settings,
                                           USBPD_ParamsTypeDef *Params, void (*PtrWakeUp)(void));
uint32_t                          CAD_StateMachine(uint8_t PortNum, USBPD_CAD_EVENT *Event, CCxPin_TypeDef *CCXX);
void                              CAD_Enter_ErrorRecovery(uint8_t PortNum);
#if defined(USBPDCORE_DRP) || defined(USBPDCORE_SRC)
uint32_t                          CAD_SRC_Set_ResistorRp(uint8_t PortNum, CAD_RP_Source_Current_Adv_Typedef RpValue);
#endif /* USBPDCORE_DRP || USBPDCORE_SRC */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_CAD_HW_IF_H_ */

