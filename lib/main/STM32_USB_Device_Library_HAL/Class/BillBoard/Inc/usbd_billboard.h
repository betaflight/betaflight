/**
  ******************************************************************************
  * @file    usbd_billboard.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_billboard.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_BB_H
#define __USB_BB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"
#include  "usbd_desc.h"
/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_BB
  * @brief This file is the Header file for usbd_billboard.c
  * @{
  */

/** @defgroup USBD_BB_Exported_Defines
  * @{
  */
#define USB_BB_CONFIG_DESC_SIZ                  18U

#ifndef USB_BB_MAX_NUM_ALT_MODE
#define USB_BB_MAX_NUM_ALT_MODE                 0x2U
#endif /* USB_BB_MAX_NUM_ALT_MODE */

#ifndef USBD_BB_IF_STRING_INDEX
#define USBD_BB_IF_STRING_INDEX                 0x10U
#endif /* USBD_BB_IF_STRING_INDEX */


#define USBD_BILLBOARD_CAPABILITY               0x0DU
#define USBD_BILLBOARD_ALTMODE_CAPABILITY       0x0FU

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef  struct  _BB_DescHeader
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bDevCapabilityType;
}
USBD_BB_DescHeader_t;

typedef struct
{
  uint16_t wSVID;
  uint8_t  bAlternateMode;
  uint8_t  iAlternateModeString;
} USBD_BB_AltModeTypeDef;

typedef struct
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bDevCapabilityType;
  uint8_t   bIndex;
  uint32_t  dwAlternateModeVdo;
} USBD_BB_AltModeCapDescTypeDef;

typedef  struct
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bDevCapabilityType;
  uint8_t   iAddtionalInfoURL;
  uint8_t   bNbrOfAltModes;
  uint8_t   bPreferredAltMode;
  uint16_t  VconnPwr;
  uint8_t   bmConfigured[32];
  uint16_t  bcdVersion;
  uint8_t   bAdditionalFailureInfo;
  uint8_t   bReserved;
  USBD_BB_AltModeTypeDef  wSVID[USB_BB_MAX_NUM_ALT_MODE];
} USBD_BosBBCapDescTypedef;

typedef enum
{
  UNSPECIFIED_ERROR = 0,
  CONFIGURATION_NOT_ATTEMPTED,
  CONFIGURATION_UNSUCCESSFUL,
  CONFIGURATION_SUCCESSFUL,
} BB_AltModeState;

/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_BB;
#define USBD_BB_CLASS    &USBD_BB
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */

#if (USBD_CLASS_BOS_ENABLED == 1)
void *USBD_BB_GetCapDesc(USBD_HandleTypeDef *pdev, uint8_t *buf);
void *USBD_BB_GetAltModeDesc(USBD_HandleTypeDef *pdev, uint8_t *buf, uint8_t idx);
#endif

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_BB_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
