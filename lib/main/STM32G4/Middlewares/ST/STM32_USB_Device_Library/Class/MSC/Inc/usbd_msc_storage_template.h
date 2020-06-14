/**
  ******************************************************************************
  * @file    usbd_msc_storage.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_msc_storage.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_MSC_STORAGE_H
#define __USBD_MSC_STORAGE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief header file for the usbd_msc_storage.c file
  * @{
  */

/** @defgroup USBD_STORAGE_Exported_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_STORAGE_Exported_Types
  * @{
  */


/**
  * @}
  */



/** @defgroup USBD_STORAGE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @{
  */
extern USBD_StorageTypeDef  USBD_MSC_Template_fops;
/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_FunctionsPrototype
  * @{
  */


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_MSC_STORAGE_H */

/**
  * @}
  */

/**
* @}
*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
