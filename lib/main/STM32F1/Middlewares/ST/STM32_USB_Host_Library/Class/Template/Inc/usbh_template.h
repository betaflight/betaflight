/**
  ******************************************************************************
  * @file    usbh_template.h
  * @author  MCD Application Team
  * @version V3.2.2
  * @date    07-July-2015
  * @brief   This file contains all the prototypes for the usbh_template.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive  ----------------------------------------------*/
#ifndef __USBH_TEMPLATE_H
#define __USBH_TEMPLATE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbh_core.h"


/** @addtogroup USBH_LIB
* @{
*/

/** @addtogroup USBH_CLASS
* @{
*/

/** @addtogroup USBH_TEMPLATE_CLASS
* @{
*/

/** @defgroup USBH_TEMPLATE_CLASS
* @brief This file is the Header file for usbh_template.c
* @{
*/ 


/**
  * @}
  */ 

/** @defgroup USBH_TEMPLATE_CLASS_Exported_Types
* @{
*/ 

/* States for TEMPLATE State Machine */


/**
* @}
*/ 

/** @defgroup USBH_TEMPLATE_CLASS_Exported_Defines
* @{
*/ 

/**
* @}
*/ 

/** @defgroup USBH_TEMPLATE_CLASS_Exported_Macros
* @{
*/ 
/**
* @}
*/ 

/** @defgroup USBH_TEMPLATE_CLASS_Exported_Variables
* @{
*/ 
extern USBH_ClassTypeDef  TEMPLATE_Class;
#define USBH_TEMPLATE_CLASS    &TEMPLATE_Class

/**
* @}
*/ 

/** @defgroup USBH_TEMPLATE_CLASS_Exported_FunctionsPrototype
* @{
*/ 
USBH_StatusTypeDef USBH_TEMPLATE_IOProcess (USBH_HandleTypeDef *phost);
USBH_StatusTypeDef USBH_TEMPLATE_Init (USBH_HandleTypeDef *phost);
/**
* @}
*/ 

#ifdef __cplusplus
}
#endif

#endif /* __USBH_TEMPLATE_H */

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
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

