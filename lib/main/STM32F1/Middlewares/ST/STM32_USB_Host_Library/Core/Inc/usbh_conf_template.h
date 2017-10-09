/**
  ******************************************************************************
  * @file    usbh_conf_template.h
  * @author  MCD Application Team
  * @version V3.2.2
  * @date    07-July-2015
  * @brief   Header file for usbh_conf_template.c
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBH_CONF_TEMPLATE_H
#define __USBH_CONF_TEMPLATE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup USBH_OTG_DRIVER
  * @{
  */
  
/** @defgroup USBH_CONF
  * @brief usb otg low level driver configuration file
  * @{
  */ 

/** @defgroup USBH_CONF_Exported_Defines
  * @{
  */ 

#define USBH_MAX_NUM_ENDPOINTS                2
#define USBH_MAX_NUM_INTERFACES               2
#define USBH_MAX_NUM_CONFIGURATION            1
#define USBH_KEEP_CFG_DESCRIPTOR              1
#define USBH_MAX_NUM_SUPPORTED_CLASS          1
#define USBH_MAX_SIZE_CONFIGURATION           0x200
#define USBH_MAX_DATA_BUFFER                  0x200
#define USBH_DEBUG_LEVEL                      2
#define USBH_USE_OS                           1
    
/** @defgroup USBH_Exported_Macros
  * @{
  */ 

 /* Memory management macros */   
#define USBH_malloc               malloc
#define USBH_free                 free
#define USBH_memset               memset
#define USBH_memcpy               memcpy
    
 /* DEBUG macros */  

  
#if (USBH_DEBUG_LEVEL > 0)
#define  USBH_UsrLog(...)   printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBH_UsrLog(...)   
#endif 
                            
                            
#if (USBH_DEBUG_LEVEL > 1)

#define  USBH_ErrLog(...)   printf("ERROR: ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBH_ErrLog(...)   
#endif 
                            
                            
#if (USBH_DEBUG_LEVEL > 2)                         
#define  USBH_DbgLog(...)   printf("DEBUG : ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBH_DbgLog(...)                         
#endif
                            
/**
  * @}
  */ 
   
/**
  * @}
  */ 


/** @defgroup USBH_CONF_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBH_CONF_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_CONF_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_CONF_Exported_FunctionsPrototype
  * @{
  */ 
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __USBH_CONF_TEMPLATE_H */


/**
  * @}
  */ 

/**
  * @}
  */ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

