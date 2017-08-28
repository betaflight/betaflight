/**
  ******************************************************************************
  * @file    usbh_hid_parser.c 
  * @author  MCD Application Team
  * @version V3.2.2
  * @date    07-July-2015
  * @brief   This file is the header file of the usbh_hid_parser.c        
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

/* Define to prevent recursive -----------------------------------------------*/ 
#ifndef __USBH_HID_PARSER_H
#define __USBH_HID_PARSER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbh_hid.h"
#include "usbh_hid_usage.h"

/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_CLASS
  * @{
  */

/** @addtogroup USBH_HID_CLASS
  * @{
  */
  
/** @defgroup USBH_HID_PARSER
  * @brief This file is the Header file for usbh_hid_parser.c
  * @{
  */ 


/** @defgroup USBH_HID_PARSER_Exported_Types
  * @{
  */ 
typedef struct 
{
  uint8_t  *data;
  uint32_t size;
  uint8_t  shift;
  uint8_t  count;
  uint8_t  sign;
  uint32_t logical_min;  /*min value device can return*/
  uint32_t logical_max;  /*max value device can return*/
  uint32_t physical_min; /*min vale read can report*/
  uint32_t physical_max; /*max value read can report*/
  uint32_t resolution;
} 
HID_Report_ItemTypedef;


uint32_t HID_ReadItem (HID_Report_ItemTypedef *ri, uint8_t ndx);
uint32_t HID_WriteItem(HID_Report_ItemTypedef *ri, uint32_t value, uint8_t ndx);


/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __USBH_HID_PARSER_H */

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
