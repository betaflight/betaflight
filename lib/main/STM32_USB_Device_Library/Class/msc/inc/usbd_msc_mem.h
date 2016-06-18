/**
  ******************************************************************************
  * @file    usbd_msc_mem.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   header for the STORAGE DISK file file
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __USBD_MEM_H
#define __USBD_MEM_H
/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"


/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USBD_MEM
  * @brief header file for the storage disk file
  * @{
  */ 

/** @defgroup USBD_MEM_Exported_Defines
  * @{
  */ 
#define USBD_STD_INQUIRY_LENGTH		36
/**
  * @}
  */ 


/** @defgroup USBD_MEM_Exported_TypesDefinitions
  * @{
  */

typedef struct _USBD_STORAGE
{
  int8_t (* Init) (uint8_t lun);
  int8_t (* GetCapacity) (uint8_t lun, uint32_t *block_num, uint32_t *block_size);
  int8_t (* IsReady) (uint8_t lun);
  int8_t (* IsWriteProtected) (uint8_t lun);
  int8_t (* Read) (uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* GetMaxLun)(void);
  int8_t *pInquiry;
  
}USBD_STORAGE_cb_TypeDef;
/**
  * @}
  */ 



/** @defgroup USBD_MEM_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_MEM_Exported_Variables
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_MEM_Exported_FunctionsPrototype
  * @{
  */ 
extern USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops;
/**
  * @}
  */ 

#endif /* __USBD_MEM_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
* @}
*/ 
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
