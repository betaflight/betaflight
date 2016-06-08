/**
  ******************************************************************************
  * @file    usbd_msc_core.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   header for the usbd_msc_core.c file
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
#ifndef _USB_MSC_CORE_H_
#define _USB_MSC_CORE_H_

#include  "usbd_ioreq.h"

/** @addtogroup USBD_MSC_BOT
  * @{
  */
  
/** @defgroup USBD_MSC
  * @brief This file is the Header file for USBD_msc.c
  * @{
  */ 


/** @defgroup USBD_BOT_Exported_Defines
  * @{
  */ 


#define BOT_GET_MAX_LUN              0xFE
#define BOT_RESET                    0xFF
#define USB_MSC_CONFIG_DESC_SIZ      32

#define MSC_EPIN_SIZE  *(uint16_t *)(((USB_OTG_CORE_HANDLE *)pdev)->dev.pConfig_descriptor + 22)
        
#define MSC_EPOUT_SIZE *(uint16_t *)(((USB_OTG_CORE_HANDLE *)pdev)->dev.pConfig_descriptor + 29)

/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Types
  * @{
  */ 

extern USBD_Class_cb_TypeDef  USBD_MSC_cb;
/**
  * @}
  */ 

/**
  * @}
  */ 
#endif  // _USB_MSC_CORE_H_
/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
