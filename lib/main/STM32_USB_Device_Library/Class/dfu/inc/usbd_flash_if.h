/**
  ******************************************************************************
  * @file    usbd_flash_if.h
  * @author  MCD Application Team
  * @version V1.0.0RC1
  * @date    18-March-2011
  * @brief   Header for usbd_flash_if.c file.
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
#ifndef __FLASH_IF_MAL_H
#define __FLASH_IF_MAL_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_dfu_mal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define FLASH_START_ADD                  0x08000000

#ifdef STM32F2XX
 #define FLASH_END_ADD                   0x08100000
 #define FLASH_IF_STRING                 "@Internal Flash   /0x08000000/03*016Ka,01*016Kg,01*064Kg,07*128Kg"
#elif defined(STM32F10X_CL)
 #define FLASH_END_ADD                   0x08040000
 #define FLASH_IF_STRING                 "@Internal Flash   /0x08000000/06*002Ka,122*002Kg"  
#endif /* STM32F2XX */


extern DFU_MAL_Prop_TypeDef DFU_Flash_cb;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __FLASH_IF_MAL_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
