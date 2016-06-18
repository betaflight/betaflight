/**
  ******************************************************************************
  * @file    usbd_mem_if_template.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   Header for usbd_mem_if_template.c file.
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
#ifndef __MEM_IF_MAL_H
#define __MEM_IF_MAL_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32F2XX
  #include "stm32f2xx.h"
#endif /* STM32F2XX */
#include "usbd_dfu_mal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define MEM_START_ADD                 0x00000000 /* Dummy start address */
#define MEM_END_ADD                   (uint32_t)(MEM_START_ADD + (5 * 1024)) /* Dummy Size = 5KB */

#define MEM_IF_STRING                 "@Dummy Memory   /0x00000000/01*002Kg,03*001Kg"

extern DFU_MAL_Prop_TypeDef DFU_Mem_cb;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MEM_IF_MAL_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
