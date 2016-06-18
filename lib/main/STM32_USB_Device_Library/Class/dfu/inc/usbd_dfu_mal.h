/**
  ******************************************************************************
  * @file    usbd_dfu_mal.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   Header for usbd_dfu_mal.c file.
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
#ifndef __DFU_MAL_H
#define __DFU_MAL_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32F2XX
 #include "stm32f2xx.h"
#elif defined(STM32F10X_CL)
 #include "stm32f10x.h"
#endif /* STM32F2XX */

#include "usbd_conf.h"
#include "usbd_dfu_core.h"

/* Exported types ------------------------------------------------------------*/
typedef struct _DFU_MAL_PROP
{
  const uint8_t* pStrDesc;
  uint16_t (*pMAL_Init)     (void);   
  uint16_t (*pMAL_DeInit)   (void);   
  uint16_t (*pMAL_Erase)    (uint32_t Add);
  uint16_t (*pMAL_Write)    (uint32_t Add, uint32_t Len);
  uint8_t  *(*pMAL_Read)    (uint32_t Add, uint32_t Len);
  uint16_t (*pMAL_CheckAdd) (uint32_t Add);
  const uint32_t EraseTiming;
  const uint32_t WriteTiming;
}
DFU_MAL_Prop_TypeDef;


/* Exported constants --------------------------------------------------------*/
#define MAL_OK                          0
#define MAL_FAIL                        1

/* utils macro ---------------------------------------------------------------*/
#define _1st_BYTE(x)  (uint8_t)((x)&0xFF)             /* 1st addressing cycle */
#define _2nd_BYTE(x)  (uint8_t)(((x)&0xFF00)>>8)      /* 2nd addressing cycle */
#define _3rd_BYTE(x)  (uint8_t)(((x)&0xFF0000)>>16)   /* 3rd addressing cycle */
#define _4th_BYTE(x)  (uint8_t)(((x)&0xFF000000)>>24) /* 4th addressing cycle */

/* Exported macro ------------------------------------------------------------*/
#define SET_POLLING_TIMING(x)   buffer[1] = _1st_BYTE(x);\
                                buffer[2] = _2nd_BYTE(x);\
                                buffer[3] = _3rd_BYTE(x);  

/* Exported functions ------------------------------------------------------- */

uint16_t MAL_Init (void);
uint16_t MAL_DeInit (void);
uint16_t MAL_Erase (uint32_t SectorAddress);
uint16_t MAL_Write (uint32_t SectorAddress, uint32_t DataLength);
uint8_t *MAL_Read  (uint32_t SectorAddress, uint32_t DataLength);
uint16_t MAL_GetStatus(uint32_t SectorAddress ,uint8_t Cmd, uint8_t *buffer);

extern uint8_t  MAL_Buffer[XFERSIZE]; /* RAM Buffer for Downloaded Data */
#endif /* __DFU_MAL_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
