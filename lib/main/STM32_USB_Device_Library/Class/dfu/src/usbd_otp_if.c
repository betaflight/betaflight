/**
  ******************************************************************************
  * @file    usbd_otp_if.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   Specific media access Layer for OTP (One Time Programming) memory.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_otp_if.h"
#include "usbd_dfu_mal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
uint16_t OTP_If_Write (uint32_t Add, uint32_t Len);
uint8_t *OTP_If_Read  (uint32_t Add, uint32_t Len);
uint16_t OTP_If_DeInit(void);
uint16_t OTP_If_CheckAdd(uint32_t Add);


/* Private variables ---------------------------------------------------------*/
DFU_MAL_Prop_TypeDef DFU_Otp_cb =
  {
    OTP_IF_STRING,
    NULL, /* Init not supported*/
    NULL, /* DeInit not supported */
    NULL, /* Erase not supported */
    OTP_If_Write,
    OTP_If_Read,
    OTP_If_CheckAdd,
    1,  /* Erase Time in ms */
    10  /* Programming Time in ms */
  };
  
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  OTP_If_Write
  *         Memory write routine.
  * @param  Add: Address to be written to.
  * @param  Len: Number of data to be written (in bytes).
  * @retval MAL_OK if operation is successeful, MAL_FAIL else.
  */
uint16_t OTP_If_Write(uint32_t Add, uint32_t Len)
{
  uint32_t idx = 0;
  
  if  (Len & 0x3) /* Not an aligned data */
  {
    for (idx = Len; idx < ((Len & 0xFFFC) + 4); idx++)
    {
      MAL_Buffer[idx] = 0xFF;
    }
  }
  
  /* Data received are Word multiple */
  for (idx = 0; idx <  Len; idx = idx + 4)
  {
    FLASH_ProgramWord(Add, *(uint32_t *)(MAL_Buffer + idx));
    Add += 4;
  }
  return MAL_OK;
}

/**
  * @brief  OTP_If_Read
  *         Memory read routine.
  * @param  Add: Address to be read from.
  * @param  Len: Number of data to be read (in bytes).
  * @retval Pointer to the phyisical address where data should be read.
  */
uint8_t *OTP_If_Read (uint32_t Add, uint32_t Len)
{
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  uint32_t idx = 0;
  for (idx = 0; idx < Len; idx += 4)
  {
    *(uint32_t*)(MAL_Buffer + idx) = *(uint32_t *)(Add + idx);
  }
  return (uint8_t*)(MAL_Buffer);
#else
  return  (uint8_t*)(Add);
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
}

/**
  * @brief  OTP_If_CheckAdd
  *         Check if the address is an allowed address for this memory.
  * @param  Add: Address to be checked.
  * @param  Len: Number of data to be read (in bytes).
  * @retval MAL_OK if the address is allowed, MAL_FAIL else.
  */
uint16_t OTP_If_CheckAdd(uint32_t Add)
{
  if ((Add >= OTP_START_ADD) && (Add < OTP_END_ADD))
  {
    return MAL_OK;
  }
  else
  {
    return MAL_FAIL;
  }
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
