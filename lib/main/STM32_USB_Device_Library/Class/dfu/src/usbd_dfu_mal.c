/**
  ******************************************************************************
  * @file    usbd_dfu_mal.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   Generic media access Layer.
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
#include "usbd_dfu_mal.h"

#include "usbd_flash_if.h"

#ifdef DFU_MAL_SUPPORT_OTP
 #include "usbd_otp_if.h"
#endif

#ifdef DFU_MAL_SUPPORT_MEM
 #include "usbd_mem_if_template.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global Memories callback and string descriptors reference tables.
   To add a new memory, modify the value of MAX_USED_MEDIA in usbd_dfu_mal.h
   and add the pointer to the callback structure in this table.
   Then add the pointer to the memory string descriptor in usbd_dfu_StringDesc table.
   No other operation is required. */
DFU_MAL_Prop_TypeDef* tMALTab[MAX_USED_MEDIA] = {
    &DFU_Flash_cb
#ifdef DFU_MAL_SUPPORT_OTP
  , &DFU_Otp_cb
#endif
#ifdef DFU_MAL_SUPPORT_MEM
  , &DFU_Mem_cb
#endif
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN const uint8_t* usbd_dfu_StringDesc[MAX_USED_MEDIA] __ALIGN_END  = {
    FLASH_IF_STRING
#ifdef DFU_MAL_SUPPORT_OTP
  , OTP_IF_STRING
#endif
#ifdef DFU_MAL_SUPPORT_MEM
  , MEM_IF_STRING
#endif
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* RAM Buffer for Downloaded Data */
__ALIGN_BEGIN uint8_t  MAL_Buffer[XFERSIZE] __ALIGN_END ; 

/* Private function prototypes -----------------------------------------------*/
static uint8_t  MAL_CheckAdd  (uint32_t Add);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  MAL_Init
  *         Initializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (MAL_OK in all cases)
  */
uint16_t MAL_Init(void)
{
  uint32_t memIdx = 0;
  
  /* Init all supported memories */
  for(memIdx = 0; memIdx < MAX_USED_MEDIA; memIdx++)
  {
    /* If the check addres is positive, exit with the memory index */
    if (tMALTab[memIdx]->pMAL_Init != NULL)
    {
      tMALTab[memIdx]->pMAL_Init();
    }
  }

  return MAL_OK;
}

/**
  * @brief  MAL_DeInit
  *         DeInitializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (MAL_OK in all cases)
  */
uint16_t MAL_DeInit(void)
{
  uint32_t memIdx = 0;
  
  /* Init all supported memories */
  for(memIdx = 0; memIdx < MAX_USED_MEDIA; memIdx++)
  {
    /* Check if the command is supported */
    if (tMALTab[memIdx]->pMAL_DeInit != NULL)
    {
      tMALTab[memIdx]->pMAL_DeInit();
    }
  }

  return MAL_OK;
}

/**
  * @brief  MAL_Erase
  *         Erase a sector of memory.
  * @param  Add: Sector address/code
  * @retval Result of the opeartion: MAL_OK if all operations are OK else MAL_FAIL
  */
uint16_t MAL_Erase(uint32_t Add)
{
  uint32_t memIdx = MAL_CheckAdd(Add);
 
  /* Check if the area is protected */
  if (DFU_MAL_IS_PROTECTED_AREA(Add))
  {
    return MAL_FAIL;
  }    
  
  if (memIdx < MAX_USED_MEDIA)
  {
    /* Check if the command is supported */
    if (tMALTab[memIdx]->pMAL_Erase != NULL)
    {
      return tMALTab[memIdx]->pMAL_Erase(Add);
    }
    else
    {
      return MAL_FAIL;
    }
  }
  else
  {
    return MAL_FAIL;
  }
}

/**
  * @brief  MAL_Write
  *         Write sectors of memory.
  * @param  Add: Sector address/code
  * @param  Len: Number of data to be written (in bytes)
  * @retval Result of the opeartion: MAL_OK if all operations are OK else MAL_FAIL
  */
uint16_t MAL_Write (uint32_t Add, uint32_t Len)
{
  uint32_t memIdx = MAL_CheckAdd(Add);
 
  /* Check if the area is protected */
  if (DFU_MAL_IS_PROTECTED_AREA(Add))
  {
    return MAL_FAIL;
  }   
  
  if (memIdx < MAX_USED_MEDIA)
  {
    /* Check if the command is supported */
    if (tMALTab[memIdx]->pMAL_Write != NULL)
    {
      return tMALTab[memIdx]->pMAL_Write(Add, Len);
    }
    else
    {
      return MAL_FAIL;
    }    
  }
  else
  {
    return MAL_FAIL;
  }
}

/**
  * @brief  MAL_Read
  *         Read sectors of memory.
  * @param  Add: Sector address/code
  * @param  Len: Number of data to be written (in bytes)
  * @retval Buffer pointer
  */
uint8_t *MAL_Read (uint32_t Add, uint32_t Len)
{
  uint32_t memIdx = MAL_CheckAdd(Add);
  
  if (memIdx < MAX_USED_MEDIA)
  {
    /* Check if the command is supported */
    if (tMALTab[memIdx]->pMAL_Read != NULL)
    {
      return tMALTab[memIdx]->pMAL_Read(Add, Len);
    }
    else
    {
      return MAL_Buffer;
    }     
  }
  else
  {
    return MAL_Buffer;
  }
}

/**
  * @brief  MAL_GetStatus
  *         Get the status of a given memory.
  * @param  Add: Sector address/code (allow to determine which memory will be addressed)
  * @param  Cmd: 0 for erase and 1 for write
  * @param  buffer: pointer to the buffer where the status data will be stored.
  * @retval Buffer pointer
  */
uint16_t MAL_GetStatus(uint32_t Add , uint8_t Cmd, uint8_t *buffer)
{
  uint32_t memIdx = MAL_CheckAdd(Add);
  
  if (memIdx < MAX_USED_MEDIA)
  {
    if (Cmd & 0x01)
    {
      SET_POLLING_TIMING(tMALTab[memIdx]->EraseTiming);
    }
    else
    {
      SET_POLLING_TIMING(tMALTab[memIdx]->WriteTiming);
    }
    
    return MAL_OK;
  }
  else
  {
    return MAL_FAIL;
  }
}

/**
  * @brief  MAL_CheckAdd
  *         Determine which memory should be managed.
  * @param  Add: Sector address/code (allow to determine which memory will be addressed)
  * @retval Index of the addressed memory.
  */
static uint8_t  MAL_CheckAdd(uint32_t Add)
{
  uint32_t memIdx = 0;
  
  /* Check with all supported memories */
  for(memIdx = 0; memIdx < MAX_USED_MEDIA; memIdx++)
  {
    /* If the check addres is positive, exit with the memory index */
    if (tMALTab[memIdx]->pMAL_CheckAdd(Add) == MAL_OK)
    {
      return memIdx;
    }
  }
  /* If no memory found, return MAX_USED_MEDIA */
  return (MAX_USED_MEDIA);
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
