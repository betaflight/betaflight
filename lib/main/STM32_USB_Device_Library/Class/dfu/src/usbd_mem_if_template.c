/**
  ******************************************************************************
  * @file    usbd_mem_if_template.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   Specific media access Layer for a template memory. This file is 
             provided as template example showing how to implement a new memory
             interface based on pre-defined API.
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
#include "usbd_mem_if_template.h"
#include "usbd_dfu_mal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
uint16_t MEM_If_Init(void);
uint16_t MEM_If_Erase (uint32_t Add);
uint16_t MEM_If_Write (uint32_t Add, uint32_t Len);
uint8_t *MEM_If_Read  (uint32_t Add, uint32_t Len);
uint16_t MEM_If_DeInit(void);
uint16_t MEM_If_CheckAdd(uint32_t Add);


/* Private variables ---------------------------------------------------------*/
DFU_MAL_Prop_TypeDef DFU_Mem_cb =
  {
    MEM_IF_STRING,
    MEM_If_Init,
    MEM_If_DeInit,
    MEM_If_Erase,
    MEM_If_Write,
    MEM_If_Read,
    MEM_If_CheckAdd,
    10, /* Erase Time in ms */
    10  /* Programming Time in ms */
  };
  
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  MEM_If_Init
  *         Memory initialization routine.
  * @param  None
  * @retval MAL_OK if operation is successeful, MAL_FAIL else.
  */
uint16_t MEM_If_Init(void)
{ 
  return MAL_OK;
}

/**
  * @brief  MEM_If_DeInit
  *         Memory deinitialization routine.
  * @param  None
  * @retval MAL_OK if operation is successeful, MAL_FAIL else.
  */
uint16_t MEM_If_DeInit(void)
{ 
  return MAL_OK;
}

/**
  * @brief  MEM_If_Erase
  *         Erase sector.
  * @param  Add: Address of sector to be erased.
  * @retval MAL_OK if operation is successeful, MAL_FAIL else.
  */
uint16_t MEM_If_Erase(uint32_t Add)
{
  return MAL_OK;
}

/**
  * @brief  MEM_If_Write
  *         Memory write routine.
  * @param  Add: Address to be written to.
  * @param  Len: Number of data to be written (in bytes).
  * @retval MAL_OK if operation is successeful, MAL_FAIL else.
  */
uint16_t MEM_If_Write(uint32_t Add, uint32_t Len)
{
  return MAL_OK;
}

/**
  * @brief  MEM_If_Read
  *         Memory read routine.
  * @param  Add: Address to be read from.
  * @param  Len: Number of data to be read (in bytes).
  * @retval Pointer to the phyisical address where data should be read.
  */
uint8_t *MEM_If_Read (uint32_t Add, uint32_t Len)
{
  /* Return a valid address to avoid HardFault */
  return  (uint8_t*)(MAL_Buffer); 
}

/**
  * @brief  MEM_If_CheckAdd
  *         Check if the address is an allowed address for this memory.
  * @param  Add: Address to be checked.
  * @param  Len: Number of data to be read (in bytes).
  * @retval MAL_OK if the address is allowed, MAL_FAIL else.
  */
uint16_t MEM_If_CheckAdd(uint32_t Add)
{
  if ((Add >= MEM_START_ADD) && (Add < MEM_END_ADD))
  {
    return MAL_OK;
  }
  else
  {
    return MAL_FAIL;
  }
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
