/**
  ******************************************************************************
  * @file    usbd_msc_scsi.c
  * @author  MCD Application Team
  * @brief   This file provides all the USBD SCSI layer functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

  /* BSPDependencies
  - "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
  - "stm32xxxxx_{eval}{discovery}_io.c"
  - "stm32xxxxx_{eval}{discovery}{adafruit}_sd.c"
  EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_bot.h"
#include "usbd_msc_scsi.h"
#include "usbd_msc.h"
#include "usbd_msc_data.h"



/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup MSC_SCSI
  * @brief Mass storage SCSI layer module
  * @{
  */

/** @defgroup MSC_SCSI_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup MSC_SCSI_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup MSC_SCSI_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup MSC_SCSI_Private_Variables
  * @{
  */

/**
  * @}
  */


/** @defgroup MSC_SCSI_Private_FunctionPrototypes
  * @{
  */
static int8_t SCSI_TestUnitReady(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Inquiry(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ReadFormatCapacity(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ReadCapacity10(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_RequestSense (USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_StartStopUnit(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ModeSense6 (USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ModeSense10 (USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Write10(USBD_HandleTypeDef  *pdev, uint8_t lun , uint8_t *params);
static int8_t SCSI_Read10(USBD_HandleTypeDef  *pdev, uint8_t lun , uint8_t *params);
static int8_t SCSI_Verify10(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_CheckAddressRange (USBD_HandleTypeDef *pdev, uint8_t lun,
                                      uint32_t blk_offset, uint32_t blk_nbr);

static int8_t SCSI_ProcessRead (USBD_HandleTypeDef *pdev, uint8_t lun);
static int8_t SCSI_ProcessWrite (USBD_HandleTypeDef *pdev, uint8_t lun);
/**
  * @}
  */


/** @defgroup MSC_SCSI_Private_Functions
  * @{
  */


/**
* @brief  SCSI_ProcessCmd
*         Process SCSI commands
* @param  pdev: device instance
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
int8_t SCSI_ProcessCmd(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *cmd)
{
  switch (cmd[0])
  {
  case SCSI_TEST_UNIT_READY:
    SCSI_TestUnitReady(pdev, lun, cmd);
    break;

  case SCSI_REQUEST_SENSE:
    SCSI_RequestSense (pdev, lun, cmd);
    break;
  case SCSI_INQUIRY:
    SCSI_Inquiry(pdev, lun, cmd);
    break;

  case SCSI_START_STOP_UNIT:
    SCSI_StartStopUnit(pdev, lun, cmd);
    break;

  case SCSI_ALLOW_MEDIUM_REMOVAL:
    SCSI_StartStopUnit(pdev, lun, cmd);
    break;

  case SCSI_MODE_SENSE6:
    SCSI_ModeSense6 (pdev, lun, cmd);
    break;

  case SCSI_MODE_SENSE10:
    SCSI_ModeSense10 (pdev, lun, cmd);
    break;

  case SCSI_READ_FORMAT_CAPACITIES:
    SCSI_ReadFormatCapacity(pdev, lun, cmd);
    break;

  case SCSI_READ_CAPACITY10:
    SCSI_ReadCapacity10(pdev, lun, cmd);
    break;

  case SCSI_READ10:
    SCSI_Read10(pdev, lun, cmd);
    break;

  case SCSI_WRITE10:
    SCSI_Write10(pdev, lun, cmd);
    break;

  case SCSI_VERIFY10:
    SCSI_Verify10(pdev, lun, cmd);
    break;

  default:
    SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, INVALID_CDB);
    return -1;
  }
  return 0;
}


/**
* @brief  SCSI_TestUnitReady
*         Process SCSI Test Unit Ready Command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_TestUnitReady(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*)pdev->pClassData;

  /* case 9 : Hi > D0 */
  if (hmsc->cbw.dDataLength != 0U)
  {
    SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);

    return -1;
  }

  if(((USBD_StorageTypeDef *)pdev->pUserData)->IsReady(lun) != 0)
  {
    SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
    hmsc->bot_state = USBD_BOT_NO_DATA;

    return -1;
  }
  hmsc->bot_data_length = 0U;

  return 0;
}

/**
* @brief  SCSI_Inquiry
*         Process Inquiry command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t  SCSI_Inquiry(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  uint8_t* pPage;
  uint16_t len;
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*)pdev->pClassData;

  if (params[1] & 0x01U)/*Evpd is set*/
  {
    len = LENGTH_INQUIRY_PAGE00;
    hmsc->bot_data_length = len;

    while (len)
    {
      len--;
      hmsc->bot_data[len] = MSC_Page00_Inquiry_Data[len];
    }
  }
  else
  {
    pPage = (uint8_t *)(void *)&((USBD_StorageTypeDef *)pdev->pUserData)->pInquiry[lun * STANDARD_INQUIRY_DATA_LEN];
    len = (uint16_t)pPage[4] + 5U;

    if (params[4] <= len)
    {
      len = params[4];
    }
    hmsc->bot_data_length = len;

    while (len)
    {
      len--;
      hmsc->bot_data[len] = pPage[len];
    }
  }

  return 0;
}

/**
* @brief  SCSI_ReadCapacity10
*         Process Read Capacity 10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ReadCapacity10(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*)pdev->pClassData;

  if(((USBD_StorageTypeDef *)pdev->pUserData)->GetCapacity(lun, &hmsc->scsi_blk_nbr, &hmsc->scsi_blk_size) != 0)
  {
    SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
    return -1;
  }
  else
  {

    hmsc->bot_data[0] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 24);
    hmsc->bot_data[1] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 16);
    hmsc->bot_data[2] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >>  8);
    hmsc->bot_data[3] = (uint8_t)(hmsc->scsi_blk_nbr - 1U);

    hmsc->bot_data[4] = (uint8_t)(hmsc->scsi_blk_size >>  24);
    hmsc->bot_data[5] = (uint8_t)(hmsc->scsi_blk_size >>  16);
    hmsc->bot_data[6] = (uint8_t)(hmsc->scsi_blk_size >>  8);
    hmsc->bot_data[7] = (uint8_t)(hmsc->scsi_blk_size);

    hmsc->bot_data_length = 8U;
    return 0;
  }
}
/**
* @brief  SCSI_ReadFormatCapacity
*         Process Read Format Capacity command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ReadFormatCapacity(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*)pdev->pClassData;

  uint16_t blk_size;
  uint32_t blk_nbr;
  uint16_t i;

  for(i = 0U; i < 12U ; i++)
  {
    hmsc->bot_data[i] = 0U;
  }

  if(((USBD_StorageTypeDef *)pdev->pUserData)->GetCapacity(lun, &blk_nbr, &blk_size) != 0U)
  {
    SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
    return -1;
  }
  else
  {
    hmsc->bot_data[3] = 0x08U;
    hmsc->bot_data[4] = (uint8_t)((blk_nbr - 1U) >> 24);
    hmsc->bot_data[5] = (uint8_t)((blk_nbr - 1U) >> 16);
    hmsc->bot_data[6] = (uint8_t)((blk_nbr - 1U) >>  8);
    hmsc->bot_data[7] = (uint8_t)(blk_nbr - 1U);

    hmsc->bot_data[8] = 0x02U;
    hmsc->bot_data[9] = (uint8_t)(blk_size >>  16);
    hmsc->bot_data[10] = (uint8_t)(blk_size >>  8);
    hmsc->bot_data[11] = (uint8_t)(blk_size);

    hmsc->bot_data_length = 12U;
    return 0;
  }
}
/**
* @brief  SCSI_ModeSense6
*         Process Mode Sense6 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ModeSense6 (USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*)pdev->pClassData;
  uint16_t len = 8U;
  hmsc->bot_data_length = len;

  while (len)
  {
    len--;
    hmsc->bot_data[len] = MSC_Mode_Sense6_data[len];
  }
  return 0;
}

/**
* @brief  SCSI_ModeSense10
*         Process Mode Sense10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ModeSense10 (USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  uint16_t len = 8U;
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*)pdev->pClassData;

  hmsc->bot_data_length = len;

  while (len)
  {
    len--;
    hmsc->bot_data[len] = MSC_Mode_Sense10_data[len];
  }

  return 0;
}

/**
* @brief  SCSI_RequestSense
*         Process Request Sense command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/

static int8_t SCSI_RequestSense (USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  uint8_t i;
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*)pdev->pClassData;

  for(i = 0U ;i < REQUEST_SENSE_DATA_LEN; i++)
  {
    hmsc->bot_data[i] = 0U;
  }

  hmsc->bot_data[0]	= 0x70U;
  hmsc->bot_data[7]	= REQUEST_SENSE_DATA_LEN - 6U;

  if((hmsc->scsi_sense_head != hmsc->scsi_sense_tail)) {

    hmsc->bot_data[2]     = hmsc->scsi_sense[hmsc->scsi_sense_head].Skey;
    hmsc->bot_data[12]    = hmsc->scsi_sense[hmsc->scsi_sense_head].w.b.ASCQ;
    hmsc->bot_data[13]    = hmsc->scsi_sense[hmsc->scsi_sense_head].w.b.ASC;
    hmsc->scsi_sense_head++;

    if (hmsc->scsi_sense_head == SENSE_LIST_DEEPTH)
    {
      hmsc->scsi_sense_head = 0U;
    }
  }
  hmsc->bot_data_length = REQUEST_SENSE_DATA_LEN;

  if (params[4] <= REQUEST_SENSE_DATA_LEN)
  {
    hmsc->bot_data_length = params[4];
  }
  return 0;
}

/**
* @brief  SCSI_SenseCode
*         Load the last error code in the error list
* @param  lun: Logical unit number
* @param  sKey: Sense Key
* @param  ASC: Additional Sense Key
* @retval none

*/
void SCSI_SenseCode(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t sKey, uint8_t ASC)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*)pdev->pClassData;

  hmsc->scsi_sense[hmsc->scsi_sense_tail].Skey  = sKey;
  hmsc->scsi_sense[hmsc->scsi_sense_tail].w.ASC = ASC << 8;
  hmsc->scsi_sense_tail++;
  if (hmsc->scsi_sense_tail == SENSE_LIST_DEEPTH)
  {
    hmsc->scsi_sense_tail = 0U;
  }
}
/**
* @brief  SCSI_StartStopUnit
*         Process Start Stop Unit command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_StartStopUnit(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*) pdev->pClassData;
  hmsc->bot_data_length = 0U;
  return 0;
}

/**
* @brief  SCSI_Read10
*         Process Read10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_Read10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*) pdev->pClassData;

  if(hmsc->bot_state == USBD_BOT_IDLE)  /* Idle */
  {
    /* case 10 : Ho <> Di */
    if ((hmsc->cbw.bmFlags & 0x80U) != 0x80U)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    if(((USBD_StorageTypeDef *)pdev->pUserData)->IsReady(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
      return -1;
    }

    hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) |
                          ((uint32_t)params[3] << 16) |
                          ((uint32_t)params[4] <<  8) |
                           (uint32_t)params[5];

    hmsc->scsi_blk_len =  ((uint32_t)params[7] <<  8) | (uint32_t)params[8];

    if(SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr,
                              hmsc->scsi_blk_len) < 0)
    {
      return -1; /* error */
    }

    hmsc->bot_state = USBD_BOT_DATA_IN;

    /* cases 4,5 : Hi <> Dn */
    if (hmsc->cbw.dDataLength != (hmsc->scsi_blk_len * hmsc->scsi_blk_size))
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }
  }
  hmsc->bot_data_length = MSC_MEDIA_PACKET;

  return SCSI_ProcessRead(pdev, lun);
}

/**
* @brief  SCSI_Write10
*         Process Write10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/

static int8_t SCSI_Write10 (USBD_HandleTypeDef  *pdev, uint8_t lun , uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*) pdev->pClassData;
  uint32_t len;

  if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
  {
    /* case 8 : Hi <> Do */
    if ((hmsc->cbw.bmFlags & 0x80U) == 0x80U)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    /* Check whether Media is ready */
    if(((USBD_StorageTypeDef *)pdev->pUserData)->IsReady(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
      return -1;
    }

    /* Check If media is write-protected */
    if(((USBD_StorageTypeDef *)pdev->pUserData)->IsWriteProtected(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, WRITE_PROTECTED);
      return -1;
    }

    hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) |
                          ((uint32_t)params[3] << 16) |
                          ((uint32_t)params[4] << 8) |
                           (uint32_t)params[5];

    hmsc->scsi_blk_len = ((uint32_t)params[7] << 8) |
                          (uint32_t)params[8];

    /* check if LBA address is in the right range */
    if(SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr,
                              hmsc->scsi_blk_len) < 0)
    {
      return -1; /* error */
    }

    len = hmsc->scsi_blk_len * hmsc->scsi_blk_size;

    /* cases 3,11,13 : Hn,Ho <> D0 */
    if (hmsc->cbw.dDataLength != len)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    len = MIN(len, MSC_MEDIA_PACKET);

    /* Prepare EP to receive first data packet */
    hmsc->bot_state = USBD_BOT_DATA_OUT;
    USBD_LL_PrepareReceive (pdev, MSC_EPOUT_ADDR, hmsc->bot_data, len);
  }
  else /* Write Process ongoing */
  {
    return SCSI_ProcessWrite(pdev, lun);
  }
  return 0;
}


/**
* @brief  SCSI_Verify10
*         Process Verify10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/

static int8_t SCSI_Verify10(USBD_HandleTypeDef  *pdev, uint8_t lun , uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*) pdev->pClassData;

  if ((params[1]& 0x02U) == 0x02U)
  {
    SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
    return -1; /* Error, Verify Mode Not supported*/
  }

  if(SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr,
                            hmsc->scsi_blk_len) < 0)
  {
    return -1; /* error */
  }
  hmsc->bot_data_length = 0U;
  return 0;
}

/**
* @brief  SCSI_CheckAddressRange
*         Check address range
* @param  lun: Logical unit number
* @param  blk_offset: first block address
* @param  blk_nbr: number of block to be processed
* @retval status
*/
static int8_t SCSI_CheckAddressRange (USBD_HandleTypeDef *pdev, uint8_t lun,
                                      uint32_t blk_offset, uint32_t blk_nbr)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = (USBD_MSC_BOT_HandleTypeDef*) pdev->pClassData;

  if ((blk_offset + blk_nbr) > hmsc->scsi_blk_nbr)
  {
    SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, ADDRESS_OUT_OF_RANGE);
    return -1;
  }
  return 0;
}

/**
* @brief  SCSI_ProcessRead
*         Handle Read Process
* @param  lun: Logical unit number
* @retval status
*/
static int8_t SCSI_ProcessRead (USBD_HandleTypeDef  *pdev, uint8_t lun)
{
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef*)pdev->pClassData;
  uint32_t len = hmsc->scsi_blk_len * hmsc->scsi_blk_size;

  len = MIN(len, MSC_MEDIA_PACKET);

  if( ((USBD_StorageTypeDef *)pdev->pUserData)->Read(lun,
                              hmsc->bot_data,
                              hmsc->scsi_blk_addr,
                              (len / hmsc->scsi_blk_size)) < 0)
  {
    SCSI_SenseCode(pdev, lun, HARDWARE_ERROR, UNRECOVERED_READ_ERROR);
    return -1;
  }

  USBD_LL_Transmit (pdev, MSC_EPIN_ADDR, hmsc->bot_data, len);

  hmsc->scsi_blk_addr += (len / hmsc->scsi_blk_size);
  hmsc->scsi_blk_len -= (len / hmsc->scsi_blk_size);

  /* case 6 : Hi = Di */
  hmsc->csw.dDataResidue -= len;

  if (hmsc->scsi_blk_len == 0U)
  {
    hmsc->bot_state = USBD_BOT_LAST_DATA_IN;
  }
  return 0;
}

/**
* @brief  SCSI_ProcessWrite
*         Handle Write Process
* @param  lun: Logical unit number
* @retval status
*/

static int8_t SCSI_ProcessWrite (USBD_HandleTypeDef  *pdev, uint8_t lun)
{
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef*) pdev->pClassData;
  uint32_t len = hmsc->scsi_blk_len * hmsc->scsi_blk_size;

  len = MIN(len, MSC_MEDIA_PACKET);

  if(((USBD_StorageTypeDef *)pdev->pUserData)->Write(lun, hmsc->bot_data,
                             hmsc->scsi_blk_addr,
                             (len / hmsc->scsi_blk_size)) < 0)
  {
    SCSI_SenseCode(pdev, lun, HARDWARE_ERROR, WRITE_FAULT);

    return -1;
  }

  hmsc->scsi_blk_addr += (len / hmsc->scsi_blk_size);
  hmsc->scsi_blk_len -= (len / hmsc->scsi_blk_size);

  /* case 12 : Ho = Do */
  hmsc->csw.dDataResidue -= len;

  if (hmsc->scsi_blk_len == 0U)
  {
    MSC_BOT_SendCSW (pdev, USBD_CSW_CMD_PASSED);
  }
  else
  {
    len = MIN((hmsc->scsi_blk_len * hmsc->scsi_blk_size), MSC_MEDIA_PACKET);
    /* Prepare EP to Receive next packet */
    USBD_LL_PrepareReceive (pdev, MSC_EPOUT_ADDR, hmsc->bot_data, len);
  }

  return 0;
}
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
