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
  *                      www.st.com/SLA0044
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
static int8_t SCSI_TestUnitReady(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Inquiry(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ReadFormatCapacity(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ReadCapacity10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ReadCapacity16(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_RequestSense(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_StartStopUnit(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_AllowPreventRemovable(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ModeSense6(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ModeSense10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Write10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Write12(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Read10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Read12(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Verify10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_CheckAddressRange(USBD_HandleTypeDef *pdev, uint8_t lun,
                                     uint32_t blk_offset, uint32_t blk_nbr);

static int8_t SCSI_ProcessRead(USBD_HandleTypeDef *pdev, uint8_t lun);
static int8_t SCSI_ProcessWrite(USBD_HandleTypeDef *pdev, uint8_t lun);

static int8_t SCSI_UpdateBotData(USBD_MSC_BOT_HandleTypeDef *hmsc,
                                 uint8_t *pBuff, uint16_t length);
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
  int8_t ret;
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  switch (cmd[0])
  {
  case SCSI_TEST_UNIT_READY:
    ret = SCSI_TestUnitReady(pdev, lun, cmd);
    break;

  case SCSI_REQUEST_SENSE:
    ret = SCSI_RequestSense(pdev, lun, cmd);
    break;

  case SCSI_INQUIRY:
    ret = SCSI_Inquiry(pdev, lun, cmd);
    break;

  case SCSI_START_STOP_UNIT:
    ret = SCSI_StartStopUnit(pdev, lun, cmd);
    break;

  case SCSI_ALLOW_MEDIUM_REMOVAL:
    ret = SCSI_AllowPreventRemovable(pdev, lun, cmd);
    break;

  case SCSI_MODE_SENSE6:
    ret = SCSI_ModeSense6(pdev, lun, cmd);
    break;

  case SCSI_MODE_SENSE10:
    ret = SCSI_ModeSense10(pdev, lun, cmd);
    break;

  case SCSI_READ_FORMAT_CAPACITIES:
    ret = SCSI_ReadFormatCapacity(pdev, lun, cmd);
    break;

  case SCSI_READ_CAPACITY10:
    ret = SCSI_ReadCapacity10(pdev, lun, cmd);
    break;

  case SCSI_READ_CAPACITY16:
    ret = SCSI_ReadCapacity16(pdev, lun, cmd);
    break;

  case SCSI_READ10:
    ret = SCSI_Read10(pdev, lun, cmd);
    break;

  case SCSI_READ12:
    ret = SCSI_Read12(pdev, lun, cmd);
    break;

  case SCSI_WRITE10:
    ret = SCSI_Write10(pdev, lun, cmd);
    break;

  case SCSI_WRITE12:
    ret = SCSI_Write12(pdev, lun, cmd);
    break;

  case SCSI_VERIFY10:
    ret = SCSI_Verify10(pdev, lun, cmd);
    break;

  default:
    SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, INVALID_CDB);
    hmsc->bot_status = USBD_BOT_STATUS_ERROR;
    ret = -1;
    break;
  }

  return ret;
}


/**
* @brief  SCSI_TestUnitReady
*         Process SCSI Test Unit Ready Command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_TestUnitReady(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  UNUSED(params);
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  /* case 9 : Hi > D0 */
  if (hmsc->cbw.dDataLength != 0U)
  {
    SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);

    return -1;
  }

  if (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED)
  {
    SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
    hmsc->bot_state = USBD_BOT_NO_DATA;
    return -1;
  }

  if (((USBD_StorageTypeDef *)pdev->pMSC_UserData)->IsReady(lun) != 0)
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
static int8_t SCSI_Inquiry(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  uint8_t *pPage;
  uint16_t len;
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  if (hmsc->cbw.dDataLength == 0U)
  {
    SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
    return -1;
  }

  if ((params[1] & 0x01U) != 0U) /* Evpd is set */
  {
    if (params[2] == 0U) /* Request for Supported Vital Product Data Pages*/
    {
      (void)SCSI_UpdateBotData(hmsc, MSC_Page00_Inquiry_Data, LENGTH_INQUIRY_PAGE00);
    }
    else if (params[2] == 0x80U) /* Request for VPD page 0x80 Unit Serial Number */
    {
      (void)SCSI_UpdateBotData(hmsc, MSC_Page80_Inquiry_Data, LENGTH_INQUIRY_PAGE80);
    }
    else /* Request Not supported */
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST,
                     INVALID_FIELED_IN_COMMAND);

      return -1;
    }
  }
  else
  {
    pPage = (uint8_t *)&((USBD_StorageTypeDef *)pdev->pMSC_UserData)->pInquiry[lun * STANDARD_INQUIRY_DATA_LEN];
    len = (uint16_t)pPage[4] + 5U;

    if (params[4] <= len)
    {
      len = params[4];
    }

    (void)SCSI_UpdateBotData(hmsc, pPage, len);
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
static int8_t SCSI_ReadCapacity10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  UNUSED(params);
  int8_t ret;
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  ret = ((USBD_StorageTypeDef *)pdev->pMSC_UserData)->GetCapacity(lun, &hmsc->scsi_blk_nbr, &hmsc->scsi_blk_size);

  if ((ret != 0) || (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED))
  {
    SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
    return -1;
  }

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


/**
* @brief  SCSI_ReadCapacity16
*         Process Read Capacity 16 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ReadCapacity16(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  UNUSED(params);
  uint8_t idx;
  int8_t ret;
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  ret = ((USBD_StorageTypeDef *)pdev->pMSC_UserData)->GetCapacity(lun, &hmsc->scsi_blk_nbr, &hmsc->scsi_blk_size);

  if ((ret != 0) || (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED))
  {
    SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
    return -1;
  }

  hmsc->bot_data_length = ((uint32_t)params[10] << 24) |
                          ((uint32_t)params[11] << 16) |
                          ((uint32_t)params[12] <<  8) |
                           (uint32_t)params[13];

  for (idx = 0U; idx < hmsc->bot_data_length; idx++)
  {
    hmsc->bot_data[idx] = 0U;
  }

  hmsc->bot_data[4] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 24);
  hmsc->bot_data[5] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 16);
  hmsc->bot_data[6] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >>  8);
  hmsc->bot_data[7] = (uint8_t)(hmsc->scsi_blk_nbr - 1U);

  hmsc->bot_data[8] = (uint8_t)(hmsc->scsi_blk_size >>  24);
  hmsc->bot_data[9] = (uint8_t)(hmsc->scsi_blk_size >>  16);
  hmsc->bot_data[10] = (uint8_t)(hmsc->scsi_blk_size >>  8);
  hmsc->bot_data[11] = (uint8_t)(hmsc->scsi_blk_size);

  hmsc->bot_data_length = ((uint32_t)params[10] << 24) |
                          ((uint32_t)params[11] << 16) |
                          ((uint32_t)params[12] <<  8) |
                           (uint32_t)params[13];

  return 0;
}


/**
* @brief  SCSI_ReadFormatCapacity
*         Process Read Format Capacity command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ReadFormatCapacity(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  UNUSED(params);
  uint16_t blk_size;
  uint32_t blk_nbr;
  uint16_t i;
  int8_t ret;
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  ret = ((USBD_StorageTypeDef *)pdev->pMSC_UserData)->GetCapacity(lun, &blk_nbr, &blk_size);

  if ((ret != 0) || (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED))
  {
    SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
    return -1;
  }

  for (i = 0U; i < 12U ; i++)
  {
    hmsc->bot_data[i] = 0U;
  }

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


/**
* @brief  SCSI_ModeSense6
*         Process Mode Sense6 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ModeSense6(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  UNUSED(lun);
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;
  uint16_t len = MODE_SENSE6_LEN;

  if (params[4] <= len)
  {
    len = params[4];
  }

  (void)SCSI_UpdateBotData(hmsc, MSC_Mode_Sense6_data, len);

  // set bit 7 of the device configuration byte to indicate write protection
  if (((USBD_StorageTypeDef *)pdev->pMSC_UserData)->IsWriteProtected(lun) != 0) {
    hmsc->bot_data[2] |= (1 << 7);
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
static int8_t SCSI_ModeSense10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  UNUSED(lun);
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;
  uint16_t len = MODE_SENSE10_LEN;

  if (params[8] <= len)
  {
    len = params[8];
  }

  (void)SCSI_UpdateBotData(hmsc, MSC_Mode_Sense10_data, len);

  return 0;
}


/**
* @brief  SCSI_RequestSense
*         Process Request Sense command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_RequestSense(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  UNUSED(lun);
  uint8_t i;
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  if (hmsc->cbw.dDataLength == 0U)
  {
    SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
    return -1;
  }

  for (i = 0U; i < REQUEST_SENSE_DATA_LEN; i++)
  {
    hmsc->bot_data[i] = 0U;
  }

  hmsc->bot_data[0] = 0x70U;
  hmsc->bot_data[7] = REQUEST_SENSE_DATA_LEN - 6U;

  if ((hmsc->scsi_sense_head != hmsc->scsi_sense_tail))
  {
    hmsc->bot_data[2] = (uint8_t)hmsc->scsi_sense[hmsc->scsi_sense_head].Skey;
    hmsc->bot_data[12] = (uint8_t)hmsc->scsi_sense[hmsc->scsi_sense_head].w.b.ASC;
    hmsc->bot_data[13] = (uint8_t)hmsc->scsi_sense[hmsc->scsi_sense_head].w.b.ASCQ;
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
* @param  ASC: Additional Sense Code
* @retval none

*/
void SCSI_SenseCode(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t sKey, uint8_t ASC)
{
  UNUSED(lun);
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  hmsc->scsi_sense[hmsc->scsi_sense_tail].Skey = sKey;
  hmsc->scsi_sense[hmsc->scsi_sense_tail].w.b.ASC = ASC;
  hmsc->scsi_sense[hmsc->scsi_sense_tail].w.b.ASCQ = 0U;
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
static int8_t SCSI_StartStopUnit(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  UNUSED(lun);
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  if ((hmsc->scsi_medium_state == SCSI_MEDIUM_LOCKED) && ((params[4] & 0x3U) == 2U))
  {
    SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);

    return -1;
  }

  if ((params[4] & 0x3U) == 0x1U) /* START=1 */
  {
    hmsc->scsi_medium_state = SCSI_MEDIUM_UNLOCKED;
  }
  else if ((params[4] & 0x3U) == 0x2U) /* START=0 and LOEJ Load Eject=1 */
  {
    hmsc->scsi_medium_state = SCSI_MEDIUM_EJECTED;
  }
  else if ((params[4] & 0x3U) == 0x3U) /* START=1 and LOEJ Load Eject=1 */
  {
    hmsc->scsi_medium_state = SCSI_MEDIUM_UNLOCKED;
  }
  else
  {
    /* .. */
  }
  hmsc->bot_data_length = 0U;

  return 0;
}


/**
* @brief  SCSI_AllowPreventRemovable
*         Process Allow Prevent Removable medium command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_AllowPreventRemovable(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  UNUSED(lun);
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  if (params[4] == 0U)
  {
    hmsc->scsi_medium_state = SCSI_MEDIUM_UNLOCKED;
  }
  else
  {
    hmsc->scsi_medium_state = SCSI_MEDIUM_LOCKED;
  }

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
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
  {
    /* case 10 : Ho <> Di */
    if ((hmsc->cbw.bmFlags & 0x80U) != 0x80U)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    if (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);

      return -1;
    }

    if (((USBD_StorageTypeDef *)pdev->pMSC_UserData)->IsReady(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
      return -1;
    }

    hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) |
                          ((uint32_t)params[3] << 16) |
                          ((uint32_t)params[4] <<  8) |
                          (uint32_t)params[5];

    hmsc->scsi_blk_len = ((uint32_t)params[7] <<  8) | (uint32_t)params[8];

    if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr,
                               hmsc->scsi_blk_len) < 0)
    {
      return -1; /* error */
    }

    /* cases 4,5 : Hi <> Dn */
    if (hmsc->cbw.dDataLength != (hmsc->scsi_blk_len * hmsc->scsi_blk_size))
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    hmsc->bot_state = USBD_BOT_DATA_IN;
  }
  hmsc->bot_data_length = MSC_MEDIA_PACKET;

  return SCSI_ProcessRead(pdev, lun);
}


/**
* @brief  SCSI_Read12
*         Process Read12 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_Read12(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
  {
    /* case 10 : Ho <> Di */
    if ((hmsc->cbw.bmFlags & 0x80U) != 0x80U)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    if (hmsc->scsi_medium_state == SCSI_MEDIUM_EJECTED)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
      return -1;
    }

    if (((USBD_StorageTypeDef *)pdev->pMSC_UserData)->IsReady(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
      return -1;
    }

    hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) |
                          ((uint32_t)params[3] << 16) |
                          ((uint32_t)params[4] <<  8) |
                          (uint32_t)params[5];

    hmsc->scsi_blk_len = ((uint32_t)params[6] << 24) |
                         ((uint32_t)params[7] << 16) |
                         ((uint32_t)params[8] << 8) |
                         (uint32_t)params[9];

    if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr,
                               hmsc->scsi_blk_len) < 0)
    {
      return -1; /* error */
    }

    /* cases 4,5 : Hi <> Dn */
    if (hmsc->cbw.dDataLength != (hmsc->scsi_blk_len * hmsc->scsi_blk_size))
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    hmsc->bot_state = USBD_BOT_DATA_IN;
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
static int8_t SCSI_Write10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;
  uint32_t len;

  if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
  {
    if (hmsc->cbw.dDataLength == 0U)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    /* case 8 : Hi <> Do */
    if ((hmsc->cbw.bmFlags & 0x80U) == 0x80U)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    /* Check whether Media is ready */
    if (((USBD_StorageTypeDef *)pdev->pMSC_UserData)->IsReady(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
      return -1;
    }

    /* Check If media is write-protected */
    if (((USBD_StorageTypeDef *)pdev->pMSC_UserData)->IsWriteProtected(lun) != 0)
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
    if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr,
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
    (void)USBD_LL_PrepareReceive(pdev, MSC_EPOUT_ADDR, hmsc->bot_data, len);
  }
  else /* Write Process ongoing */
  {
    return SCSI_ProcessWrite(pdev, lun);
  }

  return 0;
}


/**
* @brief  SCSI_Write12
*         Process Write12 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_Write12(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;
  uint32_t len;

  if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
  {
    if (hmsc->cbw.dDataLength == 0U)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    /* case 8 : Hi <> Do */
    if ((hmsc->cbw.bmFlags & 0x80U) == 0x80U)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    /* Check whether Media is ready */
    if (((USBD_StorageTypeDef *)pdev->pMSC_UserData)->IsReady(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
      hmsc->bot_state = USBD_BOT_NO_DATA;
      return -1;
    }

    /* Check If media is write-protected */
    if (((USBD_StorageTypeDef *)pdev->pMSC_UserData)->IsWriteProtected(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, WRITE_PROTECTED);
      hmsc->bot_state = USBD_BOT_NO_DATA;
      return -1;
    }

    hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) |
                          ((uint32_t)params[3] << 16) |
                          ((uint32_t)params[4] << 8) |
                          (uint32_t)params[5];

    hmsc->scsi_blk_len = ((uint32_t)params[6] << 24) |
                         ((uint32_t)params[7] << 16) |
                         ((uint32_t)params[8] << 8) |
                         (uint32_t)params[9];

    /* check if LBA address is in the right range */
    if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr,
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
    (void)USBD_LL_PrepareReceive(pdev, MSC_EPOUT_ADDR, hmsc->bot_data, len);
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
static int8_t SCSI_Verify10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

  if ((params[1] & 0x02U) == 0x02U)
  {
    SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
    return -1; /* Error, Verify Mode Not supported*/
  }

  if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr, hmsc->scsi_blk_len) < 0)
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
static int8_t SCSI_CheckAddressRange(USBD_HandleTypeDef *pdev, uint8_t lun,
                                     uint32_t blk_offset, uint32_t blk_nbr)
{
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;

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
static int8_t SCSI_ProcessRead(USBD_HandleTypeDef *pdev, uint8_t lun)
{
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;
  uint32_t len = hmsc->scsi_blk_len * hmsc->scsi_blk_size;

  len = MIN(len, MSC_MEDIA_PACKET);

  if (((USBD_StorageTypeDef *)pdev->pMSC_UserData)->Read(lun, hmsc->bot_data,
                                                     hmsc->scsi_blk_addr,
                                                     (len / hmsc->scsi_blk_size)) < 0)
  {
    SCSI_SenseCode(pdev, lun, HARDWARE_ERROR, UNRECOVERED_READ_ERROR);
    return -1;
  }

  (void)USBD_LL_Transmit(pdev, MSC_EPIN_ADDR, hmsc->bot_data, len);

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
static int8_t SCSI_ProcessWrite(USBD_HandleTypeDef *pdev, uint8_t lun)
{
  USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef *)pdev->pMSC_ClassData;
  uint32_t len = hmsc->scsi_blk_len * hmsc->scsi_blk_size;

  len = MIN(len, MSC_MEDIA_PACKET);

  if (((USBD_StorageTypeDef *)pdev->pMSC_UserData)->Write(lun, hmsc->bot_data,
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
    MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_PASSED);
  }
  else
  {
    len = MIN((hmsc->scsi_blk_len * hmsc->scsi_blk_size), MSC_MEDIA_PACKET);

    /* Prepare EP to Receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, MSC_EPOUT_ADDR, hmsc->bot_data, len);
  }

  return 0;
}


/**
* @brief  SCSI_UpdateBotData
*         fill the requested Data to transmit buffer
* @param  hmsc handler
* @param  params: Data buffer
* @param  length: Data length
* @retval status
*/
static int8_t SCSI_UpdateBotData(USBD_MSC_BOT_HandleTypeDef *hmsc,
                                 uint8_t *pBuff, uint16_t length)
{
  uint16_t len = length;

  hmsc->bot_data_length = len;

  while (len != 0U)
  {
    len--;
    hmsc->bot_data[len] = pBuff[len];
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
