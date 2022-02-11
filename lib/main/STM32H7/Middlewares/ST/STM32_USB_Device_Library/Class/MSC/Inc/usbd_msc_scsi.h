/**
  ******************************************************************************
  * @file    usbd_msc_scsi.h
  * @author  MCD Application Team
  * @brief   Header for the usbd_msc_scsi.c file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_MSC_SCSI_H
#define __USBD_MSC_SCSI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_SCSI
  * @brief header file for the storage disk file
  * @{
  */

/** @defgroup USBD_SCSI_Exported_Defines
  * @{
  */

#define SENSE_LIST_DEEPTH                           4U

/* SCSI Commands */
#define SCSI_FORMAT_UNIT                            0x04U
#define SCSI_INQUIRY                                0x12U
#define SCSI_MODE_SELECT6                           0x15U
#define SCSI_MODE_SELECT10                          0x55U
#define SCSI_MODE_SENSE6                            0x1AU
#define SCSI_MODE_SENSE10                           0x5AU
#define SCSI_ALLOW_MEDIUM_REMOVAL                   0x1EU
#define SCSI_READ6                                  0x08U
#define SCSI_READ10                                 0x28U
#define SCSI_READ12                                 0xA8U
#define SCSI_READ16                                 0x88U

#define SCSI_READ_CAPACITY10                        0x25U
#define SCSI_READ_CAPACITY16                        0x9EU

#define SCSI_REQUEST_SENSE                          0x03U
#define SCSI_START_STOP_UNIT                        0x1BU
#define SCSI_TEST_UNIT_READY                        0x00U
#define SCSI_WRITE6                                 0x0AU
#define SCSI_WRITE10                                0x2AU
#define SCSI_WRITE12                                0xAAU
#define SCSI_WRITE16                                0x8AU

#define SCSI_VERIFY10                               0x2FU
#define SCSI_VERIFY12                               0xAFU
#define SCSI_VERIFY16                               0x8FU

#define SCSI_SEND_DIAGNOSTIC                        0x1DU
#define SCSI_READ_FORMAT_CAPACITIES                 0x23U

#define NO_SENSE                                    0U
#define RECOVERED_ERROR                             1U
#define NOT_READY                                   2U
#define MEDIUM_ERROR                                3U
#define HARDWARE_ERROR                              4U
#define ILLEGAL_REQUEST                             5U
#define UNIT_ATTENTION                              6U
#define DATA_PROTECT                                7U
#define BLANK_CHECK                                 8U
#define VENDOR_SPECIFIC                             9U
#define COPY_ABORTED                                10U
#define ABORTED_COMMAND                             11U
#define VOLUME_OVERFLOW                             13U
#define MISCOMPARE                                  14U


#define INVALID_CDB                                 0x20U
#define INVALID_FIELED_IN_COMMAND                   0x24U
#define PARAMETER_LIST_LENGTH_ERROR                 0x1AU
#define INVALID_FIELD_IN_PARAMETER_LIST             0x26U
#define ADDRESS_OUT_OF_RANGE                        0x21U
#define MEDIUM_NOT_PRESENT                          0x3AU
#define MEDIUM_HAVE_CHANGED                         0x28U
#define WRITE_PROTECTED                             0x27U
#define UNRECOVERED_READ_ERROR                      0x11U
#define WRITE_FAULT                                 0x03U

#define READ_FORMAT_CAPACITY_DATA_LEN               0x0CU
#define READ_CAPACITY10_DATA_LEN                    0x08U
#define REQUEST_SENSE_DATA_LEN                      0x12U
#define STANDARD_INQUIRY_DATA_LEN                   0x24U
#define BLKVFY                                      0x04U

#define SCSI_MEDIUM_UNLOCKED                        0x00U
#define SCSI_MEDIUM_LOCKED                          0x01U
#define SCSI_MEDIUM_EJECTED                         0x02U
/**
  * @}
  */


/** @defgroup USBD_SCSI_Exported_TypesDefinitions
  * @{
  */

typedef struct _SENSE_ITEM
{
  uint8_t Skey;
  union
  {
    struct _ASCs
    {
      uint8_t ASC;
      uint8_t ASCQ;
    } b;
    uint8_t ASC;
    uint8_t *pData;
  } w;
} USBD_SCSI_SenseTypeDef;
/**
  * @}
  */

/** @defgroup USBD_SCSI_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_SCSI_Exported_Variables
  * @{
  */

/**
  * @}
  */
/** @defgroup USBD_SCSI_Exported_FunctionsPrototype
  * @{
  */
int8_t SCSI_ProcessCmd(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *cmd);

void SCSI_SenseCode(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t sKey,
                    uint8_t ASC);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_MSC_SCSI_H */
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

