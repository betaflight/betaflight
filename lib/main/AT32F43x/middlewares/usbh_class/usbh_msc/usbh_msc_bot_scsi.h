/**
  **************************************************************************
  * @file     usbh_msc_bot_scsi.h
  * @brief    usb host msc bulk-only transfer and scsi header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBH_MSC_BOT_SCSI_H
#define __USBH_MSC_BOT_SCSI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbh_core.h"
#include "usb_conf.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @addtogroup USBH_msc_bot_scsi_class
  * @{
  */

/** @defgroup USBH_msc_bot_scsi_class_definition
  * @{
  */

#define MSC_CBW_SIGNATURE                0x43425355
#define MSC_CSW_SIGNATURE                0x53425355
#define MSC_CBW_TAG                      0x50607080
#define MSC_CBW_FLAG_IN                  0x80
#define MSC_CBW_FLAG_OUT                 0x00

#define MSC_CBW_LEN                      31
#define MSC_CSW_LEN                      13
#define MSC_CBW_CB_LEN                   16
#define MSC_TEST_UNIT_READY_CMD_LEN      12
#define MSC_TEST_UNIT_READY_DATA_LEN     0
#define MSC_INQUIRY_CMD_LEN              12
#define MSC_INQUIRY_DATA_LEN             36
#define MSC_CAPACITY10_CMD_LEN           12
#define MSC_CAPACITY10_DATA_LEN          8
#define MSC_REQUEST_SENSE_CMD_LEN        12
#define MSC_REQUEST_SENSE_DATA_LEN       18
#define MSC_WRITE_CMD_LEN                12
#define MSC_READ_CMD_LEN                 10

#define MSC_OPCODE_INQUIRY               0x12
#define MSC_OPCODE_CAPACITY              0x25
#define MSC_OPCODE_TEST_UNIT_READY       0x00
#define MSC_OPCODE_REQUEST_SENSE         0x03
#define MSC_OPCODE_WRITE10               0x2A
#define MSC_OPCODE_READ10                0x28

typedef enum
{
  BOT_STATE_IDLE,
  BOT_STATE_SEND_CBW,
  BOT_STATE_SEND_CBW_WAIT,
  BOT_STATE_DATA_IN,
  BOT_STATE_DATA_IN_WAIT,
  BOT_STATE_DATA_OUT,
  BOT_STATE_DATA_OUT_WAIT,
  BOT_STATE_RECV_CSW,
  BOT_STATE_RECV_CSW_WAIT,
  BOT_STATE_ERROR_IN,
  BOT_STATE_ERROR_OUT,
  BOT_STATE_COMPLETE
}msc_bot_state_type;

typedef enum
{
  CMD_STATE_SEND,
  CMD_STATE_WAIT,
}msc_cmd_state_type;

/**
  * @brief  usb msc process state
  */
typedef enum
{
  USBH_MSC_INIT,
  USBH_MSC_INQUIRY,
  USBH_MSC_TEST_UNIT_READY,
  USBH_MSC_READ_CAPACITY10,
  USBH_MSC_REQUEST_SENSE,
  USBH_MSC_READ10,
  USBH_MSC_WRITE,
  USBH_MSC_BUSY,
  USBH_MSC_ERROR,
  USBH_MSC_IDLE
}msc_state_type;

typedef enum
{
  MSC_OK,
  MSC_NOT_READY,
  MSC_ERROR
}msc_error_type;


/**
  * @brief  usb msc inquiry data type
  */
typedef struct
{
  uint8_t pdev_type;
  uint8_t rmb;
  uint8_t version;
  uint8_t data_format;
  uint8_t length;
  uint8_t reserved[3];
  uint8_t vendor[8];
  uint8_t product[16];
  uint8_t revision[4];
}msc_scsi_data_inquiry;


/**
  * @brief  usb msc capacity data type
  */
typedef struct
{
  uint32_t blk_nbr;
  uint32_t blk_size;
}msc_scsi_data_capacity;

/**
  * @brief  usb msc bulk-only command block wrapper type
  */
typedef struct
{
  uint32_t dCBWSignature;
  uint32_t dCBWTag;
  uint32_t dCBWDataTransferLength;
  uint8_t  bmCBWFlags;
  uint8_t  bCBWLUN;
  uint8_t  bCBWCBLength;
  uint8_t  CBWCB[MSC_CBW_CB_LEN];
}msc_bot_cbw_type;

/**
  * @brief  usb msc bulk-only command status wrapper type
  */
typedef struct
{
  uint32_t dCBWSignature;
  uint32_t dCBWTag;
  uint32_t dCSWDataResidue;
  uint8_t  bCSWStatus;
}msc_bot_csw_type;

/**
  * @brief  usb msc bulk-only transfer control type
  */
typedef struct
{
  uint8_t buffer[32];
  msc_bot_cbw_type cbw;
  msc_bot_csw_type csw;
  msc_cmd_state_type cmd_state;
  msc_bot_state_type bot_state;
  uint8_t *data;
  void *msc_struct;
}msc_bot_trans_type;

/**
  * @brief  usb msc bank type
  */
typedef struct
{
  msc_scsi_data_inquiry inquiry;
  msc_scsi_data_capacity capacity;
  msc_state_type state;
  msc_error_type ready;
  usb_sts_type pre_state;
  uint8_t change;
}usbh_msc_unit_type;


usb_sts_type usb_bot_request(void *uhost, msc_bot_trans_type *bot_trans);

usb_sts_type usbh_msc_bot_scsi_get_inquiry(void *uhost,  msc_bot_trans_type *bot_trans,
                                            uint8_t lun, msc_scsi_data_inquiry *inquiry);

usb_sts_type usbh_msc_bot_scsi_capacity(void *uhost, msc_bot_trans_type *bot_trans,
                                            uint8_t lun, msc_scsi_data_capacity *capacity);

usb_sts_type usbh_msc_bot_scsi_test_unit_ready(void *uhost, msc_bot_trans_type *bot_trans,
                                            uint8_t lun);

usb_sts_type usbh_msc_bot_scsi_request_sense(void *uhost, msc_bot_trans_type *bot_trans,
                                            uint8_t lun);

usb_sts_type usbh_msc_bot_scsi_write(void *uhost, msc_bot_trans_type *bot_trans,
                                     uint32_t address, uint8_t *write_data,
                                     uint32_t write_len, uint8_t lun);

usb_sts_type usbh_msc_bot_scsi_read(void *uhost, msc_bot_trans_type *bot_trans,
                                     uint32_t address, uint8_t *read_data,
                                     uint32_t read_len, uint8_t lun);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif

