/**
  **************************************************************************
  * @file     usbh_msc_bot_scsi.c
  * @brief    usb host msc bulk-only transfer and scsi type
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
#include "usbh_msc_bot_scsi.h"
#include "usbh_msc_class.h"
#include "usb_conf.h"
#include "usbh_core.h"
#include "usbh_ctrl.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @defgroup USBH_msc_bot_scsi_class
  * @brief usb host class msc bot scsi
  * @{
  */

/** @defgroup USBH_msc_bot_scsi_class_private_functions
  * @{
  */

static usb_sts_type usbh_bot_cbw(msc_bot_cbw_type *cbw, uint32_t data_length, uint8_t cmd_len, uint8_t flag);
static usb_sts_type usbh_cmd_inquiry(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun);
static usb_sts_type usbh_cmd_capacity10(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun);
static usb_sts_type usbh_cmd_test_unit_ready(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun);
static usb_sts_type usbh_cmd_requset_sense(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun);
static usb_sts_type usbh_cmd_write(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun,
                            uint32_t data_len, uint32_t address, uint8_t *buffer);
static usb_sts_type usbh_cmd_read(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun,
                            uint32_t data_len, uint32_t address, uint8_t *buffer);

/**
  * @brief  usb host bulk-only cbw
  * @param  cbw: to the structure of msc_bot_cbw_type
  * @param  data_length: data length
  * @param  cmd_len: command len
  * @param  flag: cbw flag
  * @retval status: usb_sts_type status
  */
static usb_sts_type usbh_bot_cbw(msc_bot_cbw_type *cbw, uint32_t data_length, uint8_t cmd_len, uint8_t flag)
{
  uint8_t i_index;
  cbw->dCBWSignature           = MSC_CBW_SIGNATURE;
  cbw->dCBWTag                 = MSC_CBW_TAG;
  cbw->dCBWDataTransferLength  = data_length;
  cbw->bmCBWFlags              = flag;
  cbw->bCBWCBLength            = cmd_len;
  for(i_index = 0; i_index < MSC_CBW_CB_LEN; i_index ++)
  {
   cbw->CBWCB[i_index] = 0x00;
  }
  return USB_OK;
}

/**
  * @brief  usb host msc command inquiry
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  cmd: command buffer
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
static usb_sts_type usbh_cmd_inquiry(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun)
{
  usbh_msc_type *msc_struct = (usbh_msc_type *)bot_trans->msc_struct;
  cmd[0] = MSC_OPCODE_INQUIRY;
  cmd[1] = lun << 5;
  cmd[4] = MSC_INQUIRY_DATA_LEN;

  bot_trans->data = (uint8_t *)&msc_struct->l_unit_n[lun].inquiry;
  return USB_OK;
}

/**
  * @brief  usb host msc command capacity10
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  cmd: command buffer
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
static usb_sts_type usbh_cmd_capacity10(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun)
{
  usbh_msc_type *msc_struct = (usbh_msc_type *)bot_trans->msc_struct;
  cmd[0] = MSC_OPCODE_CAPACITY;
  cmd[1] = lun << 5;

  msc_struct->bot_trans.data = (uint8_t *)bot_trans->buffer;
  return USB_OK;
}

/**
  * @brief  usb host msc command test unit ready
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  cmd: command buffer
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
static usb_sts_type usbh_cmd_test_unit_ready(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun)
{
  cmd[0] = MSC_OPCODE_TEST_UNIT_READY;
  cmd[1] = lun << 5;

  return USB_OK;
}

/**
  * @brief  usb host msc command request sense
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  cmd: command buffer
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
static usb_sts_type usbh_cmd_requset_sense(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun)
{
  cmd[0] = MSC_OPCODE_REQUEST_SENSE;
  cmd[1] = lun << 5;
  cmd[4] = MSC_REQUEST_SENSE_DATA_LEN;

  bot_trans->data = bot_trans->buffer;
  return USB_OK;
}

/**
  * @brief  usb host msc command write
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  cmd: command buffer
  * @param  lun: logical unit number
  * @param  data_len: transfer data length
  * @param  address: logical block address
  * @param  buffer: transfer data buffer
  * @retval status: usb_sts_type status
  */
static usb_sts_type usbh_cmd_write(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun,
                            uint32_t data_len, uint32_t address, uint8_t *buffer)
{
  cmd[0] = MSC_OPCODE_WRITE10;
  cmd[1] = lun << 5;
  cmd[2] = (uint8_t)(address >> 24);
  cmd[3] = (uint8_t)(address >> 16);
  cmd[4] = (uint8_t)(address >> 8);
  cmd[5] = (uint8_t)(address & 0xFF);

  cmd[7] = data_len >> 8;
  cmd[8] = data_len;

  bot_trans->data = buffer;
  return USB_OK;
}

/**
  * @brief  usb host msc command read
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  cmd: command buffer
  * @param  lun: logical unit number
  * @param  data_len: transfer data length
  * @param  address: logical block address
  * @param  buffer: transfer data buffer
  * @retval status: usb_sts_type status
  */
static usb_sts_type usbh_cmd_read(msc_bot_trans_type *bot_trans, uint8_t *cmd, uint8_t lun,
                            uint32_t data_len, uint32_t address, uint8_t *buffer)
{
  cmd[0] = MSC_OPCODE_READ10;
  cmd[1] = lun << 5;
  cmd[2] = (uint8_t)(address >> 24);
  cmd[3] = (uint8_t)(address >> 16);
  cmd[4] = (uint8_t)(address >> 8);
  cmd[5] = (uint8_t)(address & 0xFF);

  cmd[7] = data_len >> 8;
  cmd[8] = data_len;

  bot_trans->data = buffer;
  return USB_OK;
}

/**
  * @brief  usb host csw check
  * @param  cbw: to the structure of msc_bot_cbw_type
  * @param  csw: to the structure of msc_bot_csw_type
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_check_csw(void *uhost, msc_bot_cbw_type *cbw, msc_bot_csw_type *csw)
{
  usb_sts_type status = USB_FAIL;
  if(csw->dCBWSignature == MSC_CSW_SIGNATURE)
  {
    if(csw->dCBWTag == cbw->dCBWTag)
    {
      if(csw->bCSWStatus == 0)
      {
        status = USB_OK;
      }
    }
  }
  return status;
}

/**
  * @brief  usb host msc bulk-only request
  * @param  uhost: to the structure of usbh_core_type
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @retval status: usb_sts_type status
  */
usb_sts_type usb_bot_request(void *uhost, msc_bot_trans_type *bot_trans)
{
  usb_sts_type status = USB_WAIT;
  urb_sts_type urb_status;
  usb_sts_type clr_status;
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_msc_type *msc_struct = (usbh_msc_type *)bot_trans->msc_struct;
  switch(bot_trans->bot_state)
  {
    case BOT_STATE_SEND_CBW:
      usbh_bulk_send(puhost, msc_struct->chout, (uint8_t *)(&bot_trans->cbw), MSC_CBW_LEN);
      bot_trans->bot_state = BOT_STATE_SEND_CBW_WAIT;
      break;

    case BOT_STATE_SEND_CBW_WAIT:
      urb_status = usbh_get_urb_status(puhost, msc_struct->chout);
      if(urb_status == URB_DONE)
      {
        if(bot_trans->cbw.dCBWDataTransferLength != 0)
        {
          if(bot_trans->cbw.bmCBWFlags == MSC_CBW_FLAG_IN)
          {
            bot_trans->bot_state = BOT_STATE_DATA_IN;
          }
          else
          {
            bot_trans->bot_state = BOT_STATE_DATA_OUT;
          }
        }
        else
        {
          bot_trans->bot_state = BOT_STATE_RECV_CSW;
        }
      }
      else if(urb_status == URB_NOTREADY)
      {
        bot_trans->bot_state = BOT_STATE_SEND_CBW;
      }
      else if(urb_status == URB_STALL)
      {
         bot_trans->bot_state = BOT_STATE_ERROR_OUT;
      }
      break;

    case BOT_STATE_DATA_IN:
      usbh_bulk_recv(puhost, msc_struct->chin, bot_trans->data,
                     msc_struct->in_maxpacket);
      bot_trans->bot_state = BOT_STATE_DATA_IN_WAIT;
      break;

    case BOT_STATE_DATA_IN_WAIT:
      urb_status = usbh_get_urb_status(puhost, msc_struct->chin);
      if(urb_status == URB_DONE)
      {
        if(bot_trans->cbw.dCBWDataTransferLength > msc_struct->in_maxpacket)
        {
          bot_trans->data += msc_struct->in_maxpacket;
          bot_trans->cbw.dCBWDataTransferLength -= msc_struct->in_maxpacket;
        }
        else
        {
          bot_trans->cbw.dCBWDataTransferLength = 0;
        }
        if(bot_trans->cbw.dCBWDataTransferLength > 0)
        {
          usbh_bulk_recv(puhost, msc_struct->chin, bot_trans->data,
                     msc_struct->in_maxpacket);
        }
        else
        {
          bot_trans->bot_state = BOT_STATE_RECV_CSW;
        }
      }
      else if(urb_status == URB_STALL)
      {
        bot_trans->bot_state = BOT_STATE_ERROR_IN;
      }

      break;

    case BOT_STATE_DATA_OUT:
      usbh_bulk_send(puhost, msc_struct->chout, bot_trans->data, msc_struct->out_maxpacket);
      bot_trans->bot_state = BOT_STATE_DATA_OUT_WAIT;
      break;

    case BOT_STATE_DATA_OUT_WAIT:
      urb_status = usbh_get_urb_status(puhost, msc_struct->chout);
      if(urb_status == URB_DONE)
      {
        if(bot_trans->cbw.dCBWDataTransferLength > msc_struct->out_maxpacket)
        {
          bot_trans->data += msc_struct->out_maxpacket;
          bot_trans->cbw.dCBWDataTransferLength -= msc_struct->out_maxpacket;
        }
        else
        {
          bot_trans->cbw.dCBWDataTransferLength = 0;
        }
        if(bot_trans->cbw.dCBWDataTransferLength > 0)
        {
          usbh_bulk_send(puhost, msc_struct->chout, bot_trans->data, msc_struct->out_maxpacket);
        }
        else
        {
          bot_trans->bot_state = BOT_STATE_RECV_CSW;
        }
      }
      else if(urb_status == URB_NOTREADY)
      {
        bot_trans->bot_state = BOT_STATE_DATA_OUT;
      }
      else if(urb_status == URB_STALL)
      {
        bot_trans->bot_state = BOT_STATE_ERROR_OUT;
      }
      break;

    case BOT_STATE_RECV_CSW:
      usbh_bulk_recv(puhost, msc_struct->chin, (uint8_t *)&bot_trans->csw,
                     MSC_CSW_LEN);
      bot_trans->bot_state = BOT_STATE_RECV_CSW_WAIT;

      break;
    case BOT_STATE_RECV_CSW_WAIT:
      urb_status = usbh_get_urb_status(puhost, msc_struct->chin);
      if(urb_status == URB_DONE)
      {
        bot_trans->bot_state = BOT_STATE_SEND_CBW;
        bot_trans->cmd_state = CMD_STATE_SEND;
        status = usbh_check_csw(uhost, &bot_trans->cbw, &bot_trans->csw);
      }
      else if(urb_status == URB_STALL)
      {
        bot_trans->bot_state = BOT_STATE_ERROR_IN;
      }

      break;
    case BOT_STATE_ERROR_IN:
      clr_status = usbh_clear_ept_feature(puhost, msc_struct->eptin, msc_struct->chin);
      if(clr_status == USB_OK)
      {
        bot_trans->bot_state = BOT_STATE_RECV_CSW;
        usbh_set_toggle(puhost, msc_struct->chin, 0);
      }

      break;
    case BOT_STATE_ERROR_OUT:
      clr_status = usbh_clear_ept_feature(puhost, msc_struct->eptout, msc_struct->chout);
      if(clr_status == USB_OK)
      {
        usbh_set_toggle(puhost, msc_struct->chout, 1 - puhost->hch[msc_struct->chout].toggle_out);
        usbh_set_toggle(puhost, msc_struct->chin, 0);
        bot_trans->bot_state = BOT_STATE_ERROR_IN;
      }
      break;
    case BOT_STATE_COMPLETE:
      break;

    default:
      break;
  }

  return status;
}

/**
  * @brief  usb host msc bulk-only get inquiry request
  * @param  uhost: to the structure of usbh_core_type
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  lun: logical unit number
  * @param  inquiry: to the structure of msc_scsi_data_inquiry
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_msc_bot_scsi_get_inquiry(void *uhost,  msc_bot_trans_type *bot_trans,
                                            uint8_t lun, msc_scsi_data_inquiry *inquiry)
{
  usb_sts_type status = USB_WAIT;
  switch(bot_trans->cmd_state)
  {
    case CMD_STATE_SEND:
      usbh_bot_cbw(&bot_trans->cbw, MSC_INQUIRY_DATA_LEN, MSC_INQUIRY_CMD_LEN, MSC_CBW_FLAG_IN);
      bot_trans->cbw.bCBWLUN = lun;
      usbh_cmd_inquiry(bot_trans, bot_trans->cbw.CBWCB, lun);
      bot_trans->cmd_state = CMD_STATE_WAIT;
      bot_trans->bot_state = BOT_STATE_SEND_CBW;
    break;

    case CMD_STATE_WAIT:
      status = usb_bot_request(uhost, bot_trans);
      if(status == USB_OK)
      {
        bot_trans->cmd_state = CMD_STATE_SEND;
      }
      if(status == USB_FAIL)
      {
        bot_trans->cmd_state = CMD_STATE_SEND;
      }
      break;
    default:
      break;
  }
  return status;
}

/**
  * @brief  usb host msc bulk-only capacity request
  * @param  uhost: to the structure of usbh_core_type
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  lun: logical unit number
  * @param  capacity: to the structure of msc_scsi_data_capacity
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_msc_bot_scsi_capacity(void *uhost, msc_bot_trans_type *bot_trans,
                                            uint8_t lun, msc_scsi_data_capacity *capacity)
{
  usb_sts_type status = USB_WAIT;
  switch(bot_trans->cmd_state)
  {
    case CMD_STATE_SEND:
      usbh_bot_cbw(&bot_trans->cbw, MSC_CAPACITY10_DATA_LEN, MSC_CAPACITY10_CMD_LEN, MSC_CBW_FLAG_IN);
      bot_trans->cbw.bCBWLUN = lun;
      usbh_cmd_capacity10(bot_trans, bot_trans->cbw.CBWCB, lun);
      bot_trans->cmd_state = CMD_STATE_WAIT;
      bot_trans->bot_state = BOT_STATE_SEND_CBW;
      break;

    case CMD_STATE_WAIT:
      status = usb_bot_request(uhost, bot_trans);
      if(status == USB_OK)
      {
        capacity->blk_nbr = bot_trans->buffer[3] | bot_trans->buffer[2] << 8 |
                            bot_trans->buffer[1] << 16 | bot_trans->buffer[0] << 24;
        capacity->blk_size =  bot_trans->buffer[7] | bot_trans->buffer[6] << 8 ;
        bot_trans->cmd_state = CMD_STATE_SEND;
      }
      if(status == USB_FAIL)
      {
        bot_trans->cmd_state = CMD_STATE_SEND;
      }
      break;
    default:
      break;
  }
  return status;
}

/**
  * @brief  usb host msc bulk-only tet unit ready request
  * @param  uhost: to the structure of usbh_core_type
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_msc_bot_scsi_test_unit_ready(void *uhost, msc_bot_trans_type *bot_trans,
                                            uint8_t lun)
{
  usb_sts_type status = USB_WAIT;
  switch(bot_trans->cmd_state)
  {
    case CMD_STATE_SEND:
      usbh_bot_cbw(&bot_trans->cbw, MSC_TEST_UNIT_READY_DATA_LEN,
                   MSC_TEST_UNIT_READY_CMD_LEN, MSC_CBW_FLAG_OUT);
      bot_trans->cbw.bCBWLUN = lun;
      usbh_cmd_test_unit_ready(bot_trans, bot_trans->cbw.CBWCB, lun);
      bot_trans->cmd_state = CMD_STATE_WAIT;
      bot_trans->bot_state = BOT_STATE_SEND_CBW;
    break;

    case CMD_STATE_WAIT:
      status = usb_bot_request(uhost, bot_trans);
      if(status == USB_OK)
      {
         bot_trans->cmd_state = CMD_STATE_SEND;
      }
      if(status == USB_FAIL)
      {
        bot_trans->cmd_state = CMD_STATE_SEND;
      }
      break;
    default:
      break;
  }
  return status;
}

/**
  * @brief  usb host msc bulk-only request sense request
  * @param  uhost: to the structure of usbh_core_type
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_msc_bot_scsi_request_sense(void *uhost, msc_bot_trans_type *bot_trans,
                                            uint8_t lun)
{
  usb_sts_type status = USB_WAIT;
  switch(bot_trans->cmd_state)
  {
    case CMD_STATE_SEND:
      usbh_bot_cbw(&bot_trans->cbw, MSC_REQUEST_SENSE_DATA_LEN,
                   MSC_REQUEST_SENSE_CMD_LEN, MSC_CBW_FLAG_IN);
      bot_trans->cbw.bCBWLUN = lun;
      usbh_cmd_requset_sense(bot_trans, bot_trans->cbw.CBWCB, lun);
      bot_trans->cmd_state = CMD_STATE_WAIT;
      bot_trans->bot_state = BOT_STATE_SEND_CBW;
    break;

    case CMD_STATE_WAIT:
      status = usb_bot_request(uhost, bot_trans);
      if(status == USB_OK)
      {
         bot_trans->cmd_state = CMD_STATE_SEND;
      }
      if(status == USB_FAIL)
      {
        bot_trans->cmd_state = CMD_STATE_SEND;
      }
      break;
    default:
      break;
  }
  return status;
}

/**
  * @brief  usb host msc bulk-only write request
  * @param  uhost: to the structure of usbh_core_type
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  address: logical block address
  * @param  write_data: write data buffer
  * @param  write_len: write data length
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_msc_bot_scsi_write(void *uhost, msc_bot_trans_type *bot_trans,
                                     uint32_t address, uint8_t *write_data,
                                     uint32_t write_len, uint8_t lun)
{
  usb_sts_type status = USB_WAIT;
  switch(bot_trans->cmd_state)
  {
    case CMD_STATE_SEND:
      usbh_bot_cbw(&bot_trans->cbw, write_len * 512,
                   MSC_WRITE_CMD_LEN, MSC_CBW_FLAG_OUT);
      bot_trans->cbw.bCBWLUN = lun;
      usbh_cmd_write(bot_trans, bot_trans->cbw.CBWCB, lun, write_len, address, write_data);
      bot_trans->cmd_state = CMD_STATE_WAIT;
      bot_trans->bot_state = BOT_STATE_SEND_CBW;
    break;

    case CMD_STATE_WAIT:
      status = usb_bot_request(uhost, bot_trans);
      if(status == USB_OK)
      {
         bot_trans->cmd_state = CMD_STATE_SEND;
      }
      if(status == USB_FAIL)
      {
        bot_trans->cmd_state = CMD_STATE_SEND;
      }
    break;
    default:
      break;
  }
  return status;
}

/**
  * @brief  usb host msc bulk-only read request
  * @param  uhost: to the structure of usbh_core_type
  * @param  bot_trans: to the structure of msc_bot_trans_type
  * @param  address: logical block address
  * @param  read_data: read data buffer
  * @param  read_len: read data length
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_msc_bot_scsi_read(void *uhost, msc_bot_trans_type *bot_trans,
                                     uint32_t address, uint8_t *read_data,
                                     uint32_t read_len, uint8_t lun)
{
  usb_sts_type status = USB_WAIT;
  switch(bot_trans->cmd_state)
  {
    case CMD_STATE_SEND:
      usbh_bot_cbw(&bot_trans->cbw, read_len * 512,
                   MSC_READ_CMD_LEN, MSC_CBW_FLAG_IN);
      bot_trans->cbw.bCBWLUN = lun;
      usbh_cmd_read(bot_trans, bot_trans->cbw.CBWCB, lun, read_len, address, read_data);
      bot_trans->cmd_state = CMD_STATE_WAIT;
      bot_trans->bot_state = BOT_STATE_SEND_CBW;
    break;

    case CMD_STATE_WAIT:
      status = usb_bot_request(uhost, bot_trans);
      if(status == USB_OK)
      {
         bot_trans->cmd_state = CMD_STATE_SEND;
      }
      if(status == USB_FAIL)
      {
        bot_trans->cmd_state = CMD_STATE_SEND;
      }
      break;
    default:
      break;
  }
  return status;
}

/**
  * @brief  usb host msc init
  * @param  msc_struct: to the structure of usbh_msc_type
  * @retval status: usb_sts_type status
  */
usb_sts_type msc_bot_scsi_init(usbh_msc_type *msc_struct)
{
  msc_struct->state = USBH_MSC_INIT;
  msc_struct->ctrl_state = USBH_MSC_STATE_IDLE;
  msc_struct->error = MSC_OK;
  msc_struct->cur_lun = 0;
  msc_struct->max_lun = 0;
  msc_struct->use_lun = 0;
  msc_struct->bot_trans.msc_struct = &usbh_msc;
  msc_struct->bot_trans.cmd_state = CMD_STATE_SEND;
  msc_struct->bot_trans.bot_state = BOT_STATE_SEND_CBW;
  return USB_OK;
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




