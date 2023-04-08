/**
  **************************************************************************
  * @file     usbh_msc_class.c
  * @brief    usb host msc class type
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
#include "usbh_msc_class.h"
#include "usb_conf.h"
#include "usbh_core.h"
#include "usbh_ctrl.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @defgroup USBH_msc_class
  * @brief usb host class msc demo
  * @{
  */

/** @defgroup USBH_msc_class_private_functions
  * @{
  */

static usb_sts_type uhost_init_handler(void *uhost);
static usb_sts_type uhost_reset_handler(void *uhost);
static usb_sts_type uhost_request_handler(void *uhost);
static usb_sts_type uhost_process_handler(void *uhost);

static usb_sts_type usbh_msc_get_max_lun(void *uhost, uint8_t *lun);
static usb_sts_type usbh_msc_clear_feature(void *uhost, uint8_t ept_num);


usbh_msc_type usbh_msc;

usbh_class_handler_type uhost_msc_class_handler =
{
 uhost_init_handler,
 uhost_reset_handler,
 uhost_request_handler,
 uhost_process_handler,
 &usbh_msc
};



/**
  * @brief  usb host class init handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type uhost_init_handler(void *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usb_sts_type status = USB_OK;
  uint8_t if_x, eptidx = 0;
  usbh_msc_type *pmsc = &usbh_msc;
  puhost->class_handler->pdata = &usbh_msc;

  if_x = usbh_find_interface(puhost, USB_CLASS_CODE_MSC, MSC_SUBCLASS_SCSI_TRANS, MSC_PROTOCOL_BBB);
  if(if_x == 0xFF)
  {
    USBH_DEBUG("Unsupport Device!");
    return USB_NOT_SUPPORT;
  }
  pmsc->protocol = puhost->dev.cfg_desc.interface[if_x].interface.bInterfaceProtocol;

  for(eptidx = 0; eptidx < puhost->dev.cfg_desc.interface[if_x].interface.bNumEndpoints; eptidx ++)
  {
    if(puhost->dev.cfg_desc.interface[if_x].endpoint[eptidx].bEndpointAddress & 0x80)
    {
      pmsc->eptin = puhost->dev.cfg_desc.interface[if_x].endpoint[eptidx].bEndpointAddress;
      pmsc->in_maxpacket = puhost->dev.cfg_desc.interface[if_x].endpoint[eptidx].wMaxPacketSize;
      pmsc->in_poll = puhost->dev.cfg_desc.interface[if_x].endpoint[eptidx].bInterval;

      pmsc->chin = usbh_alloc_channel(puhost, pmsc->eptin);
      /* enable channel */
      usbh_hc_open(puhost, pmsc->chin, pmsc->eptin,
                    puhost->dev.address, EPT_BULK_TYPE,
                    pmsc->in_maxpacket,
                    puhost->dev.speed);
    }
    else
    {
      pmsc->eptout = puhost->dev.cfg_desc.interface[if_x].endpoint[eptidx].bEndpointAddress;
      pmsc->out_maxpacket = puhost->dev.cfg_desc.interface[if_x].endpoint[eptidx].wMaxPacketSize;
      pmsc->out_poll = puhost->dev.cfg_desc.interface[if_x].endpoint[eptidx].bInterval;

      pmsc->chout = usbh_alloc_channel(puhost, pmsc->eptout);
      /* enable channel */
      usbh_hc_open(puhost, pmsc->chout,pmsc->eptout,
                    puhost->dev.address, EPT_BULK_TYPE,
                    pmsc->out_maxpacket,
                    puhost->dev.speed);
    }
  }

  msc_bot_scsi_init(pmsc);
  usbh_set_toggle(puhost, pmsc->chout, 0);
  usbh_set_toggle(puhost, pmsc->chin, 0);
  return status;
}

/**
  * @brief  usb host class reset handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type uhost_reset_handler(void *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_msc_type *pmsc = (usbh_msc_type *)puhost->class_handler->pdata;
  usb_sts_type status = USB_OK;
  uint8_t i_index = 0;

  if(puhost->class_handler->pdata == NULL)
  {
    return status;
  }

  for(i_index = 0; i_index < pmsc->max_lun ; i_index ++)
  {
    pmsc->l_unit_n[i_index].pre_state = USB_FAIL;
    pmsc->l_unit_n[i_index].change = 0;
    pmsc->l_unit_n[i_index].state = USBH_MSC_INIT;
    pmsc->l_unit_n[i_index].ready = MSC_NOT_READY;
  }

  if(pmsc->chin != 0 )
  {
    usbh_free_channel(puhost, pmsc->chin);
    usbh_ch_disable(puhost, pmsc->chin);
    pmsc->chin = 0;
  }

  if(pmsc->chout != 0 )
  {
    usbh_free_channel(puhost, pmsc->chout);
    usbh_ch_disable(puhost, pmsc->chout);
    pmsc->chout = 0;
  }

  return status;
}

/**
  * @brief  usb host hid class request handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type uhost_request_handler(void *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_msc_type *pmsc = (usbh_msc_type *)puhost->class_handler->pdata;
  usb_sts_type status = USB_WAIT;
  uint8_t i_index = 0;

  switch(pmsc->ctrl_state)
  {
    case USBH_MSC_STATE_IDLE:
      pmsc->ctrl_state = USBH_MSC_STATE_GET_LUN;
      break;
    case USBH_MSC_STATE_GET_LUN:
      if((status = usbh_msc_get_max_lun(uhost, (uint8_t *)&pmsc->max_lun)) == USB_OK)
      {
        pmsc->max_lun = (pmsc->max_lun & 0xFF) > USBH_SUPPORT_MAX_LUN ? USBH_SUPPORT_MAX_LUN:((pmsc->max_lun & 0xFF) + 1);
        USBH_DEBUG("Support max lun %d", pmsc->max_lun);
        for(i_index = 0; i_index < pmsc->max_lun ; i_index ++)
        {
          pmsc->l_unit_n[i_index].pre_state = USB_FAIL;
          pmsc->l_unit_n[i_index].change = 0;
          pmsc->l_unit_n[i_index].state = USBH_MSC_INIT;
        }
      }
      break;
    case USBH_MSC_STATE_ERROR:
      if((usbh_msc_clear_feature(uhost, 0)) == USB_OK)
      {
        pmsc->ctrl_state = USBH_MSC_STATE_GET_LUN;
      }
      break;
    default:
      break;
  }

  return status;
}

/**
  * @brief  usb host class process handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type uhost_process_handler(void *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_msc_type *pmsc = (usbh_msc_type *)puhost->class_handler->pdata;
  uint64_t msize = 0;
  usb_sts_type status;
  switch(pmsc->state)
  {
    case USBH_MSC_INIT:
      if(pmsc->cur_lun < pmsc->max_lun)
      {
        switch(pmsc->l_unit_n[pmsc->cur_lun].state)
        {
          case USBH_MSC_INIT:
            pmsc->l_unit_n[pmsc->cur_lun].ready = MSC_NOT_READY;
            pmsc->l_unit_n[pmsc->cur_lun].state = USBH_MSC_INQUIRY;
            break;
          case USBH_MSC_INQUIRY:
            status = usbh_msc_bot_scsi_get_inquiry(uhost, &pmsc->bot_trans, pmsc->cur_lun, &pmsc->l_unit_n[pmsc->cur_lun].inquiry);
            if(status == USB_OK)
            {
              pmsc->l_unit_n[pmsc->cur_lun].state = USBH_MSC_TEST_UNIT_READY;
            }
            else if(status == USB_FAIL)
            {
              pmsc->l_unit_n[pmsc->cur_lun].state = USBH_MSC_REQUEST_SENSE;
            }
            break;
          case USBH_MSC_TEST_UNIT_READY:
            status = usbh_msc_bot_scsi_test_unit_ready(uhost, &pmsc->bot_trans, pmsc->cur_lun);
            if(status == USB_OK)
            {
              pmsc->l_unit_n[pmsc->cur_lun].state = USBH_MSC_READ_CAPACITY10;
              pmsc->l_unit_n[pmsc->cur_lun].ready = MSC_OK;
            }
            else if(status == USB_FAIL)
            {
              pmsc->l_unit_n[pmsc->cur_lun].state = USBH_MSC_REQUEST_SENSE;
              pmsc->l_unit_n[pmsc->cur_lun].ready = MSC_NOT_READY;
            }
            break;

          case USBH_MSC_READ_CAPACITY10:
            status = usbh_msc_bot_scsi_capacity(uhost, &pmsc->bot_trans, pmsc->cur_lun, &pmsc->l_unit_n[pmsc->cur_lun].capacity);
            if(status == USB_OK)
            {
              msize = (uint64_t)pmsc->l_unit_n[pmsc->cur_lun].capacity.blk_nbr * (uint64_t)pmsc->l_unit_n[pmsc->cur_lun].capacity.blk_size;
              USBH_DEBUG("Device capacity: %llu Byte", msize);
              USBH_DEBUG("Block num: %d ", pmsc->l_unit_n[pmsc->cur_lun].capacity.blk_nbr);
              USBH_DEBUG("Block size: %d Byte", pmsc->l_unit_n[pmsc->cur_lun].capacity.blk_size);
              pmsc->l_unit_n[pmsc->cur_lun].state = USBH_MSC_IDLE;
              pmsc->cur_lun ++;
            }
            else if(status == USB_FAIL)
            {
              pmsc->l_unit_n[pmsc->cur_lun].state = USBH_MSC_REQUEST_SENSE;
            }
            break;
          case USBH_MSC_REQUEST_SENSE:
            status = usbh_msc_bot_scsi_request_sense(uhost, &pmsc->bot_trans, pmsc->cur_lun);
            if(status == USB_OK)
            {
              pmsc->l_unit_n[pmsc->cur_lun].state = USBH_MSC_TEST_UNIT_READY;
            }
            else if(status == USB_FAIL)
            {
              pmsc->l_unit_n[pmsc->cur_lun].ready = MSC_NOT_READY;
              pmsc->l_unit_n[pmsc->cur_lun].state = USBH_MSC_ERROR;
            }
            break;

          case USBH_MSC_BUSY:
            break;
          case USBH_MSC_ERROR:
            break;
          default:
            break;
        }
        
      }
      else
      {
        pmsc->state = USBH_MSC_IDLE;
      }
      break;
    case USBH_MSC_IDLE:
    if(puhost->user_handler->user_application != NULL)
    {
      puhost->user_handler->user_application();
    }
    default:
      break;
  }
  return USB_OK;
}


/**
  * @brief  usb host msc get max lun
  * @param  uhost: to the structure of usbh_core_type
  * @param  lun: max lun buffer
  * @retval status: usb_sts_type status
  */
static usb_sts_type usbh_msc_get_max_lun(void *uhost, uint8_t *lun)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usb_sts_type status = USB_WAIT;
  if(puhost->ctrl.state == CONTROL_IDLE )
  {
    puhost->ctrl.setup.bmRequestType = USB_DIR_D2H | USB_REQ_RECIPIENT_INTERFACE | USB_REQ_TYPE_CLASS;
    puhost->ctrl.setup.bRequest = MSC_REQ_GET_MAX_LUN;
    puhost->ctrl.setup.wValue = 0;
    puhost->ctrl.setup.wIndex = 0;
    puhost->ctrl.setup.wLength = 1;
    usbh_ctrl_request(puhost, lun, 1);
  }
  else
  {
    status = usbh_ctrl_result_check(puhost, CONTROL_IDLE, ENUM_IDLE);
    if(status == USB_OK || status == USB_NOT_SUPPORT)
    {
      status = USB_OK;
    }
  }
  return status;
}

/**
  * @brief  usb host msc clear feature
  * @param  uhost: to the structure of usbh_core_type
  * @param  ept_num: endpoint number
  * @retval status: usb_sts_type status
  */
static usb_sts_type usbh_msc_clear_feature(void *uhost, uint8_t ept_num)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usb_sts_type status = USB_WAIT;
  if(puhost->ctrl.state == CONTROL_IDLE )
  {
    puhost->ctrl.setup.bmRequestType = USB_DIR_H2D | USB_REQ_RECIPIENT_ENDPOINT | USB_REQ_TYPE_STANDARD;
    puhost->ctrl.setup.bRequest = USB_STD_REQ_CLEAR_FEATURE;
    puhost->ctrl.setup.wValue = 0;
    puhost->ctrl.setup.wIndex = ept_num;
    puhost->ctrl.setup.wLength = 0;
    usbh_ctrl_request(puhost, 0, 0);
  }
  else
  {
    status = usbh_ctrl_result_check(puhost, CONTROL_IDLE, ENUM_IDLE);
    if(status == USB_OK || status == USB_NOT_SUPPORT)
    {
      status = USB_OK;
    }
  }
  return status;
}

/**
  * @brief  usb host msc clear feature
  * @param  lun: logical unit number
  * @retval msc_error_type status
  */
msc_error_type usbh_msc_is_ready(void *uhost, uint8_t lun)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_msc_type *pmsc = (usbh_msc_type *)puhost->class_handler->pdata;
  return pmsc->l_unit_n[lun].ready;
}


/**
  * @brief  usb host msc read and write handle
  * @param  uhost: to the structure of usbh_core_type
  * @param  address: logical block address
  * @param  data_len: transfer data length
  * @param  buffer: transfer data buffer
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_msc_rw_handle(void *uhost, uint32_t address, uint32_t len, uint8_t *buffer, uint8_t lun)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_msc_type *pmsc = (usbh_msc_type *)puhost->class_handler->pdata;
  usb_sts_type status = USB_WAIT;
  switch(pmsc->l_unit_n[lun].state)
  {
    case USBH_MSC_READ10:
      status = usbh_msc_bot_scsi_read(uhost, &pmsc->bot_trans, address, buffer, len, lun);
      if(status == USB_OK)
      {
        pmsc->l_unit_n[lun].state = USBH_MSC_IDLE;
      }
      else if(status == USB_FAIL)
      {
        pmsc->l_unit_n[lun].state = USBH_MSC_REQUEST_SENSE;
        status = USB_WAIT;
      }
      break;
    case USBH_MSC_WRITE:
      status = usbh_msc_bot_scsi_write(uhost, &pmsc->bot_trans, address, buffer, len, lun);
      if(status == USB_OK)
      {
        pmsc->l_unit_n[lun].state = USBH_MSC_IDLE;
      }
      else if(status == USB_FAIL)
      {
        pmsc->l_unit_n[lun].state = USBH_MSC_REQUEST_SENSE;
        status = USB_WAIT;
      }
      break;
    case USBH_MSC_REQUEST_SENSE:
      status = usbh_msc_bot_scsi_request_sense(uhost, &pmsc->bot_trans, lun);
      if(status == USB_OK)
      {
        pmsc->l_unit_n[lun].state = USBH_MSC_IDLE;
        status = USB_FAIL;
      }
      else if(status == USB_FAIL)
      {
        USBH_DEBUG("device not support");
      }
      break;
    default:
      break;
  }
  return status;
}

/**
  * @brief  usb host msc read
  * @param  uhost: to the structure of usbh_core_type
  * @param  address: logical block address
  * @param  data_len: transfer data length
  * @param  buffer: transfer data buffer
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_msc_read(void *uhost, uint32_t address, uint32_t len, uint8_t *buffer, uint8_t lun)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_msc_type *pmsc = (usbh_msc_type *)puhost->class_handler->pdata;
  uint32_t timeout = 0;
  if(puhost->conn_sts == 0 || puhost->global_state != USBH_CLASS
    || pmsc->l_unit_n[lun].state != USBH_MSC_IDLE)
  {
    return USB_FAIL;
  }
  pmsc->bot_trans.msc_struct = &usbh_msc;
  pmsc->l_unit_n[lun].state = USBH_MSC_READ10;
  pmsc->use_lun = lun;

  timeout = puhost->timer;

  while(usbh_msc_rw_handle(uhost, address, len, buffer, lun) == USB_WAIT)
  {
    if(puhost->conn_sts == 0 || (puhost->timer - timeout) > (len * 10000))
    {
      pmsc->l_unit_n[lun].state = USBH_MSC_IDLE;
      return USB_FAIL;
    }
  }
  return USB_OK;
}

/**
  * @brief  usb host msc write
  * @param  uhost: to the structure of usbh_core_type
  * @param  address: logical block address
  * @param  data_len: transfer data length
  * @param  buffer: transfer data buffer
  * @param  lun: logical unit number
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_msc_write(void *uhost, uint32_t address, uint32_t len, uint8_t *buffer, uint8_t lun)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_msc_type *pmsc = (usbh_msc_type *)puhost->class_handler->pdata;
  uint32_t timeout = 0;
  if(puhost->conn_sts == 0 || puhost->global_state != USBH_CLASS
    || pmsc->l_unit_n[lun].state != USBH_MSC_IDLE)
  {
    return USB_FAIL;
  }

  pmsc->bot_trans.msc_struct = &usbh_msc;
  pmsc->l_unit_n[lun].state = USBH_MSC_WRITE;
  pmsc->use_lun = lun;

  timeout = puhost->timer;
  while(usbh_msc_rw_handle(uhost, address, len, buffer, lun) == USB_WAIT)
  {
    if(puhost->conn_sts == 0 || (puhost->timer - timeout) > (len * 10000))
    {
      pmsc->l_unit_n[lun].state = USBH_MSC_IDLE;
      return USB_FAIL;
    }
  }
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
