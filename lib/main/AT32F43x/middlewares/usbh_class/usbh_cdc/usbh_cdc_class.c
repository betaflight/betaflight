/**
  **************************************************************************
  * @file     usbh_cdc_class.c
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
#include "usbh_cdc_class.h"
#include "usb_conf.h"
#include "usbh_core.h"
#include "usbh_ctrl.h"
#include "string.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @defgroup usbh_cdc_class
  * @brief usb host class cdc demo
  * @{
  */

/** @defgroup usbh_cdc_class_private_functions
  * @{
  */

static usb_sts_type uhost_init_handler(void *uhost);
static usb_sts_type uhost_reset_handler(void *uhost);
static usb_sts_type uhost_request_handler(void *uhost);
static usb_sts_type uhost_process_handler(void *uhost);
static usb_sts_type get_linecoding(usbh_core_type *uhost, cdc_line_coding_type *linecoding);
static usb_sts_type set_linecoding(usbh_core_type *uhost, cdc_line_coding_type *linecoding);

static void cdc_process_transmission(usbh_core_type *uhost);
static void cdc_process_reception(usbh_core_type *uhost);
usbh_cdc_type usbh_cdc;

usbh_class_handler_type uhost_cdc_class_handler =
{
 uhost_init_handler,
 uhost_reset_handler,
 uhost_request_handler,
 uhost_process_handler,
 &usbh_cdc
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
  uint8_t if_x;
  usbh_cdc_type *pcdc = &usbh_cdc;
  puhost->class_handler->pdata = &usbh_cdc;

  memset((void *)pcdc, 0, sizeof(usbh_cdc_type));
  if_x = usbh_find_interface(puhost, USB_CLASS_CODE_CDC, ABSTRACT_CONTROL_MODEL, COMMON_AT_COMMAND);
  if(if_x == 0xFF)
  {
    USBH_DEBUG("cannot find the interface for communication interface class!");
    return USB_NOT_SUPPORT;
  }
  else
  {
    if(if_x < puhost->dev.cfg_desc.cfg.bNumInterfaces)
    {
      USBH_DEBUG ("switching to interface (#%d)", if_x);
      USBH_DEBUG ("class    : %xh", puhost->dev.cfg_desc.interface[if_x].interface.bInterfaceClass);
      USBH_DEBUG ("subclass : %xh", puhost->dev.cfg_desc.interface[if_x].interface.bInterfaceSubClass );
      USBH_DEBUG ("protocol : %xh", puhost->dev.cfg_desc.interface[if_x].interface.bInterfaceProtocol );   
      if(puhost->dev.cfg_desc.interface[if_x].interface.bInterfaceClass == COMMUNICATION_INTERFACE_CLASS_CODE)
      {
        USBH_DEBUG("CDC device!");
      }
    }
    else
    {
      USBH_DEBUG ("cannot select this interface."); 
    }
    
    /* collect the notification endpoint address and length */
    if(puhost->dev.cfg_desc.interface[if_x].endpoint[0].bEndpointAddress & 0x80)
    {
      pcdc->common_interface.notif_endpoint = puhost->dev.cfg_desc.interface[if_x].endpoint[0].bEndpointAddress;
      pcdc->common_interface.notif_endpoint_size  = puhost->dev.cfg_desc.interface[if_x].endpoint[0].wMaxPacketSize;
    }
    /* allocate the length for host channel number in */
    pcdc->common_interface.notif_channel = usbh_alloc_channel(puhost, pcdc->common_interface.notif_endpoint);
    
    /* enable channel */
    usbh_hc_open(puhost, 
                 pcdc->common_interface.notif_channel,
                 pcdc->common_interface.notif_endpoint,
                 puhost->dev.address,
                 EPT_INT_TYPE,
                 pcdc->common_interface.notif_endpoint_size,
                 puhost->dev.speed);
    
    usbh_set_toggle(puhost, pcdc->common_interface.notif_channel, 0);
    
    
    if_x = usbh_find_interface(puhost, DATA_INTERFACE_CLASS_CODE, RESERVED, NO_CLASS_SPECIFIC_PROTOCOL_CODE);
    if(if_x == 0xFF)
    {
      USBH_DEBUG("cannot find the interface for data interface class!");
      return USB_NOT_SUPPORT;
    }
    else
    {
      /* collect the class specific endpoint address and length */
      if(puhost->dev.cfg_desc.interface[if_x].endpoint[0].bEndpointAddress & 0x80)
      {      
        pcdc->data_interface.in_endpoint = puhost->dev.cfg_desc.interface[if_x].endpoint[0].bEndpointAddress;
        pcdc->data_interface.in_endpoint_size  = puhost->dev.cfg_desc.interface[if_x].endpoint[0].wMaxPacketSize;
      }
      else
      {
        pcdc->data_interface.out_endpoint = puhost->dev.cfg_desc.interface[if_x].endpoint[0].bEndpointAddress;
        pcdc->data_interface.out_endpoint_size  = puhost->dev.cfg_desc.interface[if_x].endpoint[0].wMaxPacketSize;
      }
      
      if(puhost->dev.cfg_desc.interface[if_x].endpoint[1].bEndpointAddress & 0x80)
      {      
        pcdc->data_interface.in_endpoint = puhost->dev.cfg_desc.interface[if_x].endpoint[1].bEndpointAddress;
        pcdc->data_interface.in_endpoint_size  = puhost->dev.cfg_desc.interface[if_x].endpoint[1].wMaxPacketSize;
      }
      else
      {
        pcdc->data_interface.out_endpoint = puhost->dev.cfg_desc.interface[if_x].endpoint[1].bEndpointAddress;
        pcdc->data_interface.out_endpoint_size  = puhost->dev.cfg_desc.interface[if_x].endpoint[1].wMaxPacketSize;
      }
      
      /* allocate the length for host channel number in */
      pcdc->data_interface.in_channel = usbh_alloc_channel(puhost, pcdc->data_interface.in_endpoint);
      /* allocate the length for host channel number out */
      pcdc->data_interface.out_channel = usbh_alloc_channel(puhost, pcdc->data_interface.out_endpoint);
      
      /* enable in channel */
      usbh_hc_open(puhost, 
                   pcdc->data_interface.in_channel,
                   pcdc->data_interface.in_endpoint,
                   puhost->dev.address,
                   EPT_BULK_TYPE,
                   pcdc->data_interface.in_endpoint_size,
                   puhost->dev.speed);
      
      /* enable out channel */
      usbh_hc_open(puhost, 
                   pcdc->data_interface.out_channel,
                   pcdc->data_interface.out_endpoint,
                   puhost->dev.address,
                   EPT_BULK_TYPE,
                   pcdc->data_interface.out_endpoint_size,
                   puhost->dev.speed);
                   
      usbh_set_toggle(puhost, pcdc->data_interface.in_channel, 0);
      usbh_set_toggle(puhost, pcdc->data_interface.out_channel, 0);     
      
      pcdc->state = CDC_IDLE_STATE;
      
    }
  }
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
  usbh_cdc_type *pcdc = (usbh_cdc_type *)puhost->class_handler->pdata;
  usb_sts_type status = USB_OK;

  if(puhost->class_handler->pdata == NULL)
  {
    return status;
  }

  if(pcdc->common_interface.notif_channel != 0 )
  {
    usbh_free_channel(puhost, pcdc->common_interface.notif_channel);
    usbh_ch_disable(puhost, pcdc->common_interface.notif_channel);
    pcdc->common_interface.notif_channel = 0;
  }
  
  if(pcdc->data_interface.in_channel != 0 )
  {
    usbh_free_channel(puhost, pcdc->data_interface.in_channel);
    usbh_ch_disable(puhost, pcdc->data_interface.in_channel);
    pcdc->data_interface.in_channel = 0;
  }

  if(pcdc->data_interface.out_channel != 0 )
  {
    usbh_free_channel(puhost, pcdc->data_interface.out_channel);
    usbh_ch_disable(puhost, pcdc->data_interface.out_channel);
    pcdc->data_interface.out_channel = 0;
  }

  return status;
}

/**
  * @brief  usb host cdc class request handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type uhost_request_handler(void *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_cdc_type *pcdc = (usbh_cdc_type *)puhost->class_handler->pdata;
  usb_sts_type status = USB_WAIT;
  
  status = get_linecoding(uhost, &pcdc->linecoding);

  return status;
}

/**
  * @brief  usb host cdc get linecoding handler
  * @param  uhost: to the structure of usbh_core_type
  * @param  linecoding: pointer to the structure of cdc_line_coding_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type get_linecoding(usbh_core_type *uhost, cdc_line_coding_type *linecoding)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usb_sts_type status = USB_WAIT;
  if(puhost->ctrl.state == CONTROL_IDLE )
  {
    uhost->ctrl.setup.bmRequestType = USB_DIR_D2H | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
    uhost->ctrl.setup.bRequest = CDC_GET_LINE_CODING;
    uhost->ctrl.setup.wValue = 0;
    uhost->ctrl.setup.wLength = LINE_CODING_STRUCTURE_SIZE;
    uhost->ctrl.setup.wIndex = 0;

    usbh_ctrl_request(uhost, linecoding->array, LINE_CODING_STRUCTURE_SIZE);
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
  * @brief  usb host cdc set linecoding handler
  * @param  uhost: to the structure of usbh_core_type
  * @param  linecoding: pointer to the structure of cdc_line_coding_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type set_linecoding(usbh_core_type *uhost, cdc_line_coding_type *linecoding)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usb_sts_type status = USB_WAIT;
  
  if(puhost->ctrl.state == CONTROL_IDLE )
  {
    uhost->ctrl.setup.bmRequestType = USB_DIR_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
    uhost->ctrl.setup.bRequest = CDC_SET_LINE_CODING;
    uhost->ctrl.setup.wValue = 0;
    uhost->ctrl.setup.wLength = LINE_CODING_STRUCTURE_SIZE;
    uhost->ctrl.setup.wIndex = 0;

    status = usbh_ctrl_request(uhost, linecoding->array, LINE_CODING_STRUCTURE_SIZE);
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
  * @brief  usb host class process handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type uhost_process_handler(void *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_cdc_type *pcdc = (usbh_cdc_type *)puhost->class_handler->pdata;
  usb_sts_type status;
  
  switch(pcdc->state)
  {
    case CDC_IDLE_STATE:
      status = USB_OK;
    break;
    
    case CDC_SET_LINE_CODING_STATE:
      status = set_linecoding(puhost, pcdc->puserlinecoding);
      if(status == USB_OK)
        pcdc->state = CDC_GET_LAST_LINE_CODING_STATE;
    break;
    
    case CDC_GET_LAST_LINE_CODING_STATE:
      status = get_linecoding(puhost, &(pcdc->linecoding));
      if(status == USB_OK)
      {
        pcdc->state = CDC_IDLE_STATE;
        if((pcdc->linecoding.line_coding_b.char_format == pcdc->puserlinecoding->line_coding_b.char_format)&&
           (pcdc->linecoding.line_coding_b.data_bits == pcdc->puserlinecoding->line_coding_b.data_bits)&&
           (pcdc->linecoding.line_coding_b.parity_type == pcdc->puserlinecoding->line_coding_b.parity_type)&&
           (pcdc->linecoding.line_coding_b.data_baudrate == pcdc->puserlinecoding->line_coding_b.data_baudrate))
        {
          /* line coding changed */
        } 
      }
      pcdc->state = CDC_IDLE_STATE;
    break;
    
    case CDC_TRANSFER_DATA:
      cdc_process_transmission(puhost);
      cdc_process_reception(puhost);   
    break;
    
    case CDC_ERROR_STATE:
      status = usbh_clear_ept_feature(puhost, 0, pcdc->common_interface.notif_channel);
      if(status == USB_OK)
        pcdc->state = CDC_IDLE_STATE;
    break;
    
    default:
    break;
  }
  
  return status;
}

/**
  * @brief  usb host cdc class process transmission handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static void cdc_process_transmission(usbh_core_type *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_cdc_type *pcdc = (usbh_cdc_type *)puhost->class_handler->pdata;

  switch(pcdc->data_tx_state)
  {
    case CDC_SEND_DATA:
      if(pcdc->tx_len > pcdc->data_interface.out_endpoint_size)
      {
        usbh_bulk_send(puhost, pcdc->data_interface.out_channel, (uint8_t*)pcdc->tx_data, pcdc->data_interface.out_endpoint_size);
      }
      else
      {
        usbh_bulk_send(puhost, pcdc->data_interface.out_channel, (uint8_t*)pcdc->tx_data, pcdc->tx_len);
      }
      pcdc->data_tx_state = CDC_SEND_DATA_WAIT;
    break;
    
    case CDC_SEND_DATA_WAIT:
      if(uhost->urb_state[pcdc->data_interface.out_channel] == URB_DONE)
      {
        if(pcdc->tx_len > pcdc->data_interface.out_endpoint_size)
        {
          pcdc->tx_len -= pcdc->data_interface.out_endpoint_size;
          pcdc->tx_data += pcdc->data_interface.out_endpoint_size;
          pcdc->data_tx_state = CDC_SEND_DATA;
        }
        else
        {
          pcdc->tx_len = 0;
          pcdc->data_tx_state = CDC_IDLE;
          cdc_transmit_complete(uhost);
        }
      }
      else if( uhost->urb_state[pcdc->data_interface.out_channel] == URB_NOTREADY)
      {
        pcdc->data_tx_state = CDC_SEND_DATA;
      }
    break;
    
    default:
    break;
  }
}

/**
  * @brief  usb host cdc class start transmission handler
  * @param  uhost: to the structure of usbh_core_type
  * @param  data: tx data pointer
  * @param  len: tx data len
  * @retval status: usb_sts_type status
  */
void cdc_start_transmission(usbh_core_type *uhost, uint8_t *data, uint32_t len)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_cdc_type *pcdc = (usbh_cdc_type *)puhost->class_handler->pdata;
  if(pcdc->data_tx_state == CDC_IDLE)
  {
    pcdc->data_tx_state = CDC_SEND_DATA;
    pcdc->state = CDC_TRANSFER_DATA;
    pcdc->tx_data = data;
    pcdc->tx_len = len;
  }
}


/**
  * @brief  usb host cdc class transmit complete
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
__weak void cdc_transmit_complete(usbh_core_type *uhost)
{
  
}

/**
  * @brief  usb host cdc class process reception handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static void cdc_process_reception(usbh_core_type *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_cdc_type *pcdc = (usbh_cdc_type *)puhost->class_handler->pdata;
  uint32_t len = 0;
   
  switch(pcdc->data_rx_state)
  {
    case CDC_RECEIVE_DATA:
      usbh_bulk_recv(puhost, pcdc->data_interface.in_channel, (uint8_t*)pcdc->rx_data, pcdc->data_interface.in_endpoint_size);
      pcdc->data_rx_state = CDC_RECEIVE_DATA_WAIT;
    break;
    
    case CDC_RECEIVE_DATA_WAIT:
      if(uhost->urb_state[pcdc->data_interface.in_channel] == URB_DONE)
      {
        len = uhost->hch[pcdc->data_interface.in_channel].trans_count;
        if(pcdc->rx_len > len && len > pcdc->data_interface.in_endpoint_size)
        {
          pcdc->rx_len -= len;
          pcdc->rx_data += len;
          pcdc->data_rx_state = CDC_RECEIVE_DATA;
        }
        else
        {
          pcdc->data_rx_state = CDC_IDLE;
          cdc_receive_complete(uhost);
          
        }
      }

    break;
    
    default:
    break;
  }
}

/**
  * @brief  usb host cdc class start reception handler
  * @param  uhost: to the structure of usbh_core_type
  * @param  data: receive data pointer
  * @param  len: receive data len
  * @retval status: usb_sts_type status
  */
void cdc_start_reception(usbh_core_type *uhost, uint8_t *data, uint32_t len)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_cdc_type *pcdc = (usbh_cdc_type *)puhost->class_handler->pdata;
  
  if(pcdc->data_rx_state == CDC_IDLE)
  {
    pcdc->data_rx_state = CDC_RECEIVE_DATA;
    pcdc->state = CDC_TRANSFER_DATA;
    pcdc->rx_data = data;
    pcdc->rx_len = len;
  }
}

/**
  * @brief  usb host cdc class reception complete
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
__weak void cdc_receive_complete(usbh_core_type *uhost)
{
  
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
