#ifndef __CDC_VCP_CH32H41X_H
#define __CDC_VCP_CH32H41X_H

#include "usbd_core.h"
#include "usbd_cdc_acm.h"


/*!< endpoint address */
#define CDC_IN_EP  0x82
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define USBD_VID            0x1A86
#define USBD_PID            0xE204
#define USBD_MAX_POWER      100
#define USBD_LANGID_STRING  1033

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_DESCRIPTOR_LEN)

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

extern struct cdc_line_coding  cdc_vcp_line_coding;
extern volatile uint8_t  ep_tx_busy_flag;
extern volatile uint8_t  usbd_cdc_info ;
extern volatile uint8_t  ep_rx_finish ;
extern volatile uint16_t ep_rx_length ;


void cdc_acm_init(uint8_t busid, uintptr_t reg_base);
uint16_t usb_vcp_rx_available(void);
// uint16_t usb_vcp_get_rx_data(uint8_t busid,uint8_t *buffer);
uint16_t usb_vcp_get_rx_data(uint8_t busid, uint8_t *buffer, uint32_t len);
uint16_t usb_vcp_send_data(uint8_t busid,uint8_t *buffer,uint16_t len);

#endif


