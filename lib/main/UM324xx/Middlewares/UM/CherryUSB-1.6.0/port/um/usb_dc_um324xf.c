/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usb_dc_um324xf.h"

#undef CONFIG_USBDEV_EP_NUM
#define CONFIG_USBDEV_EP_NUM 5

/* Endpoint state */
struct um324xf_ep_state {
    uint16_t ep_mps;         /* Endpoint max packet size */
    uint8_t ep_type;         /* Endpoint type */
    uint32_t ep_pma_addr;    /* ep pmd allocated addr */
    uint8_t *xfer_buf;
    uint32_t xfer_len;
    uint32_t actual_xfer_len;
};

/* Driver state */
struct um324xf_udc {
    struct usb_setup_packet setup;
    volatile uint8_t dev_addr;                          /*!< USB Address */
    struct um324xf_ep_state in_ep[CONFIG_USBDEV_EP_NUM];  /*!< IN endpoint parameters*/
    struct um324xf_ep_state out_ep[CONFIG_USBDEV_EP_NUM]; /*!< OUT endpoint parameters */
} g_um324xf_udc;

__WEAK void usb_dc_low_level_init(void)
{
}

__WEAK void usb_dc_low_level_deinit(void)
{
}

int usb_dc_init(uint8_t busid)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	UNUSED(busid);

	usb_dc_low_level_init();
	
	__HAL_RCM_GPIOA_CLK_ENABLE();
	__HAL_RCM_GPIOA_RELEASE_RESET();

	/* USB_DM£ºPA11  USB_DP£ºPA12 */
	GPIO_InitStructFunc(&GPIO_InitStruct);
	GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	__HAL_RCM_USB_FORCE_RESET();
	__HAL_RCM_USB_CLK_DISABLE(); 
	__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();
	__HAL_RCM_USB_CLK_ENABLE(); 
	__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();
	while ((RCM->AHB0CKENR & RCM_AHB0CKENR_USBEN) == 0);

	__HAL_RCM_USB_RELEASE_RESET(); 
	__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();

	HAL_NVIC_SetPriority(USB0_IRQn, 3, 3);
	HAL_NVIC_EnableIRQ(USB0_IRQn);

	/*** Reset ***/
	_SetWORKINGMODE(USB_FORCE_RST);
	__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();
	_SetWORKINGMODE(0);
	__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();

	/*** Clear pending interrupts ***/
	_ClrFlag(INT_ALL);

	/*** Set interrupt ***/
	_SetINTEN(INT_BUS_RESET | INT_ACK);

#ifdef CONFIG_USBDEV_SOF_ENABLE
	USB->INTEN |= INT_SOF;
#endif

	/*** Start ***/
	_SetWORKINGMODE(USB_SPEED_MODE | USB_BUS_AUTO_RST_EN | USB_DATA_NOEOP_EN | USB_TOKEN_NOEOP_EN | USB_DPPU | USB_DPPU_LO);

	return 0;
}

int usb_dc_deinit(uint8_t busid)
{
	UNUSED(busid);
	
	HAL_NVIC_DisableIRQ(USB0_IRQn);
	
	__HAL_RCM_USB_FORCE_RESET();
	__HAL_RCM_USB_CLK_DISABLE();
	__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();

    usb_dc_low_level_deinit();
    return 0;
}

int usbd_set_address(uint8_t busid, const uint8_t addr)
{
	UNUSED(busid);
	
    g_um324xf_udc.dev_addr = addr;
    return 0;
}

int usbd_set_remote_wakeup(uint8_t busid)
{
	UNUSED(busid);
	
    return -1;
}

uint8_t usbd_get_port_speed(uint8_t busid)
{
	UNUSED(busid);
	
    return USB_SPEED_FULL;
}

int usbd_ep_open(uint8_t busid, const struct usb_endpoint_descriptor *ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep->bEndpointAddress);
	
	UNUSED(busid);
	
    if (USB_EP_DIR_IS_OUT(ep->bEndpointAddress)) {
        g_um324xf_udc.out_ep[ep_idx].ep_mps = USB_GET_MAXPACKETSIZE(ep->wMaxPacketSize);
        g_um324xf_udc.out_ep[ep_idx].ep_type = USB_GET_ENDPOINT_TYPE(ep->bmAttributes);
		g_um324xf_udc.out_ep[ep_idx].ep_pma_addr = _GetEPTxAddr(ep_idx);
    } else {
        g_um324xf_udc.in_ep[ep_idx].ep_mps = USB_GET_MAXPACKETSIZE(ep->wMaxPacketSize);
        g_um324xf_udc.in_ep[ep_idx].ep_type = USB_GET_ENDPOINT_TYPE(ep->bmAttributes);
		g_um324xf_udc.in_ep[ep_idx].ep_pma_addr = _GetEPRxAddr(ep_idx);
    }
	_SetEPOpen(ep_idx);
    return 0;
}

int usbd_ep_close(uint8_t busid, const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
	
	UNUSED(busid);
	
	_SetEPClose(ep_idx);
    return 0;
}

int usbd_ep_set_stall(uint8_t busid, const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
	
	UNUSED(busid);
	
	_SetEPStall(ep_idx);
    return 0;
}

int usbd_ep_clear_stall(uint8_t busid, const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
	
	UNUSED(busid);
	
	_ClrEPStall(ep_idx);
    return 0;
}

int usbd_ep_is_stalled(uint8_t busid, const uint8_t ep, uint8_t *stalled)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
	
	UNUSED(busid);
	
    if (_IsEPStall(ep_idx)) {
		*stalled = 1;
    } else {
		*stalled = 0;
    }
    return 0;
}

int usbd_set_addr_status(const uint8_t *data)
{
    g_um324xf_udc.in_ep[0].xfer_buf = (uint8_t *)data;
    g_um324xf_udc.in_ep[0].xfer_len = 0;
    g_um324xf_udc.in_ep[0].actual_xfer_len = 0;

	_SetEPTxCount(0, 0);

    return 0;
}

int usbd_ep0_read(uint8_t *data, uint32_t data_len)
{
    g_um324xf_udc.out_ep[0].xfer_buf = data;
    g_um324xf_udc.out_ep[0].xfer_len = data_len;
    g_um324xf_udc.out_ep[0].actual_xfer_len = 0;
	
    return 0;
}

int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
	
	UNUSED(busid);

    if (!data && data_len) {
        return -1;
    }

    g_um324xf_udc.in_ep[ep_idx].xfer_buf = (uint8_t *)data;
    g_um324xf_udc.in_ep[ep_idx].xfer_len = data_len;
    g_um324xf_udc.in_ep[ep_idx].actual_xfer_len = 0;

    data_len = MIN(data_len, g_um324xf_udc.in_ep[ep_idx].ep_mps);

    um324xf_write_pma((uint8_t *)data, g_um324xf_udc.in_ep[ep_idx].ep_pma_addr, (uint8_t)data_len);
	_SetEPTxCount(ep_idx, (uint8_t)data_len);
	_SetTxValid(ep_idx);

    return 0;
}

int usbd_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t data_len)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
	
	UNUSED(busid);

    if (!data && data_len) {
        return -1;
    }

    g_um324xf_udc.out_ep[ep_idx].xfer_buf = data;
    g_um324xf_udc.out_ep[ep_idx].xfer_len = data_len;
    g_um324xf_udc.out_ep[ep_idx].actual_xfer_len = 0;
	_SetRxValid(ep_idx);

    return 0;
}

void USB0_IRQHandler(uint8_t busid)
{
    uint32_t wIstr;
    uint8_t ep_idx;
    uint8_t read_count, write_count;
	
	UNUSED(busid);

    wIstr = _GetFlag();
	
	/* ACK */
    if (wIstr & INT_ACK) {
		/* stay in loop while pending interrupts */
        while ((_GetFlag() & INT_ACK) != 0U) {
            wIstr = _GetFlag();

            /* extract highest priority endpoint number */
            ep_idx = (uint8_t)_GetEPIndex(wIstr);
			
			/* EP0 */
            if (ep_idx == 0U) {
				/* SETUP */
				if ((wIstr & INT_SUDAV) != 0U) {
					_ClrFlag(INT_EP0_ACK | INT_SUDAV);

					g_um324xf_udc.setup.bmRequestType = (USBD->SETUP03DATA >> 0);
					g_um324xf_udc.setup.bRequest = (USBD->SETUP03DATA >> 8);
					g_um324xf_udc.setup.wValue = (USBD->SETUP03DATA >> 16);
					g_um324xf_udc.setup.wIndex = (USBD->SETUP47DATA >> 0);
					g_um324xf_udc.setup.wLength = (USBD->SETUP47DATA >> 16);

					usbd_event_ep0_setup_complete_handler(0, (uint8_t *)&g_um324xf_udc.setup);
				/* OUT */
				} else if ((wIstr & INT_EP0_OUT) != 0U) {
					_ClrFlag(INT_EP0_ACK | INT_EP0_OUT);

					read_count = _GetEPRxCount(ep_idx);
					
					um324xf_read_pma(g_um324xf_udc.out_ep[ep_idx].xfer_buf, g_um324xf_udc.out_ep[ep_idx].ep_pma_addr, (uint8_t)read_count);

					g_um324xf_udc.out_ep[ep_idx].xfer_buf += read_count;
					g_um324xf_udc.out_ep[ep_idx].xfer_len -= read_count;
					g_um324xf_udc.out_ep[ep_idx].actual_xfer_len += read_count;

					usbd_event_ep_out_complete_handler(0, ep_idx, g_um324xf_udc.out_ep[ep_idx].actual_xfer_len);

					if (read_count == 0) {
						/* Out status, start reading setup */
						usbd_ep_start_read(0, 0x00, NULL, 0);
					}
				/* IN */
                } else {
                    _ClrFlag(INT_EP0_ACK);

                    write_count = _GetEPTxCount(ep_idx);
					
					g_um324xf_udc.in_ep[ep_idx].xfer_buf += write_count;
					g_um324xf_udc.in_ep[ep_idx].xfer_len -= write_count;
					g_um324xf_udc.in_ep[ep_idx].actual_xfer_len += write_count;

					usbd_event_ep_in_complete_handler(0, ep_idx | 0x80, g_um324xf_udc.in_ep[ep_idx].actual_xfer_len);
                }
			/* EP1��EP2��EP3��EP4 */
            } else {
				/* OUT */
                if ((_IsEPOut(wIstr, ep_idx)) != 0U) {
					/* clear int flag */
					if (ep_idx == ENDP1)      _ClrFlag(INT_EP1_ACK | INT_EP1_OUT);
					else if (ep_idx == ENDP2) _ClrFlag(INT_EP2_ACK | INT_EP2_OUT);
					else if (ep_idx == ENDP3) _ClrFlag(INT_EP3_ACK | INT_EP3_OUT);
					else                      _ClrFlag(INT_EP4_ACK | INT_EP4_OUT);
					
                    read_count = _GetEPRxCount(ep_idx);
                    um324xf_read_pma(g_um324xf_udc.out_ep[ep_idx].xfer_buf, g_um324xf_udc.out_ep[ep_idx].ep_pma_addr, read_count);
                    g_um324xf_udc.out_ep[ep_idx].xfer_buf += read_count;
                    g_um324xf_udc.out_ep[ep_idx].xfer_len -= read_count;
                    g_um324xf_udc.out_ep[ep_idx].actual_xfer_len += read_count;

                    if ((read_count < g_um324xf_udc.out_ep[ep_idx].ep_mps) ||
                        (g_um324xf_udc.out_ep[ep_idx].xfer_len == 0)) {
                        usbd_event_ep_out_complete_handler(0, ep_idx, g_um324xf_udc.out_ep[ep_idx].actual_xfer_len);
                    } else {
                        _SetRxValid(ep_idx);
                    }
				/* IN */	
                } else {
					/* clear int flag */
					if (ep_idx == ENDP1)      _ClrFlag(INT_EP1_ACK);
					else if (ep_idx == ENDP2) _ClrFlag(INT_EP2_ACK);
					else if (ep_idx == ENDP3) _ClrFlag(INT_EP3_ACK);
					else                      _ClrFlag(INT_EP4_ACK);
					
                    write_count = _GetEPTxCount(ep_idx);
                    g_um324xf_udc.in_ep[ep_idx].xfer_buf += write_count;
                    g_um324xf_udc.in_ep[ep_idx].xfer_len -= write_count;
                    g_um324xf_udc.in_ep[ep_idx].actual_xfer_len += write_count;

                    if (g_um324xf_udc.in_ep[ep_idx].xfer_len == 0) {
                        usbd_event_ep_in_complete_handler(0, ep_idx | 0x80, g_um324xf_udc.in_ep[ep_idx].actual_xfer_len);
                    } else {
                        write_count = MIN(g_um324xf_udc.in_ep[ep_idx].xfer_len, g_um324xf_udc.in_ep[ep_idx].ep_mps);
                        um324xf_write_pma(g_um324xf_udc.in_ep[ep_idx].xfer_buf, g_um324xf_udc.in_ep[ep_idx].ep_pma_addr, write_count);
						_SetEPTxCount(ep_idx, write_count);
						_SetTxValid(ep_idx);
                    }
                }
            }
        }
		
		return;
    }
	
	/* BUS RESET */
	if (wIstr & INT_BUS_RESET)
	{
		_ClrFlag(INT_BUS_RESET);
        memset(&g_um324xf_udc, 0, sizeof(struct um324xf_udc));
        usbd_event_reset_handler(0);
		return;
	}
#ifdef CONFIG_USBDEV_SOF_ENABLE
	/* SOF */
	if (wIstr & INT_SOF)
	{
		_ClrFlag(INT_SOF);
		return;
	}
#endif
}

/**
  * @brief   Copy a buffer from user memory area to packet memory area (PMA)
  * @param   USBx USB peripheral instance register address.
  * @param   pbUsrBuf pointer to user memory area.
  * @param   wPMABufAddr address into PMA.
  * @param   wNBytes no. of bytes to be copied.
  * @retval  None
  */
void um324xf_write_pma(uint8_t *pbUsrBuf, uint32_t wPMABufAddr, uint8_t wNBytes)
{
	uint8_t i = 0;
	
	for (i = 0; i < wNBytes; i++)
	{
		*(__IO uint8_t *)wPMABufAddr = pbUsrBuf[i];
		wPMABufAddr++;
	}
}

/**
  * @brief   Copy data from packet memory area (PMA) to user memory buffer
  * @param   USBx USB peripheral instance register address.
  * @param   pbUsrBuf pointer to user memory area.
  * @param   wPMABufAddr address into PMA.
  * @param   wNBytes no. of bytes to be copied.
  * @retval  None
  */
void um324xf_read_pma(uint8_t *pbUsrBuf, uint32_t wPMABufAddr, uint8_t wNBytes)
{
	uint8_t i = 0;
	
	for (i = 0; i < wNBytes; i++)
	{
		pbUsrBuf[i] = *(__IO uint8_t *)wPMABufAddr;
		wPMABufAddr++;
	}
}

