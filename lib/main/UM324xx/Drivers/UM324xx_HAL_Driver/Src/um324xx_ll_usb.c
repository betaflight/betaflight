
/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

#if defined (HAL_PCD_MODULE_ENABLED)

/**
  * @brief  USB_EnableGlobalInt
  *         Enables the controller's Global Int in the AHB Config reg
  * @param  USBx Selected device
  * @retval HAL status
  */
HAL_StatusTypeDef USB_EnableGlobalInt(USB_TypeDef *USBx)
{
	uint32_t winterruptmask;

	/* Clear pending interrupts */
	USBx->INTCLR = 0xFFFFFFFF;

	/* Set winterruptmask variable */
	winterruptmask = USB_INTEN_EP0_ACK_EN_Msk | USB_INTEN_EP1_ACK_EN_Msk |
	               USB_INTEN_EP2_ACK_EN_Msk | USB_INTEN_EP3_ACK_EN_Msk | USB_INTEN_EP4_ACK_EN_Msk |
	               USB_INTEN_BUS_RESET_EN_Msk | USB_INTEN_SOF_EN_Msk;

	/* Set interrupt mask */
	USBx->INTEN = winterruptmask;

	return HAL_OK;
}

/**
  * @brief  USB_DisableGlobalInt
  *         Disable the controller's Global Int in the AHB Config reg
  * @param  USBx Selected device
  * @retval HAL status
  */
HAL_StatusTypeDef USB_DisableGlobalInt(USB_TypeDef *USBx)
{
	/* Clear interrupt mask */
	USBx->INTEN = 0x00000000;

	return HAL_OK;
}

/**
  * @brief  USB_DevInit Initializes the USB controller registers
  *         for device mode
  * @param  USBx Selected device
  * @param  cfg  pointer to a USB_CfgTypeDef structure that contains
  *         the configuration information for the specified USBx peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef USB_DevInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg)
{
	UNUSED(cfg);
	
	/* Init Device */
	/* USB_FORCE_RST = 1 */
	USBx->WORKINGMODE = USB_WORKINGMODE_USB_FORCE_RST_Msk;
	
	/* USB_FORCE_RST = 0 */
	USBx->WORKINGMODE = USB_WORKINGMODE_SPEED_MODE_Msk | USB_WORKINGMODE_USB_BUS_AUTO_RST_EN_Msk;
	
	/* Clear pending interrupts */
	USBx->INTCLR = 0xFFFFFFFF;

	return HAL_OK;
}

/**
  * @brief  Activate and configure an endpoint
  * @param  USBx Selected device
  * @param  ep pointer to endpoint structure
  * @retval HAL status
  */
HAL_StatusTypeDef USB_ActivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep)
{
	uint32_t _wRegVal = PCD_GET_EPCSR(USBx, ep->num);
	
	_wRegVal &= (~USB_EPxCSR_EPx_SEND_STALL_Msk);
	
	_wRegVal |= (USB_EPxCSR_EPx_EN_Msk | USB_EPxCSR_EPx_FIFOCLR_Msk | USB_EPxCSR_EPx_RECEIVED_DONE_Msk);
	
	PCD_SET_EPCSR(USBx, ep->num, _wRegVal);
	
	PCD_SET_EP_ADDRESS(USBx, ep->num, ep->num);
  
	return HAL_OK;
}

/**
  * @brief  De-activate and de-initialize an endpoint
  * @param  USBx Selected device
  * @param  ep pointer to endpoint structure
  * @retval HAL status
  */
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep)
{
	uint32_t _wRegVal = PCD_GET_EPCSR(USBx, ep->num);
	
	_wRegVal &= (~(USB_EPxCSR_EPx_EN_Msk | USB_EPxCSR_EPx_SEND_STALL_Msk));
	
	_wRegVal |= (USB_EPxCSR_EPx_FIFOCLR_Msk | USB_EPxCSR_EPx_RECEIVED_DONE_Msk);
	
	PCD_SET_EPCSR(USBx, ep->num, _wRegVal);
	
	return HAL_OK;
}

/**
  * @brief  USB_EPStartXfer setup and starts a transfer over an EP
  * @param  USBx Selected device
  * @param  ep pointer to endpoint structure
  * @retval HAL status
  */
HAL_StatusTypeDef USB_EPStartXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep)
{
	uint32_t len;

	/* IN endpoint */
	if (ep->is_in == 1U)
	{
		/*Multi packet transfer*/
		if (ep->xfer_len > ep->maxpacket)
		{
			len = ep->maxpacket;
		}
		else
		{
			len = ep->xfer_len;
		}
		ep->xfer_count_last = len;
		
		USB_WritePMA(USBx, ep->xfer_buff, ep->pmaadress, (uint16_t)len);
		PCD_SET_EP_TX_CNT(USBx, ep->num, len);
		PCD_SET_EP_TX_VALID(USBx, ep->num);
	}
	else /* OUT endpoint */
	{
		/* Multi packet transfer */
		if (ep->xfer_len > ep->maxpacket)
		{
			len = ep->maxpacket;
			ep->xfer_len -= len;
		}
		else
		{
			len = ep->xfer_len;
			ep->xfer_len = 0U;
		}

		PCD_SET_EP_RX_VALID(USBx, ep->num);
	}

	return HAL_OK;
}

/**
  * @brief  USB_EPSetStall set a stall condition over an EP
  * @param  USBx Selected device
  * @param  ep pointer to endpoint structure
  * @retval HAL status
  */
HAL_StatusTypeDef USB_EPSetStall(USB_TypeDef *USBx, USB_EPTypeDef *ep)
{
	PCD_SET_EP_STALL(USBx, ep->num);

	return HAL_OK;
}

/**
  * @brief  USB_EPClearStall Clear a stall condition over an EP
  * @param  USBx Selected device
  * @param  ep pointer to endpoint structure
  * @retval HAL status
  */
HAL_StatusTypeDef USB_EPClearStall(USB_TypeDef *USBx, USB_EPTypeDef *ep)
{
	PCD_CLR_EP_STALL(USBx, ep->num);

	return HAL_OK;
}

/**
  * @brief  USB_StopDevice Stop the usb device mode
  * @param  USBx Selected device
  * @retval HAL status
  */
HAL_StatusTypeDef USB_StopDevice(USB_TypeDef *USBx)
{
	/* disable all interrupts and force USB reset */
	USBx->INTEN = 0x00000000;
	USBx->WORKINGMODE = USB_WORKINGMODE_USB_FORCE_RST_Msk;

	/* clear interrupt status register */
	USBx->INTCLR = 0xFFFFFFFF;

	return HAL_OK;
}

/**
  * @brief  USB_ReadInterrupts return the global USB interrupt status
  * @param  USBx Selected device
  * @retval USB Global Interrupt status
  */
uint32_t USB_ReadInterrupts(USB_TypeDef const *USBx)
{
	uint32_t tmpreg;

	tmpreg = USBx->INTSTATRAW;
	
	return tmpreg;
}

/**
  * @brief  USB_ActivateRemoteWakeup : active remote wakeup signalling
  * @param  USBx Selected device
  * @retval HAL status
  */
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_TypeDef *USBx)
{
	USB->WORKINGMODE |= USB_WORKINGMODE_USB_REMOTE_WAKEUP_Msk;

	return HAL_OK;
}

/**
  * @brief  USB_DeActivateRemoteWakeup de-active remote wakeup signalling
  * @param  USBx Selected device
  * @retval HAL status
  */
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_TypeDef *USBx)
{
	USB->WORKINGMODE &= (~USB_WORKINGMODE_USB_REMOTE_WAKEUP_Msk);

	return HAL_OK;
}

/**
  * @brief Copy a buffer from user memory area to packet memory area (PMA)
  * @param   USBx USB peripheral instance register address.
  * @param   pbUsrBuf pointer to user memory area.
  * @param   wPMABufAddr address into PMA.
  * @param   wNBytes no. of bytes to be copied.
  * @retval None
  */
void USB_WritePMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf, uint32_t wPMABufAddr, uint16_t wNBytes)
{
	uint16_t i;
	
	for (i = 0; i < wNBytes; i++)
	{
		*(__IO uint8_t *)wPMABufAddr = pbUsrBuf[i];
		wPMABufAddr++;
	}
}

/**
  * @brief Copy data from packet memory area (PMA) to user memory buffer
  * @param   USBx USB peripheral instance register address.
  * @param   pbUsrBuf pointer to user memory area.
  * @param   wPMABufAddr address into PMA.
  * @param   wNBytes no. of bytes to be copied.
  * @retval None
  */
void USB_ReadPMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf, uint32_t wPMABufAddr, uint16_t wNBytes)
{
	uint16_t i;
	
	for (i = 0; i < wNBytes; i++)
	{
		pbUsrBuf[i] = *(__IO uint8_t *)wPMABufAddr;
		wPMABufAddr++;
	}
}

#endif /* defined (HAL_PCD_MODULE_ENABLED) */
