
/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

#ifdef HAL_PCD_MODULE_ENABLED

/**
  * @brief  Configure PMA for EP
  * @param  hpcd  Device instance
  * @param  ep_addr endpoint address
  * @param  pmaadress: EP address in The PMA: In case of single buffer endpoint
  *                   this parameter is 32-bit value providing the address
  *                   in PMA allocated to endpoint.
  *                   In case of double buffer endpoint this parameter
  *                   is a 32-bit value providing the endpoint buffer 0 address
  *                   in the LSB part of 32-bit value and endpoint buffer 1 address
  *                   in the MSB part of 32-bit value.
  * @retval HAL status
  */

HAL_StatusTypeDef  HAL_PCDEx_PMAConfig(PCD_HandleTypeDef *hpcd, uint16_t ep_addr,
                                       uint32_t pmaadress)
{
	PCD_EPTypeDef *ep;

	/* initialize ep structure*/
	if ((0x80U & ep_addr) == 0x80U)
	{
		ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
	}
	else
	{
		ep = &hpcd->OUT_ep[ep_addr];
	}

	/* Configure the PMA */
	ep->pmaadress = pmaadress;

	return HAL_OK;
}

/**
  * @brief  Software Device Connection,
  *         this function is not required by USB OTG FS peripheral, it is used
  *         only by USB Device FS peripheral.
  * @param  hpcd PCD handle
  * @param  state connection state (0 : disconnected / 1: connected)
  * @retval None
  */
__weak void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state)
{
  UNUSED(hpcd);
  UNUSED(state);
}

/**
  * @brief  Send LPM message to user layer callback.
  * @param  hpcd PCD handle
  * @param  msg LPM message
  * @retval HAL status
  */
__weak void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(msg);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCDEx_LPM_Callback could be implemented in the user file
   */
}

/**
  * @brief  Send BatteryCharging message to user layer callback.
  * @param  hpcd PCD handle
  * @param  msg LPM message
  * @retval HAL status
  */
__weak void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(msg);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCDEx_BCD_Callback could be implemented in the user file
   */
}

#endif /* HAL_PCD_MODULE_ENABLED */
