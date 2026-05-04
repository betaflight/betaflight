/**
  ******************************************************************************
  * @file    usbd_conf_stm32h5xx.c
  * @author  Betaflight
  * @brief   USB Device configuration and HAL PCD callbacks for STM32H5
  *          (USB_DRD_FS peripheral).
  ******************************************************************************
  */

#include "stm32h5xx.h"
#include "stm32h5xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"

#include "usbd_cdc.h"

#include "platform.h"

PCD_HandleTypeDef hpcd_USB_DRD_FS;
void Error_Handler(void)
{
    while (1) {
    }
}

static USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);

void USB_DRD_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
}

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcdHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (pcdHandle->Instance == USB_DRD_FS) {
        /* H5 isolates VDDUSB at reset. Enable the detector and remove the
         * isolation before clocking the peripheral, otherwise the FS
         * transceiver stays unpowered and the host sees DPPU but fails
         * descriptor exchange ("Unknown USB Device, Device Descriptor Request
         * Failed"). PWR has no RCC enable bit on H5 (it is always-clocked),
         * so PWREx access here needs no __HAL_RCC_PWR_CLK_ENABLE().
         */
        HAL_PWREx_EnableUSBVoltageDetector();
        HAL_PWREx_EnableVddUSB();

        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* USB DM/DP on PA11/PA12 */
        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_USB;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        __HAL_RCC_USB_CLK_ENABLE();

        HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);
    }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcdHandle)
{
    if (pcdHandle->Instance == USB_DRD_FS) {
        __HAL_RCC_USB_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
        HAL_NVIC_DisableIRQ(USB_DRD_FS_IRQn);
    }
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
    USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
    if (hpcd->Init.low_power_enable) {
        SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd_USB_DRD_FS.pData = pdev;
    pdev->pData = &hpcd_USB_DRD_FS;

    hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
    hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
    hpcd_USB_DRD_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;

    if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK) {
        Error_Handler();
    }

    /* Configure PMA (Packet Memory Area) for endpoints */
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x00, PCD_SNG_BUF, 0x14);
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x80, PCD_SNG_BUF, 0x54);
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, CDC_IN_EP, PCD_SNG_BUF, 0x94);
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, CDC_OUT_EP, PCD_SNG_BUF, 0xD4);
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, CDC_CMD_EP, PCD_SNG_BUF, 0x114);

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    return USBD_Get_USB_Status(HAL_PCD_DeInit(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    return USBD_Get_USB_Status(HAL_PCD_Start(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    return USBD_Get_USB_Status(HAL_PCD_Stop(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type));
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_Close(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_Flush(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_SetStall(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_ClrStall(pdev->pData, ep_addr));
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
    if ((ep_addr & 0x80) == 0x80) {
        return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
    } else {
        return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
    }
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_SetAddress(pdev->pData, dev_addr));
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size));
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size));
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}

static USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
    switch (hal_status) {
    case HAL_OK:
        return USBD_OK;
    case HAL_BUSY:
        return USBD_BUSY;
    default:
        return USBD_FAIL;
    }
}
