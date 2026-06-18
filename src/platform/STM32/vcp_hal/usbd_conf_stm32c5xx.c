/**
  ******************************************************************************
  * @file    usbd_conf_stm32c5xx.c
  * @author  Betaflight
  * @brief   USB Device configuration and HAL2 PCD callbacks for STM32C5
  *          (USB_DRD_FS peripheral).
  *
  *          HAL2 (Cube 2.0) PCD API — uses hal_pcd_handle_t, separate Init +
  *          SetConfig, USE_HAL_PCD_USER_DATA for USBD handle linkage.
  ******************************************************************************
  */

#include "stm32c5xx.h"
#include "stm32c5xx_hal.h"
#include "stm32c5xx_hal_pcd.h"
#include "usbd_def.h"
#include "usbd_core.h"

#include "usbd_cdc.h"

#include "platform.h"

hal_pcd_handle_t hpcd_USB_DRD_FS;

void Error_Handler(void)
{
    while (1) {
    }
}

static USBD_StatusTypeDef USBD_Get_USB_Status(hal_status_t hal_status);

/* Get the USBD handle stored in the PCD handle's user data */
static inline USBD_HandleTypeDef *PCD_GetUSBD(hal_pcd_handle_t *hpcd)
{
    return (USBD_HandleTypeDef *)HAL_PCD_GetUserData(hpcd);
}

void USB_DRD_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
}

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/

void HAL_PCD_SetupStageCallback(hal_pcd_handle_t *hpcd)
{
    USBD_LL_SetupStage(PCD_GetUSBD(hpcd), (uint8_t *)hpcd->setup);
}

void HAL_PCD_DataOutStageCallback(hal_pcd_handle_t *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage(PCD_GetUSBD(hpcd), epnum, hpcd->out_ep[epnum].p_xfer_buffer);
}

void HAL_PCD_DataInStageCallback(hal_pcd_handle_t *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage(PCD_GetUSBD(hpcd), epnum, hpcd->in_ep[epnum].p_xfer_buffer);
}

void HAL_PCD_SOFCallback(hal_pcd_handle_t *hpcd)
{
    USBD_LL_SOF(PCD_GetUSBD(hpcd));
}

void HAL_PCD_ResetCallback(hal_pcd_handle_t *hpcd)
{
    USBD_LL_SetSpeed(PCD_GetUSBD(hpcd), USBD_SPEED_FULL);
    USBD_LL_Reset(PCD_GetUSBD(hpcd));
}

void HAL_PCD_SuspendCallback(hal_pcd_handle_t *hpcd)
{
    USBD_LL_Suspend(PCD_GetUSBD(hpcd));
}

void HAL_PCD_ResumeCallback(hal_pcd_handle_t *hpcd)
{
    USBD_LL_Resume(PCD_GetUSBD(hpcd));
}

void HAL_PCD_ISOOUTIncompleteCallback(hal_pcd_handle_t *hpcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete(PCD_GetUSBD(hpcd), epnum);
}

void HAL_PCD_ISOINIncompleteCallback(hal_pcd_handle_t *hpcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete(PCD_GetUSBD(hpcd), epnum);
}

void HAL_PCD_ConnectCallback(hal_pcd_handle_t *hpcd)
{
    USBD_LL_DevConnected(PCD_GetUSBD(hpcd));
}

void HAL_PCD_DisconnectCallback(hal_pcd_handle_t *hpcd)
{
    USBD_LL_DevDisconnected(PCD_GetUSBD(hpcd));
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    /* Select HSI/3 (= 48 MHz from the 144 MHz HSI) as the CK48 source
     * before enabling the USB peripheral clock. Reset value is 0 which
     * leaves CK48 unsourced; without this Windows enumerates the device
     * but cannot read its descriptors. RCC->CR1 already has HSIDIV3ON
     * enabled out of reset on STM32C5. */
    LL_RCC_SetCK48ClockSource(LL_RCC_CK48_CLKSOURCE_HSIDIV3);

    /* Enable USB and GPIO clocks */
    HAL_RCC_GPIOA_EnableClock();
    HAL_RCC_USB_EnableClock();

    /* Configure PA11 (DM) and PA12 (DP) as AF13 (USB) */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_12, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_11, LL_GPIO_AF_13);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_12, LL_GPIO_AF_13);

    /* Init PCD handle and register driver. Must run before SetUserData --
     * HAL_PCD_Init() unconditionally clears hpcd->p_user_data, so binding
     * pdev earlier just gets wiped. */
    if (HAL_PCD_Init(&hpcd_USB_DRD_FS, HAL_PCD_DRD_FS) != HAL_OK) {
        Error_Handler();
    }

    /* Cross-link handles now that the PCD struct is settled. */
    HAL_PCD_SetUserData(&hpcd_USB_DRD_FS, pdev);
    pdev->pData = &hpcd_USB_DRD_FS;

    /* Enable USB interrupt only after the PCD handle is fully populated;
     * earlier USB events would otherwise dereference a half-initialised
     * driver. */
    NVIC_SetPriority(USB_DRD_FS_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
    NVIC_EnableIRQ(USB_DRD_FS_IRQn);

    /* Configure the PCD peripheral */
    hal_pcd_config_t pcdConfig = {
        .dma_enable             = HAL_PCD_DMA_DISABLED,
        .pcd_speed              = HAL_PCD_SPEED_FS,
        .phy_interface          = HAL_PCD_PHY_EMBEDDED_FS,
        .sof_enable             = HAL_PCD_SOF_DISABLED,
        .lpm_enable             = HAL_PCD_LPM_DISABLED,
        .battery_charging_enable = HAL_PCD_BCD_DISABLED,
        .vbus_sensing_enable    = HAL_PCD_VBUS_SENSE_DISABLED,
        .bulk_doublebuffer_enable = HAL_PCD_BULK_DB_DISABLED,
    };

    if (HAL_PCD_SetConfig(&hpcd_USB_DRD_FS, &pcdConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Configure PMA (Packet Memory Area) for endpoints */
    HAL_PCD_PMAConfig(&hpcd_USB_DRD_FS, 0x00, HAL_PCD_SNG_BUF, 0x14);
    HAL_PCD_PMAConfig(&hpcd_USB_DRD_FS, 0x80, HAL_PCD_SNG_BUF, 0x54);
    HAL_PCD_PMAConfig(&hpcd_USB_DRD_FS, CDC_IN_EP, HAL_PCD_SNG_BUF, 0x94);
    HAL_PCD_PMAConfig(&hpcd_USB_DRD_FS, CDC_OUT_EP, HAL_PCD_SNG_BUF, 0xD4);
    HAL_PCD_PMAConfig(&hpcd_USB_DRD_FS, CDC_CMD_EP, HAL_PCD_SNG_BUF, 0x114);

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_DeInit(pdev->pData);
    return USBD_OK;
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
    return USBD_Get_USB_Status(HAL_PCD_OpenEndpoint(pdev->pData, ep_addr, ep_mps,
                                                    (hal_pcd_ep_type_t)ep_type));
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_CloseEndpoint(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_FlushEndpoint(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_SetEndpointStall(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_ClearEndpointStall(pdev->pData, ep_addr));
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    /*
     * HAL2 usb_core_ep_t has no is_stall field. Read stall state from the
     * USB endpoint register: STAT_TX bits [5:4] or STAT_RX bits [13:12],
     * where 0b01 = STALL.
     * CHEP0R..CHEP7R are contiguous 32-bit registers starting at CHEP0R.
     */
    hal_pcd_handle_t *hpcd = pdev->pData;
    USB_DRD_TypeDef *usb = (USB_DRD_TypeDef *)((uint32_t)hpcd->instance);
    volatile uint32_t *chep = &usb->CHEP0R;
    uint32_t reg = chep[ep_addr & 0x7FU];

    if ((ep_addr & 0x80U) == 0x80U) {
        return ((reg & USB_CHEP_TX_STTX) == USB_EP_TX_STALL) ? 1U : 0U;
    } else {
        return ((reg & USB_CHEP_RX_STRX) == USB_EP_RX_STALL) ? 1U : 0U;
    }
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_SetDeviceAddress(pdev->pData, dev_addr));
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    return USBD_Get_USB_Status(HAL_PCD_SetEndpointTransmit(pdev->pData, ep_addr, pbuf, size));
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    return USBD_Get_USB_Status(HAL_PCD_SetEndpointReceive(pdev->pData, ep_addr, pbuf, size));
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}

static USBD_StatusTypeDef USBD_Get_USB_Status(hal_status_t hal_status)
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
