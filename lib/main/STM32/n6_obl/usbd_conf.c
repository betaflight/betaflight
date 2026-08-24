/*
 * Override of CubeN6's usbd_conf.c.
 *
 * The submodule version uses __HAL_RCC_PWR_CLK_ENABLE / GPIOA_CLK_ENABLE
 * macros that live behind CPU_IN_SECURE_STATE in stm32n6xx_hal_rcc.h
 * (which pull in -mcmse-only HAL_MPU_*_NS variants). Our build doesn't
 * define CPU_IN_SECURE_STATE — see feedback_n6_peripheral_alias_cmse.md.
 * This file replaces those macro calls with direct RCC->AHB4ENSR writes
 * (matching the convention in main.c::tamp_clk_enable and the FSBL
 * stub's system_stm32n6xx_fsbl.c).
 *
 * Everything else is a verbatim copy of the upstream version — the
 * PCD↔USBD callback bridges, USBD_LL_* shims around HAL_PCD_*, USB
 * device init with embedded HS-PHY in FS speed.
 */

#include "main.h"
#include "usbd_dfu.h"

PCD_HandleTypeDef hpcd_USB_HS;

/* ---------------- IRQ ---------------- */

void USB1_OTG_HS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_HS);
}

/* ---------------- PCD MSP ---------------- */

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance != USB1_OTG_HS) {
        return;
    }

    /* PWR clock — direct register write avoids the CMSE-gated HAL macro. */
    SET_BIT(RCC->AHB4ENSR, RCC_AHB4ENSR_PWRENS);
    (void)RCC->AHB4ENR;

    /* VDD33USB independent USB voltage monitor. The PWREx_ helpers are
     * NS-safe (no CMSE-gated macros internally). */
    HAL_PWREx_EnableVddUSBVMEN();
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_USB33RDY) == 0U) {
        ;
    }
    HAL_PWREx_EnableVddUSB();

    /* USB1 OTG HS kernel clock from HSE_DIRECT (48 MHz on N6570-DK). */
    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection    = RCC_PERIPHCLK_USBOTGHS1;
    pclk.UsbOtgHs1ClockSelection = RCC_USBOTGHS1CLKSOURCE_HSE_DIRECT;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) {
        Error_Handler();
    }

    /* USB OTG HS PHY1 reference clock from HSE_DIRECT. */
    pclk.PeriphClockSelection  = RCC_PERIPHCLK_USBPHY1;
    pclk.UsbPhy1ClockSelection = RCC_USBPHY1CLKSOURCE_HSE_DIRECT;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) {
        Error_Handler();
    }

    /* GPIOA clock — direct register write (DP/DM are PA11/PA12). */
    SET_BIT(RCC->AHB4ENSR, RCC_AHB4ENSR_GPIOAENS);
    (void)RCC->AHB4ENR;

    LL_AHB5_GRP1_ForceReset(RCC_AHB5RSTR_OTG1PHYCTLRST);
    __HAL_RCC_USB1_OTG_HS_FORCE_RESET();
    __HAL_RCC_USB1_OTG_HS_PHY_FORCE_RESET();

    LL_RCC_HSE_SelectHSEDiv2AsDiv2Clock();
    LL_AHB5_GRP1_ReleaseReset(RCC_AHB5RSTR_OTG1PHYCTLRST);

    __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

    /* USBPHYC FSEL = 0b010 (24 MHz reference). HAL never writes this and
     * the default 0b001 silently drops every SETUP packet — see
     * feedback_n6_usbphyc_fsel.md. */
    USB1_HS_PHYC->USBPHYC_CR &= ~(0x7U << 0x4U);
    USB1_HS_PHYC->USBPHYC_CR |= (0x2U << 0x4U);

    __HAL_RCC_USB1_OTG_HS_PHY_RELEASE_RESET();
    HAL_Delay(1);
    __HAL_RCC_USB1_OTG_HS_RELEASE_RESET();
    __HAL_RCC_USB1_OTG_HS_PHY_CLK_ENABLE();

    HAL_NVIC_SetPriority(USB1_OTG_HS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USB1_OTG_HS_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance != USB1_OTG_HS) {
        return;
    }
    HAL_PWREx_DisableVddUSBVMEN();
    HAL_PWREx_DisableVddUSB();

    __HAL_RCC_USB1_OTG_HS_CLK_DISABLE();
    __HAL_RCC_USB1_OTG_HS_PHY_CLK_DISABLE();

    HAL_NVIC_DisableIRQ(USB1_OTG_HS_IRQn);
}

/* ---------------- PCD → USBD callbacks ---------------- */

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF(hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_SpeedTypeDef speed = USBD_SPEED_FULL;
    switch (hpcd->Init.speed) {
    case PCD_SPEED_HIGH: speed = USBD_SPEED_HIGH; break;
    case PCD_SPEED_FULL: speed = USBD_SPEED_FULL; break;
    default:             speed = USBD_SPEED_FULL; break;
    }
    USBD_LL_Reset(hpcd->pData);
    USBD_LL_SetSpeed(hpcd->pData, speed);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Suspend(hpcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevConnected(hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevDisconnected(hpcd->pData);
}

/* ---------------- USBD → PCD shims ---------------- */

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd_USB_HS.Instance                 = USB1_OTG_HS;
    hpcd_USB_HS.Init.dev_endpoints       = 3U;
    hpcd_USB_HS.Init.speed               = PCD_SPEED_HIGH;
    hpcd_USB_HS.Init.dma_enable          = DISABLE;
    hpcd_USB_HS.Init.phy_itface          = USB_OTG_HS_EMBEDDED_PHY;
    hpcd_USB_HS.Init.Sof_enable          = DISABLE;
    hpcd_USB_HS.Init.low_power_enable    = DISABLE;
    hpcd_USB_HS.Init.lpm_enable          = DISABLE;
    hpcd_USB_HS.Init.vbus_sensing_enable = DISABLE;
    hpcd_USB_HS.Init.use_dedicated_ep1   = DISABLE;
    hpcd_USB_HS.Init.use_external_vbus   = DISABLE;

    hpcd_USB_HS.pData = pdev;
    pdev->pData       = &hpcd_USB_HS;

    HAL_PCD_Init(&hpcd_USB_HS);
    HAL_PCDEx_SetRxFiFo(&hpcd_USB_HS, 0xA0U);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_HS, 0, 0xA0U);

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_DeInit(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Start(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Stop(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                  uint8_t ep_type, uint16_t ep_mps)
{
    HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_Close(pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_Flush(pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
    return USBD_OK;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    PCD_HandleTypeDef *hpcd = pdev->pData;
    if ((ep_addr & 0x80U) == 0x80U) {
        return hpcd->IN_ep[ep_addr & 0x7FU].is_stall;
    }
    return hpcd->OUT_ep[ep_addr & 0x7FU].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    HAL_PCD_SetAddress(pdev->pData, dev_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                    uint8_t *pbuf, uint32_t size)
{
    HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                          uint8_t *pbuf, uint32_t size)
{
    HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

void *USBD_static_malloc(uint32_t size)
{
    (void)size;
    static uint32_t mem[(sizeof(USBD_DFU_HandleTypeDef) / 4U) + 1U];
    return mem;
}

void USBD_static_free(void *p)
{
    (void)p;
}

void USBD_LL_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}
