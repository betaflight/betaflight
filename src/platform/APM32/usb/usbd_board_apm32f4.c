/**
 * @file        usbd_board.c
 *
 * @brief       This file provides firmware functions to USB board
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Includes ***************************************************************/
#include "usbd_board.h"

/* Private includes *******************************************************/
#include "usbd_core.h"
#include "platform.h"

/* Private macro **********************************************************/
#define USBD_FS_RX_FIFO_SIZE                128
#define USBD_FS_TX_FIFO_0_SIZE              64
#define USBD_FS_TX_FIFO_1_SIZE              128
#define USBD_FS_TX_FIFO_2_SIZE              0
#define USBD_FS_TX_FIFO_3_SIZE              64

#define USBD_HS_RX_FIFO_SIZE                512
#define USBD_HS_TX_FIFO_0_SIZE              128
#define USBD_HS_TX_FIFO_1_SIZE              372

/* Private typedef ********************************************************/

/* Private variables ******************************************************/
PCD_HandleTypeDef husbDevice;

/* Private function prototypes ********************************************/

/* External variables *****************************************************/

/* Private functions ******************************************************/

/**
 * @brief   This function handles USB Handler
 *
 * @param   None
 *
 * @retval  None
 *
 */
#ifdef USE_USB_FS
void OTG_FS_IRQHandler(void)
#else
void OTG_HS1_IRQHandler(void)
#endif /* USE_USB_FS */
{
    DAL_PCD_IRQHandler(&husbDevice);
}

#ifdef USE_USB_FS
void OTG_FS_WKUP_IRQHandler(void)
#else
void OTG_HS1_WKUP_IRQHandler(void)
#endif /* USE_USB_FS */
{
  if((&husbDevice)->Init.low_power_enable)
  {
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));

    /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
    PLL as system clock source (HSE and PLL are disabled in STOP mode) */

    __DAL_RCM_HSE_CONFIG(RCM_HSE_ON);

    /* Wait till HSE is ready */
    while(__DAL_RCM_GET_FLAG(RCM_FLAG_HSERDY) == RESET)
    {}

    /* Enable the main PLL. */
    __DAL_RCM_PLL_ENABLE();

    /* Wait till PLL is ready */
    while(__DAL_RCM_GET_FLAG(RCM_FLAG_PLLRDY) == RESET)
    {}

    /* Select PLL as SYSCLK */
    MODIFY_REG(RCM->CFG, RCM_CFG_SCLKSEL, RCM_SYSCLKSOURCE_PLLCLK);

    while (__DAL_RCM_GET_SYSCLK_SOURCE() != RCM_CFG_SCLKSEL_PLL)
    {}

    /* ungate PHY clock */
    __DAL_PCD_UNGATE_PHYCLOCK((&husbDevice));
  }
#ifdef USE_USB_FS
  /* Clear EINT pending Bit*/
  __DAL_USB_OTG_FS_WAKEUP_EINT_CLEAR_FLAG();
#else
  /* Clear EINT pending Bit*/
  __DAL_USB_OTG_HS_WAKEUP_EINT_CLEAR_FLAG();
#endif
}

/**
 * @brief  Initializes the PCD MSP
 *
 * @param  hpcd PCD handle
 *
 * @retval None
 */
void DAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hpcd->Instance == USB_OTG_FS)
    {
        /* Configure USB OTG GPIO */
        __DAL_RCM_GPIOA_CLK_ENABLE();

        /* USB DM, DP pin configuration */
        GPIO_InitStruct.Pin         = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull        = GPIO_NOPULL;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate   = GPIO_AF10_OTG_FS;
        DAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Configure USB OTG */
        __DAL_RCM_USB_OTG_FS_CLK_ENABLE();

        /* Configure interrupt */
        DAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);
        DAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
    else if(hpcd->Instance == USB_OTG_HS)
    {
        /* Configure USB OTG GPIO */
        __DAL_RCM_GPIOB_CLK_ENABLE();

        /* USB DM, DP pin configuration */
        GPIO_InitStruct.Pin         = GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull        = GPIO_NOPULL;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate   = GPIO_AF12_OTG_HS_FS;
        DAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Configure USB OTG */
        __DAL_RCM_USB_OTG_HS_CLK_ENABLE();

        /* Configure interrupt */
        DAL_NVIC_SetPriority(OTG_HS_IRQn, 6, 0);
        DAL_NVIC_EnableIRQ(OTG_HS_IRQn);
    }
}

/**
 * @brief  DeInitializes PCD MSP
 *
 * @param  hpcd PCD handle
 *
 * @retval None
 */
void DAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    if(hpcd->Instance == USB_OTG_FS)
    {
        /* Disable peripheral clock */
        __DAL_RCM_USB_OTG_FS_CLK_DISABLE();

        /* USB DM, DP pin configuration */
        DAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

        /* Disable peripheral interrupt */
        DAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    }
    else if(hpcd->Instance == USB_OTG_HS)
    {
        /* Disable peripheral clock */
        __DAL_RCM_USB_OTG_HS_CLK_DISABLE();

        /* USB DM, DP pin configuration */
        DAL_GPIO_DeInit(GPIOB, GPIO_PIN_14 | GPIO_PIN_15);

        /* Disable peripheral interrupt */
        DAL_NVIC_DisableIRQ(OTG_HS_IRQn);
    }
}

/**
 * @brief  Setup stage callback
 *
 * @param  hpcd: PCD handle
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
#else
void DAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    USBD_SetupStage((USBD_INFO_T*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
 * @brief  Data Out stage callback
 *
 * @param  hpcd: PCD handle
 *
 * @param  epnum: Endpoint number
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void DAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    USBD_DataOutStage((USBD_INFO_T*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

/**
 * @brief  Data In stage callback
 *
 * @param  hpcd: PCD handle
 *
 * @param  epnum: Endpoint number
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void DAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    USBD_DataInStage((USBD_INFO_T*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/**
 * @brief  SOF callback
 *
 * @param  hpcd: PCD handle
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
#else
void DAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    USBD_HandleSOF((USBD_INFO_T*)hpcd->pData);
}

/**
 * @brief  Reset callback
 *
 * @param  hpcd: PCD handle
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
#else
void DAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    USBD_DEVICE_SPEED_T speed = USBD_DEVICE_SPEED_FS;

    if (hpcd->Init.speed == PCD_SPEED_HIGH)
    {
        speed = USBD_DEVICE_SPEED_HS;
    }
    else if (hpcd->Init.speed == PCD_SPEED_FULL)
    {
        speed = USBD_DEVICE_SPEED_FS;
    }
    else
    {
        DAL_ErrorHandler();
    }

    /* Set USB core speed */
    USBD_SetSpeed((USBD_INFO_T*)hpcd->pData, speed);

    /* Reset Device. */
    USBD_Reset((USBD_INFO_T*)hpcd->pData);
}

/**
 * @brief  Suspend callback
 *
 * @param  hpcd: PCD handle
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
#else
void DAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    /* USB core enters in suspend mode */
    USBD_Suspend((USBD_INFO_T*)hpcd->pData);

    if ((hpcd->Init.phy_itface == USB_OTG_EMBEDDED_PHY) && \
        (hpcd->Init.speed == PCD_SPEED_HIGH))
    {
        /* Embedded HS PHY can not stop clock */
    }
    else
    {
        __DAL_PCD_GATE_PHYCLOCK(hpcd);
    }

    /* Enter in STOP mode. */
    if (hpcd->Init.low_power_enable)
    {
        /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register. */
        SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }
}

/**
 * @brief  Resume callback
 *
 * @param  hpcd: PCD handle
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
#else
void DAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    USBD_Resume((USBD_INFO_T*)hpcd->pData);
}

/**
 * @brief  ISOOUTIncomplete callback
 *
 * @param  hpcd: PCD handle
 *
 * @param  epnum: Endpoint number
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void DAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    USBD_IsoOutInComplete((USBD_INFO_T*)hpcd->pData, epnum);
}

/**
 * @brief  ISOINIncomplete callback
 *
 * @param  hpcd: PCD handle
 *
 * @param  epnum: Endpoint number
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void DAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    USBD_IsoInInComplete((USBD_INFO_T*)hpcd->pData, epnum);
}

/**
 * @brief  Connect callback
 *
 * @param  hpcd: PCD handle
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
#else
void DAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    USBD_Connect((USBD_INFO_T*)hpcd->pData);
}

/**
 * @brief  Disconnect callback
 *
 * @param  hpcd: PCD handle
 *
 * @retval None
 */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
#else
void DAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
{
    USBD_Disconnect((USBD_INFO_T*)hpcd->pData);
}

/* External functions *****************************************************/

/**
 * @brief   Init USB hardware
 *
 * @param   usbInfo: USB core information
 *
 * @retval  None
 */
void USBD_HardwareInit(USBD_INFO_T* usbInfo)
{
    if (usbInfo->devSpeed == USBD_SPEED_FS)
    {
        /* Link data */
        husbDevice.pData                    = usbInfo;
        usbInfo->dataPoint                  = &husbDevice;

        husbDevice.Instance                 = USB_OTG_FS;
        husbDevice.Init.dev_endpoints       = 4;
        husbDevice.Init.speed               = PCD_SPEED_FULL;
        husbDevice.Init.dma_enable          = DISABLE;
        husbDevice.Init.phy_itface          = PCD_PHY_EMBEDDED;
        husbDevice.Init.Sof_enable          = ENABLE;
        husbDevice.Init.low_power_enable    = DISABLE;
        husbDevice.Init.lpm_enable          = DISABLE;
        husbDevice.Init.vbus_sensing_enable = DISABLE;
        husbDevice.Init.use_dedicated_ep1   = DISABLE;
        if (DAL_PCD_Init(&husbDevice) != DAL_OK)
        {
            DAL_ErrorHandler();
        }

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
        /* Register USB PCD CallBacks */
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_SOF_CB_ID, PCD_SOFCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_SETUPSTAGE_CB_ID, PCD_SetupStageCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_RESET_CB_ID, PCD_ResetCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_SUSPEND_CB_ID, PCD_SuspendCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_RESUME_CB_ID, PCD_ResumeCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_CONNECT_CB_ID, PCD_ConnectCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_DISCONNECT_CB_ID, PCD_DisconnectCallback);

        DAL_PCD_RegisterDataOutStageCallback(&husbDevice, PCD_DataOutStageCallback);
        DAL_PCD_RegisterDataInStageCallback(&husbDevice, PCD_DataInStageCallback);
        DAL_PCD_RegisterIsoOutIncpltCallback(&husbDevice, PCD_ISOOUTIncompleteCallback);
        DAL_PCD_RegisterIsoInIncpltCallback(&husbDevice, PCD_ISOINIncompleteCallback);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */

        DAL_PCDEx_SetRxFiFo(&husbDevice, USBD_FS_RX_FIFO_SIZE);
        DAL_PCDEx_SetTxFiFo(&husbDevice, 0, USBD_FS_TX_FIFO_0_SIZE);
        DAL_PCDEx_SetTxFiFo(&husbDevice, 1, USBD_FS_TX_FIFO_1_SIZE);
        DAL_PCDEx_SetTxFiFo(&husbDevice, 2, USBD_FS_TX_FIFO_2_SIZE);
        DAL_PCDEx_SetTxFiFo(&husbDevice, 3, USBD_FS_TX_FIFO_3_SIZE);

        /* Start USB device core */
        USBD_StartCallback(usbInfo);
    }
    else if (usbInfo->devSpeed == USBD_SPEED_HS)
    {
        husbDevice.pData                    = usbInfo;
        usbInfo->dataPoint                  = &husbDevice;

        husbDevice.Instance                 = USB_OTG_HS;
        husbDevice.Init.dev_endpoints       = 6;
        husbDevice.Init.speed               = PCD_SPEED_HIGH;
        husbDevice.Init.dma_enable          = DISABLE;
        husbDevice.Init.phy_itface          = USB_OTG_EMBEDDED_PHY;
        husbDevice.Init.Sof_enable          = DISABLE;
        husbDevice.Init.low_power_enable    = DISABLE;
        husbDevice.Init.lpm_enable          = DISABLE;
        husbDevice.Init.vbus_sensing_enable = DISABLE;
        husbDevice.Init.use_dedicated_ep1   = DISABLE;
        husbDevice.Init.use_external_vbus   = DISABLE;
        if (DAL_PCD_Init(&husbDevice) != DAL_OK)
        {
            DAL_ErrorHandler( );
        }

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
        /* Register USB PCD CallBacks */
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_SOF_CB_ID, PCD_SOFCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_SETUPSTAGE_CB_ID, PCD_SetupStageCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_RESET_CB_ID, PCD_ResetCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_SUSPEND_CB_ID, PCD_SuspendCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_RESUME_CB_ID, PCD_ResumeCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_CONNECT_CB_ID, PCD_ConnectCallback);
        DAL_PCD_RegisterCallback(&husbDevice, DAL_PCD_DISCONNECT_CB_ID, PCD_DisconnectCallback);

        DAL_PCD_RegisterDataOutStageCallback(&husbDevice, PCD_DataOutStageCallback);
        DAL_PCD_RegisterDataInStageCallback(&husbDevice, PCD_DataInStageCallback);
        DAL_PCD_RegisterIsoOutIncpltCallback(&husbDevice, PCD_ISOOUTIncompleteCallback);
        DAL_PCD_RegisterIsoInIncpltCallback(&husbDevice, PCD_ISOINIncompleteCallback);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
        DAL_PCDEx_SetRxFiFo(&husbDevice, USBD_HS_RX_FIFO_SIZE);
        DAL_PCDEx_SetTxFiFo(&husbDevice, 0, USBD_HS_TX_FIFO_0_SIZE);
        DAL_PCDEx_SetTxFiFo(&husbDevice, 1, USBD_HS_TX_FIFO_1_SIZE);

        /* Start USB device core */
        USBD_StartCallback(usbInfo);
    }
}

/**
 * @brief   Reset USB hardware
 *
 * @param   usbInfo:usb handler information
 *
 * @retval  None
 */
void USBD_HardwareReset(USBD_INFO_T* usbInfo)
{
    DAL_PCD_DeInit(usbInfo->dataPoint);
}

/**
 * @brief   USB device start event callback
 *
 * @param   usbInfo: USB core information
 *
 * @retval  None
 */
void USBD_StartCallback(USBD_INFO_T* usbInfo)
{
    DAL_PCD_Start(usbInfo->dataPoint);
}

/**
 * @brief   USB device stop event callback
 *
 * @param   usbInfo : USB core information
 *
 * @retval  None
 */
void USBD_StopCallback(USBD_INFO_T* usbInfo)
{
    DAL_PCD_Stop(usbInfo->dataPoint);
}

/**
 * @brief   USB device open EP callback
 *
 * @param   usbInfo : USB core information
 *
 * @param   epAddr: endpoint address
 *
 * @param   epType: endpoint type
 *
 * @param   epMps: endpoint maxinum of packet size
 *
 * @retval  None
 */
void USBD_EP_OpenCallback(USBD_INFO_T* usbInfo, uint8_t epAddr, \
                          uint8_t epType, uint16_t epMps)
{
    DAL_PCD_EP_Open(usbInfo->dataPoint, epAddr, epMps, epType);
}

/**
 * @brief   USB device close EP callback
 *
 * @param   usbInfo : USB core information
 *
 * @param   epAddr: endpoint address
 *
 * @retval  None
 */
void USBD_EP_CloseCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    DAL_PCD_EP_Close(usbInfo->dataPoint, epAddr);
}

/**
 * @brief   USB device flush EP handler callback
 *
 * @param   usbInfo : USB core information
 *
 * @param   epAddr : endpoint address
 *
 * @retval  usb device status
 */
USBD_STA_T USBD_EP_FlushCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    USBD_STA_T usbStatus = USBD_OK;

    DAL_PCD_EP_Flush(usbInfo->dataPoint, epAddr);

    return usbStatus;
}

/**
 * @brief   USB device set EP on stall status callback
 *
 * @param   usbInfo : USB core information
 *
 * @param   epAddr: endpoint address
 *
 * @retval  usb device status
 */
USBD_STA_T USBD_EP_StallCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    USBD_STA_T usbStatus = USBD_OK;

    DAL_PCD_EP_SetStall(usbInfo->dataPoint, epAddr);

    return usbStatus;
}

/**
 * @brief   USB device clear EP stall status callback
 *
 * @param   usbInfo : USB core information
 *
 * @param   epAddr: endpoint address
 *
 * @retval  usb device status
 */
USBD_STA_T USBD_EP_ClearStallCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    USBD_STA_T usbStatus = USBD_OK;

    DAL_PCD_EP_ClrStall(usbInfo->dataPoint, epAddr);

    return usbStatus;
}

/**
 * @brief   USB device read EP stall status callback
 *
 * @param   usbInfo : USB core information
 *
 * @param   epAddr: endpoint address
 *
 * @retval  Stall status
 */
uint8_t USBD_EP_ReadStallStatusCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    return (DAL_PCD_EP_ReadStallStatus(usbInfo->dataPoint, epAddr));
}

/**
 * @brief   USB device set device address handler callback
 *
 * @param   usbInfo : USB core information
 *
 * @param   address : address
 *
 * @retval  usb device status
 */
USBD_STA_T USBD_SetDevAddressCallback(USBD_INFO_T* usbInfo, uint8_t address)
{
    USBD_STA_T usbStatus = USBD_OK;

    DAL_PCD_SetAddress(usbInfo->dataPoint, address);

    return usbStatus;
}

/**
 * @brief   USB device read EP last receive data size callback
 *
 * @param   usbInfo : USB core information
 *
 * @param   epAddr: endpoint address
 *
 * @retval  size of last receive data
 */
uint32_t USBD_EP_ReadRxDataLenCallback(USBD_INFO_T* usbInfo, uint8_t epAddr)
{
    return DAL_PCD_EP_GetRxCount(usbInfo->dataPoint, epAddr);
}

/**
 * @brief     USB device EP transfer handler callback
 *
 * @param     usbInfo : USB core information
 *
 * @param     epAddr : endpoint address
 *
 * @param     buffer : data buffer
 *
 * @param     length : length of data
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_EP_TransferCallback(USBD_INFO_T* usbInfo, uint8_t epAddr, \
                                    uint8_t* buffer, uint32_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    DAL_PCD_EP_Transmit(usbInfo->dataPoint, epAddr, buffer, length);

    return usbStatus;
}

/**
 * @brief     USB device EP receive handler callback
 *
 * @param     usbInfo : USB core information
 *
 * @param     epAddr : endpoint address
 *
 * @param     buffer : data buffer
 *
 * @param     length : length of data
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_EP_ReceiveCallback(USBD_INFO_T* usbInfo, uint8_t epAddr, \
                                   uint8_t* buffer, uint32_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    DAL_PCD_EP_Receive(usbInfo->dataPoint, epAddr, buffer, length);

    return usbStatus;
}
