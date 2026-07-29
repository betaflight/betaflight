/**
  ******************************************************************************
  * @file     um324xF_hal_emac.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-03-21
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */
#ifdef HAL_EMAC_MODULE_ENABLED

#if defined(EMAC)

/** @defgroup ETH ETH
  * @brief ETH HAL module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup ETH_Private_Constants ETH Private Constants
  * @{
  */
#define ETH_MACCR_MASK       ((uint32_t)0xFFFB7F7CU)
#define ETH_MACECR_MASK      ((uint32_t)0x3F077FFFU)
#define ETH_MACFFR_MASK      ((uint32_t)0x800007FFU)
#define ETH_MACWTR_MASK      ((uint32_t)0x0000010FU)
#define ETH_MACTFCR_MASK     ((uint32_t)0xFFFF00F2U)
#define ETH_MACRFCR_MASK     ((uint32_t)0x00000003U)
#define ETH_MTLTQOMR_MASK    ((uint32_t)0x00000072U)
#define ETH_MTLRQOMR_MASK    ((uint32_t)0x0000007BU)

#define ETH_DMAMR_MASK       ((uint32_t)0x00007802U)
#define ETH_DMASBMR_MASK     ((uint32_t)0x0000D001U)
#define ETH_DMACCR_MASK      ((uint32_t)0x00013FFFU)
#define ETH_DMACTCR_MASK     ((uint32_t)0x003F1010U)
#define ETH_DMACRCR_MASK     ((uint32_t)0x803F0000U)
#define ETH_MACPMTCSR_MASK   (ETH_MACPMTCSR_PD | ETH_MACPMTCSR_WFE | \
                              ETH_MACPMTCSR_MPE | ETH_MACPMTCSR_GU)

/* Timeout values */
#define ETH_SWRESET_TIMEOUT                 ((uint32_t)5000U)//((uint32_t)500U)
#define ETH_MDIO_BUS_TIMEOUT                ((uint32_t)1000U)

#define ETH_DMARXDESC_ERRORS_MASK ((uint32_t)(ETH_DMARXDESC_DBE | ETH_DMARXDESC_RE | \
                                              ETH_DMARXDESC_OE  | ETH_DMARXDESC_RWT |\
                                              ETH_DMARXDESC_LC | ETH_DMARXDESC_CE |\
                                              ETH_DMARXDESC_DE | ETH_DMARXDESC_IPV4HCE))

#define ETH_MAC_US_TICK               ((uint32_t)1000000U)

#define ETH_MACTSCR_MASK              ((uint32_t)0x0087FF2FU)

#define ETH_PTPTSHR_VALUE            ((uint32_t)0xFFFFFFFFU)
#define ETH_PTPTSLR_VALUE            ((uint32_t)0xBB9ACA00U)

/* Ethernet GMIIADDRESS register Mask */
#define ETH_MACMIIAR_CR_MASK    ((uint32_t)0xFFFFFFE3U)


/* ETHERNET FLOWCONTROL register Mask */
#define ETH_MACFCR_CLEAR_MASK   0x0000FF41U


/* ETHERNET MAC address offsets */
#define ETH_MAC_ADDR_HBASE    (uint32_t)(EMAC_BASE + 0x40U)  /* ETHERNET MAC address high offset */
#define ETH_MAC_ADDR_LBASE    (uint32_t)(EMAC_BASE + 0x44U)  /* ETHERNET MAC address low offset */

/* ETHERNET DMA Rx descriptors Frame length Shift */
#define  ETH_DMARXDESC_FRAMELENGTHSHIFT            16U
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup ETH_Private_Macros ETH Private Macros
  * @{
  */
/* Helper macros for TX descriptor handling */
#define INCR_TX_DESC_INDEX(inx, offset) do {\
                                             (inx) += (offset);\
                                             if ((inx) >= (uint32_t)ETH_TX_DESC_CNT){\
                                             (inx) = ((inx) - (uint32_t)ETH_TX_DESC_CNT);}\
                                           } while (0)

/* Helper macros for RX descriptor handling */
#define INCR_RX_DESC_INDEX(inx, offset) do {\
                                             (inx) += (offset);\
                                             if ((inx) >= (uint32_t)ETH_RX_DESC_CNT){\
                                             (inx) = ((inx) - (uint32_t)ETH_RX_DESC_CNT);}\
                                           } while (0)
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/** @defgroup ETH_Private_Functions   ETH Private Functions
  * @{
  */
static void ETH_SetMACConfig(ETH_HandleTypeDef *heth,  ETH_MACConfigTypeDef *macconf);
static void ETH_SetDMAConfig(ETH_HandleTypeDef *heth,  ETH_DMAConfigTypeDef *dmaconf);
static void ETH_MACDMAConfig(ETH_HandleTypeDef *heth);
static void ETH_DMATxDescListInit(ETH_HandleTypeDef *heth);
static void ETH_DMARxDescListInit(ETH_HandleTypeDef *heth);
static uint32_t ETH_Prepare_Tx_Descriptors(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig, uint32_t ItMode);
static void ETH_UpdateDescriptor(ETH_HandleTypeDef *heth);
static void ETH_FlushTransmitFIFO(ETH_HandleTypeDef *heth);
static void ETH_MACAddressConfig(ETH_HandleTypeDef *heth, uint32_t MacAddr, uint8_t *Addr);

#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
static void ETH_InitCallbacksToDefault(ETH_HandleTypeDef *heth);
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup ETH_Exported_Functions ETH Exported Functions
  * @{
  */

/** @defgroup ETH_Exported_Functions_Group1 Initialization and deinitialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
===============================================================================
            ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the ETH peripheral:

      (+) User must Implement HAL_ETH_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO and NVIC ).

      (+) Call the function HAL_ETH_Init() to configure the selected device with
          the selected configuration:
        (++) MAC address
        (++) Media interface (MII or RMII)
        (++) Rx DMA Descriptors Tab
        (++) Tx DMA Descriptors Tab
        (++) Length of Rx Buffers

      (+) Call the function HAL_ETH_DeInit() to restore the default configuration
          of the selected ETH peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the Ethernet peripheral registers.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_Init(ETH_HandleTypeDef *heth)
{
  uint32_t tickstart;

  if (heth == NULL)
  {
    return HAL_ERROR;
  }
  if (heth->gState == HAL_ETH_STATE_RESET)
  {
    heth->gState = HAL_ETH_STATE_BUSY;

#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)

    ETH_InitCallbacksToDefault(heth);

    if (heth->MspInitCallback == NULL)
    {
      heth->MspInitCallback = HAL_ETH_MspInit;
    }

    /* Init the low level hardware */
    heth->MspInitCallback(heth);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC. */
    HAL_ETH_MspInit(heth);

#endif /* (USE_HAL_ETH_REGISTER_CALLBACKS) */
  }

  /* Select MII or RMII Mode*/
  SYSCFG->EMACMR &= ~(SYSCFG_EMACMR_EMACMR);
  SYSCFG->EMACMR |= (uint32_t)heth->Init.MediaInterface;
  /* Dummy read to sync SYSCFG with ETH */
  (void)SYSCFG->EMACMR;

  /* Ethernet Software reset */
  /* Set the SWR bit: resets all MAC subsystem internal registers and logic */
  /* After reset all the registers holds their respective reset values */
  SET_BIT(heth->Instance->DMA_BUSMODE, EMAC_DMA_BUSMODE_RESET);

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Wait for software reset */
  while (READ_BIT(heth->Instance->DMA_BUSMODE, EMAC_DMA_BUSMODE_RESET) > 0U)
  {
    if (((HAL_GetTick() - tickstart) > ETH_SWRESET_TIMEOUT))
    {
      /* Set Error Code */
      heth->ErrorCode = HAL_ETH_ERROR_TIMEOUT;
      /* Set State as Error */
      heth->gState = HAL_ETH_STATE_ERROR;
      /* Return Error */
      return HAL_ERROR;
    }
  }

  /*------------------ MAC, MTL and DMA default Configuration ----------------*/
  ETH_MACDMAConfig(heth);


  /*------------------ DMA Tx Descriptors Configuration ----------------------*/
  ETH_DMATxDescListInit(heth);

  /*------------------ DMA Rx Descriptors Configuration ----------------------*/
  ETH_DMARxDescListInit(heth);

  /*--------------------- ETHERNET MAC Address Configuration ------------------*/
  ETH_MACAddressConfig(heth, ETH_MAC_ADDRESS0, heth->Init.MACAddr);

  heth->ErrorCode = HAL_ETH_ERROR_NONE;
  heth->gState = HAL_ETH_STATE_READY;

  return HAL_OK;
}

/**
  * @brief  DeInitializes the ETH peripheral.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_DeInit(ETH_HandleTypeDef *heth)
{
  /* Set the ETH peripheral state to BUSY */
  heth->gState = HAL_ETH_STATE_BUSY;

#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)

  if (heth->MspDeInitCallback == NULL)
  {
    heth->MspDeInitCallback = HAL_ETH_MspDeInit;
  }
  /* DeInit the low level hardware */
  heth->MspDeInitCallback(heth);
#else

  /* De-Init the low level hardware : GPIO, CLOCK, NVIC. */
  HAL_ETH_MspDeInit(heth);

#endif /* (USE_HAL_ETH_REGISTER_CALLBACKS) */

  /* Set ETH HAL state to Disabled */
  heth->gState = HAL_ETH_STATE_RESET;

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the ETH MSP.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_MspInit could be implemented in the user file
  */
}

/**
  * @brief  DeInitializes ETH MSP.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_MspDeInit(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_MspDeInit could be implemented in the user file
  */
}

#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User ETH Callback
  *         To be used instead of the weak predefined callback
  * @param heth eth handle
  * @param CallbackID ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_ETH_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *          @arg @ref HAL_ETH_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *          @arg @ref HAL_ETH_ERROR_CB_ID       Error Callback ID
  *          @arg @ref HAL_ETH_PMT_CB_ID         Power Management Callback ID
  *          @arg @ref HAL_ETH_WAKEUP_CB_ID      Wake UP Callback ID
  *          @arg @ref HAL_ETH_MSPINIT_CB_ID     MspInit callback ID
  *          @arg @ref HAL_ETH_MSPDEINIT_CB_ID   MspDeInit callback ID
  * @param pCallback pointer to the Callback function
  * @retval status
  */
HAL_StatusTypeDef HAL_ETH_RegisterCallback(ETH_HandleTypeDef *heth, HAL_ETH_CallbackIDTypeDef CallbackID,
                                           pETH_CallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    heth->ErrorCode |= HAL_ETH_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  if (heth->gState == HAL_ETH_STATE_READY)
  {
    switch (CallbackID)
    {
      case HAL_ETH_TX_COMPLETE_CB_ID :
        heth->TxCpltCallback = pCallback;
        break;

      case HAL_ETH_RX_COMPLETE_CB_ID :
        heth->RxCpltCallback = pCallback;
        break;

      case HAL_ETH_ERROR_CB_ID :
        heth->ErrorCallback = pCallback;
        break;

      case HAL_ETH_PMT_CB_ID :
        heth->PMTCallback = pCallback;
        break;


      case HAL_ETH_WAKEUP_CB_ID :
        heth->WakeUpCallback = pCallback;
        break;

      case HAL_ETH_MSPINIT_CB_ID :
        heth->MspInitCallback = pCallback;
        break;

      case HAL_ETH_MSPDEINIT_CB_ID :
        heth->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        heth->ErrorCode |= HAL_ETH_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (heth->gState == HAL_ETH_STATE_RESET)
  {
    switch (CallbackID)
    {
      case HAL_ETH_MSPINIT_CB_ID :
        heth->MspInitCallback = pCallback;
        break;

      case HAL_ETH_MSPDEINIT_CB_ID :
        heth->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        heth->ErrorCode |= HAL_ETH_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    heth->ErrorCode |= HAL_ETH_ERROR_INVALID_CALLBACK;
    /* Return error status */
    status =  HAL_ERROR;
  }

  return status;
}

/**
  * @brief  Unregister an ETH Callback
  *         ETH callabck is redirected to the weak predefined callback
  * @param heth eth handle
  * @param CallbackID ID of the callback to be unregistered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_ETH_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *          @arg @ref HAL_ETH_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *          @arg @ref HAL_ETH_ERROR_CB_ID       Error Callback ID
  *          @arg @ref HAL_ETH_PMT_CB_ID         Power Management Callback ID
  *          @arg @ref HAL_ETH_WAKEUP_CB_ID      Wake UP Callback ID
  *          @arg @ref HAL_ETH_MSPINIT_CB_ID     MspInit callback ID
  *          @arg @ref HAL_ETH_MSPDEINIT_CB_ID   MspDeInit callback ID
  * @retval status
  */
HAL_StatusTypeDef HAL_ETH_UnRegisterCallback(ETH_HandleTypeDef *heth, HAL_ETH_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (heth->gState == HAL_ETH_STATE_READY)
  {
    switch (CallbackID)
    {
      case HAL_ETH_TX_COMPLETE_CB_ID :
        heth->TxCpltCallback = HAL_ETH_TxCpltCallback;
        break;

      case HAL_ETH_RX_COMPLETE_CB_ID :
        heth->RxCpltCallback = HAL_ETH_RxCpltCallback;
        break;

      case HAL_ETH_ERROR_CB_ID :
        heth->ErrorCallback = HAL_ETH_ErrorCallback;
        break;

      case HAL_ETH_PMT_CB_ID :
        heth->PMTCallback = HAL_ETH_PMTCallback;
        break;


      case HAL_ETH_WAKEUP_CB_ID :
        heth->WakeUpCallback = HAL_ETH_WakeUpCallback;
        break;

      case HAL_ETH_MSPINIT_CB_ID :
        heth->MspInitCallback = HAL_ETH_MspInit;
        break;

      case HAL_ETH_MSPDEINIT_CB_ID :
        heth->MspDeInitCallback = HAL_ETH_MspDeInit;
        break;

      default :
        /* Update the error code */
        heth->ErrorCode |= HAL_ETH_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (heth->gState == HAL_ETH_STATE_RESET)
  {
    switch (CallbackID)
    {
      case HAL_ETH_MSPINIT_CB_ID :
        heth->MspInitCallback = HAL_ETH_MspInit;
        break;

      case HAL_ETH_MSPDEINIT_CB_ID :
        heth->MspDeInitCallback = HAL_ETH_MspDeInit;
        break;

      default :
        /* Update the error code */
        heth->ErrorCode |= HAL_ETH_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    heth->ErrorCode |= HAL_ETH_ERROR_INVALID_CALLBACK;
    /* Return error status */
    status =  HAL_ERROR;
  }

  return status;
}
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group2 IO operation functions
  *  @brief ETH Transmit and Receive functions
  *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to manage the ETH
    data transfer.

@endverbatim
  * @{
  */

/**
  * @brief  Enables Ethernet MAC and DMA reception and transmission
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_Start(ETH_HandleTypeDef *heth)
{
  if (heth->gState == HAL_ETH_STATE_READY)
  {
    heth->gState = HAL_ETH_STATE_BUSY;

    /* Set nombre of descriptors to build */
    heth->RxDescList.RxBuildDescCnt = ETH_RX_DESC_CNT;

    /* Build all descriptors */
    ETH_UpdateDescriptor(heth);

    /* Enable the MAC transmission */
    SET_BIT(heth->Instance->CONFIG, EMAC_CONFIG_TE);

    /* Enable the MAC reception */
    SET_BIT(heth->Instance->CONFIG, EMAC_CONFIG_RE);

    /* Flush Transmit FIFO */
    ETH_FlushTransmitFIFO(heth);

    /* Enable the DMA transmission */
    SET_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_ST);

    /* Enable the DMA reception */
    SET_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_SR);

    heth->gState = HAL_ETH_STATE_STARTED;

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Enables Ethernet MAC and DMA reception/transmission in Interrupt mode
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_Start_IT(ETH_HandleTypeDef *heth)
{
  if (heth->gState == HAL_ETH_STATE_READY)
  {
    heth->gState = HAL_ETH_STATE_BUSY;

    /* save IT mode to ETH Handle */
    heth->RxDescList.ItMode = 1U;
    /* Disable MMC Interrupts */

    /* Disable Rx MMC Interrupts */
    SET_BIT(heth->Instance->MMC_RIM, 0x3FFFFFF);

    /* Disable Tx MMC Interrupts */
    SET_BIT(heth->Instance->MMC_TIM, 0x3FFFFFF);

    /* Set nombre of descriptors to build */
    heth->RxDescList.RxBuildDescCnt = ETH_RX_DESC_CNT;

    /* Build all descriptors */
    ETH_UpdateDescriptor(heth);

    /* Enable the MAC transmission */
    SET_BIT(heth->Instance->CONFIG, EMAC_CONFIG_TE);

    /* Enable the MAC reception */
    SET_BIT(heth->Instance->CONFIG, EMAC_CONFIG_RE);

    /* Flush Transmit FIFO */
    ETH_FlushTransmitFIFO(heth);

    /* Enable the DMA transmission */
    SET_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_ST);

    /* Enable the DMA reception */
    SET_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_SR);

    /* Enable ETH DMA interrupts:
    - Tx complete interrupt
    - Rx complete interrupt
    - Fatal bus interrupt
    */
    __HAL_ETH_DMA_ENABLE_IT(heth, (EMAC_DMA_INTEN_NIE | EMAC_DMA_INTEN_RIE | EMAC_DMA_INTEN_TCE  |
                                   EMAC_DMA_INTEN_FBEE | EMAC_DMA_INTEN_AIE | EMAC_DMA_INTEN_RBUE));

    heth->gState = HAL_ETH_STATE_STARTED;
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Stop Ethernet MAC and DMA reception/transmission
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_Stop(ETH_HandleTypeDef *heth)
{
  if (heth->gState == HAL_ETH_STATE_STARTED)
  {
    /* Set the ETH peripheral state to BUSY */
    heth->gState = HAL_ETH_STATE_BUSY;
    /* Disable the DMA transmission */
    CLEAR_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_ST);

    /* Disable the DMA reception */
    CLEAR_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_SR);

    /* Disable the MAC reception */
    CLEAR_BIT(heth->Instance->CONFIG, EMAC_CONFIG_RE);

    /* Flush Transmit FIFO */
    ETH_FlushTransmitFIFO(heth);

    /* Disable the MAC transmission */
    CLEAR_BIT(heth->Instance->CONFIG, EMAC_CONFIG_TE);

    heth->gState = HAL_ETH_STATE_READY;

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Stop Ethernet MAC and DMA reception/transmission in Interrupt mode
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_Stop_IT(ETH_HandleTypeDef *heth)
{
  ETH_DMADescTypeDef *dmarxdesc;
  uint32_t descindex;

  if (heth->gState == HAL_ETH_STATE_STARTED)
  {
    /* Set the ETH peripheral state to BUSY */
    heth->gState = HAL_ETH_STATE_BUSY;

    __HAL_ETH_DMA_DISABLE_IT(heth, (EMAC_DMA_INTEN_NIE | EMAC_DMA_INTEN_RIE | EMAC_DMA_INTEN_TCE  |
                                   EMAC_DMA_INTEN_FBEE | EMAC_DMA_INTEN_AIE | EMAC_DMA_INTEN_RBUE));

    /* Disable the DMA transmission */
    CLEAR_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_ST);

    /* Disable the DMA reception */
    CLEAR_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_SR);

    /* Disable the MAC reception */
    CLEAR_BIT(heth->Instance->CONFIG, EMAC_CONFIG_TE);
    /* Flush Transmit FIFO */
    ETH_FlushTransmitFIFO(heth);

    /* Disable the MAC transmission */
    CLEAR_BIT(heth->Instance->CONFIG, EMAC_CONFIG_RE);

    /* Clear IOC bit to all Rx descriptors */
    for (descindex = 0; descindex < (uint32_t)ETH_RX_DESC_CNT; descindex++)
    {
      dmarxdesc = (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[descindex];
      SET_BIT(dmarxdesc->DESC1, ETH_DMARXDESC_DIC);
    }

    heth->RxDescList.ItMode = 0U;

    heth->gState = HAL_ETH_STATE_READY;

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Sends an Ethernet Packet in polling mode.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pTxConfig: Hold the configuration of packet to be transmitted
  * @param  Timeout: timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_Transmit(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig, uint32_t Timeout)
{
  uint32_t tickstart;
  ETH_DMADescTypeDef *dmatxdesc;

  if (pTxConfig == NULL)
  {
    heth->ErrorCode |= HAL_ETH_ERROR_PARAM;
    return HAL_ERROR;
  }

  if (heth->gState == HAL_ETH_STATE_STARTED)
  {
    /* Config DMA Tx descriptor by Tx Packet info */
    if (ETH_Prepare_Tx_Descriptors(heth, pTxConfig, 0) != HAL_ETH_ERROR_NONE)
    {
      /* Set the ETH error code */
      heth->ErrorCode |= HAL_ETH_ERROR_BUSY;
      return HAL_ERROR;
    }

    /* Ensure completion of descriptor preparation before transmission start */
    __DSB();

    dmatxdesc = (ETH_DMADescTypeDef *)(&heth->TxDescList)->TxDesc[heth->TxDescList.CurTxDesc];

    /* Incr current tx desc index */
    INCR_TX_DESC_INDEX(heth->TxDescList.CurTxDesc, 1U);

    /* Start transmission */
    /* issue a poll command to Tx DMA by writing address of next immediate free descriptor */
    WRITE_REG(heth->Instance->DMA_TPD, (uint32_t)(heth->TxDescList.TxDesc[heth->TxDescList.CurTxDesc]));

    tickstart = HAL_GetTick();

    /* Wait for data to be transmitted or timeout occurred */
    while ((dmatxdesc->DESC0 & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
    {
      if ((heth->Instance->DMA_DMASTATUS & EMAC_DMA_STATUS_FBE) != (uint32_t)RESET)
      {
        heth->ErrorCode |= HAL_ETH_ERROR_DMA;
        heth->DMAErrorCode = heth->Instance->DMA_DMASTATUS;
        /* Return function status */
        return HAL_ERROR;
      }

      /* Check for the Timeout */
      if (Timeout != HAL_MAX_DELAY)
      {
        if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          heth->ErrorCode |= HAL_ETH_ERROR_TIMEOUT;
          /* Clear TX descriptor so that we can proceed */
          dmatxdesc->DESC0 = (ETH_DMATXDESC_FS | ETH_DMATXDESC_LS);
          return HAL_ERROR;
        }
      }
    }

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Sends an Ethernet Packet in interrupt mode.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pTxConfig: Hold the configuration of packet to be transmitted
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_Transmit_IT(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig)
{
  if (pTxConfig == NULL)
  {
    heth->ErrorCode |= HAL_ETH_ERROR_PARAM;
    return HAL_ERROR;
  }

  if (heth->gState == HAL_ETH_STATE_STARTED)
  {
    /* Save the packet pointer to release.  */
    heth->TxDescList.CurrentPacketAddress = (uint32_t *)pTxConfig->pData;

    /* Config DMA Tx descriptor by Tx Packet info */
    if (ETH_Prepare_Tx_Descriptors(heth, pTxConfig, 1) != HAL_ETH_ERROR_NONE)
    {
      heth->ErrorCode |= HAL_ETH_ERROR_BUSY;
      return HAL_ERROR;
    }

    /* Ensure completion of descriptor preparation before transmission start */
    __DSB();

    /* Incr current tx desc index */
    INCR_TX_DESC_INDEX(heth->TxDescList.CurTxDesc, 1U);

    /* Start transmission */
    /* issue a poll command to Tx DMA by writing address of next immediate free descriptor */
    if (((heth->Instance)->DMA_DMASTATUS & EMAC_DMA_STATUS_TBU) != (uint32_t)RESET)
    {
      /* Clear TBUS ETHERNET DMA flag */
      (heth->Instance)->DMA_DMASTATUS = EMAC_DMA_STATUS_TBU;
      /* Resume DMA transmission*/
      (heth->Instance)->DMA_TPD = 0U;
    }

    return HAL_OK;

  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Read a received packet.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pAppBuff: Pointer to an application buffer to receive the packet.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_ReadData(ETH_HandleTypeDef *heth, void **pAppBuff)
{
  uint32_t descidx;
  ETH_DMADescTypeDef *dmarxdesc;
  uint32_t desccnt = 0U;
  uint32_t desccntmax;
  uint32_t bufflength;
  uint8_t rxdataready = 0U;


  if (pAppBuff == NULL)
  {
    heth->ErrorCode |= HAL_ETH_ERROR_PARAM;
    return HAL_ERROR;
  }

  if (heth->gState != HAL_ETH_STATE_STARTED)
  {
    return HAL_ERROR;
  }

  descidx = heth->RxDescList.RxDescIdx;
  dmarxdesc = (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[descidx];
  desccntmax = ETH_RX_DESC_CNT - heth->RxDescList.RxBuildDescCnt;

  /* Check if descriptor is not owned by DMA */
  while ((READ_BIT(dmarxdesc->DESC0, ETH_DMARXDESC_OWN) == (uint32_t)RESET) && (desccnt < desccntmax)
         && (rxdataready == 0U))
  {
    if (READ_BIT(dmarxdesc->DESC0,  ETH_DMARXDESC_LS)  != (uint32_t)RESET)
    {
      /* Get timestamp high */
      heth->RxDescList.TimeStamp.TimeStampHigh = dmarxdesc->DESC6;
      /* Get timestamp low */
      heth->RxDescList.TimeStamp.TimeStampLow  = dmarxdesc->DESC7;
    }
    if ((READ_BIT(dmarxdesc->DESC0, ETH_DMARXDESC_FS) != (uint32_t)RESET) || (heth->RxDescList.pRxStart != NULL))
    {
      /* Check first descriptor */
      if (READ_BIT(dmarxdesc->DESC0, ETH_DMARXDESC_FS) != (uint32_t)RESET)
      {
        heth->RxDescList.RxDescCnt = 0;
        heth->RxDescList.RxDataLength = 0;
      }

      /* Check if last descriptor */
      bufflength = heth->Init.RxBuffLen;
      if (READ_BIT(dmarxdesc->DESC0, ETH_DMARXDESC_LS) != (uint32_t)RESET)
      {
        /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
        bufflength = ((dmarxdesc->DESC0 & ETH_DMARXDESC_FL) >> ETH_DMARXDESC_FRAMELENGTHSHIFT) - 4U;

        /* Save Last descriptor index */
        heth->RxDescList.pRxLastRxDesc = dmarxdesc->DESC0;

        /* Packet ready */
        rxdataready = 1;
      }

      /* Link data */
      WRITE_REG(dmarxdesc->BackupAddr0, dmarxdesc->DESC2);
#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
      /*Call registered Link callback*/
      heth->rxLinkCallback(&heth->RxDescList.pRxStart, &heth->RxDescList.pRxEnd,
                           (uint8_t *)dmarxdesc->BackupAddr0, bufflength);
#else
      /* Link callback */
      HAL_ETH_RxLinkCallback(&heth->RxDescList.pRxStart, &heth->RxDescList.pRxEnd,
                             (uint8_t *)dmarxdesc->BackupAddr0, (uint16_t) bufflength);
#endif  /* USE_HAL_ETH_REGISTER_CALLBACKS */
      heth->RxDescList.RxDescCnt++;
      heth->RxDescList.RxDataLength += bufflength;

      /* Clear buffer pointer */
      dmarxdesc->BackupAddr0 = 0;
    }

    /* Increment current rx descriptor index */
    INCR_RX_DESC_INDEX(descidx, 1U);
    /* Get current descriptor address */
    dmarxdesc = (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[descidx];
    desccnt++;
  }

  heth->RxDescList.RxBuildDescCnt += desccnt;
  if ((heth->RxDescList.RxBuildDescCnt) != 0U)
  {
    /* Update Descriptors */
    ETH_UpdateDescriptor(heth);
  }

  heth->RxDescList.RxDescIdx = descidx;

  if (rxdataready == 1U)
  {
    /* Return received packet */
    *pAppBuff = heth->RxDescList.pRxStart;
    /* Reset first element */
    heth->RxDescList.pRxStart = NULL;

    return HAL_OK;
  }

  /* Packet not ready */
  return HAL_ERROR;
}

/**
  * @brief  This function gives back Rx Desc of the last received Packet
  *         to the DMA, so ETH DMA will be able to use these descriptors
  *         to receive next Packets.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
static void ETH_UpdateDescriptor(ETH_HandleTypeDef *heth)
{
  uint32_t descidx;
  uint32_t desccount;
  ETH_DMADescTypeDef *dmarxdesc;
  uint8_t *buff = NULL;
  uint8_t allocStatus = 1U;

  descidx = heth->RxDescList.RxBuildDescIdx;
  dmarxdesc = (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[descidx];
  desccount = heth->RxDescList.RxBuildDescCnt;

  while ((desccount > 0U) && (allocStatus != 0U))
  {
    /* Check if a buffer's attached the descriptor */
    if (READ_REG(dmarxdesc->BackupAddr0) == 0U)
    {
      /* Get a new buffer. */
#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
      /*Call registered Allocate callback*/
      heth->rxAllocateCallback(&buff);
#else
      /* Allocate callback */
      HAL_ETH_RxAllocateCallback(&buff);
#endif  /* USE_HAL_ETH_REGISTER_CALLBACKS */
      if (buff == NULL)
      {
        allocStatus = 0U;
      }
      else
      {
        WRITE_REG(dmarxdesc->BackupAddr0, (uint32_t)buff);
        WRITE_REG(dmarxdesc->DESC2, (uint32_t)buff);
      }
    }

    if (allocStatus != 0U)
    {
      /* Ensure rest of descriptor is written to RAM before the OWN bit */
      __DMB();

      WRITE_REG(dmarxdesc->DESC0, ETH_DMARXDESC_OWN);

      if (heth->RxDescList.ItMode == 0U)
      {
        WRITE_REG(dmarxdesc->DESC1, ETH_DMARXDESC_DIC | 1000U | ETH_DMARXDESC_RCH);
      }
      else
      {
        WRITE_REG(dmarxdesc->DESC1, 1000U | ETH_DMARXDESC_RCH);
      }

      /* Increment current rx descriptor index */
      INCR_RX_DESC_INDEX(descidx, 1U);
      /* Get current descriptor address */
      dmarxdesc = (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[descidx];
      desccount--;
    }
  }

  if (heth->RxDescList.RxBuildDescCnt != desccount)
  {
    /* Set the Tail pointer address */
    WRITE_REG(heth->Instance->DMA_RPD, 0);

    heth->RxDescList.RxBuildDescIdx = descidx;
    heth->RxDescList.RxBuildDescCnt = desccount;
  }
}

/**
  * @brief  Register the Rx alloc callback.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  rxAllocateCallback: pointer to function to alloc buffer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_RegisterRxAllocateCallback(ETH_HandleTypeDef *heth,
                                                     pETH_rxAllocateCallbackTypeDef rxAllocateCallback)
{
  if (rxAllocateCallback == NULL)
  {
    /* No buffer to save */
    return HAL_ERROR;
  }

  /* Set function to allocate buffer */
  heth->rxAllocateCallback = rxAllocateCallback;

  return HAL_OK;
}

/**
  * @brief  Unregister the Rx alloc callback.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_UnRegisterRxAllocateCallback(ETH_HandleTypeDef *heth)
{
  /* Set function to allocate buffer */
  heth->rxAllocateCallback = HAL_ETH_RxAllocateCallback;

  return HAL_OK;
}

/**
  * @brief  Rx Allocate callback.
  * @param  buff: pointer to allocated buffer
  * @retval None
  */
__weak void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(buff);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_RxAllocateCallback could be implemented in the user file
  */
}

/**
  * @brief  Rx Link callback.
  * @param  pStart: pointer to packet start
  * @param  pStart: pointer to packet end
  * @param  buff: pointer to received data
  * @param  Length: received data length
  * @retval None
  */
__weak void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pStart);
  UNUSED(pEnd);
  UNUSED(buff);
  UNUSED(Length);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_RxLinkCallback could be implemented in the user file
  */
}

/**
  * @brief  Set the Rx link data function.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  rxLinkCallback: pointer to function to link data
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_RegisterRxLinkCallback(ETH_HandleTypeDef *heth, pETH_rxLinkCallbackTypeDef rxLinkCallback)
{
  if (rxLinkCallback == NULL)
  {
    /* No buffer to save */
    return HAL_ERROR;
  }

  /* Set function to link data */
  heth->rxLinkCallback = rxLinkCallback;

  return HAL_OK;
}

/**
  * @brief  Unregister the Rx link callback.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_UnRegisterRxLinkCallback(ETH_HandleTypeDef *heth)
{
  /* Set function to allocate buffer */
  heth->rxLinkCallback = HAL_ETH_RxLinkCallback;

  return HAL_OK;
}

/**
  * @brief  Get the error state of the last received packet.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pErrorCode: pointer to uint32_t to hold the error code
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_GetRxDataErrorCode(ETH_HandleTypeDef *heth, uint32_t *pErrorCode)
{
  /* Get error bits. */
  *pErrorCode = READ_BIT(heth->RxDescList.pRxLastRxDesc, ETH_DMARXDESC_ERRORS_MASK);

  return HAL_OK;
}

/**
  * @brief  Set the Tx free function.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  txFreeCallback: pointer to function to release the packet
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_RegisterTxFreeCallback(ETH_HandleTypeDef *heth, pETH_txFreeCallbackTypeDef txFreeCallback)
{
  if (txFreeCallback == NULL)
  {
    /* No buffer to save */
    return HAL_ERROR;
  }

  /* Set function to free transmmitted packet */
  heth->txFreeCallback = txFreeCallback;

  return HAL_OK;
}

/**
  * @brief  Unregister the Tx free callback.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_UnRegisterTxFreeCallback(ETH_HandleTypeDef *heth)
{
  /* Set function to allocate buffer */
  heth->txFreeCallback = HAL_ETH_TxFreeCallback;

  return HAL_OK;
}

/**
  * @brief  Tx Free callback.
  * @param  buff: pointer to buffer to free
  * @retval None
  */
__weak void HAL_ETH_TxFreeCallback(uint32_t *buff)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(buff);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_TxFreeCallback could be implemented in the user file
  */
}

/**
  * @brief  Release transmitted Tx packets.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_ReleaseTxPacket(ETH_HandleTypeDef *heth)
{
  ETH_TxDescListTypeDef *dmatxdesclist = &heth->TxDescList;
  uint32_t numOfBuf =  dmatxdesclist->BuffersInUse;
  uint32_t idx =       dmatxdesclist->releaseIndex;
  uint8_t pktTxStatus = 1U;
  uint8_t pktInUse;

  /* Loop through buffers in use.  */
  while ((numOfBuf != 0U) && (pktTxStatus != 0U))
  {
    pktInUse = 1U;
    numOfBuf--;
    /* If no packet, just examine the next packet.  */
    if (dmatxdesclist->PacketAddress[idx] == NULL)
    {
      /* No packet in use, skip to next.  */
      idx = (idx + 1U) & (ETH_TX_DESC_CNT - 1U);
      pktInUse = 0U;
    }

    if (pktInUse != 0U)
    {
      /* Determine if the packet has been transmitted.  */
      if ((heth->Init.TxDesc[idx].DESC0 & ETH_DMATXDESC_OWN) == 0U)
      {

#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
        /*Call registered callbacks*/
        /* Release the packet.  */
        heth->txFreeCallback(dmatxdesclist->PacketAddress[idx]);
#else
        /* Call callbacks */
        /* Release the packet.  */
        HAL_ETH_TxFreeCallback(dmatxdesclist->PacketAddress[idx]);
#endif  /* USE_HAL_ETH_REGISTER_CALLBACKS */

        /* Clear the entry in the in-use array.  */
        dmatxdesclist->PacketAddress[idx] = NULL;

        /* Update the transmit relesae index and number of buffers in use.  */
        idx = (idx + 1U) & (ETH_TX_DESC_CNT - 1U);
        dmatxdesclist->BuffersInUse = numOfBuf;
        dmatxdesclist->releaseIndex = idx;
      }
      else
      {
        /* Get out of the loop!  */
        pktTxStatus = 0U;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles ETH interrupt request.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
void HAL_ETH_IRQHandler(ETH_HandleTypeDef *heth)
{
  /* Packet received */
  if (__HAL_ETH_DMA_GET_IT(heth, EMAC_DMA_STATUS_RI))
  {
    if (__HAL_ETH_DMA_GET_IT_SOURCE(heth, EMAC_DMA_INTEN_RIE))
    {
      /* Clear the Eth DMA Rx IT pending bits */
      __HAL_ETH_DMA_CLEAR_IT(heth, EMAC_DMA_STATUS_RI | EMAC_DMA_STATUS_NIS);

#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
      /*Call registered Receive complete callback*/
      heth->RxCpltCallback(heth);
#else
      /* Receive complete callback */
      HAL_ETH_RxCpltCallback(heth);
#endif  /* USE_HAL_ETH_REGISTER_CALLBACKS */
    }
  }

  /* Packet transmitted */
  if (__HAL_ETH_DMA_GET_IT(heth, EMAC_DMA_STATUS_TC))
  {
    if (__HAL_ETH_DMA_GET_IT_SOURCE(heth, EMAC_DMA_INTEN_TCE))
    {
      /* Clear the Eth DMA Tx IT pending bits */
      __HAL_ETH_DMA_CLEAR_IT(heth, EMAC_DMA_STATUS_TC | EMAC_DMA_STATUS_NIS);

#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
      /*Call registered Transmit complete callback*/
      heth->TxCpltCallback(heth);
#else
      /* Transfer complete callback */
      HAL_ETH_TxCpltCallback(heth);
#endif  /* USE_HAL_ETH_REGISTER_CALLBACKS */
    }
  }


  /* ETH DMA Error */
  if (__HAL_ETH_DMA_GET_IT(heth, EMAC_DMA_STATUS_AIS))
  {
    if (__HAL_ETH_DMA_GET_IT_SOURCE(heth, EMAC_DMA_INTEN_AIE))
    {
      heth->ErrorCode |= HAL_ETH_ERROR_DMA;

      /* if fatal bus error occurred */
      if (__HAL_ETH_DMA_GET_IT(heth, EMAC_DMA_STATUS_FBE))
      {
        /* Get DMA error code  */
        heth->DMAErrorCode = READ_BIT(heth->Instance->DMA_DMASTATUS, (EMAC_DMA_STATUS_FBE | EMAC_DMA_STATUS_TPS | EMAC_DMA_STATUS_RPS));

        /* Disable all interrupts */
        __HAL_ETH_DMA_DISABLE_IT(heth, EMAC_DMA_INTEN_NIE | EMAC_DMA_INTEN_AIE);

        /* Set HAL state to ERROR */
        heth->gState = HAL_ETH_STATE_ERROR;
      }
      else
      {
        /* Get DMA error status  */
        heth->DMAErrorCode = READ_BIT(heth->Instance->DMA_DMASTATUS, (EMAC_DMA_STATUS_ET | EMAC_DMA_STATUS_RWT |
                                                              EMAC_DMA_STATUS_RBU | EMAC_DMA_STATUS_AIS));

        /* Clear the interrupt summary flag */
        __HAL_ETH_DMA_CLEAR_IT(heth, (EMAC_DMA_STATUS_ET | EMAC_DMA_STATUS_RWT |
                                      EMAC_DMA_STATUS_RBU | EMAC_DMA_STATUS_AIS));
      }
#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
      /* Call registered Error callback*/
      heth->ErrorCallback(heth);
#else
      /* Ethernet DMA Error callback */
      HAL_ETH_ErrorCallback(heth);
#endif  /* USE_HAL_ETH_REGISTER_CALLBACKS */

    }
  }

}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_TxCpltCallback could be implemented in the user file
  */
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_RxCpltCallback could be implemented in the user file
  */
}

/**
  * @brief  Ethernet transfer error callbacks
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_ErrorCallback could be implemented in the user file
  */
}

/**
  * @brief  Ethernet Power Management module IT callback
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_PMTCallback(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_PMTCallback could be implemented in the user file
  */
}


/**
  * @brief  ETH WAKEUP interrupt callback
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_WakeUpCallback(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ETH_WakeUpCallback could be implemented in the user file
   */
}

/**
  * @brief  Read a PHY register
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  PHYAddr: PHY port address, must be a value from 0 to 31
  * @param  PHYReg: PHY register address, must be a value from 0 to 31
  * @param pRegValue: parameter to hold read value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_ReadPHYRegister(ETH_HandleTypeDef *heth, uint32_t PHYAddr, uint32_t PHYReg,
                                          uint32_t *pRegValue)
{
  uint32_t tmpreg1;
  uint32_t tickstart;

  /* Get the ETHERNET GMIIADDRESS value */
  tmpreg1 = heth->Instance->GMIIADDRESS;

  /* Keep only the CSR Clock Range CR[2:0] bits value */
  tmpreg1 &= ~ETH_MACMIIAR_CR_MASK;

  /* Prepare the MII address register value */
  tmpreg1 |= ((PHYAddr << 11U) & EMAC_GMIIADDRESS_PA);                        /* Set the PHY device address   */
  tmpreg1 |= (((uint32_t)PHYReg << 6U) & EMAC_GMIIADDRESS_GR);                /* Set the PHY register address */
  tmpreg1 &= ~EMAC_GMIIADDRESS_WR;                                            /* Set the read mode            */
  tmpreg1 |= EMAC_GMIIADDRESS_BUSY;                                             /* Set the MII Busy bit         */

  /* Write the result value into the MII Address register */
  heth->Instance->GMIIADDRESS = tmpreg1;


  tickstart = HAL_GetTick();

  /* Check for the Busy flag */
  while ((tmpreg1 & EMAC_GMIIADDRESS_BUSY) == EMAC_GMIIADDRESS_BUSY)
  {
    /* Check for the Timeout */
    if ((HAL_GetTick() - tickstart) > PHY_READ_TO)
    {
      return HAL_ERROR;
    }

    tmpreg1 = heth->Instance->GMIIADDRESS;
  }

  /* Get GMIIDATA value */
  *pRegValue = (uint16_t)(heth->Instance->GMIIDATA);

  return HAL_OK;
}


/**
  * @brief  Writes to a PHY register.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  PHYAddr: PHY port address, must be a value from 0 to 31
  * @param  PHYReg: PHY register address, must be a value from 0 to 31
  * @param  RegValue: the value to write
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_WritePHYRegister(ETH_HandleTypeDef *heth, uint32_t PHYAddr, uint32_t PHYReg,
                                           uint32_t RegValue)
{
  uint32_t tmpreg1;
  uint32_t tickstart;

  /* Get the ETHERNET GMIIADDRESS value */
  tmpreg1 = heth->Instance->GMIIADDRESS;

  /* Keep only the CSR Clock Range CR[2:0] bits value */
  tmpreg1 &= ~ETH_MACMIIAR_CR_MASK;

  /* Prepare the MII register address value */
  tmpreg1 |= ((PHYAddr << 11U) & EMAC_GMIIADDRESS_PA);                      /* Set the PHY device address */
  tmpreg1 |= (((uint32_t)PHYReg << 6U) & EMAC_GMIIADDRESS_GR);              /* Set the PHY register address */
  tmpreg1 |= EMAC_GMIIADDRESS_WR;                                           /* Set the write mode */
  tmpreg1 |= EMAC_GMIIADDRESS_BUSY;                                           /* Set the MII Busy bit */

  /* Give the value to the MII data register */
  heth->Instance->GMIIDATA = (uint16_t)RegValue;

  /* Write the result value into the MII Address register */
  heth->Instance->GMIIADDRESS = tmpreg1;

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Check for the Busy flag */
  while ((tmpreg1 & EMAC_GMIIADDRESS_BUSY) == EMAC_GMIIADDRESS_BUSY)
  {
    /* Check for the Timeout */
    if ((HAL_GetTick() - tickstart) > PHY_WRITE_TO)
    {
      return HAL_ERROR;
    }

    tmpreg1 = heth->Instance->GMIIADDRESS;
  }

  return HAL_OK;
}

/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group3 Peripheral Control functions
  *  @brief   ETH control functions
  *
@verbatim
  ==============================================================================
                      ##### Peripheral Control functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to control the ETH
    peripheral.

@endverbatim
  * @{
  */
/**
  * @brief  Get the configuration of the MAC and MTL subsystems.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  macconf: pointer to a ETH_MACConfigTypeDef structure that will hold
  *         the configuration of the MAC.
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_ETH_GetMACConfig(ETH_HandleTypeDef *heth, ETH_MACConfigTypeDef *macconf)
{
  if (macconf == NULL)
  {
    return HAL_ERROR;
  }

  /* Get MAC parameters */
  macconf->DeferralCheck = ((READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_DC) >> 4) > 0U) ? ENABLE : DISABLE;
  macconf->BackOffLimit = READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_BL);
  macconf->RetryTransmission = ((READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_RTPD) >> 9) == 0U) ? ENABLE : DISABLE;
  macconf->CarrierSenseDuringTransmit = ((READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_CSPD) >> 16) > 0U) ? ENABLE : DISABLE;
  macconf->ReceiveOwn = ((READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_RCPD) >> 13) == 0U) ? ENABLE : DISABLE;
  macconf->LoopbackMode = ((READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_LB) >> 12) > 0U) ? ENABLE : DISABLE;
  macconf->DuplexMode = READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_DP);
  macconf->Speed = READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_SPEED);
  macconf->Jabber = ((READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_JPD) >> 22) == 0U) ? ENABLE : DISABLE;
  macconf->Watchdog = ((READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_WDGPD) >> 23) == 0U) ? ENABLE : DISABLE;
  macconf->AutomaticPadCRCStrip = ((READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_ACS) >> 7) > 0U) ? ENABLE : DISABLE;
  macconf->InterPacketGapVal = READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_IFG);
  macconf->ChecksumOffload = ((READ_BIT(heth->Instance->CONFIG, EMAC_CONFIG_TKPE) >> 27) > 0U) ? ENABLE : DISABLE;


  macconf->TransmitFlowControl = ((READ_BIT(heth->Instance->FLOWCONTROL, EMAC_FLOWCONTROL_TFE) >> 1) > 0U) ? ENABLE : DISABLE;
  macconf->ZeroQuantaPause = ((READ_BIT(heth->Instance->FLOWCONTROL, EMAC_FLOWCONTROL_DZPQ) >> 7) == 0U) ? ENABLE : DISABLE;
  macconf->PauseLowThreshold = READ_BIT(heth->Instance->FLOWCONTROL, EMAC_FLOWCONTROL_PLT);
  macconf->PauseTime = (READ_BIT(heth->Instance->FLOWCONTROL, EMAC_FLOWCONTROL_PT) >> 16);
  macconf->ReceiveFlowControl = (READ_BIT(heth->Instance->FLOWCONTROL, EMAC_FLOWCONTROL_RFE) > 0U) ? ENABLE : DISABLE;
  macconf->UnicastPausePacketDetect = ((READ_BIT(heth->Instance->FLOWCONTROL, EMAC_FLOWCONTROL_UP) >> 1) > 0U)
                                      ? ENABLE : DISABLE;

  return HAL_OK;
}

/**
  * @brief  Get the configuration of the DMA.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  dmaconf: pointer to a ETH_DMAConfigTypeDef structure that will hold
  *         the configuration of the ETH DMA.
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_ETH_GetDMAConfig(ETH_HandleTypeDef *heth, ETH_DMAConfigTypeDef *dmaconf)
{
  if (dmaconf == NULL)
  {
    return HAL_ERROR;
  }

  dmaconf->DMAArbitration = READ_BIT(heth->Instance->DMA_BUSMODE, EMAC_DMA_BUSMODE_DSL) >> 2;
  dmaconf->AddressAlignedBeats = ((READ_BIT(heth->Instance->DMA_BUSMODE, EMAC_DMA_BUSMODE_ALIGN) >> 25) > 0U) ? ENABLE : DISABLE;
  dmaconf->RxDMABurstLength = READ_BIT(heth->Instance->DMA_BUSMODE, EMAC_DMA_BUSMODE_RXPBL);
  dmaconf->TxDMABurstLength = READ_BIT(heth->Instance->DMA_BUSMODE, EMAC_DMA_BUSMODE_TXPBL);
  dmaconf->DescriptorSkipLength = READ_BIT(heth->Instance->DMA_BUSMODE, EMAC_DMA_BUSMODE_DSL) >> 2;

  dmaconf->DropTCPIPChecksumErrorFrame = ((READ_BIT(heth->Instance->DMA_OPMODE,
                                                    EMAC_DMA_OPMODE_DT) >> 26) > 0U) ? DISABLE : ENABLE;
  dmaconf->ReceiveStoreForward = ((READ_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_RSF) >> 25) > 0U) ? ENABLE : DISABLE;
  dmaconf->FlushRxPacket = ((READ_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_FTF) >> 20) > 0U) ? DISABLE : ENABLE;
  dmaconf->TransmitStoreForward = ((READ_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_TSF) >> 21) > 0U) ? ENABLE : DISABLE;
  dmaconf->TransmitThresholdControl = READ_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_TT);
  dmaconf->ForwardErrorFrames = ((READ_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_FEF) >> 7) > 0U) ? ENABLE : DISABLE;
  dmaconf->ForwardUndersizedGoodFrames = ((READ_BIT(heth->Instance->DMA_OPMODE,
                                                    EMAC_DMA_OPMODE_FUGF) >> 6) > 0U) ? ENABLE : DISABLE;
  dmaconf->ReceiveThresholdControl = READ_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_RT);
  dmaconf->SecondFrameOperate = ((READ_BIT(heth->Instance->DMA_OPMODE, EMAC_DMA_OPMODE_OSF) >> 2) > 0U) ? ENABLE : DISABLE;
  return HAL_OK;
}

/**
  * @brief  Set the MAC configuration.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  macconf: pointer to a ETH_MACConfigTypeDef structure that contains
  *         the configuration of the MAC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_SetMACConfig(ETH_HandleTypeDef *heth,  ETH_MACConfigTypeDef *macconf)
{
  if (macconf == NULL)
  {
    return HAL_ERROR;
  }

  if (heth->gState == HAL_ETH_STATE_READY)
  {
    ETH_SetMACConfig(heth, macconf);

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Set the ETH DMA configuration.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  dmaconf: pointer to a ETH_DMAConfigTypeDef structure that will hold
  *         the configuration of the ETH DMA.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_SetDMAConfig(ETH_HandleTypeDef *heth,  ETH_DMAConfigTypeDef *dmaconf)
{
  if (dmaconf == NULL)
  {
    return HAL_ERROR;
  }

  if (heth->gState == HAL_ETH_STATE_READY)
  {
    ETH_SetDMAConfig(heth, dmaconf);

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Configures the Clock range of ETH MDIO interface.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
void HAL_ETH_SetMDIOClockRange(ETH_HandleTypeDef *heth)
{
  uint32_t hclk;
  uint32_t tmpreg;

  /* Get the ETHERNET GMIIADDRESS value */
  tmpreg = (heth->Instance)->GMIIADDRESS;
  /* Clear CSR Clock Range CR[2:0] bits */
  tmpreg &= ETH_MACMIIAR_CR_MASK;

  /* Get hclk frequency value */
  hclk = HAL_RCM_GetHCLKFreq();

  /* Set CR bits depending on hclk value */
  if ((hclk >= 20000000U) && (hclk < 35000000U))
  {
    /* CSR Clock Range between 20-35 MHz */
    tmpreg |= (uint32_t)EMAC_GMIIADDRESS_CR_DIV16;
  }
  else if ((hclk >= 35000000U) && (hclk < 60000000U))
  {
    /* CSR Clock Range between 35-60 MHz */
    tmpreg |= (uint32_t)EMAC_GMIIADDRESS_CR_DIV26;
  }
  else if ((hclk >= 60000000U) && (hclk < 100000000U))
  {
    /* CSR Clock Range between 60-100 MHz */
    tmpreg |= (uint32_t)EMAC_GMIIADDRESS_CR_DIV42;
  }
  else if ((hclk >= 100000000U) && (hclk < 150000000U))
  {
    /* CSR Clock Range between 100-150 MHz */
    tmpreg |= (uint32_t)EMAC_GMIIADDRESS_CR_DIV62;
  }
  else /* ((hclk >= 150000000)&&(hclk <= 183000000))*/
  {
    /* CSR Clock Range between 150-183 MHz */
    tmpreg |= (uint32_t)EMAC_GMIIADDRESS_CR_DIV102;
  }

  /* Write to ETHERNET MAC MIIAR: Configure the ETHERNET CSR Clock Range */
  (heth->Instance)->GMIIADDRESS = (uint32_t)tmpreg;
}

/**
  * @brief  Set the ETH MAC (L2) Filters configuration.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pFilterConfig: pointer to a ETH_MACFilterConfigTypeDef structure that contains
  *         the configuration of the ETH MAC filters.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_SetMACFilterConfig(ETH_HandleTypeDef *heth, ETH_MACFilterConfigTypeDef *pFilterConfig)
{
  uint32_t filterconfig;

  if (pFilterConfig == NULL)
  {
    return HAL_ERROR;
  }

  filterconfig = ((uint32_t)pFilterConfig->PromiscuousMode |
                  ((uint32_t)pFilterConfig->HashUnicast << 1) |
                  ((uint32_t)pFilterConfig->HashMulticast << 2)  |
                  ((uint32_t)pFilterConfig->DestAddrInverseFiltering << 3) |
                  ((uint32_t)pFilterConfig->PassAllMulticast << 4) |
                  ((uint32_t)((pFilterConfig->BroadcastFilter == DISABLE) ? 1U : 0U) << 5) |
                  ((uint32_t)pFilterConfig->SrcAddrInverseFiltering << 8) |
                  ((uint32_t)pFilterConfig->SrcAddrFiltering << 9) |
                  ((uint32_t)pFilterConfig->HachOrPerfectFilter << 10) |
                  ((uint32_t)pFilterConfig->ReceiveAllMode << 31) |
                  pFilterConfig->ControlPacketsFilter);

  MODIFY_REG(heth->Instance->FRAMEFILTER, ETH_MACFFR_MASK, filterconfig);

  return HAL_OK;
}

/**
  * @brief  Get the ETH MAC (L2) Filters configuration.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pFilterConfig: pointer to a ETH_MACFilterConfigTypeDef structure that will hold
  *         the configuration of the ETH MAC filters.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_GetMACFilterConfig(ETH_HandleTypeDef *heth, ETH_MACFilterConfigTypeDef *pFilterConfig)
{
  if (pFilterConfig == NULL)
  {
    return HAL_ERROR;
  }

  pFilterConfig->PromiscuousMode = ((READ_BIT(heth->Instance->FRAMEFILTER, EMAC_FRAMEFILTER_PR)) > 0U) ? ENABLE : DISABLE;
  pFilterConfig->HashUnicast = ((READ_BIT(heth->Instance->FRAMEFILTER, EMAC_FRAMEFILTER_HUC) >> 1) > 0U) ? ENABLE : DISABLE;
  pFilterConfig->HashMulticast = ((READ_BIT(heth->Instance->FRAMEFILTER, EMAC_FRAMEFILTER_HMC) >> 2) > 0U) ? ENABLE : DISABLE;
  pFilterConfig->DestAddrInverseFiltering = ((READ_BIT(heth->Instance->FRAMEFILTER,
                                                       EMAC_FRAMEFILTER_DAIF) >> 3) > 0U) ? ENABLE : DISABLE;
  pFilterConfig->PassAllMulticast = ((READ_BIT(heth->Instance->FRAMEFILTER, EMAC_FRAMEFILTER_PM) >> 4) > 0U) ? ENABLE : DISABLE;
  pFilterConfig->BroadcastFilter = ((READ_BIT(heth->Instance->FRAMEFILTER, EMAC_FRAMEFILTER_DBF) >> 5) == 0U) ? ENABLE : DISABLE;
  pFilterConfig->ControlPacketsFilter = READ_BIT(heth->Instance->FRAMEFILTER, EMAC_FRAMEFILTER_PCF);
  pFilterConfig->SrcAddrInverseFiltering = ((READ_BIT(heth->Instance->FRAMEFILTER,
                                                      EMAC_FRAMEFILTER_SAIF) >> 8) > 0U) ? ENABLE : DISABLE;
  pFilterConfig->SrcAddrFiltering = ((READ_BIT(heth->Instance->FRAMEFILTER, EMAC_FRAMEFILTER_SAF) >> 9) > 0U) ? ENABLE : DISABLE;
  pFilterConfig->HachOrPerfectFilter = ((READ_BIT(heth->Instance->FRAMEFILTER, EMAC_FRAMEFILTER_HPF) >> 10) > 0U)
                                       ? ENABLE : DISABLE;
  pFilterConfig->ReceiveAllMode = ((READ_BIT(heth->Instance->FRAMEFILTER, EMAC_FRAMEFILTER_RA) >> 31) > 0U) ? ENABLE : DISABLE;

  return HAL_OK;
}

/**
  * @brief  Set the source MAC Address to be matched.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  AddrNbr: The MAC address to configure
  *          This parameter must be a value of the following:
  *            ETH_MAC_ADDRESS1
  *            ETH_MAC_ADDRESS2
  *            ETH_MAC_ADDRESS3
  * @param  pMACAddr: Pointer to MAC address buffer data (6 bytes)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_SetSourceMACAddrMatch(ETH_HandleTypeDef *heth, uint32_t AddrNbr, uint8_t *pMACAddr)
{
  uint32_t macaddrlr;
  uint32_t macaddrhr;

  if (pMACAddr == NULL)
  {
    return HAL_ERROR;
  }

  /* Get mac addr high reg offset */
  macaddrhr = ((uint32_t) &(heth->Instance->MACADDR[0].hi) + AddrNbr);
  /* Get mac addr low reg offset */
  macaddrlr = ((uint32_t) &(heth->Instance->MACADDR[0].lo) + AddrNbr);

  /* Set MAC addr bits 32 to 47 */
  (*(__IO uint32_t *)macaddrhr) = (((uint32_t)(pMACAddr[5]) << 8) | (uint32_t)pMACAddr[4]);
  /* Set MAC addr bits 0 to 31 */
  (*(__IO uint32_t *)macaddrlr) = (((uint32_t)(pMACAddr[3]) << 24) | ((uint32_t)(pMACAddr[2]) << 16) |
                                   ((uint32_t)(pMACAddr[1]) << 8) | (uint32_t)pMACAddr[0]);


  return HAL_OK;
}

/**
  * @brief  Set the ETH Hash Table Value.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pHashTable: pointer to a table of two 32 bit values, that contains
  *         the 64 bits of the hash table.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_SetHashTable(ETH_HandleTypeDef *heth, uint32_t *pHashTable)
{
  if (pHashTable == NULL)
  {
    return HAL_ERROR;
  }

  heth->Instance->HASHTABLEHIGH = pHashTable[0];
  heth->Instance->HASHTABLELOW = pHashTable[1];

  return HAL_OK;
}

/**
  * @brief  Set the VLAN Identifier for Rx packets
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  ComparisonBits: 12 or 16 bit comparison mode
            must be a value of @ref ETH_VLAN_Tag_Comparison
  * @param  VLANIdentifier: VLAN Identifier value
  * @retval None
  */
void HAL_ETH_SetRxVLANIdentifier(ETH_HandleTypeDef *heth, uint32_t ComparisonBits, uint32_t VLANIdentifier)
{
  MODIFY_REG(heth->Instance->VLANTAG, EMAC_VLANTAG_VLT, VLANIdentifier);
  if (ComparisonBits == ETH_VLANTAGCOMPARISON_16BIT)
  {
    CLEAR_BIT(heth->Instance->VLANTAG, EMAC_VLANTAG_ETV);
  }
  else
  {
    SET_BIT(heth->Instance->VLANTAG, EMAC_VLANTAG_ETV);
  }
}

/**
  * @brief  Enters the Power down mode.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pPowerDownConfig: a pointer to ETH_PowerDownConfigTypeDef structure
  *         that contains the Power Down configuration
  * @retval None.
  */
void HAL_ETH_EnterPowerDownMode(ETH_HandleTypeDef *heth, ETH_PowerDownConfigTypeDef *pPowerDownConfig)
{
}

/**
  * @brief  Exits from the Power down mode.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None.
  */
void HAL_ETH_ExitPowerDownMode(ETH_HandleTypeDef *heth)
{
}

/**
  * @brief  Set the WakeUp filter.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pFilter: pointer to filter registers values
  * @param  Count: number of filter registers, must be from 1 to 8.
  * @retval None.
  */
HAL_StatusTypeDef HAL_ETH_SetWakeUpFilter(ETH_HandleTypeDef *heth, uint32_t *pFilter, uint32_t Count)
{
  return HAL_OK;
}

/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group4 Peripheral State and Errors functions
  *  @brief   ETH State and Errors functions
  *
@verbatim
  ==============================================================================
                 ##### Peripheral State and Errors functions #####
  ==============================================================================
 [..]
   This subsection provides a set of functions allowing to return the State of
   ETH communication process, return Peripheral Errors occurred during communication
   process


@endverbatim
  * @{
  */

/**
  * @brief  Returns the ETH state.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL state
  */
HAL_ETH_StateTypeDef HAL_ETH_GetState(ETH_HandleTypeDef *heth)
{
  return heth->gState;
}

/**
  * @brief  Returns the ETH error code
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval ETH Error Code
  */
uint32_t HAL_ETH_GetError(ETH_HandleTypeDef *heth)
{
  return heth->ErrorCode;
}

/**
  * @brief  Returns the ETH DMA error code
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval ETH DMA Error Code
  */
uint32_t HAL_ETH_GetDMAError(ETH_HandleTypeDef *heth)
{
  return heth->DMAErrorCode;
}

/**
  * @brief  Returns the ETH MAC error code
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval ETH MAC Error Code
  */
uint32_t HAL_ETH_GetMACError(ETH_HandleTypeDef *heth)
{
  return heth->MACErrorCode;
}

/**
  * @brief  Returns the ETH MAC WakeUp event source
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval ETH MAC WakeUp event source
  */
uint32_t HAL_ETH_GetMACWakeUpSource(ETH_HandleTypeDef *heth)
{
  return heth->MACWakeUpEvent;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup ETH_Private_Functions   ETH Private Functions
  * @{
  */

/**
  * @brief  Clears the ETHERNET transmit FIFO.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_FlushTransmitFIFO(ETH_HandleTypeDef *heth)
{
  __IO uint32_t tmpreg = 0;

  /* Set the Flush Transmit FIFO bit */
  (heth->Instance)->DMA_OPMODE |= EMAC_DMA_OPMODE_FTF;

  /* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles */
  tmpreg = (heth->Instance)->DMA_OPMODE;
  //HAL_Delay(ETH_REG_WRITE_DELAY);
  (heth->Instance)->DMA_OPMODE = tmpreg;
}

static void ETH_SetMACConfig(ETH_HandleTypeDef *heth,  ETH_MACConfigTypeDef *macconf)
{
  uint32_t tmpreg1;

  /*------------------------ ETHERNET CONFIG Configuration --------------------*/
  /* Get the ETHERNET CONFIG value */
  tmpreg1 = (heth->Instance)->CONFIG;
  /* Clear WD, PCE, PS, TE and RE bits */
  tmpreg1 &= ETH_MACCR_CLEAR_MASK;

  tmpreg1 |= (uint32_t)(((uint32_t)((macconf->Watchdog == DISABLE) ? 1U : 0U) << 23U) |
                        ((uint32_t)((macconf->Jabber == DISABLE) ? 1U : 0U) << 22U) |
                        (uint32_t)macconf->InterPacketGapVal |
                        ((uint32_t)macconf->CarrierSenseDuringTransmit << 16U) |
                        macconf->Speed |
                        ((uint32_t)((macconf->ReceiveOwn == DISABLE) ? 1U : 0U) << 13U) |
                        ((uint32_t)macconf->LoopbackMode << 12U) |
                        macconf->DuplexMode |
                        ((uint32_t)macconf->ChecksumOffload << 10U) |
                        ((uint32_t)((macconf->RetryTransmission == DISABLE) ? 1U : 0U) << 9U) |
                        ((uint32_t)macconf->AutomaticPadCRCStrip << 7U) |
                        macconf->BackOffLimit |
                        ((uint32_t)macconf->DeferralCheck << 4U));

  /* Write to ETHERNET CONFIG */
  (heth->Instance)->CONFIG = (uint32_t)tmpreg1;//0x100CA00;

  /* Wait until the write operation will be taken into account :
  at least four TX_CLK/RX_CLK clock cycles */
  tmpreg1 = (heth->Instance)->CONFIG;
  (heth->Instance)->CONFIG = tmpreg1;//(0x101CA00)

  /*----------------------- ETHERNET FLOWCONTROL Configuration --------------------*/

  /* Get the ETHERNET FLOWCONTROL value */
  tmpreg1 = (heth->Instance)->FLOWCONTROL;
  /* Clear xx bits */
  tmpreg1 &= ETH_MACFCR_CLEAR_MASK;

  tmpreg1 |= (uint32_t)((macconf->PauseTime << 16U) |
                        (uint32_t)macconf->ZeroQuantaPause |
                        macconf->PauseLowThreshold |
                        (uint32_t)macconf->UnicastSlowProtocolPacketDetect |
                        (uint32_t)macconf->ReceiveFlowControl |
                        (uint32_t)macconf->TransmitFlowControl);

  /* Write to ETHERNET FLOWCONTROL */
  (heth->Instance)->FLOWCONTROL = (uint32_t)tmpreg1;//0x80;//

  /* Wait until the write operation will be taken into account :
  at least four TX_CLK/RX_CLK clock cycles */
  tmpreg1 = (heth->Instance)->FLOWCONTROL;
  (heth->Instance)->FLOWCONTROL = tmpreg1;
  
}

static void ETH_SetDMAConfig(ETH_HandleTypeDef *heth,  ETH_DMAConfigTypeDef *dmaconf)
{
  uint32_t tmpreg1;

  /*----------------------- ETHERNET DMA_OPMODE Configuration --------------------*/
  /* Get the ETHERNET DMA_OPMODE value */
  tmpreg1 = (heth->Instance)->DMA_OPMODE;
  /* Clear xx bits */
  tmpreg1 &= ETH_DMAOMR_CLEAR_MASK;

  tmpreg1 |= (uint32_t)(((uint32_t)((dmaconf->DropTCPIPChecksumErrorFrame == DISABLE) ? 1U : 0U) << 26U) |
                        ((uint32_t)dmaconf->ReceiveStoreForward << 25U) |
                        ((uint32_t)((dmaconf->FlushRxPacket == DISABLE) ? 1U : 0U) << 20U) |
                        ((uint32_t)dmaconf->TransmitStoreForward << 21U) |
                        dmaconf->TransmitThresholdControl |
                        ((uint32_t)dmaconf->ForwardErrorFrames << 7U) |
                        ((uint32_t)dmaconf->ForwardUndersizedGoodFrames << 6U) |
                        dmaconf->ReceiveThresholdControl |
                        ((uint32_t)dmaconf->SecondFrameOperate << 2U));

  /* Write to ETHERNET DMA_OPMODE */
  (heth->Instance)->DMA_OPMODE = 0x22000c4;//(uint32_t)tmpreg1;

  /* Wait until the write operation will be taken into account:
  at least four TX_CLK/RX_CLK clock cycles */
  tmpreg1 = (heth->Instance)->DMA_OPMODE;
  (heth->Instance)->DMA_OPMODE = tmpreg1;

  /*----------------------- ETHERNET DMA_BUSMODE Configuration --------------------*/
  (heth->Instance)->DMA_BUSMODE = (uint32_t)(((uint32_t)dmaconf->AddressAlignedBeats << 25U) |
                                        dmaconf->BurstMode |
                                        dmaconf->RxDMABurstLength | /* !! if 4xPBL is selected for Tx or
                                                                       Rx it is applied for the other */
                                        dmaconf->TxDMABurstLength |
                                        ((uint32_t)dmaconf->EnhancedDescriptorFormat << 7U) |
                                        (dmaconf->DescriptorSkipLength << 2U) |
                                        dmaconf->DMAArbitration |
                                        EMAC_DMA_BUSMODE_SEPPBL); /* Enable use of separate PBL for Rx and Tx */

  /* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles */
  tmpreg1 = (heth->Instance)->DMA_BUSMODE;
  (heth->Instance)->DMA_BUSMODE = 0x2c12000;//tmpreg1;
}

/**
  * @brief  Configures Ethernet MAC and DMA with default parameters.
  *         called by HAL_ETH_Init() API.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
static void ETH_MACDMAConfig(ETH_HandleTypeDef *heth)
{
  ETH_MACConfigTypeDef macDefaultConf;
  ETH_DMAConfigTypeDef dmaDefaultConf;

  /*--------------- ETHERNET MAC registers default Configuration --------------*/
  macDefaultConf.Watchdog = ENABLE;
  macDefaultConf.Jabber = ENABLE;
  macDefaultConf.InterPacketGapVal = ETH_INTERFRAMEGAP_96BIT;
  macDefaultConf.CarrierSenseDuringTransmit = ENABLE;//DISABLE;
  macDefaultConf.ReceiveOwn = ENABLE;
  macDefaultConf.LoopbackMode = DISABLE;
  macDefaultConf.ChecksumOffload = DISABLE;//ENABLE;
  macDefaultConf.RetryTransmission = DISABLE;
  macDefaultConf.AutomaticPadCRCStrip = DISABLE;
  macDefaultConf.BackOffLimit = ETH_BACKOFFLIMIT_10;
  macDefaultConf.DeferralCheck = DISABLE;
  macDefaultConf.PauseTime = 0x0U;
  macDefaultConf.ZeroQuantaPause = DISABLE;
  macDefaultConf.PauseLowThreshold = ETH_PAUSELOWTHRESHOLD_MINUS4;
  macDefaultConf.ReceiveFlowControl = DISABLE;
  macDefaultConf.TransmitFlowControl = DISABLE;
  macDefaultConf.Speed = ETH_SPEED_100M;
  macDefaultConf.DuplexMode = ETH_FULLDUPLEX_MODE;
  macDefaultConf.UnicastSlowProtocolPacketDetect = DISABLE;

  /* MAC default configuration */
  ETH_SetMACConfig(heth, &macDefaultConf);

  /*--------------- ETHERNET DMA registers default Configuration --------------*/
  dmaDefaultConf.DropTCPIPChecksumErrorFrame = ENABLE;
  dmaDefaultConf.ReceiveStoreForward = ENABLE;
  dmaDefaultConf.FlushRxPacket = ENABLE;
  dmaDefaultConf.TransmitStoreForward = ENABLE;
  dmaDefaultConf.TransmitThresholdControl = ETH_TRANSMITTHRESHOLDCONTROL_64BYTES;
  dmaDefaultConf.ForwardErrorFrames = ENABLE;//DISABLE;
  dmaDefaultConf.ForwardUndersizedGoodFrames = ENABLE;//DISABLE;
  dmaDefaultConf.ReceiveThresholdControl = ETH_RECEIVEDTHRESHOLDCONTROL_64BYTES;
  dmaDefaultConf.SecondFrameOperate = ENABLE;
  dmaDefaultConf.AddressAlignedBeats = ENABLE;
  dmaDefaultConf.BurstMode = ETH_BURSTLENGTH_FIXED;
  dmaDefaultConf.RxDMABurstLength = ETH_RXDMABURSTLENGTH_32BEAT;
  dmaDefaultConf.TxDMABurstLength = ETH_TXDMABURSTLENGTH_32BEAT;
  dmaDefaultConf.EnhancedDescriptorFormat = DISABLE;//ENABLE;
  dmaDefaultConf.DescriptorSkipLength = 0x0U;
  dmaDefaultConf.DMAArbitration = ETH_DMAARBITRATION_ROUNDROBIN_RXTX_1_1;

  /* DMA default configuration */
  ETH_SetDMAConfig(heth, &dmaDefaultConf);
}

/**
  * @brief  Configures the selected MAC address.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  MacAddr The MAC address to configure
  *          This parameter can be one of the following values:
  *             @arg ETH_MAC_Address0: MAC Address0
  *             @arg ETH_MAC_Address1: MAC Address1
  *             @arg ETH_MAC_Address2: MAC Address2
  *             @arg ETH_MAC_Address3: MAC Address3
  * @param  Addr Pointer to MAC address buffer data (6 bytes)
  * @retval HAL status
  */
static void ETH_MACAddressConfig(ETH_HandleTypeDef *heth, uint32_t MacAddr, uint8_t *Addr)
{
  uint32_t tmpreg1;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);

  /* Calculate the selected MAC address high register */
  tmpreg1 = ((uint32_t)Addr[5U] << 8U) | (uint32_t)Addr[4U];
  /* Load the selected MAC address high register */
  (*(__IO uint32_t *)((uint32_t)(ETH_MAC_ADDR_HBASE + MacAddr))) = tmpreg1;
  /* Calculate the selected MAC address low register */
  tmpreg1 = ((uint32_t)Addr[3U] << 24U) | ((uint32_t)Addr[2U] << 16U) | ((uint32_t)Addr[1U] << 8U) | Addr[0U];

  /* Load the selected MAC address low register */
  (*(__IO uint32_t *)((uint32_t)(ETH_MAC_ADDR_LBASE + MacAddr))) = tmpreg1;
}

/**
  * @brief  Initializes the DMA Tx descriptors.
  *         called by HAL_ETH_Init() API.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_DMATxDescListInit(ETH_HandleTypeDef *heth)
{
  ETH_DMADescTypeDef *dmatxdesc;
  uint32_t i;

  /* Fill each DMATxDesc descriptor with the right values */
  for (i = 0; i < (uint32_t)ETH_TX_DESC_CNT; i++)
  {
    dmatxdesc = heth->Init.TxDesc + i;

    WRITE_REG(dmatxdesc->DESC0, 0x0);
    WRITE_REG(dmatxdesc->DESC1, 0x0);
    WRITE_REG(dmatxdesc->DESC2, 0x0);
    WRITE_REG(dmatxdesc->DESC3, 0x0);

    WRITE_REG(heth->TxDescList.TxDesc[i], (uint32_t)dmatxdesc);

    /* Set Second Address Chained bit */
    SET_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_TCH);

    if (i < ((uint32_t)ETH_TX_DESC_CNT - 1U))
    {
      WRITE_REG(dmatxdesc->DESC3, (uint32_t)(heth->Init.TxDesc + i + 1U));
    }
    else
    {
      WRITE_REG(dmatxdesc->DESC3, (uint32_t)(heth->Init.TxDesc));
    }

    /* Set the DMA Tx descriptors checksum insertion */
    SET_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL);
  }

  heth->TxDescList.CurTxDesc = 0;

  /* Set Transmit Descriptor List Address */
  WRITE_REG(heth->Instance->DMA_TLA, (uint32_t) heth->Init.TxDesc);
}

/**
  * @brief  Initializes the DMA Rx descriptors in chain mode.
  *         called by HAL_ETH_Init() API.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_DMARxDescListInit(ETH_HandleTypeDef *heth)
{
  ETH_DMADescTypeDef *dmarxdesc;
  uint32_t i;

  for (i = 0; i < (uint32_t)ETH_RX_DESC_CNT; i++)
  {
    dmarxdesc =  heth->Init.RxDesc + i;

    WRITE_REG(dmarxdesc->DESC0, 0x0);
    WRITE_REG(dmarxdesc->DESC1, 0x0);
    WRITE_REG(dmarxdesc->DESC2, 0x0);
    WRITE_REG(dmarxdesc->DESC3, 0x0);
    WRITE_REG(dmarxdesc->BackupAddr0, 0x0);
    WRITE_REG(dmarxdesc->BackupAddr1, 0x0);

    /* Set Own bit of the Rx descriptor Status */
    dmarxdesc->DESC0 = ETH_DMARXDESC_OWN;

    /* Set Buffer1 size and Second Address Chained bit */
    dmarxdesc->DESC1 = ETH_DMARXDESC_RCH | ETH_RX_BUF_SIZE;

    /* Enable Ethernet DMA Rx Descriptor interrupt */
    dmarxdesc->DESC1 &= ~ETH_DMARXDESC_DIC;

    /* Set Rx descritors addresses */
    WRITE_REG(heth->RxDescList.RxDesc[i], (uint32_t)dmarxdesc);

    if (i < ((uint32_t)ETH_RX_DESC_CNT - 1U))
    {
      WRITE_REG(dmarxdesc->DESC3, (uint32_t)(heth->Init.RxDesc + i + 1U));
    }
    else
    {
      WRITE_REG(dmarxdesc->DESC3, (uint32_t)(heth->Init.RxDesc));
    }
  }

  WRITE_REG(heth->RxDescList.RxDescIdx, 0);
  WRITE_REG(heth->RxDescList.RxDescCnt, 0);
  WRITE_REG(heth->RxDescList.RxBuildDescIdx, 0);
  WRITE_REG(heth->RxDescList.RxBuildDescCnt, 0);
  WRITE_REG(heth->RxDescList.ItMode, 0);

  /* Set Receive Descriptor List Address */
  WRITE_REG(heth->Instance->DMA_RLA, (uint32_t) heth->Init.RxDesc);
}

/**
  * @brief  Prepare Tx DMA descriptor before transmission.
  *         called by HAL_ETH_Transmit_IT and HAL_ETH_Transmit_IT() API.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pTxConfig: Tx packet configuration
  * @param  ItMode: Enable or disable Tx EOT interrept
  * @retval Status
  */
static uint32_t ETH_Prepare_Tx_Descriptors(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig, uint32_t ItMode)
{
  ETH_TxDescListTypeDef *dmatxdesclist = &heth->TxDescList;
  uint32_t descidx = dmatxdesclist->CurTxDesc;
  uint32_t firstdescidx = dmatxdesclist->CurTxDesc;
  uint32_t idx;
  uint32_t descnbr = 0;
  ETH_DMADescTypeDef *dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[descidx];

  ETH_BufferTypeDef  *txbuffer = pTxConfig->TxBuffer;
  uint32_t           bd_count = 0;

  /* Current Tx Descriptor Owned by DMA: cannot be used by the application  */
  if ((READ_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_OWN) == ETH_DMATXDESC_OWN)
      || (dmatxdesclist->PacketAddress[descidx] != NULL))
  {
    return HAL_ETH_ERROR_BUSY;
  }


  descnbr += 1U;

  /* Set header or buffer 1 address */
  WRITE_REG(dmatxdesc->DESC2, (uint32_t)txbuffer->buffer);

  /* Set header or buffer 1 Length */
  MODIFY_REG(dmatxdesc->DESC1, ETH_DMATXDESC_TBS1, txbuffer->len);

  if (READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_CSUM) != 0U)
  {
    MODIFY_REG(dmatxdesc->DESC0, ETH_DMATXDESC_CIC, pTxConfig->ChecksumCtrl);
  }

  if (READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_CRCPAD) != 0U)
  {
    MODIFY_REG(dmatxdesc->DESC0, ETH_CRC_PAD_DISABLE, pTxConfig->CRCPadCtrl);
  }


  if (READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_VLANTAG) != 0U)
  {
    /* Set Vlan Type */
    SET_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_VF);
  }

  /* Mark it as First Descriptor */
  SET_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_FS);

  /* Ensure rest of descriptor is written to RAM before the OWN bit */
  __DMB();
  /* set OWN bit of FIRST descriptor */
  SET_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_OWN);

  /* only if the packet is split into more than one descriptors > 1 */
  while (txbuffer->next != NULL)
  {
    /* Clear the LD bit of previous descriptor */
    CLEAR_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_LS);
    if (ItMode != ((uint32_t)RESET))
    {
      /* Set Interrupt on completion bit */
      SET_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_IC);
    }
    else
    {
      /* Clear Interrupt on completion bit */
      CLEAR_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_IC);
    }
    /* Increment current tx descriptor index */
    INCR_TX_DESC_INDEX(descidx, 1U);
    /* Get current descriptor address */
    dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[descidx];

    /* Clear the FD bit of new Descriptor */
    CLEAR_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_FS);

    /* Current Tx Descriptor Owned by DMA: cannot be used by the application  */
    if ((READ_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_OWN) == ETH_DMATXDESC_OWN)
        || (dmatxdesclist->PacketAddress[descidx] != NULL))
    {
      descidx = firstdescidx;
      dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[descidx];

      /* clear previous desc own bit */
      for (idx = 0; idx < descnbr; idx ++)
      {
        /* Ensure rest of descriptor is written to RAM before the OWN bit */
        __DMB();

        CLEAR_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_OWN);

        /* Increment current tx descriptor index */
        INCR_TX_DESC_INDEX(descidx, 1U);
        /* Get current descriptor address */
        dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[descidx];
      }

      return HAL_ETH_ERROR_BUSY;
    }

    descnbr += 1U;

    /* Get the next Tx buffer in the list */
    txbuffer = txbuffer->next;

    /* Set header or buffer 1 address */
    WRITE_REG(dmatxdesc->DESC2, (uint32_t)txbuffer->buffer);

    /* Set header or buffer 1 Length */
    MODIFY_REG(dmatxdesc->DESC1, ETH_DMATXDESC_TBS1, txbuffer->len);

    bd_count += 1U;

    /* Ensure rest of descriptor is written to RAM before the OWN bit */
    __DMB();
    /* Set Own bit */
    SET_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_OWN);
  }

  if (ItMode != ((uint32_t)RESET))
  {
    /* Set Interrupt on completion bit */
    SET_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_IC);
  }
  else
  {
    /* Clear Interrupt on completion bit */
    CLEAR_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_IC);
  }

  /* Mark it as LAST descriptor */
  SET_BIT(dmatxdesc->DESC0, ETH_DMATXDESC_LS);
  /* Save the current packet address to expose it to the application */
  dmatxdesclist->PacketAddress[descidx] = dmatxdesclist->CurrentPacketAddress;

  dmatxdesclist->CurTxDesc = descidx;

  /* disable the interrupt */
  __disable_irq();

  dmatxdesclist->BuffersInUse += bd_count + 1U;

  /* Enable interrupts back */
  __enable_irq();


  /* Return function status */
  return HAL_ETH_ERROR_NONE;
}


#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
static void ETH_InitCallbacksToDefault(ETH_HandleTypeDef *heth)
{
  /* Init the ETH Callback settings */
  heth->TxCpltCallback   = HAL_ETH_TxCpltCallback;    /* Legacy weak TxCpltCallback   */
  heth->RxCpltCallback   = HAL_ETH_RxCpltCallback;    /* Legacy weak RxCpltCallback   */
  heth->ErrorCallback    = HAL_ETH_ErrorCallback;     /* Legacy weak ErrorCallback */
  heth->PMTCallback      = HAL_ETH_PMTCallback;       /* Legacy weak PMTCallback      */
  heth->WakeUpCallback   = HAL_ETH_WakeUpCallback;    /* Legacy weak WakeUpCallback   */
}
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */

/**
 * @brief  pointer to get_trdes_buf_point
 * @param  base_addr    trdes base address
 * @param  number       trdes index
 * @return none         
 */
void *Get_Trdes_Buf_Point( uint32_t buf_addr, uint32_t number)
{
	return (void*)(buf_addr + number*BUF_SIZE); //buffer1 address
}


/**
 * @brief  write memory
 * @param  addr        write address
 * @param  val         write data
 * @return none
 */
void write_mem(uint32_t addr, uint32_t val)
{
	*((uint32_t*)addr) = val;
}

/**
 * @brief  read memory
 * @param  addr        read address
 * @return (*(uint32_t*)addr)   value
 */
uint32_t Read_Mem(uint32_t addr)
{
	return (*(uint32_t*)addr);
}

/**
 * @brief  rdes init
 * @param  base_addr    the rdes addr
 * @param  buf_addr     receive buffer base address
 * @param  number       rdes number
 * @return none
 */
void Rdes_Init(uint32_t base_addr, uint32_t buf_addr, uint32_t number)
{
	uint32_t i;
	volatile uint32_t wdata;
	
	for(i=0;i<number;i++)
	{
		//RDES0
        wdata = RDES0_OWN;
		write_mem(base_addr + i*DES_SIZE + 0x0, wdata);
        
		//RDES1
        wdata = (RDES1_RCH + RDES1_RBS1);
		write_mem(base_addr + i*DES_SIZE + 0x4, wdata);
        
		//RDES2 buffer1 address
		wdata = buf_addr + i*BUF_SIZE;
		write_mem(base_addr + i*DES_SIZE + 0x8, wdata);
        
		//RDES3 buffer2 address
		if(number == i + 1)
        {
            wdata = base_addr;
        }
		else
        {
            wdata = base_addr + (i+1)*DES_SIZE;
        }	
		write_mem(base_addr + i*DES_SIZE + 0xc, wdata);
	}
}

/**
 * @brief  tdes init
 * @param  base_addr    tdes base address
 * @param  buf_addr     tx buffer base address
 * @param  number       tdes number
 * @return none
 */
void Tdes_Init(uint32_t base_addr, uint32_t buf_addr, uint32_t number)
{
	uint32_t i;
	volatile uint32_t wdata;
	
	for(i=0;i<number;i++)
	{
		//TDES0
        wdata = TDES0_OWN;
		(*(volatile uint32_t *)(base_addr + i*DES_SIZE + 0x0)) = wdata;
        
		//TDES1
        wdata = (TDES1_IC + TDES1_LS + TDES1_FS + TDES1_TCH + (0x80UL<<0));
		(*(volatile uint32_t *)(base_addr + i*DES_SIZE + 0x4)) = wdata;
        
		//TDES2
		wdata = buf_addr + i*BUF_SIZE; //buffer1 address
		(*(volatile uint32_t *)(base_addr + i*DES_SIZE + 0x8)) = wdata;
        
		//TDES3
		if(number == i + 1)
        {
            wdata = base_addr;
        }	
		else
        {
            wdata = base_addr + (i+1)*DES_SIZE;
        }
		(*(volatile uint32_t *)(base_addr + i*DES_SIZE + 0xc)) = wdata;
	}
}
/**
 * @brief  emac description config
 * @param  *EMAC        pointer to EMAC_T structure
 * @return none
 */
void Emac_Description_Config(EMAC_TypeDef *emac)
{
	Rdes_Init(RDES_ADDR, EMAC_RX_BUF_ADDR, BUF_CNT);

	Tdes_Init(TDES_ADDR, EMAC_TX_BUF_ADDR, BUF_CNT);
    
	emac->DMA_RLA = RDES_ADDR;
    
	emac->DMA_TLA = TDES_ADDR;
}

/**
 * @brief  trdes buf write
 * @param  base_addr    trdes base address
 * @param  number       trdes index
 * @param  *buf         pointer to buf
 * @param  len          data length
 * @return none         
 */
void Trdes_Buf_Write( uint32_t buf_addr, uint32_t number, uint8_t *buf, uint32_t len)
{
	uint32_t i;
	uint8_t *ptr;
	ptr = Get_Trdes_Buf_Point( buf_addr, number);
	for(i=0;i<len;i++)
	{
		ptr[i] = buf[i];
	}
}


/**
 * @brief  get trdes own
 * @param  base_addr    trdes base address
 * @param  number       trdes index
 * @return *len         data length
 */
uint32_t Get_Trdes_Own(uint32_t base_addr, uint32_t number, uint32_t *len)
{
	uint32_t val;
	val = Read_Mem(base_addr + number*DES_SIZE);
	if(val&0x80000000)
	{
		return 1;
	}
	else
    {
		*len = (val&0x3fff0000)>>16;
		return 0;
    }
}

/**
 * @brief  set trdes own
 * @param  base_addr    trdes base address
 * @param  number       trdes index
 * @param  len          data length
 * @return none         
 */
void Set_Trdes_Own( uint32_t base_addr, uint32_t number, uint32_t len)
{
	volatile uint32_t val_own;
	volatile uint32_t val;
	
	val_own = Read_Mem(base_addr + number*DES_SIZE);
	if(len)
	{
		val = Read_Mem(base_addr + number*DES_SIZE + 0x4);
		val = (val&0xfffff800) + (len&0x000007ff);
		write_mem( base_addr + number*DES_SIZE + 0x4, val);
	}
	
	val_own = val_own | 0x80000000;
	write_mem( base_addr + number*DES_SIZE, val_own);
}

/**
 * @brief  set dma tpd reg
 * @param  *EMAC        pointer to EMAC_T structure
 * @return none
 */
void Set_Dma_Tpd(EMAC_TypeDef *emac)
{
    emac->DMA_TPD = 1;
}


/**
 * @brief  emac set mode
 * @param  *SCU        pointer to SCU_T structure
 * @param  mode        MII & RGMII & RMII   
 * @return none
 */
void Emac_Set_Mode(SYSCFG_TypeDef *SCU, uint32_t mode)
{
    SCU->EMACMR = ((SCU->EMACMR & (~(3<<0))) | mode);
}

/**
 * @brief  sw reset
 * @param  *emac        pointer to EMAC_TypeDef structure
 * @return none
 */
void HAL_ETH_Sw_Reset(EMAC_TypeDef *emac)
{	
	/*reset*/
	emac->DMA_BUSMODE |= EMAC_DMA_BUSMODE_RESET;
    
	/*Wait for software reset*/
	while((emac->DMA_BUSMODE & EMAC_DMA_BUSMODE_RESET) != 0){};      
}

/**
 * @brief  enable MAC RX/TX
 * @param  *heth        pointer to ETH_HandleTypeDef structure
 * @return none
 */
void HAL_ETH_EnableMAC_TxRx(ETH_HandleTypeDef *heth)
{
    uint32_t tmpreg1;
    
    /* Get the ETHERNET CONFIG value */
    tmpreg1 = (heth->Instance)->CONFIG;
    tmpreg1 |= (EMAC_CONFIG_TE+EMAC_CONFIG_RE);
    (heth->Instance)->CONFIG |= tmpreg1;
    /* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = (heth->Instance)->CONFIG;
    HAL_Delay(ETH_REG_WRITE_DELAY);
    (heth->Instance)->CONFIG = tmpreg1;
    
}

/**
 * @brief  enable DMA RX/TX
 * @param  *heth        pointer to ETH_HandleTypeDef structure
 * @return none
 */
void HAL_ETH_EnableDMA_TxRx(ETH_HandleTypeDef *heth)
{
    uint32_t tmpreg1;
    
    /* Get the ETHERNET CONFIG value */
    tmpreg1 = (heth->Instance)->DMA_OPMODE;
    tmpreg1 |= (EMAC_DMA_OPMODE_ST+EMAC_DMA_OPMODE_SR);
    (heth->Instance)->DMA_OPMODE |= tmpreg1;
    /* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles */
    tmpreg1 = (heth->Instance)->DMA_OPMODE;
    HAL_Delay(ETH_REG_WRITE_DELAY);
    (heth->Instance)->DMA_OPMODE = tmpreg1;
}
/**
  * @}
  */

/**
  * @}
  */

#endif /* ETH */

#endif /* HAL_ETH_MODULE_ENABLED */

/**
  * @}
  */

