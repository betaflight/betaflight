/**
  ******************************************************************************
  * @file    stm32h5xx_hal_pcd.c
  * @author  MCD Application Team
  * @brief   PCD HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the USB Peripheral Controller:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                    ##### How to use this driver #####
  ==============================================================================
    [..]
      The PCD HAL driver can be used as follows:

     (#) Declare a PCD_HandleTypeDef handle structure, for example:
         PCD_HandleTypeDef  hpcd;

     (#) Fill parameters of Init structure in HCD handle

     (#) Call HAL_PCD_Init() API to initialize the PCD peripheral (Core, Device core, ...)

     (#) Initialize the PCD low level resources through the HAL_PCD_MspInit() API:
         (##) Enable the PCD/USB Low Level interface clock using
              (+++) __HAL_RCC_USB_CLK_ENABLE(); For USB Device FS peripheral

         (##) Initialize the related GPIO clocks
         (##) Configure PCD pin-out
         (##) Configure PCD NVIC interrupt

     (#)Associate the Upper USB device stack to the HAL PCD Driver:
         (##) hpcd.pData = pdev;

     (#)Enable PCD transmission and reception:
         (##) HAL_PCD_Start();

  @endverbatim
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/** @addtogroup STM32H5xx_HAL_Driver
  * @{
  */

/** @defgroup PCD PCD
  * @brief PCD HAL module driver
  * @{
  */

#ifdef HAL_PCD_MODULE_ENABLED

#if defined (USB_DRD_FS)

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup PCD_Private_Macros PCD Private Macros
  * @{
  */
#define PCD_MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define PCD_MAX(a, b)  (((a) > (b)) ? (a) : (b))
/**
  * @}
  */

/* Private functions prototypes ----------------------------------------------*/
/** @defgroup PCD_Private_Functions PCD Private Functions
  * @{
  */

static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd);
#if (USE_USB_DOUBLE_BUFFER == 1U)
static HAL_StatusTypeDef HAL_PCD_EP_DB_Transmit(PCD_HandleTypeDef *hpcd, PCD_EPTypeDef *ep, uint16_t wEPVal);
static uint16_t HAL_PCD_EP_DB_Receive(PCD_HandleTypeDef *hpcd, PCD_EPTypeDef *ep, uint16_t wEPVal);
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PCD_Exported_Functions PCD Exported Functions
  * @{
  */

/** @defgroup PCD_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the PCD according to the specified
  *         parameters in the PCD_InitTypeDef and initialize the associated handle.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd)
{
  uint8_t i;

  /* Check the PCD handle allocation */
  if (hpcd == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_PCD_ALL_INSTANCE(hpcd->Instance));

  if (hpcd->State == HAL_PCD_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hpcd->Lock = HAL_UNLOCKED;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->SOFCallback = HAL_PCD_SOFCallback;
    hpcd->SetupStageCallback = HAL_PCD_SetupStageCallback;
    hpcd->ResetCallback = HAL_PCD_ResetCallback;
    hpcd->SuspendCallback = HAL_PCD_SuspendCallback;
    hpcd->ResumeCallback = HAL_PCD_ResumeCallback;
    hpcd->ConnectCallback = HAL_PCD_ConnectCallback;
    hpcd->DisconnectCallback = HAL_PCD_DisconnectCallback;
    hpcd->DataOutStageCallback = HAL_PCD_DataOutStageCallback;
    hpcd->DataInStageCallback = HAL_PCD_DataInStageCallback;
    hpcd->ISOOUTIncompleteCallback = HAL_PCD_ISOOUTIncompleteCallback;
    hpcd->ISOINIncompleteCallback = HAL_PCD_ISOINIncompleteCallback;
    hpcd->LPMCallback = HAL_PCDEx_LPM_Callback;
    hpcd->BCDCallback = HAL_PCDEx_BCD_Callback;

    if (hpcd->MspInitCallback == NULL)
    {
      hpcd->MspInitCallback = HAL_PCD_MspInit;
    }

    /* Init the low level hardware */
    hpcd->MspInitCallback(hpcd);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    HAL_PCD_MspInit(hpcd);
#endif /* (USE_HAL_PCD_REGISTER_CALLBACKS) */
  }

  hpcd->State = HAL_PCD_STATE_BUSY;

  /* Disable the Interrupts */
  __HAL_PCD_DISABLE(hpcd);

  /* Init endpoints structures */
  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    /* Init ep structure */
    hpcd->IN_ep[i].is_in = 1U;
    hpcd->IN_ep[i].num = i;
    /* Control until ep is activated */
    hpcd->IN_ep[i].type = EP_TYPE_CTRL;
    hpcd->IN_ep[i].maxpacket = 0U;
    hpcd->IN_ep[i].xfer_buff = 0U;
    hpcd->IN_ep[i].xfer_len = 0U;
  }

  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    hpcd->OUT_ep[i].is_in = 0U;
    hpcd->OUT_ep[i].num = i;
    /* Control until ep is activated */
    hpcd->OUT_ep[i].type = EP_TYPE_CTRL;
    hpcd->OUT_ep[i].maxpacket = 0U;
    hpcd->OUT_ep[i].xfer_buff = 0U;
    hpcd->OUT_ep[i].xfer_len = 0U;
  }

  /* Init Device */
  (void)USB_DevInit(hpcd->Instance, hpcd->Init);

  hpcd->USB_Address = 0U;
  hpcd->State = HAL_PCD_STATE_READY;

  /* Activate LPM */
  if (hpcd->Init.lpm_enable == 1U)
  {
    (void)HAL_PCDEx_ActivateLPM(hpcd);
  }

  return HAL_OK;
}

/**
  * @brief  DeInitializes the PCD peripheral.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd)
{
  /* Check the PCD handle allocation */
  if (hpcd == NULL)
  {
    return HAL_ERROR;
  }

  hpcd->State = HAL_PCD_STATE_BUSY;

  /* Stop Device */
  if (USB_StopDevice(hpcd->Instance) != HAL_OK)
  {
    return HAL_ERROR;
  }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  if (hpcd->MspDeInitCallback == NULL)
  {
    hpcd->MspDeInitCallback = HAL_PCD_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware */
  hpcd->MspDeInitCallback(hpcd);
#else
  /* DeInit the low level hardware: CLOCK, NVIC.*/
  HAL_PCD_MspDeInit(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

  hpcd->State = HAL_PCD_STATE_RESET;

  return HAL_OK;
}

/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_MspDeInit could be implemented in the user file
   */
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register a User USB PCD Callback
  *         To be used instead of the weak predefined callback
  * @param  hpcd USB PCD handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_PCD_SOF_CB_ID USB PCD SOF callback ID
  *          @arg @ref HAL_PCD_SETUPSTAGE_CB_ID USB PCD Setup callback ID
  *          @arg @ref HAL_PCD_RESET_CB_ID USB PCD Reset callback ID
  *          @arg @ref HAL_PCD_SUSPEND_CB_ID USB PCD Suspend callback ID
  *          @arg @ref HAL_PCD_RESUME_CB_ID USB PCD Resume callback ID
  *          @arg @ref HAL_PCD_CONNECT_CB_ID USB PCD Connect callback ID
  *          @arg @ref HAL_PCD_DISCONNECT_CB_ID USB PCD Disconnect callback ID
  *          @arg @ref HAL_PCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref HAL_PCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterCallback(PCD_HandleTypeDef *hpcd,
                                           HAL_PCD_CallbackIDTypeDef CallbackID,
                                           pPCD_CallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
    return HAL_ERROR;
  }
  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    switch (CallbackID)
    {
      case HAL_PCD_SOF_CB_ID :
        hpcd->SOFCallback = pCallback;
        break;

      case HAL_PCD_SETUPSTAGE_CB_ID :
        hpcd->SetupStageCallback = pCallback;
        break;

      case HAL_PCD_RESET_CB_ID :
        hpcd->ResetCallback = pCallback;
        break;

      case HAL_PCD_SUSPEND_CB_ID :
        hpcd->SuspendCallback = pCallback;
        break;

      case HAL_PCD_RESUME_CB_ID :
        hpcd->ResumeCallback = pCallback;
        break;

      case HAL_PCD_CONNECT_CB_ID :
        hpcd->ConnectCallback = pCallback;
        break;

      case HAL_PCD_DISCONNECT_CB_ID :
        hpcd->DisconnectCallback = pCallback;
        break;

      case HAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = pCallback;
        break;

      case HAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (hpcd->State == HAL_PCD_STATE_RESET)
  {
    switch (CallbackID)
    {
      case HAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = pCallback;
        break;

      case HAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);
  return status;
}

/**
  * @brief  Unregister an USB PCD Callback
  *         USB PCD callback is redirected to the weak predefined callback
  * @param  hpcd USB PCD handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_PCD_SOF_CB_ID USB PCD SOF callback ID
  *          @arg @ref HAL_PCD_SETUPSTAGE_CB_ID USB PCD Setup callback ID
  *          @arg @ref HAL_PCD_RESET_CB_ID USB PCD Reset callback ID
  *          @arg @ref HAL_PCD_SUSPEND_CB_ID USB PCD Suspend callback ID
  *          @arg @ref HAL_PCD_RESUME_CB_ID USB PCD Resume callback ID
  *          @arg @ref HAL_PCD_CONNECT_CB_ID USB PCD Connect callback ID
  *          @arg @ref HAL_PCD_DISCONNECT_CB_ID USB PCD Disconnect callback ID
  *          @arg @ref HAL_PCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref HAL_PCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterCallback(PCD_HandleTypeDef *hpcd, HAL_PCD_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  /* Setup Legacy weak Callbacks  */
  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    switch (CallbackID)
    {
      case HAL_PCD_SOF_CB_ID :
        hpcd->SOFCallback = HAL_PCD_SOFCallback;
        break;

      case HAL_PCD_SETUPSTAGE_CB_ID :
        hpcd->SetupStageCallback = HAL_PCD_SetupStageCallback;
        break;

      case HAL_PCD_RESET_CB_ID :
        hpcd->ResetCallback = HAL_PCD_ResetCallback;
        break;

      case HAL_PCD_SUSPEND_CB_ID :
        hpcd->SuspendCallback = HAL_PCD_SuspendCallback;
        break;

      case HAL_PCD_RESUME_CB_ID :
        hpcd->ResumeCallback = HAL_PCD_ResumeCallback;
        break;

      case HAL_PCD_CONNECT_CB_ID :
        hpcd->ConnectCallback = HAL_PCD_ConnectCallback;
        break;

      case HAL_PCD_DISCONNECT_CB_ID :
        hpcd->DisconnectCallback = HAL_PCD_DisconnectCallback;
        break;

      case HAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = HAL_PCD_MspInit;
        break;

      case HAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = HAL_PCD_MspDeInit;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (hpcd->State == HAL_PCD_STATE_RESET)
  {
    switch (CallbackID)
    {
      case HAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = HAL_PCD_MspInit;
        break;

      case HAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = HAL_PCD_MspDeInit;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);
  return status;
}

/**
  * @brief  Register USB PCD Data OUT Stage Callback
  *         To be used instead of the weak HAL_PCD_DataOutStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Data OUT Stage Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterDataOutStageCallback(PCD_HandleTypeDef *hpcd,
                                                       pPCD_DataOutStageCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->DataOutStageCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Data OUT Stage Callback
  *         USB PCD Data OUT Stage Callback is redirected to the weak HAL_PCD_DataOutStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterDataOutStageCallback(PCD_HandleTypeDef *hpcd)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->DataOutStageCallback = HAL_PCD_DataOutStageCallback; /* Legacy weak DataOutStageCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD Data IN Stage Callback
  *         To be used instead of the weak HAL_PCD_DataInStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Data IN Stage Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterDataInStageCallback(PCD_HandleTypeDef *hpcd,
                                                      pPCD_DataInStageCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->DataInStageCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Data IN Stage Callback
  *         USB PCD Data OUT Stage Callback is redirected to the weak HAL_PCD_DataInStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterDataInStageCallback(PCD_HandleTypeDef *hpcd)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->DataInStageCallback = HAL_PCD_DataInStageCallback; /* Legacy weak DataInStageCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD Iso OUT incomplete Callback
  *         To be used instead of the weak HAL_PCD_ISOOUTIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Iso OUT incomplete Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterIsoOutIncpltCallback(PCD_HandleTypeDef *hpcd,
                                                       pPCD_IsoOutIncpltCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->ISOOUTIncompleteCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Iso OUT incomplete Callback
  *         USB PCD Iso OUT incomplete Callback is redirected
  *         to the weak HAL_PCD_ISOOUTIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterIsoOutIncpltCallback(PCD_HandleTypeDef *hpcd)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->ISOOUTIncompleteCallback = HAL_PCD_ISOOUTIncompleteCallback; /* Legacy weak ISOOUTIncompleteCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD Iso IN incomplete Callback
  *         To be used instead of the weak HAL_PCD_ISOINIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Iso IN incomplete Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterIsoInIncpltCallback(PCD_HandleTypeDef *hpcd,
                                                      pPCD_IsoInIncpltCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->ISOINIncompleteCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Iso IN incomplete Callback
  *         USB PCD Iso IN incomplete Callback is redirected
  *         to the weak HAL_PCD_ISOINIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterIsoInIncpltCallback(PCD_HandleTypeDef *hpcd)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->ISOINIncompleteCallback = HAL_PCD_ISOINIncompleteCallback; /* Legacy weak ISOINIncompleteCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD BCD Callback
  *         To be used instead of the weak HAL_PCDEx_BCD_Callback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD BCD Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterBcdCallback(PCD_HandleTypeDef *hpcd, pPCD_BcdCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->BCDCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD BCD Callback
  *         USB BCD Callback is redirected to the weak HAL_PCDEx_BCD_Callback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterBcdCallback(PCD_HandleTypeDef *hpcd)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->BCDCallback = HAL_PCDEx_BCD_Callback; /* Legacy weak HAL_PCDEx_BCD_Callback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD LPM Callback
  *         To be used instead of the weak HAL_PCDEx_LPM_Callback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD LPM Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterLpmCallback(PCD_HandleTypeDef *hpcd, pPCD_LpmCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->LPMCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD LPM Callback
  *         USB LPM Callback is redirected to the weak HAL_PCDEx_LPM_Callback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterLpmCallback(PCD_HandleTypeDef *hpcd)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->LPMCallback = HAL_PCDEx_LPM_Callback; /* Legacy weak HAL_PCDEx_LPM_Callback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group2 Input and Output operation functions
  *  @brief   Data transfers functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the PCD data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Start the USB device
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);
  __HAL_PCD_ENABLE(hpcd);
  (void)USB_DevConnect(hpcd->Instance);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Stop the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);
  __HAL_PCD_DISABLE(hpcd);
  (void)USB_DevDisconnect(hpcd->Instance);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}


/**
  * @brief  This function handles PCD interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd)
{
  uint32_t wIstr = USB_ReadInterrupts(hpcd->Instance);

  if ((wIstr & USB_ISTR_CTR) == USB_ISTR_CTR)
  {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    (void)PCD_EP_ISR_Handler(hpcd);

    return;
  }

  if ((wIstr & USB_ISTR_RESET) == USB_ISTR_RESET)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_RESET);

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->ResetCallback(hpcd);
#else
    HAL_PCD_ResetCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    (void)HAL_PCD_SetAddress(hpcd, 0U);

    return;
  }

  if ((wIstr & USB_ISTR_PMAOVR) == USB_ISTR_PMAOVR)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_PMAOVR);

    return;
  }

  if ((wIstr & USB_ISTR_ERR) == USB_ISTR_ERR)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_ERR);

    return;
  }

  if ((wIstr & USB_ISTR_WKUP) == USB_ISTR_WKUP)
  {
    hpcd->Instance->CNTR &= ~(USB_CNTR_SUSPRDY);
    hpcd->Instance->CNTR &= ~(USB_CNTR_SUSPEN);

    if (hpcd->LPM_State == LPM_L1)
    {
      hpcd->LPM_State = LPM_L0;
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->LPMCallback(hpcd, PCD_LPM_L0_ACTIVE);
#else
      HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L0_ACTIVE);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->ResumeCallback(hpcd);
#else
    HAL_PCD_ResumeCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_WKUP);

    return;
  }

  if ((wIstr & USB_ISTR_SUSP) == USB_ISTR_SUSP)
  {
    /* Force low-power mode in the macrocell */
    hpcd->Instance->CNTR |= USB_CNTR_SUSPEN;

    /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_SUSP);

    hpcd->Instance->CNTR |= USB_CNTR_SUSPRDY;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->SuspendCallback(hpcd);
#else
    HAL_PCD_SuspendCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    return;
  }

  /* Handle LPM Interrupt */
  if ((wIstr & USB_ISTR_L1REQ) == USB_ISTR_L1REQ)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_L1REQ);
    if (hpcd->LPM_State == LPM_L0)
    {
      /* Force suspend and low-power mode before going to L1 state*/
      hpcd->Instance->CNTR |= USB_CNTR_SUSPRDY;
      hpcd->Instance->CNTR |= USB_CNTR_SUSPEN;

      hpcd->LPM_State = LPM_L1;
      hpcd->BESL = ((uint32_t)hpcd->Instance->LPMCSR & USB_LPMCSR_BESL) >> 2;
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->LPMCallback(hpcd, PCD_LPM_L1_ACTIVE);
#else
      HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L1_ACTIVE);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }
    else
    {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->SuspendCallback(hpcd);
#else
      HAL_PCD_SuspendCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }

    return;
  }

  if ((wIstr & USB_ISTR_SOF) == USB_ISTR_SOF)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_SOF);

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->SOFCallback(hpcd);
#else
    HAL_PCD_SOFCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    return;
  }

  if ((wIstr & USB_ISTR_ESOF) == USB_ISTR_ESOF)
  {
    /* clear ESOF flag in ISTR */
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_ESOF);

    return;
  }
}


/**
  * @brief  Data OUT stage callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */
}

/**
  * @brief  Data IN stage callback
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_DataInStageCallback could be implemented in the user file
   */
}
/**
  * @brief  Setup stage callback
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_SetupStageCallback could be implemented in the user file
   */
}

/**
  * @brief  USB Start Of Frame callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_SOFCallback could be implemented in the user file
   */
}

/**
  * @brief  USB Reset callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ResetCallback could be implemented in the user file
   */
}

/**
  * @brief  Suspend event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_SuspendCallback could be implemented in the user file
   */
}

/**
  * @brief  Resume event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ResumeCallback could be implemented in the user file
   */
}

/**
  * @brief  Incomplete ISO OUT callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ISOOUTIncompleteCallback could be implemented in the user file
   */
}

/**
  * @brief  Incomplete ISO IN callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ISOINIncompleteCallback could be implemented in the user file
   */
}

/**
  * @brief  Connection event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ConnectCallback could be implemented in the user file
   */
}

/**
  * @brief  Disconnection event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_DisconnectCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group3 Peripheral Control functions
  *  @brief   management functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the PCD data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Connect the USB device
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);
  (void)USB_DevConnect(hpcd->Instance);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Disconnect the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);
  (void)USB_DevDisconnect(hpcd->Instance);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Set the USB Device address.
  * @param  hpcd PCD handle
  * @param  address new device address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address)
{
  __HAL_LOCK(hpcd);
  hpcd->USB_Address = address;
  (void)USB_SetDevAddress(hpcd->Instance, address);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}
/**
  * @brief  Open and configure an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  ep_mps endpoint max packet size
  * @param  ep_type endpoint type
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr,
                                  uint16_t ep_mps, uint8_t ep_type)
{
  HAL_StatusTypeDef  ret = HAL_OK;
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->num = ep_addr & EP_ADDR_MSK;
  ep->maxpacket = ep_mps;
  ep->type = ep_type;

  /* Set initial data PID. */
  if (ep_type == EP_TYPE_BULK)
  {
    ep->data_pid_start = 0U;
  }

  __HAL_LOCK(hpcd);
  (void)USB_ActivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return ret;
}

/**
  * @brief  Deactivate an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_DeactivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);
  return HAL_OK;
}


/**
  * @brief  Receive an amount of data.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the reception buffer
  * @param  len amount of data to be received
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  (void)USB_EPStartXfer(hpcd->Instance, ep);

  return HAL_OK;
}

/**
  * @brief  Get Received Data Size
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval Data Size
  */
uint32_t HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef const *hpcd, uint8_t ep_addr)
{
  return hpcd->OUT_ep[ep_addr & EP_ADDR_MSK].xfer_count;
}
/**
  * @brief  Send an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the transmission buffer
  * @param  len amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_fill_db = 1U;
  ep->xfer_len_db = len;
  ep->xfer_count = 0U;
  ep->is_in = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  (void)USB_EPStartXfer(hpcd->Instance, ep);

  return HAL_OK;
}

/**
  * @brief  Set a STALL condition over an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & EP_ADDR_MSK) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr];
    ep->is_in = 0U;
  }

  ep->is_stall = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);

  (void)USB_EPSetStall(hpcd->Instance, ep);

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Clear a STALL condition over in an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & 0x0FU) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->is_stall = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_EPClearStall(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
   * @brief  Abort an USB EP transaction.
   * @param  hpcd PCD handle
   * @param  ep_addr endpoint address
   * @retval HAL status
   */
HAL_StatusTypeDef HAL_PCD_EP_Abort(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  HAL_StatusTypeDef ret;
  PCD_EPTypeDef *ep;

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
  }

  /* Stop Xfer */
  ret = USB_EPStopXfer(hpcd->Instance, ep);

  return ret;
}

/**
  * @brief  Flush an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  __HAL_LOCK(hpcd);

  if ((ep_addr & 0x80U) == 0x80U)
  {
    (void)USB_FlushTxFifo(hpcd->Instance, (uint32_t)ep_addr & EP_ADDR_MSK);
  }
  else
  {
    (void)USB_FlushRxFifo(hpcd->Instance);
  }

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Activate remote wakeup signalling
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
  return (USB_ActivateRemoteWakeup(hpcd->Instance));
}

/**
  * @brief  De-activate remote wakeup signalling.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
  return (USB_DeActivateRemoteWakeup(hpcd->Instance));
}

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group4 Peripheral State functions
  *  @brief   Peripheral State functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the PCD handle state.
  * @param  hpcd PCD handle
  * @retval HAL state
  */
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef const *hpcd)
{
  return hpcd->State;
}

/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @addtogroup PCD_Private_Functions
  * @{
  */


/**
  * @brief  This function handles PCD Endpoint interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd)
{
  PCD_EPTypeDef *ep;
  uint16_t count;
  uint16_t wIstr;
  uint16_t wEPVal;
  uint16_t TxPctSize;
  uint8_t epindex;

#if (USE_USB_DOUBLE_BUFFER != 1U)
  count = 0U;
#endif /* USE_USB_DOUBLE_BUFFER */

  /* stay in loop while pending interrupts */
  while ((hpcd->Instance->ISTR & USB_ISTR_CTR) != 0U)
  {
    wIstr = (uint16_t)hpcd->Instance->ISTR;

    /* extract highest priority endpoint number */
    epindex = (uint8_t)(wIstr & USB_ISTR_IDN);

    if (epindex == 0U)
    {
      /* Decode and service control endpoint interrupt */

      /* DIR bit = origin of the interrupt */
      if ((wIstr & USB_ISTR_DIR) == 0U)
      {
        /* DIR = 0 */

        /* DIR = 0 => IN  int */
        /* DIR = 0 implies that (EP_CTR_TX = 1) always */
        PCD_CLEAR_TX_EP_CTR(hpcd->Instance, PCD_ENDP0);
        ep = &hpcd->IN_ep[0];

        ep->xfer_count = PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);
        ep->xfer_buff += ep->xfer_count;

        /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->DataInStageCallback(hpcd, 0U);
#else
        HAL_PCD_DataInStageCallback(hpcd, 0U);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

        if ((hpcd->USB_Address > 0U) && (ep->xfer_len == 0U))
        {
          hpcd->Instance->DADDR = ((uint16_t)hpcd->USB_Address | USB_DADDR_EF);
          hpcd->USB_Address = 0U;
        }
      }
      else
      {
        /* DIR = 1 */

        /* DIR = 1 & CTR_RX => SETUP or OUT int */
        /* DIR = 1 & (CTR_TX | CTR_RX) => 2 int pending */
        ep = &hpcd->OUT_ep[0];
        wEPVal = (uint16_t)PCD_GET_ENDPOINT(hpcd->Instance, PCD_ENDP0);

        if ((wEPVal & USB_EP_SETUP) != 0U)
        {
          /* Get SETUP Packet */
          ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

          USB_ReadPMA(hpcd->Instance, (uint8_t *)hpcd->Setup,
                      ep->pmaadress, (uint16_t)ep->xfer_count);

          /* SETUP bit kept frozen while CTR_RX = 1 */
          PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);

          /* Process SETUP Packet*/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->SetupStageCallback(hpcd);
#else
          HAL_PCD_SetupStageCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else if ((wEPVal & USB_EP_VTRX) != 0U)
        {
          PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);

          /* Get Control Data OUT Packet */
          ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

          if ((ep->xfer_count != 0U) && (ep->xfer_buff != 0U))
          {
            USB_ReadPMA(hpcd->Instance, ep->xfer_buff,
                        ep->pmaadress, (uint16_t)ep->xfer_count);

            ep->xfer_buff += ep->xfer_count;

            /* Process Control Data OUT Packet */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
            hpcd->DataOutStageCallback(hpcd, 0U);
#else
            HAL_PCD_DataOutStageCallback(hpcd, 0U);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
          }

          wEPVal = (uint16_t)PCD_GET_ENDPOINT(hpcd->Instance, PCD_ENDP0);

          if (((wEPVal & USB_EP_SETUP) == 0U) && ((wEPVal & USB_EP_RX_STRX) != USB_EP_RX_VALID))
          {
            PCD_SET_EP_RX_CNT(hpcd->Instance, PCD_ENDP0, ep->maxpacket);
            PCD_SET_EP_RX_STATUS(hpcd->Instance, PCD_ENDP0, USB_EP_RX_VALID);
          }
        }
      }
    }
    else
    {
      /* Decode and service non control endpoints interrupt */
      /* process related endpoint register */
      wEPVal = (uint16_t)PCD_GET_ENDPOINT(hpcd->Instance, epindex);

      if ((wEPVal & USB_EP_VTRX) != 0U)
      {
        /* clear int flag */
        PCD_CLEAR_RX_EP_CTR(hpcd->Instance, epindex);
        ep = &hpcd->OUT_ep[epindex];

        /* OUT Single Buffering */
        if (ep->doublebuffer == 0U)
        {
          count = (uint16_t)PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

          if (count != 0U)
          {
            USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaadress, count);
          }
        }
#if (USE_USB_DOUBLE_BUFFER == 1U)
        else
        {
          /* manage double buffer bulk out */
          if (ep->type == EP_TYPE_BULK)
          {
            count = HAL_PCD_EP_DB_Receive(hpcd, ep, wEPVal);
          }
          else /* manage double buffer iso out */
          {
            /* free EP OUT Buffer */
            PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 0U);

            if ((PCD_GET_ENDPOINT(hpcd->Instance, ep->num) & USB_EP_DTOG_RX) != 0U)
            {
              /* read from endpoint BUF0Addr buffer */
              count = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

              if (count != 0U)
              {
                USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr0, count);
              }
            }
            else
            {
              /* read from endpoint BUF1Addr buffer */
              count = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

              if (count != 0U)
              {
                USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr1, count);
              }
            }
          }
        }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

        /* multi-packet on the NON control OUT endpoint */
        ep->xfer_count += count;
        ep->xfer_buff += count;

        if ((ep->xfer_len == 0U) || (count < ep->maxpacket))
        {
          /* RX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->DataOutStageCallback(hpcd, ep->num);
#else
          HAL_PCD_DataOutStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else
        {
          (void)USB_EPStartXfer(hpcd->Instance, ep);
        }
      }

      if ((wEPVal & USB_EP_VTTX) != 0U)
      {
        ep = &hpcd->IN_ep[epindex];

        /* clear int flag */
        PCD_CLEAR_TX_EP_CTR(hpcd->Instance, epindex);

        if (ep->type == EP_TYPE_ISOC)
        {
          ep->xfer_len = 0U;

#if (USE_USB_DOUBLE_BUFFER == 1U)
          if (ep->doublebuffer != 0U)
          {
            if ((wEPVal & USB_EP_DTOG_TX) != 0U)
            {
              PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
            }
            else
            {
              PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
            }
          }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

          /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->DataInStageCallback(hpcd, ep->num);
#else
          HAL_PCD_DataInStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else
        {
          /* Manage Single Buffer Transaction */
          if ((wEPVal & USB_EP_KIND) == 0U)
          {
            /* multi-packet on the NON control IN endpoint */
            TxPctSize = (uint16_t)PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);

            if (ep->xfer_len > TxPctSize)
            {
              ep->xfer_len -= TxPctSize;
            }
            else
            {
              ep->xfer_len = 0U;
            }

            /* Zero Length Packet? */
            if (ep->xfer_len == 0U)
            {
              /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
              hpcd->DataInStageCallback(hpcd, ep->num);
#else
              HAL_PCD_DataInStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
            }
            else
            {
              /* Transfer is not yet Done */
              ep->xfer_buff += TxPctSize;
              ep->xfer_count += TxPctSize;
              (void)USB_EPStartXfer(hpcd->Instance, ep);
            }
          }
#if (USE_USB_DOUBLE_BUFFER == 1U)
          /* Double Buffer bulk IN (bulk transfer Len > Ep_Mps) */
          else
          {
            (void)HAL_PCD_EP_DB_Transmit(hpcd, ep, wEPVal);
          }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */
        }
      }
    }
  }

  return HAL_OK;
}


#if (USE_USB_DOUBLE_BUFFER == 1U)
/**
  * @brief  Manage double buffer bulk out transaction from ISR
  * @param  hpcd PCD handle
  * @param  ep current endpoint handle
  * @param  wEPVal Last snapshot of EPRx register value taken in ISR
  * @retval HAL status
  */
static uint16_t HAL_PCD_EP_DB_Receive(PCD_HandleTypeDef *hpcd,
                                      PCD_EPTypeDef *ep, uint16_t wEPVal)
{
  uint16_t count;

  /* Manage Buffer0 OUT */
  if ((wEPVal & USB_EP_DTOG_RX) != 0U)
  {
    /* Get count of received Data on buffer0 */
    count = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len >= count)
    {
      ep->xfer_len -= count;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    if (ep->xfer_len == 0U)
    {
      /* set NAK to OUT endpoint since double buffer is enabled */
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_NAK);
    }

    /* Check if Buffer1 is in blocked state which requires to toggle */
    if ((wEPVal & USB_EP_DTOG_TX) != 0U)
    {
      PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 0U);
    }

    if (count != 0U)
    {
      USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr0, count);
    }
  }
  /* Manage Buffer 1 DTOG_RX=0 */
  else
  {
    /* Get count of received data */
    count = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len >= count)
    {
      ep->xfer_len -= count;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    if (ep->xfer_len == 0U)
    {
      /* set NAK on the current endpoint */
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_NAK);
    }

    /*Need to FreeUser Buffer*/
    if ((wEPVal & USB_EP_DTOG_TX) == 0U)
    {
      PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 0U);
    }

    if (count != 0U)
    {
      USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr1, count);
    }
  }

  return count;
}


/**
  * @brief  Manage double buffer bulk IN transaction from ISR
  * @param  hpcd PCD handle
  * @param  ep current endpoint handle
  * @param  wEPVal Last snapshot of EPRx register value taken in ISR
  * @retval HAL status
  */
static HAL_StatusTypeDef HAL_PCD_EP_DB_Transmit(PCD_HandleTypeDef *hpcd,
                                                PCD_EPTypeDef *ep, uint16_t wEPVal)
{
  uint32_t len;
  uint16_t TxPctSize;

  /* Data Buffer0 ACK received */
  if ((wEPVal & USB_EP_DTOG_TX) != 0U)
  {
    /* multi-packet on the NON control IN endpoint */
    TxPctSize = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len > TxPctSize)
    {
      ep->xfer_len -= TxPctSize;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    /* Transfer is completed */
    if (ep->xfer_len == 0U)
    {
      PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
      PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);

      /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->DataInStageCallback(hpcd, ep->num);
#else
      HAL_PCD_DataInStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      if ((wEPVal & USB_EP_DTOG_RX) != 0U)
      {
        PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
      }
    }
    else /* Transfer is not yet Done */
    {
      /* need to Free USB Buff */
      if ((wEPVal & USB_EP_DTOG_RX) != 0U)
      {
        PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
      }

      /* Still there is data to Fill in the next Buffer */
      if (ep->xfer_fill_db == 1U)
      {
        ep->xfer_buff += TxPctSize;
        ep->xfer_count += TxPctSize;

        /* Calculate the len of the new buffer to fill */
        if (ep->xfer_len_db >= ep->maxpacket)
        {
          len = ep->maxpacket;
          ep->xfer_len_db -= len;
        }
        else if (ep->xfer_len_db == 0U)
        {
          len = TxPctSize;
          ep->xfer_fill_db = 0U;
        }
        else
        {
          ep->xfer_fill_db = 0U;
          len = ep->xfer_len_db;
          ep->xfer_len_db = 0U;
        }

        /* Write remaining Data to Buffer */
        /* Set the Double buffer counter for pma buffer1 */
        PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, len);

        /* Copy user buffer to USB PMA */
        USB_WritePMA(hpcd->Instance, ep->xfer_buff,  ep->pmaaddr0, (uint16_t)len);
      }
    }
  }
  else /* Data Buffer1 ACK received */
  {
    /* multi-packet on the NON control IN endpoint */
    TxPctSize = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len >= TxPctSize)
    {
      ep->xfer_len -= TxPctSize;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    /* Transfer is completed */
    if (ep->xfer_len == 0U)
    {
      PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
      PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);

      /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->DataInStageCallback(hpcd, ep->num);
#else
      HAL_PCD_DataInStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      /* need to Free USB Buff */
      if ((wEPVal & USB_EP_DTOG_RX) == 0U)
      {
        PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
      }
    }
    else /* Transfer is not yet Done */
    {
      /* need to Free USB Buff */
      if ((wEPVal & USB_EP_DTOG_RX) == 0U)
      {
        PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
      }

      /* Still there is data to Fill in the next Buffer */
      if (ep->xfer_fill_db == 1U)
      {
        ep->xfer_buff += TxPctSize;
        ep->xfer_count += TxPctSize;

        /* Calculate the len of the new buffer to fill */
        if (ep->xfer_len_db >= ep->maxpacket)
        {
          len = ep->maxpacket;
          ep->xfer_len_db -= len;
        }
        else if (ep->xfer_len_db == 0U)
        {
          len = TxPctSize;
          ep->xfer_fill_db = 0U;
        }
        else
        {
          len = ep->xfer_len_db;
          ep->xfer_len_db = 0U;
          ep->xfer_fill_db = 0;
        }

        /* Set the Double buffer counter for pmabuffer1 */
        PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, len);

        /* Copy the user buffer to USB PMA */
        USB_WritePMA(hpcd->Instance, ep->xfer_buff,  ep->pmaaddr1, (uint16_t)len);
      }
    }
  }

  /*enable endpoint IN*/
  PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_VALID);

  return HAL_OK;
}
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */



/**
  * @}
  */
#endif /* defined (USB_DRD_FS) */
#endif /* HAL_PCD_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */
