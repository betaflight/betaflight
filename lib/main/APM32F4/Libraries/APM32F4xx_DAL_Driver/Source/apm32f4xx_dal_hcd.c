/**
  *
  * @file    apm32f4xx_dal_hcd.c
  * @brief   HCD DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the USB Peripheral Controller:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  @verbatim
  ==============================================================================
                    ##### How to use this driver #####
  ==============================================================================
  [..]
    (#)Declare a HCD_HandleTypeDef handle structure, for example:
       HCD_HandleTypeDef  hhcd;

    (#)Fill parameters of Init structure in HCD handle

    (#)Call DAL_HCD_Init() API to initialize the HCD peripheral (Core, Host core, ...)

    (#)Initialize the HCD low level resources through the DAL_HCD_MspInit() API:
        (##) Enable the HCD/USB Low Level interface clock using the following macros
             (+++) __DAL_RCM_USB_OTG_FS_CLK_ENABLE();
             (+++) __DAL_RCM_USB_OTG_HS_CLK_ENABLE(); (For High Speed Mode)
             (+++) __DAL_RCM_USB_OTG_HS_ULPI_CLK_ENABLE(); (For High Speed Mode)

        (##) Initialize the related GPIO clocks
        (##) Configure HCD pin-out
        (##) Configure HCD NVIC interrupt

    (#)Associate the Upper USB Host stack to the DAL HCD Driver:
        (##) hhcd.pData = phost;

    (#)Enable HCD transmission and reception:
        (##) DAL_HCD_Start();

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

#ifdef DAL_HCD_MODULE_ENABLED
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)

/** @defgroup HCD HCD
  * @brief HCD DAL module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup HCD_Private_Functions HCD Private Functions
  * @{
  */
static void HCD_HC_IN_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t chnum);
static void HCD_HC_OUT_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t chnum);
static void HCD_RXQLVL_IRQHandler(HCD_HandleTypeDef *hhcd);
static void HCD_Port_IRQHandler(HCD_HandleTypeDef *hhcd);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup HCD_Exported_Functions HCD Exported Functions
  * @{
  */

/** @defgroup HCD_Exported_Functions_Group1 Initialization and de-initialization functions
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
  * @brief  Initialize the host driver.
  * @param  hhcd HCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_Init(HCD_HandleTypeDef *hhcd)
{
  USB_OTG_GlobalTypeDef *USBx;

  /* Check the HCD handle allocation */
  if (hhcd == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_HCD_ALL_INSTANCE(hhcd->Instance));

  USBx = hhcd->Instance;

  if (hhcd->State == DAL_HCD_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hhcd->Lock = DAL_UNLOCKED;

#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
    hhcd->SOFCallback = DAL_HCD_SOF_Callback;
    hhcd->ConnectCallback = DAL_HCD_Connect_Callback;
    hhcd->DisconnectCallback = DAL_HCD_Disconnect_Callback;
    hhcd->PortEnabledCallback = DAL_HCD_PortEnabled_Callback;
    hhcd->PortDisabledCallback = DAL_HCD_PortDisabled_Callback;
    hhcd->HC_NotifyURBChangeCallback = DAL_HCD_HC_NotifyURBChange_Callback;

    if (hhcd->MspInitCallback == NULL)
    {
      hhcd->MspInitCallback = DAL_HCD_MspInit;
    }

    /* Init the low level hardware */
    hhcd->MspInitCallback(hhcd);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    DAL_HCD_MspInit(hhcd);
#endif /* (USE_DAL_HCD_REGISTER_CALLBACKS) */
  }

  hhcd->State = DAL_HCD_STATE_BUSY;

  /* Disable DMA mode for FS instance */
  if ((USBx->GCID & (0x1U << 8)) == 0U)
  {
    hhcd->Init.dma_enable = 0U;
  }

  /* Disable the Interrupts */
  __DAL_HCD_DISABLE(hhcd);

  /* Init the Core (common init.) */
  (void)USB_CoreInit(hhcd->Instance, hhcd->Init);

  /* Force Host Mode*/
  (void)USB_SetCurrentMode(hhcd->Instance, USB_HOST_MODE);

  /* Init Host */
  (void)USB_HostInit(hhcd->Instance, hhcd->Init);

  hhcd->State = DAL_HCD_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Initialize a host channel.
  * @param  hhcd HCD handle
  * @param  ch_num Channel number.
  *         This parameter can be a value from 1 to 15
  * @param  epnum Endpoint number.
  *          This parameter can be a value from 1 to 15
  * @param  dev_address Current device address
  *          This parameter can be a value from 0 to 255
  * @param  speed Current device speed.
  *          This parameter can be one of these values:
  *            HCD_DEVICE_SPEED_HIGH: High speed mode,
  *            HCD_DEVICE_SPEED_FULL: Full speed mode,
  *            HCD_DEVICE_SPEED_LOW: Low speed mode
  * @param  ep_type Endpoint Type.
  *          This parameter can be one of these values:
  *            EP_TYPE_CTRL: Control type,
  *            EP_TYPE_ISOC: Isochronous type,
  *            EP_TYPE_BULK: Bulk type,
  *            EP_TYPE_INTR: Interrupt type
  * @param  mps Max Packet Size.
  *          This parameter can be a value from 0 to 32K
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_HC_Init(HCD_HandleTypeDef *hhcd,
                                  uint8_t ch_num,
                                  uint8_t epnum,
                                  uint8_t dev_address,
                                  uint8_t speed,
                                  uint8_t ep_type,
                                  uint16_t mps)
{
  DAL_StatusTypeDef status;

  __DAL_LOCK(hhcd);
  hhcd->hc[ch_num].do_ping = 0U;
  hhcd->hc[ch_num].dev_addr = dev_address;
  hhcd->hc[ch_num].max_packet = mps;
  hhcd->hc[ch_num].ch_num = ch_num;
  hhcd->hc[ch_num].ep_type = ep_type;
  hhcd->hc[ch_num].ep_num = epnum & 0x7FU;

  if ((epnum & 0x80U) == 0x80U)
  {
    hhcd->hc[ch_num].ep_is_in = 1U;
  }
  else
  {
    hhcd->hc[ch_num].ep_is_in = 0U;
  }

  hhcd->hc[ch_num].speed = speed;

  status =  USB_HC_Init(hhcd->Instance,
                        ch_num,
                        epnum,
                        dev_address,
                        speed,
                        ep_type,
                        mps);
  __DAL_UNLOCK(hhcd);

  return status;
}

/**
  * @brief  Halt a host channel.
  * @param  hhcd HCD handle
  * @param  ch_num Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_HC_Halt(HCD_HandleTypeDef *hhcd, uint8_t ch_num)
{
  DAL_StatusTypeDef status = DAL_OK;

  __DAL_LOCK(hhcd);
  (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
  __DAL_UNLOCK(hhcd);

  return status;
}

/**
  * @brief  DeInitialize the host driver.
  * @param  hhcd HCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_DeInit(HCD_HandleTypeDef *hhcd)
{
  /* Check the HCD handle allocation */
  if (hhcd == NULL)
  {
    return DAL_ERROR;
  }

  hhcd->State = DAL_HCD_STATE_BUSY;

#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
  if (hhcd->MspDeInitCallback == NULL)
  {
    hhcd->MspDeInitCallback = DAL_HCD_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware */
  hhcd->MspDeInitCallback(hhcd);
#else
  /* DeInit the low level hardware: CLOCK, NVIC.*/
  DAL_HCD_MspDeInit(hhcd);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */

  __DAL_HCD_DISABLE(hhcd);

  hhcd->State = DAL_HCD_STATE_RESET;

  return DAL_OK;
}

/**
  * @brief  Initialize the HCD MSP.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void  DAL_HCD_MspInit(HCD_HandleTypeDef *hhcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_HCD_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitialize the HCD MSP.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void  DAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_HCD_MspDeInit could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group2 Input and Output operation functions
  *  @brief   HCD IO operation functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..] This subsection provides a set of functions allowing to manage the USB Host Data
    Transfer

@endverbatim
  * @{
  */

/**
  * @brief  Submit a new URB for processing.
  * @param  hhcd HCD handle
  * @param  ch_num Channel number.
  *         This parameter can be a value from 1 to 15
  * @param  direction Channel number.
  *          This parameter can be one of these values:
  *           0 : Output / 1 : Input
  * @param  ep_type Endpoint Type.
  *          This parameter can be one of these values:
  *            EP_TYPE_CTRL: Control type/
  *            EP_TYPE_ISOC: Isochronous type/
  *            EP_TYPE_BULK: Bulk type/
  *            EP_TYPE_INTR: Interrupt type/
  * @param  token Endpoint Type.
  *          This parameter can be one of these values:
  *            0: HC_PID_SETUP / 1: HC_PID_DATA1
  * @param  pbuff pointer to URB data
  * @param  length Length of URB data
  * @param  do_ping activate do ping protocol (for high speed only).
  *          This parameter can be one of these values:
  *           0 : do ping inactive / 1 : do ping active
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_HC_SubmitRequest(HCD_HandleTypeDef *hhcd,
                                           uint8_t ch_num,
                                           uint8_t direction,
                                           uint8_t ep_type,
                                           uint8_t token,
                                           uint8_t *pbuff,
                                           uint16_t length,
                                           uint8_t do_ping)
{
  hhcd->hc[ch_num].ep_is_in = direction;
  hhcd->hc[ch_num].ep_type  = ep_type;

  if (token == 0U)
  {
    hhcd->hc[ch_num].data_pid = HC_PID_SETUP;
    hhcd->hc[ch_num].do_ping = do_ping;
  }
  else
  {
    hhcd->hc[ch_num].data_pid = HC_PID_DATA1;
  }

  /* Manage Data Toggle */
  switch (ep_type)
  {
    case EP_TYPE_CTRL:
      if ((token == 1U) && (direction == 0U)) /*send data */
      {
        if (length == 0U)
        {
          /* For Status OUT stage, Length==0, Status Out PID = 1 */
          hhcd->hc[ch_num].toggle_out = 1U;
        }

        /* Set the Data Toggle bit as per the Flag */
        if (hhcd->hc[ch_num].toggle_out == 0U)
        {
          /* Put the PID 0 */
          hhcd->hc[ch_num].data_pid = HC_PID_DATA0;
        }
        else
        {
          /* Put the PID 1 */
          hhcd->hc[ch_num].data_pid = HC_PID_DATA1;
        }
      }
      break;

    case EP_TYPE_BULK:
      if (direction == 0U)
      {
        /* Set the Data Toggle bit as per the Flag */
        if (hhcd->hc[ch_num].toggle_out == 0U)
        {
          /* Put the PID 0 */
          hhcd->hc[ch_num].data_pid = HC_PID_DATA0;
        }
        else
        {
          /* Put the PID 1 */
          hhcd->hc[ch_num].data_pid = HC_PID_DATA1;
        }
      }
      else
      {
        if (hhcd->hc[ch_num].toggle_in == 0U)
        {
          hhcd->hc[ch_num].data_pid = HC_PID_DATA0;
        }
        else
        {
          hhcd->hc[ch_num].data_pid = HC_PID_DATA1;
        }
      }

      break;
    case EP_TYPE_INTR:
      if (direction == 0U)
      {
        /* Set the Data Toggle bit as per the Flag */
        if (hhcd->hc[ch_num].toggle_out == 0U)
        {
          /* Put the PID 0 */
          hhcd->hc[ch_num].data_pid = HC_PID_DATA0;
        }
        else
        {
          /* Put the PID 1 */
          hhcd->hc[ch_num].data_pid = HC_PID_DATA1;
        }
      }
      else
      {
        if (hhcd->hc[ch_num].toggle_in == 0U)
        {
          hhcd->hc[ch_num].data_pid = HC_PID_DATA0;
        }
        else
        {
          hhcd->hc[ch_num].data_pid = HC_PID_DATA1;
        }
      }
      break;

    case EP_TYPE_ISOC:
      hhcd->hc[ch_num].data_pid = HC_PID_DATA0;
      break;

    default:
      break;
  }

  hhcd->hc[ch_num].xfer_buff = pbuff;
  hhcd->hc[ch_num].xfer_len  = length;
  hhcd->hc[ch_num].urb_state = URB_IDLE;
  hhcd->hc[ch_num].xfer_count = 0U;
  hhcd->hc[ch_num].ch_num = ch_num;
  hhcd->hc[ch_num].state = HC_IDLE;

  return USB_HC_StartXfer(hhcd->Instance, &hhcd->hc[ch_num], (uint8_t)hhcd->Init.dma_enable);
}

/**
  * @brief  Handle HCD interrupt request.
  * @param  hhcd HCD handle
  * @retval None
  */
void DAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd)
{
  USB_OTG_GlobalTypeDef *USBx = hhcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t i;
  uint32_t interrupt;

  /* Ensure that we are in device mode */
  if (USB_GetMode(hhcd->Instance) == USB_OTG_MODE_HOST)
  {
    /* Avoid spurious interrupt */
    if (__DAL_HCD_IS_INVALID_INTERRUPT(hhcd))
    {
      return;
    }

    if (__DAL_HCD_GET_FLAG(hhcd, USB_OTG_GCINT_IP_OUTTX))
    {
      /* Incorrect mode, acknowledge the interrupt */
      __DAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GCINT_IP_OUTTX);
    }

    if (__DAL_HCD_GET_FLAG(hhcd, USB_OTG_GCINT_IIINTX))
    {
      /* Incorrect mode, acknowledge the interrupt */
      __DAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GCINT_IIINTX);
    }

    if (__DAL_HCD_GET_FLAG(hhcd, USB_OTG_GCINT_PTXFE))
    {
      /* Incorrect mode, acknowledge the interrupt */
      __DAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GCINT_PTXFE);
    }

    if (__DAL_HCD_GET_FLAG(hhcd, USB_OTG_GCINT_MMIS))
    {
      /* Incorrect mode, acknowledge the interrupt */
      __DAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GCINT_MMIS);
    }

    /* Handle Host Disconnect Interrupts */
    if (__DAL_HCD_GET_FLAG(hhcd, USB_OTG_GCINT_DEDIS))
    {
      __DAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GCINT_DEDIS);

      if ((USBx_HPRT0 & USB_OTG_HPORTCSTS_PCNNTFLG) == 0U)
      {
        /* Flush USB Fifo */
        (void)USB_FlushTxFifo(USBx, 0x10U);
        (void)USB_FlushRxFifo(USBx);

        /* Restore FS Clock */
        (void)USB_InitFSLSPClkSel(hhcd->Instance, HCFG_48_MHZ);

        /* Handle Host Port Disconnect Interrupt */
#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
        hhcd->DisconnectCallback(hhcd);
#else
        DAL_HCD_Disconnect_Callback(hhcd);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */
      }
    }

    /* Handle Host Port Interrupts */
    if (__DAL_HCD_GET_FLAG(hhcd, USB_OTG_GCINT_HPORT))
    {
      HCD_Port_IRQHandler(hhcd);
    }

    /* Handle Host SOF Interrupt */
    if (__DAL_HCD_GET_FLAG(hhcd, USB_OTG_GCINT_SOF))
    {
#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
      hhcd->SOFCallback(hhcd);
#else
      DAL_HCD_SOF_Callback(hhcd);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */

      __DAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GCINT_SOF);
    }

    /* Handle Rx Queue Level Interrupts */
    if ((__DAL_HCD_GET_FLAG(hhcd, USB_OTG_GCINT_RXFNONE)) != 0U)
    {
      USB_MASK_INTERRUPT(hhcd->Instance, USB_OTG_GCINT_RXFNONE);

      HCD_RXQLVL_IRQHandler(hhcd);

      USB_UNMASK_INTERRUPT(hhcd->Instance, USB_OTG_GCINT_RXFNONE);
    }

    /* Handle Host channel Interrupt */
    if (__DAL_HCD_GET_FLAG(hhcd, USB_OTG_GCINT_HCHAN))
    {
      interrupt = USB_HC_ReadInterrupt(hhcd->Instance);
      for (i = 0U; i < hhcd->Init.Host_channels; i++)
      {
        if ((interrupt & (1UL << (i & 0xFU))) != 0U)
        {
          if ((USBx_HC(i)->HCH & USB_OTG_HCH_EDPDRT) == USB_OTG_HCH_EDPDRT)
          {
            HCD_HC_IN_IRQHandler(hhcd, (uint8_t)i);
          }
          else
          {
            HCD_HC_OUT_IRQHandler(hhcd, (uint8_t)i);
          }
        }
      }
      __DAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GCINT_HCHAN);
    }
  }
}


/**
  * @brief  Handles HCD Wakeup interrupt request.
  * @param  hhcd HCD handle
  * @retval DAL status
  */
void DAL_HCD_WKUP_IRQHandler(HCD_HandleTypeDef *hhcd)
{
  UNUSED(hhcd);
}


/**
  * @brief  SOF callback.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void DAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_HCD_SOF_Callback could be implemented in the user file
   */
}

/**
  * @brief Connection Event callback.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void DAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_HCD_Connect_Callback could be implemented in the user file
   */
}

/**
  * @brief  Disconnection Event callback.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void DAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_HCD_Disconnect_Callback could be implemented in the user file
   */
}

/**
  * @brief  Port Enabled  Event callback.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void DAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_HCD_Disconnect_Callback could be implemented in the user file
   */
}

/**
  * @brief  Port Disabled  Event callback.
  * @param  hhcd HCD handle
  * @retval None
  */
__weak void DAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_HCD_Disconnect_Callback could be implemented in the user file
   */
}

/**
  * @brief  Notify URB state change callback.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @param  urb_state:
  *          This parameter can be one of these values:
  *            URB_IDLE/
  *            URB_DONE/
  *            URB_NOTREADY/
  *            URB_NYET/
  *            URB_ERROR/
  *            URB_STALL/
  * @retval None
  */
__weak void DAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum, HCD_URBStateTypeDef urb_state)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhcd);
  UNUSED(chnum);
  UNUSED(urb_state);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_HCD_HC_NotifyURBChange_Callback could be implemented in the user file
   */
}

#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register a User USB HCD Callback
  *         To be used instead of the weak predefined callback
  * @param  hhcd USB HCD handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref DAL_HCD_SOF_CB_ID USB HCD SOF callback ID
  *          @arg @ref DAL_HCD_CONNECT_CB_ID USB HCD Connect callback ID
  *          @arg @ref DAL_HCD_DISCONNECT_CB_ID OTG HCD Disconnect callback ID
  *          @arg @ref DAL_HCD_PORT_ENABLED_CB_ID USB HCD Port Enable callback ID
  *          @arg @ref DAL_HCD_PORT_DISABLED_CB_ID USB HCD Port Disable callback ID
  *          @arg @ref DAL_HCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref DAL_HCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_RegisterCallback(HCD_HandleTypeDef *hhcd,
                                           DAL_HCD_CallbackIDTypeDef CallbackID,
                                           pHCD_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hhcd->ErrorCode |= DAL_HCD_ERROR_INVALID_CALLBACK;
    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hhcd);

  if (hhcd->State == DAL_HCD_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_HCD_SOF_CB_ID :
        hhcd->SOFCallback = pCallback;
        break;

      case DAL_HCD_CONNECT_CB_ID :
        hhcd->ConnectCallback = pCallback;
        break;

      case DAL_HCD_DISCONNECT_CB_ID :
        hhcd->DisconnectCallback = pCallback;
        break;

      case DAL_HCD_PORT_ENABLED_CB_ID :
        hhcd->PortEnabledCallback = pCallback;
        break;

      case DAL_HCD_PORT_DISABLED_CB_ID :
        hhcd->PortDisabledCallback = pCallback;
        break;

      case DAL_HCD_MSPINIT_CB_ID :
        hhcd->MspInitCallback = pCallback;
        break;

      case DAL_HCD_MSPDEINIT_CB_ID :
        hhcd->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hhcd->ErrorCode |= DAL_HCD_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hhcd->State == DAL_HCD_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_HCD_MSPINIT_CB_ID :
        hhcd->MspInitCallback = pCallback;
        break;

      case DAL_HCD_MSPDEINIT_CB_ID :
        hhcd->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hhcd->ErrorCode |= DAL_HCD_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hhcd->ErrorCode |= DAL_HCD_ERROR_INVALID_CALLBACK;
    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hhcd);
  return status;
}

/**
  * @brief  Unregister an USB HCD Callback
  *         USB HCD callback is redirected to the weak predefined callback
  * @param  hhcd USB HCD handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref DAL_HCD_SOF_CB_ID USB HCD SOF callback ID
  *          @arg @ref DAL_HCD_CONNECT_CB_ID USB HCD Connect callback ID
  *          @arg @ref DAL_HCD_DISCONNECT_CB_ID OTG HCD Disconnect callback ID
  *          @arg @ref DAL_HCD_PORT_ENABLED_CB_ID USB HCD Port Enabled callback ID
  *          @arg @ref DAL_HCD_PORT_DISABLED_CB_ID USB HCD Port Disabled callback ID
  *          @arg @ref DAL_HCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref DAL_HCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_UnRegisterCallback(HCD_HandleTypeDef *hhcd, DAL_HCD_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hhcd);

  /* Setup Legacy weak Callbacks  */
  if (hhcd->State == DAL_HCD_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_HCD_SOF_CB_ID :
        hhcd->SOFCallback = DAL_HCD_SOF_Callback;
        break;

      case DAL_HCD_CONNECT_CB_ID :
        hhcd->ConnectCallback = DAL_HCD_Connect_Callback;
        break;

      case DAL_HCD_DISCONNECT_CB_ID :
        hhcd->DisconnectCallback = DAL_HCD_Disconnect_Callback;
        break;

      case DAL_HCD_PORT_ENABLED_CB_ID :
        hhcd->PortEnabledCallback = DAL_HCD_PortEnabled_Callback;
        break;

      case DAL_HCD_PORT_DISABLED_CB_ID :
        hhcd->PortDisabledCallback = DAL_HCD_PortDisabled_Callback;
        break;

      case DAL_HCD_MSPINIT_CB_ID :
        hhcd->MspInitCallback = DAL_HCD_MspInit;
        break;

      case DAL_HCD_MSPDEINIT_CB_ID :
        hhcd->MspDeInitCallback = DAL_HCD_MspDeInit;
        break;

      default :
        /* Update the error code */
        hhcd->ErrorCode |= DAL_HCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hhcd->State == DAL_HCD_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_HCD_MSPINIT_CB_ID :
        hhcd->MspInitCallback = DAL_HCD_MspInit;
        break;

      case DAL_HCD_MSPDEINIT_CB_ID :
        hhcd->MspDeInitCallback = DAL_HCD_MspDeInit;
        break;

      default :
        /* Update the error code */
        hhcd->ErrorCode |= DAL_HCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hhcd->ErrorCode |= DAL_HCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hhcd);
  return status;
}

/**
  * @brief  Register USB HCD Host Channel Notify URB Change Callback
  *         To be used instead of the weak DAL_HCD_HC_NotifyURBChange_Callback() predefined callback
  * @param  hhcd HCD handle
  * @param  pCallback pointer to the USB HCD Host Channel Notify URB Change Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_RegisterHC_NotifyURBChangeCallback(HCD_HandleTypeDef *hhcd,
                                                             pHCD_HC_NotifyURBChangeCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hhcd->ErrorCode |= DAL_HCD_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hhcd);

  if (hhcd->State == DAL_HCD_STATE_READY)
  {
    hhcd->HC_NotifyURBChangeCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hhcd->ErrorCode |= DAL_HCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hhcd);

  return status;
}

/**
  * @brief  Unregister the USB HCD Host Channel Notify URB Change Callback
  *         USB HCD Host Channel Notify URB Change Callback is redirected
  *         to the weak DAL_HCD_HC_NotifyURBChange_Callback() predefined callback
  * @param  hhcd HCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_UnRegisterHC_NotifyURBChangeCallback(HCD_HandleTypeDef *hhcd)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hhcd);

  if (hhcd->State == DAL_HCD_STATE_READY)
  {
    hhcd->HC_NotifyURBChangeCallback = DAL_HCD_HC_NotifyURBChange_Callback; /* Legacy weak DataOutStageCallback  */
  }
  else
  {
    /* Update the error code */
    hhcd->ErrorCode |= DAL_HCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hhcd);

  return status;
}
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group3 Peripheral Control functions
  *  @brief   Management functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the HCD data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Start the host driver.
  * @param  hhcd HCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_Start(HCD_HandleTypeDef *hhcd)
{
  __DAL_LOCK(hhcd);
  /* Enable port power */
  (void)USB_DriveVbus(hhcd->Instance, 1U);

  /* Enable global interrupt */
  __DAL_HCD_ENABLE(hhcd);
  __DAL_UNLOCK(hhcd);

  return DAL_OK;
}

/**
  * @brief  Stop the host driver.
  * @param  hhcd HCD handle
  * @retval DAL status
  */

DAL_StatusTypeDef DAL_HCD_Stop(HCD_HandleTypeDef *hhcd)
{
  __DAL_LOCK(hhcd);
  (void)USB_StopHost(hhcd->Instance);
  __DAL_UNLOCK(hhcd);

  return DAL_OK;
}

/**
  * @brief  Reset the host port.
  * @param  hhcd HCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HCD_ResetPort(HCD_HandleTypeDef *hhcd)
{
  return (USB_ResetPort(hhcd->Instance));
}

/**
  * @}
  */

/** @defgroup HCD_Exported_Functions_Group4 Peripheral State functions
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
  * @brief  Return the HCD handle state.
  * @param  hhcd HCD handle
  * @retval DAL state
  */
HCD_StateTypeDef DAL_HCD_GetState(HCD_HandleTypeDef *hhcd)
{
  return hhcd->State;
}

/**
  * @brief  Return  URB state for a channel.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval URB state.
  *          This parameter can be one of these values:
  *            URB_IDLE/
  *            URB_DONE/
  *            URB_NOTREADY/
  *            URB_NYET/
  *            URB_ERROR/
  *            URB_STALL
  */
HCD_URBStateTypeDef DAL_HCD_HC_GetURBState(HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
  return hhcd->hc[chnum].urb_state;
}


/**
  * @brief  Return the last host transfer size.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval last transfer size in byte
  */
uint32_t DAL_HCD_HC_GetXferCount(HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
  return hhcd->hc[chnum].xfer_count;
}

/**
  * @brief  Return the Host Channel state.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval Host channel state
  *          This parameter can be one of these values:
  *            HC_IDLE/
  *            HC_XFRC/
  *            HC_DALTED/
  *            HC_NYET/
  *            HC_NAK/
  *            HC_STALL/
  *            HC_XACTERR/
  *            HC_BBLERR/
  *            HC_DATATGLERR
  */
HCD_HCStateTypeDef  DAL_HCD_HC_GetState(HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
  return hhcd->hc[chnum].state;
}

/**
  * @brief  Return the current Host frame number.
  * @param  hhcd HCD handle
  * @retval Current Host frame number
  */
uint32_t DAL_HCD_GetCurrentFrame(HCD_HandleTypeDef *hhcd)
{
  return (USB_GetCurrentFrame(hhcd->Instance));
}

/**
  * @brief  Return the Host enumeration speed.
  * @param  hhcd HCD handle
  * @retval Enumeration speed
  */
uint32_t DAL_HCD_GetCurrentSpeed(HCD_HandleTypeDef *hhcd)
{
  return (USB_GetHostSpeed(hhcd->Instance));
}

/**
 * @brief   USB host configure toggle of channel
 *  
 * @param   hhcd: hhcd HCD handle
 *  
 * @param   pipe: pipe number
 *  
 * @param   toggle: toggle
 *  
 * @retval  None
 */
void DAL_HCD_ConfigToggle(HCD_HandleTypeDef* hhcd, uint8_t pipe, uint8_t toggle)
{
    if (hhcd->hc[pipe].ep_is_in)
    {
        hhcd->hc[pipe].toggle_in = toggle;
    }
    else
    {
        hhcd->hc[pipe].toggle_out = toggle;
    }
}

/**
 * @brief   USB host read toggle of channel
 *  
 * @param   hhcd: hhcd HCD handle
 *  
 * @param   pipe: pipe number
 *  
 * @retval  toggle
 */
uint8_t DAL_HCD_ReadToggle(HCD_HandleTypeDef* hhcd, uint8_t pipe)
{
    uint8_t toggle;

    if (hhcd->hc[pipe].ep_is_in)
    {
        toggle = hhcd->hc[pipe].toggle_in;
    }
    else
    {
        toggle = hhcd->hc[pipe].toggle_out;
    }

    return toggle;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup HCD_Private_Functions
  * @{
  */
/**
  * @brief  Handle Host Channel IN interrupt requests.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval none
  */
static void HCD_HC_IN_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
  USB_OTG_GlobalTypeDef *USBx = hhcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t ch_num = (uint32_t)chnum;

  uint32_t tmpreg;

  if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_AHBERR) == USB_OTG_HCHINT_AHBERR)
  {
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_AHBERR);
    hhcd->hc[ch_num].state = HC_XACTERR;
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_BABBLE) == USB_OTG_HCHINT_BABBLE)
  {
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_BABBLE);
    hhcd->hc[ch_num].state = HC_BBLERR;
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_RXTXACK) == USB_OTG_HCHINT_RXTXACK)
  {
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_RXTXACK);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_RXSTALL) == USB_OTG_HCHINT_RXSTALL)
  {
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_RXSTALL);
    hhcd->hc[ch_num].state = HC_STALL;
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_DTOG) == USB_OTG_HCHINT_DTOG)
  {
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_DTOG);
    hhcd->hc[ch_num].state = HC_DATATGLERR;
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_TERR) == USB_OTG_HCHINT_TERR)
  {
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_TERR);
    hhcd->hc[ch_num].state = HC_XACTERR;
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
  }
  else
  {
    /* ... */
  }

  if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_FOVR) == USB_OTG_HCHINT_FOVR)
  {
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_FOVR);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_TSFCMPN) == USB_OTG_HCHINT_TSFCMPN)
  {
    if (hhcd->Init.dma_enable != 0U)
    {
      hhcd->hc[ch_num].xfer_count = hhcd->hc[ch_num].XferSize - \
                                    (USBx_HC(ch_num)->HCHTSIZE & USB_OTG_HCHTSIZE_TSFSIZE);
    }

    hhcd->hc[ch_num].state = HC_XFRC;
    hhcd->hc[ch_num].ErrCnt = 0U;
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_TSFCMPN);

    if ((hhcd->hc[ch_num].ep_type == EP_TYPE_CTRL) ||
        (hhcd->hc[ch_num].ep_type == EP_TYPE_BULK))
    {
      (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
      __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_RXNAK);
    }
    else if ((hhcd->hc[ch_num].ep_type == EP_TYPE_INTR) ||
             (hhcd->hc[ch_num].ep_type == EP_TYPE_ISOC))
    {
      USBx_HC(ch_num)->HCH |= USB_OTG_HCH_ODDF;
      hhcd->hc[ch_num].urb_state = URB_DONE;

#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
      hhcd->HC_NotifyURBChangeCallback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#else
      DAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */
    }
    else
    {
      /* ... */
    }

    if (hhcd->Init.dma_enable == 1U)
    {
      if (((hhcd->hc[ch_num].XferSize / hhcd->hc[ch_num].max_packet) & 1U) != 0U)
      {
        hhcd->hc[ch_num].toggle_in ^= 1U;
      }
    }
    else
    {
      hhcd->hc[ch_num].toggle_in ^= 1U;
    }
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_TSFCMPAN) == USB_OTG_HCHINT_TSFCMPAN)
  {
    if (hhcd->hc[ch_num].state == HC_XFRC)
    {
      hhcd->hc[ch_num].urb_state = URB_DONE;
    }
    else if (hhcd->hc[ch_num].state == HC_STALL)
    {
      hhcd->hc[ch_num].urb_state = URB_STALL;
    }
    else if ((hhcd->hc[ch_num].state == HC_XACTERR) ||
             (hhcd->hc[ch_num].state == HC_DATATGLERR))
    {
      hhcd->hc[ch_num].ErrCnt++;
      if (hhcd->hc[ch_num].ErrCnt > 2U)
      {
        hhcd->hc[ch_num].ErrCnt = 0U;
        hhcd->hc[ch_num].urb_state = URB_ERROR;
      }
      else
      {
        hhcd->hc[ch_num].urb_state = URB_NOTREADY;

        /* re-activate the channel */
        tmpreg = USBx_HC(ch_num)->HCH;
        tmpreg &= ~USB_OTG_HCH_CHINT;
        tmpreg |= USB_OTG_HCH_CHEN;
        USBx_HC(ch_num)->HCH = tmpreg;
      }
    }
    else if (hhcd->hc[ch_num].state == HC_NAK)
    {
      hhcd->hc[ch_num].urb_state  = URB_NOTREADY;

      /* re-activate the channel */
      tmpreg = USBx_HC(ch_num)->HCH;
      tmpreg &= ~USB_OTG_HCH_CHINT;
      tmpreg |= USB_OTG_HCH_CHEN;
      USBx_HC(ch_num)->HCH = tmpreg;
    }
    else if (hhcd->hc[ch_num].state == HC_BBLERR)
    {
      hhcd->hc[ch_num].ErrCnt++;
      hhcd->hc[ch_num].urb_state = URB_ERROR;
    }
    else
    {
      /* ... */
    }
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_TSFCMPAN);

#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
    hhcd->HC_NotifyURBChangeCallback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#else
    DAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_RXNAK) == USB_OTG_HCHINT_RXNAK)
  {
    if (hhcd->hc[ch_num].ep_type == EP_TYPE_INTR)
    {
      hhcd->hc[ch_num].ErrCnt = 0U;
      (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
    }
    else if ((hhcd->hc[ch_num].ep_type == EP_TYPE_CTRL) ||
             (hhcd->hc[ch_num].ep_type == EP_TYPE_BULK))
    {
      hhcd->hc[ch_num].ErrCnt = 0U;

      if (hhcd->Init.dma_enable == 0U)
      {
        hhcd->hc[ch_num].state = HC_NAK;
        (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
      }
    }
    else
    {
      /* ... */
    }
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_RXNAK);
  }
  else
  {
    /* ... */
  }
}

/**
  * @brief  Handle Host Channel OUT interrupt requests.
  * @param  hhcd HCD handle
  * @param  chnum Channel number.
  *         This parameter can be a value from 1 to 15
  * @retval none
  */
static void HCD_HC_OUT_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
  USB_OTG_GlobalTypeDef *USBx = hhcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t ch_num = (uint32_t)chnum;
  uint32_t tmpreg;
  uint32_t num_packets;

  if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_AHBERR) == USB_OTG_HCHINT_AHBERR)
  {
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_AHBERR);
    hhcd->hc[ch_num].state = HC_XACTERR;
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_RXTXACK) == USB_OTG_HCHINT_RXTXACK)
  {
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_RXTXACK);

    if (hhcd->hc[ch_num].do_ping == 1U)
    {
      hhcd->hc[ch_num].do_ping = 0U;
      hhcd->hc[ch_num].urb_state = URB_NOTREADY;
      (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
    }
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_FOVR) == USB_OTG_HCHINT_FOVR)
  {
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_FOVR);
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_TSFCMPN) == USB_OTG_HCHINT_TSFCMPN)
  {
    hhcd->hc[ch_num].ErrCnt = 0U;

    /* transaction completed with NYET state, update do ping state */
    if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_RXNYET) == USB_OTG_HCHINT_RXNYET)
    {
      hhcd->hc[ch_num].do_ping = 1U;
      __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_RXNYET);
    }
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_TSFCMPN);
    hhcd->hc[ch_num].state = HC_XFRC;
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_RXNYET) == USB_OTG_HCHINT_RXNYET)
  {
    hhcd->hc[ch_num].state = HC_NYET;
    hhcd->hc[ch_num].do_ping = 1U;
    hhcd->hc[ch_num].ErrCnt = 0U;
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_RXNYET);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_RXSTALL) == USB_OTG_HCHINT_RXSTALL)
  {
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_RXSTALL);
    hhcd->hc[ch_num].state = HC_STALL;
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_RXNAK) == USB_OTG_HCHINT_RXNAK)
  {
    hhcd->hc[ch_num].ErrCnt = 0U;
    hhcd->hc[ch_num].state = HC_NAK;

    if (hhcd->hc[ch_num].do_ping == 0U)
    {
      if (hhcd->hc[ch_num].speed == HCD_DEVICE_SPEED_HIGH)
      {
        hhcd->hc[ch_num].do_ping = 1U;
      }
    }

    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_RXNAK);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_TERR) == USB_OTG_HCHINT_TERR)
  {
    if (hhcd->Init.dma_enable == 0U)
    {
      hhcd->hc[ch_num].state = HC_XACTERR;
      (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
    }
    else
    {
      hhcd->hc[ch_num].ErrCnt++;
      if (hhcd->hc[ch_num].ErrCnt > 2U)
      {
        hhcd->hc[ch_num].ErrCnt = 0U;
        hhcd->hc[ch_num].urb_state = URB_ERROR;

#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
        hhcd->HC_NotifyURBChangeCallback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#else
        DAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */
      }
      else
      {
        hhcd->hc[ch_num].urb_state = URB_NOTREADY;
      }
    }
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_TERR);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_DTOG) == USB_OTG_HCHINT_DTOG)
  {
    hhcd->hc[ch_num].state = HC_DATATGLERR;
    (void)USB_HC_Halt(hhcd->Instance, (uint8_t)ch_num);
    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_DTOG);
  }
  else if ((USBx_HC(ch_num)->HCHINT & USB_OTG_HCHINT_TSFCMPAN) == USB_OTG_HCHINT_TSFCMPAN)
  {
    if (hhcd->hc[ch_num].state == HC_XFRC)
    {
      hhcd->hc[ch_num].urb_state  = URB_DONE;
      if ((hhcd->hc[ch_num].ep_type == EP_TYPE_BULK) ||
          (hhcd->hc[ch_num].ep_type == EP_TYPE_INTR))
      {
        if (hhcd->Init.dma_enable == 0U)
        {
          hhcd->hc[ch_num].toggle_out ^= 1U;
        }

        if ((hhcd->Init.dma_enable == 1U) && (hhcd->hc[ch_num].xfer_len > 0U))
        {
          num_packets = (hhcd->hc[ch_num].xfer_len + hhcd->hc[ch_num].max_packet - 1U) / hhcd->hc[ch_num].max_packet;

          if ((num_packets & 1U) != 0U)
          {
            hhcd->hc[ch_num].toggle_out ^= 1U;
          }
        }
      }
    }
    else if (hhcd->hc[ch_num].state == HC_NAK)
    {
      hhcd->hc[ch_num].urb_state = URB_NOTREADY;
    }
    else if (hhcd->hc[ch_num].state == HC_NYET)
    {
      hhcd->hc[ch_num].urb_state  = URB_NOTREADY;
    }
    else if (hhcd->hc[ch_num].state == HC_STALL)
    {
      hhcd->hc[ch_num].urb_state  = URB_STALL;
    }
    else if ((hhcd->hc[ch_num].state == HC_XACTERR) ||
             (hhcd->hc[ch_num].state == HC_DATATGLERR))
    {
      hhcd->hc[ch_num].ErrCnt++;
      if (hhcd->hc[ch_num].ErrCnt > 2U)
      {
        hhcd->hc[ch_num].ErrCnt = 0U;
        hhcd->hc[ch_num].urb_state = URB_ERROR;
      }
      else
      {
        hhcd->hc[ch_num].urb_state = URB_NOTREADY;

        /* re-activate the channel  */
        tmpreg = USBx_HC(ch_num)->HCH;
        tmpreg &= ~USB_OTG_HCH_CHINT;
        tmpreg |= USB_OTG_HCH_CHEN;
        USBx_HC(ch_num)->HCH = tmpreg;
      }
    }
    else
    {
      /* ... */
    }

    __DAL_HCD_CLEAR_HC_INT(ch_num, USB_OTG_HCHINT_TSFCMPAN);

#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
    hhcd->HC_NotifyURBChangeCallback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#else
    DAL_HCD_HC_NotifyURBChange_Callback(hhcd, (uint8_t)ch_num, hhcd->hc[ch_num].urb_state);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */
  }
  else
  {
    /* ... */
  }
}

/**
  * @brief  Handle Rx Queue Level interrupt requests.
  * @param  hhcd HCD handle
  * @retval none
  */
static void HCD_RXQLVL_IRQHandler(HCD_HandleTypeDef *hhcd)
{
  USB_OTG_GlobalTypeDef *USBx = hhcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t pktsts;
  uint32_t pktcnt;
  uint32_t GrxstspReg;
  uint32_t xferSizePktCnt;
  uint32_t tmpreg;
  uint32_t ch_num;

  GrxstspReg = hhcd->Instance->GRXSTSP;
  ch_num = GrxstspReg & USB_OTG_GRXSTSP_EPNUM;
  pktsts = (GrxstspReg & USB_OTG_GRXSTSP_PSTS) >> 17;
  pktcnt = (GrxstspReg & USB_OTG_GRXSTSP_BCNT) >> 4;

  switch (pktsts)
  {
    case GRXSTS_PKTSTS_IN:
      /* Read the data into the host buffer. */
      if ((pktcnt > 0U) && (hhcd->hc[ch_num].xfer_buff != (void *)0))
      {
        if ((hhcd->hc[ch_num].xfer_count + pktcnt) <= hhcd->hc[ch_num].xfer_len)
        {
          (void)USB_ReadPacket(hhcd->Instance,
                               hhcd->hc[ch_num].xfer_buff, (uint16_t)pktcnt);

          /* manage multiple Xfer */
          hhcd->hc[ch_num].xfer_buff += pktcnt;
          hhcd->hc[ch_num].xfer_count += pktcnt;

          /* get transfer size packet count */
          xferSizePktCnt = (USBx_HC(ch_num)->HCHTSIZE & USB_OTG_HCHTSIZE_PCKTCNT) >> 19;

          if ((hhcd->hc[ch_num].max_packet == pktcnt) && (xferSizePktCnt > 0U))
          {
            /* re-activate the channel when more packets are expected */
            tmpreg = USBx_HC(ch_num)->HCH;
            tmpreg &= ~USB_OTG_HCH_CHINT;
            tmpreg |= USB_OTG_HCH_CHEN;
            USBx_HC(ch_num)->HCH = tmpreg;
            hhcd->hc[ch_num].toggle_in ^= 1U;
          }
        }
        else
        {
          hhcd->hc[ch_num].urb_state = URB_ERROR;
        }
      }
      break;

    case GRXSTS_PKTSTS_DATA_TOGGLE_ERR:
      break;

    case GRXSTS_PKTSTS_IN_XFER_COMP:
    case GRXSTS_PKTSTS_CH_HALTED:
    default:
      break;
  }
}

/**
  * @brief  Handle Host Port interrupt requests.
  * @param  hhcd HCD handle
  * @retval None
  */
static void HCD_Port_IRQHandler(HCD_HandleTypeDef *hhcd)
{
  USB_OTG_GlobalTypeDef *USBx = hhcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  __IO uint32_t hprt0;
  __IO uint32_t hprt0_dup;

  /* Handle Host Port Interrupts */
  hprt0 = USBx_HPRT0;
  hprt0_dup = USBx_HPRT0;

  hprt0_dup &= ~(USB_OTG_HPORTCSTS_PEN | USB_OTG_HPORTCSTS_PCINTFLG | \
                 USB_OTG_HPORTCSTS_PENCHG | USB_OTG_HPORTCSTS_POVCCHG);

  /* Check whether Port Connect detected */
  if ((hprt0 & USB_OTG_HPORTCSTS_PCINTFLG) == USB_OTG_HPORTCSTS_PCINTFLG)
  {
    if ((hprt0 & USB_OTG_HPORTCSTS_PCNNTFLG) == USB_OTG_HPORTCSTS_PCNNTFLG)
    {
#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
      hhcd->ConnectCallback(hhcd);
#else
      DAL_HCD_Connect_Callback(hhcd);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */
    }
    hprt0_dup |= USB_OTG_HPORTCSTS_PCINTFLG;
  }

  /* Check whether Port Enable Changed */
  if ((hprt0 & USB_OTG_HPORTCSTS_PENCHG) == USB_OTG_HPORTCSTS_PENCHG)
  {
    hprt0_dup |= USB_OTG_HPORTCSTS_PENCHG;

    if ((hprt0 & USB_OTG_HPORTCSTS_PEN) == USB_OTG_HPORTCSTS_PEN)
    {
      if (hhcd->Init.phy_itface  == USB_OTG_EMBEDDED_PHY)
      {
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx)
        if (hhcd->Init.speed == USBH_HS_SPEED)
        {
          if ((hprt0 & USB_OTG_HPORTCSTS_PSPDSEL) == (HPRT0_PRTSPD_LOW_SPEED << 17))
          {
            (void)USB_InitFSLSPClkSel(hhcd->Instance, HCFG_6_MHZ);
          }
          else if ((hprt0 & USB_OTG_HPORTCSTS_PSPDSEL) == (HPRT0_PRTSPD_FULL_SPEED << 17))
          {
            (void)USB_InitFSLSPClkSel(hhcd->Instance, HCFG_48_MHZ);
          }
          else
          {
            (void)USB_InitFSLSPClkSel(hhcd->Instance, HCFG_30_60_MHZ);
          }
        }
        else
        {
          if ((hprt0 & USB_OTG_HPORTCSTS_PSPDSEL) == (HPRT0_PRTSPD_LOW_SPEED << 17))
          {
            (void)USB_InitFSLSPClkSel(hhcd->Instance, HCFG_6_MHZ);
          }
          else
          {
            (void)USB_InitFSLSPClkSel(hhcd->Instance, HCFG_48_MHZ);
          }
        }
#else
        if ((hprt0 & USB_OTG_HPORTCSTS_PSPDSEL) == (HPRT0_PRTSPD_LOW_SPEED << 17))
        {
          (void)USB_InitFSLSPClkSel(hhcd->Instance, HCFG_6_MHZ);
        }
        else
        {
          (void)USB_InitFSLSPClkSel(hhcd->Instance, HCFG_48_MHZ);
        }
#endif /* APM32F405xx || APM32F407xx || APM32F417xx */
      }
      else
      {
        if (hhcd->Init.speed == HCD_SPEED_FULL)
        {
          USBx_HOST->HFIVL = 60000U;
        }
      }
#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
      hhcd->PortEnabledCallback(hhcd);
#else
      DAL_HCD_PortEnabled_Callback(hhcd);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */

    }
    else
    {
#if (USE_DAL_HCD_REGISTER_CALLBACKS == 1U)
      hhcd->PortDisabledCallback(hhcd);
#else
      DAL_HCD_PortDisabled_Callback(hhcd);
#endif /* USE_DAL_HCD_REGISTER_CALLBACKS */
    }
  }

  /* Check for an overcurrent */
  if ((hprt0 & USB_OTG_HPORTCSTS_POVCCHG) == USB_OTG_HPORTCSTS_POVCCHG)
  {
    hprt0_dup |= USB_OTG_HPORTCSTS_POVCCHG;
  }

  /* Clear Port Interrupts */
  USBx_HPRT0 = hprt0_dup;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */
#endif /* DAL_HCD_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */
