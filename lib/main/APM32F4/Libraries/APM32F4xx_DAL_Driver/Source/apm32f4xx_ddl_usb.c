/**
  *
  * @file    apm32f4xx_ddl_usb.c
  * @brief   USB Low Layer DDL module driver.
  *
  *          This file provides firmware functions to manage the following
  *          functionalities of the USB Peripheral Controller:
  *           + Initialization/de-initialization functions
  *           + I/O operation functions
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
      (#) Fill parameters of Init structure in USB_OTG_CfgTypeDef structure.

      (#) Call USB_CoreInit() API to initialize the USB Core peripheral.

      (#) The upper DAL HCD/PCD driver will call the right routines for its internal processes.

  @endverbatim

  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

#if defined (DAL_PCD_MODULE_ENABLED) || defined (DAL_HCD_MODULE_ENABLED)
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
static DAL_StatusTypeDef USB_CoreReset(USB_OTG_GlobalTypeDef *USBx);

/* Exported functions --------------------------------------------------------*/
/** @defgroup USB_DDL_Exported_Functions USB Low Layer Exported Functions
  * @{
  */

/** @defgroup USB_DDL_Exported_Functions_Group1 Initialization/de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
                      ##### Initialization/de-initialization functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the USB Core
  * @param  USBx USB Instance
  * @param  cfg pointer to a USB_OTG_CfgTypeDef structure that contains
  *         the configuration information for the specified USBx peripheral.
  * @retval DAL status
  */
DAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg)
{
  DAL_StatusTypeDef ret;
  if (cfg.phy_itface == USB_OTG_ULPI_PHY)
  {
    USBx->GGCCFG &= ~(USB_OTG_GGCCFG_PWEN);

    /* Init The ULPI Interface */
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_DPSEL | USB_OTG_GUSBCFG_ULPISEL | USB_OTG_GUSBCFG_FSSTSEL);

    /* Select vbus source */
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVDSEL | USB_OTG_GUSBCFG_ULPIEVC);
    if (cfg.use_external_vbus == 1U)
    {
      USBx->GUSBCFG |= USB_OTG_GUSBCFG_ULPIEVDSEL;
    }

    /* Reset after a PHY select */
    ret = USB_CoreReset(USBx);
  }
  else /* Embedded Phy */
  {
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx)
    /* USBD_HS_SPEED || USBH_HS_SPEED */
    if (cfg.speed == USBD_HS_SPEED)
    {
      /* Switch HS2 */
      USB_OTG_HS2->USB_SWITCH |= USB_OTG_HS2_USB_SWITCH;
      USB_OTG_HS2->POWERON_CORE |= USB_OTG_HS2_POWERON_CORE;
      USB_OTG_HS2->OTG_SUSPENDM |= USB_OTG_HS2_OTG_SUSPENDM;
      USB_OTG_HS2->SW_RREF_I2C = 0x05U;

      /* Select HS ULPI PHY, no effect on HS2 */
      USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_FSSTSEL);

      /* Reset after a PHY select */
      ret = USB_CoreReset(USBx);

      if (cfg.battery_charging_enable == 0U)
      {
        /* Activate the USB Transceiver */
        USBx->GGCCFG |= USB_OTG_GGCCFG_PWEN;
      }
      else
      {
        /* Deactivate the USB Transceiver */
        USBx->GGCCFG &= ~(USB_OTG_GGCCFG_PWEN);
      }
    }
    else
    {
      /* Select FS Embedded PHY */
      USBx->GUSBCFG |= USB_OTG_GUSBCFG_FSSTSEL;

      /* Reset after a PHY select */
      ret = USB_CoreReset(USBx);

      if (cfg.battery_charging_enable == 0U)
      {
        /* Activate the USB Transceiver */
        USBx->GGCCFG |= USB_OTG_GGCCFG_PWEN;
      }
      else
      {
        /* Deactivate the USB Transceiver */
        USBx->GGCCFG &= ~(USB_OTG_GGCCFG_PWEN);
      }
    }
#else
    /* Select FS Embedded PHY */
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_FSSTSEL;

    /* Reset after a PHY select */
    ret = USB_CoreReset(USBx);

    if (cfg.battery_charging_enable == 0U)
    {
        /* Activate the USB Transceiver */
        USBx->GGCCFG |= USB_OTG_GGCCFG_PWEN;
    }
    else
    {
        /* Deactivate the USB Transceiver */
        USBx->GGCCFG &= ~(USB_OTG_GGCCFG_PWEN);
    }
#endif /* APM32F405xx || APM32F407xx || APM32F417xx */
  }

  if (cfg.dma_enable == 1U)
  {
    USBx->GAHBCFG |= USB_OTG_GAHBCFG_BLT_2;
    USBx->GAHBCFG |= USB_OTG_GAHBCFG_DMAEN;
  }

  return ret;
}


/**
  * @brief  Set the USB turnaround time
  * @param  USBx USB Instance
  * @param  hclk: AHB clock frequency
  * @retval USB turnaround time In PHY Clocks number
  */
DAL_StatusTypeDef USB_SetTurnaroundTime(USB_OTG_GlobalTypeDef *USBx,
                                        uint32_t hclk, uint8_t speed)
{
  uint32_t UsbTrd;

  /* The USBTRD is configured according to the tables below, depending on AHB frequency
  used by application. In the low AHB frequency range it is used to stretch enough the USB response
  time to IN tokens, the USB turnaround time, so to compensate for the longer AHB read access
  latency to the Data FIFO */
  if (speed == USBD_FS_SPEED)
  {
    if ((hclk >= 14200000U) && (hclk < 15000000U))
    {
      /* hclk Clock Range between 14.2-15 MHz */
      UsbTrd = 0xFU;
    }
    else if ((hclk >= 15000000U) && (hclk < 16000000U))
    {
      /* hclk Clock Range between 15-16 MHz */
      UsbTrd = 0xEU;
    }
    else if ((hclk >= 16000000U) && (hclk < 17200000U))
    {
      /* hclk Clock Range between 16-17.2 MHz */
      UsbTrd = 0xDU;
    }
    else if ((hclk >= 17200000U) && (hclk < 18500000U))
    {
      /* hclk Clock Range between 17.2-18.5 MHz */
      UsbTrd = 0xCU;
    }
    else if ((hclk >= 18500000U) && (hclk < 20000000U))
    {
      /* hclk Clock Range between 18.5-20 MHz */
      UsbTrd = 0xBU;
    }
    else if ((hclk >= 20000000U) && (hclk < 21800000U))
    {
      /* hclk Clock Range between 20-21.8 MHz */
      UsbTrd = 0xAU;
    }
    else if ((hclk >= 21800000U) && (hclk < 24000000U))
    {
      /* hclk Clock Range between 21.8-24 MHz */
      UsbTrd = 0x9U;
    }
    else if ((hclk >= 24000000U) && (hclk < 27700000U))
    {
      /* hclk Clock Range between 24-27.7 MHz */
      UsbTrd = 0x8U;
    }
    else if ((hclk >= 27700000U) && (hclk < 32000000U))
    {
      /* hclk Clock Range between 27.7-32 MHz */
      UsbTrd = 0x7U;
    }
    else /* if(hclk >= 32000000) */
    {
      /* hclk Clock Range between 32-200 MHz */
      UsbTrd = 0x6U;
    }
  }
  else if (speed == USBD_HS_SPEED)
  {
    UsbTrd = USBD_HS_TRDT_VALUE;
  }
  else
  {
    UsbTrd = USBD_DEFAULT_TRDT_VALUE;
  }

  USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_TRTIM;
  USBx->GUSBCFG |= (uint32_t)((UsbTrd << 10) & USB_OTG_GUSBCFG_TRTIM);

  return DAL_OK;
}

/**
  * @brief  USB_EnableGlobalInt
  *         Enables the controller's Global Int in the AHB Config reg
  * @param  USBx  Selected device
  * @retval DAL status
  */
DAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx)
{
  USBx->GAHBCFG |= USB_OTG_GAHBCFG_GINTMASK;
  return DAL_OK;
}

/**
  * @brief  USB_DisableGlobalInt
  *         Disable the controller's Global Int in the AHB Config reg
  * @param  USBx  Selected device
  * @retval DAL status
  */
DAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx)
{
  USBx->GAHBCFG &= ~USB_OTG_GAHBCFG_GINTMASK;
  return DAL_OK;
}

/**
  * @brief  USB_SetCurrentMode Set functional mode
  * @param  USBx  Selected device
  * @param  mode  current core mode
  *          This parameter can be one of these values:
  *            @arg USB_DEVICE_MODE Peripheral mode
  *            @arg USB_HOST_MODE Host mode
  * @retval DAL status
  */
DAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx, USB_OTG_ModeTypeDef mode)
{
  uint32_t ms = 0U;

  USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMODE | USB_OTG_GUSBCFG_FDMODE);

  if (mode == USB_HOST_MODE)
  {
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_FHMODE;

    do
    {
      DAL_Delay(1U);
      ms++;
    } while ((USB_GetMode(USBx) != (uint32_t)USB_HOST_MODE) && (ms < 50U));
  }
  else if (mode == USB_DEVICE_MODE)
  {
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_FDMODE;

    do
    {
      DAL_Delay(1U);
      ms++;
    } while ((USB_GetMode(USBx) != (uint32_t)USB_DEVICE_MODE) && (ms < 50U));
  }
  else
  {
    return DAL_ERROR;
  }

  if (ms == 50U)
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  USB_DevInit Initializes the USB_OTG controller registers
  *         for device mode
  * @param  USBx  Selected device
  * @param  cfg   pointer to a USB_OTG_CfgTypeDef structure that contains
  *         the configuration information for the specified USBx peripheral.
  * @retval DAL status
  */
DAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg)
{
  DAL_StatusTypeDef ret = DAL_OK;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t i;

  for (i = 0U; i < 15U; i++)
  {
    USBx->DTXFIFO[i] = 0U;
  }

  /* VBUS Sensing setup */
  if (cfg.vbus_sensing_enable == 0U)
  {
    /*
     * Disable HW VBUS sensing. VBUS is internally considered to be always
     * at VBUS-Valid level (5V).
     */
    USBx_DEVICE->DCTRL |= USB_OTG_DCTRL_SDCNNT;
    USBx->GGCCFG |= USB_OTG_GGCCFG_VBSDIS;
    USBx->GGCCFG &= ~USB_OTG_GGCCFG_BDVBSEN;
    USBx->GGCCFG &= ~USB_OTG_GGCCFG_ADVBSEN;
  }
  else
  {
    /* Enable HW VBUS sensing */
    USBx->GGCCFG &= ~USB_OTG_GGCCFG_VBSDIS;
    USBx->GGCCFG |= USB_OTG_GGCCFG_BDVBSEN;
  }

  /* Restart the Phy Clock */
  USBx_PCGCCTL = 0U;

  /* Device mode configuration */
  USBx_DEVICE->DCFG |= DCFG_FRAME_INTERVAL_80;

  if (cfg.phy_itface == USB_OTG_ULPI_PHY)
  {
    if (cfg.speed == USBD_HS_SPEED)
    {
      /* Set Core speed to High speed mode */
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH);
    }
    else
    {
      /* Set Core speed to Full speed mode */
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH_IN_FULL);
    }
  }
  else
  {
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx)
    if (cfg.speed == USBD_HS_SPEED)
    {
      /* Set Core speed to High speed mode */
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH);
    }
    else if (cfg.speed == USBD_HSINFS_SPEED)
    {
      /* Set Core speed to Full speed mode */
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_FULL);
    }
    else
    {
      /* Set Core speed to Full speed mode */
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_FULL);
    }
#else
    /* Set Core speed to Full speed mode */
    (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_FULL);
#endif /* APM32F405xx || APM32F407xx || APM32F417xx */
  }

  /* Flush the FIFOs */
  if (USB_FlushTxFifo(USBx, 0x10U) != DAL_OK) /* all Tx FIFOs */
  {
    ret = DAL_ERROR;
  }

  if (USB_FlushRxFifo(USBx) != DAL_OK)
  {
    ret = DAL_ERROR;
  }

  /* Clear all pending Device Interrupts */
  USBx_DEVICE->DINIMASK = 0U;
  USBx_DEVICE->DOUTIMASK = 0U;
  USBx_DEVICE->DAEPIMASK = 0U;

  for (i = 0U; i < cfg.dev_endpoints; i++)
  {
    if ((USBx_INEP(i)->DIEPCTRL & USB_OTG_DIEPCTRL_EPEN) == USB_OTG_DIEPCTRL_EPEN)
    {
      if (i == 0U)
      {
        USBx_INEP(i)->DIEPCTRL = USB_OTG_DIEPCTRL_NAKSET;
      }
      else
      {
        USBx_INEP(i)->DIEPCTRL = USB_OTG_DIEPCTRL_EPDIS | USB_OTG_DIEPCTRL_NAKSET;
      }
    }
    else
    {
      USBx_INEP(i)->DIEPCTRL = 0U;
    }

    USBx_INEP(i)->DIEPTRS = 0U;
    USBx_INEP(i)->DIEPINT  = 0xFB7FU;
  }

  for (i = 0U; i < cfg.dev_endpoints; i++)
  {
    if ((USBx_OUTEP(i)->DOEPCTRL & USB_OTG_DOEPCTRL_EPEN) == USB_OTG_DOEPCTRL_EPEN)
    {
      if (i == 0U)
      {
        USBx_OUTEP(i)->DOEPCTRL = USB_OTG_DOEPCTRL_NAKSET;
      }
      else
      {
        USBx_OUTEP(i)->DOEPCTRL = USB_OTG_DOEPCTRL_EPDIS | USB_OTG_DOEPCTRL_NAKSET;
      }
    }
    else
    {
      USBx_OUTEP(i)->DOEPCTRL = 0U;
    }

    USBx_OUTEP(i)->DOEPTRS = 0U;
    USBx_OUTEP(i)->DOEPINT  = 0xFB7FU;
  }

  USBx_DEVICE->DINIMASK &= ~(USB_OTG_DINIMASK_FUDRM);

  /* Disable all interrupts. */
  USBx->GINTMASK = 0U;

  /* Clear any pending interrupts */
  USBx->GCINT = 0xBFFFFFFFU;

  /* Enable the common interrupts */
  if (cfg.dma_enable == 0U)
  {
    USBx->GINTMASK |= USB_OTG_GINTMASK_RXFNONEM;
  }

  /* Enable interrupts matching to the Device mode ONLY */
  USBx->GINTMASK |= USB_OTG_GINTMASK_USBSUSM | USB_OTG_GINTMASK_USBRSTM |
                   USB_OTG_GINTMASK_ENUMDM | USB_OTG_GINTMASK_INEPM |
                   USB_OTG_GINTMASK_OUTEPM   | USB_OTG_GINTMASK_IIINTXM |
                   USB_OTG_GINTMASK_IP_OUTTXM | USB_OTG_GINTMASK_RWAKEM;

  if (cfg.Sof_enable != 0U)
  {
    USBx->GINTMASK |= USB_OTG_GINTMASK_SOFM;
  }

  if (cfg.vbus_sensing_enable == 1U)
  {
    USBx->GINTMASK |= (USB_OTG_GINTMASK_SREQM | USB_OTG_GINTMASK_OTGM);
  }

  return ret;
}

/**
  * @brief  USB_FlushTxFifo Flush a Tx FIFO
  * @param  USBx  Selected device
  * @param  num  FIFO number
  *         This parameter can be a value from 1 to 15
            15 means Flush all Tx FIFOs
  * @retval DAL status
  */
DAL_StatusTypeDef USB_FlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num)
{
  __IO uint32_t count = 0U;

  /* Wait for AHB master IDLE state. */
  do
  {
    count++;

    if (count > 200000U)
    {
      return DAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTRL & USB_OTG_GRSTCTRL_AHBMIDL) == 0U);

  /* Flush TX Fifo */
  count = 0U;
  USBx->GRSTCTRL = (USB_OTG_GRSTCTRL_TXFFLU | (num << 6));

  do
  {
    count++;

    if (count > 200000U)
    {
      return DAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTRL & USB_OTG_GRSTCTRL_TXFFLU) == USB_OTG_GRSTCTRL_TXFFLU);

  return DAL_OK;
}

/**
  * @brief  USB_FlushRxFifo  Flush Rx FIFO
  * @param  USBx  Selected device
  * @retval DAL status
  */
DAL_StatusTypeDef USB_FlushRxFifo(USB_OTG_GlobalTypeDef *USBx)
{
  __IO uint32_t count = 0U;

  /* Wait for AHB master IDLE state. */
  do
  {
    count++;

    if (count > 200000U)
    {
      return DAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTRL & USB_OTG_GRSTCTRL_AHBMIDL) == 0U);

  /* Flush RX Fifo */
  count = 0U;
  USBx->GRSTCTRL = USB_OTG_GRSTCTRL_RXFFLU;

  do
  {
    count++;

    if (count > 200000U)
    {
      return DAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTRL & USB_OTG_GRSTCTRL_RXFFLU) == USB_OTG_GRSTCTRL_RXFFLU);

  return DAL_OK;
}

/**
  * @brief  USB_SetDevSpeed  Initializes the DevSpd field of DCFG register
  *         depending the PHY type and the enumeration speed of the device.
  * @param  USBx  Selected device
  * @param  speed  device speed
  *          This parameter can be one of these values:
  *            @arg USB_OTG_SPEED_HIGH: High speed mode
  *            @arg USB_OTG_SPEED_HIGH_IN_FULL: High speed core in Full Speed mode
  *            @arg USB_OTG_SPEED_FULL: Full speed mode
  * @retval  Hal status
  */
DAL_StatusTypeDef USB_SetDevSpeed(USB_OTG_GlobalTypeDef *USBx, uint8_t speed)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  USBx_DEVICE->DCFG |= speed;
  return DAL_OK;
}

/**
  * @brief  USB_GetDevSpeed  Return the Dev Speed
  * @param  USBx  Selected device
  * @retval speed  device speed
  *          This parameter can be one of these values:
  *            @arg USBD_HS_SPEED: High speed mode
  *            @arg USBD_FS_SPEED: Full speed mode
  */
uint8_t USB_GetDevSpeed(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint8_t speed;
  uint32_t DevEnumSpeed = USBx_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD;

  if (DevEnumSpeed == DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ)
  {
    speed = USBD_HS_SPEED;
  }
  else if ((DevEnumSpeed == DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ) ||
           (DevEnumSpeed == DSTS_ENUMSPD_FS_PHY_48MHZ))
  {
    speed = USBD_FS_SPEED;
  }
  else
  {
    speed = 0xFU;
  }

  return speed;
}

/**
  * @brief  Activate and configure an endpoint
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @retval DAL status
  */
DAL_StatusTypeDef USB_ActivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  if (ep->is_in == 1U)
  {
    USBx_DEVICE->DAEPIMASK |= USB_OTG_DAEPIMASK_AINM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK));

    if ((USBx_INEP(epnum)->DIEPCTRL & USB_OTG_DIEPCTRL_USBAEP) == 0U)
    {
      USBx_INEP(epnum)->DIEPCTRL |= (ep->maxpacket & USB_OTG_DIEPCTRL_MAXPS) |
                                   ((uint32_t)ep->type << 18) | (epnum << 22) |
                                   USB_OTG_DIEPCTRL_DPIDSET |
                                   USB_OTG_DIEPCTRL_USBAEP;
    }
  }
  else
  {
    USBx_DEVICE->DAEPIMASK |= USB_OTG_DAEPIMASK_AOUTM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16);

    if (((USBx_OUTEP(epnum)->DOEPCTRL) & USB_OTG_DOEPCTRL_USBAEP) == 0U)
    {
      USBx_OUTEP(epnum)->DOEPCTRL |= (ep->maxpacket & USB_OTG_DOEPCTRL_MAXPS) |
                                    ((uint32_t)ep->type << 18) |
                                    USB_OTG_DIEPCTRL_DPIDSET |
                                    USB_OTG_DOEPCTRL_USBAEP;
    }
  }
  return DAL_OK;
}

/**
  * @brief  Activate and configure a dedicated endpoint
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @retval DAL status
  */
DAL_StatusTypeDef USB_ActivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  /* Read DEPCTLn register */
  if (ep->is_in == 1U)
  {
    if (((USBx_INEP(epnum)->DIEPCTRL) & USB_OTG_DIEPCTRL_USBAEP) == 0U)
    {
      USBx_INEP(epnum)->DIEPCTRL |= (ep->maxpacket & USB_OTG_DIEPCTRL_MAXPS) |
                                   ((uint32_t)ep->type << 18) | (epnum << 22) |
                                   USB_OTG_DIEPCTRL_DPIDSET |
                                   USB_OTG_DIEPCTRL_USBAEP;
    }

    USBx_DEVICE->DEPIMASK |= USB_OTG_DAEPIMASK_AINM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK));
  }
  else
  {
    if (((USBx_OUTEP(epnum)->DOEPCTRL) & USB_OTG_DOEPCTRL_USBAEP) == 0U)
    {
      USBx_OUTEP(epnum)->DOEPCTRL |= (ep->maxpacket & USB_OTG_DOEPCTRL_MAXPS) |
                                    ((uint32_t)ep->type << 18) | (epnum << 22) |
                                    USB_OTG_DOEPCTRL_USBAEP;
    }

    USBx_DEVICE->DEPIMASK |= USB_OTG_DAEPIMASK_AOUTM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16);
  }

  return DAL_OK;
}

/**
  * @brief  De-activate and de-initialize an endpoint
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @retval DAL status
  */
DAL_StatusTypeDef USB_DeactivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  /* Read DEPCTLn register */
  if (ep->is_in == 1U)
  {
    if ((USBx_INEP(epnum)->DIEPCTRL & USB_OTG_DIEPCTRL_EPEN) == USB_OTG_DIEPCTRL_EPEN)
    {
      USBx_INEP(epnum)->DIEPCTRL |= USB_OTG_DIEPCTRL_NAKSET;
      USBx_INEP(epnum)->DIEPCTRL |= USB_OTG_DIEPCTRL_EPDIS;
    }

    USBx_DEVICE->DEPIMASK &= ~(USB_OTG_DAEPIMASK_AINM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
    USBx_DEVICE->DAEPIMASK &= ~(USB_OTG_DAEPIMASK_AINM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
    USBx_INEP(epnum)->DIEPCTRL &= ~(USB_OTG_DIEPCTRL_USBAEP |
                                   USB_OTG_DIEPCTRL_MAXPS |
                                   USB_OTG_DIEPCTRL_TXFNUM |
                                   USB_OTG_DIEPCTRL_DPIDSET |
                                   USB_OTG_DIEPCTRL_EPTYPE);
  }
  else
  {
    if ((USBx_OUTEP(epnum)->DOEPCTRL & USB_OTG_DOEPCTRL_EPEN) == USB_OTG_DOEPCTRL_EPEN)
    {
      USBx_OUTEP(epnum)->DOEPCTRL |= USB_OTG_DOEPCTRL_NAKSET;
      USBx_OUTEP(epnum)->DOEPCTRL |= USB_OTG_DOEPCTRL_EPDIS;
    }

    USBx_DEVICE->DEPIMASK &= ~(USB_OTG_DAEPIMASK_AOUTM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
    USBx_DEVICE->DAEPIMASK &= ~(USB_OTG_DAEPIMASK_AOUTM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
    USBx_OUTEP(epnum)->DOEPCTRL &= ~(USB_OTG_DOEPCTRL_USBAEP |
                                    USB_OTG_DOEPCTRL_MAXPS |
                                    USB_OTG_DOEPCTRL_DPIDSET |
                                    USB_OTG_DOEPCTRL_EPTYPE);
  }

  return DAL_OK;
}

/**
  * @brief  De-activate and de-initialize a dedicated endpoint
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @retval DAL status
  */
DAL_StatusTypeDef USB_DeactivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  /* Read DEPCTLn register */
  if (ep->is_in == 1U)
  {
    if ((USBx_INEP(epnum)->DIEPCTRL & USB_OTG_DIEPCTRL_EPEN) == USB_OTG_DIEPCTRL_EPEN)
    {
      USBx_INEP(epnum)->DIEPCTRL  |= USB_OTG_DIEPCTRL_NAKSET;
      USBx_INEP(epnum)->DIEPCTRL  |= USB_OTG_DIEPCTRL_EPDIS;
    }

    USBx_INEP(epnum)->DIEPCTRL &= ~ USB_OTG_DIEPCTRL_USBAEP;
    USBx_DEVICE->DAEPIMASK &= ~(USB_OTG_DAEPIMASK_AINM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
  }
  else
  {
    if ((USBx_OUTEP(epnum)->DOEPCTRL & USB_OTG_DOEPCTRL_EPEN) == USB_OTG_DOEPCTRL_EPEN)
    {
      USBx_OUTEP(epnum)->DOEPCTRL  |= USB_OTG_DOEPCTRL_NAKSET;
      USBx_OUTEP(epnum)->DOEPCTRL  |= USB_OTG_DOEPCTRL_EPDIS;
    }

    USBx_OUTEP(epnum)->DOEPCTRL &= ~USB_OTG_DOEPCTRL_USBAEP;
    USBx_DEVICE->DAEPIMASK &= ~(USB_OTG_DAEPIMASK_AOUTM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
  }

  return DAL_OK;
}

/**
  * @brief  USB_EPStartXfer : setup and starts a transfer over an EP
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @param  dma USB dma enabled or disabled
  *          This parameter can be one of these values:
  *           0 : DMA feature not used
  *           1 : DMA feature used
  * @retval DAL status
  */
DAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep, uint8_t dma)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;
  uint16_t pktcnt;

  /* IN endpoint */
  if (ep->is_in == 1U)
  {
    /* Zero Length Packet? */
    if (ep->xfer_len == 0U)
    {
      USBx_INEP(epnum)->DIEPTRS &= ~(USB_OTG_DIEPTRS_EPPCNT);
      USBx_INEP(epnum)->DIEPTRS |= (USB_OTG_DIEPTRS_EPPCNT & (1U << 19));
      USBx_INEP(epnum)->DIEPTRS &= ~(USB_OTG_DIEPTRS_EPTRS);
    }
    else
    {
      /* Program the transfer size and packet count
      * as follows: xfersize = N * maxpacket +
      * short_packet pktcnt = N + (short_packet
      * exist ? 1 : 0)
      */
      USBx_INEP(epnum)->DIEPTRS &= ~(USB_OTG_DIEPTRS_EPTRS);
      USBx_INEP(epnum)->DIEPTRS &= ~(USB_OTG_DIEPTRS_EPPCNT);
      USBx_INEP(epnum)->DIEPTRS |= (USB_OTG_DIEPTRS_EPPCNT &
                                     (((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket) << 19));

      USBx_INEP(epnum)->DIEPTRS |= (USB_OTG_DIEPTRS_EPTRS & ep->xfer_len);

      if (ep->type == EP_TYPE_ISOC)
      {
        USBx_INEP(epnum)->DIEPTRS &= ~(USB_OTG_DIEPTRS_TXDSEL);
        USBx_INEP(epnum)->DIEPTRS |= (USB_OTG_DIEPTRS_TXDSEL & (1U << 29));
      }
    }

    if (dma == 1U)
    {
      if ((uint32_t)ep->dma_addr != 0U)
      {
        USBx_INEP(epnum)->DIEPDMA = (uint32_t)(ep->dma_addr);
      }

      if (ep->type == EP_TYPE_ISOC)
      {
        if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U)
        {
          USBx_INEP(epnum)->DIEPCTRL |= USB_OTG_DIEPCTRL_OFSET;
        }
        else
        {
          USBx_INEP(epnum)->DIEPCTRL |= USB_OTG_DIEPCTRL_DPIDSET;
        }
      }

      /* EP enable, IN data in FIFO */
      USBx_INEP(epnum)->DIEPCTRL |= (USB_OTG_DIEPCTRL_NAKCLR | USB_OTG_DIEPCTRL_EPEN);
    }
    else
    {
      /* EP enable, IN data in FIFO */
      USBx_INEP(epnum)->DIEPCTRL |= (USB_OTG_DIEPCTRL_NAKCLR | USB_OTG_DIEPCTRL_EPEN);

      if (ep->type != EP_TYPE_ISOC)
      {
        /* Enable the Tx FIFO Empty Interrupt for this EP */
        if (ep->xfer_len > 0U)
        {
          USBx_DEVICE->DIEIMASK |= 1UL << (ep->num & EP_ADDR_MSK);
        }
      }
      else
      {
        if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U)
        {
          USBx_INEP(epnum)->DIEPCTRL |= USB_OTG_DIEPCTRL_OFSET;
        }
        else
        {
          USBx_INEP(epnum)->DIEPCTRL |= USB_OTG_DIEPCTRL_DPIDSET;
        }

        (void)USB_WritePacket(USBx, ep->xfer_buff, ep->num, (uint16_t)ep->xfer_len, dma);
      }
    }
  }
  else /* OUT endpoint */
  {
    /* Program the transfer size and packet count as follows:
    * pktcnt = N
    * xfersize = N * maxpacket
    */
    USBx_OUTEP(epnum)->DOEPTRS &= ~(USB_OTG_DOEPTRS_EPTRS);
    USBx_OUTEP(epnum)->DOEPTRS &= ~(USB_OTG_DOEPTRS_EPPCNT);

    if (ep->xfer_len == 0U)
    {
      USBx_OUTEP(epnum)->DOEPTRS |= (USB_OTG_DOEPTRS_EPTRS & ep->maxpacket);
      USBx_OUTEP(epnum)->DOEPTRS |= (USB_OTG_DOEPTRS_EPPCNT & (1U << 19));
    }
    else
    {
      pktcnt = (uint16_t)((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket);
      ep->xfer_size = ep->maxpacket * pktcnt;

      USBx_OUTEP(epnum)->DOEPTRS |= USB_OTG_DOEPTRS_EPPCNT & ((uint32_t)pktcnt << 19);
      USBx_OUTEP(epnum)->DOEPTRS |= USB_OTG_DOEPTRS_EPTRS & ep->xfer_size;
    }

    if (dma == 1U)
    {
      if ((uint32_t)ep->xfer_buff != 0U)
      {
        USBx_OUTEP(epnum)->DOEPDMA = (uint32_t)(ep->xfer_buff);
      }
    }

    if (ep->type == EP_TYPE_ISOC)
    {
      if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U)
      {
        USBx_OUTEP(epnum)->DOEPCTRL |= USB_OTG_DOEPCTRL_OFSET;
      }
      else
      {
        USBx_OUTEP(epnum)->DOEPCTRL |= USB_OTG_DOEPCTRL_DPIDSET;
      }
    }
    /* EP enable */
    USBx_OUTEP(epnum)->DOEPCTRL |= (USB_OTG_DOEPCTRL_NAKCLR | USB_OTG_DOEPCTRL_EPEN);
  }

  return DAL_OK;
}

/**
  * @brief  USB_EP0StartXfer : setup and starts a transfer over the EP  0
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @param  dma USB dma enabled or disabled
  *          This parameter can be one of these values:
  *           0 : DMA feature not used
  *           1 : DMA feature used
  * @retval DAL status
  */
DAL_StatusTypeDef USB_EP0StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep, uint8_t dma)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  /* IN endpoint */
  if (ep->is_in == 1U)
  {
    /* Zero Length Packet? */
    if (ep->xfer_len == 0U)
    {
      USBx_INEP(epnum)->DIEPTRS &= ~(USB_OTG_DIEPTRS_EPPCNT);
      USBx_INEP(epnum)->DIEPTRS |= (USB_OTG_DIEPTRS_EPPCNT & (1U << 19));
      USBx_INEP(epnum)->DIEPTRS &= ~(USB_OTG_DIEPTRS_EPTRS);
    }
    else
    {
      /* Program the transfer size and packet count
      * as follows: xfersize = N * maxpacket +
      * short_packet pktcnt = N + (short_packet
      * exist ? 1 : 0)
      */
      USBx_INEP(epnum)->DIEPTRS &= ~(USB_OTG_DIEPTRS_EPTRS);
      USBx_INEP(epnum)->DIEPTRS &= ~(USB_OTG_DIEPTRS_EPPCNT);

      if (ep->xfer_len > ep->maxpacket)
      {
        ep->xfer_len = ep->maxpacket;
      }
      USBx_INEP(epnum)->DIEPTRS |= (USB_OTG_DIEPTRS_EPPCNT & (1U << 19));
      USBx_INEP(epnum)->DIEPTRS |= (USB_OTG_DIEPTRS_EPTRS & ep->xfer_len);
    }

    if (dma == 1U)
    {
      if ((uint32_t)ep->dma_addr != 0U)
      {
        USBx_INEP(epnum)->DIEPDMA = (uint32_t)(ep->dma_addr);
      }

      /* EP enable, IN data in FIFO */
      USBx_INEP(epnum)->DIEPCTRL |= (USB_OTG_DIEPCTRL_NAKCLR | USB_OTG_DIEPCTRL_EPEN);
    }
    else
    {
      /* EP enable, IN data in FIFO */
      USBx_INEP(epnum)->DIEPCTRL |= (USB_OTG_DIEPCTRL_NAKCLR | USB_OTG_DIEPCTRL_EPEN);

      /* Enable the Tx FIFO Empty Interrupt for this EP */
      if (ep->xfer_len > 0U)
      {
        USBx_DEVICE->DIEIMASK |= 1UL << (ep->num & EP_ADDR_MSK);
      }
    }
  }
  else /* OUT endpoint */
  {
    /* Program the transfer size and packet count as follows:
    * pktcnt = N
    * xfersize = N * maxpacket
    */
    USBx_OUTEP(epnum)->DOEPTRS &= ~(USB_OTG_DOEPTRS_EPTRS);
    USBx_OUTEP(epnum)->DOEPTRS &= ~(USB_OTG_DOEPTRS_EPPCNT);

    if (ep->xfer_len > 0U)
    {
      ep->xfer_len = ep->maxpacket;
    }

    /* Store transfer size, for EP0 this is equal to endpoint max packet size */
    ep->xfer_size = ep->maxpacket;

    USBx_OUTEP(epnum)->DOEPTRS |= (USB_OTG_DOEPTRS_EPPCNT & (1U << 19));
    USBx_OUTEP(epnum)->DOEPTRS |= (USB_OTG_DOEPTRS_EPTRS & ep->xfer_size);

    if (dma == 1U)
    {
      if ((uint32_t)ep->xfer_buff != 0U)
      {
        USBx_OUTEP(epnum)->DOEPDMA = (uint32_t)(ep->xfer_buff);
      }
    }

    /* EP enable */
    USBx_OUTEP(epnum)->DOEPCTRL |= (USB_OTG_DOEPCTRL_NAKCLR | USB_OTG_DOEPCTRL_EPEN);
  }

  return DAL_OK;
}


/**
   * @brief  USB_EPStoptXfer  Stop transfer on an EP
   * @param  USBx  usb device instance
   * @param  ep pointer to endpoint structure
   * @retval DAL status
   */
DAL_StatusTypeDef USB_EPStopXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  __IO uint32_t count = 0U;
  DAL_StatusTypeDef ret = DAL_OK;
  uint32_t USBx_BASE = (uint32_t)USBx;

  /* IN endpoint */
  if (ep->is_in == 1U)
  {
    /* EP enable, IN data in FIFO */
    if (((USBx_INEP(ep->num)->DIEPCTRL) & USB_OTG_DIEPCTRL_EPEN) == USB_OTG_DIEPCTRL_EPEN)
    {
      USBx_INEP(ep->num)->DIEPCTRL |= (USB_OTG_DIEPCTRL_NAKSET);
      USBx_INEP(ep->num)->DIEPCTRL |= (USB_OTG_DIEPCTRL_EPDIS);

      do
      {
        count++;

        if (count > 10000U)
        {
          ret = DAL_ERROR;
          break;
        }
      } while (((USBx_INEP(ep->num)->DIEPCTRL) & USB_OTG_DIEPCTRL_EPEN) ==  USB_OTG_DIEPCTRL_EPEN);
    }
  }
  else /* OUT endpoint */
  {
    if (((USBx_OUTEP(ep->num)->DOEPCTRL) & USB_OTG_DOEPCTRL_EPEN) == USB_OTG_DOEPCTRL_EPEN)
    {
      USBx_OUTEP(ep->num)->DOEPCTRL |= (USB_OTG_DOEPCTRL_NAKSET);
      USBx_OUTEP(ep->num)->DOEPCTRL |= (USB_OTG_DOEPCTRL_EPDIS);

      do
      {
        count++;

        if (count > 10000U)
        {
          ret = DAL_ERROR;
          break;
        }
      } while (((USBx_OUTEP(ep->num)->DOEPCTRL) & USB_OTG_DOEPCTRL_EPEN) ==  USB_OTG_DOEPCTRL_EPEN);
    }
  }

  return ret;
}


/**
  * @brief  USB_WritePacket : Writes a packet into the Tx FIFO associated
  *         with the EP/channel
  * @param  USBx  Selected device
  * @param  src   pointer to source buffer
  * @param  ch_ep_num  endpoint or host channel number
  * @param  len  Number of bytes to write
  * @param  dma USB dma enabled or disabled
  *          This parameter can be one of these values:
  *           0 : DMA feature not used
  *           1 : DMA feature used
  * @retval DAL status
  */
DAL_StatusTypeDef USB_WritePacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *src,
                                  uint8_t ch_ep_num, uint16_t len, uint8_t dma)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint8_t *pSrc = src;
  uint32_t count32b;
  uint32_t i;

  if (dma == 0U)
  {
    count32b = ((uint32_t)len + 3U) / 4U;
    for (i = 0U; i < count32b; i++)
    {
      USBx_DFIFO((uint32_t)ch_ep_num) = __UNALIGNED_UINT32_READ(pSrc);
      pSrc++;
      pSrc++;
      pSrc++;
      pSrc++;
    }
  }

  return DAL_OK;
}

/**
  * @brief  USB_ReadPacket : read a packet from the RX FIFO
  * @param  USBx  Selected device
  * @param  dest  source pointer
  * @param  len  Number of bytes to read
  * @retval pointer to destination buffer
  */
void *USB_ReadPacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *dest, uint16_t len)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint8_t *pDest = dest;
  uint32_t pData;
  uint32_t i;
  uint32_t count32b = (uint32_t)len >> 2U;
  uint16_t remaining_bytes = len % 4U;

  for (i = 0U; i < count32b; i++)
  {
    __UNALIGNED_UINT32_WRITE(pDest, USBx_DFIFO(0U));
    pDest++;
    pDest++;
    pDest++;
    pDest++;
  }

  /* When Number of data is not word aligned, read the remaining byte */
  if (remaining_bytes != 0U)
  {
    i = 0U;
    __UNALIGNED_UINT32_WRITE(&pData, USBx_DFIFO(0U));

    do
    {
      *(uint8_t *)pDest = (uint8_t)(pData >> (8U * (uint8_t)(i)));
      i++;
      pDest++;
      remaining_bytes--;
    } while (remaining_bytes != 0U);
  }

  return ((void *)pDest);
}

/**
  * @brief  USB_EPSetStall : set a stall condition over an EP
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @retval DAL status
  */
DAL_StatusTypeDef USB_EPSetStall(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  if (ep->is_in == 1U)
  {
    if (((USBx_INEP(epnum)->DIEPCTRL & USB_OTG_DIEPCTRL_EPEN) == 0U) && (epnum != 0U))
    {
      USBx_INEP(epnum)->DIEPCTRL &= ~(USB_OTG_DIEPCTRL_EPDIS);
    }
    USBx_INEP(epnum)->DIEPCTRL |= USB_OTG_DIEPCTRL_STALLH;
  }
  else
  {
    if (((USBx_OUTEP(epnum)->DOEPCTRL & USB_OTG_DOEPCTRL_EPEN) == 0U) && (epnum != 0U))
    {
      USBx_OUTEP(epnum)->DOEPCTRL &= ~(USB_OTG_DOEPCTRL_EPDIS);
    }
    USBx_OUTEP(epnum)->DOEPCTRL |= USB_OTG_DOEPCTRL_STALLH;
  }

  return DAL_OK;
}

/**
  * @brief  USB_EPClearStall : Clear a stall condition over an EP
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @retval DAL status
  */
DAL_StatusTypeDef USB_EPClearStall(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  if (ep->is_in == 1U)
  {
    USBx_INEP(epnum)->DIEPCTRL &= ~USB_OTG_DIEPCTRL_STALLH;
    if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK))
    {
      USBx_INEP(epnum)->DIEPCTRL |= USB_OTG_DIEPCTRL_DPIDSET; /* DATA0 */
    }
  }
  else
  {
    USBx_OUTEP(epnum)->DOEPCTRL &= ~USB_OTG_DOEPCTRL_STALLH;
    if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK))
    {
      USBx_OUTEP(epnum)->DOEPCTRL |= USB_OTG_DOEPCTRL_DPIDSET; /* DATA0 */
    }
  }
  return DAL_OK;
}

/**
  * @brief  USB_StopDevice : Stop the usb device mode
  * @param  USBx  Selected device
  * @retval DAL status
  */
DAL_StatusTypeDef USB_StopDevice(USB_OTG_GlobalTypeDef *USBx)
{
  DAL_StatusTypeDef ret;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t i;

  /* Clear Pending interrupt */
  for (i = 0U; i < 15U; i++)
  {
    USBx_INEP(i)->DIEPINT = 0xFB7FU;
    USBx_OUTEP(i)->DOEPINT = 0xFB7FU;
  }

  /* Clear interrupt masks */
  USBx_DEVICE->DINIMASK  = 0U;
  USBx_DEVICE->DOUTIMASK  = 0U;
  USBx_DEVICE->DAEPIMASK = 0U;

  /* Flush the FIFO */
  ret = USB_FlushRxFifo(USBx);
  if (ret != DAL_OK)
  {
    return ret;
  }

  ret = USB_FlushTxFifo(USBx,  0x10U);
  if (ret != DAL_OK)
  {
    return ret;
  }

  return ret;
}

/**
  * @brief  USB_SetDevAddress : Stop the usb device mode
  * @param  USBx  Selected device
  * @param  address  new device address to be assigned
  *          This parameter can be a value from 0 to 255
  * @retval DAL status
  */
DAL_StatusTypeDef  USB_SetDevAddress(USB_OTG_GlobalTypeDef *USBx, uint8_t address)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  USBx_DEVICE->DCFG &= ~(USB_OTG_DCFG_DADDR);
  USBx_DEVICE->DCFG |= ((uint32_t)address << 4) & USB_OTG_DCFG_DADDR;

  return DAL_OK;
}

/**
  * @brief  USB_DevConnect : Connect the USB device by enabling Rpu
  * @param  USBx  Selected device
  * @retval DAL status
  */
DAL_StatusTypeDef  USB_DevConnect(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  /* In case phy is stopped, ensure to ungate and restore the phy CLK */
  USBx_PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

  USBx_DEVICE->DCTRL &= ~USB_OTG_DCTRL_SDCNNT;

  return DAL_OK;
}

/**
  * @brief  USB_DevDisconnect : Disconnect the USB device by disabling Rpu
  * @param  USBx  Selected device
  * @retval DAL status
  */
DAL_StatusTypeDef  USB_DevDisconnect(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  /* In case phy is stopped, ensure to ungate and restore the phy CLK */
  USBx_PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

  USBx_DEVICE->DCTRL |= USB_OTG_DCTRL_SDCNNT;

  return DAL_OK;
}

/**
  * @brief  USB_ReadInterrupts: return the global USB interrupt status
  * @param  USBx  Selected device
  * @retval DAL status
  */
uint32_t  USB_ReadInterrupts(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t tmpreg;

  tmpreg = USBx->GCINT;
  tmpreg &= USBx->GINTMASK;

  return tmpreg;
}

/**
  * @brief  USB_ReadDevAllOutEpInterrupt: return the USB device OUT endpoints interrupt status
  * @param  USBx  Selected device
  * @retval DAL status
  */
uint32_t USB_ReadDevAllOutEpInterrupt(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;

  tmpreg  = USBx_DEVICE->DAEPINT;
  tmpreg &= USBx_DEVICE->DAEPIMASK;

  return ((tmpreg & 0xffff0000U) >> 16);
}

/**
  * @brief  USB_ReadDevAllInEpInterrupt: return the USB device IN endpoints interrupt status
  * @param  USBx  Selected device
  * @retval DAL status
  */
uint32_t USB_ReadDevAllInEpInterrupt(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;

  tmpreg  = USBx_DEVICE->DAEPINT;
  tmpreg &= USBx_DEVICE->DAEPIMASK;

  return ((tmpreg & 0xFFFFU));
}

/**
  * @brief  Returns Device OUT EP Interrupt register
  * @param  USBx  Selected device
  * @param  epnum  endpoint number
  *          This parameter can be a value from 0 to 15
  * @retval Device OUT EP Interrupt register
  */
uint32_t USB_ReadDevOutEPInterrupt(USB_OTG_GlobalTypeDef *USBx, uint8_t epnum)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;

  tmpreg  = USBx_OUTEP((uint32_t)epnum)->DOEPINT;
  tmpreg &= USBx_DEVICE->DOUTIMASK;

  return tmpreg;
}

/**
  * @brief  Returns Device IN EP Interrupt register
  * @param  USBx  Selected device
  * @param  epnum  endpoint number
  *          This parameter can be a value from 0 to 15
  * @retval Device IN EP Interrupt register
  */
uint32_t USB_ReadDevInEPInterrupt(USB_OTG_GlobalTypeDef *USBx, uint8_t epnum)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;
  uint32_t msk;
  uint32_t emp;

  msk = USBx_DEVICE->DINIMASK;
  emp = USBx_DEVICE->DIEIMASK;
  msk |= ((emp >> (epnum & EP_ADDR_MSK)) & 0x1U) << 7;
  tmpreg = USBx_INEP((uint32_t)epnum)->DIEPINT & msk;

  return tmpreg;
}

/**
  * @brief  USB_ClearInterrupts: clear a USB interrupt
  * @param  USBx  Selected device
  * @param  interrupt  flag
  * @retval None
  */
void  USB_ClearInterrupts(USB_OTG_GlobalTypeDef *USBx, uint32_t interrupt)
{
  USBx->GCINT |= interrupt;
}

/**
  * @brief  Returns USB core mode
  * @param  USBx  Selected device
  * @retval return core mode : Host or Device
  *          This parameter can be one of these values:
  *           0 : Host
  *           1 : Device
  */
uint32_t USB_GetMode(USB_OTG_GlobalTypeDef *USBx)
{
  return ((USBx->GCINT) & 0x1U);
}

/**
  * @brief  Activate EP0 for Setup transactions
  * @param  USBx  Selected device
  * @retval DAL status
  */
DAL_StatusTypeDef  USB_ActivateSetup(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  /* Set the MPS of the IN EP0 to 64 bytes */
  USBx_INEP(0U)->DIEPCTRL &= ~USB_OTG_DIEPCTRL_MAXPS;

  USBx_DEVICE->DCTRL |= USB_OTG_DCTRL_GINAKCLR;

  return DAL_OK;
}

/**
  * @brief  Prepare the EP0 to start the first control setup
  * @param  USBx  Selected device
  * @param  dma USB dma enabled or disabled
  *          This parameter can be one of these values:
  *           0 : DMA feature not used
  *           1 : DMA feature used
  * @param  psetup  pointer to setup packet
  * @retval DAL status
  */
DAL_StatusTypeDef USB_EP0_OutStart(USB_OTG_GlobalTypeDef *USBx, uint8_t dma, uint8_t *psetup)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t gSNPSiD = *(__IO uint32_t *)(&USBx->GCID + 0x1U);

  if (gSNPSiD > USB_OTG_CORE_ID_300A)
  {
    if ((USBx_OUTEP(0U)->DOEPCTRL & USB_OTG_DOEPCTRL_EPEN) == USB_OTG_DOEPCTRL_EPEN)
    {
      return DAL_OK;
    }
  }

  USBx_OUTEP(0U)->DOEPTRS = 0U;
  USBx_OUTEP(0U)->DOEPTRS |= (USB_OTG_DOEPTRS_EPPCNT & (1U << 19));
  USBx_OUTEP(0U)->DOEPTRS |= (3U * 8U);
  USBx_OUTEP(0U)->DOEPTRS |=  USB_OTG_DOEPTRS_SPCNT;

  if (dma == 1U)
  {
    USBx_OUTEP(0U)->DOEPDMA = (uint32_t)psetup;
    /* EP enable */
    USBx_OUTEP(0U)->DOEPCTRL |= USB_OTG_DOEPCTRL_EPEN | USB_OTG_DOEPCTRL_USBAEP;
  }

  return DAL_OK;
}

/**
  * @brief  Reset the USB Core (needed after USB clock settings change)
  * @param  USBx  Selected device
  * @retval DAL status
  */
static DAL_StatusTypeDef USB_CoreReset(USB_OTG_GlobalTypeDef *USBx)
{
  __IO uint32_t count = 0U;

  /* Wait for AHB master IDLE state. */
  do
  {
    count++;

    if (count > 200000U)
    {
      return DAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTRL & USB_OTG_GRSTCTRL_AHBMIDL) == 0U);

  /* Core Soft Reset */
  count = 0U;
  USBx->GRSTCTRL |= USB_OTG_GRSTCTRL_CSRST;

  do
  {
    count++;

    if (count > 200000U)
    {
      return DAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTRL & USB_OTG_GRSTCTRL_CSRST) == USB_OTG_GRSTCTRL_CSRST);

  return DAL_OK;
}

/**
  * @brief  USB_HostInit : Initializes the USB OTG controller registers
  *         for Host mode
  * @param  USBx  Selected device
  * @param  cfg   pointer to a USB_OTG_CfgTypeDef structure that contains
  *         the configuration information for the specified USBx peripheral.
  * @retval DAL status
  */
DAL_StatusTypeDef USB_HostInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg)
{
  DAL_StatusTypeDef ret = DAL_OK;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t i;

  /* Restart the Phy Clock */
  USBx_PCGCCTL = 0U;

  /*
  * Disable HW VBUS sensing. VBUS is internally considered to be always
  * at VBUS-Valid level (5V).
  */
  USBx->GGCCFG |= USB_OTG_GGCCFG_VBSDIS;
  USBx->GGCCFG &= ~USB_OTG_GGCCFG_BDVBSEN;
  USBx->GGCCFG &= ~USB_OTG_GGCCFG_ADVBSEN;

  if ((USBx->GCID & (0x1U << 8)) != 0U)
  {
    if (cfg.speed == USBH_FSLS_SPEED)
    {
      /* Force Device Enumeration to FS/LS mode only */
      USBx_HOST->HCFG |= USB_OTG_HCFG_FSSPT;
    }
    else
    {
      /* Set default Max speed support */
      USBx_HOST->HCFG &= ~(USB_OTG_HCFG_FSSPT);
    }
  }
  else
  {
    /* Set default Max speed support */
    USBx_HOST->HCFG &= ~(USB_OTG_HCFG_FSSPT);
  }

  /* Make sure the FIFOs are flushed. */
  if (USB_FlushTxFifo(USBx, 0x10U) != DAL_OK) /* all Tx FIFOs */
  {
    ret = DAL_ERROR;
  }

  if (USB_FlushRxFifo(USBx) != DAL_OK)
  {
    ret = DAL_ERROR;
  }

  /* Clear all pending HC Interrupts */
  for (i = 0U; i < cfg.Host_channels; i++)
  {
    USBx_HC(i)->HCHINT = 0xFFFFFFFFU;
    USBx_HC(i)->HCHIMASK = 0U;
  }

  /* Disable all interrupts. */
  USBx->GINTMASK = 0U;

  /* Clear any pending interrupts */
  USBx->GCINT = 0xFFFFFFFFU;

#if 0
  if ((USBx->GCID & (0x1U << 8)) != 0U)
  {
    /* set Rx FIFO size */
    USBx->GRXFIFO  = 0x200U;
    USBx->GTXFCFG = (uint32_t)(((0x100U << 16) & USB_OTG_NPTXFDEP) | 0x200U);
    USBx->GHPTXFSIZE = (uint32_t)(((0xE0U << 16) & USB_OTG_GHPTXFSIZE_HPDTXFDEP) | 0x300U);
  }
  else
  {
    /* set Rx FIFO size */
    USBx->GRXFIFO  = 0x80U;
    USBx->GTXFCFG = (uint32_t)(((0x60U << 16) & USB_OTG_NPTXFDEP) | 0x80U);
    USBx->GHPTXFSIZE = (uint32_t)(((0x40U << 16)& USB_OTG_GHPTXFSIZE_HPDTXFDEP) | 0xE0U);
  }
#endif

  /* USBD_HS_SPEED || USBH_HS_SPEED */
  if (cfg.speed == USBD_HS_SPEED)
  {
    /* set Rx FIFO size */
    USBx->GRXFIFO  = 0x200U;
    USBx->GTXFCFG = (uint32_t)(((0x100U << 16) & USB_OTG_NPTXFDEP) | 0x200U);
    USBx->GHPTXFSIZE = (uint32_t)(((0xE0U << 16) & USB_OTG_GHPTXFSIZE_HPDTXFDEP) | 0x300U);
  }
  else
  {
    /* set Rx FIFO size */
    USBx->GRXFIFO  = 0x80U;
    USBx->GTXFCFG = (uint32_t)(((0x60U << 16) & USB_OTG_NPTXFDEP) | 0x80U);
    USBx->GHPTXFSIZE = (uint32_t)(((0x40U << 16)& USB_OTG_GHPTXFSIZE_HPDTXFDEP) | 0xE0U);
  }

  /* Enable the common interrupts */
  if (cfg.dma_enable == 0U)
  {
    USBx->GINTMASK |= USB_OTG_GINTMASK_RXFNONEM;
  }

  /* Enable interrupts matching to the Host mode ONLY */
  USBx->GINTMASK |= (USB_OTG_GINTMASK_HPORTM            | USB_OTG_GINTMASK_HCHM | \
                    USB_OTG_GINTMASK_SOFM             | USB_OTG_GCINT_DEDIS | \
                    USB_OTG_GINTMASK_IP_OUTTXM  | USB_OTG_GINTMASK_RWAKEM);

  return ret;
}

/**
  * @brief  USB_InitFSLSPClkSel : Initializes the FSLSPClkSel field of the
  *         HCFG register on the PHY type and set the right frame interval
  * @param  USBx  Selected device
  * @param  freq  clock frequency
  *          This parameter can be one of these values:
  *           HCFG_48_MHZ : Full Speed 48 MHz Clock
  *           HCFG_6_MHZ : Low Speed 6 MHz Clock
  * @retval DAL status
  */
DAL_StatusTypeDef USB_InitFSLSPClkSel(USB_OTG_GlobalTypeDef *USBx, uint8_t freq)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  USBx_HOST->HCFG &= ~(USB_OTG_HCFG_PHYCLKSEL);
  USBx_HOST->HCFG |= (uint32_t)freq & USB_OTG_HCFG_PHYCLKSEL;

  if (freq == HCFG_48_MHZ)
  {
    USBx_HOST->HFIVL = 48000U;
  }
  else if (freq == HCFG_6_MHZ)
  {
    USBx_HOST->HFIVL = 6000U;
  }
  else
  {
    /* ... */
  }

  return DAL_OK;
}

/**
  * @brief  USB_OTG_ResetPort : Reset Host Port
  * @param  USBx  Selected device
  * @retval DAL status
  * @note (1)The application must wait at least 10 ms
  *   before clearing the reset bit.
  */
DAL_StatusTypeDef USB_ResetPort(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  __IO uint32_t hprt0 = 0U;

  hprt0 = USBx_HPRT0;

  hprt0 &= ~(USB_OTG_HPORTCSTS_PEN | USB_OTG_HPORTCSTS_PCINTFLG |
             USB_OTG_HPORTCSTS_PENCHG | USB_OTG_HPORTCSTS_POVCCHG);

  USBx_HPRT0 = (USB_OTG_HPORTCSTS_PRST | hprt0);
  DAL_Delay(100U);                                 /* See Note #1 */
  USBx_HPRT0 = ((~USB_OTG_HPORTCSTS_PRST) & hprt0);
  DAL_Delay(10U);

  return DAL_OK;
}

/**
  * @brief  USB_DriveVbus : activate or de-activate vbus
  * @param  state  VBUS state
  *          This parameter can be one of these values:
  *           0 : Deactivate VBUS
  *           1 : Activate VBUS
  * @retval DAL status
  */
DAL_StatusTypeDef USB_DriveVbus(USB_OTG_GlobalTypeDef *USBx, uint8_t state)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  __IO uint32_t hprt0 = 0U;

  hprt0 = USBx_HPRT0;

  hprt0 &= ~(USB_OTG_HPORTCSTS_PEN | USB_OTG_HPORTCSTS_PCINTFLG |
             USB_OTG_HPORTCSTS_PENCHG | USB_OTG_HPORTCSTS_POVCCHG);

  if (((hprt0 & USB_OTG_HPORTCSTS_PP) == 0U) && (state == 1U))
  {
    USBx_HPRT0 = (USB_OTG_HPORTCSTS_PP | hprt0);
  }
  if (((hprt0 & USB_OTG_HPORTCSTS_PP) == USB_OTG_HPORTCSTS_PP) && (state == 0U))
  {
    USBx_HPRT0 = ((~USB_OTG_HPORTCSTS_PP) & hprt0);
  }
  return DAL_OK;
}

/**
  * @brief  Return Host Core speed
  * @param  USBx  Selected device
  * @retval speed : Host speed
  *          This parameter can be one of these values:
  *            @arg HCD_SPEED_HIGH: High speed mode
  *            @arg HCD_SPEED_FULL: Full speed mode
  *            @arg HCD_SPEED_LOW: Low speed mode
  */
uint32_t USB_GetHostSpeed(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  __IO uint32_t hprt0 = 0U;

  hprt0 = USBx_HPRT0;
  return ((hprt0 & USB_OTG_HPORTCSTS_PSPDSEL) >> 17);
}

/**
  * @brief  Return Host Current Frame number
  * @param  USBx  Selected device
  * @retval current frame number
  */
uint32_t USB_GetCurrentFrame(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  return (USBx_HOST->HFIFM & USB_OTG_HFIFM_FNUM);
}

/**
  * @brief  Initialize a host channel
  * @param  USBx  Selected device
  * @param  ch_num  Channel number
  *         This parameter can be a value from 1 to 15
  * @param  epnum  Endpoint number
  *          This parameter can be a value from 1 to 15
  * @param  dev_address  Current device address
  *          This parameter can be a value from 0 to 255
  * @param  speed  Current device speed
  *          This parameter can be one of these values:
  *            @arg USB_OTG_SPEED_HIGH: High speed mode
  *            @arg USB_OTG_SPEED_FULL: Full speed mode
  *            @arg USB_OTG_SPEED_LOW: Low speed mode
  * @param  ep_type  Endpoint Type
  *          This parameter can be one of these values:
  *            @arg EP_TYPE_CTRL: Control type
  *            @arg EP_TYPE_ISOC: Isochronous type
  *            @arg EP_TYPE_BULK: Bulk type
  *            @arg EP_TYPE_INTR: Interrupt type
  * @param  mps  Max Packet Size
  *          This parameter can be a value from 0 to 32K
  * @retval DAL state
  */
DAL_StatusTypeDef USB_HC_Init(USB_OTG_GlobalTypeDef *USBx, uint8_t ch_num,
                              uint8_t epnum, uint8_t dev_address, uint8_t speed,
                              uint8_t ep_type, uint16_t mps)
{
  DAL_StatusTypeDef ret = DAL_OK;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t HCcharEpDir;
  uint32_t HCcharLowSpeed;
  uint32_t HostCoreSpeed;

  /* Clear old interrupt conditions for this host channel. */
  USBx_HC((uint32_t)ch_num)->HCHINT = 0xFFFFFFFFU;

  /* Enable channel interrupts required for this transfer. */
  switch (ep_type)
  {
    case EP_TYPE_CTRL:
    case EP_TYPE_BULK:
      USBx_HC((uint32_t)ch_num)->HCHIMASK = USB_OTG_HCHIMASK_TSFCMPNM  |
                                            USB_OTG_HCHIMASK_RXSTALLM |
                                            USB_OTG_HCHIMASK_TERRM |
                                            USB_OTG_HCHIMASK_DTOGM |
                                            USB_OTG_HCHIMASK_AHBERRM |
                                            USB_OTG_HCHIMASK_RXNAKM;

      if ((epnum & 0x80U) == 0x80U)
      {
        USBx_HC((uint32_t)ch_num)->HCHIMASK |= USB_OTG_HCHIMASK_BABBLEM;
      }
      else
      {
        if ((USBx->GCID & (0x1U << 8)) != 0U)
        {
          USBx_HC((uint32_t)ch_num)->HCHIMASK |= USB_OTG_HCHIMASK_RXNYETM |
                                                 USB_OTG_HCHIMASK_RXTXACKM;
        }
      }
      break;

    case EP_TYPE_INTR:
      USBx_HC((uint32_t)ch_num)->HCHIMASK = USB_OTG_HCHIMASK_TSFCMPNM  |
                                            USB_OTG_HCHIMASK_RXSTALLM |
                                            USB_OTG_HCHIMASK_TERRM |
                                            USB_OTG_HCHIMASK_DTOGM |
                                            USB_OTG_HCHIMASK_RXNAKM   |
                                            USB_OTG_HCHIMASK_AHBERRM |
                                            USB_OTG_HCHIMASK_FOVRM;

      if ((epnum & 0x80U) == 0x80U)
      {
        USBx_HC((uint32_t)ch_num)->HCHIMASK |= USB_OTG_HCHIMASK_BABBLEM;
      }

      break;

    case EP_TYPE_ISOC:
      USBx_HC((uint32_t)ch_num)->HCHIMASK = USB_OTG_HCHIMASK_TSFCMPNM  |
                                            USB_OTG_HCHIMASK_RXTXACKM   |
                                            USB_OTG_HCHIMASK_AHBERRM |
                                            USB_OTG_HCHIMASK_FOVRM;

      if ((epnum & 0x80U) == 0x80U)
      {
        USBx_HC((uint32_t)ch_num)->HCHIMASK |= (USB_OTG_HCHIMASK_TERRM | USB_OTG_HCHIMASK_BABBLEM);
      }
      break;

    default:
      ret = DAL_ERROR;
      break;
  }

  /* Enable host channel Halt interrupt */
  USBx_HC((uint32_t)ch_num)->HCHIMASK |= USB_OTG_HCHIMASK_TSFCMPANM;

  /* Enable the top level host channel interrupt. */
  USBx_HOST->HACHIMASK |= 1UL << (ch_num & 0xFU);

  /* Make sure host channel interrupts are enabled. */
  USBx->GINTMASK |= USB_OTG_GINTMASK_HCHM;

  /* Program the HCCHAR register */
  if ((epnum & 0x80U) == 0x80U)
  {
    HCcharEpDir = (0x1U << 15) & USB_OTG_HCH_EDPDRT;
  }
  else
  {
    HCcharEpDir = 0U;
  }

  HostCoreSpeed = USB_GetHostSpeed(USBx);

  /* LS device plugged to HUB */
  if ((speed == HPRT0_PRTSPD_LOW_SPEED) && (HostCoreSpeed != HPRT0_PRTSPD_LOW_SPEED))
  {
    HCcharLowSpeed = (0x1U << 17) & USB_OTG_HCH_LSDV;
  }
  else
  {
    HCcharLowSpeed = 0U;
  }

  USBx_HC((uint32_t)ch_num)->HCH = (((uint32_t)dev_address << 22) & USB_OTG_HCH_DVADDR) |
                                      ((((uint32_t)epnum & 0x7FU) << 11) & USB_OTG_HCH_EDPNUM) |
                                      (((uint32_t)ep_type << 18) & USB_OTG_HCH_EDPTYP) |
                                      ((uint32_t)mps & USB_OTG_HCH_MAXPSIZE) | HCcharEpDir | HCcharLowSpeed;

  if ((ep_type == EP_TYPE_INTR) || (ep_type == EP_TYPE_ISOC))
  {
    USBx_HC((uint32_t)ch_num)->HCH |= USB_OTG_HCH_ODDF;
  }

  return ret;
}

/**
  * @brief  Start a transfer over a host channel
  * @param  USBx  Selected device
  * @param  hc  pointer to host channel structure
  * @param  dma USB dma enabled or disabled
  *          This parameter can be one of these values:
  *           0 : DMA feature not used
  *           1 : DMA feature used
  * @retval DAL state
  */
DAL_StatusTypeDef USB_HC_StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_HCTypeDef *hc, uint8_t dma)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t ch_num = (uint32_t)hc->ch_num;
  __IO uint32_t tmpreg;
  uint8_t  is_oddframe;
  uint16_t len_words;
  uint16_t num_packets;
  uint16_t max_hc_pkt_count = 256U;

  if (((USBx->GCID & (0x1U << 8)) != 0U) && (hc->speed == USBH_HS_SPEED))
  {
    /* in DMA mode host Core automatically issues ping  in case of NYET/NAK */
    if ((dma == 1U) && ((hc->ep_type == EP_TYPE_CTRL) || (hc->ep_type == EP_TYPE_BULK)))
    {
      USBx_HC((uint32_t)ch_num)->HCHIMASK &= ~(USB_OTG_HCHIMASK_RXNYETM |
                                               USB_OTG_HCHIMASK_RXTXACKM |
                                               USB_OTG_HCHIMASK_RXNAKM);
    }

    if ((dma == 0U) && (hc->do_ping == 1U))
    {
      (void)USB_DoPing(USBx, hc->ch_num);
      return DAL_OK;
    }

  }

  /* Compute the expected number of packets associated to the transfer */
  if (hc->xfer_len > 0U)
  {
    num_packets = (uint16_t)((hc->xfer_len + hc->max_packet - 1U) / hc->max_packet);

    if (num_packets > max_hc_pkt_count)
    {
      num_packets = max_hc_pkt_count;
      hc->XferSize = (uint32_t)num_packets * hc->max_packet;
    }
  }
  else
  {
    num_packets = 1U;
  }

  /*
   * For IN channel HCTSIZ.XferSize is expected to be an integer multiple of
   * max_packet size.
   */
  if (hc->ep_is_in != 0U)
  {
    hc->XferSize = (uint32_t)num_packets * hc->max_packet;
  }
  else
  {
    hc->XferSize = hc->xfer_len;
  }

  /* Initialize the HCTSIZn register */
  USBx_HC(ch_num)->HCHTSIZE = (hc->XferSize & USB_OTG_HCHTSIZE_TSFSIZE) |
                            (((uint32_t)num_packets << 19) & USB_OTG_HCHTSIZE_PCKTCNT) |
                            (((uint32_t)hc->data_pid << 29) & USB_OTG_HCHTSIZE_DATAPID);

  if (dma != 0U)
  {
    /* xfer_buff MUST be 32-bits aligned */
    USBx_HC(ch_num)->HCHDMA = (uint32_t)hc->xfer_buff;
  }

  is_oddframe = (((uint32_t)USBx_HOST->HFIFM & 0x01U) != 0U) ? 0U : 1U;
  USBx_HC(ch_num)->HCH &= ~USB_OTG_HCH_ODDF;
  USBx_HC(ch_num)->HCH |= (uint32_t)is_oddframe << 29;

  /* Set host channel enable */
  tmpreg = USBx_HC(ch_num)->HCH;
  tmpreg &= ~USB_OTG_HCH_CHINT;

  /* make sure to set the correct ep direction */
  if (hc->ep_is_in != 0U)
  {
    tmpreg |= USB_OTG_HCH_EDPDRT;
  }
  else
  {
    tmpreg &= ~USB_OTG_HCH_EDPDRT;
  }
  tmpreg |= USB_OTG_HCH_CHEN;
  USBx_HC(ch_num)->HCH = tmpreg;

  if (dma != 0U) /* dma mode */
  {
    return DAL_OK;
  }

  if ((hc->ep_is_in == 0U) && (hc->xfer_len > 0U))
  {
    switch (hc->ep_type)
    {
      /* Non periodic transfer */
      case EP_TYPE_CTRL:
      case EP_TYPE_BULK:

        len_words = (uint16_t)((hc->xfer_len + 3U) / 4U);

        /* check if there is enough space in FIFO space */
        if (len_words > (USBx->GNPTXFQSTS & 0xFFFFU))
        {
          /* need to process data in nptxfempty interrupt */
          USBx->GINTMASK |= USB_OTG_GINTMASK_NPTXFEMM;
        }
        break;

      /* Periodic transfer */
      case EP_TYPE_INTR:
      case EP_TYPE_ISOC:
        len_words = (uint16_t)((hc->xfer_len + 3U) / 4U);
        /* check if there is enough space in FIFO space */
        if (len_words > (USBx_HOST->HPTXSTS & 0xFFFFU)) /* split the transfer */
        {
          /* need to process data in ptxfempty interrupt */
          USBx->GINTMASK |= USB_OTG_GINTMASK_PTXFEM;
        }
        break;

      default:
        break;
    }

    /* Write packet into the Tx FIFO. */
    (void)USB_WritePacket(USBx, hc->xfer_buff, hc->ch_num, (uint16_t)hc->xfer_len, 0);
  }

  return DAL_OK;
}

/**
  * @brief Read all host channel interrupts status
  * @param  USBx  Selected device
  * @retval DAL state
  */
uint32_t USB_HC_ReadInterrupt(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  return ((USBx_HOST->HACHINT) & 0xFFFFU);
}

/**
  * @brief  Halt a host channel
  * @param  USBx  Selected device
  * @param  hc_num  Host Channel number
  *         This parameter can be a value from 1 to 15
  * @retval DAL state
  */
DAL_StatusTypeDef USB_HC_Halt(USB_OTG_GlobalTypeDef *USBx, uint8_t hc_num)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t hcnum = (uint32_t)hc_num;
  __IO uint32_t count = 0U;
  uint32_t HcEpType = (USBx_HC(hcnum)->HCH & USB_OTG_HCH_EDPTYP) >> 18;
  uint32_t ChannelEna = (USBx_HC(hcnum)->HCH & USB_OTG_HCH_CHEN) >> 31;

  if (((USBx->GAHBCFG & USB_OTG_GAHBCFG_DMAEN) == USB_OTG_GAHBCFG_DMAEN) &&
      (ChannelEna == 0U))
  {
    return DAL_OK;
  }

  /* Check for space in the request queue to issue the halt. */
  if ((HcEpType == HCCHAR_CTRL) || (HcEpType == HCCHAR_BULK))
  {
    USBx_HC(hcnum)->HCH |= USB_OTG_HCH_CHINT;

    if ((USBx->GAHBCFG & USB_OTG_GAHBCFG_DMAEN) == 0U)
    {
      if ((USBx->GNPTXFQSTS & (0xFFU << 16)) == 0U)
      {
        USBx_HC(hcnum)->HCH &= ~USB_OTG_HCH_CHEN;
        USBx_HC(hcnum)->HCH |= USB_OTG_HCH_CHEN;
        do
        {
          count++;

          if (count > 1000U)
          {
            break;
          }
        } while ((USBx_HC(hcnum)->HCH & USB_OTG_HCH_CHEN) == USB_OTG_HCH_CHEN);
      }
      else
      {
        USBx_HC(hcnum)->HCH |= USB_OTG_HCH_CHEN;
      }
    }
  }
  else
  {
    USBx_HC(hcnum)->HCH |= USB_OTG_HCH_CHINT;

    if ((USBx_HOST->HPTXSTS & (0xFFU << 16)) == 0U)
    {
      USBx_HC(hcnum)->HCH &= ~USB_OTG_HCH_CHEN;
      USBx_HC(hcnum)->HCH |= USB_OTG_HCH_CHEN;
      do
      {
        count++;

        if (count > 1000U)
        {
          break;
        }
      } while ((USBx_HC(hcnum)->HCH & USB_OTG_HCH_CHEN) == USB_OTG_HCH_CHEN);
    }
    else
    {
      USBx_HC(hcnum)->HCH |= USB_OTG_HCH_CHEN;
    }
  }

  return DAL_OK;
}

/**
  * @brief  Initiate Do Ping protocol
  * @param  USBx  Selected device
  * @param  hc_num  Host Channel number
  *         This parameter can be a value from 1 to 15
  * @retval DAL state
  */
DAL_StatusTypeDef USB_DoPing(USB_OTG_GlobalTypeDef *USBx, uint8_t ch_num)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t chnum = (uint32_t)ch_num;
  uint32_t num_packets = 1U;
  uint32_t tmpreg;

  USBx_HC(chnum)->HCHTSIZE = ((num_packets << 19) & USB_OTG_HCHTSIZE_PCKTCNT) |
                           USB_OTG_HCHTSIZE_DO_PING;

  /* Set host channel enable */
  tmpreg = USBx_HC(chnum)->HCH;
  tmpreg &= ~USB_OTG_HCH_CHINT;
  tmpreg |= USB_OTG_HCH_CHEN;
  USBx_HC(chnum)->HCH = tmpreg;

  return DAL_OK;
}

/**
  * @brief  Stop Host Core
  * @param  USBx  Selected device
  * @retval DAL state
  */
DAL_StatusTypeDef USB_StopHost(USB_OTG_GlobalTypeDef *USBx)
{
  DAL_StatusTypeDef ret = DAL_OK;
  uint32_t USBx_BASE = (uint32_t)USBx;
  __IO uint32_t count = 0U;
  uint32_t value;
  uint32_t i;

  (void)USB_DisableGlobalInt(USBx);

  /* Flush USB FIFO */
  if (USB_FlushTxFifo(USBx, 0x10U) != DAL_OK) /* all Tx FIFOs */
  {
    ret = DAL_ERROR;
  }

  if (USB_FlushRxFifo(USBx) != DAL_OK)
  {
    ret = DAL_ERROR;
  }

  /* Flush out any leftover queued requests. */
  for (i = 0U; i <= 15U; i++)
  {
    value = USBx_HC(i)->HCH;
    value |=  USB_OTG_HCH_CHINT;
    value &= ~USB_OTG_HCH_CHEN;
    value &= ~USB_OTG_HCH_EDPDRT;
    USBx_HC(i)->HCH = value;
  }

  /* Halt all channels to put them into a known state. */
  for (i = 0U; i <= 15U; i++)
  {
    value = USBx_HC(i)->HCH;
    value |= USB_OTG_HCH_CHINT;
    value |= USB_OTG_HCH_CHEN;
    value &= ~USB_OTG_HCH_EDPDRT;
    USBx_HC(i)->HCH = value;

    do
    {
      count++;

      if (count > 1000U)
      {
        break;
      }
    } while ((USBx_HC(i)->HCH & USB_OTG_HCH_CHEN) == USB_OTG_HCH_CHEN);
  }

  /* Clear any pending Host interrupts */
  USBx_HOST->HACHINT = 0xFFFFFFFFU;
  USBx->GCINT = 0xFFFFFFFFU;

  (void)USB_EnableGlobalInt(USBx);

  return ret;
}

/**
  * @brief  USB_ActivateRemoteWakeup active remote wakeup signalling
  * @param  USBx Selected device
  * @retval DAL status
  */
DAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  if ((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSSTS) == USB_OTG_DSTS_SUSSTS)
  {
    /* active Remote wakeup signalling */
    USBx_DEVICE->DCTRL |= USB_OTG_DCTRL_RWKUPS;
  }

  return DAL_OK;
}

/**
  * @brief  USB_DeActivateRemoteWakeup de-active remote wakeup signalling
  * @param  USBx Selected device
  * @retval DAL status
  */
DAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  /* active Remote wakeup signalling */
  USBx_DEVICE->DCTRL &= ~(USB_OTG_DCTRL_RWKUPS);

  return DAL_OK;
}
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */


/**
  * @}
  */

/**
  * @}
  */
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */
#endif /* defined (DAL_PCD_MODULE_ENABLED) || defined (DAL_HCD_MODULE_ENABLED) */

/**
  * @}
  */
