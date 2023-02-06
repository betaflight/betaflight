/**
  **************************************************************************
  * @file     usbd_int.h
  * @brief    usb interrupt header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_INT_H
#define __USBD_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup AT32F435_437_middlewares_usbd_drivers
  * @{
  */

/** @addtogroup USBD_drivers_int
  * @{
  */

/** @defgroup USBD_interrupt_exported_types
  * @{
  */
/* includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usb_core.h"

void usbd_irq_handler(otg_core_type *udev);
void usbd_ept_handler(usbd_core_type *udev);
void usbd_reset_handler(usbd_core_type *udev);
void usbd_sof_handler(usbd_core_type *udev);
void usbd_suspend_handler(usbd_core_type *udev);
void usbd_wakeup_handler(usbd_core_type *udev);
void usbd_inept_handler(usbd_core_type *udev);
void usbd_outept_handler(usbd_core_type *udev);
void usbd_enumdone_handler(usbd_core_type *udev);
void usbd_rxflvl_handler(usbd_core_type *udev);
void usbd_incomisioin_handler(usbd_core_type *udev);
void usbd_discon_handler(usbd_core_type *udev);
void usbd_incomisoout_handler(usbd_core_type *udev);
void usb_write_empty_txfifo(usbd_core_type *udev, uint32_t ept_num);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif

