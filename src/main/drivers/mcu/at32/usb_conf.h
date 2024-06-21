/**
  **************************************************************************
  * @file     usb_conf.h
  * @version  v2.0.5
  * @date     2022-02-11
  * @brief    usb config header file
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
#ifndef __USB_CONF_H
#define __USB_CONF_H

#ifdef __cplusplus
extern "C" {
#endif
 
#include "at32f435_437_usb.h"
#include "at32f435_437.h"
//#include "stdio.h"

/** @addtogroup AT32F437_periph_examples
  * @{
  */
  
/** @addtogroup 437_USB_device_vcp_loopback
  * @{
  */

/**
  * @brief enable usb device mode
  */
#define USE_OTG_DEVICE_MODE

/**
  * @brief enable usb host mode
  */
/* #define USE_OTG_HOST_MODE */

/**
  * @brief select otgfs1 or otgfs2 define
  */

/* use otgfs1 */
#define OTG_USB_ID                           1

/* use otgfs2 */
//#define OTG_USB_ID                         2

#if (OTG_USB_ID == 1)
#define USB_ID                           0
#define OTG_CLOCK                        CRM_OTGFS1_PERIPH_CLOCK
#define OTG_IRQ                          OTGFS1_IRQn
#define OTG_IRQ_HANDLER                  OTGFS1_IRQHandler
#define OTG_WKUP_IRQ                     OTGFS1_WKUP_IRQn
#define OTG_WKUP_HANDLER                 OTGFS1_WKUP_IRQHandler
#define OTG_WKUP_EXINT_LINE              EXINT_LINE_18

#define OTG_PIN_GPIO                     GPIOA
#define OTG_PIN_GPIO_CLOCK               CRM_GPIOA_PERIPH_CLOCK

#define OTG_PIN_DP                       GPIO_PINS_12
#define OTG_PIN_DP_SOURCE                GPIO_PINS_SOURCE12

#define OTG_PIN_DM                       GPIO_PINS_11
#define OTG_PIN_DM_SOURCE                GPIO_PINS_SOURCE11

#define OTG_PIN_VBUS                     GPIO_PINS_9
#define OTG_PIN_VBUS_SOURCE              GPIO_PINS_SOURCE9

#define OTG_PIN_ID                       GPIO_PINS_10
#define OTG_PIN_ID_SOURCE                GPIO_PINS_SOURCE10

#define OTG_PIN_SOF_GPIO                 GPIOA
#define OTG_PIN_SOF_GPIO_CLOCK           CRM_GPIOA_PERIPH_CLOCK
#define OTG_PIN_SOF                      GPIO_PINS_8
#define OTG_PIN_SOF_SOURCE               GPIO_PINS_SOURCE8

#define OTG_PIN_MUX                      GPIO_MUX_10
#endif

#if (OTG_USB_ID == 2)
#define USB_ID                           1
#define OTG_CLOCK                        CRM_OTGFS2_PERIPH_CLOCK
#define OTG_IRQ                          OTGFS2_IRQn
#define OTG_IRQ_HANDLER                  OTGFS2_IRQHandler
#define OTG_WKUP_IRQ                     OTGFS2_WKUP_IRQn
#define OTG_WKUP_HANDLER                 OTGFS2_WKUP_IRQHandler
#define OTG_WKUP_EXINT_LINE              EXINT_LINE_20

#define OTG_PIN_GPIO                     GPIOB
#define OTG_PIN_GPIO_CLOCK               CRM_GPIOB_PERIPH_CLOCK

#define OTG_PIN_DP                       GPIO_PINS_15
#define OTG_PIN_DP_SOURCE                GPIO_PINS_SOURCE15

#define OTG_PIN_DM                       GPIO_PINS_14
#define OTG_PIN_DM_SOURCE                GPIO_PINS_SOURCE14

#define OTG_PIN_VBUS                     GPIO_PINS_13
#define OTG_PIN_VBUS_SOURCE              GPIO_PINS_SOURCE13

#define OTG_PIN_ID                       GPIO_PINS_12
#define OTG_PIN_ID_SOURCE                GPIO_PINS_SOURCE10

#define OTG_PIN_SOF_GPIO                 GPIOA
#define OTG_PIN_SOF_GPIO_CLOCK           CRM_GPIOA_PERIPH_CLOCK
#define OTG_PIN_SOF                      GPIO_PINS_4
#define OTG_PIN_SOF_SOURCE               GPIO_PINS_SOURCE4

#define OTG_PIN_MUX                      GPIO_MUX_12
#endif

/**
  * @brief usb device mode config
  */
#ifdef USE_OTG_DEVICE_MODE
/**
  * @brief usb device mode fifo
  */
/* otg1 device fifo */
#define USBD_RX_SIZE                     128
#define USBD_EP0_TX_SIZE                 24
#define USBD_EP1_TX_SIZE                 20
#define USBD_EP2_TX_SIZE                 20
#define USBD_EP3_TX_SIZE                 20
#define USBD_EP4_TX_SIZE                 20
#define USBD_EP5_TX_SIZE                 20
#define USBD_EP6_TX_SIZE                 20
#define USBD_EP7_TX_SIZE                 20

/* otg2 device fifo */
#define USBD2_RX_SIZE                    128
#define USBD2_EP0_TX_SIZE                24
#define USBD2_EP1_TX_SIZE                20
#define USBD2_EP2_TX_SIZE                20
#define USBD2_EP3_TX_SIZE                20
#define USBD2_EP4_TX_SIZE                20
#define USBD2_EP5_TX_SIZE                20
#define USBD2_EP6_TX_SIZE                20
#define USBD2_EP7_TX_SIZE                20

/**
  * @brief usb endpoint max num define
  */
#ifndef USB_EPT_MAX_NUM
#define USB_EPT_MAX_NUM                   8
#endif
#endif

/**
  * @brief usb host mode config
  */
#ifdef USE_OTG_HOST_MODE
#ifndef USB_HOST_CHANNEL_NUM
#define USB_HOST_CHANNEL_NUM             16
#endif

/**
  * @brief usb host mode fifo
  */
/* otg1 host fifo */
#define USBH_RX_FIFO_SIZE                128
#define USBH_NP_TX_FIFO_SIZE             96
#define USBH_P_TX_FIFO_SIZE              96

/* otg2 host fifo */
#define USBH2_RX_FIFO_SIZE               128
#define USBH2_NP_TX_FIFO_SIZE            96
#define USBH2_P_TX_FIFO_SIZE             96
#endif

/**
  * @brief usb sof output enable
  */
/* #define USB_SOF_OUTPUT_ENABLE */

/**
  * @brief usb vbus ignore, not use vbus pin
  */
#define USB_VBUS_IGNORE

/**
  * @brief usb low power wakeup handler enable
  */
/* #define USB_LOW_POWER_WAKUP */

void usb_delay_ms(uint32_t ms);
void usb_delay_us(uint32_t us);
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
