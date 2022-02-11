/**
  ******************************************************************************
  * @file    usbd_conf_template.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_conf_template.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF_TEMPLATE_H
#define __USBD_CONF_TEMPLATE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32fxxx.h"  /* replace 'stm32xxx' with your HAL driver header filename, ex: stm32f4xx.h */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CONF
  * @brief USB device low level driver configuration file
  * @{
  */

/** @defgroup USBD_CONF_Exported_Defines
  * @{
  */

#define USBD_MAX_NUM_INTERFACES                     1U
#define USBD_MAX_NUM_CONFIGURATION                  1U
#define USBD_MAX_STR_DESC_SIZ                       0x100U
#define USBD_SELF_POWERED                           1U
#define USBD_DEBUG_LEVEL                            2U

/* ECM, RNDIS, DFU Class Config */
#define USBD_SUPPORT_USER_STRING_DESC               1U

/* BillBoard Class Config */
#define USBD_CLASS_USER_STRING_DESC                 1U
#define USBD_CLASS_BOS_ENABLED                      1U
#define USB_BB_MAX_NUM_ALT_MODE                     0x2U

/* MSC Class Config */
#define MSC_MEDIA_PACKET                            8192U

/* CDC Class Config */
#define USBD_CDC_INTERVAL                           2000U

/* DFU Class Config */
#define USBD_DFU_MAX_ITF_NUM                        1U
#define USBD_DFU_XFERS_IZE                          1024U

/* AUDIO Class Config */
#define USBD_AUDIO_FREQ                             22100U

/* CustomHID Class Config */
#define CUSTOM_HID_HS_BINTERVAL                     0x05U
#define CUSTOM_HID_FS_BINTERVAL                     0x05U
#define USBD_CUSTOMHID_OUTREPORT_BUF_SIZE           0x02U
#define USBD_CUSTOM_HID_REPORT_DESC_SIZE            163U

/* VIDEO Class Config */
#define UVC_1_1 /* #define UVC_1_0 */

/* To be used only with YUY2 and NV12 Video format, shouldn't be defined for MJPEG format */
#define USBD_UVC_FORMAT_UNCOMPRESSED

#ifdef USBD_UVC_FORMAT_UNCOMPRESSED
#define UVC_BITS_PER_PIXEL                          12U
#define UVC_UNCOMPRESSED_GUID                       UVC_GUID_NV12 /* UVC_GUID_YUY2 */

/* refer to Table 3-18 Color Matching Descriptor video class v1.1 */
#define UVC_COLOR_PRIMARIE                          0x01U
#define UVC_TFR_CHARACTERISTICS                     0x01U
#define UVC_MATRIX_COEFFICIENTS                     0x04U
#endif

/* Video Stream frame width and height */
#define UVC_WIDTH                                   176U
#define UVC_HEIGHT                                  144U

/* bEndpointAddress in Endpoint Descriptor */
#define UVC_IN_EP                                   0x81U

#define UVC_CAM_FPS_FS                              10U
#define UVC_CAM_FPS_HS                              5U

#define UVC_ISO_FS_MPS                              512U
#define UVC_ISO_HS_MPS                              512U

#define UVC_PACKET_SIZE                             UVC_ISO_FS_MPS
/* To be used with Device Only IP supporting double buffer mode */
/* #define UVC_HEADER_PACKET_CNT                     0x02U */
/* #define UVC_PACKET_SIZE                           (UVC_ISO_FS_MPS * UVC_HEADER_PACKET_CNT) */

#define UVC_MAX_FRAME_SIZE                          (UVC_WIDTH * UVC_HEIGHT * 16U / 8U)

/** @defgroup USBD_Exported_Macros
  * @{
  */

/* Memory management macros make sure to use static memory allocation */
/** Alias for memory allocation. */
#define USBD_malloc         (void *)USBD_static_malloc

/** Alias for memory release. */
#define USBD_free           USBD_static_free

/** Alias for memory set. */
#define USBD_memset         memset

/** Alias for memory copy. */
#define USBD_memcpy         memcpy

/** Alias for delay. */
#define USBD_Delay          HAL_Delay

/* DEBUG macros */
#if (USBD_DEBUG_LEVEL > 0U)
#define  USBD_UsrLog(...)   do { \
                                 printf(__VA_ARGS__); \
                                 printf("\n"); \
                               } while (0)
#else
#define USBD_UsrLog(...) do {} while (0)
#endif

#if (USBD_DEBUG_LEVEL > 1U)

#define  USBD_ErrLog(...) do { \
                               printf("ERROR: ") ; \
                               printf(__VA_ARGS__); \
                               printf("\n"); \
                             } while (0)
#else
#define USBD_ErrLog(...) do {} while (0)
#endif

#if (USBD_DEBUG_LEVEL > 2U)
#define  USBD_DbgLog(...)   do { \
                                 printf("DEBUG : ") ; \
                                 printf(__VA_ARGS__); \
                                 printf("\n"); \
                               } while (0)
#else
#define USBD_DbgLog(...) do {} while (0)
#endif

/**
  * @}
  */



/**
  * @}
  */


/** @defgroup USBD_CONF_Exported_Types
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CONF_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_CONF_Exported_Variables
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_CONF_Exported_FunctionsPrototype
  * @{
  */
/* Exported functions -------------------------------------------------------*/
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_TEMPLATE_H */


/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
