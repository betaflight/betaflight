/**
  ******************************************************************************
  * @file    usbd_dfu.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_dfu.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DFU_H
#define __USB_DFU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_DFU
  * @brief This file is the Header file for usbd_dfu.c
  * @{
  */


/** @defgroup USBD_DFU_Exported_Defines
  * @{
  */
#ifndef USBD_DFU_MAX_ITF_NUM
#define USBD_DFU_MAX_ITF_NUM            1U
#endif /* USBD_DFU_MAX_ITF_NUM */

#ifndef USBD_DFU_XFER_SIZE
#define USBD_DFU_XFER_SIZE             1024U
#endif /* USBD_DFU_XFER_SIZE */

#ifndef USBD_DFU_APP_DEFAULT_ADD
#define USBD_DFU_APP_DEFAULT_ADD       0x08008000U /* The first sector (32 KB) is reserved for DFU code */
#endif /* USBD_DFU_APP_DEFAULT_ADD */

#ifndef USBD_DFU_BM_ATTRIBUTES
#define USBD_DFU_BM_ATTRIBUTES         0x0BU
#endif /* USBD_DFU_BM_ATTRIBUTES */

#ifndef USBD_DFU_DETACH_TIMEOUT
#define USBD_DFU_DETACH_TIMEOUT        0xFFU
#endif /* USBD_DFU_DETACH_TIMEOUT */

#define USB_DFU_CONFIG_DESC_SIZ        (18U + (9U * USBD_DFU_MAX_ITF_NUM))
#define USB_DFU_DESC_SIZ               9U

#define DFU_DESCRIPTOR_TYPE            0x21U


/**************************************************/
/* DFU Requests  DFU states                       */
/**************************************************/
#define APP_STATE_IDLE                 0U
#define APP_STATE_DETACH               1U
#define DFU_STATE_IDLE                 2U
#define DFU_STATE_DNLOAD_SYNC          3U
#define DFU_STATE_DNLOAD_BUSY          4U
#define DFU_STATE_DNLOAD_IDLE          5U
#define DFU_STATE_MANIFEST_SYNC        6U
#define DFU_STATE_MANIFEST             7U
#define DFU_STATE_MANIFEST_WAIT_RESET  8U
#define DFU_STATE_UPLOAD_IDLE          9U
#define DFU_STATE_ERROR                10U

/**************************************************/
/* DFU errors                                     */
/**************************************************/
#define DFU_ERROR_NONE                 0x00U
#define DFU_ERROR_TARGET               0x01U
#define DFU_ERROR_FILE                 0x02U
#define DFU_ERROR_WRITE                0x03U
#define DFU_ERROR_ERASE                0x04U
#define DFU_ERROR_CHECK_ERASED         0x05U
#define DFU_ERROR_PROG                 0x06U
#define DFU_ERROR_VERIFY               0x07U
#define DFU_ERROR_ADDRESS              0x08U
#define DFU_ERROR_NOTDONE              0x09U
#define DFU_ERROR_FIRMWARE             0x0AU
#define DFU_ERROR_VENDOR               0x0BU
#define DFU_ERROR_USB                  0x0CU
#define DFU_ERROR_POR                  0x0DU
#define DFU_ERROR_UNKNOWN              0x0EU
#define DFU_ERROR_STALLEDPKT           0x0FU

/**************************************************/
/* DFU Manifestation State                        */
/**************************************************/
#define DFU_MANIFEST_COMPLETE          0x00U
#define DFU_MANIFEST_IN_PROGRESS       0x01U


/**************************************************/
/* Special Commands  with Download Request        */
/**************************************************/
#define DFU_CMD_GETCOMMANDS            0x00U
#define DFU_CMD_SETADDRESSPOINTER      0x21U
#define DFU_CMD_ERASE                  0x41U

#define DFU_MEDIA_ERASE                0x00U
#define DFU_MEDIA_PROGRAM              0x01U

/**************************************************/
/* Other defines                                  */
/**************************************************/
/* Bit Detach capable = bit 3 in bmAttributes field */
#define DFU_DETACH_MASK                (1U << 3)
#define DFU_MANIFEST_MASK              (1U << 2)
#define DFU_STATUS_DEPTH               6U

typedef enum
{
  DFU_DETACH = 0U,
  DFU_DNLOAD,
  DFU_UPLOAD,
  DFU_GETSTATUS,
  DFU_CLRSTATUS,
  DFU_GETSTATE,
  DFU_ABORT
} DFU_RequestTypeDef;

typedef  void (*pFunction)(void);


/**********  Descriptor of DFU interface 0 Alternate setting n ****************/
#define USBD_DFU_IF_DESC(n) \
  0x09,   /* bLength: Interface Descriptor size */ \
  USB_DESC_TYPE_INTERFACE,   /* bDescriptorType */ \
  0x00,   /* bInterfaceNumber: Number of Interface */ \
  (n),      /* bAlternateSetting: Alternate setting */ \
  0x00,   /* bNumEndpoints*/ \
  0xFE,   /* bInterfaceClass: Application Specific Class Code */ \
  0x01,   /* bInterfaceSubClass : Device Firmware Upgrade Code */ \
  0x02,   /* nInterfaceProtocol: DFU mode protocol */ \
  USBD_IDX_INTERFACE_STR + (n) + 1U /* iInterface: Index of string descriptor */

#define TRANSFER_SIZE_BYTES(size)      ((uint8_t)(size)), ((uint8_t)((size) >> 8))

#define IS_PROTECTED_AREA(add)         (uint8_t)((((add) >= 0x08000000) && ((add) < (APP_DEFAULT_ADD))) ? 1 : 0)

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

typedef struct
{
  union
  {
    uint32_t d32[USBD_DFU_XFER_SIZE / 4U];
    uint8_t d8[USBD_DFU_XFER_SIZE];
  } buffer;

  uint32_t wblock_num;
  uint32_t wlength;
  uint32_t data_ptr;
  uint32_t alt_setting;

  uint8_t dev_status[DFU_STATUS_DEPTH];
  uint8_t ReservedForAlign[2];
  uint8_t dev_state;
  uint8_t manif_state;
} USBD_DFU_HandleTypeDef;

typedef struct
{
  const uint8_t *pStrDesc;
  uint16_t (* Init)(void);
  uint16_t (* DeInit)(void);
  uint16_t (* Erase)(uint32_t Add);
  uint16_t (* Write)(uint8_t *src, uint8_t *dest, uint32_t Len);
  uint8_t *(* Read)(uint8_t *src, uint8_t *dest, uint32_t Len);
  uint16_t (* GetStatus)(uint32_t Add, uint8_t cmd, uint8_t *buff);
} USBD_DFU_MediaTypeDef;

typedef struct
{
  uint8_t           bLength;
  uint8_t           bDescriptorType;
  uint8_t           bmAttributes;
  uint16_t          wDetachTimeout;
  uint16_t          wTransferSze;
  uint16_t          bcdDFUVersion;
} __PACKED USBD_DFUFuncDescTypeDef;

/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef USBD_DFU;
#define USBD_DFU_CLASS &USBD_DFU
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_DFU_RegisterMedia(USBD_HandleTypeDef *pdev,
                               USBD_DFU_MediaTypeDef *fops);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_DFU_H */
/**
  * @}
  */

/**
  * @}
  */
