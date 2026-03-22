/**
  ******************************************************************************
  * @file    usbd_cdc_ecm.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_cdc_ecm.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#ifndef __USB_CDC_ECM_H
#define __USB_CDC_ECM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup usbd_cdc_ecm
  * @brief This file is the Header file for usbd_cdc_ecm.c
  * @{
  */


/** @defgroup usbd_cdc_ecm_Exported_Defines
  * @{
  */
/* Comment this define in order to disable the CDC ECM Notification pipe */


#define CDC_ECM_IN_EP                                   0x81U  /* EP1 for data IN */
#define CDC_ECM_OUT_EP                                  0x01U  /* EP1 for data OUT */
#define CDC_ECM_CMD_EP                                  0x82U  /* EP2 for CDC ECM commands */

#ifndef CDC_ECM_CMD_ITF_NBR
#define CDC_ECM_CMD_ITF_NBR                             0x00U /* Command Interface Number 0 */
#endif /* CDC_ECM_CMD_ITF_NBR */

#ifndef CDC_ECM_COM_ITF_NBR
#define CDC_ECM_COM_ITF_NBR                             0x01U /* Communication Interface Number 0 */
#endif /* CDC_ECM_CMD_ITF_NBR */

#ifndef CDC_ECM_HS_BINTERVAL
#define CDC_ECM_HS_BINTERVAL                            0x10U
#endif /* CDC_ECM_HS_BINTERVAL */

#ifndef CDC_ECM_FS_BINTERVAL
#define CDC_ECM_FS_BINTERVAL                            0x10U
#endif /* CDC_ECM_FS_BINTERVAL */

#ifndef USBD_SUPPORT_USER_STRING_DESC
#define USBD_SUPPORT_USER_STRING_DESC                   1U
#endif /* USBD_SUPPORT_USER_STRING_DESC */

/* CDC_ECM Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define CDC_ECM_DATA_HS_MAX_PACKET_SIZE                 512U  /* Endpoint IN & OUT Packet size */
#define CDC_ECM_DATA_FS_MAX_PACKET_SIZE                 64U   /* Endpoint IN & OUT Packet size */
#define CDC_ECM_CMD_PACKET_SIZE                         16U    /* Control Endpoint Packet size */

#define CDC_ECM_CONFIG_DESC_SIZ                         79U

#define CDC_ECM_DATA_HS_IN_PACKET_SIZE                  CDC_ECM_DATA_HS_MAX_PACKET_SIZE
#define CDC_ECM_DATA_HS_OUT_PACKET_SIZE                 CDC_ECM_DATA_HS_MAX_PACKET_SIZE

#define CDC_ECM_DATA_FS_IN_PACKET_SIZE                  CDC_ECM_DATA_FS_MAX_PACKET_SIZE
#define CDC_ECM_DATA_FS_OUT_PACKET_SIZE                 CDC_ECM_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  CDC_ECM definitions                                                    */
/*---------------------------------------------------------------------*/
#define CDC_ECM_SEND_ENCAPSULATED_COMMAND                       0x00U
#define CDC_ECM_GET_ENCAPSULATED_RESPONSE                       0x01U
#define CDC_ECM_SET_ETH_MULTICAST_FILTERS                       0x40U
#define CDC_ECM_SET_ETH_PWRM_PATTERN_FILTER                     0x41U
#define CDC_ECM_GET_ETH_PWRM_PATTERN_FILTER                     0x42U
#define CDC_ECM_SET_ETH_PACKET_FILTER                           0x43U
#define CDC_ECM_GET_ETH_STATISTIC                               0x44U

#define CDC_ECM_NET_DISCONNECTED                                0x00U
#define CDC_ECM_NET_CONNECTED                                   0x01U


/* Ethernet statistics definitions */
#define CDC_ECM_XMIT_OK_VAL                                     CDC_ECM_ETH_STATS_VAL_ENABLED
#define CDC_ECM_XMIT_OK                                         0x01U
#define CDC_ECM_RVC_OK                                          0x02U
#define CDC_ECM_XMIT_ERROR                                      0x04U
#define CDC_ECM_RCV_ERROR                                       0x08U
#define CDC_ECM_RCV_NO_BUFFER                                   0x10U
#define CDC_ECM_DIRECTED_BYTES_XMIT                             0x20U
#define CDC_ECM_DIRECTED_FRAMES_XMIT                            0x40U
#define CDC_ECM_MULTICAST_BYTES_XMIT                            0x80U

#define CDC_ECM_MULTICAST_FRAMES_XMIT                           0x01U
#define CDC_ECM_BROADCAST_BYTES_XMIT                            0x02U
#define CDC_ECM_BROADCAST_FRAMES_XMIT                           0x04U
#define CDC_ECM_DIRECTED_BYTES_RCV                              0x08U
#define CDC_ECM_DIRECTED_FRAMES_RCV                             0x10U
#define CDC_ECM_MULTICAST_BYTES_RCV                             0x20U
#define CDC_ECM_MULTICAST_FRAMES_RCV                            0x40U
#define CDC_ECM_BROADCAST_BYTES_RCV                             0x80U

#define CDC_ECM_BROADCAST_FRAMES_RCV                            0x01U
#define CDC_ECM_RCV_CRC_ERROR                                   0x02U
#define CDC_ECM_TRANSMIT_QUEUE_LENGTH                           0x04U
#define CDC_ECM_RCV_ERROR_ALIGNMENT                             0x08U
#define CDC_ECM_XMIT_ONE_COLLISION                              0x10U
#define CDC_ECM_XMIT_MORE_COLLISIONS                            0x20U
#define CDC_ECM_XMIT_DEFERRED                                   0x40U
#define CDC_ECM_XMIT_MAX_COLLISIONS                             0x80U

#define CDC_ECM_RCV_OVERRUN                                     0x40U
#define CDC_ECM_XMIT_UNDERRUN                                   0x40U
#define CDC_ECM_XMIT_HEARTBEAT_FAILURE                          0x40U
#define CDC_ECM_XMIT_TIMES_CRS_LOST                             0x40U
#define CDC_ECM_XMIT_LATE_COLLISIONS                            0x40U

#define CDC_ECM_ETH_STATS_RESERVED                              0xE0U
#define CDC_ECM_BMREQUEST_TYPE_ECM                              0xA1U

/* MAC String index */
#define CDC_ECM_MAC_STRING_INDEX                                6U

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */

typedef struct
{
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
  int8_t (* Receive)(uint8_t *Buf, uint32_t *Len);
  int8_t (* TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
  int8_t (* Process)(USBD_HandleTypeDef *pdev);
  const uint8_t *pStrDesc;
} USBD_CDC_ECM_ItfTypeDef;

typedef struct
{
  uint8_t   bmRequest;
  uint8_t   bRequest;
  uint16_t  wValue;
  uint16_t  wIndex;
  uint16_t  wLength;
  uint8_t   data[8];
} USBD_CDC_ECM_NotifTypeDef;

typedef struct
{
  uint32_t data[2000 / 4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  Reserved1;  /* Reserved Byte to force 4 bytes alignment of following fields */
  uint8_t  Reserved2;  /* Reserved Byte to force 4 bytes alignment of following fields */
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;

  __IO uint32_t  MaxPcktLen;
  __IO uint32_t  LinkStatus;
  __IO uint32_t  NotificationStatus;
  USBD_CDC_ECM_NotifTypeDef   Req;
} USBD_CDC_ECM_HandleTypeDef;

typedef enum
{
  NETWORK_CONNECTION = 0x00,
  RESPONSE_AVAILABLE = 0x01,
  CONNECTION_SPEED_CHANGE = 0x2A
} USBD_CDC_ECM_NotifCodeTypeDef;

/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_CDC_ECM;
#define USBD_CDC_ECM_CLASS &USBD_CDC_ECM
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_CDC_ECM_RegisterInterface(USBD_HandleTypeDef *pdev,
                                        USBD_CDC_ECM_ItfTypeDef *fops);

uint8_t  USBD_CDC_ECM_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff,
                                  uint32_t length);

uint8_t  USBD_CDC_ECM_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff);

uint8_t  USBD_CDC_ECM_ReceivePacket(USBD_HandleTypeDef *pdev);

uint8_t  USBD_CDC_ECM_TransmitPacket(USBD_HandleTypeDef *pdev);

uint8_t  USBD_CDC_ECM_SendNotification(USBD_HandleTypeDef *pdev,
                                       USBD_CDC_ECM_NotifCodeTypeDef  Notif,
                                       uint16_t bVal, uint8_t *pData);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_CDC_ECM_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
