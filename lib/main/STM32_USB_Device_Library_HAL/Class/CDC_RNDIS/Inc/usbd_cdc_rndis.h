/**
  ******************************************************************************
  * @file    usbd_cdc_rndis.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_cdc_rndis.c file.
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
#ifndef __USB_CDC_RNDIS_H
#define __USB_CDC_RNDIS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup usbd_cdc_rndis
  * @brief This file is the Header file for usbd_cdc_rndis.c
  * @{
  */


/** @defgroup usbd_cdc_rndis_Exported_Defines
  * @{
  */

#define CDC_RNDIS_IN_EP                                   0x81U  /* EP1 for data IN */
#define CDC_RNDIS_OUT_EP                                  0x01U  /* EP1 for data OUT */
#define CDC_RNDIS_CMD_EP                                  0x82U  /* EP2 for CDC_RNDIS commands */

#ifndef CDC_RNDIS_CMD_ITF_NBR
#define CDC_RNDIS_CMD_ITF_NBR                             0x00U /* Command Interface Number 0 */
#endif /* CDC_RNDIS_CMD_ITF_NBR */

#ifndef CDC_RNDIS_COM_ITF_NBR
#define CDC_RNDIS_COM_ITF_NBR                             0x01U /* Communication Interface Number 0 */
#endif /* CDC_RNDIS_CMD_ITF_NBR */

#ifndef CDC_RNDIS_HS_BINTERVAL
#define CDC_RNDIS_HS_BINTERVAL                            0x10U
#endif /* CDC_RNDIS_HS_BINTERVAL */

#ifndef CDC_RNDIS_FS_BINTERVAL
#define CDC_RNDIS_FS_BINTERVAL                            0x10U
#endif /* CDC_RNDIS_FS_BINTERVAL */


/* CDC_RNDIS Endpoints parameters: you can fine tune these values
   depending on the needed baudrates and performance. */
#define CDC_RNDIS_DATA_HS_MAX_PACKET_SIZE                 512U  /* Endpoint IN & OUT Packet size */
#define CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE                 64U   /* Endpoint IN & OUT Packet size */
#define CDC_RNDIS_CMD_PACKET_SIZE                         16U   /* Control Endpoint Packet size */

#define CDC_RNDIS_CONFIG_DESC_SIZ                         75U
#define CDC_RNDIS_DATA_HS_IN_PACKET_SIZE                  CDC_RNDIS_DATA_HS_MAX_PACKET_SIZE
#define CDC_RNDIS_DATA_HS_OUT_PACKET_SIZE                 CDC_RNDIS_DATA_HS_MAX_PACKET_SIZE

#define CDC_RNDIS_DATA_FS_IN_PACKET_SIZE                  CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE
#define CDC_RNDIS_DATA_FS_OUT_PACKET_SIZE                 CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  CDC_RNDIS definitions                                                    */
/*---------------------------------------------------------------------*/

/** Implemented CDC_RNDIS Version Major */
#define CDC_RNDIS_VERSION_MAJOR                     0x01U

/* Implemented CDC_RNDIS Version Minor */
#define CDC_RNDIS_VERSION_MINOR                     0x00U

/* Maximum size in bytes of a CDC_RNDIS control message
   which can be sent or received */
#define CDC_RNDIS_MESSAGE_BUFFER_SIZE               128U

/* Maximum size in bytes of an Ethernet frame
   according to the Ethernet standard */
#define CDC_RNDIS_ETH_FRAME_SIZE_MAX                1536U

/* Maximum size allocated for buffer
   inside Query messages structures */
#define CDC_RNDIS_MAX_INFO_BUFF_SZ                  200U
#define CDC_RNDIS_MAX_DATA_SZE                      2000U

/* Notification request value for a CDC_RNDIS
   Response Available notification */
#define CDC_RNDIS_NOTIFICATION_RESP_AVAILABLE       0x00000001UL


#define CDC_RNDIS_PACKET_MSG_ID                     0x00000001UL
#define CDC_RNDIS_INITIALIZE_MSG_ID                 0x00000002UL
#define CDC_RNDIS_HALT_MSG_ID                       0x00000003UL
#define CDC_RNDIS_QUERY_MSG_ID                      0x00000004UL
#define CDC_RNDIS_SET_MSG_ID                        0x00000005UL
#define CDC_RNDIS_RESET_MSG_ID                      0x00000006UL
#define CDC_RNDIS_INDICATE_STATUS_MSG_ID            0x00000007UL
#define CDC_RNDIS_KEEPALIVE_MSG_ID                  0x00000008UL

#define CDC_RNDIS_INITIALIZE_CMPLT_ID               0x80000002UL
#define CDC_RNDIS_QUERY_CMPLT_ID                    0x80000004UL
#define CDC_RNDIS_SET_CMPLT_ID                      0x80000005UL
#define CDC_RNDIS_RESET_CMPLT_ID                    0x80000006UL
#define CDC_RNDIS_KEEPALIVE_CMPLT_ID                0x80000008UL

#define CDC_RNDIS_STATUS_SUCCESS                    0x00000000UL
#define CDC_RNDIS_STATUS_FAILURE                    0xC0000001UL
#define CDC_RNDIS_STATUS_INVALID_DATA               0xC0010015UL
#define CDC_RNDIS_STATUS_NOT_SUPPORTED              0xC00000BBUL
#define CDC_RNDIS_STATUS_MEDIA_CONNECT              0x4001000BUL
#define CDC_RNDIS_STATUS_MEDIA_DISCONNECT           0x4001000CUL
/** Media state */
#define CDC_RNDIS_MEDIA_STATE_CONNECTED             0x00000000UL
#define CDC_RNDIS_MEDIA_STATE_DISCONNECTED          0x00000001UL

/** Media types */
#define CDC_RNDIS_MEDIUM_802_3                      0x00000000UL

#define CDC_RNDIS_DF_CONNECTIONLESS                 0x00000001UL
#define CDC_RNDIS_DF_CONNECTION_ORIENTED            0x00000002UL

/** Hardware status of the underlying NIC */
#define CDC_RNDIS_HW_STS_READY                      0x00000000UL
#define CDC_RNDIS_HW_STS_INITIALIZING               0x00000001UL
#define CDC_RNDIS_HW_STS_RESET                      0x00000002UL
#define CDC_RNDIS_HW_STS_CLOSING                    0x00000003UL
#define CDC_RNDIS_HW_STS_NOT_READY                  0x00000004UL

/** Packet filter */
#define CDC_RNDIS_PACKET_DIRECTED                   0x00000001UL
#define CDC_RNDIS_PACKET_MULTICAST                  0x00000002UL
#define CDC_RNDIS_PACKET_ALL_MULTICAST              0x00000004UL
#define CDC_RNDIS_PACKET_BROADCAST                  0x00000008UL
#define CDC_RNDIS_PACKET_SOURCE_ROUTING             0x00000010UL
#define CDC_RNDIS_PACKET_PROMISCUOUS                0x00000020UL
#define CDC_RNDIS_PACKET_SMT                        0x00000040UL
#define CDC_RNDIS_PACKET_ALL_LOCAL                  0x00000080UL
#define CDC_RNDIS_PACKET_GROUP                      0x00001000UL
#define CDC_RNDIS_PACKET_ALL_FUNCTIONAL             0x00002000UL
#define CDC_RNDIS_PACKET_FUNCTIONAL                 0x00004000UL
#define CDC_RNDIS_PACKET_MAC_FRAME                  0x00008000UL

#define OID_GEN_SUPPORTED_LIST                      0x00010101UL
#define OID_GEN_HARDWARE_STATUS                     0x00010102UL
#define OID_GEN_MEDIA_SUPPORTED                     0x00010103UL
#define OID_GEN_MEDIA_IN_USE                        0x00010104UL
#define OID_GEN_MAXIMUM_FRAME_SIZE                  0x00010106UL
#define OID_GEN_MAXIMUM_TOTAL_SIZE                  0x00010111UL
#define OID_GEN_LINK_SPEED                          0x00010107UL
#define OID_GEN_TRANSMIT_BLOCK_SIZE                 0x0001010AUL
#define OID_GEN_RECEIVE_BLOCK_SIZE                  0x0001010BUL
#define OID_GEN_VENDOR_ID                           0x0001010CUL
#define OID_GEN_VENDOR_DESCRIPTION                  0x0001010DUL
#define OID_GEN_CURRENT_PACKET_FILTER               0x0001010EUL
#define OID_GEN_MEDIA_CONNECT_STATUS                0x00010114UL
#define OID_GEN_MAXIMUM_SEND_PACKETS                0x00010115UL
#define OID_GEN_PHYSICAL_MEDIUM                     0x00010202UL
#define OID_GEN_XMIT_OK                             0x00020101UL
#define OID_GEN_RCV_OK                              0x00020102UL
#define OID_GEN_XMIT_ERROR                          0x00020103UL
#define OID_GEN_RCV_ERROR                           0x00020104UL
#define OID_GEN_RCV_NO_BUFFER                       0x00020105UL
#define OID_GEN_CDC_RNDIS_CONFIG_PARAMETER          0x0001021BUL
#define OID_802_3_PERMANENT_ADDRESS                 0x01010101UL
#define OID_802_3_CURRENT_ADDRESS                   0x01010102UL
#define OID_802_3_MULTICAST_LIST                    0x01010103UL
#define OID_802_3_MAXIMUM_LIST_SIZE                 0x01010104UL
#define OID_802_3_RCV_ERROR_ALIGNMENT               0x01020101UL
#define OID_802_3_XMIT_ONE_COLLISION                0x01020102UL
#define OID_802_3_XMIT_MORE_COLLISIONS              0x01020103UL


#define CDC_RNDIS_SEND_ENCAPSULATED_COMMAND         0x00U
#define CDC_RNDIS_GET_ENCAPSULATED_RESPONSE         0x01U

#define CDC_RNDIS_NET_DISCONNECTED                  0x00U
#define CDC_RNDIS_NET_CONNECTED                     0x01U

#define CDC_RNDIS_BMREQUEST_TYPE_RNDIS              0xA1U
#define CDC_RNDIS_PCKTMSG_DATAOFFSET_OFFSET         8U

/* MAC String index */
#define CDC_RNDIS_MAC_STRING_INDEX                  6U

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */

typedef struct _USBD_CDC_RNDIS_Itf
{
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
  int8_t (* Receive)(uint8_t *Buf, uint32_t *Len);
  int8_t (* TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
  int8_t (* Process)(USBD_HandleTypeDef *pdev);
  uint8_t *pStrDesc;
} USBD_CDC_RNDIS_ItfTypeDef;

/* CDC_RNDIS State values */
typedef enum
{
  CDC_RNDIS_STATE_UNINITIALIZED    = 0,
  CDC_RNDIS_STATE_BUS_INITIALIZED  = 1,
  CDC_RNDIS_STATE_INITIALIZED      = 2,
  CDC_RNDIS_STATE_DATA_INITIALIZED = 3
} USBD_CDC_RNDIS_StateTypeDef;

typedef struct
{
  uint8_t   bmRequest;
  uint8_t   bRequest;
  uint16_t  wValue;
  uint16_t  wIndex;
  uint16_t  wLength;
  uint8_t   data[8];
} USBD_CDC_RNDIS_NotifTypeDef;

typedef struct
{
  uint32_t        data[2000 / 4]; /* Force 32bits alignment */
  uint8_t         CmdOpCode;
  uint8_t         CmdLength;
  uint8_t         ResponseRdy;     /* Indicates if the Device Response to an CDC_RNDIS msg is ready */
  uint8_t         Reserved1;       /* Reserved Byte to force 4 bytes alignment of following fields */
  uint8_t         *RxBuffer;
  uint8_t         *TxBuffer;
  uint32_t        RxLength;
  uint32_t        TxLength;

  USBD_CDC_RNDIS_NotifTypeDef     Req;
  USBD_CDC_RNDIS_StateTypeDef     State;

  __IO uint32_t   TxState;
  __IO uint32_t   RxState;

  __IO uint32_t   MaxPcktLen;
  __IO uint32_t   LinkStatus;
  __IO uint32_t   NotificationStatus;
  __IO uint32_t   PacketFilter;
} USBD_CDC_RNDIS_HandleTypeDef;


typedef enum
{
  NETWORK_CONNECTION = 0x00,
  RESPONSE_AVAILABLE = 0x01,
  CONNECTION_SPEED_CHANGE = 0x2A
} USBD_CDC_RNDIS_NotifCodeTypeDef;


/* Messages Sent by the Host ---------------------*/

/* Type define for a CDC_RNDIS Initialize command message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t ReqId;
  uint32_t MajorVersion;
  uint32_t MinorVersion;
  uint32_t MaxTransferSize;
} USBD_CDC_RNDIS_InitMsgTypeDef;

/* Type define for a CDC_RNDIS Halt Message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t ReqId;
} USBD_CDC_RNDIS_HaltMsgTypeDef;

/* Type define for a CDC_RNDIS Query command message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t RequestId;
  uint32_t Oid;
  uint32_t InfoBufLength;
  uint32_t InfoBufOffset;
  uint32_t DeviceVcHandle;
  uint32_t InfoBuf[CDC_RNDIS_MAX_INFO_BUFF_SZ];
} USBD_CDC_RNDIS_QueryMsgTypeDef;

/* Type define for a CDC_RNDIS Set command message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t ReqId;
  uint32_t Oid;
  uint32_t InfoBufLength;
  uint32_t InfoBufOffset;
  uint32_t DeviceVcHandle;
  uint32_t InfoBuf[CDC_RNDIS_MAX_INFO_BUFF_SZ];
} USBD_CDC_RNDIS_SetMsgTypeDef;

/* Type define for a CDC_RNDIS Reset message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t Reserved;
} USBD_CDC_RNDIS_ResetMsgTypeDef;

/* Type define for a CDC_RNDIS Keepalive command message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t ReqId;
} USBD_CDC_RNDIS_KpAliveMsgTypeDef;


/* Messages Sent by the Device ---------------------*/

/* Type define for a CDC_RNDIS Initialize complete response message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t ReqId;
  uint32_t Status;
  uint32_t MajorVersion;
  uint32_t MinorVersion;
  uint32_t DeviceFlags;
  uint32_t Medium;
  uint32_t MaxPacketsPerTransfer;
  uint32_t MaxTransferSize;
  uint32_t PacketAlignmentFactor;
  uint32_t AFListOffset;
  uint32_t AFListSize;
} USBD_CDC_RNDIS_InitCpltMsgTypeDef;

/* Type define for a CDC_RNDIS Query complete response message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t ReqId;
  uint32_t Status;
  uint32_t InfoBufLength;
  uint32_t InfoBufOffset;
  uint32_t InfoBuf[CDC_RNDIS_MAX_INFO_BUFF_SZ];
} USBD_CDC_RNDIS_QueryCpltMsgTypeDef;

/* Type define for a CDC_RNDIS Set complete response message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t ReqId;
  uint32_t Status;
} USBD_CDC_RNDIS_SetCpltMsgTypeDef;

/* Type define for a CDC_RNDIS Reset complete message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t Status;
  uint32_t AddrReset;
} USBD_CDC_RNDIS_ResetCpltMsgTypeDef;

/* Type define for CDC_RNDIS struct to indicate a change
   in the status of the device */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t Status;
  uint32_t StsBufLength;
  uint32_t StsBufOffset;
} USBD_CDC_RNDIS_StsChangeMsgTypeDef;

/* Type define for a CDC_RNDIS Keepalive complete message */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t ReqId;
  uint32_t Status;
} USBD_CDC_RNDIS_KpAliveCpltMsgTypeDef;


/* Messages Sent by both Host and Device ---------------------*/

/* Type define for a CDC_RNDIS packet message, used to encapsulate
   Ethernet packets sent to and from the adapter */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t DataOffset;
  uint32_t DataLength;
  uint32_t OOBDataOffset;
  uint32_t OOBDataLength;
  uint32_t NumOOBDataElements;
  uint32_t PerPacketInfoOffset;
  uint32_t PerPacketInfoLength;
  uint32_t VcHandle;
  uint32_t Reserved;
} USBD_CDC_RNDIS_PacketMsgTypeDef;

/* Miscellaneous types used for parsing ---------------------*/

/* The common part for all CDC_RNDIS messages Complete response */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
  uint32_t ReqId;
  uint32_t Status;
} USBD_CDC_RNDIS_CommonCpltMsgTypeDef;

/* Type define for a single parameter structure */
typedef struct
{
  uint32_t ParamNameOffset;
  uint32_t ParamNameLength;
  uint32_t ParamType;
  uint32_t ParamValueOffset;
  uint32_t ParamValueLength;
} USBD_CDC_RNDIS_ParamStructTypeDef;


/* Type define of a single CDC_RNDIS OOB data record */
typedef struct
{
  uint32_t Size;
  uint32_t Type;
  uint32_t ClassInfoType;
  uint32_t OOBData[sizeof(uint32_t)];
} USBD_CDC_RNDIS_OOBPacketTypeDef;

/* Type define for notification structure */
typedef struct
{
  uint32_t notification;
  uint32_t reserved;
} USBD_CDC_RNDIS_NotifStructTypeDef;

/* This structure will be used to store the type, the size and ID for any
   received message from the control endpoint */
typedef struct
{
  uint32_t MsgType;
  uint32_t MsgLength;
} USBD_CDC_RNDIS_CtrlMsgTypeDef;


/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef USBD_CDC_RNDIS;
#define USBD_CDC_RNDIS_CLASS &USBD_CDC_RNDIS
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_CDC_RNDIS_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff);
uint8_t USBD_CDC_RNDIS_ReceivePacket(USBD_HandleTypeDef *pdev);
uint8_t USBD_CDC_RNDIS_TransmitPacket(USBD_HandleTypeDef *pdev);

uint8_t USBD_CDC_RNDIS_RegisterInterface(USBD_HandleTypeDef *pdev,
                                         USBD_CDC_RNDIS_ItfTypeDef *fops);

uint8_t USBD_CDC_RNDIS_SetTxBuffer(USBD_HandleTypeDef *pdev,
                                   uint8_t *pbuff, uint32_t length);

uint8_t USBD_CDC_RNDIS_SendNotification(USBD_HandleTypeDef *pdev,
                                        USBD_CDC_RNDIS_NotifCodeTypeDef Notif,
                                        uint16_t bVal, uint8_t *pData);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_CDC_RNDIS_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
