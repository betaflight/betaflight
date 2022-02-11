/**
  ******************************************************************************
  * @file    usbd_video.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_video.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#ifndef __USBD_VIDEO_H
#define __USBD_VIDEO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_VIDEO
  * @brief This file is the Header file for usbd_video.c
  * @{
  */


/** @defgroup usbd_VIDEO_Exported_Defines
  * @{
  */

/* USB Video device class specification version 1.10 */
#ifdef UVC_1_0
#define UVC_VERSION                                   0x0100U      /* UVC 1.0 */
#else
#define UVC_VERSION                                   0x0110U      /* UVC 1.1 */
#endif

/* bEndpointAddress in Endpoint Descriptor */
#ifndef UVC_IN_EP
#define UVC_IN_EP                                     0x81U
#endif /* VIDEO_IN_EP */

/* These defines shall be updated in the usbd_conf.h file */
#ifndef UVC_WIDTH
#define UVC_WIDTH                                     400U
#endif /* UVC_WIDTH */

#ifndef UVC_HEIGHT
#define UVC_HEIGHT                                    240U
#endif /* UVC_HEIGHT */

#ifndef UVC_CAM_FPS_FS
#define UVC_CAM_FPS_FS                                10U
#endif /* UVC_CAM_FPS_FS */

#ifndef UVC_CAM_FPS_HS
#define UVC_CAM_FPS_HS                                5U
#endif /* UVC_CAM_FPS_HS */

#ifndef UVC_PACKET_SIZE
#define UVC_PACKET_SIZE                               512U
#endif /* UVC_PACKET_SIZE */

#ifndef UVC_MAX_FRAME_SIZE
#define UVC_MAX_FRAME_SIZE                            (UVC_WIDTH * UVC_HEIGHT * 16U / 2U)
#endif /* UVC_MAX_FRAME_SIZE */

#ifndef UVC_COLOR_PRIMARIE
#define UVC_COLOR_PRIMARIE                            0x01U
#endif /* UVC_COLOR_PRIMARIE */

#ifndef UVC_TFR_CHARACTERISTICS
#define UVC_TFR_CHARACTERISTICS                       0x01U
#endif /* UVC_TFR_CHARACTERISTICS */

#ifndef UVC_MATRIX_COEFFICIENTS
#define UVC_MATRIX_COEFFICIENTS                       0x04U
#endif /* UVC_MATRIX_COEFFICIENTS */

#ifndef UVC_BITS_PER_PIXEL
#define UVC_BITS_PER_PIXEL                            12U
#endif /* UVC_BITS_PER_PIXEL */

#define UVC_GUID_YUY2                                 0x32595559U
#define UVC_GUID_NV12                                 0x3231564EU

#ifndef UVC_UNCOMPRESSED_GUID
#define UVC_UNCOMPRESSED_GUID                         UVC_GUID_NV12
#endif /* UVC_UNCOMPRESSED_GUID */

#define UVC_INTERVAL(n)                               (10000000U/(n))

#define UVC_MIN_BIT_RATE(n)                           (UVC_WIDTH * UVC_HEIGHT * 16U * (n)) /* 16 bit */
#define UVC_MAX_BIT_RATE(n)                           (UVC_WIDTH * UVC_HEIGHT * 16U * (n)) /* 16 bit */

#define UVC_PACKETS_IN_FRAME(n)                       (UVC_MAX_FRAME_SIZE / (n))

#ifndef UVC_ISO_FS_MPS
#define UVC_ISO_FS_MPS                                256U
#endif

#ifndef UVC_ISO_HS_MPS
#define UVC_ISO_HS_MPS                                512U
#endif

#ifndef UVC_HEADER_PACKET_CNT
#define UVC_HEADER_PACKET_CNT                         0x01U
#endif


#define UVC_REQ_READ_MASK                             0x80U
#define UVC_VC_IF_NUM                                 0x00U
#define UVC_VS_IF_NUM                                 0x01U
#define UVC_TOTAL_IF_NUM                              0x02U

#ifdef USBD_UVC_FORMAT_UNCOMPRESSED
#define UVC_CONFIG_DESC_SIZ                           (0x88U + 0x16U)
#else
#define UVC_CONFIG_DESC_SIZ                           0x88U
#endif

#define UVC_TOTAL_BUF_SIZE                            0x04U

#define UVC_VC_EP_DESC_SIZE                           0x05U
#define UVC_STREAMING_EP_DESC_SIZE                    0x07U
#define UVC_EP_DESC_TYPE                              0x25U

/* Video Interface Class Codes*/
#define UVC_CC_VIDEO                                  0x0EU

#define UVC_PLAY_STATUS_STOP                          0x00U
#define UVC_PLAY_STATUS_READY                         0x01U
#define UVC_PLAY_STATUS_STREAMING                     0x02U

#ifndef WBVAL
#define WBVAL(x) ((x) & 0xFFU),(((x) >> 8) & 0xFFU)
#endif
#ifndef DBVAL
#define DBVAL(x) ((x)& 0xFFU),(((x) >> 8) & 0xFFU),(((x)>> 16) & 0xFFU),(((x) >> 24) & 0xFFU)
#endif

/* Video Interface Protocol Codes */
#define PC_PROTOCOL_UNDEFINED                         0x00U

#define VIDEO_VC_IF_HEADER_DESC_SIZE                  0x0DU
#define VIDEO_IN_TERMINAL_DESC_SIZE                   0x08U
#define VIDEO_OUT_TERMINAL_DESC_SIZE                  0x09U
#define VIDEO_VS_IF_IN_HEADER_DESC_SIZE               0x0EU

#define VS_FORMAT_UNCOMPRESSED_DESC_SIZE              0x1BU
#define VS_FORMAT_MJPEG_DESC_SIZE                     0x0BU
#define VS_FRAME_DESC_SIZE                            0x1EU
#define VS_COLOR_MATCHING_DESC_SIZE                   0x06U

#ifdef USBD_UVC_FORMAT_UNCOMPRESSED
#define VS_FORMAT_DESC_SIZE                           VS_FORMAT_UNCOMPRESSED_DESC_SIZE
#define VS_FORMAT_SUBTYPE                             VS_FORMAT_UNCOMPRESSED
#define VS_FRAME_SUBTYPE                              VS_FRAME_UNCOMPRESSED

#define VC_HEADER_SIZE (VIDEO_VS_IF_IN_HEADER_DESC_SIZE + \
                        VS_FORMAT_UNCOMPRESSED_DESC_SIZE + \
                        VS_FRAME_DESC_SIZE + \
                        VS_COLOR_MATCHING_DESC_SIZE)
#else
#define VS_FORMAT_DESC_SIZE                           VS_FORMAT_MJPEG_DESC_SIZE
#define VS_FORMAT_SUBTYPE                             VS_FORMAT_MJPEG
#define VS_FRAME_SUBTYPE                              VS_FRAME_MJPEG

#define VC_HEADER_SIZE (VIDEO_VS_IF_IN_HEADER_DESC_SIZE + \
                        VS_FORMAT_DESC_SIZE + \
                        VS_FRAME_DESC_SIZE)
#endif

/*
 * Video Class specification release 1.1
 * Appendix A. Video Device Class Codes defines
 */

/* Video Interface Subclass values */
#define SC_UNDEFINED                                  0x00U
#define SC_VIDEOCONTROL                               0x01U
#define SC_VIDEOSTREAMING                             0x02U
#define SC_VIDEO_INTERFACE_COLLECTION                 0x03U

/* Video Class-Specific Descriptor Types */
#define CS_UNDEFINED                                  0x20U
#define CS_DEVICE                                     0x21U
#define CS_CONFIGURATION                              0x22U
#define CS_STRING                                     0x23U
#define CS_INTERFACE                                  0x24U
#define CS_ENDPOINT                                   0x25U

/* Video Class-Specific VideoControl Interface Descriptor Subtypes */
#define VC_DESCRIPTOR_UNDEFINED                       0x00U
#define VC_HEADER                                     0x01U
#define VC_INPUT_TERMINAL                             0x02U
#define VC_OUTPUT_TERMINAL                            0x03U
#define VC_SELECTOR_UNIT                              0x04U
#define VC_PROCESSING_UNIT                            0x05U
#define VC_EXTENSION_UNIT                             0x06U

/* Video Class-Specific VideoStreaming Interface Descriptor Subtypes */
#define VS_UNDEFINED                                  0x00U
#define VS_INPUT_HEADER                               0x01U
#define VS_OUTPUT_HEADER                              0x02U
#define VS_STILL_IMAGE_FRAME                          0x03U
#define VS_FORMAT_UNCOMPRESSED                        0x04U
#define VS_FRAME_UNCOMPRESSED                         0x05U
#define VS_FORMAT_MJPEG                               0x06U
#define VS_FRAME_MJPEG                                0x07U
#define VS_FORMAT_MPEG2TS                             0x0AU
#define VS_FORMAT_DV                                  0x0CU
#define VS_COLORFORMAT                                0x0DU
#define VS_FORMAT_FRAME_BASED                         0x10U
#define VS_FRAME_FRAME_BASED                          0x11U
#define VS_FORMAT_STREAM_BASED                        0x12U

/* Video Class-Specific Request values */
#define UVC_RQ_UNDEFINED                               0x00U
#define UVC_SET_CUR                                    0x01U
#define UVC_GET_CUR                                    0x81U
#define UVC_GET_MIN                                    0x82U
#define UVC_GET_MAX                                    0x83U
#define UVC_GET_RES                                    0x84U
#define UVC_GET_LEN                                    0x85U
#define UVC_GET_INFO                                   0x86U
#define UVC_GET_DEF                                    0x87U

/* VideoControl Interface Control Selectors */
#define VC_CONTROL_UNDEFINED                           0x00U
#define VC_VIDEO_POWER_MODE_CONTROL                    0x01U
#define VC_REQUEST_ERROR_CODE_CONTROL                  0x02U

/* Request Error Code Control */
#define UVC_NO_ERROR_ERR                               0x00U
#define UVC_NOT_READY_ERR                              0x01U
#define UVC_WRONG_STATE_ERR                            0x02U
#define UVC_POWER_ERR                                  0x03U
#define UVC_OUT_OF_RANGE_ERR                           0x04U
#define UVC_INVALID_UNIT_ERR                           0x05U
#define UVC_INVALID_CONTROL_ERR                        0x06U
#define UVC_INVALID_REQUEST_ERR                        0x07U
#define UVC_UNKNOWN_ERR                                0xFFU

/*Terminal Control Selectors*/
#define TE_CONTROL_UNDEFINED                           0x00U

/* Selector Unit Control Selectors */
#define SU_CONTROL_UNDEFINED                           0x00U
#define SU_INPUT_SELECT_CONTROL                        0x01U

/* Camera Terminal Control Selectors */
#define CT_CONTROL_UNDEFINED                           0x00U
#define CT_SCANNING_MODE_CONTROL                       0x01U
#define CT_AE_MODE_CONTROL                             0x02U
#define CT_AE_PRIORITY_CONTROL                         0x03U
#define CT_EXPOSURE_TIME_ABSOLUTE_CONTROL              0x04U
#define CT_EXPOSURE_TIME_RELATIVE_CONTROL              0x05U
#define CT_FOCUS_ABSOLUTE_CONTROL                      0x06U
#define CT_FOCUS_RELATIVE_CONTROL                      0x07U
#define CT_FOCUS_AUTO_CONTROL                          0x08U
#define CT_IRIS_ABSOLUTE_CONTROL                       0x09U
#define CT_IRIS_RELATIVE_CONTROL                       0x0AU
#define CT_ZOOM_ABSOLUTE_CONTROL                       0x0BU
#define CT_ZOOM_RELATIVE_CONTROL                       0x0CU
#define CT_PANTILT_ABSOLUTE_CONTROL                    0x0DU
#define CT_PANTILT_RELATIVE_CONTROL                    0x0EU
#define CT_ROLL_ABSOLUTE_CONTROL                       0x0FU
#define CT_ROLL_RELATIVE_CONTROL                       0x10U
#define CT_PRIVACY_CONTROL                             0x11U

/* Processing Unit Control Selectors */
#define PU_CONTROL_UNDEFINED                           0x00U
#define PU_BACKLIGHT_COMPENSATION_CONTROL              0x01U
#define PU_BRIGHTNESS_CONTROL                          0x02U
#define PU_CONTRAST_CONTROL                            0x03U
#define PU_GAIN_CONTROL                                0x04U
#define PU_POWER_LINE_FREQUENCY_CONTROL                0x05U
#define PU_HUE_CONTROL                                 0x06U
#define PU_SATURATION_CONTROL                          0x07U
#define PU_SHARPNESS_CONTROL                           0x08U
#define PU_GAMMA_CONTROL                               0x09U
#define PU_WHITE_BALANCE_TEMPERATURE_CONTROL           0x0AU
#define PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL      0x0BU
#define PU_WHITE_BALANCE_COMPONENT_CONTROL             0x0CU
#define PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL        0x0DU
#define PU_DIGITAL_MULTIPLIER_CONTROL                  0x0EU
#define PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL            0x0FU
#define PU_HUE_AUTO_CONTROL                            0x10U
#define PU_ANALOG_VIDEO_STANDARD_CONTROL               0x11U
#define PU_ANALOG_LOCK_STATUS_CONTROL                  0x12U

/*Extension Unit Control Selectors */
#define XU_CONTROL_UNDEFINED                           0x00U

/* VideoStreaming Interface Control Selectors */
#define VS_CONTROL_UNDEFINED                           0x00U
#define VS_PROBE_CONTROL                               0x100U
#define VS_COMMIT_CONTROL                              0x200U
#define VS_STILL_PROBE_CONTROL                         0x03U
#define VS_STILL_COMMIT_CONTROL                        0x04U
#define VS_STILL_IMAGE_TRIGGER_CONTROL                 0x05U
#define VS_STREAM_ERROR_CODE_CONTROL                   0x06U
#define VS_GENERATE_KEY_FRAME_CONTROL                  0x07U
#define VS_UPDATE_FRAME_SEGMENT_CONTROL                0x08U
#define VS_SYNC_DELAY_CONTROL                          0x09U


/* Control Capabilities */
#define UVC_SUPPORTS_GET                               0x01U
#define UVC_SUPPORTS_SET                               0x02U
#define UVC_STATE_DISABLED                             0x04U
#define UVC_AUTOUPDATE_CONTROL                         0x08U
#define UVC_ASYNCHRONOUS_CONTROL                       0x10U

/* USB Terminal Types */
#define TT_VENDOR_SPECIFIC                             0x0100U
#define TT_STREAMING                                   0x0101U

/* Input Terminal Types */
#define ITT_VENDOR_SPECIFIC                            0x0200U
#define ITT_CAMERA                                     0x0201U
#define ITT_MEDIA_TRANSPORT_INPUT                      0x0202U

/*Output Terminal Types */
#define OTT_VENDOR_SPECIFIC                            0x0300U
#define OTT_DISPLAY                                    0x0301U
#define OTT_MEDIA_TRANSPORT_OUTPUT                     0x0302U

/* External Terminal Types */
#define EXTERNAL_VENDOR_SPECIFIC                       0x0400U
#define COMPOSITE_CONNECTOR                            0x0401U
#define SVIDEO_CONNECTOR                               0x0402U
#define COMPONENT_CONNECTOR                            0x0403U


/* VIDEO Commands enumeration */
typedef enum
{
  VIDEO_CMD_START = 1U,
  VIDEO_CMD_PLAY,
  VIDEO_CMD_STOP,
} VIDEO_CMD_TypeDef;

typedef enum
{
  VIDEO_OFFSET_NONE = 0U,
  VIDEO_OFFSET_HALF,
  VIDEO_OFFSET_FULL,
  VIDEO_OFFSET_UNKNOWN,
} VIDEO_OffsetTypeDef;

typedef  struct  _VIDEO_DescHeader
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bDescriptorSubType;
} USBD_VIDEO_DescHeader_t;

typedef struct
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bDescriptorSubType;
  uint8_t  bFrameIndex;
  uint8_t  bmCapabilities;
  uint16_t wWidth;
  uint16_t wHeight;
  uint32_t dwMinBitRate;
  uint32_t dwMaxBitRate;
  uint32_t dwMaxVideoFrameBufSize;
  uint32_t dwDefaultFrameInterval;
  uint8_t  bFrameIntervalType;
  uint32_t dwMinFrameInterval;
  uint32_t dwMaxFrameInterval;
  uint32_t dwFrameIntervalStep;
} __PACKED USBD_VIDEO_VSFrameDescTypeDef;

typedef struct
{
  uint8_t cmd;
  uint8_t data[USB_MAX_EP0_SIZE];
  uint8_t len;
  uint8_t unit;
} USBD_VIDEO_ControlTypeDef;

typedef struct
{
  uint32_t                   interface;
  uint32_t                   uvc_state;
  uint8_t                    buffer[UVC_TOTAL_BUF_SIZE];
  VIDEO_OffsetTypeDef        offset;
  USBD_VIDEO_ControlTypeDef  control;
} USBD_VIDEO_HandleTypeDef;

typedef struct
{
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* Control)(uint8_t, uint8_t *, uint16_t);
  int8_t (* Data)(uint8_t **, uint16_t *, uint16_t *);
  uint8_t  *pStrDesc;
} USBD_VIDEO_ItfTypeDef;

/* UVC uses only 26 first bytes */
typedef struct
{
  uint16_t    bmHint;
  uint8_t     bFormatIndex;
  uint8_t     bFrameIndex;
  uint32_t    dwFrameInterval;
  uint16_t    wKeyFrameRate;
  uint16_t    wPFrameRate;
  uint16_t    wCompQuality;
  uint16_t    wCompWindowSize;
  uint16_t    wDelay;
  uint32_t    dwMaxVideoFrameSize;
  uint32_t    dwMaxPayloadTransferSize;
  uint32_t    dwClockFrequency;
  uint8_t     bmFramingInfo;
  uint8_t     bPreferedVersion;
  uint8_t     bMinVersion;
  uint8_t     bMaxVersion;
} __PACKED USBD_VideoControlTypeDef;

extern USBD_ClassTypeDef    USBD_VIDEO;
#define USBD_VIDEO_CLASS    &USBD_VIDEO
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */

uint8_t USBD_VIDEO_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_VIDEO_ItfTypeDef *fops);

/**
  * @}
  */


#endif /* _USBD_VIDEO_H_ */
