/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   Video Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
/*                                                                        */
/*  COMPONENT DEFINITION                                   RELEASE        */
/*                                                                        */
/*    ux_device_class_video.h                             PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file contains all the header and extern functions used by the  */
/*    USBX device video class.                                            */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DEVICE_CLASS_VIDEO_H
#define UX_DEVICE_CLASS_VIDEO_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard
   C is used to process the API information.  */

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif

/* Define options.  */

#define UX_DEVICE_CLASS_VIDEO_THREAD_STACK_SIZE                                              UX_THREAD_STACK_SIZE

/* Define external static data.  */

extern UCHAR _ux_system_device_class_video_name[];


/* Define IOCTL code.
     ux_device_class_video_ioctl(video, IOCTL_CODE, parameter).
 */

#define UX_DEVICE_CLASS_VIDEO_IOCTL_GET_ARG                                                  0x01


/* Define Video Class main constants.  */

#define UX_DEVICE_CLASS_VIDEO_CLASS                                                          0x0e
#define UX_DEVICE_CLASS_VIDEO_SUBCLASS_UNDEFINED                                             0
#define UX_DEVICE_CLASS_VIDEO_SUBCLASS_CONTROL                                               1
#define UX_DEVICE_CLASS_VIDEO_SUBCLASS_STREAMING                                             2
#define UX_DEVICE_CLASS_VIDEO_SUBCLASS_INTERFACE_COLLECTION                                  3

/* Class Code (CC).  */
#define UX_DEVICE_CLASS_VIDEO_CC_VIDEO                                                       0x0e

/* Subclass Code (SC).  */
#define UX_DEVICE_CLASS_VIDEO_SC_UNDEFINED                                                   0
#define UX_DEVICE_CLASS_VIDEO_SC_CONTROL                                                     1
#define UX_DEVICE_CLASS_VIDEO_SC_STREAMING                                                   2
#define UX_DEVICE_CLASS_VIDEO_SC_INTERFACE_COLLECTION                                        3

/* Protocol Code.  */
#define UX_DEVICE_CLASS_VIDEO_PC_PROTOCOL_UNDEFINED                                          0
#define UX_DEVICE_CLASS_VIDEO_PC_PROTOCOL_15                                                 0


/* Define Video Class-Specific (CS) descriptor types.  */
#define UX_DEVICE_CLASS_VIDEO_CS_UNDEFINED                                                   0x20
#define UX_DEVICE_CLASS_VIDEO_CS_DEVICE                                                      0x21
#define UX_DEVICE_CLASS_VIDEO_CS_CONFIGURATION                                               0x22
#define UX_DEVICE_CLASS_VIDEO_CS_STRING                                                      0x23
#define UX_DEVICE_CLASS_VIDEO_CS_INTERFACE                                                   0x24
#define UX_DEVICE_CLASS_VIDEO_CS_ENDPOINT                                                    0x25


/* Define Video Class specific Video Control (VC) interface descriptor subtypes.  */
#define UX_DEVICE_CLASS_VIDEO_VC_DESCRIPTOR_UNDEFINED                                        0x00
#define UX_DEVICE_CLASS_VIDEO_VC_HEADER                                                      0x01
#define UX_DEVICE_CLASS_VIDEO_VC_INPUT_TERMINAL                                              0x02
#define UX_DEVICE_CLASS_VIDEO_VC_OUTPUT_TERMINAL                                             0x03
#define UX_DEVICE_CLASS_VIDEO_VC_SELECTOR_UNIT                                               0x04
#define UX_DEVICE_CLASS_VIDEO_VC_PROCESSING_UNIT                                             0x05
#define UX_DEVICE_CLASS_VIDEO_VC_EXTENSION_UNIT                                              0x06
#define UX_DEVICE_CLASS_VIDEO_VC_ENCODING_UNIT                                               0x07


/* Define Video Class specific Video Stream (VS) interface descriptor subtypes.  */
#define UX_DEVICE_CLASS_VIDEO_VS_UNDEFINED                                                   0x00
#define UX_DEVICE_CLASS_VIDEO_VS_INPUT_HEADER                                                0x01
#define UX_DEVICE_CLASS_VIDEO_VS_OUTPUT_HEADER                                               0x02
#define UX_DEVICE_CLASS_VIDEO_VS_STILL_IMAGE_FRAME                                           0x03
#define UX_DEVICE_CLASS_VIDEO_VS_FORMAT_UNCOMPRESSED                                         0x04
#define UX_DEVICE_CLASS_VIDEO_VS_FRAME_UNCOMPRESSED                                          0x05
#define UX_DEVICE_CLASS_VIDEO_VS_FORMAT_MJPEG                                                0x06
#define UX_DEVICE_CLASS_VIDEO_VS_FRAME_MJPEG                                                 0x07
#define UX_DEVICE_CLASS_VIDEO_VS_FORMAT_MPEG2TS                                              0x0A
#define UX_DEVICE_CLASS_VIDEO_VS_FORMAT_DV                                                   0x0C
#define UX_DEVICE_CLASS_VIDEO_VS_COLORFORMAT                                                 0x0D
#define UX_DEVICE_CLASS_VIDEO_VS_FORMAT_FRAME_BASED                                          0x10
#define UX_DEVICE_CLASS_VIDEO_VS_FRAME_FRAME_BASED                                           0x11
#define UX_DEVICE_CLASS_VIDEO_VS_FORMAT_STREAM_BASED                                         0x12
#define UX_DEVICE_CLASS_VIDEO_VS_FORMAT_H264                                                 0x13
#define UX_DEVICE_CLASS_VIDEO_VS_FRAME_H264                                                  0x14
#define UX_DEVICE_CLASS_VIDEO_VS_FORMAT_H264_SIMULCAST                                       0x15
#define UX_DEVICE_CLASS_VIDEO_VS_FORMAT_VP8                                                  0x16
#define UX_DEVICE_CLASS_VIDEO_VS_FRAME_VP8                                                   0x17
#define UX_DEVICE_CLASS_VIDEO_VS_FORMAT_VP8_SIMULCAST                                        0x18


/* Define Video Class specific Endpoint (EP) descriptor subtypes.  */
#define UX_DEVICE_CLASS_VIDEO_EP_UNDEFINED                                                   0x00
#define UX_DEVICE_CLASS_VIDEO_EP_GENERAL                                                     0x01
#define UX_DEVICE_CLASS_VIDEO_EP_ENDPOINT                                                    0x02
#define UX_DEVICE_CLASS_VIDEO_EP_INTERRUPT                                                   0x03


/* Define Video Control Selector Codes.  */

/* Define VideoControl (VC) Interface Control Selector Codes.  */
#define UX_DEVICE_CLASS_VIDEO_VC_CONTROL_UNDEFINED                                           0x00
#define UX_DEVICE_CLASS_VIDEO_VC_VIDEO_POWER_MODE_CONTROL                                    0x01
#define UX_DEVICE_CLASS_VIDEO_VC_REQUEST_ERROR_CODE_CONTROL                                  0x02

/* Define Terminal Control (TE) Selectors.  */
#define UX_DEVICE_CLASS_VIDEO_TE_CONTROL_UNDEFINED                                           0x00

/* Define Selector Unit (SU) Control Selectors.  */
#define UX_DEVICE_CLASS_VIDEO_SU_CONTROL_UNDEFINED                                           0x00
#define UX_DEVICE_CLASS_VIDEO_SU_INPUT_SELECT_CONTROL                                        0x01

/* Define Camera Terminal (CT) Control Selectors.  */
#define UX_DEVICE_CLASS_VIDEO_CT_CONTROL_UNDEFINED                                           0x00
#define UX_DEVICE_CLASS_VIDEO_CT_SCANNING_MODE_CONTROL                                       0x01
#define UX_DEVICE_CLASS_VIDEO_CT_AE_MODE_CONTROL                                             0x02
#define UX_DEVICE_CLASS_VIDEO_CT_AE_PRIORITY_CONTROL                                         0x03
#define UX_DEVICE_CLASS_VIDEO_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL                              0x04
#define UX_DEVICE_CLASS_VIDEO_CT_EXPOSURE_TIME_RELATIVE_CONTROL                              0x05
#define UX_DEVICE_CLASS_VIDEO_CT_FOCUS_ABSOLUTE_CONTROL                                      0x06
#define UX_DEVICE_CLASS_VIDEO_CT_FOCUS_RELATIVE_CONTROL                                      0x07
#define UX_DEVICE_CLASS_VIDEO_CT_FOCUS_AUTO_CONTROL                                          0x08
#define UX_DEVICE_CLASS_VIDEO_CT_IRIS_ABSOLUTE_CONTROL                                       0x09
#define UX_DEVICE_CLASS_VIDEO_CT_IRIS_RELATIVE_CONTROL                                       0x0A
#define UX_DEVICE_CLASS_VIDEO_CT_ZOOM_ABSOLUTE_CONTROL                                       0x0B
#define UX_DEVICE_CLASS_VIDEO_CT_ZOOM_RELATIVE_CONTROL                                       0x0C
#define UX_DEVICE_CLASS_VIDEO_CT_PANTILT_ABSOLUTE_CONTROL                                    0x0D
#define UX_DEVICE_CLASS_VIDEO_CT_PANTILT_RELATIVE_CONTROL                                    0x0E
#define UX_DEVICE_CLASS_VIDEO_CT_ROLL_ABSOLUTE_CONTROL                                       0x0F
#define UX_DEVICE_CLASS_VIDEO_CT_ROLL_RELATIVE_CONTROL                                       0x10
#define UX_DEVICE_CLASS_VIDEO_CT_PRIVACY_CONTROL                                             0x11
#define UX_DEVICE_CLASS_VIDEO_CT_FOCUS_SIMPLE_CONTROL                                        0x12
#define UX_DEVICE_CLASS_VIDEO_CT_WINDOW_CONTROL                                              0x13
#define UX_DEVICE_CLASS_VIDEO_CT_REGION_OF_INTEREST_CONTROL                                  0x14

/* Define Processing Unit (PU) Control Selectors.  */
#define UX_DEVICE_CLASS_VIDEO_PU_CONTROL_UNDEFINED                                           0x00
#define UX_DEVICE_CLASS_VIDEO_PU_BACKLIGHT_COMPENSATION_CONTROL                              0x01
#define UX_DEVICE_CLASS_VIDEO_PU_BRIGHTNESS_CONTROL                                          0x02
#define UX_DEVICE_CLASS_VIDEO_PU_CONTRAST_CONTROL                                            0x03
#define UX_DEVICE_CLASS_VIDEO_PU_GAIN_CONTROL                                                0x04
#define UX_DEVICE_CLASS_VIDEO_PU_POWER_LINE_FREQUENCY_CONTROL                                0x05
#define UX_DEVICE_CLASS_VIDEO_PU_HUE_CONTROL                                                 0x06
#define UX_DEVICE_CLASS_VIDEO_PU_SATURATION_CONTROL                                          0x07
#define UX_DEVICE_CLASS_VIDEO_PU_SHARPNESS_CONTROL                                           0x08
#define UX_DEVICE_CLASS_VIDEO_PU_GAMMA_CONTROL                                               0x09
#define UX_DEVICE_CLASS_VIDEO_PU_WHITE_BALANCE_TEMPERATURE_CONTROL                           0x0A
#define UX_DEVICE_CLASS_VIDEO_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL                      0x0B
#define UX_DEVICE_CLASS_VIDEO_PU_WHITE_BALANCE_COMPONENT_CONTROL                             0x0C
#define UX_DEVICE_CLASS_VIDEO_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL                        0x0D
#define UX_DEVICE_CLASS_VIDEO_PU_DIGITAL_MULTIPLIER_CONTROL                                  0x0E
#define UX_DEVICE_CLASS_VIDEO_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL                            0x0F
#define UX_DEVICE_CLASS_VIDEO_PU_HUE_AUTO_CONTROL                                            0x10
#define UX_DEVICE_CLASS_VIDEO_PU_ANALOG_VIDEO_STANDARD_CONTROL                               0x11
#define UX_DEVICE_CLASS_VIDEO_PU_ANALOG_LOCK_STATUS_CONTROL                                  0x12
#define UX_DEVICE_CLASS_VIDEO_PU_CONTRAST_AUTO_CONTROL                                       0x13

/* Define eXtension Unit (XU) Control Selectors.  */
#define UX_DEVICE_CLASS_VIDEO_XU_CONTROL_UNDEFINED                                           0x00

/* Define VideoStreaming (VS) Interface Control Selectors.  */
#define UX_DEVICE_CLASS_VIDEO_VS_CONTROL_UNDEFINED                                           0x00
#define UX_DEVICE_CLASS_VIDEO_VS_PROBE_CONTROL                                               0x01
#define UX_DEVICE_CLASS_VIDEO_VS_COMMIT_CONTROL                                              0x02
#define UX_DEVICE_CLASS_VIDEO_VS_STILL_PROBE_CONTROL                                         0x03
#define UX_DEVICE_CLASS_VIDEO_VS_STILL_COMMIT_CONTROL                                        0x04
#define UX_DEVICE_CLASS_VIDEO_VS_STILL_IMAGE_TRIGGER_CONTROL                                 0x05
#define UX_DEVICE_CLASS_VIDEO_VS_STREAM_ERROR_CODE_CONTROL                                   0x06
#define UX_DEVICE_CLASS_VIDEO_VS_GENERATE_KEY_FRAME_CONTROL                                  0x07
#define UX_DEVICE_CLASS_VIDEO_VS_UPDATE_FRAME_SEGMENT_CONTROL                                0x08
#define UX_DEVICE_CLASS_VIDEO_VS_SYNCH_DELAY_CONTROL                                         0x09


/* Define USB Video Class terminal types.  */

/* Define USB Terminal Types (TT).  */
#define UX_DEVICE_CLASS_VIDEO_TT_VENDOR_SPECIFIC                                             0x0100
#define UX_DEVICE_CLASS_VIDEO_TT_STREAMING                                                   0x0101

/* Define USB Input Terminal Types (ITT).  */
#define UX_DEVICE_CLASS_VIDEO_ITT_VENDOR_SPECIFIC                                            0x0200
#define UX_DEVICE_CLASS_VIDEO_ITT_CAMERA                                                     0x0201
#define UX_DEVICE_CLASS_VIDEO_ITT_MEDIA_TRANSPORT_INPUT                                      0x0202

/* Define USB Output Terminal Types (OTT).  */
#define UX_DEVICE_CLASS_VIDEO_OTT_VENDOR_SPECIFIC                                            0x0300
#define UX_DEVICE_CLASS_VIDEO_OTT_DISPLAY                                                    0x0301
#define UX_DEVICE_CLASS_VIDEO_OTT_MEDIA_TRANSPORT_OUTPUT                                     0x0302

/* Define USB External Terminal Types (XTT).  */
#define UX_DEVICE_CLASS_VIDEO_XTT_EXTERNAL_VENDOR_SPECIFIC                                   0x0400
#define UX_DEVICE_CLASS_VIDEO_XTT_COMPOSITE_CONNECTOR                                        0x0401
#define UX_DEVICE_CLASS_VIDEO_XTT_SVIDEO_CONNECTOR                                           0x0402
#define UX_DEVICE_CLASS_VIDEO_XTT_COMPONENT_CONNECTOR                                        0x0403


/* Video Class GET_INFO Capabilities and status structs and definitions.  */

#define UX_DEVICE_CLASS_VIDEO_INFO_GET_REQUEST_SUPPORT                                       (1u << 0)
#define UX_DEVICE_CLASS_VIDEO_INFO_SET_REQUEST_SUPPORT                                       (1u << 1)
#define UX_DEVICE_CLASS_VIDEO_INFO_DISABLED_DUE_TO_AUTO_MODE                                 (1u << 2)
#define UX_DEVICE_CLASS_VIDEO_INFO_AUTOUPDATE_CONTROL                                        (1u << 3)
#define UX_DEVICE_CLASS_VIDEO_INFO_ASYNCHRONOUS_CONTROL                                      (1u << 4)
#define UX_DEVICE_CLASS_VIDEO_INFO_DISABLED_DUE_TO_STATE                                     (1u << 5)


/* Video Class Probe and Commit Controls structs and definitions.  */
typedef struct UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_STRUCT
{
    union UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_bmHint_UNION {
        USHORT                                  value;
        struct UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_bmHint_STRUCT {
            USHORT                              dwFrameInterval:1;
            USHORT                              wKeyFrameRate:1;
            USHORT                              wPFrameRate:1;
            USHORT                              wCompQuality:1;
            USHORT                              wCompWindowSize:1;
        }                                       bm;
    }                                           bmHint;
    UCHAR                                       bFormatIndex;
    UCHAR                                       bFrameIndex;
    ULONG                                       dwFrameInterval;
    USHORT                                      wKeyFrameRate;
    USHORT                                      wPFrameRate;
    USHORT                                      wCompQuality;
    USHORT                                      wCompWindowSize;
    USHORT                                      wDelay;
    UCHAR                                       dwMaxVideoFrameSize[4]; /* Not alignend from now on.  */
    UCHAR                                       dwMaxPayloadTransferSize[4];
    UCHAR                                       dwClockFrequency[4];
    union UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_bmFramingInfo_UNION {
        UCHAR                                   value;
        struct UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_bmFramingInfo_STRUCT {
            UCHAR                               FID_required:1;
            UCHAR                               EOF_may_present:1;
            UCHAR                               EOS_may_present:1;
        }                                       bm;
    }                                           bmFramingInfo;
    UCHAR                                       bPreferedVersion;
    UCHAR                                       bMinVersion;
    UCHAR                                       bMaxVersion;

    /* Start from offset 34, additional fields introduced in UVC 1.5.  */
    UCHAR                                       bUsage;
    UCHAR                                       bBitDepthLuma;
    UCHAR                                       bmSettings;
    UCHAR                                       bMaxNumberOfRefFramesPlus1;
    union UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_bmRateControlModes_UNION {
        USHORT                                  value;
        struct UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_bmRateControlModes_STRUCT {
            USHORT                              rate_ctrl_mode_0:4;
            USHORT                              rate_ctrl_mode_1:4;
            USHORT                              rate_ctrl_mode_2:4;
            USHORT                              rate_ctrl_mode_3:4;
        }                                       bm;
    }                                           bmRateControlModes;
    UCHAR                                       bmLayoutPerStream;
} UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL;

#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_HINT_OFFSET                      0
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_HINT_FRAME_INTERVAL              (1U<<0)
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_HINT_KEY_FRAME_RATE              (1U<<1)
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_HINT_KEY_P_FRAME_RATE            (1U<<2)
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_HINT_COMP_QUALITY                (1U<<3)
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_HINT_COMP_WINDOW_SIZE            (1U<<4)
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_FORMAT_INDEX_OFFSET              2
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_FRAME_INDEX_OFFSET               3
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_FRAME_INTERVAL_OFFSET            4
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_KEY_FRAME_RATE_OFFSET            8
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_P_FRAME_RATE_OFFSET              10
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_COMP_QUALITY_OFFSET              12
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_COMP_WINDOW_SIZE_OFFSET          14
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_DELAY_OFFSET                     16
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_MAX_VIDEO_FRAME_SIZE_OFFSET      18
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_MAX_PAYLOAD_TRANSFER_SIZE_OFFSET 22
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_CLOCK_FREQUENCY_OFFSET           26
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_FRAMING_INFO_OFFSET              30
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_FRAMING_INFO_FID_REQUIRED        (1U<<0)
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_FRAMING_INFO_EOF_MAY_PRESENT     (1U<<1)
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_FRAMING_INFO_EOS_MAY_PRESENT     (1U<<2)
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_PREFERED_VERSION_OFFSET          31
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_MIN_VERSION_OFFSET               32
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_MAX_VERSION_OFFSET               33
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_USAGE_OFFSET                     34
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_BIT_DEPTH_LUMA_OFFSET            35
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_SETTINGS_OFFSET                  36
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_MAX_N_OF_FRAMES_PLUS1_OFFSET     37
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_RATE_CONTROL_MODES_OFFSET        38
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_RATE_CONTROL_MODE_NOT_APPLICABLE             0U
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_RATE_CONTROL_MODE_VBR_WITH_UNDERFLOW         1U
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_RATE_CONTROL_MODE_CBR                        2U
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_RATE_CONTROL_MODE_CONSTANT_QP                3U
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_RATE_CONTROL_MODE_GLOBAL_VBR_WITH_UNDERFLOW  4U
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_RATE_CONTROL_MODE_VBR                        5U
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_RATE_CONTROL_MODE_GLOBAL_VBR                 6U
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CONTROL_LAYOUT_PER_STREAM_OFFSET         40


/* Video Class Still Probe and Commit Controls structs and definitions.  */
typedef struct UX_DEVICE_CLASS_VIDEO_STILL_PROBE_COMMIT_CONTROL_STRUCT
{
    UCHAR                                   bFormatIndex;
    UCHAR                                   bFrameIndex;
    UCHAR                                   bCompressionIndex;
    UCHAR                                   dwMaxVideoFrameSize[4]; /* Not aligned from now.  */
    UCHAR                                   dwMaxPayloadTransferSize[4];
} UX_DEVICE_CLASS_VIDEO_STILL_PROBE_COMMIT_CONTROL;
#define UX_DEVICE_CLASS_VIDEO_STILL_PROBE_COMMIT_CONTROL_FORMAT_INDEX_OFFSET                0
#define UX_DEVICE_CLASS_VIDEO_STILL_PROBE_COMMIT_CONTROL_FRAME_INDEX_OFFSET                 1
#define UX_DEVICE_CLASS_VIDEO_STILL_PROBE_COMMIT_CONTROL_COMPRESSION_INDEX_OFFSET           2
#define UX_DEVICE_CLASS_VIDEO_STILL_PROBE_COMMIT_CONTROL_MAX_VIDEO_FRAME_SIZE_OFFSET        3
#define UX_DEVICE_CLASS_VIDEO_STILL_PROBE_COMMIT_CONTROL_MAX_PAYLOAD_TRANSFER_SIZE_OFFSET   7


/* Video Class Status Packet structs and definitions.  */

/* Offsets and possible values.  */
typedef struct UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_HEADER_STRUCT
{
   UCHAR                                    bStatusType;
   UCHAR                                    bOriginator;
} UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_HEADER;
typedef struct UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_VC_STRUCT
{
   UCHAR                                     bStatusType;
   UCHAR                                     bOriginator;
   UCHAR                                     bEvent;
   UCHAR                                     bSelector;
   UCHAR                                     bAttribute;
   UCHAR                                     bValue[1];     /* varies on bAttribute.  */
} UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_VC;
typedef struct UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_VS_STRUCT
{
   UCHAR                                     bStatusType;
   UCHAR                                     bOriginator;
   UCHAR                                     bEvent;
   UCHAR                                     bValue[1];     /* varies on bEvent.  */
} UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_VS;
#define UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_TYPE_OFFSET                                      0
#define UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_TYPE_VC                                          1
#define UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_TYPE_VS                                          2
#define UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_ORIGINATOR_OFFSET                                1
#define UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_EVENT_OFFSET                                     2
#define UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_VC_SELECTOR_OFFSET                               3
#define UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_VC_ATTRIBUTE_OFFSET                              4
#define UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_VC_VALUE_OFFSET                                  5
#define UX_DEVICE_CLASS_VIDEO_STATUS_PACKET_VS_VALUE_OFFSET                                  3


/* Define Video Class Payload Header structs and definitions.  */
typedef struct UX_DEVICE_CLASS_VIDEO_PAYLOAD_HEADER_STRUCT
{
   UCHAR                                     bHeaderLength;
   union UX_DEVICE_CLASS_VIDEO_PAYLOAD_HEADER_INFO_UNION {
      UCHAR                                  value;
      struct UX_DEVICE_CLASS_VIDEO_PAYLOAD_HEADER_INFO_STRUCT {
         UCHAR                               bFID:1; /* Frame ID.  */
         UCHAR                               bEOF:1; /* End of Frame.  */
         UCHAR                               bPTS:1; /* Presentation Time.  */
         UCHAR                               bSCR:1; /* Source Clock Reference.  */
         UCHAR                               bRES:1; /* Reserved, Payload Specific.  */
         UCHAR                               bSTI:1; /* Still Image.  */
         UCHAR                               bERR:1; /* Error.  */
         UCHAR                               bEOH:1; /* End of Header.  */
      }                                      bm;
   }                                         bmHeaderInfo;
   UCHAR                                     dwPresentationTime[4];
   union UX_DEVICE_CLASS_VIDEO_PAYLOAD_SCR_UNION {
      UCHAR                                  byte_array[6];
      struct UX_DEVICE_CLASS_VIDEO_PAYLOAD_SCR_STRUCT {
         UCHAR                               STC[4];  /* SourceTimeClock.  */
         USHORT                              SCR;     /* 1KHz SOF token counter.  */
      }                                      bm;
   }                                         scrSourceClock;
} UX_DEVICE_CLASS_VIDEO_PAYLOAD_HEADER;
#define UX_DEVICE_CLASS_VIDEO_PAYLOAD_HEADER_LENGTH_OFFSET                                   0
#define UX_DEVICE_CLASS_VIDEO_PAYLOAD_HEADER_INFO_OFFSET                                     1
#define UX_DEVICE_CLASS_VIDEO_PAYLOAD_HEADER_PRESENTATION_TIME_OFFSET                        4
#define UX_DEVICE_CLASS_VIDEO_PAYLOAD_HEADER_PTS_OFFSET                                      4
#define UX_DEVICE_CLASS_VIDEO_PAYLOAD_HEADER_SCR_SOURCE_CLOCK_OFFSET                         6
#define UX_DEVICE_CLASS_VIDEO_PAYLOAD_HEADER_SCR_OFFSET                                      6


/* Define Video Class specific request codes.  */
#define UX_DEVICE_CLASS_VIDEO_REQUEST_CODE_UNDEFINED                                         0x00
#define UX_DEVICE_CLASS_VIDEO_RC_UNDEFINED                                                   0x00
#define UX_DEVICE_CLASS_VIDEO_SET_CUR                                                        0x01
#define UX_DEVICE_CLASS_VIDEO_GET_CUR                                                        0x81
#define UX_DEVICE_CLASS_VIDEO_GET_MIN                                                        0x82
#define UX_DEVICE_CLASS_VIDEO_GET_MAX                                                        0x83
#define UX_DEVICE_CLASS_VIDEO_GET_RES                                                        0x84
#define UX_DEVICE_CLASS_VIDEO_GET_INFO                                                       0x86
#define UX_DEVICE_CLASS_VIDEO_GET_DEF                                                        0x87
#define UX_DEVICE_CLASS_VIDEO_GET_CUR_ALL                                                    0x91
#define UX_DEVICE_CLASS_VIDEO_GET_MIN_ALL                                                    0x92
#define UX_DEVICE_CLASS_VIDEO_GET_MAX_ALL                                                    0x93
#define UX_DEVICE_CLASS_VIDEO_GET_RES_ALL                                                    0x94
#define UX_DEVICE_CLASS_VIDEO_GET_GET_DEF_ALL                                                0x97


/* Define Video Class Device Power Modes.  */
#define UX_DEVICE_CLASS_VIDEO_POWER_MODE_MASK                                                0x0Fu
#define UX_DEVICE_CLASS_VIDEO_POWER_MODE_FULL                                                0x00u
#define UX_DEVICE_CLASS_VIDEO_POWER_MODE_DEVICE_DEPENDENT                                    0x01u
#define UX_DEVICE_CLASS_VIDEO_POWER_DEPENDENT_MODE_SUPPORTED                                 0x10u
#define UX_DEVICE_CLASS_VIDEO_POWER_USES_USB                                                 0x20u
#define UX_DEVICE_CLASS_VIDEO_POWER_USES_BATTERY                                             0x40u
#define UX_DEVICE_CLASS_VIDEO_POWER_USES_AC                                                  0x80u


/* Define Video Class Request Error Codes.  */
#define UX_DEVICE_CLASS_VIDEO_REQUEST_ERROR_CODE_NO_ERROR                                    0x00
#define UX_DEVICE_CLASS_VIDEO_REQUEST_ERROR_CODE_NOT_READY                                   0x01
#define UX_DEVICE_CLASS_VIDEO_REQUEST_ERROR_CODE_WRONG_STATE                                 0x02
#define UX_DEVICE_CLASS_VIDEO_REQUEST_ERROR_CODE_POWER                                       0x03
#define UX_DEVICE_CLASS_VIDEO_REQUEST_ERROR_CODE_OUT_OF_RANGE                                0x04
#define UX_DEVICE_CLASS_VIDEO_REQUEST_ERROR_CODE_INVALID_INPUT                               0x05
#define UX_DEVICE_CLASS_VIDEO_REQUEST_ERROR_CODE_INVALID_CONTROL                             0x06
#define UX_DEVICE_CLASS_VIDEO_REQUEST_ERROR_CODE_INVALID_REQUEST                             0x07
#define UX_DEVICE_CLASS_VIDEO_REQUEST_ERROR_CODE_INVALID_VALUE_WITHIN_RANGE                  0x08
#define UX_DEVICE_CLASS_VIDEO_REQUEST_ERROR_CODE_UNKNOWN                                     0xFF


/* Define Video Class Probe and Commit Controls */
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_LENGTH_1_1                                        34
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_LENGTH_1_5                                        48
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_HINT_OFFSET                                       0
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_FORMAT_INDEX_OFFSET                               2
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_FRAME_INDEX_OFFSET                                3
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_FRAME_INTERVAL_OFFSET                             4
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_KEY_FRAME_RATE_OFFSET                             8
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_PFRAME_RAE_OFFSET                                 10
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_COMP_QUALITY_OFFSET                               12
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_COMP_WINDOW_SIZE_OFFSET                           14
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_DELAY_OFFSET                                      16
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_MAX_VIDEO_FRAME_SIZE_OFFSET                       18
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_MAX_PAYLOAD_TRANSFER_SIZE_OFFSET                  22
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_CLOCK_FREQUENCY_OFFSET                            26
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_FRAMING_INFO_OFFSET                               30
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_PREFERED_VERSION_OFFSET                           31
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_MIN_VERSION_OFFSET                                32
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_MAX_VERSION_OFFSET                                33
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_USAGE_OFFSET                                      34
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_BIT_DEPTH_LUMA_OFFSET                             35
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_SETTINGS_OFFSET                                   36
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_MAX_NUMBER_OF_REF_FRAMES_PLUS1_OFFSET             37
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_RATE_CONTROL_MODES_OFFSET                         38
#define UX_DEVICE_CLASS_VIDEO_PROBE_COMMIT_LAYOUT_PER_STREAM_OFFSET                          40


/* Define Video Class Stream Error Codes.  */
#define UX_DEVICE_CLASS_VIDEO_STREAM_ERROR_CODE_NO_ERROR                                     0
#define UX_DEVICE_CLASS_VIDEO_STREAM_ERROR_CODE_PROTECTED_CONTENT                            1
#define UX_DEVICE_CLASS_VIDEO_STREAM_ERROR_CODE_INPUT_BUFFER_UNDERRUN                        2
#define UX_DEVICE_CLASS_VIDEO_STREAM_ERROR_CODE_DATA_DISCONTINUITY                           3
#define UX_DEVICE_CLASS_VIDEO_STREAM_ERROR_CODE_OUTPUT_BUFFER_UNDERRUN                       4
#define UX_DEVICE_CLASS_VIDEO_STREAM_ERROR_CODE_OUTPUT_BUFFER_OVERRUN                        5
#define UX_DEVICE_CLASS_VIDEO_STREAM_ERROR_CODE_FORMAT_CHANGE                                6
#define UX_DEVICE_CLASS_VIDEO_STREAM_ERROR_CODE_STILL_IMAGE_CAPTURE_ERROR                    7


/* Define Video Class Task states.  */
#define UX_DEVICE_CLASS_VIDEO_STREAM_RW_STOP            (UX_STATE_RESET)
#define UX_DEVICE_CLASS_VIDEO_STREAM_RW_START           (UX_STATE_STEP + 1)
#define UX_DEVICE_CLASS_VIDEO_STREAM_RW_WAIT            (UX_STATE_STEP + 2)


/* Define Video Class callback structure.  */

struct UX_DEVICE_CLASS_VIDEO_STREAM_STRUCT;
struct UX_DEVICE_CLASS_VIDEO_STRUCT;

typedef struct UX_DEVICE_CLASS_VIDEO_CALLBACKS_STRUCT
{

    VOID        (*ux_slave_class_video_instance_activate)(VOID *);
    VOID        (*ux_slave_class_video_instance_deactivate)(VOID *);
    UINT        (*ux_device_class_video_request)(struct UX_DEVICE_CLASS_VIDEO_STRUCT *, UX_SLAVE_TRANSFER *);
    VOID         *ux_device_class_video_arg;
} UX_DEVICE_CLASS_VIDEO_CALLBACKS;

typedef struct UX_DEVICE_CLASS_VIDEO_STREAM_CALLBACKS_STRUCT
{
    VOID        (*ux_device_class_video_stream_change)(struct UX_DEVICE_CLASS_VIDEO_STREAM_STRUCT *, ULONG);
    UINT        (*ux_device_class_video_stream_request)(struct UX_DEVICE_CLASS_VIDEO_STREAM_STRUCT *, UX_SLAVE_TRANSFER *);
    VOID        (*ux_device_class_video_stream_payload_done)(struct UX_DEVICE_CLASS_VIDEO_STREAM_STRUCT *, ULONG);
} UX_DEVICE_CLASS_VIDEO_STREAM_CALLBACKS;


/* Define Video Class Calling Parameter structure */

typedef struct UX_DEVICE_CLASS_VIDEO_PAYLOAD_STRUCT
{

    ULONG                                   ux_device_class_video_payload_length;
    UCHAR                                   ux_device_class_video_payload_data[4]; /* Actually size of length.  */
} UX_DEVICE_CLASS_VIDEO_PAYLOAD;

typedef struct UX_DEVICE_CLASS_VIDEO_STREAM_PARAMETER_STRUCT
{
#if defined(UX_DEVICE_STANDALONE)
    UINT                                   (*ux_device_class_video_stream_parameter_task_function)(struct UX_DEVICE_CLASS_VIDEO_STREAM_STRUCT*);
#else
    ULONG                                    ux_device_class_video_stream_parameter_thread_stack_size;
    VOID                                   (*ux_device_class_video_stream_parameter_thread_entry)(ULONG id);
#endif
    UX_DEVICE_CLASS_VIDEO_STREAM_CALLBACKS   ux_device_class_video_stream_parameter_callbacks;

    ULONG                                    ux_device_class_video_stream_parameter_max_payload_buffer_size;
    ULONG                                    ux_device_class_video_stream_parameter_max_payload_buffer_nb;
} UX_DEVICE_CLASS_VIDEO_STREAM_PARAMETER;

typedef struct UX_DEVICE_CLASS_VIDEO_PARAMETER_STRUCT
{
    ULONG                                    ux_device_class_video_parameter_master_interface;
    UX_DEVICE_CLASS_VIDEO_CALLBACKS          ux_device_class_video_parameter_callbacks;

    ULONG                                    ux_device_class_video_parameter_streams_nb;
    UX_DEVICE_CLASS_VIDEO_STREAM_PARAMETER  *ux_device_class_video_parameter_streams;
} UX_DEVICE_CLASS_VIDEO_PARAMETER;

typedef struct UX_DEVICE_CLASS_VIDEO_STREAM_STRUCT
{

    struct UX_DEVICE_CLASS_VIDEO_STRUCT     *ux_device_class_video_stream_video;
    UX_SLAVE_INTERFACE                      *ux_device_class_video_stream_interface;
    UX_SLAVE_ENDPOINT                       *ux_device_class_video_stream_endpoint;

    ULONG                                    ux_device_class_video_stream_error;

    UX_DEVICE_CLASS_VIDEO_STREAM_CALLBACKS   ux_device_class_video_stream_callbacks;

#if !defined(UX_DEVICE_STANDALONE)
    UCHAR                                   *ux_device_class_video_stream_thread_stack;
    UX_THREAD                                ux_device_class_video_stream_thread;
#else
    UINT                                   (*ux_device_class_video_stream_task_function)(struct UX_DEVICE_CLASS_VIDEO_STREAM_STRUCT*);
    UINT                                    ux_device_class_video_stream_task_state;
    UINT                                    ux_device_class_video_stream_task_status;
#endif

    ULONG                                   ux_device_class_video_stream_buffer_error_count;
    UCHAR                                   *ux_device_class_video_stream_buffer;
    ULONG                                    ux_device_class_video_stream_buffer_size;
    ULONG                                    ux_device_class_video_stream_payload_buffer_size;

    UX_DEVICE_CLASS_VIDEO_PAYLOAD           *ux_device_class_video_stream_transfer_pos;
    UX_DEVICE_CLASS_VIDEO_PAYLOAD           *ux_device_class_video_stream_access_pos;
} UX_DEVICE_CLASS_VIDEO_STREAM;
#define _ux_device_class_video_stream_error_set(s,e) do {                       \
    (s)->ux_device_class_video_stream_error = (e);                              \
} while(0)

typedef struct UX_DEVICE_CLASS_VIDEO_STRUCT
{

    UX_SLAVE_CLASS                          *ux_device_class_video_class;
    UX_SLAVE_DEVICE                         *ux_device_class_video_device;
    UX_SLAVE_INTERFACE                      *ux_device_class_video_interface;

    ULONG                                    ux_device_class_video_error;

    UX_DEVICE_CLASS_VIDEO_CALLBACKS          ux_device_class_video_callbacks;

    ULONG                                    ux_device_class_video_streams_nb;
    UX_DEVICE_CLASS_VIDEO_STREAM            *ux_device_class_video_streams;

} UX_DEVICE_CLASS_VIDEO;
#define _ux_device_class_video_request_error_set(v,e) do {                      \
    (v)->ux_device_class_video_error = (e);                                     \
} while(0)


/* Define Video Class function prototypes.  */

UINT    _ux_device_class_video_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_video_uninitialize(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_video_activate(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_video_change(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_video_deactivate(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_video_control_request(UX_SLAVE_CLASS_COMMAND *command);

UINT    _ux_device_class_video_entry(UX_SLAVE_CLASS_COMMAND *command);

UINT    _ux_device_class_video_ioctl(UX_DEVICE_CLASS_VIDEO *video, ULONG ioctl_function,
                                     VOID *parameter);

UINT    _ux_device_class_video_stream_get(UX_DEVICE_CLASS_VIDEO *video, ULONG stream_index, UX_DEVICE_CLASS_VIDEO_STREAM **stream);

VOID    _ux_device_class_video_write_thread_entry(ULONG video_stream);
VOID    _ux_device_class_video_read_thread_entry(ULONG video_stream);

ULONG   _ux_device_class_video_max_payload_length(UX_DEVICE_CLASS_VIDEO_STREAM *video);

UINT    _ux_device_class_video_reception_start(UX_DEVICE_CLASS_VIDEO_STREAM *video);

UINT    _ux_device_class_video_read_payload_get(UX_DEVICE_CLASS_VIDEO_STREAM *video, UCHAR **payload_data, ULONG *payload_length);
UINT    _ux_device_class_video_read_payload_free(UX_DEVICE_CLASS_VIDEO_STREAM *video);

UINT    _ux_device_class_video_transmission_start(UX_DEVICE_CLASS_VIDEO_STREAM *video);

UINT    _ux_device_class_video_write_payload_get(UX_DEVICE_CLASS_VIDEO_STREAM *video, UCHAR **buffer, ULONG *max_length);
UINT    _ux_device_class_video_write_payload_commit(UX_DEVICE_CLASS_VIDEO_STREAM *video, ULONG length);

UINT    _ux_device_class_video_tasks_run(VOID *instance);
UINT    _ux_device_class_video_read_task_function(UX_DEVICE_CLASS_VIDEO_STREAM *stream);
UINT    _ux_device_class_video_write_task_function(UX_DEVICE_CLASS_VIDEO_STREAM *stream);

/* Define Video Class API prototypes.  */

#define ux_device_class_video_entry                   _ux_device_class_video_entry

#define ux_device_class_video_read_thread_entry       _ux_device_class_video_read_thread_entry
#define ux_device_class_video_write_thread_entry      _ux_device_class_video_write_thread_entry

#define ux_device_class_video_request_error_set       _ux_device_class_video_request_error_set
#define ux_device_class_video_stream_error_set        _ux_device_class_video_stream_error_set

#define ux_device_class_video_stream_get              _ux_device_class_video_stream_get

#define ux_device_class_video_max_payload_length      _ux_device_class_video_max_payload_length

#define ux_device_class_video_reception_start         _ux_device_class_video_reception_start

#define ux_device_class_video_read_payload_get        _ux_device_class_video_read_payload_get
#define ux_device_class_video_read_payload_free       _ux_device_class_video_read_payload_free

#define ux_device_class_video_transmission_start      _ux_device_class_video_transmission_start

#define ux_device_class_video_write_payload_get       _ux_device_class_video_write_payload_get
#define ux_device_class_video_write_payload_commit    _ux_device_class_video_write_payload_commit

#define ux_device_class_video_ioctl                   _ux_device_class_video_ioctl

#define ux_device_class_video_read_task_function      _ux_device_class_video_read_task_function
#define ux_device_class_video_write_task_function     _ux_device_class_video_write_task_function


/* Determine if a C++ compiler is being used.  If so, complete the standard
   C conditional started above.  */
#ifdef __cplusplus
}
#endif

#endif
