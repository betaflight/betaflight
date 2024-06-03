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
/*    ux_host_class_video.h                               PORTABLE C      */ 
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX video class.                                                   */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added new definition, field */
/*                                            and functions for video     */
/*                                            control handling, used UX   */
/*                                            prefix to refer to TX       */
/*                                            symbols instead of using    */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            added new definitions       */
/*                                            according to UVC 1.5 Class  */
/*                                            specification,              */
/*                                            resulting in version 6.1.8  */
/*                                                                        */ 
/**************************************************************************/

#ifndef UX_HOST_CLASS_VIDEO_H
#define UX_HOST_CLASS_VIDEO_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  

/* Define external static data.  */
extern UCHAR _ux_system_class_video_interface_descriptor_structure[];
extern UCHAR _ux_system_class_video_input_terminal_descriptor_structure[];     
extern UCHAR _ux_system_class_video_input_header_descriptor_structure[];       
extern UCHAR _ux_system_class_video_processing_unit_descriptor_structure[];
extern UCHAR _ux_system_class_video_streaming_interface_descriptor_structure[];
extern UCHAR _ux_system_class_video_streaming_endpoint_descriptor_structure[];  
extern UCHAR _ux_system_class_video_frame_descriptor_structure[];

extern UCHAR _ux_system_host_class_video_name[];   

/* Define Video Class IOCTL constants.  */
                                                                                            
#define UX_HOST_CLASS_VIDEO_IOCTL_GET_INPUT_TERMINAL                                        0x01
#define UX_HOST_CLASS_VIDEO_IOCTL_GET_FORMAT_NUMBER                                         0x02
#define UX_HOST_CLASS_VIDEO_IOCTL_GET_FORMAT_DATA                                           0x03
#define UX_HOST_CLASS_VIDEO_IOCTL_GET_FRAME_NUMBER                                          0x04
#define UX_HOST_CLASS_VIDEO_IOCTL_GET_FRAME_DATA                                            0x05
#define UX_HOST_CLASS_VIDEO_IOCTL_CHANNEL_START                                             0x06
#define UX_HOST_CLASS_VIDEO_IOCTL_CHANNEL_STOP                                              0x07
#define UX_HOST_CLASS_VIDEO_IOCTL_GET_FRAME_INTERVAL                                        0x08

#define UX_HOST_CLASS_VIDEO_IOCTL_ABORT_IN_PIPE                                             0x80 

/* Define Video Class main constants.  */

#define UX_HOST_CLASS_VIDEO_CLASS_TRANSFER_TIMEOUT                                          30 
#define UX_HOST_CLASS_VIDEO_CLASS                                                           0x0e
#define UX_HOST_CLASS_VIDEO_SUBCLASS_UNDEFINED                                              0
#define UX_HOST_CLASS_VIDEO_SUBCLASS_CONTROL                                                1
#define UX_HOST_CLASS_VIDEO_SUBCLASS_STREAMING                                              2
#define UX_HOST_CLASS_VIDEO_SUBCLASS_INTERFACE_COLLECTION                                   3


/* Define Video Class main descriptor types.  */

#define UX_HOST_CLASS_VIDEO_CS_UNDEFINED                                                    0x20
#define UX_HOST_CLASS_VIDEO_CS_DEVICE                                                       0x21
#define UX_HOST_CLASS_VIDEO_CS_CONFIGURATION                                                0x22 
#define UX_HOST_CLASS_VIDEO_CS_STRING                                                       0x23
#define UX_HOST_CLASS_VIDEO_CS_INTERFACE                                                    0x24
#define UX_HOST_CLASS_VIDEO_CS_ENDPOINT                                                     0x25

/* Define Video Class specific VC .  */

#define UX_HOST_CLASS_VIDEO_VC_DESCRIPTOR_UNDEFINED                                         0x00
#define UX_HOST_CLASS_VIDEO_VC_HEADER                                                       0x01
#define UX_HOST_CLASS_VIDEO_VC_INPUT_TERMINAL                                               0x02
#define UX_HOST_CLASS_VIDEO_VC_OUTPUT_TERMINAL                                              0x03
#define UX_HOST_CLASS_VIDEO_VC_SELECTOR_UNIT                                                0x04
#define UX_HOST_CLASS_VIDEO_VC_PROCESSING_UNIT                                              0x05
#define UX_HOST_CLASS_VIDEO_VC_EXTENSION_UNIT                                               0x06
#define UX_HOST_CLASS_VIDEO_VC_ENCODING_UNIT                                                0x07

/* Define Video Class specific VS .  */

#define UX_HOST_CLASS_VIDEO_VS_UNDEFINED                                                    0x00
#define UX_HOST_CLASS_VIDEO_VS_INPUT_HEADER                                                 0x01
#define UX_HOST_CLASS_VIDEO_VS_OUTPUT_HEADER                                                0x02
#define UX_HOST_CLASS_VIDEO_VS_STILL_IMAGE_FRAME                                            0x03
#define UX_HOST_CLASS_VIDEO_VS_FORMAT_UNCOMPRESSED                                          0x04
#define UX_HOST_CLASS_VIDEO_VS_FRAME_UNCOMPRESSED                                           0x05
#define UX_HOST_CLASS_VIDEO_VS_FORMAT_MJPEG                                                 0x06
#define UX_HOST_CLASS_VIDEO_VS_FRAME_MJPEG                                                  0x07
#define UX_HOST_CLASS_VIDEO_VS_FORMAT_MPEG2TS                                               0x0A
#define UX_HOST_CLASS_VIDEO_VS_FORMAT_DV                                                    0x0C
#define UX_HOST_CLASS_VIDEO_VS_COLORFORMAT                                                  0x0D
#define UX_HOST_CLASS_VIDEO_VS_FORMAT_FRAME_BASED                                           0x10
#define UX_HOST_CLASS_VIDEO_VS_FRAME_FRAME_BASED                                            0x11
#define UX_HOST_CLASS_VIDEO_VS_FORMAT_STREAM_BASED                                          0x12
#define UX_HOST_CLASS_VIDEO_VS_FORMAT_H264                                                  0x13
#define UX_HOST_CLASS_VIDEO_VS_FRAME_H264                                                   0x14
#define UX_HOST_CLASS_VIDEO_VS_FORMAT_H264_SIMULCAST                                        0x15
#define UX_HOST_CLASS_VIDEO_VS_FORMAT_VP8                                                   0x16
#define UX_HOST_CLASS_VIDEO_VS_FRAME_VP8                                                    0x17
#define UX_HOST_CLASS_VIDEO_VS_FORMAT_VP8_SIMULCAST                                         0x18


/* Define Video Class specific Control Selectors.  */

#define UX_HOST_CLASS_VIDEO_CT_CONTROL_UNDEFINED                                            0x00
#define UX_HOST_CLASS_VIDEO_CT_SCANNING_MODE_CONTROL                                        0x01
#define UX_HOST_CLASS_VIDEO_CT_AE_MODE_CONTROL                                              0x02
#define UX_HOST_CLASS_VIDEO_CT_AE_PRIORITY_CONTROL                                          0x03
#define UX_HOST_CLASS_VIDEO_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL                               0x04
#define UX_HOST_CLASS_VIDEO_CT_EXPOSURE_TIME_RELATIVE_CONTROL                               0x05
#define UX_HOST_CLASS_VIDEO_CT_FOCUS_ABSOLUTE_CONTROL                                       0x06
#define UX_HOST_CLASS_VIDEO_CT_FOCUS_RELATIVE_CONTROL                                       0x07
#define UX_HOST_CLASS_VIDEO_CT_FOCUS_AUTO_CONTROL                                           0x08
#define UX_HOST_CLASS_VIDEO_CT_IRIS_ABSOLUTE_CONTROL                                        0x09
#define UX_HOST_CLASS_VIDEO_CT_IRIS_RELATIVE_CONTROL                                        0x0A
#define UX_HOST_CLASS_VIDEO_CT_ZOOM_ABSOLUTE_CONTROL                                        0x0B
#define UX_HOST_CLASS_VIDEO_CT_ZOOM_RELATIVE_CONTROL                                        0x0C
#define UX_HOST_CLASS_VIDEO_CT_PANTILT_ABSOLUTE_CONTROL                                     0x0D
#define UX_HOST_CLASS_VIDEO_CT_PANTILT_RELATIVE_CONTROL                                     0x0E
#define UX_HOST_CLASS_VIDEO_CT_ROLL_ABSOLUTE_CONTROL                                        0x0F
#define UX_HOST_CLASS_VIDEO_CT_ROLL_RELATIVE_CONTROL                                        0x10
#define UX_HOST_CLASS_VIDEO_CT_PRIVACY_CONTROL                                              0x11
#define UX_HOST_CLASS_VIDEO_CT_FOCUS_SIMPLE_CONTROL                                         0x12
#define UX_HOST_CLASS_VIDEO_CT_WINDOW_CONTROL                                               0x13
#define UX_HOST_CLASS_VIDEO_CT_REGION_OF_INTEREST_CONTROL                                   0x14


#define UX_HOST_CLASS_VIDEO_PU_CONTROL_UNDEFINED                                            0x00
#define UX_HOST_CLASS_VIDEO_PU_BACKLIGHT_COMPENSATION_CONTROL                               0x01
#define UX_HOST_CLASS_VIDEO_PU_BRIGHTNESS_CONTROL                                           0x02
#define UX_HOST_CLASS_VIDEO_PU_CONTRAST_CONTROL                                             0x03
#define UX_HOST_CLASS_VIDEO_PU_GAIN_CONTROL                                                 0x04
#define UX_HOST_CLASS_VIDEO_PU_POWER_LINE_FREQUENCY_CONTROL                                 0x05
#define UX_HOST_CLASS_VIDEO_PU_HUE_CONTROL                                                  0x06
#define UX_HOST_CLASS_VIDEO_PU_SATURATION_CONTROL                                           0x07
#define UX_HOST_CLASS_VIDEO_PU_SHARPNESS_CONTROL                                            0x08
#define UX_HOST_CLASS_VIDEO_PU_GAMMA_CONTROL                                                0x09
#define UX_HOST_CLASS_VIDEO_PU_WHITE_BALANCE_TEMPERATURE_CONTROL                            0x0A
#define UX_HOST_CLASS_VIDEO_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL                       0x0B
#define UX_HOST_CLASS_VIDEO_PU_WHITE_BALANCE_COMPONENT_CONTROL                              0x0C
#define UX_HOST_CLASS_VIDEO_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL                         0x0D
#define UX_HOST_CLASS_VIDEO_PU_DIGITAL_MULTIPLIER_CONTROL                                   0x0E
#define UX_HOST_CLASS_VIDEO_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL                             0x0F
#define UX_HOST_CLASS_VIDEO_PU_HUE_AUTO_CONTROL                                             0x10
#define UX_HOST_CLASS_VIDEO_PU_ANALOG_VIDEO_STANDARD_CONTROL                                0x11
#define UX_HOST_CLASS_VIDEO_PU_ANALOG_LOCK_STATUS_CONTROL                                   0x12
#define UX_HOST_CLASS_VIDEO_PU_CONTRAST_AUTO_CONTROL                                        0x13


#define UX_HOST_CLASS_VIDEO_VS_CONTROL_UNDEFINED                                            0x00
#define UX_HOST_CLASS_VIDEO_VS_PROBE_CONTROL                                                0x01
#define UX_HOST_CLASS_VIDEO_VS_COMMIT_CONTROL                                               0x02
#define UX_HOST_CLASS_VIDEO_VS_STILL_PROBE_CONTROL                                          0x03
#define UX_HOST_CLASS_VIDEO_VS_STILL_COMMIT_CONTROL                                         0x04
#define UX_HOST_CLASS_VIDEO_VS_STILL_IMAGE_TRIGGER_CONTROL                                  0x05
#define UX_HOST_CLASS_VIDEO_VS_STREAM_ERROR_CODE_CONTROL                                    0x06
#define UX_HOST_CLASS_VIDEO_VS_GENERATE_KEY_FRAME_CONTROL                                   0x07
#define UX_HOST_CLASS_VIDEO_VS_UPDATE_FRAME_SEGMENT_CONTROL                                 0x08
#define UX_HOST_CLASS_VIDEO_VS_SYNCH_DELAY_CONTROL                                          0x09


/* Define USB Video Class terminal types.  */

#define UX_HOST_CLASS_VIDEO_TT_VENDOR_SPECIFIC                                              0x0100
#define UX_HOST_CLASS_VIDEO_TT_STREAMING                                                    0x0101

#define UX_HOST_CLASS_VIDEO_ITT_VENDOR_SPECIFIC                                             0x0200
#define UX_HOST_CLASS_VIDEO_ITT_CAMERA                                                      0x0201
#define UX_HOST_CLASS_VIDEO_ITT_MEDIA_TRANSPORT_INPUT                                       0x0202

#define UX_HOST_CLASS_VIDEO_OTT_VENDOR_SPECIFIC                                             0x0300
#define UX_HOST_CLASS_VIDEO_OTT_CAMERA                                                      0x0301
#define UX_HOST_CLASS_VIDEO_OTT_MEDIA_TRANSPORT_INPUT                                       0x0302


/* Define USB Video Class Request Error Code Control.  */

#define UX_HOST_CLASS_VIDEO_REQUEST_ERROR_CODE_NO_ERROR                                     0x00
#define UX_HOST_CLASS_VIDEO_REQUEST_ERROR_CODE_NOT_READY                                    0x01
#define UX_HOST_CLASS_VIDEO_REQUEST_ERROR_CODE_WRONG_STATE                                  0x02
#define UX_HOST_CLASS_VIDEO_REQUEST_ERROR_CODE_POWER                                        0x03
#define UX_HOST_CLASS_VIDEO_REQUEST_ERROR_CODE_OUT_OF_RANGE                                 0x04
#define UX_HOST_CLASS_VIDEO_REQUEST_ERROR_CODE_INVALID_INPUT                                0x05
#define UX_HOST_CLASS_VIDEO_REQUEST_ERROR_CODE_INVALID_CONTROL                              0x06
#define UX_HOST_CLASS_VIDEO_REQUEST_ERROR_CODE_INVALID_REQUEST                              0x07
#define UX_HOST_CLASS_VIDEO_REQUEST_ERROR_CODE_INVALID_VALUE_WITHIN_RANGE                   0x08
#define UX_HOST_CLASS_VIDEO_REQUEST_ERROR_CODE_UNKNOWN                                      0xFF


/* Define Video Class encoding format types.  */

#define UX_HOST_CLASS_VIDEO_INTERFACE_DESCRIPTOR_ENTRIES                                    8
#define UX_HOST_CLASS_VIDEO_INTERFACE_DESCRIPTOR_LENGTH                                     8

#define UX_HOST_CLASS_VIDEO_INPUT_TERMINAL_DESCRIPTOR_ENTRIES                               7
#define UX_HOST_CLASS_VIDEO_INPUT_TERMINAL_DESCRIPTOR_LENGTH                                8

#define UX_HOST_CLASS_VIDEO_INPUT_HEADER_DESCRIPTOR_ENTRIES                                 12
#define UX_HOST_CLASS_VIDEO_INPUT_HEADER_DESCRIPTOR_LENGTH                                  13

#define UX_HOST_CLASS_VIDEO_PROCESSING_UNIT_DESCRIPTOR_ENTRIES                              8
#define UX_HOST_CLASS_VIDEO_PROCESSING_UNIT_DESCRIPTOR_LENGTH                               9

#define UX_HOST_CLASS_VIDEO_STREAMING_INTERFACE_DESCRIPTOR_ENTRIES                          6
#define UX_HOST_CLASS_VIDEO_STREAMING_INTERFACE_DESCRIPTOR_LENGTH                           6       

#define UX_HOST_CLASS_VIDEO_STREAMING_ENDPOINT_DESCRIPTOR_ENTRIES                           6
#define UX_HOST_CLASS_VIDEO_STREAMING_ENDPOINT_DESCRIPTOR_LENGTH                            6

#define UX_HOST_CLASS_VIDEO_FRAME_DESCRIPTOR_ENTRIES                                        12
#define UX_HOST_CLASS_VIDEO_FRAME_DESCRIPTOR_LENGTH                                         30


/* Define Video Class specific interface descriptor.  */

#define UX_HOST_CLASS_VIDEO_MAX_CHANNEL                                                     8
#define UX_HOST_CLASS_VIDEO_NAME_LENGTH                                                     64

/* Define Video Class specific request codes.  */

#define UX_HOST_CLASS_VIDEO_REQUEST_CODE_UNDEFINED                                          0x00
#define UX_HOST_CLASS_VIDEO_SET_CUR                                                         0x01
#define UX_HOST_CLASS_VIDEO_GET_CUR                                                         0x81
#define UX_HOST_CLASS_VIDEO_GET_MIN                                                         0x82
#define UX_HOST_CLASS_VIDEO_GET_MAX                                                         0x83
#define UX_HOST_CLASS_VIDEO_GET_RES                                                         0x84
#define UX_HOST_CLASS_VIDEO_GET_INFO                                                        0x86
#define UX_HOST_CLASS_VIDEO_GET_DEF                                                         0x87
#define UX_HOST_CLASS_VIDEO_GET_CUR_ALL                                                     0x91
#define UX_HOST_CLASS_VIDEO_GET_MIN_ALL                                                     0x92
#define UX_HOST_CLASS_VIDEO_GET_MAX_ALL                                                     0x93
#define UX_HOST_CLASS_VIDEO_GET_RES_ALL                                                     0x94
#define UX_HOST_CLASS_VIDEO_GET_GET_DEF_ALL                                                 0x97

/* Define Video Class error codes.  */

#define UX_HOST_CLASS_VIDEO_WRONG_TYPE                                                      0x90
#define UX_HOST_CLASS_VIDEO_WRONG_INTERFACE                                                 0x91
#define UX_HOST_CLASS_VIDEO_PARAMETER_ERROR                                                 0x92
#define UX_HOST_CLASS_VIDEO_DEVICE_NOT_READY                                                0x93

/* Define Video Class Terminal Control Selectors.  */
#define UX_HOST_CLASS_VIDEO_TCS_VC_CONTROL_UNDEFINED                                        0x00
#define UX_HOST_CLASS_VIDEO_TCS_VC_VIDEO_POWER_MODE_CONTROL                                 0x01
#define UX_HOST_CLASS_VIDEO_TCS_VC_REQUEST_ERROR_CODE_CONTROL                               0x02

#define UX_HOST_CLASS_VIDEO_TCS_TE_CONTROL_UNDEFINED                                        0x00

#define UX_HOST_CLASS_VIDEO_TCS_SU_CONTROL_UNDEFINED                                        0x00
#define UX_HOST_CLASS_VIDEO_TCS_SU_INPUT_SELECT_CONTROL                                     0x01

#define UX_HOST_CLASS_VIDEO_TCS_CT_CONTROL_UNDEFINED                                        0x00
#define UX_HOST_CLASS_VIDEO_TCS_CT_SCANNING_MODE_CONTROL                                    0x01
#define UX_HOST_CLASS_VIDEO_TCS_CT_AE_MODE_CONTROL                                          0x02
#define UX_HOST_CLASS_VIDEO_TCS_CT_AE_PRIORITY_CONTROL                                      0x03
#define UX_HOST_CLASS_VIDEO_TCS_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL                           0x04
#define UX_HOST_CLASS_VIDEO_TCS_CT_EXPOSURE_TIME_RELATIVE_CONTROL                           0x05
#define UX_HOST_CLASS_VIDEO_TCS_CT_FOCUS_ABSOLUTE_CONTROL                                   0x06
#define UX_HOST_CLASS_VIDEO_TCS_CT_FOCUS_RELATIVE_CONTROL                                   0x07
#define UX_HOST_CLASS_VIDEO_TCS_CT_FOCUS_AUTO_CONTROL                                       0x08
#define UX_HOST_CLASS_VIDEO_TCS_CT_IRIS_ABSOLUTE_CONTROL                                    0x09
#define UX_HOST_CLASS_VIDEO_TCS_CT_IRIS_RELATIVE_CONTROL                                    0x0A
#define UX_HOST_CLASS_VIDEO_TCS_CT_ZOOM_ABSOLUTE_CONTROL                                    0x0B
#define UX_HOST_CLASS_VIDEO_TCS_CT_ZOOM_RELATIVE_CONTROL                                    0x0C
#define UX_HOST_CLASS_VIDEO_TCS_CT_PANTILT_ABSOLUTE_CONTROL                                 0x0D
#define UX_HOST_CLASS_VIDEO_TCS_CT_PANTILT_RELATIVE_CONTROL                                 0x0E
#define UX_HOST_CLASS_VIDEO_TCS_CT_ROLL_ABSOLUTE_CONTROL                                    0x0F
#define UX_HOST_CLASS_VIDEO_TCS_CT_ROLL_RELATIVE_CONTROL                                    0x10
#define UX_HOST_CLASS_VIDEO_TCS_CT_PRIVACY_CONTROL                                          0x11

#define UX_HOST_CLASS_VIDEO_TCS_PU_CONTROL_UNDEFINED                                        0x00
#define UX_HOST_CLASS_VIDEO_TCS_PU_BACKLIGHT_COMPENSATION_CONTROL                           0x01
#define UX_HOST_CLASS_VIDEO_TCS_PU_BRIGHTNESS_CONTROL                                       0x02
#define UX_HOST_CLASS_VIDEO_TCS_PU_CONTRAST_CONTROL                                         0x03
#define UX_HOST_CLASS_VIDEO_TCS_PU_GAIN_CONTROL                                             0x04
#define UX_HOST_CLASS_VIDEO_TCS_PU_POWER_LINE_FREQUENCY_CONTROL                             0x05
#define UX_HOST_CLASS_VIDEO_TCS_PU_HUE_CONTROL                                              0x06
#define UX_HOST_CLASS_VIDEO_TCS_PU_SATURATION_CONTROL                                       0x07
#define UX_HOST_CLASS_VIDEO_TCS_PU_SHARPNESS_CONTROL                                        0x08
#define UX_HOST_CLASS_VIDEO_TCS_PU_GAMMA_CONTROL                                            0x09
#define UX_HOST_CLASS_VIDEO_TCS_PU_WHITE_BALANCE_TEMPERATURE_CONTROL                        0x0A
#define UX_HOST_CLASS_VIDEO_TCS_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL                   0x0B
#define UX_HOST_CLASS_VIDEO_TCS_PU_WHITE_BALANCE_COMPONENT_CONTROL                          0x0C

#define UX_HOST_CLASS_VIDEO_TCS_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL                     0x0D
#define UX_HOST_CLASS_VIDEO_TCS_PU_DIGITAL_MULTIPLIER_CONTROL                               0x0E
#define UX_HOST_CLASS_VIDEO_TCS_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL                         0x0F
#define UX_HOST_CLASS_VIDEO_TCS_PU_HUE_AUTO_CONTROL                                         0x10
#define UX_HOST_CLASS_VIDEO_TCS_PU_ANALOG_VIDEO_STANDARD_CONTROL                            0x11
#define UX_HOST_CLASS_VIDEO_TCS_PU_ANALOG_LOCK_STATUS_CONTROL                               0x12
#define UX_HOST_CLASS_VIDEO_TCS_PU_CONTRAST_AUTO_CONTROL                                    0x13

#define UX_HOST_CLASS_VIDEO_TCS_XU_CONTROL_UNDEFINED                                        0x00

#define UX_HOST_CLASS_VIDEO_TCS_VS_CONTROL_UNDEFINED                                        0x00
#define UX_HOST_CLASS_VIDEO_TCS_VS_PROBE_CONTROL                                            0x01
#define UX_HOST_CLASS_VIDEO_TCS_VS_COMMIT_CONTROL                                           0x02
#define UX_HOST_CLASS_VIDEO_TCS_VS_STILL_PROBE_CONTROL                                      0x03
#define UX_HOST_CLASS_VIDEO_TCS_VS_STILL_COMMIT_CONTROL                                     0x04
#define UX_HOST_CLASS_VIDEO_TCS_VS_STILL_IMAGE_TRIGGER_CONTROL                              0x05
#define UX_HOST_CLASS_VIDEO_TCS_VS_STREAM_ERROR_CODE_CONTROL                                0x06
#define UX_HOST_CLASS_VIDEO_TCS_VS_GENERATE_KEY_FRAME_CONTROL                               0x07
#define UX_HOST_CLASS_VIDEO_TCS_VS_UPDATE_FRAME_SEGMENT_CONTROL                             0x08
#define UX_HOST_CLASS_VIDEO_TCS_VS_SYNCH_DELAY_CONTROL                                      0x09

/* Define Video Class Probe and Commit Controls */

#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_LENGTH                                             34
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_HINT                                               0
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_FORMAT_INDEX                                       2
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_FRAME_INDEX                                        3
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_FRAME_INTERVAL                                     4
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_KEY_FRAME_RATE                                     8
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_PFRAME_RAE                                         10
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_COMP_QUALITY                                       12
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_COMP_WINDOW_SIZE                                   14
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_DELAY                                              16
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_MAX_VIDEO_FRAME_SIZE                               18
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_MAX_PAYLOAD_TRANSFER_SIZE                          22
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_CLOCK_FREQUENCY                                    26
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_FRAMING_INFO                                       30
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_PREFERED_VERSION                                   31
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_MIN_VERSION                                        32
#define UX_HOST_CLASS_VIDEO_PROBE_COMMIT_MAX_VERSION                                        33

#ifndef UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST_COUNT
#define UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST_COUNT                                          8
#endif

typedef struct UX_HOST_CLASS_VIDEO_INTERFACE_HEADER_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bcdUVC;
    ULONG           wTotalLength;
    ULONG           dwClockFrequency;
    ULONG           bInCollection;
    ULONG           baInterfaceNr;
} UX_HOST_CLASS_VIDEO_INTERFACE_HEADER_DESCRIPTOR;

/* Define Video Class specific input header interface descriptor.  */

typedef struct UX_HOST_CLASS_VIDEO_INPUT_TERMINAL_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bTerminalID;
    ULONG           wTerminalType;
    ULONG           bAssocTerminal;
    ULONG           iTerminal;
} UX_HOST_CLASS_VIDEO_INPUT_TERMINAL_DESCRIPTOR;

/* Define Video Class specific input header interface descriptor.  */

typedef struct UX_HOST_CLASS_VIDEO_INPUT_HEADER_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bNumFormats;
    ULONG           wTotalLength;
    ULONG           bEndpointAddress;
    ULONG           bmInfo;
    ULONG           bTerminalLink;
    ULONG           bStillCaptureMethod;
    ULONG           bTriggerSupport;
    ULONG           bTriggerUsage;
    ULONG           bControlSize;
    ULONG           bmaControls;
} UX_HOST_CLASS_VIDEO_INPUT_HEADER_DESCRIPTOR;

/* Define Video Class Selector descriptor.  */

typedef struct UX_HOST_CLASS_VIDEO_SELECTOR_UNIT_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           bUnitID;
    ULONG           bNrInPins;
    ULONG           baSourceID;
} UX_HOST_CLASS_VIDEO_SELECTOR_UNIT_DESCRIPTOR;

/* Define Video Class Camera Terminal descriptor.  */

typedef struct UX_HOST_CLASS_VIDEO_CAMERA_TERMINAL_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           bTerminalID;
    ULONG           wTerminalType;
    ULONG           bAssocTerminal;
    ULONG           iTerminal;
    ULONG           wObjectiveFocalLengthMin;
    ULONG           wObjectiveFocalLengthMax;
    ULONG           wOcularFocalLength;
    ULONG           bControlSize;
    ULONG           bmControls;
} UX_HOST_CLASS_VIDEO_CAMERA_TERMINAL_DESCRIPTOR;

/* Define Video Class Frame descriptor.  */

typedef struct UX_HOST_CLASS_VIDEO_FRAME_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           bFrameIndex;
    ULONG           bmCapabilities;
    ULONG           wWidth;
    ULONG           wHeight;
    ULONG           dwMinBitRate;
    ULONG           dwMaxBitRate;
    ULONG           dwMaxVideoFrameBufferSize;
    ULONG           dwDefaultFrameInterval;
    ULONG           bFrameIntervalType;
} UX_HOST_CLASS_VIDEO_FRAME_DESCRIPTOR;



/* Define Video Class Processing Unit descriptor.  */

typedef struct UX_HOST_CLASS_VIDEO_PROCESSING_UNIT_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           bUnitID;
    ULONG           bSourceID;
    ULONG           wMaxMultiplier;
    ULONG           bControlSize;
    ULONG           bmControls;
    ULONG           iProcessing;
    ULONG           bmVideoStandards;
} UX_HOST_CLASS_VIDEO_PROCESSING_UNIT_DESCRIPTOR;


/* Define Video Class instance structure.  */

typedef struct UX_HOST_CLASS_VIDEO_STRUCT
{

    struct UX_HOST_CLASS_VIDEO_STRUCT                    
                    *ux_host_class_video_next_instance;
    UX_HOST_CLASS   *ux_host_class_video_class;
    UX_DEVICE       *ux_host_class_video_device;
    UX_INTERFACE    *ux_host_class_video_streaming_interface;
    ULONG           ux_host_class_video_control_interface_number;
    UX_ENDPOINT     *ux_host_class_video_isochronous_endpoint;
    UINT            ux_host_class_video_state;
    ULONG           ux_host_class_video_feature_unit_id;
    ULONG           ux_host_class_video_terminal_id;
    ULONG           ux_host_class_video_terminal_type;
    UCHAR           *ux_host_class_video_configuration_descriptor;
    ULONG           ux_host_class_video_configuration_descriptor_length;
    UCHAR           ux_host_class_video_name[UX_HOST_CLASS_VIDEO_NAME_LENGTH];
    ULONG           ux_host_class_video_number_formats;
    ULONG           ux_host_class_video_length_formats;
    UCHAR           *ux_host_class_video_format_address;
    UCHAR           *ux_host_class_video_current_format_address;
    ULONG           ux_host_class_video_current_format;
    ULONG           ux_host_class_video_number_frames;
    ULONG           ux_host_class_video_current_frame;
    UCHAR           *ux_host_class_video_current_frame_address;
    ULONG           ux_host_class_video_current_frame_interval;
    ULONG           ux_host_class_video_current_max_payload_size;
    UX_TRANSFER     ux_host_class_video_transfer_requests[UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST_COUNT];
    ULONG           ux_host_class_video_transfer_request_start_index;
    ULONG           ux_host_class_video_transfer_request_end_index;
    UX_SEMAPHORE    ux_host_class_video_semaphore;
    UX_SEMAPHORE    ux_host_class_video_semaphore_control_request;
    VOID            (*ux_host_class_video_transfer_completion_function)(UX_TRANSFER*);

} UX_HOST_CLASS_VIDEO;


/* Define Video Class isochronous USB transfer request structure.  */

typedef struct UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST_STRUCT
{

    ULONG           ux_host_class_video_transfer_request_status;
    UCHAR *         ux_host_class_video_transfer_request_data_pointer;
    ULONG           ux_host_class_video_transfer_request_requested_length;
    ULONG           ux_host_class_video_transfer_request_actual_length;
    VOID            (*ux_host_class_video_transfer_request_completion_function) (struct UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST_STRUCT *);
    UX_SEMAPHORE    ux_host_class_video_transfer_request_semaphore;
    VOID            *ux_host_class_video_transfer_request_class_instance;
    UINT            ux_host_class_video_transfer_request_completion_code;
    struct UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST_STRUCT   
                    *ux_host_class_video_transfer_request_next_video_transfer_request;
    UX_TRANSFER     ux_host_class_video_transfer_request;
} UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST;


/* Define Video Class channel/value control structures.  */

typedef struct UX_HOST_CLASS_VIDEO_CONTROL_STRUCT
{

    ULONG          ux_host_class_video_control;
    LONG           ux_host_class_video_control_min;
    LONG           ux_host_class_video_control_max;
    LONG           ux_host_class_video_control_res;
    LONG           ux_host_class_video_control_cur;
} UX_HOST_CLASS_VIDEO_CONTROL;

/* Define Video Class input terminal structures.  */

typedef struct UX_HOST_CLASS_VIDEO_PARAMETER_INPUT_TERMINAL_STRUCT
{

    ULONG           ux_host_class_video_parameter_input_terminal_id;
    ULONG           ux_host_class_video_parameter_input_terminal_type;

} UX_HOST_CLASS_VIDEO_PARAMETER_INPUT_TERMINAL;

/* Define Video Class format number structure.  */

typedef struct UX_HOST_CLASS_VIDEO_PARAMETER_NUMBER_FORMATS_STRUCT
{

    ULONG           ux_host_class_video_parameter_number_formats;

} UX_HOST_CLASS_VIDEO_PARAMETER_NUMBER_FORMATS;

/* Define Video Class format data structure.  */

typedef struct UX_HOST_CLASS_VIDEO_PARAMETER_FORMAT_DATA_STRUCT
{

    ULONG           ux_host_class_video_parameter_format_requested;
    ULONG           ux_host_class_video_parameter_format_subtype;
    ULONG           ux_host_class_video_parameter_number_frame_descriptors;

} UX_HOST_CLASS_VIDEO_PARAMETER_FORMAT_DATA;

typedef struct UX_HOST_CLASS_VIDEO_PARAMETER_FRAME_DATA_STRUCT
{

    ULONG           ux_host_class_video_parameter_frame_requested;
    ULONG           ux_host_class_video_parameter_frame_subtype;
    ULONG           ux_host_class_video_parameter_frame_width;
    ULONG           ux_host_class_video_parameter_frame_height;
    ULONG           ux_host_class_video_parameter_default_frame_interval;
    ULONG           ux_host_class_video_parameter_frame_interval_type;

} UX_HOST_CLASS_VIDEO_PARAMETER_FRAME_DATA;


typedef struct UX_HOST_CLASS_VIDEO_PARAMETER_CHANNEL_STRUCT
{

    ULONG           ux_host_class_video_parameter_channel_bandwidth_selection;
    ULONG           ux_host_class_video_parameter_format_requested;
    ULONG           ux_host_class_video_parameter_frame_requested;
    ULONG           ux_host_class_video_parameter_frame_interval_requested;

} UX_HOST_CLASS_VIDEO_PARAMETER_CHANNEL;

typedef struct UX_HOST_CLASS_VIDEO_PARAMETER_FRAME_INTERVAL_STRUCT
{

    ULONG           ux_host_class_video_parameter_frame_requested;
    ULONG          *ux_host_class_video_parameter_frame_interval_buffer;
    ULONG           ux_host_class_video_parameter_frame_interval_buffer_length;
    ULONG           ux_host_class_video_parameter_frame_interval_buffer_length_written;

} UX_HOST_CLASS_VIDEO_PARAMETER_FRAME_INTERVAL;

/* Define Video Class function prototypes.  */

UINT    _ux_host_class_video_activate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_video_configure(UX_HOST_CLASS_VIDEO *video);
UINT    _ux_host_class_video_control_get(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_CONTROL *video_control);
UINT    _ux_host_class_video_control_value_get(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_CONTROL *video_control);
UINT    _ux_host_class_video_control_value_set(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_CONTROL *video_control);
UINT    _ux_host_class_video_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_video_descriptor_get(UX_HOST_CLASS_VIDEO *video);
UINT    _ux_host_class_video_endpoints_get(UX_HOST_CLASS_VIDEO *video);
UINT    _ux_host_class_video_entry(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_video_read(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST *video_transfer_request);
UINT    _ux_host_class_video_transfer_request(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST *video_transfer_request);
VOID    _ux_host_class_video_transfer_request_completed(UX_TRANSFER *transfer_request);
VOID    _ux_host_class_video_transfer_request_callback(UX_TRANSFER *transfer_request);
UINT    _ux_host_class_video_control_list_get(UX_HOST_CLASS_VIDEO *video);
UINT    _ux_host_class_video_input_format_get(UX_HOST_CLASS_VIDEO *video);
UINT    _ux_host_class_video_input_terminal_get(UX_HOST_CLASS_VIDEO *video);
UINT    _ux_host_class_video_alternate_setting_locate(UX_HOST_CLASS_VIDEO *video, UINT max_payload_size, UINT *alternate_setting);
UINT    _ux_host_class_video_ioctl(UX_HOST_CLASS_VIDEO *video, ULONG ioctl_function, VOID *parameter);
UINT    _ux_host_class_video_format_data_get(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_PARAMETER_FORMAT_DATA *format_parameter);
UINT    _ux_host_class_video_frame_data_get(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_PARAMETER_FRAME_DATA *frame_parameter);
UINT    _ux_host_class_video_frame_interval_get(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_PARAMETER_FRAME_INTERVAL *interval_parameter);
UINT    _ux_host_class_video_channel_start(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_PARAMETER_CHANNEL *video_parameter);
UINT    _ux_host_class_video_stop(UX_HOST_CLASS_VIDEO *video);
UINT    _ux_host_class_video_start(UX_HOST_CLASS_VIDEO *video);
UINT    _ux_host_class_video_frame_parameters_set(UX_HOST_CLASS_VIDEO *video, ULONG frame_format, ULONG width, ULONG height, ULONG frame_interval);
ULONG   _ux_host_class_video_max_payload_get(UX_HOST_CLASS_VIDEO *video);
UINT    _ux_host_class_video_transfer_buffer_add(UX_HOST_CLASS_VIDEO *video, UCHAR* buffer);
UINT    _ux_host_class_video_transfer_buffers_add(UX_HOST_CLASS_VIDEO *video, UCHAR** buffers, ULONG num_buffers);
VOID    _ux_host_class_video_transfer_callback_set(UX_HOST_CLASS_VIDEO *video, VOID (*callback_function)(UX_TRANSFER*));
UINT    _ux_host_class_video_entities_parse(UX_HOST_CLASS_VIDEO *video,
                        UINT(*parse_function)(VOID  *arg,
                                            UCHAR *packed_interface_descriptor,
                                            UCHAR *packed_entity_descriptor),
                        VOID* arg);
UINT    _ux_host_class_video_control_request(UX_HOST_CLASS_VIDEO *video,
                        UINT request, UCHAR interface_index,
                        UINT entity_id, UINT control_selector,
                        UCHAR *parameter, UINT parameter_size);

/* Define Video Class API prototypes.  */

#define ux_host_class_video_entry                   _ux_host_class_video_entry
#define ux_host_class_video_control_get             _ux_host_class_video_control_get
#define ux_host_class_video_control_value_get       _ux_host_class_video_control_value_get
#define ux_host_class_video_control_value_set       _ux_host_class_video_control_value_set
#define ux_host_class_video_read                    _ux_host_class_video_read
#define ux_host_class_video_ioctl                   _ux_host_class_video_ioctl
#define ux_host_class_video_start                   _ux_host_class_video_start
#define ux_host_class_video_stop                    _ux_host_class_video_stop
#define ux_host_class_video_frame_parameters_set    _ux_host_class_video_frame_parameters_set
#define ux_host_class_video_max_payload_get         _ux_host_class_video_max_payload_get
#define ux_host_class_video_transfer_buffer_add     _ux_host_class_video_transfer_buffer_add
#define ux_host_class_video_transfer_buffers_add    _ux_host_class_video_transfer_buffers_add
#define ux_host_class_video_transfer_callback_set   _ux_host_class_video_transfer_callback_set
#define ux_host_class_video_entities_parse          _ux_host_class_video_entities_parse
#define ux_host_class_video_control_request         _ux_host_class_video_control_request

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif


