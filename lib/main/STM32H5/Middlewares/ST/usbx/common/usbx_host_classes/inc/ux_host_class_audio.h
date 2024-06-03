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
/**   Audio Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
/*                                                                        */
/*  COMPONENT DEFINITION                                   RELEASE        */
/*                                                                        */
/*    ux_host_class_audio.h                               PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file contains all the header and extern functions used by the  */
/*    USBX audio class.                                                   */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added interrupt support,    */
/*                                            used interface protocol,    */
/*                                            protect reentry with mutex, */
/*                                            added feedback support,     */
/*                                            added Audio 2.0 support,    */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_HOST_CLASS_AUDIO_H
#define UX_HOST_CLASS_AUDIO_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard
   C is used to process the API information.  */

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif

/* Defined, it enables UAC 2.0 support.  */
/* #define UX_HOST_CLASS_AUDIO_2_SUPPORT  */

/* Defined, it enables feedback endpoint support.  */
/* #define UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT  */

/* Defined, it enables optional interrupt endpoint support.  */
/* #define UX_HOST_CLASS_AUDIO_INTERRUPT_SUPPORT  */

/* Defined, it disables control_get/value_get/value_set and related code (to optimize code size).  */
/* #define UX_HOST_CLASS_AUDIO_DISABLE_CONTROLS  */

#include "ux_class_audio10.h"
#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
#include "ux_class_audio20.h"
#endif

/* Define Audio Class main constants.  */

#define UX_HOST_CLASS_AUDIO_CLASS_TRANSFER_TIMEOUT          30

#define UX_HOST_CLASS_AUDIO_CLASS                           1
#define UX_HOST_CLASS_AUDIO_SUBCLASS_UNDEFINED              0
#define UX_HOST_CLASS_AUDIO_SUBCLASS_CONTROL                1
#define UX_HOST_CLASS_AUDIO_SUBCLASS_STREAMING              2
#define UX_HOST_CLASS_AUDIO_SUBCLASS_MIDI_STREAMING         3
#define UX_HOST_CLASS_AUDIO_PROTOCOL_UNDEFINED              0
#define UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_01_00       0x00
#define UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_02_00       0x20
#define UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_03_00       0x30
#define UX_HOST_CLASS_AUDIO_PROTOCOL_AF_VERSION_01_00       UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_01_00
#define UX_HOST_CLASS_AUDIO_PROTOCOL_AF_VERSION_02_00       UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_02_00
#define UX_HOST_CLASS_AUDIO_PROTOCOL_AF_VERSION_03_00       UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_03_00

#define UX_HOST_CLASS_AUDIO_FUNCTION                            UX_HOST_CLASS_AUDIO_CLASS
#define UX_HOST_CLASS_AUDIO_FUNCTION_SUBCLASS_UNDEFINED         0x00
#define UX_HOST_CLASS_AUDIO_FUNCTION_SUBCLASS_FULL_ADC_3_0      0x01
#define UX_HOST_CLASS_AUDIO_FUNCTION_SUBCLASS_GENERIC_I_O       0x20
#define UX_HOST_CLASS_AUDIO_FUNCTION_SUBCLASS_HEADPHONE         0x21
#define UX_HOST_CLASS_AUDIO_FUNCTION_SUBCLASS_SPEAKER           0x22
#define UX_HOST_CLASS_AUDIO_FUNCTION_SUBCLASS_MICROPHONE        0x23
#define UX_HOST_CLASS_AUDIO_FUNCTION_SUBCLASS_HEADSET           0x24
#define UX_HOST_CLASS_AUDIO_FUNCTION_SUBCLASS_HEADSET_ADAPTER   0x25
#define UX_HOST_CLASS_AUDIO_FUNCTION_SUBCLASS_SPEAKERPHONE      0x26

/* Define Audio Class main descriptor types.  */

#define UX_HOST_CLASS_AUDIO_CS_UNDEFINED                    0x20
#define UX_HOST_CLASS_AUDIO_CS_DEVICE                       0x21
#define UX_HOST_CLASS_AUDIO_CS_CONFIGURATION                0x22
#define UX_HOST_CLASS_AUDIO_CS_STRING                       0x23
#define UX_HOST_CLASS_AUDIO_CS_INTERFACE                    0x24
#define UX_HOST_CLASS_AUDIO_CS_ENDPOINT                     0x25


/* Define Audio Class specific AC interface descriptor subclasses.  */

#define UX_HOST_CLASS_AUDIO_CS_AC_UNDEFINED                 0x00
#define UX_HOST_CLASS_AUDIO_CS_HEADER                       0x01
#define UX_HOST_CLASS_AUDIO_CS_INPUT_TERMINAL               0x02
#define UX_HOST_CLASS_AUDIO_CS_OUTPUT_TERMINAL              0x03
#define UX_HOST_CLASS_AUDIO_CS_MIXER_UNIT                   0x04
#define UX_HOST_CLASS_AUDIO_CS_SELECTOR_UNIT                0x05
#define UX_HOST_CLASS_AUDIO_CS_FEATURE_UNIT                 0x06
#define UX_HOST_CLASS_AUDIO_CS_PROCESSING_UNIT              0x07
#define UX_HOST_CLASS_AUDIO_CS_EXTENSION_UNIT               0x08


/* Define Audio Class specific AS interface descriptor subclasses.  */

#define UX_HOST_CLASS_AUDIO_CS_AS_UNDEFINED                 0x00
#define UX_HOST_CLASS_AUDIO_CS_AS_GENERAL                   0x01
#define UX_HOST_CLASS_AUDIO_CS_FORMAT_TYPE                  0x02
#define UX_HOST_CLASS_AUDIO_CS_FORMAT_SPECIFIC              0x03


/* Define Audio Class specific processing unit process types.  */

#define UX_HOST_CLASS_AUDIO_PROCESS_UNDEFINED               0x00
#define UX_HOST_CLASS_AUDIO_UP_DOWN_MIX_PROCESS             0x01
#define UX_HOST_CLASS_AUDIO_DOLBY_PROLOGIC_PROCESS          0x02
#define UX_HOST_CLASS_AUDIO_3D_STEREO_EXTENDED_PROCESS      0x03
#define UX_HOST_CLASS_AUDIO_REVERBERATION_PROCESS           0x04
#define UX_HOST_CLASS_AUDIO_CHORUS_PROCESS                  0x05
#define UX_HOST_CLASS_AUDIO_DYN_RANGE_COMP_PROCESS          0x06


/* Define Audio Class specific endpoint descriptor subtypes.  */

#define UX_HOST_CLASS_AUDIO_DESCRIPTOR_UNDEFINED            0x00
#define UX_HOST_CLASS_AUDIO_EP_GENERAL                      0x01


/* Define Audio Class specific request codes.  */

#define UX_HOST_CLASS_AUDIO_REQUEST_CODE_UNDEFINED          0x00
#define UX_HOST_CLASS_AUDIO_SET_CUR                         0x01
#define UX_HOST_CLASS_AUDIO_GET_CUR                         0x81
#define UX_HOST_CLASS_AUDIO_SET_MIN                         0x02
#define UX_HOST_CLASS_AUDIO_GET_MIN                         0x82
#define UX_HOST_CLASS_AUDIO_SET_MAX                         0x03
#define UX_HOST_CLASS_AUDIO_GET_MAX                         0x83
#define UX_HOST_CLASS_AUDIO_SET_RES                         0x04
#define UX_HOST_CLASS_AUDIO_GET_RES                         0x84
#define UX_HOST_CLASS_AUDIO_SET_MEM                         0x05
#define UX_HOST_CLASS_AUDIO_GET_MEM                         0x85
#define UX_HOST_CLASS_AUDIO_GET_STAT                        0xFF


/* Define Audio Class specific terminal control selectors.  */

#define UX_HOST_CLASS_AUDIO_TE_CONTROL_UNDEFINED            0x00
#define UX_HOST_CLASS_AUDIO_COPY_PROTECT_CONTROL            0x01


/* Define Audio Class specific feature unit control selectors.  */

#define UX_HOST_CLASS_AUDIO_FU_CONTROL_UNDEFINED            0x00
#define UX_HOST_CLASS_AUDIO_MUTE_CONTROL                    0x01
#define UX_HOST_CLASS_AUDIO_VOLUME_CONTROL                  0x02
#define UX_HOST_CLASS_AUDIO_BASS_CONTROL                    0x03
#define UX_HOST_CLASS_AUDIO_MID_CONTROL                     0x04
#define UX_HOST_CLASS_AUDIO_TREBLE_CONTROL                  0x05
#define UX_HOST_CLASS_AUDIO_GRAPHIC_EQUALIZER_CONTROL       0x06
#define UX_HOST_CLASS_AUDIO_AUTOMATIC_GAIN_CONTROL          0x07
#define UX_HOST_CLASS_AUDIO_DELAY_CONTROL                   0x08
#define UX_HOST_CLASS_AUDIO_BASS_BOOST_CONTROL              0x09
#define UX_HOST_CLASS_AUDIO_LOUNDNESS_CONTROL               0x0A


/* Define Audio Class input terminal types.  */

#define UX_HOST_CLASS_AUDIO_INPUT                           0x0200
#define UX_HOST_CLASS_AUDIO_MICROPHONE                      0x0201
#define UX_HOST_CLASS_AUDIO_DESKTOP_MICROPHONE              0x0202
#define UX_HOST_CLASS_AUDIO_PERSONAL_MICROPHONE             0x0203
#define UX_HOST_CLASS_AUDIO_OMNI_DIRECTIONAL_MICROPHONE     0x0204
#define UX_HOST_CLASS_AUDIO_MICROPHONE_ARRAY                0x0205
#define UX_HOST_CLASS_AUDIO_PROCESSING_MICROPHONE_ARRAY     0x0206


/* Define Audio Class output terminal types.  */

#define UX_HOST_CLASS_AUDIO_OUTPUT                          0x0300
#define UX_HOST_CLASS_AUDIO_SPEAKER                         0x0301
#define UX_HOST_CLASS_AUDIO_HEADPHONES                      0x0302
#define UX_HOST_CLASS_AUDIO_HEAD_MOUNTED_DISPLAY            0x0303
#define UX_HOST_CLASS_AUDIO_DESKTOP_SPEAKER                 0x0304
#define UX_HOST_CLASS_AUDIO_ROOM_SPEAKER                    0x0305
#define UX_HOST_CLASS_AUDIO_COMMUNICATION_SPEAKER           0x0306
#define UX_HOST_CLASS_AUDIO_LOW_FREQUENCY_SPEAKER           0x0307


/* Define Audio Class bidirectional terminal types.  */

#define UX_HOST_CLASS_AUDIO_BIDIRECTIONAL_UNDEFINED         0x0400
#define UX_HOST_CLASS_AUDIO_HANDSET                         0x0401
#define UX_HOST_CLASS_AUDIO_HEADSET                         0x0402
#define UX_HOST_CLASS_AUDIO_SPEAKERPHONE                    0x0403
#define UX_HOST_CLASS_AUDIO_ECHO_SUPRESS_SPEAKERPHONE       0x0404
#define UX_HOST_CLASS_AUDIO_ECHO_CANCEL_SPEAKERPHONE        0x0405


/* Define Audio Class telephony terminal types.  */

#define UX_HOST_CLASS_AUDIO_TELEPHONTY_UNDEFINED            0x0500
#define UX_HOST_CLASS_AUDIO_PHONE_LINE                      0x0501
#define UX_HOST_CLASS_AUDIO_TELEPHONE                       0x0502
#define UX_HOST_CLASS_AUDIO_DOWN_LINE_PHONE                 0x0503

/* Define Audio Class external terminal types.  */

/* Define Audio Class encoding format types.  */

/* Audio 1.0 Format Type Codes.  */
#define UX_HOST_CLASS_AUDIO_FORMAT_TYPE_UNDEFINED                   0
#define UX_HOST_CLASS_AUDIO_FORMAT_TYPE_I                           1
#define UX_HOST_CLASS_AUDIO_FORMAT_TYPE_II                          2
#define UX_HOST_CLASS_AUDIO_FORMAT_TYPE_III                         3

/* Audio 1.0 CS AS Interface Descriptor Type I wFormatTag.  */
#define UX_HOST_CLASS_AUDIO_FORMAT_PCM                              1
#define UX_HOST_CLASS_AUDIO_FORMAT_PCM8                             2
#define UX_HOST_CLASS_AUDIO_FORMAT_IEEE_FLOAT                       3
#define UX_HOST_CLASS_AUDIO_FORMAT_ALAW                             4
#define UX_HOST_CLASS_AUDIO_FORMAT_MULAW                            5
/* Audio 1.0 CS AS Interface Descriptor Type II wFormatTag.  */
#define UX_HOST_CLASS_AUDIO_FORMAT_TYPE_II_UNDEFINED                0x1000
#define UX_HOST_CLASS_AUDIO_FORMAT_MPEG                             0x1001
#define UX_HOST_CLASS_AUDIO_FORMAT_AC_3                             0x1002
/* Audio 1.0 CS AS Interface Descriptor Type III wFormatTag.  */
#define UX_HOST_CLASS_AUDIO_FORMAT_TYPE_III_UNDEFINED               0x2000
#define UX_HOST_CLASS_AUDIO_FORMAT_IEC1937_AC_3                     0x2001
#define UX_HOST_CLASS_AUDIO_FORMAT_IEC1937_MPEG_1_LAYER_1           0x2002
#define UX_HOST_CLASS_AUDIO_FORMAT_IEC1937_MPEG_1_L2_3_MPEG_2_NOEXT 0x2003
#define UX_HOST_CLASS_AUDIO_FORMAT_IEC1937_MPEG_2_EXT               0x2004
#define UX_HOST_CLASS_AUDIO_FORMAT_IEC1937_MPEG_2_LAYER_1_LS        0x2005
#define UX_HOST_CLASS_AUDIO_FORMAT_IEC1937_MPEG_2_LAYER_2_3_LS      0x2006

/* Audio 1.0 descriptor structs.  */

#define UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_ENTRIES            8
#define UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH             8

#define UX_HOST_CLASS_AUDIO_INPUT_TERMINAL_DESCRIPTOR_ENTRIES       10
#define UX_HOST_CLASS_AUDIO_INPUT_TERMINAL_DESCRIPTOR_LENGTH        12

#define UX_HOST_CLASS_AUDIO_OUTPUT_TERMINAL_DESCRIPTOR_ENTRIES      8
#define UX_HOST_CLASS_AUDIO_OUTPUT_TERMINAL_DESCRIPTOR_LENGTH       9

#define UX_HOST_CLASS_AUDIO_FEATURE_UNIT_DESCRIPTOR_ENTRIES         7
#define UX_HOST_CLASS_AUDIO_FEATURE_UNIT_DESCRIPTOR_LENGTH          7

#define UX_HOST_CLASS_AUDIO_STREAMING_INTERFACE_DESCRIPTOR_ENTRIES  6
#define UX_HOST_CLASS_AUDIO_STREAMING_INTERFACE_DESCRIPTOR_LENGTH   6

#define UX_HOST_CLASS_AUDIO_STREAMING_ENDPOINT_DESCRIPTOR_ENTRIES   6
#define UX_HOST_CLASS_AUDIO_STREAMING_ENDPOINT_DESCRIPTOR_LENGTH    6


/* Define Audio Class specific interface descriptor.  */

#define UX_HOST_CLASS_AUDIO_MAX_CHANNEL                         8
#define UX_HOST_CLASS_AUDIO_NAME_LENGTH                         64

typedef struct UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bFormatType;
    ULONG           bNrChannels;
    ULONG           bSubframeSize;
    ULONG           bBitResolution;
    ULONG           bSamFreqType;
} UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR;


/* Define Audio Class specific input terminal interface descriptor.  */

typedef struct UX_HOST_CLASS_AUDIO_INPUT_TERMINAL_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bTerminalID;
    ULONG           wTerminalType;
    ULONG           bAssocTerminal;
    ULONG           bNrChannels;
    ULONG           wChannelConfig;
    ULONG           iChannelNames;
    ULONG           iTerminal;
} UX_HOST_CLASS_AUDIO_INPUT_TERMINAL_DESCRIPTOR;


/* Define Audio Class specific output terminal interface descriptor.  */

typedef struct UX_HOST_CLASS_AUDIO_OUTPUT_TERMINAL_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bTerminalID;
    ULONG           wTerminalType;
    ULONG           bAssocTerminal;
    ULONG           bSourceID;
    ULONG           iTerminal;
} UX_HOST_CLASS_AUDIO_OUTPUT_TERMINAL_DESCRIPTOR;


/* Define Audio Class specific feature unit descriptor.  */

typedef struct UX_HOST_CLASS_AUDIO_FEATURE_UNIT_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bUnitID;
    ULONG           bSourceID;
    ULONG           bControlSize;
    ULONG           bmaControls;
} UX_HOST_CLASS_AUDIO_FEATURE_UNIT_DESCRIPTOR;


/* Define Audio Class streaming interface descriptor.  */

typedef struct UX_HOST_CLASS_AUDIO_STREAMING_INTERFACE_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           bTerminalLink;
    ULONG           bDelay;
    ULONG           wFormatTag;
} UX_HOST_CLASS_AUDIO_STREAMING_INTERFACE_DESCRIPTOR;


/* Define Audio Class specific streaming endpoint descriptor.  */

typedef struct UX_HOST_CLASS_AUDIO_STREAMING_ENDPOINT_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           bmAttributes;
    ULONG           bLockDelayUnits;
    ULONG           wLockDelay;
} UX_HOST_CLASS_AUDIO_STREAMING_ENDPOINT_DESCRIPTOR;


/* Define Audio Class audio control (AC) instance structure.  */
typedef struct UX_HOST_CLASS_AUDIO_AC_STRUCT
{

    /* Common fields for _AUDIO and _AUDIO_AC,
     * check pointed interface subclass (_ux_host_class_audio_subclass_get()) to
     * see remaining fields after interface pointer.  */
    struct UX_HOST_CLASS_AUDIO_STRUCT
                    *ux_host_class_audio_next_instance;
    UX_HOST_CLASS   *ux_host_class_audio_class;
    UX_INTERFACE    *ux_host_class_audio_interface;

    UX_DEVICE       *ux_host_class_audio_device;
    UCHAR *         ux_host_class_audio_configuration_descriptor;
    ULONG           ux_host_class_audio_configuration_descriptor_length;

    /* Fields for _AUDIO_CONTROL_BLOCK (control instance).  */
    UX_ENDPOINT     *ux_host_class_audio_interrupt_endpoint;
    VOID            (*ux_host_class_audio_interrupt_callback)(struct UX_HOST_CLASS_AUDIO_AC_STRUCT*ac, UCHAR*msg, ULONG len, VOID*arg);
    VOID            *ux_host_class_audio_interrupt_callback_arg;
    UINT            ux_host_class_audio_interrupt_started;

    UINT            ux_host_class_audio_as_count;
    struct UX_HOST_CLASS_AUDIO_STRUCT
                    *ux_host_class_audio_as[2]; /* By default allocate 2, actual length depends on _audio_as_count.  */
} UX_HOST_CLASS_AUDIO_AC;

/* Define Audio Class instance structure.  */

typedef struct UX_HOST_CLASS_AUDIO_STRUCT
{

    /* Common fields for _AUDIO and _AUDIO_AC,
     * check pointed interface subclass (_ux_host_class_audio_subclass_get()) to
     * see remaining fields after interface pointer.  */
    struct UX_HOST_CLASS_AUDIO_STRUCT
                    *ux_host_class_audio_next_instance;
    UX_HOST_CLASS   *ux_host_class_audio_class;
    UX_INTERFACE    *ux_host_class_audio_streaming_interface;

    UX_DEVICE       *ux_host_class_audio_device;
    UCHAR *         ux_host_class_audio_configuration_descriptor;
    ULONG           ux_host_class_audio_configuration_descriptor_length;

    /* Fields for _AUDIO (streaming instance).  */
    ULONG           ux_host_class_audio_control_interface_number;
    UCHAR           *ux_host_class_audio_sampling_descriptor;   /* UAC1.0: FormatTypeI Descriptor, UAC2.0: Clock Source Descriptor.  */
    UX_ENDPOINT     *ux_host_class_audio_isochronous_endpoint;
#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
    UX_ENDPOINT     *ux_host_class_audio_feedback_endpoint;
    UCHAR           ux_host_class_audio_feedback_buffer[4];     /* USB 2.0: feedback fits into 3(FS) or 4(HS).  */
#endif
#if defined(UX_HOST_CLASS_AUDIO_INTERRUPT_SUPPORT)
    UX_HOST_CLASS_AUDIO_AC
                    *ux_host_class_audio_ac;
#endif
    struct UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST_STRUCT
                    *ux_host_class_audio_head_transfer_request;
    struct UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST_STRUCT
                    *ux_host_class_audio_tail_transfer_request;
    UINT            ux_host_class_audio_state;
    ULONG           ux_host_class_audio_terminal_link;
    ULONG           ux_host_class_audio_type;
    ULONG           ux_host_class_audio_packet_size;
    ULONG           ux_host_class_audio_packet_freq;
    ULONG           ux_host_class_audio_packet_fraction;
#if !defined(UX_HOST_CLASS_AUDIO_DISABLE_CONTROLS)
    ULONG           ux_host_class_audio_feature_unit_id;
    UINT            ux_host_class_audio_channels;
    ULONG           ux_host_class_audio_channel_control[UX_HOST_CLASS_AUDIO_MAX_CHANNEL];
#endif
    UX_MUTEX        ux_host_class_audio_mutex;
} UX_HOST_CLASS_AUDIO;

#define _ux_host_class_audio_interface_get(a)           (((UX_HOST_CLASS_AUDIO*)(a))->ux_host_class_audio_streaming_interface->ux_interface_descriptor.bInterfaceNumber)
#define _ux_host_class_audio_protocol_get(a)            (((UX_HOST_CLASS_AUDIO*)(a))->ux_host_class_audio_streaming_interface->ux_interface_descriptor.bInterfaceProtocol)
#define _ux_host_class_audio_subclass_get(a)            (((UX_HOST_CLASS_AUDIO*)(a))->ux_host_class_audio_streaming_interface->ux_interface_descriptor.bInterfaceSubClass)

#define _ux_host_class_audio_speed_get(a)               ((a)->ux_host_class_audio_device->ux_device_speed)
#define _ux_host_class_audio_type_get(a)                ((a)->ux_host_class_audio_type)
#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
#define _ux_host_class_audio_feedback_supported(a)      ((a)->ux_host_class_audio_feedback_endpoint)
#else
#define _ux_host_class_audio_feedback_supported(a)      (UX_FALSE)
#endif
#define _ux_host_class_audio_control_interface_get(a)   ((a)->ux_host_class_audio_control_interface_number)
#define _ux_host_class_audio_terminal_link_get(a)       ((a)->ux_host_class_audio_terminal_link)
#define _ux_host_class_audio_packet_size_get(a)         ((a)->ux_host_class_audio_packet_size)
#define _ux_host_class_audio_packet_freq_get(a)         ((a)->ux_host_class_audio_packet_freq)
#define _ux_host_class_audio_packet_fraction_get(a)     ((a)->ux_host_class_audio_packet_fraction)
#define _ux_host_class_audio_max_packet_size_get(a)     ((a)->ux_host_class_audio_isochronous_endpoint->ux_endpoint_transfer_request.ux_transfer_request_packet_length)


/* Define Audio Class isochronous USB transfer request structure.  */

typedef struct UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST_STRUCT
{

    ULONG           ux_host_class_audio_transfer_request_status;
    UCHAR           *ux_host_class_audio_transfer_request_data_pointer;
    ULONG           ux_host_class_audio_transfer_request_requested_length;
    ULONG           ux_host_class_audio_transfer_request_packet_size;    /* Actual packet size to use.  */
    ULONG           ux_host_class_audio_transfer_request_actual_length;
    VOID            (*ux_host_class_audio_transfer_request_completion_function) (struct UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST_STRUCT *);
    UX_SEMAPHORE    ux_host_class_audio_transfer_request_semaphore;
    VOID            *ux_host_class_audio_transfer_request_class_instance;
    UINT            ux_host_class_audio_transfer_request_completion_code;
    struct UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST_STRUCT
                    *ux_host_class_audio_transfer_request_next_audio_transfer_request;
    UX_TRANSFER     ux_host_class_audio_transfer_request;
} UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST;


/* Define Audio Class channel/value control structures.  */

typedef struct UX_HOST_CLASS_AUDIO_CONTROL_STRUCT
{

    ULONG           ux_host_class_audio_control;
    ULONG           ux_host_class_audio_control_channel;
    ULONG           ux_host_class_audio_control_min;
    ULONG           ux_host_class_audio_control_max;
    ULONG           ux_host_class_audio_control_res;
    ULONG           ux_host_class_audio_control_cur;

    ULONG           ux_host_class_audio_control_entity; /* Entity ID.  */
    ULONG           ux_host_class_audio_control_size;   /* Size of value in 1,2 or 4 bytes.  */
} UX_HOST_CLASS_AUDIO_CONTROL;


typedef struct UX_HOST_CLASS_AUDIO_CHANNEL_STRUCT
{

    ULONG           ux_host_class_audio_channel_control;
    ULONG           ux_host_class_audio_channel;
} UX_HOST_CLASS_AUDIO_CHANNEL;


/* Define Audio Class sampling selection structure.  */

typedef struct UX_HOST_CLASS_AUDIO_SAMPLING_STRUCT
{

    ULONG           ux_host_class_audio_sampling_channels;
    ULONG           ux_host_class_audio_sampling_frequency;
    ULONG           ux_host_class_audio_sampling_resolution;
} UX_HOST_CLASS_AUDIO_SAMPLING;


/* Define Audio Class sampling characteristics structure.  */

typedef struct UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS_STRUCT
{

    ULONG           ux_host_class_audio_sampling_characteristics_channels;
    ULONG           ux_host_class_audio_sampling_characteristics_frequency_low;
    ULONG           ux_host_class_audio_sampling_characteristics_frequency_high;
    ULONG           ux_host_class_audio_sampling_characteristics_resolution;

    /* UAC 1.0, must be 1s.  UAC 2.0, from actual audio schematic.  */
    ULONG           ux_host_class_audio_sampling_characteristics_clock_mul;
    ULONG           ux_host_class_audio_sampling_characteristics_clock_div;

    /* UAC 1.0, format type I descriptor for futher operations.  */
    /* UAC 2.0, clock source descriptor (CSD) for futher operations.  */
    UCHAR           *ux_host_class_audio_sampling_characteristics_descriptor;
} UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS;


/* Define Audio Class function prototypes.  */

UINT    _ux_host_class_audio_activate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_audio_alternate_setting_locate(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_SAMPLING *audio_sampling, UINT *alternate_setting);
UINT    _ux_host_class_audio_configure(UX_HOST_CLASS_AUDIO *audio);
UINT    _ux_host_class_audio_control_get(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_CONTROL *audio_control);
UINT    _ux_host_class_audio_control_value_get(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_CONTROL *audio_control);
UINT    _ux_host_class_audio_control_value_set(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_CONTROL *audio_control);
UINT    _ux_host_class_audio_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_audio_descriptor_get(UX_HOST_CLASS_AUDIO *audio);
UINT    _ux_host_class_audio_device_controls_list_get(UX_HOST_CLASS_AUDIO *audio);
UINT    _ux_host_class_audio_device_type_get(UX_HOST_CLASS_AUDIO *audio);
UINT    _ux_host_class_audio_endpoints_get(UX_HOST_CLASS_AUDIO *audio);
UINT    _ux_host_class_audio_entry(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_audio_read(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST *audio_transfer_request);
UINT    _ux_host_class_audio_streaming_sampling_get(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *audio_sampling);
UINT    _ux_host_class_audio_streaming_sampling_set(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_SAMPLING *audio_sampling);
UINT    _ux_host_class_audio_streaming_terminal_get(UX_HOST_CLASS_AUDIO *audio);
UINT    _ux_host_class_audio_transfer_request(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST *audio_transfer_request);
VOID    _ux_host_class_audio_transfer_request_completed(UX_TRANSFER *transfer_request);
UINT    _ux_host_class_audio_write(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST *audio_transfer_request);

UINT    _ux_host_class_audio_control_request(UX_HOST_CLASS_AUDIO *audio,
                        UINT streaming_control,
                        UINT request_type, UINT request,
                        UINT request_value,
                        UINT spec_id,
                        UCHAR *parameter, ULONG parameter_size, ULONG *actual_size);

UINT    _ux_host_class_audio_descriptors_parse(UX_HOST_CLASS_AUDIO *audio,
                        UINT(*parse_function)(VOID  *arg,
                                            UCHAR *packed_interface_descriptor,
                                            UCHAR *packed_endpoint_descriptor,
                                            UCHAR *packed_audio_descriptor),
                        VOID* arg);

UINT    _ux_host_class_audio_raw_sampling_parse(UX_HOST_CLASS_AUDIO *audio,
                        UINT(*parse_function)(VOID  *arg,
                                              UCHAR *packed_interface_descriptor,
                                              UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *sam_attr),
                        VOID* arg);

UINT    _ux_host_class_audio_stop(UX_HOST_CLASS_AUDIO *audio);

VOID    _ux_host_class_audio_feedback_transfer_completed(UX_TRANSFER *transfer_request);

UINT    _ux_host_class_audio_feedback_get(UX_HOST_CLASS_AUDIO *audio, UCHAR *feedback);
UINT    _ux_host_class_audio_feedback_set(UX_HOST_CLASS_AUDIO *audio, UCHAR *feedback);

UINT    _ux_host_class_audio_entity_control_get(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_CONTROL *audio_control);
UINT    _ux_host_class_audio_entity_control_value_get(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_CONTROL *audio_control);
UINT    _ux_host_class_audio_entity_control_value_set(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_CONTROL *audio_control);

UINT    _ux_host_class_audio_interrupt_start(UX_HOST_CLASS_AUDIO_AC *audio,
                        VOID(*callback_function)(UX_HOST_CLASS_AUDIO_AC *audio,
                                                 UCHAR *message, ULONG length,
                                                 VOID *arg),
                        VOID *arg);
VOID    _ux_host_class_audio_interrupt_notification(UX_TRANSFER *transfer_request);

/* Define Audio Class API prototypes.  */

#define ux_host_class_audio_entry                   _ux_host_class_audio_entry
#define ux_host_class_audio_control_get             _ux_host_class_audio_control_get
#define ux_host_class_audio_control_value_get       _ux_host_class_audio_control_value_get
#define ux_host_class_audio_control_value_set       _ux_host_class_audio_control_value_set
#define ux_host_class_audio_read                    _ux_host_class_audio_read
#define ux_host_class_audio_streaming_sampling_get  _ux_host_class_audio_streaming_sampling_get
#define ux_host_class_audio_streaming_sampling_set  _ux_host_class_audio_streaming_sampling_set
#define ux_host_class_audio_streaming_terminal_get  _ux_host_class_audio_streaming_terminal_get
#define ux_host_class_audio_write                   _ux_host_class_audio_write

#define ux_host_class_audio_type_get                    _ux_host_class_audio_type_get
#define ux_host_class_audio_speed_get                   _ux_host_class_audio_speed_get
#define ux_host_class_audio_protocol_get                _ux_host_class_audio_protocol_get
#define ux_host_class_audio_subclass_get                _ux_host_class_audio_subclass_get
#define ux_host_class_audio_feedback_supported          _ux_host_class_audio_feedback_supported

#define ux_host_class_audio_interface_get               _ux_host_class_audio_interface_get
#define ux_host_class_audio_control_interface_get       _ux_host_class_audio_control_interface_get
#define ux_host_class_audio_terminal_link_get           _ux_host_class_audio_terminal_link_get

#define ux_host_class_audio_max_packet_size_get         _ux_host_class_audio_max_packet_size_get
#define ux_host_class_audio_packet_size_get             _ux_host_class_audio_packet_size_get
#define ux_host_class_audio_packet_freq_get             _ux_host_class_audio_packet_freq_get
#define ux_host_class_audio_packet_fraction_get         _ux_host_class_audio_packet_fraction_get

#define ux_host_class_audio_feedback_get                _ux_host_class_audio_feedback_get
#define ux_host_class_audio_feedback_set                _ux_host_class_audio_feedback_set

#define ux_host_class_audio_control_request             _ux_host_class_audio_control_request

#define ux_host_class_audio_descriptors_parse           _ux_host_class_audio_descriptors_parse

#define ux_host_class_audio_entity_control_get          _ux_host_class_audio_entity_control_get
#define ux_host_class_audio_entity_control_value_get    _ux_host_class_audio_entity_control_value_get
#define ux_host_class_audio_entity_control_value_set    _ux_host_class_audio_entity_control_value_set

#define ux_host_class_audio_raw_sampling_parse          _ux_host_class_audio_raw_sampling_parse
#define ux_host_class_audio_raw_sampling_start          _ux_host_class_audio_streaming_sampling_set

#define ux_host_class_audio_stop                        _ux_host_class_audio_stop

#define ux_host_class_audio_interrupt_start             _ux_host_class_audio_interrupt_start

/* Determine if a C++ compiler is being used.  If so, complete the standard
   C conditional started above.  */
#ifdef __cplusplus
}
#endif

#endif


