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
/**   Device Audio Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
/*                                                                        */
/*  COMPONENT DEFINITION                                   RELEASE        */
/*                                                                        */
/*    ux_device_class_audio20.h                           PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file contains all the header and extern functions used by the  */
/*    USBX audio class version 2.0.                                       */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added volume RES support,   */
/*                                            resulting in version 6.1.6  */
/*  08-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added support of multiple   */
/*                                            sampling frequencies,       */
/*                                            added clock multiplier DEFs,*/
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DEVICE_CLASS_AUDIO20_H
#define UX_DEVICE_CLASS_AUDIO20_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  


/* Define Audio Class function category codes.  */

#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_UNDEFINED                0x00
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_DESKTOP_SPEAKER          0x01
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_HOME_THEATER             0x02
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_MICROPHONE               0x03
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_HEADSET                  0x04
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_TELEPHONE                0x05
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_CONVERTER                0x06
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_VOICE_SOUND_RECORDER     0x07
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_I_O_BOX                  0x08
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_MUSICAL_INSTRUMENT       0x09
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_PRO_AUDIO                0x0A
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_AUDIO_VIDEO              0x0B
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_CONTROL_PANEL            0x0C
#define UX_DEVICE_CLASS_AUDIO20_CATEGORY_OTHER                    0xFF


/* Define Audio Class specific AC interface descriptor subclasses.  */

#define UX_DEVICE_CLASS_AUDIO20_AC_UNDEFINED                      0x00
#define UX_DEVICE_CLASS_AUDIO20_AC_HEADER                         0x01
#define UX_DEVICE_CLASS_AUDIO20_AC_INPUT_TERMINAL                 0x02
#define UX_DEVICE_CLASS_AUDIO20_AC_OUTPUT_TERMINAL                0x03
#define UX_DEVICE_CLASS_AUDIO20_AC_MIXER_UNIT                     0x04
#define UX_DEVICE_CLASS_AUDIO20_AC_SELECTOR_UNIT                  0x05
#define UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT                   0x06
#define UX_DEVICE_CLASS_AUDIO20_AC_EFFECT_UNIT                    0x07
#define UX_DEVICE_CLASS_AUDIO20_AC_PROCESSING_UNIT                0x08
#define UX_DEVICE_CLASS_AUDIO20_AC_EXTENSION_UNIT                 0x09
#define UX_DEVICE_CLASS_AUDIO20_AC_CLOCK_SOURCE                   0x0A
#define UX_DEVICE_CLASS_AUDIO20_AC_CLOCK_SELECTOR                 0x0B
#define UX_DEVICE_CLASS_AUDIO20_AC_CLOCK_MULTIPLIER               0x0C
#define UX_DEVICE_CLASS_AUDIO20_AC_SAMPLE_RATE_CONVERTER          0x0D


/* Define Audio Class specific AS interface descriptor subclasses.  */

#define UX_DEVICE_CLASS_AUDIO20_AS_UNDEFINED                      0x00
#define UX_DEVICE_CLASS_AUDIO20_AS_GENERAL                        0x01
#define UX_DEVICE_CLASS_AUDIO20_AS_FORMAT_TYPE                    0x02
#define UX_DEVICE_CLASS_AUDIO20_AS_ENCODER                        0x03
#define UX_DEVICE_CLASS_AUDIO20_AS_DECODER                        0x04


/* Define Audio Class specific endpoint descriptor subtypes.  */

#define UX_DEVICE_CLASS_AUDIO20_EP_UNDEFINED                      0x00
#define UX_DEVICE_CLASS_AUDIO20_EP_GENERAL                        0x01


/* Define Audio Class specific request codes.  */

#define UX_DEVICE_CLASS_AUDIO20_REQUEST_CODE_UNDEFINED            0x00
#define UX_DEVICE_CLASS_AUDIO20_CUR                               0x01
#define UX_DEVICE_CLASS_AUDIO20_RANGE                             0x02
#define UX_DEVICE_CLASS_AUDIO20_MEM                               0x03


/* Define Audio Class specific clock source control selectors.  */

#define UX_DEVICE_CLASS_AUDIO20_CS_CONTROL_UNDEFINED              0x00
#define UX_DEVICE_CLASS_AUDIO20_CS_SAM_FREQ_CONTROL               0x01
#define UX_DEVICE_CLASS_AUDIO20_CS_CLOCK_VALID_CONTROL            0x02


/* Define Audio Class specific clock selector control selectors.  */


#define UX_DEVICE_CLASS_AUDIO20_CX_CONTROL_UNDEFINED              0x00
#define UX_DEVICE_CLASS_AUDIO20_CX_CLOCK_SELECTOR_CONTROL         0x01


/* Define Audio Class specific clock multiplier control selectors.  */


#define UX_DEVICE_CLASS_AUDIO20_CM_CONTROL_UNDEFINED              0x00
#define UX_DEVICE_CLASS_AUDIO20_CM_NUMERATOR_CONTROL              0x01
#define UX_DEVICE_CLASS_AUDIO20_CM_DENOMINATOR_CONTROL            0x02

/* Define Audio Class specific terminal control selectors.  */

#define UX_DEVICE_CLASS_AUDIO20_TE_CONTROL_UNDEFINED              0x00
#define UX_DEVICE_CLASS_AUDIO20_TE_COPY_PROTECT_CONTROL           0x01
#define UX_DEVICE_CLASS_AUDIO20_TE_CONNECTOR_CONTROL              0x02
#define UX_DEVICE_CLASS_AUDIO20_TE_OVERLOAD_CONTROL               0x03
#define UX_DEVICE_CLASS_AUDIO20_TE_CLUSTER_CONTROL                0x04
#define UX_DEVICE_CLASS_AUDIO20_TE_UNDERFLOW_CONTROL              0x05
#define UX_DEVICE_CLASS_AUDIO20_TE_OVERFLOW_CONTROL               0x06
#define UX_DEVICE_CLASS_AUDIO20_TE_LATENCY_CONTROL                0x07


/* Define Audio Class specific feature unit control selectors.  */

#define UX_DEVICE_CLASS_AUDIO20_FU_CONTROL_UNDEFINED              0x00
#define UX_DEVICE_CLASS_AUDIO20_FU_MUTE_CONTROL                   0x01
#define UX_DEVICE_CLASS_AUDIO20_FU_VOLUME_CONTROL                 0x02
#define UX_DEVICE_CLASS_AUDIO20_FU_BASS_CONTROL                   0x03
#define UX_DEVICE_CLASS_AUDIO20_FU_MID_CONTROL                    0x04
#define UX_DEVICE_CLASS_AUDIO20_FU_TREBLE_CONTROL                 0x05
#define UX_DEVICE_CLASS_AUDIO20_FU_GRAPHIC_EQUALIZER_CONTROL      0x06
#define UX_DEVICE_CLASS_AUDIO20_FU_AUTOMATIC_GAIN_CONTROL         0x07
#define UX_DEVICE_CLASS_AUDIO20_FU_DELAY_CONTROL                  0x08
#define UX_DEVICE_CLASS_AUDIO20_FU_BASS_BOOST_CONTROL             0x09
#define UX_DEVICE_CLASS_AUDIO20_FU_LOUNDNESS_CONTROL              0x0A
#define UX_DEVICE_CLASS_AUDIO20_FU_INPUT_GAIN_CONTROL             0x0B
#define UX_DEVICE_CLASS_AUDIO20_FU_INPUT_GAIN_PAD_CONTROL         0x0C
#define UX_DEVICE_CLASS_AUDIO20_FU_PHASE_INVERTER_CONTROL         0x0D
#define UX_DEVICE_CLASS_AUDIO20_FU_UNDERFLOW_CONTROL              0x0E
#define UX_DEVICE_CLASS_AUDIO20_FU_OVERFLOW_CONTROL               0x0F
#define UX_DEVICE_CLASS_AUDIO20_FU_LATENCY_CONTROL                0x10


/* Define Audio Class encoding format type bit allocations.  */

#define UX_DEVICE_CLASS_AUDIO20_FORMAT_PCM                        (1u << 0)
#define UX_DEVICE_CLASS_AUDIO20_FORMAT_PCM8                       (1u << 1)
#define UX_DEVICE_CLASS_AUDIO20_FORMAT_IEEE_FLOAT                 (1u << 2)
#define UX_DEVICE_CLASS_AUDIO20_FORMAT_ALAW                       (1u << 3)
#define UX_DEVICE_CLASS_AUDIO20_FORMAT_MULAW                      (1u << 4)
#define UX_DEVICE_CLASS_AUDIO20_FORMAT_RAW                        (1u << 31)


/* Audio Class Control header descriptor structures.  */

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_HEADER_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           bcdADC;
    ULONG           bCategory;
    ULONG           wTotalLength;
    ULONG           bmControls;
} UX_DEVICE_CLASS_AUDIO20_AC_HEADER_DESCRIPTOR;


/* Define Audio Class specific clock source descriptor.  */

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_CLOCK_SOURCE_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bClockID;
    ULONG           bmAttributes;
    ULONG           bmControls;
    ULONG           bAssocTerminal;
    ULONG           iClockSource;
} UX_DEVICE_CLASS_AUDIO20_AC_CLOCK_SOURCE_DESCRIPTOR;


/* Define Audio Class specific input terminal interface descriptor.  */

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_INPUT_TERMINAL_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bTerminalID;
    ULONG           wTerminalType;
    ULONG           bAssocTerminal;
    ULONG           bCSourceID;
    ULONG           bNrChannels;
    ULONG           bmChannelConfig;
    ULONG           iChannelNames;
    ULONG           bmControls;
    ULONG           iTerminal;
} UX_DEVICE_CLASS_AUDIO20_AC_INPUT_TERMINAL_DESCRIPTOR;


/* Define Audio Class specific output terminal interface descriptor.  */

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_OUTPUT_TERMINAL_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bTerminalID;
    ULONG           wTerminalType;
    ULONG           bAssocTerminal;
    ULONG           bSourceID;
    ULONG           bCSourceID;
    ULONG           bmControls;
    ULONG           iTerminal;
} UX_DEVICE_CLASS_AUDIO20_AC_OUTPUT_TERMINAL_DESCRIPTOR;


/* Define Audio Class specific feature unit descriptor.  */

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bUnitID;
    ULONG           bSourceID;
    ULONG           bmaControls;
} UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT_DESCRIPTOR;

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT1_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bUnitID;
    ULONG           bSourceID;
    ULONG           bmaControls0;
    ULONG           bmaControls1;
} UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT1_DESCRIPTOR;

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT2_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bUnitID;
    ULONG           bSourceID;
    ULONG           bmaControls0;
    ULONG           bmaControls1;
    ULONG           bmaControls2;
} UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT2_DESCRIPTOR;

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT3_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bUnitID;
    ULONG           bSourceID;
    ULONG           bmaControls0;
    ULONG           bmaControls1;
    ULONG           bmaControls2;
    ULONG           bmaControls3;
} UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT3_DESCRIPTOR;

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT6_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bUnitID;
    ULONG           bSourceID;
    ULONG           bmaControls0;
    ULONG           bmaControls1;
    ULONG           bmaControls2;
    ULONG           bmaControls3;
    ULONG           bmaControls4;
    ULONG           bmaControls5;
    ULONG           bmaControls6;
} UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT6_DESCRIPTOR;

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT7_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bUnitID;
    ULONG           bSourceID;
    ULONG           bmaControls0;
    ULONG           bmaControls1;
    ULONG           bmaControls2;
    ULONG           bmaControls3;
    ULONG           bmaControls4;
    ULONG           bmaControls5;
    ULONG           bmaControls6;
    ULONG           bmaControls7;
} UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT7_DESCRIPTOR;

typedef struct UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT8_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubType;
    ULONG           bUnitID;
    ULONG           bSourceID;
    ULONG           bmaControls0;
    ULONG           bmaControls1;
    ULONG           bmaControls2;
    ULONG           bmaControls3;
    ULONG           bmaControls4;
    ULONG           bmaControls5;
    ULONG           bmaControls6;
    ULONG           bmaControls7;
    ULONG           bmaControls8;
} UX_DEVICE_CLASS_AUDIO20_AC_FEATURE_UNIT8_DESCRIPTOR;


/* Define Audio Class streaming interface descriptor.  */

typedef struct UX_DEVICE_CLASS_AUDIO20_AS_INTERFACE_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           bTerminalLink;
    ULONG           bmControls;
    ULONG           bFormatType;
    ULONG           bmFormats;
    ULONG           bNrChannels;
    ULONG           bmChannelConfig;
    ULONG           iChannelNames;
} UX_DEVICE_CLASS_AUDIO20_AS_INTERFACE_DESCRIPTOR;


/* Define Audio Class type I format type descriptor.  */

typedef struct UX_DEVICE_CLASS_AUDIO20_AS_TYPE_I_FORMAT_TYPE_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           bFormatType;
    ULONG           bSubslotSize;
    ULONG           bBitResolution;
} UX_DEVICE_CLASS_AUDIO20_AS_TYPE_I_FORMAT_TYPE_DESCRIPTOR;


/* Define Audio Class specific streaming endpoint descriptor.  */

typedef struct UX_DEVICE_CLASS_AUDIO20_AS_DATA_ENDPOINT_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           bmAttributes;
    ULONG           bmControls;
    ULONG           bLockDelayUnits;
    ULONG           wLockDelay;
} UX_DEVICE_CLASS_AUDIO20_AS_DATA_ENDPOINT_DESCRIPTOR;

typedef struct UX_DEVICE_CLASS_AUDIO20_CONTROL_STRUCT
{
    ULONG           ux_device_class_audio20_control_changed;

    ULONG           ux_device_class_audio20_control_cs_id;
    ULONG           ux_device_class_audio20_control_sampling_frequency;         /* Set to 0 to customize frequencies (with following two fields).  */
    ULONG           ux_device_class_audio20_control_sampling_frequency_cur;     /* Current selected frequency.  */
    UCHAR           *ux_device_class_audio20_control_sampling_frequency_range;  /* UAC 2.0 Layer 3 parameter block of RANGE.  */

    ULONG           ux_device_class_audio20_control_fu_id;
    USHORT          ux_device_class_audio20_control_mute[1];
    SHORT           ux_device_class_audio20_control_volume_min[1];
    SHORT           ux_device_class_audio20_control_volume_max[1];
    USHORT          ux_device_class_audio20_control_volume_res[1];
    SHORT           ux_device_class_audio20_control_volume[1];
} UX_DEVICE_CLASS_AUDIO20_CONTROL;

#define UX_DEVICE_CLASS_AUDIO20_CONTROL_MUTE_CHANGED                1u
#define UX_DEVICE_CLASS_AUDIO20_CONTROL_VOLUME_CHANGED              2u
#define UX_DEVICE_CLASS_AUDIO20_CONTROL_FREQUENCY_CHANGED           4u

typedef struct UX_DEVICE_CLASS_AUDIO20_CONTROL_GROUP_STRUCT
{
    ULONG                            ux_device_class_audio20_control_group_controls_nb;
    UX_DEVICE_CLASS_AUDIO20_CONTROL *ux_device_class_audio20_control_group_controls;
} UX_DEVICE_CLASS_AUDIO20_CONTROL_GROUP;


UINT _ux_device_class_audio20_control_process(UX_DEVICE_CLASS_AUDIO *audio,
                                              UX_SLAVE_TRANSFER *transfer,
                                              UX_DEVICE_CLASS_AUDIO20_CONTROL_GROUP *group);

#define ux_device_class_audio20_control_process _ux_device_class_audio20_control_process

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif /* ifndef UX_DEVICE_CLASS_AUDIO20_H */
