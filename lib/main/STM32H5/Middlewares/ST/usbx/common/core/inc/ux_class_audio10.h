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
/*    ux_class_audio10.h                                  PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file contains all the header and structures used by the        */
/*    USBX Audio Class (UAC) 1.0.                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/

#ifndef UX_CLASS_AUDIO10_H
#define UX_CLASS_AUDIO10_H

/* Define Audio Class Codes.  */

#define UX_CLASS_AUDIO10_CLASS                              0x01
#define UX_CLASS_AUDIO10_SUBCLASS_UNDEFINED                 0x00
#define UX_CLASS_AUDIO10_SUBCLASS_AUDIOCONTROL              0x01
#define UX_CLASS_AUDIO10_SUBCLASS_AUDIOSTREAMING            0x02
#define UX_CLASS_AUDIO10_SUBCLASS_MIDISTREAMING             0x03
#define UX_CLASS_AUDIO10_PROTOCOL_UNDEFINED                 0x00


/* Define Audio Class desctiptor types.  */
#define UX_CLASS_AUDIO10_CS_UNDEFINED                       0x20
#define UX_CLASS_AUDIO10_CS_DEVICE                          0x21
#define UX_CLASS_AUDIO10_CS_CONFIGURATION                   0x22
#define UX_CLASS_AUDIO10_CS_STRING                          0x23
#define UX_CLASS_AUDIO10_CS_INTERFACE                       0x24
#define UX_CLASS_AUDIO10_CS_ENDPOINT                        0x25


/* Define Audio Class AC interface descriptor subclasses.  */

#define UX_CLASS_AUDIO10_AC_UNDEFINED                       0x00
#define UX_CLASS_AUDIO10_AC_HEADER                          0x01
#define UX_CLASS_AUDIO10_AC_INPUT_TERMINAL                  0x02
#define UX_CLASS_AUDIO10_AC_OUTPUT_TERMINAL                 0x03
#define UX_CLASS_AUDIO10_AC_MIXER_UNIT                      0x04
#define UX_CLASS_AUDIO10_AC_SELECTOR_UNIT                   0x05
#define UX_CLASS_AUDIO10_AC_FEATURE_UNIT                    0x06
#define UX_CLASS_AUDIO10_AC_PROCESSING_UNIT                 0x07
#define UX_CLASS_AUDIO10_AC_EXTENSION_UNIT                  0x08


/* Define Audio Class Processing Unit (PU) Process Types (PT).  */

#define UX_CLASS_AUDIO10_PROCESS_UNDEFINED                  0x00
#define UX_CLASS_AUDIO10_PROCESS_UP_DOWN_MIX                0x01
#define UX_CLASS_AUDIO10_PROCESS_DOLBY_PROLOGIC             0x02
#define UX_CLASS_AUDIO10_PROCESS_3D_STEREO_EXTENDER         0x03
#define UX_CLASS_AUDIO10_PROCESS_REVERBERATION              0x04
#define UX_CLASS_AUDIO10_PROCESS_CHORUS                     0x05
#define UX_CLASS_AUDIO10_PROCESS_DYN_RANGE_COMP             0x06


/* Define Audio Class AS interface descriptor subclasses.  */

#define UX_CLASS_AUDIO10_AS_UNDEFINED                       0x00
#define UX_CLASS_AUDIO10_AS_GENERAL                         0x01
#define UX_CLASS_AUDIO10_AS_FORMAT_TYPE                     0x02
#define UX_CLASS_AUDIO10_AS_FORMAT_SPECIFIC                 0x03


/* Define Audio Class endpoint descriptor subtypes.  */

#define UX_CLASS_AUDIO10_EP_UNDEFINED                       0x00
#define UX_CLASS_AUDIO10_EP_GENERAL                         0x01


/* Define Audio Class request codes.  */

#define UX_CLASS_AUDIO10_REQUEST_CODE_UNDEFINED             0x00
#define UX_CLASS_AUDIO10_SET_CUR                            0x01
#define UX_CLASS_AUDIO10_GET_CUR                            0x81
#define UX_CLASS_AUDIO10_SET_MIN                            0x02
#define UX_CLASS_AUDIO10_GET_MIN                            0x82
#define UX_CLASS_AUDIO10_SET_MAX                            0x03
#define UX_CLASS_AUDIO10_GET_MAX                            0x83
#define UX_CLASS_AUDIO10_SET_RES                            0x04
#define UX_CLASS_AUDIO10_GET_RES                            0x84
#define UX_CLASS_AUDIO10_SET_MEM                            0x05
#define UX_CLASS_AUDIO10_GET_MEM                            0x85
#define UX_CLASS_AUDIO10_GET_STAT                           0xFF


/* Define Audio Class terminal control selectors.  */

#define UX_CLASS_AUDIO10_TE_CONTROL_UNDEFINED               0x00
#define UX_CLASS_AUDIO10_TE_COPY_PROTECT_CONTROL            0x01

/* Define Audio Class feature unit control selectors.  */

#define UX_CLASS_AUDIO10_FU_CONTROL_UNDEFINED               0x00
#define UX_CLASS_AUDIO10_FU_MUTE_CONTROL                    0x01
#define UX_CLASS_AUDIO10_FU_VOLUME_CONTROL                  0x02
#define UX_CLASS_AUDIO10_FU_BASS_CONTROL                    0x03
#define UX_CLASS_AUDIO10_FU_MID_CONTROL                     0x04
#define UX_CLASS_AUDIO10_FU_TREBLE_CONTROL                  0x05
#define UX_CLASS_AUDIO10_FU_GRAPHIC_EQUALIZER_CONTROL       0x06
#define UX_CLASS_AUDIO10_FU_AUTOMATIC_GAIN_CONTROL          0x07
#define UX_CLASS_AUDIO10_FU_DELAY_CONTROL                   0x08
#define UX_CLASS_AUDIO10_FU_BASS_BOOST_CONTROL              0x09
#define UX_CLASS_AUDIO10_FU_LOUNDNESS_CONTROL               0x0A


/* Define Audio Class processing unit control selectors.  */

/* Define Audio Class up/down-mix (UD) processing unit control selectors.  */

#define UX_CLASS_AUDIO10_UD_CONTROL_UNDEFINED               0x00
#define UX_CLASS_AUDIO10_UD_ENABLE_CONTROL                  0x01
#define UX_CLASS_AUDIO10_UD_MODE_SELECT_CONTROL             0x02

/* Define Audio Class dolby prologic (DP) processing unit control selectors.  */

#define UX_CLASS_AUDIO10_DP_CONTROL_UNDEFINED               0x00
#define UX_CLASS_AUDIO10_DP_ENABLE_CONTROL                  0x01
#define UX_CLASS_AUDIO10_DP_MODE_SELECT_CONTROL             0x02

/* Define Audio Class 3D stereo extender (3D) processing unit control selectors.  */

#define UX_CLASS_AUDIO10_3D_CONTROL_UNDEFINED               0x00
#define UX_CLASS_AUDIO10_3D_ENABLE_CONTROL                  0x01
#define UX_CLASS_AUDIO10_3D_SPACIOUSNESS_CONTROL            0x02

/* Define Audio Class reverberation (RV) processing unit control selectors.  */

#define UX_CLASS_AUDIO10_RV_CONTROL_UNDEFINED               0x00
#define UX_CLASS_AUDIO10_RV_ENABLE_CONTROL                  0x01
#define UX_CLASS_AUDIO10_RV_LEVEL_CONTROL                   0x02
#define UX_CLASS_AUDIO10_RV_TIME_CONTROL                    0x03
#define UX_CLASS_AUDIO10_RV_FEEDBACK_CONTROL                0x04

/* Define Audio Class chorus (CH) processing unit control selectors.  */

#define UX_CLASS_AUDIO10_CHORUS_CONTROL_UNDEFINED           0x00
#define UX_CLASS_AUDIO10_CHORUS_ENABLE_CONTROL              0x01
#define UX_CLASS_AUDIO10_CHORUS_LEVEL_CONTROL               0x02
#define UX_CLASS_AUDIO10_CHORUS_RATE_CONTROL                0x03
#define UX_CLASS_AUDIO10_CHORUS_DEPTH_CONTROL               0x04

/* Define Audio Class dynamic range compressor (DR) processing unit control selectors.  */

#define UX_CLASS_AUDIO10_DR_CONTROL_UNDEFINED               0x00
#define UX_CLASS_AUDIO10_DR_ENABLE_CONTROL                  0x01
#define UX_CLASS_AUDIO10_DR_COMPRESSION_RATE_CONTROL        0x02
#define UX_CLASS_AUDIO10_DR_MAXAMPL_CONTROL                 0x03
#define UX_CLASS_AUDIO10_DR_THRESHOLD_CONTROL               0x04
#define UX_CLASS_AUDIO10_DR_ATTACK_TIME                     0x05
#define UX_CLASS_AUDIO10_DR_RELEASE_TIME                    0x06

/* Define Audio Class extension unit (XU) control selectors.  */

#define UX_CLASS_AUDIO10_XU_CONTROL_UNDEFINED               0x00
#define UX_CLASS_AUDIO10_XU_ENABLE_CONTROL                  0x01


/* Define Audio Class endpoint control selectors.  */

#define UX_CLASS_AUDIO10_EP_CONTROL_UNDEFINED               0x00
#define UX_CLASS_AUDIO10_EP_SAMPLING_FREQ_CONTROL           0x01
#define UX_CLASS_AUDIO10_EP_PITCH_CONTROL                   0x02


/* Define Audio Class format type codes.  */

#define UX_CLASS_AUDIO10_FORMAT_TYPE_UNDEFINED              0x00
#define UX_CLASS_AUDIO10_FORMAT_TYPE_I                      0x01
#define UX_CLASS_AUDIO10_FORMAT_TYPE_II                     0x02
#define UX_CLASS_AUDIO10_FORMAT_TYPE_III                    0x03

/* Define Audio Class format tag codes.  */

#define UX_CLASS_AUDIO10_FORMAT_TYPE_I_UNDEFINED                    0x0000
#define UX_CLASS_AUDIO10_FORMAT_TYPE_I_PCM                          0x0001
#define UX_CLASS_AUDIO10_FORMAT_TYPE_I_PCM8                         0x0002
#define UX_CLASS_AUDIO10_FORMAT_TYPE_I_IEEE_FLOAT                   0x0003
#define UX_CLASS_AUDIO10_FORMAT_TYPE_I_ALAW                         0x0004
#define UX_CLASS_AUDIO10_FORMAT_TYPE_I_MULAW                        0x0005

#define UX_CLASS_AUDIO10_FORMAT_TYPE_II_UNDEFINED                   0x1000
#define UX_CLASS_AUDIO10_FORMAT_TYPE_II_MPEG                        0x1001
#define UX_CLASS_AUDIO10_FORMAT_TYPE_II_AC3                         0x1002

#define UX_CLASS_AUDIO10_FORMAT_TYPE_III_UNDEFINED                  0x2000
#define UX_CLASS_AUDIO10_FORMAT_TYPE_III_IEC1937_AC3                0x2001
#define UX_CLASS_AUDIO10_FORMAT_TYPE_III_IEC1937_MPEG1_L1           0x2002
#define UX_CLASS_AUDIO10_FORMAT_TYPE_III_IEC1937_MPEG1_L2_3         0x2003
#define UX_CLASS_AUDIO10_FORMAT_TYPE_III_IEC1937_MPEG2_NOEXT        0x2003
#define UX_CLASS_AUDIO10_FORMAT_TYPE_III_IEC1937_MPEG2_EXT          0x2004
#define UX_CLASS_AUDIO10_FORMAT_TYPE_III_IEC1937_MPEG2_L1_LS        0x2005
#define UX_CLASS_AUDIO10_FORMAT_TYPE_III_IEC1937_MPEG2_L2_3_LS      0x2006


/* Define Audio Class MPEG (MP) control selectors.  */

#define UX_CLASS_AUDIO10_MP_CONTROL_UNDEFINED                       0x00
#define UX_CLASS_AUDIO10_MP_DUAL_CHANNEL_CONTROL                    0x01
#define UX_CLASS_AUDIO10_MP_SECOND_STEREO_CONTROL                   0x02
#define UX_CLASS_AUDIO10_MP_MULTILINGUAL_CONTROL                    0x03
#define UX_CLASS_AUDIO10_MP_DYN_RANGE_CONTROL                       0x04
#define UX_CLASS_AUDIO10_MP_SCALING_CONTROL                         0x05
#define UX_CLASS_AUDIO10_MP_HILO_SCALING_CONTROL                    0x06

/* Define Audio Class AC-3 (AC) control selectors.  */

#define UX_CLASS_AUDIO10_AC_CONTROL_UNDEFINED                       0x00
#define UX_CLASS_AUDIO10_AC_MODE_CONTROL                            0x01
#define UX_CLASS_AUDIO10_AC_DYN_RANGE_CONTROL                       0x02
#define UX_CLASS_AUDIO10_AC_SCALING_CONTROL                         0x03
#define UX_CLASS_AUDIO10_AC_HILO_SCALING_CONTROL                    0x04


/* Class descriptor structures (packed).
 * Typedefs to be used:
 * - for byte               : UCHAR/CHAR
 * - for word               : USHORT/SHORT
 * - for double word (dword): ULONG/LONG
 * - for 64-bit-width word  : ULONG64
 * Field offset considerations inside descriptor:
 * - Minimum fields alignment: byte (8-bit)
 * - Field is not byte and not aligned : field declared as bytes array (UCHAR[])
 * - Field is word and word aligned    : field declared as USHORT/SHORT
 * - Field is dword and dword aligned  : field declared as ULONG/LONG
 */


/* Audio Class AC interface header descriptors (bInCollection=1).  */

typedef struct UX_CLASS_AUDIO10_AC_HEADER_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bcdADC[2];
    UCHAR           wTotalLength[2];
    UCHAR           bInCollection;
    UCHAR           baInterfaceNr[1];
} UX_CLASS_AUDIO10_AC_HEADER_DESCRIPTOR;

/* Define Audio Class input terminal descriptor (ITD).  */

typedef struct UX_CLASS_AUDIO10_AC_INPUT_TERMINAL_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bTerminalID;
    USHORT          wTerminalType;
    UCHAR           bAssocTerminal;
    UCHAR           bNrChannels;
    USHORT          wChannelConfig;
    UCHAR           iChannelNames;
    UCHAR           iTerminal;
} UX_CLASS_AUDIO10_AC_INPUT_TERMINAL_DESCRIPTOR, UX_CLASS_AUDIO10_AC_ITD;

/* Define Audio Class output terminal descriptor (OTD).  */

typedef struct UX_CLASS_AUDIO10_AC_OUTPUT_TERMINAL_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bTerminalID;
    USHORT          wTerminalType;
    UCHAR           bAssocTerminal;
    UCHAR           bSourceID;
    UCHAR           iTerminal;
} UX_CLASS_AUDIO10_AC_OUTPUT_TERMINAL_DESCRIPTOR, UX_CLASS_AUDIO10_AC_OTD;

/* Define Audio Class mixer unit descriptor (MUD, bNrInPins=1, bmMixerControls N=1).  */

typedef struct UX_CLASS_AUDIO10_AC_MIXER_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bUnitID;
    UCHAR           bNrInPins;
    UCHAR           baSourceID[1];
    UCHAR           bNrChannels;
    UCHAR           wChannelConfig[2];
    UCHAR           iChannelNames;
    UCHAR           bmControls[1];
    UCHAR           iMixer;
} UX_CLASS_AUDIO10_AC_MIXER_UNIT_DESCRIPTOR, UX_CLASS_AUDIO10_AC_MUD;

/* Define Audio Class selector unit descriptor (SUD, bNrInPins=1).  */

typedef struct UX_CLASS_AUDIO10_AC_SELECTOR_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bUnitID;
    UCHAR           bNrInPins;
    UCHAR           baSourceID[1];
    UCHAR           iSelector;
} UX_CLASS_AUDIO10_AC_SELECTOR_UNIT_DESCRIPTOR, UX_CLASS_AUDIO10_AC_SUD;

/* Define Audio Class feature unit descriptor (FUD, ch=1, bControlSize=2).  */

typedef struct UX_CLASS_AUDIO10_AC_FEATURE_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bUnitID;
    UCHAR           bSourceID;
    UCHAR           bControlSize;
    UCHAR           bmaControls[2 * 2];
    UCHAR           iFeature;
} UX_CLASS_AUDIO10_AC_FEATURE_UNIT_DESCRIPTOR, UX_CLASS_AUDIO10_AC_FUD;

/* FUD::bmControls.  */
#define UX_CLASS_AUDIO10_FUD_CONTROL_MUTE                               (1u << 0)
#define UX_CLASS_AUDIO10_FUD_CONTROL_VOLUME                             (1u << 1)
#define UX_CLASS_AUDIO10_FUD_CONTROL_BASS                               (1u << 2)
#define UX_CLASS_AUDIO10_FUD_CONTROL_MID                                (1u << 3)
#define UX_CLASS_AUDIO10_FUD_CONTROL_TREBLE                             (1u << 4)
#define UX_CLASS_AUDIO10_FUD_CONTROL_GRAPHIC_EQ                         (1u << 5)
#define UX_CLASS_AUDIO10_FUD_CONTROL_AUTO_GAIN                          (1u << 6)
#define UX_CLASS_AUDIO10_FUD_CONTROL_DELAY                              (1u << 7)
#define UX_CLASS_AUDIO10_FUD_CONTROL_BASS_BOOST                         (1u << 8)
#define UX_CLASS_AUDIO10_FUD_CONTROL_LOUDNESS                           (1u << 9)

/* Define Audio Class processing unit descriptor (PUD, bNrInPins=1,bControlSize=1).  */

typedef struct UX_CLASS_AUDIO10_AC_PROCESSING_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bUnitID;
    USHORT          wProcessType;
    UCHAR           bNrInPins;
    UCHAR           baSourceID[1];
    UCHAR           bNrChannels;
    UCHAR           wChannelConfig;
    UCHAR           iChannelNames;
    UCHAR           bControlSize;
    UCHAR           bmControls[1];
    UCHAR           iProcessing;
} UX_CLASS_AUDIO10_AC_PROCESSING_UNIT_DESCRIPTOR, UX_CLASS_AUDIO10_AC_PUD;

/* Define Audio Class up/down processing unit descriptor (PUD, bNrInPins=1,bControlSize=1,bNrModes=1).  */

typedef struct UX_CLASS_AUDIO10_AC_UP_DOWN_PROCESSING_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bUnitID;
    USHORT          wProcessType;
    UCHAR           bNrInPins;
    UCHAR           bSourceID;
    UCHAR           bNrChannels;
    UCHAR           wChannelConfig[2];
    UCHAR           iChannelNames;
    UCHAR           bControlSize;
    UCHAR           bmControls[1];
    UCHAR           iProcessing;
    UCHAR           bNrModes;
    UCHAR           waModes[2 * 1];
} UX_CLASS_AUDIO10_AC_UP_DOWN_PROCESSING_UNIT_DESCRIPTOR, UX_CLASS_AUDIO10_AC_UDD;

/* UDD::bmControls.  */
#define UX_CLASS_AUDIO10_UDD_CONTROL_ENABLE                             (1u << 0)
#define UX_CLASS_AUDIO10_UDD_CONTROL_MODE_SELECT                        (1u << 1)

/* Define Audio Class dolby prologic processing unit descriptor (DPD, bNrInPins=1,bControlSize=1,bNrModes=1).  */

typedef struct UX_CLASS_AUDIO10_AC_DOLBY_PROCESSING_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bUnitID;
    USHORT          wProcessType;
    UCHAR           bNrInPins;
    UCHAR           bSourceID;
    UCHAR           bNrChannels;
    UCHAR           wChannelConfig[2];
    UCHAR           iChannelNames;
    UCHAR           bControlSize;
    UCHAR           bmControls[1];
    UCHAR           iProcessing;
    UCHAR           bNrModes;
    UCHAR           daModes[2 * 1];
} UX_CLASS_AUDIO10_AC_DOLBY_PROCESSING_UNIT_DESCRIPTOR, UX_CLASS_AUDIO10_AC_DPD;

/* DPD::bmaControls.  */
#define UX_CLASS_AUDIO10_DPD_CONTROL_ENABLE                                 (1u << 0)
#define UX_CLASS_AUDIO10_DPD_CONTROL_MODE_SELECT                            (1u << 1)

/* Define Audio Class 3D Stereo Extender (3D) Processing Unit Descriptor.  */
typedef UX_CLASS_AUDIO10_AC_PUD UX_CLASS_AUDIO10_AC_3D_STEREO_EXT_PROCESSING_UNIT_DESCRIPTOR;
typedef UX_CLASS_AUDIO10_AC_PUD UX_CLASS_AUDIO10_AC_3DD;

/* 3DD::bmaControls.  */
#define UX_CLASS_AUDIO10_3DD_CONTROL_ENABLE                                 (1u << 0)
#define UX_CLASS_AUDIO10_3DD_CONTROL_MODE_SELECT                            (1u << 1)

/* Define Audio Class Reverberation (RV) Processing Unit Descriptor.  */
typedef UX_CLASS_AUDIO10_AC_PUD UX_CLASS_AUDIO10_AC_REVERBERATION_PROCESSING_UNIT_DESCRIPTOR;
typedef UX_CLASS_AUDIO10_AC_PUD UX_CLASS_AUDIO10_AC_RVD;

/* RVD::bmaControls.  */
#define UX_CLASS_AUDIO10_RVD_CONTROL_ENABLE                                 (1u << 0)
#define UX_CLASS_AUDIO10_RVD_CONTROL_REVERB_TYPE                            (1u << 1)
#define UX_CLASS_AUDIO10_RVD_CONTROL_REVERB_LEVEL                           (1u << 2)
#define UX_CLASS_AUDIO10_RVD_CONTROL_REVERB_TIME                            (1u << 3)
#define UX_CLASS_AUDIO10_RVD_CONTROL_REVERB_DELAY_FEEDBACK                  (1u << 4)

/* Define Audio Class Chorus (CH) Processing Unit Descriptor.  */
typedef UX_CLASS_AUDIO10_AC_PUD UX_CLASS_AUDIO10_AC_CHORUS_PROCESSING_UNIT_DESCRIPTOR;
typedef UX_CLASS_AUDIO10_AC_PUD UX_CLASS_AUDIO10_AC_CHORUSD;

/* CHORUSD::bmaControls.  */
#define UX_CLASS_AUDIO10_CHORUSD_CONTROL_ENABLE                             (1u << 0)
#define UX_CLASS_AUDIO10_CHORUSD_CONTROL_CHORUS_LEVEL                       (1u << 1)
#define UX_CLASS_AUDIO10_CHORUSD_CONTROL_CHORUS_MODULATION_RATE             (1u << 2)
#define UX_CLASS_AUDIO10_CHORUSD_CONTROL_CHORUS_MODULATION_DEPTH            (1u << 3)

/* Define Audio Class Dynamic Range Compressor (DR) Processing Unit Descriptor.  */
typedef UX_CLASS_AUDIO10_AC_PUD UX_CLASS_AUDIO10_AC_DYN_RNG_COMP_PROCESSING_UNIT_DESCRIPTOR;
typedef UX_CLASS_AUDIO10_AC_PUD UX_CLASS_AUDIO10_AC_DRD;

/* DRD::bmaControls.  */
#define UX_CLASS_AUDIO10_DRD_CONTROL_ENABLE                                 (1u << 0)
#define UX_CLASS_AUDIO10_DRD_CONTROL_COMPRESSION_RATIO                      (1u << 1)
#define UX_CLASS_AUDIO10_DRD_CONTROL_MAXAMPL                                (1u << 2)
#define UX_CLASS_AUDIO10_DRD_CONTROL_THRESHOLD                              (1u << 3)
#define UX_CLASS_AUDIO10_DRD_CONTROL_ATTACH_TIME                            (1u << 4)
#define UX_CLASS_AUDIO10_DRD_CONTROL_RELEASE_TIME                           (1u << 5)

/* Define Audio Class Extension unit descriptor (XUD, bNrInPins=1,bControlSize=1).  */

typedef struct UX_CLASS_AUDIO10_AC_EXTENSION_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bUnitID;
    USHORT          wExtensionCode;
    UCHAR           bNrInPins;
    UCHAR           baSourceID[1];
    UCHAR           bNrChannels;
    UCHAR           wChannelConfig[2];
    UCHAR           iChannelNames;
    UCHAR           bControlSize;
    UCHAR           bmControls[1];
    UCHAR           iExtension;
} UX_CLASS_AUDIO10_AC_EXTENSION_UNIT_DESCRIPTOR, UX_CLASS_AUDIO10_AC_XUD;

/* XUD::bmaControls.  */
#define UX_CLASS_AUDIO10_XUD_CONTROL_ENABLE                                 (1u << 0)

/* Define Audio Class Associated Interface descriptor (Association-specific x=1).  */

typedef struct UX_CLASS_AUDIO10_ASSOCIATED_INTERFACE_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bInterfaceNr;
    UCHAR           assoc_specific;
} UX_CLASS_AUDIO10_ASSOCIATED_INTERFACE_DESCRIPTOR;

/* Define Audio Class Interrupt/Isochronous Endpoint Descriptor.  */

typedef struct UX_CLASS_AUDIO10_ENDPOINT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bEndpointAddress;
    UCHAR           bmAttributes;
    UCHAR           wMaxPacketSize;
    UCHAR           bInterval;
    UCHAR           bRefresh;
    UCHAR           bSynchAddress;
} UX_CLASS_AUDIO10_ENDPOINT_DESCRIPTOR, UX_CLASS_AUDIO10_EPD;

/* Audio class-specific AS interface descriptor.  */

typedef struct UX_CLASS_AUDIO10_AS_INTERFACE_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bTerminalLink;
    UCHAR           bDelay;
    UCHAR           wFormatTag[2];
} UX_CLASS_AUDIO10_AS_INTERFACE_DESCRIPTOR, UX_CLASS_AUDIO10_AS_IFACED;


/* Audio class-specific isochronous audio data endpoint descriptor (EPD).  */

typedef struct UX_CLASS_AUDIO10_ISOCH_ENDPOINT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bmAttributes;
    UCHAR           bLockDelayUnits;
    UCHAR           wLockDelay[2];
} UX_CLASS_AUDIO10_ISOCH_ENDPOINT_DESCRIPTOR, UX_CLASS_AUDIO10_ISOCH_EPD;

/* EPD::bmAttributes.  */
#define UX_CLASS_AUDIO10_ISOCH_EPD_ATTR_SAMPLING_FREQ                       (0x1u << 0)
#define UX_CLASS_AUDIO10_ISOCH_EPD_ATTR_PITCH                               (0x1u << 1)
#define UX_CLASS_AUDIO10_ISOCH_EPD_ATTR_MAX_PACKET_SIZE_ONLY                (0x1u << 7)

/* EPD::bLockDelayUnits.  */
#define UX_CLASS_AUDIO10_EPD_LOCK_DELAY_UNITS_UNDEFINED                     0
#define UX_CLASS_AUDIO10_EPD_LOCK_DELAY_UNITS_MS                            1
#define UX_CLASS_AUDIO10_EPD_LOCK_DELAY_UNITS_DEC_PCM_SAMPLES               2


/* Audio class control request layout.  */

typedef struct UX_CLASS_AUDIO10_REQUEST_VALUE_CS_STRUCT {
    UCHAR   unused_zero;
    UCHAR   control_sel;        /* Control Selector      (CS)   */
} UX_CLASS_AUDIO10_REQUEST_VALUE_CS;

typedef struct UX_CLASS_AUDIO10_REQUEST_VALUE_MIXER_STRUCT {
    UCHAR   output_ch_num;      /* Output Channel Number (OCN)   */
    UCHAR   input_ch_num;       /* Input Channel Number  (ICN)   */
} UX_CLASS_AUDIO10_REQUEST_VALUE_MIXER;

typedef struct UX_CLASS_AUDIO10_REQUEST_VALUE_CONTROL_STRUCT {
    UCHAR   unused_zero;
    UCHAR   cs;                 /* CS       */
} UX_CLASS_AUDIO10_REQUEST_VALUE_CONTROL;

typedef struct UX_CLASS_AUDIO10_REQUEST_INDEX_EP_STRUCT {
    UCHAR   ep_addr;            /* Endpoint Address  */
    UCHAR   unused_zero;
} UX_CLASS_AUDIO10_REQUEST_INDEX_EP;

typedef struct UX_CLASS_AUDIO10_REQUEST_INDEX_INTERFACE_STRUCT {
    UCHAR   iface_num;          /* Interface number  */
    UCHAR   entity_id;
} UX_CLASS_AUDIO10_REQUEST_INDEX_INTERFACE;

typedef struct UX_CLASS_AUDIO10_REQUEST_INDEX_CONTROL_STRUCT {
    UCHAR   ep_iface;           /* Endpoint Address/Interface Number  */
    UCHAR   entity_id_zero;     /* Entity ID/Zero  */
} UX_CLASS_AUDIO10_REQUEST_INDEX_CONTROL;

typedef struct UX_CLASS_AUDIO10_REQUEST_STRUCT
{
    UCHAR           bmRequestType;
    UCHAR           bRequest;
    union UX_CLASS_AUDIO10_REQUEST_VALUE_UNION {
        USHORT      value;
        UX_CLASS_AUDIO10_REQUEST_VALUE_CS
                    control_cs;
        UX_CLASS_AUDIO10_REQUEST_VALUE_MIXER
                    control_mixer;
        UX_CLASS_AUDIO10_REQUEST_VALUE_CONTROL
                    control;
        USHORT      mem_offset;
    }               wValue;
    union UX_CLASS_AUDIO10_REQUEST_INDEX_UNION {
        USHORT      value;
        UX_CLASS_AUDIO10_REQUEST_INDEX_EP
                    ep;
        UX_CLASS_AUDIO10_REQUEST_INDEX_INTERFACE
                    iface;
        UX_CLASS_AUDIO10_REQUEST_INDEX_CONTROL
                    control;
    }               wIndex;
    USHORT          wLength;
} UX_CLASS_AUDIO10_REQUEST;

/* Audio Class 1.0 Copy Protect Control Parameter  */

typedef struct UX_CLASS_AUDIO10_COPY_PROTECT_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bCopyProtect;
} UX_CLASS_AUDIO10_COPY_PROTECT_CONTROL_PARAMETER;

#define UX_CLASS_AUDIO10_COPY_PROTECT_CONTROL_CPL0                          (1u << 0)
#define UX_CLASS_AUDIO10_COPY_PROTECT_CONTROL_CPL1                          (1u << 1)
#define UX_CLASS_AUDIO10_COPY_PROTECT_CONTROL_CPL2                          (1u << 2)

/* Audio Class 1.0 Mixer Control Parameter (specific control or 0xFF)  */

typedef struct UX_CLASS_AUDIO10_MIXER_CONTROL_PARAMETER_STRUCT
{
    USHORT          wMixer;
} UX_CLASS_AUDIO10_MIXER_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_MIXER_CONTROL_PARAMETER_FF_STRUCT
{
    USHORT          wMixer[1];
} UX_CLASS_AUDIO10_MIXER_CONTROL_PARAMETER_FF;

/* Audio Class 1.0 Selector Control Parameter  */

typedef struct UX_CLASS_AUDIO10_SELECTOR_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bSelector;
} UX_CLASS_AUDIO10_SELECTOR_CONTROL_PARAMETER;

/* Audio Class 1.0 Mute Control Parameter (specific CH or 0xFF)  */

typedef struct UX_CLASS_AUDIO10_MUTE_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bMute;
} UX_CLASS_AUDIO10_MUTE_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_MUTE_CONTROL_PARAMETER_FF_STRUCT
{
    UCHAR           bMute[1];
} UX_CLASS_AUDIO10_MUTE_CONTROL_PARAMETER_FF;

/* Audio Class 1.0 Volume Control Parameter (specific CH or 0xFF)  */

typedef struct UX_CLASS_AUDIO10_VOLUME_CONTROL_PARAMETER_STRUCT
{
    SHORT           wVolume;
} UX_CLASS_AUDIO10_VOLUME_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_VOLUME_CONTROL_PARAMETER_FF_STRUCT
{
    SHORT           wVolume[1];
} UX_CLASS_AUDIO10_VOLUME_CONTROL_PARAMETER_FF;

/* Audio Class 1.0 Bass Control Parameter (specific CH or 0xFF)  */

typedef struct UX_CLASS_AUDIO10_BASS_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bBass;
} UX_CLASS_AUDIO10_BASS_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_BASS_CONTROL_PARAMETER_FF_STRUCT
{
    UCHAR           bBass[1];
} UX_CLASS_AUDIO10_BASS_CONTROL_PARAMETER_FF;

/* Audio Class 1.0 Mid Control Parameter (specific CH or 0xFF)  */

typedef struct UX_CLASS_AUDIO10_MID_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bMid;
} UX_CLASS_AUDIO10_MID_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_MID_CONTROL_PARAMETER_FF_STRUCT
{
    UCHAR           bMid[1];
} UX_CLASS_AUDIO10_MID_CONTROL_PARAMETER_FF;

/* Audio Class 1.0 Treble Control Parameter (specific CH or 0xFF)  */

typedef struct UX_CLASS_AUDIO10_TREBLE_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bTreble;
} UX_CLASS_AUDIO10_TREBLE_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_TREBLE_CONTROL_PARAMETER_FF_STRUCT
{
    UCHAR           bTreble[1];
} UX_CLASS_AUDIO10_TREBLE_CONTROL_PARAMETER_FF;

/* Audio Class 1.0 Graphic Equalizer Control Parameter (NrBits=1)  */

typedef struct UX_CLASS_AUDIO10_GRAPHIC_EQ_CONTROL_PARAMETER_STRUCT
{
    ULONG           bmBandsPresent;
    UCHAR           bBand[1];
} UX_CLASS_AUDIO10_GRAPHIC_EQ_CONTROL_PARAMETER;

/* Audio Class 1.0 Automatic Gain Control Parameter (specific CH or 0xFF)  */

typedef struct UX_CLASS_AUDIO10_AG_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bAGC;
} UX_CLASS_AUDIO10_AG_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_AG_CONTROL_PARAMETER_FF_STRUCT
{
    UCHAR           bAGC[1];
} UX_CLASS_AUDIO10_AG_CONTROL_PARAMETER_FF;

/* Audio Class 1.0 Delay Control Parameter (specific CH or 0xFF)  */

typedef struct UX_CLASS_AUDIO10_DELAY_CONTROL_PARAMETER_STRUCT
{
    SHORT           wDelay;
} UX_CLASS_AUDIO10_DELAY_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_DELAY_CONTROL_PARAMETER_FF_STRUCT
{
    SHORT           wDelay[1];
} UX_CLASS_AUDIO10_DELAY_CONTROL_PARAMETER_FF;

/* Audio Class 1.0 BassBoost Control Parameter (specific CH or 0xFF)  */

typedef struct UX_CLASS_AUDIO10_BASSBOOST_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bBassBoost;
} UX_CLASS_AUDIO10_BASSBOOST_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_BASSBOOST_CONTROL_PARAMETER_FF_STRUCT
{
    UCHAR           bBassBoost[1];
} UX_CLASS_AUDIO10_BASSBOOST_CONTROL_PARAMETER_FF;

/* Audio Class 1.0 Loudness Control Parameter (specific CH or 0xFF)  */

typedef struct UX_CLASS_AUDIO10_LOUDNESS_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bLoudness;
} UX_CLASS_AUDIO10_LOUDNESS_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_LOUDNESS_CONTROL_PARAMETER_FF_STRUCT
{
    UCHAR           bLoudness[1];
} UX_CLASS_AUDIO10_LOUDNESS_CONTROL_PARAMETER_FF;

/* Audio Class 1.0 Enable Processing Control Parameter  */

typedef struct UX_CLASS_AUDIO10_ENABLE_PROCESSING_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bEnable;
} UX_CLASS_AUDIO10_ENABLE_PROCESSING_CONTROL_PARAMETER;

/* Audio Class 1.0 Spaciousness Control Parameter  */

typedef struct UX_CLASS_AUDIO10_SPACIOUSNESS_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bSpaciousness;
} UX_CLASS_AUDIO10_SPACIOUSNESS_CONTROL_PARAMETER;

/* Audio Class 1.0 ReverbType Control Parameter  */

typedef struct UX_CLASS_AUDIO10_REVERB_TYPE_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bReverbType;
} UX_CLASS_AUDIO10_REVERB_TYPE_CONTROL_PARAMETER;

#define UX_CLASS_AUDIO10_REVERB_TYPE_ROOM_1                             0
#define UX_CLASS_AUDIO10_REVERB_TYPE_SMALL_ROOM                         0
#define UX_CLASS_AUDIO10_REVERB_TYPE_ROOM_2                             1
#define UX_CLASS_AUDIO10_REVERB_TYPE_MEDIUM_ROOM                        1
#define UX_CLASS_AUDIO10_REVERB_TYPE_ROOM_3                             2
#define UX_CLASS_AUDIO10_REVERB_TYPE_LARGE_ROOM                         2
#define UX_CLASS_AUDIO10_REVERB_TYPE_HALL_1                             3
#define UX_CLASS_AUDIO10_REVERB_TYPE_MEDIUM_CONCERT_HALL                3
#define UX_CLASS_AUDIO10_REVERB_TYPE_HALL_2                             4
#define UX_CLASS_AUDIO10_REVERB_TYPE_LARGE_CONCERT_HALL                 4
#define UX_CLASS_AUDIO10_REVERB_TYPE_PLATE                              5
#define UX_CLASS_AUDIO10_REVERB_TYPE_PLATE_REVERBERATION                5
#define UX_CLASS_AUDIO10_REVERB_TYPE_DELAY                              6
#define UX_CLASS_AUDIO10_REVERB_TYPE_PANNING_DELAY                      7

/* Audio Class 1.0 ReverbLevel Control Parameter  */

typedef struct UX_CLASS_AUDIO10_REVERB_LEVEL_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bReverbLevel;
} UX_CLASS_AUDIO10_REVERB_LEVEL_CONTROL_PARAMETER;

/* Audio Class 1.0 ReverTime Control Parameter  */

typedef struct UX_CLASS_AUDIO10_REVERB_TIME_CONTROL_PARAMETER_STRUCT
{
    USHORT          wReverTime;
} UX_CLASS_AUDIO10_REVERB_TIME_CONTROL_PARAMETER;

/* Audio Class 1.0 ReverbDelayFeedback Control Parameter  */

typedef struct UX_CLASS_AUDIO10_REVERB_FEEDBACK_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bReverbFeedback;
} UX_CLASS_AUDIO10_REVERB_FEEDBACK_CONTROL_PARAMETER;

/* Audio Class 1.0 ChorusLevel Control Parameter  */

typedef struct UX_CLASS_AUDIO10_CHORUS_LEVEL_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bChorusLevel;
} UX_CLASS_AUDIO10_CHORUS_LEVEL_CONTROL_PARAMETER;

/* Audio Class 1.0 ChorusRate Control Parameter  */

typedef struct UX_CLASS_AUDIO10_CHORUS_RATE_CONTROL_PARAMETER_STRUCT
{
    USHORT          wChorusRate;
} UX_CLASS_AUDIO10_CHORUS_RATE_CONTROL_PARAMETER;

/* Audio Class 1.0 ChorusModulationDepth Control Parameter  */

typedef struct UX_CLASS_AUDIO10_CHORUS_DEPTH_CONTROL_PARAMETER_STRUCT
{
    USHORT          wChorusDepth;
} UX_CLASS_AUDIO10_CHORUS_DEPTH_CONTROL_PARAMETER;

/* Audio Class 1.0 Dynamic Range Compressor Compression Ratio Control Parameter  */

typedef struct UX_CLASS_AUDIO10_COMPRESSION_RATIO_CONTROL_PARAMETER_STRUCT
{
    USHORT          wRatio;
} UX_CLASS_AUDIO10_COMPRESSION_RATIO_CONTROL_PARAMETER;

/* Audio Class 1.0 Dynamic Range Compressor MaxAmpl Control Parameter  */

typedef struct UX_CLASS_AUDIO10_MAXAMPL_CONTROL_PARAMETER_STRUCT
{
    USHORT          wMaxAmpl;
} UX_CLASS_AUDIO10_MAXAMPL_CONTROL_PARAMETER;

/* Audio Class 1.0 Dynamic Range Compressor Threshold Control Parameter  */

typedef struct UX_CLASS_AUDIO10_THRESHOLD_CONTROL_PARAMETER_STRUCT
{
    USHORT          wThreshold;
} UX_CLASS_AUDIO10_THRESHOLD_CONTROL_PARAMETER;

/* Audio Class 1.0 Dynamic Range Compressor AttachTime Control Parameter  */

typedef struct UX_CLASS_AUDIO10_ATTACK_TIME_CONTROL_PARAMETER_STRUCT
{
    USHORT          wAttachTime;
} UX_CLASS_AUDIO10_ATTACK_TIME_CONTROL_PARAMETER;

/* Audio Class 1.0 Dynamic Range Compressor ReleaseTime Control Parameter  */

typedef struct UX_CLASS_AUDIO10_RELEASE_TIME_CONTROL_PARAMETER_STRUCT
{
    USHORT          wReleaseTime;
} UX_CLASS_AUDIO10_RELEASE_TIME_CONTROL_PARAMETER;

/* Audio Class 1.0 Enable Processing Control (XU_ENABLE_CONTROL) Parameter  */

typedef struct UX_CLASS_AUDIO10_XU_ENABLE_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bOn;
} UX_CLASS_AUDIO10_XU_ENABLE_CONTROL_PARAMETER;

/* Audio Class 1.0 Sampling Frequency Control (SAMPLING_FREQ_CONTROL) Parameter  */

typedef struct UX_CLASS_AUDIO10_SAMPLING_FREQ_CONTROL_PARAMETER_STRUCT
{
    UCHAR           tSampleFreq[3];
} UX_CLASS_AUDIO10_SAMPLING_FREQ_CONTROL_PARAMETER;

/* Audio Class 1.0 Pitch Control (PITCH_CONTROL) Parameter  */

typedef struct UX_CLASS_AUDIO10_PITCH_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bPitchEnable;
} UX_CLASS_AUDIO10_PITCH_CONTROL_PARAMETER;

/* Audio Class 1.0 Continuous Sampling Frequency.  */

typedef struct UX_CLASS_AUDIO10_CONTINUOUS_SAMPLING_FREQ_STRUCT
{
    UCHAR           tLowerSamFreq[3];
    UCHAR           tUpperSamFreq[3];
} UX_CLASS_AUDIO10_CONTINUOUS_SAMPLING_FREQ;

/* Audio Class 1.0 Discrete Sampling Frequency (N=1).  */

typedef struct UX_CLASS_AUDIO10_DISCRETE_SAMPLING_FREQ_STRUCT
{
    UCHAR           tSamFreq[3 * 1];
} UX_CLASS_AUDIO10_DISCRETE_SAMPLING_FREQ;

/* Audio Class 1.0 Type I Format Type Descriptor  */

typedef struct UX_CLASS_AUDIO10_TYPE_I_FORMAT_TYPE_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bFormatType;
    UCHAR           bNrChannels;
    UCHAR           bSubframeSize;
    UCHAR           bBitResolution;
    UCHAR           bSamFreqType;
} UX_CLASS_AUDIO10_TYPE_I_FORMAT_TYPE_DESCRIPTOR;

typedef struct UX_CLASS_AUDIO10_TYPE_I_FORMAT_TYPE_CONTINUOUS_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bFormatType;
    UCHAR           bNrChannels;
    UCHAR           bSubframeSize;
    UCHAR           bBitResolution;
    UCHAR           bSamFreqType;       /* 0  */
    UCHAR           tLowerSamFreq[3];
    UCHAR           tUpperSamFreq[3];
}
UX_CLASS_AUDIO10_TYPE_I_FORMAT_TYPE_CONTINUOUS_DESCRIPTOR, 
UX_CLASS_AUDIO10_TYPE_I_FORMAT_TYPE_DESCRIPTOR_0;

typedef struct UX_CLASS_AUDIO10_TYPE_I_FORMAT_TYPE_DISCRETE_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bFormatType;
    UCHAR           bNrChannels;
    UCHAR           bSubframeSize;
    UCHAR           bBitResolution;
    UCHAR           bSamFreqType;       /* 1  */
    UCHAR           tSamFreq[3 * 1];
}
UX_CLASS_AUDIO10_TYPE_I_FORMAT_TYPE_DISCRETE_DESCRIPTOR,
UX_CLASS_AUDIO10_TYPE_I_FORMAT_TYPE_DESCRIPTOR_1;

typedef struct UX_CLASS_AUDIO10_TYPE_II_FORMAT_TYPE_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bFormatType;
    USHORT          wMaxBitRate;
    USHORT          wSamplesPerFrame;
    UCHAR           bSamFreqType;
} UX_CLASS_AUDIO10_TYPE_II_FORMAT_TYPE_DESCRIPTOR;

typedef struct UX_CLASS_AUDIO10_TYPE_II_FORMAT_TYPE_CONTINUOUS_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bFormatType;
    USHORT          wMaxBitRate;
    USHORT          wSamplesPerFrame;
    UCHAR           bSamFreqType;       /* 0  */
    UCHAR           tLowerSamFreq[3];
    UCHAR           tUpperSamFreq[3];
}
UX_CLASS_AUDIO10_TYPE_II_FORMAT_TYPE_CONTINUOUS_DESCRIPTOR,
UX_CLASS_AUDIO10_TYPE_II_FORMAT_TYPE_DESCRIPTOR_0;

typedef struct UX_CLASS_AUDIO10_TYPE_II_FORMAT_TYPE_DISCRETE_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bFormatType;
    USHORT          wMaxBitRate;
    USHORT          wSamplesPerFrame;
    UCHAR           bSamFreqType;       /* 1  */
    UCHAR           tSamFreq[3 * 1];
}
UX_CLASS_AUDIO10_TYPE_II_FORMAT_TYPE_DISCRETE_DESCRIPTOR,
UX_CLASS_AUDIO10_TYPE_II_FORMAT_TYPE_DESCRIPTOR_1;


/* Audio Class 1.0 MPEG (MP) Format-Specific Descriptor (MPEGD)  */

typedef struct UX_CLASS_AUDIO10_MPEG_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           wFormatTag[2];
    UCHAR           bmMPEGCapabilities[2];
    UCHAR           bmMPEGFeatures;
} UX_CLASS_AUDIO10_MPEG_DESCRIPTOR, UX_CLASS_AUDIO10_MPEGD;

/* MPEG::bmMPEGCapabilities.  */
#define UX_CLASS_AUDIO10_MPEGD_CAP_LAYER_MASK                           (0x7u << 0)
#define UX_CLASS_AUDIO10_MPEGD_CAP_LAYER_I                              (0x1u << 0)
#define UX_CLASS_AUDIO10_MPEGD_CAP_LAYER_II                             (0x1u << 1)
#define UX_CLASS_AUDIO10_MPEGD_CAP_LAYER_III                            (0x1u << 2)
#define UX_CLASS_AUDIO10_MPEGD_CAP_MPEG1_ONLY                           (0x1u << 3)
#define UX_CLASS_AUDIO10_MPEGD_CAP_MPEG1_DUAL_CH                        (0x1u << 4)
#define UX_CLASS_AUDIO10_MPEGD_CAP_MPEG2_STEREO                         (0x1u << 5)
#define UX_CLASS_AUDIO10_MPEGD_CAP_MPEG2_7_1_CH                         (0x1u << 6)
#define UX_CLASS_AUDIO10_MPEGD_CAP_ADAPTIVE_MULTI_CH_PREDICT            (0x1u << 7)
#define UX_CLASS_AUDIO10_MPEGD_CAP_MPEG2_MULTILINGUAL_MASK              (0x3u << 8)
#define UX_CLASS_AUDIO10_MPEGD_CAP_MPEG2_MULTI_NOT_SUPPORT              (0x0u << 8)
#define UX_CLASS_AUDIO10_MPEGD_CAP_MPEG2_MULTI_FS                       (0x1u << 8)
#define UX_CLASS_AUDIO10_MPEGD_CAP_MPEG2_MULTI_FS_HALF_FS               (0x3u << 8)

/* MPEG::bmMPEGFeatures.  */
#define UX_CLASS_AUDIO10_MPEGD_FEAT_IDYN_RNG_CTRL_MASK                  (0x3u << 4) /* Internal Dynamic Range Control  */
#define UX_CLASS_AUDIO10_MPEGD_FEAT_IDYN_RNG_CTRL_NOT_SUP               (0x0u << 4) /* Not support  */
#define UX_CLASS_AUDIO10_MPEGD_FEAT_IDYN_RNG_CTRL_NOT_SCAL              (0x1u << 4) /* Not scalable  */
#define UX_CLASS_AUDIO10_MPEGD_FEAT_IDYN_RNG_CTRL_SCAL_COMMON           (0x2u << 4) /* Common boost and cut scaling value  */
#define UX_CLASS_AUDIO10_MPEGD_FEAT_IDYN_RNG_CTRL_SCAL_SEPARA           (0x3u << 4) /* Separate boost and cut scaling value  */

/* Audio Class 1.0 Dual Channel Control  */

typedef struct UX_CLASS_AUDIO10_MP_DUAL_CHANNEL_CONTROL_PARAMETER_STRUCT
{
    UCHAR           BChannel2Enable;
} UX_CLASS_AUDIO10_MP_DUAL_CHANNEL_CONTROL_PARAMETER;

/* Audio Class 1.0 Second Stereo Control  */

typedef struct UX_CLASS_AUDIO10_MP_2ND_STEREO_CONTROL_PARAMETER_STRUCT
{
    UCHAR           B2ndStereoEnable;
} UX_CLASS_AUDIO10_MP_2ND_STEREO_CONTROL_PARAMETER;

/* Audio Class 1.0 Multilingual Control  */

typedef struct UX_CLASS_AUDIO10_MP_MULTILINGUAL_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bMultiLingual;
} UX_CLASS_AUDIO10_MP_MULTILINGUAL_CONTROL_PARAMETER;

/* bMultiLingual  */
#define UX_CLASS_AUDIO10_MP_MULTILINGUAL_DECODE_NO_CHANNEL              0
#define UX_CLASS_AUDIO10_MP_MULTILINGUAL_DECODE_CHANNEL_1               1
#define UX_CLASS_AUDIO10_MP_MULTILINGUAL_DECODE_CHANNEL_2               2
#define UX_CLASS_AUDIO10_MP_MULTILINGUAL_DECODE_CHANNEL_3               3
#define UX_CLASS_AUDIO10_MP_MULTILINGUAL_DECODE_CHANNEL_4               4
#define UX_CLASS_AUDIO10_MP_MULTILINGUAL_DECODE_CHANNEL_5               5
#define UX_CLASS_AUDIO10_MP_MULTILINGUAL_DECODE_CHANNEL_6               6
#define UX_CLASS_AUDIO10_MP_MULTILINGUAL_DECODE_CHANNEL_7               7

/* Audio Class 1.0 Dynamic Range Control (MP_DYN_RANGE_CONTROL)  */

typedef struct UX_CLASS_AUDIO10_MP_DYN_RANGE_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bEnable;
} UX_CLASS_AUDIO10_MP_DYN_RANGE_CONTROL_PARAMETER;

/* Audio Class 1.0 Scaling Control  */

typedef struct UX_CLASS_AUDIO10_MP_SCALING_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bScale;
} UX_CLASS_AUDIO10_MP_SCALING_CONTROL_PARAMETER;

/* Audio Class 1.0 High/Low (HILO) Scaling Control  */

typedef struct UX_CLASS_AUDIO10_MP_HILO_SCALING_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bLowScale;
    UCHAR           bHighScale;
} UX_CLASS_AUDIO10_MP_HILO_SCALING_CONTROL_PARAMETER;


/* Audio Class 1.0 AC-3 Format Descriptor  */

typedef struct UX_CLASS_AUDIO10_AS_AC_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           wFormatTag[2];
    UCHAR           bmBSID[4];
    UCHAR           bmAC3Features;
} UX_CLASS_AUDIO10_AS_AC_DESCRIPTOR, UX_CLASS_AUDIO10_AS_ACD;

/* AC3 DECODER::bmAC3Features.  */
#define UX_CLASS_AUDIO10_ACD_FEAT_RF_MODE                           (0x1u << 0)
#define UX_CLASS_AUDIO10_ACD_FEAT_LINE_MODE                         (0x1u << 1)
#define UX_CLASS_AUDIO10_ACD_FEAT_CUSTOM0_MODE                      (0x1u << 2)
#define UX_CLASS_AUDIO10_ACD_FEAT_CUSTOM1_MODE                      (0x1u << 3)
#define UX_CLASS_AUDIO10_ACD_FEAT_IDYN_RNG_CTRL_MASK                (0x3u << 4) /* Internal Dynamic Range Control  */
#define UX_CLASS_AUDIO10_ACD_FEAT_IDYN_RNG_CTRL_NOT_SUP             (0x0u << 4) /* Not support  */
#define UX_CLASS_AUDIO10_ACD_FEAT_IDYN_RNG_CTRL_NOT_SCAL            (0x3u << 4) /* Not scalable  */
#define UX_CLASS_AUDIO10_ACD_FEAT_IDYN_RNG_CTRL_SCAL_COMMON         (0x3u << 4) /* Common boost and cut scaling value  */
#define UX_CLASS_AUDIO10_ACD_FEAT_IDYN_RNG_CTRL_SCAL_SEPARATE       (0x3u << 4) /* Separate boost and cut scaling value  */

/* Audio Class 1.0 AC-3 Mode Control (AC_MODE_CONTROL)  */

typedef struct UX_CLASS_AUDIO10_AS_AC_MODE_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bMode;
} UX_CLASS_AUDIO10_AS_AC_MODE_CONTROL_PARAMETER;

/* AC-3 Compression Mode Control.  */
#define UX_CLASS_AUDIO10_AC_MODE_RF                                 (0x1u << 0)
#define UX_CLASS_AUDIO10_AC_MODE_LINE                               (0x1u << 1)
#define UX_CLASS_AUDIO10_AC_MODE_CUSTOM0                            (0x1u << 2)
#define UX_CLASS_AUDIO10_AC_MODE_CUSTOM1                            (0x1u << 3)

/* Audio Class 1.0 AC-3 Dynamic Range Control (AC_DYN_RANGE_CONTROL)  */

typedef struct UX_CLASS_AUDIO10_AS_AC_DYN_RANGE_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bMode;
} UX_CLASS_AUDIO10_AS_AC_DYN_RANGE_CONTROL_PARAMETER;

/* Audio Class 1.0 AC-3 Scaling Control (AC_SCALING_CONTROL)  */

typedef struct UX_CLASS_AUDIO10_AS_AC_SCALING_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bScale;
} UX_CLASS_AUDIO10_AS_AC_SCALING_CONTROL_PARAMETER;

/* Audio Class 1.0 AC-3 High/Low Scaling Control (AC_HILO_SCALING_CONTROL)  */

typedef struct UX_CLASS_AUDIO10_AS_AC_HILO_SCALING_CONTROL_PARAMETER_STRUCT
{
    UCHAR           bLowScale;
    UCHAR           bHighScale;
} UX_CLASS_AUDIO10_AS_AC_HILO_SCALING_CONTROL_PARAMETER;

typedef struct UX_CLASS_AUDIO10_TYPE_III_FORMAT_TYPE_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bFormatType;
    UCHAR           bNrChannels;
    UCHAR           bSubframeSize;
    UCHAR           bBitResolution;
    UCHAR           bSamFreqType;
} UX_CLASS_AUDIO10_TYPE_III_FORMAT_TYPE_DESCRIPTOR;

typedef struct UX_CLASS_AUDIO10_TYPE_III_FORMAT_TYPE_CONTINUOUS_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bFormatType;
    UCHAR           bNrChannels;
    UCHAR           bSubframeSize;
    UCHAR           bBitResolution;
    UCHAR           bSamFreqType;   /* 0  */
    UCHAR           tLowerSamFreq[3];
    UCHAR           tUpperSamFreq[3];
}
UX_CLASS_AUDIO10_TYPE_III_FORMAT_TYPE_CONTINUOUS_DESCRIPTOR,
UX_CLASS_AUDIO10_TYPE_III_FORMAT_TYPE_DESCRIPTOR_0;

typedef struct UX_CLASS_AUDIO10_TYPE_III_FORMAT_TYPE_DISCRETE_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bFormatType;
    UCHAR           bNrChannels;
    UCHAR           bSubframeSize;
    UCHAR           bBitResolution;
    UCHAR           bSamFreqType;   /* 1  */
    UCHAR           tSamFreq[3 * 1];
}
UX_CLASS_AUDIO10_TYPE_III_FORMAT_TYPE_DISCRETE_DESCRIPTOR,
UX_CLASS_AUDIO10_TYPE_III_FORMAT_TYPE_DESCRIPTOR_1;

/* Audio Class 1.0 Interrupt Status Word  */
typedef struct UX_CLASS_AUDIO10_INT_STATUS_STRUCT
{
    UCHAR           bStatusType;
    UCHAR           bOriginator;
} UX_CLASS_AUDIO10_INT_STATUS;

/* Status Word::bStatusType.  */
#define UX_CLASS_AUDIO10_INT_STATUS_TYPE_PENDING                    (0x1u << 7)
#define UX_CLASS_AUDIO10_INT_STATUS_TYPE_MEM_CHANGED                (0x1u << 6)
#define UX_CLASS_AUDIO10_INT_STATUS_TYPE_ORIGINATOR_MASK            (0xFu << 0)
#define UX_CLASS_AUDIO10_INT_STATUS_TYPE_ORIGINATOR_AC              (0x0u << 0) /* AudioControl interface    */
#define UX_CLASS_AUDIO10_INT_STATUS_TYPE_ORIGINATOR_AS              (0x1u << 0) /* AudioStreaming interface  */
#define UX_CLASS_AUDIO10_INT_STATUS_TYPE_ORIGINATOR_AS_EP           (0x2u << 0) /* AudioStreaming endpoint   */


#endif
