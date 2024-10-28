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
/*    ux_class_audio20.h                                  PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file contains all the header and structures used by the        */
/*    USBX Audio Class (UAC) 2.0.                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/

#ifndef UX_CLASS_AUDIO20_H
#define UX_CLASS_AUDIO20_H

/* Define Audio Class Codes.  */

#define UX_CLASS_AUDIO20_IP_VERSION_02_00                  0x20

#define UX_CLASS_AUDIO20_CLASS                             0x01
#define UX_CLASS_AUDIO20_SUBCLASS_UNDEFINED                0x00
#define UX_CLASS_AUDIO20_SUBCLASS_AUDIOCONTROL             0x01
#define UX_CLASS_AUDIO20_SUBCLASS_AUDIOSTREAMING           0x02
#define UX_CLASS_AUDIO20_SUBCLASS_MIDISTREAMING            0x03
#define UX_CLASS_AUDIO20_PROTOCOL_UNDEFINED                0x00
#define UX_CLASS_AUDIO20_PROTOCOL_02_00                    UX_CLASS_AUDIO20_IP_VERSION_02_00


/* Define Audio Function (AF) code.  */

#define UX_CLASS_AUDIO20_AF_VERSION_02_00                  UX_CLASS_AUDIO20_IP_VERSION_02_00


/* Define Audio Class function category codes.  */

#define UX_CLASS_AUDIO20_CATEGORY_UNDEFINED                0x00
#define UX_CLASS_AUDIO20_CATEGORY_DESKTOP_SPEAKER          0x01
#define UX_CLASS_AUDIO20_CATEGORY_HOME_THEATER             0x02
#define UX_CLASS_AUDIO20_CATEGORY_MICROPHONE               0x03
#define UX_CLASS_AUDIO20_CATEGORY_HEADSET                  0x04
#define UX_CLASS_AUDIO20_CATEGORY_TELEPHONE                0x05
#define UX_CLASS_AUDIO20_CATEGORY_CONVERTER                0x06
#define UX_CLASS_AUDIO20_CATEGORY_VOICE_SOUND_RECORDER     0x07
#define UX_CLASS_AUDIO20_CATEGORY_I_O_BOX                  0x08
#define UX_CLASS_AUDIO20_CATEGORY_MUSICAL_INSTRUMENT       0x09
#define UX_CLASS_AUDIO20_CATEGORY_PRO_AUDIO                0x0A
#define UX_CLASS_AUDIO20_CATEGORY_AUDIO_VIDEO              0x0B
#define UX_CLASS_AUDIO20_CATEGORY_CONTROL_PANEL            0x0C
#define UX_CLASS_AUDIO20_CATEGORY_OTHER                    0xFF


/* Define Audio Class desctiptor types.  */
#define UX_CLASS_AUDIO20_CS_UNDEFINED                      0x20
#define UX_CLASS_AUDIO20_CS_DEVICE                         0x21
#define UX_CLASS_AUDIO20_CS_CONFIGURATION                  0x22
#define UX_CLASS_AUDIO20_CS_STRING                         0x23
#define UX_CLASS_AUDIO20_CS_INTERFACE                      0x24
#define UX_CLASS_AUDIO20_CS_ENDPOINT                       0x25


/* Define Audio Class AC interface descriptor subclasses.  */

#define UX_CLASS_AUDIO20_AC_UNDEFINED                      0x00
#define UX_CLASS_AUDIO20_AC_HEADER                         0x01
#define UX_CLASS_AUDIO20_AC_INPUT_TERMINAL                 0x02
#define UX_CLASS_AUDIO20_AC_OUTPUT_TERMINAL                0x03
#define UX_CLASS_AUDIO20_AC_MIXER_UNIT                     0x04
#define UX_CLASS_AUDIO20_AC_SELECTOR_UNIT                  0x05
#define UX_CLASS_AUDIO20_AC_FEATURE_UNIT                   0x06
#define UX_CLASS_AUDIO20_AC_EFFECT_UNIT                    0x07
#define UX_CLASS_AUDIO20_AC_PROCESSING_UNIT                0x08
#define UX_CLASS_AUDIO20_AC_EXTENSION_UNIT                 0x09
#define UX_CLASS_AUDIO20_AC_CLOCK_SOURCE                   0x0A
#define UX_CLASS_AUDIO20_AC_CLOCK_SELECTOR                 0x0B
#define UX_CLASS_AUDIO20_AC_CLOCK_MULTIPLIER               0x0C
#define UX_CLASS_AUDIO20_AC_SAMPLE_RATE_CONVERTER          0x0D

/* Define Audio Class Effect Unit (EU) Effect Types (ET).  */

#define UX_CLASS_AUDIO20_EFFECT_UNDEFINED                  0x00
#define UX_CLASS_AUDIO20_EFFECT_PARAM_EQ_SECTION           0x01
#define UX_CLASS_AUDIO20_EFFECT_REVERBERATION              0x02
#define UX_CLASS_AUDIO20_EFFECT_MOD_DELAY                  0x03
#define UX_CLASS_AUDIO20_EFFECT_DYN_RANGE_COMP             0x04

/* Define Audio Class Processing Unit (PU) Process Types (PT).  */

#define UX_CLASS_AUDIO20_PROCESS_UNDEFINED                 0x00
#define UX_CLASS_AUDIO20_PROCESS_UP_DOWN_MIX               0x01
#define UX_CLASS_AUDIO20_PROCESS_DOLBY_PROLOGIC            0x02
#define UX_CLASS_AUDIO20_PROCESS_STEREO_EXTENDER           0x03


/* Define Audio Class AS interface descriptor subclasses.  */

#define UX_CLASS_AUDIO20_AS_UNDEFINED                      0x00
#define UX_CLASS_AUDIO20_AS_GENERAL                        0x01
#define UX_CLASS_AUDIO20_AS_FORMAT_TYPE                    0x02
#define UX_CLASS_AUDIO20_AS_ENCODER                        0x03
#define UX_CLASS_AUDIO20_AS_DECODER                        0x04


/* Define Audio Class endpoint descriptor subtypes.  */

#define UX_CLASS_AUDIO20_EP_UNDEFINED                      0x00
#define UX_CLASS_AUDIO20_EP_GENERAL                        0x01


/* Define Audio Class Encoder Type codes.  */

#define UX_CLASS_AUDIO20_ENCODER_UNDEFINED                 0x00
#define UX_CLASS_AUDIO20_ENCODER_OTHER                     0x01
#define UX_CLASS_AUDIO20_ENCODER_MPEG                      0x02
#define UX_CLASS_AUDIO20_ENCODER_AC3                       0x03
#define UX_CLASS_AUDIO20_ENCODER_WMA                       0x04
#define UX_CLASS_AUDIO20_ENCODER_DTS                       0x05

/* Define Audio Class Decoder Type codes.  */

#define UX_CLASS_AUDIO20_DECODER_UNDEFINED                 0x00
#define UX_CLASS_AUDIO20_DECODER_OTHER                     0x01
#define UX_CLASS_AUDIO20_DECODER_MPEG                      0x02
#define UX_CLASS_AUDIO20_DECODER_AC3                       0x03
#define UX_CLASS_AUDIO20_DECODER_WMA                       0x04
#define UX_CLASS_AUDIO20_DECODER_DTS                       0x05


/* Define Audio Class request codes.  */

#define UX_CLASS_AUDIO20_REQUEST_CODE_UNDEFINED            0x00
#define UX_CLASS_AUDIO20_CUR                               0x01
#define UX_CLASS_AUDIO20_RANGE                             0x02
#define UX_CLASS_AUDIO20_MEM                               0x03


/* Define Audio Class clock source control selectors.  */

#define UX_CLASS_AUDIO20_CS_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_CS_SAM_FREQ_CONTROL               0x01
#define UX_CLASS_AUDIO20_CS_CLOCK_VALID_CONTROL            0x02

/* Define Audio Class clock selector control selectors.  */

#define UX_CLASS_AUDIO20_CX_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_CX_CLOCK_SELECTOR_CONTROL         0x01

/* Define Audio Class clock multiplier control selectors.  */

#define UX_CLASS_AUDIO20_CM_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_CM_NUMERATOR_CONTROL              0x01
#define UX_CLASS_AUDIO20_CM_DENOMINATOR_CONTROL            0x02

/* Define Audio Class terminal control selectors.  */

#define UX_CLASS_AUDIO20_TE_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_TE_COPY_PROTECT_CONTROL           0x01
#define UX_CLASS_AUDIO20_TE_CONNECTOR_CONTROL              0x02
#define UX_CLASS_AUDIO20_TE_OVERLOAD_CONTROL               0x03
#define UX_CLASS_AUDIO20_TE_CLUSTER_CONTROL                0x04
#define UX_CLASS_AUDIO20_TE_UNDERFLOW_CONTROL              0x05
#define UX_CLASS_AUDIO20_TE_OVERFLOW_CONTROL               0x06
#define UX_CLASS_AUDIO20_TE_LATENCY_CONTROL                0x07

/* Define Audio Class mixer control selectors.  */

#define UX_CLASS_AUDIO20_MU_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_MU_MIXER_CONTROL                  0x01
#define UX_CLASS_AUDIO20_MU_CLUSTER_CONTROL                0x02
#define UX_CLASS_AUDIO20_MU_UNDERFLOW_CONTROL              0x03
#define UX_CLASS_AUDIO20_MU_OVERFLOW_CONTROL               0x04
#define UX_CLASS_AUDIO20_MU_LATENCY_CONTROL                0x05

/* Define Audio Class selector control selectors.  */

#define UX_CLASS_AUDIO20_SU_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_SU_SELECTOR_CONTROL               0x01
#define UX_CLASS_AUDIO20_SU_LATENCY_CONTROL                0x02

/* Define Audio Class feature unit control selectors.  */

#define UX_CLASS_AUDIO20_FU_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_FU_MUTE_CONTROL                   0x01
#define UX_CLASS_AUDIO20_FU_VOLUME_CONTROL                 0x02
#define UX_CLASS_AUDIO20_FU_BASS_CONTROL                   0x03
#define UX_CLASS_AUDIO20_FU_MID_CONTROL                    0x04
#define UX_CLASS_AUDIO20_FU_TREBLE_CONTROL                 0x05
#define UX_CLASS_AUDIO20_FU_GRAPHIC_EQUALIZER_CONTROL      0x06
#define UX_CLASS_AUDIO20_FU_AUTOMATIC_GAIN_CONTROL         0x07
#define UX_CLASS_AUDIO20_FU_DELAY_CONTROL                  0x08
#define UX_CLASS_AUDIO20_FU_BASS_BOOST_CONTROL             0x09
#define UX_CLASS_AUDIO20_FU_LOUNDNESS_CONTROL              0x0A
#define UX_CLASS_AUDIO20_FU_INPUT_GAIN_CONTROL             0x0B
#define UX_CLASS_AUDIO20_FU_INPUT_GAIN_PAD_CONTROL         0x0C
#define UX_CLASS_AUDIO20_FU_PHASE_INVERTER_CONTROL         0x0D
#define UX_CLASS_AUDIO20_FU_UNDERFLOW_CONTROL              0x0E
#define UX_CLASS_AUDIO20_FU_OVERFLOW_CONTROL               0x0F
#define UX_CLASS_AUDIO20_FU_LATENCY_CONTROL                0x10

/* Define Audio Class effect unit control selectors.  */

/* Define Audio Class parametric equalizer (PE) section effect unit control selectors.  */
#define UX_CLASS_AUDIO20_PE_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_PE_ENABLE_CONTROL                 0x01
#define UX_CLASS_AUDIO20_PE_CENTERFREQ_CONTROL             0x02
#define UX_CLASS_AUDIO20_PE_QFACTOR_CONTROL                0x03
#define UX_CLASS_AUDIO20_PE_GAIN_CONTROL                   0x04
#define UX_CLASS_AUDIO20_PE_UNDERFLOW_CONTROL              0x05
#define UX_CLASS_AUDIO20_PE_OVERFLOW_CONTROL               0x06
#define UX_CLASS_AUDIO20_PE_LATENCY_CONTROL                0x07

/* Define Audio Class reverberation (RV) section effect unit control selectors.  */
#define UX_CLASS_AUDIO20_RV_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_RV_ENABLE_CONTROL                 0x01
#define UX_CLASS_AUDIO20_RV_TYPE_CONTROL                   0x02
#define UX_CLASS_AUDIO20_RV_LEVEL_CONTROL                  0x03
#define UX_CLASS_AUDIO20_RV_TIME_CONTROL                   0x04
#define UX_CLASS_AUDIO20_RV_FEEDBACK_CONTROL               0x05
#define UX_CLASS_AUDIO20_RV_PREDELAY_CONTROL               0x06
#define UX_CLASS_AUDIO20_RV_DENSITY_CONTROL                0x07
#define UX_CLASS_AUDIO20_RV_HIFREQ_ROLLOFF_CONTROL         0x08
#define UX_CLASS_AUDIO20_RV_UNDERFLOW_CONTROL              0x09
#define UX_CLASS_AUDIO20_RV_OVERFLOW_CONTROL               0x0A
#define UX_CLASS_AUDIO20_RV_LATENCY_CONTROL                0x0B

/* Define Audio Class modulation delay (MD) effect unit control selectors.  */

#define UX_CLASS_AUDIO20_MD_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_MD_ENABLE_CONTROL                 0x01
#define UX_CLASS_AUDIO20_MD_BALANCE_CONTROL                0x02
#define UX_CLASS_AUDIO20_MD_RATE_CONTROL                   0x03
#define UX_CLASS_AUDIO20_MD_DEPTH_CONTROL                  0x04
#define UX_CLASS_AUDIO20_MD_TIME_CONTROL                   0x05
#define UX_CLASS_AUDIO20_MD_FEEDBACK_CONTROL               0x06
#define UX_CLASS_AUDIO20_MD_UNDERFLOW_CONTROL              0x07
#define UX_CLASS_AUDIO20_MD_OVERFLOW_CONTROL               0x08
#define UX_CLASS_AUDIO20_MD_LATENCY_CONTROL                0x09

/* Define Audio Class dynamic range (DR) compressor effect unit control selectors.  */

#define UX_CLASS_AUDIO20_DR_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_DR_ENABLE_CONTROL                 0x01
#define UX_CLASS_AUDIO20_DR_COMPRESSION_RATE_CONTROL       0x02
#define UX_CLASS_AUDIO20_DR_MAXAMPL_CONTROL                0x03
#define UX_CLASS_AUDIO20_DR_THRESHOLD_CONTROL              0x04
#define UX_CLASS_AUDIO20_DR_ATTACK_TIME_CONTROL            0x05
#define UX_CLASS_AUDIO20_DR_RELEASE_TIME_CONTROL           0x06
#define UX_CLASS_AUDIO20_DR_UNDERFLOW_CONTROL              0x07
#define UX_CLASS_AUDIO20_DR_OVERFLOW_CONTROL               0x08
#define UX_CLASS_AUDIO20_DR_LATENCY_CONTROL                0x09

/* Define Audio Class processing unit control selectors.  */

/* Define Audio Class up/down-mix (UD) processing unit control selectors.  */

#define UX_CLASS_AUDIO20_UD_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_UD_ENABLE_CONTROL                 0x01
#define UX_CLASS_AUDIO20_UD_MODE_SELECT_CONTROL            0x02
#define UX_CLASS_AUDIO20_UD_CLUSTER_CONTROL                0x03
#define UX_CLASS_AUDIO20_UD_UNDERFLOW_CONTROL              0x04
#define UX_CLASS_AUDIO20_UD_OVERFLOW_CONTROL               0x05
#define UX_CLASS_AUDIO20_UD_LATENCY_CONTROL                0x06

/* Define Audio Class dolby prologic (DP) processing unit control selectors.  */

#define UX_CLASS_AUDIO20_DP_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_DP_ENABLE_CONTROL                 0x01
#define UX_CLASS_AUDIO20_DP_MODE_SELECT_CONTROL            0x02
#define UX_CLASS_AUDIO20_DP_CLUSTER_CONTROL                0x03
#define UX_CLASS_AUDIO20_DP_UNDERFLOW_CONTROL              0x04
#define UX_CLASS_AUDIO20_DP_OVERFLOW_CONTROL               0x05
#define UX_CLASS_AUDIO20_DP_LATENCY_CONTROL                0x06

/* Define Audio Class stereo extender (ST_EXT) processing unit control selectors.  */

#define UX_CLASS_AUDIO20_ST_EXT_CONTROL_UNDEFINED          0x00
#define UX_CLASS_AUDIO20_ST_EXT_ENABLE_CONTROL             0x01
#define UX_CLASS_AUDIO20_ST_EXT_WIDTH_CONTROL              0x02
#define UX_CLASS_AUDIO20_ST_EXT_UNDERFLOW_CONTROL          0x03
#define UX_CLASS_AUDIO20_ST_EXT_OVERFLOW_CONTROL           0x04
#define UX_CLASS_AUDIO20_ST_EXT_LATENCY_CONTROL            0x05

/* Define Audio Class extension unit (XU) control selectors.  */

#define UX_CLASS_AUDIO20_XU_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_XU_ENABLE_CONTROL                 0x01
#define UX_CLASS_AUDIO20_XU_CLUSTER_CONTROL                0x02
#define UX_CLASS_AUDIO20_XU_UNDERFLOW_CONTROL              0x03
#define UX_CLASS_AUDIO20_XU_OVERFLOW_CONTROL               0x04
#define UX_CLASS_AUDIO20_XU_LATENCY_CONTROL                0x05


/* Define Audio Class AudioStreaming interface control selectors.  */

#define UX_CLASS_AUDIO20_AS_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_AS_ACT_ALT_SETTING_CONTROL        0x01
#define UX_CLASS_AUDIO20_AS_VAL_ALT_SETTINGS_CONTROL       0x02
#define UX_CLASS_AUDIO20_AS_AUDIO_DATA_FORMAT_CONTROL      0x03

/* Define Audio Class encoder (EN) control selectors.  */

#define UX_CLASS_AUDIO20_EN_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_EN_BIT_RATE_CONTROL               0x01
#define UX_CLASS_AUDIO20_EN_QUALITY_CONTROL                0x02
#define UX_CLASS_AUDIO20_EN_VBR_CONTROL                    0x03
#define UX_CLASS_AUDIO20_EN_TYPE_CONTROL                   0x04
#define UX_CLASS_AUDIO20_EN_UNDERFLOW_CONTROL              0x05
#define UX_CLASS_AUDIO20_EN_OVERFLOW_CONTROL               0x06
#define UX_CLASS_AUDIO20_EN_ENCODER_ERROR_CONTROL          0x07
#define UX_CLASS_AUDIO20_EN_PARAM1_CONTROL                 0x08
#define UX_CLASS_AUDIO20_EN_PARAM2_CONTROL                 0x09
#define UX_CLASS_AUDIO20_EN_PARAM3_CONTROL                 0x0A
#define UX_CLASS_AUDIO20_EN_PARAM4_CONTROL                 0x0B
#define UX_CLASS_AUDIO20_EN_PARAM5_CONTROL                 0x0C
#define UX_CLASS_AUDIO20_EN_PARAM6_CONTROL                 0x0D
#define UX_CLASS_AUDIO20_EN_PARAM7_CONTROL                 0x0E
#define UX_CLASS_AUDIO20_EN_PARAM8_CONTROL                 0x0F

/* Define Audio Class decoder control selectors.  */

/* Define Audio Class MPEG decoder (MD) control selectors.  */

#define UX_CLASS_AUDIO20_MD_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_MD_DUAL_CHANNEL_CONTROL           0x01
#define UX_CLASS_AUDIO20_MD_SECOND_STEREO_CONTROL          0x02
#define UX_CLASS_AUDIO20_MD_MULTILINGUAL_CONTROL           0x03
#define UX_CLASS_AUDIO20_MD_DYN_RANGE_CONTROL              0x04
#define UX_CLASS_AUDIO20_MD_SCALING_CONTROL                0x05
#define UX_CLASS_AUDIO20_MD_HILO_SCALING_CONTROL           0x06
#define UX_CLASS_AUDIO20_MD_UNDERFLOW_CONTROL              0x07
#define UX_CLASS_AUDIO20_MD_OVERFLOW_CONTROL               0x08
#define UX_CLASS_AUDIO20_MD_DECODER_ERROR_CONTROL          0x09

/* Define Audio Class AC-3 decoder (AD) control selectors.  */

#define UX_CLASS_AUDIO20_AD_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_AD_MODE_CONTROL                   0x01
#define UX_CLASS_AUDIO20_AD_DYN_RANGE_CONTROL              0x02
#define UX_CLASS_AUDIO20_AD_SCALING_CONTROL                0x03
#define UX_CLASS_AUDIO20_AD_HILO_SCALING_CONTROL           0x04
#define UX_CLASS_AUDIO20_AD_UNDERFLOW_CONTROL              0x05
#define UX_CLASS_AUDIO20_AD_OVERFLOW_CONTROL               0x06
#define UX_CLASS_AUDIO20_AD_DECODER_ERROR_CONTROL          0x07

/* Define Audio Class WMA decoder (WD) control selectors.  */

#define UX_CLASS_AUDIO20_WD_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_WD_UNDERFLOW_CONTROL              0x01
#define UX_CLASS_AUDIO20_WD_OVERFLOW_CONTROL               0x02
#define UX_CLASS_AUDIO20_WD_DECODER_ERROR_CONTROL          0x03

/* Define Audio Class DTS decoder (DD) control selectors.  */

#define UX_CLASS_AUDIO20_DD_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_DD_UNDERFLOW_CONTROL              0x01
#define UX_CLASS_AUDIO20_DD_OVERFLOW_CONTROL               0x02
#define UX_CLASS_AUDIO20_DD_DECODER_ERROR_CONTROL          0x03


/* Define Audio Class endpoint control selectors.  */

#define UX_CLASS_AUDIO20_EP_CONTROL_UNDEFINED              0x00
#define UX_CLASS_AUDIO20_EP_PITCH_CONTROL                  0x01
#define UX_CLASS_AUDIO20_EP_DATA_OVERRUN_CONTROL           0x02
#define UX_CLASS_AUDIO20_EP_DATA_UNDERRUN_CONTROL          0x03


/* Define Audio Class format type codes.  */

#define UX_CLASS_AUDIO20_FORMAT_TYPE_UNDEFINED             0x00
#define UX_CLASS_AUDIO20_FORMAT_TYPE_I                     0x01
#define UX_CLASS_AUDIO20_FORMAT_TYPE_II                    0x02
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III                   0x03
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV                    0x04
#define UX_CLASS_AUDIO20_EXT_FORMAT_TYPE_I                 0x81
#define UX_CLASS_AUDIO20_EXT_FORMAT_TYPE_II                0x82
#define UX_CLASS_AUDIO20_EXT_FORMAT_TYPE_III               0x83


/* Define Audio Class encoding format type I bit allocations.  */

#define UX_CLASS_AUDIO20_FORMAT_TYPE_I_PCM                          (1u << 0)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_I_PCM8                         (1u << 1)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_I_IEEE_FLOAT                   (1u << 2)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_I_ALAW                         (1u << 3)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_I_MULAW                        (1u << 4)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_I_RAW                          (1u << 31)

/* Define Audio Class encoding format type II bit allocations.  */

#define UX_CLASS_AUDIO20_FORMAT_TYPE_II_MPEG                        (1u << 0)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_II_AC3                         (1u << 1)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_II_WMA                         (1u << 2)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_II_DTS                         (1u << 3)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_II_RAW_DATA                    (1u << 31)

/* Define Audio Class encoding format type III bit allocations.  */

#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_AC3               (1u << 0)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_MPEG1_LAYER1      (1u << 1)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_MPEG1_LAYER2_3    (1u << 2)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_MPEG2_NOEXT       (1u << 2)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_MPEG2_EXT         (1u << 3)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_MPEG2_AAC_ADTS    (1u << 4)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_MPEG2_LAYER1_LS   (1u << 5)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_MPEG2_LAYER2_3_LS (1u << 6)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_DTS_I             (1u << 7)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_DTS_II            (1u << 8)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_DTS_III           (1u << 9)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_ATRAC             (1u << 10)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_IEC61937_ATRAC2_3          (1u << 11)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_III_WMA                        (1u << 12)

/* Define Audio Class encoding format type IV bit allocations.  */

#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_PCM                         (1u << 0)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_PCM8                        (1u << 1)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEEE_FLOAT                  (1u << 2)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_ALAW                        (1u << 3)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_MULAW                       (1u << 4)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_MPEG                        (1u << 5)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_AC3                         (1u << 6)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_WMA                         (1u << 7)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_AC3                (1u << 8)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_MPEG1_LAYER1       (1u << 9)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_MPEG1_LAYER2_3     (1u << 10)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_MPEG2_NOEXT        (1u << 10)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_MPEG2_EXT          (1u << 11)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_MPEG2_AAC_ADTS     (1u << 12)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_MPEG2_LAYER1_LS    (1u << 13)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_MPEG2_LAYER2_3_LS  (1u << 14)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_DTS_I              (1u << 15)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_DTS_II             (1u << 16)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_DTS_III            (1u << 17)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_ATRAC              (1u << 18)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC61937_ATRAC2_3           (1u << 19)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_TYPE_III_WMA                (1u << 20)
#define UX_CLASS_AUDIO20_FORMAT_TYPE_IV_IEC60958_PCM                (1u << 21)

/* Define Audio Class side band protocol codes.  */

#define UX_CLASS_AUDIO20_SIDE_BAND_PROTOCOL_UNDEFINED               0x00
#define UX_CLASS_AUDIO20_SIDE_BAND_PRES_TIMESTAMP_PROTOCOL          0x01


/* Define Audio Class bmControls bit pair.  */

#define UX_CLASS_AUDIO20_CONTROL_MASK                               0x3u
#define UX_CLASS_AUDIO20_CONTROL_NOT_PRESENT                        0x0u
#define UX_CLASS_AUDIO20_CONTROL_READ_ONLY                          0x1u
#define UX_CLASS_AUDIO20_CONTROL_PROGRAMMABLE                       0x3u

/* Control position from control selector (CS).  */
#define UX_CLASS_AUDIO20_CONTROL_POS(cs)                            (((cs)-1) << 1)



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

/* Audio Class Audio Channel Cluster Descriptor.  */

typedef struct UX_CLASS_AUDIO20_CHANNEL_CLUSTER_DESCRIPTOR_STRUCT
{
    UCHAR           bNrChannels;
    UCHAR           bmChannelConfig[4];
    UCHAR           iChannelNames;
} UX_CLASS_AUDIO20_CHANNEL_CLUSTER_DESCRIPTOR;

/* bmChannelConfig channel bits (F-Front,L-Left,R-Right,C-Center,B-Back,S-Side,T-Top).  */
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_FL                 (1u << 0)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_FR                 (1u << 1)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_FC                 (1u << 2)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_LFE                (1u << 3) /* Low Frequency Effects  */
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_BL                 (1u << 4)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_BR                 (1u << 5)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_FLC                (1u << 6)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_FRC                (1u << 7)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_BC                 (1u << 8)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_SL                 (1u << 9)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_SR                 (1u << 10)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TC                 (1u << 11)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TFL                (1u << 12)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TFC                (1u << 13)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TFR                (1u << 14)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TBL                (1u << 15)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TBC                (1u << 16)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TBR                (1u << 17)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TFLC               (1u << 18)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TFRC               (1u << 19)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_LLFE               (1u << 20)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_RLFE               (1u << 21)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TSL                (1u << 22)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_TSR                (1u << 23)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_BottomC            (1u << 24) /* Bottom Center  */
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_BLC                (1u << 25)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_BRC                (1u << 26)
#define UX_CLASS_AUDIO20_CHANNEL_CLUSTER_CHANNEL_RD                 (1u << 31) /* Raw Data  */

/* Audio Class AC interface header descriptors.  */

typedef struct UX_CLASS_AUDIO20_AC_HEADER_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubtype;
    UCHAR           bcdADC[2];
    UCHAR           bCategory;
    USHORT          wTotalLength;
    UCHAR           bmControls;
} UX_CLASS_AUDIO20_AC_HEADER_DESCRIPTOR;

/* Define Audio Class clock source descriptor (CSD).  */

typedef struct UX_CLASS_AUDIO20_AC_CLOCK_SOURCE_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bClockID;
    UCHAR           bmAttributes;
    UCHAR           bmControls;
    UCHAR           bAssocTerminal;
    UCHAR           iClockSource;
} UX_CLASS_AUDIO20_AC_CLOCK_SOURCE_DESCRIPTOR, UX_CLASS_AUDIO20_AC_CSD;

/* CSD::bmAttributes.  */
#define UX_CLASS_AUDIO20_CSD_ATTR_CLOCK_TYPE_MASK                       (0x3u)
#define UX_CLASS_AUDIO20_CSD_ATTR_CLOCK_TYPE_EXTERNAL                   (0x0u)
#define UX_CLASS_AUDIO20_CSD_ATTR_CLOCK_TYPE_INTERNAL_FIXED             (0x1u)
#define UX_CLASS_AUDIO20_CSD_ATTR_CLOCK_TYPE_INTERNAL_VARIABLE          (0x2u)
#define UX_CLASS_AUDIO20_CSD_ATTR_CLOCK_TYPE_INTERNAL_PROGRAMMABLE      (0x3u)
#define UX_CLASS_AUDIO20_CSD_ATTR_CLOCK_SYNCH_TO_SOF                    (0x1u << 2)

/* CSD::bmControls.  */
#define UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_FREQ_POS                     (0)
#define UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_FREQ_MASK                    (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_FREQ_POS)
#define UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_FREQ(v)                      ((v) << UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_FREQ_POS)
#define UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_VALIDITY_POS                 (2)
#define UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_VALIDITY_MASK                (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_VALIDITY_POS)
#define UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_VALIDITY(v)                  ((v) << UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_VALIDITY_POS)

/* Define Audio Class clock selector descriptor (CXD, bNrInPins=1).  */

typedef struct UX_CLASS_AUDIO20_AC_CLOCK_SELECTOR_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bClockID;
    UCHAR           bNrInPins;
    UCHAR           baCSourceID[1];
    UCHAR           bmControls;
    UCHAR           iClockSelector;
} UX_CLASS_AUDIO20_AC_CLOCK_SELECTOR_DESCRIPTOR, UX_CLASS_AUDIO20_AC_CXD;

/* CXD::bmControls.  */
#define UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_SELECTOR_POS                 (0)
#define UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_SELECTOR_MASK                (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_SELECTOR_POS)
#define UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_SELECTOR(v)                  ((v) << UX_CLASS_AUDIO20_CSD_CONTROL_CLOCK_SELECTOR_POS)

/* Define Audio Class clock multiplier descriptor (CMD).  */

typedef struct UX_CLASS_AUDIO20_AC_CLOCK_MULTIPLIER_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bClockID;
    UCHAR           bCSourceID;
    UCHAR           bmControls;
    UCHAR           iClockMultiplier;
} UX_CLASS_AUDIO20_AC_CLOCK_MULTIPLIER_DESCRIPTOR, UX_CLASS_AUDIO20_AC_CMD;

/* CMD::bmControls.  */
#define UX_CLASS_AUDIO20_CMD_CONTROL_CLOCK_NUMERATOR_POS                (0)
#define UX_CLASS_AUDIO20_CMD_CONTROL_CLOCK_NUMERATOR_MASK               (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_CMD_CONTROL_CLOCK_NUMERATOR_POS)
#define UX_CLASS_AUDIO20_CMD_CONTROL_CLOCK_NUMERATOR(v)                 ((v) << UX_CLASS_AUDIO20_CMD_CONTROL_CLOCK_NUMERATOR_POS)
#define UX_CLASS_AUDIO20_CMD_CONTROL_CLOCK_DENOMINATOR_POS              (2)
#define UX_CLASS_AUDIO20_CMD_CONTROL_CLOCK_DENOMINATOR_MASK             (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_CMD_CONTROL_CLOCK_DENOMINATOR_POS)
#define UX_CLASS_AUDIO20_CMD_CONTROL_CLOCK_DENOMINATOR(v)               ((v) << UX_CLASS_AUDIO20_CMD_CONTROL_CLOCK_DENOMINATOR_POS)

/* Define Audio Class input terminal descriptor (ITD).  */

typedef struct UX_CLASS_AUDIO20_AC_INPUT_TERMINAL_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bTerminalID;
    USHORT          wTerminalType;
    UCHAR           bAssocTerminal;
    UCHAR           bCSourceID;
    UCHAR           bNrChannels;
    UCHAR           bmChannelConfig[4];
    UCHAR           iChannelNames;
    USHORT          bmControls;
    UCHAR           iTerminal;
} UX_CLASS_AUDIO20_AC_INPUT_TERMINAL_DESCRIPTOR, UX_CLASS_AUDIO20_AC_ITD;

/* ITD::bmControls.  */
#define UX_CLASS_AUDIO20_ITD_CONTROL_COPY_PROTECT_POS                   (0)
#define UX_CLASS_AUDIO20_ITD_CONTROL_COPY_PROTECT_MASK                  (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ITD_CONTROL_COPY_PROTECT_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_COPY_PROTECT(v)                    ((v) << UX_CLASS_AUDIO20_ITD_CONTROL_COPY_PROTECT_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_CONNECTOR_POS                      (2)
#define UX_CLASS_AUDIO20_ITD_CONTROL_CONNECTOR_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ITD_CONTROL_CONNECTOR_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_CONNECTOR(v)                       ((v) << UX_CLASS_AUDIO20_ITD_CONTROL_CONNECTOR_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_OVERLOAD_POS                       (4)
#define UX_CLASS_AUDIO20_ITD_CONTROL_OVERLOAD_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ITD_CONTROL_OVERLOAD_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_OVERLOAD(v)                        ((v) << UX_CLASS_AUDIO20_ITD_CONTROL_OVERLOAD_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_CLUSTER_POS                        (6)
#define UX_CLASS_AUDIO20_ITD_CONTROL_CLUSTER_MASK                       (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ITD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_CLUSTER(v)                         ((v) << UX_CLASS_AUDIO20_ITD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_UNDERFLOW_POS                      (8)
#define UX_CLASS_AUDIO20_ITD_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ITD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_ITD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_OVERFLOW_POS                       (10)
#define UX_CLASS_AUDIO20_ITD_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ITD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_ITD_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_ITD_CONTROL_OVERFLOW_POS)

/* Define Audio Class output terminal descriptor (OTD).  */

typedef struct UX_CLASS_AUDIO20_AC_OUTPUT_TERMINAL_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bTerminalID;
    USHORT          wTerminalType;
    UCHAR           bAssocTerminal;
    UCHAR           bSourceID;
    UCHAR           bCSourceID;
    UCHAR           bmControls[2];
    UCHAR           iTerminal;
} UX_CLASS_AUDIO20_AC_OUTPUT_TERMINAL_DESCRIPTOR, UX_CLASS_AUDIO20_AC_OTD;

/* OTD::bmControls.  */
#define UX_CLASS_AUDIO20_OTD_CONTROL_COPY_PROTECT_POS                   (0)
#define UX_CLASS_AUDIO20_OTD_CONTROL_COPY_PROTECT_MASK                  (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_OTD_CONTROL_COPY_PROTECT_POS)
#define UX_CLASS_AUDIO20_OTD_CONTROL_COPY_PROTECT(v)                    ((v) << UX_CLASS_AUDIO20_OTD_CONTROL_COPY_PROTECT_POS)
#define UX_CLASS_AUDIO20_OTD_CONTROL_CONNECTOR_POS                      (2)
#define UX_CLASS_AUDIO20_OTD_CONTROL_CONNECTOR_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_OTD_CONTROL_CONNECTOR_POS)
#define UX_CLASS_AUDIO20_OTD_CONTROL_CONNECTOR(v)                       ((v) << UX_CLASS_AUDIO20_OTD_CONTROL_CONNECTOR_POS)
#define UX_CLASS_AUDIO20_OTD_CONTROL_OVERLOAD_POS                       (4)
#define UX_CLASS_AUDIO20_OTD_CONTROL_OVERLOAD_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_OTD_CONTROL_OVERLOAD_POS)
#define UX_CLASS_AUDIO20_OTD_CONTROL_OVERLOAD(v)                        ((v) << UX_CLASS_AUDIO20_OTD_CONTROL_OVERLOAD_POS)
#define UX_CLASS_AUDIO20_OTD_CONTROL_UNDERFLOW_POS                      (6)
#define UX_CLASS_AUDIO20_OTD_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_OTD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_OTD_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_OTD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_OTD_CONTROL_OVERFLOW_POS                       (8)
#define UX_CLASS_AUDIO20_OTD_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_OTD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_OTD_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_OTD_CONTROL_OVERFLOW_POS)

/* Define Audio Class mixer unit descriptor (MUD, bNrInPins=1, bmMixerControls N=1).  */

typedef struct UX_CLASS_AUDIO20_AC_MIXER_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    UCHAR           bNrInPins;
    UCHAR           baSourceID[1];
    UCHAR           bNrChannels;
    UCHAR           bmChannelConfig[4];
    UCHAR           iChannelNames;
    UCHAR           bmMixerControls[1];
    UCHAR           bmControls;
    UCHAR           iMixer;
} UX_CLASS_AUDIO20_AC_MIXER_UNIT_DESCRIPTOR, UX_CLASS_AUDIO20_AC_MUD;

/* MUD::bmControls.  */
#define UX_CLASS_AUDIO20_MUD_CONTROL_CLUSTER_POS                        (0)
#define UX_CLASS_AUDIO20_MUD_CONTROL_CLUSTER_MASK                       (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MUD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_MUD_CONTROL_CLUSTER(v)                         ((v) << UX_CLASS_AUDIO20_MUD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_MUD_CONTROL_UNDERFLOW_POS                      (2)
#define UX_CLASS_AUDIO20_MUD_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MUD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_MUD_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_MUD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_MUD_CONTROL_OVERFLOW_POS                       (4)
#define UX_CLASS_AUDIO20_MUD_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MUD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_MUD_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_MUD_CONTROL_OVERFLOW_POS)

/* Define Audio Class selector unit descriptor (SUD, bNrInPins=1).  */

typedef struct UX_CLASS_AUDIO20_AC_SELECTOR_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    UCHAR           bNrInPins;
    UCHAR           baSourceID[1];
    UCHAR           bmControls;
    UCHAR           iSelector;
} UX_CLASS_AUDIO20_AC_SELECTOR_UNIT_DESCRIPTOR, UX_CLASS_AUDIO20_AC_SUD;

/* SUD::bmControls.  */
#define UX_CLASS_AUDIO20_SUD_CONTROL_SELECTOR_POS                       (0)
#define UX_CLASS_AUDIO20_SUD_CONTROL_SELECTOR_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_SUD_CONTROL_SELECTOR_POS)
#define UX_CLASS_AUDIO20_SUD_CONTROL_SELECTOR(v)                        ((v) << UX_CLASS_AUDIO20_SUD_CONTROL_SELECTOR_POS)

/* Define Audio Class feature unit descriptor (FUD, ch=1).  */

typedef struct UX_CLASS_AUDIO20_AC_FEATURE_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    UCHAR           bSourceID;
    UCHAR           bmaControls[4 * 2];
    UCHAR           iFeature;
} UX_CLASS_AUDIO20_AC_FEATURE_UNIT_DESCRIPTOR, UX_CLASS_AUDIO20_AC_FUD;

/* FUD::bmControls.  */
#define UX_CLASS_AUDIO20_FUD_CONTROL_MUTE_POS                           (0)
#define UX_CLASS_AUDIO20_FUD_CONTROL_MUTE_MASK                          (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_MUTE_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_MUTE(v)                            ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_MUTE_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_VOLUME_POS                         (2)
#define UX_CLASS_AUDIO20_FUD_CONTROL_VOLUME_MASK                        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_VOLUME_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_VOLUME(v)                          ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_VOLUME_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_BASS_POS                           (4)
#define UX_CLASS_AUDIO20_FUD_CONTROL_BASS_MASK                          (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_BASS_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_BASS(v)                            ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_BASS_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_MID_POS                            (6)
#define UX_CLASS_AUDIO20_FUD_CONTROL_MID_MASK                           (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_MID_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_MID(v)                             ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_MID_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_TREBLE_POS                         (8)
#define UX_CLASS_AUDIO20_FUD_CONTROL_TREBLE_MASK                        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_TREBLE_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_TREBLE(v)                          ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_TREBLE_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_GRAPHIC_EQ_POS                     (10)
#define UX_CLASS_AUDIO20_FUD_CONTROL_GRAPHIC_EQ_MASK                    (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_GRAPHIC_EQ_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_GRAPHIC_EQ(v)                      ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_GRAPHIC_EQ_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_AUTO_GAIN_POS                      (12)
#define UX_CLASS_AUDIO20_FUD_CONTROL_AUTO_GAIN_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_AUTO_GAIN_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_AUTO_GAIN(v)                       ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_AUTO_GAIN_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_DELAY_POS                          (14)
#define UX_CLASS_AUDIO20_FUD_CONTROL_DELAY_MASK                         (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_DELAY_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_DELAY(v)                           ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_DELAY_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_BASS_BOOST_POS                     (16)
#define UX_CLASS_AUDIO20_FUD_CONTROL_BASS_BOOST_MASK                    (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_BASS_BOOST_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_BASS_BOOST(v)                      ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_BASS_BOOST_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_LOUDNESS_POS                       (18)
#define UX_CLASS_AUDIO20_FUD_CONTROL_LOUDNESS_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_LOUDNESS_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_LOUDNESS(v)                        ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_LOUDNESS_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_INPUT_GAIN_POS                     (20)
#define UX_CLASS_AUDIO20_FUD_CONTROL_INPUT_GAIN_MASK                    (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_INPUT_GAIN_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_INPUT_GAIN(v)                      ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_INPUT_GAIN_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_INPUT_GAIN_PAD_POS                 (22)
#define UX_CLASS_AUDIO20_FUD_CONTROL_INPUT_GAIN_PAD_MASK                (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_INPUT_GAIN_PAD_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_INPUT_GAIN_PAD(v)                  ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_INPUT_GAIN_PAD_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_PHASE_INVERTER_POS                 (24)
#define UX_CLASS_AUDIO20_FUD_CONTROL_PHASE_INVERTER_MASK                (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_PHASE_INVERTER_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_PHASE_INVERTER(v)                  ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_PHASE_INVERTER_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_UNDERFLOW_POS                      (26)
#define UX_CLASS_AUDIO20_FUD_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_OVERFLOW_POS                       (28)
#define UX_CLASS_AUDIO20_FUD_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_FUD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_FUD_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_FUD_CONTROL_OVERFLOW_POS)

typedef struct UX_CLASS_AUDIO20_AC_FEATURE_UNIT_DESCRIPTOR_1_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    UCHAR           bSourceID;
    UCHAR           bmaControl0[4];
    UCHAR           bmaControl1[4];
    UCHAR           iFeature;
} UX_CLASS_AUDIO20_AC_FEATURE_UNIT_DESCRIPTOR_1, UX_CLASS_AUDIO20_AC_FUD_1;

typedef struct UX_CLASS_AUDIO20_AC_FEATURE_UNIT_DESCRIPTOR_2_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    UCHAR           bSourceID;
    UCHAR           bmaControl0[4];
    UCHAR           bmaControl1[4];
    UCHAR           bmaControl2[4];
    UCHAR           iFeature;
} UX_CLASS_AUDIO20_AC_FEATURE_UNIT_DESCRIPTOR_2, UX_CLASS_AUDIO20_AC_FUD_2;

/* Define Audio Class sampling rate converter descriptor (RDU).  */

typedef struct UX_CLASS_AUDIO20_AC_SAMPLING_RATE_CONVERTER_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    UCHAR           bSourceID;
    UCHAR           bSourceInID;
    UCHAR           bSourceOutID;
    UCHAR           iSRC;
} UX_CLASS_AUDIO20_AC_SAMPLING_RATE_CONVERTER_DESCRIPTOR, UX_CLASS_AUDIO20_AC_RUD;

/* Define Audio Class effect unit descriptor (EUD, ch=1).  */

typedef struct UX_CLASS_AUDIO20_AC_EFFECT_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    USHORT          wEffectType;
    UCHAR           bSourceID;
    UCHAR           bmaControls[4 * 2];
    UCHAR           iEffects;
} UX_CLASS_AUDIO20_AC_EFFECT_UNIT_DESCRIPTOR, UX_CLASS_AUDIO20_AC_EUD;

typedef struct UX_CLASS_AUDIO20_AC_EFFECT_UNIT_DESCRIPTOR_1_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    USHORT          wEffectType;
    UCHAR           bSourceID;
    UCHAR           bmaControl0[4];
    UCHAR           bmaControl1[4];
    UCHAR           iEffects;
} UX_CLASS_AUDIO20_AC_EFFECT_UNIT_DESCRIPTOR_1, UX_CLASS_AUDIO20_AC_EUD_1;

typedef struct UX_CLASS_AUDIO20_AC_EFFECT_UNIT_DESCRIPTOR_2_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    USHORT          wEffectType;
    UCHAR           bSourceID;
    UCHAR           bmaControl0[4];
    UCHAR           bmaControl1[4];
    UCHAR           bmaControl2[4];
    UCHAR           iEffects;
} UX_CLASS_AUDIO20_AC_EFFECT_UNIT_DESCRIPTOR_2, UX_CLASS_AUDIO20_AC_EUD_2;

/* Parametric Equalizer Section (PEQS) EUD(PED)::bmaControls.  */
#define UX_CLASS_AUDIO20_PED_CONTROL_ENABLE_POS                         (0)
#define UX_CLASS_AUDIO20_PED_CONTROL_ENABLE_MASK                        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_PED_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_ENABLE(v)                          ((v) << UX_CLASS_AUDIO20_PED_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_CENTER_FREQ_POS                    (2)
#define UX_CLASS_AUDIO20_PED_CONTROL_CENTER_FREQ_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_PED_CONTROL_CENTER_FREQ_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_CENTER_FREQ(v)                     ((v) << UX_CLASS_AUDIO20_PED_CONTROL_CENTER_FREQ_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_Q_FACTOR_POS                       (4)
#define UX_CLASS_AUDIO20_PED_CONTROL_Q_FACTOR_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_PED_CONTROL_Q_FACTOR_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_Q_FACTOR(v)                        ((v) << UX_CLASS_AUDIO20_PED_CONTROL_Q_FACTOR_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_GAIN_POS                           (6)
#define UX_CLASS_AUDIO20_PED_CONTROL_GAIN_MASK                          (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_PED_CONTROL_GAIN_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_GAIN(v)                            ((v) << UX_CLASS_AUDIO20_PED_CONTROL_GAIN_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_UNDERFLOW_POS                      (8)
#define UX_CLASS_AUDIO20_PED_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_PED_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_PED_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_OVERFLOW_POS                       (10)
#define UX_CLASS_AUDIO20_PED_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_PED_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_PED_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_PED_CONTROL_OVERFLOW_POS)

/* Reverberation EUD(RVD)::bmaControls.  */
#define UX_CLASS_AUDIO20_RVD_CONTROL_ENABLE_POS                         (0)
#define UX_CLASS_AUDIO20_RVD_CONTROL_ENABLE_MASK                        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_RVD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_ENABLE(v)                          ((v) << UX_CLASS_AUDIO20_RVD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_TYPE_POS                           (2)
#define UX_CLASS_AUDIO20_RVD_CONTROL_TYPE_MASK                          (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_RVD_CONTROL_TYPE_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_TYPE(v)                            ((v) << UX_CLASS_AUDIO20_RVD_CONTROL_TYPE_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_LEVEL_POS                          (4)
#define UX_CLASS_AUDIO20_RVD_CONTROL_LEVEL_MASK                         (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_RVD_CONTROL_LEVEL_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_LEVEL(v)                           ((v) << UX_CLASS_AUDIO20_RVD_CONTROL_LEVEL_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_TIME_POS                           (6)
#define UX_CLASS_AUDIO20_RVD_CONTROL_TIME_MASK                          (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_RVD_CONTROL_TIME_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_TIME(v)                            ((v) << UX_CLASS_AUDIO20_RVD_CONTROL_TIME_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_DELAY_FEEDBACK_POS                 (8)
#define UX_CLASS_AUDIO20_RVD_CONTROL_DELAY_FEEDBACK_MASK                (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_RVD_CONTROL_DELAY_FEEDBACK_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_DELAY_FEEDBACK(v)                  ((v) << UX_CLASS_AUDIO20_RVD_CONTROL_DELAY_FEEDBACK_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_PRE_DELAY_POS                      (10)
#define UX_CLASS_AUDIO20_RVD_CONTROL_PRE_DELAY_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_RVD_CONTROL_PRE_DELAY_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_PRE_DELAY(v)                       ((v) << UX_CLASS_AUDIO20_RVD_CONTROL_PRE_DELAY_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_DENSITY_POS                        (12)
#define UX_CLASS_AUDIO20_RVD_CONTROL_DENSITY_MASK                       (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_RVD_CONTROL_DENSITY_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_DENSITY(v)                         ((v) << UX_CLASS_AUDIO20_RVD_CONTROL_DENSITY_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_HI_FREQ_ROLL_OFF_POS               (14)
#define UX_CLASS_AUDIO20_RVD_CONTROL_HI_FREQ_ROLL_OFF_MASK              (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_RVD_CONTROL_HI_FREQ_ROLL_OFF_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_HI_FREQ_ROLL_OFF(v)                ((v) << UX_CLASS_AUDIO20_RVD_CONTROL_HI_FREQ_ROLL_OFF_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_UNDERFLOW_POS                      (16)
#define UX_CLASS_AUDIO20_RVD_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_RVD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_RVD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_OVERFLOW_POS                       (18)
#define UX_CLASS_AUDIO20_RVD_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_RVD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_RVD_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_RVD_CONTROL_OVERFLOW_POS)

/* Modulation Delay EUD(MDD)::bmaControls.  */
#define UX_CLASS_AUDIO20_MDD_CONTROL_ENABLE_POS                         (0)
#define UX_CLASS_AUDIO20_MDD_CONTROL_ENABLE_MASK                        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MDD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_ENABLE(v)                          ((v) << UX_CLASS_AUDIO20_MDD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_BALANCE_POS                        (2)
#define UX_CLASS_AUDIO20_MDD_CONTROL_BALANCE_MASK                       (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MDD_CONTROL_BALANCE_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_BALANCE(v)                         ((v) << UX_CLASS_AUDIO20_MDD_CONTROL_BALANCE_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_RATE_POS                           (4)
#define UX_CLASS_AUDIO20_MDD_CONTROL_RATE_MASK                          (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MDD_CONTROL_RATE_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_RATE(v)                            ((v) << UX_CLASS_AUDIO20_MDD_CONTROL_RATE_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_DEPTH_POS                          (6)
#define UX_CLASS_AUDIO20_MDD_CONTROL_DEPTH_MASK                         (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MDD_CONTROL_DEPTH_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_DEPTH(v)                           ((v) << UX_CLASS_AUDIO20_MDD_CONTROL_DEPTH_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_TIME_POS                           (8)
#define UX_CLASS_AUDIO20_MDD_CONTROL_TIME_MASK                          (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MDD_CONTROL_TIME_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_TIME(v)                            ((v) << UX_CLASS_AUDIO20_MDD_CONTROL_TIME_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_FEEDBACK_LEVEL_POS                 (10)
#define UX_CLASS_AUDIO20_MDD_CONTROL_FEEDBACK_LEVEL_MASK                (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MDD_CONTROL_FEEDBACK_LEVEL_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_FEEDBACK_LEVEL(v)                  ((v) << UX_CLASS_AUDIO20_MDD_CONTROL_FEEDBACK_LEVEL_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_UNDERFLOW_POS                      (12)
#define UX_CLASS_AUDIO20_MDD_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MDD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_MDD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_OVERFLOW_POS                       (14)
#define UX_CLASS_AUDIO20_MDD_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MDD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_MDD_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_MDD_CONTROL_OVERFLOW_POS)

/* Dynamic Range Compressor EUD(DRD)::bmaControls.  */
#define UX_CLASS_AUDIO20_DRD_CONTROL_ENABLE_POS                         (0)
#define UX_CLASS_AUDIO20_DRD_CONTROL_ENABLE_MASK                        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DRD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_ENABLE(v)                          ((v) << UX_CLASS_AUDIO20_DRD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_COMPRESS_RATIO_POS                 (2)
#define UX_CLASS_AUDIO20_DRD_CONTROL_COMPRESS_RATIO_MASK                (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DRD_CONTROL_COMPRESS_RATIO_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_COMPRESS_RATIO(v)                  ((v) << UX_CLASS_AUDIO20_DRD_CONTROL_COMPRESS_RATIO_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_MAX_AMPL_POS                       (4)
#define UX_CLASS_AUDIO20_DRD_CONTROL_MAX_AMPL_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DRD_CONTROL_MAX_AMPL_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_MAX_AMPL(v)                        ((v) << UX_CLASS_AUDIO20_DRD_CONTROL_MAX_AMPL_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_THRESHOLD_POS                      (6)
#define UX_CLASS_AUDIO20_DRD_CONTROL_THRESHOLD_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DRD_CONTROL_THRESHOLD_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_THRESHOLD(v)                       ((v) << UX_CLASS_AUDIO20_DRD_CONTROL_THRESHOLD_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_ATTACK_TIME_POS                    (8)
#define UX_CLASS_AUDIO20_DRD_CONTROL_ATTACK_TIME_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DRD_CONTROL_ATTACK_TIME_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_ATTACK_TIME(v)                     ((v) << UX_CLASS_AUDIO20_DRD_CONTROL_ATTACK_TIME_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_RELEASE_TIME_POS                   (10)
#define UX_CLASS_AUDIO20_DRD_CONTROL_RELEASE_TIME_MASK                  (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DRD_CONTROL_RELEASE_TIME_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_RELEASE_TIME(v)                    ((v) << UX_CLASS_AUDIO20_DRD_CONTROL_RELEASE_TIME_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_UNDERFLOW_POS                      (12)
#define UX_CLASS_AUDIO20_DRD_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DRD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_DRD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_OVERFLOW_POS                       (14)
#define UX_CLASS_AUDIO20_DRD_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DRD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_DRD_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_DRD_CONTROL_OVERFLOW_POS)

/* Define Audio Class processing unit descriptor (PUD, bNrInPins=1).  */

typedef struct UX_CLASS_AUDIO20_AC_PROCESSING_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    USHORT          wProcessType;
    UCHAR           bNrInPins;
    UCHAR           baSourceID[1];
    UCHAR           bNrChannels;
    UCHAR           bmChannelConfig[4];
    UCHAR           iChannelNames;
    USHORT          bmControls;
    UCHAR           iProcessing;
} UX_CLASS_AUDIO20_AC_PROCESSING_UNIT_DESCRIPTOR, UX_CLASS_AUDIO20_AC_PUD;

/* PDU::bmaControls.  */
#define UX_CLASS_AUDIO20_PDU_CONTROL_ENABLE_POS                         (0)
#define UX_CLASS_AUDIO20_PDU_CONTROL_ENABLE_MASK                        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_PDU_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_PDU_CONTROL_ENABLE(v)                          ((v) << UX_CLASS_AUDIO20_PDU_CONTROL_ENABLE_POS)

/* Define Audio Class up/down processing unit descriptor (PUD, bNrInPins=1, bNrModes=1).  */

typedef struct UX_CLASS_AUDIO20_AC_UP_DOWN_PROCESSING_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    USHORT          wProcessType;
    UCHAR           bNrInPins;
    UCHAR           baSourceID[1];
    UCHAR           bNrChannels;
    UCHAR           bmChannelConfig[4];
    UCHAR           iChannelNames;
    USHORT          bmControls;
    UCHAR           iProcessing;
    UCHAR           bNrModes;
    UCHAR           daModes[4 * 1];
} UX_CLASS_AUDIO20_AC_UP_DOWN_PROCESSING_UNIT_DESCRIPTOR, UX_CLASS_AUDIO20_AC_UDD;

/* UDD::bmaControls.  */
#define UX_CLASS_AUDIO20_UDD_CONTROL_ENABLE_POS                         (0)
#define UX_CLASS_AUDIO20_UDD_CONTROL_ENABLE_MASK                        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_UDD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_UDD_CONTROL_ENABLE(v)                          ((v) << UX_CLASS_AUDIO20_UDD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_UDD_CONTROL_MODE_SELECT_POS                    (2)
#define UX_CLASS_AUDIO20_UDD_CONTROL_MODE_SELECT_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_UDD_CONTROL_MODE_SELECT_POS)
#define UX_CLASS_AUDIO20_UDD_CONTROL_MODE_SELECT(v)                     ((v) << UX_CLASS_AUDIO20_UDD_CONTROL_MODE_SELECT_POS)
#define UX_CLASS_AUDIO20_UDD_CONTROL_CLUSTER_POS                        (4)
#define UX_CLASS_AUDIO20_UDD_CONTROL_CLUSTER_MASK                       (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_UDD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_UDD_CONTROL_CLUSTER(v)                         ((v) << UX_CLASS_AUDIO20_UDD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_UDD_CONTROL_UNDERFLOW_POS                      (6)
#define UX_CLASS_AUDIO20_UDD_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_UDD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_UDD_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_UDD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_UDD_CONTROL_OVERFLOW_POS                       (8)
#define UX_CLASS_AUDIO20_UDD_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_UDD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_UDD_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_UDD_CONTROL_OVERFLOW_POS)

/* Define Audio Class dolby prologic processing unit descriptor (DPD, bNrInPins=1, bNrModes=1).  */

typedef struct UX_CLASS_AUDIO20_AC_DOLBY_PROCESSING_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    USHORT          wProcessType;
    UCHAR           bNrInPins;
    UCHAR           baSourceID[1];
    UCHAR           bNrChannels;
    UCHAR           bmChannelConfig[4];
    UCHAR           iChannelNames;
    USHORT          bmControls;
    UCHAR           iProcessing;
    UCHAR           bNrModes;
    UCHAR           daModes[4 * 1];
} UX_CLASS_AUDIO20_AC_DOLBY_PROCESSING_UNIT_DESCRIPTOR, UX_CLASS_AUDIO20_AC_DPD;

/* DPD::bmaControls.  */
#define UX_CLASS_AUDIO20_DPD_CONTROL_ENABLE_POS                         (0)
#define UX_CLASS_AUDIO20_DPD_CONTROL_ENABLE_MASK                        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DPD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_DPD_CONTROL_ENABLE(v)                          ((v) << UX_CLASS_AUDIO20_DPD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_DPD_CONTROL_MODE_SELECT_POS                    (2)
#define UX_CLASS_AUDIO20_DPD_CONTROL_MODE_SELECT_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DPD_CONTROL_MODE_SELECT_POS)
#define UX_CLASS_AUDIO20_DPD_CONTROL_MODE_SELECT(v)                     ((v) << UX_CLASS_AUDIO20_DPD_CONTROL_MODE_SELECT_POS)
#define UX_CLASS_AUDIO20_DPD_CONTROL_CLUSTER_POS                        (4)
#define UX_CLASS_AUDIO20_DPD_CONTROL_CLUSTER_MASK                       (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DPD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_DPD_CONTROL_CLUSTER(v)                         ((v) << UX_CLASS_AUDIO20_DPD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_DPD_CONTROL_UNDERFLOW_POS                      (6)
#define UX_CLASS_AUDIO20_DPD_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DPD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_DPD_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_DPD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_DPD_CONTROL_OVERFLOW_POS                       (8)
#define UX_CLASS_AUDIO20_DPD_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DPD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_DPD_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_DPD_CONTROL_OVERFLOW_POS)

/* Define Audio Class Stereo Extender (ST_EXT) Processing Unit Descriptor.  */
typedef UX_CLASS_AUDIO20_AC_PUD UX_CLASS_AUDIO20_AC_STEREO_EXT_PROCESSING_UNIT_DESCRIPTOR;
typedef UX_CLASS_AUDIO20_AC_PUD UX_CLASS_AUDIO20_AC_ST_EXTD;

/* ST_EXTD::bmaControls.  */
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_ENABLE_POS                         (0)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_ENABLE_MASK                        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ST_EXTD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_ENABLE(v)                          ((v) << UX_CLASS_AUDIO20_ST_EXTD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_MODE_SELECT_POS                    (2)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_MODE_SELECT_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ST_EXTD_CONTROL_MODE_SELECT_POS)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_MODE_SELECT(v)                     ((v) << UX_CLASS_AUDIO20_ST_EXTD_CONTROL_MODE_SELECT_POS)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_CLUSTER_POS                        (4)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_CLUSTER_MASK                       (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ST_EXTD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_CLUSTER(v)                         ((v) << UX_CLASS_AUDIO20_ST_EXTD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_UNDERFLOW_POS                      (6)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_UNDERFLOW_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ST_EXTD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_UNDERFLOW(v)                       ((v) << UX_CLASS_AUDIO20_ST_EXTD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_OVERFLOW_POS                       (8)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_OVERFLOW_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ST_EXTD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_ST_EXTD_CONTROL_OVERFLOW(v)                        ((v) << UX_CLASS_AUDIO20_ST_EXTD_CONTROL_OVERFLOW_POS)

/* Define Audio Class extension unit descriptor (XUD, bNrInPins=1).  */

typedef struct UX_CLASS_AUDIO20_AC_EXTENSION_UNIT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bUnitID;
    USHORT          wExtensionCode;
    UCHAR           bNrInPins;
    UCHAR           baSourceID[1];
    UCHAR           bNrChannels;
    UCHAR           bmChannelConfig[4];
    UCHAR           iChannelNames;
    USHORT          bmControls;
    UCHAR           iExtension;
} UX_CLASS_AUDIO20_AC_EXTENSION_UNIT_DESCRIPTOR, UX_CLASS_AUDIO20_AC_XUD;

/* XUD::bmaControls.  */
#define UX_CLASS_AUDIO20_XUD_CONTROL_ENABLE_POS                             (0)
#define UX_CLASS_AUDIO20_XUD_CONTROL_ENABLE_MASK                            (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_XUD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_XUD_CONTROL_ENABLE(v)                              ((v) << UX_CLASS_AUDIO20_XUD_CONTROL_ENABLE_POS)
#define UX_CLASS_AUDIO20_XUD_CONTROL_CLUSTER_POS                            (2)
#define UX_CLASS_AUDIO20_XUD_CONTROL_CLUSTER_MASK                           (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_XUD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_XUD_CONTROL_CLUSTER(v)                             ((v) << UX_CLASS_AUDIO20_XUD_CONTROL_CLUSTER_POS)
#define UX_CLASS_AUDIO20_XUD_CONTROL_UNDERFLOW_POS                          (4)
#define UX_CLASS_AUDIO20_XUD_CONTROL_UNDERFLOW_MASK                         (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_XUD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_XUD_CONTROL_UNDERFLOW(v)                           ((v) << UX_CLASS_AUDIO20_XUD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_XUD_CONTROL_OVERFLOW_POS                           (6)
#define UX_CLASS_AUDIO20_XUD_CONTROL_OVERFLOW_MASK                          (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_XUD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_XUD_CONTROL_OVERFLOW(v)                            ((v) << UX_CLASS_AUDIO20_XUD_CONTROL_OVERFLOW_POS)

/* Audio class-specific AS interface descriptor.  */

typedef struct UX_CLASS_AUDIO20_AS_INTERFACE_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bTerminalLink;
    UCHAR           bmControls;
    UCHAR           bFormatType;
    UCHAR           bmFormats[4];
    UCHAR           bNrChannels;
    UCHAR           bmChannelConfig[4];
    UCHAR           iChannelNames;
} UX_CLASS_AUDIO20_AS_INTERFACE_DESCRIPTOR, UX_CLASS_AUDIO20_AS_IFACED;

/* AS interface (AS_INTERFACE)::bmaControls.  */
#define UX_CLASS_AUDIO20_AS_IFACED_CONTROL_ACTIVE_ALT_POS               (0)
#define UX_CLASS_AUDIO20_AS_IFACED_CONTROL_ACTIVE_ALT_MASK              (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_AS_IFACED_CONTROL_ACTIVE_ALT_POS)
#define UX_CLASS_AUDIO20_AS_IFACED_CONTROL_ACTIVE_ALT(v)                ((v) << UX_CLASS_AUDIO20_AS_IFACED_CONTROL_ACTIVE_ALT_POS)
#define UX_CLASS_AUDIO20_AS_IFACED_CONTROL_VALID_ALT_POS                (2)
#define UX_CLASS_AUDIO20_AS_IFACED_CONTROL_VALID_ALT_MASK               (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_AS_IFACED_CONTROL_VALID_ALT_POS)
#define UX_CLASS_AUDIO20_AS_IFACED_CONTROL_VALID_ALT(v)                 ((v) << UX_CLASS_AUDIO20_AS_IFACED_CONTROL_VALID_ALT_POS)


/* Audio class-specific AS encoder descriptor.  */

typedef struct UX_CLASS_AUDIO20_AS_ENCODER_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bEncoderID;
    UCHAR           bEncoder;
    UCHAR           unknown[3]; /* These 3 bytes are not described in spec (UAC 2.0, May 31, 2006)?  */
    ULONG           bmControls;
    UCHAR           iParam1;
    UCHAR           iParam2;
    UCHAR           iParam3;
    UCHAR           iParam4;
    UCHAR           iParam5;
    UCHAR           iParam6;
    UCHAR           iParam7;
    UCHAR           iParam8;
    UCHAR           iEncoder;
} UX_CLASS_AUDIO20_AS_ENCODER_DESCRIPTOR, UX_CLASS_AUDIO20_AS_ENCD;

/* AS ENCODER::bmaControls.  */
#define UX_CLASS_AUDIO20_ENCD_CONTROL_BIT_RATE_POS                  (0)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_BIT_RATE_MASK                 (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_BIT_RATE_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_BIT_RATE(v)                   ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_BIT_RATE_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_QUALITY_POS                   (2)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_QUALITY_MASK                  (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_QUALITY_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_QUALITY(v)                    ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_QUALITY_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_VBR_POS                       (4)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_VBR_MASK                      (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_VBR_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_VBR(v)                        ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_VBR_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_TYPE_POS                      (6)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_TYPE_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_TYPE_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_TYPE(v)                       ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_TYPE_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_UNDERFLOW_POS                 (8)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_UNDERFLOW_MASK                (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_UNDERFLOW(v)                  ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_OVERFLOW_POS                  (10)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_OVERFLOW_MASK                 (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_OVERFLOW(v)                   ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_ENCODER_ERROR_POS             (12)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_ENCODER_ERROR_MASK            (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_ENCODER_ERROR_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_ENCODER_ERROR(v)              ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_ENCODER_ERROR_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM1_POS                    (14)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM1_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM1_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM1(v)                     ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM1_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM2_POS                    (16)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM2_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM2_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM2(v)                     ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM2_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM3_POS                    (18)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM3_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM3_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM3(v)                     ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM3_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM4_POS                    (20)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM4_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM4_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM4(v)                     ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM4_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM5_POS                    (22)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM5_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM5_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM5(v)                     ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM5_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM6_POS                    (24)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM6_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM6_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM6(v)                     ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM6_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM7_POS                    (26)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM7_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM7_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM7(v)                     ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM7_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM8_POS                    (28)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM8_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM8_POS)
#define UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM8(v)                     ((v) << UX_CLASS_AUDIO20_ENCD_CONTROL_PARAM8_POS)


/* Audio class-specific AS decoder descriptor.  */

/* Audio class-specific AS MPEG decoder descriptor.  */

typedef struct UX_CLASS_AUDIO20_AS_MPEG_DECODER_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bDecoderID;
    UCHAR           bDecoder;
    UCHAR           bmMPEGCapabilities[2];
    UCHAR           bmMPEGFeatures;
    UCHAR           bmControls;
    UCHAR           iDecoder;
} UX_CLASS_AUDIO20_AS_MPEG_DECODER_DESCRIPTOR, UX_CLASS_AUDIO20_AS_MPEG_DECD;

/* MPEG DECODER::bmMPEGCapabilities.  */
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_LAYER_MASK                   (0x7u << 0)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_LAYER_I                      (0x1u << 0)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_LAYER_II                     (0x1u << 1)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_LAYER_III                    (0x1u << 2)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_MPEG1_ONLY                   (0x1u << 3)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_MPEG1_DUAL_CH                (0x1u << 4)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_MPEG2_STEREO                 (0x1u << 5)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_MPEG2_7_1_CH                 (0x1u << 6)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_ADAPTIVE_MULTI_CH_PREDICT    (0x1u << 7)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_MPEG2_MULTILINGUAL_MASK      (0x3u << 8)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_MPEG2_MULTI_NOT_SUPPORT      (0x0u << 8)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_MPEG2_MULTI_FS               (0x1u << 8)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_MPEG2_MULTI_FS_HALF_FS       (0x3u << 8)
#define UX_CLASS_AUDIO20_MPEG_DECD_CAP_SUPPORT_HALF_FS              (0x1u << 10)

/* MPEG DECODER::bmMPEGFeatures.  */
#define UX_CLASS_AUDIO20_MPEG_DECD_FEAT_IDYN_RNG_CTRL_MASK          (0x3u << 4) /* Internal Dynamic Range Control  */
#define UX_CLASS_AUDIO20_MPEG_DECD_FEAT_IDYN_RNG_CTRL_NOT_SUP       (0x0u << 4) /* Not support  */
#define UX_CLASS_AUDIO20_MPEG_DECD_FEAT_IDYN_RNG_CTRL_NOT_SCAL      (0x1u << 4) /* Not scalable  */
#define UX_CLASS_AUDIO20_MPEG_DECD_FEAT_IDYN_RNG_CTRL_SCAL_COMMON   (0x2u << 4) /* Common boost and cut scaling value  */
#define UX_CLASS_AUDIO20_MPEG_DECD_FEAT_IDYN_RNG_CTRL_SCAL_SEPARA   (0x3u << 4) /* Separate boost and cut scaling value  */

/* MPEG DECODER::bmControls.  */
#define UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_UNDERFLOW_POS            (0)
#define UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_UNDERFLOW_MASK           (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_UNDERFLOW(v)             ((v) << UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_OVERFLOW_POS             (2)
#define UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_OVERFLOW_MASK            (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_OVERFLOW(v)              ((v) << UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_DECODER_ERROR_POS        (4)
#define UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_DECODER_ERROR_MASK       (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_DECODER_ERROR_POS)
#define UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_DECODER_ERROR(v)         ((v) << UX_CLASS_AUDIO20_MPEG_DECD_CONTROL_DECODER_ERROR_POS)

/* Audio class-specific AS AC-3 decoder descriptor.  */

typedef struct UX_CLASS_AUDIO20_AS_AC3_DECODER_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bDecoderID;
    UCHAR           bDecoder;
    UCHAR           bmBSID[4];
    UCHAR           bmAC3Features;
    UCHAR           bmControls;
    UCHAR           iDecoder;
} UX_CLASS_AUDIO20_AS_AC3_DECODER_DESCRIPTOR, UX_CLASS_AUDIO20_AS_AC3_DECD;

/* AC3 DECODER::bmAC3Features.  */
#define UX_CLASS_AUDIO20_AC3_DECD_FEAT_RF_MODE                      (0x1u << 0)
#define UX_CLASS_AUDIO20_AC3_DECD_FEAT_LINE_MODE                    (0x1u << 1)
#define UX_CLASS_AUDIO20_AC3_DECD_FEAT_CUSTOM0_MODE                 (0x1u << 2)
#define UX_CLASS_AUDIO20_AC3_DECD_FEAT_CUSTOM1_MODE                 (0x1u << 3)
#define UX_CLASS_AUDIO20_AC3_DECD_FEAT_IDYN_RNG_CTRL_MASK           (0x3u << 4) /* Internal Dynamic Range Control  */
#define UX_CLASS_AUDIO20_AC3_DECD_FEAT_IDYN_RNG_CTRL_NOT_SUP        (0x0u << 4) /* Not support  */
#define UX_CLASS_AUDIO20_AC3_DECD_FEAT_IDYN_RNG_CTRL_NOT_SCAL       (0x3u << 4) /* Not scalable  */
#define UX_CLASS_AUDIO20_AC3_DECD_FEAT_IDYN_RNG_CTRL_SCAL_COMMON    (0x3u << 4) /* Common boost and cut scaling value  */
#define UX_CLASS_AUDIO20_AC3_DECD_FEAT_IDYN_RNG_CTRL_SCAL_SEPARATE  (0x3u << 4) /* Separate boost and cut scaling value  */

/* AC3 DECODER::bmControls.  */
#define UX_CLASS_AUDIO20_AC3_DECD_CONTROL_UNDERFLOW_POS             (0)
#define UX_CLASS_AUDIO20_AC3_DECD_CONTROL_UNDERFLOW_MASK            (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_AC3_DECD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_AC3_DECD_CONTROL_UNDERFLOW(v)              ((v) << UX_CLASS_AUDIO20_AC3_DECD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_AC3_DECD_CONTROL_OVERFLOW_POS              (2)
#define UX_CLASS_AUDIO20_AC3_DECD_CONTROL_OVERFLOW_MASK             (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_AC3_DECD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_AC3_DECD_CONTROL_OVERFLOW(v)               ((v) << UX_CLASS_AUDIO20_AC3_DECD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_AC3_DECD_CONTROL_DECODER_ERROR_POS         (4)
#define UX_CLASS_AUDIO20_AC3_DECD_CONTROL_DECODER_ERROR_MASK        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_AC3_DECD_CONTROL_DECODER_ERROR_POS)
#define UX_CLASS_AUDIO20_AC3_DECD_CONTROL_DECODER_ERROR(v)          ((v) << UX_CLASS_AUDIO20_AC3_DECD_CONTROL_DECODER_ERROR_POS)

/* Audio class-specific AS WMA decoder descriptor.  */

typedef struct UX_CLASS_AUDIO20_AS_WMA_DECODER_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bDecoderID;
    UCHAR           bDecoder;
    UCHAR           bmWMAProfile[2];
    UCHAR           bmControls;
    UCHAR           iDecoder;
} UX_CLASS_AUDIO20_AS_WMA_DECODER_DESCRIPTOR, UX_CLASS_AUDIO20_AS_WMA_DECD;

/* WMA DECODER::bmWMAProfile.  */
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_1_L1                      (1u << 0)
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_2_L2                      (1u << 1)
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_3_L3                      (1u << 2)
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_OTHER_L                   (1u << 3)
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_SPEECH_1_S1               (1u << 4)
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_SPEECH_2_S2               (1u << 5)
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_PRO_1_M1                  (1u << 6)
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_PRO_2_M2                  (1u << 7)
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_PRO_3_M3                  (1u << 8)
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_PRO_OTHER_M               (1u << 9)
#define UX_CLASS_AUDIO20_WMA_DECD_PROFILE_LOSSLESS_SUPPORT          (1u << 10)

/* WMA DECODER::bmControls.  */
#define UX_CLASS_AUDIO20_WMA_DECD_CONTROL_UNDERFLOW_POS             (0)
#define UX_CLASS_AUDIO20_WMA_DECD_CONTROL_UNDERFLOW_MASK            (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_WMA_DECD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_WMA_DECD_CONTROL_UNDERFLOW(v)              ((v) << UX_CLASS_AUDIO20_WMA_DECD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_WMA_DECD_CONTROL_OVERFLOW_POS              (2)
#define UX_CLASS_AUDIO20_WMA_DECD_CONTROL_OVERFLOW_MASK             (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_WMA_DECD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_WMA_DECD_CONTROL_OVERFLOW(v)               ((v) << UX_CLASS_AUDIO20_WMA_DECD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_WMA_DECD_CONTROL_DECODER_ERROR_POS         (4)
#define UX_CLASS_AUDIO20_WMA_DECD_CONTROL_DECODER_ERROR_MASK        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_WMA_DECD_CONTROL_DECODER_ERROR_POS)
#define UX_CLASS_AUDIO20_WMA_DECD_CONTROL_DECODER_ERROR(v)          ((v) << UX_CLASS_AUDIO20_WMA_DECD_CONTROL_DECODER_ERROR_POS)

/* Audio class-specific AS DTS decoder descriptor.  */

typedef struct UX_CLASS_AUDIO20_AS_DTS_DECODER_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bDecoderID;
    UCHAR           bDecoder;
    UCHAR           bmCapabilities;
    UCHAR           bmControls;
    UCHAR           iDecoder;
} UX_CLASS_AUDIO20_AS_DTS_DECODER_DESCRIPTOR, UX_CLASS_AUDIO20_AS_DTS_DECD;

/* DTS DECODER::bmCapabilities.  */
#define UX_CLASS_AUDIO20_DTS_DECD_CAP_CORE                          (0x1u << 0)
#define UX_CLASS_AUDIO20_DTS_DECD_CAP_LOSSLESS                      (0x1u << 1)
#define UX_CLASS_AUDIO20_DTS_DECD_CAP_LBR                           (0x1u << 2)
#define UX_CLASS_AUDIO20_DTS_DECD_CAP_MULTI_STREAM_MIX              (0x1u << 3)
#define UX_CLASS_AUDIO20_DTS_DECD_CAP_DUAL_DECODE                   (0x1u << 4)

/* DTS DECODER::bmControls.  */
#define UX_CLASS_AUDIO20_DTS_DECD_CONTROL_UNDERFLOW_POS             (2)
#define UX_CLASS_AUDIO20_DTS_DECD_CONTROL_UNDERFLOW_MASK            (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DTS_DECD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_DTS_DECD_CONTROL_UNDERFLOW(v)              ((v) << UX_CLASS_AUDIO20_DTS_DECD_CONTROL_UNDERFLOW_POS)
#define UX_CLASS_AUDIO20_DTS_DECD_CONTROL_OVERFLOW_POS              (4)
#define UX_CLASS_AUDIO20_DTS_DECD_CONTROL_OVERFLOW_MASK             (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DTS_DECD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_DTS_DECD_CONTROL_OVERFLOW(v)               ((v) << UX_CLASS_AUDIO20_DTS_DECD_CONTROL_OVERFLOW_POS)
#define UX_CLASS_AUDIO20_DTS_DECD_CONTROL_DECODER_ERROR_POS         (6)
#define UX_CLASS_AUDIO20_DTS_DECD_CONTROL_DECODER_ERROR_MASK        (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_DTS_DECD_CONTROL_DECODER_ERROR_POS)
#define UX_CLASS_AUDIO20_DTS_DECD_CONTROL_DECODER_ERROR(v)          ((v) << UX_CLASS_AUDIO20_DTS_DECD_CONTROL_DECODER_ERROR_POS)


/* Audio class-specific isochronous audio data endpoint descriptor (EPD).  */
typedef struct UX_CLASS_AUDIO20_ENDPOINT_DESCRIPTOR_STRUCT
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bmAttributes;
    UCHAR           bmControls;
    UCHAR           bLockDelayUnits;
    USHORT          wLockDelay;
} UX_CLASS_AUDIO20_ENDPOINT_DESCRIPTOR, UX_CLASS_AUDIO20_EPD;

/* EPD::bmAttributes.  */
#define UX_CLASS_AUDIO20_EPD_ATTR_MAX_PACKET_SIZE_ONLY              (0x1u << 7)

/* EPD::bmControls.  */
#define UX_CLASS_AUDIO20_EPD_CONTROL_PITCH_POS                      (0)
#define UX_CLASS_AUDIO20_EPD_CONTROL_PITCH_MASK                     (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_EPD_CONTROL_PITCH_POS)
#define UX_CLASS_AUDIO20_EPD_CONTROL_PITCH(v)                       ((v) << UX_CLASS_AUDIO20_EPD_CONTROL_PITCH_POS)
#define UX_CLASS_AUDIO20_EPD_CONTROL_OVERRUN_POS                    (2)
#define UX_CLASS_AUDIO20_EPD_CONTROL_OVERRUN_MASK                   (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_EPD_CONTROL_OVERRUN_POS)
#define UX_CLASS_AUDIO20_EPD_CONTROL_OVERRUN(v)                     ((v) << UX_CLASS_AUDIO20_EPD_CONTROL_OVERRUN_POS)
#define UX_CLASS_AUDIO20_EPD_CONTROL_UNDERRUN_POS                   (4)
#define UX_CLASS_AUDIO20_EPD_CONTROL_UNDERRUN_MASK                  (UX_CLASS_AUDIO20_CONTROL_MASK << UX_CLASS_AUDIO20_EPD_CONTROL_UNDERRUN_POS)
#define UX_CLASS_AUDIO20_EPD_CONTROL_UNDERRUN(v)                    ((v) << UX_CLASS_AUDIO20_EPD_CONTROL_UNDERRUN_POS)

/* EPD::bLockDelayUnits.  */
#define UX_CLASS_AUDIO20_EPD_LOCK_DELAY_UNITS_UNDEFINED             0
#define UX_CLASS_AUDIO20_EPD_LOCK_DELAY_UNITS_MS                    1
#define UX_CLASS_AUDIO20_EPD_LOCK_DELAY_UNITS_DEC_PCM_SAMPLES       2


/* Audio class control request layout.  */

typedef struct UX_CLASS_AUDIO20_REQUEST_VALUE_CS_CN_STRUCT {
    UCHAR   channel_num;        /* Channel Number       (CN)   */
    UCHAR   control_sel;        /* Control Selector     (CS)   */
} UX_CLASS_AUDIO20_REQUEST_VALUE_CS_CN;

typedef struct UX_CLASS_AUDIO20_REQUEST_VALUE_MIXER_STRUCT {
    UCHAR   mixer_ctrl_num;     /* Mixer Control Number (MCN)  */
    UCHAR   control_sel;        /* Control Selector     (CS)   */
} UX_CLASS_AUDIO20_REQUEST_VALUE_MIXER, UX_CLASS_AUDIO20_REQUEST_VALUE_CS_MCN;

typedef struct UX_CLASS_AUDIO20_REQUEST_VALUE_CONTROL_STRUCT {
    UCHAR   cn_mcn;             /* CN/MCN   */
    UCHAR   cs;                 /* CS       */
} UX_CLASS_AUDIO20_REQUEST_VALUE_CONTROL;

typedef struct UX_CLASS_AUDIO20_REQUEST_INDEX_EP_STRUCT {
    UCHAR   ep_addr;            /* Endpoint Address  */
    UCHAR   reserved_zero;
} UX_CLASS_AUDIO20_REQUEST_INDEX_EP;

typedef struct UX_CLASS_AUDIO20_REQUEST_INDEX_INTERFACE_STRUCT {
    UCHAR   iface_num;          /* Interface number  */
    UCHAR   entity_id;
} UX_CLASS_AUDIO20_REQUEST_INDEX_INTERFACE;

typedef struct UX_CLASS_AUDIO20_REQUEST_INDEX_CONTROL_STRUCT {
    UCHAR   ep_iface;           /* Endpoint Address/Interface Number  */
    UCHAR   entity_id_zero;     /* Entity ID/Zero  */
} UX_CLASS_AUDIO20_REQUEST_INDEX_CONTROL;

typedef struct UX_CLASS_AUDIO20_REQUEST_STRUCT
{
    UCHAR           bmRequestType;
    UCHAR           bRequest;
    union UX_CLASS_AUDIO20_REQUEST_VALUE_UNION {
        USHORT      value;
        UX_CLASS_AUDIO20_REQUEST_VALUE_CS_CN
                    control_cs_cn;
        UX_CLASS_AUDIO20_REQUEST_VALUE_MIXER
                    control_mixer;
        UX_CLASS_AUDIO20_REQUEST_VALUE_CONTROL
                    control;
        USHORT      mem_offset;
    }               wValue;
    union UX_CLASS_AUDIO20_REQUEST_INDEX_UNION {
        USHORT      value;
        UX_CLASS_AUDIO20_REQUEST_INDEX_EP
                    ep;
        UX_CLASS_AUDIO20_REQUEST_INDEX_INTERFACE
                    iface;
        UX_CLASS_AUDIO20_REQUEST_INDEX_CONTROL
                    control;
    }               wIndex;
    USHORT          wLength;
} UX_CLASS_AUDIO20_REQUEST;

typedef struct UX_CLASS_AUDIO20_RANGE_1B_STRUCT
{
    UCHAR           bMIN;
    UCHAR           bMAX;
    UCHAR           bRES;
} UX_CLASS_AUDIO20_RANGE_1B;

typedef struct UX_CLASS_AUDIO20_RANGE_1B_BLOCK_STRUCT
{
    USHORT          wNumSubRanges;
    UX_CLASS_AUDIO20_RANGE_1B
                    aSubRange[1];
} UX_CLASS_AUDIO20_RANGE_1B_BLOCK;

typedef struct UX_CLASS_AUDIO20_RANGE_2B_STRUCT
{
    USHORT          wMIN;
    USHORT          wMAX;
    USHORT          wRES;
} UX_CLASS_AUDIO20_RANGE_2B;

typedef struct UX_CLASS_AUDIO20_RANGE_2B_BLOCK_STRUCT
{
    USHORT          wNumSubRanges;
    UX_CLASS_AUDIO20_RANGE_2B
                    aSubRange[1];
} UX_CLASS_AUDIO20_RANGE_2B_BLOCK;

typedef struct UX_CLASS_AUDIO20_RANGE_4B_STRUCT
{
    ULONG           dMIN;
    ULONG           dMAX;
    ULONG           dRES;
} UX_CLASS_AUDIO20_RANGE_4B;

typedef struct UX_CLASS_AUDIO20_RANGE_4B_BLOCK_STRUCT
{
    USHORT          wNumSubRanges;
    struct UX_CLASS_AUDIO20_RANGE_4B_UNALIGNED_STRUCT {
        UCHAR       dMIN[4];
        UCHAR       dMAX[4];
        UCHAR       dRES[4];
    }               aSubRange[1];   /* Not 4-byte aligned.  */
} UX_CLASS_AUDIO20_RANGE_4B_BLOCK;


/* Audio Class 2.0 connector control CUR parameter block.  */

typedef struct UX_CLASS_AUDIO20_CONNECTOR_CONTROL_CUR_PARAM_BLOCK_STRUCT
{
    UCHAR           bNrChannels;
    UCHAR           bmChannelConfig[4];
    UCHAR           iChannelNames;
} UX_CLASS_AUDIO20_CONNECTOR_CONTROL_CUR_PARAM_BLOCK;

/* Audio Class 2.0 Graphic Equalizer Control CUR parameter block (NrBits=4).  */

typedef struct UX_CLASS_AUDIO20_GEQ_CONTROL_CUR_PARAM_BLOCK_STRUCT
{
    ULONG           bmBandsPresent;
    UCHAR           bCUR[4];
} UX_CLASS_AUDIO20_GEQ_CONTROL_CUR_PARAM_BLOCK;

/* Audio Class 2.0 Graphic Equalizer Control RANGE parameter block (NrBits=1).  */

typedef struct UX_CLASS_AUDIO20_GEQ_CONTROL_RANGE_PARAM_BLOCK_STRUCT
{
    ULONG           bmBandsPresent;
    UX_CLASS_AUDIO20_RANGE_1B
                    aRNG[1];
} UX_CLASS_AUDIO20_GEQ_CONTROL_RANGE_PARAM_BLOCK;

/* Audio Class 2.0 Valid Alternate Settings Control CUR parameter block (bControlSize=1).  */

typedef struct UX_CLASS_AUDIO20_VALID_ALT_SETT_CONTROL_CUR_PARAM_BLOCK_STRUCT
{
    UCHAR           bControlSize;           /* Number of bytes.  */
    UCHAR           bmValidAltSettings[1];  /* bControlSize bytes.  */
} UX_CLASS_AUDIO20_VALID_ALT_SETT_CONTROL_CUR_PARAM_BLOCK;

/* Audio Class 2.0 High/Low Scaling Control CUR parameter block.  */

typedef struct UX_CLASS_AUDIO20_HI_LO_SCALING_CONTROL_CUR_PARAM_BLOCK_STRUCT
{
    UCHAR           bCUR_Lo;
    UCHAR           bCUR_Hi;
} UX_CLASS_AUDIO20_HI_LO_SCALING_CONTROL_CUR_PARAM_BLOCK;

/* Audio Class 2.0 High/Low Scaling Control RANGE parameter block (wNumSubRanges=1).  */

typedef struct UX_CLASS_AUDIO20_HI_LO_SCALING_CONTROL_RANGE_PARAM_BLOCK_STRUCT
{
    USHORT          wNumSubRanges;
    UX_CLASS_AUDIO20_RANGE_1B
                    aRNG[1];
} UX_CLASS_AUDIO20_HI_LO_SCALING_CONTROL_RANGE_PARAM_BLOCK;


/* Audio Class 2.0 Interrupt Data Message Format.  */

typedef struct UX_CLASS_AUDIO20_INT_MESSAGE_STRUCT
{
    UCHAR           bInfo;
    UCHAR           bAttribute;
    union UX_CLASS_AUDIO20_INT_MESSAGE_VALUE {
        USHORT      value;
        UX_CLASS_AUDIO20_REQUEST_VALUE_CONTROL
                    control;
    }               wValue;
    union UX_CLASS_AUDIO20_INT_MESSAGE_INDEX {
        USHORT      value;
        UX_CLASS_AUDIO20_REQUEST_INDEX_CONTROL
                    control;
    }               wIndex;
} UX_CLASS_AUDIO20_INT_MESSAGE;


/* Audio Class 2.0 Type I Format Type Descriptor.  */

typedef struct UX_CLASS_AUDIO20_TYPE_I_FORMAT_TYPE_DESCRIPTOR
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bFormatType;
    UCHAR           bSubslotSize;
    UCHAR           bBitResolution;
} UX_CLASS_AUDIO20_TYPE_I_FORMAT_TYPE;

/* Audio Class 2.0 Type II Format Type Descriptor.  */

typedef struct UX_CLASS_AUDIO20_TYPE_II_FORMAT_TYPE_DESCRIPTOR
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bFormatType;
    USHORT          wMaxBitRate;
    USHORT          wSlotsPerFrame;
} UX_CLASS_AUDIO20_TYPE_II_FORMAT_TYPE;

/* Audio Class 2.0 Type III Format Type Descriptor.  */

typedef struct UX_CLASS_AUDIO20_TYPE_III_FORMAT_TYPE_DESCRIPTOR
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bFormatType;
    UCHAR           bSubslotSize;
    UCHAR           bBitResolution;
} UX_CLASS_AUDIO20_TYPE_III_FORMAT_TYPE;

/* Audio Class 2.0 Type IV Format Type Descriptor.  */

typedef struct UX_CLASS_AUDIO20_TYPE_IV_FORMAT_TYPE_DESCRIPTOR
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bFormatType;
} UX_CLASS_AUDIO20_TYPE_IV_FORMAT_TYPE;

/* Audio Class 2.0 Extended Type I Format Type Descriptor.  */

typedef struct UX_CLASS_AUDIO20_EXT_TYPE_I_FORMAT_TYPE_DESCRIPTOR
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bFormatType;
    UCHAR           bSubslotSize;
    UCHAR           bBitResolution;
    UCHAR           bHeaderLength;
    UCHAR           bControlSize;
    UCHAR           bSideBandProtocol;
} UX_CLASS_AUDIO20_EXT_TYPE_I_FORMAT_TYPE;

/* Audio Class 2.0 Extended Type II Format Type Descriptor.  */

typedef struct UX_CLASS_AUDIO20_EXT_TYPE_II_FORMAT_TYPE_DESCRIPTOR
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bFormatType;
    USHORT          wMaxBitRate;
    USHORT          wSamplesPerFrame;
    UCHAR           bHeaderLength;
    UCHAR           bSideBandProtocol;
} UX_CLASS_AUDIO20_EXT_TYPE_II_FORMAT_TYPE;

/* Audio Class 2.0 Extended Type III Format Type Descriptor.  */

typedef struct UX_CLASS_AUDIO20_EXT_TYPE_III_FORMAT_TYPE_DESCRIPTOR
{
    UCHAR           bLength;
    UCHAR           bDescriptorType;
    UCHAR           bDescriptorSubType;
    UCHAR           bFormatType;
    UCHAR           bSubslotSize;
    UCHAR           bBitResolution;
    UCHAR           bHeaderLength;
    UCHAR           bSideBandProtocol;
} UX_CLASS_AUDIO20_EXT_TYPE_III_FORMAT_TYPE;

/* Audio Class 2.0 Hi-Res Presentation TimeStamp Layout.  */

typedef struct UX_CLASS_AUDIO20_HI_RES_PRESENTATION_TIMESTAMP_STRUCT
{
    UCHAR           bmFlags;
    UCHAR           qNanoSeconds[8];
} UX_CLASS_AUDIO20_HI_RES_PRESENTATION_TIMESTAMP;

#define UX_CLASS_AUDIO20_HI_RES_PRESENTATION_TIMESTAMP_FLAG_VALID   (1u << 31)

#endif
