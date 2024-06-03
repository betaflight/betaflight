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
/*    ux_device_class_audio.h                             PORTABLE C      */
/*                                                           6.2.0        */
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added feedback support,     */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added interrupt support,    */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Yajun Xia                Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DEVICE_CLASS_AUDIO_H
#define UX_DEVICE_CLASS_AUDIO_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard
   C is used to process the API information.  */

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif


/* Compile option: if defined, audio feedback endpoint is supported.  */
/* #define UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT  */

/* Compile option: if defined, audio interrupt endpoint is supported.  */
/* #define UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT  */


/* Define Audio Class OS related constants.  */
#define UX_DEVICE_CLASS_AUDIO_FEEDBACK_THREAD_STACK_SIZE            UX_THREAD_STACK_SIZE
#define UX_DEVICE_CLASS_AUDIO_INTERRUPT_THREAD_STACK_SIZE           UX_THREAD_STACK_SIZE

/* Define Audio Class function (AF) constants.  */

#define UX_DEVICE_CLASS_AUDIO_FUNCTION_CLASS                        1
#define UX_DEVICE_CLASS_AUDIO_FUNCTION_SUBCLASS_UNDEFINED           0
#define UX_DEVICE_CLASS_AUDIO_FUNCTION_PROTOCOL_UNDEFINED           0
#define UX_DEVICE_CLASS_AUDIO_FUNCTION_PROTOCOL_VERSION_02_00       0x20

/* Define Audio Class interface constants.  */

#define UX_DEVICE_CLASS_AUDIO_CLASS                                 1

#define UX_DEVICE_CLASS_AUDIO_SUBCLASS_UNDEFINED                    0
#define UX_DEVICE_CLASS_AUDIO_SUBCLASS_CONTROL                      1
#define UX_DEVICE_CLASS_AUDIO_SUBCLASS_AUDIOSTREAMING               2
#define UX_DEVICE_CLASS_AUDIO_SUBCLASS_MIDISTREAMING                3

#define UX_DEVICE_CLASS_AUDIO_PROTOCOL_UNDEFINED                    0
#define UX_DEVICE_CLASS_AUTIO_PROTOCOL_VERSION_02_00                0x20


/* Define Audio Class-specific (CS) descriptor types.  */

#define UX_DEVICE_CLASS_AUDIO_CS_UNDEFINED                          0x20
#define UX_DEVICE_CLASS_AUDIO_CS_DEVICE                             0x21
#define UX_DEVICE_CLASS_AUDIO_CS_CONFIGURATION                      0x22
#define UX_DEVICE_CLASS_AUDIO_CS_STRING                             0x23
#define UX_DEVICE_CLASS_AUDIO_CS_INTERFACE                          0x24
#define UX_DEVICE_CLASS_AUDIO_CS_ENDPOINT                           0x25


/* Define Audio Class specific AC interface descriptor subclasses.  */

#define UX_DEVICE_CLASS_AUDIO_AC_UNDEFINED                          0x00
#define UX_DEVICE_CLASS_AUDIO_AC_HEADER                             0x01
#define UX_DEVICE_CLASS_AUDIO_AC_INPUT_TERMINAL                     0x02
#define UX_DEVICE_CLASS_AUDIO_AC_OUTPUT_TERMINAL                    0x03
#define UX_DEVICE_CLASS_AUDIO_AC_FEATURE_UNIT                       0x06


/* Define Audio Class specific AS interface descriptor subclasses.  */

#define UX_DEVICE_CLASS_AUDIO_AS_UNDEFINED                          0x00
#define UX_DEVICE_CLASS_AUDIO_AS_GENERAL                            0x01
#define UX_DEVICE_CLASS_AUDIO_AS_FORMAT_TYPE                        0x02


/* Define Audio Class data endpoint descriptor attributes.  */

#define UX_DEVICE_CLASS_AUDIO_EP_TRANSFER_TYPE_MASK                 (0x3u<<0)
#define UX_DEVICE_CLASS_AUDIO_EP_TRANSFER_TYPE_ISOCHRONOUS          (0x1u<<0)

#define UX_DEVICE_CLASS_AUDIO_EP_SYNCHRONIZATION_TYPE_MASK          (0x3u<<2)
#define UX_DEVICE_CLASS_AUDIO_EP_SYNCHRONIZATION_TYPE_ASYNCHRONOUS  (0x1u<<2)
#define UX_DEVICE_CLASS_AUDIO_EP_SYNCHRONIZATION_TYPE_ADAPTIVE      (0x2u<<2)
#define UX_DEVICE_CLASS_AUDIO_EP_SYNCHRONIZATION_TYPE_SYNCHRONOUS   (0x3u<<2)

#define UX_DEVICE_CLASS_AUDIO_EP_USAGE_TYPE_MASK                    (0x3u<<4)
#define UX_DEVICE_CLASS_AUDIO_EP_USAGE_TYPE_DATA                    (0x0u<<4)
#define UX_DEVICE_CLASS_AUDIO_EP_USAGE_TYPE_FEEDBACK                (0x1u<<4)
#define UX_DEVICE_CLASS_AUDIO_EP_USAGE_TYPE_IMPLICIT_FEEDBACK       (0x2u<<4)


/* Define Audio Class specific endpoint descriptor subtypes.  */

#define UX_DEVICE_CLASS_AUDIO_EP_UNDEFINED                          0x00
#define UX_DEVICE_CLASS_AUDIO_EP_GENERAL                            0x01


/* Define Audio Class specific request codes.  */

#define UX_DEVICE_CLASS_AUDIO_REQUEST_CODE_UNDEFINED                0x00

#define UX_DEVICE_CLASS_AUDIO_bmRequestType_GET_INTERFACE           0xA1
#define UX_DEVICE_CLASS_AUDIO_bmRequestType_SET_INTERFACE           0x21
#define UX_DEVICE_CLASS_AUDIO_bmRequestType_GET_ENDPOINT            0xA2
#define UX_DEVICE_CLASS_AUDIO_bmRequestType_SET_ENDPOINT            0x22

#define UX_DEVICE_CLASS_AUDIO_REQUEST_REQUEST_TYPE                  0
#define UX_DEVICE_CLASS_AUDIO_REQUEST_REQUEST                       1

#define UX_DEVICE_CLASS_AUDIO_REQUEST_VALUE_LOW                     2
#define UX_DEVICE_CLASS_AUDIO_REQUEST_CHANNEL_NUMBER                2
#define UX_DEVICE_CLASS_AUDIO_REQUEST_CN                            2
#define UX_DEVICE_CLASS_AUDIO_REQUEST_MIXER_CONTROL_NUMBER          2
#define UX_DEVICE_CLASS_AUDIO_REQUEST_MCN                           2

#define UX_DEVICE_CLASS_AUDIO_REQUEST_VALUE_HIGH                    3
#define UX_DEVICE_CLASS_AUDIO_REQUEST_CONTROL_SELECTOR              3
#define UX_DEVICE_CLASS_AUDIO_REQUEST_CS                            3

#define UX_DEVICE_CLASS_AUDIO_REQUEST_INDEX_LOW                     4
#define UX_DEVICE_CLASS_AUDIO_REQUEST_ENDPOINT                      4
#define UX_DEVICE_CLASS_AUDIO_REQUEST_INTERFACE                     4

#define UX_DEVICE_CLASS_AUDIO_REQUEST_INDEX_HIGH                    5
#define UX_DEVICE_CLASS_AUDIO_REQUEST_ENEITY_ID                     5

#define UX_DEVICE_CLASS_AUDIO_REQUEST_LENGTH                        6


/* Define Audio Class terminal types.  */

#define UX_DEVICE_CLASS_AUDIO_UNDEFINED                             0x0100
#define UX_DEVICE_CLASS_AUDIO_USB_STREAMING                         0x0101
#define UX_DEVICE_CLASS_AUDIO_USB_VENDOR_SPECIFIC                   0x01FF


/* Define Audio Class input terminal types.  */

#define UX_DEVICE_CLASS_AUDIO_INPUT                                 0x0200
#define UX_DEVICE_CLASS_AUDIO_MICROPHONE                            0x0201
#define UX_DEVICE_CLASS_AUDIO_DESKTOP_MICROPHONE                    0x0202
#define UX_DEVICE_CLASS_AUDIO_PERSONAL_MICROPHONE                   0x0203
#define UX_DEVICE_CLASS_AUDIO_OMNI_DIRECTIONAL_MICROPHONE           0x0204
#define UX_DEVICE_CLASS_AUDIO_MICROPHONE_ARRAY                      0x0205
#define UX_DEVICE_CLASS_AUDIO_PROCESSING_MICROPHONE_ARRAY           0x0206


/* Define Audio Class output terminal types.  */

#define UX_DEVICE_CLASS_AUDIO_OUTPUT                                0x0300
#define UX_DEVICE_CLASS_AUDIO_SPEAKER                               0x0301
#define UX_DEVICE_CLASS_AUDIO_HEADPHONES                            0x0302
#define UX_DEVICE_CLASS_AUDIO_HEAD_MOUNTED_DISPLAY                  0x0303
#define UX_DEVICE_CLASS_AUDIO_DESKTOP_SPEAKER                       0x0304
#define UX_DEVICE_CLASS_AUDIO_ROOM_SPEAKER                          0x0305
#define UX_DEVICE_CLASS_AUDIO_COMMUNICATION_SPEAKER                 0x0306
#define UX_DEVICE_CLASS_AUDIO_LOW_FREQUENCY_SPEAKER                 0x0307


/* Define Audio Class bidirectional terminal types.  */

#define UX_DEVICE_CLASS_AUDIO_BIDIRECTIONAL_UNDEFINED               0x0400
#define UX_DEVICE_CLASS_AUDIO_HANDSET                               0x0401
#define UX_DEVICE_CLASS_AUDIO_HEADSET                               0x0402
#define UX_DEVICE_CLASS_AUDIO_SPEAKERPHONE                          0x0403
#define UX_DEVICE_CLASS_AUDIO_ECHO_SUPRESS_SPEAKERPHONE             0x0404
#define UX_DEVICE_CLASS_AUDIO_ECHO_CANCEL_SPEAKERPHONE              0x0405


/* Define Audio Class telephony terminal types.  */

#define UX_DEVICE_CLASS_AUDIO_TELEPHONTY_UNDEFINED                  0x0500
#define UX_DEVICE_CLASS_AUDIO_PHONE_LINE                            0x0501
#define UX_DEVICE_CLASS_AUDIO_TELEPHONE                             0x0502
#define UX_DEVICE_CLASS_AUDIO_DOWN_LINE_PHONE                       0x0503


/* Define Audio Class external terminal types.  */

#define UX_DEVICE_CLASS_AUDIO_EXTERNAL_UNDEFINED                    0x0600
#define UX_DEVICE_CLASS_AUDIO_ANALOG_CONNECTOR                      0x0601
#define UX_DEVICE_CLASS_AUDIO_DIGITAL_AUDIO_INTERFACE               0x0602
#define UX_DEVICE_CLASS_AUDIO_LINE_CONNECTOR                        0x0603
#define UX_DEVICE_CLASS_AUDIO_LEGACY_AUDIO_CONNECTOR                0x0604
#define UX_DEVICE_CLASS_AUDIO_S_PDIF_INTERFACE                      0x0605
#define UX_DEVICE_CLASS_AUDIO_1394_DA_STREAM                        0x0606
#define UX_DEVICE_CLASS_AUDIO_1394_DV_STREAM_SOUNDTRACK             0x0607
#define UX_DEVICE_CLASS_AUDIO_ADAT_LIGHTPIPE                        0x0608
#define UX_DEVICE_CLASS_AUDIO_TDIF                                  0x0609
#define UX_DEVICE_CLASS_AUDIO_MADI                                  0x060A


/* Define Audio Class embedded function terminal types.  */

#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_UNDEFINED                    0x0700
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_LEVEL_CALIB_NOISE_SRC        0x0701
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_EQUALIZATION_NOISE           0x0702
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_CD_PLAYER                    0x0703
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_DAT                          0x0704
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_DCC                          0x0705
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_MINIDISK                     0x0706
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_ANALOG_TAPE                  0x0707
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_PHONOGRAPH                   0x0708
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_VCR_AUDIO                    0x0709
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_VIDEO_DISC_AUDIO             0x070A
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_DVD_AUDIO                    0x070B
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_TV_TUNER_AUDIO               0x070C
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_SATELLITE_RECEIVER_AUDIO     0x070D
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_CABLE_TUNER_AUDIO            0x070E
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_DSS_AUDIO                    0x070F
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_RADIO_RECEIVER               0x0710
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_RADIO_TRANSMITTER            0x0711
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_MULTI_TRACK_RECORDER         0x0712
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_SYNTHESIZER                  0x0713
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_PIANO                        0x0714
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_GUITAR                       0x0715
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_DRUMS_RHYTHM                 0x0716
#define UX_DEVICE_CLASS_AUDIO_EMBEDDED_OTHER                        0x0717


/* Define Audio Class encoding format types.  */

#define UX_DEVICE_CLASS_AUDIO_FORMAT_TYPE_UNDEFINED                 0
#define UX_DEVICE_CLASS_AUDIO_FORMAT_TYPE_I                         1
#define UX_DEVICE_CLASS_AUDIO_FORMAT_TYPE_II                        2
#define UX_DEVICE_CLASS_AUDIO_FORMAT_TYPE_III                       3
#define UX_DEVICE_CLASS_AUDIO_FORMAT_TYPE_IV                        4
#define UX_DEVICE_CLASS_AUDIO_EXT_FORMAT_TYPE_I                     0x81
#define UX_DEVICE_CLASS_AUDIO_EXT_FORMAT_TYPE_II                    0x82
#define UX_DEVICE_CLASS_AUDIO_EXT_FORMAT_TYPE_III                   0x83


/* Define channels.  */

#define UX_DEVICE_CLASS_AUDIO_MASTER_CHANNEL                        0
#define UX_DEVICE_CLASS_AUDIO_CHANNEL_1                             1
#define UX_DEVICE_CLASS_AUDIO_CHANNEL_2                             2
#define UX_DEVICE_CLASS_AUDIO_CHANNEL_3                             3
#define UX_DEVICE_CLASS_AUDIO_CHANNEL_4                             4
#define UX_DEVICE_CLASS_AUDIO_CHANNEL_5                             5
#define UX_DEVICE_CLASS_AUDIO_CHANNEL_6                             6
#define UX_DEVICE_CLASS_AUDIO_CHANNEL_7                             7
#define UX_DEVICE_CLASS_AUDIO_CHANNEL_8                             8
#define UX_DEVICE_CLASS_AUDIO_CHANNEL_9                             9


/* Define IOCTL code.
     ux_device_class_audio_ioctl(audio, IOCTL_CODE, parameter).
 */

#define UX_DEVICE_CLASS_AUDIO_IOCTL_GET_ARG                         1

/* Define Audio Class Task states.  */
#define UX_DEVICE_CLASS_AUDIO_INTERRUPT_STOP            (UX_STATE_RESET)
#define UX_DEVICE_CLASS_AUDIO_INTERRUPT_START           (UX_STATE_STEP + 1)
#define UX_DEVICE_CLASS_AUDIO_INTERRUPT_WAIT            (UX_STATE_STEP + 2)

#define UX_DEVICE_CLASS_AUDIO_STREAM_RW_STOP            (UX_STATE_RESET)
#define UX_DEVICE_CLASS_AUDIO_STREAM_RW_START           (UX_STATE_STEP + 1)
#define UX_DEVICE_CLASS_AUDIO_STREAM_RW_WAIT            (UX_STATE_STEP + 2)

#define UX_DEVICE_CLASS_AUDIO_STREAM_FEEDBACK_RW_STOP   (UX_STATE_RESET)
#define UX_DEVICE_CLASS_AUDIO_STREAM_FEEDBACK_RW_WAIT   (UX_STATE_STEP + 1)

/* Define Audio Class callback structure.  */

struct UX_DEVICE_CLASS_AUDIO_STREAM_STRUCT;
struct UX_DEVICE_CLASS_AUDIO_STRUCT;

typedef struct UX_DEVICE_CLASS_AUDIO_CALLBACKS_STRUCT
{

    VOID        (*ux_slave_class_audio_instance_activate)(VOID *);
    VOID        (*ux_slave_class_audio_instance_deactivate)(VOID *);
    UINT        (*ux_device_class_audio_control_process)(struct UX_DEVICE_CLASS_AUDIO_STRUCT *, UX_SLAVE_TRANSFER *);
    VOID         *ux_device_class_audio_arg;
} UX_DEVICE_CLASS_AUDIO_CALLBACKS;

typedef struct UX_DEVICE_CLASS_AUDIO_STREAM_CALLBACKS_STRUCT
{
    VOID        (*ux_device_class_audio_stream_change)(struct UX_DEVICE_CLASS_AUDIO_STREAM_STRUCT *, ULONG);
    VOID        (*ux_device_class_audio_stream_frame_done)(struct UX_DEVICE_CLASS_AUDIO_STREAM_STRUCT *, ULONG);
} UX_DEVICE_CLASS_AUDIO_STREAM_CALLBACKS;


/* Define Audio Class Calling Parameter structure */

typedef struct UX_DEVICE_CLASS_AUDIO_STREAM_PARAMETER_STRUCT
{
#if !defined(UX_DEVICE_STANDALONE)
    ULONG                                         ux_device_class_audio_stream_parameter_thread_stack_size;
    VOID                                        (*ux_device_class_audio_stream_parameter_thread_entry)(ULONG id);
#else
    UINT                                        (*ux_device_class_audio_stream_parameter_task_function)(struct UX_DEVICE_CLASS_AUDIO_STREAM_STRUCT*);
#endif

#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
#if !defined(UX_DEVICE_STANDALONE)
    ULONG                                         ux_device_class_audio_stream_parameter_feedback_thread_stack_size;
    VOID                                        (*ux_device_class_audio_stream_parameter_feedback_thread_entry)(ULONG id);
#else
    UINT                                        (*ux_device_class_audio_stream_parameter_feedback_task_function)(struct UX_DEVICE_CLASS_AUDIO_STREAM_STRUCT*);
#endif
#endif
    UX_DEVICE_CLASS_AUDIO_STREAM_CALLBACKS        ux_device_class_audio_stream_parameter_callbacks;

    ULONG                                         ux_device_class_audio_stream_parameter_max_frame_buffer_size;
    ULONG                                         ux_device_class_audio_stream_parameter_max_frame_buffer_nb;
} UX_DEVICE_CLASS_AUDIO_STREAM_PARAMETER;

typedef struct UX_DEVICE_CLASS_AUDIO_PARAMETER_STRUCT
{
    ULONG                                         ux_device_class_audio_parameter_master_interface;
    UX_DEVICE_CLASS_AUDIO_CALLBACKS               ux_device_class_audio_parameter_callbacks;

    ULONG                                         ux_device_class_audio_parameter_streams_nb;
    UX_DEVICE_CLASS_AUDIO_STREAM_PARAMETER       *ux_device_class_audio_parameter_streams;

#if defined(UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT)
    ULONG                                         ux_device_class_audio_parameter_status_size;
    ULONG                                         ux_device_class_audio_parameter_status_queue_size;
#endif
} UX_DEVICE_CLASS_AUDIO_PARAMETER;


/* Define Audio Class instance structure.  */

typedef struct UX_DEVICE_CLASS_AUDIO_FRAME_STRUCT
{

    ULONG                                   ux_device_class_audio_frame_length;
    ULONG                                   ux_device_class_audio_frame_pos;
    UCHAR                                   ux_device_class_audio_frame_data[4];
} UX_DEVICE_CLASS_AUDIO_FRAME;

typedef struct UX_DEVICE_CLASS_AUDIO_STREAM_STRUCT
{

    struct UX_DEVICE_CLASS_AUDIO_STRUCT     *ux_device_class_audio_stream_audio;
    UX_SLAVE_INTERFACE                      *ux_device_class_audio_stream_interface;
    UX_SLAVE_ENDPOINT                       *ux_device_class_audio_stream_endpoint;

#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
    UX_SLAVE_ENDPOINT                       *ux_device_class_audio_stream_feedback;

#if !defined(UX_DEVICE_STANDALONE)
    UCHAR                                   *ux_device_class_audio_stream_feedback_thread_stack;
    UX_THREAD                                ux_device_class_audio_stream_feedback_thread;
#else
    UINT                                   (*ux_device_class_audio_stream_feedback_task_function)(struct UX_DEVICE_CLASS_AUDIO_STREAM_STRUCT*);
    UINT                                     ux_device_class_audio_stream_feedback_task_state;
    UINT                                     ux_device_class_audio_stream_feedback_task_status;
#endif
#endif

    UX_DEVICE_CLASS_AUDIO_STREAM_CALLBACKS   ux_device_class_audio_stream_callbacks;

#if !defined(UX_DEVICE_STANDALONE)
    UCHAR                                   *ux_device_class_audio_stream_thread_stack;
    UX_THREAD                                ux_device_class_audio_stream_thread;
#else
    UINT                                   (*ux_device_class_audio_stream_task_function)(struct UX_DEVICE_CLASS_AUDIO_STREAM_STRUCT*);
    UINT                                     ux_device_class_audio_stream_task_state;
    UINT                                     ux_device_class_audio_stream_task_status;
#endif

    UCHAR                                   *ux_device_class_audio_stream_buffer;
    ULONG                                    ux_device_class_audio_stream_buffer_size;
    ULONG                                    ux_device_class_audio_stream_frame_buffer_size;
    ULONG                                    ux_device_class_audio_stream_buffer_error_count;

    UX_DEVICE_CLASS_AUDIO_FRAME             *ux_device_class_audio_stream_transfer_pos;
    UX_DEVICE_CLASS_AUDIO_FRAME             *ux_device_class_audio_stream_access_pos;
} UX_DEVICE_CLASS_AUDIO_STREAM;

typedef struct UX_DEVICE_CLASS_AUDIO_STRUCT
{

    UX_SLAVE_CLASS                          *ux_device_class_audio_class;
    UX_SLAVE_DEVICE                         *ux_device_class_audio_device;
    UX_SLAVE_INTERFACE                      *ux_device_class_audio_interface;

    UX_DEVICE_CLASS_AUDIO_CALLBACKS          ux_device_class_audio_callbacks;

    ULONG                                    ux_device_class_audio_streams_nb;
    UX_DEVICE_CLASS_AUDIO_STREAM            *ux_device_class_audio_streams;

#if defined(UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT)
    UX_SLAVE_ENDPOINT                       *ux_device_class_audio_interrupt;

    ULONG                                   ux_device_class_audio_status_size;       /* in Bytes.  */
    ULONG                                   ux_device_class_audio_status_queue_bytes;/* in Bytes.  */
    ULONG                                   ux_device_class_audio_status_queued;     /* in Bytes.  */
    UCHAR                                   *ux_device_class_audio_status_queue;     /* in Bytes.  */
    UCHAR                                   *ux_device_class_audio_status_head;
    UCHAR                                   *ux_device_class_audio_status_tail;

#if !defined(UX_DEVICE_STANDALONE)
    UX_SEMAPHORE                            ux_device_class_audio_status_semaphore;
    UX_MUTEX                                ux_device_class_audio_status_mutex;
#else
    UINT                                    ux_device_class_audio_interrupt_task_state;
    UINT                                    ux_device_class_audio_interrupt_task_status;
#endif
#endif
} UX_DEVICE_CLASS_AUDIO;


/* Define Audio Class function prototypes.  */

UINT    _ux_device_class_audio_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_audio_uninitialize(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_audio_activate(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_audio_change(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_audio_deactivate(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_audio_control_request(UX_SLAVE_CLASS_COMMAND *command);

UINT    _ux_device_class_audio_entry(UX_SLAVE_CLASS_COMMAND *command);

UINT    _ux_device_class_audio_ioctl(UX_DEVICE_CLASS_AUDIO *audio, ULONG ioctl_function,
                                     VOID *parameter);

UINT    _ux_device_class_audio_stream_get(UX_DEVICE_CLASS_AUDIO *audio, ULONG stream_index, UX_DEVICE_CLASS_AUDIO_STREAM **stream);

VOID    _ux_device_class_audio_write_thread_entry(ULONG audio_stream);
VOID    _ux_device_class_audio_read_thread_entry(ULONG audio_stream);
UINT    _ux_device_class_audio_write_task_function(UX_DEVICE_CLASS_AUDIO_STREAM *stream);
UINT    _ux_device_class_audio_read_task_function(UX_DEVICE_CLASS_AUDIO_STREAM *stream);
UINT    _ux_device_class_audio_reception_start(UX_DEVICE_CLASS_AUDIO_STREAM *audio);
UINT    _ux_device_class_audio_sample_read8(UX_DEVICE_CLASS_AUDIO_STREAM *audio, UCHAR *sample);
UINT    _ux_device_class_audio_sample_read16(UX_DEVICE_CLASS_AUDIO_STREAM *audio, USHORT *sample);
UINT    _ux_device_class_audio_sample_read24(UX_DEVICE_CLASS_AUDIO_STREAM *audio, ULONG *sample);
UINT    _ux_device_class_audio_sample_read32(UX_DEVICE_CLASS_AUDIO_STREAM *audio, ULONG *sample);

UINT    _ux_device_class_audio_read_frame_get(UX_DEVICE_CLASS_AUDIO_STREAM *audio, UCHAR **frame_data, ULONG *frame_length);
UINT    _ux_device_class_audio_read_frame_free(UX_DEVICE_CLASS_AUDIO_STREAM *audio);

UINT    _ux_device_class_audio_transmission_start(UX_DEVICE_CLASS_AUDIO_STREAM *audio);
UINT    _ux_device_class_audio_frame_write(UX_DEVICE_CLASS_AUDIO_STREAM *audio, UCHAR *frame, ULONG length);

UINT    _ux_device_class_audio_write_frame_get(UX_DEVICE_CLASS_AUDIO_STREAM *audio, UCHAR **buffer, ULONG *max_length);
UINT    _ux_device_class_audio_write_frame_commit(UX_DEVICE_CLASS_AUDIO_STREAM *audio, ULONG length);

VOID    _ux_device_class_audio_feedback_thread_entry(ULONG audio_stream);
UINT    _ux_device_class_audio_feedback_task_function(UX_DEVICE_CLASS_AUDIO_STREAM *stream);
UINT    _ux_device_class_audio_feedback_set(UX_DEVICE_CLASS_AUDIO_STREAM *audio, UCHAR *encoded_feedback);
UINT    _ux_device_class_audio_feedback_get(UX_DEVICE_CLASS_AUDIO_STREAM *audio, UCHAR *encoded_feedback);
ULONG   _ux_device_class_audio_speed_get(UX_DEVICE_CLASS_AUDIO_STREAM *audio);

VOID    _ux_device_class_audio_interrupt_thread_entry(ULONG audio_inst);
UINT    _ux_device_class_audio_interrupt_task_function(UX_DEVICE_CLASS_AUDIO *audio);
UINT    _ux_device_class_audio_interrupt_send(UX_DEVICE_CLASS_AUDIO *audio, UCHAR *int_data);

#if defined(UX_DEVICE_STANDALONE)
UINT    _ux_device_class_audio_tasks_run(VOID *instance);
#endif

/* Define Device Class Audio API prototypes.  */

#define ux_device_class_audio_entry                   _ux_device_class_audio_entry

#define ux_device_class_audio_read_thread_entry       _ux_device_class_audio_read_thread_entry
#define ux_device_class_audio_write_thread_entry      _ux_device_class_audio_write_thread_entry

#define ux_device_class_audio_read_task_function      _ux_device_class_audio_read_task_function
#define ux_device_class_audio_write_task_function     _ux_device_class_audio_write_task_function

#define ux_device_class_audio_stream_get              _ux_device_class_audio_stream_get

#define ux_device_class_audio_reception_start         _ux_device_class_audio_reception_start
#define ux_device_class_audio_sample_read8            _ux_device_class_audio_sample_read8
#define ux_device_class_audio_sample_read16           _ux_device_class_audio_sample_read16
#define ux_device_class_audio_sample_read24           _ux_device_class_audio_sample_read24
#define ux_device_class_audio_sample_read32           _ux_device_class_audio_sample_read32

#define ux_device_class_audio_read_frame_get          _ux_device_class_audio_read_frame_get
#define ux_device_class_audio_read_frame_free         _ux_device_class_audio_read_frame_free

#define ux_device_class_audio_transmission_start      _ux_device_class_audio_transmission_start
#define ux_device_class_audio_frame_write             _ux_device_class_audio_frame_write

#define ux_device_class_audio_write_frame_get         _ux_device_class_audio_write_frame_get
#define ux_device_class_audio_write_frame_commit      _ux_device_class_audio_write_frame_commit

#define ux_device_class_audio_ioctl                   _ux_device_class_audio_ioctl

#define ux_device_class_audio_speed_get               _ux_device_class_audio_speed_get
#define ux_device_class_audio_feedback_thread_entry   _ux_device_class_audio_feedback_thread_entry
#define ux_device_class_audio_feedback_task_function  _ux_device_class_audio_feedback_task_function
#define ux_device_class_audio_feedback_get            _ux_device_class_audio_feedback_get
#define ux_device_class_audio_feedback_set            _ux_device_class_audio_feedback_set

#define ux_device_class_audio_interrupt_send          _ux_device_class_audio_interrupt_send

/* Determine if a C++ compiler is being used.  If so, complete the standard
   C conditional started above.  */
#ifdef __cplusplus
}
#endif

#endif /* ifndef UX_DEVICE_CLASS_AUDIO_H */
