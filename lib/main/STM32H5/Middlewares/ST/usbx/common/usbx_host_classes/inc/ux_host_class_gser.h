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
/**   Generic Serial Host module class                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_gser.h                                PORTABLE C      */ 
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX Generic Serial Class.                                          */ 
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
/*                                                                        */
/**************************************************************************/

#ifndef UX_HOST_CLASS_GSER_H
#define UX_HOST_CLASS_GSER_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  


/* Define Sierra Wireless AR Class constants.  */

#define UX_HOST_CLASS_GSER_CLASS_TRANSFER_TIMEOUT               300000
#ifndef UX_HOST_CLASS_GSER_VENDOR_ID
#define UX_HOST_CLASS_GSER_VENDOR_ID                            0X05C6
#define UX_HOST_CLASS_GSER_PRODUCT_ID                           0X9002 
#endif

/* Define  serial interfaces equivalences.  */
#define UX_HOST_CLASS_GSER_INTERFACE_NUMBER                     3

/* Define  packet equivalences.  */
#define UX_HOST_CLASS_GSER_PACKET_SIZE                          128

/* Define Generic AR IOCTL functions.  */

#define UX_HOST_CLASS_GSER_REQ_SEND_ENCAPSULATED_COMMAND        0x00
#define UX_HOST_CLASS_GSER_REQ_GET_ENCAPSULATED_COMMAND         0x01
#define UX_HOST_CLASS_GSER_REQ_SET_COMM_FEATURE                 0x02
#define UX_HOST_CLASS_GSER_REQ_GET_COMM_FEATURE                 0x03
#define UX_HOST_CLASS_GSER_REQ_CLEAR_COMM_FEATURE               0x04
#define UX_HOST_CLASS_GSER_REQ_SET_AUX_LINE_STATE               0x10
#define UX_HOST_CLASS_GSER_REQ_SET_HOOK_STATE                   0x11
#define UX_HOST_CLASS_GSER_REQ_PULSE_SETUP                      0x12
#define UX_HOST_CLASS_GSER_REQ_SEND_PULSE                       0x13
#define UX_HOST_CLASS_GSER_REQ_SET_PUSLE_TIME                   0x14
#define UX_HOST_CLASS_GSER_REQ_RING_AUX_JACK                    0x15
#define UX_HOST_CLASS_GSER_REQ_SET_LINE_CODING                  0x20
#define UX_HOST_CLASS_GSER_REQ_GET_LINE_CODING                  0x21
#define UX_HOST_CLASS_GSER_REQ_SET_LINE_STATE                   0x22
#define UX_HOST_CLASS_GSER_REQ_SEND_BREAK                       0x23
#define UX_HOST_CLASS_GSER_REQ_SET_RINGER_PARMS                 0x30
#define UX_HOST_CLASS_GSER_REQ_GET_RINGER_PARMS                 0x31 
#define UX_HOST_CLASS_GSER_REQ_SET_OPERATION_PARMS              0x32
#define UX_HOST_CLASS_GSER_REQ_GET_OPERATION_PARMS              0x33
#define UX_HOST_CLASS_GSER_REQ_SET_LINE_PARMS                   0x34
#define UX_HOST_CLASS_GSER_REQ_GET_LINE_PARMS                   0x35

/* Define CDC ACM line output control values.  */

#define UX_HOST_CLASS_GSER_CTRL_DTR                             0x01
#define UX_HOST_CLASS_GSER_CTRL_RTS                             0x02

/* Define CDC ACM line input control values.  */

#define UX_HOST_CLASS_GSER_CTRL_DCD                             0x01
#define UX_HOST_CLASS_GSER_CTRL_DSR                             0x02
#define UX_HOST_CLASS_GSER_CTRL_BRK                             0x04
#define UX_HOST_CLASS_GSER_CTRL_RI                              0x08

#define UX_HOST_CLASS_GSER_CTRL_FRAMING                         0x10
#define UX_HOST_CLASS_GSER_CTRL_PARITY                          0x20
#define UX_HOST_CLASS_GSER_CTRL_OVERRUN                         0x40

/* Define CDC ACM default values.  */

#define UX_HOST_CLASS_GSER_LINE_CODING_DEFAULT_RATE             9600
#define UX_HOST_CLASS_GSER_LINE_CODING_DEFAULT_DATA_BIT         8

/* Define CDC ACM line coding definitions.  */

#define UX_HOST_CLASS_GSER_LINE_CODING_STOP_BIT_0               0
#define UX_HOST_CLASS_GSER_LINE_CODING_STOP_BIT_15              1
#define UX_HOST_CLASS_GSER_LINE_CODING_STOP_BIT_2               2

#define UX_HOST_CLASS_GSER_LINE_CODING_PARITY_NONE              0
#define UX_HOST_CLASS_GSER_LINE_CODING_PARITY_ODD               1
#define UX_HOST_CLASS_GSER_LINE_CODING_PARITY_EVEN              2
#define UX_HOST_CLASS_GSER_LINE_CODING_PARITY_MARK              3
#define UX_HOST_CLASS_GSER_LINE_CODING_PARITY_SPACE             4

#define UX_HOST_CLASS_GSER_LINE_CODING_LENGTH                   7
#define UX_HOST_CLASS_GSER_LINE_CODING_RATE                     0
#define UX_HOST_CLASS_GSER_LINE_CODING_STOP_BIT                 4
#define UX_HOST_CLASS_GSER_LINE_CODING_PARITY                   5
#define UX_HOST_CLASS_GSER_LINE_CODING_DATA_BIT                 6

/* Define CDC ACM line state definitions.  */

#define UX_HOST_CLASS_GSER_LINE_STATE_STOP_BIT_0                0
#define UX_HOST_CLASS_GSER_LINE_CODING_STOP_BIT_15              1

/* Define CDC ACM IOCTL Functions. */

#define UX_HOST_CLASS_GSER_IOCTL_SET_LINE_CODING                0
#define UX_HOST_CLASS_GSER_IOCTL_GET_LINE_CODING                1
#define UX_HOST_CLASS_GSER_IOCTL_SET_LINE_STATE                 2
#define UX_HOST_CLASS_GSER_IOCTL_SEND_BREAK                     3
#define UX_HOST_CLASS_GSER_IOCTL_ABORT_IN_PIPE                  5
#define UX_HOST_CLASS_GSER_IOCTL_ABORT_OUT_PIPE                 6
#define UX_HOST_CLASS_GSER_IOCTL_NOTIFICATION_CALLBACK          7
#define UX_HOST_CLASS_GSER_IOCTL_GET_DEVICE_STATUS              8

/* Define Reception States. */

#define UX_HOST_CLASS_GSER_RECEPTION_STATE_STOPPED              0
#define UX_HOST_CLASS_GSER_RECEPTION_STATE_STARTED              1
#define UX_HOST_CLASS_GSER_RECEPTION_STATE_IN_TRANSFER          2

/* Define the interface structures.  */

typedef struct UX_HOST_CLASS_GSER_INTERFACE_STRUCT
{

    UX_INTERFACE                                *ux_host_class_gser_interface;
    UX_ENDPOINT                                 *ux_host_class_gser_bulk_out_endpoint;
    UX_ENDPOINT                                 *ux_host_class_gser_bulk_in_endpoint;
    UX_SEMAPHORE                                ux_host_class_gser_semaphore;
    struct UX_HOST_CLASS_GSER_RECEPTION_STRUCT  *ux_host_class_gser_reception;
    ULONG                                       ux_host_class_gser_notification_count;
} UX_HOST_CLASS_GSER_INTERFACE;

/* Define Generic Serial Class instance structure.  */

typedef struct UX_HOST_CLASS_GSER_STRUCT
{

    struct UX_HOST_CLASS_GSER_STRUCT            *ux_host_class_gser_next_instance;
    UX_HOST_CLASS                               *ux_host_class_gser_class;
    UX_DEVICE                                   *ux_host_class_gser_device;
    UINT                                        ux_host_class_gser_state;
    struct UX_HOST_CLASS_GSER_INTERFACE_STRUCT  ux_host_class_gser_interface_array[UX_HOST_CLASS_GSER_INTERFACE_NUMBER];
    ULONG                                       ux_host_class_gser_device_state;
    VOID                                        (*ux_host_class_gser_device_status_change_callback)(struct UX_HOST_CLASS_GSER_STRUCT *gser, 
                                                                    ULONG  notification_type, ULONG notification_value);
} UX_HOST_CLASS_GSER;

/* Define generic serial class reception structure. */


typedef struct UX_HOST_CLASS_GSER_RECEPTION_STRUCT
{

    ULONG           ux_host_class_gser_reception_interface_index;
    ULONG           ux_host_class_gser_reception_state;
    ULONG           ux_host_class_gser_reception_block_size;
    UCHAR           *ux_host_class_gser_reception_data_buffer;
    ULONG           ux_host_class_gser_reception_data_buffer_size;
    UCHAR           *ux_host_class_gser_reception_data_head;
    UCHAR           *ux_host_class_gser_reception_data_tail;
    VOID            (*ux_host_class_gser_reception_callback)(struct UX_HOST_CLASS_GSER_STRUCT *gser, 
                                                                UINT  status,
                                                                UCHAR *reception_buffer, 
                                                                ULONG reception_size);

} UX_HOST_CLASS_GSER_RECEPTION;


/* Define GSER Line Coding IOCTL structure.  */

typedef struct UX_HOST_CLASS_GSER_LINE_CODING_STRUCT
{

    ULONG           ux_host_class_gser_line_coding_dter;
    ULONG           ux_host_class_gser_line_coding_stop_bit;
    ULONG           ux_host_class_gser_line_coding_parity;
    ULONG           ux_host_class_gser_line_coding_data_bits;

} UX_HOST_CLASS_GSER_LINE_CODING;

/* Define GSER Line State IOCTL structure.  */

typedef struct UX_HOST_CLASS_GSER_LINE_STATE_STRUCT
{

    ULONG           ux_host_class_gser_line_state_rts;
    ULONG           ux_host_class_gser_line_state_dtr;

} UX_HOST_CLASS_GSER_LINE_STATE;

/* Define GSER Line break IOCTL structure.  */

typedef struct UX_HOST_CLASS_GSER_LINE_BREAK_STRUCT
{

    ULONG           ux_host_class_gser_line_break;

} UX_HOST_CLASS_GSER_LINE_BREAK;


/* Define  GSER Class function prototypes.  */

UINT    _ux_host_class_gser_activate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_gser_configure(UX_HOST_CLASS_GSER *gser);
UINT    _ux_host_class_gser_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_gser_endpoints_get(UX_HOST_CLASS_GSER *gser);
UINT    _ux_host_class_gser_entry(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_gser_read (UX_HOST_CLASS_GSER *gser, ULONG interface_index,UCHAR *data_pointer, 
                                    ULONG requested_length, ULONG *actual_length);
UINT    _ux_host_class_gser_write(UX_HOST_CLASS_GSER *gser, ULONG interface_index,UCHAR *data_pointer, 
                                    ULONG requested_length, ULONG *actual_length);
UINT    _ux_host_class_gser_command(UX_HOST_CLASS_GSER *gser, ULONG interface_index, ULONG command,
                                    ULONG value, UCHAR *data_buffer, ULONG data_length);
UINT    _ux_host_class_gser_ioctl(UX_HOST_CLASS_GSER *gser, ULONG interface_index, ULONG ioctl_function,
                                    VOID *parameter);
VOID    _ux_host_class_gser_reception_callback (UX_TRANSFER *transfer_request);
UINT    _ux_host_class_gser_reception_stop (UX_HOST_CLASS_GSER *gser, 
                                    UX_HOST_CLASS_GSER_RECEPTION *gser_reception);
UINT    _ux_host_class_gser_reception_start (UX_HOST_CLASS_GSER *gser, 
                                    UX_HOST_CLASS_GSER_RECEPTION *gser_reception);
                                    
/* Define GSER Class API prototypes.  */

#define ux_host_class_gser_entry                     _ux_host_class_gser_entry
#define ux_host_class_gser_read                      _ux_host_class_gser_read
#define ux_host_class_gser_write                     _ux_host_class_gser_write
#define ux_host_class_gser_ioctl                     _ux_host_class_gser_ioctl
#define ux_host_class_gser_reception_start           _ux_host_class_gser_reception_start
#define ux_host_class_gser_reception_stop            _ux_host_class_gser_reception_stop

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif
