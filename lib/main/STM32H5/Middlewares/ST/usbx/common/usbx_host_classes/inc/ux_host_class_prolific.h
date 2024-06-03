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
/**   PROLIFIC Class                                                      */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_prolific.h                            PORTABLE C      */ 
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX PROLIFIC class.                                                */ 
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

#ifndef UX_HOST_CLASS_PROLIFIC_H
#define UX_HOST_CLASS_PROLIFIC_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  

/* Define PROLIFIC Class constants.  */
                                                                
#define UX_HOST_CLASS_PROLIFIC_DEVICE_INIT_DELAY                (1 * UX_PERIODIC_RATE)
#define UX_HOST_CLASS_PROLIFIC_CLASS_TRANSFER_TIMEOUT           300000
#define UX_HOST_CLASS_PROLIFIC_SETUP_BUFFER_SIZE                16
#define UX_HOST_CLASS_PROLIFIC_DEVICE_PRESENT                   1 
#define UX_HOST_CLASS_PROLIFIC_DEVICE_NOT_PRESENT               0
#define UX_HOST_CLASS_PROLIFIC_DEVICE_STATE_OFFSET              8
#define UX_HOST_CLASS_PROLIFIC_DEVICE_STATE_MASK                0x7F
#define UX_HOST_CLASS_PROLIFIC_DEVICE_TYPE_0                    0
#define UX_HOST_CLASS_PROLIFIC_DEVICE_TYPE_1                    1
#define UX_HOST_CLASS_PROLIFIC_DEVICE_TYPE_HX                   2
#define UX_HOST_CLASS_PROLIFIC_VENDOR_READ_REQUEST              1
#define UX_HOST_CLASS_PROLIFIC_VENDOR_WRITE_REQUEST             1

/* Define PROLIFIC Class descriptor subtypes in functional descriptors.  */
#define UX_HOST_CLASS_PROLIFIC_HEADER_DESCRIPTOR                0X00
#define UX_HOST_CLASS_PROLIFIC_CALL_MANAGEMENT_DESCRIPTOR       0X01
#define UX_HOST_CLASS_PROLIFIC_ABSTRACT_CONTROL_MGT_DESCRIPTOR  0X02
#define UX_HOST_CLASS_PROLIFIC_DIRECT_LINE_MGT_DESCRIPTOR       0X03
#define UX_HOST_CLASS_PROLIFIC_TELEPHONE_RINGER_DESCRIPTOR      0X04
#define UX_HOST_CLASS_PROLIFIC_REPORT_CAPABILITY_DESCRIPTOR     0X05
#define UX_HOST_CLASS_PROLIFIC_UNION_DESCRIPTOR                 0X06
#define UX_HOST_CLASS_PROLIFIC_COUNTRY_SELECTION_DESCRIPTOR     0X07
#define UX_HOST_CLASS_PROLIFIC_TELEPHONE_OPERATIONAL_DESCRIPTOR 0X08
#define UX_HOST_CLASS_PROLIFIC_USB_TERMINAL_DESCRIPTOR          0X09

/* Define PROLIFIC Class call management descriptors.  */
#define UX_HOST_CLASS_PROLIFIC_CALL_MANAGEMENT_CAPABILITIES     0x03
#define UX_HOST_CLASS_PROLIFIC_CALL_MANAGEMENT_DCM              0x01
#define UX_HOST_CLASS_PROLIFIC_CALL_MANAGEMENT_DCI              0x02

/* Define PROLIFIC command request values.  */

#define UX_HOST_CLASS_PROLIFIC_REQ_SEND_ENCAPSULATED_COMMAND    0x00
#define UX_HOST_CLASS_PROLIFIC_REQ_GET_ENCAPSULATED_COMMAND     0x01
#define UX_HOST_CLASS_PROLIFIC_REQ_SET_COMM_FEATURE             0x02
#define UX_HOST_CLASS_PROLIFIC_REQ_GET_COMM_FEATURE             0x03
#define UX_HOST_CLASS_PROLIFIC_REQ_CLEAR_COMM_FEATURE           0x04
#define UX_HOST_CLASS_PROLIFIC_REQ_SET_AUX_LINE_STATE           0x10
#define UX_HOST_CLASS_PROLIFIC_REQ_SET_HOOK_STATE               0x11
#define UX_HOST_CLASS_PROLIFIC_REQ_PULSE_SETUP                  0x12
#define UX_HOST_CLASS_PROLIFIC_REQ_SEND_PULSE                   0x13
#define UX_HOST_CLASS_PROLIFIC_REQ_SET_PUSLE_TIME               0x14
#define UX_HOST_CLASS_PROLIFIC_REQ_RING_AUX_JACK                0x15
#define UX_HOST_CLASS_PROLIFIC_REQ_SET_LINE_CODING              0x20
#define UX_HOST_CLASS_PROLIFIC_REQ_GET_LINE_CODING              0x21
#define UX_HOST_CLASS_PROLIFIC_REQ_SET_LINE_STATE               0x22
#define UX_HOST_CLASS_PROLIFIC_REQ_SEND_BREAK                   0x23
#define UX_HOST_CLASS_PROLIFIC_REQ_SET_RINGER_PARMS             0x30
#define UX_HOST_CLASS_PROLIFIC_REQ_GET_RINGER_PARMS             0x31 
#define UX_HOST_CLASS_PROLIFIC_REQ_SET_OPERATION_PARMS          0x32
#define UX_HOST_CLASS_PROLIFIC_REQ_GET_OPERATION_PARMS          0x33
#define UX_HOST_CLASS_PROLIFIC_REQ_SET_LINE_PARMS               0x34
#define UX_HOST_CLASS_PROLIFIC_REQ_GET_LINE_PARMS               0x35

/* Define  PROLIFIC line output control values.  */

#define UX_HOST_CLASS_PROLIFIC_CTRL_DTR                         0x01
#define UX_HOST_CLASS_PROLIFIC_CTRL_RTS                         0x02

/* Define  PROLIFIC line input control values.  */

#define UX_HOST_CLASS_PROLIFIC_CTRL_DCD                         0x01
#define UX_HOST_CLASS_PROLIFIC_CTRL_DSR                         0x02
#define UX_HOST_CLASS_PROLIFIC_CTRL_BRK                         0x04
#define UX_HOST_CLASS_PROLIFIC_CTRL_RI                          0x08

#define UX_HOST_CLASS_PROLIFIC_CTRL_FRAMING                     0x10
#define UX_HOST_CLASS_PROLIFIC_CTRL_PARITY                      0x20
#define UX_HOST_CLASS_PROLIFIC_CTRL_OVERRUN                     0x40

#define    UX_HOST_CLASS_PROLIFIC_COMMAND_EEPROM_READ           0x8484
#define    UX_HOST_CLASS_PROLIFIC_COMMAND_EEPROM_WRITE          0x0404
#define    UX_HOST_CLASS_PROLIFIC_COMMAND_EEPROM_ADDRESS        0x8383
#define    UX_HOST_CLASS_PROLIFIC_COMMAND_REG_CONFIGURE         0x0002
#define    UX_HOST_CLASS_PROLIFIC_COMMAND_PIPE1_RESET           0x0008
#define    UX_HOST_CLASS_PROLIFIC_COMMAND_PIPE2_RESET           0x0009

/* Define  PROLIFIC Class packet equivalences.  */

#define UX_HOST_CLASS_PROLIFIC_PACKET_SIZE                      128

/* Define  PROLIFIC default values.  */

#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_DEFAULT_RATE         19200
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_DEFAULT_DATA_BIT     8

/* Define  PROLIFIC line coding definitions.  */

#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_STOP_BIT_0           0
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_STOP_BIT_15          1
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_STOP_BIT_2           2

#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_PARITY_NONE          0
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_PARITY_ODD           1
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_PARITY_EVEN          2
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_PARITY_MARK          3
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_PARITY_SPACE         4

#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_LENGTH               7
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_RATE                 0
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_STOP_BIT             4
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_PARITY               5
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_DATA_BIT             6

/* Define  PROLIFIC line state definitions.  */

#define UX_HOST_CLASS_PROLIFIC_LINE_STATE_STOP_BIT_0            0
#define UX_HOST_CLASS_PROLIFIC_LINE_CODING_STOP_BIT_15          1

/* Define  PROLIFIC IOCTL Functions. */

#define UX_HOST_CLASS_PROLIFIC_IOCTL_SET_LINE_CODING            0
#define UX_HOST_CLASS_PROLIFIC_IOCTL_GET_LINE_CODING            1
#define UX_HOST_CLASS_PROLIFIC_IOCTL_SET_LINE_STATE             2
#define UX_HOST_CLASS_PROLIFIC_IOCTL_SEND_BREAK                 3
#define UX_HOST_CLASS_PROLIFIC_IOCTL_PURGE                      4
#define UX_HOST_CLASS_PROLIFIC_IOCTL_ABORT_IN_PIPE              5
#define UX_HOST_CLASS_PROLIFIC_IOCTL_ABORT_OUT_PIPE             6
#define UX_HOST_CLASS_PROLIFIC_IOCTL_REPORT_DEVICE_STATUS_CHANGE 7
#define UX_HOST_CLASS_PROLIFIC_IOCTL_GET_DEVICE_STATUS          8

/* Define  PROLIFIC Reception States. */

#define UX_HOST_CLASS_PROLIFIC_RECEPTION_STATE_STOPPED          0
#define UX_HOST_CLASS_PROLIFIC_RECEPTION_STATE_STARTED          1
#define UX_HOST_CLASS_PROLIFIC_RECEPTION_STATE_IN_TRANSFER      2


/* Define  PROLIFIC Class instance structure.  */

typedef struct UX_HOST_CLASS_PROLIFIC_STRUCT
{
    struct UX_HOST_CLASS_PROLIFIC_STRUCT  
                    *ux_host_class_prolific_next_instance;
    UX_HOST_CLASS   *ux_host_class_prolific_class;
    UX_DEVICE       *ux_host_class_prolific_device;
    UX_ENDPOINT     *ux_host_class_prolific_bulk_in_endpoint;
    UX_ENDPOINT     *ux_host_class_prolific_bulk_out_endpoint;
    UX_ENDPOINT     *ux_host_class_prolific_interrupt_endpoint;
    UX_INTERFACE    *ux_host_class_prolific_interface;
    UINT            ux_host_class_prolific_instance_status;
    UINT            ux_host_class_prolific_state;
    UX_SEMAPHORE    ux_host_class_prolific_semaphore;
    ULONG           ux_host_class_prolific_notification_count;
    ULONG           ux_host_class_prolific_device_state;
    VOID            (*ux_host_class_prolific_device_status_change_callback)(struct UX_HOST_CLASS_PROLIFIC_STRUCT *prolific, 
                                                                ULONG  device_state);
    
    ULONG           ux_host_class_prolific_version;
    UCHAR           ux_host_class_prolific_device_type;
    struct UX_HOST_CLASS_PROLIFIC_RECEPTION_STRUCT  
                    *ux_host_class_prolific_reception;
    
} UX_HOST_CLASS_PROLIFIC;


/* Define  PROLIFIC reception structure. */

typedef struct UX_HOST_CLASS_PROLIFIC_RECEPTION_STRUCT
{

    ULONG            ux_host_class_prolific_reception_state;
    ULONG            ux_host_class_prolific_reception_block_size;
    UCHAR            *ux_host_class_prolific_reception_data_buffer;
    ULONG            ux_host_class_prolific_reception_data_buffer_size;
    UCHAR            *ux_host_class_prolific_reception_data_head;
    UCHAR            *ux_host_class_prolific_reception_data_tail;
    VOID            (*ux_host_class_prolific_reception_callback)(struct UX_HOST_CLASS_PROLIFIC_STRUCT *prolific, 
                                                                UINT  status,
                                                                UCHAR *reception_buffer, 
                                                                ULONG reception_size);

} UX_HOST_CLASS_PROLIFIC_RECEPTION;

/* Define  PROLIFIC Line Coding IOCTL structure.  */

typedef struct UX_HOST_CLASS_PROLIFIC_LINE_CODING_STRUCT
{

    ULONG            ux_host_class_prolific_line_coding_dter;
    ULONG            ux_host_class_prolific_line_coding_stop_bit;
    ULONG            ux_host_class_prolific_line_coding_parity;
    ULONG            ux_host_class_prolific_line_coding_data_bits;

} UX_HOST_CLASS_PROLIFIC_LINE_CODING;

/* Define  PROLIFIC Line State IOCTL structure.  */

typedef struct UX_HOST_CLASS_PROLIFIC_LINE_STATE_STRUCT
{

    ULONG            ux_host_class_prolific_line_state_rts;
    ULONG            ux_host_class_prolific_line_state_dtr;

} UX_HOST_CLASS_PROLIFIC_LINE_STATE;

/* Define  PROLIFIC Line break IOCTL structure.  */

typedef struct UX_HOST_CLASS_PROLIFIC_LINE_BREAK_STRUCT
{

    ULONG            ux_host_class_prolific_line_break;

} UX_HOST_CLASS_PROLIFIC_LINE_BREAK;


/* Define Prolific Class function prototypes.  */

UINT  _ux_host_class_prolific_activate(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_prolific_configure(UX_HOST_CLASS_PROLIFIC *prolific);
UINT  _ux_host_class_prolific_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_prolific_endpoints_get(UX_HOST_CLASS_PROLIFIC *prolific);
UINT  _ux_host_class_prolific_entry(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_prolific_read (UX_HOST_CLASS_PROLIFIC *prolific, UCHAR *data_pointer, 
                                  ULONG requested_length, ULONG *actual_length);
UINT  _ux_host_class_prolific_write(UX_HOST_CLASS_PROLIFIC *prolific, UCHAR *data_pointer, 
                                  ULONG requested_length, ULONG *actual_length);
UINT  _ux_host_class_prolific_ioctl(UX_HOST_CLASS_PROLIFIC *prolific, ULONG request,
                                  VOID *parameter);
UINT  _ux_host_class_prolific_command(UX_HOST_CLASS_PROLIFIC *prolific, ULONG command,
                                    ULONG value, UCHAR *data_buffer, ULONG data_length);
VOID  _ux_host_class_prolific_transfer_request_completed(UX_TRANSFER *transfer_request);
UINT  _ux_host_class_prolific_reception_stop (UX_HOST_CLASS_PROLIFIC *prolific, 
                                    UX_HOST_CLASS_PROLIFIC_RECEPTION *prolific_reception);
UINT  _ux_host_class_prolific_reception_start (UX_HOST_CLASS_PROLIFIC *prolific, 
                                    UX_HOST_CLASS_PROLIFIC_RECEPTION *prolific_reception);
                                    
VOID  _ux_host_class_prolific_reception_callback (UX_TRANSFER *transfer_request);
UINT  _ux_host_class_prolific_setup(UX_HOST_CLASS_PROLIFIC *prolific);

/* Define Prolific Class API prototypes.  */

#define  ux_host_class_prolific_entry                           _ux_host_class_prolific_entry
#define  ux_host_class_prolific_read                            _ux_host_class_prolific_read
#define  ux_host_class_prolific_write                           _ux_host_class_prolific_write
#define  ux_host_class_prolific_ioctl                           _ux_host_class_prolific_ioctl
#define  ux_host_class_prolific_command                         _ux_host_class_prolific_command
#define  ux_host_class_prolific_reception_stop                  _ux_host_class_prolific_reception_stop
#define  ux_host_class_prolific_reception_start                 _ux_host_class_prolific_reception_start
#define  ux_host_class_prolific_setup                           _ux_host_class_prolific_setup

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif
