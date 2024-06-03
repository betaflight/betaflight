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
/**   CDC ACM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_cdc_acm.h                             PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX CDC ACM class.                                                 */ 
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
/*                                            added standalone support,   */
/*                                            used defined line coding    */
/*                                            instead of magic number,    */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_HOST_CLASS_CDC_ACM_H
#define UX_HOST_CLASS_CDC_ACM_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  

/* Define CDC ACM Class constants.  */

#define UX_HOST_CLASS_CDC_ACM_DEVICE_INIT_DELAY                 1000
#define UX_HOST_CLASS_CDC_ACM_CLASS_TRANSFER_TIMEOUT            300000
#define UX_HOST_CLASS_CDC_DATA_CLASS                            0x0A
#define UX_HOST_CLASS_CDC_CONTROL_CLASS                         0x02
#define UX_HOST_CLASS_CDC_ACM_SUBCLASS                          0X02
#define UX_HOST_CLASS_CDC_DLC_SUBCLASS                          0X01
#define UX_HOST_CLASS_CDC_ACM_CS_INTERFACE                      0x24

/* Define CDC ACM Class descriptor subtypes in functional descriptors.  */
#define UX_HOST_CLASS_CDC_ACM_HEADER_DESCRIPTOR                 0X00
#define UX_HOST_CLASS_CDC_ACM_CALL_MANAGEMENT_DESCRIPTOR        0X01
#define UX_HOST_CLASS_CDC_ACM_ABSTRACT_CONTROL_MGT_DESCRIPTOR   0X02
#define UX_HOST_CLASS_CDC_ACM_DIRECT_LINE_MGT_DESCRIPTOR        0X03
#define UX_HOST_CLASS_CDC_ACM_TELEPHONE_RINGER_DESCRIPTOR       0X04
#define UX_HOST_CLASS_CDC_ACM_REPORT_CAPABILITY_DESCRIPTOR      0X05
#define UX_HOST_CLASS_CDC_ACM_UNION_DESCRIPTOR                  0X06
#define UX_HOST_CLASS_CDC_ACM_COUNTRY_SELECTION_DESCRIPTOR      0X07
#define UX_HOST_CLASS_CDC_ACM_TELEPHONE_OPERATIONAL_DESCRIPTOR  0X08
#define UX_HOST_CLASS_CDC_ACM_USB_TERMINAL_DESCRIPTOR           0X09

/* Define CDC ACM Class call management descriptors.  */
#define UX_HOST_CLASS_CDC_ACM_CALL_MANAGEMENT_CAPABILITIES      0x03
#define UX_HOST_CLASS_CDC_ACM_CALL_MANAGEMENT_DCM               0x01
#define UX_HOST_CLASS_CDC_ACM_CALL_MANAGEMENT_DCI               0x02

/* Define CDC ACM Class union interface functional descriptors.  */
#define UX_HOST_CLASS_CDC_ACM_UNION_FUNCTIONAL_MASTER           0x03

/* Define CDC ACM command request values.  */

#define UX_HOST_CLASS_CDC_ACM_REQ_SEND_ENCAPSULATED_COMMAND     0x00
#define UX_HOST_CLASS_CDC_ACM_REQ_GET_ENCAPSULATED_COMMAND      0x01
#define UX_HOST_CLASS_CDC_ACM_REQ_SET_COMM_FEATURE              0x02
#define UX_HOST_CLASS_CDC_ACM_REQ_GET_COMM_FEATURE              0x03
#define UX_HOST_CLASS_CDC_ACM_REQ_CLEAR_COMM_FEATURE            0x04
#define UX_HOST_CLASS_CDC_ACM_REQ_SET_AUX_LINE_STATE            0x10
#define UX_HOST_CLASS_CDC_ACM_REQ_SET_HOOK_STATE                0x11
#define UX_HOST_CLASS_CDC_ACM_REQ_PULSE_SETUP                   0x12
#define UX_HOST_CLASS_CDC_ACM_REQ_SEND_PULSE                    0x13
#define UX_HOST_CLASS_CDC_ACM_REQ_SET_PUSLE_TIME                0x14
#define UX_HOST_CLASS_CDC_ACM_REQ_RING_AUX_JACK                 0x15
#define UX_HOST_CLASS_CDC_ACM_REQ_SET_LINE_CODING               0x20
#define UX_HOST_CLASS_CDC_ACM_REQ_GET_LINE_CODING               0x21
#define UX_HOST_CLASS_CDC_ACM_REQ_SET_LINE_STATE                0x22
#define UX_HOST_CLASS_CDC_ACM_REQ_SEND_BREAK                    0x23
#define UX_HOST_CLASS_CDC_ACM_REQ_SET_RINGER_PARMS              0x30
#define UX_HOST_CLASS_CDC_ACM_REQ_GET_RINGER_PARMS              0x31 
#define UX_HOST_CLASS_CDC_ACM_REQ_SET_OPERATION_PARMS           0x32
#define UX_HOST_CLASS_CDC_ACM_REQ_GET_OPERATION_PARMS           0x33
#define UX_HOST_CLASS_CDC_ACM_REQ_SET_LINE_PARMS                0x34
#define UX_HOST_CLASS_CDC_ACM_REQ_GET_LINE_PARMS                0x35

/* Define CDC ACM line output control values.  */

#define UX_HOST_CLASS_CDC_ACM_CTRL_DTR                          0x01
#define UX_HOST_CLASS_CDC_ACM_CTRL_RTS                          0x02

/* Define CDC ACM line input control values.  */

#define UX_HOST_CLASS_CDC_ACM_CTRL_DCD                          0x01
#define UX_HOST_CLASS_CDC_ACM_CTRL_DSR                          0x02
#define UX_HOST_CLASS_CDC_ACM_CTRL_BRK                          0x04
#define UX_HOST_CLASS_CDC_ACM_CTRL_RI                           0x08

#define UX_HOST_CLASS_CDC_ACM_CTRL_FRAMING                      0x10
#define UX_HOST_CLASS_CDC_ACM_CTRL_PARITY                       0x20
#define UX_HOST_CLASS_CDC_ACM_CTRL_OVERRUN                      0x40

/* Define CDC ACM Class packet equivalences.  */

#define UX_HOST_CLASS_CDC_ACM_PACKET_SIZE                       128

/* Define CDC ACM default values.  */

#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_RATE          9600
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_STOP_BIT      0
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_PARITY        0
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_DATA_BIT      8

/* Define CDC ACM line coding definitions.  */

#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_STOP_BIT_0            0
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_STOP_BIT_15           1
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_STOP_BIT_2            2

#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_PARITY_NONE           0
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_PARITY_ODD            1
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_PARITY_EVEN           2
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_PARITY_MARK           3
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_PARITY_SPACE          4

#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_LENGTH                7
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_RATE                  0
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_STOP_BIT              4
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_PARITY                5
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_DATA_BIT              6

/* Define CDC ACM line state definitions.  */

#define UX_HOST_CLASS_CDC_ACM_LINE_STATE_STOP_BIT_0             0
#define UX_HOST_CLASS_CDC_ACM_LINE_CODING_STOP_BIT_15           1

/* Define CDC ACM IOCTL Functions. */

#define UX_HOST_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING             0
#define UX_HOST_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING             1
#define UX_HOST_CLASS_CDC_ACM_IOCTL_SET_LINE_STATE              2
#define UX_HOST_CLASS_CDC_ACM_IOCTL_SEND_BREAK                  3
#define UX_HOST_CLASS_CDC_ACM_IOCTL_ABORT_IN_PIPE               5
#define UX_HOST_CLASS_CDC_ACM_IOCTL_ABORT_OUT_PIPE              6
#define UX_HOST_CLASS_CDC_ACM_IOCTL_NOTIFICATION_CALLBACK       7
#define UX_HOST_CLASS_CDC_ACM_IOCTL_GET_DEVICE_STATUS           8
#define UX_HOST_CLASS_CDC_ACM_IOCTL_WRITE_CALLBACK              9
#define UX_HOST_CLASS_CDC_ACM_IOCTL_GET_WRITE_STATUS            10

/* Define CDC ACM Reception States. */

#define UX_HOST_CLASS_CDC_ACM_RECEPTION_STATE_STOPPED           0
#define UX_HOST_CLASS_CDC_ACM_RECEPTION_STATE_STARTED           1
#define UX_HOST_CLASS_CDC_ACM_RECEPTION_STATE_IN_TRANSFER       2

/* Define supported notification types.  */

#define UX_HOST_CLASS_CDC_ACM_NOTIFICATION_NETWORK_CONNECTION   0x00
#define UX_HOST_CLASS_CDC_ACM_NOTIFICATION_RESPONSE_AVAILABLE   0x01
#define UX_HOST_CLASS_CDC_ACM_NOTIFICATION_SERIAL_STATE         0x20
#define UX_HOST_CLASS_CDC_ACM_NOTIFICATION_CALL_STATE_CHANGE    0x28
#define UX_HOST_CLASS_CDC_ACM_NOTIFICATION_LINE_STATE_CHANGE    0x29
#define UX_HOST_CLASS_CDC_ACM_NOTIFICATION_SPEED_CHANGE         0x2A

/* Define notification packet format.  */

#define UX_HOST_CLASS_CDC_ACM_NPF_REQUEST_TYPE                  0x00
#define UX_HOST_CLASS_CDC_ACM_NPF_NOTIFICATION_TYPE             0x01
#define UX_HOST_CLASS_CDC_ACM_NPF_VALUE                         0x02
#define UX_HOST_CLASS_CDC_ACM_NPF_INDEX                         0x04
#define UX_HOST_CLASS_CDC_ACM_NPF_LENGTH                        0x06

/* Define CDC ACM Class instance structure.  */

typedef struct UX_HOST_CLASS_CDC_ACM_STRUCT
{
    struct UX_HOST_CLASS_CDC_ACM_STRUCT  
                   *ux_host_class_cdc_acm_next_instance;
    UX_HOST_CLASS  *ux_host_class_cdc_acm_class;
    UX_DEVICE      *ux_host_class_cdc_acm_device;
    UX_ENDPOINT    *ux_host_class_cdc_acm_bulk_in_endpoint;
    UX_ENDPOINT    *ux_host_class_cdc_acm_bulk_out_endpoint;
    UX_ENDPOINT    *ux_host_class_cdc_acm_interrupt_endpoint;
    UX_INTERFACE   *ux_host_class_cdc_acm_interface;
    UINT           ux_host_class_cdc_acm_instance_status;
    UINT           ux_host_class_cdc_acm_state;
    ULONG          ux_host_class_cdc_acm_notification_count;
    ULONG          ux_host_class_cdc_acm_capabilities;
    ULONG          ux_host_class_cdc_acm_device_state;
    struct UX_HOST_CLASS_CDC_ACM_RECEPTION_STRUCT  
                   *ux_host_class_cdc_acm_reception;
    
    VOID           (*ux_host_class_cdc_acm_device_status_change_callback)(struct UX_HOST_CLASS_CDC_ACM_STRUCT *cdc_acm, 
                                                                ULONG  notification_type, ULONG notification_value);
#if !defined(UX_HOST_STANDALONE)
    UX_SEMAPHORE   ux_host_class_cdc_acm_semaphore;
#else
    UINT           ux_host_class_cdc_acm_status;
    VOID           *ux_host_class_cdc_acm_allocated;
    ULONG          ux_host_class_cdc_acm_interfaces_bitmap;
    ULONG          ux_host_class_cdc_acm_tick;
    struct UX_HOST_CLASS_CDC_ACM_STRUCT
                   *ux_host_class_cdc_acm_control;
    ULONG          ux_host_class_cdc_acm_write_length;
    ULONG          ux_host_class_cdc_acm_write_count;
    VOID           (*ux_host_class_cdc_acm_write_callback)(
                                struct UX_HOST_CLASS_CDC_ACM_STRUCT *cdc_acm,
                                UINT status, ULONG length);
    UCHAR          ux_host_class_cdc_acm_cmd_state;
    UCHAR          ux_host_class_cdc_acm_read_state;
    UCHAR          ux_host_class_cdc_acm_write_state;
    UCHAR          ux_host_class_cdc_acm_next_state;
#endif
} UX_HOST_CLASS_CDC_ACM;

/* Define CDC DLC Class instance structure.  */


typedef struct UX_HOST_CLASS_CDC_DLC_STRUCT
{
    struct UX_HOST_CLASS_CDC_DLC_STRUCT  
                   *ux_host_class_cdc_dlc_next_instance;
    UX_HOST_CLASS  *ux_host_class_cdc_dlc_class;
    UX_DEVICE      *ux_host_class_cdc_dlc_device;
    UX_ENDPOINT    *ux_host_class_cdc_dlc_bulk_in_endpoint;
    UX_ENDPOINT    *ux_host_class_cdc_dlc_bulk_out_endpoint;
    UX_ENDPOINT    *ux_host_class_cdc_dlc_interrupt_endpoint;
    UX_INTERFACE   *ux_host_class_cdc_dlc_interface;
    UINT           ux_host_class_cdc_dlc_instance_status;
    UINT           ux_host_class_cdc_dlc_state;
    ULONG          ux_host_class_cdc_dlc_notification_count;
    ULONG          ux_host_class_cdc_dlc_capabilities;
    struct UX_HOST_CLASS_CDC_DLC_RECEPTION_STRUCT  
                   *ux_host_class_cdc_dlc_reception;
#if !defined(UX_HOST_STANDALONE)
    UX_SEMAPHORE   ux_host_class_cdc_dlc_semaphore;
#else
    ULONG          ux_host_class_cdc_dlc_interfaces_bitmap;
    struct UX_HOST_CLASS_CDC_ACM_STRUCT
                   *ux_host_class_cdc_dlc_control;
    ULONG          ux_host_class_cdc_dlc_write_length;
    ULONG          ux_host_class_cdc_dlc_write_count;
    VOID           (*ux_host_class_cdc_dlc_write_callback)(
                                struct UX_HOST_CLASS_CDC_ACM_STRUCT *cdc_acm,
                                UINT status, ULONG length);
    UCHAR          ux_host_class_cdc_dlc_cmd_state;
    UCHAR          ux_host_class_cdc_dlc_read_state;
    UCHAR          ux_host_class_cdc_dlc_write_state;
    UCHAR          ux_host_class_cdc_dlc_next_state;
#endif
} UX_HOST_CLASS_CDC_DLC;

/* Define CDC ACM reception structure. */

typedef struct UX_HOST_CLASS_CDC_ACM_RECEPTION_STRUCT
{

    ULONG          ux_host_class_cdc_acm_reception_state;
    ULONG          ux_host_class_cdc_acm_reception_block_size;
    UCHAR          *ux_host_class_cdc_acm_reception_data_buffer;
    ULONG          ux_host_class_cdc_acm_reception_data_buffer_size;
    UCHAR          *ux_host_class_cdc_acm_reception_data_head;
    UCHAR          *ux_host_class_cdc_acm_reception_data_tail;
    VOID           (*ux_host_class_cdc_acm_reception_callback)(struct UX_HOST_CLASS_CDC_ACM_STRUCT *cdc_acm, 
                                                                UINT  status,
                                                                UCHAR *reception_buffer, 
                                                                ULONG reception_size);

} UX_HOST_CLASS_CDC_ACM_RECEPTION;

/* Define CDC ACM Line Coding IOCTL structure.  */

typedef struct UX_HOST_CLASS_CDC_ACM_LINE_CODING_STRUCT
{

    ULONG          ux_host_class_cdc_acm_line_coding_dter;
    ULONG          ux_host_class_cdc_acm_line_coding_stop_bit;
    ULONG          ux_host_class_cdc_acm_line_coding_parity;
    ULONG          ux_host_class_cdc_acm_line_coding_data_bits;

} UX_HOST_CLASS_CDC_ACM_LINE_CODING;

/* Define CDC ACM Line State IOCTL structure.  */

typedef struct UX_HOST_CLASS_CDC_ACM_LINE_STATE_STRUCT
{

    ULONG          ux_host_class_cdc_acm_line_state_rts;
    ULONG          ux_host_class_cdc_acm_line_state_dtr;

} UX_HOST_CLASS_CDC_ACM_LINE_STATE;

/* Define CDC ACM Line break IOCTL structure.  */

typedef struct UX_HOST_CLASS_CDC_ACM_LINE_BREAK_STRUCT
{

    ULONG           ux_host_class_cdc_acm_line_break;

} UX_HOST_CLASS_CDC_ACM_LINE_BREAK;

/* Define CDC ACM Class function prototypes.  */

UINT  _ux_host_class_cdc_acm_activate(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_cdc_acm_configure(UX_HOST_CLASS_CDC_ACM *cdc_acm);
UINT  _ux_host_class_cdc_acm_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_cdc_acm_endpoints_get(UX_HOST_CLASS_CDC_ACM *cdc_acm);
UINT  _ux_host_class_cdc_acm_entry(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_cdc_acm_read (UX_HOST_CLASS_CDC_ACM *cdc_acm, UCHAR *data_pointer, 
                                  ULONG requested_length, ULONG *actual_length);
UINT  _ux_host_class_cdc_acm_write(UX_HOST_CLASS_CDC_ACM *cdc_acm, UCHAR *data_pointer, 
                                  ULONG requested_length, ULONG *actual_length);
UINT  _ux_host_class_cdc_acm_ioctl(UX_HOST_CLASS_CDC_ACM *cdc_acm, ULONG request,
                                  VOID *parameter);
UINT  _ux_host_class_cdc_acm_command(UX_HOST_CLASS_CDC_ACM *cdc_acm, ULONG command,
                                    ULONG value, UCHAR *data_buffer, ULONG data_length);
VOID  _ux_host_class_cdc_acm_transfer_request_completed(UX_TRANSFER *transfer_request);
UINT  _ux_host_class_cdc_acm_capabilities_get(UX_HOST_CLASS_CDC_ACM *cdc_acm);
UINT  _ux_host_class_cdc_acm_reception_stop (UX_HOST_CLASS_CDC_ACM *cdc_acm, 
                                    UX_HOST_CLASS_CDC_ACM_RECEPTION *cdc_acm_reception);
UINT  _ux_host_class_cdc_acm_reception_start (UX_HOST_CLASS_CDC_ACM *cdc_acm, 
                                    UX_HOST_CLASS_CDC_ACM_RECEPTION *cdc_acm_reception);
                                    
VOID  _ux_host_class_cdc_acm_reception_callback (UX_TRANSFER *transfer_request);

UINT  _ux_host_class_cdc_acm_write_with_callback(UX_HOST_CLASS_CDC_ACM *cdc_acm, UCHAR *data_pointer, 
                                  ULONG requested_length);
VOID  _ux_host_class_cdc_acm_transmission_callback(UX_TRANSFER *transfer_request);

/* Define CDC ACM Class API prototypes.  */

#define ux_host_class_cdc_acm_entry                     _ux_host_class_cdc_acm_entry
#define ux_host_class_cdc_acm_read                      _ux_host_class_cdc_acm_read
#define ux_host_class_cdc_acm_write                     _ux_host_class_cdc_acm_write
#define ux_host_class_cdc_acm_ioctl                     _ux_host_class_cdc_acm_ioctl
#define ux_host_class_cdc_acm_reception_start           _ux_host_class_cdc_acm_reception_start
#define ux_host_class_cdc_acm_reception_stop            _ux_host_class_cdc_acm_reception_stop

#define ux_host_class_cdc_acm_write_with_callback       _ux_host_class_cdc_acm_write_with_callback

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif

#endif
