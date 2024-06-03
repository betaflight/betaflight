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
/**   CDC Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_device_class_cdc_acm.h                           PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file defines the equivalences for the USBX Device Class CDC    */ 
/*    ACM component.                                                      */ 
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
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added macro to disable      */
/*                                            transmission support,       */
/*                                            resulting in version 6.1.6  */
/*  08-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added write auto ZLP,       */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DEVICE_CLASS_CDC_ACM_H
#define UX_DEVICE_CLASS_CDC_ACM_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  

/* Defined, _write is pending ZLP automatically (complete transfer) after buffer is sent.  */

/* #define UX_DEVICE_CLASS_CDC_ACM_WRITE_AUTO_ZLP  */

/* Define CDC Class USB Class constants.  */
#define UX_SLAVE_CLASS_CDC_ACM_CLASS                                    10

/* Device CDC Requests */
#define UX_SLAVE_CLASS_CDC_ACM_SEND_ENCAPSULATED_COMMAND                0x00
#define UX_SLAVE_CLASS_CDC_ACM_GET_ENCAPSULATED_RESPONSE                0x01
#define UX_SLAVE_CLASS_CDC_ACM_SET_COMM_FEATURE                         0x02
#define UX_SLAVE_CLASS_CDC_ACM_GET_COMM_FEATURE                         0x03
#define UX_SLAVE_CLASS_CDC_ACM_CLEAR_COMM_FEATURE                       0x04
#define UX_SLAVE_CLASS_CDC_ACM_SET_AUX_LINE_STATE                       0x10
#define UX_SLAVE_CLASS_CDC_ACM_SET_HOOK_STATE                           0x11
#define UX_SLAVE_CLASS_CDC_ACM_PULSE_SETUP                              0x12
#define UX_SLAVE_CLASS_CDC_ACM_SEND_PULSE                               0x13
#define UX_SLAVE_CLASS_CDC_ACM_SET_PULSE_TIME                           0x14
#define UX_SLAVE_CLASS_CDC_ACM_RING_AUX_JACK                            0x15
#define UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING                          0x20
#define UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING                          0x21
#define UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE                   0x22
#define UX_SLAVE_CLASS_CDC_ACM_SEND_BREAK                               0x23
#define UX_SLAVE_CLASS_CDC_ACM_SET_RINGER_PARMS                         0x30
#define UX_SLAVE_CLASS_CDC_ACM_GET_RINGER_PARMS                         0x31
#define UX_SLAVE_CLASS_CDC_ACM_SET_OPERATION_PARMS                      0x32
#define UX_SLAVE_CLASS_CDC_ACM_GET_OPERATION_PARMS                      0x33
#define UX_SLAVE_CLASS_CDC_ACM_SET_LINE_PARMS                           0x34
#define UX_SLAVE_CLASS_CDC_ACM_GET_LINE_PARMS                           0x35
#define UX_SLAVE_CLASS_CDC_ACM_DIAL_DIGITS                              0x36
#define UX_SLAVE_CLASS_CDC_ACM_SET_UNIT_PARAMETER                       0x37
#define UX_SLAVE_CLASS_CDC_ACM_GET_UNIT_PARAMETER                       0x38
#define UX_SLAVE_CLASS_CDC_ACM_CLEAR_UNIT_PARAMETER                     0x39
#define UX_SLAVE_CLASS_CDC_ACM_GET_PROFILE                              0x3A
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_MULTICAST_FILTERS           0x40
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_POWER_MANAGEMENT_PATTERN    0x41
#define UX_SLAVE_CLASS_CDC_ACM_GET_ETHERNET_POWER_MANAGEMENT_PATTERN    0x42
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_PACKET_FILTER               0x43
#define UX_SLAVE_CLASS_CDC_ACM_GET_ETHERNET_STATISTIC                   0x44
#define UX_SLAVE_CLASS_CDC_ACM_SET_ATM_DATA_FORMAT                      0x50
#define UX_SLAVE_CLASS_CDC_ACM_GET_ATM_DEVICE_STATISTICS                0x51
#define UX_SLAVE_CLASS_CDC_ACM_SET_ATM_DEFAULT_VC                       0x52
#define UX_SLAVE_CLASS_CDC_ACM_GET_ATM_VC_STATISTICS                    0x53

/* Default line coding values.  */
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_BAUDRATE                     115200
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_STOP_BIT                     1
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARITY                       0
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_DATA_BIT                     8

/* Define line coding structure.  */
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_BAUDRATE_STRUCT              0
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_STOP_BIT_STRUCT              4
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARITY_STRUCT                5
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_DATA_BIT_STRUCT              6
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_RESPONSE_SIZE                7

/* Define line state bits.  */
#define UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_DTR                           1
#define UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_RTS                           2

/* Define Transfer direction bits.  */
#define UX_SLAVE_CLASS_CDC_ACM_ENDPOINT_XMIT                            1
#define UX_SLAVE_CLASS_CDC_ACM_ENDPOINT_RCV                             2

/* Define IOCTL functions.  */
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING                    1
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING                    2
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_STATE                     3
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_ABORT_PIPE                         4
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_STATE                     5
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_START                 6
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_STOP                  7
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_READ_TIMEOUT                   8
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_WRITE_TIMEOUT                  9

/* Define event group flag.  */
#define UX_DEVICE_CLASS_CDC_ACM_WRITE_EVENT                             1


/* CDC ACM read state machine states.  */
#define UX_DEVICE_CLASS_CDC_ACM_READ_START      (UX_STATE_STEP + 1)
#define UX_DEVICE_CLASS_CDC_ACM_READ_WAIT       (UX_STATE_STEP + 2)

/* CDC ACM write state machine states.  */
#define UX_DEVICE_CLASS_CDC_ACM_WRITE_START     (UX_STATE_STEP + 1)
#define UX_DEVICE_CLASS_CDC_ACM_WRITE_WAIT      (UX_STATE_STEP + 2)


/* Define Slave CDC Class Calling Parameter structure */

typedef struct UX_SLAVE_CLASS_CDC_ACM_PARAMETER_STRUCT
{
    VOID                    (*ux_slave_class_cdc_acm_instance_activate)(VOID *);
    VOID                    (*ux_slave_class_cdc_acm_instance_deactivate)(VOID *);
    VOID                    (*ux_slave_class_cdc_acm_parameter_change)(VOID *);

} UX_SLAVE_CLASS_CDC_ACM_PARAMETER;

/* Define CDC Class structure.  */

typedef struct UX_SLAVE_CLASS_CDC_ACM_STRUCT
{
    UX_SLAVE_INTERFACE                  *ux_slave_class_cdc_acm_interface;
    UX_SLAVE_CLASS_CDC_ACM_PARAMETER    ux_slave_class_cdc_acm_parameter;

#if !defined(UX_DEVICE_STANDALONE)
    UX_MUTEX                            ux_slave_class_cdc_acm_endpoint_in_mutex;
    UX_MUTEX                            ux_slave_class_cdc_acm_endpoint_out_mutex;
#else
    UCHAR                               *ux_device_class_cdc_acm_read_buffer;
    ULONG                               ux_device_class_cdc_acm_read_requested_length;
    ULONG                               ux_device_class_cdc_acm_read_transfer_length;
    ULONG                               ux_device_class_cdc_acm_read_actual_length;
    UINT                                ux_device_class_cdc_acm_read_status;
    UINT                                ux_device_class_cdc_acm_read_state;

    UCHAR                               *ux_device_class_cdc_acm_write_buffer;
    ULONG                               ux_device_class_cdc_acm_write_transfer_length;
    ULONG                               ux_device_class_cdc_acm_write_host_length;
    ULONG                               ux_device_class_cdc_acm_write_requested_length;
    ULONG                               ux_device_class_cdc_acm_write_actual_length;
    UINT                                ux_device_class_cdc_acm_write_status;
    UINT                                ux_device_class_cdc_acm_write_state;
#endif

    ULONG                               ux_slave_class_cdc_acm_baudrate;
    UCHAR                               ux_slave_class_cdc_acm_stop_bit;
    UCHAR                               ux_slave_class_cdc_acm_parity;
    UCHAR                               ux_slave_class_cdc_acm_data_bit;
    UCHAR                               ux_slave_class_cdc_acm_data_dtr_state;
    UCHAR                               ux_slave_class_cdc_acm_data_rts_state;
    UCHAR                               reserved[3];

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
#if !defined(UX_DEVICE_STANDALONE)
    UX_THREAD                           ux_slave_class_cdc_acm_bulkin_thread;
    UX_THREAD                           ux_slave_class_cdc_acm_bulkout_thread;
    UX_EVENT_FLAGS_GROUP                ux_slave_class_cdc_acm_event_flags_group;
    UCHAR                               *ux_slave_class_cdc_acm_bulkin_thread_stack;
    UCHAR                               *ux_slave_class_cdc_acm_bulkout_thread_stack;
#endif
    UINT                                (*ux_device_class_cdc_acm_write_callback)(struct UX_SLAVE_CLASS_CDC_ACM_STRUCT *cdc_acm, UINT status, ULONG length);
    UINT                                (*ux_device_class_cdc_acm_read_callback)(struct UX_SLAVE_CLASS_CDC_ACM_STRUCT *cdc_acm, UINT status, UCHAR *data_pointer, ULONG length);
    ULONG                               ux_slave_class_cdc_acm_transmission_status;
    ULONG                               ux_slave_class_cdc_acm_scheduled_write;
#if !defined(UX_DEVICE_STANDALONE)
    ULONG                               ux_slave_class_cdc_acm_callback_total_length;
    UCHAR                               *ux_slave_class_cdc_acm_callback_data_pointer;
    UCHAR                               *ux_slave_class_cdc_acm_callback_current_data_pointer;
#endif
#endif
} UX_SLAVE_CLASS_CDC_ACM;

/* Define some CDC Class structures */

typedef struct UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER_STRUCT 
{
    ULONG                               ux_slave_class_cdc_acm_parameter_baudrate;
    UCHAR                               ux_slave_class_cdc_acm_parameter_stop_bit;
    UCHAR                               ux_slave_class_cdc_acm_parameter_parity;
    UCHAR                               ux_slave_class_cdc_acm_parameter_data_bit;
    
} UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER;

typedef struct UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER_STRUCT 
{
    UCHAR                               ux_slave_class_cdc_acm_parameter_rts;
    UCHAR                               ux_slave_class_cdc_acm_parameter_dtr;
    
} UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER;

typedef struct UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER_STRUCT 
{
    UINT                                (*ux_device_class_cdc_acm_parameter_write_callback)(struct UX_SLAVE_CLASS_CDC_ACM_STRUCT *cdc_acm, UINT status, ULONG length);
    UINT                                (*ux_device_class_cdc_acm_parameter_read_callback)(struct UX_SLAVE_CLASS_CDC_ACM_STRUCT *cdc_acm, UINT status, UCHAR *data_pointer, ULONG length);

} UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER;



/* Requests - Ethernet Networking Control Model */

#define UX_SLAVE_CLASS_CDC_ACM_SEND_ENCAPSULATED_COMMAND                        0x00        
                                        /* Issues a command in the format of the supported control
                                           protocol. The intent of this mechanism is to support
                                           networking devices (e.g., host-based cable modems)
                                           that require an additional vendor-defined interface for
                                           media specific hardware configuration and
                                           management.  */
#define UX_SLAVE_CLASS_CDC_ACM_GET_ENCAPSULATED_RESPONSE                        0x01        
                                        /* Requests a response in the format of the supported
                                           control protocol.  */
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_MULTICAST_FILTERS                   0x40        
                                        /* As applications are loaded and unloaded on the host,
                                           the networking transport will instruct the device's MAC
                                           driver to change settings of the Networking device's
                                           multicast filters.  */
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER     0x41        
                                        /* Some hosts are able to conserve energy and stay quiet
                                           in a 'sleeping' state while not being used. USB
                                           Networking devices may provide special pattern filtering
                                           hardware that enables it to wake up the attached host
                                           on demand when something is attempting to contact the
                                           host (e.g., an incoming web browser connection).
                                           Primitives are needed in management plane to negotiate
                                           the setting of these special filters  */
#define UX_SLAVE_CLASS_CDC_ACM_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER     0x42 
                                        /* Retrieves the status of the above power management
                                           pattern filter setting  */
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_PACKET_FILTER                       0x43 
                                        /* Sets device filter for running a network analyzer
                                           application on the host machine  */
#define UX_SLAVE_CLASS_CDC_ACM_GET_ETHERNET_STATISTIC                           0x44 
                                        /* Retrieves Ethernet device statistics such as frames
                                           transmitted, frames received, and bad frames received.  */

/* Define buffer length for IN/OUT pipes.  */

#define UX_SLAVE_CLASS_CDC_ACM_BUFFER_SIZE                  4096


/* Define Device CDC Class prototypes.  */

UINT  _ux_device_class_cdc_acm_activate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_control_request(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_deactivate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_entry(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_uninitialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_write(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UCHAR *buffer, 
                                ULONG requested_length, ULONG *actual_length);
UINT  _ux_device_class_cdc_acm_read(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UCHAR *buffer, 
                                ULONG requested_length, ULONG *actual_length);
UINT  _ux_device_class_cdc_acm_ioctl(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, ULONG ioctl_function,
                                    VOID *parameter);
VOID  _ux_device_class_cdc_acm_bulkin_thread(ULONG class_pointer);
VOID  _ux_device_class_cdc_acm_bulkout_thread(ULONG class_pointer);
UINT  _ux_device_class_cdc_acm_write_with_callback(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UCHAR *buffer, 
                                ULONG requested_length);

UINT  _ux_device_class_cdc_acm_write_run(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UCHAR *buffer, 
                                ULONG requested_length, ULONG *actual_length);
UINT  _ux_device_class_cdc_acm_read_run(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UCHAR *buffer, 
                                ULONG requested_length, ULONG *actual_length);

UINT  _ux_device_class_cdc_acm_tasks_run(VOID *instance);

/* Define Device CDC Class API prototypes.  */

#define ux_device_class_cdc_acm_entry               _ux_device_class_cdc_acm_entry
#define ux_device_class_cdc_acm_read                _ux_device_class_cdc_acm_read
#define ux_device_class_cdc_acm_write               _ux_device_class_cdc_acm_write
#define ux_device_class_cdc_acm_ioctl               _ux_device_class_cdc_acm_ioctl
#define ux_device_class_cdc_acm_write_with_callback _ux_device_class_cdc_acm_write_with_callback

#define ux_device_class_cdc_acm_read_run            _ux_device_class_cdc_acm_read_run
#define ux_device_class_cdc_acm_write_run           _ux_device_class_cdc_acm_write_run

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif /* UX_DEVICE_CLASS_CDC_ACM_H */
