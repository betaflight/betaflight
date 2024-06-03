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
/**   CDC_ECM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_device_class_cdc_ecm.h                           PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file defines the equivalences for the USBX Device Class        */ 
/*    CDC_ECM component.                                                  */ 
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
/*                                            fixed spelling error,       */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added wait definitions,     */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DEVICE_CLASS_CDC_ECM_H
#define UX_DEVICE_CLASS_CDC_ECM_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  

#if !defined(UX_DEVICE_STANDALONE)
#include "nx_api.h"
#include "ux_network_driver.h"
#else

/* Assume NX definitions for compiling.  */
#define NX_PACKET                                               VOID*
/*
UINT  _ux_network_driver_deactivate(VOID *ux_instance, VOID *ux_network_handle);
VOID  _ux_network_driver_link_up(VOID *ux_network_handle);
VOID  _ux_network_driver_link_down(VOID *ux_network_handle);
*/
#ifndef _ux_network_driver_deactivate
#define _ux_network_driver_deactivate(a,b)                      do {} while(0)
#endif
#ifndef _ux_network_driver_link_up
#define _ux_network_driver_link_up(a)                           do {} while(0)
#endif
#ifndef _ux_network_driver_link_down
#define _ux_network_driver_link_down(a)                         do {} while(0)
#endif
#endif

/* Define generic CDC_ECM equivalences.  */
#define UX_DEVICE_CLASS_CDC_ECM_CLASS_COMMUNICATION_CONTROL                 0x02
#define UX_DEVICE_CLASS_CDC_ECM_SUBCLASS_COMMUNICATION_CONTROL              0x06
#define UX_DEVICE_CLASS_CDC_ECM_CLASS_COMMUNICATION_DATA                    0x0A
#define UX_DEVICE_CLASS_CDC_ECM_NEW_INTERRUPT_EVENT                         0x01
#define UX_DEVICE_CLASS_CDC_ECM_NEW_BULKOUT_EVENT                           0x02
#define UX_DEVICE_CLASS_CDC_ECM_NEW_BULKIN_EVENT                            0x04
#define UX_DEVICE_CLASS_CDC_ECM_NEW_DEVICE_STATE_CHANGE_EVENT               0x08
#define UX_DEVICE_CLASS_CDC_ECM_NETWORK_NOTIFICATION_EVENT                  0x10
#define UX_DEVICE_CLASS_CDC_ECM_INTERRUPT_RESPONSE_LENGTH                   8
#define UX_DEVICE_CLASS_CDC_ECM_MAX_CONTROL_RESPONSE_LENGTH                 256
#define UX_DEVICE_CLASS_CDC_ECM_INTERRUPT_RESPONSE_AVAILABLE_FLAG           1
#define UX_DEVICE_CLASS_CDC_ECM_BASE_IP_ADDRESS                             0xC0A80001
#define UX_DEVICE_CLASS_CDC_ECM_BASE_IP_MASK                                0xFFFFFF00
#define UX_DEVICE_CLASS_CDC_ECM_MAX_MTU                                     1518
#define UX_DEVICE_CLASS_CDC_ECM_ETHERNET_IP                                 0x0800
#define UX_DEVICE_CLASS_CDC_ECM_ETHERNET_ARP                                0x0806
#define UX_DEVICE_CLASS_CDC_ECM_ETHERNET_RARP                               0x8035
#define UX_DEVICE_CLASS_CDC_ECM_ETHERNET_PACKET_SIZE                        1536    
#define UX_DEVICE_CLASS_CDC_ECM_NX_ALIGN_PADDING                            2
#ifndef UX_DEVICE_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES
#define UX_DEVICE_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES                           16  
#endif

#define UX_DEVICE_CLASS_CDC_ECM_NX_PACKET_SIZE                              sizeof(NX_PACKET)

#define UX_DEVICE_CLASS_CDC_ECM_NX_PAYLOAD_SIZE_ASSERT                      UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(UX_DEVICE_CLASS_CDC_ECM_ETHERNET_PACKET_SIZE, UX_DEVICE_CLASS_CDC_ECM_NX_ALIGN_PADDING), UX_DEVICE_CLASS_CDC_ECM_NX_PAYLOAD_SIZE_calc_ovf)
#define UX_DEVICE_CLASS_CDC_ECM_NX_PAYLOAD_SIZE                             (UX_DEVICE_CLASS_CDC_ECM_ETHERNET_PACKET_SIZE + UX_DEVICE_CLASS_CDC_ECM_NX_ALIGN_PADDING)

#define UX_DEVICE_CLASS_CDC_ECM_NX_BUFF_SIZE_ASSERT                         \
    UX_DEVICE_CLASS_CDC_ECM_NX_PAYLOAD_SIZE_ASSERT                          \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(                    \
        UX_DEVICE_CLASS_CDC_ECM_NX_PAYLOAD_SIZE,                            \
        UX_DEVICE_CLASS_CDC_ECM_NX_PACKET_SIZE),                            \
        UX_DEVICE_CLASS_CDC_ECM_NX_BUFF_SIZE_calc_ovf)
#define UX_DEVICE_CLASS_CDC_ECM_NX_BUFF_SIZE                                (UX_DEVICE_CLASS_CDC_ECM_NX_PAYLOAD_SIZE + UX_DEVICE_CLASS_CDC_ECM_NX_PACKET_SIZE)

#define UX_DEVICE_CLASS_CDC_ECM_NX_ETHERNET_POOL_ALLOCSIZE_ASSERT           \
    UX_DEVICE_CLASS_CDC_ECM_NX_BUFF_SIZE_ASSERT                             \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_MULC_ULONG(                   \
        UX_DEVICE_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES,                          \
        UX_DEVICE_CLASS_CDC_ECM_NX_BUFF_SIZE),                              \
        UX_DEVICE_CLASS_CDC_ECM_NX_ETHERNET_POOL_ALLOCSIZE_calc1_ovf)       \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(                    \
        UX_DEVICE_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES *                         \
            UX_DEVICE_CLASS_CDC_ECM_NX_BUFF_SIZE,                           \
        32), UX_DEVICE_CLASS_CDC_ECM_NX_ETHERNET_POOL_ALLOCSIZE_calc2_ovf)
#define UX_DEVICE_CLASS_CDC_ECM_NX_ETHERNET_POOL_ALLOCSIZE                  (UX_DEVICE_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES * UX_DEVICE_CLASS_CDC_ECM_NX_BUFF_SIZE + 32)

#define UX_DEVICE_CLASS_CDC_ECM_ETHERNET_SIZE                               14
#define UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH                              6  
#define UX_DEVICE_CLASS_CDC_ECM_VENDOR_DESCRIPTION_MAX_LENGTH               64  
#define UX_DEVICE_CLASS_CDC_ECM_MAC_OPTIONS                                 8
#define UX_DEVICE_CLASS_CDC_ECM_PACKET_HEADER_MSG                           1 

/* Device CDC_ECM Requests */
#define UX_DEVICE_CLASS_CDC_ECM_SEND_ENCAPSULATED_COMMAND                   0x00
#define UX_DEVICE_CLASS_CDC_ECM_GET_ENCAPSULATED_RESPONSE                   0x01
#define UX_DEVICE_CLASS_CDC_ECM_SET_ETHERNET_MULTICAST_FILTER               0x40
#define UX_DEVICE_CLASS_CDC_ECM_SET_ETHERNET_POWER_MANAGEMENT_FILTER        0x41
#define UX_DEVICE_CLASS_CDC_ECM_GET_ETHERNET_POWER_MANAGEMENT_FILTER        0x42
#define UX_DEVICE_CLASS_CDC_ECM_SET_ETHERNET_PACKET_FILTER                  0x43

/* Define CDC_ECM Versions.  Set to 1.0 here.  */
#define UX_DEVICE_CLASS_CDC_ECM_VERSION_MAJOR                               0x00000001
#define UX_DEVICE_CLASS_CDC_ECM_VERSION_MINOR                               0x00000000

/* Define CDC_ECM Connection type supported. Set to conectionless.  */
#define UX_DEVICE_CLASS_CDC_ECM_DF_CONNECTIONLESS                           0x00000001
#define UX_DEVICE_CLASS_CDC_ECM_DF_CONNECTION_ORIENTED                      0x00000002
#define UX_DEVICE_CLASS_CDC_ECM_DF_CONNECTION_SUPPORTED                     UX_DEVICE_CLASS_CDC_ECM_DF_CONNECTIONLESS

/* Define CDC_ECM Medium supported by the device. */
#define UX_DEVICE_CLASS_CDC_ECM_MEDIUM_SUPPORTED                            0x00000000

/* Define CDC_ECM Packet size and types supported.  */
#define UX_DEVICE_CLASS_CDC_ECM_MAX_PACKET_PER_TRANSFER                     0x00000001
#define UX_DEVICE_CLASS_CDC_ECM_MAX_PACKET_TRANSFER_SIZE                    0x00000640
#define UX_DEVICE_CLASS_CDC_ECM_PACKET_ALIGNEMENT_FACTOR                    0x00000003
#define UX_DEVICE_CLASS_CDC_ECM_MAX_FRAME_SIZE                              0x000005DC
#define UX_DEVICE_CLASS_CDC_ECM_MAX_PACKET_LENGTH                           0x000005EA

/* Define LINK speeds.  */
#define UX_DEVICE_CLASS_CDC_ECM_LINK_SPEED_FS                               0x0001D4C0

/* Define LINK statess.  */
#define UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_DOWN                             0
#define UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_UP                               1
#define UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_PENDING_UP                       2
#define UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_PENDING_DOWN                     3

/* Define media connection values.  */
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_CONNECTED                             0x00000000
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_DISCONNECTED                          0x00000001

/* Define media supported values.  */
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_802_3                                 0x00000000
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_802_5                                 0x00000001
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_FDDI                                  0x00000002
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_WAN                                   0x00000003
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_LOCAL_TALK                            0x00000004
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_DIX                                   0x00000005
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_ARCNET_RAW                            0x00000006
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_ARCNET_878_2                          0x00000007
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_ATM                                   0x00000008
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_WIRELESS_WAN                          0x00000009
#define UX_DEVICE_CLASS_CDC_ECM_MEDIA_IRDA                                  0x0000000A

/* Define CDC_ECM status values.  */
#define UX_DEVICE_CLASS_CDC_ECM_STATUS_SUCCESS                              0x00000000
#define UX_DEVICE_CLASS_CDC_ECM_STATUS_FAILURE                              0xC0000001
#define UX_DEVICE_CLASS_CDC_ECM_STATUS_INVALID_DATA                         0xC0010015
#define UX_DEVICE_CLASS_CDC_ECM_STATUS_NOT_SUPPORTED                        0xC00000BB
#define UX_DEVICE_CLASS_CDC_ECM_STATUS_MEDIA_CONNECTED                      0x4001000B
#define UX_DEVICE_CLASS_CDC_ECM_STATUS_MEDIA_DISCONNECT                     0x4001000C

/* Define CDC_ECM Control Messages values.  */

/* Define CDC_ECM State machine.  */
#define UX_DEVICE_CLASS_CDC_ECM_STATE_UNINITIALIZED                         0x00000000
#define UX_DEVICE_CLASS_CDC_ECM_STATE_INITIALIZED                           0x00000001
#define UX_DEVICE_CLASS_CDC_ECM_STATE_DATA_INITIALIZED                      0x00000002

/* Define NetX errors inside the CDC_ECM class.  */
#define UX_DEVICE_CLASS_CDC_ECM_NX_SUCCESS                                  0x00
#define UX_DEVICE_CLASS_CDC_ECM_NX_NO_PACKET                                0x01
#define UX_DEVICE_CLASS_CDC_ECM_NX_UNDERFLOW                                0x02
#define UX_DEVICE_CLASS_CDC_ECM_NX_OVERFLOW                                 0x03
#define UX_DEVICE_CLASS_CDC_ECM_NX_NO_MAPPING                               0x04
#define UX_DEVICE_CLASS_CDC_ECM_NX_DELETED                                  0x05
#define UX_DEVICE_CLASS_CDC_ECM_NX_POOL_ERROR                               0x06
#define UX_DEVICE_CLASS_CDC_ECM_NX_PTR_ERROR                                0x07
#define UX_DEVICE_CLASS_CDC_ECM_NX_WAIT_ERROR                               0x08
#define UX_DEVICE_CLASS_CDC_ECM_NX_SIZE_ERROR                               0x09
#define UX_DEVICE_CLASS_CDC_ECM_NX_OPTION_ERROR                             0x0a
#define UX_DEVICE_CLASS_CDC_ECM_NX_DELETE_ERROR                             0x10
#define UX_DEVICE_CLASS_CDC_ECM_NX_CALLER_ERROR                             0x11
#define UX_DEVICE_CLASS_CDC_ECM_NX_INVALID_PACKET                           0x12
#define UX_DEVICE_CLASS_CDC_ECM_NX_INVALID_SOCKET                           0x13
#define UX_DEVICE_CLASS_CDC_ECM_NX_NOT_ENABLED                              0x14
#define UX_DEVICE_CLASS_CDC_ECM_NX_ALREADY_ENABLED                          0x15
#define UX_DEVICE_CLASS_CDC_ECM_NX_ENTRY_NOT_FOUND                          0x16
#define UX_DEVICE_CLASS_CDC_ECM_NX_NO_MORE_ENTRIES                          0x17
#define UX_DEVICE_CLASS_CDC_ECM_NX_ARP_TIMER_ERROR                          0x18
#define UX_DEVICE_CLASS_CDC_ECM_NX_RESERVED_CODE0                           0x19
#define UX_DEVICE_CLASS_CDC_ECM_NX_WAIT_ABORTED                             0x1A
#define UX_DEVICE_CLASS_CDC_ECM_NX_IP_INTERNAL_ERROR                        0x20
#define UX_DEVICE_CLASS_CDC_ECM_NX_IP_ADDRESS_ERROR                         0x21
#define UX_DEVICE_CLASS_CDC_ECM_NX_ALREADY_BOUND                            0x22
#define UX_DEVICE_CLASS_CDC_ECM_NX_PORT_UNAVAILABLE                         0x23
#define UX_DEVICE_CLASS_CDC_ECM_NX_NOT_BOUND                                0x24
#define UX_DEVICE_CLASS_CDC_ECM_NX_RESERVED_CODE1                           0x25
#define UX_DEVICE_CLASS_CDC_ECM_NX_SOCKET_UNBOUND                           0x26
#define UX_DEVICE_CLASS_CDC_ECM_NX_NOT_CREATED                              0x27
#define UX_DEVICE_CLASS_CDC_ECM_NX_SOCKETS_BOUND                            0x28
#define UX_DEVICE_CLASS_CDC_ECM_NX_NO_RESPONSE                              0x29
#define UX_DEVICE_CLASS_CDC_ECM_NX_POOL_DELETED                             0x30
#define UX_DEVICE_CLASS_CDC_ECM_NX_ALREADY_RELEASED                         0x31
#define UX_DEVICE_CLASS_CDC_ECM_NX_RESERVED_CODE2                           0x32
#define UX_DEVICE_CLASS_CDC_ECM_NX_MAX_LISTEN                               0x33
#define UX_DEVICE_CLASS_CDC_ECM_NX_DUPLICATE_LISTEN                         0x34
#define UX_DEVICE_CLASS_CDC_ECM_NX_NOT_CLOSED                               0x35
#define UX_DEVICE_CLASS_CDC_ECM_NX_NOT_LISTEN_STATE                         0x36
#define UX_DEVICE_CLASS_CDC_ECM_NX_IN_PROGRESS                              0x37
#define UX_DEVICE_CLASS_CDC_ECM_NX_NOT_CONNECTED                            0x38
#define UX_DEVICE_CLASS_CDC_ECM_NX_WINDOW_OVERFLOW                          0x39
#define UX_DEVICE_CLASS_CDC_ECM_NX_ALREADY_SUSPENDED                        0x40
#define UX_DEVICE_CLASS_CDC_ECM_NX_DISCONNECT_FAILED                        0x41
#define UX_DEVICE_CLASS_CDC_ECM_NX_STILL_BOUND                              0x42
#define UX_DEVICE_CLASS_CDC_ECM_NX_NOT_SUCCESSFUL                           0x43
#define UX_DEVICE_CLASS_CDC_ECM_NX_UNHANDLED_COMMAND                        0x44
#define UX_DEVICE_CLASS_CDC_ECM_NX_NO_FREE_PORTS                            0x45
#define UX_DEVICE_CLASS_CDC_ECM_NX_INVALID_PORT                             0x46
#define UX_DEVICE_CLASS_CDC_ECM_NX_INVALID_RELISTEN                         0x47
#define UX_DEVICE_CLASS_CDC_ECM_NX_CONNECTION_PENDING                       0x48
#define UX_DEVICE_CLASS_CDC_ECM_NX_TX_QUEUE_DEPTH                           0x49
#define UX_DEVICE_CLASS_CDC_ECM_NX_NOT_IMPLEMENTED                          0x80

/* Define timeout packet allocation value.  */
#ifndef UX_DEVICE_CLASS_CDC_ECM_PACKET_POOL_WAIT
#define UX_DEVICE_CLASS_CDC_ECM_PACKET_POOL_WAIT                            1000
#endif

#ifndef UX_DEVICE_CLASS_CDC_ECM_PACKET_POOL_INST_WAIT
#define UX_DEVICE_CLASS_CDC_ECM_PACKET_POOL_INST_WAIT                       1000
#endif

#define UX_DEVICE_CLASS_CDC_ECM_LINK_CHECK_WAIT                             10

/* Define Slave CDC_ECM Class Calling Parameter structure */

typedef struct UX_SLAVE_CLASS_CDC_ECM_PARAMETER_STRUCT
{
    VOID                    (*ux_slave_class_cdc_ecm_instance_activate)(VOID *);
    VOID                    (*ux_slave_class_cdc_ecm_instance_deactivate)(VOID *);
    ULONG                   ux_slave_class_cdc_ecm_parameter_media;
    ULONG                   ux_slave_class_cdc_ecm_parameter_vendor_id;
    ULONG                   ux_slave_class_cdc_ecm_parameter_driver_version;
    UCHAR                   ux_slave_class_cdc_ecm_parameter_vendor_description[UX_DEVICE_CLASS_CDC_ECM_VENDOR_DESCRIPTION_MAX_LENGTH];
    UCHAR                   ux_slave_class_cdc_ecm_parameter_local_node_id[UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH];
    UCHAR                   ux_slave_class_cdc_ecm_parameter_remote_node_id[UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH];
} UX_SLAVE_CLASS_CDC_ECM_PARAMETER;

/* Define CDC_ECM Class structure.  */

typedef struct UX_SLAVE_CLASS_CDC_ECM_STRUCT
{
    UX_SLAVE_INTERFACE                      *ux_slave_class_cdc_ecm_interface;
    UX_SLAVE_CLASS_CDC_ECM_PARAMETER        ux_slave_class_cdc_ecm_parameter;
    UX_SLAVE_ENDPOINT                       *ux_slave_class_cdc_ecm_bulkin_endpoint;
    UX_SLAVE_ENDPOINT                       *ux_slave_class_cdc_ecm_bulkout_endpoint;
    UX_SLAVE_ENDPOINT                       *ux_slave_class_cdc_ecm_interrupt_endpoint;
    ULONG                                   ux_slave_class_cdc_ecm_state;
    ULONG                                   ux_slave_class_cdc_ecm_current_alternate_setting;
    ULONG                                   ux_slave_class_cdc_ecm_max_transfer_size;
    ULONG                                   ux_slave_class_cdc_ecm_request_id;
    ULONG                                   ux_slave_class_cdc_ecm_statistics_xmit_ok;
    ULONG                                   ux_slave_class_cdc_ecm_statistics_rcv_ok;
    ULONG                                   ux_slave_class_cdc_ecm_statistics_xmit_error;
    ULONG                                   ux_slave_class_cdc_ecm_statistics_rcv_error;
    ULONG                                   ux_slave_class_cdc_ecm_statistics_rcv_no_buffer;
    ULONG                                   ux_slave_class_cdc_ecm_statistics_rcv_error_alignment;
    ULONG                                   ux_slave_class_cdc_ecm_statistics_xmit_one_collision;
    ULONG                                   ux_slave_class_cdc_ecm_statistics_xmit_more_collisions;
    ULONG                                   ux_slave_class_cdc_ecm_ethernet_multicast_filter;
    ULONG                                   ux_slave_class_cdc_ecm_ethernet_power_management_filter;
    ULONG                                   ux_slave_class_cdc_ecm_ethernet_packet_filter;
    UCHAR                                   ux_slave_class_cdc_ecm_local_node_id[UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH];
    UCHAR                                   ux_slave_class_cdc_ecm_remote_node_id[UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH];
    ULONG                                   ux_slave_class_cdc_ecm_nx_ip_address;
    ULONG                                   ux_slave_class_cdc_ecm_nx_ip_network_mask;

#if !defined(UX_DEVICE_STANDALONE)
    NX_IP                                   *ux_slave_class_cdc_ecm_nx_ip;
    NX_INTERFACE                            *ux_slave_class_cdc_ecm_nx_interface;
    NX_PACKET                               *ux_slave_class_cdc_ecm_xmit_queue;
    NX_PACKET                               *ux_slave_class_cdc_ecm_xmit_queue_tail;
    NX_PACKET                               *ux_slave_class_cdc_ecm_receive_queue;
    NX_PACKET_POOL                          *ux_slave_class_cdc_ecm_packet_pool;
#endif

#if !defined(UX_DEVICE_STANDALONE)
    UX_EVENT_FLAGS_GROUP                    ux_slave_class_cdc_ecm_event_flags_group;
    UX_THREAD                               ux_slave_class_cdc_ecm_bulkin_thread;
    UX_THREAD                               ux_slave_class_cdc_ecm_bulkout_thread;
    UX_THREAD                               ux_slave_class_cdc_ecm_interrupt_thread;
    UX_MUTEX                                ux_slave_class_cdc_ecm_mutex;
    UCHAR                                   *ux_slave_class_cdc_ecm_bulkin_thread_stack;
    UCHAR                                   *ux_slave_class_cdc_ecm_bulkout_thread_stack;
    UCHAR                                   *ux_slave_class_cdc_ecm_interrupt_thread_stack;
#endif

    ULONG                                   ux_slave_class_cdc_ecm_link_state;
    VOID                                    *ux_slave_class_cdc_ecm_network_handle;
    
} UX_SLAVE_CLASS_CDC_ECM;


/* Requests - Ethernet Networking Control Model */

#define UX_DEVICE_CLASS_CDC_ECM_SEND_ENCAPSULATED_COMMAND                        0x00        
                                        /* Issues a command in the format of the supported control
                                           protocol. The intent of this mechanism is to support
                                           networking devices (e.g., host-based cable modems)
                                           that require an additional vendor-defined interface for
                                           media specific hardware configuration and
                                           management.  */
#define UX_DEVICE_CLASS_CDC_ECM_GET_ENCAPSULATED_RESPONSE                        0x01        
                                        /* Requests a response in the format of the supported
                                           control protocol.  */


/* Define buffer length for IN/OUT pipes.  */

#define UX_DEVICE_CLASS_CDC_ECM_BUFFER_SIZE                  4096


/* Define Device CDC_ECM Class prototypes.  */

UINT  _ux_device_class_cdc_ecm_activate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_ecm_control_request(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_ecm_deactivate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_ecm_change(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_ecm_entry(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_ecm_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_ecm_uninitialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_ecm_write(VOID *cdc_ecm_class, NX_PACKET *packet);
VOID  _ux_device_class_cdc_ecm_bulkin_thread(ULONG cdc_ecm_class);
VOID  _ux_device_class_cdc_ecm_bulkout_thread(ULONG cdc_ecm_class);
VOID  _ux_device_class_cdc_ecm_interrupt_thread(ULONG cdc_ecm_class);


/* Define Device CDC Class API prototypes.  */

#define ux_device_class_cdc_ecm_entry    _ux_device_class_cdc_ecm_entry
#define ux_device_class_cdc_ecm_read     _ux_device_class_cdc_ecm_read 
#define ux_device_class_cdc_ecm_write    _ux_device_class_cdc_ecm_write

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif /* UX_DEVICE_CLASS_CDC_ECM_H */
