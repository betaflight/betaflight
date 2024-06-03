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
/**   CDC ECM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_cdc_ecm.h                             PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX CDC_ECM class.                                                 */ 
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
/*  02-02-2021     Xiuwen Cai               Modified comment(s), added    */
/*                                            compile option for using    */
/*                                            packet pool from NetX,      */
/*                                            resulting in version 6.1.4  */
/*  08-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            supported NX packet chain,  */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/

#ifndef UX_HOST_CLASS_CDC_ECM_H
#define UX_HOST_CLASS_CDC_ECM_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  

#if !defined(UX_HOST_STANDALONE)
#include "nx_api.h"
#include "ux_network_driver.h"
#else

/* Assume NX things for compiling.  */
#define NX_PACKET                                   VOID*
#define NX_PACKET_POOL                              VOID*
#endif


/* Define CDC_ECM Class constants.  Insert here the PID/VID of vendors and products using the CDC ECM chipset. 
   It is a better mechanism to put this value in the ux_user.h file. */
#ifndef UX_HOST_CLASS_CDC_ECM_VENDOR_ID
#define UX_HOST_CLASS_CDC_ECM_VENDOR_ID                        0x0770
#define UX_HOST_CLASS_CDC_ECM_PRODUCT_ID                       0x1042
#endif

/* Define CDC_ECM Class/subclass/protocol constants. */
#define UX_HOST_CLASS_CDC_CONTROL_CLASS                        0x02
#define UX_HOST_CLASS_CDC_DATA_CLASS                           0x0A
#define UX_HOST_CLASS_CDC_ECM_CONTROL_SUBCLASS                 0x06
#define UX_HOST_CLASS_CDC_ECM_CONTROL_PROTOCOL                 0x00
#define UX_HOST_CLASS_CDC_ECM_CS_INTERFACE                     0x24
#define UX_HOST_CLASS_CDC_ECM_FUNCTIONAL_DESCRIPTOR            0x0F
#define UX_HOST_CLASS_CDC_ECM_MAC_ADDRESS_STRING_LENGTH        32
#define UX_HOST_CLASS_CDC_ECM_MAC_ADDRESS_ASCII_LENGTH         8

#define UX_HOST_CLASS_CDC_ECM_SPEED_SELECTED_100MPBS           0x100 
#define UX_HOST_CLASS_CDC_ECM_SPEED_SELECTED_10MPBS            0x10 
#define UX_HOST_CLASS_CDC_ECM_LINK_STATE_DOWN                  0
#define UX_HOST_CLASS_CDC_ECM_LINK_STATE_UP                    1
#define UX_HOST_CLASS_CDC_ECM_LINK_STATE_PENDING_UP            2
#define UX_HOST_CLASS_CDC_ECM_LINK_STATE_PENDING_DOWN          3
#define UX_HOST_CLASS_CDC_ECM_BASE_IP_ADDRESS                  0xC0A80001
#define UX_HOST_CLASS_CDC_ECM_BASE_IP_MASK                     0xFFFFFF00
#define UX_HOST_CLASS_CDC_ECM_MAX_MTU                          1518
#define UX_HOST_CLASS_CDC_ECM_ETHERNET_IP                      0x0800
#define UX_HOST_CLASS_CDC_ECM_ETHERNET_ARP                     0x0806
#define UX_HOST_CLASS_CDC_ECM_ETHERNET_RARP                    0x8035
#define UX_HOST_CLASS_CDC_ECM_ETHERNET_PACKET_SIZE             1536    
#define UX_HOST_CLASS_CDC_ECM_NX_ALIGN_PADDING                 2
#ifndef UX_HOST_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES
#define UX_HOST_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES                16  
#endif

#define UX_HOST_CLASS_CDC_ECM_NX_PACKET_SIZE                   sizeof(NX_PACKET)

#define UX_HOST_CLASS_CDC_ECM_NX_PAYLOAD_SIZE_ASSERT           UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(UX_HOST_CLASS_CDC_ECM_ETHERNET_PACKET_SIZE, UX_HOST_CLASS_CDC_ECM_NX_ALIGN_PADDING), UX_HOST_CLASS_CDC_ECM_NX_PAYLOAD_SIZE_calc_ovf)
#define UX_HOST_CLASS_CDC_ECM_NX_PAYLOAD_SIZE                  (UX_HOST_CLASS_CDC_ECM_ETHERNET_PACKET_SIZE + UX_HOST_CLASS_CDC_ECM_NX_ALIGN_PADDING)

#define UX_HOST_CLASS_CDC_ECM_NX_BUFF_SIZE_ASSERT                           \
    UX_HOST_CLASS_CDC_ECM_NX_PAYLOAD_SIZE_ASSERT                            \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(                    \
        UX_HOST_CLASS_CDC_ECM_NX_PAYLOAD_SIZE,                              \
        UX_HOST_CLASS_CDC_ECM_NX_PACKET_SIZE),                              \
        UX_HOST_CLASS_CDC_ECM_NX_BUFF_SIZE_calc_ovf)
#define UX_HOST_CLASS_CDC_ECM_NX_BUFF_SIZE                     (UX_HOST_CLASS_CDC_ECM_NX_PAYLOAD_SIZE + UX_HOST_CLASS_CDC_ECM_NX_PACKET_SIZE)

#define UX_HOST_CLASS_CDC_ECM_NX_ETHERNET_POOL_ALLOCSIZE_ASSERT             \
    UX_HOST_CLASS_CDC_ECM_NX_BUFF_SIZE_ASSERT                               \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_MULC_ULONG(                   \
        UX_HOST_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES,                            \
        UX_HOST_CLASS_CDC_ECM_NX_BUFF_SIZE),                                \
        UX_HOST_CLASS_CDC_ECM_NX_ETHERNET_POOL_ALLOCSIZE_calc1_ovf)         \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(                    \
        UX_HOST_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES *                           \
            UX_HOST_CLASS_CDC_ECM_NX_BUFF_SIZE,                             \
        32), UX_HOST_CLASS_CDC_ECM_NX_ETHERNET_POOL_ALLOCSIZE_calc2_ovf)
#define UX_HOST_CLASS_CDC_ECM_NX_ETHERNET_POOL_ALLOCSIZE       (UX_HOST_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES * UX_HOST_CLASS_CDC_ECM_NX_BUFF_SIZE + 32)

#ifdef NX_DISABLE_PACKET_CHAIN
#undef UX_HOST_CLASS_CDC_ECM_PACKET_CHAIN_SUPPORT
#else
#define UX_HOST_CLASS_CDC_ECM_PACKET_CHAIN_SUPPORT
#endif

#define UX_HOST_CLASS_CDC_ECM_ETHERNET_SIZE                    14
                                                                
#define UX_HOST_CLASS_CDC_ECM_DEVICE_INIT_DELAY                (1 * UX_PERIODIC_RATE)
#define UX_HOST_CLASS_CDC_ECM_CLASS_TRANSFER_TIMEOUT           300000
#define UX_HOST_CLASS_CDC_ECM_SETUP_BUFFER_SIZE                16


/* Define NetX errors inside the CDC ECM class.  */
#define UX_HOST_CLASS_CDC_ECM_NX_SUCCESS                       0x00
#define UX_HOST_CLASS_CDC_ECM_NX_NO_PACKET                     0x01
#define UX_HOST_CLASS_CDC_ECM_NX_UNDERFLOW                     0x02
#define UX_HOST_CLASS_CDC_ECM_NX_OVERFLOW                      0x03
#define UX_HOST_CLASS_CDC_ECM_NX_NO_MAPPING                    0x04
#define UX_HOST_CLASS_CDC_ECM_NX_DELETED                       0x05
#define UX_HOST_CLASS_CDC_ECM_NX_POOL_ERROR                    0x06
#define UX_HOST_CLASS_CDC_ECM_NX_PTR_ERROR                     0x07
#define UX_HOST_CLASS_CDC_ECM_NX_WAIT_ERROR                    0x08
#define UX_HOST_CLASS_CDC_ECM_NX_SIZE_ERROR                    0x09
#define UX_HOST_CLASS_CDC_ECM_NX_OPTION_ERROR                  0x0a
#define UX_HOST_CLASS_CDC_ECM_NX_DELETE_ERROR                  0x10
#define UX_HOST_CLASS_CDC_ECM_NX_CALLER_ERROR                  0x11
#define UX_HOST_CLASS_CDC_ECM_NX_INVALID_PACKET                0x12
#define UX_HOST_CLASS_CDC_ECM_NX_INVALID_SOCKET                0x13
#define UX_HOST_CLASS_CDC_ECM_NX_NOT_ENABLED                   0x14
#define UX_HOST_CLASS_CDC_ECM_NX_ALREADY_ENABLED               0x15
#define UX_HOST_CLASS_CDC_ECM_NX_ENTRY_NOT_FOUND               0x16
#define UX_HOST_CLASS_CDC_ECM_NX_NO_MORE_ENTRIES               0x17
#define UX_HOST_CLASS_CDC_ECM_NX_ARP_TIMER_ERROR               0x18
#define UX_HOST_CLASS_CDC_ECM_NX_RESERVED_CODE0                0x19
#define UX_HOST_CLASS_CDC_ECM_NX_WAIT_ABORTED                  0x1A
#define UX_HOST_CLASS_CDC_ECM_NX_IP_INTERNAL_ERROR             0x20
#define UX_HOST_CLASS_CDC_ECM_NX_IP_ADDRESS_ERROR              0x21
#define UX_HOST_CLASS_CDC_ECM_NX_ALREADY_BOUND                 0x22
#define UX_HOST_CLASS_CDC_ECM_NX_PORT_UNAVAILABLE              0x23
#define UX_HOST_CLASS_CDC_ECM_NX_NOT_BOUND                     0x24
#define UX_HOST_CLASS_CDC_ECM_NX_RESERVED_CODE1                0x25
#define UX_HOST_CLASS_CDC_ECM_NX_SOCKET_UNBOUND                0x26
#define UX_HOST_CLASS_CDC_ECM_NX_NOT_CREATED                   0x27
#define UX_HOST_CLASS_CDC_ECM_NX_SOCKETS_BOUND                 0x28
#define UX_HOST_CLASS_CDC_ECM_NX_NO_RESPONSE                   0x29
#define UX_HOST_CLASS_CDC_ECM_NX_POOL_DELETED                  0x30
#define UX_HOST_CLASS_CDC_ECM_NX_ALREADY_RELEASED              0x31
#define UX_HOST_CLASS_CDC_ECM_NX_RESERVED_CODE2                0x32
#define UX_HOST_CLASS_CDC_ECM_NX_MAX_LISTEN                    0x33
#define UX_HOST_CLASS_CDC_ECM_NX_DUPLICATE_LISTEN              0x34
#define UX_HOST_CLASS_CDC_ECM_NX_NOT_CLOSED                    0x35
#define UX_HOST_CLASS_CDC_ECM_NX_NOT_LISTEN_STATE              0x36
#define UX_HOST_CLASS_CDC_ECM_NX_IN_PROGRESS                   0x37
#define UX_HOST_CLASS_CDC_ECM_NX_NOT_CONNECTED                 0x38
#define UX_HOST_CLASS_CDC_ECM_NX_WINDOW_OVERFLOW               0x39
#define UX_HOST_CLASS_CDC_ECM_NX_ALREADY_SUSPENDED             0x40
#define UX_HOST_CLASS_CDC_ECM_NX_DISCONNECT_FAILED             0x41
#define UX_HOST_CLASS_CDC_ECM_NX_STILL_BOUND                   0x42
#define UX_HOST_CLASS_CDC_ECM_NX_NOT_SUCCESSFUL                0x43
#define UX_HOST_CLASS_CDC_ECM_NX_UNHANDLED_COMMAND             0x44
#define UX_HOST_CLASS_CDC_ECM_NX_NO_FREE_PORTS                 0x45
#define UX_HOST_CLASS_CDC_ECM_NX_INVALID_PORT                  0x46
#define UX_HOST_CLASS_CDC_ECM_NX_INVALID_RELISTEN              0x47
#define UX_HOST_CLASS_CDC_ECM_NX_CONNECTION_PENDING            0x48
#define UX_HOST_CLASS_CDC_ECM_NX_TX_QUEUE_DEPTH                0x49
#define UX_HOST_CLASS_CDC_ECM_NX_NOT_IMPLEMENTED               0x80

/* Define  CDC_ECM Class packet equivalences.  */

#define UX_HOST_CLASS_CDC_ECM_PACKET_SIZE                      128
#define UX_HOST_CLASS_CDC_ECM_NODE_ID_LENGTH                   6  

/* Define supported notification types.  */

#define UX_HOST_CLASS_CDC_ECM_NOTIFICATION_NETWORK_CONNECTION   0x00
#define UX_HOST_CLASS_CDC_ECM_NOTIFICATION_RESPONSE_AVAILABLE   0x01
#define UX_HOST_CLASS_CDC_ECM_NOTIFICATION_SERIAL_STATE         0x20
#define UX_HOST_CLASS_CDC_ECM_NOTIFICATION_CALL_STATE_CHANGE    0x28
#define UX_HOST_CLASS_CDC_ECM_NOTIFICATION_LINE_STATE_CHANGE    0x29
#define UX_HOST_CLASS_CDC_ECM_NOTIFICATION_SPEED_CHANGE         0x2A

/* Define notification packet format.  */

#define UX_HOST_CLASS_CDC_ECM_NPF_REQUEST_TYPE                  0x00
#define UX_HOST_CLASS_CDC_ECM_NPF_NOTIFICATION_TYPE             0x01
#define UX_HOST_CLASS_CDC_ECM_NPF_VALUE                         0x02
#define UX_HOST_CLASS_CDC_ECM_NPF_INDEX                         0x04
#define UX_HOST_CLASS_CDC_ECM_NPF_LENGTH                        0x06

/* Define supported notification values.  */

#define UX_HOST_CLASS_CDC_ECM_NOTIFICATION_NETWORK_LINK_DOWN    0x00
#define UX_HOST_CLASS_CDC_ECM_NOTIFICATION_NETWORK_LINK_UP      0x01

/* Define packet allocation timeout in milliseconds.  */

#ifndef UX_HOST_CLASS_CDC_ECM_PACKET_POOL_WAIT
#define UX_HOST_CLASS_CDC_ECM_PACKET_POOL_WAIT                  1000
#endif

/* Define packet pool waiting time in milliseconds.  */

#ifndef UX_HOST_CLASS_CDC_ECM_PACKET_POOL_INSTANCE_WAIT
#define UX_HOST_CLASS_CDC_ECM_PACKET_POOL_INSTANCE_WAIT         100
#endif

/* Define  CDC_ECM Class instance structure.  */

typedef struct UX_HOST_CLASS_CDC_ECM_STRUCT
{
    struct UX_HOST_CLASS_CDC_ECM_STRUCT  
                    *ux_host_class_cdc_ecm_next_instance;
    UX_HOST_CLASS   *ux_host_class_cdc_ecm_class;
    UX_DEVICE       *ux_host_class_cdc_ecm_device;
    UX_ENDPOINT     *ux_host_class_cdc_ecm_bulk_in_endpoint;
    UX_ENDPOINT     *ux_host_class_cdc_ecm_bulk_out_endpoint;
    UX_ENDPOINT     *ux_host_class_cdc_ecm_interrupt_endpoint;
    UX_INTERFACE    *ux_host_class_cdc_ecm_interface_data;
    UX_INTERFACE    *ux_host_class_cdc_ecm_interface_control;
    UCHAR           ux_host_class_cdc_ecm_bulk_in_transfer_check_and_arm_in_process;
    UCHAR           ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish;
    UX_SEMAPHORE    ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish_semaphore;
    UCHAR           ux_host_class_cdc_ecm_bulk_out_transfer_check_and_arm_in_process;
    UCHAR           ux_host_class_cdc_ecm_bulk_out_transfer_waiting_for_check_and_arm_to_finish;
    UX_SEMAPHORE    ux_host_class_cdc_ecm_bulk_out_transfer_waiting_for_check_and_arm_to_finish_semaphore;
    UINT            ux_host_class_cdc_ecm_instance_status;
    UINT            ux_host_class_cdc_ecm_state;
    UX_SEMAPHORE    ux_host_class_cdc_ecm_interrupt_notification_semaphore;
    UX_THREAD       ux_host_class_cdc_ecm_thread;
    UCHAR           *ux_host_class_cdc_ecm_thread_stack;
    ULONG           ux_host_class_cdc_ecm_notification_count;
    ULONG           ux_host_class_cdc_ecm_primary_phy_id;
    ULONG           ux_host_class_cdc_ecm_primary_phy_type;
    ULONG           ux_host_class_cdc_ecm_secondary_phy_id;
    ULONG           ux_host_class_cdc_ecm_secondary_phy_type;
    ULONG           ux_host_class_cdc_ecm_model_revision_number;
    ULONG           ux_host_class_cdc_ecm_vendor_model_number;
    ULONG           ux_host_class_cdc_ecm_speed_selected;
    ULONG           ux_host_class_cdc_ecm_device_state;
    ULONG           ux_host_class_cdc_ecm_link_state;
    NX_PACKET       *ux_host_class_cdc_ecm_xmit_queue_head;
    NX_PACKET       *ux_host_class_cdc_ecm_xmit_queue_tail;
    NX_PACKET_POOL  *ux_host_class_cdc_ecm_packet_pool;
#ifdef UX_HOST_CLASS_CDC_ECM_PACKET_CHAIN_SUPPORT
    UCHAR           *ux_host_class_cdc_ecm_xmit_buffer;
    UCHAR           *ux_host_class_cdc_ecm_receive_buffer;
#endif

    UCHAR           ux_host_class_cdc_ecm_node_id[UX_HOST_CLASS_CDC_ECM_NODE_ID_LENGTH];
    VOID            (*ux_host_class_cdc_ecm_device_status_change_callback)(struct UX_HOST_CLASS_CDC_ECM_STRUCT *cdc_ecm, 
                                                                ULONG  device_state);
    VOID            *ux_host_class_cdc_ecm_network_handle;
    
} UX_HOST_CLASS_CDC_ECM;


/* Define ECM Interface Functional descriptor.  */

#define UX_HOST_CLASS_CDC_ECM_INTERFACE_DESCRIPTOR_ENTRIES               8
#define UX_HOST_CLASS_CDC_ECM_INTERFACE_DESCRIPTOR_LENGTH                13

typedef struct UX_HOST_CLASS_ECM_INTERFACE_DESCRIPTOR_STRUCT
{
    ULONG           bFunctionLength;
    ULONG           bDescriptorType;
    ULONG           bDescriptorSubtype;
    ULONG           iMACAddress;
    ULONG           bmEthernetStatistics;
    ULONG           wMaxSegmentSize;
    ULONG           wNumberMCFilters;
    ULONG           bNumberPowerFilters;
} UX_HOST_CLASS_ECM_INTERFACE_DESCRIPTOR;

/* Define CDC ECM Class function prototypes.  */

UINT  _ux_host_class_cdc_ecm_activate(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_cdc_ecm_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_cdc_ecm_endpoints_get(UX_HOST_CLASS_CDC_ECM *cdc_ecm);
UINT  _ux_host_class_cdc_ecm_entry(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_cdc_ecm_write(VOID *cdc_ecm_class, NX_PACKET *packet);
VOID  _ux_host_class_cdc_ecm_interrupt_notification(UX_TRANSFER *transfer_request);
VOID  _ux_host_class_cdc_ecm_thread(ULONG parameter);
VOID  _ux_host_class_cdc_ecm_transmission_callback(UX_TRANSFER *transfer_request);
VOID  _ux_host_class_cdc_ecm_transmit_queue_clean(UX_HOST_CLASS_CDC_ECM *cdc_ecm_control);
UINT  _ux_host_class_cdc_ecm_mac_address_get(UX_HOST_CLASS_CDC_ECM *cdc_ecm);
                                    
/* Define CDC ECM Class API prototypes.  */

#define ux_host_class_cdc_ecm_entry        _ux_host_class_cdc_ecm_entry
#define ux_host_class_cdc_ecm_write        _ux_host_class_cdc_ecm_write

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif
