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
/**   USBX Network Driver for NETX 5.3 and above                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_network_driver.h                                 PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX Network driver.                                                */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  07-29-2022     Yajun Xia                Modified comment(s),          */
/*                                            fixed ipv6 support issue,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_NETWORK_DRIVER_H
#define UX_NETWORK_DRIVER_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  

#include "tx_api.h"
#include "nx_api.h"
#define USB_NETWORK_DEVICE_MAX_INSTANCES    8
#define USB_NETWORK_DRIVER_SUCCESS          UX_SUCCESS
#define USB_NETWORK_DRIVER_FAILURE          UX_ERROR

#define USB_NETWORK_DEVICE_MAC_HEADER_SIZE  14
#define NX_ETHERNET_SIZE                    14
#define NX_ETHERNET_ARP                     0x0806
#define NX_ETHERNET_RARP                    0x0835
#define NX_ETHERNET_IP                      0x0800
#define NX_ETHERNET_IPV6                    0x86DD
#define NX_ETHERNET_MTU                     1514

typedef struct USB_NETWORK_DEVICE_STRUCT
{

    /* ip_instance is populated by NetX, as part of the interface attachment. */
    NX_IP           *ux_network_device_ip_instance;

    /* interface_ptr is populated by NetX, as part of the interface attachment. */
    NX_INTERFACE    *ux_network_device_interface_ptr;

    /* Define synchronization objects for deactivation. Note that these are only
       used if the activation/deactivation functions are not called under interrupt.  */
    UCHAR           ux_network_device_activated_by_thread;
    TX_MUTEX        ux_network_device_deactivate_mutex;
    TX_SEMAPHORE    ux_network_device_deactivate_semaphore;
    UCHAR           ux_network_device_deactivate_thread_waiting;
    UINT            ux_network_device_num_threads_inside;

    /* usb_instance is populated by USB instance activation. */
    VOID            *ux_network_device_usb_instance_ptr;

    /* The write_function is populated by USB instance activation. */
    UINT            (*ux_network_device_write_function)(VOID *ux_instance, NX_PACKET *packet_ptr);
    
    USHORT          ux_network_device_usb_link_up;
    USHORT          ux_network_device_link_status;
    
    ULONG           ux_network_physical_address_msw;
    ULONG           ux_network_physical_address_lsw;


} USB_NETWORK_DEVICE_TYPE;


UINT  _ux_network_driver_init(VOID);

UINT  _ux_network_driver_activate(VOID *ux_instance, UINT(*ux_network_device_write_function)(VOID *, NX_PACKET *),
                                VOID **ux_network_handle, ULONG physical_address_msw, ULONG physical_address_lsw);

UINT  _ux_network_driver_deactivate(VOID *ux_instance, VOID *ux_network_handle);


VOID  _ux_network_driver_entry(NX_IP_DRIVER *nx_ip_driver);

VOID  _ux_network_driver_link_up(VOID *ux_network_handle);
VOID  _ux_network_driver_link_down(VOID *ux_network_handle);

VOID  _ux_network_driver_packet_received(VOID *ux_network_handle, NX_PACKET *packet_ptr);
/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif
