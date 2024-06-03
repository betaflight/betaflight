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
/**   USBX Network Driver for NETX 5.3 and above.                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#include "ux_api.h"

#if !defined(UX_STANDALONE)
#ifndef UX_NETWORK_DRIVER_ENABLE
#define UX_NETWORK_DRIVER_ENABLE
#endif
#else
/* Standalone mode not supported.  */
#endif

#if defined(UX_NETWORK_DRIVER_ENABLE)
#include "tx_api.h"
#include "tx_thread.h"
#include "nx_api.h"

#include "ux_network_driver.h"

static UINT usb_network_driver_initialized;

static USB_NETWORK_DEVICE_TYPE usb_network_devices[USB_NETWORK_DEVICE_MAX_INSTANCES];

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_network_init                                    PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called by the application to initialize the        */ 
/*    USBX portion of the network driver.                                 */ 
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT _ux_network_driver_init(VOID)
{

UINT  status = NX_SUCCESS;

    /* Check if driver is already initialized.  */
    if (usb_network_driver_initialized == 0)
    {

        /* Driver is not initialized yet.  */
        usb_network_driver_initialized = 1;

        /* Reset the network device memory array.  */
        _ux_utility_memory_set(&usb_network_devices[0], 0, sizeof(usb_network_devices)); /* Use case of memset is verified. */
    }
    
    return(status);
}

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_network_driver_activate                         PORTABLE C      */ 
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/* The USB network driver activate function is called as the USB instance */
/* is created. This API takes a pointer to the instance, and returns a    */
/* ux_network_handle back to instance. Every time the instance receives   */
/* a network packet, it should call ux_network_driver_packet_received with*/
/* ux_network_handle.                                                     */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*  ux_instance                      Instance of the USBX network class   */ 
/*  ux_network_device_write_function Address of the function to write a   */ 
/*                                   packet when sent by the application  */ 
/*  ux_network_handle                Address where to store the network   */ 
/*                                   handle                               */ 
/*                                                                        */ 
/*  physical_address_msw             Most significant word of network ad  */ 
/*                                                                        */ 
/*  physical_address_lsw             Least significant word of network ad */ 
/*                                                                        */ 
/*                                                                        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*  Result                                                                */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
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
/*                                            resulting in version 6.1.8  */
/*                                                                        */
/**************************************************************************/

UINT _ux_network_driver_activate(VOID *ux_instance, UINT(*ux_network_device_write_function)(VOID *, NX_PACKET *),
                                VOID **ux_network_handle, ULONG physical_address_msw, ULONG physical_address_lsw)
{

UX_INTERRUPT_SAVE_AREA 

UINT    i;

    /* Critical section.  */
    UX_DISABLE
    
    /* Find an available entry in the usb_network_devices table. */
    for (i = 0; i < USB_NETWORK_DEVICE_MAX_INSTANCES; i++)
    {

        /* If the ptr to instance is NULL, we have a free entry.  */
        if (usb_network_devices[i].ux_network_device_usb_instance_ptr == NX_NULL)
        {
            
            /* Add the instance of the USBX class driver to the network device.  */
            usb_network_devices[i].ux_network_device_usb_instance_ptr = ux_instance;

            break;
        }
    }

    /* Unprotect the critical section.  */
    UX_RESTORE

    /* Did we reach the max number of instance ? */
    if (i == USB_NETWORK_DEVICE_MAX_INSTANCES)
    {

        /* Report error to application.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

        /* Return error.  */
        return(USB_NETWORK_DRIVER_FAILURE);
    }
    
    /* Store the write function.  */
    usb_network_devices[i].ux_network_device_write_function = ux_network_device_write_function;
    
    /* Store the physical address of the network interface.  */
    usb_network_devices[i].ux_network_physical_address_msw = physical_address_msw;
    usb_network_devices[i].ux_network_physical_address_lsw = physical_address_lsw;

    /* Are we not under interrupt?  */
    if (TX_THREAD_GET_SYSTEM_STATE() == 0)
    {

        /* Note that we were activated by a thread.  */
        usb_network_devices[i].ux_network_device_activated_by_thread =  UX_TRUE;

        /* Create deactivation sync objects. */
        _ux_utility_mutex_create(&usb_network_devices[i].ux_network_device_deactivate_mutex, "usb network device mutex");
        _ux_utility_semaphore_create(&usb_network_devices[i].ux_network_device_deactivate_semaphore, "usb network device semaphore", 0);
    }

    /* Is there an interface at the NETX level ?  */
    if (usb_network_devices[i].ux_network_device_interface_ptr)
    {
    
        /* Store the physical address at the NETX level.  */
        usb_network_devices[i].ux_network_device_interface_ptr -> nx_interface_physical_address_msw = physical_address_msw;
        usb_network_devices[i].ux_network_device_interface_ptr -> nx_interface_physical_address_lsw = physical_address_lsw;

        /* Is the link UP ?  */
        if (usb_network_devices[i].ux_network_device_interface_ptr -> nx_interface_link_up == NX_TRUE)

            /* Yes, store its state.  */    
            usb_network_devices[i].ux_network_device_link_status = NX_TRUE;

    }
    else

        /* Link not yet up.  */
        usb_network_devices[i].ux_network_device_link_status = NX_FALSE;    


    /* Is there a network handle associated ?  */
    if (ux_network_handle)
    
        /* Yes, the application wants to know its address.  */
        *ux_network_handle = (VOID*)&usb_network_devices[i];
    
    /* The operation was successful.  */
    return(USB_NETWORK_DRIVER_SUCCESS);
}



/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_network_driver_deactivate                       PORTABLE C      */ 
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/* The USB network driver activate function is called as the USB instance */
/* is created. This API takes a pointer to the instance, and returns a    */
/* ux_network_handle back to instance. Every time the instance receives   */
/* a network packet, it should call ux_network_driver_packet_received with*/
/* ux_network_handle.                                                     */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*  ux_instance                      Instance of the USBX network class   */ 
/*  ux_network_device_write_function Address of the function to write a   */ 
/*                                   packet when sent by the application  */ 
/*  ux_network_handle                Address where to store the network   */ 
/*                                   handle                               */ 
/*                                                                        */ 
/*  physical_address_msw             Most significant word of network ad  */ 
/*                                                                        */ 
/*  physical_address_lsw             Least significant word of network ad */ 
/*                                                                        */ 
/*                                                                        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*  Result                                                                */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
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
/*                                            resulting in version 6.1.8  */
/*                                                                        */
/**************************************************************************/
UINT _ux_network_driver_deactivate(VOID *ux_instance, VOID *ux_network_handle)
{

UX_INTERRUPT_SAVE_AREA

USB_NETWORK_DEVICE_TYPE *usb_network_device;

    UX_PARAMETER_NOT_USED(ux_instance);

    /* Check if the handle exists.  */
    if(ux_network_handle == NX_NULL)
        return(USB_NETWORK_DRIVER_FAILURE);

    /* Cast the network handle properly.  */
    usb_network_device = (USB_NETWORK_DEVICE_TYPE*) ux_network_handle;
    
    /* Critical section.  */
    UX_DISABLE
    
    /* The link is down.  */
    _ux_network_driver_link_down(ux_network_handle);

    /* Unprotect the critical section.  */
    UX_RESTORE

    /* Are the sync objects valid?  */
    if (usb_network_device -> ux_network_device_activated_by_thread)
    {

        /* Get mutex.  */
        _ux_utility_mutex_on(&usb_network_device -> ux_network_device_deactivate_mutex);

        /* Any threads in instance?   */
        if (usb_network_device -> ux_network_device_num_threads_inside != 0)
        {

            /* Signal that we're waiting.  */
            usb_network_device -> ux_network_device_deactivate_thread_waiting =  UX_TRUE;

            /* Release mutex.  */
            _ux_utility_mutex_off(&usb_network_device -> ux_network_device_deactivate_mutex);

            /* Wait for last thread inside to resume us.  */
            _ux_utility_semaphore_get(&usb_network_device -> ux_network_device_deactivate_semaphore, UX_WAIT_FOREVER);

            /* We're done waiting.  */
            usb_network_device -> ux_network_device_deactivate_thread_waiting =  UX_FALSE;
        }
        else
        {

            /* Release mutex.  */
            _ux_utility_mutex_off(&usb_network_device -> ux_network_device_deactivate_mutex);
        }

        /* Delete sync objects.  */
        _ux_utility_mutex_delete(&usb_network_device -> ux_network_device_deactivate_mutex);
        _ux_utility_semaphore_delete(&usb_network_device -> ux_network_device_deactivate_semaphore);

        /* Reset for next activation.  */
        usb_network_device -> ux_network_device_activated_by_thread =  UX_FALSE;
    }

    /* All threads are outside of the instance, and can't re-enter because the
       link flag has been set to down. Now we can clean up.  */
    
    /* Reset the instance pointer.  */
    usb_network_device -> ux_network_device_usb_instance_ptr = NX_NULL;
                                               
    /* And the write function ptr.  */
    usb_network_device -> ux_network_device_write_function = NX_NULL;

    /* The operation was successful.  */
    return(USB_NETWORK_DRIVER_SUCCESS);

}

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_network_driver_entry                            PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/* This function is called by NETX. This is the dispatcher to all the     */
/* NETX function to the driver layer.                                     */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*  nx_ip_driver                     Pointer to the NX_IP driver instance */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*  Result                                                                */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    NETX                                                                */ 
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
/*  07-29-2022     Yajun Xia                Modified comment(s),          */
/*                                            fixed ipv6 support issue,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/

VOID _ux_network_driver_entry(NX_IP_DRIVER *nx_ip_driver)
{
    
UX_INTERRUPT_SAVE_AREA 
NX_IP                           *nx_ip;
NX_PACKET                       *packet_ptr;
ULONG                           *ethernet_frame_ptr;
NX_INTERFACE                    *nx_interface_ptr;
USB_NETWORK_DEVICE_TYPE         *usb_network_device_ptr;
UINT                            i;

    /* Get the pointer to the NX_IP instance.  */
    nx_ip = nx_ip_driver -> nx_ip_driver_ptr;    

    /* Set the default status return. */
    nx_ip_driver  -> nx_ip_driver_status =  NX_NOT_SUCCESSFUL;

    /* Get the pointer to the interface in local variable.  */
    nx_interface_ptr = nx_ip_driver -> nx_ip_driver_interface;

    /* Is this the ATTACH command?  */
    if (nx_ip_driver -> nx_ip_driver_command == NX_LINK_INTERFACE_ATTACH)
    {

        /* Critical section.  */
        UX_DISABLE

        /* Find an available entry in the usb_network_devices table. */
        for (i = 0; i < USB_NETWORK_DEVICE_MAX_INSTANCES; i++)
        {

            /* If the interface pointer is NULL, it is free.  */
            if (usb_network_devices[i].ux_network_device_interface_ptr == NX_NULL)
                break;
            
        }
        
        /* Check if we have ran out of instances.  */
        if (i == USB_NETWORK_DEVICE_MAX_INSTANCES)

            /* No more instances, set the error code.  */
            nx_ip_driver -> nx_ip_driver_status = NX_NO_MORE_ENTRIES;

        else
        {

            /* Save the IP address in the network instance.  */
            usb_network_devices[i].ux_network_device_ip_instance = nx_ip;

            /* Save pointer to interface.  */
            usb_network_devices[i].ux_network_device_interface_ptr = nx_interface_ptr;

            /* Set the USB class instance in the additional link. This will be used by the USB class driver.  */
            nx_interface_ptr -> nx_interface_additional_link_info = (VOID *) &usb_network_devices[i];

            /* The operation was successful.  */
            nx_ip_driver -> nx_ip_driver_status = NX_SUCCESS;
        }

        /* Unprotect the critical section.  */
        UX_RESTORE
    }
    else
    {

        /* Get the usb instance.  */
        usb_network_device_ptr = (USB_NETWORK_DEVICE_TYPE *) nx_interface_ptr -> nx_interface_additional_link_info;

        /* Identify command.  */
        switch(nx_ip_driver  -> nx_ip_driver_command)
        {

        case NX_LINK_INITIALIZE:

            /* INIT command, set the interface parameters.  */
            nx_interface_ptr -> nx_interface_valid = NX_TRUE;
            nx_interface_ptr -> nx_interface_address_mapping_needed = NX_TRUE;
            nx_interface_ptr -> nx_interface_ip_mtu_size =  NX_ETHERNET_MTU - NX_ETHERNET_SIZE;

            /* Set the link to down for now.  */
            nx_interface_ptr -> nx_interface_link_up = NX_FALSE;

            /* Check if instance exists.  */
            if (usb_network_device_ptr -> ux_network_device_usb_instance_ptr)
            {
        
                /* Store the physical address in the nx interface.  */
                nx_interface_ptr -> nx_interface_physical_address_msw = usb_network_device_ptr -> ux_network_physical_address_msw;
                nx_interface_ptr -> nx_interface_physical_address_lsw = usb_network_device_ptr -> ux_network_physical_address_lsw;
                
            }
            else 
            {            

                /* Reset the physical address.  */
                nx_interface_ptr -> nx_interface_physical_address_msw = 0;
                nx_interface_ptr -> nx_interface_physical_address_lsw = 0;
            }

            /* Operation is successful.  */
            nx_ip_driver  -> nx_ip_driver_status =  NX_SUCCESS;        
            break;


        

        case NX_LINK_ENABLE:

            /* Set the link state to UP.  */
            nx_interface_ptr -> nx_interface_link_up = NX_TRUE;
            
            /* Reflect link state in network device.  */
            if (usb_network_device_ptr -> ux_network_device_usb_link_up == NX_TRUE)
                usb_network_device_ptr -> ux_network_device_link_status = NX_TRUE;
            else
                usb_network_device_ptr -> ux_network_device_link_status = NX_FALSE;
            
            nx_ip_driver -> nx_ip_driver_status = NX_SUCCESS;
            break;
            
        
        case NX_LINK_DISABLE:
        
            /* Set the link down.  */
            nx_interface_ptr -> nx_interface_link_up = NX_FALSE;
            usb_network_device_ptr -> ux_network_device_link_status = NX_FALSE;
            nx_ip_driver  -> nx_ip_driver_status =  NX_SUCCESS;        
            break;
            

        case NX_LINK_PACKET_SEND:
        case NX_LINK_PACKET_BROADCAST:
        case NX_LINK_ARP_SEND:
        case NX_LINK_ARP_RESPONSE_SEND:
        case NX_LINK_RARP_SEND:

            /* Place the ethernet frame at the front of the packet.  */
            packet_ptr =  nx_ip_driver -> nx_ip_driver_packet;

            /* Are the sync objects valid?  */
            if (usb_network_device_ptr -> ux_network_device_activated_by_thread == UX_TRUE)

                /* Get mutex for checking link state and setting our state.  */
                _ux_utility_mutex_on(&usb_network_device_ptr -> ux_network_device_deactivate_mutex);

            /* Do not send a packet if the link is not enabled. */
            if (usb_network_device_ptr -> ux_network_device_link_status == NX_TRUE)
            {

                /* Increment number of threads inside this instance.  */
                usb_network_device_ptr -> ux_network_device_num_threads_inside++;

                /* Are the sync objects valid?  */
                if (usb_network_device_ptr -> ux_network_device_activated_by_thread == UX_TRUE)

                    /* Release mutex.  */
                    _ux_utility_mutex_off(&usb_network_device_ptr -> ux_network_device_deactivate_mutex);

                /* Adjust the prepend pointer.  */
                packet_ptr -> nx_packet_prepend_ptr =  packet_ptr -> nx_packet_prepend_ptr - NX_ETHERNET_SIZE;
                
                /* Adjust the packet length.  */
                packet_ptr -> nx_packet_length = packet_ptr -> nx_packet_length + NX_ETHERNET_SIZE;
                
                /* Setup the ethernet frame pointer to build the ethernet frame.  Back up another 2 bytes to get 32-bit word alignment. */
                ethernet_frame_ptr =  (ULONG*)(packet_ptr -> nx_packet_prepend_ptr - 2);
                
                /* Build the ethernet frame.  */
                *ethernet_frame_ptr       = nx_ip_driver -> nx_ip_driver_physical_address_msw;
                *(ethernet_frame_ptr + 1) = nx_ip_driver -> nx_ip_driver_physical_address_lsw;
                *(ethernet_frame_ptr + 2) = (nx_interface_ptr -> nx_interface_physical_address_msw << 16) |
                    (nx_interface_ptr -> nx_interface_physical_address_lsw >> 16);
                *(ethernet_frame_ptr + 3) = (nx_interface_ptr -> nx_interface_physical_address_lsw << 16);

                if ((nx_ip_driver -> nx_ip_driver_command == NX_LINK_ARP_SEND)||
                   (nx_ip_driver -> nx_ip_driver_command == NX_LINK_ARP_RESPONSE_SEND))
                {
                    *(ethernet_frame_ptr+3) |= NX_ETHERNET_ARP;
                }
                else if (nx_ip_driver -> nx_ip_driver_command == NX_LINK_RARP_SEND)
                {
                    *(ethernet_frame_ptr+3) |= NX_ETHERNET_RARP;
                }
                else
                {
                    if (packet_ptr -> nx_packet_ip_version == NX_IP_VERSION_V4)
                        *(ethernet_frame_ptr+3) |= NX_ETHERNET_IP;
#ifdef FEATURE_NX_IPV6
                    else if (packet_ptr -> nx_packet_ip_version == NX_IP_VERSION_V6)
                        *(ethernet_frame_ptr+3) |= NX_ETHERNET_IPV6;
#endif /* FEATURE_NX_IPV6 */
                    else 
                    {
                        /* Unknown IP version */
                        /* free the packet that we will not send */
                        nx_packet_transmit_release(packet_ptr);
                        nx_ip_driver  -> nx_ip_driver_status =  NX_NOT_SUCCESSFUL;        
                        break;
                    }
                }

                /* Endian swapping if NX_LITTLE_ENDIAN is defined.  */
                NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr));
                NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr+1));
                NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr+2));
                NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr+3));
                
                /* Write the packet or queue it.  */
                nx_ip_driver -> nx_ip_driver_status = 
                    usb_network_device_ptr -> ux_network_device_write_function(usb_network_device_ptr -> ux_network_device_usb_instance_ptr,
                                                                              packet_ptr);

                /* Are the sync objects valid?  */
                if (usb_network_device_ptr -> ux_network_device_activated_by_thread == UX_TRUE)
                {

                    /* Get mutex.  */
                    _ux_utility_mutex_on(&usb_network_device_ptr -> ux_network_device_deactivate_mutex);

                    /* Decrement number of threads in instance.  */
                    usb_network_device_ptr -> ux_network_device_num_threads_inside--;

                    /* No more threads in the instance?  */
                    if (usb_network_device_ptr -> ux_network_device_num_threads_inside == 0)
                    {

                        /* Release mutex.  */
                        _ux_utility_mutex_off(&usb_network_device_ptr -> ux_network_device_deactivate_mutex);

                        /* Anyone waiting for us to exit?  */
                        if (usb_network_device_ptr -> ux_network_device_deactivate_thread_waiting == UX_TRUE)

                            /* Resume deactivate thread waiting.  */
                            _ux_utility_semaphore_put(&usb_network_device_ptr -> ux_network_device_deactivate_semaphore);
                    }
                    else
                    {

                        /* Release mutex.  */
                        _ux_utility_mutex_off(&usb_network_device_ptr -> ux_network_device_deactivate_mutex);
                    }
                }
            }
            else
            {

                /* Are the sync objects valid?  */
                if (usb_network_device_ptr -> ux_network_device_activated_by_thread == UX_TRUE)

                    /* Release mutex.  */
                    _ux_utility_mutex_off(&usb_network_device_ptr -> ux_network_device_deactivate_mutex);

                /* Link down, throw away packet.  */
                nx_packet_transmit_release(packet_ptr);
                nx_ip_driver  -> nx_ip_driver_status =  NX_SUCCESS;

                /* Report error to application.  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CLASS_CDC_ECM_LINK_STATE_DOWN_ERROR);
            }

            break;
            
        case NX_LINK_UNINITIALIZE:

            usb_network_driver_initialized =  0;

            break;
            
        case NX_LINK_MULTICAST_JOIN:
        case NX_LINK_MULTICAST_LEAVE:
        case NX_LINK_GET_STATUS:
        case NX_LINK_GET_ERROR_COUNT:
        case NX_LINK_GET_RX_COUNT:
        case NX_LINK_GET_TX_COUNT:
        case NX_LINK_GET_ALLOC_ERRORS:
        case NX_LINK_GET_SPEED:
        case NX_LINK_GET_DUPLEX_TYPE:
        case NX_LINK_USER_COMMAND :
        default:
        
            /* Invalid driver request.  */
            nx_ip_driver -> nx_ip_driver_status =  NX_UNHANDLED_COMMAND;

            break;
        }
    }

    /* We are done here.  */
    return;
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_network_driver_packet_received                  PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/* This function is called by USBX when a packet has been receiver over   */
/* the USB.                                                               */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*  ux_network_handle                Handle of the USB network instance   */ 
/*  packet_ptr                       Pointer to packet received           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*  Result                                                                */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX                                                                */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Yajun Xia                Modified comment(s),          */
/*                                            fixed ipv6 support issue,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/

VOID  _ux_network_driver_packet_received(VOID *ux_network_handle, NX_PACKET *packet_ptr)
{

USB_NETWORK_DEVICE_TYPE *usb_network_device_ptr = (USB_NETWORK_DEVICE_TYPE*) ux_network_handle;

ULONG           packet_type;
NX_IP           *nx_ip;

    /* Check the state of the Link.  */
    if (usb_network_device_ptr -> ux_network_device_link_status != NX_TRUE)
    {

        /* Link down, throw away packet.  */
        nx_packet_release(packet_ptr);
        return;

    }

    /* Pickup the packet header to determine where the packet needs to be
       sent.  */
    packet_type =  _ux_utility_short_get_big_endian(packet_ptr -> nx_packet_prepend_ptr + 12);
    
    /* Storing in into the packet the interface.  */
    packet_ptr -> nx_packet_ip_interface =  usb_network_device_ptr -> ux_network_device_interface_ptr;

    /* Get the IP instance.  */
    nx_ip = usb_network_device_ptr -> ux_network_device_ip_instance;
    
    /* Route the incoming packet according to its ethernet type.  */
    switch (packet_type)
    {
        case NX_ETHERNET_IP     :
#ifdef FEATURE_NX_IPV6
        /* fallthrough */
        case NX_ETHERNET_IPV6   :
#endif /* FEATURE_NX_IPV6 */

            /* Note:  The length reported by some Ethernet hardware includes 
               bytes after the packet as well as the Ethernet header.  In some 
               cases, the actual packet length after the Ethernet header should 
               be derived from the length in the IP header (lower 16 bits of
               the first 32-bit word).  */
    
            /* Clean off the Ethernet header.  */
            packet_ptr -> nx_packet_prepend_ptr =  packet_ptr -> nx_packet_prepend_ptr + NX_ETHERNET_SIZE;
     
            /* Adjust the packet length.  */
            packet_ptr -> nx_packet_length =  packet_ptr -> nx_packet_length - NX_ETHERNET_SIZE;
    
            /* Route to the ip receive function.  */
            _nx_ip_packet_deferred_receive(nx_ip, packet_ptr);
    
            break;
    
        case NX_ETHERNET_ARP    :
    
            /* Clean off the Ethernet header.  */
            packet_ptr -> nx_packet_prepend_ptr =  packet_ptr -> nx_packet_prepend_ptr + NX_ETHERNET_SIZE;
    
            /* Adjust the packet length.  */
            packet_ptr -> nx_packet_length =   packet_ptr -> nx_packet_length - NX_ETHERNET_SIZE;
    
            /* Route to the ARP receive function.  */
           _nx_arp_packet_deferred_receive(nx_ip, packet_ptr);
        
            break;
    
        case NX_ETHERNET_RARP   :
    
            /* Clean off the Ethernet header.  */
            packet_ptr -> nx_packet_prepend_ptr =  
                packet_ptr -> nx_packet_prepend_ptr + NX_ETHERNET_SIZE;
    
            /* Adjust the packet length.  */
            packet_ptr -> nx_packet_length =  
                packet_ptr -> nx_packet_length - NX_ETHERNET_SIZE;
    
            /* Route to the RARP receive function.  */
            _nx_rarp_packet_deferred_receive(nx_ip, packet_ptr);
            
            break;
    
    
        default :
    
            /* Invalid ethernet header... release the packet.  */
            nx_packet_release(packet_ptr);

    }
}

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_network_driver_link_up                          PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/* This function is called by USBX when the line link is up               */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*  ux_network_handle                Handle of the USB network instance   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*  Result                                                                */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX                                                                */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/

VOID _ux_network_driver_link_up(VOID *ux_network_handle)
{

USB_NETWORK_DEVICE_TYPE *usb_network_device_ptr = (USB_NETWORK_DEVICE_TYPE*)ux_network_handle;
 
    /* The USB side of the link is UP.  */        
    usb_network_device_ptr -> ux_network_device_usb_link_up = NX_TRUE;

    /* Check if there is an existing interface.  */
    if (usb_network_device_ptr -> ux_network_device_interface_ptr)
    {
        
        /* Set link status.  */
        if (usb_network_device_ptr -> ux_network_device_interface_ptr -> nx_interface_link_up)
            usb_network_device_ptr -> ux_network_device_link_status = NX_TRUE;
        else
            usb_network_device_ptr -> ux_network_device_link_status = NX_FALSE;
    }

}

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_network_driver_link_down                        PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/* This function is called by USBX when the link is down.                 */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*  ux_network_handle                Handle of the USB network instance   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*  Result                                                                */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX                                                                */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
VOID _ux_network_driver_link_down(VOID *ux_network_handle)
{

USB_NETWORK_DEVICE_TYPE *usb_network_device_ptr = (USB_NETWORK_DEVICE_TYPE*)ux_network_handle;
 
    /* Set the USB link status.  */
    usb_network_device_ptr -> ux_network_device_usb_link_up = NX_FALSE;

    /* Set the link status.  */
    usb_network_device_ptr -> ux_network_device_link_status = NX_FALSE;

}
#endif
