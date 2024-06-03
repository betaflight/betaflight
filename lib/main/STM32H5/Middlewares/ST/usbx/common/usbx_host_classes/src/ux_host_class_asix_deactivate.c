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
/**   Asix Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_asix.h"
#include "ux_host_stack.h"


#if !defined(UX_HOST_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_asix_deactivate                      PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when this instance of the asix has been     */
/*    removed from the bus either directly or indirectly. The bulk in\out */ 
/*    and interrupt pipes will be destroyed and the instance              */ 
/*    removed.                                                            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                              Asix class command pointer     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_instance_destroy Destroy the class instance    */ 
/*    _ux_host_stack_endpoint_transfer_abort Abort endpoint transfer      */ 
/*    _ux_utility_memory_free               Free memory block             */ 
/*    _ux_host_semaphore_get                Get protection semaphore      */ 
/*    _ux_host_semaphore_delete             Delete protection semaphore   */ 
/*    _ux_utility_thread_delete             Delete thread                 */
/*    _ux_network_driver_deactivate         Deactivate NetX USB interface */
/*    nx_packet_transmit_release            Release NetX packet           */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_asix_entry             Entry of asix class           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            supported NX packet chain,  */
/*                                            added reception buffer,     */
/*                                            removed internal NX pool,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_asix_deactivate(UX_HOST_CLASS_COMMAND *command)
{

UX_HOST_CLASS_ASIX          *asix;
UX_TRANSFER                 *transfer_request;
NX_PACKET                   *current_packet;
NX_PACKET                   *next_packet;
UINT                        status;

    /* Get the instance for this class.  */
    asix =  (UX_HOST_CLASS_ASIX *) command -> ux_host_class_command_instance;

    /* Protect thread reentry to this instance.  */
    status =  _ux_host_semaphore_get(&asix -> ux_host_class_asix_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)

        /* Return error.  */
        return(status);

    /* The asix is being shut down.  */
    asix -> ux_host_class_asix_state =  UX_HOST_CLASS_INSTANCE_SHUTDOWN;
    
    
    /* Check if link was up.  */
    if (asix -> ux_host_class_asix_link_state == UX_HOST_CLASS_ASIX_LINK_STATE_UP)
    {
        /* We need to free the packets that will not be sent.  */
        current_packet =  asix -> ux_host_class_asix_xmit_queue;
    
        /* Get the next packet associated with the first packet.  */
        next_packet = current_packet -> nx_packet_queue_next;
        
        /* Parse all these packets that were scheduled.  */
        while (current_packet != UX_NULL)
        {
        
            /* Free the packet that was just sent.  First do some housekeeping.  */
            current_packet -> nx_packet_prepend_ptr =  current_packet -> nx_packet_prepend_ptr + UX_HOST_CLASS_ASIX_ETHERNET_SIZE; 
            current_packet -> nx_packet_length =  current_packet -> nx_packet_length - UX_HOST_CLASS_ASIX_ETHERNET_SIZE;

            /* And ask Netx to release it.  */
            nx_packet_transmit_release(current_packet); 
            
            /* Next packet becomes the current one.  */
            current_packet = next_packet;
            
            /* Next packet now.  */
            if (current_packet != UX_NULL)

                /* Get the next packet associated with the first packet.  */
                next_packet = current_packet -> nx_packet_queue_next;
        
        }
        
    }

    /* Now the link is down.  */
    asix -> ux_host_class_asix_link_state = UX_HOST_CLASS_ASIX_LINK_STATE_DOWN;

    /* Deregister this interface to the NetX USB interface broker.  */
    _ux_network_driver_deactivate((VOID *) asix, asix -> ux_host_class_asix_network_handle);
    
    /* If the interrupt endpoint is defined, clean any pending transfer.  */
    if (asix -> ux_host_class_asix_interrupt_endpoint != UX_NULL)
    {    
        
        /* Wait for any current transfer to be out of pending.  */
        transfer_request =  &asix -> ux_host_class_asix_interrupt_endpoint -> ux_endpoint_transfer_request;
        if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)

            /* And abort any transfer.  */
            _ux_host_stack_endpoint_transfer_abort(asix -> ux_host_class_asix_interrupt_endpoint);
        
        /* And free the memory used by the interrupt endpoint.  */
        _ux_utility_memory_free(transfer_request -> ux_transfer_request_data_pointer);
        
    }
           
    /* First we take care of cleaning endpoint IN.  */
    transfer_request =  &asix -> ux_host_class_asix_bulk_in_endpoint -> ux_endpoint_transfer_request;
    if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)

        /* We need to abort transactions on the bulk In pipe.  */
        _ux_host_stack_endpoint_transfer_abort(asix -> ux_host_class_asix_bulk_in_endpoint);
    
    /* Then endpoint OUT.  */       
    transfer_request =  &asix -> ux_host_class_asix_bulk_out_endpoint -> ux_endpoint_transfer_request;
    if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)
       
        /* We need to abort transactions on the bulk Out pipe.  We normally don't need that anymore. */
        _ux_host_stack_endpoint_transfer_abort(asix -> ux_host_class_asix_bulk_out_endpoint);
    
    /* The enumeration thread needs to sleep a while to allow the application or the class that may be using
       endpoints to exit properly.  */
    _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 

    /* Destroy the instance.  */
    _ux_host_stack_class_instance_destroy(asix -> ux_host_class_asix_class, (VOID *) asix);

    /* Destroy the semaphores.  */
    _ux_host_semaphore_delete(&asix -> ux_host_class_asix_semaphore);
    _ux_host_semaphore_delete(&asix -> ux_host_class_asix_interrupt_notification_semaphore);
    
    /* Destroy the link monitoring thread.  */
    _ux_utility_thread_delete(&asix -> ux_host_class_asix_thread);
    
    /* free its stack memory.  */
    _ux_utility_memory_free(asix -> ux_host_class_asix_thread_stack);

    /* Free receive buffer memory.  */
    _ux_utility_memory_free(asix -> ux_host_class_asix_receive_buffer);

#ifdef UX_HOST_CLASS_ASIX_PACKET_CHAIN_SUPPORT

    /* Free transmit buffer memory.  */
    if (asix -> ux_host_class_asix_xmit_buffer)
        _ux_utility_memory_free(asix -> ux_host_class_asix_xmit_buffer);
#endif

    /* Before we free the device resources, we need to inform the application
        that the device is removed.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {
        
        /* Inform the application the device is removed.  */
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_REMOVAL, asix -> ux_host_class_asix_class, (VOID *) asix);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_ASIX_DEACTIVATE, asix, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(asix);

    /* Free the asix instance memory.  */
    _ux_utility_memory_free(asix);

    /* Return successful status.  */
    return(UX_SUCCESS);         
}
#endif
