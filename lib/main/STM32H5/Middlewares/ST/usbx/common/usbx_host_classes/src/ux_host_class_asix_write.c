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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_asix_write                           PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function writes to the asix interface. The call is blocking    */ 
/*    and only returns when there is either an error or when the transfer */ 
/*    is complete.                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    asix                                  Pointer to asix class         */ 
/*    packet                                Packet to write or queue      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_short_put                 Put 16-bit value              */
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
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            resulting in version 6.1.8  */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            supported NX packet chain,  */
/*                                            added queue modify protect, */
/*                                            improved error check,       */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_asix_write(VOID *asix_class, NX_PACKET *packet)
{
#if defined(UX_HOST_STANDALONE)
    UX_PARAMETER_NOT_USED(asix_class);
    UX_PARAMETER_NOT_USED(packet);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_INTERRUPT_SAVE_AREA
UX_TRANSFER         *transfer_request;
UINT                status;
NX_PACKET           *current_packet;
NX_PACKET           *next_packet;
UCHAR               *packet_header;
UX_HOST_CLASS_ASIX  *asix;
ULONG               adjusted_length;
#ifdef UX_HOST_CLASS_ASIX_PACKET_CHAIN_SUPPORT
ULONG               copied;
#endif

    /* Proper class casting.  */
    asix = (UX_HOST_CLASS_ASIX *) asix_class;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_ASIX_WRITE, asix, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Critical enter.  */
    UX_DISABLE

    /* Ensure the instance is valid.  */
    if (asix -> ux_host_class_asix_state !=  UX_HOST_CLASS_INSTANCE_LIVE)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, asix, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Critical exit.  */
        UX_RESTORE;
        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Ensure link state is OK.  */
    if (asix -> ux_host_class_asix_link_state != UX_HOST_CLASS_ASIX_LINK_STATE_UP)
    {

        /* Critical exit.  */
        UX_RESTORE;

        /* Release the packet.  */
        packet -> nx_packet_prepend_ptr =  packet -> nx_packet_prepend_ptr + NX_ETHERNET_SIZE; 
        packet -> nx_packet_length =  packet -> nx_packet_length - NX_ETHERNET_SIZE;
        nx_packet_transmit_release(packet);

        /* Report error to application.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CLASS_ETH_LINK_STATE_DOWN_ERROR);
        return(UX_CLASS_ETH_LINK_STATE_DOWN_ERROR);
    }

    /* Load the address of the current packet header at the physical header.  */
    packet_header =  packet -> nx_packet_prepend_ptr;
    
    /* Subtract 2 USHORT to store length of the packet.  */
    packet_header -= sizeof(USHORT) * 2;

#if defined(UX_HOST_CLASS_ASIX_HEADER_CHECK_ENABLE)

    /* Check packet compatibility to avoid writing to unexpected area.  */
    if (packet_header < packet -> nx_packet_data_start)
    {

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_MEMORY_ERROR);

        UX_RESTORE;
        return(UX_HOST_CLASS_MEMORY_ERROR);
    }
#endif

    /* Packet length validation.  */
    if (UX_OVERFLOW_CHECK_ADD_ULONG(packet -> nx_packet_length, sizeof(USHORT) * 2))
    {

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MATH_OVERFLOW);
        UX_RESTORE
        return(UX_MATH_OVERFLOW);
    }
    adjusted_length = packet -> nx_packet_length + sizeof(USHORT) * 2;
    if (adjusted_length > UX_HOST_CLASS_ASIX_TRANSMIT_BUFFER_SIZE)
    {

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CLASS_ETH_SIZE_ERROR);
        UX_RESTORE
        return(UX_CLASS_ETH_SIZE_ERROR);
    }

    /* Store the length of the payload in the first USHORT.  */
    _ux_utility_short_put(packet_header, (USHORT)(packet -> nx_packet_length));
    
    /* Store the negative length of the payload in the first USHORT.  */
    _ux_utility_short_put(packet_header + sizeof(USHORT), (USHORT)(~packet -> nx_packet_length));

    /* Check the queue. See if there is something that is being sent. */
    if (asix -> ux_host_class_asix_xmit_queue == UX_NULL)
    {

        /* Reset the queue pointer of this packet.  */
        packet -> nx_packet_queue_next = UX_NULL;

        /* Memorize this packet at the beginning of the queue.  */
        asix -> ux_host_class_asix_xmit_queue = packet;

        UX_RESTORE

        /* Nothing is in the queue. We need to arm this transfer. */
        /* Get the pointer to the bulk out endpoint transfer request.  */
        transfer_request =  &asix -> ux_host_class_asix_bulk_out_endpoint -> ux_endpoint_transfer_request;

#ifdef UX_HOST_CLASS_ASIX_PACKET_CHAIN_SUPPORT

        /* Check if the packets are chained.  */
        if (packet -> nx_packet_next)
        {

            /* Create buffer.  */
            if (asix -> ux_host_class_asix_xmit_buffer == UX_NULL)
            {
                asix -> ux_host_class_asix_xmit_buffer = _ux_utility_memory_allocate(UX_NO_ALIGN,
                                    UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_ASIX_TRANSMIT_BUFFER_SIZE);
                if (asix -> ux_host_class_asix_xmit_buffer == UX_NULL)
                {
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);
                    return(UX_MEMORY_INSUFFICIENT);
                }
            }

            packet -> nx_packet_length = adjusted_length;
            packet -> nx_packet_prepend_ptr -= sizeof(USHORT) * 2;
            nx_packet_data_extract_offset(packet, 0, asix -> ux_host_class_asix_xmit_buffer, packet -> nx_packet_length, &copied);

            /* Setup the transaction parameters.  */
            transfer_request -> ux_transfer_request_requested_length =  packet -> nx_packet_length;
            transfer_request -> ux_transfer_request_data_pointer = asix -> ux_host_class_asix_xmit_buffer;

            /* Restore packet status.  */
            packet -> nx_packet_length -= sizeof(USHORT) * 2;
            packet -> nx_packet_prepend_ptr += sizeof(USHORT) * 2;
        }
        else
#endif
        {

            /* Setup the transaction parameters.  */
            transfer_request -> ux_transfer_request_data_pointer     =  packet_header;
            transfer_request -> ux_transfer_request_requested_length =  adjusted_length;
        }

        /* Store the packet that owns this transaction.  */
        transfer_request -> ux_transfer_request_user_specific = packet;

        /* Perform the transfer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check if the transaction was armed successfully. We do not wait for the packet to be sent here. */
        if (status != UX_SUCCESS)
        {

            /* Could not arm this transfer.  */
            asix -> ux_host_class_asix_xmit_queue = UX_NULL;
            return(UX_ERROR);
        }
    }

    else
    
    {
    
        /* We get here when there is something in the queue.  */
        current_packet =  asix -> ux_host_class_asix_xmit_queue;

        /* Get the next packet associated with the first packet.  */
        next_packet = current_packet -> nx_packet_queue_next;

        /* Parse the current chain for the end.  */
        while (next_packet != NX_NULL)
        {
            /* Remember the current packet.  */
            current_packet = next_packet;
            
            /* See what the next packet in the chain is.  */
            next_packet = current_packet -> nx_packet_queue_next;
        }

        /* Memorize the packet to be sent.  */
        current_packet -> nx_packet_queue_next = packet;

        /* The packet to be sent is the last in the chain.  */
        packet -> nx_packet_queue_next = NX_NULL;


        UX_RESTORE
    }

    /* We are done here.  */
    return(UX_SUCCESS);            
#endif
}
