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
/**   CDC-ECM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_ecm.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_ecm_transmission_callback        PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the callback from the USBX transfer functions,     */ 
/*    it is called when a full or partial transfer has been done for a    */ 
/*    bulk out transfer.                                                  */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    nx_packet_transmit_release            Release NetX packet           */
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
/*  02-02-2021     Chaoqiong Xiao           Modified comment(s), fixed    */
/*                                            ZLP issue for transmission, */
/*                                            resulting in version 6.1.4  */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            use pre-calculated value    */
/*                                            instead of wMaxPacketSize,  */
/*                                            resulting in version 6.1.9  */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            supported NX packet chain,  */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_cdc_ecm_transmission_callback(UX_TRANSFER *transfer_request)
{
#if defined(UX_HOST_STANDALONE)
    UX_PARAMETER_NOT_USED(transfer_request);
#else

UX_HOST_CLASS_CDC_ECM           *cdc_ecm;
NX_PACKET                       *current_packet;
NX_PACKET                       *next_packet;
UCHAR                           *packet_header;
#ifdef UX_HOST_CLASS_CDC_ECM_PACKET_CHAIN_SUPPORT
ULONG                           copied;
#endif
    
    /* Get the data and control class instances for this transfer request.  */
    cdc_ecm =  (UX_HOST_CLASS_CDC_ECM *) transfer_request -> ux_transfer_request_class_instance;

    /* Is the link not up, or the class is shutting down?  */
    if (cdc_ecm -> ux_host_class_cdc_ecm_link_state != UX_HOST_CLASS_CDC_ECM_LINK_STATE_UP ||
        cdc_ecm -> ux_host_class_cdc_ecm_state == UX_HOST_CLASS_INSTANCE_SHUTDOWN)

        /* The CDC-ECM thread or deactivation routine is in the process of freeing 
           the queue. Just return so we are not simultaneously accessing it.  */
        return;

    /* Get the packet associated with this transfer.  */
    current_packet =  cdc_ecm -> ux_host_class_cdc_ecm_xmit_queue_head;
    
    /* Do a sanity check on the packet.  */
    if (current_packet == UX_NULL)
    {
    
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FATAL_ERROR);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_INTERFACE_HANDLE_UNKNOWN, cdc_ecm, 0, 0, UX_FATAL_ERROR, 0, 0)
        
        /* Something went terribly wrong. Do not proceed.  */
        return;
    }

    /* Check the state of the transfer.  */
    if (transfer_request -> ux_transfer_request_completion_code == UX_SUCCESS)
    {

        /* Check if the transfer length is not zero and it is multiple of MPS (validated on device enum).  */
        if ((transfer_request -> ux_transfer_request_requested_length != 0) &&
            (transfer_request -> ux_transfer_request_requested_length % transfer_request -> ux_transfer_request_packet_length) == 0)
        {

            /* Set transfer request length to zero.  */
            transfer_request -> ux_transfer_request_requested_length =  0;

            /* Send the transfer.  */
            _ux_host_stack_transfer_request(transfer_request);

            /* Finished processing.  */
            return;
        }

        /* Get the next packet associated with the first packet.  */
        next_packet =  current_packet -> nx_packet_queue_next;

        /* Set the next packet (or a NULL value) as the head of the xmit queue. */
        cdc_ecm -> ux_host_class_cdc_ecm_xmit_queue_head =  next_packet;
        
        /* If there is nothing else or if the link is down no need to rearm a packet. */
        if (next_packet != UX_NULL)
        {

#ifdef UX_HOST_CLASS_CDC_ECM_PACKET_CHAIN_SUPPORT

            if (next_packet -> nx_packet_next != UX_NULL)
            {

                /* Put packet to continuous buffer to transfer.  */
                packet_header = cdc_ecm -> ux_host_class_cdc_ecm_xmit_buffer;
                nx_packet_data_extract_offset(next_packet, 0, packet_header, next_packet -> nx_packet_length, &copied);
            }
            else
#endif
            {

                /* Load the address of the current packet header at the physical header.  */
                packet_header =  next_packet -> nx_packet_prepend_ptr;
            }
        
            /* Prepare the values for this new transmission.  */
            transfer_request -> ux_transfer_request_data_pointer     =  packet_header;
            transfer_request -> ux_transfer_request_requested_length =  next_packet -> nx_packet_length;
            
            /* Store the packet that owns this transaction.  */
            transfer_request -> ux_transfer_request_user_specific = next_packet;

            /* If error log is enabled, insert this message into the log buffer.  */
            UX_DEBUG_LOG("_ux_host_class_cdc_ecm_transmission_callback", "Sending packet", next_packet, next_packet, _ux_system -> ux_system_mutex.tx_mutex_suspended_count)
        
            /* If there is an error, the system will hang up. Not much we can do to
               fix this.  */
            _ux_host_stack_transfer_request(transfer_request);
        }

        /* If error log is enabled, insert this message into the log buffer.  */
        UX_DEBUG_LOG("_ux_host_class_cdc_ecm_transmission_callback", "Freeing transmitted packet", 0, transfer_request, current_packet)
        
        /* Free the packet that was just sent.  First do some housekeeping.  */
        current_packet -> nx_packet_prepend_ptr =  current_packet -> nx_packet_prepend_ptr + UX_HOST_CLASS_CDC_ECM_ETHERNET_SIZE; 
        current_packet -> nx_packet_length =  current_packet -> nx_packet_length - UX_HOST_CLASS_CDC_ECM_ETHERNET_SIZE;

        /* And ask Netx to release it.  */
        nx_packet_transmit_release(current_packet); 
    }
    else
    {

        /* The transfer failed. Retry it. Note that this can't be a transfer
           abort, because otherwise the link is either down or the class is in
           shutdown, both of which are checked for at the beginning.  */
        _ux_host_stack_transfer_request(transfer_request);
    }

    /* There is no status to be reported back to the stack.  */
    return; 
#endif
}
