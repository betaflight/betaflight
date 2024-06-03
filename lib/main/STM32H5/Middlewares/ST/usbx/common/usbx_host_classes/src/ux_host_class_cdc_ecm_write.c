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


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_ecm.h"
#include "ux_host_stack.h"


#if !defined(UX_HOST_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_ecm_write                        PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function writes to the cdc_ecm interface. The call is          */ 
/*    non-blocking and queues the packet if there is an on-going write.   */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_ecm                               Pointer to cdc_ecm class      */ 
/*    packet                                Packet to write or queue      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_host_semaphore_put                Release protection semaphore  */ 
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
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            supported NX packet chain,  */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_cdc_ecm_write(VOID *cdc_ecm_class, NX_PACKET *packet)
{

UX_INTERRUPT_SAVE_AREA

UX_TRANSFER             *transfer_request;
UINT                    status;
UCHAR                   *packet_header;
UX_HOST_CLASS_CDC_ECM   *cdc_ecm;
#ifdef UX_HOST_CLASS_CDC_ECM_PACKET_CHAIN_SUPPORT
ULONG                   copied;
#endif

    /* Get the instance.  */
    cdc_ecm = (UX_HOST_CLASS_CDC_ECM *) cdc_ecm_class;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_CDC_ECM_WRITE, cdc_ecm, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* We're arming transfer now.  */
    cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_check_and_arm_in_process =  UX_TRUE;

    /* We need to disable interrupts here because we need to make sure that if the xmit
       queue is non-null, it remains non-null until we have queued the packet.
       Note that we do not have to worry about the case where the queue is null,
       because we are the only ones that can change it from null to non-null.  */
    UX_DISABLE

    /* Ensure the instance is valid.  */
    if (cdc_ecm -> ux_host_class_cdc_ecm_state !=  UX_HOST_CLASS_INSTANCE_LIVE)
    {        

        /* Restore interrupts.  */
        UX_RESTORE

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, cdc_ecm, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Validate packet length.  */
    if (packet -> nx_packet_length > UX_HOST_CLASS_CDC_ECM_NX_PAYLOAD_SIZE)
    {

        /* Restore interrupts.  */
        UX_RESTORE

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CLASS_ETH_SIZE_ERROR);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CLASS_ETH_SIZE_ERROR, cdc_ecm, packet -> nx_packet_length, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_CLASS_ETH_SIZE_ERROR);
    }

    /* Are we in a valid state?  */
    if (cdc_ecm -> ux_host_class_cdc_ecm_link_state == UX_HOST_CLASS_CDC_ECM_LINK_STATE_UP)
    {

        /* Check the queue. See if there is something that is being sent.  */
        if (cdc_ecm -> ux_host_class_cdc_ecm_xmit_queue_head == UX_NULL)
        {
                
            /* Reset the queue pointer of this packet.  */
            packet -> nx_packet_queue_next =  UX_NULL;
            
            /* Memorize this packet at the beginning of the queue.  */
            cdc_ecm -> ux_host_class_cdc_ecm_xmit_queue_head =  packet;
            cdc_ecm -> ux_host_class_cdc_ecm_xmit_queue_tail =  packet;

            /* Restore interrupts.  */
            UX_RESTORE

            /* Now we need to arm the transfer.  */

            /* Get the pointer to the bulk out endpoint transfer request.  */
            transfer_request =  &cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_endpoint -> ux_endpoint_transfer_request;

#ifdef UX_HOST_CLASS_CDC_ECM_PACKET_CHAIN_SUPPORT

            if (packet -> nx_packet_next != UX_NULL)
            {

                /* Create buffer.  */
                if (cdc_ecm -> ux_host_class_cdc_ecm_xmit_buffer == UX_NULL)
                {
                    cdc_ecm -> ux_host_class_cdc_ecm_xmit_buffer = _ux_utility_memory_allocate(UX_NO_ALIGN,
                                        UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_CDC_ECM_NX_PAYLOAD_SIZE);
                    if (cdc_ecm -> ux_host_class_cdc_ecm_xmit_buffer == UX_NULL)
                    {
                        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);
                        return(UX_MEMORY_INSUFFICIENT);
                    }
                }

                /* Put packet to continuous buffer to transfer.  */
                packet_header = cdc_ecm -> ux_host_class_cdc_ecm_xmit_buffer;
                nx_packet_data_extract_offset(packet, 0, packet_header, packet -> nx_packet_length, &copied);
            }
            else
#endif
            {

                /* Load the address of the current packet header at the physical header.  */
                packet_header =  packet -> nx_packet_prepend_ptr;

            }

            /* Setup the transaction parameters.  */
            transfer_request -> ux_transfer_request_data_pointer     =  packet_header;
            transfer_request -> ux_transfer_request_requested_length =  packet -> nx_packet_length;
            
            /* Store the packet that owns this transaction.  */
            transfer_request -> ux_transfer_request_user_specific =  packet;

            /* Arm the transfer request.  */
            status =  _ux_host_stack_transfer_request(transfer_request);

            /* Did we successfully arm the transfer?  */
            if (status != UX_SUCCESS)
            {

                /* Clear the queue. No need to clear the tail.  */
                cdc_ecm -> ux_host_class_cdc_ecm_xmit_queue_head =  UX_NULL;

                /* We cleared the queue, so we must free the packet. First
                   we need to clean it before passing it to NetX.  */
                packet -> nx_packet_prepend_ptr =  packet -> nx_packet_prepend_ptr + UX_HOST_CLASS_CDC_ECM_ETHERNET_SIZE; 
                packet -> nx_packet_length =  packet -> nx_packet_length - UX_HOST_CLASS_CDC_ECM_ETHERNET_SIZE;

                /* And ask Netx to release it.  */
                nx_packet_transmit_release(packet);
    
                /* Could not arm this transfer.  */
                status =  UX_ERROR;
            }
        }
        else
        {

            /* The packet to be sent is the last in the chain.  */
            packet -> nx_packet_queue_next =  NX_NULL;

            /* Memorize the packet to be sent.  */
            cdc_ecm -> ux_host_class_cdc_ecm_xmit_queue_tail -> nx_packet_queue_next =  packet;

            /* Set the tail.  */
            cdc_ecm -> ux_host_class_cdc_ecm_xmit_queue_tail =  packet;

            /* Restore interrupts.  */
            UX_RESTORE

            /* Successfully added to queue.  */
            status =  UX_SUCCESS;
        }
    }
    else
    {

        /* Link was down.  */

        /* Restore interrupts.  */
        UX_RESTORE

        /* Release the packet.  */
        packet -> nx_packet_prepend_ptr =  packet -> nx_packet_prepend_ptr + UX_HOST_CLASS_CDC_ECM_ETHERNET_SIZE; 
        packet -> nx_packet_length =  packet -> nx_packet_length - UX_HOST_CLASS_CDC_ECM_ETHERNET_SIZE;
        nx_packet_transmit_release(packet);

        /* Report error to application.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CLASS_CDC_ECM_LINK_STATE_DOWN_ERROR);

        /* Return error.  */
        status =  UX_ERROR;
    }

    /* Signal that we are done arming and resume waiting thread if necessary.  */
    cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_check_and_arm_in_process =  UX_FALSE;
    if (cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_waiting_for_check_and_arm_to_finish == UX_TRUE)
        _ux_host_semaphore_put(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_waiting_for_check_and_arm_to_finish_semaphore);

    /* We are done here.  */
    return(status);            
}
#endif
