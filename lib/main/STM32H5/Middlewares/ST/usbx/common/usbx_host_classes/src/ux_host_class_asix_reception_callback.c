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
/*    _ux_host_class_asix_reception_callback              PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the callback from the USBX transfer functions,     */ 
/*    it is called when a full or partial transfer has been done for a    */ 
/*    bulk in transfer. It calls back the application.                    */
/*                                                                        */
/*    It's deprecated.                                                    */
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
/*    _ux_utility_short_get_big_endian      Get 16-bit big endian         */
/*    _ux_network_driver_packet_received    Process received packet       */
/*    nx_packet_transmit_release            Release NetX packet           */
/*    nx_packet_allocate                    Allocate NetX packet          */
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            deprecated, no RX callback, */
/*                                            modified pool reference,    */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_asix_reception_callback (UX_TRANSFER *transfer_request)
{
#if defined(UX_HOST_STANDALONE)
    UX_PARAMETER_NOT_USED(transfer_request);
#else

UX_HOST_CLASS_ASIX              *asix;
NX_PACKET                       *packet;
ULONG                           ip_given_length;
UINT                            status;
    
    /* Get the class instance for this transfer request.  */
    asix =  (UX_HOST_CLASS_ASIX *) transfer_request -> ux_transfer_request_class_instance;
    
    /* Load the packet that is associated with this reception.  */
    packet = transfer_request -> ux_transfer_request_user_specific;
    
    /* Check the state of the transfer.  If there is an error, we do not proceed with this report.  */
    if (transfer_request -> ux_transfer_request_completion_code == UX_SUCCESS)
    {

        /* Adjust the prepend, length, append fields to take the Ether header out */ 
        packet -> nx_packet_prepend_ptr += 4;                                    
        packet -> nx_packet_length = transfer_request -> ux_transfer_request_actual_length - 4;
        packet -> nx_packet_append_ptr =
            packet->nx_packet_prepend_ptr + transfer_request -> ux_transfer_request_actual_length ;
    
        /* Calculate the accurate packet length from ip header */ 
        if((*(packet -> nx_packet_prepend_ptr + 12) == 0x08) && 
            (*(packet -> nx_packet_prepend_ptr + 13) == 0))
        {
            ip_given_length = _ux_utility_short_get_big_endian(packet -> nx_packet_prepend_ptr + 16) + UX_HOST_CLASS_ASIX_ETHERNET_SIZE;
            packet->nx_packet_length = ip_given_length ;
            packet->nx_packet_append_ptr =  packet->nx_packet_prepend_ptr + ip_given_length;
        }
    
        /* Send that packet to the NetX USB broker.  */
        _ux_network_driver_packet_received(asix -> ux_host_class_asix_network_handle, packet);

    }    

    else
        
        /* Free the packet that was not successfully received.  */
        nx_packet_transmit_release(packet);
    
    /* We can accept new reception. Get a NX Packet */
    if (nx_packet_allocate(asix -> ux_host_class_asix_packet_pool, &packet, 
                            NX_RECEIVE_PACKET, NX_NO_WAIT) == NX_SUCCESS)            
    {

        /* Adjust the prepend pointer to take into account the non 3 bit alignment of the ethernet header.  */
        packet -> nx_packet_prepend_ptr += sizeof(USHORT);

        /* Set the data pointer.  */
        transfer_request -> ux_transfer_request_data_pointer =  packet -> nx_packet_prepend_ptr;

        /* And length.  */
        transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_ASIX_NX_PAYLOAD_SIZE;
        transfer_request -> ux_transfer_request_actual_length =     0;

        /* Store the packet that owns this transaction.  */
        transfer_request -> ux_transfer_request_user_specific = packet;

        /* Memorize this packet at the beginning of the queue.  */
        asix -> ux_host_class_asix_receive_queue = packet;
    
        /* Reset the queue pointer of this packet.  */
        packet -> nx_packet_queue_next = UX_NULL;

        /* Ask USB to schedule a reception.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check if the transaction was armed successfully. We do not wait for the packet to be sent here. */
        if (status != UX_SUCCESS)
        {

            /* Cancel the packet.  */
            asix -> ux_host_class_asix_receive_queue = UX_NULL;
    
        }
    }
    /* There is no status to be reported back to the stack.  */
    return; 
#endif
}
