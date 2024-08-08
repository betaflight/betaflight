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
/*    _ux_host_class_cdc_ecm_transmit_queue_clean         PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function cleans the transmit queue.                            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_ecm                             CDC ECM instance                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    No return value                                                     */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_semaphore_get                Get bulk out semaphore        */ 
/*    _ux_host_stack_endpoint_transfer_abort Abort endpoint transfer      */ 
/*    nx_packet_transmit_release            Release NetX packet           */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    CDC ECM thread and deactivation                                     */ 
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
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_cdc_ecm_transmit_queue_clean(UX_HOST_CLASS_CDC_ECM *cdc_ecm)
{

UX_INTERRUPT_SAVE_AREA

NX_PACKET               *current_packet;
NX_PACKET               *next_packet;

    /* Disable interrupts while we check the write in process flag and
       set our own state.  */
    UX_DISABLE

    /* Is there a write in process?  */
    if (cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_check_and_arm_in_process == UX_TRUE)
    {

        /* Wait for writes to complete. Note that once these writes complete,
           no more should occur since the link state is pending down.  */

        /* Mark this thread as suspended so it will be woken up.  */
        cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_waiting_for_check_and_arm_to_finish =  UX_TRUE;

        /* Restore interrupts while we wait.  */
        UX_RESTORE

        /* Wait for write function to resume us.  */
        _ux_host_semaphore_get_norc(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_waiting_for_check_and_arm_to_finish_semaphore, UX_WAIT_FOREVER);

        /* We're done waiting.  */
        cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_waiting_for_check_and_arm_to_finish =  UX_FALSE;
    }
    else
    {

        /* No writes are in process. Restore interrupts and go on to free
           the xmit queue.  */
        UX_RESTORE
    }

    /* Abort transfers on the bulk out endpoint. Note we need to do this
       before accessing the queue since the transmission callback might
       modify it.  */
    _ux_host_stack_transfer_request_abort(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_endpoint -> ux_endpoint_transfer_request);

    /* Get the first packet.  */
    current_packet =  cdc_ecm -> ux_host_class_cdc_ecm_xmit_queue_head;

    /* We need to free the packets that will not be sent.  */
    while (current_packet != UX_NULL)
    {

        /* We must get the next packet before releasing the current packet 
           because nxe_packet_transmit_release sets the pointer we pass 
           to null.  */
        next_packet =  current_packet -> nx_packet_queue_next;

        /* Free the packet. First do some housekeeping.  */
        current_packet -> nx_packet_prepend_ptr =  current_packet -> nx_packet_prepend_ptr + UX_HOST_CLASS_CDC_ECM_ETHERNET_SIZE; 
        current_packet -> nx_packet_length =  current_packet -> nx_packet_length - UX_HOST_CLASS_CDC_ECM_ETHERNET_SIZE;

        /* And ask Netx to release it.  */
        nx_packet_transmit_release(current_packet);

        /* Next packet becomes the current one.  */
        current_packet =  next_packet;
    }

    /* Clear the queue.  */
    cdc_ecm -> ux_host_class_cdc_ecm_xmit_queue_head =  UX_NULL;
}
#endif
