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
/**   ASIX Class                                                          */
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
/*    _ux_host_class_asix_interrupt_notification          PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called by the stack when an interrupt packet as    */ 
/*    been received.                                                      */ 
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
/*    _ux_host_semaphore_put                Put semaphore                 */
/*    _ux_host_stack_transfer_request       Transfer request              */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX stack                                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            use pre-calculated value    */
/*                                            instead of wMaxPacketSize,  */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined link up/down flow,  */
/*                                            refined interrupt flow,     */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_asix_interrupt_notification(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_ASIX                      *asix;

    /* Get the class instance for this transfer request.  */
    asix =  (UX_HOST_CLASS_ASIX *) transfer_request -> ux_transfer_request_class_instance;
    
    /* Check the state of the transfer.  If there is an error, we do not proceed with this notification.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)

        /* We do not proceed.  */
        return;

    /* Check if the class is in shutdown.  */
    if (asix -> ux_host_class_asix_state ==  UX_HOST_CLASS_INSTANCE_SHUTDOWN)

        /* We do not proceed.  */
        return;

    /* Increment the notification count.   */
    asix -> ux_host_class_asix_notification_count++;

    /* Ensure the length of our interrupt pipe data is correct.  */
    if (transfer_request -> ux_transfer_request_actual_length == 
            transfer_request -> ux_transfer_request_requested_length)
    {

        /* Check if the first byte is a interrupt packet signature.  */
        if (*(transfer_request -> ux_transfer_request_data_pointer + UX_HOST_CLASS_ASIX_INTERRUPT_SIGNATURE_OFFSET) == 
                UX_HOST_CLASS_ASIX_INTERRUPT_SIGNATURE_VALUE)
        {

            /* Explore the content of the interrupt packet. We treat the link up/down flag here.  */
            if (*(transfer_request -> ux_transfer_request_data_pointer + UX_HOST_CLASS_ASIX_INTERRUPT_STATE_OFFSET) &
                    UX_HOST_CLASS_ASIX_INTERRUPT_STATE_PPLS)
            {

                /* Link is up. See if we know about that.  */
                if (asix -> ux_host_class_asix_link_state != UX_HOST_CLASS_ASIX_LINK_STATE_UP && 
                    asix -> ux_host_class_asix_link_state != UX_HOST_CLASS_ASIX_LINK_STATE_PENDING_UP)
                {
            
                    /* Memorize the new link state.  */
                    asix -> ux_host_class_asix_link_state = UX_HOST_CLASS_ASIX_LINK_STATE_PENDING_UP;                    
                    
                    /* We need to inform the asix thread of this change.  */
                    _ux_host_semaphore_put(&asix -> ux_host_class_asix_interrupt_notification_semaphore);

                    /* Reactivate interrupt pipe after link up handled.  */
                    return;
                }
            }                    
            else
            {

                /* Link is down. See if we know about that.  */
                if (asix -> ux_host_class_asix_link_state != UX_HOST_CLASS_ASIX_LINK_STATE_DOWN && 
                    asix -> ux_host_class_asix_link_state != UX_HOST_CLASS_ASIX_LINK_STATE_PENDING_DOWN)
                {
            
                    /* Memorize the new link state.  */
                    asix -> ux_host_class_asix_link_state = UX_HOST_CLASS_ASIX_LINK_STATE_PENDING_DOWN;

                    /* Abort possible pending bulk in requests.  */
                    _ux_host_stack_endpoint_transfer_abort(asix -> ux_host_class_asix_bulk_in_endpoint);

                    /* We need to inform the asix thread of this change.  */
                    _ux_host_semaphore_put(&asix -> ux_host_class_asix_interrupt_notification_semaphore);

                    /* Reactivate interrupt pipe after link down handled.  */
                    return;
                }
            }
        }
    }

    /* Reactivate the ASIX interrupt pipe.  */
    _ux_host_stack_transfer_request(transfer_request);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_ASIX_INTERRUPT_NOTIFICATION, asix, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Return to caller.  */
    return;
}

