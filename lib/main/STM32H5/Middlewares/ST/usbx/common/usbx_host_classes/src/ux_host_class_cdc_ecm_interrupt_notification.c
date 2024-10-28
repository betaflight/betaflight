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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_ecm_interrupt_notification       PORTABLE C      */ 
/*                                                           6.1.11       */
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
/*    None                                                                */
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_cdc_ecm_interrupt_notification(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_CDC_ECM                       *cdc_ecm;
ULONG                                       notification_type;
ULONG                                       notification_value;


    /* Get the control class instance for this transfer request.  */
    cdc_ecm =  (UX_HOST_CLASS_CDC_ECM *) transfer_request -> ux_transfer_request_class_instance;
    
    /* Check the state of the transfer.  If there is an error, we do not proceed with this notification.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)

        /* We do not proceed.  */
        return;

    /* Check if the class is in shutdown.  */
    if (cdc_ecm -> ux_host_class_cdc_ecm_state ==  UX_HOST_CLASS_INSTANCE_SHUTDOWN)

        /* We do not proceed.  */
        return;

    /* Increment the notification count.   */
    cdc_ecm -> ux_host_class_cdc_ecm_notification_count++;

    /* Get the notification.  */
    notification_type = (ULONG) *(transfer_request -> ux_transfer_request_data_pointer + UX_HOST_CLASS_CDC_ECM_NPF_NOTIFICATION_TYPE);    
    
    /* And the value.  */
    notification_value = (ULONG) *(transfer_request -> ux_transfer_request_data_pointer + UX_HOST_CLASS_CDC_ECM_NPF_VALUE);    

    /* Check if the notification is a Network notification.  */
    if (notification_type == UX_HOST_CLASS_CDC_ECM_NOTIFICATION_NETWORK_CONNECTION)
    {

        /* Check the state of the link.  */
        if (notification_value == UX_HOST_CLASS_CDC_ECM_NOTIFICATION_NETWORK_LINK_UP)
        {

            /* Link is up. See if we know about that.  */
            if (cdc_ecm -> ux_host_class_cdc_ecm_link_state != UX_HOST_CLASS_CDC_ECM_LINK_STATE_UP && 
                cdc_ecm -> ux_host_class_cdc_ecm_link_state != UX_HOST_CLASS_CDC_ECM_LINK_STATE_PENDING_UP)
            {
        
                /* Memorize the new link state.  */
                cdc_ecm -> ux_host_class_cdc_ecm_link_state =  UX_HOST_CLASS_CDC_ECM_LINK_STATE_PENDING_UP;                    
                
                /* We need to inform the cdc_ecm thread of this change.  */
                _ux_host_semaphore_put(&cdc_ecm -> ux_host_class_cdc_ecm_interrupt_notification_semaphore);
            }
        }
        else
        {

            /* Link is down. See if we know about that.  */
            if (cdc_ecm -> ux_host_class_cdc_ecm_link_state != UX_HOST_CLASS_CDC_ECM_LINK_STATE_DOWN && 
                cdc_ecm -> ux_host_class_cdc_ecm_link_state != UX_HOST_CLASS_CDC_ECM_LINK_STATE_PENDING_DOWN)
            {

                /* We need to abort any transfers on the bulk in endpoint.  */

                /* Make sure no one does any more transfers.  */
                cdc_ecm -> ux_host_class_cdc_ecm_link_state =  UX_HOST_CLASS_CDC_ECM_LINK_STATE_PENDING_DOWN;

                /* Now abort all transfers. It's possible we're executing right before the transfer
                   is armed. If this is the case, then the transfer will not be aborted if we do the abort right now; instead, 
                   we should wait until after the transfer is armed. We must look at the CDC-ECM thread's state.  */

                /* Is it in the process of checking the link state and arming the transfer?  */
                if (cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_check_and_arm_in_process == UX_TRUE)
                {

                    /* Yes. We must wait for it to finish arming the transfer.  */

                    /* Let the CDC-ECM thread know we're waiting so it can wake us up.  */
                    cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish =  UX_TRUE;

                    /* Wait for the transfer to be armed, or possibly an error. The CDC-ECM thread will wake us up.  */
                    _ux_host_semaphore_get_norc(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish_semaphore, UX_WAIT_FOREVER);

                    /* We're no longer waiting.  */
                    cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish =  UX_FALSE;
                }

                /* Now we can abort the transfer.  */
                _ux_host_stack_transfer_request_abort(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_endpoint -> ux_endpoint_transfer_request);

                /* We need to inform the CDC-ECM thread of this change.  */
                _ux_host_semaphore_put(&cdc_ecm -> ux_host_class_cdc_ecm_interrupt_notification_semaphore);
            }
        }           
    }        

    /* Reactivate the CDC_ECM interrupt pipe.  */
    _ux_host_stack_transfer_request(transfer_request);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_CDC_ECM_INTERRUPT_NOTIFICATION, cdc_ecm, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Return to caller.  */
    return;
}

