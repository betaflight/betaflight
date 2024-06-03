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
/*    _ux_host_class_cdc_ecm_deactivate                   PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when this instance of the cdc_ecm has been  */
/*    removed from the bus either directly or indirectly. The bulk in\out */ 
/*    and interrupt pipes will be destroyed and the instance              */ 
/*    removed.                                                            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                           CDC ECM class command pointer     */ 
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
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_ecm_entry       Entry of cdc_ecm class           */ 
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
/*  02-02-2021     Xiuwen Cai               Modified comment(s), added    */
/*                                            compile option for using    */
/*                                            packet pool from NetX,      */
/*                                            resulting in version 6.1.4  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            deprecated ECM pool option, */
/*                                            supported NX packet chain,  */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_cdc_ecm_deactivate(UX_HOST_CLASS_COMMAND *command)
{

UX_INTERRUPT_SAVE_AREA

UX_HOST_CLASS_CDC_ECM       *cdc_ecm;
UX_TRANSFER                 *transfer_request;

    /* This must be the data interface, since the control interface doesn't have
       a class instance.  */

    /* Get the control instance for this class.  */
    cdc_ecm =  (UX_HOST_CLASS_CDC_ECM *) command -> ux_host_class_command_instance;

    /* The cdc_ecm is being shut down.  */
    cdc_ecm -> ux_host_class_cdc_ecm_state =  UX_HOST_CLASS_INSTANCE_SHUTDOWN;
    
    /* If the interrupt endpoint is defined, abort transfers so the link state
       doesn't change.  */
    if (cdc_ecm -> ux_host_class_cdc_ecm_interrupt_endpoint != UX_NULL)
    {    

        /* Get the transfer request.  */
        transfer_request =  &cdc_ecm -> ux_host_class_cdc_ecm_interrupt_endpoint -> ux_endpoint_transfer_request;

        /* Abort any transfers.  */
        _ux_host_stack_transfer_request_abort(transfer_request);
    }

    /* Check if link was up to see if we should clean the transmit queue. If 
       the link is pending down, that means the CDC-ECM thread is in the process 
       of cleaning the transmit queue.  */
    if (cdc_ecm -> ux_host_class_cdc_ecm_link_state == UX_HOST_CLASS_CDC_ECM_LINK_STATE_UP)

        _ux_host_class_cdc_ecm_transmit_queue_clean(cdc_ecm);

    /* Get the bulk in transfer request.  */
    transfer_request =  &cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_endpoint -> ux_endpoint_transfer_request;

    /* Now abort all transfers. It's possible we're executing right before the transfer
       is armed. If this is the case, then the transfer will not be aborted if we do the abort right now; instead, 
       we should wait until after the transfer is armed. We must look at the CDC-ECM thread's state.  */

    /* Disable interrupts while we check the link state and possibly set our state.  */
    UX_DISABLE

    /* Is it in the process of checking the link state and arming the transfer?  */
    if (cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_check_and_arm_in_process == UX_TRUE)
    {

        /* Yes. We must wait for it to finish arming the transfer.  */

        /* Let the CDC-ECM thread know we're waiting so it can wake us up.  */
        cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish =  UX_TRUE;

        /* Restore interrupts.  */
        UX_RESTORE

        /* Wait for the transfer to be armed, or possibly an error. The CDC-ECM thread will wake us up.  */
        _ux_host_semaphore_get_norc(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish_semaphore, UX_WAIT_FOREVER);

        /* We're no longer waiting.  */
        cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish =  UX_FALSE;
    }
    else
    {

        /* Restore interrupts.  */
        UX_RESTORE
    }

    /* Now we can abort the transfer.  */
    _ux_host_stack_transfer_request_abort(transfer_request);

    /* De-register this interface to the NetX USB interface broker.  */
    _ux_network_driver_deactivate((VOID *) cdc_ecm, cdc_ecm -> ux_host_class_cdc_ecm_network_handle);

    /* Destroy the control instance.  */
    _ux_host_stack_class_instance_destroy(cdc_ecm -> ux_host_class_cdc_ecm_class, (VOID *) cdc_ecm);

    /* Now wait for all threads to leave the instance before freeing the resources.  */
    _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 

    /* Free the memory used by the interrupt endpoint.  */
    if (cdc_ecm -> ux_host_class_cdc_ecm_interrupt_endpoint != UX_NULL)
        _ux_utility_memory_free(cdc_ecm -> ux_host_class_cdc_ecm_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer);

    /* Destroy the link monitoring thread. We should do this before destroying
       the notification semaphore so that it does not run due to semaphore deletion.  */
    _ux_utility_thread_delete(&cdc_ecm -> ux_host_class_cdc_ecm_thread);

    /* Free the CDC-ECM thread's stack memory.  */
    _ux_utility_memory_free(cdc_ecm -> ux_host_class_cdc_ecm_thread_stack);

    /* Destroy the bulk semaphores.  */
    _ux_host_semaphore_delete(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_out_transfer_waiting_for_check_and_arm_to_finish_semaphore);
    _ux_host_semaphore_delete(&cdc_ecm -> ux_host_class_cdc_ecm_bulk_in_transfer_waiting_for_check_and_arm_to_finish_semaphore);

    /* Destroy the notification semaphore.  */
    _ux_host_semaphore_delete(&cdc_ecm -> ux_host_class_cdc_ecm_interrupt_notification_semaphore);

#ifdef UX_HOST_CLASS_CDC_ECM_PACKET_CHAIN_SUPPORT

    /* Free packet transmission memories.  */
    if (cdc_ecm -> ux_host_class_cdc_ecm_receive_buffer)
        _ux_utility_memory_free(cdc_ecm -> ux_host_class_cdc_ecm_receive_buffer);
    if (cdc_ecm -> ux_host_class_cdc_ecm_xmit_buffer)
        _ux_utility_memory_free(cdc_ecm -> ux_host_class_cdc_ecm_xmit_buffer);
#endif

    /* Before we free the device resources, we need to inform the application
        that the device is removed.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        
        /* Inform the application the device is removed.  */
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_REMOVAL, cdc_ecm -> ux_host_class_cdc_ecm_class, (VOID *) cdc_ecm);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_CDC_ECM_DEACTIVATE, cdc_ecm, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, unregister this object.  */
    UX_TRACE_OBJECT_UNREGISTER(cdc_ecm);

    /* Free the cdc_ecm control instance memory.  */
    _ux_utility_memory_free(cdc_ecm);

    /* Return successful status.  */
    return(UX_SUCCESS);
}
#endif
