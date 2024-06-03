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
/**   Host Stack                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request                     PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a USB transaction. On entry the transfer     */ 
/*    request gives the endpoint pipe selected for this transaction and   */ 
/*    the parameters associated with the transfer (data payload, length   */ 
/*    of transaction)                                                     */
/*                                                                        */
/*    For Control pipe, the transaction is blocking and will only return  */ 
/*    when the 3 phases of the control transfer have been completed or if */ 
/*    there is a previous error. For other pipes, the USB stack will      */ 
/*    schedule the transaction on the USB but will not wait for its       */ 
/*    completion. Each request for non blocking pipes has to specify a    */ 
/*    completion routine.                                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    transfer_request                      Transfer request structure    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                     If UX_SUCCESS, transfer was   */ 
/*                                            successfully started        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    HCD Entry Function                                                  */ 
/*    _ux_utility_semaphore_put             Put semaphore                 */
/*    _ux_utility_semaphore_get             Get semaphore                 */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    USBX Components                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            optimized based on compile  */
/*                                            definitions, used UX prefix */
/*                                            to refer to TX symbols      */
/*                                            instead of using them       */
/*                                            directly,                   */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_transfer_request(UX_TRANSFER *transfer_request)
{
#if defined(UX_HOST_STANDALONE)
UINT        status;

    UX_TRANSFER_STATE_RESET(transfer_request);
    _ux_host_stack_transfer_run(transfer_request);
    if ((transfer_request -> ux_transfer_request_flags & UX_TRANSFER_FLAG_AUTO_WAIT))
    {
        while(1)
        {

            /* Allow tasks running during waiting.  */
            _ux_system_host_tasks_run();

            if (transfer_request -> ux_transfer_request_state <= UX_STATE_NEXT)
                break;
        }
        status = transfer_request -> ux_transfer_request_completion_code;
    }
    else
    {

        /* In this mode, transfer pending is a success started case.  */
        if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)
            status = UX_SUCCESS;
        else
            status = transfer_request -> ux_transfer_request_completion_code;
    }

    /* Return transfer completion status.  */
    return(status);
#else
UX_INTERRUPT_SAVE_AREA

UX_ENDPOINT     *endpoint;  
UX_DEVICE       *device;    
UX_HCD          *hcd;
UINT            status;
    

    /* Get the endpoint container from the transfer_request */
    endpoint =  transfer_request -> ux_transfer_request_endpoint;

    /* Get the device container from the endpoint.  */
    device =  endpoint -> ux_endpoint_device;

    /* Ensure we are not preempted by the enum thread while we check the device 
       state and set the transfer status.  */
    UX_DISABLE

    /* We can only transfer when the device is ATTACHED, ADDRESSED OR CONFIGURED.  */
    if ((device -> ux_device_state == UX_DEVICE_ATTACHED) || (device -> ux_device_state == UX_DEVICE_ADDRESSED)
            || (device -> ux_device_state == UX_DEVICE_CONFIGURED))
    {

        /* Set the transfer to pending.  */
        transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STATUS_PENDING;
#if !defined(UX_HOST_STANDALONE)
        /* Save the thread making this transfer. If we're under interrupt, this
           will be null.  */
        transfer_request -> ux_transfer_request_thread_pending =  _ux_utility_thread_identify();
#endif
    }
    else
    {

        /* The device is in an invalid state. Restore interrupts and return error.  */
        UX_RESTORE

        /* Check if this is endpoint 0.  */
        if ((endpoint -> ux_endpoint_descriptor.bEndpointAddress & (UINT)~UX_ENDPOINT_DIRECTION) == 0)
        {

            /* Check if the class has already protected it.  */
            if (!_ux_host_semaphore_waiting(&device -> ux_device_protection_semaphore))
            {

                /* Class is using endpoint 0. Unprotect semaphore.  */
                _ux_host_semaphore_put(&device -> ux_device_protection_semaphore);
            }
        }

        return(UX_TRANSFER_NOT_READY);
    }

    /* Restore interrupts.  */
    UX_RESTORE

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_TRANSFER_REQUEST, device, endpoint, transfer_request, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)
    
    /* With the device we have the pointer to the HCD.  */
    hcd = UX_DEVICE_HCD_GET(device);

    /* If this is endpoint 0, we protect the endpoint from a possible re-entry.  */
    if ((endpoint -> ux_endpoint_descriptor.bEndpointAddress & (UINT)~UX_ENDPOINT_DIRECTION) == 0)
    {

        /* Check if the class has already protected it.  */
        if (_ux_host_semaphore_waiting(&device -> ux_device_protection_semaphore))        
        {

            /* We are using endpoint 0. Protect with semaphore.  */
            status =  _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
    
            /* Check for status.  */
            if (status != UX_SUCCESS)
            
                /* Something went wrong. */
                return(status);
        }        
    }             
    
    /* Send the command to the controller.  */    
    status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_TRANSFER_REQUEST, transfer_request);

    /* If this is endpoint 0, we unprotect the endpoint. */
    if ((endpoint -> ux_endpoint_descriptor.bEndpointAddress & (UINT)~UX_ENDPOINT_DIRECTION) == 0)

        /* We are using endpoint 0. Unprotect with semaphore.  */
        _ux_host_semaphore_put(&device -> ux_device_protection_semaphore);

    /* And return the status.  */
    return(status);
#endif
}
