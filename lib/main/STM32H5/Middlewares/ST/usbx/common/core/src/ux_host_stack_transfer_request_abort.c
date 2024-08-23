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
/*    _ux_host_stack_transfer_request_abort               PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function aborts a pending transfer request that has been       */ 
/*    previously submitted. This function only cancels the specific       */ 
/*    transfer request.                                                   */
/*                                                                        */
/*    The call back to the function will have the                         */ 
/*    UX_TRANSFER_STATUS_ABORT status.                                    */  
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    transfer_request                      Transfer request structure    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                     If UX_SUCCESS, transfer was   */ 
/*                                            successfully aborted        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    HCD Entry Function                                                  */ 
/*    Transfer Completion Function                                        */ 
/*    _ux_utility_semaphore_put             Put semaphore                 */
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
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*  06-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed trace enabled error,  */
/*                                            resulting in version 6.1.7  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_transfer_request_abort(UX_TRANSFER *transfer_request)
{

UX_HCD          *hcd;
ULONG           completion_code;


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_TRANSFER_REQUEST_ABORT,
        transfer_request -> ux_transfer_request_endpoint -> ux_endpoint_device,
        transfer_request -> ux_transfer_request_endpoint,
        transfer_request, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)
    
    /* With the device we have the pointer to the HCD.  */
    hcd = UX_DEVICE_HCD_GET(transfer_request -> ux_transfer_request_endpoint -> ux_endpoint_device);

    /* Check pending transaction.  */
    if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)
    {

        /* Send the abort command to the controller.  */    
        hcd -> ux_hcd_entry_function(hcd, UX_HCD_TRANSFER_ABORT, transfer_request);

        /* Save the completion code since we're about to set it to ABORT. The
           reason we can't just assume its value is PENDING is that in between
           the completion code check and this line, it's possible that the transfer
           completed (by HCD function call below, or ISR), which would've
           changed the completion code to SUCCESS and put the semaphore.
           Even it's recommended to keep completion code untouched to let things
           changed later here.
           Such a case is valid, and we want to make sure we don't put() the
           transfer request's semaphore again.  */
        completion_code =  transfer_request -> ux_transfer_request_completion_code;

        /* Set the transfer_request status to abort.  */
        transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STATUS_ABORT;

        /* We need to inform the class that owns this transfer_request of the 
           abort if there is a call back mechanism.  */
        if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
            transfer_request -> ux_transfer_request_completion_function(transfer_request);
       
        /* Is a thread waiting on the semaphore?  */
        if (/* Is the transfer pending?  */
            completion_code == UX_TRANSFER_STATUS_PENDING &&
#if !defined(UX_HOST_STANDALONE)
            /* Is the thread waiting not this one? (clearly we're not waiting!)  */
            transfer_request -> ux_transfer_request_thread_pending != _ux_utility_thread_identify() && 
#endif
            /* Does the transfer request not have a completion function?  */
            transfer_request -> ux_transfer_request_completion_function == UX_NULL)

            /* Wake up the semaphore for this request.  */
            _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);
    }
    
    /* This function never fails!  */
    return(UX_SUCCESS);       
}

