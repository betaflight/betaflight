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
/**   Device Stack                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_stack_transfer_abort                     PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function aborts a pending transfer request that has been       */
/*    previously submitted. This function only cancels a specific         */
/*    transfer request.                                                   */
/*                                                                        */
/*    The call back to the function will have the                         */
/*    UX_TRANSFER_STATUS_ABORT status                                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    transfer_request                      Pointer to transfer request   */
/*    completion_code                       Completion code               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_semaphore_put             Put semaphore                 */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    Device Stack                                                        */
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
/*                                            added standalone support,   */
/*                                            assigned aborting code,     */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_transfer_abort(UX_SLAVE_TRANSFER *transfer_request, ULONG completion_code)
{

UX_INTERRUPT_SAVE_AREA

UX_SLAVE_DCD    *dcd;

    UX_PARAMETER_NOT_USED(completion_code);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_TRANSFER_ABORT, transfer_request, completion_code, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Sets the completion code due to bus reset.  */
    transfer_request -> ux_slave_transfer_request_completion_code = completion_code;

    /* Ensure we're not preempted by the transfer completion ISR.  */
    UX_DISABLE

    /* It's possible the transfer already completed. Ensure it hasn't before doing the abort.  */
    if (transfer_request -> ux_slave_transfer_request_status == UX_TRANSFER_STATUS_PENDING)
    {

        /* Call the DCD if necessary for cleaning up the pending transfer.  */
        dcd -> ux_slave_dcd_function(dcd, UX_DCD_TRANSFER_ABORT, (VOID *) transfer_request);

        /* Restore interrupts. Note that the transfer request should not be modified now.  */
        UX_RESTORE

        /* We need to set the completion code for the transfer to aborted. Note
           that the transfer request function cannot simultaneously modify this 
           because if the transfer was pending, then the transfer's thread is 
           currently waiting for it to complete.  */
        transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_ABORT;

        /* Wake up the device driver who is waiting on the semaphore.  */
        _ux_device_semaphore_put(&transfer_request -> ux_slave_transfer_request_semaphore);
    }
    else
    {

        /* Restore interrupts.  */
        UX_RESTORE
    }

    /* This function never fails.  */
    return(UX_SUCCESS);       
}

