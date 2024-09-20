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
/**   ACM CDC Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_acm_reception_stop               PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function stops background reception previously started by      */ 
/*    ux_host_class_cdc_acm_reception_start.                              */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_acm                               Pointer to cdc_acm class      */ 
/*    cdc_acm_reception                     Pointer to reception struct   */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_endpoint_transfer_abort                              */
/*                                          Abort transfer                */
/*    _ux_host_semaphore_get                Get semaphore                 */
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_cdc_acm_reception_stop(UX_HOST_CLASS_CDC_ACM *cdc_acm, 
                                    UX_HOST_CLASS_CDC_ACM_RECEPTION *cdc_acm_reception)
{

UX_TRANSFER              *transfer_request;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_CDC_ACM_RECEPTION_STOP, cdc_acm, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (cdc_acm -> ux_host_class_cdc_acm_state !=  UX_HOST_CLASS_INSTANCE_LIVE)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, cdc_acm, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* As further protection, we must ensure this instance of the interface is the data interface and not
       the control interface !  */
    if (cdc_acm -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bInterfaceClass != UX_HOST_CLASS_CDC_DATA_CLASS)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, cdc_acm, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }
    
    /* Check if we do have transfers for this application. If none, nothing to do. */
    if (cdc_acm_reception -> ux_host_class_cdc_acm_reception_state ==  UX_HOST_CLASS_CDC_ACM_RECEPTION_STATE_STOPPED)
        return(UX_SUCCESS);
        
    /* We need to abort transactions on the bulk In pipe.  */
    _ux_host_stack_endpoint_transfer_abort(cdc_acm -> ux_host_class_cdc_acm_bulk_in_endpoint);

    /* Declare the reception stopped.  */
    cdc_acm_reception -> ux_host_class_cdc_acm_reception_state =  UX_HOST_CLASS_CDC_ACM_RECEPTION_STATE_STOPPED;

    /* Obtain pointer to transfer request.  */
    transfer_request = &cdc_acm -> ux_host_class_cdc_acm_bulk_in_endpoint -> ux_endpoint_transfer_request;

    /* Reset the completion callback function.  */
    transfer_request -> ux_transfer_request_completion_function = UX_NULL;

#if !defined(UX_HOST_STANDALONE)

    /* Clear semaphore counts that were (incorrectly) increased during each transfer 
       completion.  */
    while (transfer_request -> ux_transfer_request_semaphore.tx_semaphore_count)
        _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, 0);
#endif

    /* This function never really fails.  */
    return(UX_SUCCESS);
}

