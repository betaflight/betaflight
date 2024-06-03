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
/**   Prolific Class                                                      */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_prolific.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_prolific_reception_start             PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function starts a reception with the DLC modem. This mechanism */ 
/*    allows for non blocking calls based on a packet orientated round    */ 
/*    robbin buffer. When a packet is fully or partially received, an     */
/*    application callback function is invoked and a new transfer request */
/*    is rescheduled.                                                     */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    prolific                              Pointer to prolific class     */ 
/*    prolific_reception                    Pointer to reception struct   */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_prolific_reception_start (UX_HOST_CLASS_PROLIFIC *prolific, 
                                    UX_HOST_CLASS_PROLIFIC_RECEPTION *prolific_reception)
{

UX_TRANSFER     *transfer_request;
UINT            status;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_RECEPTION_START, prolific, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (prolific -> ux_host_class_prolific_state !=  UX_HOST_CLASS_INSTANCE_LIVE)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, prolific, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Start by aligning the head and tail of buffers to the same address supplied by the application.  */
    prolific_reception -> ux_host_class_prolific_reception_data_head =  prolific_reception -> ux_host_class_prolific_reception_data_buffer;
    prolific_reception -> ux_host_class_prolific_reception_data_tail =  prolific_reception -> ux_host_class_prolific_reception_data_buffer;

    /* Get the pointer to the bulk in endpoint in the transfer_request.  */
    transfer_request =  &prolific -> ux_host_class_prolific_bulk_in_endpoint -> ux_endpoint_transfer_request;
    
    /* Initialize the transfer request.  */
    transfer_request -> ux_transfer_request_class_instance      =  (VOID *) prolific;
    transfer_request -> ux_transfer_request_data_pointer        =  prolific_reception -> ux_host_class_prolific_reception_data_head;
    transfer_request -> ux_transfer_request_requested_length    =  prolific_reception -> ux_host_class_prolific_reception_block_size;
    transfer_request -> ux_transfer_request_completion_function =  _ux_host_class_prolific_reception_callback;
    
    /* Save the prolific reception structure in the prolific structure.  */
    prolific -> ux_host_class_prolific_reception = prolific_reception;
    
    /* And declare we have a transfer in progress.  */
    prolific_reception -> ux_host_class_prolific_reception_state =  UX_HOST_CLASS_PROLIFIC_RECEPTION_STATE_STARTED;
                    
    /* Arm a first transfer on the bulk in endpoint. There is a callback to this function so we return to the caller
       right away. */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* We do not know if the first transfer was successful yet.  If the status is not OK, we need to stop the transfer
       in progress flag. */
    if (status != UX_SUCCESS)
        prolific_reception -> ux_host_class_prolific_reception_state =  UX_HOST_CLASS_PROLIFIC_RECEPTION_STATE_STOPPED;
    
    return(status); 
}

