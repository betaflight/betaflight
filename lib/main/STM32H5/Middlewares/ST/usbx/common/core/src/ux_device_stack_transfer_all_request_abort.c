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
/*    _ux_device_stack_transfer_all_request_abort         PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function cancels all the transfer requests attached to an      */
/*    endpoint. The endpoint is not reset and its toggle state is left    */
/*    the same.                                                           */
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
/*    _ux_device_stack_transfer_abort       Transfer abort                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Stack                                                        */
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
UINT  _ux_device_stack_transfer_all_request_abort(UX_SLAVE_ENDPOINT *endpoint, ULONG completion_code)
{

UX_SLAVE_TRANSFER       *transfer_request;    

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_TRANSFER_ALL_REQUEST_ABORT, endpoint, completion_code, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the transfer request for this endpoint.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;
    
    /* Abort this request.  */
    _ux_device_stack_transfer_abort(transfer_request, completion_code);

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

