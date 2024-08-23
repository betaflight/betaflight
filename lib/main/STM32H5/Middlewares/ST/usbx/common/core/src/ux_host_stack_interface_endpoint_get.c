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
/*    _ux_host_stack_interface_endpoint_get               PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns an endpoint container based on the interface  */
/*    handle and an endpoint index.                                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    interface                             Pointer to interface          */ 
/*    endpoint_index                        Index of endpoint to get      */ 
/*    endpoint                              Destination for endpoint      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
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
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_interface_endpoint_get(UX_INTERFACE *interface_ptr, UINT endpoint_index, UX_ENDPOINT **endpoint)
{

UINT            current_endpoint_index;
UX_ENDPOINT     *current_endpoint;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_INTERFACE_ENDPOINT_GET, interface_ptr, endpoint_index, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Do a sanity check on the interface handle.  */
    if (interface_ptr -> ux_interface_handle != (ULONG) (ALIGN_TYPE) interface_ptr)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_INTERFACE_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_INTERFACE_HANDLE_UNKNOWN, interface_ptr, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_INTERFACE_HANDLE_UNKNOWN);
    }
            
    /* Start with the endpoint attached to the interface.  */
    current_endpoint =  interface_ptr -> ux_interface_first_endpoint;

    /* The first endpoint has the index 0.  */    
    current_endpoint_index =  0;
    
    /* Traverse the list of the endpoints until we found the right one.  */        
    while (current_endpoint != UX_NULL)
    {

        /* Check if the endpoint index matches the current one.  */
        if (endpoint_index == current_endpoint_index)
        {

            /* Setup the return endpoint pointer.  */
            *endpoint=current_endpoint;

            /* Return success to the caller.  */
            return(UX_SUCCESS);
        }
        
        /* Move to the next endpoint.  */
        current_endpoint =  current_endpoint -> ux_endpoint_next_endpoint;
        
        /* Move to the next index.  */
        current_endpoint_index++;
    }

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_ENDPOINT_HANDLE_UNKNOWN);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, endpoint, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Return an error!  */
    return(UX_ENDPOINT_HANDLE_UNKNOWN);
}

