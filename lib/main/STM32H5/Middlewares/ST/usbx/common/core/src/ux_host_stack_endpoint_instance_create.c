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
/*    _ux_host_stack_endpoint_instance_create             PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will create an endpoint instance. The HCD layer is    */
/*    invoked to create each endpoint and the bandwidth claimed by each   */
/*    endpoint is allocated.                                              */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    endpoint                              Endpoint to delete            */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_bandwidth_check        Check bandwidth               */ 
/*    _ux_host_stack_bandwidth_claim        Claim bandwidth               */ 
/*    _ux_utility_semaphore_create          Semaphore create              */ 
/*    (ux_hcd_entry_function)               HCD entry function            */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
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
/*                                            filled default endpoint     */
/*                                            request endpoint pointer,   */
/*                                            resulting in version 6.1.7  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_endpoint_instance_create(UX_ENDPOINT *endpoint)
{

UX_HCD          *hcd;
UINT            status;
UCHAR           endpoint_type;


    /* Obtain the HCD for this endpoint.  */
    hcd = UX_DEVICE_HCD_GET(endpoint -> ux_endpoint_device);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_ENDPOINT_INSTANCE_CREATE, endpoint -> ux_endpoint_device, endpoint, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    
    /* If the endpoint needs guaranteed bandwidth, check if we have enough */
    endpoint_type = (endpoint -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE;
    switch (endpoint_type)
    {

    case UX_CONTROL_ENDPOINT:
    case UX_BULK_ENDPOINT:
        
        break;

    default:

        /* Check the bandwidth for this endpoint */
        if (_ux_host_stack_bandwidth_check(hcd, endpoint) != UX_SUCCESS)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_NO_BANDWIDTH_AVAILABLE);

            return(UX_NO_BANDWIDTH_AVAILABLE);
        }


        break;
    }

    /* Create this endpoint.  */
    status = hcd -> ux_hcd_entry_function(hcd, UX_HCD_CREATE_ENDPOINT, (VOID *) endpoint);

    /* Check status.  */
    if (status != UX_SUCCESS)
    {

        /* Return completion status.  */
        return(status);
    }

    /* Claim bandwidth if needed.  */
    if ((endpoint_type == UX_INTERRUPT_ENDPOINT) || (endpoint_type == UX_ISOCHRONOUS_ENDPOINT))
    {

        /* Claim its bandwidth */
        _ux_host_stack_bandwidth_claim(hcd, endpoint);
    }

    /* Create a semaphore for this endpoint to be attached to its transfer request.  */
    status =  _ux_host_semaphore_create(&endpoint -> ux_endpoint_transfer_request.ux_transfer_request_semaphore,
                                                                "ux_transfer_request_semaphore", 0);

    /* Check status.  */
    if (status == UX_SUCCESS)
    {

        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_ENDPOINT, endpoint, 0, 0, 0)

        /* By default transfer request contained is for endpoint itself.  */
        endpoint -> ux_endpoint_transfer_request.ux_transfer_request_endpoint = endpoint;
    }

    /* Return completion status.  */
    return(status);
}

