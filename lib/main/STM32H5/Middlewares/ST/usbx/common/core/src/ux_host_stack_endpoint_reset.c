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
/*    _ux_host_stack_endpoint_reset                       PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function resets an endpoint after a stall or other error       */ 
/*    condition.                                                          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    endpoint                              Endpoint to abort transfer    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Send transfer request         */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_endpoint_reset(UX_ENDPOINT *endpoint)
{

UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UX_DEVICE       *device;    
UX_HCD          *hcd;
UINT            status;


    /* Get the device container from the endpoint */
    device =  endpoint -> ux_endpoint_device;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_ENDPOINT_RESET, device, endpoint, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Get the control endpoint attached to the device.  */
    control_endpoint =  &device -> ux_device_control_endpoint;

    /* Retrieve the transfer_request pointer.  */
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Create a transfer_request for the CLEAR_FEATURE request.  */
    transfer_request -> ux_transfer_request_data_pointer =      UX_NULL;
    transfer_request -> ux_transfer_request_requested_length =  0;
    transfer_request -> ux_transfer_request_function =          UX_CLEAR_FEATURE;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT| UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_ENDPOINT;
    transfer_request -> ux_transfer_request_value =             UX_ENDPOINT_HALT;
    transfer_request -> ux_transfer_request_index =             endpoint -> ux_endpoint_descriptor.bEndpointAddress;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Reset the endpoint at the HCD level.  */
    if (status == UX_SUCCESS)
    {

        /* Pickup HCD pointer.  */
        hcd = UX_DEVICE_HCD_GET(device);

        /* Call HCD entry function.  */
        status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_RESET_ENDPOINT, endpoint);
    }

    /* And return the completion status.  */
    return(status);
}

