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
/*    _ux_device_stack_clear_feature                      PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function clears a specific feature (Device, Interface,         */
/*    Endpoint ....).                                                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    request_type                          Request type                  */ 
/*    request_value                         Request value                 */
/*    request_index                         Request index                 */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_slave_dcd_function)               DCD dispatch function         */ 
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_clear_feature(ULONG request_type, ULONG request_value, ULONG request_index)
{

UX_SLAVE_DCD            *dcd;
UX_SLAVE_DEVICE         *device;
UX_SLAVE_INTERFACE      *interface_ptr;
UX_SLAVE_ENDPOINT       *endpoint;
UX_SLAVE_ENDPOINT       *endpoint_target;
                                
    UX_PARAMETER_NOT_USED(request_value);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_CLEAR_FEATURE, request_type, request_value, request_index, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the control endpoint for the device.  */
    endpoint =  &device -> ux_slave_device_control_endpoint;

    /* The request can be for either the device or the endpoint.  */
    switch (request_type & UX_REQUEST_TARGET)
    {
    
    case UX_REQUEST_TARGET_DEVICE:

        /* Check if we have a DEVICE_REMOTE_WAKEUP Feature.  */
        if (request_value == UX_REQUEST_FEATURE_DEVICE_REMOTE_WAKEUP)
        {

            /* Check if we have the capability. */
            if (_ux_system_slave -> ux_system_slave_remote_wakeup_capability)
            {

                /* Disable the feature. */
                _ux_system_slave -> ux_system_slave_remote_wakeup_enabled = UX_FALSE;
            }

            else

                /* Protocol error. */
                return (UX_FUNCTION_NOT_SUPPORTED);
        }

        break;
            
    case UX_REQUEST_TARGET_ENDPOINT:

        /* The only clear feature for endpoint is ENDPOINT_STALL. This clears
           the endpoint of the stall situation and resets its data toggle. 
           We need to find the endpoint through the interface(s). */
        interface_ptr =  device -> ux_slave_device_first_interface;

#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE) || UX_MAX_DEVICE_INTERFACES > 1
        while (interface_ptr != UX_NULL)
        {
#endif

            /* Get the first endpoint for this interface.  */
            endpoint_target =  interface_ptr -> ux_slave_interface_first_endpoint;
                
            /* Parse all the endpoints.  */
            while (endpoint_target != UX_NULL)
            {

                /* Check the endpoint index.  */
                if (endpoint_target -> ux_slave_endpoint_descriptor.bEndpointAddress == request_index)
                {

                    /* Reset the endpoint.  */
                    dcd -> ux_slave_dcd_function(dcd, UX_DCD_RESET_ENDPOINT, endpoint_target);
                    
                    /* Mark its state now.  */
                    endpoint_target -> ux_slave_endpoint_state = UX_ENDPOINT_RESET;

                    /* Return the function status.  */
                    return(UX_SUCCESS);
                }

                /* Next endpoint.  */
                endpoint_target =  endpoint_target -> ux_slave_endpoint_next_endpoint;
            }

#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE) || UX_MAX_DEVICE_INTERFACES > 1
            /* Next interface.  */
            interface_ptr =  interface_ptr -> ux_slave_interface_next_interface;
        }
#endif

        /* Intentional fallthrough and go into the default case. */
        /* fall through */

    /* We get here when the endpoint is wrong. Should not happen though.  */
    default:
        
        /* We stall the command.  */
        dcd -> ux_slave_dcd_function(dcd, UX_DCD_STALL_ENDPOINT, endpoint);
    
        /* No more work to do here.  The command failed but the upper layer does not depend on it.  */
        return(UX_SUCCESS);            
    }

    /* Return the function status.  */
    return(UX_SUCCESS);
}

