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
/*    _ux_device_stack_set_feature                        PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function sets a specific feature (Device, Interface,           */
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
/*    (ux_slave_dcd_function)               DCD controller function       */ 
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
/*                                            definitions, stalled on not */
/*                                            supported device requests,  */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_set_feature(ULONG request_type, ULONG request_value, ULONG request_index)
{

UX_SLAVE_DCD            *dcd;
UX_SLAVE_DEVICE         *device;
UX_SLAVE_INTERFACE      *interface_ptr;
UX_SLAVE_ENDPOINT       *endpoint;
UX_SLAVE_ENDPOINT       *endpoint_target;

    UX_PARAMETER_NOT_USED(request_value);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_SET_FEATURE, request_value, request_index, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the control endpoint for the device.  */
    endpoint =  &device -> ux_slave_device_control_endpoint;

    /* The feature can be for either the device or the endpoint.  */
    switch (request_type & UX_REQUEST_TARGET)
    {
    
    case UX_REQUEST_TARGET_DEVICE:

        /* Check if we have a DEVICE_REMOTE_WAKEUP Feature.  */
        if (request_value == UX_REQUEST_FEATURE_DEVICE_REMOTE_WAKEUP)
        {

            /* Check if we have the capability. */
            if (_ux_system_slave -> ux_system_slave_remote_wakeup_capability)
            {

                /* Enable the feature. */
                _ux_system_slave -> ux_system_slave_remote_wakeup_enabled = UX_TRUE;

                /* OK. */
                return (UX_SUCCESS);
            }
            else

                /* Protocol error. */
                return (UX_FUNCTION_NOT_SUPPORTED);
        }

#ifdef UX_OTG_SUPPORT
        /* Check if we have a A_HNP_SUPPORT Feature. This is set when the Host is HNP capable. */
        if (request_value == UX_OTG_FEATURE_A_HNP_SUPPORT)
        {

            /* Store the A_HNP_SUPPORT flag.  */
            _ux_system_otg -> ux_system_otg_slave_set_feature_flag |= UX_OTG_FEATURE_A_HNP_SUPPORT;

            /* OK.  */
            return(UX_SUCCESS);
        }

        /* Check if the host asks us to perform HNP.  If also we become the host.  */
        if (request_value == UX_OTG_FEATURE_B_HNP_ENABLE)
        {

            /* The ISR will pick up the suspend event and check if we need to become IDLE or HOST.  */
            _ux_system_otg -> ux_system_otg_slave_set_feature_flag |= UX_OTG_FEATURE_B_HNP_ENABLE;

            /* OK.  */
            return(UX_SUCCESS);
        }
#endif

        /* Request value not supported.  */
        return(UX_FUNCTION_NOT_SUPPORTED);

    case UX_REQUEST_TARGET_ENDPOINT:

        /* The only set feature for endpoint is ENDPOINT_STALL. This forces
           the endpoint to the stall situation.
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

                    /* Stall the endpoint.  */
                    dcd -> ux_slave_dcd_function(dcd, UX_DCD_STALL_ENDPOINT, endpoint_target);

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

        /* We get here when the endpoint is wrong. Should not happen though.  */
        /* Intentionally fall through into the default case. */
        /* fall through */
    default:
        
        /* We stall the command.  */
        dcd -> ux_slave_dcd_function(dcd, UX_DCD_STALL_ENDPOINT, endpoint);
    
        /* No more work to do here.  The command failed but the upper layer does not depend on it.  */
        return(UX_SUCCESS);            
    }
}
