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
/*    _ux_device_stack_get_status                         PORTABLE C      */
/*                                                           6.1.6        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function obtains the status of a USB component of the device   */
/*    such as device or endpoint.                                         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    request_type                          Request type                  */
/*    request_index                         Request index                 */
/*    request_length                        Request length                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Transfer request              */
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
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            supported bi-dir-endpoints, */
/*                                            resulting in version 6.1.6  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_get_status(ULONG request_type, ULONG request_index, ULONG request_length)
{

UX_SLAVE_DCD            *dcd;
UX_SLAVE_TRANSFER       *transfer_request;
UX_SLAVE_DEVICE         *device;
UX_SLAVE_ENDPOINT       *endpoint;
UINT                    status;
ULONG                   data_length;

    UX_PARAMETER_NOT_USED(request_length);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_GET_STATUS, request_type, request_index, request_length, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the control endpoint for the device.  */
    endpoint =  &device -> ux_slave_device_control_endpoint;

    /* Get the pointer to the transfer request associated with the endpoint.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Reset the status buffer.  */
    *transfer_request -> ux_slave_transfer_request_data_pointer =  0;
    *(transfer_request -> ux_slave_transfer_request_data_pointer + 1) =  0;
    
    /* The default length for GET_STATUS is 2, except for OTG get Status.  */
    data_length = 2;
    
    /* The status can be for either the device or the endpoint.  */
    switch (request_type & UX_REQUEST_TARGET)
    {
    
    case UX_REQUEST_TARGET_DEVICE:

        /* When the device is probed, it is either for the power/remote capabilities or OTG role swap.  
           We differentiate with the Windex, 0 or OTG status Selector.  */
        if (request_index == UX_OTG_STATUS_SELECTOR)
        {

            /* Set the data length to 1.  */
            data_length = 1;
            
#ifdef UX_OTG_SUPPORT
            /* Store the Role Swap flag.  */
            *transfer_request -> ux_slave_transfer_request_data_pointer =  (UCHAR) _ux_system_otg -> ux_system_otg_slave_role_swap_flag;
#endif
            
        }
        else
        {

            /* Store the current power state in the status buffer. */
            if (_ux_system_slave -> ux_system_slave_power_state == UX_DEVICE_SELF_POWERED)
                *transfer_request -> ux_slave_transfer_request_data_pointer =  1;

            /* Store the remote wakeup capability state in the status buffer.  */

            if (_ux_system_slave -> ux_system_slave_remote_wakeup_enabled)
                *transfer_request -> ux_slave_transfer_request_data_pointer |=  2;
        }
        
        break;
            
    case UX_REQUEST_TARGET_ENDPOINT:

#ifndef UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT

        /* This feature returns the halt state of a specific endpoint.  The endpoint index
           is used to retrieve the endpoint container.  */
        status =  dcd -> ux_slave_dcd_function(dcd, UX_DCD_ENDPOINT_STATUS, (VOID *)(ALIGN_TYPE)(request_index & (UINT)~UX_ENDPOINT_DIRECTION));
#else

        /* This feature returns the halt state of a specific endpoint.  The endpoint address
           is used to retrieve the endpoint container.  */
        status =  dcd -> ux_slave_dcd_function(dcd, UX_DCD_ENDPOINT_STATUS, (VOID *)(ALIGN_TYPE)(request_index));
#endif

        /* Check the status. We may have a unknown endpoint.  */
        if (status != UX_ERROR)
        {

            if (status == UX_TRUE)
                *transfer_request -> ux_slave_transfer_request_data_pointer =  1;
        }                        
        else
        {
    
            /* We stall the command. Endpoint is wrong.  */
            dcd -> ux_slave_dcd_function(dcd, UX_DCD_STALL_ENDPOINT, endpoint);
    
            /* No more work to do here.  The command failed but the upper layer does not depend on it.  */
            return(UX_SUCCESS);            
        }
        break;

    default:
        
        /* We stall the command.  */
        dcd -> ux_slave_dcd_function(dcd, UX_DCD_STALL_ENDPOINT, endpoint);
    
        /* No more work to do here.  The command failed but the upper layer does not depend on it.  */
        return(UX_SUCCESS);            
    }
    
    /* Set the phase of the transfer to data out.  */
    transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_OUT;

    /* Send the descriptor with the appropriate length to the host.  */
    status =  _ux_device_stack_transfer_request(transfer_request, data_length, data_length);

    /* Return the function status.  */
    return(status);
}

