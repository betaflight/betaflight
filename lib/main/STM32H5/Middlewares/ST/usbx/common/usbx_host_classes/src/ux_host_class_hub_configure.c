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
/**   HUB Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hub.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hub_configure                        PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function calls the USBX stack to do a SET_CONFIGURATION to the */
/*    HUB. Once the HUB is configured, its interface will be activated    */ 
/*    and all the endpoints enumerated (1 interrupt endpoint in the case  */ 
/*    of the HUB).                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hub                                   Pointer to HUB                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_device_configuration_get Get device configuration    */ 
/*    _ux_host_stack_device_configuration_select                          */ 
/*                                          Select device configuration   */ 
/*    _ux_host_stack_configuration_interface_get                          */ 
/*                                          Get interface                 */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HUB Class                                                           */ 
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hub_configure(UX_HOST_CLASS_HUB *hub)
{

UINT                    status;
UX_CONFIGURATION        *configuration;
UX_DEVICE               *device;
UX_ENDPOINT             *control_endpoint;
UCHAR                   *device_status_data;
UX_TRANSFER             *transfer_request;
#if UX_MAX_DEVICES > 1
UX_DEVICE               *parent_device;
#endif


    /* A HUB normally has one configuration. So retrieve the 1st configuration
       only.  */
    _ux_host_stack_device_configuration_get(hub -> ux_host_class_hub_device, 0, &configuration);
        
    /* Get the device container for this configuration.  */
    device =  configuration -> ux_configuration_device;
    
    /* To find the true source of the HUB power source, we need to do a GET_STATUS of 
       the device.  */
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Allocate a buffer for the device status: 2 bytes.  */        
    device_status_data =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 2);
    if (device_status_data == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Create a transfer_request for the GET_STATUS request, 2 bytes are returned.  */
    transfer_request -> ux_transfer_request_requested_length =  2;
    transfer_request -> ux_transfer_request_data_pointer =      device_status_data;
    transfer_request -> ux_transfer_request_function =          UX_GET_STATUS;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             0;
    transfer_request -> ux_transfer_request_index =             0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check the status and the length of the data returned.  */
    if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == 2))
    {
        
        /* The data returned is good, now analyze power source.  */
        if (*device_status_data & UX_STATUS_DEVICE_SELF_POWERED)
            device -> ux_device_power_source =  UX_DEVICE_SELF_POWERED;
        else
            device -> ux_device_power_source =  UX_DEVICE_BUS_POWERED;

        /* Free the buffer resource now.  */
        _ux_utility_memory_free(device_status_data);
    }
    else
    {
    
        /* Free the buffer resource now.  */
        _ux_utility_memory_free(device_status_data);

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HUB, UX_CONNECTION_INCOMPATIBLE);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONNECTION_INCOMPATIBLE, hub, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Return an error.  */
        return(UX_CONNECTION_INCOMPATIBLE);
    }               

#if UX_MAX_DEVICES > 1
    /* Check the HUB power source and check the parent power source for 
       incompatible connections.  */
    if (hub -> ux_host_class_hub_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED)
    {
        
        /* Get the parent container for this device.  */
        parent_device =  device -> ux_device_parent;

        /* If the device is NULL, the parent is the root HUB and we don't have to worry 
           if the parent is not the root HUB, check for its power source.  */
        if ((parent_device != UX_NULL) && (parent_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED))
        {                        

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HUB, UX_CONNECTION_INCOMPATIBLE);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONNECTION_INCOMPATIBLE, hub, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_CONNECTION_INCOMPATIBLE);
        }            
    }
#endif

    /* We have the valid configuration. Ask the USBX stack to set this configuration.  */        
    _ux_host_stack_device_configuration_select(configuration);

    /* If the operation went well, the HUB default alternate setting for the HUB interface is 
       active and the interrupt endpoint is now enabled. We have to memorize the first interface 
       since the interrupt endpoint is hooked to it. */
    status =  _ux_host_stack_configuration_interface_get(configuration, 0, 0, &hub -> ux_host_class_hub_interface);

    /* Return completion status.  */
    return(status);
}

