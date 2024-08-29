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
/*    _ux_device_stack_alternate_setting_set              PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function sets the alternate setting for a specific interface.  */
/*    The previous interface is unmounted and all the endpoints           */
/*    associated with the alternate setting are mounted.                  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    endpoint                              Pointer to endpoint           */
/*    interface_value                       Interface value               */
/*    alternate_setting_value               Alternate setting value       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_slave_dcd_function)               DCD dispatch function         */ 
/*    _ux_utility_descriptor_parse          Parse descriptor              */
/*    _ux_device_stack_transfer_all_request_abort                         */
/*                                          Abort transfer                */
/*    _ux_utility_memory_copy               Copy memory                   */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    Device Stack                                                        */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            optimized based on compile  */
/*                                            definitions, verified       */
/*                                            memset and memcpy cases,    */
/*                                            resulting in version 6.1    */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            calculated payload size,    */
/*                                            resulting in version 6.1.9  */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_alternate_setting_set(ULONG interface_value, ULONG alternate_setting_value)
{

UX_SLAVE_DEVICE                 *device;
UX_SLAVE_INTERFACE              *interface_ptr;
#if !defined(UX_DEVICE_ALTERNATE_SETTING_SUPPORT_DISABLE)
UX_SLAVE_DCD                    *dcd;
UX_SLAVE_TRANSFER               *transfer_request;
UCHAR                           *device_framework;
ULONG                           device_framework_length;
ULONG                           descriptor_length;
UCHAR                           descriptor_type;
UX_CONFIGURATION_DESCRIPTOR     configuration_descriptor;
UX_INTERFACE_DESCRIPTOR         interface_descriptor;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_ENDPOINT               *next_endpoint;
UX_SLAVE_ENDPOINT               *endpoint_link;
ULONG                            endpoints_pool_number;
UX_SLAVE_CLASS_COMMAND          class_command;
UX_SLAVE_CLASS                  *class_ptr;
UINT                            status;
ULONG                           max_transfer_length, n_trans;
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_ALTERNATE_SETTING_SET, interface_value, alternate_setting_value, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the device. */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Protocol error must be reported when it's unconfigured */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
        return(UX_FUNCTION_NOT_SUPPORTED);

    /* Find the current interface.  */
    interface_ptr =  device -> ux_slave_device_first_interface;

#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE) || UX_MAX_DEVICE_INTERFACES > 1
    /* Scan all interfaces if any. */
    while (interface_ptr != UX_NULL)
    {

        if (interface_ptr -> ux_slave_interface_descriptor.bInterfaceNumber == interface_value)
            break;
        else
            interface_ptr =  interface_ptr -> ux_slave_interface_next_interface;
    }
#else
    if (interface_ptr -> ux_slave_interface_descriptor.bInterfaceNumber != interface_value)
        interface_ptr = UX_NULL;
#endif

    /* We must have found the interface pointer for the interface value
       requested by the caller.  */
    if (interface_ptr == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_INTERFACE_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_INTERFACE_HANDLE_UNKNOWN, interface_ptr, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_INTERFACE_HANDLE_UNKNOWN);
    }

    /* If the host is requesting a change of alternate setting to the current one,
       we do not need to do any work.  */
    if (interface_ptr -> ux_slave_interface_descriptor.bAlternateSetting == alternate_setting_value)
        return(UX_SUCCESS);       

#if defined(UX_DEVICE_ALTERNATE_SETTING_SUPPORT_DISABLE)

    /* If alternate setting is disabled, do error trap.  */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FUNCTION_NOT_SUPPORTED);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, interface_ptr, 0, 0, UX_TRACE_ERRORS, 0, 0)

    return(UX_FUNCTION_NOT_SUPPORTED);
#else

    /* Get the pointer to the DCD. */
    dcd =  &_ux_system_slave->ux_system_slave_dcd;

    /* We may have multiple configurations!  */
    device_framework =  _ux_system_slave -> ux_system_slave_device_framework;
    device_framework_length =  _ux_system_slave -> ux_system_slave_device_framework_length;

    /* Parse the device framework and locate a configuration descriptor. */
    while (device_framework_length != 0)
    {

        /* Get the length of the current descriptor.  */
        descriptor_length =  (ULONG) *device_framework;

        /* And its length.  */
        descriptor_type =*  (device_framework + 1);
                
        /* Check if this is a configuration descriptor. */
        if (descriptor_type == UX_CONFIGURATION_DESCRIPTOR_ITEM)
        {

            /* Parse the descriptor in something more readable. */
            _ux_utility_descriptor_parse(device_framework,
                        _ux_system_configuration_descriptor_structure,
                        UX_CONFIGURATION_DESCRIPTOR_ENTRIES,
                        (UCHAR *) &configuration_descriptor);

            /* Now we need to check the configuration value.  */
            if (configuration_descriptor.bConfigurationValue == device -> ux_slave_device_configuration_selected)
            {

                /* Limit the search in current configuration descriptor. */
                device_framework_length = configuration_descriptor.wTotalLength;

                /* We have found the configuration value that was selected by the host   
                   We need to scan all the interface descriptors following this
                   configuration descriptor and locate the interface for which the alternate
                   setting must be changed. */
                while (device_framework_length != 0)
                {

                    /* Get the length of the current descriptor.  */
                    descriptor_length =  (ULONG) *device_framework;

                    /* And its type.  */
                    descriptor_type = *(device_framework + 1); 
                
                    /* Check if this is an interface descriptor. */
                    if (descriptor_type == UX_INTERFACE_DESCRIPTOR_ITEM)
                    {

                        /* Parse the descriptor in something more readable. */
                        _ux_utility_descriptor_parse(device_framework,
                                    _ux_system_interface_descriptor_structure,
                                    UX_INTERFACE_DESCRIPTOR_ENTRIES,
                                    (UCHAR *) &interface_descriptor);

                        /* Check if this is the interface we are searching. */
                        if (interface_descriptor.bInterfaceNumber == interface_value &&
                            interface_descriptor.bAlternateSetting == alternate_setting_value)
                        {

                            /* We have found the right interface and alternate setting. Before
                               we mount all the endpoints for this interface, we need to
                               unmount the endpoints associated with the previous alternate setting.  */
                            endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;
                            while (endpoint != UX_NULL)
                            {

                                /* Abort any pending transfer.  */
                                _ux_device_stack_transfer_all_request_abort(endpoint, UX_TRANSFER_BUS_RESET);

                                /* The device controller must be called to destroy the endpoint.  */
                                dcd -> ux_slave_dcd_function(dcd, UX_DCD_DESTROY_ENDPOINT, (VOID *) endpoint);

                                /* Get the next endpoint.  */
                                next_endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
                
                                /* Free the endpoint.  */
                                endpoint -> ux_slave_endpoint_status =  UX_UNUSED;
                        
                                /* Make sure the endpoint instance is now cleaned up.  */
                                endpoint -> ux_slave_endpoint_state =  0;
                                endpoint -> ux_slave_endpoint_next_endpoint =  UX_NULL;
                                endpoint -> ux_slave_endpoint_interface =  UX_NULL;
                                endpoint -> ux_slave_endpoint_device =  UX_NULL;
                                                        
                                /* Now we refresh the endpoint pointer.  */
                                endpoint =  next_endpoint;
                            }

                            /* Now clear the interface endpoint entry.  */
                            interface_ptr -> ux_slave_interface_first_endpoint = UX_NULL;

                            /* Point beyond the interface descriptor.  */
                            device_framework_length -=  (ULONG) *device_framework;
                            device_framework +=  (ULONG) *device_framework;
                        
                            /* Parse the device framework and locate endpoint descriptor(s).  */
                            while (device_framework_length != 0)
                            {
                        
                                /* Get the length of the current descriptor.  */
                                descriptor_length =  (ULONG) *device_framework;
                        
                                /* And its type.  */
                                descriptor_type =  *(device_framework + 1);
                                        
                                /* Check if this is an endpoint descriptor.  */
                                switch(descriptor_type)
                                {
                        
                                case UX_ENDPOINT_DESCRIPTOR_ITEM:
                        
                                    /* Find a free endpoint in the pool and hook it to the 
                                       existing interface after it's created by DCD.  */
                                    endpoint = device -> ux_slave_device_endpoints_pool;
                                    endpoints_pool_number = device -> ux_slave_device_endpoints_pool_number;
                                    while (endpoints_pool_number != 0)
                                    {
                                        /* Check if this endpoint is free.  */
                                        if (endpoint ->    ux_slave_endpoint_status == UX_UNUSED)
                                        {
                                            /* Mark this endpoint as used now.  */
                                            endpoint ->    ux_slave_endpoint_status = UX_USED;
                                            break;
                                        }
                                    
                                        /* Try the next endpoint.  */
                                        endpoint++;
                                        
                                        /* Decrement the number of endpoints to scan from the pool.  */
                                       endpoints_pool_number--; 
                                    }
                        
                                    /* Did we find a free endpoint ?  */
                                    if (endpoints_pool_number == 0)
                                        return(UX_MEMORY_INSUFFICIENT);

                                    /* Parse the descriptor in something more readable.  */
                                    _ux_utility_descriptor_parse(device_framework,
                                                    _ux_system_endpoint_descriptor_structure,
                                                    UX_ENDPOINT_DESCRIPTOR_ENTRIES,
                                                    (UCHAR *) &endpoint -> ux_slave_endpoint_descriptor);

                                    /* Now we create a transfer request to accept transfer on this endpoint.  */
                                    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;
                                        
                                    /* Validate descriptor wMaxPacketSize.  */
                                    UX_ASSERT(endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize != 0);

                                    /* Calculate endpoint transfer payload max size.  */
                                    max_transfer_length =
                                            endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize &
                                                                                UX_MAX_PACKET_SIZE_MASK;
                                    if ((_ux_system_slave -> ux_system_slave_speed == UX_HIGH_SPEED_DEVICE) &&
                                        (endpoint -> ux_slave_endpoint_descriptor.bmAttributes & 0x1u))
                                    {
                                        n_trans = endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize &
                                                                    UX_MAX_NUMBER_OF_TRANSACTIONS_MASK;
                                        if (n_trans)
                                        {
                                            n_trans >>= UX_MAX_NUMBER_OF_TRANSACTIONS_SHIFT;
                                            n_trans ++;
                                            max_transfer_length *= n_trans;
                                        }
                                    }

                                    /* Validate max transfer size and save it.  */
                                    UX_ASSERT(max_transfer_length <= UX_SLAVE_REQUEST_DATA_MAX_LENGTH);
                                    transfer_request -> ux_slave_transfer_request_transfer_length = max_transfer_length;

                                    /* We store the endpoint in the transfer request as well.  */
                                    transfer_request -> ux_slave_transfer_request_endpoint =  endpoint;
                                        
                                    /* By default the timeout is infinite on request.  */
                                    transfer_request -> ux_slave_transfer_request_timeout = UX_WAIT_FOREVER;
                                    
                                    /* Attach the interface to the endpoint.  */
                                    endpoint -> ux_slave_endpoint_interface =  interface_ptr;
                                        
                                    /* Attach the device to the endpoint.  */
                                    endpoint -> ux_slave_endpoint_device =  device;

                                    /* Create the endpoint at the DCD level.  */
                                    status =  dcd -> ux_slave_dcd_function(dcd, UX_DCD_CREATE_ENDPOINT, (VOID *) endpoint); 

                                    /* Do a sanity check on endpoint creation.  */
                                    if (status != UX_SUCCESS)
                                    {

                                        /* Error was returned, endpoint cannot be created.  */
                                        endpoint -> ux_slave_endpoint_status = UX_UNUSED;
                                        return(status);
                                    }

                                    /* Attach this endpoint to the end of the endpoint chain.  */
                                    if (interface_ptr -> ux_slave_interface_first_endpoint == UX_NULL)
                                    {
                        
                                        interface_ptr -> ux_slave_interface_first_endpoint =  endpoint;
                                    }
                                    else
                                    {
                                        /* Multiple endpoints exist, so find the end of the chain.  */
                                        endpoint_link =  interface_ptr -> ux_slave_interface_first_endpoint;
                                        while (endpoint_link -> ux_slave_endpoint_next_endpoint != UX_NULL)
                                            endpoint_link =  endpoint_link -> ux_slave_endpoint_next_endpoint;
                                        endpoint_link -> ux_slave_endpoint_next_endpoint =  endpoint;
                                    }
                        
                                    break;
                        
                                case UX_CONFIGURATION_DESCRIPTOR_ITEM:
                                case UX_INTERFACE_DESCRIPTOR_ITEM:
                        
                                    /* We have found a new configuration or interface descriptor, this is the end of the current
                                       interface. The search for the endpoints must be terminated as if it was the end of the 
                                       entire descriptor.  */
                                    device_framework_length =  descriptor_length;
                                       
                                    break;


                                default:
                                
                                    /* We have found another descriptor embedded in the interface. Ignore it.  */
                                    break;
                                }
                        
                                /* Adjust what is left of the device framework.  */
                                device_framework_length -=  descriptor_length;
                        
                                /* Point to the next descriptor.  */
                                device_framework +=  descriptor_length;
                            }

                            /* The interface descriptor in the current class must be changed to the new alternate setting.  */
                            _ux_utility_memory_copy(&interface_ptr -> ux_slave_interface_descriptor, &interface_descriptor, sizeof(UX_INTERFACE_DESCRIPTOR)); /* Use case of memcpy is verified. */
                            
                            /* Get the class for the interface.  */
                            class_ptr =  _ux_system_slave -> ux_system_slave_interface_class_array[interface_ptr -> ux_slave_interface_descriptor.bInterfaceNumber];

                            /* Check if class driver is available. */
                            if (class_ptr == UX_NULL || class_ptr -> ux_slave_class_status == UX_UNUSED)
                            {

                                return (UX_NO_CLASS_MATCH);
                            }
                        
                            /* The interface attached to this configuration must be changed at the class
                               level.  */
                            class_command.ux_slave_class_command_request   =    UX_SLAVE_CLASS_COMMAND_CHANGE;
                            class_command.ux_slave_class_command_interface =   (VOID *) interface_ptr;

                            /* And store it.  */
                            class_command.ux_slave_class_command_class_ptr =  class_ptr;
                            
                            /* We can now memorize the interface pointer associated with this class.  */
                            class_ptr -> ux_slave_class_interface = interface_ptr;
                            
                            /* We have found a potential candidate. Call this registered class entry function to change the alternate setting.  */
                            status = class_ptr -> ux_slave_class_entry_function(&class_command);

                            /* We are done here.  */
                            return(status); 
                        }
                    }               

                    /* Adjust what is left of the device framework.  */
                    device_framework_length -=  descriptor_length;

                    /* Point to the next descriptor.  */
                    device_framework +=  descriptor_length;
                }

                /* In case alter setting not found, report protocol error. */
                break;
            }
        }

        /* Adjust what is left of the device framework.  */
        device_framework_length -=  descriptor_length;

        /* Point to the next descriptor.  */
        device_framework +=  descriptor_length;
    }

    /* Return error completion.  */
    return(UX_ERROR);
#endif
}

