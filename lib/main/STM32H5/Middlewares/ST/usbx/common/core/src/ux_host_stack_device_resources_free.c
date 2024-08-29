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
/*    _ux_host_stack_device_resources_free                PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will free all the device resources allocated.         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    device                                Device pointer                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_endpoint_transfer_abort                              */
/*                                          Abort transfer                */
/*    _ux_host_stack_endpoint_instance_delete                             */
/*                                          Delete endpoint instance      */ 
/*    _ux_utility_memory_free               Free memory block             */ 
/*    _ux_utility_memory_set                Set memory with a value       */ 
/*    _ux_utility_semaphore_delete          Semaphore delete              */ 
/*    _ux_utility_thread_schedule_other     Sleep thread to let others    */
/*                                          run                           */
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
/*                                            definitions, verified       */
/*                                            memset and memcpy cases,    */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            fixed standalone enum free, */
/*                                            freed shared device config  */
/*                                            descriptor for enum scan,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_device_resources_free(UX_DEVICE *device)
{

UX_CONFIGURATION        *configuration;
UX_INTERFACE            *interface_ptr;
UX_ENDPOINT             *endpoint;
VOID                    *container;
ULONG                   current_alternate_setting;
UX_HCD                  *hcd;
#if UX_MAX_DEVICES > 1
UINT                    device_address_byte_index;
UINT                    device_address_bit_index;
UCHAR                   device_address_byte;
#endif
#if defined(UX_HOST_STANDALONE)
UX_DEVICE               *enum_next;
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_RESOURCE_FREE, device, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

#if defined(UX_HOST_STANDALONE)

    /* Free possible allocated enumeration resources.  */
    if (device -> ux_device_flags & UX_DEVICE_FLAG_ENUM)
    {

        /* If transfer buffer is not freed, free it.  */
        if (device -> ux_device_enum_trans &&
            device -> ux_device_enum_trans -> ux_transfer_request_data_pointer)
        {
            _ux_utility_memory_free(device -> ux_device_enum_trans ->
                                            ux_transfer_request_data_pointer);
        }

        /* If configuration is not attached, free it.  */
        if ((device -> ux_device_enum_state == UX_HOST_STACK_ENUM_CONFIG_DESCR_PARSE) ||
            ((device -> ux_device_enum_state == UX_HOST_STACK_ENUM_TRANS_WAIT) &&
            (device -> ux_device_enum_next_state == UX_HOST_STACK_ENUM_CONFIG_DESCR_PARSE)))
        {
            if (device -> ux_device_enum_inst.configuration &&
                device -> ux_device_enum_inst.configuration ->
                                            ux_configuration_device == UX_NULL)
            {
                _ux_utility_memory_free(device -> ux_device_enum_inst.ptr);
            }
        }
    }

    /* Reset device flags.  */
    device -> ux_device_flags = 0;

#endif

    /* Set the alternate setting to zero.  */
    current_alternate_setting = 0;

    /* Get the first configuration registered to the device.  */
    configuration =  device -> ux_device_first_configuration;

    /* Parse all the configurations, remove all resources for the possible configuration.  */
    while (configuration != UX_NULL)
    {
        
        /* We have the correct configuration, search the interface(s).  */
        interface_ptr =  configuration -> ux_configuration_first_interface;

        /* Parse all the interfaces.  */
        while (interface_ptr != UX_NULL)
        {

            /* The alternate setting 0 has the selected alternate setting value.  */
            if (interface_ptr -> ux_interface_descriptor.bAlternateSetting == 0)
                current_alternate_setting = interface_ptr -> ux_interface_current_alternate_setting;

            /* If this is the selected interface, we need to free all the endpoints 
            attached to the alternate setting for this interface.  */
            endpoint =  interface_ptr -> ux_interface_first_endpoint;
            
            /* Parse all the endpoints.  */
            while (endpoint != UX_NULL)
            {

                /* Check if this is the selected interface.  */
                if (interface_ptr -> ux_interface_descriptor.bAlternateSetting == current_alternate_setting)
                {

                    /* Delete the endpoint instance first.  */
                    _ux_host_stack_endpoint_instance_delete(endpoint);
                }

                /* Memorize the endpoint container address.  */
                container =  (VOID *) endpoint;                  
                
                /* Get the next endpoint.  */      
                endpoint =  endpoint -> ux_endpoint_next_endpoint;
                
                /* Delete the endpoint container.  */
                _ux_utility_memory_free(container);
            }
            
            
            /* Memorize the interface container address.  */
            container =  (VOID *) interface_ptr;                  
                
            /* Get the next interface.  */      
            interface_ptr =  interface_ptr -> ux_interface_next_interface;

            /* Delete the interface container.  */
            _ux_utility_memory_free(container);
        }

        /* Memorize this configuration address before we free it.  */
        container =  (VOID *) configuration;

        /* Move to the next configuration in the list.  */
        configuration =  configuration -> ux_configuration_next_configuration;                                

        /* Free the configuration.  */
        _ux_utility_memory_free(container);
    }                       

    /* If there was a copy of packed descriptor, free it.  */
    if (device -> ux_device_packed_configuration)
    {
        _ux_utility_memory_free(device -> ux_device_packed_configuration);

        /* Pointer and keep count is set NULL later while reseting instance memory.  */
    }

    /* We need the HCD address for the control endpoint removal and to free
       the device address.  */
    hcd = UX_DEVICE_HCD_GET(device);

    /* Was the control endpoint already created ? */
    if (device -> ux_device_control_endpoint.ux_endpoint_state != 0)
    {

        /* There may be pending transactions on the control endpoint. They need to be aborted.  */
        _ux_host_stack_endpoint_transfer_abort(&device -> ux_device_control_endpoint);
    
        /* The enumeration thread needs to sleep a while to allow the application or the class that may be using
            the control endpoint to exit properly.  */
        _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 
    
        /* The control endpoint should be destroyed at the HCD level.  */
        hcd -> ux_hcd_entry_function(hcd, UX_HCD_DESTROY_ENDPOINT, (VOID *) &device -> ux_device_control_endpoint);
    }

    /* The semaphore attached to the control endpoint must be destroyed.  */
    _ux_host_semaphore_delete(&device -> ux_device_control_endpoint.ux_endpoint_transfer_request.ux_transfer_request_semaphore);

#if UX_MAX_DEVICES > 1
    /* Check if the device had an assigned address.  */
    if (device -> ux_device_address != 0)    
    {

        /* The USB address of this device can now be returned to the pool
           We need the HCD pointer for this operation.  */

        /* Calculate in which byte index the device address belongs.  */
        device_address_byte_index =  (UINT) (device -> ux_device_address-1)/8;        

        /* Now calculate the amount left in the byte index in bit.  */
        device_address_bit_index =  (UINT) (device -> ux_device_address-1)%8;     

        /* Build the mask for the address.  */
        device_address_byte =  (UCHAR)(1 << device_address_bit_index);

        /* Free the address.  */
        hcd -> ux_hcd_address[device_address_byte_index] &=  (UCHAR)~device_address_byte;
    }
#endif

    /* The semaphore for endpoint 0 protection must be destroyed.  */
    _ux_host_semaphore_delete(&device -> ux_device_protection_semaphore);

    /* Now this device can be free and its container return to the pool.  */
#if defined(UX_HOST_STANDALONE)
    enum_next = device -> ux_device_enum_next;
    _ux_utility_memory_set(device, 0, sizeof(UX_DEVICE)); /* Use case of memset is verified. */
    device -> ux_device_enum_next = enum_next;
#else

    _ux_utility_memory_set(device, 0, sizeof(UX_DEVICE)); /* Use case of memset is verified. */
#endif

    /* Mark the device handle as unused.  */
    device -> ux_device_handle =  UX_UNUSED;

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

