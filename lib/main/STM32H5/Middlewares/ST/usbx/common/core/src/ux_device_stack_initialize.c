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

UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_MULC_ULONG(sizeof(UX_SLAVE_CLASS), UX_MAX_SLAVE_CLASS_DRIVER), UX_MAX_SLAVE_CLASS_DRIVER_mul_ovf)

/* Define the names of all the USB Classes of USBX.  */

UCHAR _ux_system_slave_class_storage_name[] =                               "ux_slave_class_storage";
UCHAR _ux_system_slave_class_cdc_acm_name[] =                               "ux_slave_class_cdc_acm";
UCHAR _ux_system_slave_class_dpump_name[] =                                 "ux_slave_class_dpump";
UCHAR _ux_system_slave_class_pima_name[] =                                  "ux_slave_class_pima";
UCHAR _ux_system_slave_class_hid_name[] =                                   "ux_slave_class_hid";
UCHAR _ux_system_slave_class_rndis_name[] =                                 "ux_slave_class_rndis";
UCHAR _ux_system_slave_class_cdc_ecm_name[] =                               "ux_slave_class_cdc_ecm";
UCHAR _ux_system_slave_class_dfu_name[] =                                   "ux_slave_class_dfu";
UCHAR _ux_system_slave_class_audio_name[] =                                 "ux_slave_class_audio";

UCHAR _ux_system_device_class_printer_name[] =                              "ux_device_class_printer";
UCHAR _ux_system_device_class_ccid_name[] =                                 "ux_device_class_ccid";
UCHAR _ux_system_device_class_video_name[] =                                "ux_device_class_video";

/* Define USBX Host variable.  */
UX_SYSTEM_SLAVE *_ux_system_slave;

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_stack_initialize                         PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the generic portion of the device side of */
/*    USBX.                                                               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    device_framework_high_speed           Pointer to high speed FW      */ 
/*    device_framework_length_high_speed    Length of high speed FW       */ 
/*    device_framework_full_speed           Pointer to full speed FW      */ 
/*    device_framework_length_full_speed    Length of full speed FW       */ 
/*    string_framework                      Pointer to string FW          */ 
/*    string_framework_length               Length of string FW           */ 
/*    language_id_framework                 Pointer to language ID FW     */ 
/*    language_id_framework_length          Length of language ID FW      */ 
/*    (ux_system_slave_change_function)     Pointer to callback function  */ 
/*                                            for device changes          */ 
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate memory               */ 
/*    _ux_utility_memory_free               Free memory                   */ 
/*    _ux_utility_semaphore_create          Create semaphore              */
/*    _ux_utility_semaphore_delete          Delete semaphore              */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            added printer support,      */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added CCID support,         */
/*                                            added video support,        */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_initialize(UCHAR * device_framework_high_speed, ULONG device_framework_length_high_speed,
                                  UCHAR * device_framework_full_speed, ULONG device_framework_length_full_speed,
                                  UCHAR * string_framework, ULONG string_framework_length,
                                  UCHAR * language_id_framework, ULONG language_id_framework_length,
                                  UINT (*ux_system_slave_change_function)(ULONG))
{
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoints_pool;
UX_SLAVE_INTERFACE              *interfaces_pool;
UX_SLAVE_TRANSFER               *transfer_request;
UINT                            status;
ULONG                           interfaces_found;
ULONG                           endpoints_found;
#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE)
ULONG                           max_interface_number;
ULONG                           local_interfaces_found;
ULONG                           local_endpoints_found;
ULONG                           endpoints_in_interface_found;
UCHAR                           *device_framework;
ULONG                           device_framework_length;
UCHAR                           descriptor_type;
ULONG                           descriptor_length;
#endif
UCHAR                           *memory;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_INITIALIZE, 0, 0, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the device. */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Store the high speed device framework address and length in the project structure.  */
    _ux_system_slave -> ux_system_slave_device_framework_high_speed =             device_framework_high_speed;
    _ux_system_slave -> ux_system_slave_device_framework_length_high_speed =      device_framework_length_high_speed;

    /* Store the string framework address and length in the project structure.  */
    _ux_system_slave -> ux_system_slave_device_framework_full_speed =             device_framework_full_speed;
    _ux_system_slave -> ux_system_slave_device_framework_length_full_speed =      device_framework_length_full_speed;

    /* Store the string framework address and length in the project structure.  */
    _ux_system_slave -> ux_system_slave_string_framework =                         string_framework;
    _ux_system_slave -> ux_system_slave_string_framework_length =                  string_framework_length;

    /* Store the language ID list in the project structure.  */
    _ux_system_slave -> ux_system_slave_language_id_framework =                 language_id_framework;
    _ux_system_slave -> ux_system_slave_language_id_framework_length =          language_id_framework_length;

    /* Store the max number of slave class drivers in the project structure.  */
    UX_SYSTEM_DEVICE_MAX_CLASS_SET(UX_MAX_SLAVE_CLASS_DRIVER);
    
    /* Store the device state change function callback.  */
    _ux_system_slave -> ux_system_slave_change_function =  ux_system_slave_change_function;

    /* Allocate memory for the classes.
     * sizeof(UX_SLAVE_CLASS) * UX_MAX_SLAVE_CLASS_DRIVER) overflow is checked
     * outside of the function.
     */
    memory =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_SLAVE_CLASS) * UX_MAX_SLAVE_CLASS_DRIVER);
    if (memory == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);
    
    /* Save this memory allocation in the USBX project.  */
    _ux_system_slave -> ux_system_slave_class_array =  (UX_SLAVE_CLASS *) ((void *) memory);

    /* Allocate some memory for the Control Endpoint.  First get the address of the transfer request for the 
       control endpoint. */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Acquire a buffer for the size of the endpoint.  */
    transfer_request -> ux_slave_transfer_request_data_pointer =
          _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH);

    /* Ensure we have enough memory.  */
    if (transfer_request -> ux_slave_transfer_request_data_pointer == UX_NULL)
        status = UX_MEMORY_INSUFFICIENT;
    else
        status = UX_SUCCESS;

#if defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE)

    /* No scan, just assign predefined value.  */
    interfaces_found = UX_MAX_SLAVE_INTERFACES;
    endpoints_found = UX_MAX_DEVICE_ENDPOINTS;
#else

    /* Reset all values we are using during the scanning of the framework.  */
    interfaces_found                   =  0;
    endpoints_found                    =  0;
    max_interface_number               =  0;

    /* Go on to scan interfaces if no error.  */
    if (status == UX_SUCCESS)
    {

        /* We need to determine the maximum number of interfaces and endpoints declared in the device framework.  
        This mechanism requires that both framework behave the same way regarding the number of interfaces
        and endpoints.  */
        device_framework        =  _ux_system_slave -> ux_system_slave_device_framework_full_speed;
        device_framework_length =  _ux_system_slave -> ux_system_slave_device_framework_length_full_speed;

        /* Reset all values we are using during the scanning of the framework.  */
        local_interfaces_found             =  0;
        local_endpoints_found              =  0;
        endpoints_in_interface_found       =  0;

        /* Parse the device framework and locate interfaces and endpoint descriptor(s).  */
        while (device_framework_length != 0)
        {

            /* Get the length of this descriptor.  */
            descriptor_length =  (ULONG) *device_framework;
        
            /* And its type.  */
            descriptor_type =  *(device_framework + 1);
                    
            /* Check if this is an endpoint descriptor.  */
            switch(descriptor_type)
            {

            case UX_INTERFACE_DESCRIPTOR_ITEM:

                /* Check if this is alternate setting 0. If not, do not add another interface found.  
                If this is alternate setting 0, reset the endpoints count for this interface.  */
                if (*(device_framework + 3) == 0)
                {

                    /* Add the cumulated number of endpoints in the previous interface.  */
                    local_endpoints_found += endpoints_in_interface_found;

                    /* Read the number of endpoints for this alternate setting.  */
                    endpoints_in_interface_found = (ULONG) *(device_framework + 4);
                    
                    /* Increment the number of interfaces found in the current configuration.  */
                    local_interfaces_found++;
                }                
                else
                {

                    /* Compare the number of endpoints found in this non 0 alternate setting.  */
                    if (endpoints_in_interface_found < (ULONG) *(device_framework + 4))
                    
                        /* Adjust the number of maximum endpoints in this interface.  */
                        endpoints_in_interface_found = (ULONG) *(device_framework + 4);
                }

                /* Check and update max interface number.  */
                if (*(device_framework + 2) > max_interface_number)
                    max_interface_number = *(device_framework + 2);

                break;

            case UX_CONFIGURATION_DESCRIPTOR_ITEM:

                /* Check if the number of interfaces found in this configuration is the maximum so far. */
                if (local_interfaces_found > interfaces_found)
                    
                    /* We need to adjust the number of maximum interfaces.  */
                    interfaces_found =  local_interfaces_found;

                /* We have a new configuration. We need to reset the number of local interfaces. */
                local_interfaces_found =  0;

                /* Add the cumulated number of endpoints in the previous interface.  */
                local_endpoints_found += endpoints_in_interface_found;

                /* Check if the number of endpoints found in the previous configuration is the maximum so far. */
                if (local_endpoints_found > endpoints_found)
                    
                    /* We need to adjust the number of maximum endpoints.  */
                    endpoints_found =  local_endpoints_found;

                /* We have a new configuration. We need to reset the number of local endpoints. */
                local_endpoints_found         =  0;
                endpoints_in_interface_found  =  0;

                break;

            default:
                break;
            }

            /* Adjust what is left of the device framework.  */
            device_framework_length -=  descriptor_length;

            /* Point to the next descriptor.  */
            device_framework +=  descriptor_length;
        }
        
        /* Add the cumulated number of endpoints in the previous interface.  */
        local_endpoints_found += endpoints_in_interface_found;

        /* Check if the number of endpoints found in the previous interface is the maximum so far. */
        if (local_endpoints_found > endpoints_found)
                    
            /* We need to adjust the number of maximum endpoints.  */
            endpoints_found =  local_endpoints_found;


        /* Check if the number of interfaces found in this configuration is the maximum so far. */
        if (local_interfaces_found > interfaces_found)
            
            /* We need to adjust the number of maximum interfaces.  */
            interfaces_found =  local_interfaces_found;

        /* We do a sanity check on the finding. At least there must be one interface but endpoints are
        not necessary.  */
        if (interfaces_found == 0)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_INIT, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, device_framework, 0, 0, UX_TRACE_ERRORS, 0, 0)

            status = UX_DESCRIPTOR_CORRUPTED;
        }

        /* We do a sanity check on the finding. Max interface number should not exceed limit.  */
        if (status == UX_SUCCESS &&
            max_interface_number >= UX_MAX_SLAVE_INTERFACES)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_INIT, UX_MEMORY_INSUFFICIENT);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, device_framework, 0, 0, UX_TRACE_ERRORS, 0, 0)

            status = UX_MEMORY_INSUFFICIENT;
        }
    }
#endif

    /* Go on to allocate interfaces pool if no error.  */
    if (status == UX_SUCCESS)
    {

        /* Memorize both pool sizes.  */
        device -> ux_slave_device_interfaces_pool_number = interfaces_found;
        device -> ux_slave_device_endpoints_pool_number  = endpoints_found;

        /* We assign a pool for the interfaces.  */
        interfaces_pool =  _ux_utility_memory_allocate_mulc_safe(UX_NO_ALIGN, UX_REGULAR_MEMORY, interfaces_found, sizeof(UX_SLAVE_INTERFACE));
        if (interfaces_pool == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
        else

            /* Save the interface pool address in the device container.  */
            device -> ux_slave_device_interfaces_pool =  interfaces_pool;
    }

    /* Do we need an endpoint pool ?  */
    if (endpoints_found != 0 && status == UX_SUCCESS)
    {

        /* We assign a pool for the endpoints.  */
        endpoints_pool =  _ux_utility_memory_allocate_mulc_safe(UX_NO_ALIGN, UX_REGULAR_MEMORY, endpoints_found, sizeof(UX_SLAVE_ENDPOINT));
        if (endpoints_pool == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
        else
        {

            /* Save the endpoint pool address in the device container.  */
            device -> ux_slave_device_endpoints_pool =  endpoints_pool;

            /* We need to assign a transfer buffer to each endpoint. Each endpoint is assigned the
            maximum buffer size.  We also assign the semaphore used by the endpoint to synchronize transfer
            completion. */
            while (endpoints_pool < (device -> ux_slave_device_endpoints_pool + endpoints_found))
            {

                /* Obtain some memory.  */
                endpoints_pool -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer = 
                                _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, UX_SLAVE_REQUEST_DATA_MAX_LENGTH);

                /* Ensure we could allocate memory.  */
                if (endpoints_pool -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer == UX_NULL)
                {
                    status = UX_MEMORY_INSUFFICIENT;
                    break;
                }
        
                /* Create the semaphore for the endpoint.  */
                status =  _ux_device_semaphore_create(&endpoints_pool -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_semaphore,
                                                    "ux_transfer_request_semaphore", 0);

                /* Check completion status.  */
                if (status != UX_SUCCESS)
                {
                    status = UX_SEMAPHORE_ERROR;
                    break;
                }
        
                /* Next endpoint.  */
                endpoints_pool++;
            }
        }
    }
    else
        endpoints_pool = UX_NULL;

    /* Return successful completion.  */
    if (status == UX_SUCCESS)
        return(UX_SUCCESS);
    
    /* Free resources when there is error.  */

    /* Free device -> ux_slave_device_endpoints_pool.  */
    if (endpoints_pool)
    {

        /* In error cases creating endpoint resources, endpoints_pool is endpoint that failed.
         * Previously allocated things should be freed.  */
        while(endpoints_pool >= device -> ux_slave_device_endpoints_pool)
        {

            /* Delete ux_slave_transfer_request_semaphore.  */
            if (_ux_device_semaphore_created(&endpoints_pool -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_semaphore))
                _ux_device_semaphore_delete(&endpoints_pool -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_semaphore);

            /* Free ux_slave_transfer_request_data_pointer buffer.  */
            if (endpoints_pool -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer)
                _ux_utility_memory_free(endpoints_pool -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer);

            /* Move to previous endpoint.  */
            endpoints_pool --;
        }

        _ux_utility_memory_free(device -> ux_slave_device_endpoints_pool);
    }

    /* Free device -> ux_slave_device_interfaces_pool.  */
    if (device -> ux_slave_device_interfaces_pool)
        _ux_utility_memory_free(device -> ux_slave_device_interfaces_pool);

    /* Free device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer.  */
    if (device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer)
        _ux_utility_memory_free(device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer);

    /* Free _ux_system_slave -> ux_system_slave_class_array.  */
    _ux_utility_memory_free(_ux_system_slave -> ux_system_slave_class_array);

    /* Return completion status.  */
    return(status);
}

