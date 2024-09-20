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
/*    _ux_host_stack_configuration_enumerate              PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function reads the configuration descriptor, creates the       */
/*    configuration container(s) for the device, and enumerates all found */
/*    configurations.                                                     */
/*                                                                        */
/*    At this stage, only the containers for each subcomponents are       */ 
/*    linked. No configuration, interface or endpoints are active unless  */ 
/*    a class issues a SET_CONFIGURATION.                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    device                                Pointer to device             */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_configuration_descriptor_parse                       */ 
/*                                          Parse configuration descriptor*/ 
/*    _ux_host_stack_configuration_instance_delete                        */ 
/*                                          Delete configuration instance */ 
/*    _ux_host_stack_new_configuration_create                             */ 
/*                                          Create new configuration      */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_descriptor_parse          Parse descriptor              */ 
/*    _ux_utility_memory_allocate           Allocate block of memory      */
/*    _ux_utility_memory_free               Free block of memory          */ 
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
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_configuration_enumerate(UX_DEVICE *device)
{

UX_TRANSFER         *transfer_request;
UINT                status =  UX_ERROR;
UCHAR *             descriptor;
UX_ENDPOINT         *control_endpoint;
UX_CONFIGURATION    *configuration;
ULONG               nb_configurations;
ULONG               configuration_index;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_CONFIGURATION_ENUMERATE, device, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Retrieve the pointer to the control endpoint and its transfer_request.  */
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the configuration descriptor the first time we read 
       only the configuration descriptor when we have the configuration descriptor, we have 
       the length of the entire configuration\interface\endpoint descriptors.  */
    descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_CONFIGURATION_DESCRIPTOR_LENGTH);
    if (descriptor == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* There maybe multiple configurations for this device.  */
    nb_configurations =  device -> ux_device_descriptor.bNumConfigurations;

    /* Parse all the configurations attached to the device. We start with the first index. 
       The index and the actual configuration value may be different according to the USB specification!  */
    for (configuration_index = 0; configuration_index < nb_configurations; configuration_index++)
    {

        /* Create a transfer_request for the GET_DESCRIPTOR request.  */
        transfer_request -> ux_transfer_request_data_pointer =      descriptor;
        transfer_request -> ux_transfer_request_requested_length =  UX_CONFIGURATION_DESCRIPTOR_LENGTH;
        transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
        transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
        transfer_request -> ux_transfer_request_value =             configuration_index | (UINT)(UX_CONFIGURATION_DESCRIPTOR_ITEM << 8);
        transfer_request -> ux_transfer_request_index =             0;

        /* Send request to HCD layer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check for correct transfer and entire descriptor returned.  */
        if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == UX_CONFIGURATION_DESCRIPTOR_LENGTH))
        {

            /* Allocate some memory for the container of this descriptor.  */
            configuration =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_CONFIGURATION));

            /* Check to see if the block was allocated.  */
            if (configuration != UX_NULL)
            {

                /* This configuration must be linked to the device.  */
                _ux_host_stack_new_configuration_create(device, configuration);
                
                /* The descriptor is in a packed format, parse it locally.  */      
                _ux_utility_descriptor_parse(descriptor, _ux_system_configuration_descriptor_structure,
                                    UX_CONFIGURATION_DESCRIPTOR_ENTRIES, (UCHAR *) &configuration -> ux_configuration_descriptor);

                /* Parse the device descriptor so that we can retrieve the length 
                    of the entire configuration.  */
                status =  _ux_host_stack_configuration_descriptor_parse(device, configuration, configuration_index);

                /* Check the completion status.  */
                if (status != UX_SUCCESS)
                {
                    /* Error, delete the configuration instance.  */
                    _ux_host_stack_configuration_instance_delete(configuration);
                }
            }
            else
            {

                /* Cannot allocate configuration memory. Abort enumeration */
                status =  UX_MEMORY_INSUFFICIENT;

                break;
            }            
        }
        else
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* The device descriptor does not contain the right amount of data. Maybe corruption.  */
            status =  UX_DESCRIPTOR_CORRUPTED;

            break;
        }            
        
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(descriptor);

    /* Return completion status.  */
    return(status);             
}

