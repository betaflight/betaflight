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
/*    _ux_host_stack_configuration_descriptor_parse       PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function reads the entire configuration descriptor and         */ 
/*    enumerates the interfaces, binds the interface to a class driver... */
/*    if the device has multiple configurations, we read all the          */ 
/*    configurations but do not instantiate any configuration. Rather we  */ 
/*    let a class driver do the work.                                     */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    device                                Pointer to device             */ 
/*    configuration                         Pointer to configuration      */ 
/*    configuration_index                   Index of configuration        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_interfaces_scan        Scan host interfaces          */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
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
UINT  _ux_host_stack_configuration_descriptor_parse(UX_DEVICE *device, UX_CONFIGURATION *configuration,
                                                                        UINT configuration_index)
{

UX_TRANSFER     *transfer_request;
UINT            status;
UCHAR           *descriptor;
UX_ENDPOINT     *control_endpoint;
ULONG           total_configuration_length;


    /* Retrieve the pointer to the control endpoint and its transfer_request.  */
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Retrieve the size of all the configuration descriptor.  */
    total_configuration_length =  configuration -> ux_configuration_descriptor.wTotalLength;

    /* Allocate enough memory to read all descriptors attached to this configuration.  */
    descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, total_configuration_length);

    /* Determine if the memory was allocated.  */
    if (descriptor == UX_NULL)
    {

        /* No, return an error.  */
        return(UX_MEMORY_INSUFFICIENT);
    }
    else
    {

        /* Create a transfer_request for the GET_DESCRIPTOR request.  */
        transfer_request -> ux_transfer_request_data_pointer =      descriptor;
        transfer_request -> ux_transfer_request_requested_length =  total_configuration_length;
        transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
        transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
        transfer_request -> ux_transfer_request_value =             configuration_index | (UINT)(UX_CONFIGURATION_DESCRIPTOR_ITEM << 8);
        transfer_request -> ux_transfer_request_index =             0;

        /* Send request to HCD layer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check for correct transfer and entire descriptor returned.  */
        if((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == total_configuration_length))
        {

            /* The entire descriptor now contains the configuration descriptor,
               the interface(s) descriptors, all alternate settings, endpoints
               and descriptor specific to the class. The descriptor is parsed for all interfaces.  */
            status =  _ux_host_stack_interfaces_scan(configuration, descriptor);
        }
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(descriptor);

    /* Return completion status.  */
    return(status);
}

